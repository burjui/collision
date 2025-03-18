use core::f32;
use std::{
    iter::zip,
    mem::transmute,
    time::{Duration, Instant},
};

use anyhow::Context as _;
use grid::{CellRecord, Grid};
use itertools::Itertools;
use object::{ObjectPrototype, ObjectSoa, ObjectUpdate};
use opencl3::kernel::{ExecuteKernel, Kernel};

use crate::{
    app_config::AppConfig,
    fixed_vec::FixedVec,
    gpu::{Gpu, GpuBufferAccess, GpuDeviceBuffer},
    vector2::Vector2,
};

pub mod grid;
mod leapfrog_yoshida;
pub mod object;

pub struct PhysicsEngine {
    pub enable_constraint_bouncing: bool,
    pub enable_gpu: bool,
    objects: ObjectSoa,
    grid: Grid,
    time: f32,
    constraints: ConstraintBox,
    stats: Stats,
    restitution_coefficient: f32,
    global_gravity: Vector2<f32>,
    gpu: Gpu,
    yoshida_kernel: Kernel,
    yoshida_kernel_no_planets: Kernel,
    yoshida_position_buffer: Option<GpuDeviceBuffer<Vector2<f32>>>,
    yoshida_velocity_buffer: Option<GpuDeviceBuffer<Vector2<f32>>>,
    yoshida_planet_mass_buffer: Option<GpuDeviceBuffer<f32>>,
}

impl PhysicsEngine {
    const GRAVITATIONAL_CONSTANT: f32 = 10000.0;

    pub fn new(config: &AppConfig) -> anyhow::Result<Self> {
        let constraints = ConstraintBox::new(
            Vector2::new(0.0, 0.0),
            Vector2::new(config.width as f32, config.height as f32),
        );
        let gpu = Gpu::default(10)?;
        let yoshida_program = gpu.build_program("src/yoshida.cl")?;
        let yoshida_kernel = Kernel::create(&yoshida_program, "yoshida").context("Failed to create kernel")?;
        let yoshida_kernel_no_planets =
            Kernel::create(&yoshida_program, "yoshida_no_planets").context("Failed to create kernel")?;
        Ok(Self {
            enable_constraint_bouncing: true,
            enable_gpu: true,
            objects: ObjectSoa::default(),
            grid: Grid::default(),
            time: 0.0,
            constraints,
            stats: Stats::default(),
            restitution_coefficient: config.restitution_coefficient,
            global_gravity: Vector2::from(config.gravity),
            gpu,
            yoshida_kernel,
            yoshida_kernel_no_planets,
            yoshida_position_buffer: None,
            yoshida_velocity_buffer: None,
            yoshida_planet_mass_buffer: None,
        })
    }

    pub fn add(&mut self, object: ObjectPrototype) -> usize {
        self.objects.add(object)
    }

    #[must_use]
    pub fn grid(&self) -> &Grid {
        &self.grid
    }

    pub fn grid_mut(&mut self) -> &mut Grid {
        &mut self.grid
    }

    #[must_use]
    pub fn objects(&self) -> &ObjectSoa {
        &self.objects
    }

    pub fn objects_mut(&mut self) -> &mut ObjectSoa {
        &mut self.objects
    }

    pub fn stats(&self) -> &Stats {
        &self.stats
    }

    pub fn advance(&mut self, dt: f32) {
        let start = Instant::now();

        // Using max_object_size instead of grid cell size to avoid an unnecessary grid update
        let (max_velocity_squared, max_object_size) = zip(self.objects.velocities.iter(), self.objects.radii.iter())
            .fold(
                (0.0, 0.0),
                |(mut max_velocity_squared, mut max_object_size), (velocity, radius)| {
                    let velocity_squared = velocity.magnitude_squared();
                    if velocity_squared > max_velocity_squared {
                        max_velocity_squared = velocity_squared;
                    }
                    let object_size = radius * 2.0;
                    if object_size > max_object_size {
                        max_object_size = object_size;
                    }
                    (max_velocity_squared, max_object_size)
                },
            );
        let slowdown_factor = if max_velocity_squared > 0.0 {
            (max_object_size / max_velocity_squared.sqrt() * 1000.0).min(1.0)
        } else {
            1.0
        };
        self.time += dt * slowdown_factor;
        self.update(dt * slowdown_factor);

        self.stats.total_duration.update(start.elapsed());
    }

    #[must_use]
    pub fn constraints(&self) -> &ConstraintBox {
        &self.constraints
    }

    #[must_use]
    pub fn time(&self) -> f32 {
        self.time
    }

    fn update(&mut self, dt: f32) {
        self.time += dt;

        let start = Instant::now();
        self.update_objects(dt);
        self.stats.updates_duration.update(start.elapsed());

        let start = Instant::now();
        self.grid.update(&self.objects);
        self.stats.grid_duration.update(start.elapsed());

        let start = Instant::now();
        self.process_collisions();
        self.stats.collisions_duration.update(start.elapsed());

        let start = Instant::now();
        self.apply_constraints();
        self.stats.constraints_duration.update(start.elapsed());
    }

    fn update_objects(&mut self, dt: f32) {
        if self.enable_gpu {
            self.update_objects_leapfrog_yoshida_gpu(dt);
        } else {
            for object_index in 0..self.objects.len() {
                let update = self.update_object_leapfrog_yoshida(
                    object_index,
                    self.objects.positions[object_index],
                    self.objects.velocities[object_index],
                    dt,
                );
                self.objects.positions[object_index] = update.position;
                self.objects.velocities[object_index] = update.velocity;
            }
        }
    }

    fn update_objects_leapfrog_yoshida_gpu(&mut self, dt: f32) {
        if self.yoshida_gpu_buffers_are_outdated() {
            self.allocate_yoshida_gpu_buffers();
        }
        self.update_yoshida_gpu_buffers();
        if self.objects.planet_count > 0 {
            self.execute_yoshida_gpu_kernel(dt);
        } else {
            self.execute_yoshida_no_planets_gpu_kernel(dt);
        }
    }

    fn execute_yoshida_gpu_kernel(&mut self, dt: f32) {
        let position_buffer = self.yoshida_position_buffer.as_mut().unwrap();
        let velocity_buffer = self.yoshida_velocity_buffer.as_mut().unwrap();
        let planet_mass_buffer = self.yoshida_planet_mass_buffer.as_mut().unwrap();
        let mut kernel = ExecuteKernel::new(&self.yoshida_kernel);
        kernel.set_global_work_size(self.objects.len());
        unsafe {
            kernel.set_arg(position_buffer.buffer());
            kernel.set_arg(velocity_buffer.buffer());
            kernel.set_arg(&dt);
            kernel.set_arg(&self.global_gravity);
            kernel.set_arg(planet_mass_buffer.buffer());
            kernel.set_arg(&self.objects.planet_count);
            kernel.set_arg(&Self::GRAVITATIONAL_CONSTANT);
        }
        self.gpu
            .enqueue_execute_kernel(&mut kernel)
            .context("Failed to execute kernel")
            .unwrap();
        self.gpu
            .enqueue_read_device_buffer(position_buffer, &mut self.objects.positions)
            .unwrap();
        self.gpu
            .enqueue_read_device_buffer(velocity_buffer, &mut self.objects.velocities)
            .unwrap();
        self.gpu.submit_queue().unwrap();
    }

    fn execute_yoshida_no_planets_gpu_kernel(&mut self, dt: f32) {
        let position_buffer = self.yoshida_position_buffer.as_mut().unwrap();
        let velocity_buffer = self.yoshida_velocity_buffer.as_mut().unwrap();
        let mut kernel = ExecuteKernel::new(&self.yoshida_kernel_no_planets);
        kernel.set_global_work_size(self.objects.len());
        unsafe {
            kernel.set_arg(position_buffer.buffer());
            kernel.set_arg(velocity_buffer.buffer());
            kernel.set_arg(&dt);
            kernel.set_arg(&self.global_gravity);
        }
        self.gpu
            .enqueue_execute_kernel(&mut kernel)
            .context("Failed to execute kernel")
            .unwrap();
        self.gpu
            .enqueue_read_device_buffer(position_buffer, unsafe {
                transmute(self.objects.positions.as_mut_slice())
            })
            .unwrap();
        self.gpu
            .enqueue_read_device_buffer(velocity_buffer, unsafe {
                transmute(self.objects.velocities.as_mut_slice())
            })
            .unwrap();
        self.gpu.submit_queue().unwrap();
    }

    fn yoshida_gpu_buffers_are_outdated(&mut self) -> bool {
        self.yoshida_position_buffer
            .as_ref()
            .is_none_or(|b| b.len() != self.objects.len() * 2)
    }

    fn allocate_yoshida_gpu_buffers(&mut self) {
        let position_buffer = self
            .gpu
            .create_device_buffer(self.objects.positions.len(), GpuBufferAccess::ReadWrite)
            .context("Failed to create position buffer")
            .unwrap();
        let velocity_buffer = self
            .gpu
            .create_device_buffer(self.objects.velocities.len(), GpuBufferAccess::ReadWrite)
            .context("Failed to create velocity buffer")
            .unwrap();
        self.yoshida_position_buffer.replace(position_buffer);
        self.yoshida_velocity_buffer.replace(velocity_buffer);

        if self.objects.planet_count > 0 {
            self.yoshida_planet_mass_buffer.replace(
                self.gpu
                    .create_device_buffer(self.objects.planet_count, GpuBufferAccess::ReadOnly)
                    .context("Failed to create planet mass buffer")
                    .unwrap(),
            );
        }
    }

    fn update_yoshida_gpu_buffers(&mut self) {
        self.gpu
            .enqueue_write_buffer(self.yoshida_position_buffer.as_mut().unwrap(), &self.objects.positions)
            .unwrap();
        self.gpu
            .enqueue_write_buffer(self.yoshida_velocity_buffer.as_mut().unwrap(), &self.objects.velocities)
            .unwrap();
        if self.objects.planet_count > 0 {
            self.gpu
                .enqueue_write_buffer(
                    self.yoshida_planet_mass_buffer.as_mut().unwrap(),
                    &self.objects.masses[..self.objects.planet_count],
                )
                .unwrap();
        }
    }

    fn update_object_leapfrog_yoshida(
        &self,
        object_index: usize,
        position: Vector2<f32>,
        velocity: Vector2<f32>,
        dt: f32,
    ) -> ObjectUpdate {
        use leapfrog_yoshida::{C1, C2, C3, C4, D1, D2, D3};
        let x0 = position;
        let v0 = velocity;
        let x1 = x0 + v0 * (C1 * dt);
        let a1 = self.gravity_acceleration(object_index, x1);
        let v1 = v0 + a1 * (D1 * dt);
        let x2 = x0 + v1 * (C2 * dt);
        let a2 = self.gravity_acceleration(object_index, x2);
        let v2 = v0 + a2 * (D2 * dt);
        let x3 = x0 + v2 * (C3 * dt);
        let a3 = self.gravity_acceleration(object_index, x3);
        let v3 = v0 + a3 * (D3 * dt);
        ObjectUpdate {
            position: x0 + v3 * (C4 * dt),
            velocity: v3,
        }
    }

    fn gravity_acceleration(&self, object_index: usize, position: Vector2<f32>) -> Vector2<f32> {
        let mut gravity = Vector2::default();
        for planet_index in 0..self.objects.planet_count {
            if planet_index != object_index {
                let to_planet = self.objects.positions[planet_index] - position;
                let direction = to_planet.normalize();
                gravity += direction
                    * (Self::GRAVITATIONAL_CONSTANT * self.objects.masses[planet_index]
                        / to_planet.magnitude_squared().max(f32::EPSILON));
            }
        }
        gravity
    }

    fn process_collisions(&mut self) {
        for range in self.grid.cell_iter() {
            let cell_records = &self.grid.cell_records[range];
            let (x, y) = cell_records[0].cell_coords;
            for &CellRecord { object_index, .. } in cell_records {
                let mut candidate_area = (x.saturating_sub(1)..=(x + 1).min(self.grid.size().x - 1))
                    .cartesian_product(y.saturating_sub(1)..=(y + 1).min(self.grid.size().y - 1))
                    .flat_map(|coords| {
                        self.grid.coords_to_cells[coords]
                            .into_iter()
                            .flat_map(|(start, end)| self.grid.cell_records[start..end].iter().copied())
                    })
                    .collect::<FixedVec<_, 32>>();
                let position = self.objects.positions[object_index];
                candidate_area.sort_by(|a, b| {
                    let a = self.objects.positions[a.object_index];
                    let b = self.objects.positions[b.object_index];
                    (a - position)
                        .magnitude_squared()
                        .partial_cmp(&(b - position).magnitude_squared())
                        .unwrap()
                });
                Self::process_object_with_area_collisions(
                    object_index,
                    &candidate_area,
                    self.restitution_coefficient,
                    &mut self.objects.positions,
                    &mut self.objects.velocities,
                    &self.objects.radii,
                    &self.objects.masses,
                    &self.objects.is_planet,
                );
            }
        }
    }

    fn process_object_with_area_collisions(
        object1_index: usize,
        candidate_area: &[CellRecord],
        restitution_coefficient: f32,
        positions: &mut [Vector2<f32>],
        velocities: &mut [Vector2<f32>],
        radii: &[f32],
        masses: &[f32],
        is_planet: &[bool],
    ) {
        for &CellRecord {
            object_index: object2_index,
            ..
        } in candidate_area
        {
            if object1_index != object2_index {
                process_object_collision(
                    object1_index,
                    object2_index,
                    restitution_coefficient,
                    positions,
                    velocities,
                    radii,
                    masses,
                    is_planet,
                );
            }
        }
    }

    fn apply_constraints(&mut self) {
        let cb = self.constraints;
        for ((position, velocity), radius) in zip(
            zip(&mut self.objects.positions, &mut self.objects.velocities),
            &self.objects.radii,
        ) {
            let initial_velocity = *velocity;
            if position.x - radius < cb.topleft.x {
                position.x = cb.topleft.x + radius;
                if self.enable_constraint_bouncing {
                    velocity.x *= -1.0;
                }
            } else if position.x + radius > cb.bottomright.x {
                position.x = cb.bottomright.x - radius;
                if self.enable_constraint_bouncing {
                    velocity.x *= -1.0;
                }
            }

            if position.y - radius < cb.topleft.y {
                position.y = cb.topleft.y + radius;
                if self.enable_constraint_bouncing {
                    velocity.y *= -1.0;
                }
            } else if position.y + radius > cb.bottomright.y {
                position.y = cb.bottomright.y - radius;
                if self.enable_constraint_bouncing {
                    velocity.y *= -1.0;
                }
            }

            if *velocity != initial_velocity {
                *velocity *= self.restitution_coefficient;
            }
        }
    }
}

#[derive(Clone, Copy)]
pub struct ConstraintBox {
    pub topleft: Vector2<f32>,
    pub bottomright: Vector2<f32>,
}

impl ConstraintBox {
    #[must_use]
    pub fn new(topleft: Vector2<f32>, bottomright: Vector2<f32>) -> Self {
        Self { topleft, bottomright }
    }
}

pub struct DurationStat {
    pub current: Duration,
    pub lowest: Duration,
    pub highest: Duration,
}

impl DurationStat {
    pub fn update(&mut self, duration: Duration) {
        self.current = duration;
        self.lowest = self.lowest.min(duration);
        self.highest = self.highest.max(duration);
    }
}

impl Default for DurationStat {
    fn default() -> Self {
        Self {
            current: Duration::ZERO,
            lowest: Duration::MAX,
            highest: Duration::ZERO,
        }
    }
}

#[derive(Default)]
pub struct Stats {
    pub updates_duration: DurationStat,
    pub grid_duration: DurationStat,
    pub collisions_duration: DurationStat,
    pub constraints_duration: DurationStat,
    pub total_duration: DurationStat,
}

fn process_object_collision(
    object1_index: usize,
    object2_index: usize,
    restitution_coefficient: f32,
    positions: &mut [Vector2<f32>],
    velocities: &mut [Vector2<f32>],
    radii: &[f32],
    masses: &[f32],
    is_planet: &[bool],
) {
    let collision_distance = radii[object1_index] + radii[object2_index];
    let from_1_to_2 = positions[object1_index] - positions[object2_index];
    if from_1_to_2.magnitude_squared() <= collision_distance * collision_distance {
        let mass1 = masses[object1_index];
        let mass2 = masses[object2_index];
        let total_mass = mass1 + mass2;
        let mut velocity1 = velocities[object1_index];
        let mut velocity2 = velocities[object2_index];
        let distance = from_1_to_2.magnitude();
        {
            let divisor = (total_mass * distance * distance).max(f32::EPSILON);
            let velocity_diff = velocity1 - velocity2;
            velocity1 -= from_1_to_2 * (2.0 * mass2 * velocity_diff.dot(&from_1_to_2) / divisor);
            velocity2 -= -from_1_to_2 * (2.0 * mass1 * (-velocity_diff).dot(&(-from_1_to_2)) / divisor);
        }
        let intersection_depth = collision_distance - distance;
        let momentum1 = mass1 * velocity1.magnitude();
        let momentum2 = mass2 * velocity2.magnitude();
        let total_momentum = momentum1 + momentum2;
        let position_adjustment_base =
            from_1_to_2.normalize() * (intersection_depth / total_momentum.max(f32::EPSILON));
        positions[object1_index] += position_adjustment_base * momentum2;
        positions[object2_index] -= position_adjustment_base * momentum1;

        if !is_planet[object1_index] {
            velocity1 *= restitution_coefficient;
        }
        if !is_planet[object2_index] {
            velocity2 *= restitution_coefficient;
        }

        velocities[object1_index] = velocity1;
        velocities[object2_index] = velocity2;
    }
}
