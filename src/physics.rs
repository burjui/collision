use core::f64;
use std::{
    iter::zip,
    time::{Duration, Instant},
};

use anyhow::Context;
use grid::{CellRecord, Grid};
use object::{ObjectPrototype, ObjectSoa};
use opencl3::kernel::{ExecuteKernel, Kernel};

use crate::{
    app_config::{CONFIG, DtSource},
    fixed_vec::FixedVec,
    gpu::{Gpu, GpuBufferAccessMode, GpuDeviceBuffer},
    ring_buffer::RingBuffer,
    vector2::Vector2,
};

pub mod grid;
mod leapfrog_yoshida;
pub mod object;

pub struct PhysicsEngine {
    pub enable_constraint_bouncing: bool,
    objects: ObjectSoa,
    grid: Grid,
    time: f64,
    constraints: ConstraintBox,
    stats: Stats,
    restitution_coefficient: f64,
    global_gravity: Vector2<f64>,
    gravitational_constant: f64,
    gpu: Gpu,
    integration_kernel: Kernel,
    integration_gpu_position_buffer: Option<GpuDeviceBuffer<Vector2<f64>>>,
    integration_gpu_velocity_buffer: Option<GpuDeviceBuffer<Vector2<f64>>>,
    integration_gpu_planet_mass_buffer: Option<GpuDeviceBuffer<f64>>,
    gpu_compute_options: GpuComputeOptions,
}

impl PhysicsEngine {
    pub fn new() -> anyhow::Result<Self> {
        let constraints = ConstraintBox::new(
            Vector2::new(0.0, 0.0),
            Vector2::new(f64::from(CONFIG.window.width), f64::from(CONFIG.window.height)),
        );
        let gpu = Gpu::default(10)?;
        let integration_program = gpu.build_program("src/leapfrog_yoshida.cl")?;
        let integration_kernel =
            Kernel::create(&integration_program, "leapfrog_yoshida").context("Failed to create kernel")?;
        Ok(Self {
            enable_constraint_bouncing: true,
            objects: ObjectSoa::default(),
            grid: Grid::default(),
            time: 0.0,
            constraints,
            stats: Stats::default(),
            restitution_coefficient: CONFIG.simulation.restitution_coefficient,
            global_gravity: Vector2::from(CONFIG.simulation.gravity),
            gravitational_constant: CONFIG.simulation.gravitational_constant,
            gpu,
            integration_kernel,
            integration_gpu_position_buffer: None,
            integration_gpu_velocity_buffer: None,
            integration_gpu_planet_mass_buffer: None,
            gpu_compute_options: GpuComputeOptions::default(),
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

    pub fn planet_count(&self) -> usize {
        self.objects.planet_count
    }

    pub fn stats(&self) -> &Stats {
        &self.stats
    }

    #[must_use]
    pub fn time(&self) -> f64 {
        self.time
    }

    #[must_use]
    pub fn constraints(&self) -> ConstraintBox {
        self.constraints
    }

    pub fn advance(&mut self, speed_factor: f64, gpu_compute_options: GpuComputeOptions) {
        if gpu_compute_options != self.gpu_compute_options {
            self.stats.integration_duration = DurationStat::default();
        }
        self.gpu_compute_options = gpu_compute_options;

        let start = Instant::now();
        let dt = match CONFIG.simulation.dt {
            DtSource::Auto => {
                // Using max_object_size instead of grid cell size to avoid an unnecessary grid update
                let (max_velocity_squared, min_object_size) =
                    zip(self.objects.velocities.iter(), self.objects.radii.iter()).fold(
                        (0.0_f64, f64::MAX),
                        |(max_velocity_squared, min_object_size), (velocity, radius)| {
                            (
                                max_velocity_squared.max(velocity.magnitude_squared()),
                                min_object_size.min(radius * 2.0),
                            )
                        },
                    );
                let velocity_factor = if min_object_size == f64::MAX {
                    1.0
                } else {
                    // Two times velocity because objects can collide head on
                    let current_velocity = max_velocity_squared.sqrt();
                    current_velocity * 2.0 / min_object_size
                };
                let max_gravity_squared = self.objects.positions.iter().enumerate().fold(
                    0.0_f64,
                    |max_gravity_squared, (object_index, &position)| {
                        let gravity_squared = Self::gravity_acceleration(
                            object_index,
                            position,
                            &self.objects.positions,
                            self.global_gravity,
                            self.gravitational_constant,
                            &self.objects.masses[self.objects.planet_range()],
                        )
                        .magnitude_squared();
                        max_gravity_squared.max(gravity_squared)
                    },
                );
                // Experimentally derived
                let gravity_factor =
                    max_gravity_squared.sqrt().sqrt().max(self.global_gravity.magnitude()) / min_object_size.sqrt();
                speed_factor / 2.0 * (1.0 / (velocity_factor + gravity_factor).max(1.0))
            }
            DtSource::Fixed(dt) => dt,
        };
        self.time += dt;
        self.update(dt, gpu_compute_options);

        self.stats.total_duration.update(start.elapsed());
        self.stats.sim_time = self.time;
        self.stats.object_count = self.objects.len();
    }

    fn update(&mut self, dt: f64, gpu_compute_options: GpuComputeOptions) {
        self.time += dt;

        let start = Instant::now();
        self.integrate(dt, gpu_compute_options);
        self.stats.integration_duration.update(start.elapsed());

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

    fn integrate(&mut self, dt: f64, gpu_compute_options: GpuComputeOptions) {
        if gpu_compute_options.integration {
            self.integrate_gpu(dt);
        } else {
            self.integrate_cpu(dt);
        }
    }

    fn integrate_cpu(&mut self, dt: f64) {
        for object_index in 0..self.objects.len() {
            let update = Self::integrate_object_cpu(
                object_index,
                &self.objects.positions,
                self.objects.velocities[object_index],
                dt,
                self.global_gravity,
                self.gravitational_constant,
                &self.objects.masses[self.objects.planet_range()],
            );
            self.objects.positions[object_index] = update.position;
            self.objects.velocities[object_index] = update.velocity;
        }
    }

    fn integrate_gpu(&mut self, dt: f64) {
        if self.integration_gpu_buffers_are_outdated() {
            self.allocate_gpu_integration_buffers();
        }
        self.update_integration_gpu_buffers();
        self.execute_integration_kernel(dt);
    }

    fn execute_integration_kernel(&mut self, dt: f64) {
        let position_buffer = self.integration_gpu_position_buffer.as_mut().unwrap();
        let velocity_buffer = self.integration_gpu_velocity_buffer.as_mut().unwrap();
        let planet_mass_buffer = self.integration_gpu_planet_mass_buffer.as_mut().unwrap();
        let mut kernel = ExecuteKernel::new(&self.integration_kernel);
        kernel.set_global_work_size(self.objects.len());
        unsafe {
            kernel.set_arg(position_buffer.buffer());
            kernel.set_arg(velocity_buffer.buffer());
            kernel.set_arg(&dt);
            kernel.set_arg(&self.global_gravity);
            kernel.set_arg(planet_mass_buffer.buffer());
            kernel.set_arg(&self.objects.planet_count);
            kernel.set_arg(&self.gravitational_constant);
        }
        self.gpu
            .enqueue_execute_kernel(&mut kernel)
            .context("Failed to execute kernel")
            .unwrap();
        self.gpu
            .enqueue_read_device_buffer(position_buffer, &mut self.objects.positions)
            .context("Failed to read position buffer")
            .unwrap();
        self.gpu
            .enqueue_read_device_buffer(velocity_buffer, &mut self.objects.velocities)
            .context("Failed to read velocity buffer")
            .unwrap();
        self.gpu.wait_for_queue_completeion().unwrap();
    }

    fn integration_gpu_buffers_are_outdated(&mut self) -> bool {
        self.integration_gpu_position_buffer
            .as_ref()
            .is_none_or(|b| b.len() != self.objects.len() * 2)
    }

    fn allocate_gpu_integration_buffers(&mut self) {
        self.integration_gpu_position_buffer.replace(
            self.gpu
                .create_device_buffer(self.objects.positions.len(), GpuBufferAccessMode::ReadWrite)
                .context("Failed to create position buffer")
                .unwrap(),
        );
        self.integration_gpu_velocity_buffer.replace(
            self.gpu
                .create_device_buffer(self.objects.velocities.len(), GpuBufferAccessMode::ReadWrite)
                .context("Failed to create velocity buffer")
                .unwrap(),
        );
        self.integration_gpu_planet_mass_buffer.replace(
            self.gpu
                .create_device_buffer(
                    self.objects.planet_count + 1, /* cannot create an empty buffer */
                    GpuBufferAccessMode::ReadOnly,
                )
                .context("Failed to create planet mass buffer")
                .unwrap(),
        );
    }

    fn update_integration_gpu_buffers(&mut self) {
        let position_buffer = self.integration_gpu_position_buffer.as_mut().unwrap();
        let velocity_buffer = self.integration_gpu_velocity_buffer.as_mut().unwrap();
        let planet_mass_buffer = self.integration_gpu_planet_mass_buffer.as_mut().unwrap();
        assert!(position_buffer.len() == self.objects.positions.len());
        assert!(velocity_buffer.len() == self.objects.positions.len());
        assert!(planet_mass_buffer.len() == self.objects.planet_count + 1);
        self.gpu
            .enqueue_write_device_buffer(position_buffer, &self.objects.positions)
            .context("Failed to write position buffer")
            .unwrap();
        self.gpu
            .enqueue_write_device_buffer(velocity_buffer, &self.objects.velocities)
            .context("Failed to write position buffer")
            .unwrap();
        self.gpu
            .enqueue_write_device_buffer(planet_mass_buffer, &self.objects.masses[self.objects.planet_range()])
            .unwrap();
    }

    fn integrate_object_cpu(
        object_index: usize,
        positions: &[Vector2<f64>],
        velocity: Vector2<f64>,
        dt: f64,
        global_gravity: Vector2<f64>,
        gravitational_constant: f64,
        planet_masses: &[f64],
    ) -> ObjectUpdate {
        use leapfrog_yoshida::{C1, C2, C3, C4, D1, D2, D3};
        let x0 = positions[object_index];
        let v0 = velocity;
        let x1 = x0 + v0 * (C1 * dt);
        let a1 = Self::gravity_acceleration(
            object_index,
            x1,
            positions,
            global_gravity,
            gravitational_constant,
            planet_masses,
        );
        let v1 = v0 + a1 * (D1 * dt);
        let x2 = x0 + v1 * (C2 * dt);
        let a2 = Self::gravity_acceleration(
            object_index,
            x2,
            positions,
            global_gravity,
            gravitational_constant,
            planet_masses,
        );
        let v2 = v0 + a2 * (D2 * dt);
        let x3 = x0 + v2 * (C3 * dt);
        let a3 = Self::gravity_acceleration(
            object_index,
            x3,
            positions,
            global_gravity,
            gravitational_constant,
            planet_masses,
        );
        let v3 = v0 + a3 * (D3 * dt);
        ObjectUpdate {
            position: x0 + v3 * (C4 * dt),
            velocity: v3,
        }
    }

    fn gravity_acceleration(
        object_index: usize,
        position: Vector2<f64>,
        positions: &[Vector2<f64>],
        global_gravity: Vector2<f64>,
        gravitational_constant: f64,
        planet_masses: &[f64],
    ) -> Vector2<f64> {
        let mut gravity = global_gravity;
        for planet_index in 0..planet_masses.len() {
            if planet_index != object_index {
                let to_planet = positions[planet_index] - position;
                let direction = to_planet.normalize();
                gravity +=
                    direction * (gravitational_constant * planet_masses[planet_index] / to_planet.magnitude_squared());
            }
        }
        gravity
    }

    fn process_collisions(&mut self) {
        for range in self.grid.cell_iter() {
            const AREA_CELL_OFFSETS: [(isize, isize); 5] = [
                // // Objects in the same cell are the closest
                (0, 0),
                // // Checking only these neighboring cells to avoid duplicate collisions
                (-1, 1),
                (0, 1),
                (1, 1),
                (1, 0),
            ];
            let cell_records = &self.grid.cell_records[range];
            let (x, y) = cell_records[0].cell_coords;
            let mut process_collisions = |object_index, ox, oy| {
                let x = x.wrapping_add_signed(ox);
                let y = y.wrapping_add_signed(oy);
                if x < self.grid.size().x && y < self.grid.size().y {
                    if let Some((start, end)) = self.grid.coords_to_cells[(x, y)] {
                        let candidate_area = self.grid.cell_records[start..end]
                            .iter()
                            .copied()
                            .collect::<FixedVec<_, 32>>();
                        Self::process_object_with_cell_collisions(
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
            };
            for &CellRecord { object_index, .. } in cell_records {
                for (ox, oy) in AREA_CELL_OFFSETS {
                    let (ox, oy) = (ox.wrapping_neg(), oy.wrapping_neg());
                    process_collisions(object_index, ox, oy);
                }
            }
            for &CellRecord { object_index, .. } in cell_records {
                for (ox, oy) in AREA_CELL_OFFSETS {
                    process_collisions(object_index, ox, oy);
                }
            }
        }
    }

    fn process_object_with_cell_collisions(
        object1_index: usize,
        candidate_area: &[CellRecord],
        restitution_coefficient: f64,
        positions: &mut [Vector2<f64>],
        velocities: &mut [Vector2<f64>],
        radii: &[f64],
        masses: &[f64],
        is_planet: &[bool],
    ) {
        for &CellRecord {
            object_index: object2_index,
            ..
        } in candidate_area
        {
            if object1_index != object2_index {
                Self::process_object_collision(
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

    fn process_object_collision(
        object1_index: usize,
        object2_index: usize,
        restitution_coefficient: f64,
        positions: &mut [Vector2<f64>],
        velocities: &mut [Vector2<f64>],
        radii: &[f64],
        masses: &[f64],
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
                let divisor = (total_mass * distance * distance).max(f64::EPSILON);
                let velocity_diff = velocity1 - velocity2;
                velocity1 -= from_1_to_2 * (2.0 * mass2 * velocity_diff.dot(&from_1_to_2) / divisor);
                velocity2 -= -from_1_to_2 * (2.0 * mass1 * (-velocity_diff).dot(&(-from_1_to_2)) / divisor);
            }
            let intersection_depth = collision_distance - distance;
            let momentum1 = mass1 * velocity1.magnitude();
            let momentum2 = mass2 * velocity2.magnitude();
            let total_momentum = momentum1 + momentum2;
            let position_adjustment_base =
                from_1_to_2.normalize() * (intersection_depth / total_momentum.max(f64::EPSILON));
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

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GpuComputeOptions {
    pub integration: bool,
}

#[derive(Clone, Copy)]
struct ObjectUpdate {
    position: Vector2<f64>,
    velocity: Vector2<f64>,
}

#[derive(Default, Clone, Copy)]
pub struct ConstraintBox {
    pub topleft: Vector2<f64>,
    pub bottomright: Vector2<f64>,
}

impl ConstraintBox {
    #[must_use]
    pub fn new(topleft: Vector2<f64>, bottomright: Vector2<f64>) -> Self {
        Self { topleft, bottomright }
    }
}

#[derive(Clone, Debug)]
pub struct DurationStat {
    pub current: Duration,
    pub lowest: Duration,
    pub highest: Duration,
    pub average: RingBuffer<10, Duration>,
}

impl DurationStat {
    pub fn update(&mut self, duration: Duration) {
        self.current = duration;
        self.lowest = self.lowest.min(duration);
        self.highest = self.highest.max(duration);
        self.average.push(duration);
    }
}

impl Default for DurationStat {
    fn default() -> Self {
        Self {
            current: Duration::ZERO,
            lowest: Duration::MAX,
            highest: Duration::ZERO,
            average: RingBuffer::default(),
        }
    }
}

#[derive(Default, Clone, Debug)]
pub struct Stats {
    pub sim_time: f64,
    pub object_count: usize,
    pub integration_duration: DurationStat,
    pub grid_duration: DurationStat,
    pub collisions_duration: DurationStat,
    pub constraints_duration: DurationStat,
    pub total_duration: DurationStat,
}
