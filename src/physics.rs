use core::{f64, fmt};
use std::{
    collections::HashSet,
    iter::zip,
    time::{Duration, Instant},
};

use anyhow::Context;
use bvh::Bvh;
use grid::{CellRecord, Grid};
use object::{ObjectPrototype, ObjectSoa};
use opencl3::kernel::{ExecuteKernel, Kernel};

use crate::{
    app_config::{CONFIG, DtSource},
    gpu::{GPU, GpuBufferAccessMode, GpuDeviceBuffer},
    ring_buffer::RingBuffer,
    vector2::Vector2,
};

pub mod bvh;
pub mod grid;
pub mod object;

pub struct PhysicsEngine {
    pub enable_constraint_bouncing: bool,
    pub broad_phase: BroadPhase,
    objects: ObjectSoa,
    grid: Grid,
    bvh: Bvh,
    collisions: HashSet<(usize, usize)>,
    time: f64,
    constraints: ConstraintBox,
    stats: Stats,
    restitution_coefficient: f64,
    global_gravity: Vector2<f64>,
    gravitational_constant: f64,
    integration_kernel: Kernel,
    integration_gpu_position_buffer: Option<GpuDeviceBuffer<Vector2<f64>>>,
    integration_gpu_velocity_buffer: Option<GpuDeviceBuffer<Vector2<f64>>>,
    integration_gpu_planet_mass_buffer: Option<GpuDeviceBuffer<f64>>,
    gpu_planet_masses: Vec<f64>,
    gpu_compute_options: GpuComputeOptions,
}

impl PhysicsEngine {
    pub fn new() -> anyhow::Result<Self> {
        let constraints = ConstraintBox::new(
            Vector2::new(0.0, 0.0),
            Vector2::new(f64::from(CONFIG.window.width), f64::from(CONFIG.window.height)),
        );
        let integration_program = GPU.build_program("src/leapfrog_yoshida.cl")?;
        let integration_kernel =
            Kernel::create(&integration_program, "leapfrog_yoshida").context("Failed to create kernel")?;
        Ok(Self {
            enable_constraint_bouncing: true,
            broad_phase: BroadPhase::Grid,
            objects: ObjectSoa::default(),
            grid: Grid::default(),
            bvh: Bvh::default(),
            collisions: HashSet::default(),
            time: 0.0,
            constraints,
            stats: Stats::default(),
            restitution_coefficient: CONFIG.simulation.restitution_coefficient,
            global_gravity: Vector2::from(CONFIG.simulation.global_gravity),
            gravitational_constant: CONFIG.simulation.gravitational_constant,
            integration_kernel,
            integration_gpu_position_buffer: None,
            integration_gpu_velocity_buffer: None,
            integration_gpu_planet_mass_buffer: None,
            gpu_planet_masses: Vec::default(),
            gpu_compute_options: GpuComputeOptions::default(),
        })
    }

    pub fn add(&mut self, object: ObjectPrototype) -> usize {
        self.objects.add(object)
    }

    #[must_use]
    pub fn objects(&self) -> &ObjectSoa {
        &self.objects
    }

    pub fn objects_mut(&mut self) -> &mut ObjectSoa {
        &mut self.objects
    }

    #[must_use]
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
        if gpu_compute_options.integration != self.gpu_compute_options.integration {
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

        if matches!(self.broad_phase, BroadPhase::Grid) {
            let start = Instant::now();
            self.grid.update(&self.objects);
            self.stats.grid_duration.update(start.elapsed());
        } else {
            self.stats.grid_duration.update(Duration::ZERO);
        }

        if matches!(self.broad_phase, BroadPhase::Bvh) {
            let start = Instant::now();
            self.bvh = Bvh::new(&self.objects.positions, &self.objects.radii);
            let collisions_start = Instant::now();
            self.collisions.clear();
            for object_index in 0..self.objects.len() {
                self.bvh.find_intersections(object_index, &mut self.collisions);
            }
            println!(
                "BVH found {} collisions in {:?}",
                self.collisions.len(),
                collisions_start.elapsed()
            );
            self.stats.bvh_duration.update(start.elapsed());
        } else {
            self.stats.bvh_duration.update(Duration::ZERO);
        }

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
        #[allow(clippy::unreadable_literal)]
        const CBRT2: f64 = 1.2599210498948732;
        const W0: f64 = -CBRT2 / (2.0 - CBRT2);
        const W1: f64 = 1.0 / (2.0 - CBRT2);
        const C1: f64 = 0.5 * W1;
        const C4: f64 = C1;
        const C2: f64 = 0.5 * (W0 + W1);
        const C3: f64 = C2;
        const D1: f64 = W1;
        const D3: f64 = D1;
        const D2: f64 = W0;

        let c1dt = C1 * dt;
        let c2dt = C2 * dt;
        let c3dt = C3 * dt;
        let c4dt = C4 * dt;
        let d1dt = D1 * dt;
        let d2dt = D2 * dt;
        let d3dt = D3 * dt;
        for object_index in 0..self.objects.len() {
            let x0 = self.objects.positions[object_index];
            let v0 = self.objects.velocities[object_index];
            let x1 = x0 + v0 * c1dt;
            let a1 = Self::gravity_acceleration(
                object_index,
                x1,
                &self.objects.positions,
                self.global_gravity,
                self.gravitational_constant,
                &self.objects.masses[self.objects.planet_range()],
            );
            let v1 = v0 + a1 * d1dt;
            let x2 = x0 + v1 * c2dt;
            let a2 = Self::gravity_acceleration(
                object_index,
                x2,
                &self.objects.positions,
                self.global_gravity,
                self.gravitational_constant,
                &self.objects.masses[self.objects.planet_range()],
            );
            let v2 = v0 + a2 * d2dt;
            let x3 = x0 + v2 * c3dt;
            let a3 = Self::gravity_acceleration(
                object_index,
                x3,
                &self.objects.positions,
                self.global_gravity,
                self.gravitational_constant,
                &self.objects.masses[self.objects.planet_range()],
            );
            let v3 = v0 + a3 * d3dt;
            self.objects.positions[object_index] = x0 + v3 * c4dt;
            self.objects.velocities[object_index] = v3;
        }
    }

    fn integrate_gpu(&mut self, dt: f64) {
        if self.integration_gpu_buffers_are_outdated() {
            self.allocate_gpu_integration_buffers();
        }
        self.execute_integration_kernel(dt);
    }

    fn integration_gpu_buffers_are_outdated(&mut self) -> bool {
        self.integration_gpu_position_buffer
            .as_ref()
            .is_none_or(|b| b.len() != self.objects.len())
    }

    fn allocate_gpu_integration_buffers(&mut self) {
        self.integration_gpu_position_buffer.replace(
            GPU.create_device_buffer(self.objects.positions.len(), GpuBufferAccessMode::ReadWrite)
                .context("Failed to create position buffer")
                .unwrap(),
        );
        self.integration_gpu_velocity_buffer.replace(
            GPU.create_device_buffer(self.objects.velocities.len(), GpuBufferAccessMode::ReadWrite)
                .context("Failed to create velocity buffer")
                .unwrap(),
        );
        self.integration_gpu_planet_mass_buffer.replace(
            GPU.create_device_buffer(
                self.objects.planet_count + 1, /* cannot create an empty buffer */
                GpuBufferAccessMode::ReadOnly,
            )
            .context("Failed to create planet mass buffer")
            .unwrap(),
        );
        self.gpu_planet_masses.resize(self.objects.planet_count + 1, 0.0);
    }

    fn execute_integration_kernel(&mut self, dt: f64) {
        self.write_integration_gpu_buffers();
        let position_buffer = self.integration_gpu_position_buffer.as_mut().unwrap();
        let velocity_buffer = self.integration_gpu_velocity_buffer.as_mut().unwrap();
        let planet_mass_buffer = self.integration_gpu_planet_mass_buffer.as_mut().unwrap();
        let mut kernel = ExecuteKernel::new(&self.integration_kernel);
        kernel.set_global_work_size(self.objects.len());
        unsafe {
            kernel.set_arg(position_buffer.buffer());
            kernel.set_arg(velocity_buffer.buffer());
            kernel.set_arg(&u32::try_from(self.objects.len()).unwrap());
            kernel.set_arg(&dt);
            kernel.set_arg(&self.global_gravity);
            kernel.set_arg(planet_mass_buffer.buffer());
            kernel.set_arg(&u32::try_from(self.objects.planet_count).unwrap());
            kernel.set_arg(&self.gravitational_constant);
        }
        GPU.enqueue_execute_kernel(&mut kernel)
            .context("Failed to execute kernel")
            .unwrap();
        GPU.enqueue_read_device_buffer(position_buffer, &mut self.objects.positions)
            .context("Failed to read position buffer")
            .unwrap();
        GPU.enqueue_read_device_buffer(velocity_buffer, &mut self.objects.velocities)
            .context("Failed to read velocity buffer")
            .unwrap();
        GPU.wait_for_queue_completion().unwrap();
    }

    fn write_integration_gpu_buffers(&mut self) {
        let position_buffer = self.integration_gpu_position_buffer.as_mut().unwrap();
        let velocity_buffer = self.integration_gpu_velocity_buffer.as_mut().unwrap();
        let planet_mass_buffer = self.integration_gpu_planet_mass_buffer.as_mut().unwrap();
        assert!(position_buffer.len() == self.objects.positions.len());
        assert!(velocity_buffer.len() == self.objects.velocities.len());
        assert!(planet_mass_buffer.len() == self.objects.planet_count + 1);
        assert!(self.gpu_planet_masses.len() == self.objects.planet_count + 1);
        GPU.enqueue_write_device_buffer(position_buffer, &self.objects.positions)
            .context("Failed to write position buffer")
            .unwrap();
        GPU.enqueue_write_device_buffer(velocity_buffer, &self.objects.velocities)
            .context("Failed to write velocity buffer")
            .unwrap();
        self.gpu_planet_masses[..self.objects.planet_count]
            .copy_from_slice(&self.objects.masses[self.objects.planet_range()]);
        GPU.enqueue_write_device_buffer(planet_mass_buffer, &self.gpu_planet_masses)
            .context("Failed to write planet mass buffer")
            .unwrap();
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
        match self.broad_phase {
            BroadPhase::Grid => self.process_collisions_grid(),
            BroadPhase::Bvh => self.process_collisions_bvh(),
        }
    }

    fn process_collisions_grid(&mut self) {
        const AREA_CELL_OFFSETS: [(isize, isize); 5] = [
            // // Objects in the same cell are the closest
            (0, 0),
            // // Checking only these neighboring cells to avoid duplicate collisions
            (0, 1),
            (1, 0),
            (-1, 1),
            (1, 1),
        ];
        for range in self.grid.cell_iter() {
            for &CellRecord {
                object_index: object1_index,
                cell_coords: (x, y),
                ..
            } in &self.grid.cell_records[range]
            {
                // println!("cell ({x}, {y})");
                for (ox, oy) in AREA_CELL_OFFSETS {
                    let x = x.checked_add_signed(ox);
                    let y = y.checked_add_signed(oy);
                    if let (Some(x), Some(y)) = (x, y) {
                        if x < self.grid.size().x && y < self.grid.size().y {
                            if let Some((start, end)) = self.grid.coords_to_cells[(x, y)] {
                                for &CellRecord {
                                    object_index: object2_index,
                                    ..
                                } in &self.grid.cell_records[start..end]
                                {
                                    if object1_index != object2_index {
                                        let collision_distance =
                                            self.objects.radii[object1_index] + self.objects.radii[object2_index];
                                        let from_1_to_2 = self.objects.positions[object1_index]
                                            - self.objects.positions[object2_index];
                                        let distance_squared = from_1_to_2.magnitude_squared();
                                        if distance_squared <= collision_distance * collision_distance {
                                            let collision_pair = CollisionPair {
                                                object1_index,
                                                object2_index,
                                                distance_squared,
                                                collision_distance,
                                            };
                                            Self::process_object_collision(
                                                collision_pair,
                                                self.restitution_coefficient,
                                                &mut self.objects.positions,
                                                &mut self.objects.velocities,
                                                &self.objects.masses,
                                                &self.objects.is_planet,
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    fn process_collisions_bvh(&mut self) {
        for &(object1_index, object2_index) in &self.collisions {
            let object1_position = self.objects.positions[object1_index];
            let object2_position = self.objects.positions[object2_index];
            let object1_radius = self.objects.radii[object1_index];
            let object2_radius = self.objects.radii[object2_index];
            let distance_squared = (object1_position - object2_position).magnitude_squared();
            let collision_distance = object1_radius + object2_radius;
            if distance_squared < collision_distance * collision_distance {
                Self::process_object_collision(
                    CollisionPair {
                        object1_index,
                        object2_index,
                        distance_squared,
                        collision_distance,
                    },
                    self.restitution_coefficient,
                    &mut self.objects.positions,
                    &mut self.objects.velocities,
                    &self.objects.masses,
                    &self.objects.is_planet,
                );
            }
        }
    }

    fn process_object_collision(
        CollisionPair {
            object1_index,
            object2_index,
            distance_squared,
            collision_distance,
        }: CollisionPair,
        restitution_coefficient: f64,
        positions: &mut [Vector2<f64>],
        velocities: &mut [Vector2<f64>],
        masses: &[f64],
        is_planet: &[bool],
    ) {
        let from_1_to_2 = positions[object1_index] - positions[object2_index];
        let distance = distance_squared.sqrt();

        // Compute collision normal
        let normal = from_1_to_2 / distance;

        let mass1 = masses[object1_index];
        let mass2 = masses[object2_index];
        let total_mass = mass1 + mass2;

        // Save the initial velocities for simultaneous impulse calculation.
        let v1_initial = velocities[object1_index];
        let v2_initial = velocities[object2_index];

        // Compute the impulse scalar using the original velocities.
        let impulse_scalar = 2.0 * (v1_initial - v2_initial).dot(normal) / total_mass;

        // Update velocities using the impulse
        let new_v1 = v1_initial - normal * mass2 * impulse_scalar;
        let new_v2 = v2_initial + normal * mass1 * impulse_scalar;

        // Apply restitution coefficient if the objects aren't planets.
        let corrected_v1 = if !is_planet[object1_index] {
            new_v1 * restitution_coefficient
        } else {
            new_v1
        };
        let corrected_v2 = if !is_planet[object2_index] {
            new_v2 * restitution_coefficient
        } else {
            new_v2
        };

        velocities[object1_index] = corrected_v1;
        velocities[object2_index] = corrected_v2;

        // Correct positions based on penetration depth using inverse masses.
        let intersection_depth = collision_distance - distance;
        let inv_mass1 = 1.0 / mass1;
        let inv_mass2 = 1.0 / mass2;
        let total_inv_mass = inv_mass1 + inv_mass2;
        let correction = normal * intersection_depth;
        positions[object1_index] += correction * (inv_mass1 / total_inv_mass);
        positions[object2_index] -= correction * (inv_mass2 / total_inv_mass);
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

    pub fn grid(&self) -> &Grid {
        &self.grid
    }
}

#[derive(Clone, Copy)]
pub enum BroadPhase {
    Grid,
    Bvh,
}

impl fmt::Display for BroadPhase {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            BroadPhase::Grid => "grid",
            BroadPhase::Bvh => "bvh",
        })
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GpuComputeOptions {
    pub integration: bool,
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

#[derive(Clone, Copy)]
struct CollisionPair {
    object1_index: usize,
    object2_index: usize,
    distance_squared: f64,
    collision_distance: f64,
}

#[derive(Clone, Debug)]
pub struct DurationStat {
    pub current: Duration,
    pub lowest: Duration,
    pub highest: Duration,
    pub average: RingBuffer<32, Duration>,
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
    pub bvh_duration: DurationStat,
    pub collisions_duration: DurationStat,
    pub constraints_duration: DurationStat,
    pub total_duration: DurationStat,
}
