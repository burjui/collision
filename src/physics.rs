use core::f64;
use std::{
    iter::zip,
    time::{Duration, Instant},
};

use grid::{cell_at, CellRecord, Grid};
use object::{ObjectPrototype, ObjectSoa};

use crate::{
    app_config::{config, DtSource},
    vector2::Vector2,
};

pub mod grid;
mod leapfrog_yoshida;
pub mod object;

pub struct PhysicsEngine {
    pub enable_constraint_bouncing: bool,
    objects: ObjectSoa,
    object_updates: Vec<ObjectUpdate>,
    grid: Grid,
    time: f64,
    constraints: ConstraintBox,
    stats: Stats,
    // restitution_coefficient: f64,
    global_gravity: Vector2<f64>,
    gravitational_constant: f64,
    proximity_force_constant: f64,
    // gpu: Gpu,
    // yoshida_kernel: Kernel,
    // yoshida_kernel_no_planets: Kernel,
    // yoshida_position_buffer: Option<GpuDeviceBuffer<Vector2<f64>>>,
    // yoshida_velocity_buffer: Option<GpuDeviceBuffer<Vector2<f64>>>,
    // yoshida_planet_mass_buffer: Option<GpuDeviceBuffer<f64>>,
}

impl PhysicsEngine {
    pub fn new() -> anyhow::Result<Self> {
        let config = config();
        let constraints = ConstraintBox::new(
            Vector2::new(0.0, 0.0),
            Vector2::new(config.window.width as f64, config.window.height as f64),
        );
        // let gpu = Gpu::default(10)?;
        // let yoshida_program = gpu.build_program("src/yoshida.cl")?;
        // let yoshida_kernel = Kernel::create(&yoshida_program, "yoshida").context("Failed to create kernel")?;
        // let yoshida_kernel_no_planets =
        //     Kernel::create(&yoshida_program, "yoshida_no_planets").context("Failed to create kernel")?;
        Ok(Self {
            enable_constraint_bouncing: true,
            objects: ObjectSoa::default(),
            object_updates: Vec::default(),
            grid: Grid::default(),
            time: 0.0,
            constraints,
            stats: Stats::default(),
            // restitution_coefficient: config.simulation.restitution_coefficient,
            global_gravity: Vector2::from(config.simulation.gravity),
            gravitational_constant: config.simulation.gravitational_constant,
            proximity_force_constant: config.simulation.proximity_force_constant,
            // gpu,
            // yoshida_kernel,
            // yoshida_kernel_no_planets,
            // yoshida_position_buffer: None,
            // yoshida_velocity_buffer: None,
            // yoshida_planet_mass_buffer: None,
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
    pub fn constraints(&self) -> &ConstraintBox {
        &self.constraints
    }

    #[must_use]
    pub fn time(&self) -> f64 {
        self.time
    }

    pub fn advance(&mut self, speed_factor: f64) {
        let start = Instant::now();
        let dt = match config().simulation.dt {
            DtSource::Auto => {
                // Using max_object_size instead of grid cell size to avoid an unnecessary grid update
                let (max_velocity_squared, min_object_size) =
                    zip(self.objects.velocities.iter(), self.objects.radii.iter()).fold(
                        (0.0, f64::MAX),
                        |(mut max_velocity_squared, mut min_object_size), (velocity, radius)| {
                            let velocity_squared = velocity.magnitude_squared();
                            if velocity_squared > max_velocity_squared {
                                max_velocity_squared = velocity_squared;
                            }
                            let object_size = radius * 2.0;
                            if object_size < min_object_size {
                                min_object_size = object_size;
                            }
                            (max_velocity_squared, min_object_size)
                        },
                    );
                let velocity_factor = if min_object_size == f64::MAX {
                    1.0
                } else {
                    // Two times velocity because objects can collide head on
                    let current_velocity = max_velocity_squared.sqrt();
                    current_velocity * 2.0 / min_object_size
                };
                let max_force_acceleration = self.objects.positions.iter().enumerate().fold(
                    0.0,
                    |max_force_acceleration, (object_index, &position)| {
                        let force_acceleration = Self::force_acceleration(
                            object_index,
                            position,
                            &self.objects.positions,
                            &self.objects.radii,
                            self.global_gravity,
                            self.gravitational_constant,
                            self.proximity_force_constant,
                            &self.objects.masses[..self.objects.planet_count],
                            &self.grid,
                        )
                        .magnitude_squared();
                        if force_acceleration > max_force_acceleration {
                            force_acceleration
                        } else {
                            max_force_acceleration
                        }
                    },
                );
                // Experimentally derived
                let force_factor = max_force_acceleration.sqrt().sqrt() * 80.0 / min_object_size.sqrt();
                speed_factor / 2.0 * (1.0 / velocity_factor.max(force_factor).max(1.0))
            }
            DtSource::Fixed(dt) => dt,
        };
        self.time += dt;
        self.update(dt);

        self.stats.total_duration.update(start.elapsed());
        self.stats.sim_time = self.time;
        self.stats.object_count = self.objects.len();
    }

    fn update(&mut self, dt: f64) {
        self.time += dt;

        let start = Instant::now();
        self.grid.update(&self.objects);
        self.stats.grid_duration.update(start.elapsed());

        let start = Instant::now();
        if self.object_updates.len() != self.objects.len() {
            self.object_updates.resize(self.objects.len(), ObjectUpdate::default());
        }
        for object_index in 0..self.objects.positions.len() {
            self.object_updates[object_index] = Self::update_object_leapfrog_yoshida_cpu(
                object_index,
                dt,
                &mut self.objects.positions,
                &mut self.objects.velocities,
                &self.objects.radii,
                self.global_gravity,
                self.gravitational_constant,
                self.proximity_force_constant,
                &self.objects.masses[..self.objects.planet_count],
                &self.grid,
            );
        }
        for object_index in 0..self.objects.positions.len() {
            self.objects.positions[object_index] = self.object_updates[object_index].position;
            self.objects.velocities[object_index] = self.object_updates[object_index].velocity;
        }
        for object_index in 0..self.objects.positions.len() {
            Self::apply_constraints(
                object_index,
                &mut self.objects.positions,
                &mut self.objects.velocities,
                &self.objects.radii,
                &self.constraints,
            );
        }
        self.stats.updates_duration.update(start.elapsed());
    }

    fn update_object_leapfrog_yoshida_cpu(
        object_index: usize,
        dt: f64,
        positions: &mut [Vector2<f64>],
        velocities: &mut [Vector2<f64>],
        radii: &[f64],
        global_gravity: Vector2<f64>,
        gravitational_constant: f64,
        proximity_force_constant: f64,
        planet_masses: &[f64],
        grid: &Grid,
    ) -> ObjectUpdate {
        use leapfrog_yoshida::{C1, C2, C3, C4, D1, D2, D3};
        let x0 = positions[object_index];
        let v0 = velocities[object_index];
        let x1 = x0 + v0 * (C1 * dt);
        let a1 = Self::force_acceleration(
            object_index,
            x1,
            positions,
            radii,
            global_gravity,
            gravitational_constant,
            proximity_force_constant,
            planet_masses,
            grid,
        );
        let v1 = v0 + a1 * (D1 * dt);
        let x2 = x0 + v1 * (C2 * dt);
        let a2 = Self::force_acceleration(
            object_index,
            x2,
            positions,
            radii,
            global_gravity,
            gravitational_constant,
            proximity_force_constant,
            planet_masses,
            grid,
        );
        let v2 = v0 + a2 * (D2 * dt);
        let x3 = x0 + v2 * (C3 * dt);
        let a3 = Self::force_acceleration(
            object_index,
            x3,
            positions,
            radii,
            global_gravity,
            gravitational_constant,
            proximity_force_constant,
            planet_masses,
            grid,
        );
        let v3 = v0 + a3 * (D3 * dt);
        ObjectUpdate {
            position: x0 + v3 * (C4 * dt),
            velocity: v3,
        }
    }

    fn force_acceleration(
        object_index: usize,
        object_position: Vector2<f64>,
        positions: &[Vector2<f64>],
        radii: &[f64],
        global_gravity: Vector2<f64>,
        gravitational_constant: f64,
        proximity_force_constant: f64,
        planet_masses: &[f64],
        grid: &Grid,
    ) -> Vector2<f64> {
        global_gravity
            + Self::planetary_gravity_acceleration(
                object_index,
                object_position,
                positions,
                planet_masses,
                gravitational_constant,
            )
            + Self::proximity_force(
                object_index,
                object_position,
                positions,
                radii,
                grid,
                proximity_force_constant,
            )
    }

    fn planetary_gravity_acceleration(
        object_index: usize,
        object_position: Vector2<f64>,
        positions: &[Vector2<f64>],
        planet_masses: &[f64],
        gravitational_constant: f64,
    ) -> Vector2<f64> {
        let mut gravity = Vector2::default();
        for planet_index in 0..planet_masses.len() {
            if planet_index != object_index {
                let to_planet = positions[planet_index] - object_position;
                let direction = to_planet.normalize();
                gravity +=
                    direction * (gravitational_constant * planet_masses[planet_index] / to_planet.magnitude_squared());
            }
        }
        gravity
    }

    fn proximity_force(
        object_index: usize,
        object_position: Vector2<f64>,
        positions: &[Vector2<f64>],
        radii: &[f64],
        grid: &Grid,
        proximity_force_constant: f64,
    ) -> Vector2<f64> {
        let mut force = Vector2::default();
        let (x, y) = cell_at(object_position, grid.position(), grid.cell_size());
        const CELL_OFFSETS: [(isize, isize); 9] = [
            // Sorted by x because that's how grid cell records are sorted
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 0),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ];
        let radius = radii[object_index];
        for (dx, dy) in CELL_OFFSETS {
            let neighbor_cell_x = x.wrapping_add_signed(dx);
            let other_y = y.wrapping_add_signed(dy);
            if neighbor_cell_x < grid.size().x && other_y < grid.size().y {
                if let Some((records_start, records_end)) = grid.coords_to_cells[(neighbor_cell_x, other_y)] {
                    for &CellRecord {
                        object_index: other_object_index,
                        ..
                    } in &grid.cell_records[records_start..records_end]
                    {
                        if other_object_index != object_index {
                            let to_other = positions[other_object_index] - object_position;
                            let distance_squared = to_other.magnitude_squared();
                            let distance = distance_squared.sqrt();
                            let towards_other = to_other / distance;
                            let touch_distance = radius + radii[other_object_index];
                            let distance_divisor = touch_distance - distance;
                            if object_index == 0 {
                                println!("touch_distance: {touch_distance}");
                                println!("distance_squared: {distance_squared}");
                                println!("distance_divisor: {distance_divisor}");
                            }
                            force += -towards_other * proximity_force_constant / distance_divisor;
                        }
                    }
                }
            }
        }
        if object_index == 0 {
            println!("Proximity force: {:?}", force);
        }
        force
    }

    fn apply_constraints(
        object_index: usize,
        positions: &mut [Vector2<f64>],
        velocities: &mut [Vector2<f64>],
        radii: &[f64],
        cb: &ConstraintBox,
    ) {
        let position = &mut positions[object_index];
        let velocity = &mut velocities[object_index];
        let radius = radii[object_index];
        let crossed_horizontally = position.x - radius < cb.topleft.x || position.x + radius > cb.bottomright.x;
        let crossed_vertically = position.y - radius < cb.topleft.y || position.y + radius > cb.bottomright.y;
        let velocity_factor = Vector2::new(
            -1.0 * crossed_horizontally as u64 as f64 + 1.0 * (!crossed_horizontally) as u64 as f64,
            -1.0 * crossed_vertically as u64 as f64 + 1.0 * (!crossed_vertically) as u64 as f64,
        );
        *velocity *= velocity_factor;
        position.x = position.x.clamp(cb.topleft.x + radius, cb.bottomright.x - radius);
        position.y = position.y.clamp(cb.topleft.y + radius, cb.bottomright.y - radius);

        // if *velocity != initial_velocity {
        //     *velocity *= self.restitution_coefficient;
        // }
    }

    // fn update_objects_leapfrog_yoshida_gpu(&mut self, dt: f64) {
    //     if self.yoshida_gpu_buffers_are_outdated() {
    //         self.allocate_yoshida_gpu_buffers();
    //     }
    //     self.update_yoshida_gpu_buffers();
    //     if self.objects.planet_count > 0 {
    //         self.execute_yoshida_gpu_kernel(dt);
    //     } else {
    //         self.execute_yoshida_no_planets_gpu_kernel(dt);
    //     }
    // }

    // fn execute_yoshida_gpu_kernel(&mut self, dt: f64) {
    //     let position_buffer = self.yoshida_position_buffer.as_mut().unwrap();
    //     let velocity_buffer = self.yoshida_velocity_buffer.as_mut().unwrap();
    //     let planet_mass_buffer = self.yoshida_planet_mass_buffer.as_mut().unwrap();
    //     let mut kernel = ExecuteKernel::new(&self.yoshida_kernel);
    //     kernel.set_global_work_size(self.objects.len());
    //     unsafe {
    //         kernel.set_arg(position_buffer.buffer());
    //         kernel.set_arg(velocity_buffer.buffer());
    //         kernel.set_arg(&dt);
    //         kernel.set_arg(&self.global_gravity);
    //         kernel.set_arg(planet_mass_buffer.buffer());
    //         kernel.set_arg(&self.objects.planet_count);
    //         kernel.set_arg(&Self::GRAVITATIONAL_CONSTANT);
    //     }
    //     self.gpu
    //         .enqueue_execute_kernel(&mut kernel)
    //         .context("Failed to execute kernel")
    //         .unwrap();
    //     self.gpu
    //         .enqueue_read_device_buffer(position_buffer, &mut self.objects.positions)
    //         .unwrap();
    //     self.gpu
    //         .enqueue_read_device_buffer(velocity_buffer, &mut self.objects.velocities)
    //         .unwrap();
    //     self.gpu.submit_queue().unwrap();
    // }

    // fn execute_yoshida_no_planets_gpu_kernel(&mut self, dt: f64) {
    //     let position_buffer = self.yoshida_position_buffer.as_mut().unwrap();
    //     let velocity_buffer = self.yoshida_velocity_buffer.as_mut().unwrap();
    //     let mut kernel = ExecuteKernel::new(&self.yoshida_kernel_no_planets);
    //     kernel.set_global_work_size(self.objects.len());
    //     unsafe {
    //         kernel.set_arg(position_buffer.buffer());
    //         kernel.set_arg(velocity_buffer.buffer());
    //         kernel.set_arg(&dt);
    //         kernel.set_arg(&self.global_gravity);
    //     }
    //     self.gpu
    //         .enqueue_execute_kernel(&mut kernel)
    //         .context("Failed to execute kernel")
    //         .unwrap();
    //     self.gpu
    //         .enqueue_read_device_buffer(position_buffer, self.objects.positions.as_mut_slice())
    //         .unwrap();
    //     self.gpu
    //         .enqueue_read_device_buffer(velocity_buffer, self.objects.velocities.as_mut_slice())
    //         .unwrap();
    //     self.gpu.submit_queue().unwrap();
    // }

    // fn yoshida_gpu_buffers_are_outdated(&mut self) -> bool {
    //     self.yoshida_position_buffer
    //         .as_ref()
    //         .is_none_or(|b| b.len() != self.objects.len() * 2)
    // }

    // fn allocate_yoshida_gpu_buffers(&mut self) {
    //     let position_buffer = self
    //         .gpu
    //         .create_device_buffer(self.objects.positions.len(), GpuBufferAccess::ReadWrite)
    //         .context("Failed to create position buffer")
    //         .unwrap();
    //     let velocity_buffer = self
    //         .gpu
    //         .create_device_buffer(self.objects.velocities.len(), GpuBufferAccess::ReadWrite)
    //         .context("Failed to create velocity buffer")
    //         .unwrap();
    //     self.yoshida_position_buffer.replace(position_buffer);
    //     self.yoshida_velocity_buffer.replace(velocity_buffer);

    //     if self.objects.planet_count > 0 {
    //         self.yoshida_planet_mass_buffer.replace(
    //             self.gpu
    //                 .create_device_buffer(self.objects.planet_count, GpuBufferAccess::ReadOnly)
    //                 .context("Failed to create planet mass buffer")
    //                 .unwrap(),
    //         );
    //     }
    // }

    // fn update_yoshida_gpu_buffers(&mut self) {
    //     self.gpu
    //         .enqueue_write_buffer(self.yoshida_position_buffer.as_mut().unwrap(), &self.objects.positions)
    //         .unwrap();
    //     self.gpu
    //         .enqueue_write_buffer(self.yoshida_velocity_buffer.as_mut().unwrap(), &self.objects.velocities)
    //         .unwrap();
    //     if self.objects.planet_count > 0 {
    //         self.gpu
    //             .enqueue_write_buffer(
    //                 self.yoshida_planet_mass_buffer.as_mut().unwrap(),
    //                 &self.objects.masses[..self.objects.planet_count],
    //             )
    //             .unwrap();
    //     }
    // }
}

#[derive(Clone, Copy)]
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

#[derive(Default, Clone, Copy)]
struct ObjectUpdate {
    position: Vector2<f64>,
    velocity: Vector2<f64>,
}

#[derive(Clone, Copy, Debug)]
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

#[derive(Default, Clone, Copy, Debug)]
pub struct Stats {
    pub sim_time: f64,
    pub object_count: usize,
    pub updates_duration: DurationStat,
    pub grid_duration: DurationStat,
    pub total_duration: DurationStat,
}
