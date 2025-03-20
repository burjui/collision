use core::f64;
use std::{
    iter::zip,
    time::{Duration, Instant},
};

use grid::Grid;
use object::{ObjectPrototype, ObjectSoa, ObjectUpdate};

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
    grid: Grid,
    time: f64,
    constraints: ConstraintBox,
    stats: Stats,
    // restitution_coefficient: f64,
    global_gravity: Vector2<f64>,
    // gpu: Gpu,
    // yoshida_kernel: Kernel,
    // yoshida_kernel_no_planets: Kernel,
    // yoshida_position_buffer: Option<GpuDeviceBuffer<Vector2<f64>>>,
    // yoshida_velocity_buffer: Option<GpuDeviceBuffer<Vector2<f64>>>,
    // yoshida_planet_mass_buffer: Option<GpuDeviceBuffer<f64>>,
}

impl PhysicsEngine {
    const GRAVITATIONAL_CONSTANT: f64 = 10000.0;

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
            grid: Grid::default(),
            time: 0.0,
            constraints,
            stats: Stats::default(),
            // restitution_coefficient: config.simulation.restitution_coefficient,
            global_gravity: Vector2::from(config.simulation.gravity),
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
                let max_gravity_squared = self.objects.positions.iter().enumerate().fold(
                    0.0,
                    |max_gravity_squared, (object_index, &position)| {
                        let gravity_squared = Self::gravity_acceleration(
                            object_index,
                            position,
                            &self.objects.positions,
                            self.global_gravity,
                            &self.objects.masses[..self.objects.planet_count],
                        )
                        .magnitude_squared();
                        if gravity_squared > max_gravity_squared {
                            gravity_squared
                        } else {
                            max_gravity_squared
                        }
                    },
                );
                // Experimentally derived
                let force_factor = max_gravity_squared.sqrt().sqrt() * 80.0 / min_object_size.sqrt();
                speed_factor / 2.0 * (1.0 / velocity_factor.max(force_factor).max(1.0))
            }
            DtSource::Fixed(dt) => dt,
        };
        self.time += dt;
        self.update(dt);

        self.stats.total_duration.update(start.elapsed());
    }

    #[must_use]
    pub fn constraints(&self) -> &ConstraintBox {
        &self.constraints
    }

    #[must_use]
    pub fn time(&self) -> f64 {
        self.time
    }

    fn update(&mut self, dt: f64) {
        self.time += dt;

        let start = Instant::now();
        self.grid.update(&self.objects);
        self.stats.grid_duration.update(start.elapsed());

        let start = Instant::now();
        self.update_objects(dt);
        self.stats.updates_duration.update(start.elapsed());
    }

    fn update_objects(&mut self, dt: f64) {
        // if config().simulation.enable_gpu {
        //     self.update_objects_leapfrog_yoshida_gpu(dt);
        // } else {
        self.update_object_leapfrog_yoshida_cpu(dt)
        // }
    }

    fn update_object_leapfrog_yoshida_cpu(&mut self, dt: f64) {
        for object_index in 0..self.objects.positions.len() {
            let update = Self::update_object_leapfrog_yoshida(
                object_index,
                &self.objects.positions,
                self.objects.velocities[object_index],
                dt,
                self.global_gravity,
                &self.objects.masses[..self.objects.planet_count],
            );
            self.objects.positions[object_index] = update.position;
            self.objects.velocities[object_index] = update.velocity;
        }
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

    fn update_object_leapfrog_yoshida(
        object_index: usize,
        positions: &[Vector2<f64>],
        velocity: Vector2<f64>,
        dt: f64,
        global_gravity: Vector2<f64>,
        planet_masses: &[f64],
    ) -> ObjectUpdate {
        use leapfrog_yoshida::{C1, C2, C3, C4, D1, D2, D3};
        let x0 = positions[object_index];
        let v0 = velocity;
        let x1 = x0 + v0 * (C1 * dt);
        let a1 = Self::gravity_acceleration(object_index, x1, positions, global_gravity, planet_masses);
        let v1 = v0 + a1 * (D1 * dt);
        let x2 = x0 + v1 * (C2 * dt);
        let a2 = Self::gravity_acceleration(object_index, x2, positions, global_gravity, planet_masses);
        let v2 = v0 + a2 * (D2 * dt);
        let x3 = x0 + v2 * (C3 * dt);
        let a3 = Self::gravity_acceleration(object_index, x3, positions, global_gravity, planet_masses);
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
        planet_masses: &[f64],
    ) -> Vector2<f64> {
        let mut gravity = global_gravity;
        for planet_index in 0..planet_masses.len() {
            if planet_index != object_index {
                let to_planet = positions[planet_index] - position;
                let direction = to_planet.normalize();
                gravity += direction
                    * (Self::GRAVITATIONAL_CONSTANT * planet_masses[planet_index] / to_planet.magnitude_squared());
            }
        }
        gravity
    }
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
    pub total_duration: DurationStat,
}
