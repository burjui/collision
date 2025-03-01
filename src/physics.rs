use core::{f32, fmt};
use std::time::Instant;

use anyhow::Context as _;
use grid::{CellRecord, Grid};
use itertools::Itertools;
use object::{Object, ObjectUpdate};
use opencl3::kernel::{ExecuteKernel, Kernel};

use crate::{
    app_config::AppConfig,
    gpu::{Gpu, GpuBufferAccess, GpuHostBuffer},
    vector2::Vector2,
};

pub mod grid;
mod leapfrog_yoshida;
pub mod object;

pub struct PhysicsEngine {
    pub solver_kind: SolverKind,
    pub enable_constraint_bouncing: bool,
    pub enable_gpu: bool,
    objects: Vec<Object>,
    grid: Grid,
    time: f32,
    constraints: ConstraintBox,
    restitution_coefficient: f32,
    planets_count: usize,
    global_gravity: Vector2<f32>,
    gpu: Gpu,
    yoshida_kernel: Kernel,
    yoshida_kernel_no_planets: Kernel,
    yoshida_position_buffer: Option<GpuHostBuffer<f32>>,
    yoshida_velocity_buffer: Option<GpuHostBuffer<f32>>,
    yoshida_planet_mass_buffer: Option<GpuHostBuffer<f32>>,
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
            solver_kind: SolverKind::Grid,
            enable_constraint_bouncing: false,
            enable_gpu: true,
            objects: Vec::default(),
            grid: Grid::default(),
            time: 0.0,
            constraints,
            restitution_coefficient: config.restitution_coefficient,
            planets_count: 0,
            global_gravity: Vector2::new(0.0, 1000.0),
            gpu,
            yoshida_kernel,
            yoshida_kernel_no_planets,
            yoshida_position_buffer: None,
            yoshida_velocity_buffer: None,
            yoshida_planet_mass_buffer: None,
        })
    }

    pub fn add(&mut self, object: Object) -> usize {
        let object_index = self.objects.len();
        assert!(
            !object.is_planet || object_index == self.planets_count,
            "planets must be added before any other objects"
        );
        self.objects.push(object);
        self.planets_count += usize::from(object.is_planet);
        object_index
    }

    #[must_use]
    pub fn grid(&self) -> &Grid {
        &self.grid
    }

    pub fn grid_mut(&mut self) -> &mut Grid {
        &mut self.grid
    }

    #[must_use]
    pub fn objects(&self) -> &[Object] {
        &self.objects
    }

    pub fn objects_mut(&mut self) -> &mut [Object] {
        &mut self.objects
    }

    pub fn advance(&mut self, dt: f32, substeps: usize) {
        let start = Instant::now();
        self.update_substeps(dt, substeps);
        println!("total time: {:?}", start.elapsed());
        println!("-----------");
    }

    #[must_use]
    pub fn constraints(&self) -> &ConstraintBox {
        &self.constraints
    }

    #[must_use]
    pub fn time(&self) -> f32 {
        self.time
    }

    pub fn objects_momentum(&self) -> Vector2<f32> {
        self.objects.iter().map(Object::momentum).sum()
    }

    fn update_substeps(&mut self, dt: f32, substeps: usize) {
        println!("particles: {}", self.objects.len());
        let dt_substep = dt / substeps as f32;
        for _ in 0..substeps {
            self.update(dt_substep);
        }
    }

    fn update(&mut self, dt: f32) {
        self.time += dt;

        let start = Instant::now();
        self.update_objects(dt);
        println!("updates: {:?}", start.elapsed());

        let start = Instant::now();
        self.grid.update(&self.objects);
        println!("grid: {:?}", start.elapsed());

        let start = Instant::now();
        self.process_collisions();
        println!("collisions: {:?}", start.elapsed());

        let start = Instant::now();
        self.apply_constraints();
        println!("constraints: {:?}", start.elapsed());
    }

    fn process_collisions(&mut self) {
        match self.solver_kind {
            SolverKind::Grid => self.process_collisions_on_grid(),
            SolverKind::Bruteforce => self.process_collisions_bruteforce(),
        }
    }

    fn process_collisions_on_grid(&mut self) {
        for range in self.grid.cell_iter() {
            let cell_records = &self.grid.cell_records[range];
            let (x, y) = cell_records[0].cell_coords;
            for &CellRecord { object_index, .. } in cell_records {
                let adjacent_cells = (x.saturating_sub(1)..=x + 1)
                    .cartesian_product(y.saturating_sub(1)..=y + 1)
                    .filter(|&(x, y)| x < self.grid.size().x && y < self.grid.size().y);
                for adjacent_cell in adjacent_cells {
                    Self::process_object_with_cell_collisions(
                        object_index,
                        adjacent_cell,
                        &self.grid,
                        &mut self.objects,
                        self.restitution_coefficient,
                    );
                }
            }
        }
    }

    fn process_object_with_cell_collisions(
        object1_index: usize,
        cell: (usize, usize),
        grid: &Grid,
        objects: &mut [Object],
        restitution_coefficient: f32,
    ) {
        if let Some((start, end)) = grid.cells_map[cell] {
            // println!("cell: ({}, {})", start, end);
            for &CellRecord {
                object_index: object2_index,
                ..
            } in &grid.cell_records[start..end]
            {
                if object1_index != object2_index {
                    let [object1, object2] = objects.get_disjoint_mut([object1_index, object2_index]).unwrap();
                    process_object_collision(object1, object2, restitution_coefficient);
                }
            }
        }
    }

    fn process_collisions_bruteforce(&mut self) {
        for id1 in 0..self.objects.len() {
            for id2 in id1 + 1..self.objects.len() {
                if id1 != id2 {
                    let [object1, object2] = self.objects.get_disjoint_mut([id1, id2]).expect("out of bounds");
                    process_object_collision(object1, object2, self.restitution_coefficient);
                }
            }
        }
    }

    fn update_objects(&mut self, dt: f32) {
        if self.enable_gpu {
            self.update_objects_leapfrog_yoshida_gpu(dt);
        } else {
            for object_index in 0..self.objects.len() {
                let update = self.update_object_leapfrog_yoshida(object_index, dt);
                self.objects[object_index].update(update);
            }
        }
    }

    fn update_objects_leapfrog_yoshida_gpu(&mut self, dt: f32) {
        if self.yoshida_gpu_buffers_are_outdated() {
            self.allocate_yoshida_gpu_buffers();
        }
        self.update_yoshida_gpu_buffers();
        if self.planets_count > 0 {
            self.execute_yoshida_gpu_kernel(dt);
        } else {
            self.execute_yoshida_no_planets_gpu_kernel(dt);
        }
        self.update_objects_from_gpu_buffers();
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
            kernel.set_arg(&self.planets_count);
            kernel.set_arg(&Self::GRAVITATIONAL_CONSTANT);
        }
        self.gpu
            .enqueue_execute_kernel(&mut kernel)
            .context("Failed to execute kernel")
            .unwrap();
        self.gpu.enqueue_read_host_buffer(position_buffer).unwrap();
        self.gpu.enqueue_read_host_buffer(velocity_buffer).unwrap();
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
        self.gpu.enqueue_read_host_buffer(position_buffer).unwrap();
        self.gpu.enqueue_read_host_buffer(velocity_buffer).unwrap();
        self.gpu.submit_queue().unwrap();
    }

    fn yoshida_gpu_buffers_are_outdated(&mut self) -> bool {
        self.yoshida_position_buffer
            .as_ref()
            .is_none_or(|b| b.data().len() != self.objects.len() * 2)
    }
    fn allocate_yoshida_gpu_buffers(&mut self) {
        let mut positions = self
            .yoshida_position_buffer
            .take()
            .map(GpuHostBuffer::take_data)
            .unwrap_or_default();
        let mut velocities = self
            .yoshida_velocity_buffer
            .take()
            .map(GpuHostBuffer::take_data)
            .unwrap_or_default();
        positions.resize(self.objects.len() * 2, 0.0);
        velocities.resize(self.objects.len() * 2, 0.0);

        let position_buffer = self
            .gpu
            .create_host_buffer(positions, GpuBufferAccess::ReadWrite)
            .context("Failed to create position buffer")
            .unwrap();
        let velocity_buffer = self
            .gpu
            .create_host_buffer(velocities, GpuBufferAccess::ReadWrite)
            .context("Failed to create velocity buffer")
            .unwrap();
        self.yoshida_position_buffer.replace(position_buffer);
        self.yoshida_velocity_buffer.replace(velocity_buffer);

        if self.planets_count > 0 {
            let mut planet_masses = self
                .yoshida_planet_mass_buffer
                .take()
                .map(GpuHostBuffer::take_data)
                .unwrap_or_default();
            planet_masses.resize(self.planets_count, 0.0);
            self.yoshida_planet_mass_buffer.replace(
                self.gpu
                    .create_host_buffer(planet_masses, GpuBufferAccess::ReadOnly)
                    .context("Failed to create planet mass buffer")
                    .unwrap(),
            );
        }
    }

    fn update_yoshida_gpu_buffers(&mut self) {
        let position_buffer = self.yoshida_position_buffer.as_mut().unwrap();
        let velocity_buffer = self.yoshida_velocity_buffer.as_mut().unwrap();
        let positions_len = position_buffer.data().len();
        position_buffer.data_mut().splice(
            0..positions_len,
            self.objects
                .iter()
                .flat_map(|Object { position, .. }| [position.x, position.y]),
        );
        let velocities_len = velocity_buffer.data().len();
        velocity_buffer.data_mut().splice(
            0..velocities_len,
            self.objects
                .iter()
                .flat_map(|Object { velocity, .. }| [velocity.x, velocity.y]),
        );

        if self.planets_count > 0 {
            let planet_mass_buffer = self.yoshida_planet_mass_buffer.as_mut().unwrap();
            let planet_masses_len = planet_mass_buffer.data().len();
            planet_mass_buffer.data_mut().splice(
                0..planet_masses_len,
                self.objects[0..self.planets_count]
                    .iter()
                    .map(|&Object { mass, .. }| mass),
            );
        }
    }

    fn update_objects_from_gpu_buffers(&mut self) {
        let positions = self.yoshida_position_buffer.as_ref().unwrap().data();
        let velocities = self.yoshida_velocity_buffer.as_ref().unwrap().data();
        self.objects
            .iter_mut()
            .zip(positions.chunks(2).map(Vector2::from_slice))
            .zip(velocities.chunks(2).map(Vector2::from_slice))
            .for_each(|((object, position), velocity)| {
                object.position = position;
                object.velocity = velocity;
            });
    }

    fn update_object_leapfrog_yoshida(&self, object_index: usize, dt: f32) -> ObjectUpdate {
        use leapfrog_yoshida::{C1, C2, C3, C4, D1, D2, D3};
        let Object {
            position: x0,
            velocity: v0,
            ..
        } = self.objects[object_index];
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
        for planet_index in 0..self.planets_count {
            if planet_index != object_index {
                let planet = &self.objects[planet_index];
                let to_planet = planet.position - position;
                let direction = to_planet.normalize();
                gravity += direction
                    * (Self::GRAVITATIONAL_CONSTANT * planet.mass / to_planet.magnitude_squared().max(f32::EPSILON));
            }
        }
        gravity
    }

    fn apply_constraints(&mut self) {
        let cb = self.constraints;
        for object in &mut self.objects {
            if object.position.x - object.radius < cb.topleft.x {
                object.position.x = cb.topleft.x + object.radius;
                if self.enable_constraint_bouncing {
                    object.velocity.x *= -1.0;
                }
            } else if object.position.x + object.radius > cb.bottomright.x {
                object.position.x = cb.bottomright.x - object.radius;
                if self.enable_constraint_bouncing {
                    object.velocity.x *= -1.0;
                }
            }

            if object.position.y - object.radius < cb.topleft.y {
                object.position.y = cb.topleft.y + object.radius;
                if self.enable_constraint_bouncing {
                    object.velocity.y *= -1.0;
                }
            } else if object.position.y + object.radius > cb.bottomright.y {
                object.position.y = cb.bottomright.y - object.radius;
                if self.enable_constraint_bouncing {
                    object.velocity.y *= -1.0;
                }
            }
        }
    }
}

pub enum SolverKind {
    Bruteforce,
    Grid,
}

impl SolverKind {
    #[must_use]
    pub fn next(&self) -> Self {
        match self {
            SolverKind::Bruteforce => SolverKind::Grid,
            SolverKind::Grid => SolverKind::Bruteforce,
        }
    }
}

impl fmt::Display for SolverKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SolverKind::Bruteforce => write!(f, "bruteforce"),
            SolverKind::Grid => write!(f, "grid"),
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

fn process_object_collision(object1: &mut Object, object2: &mut Object, restitution_coefficient: f32) {
    let collision_distance = object1.radius + object2.radius;
    let from_2_to_1 = object1.position - object2.position;
    if from_2_to_1.magnitude_squared() < collision_distance * collision_distance {
        let total_mass = object1.mass + object2.mass;
        let velocity_diff = object1.velocity - object2.velocity;
        let distance = from_2_to_1.magnitude();
        let divisor = (total_mass * distance * distance).max(f32::EPSILON);
        object1.velocity -= from_2_to_1 * (2.0 * object2.mass * velocity_diff.dot(&from_2_to_1) / divisor);
        object2.velocity -= -from_2_to_1 * (2.0 * object1.mass * (-velocity_diff).dot(&(-from_2_to_1)) / divisor);
        let from_2_to_1_unit = from_2_to_1.normalize();
        let intersection_depth = collision_distance - distance;
        let distance_correction = intersection_depth;
        let momentum1 = object1.mass * object1.velocity.magnitude();
        let momentum2 = object2.mass * object2.velocity.magnitude();
        let total_momentum = momentum1 + momentum2;
        let correction_base = from_2_to_1_unit * (distance_correction / total_momentum.max(f32::EPSILON));
        object1.position += correction_base * momentum2;
        object2.position -= correction_base * momentum1;

        if !object1.is_planet {
            object1.velocity *= restitution_coefficient;
        }
        if !object2.is_planet {
            object2.velocity *= restitution_coefficient;
        }
    }
}
