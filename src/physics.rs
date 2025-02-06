use core::fmt;
use std::time::Instant;

use grid::{Grid, GridCell};
use itertools::Itertools;
use nalgebra::Vector2;
use ndarray::Array2;
use object::Object;
use smallvec::SmallVec;

pub mod grid;
pub mod object;

pub enum SolverKind {
    Bruteforce,
    Grid,
}

impl SolverKind {
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

pub struct PhysicsEngine {
    pub solver_kind: SolverKind,
    objects: Vec<Object>,
    previous_accelerations: Vec<Vector2<f64>>,
    grid: Grid,
    time: f64,
    constraints: ConstraintBox,
    restitution_coefficient: f64,
    planets_end: usize,
}

impl PhysicsEngine {
    pub fn new(constraints: ConstraintBox) -> anyhow::Result<Self> {
        Ok(Self {
            solver_kind: SolverKind::Grid,
            objects: Vec::new(),
            previous_accelerations: Vec::new(),
            grid: Grid::default(),
            time: 0.0,
            constraints,
            restitution_coefficient: 1.0,
            planets_end: 0,
        })
    }

    pub fn add(&mut self, object: Object) -> usize {
        let object_index = self.objects.len();
        assert!(
            !object.is_planet || object_index == self.planets_end,
            "planets must be added before any other objects"
        );
        self.objects.push(object);
        if object.is_planet {
            self.planets_end += 1;
        }
        self.previous_accelerations.push(object.acceleration);
        object_index
    }

    pub fn grid(&self) -> &Grid {
        &self.grid
    }

    pub fn grid_mut(&mut self) -> &mut Grid {
        &mut self.grid
    }

    pub fn objects(&self) -> &[Object] {
        &self.objects
    }

    pub fn objects_mut(&mut self) -> &mut [Object] {
        &mut self.objects
    }

    pub fn advance(&mut self, dt: f64, substeps: usize) {
        self.update_substeps(dt, substeps);
    }

    pub fn constraints(&self) -> &ConstraintBox {
        &self.constraints
    }

    pub fn time(&self) -> f64 {
        self.time
    }

    fn update_substeps(&mut self, dt: f64, substeps: usize) {
        let dt_substep = dt / substeps as f64;
        for _ in 0..substeps {
            self.update(dt_substep)
        }
    }

    fn update(&mut self, dt: f64) {
        self.time += dt;
        self.grid.update(&self.objects);
        let start = Instant::now();
        self.process_collisions();
        println!("t(collisions): {:?}", start.elapsed());
        self.apply_constraints();
        let start = Instant::now();
        self.apply_gravity();
        println!("t(gravity): {:?}", start.elapsed());
        let start = Instant::now();
        self.update_objects(dt);
        println!("t(updates): {:?}", start.elapsed());
    }

    fn apply_gravity(&mut self) {
        for object_index in 0..self.objects.len() {
            self.previous_accelerations[object_index] = self.objects[object_index].acceleration;
            self.objects[object_index].acceleration = Vector2::default();
        }
        for object_index in 0..self.objects.len() {
            for planet_index in 0..self.planets_end {
                if planet_index != object_index {
                    let [object, planet] = self.objects.get_many_mut([object_index, planet_index]).unwrap();
                    let to_planet = planet.position - object.position;
                    let direction = to_planet.normalize();
                    let gravitational_constant = 10000.0;
                    object.acceleration +=
                        direction * gravitational_constant * planet.mass / to_planet.magnitude_squared();
                }
            }
        }
    }

    fn process_collisions(&mut self) {
        match self.solver_kind {
            SolverKind::Grid => self.process_collisions_on_grid(),
            SolverKind::Bruteforce => self.process_collisions_bruteforce(),
        }
    }

    fn process_collisions_on_grid(&mut self) {
        let cells = (0..self.grid.size().x as isize - 1).cartesian_product(0..self.grid.size().y as isize - 1);
        for (x, y) in cells {
            let cell = (x as usize, y as usize);
            for &object_index in &self.grid.cells[cell] {
                let adjacent_cells = (x - 1..=x + 1)
                    .cartesian_product(y - 1..=y + 1)
                    .filter(|&(x, y)| {
                        x >= 0 && x < self.grid.size().x as isize && y >= 0 && y < self.grid.size().y as isize
                    })
                    .collect::<SmallVec<[(isize, isize); 9]>>();
                for (x, y) in adjacent_cells {
                    let adjacent_cell = (x as usize, y as usize);
                    Self::process_object_with_cell_collisions(
                        object_index,
                        adjacent_cell,
                        &self.grid.cells,
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
        cells: &Array2<GridCell>,
        objects: &mut [Object],
        restitution_coefficient: f64,
    ) {
        for &object2_index in &cells[cell] {
            if object1_index != object2_index {
                let [object1, object2] = objects.get_many_mut([object1_index, object2_index]).unwrap();
                process_object_collision(object1, object2, restitution_coefficient);
            }
        }
    }

    fn process_collisions_bruteforce(&mut self) {
        for id1 in 0..self.objects.len() {
            for id2 in id1 + 1..self.objects.len() {
                if id1 != id2 {
                    let [object1, object2] = self.objects.get_many_mut([id1, id2]).expect("out of bounds");
                    process_object_collision(object1, object2, self.restitution_coefficient);
                }
            }
        }
    }

    fn update_objects(&mut self, dt: f64) {
        for object_index in 0..self.objects.len() {
            leapfrog_kdk_update_object(
                &mut self.objects[object_index],
                self.previous_accelerations[object_index],
                dt,
            )
        }
    }

    fn apply_constraints(&mut self) {
        let cb = &self.constraints;
        for object in &mut self.objects {
            if object.position.x - object.radius < cb.topleft.x {
                object.position.x = cb.topleft.x + object.radius;
                object.velocity.x = 0.0;
            }

            if object.position.x + object.radius > cb.bottomright.x {
                object.position.x = cb.bottomright.x - object.radius;
                object.velocity.x = 0.0;
            }

            if object.position.y - object.radius < cb.topleft.y {
                object.position.y = cb.topleft.y + object.radius;
                object.velocity.y = 0.0;
            }

            if object.position.y + object.radius > cb.bottomright.y {
                object.position.y = cb.bottomright.y - object.radius;
                object.velocity.y = 0.0;
            }
        }
    }
}

#[derive(Clone, Copy)]
pub struct ConstraintBox {
    pub topleft: Vector2<f64>,
    pub bottomright: Vector2<f64>,
}

impl ConstraintBox {
    pub fn new(topleft: Vector2<f64>, bottomright: Vector2<f64>) -> Self {
        Self { topleft, bottomright }
    }
}

fn leapfrog_kdk_update_object(object: &mut Object, previous_acceleration: Vector2<f64>, dt: f64) {
    let velocity_half_step = object.velocity + 0.5 * previous_acceleration * dt;
    object.position += velocity_half_step * dt;
    object.velocity = velocity_half_step + 0.5 * object.acceleration * dt
}

fn process_object_collision(object1: &mut Object, object2: &mut Object, restitution_coefficient: f64) {
    let collision_distance = object1.radius + object2.radius;
    let from_2_to_1 = object1.position - object2.position;
    let distance = (from_2_to_1).magnitude();
    if distance < collision_distance {
        let total_mass = object1.mass + object2.mass;
        let velocity_diff = object1.velocity - object2.velocity;
        let divisor = total_mass * distance * distance;
        object1.velocity =
            object1.velocity - 2.0 * object2.mass * velocity_diff.dot(&from_2_to_1) * from_2_to_1 / divisor;
        object2.velocity =
            object2.velocity - 2.0 * object1.mass * (-velocity_diff).dot(&(-from_2_to_1)) * -from_2_to_1 / divisor;
        let from_2_to_1_unit = from_2_to_1.normalize();
        let intersection_depth = collision_distance - distance;
        let distance_correction = intersection_depth;
        let momentum1 = object1.mass * object1.velocity.magnitude();
        let momentum2 = object2.mass * object2.velocity.magnitude();
        let total_momentum = momentum1 + momentum2;
        let correction_base = from_2_to_1_unit * distance_correction / total_momentum;
        object1.position += momentum2 * correction_base;
        object2.position -= momentum1 * correction_base;

        if !object1.is_planet {
            object1.velocity *= restitution_coefficient;
        }
        if !object2.is_planet {
            object2.velocity *= restitution_coefficient;
        }
    }
}
