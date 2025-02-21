use core::{f64, fmt};
use std::time::Instant;

use grid::{Grid, GridCell};
use itertools::Itertools;
use object::Object;

use crate::{app_config::AppConfig, array2::Array2, vector2::Vector2};

pub mod grid;
mod leapfrog_yoshida;
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
    pub enable_constraint_bouncing: bool,
    objects: Vec<Object>,
    grid: Grid,
    time: f64,
    constraints: ConstraintBox,
    restitution_coefficient: f64,
    planets_count: usize,
}

impl PhysicsEngine {
    pub fn new(config: &AppConfig) -> anyhow::Result<Self> {
        let constraints = ConstraintBox::new(
            Vector2::new(0.0, 0.0),
            Vector2::new(config.width as f64, config.height as f64),
        );
        Ok(Self {
            solver_kind: SolverKind::Grid,
            enable_constraint_bouncing: false,
            objects: Vec::default(),
            grid: Grid::default(),
            time: 0.0,
            constraints,
            restitution_coefficient: config.restitution_coefficient,
            planets_count: 0,
        })
    }

    pub fn add(&mut self, object: Object) -> usize {
        let object_index = self.objects.len();
        assert!(
            !object.is_planet || object_index == self.planets_count,
            "planets must be added before any other objects"
        );
        self.objects.push(object);
        self.planets_count += object.is_planet as usize;
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

    pub fn objects_momentum(&self) -> Vector2<f64> {
        self.objects.iter().map(Object::momentum).sum()
    }

    fn update_substeps(&mut self, dt: f64, substeps: usize) {
        let dt_substep = dt / substeps as f64;
        for _ in 0..substeps {
            self.update(dt_substep)
        }
    }

    fn update(&mut self, dt: f64) {
        self.time += dt;

        let start = Instant::now();
        self.apply_gravity();
        println!("gravity: {:?}", start.elapsed());

        let start = Instant::now();
        self.grid.update(&self.objects);
        println!("grid: {:?}", start.elapsed());

        let start = Instant::now();
        self.process_collisions();
        println!("collisions: {:?}", start.elapsed());

        let start = Instant::now();
        self.update_objects(dt);
        println!("updates: {:?}", start.elapsed());

        let start = Instant::now();
        self.apply_constraints();
        println!("constraints: {:?}", start.elapsed());
    }

    fn apply_gravity(&mut self) {
        for object_index in 0..self.objects.len() {
            let gravity = self.gravity_accel(object_index, self.objects[object_index].position);
            self.objects[object_index].acceleration += gravity;
        }
    }

    fn gravity_accel(&self, object_index: usize, position: Vector2<f64>) -> Vector2<f64> {
        let mut gravity = Vector2::default();
        for planet_index in 0..self.planets_count {
            if planet_index != object_index {
                let planet = &self.objects[planet_index];
                let to_planet = planet.position - position;
                let direction = to_planet.normalize();
                let gravitational_constant = 10000.0;
                gravity += direction * (gravitational_constant * planet.mass / to_planet.magnitude_squared());
            }
        }
        gravity
    }

    fn process_collisions(&mut self) {
        match self.solver_kind {
            SolverKind::Grid => self.process_collisions_on_grid(),
            SolverKind::Bruteforce => self.process_collisions_bruteforce(),
        }
    }

    fn process_collisions_on_grid(&mut self) {
        let cells = (0..self.grid.size().x - 1).cartesian_product(0..self.grid.size().y);
        for cell @ (x, y) in cells {
            let cell = &self.grid.cells[cell];
            if !cell.is_empty() {
                for &object_index in cell.as_slice() {
                    let adjacent_cells = (x.saturating_sub(1)..=x + 1)
                        .cartesian_product(y.saturating_sub(1)..=y + 1)
                        .filter(|&(x, y)| x < self.grid.size().x && y < self.grid.size().y);
                    for adjacent_cell in adjacent_cells {
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
    }

    fn process_object_with_cell_collisions(
        object1_index: usize,
        cell: (usize, usize),
        cells: &Array2<GridCell>,
        objects: &mut [Object],
        restitution_coefficient: f64,
    ) {
        for &object2_index in cells[cell].as_slice() {
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
            self.leapfrog_yoshida_update_object(object_index, dt)
        }
    }

    fn leapfrog_yoshida_update_object(&mut self, object_index: usize, dt: f64) {
        use leapfrog_yoshida::*;
        let Object {
            position: x0,
            velocity: v0,
            ..
        } = self.objects[object_index];
        let x1 = x0 + v0 * (C1 * dt);
        let v1 = v0 + self.gravity_accel(object_index, x1) * (D1 * dt);
        let x2 = x0 + v1 * (C2 * dt);
        let v2 = v0 + self.gravity_accel(object_index, x2) * (D2 * dt);
        let x3 = x0 + v2 * (C3 * dt);
        let v3 = v0 + self.gravity_accel(object_index, x3) * (D3 * dt);
        self.objects[object_index].position = x0 + v3 * (C4 * dt);
        self.objects[object_index].velocity = v3;
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

fn process_object_collision(object1: &mut Object, object2: &mut Object, restitution_coefficient: f64) {
    let collision_distance = object1.radius + object2.radius;
    let from_2_to_1 = object1.position - object2.position;
    if from_2_to_1.magnitude_squared() < collision_distance * collision_distance {
        let total_mass = object1.mass + object2.mass;
        let velocity_diff = object1.velocity - object2.velocity;
        let distance = from_2_to_1.magnitude();
        let divisor = (total_mass * distance * distance).max(f64::EPSILON);
        object1.velocity -= from_2_to_1 * (2.0 * object2.mass * velocity_diff.dot(&from_2_to_1) / divisor);
        object2.velocity -= -from_2_to_1 * (2.0 * object1.mass * (-velocity_diff).dot(&(-from_2_to_1)) / divisor);
        let from_2_to_1_unit = from_2_to_1.normalize();
        let intersection_depth = collision_distance - distance;
        let distance_correction = intersection_depth;
        let momentum1 = object1.mass * object1.velocity.magnitude();
        let momentum2 = object2.mass * object2.velocity.magnitude();
        let total_momentum = momentum1 + momentum2;
        let correction_base = from_2_to_1_unit * (distance_correction / total_momentum.max(f64::EPSILON));
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
