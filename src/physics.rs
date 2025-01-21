use core::fmt;

use grid::Grid;
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
    grid: Grid,
    time: f64,
    constraints: ConstraintBox,
    collision_damping_coefficient: f64,
    planets_end: usize,
}

impl PhysicsEngine {
    pub fn new(constraints: ConstraintBox) -> Self {
        Self {
            solver_kind: SolverKind::Grid,
            grid: Grid::default(),
            time: 0.0,
            constraints,
            collision_damping_coefficient: 1.0 - 0.00007,
            planets_end: 0,
        }
    }

    pub fn add(&mut self, object: Object) -> usize {
        let object_index = self.grid().objects.len();
        assert!(
            !object.is_planet || object_index == self.planets_end,
            "planets must be added before any other objects"
        );
        self.grid.objects.push(object);
        if object.is_planet {
            self.planets_end += 1;
        }
        object_index
    }

    pub fn grid(&self) -> &Grid {
        &self.grid
    }

    pub fn grid_mut(&mut self) -> &mut Grid {
        &mut self.grid
    }

    pub fn object_mut(&mut self, index: usize) -> &mut Object {
        &mut self.grid.objects[index]
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
        self.grid.update();
        self.apply_gravity();
        self.process_collisions();
        self.apply_constraints();
        self.update_objects(dt);
    }

    fn apply_gravity(&mut self) {
        for object in &mut self.grid.objects {
            object.acceleration = Vector2::new(0.0, 0.0);
        }
        for object_index in 0..self.grid.objects.len() {
            for planet_index in 0..self.planets_end {
                if planet_index != object_index {
                    let [object, planet] = self.grid.objects.get_many_mut([object_index, planet_index]).unwrap();
                    let direction = (planet.position - object.position).normalize();
                    let distance = (planet.position - object.position).magnitude();
                    let scale_factor = if object.is_planet { 1.0 } else { 1700.0 };
                    object.acceleration += direction * planet.mass * object.mass * scale_factor / distance.powf(1.0);
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
                        &mut self.grid.objects,
                        self.collision_damping_coefficient,
                    );
                }
            }
        }
    }

    fn process_object_with_cell_collisions(
        object1_index: usize,
        cell: (usize, usize),
        cells: &Array2<SmallVec<[usize; 4]>>,
        objects: &mut [Object],
        collision_damping_coefficient: f64,
    ) {
        for &object2_index in &cells[cell] {
            if object1_index != object2_index {
                let [object1, object2] = objects.get_many_mut([object1_index, object2_index]).unwrap();
                process_object_collision(object1, object2, collision_damping_coefficient);
            }
        }
    }

    fn process_collisions_bruteforce(&mut self) {
        for id1 in 0..self.grid.objects.len() {
            for id2 in id1 + 1..self.grid.objects.len() {
                if id1 != id2 {
                    let [object1, object2] = self.grid.objects.get_many_mut([id1, id2]).expect("out of bounds");
                    process_object_collision(object1, object2, self.collision_damping_coefficient);
                }
            }
        }
    }

    fn update_objects(&mut self, dt: f64) {
        for object in &mut self.grid.objects {
            update_object(object, dt)
        }
    }

    fn apply_constraints(&mut self) {
        let cb = &self.constraints;
        for object in &mut self.grid.objects {
            if object.position.x - object.radius < cb.topleft.x {
                object.position.x = cb.topleft.x + object.radius;
            }

            if object.position.x + object.radius > cb.bottomright.x {
                object.position.x = cb.bottomright.x - object.radius;
            }

            if object.position.y - object.radius < cb.topleft.y {
                object.position.y = cb.topleft.y + object.radius;
            }

            if object.position.y + object.radius > cb.bottomright.y {
                object.position.y = cb.bottomright.y - object.radius;
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

fn update_object(object: &mut Object, dt: f64) {
    let displacement = object.position - object.previous_position;
    object.previous_position = object.position;
    object.position += displacement + object.acceleration * (dt * dt);
}

fn process_object_collision(object1: &mut Object, object2: &mut Object, collision_damping_coefficient: f64) {
    let axis_vector = object1.position - object2.position;
    let collision_distance = object1.radius + object2.radius;
    let distance = axis_vector.magnitude();
    if distance < collision_distance {
        let axis_vector_unit = axis_vector.normalize();
        let total_mass = object1.mass + object2.mass;
        let abs_displacement = 0.5 * (distance - collision_distance);
        object1.position -= axis_vector_unit * ((object2.mass / total_mass) * abs_displacement);
        object2.position += axis_vector_unit * ((object1.mass / total_mass) * abs_displacement);
        if !object1.is_planet {
            object1.set_velocity(object1.velocity() * collision_damping_coefficient, 1.0);
        }
        if !object2.is_planet {
            object2.set_velocity(object2.velocity() * collision_damping_coefficient, 1.0);
        }
    }
}
