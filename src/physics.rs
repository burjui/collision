use core::iter::Iterator;

use grid::{Cell, Grid};
use itertools::Itertools;
use nalgebra::Vector2;
use object::Object;

pub mod grid;
pub mod object;

pub struct PhysicsEngine {
    pub grid: Grid,
    pub time: f64,
    pub constraints: Option<ConstraintBox>,
}

impl PhysicsEngine {
    pub fn new() -> Self {
        Self {
            grid: Grid::new(),
            time: 0.0,
            constraints: None,
        }
    }

    pub fn add(&mut self, object: Object) -> usize {
        let id = self.grid.objects.len();
        self.grid.objects.push(Object { ..object });
        id
    }

    pub fn objects(&self) -> impl Iterator<Item = (usize, Object)> + '_ {
        self.grid.objects.iter().copied().enumerate()
    }

    pub fn object_mut(&mut self, id: usize) -> &mut Object {
        &mut self.grid.objects[id]
    }

    pub fn object_count(&self) -> usize {
        self.grid.objects.len()
    }

    pub fn advance(&mut self, dt: f64) {
        self.grid.update();
        self.update_substeps(dt, 8);
    }

    fn update_substeps(&mut self, dt: f64, substeps: usize) {
        let dt_substep = dt / substeps as f64;
        for _ in 0..substeps {
            self.update(dt_substep)
        }
    }

    fn update(&mut self, dt: f64) {
        self.time += dt;
        self.apply_gravity();
        self.process_collisions(dt);
        if let Some(constraint_box) = self.constraints {
            self.apply_constraints(constraint_box);
        }
        self.update_objects(dt);
    }

    fn apply_gravity(&mut self) {
        for object in &mut self.grid.objects {
            object.acceleration += Vector2::new(0.0, 10000.0);
        }
    }

    fn process_collisions(&mut self, dt: f64) {
        // self.process_collisions_on_grid(dt);
        self.process_collisions_bruteforce(dt);
    }

    fn process_collisions_on_grid(&mut self) {
        for cell in (1..self.grid.size.x - 1)
            .cartesian_product(1..self.grid.size.y - 1)
            .map(|(x, y)| Cell::new(x, y))
        {
            // 'adjacent_cell' includes 'cell'
            for adjacent_cell in (cell.x - 1..=cell.x + 1)
                .cartesian_product(cell.y - 1..=cell.y + 1)
                .map(|(x, y)| Cell::new(x, y))
            {
                self.process_cell_collisions(cell, adjacent_cell);
                // let mut collision = None;
                // for &id2 in &self.grid.cells[(adjacent_cell.x, adjacent_cell.y)] {
                //     if id1 != id2 {
                //         let [o1, o2] = self.grid.objects.get_many_mut([id1, id2]).expect("out of bounds");
                //         if intersects(o1, o2) {
                //             let (p1, p2) = (o1.position, o2.position);
                //             collide_elastic(o1, o2, dt);
                //             collision = Some(((id1, p1), (id2, p2)));
                //         };
                //     }
                // }
                // if let Some(((id1, p1), (id2, p2))) = collision {
                //     self.grid.update_object_cell(id1, p1);
                //     self.grid.update_object_cell(id2, p2);
                // }
            }
        }
    }

    fn process_cell_collisions(&mut self, cell1: Cell, cell2: Cell) {
        for &id1 in &self.grid.cells[(cell1.x, cell1.y)] {
            for &id2 in &self.grid.cells[(cell2.x, cell2.y)] {
                if id1 != id2 {
                    let [o1, o2] = self.grid.objects.get_many_mut([id1, id2]).expect("out of bounds");
                    if intersects(o1, o2) {
                        collide(o1, o2);
                    }
                }
            }
        }
    }

    fn process_collisions_bruteforce(&mut self, dt: f64) {
        for id1 in 0..self.grid.objects.len() {
            for id2 in id1 + 1..self.grid.objects.len() {
                if id1 != id2 {
                    let [object_1, object_2] = self.grid.objects.get_many_mut([id1, id2]).expect("out of bounds");
                    if intersects(object_1, object_2) {
                        collide(object_1, object_2);
                    }
                }
            }
        }
    }

    fn update_objects(&mut self, dt: f64) {
        for object in &mut self.grid.objects {
            update_object(object, dt)
        }
    }

    fn apply_constraints(&mut self, cb: ConstraintBox) {
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

    pub fn time(&self) -> f64 {
        self.time
    }
}

#[derive(Clone, Copy)]
pub struct ConstraintBox {
    topleft: Vector2<f64>,
    bottomright: Vector2<f64>,
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
    object.acceleration = Vector2::new(0.0, 0.0);
}

fn intersects(o1: &Object, o2: &Object) -> bool {
    let d = o1.position - o2.position;
    let r = o1.radius + o2.radius;
    d.magnitude() <= r
}

fn collide(object_1: &mut Object, object_2: &mut Object) {
    const RESPONSE_COEF: f64 = 1.0;

    let axis_vector = object_1.position - object_2.position;
    let collision_distance = object_1.radius + object_2.radius;
    let distance = axis_vector.magnitude();
    if distance < collision_distance {
        let axis_vectoer_unit = axis_vector.normalize();
        let total_mass = object_1.mass + object_2.mass;
        let abs_displacement = 0.5 * RESPONSE_COEF * (distance - collision_distance);
        object_1.position -= axis_vectoer_unit * ((object_2.mass / total_mass) * abs_displacement);
        object_2.position += axis_vectoer_unit * ((object_1.mass / total_mass) * abs_displacement);
    }
}
