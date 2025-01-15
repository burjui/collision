use core::iter::Iterator;

use cgmath::InnerSpace;
use grid::{Cell, Grid};
use itertools::Itertools;
use object::{Object, ObjectId};

pub mod grid;
pub mod object;

pub struct PhysicsEngine {
    pub grid: Grid,
    pub time: f64,
}

impl PhysicsEngine {
    pub fn new() -> Self {
        Self {
            grid: Grid::new(),
            time: 0.0,
        }
    }

    pub fn add(&mut self, object: Object) -> ObjectId {
        let id = ObjectId(self.grid.objects.len());
        self.grid.objects.push(Object { ..object });
        id
    }

    pub fn objects(&self) -> impl Iterator<Item = (ObjectId, Object)> + '_ {
        self.grid
            .objects
            .iter()
            .enumerate()
            .map(|(id, object)| (ObjectId(id), *object))
    }

    pub fn object_mut(&mut self, id: ObjectId) -> &mut Object {
        &mut self.grid.objects[id.0]
    }

    pub fn object_count(&self) -> usize {
        self.grid.objects.len()
    }

    pub fn advance(&mut self, dt: f64) {
        self.grid.update();
        self.update_substeps(dt, 16);
    }

    fn update_substeps(&mut self, dt: f64, substeps: usize) {
        let dt_substep = dt / substeps as f64;
        for _ in 0..substeps {
            self.update(dt_substep)
        }
    }

    fn update(&mut self, dt: f64) {
        self.time += dt;
        self.process_collisions(dt);
        self.update_objects(dt);
        self.grid.update();
    }

    fn update_objects(&mut self, dt: f64) {
        for object in &mut self.grid.objects {
            update_object(object, dt)
        }
    }

    fn process_collisions(&mut self, dt: f64) {
        self.process_collisions_on_grid(dt);
        // self.process_collisions_bruteforce(dt);
    }

    fn process_collisions_on_grid(&mut self, dt: f64) {
        for cell in (1..self.grid.size.x - 1)
            .cartesian_product(1..self.grid.size.y - 1)
            .map(|(x, y)| Cell::new(x, y))
        {
            for &id1 in &self.grid.cells[(cell.x, cell.y)] {
                for cell in (cell.x - 1..=cell.x + 1)
                    .cartesian_product(cell.y - 1..=cell.y + 1)
                    .map(|(x, y)| Cell::new(x, y))
                {
                    for &id2 in &self.grid.cells[(cell.x, cell.y)] {
                        if id1 != id2 {
                            let [o1, o2] = self.grid.objects.get_many_mut([id1, id2]).expect("out of bounds");
                            if intersects(o1, o2) {
                                collide_elastic(o1, o2, dt);
                            };
                        }
                    }
                }
            }
        }
    }

    fn process_collisions_bruteforce(&mut self, dt: f64) {
        for id1 in 0..self.grid.objects.len() {
            for id2 in id1 + 1..self.grid.objects.len() {
                if id1 != id2 {
                    let [o1, o2] = self.grid.objects.get_many_mut([id1, id2]).expect("out of bounds");
                    if intersects(o1, o2) {
                        collide_elastic(o1, o2, dt);
                    };
                }
            }
        }
    }

    pub fn time(&self) -> f64 {
        self.time
    }
}

fn update_object(object: &mut Object, dt: f64) {
    object.velocity += object.acceleration * dt;
    object.position += object.velocity * dt;
}

fn intersects(o1: &Object, o2: &Object) -> bool {
    let d = o1.position - o2.position;
    let r = o1.radius + o2.radius;
    d.magnitude() < r
}

fn collide_elastic(object1: &mut Object, object2: &mut Object, dt: f64) {
    let [Object {
        position: c1,
        velocity: v1,
        mass: m1,
        ..
    }, Object {
        position: c2,
        velocity: v2,
        mass: m2,
        ..
    }] = [*object1, *object2];
    object1.velocity =
        v1 - ((c1 - c2).dot(v1 - v2) * (c1 - c2) * 2.0 * m2) * (1.0 / ((m1 + m2) * (c1 - c2).magnitude2()));
    object2.velocity =
        v2 - ((c2 - c1).dot(v2 - v1) * (c2 - c1) * 2.0 * m1) * (1.0 / ((m1 + m2) * (c2 - c1).magnitude2()));
    let coeff_m1 = m1 / (m1 + m2);
    let coeff_m2 = m2 / (m1 + m2);
    object1.position += (c1 - c2) * coeff_m1 * dt;
    object2.position -= (c1 - c2) * coeff_m2 * dt;
}
