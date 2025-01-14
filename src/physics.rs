use core::iter::Iterator;

use cgmath::{InnerSpace, Vector2};
use object::{Object, ObjectId};

mod grid;
pub mod object;

pub struct CollisionDetector {
    objects: Vec<Object>,
    time: f64,
}

impl CollisionDetector {
    pub fn new() -> Self {
        Self {
            objects: Vec::new(),
            time: 0.0,
        }
    }

    pub fn add(&mut self, object: Object) -> ObjectId {
        let id = ObjectId(self.objects.len());
        self.objects.push(object);
        id
    }

    pub fn objects(&self) -> impl Iterator<Item = (ObjectId, Object)> + '_ {
        self.objects
            .iter()
            .enumerate()
            .map(|(id, object)| (ObjectId(id), *object))
    }

    pub fn object_mut(&mut self, id: ObjectId) -> &mut Object {
        &mut self.objects[id.0]
    }

    pub fn object_count(&self) -> usize {
        self.objects.len()
    }

    pub fn advance(&mut self, dt: f64) {
        self.update_substeps(dt, 8);
    }

    fn update_substeps(&mut self, dt: f64, substeps: usize) {
        let dt_substep = dt / substeps as f64;
        (0..substeps).for_each(|_| self.update(dt_substep));
    }

    fn update(&mut self, dt: f64) {
        self.time += dt;
        self.process_collisions();
        self.update_objects(dt);
    }

    fn update_objects(&mut self, dt: f64) {
        self.objects.iter_mut().for_each(|object| update_object(object, dt));
    }

    fn process_collisions(&mut self) {
        for id1 in 0..self.objects.len() {
            for id2 in id1 + 1..self.objects.len() {
                if id1 != id2 {
                    let [o1, o2] = self.objects.get_many_mut([id1, id2]).expect("out of bounds or overlap");
                    if intersects(o1, o2) {
                        self.collide_elastic(id1, id2)
                    };
                }
            }
        }
    }
    fn collide_elastic(&mut self, id1: usize, id2: usize) {
        let [object1, object2] = self.objects.get_many_mut([id1, id2]).unwrap();
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
    }

    pub fn time(&self) -> f64 {
        self.time
    }
}

fn update_object(object: &mut Object, dt: f64) {
    object.position += object.velocity * dt + object.acceleration * (dt * dt);
    object.acceleration = Vector2::new(0.0, 0.0);
}

fn intersects(o1: &Object, o2: &Object) -> bool {
    let d = o1.position - o2.position;
    let r = o1.radius + o2.radius;
    d.magnitude() < r
}
