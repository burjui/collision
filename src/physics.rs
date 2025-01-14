use core::iter::Iterator;

use cgmath::Vector2;
use log::debug;
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
        println!("dt: {}", dt);
        self.time += dt;
        self.process_collisions();
        self.update_objects(dt);
    }

    fn update_objects(&mut self, dt: f64) {
        self.objects.iter_mut().for_each(|object| update_object(object, dt));
    }

    fn process_collisions(&mut self) {
        for id1 in (0..self.objects.len()).map(|id| ObjectId(id)) {
            for id2 in (0..self.objects.len()).map(|id| ObjectId(id)) {
                if id1 != id2 {
                    let [o1, o2] = self
                        .objects
                        .get_many_mut([id1.0, id2.0])
                        .expect("out of bounds or overlap");
                    if Self::intersect(o1, o2) {
                        debug!("COLLIDE {id1} {id2}");
                        // Self::collision_elastic(objects, id1, id2);
                        let response_coef = 1.0;
                        let v = o1.position - o2.position;
                        let dist2 = v.x * v.x + v.y * v.y;
                        let dist = dist2.sqrt();
                        let n = v / dist;
                        let o1_radius = o1.radius / 2.0;
                        let o2_radius = o2.radius / 2.0;
                        let min_dist = o1_radius + o2_radius;
                        let mass_ratio_1 = o1_radius / (o1_radius + o2_radius);
                        let mass_ratio_2 = o2_radius / (o1_radius + o2_radius);
                        let delta = 0.5 * response_coef * (dist - min_dist);
                        // Update positions
                        o1.position -= n * (mass_ratio_2 * delta);
                        o2.position += n * (mass_ratio_1 * delta);
                    }
                }
            }
        }
    }

    pub fn time(&self) -> f64 {
        self.time
    }

    fn intersect(o1: &Object, o2: &Object) -> bool {
        let d = o1.position - o2.position;
        let r = o1.radius + o2.radius;
        d.x.abs() < r && d.y.abs() < r
    }
}

fn update_object(object: &mut Object, dt: f64) {
    let displacement = object.position - object.previous_position;
    object.previous_position = object.position;
    object.position += displacement + object.acceleration * (dt * dt);
    object.acceleration = Vector2::new(0.0, 0.0);
}
