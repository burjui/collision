use cgmath::{InnerSpace, Vector2};
use core::fmt::{Display, Formatter};
use core::iter::Iterator;
use core::option::Option;
use core::option::Option::{None, Some};
use itertools::Itertools;
use slab::Slab;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

#[derive(Copy, Clone)]
pub struct PhysicsObject {
    pub position: Vector2<f32>,
    pub velocity: Vector2<f32>,
    pub size: f32,
    pub mass: f32,
}

pub struct ObjectId(usize);

impl Display for ObjectId {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Ord, PartialOrd, Eq, PartialEq, Copy, Clone)]
enum EventKind {
    Collision,
    Separation,
}

#[derive(Copy, Clone)]
struct Event {
    time: f32,
    kind: EventKind,
    id1: usize,
    id2: usize,
}

impl Ord for Event {
    fn cmp(&self, other: &Self) -> Ordering {
        other.time.partial_cmp(&self.time).unwrap()
    }
}

impl PartialOrd for Event {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.time.partial_cmp(&self.time)
    }
}

impl Eq for Event {}

impl PartialEq for Event {
    fn eq(&self, other: &Self) -> bool {
        self.time == other.time
    }
}

pub struct CollisionDetector {
    objects: Slab<PhysicsObject>,
    grid: HashMap<(usize, usize), HashSet<usize>>,
    grid_position: Option<Vector2<f32>>,
    cell_size: f32,
    time: f32,
    events: BinaryHeap<Event>,
    collisions: HashMap<usize, HashSet<usize>>,
}

impl CollisionDetector {
    pub fn new() -> Self {
        Self {
            objects: Slab::new(),
            grid: HashMap::new(),
            grid_position: None,
            cell_size: 0.0,
            time: 0.0,
            events: BinaryHeap::new(),
            collisions: HashMap::new(),
        }
    }

    pub fn add(&mut self, object: PhysicsObject) -> ObjectId {
        ObjectId(self.objects.insert(object))
    }

    pub fn objects(&self) -> impl Iterator<Item = (ObjectId, PhysicsObject)> + '_ {
        self.objects
            .iter()
            .map(|(id, object)| (ObjectId(id), *object))
    }

    pub fn grid_position(&self) -> Option<Vector2<f32>> {
        self.grid_position.clone()
    }

    pub fn grid_cell_size(&self) -> f32 {
        self.cell_size
    }

    pub fn grid_size(&self) -> Vector2<usize> {
        self.grid
            .keys()
            .next()
            .cloned()
            .map(|(first_x, first_y)| {
                let (mut min_x, mut min_y, mut max_x, mut max_y) =
                    (first_x, first_y, first_x, first_y);
                for (x, y) in self.grid.keys() {
                    min_x = min_x.min(*x);
                    min_y = min_y.min(*y);
                    max_x = max_x.max(*x);
                    max_y = max_y.max(*y);
                }
                Vector2::new(max_x - min_x + 1, max_y - min_y + 1)
            })
            .unwrap_or(Vector2::new(0, 0))
    }

    pub fn advance(&mut self, mut dt: f32) {
        self.update_grid();
        self.scan_grid_for_collisions();

        let target_time = self.time + dt;
        while let Some(event) = self.events.peek().cloned() {
            if event.time < target_time {
                self.events.pop();
                let event_dt = event.time - self.time;
                dt -= event_dt;
                self.advance_objects(event_dt);
                self.time = event.time;

                match event.kind {
                    EventKind::Collision => {
                        self.collision(event.id1, event.id2);
                    }
                    EventKind::Separation => {
                        self.separation(event.id1, event.id2);
                    }
                }
            } else {
                break;
            }
        }

        self.time += dt;
        self.advance_objects(dt);
    }

    pub fn time(&self) -> f32 {
        self.time
    }

    fn scan_grid_for_collisions(&mut self) {
        for (_, ids) in &self.grid {
            for (&id1, &id2) in ids.iter().tuple_windows() {
                if self
                    .collisions
                    .get(&id1)
                    .and_then(|c| c.get(&id2))
                    .is_none()
                {
                    let delay = calculate_event_delay(&self.objects[id1], &self.objects[id2], true);
                    if delay < f32::INFINITY {
                        self.collisions
                            .entry(id1)
                            .or_insert_with(HashSet::new)
                            .insert(id2);
                        self.collisions
                            .entry(id2)
                            .or_insert_with(HashSet::new)
                            .insert(id1);
                        self.events.push(Event {
                            time: self.time + delay,
                            kind: EventKind::Collision,
                            id1,
                            id2,
                        });
                    }
                }
            }
        }
    }

    fn collision(&mut self, id1: usize, id2: usize) {
        self.collision_elastic(id1, id2);
        let delay = calculate_event_delay(&self.objects[id1], &self.objects[id2], false);
        self.events.push(Event {
            time: self.time + delay,
            kind: EventKind::Separation,
            id1,
            id2,
        });
    }

    fn separation(&mut self, id1: usize, id2: usize) {
        self.collisions.get_mut(&id1).unwrap().remove(&id2);
        self.collisions.get_mut(&id2).unwrap().remove(&id1);
    }

    fn advance_objects(&mut self, dt: f32) {
        for (_, object) in &mut self.objects {
            object.position += object.velocity * dt;
        }
    }

    fn update_grid(&mut self) {
        self.grid.clear();

        self.grid_position = None;
        for (_, object) in &self.objects {
            self.cell_size = self.cell_size.max(object.size + 1.0);
            let x = object.position.x - object.size / 2.0;
            let y = object.position.y - object.size / 2.0;
            let grid_position_x = self
                .grid_position
                .map(|position| position.x.min(x))
                .unwrap_or(x);
            let grid_position_y = self
                .grid_position
                .map(|position| position.y.min(y))
                .unwrap_or(y);
            self.grid_position = Some(Vector2::new(grid_position_x, grid_position_y));
        }

        if let Some(grid_position) = &self.grid_position {
            for (id, object) in &self.objects {
                let half_size = object.size / 2.0;
                let corners = &[
                    object.position + Vector2::new(-half_size, -half_size),
                    object.position + Vector2::new(-half_size, half_size),
                    object.position + Vector2::new(half_size, -half_size),
                    object.position + Vector2::new(half_size, half_size),
                ];
                for point in corners {
                    let cell_x = ((point.x - grid_position.x) / self.cell_size) as usize;
                    let cell_y = ((point.y - grid_position.y) / self.cell_size) as usize;
                    let cell = (cell_x, cell_y);
                    self.grid
                        .entry(cell)
                        .or_insert_with(HashSet::new)
                        .insert(id);
                }
            }
        }
    }

    fn collision_elastic(&mut self, id1: usize, id2: usize) {
        let object1 = self.objects[id1];
        let object2 = self.objects[id2];
        let v1 = object1.velocity;
        let v2 = object2.velocity;
        let c1 = object1.position;
        let c2 = object2.position;
        let m1 = object1.mass;
        let m2 = object2.mass;
        self.objects[id1].velocity = v1
            - ((c1 - c2).dot(v1 - v2) * (c1 - c2) * 2.0 * m2)
                * (1.0 / ((m1 + m2) * (c1 - c2).magnitude2()));
        self.objects[id2].velocity = v2
            - ((c2 - c1).dot(v2 - v1) * (c2 - c1) * 2.0 * m1)
                * (1.0 / ((m1 + m2) * (c2 - c1).magnitude2()));
    }
}

// TODO respect padding
// TODO for_collide -> enum
fn calculate_event_delay(a: &PhysicsObject, b: &PhysicsObject, for_collide: bool) -> f32 {
    let sign = if for_collide { 1.0 } else { -1.0 };
    let net_rad = (a.size + b.size) * 0.5;
    let dist = a.position - b.position;
    let coeff_c = sign * (net_rad * net_rad - dist.magnitude2());
    if coeff_c > 0.0 {
        return 0.0;
    }

    let dist_vel = a.velocity - b.velocity;
    let coeff_a = sign * -dist_vel.magnitude2();
    let coeff_b = sign * 2.0 * -dist.dot(dist_vel);
    match quad_root_ascending(coeff_a, coeff_b, coeff_c) {
        Some(result) if result >= 0.0 => result,
        _ => f32::INFINITY,
    }
}

fn quad_root_ascending(a: f32, b: f32, c: f32) -> Option<f32> {
    let determinant = b * b - a * c * 4.0;
    if determinant <= 0.0 {
        None
    } else if b >= 0.0 {
        Some((c * 2.0) / (-b - determinant.sqrt()))
    } else {
        Some((-b + determinant.sqrt()) / (a * 2.0))
    }
}
