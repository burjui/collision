use cgmath::{InnerSpace, Vector2};
use core::fmt::{Display, Formatter};
use core::iter::Iterator;
use core::option::Option;
use core::option::Option::{None, Some};
use fxhash::{FxHashMap, FxHashSet};
use itertools::Itertools;
use log::{debug, warn};
use slab::Slab;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Copy, Clone)]
pub struct Object {
    pub position: Vector2<f64>,
    pub velocity: Vector2<f64>,
    pub size: f64,
    pub mass: f64,
}

pub struct ObjectId(usize);

impl Display for ObjectId {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum EventKind {
    Collision,
    Separation,
}

#[derive(Copy, Clone)]
struct Event {
    kind: EventKind,
    time: f64,
    id1: usize,
    id2: usize,
}

impl Event {
    fn contains(&self, id: usize) -> bool {
        self.id1 == id || self.id2 == id
    }
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

#[derive(Default)]
struct Timeline {
    time: f64,
    events: BinaryHeap<Event>,
}

impl Timeline {
    fn add_event(&mut self, kind: EventKind, time: f64, id1: usize, id2: usize) {
        self.events.push(Event {
            time,
            kind,
            id1,
            id2,
        });
    }

    fn peek_event(&self) -> Option<&Event> {
        self.events.peek()
    }

    fn pop_event(&mut self) -> Option<Event> {
        self.events.pop()
    }

    fn remove_events(&mut self, mut pred: impl FnMut(&Event) -> bool) {
        self.events.retain(|event| !pred(event))
    }

    fn contains_any_events(&self, pred: impl FnMut(&Event) -> bool) -> bool {
        self.events.iter().any(pred)
    }
}

pub struct CollisionDetector {
    objects: Slab<Object>,
    grid: FxHashMap<(usize, usize), FxHashSet<usize>>,
    grid_position: Option<Vector2<f64>>,
    cell_size: f64,
    timeline: Timeline,
}

impl CollisionDetector {
    pub fn new() -> Self {
        Self {
            objects: Slab::new(),
            grid: FxHashMap::default(),
            grid_position: None,
            cell_size: 0.0,
            timeline: Timeline::default(),
        }
    }

    pub fn add(&mut self, object: Object) -> ObjectId {
        ObjectId(self.objects.insert(object))
    }

    pub fn objects(&self) -> impl Iterator<Item = (ObjectId, Object)> + '_ {
        self.objects
            .iter()
            .map(|(id, object)| (ObjectId(id), *object))
    }

    pub fn grid_position(&self) -> Option<Vector2<f64>> {
        self.grid_position
    }

    pub fn grid_cell_size(&self) -> f64 {
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
            .unwrap_or_else(|| Vector2::new(0, 0))
    }

    pub fn advance(&mut self, mut dt: f64) {
        self.update_grid();
        self.scan_grid_for_collisions();

        let target_time = self.timeline.time + dt;
        while let Some(event) = self.timeline.peek_event().cloned() {
            if dt <= 0.0 {
                break;
            }

            if event.time < target_time {
                self.timeline.pop_event();

                let (id1, id2) = (event.id1, event.id2);
                let event_dt = event.time - self.timeline.time;
                self.timeline.time = event.time;
                dt -= event_dt;
                self.advance_objects(event_dt);

                self.update_grid();
                self.scan_grid_for_collisions();

                match event.kind {
                    EventKind::Collision => {
                        self.collide(id1, id2);
                        self.update_grid();
                        self.scan_grid_for_collisions();
                    }

                    EventKind::Separation => self.separate(id1, id2),
                }
            } else {
                break;
            }
        }

        if dt > 0.0 {
            self.timeline.time += dt;
            self.advance_objects(dt);
            self.update_grid();
            self.scan_grid_for_collisions();
        }

        let excess_collisions = self
            .timeline
            .events
            .iter()
            .filter(|event| {
                matches!(event.kind, EventKind::Collision) && event.time < self.timeline.time
            })
            .count();
        let excess_separations = self
            .timeline
            .events
            .iter()
            .filter(|event| {
                matches!(event.kind, EventKind::Separation) && event.time < self.timeline.time
            })
            .count();
        if excess_collisions + excess_separations > 0 {
            warn!(
                "{} excess events ({} collisions, {} separations)",
                excess_collisions + excess_separations,
                excess_collisions,
                excess_separations
            );
        }
    }

    pub fn time(&self) -> f64 {
        self.timeline.time
    }

    pub fn object_mut(&mut self, id: ObjectId) -> &mut Object {
        &mut self.objects[id.0]
    }

    fn collide(&mut self, id1: usize, id2: usize) {
        debug!("COLLIDE {} {} (now {})", id1, id2, self.timeline.time);
        self.collision_elastic(id1, id2);

        // After collision, velocities change, making further events invalid
        self.timeline
            .remove_events(|event| event.contains(id1) || event.contains(id2));

        // Separation might not happen if objects didn't actually intersect at the time of collision
        if let Some(delay) = self.calculate_event_delay(id1, id2, EventKind::Separation) {
            let time = self.timeline.time + delay + 0.01;
            self.timeline
                .add_event(EventKind::Separation, time, id1, id2);
        }
    }

    fn separate(&mut self, id1: usize, id2: usize) {
        debug!("SEPARATE {} {} (now {})", id1, id2, self.timeline.time);
        self.timeline.remove_events(|event| {
            matches!(event.kind, EventKind::Separation)
                && event.contains(id1)
                && event.contains(id2)
        });
    }

    fn scan_grid_for_collisions(&mut self) {
        debug!("SCAN GRID");
        for ids in self.grid.values() {
            for pair in ids
                .iter()
                .cloned()
                .permutations(2)
                .map(|mut pair| {
                    pair.sort_unstable();
                    pair
                })
                .unique()
            {
                let (id1, id2) = (pair[0], pair[1]);

                if let Some(delay) = self.calculate_event_delay(id1, id2, EventKind::Collision) {
                    let collision_time = self.timeline.time + delay;
                    let collision_matters = !self.timeline.contains_any_events(|event| {
                        let these_are_separating = || {
                            event.contains(id1)
                                && event.contains(id2)
                                && (matches!(event.kind, EventKind::Separation))
                        };

                        let either_is_colliding_earlier = || {
                            (event.contains(id1) || event.contains(id2))
                                && matches!(event.kind, EventKind::Collision)
                                && event.time <= collision_time
                        };

                        these_are_separating() || either_is_colliding_earlier()
                    });
                    if collision_matters {
                        self.timeline
                            .add_event(EventKind::Collision, collision_time, id1, id2);
                    }
                }
            }
        }
    }

    fn advance_objects(&mut self, dt: f64) {
        for (_, object) in &mut self.objects {
            object.position += object.velocity * dt;
        }
    }

    fn update_grid(&mut self) {
        debug!("UPDATE GRID");
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
                    let cell_x = ((point.x - grid_position.x) / self.cell_size).floor() as usize;
                    let cell_y = ((point.y - grid_position.y) / self.cell_size).floor() as usize;
                    let cell = (cell_x, cell_y);
                    self.grid
                        .entry(cell)
                        .or_insert_with(FxHashSet::default)
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

    // Based on the following paper:
    //
    // "Collision Detection Between two Circular Rigid Bodies
    // A comparison of two methods: Periodic Interference Test (PIT) and
    // Predicted Instance of Collision (PIC) Calculation"
    //
    // by Madhav Deshpande, Neha Kharsikar and Priyanka Prabhu
    //
    fn calculate_event_delay(&self, id1: usize, id2: usize, kind: EventKind) -> Option<f64> {
        // a = |vy21|^2
        // b = 2*(p21.y*v21.y + p21.x*v21.x)
        // c = |p21|^2 - R^2
        // discriminant = b^2 - 4ac
        let (o1, o2) = (&self.objects[id1], &self.objects[id2]);
        let coeff_sign = match kind {
            EventKind::Collision => 1.0,
            EventKind::Separation => -1.0,
        };
        let p21 = o2.position - o1.position;
        let v21 = o2.velocity - o1.velocity;
        let a = coeff_sign * v21.magnitude2();
        let b = coeff_sign * 2.0 * (p21.y * v21.y + p21.x * v21.x);
        let rr = (o1.size + o2.size) * 0.5;
        let c = coeff_sign * (p21.magnitude2() - rr * rr);
        let discriminant = b * b - 4.0 * a * c;
        if discriminant < 0.0 || a == 0.0 {
            None
        } else {
            let t1 = (-b + discriminant.sqrt()) / (2.0 * a);
            let t2 = (-b - discriminant.sqrt()) / (2.0 * a);
            let t = if t1 < 0.0 {
                t2
            } else if t2 < 0.0 {
                t1
            } else {
                t1.min(t2)
            };
            if t >= 0.0 {
                Some(t)
            } else {
                None
            }
        }
    }
}

#[allow(unused)]
mod debug_utils {

    const PROBLEMATIC_IDS: [usize; 2] = [16, 26];

    pub(crate) fn are_problematic(id1: usize, id2: usize) -> bool {
        let mut a_ids = PROBLEMATIC_IDS;
        let mut b_ids = [id1, id2];
        a_ids.sort_unstable();
        b_ids.sort_unstable();
        a_ids == b_ids
    }

    pub(crate) fn is_problematic(id: usize) -> bool {
        PROBLEMATIC_IDS.iter().any(|&pid| pid == id)
    }
}
