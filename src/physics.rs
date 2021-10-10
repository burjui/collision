use crate::physics::permutation::UniquePermutation2;
use crate::physics::timeline::Timeline;
use cgmath::{InnerSpace, Vector2};
use core::fmt::{Display, Formatter};
use core::iter::Iterator;
use core::option::Option;
use core::option::Option::{None, Some};
use log::debug;
use ndarray::{Array2, Axis};
use slab::Slab;
use timeline::EventKind;

mod permutation;
mod timeline;

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

pub struct CollisionDetector {
    objects: Slab<Object>,
    grid: Array2<Vec<usize>>,
    grid_position: Option<Vector2<f64>>,
    cell_size: f64,
    timeline: Timeline,
}

impl CollisionDetector {
    pub fn new() -> Self {
        Self {
            objects: Slab::new(),
            grid: Array2::from_elem((0, 0), Vec::new()),
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
        Vector2::new(self.grid.len_of(Axis(0)), self.grid.len_of(Axis(1)))
    }

    pub fn advance(&mut self, mut dt: f64) {
        self.update_grid();
        self.scan_grid_for_collisions();

        while let Some((event, dt_leftover)) = self.timeline.advance(dt) {
            if dt <= 0.0 {
                break;
            }

            let event_dt = dt - dt_leftover;
            dt = dt_leftover;
            self.advance_objects(event_dt);
            self.update_grid();
            self.scan_grid_for_collisions();

            match event.kind {
                EventKind::Collision => {
                    self.collide(event.id1, event.id2);
                    self.update_grid();
                    self.scan_grid_for_collisions();
                }

                EventKind::Separation => self.separate(event.id1, event.id2),
            }
        }

        if dt >= 0.0 {
            self.advance_objects(dt);
            self.update_grid();
            self.scan_grid_for_collisions();
        }

        // let excess_collisions = self
        //     .timeline
        //     .events
        //     .iter()
        //     .filter(|event| {
        //         matches!(event.kind, EventKind::Collision) && event.time < self.timeline.time
        //     })
        //     .count();
        // let excess_separations = self
        //     .timeline
        //     .events
        //     .iter()
        //     .filter(|event| {
        //         matches!(event.kind, EventKind::Separation) && event.time < self.timeline.time
        //     })
        //     .count();
        // if excess_collisions + excess_separations > 0 {
        //     warn!(
        //         "{} excess events ({} collisions, {} separations)",
        //         excess_collisions + excess_separations,
        //         excess_collisions,
        //         excess_separations
        //     );
        // }
    }

    pub fn time(&self) -> f64 {
        self.timeline.time()
    }

    pub fn object_mut(&mut self, id: ObjectId) -> &mut Object {
        &mut self.objects[id.0]
    }

    fn update_grid(&mut self) {
        debug!("UPDATE GRID");
        self.grid = Array2::from_elem((0, 0), Vec::new());

        self.grid_position = None;
        let mut x_end = f64::MIN;
        let mut y_end = f64::MIN;
        for (_, object) in &self.objects {
            self.cell_size = self.cell_size.max(object.size * 2.0);
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
            x_end = x_end.max(object.position.x + object.size);
            y_end = y_end.max(object.position.y + object.size);
            self.grid_position = Some(Vector2::new(grid_position_x, grid_position_y));
        }

        if let Some(grid_position) = &self.grid_position {
            let width = ((x_end - grid_position.x) / self.cell_size).ceil() as usize;
            let height = ((y_end - grid_position.y) / self.cell_size).ceil() as usize;
            self.grid = Array2::from_elem((width, height), Vec::new());

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
                    let cellmates = &mut self.grid[cell];
                    if let Err(index) = cellmates.binary_search(&id) {
                        cellmates.insert(index, id);
                    }
                }
            }
        }
    }

    fn scan_grid_for_collisions(&mut self) {
        debug!("SCAN GRID");

        for ids in self.grid.iter() {
            if ids.len() >= 2 {
                for [id1, id2] in UniquePermutation2::new(ids) {
                    if let Some(delay) = self.calculate_event_delay(id1, id2, EventKind::Collision)
                    {
                        let collision_time = self.timeline.time() + delay;
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
    }

    fn advance_objects(&mut self, dt: f64) {
        for (_, object) in &mut self.objects {
            object.position += object.velocity * dt;
        }
    }

    fn collide(&mut self, id1: usize, id2: usize) {
        debug!("COLLIDE {} {} (now {})", id1, id2, self.timeline.time());
        self.collision_elastic(id1, id2);

        // After collision, velocities change, making further events invalid
        self.timeline
            .remove_events(|event| event.contains(id1) || event.contains(id2));

        // Separation might not happen if objects didn't actually intersect at the time of collision
        if let Some(delay) = self.calculate_event_delay(id1, id2, EventKind::Separation) {
            let time = self.timeline.time() + delay;
            self.timeline
                .add_event(EventKind::Separation, time, id1, id2);
        }
    }

    fn separate(&mut self, id1: usize, id2: usize) {
        debug!("SEPARATE {} {} (now {})", id1, id2, self.timeline.time());
        self.timeline.remove_events(|event| {
            matches!(event.kind, EventKind::Separation)
                && event.contains(id1)
                && event.contains(id2)
        });
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
