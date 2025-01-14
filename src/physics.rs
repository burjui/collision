use core::{
    iter::Iterator,
    option::{
        Option,
        Option::{None, Some},
    },
};

use cgmath::{InnerSpace, Vector2};
use log::debug;
use object::Object;
use timeline::EventKind;

use crate::physics::{
    grid::{Grid, GridBuilder},
    permutation::UniquePermutation2,
    timeline::Timeline,
};

mod grid;
pub mod object;
mod permutation;
mod timeline;

pub struct CollisionDetector {
    pub grid: Grid,
    pub timeline: Timeline,
}

impl CollisionDetector {
    pub fn new() -> Self {
        Self {
            grid: Grid::default(),
            timeline: Timeline::default(),
        }
    }

    pub fn add(&mut self, object: Object) -> usize {
        let index = self.grid.objects.len();
        self.grid.objects.push(object);
        index
    }

    pub fn grid_position(&self) -> Option<Vector2<f64>> {
        if self.grid.is_empty() {
            None
        } else {
            Some(self.grid.position)
        }
    }

    pub fn grid_cell_size(&self) -> f64 {
        self.grid.cell_size
    }

    pub fn grid_size(&self) -> Vector2<usize> {
        self.grid.size
    }

    pub fn advance(&mut self, mut dt: f64) {
        while let Some((event, dt_leftover)) = self.timeline.advance(dt) {
            if dt <= 0.0 {
                break;
            }

            let event_dt = dt - dt_leftover;
            dt = dt_leftover;
            self.advance_objects(event_dt);

            match event.kind {
                EventKind::Collision => {
                    self.collide(event.id1, event.id2);
                    self.calculate_collisions();
                }

                EventKind::Separation => self.separate(event.id1, event.id2),
            }
        }

        if dt >= 0.0 {
            self.advance_objects(dt);
            self.calculate_collisions();
        }
    }

    pub fn time(&self) -> f64 {
        self.timeline.time()
    }

    #[must_use]
    fn build_grid(&mut self) -> Grid {
        let mut grid_builder = GridBuilder::default();
        for object in self.grid.objects.iter() {
            grid_builder.add_object(*object);
        }
        grid_builder.build()
    }

    pub fn calculate_collisions(&mut self) {
        debug!("Scanning for collisions (now {})", self.timeline.time());

        self.grid = self.build_grid();
        for cell in &self.grid.cells {
            Self::calculate_collisions_in_cell(
                cell.as_slice(),
                &mut self.grid.objects,
                &mut self.timeline,
            );
        }

        debug!("{} events left", self.timeline.len());
    }

    pub fn calculate_collisions_in_cell(
        cell: &[usize],
        objects: &mut [Object],
        timeline: &mut Timeline,
    ) {
        if cell.len() >= 2 {
            for (id1, id2) in UniquePermutation2::new(cell) {
                if id1 != id2 {
                    let [o1, o2] = objects.get_many_mut([id1, id2]).unwrap();
                    if let Some(delay) = Self::calculate_event_delay(o1, o2, EventKind::Collision) {
                        let collision_time = timeline.time() + delay;
                        let mut interesting_events = timeline
                            .object_events(id1)
                            .into_iter()
                            .flatten()
                            .chain(timeline.object_events(id2).into_iter().flatten());
                        debug!("{} interesting events", interesting_events.clone().count());
                        let collision_matters = !interesting_events.any(|event| {
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
                            timeline
                                .remove_events(|event| event.contains(id1) && event.contains(id2));
                            timeline.add_event(EventKind::Collision, collision_time, id1, id2);
                        }
                    }
                }
            }
        }
    }

    fn advance_objects(&mut self, dt: f64) {
        for object in &mut self.grid.objects {
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
        let (o1, o2) = (&self.grid.objects[id1], &self.grid.objects[id2]);
        if let Some(delay) = Self::calculate_event_delay(o1, o2, EventKind::Separation) {
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
        let [object1, object2] = self.grid.objects.get_many_mut([id1, id2]).unwrap();
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
        object1.velocity = v1
            - ((c1 - c2).dot(v1 - v2) * (c1 - c2) * 2.0 * m2)
                * (1.0 / ((m1 + m2) * (c1 - c2).magnitude2()));
        object2.velocity = v2
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
    fn calculate_event_delay(o1: &Object, o2: &Object, kind: EventKind) -> Option<f64> {
        // a = |vy21|^2
        // b = 2*(p21.y*v21.y + p21.x*v21.x)
        // c = |p21|^2 - R^2
        // discriminant = b^2 - 4ac
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
