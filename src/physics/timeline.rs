use core::option::Option;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use super::object::ObjectId;

#[derive(Default)]
pub struct Timeline {
    time: f64,
    events: BinaryHeap<Event>,
    events_by_id: HashMap<ObjectId, Vec<Event>>,
}

impl Timeline {
    pub fn add_event(&mut self, kind: EventKind, time: f64, id1: ObjectId, id2: ObjectId) {
        let event = Event {
            time,
            kind,
            id1,
            id2,
        };
        self.events.push(event);
        self.events_by_id.entry(id1).or_default().push(event);
    }

    pub fn len(&self) -> usize {
        self.events.len()
    }

    pub fn time(&self) -> f64 {
        self.time
    }

    pub fn advance(&mut self, dt: f64) -> Option<(Event, f64)> {
        assert!(dt >= 0.0);

        let final_time = self.time + dt;
        self.events
            .peek()
            .filter(|event| event.time < final_time)
            .map(|event| {
                self.time = event.time + f64::EPSILON;
                Some((*event, final_time - event.time))
            })
            .unwrap_or_else(|| {
                self.time += dt;
                None
            })
    }

    pub fn remove_events(&mut self, mut pred: impl FnMut(&Event) -> bool) {
        self.events.retain(|event| !pred(event));
        for events in self.events_by_id.values_mut() {
            events.retain(|event| !pred(event));
        }
    }

    pub fn object_events(&self, id: ObjectId) -> Option<&[Event]> {
        self.events_by_id.get(&id).map(Vec::as_slice)
    }
}

#[derive(Copy, Clone)]
pub struct Event {
    pub kind: EventKind,
    pub time: f64,
    pub id1: ObjectId,
    pub id2: ObjectId,
}

impl Event {
    pub fn contains(&self, id: ObjectId) -> bool {
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
        Some(self.cmp(other))
    }
}

impl Eq for Event {}

impl PartialEq for Event {
    fn eq(&self, other: &Self) -> bool {
        self.time == other.time
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum EventKind {
    Collision,
    Separation,
}
