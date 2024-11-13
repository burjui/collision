use core::option::Option;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

use super::object::ObjectId;

#[derive(Default)]
pub struct Timeline {
    time: f64,
    events: BinaryHeap<Event>,
}

impl Timeline {
    pub(crate) fn add_event(&mut self, kind: EventKind, time: f64, id1: ObjectId, id2: ObjectId) {
        self.events.push(Event {
            time,
            kind,
            id1,
            id2,
        });
    }

    pub(crate) fn time(&self) -> f64 {
        self.time
    }

    pub(crate) fn advance(&mut self, dt: f64) -> Option<(Event, f64)> {
        assert!(dt >= 0.0);

        let final_time = self.time + dt;
        self.events
            .peek()
            .filter(|event| event.time < final_time)
            .map(|event| {
                self.time = event.time;
                Some((*event, final_time - event.time))
            })
            .unwrap_or_else(|| {
                self.time += dt;
                None
            })
    }

    pub(crate) fn remove_events(&mut self, mut pred: impl FnMut(&Event) -> bool) {
        self.events.retain(|event| !pred(event))
    }

    pub(crate) fn contains_any_events(&self, pred: impl FnMut(&Event) -> bool) -> bool {
        self.events.iter().any(pred)
    }
}

#[derive(Copy, Clone)]
pub(crate) struct Event {
    pub(crate) kind: EventKind,
    pub(crate) time: f64,
    pub(crate) id1: ObjectId,
    pub(crate) id2: ObjectId,
}

impl Event {
    pub(crate) fn contains(&self, id: ObjectId) -> bool {
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
