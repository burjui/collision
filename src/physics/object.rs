use std::fmt::{Display, Formatter};

use nalgebra::Vector2;

#[derive(Copy, Clone)]
pub struct Object {
    pub position: Vector2<f64>,
    pub velocity: Vector2<f64>,
    pub acceleration: Vector2<f64>,
    pub radius: f64,
    pub mass: f64,
}

impl Default for Object {
    fn default() -> Self {
        Self {
            velocity: Vector2::new(0.0, 0.0),
            position: Vector2::new(0.0, 0.0),
            acceleration: Vector2::new(0.0, 0.0),
            radius: 1.0,
            mass: 1.0,
        }
    }
}

//TODO get rid of indirection and ObjectId
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
pub struct ObjectId(pub(crate) usize);

impl Display for ObjectId {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}
