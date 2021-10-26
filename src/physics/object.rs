use std::fmt::{Display, Formatter};

use cgmath::Vector2;

#[derive(Copy, Clone)]
pub struct Object {
    pub position: Vector2<f64>,
    pub velocity: Vector2<f64>,
    pub size: f64,
    pub mass: f64,
}

pub struct ObjectId(pub(crate) usize);

impl Display for ObjectId {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}
