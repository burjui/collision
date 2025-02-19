use sdl2::pixels::Color;

use crate::vector2::Vector2;

#[derive(Copy, Clone)]
pub struct Object {
    pub position: Vector2<f64>,
    pub velocity: Vector2<f64>,
    pub acceleration: Vector2<f64>,
    pub radius: f64,
    pub mass: f64,
    pub color: Option<Color>,
    pub is_planet: bool,
}

impl Object {
    pub fn new(position: Vector2<f64>) -> Self {
        Self {
            position,
            velocity: Vector2::new(0.0, 0.0),
            acceleration: Vector2::new(0.0, 0.0),
            radius: 1.0,
            mass: 1.0,
            color: None,
            is_planet: false,
        }
    }

    pub fn momentum(&self) -> Vector2<f64> {
        self.velocity * self.mass
    }
}
