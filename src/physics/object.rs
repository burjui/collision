use nalgebra::Vector2;
use sdl2::pixels::Color;

#[derive(Copy, Clone)]
pub struct Object {
    pub previous_position: Vector2<f64>,
    pub position: Vector2<f64>,
    pub acceleration: Vector2<f64>,
    pub radius: f64,
    pub mass: f64,
    pub color: Option<Color>,
    pub is_planet: bool,
}

impl Object {
    pub fn new(position: Vector2<f64>) -> Self {
        Self {
            previous_position: position,
            position,
            acceleration: Vector2::new(0.0, 0.0),
            radius: 1.0,
            mass: 1.0,
            color: None,
            is_planet: false,
        }
    }

    pub fn velocity(&self) -> Vector2<f64> {
        self.position - self.previous_position
    }

    pub fn set_velocity(&mut self, velocity: Vector2<f64>) {
        self.previous_position = self.position - velocity;
    }
}
