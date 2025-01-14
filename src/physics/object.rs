use cgmath::Vector2;

#[derive(Copy, Clone)]
pub struct Object {
    pub position: Vector2<f64>,
    pub velocity: Vector2<f64>,
    pub size: f64,
    pub mass: f64,
}
