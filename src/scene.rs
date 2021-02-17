use crate::physics::{CollisionDetector, PhysicsObject};
use cgmath::Vector2;

pub fn create_scene(collision_detector: &mut CollisionDetector) {
    for i in 0..100 {
        for j in 0..100 {
            collision_detector.add(PhysicsObject {
                position: Vector2::new((i + 1) as f32 * 7.0, (j + 1) as f32 * 7.0),
                velocity: Vector2::new(0.0, 0.0),
                size: 2.0,
                mass: 1.0,
            });
        }
    }
}
