use crate::physics::{CollisionDetector, Object};
use cgmath::Vector2;

macro_rules! emitted_scene_path {
    () => {
        "emitted_scene.rs"
    };
}

pub fn create_scene(collision_detector: &mut CollisionDetector) {
    for i in 0..40 {
        for j in 0..40 {
            collision_detector.add(Object {
                position: Vector2::new(140.0 + (i + 1) as f64 * 5.0, 140.0 + (j + 1) as f64 * 5.0),
                velocity: Vector2::new(0.0, 0.0),
                size: 2.0,
                mass: 1.0,
            });
        }
    }

    // include!(concat!("../", emitted_scene_path!()));
}
