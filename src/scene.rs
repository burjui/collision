use cgmath::Vector2;

use crate::physics::object::Object;
use crate::physics::CollisionDetector;

macro_rules! emitted_scene_path {
    () => {
        "emitted_scene.rs"
    };
}

pub fn create_scene(collision_detector: &mut CollisionDetector) {
    const SIZE: f64 = 2.0;
    const SPACING: f64 = 3.0;

    for i in 0..30 {
        for j in 0..30 {
            collision_detector.add(Object {
                position: Vector2::new(
                    140.0 + (i + 1) as f64 * (SIZE + SPACING),
                    140.0 + (j + 1) as f64 * (SIZE + SPACING),
                ),
                velocity: Vector2::new(0.0, 0.0),
                size: SIZE,
                mass: 1.0,
            });
        }
    }

    // include!(concat!("../", emitted_scene_path!()));
}
