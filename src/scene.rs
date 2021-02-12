use crate::config::Config;
use crate::physics::{CollisionDetector, PhysicsObject};
use cgmath::Vector2;
use rand::{random, Rng};

#[allow(unused)]
pub fn create_random_scene(collision_detector: &mut CollisionDetector, config: &Config) {
    let mut rng = rand::thread_rng();
    for _ in 0..100 {
        collision_detector.add(PhysicsObject {
            position: Vector2::new(rng.gen_range(300.0..500.0), rng.gen_range(300.0..500.0)),
            velocity: Vector2::new(random::<f32>() * 40.0 - 20.0, random::<f32>() * 40.0 - 20.0),
            size: 15.0,
            mass: 1.0,
        });
    }
}

#[allow(unused)]
pub fn create_saved_scene(collision_detector: &mut CollisionDetector) {
    collision_detector.add(PhysicsObject {
        position: Vector2::new(300.0, 300.0),
        velocity: Vector2::new(40.0, 0.0),
        size: 15.0,
        mass: 10.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(500.0, 300.0),
        velocity: Vector2::new(-40.0, 0.0),
        size: 15.0,
        mass: 1.0,
    });
}
