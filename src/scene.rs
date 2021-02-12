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
        });
    }
}

#[allow(unused)]
pub fn create_saved_scene(collision_detector: &mut CollisionDetector) {
    collision_detector.add(PhysicsObject {
        position: Vector2::new(453.0, 474.0),
        velocity: Vector2::new(4.250103, -10.279476),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(455.0, 478.0),
        velocity: Vector2::new(9.834652, -4.284231),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(472.0, 476.0),
        velocity: Vector2::new(13.544556, -8.32799),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(460.0, 471.0),
        velocity: Vector2::new(7.173977, -7.680192),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(473.0, 483.0),
        velocity: Vector2::new(-3.8203716, -1.6473503),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(488.0, 492.0),
        velocity: Vector2::new(-3.4370804, -5.024991),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(480.0, 478.0),
        velocity: Vector2::new(3.09766, -9.488573),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(453.0, 457.0),
        velocity: Vector2::new(-12.218943, 12.65044),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(480.0, 450.0),
        velocity: Vector2::new(3.1543903, 16.371914),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(489.0, 454.0),
        velocity: Vector2::new(-11.260815, -18.9305),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(460.0, 485.0),
        velocity: Vector2::new(15.65139, 18.090874),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(453.0, 488.0),
        velocity: Vector2::new(6.7373505, -10.772245),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(486.0, 490.0),
        velocity: Vector2::new(-15.183063, 13.747635),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(479.0, 495.0),
        velocity: Vector2::new(5.9337254, 19.499897),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(461.0, 455.0),
        velocity: Vector2::new(7.3875046, -4.0563602),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(467.0, 497.0),
        velocity: Vector2::new(-7.863434, -9.902201),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(463.0, 452.0),
        velocity: Vector2::new(12.899441, -17.885077),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(474.0, 472.0),
        velocity: Vector2::new(9.6737, -15.843685),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(463.0, 468.0),
        velocity: Vector2::new(11.926693, -5.055807),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(490.0, 468.0),
        velocity: Vector2::new(18.778286, -11.720507),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(486.0, 485.0),
        velocity: Vector2::new(-7.509947, 12.107807),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(485.0, 491.0),
        velocity: Vector2::new(-10.86338, 13.726429),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(489.0, 478.0),
        velocity: Vector2::new(17.729942, 4.709627),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(485.0, 482.0),
        velocity: Vector2::new(-3.3646507, 7.845066),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(480.0, 458.0),
        velocity: Vector2::new(11.430685, -15.424358),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(453.0, 498.0),
        velocity: Vector2::new(-8.530822, 1.3864155),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(451.0, 460.0),
        velocity: Vector2::new(-2.618887, 5.5452633),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(466.0, 457.0),
        velocity: Vector2::new(9.994814, 8.984386),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(474.0, 454.0),
        velocity: Vector2::new(13.662094, -12.2027445),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(493.0, 495.0),
        velocity: Vector2::new(4.5266953, 0.56235504),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(461.0, 471.0),
        velocity: Vector2::new(-12.591691, -5.216861),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(471.0, 470.0),
        velocity: Vector2::new(5.10215, -3.6250954),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(480.0, 478.0),
        velocity: Vector2::new(8.362923, -19.381348),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(451.0, 457.0),
        velocity: Vector2::new(12.327133, -17.83434),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(452.0, 493.0),
        velocity: Vector2::new(-10.978277, 9.632763),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(492.0, 453.0),
        velocity: Vector2::new(-19.134848, 12.904766),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(450.0, 467.0),
        velocity: Vector2::new(15.472675, -4.0339355),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(464.0, 466.0),
        velocity: Vector2::new(17.490017, -15.565664),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(474.0, 485.0),
        velocity: Vector2::new(18.28938, 18.697922),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(471.0, 463.0),
        velocity: Vector2::new(-9.589384, 12.755695),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(476.0, 474.0),
        velocity: Vector2::new(-16.727367, -3.7662315),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(493.0, 477.0),
        velocity: Vector2::new(10.335613, -3.4423447),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(498.0, 492.0),
        velocity: Vector2::new(-7.006443, -10.496452),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(486.0, 462.0),
        velocity: Vector2::new(3.247263, -1.9140072),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(479.0, 462.0),
        velocity: Vector2::new(4.8627205, -4.621689),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(481.0, 470.0),
        velocity: Vector2::new(8.960066, 6.743202),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(460.0, 476.0),
        velocity: Vector2::new(-8.607969, 5.140232),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(470.0, 495.0),
        velocity: Vector2::new(-7.151904, -15.988991),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(478.0, 480.0),
        velocity: Vector2::new(-13.537664, 6.0652885),
        size: 15.0,
    });

    collision_detector.add(PhysicsObject {
        position: Vector2::new(498.0, 455.0),
        velocity: Vector2::new(19.32425, -16.356705),
        size: 15.0,
    });
}
