use collision::{
    compound::{generate_ball, generate_brick, Ball, Brick},
    physics::PhysicsEngine,
};
use nalgebra::Vector2;
use sdl2::pixels::Color;

macro_rules! emitted_scene_path {
    () => {
        "emitted_scene.rs"
    };
}

pub fn create_scene(physics: &mut PhysicsEngine) {
    let brick = Brick {
        ..Brick::new(Vector2::new(400.0, 200.0), Vector2::new(300.0, 300.0))
    };
    // brick.particle_radius = 10.0;
    generate_brick(physics, brick);

    // let mut ball = Object {
    //     radius: 20.0,
    //     mass: 100.0,
    //     ..Object::new(Vector2::new(800.0, 300.0))
    // };
    // ball.set_velocity(Vector2::new(-3.0, 0.0), 1.0);
    // physics.add(ball);

    let ball = Ball {
        velocity: Vector2::new(-3.0, 0.0),
        color: Some(Color::WHITE),
        ..Ball::new(Vector2::new(1000.0, 300.0), 20.0)
    };
    generate_ball(physics, ball);

    // let mut x_offset = false;
    // for y in 0..10 {
    //     for x in 0..10 {
    //         let position = Vector2::new(300.0 + x as f64 * 10.0 + x_offset as u8 as f64, 500.0 + y as f64 * 10.0);
    //         physics.add(Object {
    //             radius: 1.0,
    //             ..Object::new(position)
    //         });
    //     }
    //     x_offset = !x_offset;
    // }

    // include!(concat!("../", emitted_scene_path!()));
}
