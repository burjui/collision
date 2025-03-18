#![allow(unused)]

use collision::{
    compound::{generate_ball, generate_brick, Ball, Brick},
    physics::{object::ObjectPrototype, PhysicsEngine},
    vector2::Vector2,
};
use vello::peniko::color::palette::css;

pub fn create_demo(physics: &mut PhysicsEngine) {
    const RADIUS: f32 = 1.0;

    physics.add(ObjectPrototype {
        velocity: Vector2::new(-700.0, 0.0),
        radius: RADIUS,
        mass: 10000.0,
        color: Some(css::MAGENTA),
        is_planet: true,
        ..ObjectPrototype::new(Vector2::new(700.0, 500.0))
    });

    physics.add(ObjectPrototype {
        velocity: Vector2::new(700.0, 0.0),
        radius: RADIUS,
        mass: 10000.0,
        color: Some(css::YELLOW),
        is_planet: true,
        ..ObjectPrototype::new(Vector2::new(700.0, 600.0))
    });

    let brick = Brick {
        particle_radius: RADIUS,
        particle_spacing: 0.01,
        particle_mass: 0.01,
        ..Brick::new(Vector2::new(400.0, 100.0), Vector2::new(600.0, 300.0))
    };
    generate_brick(physics, &brick);

    // let ball = Ball {
    //     particle_radius: RADIUS,
    //     particle_spacing: 0.01,
    //     particle_mass: 0.01,
    //     ..Ball::new(Vector2::new(500.0, 400.0), 100.0)
    // };
    // generate_ball(physics, &ball);
}
