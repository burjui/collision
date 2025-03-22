#![allow(unused)]

use collision::{
    app_config::{self, AppConfig, CONFIG},
    compound::{Ball, Brick, generate_ball, generate_brick},
    physics::{PhysicsEngine, object::ObjectPrototype},
    vector2::Vector2,
};
use vello::peniko::color::palette::css;

pub fn create_demo(physics: &mut PhysicsEngine) {
    if CONFIG.demo.enable_planets {
        physics.add(ObjectPrototype {
            velocity: Vector2::new(-700.0, 0.0),
            radius: CONFIG.demo.object_radius,
            mass: 10000.0,
            // color: Some(css::MAGENTA),
            is_planet: true,
            ..ObjectPrototype::new(Vector2::new(700.0, 500.0))
        });

        physics.add(ObjectPrototype {
            velocity: Vector2::new(700.0, 0.0),
            radius: CONFIG.demo.object_radius,
            mass: 10000.0,
            // color: Some(css::YELLOW),
            is_planet: true,
            ..ObjectPrototype::new(Vector2::new(700.0, 600.0))
        });
    }

    if CONFIG.demo.enable_brick {
        let brick = Brick {
            particle_radius: CONFIG.demo.object_radius,
            particle_spacing: CONFIG.demo.object_spacing,
            particle_mass: 0.01,
            ..Brick::new(Vector2::new(400.0, 100.0), Vector2::new(600.0, 300.0))
        };
        generate_brick(physics, &brick);
    }

    if CONFIG.demo.enable_ball {
        let ball = Ball {
            particle_radius: CONFIG.demo.object_radius,
            particle_spacing: CONFIG.demo.object_spacing,
            particle_mass: 0.01,
            ..Ball::new(Vector2::new(400.0, 600.0), 100.0)
        };
        generate_ball(physics, &ball);
    }
}
