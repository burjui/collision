#![allow(unused)]

use num_traits::Signed;
use rand::random;
use serde_derive::Deserialize;
use vello::peniko::{
    Color,
    color::{ColorSpace, Hsl, Srgb, palette::css},
};

use crate::{
    app_config::CONFIG,
    object::{ObjectPrototype, ObjectSoa},
    physics::PhysicsEngine,
    vector2::Vector2,
};

pub fn create_demo(objects: &mut ObjectSoa) {
    if CONFIG.demo.enable_planets {
        objects.add(ObjectPrototype {
            velocity: Vector2::new(-700.0, 0.0),
            radius: CONFIG.demo.object_radius,
            mass: 10000.0,
            // color: Some(css::MAGENTA),
            is_planet: true,
            ..ObjectPrototype::new(Vector2::new(700.0, 500.0))
        });

        objects.add(ObjectPrototype {
            velocity: Vector2::new(700.0, 0.0),
            radius: CONFIG.demo.object_radius,
            mass: 10000.0,
            // color: Some(css::YELLOW),
            is_planet: true,
            ..ObjectPrototype::new(Vector2::new(700.0, 600.0))
        });
    }

    for brick in &CONFIG.demo.bricks {
        generate_brick(objects, brick);
    }

    for ball in &CONFIG.demo.balls {
        generate_ball(objects, ball);
    }
}

#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct Brick {
    pub position: Vector2<f32>,
    pub size: Vector2<f32>,
    #[serde(default)]
    pub velocity: Vector2<f32>,
    #[serde(default)]
    pub particle_radius: f32,
    #[serde(default)]
    pub particle_spacing: f32,
    #[serde(default)]
    pub particle_mass: f32,
}

pub fn generate_brick(objects: &mut ObjectSoa, brick: &Brick) -> Vec<usize> {
    let cell_size = brick.particle_radius * 2.0 + brick.particle_spacing;
    let dims = Vector2::new((brick.size.x / cell_size) as usize, (brick.size.y / cell_size) as usize);
    let mut result = Vec::new();
    for i in 0..dims.x {
        for j in 0..dims.y {
            let p = |position, index| {
                position + (index + 1) as f32 * (brick.particle_radius * 2.0 + brick.particle_spacing)
            };
            let position = Vector2::new(p(brick.position.x, i), p(brick.position.y, j))
                + if CONFIG.demo.randomize_positions {
                    Vector2::new(
                        brick.particle_radius * random::<f32>() * CONFIG.demo.randomize_position_factor,
                        brick.particle_radius * random::<f32>() * CONFIG.demo.randomize_position_factor,
                    )
                } else {
                    Vector2::default()
                };
            let color = {
                let selection_factor = (position.x - brick.position.x) / brick.size.x;
                let hue = 300.0 * selection_factor;
                let hsl = [hue as f32, 100.0, 50.0];
                let rgb = Hsl::convert::<Srgb>(hsl);
                Some(Color::new([rgb[0], rgb[1], rgb[2], 1.0]))
            };
            let radius = brick.particle_radius
                + if CONFIG.demo.randomize_radii {
                    brick.particle_radius * random::<f32>() * CONFIG.demo.randomize_radius_factor
                } else {
                    0.0
                };
            let id = objects.add(ObjectPrototype {
                velocity: brick.velocity,
                radius,
                mass: brick.particle_mass,
                color,
                ..ObjectPrototype::new(position)
            });
            result.push(id);
        }
    }
    result
}

#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct Ball {
    pub position: Vector2<f32>,
    pub radius: f32,
    #[serde(default)]
    pub velocity: Vector2<f32>,
    #[serde(default)]
    pub particle_radius: f32,
    #[serde(default)]
    pub particle_spacing: f32,
    #[serde(default)]
    pub particle_mass: f32,
}

pub fn generate_ball(objects: &mut ObjectSoa, ball: &Ball) -> Vec<usize> {
    let mut result = Vec::new();
    let num_particles = (ball.radius * 2.0 / (ball.particle_radius * 2.0 + ball.particle_spacing)) as usize;
    for i in 0..num_particles {
        for j in 0..num_particles {
            let x = -ball.radius + (i as f32) * (ball.particle_radius * 2.0 + ball.particle_spacing);
            let y = -ball.radius + (j as f32) * (ball.particle_radius * 2.0 + ball.particle_spacing);
            let position = Vector2::new(x, y)
                + if CONFIG.demo.randomize_positions {
                    Vector2::new(
                        random::<f32>() * ball.particle_radius * CONFIG.demo.randomize_position_factor,
                        random::<f32>() * ball.particle_radius * CONFIG.demo.randomize_position_factor,
                    )
                } else {
                    Vector2::default()
                };
            if position.magnitude() <= ball.radius {
                let position = ball.position + position;
                let color = {
                    let hue = 300.0 * (position - ball.position).magnitude() / ball.radius;
                    let hsl = [hue as f32, 100.0, 50.0];
                    let rgb = Hsl::convert::<Srgb>(hsl);
                    Some(Color::new([rgb[0], rgb[1], rgb[2], 1.0]))
                };
                let radius = ball.particle_radius
                    + if CONFIG.demo.randomize_radii {
                        ball.particle_radius * random::<f32>() * CONFIG.demo.randomize_radius_factor
                    } else {
                        0.0
                    };
                let mut object = ObjectPrototype {
                    radius,
                    mass: ball.particle_mass,
                    color,
                    ..ObjectPrototype::new(position)
                };
                object.velocity = ball.velocity;
                let id = objects.add(object);
                result.push(id);
            }
        }
    }
    result
}
