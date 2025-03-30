#![allow(unused)]

use num_traits::Signed;
use rand::random;
use serde_derive::Deserialize;
use vello::peniko::{
    Color,
    color::{ColorSpace, Hsl, Srgb, palette::css},
};

use crate::{
    app_config::{self, AppConfig, CONFIG},
    physics::{PhysicsEngine, object::ObjectPrototype},
    vector2::Vector2,
};

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

    for brick in &CONFIG.demo.bricks {
        let particle_radius = if brick.particle_radius > 0.0 {
            brick.particle_radius
        } else {
            CONFIG.demo.object_radius
        };
        let particle_spacing = if brick.particle_spacing >= 0.0 {
            brick.particle_spacing
        } else {
            CONFIG.demo.object_spacing
        };
        generate_brick(
            physics,
            &Brick {
                particle_radius,
                particle_spacing,
                ..Brick::new(brick.position, brick.size)
            },
        );
    }

    for ball in &CONFIG.demo.balls {
        let particle_radius = if ball.particle_radius > 0.0 {
            ball.particle_radius
        } else {
            CONFIG.demo.object_radius
        };
        let particle_spacing = if ball.particle_spacing >= 0.0 {
            ball.particle_spacing
        } else {
            CONFIG.demo.object_spacing
        };
        let ball = Ball {
            particle_radius,
            particle_spacing,
            ..Ball::new(ball.position, ball.radius)
        };
        generate_ball(physics, &ball);
    }
}

const DEFAULT_PARTICLE_RADIUS: f64 = 2.0;
const DEFAULT_PARTICLE_SPACING: f64 = 2.0;
const DEFAULT_PARTICLE_MASS: f64 = 1.0;

#[derive(Deserialize, Clone, Copy)]
pub struct Brick {
    pub position: Vector2<f64>,
    pub size: Vector2<f64>,
    #[serde(default)]
    pub velocity: Vector2<f64>,
    #[serde(default)]
    pub particle_radius: f64,
    #[serde(default = "default_spacing")]
    pub particle_spacing: f64,
    #[serde(default)]
    pub particle_mass: f64,
}

impl Brick {
    #[must_use]
    pub fn new(position: Vector2<f64>, size: Vector2<f64>) -> Self {
        Self {
            position,
            size,
            velocity: Vector2::new(0.0, 0.0),
            particle_radius: DEFAULT_PARTICLE_RADIUS,
            particle_spacing: DEFAULT_PARTICLE_SPACING,
            particle_mass: DEFAULT_PARTICLE_MASS,
        }
    }
}

pub fn generate_brick(physics: &mut PhysicsEngine, brick: &Brick) -> Vec<usize> {
    let cell_size = brick.particle_radius * 2.0 + brick.particle_spacing;
    let dims = Vector2::new((brick.size.x / cell_size) as usize, (brick.size.y / cell_size) as usize);
    let mut result = Vec::new();
    for i in 0..dims.x {
        for j in 0..dims.y {
            let p = |position, index| {
                position + (index + 1) as f64 * (brick.particle_radius * 2.0 + brick.particle_spacing)
            };
            let position = Vector2::new(p(brick.position.x, i), p(brick.position.y, j))
                + if CONFIG.demo.randomize_positions {
                    Vector2::new(
                        random::<f64>() * brick.particle_radius * CONFIG.demo.randomize_position_factor,
                        random::<f64>() * brick.particle_radius * CONFIG.demo.randomize_position_factor,
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
            let id = physics.add(ObjectPrototype {
                velocity: brick.velocity,
                radius: brick.particle_radius,
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
pub struct Ball {
    pub position: Vector2<f64>,
    pub radius: f64,
    #[serde(default)]
    pub velocity: Vector2<f64>,
    #[serde(default)]
    pub particle_radius: f64,
    #[serde(default = "default_spacing")]
    pub particle_spacing: f64,
    #[serde(default)]
    pub particle_mass: f64,
}

impl Ball {
    #[must_use]
    pub fn new(position: Vector2<f64>, radius: f64) -> Self {
        Self {
            position,
            radius,
            velocity: Vector2::new(0.0, 0.0),
            particle_radius: DEFAULT_PARTICLE_RADIUS,
            particle_spacing: DEFAULT_PARTICLE_SPACING,
            particle_mass: DEFAULT_PARTICLE_MASS,
        }
    }
}

pub fn generate_ball(physics: &mut PhysicsEngine, ball: &Ball) -> Vec<usize> {
    let mut result = Vec::new();
    let num_particles = (ball.radius * 2.0 / (ball.particle_radius * 2.0 + ball.particle_spacing)) as usize;
    for i in 0..num_particles {
        for j in 0..num_particles {
            let x = -ball.radius + (i as f64) * (ball.particle_radius * 2.0 + ball.particle_spacing);
            let y = -ball.radius + (j as f64) * (ball.particle_radius * 2.0 + ball.particle_spacing);
            let position = Vector2::new(x, y)
                + if CONFIG.demo.randomize_positions {
                    Vector2::new(
                        random::<f64>() * ball.particle_radius * CONFIG.demo.randomize_position_factor,
                        random::<f64>() * ball.particle_radius * CONFIG.demo.randomize_position_factor,
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
                let mut object = ObjectPrototype {
                    radius: ball.particle_radius,
                    mass: ball.particle_mass,
                    color,
                    ..ObjectPrototype::new(position)
                };
                object.velocity = ball.velocity;
                let id = physics.add(object);
                result.push(id);
            }
        }
    }
    result
}

fn default_spacing() -> f64 {
    -0.0
}
