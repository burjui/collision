use nalgebra::Vector2;
use sdl2::pixels::Color;

use crate::physics::{object::Object, PhysicsEngine};

const DEFAULT_PARTICLE_RADIUS: f64 = 2.0;
const DEFAULT_PARTICLE_SPACING: f64 = 2.0;
const DEFAULT_PARTICLE_MASS: f64 = 1.0;

pub struct Brick {
    pub position: Vector2<f64>,
    pub size: Vector2<f64>,
    pub particle_radius: f64,
    pub particle_spacing: f64,
    pub particle_mass: f64,
}

impl Brick {
    pub fn new(position: Vector2<f64>, size: Vector2<f64>) -> Self {
        Self {
            position,
            size,
            particle_radius: DEFAULT_PARTICLE_RADIUS,
            particle_spacing: DEFAULT_PARTICLE_SPACING,
            particle_mass: DEFAULT_PARTICLE_MASS,
        }
    }
}

pub fn generate_brick(physics: &mut PhysicsEngine, wall: Brick) -> Vec<usize> {
    let cell_size = wall.particle_radius * 2.0 + wall.particle_spacing;
    let dims = Vector2::new((wall.size.x / cell_size) as usize, (wall.size.y / cell_size) as usize);
    let mut result = Vec::new();
    for i in 0..dims.x {
        for j in 0..dims.y {
            let p =
                |position, index| position + (index + 1) as f64 * (wall.particle_radius * 2.0 + wall.particle_spacing);
            let position = Vector2::new(p(wall.position.x, i), p(wall.position.y, j));
            let id = physics.add(Object {
                radius: wall.particle_radius,
                mass: wall.particle_mass,
                ..Object::new(position)
            });
            result.push(id);
        }
    }
    result
}

pub struct Ball {
    pub position: Vector2<f64>,
    pub radius: f64,
    pub acceleration: Vector2<f64>,
    pub velocity: Vector2<f64>,
    pub particle_radius: f64,
    pub particle_spacing: f64,
    pub particle_mass: f64,
    pub color: Option<Color>,
}

impl Ball {
    pub fn new(position: Vector2<f64>, radius: f64) -> Self {
        Self {
            position,
            radius,
            velocity: Vector2::new(0.0, 0.0),
            acceleration: Vector2::new(0.0, 0.0),
            particle_radius: DEFAULT_PARTICLE_RADIUS,
            particle_spacing: DEFAULT_PARTICLE_SPACING,
            particle_mass: DEFAULT_PARTICLE_MASS,
            color: None,
        }
    }
}

pub fn generate_ball(physics: &mut PhysicsEngine, ball: Ball) -> Vec<usize> {
    let mut result = Vec::new();
    let num_particles = (ball.radius * 2.0 / (ball.particle_radius * 2.0 + ball.particle_spacing)) as usize;
    for i in 0..num_particles {
        for j in 0..num_particles {
            let x = -ball.radius + (i as f64) * (ball.particle_radius * 2.0 + ball.particle_spacing);
            let y = -ball.radius + (j as f64) * (ball.particle_radius * 2.0 + ball.particle_spacing);
            let position = Vector2::new(x, y);
            if position.magnitude() <= ball.radius {
                let position = ball.position + position;
                let mut object = Object {
                    acceleration: ball.acceleration,
                    radius: ball.particle_radius,
                    mass: ball.particle_mass,
                    color: ball.color,
                    ..Object::new(position)
                };
                object.set_velocity(ball.velocity, 1.0);
                let id = physics.add(object);
                result.push(id);
            }
        }
    }
    result
}
