use vello::peniko::Color;

use crate::{
    physics::{object::Object, PhysicsEngine},
    vector2::Vector2,
};

const DEFAULT_PARTICLE_RADIUS: f32 = 2.0;
const DEFAULT_PARTICLE_SPACING: f32 = 2.0;
const DEFAULT_PARTICLE_MASS: f32 = 1.0;

pub struct Brick {
    pub position: Vector2<f32>,
    pub size: Vector2<f32>,
    pub particle_radius: f32,
    pub particle_spacing: f32,
    pub particle_mass: f32,
}

impl Brick {
    pub fn new(position: Vector2<f32>, size: Vector2<f32>) -> Self {
        Self {
            position,
            size,
            particle_radius: DEFAULT_PARTICLE_RADIUS,
            particle_spacing: DEFAULT_PARTICLE_SPACING,
            particle_mass: DEFAULT_PARTICLE_MASS,
        }
    }
}

pub fn generate_brick(physics: &mut PhysicsEngine, brick: Brick) -> Vec<usize> {
    let cell_size = brick.particle_radius * 2.0 + brick.particle_spacing;
    let dims = Vector2::new((brick.size.x / cell_size) as usize, (brick.size.y / cell_size) as usize);
    let mut result = Vec::new();
    let mut x_offset = false;
    for i in 0..dims.x {
        for j in 0..dims.y {
            let py = |position, index| {
                position + (index + 1) as f32 * (brick.particle_radius * 2.0 + brick.particle_spacing)
            };
            let px = |position, index| py(position, index);
            let position = Vector2::new(px(brick.position.x, i), py(brick.position.y, j));
            let id = physics.add(Object {
                radius: brick.particle_radius,
                mass: brick.particle_mass,
                ..Object::new(position)
            });
            result.push(id);
            x_offset = !x_offset;
        }
    }
    result
}

pub struct Ball {
    pub position: Vector2<f32>,
    pub radius: f32,
    pub acceleration: Vector2<f32>,
    pub velocity: Vector2<f32>,
    pub particle_radius: f32,
    pub particle_spacing: f32,
    pub particle_mass: f32,
    pub color: Option<Color>,
}

impl Ball {
    pub fn new(position: Vector2<f32>, radius: f32) -> Self {
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
            let x = -ball.radius + (i as f32) * (ball.particle_radius * 2.0 + ball.particle_spacing);
            let y = -ball.radius + (j as f32) * (ball.particle_radius * 2.0 + ball.particle_spacing);
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
                object.velocity = ball.velocity;
                let id = physics.add(object);
                result.push(id);
            }
        }
    }
    result
}
