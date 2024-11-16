use cgmath::{InnerSpace, Vector2};

use crate::physics::object::{Object, ObjectId};
use crate::physics::CollisionDetector;

macro_rules! emitted_scene_path {
    () => {
        "emitted_scene.rs"
    };
}

const DEFAULT_PARTICLE_SIZE: f64 = 4.0;
const DEFAULT_PARTICLE_SPACING: f64 = 1.0;
const DEFAULT_PARTICLE_MASS: f64 = 1.0;

pub fn create_scene(collision_detector: &mut CollisionDetector) {
    let wall = Wall::new((140.0, 200.0), (200.0, 200.0));
    create_wall(collision_detector, wall);

    let mut ball = Ball::new((500.0, 300.0), 14.0);
    ball.velocity = Vector2::new(-1000.0, 0.0);
    create_ball(collision_detector, ball);

    // include!(concat!("../", emitted_scene_path!()));
}

struct Wall {
    position: Vector2<f64>,
    size: Vector2<f64>,
    velocity: Vector2<f64>,
    particle_size: f64,
    particle_spacing: f64,
    particle_mass: f64,
}

impl Wall {
    fn new(position: (f64, f64), size: (f64, f64)) -> Self {
        Self {
            position: position.into(),
            size: size.into(),
            velocity: Vector2::new(0.0, 0.0),
            particle_size: DEFAULT_PARTICLE_SIZE,
            particle_spacing: DEFAULT_PARTICLE_SPACING,
            particle_mass: DEFAULT_PARTICLE_MASS,
        }
    }
}

fn create_wall(collision_detector: &mut CollisionDetector, wall: Wall) -> Vec<ObjectId> {
    let cell_size = wall.particle_size + wall.particle_spacing;
    let dims = Vector2::new(
        (wall.size.x / cell_size) as usize,
        (wall.size.y / cell_size) as usize,
    );
    let mut result = Vec::new();
    for i in 0..dims.x {
        for j in 0..dims.y {
            let p = |position, index| {
                position + (index + 1) as f64 * (wall.particle_size + wall.particle_spacing)
            };
            let id = collision_detector.add(Object {
                position: Vector2::new(p(wall.position.x, i), p(wall.position.y, j)),
                velocity: wall.velocity,
                size: wall.particle_size,
                mass: wall.particle_mass,
            });
            result.push(id);
        }
    }
    result
}

struct Ball {
    position: Vector2<f64>,
    radius: f64,
    velocity: Vector2<f64>,
    particle_size: f64,
    particle_spacing: f64,
    particle_mass: f64,
}

impl Ball {
    fn new(position: (f64, f64), radius: f64) -> Self {
        Self {
            position: position.into(),
            radius,
            velocity: Vector2::new(0.0, 0.0),
            particle_size: DEFAULT_PARTICLE_SIZE,
            particle_spacing: DEFAULT_PARTICLE_SPACING,
            particle_mass: DEFAULT_PARTICLE_MASS,
        }
    }
}

fn create_ball(collision_detector: &mut CollisionDetector, ball: Ball) -> Vec<ObjectId> {
    let mut result = Vec::new();
    let num_particles = (ball.radius * 2.0 / (ball.particle_size + ball.particle_spacing)) as usize;
    for i in 0..num_particles {
        for j in 0..num_particles {
            let x = -ball.radius + (i as f64) * (ball.particle_size + ball.particle_spacing);
            let y = -ball.radius + (j as f64) * (ball.particle_size + ball.particle_spacing);
            let position = Vector2::new(x, y);
            if position.magnitude() <= ball.radius {
                let id = collision_detector.add(Object {
                    position: ball.position + position,
                    velocity: ball.velocity,
                    size: ball.particle_size,
                    mass: ball.particle_mass,
                });
                result.push(id);
            }
        }
    }
    result
}
