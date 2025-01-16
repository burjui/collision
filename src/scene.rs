use nalgebra::Vector2;

use crate::physics::{object::Object, PhysicsEngine};

macro_rules! emitted_scene_path {
    () => {
        "emitted_scene.rs"
    };
}

const DEFAULT_PARTICLE_RADIUS: f64 = 2.0;
const DEFAULT_PARTICLE_SPACING: f64 = 2.0;
const DEFAULT_PARTICLE_MASS: f64 = 1.0;

pub fn create_scene(physics: &mut PhysicsEngine) {
    // let wall = Wall::new(Vector2::new(700.0, 200.0), Vector2::new(200.0, 200.0));
    // create_wall(physics, wall);

    // let mut ball = Ball::new(Vector2::new(1000.0, 300.0), 20.0);
    // ball.velocity = Vector2::new(-1000.0, 0.0);
    // create_ball(physics, ball);

    // physics.add(Object {
    //     position: Vector2::new(1000.0, 300.0),
    //     velocity: Vector2::new(-1000.0, 0.0),
    //     radius: 20.0,
    //     mass: 100.0,
    //     ..Default::default()
    // });

    let mut x_offset = 0.0;
    for y in 0..10 {
        for x in 0..10 {
            physics.add(Object {
                position: Vector2::new(700.0 + x as f64 * 10.0 + x_offset, 500.0 + y as f64 * 10.0),
                radius: 3.0,
                ..Default::default()
            });
        }
        // x_offset += 3.0;
    }

    physics.grid.update();
    // include!(concat!("../", emitted_scene_path!()));
}

struct Wall {
    position: Vector2<f64>,
    size: Vector2<f64>,
    particle_radius: f64,
    particle_spacing: f64,
    particle_mass: f64,
}

impl Wall {
    fn new(position: Vector2<f64>, size: Vector2<f64>) -> Self {
        Self {
            position,
            size,
            particle_radius: DEFAULT_PARTICLE_RADIUS,
            particle_spacing: DEFAULT_PARTICLE_SPACING,
            particle_mass: DEFAULT_PARTICLE_MASS,
        }
    }
}

fn create_wall(physics: &mut PhysicsEngine, wall: Wall) -> Vec<usize> {
    let cell_size = wall.particle_radius * 2.0 + wall.particle_spacing;
    let dims = Vector2::new((wall.size.x / cell_size) as usize, (wall.size.y / cell_size) as usize);
    let mut result = Vec::new();
    for i in 0..dims.x {
        for j in 0..dims.y {
            let p =
                |position, index| position + (index + 1) as f64 * (wall.particle_radius * 2.0 + wall.particle_spacing);
            let position = Vector2::new(p(wall.position.x, i), p(wall.position.y, j));
            let id = physics.add(Object {
                position,
                radius: wall.particle_radius,
                mass: wall.particle_mass,
                ..Default::default()
            });
            result.push(id);
        }
    }
    result
}

struct Ball {
    position: Vector2<f64>,
    radius: f64,
    acceleration: Vector2<f64>,
    velocity: Vector2<f64>,
    particle_radius: f64,
    particle_spacing: f64,
    particle_mass: f64,
}

impl Ball {
    fn new(position: Vector2<f64>, radius: f64) -> Self {
        Self {
            position,
            radius,
            velocity: Vector2::new(0.0, 0.0),
            acceleration: Vector2::new(0.0, 0.0),
            particle_radius: DEFAULT_PARTICLE_RADIUS,
            particle_spacing: DEFAULT_PARTICLE_SPACING,
            particle_mass: DEFAULT_PARTICLE_MASS,
        }
    }
}

fn create_ball(physics: &mut PhysicsEngine, ball: Ball) -> Vec<usize> {
    let mut result = Vec::new();
    let num_particles = (ball.radius * 2.0 / (ball.particle_radius * 2.0 + ball.particle_spacing)) as usize;
    for i in 0..num_particles {
        for j in 0..num_particles {
            let x = -ball.radius + (i as f64) * (ball.particle_radius * 2.0 + ball.particle_spacing);
            let y = -ball.radius + (j as f64) * (ball.particle_radius * 2.0 + ball.particle_spacing);
            let position = Vector2::new(x, y);
            if position.magnitude() <= ball.radius {
                let position = ball.position + position;
                let id = physics.add(Object {
                    position,
                    velocity: ball.velocity,
                    acceleration: ball.acceleration,
                    radius: ball.particle_radius,
                    mass: ball.particle_mass,
                });
                result.push(id);
            }
        }
    }
    result
}
