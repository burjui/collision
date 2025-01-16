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
    let brick = Brick::new(Vector2::new(400.0, 200.0), Vector2::new(200.0, 200.0));
    create_wall(physics, brick);

    // let mut ball = Object {
    //     radius: 20.0,
    //     mass: 100.0,
    //     ..Object::new(Vector2::new(800.0, 300.0))
    // };
    // ball.set_velocity(Vector2::new(-3.0, 0.0), 1.0);
    // physics.add(ball);

    // let mut ball = Ball::new(Vector2::new(1000.0, 300.0), 20.0);
    // ball.velocity = Vector2::new(-1000.0, 0.0);
    // create_ball(physics, ball);

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

    physics.grid.update();
    // include!(concat!("../", emitted_scene_path!()));
}

struct Brick {
    position: Vector2<f64>,
    size: Vector2<f64>,
    particle_radius: f64,
    particle_spacing: f64,
    particle_mass: f64,
}

impl Brick {
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

fn create_wall(physics: &mut PhysicsEngine, wall: Brick) -> Vec<usize> {
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
                let mut object = Object {
                    acceleration: ball.acceleration,
                    radius: ball.particle_radius,
                    mass: ball.particle_mass,
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
