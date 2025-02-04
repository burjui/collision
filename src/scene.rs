use collision::{
    compound::{generate_brick, Brick},
    physics::{object::Object, PhysicsEngine},
};
use nalgebra::Vector2;
use sdl2::pixels::Color;

macro_rules! emitted_scene_path {
    () => {
        "emitted_scene.rs"
    };
}

pub fn create_scene(physics: &mut PhysicsEngine) {
    let position = Vector2::new(700.0, 450.0);
    let velocity = Vector2::new(-2.0, 0.0);
    let planet1 = Object {
        previous_position: position - velocity,
        radius: 10.0,
        mass: 10000.0,
        color: Some(Color::MAGENTA),
        is_planet: true,
        ..Object::new(position)
    };
    physics.add(planet1);

    let position = Vector2::new(700.0, 500.0);
    let velocity = Vector2::new(2.0, 0.0);
    let planet2 = Object {
        previous_position: position - velocity,
        radius: 10.0,
        mass: 10000.0,
        color: Some(Color::YELLOW),
        is_planet: true,
        ..Object::new(position)
    };
    physics.add(planet2);

    let brick = Brick {
        particle_radius: 3.0,
        particle_spacing: 0.0,
        particle_mass: 0.1,
        ..Brick::new(Vector2::new(200.0, 100.0), Vector2::new(990.0, 200.0))
    };
    generate_brick(physics, brick);

    // include!(concat!("../", emitted_scene_path!()));
}
