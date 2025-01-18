use collision::{
    compound::{generate_brick, Brick},
    physics::PhysicsEngine,
};
use nalgebra::Vector2;

macro_rules! emitted_scene_path {
    () => {
        "emitted_scene.rs"
    };
}

pub fn create_scene(physics: &mut PhysicsEngine) {
    let brick = Brick {
        particle_radius: 2.0,
        particle_spacing: 0.0,
        ..Brick::new(Vector2::new(200.0, 100.0), Vector2::new(1000.0, 590.0))
    };
    generate_brick(physics, brick);

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

    // include!(concat!("../", emitted_scene_path!()));
}
