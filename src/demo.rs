use collision::{
    compound::{generate_brick, Brick},
    physics::{object::Object, PhysicsEngine},
    vector2::Vector2,
};
use vello::peniko::color::palette::css;

pub fn create_demo(physics: &mut PhysicsEngine) {
    const RADIUS: f32 = 1.0;

    physics.add(Object {
        velocity: Vector2::new(-700.0, 0.0),
        radius: RADIUS,
        mass: 10000.0,
        color: Some(css::MAGENTA),
        is_planet: true,
        ..Object::new(Vector2::new(700.0, 500.0))
    });

    physics.add(Object {
        velocity: Vector2::new(700.0, 0.0),
        radius: RADIUS,
        mass: 10000.0,
        color: Some(css::YELLOW),
        is_planet: true,
        ..Object::new(Vector2::new(700.0, 600.0))
    });

    // physics.add(Object {
    //     radius: 10.0,
    //     mass: 20000.0,
    //     color: Some(css::YELLOW),
    //     is_planet: true,
    //     ..Object::new(Vector2::new(900.0, 500.0))
    // });

    let brick = Brick {
        particle_radius: RADIUS,
        particle_spacing: 0.01,
        particle_mass: 0.01,
        velocity: Vector2::new(0.0, 0.0),
        ..Brick::new(Vector2::new(400.0, 100.0), Vector2::new(600.0, 300.0))
    };
    generate_brick(physics, &brick);

    // physics.add(Object {
    //     velocity: Vector2::new(0.0, 10.0),
    //     radius: 10.0,
    //     mass: 1.0,
    //     color: Some(Color::YELLOW),
    //     ..Object::new(Vector2::new(900.0, 400.0))
    // });

    // physics.add(Object {
    //     velocity: Vector2::new(0.0, -10.0),
    //     radius: 10.0,
    //     mass: 1.0,
    //     color: Some(Color::GREEN),
    //     ..Object::new(Vector2::new(900.0, 500.0))
    // });
}
