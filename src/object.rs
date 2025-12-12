use std::ops::Range;

use vello::peniko::Color;

use crate::vector2::Vector2;

#[derive(Default)]
pub struct ObjectSoa {
    pub positions: Vec<Vector2<f32>>,
    pub velocities: Vec<Vector2<f32>>,
    pub radii: Vec<f32>,
    pub masses: Vec<f32>,
    pub colors: Vec<Option<Color>>,
    pub is_planet: Vec<bool>,
    pub planet_count: usize,
}

impl ObjectSoa {
    #[must_use]
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.positions.len()
    }

    pub fn add(&mut self, object: ObjectPrototype) -> usize {
        let object_index = self.len();
        assert!(
            !object.is_planet || object_index == self.planet_count,
            "planets must be added before any other objects"
        );
        self.positions.push(object.position);
        self.velocities.push(object.velocity);
        self.radii.push(object.radius);
        self.masses.push(object.mass);
        self.colors.push(object.color);
        self.is_planet.push(object.is_planet);
        self.planet_count += usize::from(object.is_planet);
        object_index
    }

    #[must_use]
    pub fn particle_range(&self) -> Range<usize> {
        self.planet_count..self.positions.len()
    }

    #[must_use]
    pub fn planet_range(&self) -> Range<usize> {
        0..self.planet_count
    }
}

#[derive(Copy, Clone)]
pub struct ObjectPrototype {
    pub position: Vector2<f32>,
    pub velocity: Vector2<f32>,
    pub radius: f32,
    pub mass: f32,
    pub color: Option<Color>,
    pub is_planet: bool,
}

impl ObjectPrototype {
    #[must_use]
    pub fn new(position: Vector2<f32>) -> Self {
        Self {
            position,
            velocity: Vector2::new(0.0, 0.0),
            radius: 1.0,
            mass: 1.0,
            color: None,
            is_planet: false,
        }
    }

    #[must_use]
    pub fn momentum(&self) -> Vector2<f32> {
        self.velocity * self.mass
    }
}
