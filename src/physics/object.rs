use vello::peniko::Color;

use crate::vector2::Vector2;

// TODO: FixedVec?
pub struct ObjectSoa {
    pub positions: Vec<Vector2<f32>>,
    pub velocities: Vec<Vector2<f32>>,
    pub accelerations: Vec<Vector2<f32>>,
    pub radii: Vec<f32>,
    pub masses: Vec<f32>,
    pub colors: Vec<Option<Color>>,
    pub is_planet: Vec<bool>,
    pub planet_count: usize,
}

impl ObjectSoa {
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
        self.accelerations.push(object.acceleration);
        self.radii.push(object.radius);
        self.masses.push(object.mass);
        self.colors.push(object.color);
        self.is_planet.push(object.is_planet);

        self.planet_count += usize::from(object.is_planet);
        object_index
    }

    pub fn resize(&mut self, size: usize) {
        self.positions.resize(size, Vector2::default());
        self.velocities.resize(size, Vector2::default());
        self.accelerations.resize(size, Vector2::default());
        self.radii.resize(size, 0.0);
        self.masses.resize(size, 0.0);
        self.colors.resize(size, None);
        self.is_planet.resize(size, false);
    }

    pub fn planets(&self) -> ObjectSoaRef {
        ObjectSoaRef {
            positions: &self.positions[..self.planet_count],
            velocities: &self.velocities[..self.planet_count],
            accelerations: &self.accelerations[..self.planet_count],
            radii: &self.radii[..self.planet_count],
            masses: &self.masses[..self.planet_count],
            colors: &self.colors[..self.planet_count],
            is_planet: &self.is_planet[..self.planet_count],
        }
    }
}

impl Default for ObjectSoa {
    fn default() -> Self {
        Self {
            positions: Vec::default(),
            velocities: Vec::default(),
            accelerations: Vec::default(),
            radii: Vec::default(),
            masses: Vec::default(),
            colors: Vec::default(),
            is_planet: Vec::default(),
            planet_count: 0,
        }
    }
}

pub struct ObjectSoaRef<'a> {
    pub positions: &'a [Vector2<f32>],
    pub velocities: &'a [Vector2<f32>],
    pub accelerations: &'a [Vector2<f32>],
    pub radii: &'a [f32],
    pub masses: &'a [f32],
    pub colors: &'a [Option<Color>],
    pub is_planet: &'a [bool],
}

#[derive(Copy, Clone)]
pub struct ObjectPrototype {
    pub position: Vector2<f32>,
    pub velocity: Vector2<f32>,
    pub acceleration: Vector2<f32>,
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
            acceleration: Vector2::new(0.0, 0.0),
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

    pub fn update(&mut self, update: ObjectUpdate) {
        self.position = update.position;
        self.velocity = update.velocity;
    }
}

#[derive(Copy, Clone)]
pub struct ObjectUpdate {
    pub position: Vector2<f32>,
    pub velocity: Vector2<f32>,
}
