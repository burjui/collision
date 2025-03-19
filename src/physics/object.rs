use vello::peniko::Color;

use crate::vector2::Vector2;

// TODO: FixedVec?
#[derive(Default)]
pub struct ObjectSoa {
    pub positions: Vec<Vector2<f64>>,
    pub velocities: Vec<Vector2<f64>>,
    pub radii: Vec<f64>,
    pub masses: Vec<f64>,
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

    pub fn resize(&mut self, size: usize) {
        self.positions.resize(size, Vector2::default());
        self.velocities.resize(size, Vector2::default());
        self.radii.resize(size, 0.0);
        self.masses.resize(size, 0.0);
        self.colors.resize(size, None);
        self.is_planet.resize(size, false);
    }

    pub fn planets(&self) -> ObjectSoaRef {
        ObjectSoaRef {
            positions: &self.positions[..self.planet_count],
            velocities: &self.velocities[..self.planet_count],
            radii: &self.radii[..self.planet_count],
            masses: &self.masses[..self.planet_count],
            colors: &self.colors[..self.planet_count],
            is_planet: &self.is_planet[..self.planet_count],
        }
    }
}

pub struct ObjectSoaRef<'a> {
    pub positions: &'a [Vector2<f64>],
    pub velocities: &'a [Vector2<f64>],
    pub radii: &'a [f64],
    pub masses: &'a [f64],
    pub colors: &'a [Option<Color>],
    pub is_planet: &'a [bool],
}

#[derive(Copy, Clone)]
pub struct ObjectPrototype {
    pub position: Vector2<f64>,
    pub velocity: Vector2<f64>,
    pub radius: f64,
    pub mass: f64,
    pub color: Option<Color>,
    pub is_planet: bool,
}

impl ObjectPrototype {
    #[must_use]
    pub fn new(position: Vector2<f64>) -> Self {
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
    pub fn momentum(&self) -> Vector2<f64> {
        self.velocity * self.mass
    }

    pub fn update(&mut self, update: ObjectUpdate) {
        self.position = update.position;
        self.velocity = update.velocity;
    }
}

#[derive(Copy, Clone)]
pub struct ObjectUpdate {
    pub position: Vector2<f64>,
    pub velocity: Vector2<f64>,
}

impl Default for ObjectUpdate {
    fn default() -> Self {
        Self {
            position: Vector2::default(),
            velocity: Vector2::default(),
        }
    }
}
