use cgmath::Vector2;
use core::fmt::{Display, Formatter};
use core::iter::Iterator;
use core::option::Option;
use core::option::Option::{None, Some};
use slab::Slab;
use std::collections::HashMap;
use std::vec::Vec;

pub struct PhysicsObject {
    pub position: Vector2<f32>,
    pub velocity: Vector2<f32>,
    pub size: f32,
}

pub struct ObjectId(usize);

impl Display for ObjectId {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

pub struct CollisionDetector {
    objects: Slab<PhysicsObject>,
    grid: HashMap<(usize, usize), Vec<usize>>,
    grid_position: Option<Vector2<f32>>,
    cells: HashMap<usize, Vec<(usize, usize)>>,
    cell_size: f32,
}

impl CollisionDetector {
    pub fn new() -> Self {
        Self {
            objects: Slab::new(),
            grid: HashMap::new(),
            grid_position: None,
            cells: HashMap::new(),
            cell_size: 0.0,
        }
    }

    pub fn add(&mut self, object: PhysicsObject) -> ObjectId {
        ObjectId(self.objects.insert(object))
    }

    pub fn update(&mut self) {
        self.grid.clear();
        self.cells.clear();

        self.grid_position = None;
        for (_, object) in &self.objects {
            self.cell_size = self.cell_size.max(object.size + 1.0);
            let x = object.position.x - object.size / 2.0;
            let y = object.position.y - object.size / 2.0;
            let grid_position_x = self
                .grid_position
                .map(|position| position.x.min(x))
                .unwrap_or(x);
            let grid_position_y = self
                .grid_position
                .map(|position| position.y.min(y))
                .unwrap_or(y);
            self.grid_position = Some(Vector2::new(grid_position_x, grid_position_y));

            #[cfg(debug_assertions)]
            println!(
                "@ grid_position: ({}, {}), xy = ({}, {})",
                grid_position_x, grid_position_y, x, y
            );
        }

        if let Some(grid_position) = &self.grid_position {
            #[cfg(debug_assertions)]
            println!("grid_position: ({}, {})", grid_position.x, grid_position.y);
            for (id, object) in &self.objects {
                #[cfg(debug_assertions)]
                println!("# {}", id);
                let half_size = object.size / 2.0;
                for point in &[
                    object.position + Vector2::new(-half_size, -half_size),
                    object.position + Vector2::new(-half_size, half_size),
                    object.position + Vector2::new(half_size, -half_size),
                    object.position + Vector2::new(half_size, half_size),
                ] {
                    let cell_x = ((point.x - grid_position.x) / self.cell_size) as usize;
                    let cell_y = ((point.y - grid_position.y) / self.cell_size) as usize;
                    let cell = (cell_x, cell_y);

                    #[cfg(debug_assertions)]
                    println!("({}, {}) -> {:?}", point.x, point.y, cell);
                    let cell_ids = self.grid.entry(cell).or_insert_with(Vec::new);
                    if !cell_ids.contains(&id) {
                        cell_ids.push(id);
                    }

                    let id_cells = self.cells.entry(id).or_insert_with(Vec::new);
                    if !id_cells.contains(&cell) {
                        id_cells.push(cell);
                    }
                }
            }
        }

        #[cfg(debug_assertions)]
        {
            use itertools::Itertools;

            for (id, _) in &self.objects {
                println!("# {}: {:?}", id, &self.cells[&id]);
            }

            for (cell, ids) in self.grid.iter().sorted_by_key(|(cell, _)| *cell) {
                println!("{:?}: {:?}", cell, ids);
            }

            let grid_size = self.grid_size();
            println!("grid_size: {}x{}", grid_size.x, grid_size.y);
        }
    }

    pub fn advance(&mut self, dt: f32) {
        for (_, object) in &mut self.objects {
            object.position += object.velocity * dt;
        }
    }

    pub fn objects(&self) -> impl Iterator<Item = (ObjectId, &PhysicsObject)> {
        self.objects
            .iter()
            .map(|(id, object)| (ObjectId(id), object))
    }

    pub fn grid_position(&self) -> Option<Vector2<f32>> {
        self.grid_position.clone()
    }

    pub fn grid_cell_size(&self) -> f32 {
        self.cell_size
    }

    pub fn grid_size(&self) -> Vector2<usize> {
        self.grid
            .keys()
            .next()
            .cloned()
            .map(|(first_x, first_y)| {
                let (mut min_x, mut min_y, mut max_x, mut max_y) =
                    (first_x, first_y, first_x, first_y);
                for (x, y) in self.grid.keys() {
                    min_x = min_x.min(*x);
                    min_y = min_y.min(*y);
                    max_x = max_x.max(*x);
                    max_y = max_y.max(*y);
                }
                Vector2::new(max_x - min_x + 1, max_y - min_y + 1)
            })
            .unwrap_or(Vector2::new(0, 0))
    }
}
