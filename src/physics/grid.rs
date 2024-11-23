use std::{collections::HashMap, ops::Index};

use cgmath::Vector2;
use derive_deref::Deref;

use crate::physics::object::ObjectId;

pub struct GridBuilder {
    grid: Grid,
    start: Vector2<f64>,
    cell_size: f64,
}

impl GridBuilder {
    pub fn new() -> Self {
        GridBuilder {
            grid: Grid::new(),
            start: Vector2::new(f64::MAX, f64::MAX),
            cell_size: 0.0,
        }
    }

    pub fn add_object(&mut self, id: ObjectId, position: Vector2<f64>, size: f64) -> usize {
        let index = self.grid.objects.len();
        self.grid.objects.push(GridObject {
            object: id,
            position,
            size,
            cells: Vec::new(),
        });
        let half_size = size * 0.5;
        self.start.x = self.start.x.min(position.x - half_size);
        self.start.y = self.start.y.min(position.y - half_size);
        self.cell_size = self.cell_size.max(size * 1.1);
        index
    }

    pub fn build(mut self) -> Grid {
        let mut grid_size = Vector2::new(0, 0);
        let mut cell_indices = HashMap::new();

        for GridObject {
            object,
            position,
            size,
            cells: object_cells,
        } in &mut self.grid.objects
        {
            let start = *position - Vector2::new(*size * 0.5, *size * 0.5);
            let end = start + Vector2::new(*size, *size);
            let corners = [
                (start.x, start.y),
                (start.x, end.y),
                (end.x, start.y),
                (end.x, end.y),
            ];
            for corner in corners {
                let cell = cell_at(self.start, self.cell_size, corner);
                grid_size.x = grid_size.x.max(cell.x);
                grid_size.y = grid_size.y.max(cell.y);

                let cell_index = *cell_indices.entry(cell).or_insert_with(|| {
                    let index = self.grid.cells.len();
                    self.grid.cells.push(Vec::new());
                    index
                });
                object_cells.push(CellIndex(cell_index));

                self.grid.cells[cell_index].push(*object);
            }
        }

        self.grid
    }
}

pub struct GridObject {
    pub object: ObjectId,
    position: Vector2<f64>,
    size: f64,
    cells: Vec<CellIndex>,
}

#[derive(Copy, Clone, Deref, PartialEq, Eq, Hash)]
pub struct CellIndex(usize);

pub struct Grid {
    pub position: Vector2<f64>,
    pub size: Vector2<usize>,
    pub cell_size: f64,
    pub objects: Vec<GridObject>,
    pub cells: Vec<Vec<ObjectId>>,
}

impl Grid {
    pub fn new() -> Self {
        Self {
            position: Vector2::new(0.0, 0.0),
            size: Vector2::new(0, 0),
            cell_size: 0.0,
            objects: Vec::new(),
            cells: Vec::new(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }
}

impl Index<CellIndex> for Grid {
    type Output = [ObjectId];
    fn index(&self, index: CellIndex) -> &Self::Output {
        &self.cells[index.0]
    }
}

type Cell = Vector2<usize>;

fn cell_at(start: Vector2<f64>, cell_size: f64, (x, y): (f64, f64)) -> Cell {
    let cell_x = ((x - start.x) / cell_size).floor() as usize;
    let cell_y = ((y - start.y) / cell_size).floor() as usize;
    Cell::new(cell_x, cell_y)
}
