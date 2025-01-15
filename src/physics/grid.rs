use std::ops::Index;

use cgmath::Vector2;
use ndarray::Array2;
use smallvec::SmallVec;

use super::object::Object;

pub struct Grid {
    pub position: Vector2<f64>,
    pub size: Vector2<usize>,
    pub cell_size: f64,
    pub objects: Vec<Object>,
    pub cells: Array2<SmallVec<[usize; 4]>>,
}

impl Grid {
    pub fn new() -> Self {
        Self {
            position: Vector2::new(0.0, 0.0),
            size: Vector2::new(0, 0),
            cell_size: 0.0,
            objects: Vec::new(),
            cells: Array2::from_elem((0, 0), SmallVec::new()),
        }
    }

    pub fn update(&mut self) {
        let mut end = Vector2::new(f64::MIN, f64::MIN);
        self.position = Vector2::new(f64::MAX, f64::MAX);
        self.cell_size = 0.0;

        for &Object { position, radius, .. } in &self.objects {
            self.position.x = self.position.x.min(position.x);
            self.position.y = self.position.y.min(position.y);
            end.x = end.x.max(position.x);
            end.y = end.y.max(position.y);
            self.cell_size = self.cell_size.max(radius * 2.0);
        }

        if self.position.x <= end.x && self.position.y <= end.y {
            self.size.x = ((end.x - self.position.x) / self.cell_size).ceil() as usize + 1;
            self.size.y = ((end.y - self.position.y) / self.cell_size).ceil() as usize + 1;
            self.cells = Array2::from_elem((self.size.x, self.size.y), SmallVec::new());

            for (index, &Object { position, .. }) in self.objects.iter().enumerate() {
                let cell = cell_at(position, self.position, self.cell_size);
                self.cells[(cell.x, cell.y)].push(index);
            }
        }
    }
}

impl Index<Cell> for Grid {
    type Output = [usize];
    fn index(&self, id: Cell) -> &Self::Output {
        &self.cells[(id.x, id.y)]
    }
}

pub type Cell = Vector2<usize>;

pub fn cell_at(position: Vector2<f64>, cells_start: Vector2<f64>, cell_size: f64) -> Cell {
    Cell::new(
        ((position.x - cells_start.x) / cell_size).floor() as usize,
        ((position.y - cells_start.y) / cell_size).floor() as usize,
    )
}
