use nalgebra::Vector2;
use ndarray::Array2;
use smallvec::SmallVec;

use super::object::Object;

pub type GridCell = SmallVec<[usize; 32]>;

pub struct Grid {
    position: Vector2<f64>,
    size: Vector2<usize>,
    cell_size: f64,
    pub(super) cells: Array2<GridCell>,
}

impl Grid {
    pub fn update(&mut self, objects: &[Object]) {
        let mut end = Vector2::new(f64::MIN, f64::MIN);
        self.position = Vector2::new(f64::MAX, f64::MAX);
        self.cell_size = 0.0;

        for &Object { position, radius, .. } in objects {
            self.position.x = self.position.x.min(position.x - radius);
            self.position.y = self.position.y.min(position.y - radius);
            end.x = end.x.max(position.x + radius);
            end.y = end.y.max(position.y + radius);
            self.cell_size = self.cell_size.max(radius * 2.0);
        }

        self.cells = Array2::from_elem((0, 0), SmallVec::new());
        if self.position.x <= end.x && self.position.y <= end.y {
            self.size.x = ((end.x - self.position.x) / self.cell_size).ceil() as usize + 1;
            self.size.y = ((end.y - self.position.y) / self.cell_size).ceil() as usize + 1;
            self.cells = Array2::from_elem((self.size.x, self.size.y), SmallVec::new());

            for (index, &Object { position, .. }) in objects.iter().enumerate() {
                let cell = cell_at(position, self.position, self.cell_size);
                self.cells[cell].push(index);
            }
        }
    }

    pub fn position(&self) -> Vector2<f64> {
        self.position
    }

    pub fn size(&self) -> Vector2<usize> {
        self.size
    }

    pub fn cell_size(&self) -> f64 {
        self.cell_size
    }

    pub fn cells(&self) -> &Array2<GridCell> {
        &self.cells
    }
}

impl Default for Grid {
    fn default() -> Self {
        Self {
            position: Vector2::new(0.0, 0.0),
            size: Vector2::new(0, 0),
            cell_size: 0.0,
            cells: Array2::from_elem((0, 0), SmallVec::new()),
        }
    }
}

pub fn cell_at(position: Vector2<f64>, cells_start: Vector2<f64>, cell_size: f64) -> (usize, usize) {
    (
        ((position.x - cells_start.x) / cell_size).floor() as usize,
        ((position.y - cells_start.y) / cell_size).floor() as usize,
    )
}
