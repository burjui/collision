use nalgebra::Vector2;
use ndarray::Array2;
use smallvec::SmallVec;

use super::object::Object;

pub struct Grid {
    position: Vector2<f64>,
    size: Vector2<usize>,
    cell_size: f64,
    pub(super) objects: Vec<Object>,
    pub(super) cells: Array2<SmallVec<[usize; 4]>>,
}

impl Grid {
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
                self.cells[cell].push(index);
            }
        }
    }

    // TODO remove from former cell
    // pub fn update_object_cell(&mut self, object_index: usize, previous_position: Vector2<f64>) {
    //     let object = &self.objects[object_index];
    //     let new_cell = cell_at(object.position, self.position, self.cell_size);
    //     let old_cell = cell_at(previous_position, self.position, self.cell_size);
    //     if new_cell != old_cell {
    //         self.cells[old_cell].retain(|i| *i != object_index);
    //         self.cells[new_cell].push(object_index);
    //     }
    // }

    pub fn position(&self) -> Vector2<f64> {
        self.position
    }

    pub fn size(&self) -> Vector2<usize> {
        self.size
    }

    pub fn cell_size(&self) -> f64 {
        self.cell_size
    }

    pub fn objects(&self) -> &[Object] {
        &self.objects
    }

    pub fn objects_mut(&mut self) -> &mut [Object] {
        &mut self.objects
    }

    pub fn cells(&self) -> &Array2<SmallVec<[usize; 4]>> {
        &self.cells
    }
}

impl Default for Grid {
    fn default() -> Self {
        Self {
            position: Vector2::new(0.0, 0.0),
            size: Vector2::new(0, 0),
            cell_size: 0.0,
            objects: Vec::new(),
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
