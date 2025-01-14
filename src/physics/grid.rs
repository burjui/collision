use cgmath::Vector2;
use derive_deref::Deref;
use ndarray::Array2;
use smallvec::SmallVec;

use super::object::Object;

pub struct GridBuilder {
    grid: Grid,
    start: Vector2<f64>,
    cell_size: f64,
}

impl GridBuilder {
    pub fn add_object(&mut self, object: Object) -> usize {
        let index = self.grid.objects.len();
        self.grid.objects.push(object);
        self.grid.object_cells.push(Cell::new(0, 0));
        let half_size = object.size * 0.5;
        self.start.x = self.start.x.min(object.position.x - half_size);
        self.start.y = self.start.y.min(object.position.y - half_size);
        self.cell_size = self.cell_size.max(object.size * 1.1);
        index
    }

    pub fn build(mut self) -> Grid {
        let mut grid_size = Vector2::new(0, 0);
        for Object { position, size, .. } in &self.grid.objects {
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
            }
        }

        self.grid.cells = Array2::from_elem((grid_size.x + 1, grid_size.y + 1), SmallVec::new());

        for (object_index, Object { position, size, .. }) in self.grid.objects.iter().enumerate() {
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
                self.grid.object_cells[object_index] = cell;
                self.grid.cells[(cell.x, cell.y)].push(object_index);
            }
        }

        self.grid
    }
}

impl Default for GridBuilder {
    fn default() -> Self {
        Self {
            grid: Grid::default(),
            start: Vector2::new(0.0, 0.0),
            cell_size: 0.0,
        }
    }
}

#[derive(Copy, Clone, Deref, PartialEq, Eq, Hash)]
pub struct CellIndex(usize);

pub struct Grid {
    pub position: Vector2<f64>,
    pub size: Vector2<usize>,
    pub cell_size: f64,
    pub objects: Vec<Object>,
    pub cells: Array2<SmallVec<[usize; 4]>>,
    pub object_cells: Vec<Cell>,
}

impl Grid {
    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
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
            object_cells: Vec::new(),
        }
    }
}

type Cell = Vector2<usize>;

fn cell_at(start: Vector2<f64>, cell_size: f64, (x, y): (f64, f64)) -> Cell {
    let cell_x = ((x - start.x) / cell_size).floor() as usize;
    let cell_y = ((y - start.y) / cell_size).floor() as usize;
    Cell::new(cell_x, cell_y)
}
