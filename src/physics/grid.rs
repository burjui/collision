use std::ops::Range;

use rdst::{RadixKey, RadixSort};

use super::object::Object;
use crate::{array2::Array2, vector2::Vector2};

pub struct Grid {
    position: Vector2<f32>,
    size: Vector2<usize>,
    cell_size: f32,
    pub(super) cell_records: Vec<CellRecord>,
    pub(super) coords_to_cells: Array2<Option<(usize, usize)>>,
}

impl Grid {
    pub fn update(&mut self, objects: &[Object]) {
        let mut end = Vector2::new(f32::MIN, f32::MIN);
        self.position = Vector2::new(f32::MAX, f32::MAX);
        self.cell_size = 0.0;

        for &Object { position, radius, .. } in objects {
            self.position.x = self.position.x.min(position.x - radius);
            self.position.y = self.position.y.min(position.y - radius);
            end.x = end.x.max(position.x + radius);
            end.y = end.y.max(position.y + radius);
            self.cell_size = self.cell_size.max(radius * 2.0);
        }

        if self.cell_size > 0.0 {
            self.size.x = ((end.x - self.position.x) / self.cell_size).ceil() as usize + 1;
            self.size.y = ((end.y - self.position.y) / self.cell_size).ceil() as usize + 1;

            if self.cell_records.len() != objects.len() {
                self.cell_records.resize(objects.len(), CellRecord::EMPTY);
            }

            for (object_index, &Object { position, .. }) in objects.iter().enumerate() {
                let cell = cell_at(position, self.position, self.cell_size);
                self.cell_records[object_index] = CellRecord {
                    object_index,
                    cell_coords: cell,
                    radix_key: ((cell.1 << 16) | cell.0).try_into().unwrap(),
                }
            }
            self.cell_records.radix_sort_unstable();

            if self.coords_to_cells.size() != (self.size.x, self.size.y) {
                self.coords_to_cells.reset((self.size.x, self.size.y));
            }
            if !self.cell_records.is_empty() {
                for range in CellIter::new(&self.cell_records) {
                    let cell_coords = self.cell_records[range.start].cell_coords;
                    self.coords_to_cells[cell_coords] = Some((range.start, range.end));
                }
            }
        }
    }

    #[must_use]
    pub fn position(&self) -> Vector2<f32> {
        self.position
    }

    #[must_use]
    pub fn size(&self) -> Vector2<usize> {
        self.size
    }

    #[must_use]
    pub fn cell_size(&self) -> f32 {
        self.cell_size
    }

    #[must_use]
    pub fn cell_iter(&self) -> CellIter<'_> {
        CellIter::new(&self.cell_records)
    }
}

#[derive(Copy, Clone)]
pub struct CellRecord {
    pub object_index: usize,
    pub cell_coords: (usize, usize),
    radix_key: u32,
}

impl CellRecord {
    const EMPTY: Self = Self {
        object_index: 0,
        cell_coords: (0, 0),
        radix_key: 0,
    };
}

impl Default for CellRecord {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl RadixKey for CellRecord {
    const LEVELS: usize = <u32 as RadixKey>::LEVELS;

    fn get_level(&self, level: usize) -> u8 {
        self.radix_key.get_level(level)
    }
}

pub struct CellIter<'a> {
    cells: &'a [CellRecord],
    index: usize,
}

impl<'a> CellIter<'a> {
    fn new(cells: &'a [CellRecord]) -> Self {
        Self { cells, index: 0 }
    }
}

impl Iterator for CellIter<'_> {
    type Item = Range<usize>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.cells.len() {
            let starting_cell = self.cells[self.index].cell_coords;
            let start = self.index;
            let mut end = self.index;
            while end < self.cells.len() && self.cells[end].cell_coords == starting_cell {
                end += 1;
            }
            if end > self.index {
                self.index = end;
                Some(start..end)
            } else {
                None
            }
        } else {
            None
        }
    }
}

impl Default for Grid {
    fn default() -> Self {
        Self {
            position: Vector2::new(0.0, 0.0),
            size: Vector2::new(0, 0),
            cell_size: 0.0,
            cell_records: Vec::new(),
            coords_to_cells: Array2::default((0, 0)),
        }
    }
}

#[must_use]
pub fn cell_at(position: Vector2<f32>, cells_start: Vector2<f32>, cell_size: f32) -> (usize, usize) {
    (
        ((position.x - cells_start.x) / cell_size) as usize,
        ((position.y - cells_start.y) / cell_size) as usize,
    )
}
