use std::{iter::zip, ops::Range};

use rdst::{RadixKey, RadixSort};

use super::{bvh::morton_code, object::ObjectSoa};
use crate::{array2::Array2, vector2::Vector2};

#[derive(Clone)]
pub struct Grid {
    position: Vector2<f64>,
    size: Vector2<usize>,
    cell_size: f64,
    pub cell_records: Vec<CellRecord>,
    pub coords_to_cells: Array2<Option<(usize, usize)>>,
}

impl Grid {
    pub fn update(&mut self, objects: &ObjectSoa) {
        let mut end = Vector2::new(f64::MIN, f64::MIN);
        self.position = Vector2::new(f64::MAX, f64::MAX);
        self.cell_size = 0.0;

        for (position, radius) in zip(&objects.positions, &objects.radii) {
            self.position.x = self.position.x.min(position.x - radius);
            self.position.y = self.position.y.min(position.y - radius);
            end.x = end.x.max(position.x + radius);
            end.y = end.y.max(position.y + radius);
            self.cell_size = self.cell_size.max(radius * 2.0);
        }

        if self.cell_size > 0.0 {
            self.size.x = ((end.x - self.position.x) / self.cell_size).ceil() as usize;
            self.size.y = ((end.y - self.position.y) / self.cell_size).ceil() as usize;
            self.cell_records.resize(objects.len(), CellRecord::EMPTY);
            for (object_index, &position) in objects.positions.iter().enumerate() {
                let (x, y) = cell_at(position, self.position, self.cell_size);
                self.cell_records[object_index] = CellRecord {
                    object_index,
                    cell_coords: (x, y),
                    radix_key: morton_code(x as u32, y as u32),
                }
            }
            self.cell_records.radix_sort_unstable();
            self.coords_to_cells.reset((self.size.x, self.size.y));
            if !self.cell_records.is_empty() {
                for range in CellIter::new(&self.cell_records) {
                    let cell_coords = self.cell_records[range.start].cell_coords;
                    self.coords_to_cells[cell_coords] = Some((range.start, range.end));
                }
            }
        }
    }

    #[must_use]
    pub fn position(&self) -> Vector2<f64> {
        self.position
    }

    #[must_use]
    pub fn size(&self) -> Vector2<usize> {
        self.size
    }

    #[must_use]
    pub fn cell_size(&self) -> f64 {
        self.cell_size
    }

    #[must_use]
    pub fn cell_iter(&self) -> CellIter<'_> {
        CellIter::new(&self.cell_records)
    }
}

type CellRadixKey = u32;

#[derive(Copy, Clone)]
pub struct CellRecord {
    pub object_index: usize,
    pub cell_coords: (usize, usize),
    radix_key: CellRadixKey,
}

impl CellRecord {
    const EMPTY: Self = Self {
        object_index: usize::MAX,
        cell_coords: (usize::MAX, usize::MAX),
        radix_key: CellRadixKey::MAX,
    };
}

impl Default for CellRecord {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl RadixKey for CellRecord {
    const LEVELS: usize = <CellRadixKey as RadixKey>::LEVELS;

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
            coords_to_cells: Array2::default(),
        }
    }
}

#[must_use]
pub fn cell_at(position: Vector2<f64>, cells_start: Vector2<f64>, cell_size: f64) -> (usize, usize) {
    (
        ((position.x - cells_start.x) / cell_size) as usize,
        ((position.y - cells_start.y) / cell_size) as usize,
    )
}
