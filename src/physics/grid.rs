use crate::physics::object::ObjectId;
use cgmath::Vector2;
use std::collections::{HashMap, HashSet};
use std::ops::{Deref, Range};

pub(crate) struct GridBuilder {
    objects: Vec<(ObjectId, Vector2<f64>, Vector2<f64>)>,
    start: Vector2<f64>,
    end: Vector2<f64>,
    cell_size: f64,
}

impl GridBuilder {
    pub(crate) fn new() -> Self {
        GridBuilder {
            objects: Vec::new(),
            start: Vector2::new(f64::MAX, f64::MAX),
            end: Vector2::new(f64::MIN, f64::MIN),
            cell_size: 0.0,
        }
    }

    pub(crate) fn add(&mut self, id: ObjectId, position: Vector2<f64>, size: f64) {
        let half_size = size * 0.5;
        let object_start = Vector2::new(position.x - half_size, position.y - half_size);
        let object_end = Vector2::new(position.x + half_size, position.y + half_size);
        self.objects.push((id, object_start, object_end));
        self.start.x = self.start.x.min(object_start.x);
        self.start.y = self.start.y.min(object_start.y);
        self.end.x = self.end.x.max(object_end.x);
        self.end.y = self.end.y.max(object_end.y);
        self.cell_size = self.cell_size.max(size);
    }

    pub(crate) fn build(self) -> ReadOnly<Grid> {
        let mut size = Vector2::new(0, 0);
        let unique_obj_cells = {
            let mut obj_cells = HashMap::new();
            if self.cell_size > 0.0 {
                for (id, start, end) in self.objects {
                    for (x, y) in [
                        (start.x, start.y),
                        (start.x, end.y),
                        (end.x, start.y),
                        (end.x, end.y),
                    ] {
                        let cell @ (cell_x, cell_y) = cell(self.start, self.cell_size, (x, y));
                        size.x = size.x.max(cell_x);
                        size.y = size.y.max(cell_y);
                        obj_cells
                            .entry(cell)
                            .or_insert_with(HashSet::new)
                            .insert(id);
                    }
                }
            }

            obj_cells
        };

        let mut objects = Vec::with_capacity(unique_obj_cells.len());
        let mut cells = Vec::new();
        let mut obj_cells_offset = 0;
        for (_, mates) in unique_obj_cells {
            let mut obj_in_cell_count = 0;
            for id in mates {
                obj_in_cell_count += 1;
                objects.push(id);
            }
            cells.push(obj_cells_offset..obj_cells_offset + obj_in_cell_count);
            obj_cells_offset += obj_in_cell_count;
        }

        ReadOnly(Grid {
            position: self.start,
            size,
            cell_size: self.cell_size,
            objects,
            cells,
        })
    }
}

pub(crate) struct ReadOnly<T>(T);

impl<T> Deref for ReadOnly<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

pub(crate) struct Grid {
    pub(crate) position: Vector2<f64>,
    pub(crate) size: Vector2<usize>,
    pub(crate) cell_size: f64,
    pub(crate) objects: Vec<ObjectId>,
    pub(crate) cells: Vec<Range<usize>>,
}

impl Grid {
    pub(crate) fn cells(&self) -> impl Iterator<Item = impl Iterator<Item = ObjectId> + '_> + '_ {
        self.cells
            .iter()
            .map(|range| self.objects[range.clone()].iter().copied())
    }
}

pub(crate) fn empty_grid() -> ReadOnly<Grid> {
    ReadOnly(Grid {
        position: Vector2::new(0.0, 0.0),
        size: Vector2::new(0, 0),
        cell_size: 0.0,
        objects: Vec::new(),
        cells: Vec::new(),
    })
}

type Cell = (usize, usize);

fn cell(start: Vector2<f64>, cell_size: f64, (x, y): (f64, f64)) -> Cell {
    let cell_x = ((x - start.x) / cell_size).floor() as usize;
    let cell_y = ((y - start.y) / cell_size).floor() as usize;
    (cell_x, cell_y)
}
