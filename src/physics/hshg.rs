use std::iter::zip;

use rdst::{RadixKey, RadixSort};

use super::{
    CollisionPair,
    bvh::AABB,
    grid::{CellRecord, Grid, cell_at},
};
use crate::vector2::Vector2;

#[derive(Default, Clone)]
pub struct Hshg {
    pub grids: Vec<Grid>,
    object_item: Vec<usize>,
    items: Vec<Item>,
    position_buffer: Vec<Vector2<f64>>,
    radius_buffer: Vec<f64>,
    object_index_buffer: Vec<usize>,
}

impl Hshg {
    pub fn new(positions: &[Vector2<f64>], radii: &[f64]) -> Self {
        let mut hshg = Self::default();
        hshg.object_item.resize(positions.len(), 0);
        let max_size = radii
            .iter()
            .fold(0.0_f64, |max_size, radius| max_size.max(radius * 2.0));
        for (object_index, (&position, &radius)) in zip(positions, radii).enumerate() {
            let size = radius * 2.0;
            let aabb = AABB {
                topleft: position - radius,
                bottomright: position + radius,
            };
            hshg.items.push(Item {
                object_index,
                position,
                radius,
                aabb,
                radix_key: (size / max_size * 10000.0) as u64,
            });
        }
        hshg.items.radix_sort_unstable();
        let mut range = 0..0;
        let item_count = hshg.items.len();
        if item_count > 0 {
            while range.start < item_count {
                while range.end < item_count && hshg.items[range.end].radix_key == hshg.items[range.start].radix_key {
                    range.end += 1;
                }
                if !range.is_empty() {
                    hshg.position_buffer.clear();
                    hshg.radius_buffer.clear();
                    hshg.object_index_buffer.clear();
                    let level = hshg.grids.len();
                    for (
                        offset,
                        &Item {
                            object_index,
                            position,
                            radius,
                            ..
                        },
                    ) in hshg.items[range.clone()].iter().enumerate()
                    {
                        hshg.object_item[object_index] = range.start + offset;
                        hshg.position_buffer.push(position);
                        hshg.radius_buffer.push(radius);
                        hshg.object_index_buffer.push(object_index);
                    }
                    hshg.grids.push(Grid::default());
                    hshg.grids[level].update(&hshg.position_buffer, &hshg.radius_buffer, &hshg.object_index_buffer);
                }
                range.start = range.end;
            }
        }
        hshg
    }

    pub fn find_collision_candidates(&self, candidates: &mut Vec<CollisionPair>) {
        if !self.grids.is_empty() {
            for level in (0..self.grids.len()).rev() {
                for &CellRecord { object_index, .. } in &self.grids[level].cell_records {
                    let aabb = self.items[self.object_item[object_index]].aabb;
                    self.find_candidates(object_index, aabb, level, candidates);
                }
            }
        }
    }

    pub fn find_candidates(&self, object1_index: usize, aabb: AABB, level: usize, candidates: &mut Vec<CollisionPair>) {
        let grid = &self.grids[level];
        let mut topleft_cell = cell_at(aabb.topleft, grid.position(), grid.cell_size());
        topleft_cell.x = topleft_cell.x.saturating_sub(1);
        topleft_cell.y = topleft_cell.y.saturating_sub(1);
        let mut bottomright_cell = cell_at(aabb.bottomright, grid.position(), grid.cell_size());
        bottomright_cell.x = bottomright_cell.x.saturating_add(1).min(grid.size().x - 1);
        bottomright_cell.y = bottomright_cell.y.saturating_add(1).min(grid.size().y - 1);
        for y in topleft_cell.y..=bottomright_cell.y {
            for x in topleft_cell.x..=bottomright_cell.x {
                if let Some((start, end)) = grid.coords_to_cells[(x, y)] {
                    for &CellRecord {
                        object_index: object2_index,
                        ..
                    } in &grid.cell_records[start..end]
                    {
                        if object2_index != object1_index {
                            candidates.push(CollisionPair {
                                object1_index,
                                object2_index,
                            });
                        }
                    }
                }
            }
        }
        if level > 0 {
            self.find_candidates(object1_index, aabb, level - 1, candidates);
        }
    }
}

#[derive(Default, Clone, Copy)]
struct Item {
    object_index: usize,
    position: Vector2<f64>,
    radius: f64,
    aabb: AABB,
    radix_key: u64,
}

impl RadixKey for Item {
    const LEVELS: usize = u64::LEVELS;

    #[inline]
    fn get_level(&self, level: usize) -> u8 {
        self.radix_key.get_level(level)
    }
}
