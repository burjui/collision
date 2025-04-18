use std::mem::swap;

use itertools::Itertools;
use rayon::slice::ParallelSliceMut;

use crate::{physics::NormalizedCollisionPair, vector2::Vector2};

#[derive(Default, Clone)]
pub struct Bvh {
    object_aabbs: Vec<AABB>,
    object_positions: Vec<Vector2<f64>>,
    object_radii: Vec<f64>,
    pub nodes: Vec<Node>,
    root: NodeId,
}

impl Bvh {
    pub fn new(positions: &[Vector2<f64>], radii: &[f64]) -> Self {
        let mut items = Vec::with_capacity(positions.len());
        let mut object_aabbs = Vec::with_capacity(positions.len());
        for object_index in 0..positions.len() {
            let position = positions[object_index];
            let radius = radii[object_index];
            let aabb = AABB {
                topleft: position - radius,
                bottomright: position + radius,
            };
            object_aabbs.push(aabb);
            let morton_code = morton_code(position.x as u32, position.y as u32);
            items.push(Item {
                object_index,
                morton_code,
            });
        }
        items.par_sort_unstable_by_key(|item| item.morton_code);
        let mut nodes = Vec::with_capacity(positions.len() * 2);
        for item in &items {
            nodes.push(Node {
                aabb: object_aabbs[item.object_index],
                kind: NodeKind::Leaf(item.object_index),
            });
        }
        let mut combine_area = (0..nodes.len()).map(NodeId).collect_vec();
        let mut combine_area_tmp = Vec::with_capacity(combine_area.len().div_ceil(2));
        let (mut combine_area, mut combine_area_tmp) = (&mut combine_area, &mut combine_area_tmp);
        while combine_area.len() > 1 {
            combine_area_tmp.clear();
            for chunk in combine_area.chunks(2) {
                let node_id = if chunk.len() == 1 {
                    chunk[0]
                } else {
                    let node_id = NodeId(nodes.len());
                    let left = chunk[0];
                    let right = chunk[1];
                    let left_aabb = nodes[left.0].aabb;
                    let right_aabb = nodes[right.0].aabb;
                    let aabb = AABB {
                        topleft: Vector2::new(
                            left_aabb.topleft.x.min(right_aabb.topleft.x),
                            left_aabb.topleft.y.min(right_aabb.topleft.y),
                        ),
                        bottomright: Vector2::new(
                            left_aabb.bottomright.x.max(right_aabb.bottomright.x),
                            left_aabb.bottomright.y.max(right_aabb.bottomright.y),
                        ),
                    };
                    nodes.push(Node {
                        aabb,
                        kind: NodeKind::Tree { left, right },
                    });
                    node_id
                };
                combine_area_tmp.push(node_id);
            }
            swap(&mut combine_area, &mut combine_area_tmp);
        }
        Bvh {
            object_aabbs,
            object_positions: positions.to_vec(),
            object_radii: radii.to_vec(),
            nodes,
            root: combine_area[0],
        }
    }

    pub fn find_intersections(&self, object_index: usize, candidates: &mut Vec<NormalizedCollisionPair>) {
        if !self.nodes.is_empty() {
            self.find_intersections_with(object_index, self.root, candidates);
        }
    }

    fn find_intersections_with(
        &self,
        object1_index: usize,
        node_id: NodeId,
        candidates: &mut Vec<NormalizedCollisionPair>,
    ) {
        let object_aabb = self.object_aabbs[object1_index];
        let Node { aabb, kind, .. } = &self.nodes[node_id.0];
        if object_aabb.intersects(aabb) {
            match *kind {
                NodeKind::Leaf(object2_index) => {
                    if object2_index != object1_index {
                        let object1_position = self.object_positions[object1_index];
                        let object2_position = self.object_positions[object2_index];
                        let object1_radius = self.object_radii[object1_index];
                        let object2_radius = self.object_radii[object2_index];
                        let distance_squared = (object1_position - object2_position).magnitude_squared();
                        let collision_distance = object1_radius + object2_radius;
                        if distance_squared < collision_distance * collision_distance {
                            candidates.push(NormalizedCollisionPair::new(object1_index, object2_index));
                        }
                    }
                }
                NodeKind::Tree { left, right } => {
                    self.find_intersections_with(object1_index, left, candidates);
                    self.find_intersections_with(object1_index, right, candidates);
                }
            }
        }
    }
}

pub fn morton_code(x: u32, y: u32) -> u32 {
    expand_bits(x) | (expand_bits(y) << 1)
}

fn expand_bits(mut x: u32) -> u32 {
    x &= 0xFFFF;
    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;
    x
}

#[derive(Clone, Copy)]
struct Item {
    object_index: usize,
    morton_code: u32,
}

#[derive(Default, Clone, Copy)]
pub struct AABB {
    pub topleft: Vector2<f64>,
    pub bottomright: Vector2<f64>,
}

impl AABB {
    fn intersects(&self, other: &AABB) -> bool {
        self.topleft.x <= other.bottomright.x
            && self.bottomright.x >= other.topleft.x
            && self.topleft.y <= other.bottomright.y
            && self.bottomright.y >= other.topleft.y
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct NodeId(usize);

#[derive(Clone, Copy)]
pub struct Node {
    pub aabb: AABB,
    pub kind: NodeKind,
}

#[derive(Clone, Copy)]
pub enum NodeKind {
    Leaf(usize),
    Tree { left: NodeId, right: NodeId },
}
