use std::{mem::swap, u32};

use itertools::Itertools;
use rayon::slice::ParallelSliceMut;

use crate::{physics::NormalizedCollisionPair, vector2::Vector2};

#[derive(Default, Clone)]
pub struct Bvh {
    node_aabbs: Vec<AABB>,
    node_tags: Vec<NodeTag>,
    node_leaf_indices: Vec<u32>,
    node_tree_left: Vec<NodeId>,
    node_tree_right: Vec<NodeId>,
    object_aabbs: Vec<AABB>,
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

        let mut node_aabbs = Vec::default();
        let mut node_tags = Vec::default();
        let mut node_leaf_indices = Vec::default();
        let mut node_tree_left = Vec::default();
        let mut node_tree_right = Vec::default();
        for item in &items {
            node_aabbs.push(object_aabbs[item.object_index]);
            node_tags.push(NodeTag::Leaf);
            node_leaf_indices.push(u32::try_from(item.object_index).unwrap());
            node_tree_left.push(NodeId::INVALID);
            node_tree_right.push(NodeId::INVALID);
        }

        let mut combine_area = (0..node_aabbs.len()).map(|i| NodeId(u32::try_from(i).unwrap())).collect_vec();
        let mut combine_area_tmp = Vec::with_capacity(combine_area.len().div_ceil(2));
        let (mut combine_area, mut combine_area_tmp) = (&mut combine_area, &mut combine_area_tmp);
        while combine_area.len() > 1 {
            combine_area_tmp.clear();
            for chunk in combine_area.chunks(2) {
                let node_id = if chunk.len() == 1 {
                    chunk[0]
                } else {
                    let left = chunk[0];
                    let right = chunk[1];
                    let left_aabb = node_aabbs[usize::try_from(left.0).unwrap()];
                    let right_aabb = node_aabbs[usize::try_from(right.0).unwrap()];
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
                    let node_id = NodeId(u32::try_from(node_aabbs.len()).unwrap());
                    node_aabbs.push(aabb);
                    node_tags.push(NodeTag::Tree);
                    node_leaf_indices.push(u32::MAX);
                    node_tree_left.push(left);
                    node_tree_right.push(right);
                    node_id
                };
                combine_area_tmp.push(node_id);
            }
            swap(&mut combine_area, &mut combine_area_tmp);
        }
        Bvh {
            node_aabbs,
            node_tags,
            node_leaf_indices,
            node_tree_left,
            node_tree_right,
            object_aabbs,
        }
    }

    pub fn node_aabbs(&mut self) -> &mut [AABB] {
        &mut self.node_aabbs
    }

    pub fn node_tags(&mut self) -> &mut [NodeTag] {
        &mut self.node_tags
    }

    pub fn node_leaf_indices(&mut self) -> &mut [u32] {
        &mut self.node_leaf_indices
    }

    pub fn node_tree_left(&mut self) -> &mut [NodeId] {
        &mut self.node_tree_left
    }

    pub fn node_tree_right(&mut self) -> &mut [NodeId] {
        &mut self.node_tree_right
    }

    pub fn root(&self) -> NodeId {
        let id = self.node_aabbs.len().checked_sub(1).unwrap();
        NodeId(u32::try_from(id).unwrap())
    }

    pub fn object_aabbs(&mut self) -> &mut [AABB] {
        &mut self.object_aabbs
    }

    pub fn find_intersections(
        &self,
        object_index: usize,
        positions: &[Vector2<f64>],
        radii: &[f64],
        candidates: &mut [NormalizedCollisionPair],
    ) {
        if !self.node_aabbs.is_empty() {
            self.find_intersections_with(object_index, positions, radii, candidates);
        }
    }

    fn find_intersections_with(
        &self,
        object1_index: usize,
        positions: &[Vector2<f64>],
        radii: &[f64],
        candidates: &mut [NormalizedCollisionPair],
    ) {
        const STACK_SIZE: usize = 32;
        let mut stack = [NodeId(0); STACK_SIZE];
        let mut sp = 0;
        stack[sp] = self.root();
        sp += 1;

        let mut candidate_index = 0;
        while sp > 0 {
            sp -= 1;
            let node_id = stack[sp];
            let object_aabb = self.object_aabbs[object1_index];
            let node_id = usize::try_from(node_id.0).unwrap();
            let aabb = self.node_aabbs[node_id];
            if object_aabb.intersects(&aabb) {
                let tag = self.node_tags[node_id];
                match tag {
                    NodeTag::Leaf => {
                        let object2_index = usize::try_from(self.node_leaf_indices[node_id]).unwrap();
                        if object2_index != object1_index {
                            let object1_position = positions[object1_index];
                            let object2_position = positions[object2_index];
                            let object1_radius = radii[object1_index];
                            let object2_radius = radii[object2_index];
                            let distance_squared = (object1_position - object2_position).magnitude_squared();
                            let collision_distance = object1_radius + object2_radius;
                            if distance_squared < collision_distance * collision_distance {
                                candidates[candidate_index] =
                                    NormalizedCollisionPair::new(object1_index, object2_index);
                                candidate_index += 1;
                            }
                        }
                    }
                    NodeTag::Tree => {
                        stack[sp] = self.node_tree_left[node_id];
                        sp += 1;
                        stack[sp] = self.node_tree_right[node_id];
                        sp += 1;
                    }
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

#[repr(C)]
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
pub struct NodeId(u32);

impl NodeId {
    const INVALID: Self = Self(u32::MAX);
}

#[repr(C)]
#[derive(Clone, Copy)]
pub enum NodeTag {
    Leaf,
    Tree,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Node {
    pub aabb: AABB,
    pub kind: NodeKind,
}

#[repr(C, u32)]
#[derive(Clone, Copy)]
pub enum NodeKind {
    Leaf(u32),
    Tree { left: NodeId, right: NodeId },
}
