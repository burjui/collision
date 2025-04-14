#![allow(unused)]

use std::{collections::HashSet, f64::NAN, mem::swap, ops::Range};

use itertools::Itertools;
use rdst::{RadixKey, RadixSort};

use super::CollisionPair;
use crate::{fixed_vec::FixedVec, vector2::Vector2};

const MAX_CHILDREN: usize = 2;

#[derive(Default, Clone)]
pub struct Bvh {
    object_aabbs: Vec<AABB>,
    pub nodes: Vec<Node>,
    root: NodeId,
}

impl Bvh {
    pub fn new(positions: &[Vector2<f64>], radii: &[f64]) -> Self {
        let mut items = vec![Item::PLACEHOLDER; positions.len()];
        let mut object_aabbs = vec![AABB::PLACEHOLDER; positions.len()];
        for object_index in 0..positions.len() {
            let position = positions[object_index];
            let radius = radii[object_index];
            let aabb = AABB {
                topleft: position - radius,
                bottomright: position + radius,
            };
            object_aabbs[object_index] = aabb;
            let morton_code = morton_code(position.x as u32, position.y as u32);
            items[object_index] = Item {
                object_index,
                aabb,
                morton_code,
            };
        }
        items.radix_sort_unstable();
        let mut bvh = Bvh {
            object_aabbs,
            nodes: Vec::default(),
            root: NodeId::default(),
        };
        for item in items {
            bvh.nodes.push(Node {
                aabb: item.aabb,
                kind: NodeKind::Leaf(item.object_index),
            });
        }
        let mut combining_area = (0..bvh.nodes.len()).map(NodeId).collect_vec();
        let mut combining_staging_area = Vec::new();
        while combining_area.len() > 1 {
            let level_start = bvh.nodes.len();
            combining_staging_area.clear();
            let mut end = combining_area.len();
            while end > 0 {
                let start = end.saturating_sub(MAX_CHILDREN);
                let children = combining_area[start..end]
                    .iter()
                    .copied()
                    .collect::<FixedVec<_, MAX_CHILDREN>>();
                end = start;
                let mut combined_aabb = bvh.nodes[children[0].0].aabb;
                for &child_id in &children[1..] {
                    let child_aabb = bvh.nodes[child_id.0].aabb;
                    combined_aabb = AABB {
                        topleft: Vector2::new(
                            combined_aabb.topleft.x.min(child_aabb.topleft.x),
                            combined_aabb.topleft.y.min(child_aabb.topleft.y),
                        ),
                        bottomright: Vector2::new(
                            combined_aabb.bottomright.x.max(child_aabb.bottomright.x),
                            combined_aabb.bottomright.y.max(child_aabb.bottomright.y),
                        ),
                    };
                }
                let node_id = bvh.add_node(Node {
                    aabb: combined_aabb,
                    kind: NodeKind::Tree { children },
                });
                combining_staging_area.push(node_id);
            }
            combining_area.resize(combining_staging_area.len(), NodeId::default());
            combining_area.copy_from_slice(&combining_staging_area);
        }
        bvh.root = combining_area[0];
        bvh
    }

    pub fn find_intersections(&self, object_index: usize, collisions: &mut Vec<CollisionPair>) {
        if !self.nodes.is_empty() {
            self.find_intersections_with(object_index, self.root, collisions);
        }
    }

    fn find_intersections_with(&self, object1_index: usize, node_id: NodeId, collisions: &mut Vec<CollisionPair>) {
        let object_aabb = self.object_aabbs[object1_index];
        let Node { aabb, kind } = &self.nodes[node_id.0];
        if object_aabb.intersects(&aabb) {
            match kind {
                &NodeKind::Leaf(object2_index) => {
                    if object2_index != object1_index {
                        collisions.push(CollisionPair {
                            object1_index,
                            object2_index,
                        });
                    }
                }
                NodeKind::Tree { children } => {
                    for &child_id in children.as_slice() {
                        self.find_intersections_with(object1_index, child_id, collisions);
                    }
                }
            }
        }
    }

    fn add_node(&mut self, node: Node) -> NodeId {
        let id = self.nodes.len();
        self.nodes.push(node);
        NodeId(id)
    }
}

pub fn morton_code(x: u32, y: u32) -> u32 {
    expand_bits(x as u32) | (expand_bits(y as u32) << 1)
}

#[derive(Clone, Copy)]
struct Item {
    object_index: usize,
    aabb: AABB,
    morton_code: u32,
}

impl Item {
    const PLACEHOLDER: Self = Self {
        object_index: usize::MAX,
        aabb: AABB::PLACEHOLDER,
        morton_code: u32::MAX,
    };
}

impl RadixKey for Item {
    const LEVELS: usize = u32::LEVELS;

    fn get_level(&self, level: usize) -> u8 {
        self.morton_code.get_level(level)
    }
}

fn expand_bits(mut x: u32) -> u32 {
    x = x & 0xFFFF;
    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;
    x
}

#[derive(Clone, Copy)]
pub struct AABB {
    pub topleft: Vector2<f64>,
    pub bottomright: Vector2<f64>,
}

impl AABB {
    const PLACEHOLDER: Self = Self {
        topleft: Vector2::new(NAN, NAN),
        bottomright: Vector2::new(NAN, NAN),
    };

    fn intersects(&self, other: &AABB) -> bool {
        self.topleft.x <= other.bottomright.x
            && self.bottomright.x >= other.topleft.x
            && self.topleft.y <= other.bottomright.y
            && self.bottomright.y >= other.topleft.y
    }
}

#[derive(Default, Clone, Copy)]
pub struct NodeId(usize);

#[derive(Clone, Copy)]
pub struct Node {
    pub aabb: AABB,
    pub kind: NodeKind,
}

#[derive(Clone, Copy)]
pub enum NodeKind {
    Leaf(usize),
    Tree { children: FixedVec<NodeId, MAX_CHILDREN> },
}
