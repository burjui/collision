#![allow(unused)]

use std::{collections::HashSet, f64::NAN, mem::swap};

use rdst::{RadixKey, RadixSort};

use crate::vector2::Vector2;

#[derive(Default)]
pub struct Bvh {
    object_aabbs: Vec<AABB>,
    nodes: Vec<Node>,
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
            items[object_index] = Item {
                object_index,
                aabb,
                morton_code: morton_code(position.x as u32, position.y as u32),
            };
        }
        items.radix_sort_unstable();
        let mut bvh = Bvh {
            object_aabbs,
            nodes: Vec::default(),
        };
        bvh.create_subtree(&items);
        bvh
    }

    pub fn find_intersections(&self, object_index: usize, collisions: &mut Vec<(usize, usize)>) {
        self.find_intersections_with(object_index, NodeId(0), collisions);
    }

    fn find_intersections_with(&self, object_index: usize, node_id: NodeId, collisions: &mut Vec<(usize, usize)>) {
        let object_aabb = self.object_aabbs[object_index];
        match self.nodes.get(node_id.0) {
            Some(Node { aabb, kind }) => {
                if object_aabb.intersects(&aabb) {
                    match kind {
                        &NodeKind::Leaf(other_object_index) => {
                            if other_object_index != object_index {
                                collisions.push((object_index, other_object_index));
                            }
                        }
                        &NodeKind::Tree { left, right } => {
                            self.find_intersections_with(object_index, left, collisions);
                            self.find_intersections_with(object_index, right, collisions);
                        }
                    }
                }
            }
            None => (),
        }
    }

    fn create_subtree(&mut self, items: &[Item]) -> NodeId {
        if items.len() == 1 {
            let item = items[0];
            self.add_node(Node {
                aabb: item.aabb,
                kind: NodeKind::Leaf(item.object_index),
            })
        } else {
            let node_id = self.add_node(Node::PLACEHOLDER);
            let middle = items.len() / 2;
            let left = self.create_subtree(&items[..middle]);
            let right = self.create_subtree(&items[middle..]);
            let left_aabb = self.nodes[left.0].aabb;
            let right_aabb = self.nodes[right.0].aabb;
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
            self.nodes[node_id.0] = Node {
                aabb,
                kind: NodeKind::Tree { left, right },
            };
            node_id
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
struct AABB {
    topleft: Vector2<f64>,
    bottomright: Vector2<f64>,
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

#[derive(Clone, Copy)]
struct NodeId(usize);

struct Node {
    aabb: AABB,
    kind: NodeKind,
}

impl Node {
    const PLACEHOLDER: Self = Self {
        aabb: AABB::PLACEHOLDER,
        kind: NodeKind::Leaf(usize::MAX),
    };
}

enum NodeKind {
    Leaf(usize),
    Tree { left: NodeId, right: NodeId },
}
