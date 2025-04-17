#![warn(clippy::mismatching_type_param_order)]

use rdst::{RadixKey, RadixSort};

use crate::vector2::Vector2;

#[derive(Default, Clone)]
pub struct Bvh {
    object_aabbs: Vec<AABB>,
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
                aabb,
                morton_code,
            });
        }
        let mut bvh = Bvh {
            object_aabbs,
            nodes: Vec::with_capacity(positions.len() * 2),
            root: NodeId::default(),
        };
        items.radix_sort_unstable();
        bvh.root = bvh.create_subtree(&items);
        bvh
    }

    pub fn find_intersections(&self, object_index: usize, mut f: impl FnMut((usize, usize))) {
        if !self.nodes.is_empty() {
            self.find_intersections_with(object_index, self.root, &mut f);
        }
    }

    fn find_intersections_with(&self, object1_index: usize, node_id: NodeId, f: &mut impl FnMut((usize, usize))) {
        let object_aabb = self.object_aabbs[object1_index];
        let Node { aabb, kind, .. } = &self.nodes[node_id.0];
        if object_aabb.intersects(aabb) {
            match *kind {
                NodeKind::Leaf(object2_index) => {
                    f((object1_index, object2_index));
                }
                NodeKind::Tree { left, right } => {
                    self.find_intersections_with(object1_index, left, f);
                    self.find_intersections_with(object1_index, right, f);
                }
            }
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
    aabb: AABB,
    morton_code: u32,
}

impl RadixKey for Item {
    const LEVELS: usize = u32::LEVELS;

    fn get_level(&self, level: usize) -> u8 {
        self.morton_code.get_level(level)
    }
}

#[derive(Default, Clone, Copy)]
pub struct AABB {
    pub topleft: Vector2<f64>,
    pub bottomright: Vector2<f64>,
}

impl AABB {
    const PLACEHOLDER: Self = Self {
        topleft: Vector2::new(f64::NAN, f64::NAN),
        bottomright: Vector2::new(f64::NAN, f64::NAN),
    };

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

impl Node {
    const PLACEHOLDER: Self = Self {
        aabb: AABB::PLACEHOLDER,
        kind: NodeKind::Leaf(0),
    };
}

#[derive(Clone, Copy)]
pub enum NodeKind {
    Leaf(usize),
    Tree { left: NodeId, right: NodeId },
}
