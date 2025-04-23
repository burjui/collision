use std::{iter::zip, mem::swap};

use rayon::slice::ParallelSliceMut;

use crate::{physics::NormalizedCollisionPair, vector2::Vector2};

#[derive(Default, Clone)]
pub struct Bvh {
    object_indices: Vec<usize>,
    object_aabbs: Vec<AABB>,
    nodes: Vec<Node>,
    fold_buffer_a: Vec<usize>,
    fold_buffer_b: Vec<usize>,
}

impl Bvh {
    pub fn update(&mut self, positions: &[Vector2<f64>], radii: &[f64], morton_codes: &[u32]) {
        self.object_indices.clear();
        self.object_aabbs.reserve(positions.len());

        self.object_aabbs.clear();
        self.object_aabbs.reserve(positions.len());

        self.nodes.clear();
        self.nodes.reserve(positions.len() * 2);

        self.fold_buffer_a.clear();
        self.fold_buffer_a.reserve(positions.len() * 2);

        self.fold_buffer_b.clear();
        self.fold_buffer_b.reserve(positions.len() * 2);

        self.object_aabbs.extend(zip(positions, radii).map(|(&position, &radius)| AABB {
            topleft: position - radius,
            bottomright: position + radius,
        }));

        self.object_indices.extend(0..positions.len());
        self.object_indices.par_sort_unstable_by_key(|&object_index| morton_codes[object_index]);

        for &object_index in &self.object_indices {
            self.nodes.push(Node {
                aabb: self.object_aabbs[object_index],
                tag: NodeTag::Leaf,
                data: NodeData {
                    leaf_object_index: u32::try_from(object_index).unwrap(),
                },
            });
        }

        let (mut src, mut dst) = (&mut self.fold_buffer_a, &mut self.fold_buffer_b);
        src.extend(0..self.nodes.len());
        while src.len() > 1 {
            dst.clear();
            for chunk in src.chunks(2) {
                let node_id = if chunk.len() == 1 {
                    chunk[0]
                } else {
                    let left = chunk[0];
                    let right = chunk[1];
                    let left_aabb = self.nodes[left].aabb;
                    let right_aabb = self.nodes[right].aabb;
                    let aabb = left_aabb.union(&right_aabb);
                    let node_id = self.nodes.len();
                    self.nodes.push(Node {
                        aabb,
                        tag: NodeTag::Tree,
                        data: NodeData {
                            tree: Tree {
                                left: NodeId(left.try_into().unwrap()),
                                right: NodeId(right.try_into().unwrap()),
                            },
                        },
                    });
                    node_id
                };
                dst.push(node_id);
            }
            swap(&mut src, &mut dst);
        }

        self.nodes.reverse();
        let node_count = u32::try_from(self.nodes.len()).unwrap();
        for node in &mut self.nodes {
            if let NodeTag::Tree = node.tag {
                let children = unsafe { &mut node.data.tree };
                children.left.0 = node_count - children.left.0 - 1;
                children.right.0 = node_count - children.right.0 - 1;
            }
        }
    }

    pub fn nodes(&mut self) -> &mut [Node] {
        &mut self.nodes
    }

    pub fn root(&self) -> NodeId {
        NodeId(0)
    }

    pub fn object_aabbs(&mut self) -> &mut [AABB] {
        &mut self.object_aabbs
    }

    pub fn find_intersections(
        &self,
        object1_index: usize,
        positions: &[Vector2<f64>],
        radii: &[f64],
        candidates: &mut [NormalizedCollisionPair],
    ) {
        if self.nodes.is_empty() {
            return;
        }

        const STACK_SIZE: usize = 64;
        let mut stack = [NodeId(0); STACK_SIZE];
        let mut sp = 0;
        stack[sp] = self.root();
        sp += 1;

        let object1_aabb = self.object_aabbs[object1_index];
        let object1_position = positions[object1_index];
        let object1_radius = radii[object1_index];
        let mut candidate_index = 0;

        while sp > 0 {
            sp -= 1;
            let node_id = stack[sp];
            let node_idx = node_id.0 as usize;

            if !object1_aabb.intersects(&self.nodes[node_idx].aabb) {
                continue;
            }

            match self.nodes[node_idx].tag {
                NodeTag::Leaf => {
                    let object2_index =
                        usize::try_from(unsafe { self.nodes[node_idx].data.leaf_object_index }).unwrap();
                    if object2_index == object1_index {
                        continue;
                    }

                    let object2_position = positions[object2_index];
                    let object2_radius = radii[object2_index];
                    let dx = object1_position.x - object2_position.x;
                    let dy = object1_position.y - object2_position.y;
                    let distance_squared = dx * dx + dy * dy;
                    let collision_distance = object1_radius + object2_radius;

                    if distance_squared < collision_distance * collision_distance {
                        if candidate_index < candidates.len() {
                            candidates[candidate_index] = NormalizedCollisionPair::new(object1_index, object2_index);
                            candidate_index += 1;
                        }
                    }
                }
                NodeTag::Tree => {
                    if sp + 1 < STACK_SIZE {
                        let children = unsafe { self.nodes[node_idx].data.tree };
                        stack[sp] = children.left;
                        stack[sp + 1] = children.right;
                        sp += 2;
                    }
                }
            }
        }
    }
}

pub fn morton_code(position: Vector2<f64>) -> u32 {
    let x = position.x as u32;
    let y = position.y as u32;
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

    fn union(&self, other: &AABB) -> AABB {
        AABB {
            topleft: Vector2::new(self.topleft.x.min(other.topleft.x), self.topleft.y.min(other.topleft.y)),
            bottomright: Vector2::new(
                self.bottomright.x.max(other.bottomright.x),
                self.bottomright.y.max(other.bottomright.y),
            ),
        }
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct NodeId(u32);

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Node {
    pub aabb: AABB,
    tag: NodeTag,
    data: NodeData,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub enum NodeTag {
    Leaf,
    Tree,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub union NodeData {
    leaf_object_index: u32,
    tree: Tree,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Tree {
    left: NodeId,
    right: NodeId,
}
