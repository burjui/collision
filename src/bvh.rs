use std::{iter::zip, time::Instant};

use crate::{physics::NormalizedCollisionPair, vector2::Vector2};

#[derive(Default, Clone)]
pub struct Bvh {
    object_indices: Vec<usize>,
    object_aabbs: Vec<AABB>,
    nodes: Vec<Node>,
}

impl Bvh {
    pub fn update(&mut self, positions: &[Vector2<f64>], radii: &[f64]) {
        let start = Instant::now();
        if self.object_indices.len() != positions.len() {
            self.object_indices.clear();
            self.object_indices.extend(0..positions.len());
        }
        println!("BVH: allocate object indices {:?}", start.elapsed());

        let start = Instant::now();
        self.object_aabbs.clear();
        self.object_aabbs.reserve(positions.len());
        println!("BVH: allocate object AABBs {:?}", start.elapsed());

        let start = Instant::now();

        let dummy_node = Node {
            aabb: AABB {
                topleft: Vector2::default(),
                bottomright: Vector2::default(),
            },
            tag: NodeTag::Leaf,
            data: NodeData { leaf_object_index: 0 },
        };
        self.nodes.resize(positions.len() * 2, dummy_node);
        println!("BVH: clear nodes {:?}", start.elapsed());

        let start = Instant::now();
        self.object_aabbs.extend(zip(positions, radii).map(|(&position, &radius)| AABB {
            topleft: position - radius,
            bottomright: position + radius,
        }));
        println!("BVH: init object AABBs {:?}", start.elapsed());

        let start = Instant::now();
        for (i, object_index) in self.object_indices.iter().copied().enumerate() {
            self.nodes[i] = Node {
                aabb: self.object_aabbs[object_index],
                tag: NodeTag::Leaf,
                data: NodeData {
                    leaf_object_index: u32::try_from(object_index).unwrap(),
                },
            };
        }
        println!("BVH: init nodes {:?}", start.elapsed());

        let start = Instant::now();
        let mut nodes_len = positions.len();
        let (mut src, mut dst) = (0..nodes_len, nodes_len..nodes_len);
        while src.len() > 1 {
            while src.len() > 1 {
                let left = src.start;
                let right = src.start + 1;
                src.start += 2;
                let left_aabb = self.nodes[left].aabb;
                let right_aabb = self.nodes[right].aabb;
                let aabb = left_aabb.union(&right_aabb);
                let node_id = nodes_len;
                nodes_len += 1;
                self.nodes[node_id] = Node {
                    aabb,
                    tag: NodeTag::Tree,
                    data: NodeData {
                        tree: Tree {
                            left: NodeId(left.try_into().unwrap()),
                            right: NodeId(right.try_into().unwrap()),
                        },
                    },
                };
                dst.end += 1;
            }
            if src.len() == 1 {
                dst.start -= 1;
            }
            src = dst.clone();
            dst = dst.end..dst.end;
        }
        self.nodes.truncate(nodes_len);
        println!("BVH: construct tree {:?}", start.elapsed());

        let start = Instant::now();
        self.nodes.reverse();
        let node_count = u32::try_from(self.nodes.len()).unwrap();
        for node in &mut self.nodes {
            if let NodeTag::Tree = node.tag {
                let children = unsafe { &mut node.data.tree };
                children.left.0 = node_count - children.left.0 - 1;
                children.right.0 = node_count - children.right.0 - 1;
            }
        }
        println!("BVH: reverse nodes {:?}", start.elapsed());
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
