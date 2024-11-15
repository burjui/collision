use crate::physics::object::ObjectId;
use cgmath::Vector2;
use derive_deref::Deref;
use log::debug;
use petgraph::{
    graph::{NodeIndex, UnGraph},
    visit::IntoNodeReferences,
};

#[derive(Clone, Copy)]
enum Node {
    Object {
        object: ObjectId,
        position: Vector2<f64>,
        size: f64,
    },
    Cell {
        i: usize,
        j: usize,
    },
}

type Relations = UnGraph<Node, ()>;

#[derive(Deref)]
pub struct ReadOnly<T>(T);

pub struct GridBuilder {
    relations: Relations,
    start: Vector2<f64>,
    cell_size: f64,
}

impl GridBuilder {
    pub fn new() -> Self {
        GridBuilder {
            relations: UnGraph::default(),
            start: Vector2::new(f64::MAX, f64::MAX),
            cell_size: 0.0,
        }
    }

    pub fn add(&mut self, id: ObjectId, position: Vector2<f64>, size: f64) {
        self.relations.add_node(Node::Object {
            object: id,
            position,
            size,
        });
        let half_size = size * 0.5;
        self.start.x = self.start.x.min(position.x - half_size);
        self.start.y = self.start.y.min(position.y - half_size);
        self.cell_size = self.cell_size.max(size * 2.0);
    }

    pub fn build(mut self) -> Grid {
        let mut grid_size = Vector2::new(0, 0);
        for (object_index, node) in self
            .relations
            .node_references()
            .map(|(index, node)| (index, *node))
            .collect::<Vec<_>>()
        {
            if let Node::Object { position, size, .. } = node {
                let start = position - Vector2::new(size * 0.5, size * 0.5);
                let end = start + Vector2::new(size, size);
                for (x, y) in [
                    (start.x, start.y),
                    (start.x, end.y),
                    (end.x, start.y),
                    (end.x, end.y),
                ] {
                    let (cell_i, cell_j) = cell(self.start, self.cell_size, (x, y));
                    grid_size.x = grid_size.x.max(cell_i);
                    grid_size.y = grid_size.y.max(cell_j);
                    let cell_index = self
                        .relations
                        .node_references()
                        .find_map(|(cell_index, node)| match *node {
                            Node::Cell { i, j } if i == cell_i && j == cell_j => Some(cell_index),
                            _ => None,
                        })
                        .unwrap_or_else(|| {
                            // debug!("add cell ({}, {})", cell_i, cell_j);
                            self.relations.add_node(Node::Cell {
                                i: cell_i,
                                j: cell_j,
                            })
                        });

                    if !self.relations.contains_edge(object_index, cell_index) {
                        debug!(
                            "add edge ({:?}) -> {:?}",
                            self.node(object_index),
                            self.node(cell_index)
                        );
                        self.relations.add_edge(object_index, cell_index, ());
                    }
                }
            }
        }

        Grid {
            position: self.start,
            size: grid_size,
            cell_size: self.cell_size,
            relations: self.relations,
        }
    }

    fn node(&self, index: NodeIndex) -> String {
        match self.relations[index] {
            Node::Object { object, .. } => format!("object {}", object),
            Node::Cell { i, j } => format!("cell ({}, {})", i, j),
        }
    }
}

pub struct Grid {
    pub position: Vector2<f64>,
    pub size: Vector2<usize>,
    pub cell_size: f64,
    relations: UnGraph<Node, ()>,
}

impl Grid {
    pub fn new() -> Self {
        Self {
            position: Vector2::new(0.0, 0.0),
            size: Vector2::new(0, 0),
            cell_size: 0.0,
            relations: UnGraph::default(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.relations.node_count() == 0
    }

    pub fn cells(
        &self,
        object_predicate: impl Fn(ObjectId) -> bool + 'static,
    ) -> impl Iterator<Item = impl Iterator<Item = ObjectId> + Clone + '_> {
        self.relations
            .node_references()
            .filter_map(move |(index, node)| match *node {
                Node::Object { object: id, .. } if object_predicate(id) => Some(index),
                _ => None,
            })
            .flat_map(|object_index| {
                self.relations.neighbors(object_index).map(|cell_index| {
                    match self.relations[cell_index] {
                        Node::Cell { .. } => {
                            self.relations
                                .neighbors(cell_index)
                                .filter_map(|neighbor_index| match self.relations[neighbor_index] {
                                    Node::Object { object, .. } => Some(object),
                                    _ => None,
                                })
                        }
                        _ => unreachable!("not a cell"),
                    }
                })
            })
    }
}

type Cell = (usize, usize);

fn cell(start: Vector2<f64>, cell_size: f64, (x, y): (f64, f64)) -> Cell {
    let cell_x = ((x - start.x) / cell_size).floor() as usize;
    let cell_y = ((y - start.y) / cell_size).floor() as usize;
    (cell_x, cell_y)
}
