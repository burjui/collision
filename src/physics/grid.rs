// use std::ops::Index;

// use cgmath::Vector2;
// use ndarray::Array2;
// use smallvec::SmallVec;

// use super::object::Object;
// use crate::physics::object::ObjectId;

// pub struct GridBuilder {
//     grid: Grid,
// }

// impl GridBuilder {
//     pub fn new() -> Self {
//         GridBuilder { grid: Grid::new() }
//     }

//     pub fn add_object(&mut self, object: Object) -> ObjectId {
//         let index = self.grid.objects.len();
//         self.grid.objects.push(object);
//         ObjectId(index)
//     }

//     pub fn build(mut self) -> Grid {
//         let mut cell_size: f64 = 0.0;
//         let mut end = Vector2::new(f64::MIN, f64::MIN);

//         for &Object { position, radius, .. } in &self.grid.objects {
//             self.grid.position.x = self.grid.position.x.min(position.x);
//             self.grid.position.y = self.grid.position.y.min(position.y);
//             end.x = end.x.max(position.x);
//             end.y = end.y.max(position.y);
//             cell_size = cell_size.max(radius * 2.0);
//         }

//         if self.grid.position.x <= end.x && self.grid.position.y <= end.y {
//             self.grid.size.x = ((end.x - self.grid.position.x) / cell_size).ceil() as usize + 1;
//             self.grid.size.y = ((end.y - self.grid.position.y) / cell_size).ceil() as usize + 1;
//             self.grid.cells = Array2::from_elem((self.grid.size.x, self.grid.size.y), SmallVec::new());

//             for (id, &Object { position, .. }) in self
//                 .grid
//                 .objects
//                 .iter()
//                 .enumerate()
//                 .map(|(index, object)| (ObjectId(index), object))
//             {
//                 let cell = cell_at(position, self.grid.position, cell_size);
//                 self.grid.cells[(cell.x, cell.y)].push(id);
//             }
//         }

//         self.grid
//     }
// }

// pub struct Grid {
//     pub position: Vector2<f64>,
//     pub size: Vector2<usize>,
//     pub cell_size: f64,
//     pub objects: Vec<Object>,
//     pub cells: Array2<SmallVec<[ObjectId; 4]>>,
// }

// impl Grid {
//     pub fn new() -> Self {
//         Self {
//             position: Vector2::new(0.0, 0.0),
//             size: Vector2::new(0, 0),
//             cell_size: 0.0,
//             objects: Vec::new(),
//             cells: Array2::from_elem((0, 0), SmallVec::new()),
//         }
//     }

//     pub fn is_empty(&self) -> bool {
//         self.cells.is_empty()
//     }
// }

// impl Index<Cell> for Grid {
//     type Output = [ObjectId];
//     fn index(&self, id: Cell) -> &Self::Output {
//         &self.cells[(id.x, id.y)]
//     }
// }

// pub type Cell = Vector2<usize>;

// fn cell_at(position: Vector2<f64>, cells_start: Vector2<f64>, cell_size: f64) -> Cell {
//     Cell::new(
//         ((position.x - cells_start.x) / cell_size).floor() as usize,
//         ((position.y - cells_start.y) / cell_size).floor() as usize,
//     )
// }
