pub struct Array2<T> {
    data: Box<[T]>,
    size: (usize, usize),
}

impl<T: Default + Copy> Array2<T> {
    pub fn default(size: (usize, usize)) -> Self {
        let capacity = size.0 * size.1;
        let mut data = Vec::with_capacity(capacity);
        data.resize(capacity, T::default());
        let data = data.into_boxed_slice();
        Self { data, size }
    }
}

impl<T> std::ops::Index<(usize, usize)> for Array2<T> {
    type Output = T;
    fn index(&self, index: (usize, usize)) -> &Self::Output {
        &self.data[index.1 * self.size.0 + index.0]
    }
}

impl<T> std::ops::IndexMut<(usize, usize)> for Array2<T> {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.data[index.1 * self.size.0 + index.0]
    }
}
