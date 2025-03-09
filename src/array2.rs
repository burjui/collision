pub struct Array2<T> {
    data: Vec<T>,
    size: (usize, usize),
}

impl<T: Default + Copy> Array2<T> {
    #[must_use]
    pub fn default(size: (usize, usize)) -> Self {
        let mut data = Vec::default();
        data.resize(size.0 * size.1, T::default());
        Self { data, size }
    }

    #[must_use]
    pub fn size(&self) -> (usize, usize) {
        self.size
    }

    pub fn reset(&mut self, size: (usize, usize)) {
        self.data.clear();
        self.data.resize(size.0 * size.1, T::default());
        self.size = size;
    }
}

impl<T> std::ops::Index<(usize, usize)> for Array2<T> {
    type Output = T;
    fn index(&self, index: (usize, usize)) -> &Self::Output {
        assert!(index.0 < self.size.0 && index.1 < self.size.1);
        &self.data[index.1 * self.size.0 + index.0]
    }
}

impl<T> std::ops::IndexMut<(usize, usize)> for Array2<T> {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        assert!(index.0 < self.size.0 && index.1 < self.size.1);
        &mut self.data[index.1 * self.size.0 + index.0]
    }
}
