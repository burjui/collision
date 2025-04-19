#[derive(Default)]
pub struct Array2<T> {
    data: Vec<T>,
    size: (usize, usize),
}

impl<T: Default + Copy> Array2<T> {
    pub fn new(size: (usize, usize)) -> Self {
        Self {
            data: vec![T::default(); size.0 * size.1],
            size,
        }
    }

    #[must_use]
    pub fn size(&self) -> (usize, usize) {
        self.size
    }

    pub fn reset(&mut self, size: (usize, usize)) {
        if size.0 > self.size.0 || size.1 > self.size.1 {
            self.data.clear();
            self.data.resize(size.0 * size.1, T::default());
        } else {
            self.data.fill(T::default());
        }
        self.size = size;
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    #[must_use]
    pub fn data(&self) -> &[T] {
        &self.data
    }

    #[must_use]
    pub fn data_mut(&mut self) -> &mut [T] {
        &mut self.data
    }
}

impl<T> AsRef<[T]> for Array2<T> {
    fn as_ref(&self) -> &[T] {
        &self.data
    }
}

impl<T> std::ops::Index<(usize, usize)> for Array2<T> {
    type Output = T;
    fn index(&self, index: (usize, usize)) -> &Self::Output {
        assert!(index.0 < self.size.0 && index.1 < self.size.1, "index {index:?} is out of bounds {:?}", self.size);
        &self.data[index.1 * self.size.0 + index.0]
    }
}

impl<T> std::ops::IndexMut<(usize, usize)> for Array2<T> {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        assert!(index.0 < self.size.0 && index.1 < self.size.1, "index {index:?} is out of bounds {:?}", self.size);
        &mut self.data[index.1 * self.size.0 + index.0]
    }
}

impl<T: Copy> Clone for Array2<T> {
    fn clone(&self) -> Self {
        Self {
            data: self.data.clone(),
            size: self.size,
        }
    }
}
