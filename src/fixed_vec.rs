use std::ops::{Deref, DerefMut};

#[derive(Clone, Copy)]
pub struct FixedVec<T, const N: usize> {
    inner: [T; N],
    len: usize,
}

impl<T, const N: usize> FixedVec<T, N> {
    pub fn push(&mut self, value: T) {
        assert!(self.len < N, "vector is full ({N})");
        self.inner[self.len] = value;
        self.len += 1;
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn as_slice(&self) -> &[T] {
        &self.inner[..self.len]
    }
}

impl<T: Default + Copy, const N: usize> Default for FixedVec<T, N> {
    fn default() -> Self {
        Self {
            inner: [Default::default(); N],
            len: 0,
        }
    }
}

impl<T: Copy + Default, const N: usize> FromIterator<T> for FixedVec<T, N> {
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let mut data = [Default::default(); N];
        let mut len = 0;
        for item in iter {
            assert!(len < N);
            data[len] = item;
            len += 1;
        }
        Self { inner: data, len }
    }
}

impl<T, const N: usize> Deref for FixedVec<T, N> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        &self.inner[..self.len]
    }
}

impl<T, const N: usize> DerefMut for FixedVec<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner[..self.len]
    }
}

#[test]
fn from_iter() {
    let mut vec = FixedVec::<i32, 4>::from_iter([1, 2, 3]);
    assert_eq!(vec.as_slice(), &[1, 2, 3]);
    vec.push(4);
    assert_eq!(vec.as_slice(), &[1, 2, 3, 4]);
}

#[test]
fn default() {
    let mut vec: FixedVec<i32, 4> = Default::default();
    assert_eq!(vec.as_slice(), &[]);
    vec.push(1);
    vec.push(2);
    vec.push(3);
    vec.push(4);
    assert_eq!(vec.as_slice(), &[1, 2, 3, 4]);
}

#[test]
#[should_panic]
fn overflow() {
    let mut vec = FixedVec::<i32, 4>::from_iter([1, 2, 3, 4]);
    vec.push(5);
}
