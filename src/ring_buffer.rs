use std::fmt::Debug;

#[derive(Clone)]
pub struct RingBuffer<const N: usize, T> {
    data: [T; N],
    front: usize,
    length: usize,
}

impl<const N: usize, T> RingBuffer<N, T> {
    pub fn len(&self) -> usize {
        self.length
    }

    pub fn is_empty(&self) -> bool {
        self.length == 0
    }

    pub fn push(&mut self, item: T) {
        let new_length = (self.length + 1).min(N);
        let index = (self.front + self.length) % N;
        self.front = (self.front + (self.length.checked_sub(N).unwrap_or(1) ^ 1)) % N;
        self.length = new_length;
        self.data[index] = item;
    }
}

impl<const N: usize, T: Default + Copy> Default for RingBuffer<N, T> {
    fn default() -> Self {
        Self {
            data: [T::default(); N],
            front: 7,
            length: 0,
        }
    }
}

impl<const N: usize, T: Copy> Iterator for RingBuffer<N, T> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.length == 0 {
            None
        } else {
            let item = self.data[self.front];
            self.length -= 1;
            self.front = (self.front + 1) % N;
            Some(item)
        }
    }
}

impl<const N: usize, T: Debug + Copy> Debug for RingBuffer<N, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for item in self.clone() {
            Debug::fmt(&item, f)?
        }
        Ok(())
    }
}

#[test]
fn ring_buffer() {
    const N: usize = 7;
    const EXCESS: usize = 20;
    for front in 0..N {
        let mut buffer = RingBuffer {
            data: [0; N],
            front,
            length: 0,
        };
        for i in 0..N + EXCESS {
            buffer.push(i);
        }
        let mut iter = buffer.clone();
        for i in 0..N {
            assert_eq!(iter.next(), Some(i + EXCESS));
        }
    }
}
