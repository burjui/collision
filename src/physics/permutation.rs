pub(crate) struct UniquePermutation2<'a, T> {
    items: &'a [T],
    state: u32,
}

impl<'a, T> UniquePermutation2<'a, T> {
    pub(crate) fn new(items: &'a [T]) -> Self {
        assert!(items.len() >= 2);
        Self { items, state: 0b11 }
    }
}

impl<'a, T> Iterator for UniquePermutation2<'a, T>
where
    T: Clone,
{
    type Item = (T, T);

    fn next(&mut self) -> Option<Self::Item> {
        let first = self.state.trailing_zeros() as usize;
        if first > self.items.len() - 2 {
            None
        } else {
            let second = (self.state & !(1 << first)).trailing_zeros() as usize;
            self.state = if second < self.items.len() - 1 {
                1 << first | 1 << (second + 1)
            } else {
                0b11 << (first + 1)
            };
            Some((self.items[first].clone(), self.items[second].clone()))
        }
    }
}

#[test]
fn n2k2() {
    assert_eq!(
        UniquePermutation2::new(&[1, 2])
            .collect::<Box<[_]>>()
            .as_ref(),
        &[(1, 2)]
    )
}

#[test]
fn n5k2() {
    assert_eq!(
        UniquePermutation2::new(&[1, 2, 3, 4, 5])
            .collect::<Box<[_]>>()
            .as_ref(),
        &[
            (1, 2),
            (1, 3),
            (1, 4),
            (1, 5),
            (2, 3),
            (2, 4),
            (2, 5),
            (3, 4),
            (3, 5),
            (4, 5),
        ]
    )
}
