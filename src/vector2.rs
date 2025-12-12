use std::{
    fmt,
    iter::Sum,
    marker::PhantomData,
    ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign},
};

use num_traits::Float;
use serde::{
    Deserialize,
    de::{Error, Visitor},
};

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Vector2<T> {
    pub x: T,
    pub y: T,
}

impl<T> Vector2<T> {
    pub const fn new(x: T, y: T) -> Self {
        Self { x, y }
    }

    pub fn magnitude(&self) -> T
    where
        T: Float,
    {
        self.magnitude_squared().sqrt()
    }

    pub fn magnitude_squared(&self) -> T
    where
        T: Mul<Output = T> + Add<Output = T> + Copy,
    {
        self.dot(*self)
    }

    #[must_use]
    pub fn normalize(&self) -> Vector2<T>
    where
        T: Float + Div<Output = T>,
    {
        let one_over_magnitude = T::one() / self.magnitude().max(T::epsilon());
        *self * one_over_magnitude
    }

    pub fn dot(&self, other: Vector2<T>) -> T
    where
        T: Mul<Output = T> + Add<Output = T> + Copy,
    {
        let product = *self * other;
        product.x + product.y
    }
}

impl<T: Default> Default for Vector2<T> {
    fn default() -> Self {
        Self {
            x: T::default(),
            y: T::default(),
        }
    }
}

impl<T: PartialEq> PartialEq for Vector2<T> {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl<T> Neg for Vector2<T>
where
    T: Neg<Output = T>,
{
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self { x: -self.x, y: -self.y }
    }
}

impl<T> Add for Vector2<T>
where
    T: Add<Output = T>,
{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl<T: Copy> Add<T> for Vector2<T>
where
    T: Add<Output = T>,
{
    type Output = Self;

    fn add(self, rhs: T) -> Self::Output {
        Self {
            x: self.x + rhs,
            y: self.y + rhs,
        }
    }
}

impl<T> AddAssign for Vector2<T>
where
    T: AddAssign,
{
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl<T> Sub for Vector2<T>
where
    T: Sub<Output = T>,
{
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl<T: Copy> Sub<T> for Vector2<T>
where
    T: Sub<Output = T>,
{
    type Output = Self;

    fn sub(self, rhs: T) -> Self::Output {
        Self {
            x: self.x - rhs,
            y: self.y - rhs,
        }
    }
}

impl<T> SubAssign for Vector2<T>
where
    T: SubAssign,
{
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl<T> Mul<T> for Vector2<T>
where
    T: Mul<Output = T> + Copy,
{
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl<T> Mul<Vector2<T>> for Vector2<T>
where
    T: Mul<Output = T> + Copy,
{
    type Output = Self;

    fn mul(self, rhs: Vector2<T>) -> Self::Output {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
        }
    }
}

impl<T> MulAssign<T> for Vector2<T>
where
    T: MulAssign + Copy,
{
    fn mul_assign(&mut self, rhs: T) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl<T> MulAssign<Vector2<T>> for Vector2<T>
where
    T: MulAssign + Copy,
{
    fn mul_assign(&mut self, rhs: Vector2<T>) {
        self.x *= rhs.x;
        self.y *= rhs.y;
    }
}

impl<T> Div<T> for Vector2<T>
where
    T: Div<Output = T> + Copy,
{
    type Output = Self;

    fn div(self, rhs: T) -> Self::Output {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

impl Sum for Vector2<f32> {
    fn sum<I>(iter: I) -> Self
    where
        I: Iterator<Item = Self>,
    {
        iter.fold(Vector2::default(), Add::add)
    }
}

impl From<(f32, f32)> for Vector2<f32> {
    fn from(tuple: (f32, f32)) -> Self {
        Self { x: tuple.0, y: tuple.1 }
    }
}

impl<T: fmt::Debug> fmt::Debug for Vector2<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({:?}, {:?})", self.x, self.y)
    }
}

impl<'de, T: Deserialize<'de>> Deserialize<'de> for Vector2<T> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        deserializer.deserialize_tuple(2, Vector2Visitor::<T>(PhantomData))
    }
}

struct Vector2Visitor<T>(PhantomData<T>);

impl<'de, T: Deserialize<'de>> Visitor<'de> for Vector2Visitor<T> {
    type Value = Vector2<T>;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a pair of numbers")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: serde::de::SeqAccess<'de>,
    {
        if let (Some(x), Some(y)) = (seq.next_element()?, seq.next_element()?) {
            Ok(Vector2 { x, y })
        } else {
            Err(Error::custom("expected a pair of numbers"))
        }
    }
}
