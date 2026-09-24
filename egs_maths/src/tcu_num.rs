use core::ops::{Deref, Div};

/// A numerical type that can be guaranteed to be represented as a f32
/// without any precision loss
pub trait TcuNum: PartialOrd + PartialEq + Into<f32> + Copy + Clone {}

/// An i32 that fits within the numerical bounds of f32
#[derive(Copy, Clone, PartialEq, PartialOrd, Eq, Ord)]
pub struct Safei32(i32);

impl Safei32 {
    pub const fn new<const N: i32>() -> Self {
        const {
            assert!(N <= f32::MAX as i32);
            assert!(N >= f32::MIN as i32);
        }
        Self(N)
    }
}

impl Deref for Safei32 {
    type Target = i32;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Into<f32> for Safei32 {
    fn into(self) -> f32 {
        self.0 as _
    }
}

impl Div for Safei32 {
    type Output = i32;

    fn div(self, rhs: Self) -> Self::Output {
        self.0 / rhs.0
    }
}

/// An u32 that fits within the numerical bounds of f32
#[derive(Copy, Clone, PartialEq, PartialOrd, Eq, Ord)]
pub struct Safeu32(u32);

impl Safeu32 {
    pub const fn new<const N: u32>() -> Self {
        const {
            assert!(N <= f32::MAX as u32);
        }
        Self(N)
    }
}

impl Deref for Safeu32 {
    type Target = u32;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Into<f32> for Safeu32 {
    fn into(self) -> f32 {
        self.0 as _
    }
}

impl Div for Safeu32 {
    type Output = u32;

    fn div(self, rhs: Self) -> Self::Output {
        self.0 / rhs.0
    }
}

impl TcuNum for Safei32 {}
impl TcuNum for Safeu32 {}

impl TcuNum for i16 {}
impl TcuNum for u16 {}

impl TcuNum for i8 {}
impl TcuNum for u8 {}

impl TcuNum for f32 {}
