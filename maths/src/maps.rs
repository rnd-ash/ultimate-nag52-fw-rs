use core::{ops::{Deref}};

use num_traits::{AsPrimitive, Num};

use crate::search_value;

pub trait MapNum: AsPrimitive<f32> + Num + PartialOrd + PartialEq{}

impl MapNum for i32 {}
impl MapNum for u32 {}

impl MapNum for i16 {}
impl MapNum for u16 {}

impl MapNum for i8 {}
impl MapNum for u8 {}

impl MapNum for f32 {}

pub struct Map1d<'a, T: MapNum, const N: usize>(&'a [T; N]);

impl<'a, T: MapNum, const N: usize> Deref for Map1d<'a, T, N> {
    type Target = [T; N];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<'a, T: MapNum, const N: usize> From<&'a [T; N]> for Map1d<'a, T, N> {
    fn from(value: &'a [T; N]) -> Self {
        Self(value)
    }
}

impl<'a, T: MapNum, const N: usize> Map1d<'a, T, N> {
    pub fn interp_1d<V: MapNum, const VS: usize>(&self, raw: T, z: &[V; VS]) -> f32
    where f32: num_traits::AsPrimitive<V> {
        const {
            assert!(N == VS);
        }
        let (min_idx, max_idx) = super::search_value(raw, &self.0);
        if min_idx != max_idx {
            super::interp_linear(raw, self[min_idx], self[max_idx], z[min_idx], z[max_idx])
        } else {
            z[min_idx].as_()
        }
    }
}


pub struct Map2d
<
    'a,
    const XS: usize,
    const YS: usize,
    X: MapNum,
    Y: MapNum,
>
{
    x: Map1d<'a, X, XS>,
    y: Map1d<'a, Y, YS>,
}

impl<'a,
    const XS: usize,
    const YS: usize,
    X: MapNum,
    Y: MapNum> Map2d<'a, XS, YS, X, Y> {

        pub fn new(x_axis: &'a [X; XS], y_axis: &'a [Y; YS]) -> Self {
            Self {
                x: x_axis.into(),
                y: y_axis.into(),
            }
        }

        pub fn interp<const ZS: usize, Z: MapNum>(&self, x_val: X, y_val: Y, z: &[Z; ZS]) -> f32
        where f32: AsPrimitive<Y>, f32: AsPrimitive<Z> {
            // Const bounds check
            const {
                assert!(ZS == XS*YS);
            }
            // Actual logic
            let (x_min_idx, x_max_idx) = search_value(x_val, &self.x);
            let (y_min_idx, y_max_idx) = search_value(y_val, &self.y);

            let f_11 = z[(y_min_idx * XS) + x_min_idx];
            let f_12 = z[(y_min_idx * XS) + x_max_idx];
            let f_21 = z[(y_max_idx * XS) + x_min_idx];
            let f_22 = z[(y_max_idx * XS) + x_max_idx];

            // Bilinear interpolation
            let f_11_f_12_interp = super::interp_linear::<X, Z>(x_val, self.x[x_min_idx], self.x[x_max_idx], f_11, f_12);
            let f_21_f_22_interp = super::interp_linear::<X, Z>(x_val, self.x[x_min_idx], self.x[x_max_idx], f_21, f_22);
            super::interp_linear::<Y, f32>(y_val, self.y[y_min_idx], self.y[y_max_idx], f_11_f_12_interp, f_21_f_22_interp)

        }
    }




#[cfg(test)]
pub mod maps_tests {
    use super::*;

    #[test]
    pub fn test_1d_map() {
        let x: [u8; 3] = [10, 20, 30];
        let z: [i32; 3] = [-5, 5, 10];

        let lookup: Map1d<'_, _, _> = (&x).into();
        let res = lookup.interp_1d(20, &z);
        assert_eq!(5.0, res);
        let res = lookup.interp_1d(15, &z);
        assert_eq!(0.0, res);
    }

    #[test]
    pub fn test_2d_map() {
        let x: [u8; 3] = [0, 10, 20];
        let y: [i32; 3] = [-100, 0, 100];
        let z: [i32; 9] = [
            1,2,3,
            4,5,6,
            7,8,9
        ];

        let map = Map2d::new(&x, &y);
        // Center point test
        let res = map.interp(10, 0, &z);
        assert_eq!(5.0, res);
        // X axis linear test
        let res = map.interp(15, 0, &z);
        assert_eq!(5.5, res);
        // Y axis linear test
        let res = map.interp(10, -50, &z);
        assert_eq!(3.5, res);
        // Both X and Y test together
        let res = map.interp(15, -50, &z);
        assert_eq!(4.0, res);
        // Out of bounds test (Should be clamped)
        let res = map.interp(200, 200, &z);
        assert_eq!(9.0, res);
    }
}