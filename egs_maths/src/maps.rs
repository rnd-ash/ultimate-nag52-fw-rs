use core::{
    ops::{Add, Deref, Div, Index, Mul, Sub},
    slice::SliceIndex,
};

use crate::search_value;

#[derive(Copy, Clone, PartialEq, PartialOrd)]
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

impl TcuNum for Safei32 {}

impl Div for Safei32 {
    type Output = i32;

    fn div(self, rhs: Self) -> Self::Output {
        self.0 / rhs.0
    }
}

pub trait TcuIdx<const MAX: usize>: Into<usize> {}

pub trait TcuNum: PartialOrd + PartialEq + Into<f32> + Copy + Clone {}

impl TcuNum for i16 {}
impl TcuNum for u16 {}

impl TcuNum for i8 {}
impl TcuNum for u8 {}

impl TcuNum for f32 {}

pub struct TupleMap<'a, X: TcuNum, Z: TcuNum, const N: usize>(&'a [(X, Z); N]);

impl<'a, X: TcuNum, Z: TcuNum, const N: usize> TupleMap<'a, X, Z, N> {
    pub const fn new(data: &'a [(X, Z); N]) -> Self {
        Self(data)
    }

    pub fn interp_1d<VX: TcuNum>(&self, raw: VX) -> f32 {
        let x = self.0.map(|(x, _)| x);
        let z = self.0.map(|(_, z)| z);

        let (min_idx, max_idx) = super::search_value(raw, &x);
        if min_idx != max_idx {
            super::interp_linear(raw, x[min_idx], x[max_idx], z[min_idx], z[max_idx])
        } else {
            z[min_idx].into()
        }
    }
}

pub struct Map1d<'a, X: TcuNum, Z: TcuNum, const N: usize> {
    x: &'a [X; N],
    z: &'a [Z; N],
}

impl<'a, X: TcuNum, Z: TcuNum, const N: usize> Map1d<'a, X, Z, N> {
    pub const fn new(x_axis: &'a [X; N], z_axis: &'a [Z; N]) -> Self {
        Self {
            x: x_axis,
            z: z_axis,
        }
    }

    pub fn interp_1d<VX: TcuNum>(&self, raw: VX) -> f32 {
        let (min_idx, max_idx) = super::search_value(raw, &self.x);
        if min_idx != max_idx {
            super::interp_linear(
                raw,
                self.x[min_idx],
                self.x[max_idx],
                self.z[min_idx],
                self.z[max_idx],
            )
        } else {
            self.z[min_idx].into()
        }
    }

    pub fn get_at<const XIDX: usize, XID: TcuIdx<XIDX>>(&self, x_idx: XID) -> Z {
        const {
            assert!(XIDX <= N);
        }
        todo!()
        //self.z[x_idx]
    }
}

#[macro_export]
macro_rules! declare_2d_map {
    ($name: ident, $xs: literal, $ys: literal, $xdt: ident, $ydt: ident, $zdt: ident) => {
        pub type $name<'a> = Map2d<'a, $xs, $ys, { $xs * $ys }, $xdt, $ydt, $zdt>;
    };
    ($name: ident, $xs: literal, $ys: literal, $zdt: ident) => {
        pub type $name<'a> = Indexed2dMap<'a, $xs, $ys, { $xs * $ys }, $zdt>;
    };
}

pub struct Indexed2dMap<'a, const XS: usize, const YS: usize, const ZS: usize, Z: TcuNum> {
    z: &'a [Z; ZS],
}

impl<'a, const XS: usize, const YS: usize, const ZS: usize, Z: TcuNum>
    Indexed2dMap<'a, XS, YS, ZS, Z>
{
    pub const fn new(z_axis: &'a [Z; ZS]) -> Self {
        const { assert!(ZS == XS * YS) }
        Self { z: z_axis }
    }

    pub fn get_at<const XIDX: usize, const YIDX: usize, XID: TcuIdx<XIDX>, YID: TcuIdx<YIDX>>(
        &self,
        x_idx: XID,
        y_idx: YID,
    ) -> Z {
        const {
            assert!(XIDX <= XS);
            assert!(YIDX <= YS);
        }
        self.z_as_rows()[x_idx.into()][y_idx.into()]
    }

    pub const fn z_as_rows(&self) -> &[[Z; XS]; YS] {
        unsafe { core::mem::transmute(self.z) }
    }

    pub const fn z_as_rows_mut(&mut self) -> &mut [[Z; XS]; YS] {
        unsafe { core::mem::transmute(&mut self.z) }
    }
}

pub struct Map2d<
    'a,
    const XS: usize,
    const YS: usize,
    const ZS: usize,
    X: TcuNum,
    Y: TcuNum,
    Z: TcuNum,
> {
    x: &'a [X; XS],
    y: &'a [Y; YS],
    z: &'a [Z; ZS],
}

impl<'a, const XS: usize, const YS: usize, const ZS: usize, X: TcuNum, Y: TcuNum, Z: TcuNum>
    Map2d<'a, XS, YS, ZS, X, Y, Z>
{
    pub const fn new(x_axis: &'a [X; XS], y_axis: &'a [Y; YS], z_axis: &'a [Z; ZS]) -> Self {
        const { assert!(ZS == XS * YS) }
        Self {
            x: x_axis,
            y: y_axis,
            z: z_axis,
        }
    }

    pub fn interp<VX: TcuNum, VY: TcuNum>(&self, x_val: VX, y_val: VY) -> f32 {
        // Actual logic
        let (x_min_idx, x_max_idx) = search_value(x_val, &self.x);
        let (y_min_idx, y_max_idx) = search_value(y_val, &self.y);

        let f_11 = self.z[(y_min_idx * XS) + x_min_idx];
        let f_12 = self.z[(y_min_idx * XS) + x_max_idx];
        let f_21 = self.z[(y_max_idx * XS) + x_min_idx];
        let f_22 = self.z[(y_max_idx * XS) + x_max_idx];

        // Bilinear interpolation
        let f_11_f_12_interp =
            super::interp_linear(x_val, self.x[x_min_idx], self.x[x_max_idx], f_11, f_12);
        let f_21_f_22_interp =
            super::interp_linear(x_val, self.x[x_min_idx], self.x[x_max_idx], f_21, f_22);
        super::interp_linear(
            y_val,
            self.y[y_min_idx],
            self.y[y_max_idx],
            f_11_f_12_interp,
            f_21_f_22_interp,
        )
    }

    pub fn get_at<const XIDX: usize, const YIDX: usize, XID: TcuIdx<XIDX>, YID: TcuIdx<YIDX>>(
        &self,
        x_idx: XID,
        y_idx: YID,
    ) -> Z {
        const {
            assert!(XIDX <= XS);
            assert!(YIDX <= YS);
        }
        todo!()
        //self.z_as_rows()[x_idx.into()][y_idx.into()]
    }

    pub const fn z_as_rows(&self) -> &[[Z; XS]; YS] {
        unsafe { core::mem::transmute(self.z) }
    }

    pub const fn z_as_rows_mut(&mut self) -> &mut [[Z; XS]; YS] {
        unsafe { core::mem::transmute(&mut self.z) }
    }
}

#[cfg(test)]
pub mod maps_tests {
    use super::*;

    #[test]
    pub fn test_tuple_map() {
        let data: [(u8, i16); 3] = [(20, -5), (10, 0), (0, 5)];

        let lookup: TupleMap<'_, _, _, _> = TupleMap::new(&data);
        let res = lookup.interp_1d(20i16);
        assert_eq!(-5.0, res);
        let res = lookup.interp_1d(15i16);
        assert_eq!(-2.5, res);
    }

    #[test]
    pub fn test_1d_map() {
        let x: [u8; 3] = [10, 20, 30];
        let z: [i16; 3] = [-5, 5, 10];

        let lookup: Map1d<'_, _, _, _> = Map1d::new(&x, &z);
        let res = lookup.interp_1d(20i16);
        assert_eq!(5.0, res);
        let res = lookup.interp_1d(15i16);
        assert_eq!(0.0, res);
    }

    #[test]
    pub fn test_2d_map() {
        let x: [u8; 3] = [0, 10, 20];
        let y: [i16; 3] = [-100, 0, 100];
        let z: [i16; 9] = [1, 2, 3, 4, 5, 6, 7, 8, 9];

        let map = Map2d::new(&x, &y, &z);
        // Center point test
        let res = map.interp(10i16, 0i16);
        assert_eq!(5.0, res);
        // X axis linear test
        let res = map.interp(15i16, 0i16);
        assert_eq!(5.5, res);
        // Y axis linear test
        let res = map.interp(10i16, -50i16);
        assert_eq!(3.5, res);
        // Both X and Y test together
        let res = map.interp(15i16, -50i16);
        assert_eq!(4.0, res);
        // Out of bounds test (Should be clamped)
        let res = map.interp(200i16, 200i16);
        assert_eq!(9.0, res);

        let z_as_rows = map.z_as_rows();
        assert_eq!([1, 2, 3], z_as_rows[0]);
        assert_eq!([4, 5, 6], z_as_rows[1]);
        assert_eq!([7, 8, 9], z_as_rows[2]);
    }
}
