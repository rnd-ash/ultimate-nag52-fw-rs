#![no_std]

use maps::TcuNum;
pub mod egs_timers;
pub mod maps;


/// Interp between 2 values, where `val` is located somewhere on the
/// X axis (Between `x1` and `x2`) and the output is between `y1` and `y2`
/// 
/// If the val exceeds the bounds of `x1` or `x2`, then limits `y1` or `y2`
/// are returned
pub fn interp_linear<
    X: TcuNum, 
    Y: TcuNum,
>(val: impl Into<f32>, x1: X, x2: X, y1: Y, y2: Y) -> f32
{
    let x_min: X;
    let x_max: X;
    let y_min: Y;
    let y_max: Y;
    let val_f32 = val.into();
    
    if x1 > x2 {
        // Reverse X Y (Decending slope)
        x_min = x2;
        x_max = x1;
        y_min = y2;
        y_max = y1;
    } else if x1 < x2 {
        x_min = x1;
        x_max = x2;
        y_min = y1;
        y_max = y2;
    } else {
        // X1 == X2
        return y1.into()
    }
    // Limits check
    if val_f32 < x_min.into() {
        y_min.into()
    } else if val_f32 > x_max.into() {
        y_max.into()
    } else {
        // Do interpretation
        y_min.into() + ((y_max.into() - y_min.into()) / (x_max.into()-x_min.into())) * (val_f32 - x_min.into())
    }
}


pub const fn progress_between_targets(current: f32, start: f32, end: f32) -> f32 {
    (100.0 * (current-start)) / (end-start)
}

pub fn search_value<
    const N: usize,
    T: TcuNum
>(value: impl Into<f32>, values: &[T; N]) -> (usize, usize) {
    const {
        assert!(N != 0)
    }

    if N == 1 {
        (0,0)
    } else {
        let ascending = values[0] < values[N-1];
        let val_f32 = value.into();
        if ascending && val_f32 < values[0].into() {
            (0,0)
        } else if ascending && val_f32 > values[N-1].into() {
            (N-1, N-1)
        } else if !ascending && val_f32 > values[0].into() {
            (0,0)
        } else if !ascending && val_f32 < values[N-1].into() {
            (N-1, N-1)
        } else {
            let mut min_idx = 0;
            let mut max_idx = N-1;
            
            for idx in 0..N-1 {
                let min = values[idx].into();
                let max = values[idx+1].into();
                if (ascending && val_f32 >= min && val_f32 <= max) || (!ascending && val_f32 >= max && val_f32 <= min) {
                    min_idx = idx + (val_f32 == max) as usize;
                    max_idx = idx + (val_f32 != min) as usize;
                    break;
                }
            }
            (min_idx, max_idx)

        }
    }
}

/// Dummy function to test compiler error
/// when N < 2
/// 
/// ```compile_fail
/// let x = [0, 10i32];
/// search_value(5i32, &x);
/// ```
#[allow(dead_code)]
fn test_lookup_compile_fail(){}

#[cfg(test)]
pub mod math_tests {
    use super::*;

    #[test]
    pub fn test_lookup_gt() {
        let x = [0, 10, 20i16];
        assert_eq!((0,1), search_value(5i16, &x));

        let x = [0, 10, 20i16];
        assert_eq!((1,2), search_value(15i16, &x));
    }

    #[test]
    pub fn test_lookup_gte() {
        let x = [0, 10, 20i16];
        assert_eq!((1,1), search_value(10i16, &x));
        assert_eq!((2,2), search_value(20i16, &x));
    }
}