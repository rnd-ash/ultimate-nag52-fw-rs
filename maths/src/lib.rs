#![no_std]

pub use num_traits::*;

use maps::MapNum;
//pub mod egs_timers;
pub mod maps;

pub fn interp(raw: i32, tab: &[(i32, i32)]) -> i32 {
    if raw <= tab.first().unwrap().0 {
        tab.first().unwrap().1
    } else if raw >= tab.last().unwrap().0 {
        tab.last().unwrap().1
    } else {
        let mut ret = tab.first().unwrap().1;
        for window in tab.windows(2) {
            let mut iter = window.iter();
            if let (Some((min_x, min_y)), Some((max_x, max_y))) = (iter.next(), iter.next()) {
                if (*min_x..=*max_x).contains(&raw) {
                    ret = interp_linear(raw, *min_x, *min_y, *min_y, *max_y) as i32;
                    break;
                }
            }
        }
        ret
    }
}

/// Interp between 2 values, where `val` is located somewhere on the
/// X axis (Between `x1` and `x2`) and the output is between `y1` and `y2`
/// 
/// If the val exceeds the bounds of `x1` or `x2`, then limits `y1` or `y2`
/// are returned
pub fn interp_linear<
    X: Num + PartialOrd + AsPrimitive<f32>, 
    Y: Num + PartialOrd + AsPrimitive<f32>
>(val: X, x1: X, x2: X, y1: Y, y2: Y) -> f32
where f32: AsPrimitive<Y>
{
    let x_min: X;
    let x_max: X;
    let y_min: Y;
    let y_max: Y;
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
        return y1.as_()
    }
    // Limits check
    if val < x_min {
        y_min.as_()
    } else if val > x_max {
        y_max.as_()
    } else {
        // Do interpretation
        y_min.as_() + ((y_max.as_() - y_min.as_()) / (x_max.as_()-x_min.as_())) * (val.as_() - x_min.as_())
    }
}

pub fn first_order_filter_in_place<T: Num>(_samples: u32, _new_value: T, _previous_val: &mut T) {
}

pub const fn progress_between_targets(current: f32, start: f32, end: f32) -> f32 {
    (100.0 * (current-start)) / (end-start)
}

pub fn search_value<
    const N: usize,
    T: MapNum
>(value: T, values: &[T; N]) -> (usize, usize) {
    const {
        assert!(N >= 2)
    }

    let mut min = 0;
    let mut max = N - 1;

    if value > values[N-1] {
        min = N-1;
    } else if value < values[0] {
        max = 0;
    } else {
        // Search
        for (idx, window) in values.windows(2).into_iter().enumerate() {
            if let &[lb, ub] = window {
                // Found range (Ascending or Descending)
                if value > lb && value < ub || value > ub && value < lb {
                    // Value is in between 2 values
                    min = idx;
                    max = min + 1;
                    break;
                } else if value == lb {
                    // Value is equal to the lower element
                    min = idx;
                    max = idx;
                    break;
                } else if value == ub {
                    // Value is equal to the upper element
                    min = idx + 1;
                    max = idx + 1;
                    break;
                }
            }
        }
            
    }

    (min, max)
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
    use num_traits::AsPrimitive;

    #[test]
    pub fn test_lookup_gt() {
        let x = [0, 10i32];
        assert_eq!((0,1), search_value(5i32, &x));

        let x = [0, 10, 20i32];
        assert_eq!((0,1), search_value(5i32, &x));
    }

    #[test]
    pub fn test_lookup_gte() {
        let x = [0, 10, 20i32];
        assert_eq!((1,1), search_value(10i32, &x));
        assert_eq!((2,2), search_value(20i32, &x));
    }
}