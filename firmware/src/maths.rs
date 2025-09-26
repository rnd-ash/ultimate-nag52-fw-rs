use core::marker::PhantomData;

use num_traits::{AsPrimitive, Num, NumCast};

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
                    ret = interp_int(raw, *min_y, *max_y, *min_x, *max_x);
                    break;
                }
            }
        }
        ret
    }
}

pub fn interp_int(raw: i32, out_min: i32, out_max: i32, in_min: i32, in_max: i32) -> i32 {
    let clamped: i32 = core::cmp::max(in_min, core::cmp::min(raw, in_max));
    (((out_max - out_min) * (clamped - in_min)) / (in_max - in_min)) + out_min
}

/// First order average (Low pass signal filter)
///
/// This filter works by keeping track of f(x) and f(x-1)
///
/// The average is calculated (With sample size 'k' using the following equation):
/// `f(x) = (x + (f(x-1)*k)) / (k+1)`
#[derive(Default, Copy, Clone)]
pub struct FirstOrderAverage<T: Num + Copy + 'static, const SAMPLES: usize>
where
    f32: AsPrimitive<T>,
    f32: NumCast,
{
    last_sample: f32,
    current_sample: f32,
    _phantom: PhantomData<T>,
    total_samples: usize,
}

impl<T: Num + Copy + 'static, const SAMPLES: usize> FirstOrderAverage<T, SAMPLES>
where
    f32: AsPrimitive<T>,
    T: NumCast,
{
    pub fn new() -> Self {
        Self {
            last_sample: 0.0,
            current_sample: 0.0,
            _phantom: PhantomData::default(),
            total_samples: SAMPLES,
        }
    }

    pub fn add_sample(&mut self, val: T) -> Option<f32> {
        let sample_count_float: f32 = <f32 as NumCast>::from(self.total_samples)?;

        self.last_sample = self.current_sample;
        self.current_sample = (<f32 as NumCast>::from(val)?
            + (sample_count_float * self.last_sample))
            / (sample_count_float + 1.0);
        Some(self.current_sample)
    }

    pub fn get_average(&self) -> T {
        self.current_sample.as_()
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub async fn add_sample_c<F: AsyncFnOnce() -> T>(&mut self, f: F) -> T {
        let to_add = f().await;
        let _ = self.add_sample(to_add);
        self.get_average()
    }
}
