use num_traits::{Num, FromPrimitive, AsPrimitive};

pub struct EgsCountDownTimer(u16);

impl EgsCountDownTimer {
    /// Decrement the timer, should be done
    /// every EGS cycle
    pub fn decrement(&mut self) {
        self.0 = self.0.wrapping_sub(1)
    }

    pub fn reset(&mut self) {
        self.0 = 0;
    }

    pub fn interp_value<T: Num + FromPrimitive + AsPrimitive<f32> + PartialOrd>(&self, start: T, end: T) -> T {
        if self.0 == 0 {
            end
        } else {
            let res: f32 = if start > end {
                let delta: f32 = (end-start).as_()/(self.0 as f32);
                start.as_() + delta
            } else {
                let delta: f32 = (start-end).as_()/(self.0 as f32);
                start.as_() - delta
            };
            T::from_f32(res).unwrap()
        }
    }
}