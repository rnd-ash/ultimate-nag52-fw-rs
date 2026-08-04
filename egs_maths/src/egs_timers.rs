use crate::maps::TcuNum;

#[derive(Default, Copy, Clone)]
pub struct EgsCountDownTimer(u16);

impl EgsCountDownTimer {
    pub fn new(v: u16) -> Self {
        Self(v)
    }

    /// Decrement the timer, should be done
    /// every EGS cycle
    pub fn decrement(&mut self) {
        self.0 = self.0.saturating_sub(1)
    }

    pub fn reset(&mut self) {
        self.0 = 0;
    }

    pub fn interp_value<T: TcuNum>(&self, start: T, end: T) -> f32 {
        if self.0 == 0 {
            end.into()
        } else {
            if start > end {
                let delta: f32 = (end.into() - start.into()) / (self.0 as f32);
                start.into() + delta
            } else {
                let delta: f32 = (start.into() - end.into()) / (self.0 as f32);
                start.into() - delta
            }
        }
    }
}
