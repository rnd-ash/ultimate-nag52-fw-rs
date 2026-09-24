use crate::tcu_num::TcuNum;

#[derive(Default, Copy, Clone)]
#[repr(C)]
pub struct EgsCountDownTimer(u16);

pub const EGS_TIMER_EMPTY: EgsCountDownTimer = EgsCountDownTimer(0);

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

    pub const fn ended(&self) -> bool {
        self.0 == 0
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

#[cfg(test)]
pub mod timer_tests {
    use crate::egs_timers::EgsCountDownTimer;

    #[test]
    pub fn test_countdown() {
        let mut timer = EgsCountDownTimer::new(2);
        timer.decrement();
        assert_eq!(timer.0, 1);
        timer.decrement();
        assert_eq!(timer.0, 0);
        timer.decrement();
        assert_eq!(timer.0, 0);
    }

    #[test]
    pub fn test_interp() {
        let mut current_v = 0i16;
        let targ_v = 10i16;
        const STEPS: usize = 10;
        let mut timer = EgsCountDownTimer::new(STEPS as _);

        let mut step_prog = [0; STEPS];
        for i in 0..STEPS {
            current_v = timer.interp_value(current_v, targ_v) as i16;
            timer.decrement();
            step_prog[i] = current_v;
        }
        assert_eq!(step_prog, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
    }
}
