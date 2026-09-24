#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
/// Action to do on an OK signal being received,
/// if the error counter is not 0
pub enum PassOkAction {
    /// Reset error counter to 0
    Reset,
    /// Decrement the counter by 'n'
    Decrement(u8),
}

pub struct CounterDebouncer {
    /// Threshold for confirmed DTC
    e_counter_max: u8,
    /// Current error counter
    e_counter: u8,
    act: PassOkAction,
}

impl CounterDebouncer {
    pub const fn new(threshold: u8, action: PassOkAction) -> Self {
        Self {
            e_counter_max: threshold,
            e_counter: 0,
            act: action,
        }
    }

    /// Mark associated signal as OK this pass
    pub fn mark_pass_ok(&mut self) {
        self.e_counter = match self.act {
            PassOkAction::Reset => 0,
            PassOkAction::Decrement(sub) => self.e_counter.saturating_sub(sub),
        };
    }

    /// mark associated signal as an error this pass
    pub fn mark_pass_error(&mut self) {
        self.e_counter = self.e_counter.saturating_add(1);
    }

    /// Debouncer has confirmed this error is pending
    pub fn pending(&self) -> bool {
        self.e_counter != 0
    }

    /// Debouncer has confirmed this error exists
    pub fn confirmed(&self) -> bool {
        self.e_counter >= self.e_counter_max
    }
}

/// A DTC bouncer that uses time based constants
pub struct TimerDebouncer {
    /// Detection time of the error to begin with
    first_detect_time_ms: Option<u32>,
    /// Time in ms for the fault to occur
    /// before confirming the fault
    confirmed_window_ms: u32,
}

impl TimerDebouncer {
    pub const fn new(confirm_time_ms: u32) -> Self {
        Self {
            first_detect_time_ms: None,
            confirmed_window_ms: confirm_time_ms,
        }
    }

    /// Mark associated signal as OK this pass
    pub fn mark_pass_ok(&mut self) {
        self.first_detect_time_ms = None;
    }

    /// mark associated signal as an error this pass
    pub fn mark_pass_error(&mut self, now_ms: u32) {
        if self.first_detect_time_ms.is_none() {
            self.first_detect_time_ms = Some(now_ms)
        }
    }

    /// Debouncer has confirmed this error is pending
    pub fn pending(&self) -> bool {
        self.first_detect_time_ms.is_some()
    }

    /// Debouncer has confirmed this error exists
    pub fn confirmed(&self, now_ms: u32) -> bool {
        if let Some(start_mark) = self.first_detect_time_ms {
            now_ms - start_mark > self.confirmed_window_ms
        } else {
            false
        }
    }
}
