use crate::first_order_filter_i32;

const MUL_FACTOR_PRECISION: i32 = 100;

#[repr(C)]
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct FilteredMonitoredVal {
    val: i32,
    last_raw_val: i32,
    e_counter: u8,
    e_max: u8,
}

impl FilteredMonitoredVal {
    pub const fn new(e_max: u8) -> Self {
        Self {
            val: 0,
            last_raw_val: 0,
            e_counter: 0,
            e_max,
        }
    }

    pub const fn new_with_init_value(v: i32, e_max: u8) -> Self {
        Self {
            val: v,
            last_raw_val: v,
            e_counter: 0,
            e_max,
        }
    }

    pub fn value(&self) -> Option<i32> {
        if self.e_max != 0 && self.e_counter >= self.e_max {
            None
        } else {
            Some(self.val / MUL_FACTOR_PRECISION)
        }
    }

    // Adds a new value to the filter, filters by `filter_factor`,
    // and then returns the new filtered value.
    //
    // Passing filter factor of 0 disables filter (Out = new)
    pub fn add_valid_value(&mut self, new: i32, filter_factor: u8) -> i32 {
        self.val = first_order_filter_i32(new * MUL_FACTOR_PRECISION, self.val, filter_factor) as _;
        self.last_raw_val = new;
        self.val
        //self.value()
    }

    pub fn mark_invalid(&mut self) {
        self.e_counter = self.e_counter.saturating_add(1).min(self.e_max);
    }
}
