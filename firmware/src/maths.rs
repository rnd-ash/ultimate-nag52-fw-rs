
use num_traits::Num;

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

pub fn first_order_filter_in_place<T: Num>(_samples: u32, _new_value: T, _previous_val: &mut T) {
    
}