use egs_logic::calbrations::mech::MechCal;
use egs_logic::Clutch;
use egs_maths::{first_order_filter, interp_linear};


pub const SIM_CLUTCH_K1: SimClutch = SimClutch::new(Clutch::K1, 0.01, 0.0, 4, 10, 10, 2, 1270, None);
pub const SIM_CLUTCH_K2: SimClutch = SimClutch::new(Clutch::K2, 0.01, 0.0, 5, 15, 10, 2, 846, Some(40000));
pub const SIM_CLUTCH_K3: SimClutch = SimClutch::new(Clutch::K3, 0.05, 0.01, 6, 20, 10, 2, 1205, Some(3000));

pub const SIM_CLUTCH_B1: SimClutch = SimClutch::new(Clutch::B1, 0.05, 0.01, 4, 10, 20, 2, 1270, None);
pub const SIM_CLUTCH_B2: SimClutch = SimClutch::new(Clutch::B2, 0.02, 0.01, 5, 15, 24, 2, 846, None);
pub const SIM_CLUTCH_B3: SimClutch = SimClutch::new(Clutch::B3, 0.02, 0.01, 6, 20, 28, 2, 1205, None);

#[derive(Clone, Copy)]
pub struct SimClutch {
    pub p_now: f32,
    pub capable_torque: f32,
    pub leak_percentage_80: f32,
    pub leak_percentage_neg_40: f32,

    pub filter_factor_80: u16,
    pub filter_factor_neg_40: u16,

    // Width of clutch pack chamber
    pub chamber_depth: u16,
    /// Width of the clutch pack itself,
    /// CANNOT be larger than chamber width
    pub pack_depth: u16,

    pub current_depth: f32,

    pub centrifugal_extra_p: f32,

    pub applied: bool,

    pub clamp_p: f32,

    spring_p: u16,
    friction_value: u16,
    centrifugal_val: Option<u16>,
    id: egs_logic::Clutch,
    odd: bool
}



impl SimClutch {
    pub const fn new(
        id: egs_logic::Clutch,
        leak_80: f32,
        leak_neg_40: f32,
        ff_80: u16,
        ff_neg_40: u16,
        depth: u16,
        thickness: u16,
        spring_p: u16,
        centi: Option<u16>
    ) -> Self {
        Self {
            p_now: 0.0,
            capable_torque: 0.0,
            leak_percentage_80: leak_80,
            leak_percentage_neg_40: leak_neg_40,
            filter_factor_80: ff_80,
            filter_factor_neg_40: ff_neg_40,
            chamber_depth: depth,
            pack_depth: thickness,
            current_depth: depth as f32,
            centrifugal_extra_p: 0.0,
            applied: false,
            spring_p,
            friction_value: 0,
            centrifugal_val: centi,
            clamp_p: 0.0,
            id,
            odd: false
        }
    }

    pub fn set_friction_val(&mut self, ff: u16) {
        self.friction_value = ff;
    }

    pub fn update(&mut self, p_in: f32, rot_spd: u16, temperature_c: i16, m_cal: &MechCal) {
        self.odd = !self.odd;
        // Pressure filling the clutch
        let filter_factor_now = interp_linear(temperature_c, -40.0, 80.0, self.filter_factor_neg_40, self.filter_factor_80) as u16;
        self.p_now = first_order_filter(p_in, self.p_now, filter_factor_now);
        // Pressure escaping the clutch (Due to seals, and drains)
        let leak_percent_now = interp_linear(temperature_c, -40.0, 80.0, self.leak_percentage_neg_40, self.leak_percentage_80);

        let leak_mbar = leak_percent_now * self.p_now;

        self.p_now -= leak_mbar;

        if self.p_now < 0.0 {
            self.p_now = 0.0;
        }

        self.centrifugal_extra_p = 0.0;
        if self.p_now > 0.0 {
            if let Some(centrifugal_val) = self.centrifugal_val {
                // Centrifugal extra pressure pushing on the clutch
                // Can only work if there is oil in the drum
                let delta_t = temperature_c + 50;
                let drop = (m_cal.atf_density_drop_per_c as f32 * delta_t as f32) / 100.0;
                let density = m_cal.atf_density_minus_50c as f32 - drop;
                let r = (((rot_spd as u32 * rot_spd as u32) as f32 / 1000.0) * density) / centrifugal_val as f32;
                self.centrifugal_extra_p = r / 10.0;
            }
        }

        let full_applied_depth = (self.chamber_depth - self.pack_depth) as f32;
        let travel_ratio =
            (self.chamber_depth as f32 - self.current_depth) / (self.chamber_depth as f32 - full_applied_depth)
                .clamp(0.0, 1.0);
        let spring_pressure = self.spring_p as f32 + (1.0*travel_ratio);
        let piston_p = self.p_now + self.centrifugal_extra_p;
        let excess_p = piston_p - spring_pressure;

        if excess_p < 0.0 {
            self.current_depth = first_order_filter(
                self.chamber_depth as f32,
                self.current_depth,
                2.0,
            );

            self.clamp_p = first_order_filter(
                0.0,
                self.clamp_p,
                3.0,
            );

            self.applied = false;
            self.capable_torque = 0.0;
            return;
        }
        if self.current_depth > full_applied_depth {
            let pressure_ratio =
                (excess_p / self.p_now)
                    .clamp(0.0, 1.0);

            let apply_filter = interp_linear(
                pressure_ratio,
                0.0,
                1.0,
                2.0,
                1.0,
            );

            self.current_depth = first_order_filter(
                full_applied_depth,
                self.current_depth,
                apply_filter,
            );

            self.applied = false;
            self.capable_torque = 0.0;
            return;
        }

        self.current_depth = full_applied_depth;

        self.clamp_p = first_order_filter(
            excess_p,
            self.clamp_p,
            1.0,
        );

        self.applied = self.clamp_p > 0.0;

        if self.applied && self.friction_value != 0 {
            self.capable_torque =
                (self.clamp_p * 100.0)
                    / self.friction_value as f32;
        } else {
            self.capable_torque = 0.0;
        }

        /*
        // The actual pressure pushing the clutch pack in place now
        let mut p_on_pack = self.p_now - self.spring_p as f32 + self.centrifugal_extra_p;
        self.applied = if p_on_pack < 0.0 {
            p_on_pack = 0.0;
            // Clutch pack falling back into the chamber
            self.current_depth = first_order_filter(self.chamber_depth as f32, self.current_depth, 5.0);
            if self.current_depth > self.chamber_depth as f32 {
                self.current_depth = self.chamber_depth as f32;
            }
            false
        } else {
            // Pressure is enough to theoretically resist the release spring, so the clutch pack
            // will start to move in the applying direction
            let full_applied_depth = (self.chamber_depth - self.pack_depth) as f32;
            // Rough speed increase based on P vs Spring P
            let filter = interp_linear(p_on_pack, 0.0, self.spring_p as f32, 2.0, 0.0);
            self.current_depth = first_order_filter(full_applied_depth, self.current_depth, filter);
            if self.current_depth as i16 <= full_applied_depth as i16 {
                //self.current_depth = full_applied_depth;
                true
            } else {
                false
            }
        };
        if self.applied && self.friction_value != 0 {
            // Clutch pack sealed, it is now actively holding torque
            self.capable_torque = (p_on_pack * 100.0) / self.friction_value as f32;
        } else {
            self.capable_torque = 0.0;
        }
         */
    }
}


pub struct OverlapValve {
    pub spring_p: f32,
}