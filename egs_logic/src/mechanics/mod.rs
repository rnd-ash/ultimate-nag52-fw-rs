use egs_maths::tcu_num::TcuNum;

use crate::{Gear, ShiftCircuit, calbrations::mech::MechCal};

pub struct ClutchSpeedInfo {
    pub applying_clutch_speed: i32,
    pub releasing_clutch_speed: i32,
    pub rear_sun_speed: i32,
}

fn mul_ratio(r: u16, v: impl TcuNum) -> f32 {
    (r as f32 * v.into()) / 1000.0
}

pub enum ClutchSpeedCircuit {
    None(Gear),
    Shift(ShiftCircuit),
    ToN,
    ToR,
    ToD,
}

impl ClutchSpeedInfo {
    pub fn new(speeds: &GearboxSpeeds, state: ClutchSpeedCircuit, m_cal: &MechCal) -> Self {
        match state {
            ClutchSpeedCircuit::None(gear) => Self {
                applying_clutch_speed: 0,
                releasing_clutch_speed: 0,
                rear_sun_speed: 0, //Self::rear_sun_speed(speeds, gear, m_cal),
            },
            ClutchSpeedCircuit::Shift(shift) => match shift {
                ShiftCircuit::_12 | ShiftCircuit::_21 => {
                    let k1 = (speeds.n2 as i32) - (speeds.n3 as i32);
                    let b1 = speeds.n3 as i32;

                    let (applying_clutch_speed, releasing_clutch_speed) =
                        if shift.is_up() { (k1, b1) } else { (b1, k1) };

                    Self {
                        applying_clutch_speed,
                        releasing_clutch_speed,
                        rear_sun_speed: 0,
                    }
                }
                ShiftCircuit::_23 | ShiftCircuit::_32 => {
                    let k2 = (speeds.n3 as i32)
                        - mul_ratio(m_cal.ratio_table[Gear::_3], speeds.calc_output as u16) as i32;

                    let k3_denom =
                        (m_cal.ratio_table[Gear::_2] - m_cal.ratio_table[Gear::_3]) as f32 / 1000.0;

                    let k3 = mul_ratio(m_cal.ratio_table[Gear::_2], speeds.calc_output as u16)
                        as i32
                        - speeds.n3 as i32;

                    let (applying_clutch_speed, releasing_clutch_speed) =
                        if shift.is_up() { (k2, k3) } else { (k3, k2) };

                    Self {
                        applying_clutch_speed,
                        releasing_clutch_speed,
                        rear_sun_speed: 0,
                    }
                }
                ShiftCircuit::_34 | ShiftCircuit::_43 => {
                    todo!()
                }
                ShiftCircuit::_45 | ShiftCircuit::_54 => {
                    todo!()
                }
            },
            ClutchSpeedCircuit::ToN => todo!(),
            ClutchSpeedCircuit::ToR => todo!(),
            ClutchSpeedCircuit::ToD => todo!(),
        }
    }

    /// Return just the rear sun gear speed, used when in motion in a static gear
    fn rear_sun_speed(speeds: &GearboxSpeeds, gear: Gear, m_cal: &MechCal) -> u32 {
        if matches!(gear, Gear::_4 | Gear::_5 | Gear::_R1 | Gear::_R2) {
            Self::sun_ratio_calc(Gear::_2, speeds.calc_turbine, speeds.calc_output, m_cal)
        } else {
            0
        }
    }

    // Common block of code for calculating rear sun gear ratio (For Centrifugal calc)
    fn sun_ratio_calc(gear: Gear, turbine: u32, output: u32, m_cal: &MechCal) -> u32 {
        let ratio_g = m_cal.ratio_u32(gear);
        let ratio_delta = ratio_g - m_cal.ratio_u32(Gear::_4);
        let mul = (turbine * 1000) / ratio_delta;
        (ratio_g * output) / (ratio_delta) - mul
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GearboxSpeeds {
    pub n2: u32,
    pub n3: u32,
    pub calc_rear_sun: u32,
    pub calc_turbine: u32,
    pub calc_output: u32,
}

impl GearboxSpeeds {
    pub const fn new() -> Self {
        Self {
            n2: 0,
            n3: 0,
            calc_rear_sun: 0,
            calc_turbine: 0,
            calc_output: 0,
        }
    }
}
