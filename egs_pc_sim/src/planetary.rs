use egs_logic::calbrations::mech::MechCal;
use crate::pressure_emu::SimClutch;

pub enum CarrierDest {
    OutputShaft,

}

pub struct TeethCount {
    pub sun: u16,
    pub carrier: u16,
    pub ring: u16
}

const TEETH_P1: TeethCount = TeethCount {
    sun: 50,
    carrier: 14,
    ring: 78,
};

const TEETH_P2: TeethCount = TeethCount {
    sun: 30,
    carrier: 22,
    ring: 20,
};

const TEETH_P3: TeethCount = TeethCount {
    sun: 50,
    carrier: 20,
    ring: 90,
};

pub struct GearSpeeds {
    pub sun: u16,
    pub carrier: u16,
    pub ring: u16
}

/// Input shaft to mid-shaft
pub struct PlanetarySet1 {
    pub speeds: GearSpeeds,
    pub k1: SimClutch,
    pub b1: SimClutch,

    pub s_k1: u16,
    pub s_b1: u16,
}


impl PlanetarySet1 {
    pub fn update(
        &mut self,
        p_k1: f32,
        p_b1: f32,
        temp: i16,
        s_input: &mut u16,
        t_input: f32,
        m_cal: &MechCal
    ) {
        self.k1.update(p_k1, self.s_k1, temp, m_cal);
        self.b1.update(p_b1, 0, temp, m_cal);
        let c_spd_s_full_k1 = *s_input as f32;
        let c_spd_s_full_b1 = *s_input as f32 * (TEETH_P1.sun as f32 / TEETH_P1.ring as f32);

        //self.speeds.ring = s_input;

    }
}