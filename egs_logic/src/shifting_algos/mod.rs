use egs_maths::egs_timers::EgsCountDownTimer;

pub enum CrossoverAlgoStep {
    Fill,
    Overlap,
    Boost,
}

pub enum ReleaseAlgoStep {
    FillAndRelease,
    Overlap,
}

pub enum ShiftingAlgoInner {
    Crossover(CrossoverAlgoStep),
    Release(ReleaseAlgoStep),
}

pub enum ShiftAlgoStep {
    Bleed,
    AlgoInner(ShiftingAlgoInner),
    MaxPressure,
    EndControl,
}


pub enum ShiftType {
    CrossoverUp,
    ReleaseUp,
    CrossoverDn,
    ReleaseDn,
    ToN,
    ToRorD,
}


pub struct AlgorithmData {
    pub timer_shift: EgsCountDownTimer,
    pub timer_mod: EgsCountDownTimer,
    pub timer_emergency: EgsCountDownTimer,

    pub momentum: i32,
    pub momentum_filtered: i32,

    pub overlap_shift_p: u32,
    pub max_p_apply_clutch: u32,
    pub p_apply_clutch: u32,
    pub p_shift_sol: u32,
    pub p_mod_sol: u32
}