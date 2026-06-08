pub enum CrossoverAlgoStep {
    Fill,
    Overlap,
    Boost
}

pub enum ReleaseAlgoStep {
    FillAndRelease,
    Overlap
}

pub enum ShiftingAlgoInner {
    Crossover(CrossoverAlgoStep),
    Release(ReleaseAlgoStep)
}

pub enum ShiftAlgoStep {
    Bleed,
    AlgoInner(ShiftingAlgoInner),
    MaxPressure,
    EndControl
}