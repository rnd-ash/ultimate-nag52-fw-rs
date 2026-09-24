use eframe::NativeOptions;

use crate::sim::EgsSim;

pub mod gearbox;
pub mod sim;
mod pressure_emu;
mod planetary;

fn main() {
    env_logger::init();
    defmt2log::init_from_current_exe();

    let na = NativeOptions::default();

    eframe::run_native(
        "EGS Simulator",
        na,
        Box::new(|_cc| Ok(Box::new(EgsSim::new()))),
    )
    .unwrap()
}
