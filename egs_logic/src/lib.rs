#![no_std]

use core::{
    ops::{Deref, DerefMut},
    u16,
};
use defmt::{info, println};
pub use egs_can;
pub mod dtcs;

use crate::{
    errors::DeviceMode,
    storage::{
        GearboxAdaptStorage, GearboxCalibStorage, GearboxDtcStorage, GearboxMapStorage,
        StorageBacking, StoragePoll,
    },
    types::vars::{GearboxInputs, GearboxOutputs, GearboxVars},
};

pub mod calbrations;
pub mod config;
pub mod errors;
pub mod hydraulics;
pub mod mechanics;
pub mod shifting_algos;
pub mod signal_processing;
pub mod status;
pub mod storage;
pub mod timers;
mod types;
pub use types::*;
pub mod diagnostic;

/// High precision half microsecond counter for profiling,
/// and event tracking
pub trait TcuTickCounter {
    fn start(&mut self);
    // Maximum 500ns ticks u32::MAX (~32ms)
    fn half_micros(&mut self) -> u32;
    fn value_now_raw(&self) -> u32;
    fn half_micros_since_raw(&self, v: u32) -> u32;
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct CoefficientStorage {}

#[repr(C)]
pub struct Gearbox<
    'a,
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage,
> {
    dtc_storage: DS,
    cal_storage: CS,
    map_storage: MS,
    adp_storage: AS,
    vars: &'a mut GearboxVars,
    inputs: &'a mut GearboxInputs,
    outputs: &'a mut GearboxOutputs,
}

impl<
    'a,
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage,
> Deref for Gearbox<'a, DS, CS, MS, AS>
{
    fn deref(&self) -> &Self::Target {
        self.vars
    }

    type Target = GearboxVars;
}

impl<
    'a,
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage,
> DerefMut for Gearbox<'a, DS, CS, MS, AS>
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.vars
    }
}

impl<
    'a,
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage,
> Gearbox<'a, DS, CS, MS, AS>
{
    pub fn new(
        inputs: &'a mut GearboxInputs,
        vars: &'a mut GearboxVars,
        outputs: &'a mut GearboxOutputs,
        dtc_storage: DS,
        cal_storage: CS,
        map_storage: MS,
        adp_storage: AS,
    ) -> Self {
        Self {
            dtc_storage,
            cal_storage,
            map_storage,
            adp_storage,
            vars,
            inputs,
            outputs,
        }
    }

    pub fn inputs_mut(&mut self) -> &mut GearboxInputs {
        self.inputs
    }

    pub fn outputs(&self) -> &GearboxOutputs {
        self.outputs
    }

    pub fn internal_vars(&self) -> &GearboxVars {
        &self.vars
    }

    pub fn set_init_vars(&mut self) {
        if let Some(shifter_pos) = &self.inputs.shifter_position {
            match shifter_pos {
                SelectorPos::DN
                | SelectorPos::D
                | SelectorPos::TipTronic(_)
                | SelectorPos::Trrs(_) => {
                    // Trigger 5th so that the TCU has to re-establish the right ratio it
                    // is actually in
                    self.vars.last_fwd_gear = Gear::_5;
                    self.vars.current_gear = Gear::_5;
                    self.vars.target_gear = Gear::_5;
                }
                SelectorPos::R | SelectorPos::RN | SelectorPos::RP => {
                    // Assume R2 (R1 is easy to resolve)
                    self.vars.last_fwd_gear = Gear::_2;
                    self.vars.current_gear = Gear::_R2;
                    self.vars.target_gear = Gear::_R2;
                }
                SelectorPos::P | SelectorPos::N => {
                    self.vars.last_fwd_gear = Gear::_2;
                    self.vars.current_gear = Gear::N;
                    self.vars.target_gear = Gear::N;
                }
            }
        }
    }

    /// This function MUST be called every 20ms to update the gearbox control loop
    pub fn update<TIM: TcuTickCounter>(&mut self, wall_timer: &mut TIM) {
        if self.vars.mode.contains(DeviceMode::INIT) {
            self.set_init_vars();
            let list: [&'_ mut dyn StorageBacking; _] = [
                &mut self.cal_storage,
                &mut self.adp_storage,
                &mut self.map_storage,
                &mut self.dtc_storage,
            ];
            let count = list.len();
            let mut ready = 0;
            for dev in list {
                match dev.init_poll() {
                    StoragePoll::Waiting => {}
                    StoragePoll::Ready => {
                        ready += 1;
                    }
                    StoragePoll::Error => {
                        self.vars.mode.remove(DeviceMode::INIT);
                        self.vars.mode.insert(DeviceMode::TEMP_EMERGENCY);
                        defmt::error!("Storage backing failed!");
                        break;
                    }
                }
            }
            if ready == count {
                self.vars.mode.remove(DeviceMode::INIT);
            }
            if !self.vars.mode.contains(DeviceMode::INIT) {
                defmt::info!("Gearbox init marked as done");
            }
        } else {
            // Looping
            if self.vars.mode.contains(DeviceMode::SLAVE) {
                // Special mode (Handled per-device)
            } else {
                // Start tracking times
                wall_timer.start();
                self.process_clock_time();
                self.vars.timers.update();
                self.process_engine_rpm();
                self.process_tft_sensor();
                self.process_speed_sensors(wall_timer);
                self.stat_times.input_processing = wall_timer.half_micros() as u16;

                // Shift functions
                self.calculate_solenoid_status();
                self.stat_times.shift_actuation_logic = wall_timer.half_micros() as u16;

                // Safety monitoring
                self.safety_fn();
                self.stat_times.safety = wall_timer.half_micros() as u16;

                // outputs
                self.write_can_outputs();
                self.stat_times.output_processing = wall_timer.half_micros() as u16;
            }
        }
    }

    // Sets via OR
    pub fn set_dev_mode(&mut self, mode: DeviceMode) {
        self.vars.mode |= mode;
    }

    pub fn dev_mode(&self) -> DeviceMode {
        self.vars.mode
    }

    fn safety_fn(&mut self) {
        if self.mode.has_error() {
            self.outputs.y3_en = false;
            self.outputs.y4_en = false;
            self.outputs.y5_en = false;
            self.outputs.tcc_pwm = 0;
            self.outputs.mpc_current = 0;
            self.outputs.spc_current = 0;
            self.outputs.solenoid_pwr_en = false;
        }
    }
}
