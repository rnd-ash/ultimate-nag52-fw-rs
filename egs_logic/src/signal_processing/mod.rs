use defmt::println;
use egs_can::CanError;

use crate::{
    Gearbox, TcuTickCounter, TftReading,
    errors::DeviceMode,
    storage::{GearboxAdaptStorage, GearboxCalibStorage, GearboxDtcStorage, GearboxMapStorage},
};

impl<
    'a,
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage,
> Gearbox<'a, DS, CS, MS, AS>
{
    pub fn process_clock_time(&mut self) {
        self.us_delta_this_cycle =
            self.inputs.clock_time_us.saturating_sub(self.us_clock_time) as _;
        self.us_clock_time = self.inputs.clock_time_us;
    }

    pub fn process_engine_rpm(&mut self) {
        match self.inputs.can_vars.engine_rpm {
            Ok(rpm) => {
                self.engine_rpm = rpm;
                if !self.engine_running && self.engine_rpm > 500 {
                    self.engine_running = true;
                }
            }
            Err(_e) => {
                self.engine_rpm = 0;
                self.engine_running = false;
            }
        }
    }

    pub fn process_tft_sensor(&mut self) {
        let motor_temperature = self
            .inputs
            .can_vars
            .coolant_temperature_c
            .unwrap_or_else(|_| 80);
        match self.inputs.atf {
            Some(TftReading::ParkOrNeutral) => {
                self.outputs.can_outputs.can_start = true;
                self.vars.temperature_atf =
                    core::cmp::max(0, self.inputs.can_vars.coolant_temperature_c.unwrap_or(80))
                        as _;
            }
            Some(TftReading::Temperature(grad_c)) => {
                self.outputs.can_outputs.can_start = false;
                self.vars.temperature_atf = core::cmp::max(0, grad_c);
            }
            None => {
                self.outputs.can_outputs.can_start = false;
                self.vars.temperature_atf = 20; // 20C base value
            }
        }
    }

    pub fn write_can_outputs(&mut self) {
        // State feedback (Health of the controller)

        self.outputs.can_outputs.gearbox_ok = !self.dev_mode().has_error();
        self.outputs.solenoid_pwr_en = !self.dev_mode().has_error();
        self.outputs.can_outputs.req_mil_light =
            self.dev_mode().contains(DeviceMode::PERM_EMERGENCY);

        // Gearbox states
        self.outputs.can_outputs.large_nag = self.cal_storage.mech_cal().gb_ty == 0; // Small NAG is 1, Large is 0
        self.outputs.can_outputs.input_rpm = match self.calc_input_rpm {
            Some(v) => v,
            None => u16::MAX,
        };

        self.outputs.can_outputs.manual_shifting = self.profile.manual_shifting();
        self.outputs.can_outputs.gearbox_temperature_c = self.vars.temperature_atf;
    }

    pub fn process_speed_sensors<T: TcuTickCounter>(&mut self, tim: &mut T) {
        let n2_p_raw = self.inputs.n2_pulses_raw;
        let n3_p_raw = self.inputs.n3_pulses_raw;
        //println!(
        //    "Loop time {}us - N2: {} N3: {}",
        //    &self.us_delta_this_cycle, n2_p_raw, n3_p_raw
        //);
    }
}
