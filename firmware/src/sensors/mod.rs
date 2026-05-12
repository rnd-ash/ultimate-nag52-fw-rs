use atsamd_hal::{
    adc::{
        Accumulation, Adc0, Adc1, AdcBuilder, AdcResolution, FutureAdc,
        Prescaler, Reference,
    },
    clock::{
        self,
        v2::{
            apb::ApbClk,
            gclk::GclkId,
            pclk::{Pclk},
        },
    },
    pac::{self, Supc},
};

use futures::join;

use crate::sensors::adc::{Adc0Pins, Adc1Pins, Adc1VariableInputs};
use crate::{Adc0Irqs, Adc1Irqs};

use maths;

pub mod adc;
pub mod speed_sensors;
pub mod variable_adc_input;

#[derive(Default, Copy, Clone)]
pub enum TftState {
    /// Parking lock engaged (No temperature)
    #[default]
    Pll,
    // Temperature in Celcius
    Temperature(i8),
}

#[derive(Default, Copy, Clone)]
pub struct SensorData {
    /// TFT Sensor state
    pub tft: TftState,
    /// KL15 voltage (mV)
    pub vkl15: u16,
    /// Sensors voltage (mV)
    pub vsense: u16,
    /// KL87 voltage (mV)
    pub vkl87: u16,
    /// KL87 current (mA)
    pub ikl87: u16,
    /// Temperature of PCB near external connector (C)
    pub t_pcb: i8,
    /// Temperature of PCB near TLE8242 IC (C)
    pub t_tle: i8,
}

/// Assume ADC reading is 0-4095 (=0-3.3V)
/// R1 and R2 are in Ohms
pub fn adc_reading_to_source(adc: u16, r1: u32, r2: u32) -> u16 {
    let div = (r2 * 1000) / (r1 + r2);
    let v_out = 3300 * (((adc as u32) * 1000) / 4095);
    (v_out / div) as u16
}

pub struct AdcData {
    adc0: FutureAdc<Adc0, Adc0Irqs>,
    adc1: FutureAdc<Adc1, Adc1Irqs>,
    /// Required for reading the board temperature
    supc: Supc,
    adc0_pins: Adc0Pins,
    adc1_pins: Adc1Pins,
    adc1_variable_inputs: Adc1VariableInputs,
}

impl AdcData {
    pub fn new<P: GclkId>(
        adc0: pac::Adc0,
        adc1: pac::Adc1,
        supc: Supc,
        adc0_pins: Adc0Pins,
        adc1_pins: Adc1Pins,
        adc1_variable_inputs: Adc1VariableInputs,
        apb_adc0: ApbClk<clock::v2::pclk::ids::Adc0>,
        apb_adc1: ApbClk<clock::v2::pclk::ids::Adc1>,
        pclk_adc0: Pclk<clock::v2::pclk::ids::Adc0, P>,
        pclk_adc1: Pclk<clock::v2::pclk::ids::Adc1, P>,
    ) -> Self {
        // ADC0 has access to the TFT Sensor, therefore, we cannot do any sample averaging at a hardware level!
        // This is to prevent a situation where the parking lock comes on mid-reading, which in tern causes the voltage
        // to jump to 3.3V, which (When averaged with valid Temperature readings), would cause a spike in voltage that
        // isn't quite high enough to be considered Parking lock on state
        let adc_adc0 = AdcBuilder::new(Accumulation::Single(AdcResolution::_12))
            .with_vref(Reference::Intvcc1)
            .with_clock_divider(Prescaler::Div8)
            .with_clock_cycles_per_sample(32)
            .enable(adc0, apb_adc0, &pclk_adc0)
            .unwrap()
            .into_future(Adc0Irqs);
        let adc_adc1 = AdcBuilder::new(Accumulation::Single(AdcResolution::_12))
            .with_vref(Reference::Intvcc1)
            .with_clock_divider(Prescaler::Div8)
            .with_clock_cycles_per_sample(32)
            .enable(adc1, apb_adc1, &pclk_adc1)
            .unwrap()
            .into_future(Adc1Irqs);
        Self {
            adc0: adc_adc0,
            adc1: adc_adc1,
            supc,
            adc0_pins,
            adc1_pins,
            adc1_variable_inputs,
        }
    }

    pub async fn update(&mut self) -> SensorData {
        // Poll ADC0 and ADC1 together (A bit faster)
        let (adc0_res, adc1_res) = join!(
            self.adc0_pins.poll_all(&mut self.adc0, &mut self.supc),
            self.adc1_pins.poll_all(&mut self.adc1)
        );
        let _var_res = self.adc1_variable_inputs.poll_all(&mut self.adc1).await;

        // Process the results
        //println!("{:?}", var_res);
        // PCB Temperature sensors
        let temp_tle8242 = maths::interp(adc0_res.tsen_tle82423 as i32, &TSEN_LOOKUP);
        let temp_pcb = maths::interp(adc0_res.tsen_pcb as i32, &TSEN_LOOKUP);

        // Parking lock / ATF Temperature
        let tft = if adc1_res.tft > 4090 {
            TftState::Pll
        } else {
            let temp = maths::interp(adc1_res.tft as i32, &TFT_LOOKUP);
            TftState::Temperature(temp as i8)
        };

        // Power monitors
        let vkl15 = adc_reading_to_source(adc1_res.pmon_kl15, 12_000, 3_300);
        let vsense = adc_reading_to_source(adc1_res.pmon_sens, 10_000, 15_000);
        let vkl87 = adc_reading_to_source(adc1_res.pmon_kl87, 12_000, 3_300);
        let ikl87 = if adc1_res.pmon_kl87_diag < 10 || vkl87 < 5000 {
            0
        } else {
            let reading_mv = (adc1_res.pmon_kl87_diag as f32 / 4095.0) * 3.3;
            let scale = maths::interp(temp_pcb, BTS_DIV_TEMP) as f32;
            (reading_mv * scale) as u16
        };

        SensorData {
            tft,
            vkl15,
            vsense,
            vkl87,
            ikl87,
            t_pcb: temp_pcb as i8,
            t_tle: temp_tle8242 as i8,
        }
    }
}

const MAX_ADC_VAL: u16 = 4095;

const fn tft_resistance_to_adc_12bit(r1: u32, r_tsen: u32, temp: i16) -> (i32, i32) {
    (
        ((MAX_ADC_VAL as u32 * r_tsen) / (r1 + r_tsen)) as i32,
        temp as i32,
    )
}

// https://www.nxp.com/docs/en/data-sheet/KTY83_SER.pdf
// KTY83/110
// ADC Value, Temperature
const PULLUP_TFT_SENSOR: u32 = 2000; // Ohms
const TFT_LOOKUP: &[(i32, i32)] = &[
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 500, -55),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 525, -50),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 525, -50),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 577, -40),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 632, -30),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 691, -20),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 754, -10),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 820, 0),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 889, 10),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 962, 20),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1000, 25),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1039, 30),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1118, 40),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1202, 50),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1288, 60),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1379, 70),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1472, 80),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1569, 90),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1670, 100),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1774, 110),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1882, 120),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1937, 125),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 1993, 130),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 2107, 140),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 2225, 150),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 2346, 160),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 2471, 170),
    tft_resistance_to_adc_12bit(PULLUP_TFT_SENSOR, 2535, 175),
];

// https://www.nxp.com/docs/en/data-sheet/KTY83_SER.pdf
// TDK NTCG 0402
// ADC Value, Temperature
const PULLUP_PCB_SENSOR: u32 = 12000; // Ohms
const TSEN_LOOKUP: &[(i32, i32)] = &[
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 534, 125),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 599, 120),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 760, 110),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 975, 100),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 1267, 90),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 1668, 80),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 2227, 70),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 3019, 60),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 4158, 50),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 5826, 40),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 6942, 35),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 8312, 30),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 10000, 25),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 12090, 20),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 14700, 15),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 17960, 10),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 22070, 5),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 27280, 0),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 33930, -5),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 42450, -10),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 67790, -20),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 111300, -30),
    tft_resistance_to_adc_12bit(PULLUP_PCB_SENSOR, 188500, -40),
];

/// BTS6143D voltage divider based on temperature (See datasheet)
const BTS_DIV_TEMP: &[(i32, i32)] = &[(-40, 10_000), (25, 9700), (150, 9300)];
