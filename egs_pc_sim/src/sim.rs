use egs_logic::{Clutch, GearboxInputs, GearboxOutputs, GearboxVars, TcuTickCounter, TftReading};
use egui::{Panel, Slider, UiKind::CentralPanel};
use egui_plot::{PlotPoint, PlotPoints};
use std::collections::VecDeque;
use std::time::Instant;
use std::{sync::Arc, sync::RwLock, time::Duration};
use tokio::runtime::Runtime;

use crate::gearbox::{MECH_CAL, SimGearbox};
use crate::pressure_emu::{SIM_CLUTCH_K1, SIM_CLUTCH_K2, SIM_CLUTCH_K3};

const SNAPSHOTS: usize = 50 * 60; // 50 updates/sec x 30 seconds

pub struct PhysicalCharacteristics {}

pub struct EgsSim {
    rt: tokio::runtime::Runtime,
    inputs: Arc<RwLock<GearboxInputs>>,
    pll: bool,
    atf_c: i16,
    e_trq: i16,
    e_rpm: u16,
    snapshots: Arc<RwLock<VecDeque<(GearboxInputs, GearboxVars, GearboxOutputs)>>>,
    applied_percentages: [(f32, f32, f32); ITERATIONS * 2],
    clearance: [(f32, f32, f32); ITERATIONS * 2],
    pressures: [(f32, f32, f32); ITERATIONS * 2],
}

const ITERATIONS: usize = 50 * 2; // 10 seconds

impl EgsSim {
    pub fn new() -> Self {
        let mut s = Self {
            inputs: Arc::new(RwLock::new(GearboxInputs::new())),
            rt: Runtime::new().unwrap(),
            pll: false,
            atf_c: 40,
            e_trq: 0,
            e_rpm: 0,
            snapshots: Default::default(),
            applied_percentages: [(0.0, 0.0, 0.0); ITERATIONS * 2],
            clearance: [(0.0, 0.0, 0.0); ITERATIONS * 2],
            pressures: [(0.0, 0.0, 0.0); ITERATIONS * 2],
        };

        let inp = s.inputs.clone();
        let snapshots = s.snapshots.clone();

        let mut s_k1 = SIM_CLUTCH_K1;
        let mut s_k2 = SIM_CLUTCH_K2;
        let mut s_k3 = SIM_CLUTCH_K3;
        // 4th gear
        s_k1.set_friction_val(1144);
        s_k2.set_friction_val(2007);
        s_k3.set_friction_val(1214);

        let mut p_in = 0.0;
        let mut s_in = 2500;

        for i in 0..ITERATIONS {
            p_in += 40.0;
            if p_in > 2000.0 {
                p_in = 2000.0;
            }
            s_k1.update(p_in, s_in, 80, &MECH_CAL);
            s_k2.update(p_in, s_in, 80, &MECH_CAL);
            s_k3.update(p_in * 2.0, s_in, 80, &MECH_CAL);
            s_in += 1;
            s.applied_percentages[i].0 = s_k1.capable_torque;
            s.applied_percentages[i].1 = s_k2.capable_torque;
            s.applied_percentages[i].2 = s_k3.capable_torque;

            s.clearance[i].0 = s_k1.current_depth;
            s.clearance[i].1 = s_k2.current_depth;
            s.clearance[i].2 = s_k3.current_depth;

            s.pressures[i].0 = s_k1.p_now;
            s.pressures[i].1 = s_k2.p_now;
            s.pressures[i].2 = s_k3.p_now;
        }

        for i in 0..ITERATIONS {
            p_in -= 100.0;
            if (p_in < 0.0) {
                p_in = 0.0;
            }
            s_k1.update(p_in, s_in, 80, &MECH_CAL);
            s_k2.update(p_in, s_in, 80, &MECH_CAL);
            s_k3.update(p_in, s_in, 80, &MECH_CAL);
            s_in += 1;
            s.applied_percentages[ITERATIONS + i].0 = s_k1.capable_torque;
            s.applied_percentages[ITERATIONS + i].1 = s_k2.capable_torque;
            s.applied_percentages[ITERATIONS + i].2 = s_k3.capable_torque;

            s.clearance[ITERATIONS + i].0 = s_k1.current_depth;
            s.clearance[ITERATIONS + i].1 = s_k2.current_depth;
            s.clearance[ITERATIONS + i].2 = s_k3.current_depth;

            s.pressures[ITERATIONS + i].0 = s_k1.p_now;
            s.pressures[ITERATIONS + i].1 = s_k2.p_now;
            s.pressures[ITERATIONS + i].2 = s_k3.p_now;
        }

        s.rt.spawn(async move { gearbox_loop(inp, snapshots).await });

        s
    }
}

impl eframe::App for EgsSim {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        //elegance::Theme::charcoal().install(ui.ctx());

        let mut inputs = self.inputs.read().unwrap().clone();
        let history = self.snapshots.read().unwrap();
        if let Some((_, vars, outputs)) = history.back() {
            ui.heading("Input states");
            // PLL stuff
            ui.checkbox(&mut self.pll, "Parking lock on?");
            ui.add(Slider::new(&mut self.atf_c, -40..=200).text("ATF Temperature C"));
            ui.add(Slider::new(&mut self.e_rpm, 0..=8000).text("Engine RPM"));
            ui.add(Slider::new(&mut self.e_trq, -100..=500).text("Engine Torque Nm"));
            if self.pll {
                inputs.atf = Some(TftReading::ParkOrNeutral)
            } else {
                inputs.atf = Some(TftReading::Temperature(self.atf_c))
            }
            inputs.can_vars.driver_demand_torque_nm = Ok(self.e_trq as f32);
            inputs.can_vars.engine_indicated_torque_nm = Ok(self.e_trq as f32);
            inputs.can_vars.engine_static_torque_nm = Ok(self.e_trq as f32);
            inputs.can_vars.engine_rpm = Ok(self.e_rpm);

            //ui.heading("Output states");
            //ui.code(&mut format!("{:#?}", outputs));

            let k1_p_points: Vec<f32> = self.pressures.iter().map(|x| x.0).collect();
            let k2_p_points: Vec<f32> = self.pressures.iter().map(|x| x.1).collect();
            let k3_p_points: Vec<f32> = self.pressures.iter().map(|x| x.2).collect();

            let k1_c_points: Vec<f32> = self.clearance.iter().map(|x| x.0).collect();
            let k2_c_points: Vec<f32> = self.clearance.iter().map(|x| x.1).collect();
            let k3_c_points: Vec<f32> = self.clearance.iter().map(|x| x.2).collect();

            let k1_t_points: Vec<f32> = self.applied_percentages.iter().map(|x| x.0).collect();
            let k2_t_points: Vec<f32> = self.applied_percentages.iter().map(|x| x.1).collect();
            let k3_t_points: Vec<f32> = self.applied_percentages.iter().map(|x| x.2).collect();

            let k1_p_line = egui_plot::Line::new("K1-P", PlotPoints::from_ys_f32(&k1_p_points));
            let k2_p_line = egui_plot::Line::new("K2-P", PlotPoints::from_ys_f32(&k2_p_points));
            let k3_p_line = egui_plot::Line::new("K3-P", PlotPoints::from_ys_f32(&k3_p_points));

            let k1_c_line = egui_plot::Line::new("K1-C", PlotPoints::from_ys_f32(&k1_c_points));
            let k2_c_line = egui_plot::Line::new("K2-C", PlotPoints::from_ys_f32(&k2_c_points));
            let k3_c_line = egui_plot::Line::new("K3-C", PlotPoints::from_ys_f32(&k3_c_points));

            let k1_t_line = egui_plot::Line::new("K1-T", PlotPoints::from_ys_f32(&k1_t_points));
            let k2_t_line = egui_plot::Line::new("K2-T", PlotPoints::from_ys_f32(&k2_t_points));
            let k3_t_line = egui_plot::Line::new("K3-T", PlotPoints::from_ys_f32(&k3_t_points));

            ui.heading("Clutch pressure");
            egui_plot::Plot::new("test").height(300.0).show(ui, |ui| {
                ui.line(k1_p_line);
                ui.line(k2_p_line);
                ui.line(k3_p_line);
            });
            ui.heading("Clutch clearance (0 = applied)");
            egui_plot::Plot::new("test-c").height(300.0).show(ui, |ui| {
                ui.line(k1_c_line);
                ui.line(k2_c_line);
                ui.line(k3_c_line);
            });
            ui.heading("Clutch torque capacity");
            egui_plot::Plot::new("test-t").height(300.0).show(ui, |ui| {
                ui.line(k1_t_line);
                ui.line(k2_t_line);
                ui.line(k3_t_line);
            });

            // Write inputs
            *self.inputs.write().unwrap() = inputs;
        }
    }
}

pub struct EgsSimTickCounter(Instant);

impl TcuTickCounter for EgsSimTickCounter {
    fn start(&mut self) {
        self.0 = Instant::now()
    }

    fn ticks(&mut self) -> u16 {
        let nanos = self.0.elapsed().as_millis().min(u16::MAX as _) as u16;
        self.start();
        nanos
    }
}

async fn gearbox_loop(
    inputs: Arc<RwLock<GearboxInputs>>,
    snapshots: Arc<RwLock<VecDeque<(GearboxInputs, GearboxVars, GearboxOutputs)>>>,
) {
    let mut inputs_local = GearboxInputs::new();
    let mut vars = GearboxVars::new();
    let mut outputs = GearboxOutputs::new();
    let mut gb = SimGearbox::new(&mut inputs_local, &mut vars, &mut outputs);
    let mut counter = EgsSimTickCounter(Instant::now());
    counter.start();
    loop {
        {
            let inputs = inputs.read().unwrap().clone();
            *gb.inputs_mut() = inputs;
            gb.update(&mut counter);
            let mut snapshot_queue = snapshots.write().unwrap();
            snapshot_queue.push_back((inputs, *gb.internal_vars(), *gb.outputs()));
            if snapshot_queue.len() > SNAPSHOTS {
                snapshot_queue.pop_front();
            }
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}
