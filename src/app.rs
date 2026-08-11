//! GUI shell. Currently read-only -- shows live power meter readings and
//! lets you pull the VTX's current PA calibration table on demand.
//!
//! NOT YET IMPLEMENTED: any control that WRITES to the VTX (MSP_SET_PACALIBRATION
//! sweep controls, EEPROM commit button). Those depend on confirming the
//! VTX-side C handler's exact semantics first -- see the note in msp.rs.
//! Adding them once that's confirmed should be a small extension of this
//! same App struct, not a rewrite.

use crate::worker::{Command, SharedState};
use eframe::{egui, Frame};
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};

pub struct App {
    state: Arc<Mutex<SharedState>>,
    cmd_tx: Sender<Command>,
}

impl App {
    pub fn new(state: Arc<Mutex<SharedState>>, cmd_tx: Sender<Command>) -> Self {
        Self { state, cmd_tx }
    }
}

impl eframe::App for App {

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut Frame) {
        let state = self.state.lock().unwrap();

        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.heading("RF Calibration");

            if let Some(err) = &state.last_error {
                ui.colored_label(egui::Color32::RED, err);
            }

            ui.separator();
            ui.label(match state.last_dbm {
                Some(dbm) => format!(
                    "Power meter: {:.2} dBm  ({:.3} mW)",
                    dbm,
                    10f32.powf(dbm / 10.0)
                ),
                None => "Power meter: (no reading yet)".to_string(),
            });

            ui.separator();
            ui.heading("PA calibration table");
            if ui.button("Refresh from VTX").clicked() {
                let _ = self.cmd_tx.send(Command::RefreshTable);
            }

            egui::Grid::new("pa_table").striped(true).show(ui, |ui| {
                ui.strong("idx");
                ui.strong("mW");
                ui.strong("calibration[] (mV)");
                ui.strong("detector[] (mV)");
                ui.end_row();
                for entry in &state.pa_table {
                    ui.label(entry.idx.to_string());
                    ui.label(entry.m_w.to_string());
                    ui.label(format!("{:?}", entry.value));
                    ui.label(format!("{:?}", entry.detector));
                    ui.end_row();
                }
            });
        });
    }
}
