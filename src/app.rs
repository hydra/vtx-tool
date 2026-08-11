//! GUI shell. Read-only for now -- live power meter readings, an
//! on-demand PA calibration table pull, and two scrollable per-port
//! status/error log panels (VTX / power meter), backed by logging.rs's
//! ring buffers (>=100 messages each, oldest dropped first).
//!
//! NOT YET IMPLEMENTED: any control that WRITES to the VTX (MSP_SET_PACALIBRATION
//! sweep controls, EEPROM commit). Those are unblocked now that
//! vtx_msp.c confirmed the wire semantics -- adding them should be a
//! small extension of this App struct and worker.rs, not a rewrite.

use crate::logging::{LogEntry, PortLog, SharedLogs};
use crate::worker::{Command, SharedState};
use eframe::egui;
use eframe::Frame;
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex, MutexGuard};

pub struct App {
    state: Arc<Mutex<SharedState>>,
    cmd_tx: Sender<Command>,
    logs: &'static SharedLogs,
}

impl App {
    pub fn new(state: Arc<Mutex<SharedState>>, cmd_tx: Sender<Command>, logs: &'static SharedLogs) -> Self {
        Self { state, cmd_tx, logs }
    }
}

fn level_color(ui: &egui::Ui, level: log::Level) -> egui::Color32 {
    match level {
        log::Level::Error => egui::Color32::from_rgb(220, 80, 80),
        log::Level::Warn => egui::Color32::from_rgb(220, 180, 60),
        log::Level::Debug | log::Level::Trace => egui::Color32::GRAY,
        log::Level::Info => ui.visuals().text_color(),
    }
}

fn show_log_panel(ui: &mut egui::Ui, title: &str, port_log: &MutexGuard<PortLog>) {
    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.vertical(|ui|{
            ui.horizontal(|ui| {
                ui.strong(title);
                ui.label(format!("{} message(s)", port_log.messages.len()));
            });
            egui::ScrollArea::vertical()
                .id_salt(title)
                .auto_shrink([false, false])
                .max_height(200.0)
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    egui::Grid::new(title)
                        .num_columns(1)
                        .striped(true)
                        .show(ui, |ui| {
                        for entry in port_log.messages.iter() {
                            show_log_entry(ui, entry);
                            ui.end_row();
                        }
                    });
                });
        });
    });
}

fn show_log_entry(ui: &mut egui::Ui, entry: &LogEntry) {
    let color = level_color(ui, entry.level);
    ui.colored_label(color, format!("[{:>5}] {}", entry.level, entry.text));
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut Frame) {
        let state = self.state.lock().unwrap();

        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.heading("RF Calibration");

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

            ui.separator();
            ui.heading("Connection status");
            show_log_panel(ui, "VTX port", &self.logs.vtx.lock().unwrap());
            show_log_panel(ui, "Power meter port", &self.logs.meter.lock().unwrap());
        });
    }
}
