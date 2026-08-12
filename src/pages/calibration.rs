//! Calibration page: live power meter reading (with a rolling plot), the
//! PA calibration table (now with per-level checkboxes, boost/RTC6705
//! display columns, and a live calibration-status column), and the
//! Re-calibrate sweep controls (see calibration_engine.rs for the actual sweep
//! state machine this drives).

use crate::calibration_engine::{self, CellStatus, LevelStatus};
use crate::conn_status;
use crate::worker::{Command, SharedState, SharedSweep, HISTORY_WINDOW_SECS};
use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints};
use std::collections::HashMap;
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};

/// Candidate update rates, always shown -- entries exceeding the
/// connected meter's max_update_hz() are disabled (not hidden), so the
/// available range is visible even before a rate becomes selectable.
/// Hardcoded (hz, label) pairs rather than a formatting function: with
/// only five fixed values, direct control over exact wording ("1 second"
/// singular vs "2 seconds" plural, "ms" vs "second(s)") is simpler and
/// less error-prone than generic period-formatting logic for such a
/// small, fixed set.
const CANDIDATE_HZ: &[(f64, &str)] = &[
    (20.0, "20 Hz (50ms)"),
    (10.0, "10 Hz (100ms)"),
    (5.0, "5 Hz (200ms)"),
    (1.0, "1 Hz (1 second)"),
    (0.5, "0.5 Hz (2 seconds)"),
];

/// Formats a power reading with unit scaling (uW/mW/W) rather than a
/// fixed "X.XXX mW" -- at typical calibration-sweep low-power readings
/// (tens of uW), a fixed 3-decimal mW display just rounds to 0.000,
/// which is what "always reads 0.000 mW" actually was: not a wrong
/// value, an unusable display precision for the range this tool covers.
fn format_power(mw: f32) -> String {
    if mw >= 1000.0 {
        format!("{:.3} W", mw / 1000.0)
    } else if mw >= 1.0 {
        format!("{:.3} mW", mw)
    } else {
        format!("{:.1} \u{b5}W", mw * 1000.0)
    }
}

pub struct CalibrationPageState {
    pub tolerance_pct: f32,
    pub checked: HashMap<u8, bool>,
    show_confirm_dialog: bool,
}

impl Default for CalibrationPageState {
    fn default() -> Self {
        Self {
            tolerance_pct: 10.0, // matches rf_calibration.py's own scanDetector default (max(0.1, mW*0.1))
            checked: HashMap::new(),
            show_confirm_dialog: false,
        }
    }
}

/// Group-header band colors -- deliberately muted/desaturated to sit
/// quietly in a dark theme rather than compete with the per-cell status
/// colors below, which are the ones actually meant to draw the eye.
const GROUP_CAL_COLOR: egui::Color32 = egui::Color32::from_rgb(45, 55, 70);
const GROUP_DET_COLOR: egui::Color32 = egui::Color32::from_rgb(55, 48, 66);

fn cell_color(status: CellStatus) -> Option<egui::Color32> {
    match status {
        CellStatus::Default => None,
        CellStatus::Calibrated => Some(egui::Color32::from_rgb(50, 120, 65)), // green
        CellStatus::Current => Some(egui::Color32::from_rgb(50, 100, 170)),   // blue
        CellStatus::LimitHit => Some(egui::Color32::from_rgb(165, 55, 55)),   // red
        CellStatus::Uncalibrated => Some(egui::Color32::from_rgb(185, 125, 40)), // orange
    }
}

/// Renders one calibration/detector value cell, background-tinted per
/// its CellStatus (or left as a plain label, letting the Grid's own
/// striping show through, when Default).
fn colored_cell(ui: &mut egui::Ui, text: String, status: CellStatus) {
    match cell_color(status) {
        Some(color) => {
            ui.label(egui::RichText::new(text).background_color(color));
        }
        None => {
            ui.label(text);
        }
    }
}

fn status_text(status: Option<&LevelStatus>) -> String {
    match status {
        None | Some(LevelStatus::NotCalibrated) => "Not calibrated".to_string(),
        Some(LevelStatus::Pending) => "Pending".to_string(),
        Some(LevelStatus::InProgress(s)) => s.clone(),
        Some(LevelStatus::Done) => "Done".to_string(),
        Some(LevelStatus::Aborted) => "Aborted".to_string(),
    }
}

pub fn show(
    ui: &mut egui::Ui,
    shared: &Arc<Mutex<SharedState>>,
    sweep: &SharedSweep,
    cmd_tx: &Sender<Command>,
    page: &mut CalibrationPageState,
) {
    ui.heading("RF Calibration");

    ui.separator();

    {
        let state = shared.lock().unwrap();
        ui.label(match state.last_dbm {
            Some(dbm) => format!(
                "Power meter: {:.2} dBm  ({})",
                dbm,
                format_power(10f32.powf(dbm / 10.0))
            ),
            None => "Power meter: (no reading yet)".to_string(),
        });

        let points: PlotPoints = state
            .power_history
            .iter()
            .map(|&(t, mw)| [t, mw as f64])
            .collect();
        let latest_t = state.power_history.back().map(|&(t, _)| t).unwrap_or(0.0);

        Plot::new("power_history_plot")
            .height(180.0)
            .include_x(latest_t - HISTORY_WINDOW_SECS)
            .include_x(latest_t)
            .x_axis_label("seconds")
            .y_axis_label("mW")
            .show(ui, |plot_ui| {
                plot_ui.line(Line::new("Power (mW)", points));
            });
    }

    ui.horizontal(|ui| {
        ui.label("Update frequency:");
        let mut hz = shared.lock().unwrap().update_hz;
        let max_hz = shared.lock().unwrap().meter_kind.max_update_hz() as f64;
        let selected_label = CANDIDATE_HZ
            .iter()
            .find(|&&(h, _)| h == hz)
            .map(|&(_, label)| label.to_string())
            .unwrap_or_else(|| format!("{hz} Hz"));
        egui::ComboBox::from_id_salt("update_hz")
            .selected_text(selected_label)
            .show_ui(ui, |ui| {
                for &(h, l) in CANDIDATE_HZ.iter().filter(|&&(h, _)| h <= max_hz) {
                    ui.selectable_value(&mut hz, h, l);
                }
            });
        shared.lock().unwrap().update_hz = hz;
    });

    ui.separator();
    ui.heading("PA calibration table");
    if ui.button("Refresh").clicked() {
        let _ = cmd_tx.send(Command::RefreshCalTable);
    }

    let sweep_active = sweep.lock().unwrap().as_ref().map(|e| e.is_active()).unwrap_or(false);

    // ---- Frequency-change gate (manual-frequency meters) ---------------
    let awaiting_freq_mhz = {
        let g = sweep.lock().unwrap();
        g.as_ref().and_then(|e| match e.state {
            calibration_engine::EngineState::AwaitingFreqConfirm { freq_mhz } => Some(freq_mhz),
            _ => None,
        })
    };
    if let Some(freq_mhz) = awaiting_freq_mhz {
        egui::Window::new("Set power meter frequency")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ui.ctx(), |ui| {
                ui.label(format!("Set your power meter to {freq_mhz} MHz, then continue."));
                ui.horizontal(|ui| {
                    if ui.button("Confirm").clicked() {
                        let _ = cmd_tx.send(Command::ConfirmFrequency);
                    }
                    if ui.button("Abort").clicked() {
                        let _ = cmd_tx.send(Command::AbortSweep);
                    }
                });
            });
    }

    // ---- VTX-unresponsive gate (current-limited supply power-cycled it) --
    let unresponsive = {
        let g = sweep.lock().unwrap();
        g.as_ref().and_then(|e| match e.state {
            calibration_engine::EngineState::VtxUnresponsive { level, freq_mhz, mv_at_loss } => {
                Some((level, freq_mhz, mv_at_loss))
            }
            _ => None,
        })
    };
    if let Some((level, freq_mhz, mv_at_loss)) = unresponsive {
        let vtx_ready = shared.lock().unwrap().vtx_ready;
        egui::Window::new("VTX not responding")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ui.ctx(), |ui| {
                ui.label(format!(
                    "No response from the VTX while calibrating level {level} at {freq_mhz}MHz (mv={mv_at_loss}). \
                     If it's on a current-limited supply, it may have powered off."
                ));
                ui.label("Please power the VTX back on.");
                ui.horizontal(|ui| {
                    ui.label("Connection status:");
                    conn_status::show(ui, Some(conn_status::ConnStatus::from_ready(vtx_ready)));
                });
                ui.horizontal(|ui| {
                    ui.add_enabled_ui(vtx_ready, |ui| {
                        if ui.button("Continue").clicked() {
                            let _ = cmd_tx.send(Command::ResumeAfterVtxRecovery);
                        }
                    });
                    if ui.button("Abort").clicked() {
                        let _ = cmd_tx.send(Command::AbortSweep);
                    }
                });
            });
    }

    // ---- Table -----------------------------------------------------------
    let pa_table = shared.lock().unwrap().pa_table.clone();
    let frequencies: [u16; 7] = pa_table.iter().find(|e| e.idx == 0).map(|e| e.value).unwrap_or([0; 7]);

    egui::Grid::new("pa_table").striped(true).show(ui, |ui| {
        // Group-header band: every cell in a span gets the same tinted
        // background (via RichText::background_color), with the label only
        // in the middle cell -- egui::Grid has no native merged/spanning
        // cell, so a shared-color band across the span is the closest
        // equivalent without pulling in a heavier table widget. This has
        // to be a row WITHIN this same Grid (not a separate one) so its
        // column widths are computed together with the label/data rows
        // below and everything actually lines up.
        for _ in 0..3 {
            ui.label(""); // checkbox / idx / mW -- not part of either group
        }
        for i in 0..7 {
            let text = if i == 3 { "Calibration (mV)" } else { "" };
            ui.label(egui::RichText::new(text).background_color(GROUP_CAL_COLOR));
        }
        for i in 0..7 {
            let text = if i == 3 { "Detector (mV)" } else { "" };
            ui.label(egui::RichText::new(text).background_color(GROUP_DET_COLOR));
        }
        for _ in 0..4 {
            ui.label(""); // Boost / RTC6705 / Limit / Status -- not part of either group
        }
        ui.end_row();

        ui.strong("");
        ui.strong("idx");
        ui.strong("mW");
        for f in frequencies {
            ui.strong(format!("{f}"));
        }
        for f in frequencies {
            ui.strong(format!("{f}"));
        }
        ui.strong("Boost");
        ui.strong("RTC6705");
        ui.strong("Limit (mV)");
        ui.strong("Status");
        ui.end_row();

        for entry in pa_table.iter().filter(|e| e.idx > 0) {
            // Default-checked: levels that engage the boost stage (ext_pa_enable) --
            // those are what scanPa/scanDetector's detector-based closed loop is
            // meant for; the RTC6705-alone levels are simple/structural and were
            // never really candidates for this procedure (see the target power
            // table's own comments). Only applies the FIRST time a given idx is
            // seen -- a user's manual (un)check is never overwritten by this,
            // including across table Refreshes.
            let checked = page.checked.entry(entry.idx).or_insert(entry.ext_pa_enable);
            ui.add_enabled(!sweep_active, egui::Checkbox::new(checked, ""));
            ui.label(entry.idx.to_string());
            ui.label(entry.m_w.to_string());

            let (cal_status, det_status, level_status, limit) = {
                let g = sweep.lock().unwrap();
                let cal_status: [CellStatus; 7] =
                    std::array::from_fn(|i| g.as_ref().and_then(|e| e.cal_cell_status.get(&(entry.idx, i)).copied()).unwrap_or(CellStatus::Default));
                let det_status: [CellStatus; 7] =
                    std::array::from_fn(|i| g.as_ref().and_then(|e| e.det_cell_status.get(&(entry.idx, i)).copied()).unwrap_or(CellStatus::Default));
                let level_status = g.as_ref().and_then(|e| e.per_level_status.get(&entry.idx).cloned());
                let limit = g.as_ref().and_then(|e| e.hard_limits.get(&entry.idx).copied());
                (cal_status, det_status, level_status, limit)
            };

            for i in 0..7 {
                colored_cell(ui, entry.value[i].to_string(), cal_status[i]);
            }
            for i in 0..7 {
                colored_cell(ui, entry.detector[i].to_string(), det_status[i]);
            }

            ui.label(if entry.ext_pa_enable { "Yes" } else { "No" });
            ui.label(entry.rtc6705_level.to_string());
            ui.label(limit.map(|v| v.to_string()).unwrap_or_else(|| "-".to_string()));
            ui.label(status_text(level_status.as_ref()));
            ui.end_row();
        }
    });

    // ---- Progress bars -------------------------------------------------
    {
        let g = sweep.lock().unwrap();
        if let Some(engine) = g.as_ref() {
            if engine.is_active() || engine.completed_steps > 0 {
                ui.add(egui::ProgressBar::new(engine.progress()).text("Overall").show_percentage());

                let (sub, label) = engine.sub_progress();
                if !label.is_empty() {
                    ui.add(egui::ProgressBar::new(sub).text(label));
                }
            }
        }
    }

    // ---- Tolerance + Re-calibrate/Stop --------------------------------
    ui.horizontal(|ui| {
        ui.label("Scan detector tolerance:");
        ui.add(
            egui::DragValue::new(&mut page.tolerance_pct)
                .range(0.1..=50.0)
                .suffix("%")
                .speed(0.1),
        );

        if sweep_active {
            if ui.button("Stop").clicked() {
                let _ = cmd_tx.send(Command::AbortSweep);
            }
        } else {
            let any_checked = page.checked.values().any(|&v| v);
            ui.add_enabled_ui(any_checked, |ui| {
                if ui.button("Re-calibrate").clicked() {
                    page.show_confirm_dialog = true;
                }
            });
        }
    });

    if page.show_confirm_dialog {
        let mut open = true;
        let mut confirmed = false;
        egui::Window::new("Confirm Recalibration")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .open(&mut open)
            .show(ui.ctx(), |ui| {
                ui.label("Recalibrate the checked power levels? This drives real RF output on the VTX.");
                ui.horizontal(|ui| {
                    if ui.button("Yes").clicked() {
                        confirmed = true;
                    }
                    if ui.button("No").clicked() {
                        page.show_confirm_dialog = false;
                    }
                });
            });
        if !open {
            // Closed via the window's own X -- default is No, per spec.
            page.show_confirm_dialog = false;
        }
        if confirmed {
            page.show_confirm_dialog = false;
            let levels: Vec<u8> = page.checked.iter().filter(|&(_, &v)| v).map(|(&k, _)| k).collect();
            let _ = cmd_tx.send(Command::StartSweep { levels, tolerance_pct: page.tolerance_pct });
        }
    }

    // ---- Send to VTX / Save EEPROM (independent) -----------------------
    ui.horizontal(|ui| {
        if ui.button("Send to VTX").clicked() {
            let _ = cmd_tx.send(Command::SendCalTableToVtx);
        }

        let any_calibrated = {
            let g = sweep.lock().unwrap();
            g.as_ref()
                .map(|e| e.per_level_status.values().any(|s| matches!(s, LevelStatus::Done)))
                .unwrap_or(false)
        };
        ui.add_enabled_ui(any_calibrated, |ui| {
            if ui.button("Save EEPROM").clicked() {
                let _ = cmd_tx.send(Command::SaveEeprom);
            }
        });
    });
}
