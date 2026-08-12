//! Calibration page: live power meter reading (with a rolling plot) +
//! PA calibration table pull.

use crate::worker::{Command, SharedState, HISTORY_WINDOW_SECS};
use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints};
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

pub fn show(ui: &mut egui::Ui, shared: &Arc<Mutex<SharedState>>, cmd_tx: &Sender<Command>) {
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
                for &(candidate, label) in CANDIDATE_HZ {
                    ui.add_enabled_ui(candidate <= max_hz, |ui| {
                        ui.selectable_value(&mut hz, candidate, label);
                    });
                }
            });
        shared.lock().unwrap().update_hz = hz;
    });

    ui.separator();
    ui.heading("PA calibration table");
    if ui.button("Refresh from VTX").clicked() {
        let _ = cmd_tx.send(Command::RefreshCalTable);
    }

    let state = shared.lock().unwrap();
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
}
