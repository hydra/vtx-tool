//! Calibration page: live power meter reading (with a rolling plot), the
//! PA calibration table (now with per-level checkboxes, boost/RTC6705
//! display columns, and a live calibration-status column), and the
//! Automatic/Manual sweep controls (see calibration_engine.rs for the actual sweep
//! state machine this drives).

use crate::calibration_engine::{self, CellStatus, LevelStatus};
use crate::conn_status;
use crate::msp;
use crate::power_meter::{nearest_band, FrequencyCapability};
use crate::settings::AppSettings;
use crate::worker::{Command, SharedState, SharedSweep, HISTORY_WINDOW_SECS};
use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints};
use std::collections::HashMap;
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};
use egui_table::AutoSizeMode;

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
    /// Manual mode's "fine" checkbox: true = the DAC slider moves in
    /// 1mV steps, false = 25mV steps.
    fine_step: bool,
}

impl Default for CalibrationPageState {
    fn default() -> Self {
        Self {
            tolerance_pct: 10.0, // matches rf_calibration.py's own scanDetector default (max(0.1, mW*0.1))
            checked: HashMap::new(),
            show_confirm_dialog: false,
            fine_step: false,
        }
    }
}

/// All four status colors share the same muted character (similar
/// darkness/saturation, only the hue changes) so none of them stand out
/// as harsher or harder to read than the others against default (light)
/// cell text -- the earlier bright orange was the main offender.
fn cell_color(status: CellStatus) -> Option<egui::Color32> {
    match status {
        CellStatus::Default => None,
        CellStatus::Calibrated => Some(egui::Color32::from_rgb(45, 85, 55)),    // muted green
        CellStatus::Current => Some(egui::Color32::from_rgb(45, 70, 100)),      // muted blue
        CellStatus::LimitHit => Some(egui::Color32::from_rgb(100, 45, 45)),     // muted red
        CellStatus::Uncalibrated => Some(egui::Color32::from_rgb(100, 78, 40)), // muted amber
        CellStatus::Skipped => Some(egui::Color32::from_rgb(70, 70, 75)),       // neutral grey -- deliberate, not a failure
        CellStatus::PaFailure => Some(egui::Color32::from_rgb(165, 80, 15)),    // distinct burnt orange -- hardware condition, not just a non-convergent search
        CellStatus::Manual => Some(egui::Color32::from_rgb(80, 55, 100)),       // muted violet -- hand-set, distinct from an automatic Calibrated result
    }
}

/// Renders one calibration/detector value cell inside an egui_table
/// column, filling the ENTIRE cell rect with its status color (painted
/// first, full width/height) rather than just tinting behind the text --
/// RichText::background_color only covers the glyphs' own bounding box,
/// which is what this replaces. Also forces no-wrap (Truncate, matching
/// the pattern egui_table's own demo uses for a long cell) -- without
/// it, a narrow column wraps text one character per line rather than
/// clipping it.
fn colored_cell(ui: &mut egui::Ui, text: String, status: CellStatus) {
    if let Some(color) = cell_color(status) {
        ui.painter().rect_filled(ui.max_rect(), 0.0, color);
    }
    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Truncate);
    ui.centered_and_justified(|ui| ui.label(text));
}

/// Column widths, in the same 21-column order used throughout: checkbox,
/// idx, mW, 7 calibration columns, 7 detector columns, Boost, RTC6705,
/// Limit, Status. Fixed/explicit (not content-driven), which is what
/// actually stops any column from growing to fit a header label.
const COL_WIDTHS: [f32; 21] = [
    24.0, 28.0, 40.0, // checkbox, idx, mW
    55.0, 55.0, 55.0, 55.0, 55.0, 55.0, 55.0, // calibration x7
    55.0, 55.0, 55.0, 55.0, 55.0, 55.0, 55.0, // detector x7
    45.0, 65.0, 70.0, 160.0, // Boost, RTC6705, Limit, Status
];
const GROUP_ROW_HEIGHT: f32 = 18.0;
const HEADER_ROW_HEIGHT: f32 = 18.0;
const DATA_ROW_HEIGHT: f32 = 20.0;

/// egui_table::TableDelegate implementation for the PA calibration
/// table. Unlike egui_extras::TableBuilder's closures (which directly
/// capture surrounding variables), egui_table's Table::show() drives a
/// delegate object across possibly-multiple frames/prefetch passes, so
/// state it needs has to be gathered up front into an owned/borrowed
/// struct rather than closed over ad hoc.
struct PaTableDelegate<'a> {
    entries: Vec<msp::PaCalibration>, // owned, filtered to idx > 0
    frequencies: [u16; 7],
    checked: &'a mut HashMap<u8, bool>,
    sweep_active: bool,
    cal_status: Vec<[CellStatus; 7]>,
    det_status: Vec<[CellStatus; 7]>,
    level_status: Vec<Option<LevelStatus>>,
    limits: Vec<Option<i32>>,
    /// Whatever's actively being tested right now, if anything -- see
    /// calibration_engine.rs's SweepEngine::current_step(). Used so the
    /// Current (blue) cell shows the LIVE value being tried, not the
    /// table's last-saved value for that cell, which stays stale/default
    /// until the step actually finishes.
    current: Option<calibration_engine::CurrentStep>,
    row_height: f32,
}

impl PaTableDelegate<'_> {
    fn column_label(&self, col: usize) -> String {
        match col {
            0 => String::new(),
            1 => "idx".to_string(),
            2 => "mW".to_string(),
            3..=9 => self.frequencies[col - 3].to_string(),
            10..=16 => self.frequencies[col - 10].to_string(),
            17 => "Boost".to_string(),
            18 => "RTC6705".to_string(),
            19 => "Limit (mV)".to_string(),
            20 => "Status".to_string(),
            _ => String::new(),
        }
    }
}

impl egui_table::TableDelegate for PaTableDelegate<'_> {
    fn prepare(&mut self, _info: &egui_table::PrefetchInfo) {
        // Nothing to lazily load -- everything's already gathered up
        // front into owned Vecs before the Table is constructed.
    }

    fn header_cell_ui(&mut self, ui: &mut egui::Ui, cell_inf: &egui_table::HeaderCellInfo) {
        let egui_table::HeaderCellInfo { group_index, row_nr, .. } = cell_inf;
        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Truncate);
        match row_nr {
            0 => {
                // Group band row. group_index here is the GROUP's own index
                // (0..4, matching the `groups` Vec passed to HeaderRow), not
                // a column index. Groups 1/2 are Calibration/Detector;
                // 0 and 3 are the unlabeled surrounding columns.
                let text = match group_index {
                    1 => "VBIAS (mV)",
                    2 => "Detector (mV)",
                    _ => "",
                };
                if !text.is_empty() {
                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        ui.strong(text);
                    });
                }
            }
            1 => {
                // Second header row has no explicit `groups`, so
                // egui_table gives each column its own trivial group --
                // group_index here IS the column index directly.
                ui.strong(self.column_label(*group_index));
            }
            _ => {}
        }
    }

    fn row_ui(&mut self, _ui: &mut egui::Ui, _row_nr: u64) {
        // No special per-row interaction/highlight needed beyond what
        // cell_ui already paints.
    }

    fn cell_ui(&mut self, ui: &mut egui::Ui, cell_info: &egui_table::CellInfo) {
        let egui_table::CellInfo { row_nr, col_nr, .. } = *cell_info;
        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Truncate);
        let row_nr = row_nr as usize;
        let Some(entry) = self.entries.get(row_nr) else {
            return;
        };

        match col_nr {
            0 => {
                // Default-checked: levels that engage the boost stage
                // (ext_pa_enable) -- those are what scanPa/scanDetector's
                // detector-based closed loop is meant for; the RTC6705-alone
                // levels are simple/structural and were never really
                // candidates for this procedure. Only applies the FIRST
                // time a given idx is seen -- a user's manual (un)check is
                // never overwritten by this, including across Refreshes.
                let checked = self.checked.entry(entry.idx).or_insert(entry.ext_pa_enable);
                ui.add_enabled(!self.sweep_active, egui::Checkbox::new(checked, ""));
            }
            1 => {
                ui.label(entry.idx.to_string());
            }
            2 => {
                ui.label(entry.m_w.to_string());
            }
            3..=9 => {
                let i = col_nr - 3;
                let text = self
                    .current
                    .as_ref()
                    .filter(|cs| cs.level == entry.idx && cs.freq_idx == i)
                    .and_then(|cs| cs.vbias_mv)
                    .map(|v| v.to_string())
                    .unwrap_or_else(|| entry.value[i].to_string());
                colored_cell(ui, text, self.cal_status[row_nr][i]);
            }
            10..=16 => {
                let i = col_nr - 10;
                let text = self
                    .current
                    .as_ref()
                    .filter(|cs| cs.level == entry.idx && cs.freq_idx == i)
                    .and_then(|cs| cs.detector_mv)
                    .map(|v| v.to_string())
                    .unwrap_or_else(|| entry.detector[i].to_string());
                colored_cell(ui, text, self.det_status[row_nr][i]);
            }
            17 => {
                ui.label(if entry.ext_pa_enable { "Yes" } else { "No" });
            }
            18 => {
                ui.label(entry.rtc6705_level.to_string());
            }
            19 => {
                ui.label(self.limits[row_nr].map(|v| v.to_string()).unwrap_or_else(|| "-".to_string()));
            }
            20 => {
                ui.label(status_text(self.level_status[row_nr].as_ref()));
            }
            _ => {}
        }
    }

    fn row_top_offset(&self, _ctx: &egui::Context, _table_id: egui::Id, row_nr: u64) -> f32 {
        row_nr as f32 * self.row_height // uniform row height, no per-row expand/collapse
    }
}

fn status_text(status: Option<&LevelStatus>) -> String {
    match status {
        None | Some(LevelStatus::NotCalibrated) => "Not calibrated".to_string(),
        Some(LevelStatus::Pending) => "Pending".to_string(),
        Some(LevelStatus::InProgress(s)) => s.clone(),
        Some(LevelStatus::Done) => "Done".to_string(),
        Some(LevelStatus::Aborted) => "Aborted".to_string(),
        Some(LevelStatus::Skipped) => "Skipped".to_string(),
        Some(LevelStatus::PaFailure) => "PA Failure".to_string(),
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

        ui.separator();
        ui.label("Attenuation:");
        let mut atten_db = shared.lock().unwrap().attenuation_db;
        // Added to every raw reading from the meter (see worker.rs) --
        // e.g. an external attenuator fitted ahead of the meter that its
        // own display already accounts for but the raw serial readings
        // don't. Range allows negative too, for the inverse case (a
        // pre-amp ahead of the meter rather than an attenuator).
        let response = ui.add(egui::DragValue::new(&mut atten_db).suffix(" dB").range(-50.0..=100.0).speed(0.1));
        if response.changed() {
            shared.lock().unwrap().attenuation_db = atten_db;
            let mut settings = AppSettings::load();
            settings.attenuation_db = atten_db;
            let _ = settings.save(); // best-effort -- same pattern as the port fields
        }
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
        let prompt_mhz = match shared.lock().unwrap().meter_kind.capability() {
            FrequencyCapability::ManualBand { bands_mhz } => nearest_band(&bands_mhz, freq_mhz as u32),
            _ => freq_mhz as u32,
        };
        egui::Window::new("Set power meter frequency")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ui.ctx(), |ui| {
                ui.label(format!("Set your power meter to {prompt_mhz} MHz, then continue."));
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

    // ---- Connection error gate (VTX and/or meter lost communication) ----
    let connection_lost = {
        let g = sweep.lock().unwrap();
        g.as_ref().and_then(|e| match e.state {
            calibration_engine::EngineState::ConnectionLost { level, freq_mhz, vbias_mv_at_loss, reason } => {
                Some((level, freq_mhz, vbias_mv_at_loss, reason))
            }
            _ => None,
        })
    };
    if let Some((level, freq_mhz, vbias_mv_at_loss, reason)) = connection_lost {
        let (vtx_state, meter_state) = {
            let s = shared.lock().unwrap();
            (s.vtx_port_state, s.meter_port_state)
        };
        egui::Window::new("Connection error")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ui.ctx(), |ui| {
                let cause = match reason {
                    calibration_engine::ConnectionLossReason::Vtx => "the VTX",
                    calibration_engine::ConnectionLossReason::Meter => "the power meter",
                    calibration_engine::ConnectionLossReason::Both => "the VTX and the power meter",
                };
                ui.label(format!(
                    "Lost communication with {cause} while calibrating level {level} at {freq_mhz}MHz (vbias_mv={vbias_mv_at_loss}). \
                     If the VTX is on a current-limited supply, it may have powered off."
                ));
                ui.label("Reconnecting automatically -- resumes on its own once both are Ready again.");
                ui.horizontal(|ui| {
                    ui.label("VTX:");
                    conn_status::show_port(ui, vtx_state);
                });
                ui.horizontal(|ui| {
                    ui.label("Power Meter:");
                    conn_status::show_port(ui, meter_state);
                });
                if ui.button("Abort").clicked() {
                    let _ = cmd_tx.send(Command::AbortSweep);
                }
            });
    }

    // ---- Table -----------------------------------------------------------
    let pa_table = shared.lock().unwrap().pa_table.clone();
    let frequencies: [u16; 7] = pa_table.iter().find(|e| e.idx == 0).map(|e| e.value).unwrap_or([0; 7]);
    let entries: Vec<msp::PaCalibration> = pa_table.iter().filter(|e| e.idx > 0).cloned().collect();

    let mut cal_status = Vec::with_capacity(entries.len());
    let mut det_status = Vec::with_capacity(entries.len());
    let mut level_status = Vec::with_capacity(entries.len());
    let mut limits = Vec::with_capacity(entries.len());
    let mut current = None;
    {
        let g = sweep.lock().unwrap();
        current = g.as_ref().and_then(|e| e.current_step());
        for entry in &entries {
            let cal: [CellStatus; 7] = std::array::from_fn(|i| {
                g.as_ref().and_then(|e| e.cal_cell_status.get(&(entry.idx, i)).copied()).unwrap_or(CellStatus::Default)
            });
            let det: [CellStatus; 7] = std::array::from_fn(|i| {
                g.as_ref().and_then(|e| e.det_cell_status.get(&(entry.idx, i)).copied()).unwrap_or(CellStatus::Default)
            });
            cal_status.push(cal);
            det_status.push(det);
            level_status.push(g.as_ref().and_then(|e| e.per_level_status.get(&entry.idx).cloned()));
            limits.push(g.as_ref().and_then(|e| e.hard_limits.get(&entry.idx).copied()));
        }
    }

    let mut delegate = PaTableDelegate {
        entries,
        frequencies,
        checked: &mut page.checked,
        sweep_active,
        cal_status,
        det_status,
        level_status,
        limits,
        current,
        row_height: DATA_ROW_HEIGHT,
    };

    // range(w..=w) is the actual fix here, not resizable(false) alone --
    // without an explicit range, columns were auto-shrinking to fit
    // whatever width was available (confirmed by the group header text
    // truncating despite 7*55=385px of nominal span width) instead of
    // the table scrolling horizontally when content exceeds the viewport.
    let columns: Vec<egui_table::Column> =
        COL_WIDTHS.iter().enumerate().map(|(index, &w)| {

            let last = index == COL_WIDTHS.len() - 1;

            let range = if last {
                w..=(w * 4.0)
            } else {
                w..=w
            };

            egui_table::Column::new(w)
                .resizable(false)
                .range(range)
        })
            .collect();
    let num_rows = delegate.entries.len() as f32;
    // egui_table::Table is built for virtualized scrolling of potentially
    // huge datasets, so its scroll region defaults to filling whatever
    // vertical space the parent ui hands it -- for our handful of rows
    // that left a large empty gap below the real content. Constraining
    // the allocation to the table's actual content height (both header
    // rows + N data rows, plus a little slack) fixes that; it'll still
    // scroll internally if a future table genuinely has more rows than fit.
    let table_height = GROUP_ROW_HEIGHT + HEADER_ROW_HEIGHT + num_rows * DATA_ROW_HEIGHT + 4.0;
    ui.allocate_ui(egui::vec2(ui.available_width(), table_height), |ui| {
        egui_table::Table::new()
            .id_salt("pa_table")
            .auto_size_mode(AutoSizeMode::Always)
            .num_rows(delegate.entries.len() as u64)
            .columns(columns)
            .num_sticky_cols(0)
            .headers([
                // Row 0: group band -- spans verified against egui_table's own
                // demo (rerun-io/egui_table, demo/src/table_demo.rs), which is
                // the actual mechanism for this (egui::Grid/egui_extras::Table
                // have no equivalent). Groups don't need to cover every column;
                // 0..3 and 17..21 are left as trivial/unlabeled groups.
                egui_table::HeaderRow {
                    height: GROUP_ROW_HEIGHT,
                    groups: vec![0..3, 3..10, 10..17, 17..21],
                },
                egui_table::HeaderRow::new(HEADER_ROW_HEIGHT),
            ])
            .show(ui, &mut delegate);
    });

    // ---- Progress bars (always shown) -----------------------------------
    {
        let g = sweep.lock().unwrap();
        let (overall, sub_val, sub_label) = match g.as_ref() {
            Some(engine) => {
                let (sub, label) = engine.sub_progress();
                (engine.progress(), sub, label)
            }
            None => (0.0, 0.0, ""),
        };
        ui.add(egui::ProgressBar::new(overall).text("Overall").show_percentage());
        ui.add(egui::ProgressBar::new(sub_val).text(if sub_label.is_empty() { "Idle" } else { sub_label }));
    }

    // ---- Tolerance + Automatic/Manual/Stop/Skip -------------------------
    let (automatic_mode, manual_mode, manual_dac_mv) = {
        let g = sweep.lock().unwrap();
        match g.as_ref() {
            Some(e) => (e.is_automatic_mode(), e.is_manual_mode(), e.manual_dac_mv),
            None => (false, false, 0),
        }
    };
    // boost_on comes from live VTX telemetry (SharedState), not the engine --
    // it's the actual GPIO state reported back, not just what was last requested.
    let pa_boost_on = {
        let s = shared.lock().unwrap();
        s.vtx_status.as_ref().and_then(|v| v.boost_on)
    };
    let any_checked = page.checked.values().any(|&v| v);
    let overall_ready = {
        let s = shared.lock().unwrap();
        conn_status::OverallState::from_ports(s.vtx_port_state, s.meter_port_state) == conn_status::OverallState::Ready
    };

    ui.horizontal(|ui| {
        ui.label("Scan detector tolerance:");
        ui.add(
            egui::DragValue::new(&mut page.tolerance_pct)
                .range(0.1..=50.0)
                .suffix("%")
                .speed(0.1),
        );

        // Automatic: starts fresh from Idle (with a confirm dialog), or
        // resumes in place if Manual mode is currently active (see
        // resume_automatic_from_current()) -- disabled only while
        // automatic is already the active mode; the extra !manual_mode-
        // aware "any_checked" requirement only applies to the fresh-start
        // case, since resuming doesn't need a fresh level selection.
        let automatic_enabled = !automatic_mode && overall_ready && (manual_mode || any_checked);
        if ui.add_enabled(automatic_enabled, egui::Button::new("Automatic")).clicked() {
            if manual_mode {
                let mut levels: Vec<u8> = page.checked.iter().filter(|&(_, &v)| v).map(|(&k, _)| k).collect();
                levels.sort_unstable();
                let _ = cmd_tx.send(Command::StartSweep { levels, tolerance_pct: page.tolerance_pct });
            } else {
                page.show_confirm_dialog = true;
            }
        }

        // Manual: only usable from Idle -- deliberately also disabled
        // while automatic is running (not just while already manual),
        // since starting Manual mode replaces the engine outright and
        // doing that mid-automatic-run would abruptly end it without a
        // proper abort/session-end.
        let manual_enabled = !automatic_mode && !manual_mode && overall_ready && any_checked;
        if ui.add_enabled(manual_enabled, egui::Button::new("Manual")).clicked() {
            let mut levels: Vec<u8> = page.checked.iter().filter(|&(_, &v)| v).map(|(&k, _)| k).collect();
            levels.sort_unstable();
            let _ = cmd_tx.send(Command::StartManual { levels });
        }

        if ui.add_enabled(sweep_active, egui::Button::new("Stop")).clicked() {
            let _ = cmd_tx.send(Command::AbortSweep);
        }
        if ui.add_enabled(sweep_active, egui::Button::new("Skip >")).clicked() {
            let _ = cmd_tx.send(Command::SkipCurrent);
        }

        if any_checked && !overall_ready {
            ui.label(
                egui::RichText::new("Both VTX and power meter must be Ready to calibrate.")
                    .weak()
                    .italics(),
            );
        }
    });

    // ---- Manual mode controls (always shown, disabled outside Manual) ---
    ui.horizontal(|ui| {
        ui.add_enabled_ui(manual_mode, |ui| {
            let mut current_mv = manual_dac_mv;
            let step = if page.fine_step { 1.0 } else { 25.0 };
            let response =
                ui.add(egui::Slider::new(&mut current_mv, 0..=3300).text("DAC mV").step_by(step));
            if response.changed() {
                let _ = cmd_tx.send(Command::SetManualDac { mv: current_mv });
            }
            ui.checkbox(&mut page.fine_step, "fine");

            let mut pa_on = pa_boost_on.unwrap_or(false);
            if ui.checkbox(&mut pa_on, "PA").changed() {
                let _ = cmd_tx.send(Command::SetPaBoost { on: pa_on });
            }

            if ui.button("Next >").clicked() {
                let _ = cmd_tx.send(Command::ManualNext);
            }
        });
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
            let mut levels: Vec<u8> = page.checked.iter().filter(|&(_, &v)| v).map(|(&k, _)| k).collect();
            levels.sort_unstable(); // HashMap iteration order is arbitrary -- without this, the sweep ran the selected rows in a random order instead of top-to-bottom
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
