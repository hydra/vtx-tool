
use crate::app::grid_label;
use crate::calibration_engine::{self, CellStatus, LevelStatus};
use crate::conn_status;
use crate::msp;
use crate::power_meter::{nearest_band, FrequencyCapability};
use crate::settings::AppSettings;
use crate::worker::{Command, SharedState, SharedSweep, HISTORY_WINDOW_SECS};
use eframe::egui;
use egui_plot::{AxisHints, HPlacement, Legend, Line, Plot, PlotPoints};
use std::collections::HashMap;
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use egui::SliderClamping;
use egui_table::AutoSizeMode;

const CANDIDATE_HZ: &[(f64, &str)] = &[
    (20.0, "20 Hz (50ms)"),
    (10.0, "10 Hz (100ms)"),
    (5.0, "5 Hz (200ms)"),
    (1.0, "1 Hz (1 second)"),
    (0.5, "0.5 Hz (2 seconds)"),
];

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
    show_erase_confirm_dialog: bool,
    fine_step: bool,
    plot_reset_requested: bool,
    pending_skip_count: u32,
    skip_debounce_until: Option<Instant>,
}

impl Default for CalibrationPageState {
    fn default() -> Self {
        Self {
            tolerance_pct: 10.0,
            checked: HashMap::new(),
            show_confirm_dialog: false,
            show_erase_confirm_dialog: false,
            fine_step: false,
            plot_reset_requested: false,
            pending_skip_count: 0,
            skip_debounce_until: None,
        }
    }
}

const SKIP_DEBOUNCE: Duration = Duration::from_millis(500);

fn cell_color(status: CellStatus) -> Option<egui::Color32> {
    match status {
        CellStatus::Default => None,
        CellStatus::Calibrated => Some(egui::Color32::from_rgb(45, 85, 55)),
        CellStatus::Current => Some(egui::Color32::from_rgb(45, 70, 100)),
        CellStatus::LimitHit => Some(egui::Color32::from_rgb(100, 45, 45)),
        CellStatus::Uncalibrated => Some(egui::Color32::from_rgb(100, 78, 40)),
        CellStatus::Skipped => Some(egui::Color32::from_rgb(70, 70, 75)),
        CellStatus::PaFailure => Some(egui::Color32::from_rgb(165, 80, 15)),
        CellStatus::NotSettled => Some(egui::Color32::from_rgb(140, 100, 20)),
        CellStatus::Manual => Some(egui::Color32::from_rgb(80, 55, 100)),
    }
}

fn colored_cell(ui: &mut egui::Ui, text: String, status: CellStatus) {
    if let Some(color) = cell_color(status) {
        ui.painter().rect_filled(ui.max_rect(), 0.0, color);
    }
    if status == CellStatus::Current {
        let stroke = egui::Stroke::new(2.0, ui.visuals().selection.stroke.color);
        ui.painter().rect_stroke(ui.max_rect(), 0.0, stroke, egui::StrokeKind::Inside);
    }
    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Truncate);
    ui.centered_and_justified(|ui| ui.label(text));
}

const COL_WIDTHS: [f32; 21] = [
    24.0, 28.0, 40.0,
    55.0, 55.0, 55.0, 55.0, 55.0, 55.0, 55.0,
    55.0, 55.0, 55.0, 55.0, 55.0, 55.0, 55.0,
    45.0, 65.0, 70.0, 160.0,
];
const GROUP_ROW_HEIGHT: f32 = 18.0;
const HEADER_ROW_HEIGHT: f32 = 18.0;
const DATA_ROW_HEIGHT: f32 = 20.0;

struct PaTableDelegate<'a> {
    entries: Vec<msp::PaCalibration>,
    frequencies: [u16; 7],
    checked: &'a mut HashMap<u8, bool>,
    sweep_active: bool,
    cal_status: Vec<[CellStatus; 7]>,
    det_status: Vec<[CellStatus; 7]>,
    level_status: Vec<Option<LevelStatus>>,
    limits: Vec<Option<i32>>,
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
    }

    fn header_cell_ui(&mut self, ui: &mut egui::Ui, cell_inf: &egui_table::HeaderCellInfo) {
        let egui_table::HeaderCellInfo { group_index, row_nr, .. } = cell_inf;
        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Truncate);
        match row_nr {
            0 => {
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
                ui.strong(self.column_label(*group_index));
            }
            _ => {}
        }
    }

    fn row_ui(&mut self, _ui: &mut egui::Ui, _row_nr: u64) {
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
        row_nr as f32 * self.row_height
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
        Some(LevelStatus::NotSettled) => "Not Settled".to_string(),
    }
}

fn min_max_or(points: &[[f64; 2]], default_lo: f64, default_hi: f64) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for &[_, y] in points {
        lo = lo.min(y);
        hi = hi.max(y);
    }
    if lo < hi {
        (lo, hi)
    } else {
        (default_lo, default_hi)
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

        let power_points: Vec<[f64; 2]> = state.power_history.iter().map(|&(t, mw)| [t, mw as f64]).collect();
        let temp_points_raw: Vec<[f64; 2]> = state.temp_history.iter().map(|&(t, c)| [t, c as f64]).collect();
        let mcu_temp_points_raw: Vec<[f64; 2]> = state.mcu_temp_history.iter().map(|&(t, c)| [t, c as f64]).collect();
        let latest_t = state.power_history.back().map(|&(t, _)| t).unwrap_or(0.0)
            .max(state.temp_history.back().map(|&(t, _)| t).unwrap_or(0.0))
            .max(state.mcu_temp_history.back().map(|&(t, _)| t).unwrap_or(0.0));

        let (power_lo, power_hi) = min_max_or(&power_points, 0.0, 1.0);
        let combined_temp_points: Vec<[f64; 2]> =
            temp_points_raw.iter().chain(mcu_temp_points_raw.iter()).copied().collect();
        let (temp_lo, temp_hi) = min_max_or(&combined_temp_points, 0.0, 1.0);
        let temp_to_power = move |t: f64| power_lo + (t - temp_lo) / (temp_hi - temp_lo) * (power_hi - power_lo);
        let power_to_temp = move |p: f64| temp_lo + (p - power_lo) / (power_hi - power_lo) * (temp_hi - temp_lo);
        let temp_points_scaled: PlotPoints =
            temp_points_raw.iter().map(|&[t, c]| [t, temp_to_power(c)]).collect();
        let mcu_temp_points_scaled: PlotPoints =
            mcu_temp_points_raw.iter().map(|&[t, c]| [t, temp_to_power(c)]).collect();
        let power_points: PlotPoints = power_points.into();

        let left_axis = AxisHints::new_y().label("mW").placement(HPlacement::Left);
        let right_axis = AxisHints::new_y()
            .label("°C")
            .placement(HPlacement::Right)
            .formatter(move |mark, _range| format!("{:.1}", power_to_temp(mark.value)));

        let mut plot = Plot::new("power_history_plot")
            .height(180.0)
            .include_x(latest_t - HISTORY_WINDOW_SECS)
            .include_x(latest_t)
            .x_axis_label("seconds")
            .custom_y_axes(vec![left_axis, right_axis])
            .legend(Legend::default());
        if page.plot_reset_requested {
            plot = plot.reset();
            page.plot_reset_requested = false;
        }
        let plot_response = plot.show(ui, |plot_ui| {
            plot_ui.line(Line::new("Power (mW)", power_points));
            if !temp_points_raw.is_empty() {
                plot_ui.line(Line::new("PA temp (°C)", temp_points_scaled).color(egui::Color32::YELLOW));
            }
            if !mcu_temp_points_raw.is_empty() {
                plot_ui.line(Line::new("MCU temp (°C)", mcu_temp_points_scaled).color(egui::Color32::ORANGE));
            }
        });
        plot_response.response.context_menu(|ui| {
            if ui.button("Reset").clicked() {
                page.plot_reset_requested = true;
            }
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
        let response = ui.add(egui::DragValue::new(&mut atten_db).suffix(" dB").range(-50.0..=100.0).speed(0.1));
        if response.changed() {
            shared.lock().unwrap().attenuation_db = atten_db;
            let mut settings = AppSettings::load();
            settings.attenuation_db = atten_db;
            let _ = settings.save();
        }
    });

    ui.separator();
    ui.heading("PA calibration table");

    let sweep_active = sweep.lock().unwrap().as_ref().map(|e| e.is_active()).unwrap_or(false);

    let awaiting_freq_mhz = {
        let g = sweep.lock().unwrap();
        g.as_ref().and_then(|e| match &e.state {
            calibration_engine::EngineState::AwaitingFreqConfirm { freq_mhz, .. } => Some(*freq_mhz),
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

    let connection_lost = {
        let g = sweep.lock().unwrap();
        g.as_ref().and_then(|e| match &e.state {
            calibration_engine::EngineState::ConnectionLost { level, freq_mhz, vbias_mv_at_loss, reason, .. } => {
                Some((*level, *freq_mhz, *vbias_mv_at_loss, *reason))
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
    let table_height = GROUP_ROW_HEIGHT + HEADER_ROW_HEIGHT + num_rows * DATA_ROW_HEIGHT + 4.0;
    ui.allocate_ui(egui::vec2(ui.available_width(), table_height), |ui| {
        egui_table::Table::new()
            .id_salt("pa_table")
            .auto_size_mode(AutoSizeMode::Always)
            .num_rows(delegate.entries.len() as u64)
            .columns(columns)
            .num_sticky_cols(0)
            .headers([
                egui_table::HeaderRow {
                    height: GROUP_ROW_HEIGHT,
                    groups: vec![0..3, 3..10, 10..17, 17..21],
                },
                egui_table::HeaderRow::new(HEADER_ROW_HEIGHT),
            ])
            .show(ui, &mut delegate);
    });

    if ui.button("Refresh").clicked() {
        let _ = cmd_tx.send(Command::RefreshCalTable);
    }

    ui.separator();

    {
        let g = sweep.lock().unwrap();
        let overall = g.as_ref().map(|engine| engine.progress()).unwrap_or(0.0);
        ui.add(egui::ProgressBar::new(overall).text("Overall").show_percentage());
    }

    ui.separator();

    let (automatic_mode, manual_mode, manual_dac_mv) = {
        let g = sweep.lock().unwrap();
        match g.as_ref() {
            Some(e) => (e.is_automatic_mode(), e.is_manual_mode(), e.manual_dac_mv),
            None => (false, false, 0),
        }
    };
    let pa_boost_on = {
        let s = shared.lock().unwrap();
        s.vtx_status.as_ref().and_then(|v| v.boost_on)
    };
    let any_checked = page.checked.values().any(|&v| v);
    let overall_ready = {
        let s = shared.lock().unwrap();
        conn_status::OverallState::from_ports(s.vtx_port_state, s.meter_port_state) == conn_status::OverallState::Ready
    };
    let debug = {
        let g = sweep.lock().unwrap();
        g.as_ref().map(|e| e.debug_state())
    };
    let (scan_phase, drop_active, fine_bound_mv, fine_highest_avg_mw, detector) = match debug {
        Some(d) => (d.scan_phase, d.drop_detector_active, d.fine_bound_mv, d.fine_highest_avg_mw, d.detector),
        None => ("Inactive", false, None, None, None),
    };

    const MAX_GROUP_HEIGHT: f32 = 400.0;
    let control_h_id = ui.id().with("cal_ctrl_content_h");
    let settings_h_id = ui.id().with("cal_settings_content_h");
    let debug_h_id = ui.id().with("cal_debug_content_h");
    let prev_control_h: f32 = ui.data(|d| d.get_temp(control_h_id)).unwrap_or(0.0);
    let prev_settings_h: f32 = ui.data(|d| d.get_temp(settings_h_id)).unwrap_or(0.0);
    let prev_debug_h: f32 = ui.data(|d| d.get_temp(debug_h_id)).unwrap_or(0.0);
    let target_h = prev_control_h.max(prev_settings_h).max(prev_debug_h).min(MAX_GROUP_HEIGHT);
    let mut new_heights = [0.0f32; 3];

    ui.columns(3, |columns| {
        columns[0].group(|ui| {
            ui.set_min_width(ui.available_width());
            let top = ui.cursor().top();
            ui.strong("Controls");
            egui::ScrollArea::horizontal().id_salt("control_scroll").show(ui, |ui| {
            egui::Grid::new("calibration_control_grid").num_columns(2).show(ui, |ui| {
                grid_label(ui, "Calibration");
                ui.horizontal(|ui| {
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
                        page.pending_skip_count += 1;
                        page.skip_debounce_until = Some(Instant::now() + SKIP_DEBOUNCE);
                    }
                    if ui.add_enabled(manual_mode, egui::Button::new("Save >")).clicked() {
                        let _ = cmd_tx.send(Command::ManualNext);
                    }
                });
                ui.end_row();

                if let Some(deadline) = page.skip_debounce_until {
                    let now = Instant::now();
                    if now >= deadline {
                        let count = page.pending_skip_count;
                        page.pending_skip_count = 0;
                        page.skip_debounce_until = None;
                        if count > 0 {
                            let _ = cmd_tx.send(Command::SkipMultiple { count });
                        }
                    } else {
                        ui.ctx().request_repaint_after(deadline - now);
                    }
                }

                if any_checked && !overall_ready {
                    grid_label(ui, "");
                    ui.label(
                        egui::RichText::new("Both VTX and power meter must be Ready to calibrate.")
                            .weak()
                            .italics(),
                    );
                    ui.end_row();
                }

                let mut current_mv = manual_dac_mv;
                let step = if page.fine_step { 1.0 } else { 25.0 };
                let message = if page.fine_step { "DAC mV (+/-1mv)" } else { "DAC mV (+/-25mV)" };
                grid_label(ui, message);
                let response = ui.add_enabled(
                    manual_mode,
                    egui::Slider::new(&mut current_mv, 0..=3300)
                        .clamping(SliderClamping::Never)
                        .drag_value_speed(0.1)
                        .step_by(step),
                );
                if response.changed() {
                    let _ = cmd_tx.send(Command::SetManualDac { mv: current_mv });
                }
                ui.end_row();

                grid_label(ui, "Fine");
                ui.add_enabled(manual_mode, egui::Checkbox::new(&mut page.fine_step, ""));
                ui.end_row();

                grid_label(ui, "PA");
                let mut pa_on = pa_boost_on.unwrap_or(false);
                if ui.add_enabled(manual_mode, egui::Checkbox::new(&mut pa_on, "")).changed() {
                    let _ = cmd_tx.send(Command::SetPaBoost { on: pa_on });
                }
                ui.end_row();
            });
                ui.separator();
                egui::Grid::new("calibration_storage_grid").num_columns(2).show(ui, |ui| {
                    grid_label(ui, "");
                    if ui.add_enabled(!sweep_active, egui::Button::new("Send to VTX")).clicked() {
                        let _ = cmd_tx.send(Command::SendCalTableToVtx);
                    }
                    ui.end_row();

                    grid_label(ui, "");
                    if ui.add_enabled(!sweep_active, egui::Button::new("Erase Calibration")).clicked() {
                        page.show_erase_confirm_dialog = true;
                    }
                    ui.end_row();
                });
            });
            let content_h = (ui.cursor().top() - top).min(MAX_GROUP_HEIGHT);
            new_heights[0] = content_h;
            let extra = (target_h - content_h).max(0.0).min(MAX_GROUP_HEIGHT);
            if extra > 0.0 {
                ui.add_space(extra);
            }
        });

        columns[1].group(|ui| {
            ui.set_min_width(ui.available_width());
            let top = ui.cursor().top();
            ui.strong("Settings");
            egui::ScrollArea::horizontal().id_salt("settings_scroll").show(ui, |ui| {
            egui::Grid::new("calibration_settings_grid").num_columns(2).show(ui, |ui| {
                grid_label(ui, "Scan detector tolerance");
                ui.add(
                    egui::DragValue::new(&mut page.tolerance_pct)
                        .range(0.1..=50.0)
                        .suffix("%")
                        .speed(0.1),
                );
                ui.end_row();
            });
            });
            let content_h = (ui.cursor().top() - top).min(MAX_GROUP_HEIGHT);
            new_heights[1] = content_h;
            let extra = (target_h - content_h).max(0.0).min(MAX_GROUP_HEIGHT);
            if extra > 0.0 {
                ui.add_space(extra);
            }
        });

        columns[2].group(|ui| {
            ui.set_min_width(ui.available_width());
            let top = ui.cursor().top();
            ui.strong("Debug");
            egui::ScrollArea::horizontal().id_salt("debug_scroll").show(ui, |ui| {
            egui::Grid::new("calibration_debug_grid").num_columns(2).show(ui, |ui| {
                grid_label(ui, "Scan Phase");
                ui.label(scan_phase);
                ui.end_row();

                grid_label(ui, "Drop detector");
                ui.label(if drop_active { "Active" } else { "Inactive" });
                ui.end_row();

                grid_label(ui, "fine_bound_mv");
                ui.label(match fine_bound_mv {
                    Some(v) => v.to_string(),
                    None => "None".to_string(),
                });
                ui.end_row();

                grid_label(ui, "fine_highest_average");
                ui.label(match fine_highest_avg_mw {
                    Some(v) => format!("{v:.4}mW"),
                    None => "None".to_string(),
                });
                ui.end_row();

                grid_label(ui, "Detector phase");
                ui.label(detector.as_ref().map(|d| d.phase).unwrap_or("None"));
                ui.end_row();

                grid_label(ui, "Detector below");
                ui.label(match detector.as_ref().and_then(|d| d.below) {
                    Some((mw, det_mv)) => format!("{mw:.4}mW / det={det_mv}"),
                    None => "None".to_string(),
                });
                ui.end_row();

                grid_label(ui, "Detector above");
                ui.label(match detector.as_ref().and_then(|d| d.above) {
                    Some((mw, det_mv)) => format!("{mw:.4}mW / det={det_mv}"),
                    None => "None".to_string(),
                });
                ui.end_row();

                grid_label(ui, "Pinned counter");
                ui.label(match &detector {
                    Some(d) => d.pinned_count.to_string(),
                    None => "None".to_string(),
                });
                ui.end_row();
            });
            });
            let content_h = (ui.cursor().top() - top).min(MAX_GROUP_HEIGHT);
            new_heights[2] = content_h;
            let extra = (target_h - content_h).max(0.0).min(MAX_GROUP_HEIGHT);
            if extra > 0.0 {
                ui.add_space(extra);
            }
        });
    });

    let new_target_h = new_heights.iter().cloned().fold(0.0f32, f32::max).min(MAX_GROUP_HEIGHT);
    ui.data_mut(|d| {
        d.insert_temp(control_h_id, new_heights[0]);
        d.insert_temp(settings_h_id, new_heights[1]);
        d.insert_temp(debug_h_id, new_heights[2]);
    });
    if (new_target_h - target_h).abs() > 0.5 {
        ui.ctx().request_repaint();
    }

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
            page.show_confirm_dialog = false;
        }
        if confirmed {
            page.show_confirm_dialog = false;
            let mut levels: Vec<u8> = page.checked.iter().filter(|&(_, &v)| v).map(|(&k, _)| k).collect();
            levels.sort_unstable();
            let _ = cmd_tx.send(Command::StartSweep { levels, tolerance_pct: page.tolerance_pct });
        }
    }

    if page.show_erase_confirm_dialog {
        let mut open = true;
        let mut confirmed = false;
        egui::Window::new("Confirm Erase Calibration")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .open(&mut open)
            .show(ui.ctx(), |ui| {
                ui.label("Reset EVERY level's calibration on the VTX back to its factory defaults?");
                ui.label("This writes to EEPROM immediately and cannot be undone.");
                ui.horizontal(|ui| {
                    if ui.button("Yes, erase").clicked() {
                        confirmed = true;
                    }
                    if ui.button("Cancel").clicked() {
                        page.show_erase_confirm_dialog = false;
                    }
                });
            });
        if !open {
            page.show_erase_confirm_dialog = false;
        }
        if confirmed {
            page.show_erase_confirm_dialog = false;
            let _ = cmd_tx.send(Command::EraseCalibration);
        }
    }
}
