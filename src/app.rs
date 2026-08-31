
use crate::conn_status;
use crate::logging;
use crate::logging::SharedLogs;
use crate::pages;
use crate::pages::calibration::CalibrationPageState;
use crate::pages::vtx_table::VtxTablePageState;
use crate::power_meter::PowerMeterKind;
use crate::settings::AppSettings;
use crate::vtxtable::{VtxSelectionState, VtxTableConfig};
use crate::worker::{Command, SharedState, SharedSweep};
use eframe::egui;
use eframe::Frame;

const MIN_FREQ_KHZ: u32 = 5_600_000;
const MAX_FREQ_KHZ: u32 = 6_000_000;
use egui_dock::{DockArea, DockState};
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};

#[derive(PartialEq, Clone, Copy, Debug)]
pub enum Page {
    Home,
    VtxTable,
    Calibration,
}

impl Page {
    fn title(self) -> &'static str {
        match self {
            Page::Home => "Home",
            Page::VtxTable => "VTX Table",
            Page::Calibration => "Calibration",
        }
    }
}

struct TabViewer<'a> {
    shared: &'a Arc<Mutex<SharedState>>,
    vtx_table: &'a Arc<Mutex<VtxTableConfig>>,
    sweep: &'a SharedSweep,
    cmd_tx: &'a Sender<Command>,
    vtx_table_page: &'a mut VtxTablePageState,
    calibration_page: &'a mut CalibrationPageState,
}

impl egui_dock::TabViewer for TabViewer<'_> {
    type Tab = Page;

    fn title(&mut self, tab: &mut Page) -> egui::WidgetText {
        tab.title().into()
    }

    fn id(&mut self, tab: &mut Page) -> egui::Id {
        egui::Id::new(("rf-cal-page", *tab as u8))
    }

    fn ui(&mut self, ui: &mut egui::Ui, tab: &mut Page) {
        match tab {
            Page::Home => pages::home::show(ui),
            Page::VtxTable => pages::vtx_table::show(ui, self.shared, self.vtx_table, self.cmd_tx, self.vtx_table_page),
            Page::Calibration => {
                pages::calibration::show(ui, self.shared, self.sweep, self.cmd_tx, self.calibration_page)
            }
        }
    }
}

pub struct App {
    state: Arc<Mutex<SharedState>>,
    vtx_table: Arc<Mutex<VtxTableConfig>>,
    vtx_selection: Arc<Mutex<VtxSelectionState>>,
    sweep: SharedSweep,
    cmd_tx: Sender<Command>,
    logs: &'static SharedLogs,
    dock_state: DockState<Page>,
    vtx_table_page: VtxTablePageState,
    calibration_page: CalibrationPageState,

    vtx_port_input: String,
    meter_port_input: String,
    meter_kind: PowerMeterKind,
}

impl App {
    pub fn new(
        state: Arc<Mutex<SharedState>>,
        vtx_table: Arc<Mutex<VtxTableConfig>>,
        vtx_selection: Arc<Mutex<VtxSelectionState>>,
        sweep: SharedSweep,
        cmd_tx: Sender<Command>,
        logs: &'static SharedLogs,
        initial_settings: AppSettings,
        initial_meter_kind: PowerMeterKind,
    ) -> Self {
        Self {
            state,
            vtx_table,
            vtx_selection,
            sweep,
            cmd_tx,
            logs,
            dock_state: DockState::new(vec![Page::Home]),
            vtx_table_page: VtxTablePageState {
                file_path: initial_settings.vtx_table_path.clone(),
                ..VtxTablePageState::default()
            },
            calibration_page: CalibrationPageState::default(),
            vtx_port_input: initial_settings.vtx_port,
            meter_port_input: initial_settings.meter_port,
            meter_kind: initial_meter_kind,
        }
    }

    fn open_page(&mut self, page: Page) {
        if let Some(path) = self.dock_state.find_tab(&page) {
            self.dock_state.set_active_tab(path);
        } else {
            self.dock_state.push_to_focused_leaf(page);
            if page == Page::Calibration {
                let _ = self.cmd_tx.send(Command::RefreshCalTable);
            }
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut Frame) {
        egui::Panel::bottom("logs_panel")
            .resizable(true)
            .default_size(220.0)
            .min_size(80.0)
            .show(ui, |ui| {
                egui::Panel::left("vtx_log_panel")
                    .resizable(true)
                    .default_size(420.0)
                    .min_size(160.0)
                    .show_separator_line(true)
                    .show(ui, |ui| {
                        logging::show_panel(ui, "VTX log", &self.logs.vtx.lock().unwrap());
                    });

                logging::show_panel(ui, "Power meter log", &self.logs.meter.lock().unwrap());
            });

        egui::Panel::left("nav_panel")
            .resizable(true)
            .default_size(250.0)
            .min_size(120.0)
            .max_size(500.0)
            .show_separator_line(true)
            .show(ui, |ui| {
                egui::ScrollArea::both().auto_shrink([false, false]).show(ui, |ui| {
                ui.heading("Pages");
                ui.separator();
                if ui.button("Home").clicked() {
                    self.open_page(Page::Home);
                }
                if ui.button("VTX Table").clicked() {
                    self.open_page(Page::VtxTable);
                }
                if ui.button("Calibration").clicked() {
                    self.open_page(Page::Calibration);
                }

                ui.separator();
                ui.heading("Connection");

                let (vtx_state, meter_state) = {
                    let s = self.state.lock().unwrap();
                    (s.vtx_port_state, s.meter_port_state)
                };

                ui.horizontal(|ui| {
                    ui.label("VTX");
                    conn_status::show_port(ui, vtx_state);
                });
                ui.horizontal(|ui| {
                    ui.add_enabled(
                        vtx_state.is_idle(),
                        egui::TextEdit::singleline(&mut self.vtx_port_input)
                            .desired_width(100.0)
                            .hint_text("VTX port"),
                    );
                    let label = if vtx_state.is_ready() { "Disconnect" } else { "Connect" };
                    if ui.small_button(label).clicked() {
                        if vtx_state.is_ready() {
                            let _ = self.cmd_tx.send(Command::DisconnectVtx);
                        } else if vtx_state.is_idle() {
                            let mut settings = AppSettings::load();
                            settings.vtx_port = self.vtx_port_input.clone();
                            let _ = settings.save();
                            let _ = self.cmd_tx.send(Command::ConnectVtx { port: self.vtx_port_input.clone() });
                        }
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Power Meter");
                    conn_status::show_port(ui, meter_state);
                });
                ui.horizontal(|ui| {
                    ui.add_enabled(
                        meter_state.is_idle(),
                        egui::TextEdit::singleline(&mut self.meter_port_input)
                            .desired_width(100.0)
                            .hint_text("Power meter port"),
                    );
                    let label = if meter_state.is_ready() { "Disconnect" } else { "Connect" };
                    if ui.small_button(label).clicked() {
                        if meter_state.is_ready() {
                            let _ = self.cmd_tx.send(Command::DisconnectMeter);
                        } else if meter_state.is_idle() {
                            let mut settings = AppSettings::load();
                            settings.meter_port = self.meter_port_input.clone();
                            let _ = settings.save();
                            let _ = self.cmd_tx.send(Command::ConnectMeter {
                                port: self.meter_port_input.clone(),
                                meter_kind: self.meter_kind,
                            });
                        }
                    }
                });

                ui.add_enabled_ui(meter_state.is_idle(), |ui| {
                    egui::ComboBox::from_id_salt("meter_kind")
                        .selected_text(self.meter_kind.name())
                        .show_ui(ui, |ui| {
                            for &kind in PowerMeterKind::ALL {
                                if ui
                                    .selectable_value(&mut self.meter_kind, kind, kind.name())
                                    .changed()
                                {
                                    let mut s = self.state.lock().unwrap();
                                    s.meter_kind = kind;
                                    s.update_hz = s.update_hz.min(kind.max_update_hz() as f64);
                                    drop(s);
                                    let mut settings = AppSettings::load();
                                    settings.meter_kind = kind;
                                    let _ = settings.save();
                                }
                            }
                        });
                });

                ui.separator();
                let overall = conn_status::OverallState::from_ports(vtx_state, meter_state);
                ui.horizontal(|ui| {
                    ui.label("Overall:");
                    conn_status::show_overall(ui, overall);
                });

                let all_label = if overall == conn_status::OverallState::Ready { "Disconnect-all" } else { "Connect-all" };
                if ui.button(all_label).clicked() {
                    if overall == conn_status::OverallState::Ready {
                        let _ = self.cmd_tx.send(Command::DisconnectVtx);
                        let _ = self.cmd_tx.send(Command::DisconnectMeter);
                    } else {
                        if vtx_state.is_idle() {
                            let mut settings = AppSettings::load();
                            settings.vtx_port = self.vtx_port_input.clone();
                            let _ = settings.save();
                            let _ = self.cmd_tx.send(Command::ConnectVtx { port: self.vtx_port_input.clone() });
                        }
                        if meter_state.is_idle() {
                            let mut settings = AppSettings::load();
                            settings.meter_port = self.meter_port_input.clone();
                            let _ = settings.save();
                            let _ = self.cmd_tx.send(Command::ConnectMeter {
                                port: self.meter_port_input.clone(),
                                meter_kind: self.meter_kind,
                            });
                        }
                    }
                }

                ui.separator();
                ui.heading("Frequency");
                {
                    let table = self.vtx_table.lock().unwrap();
                    let mut sel = self.vtx_selection.lock().unwrap();

                    ui.checkbox(&mut sel.pitmode, "Pit mode");

                    let mut manual_mode = sel.selected_band == 0;
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut manual_mode, false, "Band/Channel");
                        ui.selectable_value(&mut manual_mode, true, "Manual");
                    });

                    if manual_mode {
                        sel.selected_band = 0;
                        let mut freq_khz = sel.selected_freq_mhz as u32 * 1000;
                        if ui
                            .add(
                                egui::DragValue::new(&mut freq_khz)
                                    .range(MIN_FREQ_KHZ..=MAX_FREQ_KHZ)
                                    .suffix(" kHz")
                                    .speed(1000),
                            )
                            .changed()
                        {
                            sel.selected_freq_mhz = (freq_khz / 1000) as u16;
                        }
                        ui.label("Frequency");
                    } else {
                        if sel.selected_band == 0 {
                            sel.selected_band = 1;
                        }
                        let band_indices: Vec<u8> = table.bands.iter().map(|b| b.index).collect();
                        egui::ComboBox::from_label("Band")
                            .selected_text(
                                table
                                    .bands
                                    .iter()
                                    .find(|b| b.index == sel.selected_band)
                                    .map(|b| format!("{} ({})", b.name, b.letter))
                                    .unwrap_or_else(|| "-".to_string()),
                            )
                            .show_ui(ui, |ui| {
                                for idx in band_indices {
                                    if let Some(b) = table.bands.iter().find(|b| b.index == idx) {
                                        let label = format!("{} ({})", b.name, b.letter);
                                        ui.selectable_value(&mut sel.selected_band, idx, label);
                                    }
                                }
                            });

                        let chan_count = table
                            .bands
                            .iter()
                            .find(|b| b.index == sel.selected_band)
                            .map(|b| b.channel_count.max(1))
                            .unwrap_or(1);
                        ui.add(egui::Slider::new(&mut sel.selected_channel, 1..=chan_count).text("Channel"));

                        let freq = sel.frequency_mhz(&table);
                        ui.label(format!("-> {freq} MHz"));
                    }

                    let power_count = table.power_levels.len().max(1) as u8;
                    ui.add(egui::Slider::new(&mut sel.selected_power, 1..=power_count).text("Power level"));
                    if let Some(p) = table.power_levels.iter().find(|p| p.index == sel.selected_power) {
                        ui.label(format!("-> {} mW ('{}')", p.m_w, p.label));
                    }

                    if ui.button("Save").clicked() {
                        let _ = self.cmd_tx.send(Command::PushVtxConfig);
                    }
                }

                ui.separator();
                ui.heading("VTX Status");
                let (vtx_status, vtx_last_seen_at, osd_canvas, osd_keepalive_at) = {
                    let s = self.state.lock().unwrap();
                    (s.vtx_status.clone(), s.vtx_last_seen_at.clone(), s.osd_canvas, s.osd_keepalive_at.clone())
                };
                match vtx_status {
                    Some(status) => {
                        egui::Grid::new("vtx_status_grid").num_columns(2).show(ui, |ui| {
                            ui.label("Level:");
                            ui.label(status.level.to_string());
                            ui.end_row();

                            ui.label("Power:");
                            ui.label(status.power_mw.map(|mw| format!("{mw} mW")).unwrap_or_else(|| "—".to_string()));
                            ui.end_row();

                            ui.label("PA:");
                            ui.label(match status.boost_on {
                                Some(true) => "ON",
                                Some(false) => "OFF",
                                None => "—",
                            });
                            ui.end_row();

                            ui.label("RTC6705 level:");
                            ui.label(match status.rtc6705_level {
                                Some(0) => "3 dBm".to_string(),
                                Some(1) => "7 dBm".to_string(),
                                Some(2) => "11 dBm".to_string(),
                                Some(3) => "13 dBm".to_string(),
                                Some(other) => format!("? ({other})"),
                                None => "—".to_string(),
                            });
                            ui.end_row();

                            ui.label("Frequency:");
                            ui.label(status.frequency_mhz.map(|f| format!("{f} MHz")).unwrap_or_else(|| "—".to_string()));
                            ui.end_row();

                            ui.label("Vbias:");
                            ui.label(format!("{} mV", status.vbias_mv));
                            ui.end_row();

                            ui.label("Vdetector:");
                            ui.label(format!("{} mV", status.detector_mv));
                            ui.end_row();

                            ui.label("PID loop:");
                            ui.label(match status.pid_active {
                                Some(true) => "Active",
                                Some(false) => "Inactive",
                                None => "—",
                            });
                            ui.end_row();

                            ui.label("Calibration session:");
                            ui.label(match status.session_active {
                                Some(true) => "Open",
                                Some(false) => "Closed",
                                None => "—",
                            });
                            ui.end_row();

                            ui.label("MCU temperature:");
                            ui.label(status.mcu_temp_c.map(|c| format!("{c:.1} °C")).unwrap_or_else(|| "—".to_string()));
                            ui.end_row();

                            ui.label("PA NTC temperature:");
                            ui.label(status.pa_temp_c.map(|c| format!("{c:.1} °C")).unwrap_or_else(|| "—".to_string()));
                            ui.end_row();

                            ui.label("PA NTC ADC:");
                            ui.label(status.ntc_raw.map(|raw| raw.to_string()).unwrap_or_else(|| "—".to_string()));
                            ui.end_row();

                            ui.label("Last seen:");
                            ui.label(vtx_last_seen_at.as_deref().unwrap_or("—"));
                            ui.end_row();
                        });
                    }
                    None => {
                        ui.label(egui::RichText::new("No status received yet.").weak().italics());
                    }
                }

                ui.separator();
                ui.heading("OSD Status");
                egui::Grid::new("osd_status_grid").num_columns(2).show(ui, |ui| {
                    ui.label("Size:");
                    ui.label(match osd_canvas {
                        Some((cols, rows)) => format!("{cols}x{rows}"),
                        None => "None".to_string(),
                    });
                    ui.end_row();

                    ui.label("Status:");
                    ui.label(match osd_keepalive_at {
                        Some(ts) => format!("Keepalive ({ts})"),
                        None => "None".to_string(),
                    });
                    ui.end_row();
                });

                let mut osd_debug_overlay_enabled = self.state.lock().unwrap().osd_debug_overlay_enabled;
                if ui.checkbox(&mut osd_debug_overlay_enabled, "Enable debug overlay").changed() {
                    self.state.lock().unwrap().osd_debug_overlay_enabled = osd_debug_overlay_enabled;
                }
                });
            });

        egui::CentralPanel::default().show(ui, |ui| {
            let mut viewer = TabViewer {
                shared: &self.state,
                vtx_table: &self.vtx_table,
                sweep: &self.sweep,
                cmd_tx: &self.cmd_tx,
                vtx_table_page: &mut self.vtx_table_page,
                calibration_page: &mut self.calibration_page,
            };
            DockArea::new(&mut self.dock_state).show_inside(ui, &mut viewer);
        });
    }
}
