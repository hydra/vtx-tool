//! App shell: left nav panel (page list + connection controls) + center
//! egui_dock tab area + bottom panel split 50/50 between the VTX and
//! power-meter logs.
//!
//! Pinned to eframe/egui 0.36.1 + egui_dock 0.21.1 (confirmed as a real,
//! resolving combination from a build log) -- NOT 0.34.0 as originally
//! specified. egui_dock 0.21.1 depends on egui 0.36.1 internally; keeping
//! our own eframe/egui at 0.34.0 put two different copies of the egui
//! crate in the dependency graph, which caused the earlier Ui/WidgetText
//! type-mismatch errors.
//!
//! Connection lifecycle: ports are opened/closed on demand via
//! Command::Connect/Disconnect (see worker.rs), not automatically at
//! startup unless both were given on the command line (see main.rs).
//! While connected, the port fields here are shown but disabled -- the
//! worker owns the actual open ports and there's no live "change port"
//! operation, only disconnect-then-reconnect.

use crate::conn_status;
use crate::logging;
use crate::logging::SharedLogs;
use crate::pages;
use crate::pages::calibration::CalibrationPageState;
use crate::pages::vtx_table::VtxTablePageState;
use crate::power_meter::PowerMeterKind;
use crate::settings::AppSettings;
use crate::vtxtable::VtxTableConfig;
use crate::worker::{Command, SharedState, SharedSweep};
use eframe::egui;
use eframe::Frame;

/// RTC6705's usable range, as enforced by vtx_msp.c's freq_is_in_58ghz()
/// -- moved here from pages/vtx_table.rs alongside the Frequency section
/// itself.
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
        sweep: SharedSweep,
        cmd_tx: Sender<Command>,
        logs: &'static SharedLogs,
        initial_settings: AppSettings,
        initial_meter_kind: PowerMeterKind,
    ) -> Self {
        Self {
            state,
            vtx_table,
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

    /// Opens `page` as a new tab, or focuses/activates it if already open.
    fn open_page(&mut self, page: Page) {
        // find_tab() returns Option<TabPath> in this egui_dock version
        // (a named struct, not a raw tuple). Passing it straight into
        // set_active_tab() on the theory the API is symmetric.
        if let Some(path) = self.dock_state.find_tab(&page) {
            self.dock_state.set_active_tab(path);
        } else {
            self.dock_state.push_to_focused_leaf(page);
            // Runs once, exactly when the tab is first created (not on
            // every subsequent focus) -- the show() function itself
            // gets called every frame the tab is visible, so triggering
            // this there instead would mean re-sending the refresh
            // constantly rather than just on first open.
            if page == Page::Calibration {
                let _ = self.cmd_tx.send(Command::RefreshCalTable);
            }
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut Frame) {
        // Bottom: VTX log / power meter log, split 50/50 with a divider.
        egui::Panel::bottom("logs_panel")
            .resizable(true)
            .default_size(220.0)
            .min_size(80.0)
            .show(ui, |ui| {
                // Nested resizable panel gives a real draggable divider
                // between the two logs (with its own persisted width).
                // Fixed default well clear of nav_panel's own default
                // width below: egui's Panel derives its available rect
                // from parent_ui.available_rect_before_wrap(), so a
                // panel shown later (nav_panel) correctly inherits the
                // space this one already claimed -- that part isn't
                // buggy. What was happening is two independent dividers
                // (this one, and nav_panel's right edge) landing at
                // nearly the same x-position and reading as one
                // continuous line running through both panels.
                egui::Panel::left("vtx_log_panel")
                    .resizable(true)
                    .default_size(420.0)
                    .min_size(160.0)
                    .show_separator_line(true)
                    .show(ui, |ui| {
                        logging::show_panel(ui, "VTX log", &self.logs.vtx.lock().unwrap());
                    });

                // Takes whatever width the vtx_log_panel above left behind.
                logging::show_panel(ui, "Power meter log", &self.logs.meter.lock().unwrap());
            });

        // Left: page list + connection controls.
        egui::Panel::left("nav_panel")
            .resizable(true)
            .default_size(250.0)
            .min_size(120.0)
            .max_size(500.0)
            .show_separator_line(true)
            .show(ui, |ui| {
                // Content here can outgrow the panel's available height
                // (e.g. once VTX Status/OSD Status are populated, or
                // when the bottom logs panel is dragged tall) -- and now
                // that the panel itself is resizable, width too (drag it
                // narrow enough and rows like the port fields no longer
                // fit). ScrollArea::both handles both without clipping.
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
                            let _ = settings.save(); // best-effort -- remembers the port for next launch regardless of whether the connect itself succeeds
                            let _ = self.cmd_tx.send(Command::ConnectVtx { port: self.vtx_port_input.clone() });
                        }
                        // Connecting/Disconnecting: button click ignored -- an
                        // operation's already in flight for this port.
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
                                    // Reflected immediately (even before Connect) so the
                                    // calibration page's update-rate dropdown can clamp
                                    // to this kind's max_update_hz() right away.
                                    let mut s = self.state.lock().unwrap();
                                    s.meter_kind = kind;
                                    s.update_hz = s.update_hz.min(kind.max_update_hz() as f64);
                                    drop(s);
                                    let mut settings = AppSettings::load();
                                    settings.meter_kind = kind;
                                    let _ = settings.save(); // best-effort -- same pattern as the port fields
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
                        // Per-port ConnectVtx/ConnectMeter handlers already no-op
                        // if that specific port is already open (see worker.rs),
                        // so it's safe to just request both unconditionally here
                        // rather than working out which ones actually need it.
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
                    let mut cfg = self.vtx_table.lock().unwrap();

                    ui.checkbox(&mut cfg.pitmode, "Pit mode");

                    let mut manual_mode = cfg.selected_band == 0;
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut manual_mode, false, "Band/Channel");
                        ui.selectable_value(&mut manual_mode, true, "Manual");
                    });

                    if manual_mode {
                        cfg.selected_band = 0;
                        let mut freq_khz = cfg.selected_freq_mhz as u32 * 1000;
                        if ui
                            .add(
                                egui::DragValue::new(&mut freq_khz)
                                    .range(MIN_FREQ_KHZ..=MAX_FREQ_KHZ)
                                    .suffix(" kHz")
                                    .speed(1000),
                            )
                            .changed()
                        {
                            cfg.selected_freq_mhz = (freq_khz / 1000) as u16;
                        }
                        ui.label("Frequency");
                    } else {
                        if cfg.selected_band == 0 {
                            cfg.selected_band = 1;
                        }
                        let band_indices: Vec<u8> = cfg.bands.iter().map(|b| b.index).collect();
                        egui::ComboBox::from_label("Band")
                            .selected_text(
                                cfg.bands
                                    .iter()
                                    .find(|b| b.index == cfg.selected_band)
                                    .map(|b| format!("{} ({})", b.name, b.letter))
                                    .unwrap_or_else(|| "-".to_string()),
                            )
                            .show_ui(ui, |ui| {
                                for idx in band_indices {
                                    if let Some(b) = cfg.bands.iter().find(|b| b.index == idx) {
                                        let label = format!("{} ({})", b.name, b.letter);
                                        ui.selectable_value(&mut cfg.selected_band, idx, label);
                                    }
                                }
                            });

                        let chan_count = cfg
                            .bands
                            .iter()
                            .find(|b| b.index == cfg.selected_band)
                            .map(|b| b.channel_count.max(1))
                            .unwrap_or(1);
                        ui.add(egui::Slider::new(&mut cfg.selected_channel, 1..=chan_count).text("Channel"));

                        let freq = cfg.selected_frequency_mhz();
                        ui.label(format!("-> {freq} MHz"));
                    }

                    let power_count = cfg.power_levels.len().max(1) as u8;
                    ui.add(egui::Slider::new(&mut cfg.selected_power, 1..=power_count).text("Power level"));
                    if let Some(p) = cfg.power_levels.iter().find(|p| p.index == cfg.selected_power) {
                        ui.label(format!("-> {} mW ('{}')", p.m_w, p.label));
                    }

                    if ui.button("Save").clicked() {
                        let _ = self.cmd_tx.send(Command::PushVtxConfig);
                    }
                }

                ui.separator();
                ui.heading("VTX Status");
                // Every value below is either read directly off the VTX's
                // own MSP_PACALIBRATION reply, or (Power) derived from two
                // VTX-reported facts (the reported level, looked up against
                // the mW column of the calibration table also read from the
                // VTX) -- see worker::VtxStatus's own doc comment. "—" means
                // no reading has arrived yet (or the connected firmware
                // doesn't send that particular field), never a guessed or
                // defaulted value that could be mistaken for real data.
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
                                // Fixed, known mapping (rtc6705.h's own
                                // rtc6705_power_t enum) -- not a guess, just a
                                // more readable form of the same raw value.
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

                            ui.label("PA NTC ADC:");
                            ui.label(status.ntc_raw.map(|raw| raw.to_string()).unwrap_or_else(|| "—".to_string()));
                            ui.end_row();

                            ui.label("PA temperature:");
                            ui.label(status.pa_temp_c.map(|c| format!("{c:.1} °C")).unwrap_or_else(|| "—".to_string()));
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
                // Both fields below come from what THIS tool has sent/
                // received over MSP_DISPLAYPORT -- see
                // build_status_displayport_frames()'s doc comment in
                // worker.rs for why this overlay exists (an MCU hang should
                // show up here as a screen that stops updating, independent
                // of whatever the serial link itself is or isn't reporting).
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

                // While unchecked, worker.rs sends NO MSP_DISPLAYPORT traffic
                // at all -- no keepalive, no clear, no draw_string/draw_screen
                // -- so the firmware's own OSD content (e.g. debug_pa_loop()'s
                // PID debug rows) can be observed with this tool's own
                // overlay entirely out of the picture.
                let mut osd_debug_overlay_enabled = self.state.lock().unwrap().osd_debug_overlay_enabled;
                if ui.checkbox(&mut osd_debug_overlay_enabled, "Enable debug overlay").changed() {
                    self.state.lock().unwrap().osd_debug_overlay_enabled = osd_debug_overlay_enabled;
                }
                }); // ScrollArea
            });

        // Center: whichever pages are open, as dock tabs.
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
