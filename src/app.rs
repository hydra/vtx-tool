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

use crate::logging;
use crate::logging::SharedLogs;
use crate::pages;
use crate::pages::vtx_table::VtxTablePageState;
use crate::settings::AppSettings;
use crate::vtxtable::VtxTableConfig;
use crate::worker::{Command, SharedState};
use eframe::egui;
use eframe::Frame;
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
    cmd_tx: &'a Sender<Command>,
    vtx_table_page: &'a mut VtxTablePageState,
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
            Page::Calibration => pages::calibration::show(ui, self.shared, self.cmd_tx),
        }
    }
}

pub struct App {
    state: Arc<Mutex<SharedState>>,
    vtx_table: Arc<Mutex<VtxTableConfig>>,
    cmd_tx: Sender<Command>,
    logs: &'static SharedLogs,
    dock_state: DockState<Page>,
    vtx_table_page: VtxTablePageState,

    vtx_port_input: String,
    meter_port_input: String,
}

impl App {
    pub fn new(
        state: Arc<Mutex<SharedState>>,
        vtx_table: Arc<Mutex<VtxTableConfig>>,
        cmd_tx: Sender<Command>,
        logs: &'static SharedLogs,
        initial_settings: AppSettings,
    ) -> Self {
        Self {
            state,
            vtx_table,
            cmd_tx,
            logs,
            dock_state: DockState::new(vec![Page::Home]),
            vtx_table_page: VtxTablePageState::default(),
            vtx_port_input: initial_settings.vtx_port,
            meter_port_input: initial_settings.meter_port,
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
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut Frame) {
        // Bottom: VTX log / power meter log, split 50/50 with a divider.
        egui::Panel::bottom("logs_panel")
            .resizable(true)
            .default_size(220.0)
            .show_inside(ui, |ui| {
                ui.horizontal(|ui| {
                    let half_width = (ui.available_width() - 8.0) / 2.0;

                    ui.allocate_ui(egui::vec2(half_width, ui.available_height()), |ui| {
                        logging::show_panel(ui, "VTX log", &self.logs.vtx.lock().unwrap());
                    });

                    ui.add(egui::Separator::default().vertical());

                    ui.allocate_ui(egui::vec2(half_width, ui.available_height()), |ui| {
                        logging::show_panel(ui, "Power meter log", &self.logs.meter.lock().unwrap());
                    });
                });
            });

        // Left: page list + connection controls.
        egui::Panel::left("nav_panel")
            .resizable(false)
            .default_size(180.0)
            .show_inside(ui, |ui| {
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

                let connected = self.state.lock().unwrap().connected;

                ui.add_enabled(
                    !connected,
                    egui::TextEdit::singleline(&mut self.vtx_port_input).hint_text("VTX port"),
                );
                ui.add_enabled(
                    !connected,
                    egui::TextEdit::singleline(&mut self.meter_port_input).hint_text("Power meter port"),
                );

                if connected {
                    if ui.button("Disconnect").clicked() {
                        let _ = self.cmd_tx.send(Command::Disconnect);
                    }
                } else if ui.button("Connect").clicked() {
                    let settings = AppSettings {
                        vtx_port: self.vtx_port_input.clone(),
                        meter_port: self.meter_port_input.clone(),
                    };
                    let _ = settings.save(); // best-effort -- remembers ports for next launch regardless of whether the connect itself succeeds
                    let _ = self.cmd_tx.send(Command::Connect {
                        vtx_port: self.vtx_port_input.clone(),
                        meter_port: self.meter_port_input.clone(),
                    });
                }
            });

        // Center: whichever pages are open, as dock tabs.
        egui::CentralPanel::default().show_inside(ui, |ui| {
            let mut viewer = TabViewer {
                shared: &self.state,
                vtx_table: &self.vtx_table,
                cmd_tx: &self.cmd_tx,
                vtx_table_page: &mut self.vtx_table_page,
            };
            DockArea::new(&mut self.dock_state).show_inside(ui, &mut viewer);
        });
    }
}
