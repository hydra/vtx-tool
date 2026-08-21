//! VTX Table page. This is now purely a LOCAL editor for the table the
//! app hands back to the VTX when it asks (see worker.rs's passive
//! responder and vtxtable.rs) -- there's no MSP push/pull to the VTX
//! here anymore, since Betaflight's real protocol never has the FC pull
//! a table FROM a VTX (confirmed against betaflight/betaflight's actual
//! msp.c and PR #11705). Three ways to populate the table: hand-edit
//! below, load a saved JSON file, or import Betaflight CLI `vtxtable`
//! lines.

use crate::msp::{VtxBand, VtxPowerLevel};
use crate::settings::AppSettings;
use crate::vtxtable::VtxTableConfig;
use crate::worker::{Command, SharedState};
use eframe::egui;
use std::path::Path;
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};

const MAX_CHANNELS_PER_BAND: usize = 8; // fixed by the wire format (freq[8] in VtxBand/SET_VTXTABLE_BAND)

/// UI-thread-owned scratch state that doesn't belong in the persisted
/// VtxTableConfig itself (file path text field, CLI paste buffer, last
/// error).
#[derive(Default)]
pub struct VtxTablePageState {
    pub file_path: String,
    pub cli_text: String,
    pub last_error: Option<String>,
    /// Set when Save is clicked and the target file already exists --
    /// renders the overwrite-confirm dialog instead of saving
    /// immediately. Cleared (without saving) if the user cancels or
    /// closes the dialog.
    pub confirm_overwrite: bool,
    /// Bands removed by lowering "Number of bands", most-recently-
    /// removed last -- restored (in reverse, i.e. most-recent-first)
    /// the next time the count is raised again, so briefly lowering
    /// then raising the count never loses what was actually typed in.
    /// Cleared on Load/Import, since restoring bands from a previous,
    /// unrelated table into a freshly loaded one wouldn't make sense.
    pub removed_bands: Vec<VtxBand>,
    /// Same idea as removed_bands, for power levels.
    pub removed_power_levels: Vec<VtxPowerLevel>,
}

fn resize_bands(cfg: &mut VtxTableConfig, target: usize, removed: &mut Vec<VtxBand>) {
    while cfg.bands.len() > target {
        if let Some(b) = cfg.bands.pop() {
            removed.push(b);
        }
    }
    while cfg.bands.len() < target {
        let band = removed.pop().unwrap_or_else(|| {
            let idx = cfg.bands.len() as u8 + 1;
            VtxBand {
                index: idx,
                name: format!("BAND{idx}"),
                letter: char::from(b'A' + (idx - 1).min(25)),
                is_factory: false,
                channel_count: cfg.channels,
                freqs_mhz: [5800; 8],
            }
        });
        cfg.bands.push(band);
    }
    for (i, b) in cfg.bands.iter_mut().enumerate() {
        b.index = i as u8 + 1;
    }
}

fn resize_power_levels(cfg: &mut VtxTableConfig, target: usize, removed: &mut Vec<VtxPowerLevel>) {
    while cfg.power_levels.len() > target {
        if let Some(p) = cfg.power_levels.pop() {
            removed.push(p);
        }
    }
    while cfg.power_levels.len() < target {
        let level = removed.pop().unwrap_or_else(|| {
            let idx = cfg.power_levels.len() as u8 + 1;
            VtxPowerLevel {
                index: idx,
                m_w: 25,
                label: "25".to_string(),
            }
        });
        cfg.power_levels.push(level);
    }
    for (i, p) in cfg.power_levels.iter_mut().enumerate() {
        p.index = i as u8 + 1;
    }
}

pub fn show(
    ui: &mut egui::Ui,
    shared: &Arc<Mutex<SharedState>>,
    vtx_table: &Arc<Mutex<VtxTableConfig>>,
    cmd_tx: &Sender<Command>,
    page: &mut VtxTablePageState,
) {
    let mut cfg = vtx_table.lock().unwrap();

    ui.heading("VTX Table");

    // ---- File I/O + CLI import -------------------------------------
    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.label("File:");
            ui.text_edit_singleline(&mut page.file_path);
            if ui.button("Load").clicked() {
                match VtxTableConfig::load_from_file(Path::new(&page.file_path)) {
                    Ok(loaded) => {
                        *cfg = loaded;
                        page.last_error = None;
                        page.removed_bands.clear();
                        page.removed_power_levels.clear();
                        let mut settings = AppSettings::load();
                        settings.vtx_table_path = page.file_path.clone();
                        let _ = settings.save(); // best-effort -- same pattern as the port fields
                    }
                    Err(e) => page.last_error = Some(format!("load failed: {e}")),
                }
            }
            if ui.button("Save").clicked() {
                if Path::new(&page.file_path).exists() {
                    page.confirm_overwrite = true;
                } else {
                    match cfg.save_to_file(Path::new(&page.file_path)) {
                        Ok(()) => {
                            page.last_error = None;
                            let mut settings = AppSettings::load();
                            settings.vtx_table_path = page.file_path.clone();
                            let _ = settings.save();
                        }
                        Err(e) => page.last_error = Some(format!("save failed: {e}")),
                    }
                }
            }
        });

        ui.collapsing("Import from Betaflight CLI", |ui| {
            ui.label("Paste 'vtxtable ...' lines (e.g. from Betaflight's CLI 'diff' output):");
            ui.add(egui::TextEdit::multiline(&mut page.cli_text).desired_rows(6));
            if ui.button("Import").clicked() {
                match VtxTableConfig::parse_betaflight_cli(&page.cli_text) {
                    Ok(parsed) => {
                        *cfg = parsed;
                        page.last_error = None;
                        page.removed_bands.clear();
                        page.removed_power_levels.clear();
                    }
                    Err(e) => page.last_error = Some(format!("import failed: {e}")),
                }
            }
        });

        if let Some(err) = &page.last_error {
            ui.colored_label(egui::Color32::RED, err);
        }
    });

    if page.confirm_overwrite {
        let mut open = true;
        let mut confirmed = false;
        let mut cancelled = false;
        egui::Window::new("Overwrite existing file?")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .open(&mut open)
            .show(ui.ctx(), |ui| {
                ui.label(format!("'{}' already exists. Overwrite it?", page.file_path));
                ui.horizontal(|ui| {
                    if ui.button("Overwrite").clicked() {
                        confirmed = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancelled = true;
                    }
                });
            });
        if !open {
            cancelled = true; // closed via the window's own X -- default is Cancel, per spec
        }
        if confirmed {
            page.confirm_overwrite = false;
            match cfg.save_to_file(Path::new(&page.file_path)) {
                Ok(()) => {
                    page.last_error = None;
                    let mut settings = AppSettings::load();
                    settings.vtx_table_path = page.file_path.clone();
                    let _ = settings.save();
                }
                Err(e) => page.last_error = Some(format!("save failed: {e}")),
            }
        } else if cancelled {
            page.confirm_overwrite = false;
        }
    }

    ui.separator();

    // ---- Bands table --------------------------------------------------
    ui.group(|ui| {
        ui.strong("Bands");

        ui.horizontal(|ui| {
            let mut num_bands = cfg.bands.len() as u8;
            if ui.add(egui::DragValue::new(&mut num_bands).range(0..=20)).changed() {
                resize_bands(&mut cfg, num_bands as usize, &mut page.removed_bands);
            }
            ui.label("Number of bands");

            ui.add_space(20.0);

            let mut chans = cfg.channels;
            if ui
                .add(egui::DragValue::new(&mut chans).range(1..=MAX_CHANNELS_PER_BAND as u8))
                .changed()
            {
                cfg.channels = chans;
                for b in &mut cfg.bands {
                    b.channel_count = chans;
                }
            }
            ui.label("Number of channels by band");
        });

        let channels = cfg.channels;
        egui::Grid::new("vtx_band_grid").striped(true).show(ui, |ui| {
            ui.strong("Band");
            for ch in 1..=channels {
                ui.strong(format!("{ch}"));
            }
            ui.end_row();

            for band in &mut cfg.bands {
                ui.horizontal(|ui| {
                    ui.label(format!("{}", band.index));
                    ui.add(egui::TextEdit::singleline(&mut band.name).desired_width(60.0));
                    ui.label(format!("({})", band.letter));
                });
                for ch in 0..channels as usize {
                    ui.add(egui::DragValue::new(&mut band.freqs_mhz[ch]).range(5000..=6000).suffix(" MHz"));
                }
                ui.end_row();
            }
        });
    });

    ui.separator();

    // ---- Power levels table --------------------------------------------
    ui.group(|ui| {
        ui.strong("Power levels");

        let mut num_levels = cfg.power_levels.len() as u8;
        if ui.add(egui::DragValue::new(&mut num_levels).range(0..=20)).changed() {
            resize_power_levels(&mut cfg, num_levels as usize, &mut page.removed_power_levels);
        }
        ui.label("Number of power levels");

        egui::Grid::new("vtx_power_grid").striped(true).show(ui, |ui| {
            ui.strong("");
            for pl in &cfg.power_levels {
                ui.strong(format!("{}", pl.index));
            }
            ui.end_row();

            ui.label("Value");
            for pl in &mut cfg.power_levels {
                ui.add(egui::DragValue::new(&mut pl.m_w).range(0..=5000).suffix(" mW"));
            }
            ui.end_row();

            ui.label("Label");
            for pl in &mut cfg.power_levels {
                ui.add(egui::TextEdit::singleline(&mut pl.label).desired_width(50.0));
            }
            ui.end_row();
        });
    });

    ui.separator();

    // ---- Debug: query what the VTX itself currently reports -----------
    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.strong("Debug: query VTX's own reported config");
            if ui.button("Query VTX").clicked() {
                let _ = cmd_tx.send(Command::RefreshVtxConfig);
            }
        });
        let s = shared.lock().unwrap();
        match &s.vtx_config {
            Some(c) => ui.label(format!(
                "band={} channel={} freq={} power={} pit={}",
                c.band, c.channel, c.frequency_mhz, c.power, c.pitmode
            )),
            None => ui.label("(not queried yet)"),
        };
    });
}
