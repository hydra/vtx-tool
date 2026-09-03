
use crate::msp::{VtxBand, VtxPowerLevel};
use crate::settings::AppSettings;
use crate::state::SharedHandles;
use crate::vtxtable::VtxTableConfig;
use crate::worker::Command;
use eframe::egui;
use std::path::Path;
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};

const MAX_CHANNELS_PER_BAND: usize = 8;

#[derive(Default)]
pub struct VtxTablePageState {
    pub file_path: String,
    pub cli_text: String,
    pub last_error: Option<String>,
    pub confirm_overwrite: bool,
    pub removed_bands: Vec<VtxBand>,
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
    shared: &SharedHandles,
    vtx_table: &Arc<Mutex<VtxTableConfig>>,
    cmd_tx: &Sender<Command>,
    page: &mut VtxTablePageState,
) {
    let mut cfg = vtx_table.lock().unwrap();

    ui.heading("VTX Table");

    {
        let mut sync = shared.table_sync.lock().unwrap();
        ui.horizontal(|ui| {
            ui.checkbox(&mut sync.ready, "Ready");
            ui.separator();
            ui.label("Synchronized:");
            ui.colored_label(
                if sync.synchronized { egui::Color32::GREEN } else { egui::Color32::RED },
                if sync.synchronized { "True" } else { "False" },
            );
        });
    }

    let mut modified = false;

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
                        modified = true;
                        let mut settings = AppSettings::load();
                        settings.vtx_table_path = page.file_path.clone();
                        let _ = settings.save();
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
                        modified = true;
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
            cancelled = true;
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

    ui.group(|ui| {
        ui.strong("Bands");

        ui.horizontal(|ui| {
            let mut num_bands = cfg.bands.len() as u8;
            if ui.add(egui::DragValue::new(&mut num_bands).range(0..=20)).changed() {
                resize_bands(&mut cfg, num_bands as usize, &mut page.removed_bands);
                modified = true;
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
                modified = true;
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
                    modified |= ui.add(egui::TextEdit::singleline(&mut band.name).desired_width(60.0)).changed();
                    ui.label(format!("({})", band.letter));
                });
                for ch in 0..channels as usize {
                    modified |= ui.add(egui::DragValue::new(&mut band.freqs_mhz[ch]).range(5000..=6000).suffix(" MHz")).changed();
                }
                ui.end_row();
            }
        });
    });

    ui.separator();

    ui.group(|ui| {
        ui.strong("Power levels");

        let mut num_levels = cfg.power_levels.len() as u8;
        if ui.add(egui::DragValue::new(&mut num_levels).range(0..=20)).changed() {
            resize_power_levels(&mut cfg, num_levels as usize, &mut page.removed_power_levels);
            modified = true;
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
                modified |= ui.add(egui::DragValue::new(&mut pl.m_w).range(0..=5000).suffix(" mW")).changed();
            }
            ui.end_row();

            ui.label("Label");
            for pl in &mut cfg.power_levels {
                modified |= ui.add(egui::TextEdit::singleline(&mut pl.label).desired_width(50.0)).changed();
            }
            ui.end_row();
        });
    });

    if modified {
        let mut sync = shared.table_sync.lock().unwrap();
        sync.ready = false;
        sync.synchronized = false;
    }

    ui.separator();

    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.strong("Debug: query VTX's own reported config");
            if ui.button("Query VTX").clicked() {
                let _ = cmd_tx.send(Command::RefreshVtxConfig);
            }
        });
        let vtx = shared.vtx.lock().unwrap();
        match &vtx.config {
            Some(c) => ui.label(format!(
                "band={} channel={} freq={} power={} pit={}",
                c.band, c.channel, c.frequency_mhz, c.power, c.pitmode
            )),
            None => ui.label("(not queried yet)"),
        };
    });
}
