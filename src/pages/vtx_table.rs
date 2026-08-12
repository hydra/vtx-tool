//! VTX Table page. This is now purely a LOCAL editor for the table the
//! app hands back to the VTX when it asks (see worker.rs's passive
//! responder and vtxtable.rs) -- there's no MSP push/pull to the VTX
//! here anymore, since Betaflight's real protocol never has the FC pull
//! a table FROM a VTX (confirmed against betaflight/betaflight's actual
//! msp.c and PR #11705). Three ways to populate the table: hand-edit
//! below, load a saved JSON file, or import Betaflight CLI `vtxtable`
//! lines.

use crate::msp::{VtxBand, VtxPowerLevel};
use crate::vtxtable::VtxTableConfig;
use crate::worker::{Command, SharedState};
use eframe::egui;
use std::path::Path;
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};

const MAX_CHANNELS_PER_BAND: usize = 8; // fixed by the wire format (freq[8] in VtxBand/SET_VTXTABLE_BAND)
const MIN_FREQ_KHZ: u32 = 5_600_000; // RTC6705's usable range, as enforced by vtx_msp.c's freq_is_in_58ghz()
const MAX_FREQ_KHZ: u32 = 6_000_000;

/// UI-thread-owned scratch state that doesn't belong in the persisted
/// VtxTableConfig itself (file path text field, CLI paste buffer, last
/// error).
#[derive(Default)]
pub struct VtxTablePageState {
    pub file_path: String,
    pub cli_text: String,
    pub last_error: Option<String>,
}

fn resize_bands(cfg: &mut VtxTableConfig, target: usize) {
    while cfg.bands.len() < target {
        let idx = cfg.bands.len() as u8 + 1;
        cfg.bands.push(VtxBand {
            index: idx,
            name: format!("BAND{idx}"),
            letter: char::from(b'A' + (idx - 1).min(25)),
            is_factory: false,
            channel_count: cfg.channels,
            freqs_mhz: [5800; 8],
        });
    }
    cfg.bands.truncate(target);
    for (i, b) in cfg.bands.iter_mut().enumerate() {
        b.index = i as u8 + 1;
    }
}

fn resize_power_levels(cfg: &mut VtxTableConfig, target: usize) {
    while cfg.power_levels.len() < target {
        let idx = cfg.power_levels.len() as u8 + 1;
        cfg.power_levels.push(VtxPowerLevel {
            index: idx,
            m_w: 25,
            label: "25".to_string(),
        });
    }
    cfg.power_levels.truncate(target);
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
            if ui.button("Save").clicked() {
                match cfg.save_to_file(Path::new(&page.file_path)) {
                    Ok(()) => page.last_error = None,
                    Err(e) => page.last_error = Some(format!("save failed: {e}")),
                }
            }
            if ui.button("Load").clicked() {
                match VtxTableConfig::load_from_file(Path::new(&page.file_path)) {
                    Ok(loaded) => {
                        *cfg = loaded;
                        page.last_error = None;
                    }
                    Err(e) => page.last_error = Some(format!("load failed: {e}")),
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
                    }
                    Err(e) => page.last_error = Some(format!("import failed: {e}")),
                }
            }
        });

        if let Some(err) = &page.last_error {
            ui.colored_label(egui::Color32::RED, err);
        }
    });

    ui.separator();

    // ---- Selected: what this tool reports to the VTX when it asks ---
    ui.group(|ui| {
        ui.strong("Frequency");

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
            let _ = cmd_tx.send(Command::PushVtxConfig);
        }
    });

    ui.separator();

    // ---- Bands table --------------------------------------------------
    ui.group(|ui| {
        ui.strong("Bands");

        ui.horizontal(|ui| {
            let mut num_bands = cfg.bands.len() as u8;
            if ui.add(egui::DragValue::new(&mut num_bands).range(0..=20)).changed() {
                resize_bands(&mut cfg, num_bands as usize);
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
            resize_power_levels(&mut cfg, num_levels as usize);
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
