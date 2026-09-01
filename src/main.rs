
mod app;
mod calibration_engine;
mod conn_status;
mod logging;
mod msp;
mod pages;
mod power_meter;
mod settings;
mod vtxtable;
mod worker;

use clap::Parser;
use log::LevelFilter;
use power_meter::PowerMeterKind;
use settings::AppSettings;
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use vtxtable::{VtxSelectionState, VtxTableConfig};

#[derive(Parser, Debug)]
#[command(name = "rf-cal", about = "RF PA calibration + VTX table tool")]
struct Args {
    #[arg(long)]
    vtx_port: Option<String>,

    #[arg(long)]
    meter_port: Option<String>,

    #[arg(long, value_parser = power_meter::parse_cli)]
    meter_kind: Option<PowerMeterKind>,

    #[arg(long)]
    attenuation: Option<f32>,

    #[arg(long, default_value = "debug")]
    log_level: LevelFilter,

    #[arg(long)]
    vtx_table: Option<String>,
}

fn main() -> eframe::Result<()> {
    let args = Args::parse();

    let logs = logging::init(args.log_level);

    let mut initial_settings = AppSettings::load();
    if let Some(p) = &args.vtx_port {
        initial_settings.vtx_port = p.clone();
    }
    if let Some(p) = &args.meter_port {
        initial_settings.meter_port = p.clone();
    }
    if let Some(k) = args.meter_kind {
        initial_settings.meter_kind = k;
    }
    if let Some(a) = args.attenuation {
        initial_settings.attenuation_db = a;
    }
    if let Some(t) = &args.vtx_table {
        initial_settings.vtx_table_path = t.clone();
    }
    let initial_meter_kind = initial_settings.meter_kind;

    let state = Arc::new(Mutex::new(worker::SharedState::default()));
    state.lock().unwrap().meter_kind = initial_meter_kind;
    state.lock().unwrap().attenuation_db = initial_settings.attenuation_db;
    let (initial_vtx_table, vtx_table_ready_at_startup) = if initial_settings.vtx_table_path.is_empty() {
        (VtxTableConfig::default(), false)
    } else {
        match VtxTableConfig::load_from_file(std::path::Path::new(&initial_settings.vtx_table_path)) {
            Ok(loaded) => {
                let ready = !loaded.bands.is_empty() && !loaded.power_levels.is_empty();
                (loaded, ready)
            }
            Err(_) => (VtxTableConfig::default(), false),
        }
    };
    state.lock().unwrap().vtx_table_ready = vtx_table_ready_at_startup;
    let vtx_table = Arc::new(Mutex::new(initial_vtx_table));
    let vtx_selection = Arc::new(Mutex::new(VtxSelectionState::default()));
    let sweep: worker::SharedSweep = Arc::new(Mutex::new(None));
    let (cmd_tx, cmd_rx) = mpsc::channel();

    let auto_connect = args.vtx_port.is_some() && args.meter_port.is_some() && args.meter_kind.is_some();

    let native_options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default().with_inner_size([1920.0, 1080.0]),
        ..Default::default()
    };
    eframe::run_native(
        "RF Calibration",
        native_options,
        Box::new(move |cc| {
            worker::spawn(state.clone(), vtx_table.clone(), vtx_selection.clone(), sweep.clone(), cmd_rx, cc.egui_ctx.clone());

            if auto_connect {
                let _ = cmd_tx.send(worker::Command::ConnectVtx { port: initial_settings.vtx_port.clone() });
                let _ = cmd_tx.send(worker::Command::ConnectMeter {
                    port: initial_settings.meter_port.clone(),
                    meter_kind: initial_meter_kind,
                });
            }

            cc.egui_ctx.all_styles_mut(|style| {
                style.visuals.indent_has_left_vline = false;
            });

            Ok(Box::new(app::App::new(
                state.clone(),
                vtx_table.clone(),
                vtx_selection.clone(),
                sweep.clone(),
                cmd_tx.clone(),
                logs,
                initial_settings,
                initial_meter_kind,
            )))
        }),
    )
}
