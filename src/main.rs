//! rf-cal: PA calibration + VTX table tool.
//!
//! Connection is manual by default (Connect button in the left panel);
//! auto-connects at startup only if ALL THREE of --vtx-port,
//! --meter-port, and --meter-kind are given on the command line (the
//! meter kind determines the serial protocol/baud used to open the
//! meter port, so it's just as required as the ports themselves for an
//! actually-working auto-connect, not optional). Last-used ports are
//! remembered (settings.rs) and pre-filled in the port fields regardless
//! of whether auto-connect fires.

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
use vtxtable::VtxTableConfig;

#[derive(Parser, Debug)]
#[command(name = "rf-cal", about = "RF PA calibration + VTX table tool")]
struct Args {
    /// Serial port for the VTX's MSP interface (e.g. /dev/ttyACM0, COM5).
    /// If omitted, the app starts disconnected -- use the Connect button.
    #[arg(long)]
    vtx_port: Option<String>,

    /// Serial port for the power meter. Auto-connect only fires if this,
    /// --vtx-port, AND --meter-kind are all given.
    #[arg(long)]
    meter_port: Option<String>,

    /// Power meter model, e.g. immersionrc-v1. Determines the serial
    /// protocol/baud used to talk to it -- required (alongside the two
    /// ports) for auto-connect, since there's no correct way to open the
    /// meter port without knowing this.
    ///
    /// Parsed via power_meter::parse_cli, a plain function, rather than
    /// clap's ValueEnum -- see power_meter.rs's PowerMeterKind::cli_name()
    /// doc comment for why. The field type is still the real
    /// PowerMeterKind enum either way, just not going through
    /// #[arg(value_enum)]'s trait-based codegen path.
    #[arg(long, value_parser = power_meter::parse_cli)]
    meter_kind: Option<PowerMeterKind>,

    /// Minimum log level to record (error, warn, info, debug, trace)
    #[arg(long, default_value = "debug")]
    log_level: LevelFilter,
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
    let initial_meter_kind = args.meter_kind.unwrap_or_default();

    let state = Arc::new(Mutex::new(worker::SharedState::default()));
    state.lock().unwrap().meter_kind = initial_meter_kind;
    let vtx_table = Arc::new(Mutex::new(VtxTableConfig::default()));
    let sweep: worker::SharedSweep = Arc::new(Mutex::new(None));
    let (cmd_tx, cmd_rx) = mpsc::channel();

    let auto_connect = args.vtx_port.is_some() && args.meter_port.is_some() && args.meter_kind.is_some();

    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "RF Calibration",
        native_options,
        Box::new(move |cc| {
            worker::spawn(state.clone(), vtx_table.clone(), sweep.clone(), cmd_rx, cc.egui_ctx.clone());

            if auto_connect {
                let _ = cmd_tx.send(worker::Command::Connect {
                    vtx_port: initial_settings.vtx_port.clone(),
                    meter_port: initial_settings.meter_port.clone(),
                    meter_kind: initial_meter_kind,
                });
            }

            Ok(Box::new(app::App::new(
                state.clone(),
                vtx_table.clone(),
                sweep.clone(),
                cmd_tx.clone(),
                logs,
                initial_settings,
                initial_meter_kind,
            )))
        }),
    )
}
