//! rf-cal: PA calibration + VTX table tool.
//!
//! Connection is manual by default (Connect button in the left panel);
//! auto-connects at startup only if BOTH --vtx-port and --meter-port are
//! given on the command line. Last-used ports are remembered (settings.rs)
//! and pre-filled in the port fields regardless.

mod app;
mod logging;
mod msp;
mod pages;
mod power_meter;
mod settings;
mod vtxtable;
mod worker;

use clap::Parser;
use log::LevelFilter;
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

    /// Serial port for the ImmersionRC power meter. Same rule as
    /// --vtx-port: auto-connect only fires if BOTH are given.
    #[arg(long)]
    meter_port: Option<String>,

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

    let state = Arc::new(Mutex::new(worker::SharedState::default()));
    let vtx_table = Arc::new(Mutex::new(VtxTableConfig::default()));
    let (cmd_tx, cmd_rx) = mpsc::channel();

    let auto_connect = args.vtx_port.is_some() && args.meter_port.is_some();

    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "RF Calibration",
        native_options,
        Box::new(move |cc| {
            worker::spawn(state.clone(), vtx_table.clone(), cmd_rx, cc.egui_ctx.clone());

            if auto_connect {
                let _ = cmd_tx.send(worker::Command::Connect {
                    vtx_port: initial_settings.vtx_port.clone(),
                    meter_port: initial_settings.meter_port.clone(),
                });
            }

            Ok(Box::new(app::App::new(
                state.clone(),
                vtx_table.clone(),
                cmd_tx.clone(),
                logs,
                initial_settings,
            )))
        }),
    )
}
