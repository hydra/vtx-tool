//! rf-cal: PA calibration tool.
//!
//! Read-only shell for now (see app.rs) -- live power meter readings,
//! on-demand PA calibration table reads, and per-port status/error logs.
//! The write path (sweep controls, EEPROM commit) is the next piece to
//! add now that vtx_msp.c has confirmed the wire semantics.

mod app;
mod logging;
mod msp;
mod power_meter;
mod worker;

use clap::Parser;
use log::LevelFilter;
use std::sync::mpsc;
use std::sync::{Arc, Mutex};

#[derive(Parser, Debug)]
#[command(name = "rf-cal", about = "RF PA calibration tool")]
struct Args {
    /// Serial port for the VTX's MSP interface (e.g. /dev/ttyACM0, COM5)
    #[arg(long)]
    vtx_port: String,

    /// Serial port for the ImmersionRC power meter
    #[arg(long)]
    meter_port: String,

    /// Minimum log level to record (error, warn, info, debug, trace)
    #[arg(long, default_value = "debug")]
    log_level: LevelFilter,
}

fn main() -> eframe::Result<()> {
    let args = Args::parse();

    // Install the log sink before anything else logs (the worker thread
    // in particular) -- see logging.rs. Errors go through log::error!,
    // successful command responses through log::debug!, both tagged
    // target="vtx" or target="meter" so app.rs can show them in the
    // right panel.
    let logs = logging::init(args.log_level);

    let state = Arc::new(Mutex::new(worker::SharedState::default()));
    let (cmd_tx, cmd_rx) = mpsc::channel();

    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "RF Calibration",
        native_options,
        Box::new(move |cc| {
            worker::spawn(
                args.vtx_port.clone(),
                args.meter_port.clone(),
                state.clone(),
                cmd_rx,
                cc.egui_ctx.clone(),
            );
            Ok(Box::new(app::App::new(state.clone(), cmd_tx.clone(), logs)))
        }),
    )
}
