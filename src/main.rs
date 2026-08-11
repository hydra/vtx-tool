//! rf-cal: PA calibration tool.
//!
//! Currently a read-only shell (see app.rs) -- live power meter readings
//! and on-demand PA calibration table reads. The write path (sweep
//! controls, EEPROM commit) is held back pending confirmation of
//! MSP_SET_PACALIBRATION's exact semantics against the VTX-side C
//! handler (see msp.rs).

mod app;
mod msp;
mod power_meter;
mod worker;

use clap::Parser;
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
}

fn main() -> eframe::Result<()> {
    let args = Args::parse();

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
            Ok(Box::new(app::App::new(state.clone(), cmd_tx.clone())))
        }),
    )
}
