//! Background I/O thread. eframe's UI thread must never block on serial
//! I/O (it would freeze the GUI), so the actual MSP/power-meter
//! communication happens here. Live values (power meter reading, PA
//! table) go into `SharedState` under a mutex; status/error messages go
//! through the `log` crate (target "vtx" or "meter") into the buffers
//! `logging::init()` set up, which app.rs renders as two scrollable panels.

use crate::msp::{self, function, MspLink, PaCalibration};
use crate::power_meter::PowerMeter;
use anyhow::Result;
use log::{debug, error};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[derive(Default)]
pub struct SharedState {
    pub pa_table: Vec<PaCalibration>,
    pub last_dbm: Option<f32>,
}

pub enum Command {
    RefreshTable,
}

pub fn spawn(
    vtx_port: String,
    meter_port: String,
    state: Arc<Mutex<SharedState>>,
    cmd_rx: Receiver<Command>,
    ctx: eframe::egui::Context,
) {
    std::thread::spawn(move || {
        let mut vtx = match MspLink::open(&vtx_port, 115200) {
            Ok(l) => {
                debug!(target: "vtx", "opened {vtx_port}");
                l
            }
            Err(e) => {
                error!(target: "vtx", "open failed: {e}");
                ctx.request_repaint();
                return;
            }
        };
        let mut meter = match PowerMeter::open(&meter_port, 115200) {
            Ok(m) => {
                debug!(target: "meter", "opened {meter_port}");
                m
            }
            Err(e) => {
                error!(target: "meter", "open failed: {e}");
                ctx.request_repaint();
                return;
            }
        };

        loop {
            while let Ok(cmd) = cmd_rx.try_recv() {
                match cmd {
                    Command::RefreshTable => match read_pa_table(&mut vtx) {
                        Ok(table) => {
                            debug!(target: "vtx", "PA table refreshed: {} entries", table.len());
                            state.lock().unwrap().pa_table = table;
                        }
                        Err(e) => error!(target: "vtx", "table read failed: {e}"),
                    },
                }
                ctx.request_repaint();
            }

            match meter.read_dbm(Duration::from_millis(300)) {
                Ok(dbm) => {
                    debug!(target: "meter", "{dbm:.2} dBm");
                    state.lock().unwrap().last_dbm = Some(dbm);
                }
                Err(e) => error!(target: "meter", "read failed: {e}"),
            }
            ctx.request_repaint();
            std::thread::sleep(Duration::from_millis(100));
        }
    });
}

fn read_pa_table(link: &mut MspLink) -> Result<Vec<PaCalibration>> {
    link.send_v2(function::PACALTABLE, None)?;
    debug!(target: "vtx", "sent PACALTABLE request");
    let mut entries = Vec::new();
    loop {
        match link.read_frame(Duration::from_millis(300))? {
            Some(frame) if frame.function == function::SET_PACALTABLE => {
                let entry = msp::decode_pa_calibration(&frame.payload)?;
                debug!(target: "vtx", "received level {} ({} mW)", entry.idx, entry.m_w);
                entries.push(entry);
            }
            Some(frame) => {
                debug!(target: "vtx", "ignored frame, function=0x{:04x}", frame.function);
            }
            None => break,
        }
    }
    Ok(entries)
}
