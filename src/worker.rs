//! Background I/O thread. eframe's UI thread must never block on serial
//! I/O (it would freeze the GUI), so the actual MSP/power-meter
//! communication happens here, publishing results into `SharedState`
//! under a mutex and waking the UI via `egui::Context::request_repaint()`.

use crate::msp::{self, function, MspLink, PaCalibration};
use crate::power_meter::PowerMeter;
use anyhow::Result;
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[derive(Default)]
pub struct SharedState {
    pub pa_table: Vec<PaCalibration>,
    pub last_dbm: Option<f32>,
    pub last_error: Option<String>,
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
            Ok(l) => l,
            Err(e) => {
                state.lock().unwrap().last_error = Some(format!("VTX open failed: {e}"));
                ctx.request_repaint();
                return;
            }
        };
        let mut meter = match PowerMeter::open(&meter_port, 115200) {
            Ok(m) => m,
            Err(e) => {
                state.lock().unwrap().last_error = Some(format!("Power meter open failed: {e}"));
                ctx.request_repaint();
                return;
            }
        };

        loop {
            while let Ok(cmd) = cmd_rx.try_recv() {
                match cmd {
                    Command::RefreshTable => match read_pa_table(&mut vtx) {
                        Ok(table) => {
                            let mut s = state.lock().unwrap();
                            s.pa_table = table;
                            s.last_error = None;
                        }
                        Err(e) => {
                            state.lock().unwrap().last_error = Some(format!("Table read failed: {e}"));
                        }
                    },
                }
                ctx.request_repaint();
            }

            match meter.read_dbm(Duration::from_millis(300)) {
                Ok(dbm) => {
                    let mut s = state.lock().unwrap();
                    s.last_dbm = Some(dbm);
                    s.last_error = None;
                }
                Err(e) => {
                    state.lock().unwrap().last_error = Some(format!("Meter read failed: {e}"));
                }
            }
            ctx.request_repaint();
            std::thread::sleep(Duration::from_millis(100));
        }
    });
}

fn read_pa_table(link: &mut MspLink) -> Result<Vec<PaCalibration>> {
    link.send_v2(function::PACALTABLE, None)?;
    let mut entries = Vec::new();
    loop {
        match link.read_frame(Duration::from_millis(300))? {
            Some(frame) if frame.function == function::SET_PACALTABLE => {
                entries.push(msp::decode_pa_calibration(&frame.payload)?);
            }
            Some(_) => continue,
            None => break,
        }
    }
    Ok(entries)
}
