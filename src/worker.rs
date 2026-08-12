//! Background I/O thread. eframe's UI thread must never block on serial
//! I/O, so all MSP/power-meter communication happens here. Ports are
//! opened/closed on demand (Connect/Disconnect commands), not
//! automatically at startup unless both were given on the command line
//! (see main.rs).
//!
//! This thread plays two roles on the VTX link: an ACTIVE client
//! (sending queries on Command) and a PASSIVE responder, answering the
//! VTX's own unsolicited MSP_VTX_CONFIG query with the locally-held
//! VtxTableConfig (see vtxtable.rs's header comment for the protocol
//! background). Both share one serial link and one read call each loop
//! iteration -- see the comment at the passive-poll site for the
//! resulting (accepted) race condition.

use crate::msp::{self, function, MspLink};
use crate::power_meter::{PowerMeter, PowerMeterKind};
use crate::vtxtable::VtxTableConfig;
use anyhow::Result;
use log::{debug, error};
use std::collections::VecDeque;
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// How long the power-meter plot's rolling history window covers.
pub const HISTORY_WINDOW_SECS: f64 = 60.0;

pub struct SharedState {
    pub pa_table: Vec<msp::PaCalibration>,
    pub last_dbm: Option<f32>,
    /// (elapsed_secs_since_worker_start, mW) pairs, pruned to the last
    /// HISTORY_WINDOW_SECS. elapsed-seconds rather than wall-clock time
    /// so the plot's x-axis is a simple, always-increasing float.
    pub power_history: VecDeque<(f64, f32)>,
    /// Selected power meter kind -- set immediately when the user picks
    /// it in the left panel's dropdown (even before connecting), so the
    /// calibration page's update-rate dropdown can clamp its options to
    /// this kind's max_update_hz() right away.
    pub meter_kind: PowerMeterKind,
    /// User-configured polling rate, clamped to meter_kind.max_update_hz()
    /// wherever it's set (see pages/calibration.rs).
    pub update_hz: u32,
    /// Last thing the VTX itself reported when WE queried it -- a
    /// debug/confirmation aid, unrelated to the passive auto-responder.
    pub vtx_config: Option<msp::VtxConfig>,
    pub connected: bool,
}

impl Default for SharedState {
    fn default() -> Self {
        Self {
            pa_table: Vec::new(),
            last_dbm: None,
            power_history: VecDeque::new(),
            meter_kind: PowerMeterKind::default(),
            update_hz: 5, // conservative default, within every currently-known meter's max
            vtx_config: None,
            connected: false,
        }
    }
}

pub enum Command {
    Connect {
        vtx_port: String,
        meter_port: String,
        meter_kind: PowerMeterKind,
    },
    Disconnect,
    RefreshCalTable,
    RefreshVtxConfig,
}

pub fn spawn(
    state: Arc<Mutex<SharedState>>,
    vtx_table: Arc<Mutex<VtxTableConfig>>,
    cmd_rx: Receiver<Command>,
    ctx: eframe::egui::Context,
) {
    std::thread::spawn(move || {
        let start = Instant::now();
        let mut vtx: Option<MspLink> = None;
        let mut meter: Option<PowerMeter> = None;
        let mut last_meter_read = Instant::now();

        loop {
            while let Ok(cmd) = cmd_rx.try_recv() {
                match cmd {
                    Command::Connect { vtx_port, meter_port, meter_kind } => {
                        vtx = match MspLink::open(&vtx_port, 115200) {
                            Ok(l) => {
                                debug!(target: "vtx", "opened {vtx_port}");
                                Some(l)
                            }
                            Err(e) => {
                                error!(target: "vtx", "open failed: {e}");
                                None
                            }
                        };
                        meter = match PowerMeter::open(meter_kind, &meter_port) {
                            Ok(m) => {
                                debug!(target: "meter", "opened {meter_port} ({})", meter_kind.name());
                                Some(m)
                            }
                            Err(e) => {
                                error!(target: "meter", "open failed: {e}");
                                None
                            }
                        };
                        let mut s = state.lock().unwrap();
                        s.connected = vtx.is_some() && meter.is_some();
                        s.meter_kind = meter_kind;
                        s.update_hz = s.update_hz.min(meter_kind.max_update_hz()).max(1);
                        s.power_history.clear();
                        drop(s);
                        last_meter_read = Instant::now() - Duration::from_secs(1); // force an immediate first read
                    }

                    Command::Disconnect => {
                        vtx = None;
                        meter = None;
                        debug!(target: "vtx", "disconnected");
                        debug!(target: "meter", "disconnected");
                        state.lock().unwrap().connected = false;
                    }

                    Command::RefreshCalTable => {
                        if let Some(link) = vtx.as_mut() {
                            match read_pa_table(link) {
                                Ok(table) => {
                                    debug!(target: "vtx", "PA table refreshed: {} entries", table.len());
                                    state.lock().unwrap().pa_table = table;
                                }
                                Err(e) => error!(target: "vtx", "PA table read failed: {e}"),
                            }
                        } else {
                            error!(target: "vtx", "RefreshCalTable requested while disconnected");
                        }
                    }

                    Command::RefreshVtxConfig => {
                        if let Some(link) = vtx.as_mut() {
                            match read_vtx_config(link) {
                                Ok(cfg) => {
                                    debug!(target: "vtx", "VTX reports: band={} ch={} freq={} power={} pit={}",
                                        cfg.band, cfg.channel, cfg.frequency_mhz, cfg.power, cfg.pitmode);
                                    state.lock().unwrap().vtx_config = Some(cfg);
                                }
                                Err(e) => error!(target: "vtx", "config read failed: {e}"),
                            }
                        } else {
                            error!(target: "vtx", "RefreshVtxConfig requested while disconnected");
                        }
                    }
                }
                ctx.request_repaint();
            }

            // Passive: answer any unsolicited query from the VTX (chiefly
            // MSP_VTX_CONFIG at its boot). See module doc for the shared
            // read-call trade-off with the active command handlers above.
            if let Some(link) = vtx.as_mut() {
                if let Ok(Some(frame)) = link.read_frame(Duration::from_millis(20)) {
                    if frame.function == function::VTX_CONFIG && frame.payload.is_empty() {
                        let response = vtx_table.lock().unwrap().encode_vtx_config_response();
                        match link.send_v1(function::VTX_CONFIG as u8, &response) {
                            Ok(()) => debug!(target: "vtx", "answered VTX_CONFIG query (acting as FC)"),
                            Err(e) => error!(target: "vtx", "failed to answer VTX_CONFIG query: {e}"),
                        }
                        ctx.request_repaint();
                    }
                }
            }

            // Power meter poll, rate-limited to the configured Hz
            // (clamped to the connected meter's max -- see Command::Connect
            // and pages/calibration.rs's dropdown). Outer loop tick is
            // 10ms so higher Hz settings (e.g. 20Hz = 50ms interval) are
            // actually achievable, not just theoretically requested.
            if let Some(m) = meter.as_mut() {
                let interval = {
                    let hz = state.lock().unwrap().update_hz.max(1);
                    Duration::from_secs_f64(1.0 / hz as f64)
                };
                if last_meter_read.elapsed() >= interval {
                    last_meter_read = Instant::now();
                    match m.read_dbm(Duration::from_millis(300)) {
                        Ok(dbm) => {
                            let mw = 10f32.powf(dbm / 10.0);
                            debug!(target: "meter", "{dbm:.2} dBm ({mw:.6} mW)");
                            let elapsed = start.elapsed().as_secs_f64();
                            let mut s = state.lock().unwrap();
                            s.last_dbm = Some(dbm);
                            s.power_history.push_back((elapsed, mw));
                            while let Some(&(t, _)) = s.power_history.front() {
                                if elapsed - t > HISTORY_WINDOW_SECS {
                                    s.power_history.pop_front();
                                } else {
                                    break;
                                }
                            }
                        }
                        Err(e) => error!(target: "meter", "read failed: {e}"),
                    }
                    ctx.request_repaint();
                }
            }

            std::thread::sleep(Duration::from_millis(10));
        }
    });
}

fn read_pa_table(link: &mut MspLink) -> Result<Vec<msp::PaCalibration>> {
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

fn read_vtx_config(link: &mut MspLink) -> Result<msp::VtxConfig> {
    link.send_v1(function::VTX_CONFIG as u8, &[])?;
    debug!(target: "vtx", "sent VTX_CONFIG query");
    loop {
        match link.read_frame(Duration::from_millis(300))? {
            Some(frame) if frame.function == function::VTX_CONFIG && !frame.payload.is_empty() => {
                return msp::decode_vtx_config(&frame.payload);
            }
            Some(_) => continue,
            None => anyhow::bail!("no VTX_CONFIG response within timeout"),
        }
    }
}
