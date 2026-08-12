//! Background I/O thread. eframe's UI thread must never block on serial
//! I/O, so all MSP/power-meter communication happens here. Unlike the
//! earlier version, ports are opened/closed on demand (Connect/Disconnect
//! commands) rather than once at startup -- see app.rs's left-panel
//! connect controls.
//!
//! This thread now plays two roles on the VTX link: an ACTIVE client
//! (sending queries on Command, e.g. RefreshCalTable) and a PASSIVE
//! responder, answering the VTX's own unsolicited MSP_VTX_CONFIG query
//! (empty payload) with the locally-held VtxTableConfig, the same way
//! Betaflight's own vtxTableConfig() answers a VTX at boot (see
//! vtxtable.rs's header comment for the protocol background this is
//! based on). Both share one serial link and one read call each loop
//! iteration -- see the comment at the passive-poll site for the
//! resulting (accepted) race condition.

use crate::msp::{self, function, MspLink};
use crate::power_meter::PowerMeter;
use crate::vtxtable::VtxTableConfig;
use anyhow::Result;
use log::{debug, error};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[derive(Default)]
pub struct SharedState {
    pub pa_table: Vec<msp::PaCalibration>,
    pub last_dbm: Option<f32>,
    /// Last thing the VTX itself reported when WE queried it -- a
    /// debug/confirmation aid, separate from (and unrelated to) the
    /// passive auto-responder below, which answers the VTX's queries
    /// rather than making them.
    pub vtx_config: Option<msp::VtxConfig>,
    pub connected: bool,
}

pub enum Command {
    Connect { vtx_port: String, meter_port: String },
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
        let mut vtx: Option<MspLink> = None;
        let mut meter: Option<PowerMeter> = None;

        loop {
            while let Ok(cmd) = cmd_rx.try_recv() {
                match cmd {
                    Command::Connect { vtx_port, meter_port } => {
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
                        meter = match PowerMeter::open(&meter_port, 115200) {
                            Ok(m) => {
                                debug!(target: "meter", "opened {meter_port}");
                                Some(m)
                            }
                            Err(e) => {
                                error!(target: "meter", "open failed: {e}");
                                None
                            }
                        };
                        state.lock().unwrap().connected = vtx.is_some() && meter.is_some();
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
            // MSP_VTX_CONFIG at its boot). Shares the same read call
            // budget as the active command handlers above -- if a query
            // happens to arrive in the same narrow window as an active
            // command awaiting its own reply, one can "steal" the
            // other's frame (the loser just times out and, for a button
            // click, can simply be retried; VTX_CONFIG queries only
            // really happen once at VTX boot in this firmware, so this
            // is an accepted trade-off rather than a full frame-dispatch
            // queue).
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

            if let Some(m) = meter.as_mut() {
                match m.read_dbm(Duration::from_millis(300)) {
                    Ok(dbm) => {
                        debug!(target: "meter", "{dbm:.2} dBm");
                        state.lock().unwrap().last_dbm = Some(dbm);
                    }
                    Err(e) => error!(target: "meter", "read failed: {e}"),
                }
                ctx.request_repaint();
            }

            std::thread::sleep(Duration::from_millis(50));
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
