//! Background I/O thread. eframe's UI thread must never block on serial
//! I/O, so all MSP/power-meter communication happens here. Ports are
//! opened/closed on demand (Connect/Disconnect commands), not
//! automatically at startup unless both were given on the command line
//! (see main.rs).
//!
//! This thread plays several roles on the VTX link, all sharing ONE read
//! call per loop tick (see the dispatch in the main loop below): an
//! ACTIVE client (sending queries on Command, each with its own
//! dedicated read-until-quiet loop), a PASSIVE responder (answering the
//! VTX's own unsolicited MSP_VTX_CONFIG query), and -- new in this
//! revision -- the calibration sweep engine's PACALIBRATION response
//! listener. The sweep itself (calibration.rs) is advanced one step per
//! tick here too, rather than run as a blocking loop, so meter polling
//! and the passive responder keep working throughout a sweep that can
//! take several minutes.

use crate::calibration::{self, SweepEngine};
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

pub type SharedSweep = Arc<Mutex<Option<SweepEngine>>>;

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
    /// wherever it's set (see pages/calibration.rs). f64 rather than an
    /// integer Hz -- the dropdown includes fractional rates (0.5Hz).
    pub update_hz: f64,
    /// Set while a sweep is active: whatever update_hz was before the
    /// sweep forced it to sweep_hz, so it can be restored when the sweep
    /// finishes or aborts.
    pub pre_sweep_update_hz: Option<f64>,
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
            update_hz: 1.0, // conservative default; ImmersionRC V1's own max is now 5Hz, so this leaves headroom below it and stays valid for any future slower-max meter too
            pre_sweep_update_hz: None,
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
    /// Actively sends the current VtxTableConfig selection (band/channel/
    /// frequency/pitmode/power) to the VTX, unprompted -- unlike the
    /// passive auto-responder (which only answers the VTX's own empty
    /// MSP_VTX_CONFIG query), this pushes the same 15-byte payload as a
    /// command. vtx_msp.c's dispatch only branches on payload size
    /// (empty = query, >=15 bytes = apply), not on the request/response
    /// marker, so this is processed by the exact same
    /// handle_msp_set_vtx_config() path a real FC's push would hit.
    PushVtxConfig,
    /// Starts a calibration sweep over `levels` (1-based indices into the
    /// PA table, from the checked rows) at the given detector tolerance.
    /// Frequencies and per-level mW targets come from the already-loaded
    /// pa_table -- refresh it first if it's empty/stale.
    StartSweep { levels: Vec<u8>, tolerance_pct: f32 },
    /// UI confirms the user has retuned a manual-frequency meter.
    ConfirmFrequency,
    /// Stops the sweep and pushes a pitmode-forced VTX_CONFIG as a safe
    /// state (see calibration::safe_state_payload).
    AbortSweep,
    /// Pushes the working pa_table's calibration[]/detector[] for every
    /// real level (idx >= 1) to the VTX via SET_PACALTABLE. Independent
    /// of SaveEeprom -- this only updates the VTX's RAM copy.
    SendCalTableToVtx,
    /// Commits whatever's currently in the VTX's RAM (including
    /// anything sent by SendCalTableToVtx) to EEPROM.
    SaveEeprom,
}

pub fn spawn(
    state: Arc<Mutex<SharedState>>,
    vtx_table: Arc<Mutex<VtxTableConfig>>,
    sweep: SharedSweep,
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
                        s.update_hz = s.update_hz.min(meter_kind.max_update_hz() as f64).max(0.01);
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
                        *sweep.lock().unwrap() = None;
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

                    Command::PushVtxConfig => {
                        if let Some(link) = vtx.as_mut() {
                            let payload = vtx_table.lock().unwrap().encode_vtx_config_response();
                            match link.send_v1(function::VTX_CONFIG as u8, &payload) {
                                Ok(()) => debug!(target: "vtx", "pushed VTX_CONFIG (Save)"),
                                Err(e) => error!(target: "vtx", "failed to push VTX_CONFIG: {e}"),
                            }
                        } else {
                            error!(target: "vtx", "PushVtxConfig requested while disconnected");
                        }
                    }

                    Command::StartSweep { levels, tolerance_pct } => {
                        if vtx.is_none() {
                            error!(target: "vtx", "StartSweep requested while disconnected");
                        } else {
                            let (pa_table, meter_kind, prev_update_hz) = {
                                let s = state.lock().unwrap();
                                (s.pa_table.clone(), s.meter_kind, s.update_hz)
                            };
                            let freq_entry = pa_table.iter().find(|e| e.idx == 0);
                            let frequencies: Vec<u16> =
                                freq_entry.map(|e| e.value.iter().copied().filter(|&f| f > 0).collect()).unwrap_or_default();
                            let sign_inverted = freq_entry.map(|e| e.dac_sign_inverted).unwrap_or(false);

                            let mut target_mw_by_level = std::collections::HashMap::new();
                            for &lvl in &levels {
                                if let Some(entry) = pa_table.iter().find(|e| e.idx == lvl) {
                                    target_mw_by_level.insert(lvl, entry.m_w);
                                }
                            }

                            if frequencies.is_empty() {
                                error!(target: "vtx", "StartSweep: no frequency breakpoints in the PA table -- Refresh it first");
                            } else if levels.is_empty() {
                                error!(target: "vtx", "StartSweep: no power levels selected");
                            } else {
                                let mut engine = SweepEngine::new(
                                    levels,
                                    frequencies,
                                    tolerance_pct,
                                    sign_inverted,
                                    target_mw_by_level,
                                    meter_kind.max_update_hz(),
                                );
                                engine.start(meter_kind.requires_manual_frequency());
                                let sweep_hz = engine.sweep_hz;
                                debug!(target: "vtx", "sweep started: {} levels, {} frequencies, tolerance {tolerance_pct}%",
                                    engine.levels.len(), engine.frequencies.len());
                                {
                                    let mut s = state.lock().unwrap();
                                    s.pre_sweep_update_hz = Some(prev_update_hz);
                                    s.update_hz = sweep_hz;
                                }
                                *sweep.lock().unwrap() = Some(engine);
                            }
                        }
                    }

                    Command::ConfirmFrequency => {
                        if let Some(engine) = sweep.lock().unwrap().as_mut() {
                            engine.confirm_frequency();
                            debug!(target: "vtx", "frequency change confirmed by user");
                        }
                    }

                    Command::AbortSweep => {
                        let payload = calibration::safe_state_payload(&vtx_table.lock().unwrap());
                        if let Some(link) = vtx.as_mut() {
                            match link.send_v1(function::VTX_CONFIG as u8, &payload) {
                                Ok(()) => debug!(target: "vtx", "sweep aborted, pitmode-safe state sent"),
                                Err(e) => error!(target: "vtx", "abort: failed to send safe state: {e}"),
                            }
                        }
                        if let Some(engine) = sweep.lock().unwrap().as_mut() {
                            engine.abort();
                        }
                        let mut s = state.lock().unwrap();
                        if let Some(prev) = s.pre_sweep_update_hz.take() {
                            s.update_hz = prev;
                        }
                    }

                    Command::SendCalTableToVtx => {
                        if let Some(link) = vtx.as_mut() {
                            let pa_table = state.lock().unwrap().pa_table.clone();
                            let mut sent = 0;
                            for entry in pa_table.iter().filter(|e| e.idx > 0) {
                                let payload = msp::encode_pa_calibration(entry);
                                if link.send_v2(function::SET_PACALTABLE, Some(&payload)).is_ok() {
                                    sent += 1;
                                }
                            }
                            debug!(target: "vtx", "sent {sent} calibration table entries to VTX");
                        } else {
                            error!(target: "vtx", "SendCalTableToVtx requested while disconnected");
                        }
                    }

                    Command::SaveEeprom => {
                        if let Some(link) = vtx.as_mut() {
                            match link.send_v1(function::EEPROM_WRITE as u8, &[]) {
                                Ok(()) => debug!(target: "vtx", "sent EEPROM_WRITE"),
                                Err(e) => error!(target: "vtx", "failed to send EEPROM_WRITE: {e}"),
                            }
                        } else {
                            error!(target: "vtx", "SaveEeprom requested while disconnected");
                        }
                    }
                }
                ctx.request_repaint();
            }

            // One read per tick, dispatched to whichever consumer wants
            // it: the passive VTX_CONFIG responder, or the sweep engine's
            // PACALIBRATION response listener. See module doc for why
            // this is consolidated into a single read rather than one
            // per consumer (the earlier version's frame-stealing race
            // between separate reads).
            let mut pa_calibration_reading: Option<msp::PaCalibrationReading> = None;
            if let Some(link) = vtx.as_mut() {
                if let Ok(Some(frame)) = link.read_frame(Duration::from_millis(20)) {
                    if frame.function == function::VTX_CONFIG && frame.payload.is_empty() {
                        let response = vtx_table.lock().unwrap().encode_vtx_config_response();
                        match link.send_v1(function::VTX_CONFIG as u8, &response) {
                            Ok(()) => debug!(target: "vtx", "answered VTX_CONFIG query (acting as FC)"),
                            Err(e) => error!(target: "vtx", "failed to answer VTX_CONFIG query: {e}"),
                        }
                        ctx.request_repaint();
                    } else if frame.function == function::PACALIBRATION {
                        if let Ok(reading) = msp::decode_pa_calibration_reading(&frame.payload) {
                            pa_calibration_reading = Some(reading);
                        }
                    }
                }
            }

            // Advance the sweep by one step, if active.
            if let Some(link) = vtx.as_mut() {
                let history_snapshot = state.lock().unwrap().power_history.clone();
                let mut sweep_guard = sweep.lock().unwrap();
                if let Some(engine) = sweep_guard.as_mut() {
                    if let Err(e) = engine.poll(link, &history_snapshot, pa_calibration_reading) {
                        error!(target: "vtx", "sweep step failed: {e}");
                    }
                    if let Some(result) = engine.pending_result.take() {
                        let mut s = state.lock().unwrap();
                        if let Some(entry) = s.pa_table.iter_mut().find(|e| e.idx == result.level) {
                            if let Some(mv) = result.calibration_mv {
                                if let Some(slot) = entry.value.get_mut(result.freq_idx) {
                                    *slot = mv;
                                }
                            }
                            if let Some(det) = result.detector_mv {
                                if let Some(slot) = entry.detector.get_mut(result.freq_idx) {
                                    *slot = det;
                                }
                            }
                        }
                        debug!(target: "vtx", "sweep result: level={} freq_idx={} cal={:?} det={:?}",
                            result.level, result.freq_idx, result.calibration_mv, result.detector_mv);
                    }
                    if !engine.is_active() {
                        // Sweep finished (not aborted -- AbortSweep restores this itself).
                        let mut s = state.lock().unwrap();
                        if let Some(prev) = s.pre_sweep_update_hz.take() {
                            s.update_hz = prev;
                        }
                        debug!(target: "vtx", "sweep finished");
                    }
                    ctx.request_repaint();
                }
            }

            // Power meter poll, rate-limited to the configured Hz
            // (clamped to the connected meter's max -- see Command::Connect
            // and pages/calibration.rs's dropdown). Outer loop tick is
            // 10ms so higher Hz settings (e.g. 20Hz = 50ms interval) are
            // actually achievable, not just theoretically requested.
            if let Some(m) = meter.as_mut() {
                let interval = {
                    let hz = state.lock().unwrap().update_hz.max(0.01);
                    Duration::from_secs_f64(1.0 / hz)
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
    let quiet_period = Duration::from_millis(300);
    // Deadline is only ever pushed out by a MATCHING frame, not by any
    // traffic at all -- the VTX firmware sends its own MSP_STATUS/MSP_RC
    // requests every 100ms regardless of what's listening, so a naive
    // "reset on any frame" timeout would never fire and this loop would
    // block the whole worker thread indefinitely (this is what was
    // actually happening: "unknown command 0x0065/0x0069" in the log is
    // exactly MSP_STATUS/MSP_RC arriving here and getting ignored, but
    // still (previously) resetting the clock).
    let mut deadline = Instant::now() + quiet_period;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        match link.read_frame(remaining)? {
            Some(frame) if frame.function == function::SET_PACALTABLE => {
                let entry = msp::decode_pa_calibration(&frame.payload)?;
                debug!(target: "vtx", "received level {} ({} mW)", entry.idx, entry.m_w);
                entries.push(entry);
                deadline = Instant::now() + quiet_period; // still hearing PACALTABLE entries -- extend the window
            }
            Some(frame) => {
                // Unrelated traffic -- ignored, deadline NOT extended.
                debug!(target: "vtx", "ignored frame while collecting PACALTABLE, function=0x{:04x}", frame.function);
            }
            None => break,
        }
    }
    Ok(entries)
}

fn read_vtx_config(link: &mut MspLink) -> Result<msp::VtxConfig> {
    link.send_v1(function::VTX_CONFIG as u8, &[])?;
    debug!(target: "vtx", "sent VTX_CONFIG query");
    let deadline = Instant::now() + Duration::from_millis(300); // fixed -- unrelated traffic must not extend this, same reasoning as read_pa_table
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            anyhow::bail!("no VTX_CONFIG response within timeout");
        }
        match link.read_frame(remaining)? {
            Some(frame) if frame.function == function::VTX_CONFIG && !frame.payload.is_empty() => {
                return msp::decode_vtx_config(&frame.payload);
            }
            Some(_) => continue,
            None => anyhow::bail!("no VTX_CONFIG response within timeout"),
        }
    }
}
