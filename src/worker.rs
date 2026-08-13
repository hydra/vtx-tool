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
//! listener. The sweep itself (calibration_engine.rs) is advanced one step per
//! tick here too, rather than run as a blocking loop, so meter polling
//! and the passive responder keep working throughout a sweep that can
//! take several minutes.

use crate::calibration_engine::{self, SweepEngine};
use crate::conn_status::PortState;
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
    /// Total power readings ever taken, monotonically increasing --
    /// unlike power_history.len() (which is capped by the rolling
    /// HISTORY_WINDOW_SECS window and stops growing once it fills, since
    /// every push evicts an old entry), this never plateaus. The sweep
    /// engine's "wait for K new readings" tracking uses this instead of
    /// power_history.len() for exactly that reason -- using len() was a
    /// real bug (see calibration_engine.rs's SampleWait doc comment).
    pub reading_seq: u64,
    /// Selected power meter kind -- set immediately when the user picks
    /// it in the left panel's dropdown (even before connecting), so the
    /// calibration page's update-rate dropdown can clamp its options to
    /// this kind's max_update_hz() right away.
    pub meter_kind: PowerMeterKind,
    /// Correction applied (added) to every raw dBm reading before it's
    /// stored or converted to mW -- e.g. an external attenuator fitted
    /// ahead of the meter that its own display already accounts for but
    /// the raw serial readings don't. Set via the calibration page's
    /// slider; defaults from AppSettings::attenuation_db (see main.rs)
    /// and is persisted back there whenever the user changes it.
    pub attenuation_db: f32,
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
    /// Each port's own connection lifecycle -- see conn_status.rs's
    /// module doc for why this is tracked per-port rather than as one
    /// combined "connected" flag (which conflated "both ports opened"
    /// with "at least one succeeded", losing exactly the distinction
    /// needed to show accurate per-port status or gate the Connect
    /// button correctly).
    pub vtx_port_state: PortState,
    pub meter_port_state: PortState,
    /// True if ANY frame (not just ones a specific command was waiting
    /// for) has been seen from the VTX within the last READY_WINDOW.
    /// NOT the same thing as vtx_port_state -- this is a narrower
    /// heartbeat used only by the calibration sweep to detect a
    /// current-limited supply power-cycling the VTX mid-sweep (see
    /// conn_status.rs's module doc for the full distinction).
    pub vtx_ready: bool,
}

/// How recently the VTX (or power meter) must have said ANYTHING for
/// vtx_ready to be true / for the meter's alive-check to reconfirm
/// meter_port_state::Ready. The VTX's own firmware chatters
/// (MSP_STATUS/MSP_RC) roughly every 100ms on its own, and the meter's
/// alive-check runs every 100ms too, so this has real margin without
/// being slow to notice a genuine loss.
const READY_WINDOW: Duration = Duration::from_millis(500);
/// How often to retry reopening a port whose handle was dropped after a
/// hard I/O error while in PortState::LostCommunication (e.g. the device
/// was unplugged) -- frequent enough to reconnect promptly once it's
/// plugged back in, without hammering the OS.
const RECONNECT_INTERVAL: Duration = Duration::from_secs(1);

impl Default for SharedState {
    fn default() -> Self {
        Self {
            pa_table: Vec::new(),
            last_dbm: None,
            power_history: VecDeque::new(),
            reading_seq: 0,
            meter_kind: PowerMeterKind::default(),
            attenuation_db: 30.0, // matches settings.rs's default_attenuation_db(); main.rs overwrites this from AppSettings right after construction
            update_hz: 1.0, // conservative default; ImmersionRC V1's own max is now 5Hz, so this leaves headroom below it and stays valid for any future slower-max meter too
            pre_sweep_update_hz: None,
            vtx_config: None,
            vtx_port_state: PortState::Disconnected,
            meter_port_state: PortState::Disconnected,
            vtx_ready: false,
        }
    }
}

pub enum Command {
    /// No-op (logged) if the VTX port is already open -- see
    /// PortState::is_idle's doc comment.
    ConnectVtx { port: String },
    /// No-op (logged) if the VTX port is already closed.
    DisconnectVtx,
    /// No-op (logged) if the meter port is already open.
    ConnectMeter { port: String, meter_kind: PowerMeterKind },
    /// No-op (logged) if the meter port is already closed.
    DisconnectMeter,
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
    /// pa_table -- refresh it first if it's empty/stale. The UI is
    /// expected to have already gated this on
    /// conn_status::OverallState::Ready, but the handler re-checks both
    /// ports defensively regardless.
    StartSweep { levels: Vec<u8>, tolerance_pct: f32 },
    /// UI confirms the user has retuned a manual-frequency meter.
    ConfirmFrequency,
    /// Stops the sweep and pushes a pitmode-forced VTX_CONFIG as a safe
    /// state (see calibration_engine::safe_state_payload).
    AbortSweep,
    /// Skip whatever (level, freq) point is currently in progress and
    /// move on to whatever's next in normal progression order (see
    /// SweepEngine::skip_current).
    SkipCurrent,
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
        let mut vtx_last_seen: Option<Instant> = None;
        let mut meter_last_seen: Option<Instant> = None; // updated on any successful reply (V check or D read)
        let mut last_meter_alive_check = Instant::now(); // when the "V" presence-check last ran, for its own 100ms retry cadence
        // Remembered only across a successful connect -> cleared on an
        // explicit Disconnect, so LostCommunication's periodic reopen
        // (below) only ever fires for a port that was actually working
        // and lost it, never for one the user deliberately closed.
        let mut vtx_port_path: Option<String> = None;
        let mut meter_port_path: Option<String> = None;
        let mut vtx_last_reconnect_attempt = Instant::now();
        let mut meter_last_reconnect_attempt = Instant::now();

        loop {
            while let Ok(cmd) = cmd_rx.try_recv() {
                match cmd {
                    Command::ConnectVtx { port } => {
                        if vtx.is_some() {
                            debug!(target: "vtx", "ConnectVtx requested but already connected -- ignoring");
                        } else {
                            state.lock().unwrap().vtx_port_state = PortState::Connecting;
                            ctx.request_repaint();
                            match MspLink::open(&port, 115200) {
                                Ok(l) => {
                                    debug!(target: "vtx", "opened {port}");
                                    vtx = Some(l);
                                    vtx_last_seen = None;
                                    vtx_port_path = Some(port.clone());
                                    state.lock().unwrap().vtx_port_state = PortState::Ready;

                                    // Push a pitmode-safe VTX_CONFIG immediately on connect,
                                    // regardless of what the VTX was doing before. This isn't
                                    // specific to the VTX_CONFIG payload itself -- what actually
                                    // matters is that vtx_apply_hw() runs at all, which
                                    // unconditionally calls rf_pa_apply_level(NULL) as its very
                                    // first step, and THAT clears g_calibration_override
                                    // regardless of pitmode. A prior session's sweep can leave
                                    // that flag set (it suspends the closed loop so a manual mV
                                    // override holds still -- see rf_pa_set_calibration() in the
                                    // firmware), and if left set across a reconnect, a fresh
                                    // session inherits a VTX that's not running its normal
                                    // closed loop. This guarantees a known-clean starting point
                                    // every time, independent of how the previous session ended.
                                    if let Some(link) = vtx.as_mut() {
                                        let payload = calibration_engine::safe_state_payload(&vtx_table.lock().unwrap());
                                        match link.send_v1(function::VTX_CONFIG as u8, &payload) {
                                            Ok(()) => debug!(target: "vtx", "pushed pitmode-safe VTX_CONFIG on connect"),
                                            Err(e) => error!(target: "vtx", "failed to push safe-state VTX_CONFIG on connect: {e}"),
                                        }
                                    }
                                }
                                Err(e) => {
                                    error!(target: "vtx", "open failed: {e}");
                                    state.lock().unwrap().vtx_port_state = PortState::Disconnected;
                                }
                            }
                        }
                    }

                    Command::DisconnectVtx => {
                        if vtx.is_none() {
                            debug!(target: "vtx", "DisconnectVtx requested but not connected -- ignoring");
                        } else {
                            state.lock().unwrap().vtx_port_state = PortState::Disconnecting;
                            ctx.request_repaint();
                            vtx = None;
                            vtx_last_seen = None;
                            vtx_port_path = None;
                            debug!(target: "vtx", "disconnected");
                            let mut s = state.lock().unwrap();
                            s.vtx_port_state = PortState::Disconnected;
                            s.vtx_ready = false;
                            drop(s);
                            *sweep.lock().unwrap() = None;
                        }
                    }

                    Command::ConnectMeter { port, meter_kind } => {
                        if meter.is_some() {
                            debug!(target: "meter", "ConnectMeter requested but already connected -- ignoring");
                        } else {
                            {
                                let mut s = state.lock().unwrap();
                                s.meter_port_state = PortState::Connecting;
                                s.meter_kind = meter_kind;
                                s.update_hz = s.update_hz.min(meter_kind.max_update_hz() as f64).max(0.01);
                                s.power_history.clear();
                            }
                            ctx.request_repaint();
                            match PowerMeter::open(meter_kind, &port) {
                                Ok(m) => {
                                    debug!(target: "meter", "opened {port} ({})", meter_kind.name());
                                    meter = Some(m);
                                    meter_last_seen = None;
                                    meter_port_path = Some(port.clone());
                                    state.lock().unwrap().meter_port_state = PortState::Ready;
                                    last_meter_read = Instant::now() - Duration::from_secs(1); // force an immediate first read
                                    last_meter_alive_check = Instant::now() - Duration::from_secs(1); // force an immediate first alive-check
                                }
                                Err(e) => {
                                    error!(target: "meter", "open failed: {e}");
                                    state.lock().unwrap().meter_port_state = PortState::Disconnected;
                                }
                            }
                        }
                    }

                    Command::DisconnectMeter => {
                        if meter.is_none() {
                            debug!(target: "meter", "DisconnectMeter requested but not connected -- ignoring");
                        } else {
                            state.lock().unwrap().meter_port_state = PortState::Disconnecting;
                            ctx.request_repaint();
                            meter = None;
                            meter_last_seen = None;
                            meter_port_path = None;
                            debug!(target: "meter", "disconnected");
                            state.lock().unwrap().meter_port_state = PortState::Disconnected;
                        }
                    }

                    Command::RefreshCalTable => {
                        if let Some(link) = vtx.as_mut() {
                            match read_pa_table(link) {
                                Ok(table) => {
                                    debug!(target: "vtx", "PA table refreshed: {} entries", table.len());
                                    state.lock().unwrap().pa_table = table;
                                    if let Some(engine) = sweep.lock().unwrap().as_mut() {
                                        engine.clear_hard_limits();
                                        engine.clear_cell_status();
                                    }
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
                        let (vtx_state_now, meter_state_now) = {
                            let s = state.lock().unwrap();
                            (s.vtx_port_state, s.meter_port_state)
                        };
                        if vtx_state_now != PortState::Ready || meter_state_now != PortState::Ready {
                            error!(target: "vtx", "StartSweep requested while not fully connected (vtx={:?} meter={:?})",
                                vtx_state_now, meter_state_now);
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
                                engine.start(meter_kind.capability());
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
                        let payload = calibration_engine::safe_state_payload(&vtx_table.lock().unwrap());
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
                    Command::SkipCurrent => {
                        // Mirrors AbortSweep's own safe-state push, and for the same
                        // reason: skip_current() itself only updates internal
                        // bookkeeping (see its doc comment) and was never sending
                        // anything to the VTX at all. The engine's own frequency-
                        // advance logic WILL push a fresh retune shortly after this
                        // (even when the manual-frequency prompt itself gets
                        // consolidated/skipped for being the same band as before),
                        // but that's an ordinary retune, not a safety reset -- it
                        // doesn't force pitmode, and under repeated rapid skips (real
                        // symptom seen: the VTX stopped responding to
                        // SET_PACALIBRATION at all after several skip-then-retune
                        // cycles in quick succession, recoverable only via a full
                        // reconnect+power-cycle) that leaves the VTX cycling through
                        // several override-then-retune transitions back to back with
                        // no defined safe/settled point in between. Pushing pitmode
                        // here gives it one.
                        let payload = calibration_engine::safe_state_payload(&vtx_table.lock().unwrap());
                        if let Some(link) = vtx.as_mut() {
                            match link.send_v1(function::VTX_CONFIG as u8, &payload) {
                                Ok(()) => debug!(target: "vtx", "skip: pitmode-safe state sent before advancing"),
                                Err(e) => error!(target: "vtx", "skip: failed to send safe state: {e}"),
                            }
                        }
                        if let Some(engine) = sweep.lock().unwrap().as_mut() {
                            engine.skip_current();
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
            // between separate reads). ANY frame received here also
            // counts as a heartbeat -- see vtx_last_seen/vtx_ready.
            let mut pa_calibration_reading: Option<msp::PaCalibrationReading> = None;
            let mut vtx_link_lost = false;
            if let Some(link) = vtx.as_mut() {
                match link.read_frame(Duration::from_millis(20)) {
                    Ok(Some(frame)) => {
                        vtx_last_seen = Some(Instant::now());
                        if frame.function == function::VTX_CONFIG && frame.payload.is_empty() {
                            let response = vtx_table.lock().unwrap().encode_vtx_config_response();
                            match link.send_v1(function::VTX_CONFIG as u8, &response) {
                                Ok(()) => debug!(target: "vtx", "answered VTX_CONFIG query (acting as FC)"),
                                Err(e) => {
                                    error!(target: "vtx", "failed to answer VTX_CONFIG query: {e}");
                                    vtx_link_lost = true;
                                }
                            }
                            ctx.request_repaint();
                        } else if frame.function == function::PACALIBRATION {
                            if let Ok(reading) = msp::decode_pa_calibration_reading(&frame.payload) {
                                pa_calibration_reading = Some(reading);
                            }
                        }
                    }
                    Ok(None) => {} // benign timeout -- no traffic, not an error
                    Err(e) => {
                        // A real I/O error (as opposed to a timeout) -- most likely
                        // the port itself is gone (device unplugged). Disconnect
                        // rather than spin retrying a dead port.
                        error!(target: "vtx", "VTX link error, disconnecting: {e}");
                        vtx_link_lost = true;
                    }
                }
            }
            if vtx_link_lost {
                vtx = None;
                vtx_last_seen = None;
                let mut s = state.lock().unwrap();
                s.vtx_port_state = PortState::LostCommunication;
                s.vtx_ready = false;
                drop(s);
                if let Some(engine) = sweep.lock().unwrap().as_mut() {
                    engine.force_connection_lost(calibration_engine::ConnectionLossReason::Vtx);
                }
            }
            let vtx_ready = vtx_last_seen.map(|t| t.elapsed() < READY_WINDOW).unwrap_or(false);
            if vtx.is_some() {
                let mut s = state.lock().unwrap();
                s.vtx_ready = vtx_ready;
                if vtx_ready {
                    if s.vtx_port_state != PortState::Ready {
                        s.vtx_port_state = PortState::Ready;
                    }
                } else if s.vtx_port_state == PortState::Ready {
                    // Handle still open, but no traffic in a while -- general
                    // "lost communication" display. The sweep (if any) notices
                    // this same staleness via vtx_ready in its own poll() and
                    // pauses itself; this just keeps the left panel honest
                    // outside of a sweep too. Auto-recovers to Ready above the
                    // moment fresh traffic arrives, no reopen needed since the
                    // handle was never actually broken.
                    s.vtx_port_state = PortState::LostCommunication;
                }
            }

            // Periodic reopen while LostCommunication with the handle actually
            // dropped (the hard-I/O-error case, e.g. device unplugged) --
            // vtx_port_path is only ever Some after a successful connect and is
            // cleared on an explicit Disconnect, so this never fires for a port
            // that was never connected or was deliberately closed.
            if vtx.is_none() {
                if let Some(path) = vtx_port_path.clone() {
                    let is_lost = state.lock().unwrap().vtx_port_state == PortState::LostCommunication;
                    if is_lost && vtx_last_reconnect_attempt.elapsed() >= RECONNECT_INTERVAL {
                        vtx_last_reconnect_attempt = Instant::now();
                        match MspLink::open(&path, 115200) {
                            Ok(l) => {
                                debug!(target: "vtx", "reopened {path} after lost communication");
                                vtx = Some(l);
                                vtx_last_seen = None;
                                state.lock().unwrap().vtx_port_state = PortState::Ready;
                                if let Some(link) = vtx.as_mut() {
                                    let payload = calibration_engine::safe_state_payload(&vtx_table.lock().unwrap());
                                    if let Err(e) = link.send_v1(function::VTX_CONFIG as u8, &payload) {
                                        error!(target: "vtx", "failed to push safe-state VTX_CONFIG after reconnect: {e}");
                                    }
                                }
                            }
                            Err(e) => debug!(target: "vtx", "reconnect attempt for {path} failed: {e}"),
                        }
                    }
                }
            }

            // Advance the sweep by one step, if active. meter_ready here
            // reflects meter_last_seen as of the meter section's last run
            // (one tick stale at most, same as vtx_ready above) -- fine
            // for this purpose, same as how vtx_ready is used immediately
            // after being freshly computed earlier this same tick.
            let meter_ready = meter_last_seen.map(|t| t.elapsed() < READY_WINDOW).unwrap_or(false);
            if let Some(link) = vtx.as_mut() {
                let (history_snapshot, reading_seq) = {
                    let s = state.lock().unwrap();
                    (s.power_history.clone(), s.reading_seq)
                };
                let mut sweep_guard = sweep.lock().unwrap();
                if let Some(engine) = sweep_guard.as_mut() {
                    let was_active = engine.is_active();
                    if let Err(e) = engine.poll(link, &history_snapshot, reading_seq, pa_calibration_reading, vtx_ready, meter_ready) {
                        error!(target: "vtx", "sweep step failed: {e}");
                    }
                    if let Some(freq) = engine.pending_meter_frequency.take() {
                        if let Some(m) = meter.as_mut() {
                            match m.set_frequency(freq) {
                                Ok(()) => debug!(target: "meter", "set_frequency({freq}) requested"),
                                Err(e) => error!(target: "meter", "set_frequency failed: {e}"),
                            }
                        } else {
                            error!(target: "meter", "sweep requested set_frequency({freq}) but the power meter isn't connected");
                        }
                    }
                    if let Some(result) = engine.pending_result.take() {
                        if result.success {
                            let mut s = state.lock().unwrap();
                            if let Some(entry) = s.pa_table.iter_mut().find(|e| e.idx == result.level) {
                                if let Some(vbias_mv) = result.vbias_mv {
                                    if let Some(slot) = entry.value.get_mut(result.freq_idx) {
                                        *slot = vbias_mv;
                                    }
                                }
                                if let Some(det) = result.detector_mv {
                                    if let Some(slot) = entry.detector.get_mut(result.freq_idx) {
                                        *slot = det;
                                    }
                                }
                            }
                        }
                        debug!(target: "vtx", "sweep result: level={} freq_idx={} vbias={:?} det={:?} success={}",
                            result.level, result.freq_idx, result.vbias_mv, result.detector_mv, result.success);
                    }
                    if was_active && !engine.is_active() {
                        // Just transitioned to finished this tick (not aborted --
                        // AbortSweep restores update_hz itself). Only runs once,
                        // not every tick afterward -- the engine object persists
                        // after finishing, so gating on the transition (not just
                        // "currently inactive") is what avoids re-doing this
                        // every 10ms indefinitely.
                        let mut s = state.lock().unwrap();
                        if let Some(prev) = s.pre_sweep_update_hz.take() {
                            s.update_hz = prev;
                        }
                    }
                    ctx.request_repaint();
                }
            }

            // Power meter reading, rate-limited to the configured Hz
            // (clamped to the connected meter's max -- see Command::ConnectMeter
            // and pages/calibration.rs's dropdown). Outer loop tick is
            // 10ms so higher Hz settings (e.g. 20Hz = 50ms interval) are
            // actually achievable, not just theoretically requested. This
            // is NOT the connect/disconnect signal (see the separate V-based
            // check below) -- an unparseable reply here is usually just
            // meter noise, not proof the device is gone, so it only logs.
            if let Some(m) = meter.as_mut() {
                let interval = {
                    let hz = state.lock().unwrap().update_hz.max(0.01);
                    Duration::from_secs_f64(1.0 / hz)
                };
                if last_meter_read.elapsed() >= interval {
                    last_meter_read = Instant::now();
                    match m.read_dbm(Duration::from_millis(300)) {
                        Ok(raw_dbm) => {
                            let attenuation_db = state.lock().unwrap().attenuation_db;
                            let dbm = raw_dbm + attenuation_db;
                            let mw = 10f32.powf(dbm / 10.0);
                            debug!(target: "meter", "{raw_dbm:.2} dBm raw, {dbm:.2} dBm corrected (+{attenuation_db:.1}dB) ({mw:.6} mW)");
                            let elapsed = start.elapsed().as_secs_f64();
                            let mut s = state.lock().unwrap();
                            s.last_dbm = Some(dbm);
                            s.power_history.push_back((elapsed, mw));
                            s.reading_seq += 1;
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

            // Power meter presence check ("V"), independent of the D-based
            // reading above -- runs on its own ~100ms cadence regardless of
            // update_hz, since detecting connect/disconnect quickly matters
            // more here than the graph's sample rate. Distinguishes an
            // ordinary timeout (meter just didn't answer this cycle --
            // retried next cycle, not treated as disconnection) from a real
            // I/O error (port gone -- e.g. unplugged), which disconnects
            // immediately rather than spinning on a dead port.
            let mut meter_link_lost = false;
            if let Some(m) = meter.as_mut() {
                if last_meter_alive_check.elapsed() >= Duration::from_millis(100) {
                    last_meter_alive_check = Instant::now();
                    match m.check_alive(Duration::from_millis(300)) {
                        Ok(()) => meter_last_seen = Some(Instant::now()),
                        Err(e) => {
                            let is_timeout = e
                                .downcast_ref::<std::io::Error>()
                                .map(|io| io.kind() == std::io::ErrorKind::TimedOut)
                                .unwrap_or(false);
                            if is_timeout {
                                debug!(target: "meter", "alive-check timed out, retrying");
                            } else {
                                error!(target: "meter", "power meter link error, disconnecting: {e}");
                                meter_link_lost = true;
                            }
                        }
                    }
                }
            }
            if meter_link_lost {
                meter = None;
                meter_last_seen = None;
                state.lock().unwrap().meter_port_state = PortState::LostCommunication;
                if let Some(engine) = sweep.lock().unwrap().as_mut() {
                    engine.force_connection_lost(calibration_engine::ConnectionLossReason::Meter);
                }
            } else if meter.is_some() {
                let is_ready = meter_last_seen.map(|t| t.elapsed() < READY_WINDOW).unwrap_or(false);
                let mut s = state.lock().unwrap();
                if is_ready {
                    if s.meter_port_state != PortState::Ready {
                        s.meter_port_state = PortState::Ready;
                    }
                } else if s.meter_port_state == PortState::Ready {
                    // Same reasoning as the VTX side above: handle still open,
                    // just no traffic in a while -- shows as lost communication
                    // and auto-recovers to Ready the moment traffic resumes, no
                    // reopen needed since the handle itself was never broken.
                    s.meter_port_state = PortState::LostCommunication;
                }
            }

            // Periodic reopen while LostCommunication with the handle actually
            // dropped -- same reasoning as the VTX side above.
            if meter.is_none() {
                if let Some(path) = meter_port_path.clone() {
                    let (is_lost, kind) = {
                        let s = state.lock().unwrap();
                        (s.meter_port_state == PortState::LostCommunication, s.meter_kind)
                    };
                    if is_lost && meter_last_reconnect_attempt.elapsed() >= RECONNECT_INTERVAL {
                        meter_last_reconnect_attempt = Instant::now();
                        match PowerMeter::open(kind, &path) {
                            Ok(m) => {
                                debug!(target: "meter", "reopened {path} after lost communication");
                                meter = Some(m);
                                meter_last_seen = None;
                                state.lock().unwrap().meter_port_state = PortState::Ready;
                                last_meter_read = Instant::now() - Duration::from_secs(1);
                                last_meter_alive_check = Instant::now() - Duration::from_secs(1);
                            }
                            Err(e) => debug!(target: "meter", "reconnect attempt for {path} failed: {e}"),
                        }
                    }
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
