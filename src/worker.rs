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

/// How often to send an unprompted MSP_PACALIBRATION status query,
/// independent of whether a sweep is active -- see VtxStatus.
pub const VTX_STATUS_QUERY_INTERVAL: Duration = Duration::from_millis(250);

/// Live VTX status for the left panel's "VTX Status" section, built from
/// an MSP_PACALIBRATION reply -- polled continuously (see
/// VTX_STATUS_QUERY_INTERVAL), not just during a sweep. Every field here
/// is either read directly from the VTX's reply, or (power_mw) derived
/// from two VTX-reported facts (the reported level index, looked up
/// against the mW column of the calibration table also read from the
/// VTX) -- never guessed or computed client-side from anything else.
/// Optional fields are None (not defaulted to something that looks like
/// real data) when the connected firmware doesn't send the extended
/// payload these come from -- see msp::PaCalibrationReading.
#[derive(Debug, Clone)]
pub struct VtxStatus {
    pub level: u8,
    pub power_mw: Option<u16>,
    pub boost_on: Option<bool>,
    pub rtc6705_level: Option<u8>,
    pub frequency_mhz: Option<u16>,
    pub vbias_mv: u16,
    pub detector_mv: u16,
    pub pid_active: Option<bool>,
    /// Whether a calibration session is currently open on the VTX -- see
    /// calibration_engine.rs's session begin/end sends and
    /// rf_pa_calibration_session_begin()'s doc comment in the firmware's
    /// rf_pa.h for what that actually changes.
    pub session_active: Option<bool>,
    /// PA thermistor: raw 12-bit ADC code and the firmware's own
    /// conversion to degrees C -- see msp::PaCalibrationReading's own
    /// doc comment for why both are 0 (not None -- present, but 0) on a
    /// board with no NTC configured, once the firmware reports the
    /// extended payload these come from at all.
    pub ntc_raw: Option<u16>,
    pub pa_temp_c: Option<f32>,
}

pub type SharedSweep = Arc<Mutex<Option<SweepEngine>>>;

pub struct SharedState {
    pub pa_table: Vec<msp::PaCalibration>,
    pub last_dbm: Option<f32>,
    /// (elapsed_secs_since_worker_start, mW) pairs, pruned to the last
    /// HISTORY_WINDOW_SECS. elapsed-seconds rather than wall-clock time
    /// so the plot's x-axis is a simple, always-increasing float.
    pub power_history: VecDeque<(f64, f32)>,
    /// (elapsed_secs_since_worker_start, °C) pairs -- same time basis and
    /// pruning window as power_history, but populated from the VTX's own
    /// status replies (see msp::PaCalibrationReading::pa_temp_c) rather
    /// than the power meter, since it's a separate, asynchronous source
    /// arriving at its own rate. Only appended to when a reply actually
    /// carries a reading (firmware sends the extended payload) -- stays
    /// empty on an older firmware or a board with no NTC, rather than
    /// filling with 0s that would look like real readings.
    pub temp_history: VecDeque<(f64, f32)>,
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
    /// See VtxStatus's own doc comment. None until the first
    /// MSP_PACALIBRATION reply has actually arrived.
    pub vtx_status: Option<VtxStatus>,
    /// (columns, rows) as last reported by the VTX's own MSP_SET_OSD_CANVAS
    /// reply -- see msp_displayport_handle_msp()'s KEEPALIVE handling in
    /// the firmware, which sends this once per connection. None until
    /// that reply has actually arrived.
    pub osd_canvas: Option<(u8, u8)>,
    /// Wall-clock HH:MM:SS (UTC) of the most recent DisplayPort keepalive
    /// this tool sent -- None until the first one goes out. Updated every
    /// time a keepalive is sent, whether the initial one on connect or a
    /// periodic one.
    pub osd_keepalive_at: Option<String>,
    /// Wall-clock HH:MM:SS (UTC) of the most recent frame received from
    /// the VTX, of any kind -- None until the first one arrives. Distinct
    /// from vtx_ready (a derived "was it recent enough" bool): this is
    /// the raw timestamp itself, for the VTX Status panel.
    pub vtx_last_seen_at: Option<String>,
    /// Written directly by the UI's "Enable debug overlay" checkbox in
    /// the OSD Status section. While false, worker.rs sends NO
    /// MSP_DISPLAYPORT traffic at all -- no keepalive, no clear, no
    /// draw_string/draw_screen -- specifically so the firmware's own
    /// OSD content (e.g. debug_pa_loop()'s PID debug rows) can be
    /// observed with this tool's own overlay entirely out of the
    /// picture. Defaults true to match this tool's existing behavior.
    pub osd_debug_overlay_enabled: bool,
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
            temp_history: VecDeque::new(),
            reading_seq: 0,
            meter_kind: PowerMeterKind::default(),
            attenuation_db: 30.0, // matches settings.rs's default_attenuation_db(); main.rs overwrites this from AppSettings right after construction
            update_hz: 1.0, // conservative default; ImmersionRC V1's own max is now 5Hz, so this leaves headroom below it and stays valid for any future slower-max meter too
            pre_sweep_update_hz: None,
            vtx_config: None,
            vtx_port_state: PortState::Disconnected,
            meter_port_state: PortState::Disconnected,
            vtx_ready: false,
            vtx_status: None,
            osd_canvas: None,
            osd_keepalive_at: None,
            vtx_last_seen_at: None,
            osd_debug_overlay_enabled: true,
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
    /// Starts Manual mode -- see SweepEngine::start_manual(). Same
    /// `levels` meaning as StartSweep's; no tolerance since Manual mode
    /// doesn't do any automatic bracketing itself (the engine still
    /// needs one internally, in case Re-calibrate later resumes
    /// automatic scanning on this same instance -- see the handler).
    StartManual { levels: Vec<u8> },
    /// UI's DAC slider changed -- see SweepEngine::set_manual_dac().
    SetManualDac { mv: i32 },
    /// "Next" in Manual mode: commits the current slider value and the
    /// VTX's live detector reading as this cell's result, then advances
    /// -- see SweepEngine::manual_next().
    ManualNext,
    /// "Manual" pressed again to exit -- see SweepEngine::exit_manual().
    /// The in-progress (not yet "Next"-ed) point is left untouched.
    ExitManual,
    /// Manual mode's PA-enable checkbox -- see SweepEngine::set_pa_boost().
    SetPaBoost { on: bool },
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
    /// real level (idx >= 1) to the VTX via SET_PACALTABLE, which
    /// persists to EEPROM immediately as part of processing each entry
    /// (see vtx_msp_set_calibration_table()'s own rf_pa_write_eeprom()
    /// call in vtx_msp.c) -- there's no separate "commit" step needed
    /// after this.
    SendCalTableToVtx,
    /// Resets every level's calibration[]/detector[] on the VTX back to
    /// this target's own compiled-in defaults, and persists that
    /// immediately (see vtx_msp_set_calibration_table()'s 1-byte reset
    /// payload in vtx_msp.c -- this is NOT a value this tool invents;
    /// what's "safe/uncalibrated" depends on the board's PA_DAC_SIGN and
    /// only the firmware knows it).
    EraseCalibration,
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
        let mut last_status_query = Instant::now();
        // Periodic MSP-level TX/RX packet-count log -- see
        // MspLink::tx_rx_counts()'s own doc comment for why: a basic
        // "are commands reaching the VTX, are replies coming back at
        // all" diagnostic that doesn't depend on interpreting any
        // specific command's own behavior.
        let mut last_txrx_log = Instant::now();
        const TXRX_LOG_INTERVAL: Duration = Duration::from_secs(5);
        // DisplayPort debug overlay -- see build_status_displayport_frames()
        // and its call site below. Queued and drained one frame per tick
        // (same discipline as every other send in this file), rather than
        // firing all ~10 frames from one status update in a single burst --
        // DisplayPort commands use MspCommandKind::Other (zero settle), so
        // nothing else would naturally pace them.
        let mut displayport_queue: VecDeque<Vec<u8>> = VecDeque::new();
        let mut last_displayport_keepalive = Instant::now();
        // Chosen to comfortably beat any reasonable OSD-side keepalive
        // timeout without adding meaningful extra traffic -- this is a
        // low-frequency "still here" signal, not the actual debug content
        // (that's driven by every status reply, far more often than this).
        const DISPLAYPORT_KEEPALIVE_INTERVAL: Duration = Duration::from_secs(2);
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
                                        // Open the OSD debug overlay and start it from a
                                        // known-clean (cleared) screen -- see
                                        // build_status_displayport_frames()'s doc comment
                                        // for why this exists. Gated on the "Enable debug
                                        // overlay" checkbox -- while off, this tool sends
                                        // no MSP_DISPLAYPORT traffic at all, so the
                                        // firmware's own OSD content can be observed with
                                        // this tool's own overlay entirely out of the
                                        // picture.
                                        if state.lock().unwrap().osd_debug_overlay_enabled {
                                            if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &msp::encode_displayport_keepalive()) {
                                                error!(target: "vtx", "failed to send DisplayPort keepalive on connect: {e}");
                                            }
                                            if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &msp::encode_displayport_clear()) {
                                                error!(target: "vtx", "failed to clear DisplayPort screen on connect: {e}");
                                            }
                                            last_displayport_keepalive = Instant::now();
                                            let mut s = state.lock().unwrap();
                                            s.osd_keepalive_at = Some(format_time_hms());
                                            s.osd_canvas = None;
                                            drop(s);
                                            displayport_queue.clear();
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
                            if let Some(link) = vtx.as_mut() {
                                if state.lock().unwrap().osd_debug_overlay_enabled {
                                    if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &msp::encode_displayport_release()) {
                                        error!(target: "vtx", "failed to release DisplayPort on disconnect: {e}");
                                    }
                                }
                            }
                            displayport_queue.clear();
                            vtx = None;
                            vtx_last_seen = None;
                            vtx_port_path = None;
                            debug!(target: "vtx", "disconnected");
                            let mut s = state.lock().unwrap();
                            s.vtx_port_state = PortState::Disconnected;
                            s.vtx_ready = false;
                            s.vtx_status = None;
                            s.osd_canvas = None;
                            s.osd_keepalive_at = None;
                            s.vtx_last_seen_at = None;
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
                            // If Manual mode is currently active, "Re-calibrate"
                            // resumes automatic scanning from wherever it was
                            // sitting rather than starting over from scratch --
                            // see resume_automatic_from_current()'s own doc
                            // comment (every point Manual mode already
                            // committed via "Next" stays as its Manual result).
                            let resumed = {
                                let mut guard = sweep.lock().unwrap();
                                match guard.as_mut() {
                                    Some(engine) if matches!(&engine.state, calibration_engine::EngineState::Manual) => {
                                        engine.resume_automatic_from_current();
                                        true
                                    }
                                    _ => false,
                                }
                            };
                            if resumed {
                                debug!(target: "vtx", "resumed automatic calibration from current manual position");
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
                    }

                    Command::StartManual { levels } => {
                        let (vtx_state_now, meter_state_now) = {
                            let s = state.lock().unwrap();
                            (s.vtx_port_state, s.meter_port_state)
                        };
                        if vtx_state_now != PortState::Ready || meter_state_now != PortState::Ready {
                            error!(target: "vtx", "StartManual requested while not fully connected (vtx={:?} meter={:?})",
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
                                error!(target: "vtx", "StartManual: no frequency breakpoints in the PA table -- Refresh it first");
                            } else if levels.is_empty() {
                                error!(target: "vtx", "StartManual: no power levels selected");
                            } else {
                                // tolerance_pct is irrelevant to Manual mode itself,
                                // but the engine needs one anyway in case Re-calibrate
                                // later resumes automatic scanning on this same
                                // instance -- 10% matches automatic mode's own default.
                                let mut engine = SweepEngine::new(
                                    levels,
                                    frequencies,
                                    10.0,
                                    sign_inverted,
                                    target_mw_by_level,
                                    meter_kind.max_update_hz(),
                                );
                                engine.start_manual(meter_kind.capability());
                                // Seed the slider from whatever's already stored for
                                // the first (level, freq) cell, so Manual mode starts
                                // from the existing calibration rather than a blind
                                // default -- see set_manual_dac()'s own doc comment.
                                if let Some(&level) = engine.levels.first() {
                                    if let Some(entry) = pa_table.iter().find(|e| e.idx == level) {
                                        if let Some(&mv) = entry.value.first() {
                                            engine.set_manual_dac(mv as i32);
                                        }
                                    }
                                }
                                let sweep_hz = engine.sweep_hz;
                                debug!(target: "vtx", "manual mode started: {} levels, {} frequencies",
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

                    Command::SetManualDac { mv } => {
                        if let Some(engine) = sweep.lock().unwrap().as_mut() {
                            engine.set_manual_dac(mv);
                        }
                    }

                    Command::ManualNext => {
                        let detector_mv = {
                            let s = state.lock().unwrap();
                            s.vtx_status.as_ref().map(|v| v.detector_mv).unwrap_or(0)
                        };
                        let next_pos = {
                            let mut guard = sweep.lock().unwrap();
                            guard.as_mut().and_then(|engine| engine.manual_next(detector_mv))
                        };
                        // Seed the slider for the new cell from its existing table
                        // value, same as StartManual's own initial seed -- otherwise
                        // it'd stay wherever the previous cell's slider was left,
                        // which is very likely wrong for a different level/frequency.
                        if let Some((level, freq_idx)) = next_pos {
                            let pa_table = state.lock().unwrap().pa_table.clone();
                            if let Some(entry) = pa_table.iter().find(|e| e.idx == level) {
                                if let Some(&mv) = entry.value.get(freq_idx) {
                                    if let Some(engine) = sweep.lock().unwrap().as_mut() {
                                        engine.set_manual_dac(mv as i32);
                                    }
                                }
                            }
                        }
                    }

                    Command::ExitManual => {
                        if let Some(engine) = sweep.lock().unwrap().as_mut() {
                            engine.exit_manual();
                        }
                        let mut s = state.lock().unwrap();
                        if let Some(prev) = s.pre_sweep_update_hz.take() {
                            s.update_hz = prev;
                        }
                    }

                    Command::SetPaBoost { on } => {
                        if let Some(engine) = sweep.lock().unwrap().as_mut() {
                            engine.set_pa_boost(on);
                        }
                    }

                    Command::ConfirmFrequency => {
                        if let Some(engine) = sweep.lock().unwrap().as_mut() {
                            engine.confirm_frequency();
                            debug!(target: "vtx", "frequency change confirmed by user");
                        }
                    }

                    Command::AbortSweep => {
                        // No direct sends here anymore -- only the engine sends
                        // commands (see pending_sends' doc comment on
                        // SweepEngine). abort() computes its own pitmode-forced
                        // payload internally now, from the engine's own current
                        // (level, freq) -- see its own doc comment for why this
                        // no longer goes through vtx_table -- and queues it (plus
                        // a session-end right behind it) for poll()'s own next
                        // eligible tick to actually issue, through the exact same
                        // link.can_send_now() gate as every other send.
                        if let Some(engine) = sweep.lock().unwrap().as_mut() {
                            engine.abort();
                        }
                        let mut s = state.lock().unwrap();
                        if let Some(prev) = s.pre_sweep_update_hz.take() {
                            s.update_hz = prev;
                        }
                    }
                    Command::SkipCurrent => {
                        // No direct send here anymore -- only the engine sends
                        // commands. skip_current() computes its own pitmode-
                        // forced payload internally now, from the engine's own
                        // current (level, freq) -- see its own doc comment for
                        // why this no longer goes through vtx_table -- and
                        // queues it for poll()'s own next eligible tick to
                        // issue, same gated mechanism as every other send in
                        // this file -- so it's structurally impossible for this
                        // to land ahead of, or without, the settle window a
                        // retune needs (which is exactly what happened when this
                        // sent directly: a VTX_CONFIG push landing immediately,
                        // with no wait, before the settle gate the engine itself
                        // was tracking ever got a say).
                        let next_pos = {
                            let mut guard = sweep.lock().unwrap();
                            guard.as_mut().and_then(|engine| engine.skip_current())
                        };
                        // Only Some() when Manual mode's own skip advanced the
                        // position -- reseed the slider from the new cell's
                        // existing table value, same as ManualNext does.
                        if let Some((level, freq_idx)) = next_pos {
                            let pa_table = state.lock().unwrap().pa_table.clone();
                            if let Some(entry) = pa_table.iter().find(|e| e.idx == level) {
                                if let Some(&mv) = entry.value.get(freq_idx) {
                                    if let Some(engine) = sweep.lock().unwrap().as_mut() {
                                        engine.set_manual_dac(mv as i32);
                                    }
                                }
                            }
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

                    Command::EraseCalibration => {
                        if let Some(link) = vtx.as_mut() {
                            // See vtx_msp_set_calibration_table()'s own doc
                            // comment in vtx_msp.c -- a 1-byte payload of
                            // exactly 0xFF resets every level back to this
                            // target's own compiled-in defaults and persists
                            // immediately. NOT a value this tool invents.
                            match link.send_v2(function::SET_PACALTABLE, Some(&[0xFFu8])) {
                                Ok(_) => {
                                    debug!(target: "vtx", "sent calibration reset-to-defaults");
                                    match read_pa_table(link) {
                                        Ok(table) => {
                                            debug!(target: "vtx", "PA table refreshed after erase: {} entries", table.len());
                                            state.lock().unwrap().pa_table = table;
                                            if let Some(engine) = sweep.lock().unwrap().as_mut() {
                                                engine.clear_hard_limits();
                                                engine.clear_cell_status();
                                            }
                                        }
                                        Err(e) => error!(target: "vtx", "PA table read after erase failed: {e}"),
                                    }
                                }
                                Err(e) => error!(target: "vtx", "failed to send calibration reset: {e}"),
                            }
                        } else {
                            error!(target: "vtx", "EraseCalibration requested while disconnected");
                        }
                    }
                }
                ctx.request_repaint();
            }

            // Unprompted periodic status query -- see VtxStatus/
            // VTX_STATUS_QUERY_INTERVAL. Independent of any active sweep
            // (which also uses MSP_PACALIBRATION, just as a reply to ITS
            // OWN SET_PACALIBRATION commands) -- both are handled
            // uniformly in the read-dispatch below regardless of which
            // caller's request prompted a given reply. Skips silently
            // (not an error) when the link is still settling from a
            // previous command -- that's routine during an active sweep,
            // not a failure.
            if let Some(link) = vtx.as_mut() {
                if last_status_query.elapsed() >= VTX_STATUS_QUERY_INTERVAL && link.can_send_now() {
                    last_status_query = Instant::now();
                    if let Err(e) = link.send_v2(function::PACALIBRATION, None) {
                        error!(target: "vtx", "failed to send status query: {e}");
                    }
                }
                if last_txrx_log.elapsed() >= TXRX_LOG_INTERVAL {
                    last_txrx_log = Instant::now();
                    let (tx, rx) = link.tx_rx_counts();
                    debug!(target: "vtx", "MSP link: tx={tx} rx={rx}");
                }
                let osd_debug_overlay_enabled = state.lock().unwrap().osd_debug_overlay_enabled;
                if osd_debug_overlay_enabled {
                    if last_displayport_keepalive.elapsed() >= DISPLAYPORT_KEEPALIVE_INTERVAL && link.can_send_now() {
                        last_displayport_keepalive = Instant::now();
                        if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &msp::encode_displayport_keepalive()) {
                            error!(target: "vtx", "failed to send DisplayPort keepalive: {e}");
                        }
                        state.lock().unwrap().osd_keepalive_at = Some(format_time_hms());
                    }
                    // Whole batch in one go, not paced one frame per tick --
                    // checked both sides before removing the old pacing:
                    // DRAW_STRING/DRAW_SCREEN never call note_sent() (they
                    // don't extend the settle gate themselves, so nothing
                    // here was ever waiting on THEM specifically), and the
                    // firmware's 1024-byte UART RX ring buffer comfortably
                    // absorbs a full ~300-350 byte batch arriving as one
                    // burst before msp_loop_process() next drains it, with
                    // nothing slow or blocking in msp_displayport_handle_msp()'s
                    // own per-message work either. Still checked before
                    // every single send so this never jumps ahead of a
                    // genuine pending retune settle window from something
                    // else -- if that gate closes mid-batch, whatever's
                    // left just waits for the next tick, same as before.
                    while link.can_send_now() {
                        let Some(frame) = displayport_queue.pop_front() else { break };
                        if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &frame) {
                            error!(target: "vtx", "failed to send DisplayPort frame: {e}");
                            break;
                        }
                    }
                } else if !displayport_queue.is_empty() {
                    // Disabled mid-batch (or just toggled off) -- drop whatever was
                    // queued rather than let it drain out later once re-enabled,
                    // showing stale content from before the toggle.
                    displayport_queue.clear();
                }
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
                        state.lock().unwrap().vtx_last_seen_at = Some(format_time_hms());
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
                        } else if frame.function == function::VTX_CONFIG && !frame.payload.is_empty() {
                            // The VTX's reply to a query we queued (see
                            // calibration_engine::PendingSend::RequestVtxConfig's
                            // own doc comment -- queued after every
                            // frequency/pitmode push, sent by the same
                            // firmware path worker.rs's own startup-time
                            // read_vtx_config() already uses, just
                            // non-blocking here). This is what updates the
                            // Frequency panel's UI -- from the VTX's own
                            // reported state, not from what we assumed our
                            // push did, since the VTX may reject, ignore,
                            // or otherwise not apply it.
                            match msp::decode_vtx_config(&frame.payload) {
                                Ok(cfg) => {
                                    let mut vt = vtx_table.lock().unwrap();
                                    vt.selected_band = cfg.band;
                                    vt.selected_channel = cfg.channel;
                                    vt.selected_power = cfg.power;
                                    vt.selected_freq_mhz = cfg.frequency_mhz;
                                    vt.pitmode = cfg.pitmode;
                                    drop(vt);
                                    debug!(target: "vtx", "VTX_CONFIG reply: band={} channel={} power={} freq={}MHz pitmode={}",
                                        cfg.band, cfg.channel, cfg.power, cfg.frequency_mhz, cfg.pitmode);
                                    ctx.request_repaint();
                                }
                                Err(e) => error!(target: "vtx", "failed to decode VTX_CONFIG reply: {e}"),
                            }
                        } else if frame.function == function::SET_OSD_CANVAS {
                            if let Some(canvas) = msp::decode_osd_canvas(&frame.payload) {
                                state.lock().unwrap().osd_canvas = Some(canvas);
                                debug!(target: "vtx", "OSD canvas: {}x{}", canvas.0, canvas.1);
                            }
                        } else if frame.function == function::PACALIBRATION {
                            if let Ok(reading) = msp::decode_pa_calibration_reading(&frame.payload) {
                                let mut s = state.lock().unwrap();
                                let power_mw =
                                    s.pa_table.iter().find(|e| e.idx == reading.power_level).map(|e| e.m_w);
                                let status = VtxStatus {
                                    level: reading.power_level,
                                    power_mw,
                                    boost_on: reading.boost_on,
                                    rtc6705_level: reading.rtc6705_level,
                                    frequency_mhz: reading.frequency_mhz,
                                    vbias_mv: reading.vref_mv,
                                    detector_mv: reading.detector_mv,
                                    pid_active: reading.pid_active,
                                    session_active: reading.session_active,
                                    ntc_raw: reading.ntc_raw,
                                    pa_temp_c: reading.pa_temp_c,
                                };
                                s.vtx_status = Some(status.clone());
                                if let Some(temp_c) = reading.pa_temp_c {
                                    let elapsed = start.elapsed().as_secs_f64();
                                    s.temp_history.push_back((elapsed, temp_c));
                                    while let Some(&(t, _)) = s.temp_history.front() {
                                        if elapsed - t > HISTORY_WINDOW_SECS {
                                            s.temp_history.pop_front();
                                        } else {
                                            break;
                                        }
                                    }
                                }
                                let osd_debug_overlay_enabled = s.osd_debug_overlay_enabled;
                                drop(s);
                                // Only start a fresh batch once the previous one has
                                // fully drained (ending in its own DRAW_SCREEN) --
                                // replacing mid-batch (the previous behavior) could
                                // abandon a batch before DRAW_SCREEN ever went out,
                                // leaving whichever rows hadn't been overwritten yet
                                // showing stale content indefinitely, or -- if this
                                // kept happening every single status reply -- no
                                // DRAW_SCREEN ever landing at all. This sends less
                                // often than every status reply whenever the link is
                                // busy, but every batch that DOES go out is complete.
                                // Gated on "Enable debug overlay" too -- while off,
                                // nothing gets queued at all, so worker.rs's own drain
                                // loop below never has anything of this tool's own to
                                // send.
                                if osd_debug_overlay_enabled && displayport_queue.is_empty() {
                                    displayport_queue.push_back(msp::encode_displayport_clear());
                                    displayport_queue.extend(build_status_displayport_frames(&status));
                                }
                                // Always logged now -- was previously gated to only
                                // the first reply after a state-changing send, to
                                // avoid filling the log at VTX_STATUS_QUERY_INTERVAL.
                                // Logging every reply unconditionally instead, since
                                // a complete, continuous status trace is what's
                                // actually needed while tracking down the VTX
                                // becoming unresponsive.
                                debug!(target: "vtx", "status: level={} power_mw={:?} boost_on={:?} rtc6705_level={:?} freq_mhz={:?} vbias_mv={} detector_mv={} pid_active={:?} session_active={:?} ntc_raw={:?} pa_temp_c={:?}",
                                    reading.power_level, power_mw, reading.boost_on, reading.rtc6705_level,
                                    reading.frequency_mhz, reading.vref_mv, reading.detector_mv,
                                    reading.pid_active, reading.session_active, reading.ntc_raw, reading.pa_temp_c);
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
                                    // Same "open + clear" as the initial connect --
                                    // this is specifically the "reconnecting after
                                    // power failure" case. Gated on the "Enable debug
                                    // overlay" checkbox, same as every other DisplayPort
                                    // send site.
                                    if state.lock().unwrap().osd_debug_overlay_enabled {
                                        if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &msp::encode_displayport_keepalive()) {
                                            error!(target: "vtx", "failed to send DisplayPort keepalive after reconnect: {e}");
                                        }
                                        if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &msp::encode_displayport_clear()) {
                                            error!(target: "vtx", "failed to clear DisplayPort screen after reconnect: {e}");
                                        }
                                        last_displayport_keepalive = Instant::now();
                                        let mut s = state.lock().unwrap();
                                        s.osd_keepalive_at = Some(format_time_hms());
                                        s.osd_canvas = None;
                                        drop(s);
                                        displayport_queue.clear();
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
                    match engine.poll(link, &history_snapshot, reading_seq, pa_calibration_reading, vtx_ready, meter_ready) {
                        Ok(_sent) => {}
                        Err(e) => error!(target: "vtx", "sweep step failed: {e}"),
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
                        debug!(target: "vtx", "sweep result: level={} freq_idx={} vbias={:?} det={:?} success={} pa_failure={} not_settled={}",
                            result.level, result.freq_idx, result.vbias_mv, result.detector_mv, result.success, result.pa_failure, result.not_settled);
                    }
                    if was_active && !engine.is_active() {
                        // Just transitioned to finished this tick (not aborted --
                        // AbortSweep restores update_hz itself, and abort() queues
                        // its own session-close push). poll() itself already queued
                        // one the moment it detected "all frequencies complete" (see
                        // PendingSend::CalibrationState there) -- nothing to send
                        // here, just the update_hz restore. Only runs once, not
                        // every tick afterward -- the engine object persists after
                        // finishing, so gating on the transition (not just
                        // "currently inactive") is what avoids re-doing this every
                        // 10ms indefinitely.
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

/// Builds the DisplayPort debug overlay for one VTX status reply: nine
/// label/value rows (Level, Power mW, PA, RTC6705, Freq, VBIAS, Vdet,
/// PID, Session) followed by a DRAW_SCREEN to commit them, each row a
/// single DRAW_STRING starting at column 0 -- the label/value split and
/// justification are computed here in Rust (trivial with format!'s own
/// padding) rather than asking the firmware's single "write chars at
/// (row,col)" primitive to do any layout of its own. See the "Check the
/// C MSP and VTX code" investigation this was built alongside: this
/// overlay exists so a hung MCU is visible on the OSD itself (frozen
/// text, stopped LED) as a check independent of whatever the serial
/// link is or isn't reporting.
fn build_status_displayport_frames(status: &VtxStatus) -> Vec<Vec<u8>> {
    // Label column left-justified at 10 chars, value column right-
    // justified at 18 -- 28 total. Not the full 30-column canvas: row 0
    // and column 0 are never written (some OSD overlays clip or corrupt
    // the very first row/column), and neither is the last column (29),
    // leaving a one-column margin on both sides -- see the DRAW_STRING
    // call below, which starts at (row+1, 1).
    fn row(label: &str, value: String) -> String {
        format!("{label:<10}{value:>18}").to_uppercase() // the OSD font has no lowercase glyphs
    }
    let rows = [
        row("Level", status.level.to_string()),
        row("Power mW", status.power_mw.map(|v| format!("{v}mW")).unwrap_or_else(|| "-".to_string())),
        row("PA", status.boost_on.map(|b| if b { "ON" } else { "OFF" }.to_string()).unwrap_or_else(|| "?".to_string())),
        row("RTC6705", status.rtc6705_level.map(|v| v.to_string()).unwrap_or_else(|| "-".to_string())),
        row("Freq", status.frequency_mhz.map(|v| format!("{v}MHz")).unwrap_or_else(|| "-".to_string())),
        row("VBIAS", format!("{}mV", status.vbias_mv)),
        row("Vdet", status.detector_mv.to_string()),
        row("PID", status.pid_active.map(|b| if b { "Active" } else { "Idle" }.to_string()).unwrap_or_else(|| "?".to_string())),
        row("Session", status.session_active.map(|b| if b { "Open" } else { "Closed" }.to_string()).unwrap_or_else(|| "?".to_string())),
    ];

    let mut frames: Vec<Vec<u8>> = rows
        .iter()
        .enumerate()
        .map(|(i, text)| msp::encode_displayport_draw_string((i + 1) as u8, 1, text))
        .collect();
    frames.push(msp::encode_displayport_draw_screen());
    frames
}

/// Wall-clock HH:MM:SS (UTC) -- used for the OSD status panel's
/// keepalive/last-seen timestamps. Deliberately not chrono (not already
/// a dependency) -- std::time::SystemTime is enough for this.
fn format_time_hms() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let (h, m, s) = ((secs / 3600) % 24, (secs / 60) % 60, secs % 60);
    format!("{h:02}:{m:02}:{s:02}")
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
