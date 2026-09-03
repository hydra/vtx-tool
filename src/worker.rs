
use crate::calibration_engine::{self, SweepEngine};
use crate::conn_status::PortState;
use crate::msp::{self, function, MspLink};
use crate::power_meter::{PowerMeter, PowerMeterKind};
use crate::state::{SharedHandles, VtxStatus};
use crate::vtxtable::{VtxSelectionState, VtxTableConfig};
use anyhow::Result;
use log::{debug, error};
use std::collections::VecDeque;
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub const HISTORY_WINDOW_SECS: f64 = 60.0;

pub const VTX_STATUS_QUERY_INTERVAL: Duration = Duration::from_millis(250);

pub type SharedSweep = Arc<Mutex<Option<SweepEngine>>>;

const READY_WINDOW: Duration = Duration::from_millis(500);
const RECONNECT_INTERVAL: Duration = Duration::from_secs(1);

pub enum Command {
    ConnectVtx { port: String },
    DisconnectVtx,
    ConnectMeter { port: String, meter_kind: PowerMeterKind },
    DisconnectMeter,
    RefreshCalTable,
    RefreshVtxConfig,
    PushVtxConfig,
    StartSweep { levels: Vec<u8>, tolerance_pct: f32 },
    StartManual { levels: Vec<u8> },
    SetManualDac { mv: i32 },
    ManualNext,
    ExitManual,
    SetPaBoost { on: bool },
    ConfirmFrequency,
    AbortSweep,
    SkipMultiple { count: u32 },
    SendCalTableToVtx,
    EraseCalibration,
}

pub fn spawn(
    shared: SharedHandles,
    vtx_table: Arc<Mutex<VtxTableConfig>>,
    vtx_selection: Arc<Mutex<VtxSelectionState>>,
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
        let mut meter_last_seen: Option<Instant> = None;
        let mut last_meter_alive_check = Instant::now();
        let mut vtx_port_path: Option<String> = None;
        let mut meter_port_path: Option<String> = None;
        let mut vtx_last_reconnect_attempt = Instant::now();
        let mut meter_last_reconnect_attempt = Instant::now();
        let mut last_status_query = Instant::now();
        let mut last_txrx_log = Instant::now();
        const TXRX_LOG_INTERVAL: Duration = Duration::from_secs(5);
        let mut displayport_queue: VecDeque<Vec<u8>> = VecDeque::new();
        let mut last_displayport_keepalive = Instant::now();
        const DISPLAYPORT_KEEPALIVE_INTERVAL: Duration = Duration::from_secs(2);
        let mut last_history_trim = Instant::now();
        const HISTORY_TRIM_INTERVAL: Duration = Duration::from_secs(1);
        loop {
            while let Ok(cmd) = cmd_rx.try_recv() {
                match cmd {
                    Command::ConnectVtx { port } => {
                        if vtx.is_some() {
                            debug!(target: "vtx", "ConnectVtx requested but already connected -- ignoring");
                        } else {
                            shared.vtx.lock().unwrap().port_state = PortState::Connecting;
                            ctx.request_repaint();
                            match MspLink::open(&port, 115200) {
                                Ok(l) => {
                                    debug!(target: "vtx", "opened {port}");
                                    vtx = Some(l);
                                    vtx_last_seen = None;
                                    vtx_port_path = Some(port.clone());
                                    shared.vtx.lock().unwrap().port_state = PortState::Ready;

                                    if let Some(link) = vtx.as_mut() {
                                        let payload = calibration_engine::safe_state_payload(&vtx_table.lock().unwrap(), &vtx_selection.lock().unwrap());
                                        match link.send_v1(function::VTX_CONFIG as u8, &payload) {
                                            Ok(()) => debug!(target: "vtx", "pushed pitmode-safe VTX_CONFIG on connect"),
                                            Err(e) => error!(target: "vtx", "failed to push safe-state VTX_CONFIG on connect: {e}"),
                                        }
                                        if shared.osd.lock().unwrap().debug_overlay_enabled {
                                            if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &msp::encode_displayport_keepalive()) {
                                                error!(target: "vtx", "failed to send DisplayPort keepalive on connect: {e}");
                                            }
                                            if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &msp::encode_displayport_clear()) {
                                                error!(target: "vtx", "failed to clear DisplayPort screen on connect: {e}");
                                            }
                                            last_displayport_keepalive = Instant::now();
                                            {
                                                let mut osd = shared.osd.lock().unwrap();
                                                osd.keepalive_at = Some(format_time_hms());
                                                osd.canvas = None;
                                            }
                                            displayport_queue.clear();
                                        }
                                    }
                                }
                                Err(e) => {
                                    error!(target: "vtx", "open failed: {e}");
                                    shared.vtx.lock().unwrap().port_state = PortState::Disconnected;
                                }
                            }
                        }
                    }

                    Command::DisconnectVtx => {
                        if vtx.is_none() {
                            debug!(target: "vtx", "DisconnectVtx requested but not connected -- ignoring");
                        } else {
                            shared.vtx.lock().unwrap().port_state = PortState::Disconnecting;
                            ctx.request_repaint();
                            if let Some(link) = vtx.as_mut() {
                                if shared.osd.lock().unwrap().debug_overlay_enabled {
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
                            {
                                let mut vtx_state = shared.vtx.lock().unwrap();
                                vtx_state.port_state = PortState::Disconnected;
                                vtx_state.ready = false;
                                vtx_state.status = None;
                                vtx_state.last_seen_at = None;
                            }
                            {
                                let mut osd = shared.osd.lock().unwrap();
                                osd.canvas = None;
                                osd.keepalive_at = None;
                            }
                            *sweep.lock().unwrap() = None;
                        }
                    }

                    Command::ConnectMeter { port, meter_kind } => {
                        if meter.is_some() {
                            debug!(target: "meter", "ConnectMeter requested but already connected -- ignoring");
                        } else {
                            {
                                let mut meter_state = shared.meter.lock().unwrap();
                                meter_state.port_state = PortState::Connecting;
                                meter_state.kind = meter_kind;
                                meter_state.update_hz =
                                    meter_state.update_hz.min(meter_kind.max_update_hz() as f64).max(0.01);
                                meter_state.power_history.clear();
                            }
                            ctx.request_repaint();
                            match PowerMeter::open(meter_kind, &port) {
                                Ok(m) => {
                                    debug!(target: "meter", "opened {port} ({})", meter_kind.name());
                                    meter = Some(m);
                                    meter_last_seen = None;
                                    meter_port_path = Some(port.clone());
                                    shared.meter.lock().unwrap().port_state = PortState::Ready;
                                    last_meter_read = Instant::now() - Duration::from_secs(1);
                                    last_meter_alive_check = Instant::now() - Duration::from_secs(1);
                                }
                                Err(e) => {
                                    error!(target: "meter", "open failed: {e}");
                                    shared.meter.lock().unwrap().port_state = PortState::Disconnected;
                                }
                            }
                        }
                    }

                    Command::DisconnectMeter => {
                        if meter.is_none() {
                            debug!(target: "meter", "DisconnectMeter requested but not connected -- ignoring");
                        } else {
                            shared.meter.lock().unwrap().port_state = PortState::Disconnecting;
                            ctx.request_repaint();
                            meter = None;
                            meter_last_seen = None;
                            meter_port_path = None;
                            debug!(target: "meter", "disconnected");
                            shared.meter.lock().unwrap().port_state = PortState::Disconnected;
                        }
                    }

                    Command::RefreshCalTable => {
                        if let Some(link) = vtx.as_mut() {
                            match read_pa_table(link) {
                                Ok(table) => {
                                    debug!(target: "vtx", "PA table refreshed: {} entries", table.len());
                                    shared.vtx.lock().unwrap().pa_table = table;
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
                                    shared.vtx.lock().unwrap().config = Some(cfg);
                                }
                                Err(e) => error!(target: "vtx", "config read failed: {e}"),
                            }
                        } else {
                            error!(target: "vtx", "RefreshVtxConfig requested while disconnected");
                        }
                    }

                    Command::PushVtxConfig => {
                        if let Some(link) = vtx.as_mut() {
                            let table = vtx_table.lock().unwrap();
                            let payload = vtx_selection.lock().unwrap().encode_vtx_config_response(&table);
                            drop(table);
                            match link.send_v1(function::VTX_CONFIG as u8, &payload) {
                                Ok(()) => debug!(target: "vtx", "pushed VTX_CONFIG (Save)"),
                                Err(e) => error!(target: "vtx", "failed to push VTX_CONFIG: {e}"),
                            }
                        } else {
                            error!(target: "vtx", "PushVtxConfig requested while disconnected");
                        }
                    }

                    Command::StartSweep { levels, tolerance_pct } => {
                        let (vtx_state_now, meter_state_now) = shared.port_states();
                        if vtx_state_now != PortState::Ready || meter_state_now != PortState::Ready {
                            error!(target: "vtx", "StartSweep requested while not fully connected (vtx={:?} meter={:?})",
                                vtx_state_now, meter_state_now);
                        } else {
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
                            let pa_table = shared.vtx.lock().unwrap().pa_table.clone();
                            let (meter_kind, prev_update_hz) = {
                                let meter_state = shared.meter.lock().unwrap();
                                (meter_state.kind, meter_state.update_hz)
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
                                    let mut meter_state = shared.meter.lock().unwrap();
                                    meter_state.pre_sweep_update_hz = Some(prev_update_hz);
                                    meter_state.update_hz = sweep_hz;
                                }
                                *sweep.lock().unwrap() = Some(engine);
                            }
                            }
                        }
                    }

                    Command::StartManual { levels } => {
                        let (vtx_state_now, meter_state_now) = shared.port_states();
                        if vtx_state_now != PortState::Ready || meter_state_now != PortState::Ready {
                            error!(target: "vtx", "StartManual requested while not fully connected (vtx={:?} meter={:?})",
                                vtx_state_now, meter_state_now);
                        } else {
                            let pa_table = shared.vtx.lock().unwrap().pa_table.clone();
                            let (meter_kind, prev_update_hz) = {
                                let meter_state = shared.meter.lock().unwrap();
                                (meter_state.kind, meter_state.update_hz)
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
                                let mut engine = SweepEngine::new(
                                    levels,
                                    frequencies,
                                    10.0,
                                    sign_inverted,
                                    target_mw_by_level,
                                    meter_kind.max_update_hz(),
                                );
                                engine.start_manual(meter_kind.capability());
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
                                    let mut meter_state = shared.meter.lock().unwrap();
                                    meter_state.pre_sweep_update_hz = Some(prev_update_hz);
                                    meter_state.update_hz = sweep_hz;
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
                            shared.vtx.lock().unwrap().status.as_ref().map(|v| v.detector_mv).unwrap_or(0)
                        };
                        let next_pos = {
                            let mut guard = sweep.lock().unwrap();
                            guard.as_mut().and_then(|engine| engine.manual_next(detector_mv))
                        };
                        if let Some((level, freq_idx)) = next_pos {
                            let pa_table = shared.vtx.lock().unwrap().pa_table.clone();
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
                        let mut meter_state = shared.meter.lock().unwrap();
                        if let Some(prev) = meter_state.pre_sweep_update_hz.take() {
                            meter_state.update_hz = prev;
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
                        if let Some(engine) = sweep.lock().unwrap().as_mut() {
                            engine.abort();
                        }
                        let mut meter_state = shared.meter.lock().unwrap();
                        if let Some(prev) = meter_state.pre_sweep_update_hz.take() {
                            meter_state.update_hz = prev;
                        }
                    }
                    Command::SkipMultiple { count } => {
                        let next_pos = {
                            let mut guard = sweep.lock().unwrap();
                            guard.as_mut().and_then(|engine| engine.skip_multiple(count))
                        };
                        if let Some((level, freq_idx)) = next_pos {
                            let pa_table = shared.vtx.lock().unwrap().pa_table.clone();
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
                            let pa_table = shared.vtx.lock().unwrap().pa_table.clone();
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
                            match link.send_v2(function::SET_PACALTABLE, Some(&[0xFFu8])) {
                                Ok(_) => {
                                    debug!(target: "vtx", "sent calibration reset-to-defaults");
                                    match read_pa_table(link) {
                                        Ok(table) => {
                                            debug!(target: "vtx", "PA table refreshed after erase: {} entries", table.len());
                                            shared.vtx.lock().unwrap().pa_table = table;
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

            if last_history_trim.elapsed() >= HISTORY_TRIM_INTERVAL {
                last_history_trim = Instant::now();
                let now = start.elapsed().as_secs_f64();
                let (vtx_port, meter_port) = shared.port_states();
                let both_disconnected = vtx_port.is_idle() && meter_port.is_idle();
                if !both_disconnected {
                    trim_stale_history(&mut shared.meter.lock().unwrap().power_history, now);
                    let mut vtx_state = shared.vtx.lock().unwrap();
                    trim_stale_history(&mut vtx_state.pa_temp_history, now);
                    trim_stale_history(&mut vtx_state.mcu_temp_history, now);
                }
                ctx.request_repaint();
            }

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
                let osd_debug_overlay_enabled = shared.osd.lock().unwrap().debug_overlay_enabled;
                if osd_debug_overlay_enabled {
                    if last_displayport_keepalive.elapsed() >= DISPLAYPORT_KEEPALIVE_INTERVAL && link.can_send_now() {
                        last_displayport_keepalive = Instant::now();
                        if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &msp::encode_displayport_keepalive()) {
                            error!(target: "vtx", "failed to send DisplayPort keepalive: {e}");
                        }
                        shared.osd.lock().unwrap().keepalive_at = Some(format_time_hms());
                    }
                    while link.can_send_now() {
                        let Some(frame) = displayport_queue.pop_front() else { break };
                        if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &frame) {
                            error!(target: "vtx", "failed to send DisplayPort frame: {e}");
                            break;
                        }
                    }
                } else if !displayport_queue.is_empty() {
                    displayport_queue.clear();
                }
            }

            let mut pa_calibration_reading: Option<msp::PaCalibrationReading> = None;
            let mut vtx_link_lost = false;
            if let Some(link) = vtx.as_mut() {
                match link.read_frame(Duration::from_millis(20)) {
                    Ok(Some(frame)) => {
                        vtx_last_seen = Some(Instant::now());
                        shared.vtx.lock().unwrap().last_seen_at = Some(format_time_hms());
                        if frame.function == function::VTX_CONFIG && frame.payload.is_empty() {
                            let table = vtx_table.lock().unwrap();
                            let table_empty = table.bands.is_empty() || table.power_levels.is_empty();
                            let ready = shared.table_sync.lock().unwrap().ready;
                            if table_empty || !ready {
                                debug!(target: "vtx", "VTX_CONFIG query received but not replying (table_empty={table_empty}, ready={ready})");
                                drop(table);
                            } else {
                                let response = vtx_selection.lock().unwrap().encode_vtx_config_response(&table);
                                drop(table);
                                match link.send_v1(function::VTX_CONFIG as u8, &response) {
                                    Ok(()) => {
                                        debug!(target: "vtx", "answered VTX_CONFIG query (acting as FC)");
                                        shared.table_sync.lock().unwrap().synchronized = true;
                                    }
                                    Err(e) => {
                                        error!(target: "vtx", "failed to answer VTX_CONFIG query: {e}");
                                        vtx_link_lost = true;
                                    }
                                }
                            }
                            ctx.request_repaint();
                        } else if frame.function == function::VTX_CONFIG && !frame.payload.is_empty() {
                            match msp::decode_vtx_config(&frame.payload) {
                                Ok(cfg) => {
                                    let mut sel = vtx_selection.lock().unwrap();
                                    sel.selected_band = cfg.band;
                                    sel.selected_channel = cfg.channel;
                                    sel.selected_power = cfg.power;
                                    sel.selected_freq_mhz = cfg.frequency_mhz;
                                    sel.pitmode = cfg.pitmode;
                                    drop(sel);
                                    debug!(target: "vtx", "VTX_CONFIG reply: band={} channel={} power={} freq={}MHz pitmode={}",
                                        cfg.band, cfg.channel, cfg.power, cfg.frequency_mhz, cfg.pitmode);
                                    ctx.request_repaint();
                                }
                                Err(e) => error!(target: "vtx", "failed to decode VTX_CONFIG reply: {e}"),
                            }
                        } else if frame.function == function::SET_OSD_CANVAS {
                            if let Some(canvas) = msp::decode_osd_canvas(&frame.payload) {
                                shared.osd.lock().unwrap().canvas = Some(canvas);
                                debug!(target: "vtx", "OSD canvas: {}x{}", canvas.0, canvas.1);
                            }
                        } else if frame.function == function::PACALIBRATION {
                            if let Ok(reading) = msp::decode_pa_calibration_reading(&frame.payload) {
                                let mut vtx_state = shared.vtx.lock().unwrap();
                                let power_mw = vtx_state
                                    .pa_table
                                    .iter()
                                    .find(|e| e.idx == reading.power_level)
                                    .map(|e| e.m_w);
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
                                    mcu_temp_c: reading.mcu_temp_c,
                                    ntc_raw: reading.ntc_raw,
                                    pa_temp_c: reading.pa_temp_c,
                                };
                                vtx_state.status = Some(status.clone());
                                if let Some(temp_c) = reading.pa_temp_c {
                                    let elapsed = start.elapsed().as_secs_f64();
                                    vtx_state.pa_temp_history.push_back((elapsed, temp_c));
                                }
                                if let Some(mcu_temp_c) = reading.mcu_temp_c {
                                    let elapsed = start.elapsed().as_secs_f64();
                                    vtx_state.mcu_temp_history.push_back((elapsed, mcu_temp_c));
                                }
                                drop(vtx_state);
                                let osd_debug_overlay_enabled =
                                    shared.osd.lock().unwrap().debug_overlay_enabled;
                                if osd_debug_overlay_enabled && displayport_queue.is_empty() {
                                    displayport_queue.push_back(msp::encode_displayport_clear());
                                    displayport_queue.extend(build_status_displayport_frames(&status));
                                }
                                debug!(target: "vtx", "status: level={} power_mw={:?} boost_on={:?} rtc6705_level={:?} freq_mhz={:?} vbias_mv={} detector_mv={} pid_active={:?} session_active={:?} mcu_temp_c={:?} ntc_raw={:?} pa_temp_c={:?}",
                                    reading.power_level, power_mw, reading.boost_on, reading.rtc6705_level,
                                    reading.frequency_mhz, reading.vref_mv, reading.detector_mv,
                                    reading.pid_active, reading.session_active, reading.mcu_temp_c, reading.ntc_raw, reading.pa_temp_c);
                                pa_calibration_reading = Some(reading);
                            }
                        }
                    }
                    Ok(None) => {}
                    Err(e) => {
                        error!(target: "vtx", "VTX link error, disconnecting: {e}");
                        vtx_link_lost = true;
                    }
                }
            }
            if vtx_link_lost {
                vtx = None;
                vtx_last_seen = None;
                {
                    let mut vtx_state = shared.vtx.lock().unwrap();
                    vtx_state.port_state = PortState::LostCommunication;
                    vtx_state.ready = false;
                }
                if let Some(engine) = sweep.lock().unwrap().as_mut() {
                    engine.force_connection_lost(calibration_engine::ConnectionLossReason::Vtx);
                }
            }
            let vtx_ready = vtx_last_seen.map(|t| t.elapsed() < READY_WINDOW).unwrap_or(false);
            if vtx.is_some() {
                let mut vtx_state = shared.vtx.lock().unwrap();
                vtx_state.ready = vtx_ready;
                if vtx_ready {
                    if vtx_state.port_state != PortState::Ready {
                        vtx_state.port_state = PortState::Ready;
                    }
                } else if vtx_state.port_state == PortState::Ready {
                    vtx_state.port_state = PortState::LostCommunication;
                }
            }

            if vtx.is_none() {
                if let Some(path) = vtx_port_path.clone() {
                    let is_lost = shared.vtx.lock().unwrap().port_state == PortState::LostCommunication;
                    if is_lost && vtx_last_reconnect_attempt.elapsed() >= RECONNECT_INTERVAL {
                        vtx_last_reconnect_attempt = Instant::now();
                        match MspLink::open(&path, 115200) {
                            Ok(l) => {
                                debug!(target: "vtx", "reopened {path} after lost communication");
                                vtx = Some(l);
                                vtx_last_seen = None;
                                shared.vtx.lock().unwrap().port_state = PortState::Ready;
                                if let Some(link) = vtx.as_mut() {
                                    let payload = calibration_engine::safe_state_payload(&vtx_table.lock().unwrap(), &vtx_selection.lock().unwrap());
                                    if let Err(e) = link.send_v1(function::VTX_CONFIG as u8, &payload) {
                                        error!(target: "vtx", "failed to push safe-state VTX_CONFIG after reconnect: {e}");
                                    }
                                    if shared.osd.lock().unwrap().debug_overlay_enabled {
                                        if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &msp::encode_displayport_keepalive()) {
                                            error!(target: "vtx", "failed to send DisplayPort keepalive after reconnect: {e}");
                                        }
                                        if let Err(e) = link.send_v1(function::DISPLAYPORT as u8, &msp::encode_displayport_clear()) {
                                            error!(target: "vtx", "failed to clear DisplayPort screen after reconnect: {e}");
                                        }
                                        last_displayport_keepalive = Instant::now();
                                        {
                                            let mut osd = shared.osd.lock().unwrap();
                                            osd.keepalive_at = Some(format_time_hms());
                                            osd.canvas = None;
                                        }
                                        displayport_queue.clear();
                                    }
                                }
                            }
                            Err(e) => debug!(target: "vtx", "reconnect attempt for {path} failed: {e}"),
                        }
                    }
                }
            }

            let meter_ready = meter_last_seen.map(|t| t.elapsed() < READY_WINDOW).unwrap_or(false);
            if let Some(link) = vtx.as_mut() {
                let (history_snapshot, reading_seq) = {
                    let meter_state = shared.meter.lock().unwrap();
                    (meter_state.power_history.clone(), meter_state.reading_seq)
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
                            let mut vtx_state = shared.vtx.lock().unwrap();
                            if let Some(entry) =
                                vtx_state.pa_table.iter_mut().find(|e| e.idx == result.level)
                            {
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
                        let mut meter_state = shared.meter.lock().unwrap();
                        if let Some(prev) = meter_state.pre_sweep_update_hz.take() {
                            meter_state.update_hz = prev;
                        }
                    }
                    ctx.request_repaint();
                }
            }

            if let Some(m) = meter.as_mut() {
                let interval = {
                    let hz = shared.meter.lock().unwrap().update_hz.max(0.01);
                    Duration::from_secs_f64(1.0 / hz)
                };
                if last_meter_read.elapsed() >= interval {
                    last_meter_read = Instant::now();
                    match m.read_dbm(Duration::from_millis(300)) {
                        Ok(raw_dbm) => {
                            let attenuation_db = shared.meter.lock().unwrap().attenuation_db;
                            let dbm = raw_dbm + attenuation_db;
                            let mw = 10f32.powf(dbm / 10.0);
                            debug!(target: "meter", "{raw_dbm:.2} dBm raw, {dbm:.2} dBm corrected (+{attenuation_db:.1}dB) ({mw:.6} mW)");
                            let elapsed = start.elapsed().as_secs_f64();
                            let mut meter_state = shared.meter.lock().unwrap();
                            meter_state.last_dbm = Some(dbm);
                            meter_state.power_history.push_back((elapsed, mw));
                            meter_state.reading_seq += 1;
                        }
                        Err(e) => error!(target: "meter", "read failed: {e}"),
                    }
                    ctx.request_repaint();
                }
            }

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
                shared.meter.lock().unwrap().port_state = PortState::LostCommunication;
                if let Some(engine) = sweep.lock().unwrap().as_mut() {
                    engine.force_connection_lost(calibration_engine::ConnectionLossReason::Meter);
                }
            } else if meter.is_some() {
                let is_ready = meter_last_seen.map(|t| t.elapsed() < READY_WINDOW).unwrap_or(false);
                let mut meter_state = shared.meter.lock().unwrap();
                if is_ready {
                    if meter_state.port_state != PortState::Ready {
                        meter_state.port_state = PortState::Ready;
                    }
                } else if meter_state.port_state == PortState::Ready {
                    meter_state.port_state = PortState::LostCommunication;
                }
            }

            if meter.is_none() {
                if let Some(path) = meter_port_path.clone() {
                    let (is_lost, kind) = {
                        let meter_state = shared.meter.lock().unwrap();
                        (meter_state.port_state == PortState::LostCommunication, meter_state.kind)
                    };
                    if is_lost && meter_last_reconnect_attempt.elapsed() >= RECONNECT_INTERVAL {
                        meter_last_reconnect_attempt = Instant::now();
                        match PowerMeter::open(kind, &path) {
                            Ok(m) => {
                                debug!(target: "meter", "reopened {path} after lost communication");
                                meter = Some(m);
                                meter_last_seen = None;
                                shared.meter.lock().unwrap().port_state = PortState::Ready;
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

fn build_status_displayport_frames(status: &VtxStatus) -> Vec<Vec<u8>> {
    fn row(label: &str, value: String) -> String {
        format!("{label:<10}{value:>18}").to_uppercase()
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

fn format_time_hms() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let (h, m, s) = ((secs / 3600) % 24, (secs / 60) % 60, secs % 60);
    format!("{h:02}:{m:02}:{s:02}")
}

fn trim_stale_history(history: &mut VecDeque<(f64, f32)>, now: f64) {
    while let Some(&(t, _)) = history.front() {
        if now - t > HISTORY_WINDOW_SECS {
            history.pop_front();
        } else {
            break;
        }
    }
}

fn read_pa_table(link: &mut MspLink) -> Result<Vec<msp::PaCalibration>> {
    link.send_v2(function::PACALTABLE, None)?;
    debug!(target: "vtx", "sent PACALTABLE request");
    let mut entries = Vec::new();
    let quiet_period = Duration::from_millis(300);
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
                deadline = Instant::now() + quiet_period;
            }
            Some(frame) => {
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
    let deadline = Instant::now() + Duration::from_millis(300);
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
