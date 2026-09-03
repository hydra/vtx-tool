use crate::conn_status::PortState;
use crate::msp;
use crate::power_meter::PowerMeterKind;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

const DEFAULT_ATTENUATION_DB: f32 = 30.0;
const DEFAULT_UPDATE_HZ: f64 = 1.0;

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
    pub session_active: Option<bool>,
    pub mcu_temp_c: Option<f32>,
    pub ntc_raw: Option<u16>,
    pub pa_temp_c: Option<f32>,
}

pub struct MeterState {
    pub port_state: PortState,
    pub kind: PowerMeterKind,
    pub attenuation_db: f32,
    pub update_hz: f64,
    pub pre_sweep_update_hz: Option<f64>,
    pub last_dbm: Option<f32>,
    pub power_history: VecDeque<(f64, f32)>,
    pub reading_seq: u64,
}

impl Default for MeterState {
    fn default() -> Self {
        Self {
            port_state: PortState::Disconnected,
            kind: PowerMeterKind::default(),
            attenuation_db: DEFAULT_ATTENUATION_DB,
            update_hz: DEFAULT_UPDATE_HZ,
            pre_sweep_update_hz: None,
            last_dbm: None,
            power_history: VecDeque::new(),
            reading_seq: 0,
        }
    }
}

pub struct VtxState {
    pub port_state: PortState,
    pub ready: bool,
    pub last_seen_at: Option<String>,
    pub status: Option<VtxStatus>,
    pub config: Option<msp::VtxConfig>,
    pub pa_table: Vec<msp::PaCalibration>,
    pub pa_temp_history: VecDeque<(f64, f32)>,
    pub mcu_temp_history: VecDeque<(f64, f32)>,
}

impl Default for VtxState {
    fn default() -> Self {
        Self {
            port_state: PortState::Disconnected,
            ready: false,
            last_seen_at: None,
            status: None,
            config: None,
            pa_table: Vec::new(),
            pa_temp_history: VecDeque::new(),
            mcu_temp_history: VecDeque::new(),
        }
    }
}

pub struct OsdState {
    pub canvas: Option<(u8, u8)>,
    pub keepalive_at: Option<String>,
    pub debug_overlay_enabled: bool,
}

impl Default for OsdState {
    fn default() -> Self {
        Self {
            canvas: None,
            keepalive_at: None,
            debug_overlay_enabled: true,
        }
    }
}

#[derive(Default)]
pub struct VtxTableSyncState {
    pub ready: bool,
    pub synchronized: bool,
}

/// Shared application state, split by hardware/feature domain so each mutex
/// guards one cohesive concern rather than one monolithic struct.
///
/// Lock discipline: acquire a single domain lock at a time wherever possible.
/// When two must be held at once, acquire them in field-declaration order
/// (`meter`, `vtx`, `osd`, `table_sync`) so the global ordering stays consistent
/// and deadlock-free.
#[derive(Clone, Default)]
pub struct SharedHandles {
    pub meter: Arc<Mutex<MeterState>>,
    pub vtx: Arc<Mutex<VtxState>>,
    pub osd: Arc<Mutex<OsdState>>,
    pub table_sync: Arc<Mutex<VtxTableSyncState>>,
}

impl SharedHandles {
    pub fn new() -> Self {
        Self::default()
    }

    /// `(vtx, meter)` port states, taken as two independent short-lived locks.
    pub fn port_states(&self) -> (PortState, PortState) {
        let meter = self.meter.lock().unwrap().port_state;
        let vtx = self.vtx.lock().unwrap().port_state;
        (vtx, meter)
    }
}
