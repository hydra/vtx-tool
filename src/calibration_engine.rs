
use crate::msp::{self, function, MspCommandKind, MspLink};
use crate::power_meter::{nearest_band, FrequencyCapability};
use crate::vtxtable::{VtxSelectionState, VtxTableConfig};
use log::debug;
use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};

fn power_up_step(sign_inverted: bool) -> i32 {
    if sign_inverted {
        -1
    } else {
        1
    }
}

fn coarse_ramp_start_vbias_mv(sign_inverted: bool, bound_lo: i32, bound_hi: i32) -> i32 {
    let up = power_up_step(sign_inverted);
    if up > 0 {
        (bound_lo + COARSE_RAMP_START_MARGIN_MV).min(bound_hi)
    } else {
        (bound_hi - COARSE_RAMP_START_MARGIN_MV).max(bound_lo)
    }
}

fn rolling_average_since(history: &VecDeque<(f64, f32)>, since_secs: f64, window_secs: f64) -> Option<f32> {
    let now = history.back()?.0;
    let window_start = (now - window_secs).max(since_secs);
    if now - window_start < window_secs - 0.01 {
        return None;
    }
    let (sum, count) = history
        .iter()
        .rev()
        .take_while(|(t, _)| *t >= window_start)
        .fold((0.0f32, 0u32), |(s, c), (_, mw)| (s + mw, c + 1));
    if count == 0 {
        None
    } else {
        Some(sum / count as f32)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweepOp {
    ScanPa,
    ScanDetector,
}

impl SweepOp {
    fn label(self) -> &'static str {
        match self {
            SweepOp::ScanPa => "ScanPa",
            SweepOp::ScanDetector => "ScanDetector",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum LevelStatus {
    NotCalibrated,
    Pending,
    InProgress(String),
    Done,
    Aborted,
    Skipped,
    PaFailure,
    NotSettled,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellStatus {
    Default,
    Calibrated,
    Current,
    LimitHit,
    Uncalibrated,
    Skipped,
    Manual,
    PaFailure,
    NotSettled,
}

enum ScanPaOutcome {
    Success,
    Uncalibrated,
    PaFailure,
    NotSettled,
}

pub(crate) struct SampleWait {
    start_seq: u64,
    needed: usize,
    skip_first: usize,
}

impl SampleWait {
    fn new(current_seq: u64, needed: usize, skip_first: usize) -> Self {
        Self { start_seq: current_seq, needed, skip_first }
    }

    fn ready(&self, current_seq: u64) -> bool {
        current_seq >= self.start_seq + self.needed as u64
    }

    fn average(&self, history: &VecDeque<(f64, f32)>) -> f32 {
        let take = self.needed - self.skip_first;
        let sum: f32 = history.iter().rev().take(take).map(|&(_, mw)| mw).sum();
        sum / take.max(1) as f32
    }
}

const COARSE_RAMP_START_MARGIN_MV: i32 = 0;
const COARSE_RAMP_STEP_MV: i32 = 25;
const PA_FAILURE_WINDOW_SECS: f64 = 3.0;
const PA_FAILURE_DROP_FRACTION: f32 = 0.03;
const FINE_SETTLE_DELAY: Duration = Duration::from_secs(2);
const PA_FAILURE_GRACE_DURATION: Duration = Duration::from_secs(15);
const BOOST_ENABLE_SETTLE_DELAY: Duration = Duration::from_secs(2);
const SETTLE_BELOW_TARGET_TIMEOUT: Duration = Duration::from_secs(60);
const HEARTBEAT_TIMEOUT: Duration = Duration::from_millis(500);
const HEARTBEAT_BACKOFF_MV: i32 = 50;
const PINNED_LIMIT: u32 = 5;
const SEND_INTERVAL: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResumeMode {
    Automatic,
    Manual,
}

pub(crate) enum ScanPaPhase {
    Settle,
    CoarseRamp,
    Fine,
}

pub(crate) struct ScanPaState {
    phase: ScanPaPhase,
    vbias_mv: i32,
    wait: Option<SampleWait>,
    settle_started_instant: Option<Instant>,
    settle_timed_out: bool,
    coarse_steps_taken: u32,
    last_below_target_mv: Option<i32>,
    coarse_step_mv: i32,
    fine_bound_mv: Option<i32>,
    fine_started_at_secs: Option<f64>,
    fine_highest_avg_mw: Option<f32>,
    fine_settle_until: Option<Instant>,
    fine_started_instant: Option<Instant>,
}

pub(crate) enum ScanDetectorPhase {
    Backoff,
    Bracket,
}

pub(crate) struct ScanDetectorState {
    phase: ScanDetectorPhase,
    vbias_mv: i32,
    wait: Option<SampleWait>,
    below: Option<(f32, u16)>,
    above: Option<(f32, u16)>,
    pinned_count: u32,
    last_reading: Option<msp::PaCalibrationReading>,
}

pub(crate) enum AutomaticStep {
    EnteringPoint,
    ScanPa(ScanPaState),
    ScanDetector(ScanDetectorState),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionLossReason {
    Vtx,
    Meter,
    Both,
}

pub enum EngineState {
    Idle,
    AwaitingFreqConfirm { freq_mhz: u16, resume: ResumeMode },
    Automatic(AutomaticStep),
    Manual,
    ConnectionLost {
        level: u8,
        freq_mhz: u16,
        vbias_mv_at_loss: i32,
        reason: ConnectionLossReason,
        resume: ResumeMode,
    },
}

pub struct SweepResult {
    pub level: u8,
    pub freq_idx: usize,
    pub vbias_mv: Option<u16>,
    pub detector_mv: Option<u16>,
    pub success: bool,
    pub pa_failure: bool,
    pub not_settled: bool,
}

pub struct CurrentStep {
    pub level: u8,
    pub freq_idx: usize,
    pub vbias_mv: Option<i32>,
    pub detector_mv: Option<i32>,
    pub cal_is_current: bool,
    pub det_is_current: bool,
}

pub struct StepDebugInfo {
    pub scan_phase: &'static str,
    pub drop_detector_active: bool,
    pub fine_bound_mv: Option<i32>,
    pub fine_highest_avg_mw: Option<f32>,
    pub detector: Option<DetectorDebugInfo>,
}

pub struct DetectorDebugInfo {
    pub phase: &'static str,
    pub below: Option<(f32, u16)>,
    pub above: Option<(f32, u16)>,
    pub pinned_count: u32,
}

pub struct SweepEngine {
    pub levels: Vec<u8>,
    pub frequencies: Vec<u16>,
    pub tolerance_pct: f32,
    pub sign_inverted: bool,
    pub target_mw_by_level: HashMap<u8, u16>,
    pub sweep_hz: f64,

    pub state: EngineState,
    freq_idx: usize,
    level_idx: usize,
    last_send: Instant,

    pub per_level_status: HashMap<u8, LevelStatus>,
    pub cal_cell_status: HashMap<(u8, usize), CellStatus>,
    pub det_cell_status: HashMap<(u8, usize), CellStatus>,
    pub total_steps: usize,
    pub completed_steps: usize,

    pub pending_result: Option<SweepResult>,

    meter_capability: FrequencyCapability,
    last_prompted_band: Option<u32>,
    pending_frequency_push: Option<u16>,
    pub pending_meter_frequency: Option<u32>,

    pub hard_limits: HashMap<u8, i32>,
    pending_sends: VecDeque<PendingSend>,
    unresponsive_since: Option<Instant>,

    pub manual_dac_mv: i32,
    manual_send_pending: bool,
    session_active: bool,
    boost_mode: BoostMode,
    last_boost_on: Option<bool>,
    boost_settle_until: Option<Instant>,
    pa_enable_settle_pending: bool,
}

enum PendingSend {
    SafeState(Vec<u8>),
    CalibrationState,
    RequestVtxConfig,
    DacLow { level: u8, vbias_mv: u16 },
    RestoreBoost { level: u8 },
}


#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BoostMode {
    Off,
    On,
    Auto,
}

impl BoostMode {
    fn wire_byte(self) -> u8 {
        match self {
            BoostMode::Off => 0,
            BoostMode::On => 1,
            BoostMode::Auto => 2,
        }
    }
}

impl SweepEngine {
    pub fn new(
        levels: Vec<u8>,
        frequencies: Vec<u16>,
        tolerance_pct: f32,
        sign_inverted: bool,
        target_mw_by_level: HashMap<u8, u16>,
        meter_max_hz: u32,
    ) -> Self {
        let mut per_level_status = HashMap::new();
        for &lvl in &levels {
            per_level_status.insert(lvl, LevelStatus::Pending);
        }
        let total_steps = levels.len() * frequencies.len() * 2;
        Self {
            levels,
            frequencies,
            tolerance_pct,
            sign_inverted,
            target_mw_by_level,
            sweep_hz: 10.0_f64.min(meter_max_hz as f64),
            state: EngineState::Idle,
            freq_idx: 0,
            level_idx: 0,
            last_send: Instant::now() - SEND_INTERVAL,
            per_level_status,
            cal_cell_status: HashMap::new(),
            det_cell_status: HashMap::new(),
            total_steps,
            completed_steps: 0,
            pending_result: None,
            meter_capability: FrequencyCapability::Manual { min_mhz: 0, max_mhz: 0 },
            last_prompted_band: None,
            pending_frequency_push: None,
            pending_meter_frequency: None,
            hard_limits: HashMap::new(),
            pending_sends: VecDeque::new(),
            unresponsive_since: None,
            manual_dac_mv: 0,
            manual_send_pending: false,
            session_active: false,
            boost_mode: BoostMode::Auto,
            last_boost_on: None,
            boost_settle_until: None,
            pa_enable_settle_pending: false,
        }
    }

    pub fn progress(&self) -> f32 {
        if self.total_steps == 0 {
            return 1.0;
        }
        self.completed_steps as f32 / self.total_steps as f32
    }

    pub fn start(&mut self, capability: FrequencyCapability) {
        if self.levels.is_empty() || self.frequencies.is_empty() {
            debug!(target: "vtx", "[sweep] start() called with no levels/frequencies -- not starting");
            self.state = EngineState::Idle;
            return;
        }
        self.freq_idx = 0;
        self.level_idx = 0;
        self.last_prompted_band = None;
        self.meter_capability = capability;
        self.session_active = true;
        self.boost_mode = BoostMode::Auto;
        debug!(target: "vtx", "[sweep] starting: {} levels {:?}, {} frequencies {:?}, tolerance={}%, sign_inverted={}, sweep_hz={}, meter_capability={:?}",
            self.levels.len(), self.levels, self.frequencies.len(), self.frequencies, self.tolerance_pct, self.sign_inverted, self.sweep_hz, self.meter_capability);
        self.pending_sends.push_back(PendingSend::CalibrationState);
        let first_freq = self.frequencies[0];
        self.begin_frequency(first_freq, ResumeMode::Automatic);
    }

    pub fn start_manual(&mut self, capability: FrequencyCapability) {
        if self.levels.is_empty() || self.frequencies.is_empty() {
            debug!(target: "vtx", "[sweep] start_manual() called with no levels/frequencies -- not starting");
            self.state = EngineState::Idle;
            return;
        }
        self.freq_idx = 0;
        self.level_idx = 0;
        self.last_prompted_band = None;
        self.meter_capability = capability;
        self.manual_send_pending = false;
        self.session_active = true;
        self.boost_mode = BoostMode::Off;
        debug!(target: "vtx", "[sweep] starting manual mode: {} levels {:?}, {} frequencies {:?}, meter_capability={:?}",
            self.levels.len(), self.levels, self.frequencies.len(), self.frequencies, self.meter_capability);
        self.pending_sends.push_back(PendingSend::CalibrationState);
        let first_freq = self.frequencies[0];
        self.begin_frequency(first_freq, ResumeMode::Manual);
    }

    pub fn set_pa_boost(&mut self, on: bool) {
        self.boost_mode = if on { BoostMode::On } else { BoostMode::Off };
        self.manual_send_pending = true;
    }

    pub fn set_manual_dac(&mut self, mv: i32) {
        self.manual_dac_mv = mv.clamp(0, 3300);
        self.manual_send_pending = true;
    }

    pub fn manual_next(&mut self, detector_mv: u16) -> Option<(u8, usize)> {
        if !matches!(&self.state, EngineState::Manual) {
            return None;
        }
        let level = self.levels[self.level_idx];
        debug!(target: "vtx", "[sweep] manual: level={level} freq={}MHz vbias_mv={} detector_mv={detector_mv}",
            self.frequencies[self.freq_idx], self.manual_dac_mv);
        self.completed_steps += 2;
        self.pending_result = Some(SweepResult {
            level,
            freq_idx: self.freq_idx,
            vbias_mv: Some(self.manual_dac_mv.clamp(0, 3300) as u16),
            detector_mv: Some(detector_mv),
            success: true,
            pa_failure: false,
            not_settled: false,
        });
        Self::set_cell_status(&mut self.cal_cell_status, (level, self.freq_idx), CellStatus::Manual);
        Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), CellStatus::Manual);
        self.per_level_status.insert(level, LevelStatus::Done);
        self.advance_position()
    }

    fn advance_indices(&mut self) -> Option<bool> {
        if self.level_idx + 1 < self.levels.len() {
            self.level_idx += 1;
            return Some(false);
        }
        if self.freq_idx + 1 >= self.frequencies.len() {
            return None;
        }
        self.level_idx = 0;
        self.freq_idx += 1;
        Some(true)
    }

    fn advance_position(&mut self) -> Option<(u8, usize)> {
        let resume = match &self.state {
            EngineState::Manual => ResumeMode::Manual,
            _ => ResumeMode::Automatic,
        };
        let safe_state_payload = self.safe_state_payload_at_current_point();
        match self.advance_indices() {
            None => {
                debug!(target: "vtx", "[sweep] all frequencies complete");
                self.pending_sends.push_back(PendingSend::SafeState(safe_state_payload));
                self.state = EngineState::Idle;
                self.session_active = false;
                self.boost_mode = BoostMode::Auto;
                self.pending_sends.push_back(PendingSend::CalibrationState);
                None
            }
            Some(true) => {
                let next_freq = self.frequencies[self.freq_idx];
                self.begin_frequency(next_freq, resume);
                Some((self.levels[self.level_idx], self.freq_idx))
            }
            Some(false) => {
                self.state = match resume {
                    ResumeMode::Automatic => EngineState::Automatic(AutomaticStep::EnteringPoint),
                    ResumeMode::Manual => EngineState::Manual,
                };
                Some((self.levels[self.level_idx], self.freq_idx))
            }
        }
    }

    pub fn exit_manual(&mut self) {
        debug!(target: "vtx", "[sweep] manual mode exited at level={:?} freq_idx={}",
            self.levels.get(self.level_idx), self.freq_idx);
        let safe_state_payload = self.safe_state_payload_at_current_point();
        self.state = EngineState::Idle;
        self.pending_frequency_push = None;
        self.manual_send_pending = false;
        self.session_active = false;
        self.boost_mode = BoostMode::Auto;
        self.pending_sends.push_back(PendingSend::SafeState(safe_state_payload));
        self.pending_sends.push_back(PendingSend::CalibrationState);
    }

    pub fn resume_automatic_from_current(&mut self) {
        debug!(target: "vtx", "[sweep] resuming automatic mode from level={:?} freq_idx={}",
            self.levels.get(self.level_idx), self.freq_idx);
        self.manual_send_pending = false;
        self.state = EngineState::Automatic(AutomaticStep::EnteringPoint);
        self.boost_mode = BoostMode::Auto;
        self.pending_sends.push_back(PendingSend::CalibrationState);
        let resume_level = self.levels[self.level_idx];
        self.pending_sends.push_back(PendingSend::RestoreBoost { level: resume_level });
    }

    fn begin_frequency(&mut self, freq_mhz: u16, resume: ResumeMode) {
        let not_confirming_state = match resume {
            ResumeMode::Automatic => EngineState::Automatic(AutomaticStep::EnteringPoint),
            ResumeMode::Manual => EngineState::Manual,
        };
        match self.meter_capability.clone() {
            FrequencyCapability::Manual { .. } => {
                self.state = EngineState::AwaitingFreqConfirm { freq_mhz, resume };
            }
            FrequencyCapability::ManualBand { bands_mhz } => {
                let band = nearest_band(&bands_mhz, freq_mhz as u32);
                if Some(band) == self.last_prompted_band {
                    debug!(target: "vtx", "[sweep] {freq_mhz}MHz maps to the same band ({band}MHz) as the last prompt -- skipping prompt");
                    self.state = not_confirming_state;
                    self.pending_frequency_push = Some(freq_mhz);
                } else {
                    self.last_prompted_band = Some(band);
                    self.state = EngineState::AwaitingFreqConfirm { freq_mhz, resume };
                }
            }
            FrequencyCapability::ProgrammableBand { bands_mhz } => {
                let band = nearest_band(&bands_mhz, freq_mhz as u32);
                debug!(target: "vtx", "[sweep] requesting meter retune to nearest band {band}MHz (for VTX freq {freq_mhz}MHz)");
                self.pending_meter_frequency = Some(band);
                self.state = not_confirming_state;
                self.pending_frequency_push = Some(freq_mhz);
            }
            FrequencyCapability::FullyProgrammable { .. } => {
                debug!(target: "vtx", "[sweep] requesting meter retune to {freq_mhz}MHz");
                self.pending_meter_frequency = Some(freq_mhz as u32);
                self.state = not_confirming_state;
                self.pending_frequency_push = Some(freq_mhz);
            }
        }
    }

    pub fn confirm_frequency(&mut self) {
        if let EngineState::AwaitingFreqConfirm { freq_mhz, resume } = &self.state {
            let (freq_mhz, resume) = (*freq_mhz, *resume);
            debug!(target: "vtx", "[sweep] frequency confirmed, resuming at {freq_mhz}MHz");
            self.state = match resume {
                ResumeMode::Automatic => EngineState::Automatic(AutomaticStep::EnteringPoint),
                ResumeMode::Manual => EngineState::Manual,
            };
            self.pending_frequency_push = Some(freq_mhz);
        }
    }

    pub fn abort(&mut self) {
        debug!(target: "vtx", "[sweep] aborted at level={:?} freq_idx={} ({}/{} steps completed)",
            self.levels.get(self.level_idx), self.freq_idx, self.completed_steps, self.total_steps);
        let safe_state_payload = self.safe_state_payload_at_current_point();
        for status in self.per_level_status.values_mut() {
            if !matches!(status, LevelStatus::Done) {
                *status = LevelStatus::Aborted;
            }
        }
        self.state = EngineState::Idle;
        self.pending_frequency_push = None;
        self.manual_send_pending = false;
        self.session_active = false;
        self.boost_mode = BoostMode::Auto;
        self.pending_sends.clear();
        self.pending_sends.push_back(PendingSend::SafeState(safe_state_payload));
        self.pending_sends.push_back(PendingSend::CalibrationState);
    }

    pub fn skip_multiple(&mut self, count: u32) -> Option<(u8, usize)> {
        if count == 0 {
            return None;
        }
        let resume = match &self.state {
            EngineState::Manual => ResumeMode::Manual,
            EngineState::Automatic(_) => ResumeMode::Automatic,
            _ => return None,
        };
        let starting_level = self.levels[self.level_idx];
        debug!(target: "vtx", "[sweep] skip x{count} starting at level={starting_level} freq_idx={} ({}/{} steps completed)",
            self.freq_idx, self.completed_steps, self.total_steps);

        let mut crossed_frequency = false;

        for i in 0..count {
            let level = self.levels[self.level_idx];
            if i == 0 {
                let ops = match &self.state {
                    EngineState::Automatic(AutomaticStep::ScanPa(_)) => {
                        Self::set_cell_status(&mut self.cal_cell_status, (level, self.freq_idx), CellStatus::Skipped);
                        Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), CellStatus::Skipped);
                        2
                    }
                    EngineState::Automatic(AutomaticStep::ScanDetector(_)) => {
                        Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), CellStatus::Skipped);
                        1
                    }
                    EngineState::Automatic(AutomaticStep::EnteringPoint) => 0,
                    EngineState::Manual => {
                        Self::set_cell_status(&mut self.cal_cell_status, (level, self.freq_idx), CellStatus::Skipped);
                        Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), CellStatus::Skipped);
                        2
                    }
                    _ => 0,
                };
                self.completed_steps += ops;
            } else {
                Self::set_cell_status(&mut self.cal_cell_status, (level, self.freq_idx), CellStatus::Skipped);
                Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), CellStatus::Skipped);
                self.completed_steps += 2;
            }
            self.per_level_status.insert(level, LevelStatus::Skipped);

            match self.advance_indices() {
                None => {
                    debug!(target: "vtx", "[sweep] skip x{count}: all frequencies complete after {} skips", i + 1);
                    let safe_state_payload = self.safe_state_payload_at_current_point();
                    self.pending_sends.push_back(PendingSend::SafeState(safe_state_payload));
                    self.state = EngineState::Idle;
                    self.session_active = false;
                    self.boost_mode = BoostMode::Auto;
                    self.pending_sends.push_back(PendingSend::CalibrationState);
                    return None;
                }
                Some(crossed) => crossed_frequency |= crossed,
            }
        }

        let (bound_lo, bound_hi) = self.effective_bounds(starting_level);
        let low_mv = coarse_ramp_start_vbias_mv(self.sign_inverted, bound_lo, bound_hi);
        self.pending_sends.push_back(PendingSend::DacLow { level: starting_level, vbias_mv: low_mv as u16 });
        let pitmode_payload = self.safe_state_payload_at_current_point();
        self.pending_sends.push_back(PendingSend::SafeState(pitmode_payload));
        let resume_level = self.levels[self.level_idx];
        self.pending_sends.push_back(PendingSend::RestoreBoost { level: resume_level });
        debug!(target: "vtx", "[sweep] skip x{count}: safe transition queued (level={starting_level} vbias_mv={low_mv} boost=Off, then pitmode, then boost restored for level={resume_level}), resuming at level={resume_level} freq_idx={}",
            self.freq_idx);

        if crossed_frequency {
            let freq_mhz = self.frequencies[self.freq_idx];
            self.begin_frequency(freq_mhz, resume);
        } else {
            self.state = match resume {
                ResumeMode::Automatic => EngineState::Automatic(AutomaticStep::EnteringPoint),
                ResumeMode::Manual => EngineState::Manual,
            };
        }

        match resume {
            ResumeMode::Manual => Some((self.levels[self.level_idx], self.freq_idx)),
            ResumeMode::Automatic => None,
        }
    }

    pub fn is_active(&self) -> bool {
        !matches!(&self.state, EngineState::Idle)
    }

    pub fn is_manual_mode(&self) -> bool {
        match &self.state {
            EngineState::Manual => true,
            EngineState::AwaitingFreqConfirm { resume, .. } => *resume == ResumeMode::Manual,
            EngineState::ConnectionLost { resume, .. } => *resume == ResumeMode::Manual,
            _ => false,
        }
    }

    pub fn is_automatic_mode(&self) -> bool {
        match &self.state {
            EngineState::Automatic(_) => true,
            EngineState::AwaitingFreqConfirm { resume, .. } => *resume == ResumeMode::Automatic,
            EngineState::ConnectionLost { resume, .. } => *resume == ResumeMode::Automatic,
            _ => false,
        }
    }

    pub fn poll(
        &mut self,
        link: &mut MspLink,
        history: &VecDeque<(f64, f32)>,
        reading_seq: u64,
        latest_reading: Option<msp::PaCalibrationReading>,
        vtx_ready: bool,
        meter_ready: bool,
    ) -> anyhow::Result<bool> {
        let result = self.poll_dispatch(link, history, reading_seq, latest_reading.clone(), vtx_ready, meter_ready);

        if let EngineState::Automatic(AutomaticStep::ScanDetector(detector_state)) = &mut self.state {
            if let Some(reading) = latest_reading {
                detector_state.last_reading = Some(reading);
            }
        }
        result
    }

    fn poll_dispatch(
        &mut self,
        link: &mut MspLink,
        history: &VecDeque<(f64, f32)>,
        reading_seq: u64,
        latest_reading: Option<msp::PaCalibrationReading>,
        vtx_ready: bool,
        meter_ready: bool,
    ) -> anyhow::Result<bool> {
        if let Some(reading) = &latest_reading {
            if let Some(boost_on) = reading.boost_on {
                if boost_on && self.last_boost_on != Some(true) {
                    debug!(target: "vtx", "[sweep] PA boost just enabled -- ignoring samples for {BOOST_ENABLE_SETTLE_DELAY:?}");
                    self.boost_settle_until = Some(Instant::now() + BOOST_ENABLE_SETTLE_DELAY);
                    self.pa_enable_settle_pending = true;
                }
                self.last_boost_on = Some(boost_on);
            }
        }

        if !self.pending_sends.is_empty() {
            if !link.can_send_now() {
                return Ok(false);
            }
            match self.pending_sends.pop_front().unwrap() {
                PendingSend::SafeState(payload) => {
                    link.send_v1(function::VTX_CONFIG as u8, &payload)?;
                    link.note_sent(MspCommandKind::Retune);
                    debug!(target: "vtx", "[sweep] pitmode-safe state sent");
                    self.pending_sends.push_back(PendingSend::RequestVtxConfig);
                }
                PendingSend::CalibrationState => {
                    let payload = msp::encode_pa_calibration_request(0, None, self.session_active, self.boost_mode.wire_byte());
                    link.send_v2(function::SET_PACALIBRATION, Some(&payload))?;
                    link.note_sent(MspCommandKind::Other);
                    debug!(target: "vtx", "[sweep] calibration state pushed: session_active={} boost_mode={:?}",
                        self.session_active, self.boost_mode);
                }
                PendingSend::RequestVtxConfig => {
                    link.send_v1(function::VTX_CONFIG as u8, &[])?;
                    link.note_sent(MspCommandKind::Other);
                    debug!(target: "vtx", "[sweep] requested current VTX_CONFIG (to confirm the last push actually landed)");
                }
                PendingSend::DacLow { level, vbias_mv } => {
                    let payload = msp::encode_pa_calibration_request(level, Some(vbias_mv), self.session_active, BoostMode::Off.wire_byte());
                    link.send_v2(function::SET_PACALIBRATION, Some(&payload))?;
                    link.note_sent(MspCommandKind::Calibration);
                    debug!(target: "vtx", "[sweep] DAC parked low: level={level} vbias_mv={vbias_mv} boost=Off");
                }
                PendingSend::RestoreBoost { level } => {
                    let payload = msp::encode_pa_calibration_request(level, None, self.session_active, BoostMode::On.wire_byte());
                    link.send_v2(function::SET_PACALIBRATION, Some(&payload))?;
                    link.note_sent(MspCommandKind::Calibration);
                    debug!(target: "vtx", "[sweep] boost restored: level={level} boost=On");
                }
            }
            return Ok(true);
        }

        if let EngineState::ConnectionLost { level, freq_mhz, vbias_mv_at_loss, reason, .. } = &self.state {
            let (level, freq_mhz, vbias_mv_at_loss, reason) = (*level, *freq_mhz, *vbias_mv_at_loss, *reason);
            if vtx_ready && meter_ready {
                self.auto_resume(level, freq_mhz, vbias_mv_at_loss, reason);
            }
            return Ok(false);
        }
        if matches!(&self.state, EngineState::Manual) {
            return self.poll_manual(link, vtx_ready, meter_ready);
        }
        if matches!(&self.state, EngineState::Automatic(_)) {
            return self.poll_automatic(link, history, reading_seq, vtx_ready, meter_ready);
        }
        Ok(false)
    }

    fn poll_automatic(
        &mut self,
        link: &mut MspLink,
        history: &VecDeque<(f64, f32)>,
        reading_seq: u64,
        vtx_ready: bool,
        meter_ready: bool,
    ) -> anyhow::Result<bool> {
        if !link.can_send_now() {
            return Ok(false);
        }

        let vbias_mv_now = match &self.state {
            EngineState::Automatic(AutomaticStep::ScanPa(st)) => st.vbias_mv,
            EngineState::Automatic(AutomaticStep::ScanDetector(st)) => st.vbias_mv,
            _ => 0,
        };
        if self.maybe_trip_connection_lost(vtx_ready, meter_ready, vbias_mv_now) {
            return Ok(false);
        }

        if let Some(freq_mhz) = self.pending_frequency_push.take() {
            let level = self.levels.first().copied().unwrap_or(1);
            let payload = self.build_vtx_config_frequency_payload(freq_mhz, level, false);
            link.send_v1(function::VTX_CONFIG as u8, &payload)?;
            link.note_sent(MspCommandKind::Retune);
            debug!(target: "vtx", "[sweep] pushed frequency change to {freq_mhz}MHz (power={level})");
            self.pending_sends.push_back(PendingSend::RequestVtxConfig);
            return Ok(true);
        }

        let level = self.levels[self.level_idx];
        let freq_mhz = self.frequencies[self.freq_idx];
        let target_mw = *self.target_mw_by_level.get(&level).unwrap_or(&0) as f32;
        let now_seq = reading_seq;
        let throttled = self.last_send.elapsed() < SEND_INTERVAL;

        let step = match std::mem::replace(&mut self.state, EngineState::Idle) {
            EngineState::Automatic(step) => step,
            other => {
                self.state = other;
                return Ok(false);
            }
        };

        match step {
            AutomaticStep::EnteringPoint => {
                let (bound_lo, bound_hi) = self.effective_bounds(level);
                let start_vbias_mv = coarse_ramp_start_vbias_mv(self.sign_inverted, bound_lo, bound_hi);
                let needs_settle = self.pa_enable_settle_pending;
                let st = ScanPaState {
                    phase: if needs_settle { ScanPaPhase::Settle } else { ScanPaPhase::CoarseRamp },
                    vbias_mv: start_vbias_mv,
                    wait: None,
                    settle_started_instant: if needs_settle { Some(Instant::now()) } else { None },
                    settle_timed_out: false,
                    coarse_steps_taken: 0,
                    last_below_target_mv: None,
                    coarse_step_mv: COARSE_RAMP_STEP_MV,
                    fine_bound_mv: None,
                    fine_started_at_secs: None,
                    fine_highest_avg_mw: None,
                    fine_settle_until: None,
                    fine_started_instant: None,
                };
                debug!(target: "vtx", "[sweep] level={level} freq={freq_mhz}MHz target={target_mw}mW: starting ScanPa ({}) from vbias_mv={start_vbias_mv} (sign_inverted={})",
                    if needs_settle { "settle, then coarse ramp" } else { "coarse ramp" }, self.sign_inverted);
                self.per_level_status.insert(
                    level,
                    LevelStatus::InProgress(format!(
                        "{} / {} @ {freq_mhz}MHz",
                        SweepOp::ScanPa.label(),
                        if needs_settle { "settling" } else { "coarse ramp" }
                    )),
                );
                self.tick_scan_pa(link, history, now_seq, throttled, level, freq_mhz, target_mw, st)
            }
            AutomaticStep::ScanPa(st) => self.tick_scan_pa(link, history, now_seq, throttled, level, freq_mhz, target_mw, st),
            AutomaticStep::ScanDetector(st) => self.tick_scan_detector(link, history, now_seq, throttled, level, freq_mhz, target_mw, st),
        }
    }

    fn tick_scan_pa(
        &mut self,
        link: &mut MspLink,
        history: &VecDeque<(f64, f32)>,
        now_seq: u64,
        throttled: bool,
        level: u8,
        freq_mhz: u16,
        target_mw: f32,
        mut st: ScanPaState,
    ) -> anyhow::Result<bool> {
        if let Some(settle_until) = self.boost_settle_until {
            if Instant::now() < settle_until {
                st.wait = None;
                let mut sent = false;
                if !throttled {
                    self.send_calibration(link, level, st.vbias_mv)?;
                    self.last_send = Instant::now();
                    sent = true;
                }
                self.state = EngineState::Automatic(AutomaticStep::ScanPa(st));
                return Ok(sent);
            }
            self.boost_settle_until = None;
        }

        if matches!(st.phase, ScanPaPhase::Fine) {
            if let Some(started_at) = st.fine_started_at_secs {
                if let Some(rolling) = rolling_average_since(history, started_at, PA_FAILURE_WINDOW_SECS) {
                    let in_grace_period = st
                        .fine_started_instant
                        .map(|t| t.elapsed() < PA_FAILURE_GRACE_DURATION)
                        .unwrap_or(false);
                    match st.fine_highest_avg_mw {
                        Some(peak) if !in_grace_period && rolling < peak * (1.0 - PA_FAILURE_DROP_FRACTION) => {
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: PA FAILURE -- {PA_FAILURE_WINDOW_SECS}s rolling average ({rolling:.4}mW) fell more than {:.0}% below the peak seen this fine creep ({peak:.4}mW) at vbias_mv={} -- PA likely thermally rolling off, bailing this (level,freq)", PA_FAILURE_DROP_FRACTION * 100.0, st.vbias_mv);
                            self.finish_scan_pa(level, st.vbias_mv, ScanPaOutcome::PaFailure);
                            return Ok(false);
                        }
                        Some(peak) => st.fine_highest_avg_mw = Some(peak.max(rolling)),
                        None => st.fine_highest_avg_mw = Some(rolling),
                    }
                }
            }

            if let Some(settle_until) = st.fine_settle_until {
                if Instant::now() < settle_until {
                    st.wait = None;
                    let mut sent = false;
                    if !throttled {
                        self.send_calibration(link, level, st.vbias_mv)?;
                        self.last_send = Instant::now();
                        sent = true;
                    }
                    self.state = EngineState::Automatic(AutomaticStep::ScanPa(st));
                    return Ok(sent);
                }
                st.fine_settle_until = None;
            }
        }

        if st.wait.is_none() {
            let mut sent = false;
            if !throttled {
                self.send_calibration(link, level, st.vbias_mv)?;
                self.last_send = Instant::now();
                sent = true;
                let (needed, skip) = match st.phase {
                    ScanPaPhase::Fine | ScanPaPhase::CoarseRamp | ScanPaPhase::Settle => (4, 0),
                };
                st.wait = Some(SampleWait::new(now_seq, needed, skip));
            }
            self.state = EngineState::Automatic(AutomaticStep::ScanPa(st));
            return Ok(sent);
        }

        let wait = st.wait.as_ref().unwrap();
        if !wait.ready(now_seq) {
            let mut sent = false;
            if !throttled {
                self.send_calibration(link, level, st.vbias_mv)?;
                self.last_send = Instant::now();
                sent = true;
            }
            self.state = EngineState::Automatic(AutomaticStep::ScanPa(st));
            return Ok(sent);
        }

        let avg_mw = wait.average(history);
        let up = power_up_step(self.sign_inverted);
        let (bound_lo, bound_hi) = self.effective_bounds(level);
        debug!(target: "vtx", "[sweep] ScanPa level={level} freq={freq_mhz}MHz vbias_mv={} avg={avg_mw:.4}mW target={target_mw}mW", st.vbias_mv);

        match st.phase {
            ScanPaPhase::Settle => {
                if avg_mw <= target_mw {
                    debug!(target: "vtx", "[sweep] ScanPa level={level}: settle phase -- reading ({avg_mw:.4}mW) at or below target ({target_mw}mW) at vbias_mv={} -- proceeding to coarse ramp", st.vbias_mv);
                    st.phase = ScanPaPhase::CoarseRamp;
                    st.settle_started_instant = None;
                    st.wait = None;
                    self.pa_enable_settle_pending = false;
                } else if st
                    .settle_started_instant
                    .map(|t| t.elapsed() >= SETTLE_BELOW_TARGET_TIMEOUT)
                    .unwrap_or(true)
                {
                    debug!(target: "vtx", "[sweep] ScanPa level={level}: settle phase timed out after {SETTLE_BELOW_TARGET_TIMEOUT:?} without reading at or below target ({avg_mw:.4}mW > {target_mw}mW) at vbias_mv={} -- proceeding to coarse ramp anyway (its own bailout logic is still the backstop)", st.vbias_mv);
                    st.phase = ScanPaPhase::CoarseRamp;
                    st.settle_started_instant = None;
                    st.settle_timed_out = true;
                    st.wait = None;
                    self.pa_enable_settle_pending = false;
                } else {
                    debug!(target: "vtx", "[sweep] ScanPa level={level}: settle phase -- reading ({avg_mw:.4}mW) still above target ({target_mw}mW) at vbias_mv={}, waiting for it to decay", st.vbias_mv);
                    st.wait = None;
                }
            }
            ScanPaPhase::CoarseRamp => {
                if avg_mw >= target_mw {
                    let Some(fine_start) = st.last_below_target_mv else {
                        if st.settle_timed_out {
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: coarse ramp's starting point still above target ({avg_mw:.4}mW >= {target_mw}mW) at vbias_mv={} right after settle phase timed out -- PA hadn't finished its boost-enable transient in time, not a genuinely unreachable target, skipping to the next frequency", st.vbias_mv);
                            self.finish_scan_pa(level, st.vbias_mv, ScanPaOutcome::NotSettled);
                        } else {
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: coarse ramp's starting point already read at or above target ({avg_mw:.4}mW >= {target_mw}mW) at vbias_mv={} -- no below-target point was ever established, bailing this (level,freq) rather than reporting a false success", st.vbias_mv);
                            self.finish_scan_pa(level, st.vbias_mv, ScanPaOutcome::Uncalibrated);
                        }
                        return Ok(false);
                    };
                    debug!(target: "vtx", "[sweep] ScanPa level={level}: coarse ramp overshot ({avg_mw:.4}mW >= {target_mw}mW) at vbias_mv={} -- entering fine creep from last below-target point vbias_mv={fine_start}, settling {FINE_SETTLE_DELAY:?} before trusting any samples",
                        st.vbias_mv);
                    let margin_mv = {
                        let m = st.vbias_mv.abs() * 25 / 100;
                        if m == 0 { st.coarse_step_mv } else { m }
                    };
                    let padded_bound_mv = (st.vbias_mv + up * margin_mv).clamp(bound_lo, bound_hi);
                    debug!(target: "vtx", "[sweep] ScanPa level={level}: fine creep ceiling padded from vbias_mv={} to vbias_mv={padded_bound_mv} (+{margin_mv}mV toward more power)", st.vbias_mv);
                    st.fine_bound_mv = Some(padded_bound_mv);
                    st.vbias_mv = fine_start;
                    st.phase = ScanPaPhase::Fine;
                    st.wait = None;
                    st.fine_started_at_secs = history.back().map(|e| e.0);
                    st.fine_highest_avg_mw = None;
                    st.fine_settle_until = Some(Instant::now() + FINE_SETTLE_DELAY);
                    st.fine_started_instant = Some(Instant::now());
                } else {
                    let next_step_mv = if avg_mw >= target_mw * 0.90 {
                        let halved = (st.coarse_step_mv / 2).max(1);
                        debug!(target: "vtx", "[sweep] ScanPa level={level}: coarse ramp within 10% of target ({avg_mw:.4}mW >= {:.4}mW) at vbias_mv={} -- halving step to {halved}mV",
                            target_mw * 0.90, st.vbias_mv);
                        st.coarse_step_mv = halved;
                        halved
                    } else {
                        st.coarse_step_mv
                    };
                    if !(bound_lo..=bound_hi).contains(&(st.vbias_mv + up * next_step_mv)) {
                        debug!(target: "vtx", "[sweep] ScanPa level={level}: coarse ramp hit bound [{bound_lo},{bound_hi}] at vbias_mv={} without reaching target -- bailing this (level,freq) as best-effort", st.vbias_mv);
                        self.finish_scan_pa(level, st.vbias_mv, ScanPaOutcome::Uncalibrated);
                        return Ok(false);
                    }
                    st.last_below_target_mv = Some(st.vbias_mv);
                    st.vbias_mv += up * next_step_mv;
                    st.coarse_steps_taken += 1;
                    st.wait = None;
                }
            }
            ScanPaPhase::Fine => {
                let (fine_lo, fine_hi) = match st.fine_bound_mv {
                    Some(b) if up > 0 => (bound_lo, bound_hi.min(b)),
                    Some(b) => (bound_lo.max(b), bound_hi),
                    None => (bound_lo, bound_hi),
                };
                if avg_mw >= target_mw {
                    debug!(target: "vtx", "[sweep] ScanPa level={level}: fine creep converged at vbias_mv={} ({avg_mw:.4}mW)", st.vbias_mv);
                    self.finish_scan_pa(level, st.vbias_mv, ScanPaOutcome::Success);
                    return Ok(false);
                } else if !(fine_lo..=fine_hi).contains(&(st.vbias_mv + up)) {
                    debug!(target: "vtx", "[sweep] ScanPa level={level}: fine creep hit bound [{fine_lo},{fine_hi}] at vbias_mv={} without reaching target ({avg_mw:.4}mW < {target_mw}mW) -- bailing this (level,freq)", st.vbias_mv);
                    self.finish_scan_pa(level, st.vbias_mv, ScanPaOutcome::Uncalibrated);
                    return Ok(false);
                } else {
                    st.vbias_mv += up;
                    st.wait = None;
                }
            }
        }

        self.per_level_status.insert(
            level,
            LevelStatus::InProgress(format!(
                "{} / {} @ {freq_mhz}MHz vbias_mv={}",
                SweepOp::ScanPa.label(),
                match st.phase {
                    ScanPaPhase::Settle => "settling",
                    ScanPaPhase::CoarseRamp => "coarse ramp",
                    ScanPaPhase::Fine => "fine",
                },
                st.vbias_mv
            )),
        );
        self.state = EngineState::Automatic(AutomaticStep::ScanPa(st));
        Ok(false)
    }

    fn tick_scan_detector(
        &mut self,
        link: &mut MspLink,
        history: &VecDeque<(f64, f32)>,
        now_seq: u64,
        throttled: bool,
        level: u8,
        freq_mhz: u16,
        target_mw: f32,
        mut st: ScanDetectorState,
    ) -> anyhow::Result<bool> {
        if let Some(settle_until) = self.boost_settle_until {
            if Instant::now() < settle_until {
                st.wait = None;
                let mut sent = false;
                if !throttled {
                    self.send_calibration(link, level, st.vbias_mv)?;
                    self.last_send = Instant::now();
                    sent = true;
                }
                self.state = EngineState::Automatic(AutomaticStep::ScanDetector(st));
                return Ok(sent);
            }
            self.boost_settle_until = None;
        }

        if st.wait.is_none() {
            let mut sent = false;
            if !throttled {
                self.send_calibration(link, level, st.vbias_mv)?;
                self.last_send = Instant::now();
                sent = true;
                let (needed, skip) = match st.phase {
                    ScanDetectorPhase::Backoff => (1, 0),
                    ScanDetectorPhase::Bracket => (20, 10),
                };
                st.wait = Some(SampleWait::new(now_seq, needed, skip));
            }
            self.state = EngineState::Automatic(AutomaticStep::ScanDetector(st));
            return Ok(sent);
        }

        let wait = st.wait.as_ref().unwrap();
        if !wait.ready(now_seq) {
            let mut sent = false;
            if !throttled {
                self.send_calibration(link, level, st.vbias_mv)?;
                self.last_send = Instant::now();
                sent = true;
            }
            self.state = EngineState::Automatic(AutomaticStep::ScanDetector(st));
            return Ok(sent);
        }

        let Some(reading) = st.last_reading else {
            let mut sent = false;
            if !throttled {
                self.send_calibration(link, level, st.vbias_mv)?;
                self.last_send = Instant::now();
                sent = true;
            }
            self.state = EngineState::Automatic(AutomaticStep::ScanDetector(st));
            return Ok(sent);
        };

        let avg_mw = wait.average(history);
        let up = power_up_step(self.sign_inverted);
        let (bound_lo, bound_hi) = self.effective_bounds(level);

        debug!(target: "vtx", "[sweep] ScanDetector level={level} freq={freq_mhz}MHz vbias_mv={} avg={avg_mw:.4}mW target={target_mw}mW detector={}", reading.vref_mv, reading.detector_mv);

        match st.phase {
            ScanDetectorPhase::Backoff => {
                if avg_mw < target_mw {
                    debug!(target: "vtx", "[sweep] ScanDetector level={level}: backoff crossed below target at vbias_mv={}, entering bracket search", reading.vref_mv);
                    st.phase = ScanDetectorPhase::Bracket;
                    st.wait = None;
                    st.last_reading = None;
                    st.pinned_count = 0;
                } else {
                    let desired = st.vbias_mv - up;
                    let clamped = desired.clamp(bound_lo, bound_hi);
                    st.pinned_count = if desired != clamped { st.pinned_count + 1 } else { 0 };
                    st.vbias_mv = clamped;
                    st.wait = None;
                    st.last_reading = None;
                    if st.pinned_count >= PINNED_LIMIT {
                        debug!(target: "vtx", "[sweep] ScanDetector level={level}: pinned at bound [{bound_lo},{bound_hi}] during backoff without crossing below target -- bailing this (level,freq)");
                        self.finish_scan_detector(level, reading.detector_mv, false);
                        return Ok(false);
                    }
                }
            }
            ScanDetectorPhase::Bracket => {
                let dev = (target_mw * self.tolerance_pct / 100.0).max(0.1);
                let desired = if avg_mw < target_mw - dev {
                    st.vbias_mv + up * 2
                } else if avg_mw > target_mw + dev {
                    st.vbias_mv - up * 2
                } else if avg_mw < target_mw {
                    debug!(target: "vtx", "[sweep] ScanDetector level={level}: bracket 'below' point captured vbias_mv={} avg={avg_mw:.4}mW detector={}", reading.vref_mv, reading.detector_mv);
                    st.below = Some((avg_mw, reading.detector_mv));
                    st.vbias_mv + up
                } else {
                    debug!(target: "vtx", "[sweep] ScanDetector level={level}: bracket 'above' point captured vbias_mv={} avg={avg_mw:.4}mW detector={}", reading.vref_mv, reading.detector_mv);
                    st.above = Some((avg_mw, reading.detector_mv));
                    st.vbias_mv - up
                };
                let clamped = desired.clamp(bound_lo, bound_hi);
                st.pinned_count = if desired != clamped { st.pinned_count + 1 } else { 0 };
                st.vbias_mv = clamped;
                st.wait = None;
                st.last_reading = None;

                if st.pinned_count >= PINNED_LIMIT {
                    debug!(target: "vtx", "[sweep] ScanDetector level={level} freq={freq_mhz}MHz: pinned at bound [{bound_lo},{bound_hi}] for {} attempts, target {target_mw}mW unreachable within the safe limit -- bailing with last-seen detector={} as a rough (not interpolated) fallback",
                            st.pinned_count, reading.detector_mv);
                    self.finish_scan_detector(level, reading.detector_mv, false);
                    return Ok(false);
                }

                if let (Some(below), Some(above)) = (st.below, st.above) {
                    let detector = interpolate(target_mw, below, above);
                    debug!(target: "vtx", "[sweep] ScanDetector level={level} freq={freq_mhz}MHz: interpolated detector={detector} from below={below:?} above={above:?}");
                    self.finish_scan_detector(level, detector, true);
                    return Ok(false);
                }
            }
        }

        self.per_level_status.insert(
            level,
            LevelStatus::InProgress(format!(
                "{} / {} @ {freq_mhz}MHz vbias_mv={}",
                SweepOp::ScanDetector.label(),
                match st.phase {
                    ScanDetectorPhase::Backoff => "backoff",
                    ScanDetectorPhase::Bracket => "bracket",
                },
                st.vbias_mv
            )),
        );

        self.state = EngineState::Automatic(AutomaticStep::ScanDetector(st));
        Ok(false)
    }

    pub fn current_step(&self) -> Option<CurrentStep> {
        let level = self.levels.get(self.level_idx).copied()?;
        if matches!(&self.state, EngineState::Manual) {
            return Some(CurrentStep {
                level,
                freq_idx: self.freq_idx,
                vbias_mv: Some(self.manual_dac_mv),
                detector_mv: None,
                cal_is_current: true,
                det_is_current: true,
            });
        }
        match &self.state {
            EngineState::Automatic(AutomaticStep::ScanPa(st)) => Some(CurrentStep {
                level,
                freq_idx: self.freq_idx,
                vbias_mv: Some(st.vbias_mv),
                detector_mv: None,
                cal_is_current: true,
                det_is_current: false,
            }),
            EngineState::Automatic(AutomaticStep::ScanDetector(st)) => Some(CurrentStep {
                level,
                freq_idx: self.freq_idx,
                vbias_mv: None,
                detector_mv: st.last_reading.map(|r| r.detector_mv as i32),
                cal_is_current: false,
                det_is_current: true,
            }),
            _ => None,
        }
    }

    pub fn debug_state(&self) -> StepDebugInfo {
        match &self.state {
            EngineState::Automatic(AutomaticStep::ScanPa(st)) => {
                let scan_phase = match st.phase {
                    ScanPaPhase::Settle => "Settle",
                    ScanPaPhase::CoarseRamp => "Coarse",
                    ScanPaPhase::Fine => "Fine",
                };
                let drop_detector_active = matches!(st.phase, ScanPaPhase::Fine)
                    && st
                        .fine_started_instant
                        .map(|t| t.elapsed() >= PA_FAILURE_GRACE_DURATION)
                        .unwrap_or(false);
                StepDebugInfo {
                    scan_phase,
                    drop_detector_active,
                    fine_bound_mv: st.fine_bound_mv,
                    fine_highest_avg_mw: st.fine_highest_avg_mw,
                    detector: None,
                }
            }
            EngineState::Automatic(AutomaticStep::ScanDetector(st)) => StepDebugInfo {
                scan_phase: "Inactive",
                drop_detector_active: false,
                fine_bound_mv: None,
                fine_highest_avg_mw: None,
                detector: Some(DetectorDebugInfo {
                    phase: match st.phase {
                        ScanDetectorPhase::Backoff => "Backoff",
                        ScanDetectorPhase::Bracket => "Bracket",
                    },
                    below: st.below,
                    above: st.above,
                    pinned_count: st.pinned_count,
                }),
            },
            _ => StepDebugInfo {
                scan_phase: "Inactive",
                drop_detector_active: false,
                fine_bound_mv: None,
                fine_highest_avg_mw: None,
                detector: None,
            },
        }
    }

    fn effective_bounds(&self, level: u8) -> (i32, i32) {
        let up = power_up_step(self.sign_inverted);
        let (mut lo, mut hi) = (0i32, 3300i32);
        if let Some(&limit) = self.hard_limits.get(&level) {
            if up > 0 {
                hi = hi.min(limit);
            } else {
                lo = lo.max(limit);
            }
        }
        (lo, hi)
    }

    fn safe_state_payload_at_current_point(&self) -> Vec<u8> {
        let level = self.levels.get(self.level_idx).copied().unwrap_or(1);
        let freq_mhz = self.frequencies.get(self.freq_idx).copied().unwrap_or(5800);
        self.build_vtx_config_frequency_payload(freq_mhz, level, true)
    }

    fn auto_resume(&mut self, level: u8, freq_mhz: u16, vbias_mv_at_loss: i32, reason: ConnectionLossReason) {
        let resume = match &self.state {
            EngineState::ConnectionLost { resume, .. } => *resume,
            _ => ResumeMode::Automatic,
        };
        debug!(target: "vtx", "[sweep] connection restored ({reason:?}), resuming: level={level} freq={freq_mhz}MHz resume={resume:?}");
        if matches!(reason, ConnectionLossReason::Vtx | ConnectionLossReason::Both) {
            let up = power_up_step(self.sign_inverted);
            let safe_vbias_mv = (vbias_mv_at_loss - up * HEARTBEAT_BACKOFF_MV).clamp(0, 3300);
            self.hard_limits.insert(level, safe_vbias_mv);
            debug!(target: "vtx", "[sweep] hard limit set to {safe_vbias_mv}mV (backed off {HEARTBEAT_BACKOFF_MV}mV from trip point {vbias_mv_at_loss}mV)");
            match resume {
                ResumeMode::Manual => {
                    self.manual_dac_mv = safe_vbias_mv;
                    self.manual_send_pending = true;
                }
                ResumeMode::Automatic => {}
            }
        } else {
            debug!(target: "vtx", "[sweep] meter-only dropout -- resuming without changing mV or setting a hard limit");
        }
        self.unresponsive_since = None;
        self.state = match resume {
            ResumeMode::Automatic => {
                EngineState::Automatic(AutomaticStep::EnteringPoint)
            }
            ResumeMode::Manual => EngineState::Manual,
        };
        self.pending_frequency_push = Some(freq_mhz);
    }

    pub fn force_connection_lost(&mut self, reason: ConnectionLossReason) {
        if !matches!(&self.state, EngineState::Automatic(_)) {
            return;
        }
        let vbias_mv_at_loss = match &self.state {
            EngineState::Automatic(AutomaticStep::ScanPa(st)) => st.vbias_mv,
            EngineState::Automatic(AutomaticStep::ScanDetector(st)) => st.vbias_mv,
            _ => 0,
        };
        let level = self.levels.get(self.level_idx).copied().unwrap_or(0);
        let freq_mhz = self.frequencies.get(self.freq_idx).copied().unwrap_or(0);
        debug!(target: "vtx", "[sweep] connection forcibly lost ({reason:?}): level={level} freq={freq_mhz}MHz vbias_mv={vbias_mv_at_loss}");
        if !matches!(reason, ConnectionLossReason::Meter) {
            match &self.state {
                EngineState::Automatic(AutomaticStep::ScanPa(_)) => {
                    Self::set_cell_status(&mut self.cal_cell_status, (level, self.freq_idx), CellStatus::LimitHit);
                }
                EngineState::Automatic(AutomaticStep::ScanDetector(_)) => {
                    Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), CellStatus::LimitHit);
                }
                _ => {}
            }
        }
        self.unresponsive_since = None;
        self.state = EngineState::ConnectionLost { level, freq_mhz, vbias_mv_at_loss, reason, resume: ResumeMode::Automatic };
    }

    pub fn clear_hard_limits(&mut self) {
        self.hard_limits.clear();
    }

    pub fn clear_cell_status(&mut self) {
        self.cal_cell_status.clear();
        self.det_cell_status.clear();
    }

    fn set_cell_status(map: &mut HashMap<(u8, usize), CellStatus>, key: (u8, usize), status: CellStatus) {
        if matches!(map.get(&key), Some(CellStatus::LimitHit)) && status != CellStatus::LimitHit {
            return;
        }
        map.insert(key, status);
    }

    fn maybe_trip_connection_lost(&mut self, vtx_ready: bool, meter_ready: bool, vbias_mv_now: i32) -> bool {
        if vtx_ready && meter_ready {
            self.unresponsive_since = None;
            return false;
        }
        let since = *self.unresponsive_since.get_or_insert_with(Instant::now);
        if since.elapsed() <= HEARTBEAT_TIMEOUT {
            return false;
        }
        let level = self.levels.get(self.level_idx).copied().unwrap_or(0);
        let freq_mhz = self.frequencies.get(self.freq_idx).copied().unwrap_or(0);
        let reason = match (vtx_ready, meter_ready) {
            (false, false) => ConnectionLossReason::Both,
            (false, true) => ConnectionLossReason::Vtx,
            (true, false) => ConnectionLossReason::Meter,
            (true, true) => unreachable!("only reached when at least one is false"),
        };
        let resume = if matches!(&self.state, EngineState::Manual) { ResumeMode::Manual } else { ResumeMode::Automatic };
        debug!(target: "vtx", "[sweep] connection lost ({reason:?}) for {:?} -- pausing (level={level} freq={freq_mhz}MHz vbias_mv={vbias_mv_now}, resume={resume:?})",
            since.elapsed());
        if !matches!(reason, ConnectionLossReason::Meter) {
            if matches!(&self.state, EngineState::Manual) {
                Self::set_cell_status(&mut self.cal_cell_status, (level, self.freq_idx), CellStatus::LimitHit);
            } else {
                match &self.state {
                    EngineState::Automatic(AutomaticStep::ScanPa(_)) => {
                        Self::set_cell_status(&mut self.cal_cell_status, (level, self.freq_idx), CellStatus::LimitHit);
                    }
                    EngineState::Automatic(AutomaticStep::ScanDetector(_)) => {
                        Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), CellStatus::LimitHit);
                    }
                    _ => {}
                }
            }
        }
        self.state = EngineState::ConnectionLost { level, freq_mhz, vbias_mv_at_loss: vbias_mv_now, reason, resume };
        true
    }

    fn poll_manual(&mut self, link: &mut MspLink, vtx_ready: bool, meter_ready: bool) -> anyhow::Result<bool> {
        if !link.can_send_now() {
            return Ok(false);
        }
        if self.maybe_trip_connection_lost(vtx_ready, meter_ready, self.manual_dac_mv) {
            return Ok(false);
        }
        if let Some(freq_mhz) = self.pending_frequency_push.take() {
            let level = self.levels.first().copied().unwrap_or(1);
            let payload = self.build_vtx_config_frequency_payload(freq_mhz, level, false);
            link.send_v1(function::VTX_CONFIG as u8, &payload)?;
            link.note_sent(MspCommandKind::Retune);
            debug!(target: "vtx", "[sweep] manual: pushed frequency change to {freq_mhz}MHz (power={level})");
            self.pending_sends.push_back(PendingSend::RequestVtxConfig);
            return Ok(true);
        }
        if !self.manual_send_pending || self.last_send.elapsed() < SEND_INTERVAL {
            return Ok(false);
        }
        let level = self.levels[self.level_idx];
        let vbias_mv = self.manual_dac_mv.clamp(0, 3300) as u16;
        let payload = msp::encode_pa_calibration_request(level, Some(vbias_mv), self.session_active, self.boost_mode.wire_byte());
        link.send_v2(function::SET_PACALIBRATION, Some(&payload))?;
        link.note_sent(MspCommandKind::Calibration);
        self.last_send = Instant::now();
        self.manual_send_pending = false;
        Ok(true)
    }

    fn send_calibration(&mut self, link: &mut MspLink, level: u8, vbias_mv: i32) -> anyhow::Result<()> {
        let (lo, hi) = self.effective_bounds(level);
        let vbias_mv = vbias_mv.clamp(lo, hi) as u16;
        let payload = msp::encode_pa_calibration_request(level, Some(vbias_mv), self.session_active, self.boost_mode.wire_byte());
        link.send_v2(function::SET_PACALIBRATION, Some(&payload))?;
        link.note_sent(MspCommandKind::Calibration);
        Ok(())
    }

    fn finish_scan_pa(&mut self, level: u8, vbias_mv: i32, outcome: ScanPaOutcome) {
        self.completed_steps += 1;
        let success = matches!(outcome, ScanPaOutcome::Success);
        let pa_failure = matches!(outcome, ScanPaOutcome::PaFailure);
        let not_settled = matches!(outcome, ScanPaOutcome::NotSettled);
        self.pending_result = Some(SweepResult {
            level,
            freq_idx: self.freq_idx,
            vbias_mv: Some(vbias_mv.clamp(0, 3300) as u16),
            detector_mv: None,
            success,
            pa_failure,
            not_settled,
        });
        Self::set_cell_status(
            &mut self.cal_cell_status,
            (level, self.freq_idx),
            match outcome {
                ScanPaOutcome::PaFailure => CellStatus::PaFailure,
                ScanPaOutcome::NotSettled => CellStatus::NotSettled,
                ScanPaOutcome::Success => CellStatus::Calibrated,
                ScanPaOutcome::Uncalibrated => CellStatus::Uncalibrated,
            },
        );

        if pa_failure || not_settled {
            let cell_status = if not_settled { CellStatus::NotSettled } else { CellStatus::PaFailure };
            let level_status = if not_settled { LevelStatus::NotSettled } else { LevelStatus::PaFailure };
            Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), cell_status);
            self.completed_steps += 1;
            self.per_level_status.insert(level, level_status);
            self.advance_position();
            return;
        }

        let up = power_up_step(self.sign_inverted);
        let (bound_lo, bound_hi) = self.effective_bounds(level);
        let detector_start_vbias_mv = (vbias_mv - up * 5).clamp(bound_lo, bound_hi);
        self.state = EngineState::Automatic(AutomaticStep::ScanDetector(ScanDetectorState {
            phase: ScanDetectorPhase::Backoff,
            vbias_mv: detector_start_vbias_mv,
            wait: None,
            below: None,
            above: None,
            pinned_count: 0,
            last_reading: None,
        }));
    }

    fn finish_scan_detector(&mut self, level: u8, detector_mv: u16, success: bool) {
        self.completed_steps += 1;
        self.pending_result = Some(SweepResult {
            level,
            freq_idx: self.freq_idx,
            vbias_mv: None,
            detector_mv: Some(detector_mv),
            success,
            pa_failure: false,
            not_settled: false,
        });
        Self::set_cell_status(
            &mut self.det_cell_status,
            (level, self.freq_idx),
            if success { CellStatus::Calibrated } else { CellStatus::Uncalibrated },
        );
        self.per_level_status.insert(level, LevelStatus::Done);
        self.advance_position();
    }

    fn build_vtx_config_frequency_payload(&self, freq_mhz: u16, power: u8, pitmode: bool) -> Vec<u8> {
        let mut p = vec![0u8; 15];
        p[0] = 5;
        p[1] = 0;
        p[2] = 0;
        p[3] = power;
        p[4] = pitmode as u8;
        p[5] = (freq_mhz & 0xff) as u8;
        p[6] = (freq_mhz >> 8) as u8;
        p[7] = 1;
        p[8] = 0;
        p[9] = 0;
        p[10] = 0;
        p[11] = 1;
        p[12] = 0;
        p[13] = 0;
        p[14] = 0;
        p
    }
}

fn interpolate(target_mw: f32, below: (f32, u16), above: (f32, u16)) -> u16 {
    let (p0, d0) = (below.0, below.1 as f32);
    let (p1, d1) = (above.0, above.1 as f32);
    if (p1 - p0).abs() < f32::EPSILON {
        return below.1;
    }
    let t = (target_mw - p0) / (p1 - p0);
    (d0 + t * (d1 - d0)).round().clamp(0.0, 65535.0) as u16
}

pub fn safe_state_payload(table: &VtxTableConfig, selection: &VtxSelectionState) -> Vec<u8> {
    let mut sel = *selection;
    sel.pitmode = true;
    sel.encode_vtx_config_response(table)
}
