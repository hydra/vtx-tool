//! Calibration sweep engine, ported from rf_calibration.py's
//! scanPa/scanDetector, generalized for DAC polarity and a
//! user-configurable tolerance, and restructured as a step-advanced
//! state machine rather than a blocking async loop -- the worker's main
//! loop calls `poll()` once per tick, so meter polling and the passive
//! VTX_CONFIG responder keep running throughout a sweep that can take
//! several minutes.
//!
//! ORDERING: frequency-major, not level-major (per explicit requirement)
//! -- freq 1 on every selected level, then freq 2 on every selected
//! level, etc., so the user only has to retune a manual-frequency power
//! meter once per frequency rather than once per (level, frequency)
//! pair.
//!
//! READINGS: rather than managing its own separate meter-read cadence
//! (like the original script's dedicated sleep/poll loop), this rides
//! the SAME power_history the graph already displays -- "wait for K
//! readings" means "wait for K new entries in power_history since the
//! DAC value was last changed", so the sweep and the graph are always
//! looking at identical data (satisfies "add all power meter results to
//! the graph" -- there's only one stream, not a side channel).
//!
//! POLARITY: PA_DAC_SIGN is read off the PA calibration table's index-0
//! entry (dac_sign_inverted, see msp.rs) rather than assumed -- every
//! "step toward more power" in this file goes through power_up_step()
//! rather than a hardcoded `+=`.

use crate::msp::{self, function, MspLink};
use crate::vtxtable::VtxTableConfig;
use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};

/// +1 if increasing the DAC mV increases RF power (normal/typical wiring),
/// -1 if decreasing DAC mV increases RF power (inverted, e.g. RTC76401 --
/// PA_DAC_SIGN > 0 in firmware terms). Note this is the OPPOSITE
/// convention from PA_DAC_SIGN itself (which describes a PID error-
/// correction sign) -- this describes literal "make the number bigger or
/// smaller to get more power" for the sweep's arithmetic.
fn power_up_step(sign_inverted: bool) -> i32 {
    if sign_inverted {
        -1
    } else {
        1
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
    InProgress(String), // pre-formatted "ScanPa/coarse @5800MHz mv=2900"
    Done,
    Aborted,
}

/// Tracks "wait for `needed` new power_history samples since this was
/// created", then hands back their average.
struct SampleWait {
    start_len: usize,
    needed: usize,
    skip_first: usize, // for the bracket phase: skip this many, average the rest
}

impl SampleWait {
    fn new(current_len: usize, needed: usize, skip_first: usize) -> Self {
        Self { start_len: current_len, needed, skip_first }
    }

    fn ready(&self, current_len: usize) -> bool {
        current_len >= self.start_len + self.needed
    }

    /// Average of the `needed - skip_first` newest samples collected
    /// since this wait started (the oldest `skip_first` of the window
    /// are discarded as settling time).
    fn average(&self, history: &VecDeque<(f64, f32)>) -> f32 {
        let take = self.needed - self.skip_first;
        let sum: f32 = history.iter().rev().take(take).map(|&(_, mw)| mw).sum();
        sum / take.max(1) as f32
    }
}

enum ScanPaPhase {
    CoarseRamp,
    Backoff,
    Fine,
}

struct ScanPaState {
    phase: ScanPaPhase,
    mv: i32,
    wait: Option<SampleWait>,
}

enum ScanDetectorPhase {
    Backoff,
    Bracket,
}

struct ScanDetectorState {
    phase: ScanDetectorPhase,
    mv: i32,
    wait: Option<SampleWait>,
    below: Option<(f32, u16)>, // (power_mw, detector_mv)
    above: Option<(f32, u16)>,
}

enum StepState {
    Pa(ScanPaState),
    Detector(ScanDetectorState),
}

pub enum EngineState {
    Idle,
    AwaitingFreqConfirm { freq_mhz: u16 },
    Running,
}

/// One (level, frequency) result as it's produced -- applied to the
/// working PA table immediately so "Send to VTX" always reflects
/// whatever's completed so far, even mid-sweep.
pub struct SweepResult {
    pub level: u8,
    pub freq_idx: usize,
    pub calibration_mv: Option<u16>,
    pub detector_mv: Option<u16>,
}

pub struct SweepEngine {
    pub levels: Vec<u8>,       // selected levels, ascending
    pub frequencies: Vec<u16>, // from PA table idx=0's value[] (frequency breakpoints)
    pub tolerance_pct: f32,    // scanDetector's bracketing tolerance only (per explicit scope)
    pub sign_inverted: bool,
    pub target_mw_by_level: HashMap<u8, u16>,
    pub sweep_hz: f64, // what update_hz was set to for the sweep -- worker.rs restores the previous value on finish/abort

    pub state: EngineState,
    freq_idx: usize,
    level_idx: usize, // index into `levels`
    step: Option<StepState>,
    last_send: Instant,

    pub per_level_status: HashMap<u8, LevelStatus>,
    pub total_steps: usize,
    pub completed_steps: usize,

    pub pending_result: Option<SweepResult>, // set when a (level, freq) finishes; caller drains it into the working table
}

const SEND_INTERVAL: Duration = Duration::from_millis(100); // throttles resends while waiting on a step, independent of the outer 10ms loop tick

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
        let total_steps = levels.len() * frequencies.len() * 2; // 2 ops (ScanPa, ScanDetector) per (level, freq)
        Self {
            levels,
            frequencies,
            tolerance_pct,
            sign_inverted,
            target_mw_by_level,
            sweep_hz: 10.0_f64.min(meter_max_hz as f64), // "0.1s if possible, else the meter's fastest" -- see module doc
            state: EngineState::Idle,
            freq_idx: 0,
            level_idx: 0,
            step: None,
            last_send: Instant::now() - SEND_INTERVAL,
            per_level_status,
            total_steps,
            completed_steps: 0,
            pending_result: None,
        }
    }

    pub fn progress(&self) -> f32 {
        if self.total_steps == 0 {
            return 1.0;
        }
        self.completed_steps as f32 / self.total_steps as f32
    }

    /// Call once to begin. If the meter needs manual frequency changes,
    /// starts in AwaitingFreqConfirm rather than Running.
    pub fn start(&mut self, requires_manual_frequency: bool) {
        if self.levels.is_empty() || self.frequencies.is_empty() {
            self.state = EngineState::Idle;
            return;
        }
        self.freq_idx = 0;
        self.level_idx = 0;
        self.step = None;
        if requires_manual_frequency {
            self.state = EngineState::AwaitingFreqConfirm { freq_mhz: self.frequencies[0] };
        } else {
            self.state = EngineState::Running;
        }
    }

    /// UI calls this after the user confirms they've retuned the meter.
    pub fn confirm_frequency(&mut self) {
        if matches!(self.state, EngineState::AwaitingFreqConfirm { .. }) {
            self.state = EngineState::Running;
        }
    }

    /// Begins a safe-state abort -- caller (worker.rs) is expected to
    /// have already sent a pitmode-forced VTX_CONFIG; this just marks
    /// remaining levels as Aborted and stops the engine.
    pub fn abort(&mut self) {
        for status in self.per_level_status.values_mut() {
            if !matches!(status, LevelStatus::Done) {
                *status = LevelStatus::Aborted;
            }
        }
        self.state = EngineState::Idle;
        self.step = None;
    }

    pub fn is_active(&self) -> bool {
        !matches!(self.state, EngineState::Idle)
    }

    /// Advances the sweep by whatever's possible this tick. `link` is
    /// used to send SET_PACALIBRATION; `history` is the live power
    /// reading buffer; `latest_reading` is the most recent decoded
    /// MSP_PACALIBRATION response, if one arrived this tick (worker.rs
    /// is responsible for routing that frame here).
    pub fn poll(
        &mut self,
        link: &mut MspLink,
        history: &VecDeque<(f64, f32)>,
        latest_reading: Option<msp::PaCalibrationReading>,
    ) -> anyhow::Result<()> {
        if !matches!(self.state, EngineState::Running) {
            return Ok(());
        }

        if self.level_idx >= self.levels.len() {
            // Finished this frequency's levels -- advance to the next frequency.
            self.freq_idx += 1;
            self.level_idx = 0;
            self.step = None;
            if self.freq_idx >= self.frequencies.len() {
                self.state = EngineState::Idle; // whole sweep done
                return Ok(());
            }
            self.state = EngineState::AwaitingFreqConfirm { freq_mhz: self.frequencies[self.freq_idx] };
            return Ok(());
        }

        let level = self.levels[self.level_idx];
        let freq_mhz = self.frequencies[self.freq_idx];
        let target_mw = *self.target_mw_by_level.get(&level).unwrap_or(&0) as f32;

        if self.step.is_none() {
            // Entering a fresh (level, freq): start with ScanPa's coarse ramp.
            let start_mv = 3200i32; // safe/near-off starting point, matches VTX_BIAS_OFF_MV's neighborhood
            self.step = Some(StepState::Pa(ScanPaState {
                phase: ScanPaPhase::CoarseRamp,
                mv: start_mv,
                wait: None,
            }));
            self.per_level_status.insert(
                level,
                LevelStatus::InProgress(format!("{} / coarse ramp @ {freq_mhz}MHz", SweepOp::ScanPa.label())),
            );
        }

        let now_len = history.len();
        let throttled = self.last_send.elapsed() < SEND_INTERVAL;

        match self.step.take().unwrap() {
            StepState::Pa(mut st) => {
                if st.wait.is_none() {
                    if !throttled {
                        self.send_calibration(link, level, st.mv)?;
                        self.last_send = Instant::now();
                        let (needed, skip) = match st.phase {
                            ScanPaPhase::Fine | ScanPaPhase::CoarseRamp => (4, 0),
                            ScanPaPhase::Backoff => (1, 0),
                        };
                        st.wait = Some(SampleWait::new(now_len, needed, skip));
                    }
                    self.step = Some(StepState::Pa(st));
                    return Ok(());
                }

                let wait = st.wait.as_ref().unwrap();
                if !wait.ready(now_len) {
                    if !throttled {
                        self.send_calibration(link, level, st.mv)?;
                        self.last_send = Instant::now();
                    }
                    self.step = Some(StepState::Pa(st));
                    return Ok(());
                }

                let avg_mw = wait.average(history);
                let up = power_up_step(self.sign_inverted);

                match st.phase {
                    ScanPaPhase::CoarseRamp => {
                        if avg_mw >= target_mw * 0.80 {
                            st.phase = ScanPaPhase::Backoff;
                            st.wait = None;
                        } else if !(0..=3300).contains(&(st.mv + up * 50)) {
                            // Ran off the end of the DAC range without reaching 80% -- bail this (level, freq) as best-effort.
                            self.finish_scan_pa(level, st.mv);
                            return Ok(());
                        } else {
                            st.mv += up * 50;
                            st.wait = None;
                        }
                    }
                    ScanPaPhase::Backoff => {
                        if avg_mw < target_mw {
                            st.phase = ScanPaPhase::Fine;
                            st.wait = None;
                        } else {
                            st.mv -= up * 5;
                            st.wait = None;
                        }
                    }
                    ScanPaPhase::Fine => {
                        if avg_mw >= target_mw || !(0..=3300).contains(&(st.mv + up)) {
                            self.finish_scan_pa(level, st.mv);
                            return Ok(());
                        } else {
                            st.mv += up;
                            st.wait = None;
                        }
                    }
                }

                self.per_level_status.insert(
                    level,
                    LevelStatus::InProgress(format!(
                        "{} / {} @ {freq_mhz}MHz mv={}",
                        SweepOp::ScanPa.label(),
                        match st.phase {
                            ScanPaPhase::CoarseRamp => "coarse ramp",
                            ScanPaPhase::Backoff => "backoff",
                            ScanPaPhase::Fine => "fine",
                        },
                        st.mv
                    )),
                );
                self.step = Some(StepState::Pa(st));
            }

            StepState::Detector(mut st) => {
                if st.wait.is_none() {
                    if !throttled {
                        self.send_calibration(link, level, st.mv)?;
                        self.last_send = Instant::now();
                        let (needed, skip) = match st.phase {
                            ScanDetectorPhase::Backoff => (1, 0),
                            ScanDetectorPhase::Bracket => (20, 10),
                        };
                        st.wait = Some(SampleWait::new(now_len, needed, skip));
                    }
                    self.step = Some(StepState::Detector(st));
                    return Ok(());
                }

                let wait = st.wait.as_ref().unwrap();
                if !wait.ready(now_len) {
                    if !throttled {
                        self.send_calibration(link, level, st.mv)?;
                        self.last_send = Instant::now();
                    }
                    self.step = Some(StepState::Detector(st));
                    return Ok(());
                }

                let avg_mw = wait.average(history);
                let up = power_up_step(self.sign_inverted);
                let detector_now = latest_reading.map(|r| r.detector_mv).unwrap_or(0);

                match st.phase {
                    ScanDetectorPhase::Backoff => {
                        if avg_mw < target_mw {
                            st.phase = ScanDetectorPhase::Bracket;
                            st.wait = None;
                        } else {
                            st.mv -= up;
                            st.wait = None;
                        }
                    }
                    ScanDetectorPhase::Bracket => {
                        let dev = (target_mw * self.tolerance_pct / 100.0).max(0.1);
                        if avg_mw < target_mw - dev {
                            st.mv += up * 2;
                        } else if avg_mw > target_mw + dev {
                            st.mv -= up * 2;
                        } else if avg_mw < target_mw {
                            st.below = Some((avg_mw, detector_now));
                            st.mv += up;
                        } else {
                            st.above = Some((avg_mw, detector_now));
                            st.mv -= up;
                        }
                        st.wait = None;

                        if let (Some(below), Some(above)) = (st.below, st.above) {
                            let detector = interpolate(target_mw, below, above);
                            self.finish_scan_detector(level, detector);
                            return Ok(());
                        }
                    }
                }

                self.per_level_status.insert(
                    level,
                    LevelStatus::InProgress(format!(
                        "{} / {} @ {freq_mhz}MHz mv={}",
                        SweepOp::ScanDetector.label(),
                        match st.phase {
                            ScanDetectorPhase::Backoff => "backoff",
                            ScanDetectorPhase::Bracket => "bracket",
                        },
                        st.mv
                    )),
                );
                self.step = Some(StepState::Detector(st));
            }
        }

        Ok(())
    }

    fn send_calibration(&self, link: &mut MspLink, level: u8, mv: i32) -> anyhow::Result<()> {
        let mv = mv.clamp(0, 3300) as u16;
        link.send_v2(function::SET_PACALIBRATION, Some(&msp::encode_pa_calibration_request(level, Some(mv))))
    }

    fn finish_scan_pa(&mut self, level: u8, mv: i32) {
        self.completed_steps += 1;
        self.pending_result = Some(SweepResult {
            level,
            freq_idx: self.freq_idx,
            calibration_mv: Some(mv.clamp(0, 3300) as u16),
            detector_mv: None,
        });
        // ScanDetector runs immediately after ScanPa for the same (level, freq).
        self.step = Some(StepState::Detector(ScanDetectorState {
            phase: ScanDetectorPhase::Backoff,
            mv: (mv - 5).max(0),
            wait: None,
            below: None,
            above: None,
        }));
    }

    fn finish_scan_detector(&mut self, level: u8, detector_mv: u16) {
        self.completed_steps += 1;
        self.pending_result = Some(SweepResult {
            level,
            freq_idx: self.freq_idx,
            calibration_mv: None,
            detector_mv: Some(detector_mv),
        });
        self.step = None;
        self.level_idx += 1;
        self.per_level_status.insert(level, LevelStatus::Done);
    }
}

/// Linear interpolation of the detector value at exactly `target_mw`,
/// between a (power, detector) pair below target and one above it.
fn interpolate(target_mw: f32, below: (f32, u16), above: (f32, u16)) -> u16 {
    let (p0, d0) = (below.0, below.1 as f32);
    let (p1, d1) = (above.0, above.1 as f32);
    if (p1 - p0).abs() < f32::EPSILON {
        return below.1;
    }
    let t = (target_mw - p0) / (p1 - p0);
    (d0 + t * (d1 - d0)).round().clamp(0.0, 65535.0) as u16
}

/// Builds a one-off MSP_VTX_CONFIG payload with pitmode forced on,
/// without mutating the stored VtxTableConfig -- used for the
/// safe-state-on-abort push (see worker.rs's Command::AbortSweep).
pub fn safe_state_payload(vtx_table: &VtxTableConfig) -> Vec<u8> {
    let mut cfg = vtx_table.clone();
    cfg.pitmode = true;
    cfg.encode_vtx_config_response()
}
