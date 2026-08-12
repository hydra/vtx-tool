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
use log::debug;
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

/// Tracks "wait for `needed` new power readings since this was created".
///
/// BUG FIX: this used to compare `power_history.len()` before/after --
/// which works only until the rolling HISTORY_WINDOW_SECS window fills
/// up. Once it's full, every push evicts an old entry, so len() stops
/// growing and just hovers at a roughly constant size -- `ready()` would
/// then never return true again, which is exactly what produced the
/// "stuck at mv=950" symptom (the sweep was genuinely waiting forever
/// for a length increase that could never happen once the plot had been
/// running past the 60s window). Now tracked against SharedState's
/// reading_seq instead -- a plain counter incremented on every reading,
/// which never plateaus.
struct SampleWait {
    start_seq: u64,
    needed: usize,
    skip_first: usize, // for the bracket phase: skip this many, average the rest
}

impl SampleWait {
    fn new(current_seq: u64, needed: usize, skip_first: usize) -> Self {
        Self { start_seq: current_seq, needed, skip_first }
    }

    fn ready(&self, current_seq: u64) -> bool {
        current_seq >= self.start_seq + self.needed as u64
    }

    /// Average of the `needed - skip_first` newest samples in `history`
    /// (the oldest `skip_first` of the window are discarded as settling
    /// time). Still correct against the rolling window -- taking the N
    /// most recent entries by recency was never the broken part, only
    /// the "how many new ones arrived" tracking above was.
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

    requires_manual_frequency: bool, // cached from start()'s argument, so poll()'s own frequency-advance logic can honor it too, not just the first frequency
    /// Set whenever the sweep needs to push a new frequency to the VTX
    /// (entering the first frequency in start(), or after confirm_frequency()).
    /// poll() sends this via MSP_VTX_CONFIG at the top of its next call,
    /// then clears it. This was missing entirely in the first version of
    /// this file -- the sweep only ever sent SET_PACALIBRATION (selecting
    /// a power level + DAC mv), never actually retuned the VTX, so it
    /// stayed on whatever frequency it was last configured to.
    pending_frequency_push: Option<u16>,
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
            requires_manual_frequency: false, // set for real in start()
            pending_frequency_push: None,
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
            debug!(target: "vtx", "[sweep] start() called with no levels/frequencies -- not starting");
            self.state = EngineState::Idle;
            return;
        }
        self.freq_idx = 0;
        self.level_idx = 0;
        self.step = None;
        self.requires_manual_frequency = requires_manual_frequency;
        debug!(target: "vtx", "[sweep] starting: {} levels {:?}, {} frequencies {:?}, tolerance={}%, sign_inverted={}, sweep_hz={}, manual_freq={requires_manual_frequency}",
            self.levels.len(), self.levels, self.frequencies.len(), self.frequencies, self.tolerance_pct, self.sign_inverted, self.sweep_hz);
        if requires_manual_frequency {
            self.state = EngineState::AwaitingFreqConfirm { freq_mhz: self.frequencies[0] };
        } else {
            self.state = EngineState::Running;
            self.pending_frequency_push = Some(self.frequencies[0]);
        }
    }

    /// UI calls this after the user confirms they've retuned the meter.
    pub fn confirm_frequency(&mut self) {
        if let EngineState::AwaitingFreqConfirm { freq_mhz } = self.state {
            debug!(target: "vtx", "[sweep] frequency confirmed, resuming at {freq_mhz}MHz");
            self.state = EngineState::Running;
            self.pending_frequency_push = Some(freq_mhz);
        }
    }

    /// Begins a safe-state abort -- caller (worker.rs) is expected to
    /// have already sent a pitmode-forced VTX_CONFIG; this just marks
    /// remaining levels as Aborted and stops the engine.
    pub fn abort(&mut self) {
        debug!(target: "vtx", "[sweep] aborted at level={:?} freq_idx={} ({}/{} steps completed)",
            self.levels.get(self.level_idx), self.freq_idx, self.completed_steps, self.total_steps);
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
    /// reading buffer (for averaging); `reading_seq` is
    /// SharedState::reading_seq, a monotonic counter used for "has a new
    /// reading arrived" instead of history.len() (see SampleWait's doc
    /// comment for why that distinction matters); `latest_reading` is
    /// the most recent decoded MSP_PACALIBRATION response, if one
    /// arrived this tick (worker.rs is responsible for routing that
    /// frame here).
    pub fn poll(
        &mut self,
        link: &mut MspLink,
        history: &VecDeque<(f64, f32)>,
        reading_seq: u64,
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
                debug!(target: "vtx", "[sweep] all frequencies complete");
                return Ok(());
            }
            let next_freq = self.frequencies[self.freq_idx];
            debug!(target: "vtx", "[sweep] frequency {}/{} complete, next: {next_freq}MHz",
                self.freq_idx, self.frequencies.len());
            if self.requires_manual_frequency {
                self.state = EngineState::AwaitingFreqConfirm { freq_mhz: next_freq };
            } else {
                self.state = EngineState::Running;
                self.pending_frequency_push = Some(next_freq);
            }
            return Ok(());
        }

        if let Some(freq_mhz) = self.pending_frequency_push.take() {
            let level = self.levels.first().copied().unwrap_or(1);
            let payload = build_vtx_config_frequency_payload(freq_mhz, level);
            link.send_v1(function::VTX_CONFIG as u8, &payload)?;
            debug!(target: "vtx", "[sweep] pushed frequency change to {freq_mhz}MHz (power={level})");
            return Ok(()); // let the retune land before the next tick starts sending SET_PACALIBRATION
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
            debug!(target: "vtx", "[sweep] level={level} freq={freq_mhz}MHz target={target_mw}mW: starting ScanPa (coarse ramp) from mv={start_mv}");
            self.per_level_status.insert(
                level,
                LevelStatus::InProgress(format!("{} / coarse ramp @ {freq_mhz}MHz", SweepOp::ScanPa.label())),
            );
        }

        let now_seq = reading_seq;
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
                        st.wait = Some(SampleWait::new(now_seq, needed, skip));
                    }
                    self.step = Some(StepState::Pa(st));
                    return Ok(());
                }

                let wait = st.wait.as_ref().unwrap();
                if !wait.ready(now_seq) {
                    if !throttled {
                        self.send_calibration(link, level, st.mv)?;
                        self.last_send = Instant::now();
                    }
                    self.step = Some(StepState::Pa(st));
                    return Ok(());
                }

                let avg_mw = wait.average(history);
                let up = power_up_step(self.sign_inverted);
                debug!(target: "vtx", "[sweep] ScanPa level={level} freq={freq_mhz}MHz mv={} avg={avg_mw:.4}mW target={target_mw}mW", st.mv);

                match st.phase {
                    ScanPaPhase::CoarseRamp => {
                        if avg_mw >= target_mw * 0.80 {
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: coarse ramp reached 80% ({avg_mw:.4}mW >= {:.4}mW) at mv={}, entering backoff", target_mw * 0.80, st.mv);
                            st.phase = ScanPaPhase::Backoff;
                            st.wait = None;
                        } else if !(0..=3300).contains(&(st.mv + up * 50)) {
                            // Ran off the end of the DAC range without reaching 80% -- bail this (level, freq) as best-effort.
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: coarse ramp hit DAC bound at mv={} without reaching 80% target -- bailing this (level,freq) as best-effort", st.mv);
                            self.finish_scan_pa(level, st.mv);
                            return Ok(());
                        } else {
                            st.mv += up * 50;
                            st.wait = None;
                        }
                    }
                    ScanPaPhase::Backoff => {
                        if avg_mw < target_mw {
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: backoff crossed below target at mv={}, entering fine creep", st.mv);
                            st.phase = ScanPaPhase::Fine;
                            st.wait = None;
                        } else {
                            st.mv -= up * 5;
                            st.wait = None;
                        }
                    }
                    ScanPaPhase::Fine => {
                        if avg_mw >= target_mw || !(0..=3300).contains(&(st.mv + up)) {
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: fine creep converged at mv={} ({avg_mw:.4}mW)", st.mv);
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
                        st.wait = Some(SampleWait::new(now_seq, needed, skip));
                    }
                    self.step = Some(StepState::Detector(st));
                    return Ok(());
                }

                let wait = st.wait.as_ref().unwrap();
                if !wait.ready(now_seq) {
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
                debug!(target: "vtx", "[sweep] ScanDetector level={level} freq={freq_mhz}MHz mv={} avg={avg_mw:.4}mW target={target_mw}mW detector={detector_now}", st.mv);

                match st.phase {
                    ScanDetectorPhase::Backoff => {
                        if avg_mw < target_mw {
                            debug!(target: "vtx", "[sweep] ScanDetector level={level}: backoff crossed below target at mv={}, entering bracket search", st.mv);
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
                            debug!(target: "vtx", "[sweep] ScanDetector level={level}: bracket 'below' point captured mv={} avg={avg_mw:.4}mW detector={detector_now}", st.mv);
                            st.below = Some((avg_mw, detector_now));
                            st.mv += up;
                        } else {
                            debug!(target: "vtx", "[sweep] ScanDetector level={level}: bracket 'above' point captured mv={} avg={avg_mw:.4}mW detector={detector_now}", st.mv);
                            st.above = Some((avg_mw, detector_now));
                            st.mv -= up;
                        }
                        st.wait = None;

                        if let (Some(below), Some(above)) = (st.below, st.above) {
                            let detector = interpolate(target_mw, below, above);
                            debug!(target: "vtx", "[sweep] ScanDetector level={level} freq={freq_mhz}MHz: interpolated detector={detector} from below={below:?} above={above:?}");
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

    /// Progress within the CURRENT (level, freq, op) step only -- coarse,
    /// phase-block-based (not fine-grained, since these are open-ended
    /// searches with no fixed step count to measure against). Returns
    /// (0.0, "") when nothing is in progress.
    pub fn sub_progress(&self) -> (f32, &'static str) {
        match &self.step {
            None => (0.0, ""),
            Some(StepState::Pa(st)) => match st.phase {
                ScanPaPhase::CoarseRamp => (0.15, "ScanPa: coarse ramp"),
                ScanPaPhase::Backoff => (0.55, "ScanPa: backoff"),
                ScanPaPhase::Fine => (0.85, "ScanPa: fine creep"),
            },
            Some(StepState::Detector(st)) => match st.phase {
                ScanDetectorPhase::Backoff => (0.20, "ScanDetector: backoff"),
                ScanDetectorPhase::Bracket => {
                    // A little extra resolution within Bracket: having
                    // captured one of the two brackets already is
                    // meaningfully further along than having neither.
                    let extra = match (st.below.is_some(), st.above.is_some()) {
                        (false, false) => 0.0,
                        _ => 0.35,
                    };
                    (0.45 + extra, "ScanDetector: bracket search")
                }
            },
        }
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

/// Builds an MSP_VTX_CONFIG payload that retunes the VTX to `freq_mhz`
/// directly (band=0) -- independent of the stored VtxTableConfig's
/// "Selected" state (a separate, user-facing concept the sweep
/// shouldn't disturb). `power` just needs to be a valid level index so
/// vtx_apply_hw() picks a sensible RTC6705 register while retuning; the
/// SET_PACALIBRATION calls that immediately follow set the real level
/// and DAC value regardless (and re-set the RTC6705 register too, via
/// vtx_msp_set_calibration()), so this doesn't need to be exact.
fn build_vtx_config_frequency_payload(freq_mhz: u16, power: u8) -> Vec<u8> {
    let mut p = vec![0u8; 15];
    p[0] = 5; // VTXDEV_MSP
    p[1] = 0; // band=0 -> use the raw frequency field directly
    p[2] = 0; // channel (unused when band=0)
    p[3] = power;
    p[4] = 0; // pitmode=false, matches rf_calibration.py's own convention throughout scanPa/scanDetector
    p[5] = (freq_mhz & 0xff) as u8;
    p[6] = (freq_mhz >> 8) as u8;
    p[7] = 1; // device_ready
    p[8] = 0; // low_power_disarm
    p[9] = 0;
    p[10] = 0; // pit_mode_freq
    p[11] = 1; // vtx_table_available -- MUST be nonzero or handle_msp_set_vtx_config() ignores the whole push
    p[12] = 0; // band_count -- not meaningful for a direct-frequency push; a mismatch just makes the VTX re-broadcast its own table afterward, harmless
    p[13] = 0; // channel_count
    p[14] = 0; // power_level_count
    p
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
