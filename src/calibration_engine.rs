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
use crate::power_meter::{nearest_band, FrequencyCapability};
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

/// Status of one (level, frequency) cell in the calibration/detector
/// grids -- separate from LevelStatus (which is per-level, for the
/// Status column), since a level spans 7 frequencies that can each be
/// in a different state. Deliberately has no color/rendering baked in
/// here -- that mapping lives in the UI (pages/calibration.rs), keeping
/// this file free of an egui dependency.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellStatus {
    Default,
    /// Successfully calibrated this run.
    Calibrated,
    /// Actively being worked on right now.
    Current,
    /// This is the specific point where a hard limit was discovered
    /// (the VTX went unresponsive while sitting at this value). Sticky
    /// -- once set, later Calibrated/Uncalibrated outcomes for the same
    /// cell don't overwrite it (see set_cell_status), since the point
    /// of this color is "this is where the trip happened", not "this
    /// is this cell's current state".
    LimitHit,
    /// Could not be properly calibrated -- gave up because a hard limit
    /// (or the physical DAC range) made the target unreachable, or a
    /// search got pinned without ever reaching the tolerance band.
    Uncalibrated,
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

/// Starting point for the coarse ramp -- extracted as a constant since
/// sub_progress() needs the same value to compute how far the ramp has
/// traveled toward the DAC boundary.
const COARSE_RAMP_START_MV: i32 = 3200;
const COARSE_RAMP_STEP_MV: i32 = 50;

enum ScanPaPhase {
    CoarseRamp,
    Backoff,
    Fine,
}

struct ScanPaState {
    phase: ScanPaPhase,
    mv: i32,
    wait: Option<SampleWait>,
    coarse_steps_taken: u32, // how many 50mV coarse-ramp steps so far -- drives sub_progress()
    pinned_count: u32, // consecutive steps clamped at a bound without progress -- see PINNED_LIMIT
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
    pinned_count: u32, // consecutive steps clamped at a bound without progress -- see PINNED_LIMIT
}

enum StepState {
    Pa(ScanPaState),
    Detector(ScanDetectorState),
}

#[derive(Clone, Copy)]
pub enum EngineState {
    Idle,
    AwaitingFreqConfirm { freq_mhz: u16 },
    Running,
    /// No traffic at all from the VTX for longer than a sweep in
    /// progress should ever go quiet -- most likely explanation is a
    /// current-limited supply tripping and power-cycling the VTX.
    /// Paused here (not aborted) so the user can power it back on and
    /// continue; captures where things were when it happened so
    /// resume_after_recovery() can back off to a safe point and set a
    /// hard ceiling on `level` for the rest of the sweep.
    VtxUnresponsive { level: u8, freq_mhz: u16, mv_at_loss: i32 },
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
    /// Per (level, freq_idx) cell status for the calibration[] / detector[]
    /// grid columns -- see CellStatus. Keyed separately since a level's
    /// calibration cell and detector cell for the same frequency are
    /// often in different states at any given moment (e.g. calibration
    /// done/green while detector is current/blue).
    pub cal_cell_status: HashMap<(u8, usize), CellStatus>,
    pub det_cell_status: HashMap<(u8, usize), CellStatus>,
    pub total_steps: usize,
    pub completed_steps: usize,

    pub pending_result: Option<SweepResult>, // set when a (level, freq) finishes; caller drains it into the working table

    meter_capability: FrequencyCapability, // cached from start()'s argument, so poll()'s own frequency-advance logic can honor it too, not just the first frequency
    /// For ManualBand: the last band value actually prompted for --
    /// consecutive VTX frequencies mapping to the same nearest band
    /// don't get a repeat prompt. Reset to None in start().
    last_prompted_band: Option<u32>,
    /// Set whenever the sweep needs to push a new frequency to the VTX
    /// (entering the first frequency in start(), or after confirm_frequency()).
    /// poll() sends this via MSP_VTX_CONFIG at the top of its next call,
    /// then clears it. This was missing entirely in the first version of
    /// this file -- the sweep only ever sent SET_PACALIBRATION (selecting
    /// a power level + DAC mv), never actually retuned the VTX, so it
    /// stayed on whatever frequency it was last configured to.
    pending_frequency_push: Option<u16>,
    /// Set whenever a ProgrammableBand/FullyProgrammable meter needs its
    /// own frequency changed too -- worker.rs drains this and calls
    /// PowerMeter::set_frequency(), then clears it. Unlike
    /// pending_frequency_push (which the engine can act on directly via
    /// its own MspLink), the engine has no handle to the PowerMeter
    /// itself, so this is a request rather than an action.
    pub pending_meter_frequency: Option<u32>,

    /// Per-level mV ceiling discovered when the VTX went unresponsive
    /// mid-scan (see EngineState::VtxUnresponsive / resume_after_recovery).
    /// pub so the UI can show it in a table column; cleared by
    /// clear_hard_limits() when the PA table is Refreshed.
    pub hard_limits: HashMap<u8, i32>,
    /// First time this poll() saw the VTX go quiet while Running, or
    /// None if it's currently responsive. A SUSTAINED silence (not a
    /// single missed tick) is what actually triggers VtxUnresponsive --
    /// see HEARTBEAT_TIMEOUT.
    unresponsive_since: Option<Instant>,
}

/// How long the VTX can go completely silent while a sweep is Running
/// before it's treated as having lost power.
///
/// TUNED DOWN from an initial 2s: the actual failure mode observed was
/// that during the detection window, the sweep kept stepping mV further
/// in the "more power" direction, because once the VTX dies the RF
/// signal disappears and the power meter reads near-zero -- which the
/// algorithm reads as "need more power, keep pushing" rather than "the
/// device is gone". By the time VtxUnresponsive fired, mv_at_loss could
/// already be well past the actual trip point, so backing off a small
/// margin from THAT point wasn't actually safe -- the reported "sets a
/// limit, trips again, sets a new limit" cycle is exactly what that
/// looks like. Shortening this reduces (but for a genuinely fast
/// hardware overcurrent trip, can't fully eliminate) how far the sweep
/// can wander past the real limit before noticing.
const HEARTBEAT_TIMEOUT: Duration = Duration::from_millis(500);
/// How far to back off (in the safe/less-power direction) from the mV
/// value that was active when the VTX went unresponsive, when setting
/// the new hard limit for that level. Increased alongside the shorter
/// HEARTBEAT_TIMEOUT above, as a second, independent margin of safety.
const HEARTBEAT_BACKOFF_MV: i32 = 50;
/// How many consecutive steps can get clamped at a bound (DAC range or
/// a hard limit) without the reading ever reaching the target/tolerance
/// band before giving up on that (level, freq) as unreachable. Without
/// this, a target that's genuinely unreachable within a hard limit
/// (confirmed by a real run: ScanDetector pinned at a limit for many
/// minutes, since the target was never getting anywhere close) spins
/// forever with no exit condition.
const PINNED_LIMIT: u32 = 5;

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
            cal_cell_status: HashMap::new(),
            det_cell_status: HashMap::new(),
            total_steps,
            completed_steps: 0,
            pending_result: None,
            meter_capability: FrequencyCapability::Manual { min_mhz: 0, max_mhz: 0 }, // set for real in start()
            last_prompted_band: None,
            pending_frequency_push: None,
            pending_meter_frequency: None,
            hard_limits: HashMap::new(),
            unresponsive_since: None,
        }
    }

    pub fn progress(&self) -> f32 {
        if self.total_steps == 0 {
            return 1.0;
        }
        self.completed_steps as f32 / self.total_steps as f32
    }

    /// Call once to begin. See begin_frequency() for how `capability`
    /// determines whether this starts Running immediately or pauses on
    /// AwaitingFreqConfirm.
    pub fn start(&mut self, capability: FrequencyCapability) {
        if self.levels.is_empty() || self.frequencies.is_empty() {
            debug!(target: "vtx", "[sweep] start() called with no levels/frequencies -- not starting");
            self.state = EngineState::Idle;
            return;
        }
        self.freq_idx = 0;
        self.level_idx = 0;
        self.step = None;
        self.last_prompted_band = None;
        self.meter_capability = capability;
        debug!(target: "vtx", "[sweep] starting: {} levels {:?}, {} frequencies {:?}, tolerance={}%, sign_inverted={}, sweep_hz={}, meter_capability={:?}",
            self.levels.len(), self.levels, self.frequencies.len(), self.frequencies, self.tolerance_pct, self.sign_inverted, self.sweep_hz, self.meter_capability);
        let first_freq = self.frequencies[0];
        self.begin_frequency(first_freq);
    }

    /// Decides what happens when the sweep is about to start working on
    /// `freq_mhz`, based on the meter's capability -- shared by start()
    /// (the first frequency) and poll()'s own advance-to-next-frequency
    /// logic (every frequency after that), so both honor the same rules:
    ///   Manual: always prompt (no concept of "band" to consolidate by).
    ///   ManualBand: prompt with the nearest band, UNLESS it's the same
    ///     band as the last prompt (consecutive VTX frequencies that map
    ///     to the same band don't re-prompt).
    ///   ProgrammableBand: no prompt -- request the meter retune to the
    ///     nearest band via pending_meter_frequency.
    ///   FullyProgrammable: no prompt -- request the exact frequency via
    ///     pending_meter_frequency.
    /// Either way, pending_frequency_push is always set, since the VTX
    /// itself needs retuning regardless of what the meter needs.
    fn begin_frequency(&mut self, freq_mhz: u16) {
        match self.meter_capability.clone() {
            FrequencyCapability::Manual { .. } => {
                self.state = EngineState::AwaitingFreqConfirm { freq_mhz };
            }
            FrequencyCapability::ManualBand { bands_mhz } => {
                let band = nearest_band(&bands_mhz, freq_mhz as u32);
                if Some(band) == self.last_prompted_band {
                    debug!(target: "vtx", "[sweep] {freq_mhz}MHz maps to the same band ({band}MHz) as the last prompt -- skipping prompt");
                    self.state = EngineState::Running;
                    self.pending_frequency_push = Some(freq_mhz);
                } else {
                    self.last_prompted_band = Some(band);
                    self.state = EngineState::AwaitingFreqConfirm { freq_mhz };
                }
            }
            FrequencyCapability::ProgrammableBand { bands_mhz } => {
                let band = nearest_band(&bands_mhz, freq_mhz as u32);
                debug!(target: "vtx", "[sweep] requesting meter retune to nearest band {band}MHz (for VTX freq {freq_mhz}MHz)");
                self.pending_meter_frequency = Some(band);
                self.state = EngineState::Running;
                self.pending_frequency_push = Some(freq_mhz);
            }
            FrequencyCapability::FullyProgrammable { .. } => {
                debug!(target: "vtx", "[sweep] requesting meter retune to {freq_mhz}MHz");
                self.pending_meter_frequency = Some(freq_mhz as u32);
                self.state = EngineState::Running;
                self.pending_frequency_push = Some(freq_mhz);
            }
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
    /// arrived this tick; `vtx_ready` is SharedState::vtx_ready -- true
    /// if ANY frame (not just ones addressed to the sweep) has been seen
    /// from the VTX recently, used to detect a current-limited supply
    /// power-cycling it mid-sweep.
    pub fn poll(
        &mut self,
        link: &mut MspLink,
        history: &VecDeque<(f64, f32)>,
        reading_seq: u64,
        latest_reading: Option<msp::PaCalibrationReading>,
        vtx_ready: bool,
    ) -> anyhow::Result<()> {
        if !matches!(self.state, EngineState::Running) {
            return Ok(());
        }

        if vtx_ready {
            self.unresponsive_since = None;
        } else {
            let since = *self.unresponsive_since.get_or_insert_with(Instant::now);
            if since.elapsed() > HEARTBEAT_TIMEOUT {
                let mv_at_loss = match &self.step {
                    Some(StepState::Pa(st)) => st.mv,
                    Some(StepState::Detector(st)) => st.mv,
                    None => 0,
                };
                let level = self.levels.get(self.level_idx).copied().unwrap_or(0);
                let freq_mhz = self.frequencies.get(self.freq_idx).copied().unwrap_or(0);
                debug!(target: "vtx", "[sweep] no response from VTX for {:?} -- pausing (level={level} freq={freq_mhz}MHz mv={mv_at_loss}), likely a power-limited supply tripping",
                    since.elapsed());
                match &self.step {
                    Some(StepState::Pa(_)) => {
                        Self::set_cell_status(&mut self.cal_cell_status, (level, self.freq_idx), CellStatus::LimitHit);
                    }
                    Some(StepState::Detector(_)) => {
                        Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), CellStatus::LimitHit);
                    }
                    None => {}
                }
                self.state = EngineState::VtxUnresponsive { level, freq_mhz, mv_at_loss };
                return Ok(());
            }
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
            self.begin_frequency(next_freq);
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
            self.step = Some(StepState::Pa(ScanPaState {
                phase: ScanPaPhase::CoarseRamp,
                mv: COARSE_RAMP_START_MV,
                wait: None,
                coarse_steps_taken: 0,
                pinned_count: 0,
            }));
            debug!(target: "vtx", "[sweep] level={level} freq={freq_mhz}MHz target={target_mw}mW: starting ScanPa (coarse ramp) from mv={COARSE_RAMP_START_MV}");
            self.per_level_status.insert(
                level,
                LevelStatus::InProgress(format!("{} / coarse ramp @ {freq_mhz}MHz", SweepOp::ScanPa.label())),
            );
            Self::set_cell_status(&mut self.cal_cell_status, (level, self.freq_idx), CellStatus::Current);
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
                let (bound_lo, bound_hi) = self.effective_bounds(level);
                debug!(target: "vtx", "[sweep] ScanPa level={level} freq={freq_mhz}MHz mv={} avg={avg_mw:.4}mW target={target_mw}mW", st.mv);

                match st.phase {
                    ScanPaPhase::CoarseRamp => {
                        if avg_mw >= target_mw * 0.80 {
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: coarse ramp reached 80% ({avg_mw:.4}mW >= {:.4}mW) at mv={}, entering backoff", target_mw * 0.80, st.mv);
                            st.phase = ScanPaPhase::Backoff;
                            st.wait = None;
                        } else if !(bound_lo..=bound_hi).contains(&(st.mv + up * COARSE_RAMP_STEP_MV)) {
                            // Ran off the end of the allowed range (DAC bound, or a hard limit from
                            // a previous VTX power-loss on this level) without reaching 80% -- bail
                            // this (level, freq) as best-effort.
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: coarse ramp hit bound [{bound_lo},{bound_hi}] at mv={} without reaching 80% target -- bailing this (level,freq) as best-effort", st.mv);
                            self.finish_scan_pa(level, st.mv, false);
                            return Ok(());
                        } else {
                            st.mv += up * COARSE_RAMP_STEP_MV;
                            st.coarse_steps_taken += 1;
                            st.wait = None;
                        }
                    }
                    ScanPaPhase::Backoff => {
                        if avg_mw < target_mw {
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: backoff crossed below target at mv={}, entering fine creep", st.mv);
                            st.phase = ScanPaPhase::Fine;
                            st.wait = None;
                            st.pinned_count = 0;
                        } else {
                            let desired = st.mv - up * 5;
                            let clamped = desired.clamp(bound_lo, bound_hi);
                            st.pinned_count = if desired != clamped { st.pinned_count + 1 } else { 0 };
                            st.mv = clamped;
                            st.wait = None;
                            if st.pinned_count >= PINNED_LIMIT {
                                debug!(target: "vtx", "[sweep] ScanPa level={level}: pinned at bound [{bound_lo},{bound_hi}] during backoff without crossing below target -- bailing this (level,freq)");
                                self.finish_scan_pa(level, st.mv, false);
                                return Ok(());
                            }
                        }
                    }
                    ScanPaPhase::Fine => {
                        if avg_mw >= target_mw {
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: fine creep converged at mv={} ({avg_mw:.4}mW)", st.mv);
                            self.finish_scan_pa(level, st.mv, true);
                            return Ok(());
                        } else if !(bound_lo..=bound_hi).contains(&(st.mv + up)) {
                            debug!(target: "vtx", "[sweep] ScanPa level={level}: fine creep hit bound [{bound_lo},{bound_hi}] at mv={} without reaching target ({avg_mw:.4}mW < {target_mw}mW) -- bailing this (level,freq)", st.mv);
                            self.finish_scan_pa(level, st.mv, false);
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
                let (bound_lo, bound_hi) = self.effective_bounds(level);
                let detector_now = latest_reading.map(|r| r.detector_mv).unwrap_or(0);
                debug!(target: "vtx", "[sweep] ScanDetector level={level} freq={freq_mhz}MHz mv={} avg={avg_mw:.4}mW target={target_mw}mW detector={detector_now}", st.mv);

                match st.phase {
                    ScanDetectorPhase::Backoff => {
                        if avg_mw < target_mw {
                            debug!(target: "vtx", "[sweep] ScanDetector level={level}: backoff crossed below target at mv={}, entering bracket search", st.mv);
                            st.phase = ScanDetectorPhase::Bracket;
                            st.wait = None;
                            st.pinned_count = 0;
                        } else {
                            let desired = st.mv - up;
                            let clamped = desired.clamp(bound_lo, bound_hi);
                            st.pinned_count = if desired != clamped { st.pinned_count + 1 } else { 0 };
                            st.mv = clamped;
                            st.wait = None;
                            if st.pinned_count >= PINNED_LIMIT {
                                debug!(target: "vtx", "[sweep] ScanDetector level={level}: pinned at bound [{bound_lo},{bound_hi}] during backoff without crossing below target -- bailing this (level,freq)");
                                self.finish_scan_detector(level, detector_now, false);
                                return Ok(());
                            }
                        }
                    }
                    ScanDetectorPhase::Bracket => {
                        let dev = (target_mw * self.tolerance_pct / 100.0).max(0.1);
                        let desired = if avg_mw < target_mw - dev {
                            st.mv + up * 2
                        } else if avg_mw > target_mw + dev {
                            st.mv - up * 2
                        } else if avg_mw < target_mw {
                            debug!(target: "vtx", "[sweep] ScanDetector level={level}: bracket 'below' point captured mv={} avg={avg_mw:.4}mW detector={detector_now}", st.mv);
                            st.below = Some((avg_mw, detector_now));
                            st.mv + up
                        } else {
                            debug!(target: "vtx", "[sweep] ScanDetector level={level}: bracket 'above' point captured mv={} avg={avg_mw:.4}mW detector={detector_now}", st.mv);
                            st.above = Some((avg_mw, detector_now));
                            st.mv - up
                        };
                        let clamped = desired.clamp(bound_lo, bound_hi);
                        // The critical case this closes: with a hard limit active,
                        // the target can be genuinely unreachable within the safe
                        // range (confirmed by a real run: this got clamped at the
                        // same bound for the whole rest of the scan, since the
                        // reading was nowhere near target and never could be).
                        // Without counting consecutive clamps, that has no exit
                        // condition and spins forever.
                        st.pinned_count = if desired != clamped { st.pinned_count + 1 } else { 0 };
                        st.mv = clamped;
                        st.wait = None;

                        if st.pinned_count >= PINNED_LIMIT {
                            debug!(target: "vtx", "[sweep] ScanDetector level={level} freq={freq_mhz}MHz: pinned at bound [{bound_lo},{bound_hi}] for {} attempts, target {target_mw}mW unreachable within the safe limit -- bailing with last-seen detector={detector_now} as a rough (not interpolated) fallback",
                                st.pinned_count);
                            self.finish_scan_detector(level, detector_now, false);
                            return Ok(());
                        }

                        if let (Some(below), Some(above)) = (st.below, st.above) {
                            let detector = interpolate(target_mw, below, above);
                            debug!(target: "vtx", "[sweep] ScanDetector level={level} freq={freq_mhz}MHz: interpolated detector={detector} from below={below:?} above={above:?}");
                            self.finish_scan_detector(level, detector, true);
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
    /// searches with no fixed step count to measure against), EXCEPT for
    /// CoarseRamp: since it steps in known, fixed 50mV increments toward
    /// a known boundary, the fraction of that distance already covered
    /// gives a real (if approximate -- we don't know in advance how many
    /// steps it'll actually take to reach 80% target) sense of movement,
    /// rather than a static placeholder. Returns (0.0, "") when nothing
    /// is in progress.
    pub fn sub_progress(&self) -> (f32, &'static str) {
        match &self.step {
            None => (0.0, ""),
            Some(StepState::Pa(st)) => match st.phase {
                ScanPaPhase::CoarseRamp => {
                    let up = power_up_step(self.sign_inverted);
                    let level = self.levels.get(self.level_idx).copied().unwrap_or(0);
                    let (bound_lo, bound_hi) = self.effective_bounds(level);
                    let boundary = if up > 0 { bound_hi } else { bound_lo };
                    let total_possible = ((boundary - COARSE_RAMP_START_MV) as f32 / COARSE_RAMP_STEP_MV as f32)
                        .abs()
                        .max(1.0);
                    let frac = (st.coarse_steps_taken as f32 / total_possible).clamp(0.0, 1.0);
                    (0.4 * frac, "ScanPa: coarse ramp")
                }
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

    /// (min_mv, max_mv) for `level` -- the full 0..=3300 DAC range,
    /// narrowed on whichever side is "more power" if a hard limit was
    /// set for this level (see EngineState::VtxUnresponsive /
    /// resume_after_recovery).
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

    /// Called after the user confirms the VTX is back (SharedState::vtx_ready
    /// true) and clicks Continue. Sets a hard limit for the level that was
    /// active when the VTX went quiet -- backed off HEARTBEAT_BACKOFF_MV
    /// from the trip point, in the safe (less power) direction -- backs
    /// the in-progress step off to that same safe point (continuing right
    /// at the trip point would just trip it again), and re-pushes the
    /// current frequency, since the VTX's own reboot means it needs
    /// retuning and level reselection from scratch.
    pub fn resume_after_recovery(&mut self) {
        if let EngineState::VtxUnresponsive { level, freq_mhz, mv_at_loss } = self.state {
            let up = power_up_step(self.sign_inverted);
            let safe_mv = (mv_at_loss - up * HEARTBEAT_BACKOFF_MV).clamp(0, 3300);
            self.hard_limits.insert(level, safe_mv);
            debug!(target: "vtx", "[sweep] resuming after VTX recovery: level={level} freq={freq_mhz}MHz, hard limit set to {safe_mv}mV (backed off {HEARTBEAT_BACKOFF_MV}mV from trip point {mv_at_loss}mV)");
            match &mut self.step {
                Some(StepState::Pa(st)) => {
                    st.mv = safe_mv;
                    st.wait = None;
                }
                Some(StepState::Detector(st)) => {
                    st.mv = safe_mv;
                    st.wait = None;
                }
                None => {}
            }
            self.unresponsive_since = None;
            self.state = EngineState::Running;
            self.pending_frequency_push = Some(freq_mhz);
        }
    }

    /// Discards any hard limits discovered so far -- called when the PA
    /// table is Refreshed, per the explicit requirement that a refresh
    /// should forget them.
    pub fn clear_hard_limits(&mut self) {
        self.hard_limits.clear();
    }

    /// Resets every cell in the calibration/detector grid back to
    /// Default. Called on Refresh (alongside clear_hard_limits) -- per
    /// spec, a Refresh should forget what a previous run marked, not
    /// just the hard limits it found.
    pub fn clear_cell_status(&mut self) {
        self.cal_cell_status.clear();
        self.det_cell_status.clear();
    }

    /// Sets `map[key] = status`, except LimitHit is sticky -- once a
    /// cell is marked as the point where a trip happened, a later
    /// Calibrated/Uncalibrated/Current outcome for that same cell
    /// doesn't overwrite it (the point of that color is "this is where
    /// it happened", not "this cell's current state").
    fn set_cell_status(map: &mut HashMap<(u8, usize), CellStatus>, key: (u8, usize), status: CellStatus) {
        if matches!(map.get(&key), Some(CellStatus::LimitHit)) && status != CellStatus::LimitHit {
            return;
        }
        map.insert(key, status);
    }

    fn send_calibration(&self, link: &mut MspLink, level: u8, mv: i32) -> anyhow::Result<()> {
        let (lo, hi) = self.effective_bounds(level);
        let mv = mv.clamp(lo, hi) as u16;
        link.send_v2(function::SET_PACALIBRATION, Some(&msp::encode_pa_calibration_request(level, Some(mv))))
    }

    fn finish_scan_pa(&mut self, level: u8, mv: i32, success: bool) {
        self.completed_steps += 1;
        self.pending_result = Some(SweepResult {
            level,
            freq_idx: self.freq_idx,
            calibration_mv: Some(mv.clamp(0, 3300) as u16),
            detector_mv: None,
        });
        Self::set_cell_status(
            &mut self.cal_cell_status,
            (level, self.freq_idx),
            if success { CellStatus::Calibrated } else { CellStatus::Uncalibrated },
        );
        // ScanDetector runs immediately after ScanPa for the same (level, freq).
        // Starting point is a small step toward LESS power from the just-found
        // calibration value (matches the original script's intent: start
        // slightly on the safe side before searching for where the reading
        // crosses target). BUG FIXED here: the original script (and this
        // file's first version) used a hardcoded "mv - 5", which only means
        // "less power" under normal polarity -- on an inverted board (like
        // our confirmed RTC76401) that's actually 5mV toward MORE power. A
        // real run showed this seeding ScanDetector 5mV past a hard limit
        // that had just been set. Now direction-aware via power_up_step(),
        // and clamped to the current bounds regardless.
        let up = power_up_step(self.sign_inverted);
        let (bound_lo, bound_hi) = self.effective_bounds(level);
        let detector_start_mv = (mv - up * 5).clamp(bound_lo, bound_hi);
        Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), CellStatus::Current);
        self.step = Some(StepState::Detector(ScanDetectorState {
            phase: ScanDetectorPhase::Backoff,
            mv: detector_start_mv,
            wait: None,
            below: None,
            above: None,
            pinned_count: 0,
        }));
    }

    fn finish_scan_detector(&mut self, level: u8, detector_mv: u16, success: bool) {
        self.completed_steps += 1;
        self.pending_result = Some(SweepResult {
            level,
            freq_idx: self.freq_idx,
            calibration_mv: None,
            detector_mv: Some(detector_mv),
        });
        Self::set_cell_status(
            &mut self.det_cell_status,
            (level, self.freq_idx),
            if success { CellStatus::Calibrated } else { CellStatus::Uncalibrated },
        );
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
