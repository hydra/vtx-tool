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
//!
//! STATE MACHINE SHAPE: the engine holds exactly ONE position
//! (level_idx/freq_idx) regardless of which mode is driving it --
//! Automatic and Manual both read and advance the SAME fields, through
//! the SAME advance_position() function, rather than each mode tracking
//! its own copy. Frequency retuning only ever happens from inside
//! begin_frequency(), itself only ever called from start()/
//! start_manual() (the very first frequency) and advance_position()
//! (every frequency after that) -- there is exactly one place in this
//! file that ever changes what frequency the VTX is tuned to.
//!
//! Which sub-mode is active is expressed entirely by which EngineState
//! variant self.state currently holds (Automatic(..) or Manual) --
//! there is no separate "in_manual_mode" flag traveling alongside it.
//! AwaitingFreqConfirm and ConnectionLost, the two states reachable from
//! either mode, carry a `resume: ResumeMode` field recording which one
//! to return to, rather than relying on a flag that lives outside the
//! state and has to be kept in sync with it by hand.

use crate::msp::{self, function, MspCommandKind, MspLink};
use crate::power_meter::{nearest_band, FrequencyCapability};
use crate::vtxtable::{VtxSelectionState, VtxTableConfig};
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

/// The coarse ramp's starting point -- the safe/low-power end of the
/// effective range for THIS board's sign, backed off
/// COARSE_RAMP_START_MARGIN_MV so it isn't sitting exactly on the
/// boundary. For a normal board (sign_inverted=false, up>0: higher
/// vbias_mv means more power) that's near bound_lo; for an inverted
/// board (sign_inverted=true, up<0: lower vbias_mv means more power)
/// that's near bound_hi -- the same "which end is safe" logic
/// sub_progress() already applies to its own "boundary" (the opposite
/// end, where the ramp is headed TOWARD), just applied to the end it
/// starts FROM instead.
fn coarse_ramp_start_vbias_mv(sign_inverted: bool, bound_lo: i32, bound_hi: i32) -> i32 {
    let up = power_up_step(sign_inverted);
    if up > 0 {
        (bound_lo + COARSE_RAMP_START_MARGIN_MV).min(bound_hi)
    } else {
        (bound_hi - COARSE_RAMP_START_MARGIN_MV).max(bound_lo)
    }
}

/// Average of `history` readings that occurred both (a) at or after
/// `since_secs` (the point this belongs to -- readings from a PREVIOUS
/// frequency or level must never leak into this) and (b) within the
/// last `window_secs` seconds, using history's own elapsed-seconds time
/// base (not wall-clock) so this needs no separate clock reference.
/// Returns None if there isn't yet a full window's worth of same-point
/// history -- comparing a real peak against a partial/short window
/// right as Fine creep begins would risk a false-positive trigger.
fn rolling_average_since(history: &VecDeque<(f64, f32)>, since_secs: f64, window_secs: f64) -> Option<f32> {
    let now = history.back()?.0;
    let window_start = (now - window_secs).max(since_secs);
    if now - window_start < window_secs - 0.01 {
        return None; // not enough same-point history yet to fill the window
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
    InProgress(String), // pre-formatted "ScanPa/coarse @5800MHz vbias_mv=2900"
    Done,
    Aborted,
    Skipped,
    PaFailure,
    /// ScanPaPhase::Settle never got a genuine below-target reading
    /// within SETTLE_BELOW_TARGET_TIMEOUT of boost turning on -- see
    /// ScanPaOutcome::NotSettled's own doc comment for why this is kept
    /// distinct from PaFailure and Uncalibrated.
    NotSettled,
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
    /// User deliberately skipped this point (see SweepEngine::skip_multiple)
    /// -- distinct from Uncalibrated, which means a search actually ran
    /// and failed to converge; this means no search ran at all.
    Skipped,
    /// Set directly by the user via Manual mode's DAC slider, rather
    /// than found by an automatic search -- distinct from Calibrated so
    /// the UI can show which cells were hand-set.
    Manual,
    /// Fine creep aborted because the rolling power-meter average
    /// dropped below its own peak recorded during this creep -- the PA
    /// is very likely thermally rolling off under sustained drive
    /// rather than the target being genuinely unreachable. Distinct
    /// from Uncalibrated (which covers "hit a bound" or "pinned"
    /// failures) so the UI can flag this as a hardware condition worth
    /// investigating, not just a calibration search that didn't
    /// converge.
    PaFailure,
    /// ScanPaPhase::Settle never got a genuine below-target reading
    /// within SETTLE_BELOW_TARGET_TIMEOUT of boost turning on -- distinct
    /// from both Uncalibrated (a search actually ran and failed to
    /// converge -- this means no search ever ran, CoarseRamp's own
    /// bailout fired on its very first check) and PaFailure (a
    /// convergING search that then thermally rolled off -- this is the
    /// PA never having finished warming up in the first place). See
    /// ScanPaOutcome::NotSettled's own doc comment.
    NotSettled,
}

/// Why a ScanPa step ended, for finish_scan_pa() -- determines both the
/// cell/level status shown in the UI and whether ScanDetector runs at
/// all for this (level, freq).
enum ScanPaOutcome {
    /// Found a value that reads at or below target -- ScanDetector runs
    /// next, same (level, freq).
    Success,
    /// A search actually ran (a genuine below-target point was found and
    /// stepped from) but never converged -- hit a bound, or similar.
    /// ScanDetector still runs, since the failure is about vbias, not
    /// about whether the PA responds usably at all.
    Uncalibrated,
    /// Fine creep's own rolling-average check caught the PA thermally
    /// rolling off under sustained drive. ScanDetector skipped entirely
    /// -- driving further for a detector search only makes this worse.
    PaFailure,
    /// ScanPaPhase::Settle never got a genuine below-target reading
    /// within SETTLE_BELOW_TARGET_TIMEOUT of boost turning on -- distinct
    /// from Uncalibrated: no search ever actually ran here (CoarseRamp's
    /// own bailout fired on its very first check), so this isn't a
    /// verdict on whether the target is reachable at all, just that the
    /// PA hadn't finished its boost-enable transient in time.
    /// ScanDetector skipped for the same reason as PaFailure -- there's
    /// nothing stable for it to search against yet.
    NotSettled,
}

/// Tracks "wait for `needed` new power readings since this was created".
///
/// BUG FIX: this used to compare `power_history.len()` before/after --
/// which works only until the rolling HISTORY_WINDOW_SECS window fills
/// up. Once it's full, every push evicts an old entry, so len() stops
/// growing and just hovers at a roughly constant size -- `ready()` would
/// then never return true again, which is exactly what produced the
/// "stuck at vbias_mv=950" symptom (the sweep was genuinely waiting forever
/// for a length increase that could never happen once the plot had been
/// running past the 60s window). Now tracked against SharedState's
/// reading_seq instead -- a plain counter incremented on every reading,
/// which never plateaus.
pub(crate) struct SampleWait {
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

/// Margin the coarse ramp starts away from the DAC boundary that's safe
/// (low-power) for this board's sign -- NOT a fixed DAC value, since
/// "safe/low-power" is a different physical end of the range depending
/// on sign_inverted (see coarse_ramp_start_vbias_mv()). Matches the
/// previous fixed 3200 (100mV shy of the 3300mV default max) for
/// inverted boards, where that was correct; now derived symmetrically
/// for either sign rather than assumed.
const COARSE_RAMP_START_MARGIN_MV: i32 = 0;
const COARSE_RAMP_STEP_MV: i32 = 25;
/// Rolling window used by Fine creep's thermal-rolloff check -- see
/// rolling_average_since() and the Fine match arm's doc comment.
const PA_FAILURE_WINDOW_SECS: f64 = 3.0;
/// How far (as a fraction of the peak) the rolling average has to fall
/// below its own recorded peak before Fine creep treats it as thermal
/// rolloff rather than ordinary meter noise. A real run showed a
/// genuinely converging creep (5.7mW climbing smoothly to 9.06mW, target
/// 10mW) get aborted by a rolling-average dip of ~0.2% below its own
/// peak -- well within this same log's routine reading-to-reading
/// jitter (commonly 0.5-1%). 3% is comfortably above that noise floor
/// while still catching a real decline (the worked "very thermally
/// limited" example dropped from a 94% peak down to 89%, a ~5 point --
/// not a fractional -- decline, well past this threshold).
const PA_FAILURE_DROP_FRACTION: f32 = 0.03;
/// How long to wait, after CoarseRamp hands off to Fine, before trusting
/// any samples for Fine's own convergence check -- see the CoarseRamp
/// match arm's doc comment for why. A real run on a Skyworks SE5004L
/// board showed the PA's actual output measurably lagging the DAC value
/// change at this handoff (capacitors in the VBIAS circuit), and
/// SampleWait's own 4-sample gate is purely count-based -- a fast meter
/// can satisfy it well before the hardware has physically caught up.
const FINE_SETTLE_DELAY: Duration = Duration::from_secs(2);
/// How long after Fine creep begins before the PA-Failure (thermal
/// rolloff) check is actually armed. A real run showed the drop-
/// detection tracking firing within the first few seconds of Fine
/// starting -- well before the PA had any chance to genuinely thermally
/// roll off, on ordinary settling/ramp-up behavior instead. This is
/// deliberately much longer than FINE_SETTLE_DELAY above (a different,
/// narrower concern -- letting the DAC's own physical settle finish) so
/// there's real margin against early-Fine transients of any kind before
/// a drop actually causes an abort.
const PA_FAILURE_GRACE_DURATION: Duration = Duration::from_secs(15);
/// How long to ignore samples after the PA's boost stage is observed
/// transitioning from off to on -- separate from FINE_SETTLE_DELAY
/// above (that one covers a DAC value change specifically at the
/// CoarseRamp->Fine handoff; this covers the PA actually powering up at
/// all, which can happen at the start of ANY step -- ScanPa's coarse
/// ramp, Fine creep, or ScanDetector's own search -- whenever this is
/// the first point after a level/frequency change and the level's
/// ext_pa_enable first takes effect, or Manual mode's PA checkbox is
/// switched on). A real run showed the coarse ramp's very first sample,
/// taken the instant boost turned on, read as an immediate false
/// overshoot -- before the PA had any chance to actually stabilize.
const BOOST_ENABLE_SETTLE_DELAY: Duration = Duration::from_secs(2);
/// Default timeout for ScanPaPhase::Settle -- how long to actively wait
/// for the PA's own boost-enable power spike to decay to or below
/// target before giving up and proceeding to CoarseRamp anyway. A real
/// run on a board with the OPAMP-buffered VBIAS circuit reinstated
/// showed this transient (independent of DAC value -- manually holding
/// the DAC completely fixed still showed power slowly decaying over
/// several seconds after boost-on) still reading 14-21mW well past
/// BOOST_ENABLE_SETTLE_DELAY's fixed 2s, for a 5mW target, at
/// vbias_mv=0 -- exactly the situation CoarseRamp's own "starting point
/// already at/above target" bailout exists to catch (see that match
/// arm's own doc comment), but here the cause was this decaying
/// transient, not a genuinely-unreachable target: the same board's own
/// bench sweep (DAC held fixed, no PA) showed 5mW was easily reachable
/// once the reading actually settled.
///
/// 15s was a first attempt and turned out not to be enough -- a real
/// run showed the decay still clearly trending downward (16.6mW ->
/// 14.5mW over that window, temperature still climbing throughout, no
/// sign of leveling off) when the timeout hit, so CoarseRamp's own
/// bailout fired on a transient that hadn't finished yet, not a
/// genuinely unreachable target. This is worse at the low end of the
/// power range (5mW), where the gap between "transient" and "target" is
/// largest and takes longest to close. Confirmed safe to wait this long
/// at vbias_mv=0 -- the OPAMP-buffered circuit is fine sitting there for
/// 30-60+ seconds.
const SETTLE_BELOW_TARGET_TIMEOUT: Duration = Duration::from_secs(60);
/// How long the VTX (or meter) can go completely silent while a sweep is
/// active before it's treated as a lost connection.
///
/// TUNED DOWN from an initial 2s: the actual failure mode observed was
/// that during the detection window, the sweep kept stepping mV further
/// in the "more power" direction, because once the VTX dies the RF
/// signal disappears and the power meter reads near-zero -- which the
/// algorithm reads as "need more power, keep pushing" rather than "the
/// device is gone". By the time ConnectionLost fired, vbias_mv_at_loss could
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

/// Which sub-mode to return to once a shared, mode-agnostic pause
/// (AwaitingFreqConfirm, ConnectionLost) resolves. Carried as DATA on
/// those two state variants themselves, rather than a separate
/// "in_manual_mode" flag living outside EngineState that every
/// transition would otherwise have to remember to keep in sync by hand
/// -- the state that needs the answer now always has it right there.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResumeMode {
    Automatic,
    Manual,
}

pub(crate) enum ScanPaPhase {
    /// Runs first, before CoarseRamp ever takes a step -- see
    /// SETTLE_BELOW_TARGET_TIMEOUT's own doc comment for why. Holds
    /// vbias_mv fixed at the coarse ramp's own starting point (set once
    /// at entry, never touched by this phase) and just watches the
    /// reading: once it's at or below target, hands off to CoarseRamp
    /// exactly as if this phase had never run (same starting vbias_mv,
    /// still-fresh coarse-ramp state). If the reading is already at or
    /// below target the very first time this phase checks, there's
    /// nothing to wait for -- hands off immediately, adding no delay to
    /// the common case where this transient either doesn't apply or has
    /// already decayed by the time this (level, freq) is reached (e.g.
    /// every point after the first, since the PA stays on across the
    /// whole sweep). Bounded by SETTLE_BELOW_TARGET_TIMEOUT -- if the
    /// reading hasn't decayed to target by then, proceeds to CoarseRamp
    /// anyway, which still has its own bailout logic as a backstop for
    /// a starting point that's genuinely, not just transiently, too high.
    Settle,
    CoarseRamp,
    Fine,
}

pub(crate) struct ScanPaState {
    phase: ScanPaPhase,
    vbias_mv: i32,
    wait: Option<SampleWait>,
    /// Wall-clock instant Settle phase began, for
    /// SETTLE_BELOW_TARGET_TIMEOUT -- separate from fine_started_instant
    /// below (a different phase's own timing). None once Settle has
    /// handed off to CoarseRamp; never read again after that point.
    settle_started_instant: Option<Instant>,
    /// True only if THIS point's Settle phase resolved via
    /// SETTLE_BELOW_TARGET_TIMEOUT rather than a genuine below-target
    /// reading -- read once, by CoarseRamp's own "starting point already
    /// at/above target" bail (see that match arm's doc comment), to
    /// attribute that specific failure to ScanPaOutcome::NotSettled
    /// instead of the generic Uncalibrated. False whenever Settle didn't
    /// run at all for this point (pa_enable_settle_pending was already
    /// consumed by an earlier point since the same boost-enable event).
    settle_timed_out: bool,
    coarse_steps_taken: u32, // how many coarse-ramp steps so far -- drives sub_progress()
    /// Last CoarseRamp vbias_mv confirmed to read below target, remembered so
    /// that the moment CoarseRamp overshoots, Fine creep can start from
    /// there directly -- see the CoarseRamp match arm's doc comment for
    /// why (replaced a separate Backoff phase that stepped AWAY from an
    /// overshoot in fixed 5mV chunks, which a real run showed skipping
    /// clean past the target region when the RF response is steep).
    last_below_target_mv: Option<i32>,
    /// Current coarse-ramp step size -- starts at COARSE_RAMP_STEP_MV,
    /// halves (floor 1mV) each time a step reads within 10% of target
    /// without yet overshooting it. See the CoarseRamp match arm: this
    /// keeps coarse ramp taking real, MEASURED steps -- narrower ones as
    /// it gets close -- until it actually observes an overshoot, rather
    /// than stopping at a percentage threshold and handing off to Fine
    /// on an assumption.
    coarse_step_mv: i32,
    /// Tight ceiling (toward more power) for Fine creep, computed once
    /// when CoarseRamp hands off -- see the CoarseRamp match arm's doc
    /// comment. None until that handoff happens; Fine always has one by
    /// the time it starts stepping.
    fine_bound_mv: Option<i32>,
    /// history's own elapsed-seconds timestamp when Fine phase began --
    /// readings from before this belong to a previous point (frequency
    /// or level) and must never feed the thermal-rolloff check below.
    fine_started_at_secs: Option<f64>,
    /// Highest PA_FAILURE_WINDOW_SECS rolling average mW seen so far
    /// during this Fine creep -- see the Fine match arm's doc comment
    /// and rolling_average_since().
    fine_highest_avg_mw: Option<f32>,
    /// Wall-clock deadline for FINE_SETTLE_DELAY -- set the moment
    /// CoarseRamp hands off to Fine, cleared once elapsed. While Some
    /// and not yet elapsed, Fine resends the DAC value (so it's
    /// definitely in flight) but discards any completed SampleWait
    /// rather than using it for the convergence check -- see the Fine
    /// match arm's doc comment.
    fine_settle_until: Option<Instant>,
    /// Wall-clock instant Fine creep began -- separate from
    /// fine_started_at_secs (history's own elapsed-seconds clock, used
    /// by rolling_average_since) because PA_FAILURE_GRACE_DURATION and
    /// debug_state()'s drop_detector_active need a clock they can read
    /// without needing `history` in scope.
    fine_started_instant: Option<Instant>,
}

pub(crate) enum ScanDetectorPhase {
    Backoff,
    Bracket,
}

pub(crate) struct ScanDetectorState {
    phase: ScanDetectorPhase,
    vbias_mv: i32, // DAC/calibration mV currently being tried -- NOT the detector reading itself
    wait: Option<SampleWait>,
    below: Option<(f32, u16)>, // (power_mw, detector_mv)
    above: Option<(f32, u16)>,
    pinned_count: u32, // consecutive steps clamped at a bound without progress -- see PINNED_LIMIT
    /// Most recent COMPLETE VTX status reply received SINCE the current
    /// vbias_mv was commanded -- reset to None every time vbias_mv
    /// changes (see every site that does `st.wait = None` below, now
    /// paired with `st.last_reading = None`), so a decision can never
    /// pair this step's freshly-averaged meter reading with a VTX status
    /// reply that actually describes an earlier, already-superseded
    /// vbias_mv. Holds the whole reading (vref_mv and detector_mv
    /// together, exactly as the VTX reported them in the SAME message)
    /// rather than separate fields updated independently -- that's what
    /// let vbias_mv (self-tracked, commanded) and a detector_mv field
    /// (separately updated, reported) drift out of sync by a tick,
    /// which is exactly what produced a real detector=0 bug: the
    /// bracket/backoff decision below firing before this step's first
    /// real reading had ever arrived, reading a leftover sentinel
    /// instead. None until a fresh reading arrives -- there is no
    /// synthetic/sentinel/zero fallback for this; every place that
    /// needs it either has a real, current reading or doesn't make the
    /// decision yet (see the atomic-data gate in poll_automatic()'s
    /// ScanDetector arm).
    last_reading: Option<msp::PaCalibrationReading>,
}

/// The Automatic state's own sub-state -- see the module doc comment.
/// EnteringPoint is an explicit state (not, as before, the absence of
/// one) for exactly the (level, freq) currently named by
/// self.level_idx/self.freq_idx: it decides whether ScanPa needs to run
/// its Settle phase first, then hands off to ScanPa.
pub(crate) enum AutomaticStep {
    EnteringPoint,
    ScanPa(ScanPaState),
    ScanDetector(ScanDetectorState),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionLossReason {
    /// The VTX itself went quiet -- most likely explanation is a
    /// current-limited supply tripping and power-cycling it. This is
    /// the only case where resuming sets a hard mV limit (see
    /// try_auto_resume): vbias_mv_at_loss is only meaningful evidence of a
    /// dangerous DAC value when the VTX itself is what stopped
    /// responding, not when it's the meter that dropped out.
    Vtx,
    /// The power meter's connection was lost or disconnected -- the VTX
    /// itself may be fine.
    Meter,
    Both,
}

pub enum EngineState {
    Idle,
    /// Waiting for the operator to confirm they've retuned a manual-
    /// frequency power meter (or, for a band-only meter, retuned to the
    /// nearest band) -- see begin_frequency(). `resume` says which mode
    /// to land in once confirm_frequency() is called.
    AwaitingFreqConfirm { freq_mhz: u16, resume: ResumeMode },
    /// Automatic sweep running -- see AutomaticStep for its own
    /// sub-states (EnteringPoint, ScanPa, ScanDetector).
    Automatic(AutomaticStep),
    /// Manual mode is active: the person is directly driving the DAC via
    /// a slider rather than an automatic search running. poll() takes a
    /// completely different path in this state (see poll_manual()) --
    /// no ScanPa/ScanDetector stepping, just sending whatever
    /// manual_dac_mv currently holds when it changes.
    Manual,
    /// No traffic from the VTX, or the power meter's connection lost/
    /// disconnected, for longer than a sweep in progress should ever go
    /// quiet. Paused here (not aborted) -- auto-resumes on its own once
    /// both are responsive again (see try_auto_resume in poll()), no
    /// user action needed beyond an optional Abort. Captures where
    /// things were when it happened so a genuine VTX trip (reason
    /// includes Vtx) can back off to a safe point and set a hard
    /// ceiling on `level` for the rest of the sweep. `resume` says which
    /// mode auto_resume() should return to.
    ConnectionLost {
        level: u8,
        freq_mhz: u16,
        vbias_mv_at_loss: i32,
        reason: ConnectionLossReason,
        resume: ResumeMode,
    },
}

/// One (level, frequency) result as it's produced -- applied to the
/// working PA table immediately so "Send to VTX" always reflects
/// whatever's completed so far, even mid-sweep.
pub struct SweepResult {
    pub level: u8,
    pub freq_idx: usize,
    pub vbias_mv: Option<u16>,
    pub detector_mv: Option<u16>,
    /// Whether this particular step (ScanPa for vbias_mv, ScanDetector
    /// for detector_mv) actually converged, as opposed to bailing out
    /// (hit a bound, or got pinned without ever reaching the tolerance
    /// band -- see CellStatus::Uncalibrated). worker.rs only commits the
    /// value into the working PA table when this is true -- a failed
    /// attempt's value is discarded, leaving the table's existing
    /// (possibly still-uncalibrated) entry alone.
    pub success: bool,
    /// True only for a ScanPa result that bailed specifically because
    /// Fine creep's thermal-rolloff check tripped (see
    /// CellStatus::PaFailure) -- always false when success is true.
    /// worker.rs doesn't need to treat this specially for the pa_table
    /// write (success already gates that correctly); it exists for
    /// logging/diagnostics.
    pub pa_failure: bool,
    /// True only for a ScanPa result that bailed specifically because
    /// ScanPaPhase::Settle timed out without ever seeing a below-target
    /// reading (see CellStatus::NotSettled) -- always false when success
    /// is true, and mutually exclusive with pa_failure. Same as
    /// pa_failure: exists for logging/diagnostics, not needed for the
    /// pa_table write.
    pub not_settled: bool,
}

/// One (level, frequency) point currently in progress -- what the UI's
/// sweep table needs to highlight/annotate the active cell.
pub struct CurrentStep {
    pub level: u8,
    pub freq_idx: usize,
    pub vbias_mv: Option<i32>,
    pub detector_mv: Option<i32>,
}

/// Snapshot of the engine's internal ScanPa/ScanDetector step state, for
/// the UI's diagnostic indicators displayed under the progress bar --
/// separate from CurrentStep above, which only covers what the sweep
/// table itself needs. See debug_state().
pub struct StepDebugInfo {
    /// ScanPa's own sub-phase: "Coarse" or "Fine" during a ScanPa step,
    /// "Inactive" during a ScanDetector step or when no step is running.
    pub scan_phase: &'static str,
    /// True once Fine creep's PA-Failure (thermal rolloff) check is
    /// actually armed -- Fine phase AND past PA_FAILURE_GRACE_DURATION.
    /// False the rest of the time, including during the grace period
    /// itself, even though fine_highest_avg_mw is still being tracked
    /// then (see the Fine PA-Failure check's own doc comment).
    pub drop_detector_active: bool,
    pub fine_bound_mv: Option<i32>,
    pub fine_highest_avg_mw: Option<f32>,
    /// Some() only during a ScanDetector step.
    pub detector: Option<DetectorDebugInfo>,
}

pub struct DetectorDebugInfo {
    pub phase: &'static str, // "Backoff" or "Bracket"
    pub below: Option<(f32, u16)>,
    pub above: Option<(f32, u16)>,
    pub pinned_count: u32,
}

pub struct SweepEngine {
    pub levels: Vec<u8>,       // selected levels, ascending
    pub frequencies: Vec<u16>, // from PA table idx=0's value[] (frequency breakpoints)
    pub tolerance_pct: f32,    // scanDetector's bracketing tolerance only (per explicit scope)
    pub sign_inverted: bool,
    pub target_mw_by_level: HashMap<u8, u16>,
    pub sweep_hz: f64, // what update_hz was set to for the sweep -- worker.rs restores the previous value on finish/abort

    /// Which mode is active (Automatic vs Manual), and each mode's own
    /// sub-state, all expressed by this one field -- see the module doc
    /// comment. There is no separate flag anywhere in this struct for
    /// "which mode is this".
    pub state: EngineState,
    /// The ONE position both modes read and advance -- see the module
    /// doc comment. Never duplicated per-mode.
    freq_idx: usize,
    level_idx: usize, // index into `levels`
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

    meter_capability: FrequencyCapability, // cached from start()'s argument, so advance_position()'s own frequency-advance logic can honor it too, not just the first frequency
    /// For ManualBand: the last band value actually prompted for --
    /// consecutive VTX frequencies mapping to the same nearest band
    /// don't get a repeat prompt. Reset to None in start().
    last_prompted_band: Option<u32>,
    /// Set whenever the sweep needs to push a new frequency to the VTX
    /// (entering the first frequency in start(), or after confirm_frequency()).
    /// poll() sends this via MSP_VTX_CONFIG at the top of its next call,
    /// then clears it. This was missing entirely in the first version of
    /// this file -- the sweep only ever sent SET_PACALIBRATION (selecting
    /// a power level + DAC vbias_mv), never actually retuned the VTX, so it
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
    /// mid-scan (see EngineState::ConnectionLost / auto_resume).
    /// pub so the UI can show it in a table column; cleared by
    /// clear_hard_limits() when the PA table is Refreshed.
    pub hard_limits: HashMap<u8, i32>,
    /// Sends the engine owes but hasn't issued yet -- safe-state pushes
    /// (from skip_multiple()/abort()) and calibration session begin/end,
    /// each processed one per eligible tick (see poll()'s own top-
    /// priority handling), respecting link.can_send_now() the exact same
    /// way every other send in this file does. This is what "only the
    /// engine sends commands" actually means in practice: a caller like
    /// worker.rs computes a payload (it owns vtx_table, this engine
    /// doesn't) and hands it to skip_multiple()/abort() to queue, rather
    /// than sending it directly itself -- which is exactly how a real
    /// bug happened (an external, ungated send landing while the link
    /// was still supposed to be settling from a previous command).
    pending_sends: VecDeque<PendingSend>,
    /// First time this poll() saw the VTX and/or meter go quiet while
    /// active, or None if both are currently responsive. A SUSTAINED
    /// silence (not a single missed tick) is what actually triggers
    /// ConnectionLost -- see HEARTBEAT_TIMEOUT.
    unresponsive_since: Option<Instant>,

    /// Current DAC value Manual mode is tracking -- set by
    /// set_manual_dac() (the UI slider), read by poll_manual() to know
    /// what to send. Not itself gated by anything; only the actual send
    /// is (see manual_send_pending).
    pub manual_dac_mv: i32,
    /// True when manual_dac_mv has changed since it was last actually
    /// sent -- poll_manual() sends and clears this, throttled by
    /// last_send/SEND_INTERVAL the same way automatic mode's own steps
    /// are, so a fast slider drag doesn't flood the link (only ever
    /// matters as a rate limit here, since MspLink::send() would refuse
    /// outright if the retune settle window from the initial per-point
    /// retune were still open).
    manual_send_pending: bool,
    /// Current desired calibration-session state, included on every
    /// SET_PACALIBRATION send (see vtx_msp_set_calibration()'s doc
    /// comment in vtx_msp.c) -- true for the whole time start()/
    /// start_manual() has been active and hasn't ended.
    session_active: bool,
    /// Current desired PA boost mode, included on every SET_PACALIBRATION
    /// send the same way session_active is. Auto outside Manual mode;
    /// Off immediately after start_manual() (PA must not be enabled when
    /// Manual mode starts); On/Off after set_pa_boost().
    boost_mode: BoostMode,
    /// Most recently observed boost_on value from live VTX telemetry --
    /// compared against each new reading to detect an off-to-on
    /// transition, which starts boost_settle_until below. None until the
    /// first status reply arrives.
    last_boost_on: Option<bool>,
    /// Wall-clock deadline: no sample is trusted for ANY convergence
    /// check (ScanPa coarse or fine, ScanDetector backoff or bracket)
    /// while Instant::now() is before this. Set on every observed
    /// boost_on false/none -> true transition, engine-wide rather than
    /// per-step since boost state itself is global -- see
    /// BOOST_ENABLE_SETTLE_DELAY's doc comment for why enabling the PA
    /// needs this regardless of which phase happens to be running at the
    /// time.
    boost_settle_until: Option<Instant>,
    /// Set on the same boost off->on transition as boost_settle_until
    /// above, but consumed differently: the NEXT fresh (level, freq)
    /// entry that sees this true starts in ScanPaPhase::Settle instead
    /// of jumping straight to CoarseRamp, then clears this once Settle
    /// resolves (either satisfied or timed out). Every subsequent entry
    /// after that skips Settle entirely, until boost toggles off and
    /// back on again.
    ///
    /// Deliberately engine-level and consumed-once rather than
    /// re-armed on every fresh entry: a real run showed the PA's
    /// boost-enable transient is a single, continuous physical event
    /// (self-heating that took 30+ seconds to decay, still trending at
    /// the end) that happens once when boost turns on and stays on for
    /// the rest of the sweep -- re-running Settle's full wait at every
    /// (level, freq) was re-discovering the same already-known decay
    /// and paying its timeout repeatedly instead of once.
    pa_enable_settle_pending: bool,
}

/// A send the engine owes but hasn't issued yet -- see pending_sends'
/// doc comment on SweepEngine. Each variant carries everything poll()
/// needs to actually issue it once link.can_send_now() allows.
enum PendingSend {
    /// A pitmode-safe VTX_CONFIG push -- from abort()/skip_multiple()/
    /// exit_manual()/advance_position()'s own completion branch, all via
    /// safe_state_payload_at_current_point(), which computes this
    /// engine-internally now (see that function's own doc comment).
    SafeState(Vec<u8>),
    /// A calibration-state-only push -- establishes/updates
    /// session_active and/or boost_mode (both fields on SweepEngine
    /// itself now, carried on every SET_PACALIBRATION send -- see
    /// vtx_msp_set_calibration()'s doc comment in vtx_msp.c) without
    /// stepping any real (level, freq) point. Sent with level=0, mv=0
    /// (vtx_msp_set_calibration()'s own "don't touch either" convention)
    /// plus whichever session_active/boost_mode the engine currently
    /// holds at send time -- there's no separate command for this, so a
    /// state-only push still has to go out as a SET_PACALIBRATION.
    CalibrationState,
    /// An empty-payload MSP_VTX_CONFIG query -- queued after every
    /// frequency/pitmode push (see
    /// SweepEngine::build_vtx_config_frequency_payload()). The firmware
    /// already replies to an empty-payload MSP_VTX_CONFIG with its own
    /// actual, current state (vtx_msp_handle_msp()'s data_size==0
    /// branch, via vtx_msp_push_vtx_config() -- see vtx_msp.c), the
    /// exact same mechanism worker.rs's own read_vtx_config() already
    /// uses at startup, just non-blocking here since it goes through
    /// this queue instead. worker.rs's own frame-read loop picks up the
    /// reply (a non-empty VTX_CONFIG frame) and updates the Frequency
    /// panel's UI from it -- deliberately NOT from what we assumed we
    /// pushed, since the VTX may reject, ignore, or otherwise not apply
    /// it; this only reflects what the VTX itself actually reports back.
    RequestVtxConfig,
    /// Explicitly parks the DAC at `vbias_mv` for `level`, with boost
    /// forced off -- queued by skip_multiple() alongside its own
    /// pitmode-forced SafeState push, so a multi-skip batch fully parks
    /// the PA (not just suppresses RF output via pitmode) while
    /// transitioning between points, instead of leaving the DAC sitting
    /// at whatever value it was at when the batch began. Unlike
    /// CalibrationState (which always uses level=0/mv=None, "don't touch
    /// either"), this commands a specific level/mv/boost=Off outright.
    DacLow { level: u8, vbias_mv: u16 },
    /// Explicitly commands boost=On for `level` (mv untouched) -- the
    /// one-time "undo" for DacLow's boost=Off, queued right after it as
    /// part of skip_multiple()'s resume sequence. Necessary because of a
    /// real firmware behavior (see vtx_msp_set_calibration() in
    /// vtx_msp.c): boost_mode=2 (Auto) only clears the manual-override
    /// flag (rf_pa_manual_boost_clear()) -- it does NOT call
    /// rf_pa_boost_on() or otherwise re-apply the level's own
    /// ext_pa_enable state. Since DacLow's boost=Off physically called
    /// rf_pa_boost_off(), simply letting the next regular ScanPa send
    /// (which always uses boost_mode=Auto) run its course left the PA
    /// stuck off for the rest of the sweep -- confirmed from a live log
    /// where boost_on stayed false through every subsequent point after
    /// the first Skip, even though every one of those sends correctly
    /// carried boost_mode=Auto. This restores the physical state once;
    /// the very next Auto send then correctly clears the override flag
    /// without disturbing it.
    RestoreBoost { level: u8 },
}


/// The wire-level meaning of SET_PACALIBRATION's trailing boost byte --
/// see vtx_msp_set_calibration()'s doc comment in vtx_msp.c.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BoostMode {
    Off,
    On,
    /// Automatic mode's own ext_pa_enable-driven behavior -- the
    /// firmware default, and where this always sits for automatic
    /// sends.
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

    /// Call once to begin. See begin_frequency() for how `capability`
    /// determines whether this starts Automatic immediately or pauses on
    /// AwaitingFreqConfirm.
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

    /// Starts Manual mode -- the "Manual" button's counterpart to
    /// start(). Same setup (first selected level/frequency, session
    /// begin, initial retune), but lands in Manual once any
    /// AwaitingFreqConfirm prompt clears, rather than Automatic.
    ///
    /// This does NOT set manual_dac_mv -- the caller (worker.rs, which
    /// owns the PA table this engine doesn't) is expected to follow this
    /// with set_manual_dac(), seeded from whatever's currently stored
    /// for (levels[0], frequencies[0]) in the working table, so the
    /// slider starts from the existing calibration rather than an
    /// arbitrary default.
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
        // PA must not be enabled when manual mode starts -- see
        // set_pa_boost()'s doc comment and rf_pa_manual_boost_set() in
        // the firmware.
        self.boost_mode = BoostMode::Off;
        debug!(target: "vtx", "[sweep] starting manual mode: {} levels {:?}, {} frequencies {:?}, meter_capability={:?}",
            self.levels.len(), self.levels, self.frequencies.len(), self.frequencies, self.meter_capability);
        self.pending_sends.push_back(PendingSend::CalibrationState);
        let first_freq = self.frequencies[0];
        self.begin_frequency(first_freq, ResumeMode::Manual);
    }

    /// Manual mode's explicit PA-enable checkbox -- see
    /// rf_pa_manual_boost_set()'s doc comment in the firmware's rf_pa.h.
    /// Doesn't queue a separate send -- boost_mode rides on the next
    /// regular per-step send poll_manual() makes (same as manual_dac_mv
    /// does), so this just updates the state and makes sure that send
    /// actually happens soon, even if the DAC value itself hasn't
    /// changed.
    pub fn set_pa_boost(&mut self, on: bool) {
        self.boost_mode = if on { BoostMode::On } else { BoostMode::Off };
        self.manual_send_pending = true;
    }

    /// Sets the DAC value Manual mode should be driving -- called by the
    /// UI's slider on every change (including the initial seed after
    /// start_manual()/each manual_next() advance). Doesn't send anything
    /// itself; poll_manual() picks this up on its own next eligible
    /// tick, throttled the same way every other send in this file is.
    /// Clamped to the DAC's own raw range (0-3300mV) as basic sanity --
    /// deliberately NOT clamped to effective_bounds(), since Manual mode
    /// is meant to let the person reach the full range directly.
    pub fn set_manual_dac(&mut self, mv: i32) {
        self.manual_dac_mv = mv.clamp(0, 3300);
        self.manual_send_pending = true;
    }

    /// "Next" in Manual mode: commits manual_dac_mv as this cell's
    /// calibration value and `detector_mv` (read by the caller from the
    /// live VTX status, NOT the external power meter) as its detector
    /// value, in one step -- unlike automatic mode's two-phase ScanPa/
    /// ScanDetector, since the person already knows both simultaneously.
    /// Marks both cells CellStatus::Manual, then advances exactly like
    /// automatic mode's own level/frequency rollover (same
    /// advance_position(), see its own doc comment). Returns the new
    /// (level, freq_idx) position for the caller to reseed
    /// set_manual_dac() from the working table, or None if that was the
    /// last point (manual mode is now finished, session closed).
    pub fn manual_next(&mut self, detector_mv: u16) -> Option<(u8, usize)> {
        if !matches!(&self.state, EngineState::Manual) {
            return None;
        }
        let level = self.levels[self.level_idx];
        debug!(target: "vtx", "[sweep] manual: level={level} freq={}MHz vbias_mv={} detector_mv={detector_mv}",
            self.frequencies[self.freq_idx], self.manual_dac_mv);
        self.completed_steps += 2; // counts as both ops, same as skip_multiple()/PA Failure
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

    /// Pure position arithmetic, shared by advance_position() and
    /// skip_multiple(): advances (level_idx, freq_idx) by exactly one
    /// step, wrapping level_idx into freq_idx exactly the way the whole
    /// sweep enumerates points. No side effects -- doesn't touch
    /// self.state, doesn't push anything, doesn't call
    /// begin_frequency(). Returns Some(true) if this step crossed into a
    /// new frequency, Some(false) if it just moved to the next level
    /// within the same frequency, or None if this was the last point in
    /// the whole sweep -- in that case level_idx/freq_idx are left
    /// UNCHANGED (still pointing at the last valid point, not advanced
    /// past the end), so a caller can still build a safe-state payload
    /// for wherever it actually stopped.
    fn advance_indices(&mut self) -> Option<bool> {
        if self.level_idx + 1 < self.levels.len() {
            self.level_idx += 1;
            return Some(false);
        }
        if self.freq_idx + 1 >= self.frequencies.len() {
            return None; // whole sweep complete -- indices left at the last valid point
        }
        self.level_idx = 0;
        self.freq_idx += 1;
        Some(true)
    }

    /// The ONE place level_idx/freq_idx ever change, for EITHER mode --
    /// see the module doc comment. Called by every "this point is done,
    /// move on" site in this file: finish_scan_pa()'s failure paths,
    /// finish_scan_detector(), manual_next(), and skip_multiple() (both
    /// modes). Reads which mode is currently active from self.state
    /// itself (no separate flag to pass in or keep in sync), and always
    /// leaves self.state set to wherever that mode should be once this
    /// returns -- the caller never needs its own follow-up state
    /// assignment.
    ///
    /// Returns the new (level, freq_idx) if there's a next point, or
    /// None if the sweep/manual session is now complete (session closed,
    /// state set to Idle, right here -- not left for the caller to
    /// notice separately, which is exactly the duplication this
    /// replaces: before this, automatic mode's own "all frequencies
    /// complete" handling and Manual mode's were two separately
    /// hand-maintained copies of the same rollover logic).
    fn advance_position(&mut self) -> Option<(u8, usize)> {
        let resume = match &self.state {
            EngineState::Manual => ResumeMode::Manual,
            _ => ResumeMode::Automatic,
        };
        // Captured before either index moves -- this is the point that
        // just finished, which is what a safe-state push (if this turns
        // out to be the last point) needs to preserve. See
        // safe_state_payload_at_current_point()'s own doc comment.
        let safe_state_payload = self.safe_state_payload_at_current_point();
        match self.advance_indices() {
            None => {
                debug!(target: "vtx", "[sweep] all frequencies complete");
                // Force pitmode on at the last-used frequency -- without
                // this, a sweep/manual session that runs all the way to
                // completion (as opposed to being Abort-ed) left the VTX
                // live, PA on, at whatever the last point happened to
                // be. session_active/boost_mode below only affect the
                // firmware's OWN closed-loop behavior; they don't touch
                // pitmode at all, which lives on a completely separate
                // VTX_CONFIG field.
                self.pending_sends.push_back(PendingSend::SafeState(safe_state_payload));
                self.state = EngineState::Idle;
                self.session_active = false;
                self.boost_mode = BoostMode::Auto;
                self.pending_sends.push_back(PendingSend::CalibrationState);
                None
            }
            Some(true) => {
                let next_freq = self.frequencies[self.freq_idx];
                self.begin_frequency(next_freq, resume); // may land on AwaitingFreqConfirm
                Some((self.levels[self.level_idx], self.freq_idx))
            }
            Some(false) => {
                // Same frequency, next level -- no retune needed, just
                // return directly to whichever mode is active, ready for
                // its next point.
                self.state = match resume {
                    ResumeMode::Automatic => EngineState::Automatic(AutomaticStep::EnteringPoint),
                    ResumeMode::Manual => EngineState::Manual,
                };
                Some((self.levels[self.level_idx], self.freq_idx))
            }
        }
    }

    /// Exits Manual mode via the "Manual" button pressed a second time.
    /// The current, not-yet-confirmed point is left exactly as it was --
    /// no result written, no cell status touched -- matching "Next"
    /// being the only thing that commits a point. Forces pitmode on at
    /// the last-used frequency (see safe_state_payload_at_current_point()'s
    /// own doc comment) -- this is also a way calibration ends, same as
    /// abort() and advance_position()'s own completion branch.
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

    /// "Re-calibrate" pressed while Manual mode is active: resumes
    /// automatic scanning from wherever Manual mode was currently
    /// sitting (NOT from the beginning) -- the current point's manual
    /// slider value is discarded (automatic's own ScanPa starts fresh
    /// from coarse_ramp_start_vbias_mv(), same as any other point), but
    /// every earlier point Manual mode already committed via "Next" stays
    /// as its Manual result. The calibration session stays open throughout
    /// (it doesn't care which sub-mode is driving it), so no new
    /// SessionBegin is queued here -- session_active just stays true. No
    /// retune here either -- the VTX is already on the right frequency,
    /// this only changes which mode is driving it.
    pub fn resume_automatic_from_current(&mut self) {
        debug!(target: "vtx", "[sweep] resuming automatic mode from level={:?} freq_idx={}",
            self.levels.get(self.level_idx), self.freq_idx);
        self.manual_send_pending = false;
        self.state = EngineState::Automatic(AutomaticStep::EnteringPoint);
        // Clears Manual mode's PA boost override, if one was set, so
        // automatic mode's own ext_pa_enable-driven behavior (which the
        // very next retune will apply) takes back over rather than
        // staying pinned at whatever the checkbox last said.
        self.boost_mode = BoostMode::Auto;
        self.pending_sends.push_back(PendingSend::CalibrationState);
    }

    /// Decides what happens when the sweep is about to start working on
    /// `freq_mhz`, based on the meter's capability -- shared by start()/
    /// start_manual() (the first frequency) and advance_position() (every
    /// frequency after that), so all three honor the same rules:
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
    /// `resume` is which mode to land in once any prompt clears --
    /// carried on AwaitingFreqConfirm itself when one is needed.
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

    /// UI calls this after the user confirms they've retuned the meter.
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

    /// Begins a safe-state abort -- forces pitmode on at whatever
    /// (level, freq) this engine was actually working on (see
    /// safe_state_payload_at_current_point()'s own doc comment; this
    /// deliberately does NOT use vtx_table's own cached frequency, which
    /// is the Frequency panel's separate, unrelated setting). Queued
    /// rather than sent directly, so it goes out through the exact same
    /// gated mechanism as every other send in this file (see
    /// pending_sends' doc comment on SweepEngine). Clears anything
    /// already queued or pending (e.g. a skip's own not-yet-sent
    /// safe-state push) and any pending retune, since abort takes
    /// absolute precedence over whatever the sweep was about to do next.
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

    /// Skips `count` (level, freq) points in a row, advancing exactly as
    /// if each had finished normally (marked Skipped instead of
    /// Calibrated/Uncalibrated) -- but without cycling the VTX through
    /// every intermediate point along the way. This is what the UI's
    /// Skip button now sends: it debounces rapid presses into a single
    /// call here (500ms after the last press -- see
    /// pages/calibration.rs), rather than sending `count` separate
    /// single-skip commands. Previously, each skip queued its own
    /// pitmode-safe transition (and, if it crossed a frequency, its own
    /// retune) -- pressing Skip 4 times in under a second used to queue
    /// 4 rounds of that traffic, each gated by its own settle window,
    /// which then took several visible seconds to drain, with the VTX
    /// visibly/audibly cycling through every intermediate frequency
    /// along the way. Batching means exactly ONE combined safe
    /// transition covers the whole batch, and exactly one retune (if
    /// any) lands the VTX directly on wherever the batch actually ends.
    ///
    /// The first point skipped is whatever was ACTUALLY in progress when
    /// the first press arrived -- same cell-accounting distinction the
    /// old single-skip path made (mid-ScanPa: both cells outstanding;
    /// mid-ScanDetector: just the detector cell; Manual/EnteringPoint:
    /// nothing committed yet). Every point after the first was never
    /// visited at all (fast-forwarded past, not stepped into), so those
    /// always mark both cells.
    ///
    /// The combined safe transition is two queued sends, not one: a
    /// pitmode-forced VTX_CONFIG (via safe_state_payload_at_current_point(),
    /// same as abort()/exit_manual()) AND an explicit SET_PACALIBRATION
    /// parking the DAC at the safe/low-power end of the STARTING level's
    /// range (see coarse_ramp_start_vbias_mv()) with boost forced off --
    /// pitmode alone suppresses RF output but doesn't itself move the
    /// DAC or touch the boost stage, and both together park the PA more
    /// thoroughly than either alone during a transition that might
    /// otherwise take a visible moment to land.
    ///
    /// If the batch runs past the last remaining point, ends the session
    /// exactly like advance_position()'s own completion branch (one
    /// safe-state push at wherever the batch actually stopped, then
    /// close out) -- the DAC-low push is skipped in that case, since
    /// pitmode already covers "session's over, VTX should go quiet" and
    /// there's no "resume" to protect against.
    ///
    /// Returns the new (level, freq_idx) only for Manual mode (the UI
    /// reseeds the DAC slider from it) -- automatic mode's own table
    /// redraws from cell status instead, so it always returns None,
    /// matching skip_current()'s old behavior. count=0 is a no-op.
    pub fn skip_multiple(&mut self, count: u32) -> Option<(u8, usize)> {
        if count == 0 {
            return None;
        }
        let resume = match &self.state {
            EngineState::Manual => ResumeMode::Manual,
            EngineState::Automatic(_) => ResumeMode::Automatic,
            _ => return None, // not in a skippable state (Idle, AwaitingFreqConfirm, ConnectionLost)
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
                // Fast-forwarded past entirely -- never visited, so both
                // cells are still outstanding.
                Self::set_cell_status(&mut self.cal_cell_status, (level, self.freq_idx), CellStatus::Skipped);
                Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), CellStatus::Skipped);
                self.completed_steps += 2;
            }
            self.per_level_status.insert(level, LevelStatus::Skipped);

            match self.advance_indices() {
                None => {
                    // Whole sweep completed partway through this batch --
                    // same handling as advance_position()'s own completion
                    // branch, at wherever we actually stopped
                    // (advance_indices() leaves level_idx/freq_idx at the
                    // last valid point on this outcome).
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

        // ONE combined safe transition for the whole batch -- not one
        // per skipped point. DAC-low first (park the PA before any
        // retune that follows), then pitmode, then an explicit boost
        // restore for wherever the batch is resuming -- see
        // RestoreBoost's own doc comment for why this last step is
        // necessary (Auto alone does not undo DacLow's boost=Off).
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
            self.begin_frequency(freq_mhz, resume); // may land on AwaitingFreqConfirm
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

    /// True whenever Manual mode is the active sub-mode -- covers
    /// Manual itself plus its own AwaitingFreqConfirm/ConnectionLost
    /// pauses (both carry `resume`, checked here rather than a separate
    /// flag), false once Idle or switched to automatic. Used by the UI
    /// to decide which of the Automatic/Manual buttons to disable, and
    /// whether the DAC slider/fine checkbox/PA checkbox are usable.
    pub fn is_manual_mode(&self) -> bool {
        match &self.state {
            EngineState::Manual => true,
            EngineState::AwaitingFreqConfirm { resume, .. } => *resume == ResumeMode::Manual,
            EngineState::ConnectionLost { resume, .. } => *resume == ResumeMode::Manual,
            _ => false,
        }
    }

    /// True whenever automatic scanning is the active sub-mode --
    /// active but NOT manual. See is_manual_mode()'s own doc comment.
    pub fn is_automatic_mode(&self) -> bool {
        match &self.state {
            EngineState::Automatic(_) => true,
            EngineState::AwaitingFreqConfirm { resume, .. } => *resume == ResumeMode::Automatic,
            EngineState::ConnectionLost { resume, .. } => *resume == ResumeMode::Automatic,
            _ => false,
        }
    }

    /// Advances the sweep by whatever's possible this tick. `link` is
    /// used to send SET_PACALIBRATION; `history` is the live power
    /// reading buffer (for averaging); `reading_seq` is
    /// SharedState::reading_seq, a monotonic counter used for "has a new
    /// reading arrived" instead of history.len() (see SampleWait's doc
    /// comment for why that distinction matters); `latest_reading` is
    /// the most recent decoded MSP_PACALIBRATION response, if one
    /// arrived this tick; `vtx_ready`/`meter_ready` are
    /// SharedState::vtx_ready and the equivalent fast heartbeat for the
    /// meter -- true if either has been heard from recently. Losing
    /// EITHER pauses the sweep in ConnectionLost, which this function
    /// also auto-resumes from once both recover, without any external
    /// "Continue" command.
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

    /// Top-of-tick handling that applies regardless of which state is
    /// active, then dispatches to whichever mode-specific poll function
    /// self.state currently calls for. This is the ONE place that reads
    /// self.state to decide "which mode's own poll function runs this
    /// tick" -- poll_automatic()/poll_manual() themselves never need to
    /// re-check which mode they're in, since the dispatcher only ever
    /// calls the one that matches.
    fn poll_dispatch(
        &mut self,
        link: &mut MspLink,
        history: &VecDeque<(f64, f32)>,
        reading_seq: u64,
        latest_reading: Option<msp::PaCalibrationReading>,
        vtx_ready: bool,
        meter_ready: bool,
    ) -> anyhow::Result<bool> {
        // Track boost_on transitions from live telemetry before anything
        // else this tick -- the PA can power up at the start of ANY
        // phase (ScanPa coarse, Fine, ScanDetector, or Manual mode's own
        // PA checkbox), so this has to run unconditionally rather than
        // being embedded in one specific step's own handling. See
        // BOOST_ENABLE_SETTLE_DELAY's doc comment.
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

        // Queued sends (safe-state pushes from skip_multiple()/abort(),
        // calibration session begin/end) take absolute priority over
        // everything else, and are processed regardless of engine state
        // -- abort() already transitions to Idle synchronously, but its
        // queued send still needs to go out. One per tick, same
        // link.can_send_now() gate as every other send in this file, so
        // there's no separate bookkeeping to keep in sync -- the link
        // itself is the single source of truth for whether a send is
        // allowed right now.
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

        // Each check below borrows self.state only long enough to copy
        // out what it needs (every field involved is Copy), so the
        // borrow is fully released before any &mut self call that
        // follows -- deliberately not one single match on self.state,
        // since EngineState can no longer derive Copy (AutomaticStep
        // holds ScanPa/ScanDetector search state that isn't) and a few
        // of these arms need &mut self methods.
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
        Ok(false) // Idle, AwaitingFreqConfirm -- nothing to do
    }

    /// poll_dispatch()'s target while EngineState::Automatic(_). Takes
    /// the current AutomaticStep out of self.state (mirroring the old
    /// self.step.take() pattern), handles EnteringPoint by constructing
    /// a fresh ScanPaState and falling straight into the exact same
    /// first tick ScanPa itself would get (see tick_scan_pa() -- no
    /// wasted tick just for entering a point, matching this file's
    /// original behavior), then dispatches ScanPa/ScanDetector to their
    /// own tick functions.
    fn poll_automatic(
        &mut self,
        link: &mut MspLink,
        history: &VecDeque<(f64, f32)>,
        reading_seq: u64,
        vtx_ready: bool,
        meter_ready: bool,
    ) -> anyhow::Result<bool> {
        if !link.can_send_now() {
            // Waiting out a settle period from whatever was last sent.
            // Do nothing this tick, including the heartbeat check below
            // (the VTX being quiet during a deliberate settle pause is
            // expected, not a sign of lost communication).
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

        // Frequency changes take priority over the regular per-step send,
        // same as poll_manual()'s own handling -- begin_frequency() (called
        // from start()/advance_position()) sets pending_frequency_push
        // (and, for Manual/ManualBand meters, routes through
        // AwaitingFreqConfirm first); this is what actually issues the
        // VTX_CONFIG retune once that's ready. Without this check here,
        // the engine's own level_idx/freq_idx (and therefore its log
        // messages and the UI's status view) advance to the new
        // frequency immediately, but the VTX itself is never actually
        // retuned -- it silently stays on whatever frequency it was
        // last tuned to.
        if let Some(freq_mhz) = self.pending_frequency_push.take() {
            let level = self.levels.first().copied().unwrap_or(1);
            let payload = self.build_vtx_config_frequency_payload(freq_mhz, level, false); // normal retune -- pitmode off
            link.send_v1(function::VTX_CONFIG as u8, &payload)?;
            link.note_sent(MspCommandKind::Retune);
            debug!(target: "vtx", "[sweep] pushed frequency change to {freq_mhz}MHz (power={level})");
            self.pending_sends.push_back(PendingSend::RequestVtxConfig);
            return Ok(true); // next tick will wait out the settle gate before sending anything else
        }

        let level = self.levels[self.level_idx];
        let freq_mhz = self.frequencies[self.freq_idx];
        let target_mw = *self.target_mw_by_level.get(&level).unwrap_or(&0) as f32;
        let now_seq = reading_seq;
        let throttled = self.last_send.elapsed() < SEND_INTERVAL;

        let step = match std::mem::replace(&mut self.state, EngineState::Idle) {
            EngineState::Automatic(step) => step,
            other => {
                // Shouldn't happen -- poll_dispatch() only calls this
                // function when self.state matched Automatic(_). Put
                // whatever it actually was back rather than losing it.
                self.state = other;
                return Ok(false);
            }
        };

        match step {
            AutomaticStep::EnteringPoint => {
                // Entering a fresh (level, freq): starts in Settle only if
                // boost has turned on since Settle last ran (see
                // pa_enable_settle_pending's own doc comment) -- otherwise
                // goes straight to CoarseRamp, since Settle's transient is a
                // one-time event per boost-enable, not per (level, freq).
                // Settle hands off to CoarseRamp once the reading is
                // genuinely at or below target -- from the safe/low-power
                // end of the range for THIS board's sign, see
                // coarse_ramp_start_vbias_mv()'s own doc comment.
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
                Self::set_cell_status(&mut self.cal_cell_status, (level, self.freq_idx), CellStatus::Current);
                self.tick_scan_pa(link, history, now_seq, throttled, level, freq_mhz, target_mw, st)
            }
            AutomaticStep::ScanPa(st) => self.tick_scan_pa(link, history, now_seq, throttled, level, freq_mhz, target_mw, st),
            AutomaticStep::ScanDetector(st) => self.tick_scan_detector(link, history, now_seq, throttled, level, freq_mhz, target_mw, st),
        }
    }

    /// One tick of ScanPa (whichever of Settle/CoarseRamp/Fine is
    /// active) -- see the module's own diagrams for the full internal
    /// flow of each phase. Sends/resends as needed, evaluates once
    /// enough fresh samples exist, and either steps within the current
    /// phase, hands off to the next phase, or finishes via
    /// finish_scan_pa() (success or failure -- see ScanPaOutcome).
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
        // PA-enable settle gate -- applies to Settle, CoarseRamp,
        // and Fine alike, since the PA can power up at the start
        // of any of them (whichever phase happens to be running
        // when the level/frequency's boost first turns on). See
        // BOOST_ENABLE_SETTLE_DELAY's doc comment. Placed before
        // `wait` is ever borrowed below, same reasoning as
        // fine_settle_until's own placement.
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
            self.boost_settle_until = None; // settled -- proceed normally from here on
        }

        if matches!(st.phase, ScanPaPhase::Fine) {
            if let Some(started_at) = st.fine_started_at_secs {
                if let Some(rolling) = rolling_average_since(history, started_at, PA_FAILURE_WINDOW_SECS) {
                    // Still tracked during the grace period (so once it
                    // ends we have a real peak to compare against, not a
                    // fresh start with no history) -- only the ABORT
                    // itself is suppressed while still within
                    // PA_FAILURE_GRACE_DURATION of Fine creep beginning.
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

            // 1-second DAC-settle delay right after the CoarseRamp->Fine
            // handoff -- see FINE_SETTLE_DELAY's doc comment. Placed
            // before `wait` is ever borrowed below, so resetting
            // st.wait here can't run into any borrow-checker questions
            // about a still-live reference into the same field.
            if let Some(settle_until) = st.fine_settle_until {
                if Instant::now() < settle_until {
                    // Still settling -- discard any samples collected so
                    // far (SampleWait's own gate is purely count-based,
                    // so it can fill well before the VBIAS circuit has
                    // physically caught up) and resend (throttled) so
                    // the DAC value is definitely in flight, but don't
                    // start counting toward the convergence check yet.
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
                st.fine_settle_until = None; // settled -- proceed normally from here on
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
                    st.wait = None; // take another fresh sample window at the same, unchanged vbias_mv
                }
            }
            ScanPaPhase::CoarseRamp => {
                // Always wait for a genuine, MEASURED overshoot
                // (avg_mw >= target) before ever handing off to
                // Fine -- never stop at a percentage-of-target
                // threshold on the assumption it'll overshoot.
                // Once within 10% of target, narrow in by
                // halving the step size (floor 1mV) for each
                // subsequent real coarse step, rather than
                // continuing at the full 25mV and risking a
                // large, unconfirmed overshoot -- but still
                // actually TAKE and MEASURE that narrower step,
                // same as any other coarse step, rather than
                // computing where it would probably land and
                // handing off on that guess.
                if avg_mw >= target_mw {
                    let Some(fine_start) = st.last_below_target_mv else {
                        // The STARTING point itself already overshot --
                        // no coarse step has ever confirmed a genuine
                        // below-target point yet, so there's nothing to
                        // bracket Fine creep against. Handing off anyway
                        // would collapse fine_start and fine_bound_mv to
                        // this same vbias_mv, and Fine would trivially
                        // "converge" on its very first sample at
                        // whatever this overshoot value is -- reporting
                        // success at a value that was never actually
                        // approached from below or searched for at all.
                        // A real run showed exactly this: the coarse
                        // ramp's starting point read 58mW against a
                        // 10mW target, and the point got marked
                        // Calibrated at 53mW. Bail honestly instead --
                        // this is worth surfacing, not working around,
                        // since a starting point already this far above
                        // target usually means something upstream
                        // changed (residual state from a previous point,
                        // the PA behaving differently than expected at
                        // this frequency) that's worth the person
                        // actually looking into.
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
                    // Pad the ceiling 25% beyond the exact overshoot
                    // point, in the direction of more power (sign-aware
                    // via `up`, so this works the same for both normal
                    // and inverted boards) -- without this, Fine creep
                    // is hard-bounded at the EXACT vbias_mv that
                    // overshot during the coarse ramp, and a real run
                    // showed that if the PA's response drifts (e.g.
                    // thermal) between the coarse ramp's reading and
                    // Fine creep's own approach, that same vbias_mv
                    // sometimes no longer reaches target -- Fine creep
                    // then walks all the way to this bound and still
                    // falls short, bailing on a target that IS
                    // reachable, just slightly further past where the
                    // coarse ramp first saw it cross.
                    //
                    // 25% of the overshoot point's own value; falls
                    // back to the current coarse step size if that
                    // would be zero (an overshoot sitting exactly at
                    // vbias_mv=0 would otherwise get no margin at all,
                    // silently defeating this fix for exactly the
                    // starting-point-adjacent case it's most likely to
                    // matter for). Still clamped to (bound_lo,
                    // bound_hi) -- this pads for drift, it doesn't
                    // license exceeding the safe range.
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
                        // Ran off the end of the allowed range (DAC bound, or a hard limit
                        // from a previous VTX power-loss on this level) without ever
                        // overshooting -- bail this (level, freq) as best-effort.
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
                // Fine's own ceiling combines the tight bound handed off
                // from CoarseRamp with the global safety bound, taking
                // whichever is reached first -- fine_bound_mv should
                // always be the tighter of the two in normal operation,
                // but a concurrently-set hard limit is still respected.
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

    /// One tick of ScanDetector (Backoff or Bracket) -- see the
    /// module's own diagrams for the full internal flow. Same
    /// send/wait/evaluate shape as tick_scan_pa(), plus an extra atomic-
    /// data gate: no decision here runs without a real VTX status reply
    /// that postdates the current vbias_mv (see ScanDetectorState's own
    /// last_reading doc comment).
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
        // PA-enable settle gate -- see the same check at the top
        // of tick_scan_pa() for why this needs to cover
        // ScanDetector too, not just ScanPa.
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
            self.boost_settle_until = None; // settled -- proceed normally from here on
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

        // Atomic-data gate: no decision below runs without a real
        // VTX status reply that postdates the current vbias_mv --
        // see last_reading's own doc comment. wait.ready() only
        // means the METER has enough fresh samples; it says
        // nothing about whether the VTX's own reported state has
        // arrived yet, since the meter and the VTX are two
        // independent, asynchronously-polled data sources.
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
                // The critical case this closes: with a hard limit active,
                // the target can be genuinely unreachable within the safe
                // range (confirmed by a real run: this got clamped at the
                // same bound for the whole rest of the scan, since the
                // reading was nowhere near target and never could be).
                // Without counting consecutive clamps, that has no exit
                // condition and spins forever.
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

    /// Progress within the CURRENT (level, freq, op) step only -- coarse,
    /// phase-block-based (not fine-grained, since these are open-ended
    /// searches with no fixed step count to measure against), EXCEPT for
    /// CoarseRamp: since it steps in known, fixed increments toward
    /// a known boundary, the fraction of that distance already covered
    /// gives a real (if approximate -- we don't know in advance how many
    /// steps it'll actually take to reach target) sense of movement,
    /// rather than a static placeholder. Returns (0.0, "") when nothing
    /// is in progress.
    pub fn sub_progress(&self) -> (f32, &'static str) {
        match &self.state {
            EngineState::Automatic(AutomaticStep::ScanPa(st)) => match st.phase {
                ScanPaPhase::Settle => (0.0, "ScanPa: settling"),
                ScanPaPhase::CoarseRamp => {
                    let up = power_up_step(self.sign_inverted);
                    let level = self.levels.get(self.level_idx).copied().unwrap_or(0);
                    let (bound_lo, bound_hi) = self.effective_bounds(level);
                    let boundary = if up > 0 { bound_hi } else { bound_lo };
                    let start = coarse_ramp_start_vbias_mv(self.sign_inverted, bound_lo, bound_hi);
                    let total_possible = ((boundary - start) as f32 / COARSE_RAMP_STEP_MV as f32)
                        .abs()
                        .max(1.0);
                    let frac = (st.coarse_steps_taken as f32 / total_possible).clamp(0.0, 1.0);
                    (0.6 * frac, "ScanPa: coarse ramp")
                }
                ScanPaPhase::Fine => (0.8, "ScanPa: fine creep"),
            },
            EngineState::Automatic(AutomaticStep::ScanDetector(st)) => match st.phase {
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
            _ => (0.0, ""),
        }
    }

    pub fn current_step(&self) -> Option<CurrentStep> {
        let level = self.levels.get(self.level_idx).copied()?;
        if matches!(&self.state, EngineState::Manual) {
            // Manual mode never uses AutomaticStep -- report the slider's
            // live value directly instead. Detector isn't reported here;
            // the VTX Status panel's own live detector_mv already covers
            // that (Manual mode doesn't run its own search against it).
            return Some(CurrentStep { level, freq_idx: self.freq_idx, vbias_mv: Some(self.manual_dac_mv), detector_mv: None });
        }
        match &self.state {
            EngineState::Automatic(AutomaticStep::ScanPa(st)) => Some(CurrentStep {
                level,
                freq_idx: self.freq_idx,
                vbias_mv: Some(st.vbias_mv),
                detector_mv: None,
            }),
            EngineState::Automatic(AutomaticStep::ScanDetector(st)) => Some(CurrentStep {
                level,
                freq_idx: self.freq_idx,
                vbias_mv: None,
                detector_mv: st.last_reading.map(|r| r.detector_mv as i32),
            }),
            _ => None,
        }
    }

    /// Diagnostic snapshot of the engine's internal ScanPa/ScanDetector
    /// step state, for the UI's debug indicators under the progress bar.
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

    /// (min_vbias_mv, max_vbias_mv) for `level` -- the full 0..=3300 DAC range,
    /// narrowed on whichever side is "more power" if a hard limit was
    /// set for this level (see EngineState::ConnectionLost / auto_resume).
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

    /// Builds a pitmode-forced VTX_CONFIG payload at whatever (level,
    /// freq) this engine itself is CURRENTLY (or, if just past the last
    /// point, was MOST RECENTLY) working on -- level_idx/freq_idx are
    /// never reset by abort()/skip_multiple(), and advance_position()'s
    /// own completion branch captures them before either index moves,
    /// so this is always the right point in every case that calls it.
    ///
    /// Deliberately self-contained (no vtx_table involved) -- every
    /// place calibration ends (Abort, Skip/Next past the last point,
    /// exiting Manual mode) needs to force pitmode WITHOUT changing
    /// which frequency the VTX is left on, and vtx_table is the
    /// Frequency panel's own, unrelated setting (see
    /// Command::SkipMultiple's own comment in worker.rs for the exact
    /// failure that caused when this used vtx_table's cached frequency
    /// instead of the sweep's own).
    fn safe_state_payload_at_current_point(&self) -> Vec<u8> {
        let level = self.levels.get(self.level_idx).copied().unwrap_or(1);
        let freq_mhz = self.frequencies.get(self.freq_idx).copied().unwrap_or(5800);
        self.build_vtx_config_frequency_payload(freq_mhz, level, true)
    }

    /// Called by poll() once both vtx_ready and meter_ready are true
    /// again while in ConnectionLost -- no external "Continue" command
    /// needed. Only when the VTX itself was actually involved (reason is
    /// Vtx or Both) does this set a hard limit and back the in-progress
    /// step's mV off to a safe point (backed off HEARTBEAT_BACKOFF_MV
    /// from the trip point, in the safe/less-power direction) -- a pure
    /// meter dropout says nothing about whether the mV value itself was
    /// dangerous, so that case just resumes as-is. Either way, re-pushes
    /// the frequency: harmless if the VTX never actually lost power, and
    /// necessary if it did (a reboot needs retuning and level reselection
    /// from scratch).
    fn auto_resume(&mut self, level: u8, freq_mhz: u16, vbias_mv_at_loss: i32, reason: ConnectionLossReason) {
        let resume = match &self.state {
            EngineState::ConnectionLost { resume, .. } => *resume,
            _ => ResumeMode::Automatic, // shouldn't happen -- called only from the ConnectionLost arm
        };
        debug!(target: "vtx", "[sweep] connection restored ({reason:?}), resuming: level={level} freq={freq_mhz}MHz resume={resume:?}");
        if matches!(reason, ConnectionLossReason::Vtx | ConnectionLossReason::Both) {
            let up = power_up_step(self.sign_inverted);
            let safe_vbias_mv = (vbias_mv_at_loss - up * HEARTBEAT_BACKOFF_MV).clamp(0, 3300);
            self.hard_limits.insert(level, safe_vbias_mv);
            debug!(target: "vtx", "[sweep] hard limit set to {safe_vbias_mv}mV (backed off {HEARTBEAT_BACKOFF_MV}mV from trip point {vbias_mv_at_loss}mV)");
            match resume {
                ResumeMode::Manual => {
                    // Same backed-off value the slider will show on resume --
                    // the person can always push it back up manually, but
                    // resuming AT the value that just caused a trip would be
                    // exactly the wrong default.
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
                // Whatever ScanPa/ScanDetector state was active when the
                // connection dropped is gone (ConnectionLost carried none
                // of it) -- resuming re-enters this (level, freq) fresh,
                // same as any other point, rather than trying to
                // reconstruct exactly where the search was.
                EngineState::Automatic(AutomaticStep::EnteringPoint)
            }
            ResumeMode::Manual => EngineState::Manual,
        };
        self.pending_frequency_push = Some(freq_mhz);
    }

    /// For the one case poll()'s own heartbeat-based detection can't
    /// reach: a hard I/O error on the VTX link drops it to None in
    /// worker.rs, and poll() is only ever called with a live
    /// `&mut MspLink` -- so without this, the engine's state would just
    /// freeze at Automatic forever (poll() never called again to notice
    /// anything), never surfacing the "Connection error" dialog for a
    /// scenario that's just as real as the heartbeat-timeout one poll()
    /// does catch. Only meaningful while actively Automatic (silently a
    /// no-op otherwise -- e.g. already paused, Manual mode, or not
    /// sweeping at all) -- preserved exactly as before this refactor;
    /// Manual mode never called this, which may be worth a separate look
    /// but isn't something this restructuring changes.
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

    /// Shared heartbeat tracking for both automatic (poll_automatic())
    /// and Manual (poll_manual()) modes -- if the VTX and/or meter have
    /// been silent for longer than HEARTBEAT_TIMEOUT, transitions to
    /// ConnectionLost. Manual mode gets the exact same "Connection
    /// error" dialog and auto-resume behavior automatic mode already has
    /// (see auto_resume(), which resumes back into whichever mode was
    /// active when the trip happened, via the `resume` this stores) --
    /// the person needs to know their PA just went quiet regardless of
    /// which mode was driving it. `vbias_mv_now` is whatever DAC value
    /// was in play when this is called (automatic mode reads it from
    /// the active step, Manual mode from manual_dac_mv) -- captured as
    /// evidence a hard limit may sit near this value if the VTX itself
    /// is what went quiet. Returns true if it just transitioned to
    /// ConnectionLost this call (the caller should do nothing else this
    /// tick).
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
        // Only mark the cell as the trip point (red, sticky) when the VTX
        // itself was actually involved -- a pure meter dropout says
        // nothing about whether this mV value was dangerous, so marking
        // it here would be misleading.
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

    /// poll_dispatch()'s target while EngineState::Manual. Sends
    /// manual_dac_mv if it's changed since the last send, throttled by
    /// SEND_INTERVAL the same way every other send in this file is.
    /// Deliberately does NOT clamp to effective_bounds() the way
    /// send_calibration() does for automatic mode -- Manual mode is
    /// meant to reach the DAC's full raw range, not the same safety-
    /// narrowed bounds automatic searches respect. Heartbeat/
    /// ConnectionLost tracking is the same as automatic mode's own (see
    /// maybe_trip_connection_lost).
    fn poll_manual(&mut self, link: &mut MspLink, vtx_ready: bool, meter_ready: bool) -> anyhow::Result<bool> {
        if !link.can_send_now() {
            return Ok(false);
        }
        if self.maybe_trip_connection_lost(vtx_ready, meter_ready, self.manual_dac_mv) {
            return Ok(false);
        }
        // Frequency changes take priority over the regular per-step DAC
        // send, same as poll_automatic()'s own handling -- start_manual()
        // and advance_position() both call begin_frequency() correctly,
        // which sets pending_frequency_push (and, for Manual/ManualBand
        // meters, routes through AwaitingFreqConfirm first); this is what
        // actually issues the VTX_CONFIG retune once that's ready.
        if let Some(freq_mhz) = self.pending_frequency_push.take() {
            let level = self.levels.first().copied().unwrap_or(1);
            let payload = self.build_vtx_config_frequency_payload(freq_mhz, level, false); // normal retune -- pitmode off
            link.send_v1(function::VTX_CONFIG as u8, &payload)?;
            link.note_sent(MspCommandKind::Retune);
            debug!(target: "vtx", "[sweep] manual: pushed frequency change to {freq_mhz}MHz (power={level})");
            self.pending_sends.push_back(PendingSend::RequestVtxConfig);
            return Ok(true); // next tick will wait out the settle gate before sending anything else
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

    /// Why a ScanPa step ended -- see ScanPaOutcome. Records the result,
    /// sets cell status, and either hands off to ScanDetector (Success)
    /// or skips it entirely and advances past this point (PaFailure,
    /// NotSettled) or advances past it after still recording the
    /// attempt (Uncalibrated is handled the same way as PaFailure/
    /// NotSettled here -- see below).
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
            // Skip ScanDetector entirely -- if the PA is thermally
            // rolling off (PaFailure), or hadn't even finished its
            // boost-enable transient (NotSettled), driving it further
            // for a detector search only makes that worse, or searches
            // against a target that hasn't stabilized yet. Mark the
            // detector cell the same way (no search ran there either)
            // and account for BOTH steps (this ScanPa op plus the
            // ScanDetector op that never ran) in completed_steps, same
            // reasoning as skip_multiple() -- otherwise the progress bar
            // would never reach 100%.
            let cell_status = if not_settled { CellStatus::NotSettled } else { CellStatus::PaFailure };
            let level_status = if not_settled { LevelStatus::NotSettled } else { LevelStatus::PaFailure };
            Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), cell_status);
            self.completed_steps += 1;
            self.per_level_status.insert(level, level_status);
            self.advance_position();
            return;
        }

        // ScanDetector runs immediately after ScanPa for the same (level,
        // freq) -- including on ScanPaOutcome::Uncalibrated (success ==
        // false but neither pa_failure nor not_settled): the search
        // still produced a vbias_mv, it just never confirmed it reaches
        // target, so ScanDetector still runs against it (see
        // ScanPaOutcome::Uncalibrated's own doc comment -- "the failure
        // is about vbias, not about whether the PA responds usably at
        // all").
        // Starting point is a small step toward LESS power from the just-found
        // calibration value (matches the original script's intent: start
        // slightly on the safe side before searching for where the reading
        // crosses target). BUG FIXED here: the original script (and this
        // file's first version) used a hardcoded "vbias_mv - 5", which only means
        // "less power" under normal polarity -- on an inverted board (like
        // our confirmed RTC76401) that's actually 5mV toward MORE power. A
        // real run showed this seeding ScanDetector 5mV past a hard limit
        // that had just been set. Now direction-aware via power_up_step(),
        // and clamped to the current bounds regardless.
        let up = power_up_step(self.sign_inverted);
        let (bound_lo, bound_hi) = self.effective_bounds(level);
        let detector_start_vbias_mv = (vbias_mv - up * 5).clamp(bound_lo, bound_hi);
        Self::set_cell_status(&mut self.det_cell_status, (level, self.freq_idx), CellStatus::Current);
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
            pa_failure: false, // only a ScanPa result can trigger PA Failure -- see finish_scan_pa
            not_settled: false, // only a ScanPa result can trigger NotSettled -- see finish_scan_pa
        });
        Self::set_cell_status(
            &mut self.det_cell_status,
            (level, self.freq_idx),
            if success { CellStatus::Calibrated } else { CellStatus::Uncalibrated },
        );
        self.per_level_status.insert(level, LevelStatus::Done);
        self.advance_position();
    }

    /// Builds an MSP_VTX_CONFIG payload that retunes the VTX to
    /// `freq_mhz` directly (band=0) -- independent of the stored
    /// VtxSelectionState (a separate, user-facing concept this file has
    /// no access to and doesn't directly touch).
    /// `power` just needs to be a valid level index so vtx_apply_hw()
    /// picks a sensible RTC6705 register while retuning; the
    /// SET_PACALIBRATION calls that immediately follow set the real
    /// level and DAC value regardless (and re-set the RTC6705 register
    /// too, via vtx_msp_set_calibration()), so this doesn't need to be
    /// exact.
    ///
    /// Every actual send of this payload is followed by queuing
    /// PendingSend::RequestVtxConfig (see its own doc comment) -- this
    /// function itself only builds bytes, it doesn't send or track
    /// anything.
    fn build_vtx_config_frequency_payload(&self, freq_mhz: u16, power: u8, pitmode: bool) -> Vec<u8> {
        let mut p = vec![0u8; 15];
        p[0] = 5; // VTXDEV_MSP
        p[1] = 0; // band=0 -> use the raw frequency field directly
        p[2] = 0; // channel (unused when band=0)
        p[3] = power;
        p[4] = pitmode as u8;
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
/// without mutating the stored VtxSelectionState -- used for the
/// safe-state-on-connect and safe-state-on-reconnect pushes (see
/// worker.rs), where there's no active sweep whose frequency needs
/// preserving, so the current selection (resolved against `table`) is
/// exactly what's wanted. NOT used for abort()/skip_multiple()/
/// exit_manual() -- those compute their own payload internally now (see
/// SweepEngine::safe_state_payload_at_current_point()), since a sweep in
/// progress needs to preserve its OWN last-used frequency, not revert to
/// the separate, unrelated Frequency-panel selection.
pub fn safe_state_payload(table: &VtxTableConfig, selection: &VtxSelectionState) -> Vec<u8> {
    let mut sel = *selection; // Copy -- cheap, no clone() needed
    sel.pitmode = true;
    sel.encode_vtx_config_response(table)
}
