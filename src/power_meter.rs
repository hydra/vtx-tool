//! Power meter abstraction. `PowerMeterKind` picks the serial protocol,
//! default baud, and max sustainable update rate; `FrequencyCapability`
//! (from `PowerMeterKind::capability()`) describes how -- or whether --
//! the tool can get the meter listening on a given frequency, which
//! drives the calibration sweep's per-frequency prompting behavior (see
//! calibration_engine.rs).

use anyhow::{bail, Result};
use serialport::ClearBuffer;
use std::io::{BufRead, BufReader, Write};
use std::time::Duration;

/// Describes how a meter's listening frequency can be set, in
/// increasing order of automation. This is what the calibration sweep
/// actually branches on (see calibration_engine.rs's begin_frequency):
/// Manual/ManualBand need a user prompt at each frequency change (with
/// ManualBand consolidating consecutive VTX frequencies that map to the
/// same nearest band into one prompt); ProgrammableBand/FullyProgrammable
/// don't, since the tool can just tell the meter directly.
#[derive(Debug, Clone, PartialEq)]
pub enum FrequencyCapability {
    /// Continuous range, no serial API to set it -- user must retune by
    /// hand. The sweep prompts for every individual frequency (no
    /// consolidation -- there's no concept of a "band" to group by).
    Manual { min_mhz: u32, max_mhz: u32 },
    /// One or more selectable bands, no serial API to select one -- the
    /// sweep prompts with the NEAREST band to each calibration
    /// frequency, consolidating consecutive frequencies that share the
    /// same nearest band into a single prompt.
    ManualBand { bands_mhz: Vec<u32> },
    /// Selectable bands AND a serial API (PowerMeter::set_frequency) to
    /// pick the nearest one automatically -- no prompt.
    ProgrammableBand { bands_mhz: Vec<u32> },
    /// Continuous range and a serial API to set the exact frequency --
    /// no prompt.
    FullyProgrammable { min_mhz: u32, max_mhz: u32 },
}

/// Standard band list shared by both ImmersionRC variants -- V1 can't
/// select these itself (ManualBand), V2 can (ProgrammableBand).
const IMMERSIONRC_BANDS_MHZ: &[u32] = &[5800, 2400, 1200, 900, 868, 433, 72, 35];

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PowerMeterKind {
    ImmersionRcV1,
    /// Protocol mostly unconfirmed -- see PowerMeter::set_frequency's doc
    /// comment for the one piece that IS confirmed (from cImmersionRC.py).
    ImmersionRcV2,
    /// Stand-in for "some meter with a manual range, unknown protocol" --
    /// reads/aliveness log a message and return an error rather than
    /// fabricate data. Fill in a real implementation when the protocol
    /// for an actual device is known.
    GenericManual,
    /// Stand-in for "some meter with a real frequency-set API, unknown
    /// protocol" (e.g. a spectrum analyzer) -- same treatment as
    /// GenericManual.
    GenericFullyProgrammable,
}

impl PowerMeterKind {
    pub const ALL: &'static [PowerMeterKind] = &[
        PowerMeterKind::ImmersionRcV1,
        PowerMeterKind::ImmersionRcV2,
        PowerMeterKind::GenericManual,
        PowerMeterKind::GenericFullyProgrammable,
    ];

    pub fn name(self) -> &'static str {
        match self {
            PowerMeterKind::ImmersionRcV1 => "ImmersionRC V1",
            PowerMeterKind::ImmersionRcV2 => "ImmersionRC V2",
            PowerMeterKind::GenericManual => "Generic Manual",
            PowerMeterKind::GenericFullyProgrammable => "Generic Fully Programmable",
        }
    }

    /// The string used on the command line (--meter-kind immersionrc-v1).
    /// Kept in sync with parse_cli() below by hand (one match arm each) --
    /// avoids clap's ValueEnum/value_parser! inference, which failed to
    /// compile earlier in this project for reasons that pointed at a
    /// dependency-graph issue rather than the impl itself; a plain
    /// function sidesteps that trait chain entirely.
    pub fn cli_name(self) -> &'static str {
        match self {
            PowerMeterKind::ImmersionRcV1 => "immersionrc-v1",
            PowerMeterKind::ImmersionRcV2 => "immersionrc-v2",
            PowerMeterKind::GenericManual => "generic-manual",
            PowerMeterKind::GenericFullyProgrammable => "generic-fully-programmable",
        }
    }

    pub fn default_baud(self) -> u32 {
        match self {
            PowerMeterKind::ImmersionRcV1 => 115200,
            // ASSUMPTION: same baud as V1 -- unconfirmed for V2 specifically.
            PowerMeterKind::ImmersionRcV2 => 115200,
            // Placeholder -- protocol (and therefore baud) unknown.
            PowerMeterKind::GenericManual | PowerMeterKind::GenericFullyProgrammable => 115200,
        }
    }

    /// Bench-confirmed for V1: faster than this produces errors in
    /// practice on real hardware. Everything else is an unconfirmed
    /// placeholder.
    pub fn max_update_hz(self) -> u32 {
        match self {
            PowerMeterKind::ImmersionRcV1 => 5,
            // ASSUMPTION: copied from V1 -- not bench-confirmed for V2.
            PowerMeterKind::ImmersionRcV2 => 5,
            // Conservative placeholder -- protocol unknown.
            PowerMeterKind::GenericManual | PowerMeterKind::GenericFullyProgrammable => 1,
        }
    }

    pub fn capability(self) -> FrequencyCapability {
        match self {
            PowerMeterKind::ImmersionRcV1 => FrequencyCapability::ManualBand {
                bands_mhz: IMMERSIONRC_BANDS_MHZ.to_vec(),
            },
            PowerMeterKind::ImmersionRcV2 => FrequencyCapability::ProgrammableBand {
                bands_mhz: IMMERSIONRC_BANDS_MHZ.to_vec(),
            },
            // Range is an arbitrary placeholder -- unknown for any real device.
            PowerMeterKind::GenericManual => FrequencyCapability::Manual { min_mhz: 5000, max_mhz: 10000 },
            PowerMeterKind::GenericFullyProgrammable => {
                FrequencyCapability::FullyProgrammable { min_mhz: 5000, max_mhz: 10000 }
            }
        }
    }
}

impl Default for PowerMeterKind {
    fn default() -> Self {
        PowerMeterKind::ImmersionRcV1
    }
}

/// Finds the value in `bands` closest to `freq_mhz`. `bands` must be
/// non-empty; if it's empty this just returns `freq_mhz` itself
/// unchanged (a defensive fallback, not expected to actually happen).
pub fn nearest_band(bands: &[u32], freq_mhz: u32) -> u32 {
    bands
        .iter()
        .copied()
        .min_by_key(|&b| freq_mhz.abs_diff(b))
        .unwrap_or(freq_mhz)
}

/// clap value_parser function for --meter-kind (wired in via
/// `#[arg(long, value_parser = power_meter::parse_cli)]` in main.rs).
/// The Args field stays a real `PowerMeterKind`, not a String -- clap
/// calls this function with the raw argument text and uses whatever it
/// returns (or reports the Err string back to the user as a normal
/// clap parse error), no ValueEnum/trait-inference machinery involved
/// at all.
pub fn parse_cli(s: &str) -> Result<PowerMeterKind, String> {
    PowerMeterKind::ALL
        .iter()
        .copied()
        .find(|k| k.cli_name().eq_ignore_ascii_case(s))
        .ok_or_else(|| {
            let choices: Vec<&str> = PowerMeterKind::ALL.iter().map(|k| k.cli_name()).collect();
            format!("unknown power meter kind '{s}' (expected one of: {})", choices.join(", "))
        })
}

pub struct PowerMeter {
    kind: PowerMeterKind,
    reader: BufReader<Box<dyn serialport::SerialPort>>,
}

impl PowerMeter {
    pub fn open(kind: PowerMeterKind, path: &str) -> Result<Self> {
        // Opening the port itself is protocol-independent -- valid for
        // every kind, including the two unknown-protocol stubs (reads on
        // those just fail loudly afterward rather than opening failing).
        let port = serialport::new(path, kind.default_baud())
            .timeout(Duration::from_millis(300))
            .open()?;
        Ok(Self {
            kind,
            reader: BufReader::new(port),
        })
    }

    pub fn kind(&self) -> PowerMeterKind {
        self.kind
    }

    pub fn read_dbm(&mut self, timeout: Duration) -> Result<f32> {
        match self.kind {
            PowerMeterKind::ImmersionRcV1 => self.read_dbm_immersionrc(timeout),
            // ASSUMPTION: V2 uses the same "D\r\n" -> ASCII dBm reply
            // protocol as V1 -- cImmersionRC.py (which is where
            // set_frequency's confirmed F-command below comes from) uses
            // the identical D-command for its reading loop, so this is a
            // reasonable guess, not a bench-confirmed one.
            PowerMeterKind::ImmersionRcV2 => self.read_dbm_immersionrc(timeout),
            PowerMeterKind::GenericManual | PowerMeterKind::GenericFullyProgrammable => {
                log::warn!(target: "meter", "read_dbm not implemented for {} -- protocol unknown", self.kind.name());
                bail!("read_dbm not implemented for {} (protocol unknown)", self.kind.name());
            }
        }
    }

    /// Sends the meter's presence-check command ("V\r\n" for the
    /// ImmersionRC protocol -- a version query, going by convention with
    /// D=reading, F=frequency) and returns Ok(()) if ANY reply comes
    /// back, without caring what it says. Used purely to detect "is
    /// something actually there answering" -- separate from read_dbm's
    /// power polling.
    pub fn check_alive(&mut self, timeout: Duration) -> Result<()> {
        match self.kind {
            PowerMeterKind::ImmersionRcV1 => self.send_and_read_line(b"V\r\n", timeout).map(|_| ()),
            // ASSUMPTION: same V-command as V1 -- unconfirmed for V2 specifically.
            PowerMeterKind::ImmersionRcV2 => self.send_and_read_line(b"V\r\n", timeout).map(|_| ()),
            PowerMeterKind::GenericManual | PowerMeterKind::GenericFullyProgrammable => {
                log::warn!(target: "meter", "check_alive not implemented for {} -- protocol unknown", self.kind.name());
                bail!("check_alive not implemented for {} (protocol unknown)", self.kind.name());
            }
        }
    }

    /// Commands the meter to listen on (or nearest to) `freq_mhz` --
    /// only meaningful for ProgrammableBand/FullyProgrammable kinds; the
    /// calibration sweep only calls this for those (see
    /// calibration_engine.rs's begin_frequency), but every kind has an
    /// implementation here for completeness/safety if called directly.
    ///
    /// ImmersionRC V2 is a log-only stub for now, EXCEPT one piece is
    /// actually confirmed: cImmersionRC.py's frequency setter sends
    /// `F{i}\r\n` where `i = (clamp(freq_mhz, 5650, 5950) - 5625) / 50`
    /// -- but that formula is specific to the narrow 5.6-5.95GHz band
    /// (50MHz channel spacing within it), and our band list spans
    /// several unrelated RF bands (2.4GHz, 1.2GHz, 900MHz, etc.) that
    /// formula says nothing about. Wiring in just the 5.8GHz case would
    /// leave a confusing half-implementation, so this stays a uniform
    /// log-only stub across all bands for now -- worth revisiting if the
    /// 5.8GHz case alone is useful sooner than the rest.
    pub fn set_frequency(&mut self, freq_mhz: u32) -> Result<()> {
        match self.kind {
            PowerMeterKind::ImmersionRcV1 => {
                log::warn!(target: "meter", "set_frequency() called on ImmersionRC V1, which has no frequency-set API (ManualBand) -- ignoring");
                Ok(())
            }
            PowerMeterKind::ImmersionRcV2 => {
                log::warn!(target: "meter", "set_frequency({freq_mhz}) not yet implemented for ImmersionRC V2 -- protocol unknown beyond the 5.8GHz case (see doc comment), no-op for now");
                Ok(())
            }
            PowerMeterKind::GenericManual => {
                log::warn!(target: "meter", "set_frequency() called on a Manual-capability meter, which has no frequency-set API -- ignoring");
                Ok(())
            }
            PowerMeterKind::GenericFullyProgrammable => {
                log::warn!(target: "meter", "set_frequency({freq_mhz}) not yet implemented for {} -- protocol unknown, no-op for now", self.kind.name());
                Ok(())
            }
        }
    }

    /// ImmersionRC "D\r\n" -> ASCII dBm reply, shared by V1 and V2 (see
    /// the ASSUMPTION note on read_dbm's V2 arm).
    fn read_dbm_immersionrc(&mut self, timeout: Duration) -> Result<f32> {
        let line = self.send_and_read_line(b"D\r\n", timeout)?;
        line.parse::<f32>()
            .map_err(|e| anyhow::anyhow!("meter replied '{line}' (not a number): {e}"))
    }

    /// Sends `cmd`, reads back one non-empty line. Clears stale input
    /// before writing each command -- a fix for a real symptom seen on
    /// Windows (misaligned command/response pairs from a byte-by-byte
    /// read loop this used to use). Reads a full line via a BufReader
    /// rather than one byte at a time -- the other Windows-specific fix,
    /// for spurious "semaphore timeout" errors under rapid tiny reads.
    fn send_and_read_line(&mut self, cmd: &[u8], timeout: Duration) -> Result<String> {
        let _ = self.reader.get_mut().clear(ClearBuffer::Input);

        self.reader.get_mut().set_timeout(timeout)?;
        self.reader.get_mut().write_all(cmd)?;

        let mut line = String::new();
        loop {
            line.clear();
            let n = self.reader.read_line(&mut line)?;
            if n == 0 {
                bail!("power meter closed the connection");
            }
            let trimmed = line.trim();
            if !trimmed.is_empty() {
                return Ok(trimmed.to_string());
            }
        }
    }

    /// Convenience: dBm -> mW.
    pub fn read_mw(&mut self, timeout: Duration) -> Result<f32> {
        Ok(10f32.powf(self.read_dbm(timeout)? / 10.0))
    }
}
