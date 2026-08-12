//! Power meter abstraction. `PowerMeterKind` picks the serial protocol,
//! default baud, and max sustainable update rate -- currently only
//! ImmersionRC V1 exists, but this is the seam for adding more later
//! (a match arm per kind here, plus a UI entry in app.rs's dropdown).

use anyhow::{bail, Result};
use serialport::ClearBuffer;
use std::io::{BufRead, BufReader, Write};
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PowerMeterKind {
    ImmersionRcV1,
}

impl PowerMeterKind {
    pub const ALL: &'static [PowerMeterKind] = &[PowerMeterKind::ImmersionRcV1];

    pub fn name(self) -> &'static str {
        match self {
            PowerMeterKind::ImmersionRcV1 => "ImmersionRC V1",
        }
    }

    /// The string used on the command line (--meter-kind immersionrc-v1).
    /// Kept in sync with parse_cli() below by hand (one match arm each);
    /// with only one variant right now that's simpler than any macro
    /// trickery, and avoids depending on clap's ValueEnum/value_parser!
    /// inference, which two separate attempts (derived, then hand-
    /// implemented ValueEnum) both failed to get past -- same identical
    /// "trait bounds not satisfied" error either way, which points at a
    /// dependency-graph issue (most likely two incompatible copies of
    /// clap/clap_builder resolved -- the same class of problem this
    /// session already hit once with egui/egui_dock) rather than
    /// anything wrong with the impl itself. `cargo tree -i clap_builder`
    /// would confirm/fix that, but it's no longer blocking anything --
    /// see parse_cli() below, which sidesteps the whole trait chain.
    pub fn cli_name(self) -> &'static str {
        match self {
            PowerMeterKind::ImmersionRcV1 => "immersionrc-v1",
        }
    }

    pub fn default_baud(self) -> u32 {
        match self {
            PowerMeterKind::ImmersionRcV1 => 115200,
        }
    }

    /// Bench-confirmed: faster than this produces errors in practice on
    /// real hardware (not a placeholder guess anymore, unlike when this
    /// was first written at 20Hz).
    pub fn max_update_hz(self) -> u32 {
        match self {
            PowerMeterKind::ImmersionRcV1 => 5,
        }
    }

    /// True if this meter has no way for the tool to command its
    /// listening frequency (cImmersionRC.py's "F<n>\r\n" is a V2-meter
    /// command; nothing here assumes the V1 meter supports it -- see
    /// read_dbm_immersionrc_v1's doc comment below). When true, the
    /// calibration sweep must pause at each frequency change and get an
    /// explicit user confirmation that they've retuned the meter by hand
    /// before continuing.
    pub fn requires_manual_frequency(self) -> bool {
        match self {
            PowerMeterKind::ImmersionRcV1 => true,
        }
    }
}

impl Default for PowerMeterKind {
    fn default() -> Self {
        PowerMeterKind::ImmersionRcV1
    }
}

/// clap value_parser function for --meter-kind (wired in via
/// `#[arg(long, value_parser = power_meter::parse_cli)]` in main.rs).
/// The Args field stays a real `PowerMeterKind`, not a String -- clap
/// calls this function with the raw argument text and uses whatever it
/// returns (or reports the Err string back to the user as a normal
/// clap parse error), no ValueEnum/trait-inference machinery involved
/// at all. This is the standard, well-supported clap mechanism for a
/// custom type that isn't going through #[derive(ValueEnum)].
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
        match kind {
            PowerMeterKind::ImmersionRcV1 => {
                let port = serialport::new(path, kind.default_baud())
                    .timeout(Duration::from_millis(300))
                    .open()?;
                Ok(Self {
                    kind,
                    reader: BufReader::new(port),
                })
            }
        }
    }

    pub fn kind(&self) -> PowerMeterKind {
        self.kind
    }

    pub fn read_dbm(&mut self, timeout: Duration) -> Result<f32> {
        match self.kind {
            PowerMeterKind::ImmersionRcV1 => self.read_dbm_immersionrc_v1(timeout),
        }
    }

    /// Sends the meter's presence-check command ("V\r\n" for ImmersionRC
    /// V1 -- a version query, going by convention with D=reading,
    /// F=frequency) and returns Ok(()) if ANY reply comes back, without
    /// caring what it says. Used purely to detect "is something actually
    /// there answering" -- separate from read_dbm's power polling, which
    /// is used for the graph/sweep and runs on its own (usually much
    /// slower) update_hz-driven cadence.
    pub fn check_alive(&mut self, timeout: Duration) -> Result<()> {
        match self.kind {
            PowerMeterKind::ImmersionRcV1 => self.send_and_read_line(b"V\r\n", timeout).map(|_| ()),
        }
    }

    /// ImmersionRC Power Meter V1 serial protocol: send "D\r\n", receive
    /// back a dBm reading as ASCII text terminated by CRLF, e.g.
    /// "-40.91\r\n". This is deliberately NOT a port of cImmersionRC.py
    /// -- that script targets the V2 meter, which also supports a
    /// frequency-set command and async push updates; nothing here
    /// assumes the V1 meter supports those.
    fn read_dbm_immersionrc_v1(&mut self, timeout: Duration) -> Result<f32> {
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
