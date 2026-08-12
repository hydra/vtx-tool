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

// Implemented by hand rather than #[derive(clap::ValueEnum)] -- the
// derive was failing to satisfy ValueEnum's own trait bounds in a real
// build (clap_builder-4.6.6), for reasons not pinned down from the error
// alone. Only two small methods for one variant right now, so hand-
// written sidesteps needing to diagnose a macro/version interaction
// blind; revisit the derive if this gets unwieldy once more meter kinds
// exist.
impl clap::ValueEnum for PowerMeterKind {
    fn value_variants<'a>() -> &'a [Self] {
        &[PowerMeterKind::ImmersionRcV1]
    }

    fn to_possible_value(&self) -> Option<clap::builder::PossibleValue> {
        Some(match self {
            PowerMeterKind::ImmersionRcV1 => clap::builder::PossibleValue::new("immersionrc-v1"),
        })
    }
}

impl PowerMeterKind {
    pub const ALL: &'static [PowerMeterKind] = &[PowerMeterKind::ImmersionRcV1];

    pub fn name(self) -> &'static str {
        match self {
            PowerMeterKind::ImmersionRcV1 => "ImmersionRC V1",
        }
    }

    pub fn default_baud(self) -> u32 {
        match self {
            PowerMeterKind::ImmersionRcV1 => 115200,
        }
    }

    /// UNVERIFIED against real hardware timing -- a conservative
    /// placeholder based on the protocol just being a simple ASCII
    /// "D\r\n" -> text reply exchange, not a documented rate spec from
    /// ImmersionRC. Tighten this once you've bench-tested how fast the
    /// meter can actually respond reliably without falling behind.
    pub fn max_update_hz(self) -> u32 {
        match self {
            PowerMeterKind::ImmersionRcV1 => 20,
        }
    }
}

impl Default for PowerMeterKind {
    fn default() -> Self {
        PowerMeterKind::ImmersionRcV1
    }
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

    /// ImmersionRC Power Meter V1 serial protocol: send "D\r\n", receive
    /// back a dBm reading as ASCII text terminated by CRLF, e.g.
    /// "-40.91\r\n". This is deliberately NOT a port of cImmersionRC.py
    /// -- that script targets the V2 meter, which also supports a
    /// frequency-set command and async push updates; nothing here
    /// assumes the V1 meter supports those.
    ///
    /// Reads a full line via a BufReader rather than one byte at a time,
    /// and clears stale input before writing each command -- both are
    /// fixes for real symptoms seen on Windows (spurious "semaphore
    /// timeout" errors and misaligned command/response pairs from a
    /// byte-by-byte read loop).
    fn read_dbm_immersionrc_v1(&mut self, timeout: Duration) -> Result<f32> {
        let _ = self.reader.get_mut().clear(ClearBuffer::Input);

        self.reader.get_mut().set_timeout(timeout)?;
        self.reader.get_mut().write_all(b"D\r\n")?;

        let mut line = String::new();
        loop {
            line.clear();
            let n = self.reader.read_line(&mut line)?;
            if n == 0 {
                bail!("power meter closed the connection");
            }
            let trimmed = line.trim();
            if trimmed.is_empty() {
                continue;
            }
            return trimmed.parse::<f32>().map_err(|e| {
                anyhow::anyhow!("meter replied '{trimmed}' (not a number): {e}")
            });
        }
    }

    /// Convenience: dBm -> mW.
    pub fn read_mw(&mut self, timeout: Duration) -> Result<f32> {
        Ok(10f32.powf(self.read_dbm(timeout)? / 10.0))
    }
}
