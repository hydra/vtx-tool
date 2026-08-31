
use anyhow::{bail, Result};
use serialport::ClearBuffer;
use std::io::{BufRead, BufReader, Write};
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PowerMeterKind {
    ImmersionRcV1,
}

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

    pub fn read_mw(&mut self, timeout: Duration) -> Result<f32> {
        Ok(10f32.powf(self.read_dbm(timeout)? / 10.0))
    }
}
