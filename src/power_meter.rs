
use anyhow::{bail, Result};
use serialport::ClearBuffer;
use std::io::{BufRead, BufReader, Write};
use std::time::Duration;

#[derive(Debug, Clone, PartialEq)]
pub enum FrequencyCapability {
    Manual { min_mhz: u32, max_mhz: u32 },
    ManualBand { bands_mhz: Vec<u32> },
    ProgrammableBand { bands_mhz: Vec<u32> },
    FullyProgrammable { min_mhz: u32, max_mhz: u32 },
}

const IMMERSIONRC_V1_BANDS_MHZ: &[u32] = &[5800, 2400, 1200, 900, 868, 433, 72, 35];

const IMMERSIONRC_V2_FREQ_TABLE_MHZ: &[u32] = &[
    35, 72, 433, 868, 915, 1200, 2400,
    5600, 5650, 5700, 5750, 5800, 5850, 5900, 5950, 6000,
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PowerMeterKind {
    ImmersionRcV1,
    ImmersionRcV2,
    GenericManual,
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
            PowerMeterKind::ImmersionRcV2 => 115200,
            PowerMeterKind::GenericManual | PowerMeterKind::GenericFullyProgrammable => 115200,
        }
    }

    pub fn max_update_hz(self) -> u32 {
        match self {
            PowerMeterKind::ImmersionRcV1 => 5,
            PowerMeterKind::ImmersionRcV2 => 5,
            PowerMeterKind::GenericManual | PowerMeterKind::GenericFullyProgrammable => 1,
        }
    }

    pub fn capability(self) -> FrequencyCapability {
        match self {
            PowerMeterKind::ImmersionRcV1 => FrequencyCapability::ManualBand {
                bands_mhz: IMMERSIONRC_V1_BANDS_MHZ.to_vec(),
            },
            PowerMeterKind::ImmersionRcV2 => FrequencyCapability::ProgrammableBand {
                bands_mhz: IMMERSIONRC_V2_FREQ_TABLE_MHZ.to_vec(),
            },
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

pub fn nearest_band(bands: &[u32], freq_mhz: u32) -> u32 {
    bands
        .iter()
        .copied()
        .min_by_key(|&b| freq_mhz.abs_diff(b))
        .unwrap_or(freq_mhz)
}

fn freq_to_v2_index(freq_mhz: u32) -> u8 {
    IMMERSIONRC_V2_FREQ_TABLE_MHZ
        .iter()
        .enumerate()
        .min_by_key(|&(_, &f)| freq_mhz.abs_diff(f))
        .map(|(i, _)| i as u8)
        .unwrap_or(0)
}

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
            PowerMeterKind::ImmersionRcV2 => self.read_dbm_immersionrc(timeout),
            PowerMeterKind::GenericManual | PowerMeterKind::GenericFullyProgrammable => {
                log::warn!(target: "meter", "read_dbm not implemented for {} -- protocol unknown", self.kind.name());
                bail!("read_dbm not implemented for {} (protocol unknown)", self.kind.name());
            }
        }
    }

    pub fn check_alive(&mut self, timeout: Duration) -> Result<()> {
        match self.kind {
            PowerMeterKind::ImmersionRcV1 => self.send_and_read_line(b"V\r\n", timeout).map(|_| ()),
            PowerMeterKind::ImmersionRcV2 => self.send_and_read_line(b"V\r\n", timeout).map(|_| ()),
            PowerMeterKind::GenericManual | PowerMeterKind::GenericFullyProgrammable => {
                log::warn!(target: "meter", "check_alive not implemented for {} -- protocol unknown", self.kind.name());
                bail!("check_alive not implemented for {} (protocol unknown)", self.kind.name());
            }
        }
    }

    pub fn set_frequency(&mut self, freq_mhz: u32) -> Result<()> {
        match self.kind {
            PowerMeterKind::ImmersionRcV1 => {
                log::warn!(target: "meter", "set_frequency() called on ImmersionRC V1, which has no frequency-set API (ManualBand) -- ignoring");
                Ok(())
            }
            PowerMeterKind::ImmersionRcV2 => {
                let idx = freq_to_v2_index(freq_mhz);
                let cmd = format!("F{idx}\r\n");
                self.send_and_read_line(cmd.as_bytes(), Duration::from_millis(300))
                    .map(|_| ())
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

    pub fn read_peak_dbm(&mut self, timeout: Duration) -> Result<f32> {
        match self.kind {
            PowerMeterKind::ImmersionRcV2 => {
                let line = self.send_and_read_line(b"E\r\n", timeout)?;
                line.parse::<f32>()
                    .map_err(|e| anyhow::anyhow!("meter replied '{line}' (not a number): {e}"))
            }
            _ => {
                log::warn!(target: "meter", "read_peak_dbm not implemented for {} -- only confirmed for ImmersionRC V2", self.kind.name());
                bail!("read_peak_dbm not implemented for {} (only confirmed for ImmersionRC V2)", self.kind.name());
            }
        }
    }

    pub fn read_frequency_raw(&mut self, timeout: Duration) -> Result<u32> {
        match self.kind {
            PowerMeterKind::ImmersionRcV2 => {
                let line = self.send_and_read_line(b"F\r\n", timeout)?;
                line.parse::<u32>()
                    .map_err(|e| anyhow::anyhow!("meter replied '{line}' (not a number): {e}"))
            }
            _ => {
                log::warn!(target: "meter", "read_frequency_raw not implemented for {} -- only confirmed for ImmersionRC V2", self.kind.name());
                bail!("read_frequency_raw not implemented for {} (only confirmed for ImmersionRC V2)", self.kind.name());
            }
        }
    }

    fn read_dbm_immersionrc(&mut self, timeout: Duration) -> Result<f32> {
        let line = self.send_and_read_line(b"D\r\n", timeout)?;
        line.parse::<f32>()
            .map_err(|e| anyhow::anyhow!("meter replied '{line}' (not a number): {e}"))
    }

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

    pub fn read_mw(&mut self, timeout: Duration) -> Result<f32> {
        Ok(10f32.powf(self.read_dbm(timeout)? / 10.0))
    }
}
