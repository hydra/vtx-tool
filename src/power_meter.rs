//! ImmersionRC Power Meter V1 serial protocol: send "D\r\n", receive back
//! a dBm reading as ASCII text terminated by CRLF, e.g. "-40.91\r\n".
//!
//! This is deliberately NOT a port of cImmersionRC.py -- that script
//! targets the V2 meter, which also supports a frequency-set command
//! ("F<n>\r\n") and async push updates. Nothing here assumes the V1
//! meter supports frequency setting; if you need it, confirm the V1's
//! command set first rather than assuming parity with V2.

use anyhow::{bail, Result};
use std::io::{Read, Write};
use std::time::{Duration, Instant};

pub struct PowerMeter {
    port: Box<dyn serialport::SerialPort>,
}

impl PowerMeter {
    pub fn open(path: &str, baud: u32) -> Result<Self> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(50))
            .open()?;
        Ok(Self { port })
    }

    /// Sends "D\r\n" and blocks for a line back, parsed as dBm.
    pub fn read_dbm(&mut self, timeout: Duration) -> Result<f32> {
        self.port.write_all(b"D\r\n")?;

        let deadline = Instant::now() + timeout;
        let mut line = Vec::new();
        let mut byte = [0u8; 1];

        loop {
            if Instant::now() >= deadline {
                bail!("power meter did not respond within {:?}", timeout);
            }
            match self.port.read_exact(&mut byte) {
                Ok(()) => {}
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => return Err(e.into()),
            }
            match byte[0] {
                b'\r' | b'\n' => {
                    if line.is_empty() {
                        continue; // tolerate a leading CR/LF before the real reply
                    }
                    break;
                }
                b => line.push(b),
            }
        }

        let text = String::from_utf8_lossy(&line);
        text.trim()
            .parse::<f32>()
            .map_err(|e| anyhow::anyhow!("could not parse power meter reply '{}': {}", text, e))
    }

    /// Convenience: dBm -> mW.
    pub fn read_mw(&mut self, timeout: Duration) -> Result<f32> {
        Ok(10f32.powf(self.read_dbm(timeout)? / 10.0))
    }
}
