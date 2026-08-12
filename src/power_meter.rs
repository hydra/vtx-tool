//! ImmersionRC Power Meter V1 serial protocol: send "D\r\n", receive back
//! a dBm reading as ASCII text terminated by CRLF, e.g. "-40.91\r\n".
//!
//! This is deliberately NOT a port of cImmersionRC.py -- that script
//! targets the V2 meter, which also supports a frequency-set command
//! ("F<n>\r\n") and async push updates. Nothing here assumes the V1
//! meter supports frequency setting; if you need it, confirm the V1's
//! command set first rather than assuming parity with V2.
//!
//! Reads a full line via a BufReader rather than one byte at a time --
//! the original byte-by-byte read_exact() loop issued a raw OS read
//! call per byte with a very short (50ms) timeout, which is a known
//! source of spurious "semaphore timeout" errors (os error 121) on
//! Windows COM ports under rapid polling. It also clears any stale
//! input before writing each command: a single interrupted read used to
//! leave partial bytes in the buffer, which could misalign the next
//! command/response pair -- plausibly why the meter was sometimes seen
//! replying with its own "Syntax" error to what it received as a
//! corrupted command.

use anyhow::{bail, Result};
use serialport::ClearBuffer;
use std::io::{BufRead, BufReader, Write};
use std::time::Duration;

pub struct PowerMeter {
    reader: BufReader<Box<dyn serialport::SerialPort>>,
}

impl PowerMeter {
    pub fn open(path: &str, baud: u32) -> Result<Self> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(300))
            .open()?;
        Ok(Self {
            reader: BufReader::new(port),
        })
    }

    /// Sends "D\r\n" and blocks for a line back, parsed as dBm.
    pub fn read_dbm(&mut self, timeout: Duration) -> Result<f32> {
        // Discard anything left over from a previous, possibly
        // interrupted exchange before issuing a fresh command.
        let _ = self.reader.get_mut().clear(ClearBuffer::Input);

        self.reader.get_mut().set_timeout(timeout)?;
        self.reader.get_mut().write_all(b"D\r\n")?;

        // Loop past a stray leading CR/LF; bail on the underlying read
        // timing out (propagates as an io::Error from read_line, e.g.
        // TimedOut) or the port closing (0 bytes read).
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
