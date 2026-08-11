//! MSP v1/v2 framing, checksums, and a synchronous byte-at-a-time parser.
//! Ported from cMsp.py's wire protocol -- function IDs, payload byte
//! layouts, and checksum algorithms match that reference exactly.
//!
//! NOT YET IMPLEMENTED HERE: the calibration sweep itself. MSP_SET_PACALIBRATION's
//! exact semantics for value=0 vs value=1 vs a real mV number are only
//! inferable from how the Python client happens to call it (see
//! rf_calibration.py's scanPa/scanDetector) -- worth confirming against
//! the actual VTX-side C handler before this crate sends live DAC
//! commands to real hardware.

use anyhow::{bail, Result};
use std::io::{Read, Write};
use std::time::{Duration, Instant};

/// Custom + standard MSP function IDs used by this fork (see cMsp.py's
/// MspFunction class).
pub mod function {
    pub const VTX_CONFIG: u16 = 88;
    pub const VTXTABLE_POWERLEVEL: u16 = 138;
    pub const SET_VTXTABLE_POWERLEVEL: u16 = 228;
    pub const VTXTABLE_BAND: u16 = 137;
    pub const SET_VTXTABLE_BAND: u16 = 227;
    pub const DEBUG: u16 = 254;
    pub const STATUS: u16 = 101;
    pub const RC: u16 = 105;
    pub const EEPROM_WRITE: u16 = 250;
    pub const PACALTABLE: u16 = 0x4800;
    pub const SET_PACALTABLE: u16 = 0x4801;
    pub const PACALIBRATION: u16 = 0x4802;
    pub const SET_PACALIBRATION: u16 = 0x4803;
}

/// One decoded PA calibration table entry, matching cMsp.py's
/// PaCalibration (idx, mW, 7 calibration mV points, 7 detector points).
#[derive(Debug, Clone, Default)]
pub struct PaCalibration {
    pub idx: u8,
    pub m_w: u16,
    pub value: [u16; 7],
    pub detector: [u16; 7],
}

/// A fully decoded, checksum-verified MSP frame.
#[derive(Debug, Clone)]
pub struct MspFrame {
    pub is_v2: bool,
    pub msp_type: u8, // b'<' request, b'>' response, b'!' error
    pub function: u16,
    pub payload: Vec<u8>,
}

fn checksum_v1(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |csum, b| csum ^ b)
}

fn crc8_dvb_s2(mut crc: u8, byte: u8) -> u8 {
    crc ^= byte;
    for _ in 0..8 {
        crc = if crc & 0x80 != 0 {
            (crc << 1) ^ 0xD5
        } else {
            crc << 1
        };
    }
    crc
}

fn checksum_v2(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |crc, &b| crc8_dvb_s2(crc, b))
}

/// Builds an MSPv1 ($M<) command frame.
pub fn build_command_v1(cmd: u8, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![payload.len() as u8, cmd];
    frame.extend_from_slice(payload);
    let csum = checksum_v1(&frame);
    let mut out = b"$M<".to_vec();
    out.extend_from_slice(&frame);
    out.push(csum);
    out
}

/// Builds an MSPv2 ($X<) command frame. `payload = None` sends a
/// zero-length-payload request frame (matches build_msp_commandV2's
/// `payload != None` branch).
pub fn build_command_v2(cmd: u16, payload: Option<&[u8]>) -> Vec<u8> {
    let flag = 0u8;
    let mut frame = vec![flag, (cmd & 0xff) as u8, ((cmd >> 8) & 0xff) as u8];
    match payload {
        Some(p) => {
            frame.push((p.len() & 0xff) as u8);
            frame.push(((p.len() >> 8) & 0xff) as u8);
            frame.extend_from_slice(p);
        }
        None => {
            frame.push(0);
            frame.push(0);
        }
    }
    let csum = checksum_v2(&frame);
    let mut out = b"$X<".to_vec();
    out.extend_from_slice(&frame);
    out.push(csum);
    out
}

/// A serial link speaking MSP, with a blocking-with-timeout read loop.
/// Not async -- this tool is an interactive one-shot calibration run, not
/// a long-lived service, so a simple blocking model keeps it dependency-light.
pub struct MspLink {
    port: Box<dyn serialport::SerialPort>,
}

impl MspLink {
    pub fn open(path: &str, baud: u32) -> Result<Self> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(50))
            .open()?;
        Ok(Self { port })
    }

    pub fn send(&mut self, frame: &[u8]) -> Result<()> {
        self.port.write_all(frame)?;
        Ok(())
    }

    pub fn send_v1(&mut self, cmd: u8, payload: &[u8]) -> Result<()> {
        self.send(&build_command_v1(cmd, payload))
    }

    pub fn send_v2(&mut self, cmd: u16, payload: Option<&[u8]>) -> Result<()> {
        self.send(&build_command_v2(cmd, payload))
    }

    /// Blocks until one complete, checksum-valid frame arrives or `timeout`
    /// elapses. Returns Ok(None) on timeout (not an error -- the VTX may
    /// simply not have anything to send).
    pub fn read_frame(&mut self, timeout: Duration) -> Result<Option<MspFrame>> {
        let deadline = Instant::now() + timeout;
        let mut byte = [0u8; 1];

        loop {
            if Instant::now() >= deadline {
                return Ok(None);
            }
            match self.port.read_exact(&mut byte) {
                Ok(()) => {}
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => return Err(e.into()),
            }
            if byte[0] != b'$' {
                continue;
            }
            // Found '$' -- read the rest of the header to decide v1 vs v2.
            if let Some(frame) = self.try_read_after_dollar(deadline)? {
                return Ok(Some(frame));
            }
            // Bad frame (checksum mismatch, unexpected header) -- keep
            // scanning for the next '$' rather than giving up.
        }
    }

    fn read_byte(&mut self, deadline: Instant) -> Result<Option<u8>> {
        let mut b = [0u8; 1];
        loop {
            if Instant::now() >= deadline {
                return Ok(None);
            }
            match self.port.read_exact(&mut b) {
                Ok(()) => return Ok(Some(b[0])),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => return Err(e.into()),
            }
        }
    }

    fn try_read_after_dollar(&mut self, deadline: Instant) -> Result<Option<MspFrame>> {
        let marker = match self.read_byte(deadline)? {
            Some(b) => b,
            None => return Ok(None),
        };

        match marker {
            b'X' => self.read_v2_after_marker(deadline),
            b'M' => self.read_v1_after_marker(deadline),
            _ => Ok(None),
        }
    }

    fn read_v2_after_marker(&mut self, deadline: Instant) -> Result<Option<MspFrame>> {
        let msp_type = match self.read_byte(deadline)? {
            Some(b @ (b'<' | b'>' | b'!')) => b,
            _ => return Ok(None),
        };

        let mut crc = 0u8;
        let mut hdr = [0u8; 5]; // flag, fn_lo, fn_hi, size_lo, size_hi
        for slot in hdr.iter_mut() {
            let b = match self.read_byte(deadline)? {
                Some(b) => b,
                None => return Ok(None),
            };
            crc = crc8_dvb_s2(crc, b);
            *slot = b;
        }
        let function = (hdr[1] as u16) | ((hdr[2] as u16) << 8);
        let size = (hdr[3] as usize) | ((hdr[4] as usize) << 8);

        let mut payload = vec![0u8; size];
        for slot in payload.iter_mut() {
            let b = match self.read_byte(deadline)? {
                Some(b) => b,
                None => return Ok(None),
            };
            crc = crc8_dvb_s2(crc, b);
            *slot = b;
        }

        let recv_crc = match self.read_byte(deadline)? {
            Some(b) => b,
            None => return Ok(None),
        };
        if recv_crc != crc {
            return Ok(None); // bad frame, caller resumes scanning for '$'
        }

        Ok(Some(MspFrame {
            is_v2: true,
            msp_type,
            function,
            payload,
        }))
    }

    fn read_v1_after_marker(&mut self, deadline: Instant) -> Result<Option<MspFrame>> {
        let msp_type = match self.read_byte(deadline)? {
            Some(b @ (b'<' | b'>' | b'!')) => b,
            _ => return Ok(None),
        };

        let size = match self.read_byte(deadline)? {
            Some(b) => b as usize,
            None => return Ok(None),
        };
        let cmd = match self.read_byte(deadline)? {
            Some(b) => b,
            None => return Ok(None),
        };

        let mut payload = vec![0u8; size];
        for slot in payload.iter_mut() {
            let b = match self.read_byte(deadline)? {
                Some(b) => b,
                None => return Ok(None),
            };
            *slot = b;
        }

        let recv_csum = match self.read_byte(deadline)? {
            Some(b) => b,
            None => return Ok(None),
        };
        let mut frame_bytes = vec![size as u8, cmd];
        frame_bytes.extend_from_slice(&payload);
        if checksum_v1(&frame_bytes) != recv_csum {
            return Ok(None);
        }

        Ok(Some(MspFrame {
            is_v2: false,
            msp_type,
            function: cmd as u16,
            payload,
        }))
    }
}

/// Decodes a PACALTABLE/SET_PACALTABLE payload into one PaCalibration
/// entry, matching cMsp.py's MSP_SET_PACALTABLE handler byte layout:
/// [idx, mW_lo, mW_hi, value(7x u16 LE), detector(7x u16 LE)] = 31 bytes.
pub fn decode_pa_calibration(payload: &[u8]) -> Result<PaCalibration> {
    if payload.len() < 31 {
        bail!("PACALTABLE payload too short: {} bytes", payload.len());
    }
    let mut entry = PaCalibration {
        idx: payload[0],
        m_w: (payload[1] as u16) | ((payload[2] as u16) << 8),
        ..Default::default()
    };
    for i in 0..7 {
        entry.value[i] = (payload[3 + i * 2] as u16) | ((payload[4 + i * 2] as u16) << 8);
    }
    for i in 0..7 {
        entry.detector[i] = (payload[17 + i * 2] as u16) | ((payload[18 + i * 2] as u16) << 8);
    }
    Ok(entry)
}

/// Encodes a PaCalibration entry into a SET_PACALTABLE payload (same
/// layout as decode_pa_calibration expects).
pub fn encode_pa_calibration(entry: &PaCalibration) -> Vec<u8> {
    let mut p = Vec::with_capacity(31);
    p.push(entry.idx);
    p.push((entry.m_w & 0xff) as u8);
    p.push((entry.m_w >> 8) as u8);
    for v in entry.value {
        p.push((v & 0xff) as u8);
        p.push((v >> 8) as u8);
    }
    for v in entry.detector {
        p.push((v & 0xff) as u8);
        p.push((v >> 8) as u8);
    }
    p
}
