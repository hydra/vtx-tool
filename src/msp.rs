
use anyhow::{bail, Result};
use std::io::{Read, Write};
use std::time::{Duration, Instant};

pub mod function {
    pub const VTX_CONFIG: u16 = 88;
    pub const VTXTABLE_POWERLEVEL: u16 = 138;
    pub const SET_VTXTABLE_POWERLEVEL: u16 = 228;
    pub const VTXTABLE_BAND: u16 = 137;
    pub const SET_VTXTABLE_BAND: u16 = 227;
    pub const DEBUG: u16 = 254;
    pub const STATUS: u16 = 101;
    pub const RC: u16 = 105;
    pub const PACALTABLE: u16 = 0x4800;
    pub const SET_PACALTABLE: u16 = 0x4801;
    pub const PACALIBRATION: u16 = 0x4802;
    pub const SET_PACALIBRATION: u16 = 0x4803;
    pub const DISPLAYPORT: u16 = 182;
    pub const SET_OSD_CANVAS: u16 = 188;
}

#[derive(Debug, Clone, Copy, Default)]
pub struct PaCalibrationReading {
    pub power_level: u8,
    pub vref_mv: u16,
    pub detector_mv: u16,
    pub boost_on: Option<bool>,
    pub rtc6705_level: Option<u8>,
    pub pid_active: Option<bool>,
    pub frequency_mhz: Option<u16>,
    pub session_active: Option<bool>,
    pub ntc_raw: Option<u16>,
    pub pa_temp_c: Option<f32>,
}

pub fn decode_pa_calibration_reading(payload: &[u8]) -> Result<PaCalibrationReading> {
    if payload.len() < 5 {
        bail!("PACALIBRATION payload too short: {} bytes", payload.len());
    }
    let (boost_on, rtc6705_level, pid_active, frequency_mhz) = if payload.len() >= 10 {
        (
            Some(payload[5] != 0),
            Some(payload[6]),
            Some(payload[7] != 0),
            Some((payload[8] as u16) | ((payload[9] as u16) << 8)),
        )
    } else {
        (None, None, None, None)
    };
    let session_active = if payload.len() >= 11 { Some(payload[10] != 0) } else { None };
    let (ntc_raw, pa_temp_c) = if payload.len() >= 15 {
        let ntc_raw = (payload[11] as u16) | ((payload[12] as u16) << 8);
        let pa_temp_c_x10 = (payload[13] as i16) | ((payload[14] as i16) << 8);
        (Some(ntc_raw), Some(pa_temp_c_x10 as f32 / 10.0))
    } else {
        (None, None)
    };
    Ok(PaCalibrationReading {
        power_level: payload[0],
        vref_mv: (payload[1] as u16) | ((payload[2] as u16) << 8),
        detector_mv: (payload[3] as u16) | ((payload[4] as u16) << 8),
        boost_on,
        rtc6705_level,
        pid_active,
        frequency_mhz,
        session_active,
        ntc_raw,
        pa_temp_c,
    })
}

pub fn encode_pa_calibration_request(power_level: u8, mv: Option<u16>, session_active: bool, boost_mode: u8) -> Vec<u8> {
    let mv = mv.unwrap_or(0);
    vec![
        power_level,
        (mv & 0xff) as u8,
        (mv >> 8) as u8,
        if session_active { 1 } else { 0 },
        boost_mode,
    ]
}

pub mod displayport_cmd {
    pub const KEEPALIVE: u8 = 0;
    pub const RELEASE: u8 = 1;
    pub const CLEAR: u8 = 2;
    pub const DRAW_STRING: u8 = 3;
    pub const DRAW_SCREEN: u8 = 4;
}

pub const DISPLAYPORT_COLUMNS: u8 = 30;
pub const DISPLAYPORT_ROWS: u8 = 16;

pub fn encode_displayport_keepalive() -> Vec<u8> {
    vec![displayport_cmd::KEEPALIVE]
}

pub fn encode_displayport_release() -> Vec<u8> {
    vec![displayport_cmd::RELEASE]
}

pub fn encode_displayport_clear() -> Vec<u8> {
    vec![displayport_cmd::CLEAR]
}

pub fn encode_displayport_draw_screen() -> Vec<u8> {
    vec![displayport_cmd::DRAW_SCREEN]
}

pub fn encode_displayport_draw_string(row: u8, col: u8, text: &str) -> Vec<u8> {
    let max_len = DISPLAYPORT_COLUMNS.saturating_sub(col) as usize;
    let bytes = text.as_bytes();
    let bytes = if bytes.len() > max_len { &bytes[..max_len] } else { bytes };
    let mut v = vec![displayport_cmd::DRAW_STRING, row, col, 0];
    v.extend_from_slice(bytes);
    v
}

pub fn decode_osd_canvas(payload: &[u8]) -> Option<(u8, u8)> {
    if payload.len() < 2 {
        return None;
    }
    Some((payload[0], payload[1]))
}

#[derive(Debug, Clone, Default)]
pub struct PaCalibration {
    pub idx: u8,
    pub m_w: u16,
    pub value: [u16; 7],
    pub detector: [u16; 7],
    pub ext_pa_enable: bool,
    pub rtc6705_level: u8,
    pub dac_sign_inverted: bool,
}

#[derive(Debug, Clone, Default)]
pub struct VtxConfig {
    pub vtx_type: u8,
    pub band: u8,
    pub channel: u8,
    pub power: u8,
    pub pitmode: bool,
    pub frequency_mhz: u16,
    pub device_ready: bool,
    pub low_power_disarm: u8,
    pub pit_mode_freq: u16,
    pub vtx_table_available: bool,
    pub band_count: u8,
    pub channel_count: u8,
    pub power_level_count: u8,
}

pub fn decode_vtx_config(payload: &[u8]) -> Result<VtxConfig> {
    if payload.len() < 15 {
        bail!("MSP_VTX_CONFIG payload too short: {} bytes", payload.len());
    }
    Ok(VtxConfig {
        vtx_type: payload[0],
        band: payload[1],
        channel: payload[2],
        power: payload[3],
        pitmode: payload[4] != 0,
        frequency_mhz: (payload[5] as u16) | ((payload[6] as u16) << 8),
        device_ready: payload[7] != 0,
        low_power_disarm: payload[8],
        pit_mode_freq: (payload[9] as u16) | ((payload[10] as u16) << 8),
        vtx_table_available: payload[11] != 0,
        band_count: payload[12],
        channel_count: payload[13],
        power_level_count: payload[14],
    })
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct VtxBand {
    pub index: u8,
    pub name: String,
    pub letter: char,
    pub is_factory: bool,
    pub channel_count: u8,
    pub freqs_mhz: [u16; 8],
}

impl Default for VtxBand {
    fn default() -> Self {
        Self {
            index: 0,
            name: String::new(),
            letter: '?',
            is_factory: false,
            channel_count: 8,
            freqs_mhz: [0; 8],
        }
    }
}

pub fn decode_vtx_band(payload: &[u8]) -> Result<VtxBand> {
    if payload.len() < 29 {
        bail!("SET_VTXTABLE_BAND payload too short: {} bytes", payload.len());
    }
    let name_len = (payload[1] as usize).min(8);
    let name = String::from_utf8_lossy(&payload[2..2 + name_len])
        .trim_end()
        .to_string();
    let mut freqs_mhz = [0u16; 8];
    for (i, f) in freqs_mhz.iter_mut().enumerate() {
        *f = (payload[13 + i * 2] as u16) | ((payload[14 + i * 2] as u16) << 8);
    }
    Ok(VtxBand {
        index: payload[0],
        name,
        letter: payload[10] as char,
        is_factory: payload[11] != 0,
        channel_count: payload[12],
        freqs_mhz,
    })
}

pub fn encode_vtx_band(band: &VtxBand) -> Vec<u8> {
    let mut p = vec![0u8; 29];
    p[0] = band.index;
    p[1] = 8;
    let name_bytes = band.name.as_bytes();
    for i in 0..8 {
        p[2 + i] = *name_bytes.get(i).unwrap_or(&b' ');
    }
    p[10] = band.letter as u8;
    p[11] = if band.is_factory { 1 } else { 0 };
    p[12] = band.channel_count;
    for (i, f) in band.freqs_mhz.iter().enumerate() {
        p[13 + i * 2] = (f & 0xff) as u8;
        p[14 + i * 2] = (f >> 8) as u8;
    }
    p
}

#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct VtxPowerLevel {
    pub index: u8,
    pub m_w: u16,
    pub label: String,
}

pub fn decode_vtx_power_level(payload: &[u8]) -> Result<VtxPowerLevel> {
    if payload.len() < 4 {
        bail!("SET_VTXTABLE_POWERLEVEL payload too short: {} bytes", payload.len());
    }
    let label_len = (payload[3] as usize).min(payload.len().saturating_sub(4));
    let label = String::from_utf8_lossy(&payload[4..4 + label_len]).to_string();
    Ok(VtxPowerLevel {
        index: payload[0],
        m_w: (payload[1] as u16) | ((payload[2] as u16) << 8),
        label,
    })
}

pub fn encode_vtx_power_level(pl: &VtxPowerLevel) -> Vec<u8> {
    let label_bytes = pl.label.as_bytes();
    let label_len = label_bytes.len().min(255) as u8;
    let mut p = vec![pl.index, (pl.m_w & 0xff) as u8, (pl.m_w >> 8) as u8, label_len];
    p.extend_from_slice(&label_bytes[..label_len as usize]);
    p
}

#[derive(Debug, Clone)]
pub struct MspFrame {
    pub is_v2: bool,
    pub msp_type: u8,
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

pub fn build_command_v1(cmd: u8, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![payload.len() as u8, cmd];
    frame.extend_from_slice(payload);
    let csum = checksum_v1(&frame);
    let mut out = b"$M<".to_vec();
    out.extend_from_slice(&frame);
    out.push(csum);
    out
}

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

#[derive(Debug, Clone, Copy)]
pub enum MspCommandKind {
    Retune,
    Calibration,
    Other,
}

impl MspCommandKind {
    pub fn settle_duration(self) -> Duration {
        match self {
            MspCommandKind::Retune => Duration::from_millis(50),
            MspCommandKind::Calibration | MspCommandKind::Other => Duration::ZERO,
        }
    }
}

pub struct MspLink {
    port: Box<dyn serialport::SerialPort>,
    can_send_at: Instant,
    tx_count: u64,
    rx_count: u64,
}

impl MspLink {
    pub fn open(path: &str, baud: u32) -> Result<Self> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(50))
            .open()?;
        Ok(Self { port, can_send_at: Instant::now(), tx_count: 0, rx_count: 0 })
    }

    pub fn can_send_now(&self) -> bool {
        Instant::now() >= self.can_send_at
    }

    pub fn blocked_for(&self) -> Option<Duration> {
        let now = Instant::now();
        if now >= self.can_send_at {
            None
        } else {
            Some(self.can_send_at - now)
        }
    }

    pub fn note_sent(&mut self, kind: MspCommandKind) {
        let until = Instant::now() + kind.settle_duration();
        if until > self.can_send_at {
            self.can_send_at = until;
        }
    }

    pub fn tx_rx_counts(&self) -> (u64, u64) {
        (self.tx_count, self.rx_count)
    }

    pub fn send(&mut self, frame: &[u8]) -> Result<()> {
        if !self.can_send_now() {
            bail!(
                "send blocked: {:?} remaining before the settle window from a previous command clears",
                self.blocked_for().unwrap_or_default()
            );
        }
        self.port.write_all(frame)?;
        self.tx_count += 1;
        Ok(())
    }

    pub fn send_v1(&mut self, cmd: u8, payload: &[u8]) -> Result<()> {
        self.send(&build_command_v1(cmd, payload))
    }

    pub fn send_v2(&mut self, cmd: u16, payload: Option<&[u8]>) -> Result<()> {
        self.send(&build_command_v2(cmd, payload))
    }

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
            if let Some(frame) = self.try_read_after_dollar(deadline)? {
                self.rx_count += 1;
                return Ok(Some(frame));
            }
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
        let mut hdr = [0u8; 5];
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
            return Ok(None);
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
    if payload.len() >= 33 {
        if entry.idx == 0 {
            entry.dac_sign_inverted = payload[31] != 0;
        } else {
            entry.ext_pa_enable = payload[31] != 0;
        }
        entry.rtc6705_level = payload[32];
    }
    Ok(entry)
}

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
