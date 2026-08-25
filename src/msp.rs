//! MSP v1/v2 framing, checksums, and a synchronous byte-at-a-time parser.
//! Ported from cMsp.py's wire protocol -- function IDs, payload byte
//! layouts, and checksum algorithms match that reference exactly.
//!
//! The calibration sweep itself lives in calibration_engine.rs, built on the
//! types here (PaCalibration, PaCalibrationReading, and the
//! encode/decode functions for SET_PACALIBRATION and MSP_PACALIBRATION).

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
    /// MSP_DISPLAYPORT -- see msp_displayport.c. Sent via send_v1(), not
    /// send_v2() -- it's a classic low-numbered MSP command, matching
    /// both convention and how this firmware's own MSP_SET_OSD_CANVAS
    /// reply already goes out (v1).
    pub const DISPLAYPORT: u16 = 182;
    /// MSP_SET_OSD_CANVAS -- the VTX's reply to a DisplayPort KEEPALIVE,
    /// reporting its own canvas size (columns, rows). Sent via v1 by the
    /// firmware (see msp_displayport.c) -- decode_osd_canvas() below reads
    /// it regardless of which framing carried it, same as every other
    /// incoming frame this tool handles.
    pub const SET_OSD_CANVAS: u16 = 188;
}

/// Decoded MSP_PACALIBRATION response -- vtx_msp_push_calibration()'s
/// payload: [power_level, vref_mv(u16 LE), detector_mv(u16 LE)]. Sent by
/// the VTX after every SET_PACALIBRATION it processes (see
/// vtx_msp_set_calibration() in vtx_msp.c), whether that was a real mV
/// override or just a value=0 telemetry poll.
#[derive(Debug, Clone, Copy, Default)]
pub struct PaCalibrationReading {
    pub power_level: u8,
    pub vref_mv: u16,
    pub detector_mv: u16,
    /// Only present when the VTX firmware sends the extended (10-byte
    /// or later) payload -- an older firmware's original 5-byte reply
    /// still decodes the three fields above fine, with these left None
    /// rather than defaulted to something that would look like real
    /// data.
    pub boost_on: Option<bool>,
    pub rtc6705_level: Option<u8>,
    pub pid_active: Option<bool>,
    pub frequency_mhz: Option<u16>,
    /// Only present with the 11-byte payload -- see
    /// rf_pa_calibration_session_begin()'s doc comment in the firmware's
    /// rf_pa.h. Set by the trailing session_active field on this tool's
    /// own SET_PACALIBRATION requests (see encode_pa_calibration_request).
    pub session_active: Option<bool>,
    /// Only present with the 15-byte payload. Raw 12-bit ADC code from
    /// the PA's NTC thermistor, and the firmware's own conversion to
    /// degrees C from it (see rf_pa_ntc_raw_to_celsius()'s doc comment
    /// in the firmware's rf_pa.c for the assumed circuit and math).
    /// Both are 0 on a board with no NTC configured -- not
    /// distinguishable here from a board that genuinely reported a 0
    /// code, but that's the same convention the firmware's own reply
    /// already uses elsewhere for "nothing to report".
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

/// Encodes a SET_PACALIBRATION request: select `power_level` (switches
/// the VTX's active level if different from its current one), optionally
/// override the DAC directly, and carry the tool's current calibration-
/// session/PA-boost state -- see vtx_msp_set_calibration()'s doc comment
/// in vtx_msp.c for the full 5-byte payload layout. `mv = None` sends
/// value=0, which doesn't touch the DAC (matches rf_calibration.py's
/// send_MSP_SET_PACALIBRATION(0) pattern). `session_active` and
/// `boost_mode` (0=off, 1=on, 2=auto/ext_pa_enable-driven) are sent on
/// EVERY request now, not just a one-off "begin/end" message -- there is
/// no separate command for either; the firmware only acts when a value
/// actually differs from its current state, so repeating the same value
/// on every calibration point is a no-op there, not something that needs
/// avoiding on this end.
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

/// DisplayPort sub-command bytes -- see msp_displayport.c's
/// msp_displayport_cmd_t. Only the ones this tool actually sends are
/// listed (CLEAR/SET_OPTIONS/DRAW_SYSTEM are firmware-side concerns not
/// used here).
pub mod displayport_cmd {
    pub const KEEPALIVE: u8 = 0;
    pub const RELEASE: u8 = 1;
    pub const CLEAR: u8 = 2;
    pub const DRAW_STRING: u8 = 3;
    pub const DRAW_SCREEN: u8 = 4;
}

/// The firmware's own OSD canvas dimensions -- see canvas_char.h's
/// COLUMN_SIZE/ROW_SIZE. encode_displayport_draw_string() clamps against
/// this itself (belt-and-suspenders alongside the firmware's own clamp
/// in msp_displayport_handle_msp() -- neither should be the only thing
/// standing between a too-long string and a buffer overflow).
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

/// Encodes DRAW_STRING for one row starting at `col`. `text` is clamped
/// to DISPLAYPORT_COLUMNS - col bytes -- matching, but not relying on,
/// the firmware's own clamp for the same reason. byte[3] is an unused
/// "attribute" byte the firmware's own offset math expects present
/// (text starts at payload[4]) but never reads.
pub fn encode_displayport_draw_string(row: u8, col: u8, text: &str) -> Vec<u8> {
    let max_len = DISPLAYPORT_COLUMNS.saturating_sub(col) as usize;
    let bytes = text.as_bytes();
    let bytes = if bytes.len() > max_len { &bytes[..max_len] } else { bytes };
    let mut v = vec![displayport_cmd::DRAW_STRING, row, col, 0];
    v.extend_from_slice(bytes);
    v
}

/// Decodes MSP_SET_OSD_CANVAS's 2-byte payload: [columns, rows] -- see
/// msp_displayport_handle_msp()'s KEEPALIVE handling in the firmware,
/// which is what actually sends this (as a reply, not something this
/// tool requests separately).
pub fn decode_osd_canvas(payload: &[u8]) -> Option<(u8, u8)> {
    if payload.len() < 2 {
        return None;
    }
    Some((payload[0], payload[1]))
}

/// One decoded PA calibration table entry, matching cMsp.py's
/// PaCalibration (idx, mW, 7 calibration mV points, 7 detector points).
#[derive(Debug, Clone, Default)]
pub struct PaCalibration {
    pub idx: u8,
    pub m_w: u16,
    pub value: [u16; 7],
    pub detector: [u16; 7],
    /// Real levels (idx >= 1): whether this level engages the external
    /// boost PA stage. Display-only -- not editable from this tool.
    pub ext_pa_enable: bool,
    /// Real levels (idx >= 1): the raw RTC6705 register value for this
    /// level. Display-only.
    pub rtc6705_level: u8,
    /// Only meaningful when idx == 0 -- that entry has no real
    /// ext_pa_enable, so vtx_msp.c repurposes that byte to carry the
    /// board's DAC polarity instead: true if PA_DAC_SIGN > 0 (inverted,
    /// e.g. RTC76401: lower DAC mV means MORE RF output), false if
    /// PA_DAC_SIGN < 0 (normal/typical: higher DAC mV means more
    /// output). The calibration sweep reads this off idx 0's entry to
    /// know which direction to step the DAC.
    pub dac_sign_inverted: bool,
}

/// Decoded MSP_VTX_CONFIG response, matching vtx_msp.c's
/// vtx_msp_push_vtx_config() payload layout exactly (15 bytes).
#[derive(Debug, Clone, Default)]
pub struct VtxConfig {
    pub vtx_type: u8, // 5 = VTXDEV_MSP
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

/// Decoded VTX band entry, matching vtx_msp_push_band_table()'s
/// SET_VTXTABLE_BAND payload layout (29 bytes): index, 8-byte name,
/// letter, factory flag, channel count, 8x u16 LE frequencies (MHz).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct VtxBand {
    pub index: u8,
    pub name: String, // up to 8 chars
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

/// Encodes a VtxBand into a SET_VTXTABLE_BAND payload (29 bytes) --
/// symmetric with decode_vtx_band, matches vtx_msp_push_band_table()'s
/// layout so the VTX-side handler (once it accepts incoming SETs -- see
/// the note in vtx_msp.c) can parse it the same way.
pub fn encode_vtx_band(band: &VtxBand) -> Vec<u8> {
    let mut p = vec![0u8; 29];
    p[0] = band.index;
    p[1] = 8; // name field is a fixed 8 bytes on the wire
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

/// Decoded VTX power level entry, matching vtx_msp_push_power_table()'s
/// SET_VTXTABLE_POWERLEVEL payload layout: index, mW (u16 LE), label_len,
/// ASCII label.
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

/// Encodes a VtxPowerLevel into a SET_VTXTABLE_POWERLEVEL payload --
/// symmetric with decode_vtx_power_level.
pub fn encode_vtx_power_level(pl: &VtxPowerLevel) -> Vec<u8> {
    let label_bytes = pl.label.as_bytes();
    let label_len = label_bytes.len().min(255) as u8;
    let mut p = vec![pl.index, (pl.m_w & 0xff) as u8, (pl.m_w >> 8) as u8, label_len];
    p.extend_from_slice(&label_bytes[..label_len as usize]);
    p
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
/// Classifies an MSP send by how long the link should refuse further
/// sends afterward -- see MspLink::note_sent(), and send()'s own
/// enforcement of the resulting block. Putting this enforcement on the
/// link itself (rather than as bookkeeping some higher-level caller is
/// expected to check) makes it structurally impossible for ANY sender,
/// present or future, to bypass a settle requirement by mistake -- which
/// is exactly how a real bug happened: code outside the sweep engine
/// sent directly, with no way for anything to know it needed to wait.
#[derive(Debug, Clone, Copy)]
pub enum MspCommandKind {
    /// A VTX_CONFIG push where frequency, power, or pitmode actually
    /// changes -- triggers a synth retune on the VTX
    /// (rtc6705_set_frequency()), which per the firmware blocks the
    /// VTX's entire main loop until rtc6705_wait_state_stable() returns.
    /// That function's own intended worst case is
    /// RTC6705_LOCK_WAIT_TIMEOUT_US (5ms) -- previously the delay hook
    /// it's built on (rtc6705_hook_delay_us) resolved to millisecond-
    /// rather than microsecond-granularity (a ~100x overshoot per
    /// iteration of that function's own polling loop, since each
    /// iteration calls it with us=10 expecting ~10us but got ~1ms via
    /// HAL_Delay's own millisecond floor), making the REAL worst case
    /// closer to ~500ms -- during which no incoming MSP byte on either
    /// UART or USB was processed at all, since the whole MCU main loop
    /// was frozen inside the busy-wait. Now fixed firmware-side (see
    /// rtc6705_hook_delay_us() in rtc6705.c), so this settle window is
    /// back down to the intended ~5ms plus real margin, not the ~500ms
    /// this was previously padded against.
    Retune,
    /// SET_PACALIBRATION -- a direct DAC write (dac_ch2_write_mv()) with
    /// no synth reprogramming involved, so no comparable blocking is
    /// expected. The sweep's own SampleWait mechanism already waits for
    /// fresh readings before acting on any result, which provides
    /// whatever settle time the DAC/detector themselves need -- this
    /// isn't a substitute for that, just confirms no ADDITIONAL
    /// command-level wait is required on top of it.
    Calibration,
    /// Anything else (queries, session begin/end, table pushes, EEPROM
    /// writes) -- no known settle requirement.
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
    /// No send is allowed before this instant -- see note_sent() and
    /// send()'s own enforcement. Starts at "now" (unblocked) on open().
    can_send_at: Instant,
    /// Every frame actually written to the port via send() -- incremented
    /// regardless of MSP version/function, purely "did a write happen".
    /// See tx_rx_counts()/worker.rs's periodic log of these -- added
    /// specifically to make "are commands even reaching the VTX, are
    /// replies even coming back" answerable from the log instead of
    /// inferred indirectly from symptoms.
    tx_count: u64,
    /// Every frame read_frame() successfully parsed (checksum-valid,
    /// complete) and returned as Some(..) -- NOT incremented for
    /// timeouts or bytes discarded while scanning for the next '$'.
    rx_count: u64,
}

impl MspLink {
    pub fn open(path: &str, baud: u32) -> Result<Self> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(50))
            .open()?;
        Ok(Self { port, can_send_at: Instant::now(), tx_count: 0, rx_count: 0 })
    }

    /// Cheap, non-blocking check for whether a send would actually go
    /// out right now. Callers that want to avoid attempting a send they
    /// know will be refused (the normal, expected case for anything
    /// respecting the gate) should check this first -- but send()
    /// refuses regardless, so nothing can slip through by skipping the
    /// check.
    pub fn can_send_now(&self) -> bool {
        Instant::now() >= self.can_send_at
    }

    /// How much longer until can_send_now() would return true, or None
    /// if it already does.
    pub fn blocked_for(&self) -> Option<Duration> {
        let now = Instant::now();
        if now >= self.can_send_at {
            None
        } else {
            Some(self.can_send_at - now)
        }
    }

    /// Extends the send-block until at least now + kind's own settle
    /// duration. Call this right after any send that needs one. Only
    /// ever extends, never shortens -- a send needing a short (or no)
    /// settle shouldn't cut short a longer window a previous, more
    /// demanding send already established.
    pub fn note_sent(&mut self, kind: MspCommandKind) {
        let until = Instant::now() + kind.settle_duration();
        if until > self.can_send_at {
            self.can_send_at = until;
        }
    }

    /// Current (tx_count, rx_count) -- see their own doc comments on the
    /// struct. worker.rs logs these periodically as a basic MSP-level
    /// diagnostic: if tx keeps climbing but rx doesn't, commands aren't
    /// reaching the VTX or it isn't replying; if neither climbs, sends
    /// themselves aren't happening.
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
                self.rx_count += 1;
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
    // Trailing 2 bytes are a newer addition -- tolerate a bare 31-byte
    // payload (older firmware) by leaving these at their Default (false/0).
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

/// Encodes a PaCalibration entry into a SET_PACALTABLE payload (same
/// layout as decode_pa_calibration expects). Only used to push
/// calibration[]/detector[] edits back -- ext_pa_enable/rtc6705_level/
/// dac_sign_inverted are display-only and not meaningful to send, so
/// this only encodes the original 31-byte shape.
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
