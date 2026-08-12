//! The tool's own local VTX table -- this is what the app hands back to
//! the VTX when it asks "what's my config?" (an empty MSP_VTX_CONFIG
//! request), playing the same role Betaflight's own vtxTableConfig()
//! plays for a real FC (see the PR #11705 quote this whole design is
//! based on: "At boot up Betaflight will wait for MSP_VTX_CONFIG
//! request to then send the current band/channel/power settings to the
//! VTX that are stored in Betaflight EEPROM").
//!
//! Three ways to populate it: hand-edit in the UI, load a previously
//! saved JSON file, or import a Betaflight CLI `vtxtable` dump (paste
//! the output of Betaflight's `diff` / `dump` CLI command, or just the
//! vtxtable lines).

use crate::msp::{VtxBand, VtxPowerLevel};
use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VtxTableConfig {
    pub channels: u8,
    pub bands: Vec<VtxBand>,
    pub power_levels: Vec<VtxPowerLevel>,

    /// What this tool currently reports as "selected" when the VTX asks
    /// -- the direct analogue of Betaflight's own currently-armed
    /// band/channel/power/frequency state. Edited in the "Selected"
    /// section of the VTX Table page.
    pub selected_band: u8,      // 1-based; 0 = frequency mode (use selected_freq_mhz directly)
    pub selected_channel: u8,   // 1-based
    pub selected_power: u8,     // 1-based, index into power_levels
    pub selected_freq_mhz: u16, // used when selected_band == 0
    pub pitmode: bool,
}

impl Default for VtxTableConfig {
    fn default() -> Self {
        Self {
            channels: 8,
            bands: Vec::new(),
            power_levels: Vec::new(),
            selected_band: 1,
            selected_channel: 1,
            selected_power: 1,
            selected_freq_mhz: 5800,
            pitmode: false,
        }
    }
}

impl VtxTableConfig {
    pub fn save_to_file(&self, path: &Path) -> Result<()> {
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(path, json)?;
        Ok(())
    }

    pub fn load_from_file(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)?;
        Ok(serde_json::from_str(&text)?)
    }

    /// Resolves selected_band/selected_channel/selected_freq_mhz into an
    /// actual frequency (MHz) -- band=0 means "use selected_freq_mhz
    /// directly", matching MSP_VTX_CONFIG's own band==0 convention.
    pub fn selected_frequency_mhz(&self) -> u16 {
        if self.selected_band == 0 {
            return self.selected_freq_mhz;
        }
        self.bands
            .iter()
            .find(|b| b.index == self.selected_band)
            .and_then(|b| b.freqs_mhz.get((self.selected_channel as usize).saturating_sub(1)))
            .copied()
            .unwrap_or(self.selected_freq_mhz)
    }

    /// Encodes the current selection as an MSP_VTX_CONFIG response
    /// payload (15 bytes) -- same layout as vtx_msp.c's own
    /// vtx_msp_push_vtx_config(), since it's the same message type
    /// regardless of which side initiated the exchange.
    pub fn encode_vtx_config_response(&self) -> Vec<u8> {
        let freq = self.selected_frequency_mhz();
        let mut p = vec![0u8; 15];
        p[0] = 5; // VTXDEV_MSP
        p[1] = self.selected_band;
        p[2] = self.selected_channel;
        p[3] = self.selected_power;
        p[4] = self.pitmode as u8;
        p[5] = (freq & 0xff) as u8;
        p[6] = (freq >> 8) as u8;
        p[7] = 1; // device_ready
        p[8] = 0; // low_power_disarm
        p[9] = 0;
        p[10] = 0; // pit_mode_freq
        p[11] = 1; // vtx_table_available
        p[12] = self.bands.len() as u8;
        p[13] = self.channels;
        p[14] = self.power_levels.len() as u8;
        p
    }

    /// Parses Betaflight CLI `vtxtable` lines (e.g. from `diff` or
    /// `dump`, or just the vtxtable block itself -- any surrounding
    /// text/other CLI commands are ignored). Format, per Betaflight's
    /// own CLI:
    ///   vtxtable bands <N>
    ///   vtxtable channels <N>
    ///   vtxtable band <idx> <NAME> <LETTER> <FACTORY|CUSTOM> <freq1> .. <freqN>
    ///   vtxtable powerlevels <N>
    ///   vtxtable powervalues <v1> <v2> ..
    ///   vtxtable powerlabels <l1> <l2> ..
    /// `bands`/`powerlevels` counts are informational here -- the real
    /// counts come from how many `band` lines and powervalues/powerlabels
    /// entries actually got parsed.
    pub fn parse_betaflight_cli(text: &str) -> Result<Self> {
        let mut cfg = VtxTableConfig::default();
        let mut power_values: Vec<u16> = Vec::new();
        let mut power_labels: Vec<String> = Vec::new();

        for line in text.lines() {
            let line = line.trim();
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 2 || parts[0] != "vtxtable" {
                continue;
            }

            match parts[1] {
                "channels" => {
                    if let Some(n) = parts.get(2).and_then(|s| s.parse::<u8>().ok()) {
                        cfg.channels = n;
                    }
                }
                "band" => {
                    // vtxtable band <idx> <NAME> <LETTER> <FACTORY|CUSTOM> <freqs...>
                    if parts.len() < 6 {
                        continue;
                    }
                    let index: u8 = parts[2].parse().unwrap_or(0);
                    let name = parts[3].to_string();
                    let letter = parts[4].chars().next().unwrap_or('?');
                    let is_factory = parts[5].eq_ignore_ascii_case("FACTORY");
                    let freq_strs = &parts[6..];
                    let mut freqs_mhz = [0u16; 8];
                    for (i, f) in freq_strs.iter().take(8).enumerate() {
                        freqs_mhz[i] = f.parse().unwrap_or(0);
                    }
                    let channel_count = freq_strs.len().min(8) as u8;
                    cfg.bands.push(VtxBand {
                        index,
                        name,
                        letter,
                        is_factory,
                        channel_count,
                        freqs_mhz,
                    });
                }
                "powervalues" => {
                    power_values = parts[2..].iter().filter_map(|s| s.parse().ok()).collect();
                }
                "powerlabels" => {
                    power_labels = parts[2..].iter().map(|s| s.to_string()).collect();
                }
                // "bands" / "powerlevels" are just count hints -- derived
                // from the actual parsed data below instead, so skipped.
                _ => {}
            }
        }

        cfg.bands.sort_by_key(|b| b.index);

        cfg.power_levels = power_values
            .iter()
            .enumerate()
            .map(|(i, &value)| VtxPowerLevel {
                index: i as u8 + 1,
                m_w: value,
                label: power_labels.get(i).cloned().unwrap_or_else(|| value.to_string()),
            })
            .collect();

        if cfg.bands.is_empty() && cfg.power_levels.is_empty() {
            anyhow::bail!("no 'vtxtable band'/'vtxtable powervalues' lines found in the pasted text");
        }

        Ok(cfg)
    }
}
