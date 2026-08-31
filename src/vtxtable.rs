
use crate::msp::{VtxBand, VtxPowerLevel};
use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VtxTableConfig {
    pub channels: u8,
    pub bands: Vec<VtxBand>,
    pub power_levels: Vec<VtxPowerLevel>,
}

impl Default for VtxTableConfig {
    fn default() -> Self {
        Self {
            channels: 8,
            bands: Vec::new(),
            power_levels: Vec::new(),
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct VtxSelectionState {
    pub selected_band: u8,
    pub selected_channel: u8,
    pub selected_power: u8,
    pub selected_freq_mhz: u16,
    pub pitmode: bool,
}

impl Default for VtxSelectionState {
    fn default() -> Self {
        Self {
            selected_band: 1,
            selected_channel: 1,
            selected_power: 1,
            selected_freq_mhz: 5800,
            pitmode: false,
        }
    }
}

impl VtxSelectionState {
    pub fn frequency_mhz(&self, table: &VtxTableConfig) -> u16 {
        if self.selected_band == 0 {
            return self.selected_freq_mhz;
        }
        table
            .bands
            .iter()
            .find(|b| b.index == self.selected_band)
            .and_then(|b| b.freqs_mhz.get((self.selected_channel as usize).saturating_sub(1)))
            .copied()
            .unwrap_or(self.selected_freq_mhz)
    }

    pub fn encode_vtx_config_response(&self, table: &VtxTableConfig) -> Vec<u8> {
        let freq = self.frequency_mhz(table);
        let mut p = vec![0u8; 15];
        p[0] = 5;
        p[1] = self.selected_band;
        p[2] = self.selected_channel;
        p[3] = self.selected_power;
        p[4] = self.pitmode as u8;
        p[5] = (freq & 0xff) as u8;
        p[6] = (freq >> 8) as u8;
        p[7] = 1;
        p[8] = 0;
        p[9] = 0;
        p[10] = 0;
        p[11] = 1;
        p[12] = table.bands.len() as u8;
        p[13] = table.channels;
        p[14] = table.power_levels.len() as u8;
        p
    }
}
