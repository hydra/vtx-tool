//! Persists last-used COM ports, meter kind, and attenuation across runs.
//! Deliberately dependency-free (no `directories`-style crate for a
//! proper OS config dir, since I have no way to verify a working version
//! number for one without network access) -- settings live in a JSON
//! file next to wherever the tool is run from. Fine for now; worth
//! swapping for a real config-dir crate later if that becomes annoying.

use crate::power_meter::PowerMeterKind;
use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

const SETTINGS_FILE: &str = "rf-cal-settings.json";

/// Matches the calibration page's own default -- see CalibrationPageState.
fn default_attenuation_db() -> f32 {
    30.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppSettings {
    #[serde(default)]
    pub vtx_port: String,
    #[serde(default)]
    pub meter_port: String,
    /// #[serde(default)] here (rather than requiring the field) means a
    /// settings file saved before this field existed still parses
    /// correctly instead of silently discarding the ports it DOES have
    /// and falling back to Self::default() entirely.
    #[serde(default)]
    pub meter_kind: PowerMeterKind,
    /// Same reasoning as meter_kind, but with an explicit default
    /// function rather than the derived one -- f32's own Default is
    /// 0.0dB, not the 30dB the UI actually defaults to.
    #[serde(default = "default_attenuation_db")]
    pub attenuation_db: f32,
}

impl Default for AppSettings {
    fn default() -> Self {
        Self {
            vtx_port: String::new(),
            meter_port: String::new(),
            meter_kind: PowerMeterKind::default(),
            attenuation_db: default_attenuation_db(),
        }
    }
}

fn settings_path() -> PathBuf {
    PathBuf::from(SETTINGS_FILE)
}

impl AppSettings {
    pub fn load() -> Self {
        std::fs::read_to_string(settings_path())
            .ok()
            .and_then(|text| serde_json::from_str(&text).ok())
            .unwrap_or_default()
    }

    pub fn save(&self) -> Result<()> {
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(settings_path(), json)?;
        Ok(())
    }
}
