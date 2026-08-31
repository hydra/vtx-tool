
use crate::power_meter::PowerMeterKind;
use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

const SETTINGS_FILE: &str = "rf-cal-settings.json";

fn default_attenuation_db() -> f32 {
    30.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppSettings {
    #[serde(default)]
    pub vtx_port: String,
    #[serde(default)]
    pub meter_port: String,
    #[serde(default)]
    pub meter_kind: PowerMeterKind,
    #[serde(default = "default_attenuation_db")]
    pub attenuation_db: f32,
    #[serde(default)]
    pub vtx_table_path: String,
}

impl Default for AppSettings {
    fn default() -> Self {
        Self {
            vtx_port: String::new(),
            meter_port: String::new(),
            meter_kind: PowerMeterKind::default(),
            attenuation_db: default_attenuation_db(),
            vtx_table_path: String::new(),
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
