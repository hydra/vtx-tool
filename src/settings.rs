//! Persists last-used COM ports across runs. Deliberately dependency-free
//! (no `directories`-style crate for a proper OS config dir, since I
//! have no way to verify a working version number for one without
//! network access) -- settings live in a JSON file next to wherever the
//! tool is run from. Fine for now; worth swapping for a real config-dir
//! crate later if that becomes annoying.

use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

const SETTINGS_FILE: &str = "rf-cal-settings.json";

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AppSettings {
    pub vtx_port: String,
    pub meter_port: String,
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
