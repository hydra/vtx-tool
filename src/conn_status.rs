//! Shared connection-status concept and its rendering helper. Used by
//! the left panel (VTX/meter status next to their port fields) and the
//! calibration page's VTX-unresponsive dialog, so both draw the same
//! Ready/Disconnected concept the same way instead of each having their
//! own copy of the enum/color/label logic.

use eframe::egui;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnStatus {
    Disconnected,
    Ready,
}

impl ConnStatus {
    pub fn from_ready(ready: bool) -> Self {
        if ready {
            ConnStatus::Ready
        } else {
            ConnStatus::Disconnected
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            ConnStatus::Disconnected => "Disconnected",
            ConnStatus::Ready => "Ready",
        }
    }

    pub fn color(self) -> egui::Color32 {
        match self {
            ConnStatus::Disconnected => egui::Color32::from_rgb(220, 80, 80),
            ConnStatus::Ready => egui::Color32::from_rgb(80, 200, 120),
        }
    }
}

/// Renders `status` as a colored label. `None` renders nothing -- used
/// for "not attempted yet" (e.g. before Connect has ever been pressed),
/// as distinct from a known Disconnected state.
pub fn show(ui: &mut egui::Ui, status: Option<ConnStatus>) {
    if let Some(status) = status {
        ui.colored_label(status.color(), status.label());
    }
}
