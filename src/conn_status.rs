//! Shared connection-status concepts. Two distinct things live here,
//! deliberately kept separate rather than merged into one:
//!
//! - `PortState`: the actual connection LIFECYCLE for a serial port
//!   (Disconnected -> Connecting -> Ready -> Disconnecting -> ...),
//!   driven by explicit Connect/Disconnect actions and hard I/O errors
//!   (port physically gone). This is what the left panel shows next to
//!   each port and what gates the Connect-all/Disconnect-all button and
//!   whether calibration is allowed to start.
//! - `ConnStatus`: a narrower Ready/Disconnected HEARTBEAT signal, used
//!   only by the calibration sweep's "VTX not responding" dialog (a
//!   different question -- "is it answering right now", not "did we
//!   open the port" -- a port can be PortState::Ready while transiently
//!   failing this, e.g. mid-power-trip during a sweep).
//!
//! Deliberately not merged: PortState shouldn't flicker on an ordinary
//! momentary heartbeat gap (that would be noisy/annoying for a status
//! that's meant to answer "is the port open"), while the sweep's
//! power-trip detection specifically NEEDS to react to exactly that kind
//! of gap, fast.

use eframe::egui;

/// A serial port's connection lifecycle. Every port (VTX, power meter)
/// tracks one of these independently -- see worker.rs's per-port
/// Connect/Disconnect command handlers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortState {
    Disconnected,
    Connecting,
    Ready,
    Disconnecting,
}

impl PortState {
    pub fn label(self) -> &'static str {
        match self {
            PortState::Disconnected => "Disconnected",
            PortState::Connecting => "Connecting",
            PortState::Ready => "Ready",
            PortState::Disconnecting => "Disconnecting",
        }
    }

    pub fn color(self) -> egui::Color32 {
        match self {
            PortState::Disconnected => egui::Color32::from_rgb(180, 70, 70),
            PortState::Connecting | PortState::Disconnecting => egui::Color32::from_rgb(200, 160, 60),
            PortState::Ready => egui::Color32::from_rgb(70, 170, 100),
        }
    }

    pub fn is_ready(self) -> bool {
        matches!(self, PortState::Ready)
    }

    /// True only when fully at rest, disconnected -- the gate for
    /// whether the port's text field is editable and whether a Connect
    /// command should actually attempt to open it.
    pub fn is_idle(self) -> bool {
        matches!(self, PortState::Disconnected)
    }
}

impl Default for PortState {
    fn default() -> Self {
        PortState::Disconnected
    }
}

pub fn show_port(ui: &mut egui::Ui, state: PortState) {
    ui.colored_label(state.color(), state.label());
}

/// Whether BOTH ports are Ready -- the gate for allowing calibration to
/// start, and what the Connect-all/Disconnect-all button label reflects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverallState {
    Ready,
    NotReady,
}

impl OverallState {
    pub fn from_ports(vtx: PortState, meter: PortState) -> Self {
        if vtx.is_ready() && meter.is_ready() {
            OverallState::Ready
        } else {
            OverallState::NotReady
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            OverallState::Ready => "Ready",
            OverallState::NotReady => "Not Ready",
        }
    }

    pub fn color(self) -> egui::Color32 {
        match self {
            OverallState::Ready => egui::Color32::from_rgb(70, 170, 100),
            OverallState::NotReady => egui::Color32::from_rgb(180, 70, 70),
        }
    }
}

pub fn show_overall(ui: &mut egui::Ui, state: OverallState) {
    ui.colored_label(state.color(), state.label());
}

/// The sweep-specific heartbeat signal -- see the module doc for why
/// this is deliberately separate from PortState.
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
