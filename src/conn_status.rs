//! Shared connection-status concept: `PortState`, a serial port's
//! connection lifecycle (Disconnected -> Connecting -> Ready ->
//! Disconnecting, plus LostCommunication for "was Ready, isn't now").
//! Every port (VTX, power meter) tracks one of these independently -- see
//! worker.rs's per-port Connect/Disconnect command handlers and its
//! hard-error/heartbeat-loss detection. This is what the left panel shows
//! next to each port, what the calibration sweep's "Connection error"
//! dialog shows for both ports, and what gates the Connect-all/
//! Disconnect-all button label and whether calibration is allowed to
//! start (via OverallState below).

use eframe::egui;

/// A serial port's connection lifecycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortState {
    Disconnected,
    Connecting,
    Ready,
    /// Was Ready, but communication has stopped -- either a hard I/O
    /// error (port physically gone; worker.rs releases the handle and
    /// periodically retries reopening it) or sustained silence with the
    /// handle still nominally open. Distinct from Disconnected: the
    /// latter means "never connected, or the user explicitly
    /// disconnected"; this means "was working, now isn't, without the
    /// user asking for that". Recovers to Ready automatically -- no user
    /// action needed, whether recovery means fresh traffic resuming on
    /// the same handle or a periodic reopen attempt succeeding.
    LostCommunication,
    Disconnecting,
}

impl PortState {
    pub fn label(self) -> &'static str {
        match self {
            PortState::Disconnected => "Disconnected",
            PortState::Connecting => "Connecting",
            PortState::Ready => "Ready",
            PortState::LostCommunication => "Lost Communication",
            PortState::Disconnecting => "Disconnecting",
        }
    }

    pub fn color(self) -> egui::Color32 {
        match self {
            PortState::Disconnected => egui::Color32::from_rgb(180, 70, 70),
            PortState::Connecting | PortState::Disconnecting => egui::Color32::from_rgb(200, 160, 60),
            PortState::Ready => egui::Color32::from_rgb(70, 170, 100),
            PortState::LostCommunication => egui::Color32::from_rgb(200, 90, 40),
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
