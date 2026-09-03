use eframe::egui;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortState {
    Disconnected,
    Connecting,
    Ready,
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
            PortState::Connecting | PortState::Disconnecting => {
                egui::Color32::from_rgb(200, 160, 60)
            }
            PortState::Ready => egui::Color32::from_rgb(70, 170, 100),
            PortState::LostCommunication => egui::Color32::from_rgb(200, 90, 40),
        }
    }

    pub fn is_ready(self) -> bool {
        matches!(self, PortState::Ready)
    }

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
