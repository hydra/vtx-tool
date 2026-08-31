
use eframe::egui;

pub fn show(ui: &mut egui::Ui) {
    ui.centered_and_justified(|ui| {
        ui.heading("RF Calibration Tool");
    });
}
