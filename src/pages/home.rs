use eframe::egui;

pub fn show(ui: &mut egui::Ui) {
    let size_id = ui.id().with("home_branding_size");
    let available = ui.available_rect_before_wrap();
    let last_size: egui::Vec2 = ui.data(|d| d.get_temp(size_id)).unwrap_or(available.size());
    let target_rect = egui::Rect::from_center_size(available.center(), last_size);

    let content_size = ui
        .scope_builder(egui::UiBuilder::new().max_rect(target_rect), |ui| {
            crate::app::show_branding(ui);
            ui.min_rect().size()
        })
        .inner;

    if (content_size - last_size).length() > 0.5 {
        ui.data_mut(|d| d.insert_temp(size_id, content_size));
        ui.ctx().request_repaint();
    }
}
