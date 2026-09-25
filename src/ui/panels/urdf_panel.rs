use crate::kinematics::RobotArm;
use crate::presets::{SAMPLE_URDF_INDUSTRIAL_6DOF, SAMPLE_URDF_PLANAR_3DOF};
use crate::urdf::{export_urdf, parse_urdf};
use egui::{Color32, TextEdit, Ui};

/// Renders the URDF XML Import, Editor, and Export panel.
pub fn render_urdf_panel(
    ui: &mut Ui,
    urdf_text: &mut String,
    parse_error: &mut Option<String>,
    on_robot_loaded: &mut Option<RobotArm>,
    current_robot: &RobotArm,
) {
    ui.heading("📄 URDF Import / Export");
    ui.separator();

    ui.horizontal(|ui| {
        ui.label("Load Sample URDF:");
        if ui.button("Planar 3-DOF XML").clicked() {
            *urdf_text = SAMPLE_URDF_PLANAR_3DOF.to_string();
            *parse_error = None;
        }
        if ui.button("Industrial 6-DOF XML").clicked() {
            *urdf_text = SAMPLE_URDF_INDUSTRIAL_6DOF.to_string();
            *parse_error = None;
        }
    });

    ui.add_space(6.0);
    ui.horizontal(|ui| {
        if ui.button("📥 Parse & Build Robot").clicked() {
            match parse_urdf(urdf_text) {
                Ok(new_arm) => {
                    *on_robot_loaded = Some(new_arm);
                    *parse_error = None;
                }
                Err(err) => {
                    *parse_error = Some(err);
                }
            }
        }
        if ui.button("📤 Export Active Robot to URDF").clicked() {
            *urdf_text = export_urdf(current_robot);
            *parse_error = None;
        }
    });

    if let Some(err) = parse_error {
        ui.add_space(4.0);
        ui.colored_label(Color32::from_rgb(240, 80, 80), format!("❌ {}", err));
    }

    ui.add_space(6.0);
    ui.label("URDF XML Document:");
    egui::ScrollArea::vertical()
        .max_height(260.0)
        .show(ui, |ui| {
            ui.add(
                TextEdit::multiline(urdf_text)
                    .code_editor()
                    .desired_rows(14)
                    .desired_width(f32::INFINITY),
            );
        });
}
