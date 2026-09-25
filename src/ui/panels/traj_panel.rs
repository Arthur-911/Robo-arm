use egui::{DragValue, Grid, Slider, Ui};
use nalgebra::Point3;

use crate::trajectory::{PolynomialType, TrajectoryContinuityMode, TrajectoryController};

/// Renders the Trajectory Planning and Waypoints Sequencer panel.
pub fn render_trajectory_panel(
    ui: &mut Ui,
    controller: &mut TrajectoryController,
    current_target_pos: Point3<f64>,
    on_export_requested: &mut Option<&'static str>,
) {
    ui.heading("📈 Trajectory Planning");
    ui.separator();

    // Interpolation selector
    ui.group(|ui| {
        ui.strong("Interpolation Algorithm:");
        ui.horizontal(|ui| {
            if ui
                .radio(
                    controller.planner.poly_type == PolynomialType::Quintic,
                    "Quintic (Minimum Jerk)",
                )
                .clicked()
            {
                controller
                    .planner
                    .set_polynomial_type(PolynomialType::Quintic);
            }
            if ui
                .radio(
                    controller.planner.poly_type == PolynomialType::Cubic,
                    "Cubic Spline",
                )
                .clicked()
            {
                controller
                    .planner
                    .set_polynomial_type(PolynomialType::Cubic);
            }
        });

        ui.add_space(3.0);
        ui.strong("Waypoint Motion Profile:");
        ui.horizontal(|ui| {
            if ui
                .radio(
                    controller.planner.continuity_mode == TrajectoryContinuityMode::SmoothContinuous,
                    "Continuous Smooth (C1/C2)",
                )
                .clicked()
            {
                controller
                    .planner
                    .set_continuity_mode(TrajectoryContinuityMode::SmoothContinuous);
            }
            if ui
                .radio(
                    controller.planner.continuity_mode == TrajectoryContinuityMode::StopAtWaypoints,
                    "Stop at Waypoints",
                )
                .clicked()
            {
                controller
                    .planner
                    .set_continuity_mode(TrajectoryContinuityMode::StopAtWaypoints);
            }
        });
        ui.checkbox(&mut controller.enable_smoothing, "🛡️ Joint Velocity Limiter & Filter");
    });

    ui.add_space(6.0);

    // Playback Controls
    ui.group(|ui| {
        ui.strong("Playback Controller:");
        ui.horizontal(|ui| {
            let play_label = if controller.is_playing {
                "⏸ Pause"
            } else {
                "▶ Play"
            };
            if ui.button(play_label).clicked() {
                controller.toggle_play();
            }
            if ui.button("⏹ Stop / Reset").clicked() {
                controller.reset();
            }
            ui.checkbox(&mut controller.is_looping, "Loop");
        });

        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label("Speed:");
            ui.add(Slider::new(&mut controller.speed_multiplier, 0.2..=4.0).suffix("x"));
        });

        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label("Timeline:");
            let total = controller.planner.total_duration;
            let mut t = controller.current_time;
            if ui
                .add(Slider::new(&mut t, 0.0..=total.max(0.1)).suffix(" s"))
                .changed()
            {
                controller.seek(t);
            }
        });
    });

    ui.add_space(6.0);

    // Waypoints Sequencer
    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.strong(format!(
                "Waypoints ({})",
                controller.planner.waypoints.len()
            ));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("🗑 Clear All").clicked() {
                    controller.planner.clear_waypoints();
                }
            });
        });

        ui.add_space(4.0);
        if ui.button("➕ Add Current Target as Waypoint").clicked() {
            let count = controller.planner.waypoints.len() + 1;
            controller
                .planner
                .add_waypoint(format!("WP {}", count), current_target_pos, 2.0);
        }

        ui.add_space(6.0);
        let mut remove_idx = None;

        egui::ScrollArea::vertical()
            .max_height(160.0)
            .show(ui, |ui| {
                Grid::new("waypoints_table")
                    .num_columns(4)
                    .spacing([8.0, 4.0])
                    .striped(true)
                    .show(ui, |ui| {
                        ui.strong("#");
                        ui.strong("Position (X, Y, Z)");
                        ui.strong("Time (s)");
                        ui.strong("Action");
                        ui.end_row();

                        for (i, wp) in controller.planner.waypoints.iter_mut().enumerate() {
                            ui.label(format!("{}.", i + 1));
                            ui.monospace(format!(
                                "({:.2}, {:.2}, {:.2})",
                                wp.position.x, wp.position.y, wp.position.z
                            ));
                            if ui
                                .add(
                                    DragValue::new(&mut wp.duration)
                                        .speed(0.1)
                                        .range(0.1..=20.0)
                                        .suffix("s"),
                                )
                                .changed()
                            {
                                // duration changed, will be rebuilt
                            }
                            if ui.button("❌").clicked() {
                                remove_idx = Some(i);
                            }
                            ui.end_row();
                        }
                    });
            });

        if let Some(idx) = remove_idx {
            controller.planner.remove_waypoint(idx);
        }
    });

    ui.add_space(6.0);
    ui.horizontal(|ui| {
        if ui.button("🧹 Clear Motion Trail").clicked() {
            controller.clear_trail();
        }
    });

    ui.add_space(8.0);
    ui.group(|ui| {
        ui.strong("📤 Export Motion Script:");
        ui.horizontal_wrapped(|ui| {
            if ui.button("🐍 Python (NumPy)").clicked() {
                *on_export_requested = Some("python");
            }
            if ui.button("🤖 ROS 2 (MoveIt)").clicked() {
                *on_export_requested = Some("ros2");
            }
            if ui.button("⚙️ G-Code / CSV").clicked() {
                *on_export_requested = Some("gcode");
            }
        });
    });
}
