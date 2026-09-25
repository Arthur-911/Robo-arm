use egui::{Color32, Grid, Slider, Ui};
use nalgebra::{Point3, Vector3};

use crate::kinematics::CollisionReport;
use crate::workcell::{EOATType, ToolState, WorkcellEnvironment, Workpiece, WorkpieceManager};

/// Renders the Workcell Manipulation, EOAT Tool Changer, and Pick-and-Place panel.
pub fn render_manipulation_panel(
    ui: &mut Ui,
    tool_state: &mut ToolState,
    workpieces: &mut WorkpieceManager,
    environment: &mut WorkcellEnvironment,
    collision_report: &CollisionReport,
    target_pos: &mut Point3<f64>,
    on_solve_requested: &mut bool,
) {
    ui.heading("🖐️ Manipulation & Workcell");
    ui.separator();

    // 0. Direct Hand Movement Controls
    ui.group(|ui| {
        ui.strong("🕹️ Move Robot Hand (Direct Jog):");
        ui.horizontal(|ui| {
            if ui.button("◀ -X").clicked() {
                target_pos.x -= 0.03;
                *on_solve_requested = true;
            }
            if ui.button("+X ▶").clicked() {
                target_pos.x += 0.03;
                *on_solve_requested = true;
            }
            if ui.button("▲ +Y").clicked() {
                target_pos.y += 0.03;
                *on_solve_requested = true;
            }
            if ui.button("▼ -Y").clicked() {
                target_pos.y -= 0.03;
                *on_solve_requested = true;
            }
            if ui.button("⏫ +Z (Up)").clicked() {
                target_pos.z += 0.03;
                *on_solve_requested = true;
            }
            if ui.button("⏬ -Z (Down)").clicked() {
                target_pos.z -= 0.03;
                *on_solve_requested = true;
            }
        });

        ui.add_space(3.0);
        ui.horizontal_wrapped(|ui| {
            if ui.button("🎯 Hand to Table Center").clicked() {
                *target_pos = Point3::new(0.42, 0.0, 0.12);
                *on_solve_requested = true;
            }
            for wp in &workpieces.workpieces {
                if ui.button(format!("📍 Hand to {}", wp.name)).clicked() {
                    *target_pos = Point3::new(wp.position.x, wp.position.y, wp.position.z + 0.06);
                    *on_solve_requested = true;
                }
            }
        });
        ui.small("💡 Or left-click & drag the hand directly in 3D, or use keyboard arrow keys.");
    });

    ui.add_space(6.0);

    // 1. End-Of-Arm Tooling (EOAT) Selection
    ui.group(|ui| {
        ui.strong("End-Of-Arm Tool (EOAT):");
        ui.horizontal(|ui| {
            ui.selectable_value(
                &mut tool_state.tool_type,
                EOATType::ParallelGripper,
                "🖐️ Parallel Gripper",
            );
            ui.selectable_value(
                &mut tool_state.tool_type,
                EOATType::VacuumCup,
                "🧲 Vacuum Cup",
            );
            ui.selectable_value(
                &mut tool_state.tool_type,
                EOATType::WeldingTorch,
                "🔥 Welding Torch",
            );
        });

        ui.add_space(4.0);

        match tool_state.tool_type {
            EOATType::ParallelGripper => {
                ui.horizontal(|ui| {
                    ui.label("Jaw Opening:");
                    ui.add(
                        Slider::new(&mut tool_state.gripper_opening, 0.0..=1.0).show_value(false),
                    );
                    let pct = (tool_state.gripper_opening * 100.0) as u32;
                    ui.label(format!("{}%", pct));
                });

                ui.horizontal(|ui| {
                    let is_open = tool_state.gripper_opening > 0.4;
                    let btn_text = if is_open {
                        "✊ Close Gripper (G)"
                    } else {
                        "✋ Open Gripper (G)"
                    };
                    if ui.button(btn_text).clicked() {
                        tool_state.toggle_gripper();
                    }
                });
            }
            EOATType::VacuumCup => {
                let status_text = if tool_state.is_vacuum_active {
                    "🟢 Suction Active (V)"
                } else {
                    "⚪ Suction Off (V)"
                };
                if ui.button(status_text).clicked() {
                    tool_state.toggle_vacuum();
                }
            }
            EOATType::WeldingTorch => {
                let status_text = if tool_state.is_welding_active {
                    "⚡ Electric Arc Active (W)"
                } else {
                    "⚪ Arc Off (W)"
                };
                if ui.button(status_text).clicked() {
                    tool_state.toggle_welding();
                }
            }
        }
    });

    ui.add_space(8.0);

    // 2. Interactive Workpieces & Pick-and-Place
    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.strong(format!("Workpieces ({})", workpieces.workpieces.len()));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("🔄 Reset Demo").clicked() {
                    workpieces.reset_demo_workpieces();
                }
            });
        });

        ui.add_space(4.0);
        ui.horizontal_wrapped(|ui| {
            if ui.button("➕ Red Cube").clicked() {
                let id = workpieces.next_id;
                workpieces.next_id += 1;
                workpieces.workpieces.push(Workpiece::new_box(
                    id,
                    format!("Cube {}", id),
                    Point3::new(0.38, 0.15, 0.03),
                    Vector3::new(0.025, 0.025, 0.025),
                    Color32::from_rgb(235, 60, 50),
                ));
            }
            if ui.button("➕ Blue Billet").clicked() {
                let id = workpieces.next_id;
                workpieces.next_id += 1;
                workpieces.workpieces.push(Workpiece::new_cylinder(
                    id,
                    format!("Billet {}", id),
                    Point3::new(0.40, -0.15, 0.04),
                    0.025,
                    0.07,
                    Color32::from_rgb(45, 140, 240),
                ));
            }
            if ui.button("➕ Gold Sphere").clicked() {
                let id = workpieces.next_id;
                workpieces.next_id += 1;
                workpieces.workpieces.push(Workpiece::new_sphere(
                    id,
                    format!("Sphere {}", id),
                    Point3::new(0.35, -0.25, 0.025),
                    0.025,
                    Color32::from_rgb(240, 190, 40),
                ));
            }
        });

        ui.add_space(6.0);
        let mut remove_id = None;

        Grid::new("workpieces_grid")
            .num_columns(4)
            .spacing([8.0, 4.0])
            .striped(true)
            .show(ui, |ui| {
                ui.strong("Item");
                ui.strong("Position");
                ui.strong("Status");
                ui.strong("Action");
                ui.end_row();

                for wp in &workpieces.workpieces {
                    ui.label(&wp.name);
                    ui.monospace(format!(
                        "({:.2}, {:.2}, {:.2})",
                        wp.position.x, wp.position.y, wp.position.z
                    ));

                    if wp.is_grasped {
                        ui.colored_label(Color32::from_rgb(0, 230, 200), "✊ GRASPED");
                    } else {
                        ui.label("Table / Floor");
                    }

                    ui.horizontal(|ui| {
                        if ui.button("📍 Reach").clicked() {
                            *target_pos =
                                Point3::new(wp.position.x, wp.position.y, wp.position.z + 0.05);
                            *on_solve_requested = true;
                        }
                        if ui.button("❌").clicked() {
                            remove_id = Some(wp.id);
                        }
                    });
                    ui.end_row();
                }
            });

        if let Some(id) = remove_id {
            workpieces.workpieces.retain(|w| w.id != id);
            if workpieces.currently_held_id == Some(id) {
                workpieces.currently_held_id = None;
            }
        }
    });

    ui.add_space(8.0);

    // 3. Collision Monitor & Safety Status
    ui.group(|ui| {
        ui.strong("Collision Detection & Safety Status:");
        if collision_report.in_collision {
            ui.colored_label(
                Color32::from_rgb(255, 60, 60),
                "⚠️ COLLISION DETECTED (Links highlighted red)",
            );
            for detail in &collision_report.details {
                ui.label(format!("• {}", detail));
            }
        } else {
            ui.colored_label(
                Color32::from_rgb(50, 215, 80),
                "✔ All links clear — No collisions detected",
            );
        }
    });

    ui.add_space(8.0);

    // 4. Environment Furnishing Toggles
    ui.group(|ui| {
        ui.strong("Workcell Furnishings:");
        ui.checkbox(&mut environment.show_table, "Assembly Workstation Table");
        ui.checkbox(
            &mut environment.show_safety_enclosure,
            "Safety Perimeter Enclosure",
        );
        ui.checkbox(&mut environment.show_pedestal, "Industrial Ground Pedestal");
    });
}
