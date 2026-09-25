use egui::{Color32, DragValue, Grid, Slider, Ui};
use nalgebra::Point3;

use crate::kinematics::{IKSolution, IKSolverMode, IKSolverParams, IKSolverType};

/// Renders the Inverse Kinematics controls, target inputs, and solver metrics panel.
#[allow(clippy::too_many_arguments)]
pub fn render_ik_panel(
    ui: &mut Ui,
    target_pos: &mut Point3<f64>,
    target_rpy_deg: &mut [f64; 3],
    solver_type: &mut IKSolverType,
    solver_mode: &mut IKSolverMode,
    params: &mut IKSolverParams,
    continuous_solve: &mut bool,
    last_solution: &Option<IKSolution>,
    on_solve_requested: &mut bool,
    manip_data: Option<&crate::kinematics::ManipulabilityData>,
    show_ellipsoid: &mut bool,
    show_rotation_gizmo: &mut bool,
) {
    ui.heading("🎯 Inverse Kinematics");
    ui.separator();

    // Viewport Gizmo Toggles
    ui.horizontal(|ui| {
        ui.checkbox(show_rotation_gizmo, "🔄 3D Rotation Gizmo");
        ui.checkbox(show_ellipsoid, "🌐 Manipulability Ellipsoid");
    });

    ui.add_space(6.0);

    // Quick Hand Jog Buttons
    ui.group(|ui| {
        ui.strong("🕹️ Quick Hand Jogger:");
        ui.horizontal(|ui| {
            if ui.button("◀ -X").clicked() {
                target_pos.x -= 0.03;
                if *continuous_solve {
                    *on_solve_requested = true;
                }
            }
            if ui.button("+X ▶").clicked() {
                target_pos.x += 0.03;
                if *continuous_solve {
                    *on_solve_requested = true;
                }
            }
            if ui.button("▲ +Y").clicked() {
                target_pos.y += 0.03;
                if *continuous_solve {
                    *on_solve_requested = true;
                }
            }
            if ui.button("▼ -Y").clicked() {
                target_pos.y -= 0.03;
                if *continuous_solve {
                    *on_solve_requested = true;
                }
            }
            if ui.button("⏫ +Z").clicked() {
                target_pos.z += 0.03;
                if *continuous_solve {
                    *on_solve_requested = true;
                }
            }
            if ui.button("⏬ -Z").clicked() {
                target_pos.z -= 0.03;
                if *continuous_solve {
                    *on_solve_requested = true;
                }
            }
        });
        ui.small("💡 Use Keyboard Arrow Keys (and PageUp/PageDown) to jog the hand directly.");
    });

    ui.add_space(6.0);

    // Target Position Inputs
    ui.group(|ui| {
        ui.strong("Target Coordinates (Meters):");
        let mut pos_changed = false;
        Grid::new("target_xyz_grid")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("X Axis:");
                if ui
                    .add(DragValue::new(&mut target_pos.x).speed(0.01).suffix(" m"))
                    .changed()
                {
                    pos_changed = true;
                }
                ui.end_row();

                ui.label("Y Axis:");
                if ui
                    .add(DragValue::new(&mut target_pos.y).speed(0.01).suffix(" m"))
                    .changed()
                {
                    pos_changed = true;
                }
                ui.end_row();

                ui.label("Z Axis:");
                if ui
                    .add(DragValue::new(&mut target_pos.z).speed(0.01).suffix(" m"))
                    .changed()
                {
                    pos_changed = true;
                }
                ui.end_row();
            });
        if pos_changed && *continuous_solve {
            *on_solve_requested = true;
        }
    });

    ui.add_space(6.0);

    // Target Orientation Inputs
    ui.group(|ui| {
        ui.strong("Target Orientation (Euler Degrees):");
        let mut rot_changed = false;
        Grid::new("target_rpy_grid")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("Roll:");
                if ui
                    .add(
                        DragValue::new(&mut target_rpy_deg[0])
                            .speed(1.0)
                            .suffix("°"),
                    )
                    .changed()
                {
                    rot_changed = true;
                }
                ui.end_row();

                ui.label("Pitch:");
                if ui
                    .add(
                        DragValue::new(&mut target_rpy_deg[1])
                            .speed(1.0)
                            .suffix("°"),
                    )
                    .changed()
                {
                    rot_changed = true;
                }
                ui.end_row();

                ui.label("Yaw:");
                if ui
                    .add(
                        DragValue::new(&mut target_rpy_deg[2])
                            .speed(1.0)
                            .suffix("°"),
                    )
                    .changed()
                {
                    rot_changed = true;
                }
                ui.end_row();
            });
        if rot_changed && *continuous_solve {
            *solver_mode = IKSolverMode::FullPose;
            *on_solve_requested = true;
        }
    });

    ui.add_space(8.0);

    // Solver Selection
    ui.group(|ui| {
        ui.strong("IK Algorithm:");
        ui.radio_value(
            solver_type,
            IKSolverType::JacobianDLS,
            "Jacobian Damped Least Squares (DLS)",
        );
        ui.radio_value(
            solver_type,
            IKSolverType::FABRIK,
            "FABRIK (Forward/Backward Reaching)",
        );
        ui.radio_value(
            solver_type,
            IKSolverType::JacobianTranspose,
            "Jacobian Transpose",
        );

        ui.add_space(4.0);
        ui.strong("Target Objective:");
        ui.horizontal(|ui| {
            ui.radio_value(
                solver_mode,
                IKSolverMode::PositionOnly,
                "Position Only (3D)",
            );
            ui.radio_value(solver_mode, IKSolverMode::FullPose, "Full 6D Pose");
        });
    });

    ui.add_space(8.0);

    // Solver Hyperparameters
    egui::CollapsingHeader::new("⚙️ Solver Tuning & Convergence")
        .default_open(false)
        .show(ui, |ui| {
            Grid::new("ik_tuning_grid")
                .num_columns(2)
                .spacing([10.0, 6.0])
                .show(ui, |ui| {
                    ui.label("Damping Factor (λ):");
                    ui.add(Slider::new(&mut params.damping, 0.001..=0.3).logarithmic(true));
                    ui.end_row();

                    ui.label("Step Scale (α):");
                    ui.add(Slider::new(&mut params.step_size, 0.1..=1.5));
                    ui.end_row();

                    ui.label("Tolerance (mm):");
                    let mut tol_mm = params.tolerance * 1000.0;
                    if ui
                        .add(Slider::new(&mut tol_mm, 0.1..=10.0).suffix(" mm"))
                        .changed()
                    {
                        params.tolerance = tol_mm / 1000.0;
                    }
                    ui.end_row();

                    ui.label("Max Iterations:");
                    ui.add(Slider::new(&mut params.max_iterations, 5..=200));
                    ui.end_row();

                    ui.label("Null-Space Centering:");
                    ui.checkbox(
                        &mut params.enable_null_space_centering,
                        "Joint Limit Avoidance",
                    );
                    ui.end_row();

                    ui.label("Adaptive SR Damping:");
                    ui.checkbox(&mut params.adaptive_damping, "Singularity-Robust");
                    ui.end_row();

                    ui.label("Line-Search Verification:");
                    ui.checkbox(&mut params.enable_line_search, "Monotonic Step Check");
                    ui.end_row();
                });
        });

    ui.add_space(8.0);

    // Execution Buttons
    ui.horizontal(|ui| {
        if ui.button("⚡ Solve Inverse Kinematics").clicked() {
            *on_solve_requested = true;
        }
        ui.checkbox(continuous_solve, "Real-time Tracking");
    });

    ui.add_space(8.0);

    // Performance & Benchmark Monitor
    if let Some(sol) = last_solution {
        ui.group(|ui| {
            ui.horizontal(|ui| {
                ui.strong("Status:");
                if sol.converged {
                    ui.colored_label(Color32::from_rgb(50, 210, 80), "✔ CONVERGED");
                } else {
                    ui.colored_label(Color32::from_rgb(240, 70, 70), "✖ NOT CONVERGED");
                }
            });

            Grid::new("sol_metrics_grid")
                .num_columns(2)
                .spacing([12.0, 3.0])
                .show(ui, |ui| {
                    ui.label("Iterations:");
                    ui.monospace(format!("{}", sol.iterations));
                    ui.end_row();

                    ui.label("Position Error:");
                    ui.monospace(format!("{:.3} mm", sol.residual_position_error * 1000.0));
                    ui.end_row();

                    if *solver_mode == IKSolverMode::FullPose {
                        ui.label("Orientation Error:");
                        ui.monospace(format!(
                            "{:.2}°",
                            sol.residual_orientation_error.to_degrees()
                        ));
                        ui.end_row();
                    }

                    ui.label("Solve Time:");
                    ui.monospace(format!("{} µs", sol.solve_time_us));
                    ui.end_row();

                    if let Some(m_data) = manip_data {
                        ui.label("Manipulability Index:");
                        let text = format!("{:.4}", m_data.score);
                        if m_data.is_near_singularity {
                            ui.colored_label(
                                Color32::from_rgb(255, 60, 40),
                                format!("{} (SINGULAR)", text),
                            );
                        } else {
                            ui.monospace(text);
                        }
                        ui.end_row();
                    }
                });
        });
    }
}
