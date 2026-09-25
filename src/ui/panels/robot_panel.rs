use crate::kinematics::RobotArm;
use crate::presets;
use egui::{Color32, CollapsingHeader, DragValue, Grid, Slider, Ui};
use nalgebra::Point3;

/// Renders the Robot Configuration, Manual Joint Inspector, and Motor Dynamics Load panel.
pub fn render_robot_panel(
    ui: &mut Ui,
    robot: &mut RobotArm,
    on_preset_selected: &mut Option<RobotArm>,
    dynamics_report: &crate::kinematics::JointDynamicsReport,
    payload_mass_kg: &mut f64,
    target_pos: &mut Point3<f64>,
    target_rpy_deg: &mut [f64; 3],
    smoother: &mut crate::trajectory::JointTrajectorySmoother,
    show_graphs: &mut bool,
) {
    ui.heading("🤖 Robot Configuration");
    ui.separator();

    // Preset selector
    ui.label("Preset Robot Models:");
    ui.horizontal_wrapped(|ui| {
        if ui.button("2D Planar 3-DOF").clicked() {
            *on_preset_selected = Some(presets::planar_3dof());
        }
        if ui.button("3D SCARA 4-DOF").clicked() {
            *on_preset_selected = Some(presets::scara_4dof());
        }
        if ui.button("Industrial 6-DOF").clicked() {
            *on_preset_selected = Some(presets::industrial_6dof());
        }
        if ui.button("Redundant 7-DOF").clicked() {
            *on_preset_selected = Some(presets::redundant_7dof());
        }
    });

    ui.add_space(8.0);
    ui.group(|ui| {
        ui.label(format!("Active Robot: {}", robot.name));
        ui.label(format!("Actuated DOFs: {}", robot.dof()));
        ui.label(format!("Max Reach: {:.3} m", robot.total_reach()));
    });

    ui.add_space(8.0);
    let mut joint_moved = false;

    ui.horizontal_wrapped(|ui| {
        if ui.button("🏠 Home Position").clicked() {
            robot.reset_to_home();
            joint_moved = true;
        }
        if ui.button("🔄 Zero All Joints").clicked() {
            for j in &mut robot.joints {
                j.set_position(0.0);
            }
            joint_moved = true;
        }
        if ui.button("📐 Ready Pose").clicked() {
            let dof = robot.dof();
            let mut angles = vec![0.0; dof];
            if dof >= 6 {
                angles[0] = 0.0;
                angles[1] = 0.5236; // 30 deg
                angles[2] = -1.0472; // -60 deg
                angles[3] = 0.0;
                angles[4] = 0.5236; // 30 deg
                angles[5] = 0.0;
            } else if dof == 4 {
                angles[0] = 0.5236;
                angles[1] = -0.5236;
                angles[2] = 0.1;
                angles[3] = 0.0;
            } else if dof == 3 {
                angles[0] = 0.5236;
                angles[1] = -0.7854;
                angles[2] = 0.2618;
            }
            robot.set_actuated_joint_positions(&angles);
            joint_moved = true;
        }
    });

    ui.add_space(8.0);
    CollapsingHeader::new("⚙️ Manual Joint Control & Jogging")
        .default_open(true)
        .show(ui, |ui| {
            ui.label("Directly adjust or jog individual joint angles below:");
            ui.add_space(4.0);

            for (i, joint) in robot.joints.iter_mut().enumerate() {
                if !joint.is_actuated() {
                    continue;
                }

                let (min, max) = joint
                    .limits
                    .unwrap_or((-std::f64::consts::PI, std::f64::consts::PI));
                let is_prismatic =
                    joint.joint_type == crate::kinematics::JointType::Prismatic;

                egui::Frame::none()
                    .fill(Color32::from_rgb(26, 28, 34))
                    .inner_margin(6.0)
                    .rounding(4.0)
                    .show(ui, |ui| {
                        // Header row: Joint Name and Limit Warning
                        let is_near_limit = if is_prismatic {
                            (joint.current_position - min).abs() < 0.01
                                || (joint.current_position - max).abs() < 0.01
                        } else {
                            let deg = joint.current_position.to_degrees();
                            let min_deg = min.to_degrees();
                            let max_deg = max.to_degrees();
                            (deg - min_deg).abs() < 3.0 || (deg - max_deg).abs() < 3.0
                        };

                        ui.horizontal(|ui| {
                            ui.strong(format!("J{}: {}", i + 1, joint.name));
                            if is_near_limit {
                                ui.colored_label(Color32::from_rgb(255, 120, 40), "⚠ Limit");
                            }
                        });

                        // Main adjustment row: Slider + DragValue
                        ui.horizontal(|ui| {
                            if is_prismatic {
                                let mut pos = joint.current_position;
                                if ui
                                    .add(
                                        Slider::new(&mut pos, min..=max)
                                            .suffix(" m")
                                            .step_by(0.005),
                                    )
                                    .changed()
                                {
                                    joint.set_position(pos);
                                    joint_moved = true;
                                }
                                if ui
                                    .add(DragValue::new(&mut pos).speed(0.005).suffix(" m"))
                                    .changed()
                                {
                                    joint.set_position(pos);
                                    joint_moved = true;
                                }
                            } else {
                                let mut deg = joint.current_position.to_degrees();
                                let min_deg = min.to_degrees();
                                let max_deg = max.to_degrees();

                                if ui
                                    .add(
                                        Slider::new(&mut deg, min_deg..=max_deg)
                                            .suffix("°")
                                            .step_by(0.5),
                                    )
                                    .changed()
                                {
                                    joint.set_position(deg.to_radians());
                                    joint_moved = true;
                                }
                                if ui
                                    .add(DragValue::new(&mut deg).speed(0.5).suffix("°"))
                                    .changed()
                                {
                                    joint.set_position(deg.to_radians());
                                    joint_moved = true;
                                }
                            }
                        });

                        // Jog buttons row: [-10°] [-1°] [0°] [+1°] [+10°]
                        ui.horizontal(|ui| {
                            ui.label("Jog:");
                            if is_prismatic {
                                if ui.small_button("-10cm").clicked() {
                                    joint.set_position(joint.current_position - 0.10);
                                    joint_moved = true;
                                }
                                if ui.small_button("-1cm").clicked() {
                                    joint.set_position(joint.current_position - 0.01);
                                    joint_moved = true;
                                }
                                if ui.small_button("0cm").clicked() {
                                    joint.set_position(0.0);
                                    joint_moved = true;
                                }
                                if ui.small_button("+1cm").clicked() {
                                    joint.set_position(joint.current_position + 0.01);
                                    joint_moved = true;
                                }
                                if ui.small_button("+10cm").clicked() {
                                    joint.set_position(joint.current_position + 0.10);
                                    joint_moved = true;
                                }
                            } else {
                                if ui.small_button("-10°").clicked() {
                                    joint.set_position(
                                        joint.current_position - 10.0_f64.to_radians(),
                                    );
                                    joint_moved = true;
                                }
                                if ui.small_button("-1°").clicked() {
                                    joint.set_position(
                                        joint.current_position - 1.0_f64.to_radians(),
                                    );
                                    joint_moved = true;
                                }
                                if ui.small_button("0°").clicked() {
                                    joint.set_position(0.0);
                                    joint_moved = true;
                                }
                                if ui.small_button("+1°").clicked() {
                                    joint.set_position(
                                        joint.current_position + 1.0_f64.to_radians(),
                                    );
                                    joint_moved = true;
                                }
                                if ui.small_button("+10°").clicked() {
                                    joint.set_position(
                                        joint.current_position + 10.0_f64.to_radians(),
                                    );
                                    joint_moved = true;
                                }
                            }
                        });
                    });
                ui.add_space(4.0);
            }
        });

    if joint_moved {
        *target_pos = robot.end_effector_position();
        let (r, p, y) = robot.end_effector_pose().rotation.euler_angles();
        *target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
        smoother.reset(&robot.get_actuated_joint_positions());
    }

    ui.add_space(8.0);
    CollapsingHeader::new("📍 Forward Kinematics Readout")
        .default_open(false)
        .show(ui, |ui| {
            let ee_pos = robot.end_effector_position();
            let ee_rot = robot.end_effector_pose().rotation;
            let (r, p, y) = ee_rot.euler_angles();

            Grid::new("fk_readout_grid")
                .num_columns(2)
                .spacing([12.0, 4.0])
                .show(ui, |ui| {
                    ui.label("End-Effector X:");
                    ui.monospace(format!("{:.4} m", ee_pos.x));
                    ui.end_row();

                    ui.label("End-Effector Y:");
                    ui.monospace(format!("{:.4} m", ee_pos.y));
                    ui.end_row();

                    ui.label("End-Effector Z:");
                    ui.monospace(format!("{:.4} m", ee_pos.z));
                    ui.end_row();

                    ui.label("Roll (deg):");
                    ui.monospace(format!("{:.2}°", r.to_degrees()));
                    ui.end_row();

                    ui.label("Pitch (deg):");
                    ui.monospace(format!("{:.2}°", p.to_degrees()));
                    ui.end_row();

                    ui.label("Yaw (deg):");
                    ui.monospace(format!("{:.2}°", y.to_degrees()));
                    ui.end_row();
                });
        });

    ui.add_space(8.0);
    ui.separator();

    // Graphing Click Button (graphs removed by default unless clicked)
    ui.horizontal(|ui| {
        let btn_text = if *show_graphs {
            "📊 Hide Dynamics & Strain Graphs"
        } else {
            "📊 Show Dynamics & Strain Graphs"
        };
        if ui.button(btn_text).clicked() {
            *show_graphs = !*show_graphs;
        }
    });

    if *show_graphs {
        ui.add_space(4.0);
        CollapsingHeader::new("⚡ Inverse Dynamics & Motor Load Gauges")
            .default_open(true)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Payload Mass:");
                    ui.add(Slider::new(payload_mass_kg, 0.0..=10.0).suffix(" kg"));
                });

                ui.add_space(4.0);
                Grid::new("dynamics_load_grid")
                    .num_columns(4)
                    .spacing([8.0, 4.0])
                    .striped(true)
                    .show(ui, |ui| {
                        ui.strong("Joint");
                        ui.strong("Torque");
                        ui.strong("Rated");
                        ui.strong("Motor Strain Load");
                        ui.end_row();

                        for (i, joint) in robot.joints.iter().enumerate() {
                            if i >= dynamics_report.joint_torques_nm.len() {
                                break;
                            }
                            ui.label(&joint.name);
                            let tau = dynamics_report.joint_torques_nm[i];
                            ui.monospace(format!("{:.1} Nm", tau));
                            let cap = dynamics_report.rated_capacities_nm[i];
                            ui.monospace(format!("{:.0} Nm", cap));

                            let pct = dynamics_report.load_percentages[i];
                            let ratio = (pct / 100.0).clamp(0.0, 1.0) as f32;
                            let text = format!("{:.0}%", pct);
                            ui.add(egui::ProgressBar::new(ratio).text(text).animate(false));
                            ui.end_row();
                        }
                    });
            });
    }
}
