use egui::{Color32, DragValue, Grid, Slider, Ui};
use nalgebra::Point3;

use crate::kinematics::RobotArm;
use crate::trajectory::{
    plan_cartesian_rrt, PolynomialType, RrtAlgorithm, RrtParams, RrtPlanResult,
    TrajectoryContinuityMode, TrajectoryController, TrajectoryRecorder,
};
use crate::workcell::WorkcellEnvironment;

/// Sub-sections of the Trajectory & Motion Control Panel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrajSubTab {
    Waypoints,
    RrtPlanner,
    LiveRecorder,
    ExportImport,
}

/// Renders the enhanced Trajectory Planning, RRT Obstacle Avoidance, and Recording panel.
#[allow(clippy::too_many_arguments)]
pub fn render_trajectory_panel(
    ui: &mut Ui,
    controller: &mut TrajectoryController,
    recorder: &mut TrajectoryRecorder,
    robot: &RobotArm,
    environment: &WorkcellEnvironment,
    rrt_params: &mut RrtParams,
    rrt_goal_pos: &mut Point3<f64>,
    last_rrt_result: &mut Option<RrtPlanResult>,
    current_target_pos: Point3<f64>,
    on_export_requested: &mut Option<&'static str>,
    on_import_json: &mut Option<String>,
) {
    ui.heading("📈 Trajectory & Motion Engine");
    ui.separator();

    // Sub-tab navigation selector
    let mut current_sub_tab = ui.data_mut(|d| {
        d.get_temp::<TrajSubTab>(egui::Id::new("traj_sub_tab"))
            .unwrap_or(TrajSubTab::Waypoints)
    });

    ui.horizontal_wrapped(|ui| {
        ui.selectable_value(&mut current_sub_tab, TrajSubTab::Waypoints, "📈 Waypoints");
        ui.selectable_value(
            &mut current_sub_tab,
            TrajSubTab::RrtPlanner,
            "🚀 RRT Planner",
        );
        ui.selectable_value(&mut current_sub_tab, TrajSubTab::LiveRecorder, "⏺ Recorder");
        ui.selectable_value(
            &mut current_sub_tab,
            TrajSubTab::ExportImport,
            "📤 Export / Import",
        );
    });
    ui.data_mut(|d| d.insert_temp(egui::Id::new("traj_sub_tab"), current_sub_tab));

    ui.separator();

    match current_sub_tab {
        TrajSubTab::Waypoints => {
            render_waypoints_section(ui, controller, current_target_pos, on_export_requested);
        }
        TrajSubTab::RrtPlanner => {
            render_rrt_planner_section(
                ui,
                controller,
                robot,
                environment,
                rrt_params,
                rrt_goal_pos,
                last_rrt_result,
                current_target_pos,
            );
        }
        TrajSubTab::LiveRecorder => {
            render_live_recorder_section(ui, controller, recorder, robot, on_export_requested);
        }
        TrajSubTab::ExportImport => {
            render_export_import_section(ui, recorder, on_export_requested, on_import_json);
        }
    }
}

/// Section 1: Traditional Waypoint Sequencer & Smooth Polynomial Interpolation
fn render_waypoints_section(
    ui: &mut Ui,
    controller: &mut TrajectoryController,
    current_target_pos: Point3<f64>,
    on_export_requested: &mut Option<&'static str>,
) {
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
                    controller.planner.continuity_mode
                        == TrajectoryContinuityMode::SmoothContinuous,
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
        ui.checkbox(
            &mut controller.enable_smoothing,
            "🛡️ Joint Velocity Limiter & Filter",
        );
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
        ui.strong("📤 Quick Export:");
        ui.horizontal_wrapped(|ui| {
            if ui.button("🐍 Python").clicked() {
                *on_export_requested = Some("python");
            }
            if ui.button("🤖 ROS 2").clicked() {
                *on_export_requested = Some("ros2");
            }
            if ui.button("⚙️ G-Code").clicked() {
                *on_export_requested = Some("gcode");
            }
            if ui.button("💾 JSON").clicked() {
                *on_export_requested = Some("json");
            }
        });
    });
}

/// Section 2: RRT / Obstacle Avoidance Motion Planner
#[allow(clippy::too_many_arguments)]
fn render_rrt_planner_section(
    ui: &mut Ui,
    controller: &mut TrajectoryController,
    robot: &RobotArm,
    environment: &WorkcellEnvironment,
    rrt_params: &mut RrtParams,
    rrt_goal_pos: &mut Point3<f64>,
    last_rrt_result: &mut Option<RrtPlanResult>,
    current_target_pos: Point3<f64>,
) {
    ui.group(|ui| {
        ui.strong("🛡️ Active Workcell Obstacles:");
        if environment.obstacles.is_empty() {
            ui.label(
                egui::RichText::new("No obstacles active. Workspace is clear.")
                    .color(Color32::from_rgb(100, 200, 100)),
            );
        } else {
            ui.horizontal_wrapped(|ui| {
                for obs in &environment.obstacles {
                    ui.label(
                        egui::RichText::new(format!("🧱 {}", obs.name))
                            .color(Color32::from_rgb(255, 180, 80)),
                    );
                }
            });
        }
    });

    ui.add_space(6.0);

    // Goal Configuration
    ui.group(|ui| {
        ui.strong("🎯 Goal Destination Target (XYZ in meters):");
        ui.horizontal(|ui| {
            ui.label("X:");
            ui.add(DragValue::new(&mut rrt_goal_pos.x).speed(0.01).suffix(" m"));
            ui.label("Y:");
            ui.add(DragValue::new(&mut rrt_goal_pos.y).speed(0.01).suffix(" m"));
            ui.label("Z:");
            ui.add(DragValue::new(&mut rrt_goal_pos.z).speed(0.01).suffix(" m"));
        });

        ui.horizontal(|ui| {
            if ui.button("📍 Match Current IK Target").clicked() {
                *rrt_goal_pos = current_target_pos;
            }
            if ui.button("📍 Across Obstacle Preset").clicked() {
                *rrt_goal_pos = Point3::new(-0.25, 0.45, 0.55);
            }
        });
    });

    ui.add_space(6.0);

    // RRT Hyperparameters
    ui.group(|ui| {
        ui.strong("⚙️ RRT Planning Hyperparameters:");
        ui.horizontal(|ui| {
            ui.label("Algorithm:");
            ui.radio_value(
                &mut rrt_params.algorithm,
                RrtAlgorithm::RrtConnect,
                "RRT-Connect (Bi-directional)",
            );
            ui.radio_value(
                &mut rrt_params.algorithm,
                RrtAlgorithm::StandardRrt,
                "Standard RRT",
            );
        });

        ui.horizontal(|ui| {
            ui.label("Max Iterations:");
            ui.add(Slider::new(&mut rrt_params.max_iterations, 500..=6000));
        });

        ui.horizontal(|ui| {
            ui.label("Joint Step Size:");
            ui.add(
                Slider::new(&mut rrt_params.step_size, 0.05..=0.35)
                    .suffix(" rad")
                    .custom_formatter(|n, _| format!("{:.2} rad ({:.1}°)", n, n.to_degrees())),
            );
        });

        ui.horizontal(|ui| {
            ui.checkbox(
                &mut rrt_params.shortcut_path,
                "✨ Path Shortcutting (Angle-Space Smoothing)",
            );
        });
    });

    ui.add_space(8.0);

    // Plan Path Action Button
    let plan_clicked = ui
        .add_sized(
            [ui.available_width(), 32.0],
            egui::Button::new(
                egui::RichText::new("🚀 Plan Collision-Free Path (RRT)")
                    .strong()
                    .color(Color32::WHITE),
            )
            .fill(Color32::from_rgb(40, 120, 220)),
        )
        .clicked();

    if plan_clicked {
        let obstacles = environment.obstacles.clone();
        let plan_res = plan_cartesian_rrt(robot, *rrt_goal_pos, &obstacles, rrt_params);
        *last_rrt_result = Some(plan_res);
    }

    // Display Plan Results
    if let Some(res) = last_rrt_result {
        ui.add_space(6.0);
        ui.group(|ui| {
            if res.success {
                ui.label(
                    egui::RichText::new(format!("✅ {}", res.message))
                        .color(Color32::from_rgb(60, 220, 100))
                        .strong(),
                );
                ui.label(format!(
                    "Path Length: {:.3}m  |  Waypoints: {}  |  Tree Nodes: {}",
                    res.total_path_length(),
                    res.cartesian_path.len(),
                    res.total_nodes
                ));

                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    if ui
                        .button(
                            egui::RichText::new("▶ Load into Trajectory & Play")
                                .strong()
                                .color(Color32::WHITE),
                        )
                        .clicked()
                    {
                        controller.planner.waypoints = res.to_waypoints(0.4);
                        controller.planner.rebuild_trajectory();
                        controller.reset();
                        controller.play();
                    }

                    if ui.button("➕ Append to Waypoints").clicked() {
                        let wps = res.to_waypoints(0.4);
                        for wp in wps {
                            controller
                                .planner
                                .add_waypoint(wp.name, wp.position, wp.duration);
                        }
                    }
                });
            } else {
                ui.label(
                    egui::RichText::new(format!("❌ {}", res.message))
                        .color(Color32::from_rgb(255, 90, 80))
                        .strong(),
                );
                ui.label(format!(
                    "Evaluated in {:.2}ms ({} iterations)",
                    res.planning_time_us as f64 / 1000.0,
                    res.iterations
                ));
            }
        });
    }
}

/// Section 3: Live Interactive Trajectory Recording & Replay
fn render_live_recorder_section(
    ui: &mut Ui,
    controller: &mut TrajectoryController,
    recorder: &mut TrajectoryRecorder,
    _robot: &RobotArm,
    on_export_requested: &mut Option<&'static str>,
) {
    ui.group(|ui| {
        ui.strong("⏺ Live Motion Capture:");
        ui.label("Record your mouse movements, IK jogging, or procedural animations in real-time.");

        ui.add_space(6.0);

        if recorder.is_recording {
            let elapsed = recorder.recording.total_duration;
            let count = recorder.recording.frame_count();
            ui.horizontal(|ui| {
                ui.label(
                    egui::RichText::new("🔴 RECORDING LIVE...")
                        .color(Color32::from_rgb(255, 60, 60))
                        .strong(),
                );
                ui.label(format!("{:.1}s ({} frames)", elapsed, count));
            });

            if ui
                .add_sized(
                    [ui.available_width(), 30.0],
                    egui::Button::new("⏹ Stop Recording").fill(Color32::from_rgb(200, 50, 50)),
                )
                .clicked()
            {
                recorder.stop_recording();
            }
        } else {
            ui.horizontal(|ui| {
                if ui
                    .add_sized(
                        [160.0, 30.0],
                        egui::Button::new("⏺ Start Recording").fill(Color32::from_rgb(50, 160, 80)),
                    )
                    .clicked()
                {
                    recorder.start_recording(Some(format!(
                        "Session {}",
                        recorder.recording.frame_count() + 1
                    )));
                }

                if !recorder.recording.is_empty() && ui.button("🗑 Clear Recording").clicked() {
                    recorder.clear();
                }
            });
        }
    });

    ui.add_space(6.0);

    // Recording Session Statistics & Replay
    if !recorder.recording.is_empty() {
        ui.group(|ui| {
            ui.strong("📼 Recorded Session Overview:");
            ui.label(format!(
                "Total Duration: {:.2}s  |  Frames: {}  |  Path Distance: {:.3}m",
                recorder.recording.total_duration,
                recorder.recording.frame_count(),
                recorder.recording.total_path_distance()
            ));

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                let replay_label = if recorder.is_replaying {
                    "⏸ Pause Replay"
                } else {
                    "▶ Replay Recording"
                };
                if ui.button(replay_label).clicked() {
                    recorder.toggle_replay();
                }
                if ui.button("⏹ Stop Replay").clicked() {
                    recorder.stop_replay();
                }
                ui.checkbox(&mut recorder.replay_loop, "Loop");
            });

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Replay Speed:");
                ui.add(Slider::new(&mut recorder.replay_speed, 0.2..=3.0).suffix("x"));
            });

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Timeline:");
                let total = recorder.recording.total_duration;
                ui.add(Slider::new(&mut recorder.replay_time, 0.0..=total.max(0.1)).suffix(" s"));
            });

            ui.add_space(6.0);
            if ui
                .button("📋 Convert Recording to Trajectory Waypoints")
                .clicked()
            {
                let wps = recorder.recording.to_waypoints(16);
                controller.planner.waypoints = wps;
                controller.planner.rebuild_trajectory();
            }

            ui.add_space(4.0);
            ui.horizontal_wrapped(|ui| {
                if ui.button("💾 Export Recorded CSV").clicked() {
                    *on_export_requested = Some("csv_recorded");
                }
                if ui.button("💾 Export Recorded JSON").clicked() {
                    *on_export_requested = Some("json");
                }
            });
        });
    }
}

/// Section 4: Export & Import Hub (Python, ROS 2, Arduino C++, MATLAB, JSON, G-Code)
fn render_export_import_section(
    ui: &mut Ui,
    recorder: &TrajectoryRecorder,
    on_export_requested: &mut Option<&'static str>,
    on_import_json: &mut Option<String>,
) {
    ui.group(|ui| {
        ui.strong("📤 Code & Format Generators:");
        ui.label("Export planned trajectory and recorded motions into production formats:");

        ui.add_space(6.0);

        Grid::new("export_buttons_grid")
            .num_columns(2)
            .spacing([12.0, 8.0])
            .show(ui, |ui| {
                if ui
                    .add_sized(
                        [180.0, 28.0],
                        egui::Button::new("🐍 Python (NumPy / Matplotlib)"),
                    )
                    .clicked()
                {
                    *on_export_requested = Some("python");
                }
                ui.label("Standalone Python simulation & plotting script");
                ui.end_row();

                if ui
                    .add_sized(
                        [180.0, 28.0],
                        egui::Button::new("🤖 ROS 2 (JointTrajectory)"),
                    )
                    .clicked()
                {
                    *on_export_requested = Some("ros2");
                }
                ui.label("ROS 2 FollowJointTrajectory action YAML goal");
                ui.end_row();

                if ui
                    .add_sized([180.0, 28.0], egui::Button::new("⚡ Arduino / ESP32 C++"))
                    .clicked()
                {
                    *on_export_requested = Some("arduino");
                }
                ui.label("Ready-to-flash C++ sketch with servo angle arrays");
                ui.end_row();

                if ui
                    .add_sized([180.0, 28.0], egui::Button::new("📐 MATLAB / Simulink"))
                    .clicked()
                {
                    *on_export_requested = Some("matlab");
                }
                ui.label("3D trajectory and joint angle plot script");
                ui.end_row();

                if ui
                    .add_sized([180.0, 28.0], egui::Button::new("⚙️ CNC G-Code & CSV"))
                    .clicked()
                {
                    *on_export_requested = Some("gcode");
                }
                ui.label("Standard G01 linear motion & CSV coordinates");
                ui.end_row();

                if ui
                    .add_sized([180.0, 28.0], egui::Button::new("💾 Full Session JSON"))
                    .clicked()
                {
                    *on_export_requested = Some("json");
                }
                ui.label("Complete trajectory session backup");
                ui.end_row();

                if !recorder.recording.is_empty() {
                    if ui
                        .add_sized([180.0, 28.0], egui::Button::new("📼 Recorded CSV"))
                        .clicked()
                    {
                        *on_export_requested = Some("csv_recorded");
                    }
                    ui.label("Time-series joint & tool positions");
                    ui.end_row();
                }
            });
    });

    ui.add_space(8.0);

    // JSON Trajectory Import
    ui.group(|ui| {
        ui.strong("📥 Import Trajectory JSON:");
        ui.label("Paste a previously exported trajectory JSON payload here to reload waypoints:");

        ui.add_space(4.0);

        let mut import_buf = ui.data_mut(|d| {
            d.get_temp::<String>(egui::Id::new("json_import_text"))
                .unwrap_or_default()
        });

        ui.add(
            egui::TextEdit::multiline(&mut import_buf)
                .hint_text("{\n  \"format_version\": \"kine-rs-trajectory-v1.0\",\n  ...\n}")
                .desired_rows(6)
                .desired_width(ui.available_width()),
        );

        ui.horizontal(|ui| {
            if ui.button("📥 Load Trajectory").clicked() && !import_buf.trim().is_empty() {
                *on_import_json = Some(import_buf.clone());
            }
            if ui.button("Clear").clicked() {
                import_buf.clear();
            }
        });

        ui.data_mut(|d| d.insert_temp(egui::Id::new("json_import_text"), import_buf));
    });
}
