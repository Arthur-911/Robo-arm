use eframe::App;
use egui::{Color32, Context, Key, PointerButton, Pos2, Sense};
use nalgebra::{Point3, Vector3};
use web_time::Instant;

use super::camera::OrbitCamera;
use super::panels::*;
use super::renderer_3d::{render_scene_3d, GizmoDragAxis, RenderSettings};
use crate::kinematics::{
    check_collisions, compute_gravity_torques, compute_manipulability, solve_ik, IKSolution,
    IKSolverMode, IKSolverParams, IKSolverType, RobotArm,
};
use crate::math::euler_to_quaternion;
use crate::presets;
use crate::trajectory::TrajectoryController;
use crate::workcell::{ToolState, WorkcellEnvironment, WorkpieceManager};

/// Active tab in the sidebar control panel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AppTab {
    IK,
    Robot,
    Chat,
    Trajectory,
    Manipulation,
    URDF,
}

/// Target motion descriptor for smooth animated transitions.
#[derive(Debug, Clone)]
pub enum SmoothMotionTarget {
    Cartesian {
        start: Point3<f64>,
        goal: Point3<f64>,
        start_rpy: [f64; 3],
        goal_rpy: [f64; 3],
    },
    Joints {
        start_q: Vec<f64>,
        goal_q: Vec<f64>,
    },
    Gripper {
        start: f32,
        goal: f32,
    },
    PickSequence {
        stage: u8,
        stage_start: Instant,
        stage_duration: f64,
        approach_pos: Point3<f64>,
        grasp_pos: Point3<f64>,
        lift_pos: Point3<f64>,
    },
}

/// State tracking an active smooth motion interpolation.
#[derive(Debug, Clone)]
pub struct SmoothMotionState {
    pub start_time: Instant,
    pub duration_secs: f64,
    pub target: SmoothMotionTarget,
}

/// The core application state for the kine-rs simulator.
pub struct RoboSimApp {
    pub robot: RobotArm,
    pub camera: OrbitCamera,
    pub render_settings: RenderSettings,

    // IK Target & Settings
    pub target_pos: Point3<f64>,
    pub target_rpy_deg: [f64; 3],
    pub solver_type: IKSolverType,
    pub solver_mode: IKSolverMode,
    pub solver_params: IKSolverParams,
    pub continuous_solve: bool,
    pub last_solution: Option<IKSolution>,

    // Trajectory & Motion Planning
    pub controller: TrajectoryController,
    pub recorder: crate::trajectory::TrajectoryRecorder,
    pub rrt_params: crate::trajectory::RrtParams,
    pub rrt_goal_pos: Point3<f64>,
    pub last_rrt_result: Option<crate::trajectory::RrtPlanResult>,

    // Workcell & Manipulation
    pub tool_state: ToolState,
    pub workpieces: WorkpieceManager,
    pub environment: WorkcellEnvironment,
    pub payload_mass_kg: f64,

    // URDF State
    pub urdf_editor_text: String,
    pub urdf_error: Option<String>,

    // Robotic Chat / Command Console
    pub chat: super::chat_command::ChatCommandConsole,

    // Smooth motion execution for text & interactive commands
    pub active_smooth_motion: Option<SmoothMotionState>,

    // Graph visibility toggle (hidden by default unless user clicks graphing button)
    pub show_graphs: bool,

    // Navigation & Interaction
    pub active_tab: AppTab,
    pub active_drag_axis: GizmoDragAxis,
    pub drag_start_mouse: Option<Pos2>,
    pub drag_start_target: Point3<f64>,
    pub drag_start_rpy: [f64; 3],

    // Export Modal Window (Title, Generated Code)
    pub export_modal: Option<(String, String)>,

    // Performance & FPS tracking
    pub last_frame_time: Instant,
    pub fps: f32,
    pub show_about_dialog: bool,
}

impl Default for RoboSimApp {
    fn default() -> Self {
        let robot = presets::industrial_6dof();
        let target_pos = robot.end_effector_position();
        let (r, p, y) = robot.end_effector_pose().rotation.euler_angles();

        Self {
            robot,
            camera: OrbitCamera::default(),
            render_settings: RenderSettings::default(),
            target_pos,
            target_rpy_deg: [r.to_degrees(), p.to_degrees(), y.to_degrees()],
            solver_type: IKSolverType::JacobianDLS,
            solver_mode: IKSolverMode::PositionOnly,
            solver_params: IKSolverParams::default(),
            continuous_solve: true,
            last_solution: None,
            controller: TrajectoryController::default(),
            recorder: crate::trajectory::TrajectoryRecorder::default(),
            rrt_params: crate::trajectory::RrtParams::default(),
            rrt_goal_pos: target_pos,
            last_rrt_result: None,
            tool_state: ToolState::default(),
            workpieces: WorkpieceManager::default(),
            environment: WorkcellEnvironment::default(),
            payload_mass_kg: 2.5,
            urdf_editor_text: presets::SAMPLE_URDF_INDUSTRIAL_6DOF.to_string(),
            urdf_error: None,
            chat: super::chat_command::ChatCommandConsole::default(),
            active_smooth_motion: None,
            show_graphs: false,
            active_tab: AppTab::IK,
            active_drag_axis: GizmoDragAxis::None,
            drag_start_mouse: None,
            drag_start_target: target_pos,
            drag_start_rpy: [r.to_degrees(), p.to_degrees(), y.to_degrees()],
            export_modal: None,
            last_frame_time: Instant::now(),
            fps: 60.0,
            show_about_dialog: false,
        }
    }
}

impl RoboSimApp {
    /// Executes the Inverse Kinematics solver against the current target.
    pub fn execute_solve(&mut self) {
        let target_rot = if self.solver_mode == IKSolverMode::FullPose {
            Some(euler_to_quaternion(
                self.target_rpy_deg[0].to_radians(),
                self.target_rpy_deg[1].to_radians(),
                self.target_rpy_deg[2].to_radians(),
            ))
        } else {
            None
        };

        let sol = solve_ik(
            &self.robot,
            self.target_pos,
            target_rot,
            self.solver_type,
            self.solver_mode,
            &self.solver_params,
        );

        if !sol.joint_positions.is_empty() {
            let final_q = if self.controller.enable_smoothing && self.continuous_solve {
                self.controller.smoother.filter(&sol.joint_positions, 0.016)
            } else {
                self.controller.smoother.reset(&sol.joint_positions);
                sol.joint_positions.clone()
            };
            self.robot.set_actuated_joint_positions(&final_q);
        }

        let ee = self.robot.end_effector_position();
        self.controller.add_trail_point(ee);
        self.last_solution = Some(sol);
    }

    /// Plans a collision-free path to target_pos avoiding all workcell obstacles using RRT.
    pub fn plan_rrt_to_target(&mut self, target_pos: Point3<f64>) -> bool {
        let obstacles = self.environment.obstacles.clone();
        let plan_res = crate::trajectory::plan_cartesian_rrt(
            &self.robot,
            target_pos,
            &obstacles,
            &self.rrt_params,
        );
        let success = plan_res.success;
        if success {
            self.controller.planner.waypoints = plan_res.to_waypoints(0.4);
            self.controller.planner.rebuild_trajectory();
            self.controller.reset();
            self.controller.play();
        }
        self.last_rrt_result = Some(plan_res);
        success
    }

    /// Starts a smooth minimum-jerk Cartesian movement to goal_pos and optional goal_rpy.
    pub fn start_smooth_cartesian_move(
        &mut self,
        goal: Point3<f64>,
        goal_rpy: Option<[f64; 3]>,
        duration: f64,
    ) {
        let start = self.target_pos;
        let start_rpy = self.target_rpy_deg;
        let final_rpy = goal_rpy.unwrap_or(start_rpy);
        let dist = (goal - start).norm();
        let dur = duration.max(dist / 0.35).clamp(0.4, 3.5);

        self.active_smooth_motion = Some(SmoothMotionState {
            start_time: Instant::now(),
            duration_secs: dur,
            target: SmoothMotionTarget::Cartesian {
                start,
                goal,
                start_rpy,
                goal_rpy: final_rpy,
            },
        });
    }

    /// Starts a smooth minimum-jerk joint movement to goal_q.
    pub fn start_smooth_joint_move(&mut self, goal_q: Vec<f64>, duration: f64) {
        let start_q = self.robot.get_actuated_joint_positions();
        let max_delta = start_q
            .iter()
            .zip(goal_q.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max);
        let dur = duration.max(max_delta / 1.5).clamp(0.4, 3.5);

        self.active_smooth_motion = Some(SmoothMotionState {
            start_time: Instant::now(),
            duration_secs: dur,
            target: SmoothMotionTarget::Joints { start_q, goal_q },
        });
    }

    /// Starts a smooth gripper transition.
    pub fn start_smooth_gripper_move(&mut self, goal_opening: f32, duration: f64) {
        let start = self.tool_state.gripper_opening;
        self.active_smooth_motion = Some(SmoothMotionState {
            start_time: Instant::now(),
            duration_secs: duration.clamp(0.2, 1.0),
            target: SmoothMotionTarget::Gripper {
                start,
                goal: goal_opening,
            },
        });
    }

    /// Starts an automated smooth multi-stage pick sequence: approach -> descend -> grasp -> lift.
    pub fn start_smooth_pick_sequence(&mut self, grasp_pos: Point3<f64>) {
        let approach_pos = grasp_pos + Vector3::new(0.0, 0.0, 0.08);
        let lift_pos = grasp_pos + Vector3::new(0.0, 0.0, 0.15);

        self.tool_state.gripper_opening = 0.85; // Open jaws
        self.active_smooth_motion = Some(SmoothMotionState {
            start_time: Instant::now(),
            duration_secs: 0.8,
            target: SmoothMotionTarget::PickSequence {
                stage: 0,
                stage_start: Instant::now(),
                stage_duration: 0.8,
                approach_pos,
                grasp_pos,
                lift_pos,
            },
        });
    }

    /// Immediately snaps any pending smooth motion to its goal (used for tests or rapid reset).
    pub fn finish_active_smooth_motion(&mut self) {
        if let Some(motion) = self.active_smooth_motion.take() {
            match motion.target {
                SmoothMotionTarget::Cartesian { goal, goal_rpy, .. } => {
                    self.target_pos = goal;
                    self.target_rpy_deg = goal_rpy;
                    self.execute_solve();
                }
                SmoothMotionTarget::Joints { goal_q, .. } => {
                    self.robot.set_actuated_joint_positions(&goal_q);
                    self.target_pos = self.robot.end_effector_position();
                    let (r, p, y) = self.robot.end_effector_pose().rotation.euler_angles();
                    self.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
                    self.controller.smoother.reset(&goal_q);
                }
                SmoothMotionTarget::Gripper { goal, .. } => {
                    self.tool_state.gripper_opening = goal;
                }
                SmoothMotionTarget::PickSequence { lift_pos, .. } => {
                    self.target_pos = lift_pos;
                    self.tool_state.gripper_opening = 0.05;
                    self.execute_solve();
                }
            }
        }
    }
}

impl App for RoboSimApp {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        // Calculate instantaneous FPS
        let now = Instant::now();
        let dt = (now - self.last_frame_time).as_secs_f32();
        self.last_frame_time = now;
        if dt > 0.001 {
            self.fps = self.fps * 0.9 + (1.0 / dt) * 0.1;
        }

        // Handle Drag-and-Drop URDF files
        let dropped_files = ctx.input(|i| i.raw.dropped_files.clone());
        for file in dropped_files {
            let content = if let Some(bytes) = file.bytes {
                String::from_utf8(bytes.to_vec()).ok()
            } else if let Some(path) = file.path {
                std::fs::read_to_string(path).ok()
            } else {
                None
            };

            if let Some(urdf_str) = content {
                match crate::urdf::parse_urdf(&urdf_str) {
                    Ok(new_robot) => {
                        self.robot = new_robot;
                        self.target_pos = self.robot.end_effector_position();
                        let (r, p, y) = self.robot.end_effector_pose().rotation.euler_angles();
                        self.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
                        self.urdf_editor_text = urdf_str;
                        self.urdf_error = None;
                        self.controller.clear_trail();
                    }
                    Err(err) => {
                        self.urdf_error = Some(format!("Failed to parse dropped URDF: {}", err));
                    }
                }
            }
        }

        // Real-time calculations: Manipulability, Dynamics, Collisions & Object grasp updates
        let manip_data = compute_manipulability(&self.robot);
        let dynamics_report = compute_gravity_torques(&self.robot, self.payload_mass_kg);
        let collision_report = check_collisions(&self.robot, &self.environment.obstacles);

        let ee_pose = self.robot.end_effector_pose();
        let tcp_pos = self.robot.end_effector_position();
        let tcp_rot = ee_pose.rotation;
        self.workpieces
            .update(tcp_pos, tcp_rot, self.tool_state.is_gripping());

        // Handle Trajectory Playback
        if self.controller.is_playing {
            ctx.request_repaint();
            if let Some(target) = self.controller.update(dt as f64) {
                self.target_pos = target;
                self.execute_solve();
            }
        }

        // Handle Live Trajectory Recording
        if self.recorder.is_recording {
            let q = self.robot.get_actuated_joint_positions();
            self.recorder.maybe_record_frame(
                tcp_pos,
                self.target_rpy_deg,
                &q,
                self.tool_state.gripper_opening,
            );
        }

        // Handle Trajectory Recording Replay
        if self.recorder.is_replaying {
            ctx.request_repaint();
            if let Some(frame) = self.recorder.update_replay(dt as f64) {
                self.robot
                    .set_actuated_joint_positions(&frame.joint_positions);
                self.target_pos = frame.ee_position;
                self.target_rpy_deg = frame.ee_rpy_deg;
                self.tool_state.gripper_opening = frame.gripper_value;
                self.controller.add_trail_point(frame.ee_position);
            }
        }

        // Handle Active Smooth Motion Interpolation (Minimum-Jerk S-Curve)
        if let Some(mut motion) = self.active_smooth_motion.take() {
            ctx.request_repaint(); // 60 FPS continuous update!

            let mut finished = false;
            match &mut motion.target {
                SmoothMotionTarget::Cartesian {
                    start,
                    goal,
                    start_rpy,
                    goal_rpy,
                } => {
                    let elapsed = (Instant::now() - motion.start_time).as_secs_f64();
                    let u = (elapsed / motion.duration_secs).clamp(0.0, 1.0);
                    let s = 10.0 * u.powi(3) - 15.0 * u.powi(4) + 6.0 * u.powi(5);

                    self.target_pos = *start + (*goal - *start) * s;
                    self.target_rpy_deg = [
                        start_rpy[0] + (goal_rpy[0] - start_rpy[0]) * s,
                        start_rpy[1] + (goal_rpy[1] - start_rpy[1]) * s,
                        start_rpy[2] + (goal_rpy[2] - start_rpy[2]) * s,
                    ];
                    self.execute_solve();
                    if u >= 1.0 {
                        finished = true;
                    }
                }
                SmoothMotionTarget::Joints { start_q, goal_q } => {
                    let elapsed = (Instant::now() - motion.start_time).as_secs_f64();
                    let u = (elapsed / motion.duration_secs).clamp(0.0, 1.0);
                    let s = 10.0 * u.powi(3) - 15.0 * u.powi(4) + 6.0 * u.powi(5);

                    let mut curr_q = vec![0.0; start_q.len()];
                    for i in 0..start_q.len() {
                        curr_q[i] = start_q[i] + (goal_q[i] - start_q[i]) * s;
                    }
                    self.robot.set_actuated_joint_positions(&curr_q);
                    self.target_pos = self.robot.end_effector_position();
                    let (r, p, y) = self.robot.end_effector_pose().rotation.euler_angles();
                    self.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
                    self.controller.smoother.reset(&curr_q);
                    self.controller.add_trail_point(self.target_pos);
                    if u >= 1.0 {
                        finished = true;
                    }
                }
                SmoothMotionTarget::Gripper { start, goal } => {
                    let elapsed = (Instant::now() - motion.start_time).as_secs_f64();
                    let u = (elapsed / motion.duration_secs).clamp(0.0, 1.0);
                    let s = 10.0 * u.powi(3) - 15.0 * u.powi(4) + 6.0 * u.powi(5);
                    self.tool_state.gripper_opening = *start + (*goal - *start) * s as f32;
                    if u >= 1.0 {
                        finished = true;
                    }
                }
                SmoothMotionTarget::PickSequence {
                    stage,
                    stage_start,
                    stage_duration,
                    approach_pos,
                    grasp_pos,
                    lift_pos,
                } => {
                    let elapsed = (Instant::now() - *stage_start).as_secs_f64();
                    let u = (elapsed / *stage_duration).clamp(0.0, 1.0);
                    let s = 10.0 * u.powi(3) - 15.0 * u.powi(4) + 6.0 * u.powi(5);

                    match *stage {
                        0 => {
                            let start = self.target_pos;
                            self.target_pos = start + (*approach_pos - start) * s;
                            self.execute_solve();
                            if u >= 1.0 {
                                *stage = 1;
                                *stage_start = Instant::now();
                                *stage_duration = 0.7;
                            }
                        }
                        1 => {
                            self.target_pos = *approach_pos + (*grasp_pos - *approach_pos) * s;
                            self.execute_solve();
                            if u >= 1.0 {
                                *stage = 2;
                                *stage_start = Instant::now();
                                *stage_duration = 0.35;
                            }
                        }
                        2 => {
                            self.tool_state.gripper_opening = 0.85 + (0.05 - 0.85) * s as f32;
                            if u >= 1.0 {
                                self.tool_state.gripper_opening = 0.05;
                                *stage = 3;
                                *stage_start = Instant::now();
                                *stage_duration = 0.8;
                            }
                        }
                        3 => {
                            self.target_pos = *grasp_pos + (*lift_pos - *grasp_pos) * s;
                            self.execute_solve();
                            if u >= 1.0 {
                                finished = true;
                            }
                        }
                        _ => {
                            finished = true;
                        }
                    }
                }
            }

            if !finished {
                self.active_smooth_motion = Some(motion);
            }
        }

        // Top Menu Bar
        egui::TopBottomPanel::top("top_menu_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("Presets", |ui| {
                    if ui.button("2D Planar 3-DOF Arm").clicked() {
                        self.robot = presets::planar_3dof();
                        self.target_pos = self.robot.end_effector_position();
                        let (r, p, y) = self.robot.end_effector_pose().rotation.euler_angles();
                        self.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
                        self.controller.clear_trail();
                        ui.close_menu();
                    }
                    if ui.button("3D SCARA 4-DOF Robot").clicked() {
                        self.robot = presets::scara_4dof();
                        self.target_pos = self.robot.end_effector_position();
                        let (r, p, y) = self.robot.end_effector_pose().rotation.euler_angles();
                        self.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
                        self.controller.clear_trail();
                        ui.close_menu();
                    }
                    if ui.button("Industrial 6-DOF (UR5 style)").clicked() {
                        self.robot = presets::industrial_6dof();
                        self.target_pos = self.robot.end_effector_position();
                        let (r, p, y) = self.robot.end_effector_pose().rotation.euler_angles();
                        self.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
                        self.controller.clear_trail();
                        ui.close_menu();
                    }
                    if ui.button("Redundant 7-DOF (iiwa style)").clicked() {
                        self.robot = presets::redundant_7dof();
                        self.target_pos = self.robot.end_effector_position();
                        let (r, p, y) = self.robot.end_effector_pose().rotation.euler_angles();
                        self.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
                        self.controller.clear_trail();
                        ui.close_menu();
                    }
                });

                ui.menu_button("View", |ui| {
                    ui.checkbox(
                        &mut self.render_settings.show_solid_mesh,
                        "🦾 Solid 3D Robotic Arm",
                    );
                    ui.checkbox(
                        &mut self.render_settings.show_gripper,
                        "🖐️ Industrial EOAT Tool",
                    );
                    ui.checkbox(
                        &mut self.render_settings.show_rotation_gizmo,
                        "🔄 3D Rotation Gizmo",
                    );
                    ui.checkbox(
                        &mut self.render_settings.show_ellipsoid,
                        "🌐 Manipulability Ellipsoid",
                    );
                    ui.checkbox(
                        &mut self.render_settings.show_workcell_table,
                        "🏭 Workcell Table",
                    );
                    ui.checkbox(
                        &mut self.render_settings.show_workpieces,
                        "📦 Spawnable Workpieces",
                    );
                    ui.checkbox(
                        &mut self.render_settings.show_obstacles,
                        "🚧 Obstacle Barriers",
                    );
                    ui.separator();
                    ui.checkbox(&mut self.render_settings.show_grid, "Ground Grid");
                    ui.checkbox(&mut self.render_settings.show_axes, "Origin Axes");
                    ui.checkbox(&mut self.render_settings.show_trail, "Motion Trail");
                    ui.checkbox(
                        &mut self.render_settings.show_planned_path,
                        "Planned Spline Path",
                    );
                    ui.checkbox(&mut self.render_settings.show_joint_frames, "Joint Frames");
                    ui.checkbox(
                        &mut self.render_settings.show_reach_envelope,
                        "Reach Envelope",
                    );
                    ui.separator();
                    if ui.button("Reset Camera (H)").clicked() {
                        self.camera.reset();
                        ui.close_menu();
                    }
                    if ui.button("Focus on Tool Center Point (F)").clicked() {
                        let ee = self.robot.end_effector_position();
                        self.camera
                            .focus_on(Point3::new(ee.x as f32, ee.y as f32, ee.z as f32));
                        ui.close_menu();
                    }
                });

                ui.menu_button("Actions", |ui| {
                    if ui.button("Reset Robot to Home (R)").clicked() {
                        self.robot.reset_to_home();
                        self.target_pos = self.robot.end_effector_position();
                        let (r, p, y) = self.robot.end_effector_pose().rotation.euler_angles();
                        self.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
                        ui.close_menu();
                    }
                    if ui.button("Toggle Gripper (G)").clicked() {
                        self.tool_state.toggle_gripper();
                        ui.close_menu();
                    }
                    if ui.button("Toggle Vacuum (V)").clicked() {
                        self.tool_state.toggle_vacuum();
                        ui.close_menu();
                    }
                    if ui.button("Toggle Arc Welding (W)").clicked() {
                        self.tool_state.toggle_welding();
                        ui.close_menu();
                    }
                    if ui.button("Reset Demo Workpieces").clicked() {
                        self.workpieces.reset_demo_workpieces();
                        ui.close_menu();
                    }
                    if ui.button("Clear Motion Trail (C)").clicked() {
                        self.controller.clear_trail();
                        ui.close_menu();
                    }
                    if ui.button("Sync Target to End-Effector").clicked() {
                        self.target_pos = self.robot.end_effector_position();
                        let (r, p, y) = self.robot.end_effector_pose().rotation.euler_angles();
                        self.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
                        ui.close_menu();
                    }
                });

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("ℹ About").clicked() {
                        self.show_about_dialog = true;
                    }
                    ui.label(format!("FPS: {:.0}", self.fps));
                });
            });
        });

        // Bottom Status Bar
        egui::TopBottomPanel::bottom("bottom_status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("Robot: {}", self.robot.name));
                ui.separator();
                ui.label(format!("DOFs: {}", self.robot.dof()));
                ui.separator();
                let ee = self.robot.end_effector_position();
                ui.label(format!("EE: ({:.3}, {:.3}, {:.3})", ee.x, ee.y, ee.z));
                ui.separator();
                let dist = (self.target_pos - ee).norm();
                ui.label(format!("Dist: {:.3} mm", dist * 1000.0));
                ui.separator();
                if let Some(sol) = &self.last_solution {
                    if sol.converged {
                        ui.colored_label(Color32::from_rgb(50, 200, 70), "✔ Solved");
                    } else {
                        ui.colored_label(Color32::from_rgb(240, 70, 70), "✖ Incomplete");
                    }
                }
                ui.separator();
                // Manipulability index readout
                if manip_data.is_near_singularity {
                    ui.colored_label(
                        Color32::from_rgb(255, 60, 50),
                        format!("⚠ Singularity (μ={:.3})", manip_data.score),
                    );
                } else {
                    ui.colored_label(
                        Color32::from_rgb(40, 210, 180),
                        format!("μ: {:.3}", manip_data.score),
                    );
                }
                ui.separator();
                // Collision status
                if collision_report.in_collision {
                    ui.colored_label(Color32::from_rgb(255, 60, 50), "💥 COLLISION");
                } else {
                    ui.colored_label(Color32::from_rgb(50, 200, 70), "🛡️ Safe");
                }
                ui.separator();
                ui.toggle_value(&mut self.show_graphs, "📊 Graphs");

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label("Handles/Rings Drag | G Gripper | V Vacuum | W Weld | F Focus | Shift+Drag Pan");
                });
            });
        });

        // Left Control Panels
        egui::SidePanel::left("left_control_sidebar")
            .resizable(true)
            .default_width(340.0)
            .min_width(280.0)
            .max_width(500.0)
            .show(ctx, |ui| {
                // Tab Selection Bar
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.active_tab, AppTab::IK, "🎯 IK");
                    ui.selectable_value(&mut self.active_tab, AppTab::Robot, "🤖 Robot");
                    ui.selectable_value(&mut self.active_tab, AppTab::Chat, "💬 Chat");
                    ui.selectable_value(&mut self.active_tab, AppTab::Trajectory, "📈 Traj");
                    ui.selectable_value(&mut self.active_tab, AppTab::Manipulation, "🖐️ Cell");
                    ui.selectable_value(&mut self.active_tab, AppTab::URDF, "📄 URDF");
                });
                ui.separator();

                let mut on_preset = None;
                let mut solve_requested = false;
                let mut robot_loaded = None;

                if self.active_tab == AppTab::Chat {
                    super::chat_command::render_chat_panel(ui, self);
                } else {
                    egui::ScrollArea::vertical().show(ui, |ui| match self.active_tab {
                        AppTab::IK => {
                            render_ik_panel(
                                ui,
                                &mut self.target_pos,
                                &mut self.target_rpy_deg,
                                &mut self.solver_type,
                                &mut self.solver_mode,
                                &mut self.solver_params,
                                &mut self.continuous_solve,
                                &self.last_solution,
                                &mut solve_requested,
                                Some(&manip_data),
                                &mut self.render_settings.show_ellipsoid,
                                &mut self.render_settings.show_rotation_gizmo,
                            );
                        }
                        AppTab::Robot => {
                            render_robot_panel(
                                ui,
                                &mut self.robot,
                                &mut on_preset,
                                &dynamics_report,
                                &mut self.payload_mass_kg,
                                &mut self.target_pos,
                                &mut self.target_rpy_deg,
                                &mut self.controller.smoother,
                                &mut self.show_graphs,
                            );
                        }
                        AppTab::Trajectory => {
                            let mut export_req = None;
                            let mut import_json_req = None;
                            render_trajectory_panel(
                                ui,
                                &mut self.controller,
                                &mut self.recorder,
                                &self.robot,
                                &self.environment,
                                &mut self.rrt_params,
                                &mut self.rrt_goal_pos,
                                &mut self.last_rrt_result,
                                self.target_pos,
                                &mut export_req,
                                &mut import_json_req,
                            );

                            if let Some(json_content) = import_json_req {
                                match crate::export::import_from_json(&json_content) {
                                    Ok((wps, rec)) => {
                                        if !wps.is_empty() {
                                            self.controller.planner.waypoints = wps;
                                            self.controller.planner.rebuild_trajectory();
                                        }
                                        if let Some(r) = rec {
                                            self.recorder.recording = r;
                                        }
                                        self.chat.add_robot_reply(
                                            "Trajectory JSON imported successfully!",
                                        );
                                    }
                                    Err(e) => {
                                        self.chat.add_system_error(&format!(
                                            "Trajectory import failed: {}",
                                            e
                                        ));
                                    }
                                }
                            }

                            if let Some(fmt) = export_req {
                                match fmt {
                                    "python" => {
                                        let code = crate::export::export_to_python(
                                            &self.robot,
                                            &self.controller.planner,
                                        );
                                        self.export_modal = Some((
                                            "Python Trajectory Script (NumPy / Matplotlib)"
                                                .to_string(),
                                            code,
                                        ));
                                    }
                                    "ros2" => {
                                        let code = crate::export::export_to_ros2(
                                            &self.robot,
                                            &self.controller.planner,
                                        );
                                        self.export_modal = Some((
                                            "ROS 2 JointTrajectory Action YAML".to_string(),
                                            code,
                                        ));
                                    }
                                    "gcode" => {
                                        let code = crate::export::export_to_gcode_csv(
                                            &self.robot,
                                            &self.controller.planner,
                                        );
                                        self.export_modal = Some((
                                            "CNC G-Code & CSV Time-Series Output".to_string(),
                                            code,
                                        ));
                                    }
                                    "json" => {
                                        let code = crate::export::export_to_json(
                                            &self.robot,
                                            &self.controller.planner,
                                            Some(&self.recorder.recording),
                                        );
                                        self.export_modal = Some((
                                            "Trajectory Session JSON Backup".to_string(),
                                            code,
                                        ));
                                    }
                                    "arduino" => {
                                        let code = crate::export::export_to_arduino_cpp(
                                            &self.robot,
                                            &self.controller.planner,
                                            Some(&self.recorder.recording),
                                        );
                                        self.export_modal = Some((
                                            "Arduino / ESP32 C++ Motion Controller".to_string(),
                                            code,
                                        ));
                                    }
                                    "matlab" => {
                                        let code = crate::export::export_to_matlab(
                                            &self.robot,
                                            &self.controller.planner,
                                            Some(&self.recorder.recording),
                                        );
                                        self.export_modal = Some((
                                            "MATLAB Trajectory Simulation & Plotting".to_string(),
                                            code,
                                        ));
                                    }
                                    "csv_recorded" => {
                                        let code = crate::export::export_recorded_to_csv(
                                            &self.recorder.recording,
                                        );
                                        self.export_modal = Some((
                                            "Recorded Motion CSV Time-Series".to_string(),
                                            code,
                                        ));
                                    }
                                    _ => {}
                                }
                            }
                        }
                        AppTab::Manipulation => {
                            render_manipulation_panel(
                                ui,
                                &mut self.tool_state,
                                &mut self.workpieces,
                                &mut self.environment,
                                &collision_report,
                                &mut self.target_pos,
                                &mut solve_requested,
                            );
                        }
                        AppTab::URDF => {
                            render_urdf_panel(
                                ui,
                                &mut self.urdf_editor_text,
                                &mut self.urdf_error,
                                &mut robot_loaded,
                                &self.robot,
                            );
                        }
                        AppTab::Chat => unreachable!(),
                    });
                }

                if let Some(p) = on_preset {
                    self.robot = p;
                    self.target_pos = self.robot.end_effector_position();
                    let (r, p_ang, y) = self.robot.end_effector_pose().rotation.euler_angles();
                    self.target_rpy_deg = [r.to_degrees(), p_ang.to_degrees(), y.to_degrees()];
                    self.controller.clear_trail();
                }

                if let Some(new_bot) = robot_loaded {
                    self.robot = new_bot;
                    self.target_pos = self.robot.end_effector_position();
                    let (r, p_ang, y) = self.robot.end_effector_pose().rotation.euler_angles();
                    self.target_rpy_deg = [r.to_degrees(), p_ang.to_degrees(), y.to_degrees()];
                    self.controller.clear_trail();
                }

                if solve_requested {
                    self.execute_solve();
                }
            });

        // Center 3D Interactive Canvas
        egui::CentralPanel::default().show(ctx, |ui| {
            let available_rect = ui.available_rect_before_wrap();
            let response = ui.allocate_rect(available_rect, Sense::click_and_drag());

            // Handle Camera Navigation
            if response.dragged_by(PointerButton::Secondary) {
                self.camera.rotate(response.drag_delta());
            } else if response.dragged_by(PointerButton::Middle)
                || (response.dragged_by(PointerButton::Primary) && ui.input(|i| i.modifiers.shift))
            {
                self.camera.pan(response.drag_delta());
            }

            // Scroll Zoom
            let scroll_delta = ui.input(|i| i.raw_scroll_delta.y);
            if scroll_delta != 0.0 && response.hovered() {
                self.camera.zoom(scroll_delta);
            }

            // Handle Gizmo Dragging & 3D Render
            let hovered_axis = render_scene_3d(
                ui,
                available_rect,
                &self.camera,
                &self.robot,
                self.target_pos,
                &self.controller.trail,
                &self.controller.planner,
                &self.render_settings,
                self.active_drag_axis,
                &self.tool_state,
                &self.workpieces,
                &self.environment,
                &collision_report,
                Some(&manip_data),
            );

            if response.drag_started_by(PointerButton::Primary) && !ui.input(|i| i.modifiers.shift)
            {
                if let Some(axis) = hovered_axis {
                    self.active_drag_axis = axis;
                    self.drag_start_mouse = response.interact_pointer_pos();
                    self.drag_start_target = self.target_pos;
                    self.drag_start_rpy = self.target_rpy_deg;
                    if matches!(
                        axis,
                        GizmoDragAxis::RotRoll | GizmoDragAxis::RotPitch | GizmoDragAxis::RotYaw
                    ) {
                        self.solver_mode = IKSolverMode::FullPose;
                    }
                } else {
                    // Empty canvas drag: rotate camera
                    self.active_drag_axis = GizmoDragAxis::None;
                }
            }

            if response.dragged_by(PointerButton::Primary) && !ui.input(|i| i.modifiers.shift) {
                if self.active_drag_axis == GizmoDragAxis::None {
                    // Canvas drag rotates camera
                    self.camera.rotate(response.drag_delta());
                } else if let Some(start_mouse) = self.drag_start_mouse {
                    let curr_mouse = response.interact_pointer_pos().unwrap_or(start_mouse);
                    let delta_screen = curr_mouse - start_mouse;

                    let origin_f32 = Point3::new(
                        self.drag_start_target.x as f32,
                        self.drag_start_target.y as f32,
                        self.drag_start_target.z as f32,
                    );
                    let depth =
                        if let Some((_, d)) = self.camera.project(origin_f32, available_rect) {
                            d.max(0.1)
                        } else {
                            self.camera.distance
                        };

                    match self.active_drag_axis {
                        GizmoDragAxis::AxisX => {
                            let delta_m = self.camera.project_axis_delta(
                                origin_f32,
                                Vector3::x(),
                                delta_screen,
                                available_rect,
                            );
                            let mut new_pos = self.drag_start_target;
                            new_pos.x += delta_m as f64;
                            self.target_pos = new_pos;
                        }
                        GizmoDragAxis::AxisY => {
                            let delta_m = self.camera.project_axis_delta(
                                origin_f32,
                                Vector3::y(),
                                delta_screen,
                                available_rect,
                            );
                            let mut new_pos = self.drag_start_target;
                            new_pos.y += delta_m as f64;
                            self.target_pos = new_pos;
                        }
                        GizmoDragAxis::AxisZ => {
                            let delta_m = self.camera.project_axis_delta(
                                origin_f32,
                                Vector3::z(),
                                delta_screen,
                                available_rect,
                            );
                            let mut new_pos = self.drag_start_target;
                            new_pos.z += delta_m as f64;
                            self.target_pos = new_pos;
                        }
                        GizmoDragAxis::TargetCenter => {
                            // Direct View-Plane dragging: hand tracks cursor in 3D 1:1!
                            let world_disp = self.camera.screen_delta_to_world(
                                delta_screen,
                                depth,
                                available_rect,
                            );
                            let mut new_pos = self.drag_start_target;
                            new_pos.x += world_disp.x as f64;
                            new_pos.y += world_disp.y as f64;
                            new_pos.z += world_disp.z as f64;
                            self.target_pos = new_pos;
                        }
                        GizmoDragAxis::RotRoll => {
                            let angle_deg = self.camera.project_ring_angle(
                                origin_f32,
                                start_mouse,
                                curr_mouse,
                                available_rect,
                            );
                            self.target_rpy_deg[0] = self.drag_start_rpy[0] + angle_deg as f64;
                        }
                        GizmoDragAxis::RotPitch => {
                            let angle_deg = self.camera.project_ring_angle(
                                origin_f32,
                                start_mouse,
                                curr_mouse,
                                available_rect,
                            );
                            self.target_rpy_deg[1] = self.drag_start_rpy[1] + angle_deg as f64;
                        }
                        GizmoDragAxis::RotYaw => {
                            let angle_deg = self.camera.project_ring_angle(
                                origin_f32,
                                start_mouse,
                                curr_mouse,
                                available_rect,
                            );
                            self.target_rpy_deg[2] = self.drag_start_rpy[2] + angle_deg as f64;
                        }
                        GizmoDragAxis::None => {}
                    }

                    if self.continuous_solve {
                        self.execute_solve();
                    }
                }
            }

            if response.drag_stopped() {
                if self.active_drag_axis != GizmoDragAxis::None && !self.continuous_solve {
                    self.execute_solve();
                }
                self.active_drag_axis = GizmoDragAxis::None;
                self.drag_start_mouse = None;
            }

            // Keyboard Shortcuts & Hand Jogging (only active when not typing into text boxes)
            if !ctx.wants_keyboard_input() {
                if ui.input(|i| i.key_pressed(Key::Space)) {
                    self.controller.toggle_play();
                }
                if ui.input(|i| i.key_pressed(Key::R)) {
                    self.robot.reset_to_home();
                    self.target_pos = self.robot.end_effector_position();
                    let (r, p, y) = self.robot.end_effector_pose().rotation.euler_angles();
                    self.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
                }
                if ui.input(|i| i.key_pressed(Key::H)) {
                    self.camera.reset();
                }
                if ui.input(|i| i.key_pressed(Key::F)) {
                    let ee = self.robot.end_effector_position();
                    self.camera
                        .focus_on(Point3::new(ee.x as f32, ee.y as f32, ee.z as f32));
                }
                if ui.input(|i| i.key_pressed(Key::G)) {
                    self.tool_state.toggle_gripper();
                }
                if ui.input(|i| i.key_pressed(Key::V)) {
                    self.tool_state.toggle_vacuum();
                }
                if ui.input(|i| i.key_pressed(Key::W)) {
                    self.tool_state.toggle_welding();
                }
                if ui.input(|i| i.key_pressed(Key::C)) {
                    self.controller.clear_trail();
                }

                // Keyboard Hand Jogging (Arrow keys: Left/Right = X, Up/Down = Y, PageUp/PageDown = Z)
                let step = if ui.input(|i| i.modifiers.shift) {
                    0.05
                } else {
                    0.015
                };
                let mut jogged = false;
                if ui.input(|i| i.key_pressed(Key::ArrowLeft)) {
                    self.target_pos.x -= step;
                    jogged = true;
                }
                if ui.input(|i| i.key_pressed(Key::ArrowRight)) {
                    self.target_pos.x += step;
                    jogged = true;
                }
                if ui.input(|i| i.key_pressed(Key::ArrowUp)) {
                    self.target_pos.y += step;
                    jogged = true;
                }
                if ui.input(|i| i.key_pressed(Key::ArrowDown)) {
                    self.target_pos.y -= step;
                    jogged = true;
                }
                if ui.input(|i| i.key_pressed(Key::PageUp)) {
                    self.target_pos.z += step;
                    jogged = true;
                }
                if ui.input(|i| i.key_pressed(Key::PageDown)) {
                    self.target_pos.z -= step;
                    jogged = true;
                }
                if ui.input(|i| i.key_pressed(Key::OpenBracket)) {
                    self.tool_state.gripper_opening = 0.05; // '[' close hand
                }
                if ui.input(|i| i.key_pressed(Key::CloseBracket)) {
                    self.tool_state.gripper_opening = 0.85; // ']' open hand
                }
                if jogged {
                    self.execute_solve();
                }
            }

            // Quick Command HUD overlay at the bottom of the 3D scene
            super::chat_command::render_quick_command_hud(ui, self);
        });

        // Code Export Modal Window
        if let Some((title, ref mut code)) = &mut self.export_modal {
            let mut close = false;
            egui::Window::new(title.as_str())
                .collapsible(false)
                .resizable(true)
                .default_size([650.0, 480.0])
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        if ui.button("📋 Copy to Clipboard").clicked() {
                            ui.output_mut(|o| o.copied_text = code.clone());
                        }
                        if ui.button("Close").clicked() {
                            close = true;
                        }
                    });
                    ui.separator();
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        ui.add(
                            egui::TextEdit::multiline(code)
                                .font(egui::TextStyle::Monospace)
                                .code_editor()
                                .desired_width(f32::INFINITY),
                        );
                    });
                });
            if close {
                self.export_modal = None;
            }
        }

        // About / Info Window
        if self.show_about_dialog {
            egui::Window::new("About kine-rs")
                .collapsible(false)
                .resizable(false)
                .show(ctx, |ui| {
                    ui.heading("kine-rs: Robotic Arm Kinematics & Manipulation Simulator");
                    ui.label("A high-performance 2D/3D robotics simulator built in pure Rust.");
                    ui.separator();
                    ui.label("• Forward & Inverse Kinematics (FK / IK with DLS, Transpose, and FABRIK)");
                    ui.label("• 3D Rotation Gizmo Rings & Manipulability Ellipsoid (Yoshikawa Index)");
                    ui.label("• Interactive EOAT Tooling (Parallel Gripper, Vacuum Cup, Arc Welding Torch)");
                    ui.label("• Workcell Environment & Physics-based Pick-and-Place Manipulation");
                    ui.label("• Real-time Dynamics & Motor Strain Gravity Torque Estimation");
                    ui.label("• Capsule-to-Obstacle & Self-Collision Detection");
                    ui.label("• Minimum-Jerk Quintic & Cubic Trajectory Planning");
                    ui.label("• Code Generation for Python (NumPy/Matplotlib), ROS 2, and CNC G-Code");
                    ui.label("• Drag-and-Drop URDF XML Importer & Exporter");
                    ui.separator();
                    if ui.button("Close").clicked() {
                        self.show_about_dialog = false;
                    }
                });
        }
    }
}
