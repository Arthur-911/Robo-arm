use egui::{Color32, Key, ScrollArea, TextEdit, Ui};
use nalgebra::{Point3, Vector3};

use crate::kinematics::{IKSolverMode, IKSolverType};
use crate::presets;
use crate::trajectory::TrajectoryContinuityMode;
use crate::ui::app::RoboSimApp;

/// Who sent the chat message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChatSender {
    User,
    Robot,
    System,
}

/// A recorded chat / command message.
#[derive(Debug, Clone)]
pub struct ChatMessage {
    pub sender: ChatSender,
    pub text: String,
    pub is_error: bool,
}

/// Interactive Chat & Robotic Command Console State.
#[derive(Debug, Clone)]
pub struct ChatCommandConsole {
    pub messages: Vec<ChatMessage>,
    pub input_text: String,
    pub command_history: Vec<String>,
    pub history_idx: Option<usize>,
    pub is_floating_open: bool,
    pub jog_step_deg: f64,
}

impl Default for ChatCommandConsole {
    fn default() -> Self {
        let mut console = Self {
            messages: Vec::new(),
            input_text: String::new(),
            command_history: Vec::new(),
            history_idx: None,
            is_floating_open: true,
            jog_step_deg: 5.0,
        };

        console.messages.push(ChatMessage {
            sender: ChatSender::Robot,
            text: "🤖 RoboSim AI Copilot online! Type commands like 'move x 0.4 y 0.2 z 0.3', 'joint 1 45', 'grip', 'wave', or 'help' to command the robotic arm.".to_string(),
            is_error: false,
        });

        console
    }
}

impl ChatCommandConsole {
    pub fn add_user_message(&mut self, text: &str) {
        self.messages.push(ChatMessage {
            sender: ChatSender::User,
            text: text.to_string(),
            is_error: false,
        });
    }

    pub fn add_robot_reply(&mut self, text: &str) {
        self.messages.push(ChatMessage {
            sender: ChatSender::Robot,
            text: text.to_string(),
            is_error: false,
        });
    }

    pub fn add_system_error(&mut self, text: &str) {
        self.messages.push(ChatMessage {
            sender: ChatSender::System,
            text: text.to_string(),
            is_error: true,
        });
    }

    pub fn clear(&mut self) {
        self.messages.clear();
        self.messages.push(ChatMessage {
            sender: ChatSender::System,
            text: "Console cleared. Ready for commands.".to_string(),
            is_error: false,
        });
    }

    /// History navigation: Previous command (<kbd>↑</kbd>)
    pub fn history_prev(&mut self) {
        if self.command_history.is_empty() {
            return;
        }
        let next_idx = match self.history_idx {
            None => self.command_history.len().saturating_sub(1),
            Some(idx) => idx.saturating_sub(1),
        };
        self.history_idx = Some(next_idx);
        if let Some(cmd) = self.command_history.get(next_idx) {
            self.input_text = cmd.clone();
        }
    }

    /// History navigation: Next command (<kbd>↓</kbd>)
    pub fn history_next(&mut self) {
        if let Some(idx) = self.history_idx {
            if idx + 1 < self.command_history.len() {
                self.history_idx = Some(idx + 1);
                self.input_text = self.command_history[idx + 1].clone();
            } else {
                self.history_idx = None;
                self.input_text.clear();
            }
        }
    }
}

/// Executes a natural language or structured robotic command against the application state.
pub fn execute_chat_command(raw_input: &str, app: &mut RoboSimApp) -> (String, bool) {
    let input = raw_input.trim();
    if input.is_empty() {
        return ("Empty command.".to_string(), true);
    }

    let lower = input.to_lowercase();
    let tokens: Vec<&str> = input.split_whitespace().collect();
    let lower_tokens: Vec<String> = tokens.iter().map(|s| s.to_lowercase()).collect();

    // 1. HELP & COMMANDS
    if lower == "help" || lower == "commands" || lower == "?" || lower == "/help" {
        return (
            "📖 Available Robotic Commands:\n\
            • move x <val> y <val> z <val> : Move end-effector (meters)\n\
            • goto <x> <y> <z>             : Quick 3D target coordinates\n\
            • up / down / left / right <d> : Relative tool jog (meters)\n\
            • joint <num> <angle>          : Set joint angle (e.g. 'joint 1 45' in deg)\n\
            • joints <a1>, <a2>, <a3>...   : Set all joint angles simultaneously\n\
            • jog j<num> <+/-deg>          : Increment joint angle (e.g. 'jog j2 +10')\n\
            • roll / pitch / yaw <deg>     : Set target orientation angle\n\
            • grip / grab / clamp          : Close mechanical gripper\n\
            • release / open / drop        : Open mechanical gripper\n\
            • vacuum on / off              : Toggle suction tool\n\
            • weld on / off                : Toggle electric arc welding tool\n\
            • pick <red|blue|gold|1|2|3>   : Auto pick up a workcell workpiece\n\
            • wave / nod / dance / circle  : Run scripted smooth procedural animations\n\
            • home / reset                 : Reset robot to home configuration\n\
            • zero                         : Set all joint angles to 0°\n\
            • preset <6dof|scara|7dof|2d>  : Switch robot kinematic model\n\
            • play / pause / stop / speed  : Trajectory playback controller\n\
            • status                       : Show current coordinates & joint angles\n\
            • clear                        : Clear chat message log".to_string(),
            false,
        );
    }

    // 2. STATUS & REPORT
    if lower == "status" || lower == "info" || lower == "pos" {
        let ee = app.robot.end_effector_position();
        let (r, p, y) = app.robot.end_effector_pose().rotation.euler_angles();
        let joints = app.robot.get_actuated_joint_positions();
        let mut joint_str = String::new();
        for (i, q) in joints.iter().enumerate() {
            if i > 0 {
                joint_str.push_str(", ");
            }
            joint_str.push_str(&format!("J{}:{:.1}°", i + 1, q.to_degrees()));
        }
        return (
            format!(
                "🤖 Robot '{}' ({} DOFs)\n\
                📍 TCP Position : X={:.4}m, Y={:.4}m, Z={:.4}m\n\
                🔄 Orientation  : Roll={:.1}°, Pitch={:.1}°, Yaw={:.1}°\n\
                🦾 Joints       : [{}]\n\
                🖐️ Tool State   : Gripper={:.0}%, Vacuum={}, Weld={}",
                app.robot.name,
                app.robot.dof(),
                ee.x, ee.y, ee.z,
                r.to_degrees(), p.to_degrees(), y.to_degrees(),
                joint_str,
                (1.0 - app.tool_state.gripper_opening) * 100.0,
                if app.tool_state.is_vacuum_active { "ON" } else { "OFF" },
                if app.tool_state.is_welding_active { "ON" } else { "OFF" }
            ),
            false,
        );
    }

    // 3. HOME & ZERO & SYNC
    if lower == "home" || lower == "reset" || lower == "reset home" {
        let home_q: Vec<f64> = app
            .robot
            .joints
            .iter()
            .filter(|j| j.is_actuated())
            .map(|j| j.home_position)
            .collect();
        app.start_smooth_joint_move(home_q, 1.2);
        app.controller.clear_trail();
        return ("🏠 Smoothly gliding robot to home configuration...".to_string(), false);
    }

    if lower == "zero" || lower == "zero joints" {
        let zero_q = vec![0.0; app.robot.dof()];
        app.start_smooth_joint_move(zero_q, 1.2);
        app.controller.clear_trail();
        return ("🔄 Smoothly driving all joints to 0.0°...".to_string(), false);
    }

    if lower == "sync" || lower == "sync target" {
        app.target_pos = app.robot.end_effector_position();
        let (r, p, y) = app.robot.end_effector_pose().rotation.euler_angles();
        app.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
        return ("🎯 Target synced to end-effector pose.".to_string(), false);
    }

    if lower == "clear trail" || lower == "reset trail" {
        app.controller.clear_trail();
        return ("🧹 Motion trail cleared.".to_string(), false);
    }

    // 4. TOOL & GRIPPER CONTROLS
    if lower == "grip" || lower == "grab" || lower == "close" || lower == "close gripper" || lower == "clamp" {
        app.start_smooth_gripper_move(0.05, 0.35);
        return ("🖐️ Gripper jaws closing smoothly...".to_string(), false);
    }

    if lower == "release" || lower == "open" || lower == "open gripper" || lower == "drop" {
        app.start_smooth_gripper_move(0.85, 0.35);
        return ("✋ Gripper jaws opening smoothly...".to_string(), false);
    }

    if lower.starts_with("gripper") {
        if let Some(val_str) = lower_tokens.get(1) {
            let clean = val_str.trim_end_matches('%');
            if let Ok(v) = clean.parse::<f64>() {
                let ratio = if v > 1.0 { (v / 100.0).clamp(0.0, 1.0) } else { v.clamp(0.0, 1.0) };
                let target_opening = (1.0 - ratio) as f32;
                app.start_smooth_gripper_move(target_opening, 0.35);
                return (format!("🖐️ Gripper opening smoothly transitioning to {:.0}%.", ratio * 100.0), false);
            }
        }
    }

    if lower == "vacuum on" || lower == "vacuum" {
        app.tool_state.is_vacuum_active = true;
        return ("💨 Vacuum suction activated.".to_string(), false);
    }
    if lower == "vacuum off" {
        app.tool_state.is_vacuum_active = false;
        return ("💨 Vacuum suction deactivated.".to_string(), false);
    }

    if lower == "weld on" || lower == "weld" {
        app.tool_state.is_welding_active = true;
        return ("⚡ Arc welding torch activated.".to_string(), false);
    }
    if lower == "weld off" {
        app.tool_state.is_welding_active = false;
        return ("⚡ Arc welding torch deactivated.".to_string(), false);
    }

    // 5. PRESETS
    if lower.starts_with("preset") {
        let name = lower_tokens.get(1).map(|s| s.as_str()).unwrap_or("");
        let new_robot = if name.contains("6dof") || name.contains("industrial") || name.contains("ur5") {
            Some(presets::industrial_6dof())
        } else if name.contains("scara") || name.contains("4dof") {
            Some(presets::scara_4dof())
        } else if name.contains("7dof") || name.contains("redundant") || name.contains("iiwa") {
            Some(presets::redundant_7dof())
        } else if name.contains("planar") || name.contains("2d") || name.contains("3dof") {
            Some(presets::planar_3dof())
        } else {
            None
        };

        if let Some(bot) = new_robot {
            let bot_name = bot.name.clone();
            app.robot = bot;
            app.target_pos = app.robot.end_effector_position();
            let (r, p, y) = app.robot.end_effector_pose().rotation.euler_angles();
            app.target_rpy_deg = [r.to_degrees(), p.to_degrees(), y.to_degrees()];
            app.controller.clear_trail();
            app.controller.smoother.reset(&app.robot.get_actuated_joint_positions());
            return (format!("🤖 Loaded robot preset: '{}'.", bot_name), false);
        } else {
            return ("Usage: preset <industrial|scara|7dof|planar>".to_string(), true);
        }
    }

    // 6. PROCEDURAL DEMO ANIMATIONS (WAVE, NOD, DANCE, CIRCLE)
    if lower == "wave" {
        app.controller.planner.clear_waypoints();
        app.controller.planner.set_continuity_mode(TrajectoryContinuityMode::SmoothContinuous);
        let center = app.robot.end_effector_position();
        app.controller.planner.add_waypoint("Center", center, 1.0);
        app.controller.planner.add_waypoint("Wave Left", center + Vector3::new(0.0, 0.12, 0.05), 0.7);
        app.controller.planner.add_waypoint("Wave Right", center + Vector3::new(0.0, -0.12, 0.05), 0.7);
        app.controller.planner.add_waypoint("Wave Left 2", center + Vector3::new(0.0, 0.12, 0.05), 0.7);
        app.controller.planner.add_waypoint("Return", center, 0.8);
        app.controller.is_looping = false;
        app.controller.reset();
        app.controller.play();
        return ("👋 Executing smooth robotic wave greeting!".to_string(), false);
    }

    if lower == "nod" {
        app.controller.planner.clear_waypoints();
        app.controller.planner.set_continuity_mode(TrajectoryContinuityMode::SmoothContinuous);
        let center = app.robot.end_effector_position();
        app.controller.planner.add_waypoint("Start", center, 0.8);
        app.controller.planner.add_waypoint("Nod Down 1", center + Vector3::new(0.0, 0.0, -0.08), 0.6);
        app.controller.planner.add_waypoint("Nod Up 1", center + Vector3::new(0.0, 0.0, 0.04), 0.6);
        app.controller.planner.add_waypoint("Nod Down 2", center + Vector3::new(0.0, 0.0, -0.08), 0.6);
        app.controller.planner.add_waypoint("Center", center, 0.7);
        app.controller.is_looping = false;
        app.controller.reset();
        app.controller.play();
        return ("🤖 Executing robotic nod gesture.".to_string(), false);
    }

    if lower == "dance" {
        app.controller.planner.clear_waypoints();
        app.controller.planner.set_continuity_mode(TrajectoryContinuityMode::SmoothContinuous);
        let c = app.robot.end_effector_position();
        let r = 0.10;
        app.controller.planner.add_waypoint("P0", c, 1.0);
        app.controller.planner.add_waypoint("P1", c + Vector3::new(r, r, 0.05), 0.8);
        app.controller.planner.add_waypoint("P2", c + Vector3::new(0.0, 0.0, -0.05), 0.8);
        app.controller.planner.add_waypoint("P3", c + Vector3::new(-r, -r, 0.05), 0.8);
        app.controller.planner.add_waypoint("P4", c + Vector3::new(0.0, 0.0, -0.05), 0.8);
        app.controller.planner.add_waypoint("P5", c + Vector3::new(r, -r, 0.05), 0.8);
        app.controller.planner.add_waypoint("P6", c, 0.9);
        app.controller.is_looping = true;
        app.controller.reset();
        app.controller.play();
        return ("✨ Playing multi-axis smooth figure-8 dance routine!".to_string(), false);
    }

    // 7. TRAJECTORY PLAYBACK
    if lower == "play" || lower == "start" {
        app.controller.play();
        return ("▶ Trajectory playback started.".to_string(), false);
    }
    if lower == "pause" {
        app.controller.pause();
        return ("⏸ Trajectory playback paused.".to_string(), false);
    }
    if lower == "stop" {
        app.controller.reset();
        return ("⏹ Trajectory playback stopped & reset.".to_string(), false);
    }
    if lower == "loop on" {
        app.controller.is_looping = true;
        return ("🔁 Trajectory looping enabled.".to_string(), false);
    }
    if lower == "loop off" {
        app.controller.is_looping = false;
        return ("➡️ Trajectory looping disabled.".to_string(), false);
    }
    if lower.starts_with("speed") {
        if let Some(val_str) = lower_tokens.get(1) {
            let clean = val_str.trim_end_matches('x');
            if let Ok(s) = clean.parse::<f64>() {
                app.controller.speed_multiplier = s.clamp(0.1, 10.0);
                return (format!("⚡ Trajectory speed multiplier set to {:.2}x.", app.controller.speed_multiplier), false);
            }
        }
    }

    // 8. DIRECT JOINT COMMANDS: "joint 1 45", "j2 -30", "joints 0, 45, -90"
    if lower.starts_with("joint ") || lower.starts_with("j") && lower.chars().nth(1).map(|c| c.is_ascii_digit()).unwrap_or(false) {
        // e.g. "joint 1 45" or "j1 45"
        let (joint_num, val_str): (Option<&str>, Option<&str>) = if lower.starts_with("joint ") {
            (
                lower_tokens.get(1).map(|s| s.as_str()),
                lower_tokens.get(2).map(|s| s.as_str()),
            )
        } else {
            // "j1 45"
            let num_str = lower_tokens[0].trim_start_matches('j');
            (Some(num_str), lower_tokens.get(1).map(|s| s.as_str()))
        };

        if let (Some(num_s), Some(val_s)) = (joint_num, val_str) {
            if let (Ok(j_idx), Ok(angle_val)) = (
                num_s.parse::<usize>(),
                val_s
                    .trim_end_matches("deg")
                    .trim_end_matches('°')
                    .parse::<f64>(),
            ) {
                let actuated_joints: Vec<usize> = app.robot.joints.iter().enumerate().filter(|(_, j)| j.is_actuated()).map(|(i, _)| i).collect();
                if j_idx >= 1 && j_idx <= actuated_joints.len() {
                    let actual_idx = actuated_joints[j_idx - 1];
                    let is_prismatic = app.robot.joints[actual_idx].joint_type == crate::kinematics::JointType::Prismatic;
                    
                    let target_rad_or_m = if is_prismatic {
                        angle_val
                    } else if val_s.ends_with("rad") {
                        angle_val
                    } else {
                        angle_val.to_radians()
                    };

                    let old_pos = app.robot.joints[actual_idx].current_position;
                    let mut goal_q = app.robot.get_actuated_joint_positions();
                    goal_q[j_idx - 1] = target_rad_or_m;
                    app.start_smooth_joint_move(goal_q, 1.0);

                    let unit_str = if is_prismatic { "m" } else { "°" };
                    let display_val = if is_prismatic {
                        target_rad_or_m
                    } else {
                        target_rad_or_m.to_degrees()
                    };
                    return (
                        format!(
                            "🦾 Joint {} ('{}') smoothly gliding from {:.1}{} to {:.1}{}...",
                            j_idx,
                            app.robot.joints[actual_idx].name,
                            if is_prismatic {
                                old_pos
                            } else {
                                old_pos.to_degrees()
                            },
                            unit_str,
                            display_val,
                            unit_str
                        ),
                        false,
                    );
                } else {
                    return (format!("Joint index {} out of range (1..={}).", j_idx, actuated_joints.len()), true);
                }
            }
        }
    }

    // 8b. RELATIVE JOINT JOG: "jog j1 +10", "jog 2 -5", "jog j3 15deg"
    if lower.starts_with("jog j")
        || (lower.starts_with("jog ")
            && lower_tokens
                .get(1)
                .map(|s| s.starts_with('j') || s.chars().all(|c| c.is_ascii_digit()))
                .unwrap_or(false))
    {
        let (joint_num, val_str) = if lower_tokens
            .get(1)
            .map(|s| s.starts_with('j'))
            .unwrap_or(false)
        {
            let num_s = lower_tokens[1].trim_start_matches('j');
            (Some(num_s), lower_tokens.get(2).map(|s| s.as_str()))
        } else {
            (
                lower_tokens.get(1).map(|s| s.as_str()),
                lower_tokens.get(2).map(|s| s.as_str()),
            )
        };

        if let (Some(num_s), Some(val_s)) = (joint_num, val_str) {
            if let (Ok(j_idx), Ok(delta_val)) = (
                num_s.parse::<usize>(),
                val_s
                    .trim_end_matches("deg")
                    .trim_end_matches('°')
                    .parse::<f64>(),
            ) {
                let actuated_joints: Vec<usize> = app
                    .robot
                    .joints
                    .iter()
                    .enumerate()
                    .filter(|(_, j)| j.is_actuated())
                    .map(|(i, _)| i)
                    .collect();
                if j_idx >= 1 && j_idx <= actuated_joints.len() {
                    let actual_idx = actuated_joints[j_idx - 1];
                    let is_prismatic = app.robot.joints[actual_idx].joint_type
                        == crate::kinematics::JointType::Prismatic;
                    let delta = if is_prismatic {
                        delta_val
                    } else if val_s.ends_with("rad") {
                        delta_val
                    } else {
                        delta_val.to_radians()
                    };

                    let old_pos = app.robot.joints[actual_idx].current_position;
                    let target_pos_val = old_pos + delta;
                    let mut goal_q = app.robot.get_actuated_joint_positions();
                    goal_q[j_idx - 1] = target_pos_val;
                    app.start_smooth_joint_move(goal_q, 0.7);

                    let unit_str = if is_prismatic { "m" } else { "°" };
                    let display_val = if is_prismatic {
                        target_pos_val
                    } else {
                        target_pos_val.to_degrees()
                    };
                    let delta_disp = if is_prismatic {
                        delta
                    } else {
                        delta.to_degrees()
                    };
                    return (
                        format!(
                            "🦾 Joint {} ('{}') smoothly jogging {:+.1}{} -> Target: {:.1}{}...",
                            j_idx,
                            app.robot.joints[actual_idx].name,
                            delta_disp,
                            unit_str,
                            display_val,
                            unit_str
                        ),
                        false,
                    );
                } else {
                    return (
                        format!(
                            "Joint index {} out of range (1..={}).",
                            j_idx,
                            actuated_joints.len()
                        ),
                        true,
                    );
                }
            }
        }
    }

    // "joints 0, 45, -90, 0, 45, 0"
    if lower.starts_with("joints ") {
        let remainder = input["joints ".len()..].trim();
        let parts: Vec<&str> = remainder.split(|c| c == ',' || c == ' ').filter(|s| !s.is_empty()).collect();
        let mut actuated = app.robot.get_actuated_joint_positions();
        let mut set_count = 0;
        for (i, p) in parts.iter().enumerate() {
            if i < actuated.len() {
                if let Ok(deg) = p.trim_end_matches('°').parse::<f64>() {
                    actuated[i] = deg.to_radians();
                    set_count += 1;
                }
            }
        }
        if set_count > 0 {
            app.start_smooth_joint_move(actuated, 1.2);
            return (
                format!("🦾 Smoothly driving {} joints simultaneously...", set_count),
                false,
            );
        }
    }

    // 9. RELATIVE CARTESIAN JOGS: "up 0.05", "down 0.05", "left 0.1", "right 0.1", "forward 0.1", "backward 0.1"
    let step_directions = [
        ("up", Vector3::new(0.0, 0.0, 1.0)),
        ("down", Vector3::new(0.0, 0.0, -1.0)),
        ("left", Vector3::new(0.0, 1.0, 0.0)),
        ("right", Vector3::new(0.0, -1.0, 0.0)),
        ("forward", Vector3::new(1.0, 0.0, 0.0)),
        ("backward", Vector3::new(-1.0, 0.0, 0.0)),
    ];

    for (cmd_name, dir_vec) in step_directions {
        if lower.starts_with(cmd_name) {
            let dist: f64 = lower_tokens
                .get(1)
                .and_then(|s| {
                    s.trim_end_matches('m')
                        .trim_end_matches("cm")
                        .parse::<f64>()
                        .ok()
                })
                .unwrap_or(0.05);
            let dist_m = if input.contains("cm") {
                dist / 100.0
            } else {
                dist
            };
            let goal = app.target_pos + dir_vec * dist_m;
            app.start_smooth_cartesian_move(goal, None, 0.6);
            return (
                format!(
                    "🎯 Smoothly jogging {} by {:.3}m -> Target: ({:.3}, {:.3}, {:.3})...",
                    cmd_name, dist_m, goal.x, goal.y, goal.z
                ),
                false,
            );
        }
    }

    // 10. GOTO / MOVE CARTESIAN COMMANDS:
    // "goto 0.4 0.2 0.3" or "goto 0.4, 0.2, 0.3"
    if lower.starts_with("goto ") {
        let remainder = input["goto ".len()..].trim();
        let parts: Vec<&str> = remainder
            .split(|c| c == ',' || c == ' ')
            .filter(|s| !s.is_empty())
            .collect();
        if parts.len() >= 3 {
            if let (Ok(x), Ok(y), Ok(z)) = (
                parts[0].parse::<f64>(),
                parts[1].parse::<f64>(),
                parts[2].parse::<f64>(),
            ) {
                let goal = Point3::new(x, y, z);
                app.start_smooth_cartesian_move(goal, None, 1.0);
                return (
                    format!(
                        "📍 Smoothly gliding to ({:.3}, {:.3}, {:.3})...",
                        x, y, z
                    ),
                    false,
                );
            }
        }
    }

    // "move x 0.4 y 0.2 z 0.3"
    if lower.starts_with("move") {
        let mut x = app.target_pos.x;
        let mut y = app.target_pos.y;
        let mut z = app.target_pos.z;
        let mut parsed = false;

        let mut i = 1;
        while i < lower_tokens.len() {
            let key = lower_tokens[i].as_str();
            if let Some(val_str) = lower_tokens.get(i + 1) {
                if let Ok(val) = val_str.parse::<f64>() {
                    match key {
                        "x" | "-x" | "+x" => {
                            if key.starts_with('+')
                                || key.starts_with('-') && !val_str.starts_with('-')
                            {
                                x += val;
                            } else {
                                x = val;
                            }
                            parsed = true;
                            i += 2;
                            continue;
                        }
                        "y" | "-y" | "+y" => {
                            y = val;
                            parsed = true;
                            i += 2;
                            continue;
                        }
                        "z" | "-z" | "+z" => {
                            z = val;
                            parsed = true;
                            i += 2;
                            continue;
                        }
                        _ => {}
                    }
                }
            }
            i += 1;
        }

        if parsed {
            let goal = Point3::new(x, y, z);
            app.start_smooth_cartesian_move(goal, None, 1.0);
            return (
                format!(
                    "🎯 Smoothly moving TCP to ({:.3}, {:.3}, {:.3})...",
                    x, y, z
                ),
                false,
            );
        }
    }

    // 11. ORIENTATION COMMANDS: "roll 30", "pitch 45", "yaw -90", "rpy 10 20 30"
    if lower.starts_with("roll ") {
        if let Some(val_str) = lower_tokens.get(1) {
            if let Ok(deg) = val_str.parse::<f64>() {
                app.target_rpy_deg[0] = deg;
                app.solver_mode = IKSolverMode::FullPose;
                app.execute_solve();
                return (format!("🔄 Set Roll to {:.1}°.", deg), false);
            }
        }
    }
    if lower.starts_with("pitch ") {
        if let Some(val_str) = lower_tokens.get(1) {
            if let Ok(deg) = val_str.parse::<f64>() {
                app.target_rpy_deg[1] = deg;
                app.solver_mode = IKSolverMode::FullPose;
                app.execute_solve();
                return (format!("🔄 Set Pitch to {:.1}°.", deg), false);
            }
        }
    }
    if lower.starts_with("yaw ") {
        if let Some(val_str) = lower_tokens.get(1) {
            if let Ok(deg) = val_str.parse::<f64>() {
                app.target_rpy_deg[2] = deg;
                app.solver_mode = IKSolverMode::FullPose;
                app.execute_solve();
                return (format!("🔄 Set Yaw to {:.1}°.", deg), false);
            }
        }
    }
    if lower.starts_with("rpy ") {
        if lower_tokens.len() >= 4 {
            if let (Ok(r), Ok(p), Ok(y)) = (lower_tokens[1].parse::<f64>(), lower_tokens[2].parse::<f64>(), lower_tokens[3].parse::<f64>()) {
                app.target_rpy_deg = [r, p, y];
                app.solver_mode = IKSolverMode::FullPose;
                app.execute_solve();
                return (format!("🔄 Set Orientation to Roll={:.1}°, Pitch={:.1}°, Yaw={:.1}°.", r, p, y), false);
            }
        }
    }

    // 12. PICK AND PLACE WORKPIECES
    if lower.starts_with("pick") {
        let target_item = lower_tokens.get(1).map(|s| s.as_str()).unwrap_or("");
        let wp_idx = if target_item.contains("red") || target_item.contains("box") || target_item.contains("cube") || target_item == "1" {
            Some(0)
        } else if target_item.contains("blue") || target_item.contains("cylinder") || target_item.contains("billet") || target_item == "2" {
            Some(1)
        } else if target_item.contains("gold") || target_item.contains("sphere") || target_item.contains("brass") || target_item == "3" {
            Some(2)
        } else {
            Some(0) // Default to first workpiece
        };

        if let Some(idx) = wp_idx {
            if let Some(wp) = app.workpieces.workpieces.get(idx) {
                let wp_pos = wp.position;
                let wp_name = wp.name.clone();
                app.start_smooth_pick_sequence(wp_pos + Vector3::new(0.0, 0.0, 0.03));
                return (
                    format!(
                        "📦 Executing smooth pick sequence for '{}' at ({:.3}, {:.3}, {:.3})...",
                        wp_name, wp_pos.x, wp_pos.y, wp_pos.z
                    ),
                    false,
                );
            }
        }
        return ("Usage: pick <red|blue|gold|1|2|3>".to_string(), true);
    }

    if lower == "reset workpieces" || lower == "reset demo" {
        app.workpieces.reset_demo_workpieces();
        return ("📦 Reset demo workpieces to initial tabletop locations.".to_string(), false);
    }

    // 13. CAMERA COMMANDS
    if lower == "focus" || lower == "focus tcp" {
        let ee = app.robot.end_effector_position();
        app.camera.focus_on(Point3::new(ee.x as f32, ee.y as f32, ee.z as f32));
        return ("📷 Camera focused on Tool Center Point.".to_string(), false);
    }

    if lower == "reset camera" || lower == "camera home" {
        app.camera.reset();
        return ("📷 Camera view reset to home orientation.".to_string(), false);
    }

    // 14. SOLVER MODE
    if lower == "solver dls" {
        app.solver_type = IKSolverType::JacobianDLS;
        return ("⚡ IK Solver switched to Jacobian Damped Least Squares (DLS).".to_string(), false);
    }
    if lower == "solver fabrik" {
        app.solver_type = IKSolverType::FABRIK;
        return ("⚡ IK Solver switched to FABRIK.".to_string(), false);
    }
    if lower == "solver transpose" {
        app.solver_type = IKSolverType::JacobianTranspose;
        return ("⚡ IK Solver switched to Jacobian Transpose.".to_string(), false);
    }

    // Unknown command fallback
    (
        format!(
            "❓ Unknown command: \"{}\". Type 'help' to see available commands (e.g. 'move x 0.4 y 0.2', 'joint 1 45', 'grip', 'home', 'wave').",
            input
        ),
        true,
    )
}

/// Renders the full conversational Chat AI Copilot panel in the sidebar.
pub fn render_chat_panel(ui: &mut Ui, app: &mut RoboSimApp) {
    ui.heading("💬 AI Robotic Copilot");
    ui.label("Direct the robotic arm, tools, and routines with conversational or structured text commands.");
    ui.separator();

    // Quick Action Chips
    ui.label("⚡ Quick Commands:");
    let mut cmd_to_execute = None;

    ui.horizontal_wrapped(|ui| {
        let chips = [
            ("👋 Wave", "wave"),
            ("🤖 Nod", "nod"),
            ("✨ Dance", "dance"),
            ("🖐️ Grip", "grip"),
            ("✋ Release", "release"),
            ("🔴 Pick Red", "pick red"),
            ("🏠 Home", "home"),
            ("🔄 Zero", "zero"),
            ("🎯 Sync", "sync"),
            ("📊 Status", "status"),
            ("❓ Help", "help"),
        ];
        for (label, cmd) in chips {
            if ui.button(label).clicked() {
                cmd_to_execute = Some(cmd.to_string());
            }
        }
    });

    ui.add_space(6.0);
    ui.separator();

    // Scrollable Chat History
    let available_height = (ui.available_height() - 95.0).max(120.0);
    ScrollArea::vertical()
        .max_height(available_height)
        .auto_shrink([false, false])
        .stick_to_bottom(true)
        .show(ui, |ui| {
            for msg in &app.chat.messages {
                let (prefix, bg_color, text_color) = match msg.sender {
                    ChatSender::User => (
                        "👤 You: ",
                        Color32::from_rgb(30, 45, 65),
                        Color32::from_rgb(180, 225, 255),
                    ),
                    ChatSender::Robot => (
                        "🤖 Robot: ",
                        Color32::from_rgb(28, 40, 32),
                        Color32::from_rgb(200, 245, 200),
                    ),
                    ChatSender::System => {
                        if msg.is_error {
                            (
                                "⚠ Error: ",
                                Color32::from_rgb(60, 25, 25),
                                Color32::from_rgb(255, 170, 170),
                            )
                        } else {
                            (
                                "ℹ System: ",
                                Color32::from_rgb(35, 35, 45),
                                Color32::from_rgb(210, 210, 220),
                            )
                        }
                    }
                };

                egui::Frame::none()
                    .fill(bg_color)
                    .inner_margin(6.0)
                    .rounding(6.0)
                    .show(ui, |ui| {
                        ui.horizontal_wrapped(|ui| {
                            ui.colored_label(text_color, format!("{}{}", prefix, msg.text));
                        });
                    });
                ui.add_space(3.0);
            }
        });

    ui.add_space(4.0);
    ui.separator();

    // History navigation with Up/Down keys
    if ui.input(|i| i.key_pressed(Key::ArrowUp)) {
        app.chat.history_prev();
    }
    if ui.input(|i| i.key_pressed(Key::ArrowDown)) {
        app.chat.history_next();
    }

    // Text Input Bar
    let mut send_clicked = false;
    let mut clear_clicked = false;

    ui.horizontal(|ui| {
        let input_resp = ui.add(
            TextEdit::singleline(&mut app.chat.input_text)
                .hint_text("Type command (e.g. 'move x 0.4 y 0.2', 'joint 1 45', 'wave')...")
                .desired_width((ui.available_width() - 110.0).max(100.0)),
        );

        if input_resp.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter)) {
            send_clicked = true;
        }

        if ui.button("Send ↵").clicked() {
            send_clicked = true;
        }

        if ui.button("🗑").on_hover_text("Clear message history").clicked() {
            clear_clicked = true;
        }
    });

    if clear_clicked {
        app.chat.clear();
    }

    if send_clicked {
        let trimmed = app.chat.input_text.trim().to_string();
        if !trimmed.is_empty() {
            cmd_to_execute = Some(trimmed);
        }
    }

    // Execute requested command
    if let Some(cmd) = cmd_to_execute {
        app.chat.add_user_message(&cmd);
        app.chat.command_history.push(cmd.clone());
        app.chat.history_idx = None;
        app.chat.input_text.clear();

        let (reply, is_err) = execute_chat_command(&cmd, app);
        if is_err {
            app.chat.add_system_error(&reply);
        } else {
            app.chat.add_robot_reply(&reply);
        }
    }
}

/// Renders the floating quick-command HUD at the bottom of the central canvas.
pub fn render_quick_command_hud(ui: &mut Ui, app: &mut RoboSimApp) {
    if !app.chat.is_floating_open {
        // Render small expand button at bottom-left corner
        egui::Area::new(egui::Id::new("quick_command_toggle_area"))
            .anchor(egui::Align2::LEFT_BOTTOM, egui::vec2(16.0, -32.0))
            .show(ui.ctx(), |ui| {
                if ui.button("💬 AI Command Bar").clicked() {
                    app.chat.is_floating_open = true;
                }
            });
        return;
    }

    let mut cmd_to_execute = None;

    egui::Area::new(egui::Id::new("quick_command_hud_area"))
        .anchor(egui::Align2::CENTER_BOTTOM, egui::vec2(0.0, -32.0))
        .show(ui.ctx(), |ui| {
            egui::Frame::none()
                .fill(Color32::from_rgba_premultiplied(20, 24, 32, 235))
                .stroke(egui::Stroke::new(1.0_f32, Color32::from_rgb(50, 70, 95)))
                .inner_margin(8.0)
                .rounding(8.0)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.colored_label(Color32::from_rgb(60, 180, 255), "💬 Command:");

                        let input_resp = ui.add(
                            TextEdit::singleline(&mut app.chat.input_text)
                                .hint_text("e.g. 'move x 0.4 y 0.2', 'joint 1 45', 'wave', 'grip'...")
                                .desired_width(260.0),
                        );

                        if input_resp.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter)) {
                            let trimmed = app.chat.input_text.trim().to_string();
                            if !trimmed.is_empty() {
                                cmd_to_execute = Some(trimmed);
                            }
                        }

                        if ui.button("Send ↵").clicked() {
                            let trimmed = app.chat.input_text.trim().to_string();
                            if !trimmed.is_empty() {
                                cmd_to_execute = Some(trimmed);
                            }
                        }

                        ui.separator();

                        if ui.small_button("👋 Wave").clicked() {
                            cmd_to_execute = Some("wave".to_string());
                        }
                        if ui.small_button("🖐️ Grip").clicked() {
                            cmd_to_execute = Some("grip".to_string());
                        }
                        if ui.small_button("✋ Release").clicked() {
                            cmd_to_execute = Some("release".to_string());
                        }
                        if ui.small_button("🏠 Home").clicked() {
                            cmd_to_execute = Some("home".to_string());
                        }
                        if ui.small_button("📊 Status").clicked() {
                            cmd_to_execute = Some("status".to_string());
                        }

                        ui.separator();

                        if ui.small_button("💬 Tab").on_hover_text("Open full chat tab").clicked() {
                            app.active_tab = crate::ui::app::AppTab::Chat;
                        }

                        if ui.small_button("✕").on_hover_text("Minimize command bar").clicked() {
                            app.chat.is_floating_open = false;
                        }
                    });

                    // Show last robot response if available
                    if let Some(last_msg) = app.chat.messages.last() {
                        if last_msg.sender != ChatSender::User {
                            ui.add_space(2.0);
                            ui.horizontal(|ui| {
                                let (tag, color) = if last_msg.is_error {
                                    ("⚠", Color32::from_rgb(255, 120, 120))
                                } else {
                                    ("🤖", Color32::from_rgb(140, 230, 160))
                                };
                                let snippet: String = last_msg
                                    .text
                                    .lines()
                                    .next()
                                    .unwrap_or("")
                                    .chars()
                                    .take(75)
                                    .collect();
                                ui.colored_label(color, format!("{} {}", tag, snippet));
                            });
                        }
                    }
                });
        });

    if let Some(cmd) = cmd_to_execute {
        app.chat.add_user_message(&cmd);
        app.chat.command_history.push(cmd.clone());
        app.chat.history_idx = None;
        app.chat.input_text.clear();

        let (reply, is_err) = execute_chat_command(&cmd, app);
        if is_err {
            app.chat.add_system_error(&reply);
        } else {
            app.chat.add_robot_reply(&reply);
        }
    }
}

