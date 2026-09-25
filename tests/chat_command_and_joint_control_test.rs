use kine_rs::ui::app::{RoboSimApp, SmoothMotionTarget};
use kine_rs::ui::chat_command::{execute_chat_command, ChatCommandConsole, ChatSender};
use nalgebra::Point3;

#[test]
fn test_chat_command_direct_joint_moves() {
    let mut app = RoboSimApp::default();
    assert_eq!(app.robot.dof(), 6);

    // 1. Move joint 1 to 45 degrees
    let (reply, is_err) = execute_chat_command("joint 1 45", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    assert!(app.active_smooth_motion.is_some());
    app.finish_active_smooth_motion();
    let q = app.robot.get_actuated_joint_positions();
    assert!((q[0].to_degrees() - 45.0).abs() < 1e-4);

    // Verify target_pos and target_rpy_deg were synced with FK
    let ee_pos = app.robot.end_effector_position();
    assert!((app.target_pos.x - ee_pos.x).abs() < 1e-5);
    assert!((app.target_pos.y - ee_pos.y).abs() < 1e-5);
    assert!((app.target_pos.z - ee_pos.z).abs() < 1e-5);

    // 2. Short syntax "j2 -30"
    let (reply, is_err) = execute_chat_command("j2 -30", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
    let q = app.robot.get_actuated_joint_positions();
    assert!((q[1].to_degrees() - (-30.0)).abs() < 1e-4);

    // 3. Relative jog "jog j1 +10" (45 + 10 = 55 deg)
    let (reply, is_err) = execute_chat_command("jog j1 +10", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
    let q = app.robot.get_actuated_joint_positions();
    assert!((q[0].to_degrees() - 55.0).abs() < 1e-4);

    // 4. Relative jog "jog j2 -5" (-30 - 5 = -35 deg)
    let (reply, is_err) = execute_chat_command("jog j2 -5", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
    let q = app.robot.get_actuated_joint_positions();
    assert!((q[1].to_degrees() - (-35.0)).abs() < 1e-4);

    // 5. Multi-joint simultaneous assignment "joints 10, 20, 30, -10, -20, -30"
    let (reply, is_err) = execute_chat_command("joints 10, 20, 30, -10, -20, -30", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
    let q = app.robot.get_actuated_joint_positions();
    assert!((q[0].to_degrees() - 10.0).abs() < 1e-3);
    assert!((q[1].to_degrees() - 20.0).abs() < 1e-3);
    assert!((q[2].to_degrees() - 30.0).abs() < 1e-3);
    assert!((q[3].to_degrees() - (-10.0)).abs() < 1e-3);
    assert!((q[4].to_degrees() - (-20.0)).abs() < 1e-3);
    assert!((q[5].to_degrees() - (-30.0)).abs() < 1e-3);

    // 6. Zero all joints
    let (reply, is_err) = execute_chat_command("zero", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
    for val in app.robot.get_actuated_joint_positions() {
        assert!(val.abs() < 1e-6);
    }

    // 7. Reset to home
    let (reply, is_err) = execute_chat_command("home", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
}

#[test]
fn test_chat_command_cartesian_and_jogs() {
    let mut app = RoboSimApp::default();

    // 1. Move to Cartesian coordinates
    let (reply, is_err) = execute_chat_command("move x 0.40 y 0.15 z 0.35", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    assert!(app.active_smooth_motion.is_some());
    app.finish_active_smooth_motion();
    assert!((app.target_pos.x - 0.40).abs() < 1e-4);
    assert!((app.target_pos.y - 0.15).abs() < 1e-4);
    assert!((app.target_pos.z - 0.35).abs() < 1e-4);

    // 2. Relative jogs
    let old_z = app.target_pos.z;
    let (reply, is_err) = execute_chat_command("up 0.05", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
    assert!((app.target_pos.z - (old_z + 0.05)).abs() < 1e-4);

    let old_y = app.target_pos.y;
    let (reply, is_err) = execute_chat_command("left 0.03", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
    assert!((app.target_pos.y - (old_y + 0.03)).abs() < 1e-4);

    // 3. Goto shortcut
    let (reply, is_err) = execute_chat_command("goto 0.35 0.10 0.25", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
    assert!((app.target_pos.x - 0.35).abs() < 1e-4);
    assert!((app.target_pos.y - 0.10).abs() < 1e-4);
    assert!((app.target_pos.z - 0.25).abs() < 1e-4);
}

#[test]
fn test_chat_command_tool_and_workcell() {
    let mut app = RoboSimApp::default();

    // 1. Gripper commands
    let (reply, is_err) = execute_chat_command("grip", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
    assert!(app.tool_state.is_gripping());

    let (reply, is_err) = execute_chat_command("release", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
    assert!(!app.tool_state.is_gripping());

    let (reply, is_err) = execute_chat_command("gripper 75%", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    app.finish_active_smooth_motion();
    assert!((app.tool_state.gripper_opening - 0.25).abs() < 1e-3);

    // 2. Vacuum tool
    let (reply, is_err) = execute_chat_command("vacuum on", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    assert!(app.tool_state.is_vacuum_active);

    let (reply, is_err) = execute_chat_command("vacuum off", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    assert!(!app.tool_state.is_vacuum_active);

    // 3. Welding torch
    let (reply, is_err) = execute_chat_command("weld on", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    assert!(app.tool_state.is_welding_active);

    let (reply, is_err) = execute_chat_command("weld off", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    assert!(!app.tool_state.is_welding_active);

    // 4. Pick workpiece
    let (reply, is_err) = execute_chat_command("pick red", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    assert!(app.active_smooth_motion.is_some());
    app.finish_active_smooth_motion();
    assert!(app.tool_state.is_gripping());
}

#[test]
fn test_chat_command_procedural_animations() {
    let mut app = RoboSimApp::default();

    // Wave routine
    let (reply, is_err) = execute_chat_command("wave", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    assert!(app.controller.is_playing);
    assert!(app.controller.planner.waypoints.len() >= 4);

    // Nod routine
    let (reply, is_err) = execute_chat_command("nod", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    assert!(app.controller.is_playing);
    assert!(app.controller.planner.waypoints.len() >= 4);

    // Dance routine
    let (reply, is_err) = execute_chat_command("dance", &mut app);
    assert!(!is_err, "Reply error: {}", reply);
    assert!(app.controller.is_playing);
    assert!(app.controller.is_looping);
}

#[test]
fn test_chat_console_state_and_history() {
    let mut console = ChatCommandConsole::default();
    assert_eq!(console.messages.len(), 1);
    assert_eq!(console.messages[0].sender, ChatSender::Robot);

    console.add_user_message("joint 1 45");
    assert_eq!(console.messages.len(), 2);
    assert_eq!(console.messages[1].sender, ChatSender::User);

    console.add_robot_reply("Joint 1 set to 45°");
    assert_eq!(console.messages.len(), 3);
    assert_eq!(console.messages[2].sender, ChatSender::Robot);

    // History navigation
    console.command_history.push("home".to_string());
    console.command_history.push("joint 1 45".to_string());
    console.command_history.push("wave".to_string());

    console.history_prev();
    assert_eq!(console.input_text, "wave");
    console.history_prev();
    assert_eq!(console.input_text, "joint 1 45");
    console.history_next();
    assert_eq!(console.input_text, "wave");
    console.history_next();
    assert_eq!(console.input_text, "");

    // Clear
    console.clear();
    assert_eq!(console.messages.len(), 1);
    assert_eq!(console.messages[0].sender, ChatSender::System);
}

#[test]
fn test_smooth_motion_interpolation() {
    let mut app = RoboSimApp::default();
    let goal = Point3::new(0.40, 0.20, 0.30);
    app.start_smooth_cartesian_move(goal, None, 1.0);

    assert!(app.active_smooth_motion.is_some());
    if let Some(motion) = &app.active_smooth_motion {
        match &motion.target {
            SmoothMotionTarget::Cartesian { goal: g, .. } => {
                assert!((g.x - 0.40).abs() < 1e-4);
            }
            _ => panic!("Expected Cartesian smooth motion target"),
        }
    }

    app.finish_active_smooth_motion();
    assert!(app.active_smooth_motion.is_none());
    assert!((app.target_pos.x - 0.40).abs() < 1e-4);
    assert!((app.target_pos.y - 0.20).abs() < 1e-4);
    assert!((app.target_pos.z - 0.30).abs() < 1e-4);
}
