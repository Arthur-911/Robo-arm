use nalgebra::{Point3, UnitQuaternion, Vector3};
use kine_rs::kinematics::{
    check_collisions, compute_gravity_torques, compute_manipulability, ObstacleBox,
};
use kine_rs::presets;
use kine_rs::workcell::{EOATType, ToolState, WorkpieceManager};
use kine_rs::export::{export_to_gcode_csv, export_to_python, export_to_ros2};
use kine_rs::trajectory::TrajectoryPlanner;

#[test]
fn test_yoshikawa_manipulability() {
    let robot = presets::industrial_6dof();
    let data = compute_manipulability(&robot);

    // Score should be positive and non-zero for home position
    assert!(data.score > 0.0);
    assert!(data.semi_axes[0] >= data.semi_axes[1]);
    assert!(data.semi_axes[1] >= data.semi_axes[2]);
}

#[test]
fn test_joint_gravity_dynamics() {
    let robot = presets::industrial_6dof();
    let report_no_payload = compute_gravity_torques(&robot, 0.0);
    assert_eq!(report_no_payload.joint_torques_nm.len(), robot.total_joints());
    assert_eq!(report_no_payload.load_percentages.len(), robot.total_joints());

    // With a 10 kg payload, base shoulder joints should experience higher torque
    let report_heavy = compute_gravity_torques(&robot, 10.0);
    assert!(report_heavy.joint_torques_nm[1].abs() > report_no_payload.joint_torques_nm[1].abs());
}

#[test]
fn test_collision_detection() {
    let robot = presets::industrial_6dof();
    let obstacles = vec![
        ObstacleBox::new("Table", Point3::new(0.4, 0.0, -0.02), Vector3::new(0.3, 0.3, 0.02)),
        // Place an obstacle right at the end effector
        ObstacleBox::new("Obstacle", robot.end_effector_position(), Vector3::new(0.1, 0.1, 0.1)),
    ];

    let report = check_collisions(&robot, &obstacles);
    assert!(report.in_collision);
    assert!(!report.colliding_links.is_empty());
}

#[test]
fn test_workpiece_pick_and_place() {
    let mut mgr = WorkpieceManager::default();
    assert_eq!(mgr.workpieces.len(), 3);
    assert!(mgr.currently_held_id.is_none());

    let mut tool = ToolState::default();
    tool.tool_type = EOATType::ParallelGripper;
    tool.gripper_opening = 0.8; // Open
    assert!(!tool.is_gripping());

    let cube_pos = mgr.workpieces[0].position;

    // Approach cube with open gripper -> no grasp
    mgr.update(cube_pos, UnitQuaternion::identity(), tool.is_gripping());
    assert!(mgr.currently_held_id.is_none());

    // Close gripper at workpiece location -> grasped!
    tool.gripper_opening = 0.1;
    assert!(tool.is_gripping());
    mgr.update(cube_pos, UnitQuaternion::identity(), tool.is_gripping());
    assert!(mgr.currently_held_id.is_some());

    // Move gripper away -> workpiece follows!
    let lifted_pos = cube_pos + Vector3::new(0.0, 0.0, 0.15);
    mgr.update(lifted_pos, UnitQuaternion::identity(), tool.is_gripping());
    assert!((mgr.workpieces[0].position.z - lifted_pos.z).abs() < 1e-4);

    // Release workpiece
    tool.gripper_opening = 0.9;
    assert!(!tool.is_gripping());
    mgr.update(lifted_pos, UnitQuaternion::identity(), tool.is_gripping());
    assert!(mgr.currently_held_id.is_none());
}

#[test]
fn test_code_generators() {
    let robot = presets::industrial_6dof();
    let planner = TrajectoryPlanner::default();

    let py_code = export_to_python(&robot, &planner);
    assert!(py_code.contains("import numpy as np"));
    assert!(py_code.contains("waypoints = ["));

    let ros2_yaml = export_to_ros2(&robot, &planner);
    assert!(ros2_yaml.contains("trajectory_msgs/msg/JointTrajectory"));
    assert!(ros2_yaml.contains("joint_names:"));

    let gcode = export_to_gcode_csv(&robot, &planner);
    assert!(gcode.contains("G01 X"));
    assert!(gcode.contains("time_s,x_m,y_m,z_m"));
}
