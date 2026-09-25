use nalgebra::{Point3, Vector3};

use kine_rs::export::{
    export_recorded_to_csv, export_to_arduino_cpp, export_to_json, export_to_matlab,
    import_from_json,
};
use kine_rs::kinematics::{is_configuration_valid, ObstacleBox};
use kine_rs::presets;
use kine_rs::trajectory::{
    plan_cartesian_rrt, plan_rrt_connect, plan_standard_rrt, shortcut_path, RrtAlgorithm,
    RrtParams, SimpleRng, TrajectoryPlanner, TrajectoryRecorder,
};

#[test]
fn test_simple_rng_determinism() {
    let mut rng1 = SimpleRng::new(12345);
    let mut rng2 = SimpleRng::new(12345);

    for _ in 0..100 {
        assert_eq!(rng1.next_u64(), rng2.next_u64());
        let f1 = rng1.next_f64();
        let f2 = rng2.next_f64();
        assert_eq!(f1, f2);
        assert!((0.0..1.0).contains(&f1));
    }
}

#[test]
fn test_rrt_connect_clear_path() {
    let robot = presets::industrial_6dof();
    let start_q = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let goal_q = vec![0.3, -0.4, 0.5, 0.0, 0.3, 0.0];
    let obstacles = Vec::new();

    let params = RrtParams {
        max_iterations: 1000,
        step_size: 0.15,
        goal_bias: 0.2,
        collision_resolution: 0.05,
        shortcut_path: true,
        max_shortcut_iterations: 30,
        algorithm: RrtAlgorithm::RrtConnect,
        seed: 999,
    };

    let result = plan_rrt_connect(&robot, &start_q, &goal_q, &obstacles, &params);
    assert!(result.success, "RRT-Connect should find clear path");
    assert!(!result.joint_path.is_empty());
    assert_eq!(result.joint_path[0], start_q);
    assert_eq!(*result.joint_path.last().unwrap(), goal_q);
}

#[test]
fn test_rrt_connect_obstacle_avoidance() {
    let robot = presets::industrial_6dof();
    // Arm moving across workspace
    let start_q = vec![0.4, -0.5, 0.4, 0.0, 0.0, 0.0];
    let goal_q = vec![-0.4, -0.5, 0.4, 0.0, 0.0, 0.0];

    // Place an obstacle right in the center directly blocking the straight path
    let obstacles = vec![ObstacleBox::new(
        "Center Barrier",
        Point3::new(0.0, 0.35, 0.45),
        Vector3::new(0.08, 0.08, 0.20),
    )];

    let params = RrtParams {
        max_iterations: 3000,
        step_size: 0.15,
        goal_bias: 0.15,
        collision_resolution: 0.04,
        shortcut_path: true,
        max_shortcut_iterations: 40,
        algorithm: RrtAlgorithm::RrtConnect,
        seed: 42,
    };

    let result = plan_rrt_connect(&robot, &start_q, &goal_q, &obstacles, &params);
    assert!(
        result.success,
        "RRT-Connect should navigate around the obstacle: {}",
        result.message
    );

    // Verify all points on the returned path are collision-free!
    for q in &result.joint_path {
        assert!(
            is_configuration_valid(&robot, q, &obstacles),
            "Every node in planned RRT path must be collision-free!"
        );
    }
}

#[test]
fn test_rrt_standard_planning() {
    let robot = presets::planar_3dof();
    let start_q = vec![0.0, 0.0, 0.0];
    let goal_q = vec![0.5, -0.6, 0.2];
    let obstacles = Vec::new();

    let params = RrtParams {
        max_iterations: 2000,
        step_size: 0.15,
        goal_bias: 0.2,
        collision_resolution: 0.04,
        shortcut_path: false,
        max_shortcut_iterations: 0,
        algorithm: RrtAlgorithm::StandardRrt,
        seed: 777,
    };

    let result = plan_standard_rrt(&robot, &start_q, &goal_q, &obstacles, &params);
    assert!(result.success, "Standard RRT should find valid path");
    assert_eq!(result.joint_path[0], start_q);
    assert_eq!(*result.joint_path.last().unwrap(), goal_q);
}

#[test]
fn test_rrt_path_shortcutting() {
    let robot = presets::scara_4dof();
    let obstacles = Vec::new();

    // Create a deliberately zig-zag path
    let path = vec![
        vec![0.0, 0.0, 0.0, 0.0],
        vec![0.2, 0.1, 0.05, 0.1],
        vec![0.1, 0.3, 0.10, 0.2],
        vec![0.3, 0.2, 0.15, 0.3],
        vec![0.5, 0.5, 0.20, 0.5],
    ];

    let mut rng = SimpleRng::new(42);
    let smoothed = shortcut_path(&robot, path.clone(), &obstacles, 40, 0.04, &mut rng);

    assert!(
        smoothed.len() <= path.len(),
        "Shortcutting should reduce or keep waypoint count"
    );
    assert_eq!(smoothed[0], path[0]);
    assert_eq!(*smoothed.last().unwrap(), *path.last().unwrap());
}

#[test]
fn test_rrt_cartesian_planning() {
    let robot = presets::industrial_6dof();
    let obstacles = Vec::new();
    let target = Point3::new(0.35, 0.20, 0.40);

    let params = RrtParams::default();
    let result = plan_cartesian_rrt(&robot, target, &obstacles, &params);

    assert!(
        result.success,
        "Cartesian RRT planning should succeed for reachable target"
    );
    assert!(!result.cartesian_path.is_empty());

    let waypoints = result.to_waypoints(0.4);
    assert_eq!(waypoints.len(), result.cartesian_path.len());
    assert_eq!(waypoints[0].name, "RRT Start");
    assert_eq!(waypoints.last().unwrap().name, "RRT Goal");
}

#[test]
fn test_trajectory_recorder_live_session() {
    let mut recorder = TrajectoryRecorder::default();
    assert!(!recorder.is_recording);
    assert_eq!(recorder.recording.frame_count(), 0);

    recorder.start_recording(Some("Unit Test Session".to_string()));
    assert!(recorder.is_recording);

    // Simulate 4 frames captured
    recorder
        .recording
        .frames
        .push(kine_rs::trajectory::TrajectoryFrame {
            time: 0.0,
            ee_position: Point3::new(0.4, 0.0, 0.4),
            ee_rpy_deg: [0.0, 0.0, 0.0],
            joint_positions: vec![0.0; 6],
            gripper_value: 0.0,
        });
    recorder
        .recording
        .frames
        .push(kine_rs::trajectory::TrajectoryFrame {
            time: 1.0,
            ee_position: Point3::new(0.4, 0.2, 0.4),
            ee_rpy_deg: [10.0, 0.0, 0.0],
            joint_positions: vec![0.2; 6],
            gripper_value: 0.5,
        });
    recorder
        .recording
        .frames
        .push(kine_rs::trajectory::TrajectoryFrame {
            time: 2.0,
            ee_position: Point3::new(0.4, 0.4, 0.5),
            ee_rpy_deg: [20.0, 0.0, 0.0],
            joint_positions: vec![0.4; 6],
            gripper_value: 1.0,
        });

    recorder.stop_recording();
    assert!(!recorder.is_recording);
    assert_eq!(recorder.recording.frame_count(), 3);
    assert!((recorder.recording.total_duration - 2.0).abs() < 1e-5);
    assert!(recorder.recording.total_path_distance() > 0.4);

    // Test time interpolation
    let frame_mid = recorder.recording.sample_at_time(0.5).unwrap();
    assert!((frame_mid.ee_position.y - 0.1).abs() < 1e-3);
    assert!((frame_mid.ee_rpy_deg[0] - 5.0).abs() < 1e-3);
    assert!((frame_mid.gripper_value - 0.25).abs() < 1e-3);

    // Test downsampling to waypoints
    let wps = recorder.recording.to_waypoints(2);
    assert_eq!(wps.len(), 2);
    assert_eq!(wps[0].position, Point3::new(0.4, 0.0, 0.4));
    assert_eq!(wps[1].position, Point3::new(0.4, 0.4, 0.5));
}

#[test]
fn test_trajectory_recorder_replay_loop() {
    let mut recorder = TrajectoryRecorder::default();
    recorder
        .recording
        .frames
        .push(kine_rs::trajectory::TrajectoryFrame {
            time: 0.0,
            ee_position: Point3::new(0.0, 0.0, 0.0),
            ee_rpy_deg: [0.0, 0.0, 0.0],
            joint_positions: vec![0.0],
            gripper_value: 0.0,
        });
    recorder
        .recording
        .frames
        .push(kine_rs::trajectory::TrajectoryFrame {
            time: 2.0,
            ee_position: Point3::new(1.0, 0.0, 0.0),
            ee_rpy_deg: [0.0, 0.0, 0.0],
            joint_positions: vec![1.0],
            gripper_value: 1.0,
        });
    recorder.recording.total_duration = 2.0;

    recorder.toggle_replay();
    assert!(recorder.is_replaying);

    let f1 = recorder.update_replay(1.0).unwrap();
    assert!((f1.time - 1.0).abs() < 1e-5);
    assert!((f1.ee_position.x - 0.5).abs() < 1e-3);

    // Step beyond 2.0 seconds with loop=true -> wraps around
    let f2 = recorder.update_replay(1.5).unwrap();
    assert!((f2.time - 0.5).abs() < 1e-3);
}

#[test]
fn test_export_json_and_import_roundtrip() {
    let robot = presets::industrial_6dof();
    let planner = TrajectoryPlanner::default();
    let mut recorder = TrajectoryRecorder::default();

    recorder
        .recording
        .frames
        .push(kine_rs::trajectory::TrajectoryFrame {
            time: 0.0,
            ee_position: Point3::new(0.4, 0.0, 0.3),
            ee_rpy_deg: [0.0, 0.0, 0.0],
            joint_positions: vec![0.0; 6],
            gripper_value: 0.0,
        });

    let json_str = export_to_json(&robot, &planner, Some(&recorder.recording));
    assert!(json_str.contains("kine-rs-trajectory-v1.0"));
    assert!(json_str.contains(&robot.name));

    // Re-import from JSON
    let (imported_wps, imported_rec) = import_from_json(&json_str).expect("Valid JSON import");
    assert_eq!(imported_wps.len(), planner.waypoints.len());
    assert!(imported_rec.is_some());
    assert_eq!(imported_rec.unwrap().frame_count(), 1);
}

#[test]
fn test_export_arduino_cpp_and_matlab() {
    let robot = presets::industrial_6dof();
    let planner = TrajectoryPlanner::default();
    let recorder = TrajectoryRecorder::default();

    // Arduino C++ Export
    let cpp = export_to_arduino_cpp(&robot, &planner, Some(&recorder.recording));
    assert!(cpp.contains("#include <Servo.h>"));
    assert!(cpp.contains("NUM_SERVOS = 6;"));
    assert!(cpp.contains("void setup()"));
    assert!(cpp.contains("void loop()"));

    // MATLAB Export
    let matlab = export_to_matlab(&robot, &planner, Some(&recorder.recording));
    assert!(matlab.contains("cartesian_traj = ["));
    assert!(matlab.contains("figure('Name'"));
    assert!(matlab.contains("plot3("));

    // Recorded CSV Export
    let csv = export_recorded_to_csv(&recorder.recording);
    assert!(csv.starts_with("time_s,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg,gripper\n"));
}
