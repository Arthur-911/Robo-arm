use kine_rs::trajectory::*;

#[test]
fn test_cubic_polynomial_boundary_conditions() {
    let s0 = 1.0;
    let s1 = 4.0;
    let v0 = 0.0;
    let v1 = 0.0;
    let dur = 3.0;

    let poly = CubicPolynomial::new(s0, s1, v0, v1, dur);

    assert!((poly.position(0.0) - s0).abs() < 1e-6);
    assert!((poly.position(dur) - s1).abs() < 1e-6);
    assert!((poly.velocity(0.0) - v0).abs() < 1e-6);
    assert!((poly.velocity(dur) - v1).abs() < 1e-6);
}

#[test]
fn test_quintic_polynomial_boundary_conditions() {
    let s0 = 2.0;
    let s1 = 5.0;
    let v0 = 0.0;
    let v1 = 0.0;
    let a0 = 0.0;
    let a1 = 0.0;
    let dur = 4.0;

    let poly = QuinticPolynomial::new(s0, s1, v0, v1, a0, a1, dur);

    assert!((poly.position(0.0) - s0).abs() < 1e-6);
    assert!((poly.position(dur) - s1).abs() < 1e-6);
    assert!((poly.velocity(0.0) - v0).abs() < 1e-6);
    assert!((poly.velocity(dur) - v1).abs() < 1e-6);
    assert!((poly.acceleration(0.0) - a0).abs() < 1e-6);
    assert!((poly.acceleration(dur) - a1).abs() < 1e-6);
}

#[test]
fn test_trajectory_controller_playback() {
    let mut controller = TrajectoryController::default();
    assert!(!controller.is_playing);

    controller.play();
    assert!(controller.is_playing);

    let pt0 = controller.update(0.0).unwrap();
    let pt1 = controller.update(1.0).unwrap();

    assert_ne!(pt0, pt1);
    assert!((controller.current_time - 1.0).abs() < 1e-5);
}

#[test]
fn test_smooth_continuous_trajectory() {
    let mut planner = TrajectoryPlanner::default();
    planner.set_continuity_mode(TrajectoryContinuityMode::SmoothContinuous);
    assert_eq!(planner.segments.len(), planner.waypoints.len() - 1);

    // At the start waypoint (t = 0), velocity should be zero
    let (p0, v0, _) = planner.evaluate_state(0.0).unwrap();
    assert!((p0 - planner.waypoints[0].position).norm() < 1e-4);
    assert!(v0.norm() < 1e-4);

    // In smooth continuous mode, intermediate waypoint 1 (t = waypoints[1].duration = 2.0)
    // should have non-zero velocity matching the smooth fly-through!
    let (p_mid, v_mid, _) = planner.evaluate_state(2.0).unwrap();
    assert!((p_mid - planner.waypoints[1].position).norm() < 1e-4);
    assert!(
        v_mid.norm() > 0.01,
        "Intermediate waypoint should have continuous non-zero fly-through velocity!"
    );

    // Compare with StopAtWaypoints mode
    planner.set_continuity_mode(TrajectoryContinuityMode::StopAtWaypoints);
    let (_, v_stop, _) = planner.evaluate_state(2.0).unwrap();
    assert!(
        v_stop.norm() < 1e-4,
        "StopAtWaypoints mode should halt with zero velocity at waypoint!"
    );
}

#[test]
fn test_joint_trajectory_smoother() {
    let mut smoother = JointTrajectorySmoother::new(2.0, 0.5);
    smoother.reset(&[0.0, 0.0, 0.0]);

    // Target step of 1.0 rad with dt = 0.02s
    let target = [1.0, 1.0, 1.0];
    let next_q = smoother.filter(&target, 0.02);

    // Max delta allowed per step = 2.0 * 0.02 = 0.04 rad
    for &val in next_q.iter().take(3) {
        assert!(val > 0.0);
        assert!(val <= 0.04 + 1e-6, "Should clamp step to max velocity");
    }
}
