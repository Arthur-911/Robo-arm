use kine_rs::kinematics::*;
use kine_rs::presets::*;
use nalgebra::Point3;

#[test]
fn test_planar_arm_fk() {
    let mut arm = planar_3dof();
    arm.reset_to_home();
    let ee_pos = arm.end_effector_position();

    // In home position (all q = 0), joints extend along X axis: 0.4 + 0.3 + 0.2 = 0.9
    // plus base 0.05 Z
    assert!((ee_pos.x - 0.9).abs() < 1e-4);
    assert!((ee_pos.y - 0.0).abs() < 1e-4);
    assert!((ee_pos.z - 0.05).abs() < 1e-4);
}

#[test]
fn test_jacobian_dimensions() {
    let arm = industrial_6dof();
    assert_eq!(arm.dof(), 6);

    let jac = arm.compute_jacobian();
    assert_eq!(jac.nrows(), 6);
    assert_eq!(jac.ncols(), 6);

    let pos_jac = arm.compute_position_jacobian();
    assert_eq!(pos_jac.nrows(), 3);
    assert_eq!(pos_jac.ncols(), 6);
}

#[test]
fn test_jacobian_dls_ik_convergence() {
    let arm = planar_3dof();
    let target = Point3::new(0.5, 0.4, 0.05);

    let params = IKSolverParams {
        max_iterations: 100,
        tolerance: 1e-3,
        step_size: 0.9,
        damping: 0.05,
        ..Default::default()
    };

    let solution = solve_jacobian_ik(
        &arm,
        target,
        None,
        IKSolverMode::PositionOnly,
        IKSolverType::JacobianDLS,
        &params,
    );

    assert!(
        solution.converged,
        "Jacobian DLS should converge for reachable target within tolerance, err={}",
        solution.residual_position_error
    );
    assert!(solution.residual_position_error < 1e-3);
}

#[test]
fn test_fabrik_ik_convergence() {
    let arm = planar_3dof();
    let target = Point3::new(0.4, 0.3, 0.05);

    let params = IKSolverParams {
        max_iterations: 80,
        tolerance: 2e-3,
        step_size: 0.8,
        ..Default::default()
    };

    let solution = solve_fabrik_ik(&arm, target, &params);
    assert!(
        solution.converged,
        "FABRIK should converge, err={}",
        solution.residual_position_error
    );
    assert!(solution.residual_position_error < 2e-3);
}

#[test]
fn test_fk_zero_allocation_and_cached_origin() {
    let arm = industrial_6dof();
    let poses1 = arm.forward_kinematics();

    let mut poses2 = Vec::new();
    arm.forward_kinematics_into(&mut poses2);

    assert_eq!(poses1.len(), poses2.len());
    for (p1, p2) in poses1.iter().zip(poses2.iter()) {
        assert!((p1.translation.vector - p2.translation.vector).norm() < 1e-9);
    }

    // Verify Jacobian from precomputed poses matches standard compute_jacobian
    let j1 = arm.compute_jacobian();
    let j2 = arm.compute_jacobian_from_poses(&poses2);
    assert!((j1 - j2).norm() < 1e-9);
}

#[test]
fn test_full_6d_pose_ik_convergence() {
    let mut arm = industrial_6dof();
    arm.set_actuated_joint_positions(&[0.1, 0.4, -0.6, 0.2, 0.1, 0.0]);
    let original_ee = arm.end_effector_pose();
    let target_pos =
        Point3::from(original_ee.translation.vector) + nalgebra::Vector3::new(0.04, -0.04, 0.02);
    let target_rot = original_ee.rotation;

    let params = IKSolverParams {
        max_iterations: 80,
        tolerance: 2e-3,
        orientation_tolerance: 1e-2,
        adaptive_damping: true,
        enable_line_search: true,
        ..Default::default()
    };

    let sol = solve_jacobian_ik(
        &arm,
        target_pos,
        Some(target_rot),
        IKSolverMode::FullPose,
        IKSolverType::JacobianDLS,
        &params,
    );

    assert!(
        sol.converged,
        "Full 6D Pose IK should converge smoothly, pos_err={}, rot_err={}",
        sol.residual_position_error, sol.residual_orientation_error
    );
    assert!(sol.residual_position_error < 2e-3);
    assert!(sol.residual_orientation_error < 1e-2);
}

#[test]
fn test_adaptive_damping_near_singularity() {
    let arm = planar_3dof();
    // Target is right at the boundary envelope of the arm (singularity configuration)
    let boundary_target = Point3::new(0.895, 0.0, 0.05);

    let params = IKSolverParams {
        max_iterations: 60,
        tolerance: 5e-3,
        adaptive_damping: true,
        singularity_threshold: 0.05,
        max_damping: 0.3,
        ..Default::default()
    };

    let sol = solve_jacobian_ik(
        &arm,
        boundary_target,
        None,
        IKSolverMode::PositionOnly,
        IKSolverType::JacobianDLS,
        &params,
    );

    // Should remain numerically stable and not blow up joint positions or produce NaNs
    for q in &sol.joint_positions {
        assert!(!q.is_nan());
        assert!(!q.is_infinite());
    }
}
