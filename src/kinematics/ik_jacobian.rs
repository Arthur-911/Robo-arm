use nalgebra::{DMatrix, DVector, Point3, UnitQuaternion, Vector3};
use web_time::Instant;

use super::robot::RobotArm;
use super::solver::{IKSolution, IKSolverMode, IKSolverParams, IKSolverType};

/// Clamps joint values within each joint's mechanical limits in-place.
fn clamp_q_inplace(robot: &RobotArm, q: &mut [f64]) {
    let mut actuated_i = 0;
    for joint in &robot.joints {
        if joint.is_actuated() && actuated_i < q.len() {
            q[actuated_i] = joint.clamp_position(q[actuated_i]);
            actuated_i += 1;
        }
    }
}

/// Solves inverse kinematics using high-performance, singularity-robust Jacobian-based methods.
/// Features task-space DLS with Cholesky decomposition, adaptive Levenberg-Marquardt damping,
/// quaternion shortest-path tracking, and smooth null-space limit avoidance.
pub fn solve_jacobian_ik(
    robot: &RobotArm,
    target_position: Point3<f64>,
    target_orientation: Option<UnitQuaternion<f64>>,
    mode: IKSolverMode,
    solver_type: IKSolverType,
    params: &IKSolverParams,
) -> IKSolution {
    let start_time = Instant::now();
    let dof = robot.dof();
    if dof == 0 {
        return IKSolution {
            joint_positions: vec![],
            converged: false,
            iterations: 0,
            residual_position_error: (robot.end_effector_position() - target_position).norm(),
            residual_orientation_error: 0.0,
            solve_time_us: start_time.elapsed().as_micros(),
        };
    }

    let mut q = robot.get_actuated_joint_positions();
    let use_full_pose = mode == IKSolverMode::FullPose && target_orientation.is_some();
    let task_dim = if use_full_pose { 6 } else { 3 };

    // Preallocated buffers for zero-allocation iteration loops
    let total_frames = robot.joints.len() + 1;
    let mut poses = Vec::with_capacity(total_frames);
    let mut candidate_poses = Vec::with_capacity(total_frames);
    let mut j = DMatrix::zeros(task_dim, dof);

    let mut residual_pos = 0.0;
    let mut residual_rot = 0.0;
    let mut converged = false;
    let mut iterations = 0;
    let mut current_damping = params.damping.max(1e-4);

    for iter in 0..params.max_iterations {
        iterations = iter + 1;

        // 1. Single forward kinematics evaluation per iteration
        robot.forward_kinematics_with_q_into(&q, &mut poses);
        let current_pose = *poses.last().unwrap_or(&robot.base_transform);
        let current_pos = Point3::from(current_pose.translation.vector);

        let pos_err = target_position - current_pos;
        residual_pos = pos_err.norm();

        let (error_vec, rot_norm) = if use_full_pose {
            let target_rot = target_orientation.unwrap();
            let current_rot = current_pose.rotation;

            let diff_rot = target_rot * current_rot.inverse();
            let (axis, angle) = diff_rot
                .axis_angle()
                .unwrap_or((nalgebra::Unit::new_normalize(Vector3::x()), 0.0));
            let rot_err = axis.into_inner() * angle;
            residual_rot = rot_err.norm();

            let mut err = DVector::zeros(6);
            err[0] = pos_err.x;
            err[1] = pos_err.y;
            err[2] = pos_err.z;
            err[3] = rot_err.x;
            err[4] = rot_err.y;
            err[5] = rot_err.z;
            (err, residual_rot)
        } else {
            residual_rot = 0.0;
            let mut err = DVector::zeros(3);
            err[0] = pos_err.x;
            err[1] = pos_err.y;
            err[2] = pos_err.z;
            (err, 0.0)
        };

        // Convergence check
        if residual_pos < params.tolerance
            && (!use_full_pose || rot_norm < params.orientation_tolerance)
        {
            converged = true;
            break;
        }

        // 2. Compute Jacobian directly from precomputed poses
        if use_full_pose {
            robot.compute_jacobian_from_poses_into(&poses, &mut j);
        } else {
            robot.compute_position_jacobian_from_poses_into(&poses, &mut j);
        }

        // 3. Compute delta_q
        let delta_q = match solver_type {
            IKSolverType::JacobianTranspose => {
                let jt = j.transpose();
                let mut dq = jt * &error_vec;
                let norm = dq.norm();
                if norm > 0.5 {
                    dq *= 0.5 / norm;
                }
                dq * params.step_size
            }
            IKSolverType::JacobianDLS | IKSolverType::FABRIK => {
                let mut dq_result = if task_dim <= dof {
                    // Task-space DLS formulation: dq = J^T * (J * J^T + lambda^2 * I)^(-1) * e
                    let j_jt = &j * j.transpose();

                    // Singularity-Robust (SR) Adaptive Damping:
                    let lambda = if params.adaptive_damping {
                        let j_pos = j.rows(0, 3);
                        let a = &j_pos * j_pos.transpose();
                        let det_pos = a[(0, 0)] * (a[(1, 1)] * a[(2, 2)] - a[(1, 2)] * a[(2, 1)])
                            - a[(0, 1)] * (a[(1, 0)] * a[(2, 2)] - a[(1, 2)] * a[(2, 0)])
                            + a[(0, 2)] * (a[(1, 0)] * a[(2, 1)] - a[(1, 1)] * a[(2, 0)]);

                        let w = det_pos.max(0.0).sqrt();
                        if w < params.singularity_threshold {
                            let ratio = (1.0 - w / params.singularity_threshold).max(0.0);
                            current_damping
                                + (params.max_damping - current_damping) * ratio * ratio
                        } else {
                            current_damping
                        }
                    } else {
                        current_damping
                    };

                    let lambda_sq = lambda * lambda;
                    let id_task = DMatrix::identity(task_dim, task_dim);
                    let a_mat = j_jt + id_task * lambda_sq;

                    // Solve (J * J^T + lambda^2 * I) y = e using Cholesky decomposition
                    let chol_opt = a_mat.clone().cholesky();

                    let solved_y = if let Some(ref chol) = chol_opt {
                        chol.solve(&error_vec)
                    } else if let Some(inv) = a_mat.clone().try_inverse() {
                        &inv * &error_vec
                    } else {
                        error_vec.clone() * 0.1
                    };

                    let jt = j.transpose();
                    let mut dq_task = &jt * solved_y;

                    // Secondary task: Null space projection for redundant DOFs
                    if params.enable_null_space_centering && dof > task_dim {
                        let j_pinv = if let Some(ref chol) = chol_opt {
                            let solved_j = chol.solve(&j);
                            &jt * solved_j
                        } else {
                            let inv = a_mat.try_inverse().unwrap_or_else(|| DMatrix::identity(task_dim, task_dim));
                            &jt * (&inv * &j)
                        };

                        let null_proj = DMatrix::identity(dof, dof) - j_pinv;

                        // Smooth nonlinear barrier gradient towards joint midpoints
                        let mut grad = DVector::zeros(dof);
                        let mut actuated_i = 0;
                        for joint in &robot.joints {
                            if joint.is_actuated() {
                                if let Some((min, max)) = joint.limits {
                                    let center = (min + max) * 0.5;
                                    let half_range = ((max - min) * 0.5).max(1e-3);
                                    let u = ((q[actuated_i] - center) / half_range).clamp(-0.95, 0.95);
                                    let barrier = 1.0 + (u * u) / (1.0 - u * u);
                                    grad[actuated_i] = -barrier * (u / half_range);
                                }
                                actuated_i += 1;
                            }
                        }

                        let dq_null = null_proj * (grad * params.null_space_weight);
                        dq_task += dq_null;
                    }

                    dq_task
                } else {
                    // Joint-space DLS formulation (for under-actuated systems dof < task_dim)
                    let jt = j.transpose();
                    let jtj = &jt * &j;
                    let id_joint = DMatrix::identity(dof, dof);
                    let lambda_sq = current_damping * current_damping;
                    let a_mat = jtj + id_joint * lambda_sq;

                    let jt_e = &jt * &error_vec;
                    if let Some(chol) = a_mat.clone().cholesky() {
                        chol.solve(&jt_e)
                    } else if let Some(inv) = a_mat.try_inverse() {
                        &inv * jt_e
                    } else {
                        jt_e * 0.1
                    }
                };

                // Clamp max step magnitude per iteration to prevent erratic jumps
                let max_step = 0.5;
                let norm = dq_result.norm();
                if norm > max_step {
                    dq_result *= max_step / norm;
                }

                dq_result
            }
        };

        // 4. Update joint angles with optional Levenberg-Marquardt adaptive step check
        if params.enable_line_search {
            let current_err_norm = (residual_pos * residual_pos + residual_rot * residual_rot).sqrt();
            let mut candidate_q = q.clone();
            for i in 0..dof {
                candidate_q[i] += delta_q[i] * params.step_size;
            }
            clamp_q_inplace(robot, &mut candidate_q);

            robot.forward_kinematics_with_q_into(&candidate_q, &mut candidate_poses);
            let cand_ee = candidate_poses.last().unwrap();
            let cand_pos = Point3::from(cand_ee.translation.vector);
            let cand_pos_err = (target_position - cand_pos).norm();

            let cand_rot_err = if use_full_pose {
                let cand_target_rot = target_orientation.unwrap();
                let cand_diff = cand_target_rot * cand_ee.rotation.inverse();
                cand_diff
                    .axis_angle()
                    .map(|(a, ang)| (a.into_inner() * ang).norm())
                    .unwrap_or(0.0)
            } else {
                0.0
            };
            let cand_err_norm = (cand_pos_err * cand_pos_err + cand_rot_err * cand_rot_err).sqrt();

            if cand_err_norm <= current_err_norm {
                // Step improves error: accept and decrease damping towards baseline
                q = candidate_q;
                current_damping = (current_damping * 0.8).max(params.damping * 0.2);
            } else {
                // Step worsened error: try half step or increase damping for next iteration
                let mut half_q = q.clone();
                for i in 0..dof {
                    half_q[i] += delta_q[i] * (params.step_size * 0.5);
                }
                clamp_q_inplace(robot, &mut half_q);

                robot.forward_kinematics_with_q_into(&half_q, &mut candidate_poses);
                let half_ee = candidate_poses.last().unwrap();
                let half_pos = Point3::from(half_ee.translation.vector);
                let half_pos_err = (target_position - half_pos).norm();
                let half_rot_err = if use_full_pose {
                    let cand_target_rot = target_orientation.unwrap();
                    let cand_diff = cand_target_rot * half_ee.rotation.inverse();
                    cand_diff
                        .axis_angle()
                        .map(|(a, ang)| (a.into_inner() * ang).norm())
                        .unwrap_or(0.0)
                } else {
                    0.0
                };
                let half_err_norm = (half_pos_err * half_pos_err + half_rot_err * half_rot_err).sqrt();

                if half_err_norm < current_err_norm {
                    q = half_q;
                } else {
                    // Increase damping to emphasize steepest-descent direction
                    current_damping = (current_damping * 2.0).min(params.max_damping * 2.0);
                }
            }
        } else {
            for i in 0..dof {
                q[i] += delta_q[i] * params.step_size;
            }
            clamp_q_inplace(robot, &mut q);
        }
    }

    let elapsed = start_time.elapsed().as_micros();

    IKSolution {
        joint_positions: q,
        converged,
        iterations,
        residual_position_error: residual_pos,
        residual_orientation_error: residual_rot,
        solve_time_us: elapsed,
    }
}
