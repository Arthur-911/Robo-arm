use nalgebra::{Point3, Unit};
use web_time::Instant;

use super::joint::JointType;
use super::robot::RobotArm;
use super::solver::{IKSolution, IKSolverParams};

/// Solves Inverse Kinematics using the FABRIK (Forward And Backward Reaching Inverse Kinematics) algorithm
/// combined with joint constraint projections and zero-allocation forward kinematics.
pub fn solve_fabrik_ik(
    robot: &RobotArm,
    target_position: Point3<f64>,
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

    let mut working_robot = robot.clone();
    let num_joints = working_robot.joints.len();
    let total_frames = num_joints + 1;
    let mut poses = Vec::with_capacity(total_frames);
    let mut points: Vec<Point3<f64>> = Vec::with_capacity(total_frames);
    let mut lengths = Vec::with_capacity(total_frames.saturating_sub(1));

    working_robot.forward_kinematics_into(&mut poses);
    let mut residual_pos =
        (Point3::from(poses.last().unwrap().translation.vector) - target_position).norm();
    let mut converged = false;
    let mut iterations = 0;

    // Outer FABRIK / CCD refinement loop
    for iter in 0..params.max_iterations {
        iterations = iter + 1;

        if residual_pos < params.tolerance {
            converged = true;
            break;
        }

        // Extract current world positions of all frames from cached poses
        points.clear();
        for p in &poses {
            points.push(Point3::from(p.translation.vector));
        }
        let n = points.len();
        if n < 2 {
            break;
        }

        // Calculate link lengths
        lengths.clear();
        let mut total_length = 0.0;
        for i in 0..n - 1 {
            let len = (points[i + 1] - points[i]).norm().max(1e-4);
            lengths.push(len);
            total_length += len;
        }

        let base_pos = points[0];
        let dist_to_target = (target_position - base_pos).norm();

        if dist_to_target > total_length {
            // Target is unreachable: stretch the arm towards the target
            for i in 0..n - 1 {
                let r = (target_position - points[i]).norm().max(1e-4);
                let lambda = lengths[i] / r;
                points[i + 1] = points[i] + (target_position - points[i]) * lambda;
            }
        } else {
            // Stage 1: Backward Reaching
            points[n - 1] = target_position;
            for i in (0..n - 1).rev() {
                let r = (points[i + 1] - points[i]).norm().max(1e-4);
                let lambda = lengths[i] / r;
                points[i] = points[i + 1] + (points[i] - points[i + 1]) * lambda;
            }

            // Stage 2: Forward Reaching
            points[0] = base_pos;
            for i in 0..n - 1 {
                let r = (points[i + 1] - points[i]).norm().max(1e-4);
                let lambda = lengths[i] / r;
                points[i + 1] = points[i] + (points[i + 1] - points[i]) * lambda;
            }
        }

        // Angle extraction and constraint projection:
        // Iteratively project joint angles towards desired segment vectors
        for i in (0..num_joints).rev() {
            working_robot.forward_kinematics_into(&mut poses);
            let p_curr_joint = Point3::from(poses[i].translation.vector);
            let p_ee = Point3::from(poses.last().unwrap().translation.vector);

            let joint = &working_robot.joints[i];
            if !joint.is_actuated() {
                continue;
            }

            let axis_world = poses[i].rotation * joint.axis.into_inner();
            let axis_unit = Unit::new_normalize(axis_world);

            match joint.joint_type {
                JointType::Revolute | JointType::Continuous => {
                    let v_curr = p_ee - p_curr_joint;
                    let v_targ = target_position - p_curr_joint;

                    let v_curr_proj = v_curr - axis_unit.into_inner() * v_curr.dot(&axis_unit);
                    let v_targ_proj = v_targ - axis_unit.into_inner() * v_targ.dot(&axis_unit);

                    let norm_curr = v_curr_proj.norm();
                    let norm_targ = v_targ_proj.norm();

                    if norm_curr > 1e-5 && norm_targ > 1e-5 {
                        let u_curr = v_curr_proj / norm_curr;
                        let u_targ = v_targ_proj / norm_targ;

                        let dot = u_curr.dot(&u_targ).clamp(-1.0, 1.0);
                        let cross = u_curr.cross(&u_targ);
                        let sign = cross.dot(&axis_unit);
                        let angle_delta = dot.acos() * sign.signum();

                        let new_val = working_robot.joints[i].clamp_position(
                            working_robot.joints[i].current_position
                                + angle_delta * params.step_size,
                        );
                        working_robot.joints[i].current_position = new_val;
                    }
                }
                JointType::Prismatic => {
                    let v_err = target_position - p_ee;
                    let delta_disp = v_err.dot(&axis_unit);
                    let new_val = working_robot.joints[i].clamp_position(
                        working_robot.joints[i].current_position + delta_disp * params.step_size,
                    );
                    working_robot.joints[i].current_position = new_val;
                }
                JointType::Fixed => {}
            }
        }

        working_robot.forward_kinematics_into(&mut poses);
        residual_pos =
            (Point3::from(poses.last().unwrap().translation.vector) - target_position).norm();
    }

    let elapsed = start_time.elapsed().as_micros();
    let q = working_robot.get_actuated_joint_positions();

    IKSolution {
        joint_positions: q,
        converged,
        iterations,
        residual_position_error: residual_pos,
        residual_orientation_error: 0.0,
        solve_time_us: elapsed,
    }
}
