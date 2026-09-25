use nalgebra::{DMatrix, Isometry3, Point3};
use serde::{Deserialize, Serialize};

use super::joint::{Joint, JointType};
use super::link::Link;

/// A serial kinematic chain representing a robotic arm manipulator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotArm {
    pub name: String,
    pub joints: Vec<Joint>,
    pub links: Vec<Link>,
    pub base_transform: Isometry3<f64>,
}

impl RobotArm {
    /// Creates a new robot arm with an optional base transform.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            joints: Vec::new(),
            links: Vec::new(),
            base_transform: Isometry3::identity(),
        }
    }

    /// Adds a joint and connecting link to the kinematic chain.
    pub fn add_joint_and_link(&mut self, joint: Joint, link: Link) {
        self.joints.push(joint);
        self.links.push(link);
    }

    /// Number of actuated degrees of freedom (DOFs).
    pub fn dof(&self) -> usize {
        self.joints.iter().filter(|j| j.is_actuated()).count()
    }

    /// Total number of joints (including fixed).
    pub fn total_joints(&self) -> usize {
        self.joints.len()
    }

    /// Evaluates Forward Kinematics (FK) into an existing vector, avoiding heap allocations.
    pub fn forward_kinematics_into(&self, poses: &mut Vec<Isometry3<f64>>) {
        poses.clear();
        poses.reserve(self.joints.len() + 1);
        let mut current_pose = self.base_transform;
        poses.push(current_pose);

        for joint in &self.joints {
            let local_tf = joint.local_transform(joint.current_position);
            current_pose *= local_tf;
            poses.push(current_pose);
        }
    }

    /// Evaluates Forward Kinematics (FK).
    /// Returns global poses for:
    /// - Index 0: Base transform
    /// - Index 1..=N: Frame after each joint (poses[N] is the end-effector).
    pub fn forward_kinematics(&self) -> Vec<Isometry3<f64>> {
        let mut poses = Vec::with_capacity(self.joints.len() + 1);
        self.forward_kinematics_into(&mut poses);
        poses
    }

    /// Evaluates Forward Kinematics for candidate joint values into an existing vector.
    pub fn forward_kinematics_with_q_into(
        &self,
        q_actuated: &[f64],
        poses: &mut Vec<Isometry3<f64>>,
    ) {
        poses.clear();
        poses.reserve(self.joints.len() + 1);
        let mut current_pose = self.base_transform;
        poses.push(current_pose);

        let mut q_idx = 0;
        for joint in &self.joints {
            let q = if joint.is_actuated() {
                let val = if q_idx < q_actuated.len() {
                    q_actuated[q_idx]
                } else {
                    joint.current_position
                };
                q_idx += 1;
                val
            } else {
                joint.current_position
            };

            let local_tf = joint.local_transform(q);
            current_pose *= local_tf;
            poses.push(current_pose);
        }
    }

    /// Evaluates Forward Kinematics for a given set of candidate joint values without mutating self.
    pub fn forward_kinematics_with_q(&self, q_actuated: &[f64]) -> Vec<Isometry3<f64>> {
        let mut poses = Vec::with_capacity(self.joints.len() + 1);
        self.forward_kinematics_with_q_into(q_actuated, &mut poses);
        poses
    }

    /// Returns the global positions of all joint origins and the end-effector.
    pub fn joint_positions_3d(&self) -> Vec<Point3<f64>> {
        self.forward_kinematics()
            .iter()
            .map(|iso| Point3::from(iso.translation.vector))
            .collect()
    }

    /// Returns the end-effector global transformation matrix.
    pub fn end_effector_pose(&self) -> Isometry3<f64> {
        let poses = self.forward_kinematics();
        *poses.last().unwrap_or(&self.base_transform)
    }

    /// Returns the end-effector position in world coordinates.
    pub fn end_effector_position(&self) -> Point3<f64> {
        Point3::from(self.end_effector_pose().translation.vector)
    }

    /// Returns the current active joint values (radians / meters).
    pub fn get_actuated_joint_positions(&self) -> Vec<f64> {
        self.joints
            .iter()
            .filter(|j| j.is_actuated())
            .map(|j| j.current_position)
            .collect()
    }

    /// Sets the active joint values, automatically clamping each to defined limits.
    pub fn set_actuated_joint_positions(&mut self, q: &[f64]) {
        let mut idx = 0;
        for joint in &mut self.joints {
            if joint.is_actuated() && idx < q.len() {
                joint.set_position(q[idx]);
                idx += 1;
            }
        }
    }

    /// Resets all joints to their designated home position.
    pub fn reset_to_home(&mut self) {
        for joint in &mut self.joints {
            joint.current_position = joint.home_position;
        }
    }

    /// Computes the estimated total reach envelope radius of the arm.
    pub fn total_reach(&self) -> f64 {
        let mut reach = 0.0;
        for joint in &self.joints {
            reach += joint.origin_translation.norm();
            if let Some((_, max)) = joint.limits {
                if joint.joint_type == JointType::Prismatic {
                    reach += max.abs();
                }
            }
        }
        reach.max(0.1)
    }

    /// Computes the 6xN Geometric Spatial Jacobian matrix into an existing matrix using precomputed FK poses.
    pub fn compute_jacobian_from_poses_into(
        &self,
        poses: &[Isometry3<f64>],
        jacobian: &mut DMatrix<f64>,
    ) {
        let dof = self.dof();
        if jacobian.nrows() != 6 || jacobian.ncols() != dof {
            *jacobian = DMatrix::zeros(6, dof);
        } else {
            jacobian.fill(0.0);
        }

        let p_end = poses.last().unwrap().translation.vector;

        let mut col = 0;
        for (i, joint) in self.joints.iter().enumerate() {
            if !joint.is_actuated() {
                continue;
            }

            let joint_frame = poses[i] * joint.origin_isometry();
            let p_i = joint_frame.translation.vector;
            let z_i = joint_frame.rotation * joint.axis.into_inner();

            match joint.joint_type {
                JointType::Revolute | JointType::Continuous => {
                    let jv = z_i.cross(&(p_end - p_i));
                    let jw = z_i;

                    jacobian[(0, col)] = jv.x;
                    jacobian[(1, col)] = jv.y;
                    jacobian[(2, col)] = jv.z;
                    jacobian[(3, col)] = jw.x;
                    jacobian[(4, col)] = jw.y;
                    jacobian[(5, col)] = jw.z;
                }
                JointType::Prismatic => {
                    jacobian[(0, col)] = z_i.x;
                    jacobian[(1, col)] = z_i.y;
                    jacobian[(2, col)] = z_i.z;
                }
                JointType::Fixed => {}
            }

            col += 1;
        }
    }

    /// Computes the 6xN Geometric Spatial Jacobian matrix using precomputed FK poses.
    pub fn compute_jacobian_from_poses(&self, poses: &[Isometry3<f64>]) -> DMatrix<f64> {
        let dof = self.dof();
        let mut jacobian = DMatrix::zeros(6, dof);
        self.compute_jacobian_from_poses_into(poses, &mut jacobian);
        jacobian
    }

    /// Computes the 6xN Geometric Spatial Jacobian matrix in world coordinates.
    /// Top 3 rows: Linear velocity contribution (Jv)
    /// Bottom 3 rows: Angular velocity contribution (Jw)
    pub fn compute_jacobian(&self) -> DMatrix<f64> {
        let poses = self.forward_kinematics();
        self.compute_jacobian_from_poses(&poses)
    }

    /// Computes the 3xN Position Jacobian matrix into an existing matrix using precomputed FK poses.
    pub fn compute_position_jacobian_from_poses_into(
        &self,
        poses: &[Isometry3<f64>],
        jacobian: &mut DMatrix<f64>,
    ) {
        let dof = self.dof();
        if jacobian.nrows() != 3 || jacobian.ncols() != dof {
            *jacobian = DMatrix::zeros(3, dof);
        } else {
            jacobian.fill(0.0);
        }

        let p_end = poses.last().unwrap().translation.vector;

        let mut col = 0;
        for (i, joint) in self.joints.iter().enumerate() {
            if !joint.is_actuated() {
                continue;
            }

            let joint_frame = poses[i] * joint.origin_isometry();
            let p_i = joint_frame.translation.vector;
            let z_i = joint_frame.rotation * joint.axis.into_inner();

            match joint.joint_type {
                JointType::Revolute | JointType::Continuous => {
                    let jv = z_i.cross(&(p_end - p_i));
                    jacobian[(0, col)] = jv.x;
                    jacobian[(1, col)] = jv.y;
                    jacobian[(2, col)] = jv.z;
                }
                JointType::Prismatic => {
                    jacobian[(0, col)] = z_i.x;
                    jacobian[(1, col)] = z_i.y;
                    jacobian[(2, col)] = z_i.z;
                }
                JointType::Fixed => {}
            }

            col += 1;
        }
    }

    /// Computes the 3xN Position Jacobian matrix using precomputed FK poses.
    pub fn compute_position_jacobian_from_poses(&self, poses: &[Isometry3<f64>]) -> DMatrix<f64> {
        let dof = self.dof();
        let mut jacobian = DMatrix::zeros(3, dof);
        self.compute_position_jacobian_from_poses_into(poses, &mut jacobian);
        jacobian
    }

    /// Computes the 3xN Position Jacobian in world coordinates.
    pub fn compute_position_jacobian(&self) -> DMatrix<f64> {
        let poses = self.forward_kinematics();
        self.compute_position_jacobian_from_poses(&poses)
    }
}
