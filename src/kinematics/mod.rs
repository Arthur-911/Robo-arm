//! Robot kinematics module: forward kinematics, spatial Jacobians, and inverse kinematics solvers.

pub mod collision;
pub mod dynamics;
pub mod ik_fabrik;
pub mod ik_jacobian;
pub mod joint;
pub mod link;
pub mod manipulability;
pub mod robot;
pub mod solver;

pub use collision::{
    check_collisions, check_collisions_for_q, get_robot_link_capsules_from_poses,
    is_configuration_valid, CollisionReport, LinkCapsule, ObstacleBox,
};
pub use dynamics::{compute_gravity_torques, JointDynamicsReport};
pub use ik_fabrik::solve_fabrik_ik;
pub use ik_jacobian::solve_jacobian_ik;
pub use joint::{Joint, JointType};
pub use link::{Link, LinkGeometry};
pub use manipulability::{compute_manipulability, ManipulabilityData};
pub use robot::RobotArm;
pub use solver::{IKSolution, IKSolverMode, IKSolverParams, IKSolverType};

use nalgebra::{Point3, UnitQuaternion};

/// Unified dispatch function for Inverse Kinematics solving.
pub fn solve_ik(
    robot: &RobotArm,
    target_pos: Point3<f64>,
    target_rot: Option<UnitQuaternion<f64>>,
    solver_type: IKSolverType,
    mode: IKSolverMode,
    params: &IKSolverParams,
) -> IKSolution {
    match solver_type {
        IKSolverType::FABRIK => solve_fabrik_ik(robot, target_pos, params),
        IKSolverType::JacobianDLS | IKSolverType::JacobianTranspose => {
            solve_jacobian_ik(robot, target_pos, target_rot, mode, solver_type, params)
        }
    }
}
