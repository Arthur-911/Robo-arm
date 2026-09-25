//! Trajectory generation, polynomial interpolation (cubic & quintic minimum-jerk), and execution controller.

pub mod controller;
pub mod planner;
pub mod polynomial;
pub mod recorder;
pub mod rrt;

pub use controller::{JointTrajectorySmoother, TrajectoryController};
pub use planner::{Segment3D, TrajectoryContinuityMode, TrajectoryPlanner, Waypoint};
pub use polynomial::{CubicPolynomial, PolynomialType, QuinticPolynomial};
pub use recorder::{TrajectoryFrame, TrajectoryRecorder, TrajectoryRecording};
pub use rrt::{
    is_edge_valid, plan_cartesian_rrt, plan_rrt, plan_rrt_connect, plan_standard_rrt,
    shortcut_path, RrtAlgorithm, RrtParams, RrtPlanResult, SimpleRng,
};
