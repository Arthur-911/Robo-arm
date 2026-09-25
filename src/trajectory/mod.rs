//! Trajectory generation, polynomial interpolation (cubic & quintic minimum-jerk), and execution controller.

pub mod controller;
pub mod planner;
pub mod polynomial;

pub use controller::{JointTrajectorySmoother, TrajectoryController};
pub use planner::{Segment3D, TrajectoryContinuityMode, TrajectoryPlanner, Waypoint};
pub use polynomial::{CubicPolynomial, PolynomialType, QuinticPolynomial};
