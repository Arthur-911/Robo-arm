use serde::{Deserialize, Serialize};

/// Mode of IK target matching.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IKSolverMode {
    /// Match 3D end-effector position (X, Y, Z).
    PositionOnly,
    /// Match full 6D end-effector pose (Position + Orientation).
    FullPose,
}

/// Selector for the active inverse kinematics algorithm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IKSolverType {
    /// Iterative Forward And Backward Reaching Inverse Kinematics.
    FABRIK,
    /// Damped Least Squares (Levenberg-Marquardt) Jacobian solver.
    JacobianDLS,
    /// Jacobian Transpose solver.
    JacobianTranspose,
}

/// Configuration parameters for inverse kinematics solvers.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IKSolverParams {
    /// Maximum allowed iterations per solve cycle.
    pub max_iterations: usize,
    /// Position tolerance in meters (convergence threshold).
    pub tolerance: f64,
    /// Orientation tolerance in radians.
    pub orientation_tolerance: f64,
    /// Damping factor lambda for DLS singularity handling.
    pub damping: f64,
    /// Gradient step scale factor (learning rate).
    pub step_size: f64,
    /// Whether to enable null-space projection for joint limit avoidance.
    pub enable_null_space_centering: bool,
    /// Null-space centering weight factor.
    pub null_space_weight: f64,
    /// Whether to dynamically scale damping near kinematic singularities (Singularity-Robust DLS).
    #[serde(default = "default_true")]
    pub adaptive_damping: bool,
    /// Yoshikawa manipulability threshold below which adaptive damping engages.
    #[serde(default = "default_singularity_threshold")]
    pub singularity_threshold: f64,
    /// Peak damping value applied at deep singularity configurations.
    #[serde(default = "default_max_damping")]
    pub max_damping: f64,
    /// Whether to use backtracking line-search to guarantee monotonic error reduction.
    #[serde(default = "default_true")]
    pub enable_line_search: bool,
}

fn default_true() -> bool {
    true
}

fn default_singularity_threshold() -> f64 {
    0.03
}

fn default_max_damping() -> f64 {
    0.25
}

impl Default for IKSolverParams {
    fn default() -> Self {
        Self {
            max_iterations: 60,
            tolerance: 1e-3,             // 1 millimeter
            orientation_tolerance: 1e-2, // ~0.57 degrees
            damping: 0.05,
            step_size: 0.8,
            enable_null_space_centering: true,
            null_space_weight: 0.1,
            adaptive_damping: true,
            singularity_threshold: 0.03,
            max_damping: 0.25,
            enable_line_search: true,
        }
    }
}

/// Performance and convergence report returned by an IK solve invocation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IKSolution {
    pub joint_positions: Vec<f64>,
    pub converged: bool,
    pub iterations: usize,
    pub residual_position_error: f64,
    pub residual_orientation_error: f64,
    pub solve_time_us: u128,
}
