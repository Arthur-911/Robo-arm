use nalgebra::{Matrix3, Vector3};

use super::RobotArm;

/// Evaluates Yoshikawa Manipulability metrics and 3D ellipsoid semi-axes.
#[derive(Debug, Clone)]
pub struct ManipulabilityData {
    /// Yoshikawa manipulability index w = sqrt(det(J * J^T)).
    pub score: f64,
    /// 3D principal semi-axis lengths (sigma_1 >= sigma_2 >= sigma_3).
    pub semi_axes: [f64; 3],
    /// 3D principal axis direction unit vectors.
    pub axes_dirs: [Vector3<f64>; 3],
    /// Indicates whether the current pose is in or near a kinematic singularity.
    pub is_near_singularity: bool,
}

/// Computes the linear velocity manipulability ellipsoid from the position Jacobian.
pub fn compute_manipulability(robot: &RobotArm) -> ManipulabilityData {
    let j_pos = robot.compute_position_jacobian();
    let a_dyn = &j_pos * j_pos.transpose();
    let a = Matrix3::new(
        a_dyn[(0, 0)],
        a_dyn[(0, 1)],
        a_dyn[(0, 2)],
        a_dyn[(1, 0)],
        a_dyn[(1, 1)],
        a_dyn[(1, 2)],
        a_dyn[(2, 0)],
        a_dyn[(2, 1)],
        a_dyn[(2, 2)],
    );

    let det_a = a.determinant().max(0.0);
    let score = det_a.sqrt();

    // Eigen-decomposition of 3x3 symmetric matrix A
    let eigen = a.symmetric_eigen();
    let eigenvalues = eigen.eigenvalues;
    let eigenvectors = eigen.eigenvectors;

    // Sort eigenvalues descending
    let mut indices = [0, 1, 2];
    indices.sort_by(|&i, &j| {
        eigenvalues[j]
            .partial_cmp(&eigenvalues[i])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut semi_axes = [0.0; 3];
    let mut axes_dirs = [Vector3::x(), Vector3::y(), Vector3::z()];

    for (k, &idx) in indices.iter().enumerate() {
        let val = eigenvalues[idx].max(0.0);
        semi_axes[k] = val.sqrt();
        let col = eigenvectors.column(idx);
        let dir = Vector3::new(col[0], col[1], col[2]);
        let norm = dir.norm();
        axes_dirs[k] = if norm > 1e-6 {
            dir / norm
        } else if k == 0 {
            Vector3::x()
        } else if k == 1 {
            Vector3::y()
        } else {
            Vector3::z()
        };
    }

    let is_near_singularity = score < 0.015 || semi_axes[2] < 0.02;

    ManipulabilityData {
        score,
        semi_axes,
        axes_dirs,
        is_near_singularity,
    }
}
