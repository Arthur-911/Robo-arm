use nalgebra::{Point3, Vector3};

use super::RobotArm;

/// Static joint torque and motor strain analysis under gravity.
#[derive(Debug, Clone)]
pub struct JointDynamicsReport {
    /// Static gravity torque on each joint (in Newton-meters, Nm).
    pub joint_torques_nm: Vec<f64>,
    /// Nominal rated continuous torque capacity for each joint (Nm).
    pub rated_capacities_nm: Vec<f64>,
    /// Motor load percentage (0.0 to 100%+).
    pub load_percentages: Vec<f64>,
    /// Maximum load percentage observed across all joints.
    pub peak_load_pct: f64,
}

/// Computes static joint gravity torques and motor load percentages.
pub fn compute_gravity_torques(robot: &RobotArm, payload_mass_kg: f64) -> JointDynamicsReport {
    let poses = robot.forward_kinematics();
    let num_joints = robot.joints.len();
    let g_accel = Vector3::new(0.0, 0.0, -9.80665); // Standard gravity vector (m/s^2)

    let mut joint_torques = vec![0.0; num_joints];
    let mut rated_capacities = Vec::with_capacity(num_joints);

    // Default continuous torque limits based on actuator position in kinematic chain
    for j in 0..num_joints {
        let capacity = match j {
            0 => 180.0, // Base yaw motor (Nm)
            1 => 220.0, // Shoulder pitch motor (Nm)
            2 => 140.0, // Elbow pitch motor (Nm)
            3 => 65.0,  // Wrist 1 motor (Nm)
            4 => 45.0,  // Wrist 2 motor (Nm)
            _ => 30.0,  // Wrist 3 / Tool motor (Nm)
        };
        rated_capacities.push(capacity);
    }

    // Determine center of mass and gravity force for each link
    let mut com_positions = Vec::with_capacity(num_joints);
    let mut link_forces = Vec::with_capacity(num_joints);

    for i in 0..num_joints {
        let p_start = Point3::from(poses[i].translation.vector);
        let p_end = if i + 1 < poses.len() {
            Point3::from(poses[i + 1].translation.vector)
        } else {
            p_start
        };
        // Link CoM at midpoint of link segment
        let com = Point3::from((p_start.coords + p_end.coords) * 0.5);
        com_positions.push(com);

        let link_mass = if i < robot.links.len() {
            robot.links[i].mass.max(0.2)
        } else {
            1.0
        };

        // Add payload mass to last link (tool / end-effector)
        let total_mass = if i == num_joints - 1 {
            link_mass + payload_mass_kg
        } else {
            link_mass
        };

        link_forces.push(g_accel * total_mass);
    }

    // Compute torque on each joint j from all downstream links k >= j
    for j in 0..num_joints {
        let p_joint = Point3::from(poses[j].translation.vector);
        let a_local = robot.joints[j].axis.into_inner();
        let a_world = poses[j].rotation * a_local;
        let axis = if a_world.norm() > 1e-4 {
            a_world.normalize()
        } else {
            Vector3::z()
        };

        let mut total_torque_vector = Vector3::zeros();

        for k in j..num_joints {
            let r_arm = com_positions[k] - p_joint;
            let force = link_forces[k];
            // Torque = r x F
            let link_torque = r_arm.cross(&force);
            total_torque_vector += link_torque;
        }

        // Project total torque vector onto joint actuation axis: tau = - (tau_g . axis)
        let tau = -total_torque_vector.dot(&axis);
        joint_torques[j] = tau;
    }

    let mut load_percentages = Vec::with_capacity(num_joints);
    let mut peak_load_pct = 0.0;

    for j in 0..num_joints {
        let pct = (joint_torques[j].abs() / rated_capacities[j]) * 100.0;
        if pct > peak_load_pct {
            peak_load_pct = pct;
        }
        load_percentages.push(pct);
    }

    JointDynamicsReport {
        joint_torques_nm: joint_torques,
        rated_capacities_nm: rated_capacities,
        load_percentages,
        peak_load_pct,
    }
}
