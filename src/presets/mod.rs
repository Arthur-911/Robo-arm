use nalgebra::Vector3;

use crate::kinematics::{Joint, Link, RobotArm};

pub mod sample_urdf;
pub use sample_urdf::*;

/// Constructs a classic 2D 3-DOF Planar articulated arm.
pub fn planar_3dof() -> RobotArm {
    let mut arm = RobotArm::new("Planar 3-DOF Arm");

    // Joint 1: Base revolute around Z
    arm.add_joint_and_link(
        Joint::new_revolute(
            "shoulder",
            Vector3::new(0.0, 0.0, 0.05),
            Vector3::zeros(),
            Vector3::z(),
            Some((-std::f64::consts::PI * 0.95, std::f64::consts::PI * 0.95)),
        ),
        Link::new("link_1", [0.95, 0.75, 0.10, 1.0]).with_cylinder(0.045, 0.4),
    );

    // Joint 2: Elbow revolute around Z
    arm.add_joint_and_link(
        Joint::new_revolute(
            "elbow",
            Vector3::new(0.4, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::z(),
            Some((-std::f64::consts::PI * 0.9, std::f64::consts::PI * 0.9)),
        ),
        Link::new("link_2", [0.95, 0.75, 0.10, 1.0]).with_cylinder(0.038, 0.3),
    );

    // Joint 3: Wrist revolute around Z
    arm.add_joint_and_link(
        Joint::new_revolute(
            "wrist",
            Vector3::new(0.3, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::z(),
            Some((-std::f64::consts::PI * 0.9, std::f64::consts::PI * 0.9)),
        ),
        Link::new("link_3", [0.24, 0.26, 0.30, 1.0]).with_cylinder(0.032, 0.2),
    );

    // End-effector virtual tip
    arm.add_joint_and_link(
        Joint::new_fixed("ee_fixed", Vector3::new(0.2, 0.0, 0.0), Vector3::zeros()),
        Link::new("ee_tip", [0.20, 0.85, 0.75, 1.0]).with_sphere(0.03),
    );

    arm
}

/// Constructs a 4-DOF SCARA (Selective Compliance Articulated Robot Arm) manipulator.
pub fn scara_4dof() -> RobotArm {
    let mut arm = RobotArm::new("SCARA 4-DOF Robot");

    // Joint 1: Base yaw (Revolute Z)
    arm.add_joint_and_link(
        Joint::new_revolute(
            "j1_yaw",
            Vector3::new(0.0, 0.0, 0.25),
            Vector3::zeros(),
            Vector3::z(),
            Some((-std::f64::consts::PI * 0.9, std::f64::consts::PI * 0.9)),
        ),
        Link::new("arm_link_1", [0.93, 0.94, 0.96, 1.0]).with_cylinder(0.055, 0.35),
    );

    // Joint 2: Elbow yaw (Revolute Z)
    arm.add_joint_and_link(
        Joint::new_revolute(
            "j2_yaw",
            Vector3::new(0.35, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::z(),
            Some((-std::f64::consts::PI * 0.85, std::f64::consts::PI * 0.85)),
        ),
        Link::new("arm_link_2", [0.15, 0.50, 0.86, 1.0]).with_cylinder(0.048, 0.30),
    );

    // Joint 3: Quill vertical slide (Prismatic -Z)
    arm.add_joint_and_link(
        Joint::new_prismatic(
            "j3_z_slide",
            Vector3::new(0.30, 0.0, 0.0),
            Vector3::zeros(),
            -Vector3::z(),
            Some((0.0, 0.25)),
        ),
        Link::new("quill_link", [0.86, 0.89, 0.94, 1.0]).with_cylinder(0.024, 0.25),
    );

    // Joint 4: Quill roll (Revolute Z)
    arm.add_joint_and_link(
        Joint::new_revolute(
            "j4_roll",
            Vector3::new(0.0, 0.0, -0.05),
            Vector3::zeros(),
            Vector3::z(),
            Some((-std::f64::consts::PI, std::f64::consts::PI)),
        ),
        Link::new("gripper_link", [0.22, 0.25, 0.30, 1.0]).with_box([0.04, 0.08, 0.04]),
    );

    arm
}

/// Constructs a 6-DOF Industrial Manipulator (UR5/Puma 560 style with spherical wrist).
pub fn industrial_6dof() -> RobotArm {
    let mut arm = RobotArm::new("Industrial 6-DOF Manipulator");

    // Base waist yaw
    arm.add_joint_and_link(
        Joint::new_revolute(
            "base_yaw",
            Vector3::new(0.0, 0.0, 0.15),
            Vector3::zeros(),
            Vector3::z(),
            Some((-std::f64::consts::PI, std::f64::consts::PI)),
        ),
        Link::new("base_column", [0.22, 0.24, 0.28, 1.0]).with_cylinder(0.07, 0.15),
    );

    // Shoulder pitch
    arm.add_joint_and_link(
        Joint::new_revolute(
            "shoulder_pitch",
            Vector3::new(0.0, 0.1, 0.1),
            Vector3::zeros(),
            Vector3::y(),
            Some((-std::f64::consts::PI * 0.8, std::f64::consts::PI * 0.8)),
        ),
        Link::new("upper_arm", [0.96, 0.44, 0.08, 1.0]).with_cylinder(0.055, 0.42),
    );

    // Elbow pitch
    arm.add_joint_and_link(
        Joint::new_revolute(
            "elbow_pitch",
            Vector3::new(0.0, -0.1, 0.42),
            Vector3::zeros(),
            Vector3::y(),
            Some((-std::f64::consts::PI * 0.9, std::f64::consts::PI * 0.9)),
        ),
        Link::new("forearm", [0.96, 0.44, 0.08, 1.0]).with_cylinder(0.045, 0.38),
    );

    // Wrist 1: Pitch
    arm.add_joint_and_link(
        Joint::new_revolute(
            "wrist_1_pitch",
            Vector3::new(0.0, 0.1, 0.38),
            Vector3::zeros(),
            Vector3::y(),
            Some((-std::f64::consts::PI, std::f64::consts::PI)),
        ),
        Link::new("wrist_1", [0.24, 0.26, 0.30, 1.0]).with_cylinder(0.04, 0.1),
    );

    // Wrist 2: Yaw
    arm.add_joint_and_link(
        Joint::new_revolute(
            "wrist_2_yaw",
            Vector3::new(0.0, 0.1, 0.0),
            Vector3::zeros(),
            Vector3::z(),
            Some((-std::f64::consts::PI, std::f64::consts::PI)),
        ),
        Link::new("wrist_2", [0.96, 0.44, 0.08, 1.0]).with_cylinder(0.035, 0.1),
    );

    // Wrist 3: Roll
    arm.add_joint_and_link(
        Joint::new_revolute(
            "wrist_3_roll",
            Vector3::new(0.0, 0.0, 0.1),
            Vector3::zeros(),
            Vector3::x(),
            Some((-std::f64::consts::PI, std::f64::consts::PI)),
        ),
        Link::new("flange", [0.85, 0.88, 0.93, 1.0]).with_cylinder(0.03, 0.05),
    );

    // End effector tool point
    arm.add_joint_and_link(
        Joint::new_fixed("tool0", Vector3::new(0.08, 0.0, 0.0), Vector3::zeros()),
        Link::new("tcp", [0.20, 0.85, 0.75, 1.0]).with_sphere(0.02),
    );

    arm
}

/// Constructs a 7-DOF Redundant Manipulator (KUKA LBR iiwa style).
pub fn redundant_7dof() -> RobotArm {
    let mut arm = RobotArm::new("Redundant 7-DOF Robot (iiwa style)");

    let colors = [
        [0.9, 0.3, 0.2, 1.0],   // Red
        [0.95, 0.6, 0.15, 1.0], // Orange
        [0.95, 0.8, 0.1, 1.0],  // Yellow
        [0.2, 0.75, 0.4, 1.0],  // Green
        [0.2, 0.6, 0.85, 1.0],  // Blue
        [0.6, 0.3, 0.85, 1.0],  // Purple
        [0.85, 0.2, 0.6, 1.0],  // Magenta
    ];

    let axes = [
        Vector3::z(),
        Vector3::y(),
        Vector3::z(),
        Vector3::y(),
        Vector3::z(),
        Vector3::y(),
        Vector3::z(),
    ];

    let offsets = [
        Vector3::new(0.0, 0.0, 0.18),
        Vector3::new(0.0, 0.0, 0.16),
        Vector3::new(0.0, 0.0, 0.18),
        Vector3::new(0.0, 0.0, 0.16),
        Vector3::new(0.0, 0.0, 0.18),
        Vector3::new(0.0, 0.0, 0.14),
        Vector3::new(0.0, 0.0, 0.12),
    ];

    for i in 0..7 {
        arm.add_joint_and_link(
            Joint::new_revolute(
                format!("joint_{}", i + 1),
                offsets[i],
                Vector3::zeros(),
                axes[i],
                Some((-2.9, 2.9)),
            ),
            Link::new(format!("link_{}", i + 1), colors[i])
                .with_cylinder(0.05 - (i as f64 * 0.003), offsets[i].z),
        );
    }

    arm.add_joint_and_link(
        Joint::new_fixed("flange", Vector3::new(0.0, 0.0, 0.06), Vector3::zeros()),
        Link::new("end_tip", [1.0, 1.0, 1.0, 1.0]).with_sphere(0.025),
    );

    arm
}
