use crate::kinematics::{JointType, RobotArm};

/// Serializes a `RobotArm` kinematic chain into standard URDF XML format.
pub fn export_urdf(robot: &RobotArm) -> String {
    let mut xml = String::new();
    xml.push_str("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
    xml.push_str(&format!("<robot name=\"{}\">\n", robot.name));

    // Base link
    xml.push_str("  <link name=\"base_link\">\n");
    xml.push_str("    <visual>\n");
    xml.push_str("      <geometry>\n");
    xml.push_str("        <cylinder radius=\"0.08\" length=\"0.05\"/>\n");
    xml.push_str("      </geometry>\n");
    xml.push_str("      <material name=\"base_mat\">\n");
    xml.push_str("        <color rgba=\"0.3 0.3 0.35 1.0\"/>\n");
    xml.push_str("      </material>\n");
    xml.push_str("    </visual>\n");
    xml.push_str("  </link>\n\n");

    let mut parent_link = "base_link".to_string();

    for (i, (joint, link)) in robot.joints.iter().zip(robot.links.iter()).enumerate() {
        let child_link = if link.name.is_empty() {
            format!("link_{}", i + 1)
        } else {
            link.name.clone()
        };

        // Link definition
        xml.push_str(&format!("  <link name=\"{}\">\n", child_link));
        xml.push_str("    <visual>\n");
        xml.push_str("      <geometry>\n");
        let len = joint.origin_translation.norm().max(0.1);
        xml.push_str(&format!(
            "        <cylinder radius=\"0.04\" length=\"{:.4}\"/>\n",
            len
        ));
        xml.push_str("      </geometry>\n");
        xml.push_str(&format!("      <material name=\"mat_{}\">\n", child_link));
        xml.push_str(&format!(
            "        <color rgba=\"{:.2} {:.2} {:.2} {:.2}\"/>\n",
            link.color[0], link.color[1], link.color[2], link.color[3]
        ));
        xml.push_str("      </material>\n");
        xml.push_str("    </visual>\n");
        xml.push_str("  </link>\n\n");

        // Joint definition
        let type_str = match joint.joint_type {
            JointType::Revolute => "revolute",
            JointType::Continuous => "continuous",
            JointType::Prismatic => "prismatic",
            JointType::Fixed => "fixed",
        };

        xml.push_str(&format!(
            "  <joint name=\"{}\" type=\"{}\">\n",
            joint.name, type_str
        ));
        xml.push_str(&format!("    <parent link=\"{}\"/>\n", parent_link));
        xml.push_str(&format!("    <child link=\"{}\"/>\n", child_link));
        xml.push_str(&format!(
            "    <origin xyz=\"{:.4} {:.4} {:.4}\" rpy=\"{:.4} {:.4} {:.4}\"/>\n",
            joint.origin_translation.x,
            joint.origin_translation.y,
            joint.origin_translation.z,
            joint.origin_rpy.x,
            joint.origin_rpy.y,
            joint.origin_rpy.z
        ));

        let ax = joint.axis.into_inner();
        xml.push_str(&format!(
            "    <axis xyz=\"{:.4} {:.4} {:.4}\"/>\n",
            ax.x, ax.y, ax.z
        ));

        if let Some((min, max)) = joint.limits {
            xml.push_str(&format!(
                "    <limit lower=\"{:.4}\" upper=\"{:.4}\" effort=\"100.0\" velocity=\"3.14\"/>\n",
                min, max
            ));
        }

        xml.push_str("  </joint>\n\n");

        parent_link = child_link;
    }

    xml.push_str("</robot>\n");
    xml
}
