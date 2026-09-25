use nalgebra::Vector3;
use roxmltree::Document;
use std::collections::HashMap;

use crate::kinematics::{Joint, JointType, Link, LinkGeometry, RobotArm};

/// Parses a 3-element float array from a space-separated string (e.g., "0.1 0.2 0.3").
fn parse_vec3(s: &str) -> Option<Vector3<f64>> {
    let parts: Vec<f64> = s
        .split_whitespace()
        .filter_map(|p| p.parse::<f64>().ok())
        .collect();
    if parts.len() == 3 {
        Some(Vector3::new(parts[0], parts[1], parts[2]))
    } else {
        None
    }
}

/// Parses a 4-element float array for RGBA color (e.g., "0.8 0.2 0.2 1.0").
fn parse_color(s: &str) -> Option<[f32; 4]> {
    let parts: Vec<f32> = s
        .split_whitespace()
        .filter_map(|p| p.parse::<f32>().ok())
        .collect();
    if parts.len() == 4 {
        Some([parts[0], parts[1], parts[2], parts[3]])
    } else {
        None
    }
}

/// Intermediate structure for parsed URDF links.
#[allow(dead_code)]
struct UrdfLinkData {
    name: String,
    geometry: Option<LinkGeometry>,
    color: [f32; 4],
}

/// Intermediate structure for parsed URDF joints.
struct UrdfJointData {
    name: String,
    joint_type: JointType,
    parent_link: String,
    child_link: String,
    origin_xyz: Vector3<f64>,
    origin_rpy: Vector3<f64>,
    axis: Vector3<f64>,
    limits: Option<(f64, f64)>,
}

/// Parses a URDF XML string and constructs a `RobotArm`.
pub fn parse_urdf(xml_content: &str) -> Result<RobotArm, String> {
    let doc = Document::parse(xml_content).map_err(|e| format!("XML parse error: {}", e))?;
    let robot_node = doc.root_element();

    if robot_node.tag_name().name() != "robot" {
        return Err("Root XML element must be <robot>".into());
    }

    let robot_name = robot_node.attribute("name").unwrap_or("unnamed_robot");
    let mut links_map: HashMap<String, UrdfLinkData> = HashMap::new();
    let mut joints_list: Vec<UrdfJointData> = Vec::new();

    for child in robot_node.children() {
        if !child.is_element() {
            continue;
        }

        match child.tag_name().name() {
            "link" => {
                let name = child.attribute("name").unwrap_or("link").to_string();
                let mut geometry = None;
                let mut color = [0.7, 0.7, 0.8, 1.0];

                if let Some(visual) = child.children().find(|n| n.tag_name().name() == "visual") {
                    if let Some(geom) = visual
                        .children()
                        .find(|n| n.tag_name().name() == "geometry")
                    {
                        for g in geom.children() {
                            match g.tag_name().name() {
                                "cylinder" => {
                                    let radius = g
                                        .attribute("radius")
                                        .and_then(|s| s.parse().ok())
                                        .unwrap_or(0.05);
                                    let length = g
                                        .attribute("length")
                                        .and_then(|s| s.parse().ok())
                                        .unwrap_or(0.2);
                                    geometry = Some(LinkGeometry::Cylinder { radius, length });
                                }
                                "box" => {
                                    let size_vec = g
                                        .attribute("size")
                                        .and_then(parse_vec3)
                                        .unwrap_or(Vector3::new(0.1, 0.1, 0.1));
                                    geometry = Some(LinkGeometry::Box {
                                        size: [size_vec.x, size_vec.y, size_vec.z],
                                    });
                                }
                                "sphere" => {
                                    let radius = g
                                        .attribute("radius")
                                        .and_then(|s| s.parse().ok())
                                        .unwrap_or(0.05);
                                    geometry = Some(LinkGeometry::Sphere { radius });
                                }
                                _ => {}
                            }
                        }
                    }

                    if let Some(material) = visual
                        .children()
                        .find(|n| n.tag_name().name() == "material")
                    {
                        if let Some(color_node) =
                            material.children().find(|n| n.tag_name().name() == "color")
                        {
                            if let Some(rgba) = color_node.attribute("rgba").and_then(parse_color) {
                                color = rgba;
                            }
                        }
                    }
                }

                links_map.insert(
                    name.clone(),
                    UrdfLinkData {
                        name,
                        geometry,
                        color,
                    },
                );
            }
            "joint" => {
                let name = child.attribute("name").unwrap_or("joint").to_string();
                let type_str = child.attribute("type").unwrap_or("fixed");
                let joint_type = match type_str {
                    "revolute" => JointType::Revolute,
                    "continuous" => JointType::Continuous,
                    "prismatic" => JointType::Prismatic,
                    _ => JointType::Fixed,
                };

                let parent_link = child
                    .children()
                    .find(|n| n.tag_name().name() == "parent")
                    .and_then(|n| n.attribute("link"))
                    .unwrap_or("")
                    .to_string();

                let child_link = child
                    .children()
                    .find(|n| n.tag_name().name() == "child")
                    .and_then(|n| n.attribute("link"))
                    .unwrap_or("")
                    .to_string();

                let (origin_xyz, origin_rpy) = if let Some(origin) =
                    child.children().find(|n| n.tag_name().name() == "origin")
                {
                    let xyz = origin
                        .attribute("xyz")
                        .and_then(parse_vec3)
                        .unwrap_or(Vector3::zeros());
                    let rpy = origin
                        .attribute("rpy")
                        .and_then(parse_vec3)
                        .unwrap_or(Vector3::zeros());
                    (xyz, rpy)
                } else {
                    (Vector3::zeros(), Vector3::zeros())
                };

                let axis = if let Some(axis_node) =
                    child.children().find(|n| n.tag_name().name() == "axis")
                {
                    axis_node
                        .attribute("xyz")
                        .and_then(parse_vec3)
                        .unwrap_or(Vector3::z())
                } else {
                    Vector3::z()
                };

                let limits = child
                    .children()
                    .find(|n| n.tag_name().name() == "limit")
                    .and_then(|lim| {
                        let lower = lim.attribute("lower").and_then(|s| s.parse::<f64>().ok());
                        let upper = lim.attribute("upper").and_then(|s| s.parse::<f64>().ok());
                        match (lower, upper) {
                            (Some(l), Some(u)) => Some((l, u)),
                            _ => None,
                        }
                    });

                joints_list.push(UrdfJointData {
                    name,
                    joint_type,
                    parent_link,
                    child_link,
                    origin_xyz,
                    origin_rpy,
                    axis,
                    limits,
                });
            }
            _ => {}
        }
    }

    if joints_list.is_empty() {
        return Err("URDF does not contain any valid joints".into());
    }

    // Determine root link (the link that is never a child)
    let child_links: std::collections::HashSet<String> =
        joints_list.iter().map(|j| j.child_link.clone()).collect();
    let root_link = joints_list
        .iter()
        .map(|j| &j.parent_link)
        .find(|p| !child_links.contains(*p))
        .cloned()
        .unwrap_or_else(|| joints_list[0].parent_link.clone());

    let mut robot = RobotArm::new(robot_name);

    // Chain traversal starting from root_link
    let mut current_link = root_link;
    let palette = [
        [0.2, 0.6, 0.86, 1.0],  // Azure
        [0.9, 0.4, 0.2, 1.0],   // Coral
        [0.2, 0.8, 0.4, 1.0],   // Emerald
        [0.8, 0.3, 0.7, 1.0],   // Purple
        [0.95, 0.75, 0.1, 1.0], // Gold
        [0.3, 0.8, 0.9, 1.0],   // Cyan
    ];

    let mut color_idx = 0;
    while let Some(joint_data) = joints_list.iter().find(|j| j.parent_link == current_link) {
        let child_data = links_map.get(&joint_data.child_link);
        let link_color = child_data
            .map(|d| d.color)
            .unwrap_or(palette[color_idx % palette.len()]);
        color_idx += 1;

        let link = Link::new(&joint_data.child_link, link_color);

        let joint = match joint_data.joint_type {
            JointType::Revolute => Joint::new_revolute(
                &joint_data.name,
                joint_data.origin_xyz,
                joint_data.origin_rpy,
                joint_data.axis,
                joint_data.limits,
            ),
            JointType::Continuous => {
                let mut j = Joint::new_revolute(
                    &joint_data.name,
                    joint_data.origin_xyz,
                    joint_data.origin_rpy,
                    joint_data.axis,
                    None,
                );
                j.joint_type = JointType::Continuous;
                j
            }
            JointType::Prismatic => Joint::new_prismatic(
                &joint_data.name,
                joint_data.origin_xyz,
                joint_data.origin_rpy,
                joint_data.axis,
                joint_data.limits,
            ),
            JointType::Fixed => Joint::new_fixed(
                &joint_data.name,
                joint_data.origin_xyz,
                joint_data.origin_rpy,
            ),
        };

        current_link = joint_data.child_link.clone();
        robot.add_joint_and_link(joint, link);
    }

    if robot.joints.is_empty() {
        return Err("Failed to assemble kinematic chain from URDF joints".into());
    }

    Ok(robot)
}
