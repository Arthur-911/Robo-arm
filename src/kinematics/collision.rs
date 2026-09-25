use nalgebra::{Point3, Vector3};

use super::{LinkGeometry, RobotArm};

/// A cuboid obstacle in the robotic workspace.
#[derive(Debug, Clone)]
pub struct ObstacleBox {
    pub name: String,
    pub center: Point3<f64>,
    pub half_size: Vector3<f64>,
    pub color: [f32; 4],
}

impl ObstacleBox {
    pub fn new(name: impl Into<String>, center: Point3<f64>, half_size: Vector3<f64>) -> Self {
        Self {
            name: name.into(),
            center,
            half_size,
            color: [0.95, 0.35, 0.25, 0.85],
        }
    }
}

/// A cylindrical capsule bounding volume for a robot link.
#[derive(Debug, Clone)]
pub struct LinkCapsule {
    pub link_index: usize,
    pub start: Point3<f64>,
    pub end: Point3<f64>,
    pub radius: f64,
}

/// Detailed collision detection diagnostic report.
#[derive(Debug, Clone, Default)]
pub struct CollisionReport {
    pub in_collision: bool,
    pub colliding_links: Vec<usize>,
    pub details: Vec<String>,
}

/// Extracts bounding capsules for each robot link in the current configuration into an existing vector.
pub fn get_robot_link_capsules_into(robot: &RobotArm, capsules: &mut Vec<LinkCapsule>, poses: &mut Vec<nalgebra::Isometry3<f64>>) {
    robot.forward_kinematics_into(poses);
    capsules.clear();
    let num_links = robot.links.len();
    capsules.reserve(num_links);

    for i in 0..poses.len().saturating_sub(1) {
        let start = Point3::from(poses[i].translation.vector);
        let end = Point3::from(poses[i + 1].translation.vector);

        let r = if i < num_links {
            if let Some(LinkGeometry::Cylinder { radius, .. }) = robot.links[i].visual_geometry {
                radius
            } else {
                (0.060 * 0.93_f64.powi(i as i32)).clamp(0.032, 0.075)
            }
        } else {
            0.035
        };

        capsules.push(LinkCapsule {
            link_index: i,
            start,
            end,
            radius: r,
        });
    }
}

/// Extracts bounding capsules for each robot link in the current configuration.
pub fn get_robot_link_capsules(robot: &RobotArm) -> Vec<LinkCapsule> {
    let mut capsules = Vec::with_capacity(robot.links.len());
    let mut poses = Vec::with_capacity(robot.joints.len() + 1);
    get_robot_link_capsules_into(robot, &mut capsules, &mut poses);
    capsules
}

/// Computes the exact shortest distance between a 3D line segment and an axis-aligned box.
/// Evaluates exact critical slab transition points and segment endpoints.
pub fn segment_to_box_distance(
    p0: Point3<f64>,
    p1: Point3<f64>,
    box_center: Point3<f64>,
    half_size: Vector3<f64>,
) -> (f64, Point3<f64>) {
    let b_min = box_center - half_size;
    let b_max = box_center + half_size;
    let dir = p1 - p0;

    // Collect exact candidate t values (endpoints, midpoint, and face boundary crossings)
    let mut candidates = [0.0; 9];
    let mut count = 0;

    candidates[count] = 0.0; count += 1;
    candidates[count] = 0.5; count += 1;
    candidates[count] = 1.0; count += 1;

    for k in 0..3 {
        let d_k = dir[k];
        if d_k.abs() > 1e-6 {
            let t_min = (b_min[k] - p0[k]) / d_k;
            if t_min > 0.0 && t_min < 1.0 {
                candidates[count] = t_min;
                count += 1;
            }
            let t_max = (b_max[k] - p0[k]) / d_k;
            if t_max > 0.0 && t_max < 1.0 {
                candidates[count] = t_max;
                count += 1;
            }
        }
    }

    let mut min_dist_sq = f64::MAX;
    let mut closest_point_on_box = box_center;

    for &t in &candidates[..count] {
        let pt = p0 + dir * t;
        let clamped_x = pt.x.clamp(b_min.x, b_max.x);
        let clamped_y = pt.y.clamp(b_min.y, b_max.y);
        let clamped_z = pt.z.clamp(b_min.z, b_max.z);

        let box_pt = Point3::new(clamped_x, clamped_y, clamped_z);
        let dist_sq = (pt - box_pt).norm_squared();

        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
            closest_point_on_box = box_pt;
        }
    }

    (min_dist_sq.sqrt(), closest_point_on_box)
}

/// Computes the shortest distance between two 3D line segments.
pub fn segment_to_segment_distance(
    p1: Point3<f64>,
    p2: Point3<f64>,
    p3: Point3<f64>,
    p4: Point3<f64>,
) -> f64 {
    let u = p2 - p1;
    let v = p4 - p3;
    let w = p1 - p3;

    let a = u.dot(&u);
    let b = u.dot(&v);
    let c = v.dot(&v);
    let d = u.dot(&w);
    let e = v.dot(&w);

    let d_denom = a * c - b * b;
    let sc: f64;
    let mut s_n: f64;
    let mut s_d = d_denom;
    let tc: f64;
    let mut t_n: f64;
    let mut t_d = d_denom;

    if d_denom < 1e-6 {
        s_n = 0.0;
        s_d = 1.0;
        t_n = e;
        t_d = c;
    } else {
        s_n = b * e - c * d;
        t_n = a * e - b * d;
        if s_n < 0.0 {
            s_n = 0.0;
            t_n = e;
            t_d = c;
        } else if s_n > s_d {
            s_n = s_d;
            t_n = e + b;
            t_d = c;
        }
    }

    if t_n < 0.0 {
        tc = 0.0;
        if -d < 0.0 {
            sc = 0.0;
        } else if -d > a {
            sc = 1.0;
        } else {
            sc = -d / a;
        }
    } else if t_n > t_d {
        tc = 1.0;
        if (-d + b) < 0.0 {
            sc = 0.0;
        } else if (-d + b) > a {
            sc = 1.0;
        } else {
            sc = (-d + b) / a;
        }
    } else {
        tc = if t_d.abs() < 1e-6 { 0.0 } else { t_n / t_d };
        sc = if s_d.abs() < 1e-6 { 0.0 } else { s_n / s_d };
    }

    let d_p = w + u * sc - v * tc;
    d_p.norm()
}

/// Evaluates self-collisions and obstacle collisions for the robot.
pub fn check_collisions(robot: &RobotArm, obstacles: &[ObstacleBox]) -> CollisionReport {
    let mut report = CollisionReport::default();
    let capsules = get_robot_link_capsules(robot);
    let n = capsules.len();

    // 1. Check Link-to-Obstacle Collisions
    for cap in &capsules {
        for obs in obstacles {
            let (dist, _) = segment_to_box_distance(cap.start, cap.end, obs.center, obs.half_size);
            if dist <= cap.radius {
                report.in_collision = true;
                if !report.colliding_links.contains(&cap.link_index) {
                    report.colliding_links.push(cap.link_index);
                }
                report.details.push(format!(
                    "Link {} collision with obstacle '{}' (dist: {:.3}m, radius: {:.3}m)",
                    cap.link_index + 1,
                    obs.name,
                    dist,
                    cap.radius
                ));
            }
        }
    }

    // 2. Check Link-to-Link Self-Collisions (Non-adjacent links)
    for i in 0..n {
        for j in (i + 2)..n {
            let dist = segment_to_segment_distance(
                capsules[i].start,
                capsules[i].end,
                capsules[j].start,
                capsules[j].end,
            );
            let combined_radius = capsules[i].radius + capsules[j].radius;
            if dist <= combined_radius {
                report.in_collision = true;
                if !report.colliding_links.contains(&capsules[i].link_index) {
                    report.colliding_links.push(capsules[i].link_index);
                }
                if !report.colliding_links.contains(&capsules[j].link_index) {
                    report.colliding_links.push(capsules[j].link_index);
                }
                report.details.push(format!(
                    "Self-collision: Link {} intersects Link {} (dist: {:.3}m)",
                    capsules[i].link_index + 1,
                    capsules[j].link_index + 1,
                    dist
                ));
            }
        }
    }

    report
}
