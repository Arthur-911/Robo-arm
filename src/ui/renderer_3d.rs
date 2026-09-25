use egui::{Color32, FontId, Pos2, Rect, Shape, Stroke};
use nalgebra::{Point3, Vector3};

use super::camera::OrbitCamera;
use crate::kinematics::{CollisionReport, LinkGeometry, ManipulabilityData, RobotArm};
use crate::trajectory::TrajectoryPlanner;
use crate::workcell::{EOATType, ToolState, WorkcellEnvironment, WorkpieceManager, WorkpieceShape};

/// Active interaction mode with the 3D viewport canvas.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GizmoDragAxis {
    None,
    AxisX,
    AxisY,
    AxisZ,
    TargetCenter,
    RotRoll,  // Red X-rotation ring
    RotPitch, // Green Y-rotation ring
    RotYaw,   // Blue Z-rotation ring
}

/// Parameters configuring visual rendering in the 3D viewport.
#[derive(Debug, Clone)]
pub struct RenderSettings {
    pub show_grid: bool,
    pub show_axes: bool,
    pub show_trail: bool,
    pub show_planned_path: bool,
    pub show_joint_frames: bool,
    pub show_reach_envelope: bool,
    pub show_solid_mesh: bool,
    pub show_gripper: bool,
    pub show_rotation_gizmo: bool,
    pub show_ellipsoid: bool,
    pub show_workcell_table: bool,
    pub show_workpieces: bool,
    pub show_obstacles: bool,
    pub grid_size: f32,
    pub grid_subdivisions: usize,
}

impl Default for RenderSettings {
    fn default() -> Self {
        Self {
            show_grid: true,
            show_axes: true,
            show_trail: true,
            show_planned_path: true,
            show_joint_frames: false,
            show_reach_envelope: false,
            show_solid_mesh: true,
            show_gripper: true,
            show_rotation_gizmo: true,
            show_ellipsoid: false,
            show_workcell_table: true,
            show_workpieces: true,
            show_obstacles: true,
            grid_size: 2.0,
            grid_subdivisions: 10,
        }
    }
}

/// A 2.5D visual primitive queued for depth-sorted rendering (Painter's algorithm).
#[allow(dead_code)]
enum RenderPrimitive {
    Polygon {
        depth: f32,
        points: Vec<Pos2>,
        fill: Color32,
        stroke: Stroke,
    },
    Line {
        depth: f32,
        p1: Pos2,
        p2: Pos2,
        stroke: Stroke,
    },
    Circle {
        depth: f32,
        center: Pos2,
        radius: f32,
        fill: Color32,
        stroke: Stroke,
    },
}

impl RenderPrimitive {
    #[inline]
    fn depth(&self) -> f32 {
        match self {
            RenderPrimitive::Polygon { depth, .. } => *depth,
            RenderPrimitive::Line { depth, .. } => *depth,
            RenderPrimitive::Circle { depth, .. } => *depth,
        }
    }
}

/// Calculates directional diffuse + specular lighting on a 3D surface.
fn shade_color(
    base_color: Color32,
    normal: Vector3<f32>,
    view_dir: Vector3<f32>,
    light_dir: Vector3<f32>,
    fill_dir: Vector3<f32>,
    specular_shininess: f32,
    is_metallic: bool,
) -> Color32 {
    let ndotl1 = normal.dot(&light_dir).max(0.0);
    let ndotl2 = normal.dot(&fill_dir).max(0.0);

    let ambient = 0.28;
    let diffuse = 0.52 * ndotl1 + 0.16 * ndotl2;
    let intensity = (ambient + diffuse).clamp(0.0, 1.0);

    let half_vec = (light_dir + view_dir).normalize();
    let ndoth = normal.dot(&half_vec).max(0.0);
    let spec_mult = if is_metallic { 0.40 } else { 0.18 };
    let spec = ndoth.powf(specular_shininess) * spec_mult;

    let r = ((base_color.r() as f32 * intensity) + spec * 255.0).clamp(0.0, 255.0) as u8;
    let g = ((base_color.g() as f32 * intensity) + spec * 255.0).clamp(0.0, 255.0) as u8;
    let b = ((base_color.b() as f32 * intensity) + spec * 255.0).clamp(0.0, 255.0) as u8;

    Color32::from_rgba_unmultiplied(r, g, b, base_color.a())
}

/// Adds a 3D planar quadrilateral facet to the render queue with back-face culling.
#[allow(clippy::too_many_arguments)]
fn add_quad_3d(
    primitives: &mut Vec<RenderPrimitive>,
    camera: &OrbitCamera,
    rect: Rect,
    v0: Point3<f32>,
    v1: Point3<f32>,
    v2: Point3<f32>,
    v3: Point3<f32>,
    normal: Vector3<f32>,
    base_color: Color32,
    stroke: Stroke,
    shininess: f32,
    is_metallic: bool,
    light_dir: Vector3<f32>,
    fill_dir: Vector3<f32>,
) {
    let center = Point3::from((v0.coords + v1.coords + v2.coords + v3.coords) * 0.25);
    let view_vec = camera.eye_position() - center;
    if normal.dot(&view_vec) <= 0.0 {
        return; // Back-face culling
    }

    if let (Some((s0, d0)), Some((s1, d1)), Some((s2, d2)), Some((s3, d3))) = (
        camera.project(v0, rect),
        camera.project(v1, rect),
        camera.project(v2, rect),
        camera.project(v3, rect),
    ) {
        let avg_depth = (d0 + d1 + d2 + d3) * 0.25;
        let color = shade_color(
            base_color,
            normal,
            view_vec.normalize(),
            light_dir,
            fill_dir,
            shininess,
            is_metallic,
        );
        primitives.push(RenderPrimitive::Polygon {
            depth: avg_depth,
            points: vec![s0, s1, s2, s3],
            fill: color,
            stroke,
        });
    }
}

/// Adds a 3D circular cap (disc) to the render queue.
#[allow(clippy::too_many_arguments)]
fn add_circle_cap_3d(
    primitives: &mut Vec<RenderPrimitive>,
    camera: &OrbitCamera,
    rect: Rect,
    center: Point3<f32>,
    normal: Vector3<f32>,
    u: Vector3<f32>,
    w: Vector3<f32>,
    radius: f32,
    subdivisions: usize,
    base_color: Color32,
    stroke: Stroke,
    is_metallic: bool,
    light_dir: Vector3<f32>,
    fill_dir: Vector3<f32>,
) {
    let view_vec = camera.eye_position() - center;
    if normal.dot(&view_vec) <= 0.0 {
        return;
    }
    let mut screen_pts = Vec::with_capacity(subdivisions);
    let mut total_depth = 0.0;
    for k in 0..subdivisions {
        let theta = (k as f32 / subdivisions as f32) * std::f32::consts::TAU;
        let pt = center + (u * theta.cos() + w * theta.sin()) * radius;
        if let Some((s_pos, d)) = camera.project(pt, rect) {
            screen_pts.push(s_pos);
            total_depth += d;
        } else {
            return;
        }
    }
    let avg_depth = total_depth / subdivisions as f32;
    let color = shade_color(
        base_color,
        normal,
        view_vec.normalize(),
        light_dir,
        fill_dir,
        24.0,
        is_metallic,
    );
    primitives.push(RenderPrimitive::Polygon {
        depth: avg_depth,
        points: screen_pts,
        fill: color,
        stroke,
    });
}

/// Constructs a solid 3D cylinder or tapered frustum with lit quad facets.
#[allow(clippy::too_many_arguments)]
fn add_cylinder_3d(
    primitives: &mut Vec<RenderPrimitive>,
    camera: &OrbitCamera,
    rect: Rect,
    p_start: Point3<f32>,
    p_end: Point3<f32>,
    r_start: f32,
    r_end: f32,
    subdivisions: usize,
    body_color: Color32,
    cap_start: bool,
    cap_end: bool,
    cap_color: Color32,
    is_metallic: bool,
    light_dir: Vector3<f32>,
    fill_dir: Vector3<f32>,
) {
    let v = p_end - p_start;
    let len = v.norm();
    if len < 1e-4 {
        return;
    }
    let axis = v / len;
    let helper = if axis.z.abs() < 0.85 {
        Vector3::z()
    } else {
        Vector3::y()
    };
    let u = axis.cross(&helper).normalize();
    let w = axis.cross(&u).normalize();

    let stroke = Stroke::new(0.5_f32, Color32::from_rgba_unmultiplied(20, 24, 30, 80));

    // Side quad facets
    for k in 0..subdivisions {
        let theta0 = (k as f32 / subdivisions as f32) * std::f32::consts::TAU;
        let theta1 = ((k + 1) as f32 / subdivisions as f32) * std::f32::consts::TAU;

        let radial0 = u * theta0.cos() + w * theta0.sin();
        let radial1 = u * theta1.cos() + w * theta1.sin();

        let v0 = p_start + radial0 * r_start;
        let v1 = p_start + radial1 * r_start;
        let v2 = p_end + radial1 * r_end;
        let v3 = p_end + radial0 * r_end;

        let normal = ((radial0 + radial1) * 0.5).normalize();

        add_quad_3d(
            primitives,
            camera,
            rect,
            v0,
            v1,
            v2,
            v3,
            normal,
            body_color,
            stroke,
            16.0,
            is_metallic,
            light_dir,
            fill_dir,
        );
    }

    if cap_start {
        add_circle_cap_3d(
            primitives,
            camera,
            rect,
            p_start,
            -axis,
            u,
            w,
            r_start,
            subdivisions,
            cap_color,
            stroke,
            is_metallic,
            light_dir,
            fill_dir,
        );
    }
    if cap_end {
        add_circle_cap_3d(
            primitives,
            camera,
            rect,
            p_end,
            axis,
            u,
            w,
            r_end,
            subdivisions,
            cap_color,
            stroke,
            is_metallic,
            light_dir,
            fill_dir,
        );
    }
}

/// Constructs a solid 3D oriented cuboid box with 6 lit quad faces.
#[allow(clippy::too_many_arguments)]
fn add_box_3d(
    primitives: &mut Vec<RenderPrimitive>,
    camera: &OrbitCamera,
    rect: Rect,
    center: Point3<f32>,
    half_size: Vector3<f32>,
    axis_x: Vector3<f32>,
    axis_y: Vector3<f32>,
    axis_z: Vector3<f32>,
    color: Color32,
    stroke: Stroke,
    is_metallic: bool,
    light_dir: Vector3<f32>,
    fill_dir: Vector3<f32>,
) {
    // +X face
    let x_plus = center + axis_x * half_size.x;
    let v_xp0 = x_plus - axis_y * half_size.y - axis_z * half_size.z;
    let v_xp1 = x_plus + axis_y * half_size.y - axis_z * half_size.z;
    let v_xp2 = x_plus + axis_y * half_size.y + axis_z * half_size.z;
    let v_xp3 = x_plus - axis_y * half_size.y + axis_z * half_size.z;
    add_quad_3d(
        primitives,
        camera,
        rect,
        v_xp0,
        v_xp1,
        v_xp2,
        v_xp3,
        axis_x,
        color,
        stroke,
        16.0,
        is_metallic,
        light_dir,
        fill_dir,
    );

    // -X face
    let x_minus = center - axis_x * half_size.x;
    let v_xm0 = x_minus + axis_y * half_size.y - axis_z * half_size.z;
    let v_xm1 = x_minus - axis_y * half_size.y - axis_z * half_size.z;
    let v_xm2 = x_minus - axis_y * half_size.y + axis_z * half_size.z;
    let v_xm3 = x_minus + axis_y * half_size.y + axis_z * half_size.z;
    add_quad_3d(
        primitives,
        camera,
        rect,
        v_xm0,
        v_xm1,
        v_xm2,
        v_xm3,
        -axis_x,
        color,
        stroke,
        16.0,
        is_metallic,
        light_dir,
        fill_dir,
    );

    // +Y face
    let y_plus = center + axis_y * half_size.y;
    let v_yp0 = y_plus + axis_x * half_size.x - axis_z * half_size.z;
    let v_yp1 = y_plus - axis_x * half_size.x - axis_z * half_size.z;
    let v_yp2 = y_plus - axis_x * half_size.x + axis_z * half_size.z;
    let v_yp3 = y_plus + axis_x * half_size.x + axis_z * half_size.z;
    add_quad_3d(
        primitives,
        camera,
        rect,
        v_yp0,
        v_yp1,
        v_yp2,
        v_yp3,
        axis_y,
        color,
        stroke,
        16.0,
        is_metallic,
        light_dir,
        fill_dir,
    );

    // -Y face
    let y_minus = center - axis_y * half_size.y;
    let v_ym0 = y_minus - axis_x * half_size.x - axis_z * half_size.z;
    let v_ym1 = y_minus + axis_x * half_size.x - axis_z * half_size.z;
    let v_ym2 = y_minus + axis_x * half_size.x + axis_z * half_size.z;
    let v_ym3 = y_minus - axis_x * half_size.x + axis_z * half_size.z;
    add_quad_3d(
        primitives,
        camera,
        rect,
        v_ym0,
        v_ym1,
        v_ym2,
        v_ym3,
        -axis_y,
        color,
        stroke,
        16.0,
        is_metallic,
        light_dir,
        fill_dir,
    );

    // +Z face
    let z_plus = center + axis_z * half_size.z;
    let v_zp0 = z_plus - axis_x * half_size.x - axis_y * half_size.y;
    let v_zp1 = z_plus + axis_x * half_size.x - axis_y * half_size.y;
    let v_zp2 = z_plus + axis_x * half_size.x + axis_y * half_size.y;
    let v_zp3 = z_plus - axis_x * half_size.x + axis_y * half_size.y;
    add_quad_3d(
        primitives,
        camera,
        rect,
        v_zp0,
        v_zp1,
        v_zp2,
        v_zp3,
        axis_z,
        color,
        stroke,
        16.0,
        is_metallic,
        light_dir,
        fill_dir,
    );

    // -Z face
    let z_minus = center - axis_z * half_size.z;
    let v_zm0 = z_minus - axis_x * half_size.x + axis_y * half_size.y;
    let v_zm1 = z_minus + axis_x * half_size.x + axis_y * half_size.y;
    let v_zm2 = z_minus + axis_x * half_size.x - axis_y * half_size.y;
    let v_zm3 = z_minus - axis_x * half_size.x - axis_y * half_size.y;
    add_quad_3d(
        primitives,
        camera,
        rect,
        v_zm0,
        v_zm1,
        v_zm2,
        v_zm3,
        -axis_z,
        color,
        stroke,
        16.0,
        is_metallic,
        light_dir,
        fill_dir,
    );
}

/// Builds the complete 3D volumetric model of the robotic arm.
#[allow(clippy::too_many_arguments)]
fn render_solid_robot_arm(
    primitives: &mut Vec<RenderPrimitive>,
    camera: &OrbitCamera,
    rect: Rect,
    robot: &RobotArm,
    settings: &RenderSettings,
    tool_state: &ToolState,
    colliding_links: &[usize],
    light_dir: Vector3<f32>,
    fill_dir: Vector3<f32>,
) {
    let poses = robot.forward_kinematics();
    if poses.is_empty() {
        return;
    }
    let num_frames = poses.len();

    // 1. Heavy Industrial Base Pedestal
    let base_trans = robot.base_transform.translation.vector.cast::<f32>();
    let base_pos = Point3::from(base_trans);

    // A. Soft Contact Shadow on Ground Grid
    let shadow_subdiv = 16;
    let mut shadow_pts = Vec::with_capacity(shadow_subdiv);
    let mut shadow_depth_total = 0.0;
    for k in 0..shadow_subdiv {
        let theta = (k as f32 / shadow_subdiv as f32) * std::f32::consts::TAU;
        let p = Point3::new(
            base_pos.x + 0.22 * theta.cos(),
            base_pos.y + 0.22 * theta.sin(),
            0.001,
        );
        if let Some((s_pos, d)) = camera.project(p, rect) {
            shadow_pts.push(s_pos);
            shadow_depth_total += d;
        }
    }
    if shadow_pts.len() == shadow_subdiv {
        primitives.push(RenderPrimitive::Polygon {
            depth: (shadow_depth_total / shadow_subdiv as f32) + 0.02,
            points: shadow_pts,
            fill: Color32::from_rgba_unmultiplied(12, 16, 22, 100),
            stroke: Stroke::NONE,
        });
    }

    // B. Flanged Cast Baseplate (Mounting Footing)
    let base_h = 0.038_f32;
    let base_r = 0.165_f32;
    let baseplate_bottom = Point3::new(base_pos.x, base_pos.y, 0.002);
    let baseplate_top = Point3::new(base_pos.x, base_pos.y, base_h);
    let dark_iron = Color32::from_rgb(36, 40, 48);
    let bolt_washer = Color32::from_rgb(165, 175, 190);
    let bolt_head = Color32::from_rgb(22, 25, 30);

    add_cylinder_3d(
        primitives,
        camera,
        rect,
        baseplate_bottom,
        baseplate_top,
        base_r,
        base_r * 0.98,
        16,
        dark_iron,
        false,
        true,
        dark_iron,
        false,
        light_dir,
        fill_dir,
    );

    // C. 6 Floor Anchoring Bolts
    for b in 0..6 {
        let angle = (b as f32 / 6.0) * std::f32::consts::TAU;
        let bolt_p_bottom = Point3::new(
            base_pos.x + 0.142 * angle.cos(),
            base_pos.y + 0.142 * angle.sin(),
            base_h,
        );
        let bolt_p_top = Point3::new(bolt_p_bottom.x, bolt_p_bottom.y, base_h + 0.009);
        add_cylinder_3d(
            primitives,
            camera,
            rect,
            bolt_p_bottom,
            bolt_p_top,
            0.012,
            0.012,
            8,
            bolt_washer,
            false,
            true,
            bolt_head,
            true,
            light_dir,
            fill_dir,
        );
    }

    // D. Swivel Turret / Base Riser Column
    let first_joint_z = if num_frames > 1 {
        poses[1].translation.vector.z as f32
    } else {
        base_h + 0.12
    }
    .max(base_h + 0.04);

    let turret_top = Point3::new(base_pos.x, base_pos.y, first_joint_z);
    let turret_color = if !robot.links.is_empty() {
        let c = robot.links[0].color;
        Color32::from_rgba_unmultiplied(
            (c[0] * 255.0) as u8,
            (c[1] * 255.0) as u8,
            (c[2] * 255.0) as u8,
            255,
        )
    } else {
        Color32::from_rgb(48, 52, 62)
    };

    add_cylinder_3d(
        primitives,
        camera,
        rect,
        baseplate_top,
        turret_top,
        0.132,
        0.088,
        16,
        turret_color,
        false,
        true,
        turret_color,
        false,
        light_dir,
        fill_dir,
    );

    // E. Turntable Machined Ring & Industrial Accent
    let ring_bot = Point3::new(base_pos.x, base_pos.y, base_h + 0.005);
    let ring_top = Point3::new(base_pos.x, base_pos.y, base_h + 0.015);
    add_cylinder_3d(
        primitives,
        camera,
        rect,
        ring_bot,
        ring_top,
        0.134,
        0.132,
        16,
        Color32::from_rgb(195, 205, 220),
        false,
        false,
        Color32::WHITE,
        true,
        light_dir,
        fill_dir,
    );

    let accent_bot = Point3::new(base_pos.x, base_pos.y, base_h + 0.035);
    let accent_top = Point3::new(base_pos.x, base_pos.y, base_h + 0.045);
    add_cylinder_3d(
        primitives,
        camera,
        rect,
        accent_bot,
        accent_top,
        0.122,
        0.120,
        16,
        Color32::from_rgb(245, 115, 20),
        false,
        false,
        Color32::WHITE,
        false,
        light_dir,
        fill_dir,
    );

    // 2. Articulated Arm Links (Volumetric Casings)
    for i in 0..num_frames - 1 {
        let p_start = Point3::from(poses[i].translation.vector.cast::<f32>());
        let p_end = Point3::from(poses[i + 1].translation.vector.cast::<f32>());
        let v = p_end - p_start;
        let len = v.norm();

        // Highlight link in flashing red if currently colliding
        let is_colliding = colliding_links.contains(&i);
        let link_color = if is_colliding {
            Color32::from_rgb(255, 45, 45) // Warning red
        } else if i < robot.links.len() {
            let c = robot.links[i].color;
            Color32::from_rgba_unmultiplied(
                (c[0] * 255.0) as u8,
                (c[1] * 255.0) as u8,
                (c[2] * 255.0) as u8,
                255,
            )
        } else {
            Color32::from_rgb(240, 110, 25)
        };

        let r_nominal = if i < robot.links.len() {
            if let Some(LinkGeometry::Cylinder { radius, .. }) = robot.links[i].visual_geometry {
                radius as f32
            } else {
                (0.062 * 0.93_f32.powi(i as i32)).clamp(0.032, 0.075)
            }
        } else {
            0.040
        };

        if len >= 0.025 {
            let u_z = v / len;
            let helper = if u_z.z.abs() < 0.85 {
                Vector3::z()
            } else {
                Vector3::y()
            };
            let u_x = u_z.cross(&helper).normalize();
            let u_y = u_z.cross(&u_x).normalize();

            // Main tapered link body
            add_cylinder_3d(
                primitives,
                camera,
                rect,
                p_start,
                p_end,
                r_nominal * 1.05,
                r_nominal * 0.95,
                14,
                link_color,
                false,
                false,
                link_color,
                false,
                light_dir,
                fill_dir,
            );

            // Dark titanium collars at start and end
            let collar_len = (len * 0.12).clamp(0.015, 0.035);
            let collar_color = if is_colliding {
                Color32::from_rgb(180, 20, 20)
            } else {
                Color32::from_rgb(32, 35, 42)
            };
            let p_start_collar_end = p_start + u_z * collar_len;
            add_cylinder_3d(
                primitives,
                camera,
                rect,
                p_start,
                p_start_collar_end,
                r_nominal * 1.09,
                r_nominal * 1.07,
                14,
                collar_color,
                false,
                false,
                collar_color,
                true,
                light_dir,
                fill_dir,
            );

            let p_end_collar_start = p_end - u_z * collar_len;
            add_cylinder_3d(
                primitives,
                camera,
                rect,
                p_end_collar_start,
                p_end,
                r_nominal * 0.97,
                r_nominal * 1.04,
                14,
                collar_color,
                false,
                false,
                collar_color,
                true,
                light_dir,
                fill_dir,
            );

            // Inset side accent stripes
            let stripe_color = Color32::from_rgb(28, 30, 36);
            let stripe_start = p_start + u_z * (collar_len + 0.01);
            let stripe_end = p_end - u_z * (collar_len + 0.01);
            if (stripe_end - stripe_start).norm() > 0.02 {
                let stripe_w = r_nominal * 0.40;
                let stroke = Stroke::new(0.5_f32, Color32::from_rgba_unmultiplied(10, 12, 16, 90));
                // +X side stripe
                let v0 = stripe_start + u_x * (r_nominal * 0.99) - u_y * (stripe_w * 0.5);
                let v1 = stripe_start + u_x * (r_nominal * 0.99) + u_y * (stripe_w * 0.5);
                let v2 = stripe_end + u_x * (r_nominal * 0.92) + u_y * (stripe_w * 0.5);
                let v3 = stripe_end + u_x * (r_nominal * 0.92) - u_y * (stripe_w * 0.5);
                add_quad_3d(
                    primitives,
                    camera,
                    rect,
                    v0,
                    v1,
                    v2,
                    v3,
                    u_x,
                    stripe_color,
                    stroke,
                    12.0,
                    false,
                    light_dir,
                    fill_dir,
                );

                // -X side stripe
                let vm0 = stripe_start - u_x * (r_nominal * 0.99) + u_y * (stripe_w * 0.5);
                let vm1 = stripe_start - u_x * (r_nominal * 0.99) - u_y * (stripe_w * 0.5);
                let vm2 = stripe_end - u_x * (r_nominal * 0.92) - u_y * (stripe_w * 0.5);
                let vm3 = stripe_end - u_x * (r_nominal * 0.92) + u_y * (stripe_w * 0.5);
                add_quad_3d(
                    primitives,
                    camera,
                    rect,
                    vm0,
                    vm1,
                    vm2,
                    vm3,
                    -u_x,
                    stripe_color,
                    stroke,
                    12.0,
                    false,
                    light_dir,
                    fill_dir,
                );
            }
        } else {
            // Compound joint / wrist cross hub
            let hub_r = r_nominal * 1.15;
            let dark_hub = if is_colliding {
                Color32::from_rgb(240, 40, 40)
            } else {
                Color32::from_rgb(45, 50, 60)
            };
            let p_hub_top = p_start + Vector3::new(0.0, 0.0, hub_r * 0.6);
            let p_hub_bot = p_start - Vector3::new(0.0, 0.0, hub_r * 0.6);
            add_cylinder_3d(
                primitives, camera, rect, p_hub_bot, p_hub_top, hub_r, hub_r, 12, dark_hub, true,
                true, dark_hub, true, light_dir, fill_dir,
            );
        }
    }

    // 3. Mechanical Joint Actuators (Motor Drums & Harmonic Drives)
    #[allow(clippy::needless_range_loop)]
    for j in 0..robot.joints.len() {
        let p_joint = Point3::from(poses[j].translation.vector.cast::<f32>());
        let a_local = robot.joints[j].axis.into_inner();
        let a_world_64 = poses[j].rotation * a_local;
        let a_world = a_world_64.cast::<f32>();
        let axis_dir = if a_world.norm() > 1e-4 {
            a_world.normalize()
        } else {
            Vector3::z()
        };

        let r_nom = (0.058 * 0.93_f32.powi(j as i32)).clamp(0.034, 0.070);
        let r_drum = (r_nom * 1.25).clamp(0.044, 0.082);
        let h_drum = (r_nom * 1.05).clamp(0.036, 0.070);

        let drum_start = p_joint - axis_dir * h_drum;
        let drum_end = p_joint + axis_dir * h_drum;
        let drum_body_color = Color32::from_rgb(34, 38, 46);
        let bearing_plate_color = Color32::from_rgb(175, 185, 200);

        // Cylindrical motor canister
        add_cylinder_3d(
            primitives,
            camera,
            rect,
            drum_start,
            drum_end,
            r_drum,
            r_drum,
            14,
            drum_body_color,
            true,
            true,
            bearing_plate_color,
            true,
            light_dir,
            fill_dir,
        );

        // Concentric Bearing Covers on both faces
        let helper = if axis_dir.z.abs() < 0.85 {
            Vector3::z()
        } else {
            Vector3::y()
        };
        let u_d = axis_dir.cross(&helper).normalize();
        let w_d = axis_dir.cross(&u_d).normalize();
        let stroke_subtle = Stroke::new(0.5_f32, Color32::from_rgba_unmultiplied(15, 18, 24, 100));

        // +axis end cap details
        add_circle_cap_3d(
            primitives,
            camera,
            rect,
            drum_end + axis_dir * 0.001,
            axis_dir,
            u_d,
            w_d,
            r_drum * 0.52,
            12,
            Color32::from_rgb(58, 64, 76),
            stroke_subtle,
            true,
            light_dir,
            fill_dir,
        );
        add_circle_cap_3d(
            primitives,
            camera,
            rect,
            drum_end + axis_dir * 0.002,
            axis_dir,
            u_d,
            w_d,
            r_drum * 0.24,
            8,
            Color32::from_rgb(22, 25, 32),
            stroke_subtle,
            true,
            light_dir,
            fill_dir,
        );

        // -axis end cap details
        add_circle_cap_3d(
            primitives,
            camera,
            rect,
            drum_start - axis_dir * 0.001,
            -axis_dir,
            u_d,
            w_d,
            r_drum * 0.52,
            12,
            Color32::from_rgb(58, 64, 76),
            stroke_subtle,
            true,
            light_dir,
            fill_dir,
        );
        add_circle_cap_3d(
            primitives,
            camera,
            rect,
            drum_start - axis_dir * 0.002,
            -axis_dir,
            u_d,
            w_d,
            r_drum * 0.24,
            8,
            Color32::from_rgb(22, 25, 32),
            stroke_subtle,
            true,
            light_dir,
            fill_dir,
        );
    }

    // 4. Interchangeable End-Of-Arm Tooling (EOAT)
    if settings.show_gripper {
        if let Some(ee_pose) = poses.last() {
            let p_ee = Point3::from(ee_pose.translation.vector.cast::<f32>());
            let r_ee = ee_pose.rotation.cast::<f32>();

            // Forward direction
            let u_z = if poses.len() >= 2 {
                let p_penult =
                    Point3::from(poses[poses.len() - 2].translation.vector.cast::<f32>());
                let dir = p_ee - p_penult;
                if dir.norm() > 1e-3 {
                    dir.normalize()
                } else {
                    r_ee * Vector3::x()
                }
            } else {
                r_ee * Vector3::x()
            };

            let helper = if u_z.z.abs() < 0.85 {
                Vector3::z()
            } else {
                Vector3::y()
            };
            let u_x = u_z.cross(&helper).normalize();
            let u_y = u_z.cross(&u_x).normalize();

            // A. Tool Flange Adapter (ISO 9409-1)
            let flange_end = p_ee + u_z * 0.016;
            add_cylinder_3d(
                primitives,
                camera,
                rect,
                p_ee,
                flange_end,
                0.038,
                0.038,
                14,
                Color32::from_rgb(195, 205, 220),
                true,
                true,
                Color32::from_rgb(195, 205, 220),
                true,
                light_dir,
                fill_dir,
            );

            match tool_state.tool_type {
                EOATType::ParallelGripper => {
                    // Animated mechanical jaw opening
                    let opening = tool_state.gripper_opening.clamp(0.0, 1.0);
                    let finger_x_off = 0.014 + opening * 0.024; // dynamically widens/closes

                    let chassis_center = p_ee + u_z * 0.038;
                    let chassis_half = Vector3::new(0.038, 0.022, 0.018);
                    let chassis_color = Color32::from_rgb(28, 32, 40);
                    let stroke_chassis =
                        Stroke::new(0.5_f32, Color32::from_rgba_unmultiplied(15, 18, 24, 100));

                    add_box_3d(
                        primitives,
                        camera,
                        rect,
                        chassis_center,
                        chassis_half,
                        u_x,
                        u_y,
                        u_z,
                        chassis_color,
                        stroke_chassis,
                        true,
                        light_dir,
                        fill_dir,
                    );

                    // LED status bar
                    let led_center = chassis_center + u_y * (chassis_half.y + 0.001);
                    let led_half_x = 0.020_f32;
                    let led_half_z = 0.008_f32;
                    let led_v0 = led_center - u_x * led_half_x - u_z * led_half_z;
                    let led_v1 = led_center + u_x * led_half_x - u_z * led_half_z;
                    let led_v2 = led_center + u_x * led_half_x + u_z * led_half_z;
                    let led_v3 = led_center - u_x * led_half_x + u_z * led_half_z;
                    add_quad_3d(
                        primitives,
                        camera,
                        rect,
                        led_v0,
                        led_v1,
                        led_v2,
                        led_v3,
                        u_y,
                        Color32::from_rgb(0, 235, 215),
                        Stroke::NONE,
                        32.0,
                        false,
                        light_dir,
                        fill_dir,
                    );

                    let finger_color = Color32::from_rgb(46, 52, 62);
                    let pad_color = Color32::from_rgb(245, 110, 20);

                    // Left Finger
                    let left_knuckle_center = p_ee + u_z * 0.070 - u_x * finger_x_off;
                    let finger_half = Vector3::new(0.006, 0.012, 0.015);
                    add_box_3d(
                        primitives,
                        camera,
                        rect,
                        left_knuckle_center,
                        finger_half,
                        u_x,
                        u_y,
                        u_z,
                        finger_color,
                        stroke_chassis,
                        true,
                        light_dir,
                        fill_dir,
                    );

                    let left_tip_end = p_ee + u_z * 0.108 - u_x * (finger_x_off * 0.45);
                    let left_tip_center = Point3::from(
                        (left_knuckle_center.coords + left_tip_end.coords) * 0.5 + u_z * 0.012,
                    );
                    add_box_3d(
                        primitives,
                        camera,
                        rect,
                        left_tip_center,
                        Vector3::new(0.005, 0.010, 0.012),
                        u_x,
                        u_y,
                        u_z,
                        finger_color,
                        stroke_chassis,
                        true,
                        light_dir,
                        fill_dir,
                    );

                    // Left gripping pad
                    let left_pad_center = left_knuckle_center + u_x * (finger_half.x + 0.001);
                    let pad_v0 = left_pad_center - u_y * 0.009 - u_z * 0.012;
                    let pad_v1 = left_pad_center + u_y * 0.009 - u_z * 0.012;
                    let pad_v2 = left_pad_center + u_y * 0.009 + u_z * 0.012;
                    let pad_v3 = left_pad_center - u_y * 0.009 + u_z * 0.012;
                    add_quad_3d(
                        primitives,
                        camera,
                        rect,
                        pad_v0,
                        pad_v1,
                        pad_v2,
                        pad_v3,
                        u_x,
                        pad_color,
                        Stroke::NONE,
                        10.0,
                        false,
                        light_dir,
                        fill_dir,
                    );

                    // Right Finger
                    let right_knuckle_center = p_ee + u_z * 0.070 + u_x * finger_x_off;
                    add_box_3d(
                        primitives,
                        camera,
                        rect,
                        right_knuckle_center,
                        finger_half,
                        u_x,
                        u_y,
                        u_z,
                        finger_color,
                        stroke_chassis,
                        true,
                        light_dir,
                        fill_dir,
                    );

                    let right_tip_end = p_ee + u_z * 0.108 + u_x * (finger_x_off * 0.45);
                    let right_tip_center = Point3::from(
                        (right_knuckle_center.coords + right_tip_end.coords) * 0.5 + u_z * 0.012,
                    );
                    add_box_3d(
                        primitives,
                        camera,
                        rect,
                        right_tip_center,
                        Vector3::new(0.005, 0.010, 0.012),
                        u_x,
                        u_y,
                        u_z,
                        finger_color,
                        stroke_chassis,
                        true,
                        light_dir,
                        fill_dir,
                    );

                    // Right gripping pad
                    let right_pad_center = right_knuckle_center - u_x * (finger_half.x + 0.001);
                    let rpad_v0 = right_pad_center + u_y * 0.009 - u_z * 0.012;
                    let rpad_v1 = right_pad_center - u_y * 0.009 - u_z * 0.012;
                    let rpad_v2 = right_pad_center - u_y * 0.009 + u_z * 0.012;
                    let rpad_v3 = right_pad_center + u_y * 0.009 + u_z * 0.012;
                    add_quad_3d(
                        primitives,
                        camera,
                        rect,
                        rpad_v0,
                        rpad_v1,
                        rpad_v2,
                        rpad_v3,
                        -u_x,
                        pad_color,
                        Stroke::NONE,
                        10.0,
                        false,
                        light_dir,
                        fill_dir,
                    );
                }
                EOATType::VacuumCup => {
                    // Pneumatic bellows suction cup
                    let cup_stem_end = p_ee + u_z * 0.035;
                    add_cylinder_3d(
                        primitives,
                        camera,
                        rect,
                        flange_end,
                        cup_stem_end,
                        0.014,
                        0.014,
                        12,
                        Color32::from_rgb(180, 150, 40), // Brass fitting
                        true,
                        false,
                        Color32::from_rgb(180, 150, 40),
                        true,
                        light_dir,
                        fill_dir,
                    );

                    // Rubber Bellows Convolutions
                    let rubber_color = Color32::from_rgb(32, 34, 40);
                    let p_bellows_1 = cup_stem_end + u_z * 0.020;
                    let p_bellows_2 = cup_stem_end + u_z * 0.040;
                    let p_bellows_tip = cup_stem_end + u_z * 0.065;

                    add_cylinder_3d(
                        primitives,
                        camera,
                        rect,
                        cup_stem_end,
                        p_bellows_1,
                        0.024,
                        0.034,
                        14,
                        rubber_color,
                        false,
                        false,
                        rubber_color,
                        false,
                        light_dir,
                        fill_dir,
                    );
                    add_cylinder_3d(
                        primitives,
                        camera,
                        rect,
                        p_bellows_1,
                        p_bellows_2,
                        0.034,
                        0.042,
                        14,
                        rubber_color,
                        false,
                        false,
                        rubber_color,
                        false,
                        light_dir,
                        fill_dir,
                    );
                    add_cylinder_3d(
                        primitives,
                        camera,
                        rect,
                        p_bellows_2,
                        p_bellows_tip,
                        0.042,
                        0.048,
                        16,
                        rubber_color,
                        false,
                        true,
                        rubber_color,
                        false,
                        light_dir,
                        fill_dir,
                    );

                    // Vacuum active indicator ring
                    let vac_color = if tool_state.is_vacuum_active {
                        Color32::from_rgb(40, 240, 120) // Active suction green
                    } else {
                        Color32::from_rgb(60, 65, 75)
                    };
                    add_cylinder_3d(
                        primitives,
                        camera,
                        rect,
                        cup_stem_end - u_z * 0.008,
                        cup_stem_end - u_z * 0.002,
                        0.022,
                        0.022,
                        12,
                        vac_color,
                        false,
                        false,
                        vac_color,
                        false,
                        light_dir,
                        fill_dir,
                    );
                }
                EOATType::WeldingTorch => {
                    // Robotic swan-neck welding torch
                    let torch_neck_end = p_ee + u_z * 0.060 + u_x * 0.025;
                    add_cylinder_3d(
                        primitives,
                        camera,
                        rect,
                        flange_end,
                        torch_neck_end,
                        0.016,
                        0.012,
                        12,
                        Color32::from_rgb(180, 190, 205), // Chrome neck
                        true,
                        false,
                        Color32::from_rgb(180, 190, 205),
                        true,
                        light_dir,
                        fill_dir,
                    );

                    // Ceramic Gas Cup & Copper Nozzle
                    let nozzle_end = torch_neck_end + (u_z * 0.8 + u_x * 0.6).normalize() * 0.045;
                    add_cylinder_3d(
                        primitives,
                        camera,
                        rect,
                        torch_neck_end,
                        nozzle_end,
                        0.014,
                        0.008,
                        12,
                        Color32::from_rgb(220, 110, 45), // Copper nozzle
                        false,
                        true,
                        Color32::from_rgb(240, 130, 60),
                        true,
                        light_dir,
                        fill_dir,
                    );

                    // Active electric welding arc spark burst
                    if tool_state.is_welding_active {
                        let arc_center = nozzle_end + (u_z * 0.8 + u_x * 0.6).normalize() * 0.006;
                        if let Some((s_arc, d_arc)) = camera.project(arc_center, rect) {
                            primitives.push(RenderPrimitive::Circle {
                                depth: d_arc - 0.02,
                                center: s_arc,
                                radius: 10.0,
                                fill: Color32::from_rgba_unmultiplied(220, 240, 255, 230),
                                stroke: Stroke::new(2.5_f32, Color32::from_rgb(60, 180, 255)),
                            });
                        }
                    }
                }
            }

            // TCP reticle circle
            let p_tcp = p_ee + u_z * 0.098;
            if let Some((s_tcp, d_tcp)) = camera.project(p_tcp, rect) {
                let tcp_radius = (7.0 / d_tcp).clamp(3.0, 9.0);
                primitives.push(RenderPrimitive::Circle {
                    depth: d_tcp - 0.01,
                    center: s_tcp,
                    radius: tcp_radius,
                    fill: Color32::from_rgba_unmultiplied(0, 235, 215, 120),
                    stroke: Stroke::new(1.5_f32, Color32::WHITE),
                });
            }
        }
    }
}

/// Renders the workcell environment table, legs, and safety zone border.
fn render_workcell_environment(
    primitives: &mut Vec<RenderPrimitive>,
    camera: &OrbitCamera,
    rect: Rect,
    env: &WorkcellEnvironment,
    light_dir: Vector3<f32>,
    fill_dir: Vector3<f32>,
) {
    if !env.show_table {
        return;
    }

    let center_f32 = Point3::new(
        env.table_center.x as f32,
        env.table_center.y as f32,
        env.table_center.z as f32,
    );

    let half = env.table_half_size;
    let table_color = Color32::from_rgb(70, 78, 92); // Sturdy industrial steel table
    let stroke_table = Stroke::new(0.5_f32, Color32::from_rgba_unmultiplied(30, 35, 45, 100));

    // Table Top Box
    add_box_3d(
        primitives,
        camera,
        rect,
        center_f32,
        half,
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        table_color,
        stroke_table,
        false,
        light_dir,
        fill_dir,
    );

    // 4 Steel Support Legs
    let leg_h = (center_f32.z - half.z).max(0.0);
    let leg_r = 0.022_f32;
    let leg_color = Color32::from_rgb(45, 50, 60);

    let leg_offsets = [
        (-half.x * 0.85, -half.y * 0.85),
        (-half.x * 0.85, half.y * 0.85),
        (half.x * 0.85, -half.y * 0.85),
        (half.x * 0.85, half.y * 0.85),
    ];

    for &(ox, oy) in &leg_offsets {
        let p_top = Point3::new(center_f32.x + ox, center_f32.y + oy, center_f32.z - half.z);
        let p_bot = Point3::new(
            center_f32.x + ox,
            center_f32.y + oy,
            center_f32.z - half.z - leg_h,
        );
        add_cylinder_3d(
            primitives, camera, rect, p_bot, p_top, leg_r, leg_r, 8, leg_color, false, false,
            leg_color, true, light_dir, fill_dir,
        );
    }
}

/// Renders interactive workpieces (Cubes, Cylinders, Spheres) in the workcell.
fn render_workpieces(
    primitives: &mut Vec<RenderPrimitive>,
    camera: &OrbitCamera,
    rect: Rect,
    mgr: &WorkpieceManager,
    light_dir: Vector3<f32>,
    fill_dir: Vector3<f32>,
) {
    for wp in &mgr.workpieces {
        let p_f32 = Point3::new(
            wp.position.x as f32,
            wp.position.y as f32,
            wp.position.z as f32,
        );
        let stroke_wp = if wp.is_grasped {
            Stroke::new(1.8_f32, Color32::from_rgb(0, 240, 220)) // Glowing cyan grasp outline
        } else {
            Stroke::new(0.5_f32, Color32::from_rgba_unmultiplied(20, 20, 25, 100))
        };

        match wp.shape {
            WorkpieceShape::Box { half_size } => {
                let rot_f32 = wp.rotation.cast::<f32>();
                let ax = rot_f32 * Vector3::x();
                let ay = rot_f32 * Vector3::y();
                let az = rot_f32 * Vector3::z();
                add_box_3d(
                    primitives, camera, rect, p_f32, half_size, ax, ay, az, wp.color, stroke_wp,
                    false, light_dir, fill_dir,
                );
            }
            WorkpieceShape::Cylinder { radius, height } => {
                let half_h = height * 0.5;
                let rot_f32 = wp.rotation.cast::<f32>();
                let ax_z = rot_f32 * Vector3::z();
                let p_bot = p_f32 - ax_z * half_h;
                let p_top = p_f32 + ax_z * half_h;
                add_cylinder_3d(
                    primitives, camera, rect, p_bot, p_top, radius, radius, 14, wp.color, true,
                    true, wp.color, true, light_dir, fill_dir,
                );
            }
            WorkpieceShape::Sphere { radius } => {
                if let Some((s_pos, depth)) = camera.project(p_f32, rect) {
                    let focal = 1.0 / (camera.fov_y * 0.5).tan();
                    let s_radius = (radius / depth) * focal * (rect.height() * 0.5);
                    primitives.push(RenderPrimitive::Circle {
                        depth,
                        center: s_pos,
                        radius: s_radius.clamp(2.0, 60.0),
                        fill: wp.color,
                        stroke: stroke_wp,
                    });
                }
            }
        }
    }
}

/// Renders obstacle collision volumes in the workspace.
fn render_obstacles(
    primitives: &mut Vec<RenderPrimitive>,
    camera: &OrbitCamera,
    rect: Rect,
    env: &WorkcellEnvironment,
    light_dir: Vector3<f32>,
    fill_dir: Vector3<f32>,
) {
    for obs in &env.obstacles {
        // Skip table top as it's already rendered with table furniture
        if obs.name == "Assembly Workstation Table" {
            continue;
        }

        let p_f32 = Point3::new(
            obs.center.x as f32,
            obs.center.y as f32,
            obs.center.z as f32,
        );
        let half_f32 = Vector3::new(
            obs.half_size.x as f32,
            obs.half_size.y as f32,
            obs.half_size.z as f32,
        );

        let obs_color = Color32::from_rgba_unmultiplied(
            (obs.color[0] * 255.0) as u8,
            (obs.color[1] * 255.0) as u8,
            (obs.color[2] * 255.0) as u8,
            160,
        );
        let stroke_obs = Stroke::new(1.0_f32, Color32::from_rgb(255, 90, 70));

        add_box_3d(
            primitives,
            camera,
            rect,
            p_f32,
            half_f32,
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            obs_color,
            stroke_obs,
            false,
            light_dir,
            fill_dir,
        );
    }
}

/// Renders the Yoshikawa velocity manipulability ellipsoid at the TCP.
fn render_manipulability_ellipsoid(
    painter: &egui::Painter,
    camera: &OrbitCamera,
    rect: Rect,
    tcp_pos: Point3<f32>,
    data: &ManipulabilityData,
) {
    let scale = 0.22_f32; // Visual scaling factor
    let r1 = (data.semi_axes[0] as f32 * scale).clamp(0.02, 0.45);
    let r2 = (data.semi_axes[1] as f32 * scale).clamp(0.015, 0.40);
    let r3 = (data.semi_axes[2] as f32 * scale).clamp(0.01, 0.35);

    let v1 = data.axes_dirs[0].cast::<f32>();
    let v2 = data.axes_dirs[1].cast::<f32>();
    let v3 = data.axes_dirs[2].cast::<f32>();

    let ellipse_color = if data.is_near_singularity {
        Color32::from_rgba_unmultiplied(255, 60, 40, 160) // Singularity warning red
    } else {
        Color32::from_rgba_unmultiplied(40, 220, 200, 140) // High dexterity cyan
    };
    let stroke = Stroke::new(1.5_f32, ellipse_color);

    let num_pts = 32;

    // Draw 3 principal orbital rings (Plane 1-2, Plane 1-3, Plane 2-3)
    let planes = [(v1, v2, r1, r2), (v1, v3, r1, r3), (v2, v3, r2, r3)];

    for &(axis_a, axis_b, ra, rb) in &planes {
        let mut prev_pt: Option<Pos2> = None;
        for k in 0..=num_pts {
            let theta = (k as f32 / num_pts as f32) * std::f32::consts::TAU;
            let pt = tcp_pos + axis_a * (ra * theta.cos()) + axis_b * (rb * theta.sin());
            if let Some((s_pt, _)) = camera.project(pt, rect) {
                if let Some(prev) = prev_pt {
                    painter.line_segment([prev, s_pt], stroke);
                }
                prev_pt = Some(s_pt);
            }
        }
    }
}

/// Renders the complete 3D scene onto the egui Painter.
#[allow(clippy::too_many_arguments)]
pub fn render_scene_3d(
    ui: &mut egui::Ui,
    rect: Rect,
    camera: &OrbitCamera,
    robot: &RobotArm,
    target_pos: Point3<f64>,
    trail: &[Point3<f64>],
    planner: &TrajectoryPlanner,
    settings: &RenderSettings,
    active_drag_axis: GizmoDragAxis,
    tool_state: &ToolState,
    workpieces: &WorkpieceManager,
    environment: &WorkcellEnvironment,
    collision_report: &CollisionReport,
    manip_data: Option<&ManipulabilityData>,
) -> Option<GizmoDragAxis> {
    let painter = ui.painter_at(rect);
    let mut hovered_axis = GizmoDragAxis::None;

    // 1. Draw Metric Ground Plane Grid
    if settings.show_grid {
        let size = settings.grid_size;
        let half = size * 0.5;
        let step = size / settings.grid_subdivisions as f32;

        let grid_stroke = Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(100, 110, 130, 45));
        let axis_stroke = Stroke::new(1.5_f32, Color32::from_rgba_unmultiplied(120, 135, 160, 90));

        let mut i = -half;
        while i <= half + 1e-4 {
            let is_axis = i.abs() < 1e-3;
            let stroke = if is_axis { axis_stroke } else { grid_stroke };

            // Lines parallel to Y
            let p1 = Point3::new(i, -half, 0.0);
            let p2 = Point3::new(i, half, 0.0);
            if let (Some((s1, _)), Some((s2, _))) =
                (camera.project(p1, rect), camera.project(p2, rect))
            {
                painter.line_segment([s1, s2], stroke);
            }

            // Lines parallel to X
            let p3 = Point3::new(-half, i, 0.0);
            let p4 = Point3::new(half, i, 0.0);
            if let (Some((s3, _)), Some((s4, _))) =
                (camera.project(p3, rect), camera.project(p4, rect))
            {
                painter.line_segment([s3, s4], stroke);
            }

            i += step;
        }
    }

    // 2. Draw World Origin Coordinate Axes (RGB = XYZ)
    if settings.show_axes {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let ax_len = 0.25;
        let p_x = Point3::new(ax_len, 0.0, 0.0);
        let p_y = Point3::new(0.0, ax_len, 0.0);
        let p_z = Point3::new(0.0, 0.0, ax_len);

        if let Some((s_origin, _)) = camera.project(origin, rect) {
            if let Some((s_x, _)) = camera.project(p_x, rect) {
                painter.line_segment(
                    [s_origin, s_x],
                    Stroke::new(2.5_f32, Color32::from_rgb(235, 60, 60)),
                );
            }
            if let Some((s_y, _)) = camera.project(p_y, rect) {
                painter.line_segment(
                    [s_origin, s_y],
                    Stroke::new(2.5_f32, Color32::from_rgb(60, 210, 60)),
                );
            }
            if let Some((s_z, _)) = camera.project(p_z, rect) {
                painter.line_segment(
                    [s_origin, s_z],
                    Stroke::new(2.5_f32, Color32::from_rgb(60, 130, 245)),
                );
            }
        }
    }

    // 3. Draw Workspace Reach Envelope (Sphere wireframe)
    if settings.show_reach_envelope {
        let reach = robot.total_reach() as f32;
        let num_pts = 36;
        let mut prev_pt: Option<Pos2> = None;
        for i in 0..=num_pts {
            let theta = (i as f32 / num_pts as f32) * std::f32::consts::TAU;
            let pt = Point3::new(reach * theta.cos(), reach * theta.sin(), 0.05);
            if let Some((s_pt, _)) = camera.project(pt, rect) {
                if let Some(prev) = prev_pt {
                    painter.line_segment(
                        [prev, s_pt],
                        Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(80, 160, 240, 60)),
                    );
                }
                prev_pt = Some(s_pt);
            }
        }
    }

    // 4. Draw Planned Trajectory Path and Waypoint Markers
    if settings.show_planned_path && !planner.waypoints.is_empty() {
        let path_samples = planner.sample_path(20);
        let mut prev_pos: Option<Pos2> = None;

        for pt in &path_samples {
            let p_f32 = Point3::new(pt.x as f32, pt.y as f32, pt.z as f32);
            if let Some((s_pt, _)) = camera.project(p_f32, rect) {
                if let Some(prev) = prev_pos {
                    painter.line_segment(
                        [prev, s_pt],
                        Stroke::new(1.8_f32, Color32::from_rgba_unmultiplied(255, 200, 50, 160)),
                    );
                }
                prev_pos = Some(s_pt);
            }
        }

        // Draw Waypoint Spheres
        for (idx, wp) in planner.waypoints.iter().enumerate() {
            let p_f32 = Point3::new(
                wp.position.x as f32,
                wp.position.y as f32,
                wp.position.z as f32,
            );
            if let Some((s_pt, depth)) = camera.project(p_f32, rect) {
                let radius = (6.0 / depth).clamp(3.0, 10.0);
                painter.circle_filled(s_pt, radius, Color32::from_rgb(255, 180, 40));
                painter.circle_stroke(s_pt, radius, Stroke::new(1.5_f32, Color32::WHITE));

                let text_pos = s_pt + egui::vec2(radius + 3.0, -radius);
                painter.text(
                    text_pos,
                    egui::Align2::LEFT_CENTER,
                    format!("WP {}", idx + 1),
                    FontId::monospace(10.0),
                    Color32::from_rgb(240, 200, 100),
                );
            }
        }
    }

    // 5. Draw End-Effector Motion Trail
    if settings.show_trail && trail.len() > 1 {
        let n = trail.len();
        for i in 0..n - 1 {
            let p1 = Point3::new(trail[i].x as f32, trail[i].y as f32, trail[i].z as f32);
            let p2 = Point3::new(
                trail[i + 1].x as f32,
                trail[i + 1].y as f32,
                trail[i + 1].z as f32,
            );
            if let (Some((s1, _)), Some((s2, _))) =
                (camera.project(p1, rect), camera.project(p2, rect))
            {
                let alpha = ((i + 1) as f32 / n as f32 * 200.0) as u8;
                let stroke_width = (1.0 + (i as f32 / n as f32) * 2.0).clamp(1.0, 3.0);
                painter.line_segment(
                    [s1, s2],
                    Stroke::new(
                        stroke_width,
                        Color32::from_rgba_unmultiplied(70, 220, 200, alpha),
                    ),
                );
            }
        }
    }

    // 6. Draw Solid 3D Scene Components (Robot, Table, Workpieces, Obstacles)
    let poses = robot.forward_kinematics();
    let num_frames = poses.len();

    let light_dir = Vector3::new(0.42, -0.62, 0.85).normalize();
    let fill_dir = Vector3::new(-0.55, 0.45, 0.35).normalize();
    let mut primitives = Vec::with_capacity(512);

    if settings.show_workcell_table {
        render_workcell_environment(
            &mut primitives,
            camera,
            rect,
            environment,
            light_dir,
            fill_dir,
        );
    }

    if settings.show_obstacles {
        render_obstacles(
            &mut primitives,
            camera,
            rect,
            environment,
            light_dir,
            fill_dir,
        );
    }

    if settings.show_workpieces {
        render_workpieces(
            &mut primitives,
            camera,
            rect,
            workpieces,
            light_dir,
            fill_dir,
        );
    }

    if settings.show_solid_mesh {
        render_solid_robot_arm(
            &mut primitives,
            camera,
            rect,
            robot,
            settings,
            tool_state,
            &collision_report.colliding_links,
            light_dir,
            fill_dir,
        );
    } else {
        // Fallback wireframe mode
        for i in 0..num_frames - 1 {
            let p_start = Point3::from(poses[i].translation.vector.cast::<f32>());
            let p_end = Point3::from(poses[i + 1].translation.vector.cast::<f32>());

            let link_color = if collision_report.colliding_links.contains(&i) {
                Color32::from_rgb(255, 45, 45)
            } else if i < robot.links.len() {
                let c = robot.links[i].color;
                Color32::from_rgba_unmultiplied(
                    (c[0] * 255.0) as u8,
                    (c[1] * 255.0) as u8,
                    (c[2] * 255.0) as u8,
                    255,
                )
            } else {
                Color32::from_rgb(180, 180, 200)
            };

            if let (Some((s_start, d_start)), Some((s_end, d_end))) =
                (camera.project(p_start, rect), camera.project(p_end, rect))
            {
                let avg_depth = (d_start + d_end) * 0.5;
                let link_width = (16.0 / avg_depth).clamp(4.0, 26.0);

                painter.line_segment([s_start, s_end], Stroke::new(link_width, link_color));

                let joint_radius = link_width * 0.7;
                painter.circle_filled(s_start, joint_radius, Color32::from_rgb(60, 65, 80));
                painter.circle_stroke(
                    s_start,
                    joint_radius,
                    Stroke::new(1.5_f32, Color32::from_rgb(140, 150, 175)),
                );
            }
        }
    }

    // Sort all queued primitives from furthest to closest (Painter's algorithm)
    primitives.sort_unstable_by(|a, b| {
        b.depth()
            .partial_cmp(&a.depth())
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    for prim in primitives {
        match prim {
            RenderPrimitive::Polygon {
                points,
                fill,
                stroke,
                ..
            } => {
                painter.add(Shape::convex_polygon(points, fill, stroke));
            }
            RenderPrimitive::Line { p1, p2, stroke, .. } => {
                painter.line_segment([p1, p2], stroke);
            }
            RenderPrimitive::Circle {
                center,
                radius,
                fill,
                stroke,
                ..
            } => {
                painter.circle_filled(center, radius, fill);
                painter.circle_stroke(center, radius, stroke);
            }
        }
    }

    // Optional Manipulability Ellipsoid
    if settings.show_ellipsoid {
        if let Some(m_data) = manip_data {
            if let Some(ee_pose) = poses.last() {
                let tcp = Point3::from(ee_pose.translation.vector.cast::<f32>());
                render_manipulability_ellipsoid(&painter, camera, rect, tcp, m_data);
            }
        }
    }

    // Optional Joint Coordinate Frame Triads
    if settings.show_joint_frames {
        for pose in &poses {
            let p = Point3::from(pose.translation.vector.cast::<f32>());
            if let Some((s_origin, _)) = camera.project(p, rect) {
                let rot = pose.rotation.cast::<f32>();
                let axis_len = 0.06_f32;
                let px = p + rot * Vector3::x() * axis_len;
                let py = p + rot * Vector3::y() * axis_len;
                let pz = p + rot * Vector3::z() * axis_len;

                if let Some((sx, _)) = camera.project(px, rect) {
                    painter.line_segment(
                        [s_origin, sx],
                        Stroke::new(2.0_f32, Color32::from_rgb(240, 50, 50)),
                    );
                }
                if let Some((sy, _)) = camera.project(py, rect) {
                    painter.line_segment(
                        [s_origin, sy],
                        Stroke::new(2.0_f32, Color32::from_rgb(50, 220, 50)),
                    );
                }
                if let Some((sz, _)) = camera.project(pz, rect) {
                    painter.line_segment(
                        [s_origin, sz],
                        Stroke::new(2.0_f32, Color32::from_rgb(50, 120, 240)),
                    );
                }
            }
        }
    }

    // 7. Draw Target Destination Sphere & 3D Interactive Gizmo (Translation & Rotation)
    let target_f32 = Point3::new(
        target_pos.x as f32,
        target_pos.y as f32,
        target_pos.z as f32,
    );

    // Subtle laser guidance line from gripper TCP to target
    if settings.show_solid_mesh && settings.show_gripper {
        if let Some(ee_pose) = poses.last() {
            let p_ee = Point3::from(ee_pose.translation.vector.cast::<f32>());
            let r_ee = ee_pose.rotation.cast::<f32>();
            let u_z = if poses.len() >= 2 {
                let p_penult =
                    Point3::from(poses[poses.len() - 2].translation.vector.cast::<f32>());
                let dir = p_ee - p_penult;
                if dir.norm() > 1e-3 {
                    dir.normalize()
                } else {
                    r_ee * Vector3::x()
                }
            } else {
                r_ee * Vector3::x()
            };
            let p_tcp = p_ee + u_z * 0.098;
            let dist = (target_pos - p_tcp.cast::<f64>()).norm();
            if dist < 0.8 {
                if let (Some((s_tcp, _)), Some((s_t, _))) = (
                    camera.project(p_tcp, rect),
                    camera.project(target_f32, rect),
                ) {
                    painter.line_segment(
                        [s_tcp, s_t],
                        Stroke::new(1.2_f32, Color32::from_rgba_unmultiplied(0, 235, 215, 120)),
                    );
                }
            }
        }
    }

    let mouse_pos = ui
        .input(|i| {
            i.pointer
                .interact_pos()
                .or_else(|| i.pointer.latest_pos())
                .or_else(|| i.pointer.hover_pos())
        })
        .unwrap_or(Pos2::new(-9999.0, -9999.0));

    // Check if user is hovering or clicking directly on the robot's end-effector / gripper
    let p_ee = robot.end_effector_position();
    let p_ee_f32 = Point3::new(p_ee.x as f32, p_ee.y as f32, p_ee.z as f32);
    let (is_hand_hovered, s_ee_opt) = if let Some((s_ee, ee_depth)) = camera.project(p_ee_f32, rect)
    {
        let r_hand = (48.0 / ee_depth).clamp(28.0, 65.0);
        let hovered = (mouse_pos - s_ee).length() <= r_hand;
        (hovered, Some((s_ee, ee_depth)))
    } else {
        (false, None)
    };

    if let Some((s_target, depth)) = camera.project(target_f32, rect) {
        let target_radius = (16.0 / depth).clamp(9.0, 26.0);
        let dist_to_mouse = (mouse_pos - s_target).length();
        let is_center_hovered = dist_to_mouse <= (target_radius * 1.5);

        // Visual feedback on robot hand
        if let Some((s_ee, ee_depth)) = s_ee_opt {
            let r_glow = (34.0 / ee_depth).clamp(20.0, 50.0);
            if active_drag_axis == GizmoDragAxis::TargetCenter {
                painter.circle_stroke(
                    s_ee,
                    r_glow,
                    Stroke::new(2.5_f32, Color32::from_rgb(255, 230, 80)),
                );
            } else if is_hand_hovered {
                painter.circle_stroke(
                    s_ee,
                    r_glow,
                    Stroke::new(2.5_f32, Color32::from_rgb(0, 240, 220)),
                );
                painter.text(
                    s_ee + egui::vec2(0.0, r_glow + 8.0),
                    egui::Align2::CENTER_TOP,
                    "🖐️ Drag Hand (Left Click)",
                    FontId::proportional(11.0),
                    Color32::from_rgb(0, 240, 220),
                );
            }
        }

        // Glowing outer pulse on target handle
        painter.circle_filled(
            s_target,
            target_radius * 1.5,
            Color32::from_rgba_unmultiplied(255, 170, 0, 45),
        );
        // Center sphere
        let center_color = if active_drag_axis == GizmoDragAxis::TargetCenter
            || is_center_hovered
            || is_hand_hovered
        {
            Color32::from_rgb(255, 230, 80)
        } else {
            Color32::from_rgb(255, 160, 20)
        };
        painter.circle_filled(s_target, target_radius, center_color);
        painter.circle_stroke(
            s_target,
            target_radius,
            Stroke::new(2.0_f32, Color32::WHITE),
        );

        if is_center_hovered || is_hand_hovered {
            hovered_axis = GizmoDragAxis::TargetCenter;
        }

        // 3D Axis Handles for interactive translation
        let handle_len = 0.18_f32;
        let px_end = target_f32 + Vector3::new(handle_len, 0.0, 0.0);
        let py_end = target_f32 + Vector3::new(0.0, handle_len, 0.0);
        let pz_end = target_f32 + Vector3::new(0.0, 0.0, handle_len);

        // X Axis (Red)
        if let Some((s_x, _)) = camera.project(px_end, rect) {
            let is_x_active = active_drag_axis == GizmoDragAxis::AxisX;
            let is_x_hovered = (mouse_pos - s_x).length() < 18.0
                || point_near_segment(mouse_pos, s_target, s_x, 12.0);
            if is_x_hovered {
                hovered_axis = GizmoDragAxis::AxisX;
            }
            let color = if is_x_active || is_x_hovered {
                Color32::from_rgb(255, 100, 100)
            } else {
                Color32::from_rgb(230, 40, 40)
            };
            painter.line_segment([s_target, s_x], Stroke::new(3.5_f32, color));
            painter.circle_filled(s_x, 6.0, color);
            painter.text(
                s_x + egui::vec2(8.0, 0.0),
                egui::Align2::LEFT_CENTER,
                "+X",
                FontId::monospace(10.0),
                color,
            );
        }

        // Y Axis (Green)
        if let Some((s_y, _)) = camera.project(py_end, rect) {
            let is_y_active = active_drag_axis == GizmoDragAxis::AxisY;
            let is_y_hovered = (mouse_pos - s_y).length() < 18.0
                || point_near_segment(mouse_pos, s_target, s_y, 12.0);
            if is_y_hovered {
                hovered_axis = GizmoDragAxis::AxisY;
            }
            let color = if is_y_active || is_y_hovered {
                Color32::from_rgb(100, 255, 100)
            } else {
                Color32::from_rgb(40, 210, 40)
            };
            painter.line_segment([s_target, s_y], Stroke::new(3.5_f32, color));
            painter.circle_filled(s_y, 6.0, color);
            painter.text(
                s_y + egui::vec2(8.0, 0.0),
                egui::Align2::LEFT_CENTER,
                "+Y",
                FontId::monospace(10.0),
                color,
            );
        }

        // Z Axis (Blue)
        if let Some((s_z, _)) = camera.project(pz_end, rect) {
            let is_z_active = active_drag_axis == GizmoDragAxis::AxisZ;
            let is_z_hovered = (mouse_pos - s_z).length() < 18.0
                || point_near_segment(mouse_pos, s_target, s_z, 12.0);
            if is_z_hovered {
                hovered_axis = GizmoDragAxis::AxisZ;
            }
            let color = if is_z_active || is_z_hovered {
                Color32::from_rgb(120, 180, 255)
            } else {
                Color32::from_rgb(50, 130, 255)
            };
            painter.line_segment([s_target, s_z], Stroke::new(3.5_f32, color));
            painter.circle_filled(s_z, 6.0, color);
            painter.text(
                s_z + egui::vec2(8.0, 0.0),
                egui::Align2::LEFT_CENTER,
                "+Z",
                FontId::monospace(10.0),
                color,
            );
        }

        // 3D Orbital Rotation Gizmo Rings (Yaw, Pitch, Roll)
        if settings.show_rotation_gizmo {
            let r_ring = 0.15_f32;
            let num_ring_pts = 24;

            // 1. Roll Ring (Red, around X in YZ plane)
            let mut prev_roll: Option<Pos2> = None;
            let mut is_roll_hovered = false;
            for k in 0..=num_ring_pts {
                let theta = (k as f32 / num_ring_pts as f32) * std::f32::consts::TAU;
                let pt = target_f32 + Vector3::new(0.0, r_ring * theta.cos(), r_ring * theta.sin());
                if let Some((s_pt, _)) = camera.project(pt, rect) {
                    if let Some(prev) = prev_roll {
                        if point_near_segment(mouse_pos, prev, s_pt, 11.0) {
                            is_roll_hovered = true;
                        }
                    }
                    prev_roll = Some(s_pt);
                }
            }
            if is_roll_hovered {
                hovered_axis = GizmoDragAxis::RotRoll;
            }
            let roll_color = if active_drag_axis == GizmoDragAxis::RotRoll || is_roll_hovered {
                Color32::from_rgb(255, 120, 120)
            } else {
                Color32::from_rgba_unmultiplied(230, 50, 50, 160)
            };
            let roll_width = if active_drag_axis == GizmoDragAxis::RotRoll || is_roll_hovered {
                3.5_f32
            } else {
                2.0_f32
            };
            let mut prev_pt: Option<Pos2> = None;
            for k in 0..=num_ring_pts {
                let theta = (k as f32 / num_ring_pts as f32) * std::f32::consts::TAU;
                let pt = target_f32 + Vector3::new(0.0, r_ring * theta.cos(), r_ring * theta.sin());
                if let Some((s_pt, _)) = camera.project(pt, rect) {
                    if let Some(prev) = prev_pt {
                        painter.line_segment([prev, s_pt], Stroke::new(roll_width, roll_color));
                    }
                    prev_pt = Some(s_pt);
                }
            }

            // 2. Pitch Ring (Green, around Y in XZ plane)
            let mut prev_pitch: Option<Pos2> = None;
            let mut is_pitch_hovered = false;
            for k in 0..=num_ring_pts {
                let theta = (k as f32 / num_ring_pts as f32) * std::f32::consts::TAU;
                let pt = target_f32 + Vector3::new(r_ring * theta.cos(), 0.0, r_ring * theta.sin());
                if let Some((s_pt, _)) = camera.project(pt, rect) {
                    if let Some(prev) = prev_pitch {
                        if point_near_segment(mouse_pos, prev, s_pt, 11.0) {
                            is_pitch_hovered = true;
                        }
                    }
                    prev_pitch = Some(s_pt);
                }
            }
            if is_pitch_hovered {
                hovered_axis = GizmoDragAxis::RotPitch;
            }
            let pitch_color = if active_drag_axis == GizmoDragAxis::RotPitch || is_pitch_hovered {
                Color32::from_rgb(120, 255, 120)
            } else {
                Color32::from_rgba_unmultiplied(50, 210, 50, 160)
            };
            let pitch_width = if active_drag_axis == GizmoDragAxis::RotPitch || is_pitch_hovered {
                3.5_f32
            } else {
                2.0_f32
            };
            let mut prev_p: Option<Pos2> = None;
            for k in 0..=num_ring_pts {
                let theta = (k as f32 / num_ring_pts as f32) * std::f32::consts::TAU;
                let pt = target_f32 + Vector3::new(r_ring * theta.cos(), 0.0, r_ring * theta.sin());
                if let Some((s_pt, _)) = camera.project(pt, rect) {
                    if let Some(prev) = prev_p {
                        painter.line_segment([prev, s_pt], Stroke::new(pitch_width, pitch_color));
                    }
                    prev_p = Some(s_pt);
                }
            }

            // 3. Yaw Ring (Blue, around Z in XY plane)
            let mut prev_yaw: Option<Pos2> = None;
            let mut is_yaw_hovered = false;
            for k in 0..=num_ring_pts {
                let theta = (k as f32 / num_ring_pts as f32) * std::f32::consts::TAU;
                let pt = target_f32 + Vector3::new(r_ring * theta.cos(), r_ring * theta.sin(), 0.0);
                if let Some((s_pt, _)) = camera.project(pt, rect) {
                    if let Some(prev) = prev_yaw {
                        if point_near_segment(mouse_pos, prev, s_pt, 11.0) {
                            is_yaw_hovered = true;
                        }
                    }
                    prev_yaw = Some(s_pt);
                }
            }
            if is_yaw_hovered {
                hovered_axis = GizmoDragAxis::RotYaw;
            }
            let yaw_color = if active_drag_axis == GizmoDragAxis::RotYaw || is_yaw_hovered {
                Color32::from_rgb(120, 190, 255)
            } else {
                Color32::from_rgba_unmultiplied(50, 140, 255, 160)
            };
            let yaw_width = if active_drag_axis == GizmoDragAxis::RotYaw || is_yaw_hovered {
                3.5_f32
            } else {
                2.0_f32
            };
            let mut prev_y: Option<Pos2> = None;
            for k in 0..=num_ring_pts {
                let theta = (k as f32 / num_ring_pts as f32) * std::f32::consts::TAU;
                let pt = target_f32 + Vector3::new(r_ring * theta.cos(), r_ring * theta.sin(), 0.0);
                if let Some((s_pt, _)) = camera.project(pt, rect) {
                    if let Some(prev) = prev_y {
                        painter.line_segment([prev, s_pt], Stroke::new(yaw_width, yaw_color));
                    }
                    prev_y = Some(s_pt);
                }
            }
        }
    }

    if hovered_axis != GizmoDragAxis::None {
        Some(hovered_axis)
    } else {
        None
    }
}

/// Helper to check if a 2D point is near a line segment.
fn point_near_segment(p: Pos2, a: Pos2, b: Pos2, tolerance: f32) -> bool {
    let ab = b - a;
    let len_sq = ab.length_sq();
    if len_sq < 1e-4 {
        return (p - a).length() <= tolerance;
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    let projection = a + ab * t;
    (p - projection).length() <= tolerance
}
