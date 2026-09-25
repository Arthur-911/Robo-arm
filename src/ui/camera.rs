use egui::{Pos2, Rect, Vec2};
use nalgebra::{Point3, Vector3};

use crate::math::Ray;

/// An intuitive 3D Orbit & Pan camera tailored for robotics inspection.
#[derive(Debug, Clone)]
pub struct OrbitCamera {
    /// Focal point that the camera orbits around (in meters).
    pub target: Point3<f32>,
    /// Azimuthal angle around the Z axis (radians).
    pub azimuth: f32,
    /// Elevation angle above the ground plane (radians).
    pub elevation: f32,
    /// Distance from the camera eye to the focal target.
    pub distance: f32,
    /// Vertical Field of View (radians).
    pub fov_y: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: Point3::new(0.0, 0.0, 0.35),
            azimuth: 0.75,   // ~43 degrees
            elevation: 0.52, // ~30 degrees
            distance: 2.2,
            fov_y: 45.0_f32.to_radians(),
        }
    }
}

impl OrbitCamera {
    /// Resets the camera to default home view.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Focuses the camera center directly on a 3D coordinate (e.g. TCP or object).
    pub fn focus_on(&mut self, point: Point3<f32>) {
        self.target = point;
        self.distance = self.distance.clamp(0.6, 2.5);
    }

    /// Computes the camera eye position in world space.
    pub fn eye_position(&self) -> Point3<f32> {
        let cos_elev = self.elevation.cos();
        let sin_elev = self.elevation.sin();
        let cos_azim = self.azimuth.cos();
        let sin_azim = self.azimuth.sin();

        Point3::new(
            self.target.x + self.distance * cos_elev * sin_azim,
            self.target.y - self.distance * cos_elev * cos_azim,
            self.target.z + self.distance * sin_elev,
        )
    }

    /// Rotates the camera (orbital drag).
    pub fn rotate(&mut self, delta: Vec2) {
        let sensitivity = 0.006;
        self.azimuth += delta.x * sensitivity;
        self.elevation -= delta.y * sensitivity;
        // Clamp elevation to avoid pole flipping
        let max_elev = 85.0_f32.to_radians();
        self.elevation = self.elevation.clamp(-max_elev, max_elev);
    }

    /// Pans the camera in the view plane.
    pub fn pan(&mut self, delta: Vec2) {
        let eye = self.eye_position();
        let forward = (self.target - eye).normalize();
        let world_up = Vector3::new(0.0, 0.0, 1.0);
        let right = forward.cross(&world_up).normalize();
        let up = right.cross(&forward).normalize();

        let pan_scale = self.distance * 0.0015;
        let shift = -right * (delta.x * pan_scale) + up * (delta.y * pan_scale);
        self.target += shift;
    }

    /// Returns the (right, up) orthogonal unit vectors defining the camera view plane.
    pub fn view_plane_axes(&self) -> (Vector3<f32>, Vector3<f32>) {
        let eye = self.eye_position();
        let forward = (self.target - eye).normalize();
        let world_up = Vector3::new(0.0, 0.0, 1.0);
        let right = if forward.cross(&world_up).norm() > 1e-4 {
            forward.cross(&world_up).normalize()
        } else {
            Vector3::x()
        };
        let up = right.cross(&forward).normalize();
        (right, up)
    }

    /// Converts a 2D screen delta (pixels) into a 3D world displacement in the view plane at given depth.
    pub fn screen_delta_to_world(&self, delta_screen: Vec2, depth: f32, rect: Rect) -> Vector3<f32> {
        let (right, up) = self.view_plane_axes();
        let focal = 1.0 / (self.fov_y * 0.5).tan();
        let k = (focal * rect.height() * 0.5).max(1.0);
        let dx_cam = delta_screen.x * depth / k;
        let dy_cam = -delta_screen.y * depth / k;
        right * dx_cam + up * dy_cam
    }

    /// Projects screen drag delta onto a 3D world axis handle.
    pub fn project_axis_delta(&self, origin: Point3<f32>, axis: Vector3<f32>, delta_screen: Vec2, rect: Rect) -> f32 {
        let handle_len = 0.18_f32;
        if let (Some((s0, _)), Some((s1, _))) = (self.project(origin, rect), self.project(origin + axis * handle_len, rect)) {
            let d = s1 - s0;
            let l_sq = d.length_sq();
            if l_sq > 4.0 {
                let proj_pixels = delta_screen.dot(d);
                return (proj_pixels / l_sq) * handle_len;
            }
        }
        0.0
    }

    /// Computes the angle swept around the projected origin by a mouse drag.
    pub fn project_ring_angle(&self, origin: Point3<f32>, start_mouse: Pos2, curr_mouse: Pos2, rect: Rect) -> f32 {
        if let Some((s_center, _)) = self.project(origin, rect) {
            let v_start = start_mouse - s_center;
            let v_curr = curr_mouse - s_center;
            if v_start.length() > 4.0 && v_curr.length() > 4.0 {
                let theta_start = v_start.y.atan2(v_start.x);
                let theta_curr = v_curr.y.atan2(v_curr.x);
                let mut diff = theta_curr - theta_start;
                while diff > std::f32::consts::PI { diff -= std::f32::consts::TAU; }
                while diff < -std::f32::consts::PI { diff += std::f32::consts::TAU; }
                return diff.to_degrees();
            }
        }
        0.0
    }

    /// Zooms the camera closer or further from target.
    pub fn zoom(&mut self, scroll_delta: f32) {
        let zoom_factor = (1.0 - scroll_delta * 0.002).clamp(0.5, 1.8);
        self.distance = (self.distance * zoom_factor).clamp(0.2, 20.0);
    }

    /// Projects a 3D world coordinate point to 2D screen coordinates within `rect`.
    /// Returns `Some((screen_pos, depth))` if point is in front of the camera.
    pub fn project(&self, point: Point3<f32>, rect: Rect) -> Option<(Pos2, f32)> {
        let eye = self.eye_position();
        let forward = (self.target - eye).normalize();
        let world_up = Vector3::new(0.0, 0.0, 1.0);
        let right = forward.cross(&world_up).normalize();
        let up = right.cross(&forward).normalize();

        let v = point - eye;
        let z_cam = v.dot(&forward);
        if z_cam <= 0.05 {
            return None; // Clipped behind near plane
        }

        let x_cam = v.dot(&right);
        let y_cam = v.dot(&up);

        let aspect = rect.width() / rect.height().max(1.0);
        let focal = 1.0 / (self.fov_y * 0.5).tan();

        let u_ndc = (x_cam / z_cam) * (focal / aspect);
        let v_ndc = (y_cam / z_cam) * focal;

        let screen_x = rect.center().x + u_ndc * (rect.width() * 0.5);
        let screen_y = rect.center().y - v_ndc * (rect.height() * 0.5);

        Some((Pos2::new(screen_x, screen_y), z_cam))
    }

    /// Casts a ray from a 2D screen coordinate through the camera projection.
    pub fn screen_to_ray(&self, screen_pos: Pos2, rect: Rect) -> Ray {
        let eye = self.eye_position();
        let forward = (self.target - eye).normalize();
        let world_up = Vector3::new(0.0, 0.0, 1.0);
        let right = forward.cross(&world_up).normalize();
        let up = right.cross(&forward).normalize();

        let aspect = rect.width() / rect.height().max(1.0);
        let focal = 1.0 / (self.fov_y * 0.5).tan();

        let u_ndc = (screen_pos.x - rect.center().x) / (rect.width() * 0.5);
        let v_ndc = -(screen_pos.y - rect.center().y) / (rect.height() * 0.5);

        let x_cam = (u_ndc / focal) * aspect;
        let y_cam = v_ndc / focal;
        let z_cam = 1.0;

        let ray_dir = (right * x_cam + up * y_cam + forward * z_cam).normalize();

        Ray::new(
            Point3::new(eye.x as f64, eye.y as f64, eye.z as f64),
            Vector3::new(ray_dir.x as f64, ray_dir.y as f64, ray_dir.z as f64),
        )
    }
}
