use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};

/// Creates a unit quaternion from Euler angles (Roll-Pitch-Yaw in ZYX convention).
pub fn euler_to_quaternion(roll: f64, pitch: f64, yaw: f64) -> UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(roll, pitch, yaw)
}

/// Converts a unit quaternion to Euler angles (Roll, Pitch, Yaw in radians).
pub fn quaternion_to_euler(q: &UnitQuaternion<f64>) -> (f64, f64, f64) {
    q.euler_angles()
}

/// Constructs an Isometry3 from translation vector and RPY Euler angles.
pub fn make_isometry(translation: Vector3<f64>, rpy: Vector3<f64>) -> Isometry3<f64> {
    let rot = euler_to_quaternion(rpy.x, rpy.y, rpy.z);
    Isometry3::from_parts(translation.into(), rot)
}

/// Represents a ray in 3D space with an origin and normalized direction.
#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub origin: Point3<f64>,
    pub direction: Vector3<f64>,
}

impl Ray {
    pub fn new(origin: Point3<f64>, direction: Vector3<f64>) -> Self {
        Self {
            origin,
            direction: direction.normalize(),
        }
    }

    /// Computes intersection point with a plane defined by a point on plane and normal vector.
    pub fn intersect_plane(
        &self,
        plane_point: Point3<f64>,
        plane_normal: Vector3<f64>,
    ) -> Option<Point3<f64>> {
        let denom = self.direction.dot(&plane_normal);
        if denom.abs() < 1e-6 {
            return None; // Ray is parallel to plane
        }
        let t = (plane_point - self.origin).dot(&plane_normal) / denom;
        if t >= 0.0 {
            Some(self.origin + self.direction * t)
        } else {
            None
        }
    }

    /// Computes the closest distance between this ray and a 3D line segment (p1, p2).
    pub fn distance_to_segment(&self, p1: Point3<f64>, p2: Point3<f64>) -> f64 {
        let u = self.direction;
        let v = p2 - p1;
        let w = self.origin - p1;

        let a = u.dot(&u); // always 1.0 since normalized
        let b = u.dot(&v);
        let c = v.dot(&v);
        let d = u.dot(&w);
        let e = v.dot(&w);

        let denom = a * c - b * b;
        let (sc, tc);

        if denom < 1e-6 {
            sc = 0.0;
            tc = if b > c { d / b } else { e / c };
        } else {
            sc = (b * e - c * d) / denom;
            tc = (a * e - b * d) / denom;
        }

        let tc_clamped = tc.clamp(0.0, 1.0);
        let sc_clamped = sc.max(0.0);

        let pt_ray = self.origin + u * sc_clamped;
        let pt_seg = p1 + v * tc_clamped;
        (pt_ray - pt_seg).norm()
    }
}
