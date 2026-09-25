use nalgebra::{Point3, Vector3};
use serde::{Deserialize, Serialize};

use super::polynomial::{CubicPolynomial, PolynomialType, QuinticPolynomial};

/// Mode determining velocity and acceleration continuity across waypoints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum TrajectoryContinuityMode {
    /// Seamless C1/C2 continuity: computes continuous velocity and acceleration vectors
    /// across waypoints (minimum-jerk fly-through without halting).
    #[default]
    SmoothContinuous,
    /// Traditional point-to-point: arm decelerates to a complete stop at each waypoint (dwell mode).
    StopAtWaypoints,
}

/// A 3D spatial waypoint along a planned robotic trajectory.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Waypoint {
    pub name: String,
    pub position: Point3<f64>,
    /// Time in seconds to reach this waypoint from the preceding waypoint.
    pub duration: f64,
}

impl Waypoint {
    pub fn new(name: impl Into<String>, position: Point3<f64>, duration: f64) -> Self {
        Self {
            name: name.into(),
            position,
            duration: duration.max(0.1),
        }
    }
}

/// A 3D segment interpolating between two waypoints.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Segment3D {
    Cubic {
        x: CubicPolynomial,
        y: CubicPolynomial,
        z: CubicPolynomial,
        duration: f64,
    },
    Quintic {
        x: QuinticPolynomial,
        y: QuinticPolynomial,
        z: QuinticPolynomial,
        duration: f64,
    },
}

impl Segment3D {
    pub fn evaluate(&self, t: f64) -> Point3<f64> {
        match self {
            Self::Cubic { x, y, z, .. } => Point3::new(x.position(t), y.position(t), z.position(t)),
            Self::Quintic { x, y, z, .. } => {
                Point3::new(x.position(t), y.position(t), z.position(t))
            }
        }
    }

    pub fn evaluate_velocity(&self, t: f64) -> Vector3<f64> {
        match self {
            Self::Cubic { x, y, z, .. } => Vector3::new(x.velocity(t), y.velocity(t), z.velocity(t)),
            Self::Quintic { x, y, z, .. } => {
                Vector3::new(x.velocity(t), y.velocity(t), z.velocity(t))
            }
        }
    }

    pub fn evaluate_acceleration(&self, t: f64) -> Vector3<f64> {
        match self {
            Self::Cubic { x, y, z, .. } => {
                Vector3::new(x.acceleration(t), y.acceleration(t), z.acceleration(t))
            }
            Self::Quintic { x, y, z, .. } => {
                Vector3::new(x.acceleration(t), y.acceleration(t), z.acceleration(t))
            }
        }
    }

    pub fn duration(&self) -> f64 {
        match self {
            Self::Cubic { duration, .. } => *duration,
            Self::Quintic { duration, .. } => *duration,
        }
    }
}

/// Multi-waypoint trajectory generator supporting minimum-jerk smooth interpolation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrajectoryPlanner {
    pub waypoints: Vec<Waypoint>,
    pub poly_type: PolynomialType,
    #[serde(default)]
    pub continuity_mode: TrajectoryContinuityMode,
    pub segments: Vec<Segment3D>,
    pub total_duration: f64,
    #[serde(default)]
    pub cumulative_times: Vec<f64>,
}

impl Default for TrajectoryPlanner {
    fn default() -> Self {
        let mut planner = Self {
            waypoints: vec![
                Waypoint::new("Start", Point3::new(0.4, 0.0, 0.4), 2.0),
                Waypoint::new("WP 1", Point3::new(0.3, 0.3, 0.5), 2.0),
                Waypoint::new("WP 2", Point3::new(-0.2, 0.4, 0.3), 2.5),
                Waypoint::new("WP 3", Point3::new(-0.3, -0.2, 0.4), 2.5),
                Waypoint::new("End", Point3::new(0.4, 0.0, 0.4), 2.0),
            ],
            poly_type: PolynomialType::Quintic,
            continuity_mode: TrajectoryContinuityMode::SmoothContinuous,
            segments: Vec::new(),
            total_duration: 0.0,
            cumulative_times: Vec::new(),
        };
        planner.rebuild_trajectory();
        planner
    }
}

impl TrajectoryPlanner {
    pub fn add_waypoint(&mut self, name: impl Into<String>, position: Point3<f64>, duration: f64) {
        self.waypoints.push(Waypoint::new(name, position, duration));
        self.rebuild_trajectory();
    }

    pub fn remove_waypoint(&mut self, index: usize) {
        if index < self.waypoints.len() {
            self.waypoints.remove(index);
            self.rebuild_trajectory();
        }
    }

    pub fn clear_waypoints(&mut self) {
        self.waypoints.clear();
        self.segments.clear();
        self.cumulative_times.clear();
        self.total_duration = 0.0;
    }

    pub fn set_polynomial_type(&mut self, poly_type: PolynomialType) {
        self.poly_type = poly_type;
        self.rebuild_trajectory();
    }

    pub fn set_continuity_mode(&mut self, mode: TrajectoryContinuityMode) {
        self.continuity_mode = mode;
        self.rebuild_trajectory();
    }

    /// Reconstructs the piecewise polynomial segments connecting waypoints with continuous C1/C2 derivatives.
    pub fn rebuild_trajectory(&mut self) {
        self.segments.clear();
        self.cumulative_times.clear();
        self.total_duration = 0.0;

        let n = self.waypoints.len();
        if n < 2 {
            if n == 1 {
                self.cumulative_times.push(0.0);
            }
            return;
        }

        // 1. Calculate waypoint tangent velocities and accelerations
        let mut velocities = vec![Vector3::zeros(); n];
        let mut accelerations = vec![Vector3::zeros(); n];

        if self.continuity_mode == TrajectoryContinuityMode::SmoothContinuous && n >= 3 {
            // Catmull-Rom centripetal tangent velocities for intermediate waypoints
            for i in 1..n - 1 {
                let dt_prev = self.waypoints[i].duration.max(1e-4);
                let dt_next = self.waypoints[i + 1].duration.max(1e-4);
                let dp = self.waypoints[i + 1].position - self.waypoints[i - 1].position;
                velocities[i] = dp / (dt_prev + dt_next);
            }

            // Continuous intermediate accelerations
            for i in 1..n - 1 {
                let dt_prev = self.waypoints[i].duration.max(1e-4);
                let dt_next = self.waypoints[i + 1].duration.max(1e-4);
                let dv = velocities[i + 1] - velocities[i - 1];
                accelerations[i] = dv / (dt_prev + dt_next);
            }
        }

        // 2. Build piecewise polynomial segments
        self.cumulative_times.reserve(n);
        self.cumulative_times.push(0.0);
        let mut accum_time = 0.0;

        for i in 0..n - 1 {
            let p0 = self.waypoints[i].position;
            let p1 = self.waypoints[i + 1].position;
            let dur = self.waypoints[i + 1].duration.max(1e-4);

            let v0 = velocities[i];
            let v1 = velocities[i + 1];
            let a0 = accelerations[i];
            let a1 = accelerations[i + 1];

            let segment = match self.poly_type {
                PolynomialType::Cubic => Segment3D::Cubic {
                    x: CubicPolynomial::new(p0.x, p1.x, v0.x, v1.x, dur),
                    y: CubicPolynomial::new(p0.y, p1.y, v0.y, v1.y, dur),
                    z: CubicPolynomial::new(p0.z, p1.z, v0.z, v1.z, dur),
                    duration: dur,
                },
                PolynomialType::Quintic => Segment3D::Quintic {
                    x: QuinticPolynomial::new(p0.x, p1.x, v0.x, v1.x, a0.x, a1.x, dur),
                    y: QuinticPolynomial::new(p0.y, p1.y, v0.y, v1.y, a0.y, a1.y, dur),
                    z: QuinticPolynomial::new(p0.z, p1.z, v0.z, v1.z, a0.z, a1.z, dur),
                    duration: dur,
                },
            };

            self.segments.push(segment);
            accum_time += dur;
            self.cumulative_times.push(accum_time);
        }

        self.total_duration = accum_time;
    }

    /// Fast O(log S) binary-search segment index lookup.
    fn find_segment_index(&self, t: f64) -> usize {
        let num_segs = self.segments.len();
        if num_segs <= 1 {
            return 0;
        }
        let partition = self.cumulative_times.partition_point(|&cum| cum <= t);
        partition.saturating_sub(1).min(num_segs - 1)
    }

    /// Evaluates the target position at a given time along the trajectory timeline using O(log S) binary search.
    pub fn evaluate(&self, t: f64) -> Option<Point3<f64>> {
        if self.segments.is_empty() {
            return self.waypoints.first().map(|wp| wp.position);
        }

        let clamped_t = t.clamp(0.0, self.total_duration);
        let idx = self.find_segment_index(clamped_t);
        let seg = &self.segments[idx];
        let local_t = (clamped_t - self.cumulative_times[idx]).clamp(0.0, seg.duration());
        Some(seg.evaluate(local_t))
    }

    /// Evaluates full kinematic state (Position, Velocity, Acceleration) along the trajectory timeline.
    pub fn evaluate_state(&self, t: f64) -> Option<(Point3<f64>, Vector3<f64>, Vector3<f64>)> {
        if self.segments.is_empty() {
            return self
                .waypoints
                .first()
                .map(|wp| (wp.position, Vector3::zeros(), Vector3::zeros()));
        }

        let clamped_t = t.clamp(0.0, self.total_duration);
        let idx = self.find_segment_index(clamped_t);
        let seg = &self.segments[idx];
        let local_t = (clamped_t - self.cumulative_times[idx]).clamp(0.0, seg.duration());
        Some((
            seg.evaluate(local_t),
            seg.evaluate_velocity(local_t),
            seg.evaluate_acceleration(local_t),
        ))
    }

    /// Samples the continuous path curve into a polyline for 3D visualization.
    pub fn sample_path(&self, points_per_segment: usize) -> Vec<Point3<f64>> {
        let mut samples = Vec::new();
        if self.segments.is_empty() {
            return samples;
        }

        for seg in &self.segments {
            let dur = seg.duration();
            let steps = points_per_segment.max(2);
            for step in 0..=steps {
                let tau = (step as f64 / steps as f64) * dur;
                samples.push(seg.evaluate(tau));
            }
        }

        samples
    }
}
