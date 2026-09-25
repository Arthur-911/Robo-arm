use nalgebra::Point3;
use serde::{Deserialize, Serialize};

use super::planner::TrajectoryPlanner;

/// Real-time joint trajectory smoother providing exponential moving average (EMA)
/// and velocity-rate limiting for buttery-smooth interactive tracking.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointTrajectorySmoother {
    pub smoothed_q: Vec<f64>,
    /// Maximum joint speed in rad/s (or m/s). Default: 3.5.
    pub max_joint_velocity: f64,
    /// Low-pass filter smoothing coefficient (0.0 < alpha <= 1.0). Default: 0.4.
    pub smoothing_alpha: f64,
}

impl Default for JointTrajectorySmoother {
    fn default() -> Self {
        Self {
            smoothed_q: Vec::new(),
            max_joint_velocity: 3.5,
            smoothing_alpha: 0.4,
        }
    }
}

impl JointTrajectorySmoother {
    pub fn new(max_velocity: f64, alpha: f64) -> Self {
        Self {
            smoothed_q: Vec::new(),
            max_joint_velocity: max_velocity.max(0.1),
            smoothing_alpha: alpha.clamp(0.01, 1.0),
        }
    }

    pub fn reset(&mut self, q: &[f64]) {
        self.smoothed_q = q.to_vec();
    }

    /// Filters raw target joint positions towards the goal with velocity clamping and smoothing.
    pub fn filter(&mut self, target_q: &[f64], dt: f64) -> Vec<f64> {
        if self.smoothed_q.len() != target_q.len() {
            self.smoothed_q = target_q.to_vec();
            return self.smoothed_q.clone();
        }

        let dt_clamped = dt.clamp(1e-4, 0.1);
        let max_delta = self.max_joint_velocity * dt_clamped;

        for (i, &tq) in target_q.iter().enumerate() {
            let diff = tq - self.smoothed_q[i];
            let filtered_step = diff * self.smoothing_alpha;
            let clamped_step = filtered_step.clamp(-max_delta, max_delta);
            self.smoothed_q[i] += clamped_step;
        }

        self.smoothed_q.clone()
    }
}

/// Manages real-time playback, scrubbing, and trail tracing for robotic trajectories.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrajectoryController {
    pub planner: TrajectoryPlanner,
    pub is_playing: bool,
    pub current_time: f64,
    pub speed_multiplier: f64,
    pub is_looping: bool,
    #[serde(skip)]
    pub trail: Vec<Point3<f64>>,
    pub max_trail_len: usize,
    #[serde(default)]
    pub smoother: JointTrajectorySmoother,
    #[serde(default = "default_true")]
    pub enable_smoothing: bool,
}

fn default_true() -> bool {
    true
}

impl Default for TrajectoryController {
    fn default() -> Self {
        Self {
            planner: TrajectoryPlanner::default(),
            is_playing: false,
            current_time: 0.0,
            speed_multiplier: 1.0,
            is_looping: true,
            trail: Vec::new(),
            max_trail_len: 400,
            smoother: JointTrajectorySmoother::default(),
            enable_smoothing: true,
        }
    }
}

impl TrajectoryController {
    pub fn play(&mut self) {
        self.is_playing = true;
    }

    pub fn pause(&mut self) {
        self.is_playing = false;
    }

    pub fn toggle_play(&mut self) {
        self.is_playing = !self.is_playing;
    }

    pub fn reset(&mut self) {
        self.current_time = 0.0;
        self.is_playing = false;
        self.clear_trail();
    }

    pub fn seek(&mut self, t: f64) {
        self.current_time = t.clamp(0.0, self.planner.total_duration);
    }

    pub fn clear_trail(&mut self) {
        self.trail.clear();
    }

    /// Amortized O(1) trail point addition, avoiding expensive per-frame vector shifts.
    pub fn add_trail_point(&mut self, pt: Point3<f64>) {
        if let Some(last) = self.trail.last() {
            if (pt - *last).norm() < 1e-4 {
                return;
            }
        }
        self.trail.push(pt);
        // Batch drain excess to amortize memory shift overhead
        if self.trail.len() > self.max_trail_len + 32 {
            let excess = self.trail.len() - self.max_trail_len;
            self.trail.drain(0..excess);
        }
    }

    /// Advances the trajectory clock by `dt` seconds and evaluates target position.
    pub fn update(&mut self, dt: f64) -> Option<Point3<f64>> {
        if self.is_playing && self.planner.total_duration > 0.0 {
            self.current_time += dt * self.speed_multiplier;
            if self.current_time > self.planner.total_duration {
                if self.is_looping {
                    self.current_time %= self.planner.total_duration;
                } else {
                    self.current_time = self.planner.total_duration;
                    self.is_playing = false;
                }
            }
        }

        self.planner.evaluate(self.current_time)
    }
}
