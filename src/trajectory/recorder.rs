use nalgebra::Point3;
use serde::{Deserialize, Serialize};
use web_time::Instant;

use super::planner::Waypoint;

/// A timestamped state frame in a recorded robotic trajectory session.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrajectoryFrame {
    /// Relative time in seconds from recording start.
    pub time: f64,
    /// Cartesian Tool Center Point position (meters).
    pub ee_position: Point3<f64>,
    /// Tool orientation in Euler angles [Roll, Pitch, Yaw] in degrees.
    pub ee_rpy_deg: [f64; 3],
    /// Actuated joint positions (radians or meters).
    pub joint_positions: Vec<f64>,
    /// Gripper jaw opening factor (0.0 = fully closed, 1.0 = fully open).
    pub gripper_value: f32,
}

/// A serialized robotic motion recording session.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TrajectoryRecording {
    pub name: String,
    pub created_at: String,
    pub total_duration: f64,
    pub frames: Vec<TrajectoryFrame>,
}

impl TrajectoryRecording {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            created_at: "Session".to_string(),
            total_duration: 0.0,
            frames: Vec::new(),
        }
    }

    /// Total number of recorded frames.
    pub fn frame_count(&self) -> usize {
        self.frames.len()
    }

    /// Whether the recording is empty.
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// Total 3D distance traversed by the end-effector in meters.
    pub fn total_path_distance(&self) -> f64 {
        let mut dist = 0.0;
        for i in 0..self.frames.len().saturating_sub(1) {
            dist += (self.frames[i + 1].ee_position - self.frames[i].ee_position).norm();
        }
        dist
    }

    /// Interpolates the robotic state at arbitrary time `t` (0.0 <= t <= total_duration).
    pub fn sample_at_time(&self, t: f64) -> Option<TrajectoryFrame> {
        if self.frames.is_empty() {
            return None;
        }
        if self.frames.len() == 1 || t <= self.frames[0].time {
            return Some(self.frames[0].clone());
        }
        if t >= self.frames.last().unwrap().time {
            return Some(self.frames.last().unwrap().clone());
        }

        // Binary search for surrounding frames
        let mut low = 0;
        let mut high = self.frames.len() - 1;

        while low + 1 < high {
            let mid = (low + high) / 2;
            if self.frames[mid].time <= t {
                low = mid;
            } else {
                high = mid;
            }
        }

        let f0 = &self.frames[low];
        let f1 = &self.frames[high];
        let dt = (f1.time - f0.time).max(1e-6);
        let u = ((t - f0.time) / dt).clamp(0.0, 1.0);

        // Linear interpolation for Cartesian position
        let p_interp = f0.ee_position + (f1.ee_position - f0.ee_position) * u;

        // Linear interpolation for RPY orientation
        let mut rpy_interp = [0.0; 3];
        for (k, val) in rpy_interp.iter_mut().enumerate() {
            *val = f0.ee_rpy_deg[k] + (f1.ee_rpy_deg[k] - f0.ee_rpy_deg[k]) * u;
        }

        // Interpolation for joints
        let mut q_interp = Vec::with_capacity(f0.joint_positions.len());
        for (q0, q1) in f0.joint_positions.iter().zip(f1.joint_positions.iter()) {
            q_interp.push(q0 + (q1 - q0) * u);
        }

        // Gripper value
        let grip_interp = f0.gripper_value + (f1.gripper_value - f0.gripper_value) * (u as f32);

        Some(TrajectoryFrame {
            time: t,
            ee_position: p_interp,
            ee_rpy_deg: rpy_interp,
            joint_positions: q_interp,
            gripper_value: grip_interp,
        })
    }

    /// Automatically downsamples recorded frames into smooth keyframe `Waypoint`s.
    pub fn to_waypoints(&self, max_waypoints: usize) -> Vec<Waypoint> {
        let n = self.frames.len();
        if n == 0 {
            return Vec::new();
        }
        if n <= max_waypoints || max_waypoints < 2 {
            return self
                .frames
                .iter()
                .enumerate()
                .map(|(i, f)| {
                    let dur = if i == 0 {
                        1.5
                    } else {
                        (f.time - self.frames[i - 1].time).max(0.1)
                    };
                    Waypoint::new(format!("Rec WP {}", i + 1), f.ee_position, dur)
                })
                .collect();
        }

        let mut waypoints = Vec::with_capacity(max_waypoints);
        let step = (n - 1) as f64 / (max_waypoints - 1) as f64;

        for k in 0..max_waypoints {
            let idx = ((k as f64 * step).round() as usize).min(n - 1);
            let frame = &self.frames[idx];
            let dur = if k == 0 {
                1.5
            } else {
                let prev_idx = (((k - 1) as f64 * step).round() as usize).min(n - 1);
                (frame.time - self.frames[prev_idx].time).max(0.1)
            };

            waypoints.push(Waypoint::new(
                format!("Rec WP {}", k + 1),
                frame.ee_position,
                dur,
            ));
        }

        waypoints
    }
}

/// Interactive live trajectory recorder and playback engine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrajectoryRecorder {
    pub recording: TrajectoryRecording,
    pub is_recording: bool,
    #[serde(skip)]
    pub start_instant: Option<Instant>,
    pub sample_interval_secs: f64,
    pub last_sample_time: f64,
    pub is_replaying: bool,
    pub replay_time: f64,
    pub replay_speed: f64,
    pub replay_loop: bool,
}

impl Default for TrajectoryRecorder {
    fn default() -> Self {
        Self {
            recording: TrajectoryRecording::new("Default Recording"),
            is_recording: false,
            start_instant: None,
            sample_interval_secs: 1.0 / 30.0, // 30 FPS default capture rate
            last_sample_time: -1.0,
            is_replaying: false,
            replay_time: 0.0,
            replay_speed: 1.0,
            replay_loop: true,
        }
    }
}

impl TrajectoryRecorder {
    /// Starts live recording of robot movements.
    pub fn start_recording(&mut self, name: Option<String>) {
        if let Some(n) = name {
            self.recording = TrajectoryRecording::new(n);
        } else {
            self.recording = TrajectoryRecording::new("Live Session");
        }
        self.is_recording = true;
        self.start_instant = Some(Instant::now());
        self.last_sample_time = -1.0;
        self.is_replaying = false;
    }

    /// Stops recording and computes final duration.
    pub fn stop_recording(&mut self) {
        self.is_recording = false;
        self.start_instant = None;
        if let Some(last) = self.recording.frames.last() {
            self.recording.total_duration = last.time;
        }
    }

    /// Captures a frame if recording is active and the sample interval has elapsed.
    pub fn maybe_record_frame(
        &mut self,
        ee_position: Point3<f64>,
        ee_rpy_deg: [f64; 3],
        joint_positions: &[f64],
        gripper_value: f32,
    ) {
        if !self.is_recording {
            return;
        }

        let elapsed = if let Some(t0) = self.start_instant {
            t0.elapsed().as_secs_f64()
        } else {
            return;
        };

        if elapsed - self.last_sample_time >= self.sample_interval_secs {
            self.recording.frames.push(TrajectoryFrame {
                time: elapsed,
                ee_position,
                ee_rpy_deg,
                joint_positions: joint_positions.to_vec(),
                gripper_value,
            });
            self.last_sample_time = elapsed;
            self.recording.total_duration = elapsed;
        }
    }

    /// Toggles playback of the recorded trajectory.
    pub fn toggle_replay(&mut self) {
        if self.recording.frames.is_empty() {
            return;
        }
        self.is_replaying = !self.is_replaying;
        if self.is_replaying && self.replay_time >= self.recording.total_duration {
            self.replay_time = 0.0;
        }
    }

    /// Stops replay and rewinds timeline.
    pub fn stop_replay(&mut self) {
        self.is_replaying = false;
        self.replay_time = 0.0;
    }

    /// Steps the replay timeline forward by `dt` seconds and returns the interpolated frame.
    pub fn update_replay(&mut self, dt: f64) -> Option<TrajectoryFrame> {
        if !self.is_replaying || self.recording.frames.is_empty() {
            return None;
        }

        self.replay_time += dt * self.replay_speed;

        if self.replay_time > self.recording.total_duration {
            if self.replay_loop && self.recording.total_duration > 0.05 {
                self.replay_time %= self.recording.total_duration;
            } else {
                self.replay_time = self.recording.total_duration;
                self.is_replaying = false;
            }
        }

        self.recording.sample_at_time(self.replay_time)
    }

    /// Clears all recorded frames.
    pub fn clear(&mut self) {
        self.recording.frames.clear();
        self.recording.total_duration = 0.0;
        self.is_recording = false;
        self.is_replaying = false;
        self.replay_time = 0.0;
    }
}
