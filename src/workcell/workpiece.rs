use egui::Color32;
use nalgebra::{Point3, UnitQuaternion, Vector3};

/// Shape geometry of a manipulable workpiece.
#[derive(Debug, Clone)]
pub enum WorkpieceShape {
    Box { half_size: Vector3<f32> },
    Cylinder { radius: f32, height: f32 },
    Sphere { radius: f32 },
}

/// A physical object in the workcell that can be grasped, manipulated, and placed.
#[derive(Debug, Clone)]
pub struct Workpiece {
    pub id: usize,
    pub name: String,
    pub shape: WorkpieceShape,
    pub position: Point3<f64>,
    pub rotation: UnitQuaternion<f64>,
    pub color: Color32,
    pub is_grasped: bool,
    pub grasp_offset: Vector3<f64>,
}

impl Workpiece {
    pub fn new_box(
        id: usize,
        name: impl Into<String>,
        position: Point3<f64>,
        half_size: Vector3<f32>,
        color: Color32,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            shape: WorkpieceShape::Box { half_size },
            position,
            rotation: UnitQuaternion::identity(),
            color,
            is_grasped: false,
            grasp_offset: Vector3::zeros(),
        }
    }

    pub fn new_cylinder(
        id: usize,
        name: impl Into<String>,
        position: Point3<f64>,
        radius: f32,
        height: f32,
        color: Color32,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            shape: WorkpieceShape::Cylinder { radius, height },
            position,
            rotation: UnitQuaternion::identity(),
            color,
            is_grasped: false,
            grasp_offset: Vector3::zeros(),
        }
    }

    pub fn new_sphere(
        id: usize,
        name: impl Into<String>,
        position: Point3<f64>,
        radius: f32,
        color: Color32,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            shape: WorkpieceShape::Sphere { radius },
            position,
            rotation: UnitQuaternion::identity(),
            color,
            is_grasped: false,
            grasp_offset: Vector3::zeros(),
        }
    }

    /// Returns the vertical ground resting offset for this shape.
    pub fn ground_resting_z(&self) -> f64 {
        match self.shape {
            WorkpieceShape::Box { half_size } => half_size.z as f64,
            WorkpieceShape::Cylinder { height, .. } => (height * 0.5) as f64,
            WorkpieceShape::Sphere { radius } => radius as f64,
        }
    }
}

/// Manages a collection of interactive workpieces in the workcell.
#[derive(Debug, Clone)]
pub struct WorkpieceManager {
    pub workpieces: Vec<Workpiece>,
    pub next_id: usize,
    pub currently_held_id: Option<usize>,
}

impl Default for WorkpieceManager {
    fn default() -> Self {
        let mut mgr = Self {
            workpieces: Vec::new(),
            next_id: 1,
            currently_held_id: None,
        };
        mgr.reset_demo_workpieces();
        mgr
    }
}

impl WorkpieceManager {
    /// Spawns a set of industrial workpieces ready for pick-and-place tasks.
    pub fn reset_demo_workpieces(&mut self) {
        self.workpieces.clear();
        self.currently_held_id = None;
        self.next_id = 1;

        // Red Industrial Cube
        self.workpieces.push(Workpiece::new_box(
            self.next_id,
            "Machined Red Block",
            Point3::new(0.40, 0.20, 0.03),
            Vector3::new(0.03, 0.03, 0.03),
            Color32::from_rgb(235, 60, 50),
        ));
        self.next_id += 1;

        // Precision Blue Billet (Cylinder)
        self.workpieces.push(Workpiece::new_cylinder(
            self.next_id,
            "Aluminum Blue Billet",
            Point3::new(0.42, -0.18, 0.04),
            0.028,
            0.08,
            Color32::from_rgb(45, 140, 240),
        ));
        self.next_id += 1;

        // Golden Sphere
        self.workpieces.push(Workpiece::new_sphere(
            self.next_id,
            "Brass Golden Sphere",
            Point3::new(0.32, -0.30, 0.03),
            0.03,
            Color32::from_rgb(240, 190, 40),
        ));
        self.next_id += 1;
    }

    /// Evaluates grasp interactions and updates positions of held objects.
    pub fn update(&mut self, tcp_pos: Point3<f64>, tcp_rot: UnitQuaternion<f64>, is_gripping: bool) {
        if is_gripping {
            if self.currently_held_id.is_none() {
                // Try grasping closest workpiece within contact threshold
                let grasp_threshold = 0.075; // 7.5 cm contact zone
                let mut closest_idx = None;
                let mut min_dist = grasp_threshold;

                for (idx, wp) in self.workpieces.iter().enumerate() {
                    let dist = (wp.position - tcp_pos).norm();
                    if dist < min_dist {
                        min_dist = dist;
                        closest_idx = Some(idx);
                    }
                }

                if let Some(idx) = closest_idx {
                    let wp = &mut self.workpieces[idx];
                    wp.is_grasped = true;
                    // Store relative offset in TCP local coordinates
                    wp.grasp_offset = tcp_rot.inverse() * (wp.position - tcp_pos);
                    self.currently_held_id = Some(wp.id);
                }
            }

            // Update held workpiece transform to follow gripper TCP
            if let Some(held_id) = self.currently_held_id {
                if let Some(wp) = self.workpieces.iter_mut().find(|w| w.id == held_id) {
                    wp.position = tcp_pos + tcp_rot * wp.grasp_offset;
                    wp.rotation = tcp_rot;
                }
            }
        } else {
            // Gripper opened / released
            if let Some(held_id) = self.currently_held_id {
                if let Some(wp) = self.workpieces.iter_mut().find(|w| w.id == held_id) {
                    wp.is_grasped = false;
                    // Settle onto ground plane
                    let ground_z = wp.ground_resting_z();
                    if wp.position.z < ground_z {
                        wp.position.z = ground_z;
                    }
                }
                self.currently_held_id = None;
            }
        }
    }
}
