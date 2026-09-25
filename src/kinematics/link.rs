use serde::{Deserialize, Serialize};

/// Visual shape geometry for a robot link.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LinkGeometry {
    Cylinder { radius: f64, length: f64 },
    Box { size: [f64; 3] },
    Sphere { radius: f64 },
    Mesh { filename: String, scale: [f64; 3] },
}

/// A rigid link connecting adjacent joints in a robot manipulator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Link {
    pub name: String,
    pub visual_geometry: Option<LinkGeometry>,
    pub color: [f32; 4],
    pub mass: f64,
}

impl Link {
    pub fn new(name: impl Into<String>, color: [f32; 4]) -> Self {
        Self {
            name: name.into(),
            visual_geometry: None,
            color,
            mass: 1.0,
        }
    }

    pub fn with_cylinder(mut self, radius: f64, length: f64) -> Self {
        self.visual_geometry = Some(LinkGeometry::Cylinder { radius, length });
        self
    }

    pub fn with_box(mut self, size: [f64; 3]) -> Self {
        self.visual_geometry = Some(LinkGeometry::Box { size });
        self
    }

    pub fn with_sphere(mut self, radius: f64) -> Self {
        self.visual_geometry = Some(LinkGeometry::Sphere { radius });
        self
    }
}
