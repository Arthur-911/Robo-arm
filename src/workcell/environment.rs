use nalgebra::{Point3, Vector3};

use crate::kinematics::ObstacleBox;

/// Environmental equipment and physical workcell furnishings.
#[derive(Debug, Clone)]
pub struct WorkcellEnvironment {
    pub show_table: bool,
    pub show_pedestal: bool,
    pub show_safety_enclosure: bool,
    /// Center of the assembly / inspection workbench.
    pub table_center: Point3<f64>,
    /// Half-dimensions (half-width-X, half-depth-Y, half-height-Z) of the work table.
    pub table_half_size: Vector3<f32>,
    /// Physical obstacle barriers for collision checking and avoidance.
    pub obstacles: Vec<ObstacleBox>,
}

impl Default for WorkcellEnvironment {
    fn default() -> Self {
        let table_center = Point3::new(0.45, 0.0, -0.02);
        let table_half_size = Vector3::new(0.35, 0.45, 0.02);

        let mut env = Self {
            show_table: true,
            show_pedestal: true,
            show_safety_enclosure: false,
            table_center,
            table_half_size,
            obstacles: Vec::new(),
        };
        env.reset_obstacles();
        env
    }
}

impl WorkcellEnvironment {
    /// Resets obstacle objects for collision evaluation.
    pub fn reset_obstacles(&mut self) {
        self.obstacles.clear();

        // 1. Workcell Table Top obstacle (protects table surface from collision)
        self.obstacles.push(ObstacleBox::new(
            "Assembly Workstation Table",
            self.table_center,
            Vector3::new(
                self.table_half_size.x as f64,
                self.table_half_size.y as f64,
                self.table_half_size.z as f64,
            ),
        ));

        // 2. Safety Fixture Barrier Pillar
        self.obstacles.push(ObstacleBox::new(
            "Inspection Camera Pillar",
            Point3::new(-0.25, 0.40, 0.25),
            Vector3::new(0.04, 0.04, 0.25),
        ));
    }
}
