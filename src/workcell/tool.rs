use serde::{Deserialize, Serialize};

/// Type of End-Of-Arm Tooling (EOAT) attached to the robot wrist flange.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum EOATType {
    /// 2-Finger Parallel Mechanical Gripper (e.g. Robotiq 2F-85).
    #[default]
    ParallelGripper,
    /// Pneumatic Suction Bellows Cup (e.g. Piab/Schunk vacuum cup).
    VacuumCup,
    /// Robotic Industrial MIG/TIG Welding Torch with electric arc.
    WeldingTorch,
}

/// Dynamic active state of the robotic end-effector tool.
#[derive(Debug, Clone)]
pub struct ToolState {
    pub tool_type: EOATType,
    /// Opening ratio for mechanical jaws: 0.0 (fully closed) to 1.0 (fully open).
    pub gripper_opening: f32,
    /// Whether pneumatic vacuum suction is active.
    pub is_vacuum_active: bool,
    /// Whether electric arc welding power is active.
    pub is_welding_active: bool,
}

impl Default for ToolState {
    fn default() -> Self {
        Self {
            tool_type: EOATType::ParallelGripper,
            gripper_opening: 0.75, // Default comfortably open
            is_vacuum_active: false,
            is_welding_active: false,
        }
    }
}

impl ToolState {
    /// Toggles the mechanical gripper between open and closed.
    pub fn toggle_gripper(&mut self) {
        if self.gripper_opening > 0.4 {
            self.gripper_opening = 0.05; // Close
        } else {
            self.gripper_opening = 0.85; // Open
        }
    }

    /// Toggles pneumatic vacuum suction.
    pub fn toggle_vacuum(&mut self) {
        self.is_vacuum_active = !self.is_vacuum_active;
    }

    /// Toggles electric welding arc.
    pub fn toggle_welding(&mut self) {
        self.is_welding_active = !self.is_welding_active;
    }

    /// Determines whether the active tool is currently exerting a grasping force.
    pub fn is_gripping(&self) -> bool {
        match self.tool_type {
            EOATType::ParallelGripper => self.gripper_opening < 0.35,
            EOATType::VacuumCup => self.is_vacuum_active,
            EOATType::WeldingTorch => false,
        }
    }
}
