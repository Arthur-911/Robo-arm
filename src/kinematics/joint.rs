use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

use crate::math::make_isometry;

/// Type of robotic joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum JointType {
    /// Rotational joint with finite limits.
    Revolute,
    /// Rotational joint with continuous rotation (-infinity to +infinity).
    Continuous,
    /// Linear sliding joint with finite limits.
    Prismatic,
    /// Rigidly fixed connection.
    Fixed,
}

/// A robotic joint connecting two links.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Joint {
    pub name: String,
    pub joint_type: JointType,
    pub origin_translation: Vector3<f64>,
    pub origin_rpy: Vector3<f64>,
    pub axis: Unit<Vector3<f64>>,
    pub limits: Option<(f64, f64)>,
    pub current_position: f64,
    pub home_position: f64,
    #[serde(default = "Isometry3::identity")]
    pub origin_isometry: Isometry3<f64>,
}

impl Joint {
    /// Creates a new revolute joint with limits in radians.
    pub fn new_revolute(
        name: impl Into<String>,
        origin_translation: Vector3<f64>,
        origin_rpy: Vector3<f64>,
        axis: Vector3<f64>,
        limits: Option<(f64, f64)>,
    ) -> Self {
        let normalized_axis = Unit::new_normalize(axis);
        let origin_isometry = make_isometry(origin_translation, origin_rpy);
        Self {
            name: name.into(),
            joint_type: JointType::Revolute,
            origin_translation,
            origin_rpy,
            axis: normalized_axis,
            limits,
            current_position: 0.0,
            home_position: 0.0,
            origin_isometry,
        }
    }

    /// Creates a new prismatic (linear) joint with limits in meters.
    pub fn new_prismatic(
        name: impl Into<String>,
        origin_translation: Vector3<f64>,
        origin_rpy: Vector3<f64>,
        axis: Vector3<f64>,
        limits: Option<(f64, f64)>,
    ) -> Self {
        let normalized_axis = Unit::new_normalize(axis);
        let origin_isometry = make_isometry(origin_translation, origin_rpy);
        Self {
            name: name.into(),
            joint_type: JointType::Prismatic,
            origin_translation,
            origin_rpy,
            axis: normalized_axis,
            limits,
            current_position: 0.0,
            home_position: 0.0,
            origin_isometry,
        }
    }

    /// Creates a fixed joint.
    pub fn new_fixed(
        name: impl Into<String>,
        origin_translation: Vector3<f64>,
        origin_rpy: Vector3<f64>,
    ) -> Self {
        let origin_isometry = make_isometry(origin_translation, origin_rpy);
        Self {
            name: name.into(),
            joint_type: JointType::Fixed,
            origin_translation,
            origin_rpy,
            axis: Unit::new_normalize(Vector3::z()),
            limits: None,
            current_position: 0.0,
            home_position: 0.0,
            origin_isometry,
        }
    }

    /// Updates origin translation and RPY, refreshing the precalculated transform cache.
    pub fn set_origin(&mut self, translation: Vector3<f64>, rpy: Vector3<f64>) {
        self.origin_translation = translation;
        self.origin_rpy = rpy;
        self.origin_isometry = make_isometry(translation, rpy);
    }

    /// Returns the precomputed spatial transform of the joint's attachment frame.
    #[inline]
    pub fn origin_isometry(&self) -> Isometry3<f64> {
        if self.origin_isometry == Isometry3::identity()
            && (self.origin_translation != Vector3::zeros() || self.origin_rpy != Vector3::zeros())
        {
            make_isometry(self.origin_translation, self.origin_rpy)
        } else {
            self.origin_isometry
        }
    }

    /// Computes the relative spatial transformation induced by this joint at angle/displacement `q`.
    #[inline]
    pub fn local_transform(&self, q: f64) -> Isometry3<f64> {
        let origin_iso = self.origin_isometry();

        let motion_iso = match self.joint_type {
            JointType::Revolute | JointType::Continuous => {
                let rot = UnitQuaternion::from_axis_angle(&self.axis, q);
                Isometry3::from_parts(Translation3::identity(), rot)
            }
            JointType::Prismatic => {
                let trans = Translation3::from(self.axis.into_inner() * q);
                Isometry3::from_parts(trans, UnitQuaternion::identity())
            }
            JointType::Fixed => Isometry3::identity(),
        };

        origin_iso * motion_iso
    }

    /// Clamps a candidate joint position within defined min/max limits.
    pub fn clamp_position(&self, val: f64) -> f64 {
        if let Some((min, max)) = self.limits {
            val.clamp(min, max)
        } else {
            val
        }
    }

    /// Sets the joint position, automatically clamping to limits.
    pub fn set_position(&mut self, val: f64) {
        self.current_position = self.clamp_position(val);
    }

    /// Checks if this joint is an active degree of freedom (i.e. not fixed).
    pub fn is_actuated(&self) -> bool {
        self.joint_type != JointType::Fixed
    }
}
