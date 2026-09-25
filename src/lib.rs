//! # `kine-rs`: High-Performance 2D/3D Robotic Arm Kinematics & IK Simulator
//!
//! A modular, production-ready robotics library and simulator written in pure Rust.
//! Features forward kinematics, spatial Jacobians, Damped Least Squares (DLS) IK,
//! FABRIK with kinematic constraints, quintic polynomial trajectory planning,
//! URDF import/export, and interactive GUI powered by `eframe` (`egui`).

pub mod export;
pub mod kinematics;
pub mod math;
pub mod presets;
pub mod trajectory;
pub mod urdf;
pub mod workcell;

pub mod ui;

#[cfg(target_arch = "wasm32")]
pub mod web;
