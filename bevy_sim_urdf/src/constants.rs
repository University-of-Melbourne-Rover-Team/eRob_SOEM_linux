//! Shared tuning constants for controls, defaults, and DLS IK behavior.

use bevy::prelude::KeyCode;

pub(crate) const DEFAULT_TRIANGLE_CAP: usize = 25_000;
pub(crate) const JOINT_SPEED: f32 = 1.2;
pub(crate) const TARGET_MOVE_SPEED: f32 = 0.25;
pub(crate) const TARGET_ROTATE_SPEED: f32 = 1.2;
pub(crate) const TASK_TARGET_LINK: &str = "link6";
pub(crate) const DEFAULT_HOME_JOINTS: [f64; 6] = [-0.6, 2.5, 1.5, -1.2, -1.4, 2.0];
pub(crate) const DEFAULT_LIMIT: f32 = std::f32::consts::PI;

pub(crate) const DLS_ALLOWABLE_TARGET_DISTANCE: f64 = 0.001;
pub(crate) const DLS_ALLOWABLE_TARGET_ANGLE: f64 = 0.005;
pub(crate) const DLS_BASE_DAMPING: f64 = 0.02;
pub(crate) const DLS_SINGULARITY_THRESHOLD: f64 = 0.05;
pub(crate) const DLS_SINGULARITY_DAMPING: f64 = 0.25;
pub(crate) const DLS_STEP_SCALE: f64 = 0.8;
pub(crate) const DLS_MAX_JOINT_STEP: f64 = 0.12;
pub(crate) const DLS_MAX_TRIES: usize = 32;

pub(crate) const KEY_PAIRS: [(KeyCode, KeyCode); 8] = [
    (KeyCode::KeyQ, KeyCode::KeyA),
    (KeyCode::KeyW, KeyCode::KeyS),
    (KeyCode::KeyE, KeyCode::KeyD),
    (KeyCode::KeyR, KeyCode::KeyF),
    (KeyCode::KeyT, KeyCode::KeyG),
    (KeyCode::KeyY, KeyCode::KeyH),
    (KeyCode::KeyU, KeyCode::KeyJ),
    (KeyCode::KeyI, KeyCode::KeyK),
];
