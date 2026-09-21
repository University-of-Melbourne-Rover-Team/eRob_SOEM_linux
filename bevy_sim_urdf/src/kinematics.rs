//! Kinematics helpers and the Jacobian damped least-squares IK implementation.

use bevy::prelude::{Resource, Vec3};
use k::nalgebra::{DMatrix, DVector, Isometry3, UnitQuaternion, Vector6};
use std::collections::HashMap;
use std::path::Path;

use crate::constants::{
    DLS_ALLOWABLE_TARGET_ANGLE, DLS_ALLOWABLE_TARGET_DISTANCE, DLS_BASE_DAMPING,
    DLS_MAX_JOINT_STEP, DLS_MAX_TRIES, DLS_SINGULARITY_DAMPING, DLS_SINGULARITY_THRESHOLD,
    DLS_STEP_SCALE,
};

#[derive(Resource)]
pub(crate) struct KinematicsState {
    pub(crate) chain: k::Chain<f64>,
    pub(crate) joint_names: Vec<String>,
}

/// Loads a kinematic chain from the URDF and records movable joint ordering.
pub(crate) fn create_kinematics_state(urdf_path: &Path) -> Result<KinematicsState, String> {
    let chain = k::Chain::<f64>::from_urdf_file(urdf_path)
        .map_err(|error| format!("failed to load URDF with k: {error}"))?;
    let joint_names = chain
        .iter_joints()
        .map(|joint| joint.name.clone())
        .collect::<Vec<_>>();
    Ok(KinematicsState { chain, joint_names })
}

/// Computes the world-space position of a link after applying joint values.
pub(crate) fn link_position_from_joint_values(
    kinematics: &mut KinematicsState,
    joint_values: &HashMap<String, f32>,
    target_link: &str,
) -> Result<Vec3, String> {
    let positions = kinematics
        .joint_names
        .iter()
        .map(|name| joint_values.get(name).copied().unwrap_or(0.0) as f64)
        .collect::<Vec<_>>();
    kinematics
        .chain
        .set_joint_positions(&positions)
        .map_err(|error| format!("failed to set joint positions: {error}"))?;
    kinematics.chain.update_transforms();

    let target_node = kinematics
        .chain
        .find_link(target_link)
        .ok_or_else(|| format!("target link '{target_link}' was not found in the URDF"))?;
    let target_transform = target_node
        .world_transform()
        .ok_or_else(|| format!("target link '{target_link}' has no world transform"))?;
    let translation = target_transform.translation.vector;
    Ok(Vec3::new(
        translation.x as f32,
        translation.y as f32,
        translation.z as f32,
    ))
}

/// Solves IK from current joint values and returns the resulting joint map.
pub(crate) fn solve_task_space_ik_values(
    kinematics: &mut KinematicsState,
    current_values: &HashMap<String, f32>,
    target_link: &str,
    target_xyz: Vec3,
    target_rpy: Option<Vec3>,
) -> Result<HashMap<String, f32>, String> {
    let positions = kinematics
        .joint_names
        .iter()
        .map(|name| current_values.get(name).copied().unwrap_or(0.0) as f64)
        .collect::<Vec<_>>();

    kinematics
        .chain
        .set_joint_positions(&positions)
        .map_err(|error| format!("failed to set joint positions: {error}"))?;
    kinematics.chain.update_transforms();
    let solved = solve_task_space_ik(&kinematics.chain, target_link, target_xyz, target_rpy);

    let values = kinematics
        .joint_names
        .iter()
        .cloned()
        .zip(
            kinematics
                .chain
                .joint_positions()
                .into_iter()
                .map(|value| value as f32),
        )
        .collect();
    kinematics.chain.set_joint_positions(&positions)
        .map_err(|error| format!("failed to restore measured joint positions: {error}"))?;
    kinematics.chain.update_transforms();
    solved?;
    Ok(values)
}

/// Moves the kinematic chain toward a task-space target using DLS IK.
pub(crate) fn solve_task_space_ik(
    chain: &k::Chain<f64>,
    target_link: &str,
    target_xyz: Vec3,
    target_rpy: Option<Vec3>,
) -> Result<(), String> {
    let target_node = chain
        .find_link(target_link)
        .ok_or_else(|| format!("target link '{target_link}' was not found in the URDF"))?;
    let mut target_transform = target_node
        .world_transform()
        .ok_or_else(|| format!("target link '{target_link}' has no world transform"))?;
    target_transform.translation.vector.x = target_xyz.x as f64;
    target_transform.translation.vector.y = target_xyz.y as f64;
    target_transform.translation.vector.z = target_xyz.z as f64;
    if let Some(target_rpy) = target_rpy {
        target_transform.rotation = rpy_quat_f64(target_rpy);
    }

    let arm = k::SerialChain::from_end(target_node);
    solve_damped_least_squares_ik(&arm, &target_transform, target_rpy.is_some())?;
    chain.update_transforms();
    Ok(())
}

/// Iteratively applies damped least-squares corrections until the target is reached.
fn solve_damped_least_squares_ik(
    arm: &k::SerialChain<f64>,
    target_transform: &Isometry3<f64>,
    constrain_orientation: bool,
) -> Result<(), String> {
    let original_positions = arm.joint_positions();
    let operational_space = if constrain_orientation {
        [true, true, true, true, true, true]
    } else {
        [true, true, true, false, false, false]
    };

    let mut last_position_error = f64::INFINITY;
    let mut last_rotation_error = 0.0;
    for _ in 0..DLS_MAX_TRIES {
        let current_transform = arm.end_transform();
        let error = pose_diff_with_operational_space(
            target_transform,
            &current_transform,
            operational_space,
        );
        let (position_error, rotation_error) =
            target_error_norms(&error, operational_space, constrain_orientation);
        if position_error < DLS_ALLOWABLE_TARGET_DISTANCE
            && rotation_error < DLS_ALLOWABLE_TARGET_ANGLE
        {
            return Ok(());
        }
        last_position_error = position_error;
        last_rotation_error = rotation_error;

        let jacobian = jacobian_with_operational_space(arm, operational_space);
        let damping = adaptive_dls_damping(&jacobian);
        let rows = jacobian.nrows();
        let jjt = &jacobian * jacobian.transpose();
        let damped = jjt + DMatrix::identity(rows, rows) * damping * damping;
        let Some(task_space_step) = damped.lu().solve(&error) else {
            arm.set_joint_positions(&original_positions)
                .map_err(|error| format!("failed to restore joint positions: {error}"))?;
            return Err("DLS IK failed to solve the damped normal equations".to_string());
        };
        let mut joint_step = jacobian.transpose() * task_space_step;

        for value in joint_step.iter_mut() {
            *value = (*value * DLS_STEP_SCALE).clamp(-DLS_MAX_JOINT_STEP, DLS_MAX_JOINT_STEP);
        }

        let mut next_positions = arm.joint_positions();
        for (position, step) in next_positions.iter_mut().zip(joint_step.iter()) {
            *position += step;
        }
        arm.set_joint_positions_clamped(&next_positions);
    }

    arm.set_joint_positions(&original_positions)
        .map_err(|error| format!("failed to restore joint positions: {error}"))?;
    Err(format!(
        "DLS IK did not converge after {DLS_MAX_TRIES} iterations; position_error={last_position_error:.6}, rotation_error={last_rotation_error:.6}"
    ))
}

/// Builds the constrained task-space error vector between current and target poses.
fn pose_diff_with_operational_space(
    target: &Isometry3<f64>,
    current: &Isometry3<f64>,
    operational_space: [bool; 6],
) -> DVector<f64> {
    let position_diff = target.translation.vector - current.translation.vector;
    let rotation_diff = current.rotation.rotation_to(&target.rotation).scaled_axis();
    let full_diff = Vector6::new(
        position_diff.x,
        position_diff.y,
        position_diff.z,
        rotation_diff.x,
        rotation_diff.y,
        rotation_diff.z,
    );
    let active_dof = operational_space.iter().filter(|enabled| **enabled).count();
    let mut constrained_diff = DVector::zeros(active_dof);
    let mut index = 0;
    for (row, enabled) in operational_space.iter().enumerate() {
        if *enabled {
            constrained_diff[index] = full_diff[row];
            index += 1;
        }
    }
    constrained_diff
}

/// Filters the full Jacobian down to the active task-space rows.
fn jacobian_with_operational_space(
    arm: &k::SerialChain<f64>,
    operational_space: [bool; 6],
) -> DMatrix<f64> {
    let full_jacobian = k::jacobian(arm);
    let active_rows = operational_space
        .iter()
        .enumerate()
        .filter_map(|(row, enabled)| enabled.then_some(row))
        .collect::<Vec<_>>();

    DMatrix::from_fn(active_rows.len(), full_jacobian.ncols(), |row, col| {
        full_jacobian[(active_rows[row], col)]
    })
}

/// Raises damping near singular configurations based on the smallest singular value.
fn adaptive_dls_damping(jacobian: &DMatrix<f64>) -> f64 {
    let singular_values = jacobian.clone().svd(false, false).singular_values;
    let min_singular_value = singular_values
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);
    let singularity_ratio = ((DLS_SINGULARITY_THRESHOLD - min_singular_value)
        / DLS_SINGULARITY_THRESHOLD)
        .clamp(0.0, 1.0);

    DLS_BASE_DAMPING + singularity_ratio * DLS_SINGULARITY_DAMPING
}

/// Splits the constrained error vector into position and orientation norms.
fn target_error_norms(
    error: &DVector<f64>,
    operational_space: [bool; 6],
    constrain_orientation: bool,
) -> (f64, f64) {
    let mut position_error = Vec3::ZERO;
    let mut rotation_error = Vec3::ZERO;
    let mut index = 0;
    for row in 0..3 {
        if operational_space[row] {
            position_error[row] = error[index] as f32;
            index += 1;
        }
    }
    if constrain_orientation {
        for row in 0..3 {
            if operational_space[row + 3] {
                rotation_error[row] = error[index] as f32;
                index += 1;
            }
        }
    }

    (
        position_error.length() as f64,
        rotation_error.length() as f64,
    )
}

/// Converts URDF roll-pitch-yaw values into a nalgebra quaternion.
fn rpy_quat_f64(rpy: Vec3) -> UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(rpy.x as f64, rpy.y as f64, rpy.z as f64)
}

// All six joints use a 19-bit output encoder, zero offset, and matching direction.
const COUNTS_PER_TURN: f64 = 524288.0;

pub(crate) fn radians_to_counts(radians: f64) -> Result<i32, String> {
    let counts = (radians * COUNTS_PER_TURN / std::f64::consts::TAU).round();
    if !counts.is_finite() || counts < i32::MIN as f64 || counts > i32::MAX as f64 {
        return Err("Target is outside signed 32-bit encoder range".to_string());
    }
    Ok(counts as i32)
}

pub(crate) fn counts_to_radians(counts: i32) -> f32 {
    (counts as f64 * std::f64::consts::TAU / COUNTS_PER_TURN) as f32
}
