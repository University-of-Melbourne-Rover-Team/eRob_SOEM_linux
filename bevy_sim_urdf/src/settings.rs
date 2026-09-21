//! Command-line parsing and startup joint target initialization for the viewer.

use bevy::prelude::Vec3;
use std::collections::HashMap;
use std::env;
use std::path::{Path, PathBuf};

use crate::constants::{DEFAULT_HOME_JOINTS, DEFAULT_TRIANGLE_CAP, TASK_TARGET_LINK};
use crate::kinematics::solve_task_space_ik;

#[derive(bevy::prelude::Resource)]
pub(crate) struct ViewerSettings {
    pub(crate) urdf_path: PathBuf,
    pub(crate) mesh_dir_override: Option<PathBuf>,
    pub(crate) triangle_cap: usize,
    pub(crate) initial_joint_values: HashMap<String, f32>,
    pub(crate) target_xyz: Option<Vec3>,
    pub(crate) target_rpy: Vec3,
    pub(crate) target_orientation_enabled: bool,
    pub(crate) dry_run: bool,
    pub(crate) hardware: bool,
    pub(crate) initial_command_requested: bool,
}

/// Parses CLI arguments and computes the initial viewer settings.
pub(crate) fn parse_viewer_settings(repo_root: &Path) -> ViewerSettings {
    let mut triangle_cap = None;
    let mut mesh_dir_override = None;
    let mut urdf_path = None;
    let mut joints_arg = None;
    let mut target_xyz_arg = None;
    let mut target_rpy_arg = None;
    let mut dry_run = false;
    let mut hardware = false;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        if let Some(value) = arg.strip_prefix("--joints=") {
            joints_arg = Some(value.to_string());
            continue;
        }
        if let Some(value) = arg.strip_prefix("--target-xyz=") {
            target_xyz_arg = Some(value.to_string());
            continue;
        }
        if let Some(value) = arg.strip_prefix("--target-rpy=") {
            target_rpy_arg = Some(value.to_string());
            continue;
        }

        match arg.as_str() {
            "--hardware" => {
                hardware = true;
            }
            "--dry-run" => {
                dry_run = true;
            }
            "--tri-cap" => {
                if let Some(value) = args.next() {
                    if let Ok(parsed) = value.parse() {
                        triangle_cap = Some(parsed);
                    }
                }
            }
            "--mesh-dir" => {
                if let Some(value) = args.next() {
                    mesh_dir_override = Some(PathBuf::from(value));
                }
            }
            "--urdf" => {
                if let Some(value) = args.next() {
                    urdf_path = Some(PathBuf::from(value));
                }
            }
            "--joints" => {
                if let Some(value) = args.next() {
                    joints_arg = Some(value);
                }
            }
            "--target-xyz" => {
                if let Some(value) = args.next() {
                    target_xyz_arg = Some(value);
                }
            }
            "--target-rpy" => {
                if let Some(value) = args.next() {
                    target_rpy_arg = Some(value);
                }
            }
            _ => {}
        }
    }

    let urdf_path = resolve_repo_path(
        repo_root,
        urdf_path.unwrap_or_else(|| {
            PathBuf::from("assets")
                .join("urdf")
                .join("urdf_assembly_rigid_stl_collapsed.urdf")
        }),
    );
    let mesh_dir_override = mesh_dir_override.map(|path| resolve_repo_path(repo_root, path));
    let target_xyz = target_xyz_arg
        .as_deref()
        .map(parse_csv_vec3)
        .transpose()
        .expect("--target-xyz must be three comma-separated numbers");
    let target_rpy = target_rpy_arg
        .as_deref()
        .map(parse_csv_vec3)
        .transpose()
        .expect("--target-rpy must be three comma-separated numbers");
    let target_orientation_enabled = target_rpy.is_some();
    let target_rpy = target_rpy.unwrap_or(Vec3::ZERO);
    let initial_joint_values = compute_initial_joint_values(
        &urdf_path,
        joints_arg.as_deref(),
        target_xyz,
        target_orientation_enabled.then_some(target_rpy),
    )
    .expect("failed to calculate initial joint values with k");

    ViewerSettings {
        urdf_path,
        mesh_dir_override,
        triangle_cap: triangle_cap.unwrap_or(DEFAULT_TRIANGLE_CAP),
        initial_joint_values,
        target_xyz,
        target_rpy,
        target_orientation_enabled,
        dry_run,
        hardware,
        initial_command_requested: joints_arg.is_some() || target_xyz_arg.is_some(),
    }
}

/// Resolves a user-provided path against the crate directory when it is relative.
fn resolve_repo_path(repo_root: &Path, path: PathBuf) -> PathBuf {
    if path.is_absolute() {
        path
    } else {
        repo_root.join(path)
    }
}

/// Parses a comma-separated list of floating-point values.
fn parse_csv_f32(value: &str) -> Result<Vec<f32>, String> {
    value
        .split(',')
        .map(|part| {
            part.trim()
                .parse::<f32>()
                .map_err(|error| format!("failed to parse '{part}' as a number: {error}"))
        })
        .collect()
}

/// Parses a comma-separated xyz or rpy triple into a Bevy vector.
fn parse_csv_vec3(value: &str) -> Result<Vec3, String> {
    let values = parse_csv_f32(value)?;
    if values.len() != 3 {
        return Err(format!(
            "expected three comma-separated values, got {}",
            values.len()
        ));
    }
    Ok(Vec3::new(values[0], values[1], values[2]))
}

/// Builds the initial joint angle map, optionally solving IK for a startup target.
fn compute_initial_joint_values(
    urdf_path: &Path,
    joints_arg: Option<&str>,
    target_xyz: Option<Vec3>,
    target_rpy: Option<Vec3>,
) -> Result<HashMap<String, f32>, String> {
    let chain = k::Chain::<f64>::from_urdf_file(urdf_path)
        .map_err(|error| format!("failed to load URDF with k: {error}"))?;
    let joint_names = chain
        .iter_joints()
        .map(|joint| joint.name.clone())
        .collect::<Vec<_>>();
    let initial_positions = if let Some(joints_arg) = joints_arg {
        parse_csv_f32(joints_arg)?
            .into_iter()
            .map(|value| value as f64)
            .collect::<Vec<_>>()
    } else if chain.dof() == DEFAULT_HOME_JOINTS.len() {
        DEFAULT_HOME_JOINTS.to_vec()
    } else {
        vec![0.0; chain.dof()]
    };

    if initial_positions.len() != chain.dof() {
        return Err(format!(
            "expected {} joint values, got {}",
            chain.dof(),
            initial_positions.len()
        ));
    }

    chain
        .set_joint_positions(&initial_positions)
        .map_err(|error| format!("failed to set joint positions: {error}"))?;
    chain.update_transforms();

    if let Some(target_xyz) = target_xyz {
        solve_task_space_ik(&chain, TASK_TARGET_LINK, target_xyz, target_rpy)?;
    }

    Ok(joint_names
        .into_iter()
        .zip(
            chain
                .joint_positions()
                .into_iter()
                .map(|value| value as f32),
        )
        .collect())
}
