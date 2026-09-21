//! Minimal URDF XML parsing for links, joints, visuals, and mesh references.

use bevy::prelude::{Color, Vec3};
use std::fs;
use std::path::{Path, PathBuf};

use crate::constants::{DEFAULT_LIMIT, KEY_PAIRS};
use crate::model::{JointSpec, LinkSpec, RobotModel, VisualSpec};

/// Resolves mesh filenames relative to the URDF or an override mesh directory.
pub(crate) fn resolve_mesh_path(
    urdf_dir: &Path,
    mesh_dir_override: Option<&Path>,
    mesh_file: &str,
) -> PathBuf {
    let mesh_path = Path::new(mesh_file);
    if mesh_path.is_absolute() {
        return mesh_path.to_path_buf();
    }

    if let Some(mesh_dir) = mesh_dir_override {
        return mesh_dir.join(mesh_path.file_name().unwrap_or_default());
    }

    urdf_dir.join(mesh_path)
}

/// Parses the URDF file into a lightweight robot model used by the viewer.
pub(crate) fn parse_urdf(path: &Path) -> Result<RobotModel, String> {
    let xml = fs::read_to_string(path).map_err(|error| error.to_string())?;
    let mut links = Vec::new();
    let mut joints = Vec::new();

    for block in tag_blocks(&xml, "link") {
        let Some(name) = attr_value(&block.start_tag, "name") else {
            continue;
        };
        let mut visuals = Vec::new();
        for visual_block in tag_blocks(&block.body, "visual") {
            let Some(mesh_tag) = first_tag(&visual_block.body, "mesh") else {
                continue;
            };
            let Some(mesh_file) = attr_value(&mesh_tag, "filename") else {
                continue;
            };

            let origin_tag = first_tag(&visual_block.body, "origin").unwrap_or_default();
            let color_tag = first_tag(&visual_block.body, "color").unwrap_or_default();
            let scale = attr_value(&mesh_tag, "scale")
                .map(|value| parse_vec3(&value, Vec3::ONE))
                .unwrap_or(Vec3::ONE);
            let color = attr_value(&color_tag, "rgba")
                .map(|value| parse_color(&value))
                .unwrap_or(Color::srgb(0.7, 0.7, 0.7));

            visuals.push(VisualSpec {
                mesh_file,
                xyz: attr_value(&origin_tag, "xyz")
                    .map(|value| parse_vec3(&value, Vec3::ZERO))
                    .unwrap_or(Vec3::ZERO),
                rpy: attr_value(&origin_tag, "rpy")
                    .map(|value| parse_vec3(&value, Vec3::ZERO))
                    .unwrap_or(Vec3::ZERO),
                scale,
                color,
            });
        }
        links.push(LinkSpec { name, visuals });
    }

    let mut moving_index = 0usize;
    for block in tag_blocks(&xml, "joint") {
        let Some(name) = attr_value(&block.start_tag, "name") else {
            continue;
        };
        let joint_type =
            attr_value(&block.start_tag, "type").unwrap_or_else(|| "fixed".to_string());
        let parent = first_tag(&block.body, "parent")
            .and_then(|tag| attr_value(&tag, "link"))
            .unwrap_or_default();
        let child = first_tag(&block.body, "child")
            .and_then(|tag| attr_value(&tag, "link"))
            .unwrap_or_default();
        let origin_tag = first_tag(&block.body, "origin").unwrap_or_default();
        let axis_tag = first_tag(&block.body, "axis").unwrap_or_default();
        let limit_tag = first_tag(&block.body, "limit").unwrap_or_default();

        let is_moving = joint_type == "revolute" || joint_type == "continuous";
        let (increase_key, decrease_key) = if is_moving {
            let pair = KEY_PAIRS.get(moving_index).copied();
            moving_index += 1;
            pair.map(|(increase, decrease)| (Some(increase), Some(decrease)))
                .unwrap_or((None, None))
        } else {
            (None, None)
        };

        let (mut lower, mut upper) = if joint_type == "continuous" {
            (f32::NEG_INFINITY, f32::INFINITY)
        } else {
            (
                attr_value(&limit_tag, "lower")
                    .and_then(|value| value.parse::<f32>().ok())
                    .unwrap_or(-DEFAULT_LIMIT),
                attr_value(&limit_tag, "upper")
                    .and_then(|value| value.parse::<f32>().ok())
                    .unwrap_or(DEFAULT_LIMIT),
            )
        };
        if lower >= upper {
            lower = -DEFAULT_LIMIT;
            upper = DEFAULT_LIMIT;
        }
        joints.push(JointSpec {
            name,
            joint_type,
            parent,
            child,
            origin_xyz: attr_value(&origin_tag, "xyz")
                .map(|value| parse_vec3(&value, Vec3::ZERO))
                .unwrap_or(Vec3::ZERO),
            origin_rpy: attr_value(&origin_tag, "rpy")
                .map(|value| parse_vec3(&value, Vec3::ZERO))
                .unwrap_or(Vec3::ZERO),
            axis: attr_value(&axis_tag, "xyz")
                .map(|value| parse_vec3(&value, Vec3::Z))
                .unwrap_or(Vec3::Z),
            lower,
            upper,
            increase_key,
            decrease_key,
        });
    }

    Ok(RobotModel { links, joints })
}

struct TagBlock {
    start_tag: String,
    body: String,
}

/// Extracts all matching XML blocks for a tag using a small URDF-oriented parser.
fn tag_blocks(xml: &str, tag: &str) -> Vec<TagBlock> {
    let mut blocks = Vec::new();
    let open_pattern = format!("<{tag}");
    let close_pattern = format!("</{tag}>");
    let mut search_from = 0usize;

    while let Some(open_rel) = xml[search_from..].find(&open_pattern) {
        let open = search_from + open_rel;
        let Some(start_end_rel) = xml[open..].find('>') else {
            break;
        };
        let start_end = open + start_end_rel + 1;
        let start_tag = xml[open..start_end].to_string();

        if start_tag.trim_end().ends_with("/>") {
            blocks.push(TagBlock {
                start_tag,
                body: String::new(),
            });
            search_from = start_end;
            continue;
        }

        let Some(close_rel) = xml[start_end..].find(&close_pattern) else {
            break;
        };
        let close = start_end + close_rel;
        blocks.push(TagBlock {
            start_tag,
            body: xml[start_end..close].to_string(),
        });
        search_from = close + close_pattern.len();
    }

    blocks
}

/// Finds the first opening XML tag with the requested name.
fn first_tag(xml: &str, tag: &str) -> Option<String> {
    let open = format!("<{tag}");
    let start = xml.find(&open)?;
    let end = xml[start..].find('>')?;
    Some(xml[start..start + end + 1].to_string())
}

/// Extracts a quoted attribute value from an XML tag.
fn attr_value(tag: &str, attr: &str) -> Option<String> {
    let pattern = format!("{attr}=\"");
    let start = tag.find(&pattern)? + pattern.len();
    let end = tag[start..].find('"')?;
    Some(tag[start..start + end].to_string())
}

/// Parses a whitespace-separated xyz/rpy vector, falling back on malformed input.
fn parse_vec3(value: &str, fallback: Vec3) -> Vec3 {
    let parts = value
        .split_whitespace()
        .filter_map(|part| part.parse::<f32>().ok())
        .collect::<Vec<_>>();
    if parts.len() == 3 {
        Vec3::new(parts[0], parts[1], parts[2])
    } else {
        fallback
    }
}

/// Parses URDF rgba text into a Bevy color.
fn parse_color(value: &str) -> Color {
    let parts = value
        .split_whitespace()
        .filter_map(|part| part.parse::<f32>().ok())
        .collect::<Vec<_>>();
    let r = *parts.first().unwrap_or(&0.7);
    let g = *parts.get(1).unwrap_or(&0.7);
    let b = *parts.get(2).unwrap_or(&0.7);
    let a = *parts.get(3).unwrap_or(&1.0);
    Color::srgba(r, g, b, a)
}
