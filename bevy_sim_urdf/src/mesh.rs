//! Binary STL loading utilities for constructing Bevy meshes from URDF visuals.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::PrimitiveTopology;
use bevy::prelude::{Mesh, Vec3};
use std::fs;
use std::path::Path;

/// Loads a binary STL file into a Bevy mesh, optionally downsampling triangles.
pub(crate) fn load_binary_stl_mesh(
    path: &Path,
    scale: Vec3,
    triangle_cap: usize,
) -> Result<Mesh, String> {
    let bytes = fs::read(path).map_err(|error| error.to_string())?;
    if bytes.len() < 84 {
        return Err("file is too small to be a binary STL".to_string());
    }

    let triangle_count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    let expected_len = 84 + triangle_count * 50;
    if expected_len > bytes.len() {
        return Err(format!(
            "binary STL length mismatch: expected at least {expected_len}, got {}",
            bytes.len()
        ));
    }

    let stride = if triangle_cap == 0 {
        1
    } else {
        triangle_count.div_ceil(triangle_cap).max(1)
    };

    let sampled_count = triangle_count.div_ceil(stride);
    let mut positions = Vec::with_capacity(sampled_count * 3);
    let mut normals = Vec::with_capacity(sampled_count * 3);
    let mut uvs = Vec::with_capacity(sampled_count * 3);

    for triangle_index in (0..triangle_count).step_by(stride) {
        let offset = 84 + triangle_index * 50;
        let file_normal = read_vec3(&bytes, offset).normalize_or_zero();
        let vertices = [
            read_vec3(&bytes, offset + 12) * scale,
            read_vec3(&bytes, offset + 24) * scale,
            read_vec3(&bytes, offset + 36) * scale,
        ];
        let normal = if file_normal.length_squared() > 1.0e-10 {
            file_normal
        } else {
            (vertices[1] - vertices[0])
                .cross(vertices[2] - vertices[0])
                .normalize_or_zero()
        };

        for vertex in vertices {
            positions.push([vertex.x, vertex.y, vertex.z]);
            normals.push([normal.x, normal.y, normal.z]);
            uvs.push([0.0, 0.0]);
        }
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    Ok(mesh)
}

/// Reads a little-endian Vec3 from the STL byte buffer.
fn read_vec3(bytes: &[u8], offset: usize) -> Vec3 {
    Vec3::new(
        read_f32(bytes, offset),
        read_f32(bytes, offset + 4),
        read_f32(bytes, offset + 8),
    )
}

/// Reads one little-endian f32 from the STL byte buffer.
fn read_f32(bytes: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}
