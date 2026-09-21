//! Bevy scene setup, ECS components, input systems, camera control, and gizmo drawing.

use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;
use std::collections::HashMap;

use crate::constants::{JOINT_SPEED, TARGET_MOVE_SPEED, TARGET_ROTATE_SPEED, TASK_TARGET_LINK};
use crate::hardware::HardwareBridge;
use crate::kinematics::{
    KinematicsState, create_kinematics_state, link_position_from_joint_values,
    solve_task_space_ik_values,
};
use crate::mesh::load_binary_stl_mesh;
use crate::model::RobotModelResource;
use crate::settings::ViewerSettings;
use crate::ui::{UiFont, spawn_joint_angles_ui};
use crate::urdf::resolve_mesh_path;

#[derive(Component)]
pub(crate) struct JointState {
    pub(crate) name: String,
    origin_xyz: Vec3,
    origin_rotation: Quat,
    axis: Vec3,
    pub(crate) value: f32,
    pub(crate) target_value: f32,
    lower: f32,
    upper: f32,
    increase_key: KeyCode,
    decrease_key: KeyCode,
}

impl JointState {
    pub(crate) fn update_transform(&self, transform: &mut Transform) {
        transform.translation = self.origin_xyz;
        transform.rotation = self.origin_rotation * Quat::from_axis_angle(self.axis, self.value);
    }
}

#[derive(Component)]
pub(crate) struct OrbitCamera {
    target: Vec3,
    radius: f32,
    yaw: f32,
    pitch: f32,
}

#[derive(Component)]
pub(crate) struct TargetMarker;

#[derive(Component)]
pub(crate) struct HardwareRobotRoot;

#[derive(Resource)]
pub(crate) struct TaskSpaceControl {
    enabled: bool,
    target: Vec3,
    target_rpy: Vec3,
    target_orientation_enabled: bool,
    target_link: String,
}

/// Builds lights, ground, robot entities, meshes, target marker, and camera.
pub(crate) fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    settings: Res<ViewerSettings>,
    model: Res<RobotModelResource>,
    ui_font: Res<UiFont>,
) {
    commands.spawn((
        PointLight {
            intensity: 3500.0,
            range: 8.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(0.4, -0.8, 1.2),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 2500.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.8, -0.4, -0.5)),
    ));

    spawn_ground(&mut commands, &mut meshes, &mut materials);

    let mut kinematics = create_kinematics_state(&settings.urdf_path)
        .expect("failed to initialize k kinematics state");
    let task_target = settings.target_xyz.unwrap_or_else(|| {
        link_position_from_joint_values(
            &mut kinematics,
            &settings.initial_joint_values,
            TASK_TARGET_LINK,
        )
        .unwrap_or(Vec3::new(-0.20, 0.30, 0.28))
    });
    commands.insert_resource(TaskSpaceControl {
        enabled: settings.target_xyz.is_some(),
        target: task_target,
        target_rpy: settings.target_rpy,
        target_orientation_enabled: settings.target_orientation_enabled,
        target_link: TASK_TARGET_LINK.to_string(),
    });
    commands.insert_resource(kinematics);
    spawn_target_marker(
        &mut commands,
        &mut meshes,
        &mut materials,
        task_target,
        settings.target_rpy,
    );

    let mut link_entities = HashMap::new();
    for link in &model.0.links {
        let entity = commands
            .spawn((Transform::default(), Visibility::Inherited, Name::new(link.name.clone())))
            .id();
        if settings.hardware_config.is_some()
            && !model.0.joints.iter().any(|joint| joint.child == link.name)
        {
            // Until the first feedback sample there is no measured pose to show.
            commands.entity(entity).insert((HardwareRobotRoot, Visibility::Hidden));
        }
        link_entities.insert(link.name.clone(), entity);
    }

    let moving_joints = model
        .0
        .joints
        .iter()
        .filter(|joint| joint.is_moving())
        .count();
    println!(
        "Loaded {} links, {} joints, {} moving joints",
        model.0.links.len(),
        model.0.joints.len(),
        moving_joints
    );

    for joint in &model.0.joints {
        let Some(parent) = link_entities.get(&joint.parent).copied() else {
            warn!(
                "joint {} references missing parent {}",
                joint.name, joint.parent
            );
            continue;
        };
        let Some(child) = link_entities.get(&joint.child).copied() else {
            warn!(
                "joint {} references missing child {}",
                joint.name, joint.child
            );
            continue;
        };

        let origin_rotation = rpy_quat(joint.origin_rpy);
        let axis = joint.axis.normalize_or_zero();
        let initial_value = settings
            .initial_joint_values
            .get(&joint.name)
            .copied()
            .unwrap_or(0.0)
            .clamp(joint.lower, joint.upper);
        let joint_rotation = if joint.is_moving() {
            Quat::from_axis_angle(axis, initial_value)
        } else {
            Quat::IDENTITY
        };
        commands.entity(child).insert(
            Transform::from_translation(joint.origin_xyz)
                .with_rotation(origin_rotation * joint_rotation),
        );

        if joint.is_moving() {
            if let (Some(increase_key), Some(decrease_key)) =
                (joint.increase_key, joint.decrease_key)
            {
                commands.entity(child).insert(JointState {
                    name: joint.name.clone(),
                    origin_xyz: joint.origin_xyz,
                    origin_rotation,
                    axis,
                    value: initial_value,
                    target_value: initial_value,
                    lower: joint.lower,
                    upper: joint.upper,
                    increase_key,
                    decrease_key,
                });
            }
        }

        commands.entity(parent).add_child(child);
    }

    let urdf_dir = settings
        .urdf_path
        .parent()
        .expect("URDF should have a parent directory");

    for link in &model.0.links {
        let Some(entity) = link_entities.get(&link.name).copied() else {
            continue;
        };

        for visual in &link.visuals {
            let mesh_path = resolve_mesh_path(
                urdf_dir,
                settings.mesh_dir_override.as_deref(),
                &visual.mesh_file,
            );
            let mesh = load_binary_stl_mesh(&mesh_path, visual.scale, settings.triangle_cap)
                .unwrap_or_else(|error| panic!("failed to load {}: {error}", mesh_path.display()));
            let material = materials.add(StandardMaterial {
                base_color: visual.color,
                perceptual_roughness: 0.65,
                metallic: 0.05,
                cull_mode: None,
                ..default()
            });

            commands.entity(entity).with_children(|parent| {
                parent.spawn((
                    Mesh3d(meshes.add(mesh)),
                    MeshMaterial3d(material),
                    Transform::from_translation(visual.xyz).with_rotation(rpy_quat(visual.rpy)),
                ));
            });
        }
    }

    let target = Vec3::new(0.03, 0.01, 0.18);
    commands.spawn((
        Camera3d::default(),
        camera_transform(-0.7, -0.55, 1.8, target),
        OrbitCamera {
            target,
            radius: 1.8,
            yaw: -0.7,
            pitch: -0.55,
        },
    ));

    spawn_joint_angles_ui(&mut commands, &ui_font);
}

/// Applies direct keyboard-driven joint angle updates when task-space IK is disabled.
pub(crate) fn drive_joints(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    task_control: Option<Res<TaskSpaceControl>>,
    hardware: Option<Res<HardwareBridge>>,
    mut joints: Query<(&mut JointState, &mut Transform)>,
) {
    if hardware.as_ref().is_some_and(|bridge| !bridge.can_command()) {
        return;
    }
    if let Some(task_control) = task_control {
        if task_control.enabled {
            return;
        }
    }
    for (mut joint, mut transform) in &mut joints {
        let mut delta = 0.0;
        if keyboard.pressed(joint.increase_key) {
            delta += JOINT_SPEED * time.delta_secs();
        }
        if keyboard.pressed(joint.decrease_key) {
            delta -= JOINT_SPEED * time.delta_secs();
        }
        if keyboard.just_pressed(KeyCode::Space) {
            joint.target_value = 0.0_f32.clamp(joint.lower, joint.upper);
        } else if delta != 0.0 {
            joint.target_value = (joint.target_value + delta).clamp(joint.lower, joint.upper);
        }
        if hardware.is_none() {
            joint.value = joint.target_value;
            joint.update_transform(&mut transform);
        }
    }
}

/// Moves the task-space target from keyboard input and solves IK for the robot.
pub(crate) fn drive_task_space_target(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut task_control: ResMut<TaskSpaceControl>,
    hardware: Option<Res<HardwareBridge>>,
    mut kinematics: ResMut<KinematicsState>,
    mut marker: Query<&mut Transform, (With<TargetMarker>, Without<JointState>)>,
    mut joints: Query<(&mut JointState, &mut Transform), Without<TargetMarker>>,
) {
    if hardware.as_ref().is_some_and(|bridge| !bridge.can_command()) {
        return;
    }
    if keyboard.just_pressed(KeyCode::KeyM) {
        task_control.enabled = !task_control.enabled;
        if task_control.enabled && hardware.is_some() {
            let actual = joints.iter().map(|(joint, _)| (joint.name.clone(), joint.value)).collect();
            if let Ok(position) = link_position_from_joint_values(
                &mut kinematics, &actual, &task_control.target_link,
            ) {
                task_control.target = position;
                task_control.target_orientation_enabled = false;
            }
        }
        println!(
            "task-space IK: {}",
            if task_control.enabled {
                "enabled"
            } else {
                "disabled"
            }
        );
    }

    if !task_control.enabled {
        return;
    }

    let mut direction = Vec3::ZERO;
    if keyboard.pressed(KeyCode::ArrowRight) {
        direction.x += 1.0;
    }
    if keyboard.pressed(KeyCode::ArrowLeft) {
        direction.x -= 1.0;
    }
    if keyboard.pressed(KeyCode::ArrowUp) {
        direction.y += 1.0;
    }
    if keyboard.pressed(KeyCode::ArrowDown) {
        direction.y -= 1.0;
    }
    if keyboard.pressed(KeyCode::PageUp) {
        direction.z += 1.0;
    }
    if keyboard.pressed(KeyCode::PageDown) {
        direction.z -= 1.0;
    }

    let mut rotation_delta = Vec3::ZERO;
    if keyboard.pressed(KeyCode::KeyZ) {
        rotation_delta.x += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyX) {
        rotation_delta.x -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyC) {
        rotation_delta.y += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyV) {
        rotation_delta.y -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyB) {
        rotation_delta.z += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyN) {
        rotation_delta.z -= 1.0;
    }

    let moved = direction.length_squared() > 0.0;
    let rotated = rotation_delta.length_squared() > 0.0;
    if moved {
        task_control.target += direction.normalize() * TARGET_MOVE_SPEED * time.delta_secs();
    }
    if rotated {
        task_control.target_orientation_enabled = true;
        task_control.target_rpy +=
            rotation_delta.normalize() * TARGET_ROTATE_SPEED * time.delta_secs();
    }
    for mut transform in &mut marker {
        transform.translation = task_control.target;
        transform.rotation = rpy_quat(task_control.target_rpy);
    }

    if !(moved || rotated) {
        return;
    }

    let current_values = joints
        .iter_mut()
        .map(|(joint, _)| (joint.name.clone(), joint.value))
        .collect::<HashMap<_, _>>();

    match solve_task_space_ik_values(
        &mut kinematics,
        &current_values,
        &task_control.target_link,
        task_control.target,
        task_control
            .target_orientation_enabled
            .then_some(task_control.target_rpy),
    ) {
        Ok(values) => {
            for (mut joint, mut transform) in &mut joints {
                if let Some(value) = values.get(&joint.name) {
                    joint.target_value = value.clamp(joint.lower, joint.upper);
                    if hardware.is_none() {
                        joint.value = joint.target_value;
                        joint.update_transform(&mut transform);
                    }
                }
            }
        }
        Err(_error) => {
            warn!(
                "task-space IK did not converge; target may be unreachable or near a singularity"
            );
        }
    }
}

/// Updates the orbit camera from mouse drag and wheel input.
pub(crate) fn orbit_camera(
    mut mouse_motion: MessageReader<MouseMotion>,
    mut mouse_wheel: MessageReader<MouseWheel>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut cameras: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    let mut orbit_delta = Vec2::ZERO;
    if mouse_buttons.pressed(MouseButton::Left) || mouse_buttons.pressed(MouseButton::Right) {
        for event in mouse_motion.read() {
            orbit_delta += event.delta;
        }
    }

    let mut zoom_delta = 0.0;
    for event in mouse_wheel.read() {
        zoom_delta += event.y;
    }

    for (mut camera, mut transform) in &mut cameras {
        if orbit_delta != Vec2::ZERO {
            camera.yaw -= orbit_delta.x * 0.006;
            camera.pitch = (camera.pitch - orbit_delta.y * 0.006).clamp(-1.35, 1.35);
        }
        if zoom_delta != 0.0 {
            camera.radius = (camera.radius * (1.0 - zoom_delta * 0.08)).clamp(0.2, 8.0);
        }
        *transform = camera_transform(camera.yaw, camera.pitch, camera.radius, camera.target);
    }
}

/// Draws colored joint axes in world space for debugging.
pub(crate) fn draw_joint_axes(mut gizmos: Gizmos, joints: Query<(&JointState, &GlobalTransform)>) {
    for (joint, global_transform) in &joints {
        let transform = global_transform.compute_transform();
        let origin = transform.translation;
        let axis = (transform.rotation * joint.axis).normalize_or_zero();
        let color =
            if joint.axis.x.abs() > joint.axis.y.abs() && joint.axis.x.abs() > joint.axis.z.abs() {
                Color::srgb(1.0, 0.15, 0.15)
            } else if joint.axis.y.abs() > joint.axis.z.abs() {
                Color::srgb(0.2, 1.0, 0.25)
            } else {
                Color::srgb(0.25, 0.45, 1.0)
            };

        gizmos.line(origin - axis * 0.08, origin + axis * 0.08, color);
    }
}

/// Builds an orbit camera transform from yaw, pitch, radius, and target point.
fn camera_transform(yaw: f32, pitch: f32, radius: f32, target: Vec3) -> Transform {
    let direction = Vec3::new(
        yaw.cos() * pitch.cos(),
        yaw.sin() * pitch.cos(),
        pitch.sin(),
    );
    Transform::from_translation(target + direction * radius).looking_at(target, Vec3::Z)
}

/// Spawns the simple ground plane under the robot.
fn spawn_ground(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(3.0, 3.0, 0.01))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.12, 0.13, 0.13),
            perceptual_roughness: 0.9,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, -0.008),
    ));
}

/// Spawns the red task-space target marker cube.
fn spawn_target_marker(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    target: Vec3,
    target_rpy: Vec3,
) {
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.035, 0.035, 0.035))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.2, 0.15),
            emissive: Color::srgb(0.45, 0.04, 0.02).into(),
            ..default()
        })),
        Transform::from_translation(target).with_rotation(rpy_quat(target_rpy)),
        TargetMarker,
        Name::new("task_space_target"),
    ));
}

/// Converts URDF roll-pitch-yaw values into a Bevy quaternion.
fn rpy_quat(rpy: Vec3) -> Quat {
    Quat::from_rotation_z(rpy.z) * Quat::from_rotation_y(rpy.y) * Quat::from_rotation_x(rpy.x)
}

/// Initialize the task-space marker from the measured pose on connection/reconnection.
pub(crate) fn seed_hardware_task_target(
    hardware: Option<ResMut<HardwareBridge>>,
    mut task_control: ResMut<TaskSpaceControl>,
    mut kinematics: ResMut<KinematicsState>,
    joints: Query<&JointState>,
    mut marker: Query<&mut Transform, (With<TargetMarker>, Without<JointState>)>,
) {
    let Some(mut hardware) = hardware else { return; };
    if !hardware.reset_task_target { return; }
    hardware.reset_task_target = false;
    let actual = joints.iter().map(|joint| (joint.name.clone(), joint.value)).collect();
    if let Ok(position) = link_position_from_joint_values(
        &mut kinematics, &actual, &task_control.target_link,
    ) {
        task_control.target = position;
        task_control.target_orientation_enabled = false;
        for mut transform in &mut marker {
            transform.translation = position;
        }
    }
}

pub(crate) fn show_measured_robot(
    hardware: Option<Res<HardwareBridge>>,
    mut roots: Query<&mut Visibility, With<HardwareRobotRoot>>,
) {
    if hardware.as_ref().is_some_and(|hardware| hardware.has_feedback()) {
        for mut visibility in &mut roots {
            *visibility = Visibility::Inherited;
        }
    }
}
