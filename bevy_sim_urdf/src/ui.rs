use bevy::prelude::*;

use crate::scene::JointState;
use crate::hardware::HardwareBridge;

#[derive(Resource)]
pub(crate) struct UiFont(pub(crate) Handle<Font>);

#[derive(Component)]
pub(crate) struct JointAnglesText;

pub(crate) fn load_ui_font(mut commands: Commands, asset_server: Res<AssetServer>) {
    let font = asset_server.load("fonts/Inter.ttf");
    commands.insert_resource(UiFont(font));
}

pub(crate) fn spawn_joint_angles_ui(commands: &mut Commands, ui_font: &UiFont) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: px(20),
                top: px(20),
                width: px(260),
                padding: UiRect::all(px(16)),
                flex_direction: FlexDirection::Column,
                row_gap: px(8),
                ..default()
            },
            BackgroundColor(Color::srgba(0.02, 0.025, 0.03, 0.9)),
        ))
        .with_children(|parent| {
            // parent.spawn((
            //     Text::new("JOINT POSITIONS"),
            //     TextFont {
            //         font_size: 22.0,
            //         ..default()
            //     },
            // ));

            parent.spawn((
                Text::new("J1   0.00°\nJ2   0.00°\nJ3   0.00°\nJ4   0.00°\nJ5   0.00°\nJ6   0.00°"),
                TextFont {
                    font: ui_font.0.clone(),
                    font_size: 18.0,
                    ..default()
                },
                JointAnglesText,
            ));
        });
}

pub(crate) fn update_joint_angles_ui(
    hardware: Option<Res<HardwareBridge>>,
    joints: Query<&JointState>,
    mut text_query: Query<&mut Text, With<JointAnglesText>>,
) {
    let Ok(mut text) = text_query.single_mut() else {
        return;
    };

    let mut output = String::from("JOINT POSITIONS\n");
    if let Some(hardware) = &hardware {
        output.push_str(hardware.status());
        output.push('\n');
        if let Some(error) = &hardware.last_error {
            output.push_str(error);
            output.push('\n');
        }
    }
    output.push('\n');

    if hardware.as_ref().is_some_and(|hardware| !hardware.has_feedback()) {
        **text = output;
        return;
    }
    let mut ordered_joints = joints.iter().collect::<Vec<_>>();
    ordered_joints.sort_by(|a, b| a.name.cmp(&b.name));
    for joint in ordered_joints {
        output.push_str(&format!(
            "{}   {:>7.2}°\n",
            joint.name,
            joint.value.to_degrees(),
        ));
    }

    **text = output;
}
