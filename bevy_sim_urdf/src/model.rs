//! Data structures that represent the parsed robot model and Bevy resource wrapper.

use bevy::prelude::{Color, KeyCode, Resource, Vec3};

#[derive(Clone)]
pub(crate) struct LinkSpec {
    pub(crate) name: String,
    pub(crate) visuals: Vec<VisualSpec>,
}

#[derive(Clone)]
pub(crate) struct VisualSpec {
    pub(crate) mesh_file: String,
    pub(crate) xyz: Vec3,
    pub(crate) rpy: Vec3,
    pub(crate) scale: Vec3,
    pub(crate) color: Color,
}

#[derive(Clone)]
pub(crate) struct JointSpec {
    pub(crate) name: String,
    pub(crate) joint_type: String,
    pub(crate) parent: String,
    pub(crate) child: String,
    pub(crate) origin_xyz: Vec3,
    pub(crate) origin_rpy: Vec3,
    pub(crate) axis: Vec3,
    pub(crate) lower: f32,
    pub(crate) upper: f32,
    pub(crate) increase_key: Option<KeyCode>,
    pub(crate) decrease_key: Option<KeyCode>,
}

impl JointSpec {
    /// Returns true when the joint should be controllable in the viewer.
    pub(crate) fn is_moving(&self) -> bool {
        self.joint_type == "revolute" || self.joint_type == "continuous"
    }
}

pub(crate) struct RobotModel {
    pub(crate) links: Vec<LinkSpec>,
    pub(crate) joints: Vec<JointSpec>,
}

#[derive(Resource)]
pub(crate) struct RobotModelResource(pub(crate) RobotModel);

/// Wraps the parsed robot model so Bevy systems can access it as a resource.
pub(crate) fn model_resource(model: RobotModel) -> RobotModelResource {
    RobotModelResource(model)
}
