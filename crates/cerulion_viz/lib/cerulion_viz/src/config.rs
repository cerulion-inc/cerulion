// SPDX-License-Identifier: AGPL-3.0-only
//! `VizConfig` — the parametrization surface for the (previously
//! Go2-hardcoded) visualization constants.
//!
//! When `go2_viz` was lifted into the product `cerulion_viz`,
//! the five Go2-specific constant groups were parametrized behind a config
//! surface so the `cerulion viz` verb can drive a
//! per-robot visualization while `examples/go2` stays byte-identical. Each group
//! is a small `Default`-carrying struct co-located with the module + private
//! helpers that use it; `VizConfig` aggregates the five, and its `Default`
//! reproduces the exact Go2 values.
//!
//! | # | Group | Type | Home module |
//! |---|-------|------|-------------|
//! | 1 | app / recording id | [`StreamConfig`] | [`crate::stream`] |
//! | 2 | camera pinhole | [`CameraConfig`] | [`crate::tf`] |
//! | 3 | URDF / joint | [`UrdfConfig`] | [`crate::skeleton`] |
//! | 4 | blueprint layout | [`BlueprintConfig`] | [`crate::blueprint`] |
//! | 5 | entity topology | [`EntityTopology`] | [`crate::tf`] |
//!
//! Production routes through each group's `Default` (so the config is genuinely
//! USED, never an inert parallel struct): `stream::connect_default`,
//! `tf::log_camera_pinhole_static`, `tf::entity_path_for_frame`,
//! `blueprint::go2_blueprint`, and `skeleton::parse_urdf` each delegate to their
//! group's default. Each group also exposes a config-driven builder
//! (`connect_with_config`, `CameraConfig::pinhole`, `EntityTopology::entity_path`,
//! `blueprint_from`, `Skeleton::from_urdf_str_with_config`) for a non-default
//! robot.

pub use crate::blueprint::BlueprintConfig;
pub use crate::skeleton::UrdfConfig;
pub use crate::stream::StreamConfig;
pub use crate::tf::{CameraConfig, EntityTopology};

/// The full visualization configuration surface: the five parametrized
/// constant groups. `Default` reproduces the exact Go2 values so `examples/go2`
/// stays byte-identical; the `cerulion viz` verb
/// constructs a per-robot `VizConfig`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct VizConfig {
    /// App / recording identity (constant 1).
    pub stream: StreamConfig,
    /// Camera pinhole intrinsics (constant 2).
    pub camera: CameraConfig,
    /// URDF / joint config (constant 3).
    pub urdf: UrdfConfig,
    /// Blueprint dashboard layout (constant 4).
    pub blueprint: BlueprintConfig,
    /// Frame_id → entity-path topology (constant 5).
    pub topology: EntityTopology,
}
