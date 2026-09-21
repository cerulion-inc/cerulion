// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle pins for the `VizConfig` parametrization.
//!
//! Two contracts, per the chunk brief:
//!
//! (a) **`Default` IS the Go2 values.** Every field of `VizConfig::default()`
//!     is asserted against a HAND-PASTED literal copied from the pre-move Go2
//!     source — NOT read back from the same constant the default is built from —
//!     so a drift in any Go2 constant (or a mis-wired default) fails here.
//!
//! (b) **A non-default config flows through.** For each constant group where an
//!     observable is practical, a custom config produces custom output through
//!     the config-driven builder: `CameraConfig` → the pinhole's
//!     resolution/focal inputs; `EntityTopology` → resolved entity paths;
//!     `BlueprintConfig` → the consumed layout params (the `rerun::Blueprint` is
//!     opaque, so the flow-through is asserted at the config + it is proven to
//!     build); `StreamConfig` → the `StoreInfo` identity the built stream
//!     carries, read back through the shared `build_recording_builder` helper +
//!     a non-network `memory()` sink (the same builder production connects
//!     with). The
//!     URDF flow-through (private `UrdfModel`) is pinned in-module in
//!     `skeleton.rs` (`urdf_config_flows_through_to_entity_paths_and_bindings`).

use cerulion_viz::blueprint::{blueprint_from, BlueprintConfig};
use cerulion_viz::config::VizConfig;
use cerulion_viz::skeleton::UrdfConfig;
use cerulion_viz::stream::{build_recording_builder, StreamConfig};
use cerulion_viz::tf::{entity_path_for_frame, CameraConfig, EntityTopology};

// =========================================================================
// (a) VizConfig::default() IS the Go2 values (hand-pasted literals).
// =========================================================================

#[test]
fn default_stream_is_go2() {
    let s = VizConfig::default().stream;
    assert_eq!(s.app_id, "go2");
    assert_eq!(s.recording_id, "go2_live");
    // The whole-config default equals the group default (aggregation pin).
    assert_eq!(VizConfig::default().stream, StreamConfig::default());
}

#[test]
fn default_camera_is_go2_720p() {
    let c = VizConfig::default().camera;
    assert_eq!(c.width, 1280);
    assert_eq!(c.height, 720);
    assert_eq!(c.focal_px, 650.0_f32);
    assert_eq!(VizConfig::default().camera, CameraConfig::default());
}

#[test]
fn default_urdf_is_go2() {
    let u = VizConfig::default().urdf;
    assert_eq!(
        u.default_path,
        "/home/unitree/unitree_ros/robots/go2_description/urdf/go2_description.urdf"
    );
    assert_eq!(u.robot_root, "world/tf-tree/robot");
    // The Go2 SDK motor-array order (FR/FL/RR/RL leg, hip/thigh/calf within).
    let joints: Vec<&str> = u.motor_joints.iter().map(|s| s.as_str()).collect();
    assert_eq!(
        joints,
        vec![
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
        ]
    );
    assert_eq!(u, UrdfConfig::default());
}

#[test]
fn default_blueprint_is_go2() {
    let b = VizConfig::default().blueprint;
    assert_eq!(b.world_origin, "/world");
    assert_eq!(b.scene_name, "Scene");
    assert_eq!(b.plots_name, "Telemetry");
    assert_eq!(b.status_name, "Status & Field Dumps");
    assert_eq!(b.scene_share, 3.0_f32);
    assert_eq!(b.sidebar_share, 1.0_f32);
    assert!(b.auto_views);
    assert_eq!(b, BlueprintConfig::default());
}

#[test]
fn default_topology_is_go2() {
    let t = VizConfig::default().topology;
    assert_eq!(t.world_root, "world");
    assert_eq!(t.odom_entity, "world/tf-tree/odom");
    assert_eq!(t.base_entity, "world/tf-tree/odom/base");
    assert_eq!(t.lidar_entity, "world/tf-tree/odom/base/lidar");
    assert_eq!(t.camera_entity, "world/tf-tree/odom/base/camera");
    fn as_strs(v: &[String]) -> Vec<&str> {
        v.iter().map(String::as_str).collect()
    }
    assert_eq!(as_strs(&t.odom_frames), ["odom", "odom_frame"]);
    assert_eq!(
        as_strs(&t.base_frames),
        ["base", "base_link", "base_footprint"]
    );
    assert_eq!(
        as_strs(&t.lidar_frames),
        ["lidar", "livox_frame", "utlidar_lidar", "laser", "velodyne"]
    );
    assert_eq!(
        as_strs(&t.camera_frames),
        [
            "camera",
            "camera_link",
            "front_camera",
            "camera_optical_frame"
        ]
    );
    assert_eq!(t, EntityTopology::default());
}

// =========================================================================
// (b) A non-default config flows through to observable output.
// =========================================================================

#[test]
fn camera_config_flows_through_to_pinhole_inputs() {
    // Default resolves the Go2 720p intrinsics (the pinhole build inputs).
    assert_eq!(CameraConfig::default().resolution(), [1280.0, 720.0]);
    assert_eq!(CameraConfig::default().focal(), [650.0, 650.0]);
    // A non-default camera resolves ITS intrinsics — the same values that feed
    // the (opaque) `rerun::Pinhole`.
    let custom = CameraConfig {
        width: 640,
        height: 480,
        focal_px: 500.0,
    };
    assert_eq!(custom.resolution(), [640.0, 480.0]);
    assert_eq!(custom.focal(), [500.0, 500.0]);
    // And the pinhole builds without panic (the flow-through consumer).
    let _pinhole = custom.pinhole();
}

#[test]
fn entity_topology_flows_through_to_paths() {
    // Default: the two public surfaces — the free `entity_path_for_frame` and
    // `EntityTopology::default().entity_path` — agree on output for the default
    // topology. NOTE: this is an AGREEMENT pin, not a routing proof.
    // `entity_path_for_frame` is defined as `default_topology().entity_path`,
    // and `default_topology` is a `OnceLock` over `EntityTopology::default`, so
    // BOTH sides reduce to `EntityTopology::default().entity_path("lidar")` — the
    // equality holds by construction (it would pass even if production hardcoded
    // the strings). The REAL flow-through — a NON-default topology changing the
    // resolved output — is pinned by the custom-topology block below.
    let d = EntityTopology::default();
    assert_eq!(d.entity_path("lidar"), entity_path_for_frame("lidar"));
    assert_eq!(d.entity_path("lidar").path, "world/tf-tree/odom/base/lidar");
    assert!(d.entity_path("lidar").known);

    // A non-default topology maps its OWN aliases to its OWN entities.
    let custom = EntityTopology {
        world_root: "scene".to_string(),
        odom_entity: "scene/world".to_string(),
        base_entity: "scene/world/body".to_string(),
        lidar_entity: "scene/world/body/scanner".to_string(),
        camera_entity: "scene/world/body/eye".to_string(),
        odom_frames: vec!["map".to_string()],
        base_frames: vec!["torso".to_string()],
        lidar_frames: vec!["scanner_link".to_string()],
        camera_frames: vec!["eye_link".to_string()],
    };
    assert_eq!(
        custom.entity_path("scanner_link").path,
        "scene/world/body/scanner"
    );
    assert!(custom.entity_path("scanner_link").known);
    assert_eq!(custom.entity_path("torso").path, "scene/world/body");
    // An unknown frame falls back under the custom base entity (sanitized).
    let fb = custom.entity_path("wrist_cam");
    assert_eq!(fb.path, "scene/world/body/wrist_cam");
    assert!(!fb.known);
    // A Go2 alias is NOT known under the custom topology (the override replaced
    // the alias table, not merely added to it).
    assert!(!custom.entity_path("lidar").known);
}

#[test]
fn blueprint_config_flows_through() {
    // Default layout params feed `go2_blueprint` (byte-identical).
    let d = BlueprintConfig::default();
    assert_eq!(d.world_origin, "/world");
    assert_eq!([d.scene_share, d.sidebar_share], [3.0, 1.0]);
    // A non-default layout is CONSUMED by the builder (the `rerun::Blueprint` is
    // opaque, so the observable is the resolved layout params + that it builds).
    let custom = BlueprintConfig {
        world_origin: "/robot".to_string(),
        scene_name: "Hero".to_string(),
        plots_name: "Signals".to_string(),
        status_name: "Notes".to_string(),
        scene_share: 5.0,
        sidebar_share: 2.0,
        auto_views: false,
    };
    assert_eq!(custom.world_origin, "/robot");
    assert_eq!([custom.scene_share, custom.sidebar_share], [5.0, 2.0]);
    assert!(!custom.auto_views);
    let _blueprint = blueprint_from(&custom);
    // The default-config builder path also constructs.
    let _default_blueprint = blueprint_from(&d);
}

#[test]
fn stream_config_flows_through_to_recording_identity() {
    // A custom recording identity reaches the built `RecordingStream`'s
    // `StoreInfo` through the SAME builder-construction production uses
    // (`build_recording_builder`, which `connect_with_config` calls). We attach
    // a non-network `memory()` sink so no socket opens — the store info is
    // seeded from the config regardless of sink (both `.memory()` and
    // `.connect_grpc()` build it via the SDK builder's `into_args`), so this
    // pins the config→stream flow-through, NOT a read-back of the just-assigned
    // literal. A revert hardcoding the identity in the helper fails here.
    let custom = StreamConfig {
        app_id: "spot".to_string(),
        recording_id: "spot_live".to_string(),
    };
    let (rec, _storage) = build_recording_builder(&custom)
        .memory()
        .expect("memory sink builds");
    let info = rec.store_info().expect("a built stream carries StoreInfo");
    assert_eq!(info.application_id().as_str(), "spot");
    assert_eq!(info.recording_id().as_str(), "spot_live");

    // The default (Go2) identity likewise reaches the stream — the production
    // `connect_default` builds through this same helper, so a drift in the
    // `StreamConfig` default would surface here too.
    let (rec_d, _storage_d) = build_recording_builder(&StreamConfig::default())
        .memory()
        .expect("memory sink builds");
    let info_d = rec_d
        .store_info()
        .expect("a built stream carries StoreInfo");
    assert_eq!(info_d.application_id().as_str(), "go2");
    assert_eq!(info_d.recording_id().as_str(), "go2_live");
    assert_ne!(custom, StreamConfig::default());
}
