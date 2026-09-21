// SPDX-License-Identifier: AGPL-3.0-only
//! Robotics-bench vendoring: acceptance pins for the vendored
//! ROS2 message packages — `vision_msgs`, `radar_msgs`, `grid_map_msgs`
//! (+ the transitive `unique_identifier_msgs/UUID` dep).
//!
//! These run against the REAL generated bindings (parse → resolve →
//! generate → compile), so they catch any drift in the vendored `.msg`
//! field set or its codegen. Each variable-payload type is exercised with
//! a genuine loan/fill round-trip over an isolated iceoryx2 `TestTransport`
//! (per-test SHM root → parallel-safe, no `#[serial]` needed), and the new
//! schema hashes are asserted pairwise-distinct (incl. the
//! `vision_msgs/Pose2D` vs `geometry_msgs/Pose2D` bare-name pair, which the
//! recipe-3 hash keeps distinct on the wire — though note these two differ
//! in BOTH package and field layout, so this pair alone does not isolate
//! package-qualification from layout-sensitivity).

use std::sync::atomic::{AtomicU64, Ordering};

use cerulion_core::message::ShmMessage;
use cerulion_core::testing::TestTransport;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::wire::MaxSliceLen;

use native_ros2_messages::grid_map_msgs;
use native_ros2_messages::radar_msgs;
use native_ros2_messages::vision_msgs;

// =====================================================================
// Test infrastructure (mirrors generated_bindings_test.rs / roundtrip_test.rs)
// =====================================================================

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

/// 64 KiB comfortably covers every payload in this suite.
const MAX_SLICE_LEN: MaxSliceLen = MaxSliceLen::const_new(64 * 1024);

/// Build an isolated iceoryx2 publisher + matching subscriber on a unique
/// topic. The returned `TestTransport` owns the iceoryx2 node and MUST be
/// kept in scope so the ports stay valid — each caller binds it to a local.
fn make_pub_sub(label: &str) -> (TestTransport, CerulionPublisher, CerulionSubscriber) {
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    let topic = format!("test/vendored/{label}/{id}");
    let tt = TestTransport::with_buffer_size(4);
    let publisher = tt.publisher(&topic, MAX_SLICE_LEN, 0);
    let subscriber = tt.subscriber(&topic);
    (tt, publisher, subscriber)
}

// =====================================================================
// vision_msgs
// =====================================================================

/// `vision_msgs/ObjectHypothesis` is the leaf hypothesis type embedded in
/// every Detection*/Classification message. It carries a variable `string
/// class_id` and a fixed `float64 score`. Round-trip BOTH: the variable
/// string via `set_class_id` / `class_id()`, the fixed scalar via the
/// loaned-section direct write / Deref read.
#[test]
fn vision_object_hypothesis_string_and_scalar_round_trip() {
    let (_tt, mut publisher, mut subscriber) = make_pub_sub("obj_hyp");

    {
        let mut proxy = publisher
            .loan_proxy::<vision_msgs::ObjectHypothesis>()
            .expect("loan_proxy");
        // Fixed field — direct write into the loaned SHM fixed section.
        proxy.score = 0.875;
        // Variable field — loan + copy into SHM.
        proxy.set_class_id("traffic_light").expect("set_class_id");
    }

    let (recovered_id, recovered_score) = subscriber
        .try_view::<vision_msgs::ObjectHypothesis, _>(|view| {
            (
                view.class_id().expect("class_id utf8").to_string(),
                view.score,
            )
        })
        .expect("try_view")
        .expect("subscriber should observe the frame");

    assert_eq!(recovered_id, "traffic_light");
    assert_eq!(recovered_score, 0.875);
}

/// `vision_msgs/Detection3DArray` is the headline detection-pipeline type
/// the benchmark graphs publish. Its two variable fields (`header`,
/// `detections` — a nested `Detection3D[]`, both raw-bytes under the
/// nested-as-bytes representation) are written in declaration
/// order and the `detections` payload read back byte-for-byte.
#[test]
fn vision_detection3d_array_variable_round_trip() {
    // Explicit contract pin: `header` + `detections` are the two variable
    // fields (the nested `Detection3D[]` is one nested-as-bytes region).
    assert_eq!(vision_msgs::Detection3DArray::VARIABLE_FIELD_COUNT, 2);

    let (_tt, mut publisher, mut subscriber) = make_pub_sub("det3d_array");

    let detections_payload: Vec<u8> = (0u8..64).collect();

    {
        let mut proxy = publisher
            .loan_proxy::<vision_msgs::Detection3DArray>()
            .expect("loan_proxy");
        // Write every variable field (publish-on-Drop requires all of them).
        proxy
            .set_header_bytes(&[0xDE, 0xAD])
            .expect("set_header_bytes");
        proxy
            .set_detections_bytes(&detections_payload)
            .expect("set_detections_bytes");
    }

    let recovered = subscriber
        .try_view::<vision_msgs::Detection3DArray, _>(|view| view.detections_bytes().to_vec())
        .expect("try_view")
        .expect("subscriber should observe the frame");

    assert_eq!(recovered, detections_payload);
}

// =====================================================================
// radar_msgs
// =====================================================================

/// `radar_msgs/RadarReturn` is a pure-primitive (5 × float32) FIXED schema
/// — it must classify as fixed (`VARIABLE_FIELD_COUNT == 0`) and round-trip
/// its scalars through a direct loaned-section write.
#[test]
fn radar_return_is_fixed_and_round_trips_scalars() {
    assert_eq!(variable_field_count::<radar_msgs::RadarReturn>(), 0);

    let (_tt, mut publisher, mut subscriber) = make_pub_sub("radar_return");

    {
        let mut proxy = publisher
            .loan_proxy::<radar_msgs::RadarReturn>()
            .expect("loan_proxy");
        proxy.range = 12.5;
        proxy.azimuth = -0.25;
        proxy.elevation = 0.0;
        proxy.doppler_velocity = 3.5;
        proxy.amplitude = -42.0;
    }

    let (range, azimuth, doppler, amplitude) = subscriber
        .try_view::<radar_msgs::RadarReturn, _>(|view| {
            (
                view.range,
                view.azimuth,
                view.doppler_velocity,
                view.amplitude,
            )
        })
        .expect("try_view")
        .expect("subscriber should observe the frame");

    assert_eq!(range, 12.5);
    assert_eq!(azimuth, -0.25);
    assert_eq!(doppler, 3.5);
    assert_eq!(amplitude, -42.0);
}

/// `radar_msgs/RadarTracks` is the headline radar-pipeline output type the
/// benchmark graphs publish (`std_msgs/Header` + `RadarTrack[]`). Round-trip
/// its variable `tracks` payload (raw bytes) through loan/fill.
#[test]
fn radar_tracks_variable_round_trip() {
    // Explicit contract pin: `header` + `tracks` are the two variable fields
    // (the nested `RadarTrack[]` is one nested-as-bytes region).
    assert_eq!(radar_msgs::RadarTracks::VARIABLE_FIELD_COUNT, 2);

    let (_tt, mut publisher, mut subscriber) = make_pub_sub("radar_tracks");

    let tracks_payload: Vec<u8> = vec![0x11, 0x22, 0x33, 0x44, 0x55];

    {
        let mut proxy = publisher
            .loan_proxy::<radar_msgs::RadarTracks>()
            .expect("loan_proxy");
        proxy.set_header_bytes(&[0x01]).expect("set_header_bytes");
        proxy
            .set_tracks_bytes(&tracks_payload)
            .expect("set_tracks_bytes");
    }

    let recovered = subscriber
        .try_view::<radar_msgs::RadarTracks, _>(|view| view.tracks_bytes().to_vec())
        .expect("try_view")
        .expect("subscriber should observe the frame");

    assert_eq!(recovered, tracks_payload);
}

// =====================================================================
// grid_map_msgs
// =====================================================================

/// `grid_map_msgs/GridMap` is the headline grid-map type (4 variable fields:
/// `header`, `layers`, `basic_layers`, `data`). Write all four in declaration
/// order and read back two DISTINCT payloads (`layers`, `data`) — proving
/// the per-field offset table routes each variable region correctly (a
/// sibling-swap regression would surface as crossed payloads).
#[test]
fn grid_map_multi_variable_round_trip() {
    // Explicit contract pin: `header`, `layers`, `basic_layers`, `data` are the
    // four variable fields (the two `uint16` start-index tail fields are fixed).
    assert_eq!(grid_map_msgs::GridMap::VARIABLE_FIELD_COUNT, 4);

    let (_tt, mut publisher, mut subscriber) = make_pub_sub("grid_map");

    let layers_payload: Vec<u8> = vec![0xAA; 6];
    let data_payload: Vec<u8> = (100u8..148).collect();

    {
        let mut proxy = publisher
            .loan_proxy::<grid_map_msgs::GridMap>()
            .expect("loan_proxy");
        proxy.set_header_bytes(&[0x07]).expect("set_header_bytes");
        proxy
            .set_layers_bytes(&layers_payload)
            .expect("set_layers_bytes");
        proxy
            .set_basic_layers_bytes(&[0xBB, 0xBB])
            .expect("set_basic_layers_bytes");
        proxy.set_data_bytes(&data_payload).expect("set_data_bytes");
    }

    let (recovered_layers, recovered_data) = subscriber
        .try_view::<grid_map_msgs::GridMap, _>(|view| {
            (view.layers_bytes().to_vec(), view.data_bytes().to_vec())
        })
        .expect("try_view")
        .expect("subscriber should observe the frame");

    assert_eq!(recovered_layers, layers_payload);
    assert_eq!(recovered_data, data_payload);
    // The two payloads are distinct, so crossed offset entries would fail
    // at least one of the asserts above.
    assert_ne!(recovered_layers, recovered_data);
}

/// After the upstream-faithful header move, `grid_map_msgs/GridMapInfo` carries
/// NO variable fields (`resolution`/`length_x`/`length_y` floats + a fixed
/// `geometry_msgs/Pose`) — it must classify as fixed (`VARIABLE_FIELD_COUNT ==
/// 0`) and round-trip its scalars through a direct loaned-section write. This
/// pins the contract created by moving `header` onto `GridMap`.
#[test]
fn grid_map_info_is_fixed_and_round_trips_scalars() {
    assert_eq!(variable_field_count::<grid_map_msgs::GridMapInfo>(), 0);

    let (_tt, mut publisher, mut subscriber) = make_pub_sub("grid_map_info");

    {
        let mut proxy = publisher
            .loan_proxy::<grid_map_msgs::GridMapInfo>()
            .expect("loan_proxy");
        proxy.resolution = 0.05;
        proxy.length_x = 10.0;
        proxy.length_y = 20.0;
    }

    let (resolution, length_x, length_y) = subscriber
        .try_view::<grid_map_msgs::GridMapInfo, _>(|view| {
            (view.resolution, view.length_x, view.length_y)
        })
        .expect("try_view")
        .expect("subscriber should observe the frame");

    assert_eq!(resolution, 0.05);
    assert_eq!(length_x, 10.0);
    assert_eq!(length_y, 20.0);
}

// =====================================================================
// Schema-hash distinctness
// =====================================================================

/// Every newly vendored type must hash distinctly on the wire. This is the
/// regression guard for the recipe-3 (package-qualified, layout-sensitive)
/// schema hash: in particular, `vision_msgs/Pose2D` and the pre-existing
/// `geometry_msgs/Pose2D` share a bare NAME but MUST NOT share a wire hash
/// (they have different layouts AND different packages).
#[test]
fn vendored_schema_hashes_are_pairwise_distinct() {
    use native_ros2_messages::geometry_msgs;
    use native_ros2_messages::unique_identifier_msgs;

    let hashes: &[(&str, u64)] = &[
        // vision_msgs
        (
            "vision_msgs/ObjectHypothesis",
            vision_msgs::ObjectHypothesis::SCHEMA_HASH,
        ),
        (
            "vision_msgs/ObjectHypothesisWithPose",
            vision_msgs::ObjectHypothesisWithPose::SCHEMA_HASH,
        ),
        ("vision_msgs/Point2D", vision_msgs::Point2D::SCHEMA_HASH),
        ("vision_msgs/Pose2D", vision_msgs::Pose2D::SCHEMA_HASH),
        (
            "vision_msgs/BoundingBox2D",
            vision_msgs::BoundingBox2D::SCHEMA_HASH,
        ),
        (
            "vision_msgs/BoundingBox2DArray",
            vision_msgs::BoundingBox2DArray::SCHEMA_HASH,
        ),
        (
            "vision_msgs/BoundingBox3D",
            vision_msgs::BoundingBox3D::SCHEMA_HASH,
        ),
        (
            "vision_msgs/BoundingBox3DArray",
            vision_msgs::BoundingBox3DArray::SCHEMA_HASH,
        ),
        (
            "vision_msgs/Detection2D",
            vision_msgs::Detection2D::SCHEMA_HASH,
        ),
        (
            "vision_msgs/Detection2DArray",
            vision_msgs::Detection2DArray::SCHEMA_HASH,
        ),
        (
            "vision_msgs/Detection3D",
            vision_msgs::Detection3D::SCHEMA_HASH,
        ),
        (
            "vision_msgs/Detection3DArray",
            vision_msgs::Detection3DArray::SCHEMA_HASH,
        ),
        (
            "vision_msgs/Classification",
            vision_msgs::Classification::SCHEMA_HASH,
        ),
        (
            "vision_msgs/VisionClass",
            vision_msgs::VisionClass::SCHEMA_HASH,
        ),
        (
            "vision_msgs/VisionInfo",
            vision_msgs::VisionInfo::SCHEMA_HASH,
        ),
        ("vision_msgs/LabelInfo", vision_msgs::LabelInfo::SCHEMA_HASH),
        // radar_msgs
        (
            "radar_msgs/RadarReturn",
            radar_msgs::RadarReturn::SCHEMA_HASH,
        ),
        ("radar_msgs/RadarScan", radar_msgs::RadarScan::SCHEMA_HASH),
        ("radar_msgs/RadarTrack", radar_msgs::RadarTrack::SCHEMA_HASH),
        (
            "radar_msgs/RadarTracks",
            radar_msgs::RadarTracks::SCHEMA_HASH,
        ),
        // grid_map_msgs
        ("grid_map_msgs/GridMap", grid_map_msgs::GridMap::SCHEMA_HASH),
        (
            "grid_map_msgs/GridMapInfo",
            grid_map_msgs::GridMapInfo::SCHEMA_HASH,
        ),
        // unique_identifier_msgs (transitive dep)
        (
            "unique_identifier_msgs/UUID",
            unique_identifier_msgs::UUID::SCHEMA_HASH,
        ),
    ];

    for i in 0..hashes.len() {
        for j in (i + 1)..hashes.len() {
            assert_ne!(
                hashes[i].1, hashes[j].1,
                "schema hash collision between {} and {}",
                hashes[i].0, hashes[j].0
            );
        }
    }

    // Bare-name collision guard: same bare name "Pose2D", distinct
    // packages, distinct wire hashes. NOTE: these two ALSO differ in their
    // field layout (vision_msgs/Pose2D = Point2D + theta; geometry_msgs/Pose2D
    // = x, y, theta), so distinctness HERE does not by itself isolate
    // package-qualification — a package-BLIND but layout-sensitive recipe
    // would also tell them apart. It still earns its keep as a cheap
    // regression guard that the two never alias on the wire. (The
    // package-qualification contract itself is exercised by build.rs's
    // duplicate-name resolution + the pairwise-distinctness sweep above.)
    assert_ne!(
        vision_msgs::Pose2D::SCHEMA_HASH,
        geometry_msgs::Pose2D::SCHEMA_HASH,
        "vision_msgs/Pose2D must not collide on the wire with geometry_msgs/Pose2D"
    );
}

/// Tiny generic helper so the fixed-schema assertion reads cleanly.
fn variable_field_count<T: ShmMessage>() -> usize {
    T::VARIABLE_FIELD_COUNT
}
