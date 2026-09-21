// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end round-trip coverage for every ROS2 message schema under the
//! unified SHM-backed wire format.
//!
//! The suite asserts a single
//! property: every schema must round-trip a representative `<Name>Snapshot`
//! value through `loan_proxy<T>() + write_from_snapshot()` on the publisher
//! side and `try_view::<T, _>(|view| view.snapshot())` on the subscriber
//! side, with the recovered snapshot equal to the original.
//!
//! All 254 schemas are covered (61 fixed + 193 variable), including the
//! vendored MoveIt packages (moveit_msgs, control_msgs,
//! octomap_msgs, object_recognition_msgs), the vendored
//! perception packages (vision_msgs, radar_msgs, grid_map_msgs,
//! unique_identifier_msgs), and the vendored Autoware
//! packages (autoware_perception_msgs, autoware_planning_msgs). Per-schema test
//! cases use the `roundtrip_fixed!` / `roundtrip_var!` macros below; the
//! shared body lives in [`assert_round_trip_fixed`] and
//! [`assert_round_trip_var`]. Each test builds a private
//! [`cerulion_core::testing::TestTransport`] with its own isolated
//! iceoryx2 SHM root, so the parallel-test runner can dispatch all 254 without
//! contention or `--test-threads=1` gating — even though every test reuses the
//! same topic shape, the per-call SHM isolation keeps them independent.
//!
//! # Snapshot construction policy
//!
//! Snapshots are built from `Default::default()` plus
//! a small handful of field overrides aimed at catching basic field-routing
//! bugs (a wrong getter returning a sibling field's value, an off-by-one in
//! the offset table, etc.). Hand-crafting deeply-populated nested structures
//! across 254 schemas is explicitly out of scope — the round-trip property
//! holds for default-valued snapshots too, and dedicated tests already cover the
//! "non-default values flow through correctly" property for representative
//! schemas.
//!
//! # Schemas without serde derives
//!
//! There are none. Any schema that (transitively)
//! contains a fixed array larger than 32 elements — the upper bound of
//! `serde`'s blanket impls — cannot derive
//! `Serialize` / `Deserialize` with a plain derive: the three direct large-array schemas
//! (`geometry_msgs::{Pose,Twist,Accel}WithCovariance`) and, because
//! nested resolution makes the poisoning transitive, every schema embedding one
//! (`nav_msgs::Odometry`, the `*WithCovarianceStamped` trio,
//! `autoware_perception_msgs::PredictedObjectKinematics`,
//! `vision_msgs::ObjectHypothesisWithPose`) — nine types in all.
//!
//! Those fields route through `cerulion_core::codegen::big_array` via a
//! per-field `#[serde(with = ...)]`, so the whole corpus derives serde
//! uniformly. `tests/serde_corpus_test.rs` is the gate.
//!
//! This suite does not depend on that: the round-trip property it tests needs
//! only `Default + Clone + PartialEq`, so every schema participates identically
//! with or without serde.

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::SystemTime;

use cerulion_core::message::ShmMessage;
use cerulion_core::testing::TestTransport;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::wire::MaxSliceLen;

use native_ros2_messages::{
    action_msgs, autoware_perception_msgs, autoware_planning_msgs, builtin_interfaces,
    control_msgs, diagnostic_msgs, geometry_msgs, grid_map_msgs, moveit_msgs, nav_msgs,
    object_recognition_msgs, octomap_msgs, radar_msgs, sensor_msgs, shape_msgs, statistics_msgs,
    std_msgs, tf2_msgs, trajectory_msgs, unique_identifier_msgs, vision_msgs, visualization_msgs,
};

// =============================================================
// Test infrastructure
// =============================================================

/// Per-test buffer big enough for every schema in the suite.
///
/// The largest schemas (`Image`, `PointCloud2`, `Marker`) only carry empty
/// `Vec`s in their default snapshots, so the overhead is just header +
/// fixed-section + offset table for the variable-field count of the schema.
/// 16 KiB comfortably covers all 254 schemas including future growth.
const MAX_SLICE_LEN: MaxSliceLen = MaxSliceLen::const_new(16 * 1024);

/// Subscriber channel capacity. Each test publishes exactly one frame,
/// so 4 is generous.
const SUB_CAPACITY: usize = 4;

/// Monotonic counter so parallel-test topic names are distinct. Each test
/// also gets its own isolated iceoryx2 SHM root via `TestTransport`, so
/// cross-test collisions are already impossible — the counter is belt-and-
/// suspenders plus trace clarity.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(label: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/roundtrip/{label}/{nanos}/{id}")
}

/// Build an isolated iceoryx2 publisher + matching subscriber on a unique
/// topic. The returned
/// `TestTransport` owns the iceoryx2 node and MUST be kept in scope so the
/// publisher/subscriber stay valid — each caller binds it to a local that
/// lives to the end of the test. Per-call isolated SHM root → parallel-safe,
/// no `#[serial]` needed even though every test reuses the same topic shape.
fn make_pub_sub(label: &str) -> (TestTransport, CerulionPublisher, CerulionSubscriber) {
    let topic = unique_topic(label);
    let tt = TestTransport::with_buffer_size(SUB_CAPACITY);
    let publisher = tt.publisher(&topic, MAX_SLICE_LEN, 0);
    let subscriber = tt.subscriber(&topic);
    (tt, publisher, subscriber)
}

/// Round-trip helper for fixed schemas (`VARIABLE_FIELD_COUNT == 0`).
///
/// `T` is the unit-marker (e.g. `geometry_msgs::Vector3`); `Snap` is the
/// matching `<Name>Snapshot` companion. The fixed-schema
/// `write_from_snapshot` returns `()` (no Result) and the writer is a
/// `&mut <Name>Shm` plain-data type — no fallible setters in the chain.
fn assert_round_trip_fixed<T, Snap, F>(label: &str, mutate: F)
where
    T: ShmMessage,
    // Note: codegen omits `Debug` from the three large-array snapshots
    // (`PoseWithCovariance`, `TwistWithCovariance`, `AccelWithCovariance`)
    // because their fixed arrays exceed the 32-element `Debug`-blanket cap.
    // Keep the bound minimal so the same helper covers all 254 schemas.
    Snap: Default + Clone + PartialEq,
    // The Writer<'a> for fixed schemas is `&'a mut <Name>Shm`, which carries
    // an inherent `write_from_snapshot(&mut self, &Snap) -> ()` method. We
    // reach it through the OutputProxy's `Deref` to `Writer<'a>`. The
    // `snapshot()` method lives on `&<Name>Shm` and returns `Snap` by value.
    F: FnOnce(&mut Snap),
    for<'a> T::Writer<'a>: WriteFromSnapshot<Snap>,
    for<'a> T::Reader<'a>: ToSnapshot<Snap>,
{
    let mut snap = Snap::default();
    mutate(&mut snap);
    let original = snap.clone();

    let (_tt, mut publisher, mut subscriber) = make_pub_sub(label);

    {
        let mut proxy = publisher.loan_proxy::<T>().expect("loan_proxy");
        // Deref to `&mut <Name>Shm` and invoke the inherent helper.
        proxy.write_from_snapshot(&snap);
    }

    let recovered: Snap = subscriber
        .try_view::<T, _>(|view| view.to_snapshot())
        .expect("try_view")
        .expect("subscriber should observe the published frame");

    // `assert!` (not `assert_eq!`) keeps the `Debug` bound out of the
    // helper's signature — three large-array snapshots lack `Debug`.
    assert!(recovered == original, "round-trip mismatch for {label}",);
}

/// Round-trip helper for variable schemas (`VARIABLE_FIELD_COUNT > 0`).
///
/// Same shape as [`assert_round_trip_fixed`] but the fallible
/// `write_from_snapshot` is propagated. Variable-schema setters can fail if
/// the loan buffer is too small for the snapshot payload; with the chosen
/// `MAX_SLICE_LEN` and default-valued snapshots, this never happens in the
/// suite.
fn assert_round_trip_var<T, Snap, F>(label: &str, mutate: F)
where
    T: ShmMessage,
    // Note: codegen omits `Debug` from the three large-array snapshots
    // (`PoseWithCovariance`, `TwistWithCovariance`, `AccelWithCovariance`)
    // because their fixed arrays exceed the 32-element `Debug`-blanket cap.
    // Keep the bound minimal so the same helper covers all 254 schemas.
    Snap: Default + Clone + PartialEq,
    F: FnOnce(&mut Snap),
    for<'a> T::Writer<'a>: TryWriteFromSnapshot<Snap>,
    for<'a> T::Reader<'a>: ToSnapshot<Snap>,
{
    let mut snap = Snap::default();
    mutate(&mut snap);
    let original = snap.clone();

    let (_tt, mut publisher, mut subscriber) = make_pub_sub(label);

    {
        let mut proxy = publisher.loan_proxy::<T>().expect("loan_proxy");
        proxy
            .write_from_snapshot(&snap)
            .expect("write_from_snapshot");
    }

    let recovered: Snap = subscriber
        .try_view::<T, _>(|view| view.to_snapshot())
        .expect("try_view")
        .expect("subscriber should observe the published frame");

    assert!(recovered == original, "round-trip mismatch for {label}",);
}

// -------------------------------------------------------------
// Adapter traits to bridge the inherent `write_from_snapshot` /
// `snapshot` methods into a generic call site. Codegen emits these
// methods directly on `<Name>Shm` (and the `&mut <Name>Shm` /
// `&<Name>Shm` reference types it uses for fixed schemas), but they
// are not part of any common trait — so the helpers above need a
// thin shim. The shim is implemented per-schema by the
// `roundtrip_fixed!` / `roundtrip_var!` macros.
// -------------------------------------------------------------

/// Bridges `<Name>Shm::write_from_snapshot(&Snap)` for fixed schemas.
trait WriteFromSnapshot<Snap> {
    fn write_from_snapshot(&mut self, snap: &Snap);
}

/// Bridges `<Name>Shm::write_from_snapshot(&Snap) -> Result<(), _>` for
/// variable schemas.
trait TryWriteFromSnapshot<Snap> {
    fn write_from_snapshot(&mut self, snap: &Snap) -> Result<(), cerulion_core::TransportError>;
}

/// Bridges `<Name>Shm::snapshot(&self) -> Snap` for both kinds.
trait ToSnapshot<Snap> {
    fn to_snapshot(&self) -> Snap;
}

// =============================================================
// Per-schema test cases
//
// Conventions:
//
// * `roundtrip_fixed!(module, SchemaName, |s| { ... })` — fixed schema.
// * `roundtrip_var!(module, SchemaName, |s| { ... })` — variable schema.
// * The body mutates the default-valued `<Name>Snapshot`. For schemas
//   where every field is non-trivially constructible (e.g. nested types
//   stored as `Vec<u8>`, the nested-as-bytes representation), an empty body is OK —
//   default-valued round-trip still asserts the codegen plumbing works.
// =============================================================

/// Generate one round-trip `#[test]` for a fixed-layout schema.
///
/// Expands to:
/// 1. The `WriteFromSnapshot` / `ToSnapshot` shim impls bridging the
///    inherent codegen methods into the generic helper signature.
/// 2. A `#[test]` fn named `roundtrip_<module>_<schema>` that builds a
///    snapshot, applies the user mutator, and asserts round-trip.
macro_rules! roundtrip_fixed {
    ($module:ident, $schema:ident $(, |$s:ident| $body:block)?) => {
        paste::paste! {
            impl WriteFromSnapshot<$module::[<$schema Snapshot>]>
                for &mut $module::[<$schema Shm>]
            {
                fn write_from_snapshot(
                    &mut self,
                    snap: &$module::[<$schema Snapshot>],
                ) {
                    (**self).write_from_snapshot(snap);
                }
            }
            impl ToSnapshot<$module::[<$schema Snapshot>]>
                for &$module::[<$schema Shm>]
            {
                fn to_snapshot(&self) -> $module::[<$schema Snapshot>] {
                    (**self).snapshot()
                }
            }

            #[test]
            fn [<roundtrip_ $module _ $schema:snake>]() {
                #[allow(unused_mut, unused_variables, clippy::redundant_closure_call)]
                assert_round_trip_fixed::<$module::$schema, $module::[<$schema Snapshot>], _>(
                    concat!(stringify!($module), "::", stringify!($schema)),
                    |s| { $(let $s = s; $body)? },
                );
            }
        }
    };
}

/// Generate one round-trip `#[test]` for a variable-layout schema.
///
/// Variable schemas use opaque `<Name>Shm<'a>` (lifetime-parameterised) for
/// both reader and writer, so the shim impls are over the lifetime-tagged
/// type rather than the `&mut`/`&` references used by fixed schemas.
macro_rules! roundtrip_var {
    ($module:ident, $schema:ident $(, |$s:ident| $body:block)?) => {
        paste::paste! {
            impl<'a> TryWriteFromSnapshot<$module::[<$schema Snapshot>]>
                for $module::[<$schema Shm>]<'a>
            {
                fn write_from_snapshot(
                    &mut self,
                    snap: &$module::[<$schema Snapshot>],
                ) -> Result<(), cerulion_core::TransportError> {
                    $module::[<$schema Shm>]::write_from_snapshot(self, snap)
                }
            }
            impl<'a> ToSnapshot<$module::[<$schema Snapshot>]>
                for $module::[<$schema Shm>]<'a>
            {
                fn to_snapshot(&self) -> $module::[<$schema Snapshot>] {
                    $module::[<$schema Shm>]::snapshot(self)
                }
            }

            #[test]
            fn [<roundtrip_ $module _ $schema:snake>]() {
                #[allow(unused_mut, unused_variables, clippy::redundant_closure_call)]
                assert_round_trip_var::<$module::$schema, $module::[<$schema Snapshot>], _>(
                    concat!(stringify!($module), "::", stringify!($schema)),
                    |s| { $(let $s = s; $body)? },
                );
            }
        }
    };
}

// -------------------------------------------------------------
// builtin_interfaces (2 fixed)
// -------------------------------------------------------------
roundtrip_fixed!(builtin_interfaces, Time, |s| {
    s.sec = 1_704_067_200;
    s.nanosec = 500_000_000;
});
roundtrip_fixed!(builtin_interfaces, Duration, |s| {
    s.sec = -10;
    s.nanosec = 100;
});

// -------------------------------------------------------------
// std_msgs (30: 16 fixed + 14 variable)
// -------------------------------------------------------------
roundtrip_fixed!(std_msgs, Bool, |s| {
    s.data = true;
});
roundtrip_fixed!(std_msgs, Byte, |s| {
    s.data = 7;
});
roundtrip_fixed!(std_msgs, Char, |s| {
    s.data = 42;
});
roundtrip_fixed!(std_msgs, ColorRGBA, |s| {
    s.r = 0.25;
    s.g = 0.5;
    s.b = 0.75;
    s.a = 1.0;
});
roundtrip_fixed!(std_msgs, Empty);
roundtrip_fixed!(std_msgs, Float32, |s| {
    s.data = 1.5;
});
roundtrip_fixed!(std_msgs, Float64, |s| {
    s.data = 2.5;
});
roundtrip_fixed!(std_msgs, Int8, |s| {
    s.data = -8;
});
roundtrip_fixed!(std_msgs, Int16, |s| {
    s.data = -16;
});
roundtrip_fixed!(std_msgs, Int32, |s| {
    s.data = -32;
});
roundtrip_fixed!(std_msgs, Int64, |s| {
    s.data = -64;
});
roundtrip_fixed!(std_msgs, UInt8, |s| {
    s.data = 8;
});
roundtrip_fixed!(std_msgs, UInt16, |s| {
    s.data = 16;
});
roundtrip_fixed!(std_msgs, UInt32, |s| {
    s.data = 32;
});
roundtrip_fixed!(std_msgs, UInt64, |s| {
    s.data = 64;
});

// MultiArrayDimension is variable due to `label: String`.
roundtrip_var!(std_msgs, MultiArrayDimension, |s| {
    s.label = "axis_x".into();
    s.size = 3;
    s.stride = 12;
});
// MultiArrayLayout's `dim` field is DynamicArray<Nested>, stored as
// raw Vec<u8> in the Snapshot (nested-as-bytes). Default => empty vec.
roundtrip_var!(std_msgs, MultiArrayLayout, |s| {
    s.data_offset = 5;
});
roundtrip_var!(std_msgs, Header);
roundtrip_var!(std_msgs, String, |s| {
    s.data = "hello".into();
});
roundtrip_var!(std_msgs, Int8MultiArray, |s| {
    s.data = vec![1, 2, 3];
});
roundtrip_var!(std_msgs, Int16MultiArray, |s| {
    s.data = vec![1, 2, 3];
});
roundtrip_var!(std_msgs, Int32MultiArray, |s| {
    s.data = vec![1, 2, 3];
});
roundtrip_var!(std_msgs, Int64MultiArray, |s| {
    s.data = vec![1, 2, 3];
});
roundtrip_var!(std_msgs, UInt8MultiArray, |s| {
    s.data = vec![1, 2, 3];
});
roundtrip_var!(std_msgs, UInt16MultiArray, |s| {
    s.data = vec![1, 2, 3];
});
roundtrip_var!(std_msgs, UInt32MultiArray, |s| {
    s.data = vec![1, 2, 3];
});
roundtrip_var!(std_msgs, UInt64MultiArray, |s| {
    s.data = vec![1, 2, 3];
});
roundtrip_var!(std_msgs, Float32MultiArray, |s| {
    s.data = vec![1.0, 2.0];
});
roundtrip_var!(std_msgs, Float64MultiArray, |s| {
    s.data = vec![1.0, 2.0];
});
roundtrip_var!(std_msgs, ByteMultiArray, |s| {
    s.data = vec![1, 2, 3];
});

// -------------------------------------------------------------
// geometry_msgs (32: 5 fixed + 27 variable)
// -------------------------------------------------------------
roundtrip_fixed!(geometry_msgs, Point, |s| {
    s.x = 1.0;
    s.y = 2.0;
    s.z = 3.0;
});
roundtrip_fixed!(geometry_msgs, Point32, |s| {
    s.x = 1.0;
    s.y = 2.0;
    s.z = 3.0;
});
roundtrip_fixed!(geometry_msgs, Vector3, |s| {
    s.x = 1.0;
    s.y = 2.0;
    s.z = 3.0;
});
roundtrip_fixed!(geometry_msgs, Quaternion, |s| {
    s.w = 1.0;
});
roundtrip_fixed!(geometry_msgs, Pose2D, |s| {
    s.x = 1.0;
    s.y = 2.0;
    s.theta = 0.5;
});

// All Stamped/composite types are variable because of nested-Header /
// nested-Vector3 etc. Their Snapshot fields are raw Vec<u8> (nested-as-bytes).
roundtrip_fixed!(geometry_msgs, Accel);
roundtrip_var!(geometry_msgs, AccelStamped);
roundtrip_fixed!(geometry_msgs, AccelWithCovariance);
roundtrip_var!(geometry_msgs, AccelWithCovarianceStamped);
roundtrip_fixed!(geometry_msgs, Inertia);
roundtrip_var!(geometry_msgs, InertiaStamped);
roundtrip_var!(geometry_msgs, PointStamped);
roundtrip_var!(geometry_msgs, Polygon);
roundtrip_var!(geometry_msgs, PolygonInstance, |s| {
    s.id = 7;
});
roundtrip_var!(geometry_msgs, PolygonInstanceStamped);
roundtrip_var!(geometry_msgs, PolygonStamped);
roundtrip_fixed!(geometry_msgs, Pose);
roundtrip_var!(geometry_msgs, PoseArray);
roundtrip_var!(geometry_msgs, PoseStamped);
roundtrip_fixed!(geometry_msgs, PoseWithCovariance);
roundtrip_var!(geometry_msgs, PoseWithCovarianceStamped);
roundtrip_var!(geometry_msgs, QuaternionStamped);
roundtrip_fixed!(geometry_msgs, Transform);
roundtrip_var!(geometry_msgs, TransformStamped);
roundtrip_fixed!(geometry_msgs, Twist);
roundtrip_var!(geometry_msgs, TwistStamped);
roundtrip_fixed!(geometry_msgs, TwistWithCovariance);
roundtrip_var!(geometry_msgs, TwistWithCovarianceStamped);
roundtrip_var!(geometry_msgs, Vector3Stamped);
roundtrip_var!(geometry_msgs, VelocityStamped);
roundtrip_fixed!(geometry_msgs, Wrench);
roundtrip_var!(geometry_msgs, WrenchStamped);

// -------------------------------------------------------------
// sensor_msgs (27: 3 fixed + 24 variable)
// -------------------------------------------------------------
roundtrip_fixed!(sensor_msgs, JoyFeedback, |s| {
    s.r#type = 1;
    s.id = 2;
    s.intensity = 0.5;
});
roundtrip_fixed!(sensor_msgs, NavSatStatus, |s| {
    s.status = 1;
});
roundtrip_fixed!(sensor_msgs, RegionOfInterest, |s| {
    s.x_offset = 10;
    s.y_offset = 20;
    s.height = 480;
    s.width = 640;
    s.do_rectify = true;
});

roundtrip_var!(sensor_msgs, BatteryState, |s| {
    s.voltage = 12.5;
});
roundtrip_var!(sensor_msgs, CameraInfo, |s| {
    s.width = 640;
    s.height = 480;
    s.distortion_model = "plumb_bob".into();
});
roundtrip_var!(sensor_msgs, ChannelFloat32, |s| {
    s.name = "intensity".into();
    s.values = vec![1.0, 2.0];
});
roundtrip_var!(sensor_msgs, CompressedImage, |s| {
    s.format = "jpeg".into();
    s.data = vec![0xFF, 0xD8];
});
roundtrip_var!(sensor_msgs, FluidPressure, |s| {
    s.fluid_pressure = 101_325.0;
    s.variance = 1.0;
});
roundtrip_var!(sensor_msgs, Illuminance, |s| {
    s.illuminance = 500.0;
    s.variance = 0.5;
});
roundtrip_var!(sensor_msgs, Image, |s| {
    s.height = 32;
    s.width = 32;
    s.encoding = "rgb8".into();
    s.is_bigendian = 0;
    s.step = 96;
    s.data = vec![1, 2, 3, 4];
});
roundtrip_var!(sensor_msgs, Imu);
roundtrip_var!(sensor_msgs, JointState, |s| {
    s.name = vec![];
    s.position = vec![1.0, 2.0];
});
roundtrip_var!(sensor_msgs, Joy, |s| {
    s.axes = vec![0.5, -0.5];
    s.buttons = vec![0, 1];
});
roundtrip_var!(sensor_msgs, JoyFeedbackArray);
roundtrip_var!(sensor_msgs, LaserEcho, |s| {
    s.echoes = vec![1.0, 2.0];
});
roundtrip_var!(sensor_msgs, LaserScan, |s| {
    s.angle_min = -1.57;
    s.angle_max = 1.57;
    s.angle_increment = 0.01;
    s.ranges = vec![1.0, 2.0, 3.0];
    s.intensities = vec![100.0, 200.0, 300.0];
});
roundtrip_var!(sensor_msgs, MagneticField);
roundtrip_var!(sensor_msgs, MultiDOFJointState);
roundtrip_var!(sensor_msgs, MultiEchoLaserScan);
roundtrip_var!(sensor_msgs, NavSatFix, |s| {
    s.latitude = 37.7749;
    s.longitude = -122.4194;
    s.altitude = 16.0;
});
roundtrip_var!(sensor_msgs, PointCloud);
roundtrip_var!(sensor_msgs, PointCloud2, |s| {
    s.height = 1;
    s.width = 100;
    s.point_step = 16;
    s.row_step = 1600;
    s.is_dense = true;
});
roundtrip_var!(sensor_msgs, PointField, |s| {
    s.name = "x".into();
    s.offset = 0;
    s.datatype = 7;
    s.count = 1;
});
roundtrip_var!(sensor_msgs, Range, |s| {
    s.radiation_type = 0;
    s.field_of_view = 0.44;
    s.min_range = 0.05;
    s.max_range = 3.0;
    s.range = 1.5;
});
roundtrip_var!(sensor_msgs, RelativeHumidity, |s| {
    s.relative_humidity = 0.55;
    s.variance = 0.01;
});
roundtrip_var!(sensor_msgs, Temperature, |s| {
    s.temperature = 25.0;
    s.variance = 0.1;
});
roundtrip_var!(sensor_msgs, TimeReference, |s| {
    s.source = "gps".into();
});

// -------------------------------------------------------------
// nav_msgs (5 variable)
// -------------------------------------------------------------
roundtrip_var!(nav_msgs, GridCells, |s| {
    s.cell_width = 0.1;
    s.cell_height = 0.1;
});
roundtrip_fixed!(nav_msgs, MapMetaData, |s| {
    s.resolution = 0.05;
    s.width = 100;
    s.height = 100;
});
roundtrip_var!(nav_msgs, OccupancyGrid, |s| {
    s.data = vec![0, 50, 100];
});
roundtrip_var!(nav_msgs, Odometry);
roundtrip_var!(nav_msgs, Path);

// -------------------------------------------------------------
// visualization_msgs (12: 1 fixed + 11 variable)
// -------------------------------------------------------------
roundtrip_fixed!(visualization_msgs, UVCoordinate, |s| {
    s.u = 0.25;
    s.v = 0.75;
});
roundtrip_var!(visualization_msgs, ImageMarker, |s| {
    s.ns = "marker_ns".into();
    s.id = 1;
});
roundtrip_var!(visualization_msgs, InteractiveMarker, |s| {
    s.name = "im_1".into();
    s.description = "test".into();
    s.scale = 1.0;
});
roundtrip_var!(visualization_msgs, InteractiveMarkerControl, |s| {
    s.name = "ctrl_1".into();
});
roundtrip_var!(visualization_msgs, InteractiveMarkerFeedback, |s| {
    s.client_id = "rviz".into();
    s.marker_name = "im_1".into();
});
roundtrip_var!(visualization_msgs, InteractiveMarkerInit, |s| {
    s.server_id = "server_1".into();
    s.seq_num = 7;
});
roundtrip_var!(visualization_msgs, InteractiveMarkerPose, |s| {
    s.name = "im_1".into();
});
roundtrip_var!(visualization_msgs, InteractiveMarkerUpdate, |s| {
    s.server_id = "server_1".into();
    s.seq_num = 9;
});
roundtrip_var!(visualization_msgs, Marker, |s| {
    s.ns = "ns_1".into();
    s.id = 42;
    s.r#type = 0;
    s.action = 0;
});
roundtrip_var!(visualization_msgs, MarkerArray);
roundtrip_var!(visualization_msgs, MenuEntry, |s| {
    s.id = 1;
    s.parent_id = 0;
    s.title = "menu".into();
    s.command = "do_thing".into();
});
roundtrip_var!(visualization_msgs, MeshFile, |s| {
    s.filename = "robot.dae".into();
    s.data = vec![1, 2, 3];
});

// -------------------------------------------------------------
// trajectory_msgs (4 variable)
// -------------------------------------------------------------
roundtrip_var!(trajectory_msgs, JointTrajectory);
roundtrip_var!(trajectory_msgs, JointTrajectoryPoint, |s| {
    s.positions = vec![1.0, 2.0];
    s.velocities = vec![0.1, 0.2];
});
roundtrip_var!(trajectory_msgs, MultiDOFJointTrajectory);
roundtrip_var!(trajectory_msgs, MultiDOFJointTrajectoryPoint);

// -------------------------------------------------------------
// shape_msgs (4: 2 fixed + 2 variable)
// -------------------------------------------------------------
roundtrip_fixed!(shape_msgs, MeshTriangle, |s| {
    s.vertex_indices = [1, 2, 3];
});
roundtrip_fixed!(shape_msgs, Plane, |s| {
    s.coef = [1.0, 0.0, 0.0, 0.0];
});
roundtrip_var!(shape_msgs, Mesh);
roundtrip_var!(shape_msgs, SolidPrimitive, |s| {
    s.r#type = 1;
    s.dimensions = vec![1.0, 2.0, 3.0];
});

// -------------------------------------------------------------
// diagnostic_msgs (3 variable)
// -------------------------------------------------------------
roundtrip_var!(diagnostic_msgs, DiagnosticArray);
roundtrip_var!(diagnostic_msgs, DiagnosticStatus, |s| {
    s.level = 0;
    s.name = "battery".into();
    s.message = "ok".into();
    s.hardware_id = "0xCAFE".into();
});
roundtrip_var!(diagnostic_msgs, KeyValue, |s| {
    s.key = "voltage".into();
    s.value = "12.0".into();
});

// -------------------------------------------------------------
// statistics_msgs (3: 2 fixed + 1 variable)
// -------------------------------------------------------------
roundtrip_fixed!(statistics_msgs, StatisticDataPoint, |s| {
    s.data_type = 1;
    // 2.5 (not 3.14) — clippy's `approx_constant` flags 3.14 as
    // "use std::f64::consts::PI" which is overkill for a synthetic
    // round-trip value.
    s.data = 2.5;
});
// StatisticDataType has zero fields (constants-only schema). The Default
// snapshot is `{}` and round-trip is vacuously OK; codegen still emits
// the unit marker + ShmMessage impl so it must participate.
roundtrip_fixed!(statistics_msgs, StatisticDataType);
roundtrip_var!(statistics_msgs, MetricsMessage, |s| {
    s.measurement_source_name = "src".into();
    s.metrics_source = "cpu".into();
    s.unit = "percent".into();
});

// -------------------------------------------------------------
// action_msgs (3 variable)
// -------------------------------------------------------------
// `goal_id` is upstream's `unique_identifier_msgs/UUID` (a nested
// fixed message wrapping `uint8[16]`), not a bare `uint8[16]`.
roundtrip_fixed!(action_msgs, GoalInfo, |s| {
    s.goal_id.uuid = [1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
});
roundtrip_fixed!(action_msgs, GoalStatus, |s| {
    s.status = 4;
});
roundtrip_var!(action_msgs, GoalStatusArray);

// -------------------------------------------------------------
// tf2_msgs (2 variable)
// -------------------------------------------------------------
roundtrip_var!(tf2_msgs, TFMessage);
roundtrip_var!(tf2_msgs, TF2Error, |s| {
    s.error = 7;
    s.error_string = "transform unavailable".into();
});

// =============================================================
// MoveIt 2 message packages (vendored for rmw_cerulion)
// =============================================================

// -------------------------------------------------------------
// moveit_msgs (46: 7 fixed + 39 variable)
// -------------------------------------------------------------
roundtrip_var!(moveit_msgs, AllowedCollisionEntry, |s| {
    // Non-empty bool[]: exercises the bool-as-u8 storage + validated
    // &[bool] reader.
    s.enabled = vec![true, false, true, true, false];
});
roundtrip_var!(moveit_msgs, AllowedCollisionMatrix);
roundtrip_var!(moveit_msgs, AttachedCollisionObject);
roundtrip_var!(moveit_msgs, BoundingVolume);
roundtrip_fixed!(moveit_msgs, CartesianPoint, |s| {
    s.pose.position.x = 0.5;
    s.velocity.linear.y = 0.25;
});
roundtrip_var!(moveit_msgs, CartesianTrajectory);
roundtrip_fixed!(moveit_msgs, CartesianTrajectoryPoint);
roundtrip_var!(moveit_msgs, CollisionObject, |s| {
    s.id = "box_1".into();
    // MOVE (=3), NOT the default ADD (=0) — a default-valued override
    // documents coverage it doesn't provide.
    s.operation = 3;
});
roundtrip_fixed!(moveit_msgs, ConstraintEvalResult);
roundtrip_var!(moveit_msgs, Constraints);
roundtrip_var!(moveit_msgs, ContactInformation);
roundtrip_fixed!(moveit_msgs, CostSource);
roundtrip_var!(moveit_msgs, DisplayRobotState);
roundtrip_var!(moveit_msgs, DisplayTrajectory);
roundtrip_var!(moveit_msgs, GenericTrajectory);
roundtrip_var!(moveit_msgs, Grasp);
roundtrip_var!(moveit_msgs, GripperTranslation);
roundtrip_var!(moveit_msgs, JointConstraint, |s| {
    s.joint_name = "joint_1".into();
    s.position = 1.25;
    s.weight = 1.0;
});
roundtrip_var!(moveit_msgs, JointLimits, |s| {
    s.joint_name = "joint_2".into();
    s.has_position_limits = true;
    s.max_position = 3.5;
});
roundtrip_var!(moveit_msgs, KinematicSolverInfo);
roundtrip_var!(moveit_msgs, LinkPadding);
roundtrip_var!(moveit_msgs, LinkScale);
roundtrip_var!(moveit_msgs, MotionPlanDetailedResponse);
roundtrip_var!(moveit_msgs, MotionPlanRequest, |s| {
    s.group_name = "panda_arm".into();
    s.num_planning_attempts = 3;
    s.allowed_planning_time = 5.0;
    s.max_velocity_scaling_factor = 0.8;
});
roundtrip_var!(moveit_msgs, MotionPlanResponse);
roundtrip_var!(moveit_msgs, MotionSequenceItem);
roundtrip_var!(moveit_msgs, MotionSequenceRequest);
roundtrip_var!(moveit_msgs, MotionSequenceResponse);
// Upstream Jazzy's `string message` / `string source` make this
// schema VARIABLE, so it round-trips through the variable helper.
roundtrip_var!(moveit_msgs, MoveItErrorCodes, |s| {
    s.val = 1;
    s.message = "no ik solution".into();
    s.source = "kinematics".into();
});
roundtrip_var!(moveit_msgs, ObjectColor);
roundtrip_var!(moveit_msgs, OrientationConstraint);
roundtrip_fixed!(moveit_msgs, OrientedBoundingBox);
roundtrip_var!(moveit_msgs, PlaceLocation);
roundtrip_var!(moveit_msgs, PlannerInterfaceDescription);
roundtrip_var!(moveit_msgs, PlannerParams);
roundtrip_var!(moveit_msgs, PlanningOptions);
roundtrip_var!(moveit_msgs, PlanningScene, |s| {
    s.name = "demo_scene".into();
    s.is_diff = true;
    // DISTINCT non-empty bytes in raw-bytes siblings: catches
    // offset-table sibling swaps that all-empty defaults cannot.
    // Complex variable fields are Vec<u8> in
    // snapshots (the nested-as-bytes limitation).
    s.robot_state = vec![0xA1, 0xA2, 0xA3];
    s.world = vec![0xB1];
    s.allowed_collision_matrix = vec![0xC1, 0xC2];
});
roundtrip_fixed!(moveit_msgs, PlanningSceneComponents);
roundtrip_var!(moveit_msgs, PlanningSceneWorld);
roundtrip_var!(moveit_msgs, PositionConstraint);
roundtrip_var!(moveit_msgs, PositionIKRequest);
roundtrip_var!(moveit_msgs, RobotState);
roundtrip_var!(moveit_msgs, RobotTrajectory);
roundtrip_var!(moveit_msgs, TrajectoryConstraints);
roundtrip_var!(moveit_msgs, VisibilityConstraint);
roundtrip_var!(moveit_msgs, WorkspaceParameters, |s| {
    s.min_corner.x = -1.0;
    s.max_corner.x = 1.0;
});

// -------------------------------------------------------------
// control_msgs (39: 3 fixed + 36 variable)
// -------------------------------------------------------------
roundtrip_var!(control_msgs, AdmittanceControllerState);
roundtrip_var!(control_msgs, BatteryStateArray);
roundtrip_fixed!(control_msgs, CANopenState);
roundtrip_var!(control_msgs, DynamicInterfaceGroupValues);
roundtrip_var!(control_msgs, DynamicInterfaceValues);
roundtrip_var!(control_msgs, DynamicJointState);
roundtrip_var!(control_msgs, EtherCATState);
roundtrip_var!(control_msgs, Float64Values);
roundtrip_var!(control_msgs, GenericHardwareState);
roundtrip_fixed!(control_msgs, GripperCommand, |s| {
    s.position = 0.04;
    s.max_effort = 10.0;
});
roundtrip_var!(control_msgs, HardwareDeviceDiagnostics);
roundtrip_var!(control_msgs, HardwareDeviceStatus);
roundtrip_var!(control_msgs, HardwareDiagnostics);
roundtrip_var!(control_msgs, HardwareStatus);
roundtrip_var!(control_msgs, InterfaceValue);
roundtrip_var!(control_msgs, JointCommand);
roundtrip_var!(control_msgs, JointComponentTolerance);
roundtrip_var!(control_msgs, JointControllerState);
roundtrip_var!(control_msgs, JointJog, |s| {
    s.joint_names = vec![];
    s.displacements = vec![0.1, -0.2];
    s.duration = 0.05;
});
roundtrip_var!(control_msgs, JointTolerance);
roundtrip_var!(control_msgs, JointTrajectoryControllerState);
roundtrip_var!(control_msgs, JointWrenchTrajectory);
roundtrip_var!(control_msgs, JointWrenchTrajectoryPoint);
roundtrip_var!(control_msgs, Keys);
roundtrip_var!(control_msgs, MecanumDriveControllerState);
roundtrip_var!(control_msgs, MotionArgument);
roundtrip_var!(control_msgs, MotionPrimitive);
roundtrip_var!(control_msgs, MotionPrimitiveSequence);
roundtrip_var!(control_msgs, MultiDOFCommand);
roundtrip_var!(control_msgs, MultiDOFStateStamped);
roundtrip_var!(control_msgs, PidState);
roundtrip_var!(control_msgs, SingleDOFState);
roundtrip_var!(control_msgs, SingleDOFStateStamped);
roundtrip_fixed!(control_msgs, SpeedScalingFactor);
roundtrip_var!(control_msgs, SteeringControllerCommand);
roundtrip_var!(control_msgs, SteeringControllerStatus);
roundtrip_var!(control_msgs, VDA5050SafetyState);
roundtrip_var!(control_msgs, VDA5050State);
roundtrip_var!(control_msgs, WrenchFramed);

// -------------------------------------------------------------
// octomap_msgs (2: 0 fixed + 2 variable)
// -------------------------------------------------------------
roundtrip_var!(octomap_msgs, Octomap, |s| {
    s.binary = true;
    s.id = "OcTree".into();
    s.resolution = 0.05;
    s.data = vec![1, -2, 3];
});
roundtrip_var!(octomap_msgs, OctomapWithPose);

// -------------------------------------------------------------
// object_recognition_msgs (6: 0 fixed + 6 variable)
// -------------------------------------------------------------
roundtrip_var!(object_recognition_msgs, ObjectInformation);
roundtrip_var!(object_recognition_msgs, ObjectType, |s| {
    s.key = "mug".into();
    s.db = "household".into();
});
roundtrip_var!(object_recognition_msgs, RecognizedObject);
roundtrip_var!(object_recognition_msgs, RecognizedObjectArray);
roundtrip_var!(object_recognition_msgs, Table);
roundtrip_var!(object_recognition_msgs, TableArray);

// =============================================================
// Vendored perception message packages
// =============================================================

// -------------------------------------------------------------
// vision_msgs (16: 4 fixed + 12 variable)
// -------------------------------------------------------------
roundtrip_fixed!(vision_msgs, BoundingBox2D, |s| {
    // center is a fixed-nested Pose2D (Point2D + theta).
    s.center.position.x = 100.0;
    s.size_x = 64.0;
    s.size_y = 48.0;
});
roundtrip_var!(vision_msgs, BoundingBox2DArray, |s| {
    // DISTINCT non-empty bytes in raw-bytes siblings: catches an
    // offset-table sibling swap that all-empty defaults cannot.
    s.header = vec![0x01];
    s.boxes = vec![0xAA, 0xBB, 0xCC];
});
roundtrip_fixed!(vision_msgs, BoundingBox3D, |s| {
    s.center.position.x = 0.5;
    s.size.x = 1.0;
    s.size.y = 2.0;
});
roundtrip_var!(vision_msgs, BoundingBox3DArray, |s| {
    s.header = vec![0x02];
    s.boxes = vec![0xDD, 0xEE];
});
roundtrip_var!(vision_msgs, Classification, |s| {
    s.results = vec![1, 2, 3];
});
roundtrip_var!(vision_msgs, Detection2D, |s| {
    s.id = "det2d_1".into();
    s.bbox.size_x = 12.0;
    s.results = vec![0x10, 0x20];
});
roundtrip_var!(vision_msgs, Detection2DArray, |s| {
    s.detections = (0u8..16).collect();
});
roundtrip_var!(vision_msgs, Detection3D, |s| {
    s.id = "det3d_1".into();
    s.bbox.size.x = 3.0;
    s.results = vec![0x30, 0x40];
});
roundtrip_var!(vision_msgs, Detection3DArray, |s| {
    s.detections = (0u8..16).collect();
});
roundtrip_var!(vision_msgs, LabelInfo, |s| {
    s.threshold = 0.5;
    s.class_map = vec![0x01, 0x02];
});
roundtrip_var!(vision_msgs, ObjectHypothesis, |s| {
    s.class_id = "person".into();
    s.score = 0.875;
});
roundtrip_var!(vision_msgs, ObjectHypothesisWithPose, |s| {
    // hypothesis is the nested-as-bytes ObjectHypothesis; pose is a
    // fixed-nested PoseWithCovariance (no Debug, but Default+Clone+PartialEq).
    s.hypothesis = vec![0xA1, 0xA2];
});
roundtrip_fixed!(vision_msgs, Point2D, |s| {
    s.x = 1.0;
    s.y = 2.0;
});
roundtrip_fixed!(vision_msgs, Pose2D, |s| {
    s.position.x = 1.0;
    s.position.y = 2.0;
    s.theta = 0.5;
});
roundtrip_var!(vision_msgs, VisionClass, |s| {
    s.class_id = 7;
    s.class_name = "car".into();
});
roundtrip_var!(vision_msgs, VisionInfo, |s| {
    s.method = "yolo_v8".into();
    s.database_location = "rosparam:///classes".into();
    s.database_version = 3;
});

// -------------------------------------------------------------
// radar_msgs (4: 2 fixed + 2 variable)
// -------------------------------------------------------------
roundtrip_fixed!(radar_msgs, RadarReturn, |s| {
    s.range = 12.5;
    s.azimuth = -0.25;
    s.amplitude = -42.0;
});
roundtrip_var!(radar_msgs, RadarScan, |s| {
    s.header = vec![0x05];
    s.returns = vec![0x11, 0x22, 0x33, 0x44];
});
roundtrip_fixed!(radar_msgs, RadarTrack, |s| {
    // uuid is a fixed-nested unique_identifier_msgs/UUID (u8[16]); the
    // covariance arrays are the align-4 float32[6] tail after the u16.
    s.uuid.uuid = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
    s.position.x = 5.0;
    s.classification = 2;
    s.size_covariance = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
});
roundtrip_var!(radar_msgs, RadarTracks, |s| {
    s.header = vec![0x01];
    s.tracks = vec![0x11, 0x22, 0x33];
});

// -------------------------------------------------------------
// grid_map_msgs (2: 1 fixed + 1 variable)
// -------------------------------------------------------------
roundtrip_var!(grid_map_msgs, GridMap, |s| {
    // `header` is the leading variable field (std_msgs/Header); `info` is
    // a fixed nested struct (GridMapInfo). Distinct raw-bytes siblings + the
    // fixed nested `info` + two u16 tail fields: a crossed offset entry would
    // surface as a swapped payload.
    s.header = vec![0x07];
    s.info.resolution = 0.25;
    s.layers = vec![0xAA; 6];
    s.basic_layers = vec![0xBB, 0xBB];
    s.data = (100u8..148).collect();
    s.outer_start_index = 3;
    s.inner_start_index = 5;
});
roundtrip_fixed!(grid_map_msgs, GridMapInfo, |s| {
    s.resolution = 0.05;
    s.length_x = 10.0;
    s.length_y = 20.0;
    s.pose.position.x = 1.0;
});

// -------------------------------------------------------------
// unique_identifier_msgs (1: 1 fixed + 0 variable)
// -------------------------------------------------------------
roundtrip_fixed!(unique_identifier_msgs, UUID, |s| {
    s.uuid = [1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
});

// -------------------------------------------------------------
// autoware_perception_msgs (6: 1 fixed + 5 variable)
// -------------------------------------------------------------
roundtrip_fixed!(autoware_perception_msgs, ObjectClassification, |s| {
    s.label = 1;
    s.probability = 0.875;
});
roundtrip_var!(autoware_perception_msgs, Shape, |s| {
    // `footprint` (geometry_msgs/Polygon) is the lone variable field; `type`
    // (u8) + `dimensions` (Vector3) are fixed. Distinct non-empty bytes pin
    // the offset entry.
    s.footprint = vec![0xAA, 0xBB, 0xCC];
});
roundtrip_var!(autoware_perception_msgs, PredictedPath, |s| {
    // `path` (geometry_msgs/Pose[<=100]) → raw bytes; `confidence` (f32) +
    // `time_step` (Duration) are fixed.
    s.path = vec![0x01, 0x02];
    s.confidence = 0.5;
});
roundtrip_var!(autoware_perception_msgs, PredictedObjectKinematics, |s| {
    // `predicted_paths` is the lone variable field; the three
    // *WithCovariance fixed-nested members carry the f64[36] tails.
    s.predicted_paths = vec![0x11, 0x22, 0x33];
});
roundtrip_var!(autoware_perception_msgs, PredictedObject, |s| {
    // `classification`/`kinematics`/`shape` are raw-bytes variable fields;
    // `object_id` (UUID) + `existence_probability` (f32) are fixed.
    s.existence_probability = 0.9;
    s.classification = vec![0x10];
    s.kinematics = vec![0x20, 0x21];
    s.shape = vec![0x30, 0x31, 0x32];
});
roundtrip_var!(autoware_perception_msgs, PredictedObjects, |s| {
    s.header = vec![0x07];
    s.objects = vec![0xAA, 0xBB, 0xCC];
});

// -------------------------------------------------------------
// autoware_planning_msgs (5: 1 fixed + 4 variable)
// -------------------------------------------------------------
roundtrip_fixed!(autoware_planning_msgs, TrajectoryPoint, |s| {
    s.longitudinal_velocity_mps = 5.0;
    s.lateral_velocity_mps = 0.25;
    s.pose.position.x = 1.0;
});
roundtrip_var!(autoware_planning_msgs, Trajectory, |s| {
    s.header = vec![0x01];
    s.points = vec![0xDE, 0xAD, 0xBE, 0xEF];
});
roundtrip_var!(autoware_planning_msgs, LaneletPrimitive, |s| {
    s.id = 42;
    s.primitive_type = "lane".into();
});
roundtrip_var!(autoware_planning_msgs, LaneletSegment, |s| {
    // `preferred_primitive` (non-array nested) + `primitives` (array) are both
    // raw-bytes variable fields.
    s.preferred_primitive = vec![0x01, 0x02];
    s.primitives = vec![0x03, 0x04, 0x05];
});
roundtrip_var!(autoware_planning_msgs, LaneletRoute, |s| {
    // `header` + `segments` are variable; `start_pose`/`goal_pose` (Pose),
    // `uuid` (UUID), `allow_modification` (bool) are fixed.
    s.header = vec![0x08];
    s.segments = vec![0x09, 0x0A];
});
