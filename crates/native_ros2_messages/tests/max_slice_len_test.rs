// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end coverage that codegen-emitted `<T as ShmMessage>::MAX_SLICE_LEN`
//! lands on the expected tier for representative in-repo schemas.
//!
//! **Scope:** spot-checks one to five representative schemas per tier, plus
//! a hand-oracle pin, as a literal byte count, for each arm whose tier is easy to get wrong.
//! These tests catch tier-shift drift (e.g. someone moves `Header` from
//! TIER_TINY to TIER_HUGE) but do NOT enumerate every schema — a schema added
//! to `native_ros2_messages/` without an explicit arm in
//! `variable_schema_max_slice_len` falls through to the `_ => TIER_HUGE`
//! user-defined catch-all and these tests will not catch it.
//!
//! A sibling test closes that enumeration gap from the other side:
//! `tier_container_rule_test.rs` walks EVERY vendored `.msg` and calls
//! `variable_schema_max_slice_len` on every variable schema, which panics for
//! an in-repo schema with no arm. Totality lives there; the per-type VALUES
//! live here.
//!
//! Tests use `<T as cerulion_core::message::ShmMessage>::MAX_SLICE_LEN`
//! directly so the assertion runs against the live trait surface, not the
//! codegen string.

// `clippy::unimplemented` is denied workspace-wide (it has no business in
// production code). These four `unimplemented!()`s are TEST FIXTURES whose
// reader/writer paths are never exercised — the file documents the choice at
// `all_variables_written` below: `unimplemented!()` is deliberately preferred
// over an `unsafe` stand-in, because a fixture that is reached should panic
// loudly rather than read uninitialised memory.
#![allow(clippy::unimplemented)]

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;

// Tiers are `MaxSliceLen` constants so the
// assertions match the trait const's `Option<MaxSliceLen>` shape.
// `MaxSliceLen::const_new` panics at const-eval if the value is
// invalid; every tier value is > WireHeader::SIZE (32) AND ≤ u32::MAX,
// so it always returns a valid MaxSliceLen.
// The tiers are sized generously — free because
// iceoryx2 `Static` pools are lazy/demand-paged on Linux+macOS.
const TIER_HUGE: MaxSliceLen = MaxSliceLen::const_new(128 * 1024 * 1024);
const TIER_LARGE: MaxSliceLen = MaxSliceLen::const_new(16 * 1024 * 1024);
const TIER_MEDIUM: MaxSliceLen = MaxSliceLen::const_new(4 * 1024 * 1024);
const TIER_SMALL: MaxSliceLen = MaxSliceLen::const_new(256 * 1024);
const TIER_TINY: MaxSliceLen = MaxSliceLen::const_new(16 * 1024);

#[test]
fn test_image_class_lands_on_tier_huge() {
    use native_ros2_messages::nav_msgs::OccupancyGrid;
    use native_ros2_messages::sensor_msgs::{CompressedImage, Image, PointCloud2};

    assert_eq!(<Image as ShmMessage>::MAX_SLICE_LEN, Some(TIER_HUGE));
    assert_eq!(<PointCloud2 as ShmMessage>::MAX_SLICE_LEN, Some(TIER_HUGE));
    assert_eq!(
        <OccupancyGrid as ShmMessage>::MAX_SLICE_LEN,
        Some(TIER_HUGE)
    );
    // CompressedImage is HUGE, not LARGE. A 4K PNG over
    // `image_transport` is 13.7-24 MB, so it exceeds the 16 MiB LARGE ceiling
    // OUTRIGHT — the same hard-fail class as a raw 4K
    // Image. Pinned to the NUMBER below so a coordinated re-bless of the tier
    // constants cannot satisfy it.
    assert_eq!(
        <CompressedImage as ShmMessage>::MAX_SLICE_LEN,
        Some(TIER_HUGE)
    );
    assert_eq!(TIER_HUGE.get(), 134_217_728, "the huge tier is 128 MiB");
}

#[test]
fn test_marker_and_interactive_marker_types_land_on_tier_large() {
    use native_ros2_messages::sensor_msgs::PointCloud;
    use native_ros2_messages::visualization_msgs::{
        InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerInit,
        InteractiveMarkerUpdate, Marker, MarkerArray, MeshFile,
    };

    // `sensor_msgs/CompressedImage` is TIER_HUGE (see the huge-tier test).
    assert_eq!(<PointCloud as ShmMessage>::MAX_SLICE_LEN, Some(TIER_LARGE));
    assert_eq!(<MarkerArray as ShmMessage>::MAX_SLICE_LEN, Some(TIER_LARGE));
    assert_eq!(
        <InteractiveMarkerInit as ShmMessage>::MAX_SLICE_LEN,
        Some(TIER_LARGE)
    );
    assert_eq!(
        <InteractiveMarkerUpdate as ShmMessage>::MAX_SLICE_LEN,
        Some(TIER_LARGE)
    );
    assert_eq!(<MeshFile as ShmMessage>::MAX_SLICE_LEN, Some(TIER_LARGE));
    // Marker is LARGE, not MEDIUM: it carries upstream Jazzy's
    // `sensor_msgs/CompressedImage texture` + `visualization_msgs/MeshFile
    // mesh_file` — a container must not cap below its own members.
    //
    // Marker is LARGE even though CompressedImage is
    // HUGE. That is the table's one deliberate container-rule waiver (a Marker
    // texture is a mesh decal, not a camera frame); it is written out beside
    // Marker's arm in `wire_impl.rs` and machine-scoped by
    // `tier_container_rule_test.rs`. Asserting it HERE too means a
    // cascade of Marker to HUGE cannot land as a quiet table edit.
    assert_eq!(<Marker as ShmMessage>::MAX_SLICE_LEN, Some(TIER_LARGE));
    // The pair is LARGE, not MEDIUM, because
    // InteractiveMarkerControl carries `Marker[] markers` (LARGE) and
    // InteractiveMarker carries those controls — a 100K-triangle embedded-STL
    // marker (~5 MB) overflows the 4 MiB MEDIUM ceiling.
    assert_eq!(
        <InteractiveMarkerControl as ShmMessage>::MAX_SLICE_LEN,
        Some(TIER_LARGE)
    );
    assert_eq!(
        <InteractiveMarker as ShmMessage>::MAX_SLICE_LEN,
        Some(TIER_LARGE)
    );
    assert_eq!(TIER_LARGE.get(), 16_777_216, "the large tier is 16 MiB");
}

#[test]
fn test_paths_pose_arrays_and_trajectories_land_on_tier_medium() {
    use native_ros2_messages::geometry_msgs::PoseArray;
    use native_ros2_messages::nav_msgs::Path;
    use native_ros2_messages::std_msgs::String as StdString;
    use native_ros2_messages::trajectory_msgs::JointTrajectory;

    assert_eq!(<Path as ShmMessage>::MAX_SLICE_LEN, Some(TIER_MEDIUM));
    // `tf2_msgs/TFMessage` is SMALL and
    // is asserted in the small-tier test below.
    // `Marker`, `InteractiveMarker` and
    // `InteractiveMarkerControl` are TIER_LARGE, and the four
    // `geometry_msgs/Polygon*` types are SMALL — so none of them is here.
    assert_eq!(<PoseArray as ShmMessage>::MAX_SLICE_LEN, Some(TIER_MEDIUM));
    assert_eq!(
        <JointTrajectory as ShmMessage>::MAX_SLICE_LEN,
        Some(TIER_MEDIUM)
    );
    // HEADLINE CORRECTNESS PIN: `std_msgs/String` is MEDIUM, not
    // TINY. `/robot_description` is a latched String on essentially every
    // ROS robot and a URDF runs 60 KB (Go2-class) to 500 KB
    // (humanoid-class) — every one of those exceeds TINY's ~16.3 KB payload
    // budget, so publishing one would hard-fail `PayloadTooLarge`, and `Static`
    // pools cannot grow. Asserted as a HAND value against the spelled-out
    // number below: this line is what stands between a correct table and a
    // silent regression.
    assert_eq!(<StdString as ShmMessage>::MAX_SLICE_LEN, Some(TIER_MEDIUM));
    assert_eq!(
        TIER_MEDIUM.get(),
        4_194_304,
        "the medium tier is 4 MiB — spelled out so the String assertion above \
         is pinned to a NUMBER, not to whatever TIER_MEDIUM becomes"
    );
}

#[test]
fn test_typical_scans_and_multiarrays_land_on_tier_small() {
    use native_ros2_messages::geometry_msgs::Polygon;
    use native_ros2_messages::sensor_msgs::{JointState, LaserScan};
    use native_ros2_messages::std_msgs::{Float64MultiArray, UInt8MultiArray};
    use native_ros2_messages::tf2_msgs::TFMessage;
    use native_ros2_messages::trajectory_msgs::JointTrajectoryPoint;

    // `TFMessage` is SMALL, not MEDIUM. At 4 MiB a bridged `/tf`
    // derives the bridge-wide 1 MiB default and sits at ingress depth 64
    // (38.6 ms of absorption at 1.66 kHz), measured losing 12.67 % / 13.15 % in
    // a burst bench; at 256 KiB it derives depth 256 = 154 ms. The
    // capacity side still holds by the table's OWN sizing rule — a
    // `TransformStamped` is ~150 B on the wire, so MEDIUM's ~1000-element design
    // target is ~150 KB and fits here. Asserted as a HAND value, never re-blessed
    // from the actual: a change to this line changes what a `/tf` publisher can
    // fit in one message (~1,700 transforms) as well as its queue depth.
    assert_eq!(<TFMessage as ShmMessage>::MAX_SLICE_LEN, Some(TIER_SMALL));
    assert_eq!(
        TIER_SMALL.get(),
        262_144,
        "the small tier is 256 KiB — spelled out so the TFMessage assertion \
         above is pinned to a NUMBER, not to whatever TIER_SMALL becomes"
    );

    assert_eq!(<LaserScan as ShmMessage>::MAX_SLICE_LEN, Some(TIER_SMALL));
    assert_eq!(<JointState as ShmMessage>::MAX_SLICE_LEN, Some(TIER_SMALL));
    // `sensor_msgs/Joy` is TINY (joydev caps axes at 64)
    // and is asserted in the tiny-tier test below. `geometry_msgs/Polygon` and
    // `trajectory_msgs/JointTrajectoryPoint` are SMALL and belong to this
    // group — and each of those two ALSO keeps a container rule
    // satisfied from the member side (SolidPrimitive ⊃ Polygon;
    // JointWrenchTrajectoryPoint ⊃ JointTrajectoryPoint), which is why they are
    // pinned here as well as in the literal table below.
    assert_eq!(<Polygon as ShmMessage>::MAX_SLICE_LEN, Some(TIER_SMALL));
    assert_eq!(
        <JointTrajectoryPoint as ShmMessage>::MAX_SLICE_LEN,
        Some(TIER_SMALL)
    );
    assert_eq!(
        <UInt8MultiArray as ShmMessage>::MAX_SLICE_LEN,
        Some(TIER_SMALL)
    );
    assert_eq!(
        <Float64MultiArray as ShmMessage>::MAX_SLICE_LEN,
        Some(TIER_SMALL)
    );
}

#[test]
fn test_header_and_stamped_types_land_on_tier_tiny() {
    use native_ros2_messages::geometry_msgs::PoseStamped;
    use native_ros2_messages::nav_msgs::Odometry;
    use native_ros2_messages::sensor_msgs::{CameraInfo, Imu, Joy};
    use native_ros2_messages::std_msgs::Header;

    // `Pose` / `Twist` do not appear here — nested resolution
    // classifies them as FIXED schemas with an auto-computed
    // MAX_SLICE_LEN (see `test_pose_class_schemas_are_fixed_after_nested_
    // resolution` below). Header (string frame_id) and the Stamped/Imu
    // types (embed Header) remain variable on TIER_TINY.
    assert_eq!(<Header as ShmMessage>::MAX_SLICE_LEN, Some(TIER_TINY));
    assert_eq!(<PoseStamped as ShmMessage>::MAX_SLICE_LEN, Some(TIER_TINY));
    assert_eq!(<Imu as ShmMessage>::MAX_SLICE_LEN, Some(TIER_TINY));
    // The three highest-rate types at TINY rather than SMALL, each buying 4x
    // ingress-route depth (256 -> 1024) on a stream that genuinely runs at
    // rate. `std_msgs/String` is NOT in this group (it is
    // MEDIUM, a correctness pin), so a re-bless that flattened the
    // tier constants would have to break the medium-tier test as well.
    assert_eq!(<Joy as ShmMessage>::MAX_SLICE_LEN, Some(TIER_TINY));
    assert_eq!(<CameraInfo as ShmMessage>::MAX_SLICE_LEN, Some(TIER_TINY));
    assert_eq!(<Odometry as ShmMessage>::MAX_SLICE_LEN, Some(TIER_TINY));
    assert_eq!(
        TIER_TINY.get(),
        16_384,
        "the tiny tier is 16 KiB — spelled out so the assertions above are \
         pinned to a NUMBER, not to whatever TIER_TINY becomes"
    );
}

/// Arms whose tier is easy to get wrong, each pinned to the
/// LITERAL byte count of its tier.
///
/// The literal is the point. Every other assertion in this file compares
/// against a `TIER_*` constant declared at the top, so a re-bless that edited
/// the table AND those constants in lockstep would pass — a
/// coordinated re-bless. Here the number is written out
/// per type, so a tier's VALUE and a type's PLACEMENT cannot cancel out.
///
/// This reads the generated `MAX_SLICE_LEN` off the trait, not
/// `variable_schema_max_slice_len`, so it also proves codegen carried the
/// number into `native_ros2_messages` rather than pinning the table to itself.
///
/// Totality (no in-repo schema left without an arm) is
/// `tier_container_rule_test.rs`'s job; the container rule is too.
#[test]
fn every_retiered_arm_lands_on_its_audited_tier() {
    const HUGE: u32 = 134_217_728;
    const LARGE: u32 = 16_777_216;
    const MEDIUM: u32 = 4_194_304;
    const SMALL: u32 = 262_144;
    const TINY: u32 = 16_384;

    macro_rules! pin {
        ($($t:ty => $bytes:expr, $why:literal;)*) => {
            $(assert_eq!(
                <$t as ShmMessage>::MAX_SLICE_LEN,
                MaxSliceLen::try_new($bytes),
                concat!("tier pin ", stringify!($t), ": ", $why),
            );)*
        };
    }

    use native_ros2_messages::{
        control_msgs, geometry_msgs, moveit_msgs, nav_msgs, object_recognition_msgs, sensor_msgs,
        shape_msgs, statistics_msgs, std_msgs, trajectory_msgs, visualization_msgs,
    };

    // ── correctness: a lower tier is a latent or live PayloadTooLarge ──
    pin! {
        std_msgs::String => MEDIUM, "MEDIUM, not TINY; /robot_description URDFs are 60-500 KB";
        sensor_msgs::CompressedImage => HUGE, "HUGE, not LARGE; a 4K PNG is 13.7-24 MB";
        object_recognition_msgs::RecognizedObject => HUGE, "HUGE, not LARGE; embeds PointCloud2 (HUGE)";
        object_recognition_msgs::ObjectInformation => HUGE, "HUGE, not LARGE; embeds PointCloud2 (HUGE)";
        visualization_msgs::InteractiveMarkerControl => LARGE, "LARGE, not MEDIUM; embeds Marker[] (LARGE)";
        visualization_msgs::InteractiveMarker => LARGE, "LARGE, not MEDIUM; embeds InteractiveMarkerControl[]";
    }

    // ── high-rate depth: the lower tier buys 4x ingress depth on a real stream ──
    pin! {
        sensor_msgs::Joy => TINY, "TINY, not SMALL; joydev caps axes at 64, worst 850 B";
        sensor_msgs::CameraInfo => TINY, "TINY, not SMALL; d[] <= 18 coefficients, worst ~500 B";
        nav_msgs::Odometry => TINY, "TINY, not SMALL; 688 B fixed + two frame strings";
        trajectory_msgs::JointTrajectoryPoint => SMALL, "SMALL, not MEDIUM; 60-DOF single point = 1,920 B";
        trajectory_msgs::MultiDOFJointTrajectoryPoint => SMALL, "SMALL, not MEDIUM; 10 multi-DOF joints = 1.6 KB";
        sensor_msgs::MultiDOFJointState => SMALL, "SMALL, not MEDIUM; 100-joint aggregator ~17.6 KB";
        control_msgs::DynamicJointState => SMALL, "SMALL, not MEDIUM; 90-DOF x 12 interfaces ~32 KB";
        control_msgs::MultiDOFStateStamped => SMALL, "SMALL, not MEDIUM; 100-DOF worst 12 KB";
        control_msgs::Float64Values => SMALL, "SMALL, not MEDIUM; Float64MultiArray is SMALL too";
        control_msgs::DynamicInterfaceValues => SMALL, "SMALL, not MEDIUM; 100 interfaces/side ~10 KB";
        control_msgs::DynamicInterfaceGroupValues => SMALL, "SMALL, not MEDIUM; 50 groups x 20 interfaces ~45 KB";
        control_msgs::SteeringControllerStatus => SMALL, "SMALL, not MEDIUM; 16-wheel platform 740 B";
        control_msgs::SteeringControllerCommand => TINY, "TINY, not SMALL; Header + 2 f64 ~= 120 B";
        moveit_msgs::ContactInformation => TINY, "TINY, not SMALL; ~500 B worst";
        object_recognition_msgs::Table => SMALL, "SMALL, not MEDIUM; 2,000-point hull = 48 KB";
        geometry_msgs::Polygon => SMALL, "SMALL, not MEDIUM; 5,000-vertex geofence ~60 KB";
        geometry_msgs::PolygonStamped => SMALL, "SMALL, not MEDIUM; atomic with Polygon";
        geometry_msgs::PolygonInstance => SMALL, "SMALL, not MEDIUM; atomic with Polygon";
        geometry_msgs::PolygonInstanceStamped => SMALL, "SMALL, not MEDIUM; atomic with Polygon";
    }

    // ── rule consistency: little/no rate stake ──
    pin! {
        moveit_msgs::KinematicSolverInfo => SMALL, "SMALL, not MEDIUM; 150-joint work cell ~24 KB";
        moveit_msgs::PlannerInterfaceDescription => SMALL, "SMALL, not MEDIUM; 200 ids ~8.8 KB (over TINY)";
        moveit_msgs::PlannerParams => SMALL, "SMALL, not MEDIUM; 100 params with prose = 20 KB";
        moveit_msgs::AllowedCollisionEntry => TINY, "TINY, not SMALL; 500-body row ~500 B";
        sensor_msgs::BatteryState => TINY, "TINY, not SMALL; bounded by pack SERIES count, worst ~1.7 KB";
        sensor_msgs::LaserEcho => TINY, "TINY, not SMALL; <= 5 echoes per beam ~60 B";
        control_msgs::HardwareDeviceDiagnostics => SMALL, "SMALL, not MEDIUM; 100 x 200-B KeyValue = 20 KB";
        statistics_msgs::MetricsMessage => SMALL, "SMALL, not MEDIUM; closed 5-point vocabulary (TINY has no headroom)";
        control_msgs::BatteryStateArray => SMALL, "SMALL, not MEDIUM; 10 packs x 2 KB = 20 KB";
        control_msgs::GenericHardwareState => SMALL, "SMALL, not MEDIUM; verbose worst ~3 KB";
        control_msgs::EtherCATState => TINY, "TINY, not SMALL; ESI id strings, worst 300 B";
        control_msgs::MotionPrimitive => SMALL, "SMALL, not MEDIUM; worst 15 KB";
        control_msgs::HardwareDeviceStatus => SMALL, "SMALL, not MEDIUM; floor met by its four members at SMALL";
        std_msgs::MultiArrayLayout => TINY, "TINY, not SMALL; 8-dim tensor layout ~1 KB";
    }

    // ── consistency with no depth stake (both tiers derive depth 64) ──
    pin! {
        object_recognition_msgs::TableArray => MEDIUM, "MEDIUM, not LARGE; 25 dense tables x 4 = 500 KB";
        moveit_msgs::Grasp => MEDIUM, "MEDIUM, not LARGE; floor = JointTrajectory members (MEDIUM)";
        moveit_msgs::PlaceLocation => MEDIUM, "MEDIUM, not LARGE; same floor, one fewer posture";
    }

    // ── SMALL rather than TINY, on headroom ──
    pin! {
        control_msgs::VDA5050State => SMALL, "SMALL, not MEDIUM and not TINY: \
            worst 2.5 KB x 4 = 10 KB would eat 62% of TINY, the type is Lyrical-new, and \
            under-tiering hard-fails on the ERROR path";
    }

    // ── tiers that look movable and are not ──
    //
    // Three of these depend on a pin above, so pinning
    // them here is what catches a revert: if `std_msgs/String` ever
    // drops below MEDIUM, or `JointTrajectoryPoint`/`Polygon` ever rise above
    // SMALL, these become real container violations and
    // `tier_container_rule_test.rs` fails — but ONLY if they are still on
    // the tiers pinned here.
    //
    // `JoyFeedbackArray` is a different kind: its reason is CAPACITY,
    // which no container walk can catch, so this literal is the
    // only thing that keeps a move to TINY from landing unnoticed.
    pin! {
        sensor_msgs::JoyFeedbackArray => SMALL,
            "SMALL, not TINY: TINY looks sufficient on the ground that \
             `JoyFeedback.id` is uint8 and so bounds the array at 256 elements. The wire carries \
             an UNBOUNDED `JoyFeedback[]` with no uniqueness constraint, so that is not a wire \
             bound; and as a SEMANTIC bound it is wrong, because a command is addressed by \
             the PAIR (type, id) and `type` is uint8 too — 3 defined types x 256 ids x an 8-byte \
             measured stride = 6,144 B, and 6,144 x 4 = 24 KiB does not fit TINY's 16 KiB. \
             TINY holds 2,047 elements, SMALL holds 32,767. TINY also buys \
             nothing (a low-rate command topic TO a joystick), and a nonzero breakage class for a \
             nil benefit is never the trade — same call as VDA5050State";
    }
    pin! {
        control_msgs::AdmittanceControllerState => MEDIUM,
            "MEDIUM, not SMALL: it embeds std_msgs/String, which is MEDIUM, \
             and a container must not cap below its members";
        control_msgs::JointWrenchTrajectoryPoint => SMALL,
            "SMALL, not MEDIUM: the container rule holds at SMALL because its member \
             JointTrajectoryPoint is SMALL";
        shape_msgs::SolidPrimitive => SMALL,
            "SMALL, not MEDIUM: the container rule holds at SMALL because its member \
             Polygon is SMALL";
    }
}

/// Nested resolution reclassifies the
/// nested-only geometry types as FIXED schemas. Their MAX_SLICE_LEN is
/// the provably-correct auto-computed bound (WireHeader + fixed section),
/// not a tier — and VARIABLE_FIELD_COUNT drops to 0, which is what makes
/// them loanable end-to-end (`rmw can_loan_messages` correctness for the
/// MoveIt integration).
#[test]
fn test_pose_class_schemas_are_fixed_after_nested_resolution() {
    use cerulion_core::wire::WireHeader;
    use native_ros2_messages::geometry_msgs::{Pose, Transform, Twist, Wrench};

    macro_rules! assert_fixed {
        ($t:ty) => {
            assert_eq!(<$t as ShmMessage>::VARIABLE_FIELD_COUNT, 0);
            assert_eq!(
                <$t as ShmMessage>::MAX_SLICE_LEN,
                MaxSliceLen::try_new(
                    (WireHeader::SIZE + <$t as ShmMessage>::WIRE_FIXED_SIZE) as u32
                )
            );
        };
    }

    assert_fixed!(Pose);
    assert_fixed!(Twist);
    assert_fixed!(Transform);
    assert_fixed!(Wrench);
}

#[test]
fn test_fixed_only_schemas_compute_max_slice_len_from_wire_size() {
    use cerulion_core::wire::WireHeader;
    use native_ros2_messages::geometry_msgs::{Point, Quaternion, Vector3};
    use native_ros2_messages::shape_msgs::{MeshTriangle, Plane};
    use native_ros2_messages::std_msgs::ColorRGBA;

    // Fixed-only schemas auto-compute MAX_SLICE_LEN as
    // WireHeader::SIZE + WIRE_FIXED_SIZE (provably correct upper bound:
    // every published frame is exactly this size). Includes the
    // FixedArray-only types (`MeshTriangle` = `uint32[3]`,
    // `Plane` = `float64[4]`) which are a recurring classification footgun:
    // misclassifying them as variable would
    // route them through the variable codegen path and assign one
    // of the 5 tiers instead of the auto-computed bound.
    // Trait const is `Option<NonZeroU32>`; expected via
    // `NonZeroU32::new(...)` which returns `Option<NonZeroU32>`
    // directly. Each fixed-schema sum is > 0 because `WireHeader::SIZE`
    // is 32, so `Some(...)` always materialises.
    assert_eq!(
        <Vector3 as ShmMessage>::MAX_SLICE_LEN,
        MaxSliceLen::try_new((WireHeader::SIZE + <Vector3 as ShmMessage>::WIRE_FIXED_SIZE) as u32)
    );
    assert_eq!(
        <Point as ShmMessage>::MAX_SLICE_LEN,
        MaxSliceLen::try_new((WireHeader::SIZE + <Point as ShmMessage>::WIRE_FIXED_SIZE) as u32)
    );
    assert_eq!(
        <Quaternion as ShmMessage>::MAX_SLICE_LEN,
        MaxSliceLen::try_new(
            (WireHeader::SIZE + <Quaternion as ShmMessage>::WIRE_FIXED_SIZE) as u32
        )
    );
    assert_eq!(
        <ColorRGBA as ShmMessage>::MAX_SLICE_LEN,
        MaxSliceLen::try_new(
            (WireHeader::SIZE + <ColorRGBA as ShmMessage>::WIRE_FIXED_SIZE) as u32
        )
    );
    assert_eq!(
        <MeshTriangle as ShmMessage>::MAX_SLICE_LEN,
        MaxSliceLen::try_new(
            (WireHeader::SIZE + <MeshTriangle as ShmMessage>::WIRE_FIXED_SIZE) as u32
        )
    );
    assert_eq!(
        <Plane as ShmMessage>::MAX_SLICE_LEN,
        MaxSliceLen::try_new((WireHeader::SIZE + <Plane as ShmMessage>::WIRE_FIXED_SIZE) as u32)
    );
}

/// Hand-written `impl ShmMessage` block used by
/// `test_hand_written_impl_omits_max_slice_len_const`. The trait
/// surface requires four named consts + the GATs + four method impls;
/// this stub satisfies the type-check side of the trait while OMITTING
/// `MAX_SLICE_LEN`, so the `Option<NonZeroU32> = None` default value
/// declared on the trait is what flows through.
///
/// The `build_reader`/`build_writer`/`payload_wire_size`/
/// `all_variables_written` bodies use `unimplemented!()` because the
/// test only reads the trait const at compile time and never invokes
/// these methods. Using `unimplemented!()` (rather than an `unsafe`
/// pointer cast over `&[u8]`) keeps this fixture sound — a future
/// test that exercises the round-trip path will have to replace these
/// stubs with a real safe implementation.
struct HandWrittenMarker;

struct HandWrittenShm;

impl ShmMessage for HandWrittenMarker {
    const SCHEMA_HASH: u64 = 0xdead_beef_dead_beef;
    const VARIABLE_FIELD_COUNT: usize = 0;
    const WIRE_FIXED_SIZE: usize = 4;
    // MAX_SLICE_LEN intentionally omitted — exercises the trait default.

    type Reader<'a> = &'a HandWrittenShm;
    type Writer<'a> = &'a mut HandWrittenShm;

    fn build_reader(_bytes: &[u8]) -> Self::Reader<'_> {
        unimplemented!("test fixture; reader path not exercised")
    }

    fn build_writer<'a>(
        _bytes: &'a mut [u8],
        _max_capacity: cerulion_core::wire::MaxPayloadCapacity,
        _topic: std::sync::Arc<str>,
    ) -> Self::Writer<'a> {
        unimplemented!("test fixture; writer path not exercised")
    }

    fn payload_wire_size(_writer: &Self::Writer<'_>) -> usize {
        unimplemented!("test fixture; writer path not exercised")
    }

    fn all_variables_written(_writer: &Self::Writer<'_>) -> bool {
        unimplemented!("test fixture; writer path not exercised")
    }
}

#[test]
fn test_hand_written_impl_omits_max_slice_len_const() {
    // The trait const default is `None` so
    // hand-written impls (and any third-party `impl ShmMessage`) that
    // OMIT `MAX_SLICE_LEN` see the trait default flow through, and the
    // runtime resolver routes a `None` to its
    // fallback (`DEFAULT_MAX_SLICE_LEN`). This pins
    // the default value itself so a change that flips the default
    // to e.g. `Some(0)` (which would be a footgun — zero-byte loans
    // always fail at runtime) breaks the build here.
    assert_eq!(<HandWrittenMarker as ShmMessage>::MAX_SLICE_LEN, None);
}
