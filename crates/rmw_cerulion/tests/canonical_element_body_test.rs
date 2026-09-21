// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The rmw bridge's variable-nested ELEMENT BODY is the CANONICAL
//! Cerulion encoding — proven against hand-built byte oracles and against the
//! independent `FrameWalker`.
//!
//! # Why this file exists
//!
//! The other rmw complex-codec tests round-trip the bridge against
//! ITSELF (`flatten` then `unflatten` on the same `BridgedMessage`, asserting
//! only the decoded field values). A symmetric encoder+decoder pair is
//! self-consistent no matter what convention it invents, so those tests stay
//! green while a bridge emits a body — `[packed fixed][u32
//! count-of-variable-members][per var: u32 len + payload]` — that no other
//! component in the system can read. They cannot tell a packed body from
//! the canonical one. The tests in THIS file can: they compare against
//! oracles the bridge did not produce.
//!
//! # The two oracles, deliberately different in kind
//!
//! 1. **Hand-built bytes.** Each expected element body is assembled here, by
//!    hand, from the `.msg` field order — never by calling
//!    `CanonicalBodyBuilder` (which would re-compare the implementation with
//!    itself). If the writer changes shape, these fail with a byte diff.
//!
//! 2. **The `FrameWalker`.** An INDEPENDENT reader, built from the parsed
//!    `native_ros2_messages::BUILTIN_MSGS` `.msg` text rather than from the
//!    introspection data the bridge encodes from, and with its own strict
//!    degrade discipline (non-canonical bytes become `NestedArrayOpaque`,
//!    never a guessed element list). Its verdict is the user-visible payoff:
//!    these very topics — `nav_msgs/Path`, `tf2_msgs/TFMessage` — are the ones
//!    that draw as a text dump instead of a polyline when the body is packed.
//!
//! The two schema derivations meeting is itself an assertion: the bridge
//! computes its `schema_hash` from hand-built introspection, the walker from
//! the vendored `.msg` text, and `walk_by_hash` only resolves if they agree.
//!
//! # What fails these tests
//!
//! Every test here FAILS against an encoder that packs the element body
//! (the packed shape described above). A test whose red was
//! never observed is not a pin.

use std::ffi::CString;
use std::os::raw::c_void;

use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, FrameWalker, MessageSchema};
use cerulion_core::wire::WireHeader;
use rmw_cerulion::ffi::{
    rosidl_message_type_support_t, rosidl_typesupport_introspection_c__MessageMember,
    rosidl_typesupport_introspection_c__MessageMembers,
};
use rmw_cerulion::type_bridge::BridgedMessage;

// Introspection type ids (rosidl_typesupport_introspection_c), matching
// `rmw_cerulion::type_bridge::ros_type`.
//
// `ROS_TYPE_INT32` is `13`; `6` is BOOLEAN. A `Time` fixture that declares
// `sec` with the wrong id goes unnoticed, because `Time` is
// recursively FIXED (copied wholesale at `size_of_`, so no byte moves) and
// because a VARIABLE nested field like `Header` contributes only its qualified
// NAME to the parent's schema hash (recipe 3), so no hash gate sees it either.
// A fixture that puts an `int32` in a VARIABLE element's own fixed section
// would skew for real.
const ROS_TYPE_BOOL: u8 = 6;
const ROS_TYPE_DOUBLE: u8 = 2;
const ROS_TYPE_UINT8: u8 = 8;
const ROS_TYPE_UINT32: u8 = 12;
const ROS_TYPE_INT32: u8 = 13;
const ROS_TYPE_STRING: u8 = 16;
const ROS_TYPE_MESSAGE: u8 = 18;

// ---------------------------------------------------------------------------
// Hand-built introspection fixtures (no ROS installation required)
// ---------------------------------------------------------------------------

fn cstr(s: &str) -> *const std::os::raw::c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn member(
    name: &str,
    type_id: u8,
    offset: usize,
    is_array: bool,
    nested: *const rosidl_message_type_support_t,
) -> rosidl_typesupport_introspection_c__MessageMember {
    rosidl_typesupport_introspection_c__MessageMember {
        name_: cstr(name),
        type_id_: type_id,
        members_: nested,
        is_array_: is_array,
        array_size_: 0,
        is_upper_bound_: false,
        offset_: offset as u32,
        ..Default::default()
    }
}

fn make_members(
    namespace: &str,
    name: &str,
    size_of: usize,
    members: Vec<rosidl_typesupport_introspection_c__MessageMember>,
) -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let members = Box::leak(members.into_boxed_slice());
    let mm = rosidl_typesupport_introspection_c__MessageMembers {
        message_namespace_: cstr(namespace),
        message_name_: cstr(name),
        member_count_: members.len() as u32,
        size_of_: size_of,
        members_: members.as_ptr(),
        ..Default::default()
    };
    Box::leak(Box::new(mm))
}

fn ts(
    members: *const rosidl_typesupport_introspection_c__MessageMembers,
) -> *const rosidl_message_type_support_t {
    Box::leak(Box::new(rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: members as *const c_void,
        ..Default::default()
    }))
}

// ---------------------------------------------------------------------------
// C message structs — the shapes rosidl generates
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone, Copy)]
struct CRosString {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

#[repr(C)]
struct CRosSeq {
    data: *mut c_void,
    size: usize,
    capacity: usize,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct CTime {
    sec: i32,
    nanosec: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct CHeader {
    stamp: CTime,
    frame_id: CRosString,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct CPoint {
    x: f64,
    y: f64,
    z: f64,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct CQuat {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct CPose {
    position: CPoint,
    orientation: CQuat,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct CPoseStamped {
    header: CHeader,
    pose: CPose,
}

#[repr(C)]
struct CPath {
    header: CHeader,
    poses: CRosSeq,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct CTransform {
    translation: CPoint,
    rotation: CQuat,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct CTransformStamped {
    header: CHeader,
    child_frame_id: CRosString,
    transform: CTransform,
}

#[repr(C)]
struct CTFMessage {
    transforms: CRosSeq,
}

/// `sensor_msgs/PointField` in C — the variable member FIRST, then three fixed
/// members whose C offsets (24, 28, 32) all differ from their wire offsets
/// (0, 4, 8). See [`point_field_members`].
#[repr(C)]
#[derive(Clone, Copy)]
struct CPointField {
    name: CRosString,
    offset: u32,
    datatype: u8,
    count: u32,
}

#[repr(C)]
struct CPointCloud2 {
    header: CHeader,
    height: u32,
    width: u32,
    fields: CRosSeq,
    is_bigendian: bool,
    point_step: u32,
    row_step: u32,
    data: CRosSeq,
    is_dense: bool,
}

extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
}

fn heap_string(s: &str) -> CRosString {
    let mem = unsafe { calloc(s.len() + 1, 1) } as *mut u8;
    unsafe { std::ptr::copy_nonoverlapping(s.as_ptr(), mem, s.len()) };
    CRosString {
        data: mem,
        size: s.len(),
        capacity: s.len() + 1,
    }
}

fn time_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "builtin_interfaces__msg",
        "Time",
        std::mem::size_of::<CTime>(),
        vec![
            member(
                "sec",
                ROS_TYPE_INT32,
                std::mem::offset_of!(CTime, sec),
                false,
                std::ptr::null(),
            ),
            member(
                "nanosec",
                ROS_TYPE_UINT32,
                std::mem::offset_of!(CTime, nanosec),
                false,
                std::ptr::null(),
            ),
        ],
    )
}

fn header_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "std_msgs__msg",
        "Header",
        std::mem::size_of::<CHeader>(),
        vec![
            member(
                "stamp",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CHeader, stamp),
                false,
                ts(time_members()),
            ),
            member(
                "frame_id",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CHeader, frame_id),
                false,
                std::ptr::null(),
            ),
        ],
    )
}

fn xyz_members(ns: &str, name: &str) -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        ns,
        name,
        std::mem::size_of::<CPoint>(),
        vec![
            member("x", ROS_TYPE_DOUBLE, 0, false, std::ptr::null()),
            member("y", ROS_TYPE_DOUBLE, 8, false, std::ptr::null()),
            member("z", ROS_TYPE_DOUBLE, 16, false, std::ptr::null()),
        ],
    )
}

fn quaternion_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "geometry_msgs__msg",
        "Quaternion",
        std::mem::size_of::<CQuat>(),
        vec![
            member("x", ROS_TYPE_DOUBLE, 0, false, std::ptr::null()),
            member("y", ROS_TYPE_DOUBLE, 8, false, std::ptr::null()),
            member("z", ROS_TYPE_DOUBLE, 16, false, std::ptr::null()),
            member("w", ROS_TYPE_DOUBLE, 24, false, std::ptr::null()),
        ],
    )
}

fn pose_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "geometry_msgs__msg",
        "Pose",
        std::mem::size_of::<CPose>(),
        vec![
            member(
                "position",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CPose, position),
                false,
                ts(xyz_members("geometry_msgs__msg", "Point")),
            ),
            member(
                "orientation",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CPose, orientation),
                false,
                ts(quaternion_members()),
            ),
        ],
    )
}

fn pose_stamped_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "geometry_msgs__msg",
        "PoseStamped",
        std::mem::size_of::<CPoseStamped>(),
        vec![
            member(
                "header",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CPoseStamped, header),
                false,
                ts(header_members()),
            ),
            member(
                "pose",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CPoseStamped, pose),
                false,
                ts(pose_members()),
            ),
        ],
    )
}

fn path_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "nav_msgs__msg",
        "Path",
        std::mem::size_of::<CPath>(),
        vec![
            member(
                "header",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CPath, header),
                false,
                ts(header_members()),
            ),
            member(
                "poses",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CPath, poses),
                true,
                ts(pose_stamped_members()),
            ),
        ],
    )
}

fn transform_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "geometry_msgs__msg",
        "Transform",
        std::mem::size_of::<CTransform>(),
        vec![
            member(
                "translation",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CTransform, translation),
                false,
                ts(xyz_members("geometry_msgs__msg", "Vector3")),
            ),
            member(
                "rotation",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CTransform, rotation),
                false,
                ts(quaternion_members()),
            ),
        ],
    )
}

fn transform_stamped_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "geometry_msgs__msg",
        "TransformStamped",
        std::mem::size_of::<CTransformStamped>(),
        vec![
            member(
                "header",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CTransformStamped, header),
                false,
                ts(header_members()),
            ),
            member(
                "child_frame_id",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CTransformStamped, child_frame_id),
                false,
                std::ptr::null(),
            ),
            member(
                "transform",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CTransformStamped, transform),
                false,
                ts(transform_members()),
            ),
        ],
    )
}

/// `sensor_msgs/PointField` — `string name`, `uint32 offset`, `uint8 datatype`,
/// `uint32 count`.
///
/// THE shape no other fixture in this file has: THREE fixed members, with the
/// VARIABLE one (`name`) declared FIRST. Every other variable-nested fixture
/// here (`Header`, `PoseStamped`, `TransformStamped`) has exactly ONE fixed
/// member, so the encoder/decoder's wire-offset relocation loop never advances
/// `fixed_idx` past 0 and its per-member offset arithmetic goes unexercised.
///
/// Two independent things only this fixture exercises:
///
/// - **Relocation.** In C the members sit at 0 (`name`, 24 B on 64-bit), 24,
///   28, 32; on the wire the fixed section holds only the three fixed members,
///   at 0, 4, 8. Every C offset differs from its wire offset, so a loop that
///   confused the two would still pass with one fixed member and fails here.
/// - **Interior padding.** `datatype` is a `u8` at wire offset 4 and `count` is
///   a `u32`, so the wire fixed section carries three ZERO pad bytes at [5..8]
///   — padding BETWEEN two fixed members, not trailing. This is the
///   `visualization_msgs/Marker` shape.
fn point_field_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "sensor_msgs__msg",
        "PointField",
        std::mem::size_of::<CPointField>(),
        vec![
            member(
                "name",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CPointField, name),
                false,
                std::ptr::null(),
            ),
            member(
                "offset",
                ROS_TYPE_UINT32,
                std::mem::offset_of!(CPointField, offset),
                false,
                std::ptr::null(),
            ),
            member(
                "datatype",
                ROS_TYPE_UINT8,
                std::mem::offset_of!(CPointField, datatype),
                false,
                std::ptr::null(),
            ),
            member(
                "count",
                ROS_TYPE_UINT32,
                std::mem::offset_of!(CPointField, count),
                false,
                std::ptr::null(),
            ),
        ],
    )
}

/// `sensor_msgs/PointCloud2` — the top-level carrier for `PointField[] fields`.
fn point_cloud2_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "sensor_msgs__msg",
        "PointCloud2",
        std::mem::size_of::<CPointCloud2>(),
        vec![
            member(
                "header",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CPointCloud2, header),
                false,
                ts(header_members()),
            ),
            member(
                "height",
                ROS_TYPE_UINT32,
                std::mem::offset_of!(CPointCloud2, height),
                false,
                std::ptr::null(),
            ),
            member(
                "width",
                ROS_TYPE_UINT32,
                std::mem::offset_of!(CPointCloud2, width),
                false,
                std::ptr::null(),
            ),
            member(
                "fields",
                ROS_TYPE_MESSAGE,
                std::mem::offset_of!(CPointCloud2, fields),
                true,
                ts(point_field_members()),
            ),
            member(
                "is_bigendian",
                ROS_TYPE_BOOL,
                std::mem::offset_of!(CPointCloud2, is_bigendian),
                false,
                std::ptr::null(),
            ),
            member(
                "point_step",
                ROS_TYPE_UINT32,
                std::mem::offset_of!(CPointCloud2, point_step),
                false,
                std::ptr::null(),
            ),
            member(
                "row_step",
                ROS_TYPE_UINT32,
                std::mem::offset_of!(CPointCloud2, row_step),
                false,
                std::ptr::null(),
            ),
            member(
                "data",
                ROS_TYPE_UINT8,
                std::mem::offset_of!(CPointCloud2, data),
                true,
                std::ptr::null(),
            ),
            member(
                "is_dense",
                ROS_TYPE_BOOL,
                std::mem::offset_of!(CPointCloud2, is_dense),
                false,
                std::ptr::null(),
            ),
        ],
    )
}

fn tf_message_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "tf2_msgs__msg",
        "TFMessage",
        std::mem::size_of::<CTFMessage>(),
        vec![member(
            "transforms",
            ROS_TYPE_MESSAGE,
            std::mem::offset_of!(CTFMessage, transforms),
            true,
            ts(transform_stamped_members()),
        )],
    )
}

// ---------------------------------------------------------------------------
// HAND oracles — assembled from the .msg field order, NEVER from the codec
// ---------------------------------------------------------------------------

/// `std_msgs/Header` body: fixed `Time` (8) + 1 offset entry (8) + frame_id.
fn header_body_oracle(sec: i32, nanosec: u32, frame_id: &str) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(&sec.to_le_bytes()); // [0..4]  stamp.sec
    b.extend_from_slice(&nanosec.to_le_bytes()); // [4..8]  stamp.nanosec
    b.extend_from_slice(&16u32.to_le_bytes()); // [8..12] entry.offset (8 fixed + 8 table)
    b.extend_from_slice(&(frame_id.len() as u32).to_le_bytes()); // [12..16] entry.length
    b.extend_from_slice(frame_id.as_bytes());
    b
}

/// `geometry_msgs/Pose` — a recursively-FIXED nested value: raw 56 bytes.
fn pose_fixed_oracle(p: &CPoint, q: &CQuat) -> Vec<u8> {
    let mut b = Vec::new();
    for v in [p.x, p.y, p.z, q.x, q.y, q.z, q.w] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    assert_eq!(b.len(), 56);
    b
}

/// `geometry_msgs/PoseStamped` element body.
///
/// Wire layout: `pose` is fixed (56 bytes) so it forms the whole fixed
/// section; `header` is the single variable field, so the table is 8 bytes and
/// the header sub-frame starts at 64.
fn pose_stamped_body_oracle(sec: i32, nanosec: u32, frame_id: &str, pose: &[u8]) -> Vec<u8> {
    let header = header_body_oracle(sec, nanosec, frame_id);
    let mut b = Vec::new();
    b.extend_from_slice(pose); // [0..56]  fixed section = pose
    b.extend_from_slice(&64u32.to_le_bytes()); // [56..60] entry.offset = 56 + 8
    b.extend_from_slice(&(header.len() as u32).to_le_bytes()); // [60..64] entry.length
    b.extend_from_slice(&header); // [64..]   header sub-frame
    b
}

/// `geometry_msgs/TransformStamped` element body — the TWO-variable-member
/// case, where the canonical 8N-byte table (16) and a packed
/// `4 + 4N` framing (12) differ in LENGTH as well as content.
fn transform_stamped_body_oracle(
    sec: i32,
    nanosec: u32,
    frame_id: &str,
    child_frame_id: &str,
    transform: &[u8],
) -> Vec<u8> {
    let header = header_body_oracle(sec, nanosec, frame_id);
    let mut b = Vec::new();
    b.extend_from_slice(transform); // [0..56]  fixed section = transform
                                    // Table: 2 entries × 8 = 16 bytes, so payloads start at 72.
    b.extend_from_slice(&72u32.to_le_bytes()); // header.offset
    b.extend_from_slice(&(header.len() as u32).to_le_bytes()); // header.length
    b.extend_from_slice(&((72 + header.len()) as u32).to_le_bytes()); // child.offset
    b.extend_from_slice(&(child_frame_id.len() as u32).to_le_bytes()); // child.length
    b.extend_from_slice(&header);
    b.extend_from_slice(child_frame_id.as_bytes());
    b
}

/// `sensor_msgs/PointField` element body — the THREE-FIXED-MEMBER case with
/// INTERIOR padding.
///
/// Wire layout, spelled out because that is the whole point of this oracle:
///
/// ```text
/// [0..4]   offset    (u32)
/// [4..5]   datatype  (u8)
/// [5..8]   PADDING   (3 zero bytes — `count` is u32-aligned)
/// [8..12]  count     (u32)          ← fixed_size = 12
/// [12..16] entry[0]  offset = 20    ← `name`, the only variable member
/// [16..20] entry[0]  length         ← data floor = 12 + 8 = 20
/// [20..]   name bytes
/// ```
///
/// The pad bytes MUST be zero: a `repr(C)` fixed section is copied wholesale, so
/// leaving them as whatever the source struct held would make the frame
/// nondeterministic (Principle #7).
fn point_field_body_oracle(offset: u32, datatype: u8, count: u32, name: &str) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(&offset.to_le_bytes()); // [0..4]
    b.push(datatype); // [4..5]
    b.extend_from_slice(&[0, 0, 0]); // [5..8]   interior padding
    b.extend_from_slice(&count.to_le_bytes()); // [8..12]
    b.extend_from_slice(&20u32.to_le_bytes()); // [12..16] entry.offset
    b.extend_from_slice(&(name.len() as u32).to_le_bytes()); // [16..20] entry.length
    b.extend_from_slice(name.as_bytes()); // [20..]
    b
}

/// A counted element array: `u32 count` + per element (`u32 len` + body).
/// This array framing does not depend on how the element bodies are encoded.
fn counted_array_oracle(bodies: &[Vec<u8>]) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(&(bodies.len() as u32).to_le_bytes());
    for body in bodies {
        b.extend_from_slice(&(body.len() as u32).to_le_bytes());
        b.extend_from_slice(body);
    }
    b
}

/// Read variable entry `idx` out of a frame PAYLOAD (offsets are
/// payload-relative), computed here rather than via the shared codec.
fn payload_entry(payload: &[u8], fixed_size: usize, idx: usize) -> (usize, usize) {
    let s = fixed_size + 8 * idx;
    let off = u32::from_le_bytes(payload[s..s + 4].try_into().unwrap()) as usize;
    let len = u32::from_le_bytes(payload[s + 4..s + 8].try_into().unwrap()) as usize;
    (off, len)
}

// ---------------------------------------------------------------------------
// The independent decoder
// ---------------------------------------------------------------------------

/// A `FrameWalker` over the vendored `.msg` corpus — derived from the `.msg`
/// TEXT, entirely separately from the introspection data the bridge encodes
/// from. This is what makes it an oracle rather than a mirror.
fn builtin_walker() -> FrameWalker {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (package, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(package)) {
            schemas.push(s);
        }
    }
    let (walker, _warnings) = FrameWalker::new(schemas);
    walker
}

fn sample_pose(base: f64) -> CPose {
    CPose {
        position: CPoint {
            x: base,
            y: base + 1.0,
            z: base + 2.0,
        },
        orientation: CQuat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
    }
}

fn flatten_path(poses: &mut [CPoseStamped], header: CHeader) -> Vec<u8> {
    let bridge = unsafe { BridgedMessage::new(path_members()) }.expect("bridge");
    let msg = CPath {
        header,
        poses: CRosSeq {
            data: poses.as_mut_ptr() as *mut c_void,
            size: poses.len(),
            capacity: poses.len(),
        },
    };
    unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten")
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// THE headline: a `nav_msgs/Path` flattened by the rmw bridge DECODES through
/// the independent `FrameWalker` into a real element list.
///
/// With a packed body the `poses` field comes back `NestedArrayOpaque` — which is
/// precisely why an rmw robot's `/plan` would render as a text dump instead of a
/// polyline. The element VALUES are checked against the hand-written inputs,
/// so this is not merely "it decoded to something".
#[test]
fn rmw_path_decodes_through_the_independent_frame_walker() {
    let walker = builtin_walker();
    let mut poses = [
        CPoseStamped {
            header: CHeader {
                stamp: CTime { sec: 1, nanosec: 2 },
                frame_id: heap_string("odom"),
            },
            pose: sample_pose(1.0),
        },
        CPoseStamped {
            header: CHeader {
                stamp: CTime { sec: 3, nanosec: 4 },
                frame_id: heap_string("base_link"),
            },
            pose: sample_pose(10.0),
        },
    ];
    let frame = flatten_path(
        &mut poses,
        CHeader {
            stamp: CTime { sec: 5, nanosec: 6 },
            frame_id: heap_string("map"),
        },
    );

    // The bridge hashed from introspection; the walker resolves by that hash
    // against a schema parsed from the vendored .msg text. Resolving at all
    // proves the two derivations agree.
    let fv = walker
        .walk_by_hash(&frame)
        .expect("the rmw frame must resolve + walk as nav_msgs/Path");

    let poses_field = fv.field("poses").expect("poses field present");
    let elements = match poses_field {
        FrameValueKind::NestedArray { elements, .. } => elements,
        other => panic!(
            "poses must decode as a canonical element list, got {other:?} \
             (NestedArrayOpaque here means a non-canonical element body)"
        ),
    };
    assert_eq!(elements.len(), 2, "two poses in, two elements out");

    // Element values against the hand-written inputs.
    let expect = [(1i32, 2u32, "odom", 1.0f64), (3, 4, "base_link", 10.0)];
    for (i, (sec, nanosec, frame_id, base)) in expect.iter().enumerate() {
        let FrameValueKind::Nested(elem) = &elements[i] else {
            panic!("element {i} must be a decoded nested message");
        };
        let FrameValueKind::Nested(hdr) = elem.field("header").expect("header") else {
            panic!("element {i} header must decode");
        };
        assert_eq!(
            hdr.field("frame_id"),
            Some(&FrameValueKind::Str(frame_id)),
            "element {i} frame_id"
        );
        let FrameValueKind::Nested(stamp) = hdr.field("stamp").expect("stamp") else {
            panic!("element {i} stamp must decode");
        };
        assert_eq!(stamp.field("sec"), Some(&FrameValueKind::I32(*sec)));
        assert_eq!(stamp.field("nanosec"), Some(&FrameValueKind::U32(*nanosec)));

        let FrameValueKind::Nested(pose) = elem.field("pose").expect("pose") else {
            panic!("element {i} pose must decode");
        };
        let FrameValueKind::Nested(pos) = pose.field("position").expect("position") else {
            panic!("element {i} position must decode");
        };
        assert_eq!(pos.field("x"), Some(&FrameValueKind::F64(*base)));
        assert_eq!(pos.field("y"), Some(&FrameValueKind::F64(base + 1.0)));
        assert_eq!(pos.field("z"), Some(&FrameValueKind::F64(base + 2.0)));
    }

    // The Path's OWN header (a single variable-nested field, not an element)
    // still decodes — the canonical element body does not disturb the non-array path.
    let FrameValueKind::Nested(top) = fv.field("header").expect("header") else {
        panic!("top-level header must decode");
    };
    assert_eq!(top.field("frame_id"), Some(&FrameValueKind::Str("map")));
}

/// The `poses` blob, byte for byte, against a HAND-BUILT oracle.
///
/// This is the assertion a self-round-trip test can never
/// make: it fixes the exact bytes, so any framing change is a byte diff rather
/// than a silent success.
#[test]
fn rmw_pose_stamped_element_bodies_match_the_hand_oracle() {
    let pose0 = sample_pose(1.0);
    let pose1 = sample_pose(10.0);
    let mut poses = [
        CPoseStamped {
            header: CHeader {
                stamp: CTime { sec: 1, nanosec: 2 },
                frame_id: heap_string("odom"),
            },
            pose: pose0,
        },
        CPoseStamped {
            header: CHeader {
                stamp: CTime { sec: 3, nanosec: 4 },
                frame_id: heap_string("base_link"),
            },
            pose: pose1,
        },
    ];
    let frame = flatten_path(
        &mut poses,
        CHeader {
            stamp: CTime { sec: 0, nanosec: 0 },
            frame_id: heap_string("map"),
        },
    );
    let payload = &frame[WireHeader::SIZE..];

    // Path has no fixed fields, so its table sits at payload offset 0 and
    // `poses` is variable entry 1 (after `header`).
    let (off, len) = payload_entry(payload, 0, 1);
    let blob = &payload[off..off + len];

    let expected = counted_array_oracle(&[
        pose_stamped_body_oracle(
            1,
            2,
            "odom",
            &pose_fixed_oracle(&pose0.position, &pose0.orientation),
        ),
        pose_stamped_body_oracle(
            3,
            4,
            "base_link",
            &pose_fixed_oracle(&pose1.position, &pose1.orientation),
        ),
    ]);
    assert_eq!(
        blob, expected,
        "the poses blob must be the canonical encoding byte for byte"
    );

    // Spell out the discriminator: a packed body carries the
    // variable-member COUNT where the canonical body carries the OFFSET.
    let first_body_start = 4 + 4; // count + first element length prefix
    let entry_offset = u32::from_le_bytes(
        blob[first_body_start + 56..first_body_start + 60]
            .try_into()
            .unwrap(),
    );
    assert_eq!(
        entry_offset, 64,
        "bytes [56..60] of the element body must be the offset-table entry's \
         OFFSET (64); the packed encoding put the variable-member count (1) here"
    );
}

/// `tf2_msgs/TFMessage` — an element with TWO variable members, where the
/// canonical table (8N = 16 bytes) and a packed framing (4 + 4N = 12)
/// differ in LENGTH, not just content. Byte oracle plus walker decode.
#[test]
fn rmw_tf_message_two_variable_member_element_matches_the_oracle_and_walks() {
    let bridge = unsafe { BridgedMessage::new(tf_message_members()) }.expect("bridge");
    let transform = CTransform {
        translation: CPoint {
            x: 1.5,
            y: -2.5,
            z: 3.5,
        },
        rotation: CQuat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
    };
    let mut transforms = [CTransformStamped {
        header: CHeader {
            stamp: CTime { sec: 7, nanosec: 8 },
            frame_id: heap_string("odom"),
        },
        child_frame_id: heap_string("base_link"),
        transform,
    }];
    let msg = CTFMessage {
        transforms: CRosSeq {
            data: transforms.as_mut_ptr() as *mut c_void,
            size: transforms.len(),
            capacity: transforms.len(),
        },
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    let payload = &frame[WireHeader::SIZE..];

    // TFMessage has no fixed fields and one variable field.
    let (off, len) = payload_entry(payload, 0, 0);
    let blob = &payload[off..off + len];

    let expected = counted_array_oracle(&[transform_stamped_body_oracle(
        7,
        8,
        "odom",
        "base_link",
        &pose_fixed_oracle(&transform.translation, &transform.rotation),
    )]);
    assert_eq!(blob, expected, "TransformStamped element body oracle");

    // The two framings differ in LENGTH here, so the element length prefix
    // alone separates them: canonical 56 + 16 + header(8+8+4=20) + 9 = 101;
    // the packed form would be 56 + 4 + (4+20') + (4+9) with a 16-byte header
    // body = 4 fewer framing bytes per element.
    let elem_len = u32::from_le_bytes(blob[4..8].try_into().unwrap()) as usize;
    assert_eq!(
        elem_len,
        56 + 16 + 20 + 9,
        "a 2-variable-member element must carry a 16-byte offset table"
    );

    let walker = builtin_walker();
    let fv = walker
        .walk_by_hash(&frame)
        .expect("walk tf2_msgs/TFMessage");
    let FrameValueKind::NestedArray { elements, .. } =
        fv.field("transforms").expect("transforms field")
    else {
        panic!("transforms must decode as a canonical element list, not opaque");
    };
    assert_eq!(elements.len(), 1);
    let FrameValueKind::Nested(t) = &elements[0] else {
        panic!("element must decode");
    };
    assert_eq!(
        t.field("child_frame_id"),
        Some(&FrameValueKind::Str("base_link"))
    );
    let FrameValueKind::Nested(hdr) = t.field("header").expect("header") else {
        panic!("header must decode");
    };
    assert_eq!(hdr.field("frame_id"), Some(&FrameValueKind::Str("odom")));
}

/// EDGE: an empty element array. The count is 0 and there are no bodies, so
/// the walker must yield an empty list — never opaque, never a fabricated
/// element.
///
/// SCOPE: an empty array's BYTES are convention-independent (`[0u32]`
/// under both framings — the array layer is the same), so the blob assert
/// below cannot distinguish them. Against a packed encoder this test
/// still fails, but via the Path's own top-level `header` (a variable-nested
/// field, packed by such an encoder) rather than via `poses`. Its unique
/// contribution is therefore the "empty ⇒ empty list, not opaque and not a
/// fabricated element" claim, not the framing discrimination that
/// `single_element_array_byte_oracle_pins_the_ambiguous_case` owns.
#[test]
fn empty_element_array_decodes_as_an_empty_list() {
    let mut poses: [CPoseStamped; 0] = [];
    let frame = flatten_path(
        &mut poses,
        CHeader {
            stamp: CTime { sec: 0, nanosec: 0 },
            frame_id: heap_string("map"),
        },
    );
    let payload = &frame[WireHeader::SIZE..];
    let (off, len) = payload_entry(payload, 0, 1);
    assert_eq!(&payload[off..off + len], &0u32.to_le_bytes());

    let walker = builtin_walker();
    let fv = walker.walk_by_hash(&frame).expect("walk");
    let FrameValueKind::NestedArray { elements, .. } = fv.field("poses").expect("poses") else {
        panic!("an empty array must still decode as a (empty) element list");
    };
    assert!(elements.is_empty());
}

/// EDGE: a single-element array — the case where the canonical 8N table and
/// the packed `4 + 4N` framing are the SAME SIZE (8 == 8), so only the VALUE
/// of one u32 separates them. The byte oracle is the only thing that can tell
/// them apart, which is exactly why teaching the decoder both is not an option.
#[test]
fn single_element_array_byte_oracle_pins_the_ambiguous_case() {
    let pose = sample_pose(0.0);
    let mut poses = [CPoseStamped {
        header: CHeader {
            stamp: CTime { sec: 0, nanosec: 0 },
            frame_id: heap_string("odom"),
        },
        pose,
    }];
    let frame = flatten_path(
        &mut poses,
        CHeader {
            stamp: CTime { sec: 0, nanosec: 0 },
            frame_id: heap_string("map"),
        },
    );
    let payload = &frame[WireHeader::SIZE..];
    let (off, len) = payload_entry(payload, 0, 1);
    let blob = &payload[off..off + len];

    let body = pose_stamped_body_oracle(
        0,
        0,
        "odom",
        &pose_fixed_oracle(&pose.position, &pose.orientation),
    );
    assert_eq!(blob, counted_array_oracle(std::slice::from_ref(&body)));

    // The element's own header sub-frame is 20 bytes under BOTH conventions
    // (the bridge docs' worked example); they differ only at [8..12].
    let header_start = 64;
    assert_eq!(body.len() - header_start, 20);
    assert_eq!(
        &body[header_start + 8..header_start + 12],
        &16u32.to_le_bytes(),
        "the nested Header's entry OFFSET (16), where the packed form wrote \
         the variable-member count (1) — same length, one differing u32"
    );

    let walker = builtin_walker();
    let fv = walker.walk_by_hash(&frame).expect("walk");
    let FrameValueKind::NestedArray { elements, .. } = fv.field("poses").expect("poses") else {
        panic!("single-element array must decode");
    };
    assert_eq!(elements.len(), 1);
}

/// The bridge round-trips its OWN frames (encoder and decoder agree
/// with each other), and the round trip is deterministic. This is the arm the
/// self-round-trip tests also cover — kept so a change that makes the walker
/// happy while breaking rmw's own take path fails here.
#[test]
fn rmw_still_round_trips_its_own_frames_deterministically() {
    let bridge = unsafe { BridgedMessage::new(path_members()) }.expect("bridge");
    let mut poses = [CPoseStamped {
        header: CHeader {
            stamp: CTime {
                sec: 11,
                nanosec: 22,
            },
            frame_id: heap_string("odom"),
        },
        pose: sample_pose(2.0),
    }];
    let msg = CPath {
        header: CHeader {
            stamp: CTime {
                sec: 33,
                nanosec: 44,
            },
            frame_id: heap_string("map"),
        },
        poses: CRosSeq {
            data: poses.as_mut_ptr() as *mut c_void,
            size: poses.len(),
            capacity: poses.len(),
        },
    };
    let a = unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("a");
    let b = unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("b");
    assert_eq!(a, b, "same value ⇒ identical bytes (Principle #7)");

    let mut out = CPath {
        header: CHeader {
            stamp: CTime { sec: 0, nanosec: 0 },
            frame_id: CRosString {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            },
        },
        poses: CRosSeq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        },
    };
    let ok = unsafe { bridge.unflatten(&a[WireHeader::SIZE..], &mut out as *mut _ as *mut c_void) };
    assert!(ok, "the bridge must decode its own canonical frame");
    assert_eq!(out.poses.size, 1);
    let decoded = unsafe { std::slice::from_raw_parts(out.poses.data as *const CPoseStamped, 1) };
    assert_eq!(decoded[0].header.stamp.sec, 11);
    assert_eq!(decoded[0].header.stamp.nanosec, 22);
    assert_eq!(decoded[0].pose.position.x, 2.0);
    assert_eq!(decoded[0].pose.orientation.w, 1.0);
    let fid = unsafe {
        std::slice::from_raw_parts(
            decoded[0].header.frame_id.data,
            decoded[0].header.frame_id.size,
        )
    };
    assert_eq!(fid, b"odom");
}

/// A variable-nested element with THREE fixed members and INTERIOR
/// padding — the `visualization_msgs/Marker` shape.
///
/// Every other variable-nested fixture here carries exactly ONE fixed member
/// (`Header.stamp`, `PoseStamped.pose`, `TransformStamped.transform`), so the
/// encoder's and decoder's fixed-member relocation loops never advance
/// `fixed_idx` past 0: an implementation that ignored `layout.fixed_fields[i]`
/// entirely and copied the C struct's leading bytes would pass all of
/// them. `sensor_msgs/PointField` breaks that on both axes at once —
///
/// - RELOCATION: in C the members sit at `name`@0 (24 B), `offset`@24,
///   `datatype`@28, `count`@32; on the wire the fixed section holds only the
///   three fixed members, at 0, 4 and 8. Not one C offset equals its wire
///   offset, so a loop confusing the two reads garbage rather than passing.
/// - INTERIOR PADDING: `datatype` is a `u8` at wire offset 4 and `count` is
///   `u32`-aligned, so bytes [5..8] are pad — BETWEEN two fixed members, not
///   trailing — and they must be ZERO for the frame to be deterministic
///   (Principle #7), which a wholesale struct copy would not guarantee.
///
/// Oracles: hand-built bytes for the element body, plus the independent
/// `FrameWalker` decoding the values back.
#[test]
fn rmw_point_field_element_relocates_three_fixed_members_across_interior_padding() {
    // Anti-tautology on the fixture itself: if the C struct ever stopped
    // differing from the wire layout, this test would silently stop testing
    // relocation. Pin the C offsets that make it a real relocation.
    assert_eq!(std::mem::offset_of!(CPointField, offset), 24);
    assert_eq!(std::mem::offset_of!(CPointField, datatype), 28);
    assert_eq!(std::mem::offset_of!(CPointField, count), 32);

    let bridge = unsafe { BridgedMessage::new(point_cloud2_members()) }.expect("bridge");
    let mut fields = [
        CPointField {
            name: heap_string("x"),
            offset: 0,
            datatype: 7, // FLOAT32
            count: 1,
        },
        CPointField {
            name: heap_string("intensity"),
            offset: 12,
            datatype: 8, // FLOAT64
            count: 3,
        },
    ];
    let mut cloud_data = [1u8, 2, 3, 4];
    let msg = CPointCloud2 {
        header: CHeader {
            stamp: CTime {
                sec: 42,
                nanosec: 7,
            },
            frame_id: heap_string("lidar"),
        },
        height: 1,
        width: 2,
        fields: CRosSeq {
            data: fields.as_mut_ptr() as *mut c_void,
            size: fields.len(),
            capacity: fields.len(),
        },
        is_bigendian: false,
        point_step: 16,
        row_step: 32,
        data: CRosSeq {
            data: cloud_data.as_mut_ptr() as *mut c_void,
            size: cloud_data.len(),
            capacity: cloud_data.len(),
        },
        is_dense: true,
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    let payload = &frame[WireHeader::SIZE..];

    // PointCloud2's fixed section is height/width/is_bigendian/point_step/
    // row_step/is_dense — six members, so the table starts at `fixed_size` and
    // `fields` is variable entry 1 (after `header`).
    let fixed_size = 24;
    let (off, len) = payload_entry(payload, fixed_size, 1);
    let blob = &payload[off..off + len];

    let expected = counted_array_oracle(&[
        point_field_body_oracle(0, 7, 1, "x"),
        point_field_body_oracle(12, 8, 3, "intensity"),
    ]);
    assert_eq!(
        blob, expected,
        "the fields blob must be the canonical encoding byte for byte, with the \
         three fixed members RELOCATED to wire offsets 0/4/8 and zero padding at [5..8]"
    );

    // Name the padding claim explicitly rather than leaving it inside the
    // whole-blob compare: [5..8] of the first element body must be zero, not
    // whatever the C struct held between `datatype` and `count`.
    let first_body = 4 + 4; // array count + first element's length prefix
    assert_eq!(
        &blob[first_body + 5..first_body + 8],
        &[0, 0, 0],
        "interior padding between `datatype` and `count` must be zeroed"
    );

    // And the independent reader gets the VALUES back — the discriminator a
    // byte oracle alone cannot give, since it would also pass if the decoder
    // read the same wrong offsets the encoder wrote.
    let walker = builtin_walker();
    let fv = walker
        .walk_by_hash(&frame)
        .expect("walk sensor_msgs/PointCloud2");
    let FrameValueKind::NestedArray { elements, .. } = fv.field("fields").expect("fields field")
    else {
        panic!("fields must decode as a canonical element list, not opaque");
    };
    assert_eq!(elements.len(), 2);
    let expect_element = |i: usize, name: &str, offset: u32, datatype: u8, count: u32| {
        let FrameValueKind::Nested(pf) = &elements[i] else {
            panic!("element {i} must decode");
        };
        assert_eq!(pf.field("name"), Some(&FrameValueKind::Str(name)));
        assert_eq!(pf.field("offset"), Some(&FrameValueKind::U32(offset)));
        assert_eq!(pf.field("datatype"), Some(&FrameValueKind::U8(datatype)));
        assert_eq!(pf.field("count"), Some(&FrameValueKind::U32(count)));
    };
    expect_element(0, "x", 0, 7, 1);
    expect_element(1, "intensity", 12, 8, 3);

    // Round-trip through the bridge's OWN decoder too: the relocation loop runs
    // in reverse there, and it is the half that reads `layout.fixed_fields[i]`
    // per member (`fixed_idx` reaching 2 for the first time in this file).
    let mut out = CPointCloud2 {
        header: CHeader {
            stamp: CTime { sec: 0, nanosec: 0 },
            frame_id: CRosString {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            },
        },
        height: 0,
        width: 0,
        fields: CRosSeq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        },
        is_bigendian: true,
        point_step: 0,
        row_step: 0,
        data: CRosSeq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        },
        is_dense: false,
    };
    let ok = unsafe { bridge.unflatten(payload, &mut out as *mut _ as *mut c_void) };
    assert!(ok, "the bridge must decode its own canonical frame");
    assert_eq!(out.fields.size, 2);
    let decoded = unsafe { std::slice::from_raw_parts(out.fields.data as *const CPointField, 2) };
    assert_eq!(
        (decoded[0].offset, decoded[0].datatype, decoded[0].count),
        (0, 7, 1),
        "element 0's three fixed members must land back at their C offsets"
    );
    assert_eq!(
        (decoded[1].offset, decoded[1].datatype, decoded[1].count),
        (12, 8, 3),
        "element 1's three fixed members must land back at their C offsets"
    );
    let name1 = unsafe { std::slice::from_raw_parts(decoded[1].name.data, decoded[1].name.size) };
    assert_eq!(name1, b"intensity");
}

/// GUARD: this file's hand-typed introspection ids must equal the
/// bridge's own `ros_type` table.
///
/// The fixtures re-type the rosidl `field_types.h` numbers as local constants,
/// and a wrong one is close to invisible: a `ROS_TYPE_INT32` reading `6` (BOOLEAN)
/// goes unnoticed while the only place it is used sits inside a
/// recursively-FIXED nested message (copied wholesale — no byte moves) whose
/// parent folds in just its qualified NAME (recipe 3 — no hash moves). Byte
/// oracles and the `walk_by_hash` gate are both structurally blind to it; the
/// error is reachable only when a VARIABLE element has its own fixed
/// section. This is the cheap standing check that a wrong constant fails loudly.
#[test]
fn fixture_introspection_ids_match_the_bridge_table() {
    use rmw_cerulion::type_bridge::ros_type;
    assert_eq!(ROS_TYPE_BOOL, ros_type::BOOLEAN, "bool");
    assert_eq!(ROS_TYPE_DOUBLE, ros_type::DOUBLE, "float64");
    assert_eq!(ROS_TYPE_UINT8, ros_type::UINT8, "uint8");
    assert_eq!(ROS_TYPE_UINT32, ros_type::UINT32, "uint32");
    assert_eq!(ROS_TYPE_INT32, ros_type::INT32, "int32");
    assert_eq!(ROS_TYPE_STRING, ros_type::STRING, "string");
    assert_eq!(ROS_TYPE_MESSAGE, ros_type::MESSAGE, "nested message");
}
