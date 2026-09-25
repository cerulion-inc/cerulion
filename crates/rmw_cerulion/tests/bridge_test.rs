// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! Type-bridge tests with HAND-BUILT rosidl introspection data
//! — no ROS installation required.
//!
//! The decisive assertions are the INTEROP ones: frames flattened by
//! the bridge from C structs must decode through the NATIVE generated
//! readers (`native_ros2_messages`), and vice versa — that is the
//! contract that makes `cerulion topic echo` work on MoveIt traffic
//! and native nodes interoperate with ROS 2 nodes.

use std::ffi::CString;
use std::os::raw::c_void;

use rmw_cerulion::ffi::{
    rosidl_message_type_support_t, rosidl_typesupport_introspection_c__MessageMember,
    rosidl_typesupport_introspection_c__MessageMembers,
};
use rmw_cerulion::type_bridge::BridgedMessage;

use cerulion_core::wire::WireHeader;

// Introspection type ids.
const ROS_TYPE_DOUBLE: u8 = 2;
const ROS_TYPE_UINT32: u8 = 12;
const ROS_TYPE_STRING: u8 = 16;
const ROS_TYPE_MESSAGE: u8 = 18;

/// Leak a C string (test fixtures live for the process).
fn cstr(s: &str) -> *const std::os::raw::c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn member(
    name: &str,
    type_id: u8,
    offset: u32,
    is_array: bool,
    array_size: usize,
    nested: *const rosidl_message_type_support_t,
) -> rosidl_typesupport_introspection_c__MessageMember {
    rosidl_typesupport_introspection_c__MessageMember {
        name_: cstr(name),
        type_id_: type_id,
        members_: nested,
        is_array_: is_array,
        array_size_: array_size,
        is_upper_bound_: false,
        offset_: offset,
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

fn make_typesupport(
    members: *const rosidl_typesupport_introspection_c__MessageMembers,
) -> *const rosidl_message_type_support_t {
    let ts = rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: members as *const c_void,
        ..Default::default()
    };
    Box::leak(Box::new(ts))
}

// =====================================================================
// Fixtures: C structs exactly as rosidl generates them
// =====================================================================

#[repr(C)]
#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct CVector3 {
    x: f64,
    y: f64,
    z: f64,
}

#[repr(C)]
#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct CQuaternion {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

#[repr(C)]
#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct CTransform {
    translation: CVector3,
    rotation: CQuaternion,
}

#[repr(C)]
struct CRosString {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

#[repr(C)]
struct CDoubleSeq {
    data: *mut f64,
    size: usize,
    capacity: usize,
    #[cfg(cerulion_has_is_rosidl_buffer)]
    is_rosidl_buffer: bool,
    #[cfg(cerulion_has_is_rosidl_buffer)]
    owns_rosidl_buffer: bool,
}

/// std_msgs/String-like: { data: string }
#[repr(C)]
struct CStringMsg {
    data: CRosString,
}

/// sensor_msgs/JointState-flavored subset: { stamp_sec: u32,
/// position: f64[] , name: string }
#[repr(C)]
struct CJointish {
    stamp_sec: u32,
    position: CDoubleSeq,
    name: CRosString,
}

fn vector3_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "geometry_msgs__msg",
        "Vector3",
        std::mem::size_of::<CVector3>(),
        vec![
            member("x", ROS_TYPE_DOUBLE, 0, false, 0, std::ptr::null()),
            member("y", ROS_TYPE_DOUBLE, 8, false, 0, std::ptr::null()),
            member("z", ROS_TYPE_DOUBLE, 16, false, 0, std::ptr::null()),
        ],
    )
}

fn quaternion_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "geometry_msgs__msg",
        "Quaternion",
        std::mem::size_of::<CQuaternion>(),
        vec![
            member("x", ROS_TYPE_DOUBLE, 0, false, 0, std::ptr::null()),
            member("y", ROS_TYPE_DOUBLE, 8, false, 0, std::ptr::null()),
            member("z", ROS_TYPE_DOUBLE, 16, false, 0, std::ptr::null()),
            member("w", ROS_TYPE_DOUBLE, 24, false, 0, std::ptr::null()),
        ],
    )
}

fn transform_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let v_ts = make_typesupport(vector3_members());
    let q_ts = make_typesupport(quaternion_members());
    make_members(
        "geometry_msgs__msg",
        "Transform",
        std::mem::size_of::<CTransform>(),
        vec![
            member("translation", ROS_TYPE_MESSAGE, 0, false, 0, v_ts),
            member("rotation", ROS_TYPE_MESSAGE, 24, false, 0, q_ts),
        ],
    )
}

// =====================================================================
// Fixed-message bridging
// =====================================================================

#[test]
fn fixed_message_bridges_with_zero_copy_loan() {
    use cerulion_core::message::ShmMessage;
    let bridge = unsafe { BridgedMessage::new(vector3_members()) }.expect("bridge");
    assert_eq!(bridge.qualified_name, "geometry_msgs/Vector3");
    // recipe-3: the bridge hash must equal the NATIVE generated
    // SCHEMA_HASH (byte-compatible interop), not a name-only literal.
    assert_eq!(
        bridge.schema_hash(),
        <native_ros2_messages::geometry_msgs::Vector3 as ShmMessage>::SCHEMA_HASH
    );
    assert!(bridge.layout.is_fixed());
    assert_eq!(bridge.layout.fixed_size, 24);
    assert_eq!(bridge.c_size, 24);
    assert!(
        bridge.can_loan,
        "C layout ≡ wire layout for all-primitive fixed messages"
    );
}

#[test]
fn nested_fixed_message_bridges_and_roundtrips() {
    use cerulion_core::message::ShmMessage;
    let bridge = unsafe { BridgedMessage::new(transform_members()) }.expect("bridge");
    assert_eq!(bridge.qualified_name, "geometry_msgs/Transform");
    assert!(bridge.layout.is_fixed());
    assert_eq!(bridge.layout.fixed_size, 56);
    assert!(bridge.can_loan);

    let msg = CTransform {
        translation: CVector3 {
            x: 1.5,
            y: -2.5,
            z: 3.25,
        },
        rotation: CQuaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 7, 99) }.expect("flatten");
    assert_eq!(frame.len(), WireHeader::SIZE + 56);

    let header = WireHeader::read_from_buf(&frame).expect("header");
    // recipe-3: bridged frame carries the NATIVE generated SCHEMA_HASH.
    assert_eq!(
        header.schema_hash,
        <native_ros2_messages::geometry_msgs::Transform as ShmMessage>::SCHEMA_HASH
    );
    assert_eq!(header.total_size as usize, frame.len());
    assert_eq!(header.sequence, 7);
    assert_eq!(header.timestamp_ns, 99);

    let mut out = CTransform::default();
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out, msg);
}

/// INTEROP: a frame flattened from a C Transform must read correctly
/// through the NATIVE generated TransformShm (the `cerulion topic echo`
/// / native-subscriber path).
#[test]
fn bridged_frame_decodes_through_native_generated_reader() {
    use native_ros2_messages::geometry_msgs::TransformShm;

    let bridge = unsafe { BridgedMessage::new(transform_members()) }.expect("bridge");
    // The runtime hash must equal the generated SCHEMA_HASH.
    use cerulion_core::message::ShmMessage;
    assert_eq!(
        bridge.schema_hash(),
        <native_ros2_messages::geometry_msgs::Transform as ShmMessage>::SCHEMA_HASH
    );

    let msg = CTransform {
        translation: CVector3 {
            x: 10.0,
            y: 20.0,
            z: 30.0,
        },
        rotation: CQuaternion {
            x: 0.1,
            y: 0.2,
            z: 0.3,
            w: 0.9,
        },
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let native = TransformShm::from_bytes(&frame[WireHeader::SIZE..]);
    assert_eq!(native.translation.x, 10.0);
    assert_eq!(native.translation.y, 20.0);
    assert_eq!(native.translation.z, 30.0);
    assert_eq!(native.rotation.w, 0.9);
}

// =====================================================================
// Variable-message bridging
// =====================================================================

fn string_msg_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "std_msgs__msg",
        "String",
        std::mem::size_of::<CStringMsg>(),
        vec![member(
            "data",
            ROS_TYPE_STRING,
            0,
            false,
            0,
            std::ptr::null(),
        )],
    )
}

fn jointish_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "test_msgs__msg",
        "Jointish",
        std::mem::size_of::<CJointish>(),
        vec![
            member("stamp_sec", ROS_TYPE_UINT32, 0, false, 0, std::ptr::null()),
            member(
                "position",
                ROS_TYPE_DOUBLE,
                std::mem::offset_of!(CJointish, position) as u32,
                true,
                0,
                std::ptr::null(),
            ),
            member(
                "name",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CJointish, name) as u32,
                false,
                0,
                std::ptr::null(),
            ),
        ],
    )
}

fn ros_string(s: &str) -> CRosString {
    let mut buf = s.as_bytes().to_vec();
    buf.push(0);
    let mut boxed = buf.into_boxed_slice();
    let data = boxed.as_mut_ptr();
    std::mem::forget(boxed);
    CRosString {
        data,
        size: s.len(),
        capacity: s.len() + 1,
    }
}

fn double_seq(values: &[f64]) -> CDoubleSeq {
    let mut boxed = values.to_vec().into_boxed_slice();
    let data = boxed.as_mut_ptr();
    let len = boxed.len();
    std::mem::forget(boxed);
    CDoubleSeq {
        data,
        size: len,
        capacity: len,
        #[cfg(cerulion_has_is_rosidl_buffer)]
        is_rosidl_buffer: false,
        #[cfg(cerulion_has_is_rosidl_buffer)]
        owns_rosidl_buffer: false,
    }
}

#[test]
fn string_message_flattens_to_native_compatible_frame() {
    use cerulion_core::message::ShmMessage;
    use native_ros2_messages::std_msgs::{String as NativeString, StringShm};

    let bridge = unsafe { BridgedMessage::new(string_msg_members()) }.expect("bridge");
    assert!(!bridge.layout.is_fixed());
    assert!(!bridge.can_loan);
    assert_eq!(
        bridge.schema_hash(),
        <NativeString as ShmMessage>::SCHEMA_HASH
    );

    let msg = CStringMsg {
        data: ros_string("hello cerulion"),
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    // Native reader decodes the bridged frame.
    let native = StringShm::from_bytes(&frame[WireHeader::SIZE..]);
    assert_eq!(native.data().expect("utf8"), "hello cerulion");

    // And the bridge unflattens its own frame into a fresh C message.
    let mut out = CStringMsg {
        data: CRosString {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        },
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    let round = unsafe { std::slice::from_raw_parts(out.data.data, out.data.size) };
    assert_eq!(round, b"hello cerulion");
}

#[test]
fn mixed_fixed_and_variable_fields_roundtrip() {
    let bridge = unsafe { BridgedMessage::new(jointish_members()) }.expect("bridge");
    assert!(!bridge.layout.is_fixed());
    // stamp_sec is the only fixed-section field.
    assert_eq!(bridge.layout.fixed_fields.len(), 1);
    assert_eq!(bridge.layout.variable_fields.len(), 2);

    let msg = CJointish {
        stamp_sec: 42,
        position: double_seq(&[1.0, 2.5, -3.5]),
        name: ros_string("panda_joint1"),
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut out = CJointish {
        stamp_sec: 0,
        position: CDoubleSeq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
        name: CRosString {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        },
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out.stamp_sec, 42);
    let pos = unsafe { std::slice::from_raw_parts(out.position.data, out.position.size) };
    assert_eq!(pos, &[1.0, 2.5, -3.5]);
    let name = unsafe { std::slice::from_raw_parts(out.name.data, out.name.size) };
    assert_eq!(name, b"panda_joint1");
}

/// Determinism: same message, same counters → bit-identical frames
/// (Principle #7 — the bridge contains no clocks, no RNG, no iteration
/// over unordered containers).
#[test]
fn flatten_is_bit_identical_across_runs() {
    let bridge = unsafe { BridgedMessage::new(jointish_members()) }.expect("bridge");
    let msg = CJointish {
        stamp_sec: 7,
        position: double_seq(&[0.25; 8]),
        name: ros_string("determinism"),
    };
    let a = unsafe { bridge.flatten(&msg as *const _ as *const c_void, 3, 1000) }.expect("flatten");
    let b = unsafe { bridge.flatten(&msg as *const _ as *const c_void, 3, 1000) }.expect("flatten");
    assert_eq!(a, b);
}

/// Malformed frames are rejected loudly, never UB: truncated payloads
/// and lying offset tables return false.
#[test]
fn malformed_frames_are_rejected() {
    let bridge = unsafe { BridgedMessage::new(jointish_members()) }.expect("bridge");
    let mut out = CJointish {
        stamp_sec: 0,
        position: CDoubleSeq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
        name: CRosString {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        },
    };

    // Too short for fixed section + offset table.
    let ok = unsafe { bridge.unflatten(&[0u8; 4], &mut out as *mut _ as *mut c_void) };
    assert!(!ok);

    // Offset table pointing past the payload.
    let msg = CJointish {
        stamp_sec: 1,
        position: double_seq(&[1.0]),
        name: ros_string("x"),
    };
    let mut frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    // Corrupt the first offset entry's length to absurdity.
    let table = WireHeader::SIZE + bridge.layout.fixed_size;
    frame[table + 4..table + 8].copy_from_slice(&u32::MAX.to_le_bytes());
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(!ok);
}

// =====================================================================
// Robustness: padding determinism, corrupt-header
// guards, complex-codec coverage (sequences of strings / fixed nested /
// variable nested), hostile-count decode rejection.
// =====================================================================

const ROS_TYPE_UINT8: u8 = 8;
const ROS_TYPE_UINT64: u8 = 14;

/// Nested struct WITH repr(C) padding: 7 pad bytes between `flag` and
/// `value`, none trailing.
#[repr(C)]
#[derive(Default, Clone, Copy)]
struct CPadded {
    flag: u8,
    value: u64,
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct CPadOuter {
    inner: CPadded,
}

fn padded_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "test_msgs__msg",
        "Padded",
        std::mem::size_of::<CPadded>(),
        vec![
            member("flag", ROS_TYPE_UINT8, 0, false, 0, std::ptr::null()),
            member("value", ROS_TYPE_UINT64, 8, false, 0, std::ptr::null()),
        ],
    )
}

fn pad_outer_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let p_ts = make_typesupport(padded_members());
    make_members(
        "test_msgs__msg",
        "PadOuter",
        std::mem::size_of::<CPadOuter>(),
        vec![member("inner", ROS_TYPE_MESSAGE, 0, false, 0, p_ts)],
    )
}

/// Uninitialized C-side padding must NOT leak into the wire frame —
/// two messages with identical field values but different garbage in
/// their padding bytes must flatten to byte-identical frames with
/// zeroed padding (Principle #7: replay = live, byte-for-byte).
#[test]
fn struct_padding_is_zeroed_for_deterministic_frames() {
    assert_eq!(
        std::mem::size_of::<CPadded>(),
        16,
        "fixture assumes 7 pad bytes at 1..8"
    );
    let bridge = unsafe { BridgedMessage::new(pad_outer_members()) }.expect("bridge");

    // Two buffers: same field values, DIFFERENT garbage in padding.
    let make_poisoned = |poison: u8| -> [u8; 16] {
        let mut buf = [poison; 16];
        buf[0] = 1; // flag
        buf[8..16].copy_from_slice(&0xDEAD_BEEF_u64.to_le_bytes());
        buf
    };
    let a = make_poisoned(0xAA);
    let b = make_poisoned(0x55);

    let fa = unsafe { bridge.flatten(a.as_ptr() as *const c_void, 1, 2) }.expect("flatten a");
    let fb = unsafe { bridge.flatten(b.as_ptr() as *const c_void, 1, 2) }.expect("flatten b");
    assert_eq!(fa, fb, "padding garbage leaked into the wire frame");
    // Padding bytes (payload offsets 1..8) must be zero.
    assert_eq!(&fa[WireHeader::SIZE + 1..WireHeader::SIZE + 8], &[0u8; 7]);
}

/// A corrupt C sequence header (garbage size) must produce an
/// encode ERROR — not a panic, OOM abort, or out-of-bounds slice.
#[test]
fn corrupt_sequence_header_errors_instead_of_panicking() {
    let bridge = unsafe { BridgedMessage::new(jointish_members()) }.expect("bridge");
    let msg = CJointish {
        stamp_sec: 1,
        position: CDoubleSeq {
            data: std::ptr::NonNull::<f64>::dangling().as_ptr(),
            size: usize::MAX, // corrupt
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
        name: CRosString {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        },
    };
    let res = unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) };
    let err = res.expect_err("corrupt size must error");
    assert!(err.to_string().contains("encode failed"), "got: {err}");
}

// ---------------------------------------------------------------------
// Complex codec fixtures
// ---------------------------------------------------------------------

/// { names: string[] } — sequence of strings (Complex op).
#[repr(C)]
struct CStringSeqMsg {
    names: CPrimSeqRaw,
}

/// Generic rosidl sequence header for complex tests.
#[repr(C)]
struct CRosSeqRaw {
    data: *mut c_void,
    size: usize,
    capacity: usize,
}

/// Primitive and string sequences carry the two Lyrical Buffer flags after
/// the header (message sequences do not); the flags exist on the era the
/// crate is built for, so this fixture is exactly the distro's struct.
#[repr(C)]
struct CPrimSeqRaw {
    data: *mut c_void,
    size: usize,
    capacity: usize,
    #[cfg(cerulion_has_is_rosidl_buffer)]
    is_rosidl_buffer: bool,
    #[cfg(cerulion_has_is_rosidl_buffer)]
    owns_rosidl_buffer: bool,
}

fn string_seq_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "test_msgs__msg",
        "StringSeq",
        std::mem::size_of::<CStringSeqMsg>(),
        vec![member(
            "names",
            ROS_TYPE_STRING,
            0,
            true,
            0,
            std::ptr::null(),
        )],
    )
}

/// { points: geometry_msgs/Vector3[] } — sequence of FIXED nested
/// (back-to-back structs codec).
#[repr(C)]
struct CPointSeqMsg {
    points: CRosSeqRaw,
}

fn point_seq_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let v_ts = make_typesupport(vector3_members());
    make_members(
        "test_msgs__msg",
        "PointSeq",
        std::mem::size_of::<CPointSeqMsg>(),
        vec![member("points", ROS_TYPE_MESSAGE, 0, true, 0, v_ts)],
    )
}

/// { items: std_msgs/String[] } — sequence of VARIABLE nested
/// (count + per-element length-prefixed codec).
#[repr(C)]
struct CItemSeqMsg {
    items: CRosSeqRaw,
}

fn item_seq_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let s_ts = make_typesupport(string_msg_members());
    make_members(
        "test_msgs__msg",
        "ItemSeq",
        std::mem::size_of::<CItemSeqMsg>(),
        vec![member("items", ROS_TYPE_MESSAGE, 0, true, 0, s_ts)],
    )
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

fn read_string(s: &CRosString) -> String {
    if s.data.is_null() {
        return String::new();
    }
    unsafe { String::from_utf8_lossy(std::slice::from_raw_parts(s.data, s.size)).into_owned() }
}

#[test]
fn complex_codec_sequence_of_strings_roundtrips() {
    let bridge = unsafe { BridgedMessage::new(string_seq_members()) }.expect("bridge");

    let mut elems: Vec<CRosString> =
        vec![heap_string("alpha"), heap_string(""), heap_string("γréek")];
    let msg = CStringSeqMsg {
        names: CPrimSeqRaw {
            data: elems.as_mut_ptr() as *mut c_void,
            size: elems.len(),
            capacity: elems.len(),
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut out = CStringSeqMsg {
        names: CPrimSeqRaw {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out.names.size, 3);
    let decoded = unsafe { std::slice::from_raw_parts(out.names.data as *const CRosString, 3) };
    assert_eq!(read_string(&decoded[0]), "alpha");
    assert_eq!(read_string(&decoded[1]), "");
    assert_eq!(read_string(&decoded[2]), "γréek");
}

#[test]
fn complex_codec_sequence_of_fixed_nested_roundtrips() {
    let bridge = unsafe { BridgedMessage::new(point_seq_members()) }.expect("bridge");

    let mut pts = vec![
        CVector3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
        CVector3 {
            x: -4.5,
            y: 0.0,
            z: 9.75,
        },
    ];
    let msg = CPointSeqMsg {
        points: CRosSeqRaw {
            data: pts.as_mut_ptr() as *mut c_void,
            size: pts.len(),
            capacity: pts.len(),
        },
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut out = CPointSeqMsg {
        points: CRosSeqRaw {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        },
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out.points.size, 2);
    let decoded = unsafe { std::slice::from_raw_parts(out.points.data as *const CVector3, 2) };
    assert_eq!(decoded[0], pts[0]);
    assert_eq!(decoded[1], pts[1]);
}

#[test]
fn complex_codec_sequence_of_variable_nested_roundtrips() {
    let bridge = unsafe { BridgedMessage::new(item_seq_members()) }.expect("bridge");

    let mut items = vec![
        CStringMsg {
            data: heap_string("first"),
        },
        CStringMsg {
            data: heap_string("second, longer payload"),
        },
    ];
    let msg = CItemSeqMsg {
        items: CRosSeqRaw {
            data: items.as_mut_ptr() as *mut c_void,
            size: items.len(),
            capacity: items.len(),
        },
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut out = CItemSeqMsg {
        items: CRosSeqRaw {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        },
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out.items.size, 2);
    let decoded = unsafe { std::slice::from_raw_parts(out.items.data as *const CStringMsg, 2) };
    assert_eq!(read_string(&decoded[0].data), "first");
    assert_eq!(read_string(&decoded[1].data), "second, longer payload");
}

/// Determinism through the complex codec: same value → identical bytes
/// across repeated flattens (heap addresses differ run to run; content
/// must not).
#[test]
fn complex_codec_flatten_is_deterministic() {
    let bridge = unsafe { BridgedMessage::new(string_seq_members()) }.expect("bridge");
    let mut elems = vec![heap_string("repeat")];
    let msg = CStringSeqMsg {
        names: CPrimSeqRaw {
            data: elems.as_mut_ptr() as *mut c_void,
            size: 1,
            capacity: 1,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    let a = unsafe { bridge.flatten(&msg as *const _ as *const c_void, 5, 50) }.expect("a");
    let b = unsafe { bridge.flatten(&msg as *const _ as *const c_void, 5, 50) }.expect("b");
    assert_eq!(a, b);
}

/// Hostile decode: a frame claiming u32::MAX string elements must be
/// rejected BEFORE any allocation sized by that count.
#[test]
fn hostile_element_count_is_rejected_on_decode() {
    let bridge = unsafe { BridgedMessage::new(string_seq_members()) }.expect("bridge");

    // Well-formed empty message → valid frame, then poison the count.
    let msg = CStringSeqMsg {
        names: CPrimSeqRaw {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    let mut frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    // The complex entry's payload starts with the u32 element count —
    // find it via the offset table (single variable field, entry 0).
    let payload = &frame[WireHeader::SIZE..];
    let table_base = bridge.layout.fixed_size;
    let off = u32::from_le_bytes(payload[table_base..table_base + 4].try_into().unwrap()) as usize;
    let abs = WireHeader::SIZE + off;
    frame[abs..abs + 4].copy_from_slice(&u32::MAX.to_le_bytes());

    let mut out = CStringSeqMsg {
        names: CPrimSeqRaw {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(!ok, "hostile count must be rejected");
    assert_eq!(out.names.size, 0, "no allocation for hostile count");
}

// =====================================================================
// Header-class direct variable-nested members
// (the MoveIt workload), complex-codec padding, alignment interop,
// deterministic hostile-count rejection.
// =====================================================================

const ROS_TYPE_INT32: u8 = 13;
const ROS_TYPE_FLOAT: u8 = 1;

/// builtin_interfaces/Time: { sec: i32, nanosec: u32 } — fixed.
#[repr(C)]
#[derive(Default, Clone, Copy)]
struct CTime {
    sec: i32,
    nanosec: u32,
}

/// std_msgs/Header: { stamp: Time, frame_id: string } — VARIABLE, and a
/// DIRECT (non-array) variable nested member of every stamped message.
#[repr(C)]
struct CHeader {
    stamp: CTime,
    frame_id: CRosString,
}

/// PoseStamped-shaped: { header: Header, x: f64 } — the exact shape that
/// fails registration under a non-recursive classification
/// ("fixed-field order mismatch: layout 'x' vs introspection 'header'").
#[repr(C)]
struct CStamped {
    header: CHeader,
    x: f64,
}

fn time_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "builtin_interfaces__msg",
        "Time",
        std::mem::size_of::<CTime>(),
        vec![
            member("sec", ROS_TYPE_INT32, 0, false, 0, std::ptr::null()),
            member("nanosec", ROS_TYPE_UINT32, 4, false, 0, std::ptr::null()),
        ],
    )
}

fn header_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let t_ts = make_typesupport(time_members());
    make_members(
        "std_msgs__msg",
        "Header",
        std::mem::size_of::<CHeader>(),
        vec![
            member("stamp", ROS_TYPE_MESSAGE, 0, false, 0, t_ts),
            member(
                "frame_id",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CHeader, frame_id) as u32,
                false,
                0,
                std::ptr::null(),
            ),
        ],
    )
}

fn stamped_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let h_ts = make_typesupport(header_members());
    make_members(
        "test_msgs__msg",
        "Stamped",
        std::mem::size_of::<CStamped>(),
        vec![
            member("header", ROS_TYPE_MESSAGE, 0, false, 0, h_ts),
            member(
                "x",
                ROS_TYPE_DOUBLE,
                std::mem::offset_of!(CStamped, x) as u32,
                false,
                0,
                std::ptr::null(),
            ),
        ],
    )
}

/// The MoveIt workload: a message with a direct
/// variable nested member (std_msgs/Header) must register and roundtrip.
/// A non-recursive fixed/variable classification fails this at BridgedMessage::new.
#[test]
fn header_bearing_message_registers_and_roundtrips() {
    let bridge = unsafe { BridgedMessage::new(stamped_members()) }
        .expect("Header-bearing message MUST register (the MoveIt workload)");
    assert!(!bridge.can_loan, "variable message");

    let msg = CStamped {
        header: CHeader {
            stamp: CTime {
                sec: 7,
                nanosec: 99,
            },
            frame_id: heap_string("panda_link0"),
        },
        x: 1.25,
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut out = CStamped {
        header: CHeader {
            stamp: CTime::default(),
            frame_id: CRosString {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            },
        },
        x: 0.0,
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out.header.stamp.sec, 7);
    assert_eq!(out.header.stamp.nanosec, 99);
    assert_eq!(read_string(&out.header.frame_id), "panda_link0");
    assert_eq!(out.x, 1.25);
}

/// The bare Header itself must ALSO interop with the native generated
/// reader — same FQN hash, same payload layout.
#[test]
fn header_bridges_native_compatible() {
    use cerulion_core::message::ShmMessage;
    use native_ros2_messages::std_msgs::HeaderShm;

    let bridge = unsafe { BridgedMessage::new(header_members()) }.expect("bridge");
    assert_eq!(
        bridge.schema_hash(),
        <native_ros2_messages::std_msgs::Header as ShmMessage>::SCHEMA_HASH
    );

    let msg = CHeader {
        stamp: CTime {
            sec: 11,
            nanosec: 22,
        },
        frame_id: heap_string("base_link"),
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    let native = HeaderShm::from_bytes(&frame[WireHeader::SIZE..]);
    assert_eq!(native.frame_id().expect("frame_id"), "base_link");
}

/// Padding gap: padding through the complex codec (sequence of padded
/// fixed nested structs hits zero_struct_padding, not pad_ranges).
#[test]
fn complex_codec_zeroes_padding_in_fixed_nested_sequences() {
    let bridge = unsafe { BridgedMessage::new(padded_seq_members()) }.expect("bridge");

    let make_poisoned = |poison: u8| -> [u8; 16] {
        let mut buf = [poison; 16];
        buf[0] = 1;
        buf[8..16].copy_from_slice(&0xFEED_F00D_u64.to_le_bytes());
        buf
    };
    let mut a_elems = [make_poisoned(0xAA)];
    let mut b_elems = [make_poisoned(0x55)];
    let msg_a = CPadSeqMsg {
        items: CRosSeqRaw {
            data: a_elems.as_mut_ptr() as *mut c_void,
            size: 1,
            capacity: 1,
        },
    };
    let msg_b = CPadSeqMsg {
        items: CRosSeqRaw {
            data: b_elems.as_mut_ptr() as *mut c_void,
            size: 1,
            capacity: 1,
        },
    };
    let fa = unsafe { bridge.flatten(&msg_a as *const _ as *const c_void, 0, 0) }.expect("a");
    let fb = unsafe { bridge.flatten(&msg_b as *const _ as *const c_void, 0, 0) }.expect("b");
    assert_eq!(
        fa, fb,
        "complex-codec padding garbage leaked into the frame"
    );
}

#[repr(C)]
struct CPadSeqMsg {
    items: CRosSeqRaw,
}

fn padded_seq_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let p_ts = make_typesupport(padded_members());
    make_members(
        "test_msgs__msg",
        "PadSeq",
        std::mem::size_of::<CPadSeqMsg>(),
        vec![member("items", ROS_TYPE_MESSAGE, 0, true, 0, p_ts)],
    )
}

/// Alignment-interop lock: odd-length string followed by an
/// aligned primitive sequence — the bridge's alignment padding rule must
/// match the native generated reader, not just its own decoder.
/// sensor_msgs/ChannelFloat32 = { string name; float32[] values }.
#[test]
fn channel_float32_alignment_interops_with_native_reader() {
    use cerulion_core::message::ShmMessage;
    use native_ros2_messages::sensor_msgs::ChannelFloat32Shm;

    #[repr(C)]
    struct CChannel {
        name: CRosString,
        values: CPrimSeqRaw,
    }
    let members = make_members(
        "sensor_msgs__msg",
        "ChannelFloat32",
        std::mem::size_of::<CChannel>(),
        vec![
            member("name", ROS_TYPE_STRING, 0, false, 0, std::ptr::null()),
            member(
                "values",
                ROS_TYPE_FLOAT,
                std::mem::offset_of!(CChannel, values) as u32,
                true,
                0,
                std::ptr::null(),
            ),
        ],
    );
    let bridge = unsafe { BridgedMessage::new(members) }.expect("bridge");
    assert_eq!(
        bridge.schema_hash(),
        <native_ros2_messages::sensor_msgs::ChannelFloat32 as ShmMessage>::SCHEMA_HASH
    );

    let mut values = [1.5f32, -2.5, 4.25];
    let msg = CChannel {
        name: heap_string("rgb"), // odd length 3 → alignment padding needed
        values: CPrimSeqRaw {
            data: values.as_mut_ptr() as *mut c_void,
            size: values.len(),
            capacity: values.len(),
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let native = ChannelFloat32Shm::from_bytes(&frame[WireHeader::SIZE..]);
    assert_eq!(native.name().expect("name"), "rgb");
    assert_eq!(native.values(), &values[..]);
}

/// Deterministic hostile-count rejection (no overcommit dependence):
/// count=1000 with an empty remainder must be rejected; without the
/// count-vs-bytes guard a 16 KB calloc would SUCCEED everywhere and
/// out.names.size would become 1000.
#[test]
fn hostile_count_with_small_buffer_is_rejected_deterministically() {
    let bridge = unsafe { BridgedMessage::new(string_seq_members()) }.expect("bridge");
    let msg = CStringSeqMsg {
        names: CPrimSeqRaw {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    let mut frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    let payload = &frame[WireHeader::SIZE..];
    let table_base = bridge.layout.fixed_size;
    let off = u32::from_le_bytes(payload[table_base..table_base + 4].try_into().unwrap()) as usize;
    let abs = WireHeader::SIZE + off;
    frame[abs..abs + 4].copy_from_slice(&1000u32.to_le_bytes());

    let mut out = CStringSeqMsg {
        names: CPrimSeqRaw {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(!ok);
    assert_eq!(
        out.names.size, 0,
        "guard must reject before prepare_sequence"
    );
}

// =====================================================================
// Flatten-into-loan: frame_size pre-pass + flatten_into
// (bounds-checked cursor). Byte identity with the heap flatten, no
// out-of-bounds writes on lying buffer sizes, hostile counts rejected
// in the size pre-pass.
// =====================================================================

/// Run frame_size + flatten_into over a POISONED buffer and assert the
/// result is byte-identical to the heap flatten. The poison fill proves
/// every output byte is deterministically initialized by the encode
/// (head zeroing + field ops + alignment padding) — none of the prior
/// buffer contents can leak through (the uninit-SHM-loan obligation).
fn assert_flatten_into_identical(bridge: &BridgedMessage, msg: *const c_void) -> Vec<u8> {
    let expected = unsafe { bridge.flatten(msg, 5, 777) }.expect("flatten");
    let size = unsafe { bridge.frame_size(msg) }.expect("frame_size");
    assert_eq!(size, expected.len(), "frame_size must equal flatten length");
    let mut buf = vec![0xA5u8; size];
    let n = unsafe { bridge.flatten_into(msg, 5, 777, &mut buf) }.expect("flatten_into");
    assert_eq!(n, size, "cursor must end exactly at the pre-pass size");
    assert_eq!(
        buf, expected,
        "flatten_into must be byte-identical to flatten"
    );
    // Hardening: complementary poison. flatten() shares the
    // cursor code path, so buf==expected alone is self-comparison; an
    // unwritten byte could escape if the heap behind flatten()'s Vec
    // happened to match the poison. Two runs over COMPLEMENTARY
    // poisons must agree at every byte — a deterministic detector
    // for any byte the encode failed to initialize.
    let mut buf2 = vec![0x5Au8; size];
    let n2 = unsafe { bridge.flatten_into(msg, 5, 777, &mut buf2) }.expect("flatten_into #2");
    assert_eq!(n2, size);
    assert_eq!(
        buf, buf2,
        "complementary-poison runs must be byte-identical (unwritten byte detected)"
    );
    expected
}

#[test]
fn flatten_into_matches_flatten_for_fixed_message() {
    let bridge = unsafe { BridgedMessage::new(transform_members()) }.expect("bridge");
    let msg = CTransform {
        translation: CVector3 {
            x: 1.5,
            y: -2.5,
            z: 3.25,
        },
        rotation: CQuaternion {
            x: 0.1,
            y: 0.2,
            z: 0.3,
            w: 0.9,
        },
    };
    let frame = assert_flatten_into_identical(&bridge, &msg as *const _ as *const c_void);
    assert_eq!(frame.len(), WireHeader::SIZE + 56);
}

/// uint8[] payload — the Image / PointCloud2 / UInt8MultiArray shape
/// the loan path was built for.
#[test]
fn flatten_into_matches_flatten_for_uint8_payload() {
    #[repr(C)]
    struct CByteSeqMsg {
        stamp: u32,
        data: CPrimSeqRaw,
    }
    let members = make_members(
        "test_msgs__msg",
        "ByteSeq",
        std::mem::size_of::<CByteSeqMsg>(),
        vec![
            member("stamp", ROS_TYPE_UINT32, 0, false, 0, std::ptr::null()),
            member(
                "data",
                ROS_TYPE_UINT8,
                std::mem::offset_of!(CByteSeqMsg, data) as u32,
                true,
                0,
                std::ptr::null(),
            ),
        ],
    );
    let bridge = unsafe { BridgedMessage::new(members) }.expect("bridge");

    let mut payload: Vec<u8> = (0..65_536u32)
        .map(|i| (i.wrapping_mul(31) >> 3) as u8)
        .collect();
    let msg = CByteSeqMsg {
        stamp: 99,
        data: CPrimSeqRaw {
            data: payload.as_mut_ptr() as *mut c_void,
            size: payload.len(),
            capacity: payload.len(),
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    let frame = assert_flatten_into_identical(&bridge, &msg as *const _ as *const c_void);
    // The payload bytes themselves survive verbatim at the tail.
    assert_eq!(&frame[frame.len() - payload.len()..], &payload[..]);
}

/// Strings + a DIRECT variable nested complex (std_msgs/Header) — the
/// MoveIt-stamped-message shape, through the Complex op.
#[test]
fn flatten_into_matches_flatten_for_strings_and_nested_header() {
    let bridge = unsafe { BridgedMessage::new(stamped_members()) }.expect("bridge");
    let msg = CStamped {
        header: CHeader {
            stamp: CTime {
                sec: 7,
                nanosec: 99,
            },
            frame_id: heap_string("panda_link0"),
        },
        x: 1.25,
    };
    assert_flatten_into_identical(&bridge, &msg as *const _ as *const c_void);
}

#[test]
fn flatten_into_matches_flatten_for_empty_sequences() {
    let bridge = unsafe { BridgedMessage::new(jointish_members()) }.expect("bridge");
    let msg = CJointish {
        stamp_sec: 3,
        position: CDoubleSeq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
        name: CRosString {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        },
    };
    let frame = assert_flatten_into_identical(&bridge, &msg as *const _ as *const c_void);
    // Empty variable entries still leave a complete, all-initialized
    // head (header + fixed + offset table).
    let header = WireHeader::read_from_buf(&frame).expect("header");
    assert_eq!(header.offset_table_count, 2);
}

/// Bounded sequence filled to its UPPER BOUND — the max-bound edge of
/// the size pre-pass.
#[test]
fn flatten_into_matches_flatten_at_max_bound() {
    #[repr(C)]
    struct CBounded {
        values: CDoubleSeq,
    }
    let mut m = member(
        "values",
        ROS_TYPE_DOUBLE,
        0,
        true,
        4, // bound
        std::ptr::null(),
    );
    m.is_upper_bound_ = true;
    let members = make_members(
        "test_msgs__msg",
        "BoundedSeq",
        std::mem::size_of::<CBounded>(),
        vec![m],
    );
    let bridge = unsafe { BridgedMessage::new(members) }.expect("bridge");
    let msg = CBounded {
        values: double_seq(&[1.0, 2.0, 3.0, 4.0]), // exactly at the bound
    };
    let frame = assert_flatten_into_identical(&bridge, &msg as *const _ as *const c_void);
    assert_eq!(
        frame.len(),
        WireHeader::SIZE + bridge.layout.fixed_size + 8 + 4 * 8
    );
}

/// An undersized buffer must produce Err — no panic, and not a single
/// byte written beyond the slice it was given (poison-pattern tail).
#[test]
fn flatten_into_undersized_buffer_errs_without_oob() {
    let bridge = unsafe { BridgedMessage::new(jointish_members()) }.expect("bridge");
    let msg = CJointish {
        stamp_sec: 42,
        position: double_seq(&[1.0, 2.5, -3.5]),
        name: ros_string("panda_joint1"),
    };
    let size = unsafe { bridge.frame_size(&msg as *const _ as *const c_void) }.expect("size");

    let mut buf = vec![0xCDu8; size + 64];
    let res = unsafe {
        bridge.flatten_into(
            &msg as *const _ as *const c_void,
            0,
            0,
            &mut buf[..size - 1],
        )
    };
    let err = res.expect_err("undersized buffer must error");
    assert!(err.to_string().contains("encode failed"), "got: {err}");
    assert!(
        buf[size - 1..].iter().all(|&b| b == 0xCD),
        "no byte beyond the undersized slice may be touched"
    );

    // Too small even for the pre-zeroed head: must fail up-front.
    let mut tiny = [0xCDu8; 8];
    let res = unsafe { bridge.flatten_into(&msg as *const _ as *const c_void, 0, 0, &mut tiny) };
    assert!(res.is_err(), "buffer smaller than the head must error");
    assert!(
        tiny.iter().all(|&b| b == 0xCD),
        "the Err path must be write-free (capacity check precedes head zeroing)"
    );
}

/// A buffer LARGER than frame_size must also Err (the cursor must end
/// EXACTLY at the pre-pass size — a lying size means a header whose
/// total_size disagrees with the sent slot).
#[test]
fn flatten_into_oversized_buffer_errs() {
    let bridge = unsafe { BridgedMessage::new(jointish_members()) }.expect("bridge");
    let msg = CJointish {
        stamp_sec: 1,
        position: double_seq(&[9.0]),
        name: ros_string("x"),
    };
    let size = unsafe { bridge.frame_size(&msg as *const _ as *const c_void) }.expect("size");
    let mut buf = vec![0u8; size + 8];
    let res = unsafe { bridge.flatten_into(&msg as *const _ as *const c_void, 0, 0, &mut buf) };
    let err = res.expect_err("oversized buffer must error");
    assert!(err.to_string().contains("frame size changed"), "got: {err}");
}

/// Hostile / corrupt sequence counts are rejected in the SIZE PRE-PASS
/// — before any loan or allocation could be sized from them.
#[test]
fn frame_size_rejects_hostile_counts() {
    let bridge = unsafe { BridgedMessage::new(jointish_members()) }.expect("bridge");

    // Corrupt sequence header: count x stride overflows.
    let msg = CJointish {
        stamp_sec: 1,
        position: CDoubleSeq {
            data: std::ptr::NonNull::<f64>::dangling().as_ptr(),
            size: usize::MAX,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
        name: CRosString {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        },
    };
    let err = unsafe { bridge.frame_size(&msg as *const _ as *const c_void) }
        .expect_err("corrupt count must be rejected by the pre-pass");
    assert!(err.to_string().contains("encode failed"), "got: {err}");

    // Corrupt string header: size beyond MAX_FRAME_BYTES.
    let msg = CJointish {
        stamp_sec: 1,
        position: CDoubleSeq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
        name: CRosString {
            data: std::ptr::NonNull::<u8>::dangling().as_ptr(),
            size: usize::MAX,
            capacity: 0,
        },
    };
    let err = unsafe { bridge.frame_size(&msg as *const _ as *const c_void) }
        .expect_err("corrupt string size must be rejected by the pre-pass");
    assert!(err.to_string().contains("encode failed"), "got: {err}");
}

/// The actual TOCTOU window `FrameCursor` / `ERR_FRAME_SIZE_CHANGED`
/// exists for: a sequence whose size SHRINKS between the `frame_size`
/// pre-pass and the `flatten_into` write pass, with a buffer sized to
/// the ORIGINAL pre-pass length. The write pass under-fills the slot, so
/// `require_full` must reject it — never hand back a short, partially-
/// initialized frame. (The undersized/oversized tests reach the guard
/// via statically-wrong buffer lengths; this reaches it via a real
/// mid-pass mutation — the production hazard the design targets.)
#[test]
fn flatten_into_detects_sequence_shrink_between_passes() {
    let bridge = unsafe { BridgedMessage::new(jointish_members()) }.expect("bridge");
    let mut msg = CJointish {
        stamp_sec: 7,
        position: double_seq(&[1.0, 2.0, 3.0]),
        name: ros_string("link"),
    };
    let size = unsafe { bridge.frame_size(&msg as *const _ as *const c_void) }.expect("size");
    // Shrink AFTER sizing (data still backs 3 doubles, so reading 2 is
    // in-bounds). The write pass now produces fewer bytes than the slot.
    msg.position.size = 2;
    let mut buf = vec![0u8; size];
    let res = unsafe { bridge.flatten_into(&msg as *const _ as *const c_void, 0, 0, &mut buf) };
    let err = res.expect_err("a shrunk sequence must be detected, not silently under-filled");
    assert!(err.to_string().contains("frame size changed"), "got: {err}");
}

/// GROW twin: a sequence that grows between the passes must Err at the
/// cursor's capacity guard — WITHOUT writing a single byte past the
/// buffer it was given (the message now wants more bytes than the loan
/// was sized for).
#[test]
fn flatten_into_detects_sequence_grow_between_passes_without_oob() {
    let bridge = unsafe { BridgedMessage::new(jointish_members()) }.expect("bridge");
    // Backing storage for 4 doubles, but sized to 3 for the pre-pass.
    let mut msg = CJointish {
        stamp_sec: 7,
        position: double_seq(&[1.0, 2.0, 3.0, 4.0]),
        name: ros_string("link"),
    };
    msg.position.size = 3;
    let size = unsafe { bridge.frame_size(&msg as *const _ as *const c_void) }.expect("size");
    // Grow back to 4 (all 4 elements are valid memory → no OOB read) and
    // give a buffer sized to the ORIGINAL pre-pass with a poison tail.
    msg.position.size = 4;
    let mut buf = vec![0xCDu8; size + 64];
    let res =
        unsafe { bridge.flatten_into(&msg as *const _ as *const c_void, 0, 0, &mut buf[..size]) };
    assert!(
        res.is_err(),
        "a grown sequence must be rejected, not written past the loan"
    );
    assert!(
        buf[size..].iter().all(|&b| b == 0xCD),
        "the cursor must not touch a byte past the buffer it was given"
    );
}

/// Cross-pass drift on a COMPLEX (nested variable) field: the inner
/// `std_msgs/Header` `frame_id` string shrinks between `frame_size` and
/// `flatten_into`, so the complex field's encoded length changes. Both
/// passes share `encode_complex`, so the only way they disagree is a
/// real mid-pass mutation — and it must be caught (`require_full`), never
/// produce a short frame. (Closes the Complex-op arm of the TOCTOU
/// window the prim-seq tests above cover for the simple arms.)
#[test]
fn flatten_into_detects_complex_field_drift_between_passes() {
    let bridge = unsafe { BridgedMessage::new(stamped_members()) }.expect("bridge");
    let mut msg = CStamped {
        header: CHeader {
            stamp: CTime {
                sec: 7,
                nanosec: 99,
            },
            frame_id: ros_string("panda_link0"), // 11 bytes
        },
        x: 1.25,
    };
    let size = unsafe { bridge.frame_size(&msg as *const _ as *const c_void) }.expect("size");
    // Shrink the nested string AFTER sizing (data still backs 11+NUL, so
    // reading 5 is in-bounds). The complex field now encodes shorter.
    msg.header.frame_id.size = 5;
    let mut buf = vec![0u8; size];
    let res = unsafe { bridge.flatten_into(&msg as *const _ as *const c_void, 0, 0, &mut buf) };
    let err = res.expect_err("a shrunk nested-complex field must be detected");
    assert!(err.to_string().contains("frame size changed"), "got: {err}");
}

// =====================================================================
// Uint8[] decode allocates via malloc (uninitialized) instead
// of calloc -- the copy overwrites every byte, so a zero-fill is
// redundant. These prove the decoded sequence is byte-exact and leaves
// no stale/uninitialized bytes within its allocation (capacity == size
// on the C path, so reading the whole buffer must equal the payload).
//
// NOTE: these are CORRECTNESS pins, not opt guards -- they pass
// identically on a calloc revert (calloc then fully-overwrite is also
// byte-exact). Only the perf bench guards the malloc optimization; the
// test names below reflect the correctness property, not the allocator.
// =====================================================================

#[repr(C)]
struct CU8Seq {
    data: *mut u8,
    size: usize,
    capacity: usize,
    #[cfg(cerulion_has_is_rosidl_buffer)]
    is_rosidl_buffer: bool,
    #[cfg(cerulion_has_is_rosidl_buffer)]
    owns_rosidl_buffer: bool,
}

#[repr(C)]
struct CU8SeqMsg {
    data: CU8Seq,
}

fn u8_seq_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    make_members(
        "test_msgs__msg",
        "U8Seq",
        std::mem::size_of::<CU8SeqMsg>(),
        vec![member("data", ROS_TYPE_UINT8, 0, true, 0, std::ptr::null())],
    )
}

fn u8_seq(values: &[u8]) -> CU8Seq {
    let mut boxed = values.to_vec().into_boxed_slice();
    let data = boxed.as_mut_ptr();
    let len = boxed.len();
    std::mem::forget(boxed); // fixture: process-lifetime
    CU8Seq {
        data,
        size: len,
        capacity: len,
        #[cfg(cerulion_has_is_rosidl_buffer)]
        is_rosidl_buffer: false,
        #[cfg(cerulion_has_is_rosidl_buffer)]
        owns_rosidl_buffer: false,
    }
}

unsafe fn cu8seq_bytes(s: &CU8Seq) -> Vec<u8> {
    if s.data.is_null() || s.size == 0 {
        return Vec::new();
    }
    std::slice::from_raw_parts(s.data, s.size).to_vec()
}

#[test]
fn c_uint8_sequence_decode_is_byte_exact() {
    let bridge = unsafe { BridgedMessage::new(u8_seq_members()) }.expect("bridge");
    // >256 bytes, per-index pattern: a truncated or stale byte mismatches.
    let payload: Vec<u8> = (0..300u32)
        .map(|i| (i.wrapping_mul(37).wrapping_add(1)) as u8)
        .collect();
    let msg = CU8SeqMsg {
        data: u8_seq(&payload),
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut out = CU8SeqMsg {
        data: CU8Seq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok, "uint8[] decode must succeed");
    assert_eq!(out.data.size, payload.len(), "size == count");
    // The C decode allocates exactly `count` bytes (capacity == size), so
    // there is NO tail beyond size -- reading the whole allocation must
    // equal the payload, proving malloc left nothing uninitialized.
    assert_eq!(
        out.data.capacity,
        payload.len(),
        "capacity == count (tight)"
    );
    assert_eq!(unsafe { cu8seq_bytes(&out.data) }, payload, "byte-exact");
}

#[test]
fn c_uint8_decode_fully_replaces_prior_buffer_no_stale_tail() {
    let bridge = unsafe { BridgedMessage::new(u8_seq_members()) }.expect("bridge");
    let long: Vec<u8> = (0..400u32)
        .map(|i| (i.wrapping_mul(31).wrapping_add(7) | 1) as u8)
        .collect();
    let short: Vec<u8> = (0..80u32)
        .map(|i| (i.wrapping_mul(53).wrapping_add(3) | 1) as u8)
        .collect();
    let long_frame = {
        let msg = CU8SeqMsg {
            data: u8_seq(&long),
        };
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten long")
    };
    let short_frame = {
        let msg = CU8SeqMsg {
            data: u8_seq(&short),
        };
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten short")
    };

    let mut out = CU8SeqMsg {
        data: CU8Seq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    // First decode the LONG payload (allocates 400 malloc'd bytes).
    assert!(unsafe {
        bridge.unflatten(
            &long_frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    });
    assert_eq!(out.data.size, long.len());
    assert_eq!(unsafe { cu8seq_bytes(&out.data) }, long);
    // Then decode the SHORT payload into the SAME seq: assign_prim_sequence
    // frees the old buffer and mallocs a fresh one, fully overwritten. No
    // byte of the long payload may survive; capacity tightens to 80.
    assert!(unsafe {
        bridge.unflatten(
            &short_frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    });
    assert_eq!(out.data.size, short.len(), "size shrinks to short count");
    assert_eq!(
        out.data.capacity,
        short.len(),
        "capacity tightens (no stale tail)"
    );
    assert_eq!(
        unsafe { cu8seq_bytes(&out.data) },
        short,
        "content fully replaced"
    );
}

/// Empty edge, C path: an empty uint8[] (count == 0) hits
/// assign_prim_sequence's `bytes.is_empty()` branch, which frees any
/// prior buffer and sets data=null / size=0 / capacity=0 (no malloc).
/// Decode a NON-empty payload first (bridge-malloc'd), THEN the empty one
/// into the SAME seq, so the clear frees the bridge's own libc buffer
/// (allocator-correct) -- proving the empty decode succeeds, fully clears,
/// and does not crash.
#[test]
fn c_uint8_empty_sequence_decodes_to_size_zero() {
    let bridge = unsafe { BridgedMessage::new(u8_seq_members()) }.expect("bridge");
    let nonempty: Vec<u8> = (0..64u32)
        .map(|i| (i.wrapping_mul(7).wrapping_add(1) | 1) as u8)
        .collect();
    let empty: Vec<u8> = Vec::new();

    let nonempty_frame = {
        let msg = CU8SeqMsg {
            data: u8_seq(&nonempty),
        };
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }
            .expect("flatten nonempty")
    };
    let empty_frame = {
        let msg = CU8SeqMsg {
            data: u8_seq(&empty),
        };
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten empty")
    };

    let mut out = CU8SeqMsg {
        data: CU8Seq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    // Prime with the non-empty payload (bridge mallocs the buffer).
    assert!(unsafe {
        bridge.unflatten(
            &nonempty_frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    });
    assert_eq!(out.data.size, nonempty.len());
    // Now decode the EMPTY payload into the SAME seq.
    let ok = unsafe {
        bridge.unflatten(
            &empty_frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok, "empty uint8[] decode must succeed");
    assert_eq!(out.data.size, 0, "decoded empty (size 0)");
    assert_eq!(out.data.capacity, 0, "capacity cleared");
    assert!(
        unsafe { cu8seq_bytes(&out.data) }.is_empty(),
        "no bytes remain"
    );
}

// =====================================================================
// Init_loaned_payload + loan_pad_ranges — the loaned
// message's construction, pinned at the bridge seam where the oracle is
// deterministic. The rmw e2e twin (`rmw_e2e_test.rs::*`) drives
// the same paths over real SHM but cannot force a RECYCLED slot; here a
// garbage-filled buffer stands in for one, so removing the C zero
// pre-pass fails these tests.
// =====================================================================

/// Faithful model of rosidl_generator_c's `__init` for a Vector3 where
/// only `z` declares a default (`float64 z 7`): the C generator emits
/// assignments ONLY for members that need init (rosidl#477) — x/y must
/// come from the bridge's zero pre-pass.
unsafe extern "C" fn v3_c_init(
    msg: *mut std::os::raw::c_void,
    _init: rmw_cerulion::ffi::rosidl_runtime_c__message_initialization,
) {
    (*(msg as *mut CVector3)).z = 7.0;
}

fn vector3_members_with_init() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let members = Box::leak(
        vec![
            member("x", ROS_TYPE_DOUBLE, 0, false, 0, std::ptr::null()),
            member("y", ROS_TYPE_DOUBLE, 8, false, 0, std::ptr::null()),
            member("z", ROS_TYPE_DOUBLE, 16, false, 0, std::ptr::null()),
        ]
        .into_boxed_slice(),
    );
    let mm = rosidl_typesupport_introspection_c__MessageMembers {
        message_namespace_: cstr("geometry_msgs__msg"),
        message_name_: cstr("Vector3"),
        member_count_: members.len() as u32,
        size_of_: std::mem::size_of::<CVector3>(),
        members_: members.as_ptr(),
        init_function: Some(v3_c_init),
        ..Default::default()
    };
    Box::leak(Box::new(mm))
}

/// 8-aligned byte buffer: the init callbacks dereference the payload AS
/// the message struct (align 8), exactly as a real typesupport does, so
/// a bare `[u8; N]` (align 1) would make the test itself UB on a legal
/// layout — the SHM slot the production path hands out is 8-aligned
/// (asserted fail-closed at borrow), and the stand-in must be too.
#[repr(C, align(8))]
struct Aligned8<const N: usize>([u8; N]);

#[test]
fn c_init_loaned_payload_zeroes_then_applies_defaults() {
    let bridge = unsafe { BridgedMessage::new(vector3_members_with_init()) }.expect("bridge");
    assert!(bridge.can_loan);
    // Garbage-filled buffer = the deterministic stand-in for a recycled
    // SHM slot. Without the zero pre-pass the default-less members would
    // ship whatever the slot last held.
    let mut buf = Aligned8([0xAAu8; 24]);
    unsafe { bridge.init_loaned_payload(buf.0.as_mut_ptr() as *mut std::os::raw::c_void) };
    let v = unsafe { std::ptr::read(buf.0.as_ptr() as *const CVector3) };
    assert_eq!(
        v.x.to_bits(),
        0.0f64.to_bits(),
        "default-less member comes from the zero pre-pass, not recycled slot bytes"
    );
    assert_eq!(v.y.to_bits(), 0.0f64.to_bits());
    assert_eq!(
        v.z.to_bits(),
        7.0f64.to_bits(),
        "declared default applied by init_function"
    );
}

#[test]
fn c_init_loaned_payload_without_init_function_is_the_zeroed_baseline() {
    let bridge = unsafe { BridgedMessage::new(vector3_members()) }.expect("bridge");
    assert!(bridge.can_loan);
    let mut buf = Aligned8([0x55u8; 24]);
    unsafe { bridge.init_loaned_payload(buf.0.as_mut_ptr() as *mut std::os::raw::c_void) };
    assert!(
        buf.0.iter().all(|&b| b == 0),
        "None-arm fallback is the documented zeroed baseline"
    );
}

/// bool-then-f64: 7 repr(C) padding bytes at [1, 8) plus no tail pad.
#[repr(C)]
struct CPadFlagVal {
    flag: u8,
    value: f64,
}

#[test]
fn loan_pad_ranges_cover_exactly_the_padding() {
    const ROS_TYPE_BOOLEAN: u8 = 6;
    let padded = make_members(
        "rmw_bridge__msg",
        "PadFlagVal",
        std::mem::size_of::<CPadFlagVal>(),
        vec![
            member("flag", ROS_TYPE_BOOLEAN, 0, false, 0, std::ptr::null()),
            member("value", ROS_TYPE_DOUBLE, 8, false, 0, std::ptr::null()),
        ],
    );
    let bridge = unsafe { BridgedMessage::new(padded) }.expect("bridge");
    assert!(bridge.can_loan);
    assert_eq!(
        bridge.loan_pad_ranges(),
        &[(1usize, 7usize)],
        "exactly the inter-field gap — never a field byte, never less than the gap"
    );

    let no_pad = unsafe { BridgedMessage::new(vector3_members()) }.expect("bridge");
    assert!(
        no_pad.loan_pad_ranges().is_empty(),
        "a gapless layout has nothing to zero"
    );
}

/// Lyrical and Rolling: a top-level `uint8[]` INSTANCE whose header says
/// Buffer-backed holds a Buffer object behind `data`, not elements. The
/// size pass, the write pass and the fill all refuse it (a bogus non-null
/// pointer would be read or freed if any of them did not), and the same
/// message with the flag clear goes through.
#[cfg(cerulion_has_is_rosidl_buffer)]
#[test]
fn c_buffer_backed_top_level_instance_is_refused_by_size_flatten_and_fill() {
    let bridge = unsafe { BridgedMessage::new(u8_seq_members()) }.expect("bridge");
    let mut backing = [1u8, 2, 3];
    let mut msg = CU8SeqMsg {
        data: CU8Seq {
            data: backing.as_mut_ptr(),
            size: backing.len(),
            capacity: backing.len(),
            is_rosidl_buffer: true,
            owns_rosidl_buffer: false,
        },
    };
    let size = unsafe { bridge.frame_size(&msg as *const _ as *const c_void) };
    assert!(
        size.is_err(),
        "the size pass must refuse a Buffer-backed instance"
    );
    let frame = unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) };
    let err = frame.expect_err("the write pass must refuse a Buffer-backed instance");
    assert!(err.to_string().contains("rosidl Buffer"), "got: {err}");
    // The fill refuses the same instance and leaves its pointer untouched.
    let plain = CU8SeqMsg {
        data: u8_seq(&[9, 8, 7]),
    };
    let good = unsafe { bridge.flatten(&plain as *const _ as *const c_void, 0, 0) }.expect("plain");
    let ok =
        unsafe { bridge.unflatten(&good[WireHeader::SIZE..], &mut msg as *mut _ as *mut c_void) };
    assert!(!ok, "the fill must refuse a Buffer-backed instance");
    assert_eq!(msg.data.data as usize, backing.as_mut_ptr() as usize);
    assert_eq!(msg.data.size, 3);
    // Control: the flag is the ONLY difference.
    msg.data.is_rosidl_buffer = false;
    let size = unsafe { bridge.frame_size(&msg as *const _ as *const c_void) }.expect("plain size");
    assert!(size > 0);
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("plain flatten");
    assert!(frame.len() > WireHeader::SIZE);
}
