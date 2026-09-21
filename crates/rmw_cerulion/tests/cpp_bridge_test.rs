// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! introspection_cpp bridge tests — NO ROS install
//! required.
//!
//! Strategy: the cpp bridge touches C++ containers ONLY through the
//! member function pointers and the compiled `std::string` shim. Tests
//! exploit that:
//! - **Sequences** are faked with a Rust `#[repr(C)]` header + Rust
//!   `extern "C"` accessor fns — the bridge cannot tell the difference
//!   from a real `std::vector`, because the function-pointer contract IS
//!   the interface.
//! - **Strings** are REAL `std::string`s, placement-constructed into the
//!   fixture struct through the shim — exercising the platform's actual
//!   string ABI (libc++ on macOS, libstdc++ on Linux).
//!
//! The decisive assertions are interop ones: cpp-bridged frames must
//! decode through the independent native generated readers and carry the
//! native SCHEMA_HASH, byte-for-byte. (The C bridge is proven against the
//! same native readers in `bridge_test.rs`; cpp↔native + C↔native
//! transitively give cpp↔C compatibility — no test here builds a C
//! bridge.)
//!
//! **Caveat on that transitivity:** it only covers shapes the native
//! readers actually exercise here, which are a fixed `Vector3` and a bare
//! `String` — neither reaches a variable-nested ELEMENT body. Through that gap
//! a cpp bridge could carry a packed, non-canonical element encoding
//! unseen. `cpp_variable_nested_element_body_is_canonical`
//! closes it with a byte oracle plus an independent `FrameWalker` decode.

use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};

use rmw_cerulion::ffi::introspection_cpp::{
    rmw_cerulion_cppstring_construct, rmw_cerulion_cppstring_destruct,
    rmw_cerulion_cppstring_sizeof, CppMessageMember, CppMessageMembers, VecTriplet,
};
// The owning-leg fixture builds a REAL `std::vector<uint8_t>` so
// the storage handed to the production release was allocated by the
// allocator that frees it. The vector externs — and the `pub(crate)`
// assign entry's own `extern "C"` block — are already declared further
// down this file for the fixtures and are module-scope, so this
// test reuses those rather than declaring a second set.
use rmw_cerulion::ffi::rosidl_message_type_support_t;
use rmw_cerulion::type_bridge_cpp::CppBridgedMessage;

use cerulion_core::wire::WireHeader;

const ROS_TYPE_DOUBLE: u8 = 2;
const ROS_TYPE_BOOLEAN: u8 = 6;
const ROS_TYPE_STRING: u8 = 16;
const ROS_TYPE_MESSAGE: u8 = 18;

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn member(
    name: &str,
    type_id: u8,
    offset: u32,
    is_array: bool,
    nested: *const rosidl_message_type_support_t,
) -> CppMessageMember {
    CppMessageMember {
        name_: cstr(name),
        type_id_: type_id,
        string_upper_bound_: 0,
        members_: nested,
        is_key_: false,
        is_array_: is_array,
        array_size_: 0,
        is_upper_bound_: false,
        offset_: offset,
        default_value_: std::ptr::null(),
        size_function: None,
        get_const_function: None,
        get_function: None,
        fetch_function: None,
        assign_function: None,
        resize_function: None,
    }
}

fn make_members(
    namespace: &str,
    name: &str,
    size_of: usize,
    members: Vec<CppMessageMember>,
) -> *const CppMessageMembers {
    let members = Box::leak(members.into_boxed_slice());
    Box::leak(Box::new(CppMessageMembers {
        message_namespace_: cstr(namespace),
        message_name_: cstr(name),
        member_count_: members.len() as u32,
        size_of_: size_of,
        has_any_key_member_: false,
        members_: members.as_ptr(),
        init_function: None,
        fini_function: None,
    }))
}

// =====================================================================
// Fake std::vector<f64>: repr(C) header + accessor fns. The bridge only
// ever touches it through the function pointers.
// =====================================================================

#[repr(C)]
struct FakeVecF64 {
    data: *mut f64,
    len: usize,
    cap: usize,
}

unsafe extern "C" fn vecf64_size(field: *const c_void) -> usize {
    (*(field as *const FakeVecF64)).len
}
unsafe extern "C" fn vecf64_get_const(field: *const c_void, idx: usize) -> *const c_void {
    let v = &*(field as *const FakeVecF64);
    v.data.add(idx) as *const c_void
}
unsafe extern "C" fn vecf64_get(field: *mut c_void, idx: usize) -> *mut c_void {
    let v = &mut *(field as *mut FakeVecF64);
    v.data.add(idx) as *mut c_void
}
unsafe extern "C" fn vecf64_resize(field: *mut c_void, size: usize) {
    let v = &mut *(field as *mut FakeVecF64);
    // Test fixture: leak old storage (process-lifetime fixtures).
    let mut storage = vec![0f64; size.max(1)];
    v.data = storage.as_mut_ptr();
    v.len = size;
    v.cap = size;
    std::mem::forget(storage);
}

/// Fake std::vector<bool>: BIT-PACKED like the real thing, so a
/// contiguous-memcpy regression cannot pass accidentally.
#[repr(C)]
struct FakeVecBool {
    bits: *mut u8,
    len: usize,
}

unsafe extern "C" fn vecbool_size(field: *const c_void) -> usize {
    (*(field as *const FakeVecBool)).len
}
unsafe extern "C" fn vecbool_fetch(field: *const c_void, idx: usize, out: *mut c_void) {
    let v = &*(field as *const FakeVecBool);
    let bit = (*v.bits.add(idx / 8) >> (idx % 8)) & 1;
    *(out as *mut bool) = bit != 0;
}
unsafe extern "C" fn vecbool_assign(field: *mut c_void, idx: usize, val: *const c_void) {
    let v = &mut *(field as *mut FakeVecBool);
    let b = *(val as *const bool);
    let byte = v.bits.add(idx / 8);
    if b {
        *byte |= 1 << (idx % 8);
    } else {
        *byte &= !(1 << (idx % 8));
    }
}
unsafe extern "C" fn vecbool_resize(field: *mut c_void, size: usize) {
    let v = &mut *(field as *mut FakeVecBool);
    let mut storage = vec![0u8; size.div_ceil(8).max(1)];
    v.bits = storage.as_mut_ptr();
    v.len = size;
    std::mem::forget(storage);
}

// =====================================================================
// std::string helpers (REAL C++ strings via the shim)
// =====================================================================

/// Aligned storage for one std::string (32 bytes on both libstdc++ and
/// libc++; over-aligned to 16 to be safe).
#[repr(C, align(16))]
struct StringSlot([u8; 32]);

impl StringSlot {
    fn construct(&mut self, s: &str) {
        assert!(unsafe { rmw_cerulion_cppstring_sizeof() } <= 32);
        unsafe {
            rmw_cerulion_cppstring_construct(
                self.0.as_mut_ptr() as *mut c_void,
                s.as_ptr() as *const c_char,
                s.len(),
            );
        }
    }
    fn read(&self) -> String {
        let mut data: *const c_char = std::ptr::null();
        let mut len: usize = 0;
        unsafe {
            rmw_cerulion::ffi::introspection_cpp::rmw_cerulion_cppstring_view(
                self.0.as_ptr() as *const c_void,
                &mut data,
                &mut len,
            );
            String::from_utf8_lossy(std::slice::from_raw_parts(data as *const u8, len)).into_owned()
        }
    }
    fn destruct(&mut self) {
        unsafe { rmw_cerulion_cppstring_destruct(self.0.as_mut_ptr() as *mut c_void) }
    }
}

// =====================================================================
// Fixtures
// =====================================================================

/// All-primitive fixed message — the zero-copy class.
#[repr(C)]
#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct CppVector3 {
    x: f64,
    y: f64,
    z: f64,
}

fn vector3_members_cpp() -> *const CppMessageMembers {
    make_members(
        "geometry_msgs::msg",
        "Vector3",
        std::mem::size_of::<CppVector3>(),
        vec![
            member("x", ROS_TYPE_DOUBLE, 0, false, std::ptr::null()),
            member("y", ROS_TYPE_DOUBLE, 8, false, std::ptr::null()),
            member("z", ROS_TYPE_DOUBLE, 16, false, std::ptr::null()),
        ],
    )
}

/// std_msgs/String-shaped C++ message: { data: std::string }.
#[repr(C, align(16))]
struct CppStringMsg {
    data: StringSlot,
}

fn string_msg_members_cpp() -> *const CppMessageMembers {
    make_members(
        "std_msgs::msg",
        "String",
        std::mem::size_of::<CppStringMsg>(),
        vec![member("data", ROS_TYPE_STRING, 0, false, std::ptr::null())],
    )
}

/// JointState-flavored: { stamp_sec: f64, position: vector<f64>,
/// name: std::string } — fixed + prim-seq + string in one message.
#[repr(C, align(16))]
struct CppJointish {
    stamp_sec: f64,
    position: FakeVecF64,
    name: StringSlot,
}

/// A jointish typesupport with init/fini, so
/// forging is enabled. `make_members` leaves both `None`, and bridge
/// construction disables forging when either is absent — with that fixture the
/// reuse arm below would return before its assertions. The bodies are
/// minimal because the pre-pass never calls them: what it needs is the
/// typesupport to be forgeable at all.
unsafe extern "C" fn jointish_init(msg: *mut c_void, _kind: u32) {
    std::ptr::write_bytes(msg as *mut u8, 0, std::mem::size_of::<CppJointish>());
}
unsafe extern "C" fn jointish_fini(_msg: *mut c_void) {}

fn jointish_forgeable_members_cpp() -> *const CppMessageMembers {
    let members = jointish_members_cpp() as *mut CppMessageMembers;
    // SAFETY: `make_members` leaks its box, so this points at a
    // process-lifetime fixture nothing else has yet observed.
    unsafe {
        (*members).init_function = Some(jointish_init);
        (*members).fini_function = Some(jointish_fini);
    }
    members
}

fn jointish_members_cpp() -> *const CppMessageMembers {
    let mut pos = member(
        "position",
        ROS_TYPE_DOUBLE,
        std::mem::offset_of!(CppJointish, position) as u32,
        true,
        std::ptr::null(),
    );
    pos.size_function = Some(vecf64_size);
    pos.get_const_function = Some(vecf64_get_const);
    pos.get_function = Some(vecf64_get);
    pos.resize_function = Some(vecf64_resize);
    make_members(
        "test_msgs::msg",
        "Jointish",
        std::mem::size_of::<CppJointish>(),
        vec![
            member("stamp_sec", ROS_TYPE_DOUBLE, 0, false, std::ptr::null()),
            pos,
            member(
                "name",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CppJointish, name) as u32,
                false,
                std::ptr::null(),
            ),
        ],
    )
}

// =====================================================================
// Tests
// =====================================================================

#[test]
fn cpp_fixed_message_is_zero_copy_loanable() {
    use cerulion_core::message::ShmMessage;
    let bridge = unsafe { CppBridgedMessage::new(vector3_members_cpp()) }.expect("bridge");
    assert_eq!(bridge.qualified_name, "geometry_msgs/Vector3");
    // recipe-3: the C++ bridge hash must equal the NATIVE generated
    // SCHEMA_HASH (byte-compatible interop), not a name-only literal.
    assert_eq!(
        bridge.schema_hash(),
        <native_ros2_messages::geometry_msgs::Vector3 as ShmMessage>::SCHEMA_HASH
    );
    assert!(bridge.can_loan, "fixed C++ messages are loanable too");

    let msg = CppVector3 {
        x: 1.0,
        y: -2.0,
        z: 0.5,
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 3, 30) }.expect("flatten");
    assert_eq!(frame.len(), WireHeader::SIZE + 24);

    let mut out = CppVector3::default();
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out, msg);
}

/// THE interop contract: a cpp-bridged frame must decode through an
/// INDEPENDENT oracle — the native generated reader (`StringShm`) — and
/// carry the native `SCHEMA_HASH`. That is what lets a C++ talker reach a
/// Python or native Cerulion listener. (The C bridge is proven against
/// the same native readers in `bridge_test.rs`; cpp↔native + C↔native
/// transitively give cpp↔C byte-compatibility — this test does NOT build
/// a C bridge, hence the native-reader name rather than a cross-bridge
/// one.)
#[test]
fn cpp_string_frame_decodes_via_native_reader_and_matches_schema_hash() {
    use cerulion_core::message::ShmMessage;
    use native_ros2_messages::std_msgs::StringShm;

    let bridge = unsafe { CppBridgedMessage::new(string_msg_members_cpp()) }.expect("bridge");
    // recipe-3: bridged frame carries the NATIVE generated SCHEMA_HASH.
    assert_eq!(
        bridge.schema_hash(),
        <native_ros2_messages::std_msgs::String as ShmMessage>::SCHEMA_HASH
    );

    let mut msg = CppStringMsg {
        data: StringSlot([0; 32]),
    };
    msg.data.construct("Hello World: 42");
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 7, 70) }.expect("flatten");

    // Native generated reader decodes the cpp-bridged frame.
    let native = StringShm::from_bytes(&frame[WireHeader::SIZE..]);
    assert_eq!(native.data().expect("data"), "Hello World: 42");

    // And the roundtrip back into a REAL std::string.
    let mut out = CppStringMsg {
        data: StringSlot([0; 32]),
    };
    out.data.construct(""); // initialized message, per rmw_take contract
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out.data.read(), "Hello World: 42");

    msg.data.destruct();
    out.data.destruct();
}

#[test]
fn cpp_mixed_message_roundtrips_through_function_pointers() {
    let bridge = unsafe { CppBridgedMessage::new(jointish_members_cpp()) }.expect("bridge");

    let mut positions = [1.5f64, -2.25, 4.125];
    let mut msg = CppJointish {
        stamp_sec: 9.0,
        position: FakeVecF64 {
            data: positions.as_mut_ptr(),
            len: positions.len(),
            cap: positions.len(),
        },
        name: StringSlot([0; 32]),
    };
    msg.name.construct("panda_joint1");

    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut out = CppJointish {
        stamp_sec: 0.0,
        position: FakeVecF64 {
            data: std::ptr::null_mut(),
            len: 0,
            cap: 0,
        },
        name: StringSlot([0; 32]),
    };
    out.name.construct("");
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out.stamp_sec, 9.0);
    assert_eq!(out.position.len, 3);
    let decoded = unsafe { std::slice::from_raw_parts(out.position.data, 3) };
    assert_eq!(decoded, &positions[..]);
    assert_eq!(out.name.read(), "panda_joint1");

    msg.name.destruct();
    out.name.destruct();
}

/// vector<bool> is bit-packed — the fixture is too, so a contiguous
/// memcpy implementation CANNOT pass this test.
#[test]
fn cpp_bool_sequence_goes_through_fetch_assign() {
    #[repr(C)]
    struct CppFlags {
        flags: FakeVecBool,
    }
    let mut m = member("flags", ROS_TYPE_BOOLEAN, 0, true, std::ptr::null());
    m.size_function = Some(vecbool_size);
    m.fetch_function = Some(vecbool_fetch);
    m.assign_function = Some(vecbool_assign);
    m.resize_function = Some(vecbool_resize);
    let members = make_members(
        "test_msgs::msg",
        "Flags",
        std::mem::size_of::<CppFlags>(),
        vec![m],
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");

    let pattern = [true, false, true, true, false, false, true, false, true];
    let mut bits = vec![0u8; 2];
    for (i, &b) in pattern.iter().enumerate() {
        if b {
            bits[i / 8] |= 1 << (i % 8);
        }
    }
    let msg = CppFlags {
        flags: FakeVecBool {
            bits: bits.as_mut_ptr(),
            len: pattern.len(),
        },
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    // Wire form: one byte per bool, 0/1 (the canonical Cerulion form).
    let payload = &frame[WireHeader::SIZE..];
    let table_base = bridge.layout.fixed_size;
    let off = u32::from_le_bytes(payload[table_base..table_base + 4].try_into().unwrap()) as usize;
    let len =
        u32::from_le_bytes(payload[table_base + 4..table_base + 8].try_into().unwrap()) as usize;
    assert_eq!(len, pattern.len());
    let wire_bools: Vec<bool> = payload[off..off + len].iter().map(|&b| b != 0).collect();
    assert_eq!(wire_bools, pattern);

    // Decode back through assign_function into a fresh bit-packed vec.
    let mut out = CppFlags {
        flags: FakeVecBool {
            bits: std::ptr::null_mut(),
            len: 0,
        },
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out.flags.len, pattern.len());
    let mut decoded = Vec::new();
    for i in 0..out.flags.len {
        let mut v = false;
        unsafe {
            vecbool_fetch(
                &out.flags as *const _ as *const c_void,
                i,
                &mut v as *mut bool as *mut c_void,
            )
        };
        decoded.push(v);
    }
    assert_eq!(decoded, pattern);
}

/// Direct variable nested member (Header-class) through the CPP bridge —
/// the MoveIt workload shape, C++ edition.
#[test]
fn cpp_header_bearing_message_registers_and_roundtrips() {
    #[repr(C)]
    #[derive(Default, Clone, Copy)]
    struct CppTime {
        sec: i32,
        nanosec: u32,
    }
    #[repr(C, align(16))]
    struct CppHeader {
        stamp: CppTime,
        frame_id: StringSlot,
    }
    #[repr(C, align(16))]
    struct CppStamped {
        header: CppHeader,
        x: f64,
    }

    const ROS_TYPE_INT32: u8 = 13;
    const ROS_TYPE_UINT32: u8 = 12;
    let time_members = make_members(
        "builtin_interfaces::msg",
        "Time",
        std::mem::size_of::<CppTime>(),
        vec![
            member("sec", ROS_TYPE_INT32, 0, false, std::ptr::null()),
            member("nanosec", ROS_TYPE_UINT32, 4, false, std::ptr::null()),
        ],
    );
    let time_ts = Box::leak(Box::new(rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
        data: time_members as *const c_void,
        ..Default::default()
    }));
    let header_members = make_members(
        "std_msgs::msg",
        "Header",
        std::mem::size_of::<CppHeader>(),
        vec![
            member("stamp", ROS_TYPE_MESSAGE, 0, false, time_ts),
            member(
                "frame_id",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CppHeader, frame_id) as u32,
                false,
                std::ptr::null(),
            ),
        ],
    );
    let header_ts = Box::leak(Box::new(rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
        data: header_members as *const c_void,
        ..Default::default()
    }));
    let stamped_members = make_members(
        "test_msgs::msg",
        "CppStamped",
        std::mem::size_of::<CppStamped>(),
        vec![
            member("header", ROS_TYPE_MESSAGE, 0, false, header_ts),
            member(
                "x",
                ROS_TYPE_DOUBLE,
                std::mem::offset_of!(CppStamped, x) as u32,
                false,
                std::ptr::null(),
            ),
        ],
    );

    let bridge = unsafe { CppBridgedMessage::new(stamped_members) }
        .expect("Header-bearing C++ message MUST register");

    let mut msg = CppStamped {
        header: CppHeader {
            stamp: CppTime { sec: 5, nanosec: 6 },
            frame_id: StringSlot([0; 32]),
        },
        x: 2.5,
    };
    msg.header.frame_id.construct("base_link");
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut out = CppStamped {
        header: CppHeader {
            stamp: CppTime::default(),
            frame_id: StringSlot([0; 32]),
        },
        x: 0.0,
    };
    out.header.frame_id.construct("");
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out.header.stamp.sec, 5);
    assert_eq!(out.header.stamp.nanosec, 6);
    assert_eq!(out.header.frame_id.read(), "base_link");
    assert_eq!(out.x, 2.5);

    msg.header.frame_id.destruct();
    out.header.frame_id.destruct();
}

// =====================================================================
// Complex codec via function pointers, hostile-count
// resize spy, padding poison, corrupt string cap, bounded-seq guard.
// =====================================================================

use std::sync::atomic::{AtomicUsize, Ordering};

/// Generic fake sequence of fixed-size elements (repr(C) header + Rust
/// accessor fns) — stands in for std::vector<T>.
#[repr(C)]
struct FakeSeq {
    data: *mut u8,
    len: usize,
    elem: usize,
}

unsafe extern "C" fn seq_size(field: *const c_void) -> usize {
    (*(field as *const FakeSeq)).len
}
unsafe extern "C" fn seq_get_const(field: *const c_void, idx: usize) -> *const c_void {
    let v = &*(field as *const FakeSeq);
    v.data.add(idx * v.elem) as *const c_void
}
unsafe extern "C" fn seq_get(field: *mut c_void, idx: usize) -> *mut c_void {
    let v = &mut *(field as *mut FakeSeq);
    v.data.add(idx * v.elem) as *mut c_void
}

static RESIZE_CALLS: AtomicUsize = AtomicUsize::new(0);
static RESIZE_LAST: AtomicUsize = AtomicUsize::new(0);

unsafe extern "C" fn seq_resize_string_elems(field: *mut c_void, size: usize) {
    RESIZE_CALLS.fetch_add(1, Ordering::SeqCst);
    RESIZE_LAST.store(size, Ordering::SeqCst);
    let v = &mut *(field as *mut FakeSeq);
    // PROPERLY ALIGNED storage (StringSlot is align(16); a Vec<u8>
    // would only be align-1 by type).
    let mut storage: Vec<StringSlot> = Vec::with_capacity(size.max(1));
    for _ in 0..size {
        storage.push(StringSlot([0; 32]));
    }
    // construct empty std::strings in place for each CppStringMsg slot
    for slot in storage.iter_mut() {
        rmw_cerulion_cppstring_construct(slot.0.as_mut_ptr() as *mut c_void, std::ptr::null(), 0);
    }
    v.data = storage.as_mut_ptr() as *mut u8;
    v.len = size;
    std::mem::forget(storage); // fixture: process-lifetime
}

/// { items: CppStringMsg[] } — sequence of VARIABLE nested messages
/// containing real std::strings. THE MoveIt payload shape
/// (JointTrajectory.points-class), in the language MoveIt speaks.
#[repr(C)]
struct CppItemSeqMsg {
    items: FakeSeq,
}

fn cpp_item_seq_members() -> *const CppMessageMembers {
    let s_ts = Box::leak(Box::new(rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
        data: string_msg_members_cpp() as *const c_void,
        ..Default::default()
    }));
    let mut m = member("items", ROS_TYPE_MESSAGE, 0, true, s_ts);
    m.size_function = Some(seq_size);
    m.get_const_function = Some(seq_get_const);
    m.get_function = Some(seq_get);
    m.resize_function = Some(seq_resize_string_elems);
    make_members(
        "test_msgs::msg",
        "CppItemSeq",
        std::mem::size_of::<CppItemSeqMsg>(),
        vec![m],
    )
}

#[test]
#[serial]
fn cpp_complex_codec_variable_nested_seq_roundtrips_deterministically() {
    let bridge = unsafe { CppBridgedMessage::new(cpp_item_seq_members()) }.expect("bridge");

    // Two elements with real std::strings.
    assert!(
        unsafe { rmw_cerulion_cppstring_sizeof() } <= 32,
        "fixture slot too small"
    );
    let mut items: Vec<CppStringMsg> = (0..2)
        .map(|_| CppStringMsg {
            data: StringSlot([0; 32]),
        })
        .collect();
    items[0].data.construct("first");
    items[1].data.construct("second, longer payload");
    let msg = CppItemSeqMsg {
        items: FakeSeq {
            data: items.as_mut_ptr() as *mut u8,
            len: 2,
            elem: std::mem::size_of::<CppStringMsg>(),
        },
    };
    let fa = unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("a");
    let fb = unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("b");
    assert_eq!(fa, fb, "complex codec must be deterministic");

    // Decode back through resize_function + cppstring_assign.
    RESIZE_CALLS.store(0, Ordering::SeqCst);
    let mut out = CppItemSeqMsg {
        items: FakeSeq {
            data: std::ptr::null_mut(),
            len: 0,
            elem: std::mem::size_of::<CppStringMsg>(),
        },
    };
    let ok =
        unsafe { bridge.unflatten(&fa[WireHeader::SIZE..], &mut out as *mut _ as *mut c_void) };
    assert!(ok);
    assert_eq!(RESIZE_CALLS.load(Ordering::SeqCst), 1);
    assert_eq!(out.items.len, 2);
    let decoded = unsafe { std::slice::from_raw_parts(out.items.data as *const CppStringMsg, 2) };
    assert_eq!(decoded[0].data.read(), "first");
    assert_eq!(decoded[1].data.read(), "second, longer payload");

    items[0].data.destruct();
    items[1].data.destruct();
}

/// Hostile count through the cpp decode path: the count guard must
/// reject BEFORE resize_function executes inside a real C++ container
/// (a u32::MAX resize would OOM-abort the host).
#[test]
#[serial]
fn cpp_hostile_count_never_reaches_resize_function() {
    let bridge = unsafe { CppBridgedMessage::new(cpp_item_seq_members()) }.expect("bridge");

    // Well-formed empty message → valid frame, then poison the count.
    let msg = CppItemSeqMsg {
        items: FakeSeq {
            data: std::ptr::null_mut(),
            len: 0,
            elem: std::mem::size_of::<CppStringMsg>(),
        },
    };
    let mut frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    let payload = &frame[WireHeader::SIZE..];
    let table_base = bridge.layout.fixed_size;
    let off = u32::from_le_bytes(payload[table_base..table_base + 4].try_into().unwrap()) as usize;
    let abs = WireHeader::SIZE + off;
    frame[abs..abs + 4].copy_from_slice(&u32::MAX.to_le_bytes());

    RESIZE_CALLS.store(0, Ordering::SeqCst);
    let mut out = CppItemSeqMsg {
        items: FakeSeq {
            data: std::ptr::null_mut(),
            len: 0,
            elem: std::mem::size_of::<CppStringMsg>(),
        },
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(!ok, "hostile count must be rejected");
    assert_eq!(
        RESIZE_CALLS.load(Ordering::SeqCst),
        0,
        "resize_function must NEVER run for a hostile count"
    );
}

/// Bounded-sequence guard: a frame claiming more
/// elements than the bound must be rejected BEFORE resize_function —
/// rosidl's BoundedVector::resize THROWS past the bound, and a C++
/// exception through the extern "C" pointer into Rust is UB.
#[test]
#[serial]
fn cpp_bounded_sequence_over_bound_rejected_before_resize() {
    let s_ts = Box::leak(Box::new(rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
        data: string_msg_members_cpp() as *const c_void,
        ..Default::default()
    }));
    let mut m = member("items", ROS_TYPE_MESSAGE, 0, true, s_ts);
    m.array_size_ = 2;
    m.is_upper_bound_ = true; // bounded: <=2
    m.size_function = Some(seq_size);
    m.get_const_function = Some(seq_get_const);
    m.get_function = Some(seq_get);
    m.resize_function = Some(seq_resize_string_elems);
    let members = make_members(
        "test_msgs::msg",
        "CppBoundedSeq",
        std::mem::size_of::<CppItemSeqMsg>(),
        vec![m],
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");

    let msg = CppItemSeqMsg {
        items: FakeSeq {
            data: std::ptr::null_mut(),
            len: 0,
            elem: std::mem::size_of::<CppStringMsg>(),
        },
    };
    let mut frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    let payload = &frame[WireHeader::SIZE..];
    let table_base = bridge.layout.fixed_size;
    let off = u32::from_le_bytes(payload[table_base..table_base + 4].try_into().unwrap()) as usize;
    let abs = WireHeader::SIZE + off;
    // Claim 3 elements (> bound 2) with enough trailing bytes that the
    // per-element length prefixes parse — AND patch the offset-table
    // LEN so read_var_entry hands decode the full 16 bytes (without
    // this the count-plausibility check rejects first
    // and the bound guard is never reached — with the guard disabled
    // such a test still passes).
    frame[abs..abs + 4].copy_from_slice(&3u32.to_le_bytes());
    frame.extend_from_slice(&[0u8; 12]);
    let len_at = WireHeader::SIZE + table_base + 4;
    frame[len_at..len_at + 4].copy_from_slice(&16u32.to_le_bytes());

    RESIZE_CALLS.store(0, Ordering::SeqCst);
    let mut out = CppItemSeqMsg {
        items: FakeSeq {
            data: std::ptr::null_mut(),
            len: 0,
            elem: std::mem::size_of::<CppStringMsg>(),
        },
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(!ok, "over-bound count must be rejected");
    assert_eq!(
        RESIZE_CALLS.load(Ordering::SeqCst),
        0,
        "resize must not run"
    );
}

/// Padding determinism through the cpp FixedCopy path: a nested struct with repr(C)
/// padding, poisoned differently in two source buffers, must flatten
/// to byte-identical frames with zeroed padding.
#[test]
fn cpp_struct_padding_is_zeroed_for_deterministic_frames() {
    #[repr(C)]
    #[derive(Clone, Copy)]
    struct CppPadded {
        flag: u8,
        value: u64,
    }
    const ROS_TYPE_UINT8: u8 = 8;
    const ROS_TYPE_UINT64: u8 = 14;
    assert_eq!(std::mem::size_of::<CppPadded>(), 16);

    let padded = make_members(
        "test_msgs::msg",
        "CppPadded",
        std::mem::size_of::<CppPadded>(),
        vec![
            member("flag", ROS_TYPE_UINT8, 0, false, std::ptr::null()),
            member("value", ROS_TYPE_UINT64, 8, false, std::ptr::null()),
        ],
    );
    let p_ts = Box::leak(Box::new(rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
        data: padded as *const c_void,
        ..Default::default()
    }));
    let outer = make_members(
        "test_msgs::msg",
        "CppPadOuter",
        std::mem::size_of::<CppPadded>(),
        vec![member("inner", ROS_TYPE_MESSAGE, 0, false, p_ts)],
    );
    let bridge = unsafe { CppBridgedMessage::new(outer) }.expect("bridge");

    let make_poisoned = |poison: u8| -> [u8; 16] {
        let mut buf = [poison; 16];
        buf[0] = 1;
        buf[8..16].copy_from_slice(&0xCAFE_F00D_u64.to_le_bytes());
        buf
    };
    let a = make_poisoned(0xAA);
    let b = make_poisoned(0x55);
    let fa = unsafe { bridge.flatten(a.as_ptr() as *const c_void, 1, 2) }.expect("a");
    let fb = unsafe { bridge.flatten(b.as_ptr() as *const c_void, 1, 2) }.expect("b");
    assert_eq!(fa, fb, "padding garbage leaked through the cpp bridge");
    assert_eq!(&fa[WireHeader::SIZE + 1..WireHeader::SIZE + 8], &[0u8; 7]);
}

/// Corrupt std::string size cap: a real string read with a tiny cap
/// must error (exercises the cap branch with a REAL string — no ABI
/// forgery), and a corrupt seq size_function must yield Encode Err.
#[test]
fn cpp_corrupt_sizes_error_instead_of_ub() {
    // (a) string cap
    let mut slot = StringSlot([0; 32]);
    slot.construct("0123456789");
    let res = unsafe {
        rmw_cerulion::ffi::introspection_cpp::cppstring_bytes(slot.0.as_ptr() as *const c_void, 4)
    };
    assert!(res.is_err(), "oversize string must be capped");
    slot.destruct();

    // (b) corrupt size_function → flatten Err
    unsafe extern "C" fn evil_size(_f: *const c_void) -> usize {
        usize::MAX
    }
    let mut m = member("vals", ROS_TYPE_DOUBLE, 0, true, std::ptr::null());
    m.size_function = Some(evil_size);
    m.get_const_function = Some(seq_get_const);
    m.get_function = Some(seq_get);
    m.resize_function = Some(seq_resize_string_elems);
    let members = make_members(
        "test_msgs::msg",
        "CppEvilSeq",
        std::mem::size_of::<FakeSeq>(),
        vec![m],
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");
    let msg = FakeSeq {
        data: std::ptr::null_mut(),
        len: 0,
        elem: 8,
    };
    let res = unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) };
    let err = res.expect_err("corrupt size_function must error");
    assert!(err.to_string().contains("encode failed"), "got: {err}");
}

/// Bounded PRIMITIVE sequence over-bound rejection (the second
/// resize_function call site, write_prim_seq_cpp): the PrimSeq count
/// derives from the offset-table LEN — no count-plausibility check
/// runs first, so this is the path where ONLY the bound guard stands
/// between a hostile frame and BoundedVector::resize's throw.
#[test]
fn cpp_bounded_prim_sequence_over_bound_rejected_before_resize() {
    static PRIM_RESIZE_CALLS: AtomicUsize = AtomicUsize::new(0);
    unsafe extern "C" fn prim_resize(field: *mut c_void, size: usize) {
        PRIM_RESIZE_CALLS.fetch_add(1, Ordering::SeqCst);
        let v = &mut *(field as *mut FakeSeq);
        let mut storage = vec![0f64; size.max(1)];
        v.data = storage.as_mut_ptr() as *mut u8;
        v.len = size;
        std::mem::forget(storage);
    }
    let mut m = member("vals", ROS_TYPE_DOUBLE, 0, true, std::ptr::null());
    m.array_size_ = 2;
    m.is_upper_bound_ = true; // bounded: <=2 doubles
    m.size_function = Some(seq_size);
    m.get_const_function = Some(seq_get_const);
    m.get_function = Some(seq_get);
    m.resize_function = Some(prim_resize);
    let members = make_members(
        "test_msgs::msg",
        "CppBoundedPrim",
        std::mem::size_of::<FakeSeq>(),
        vec![m],
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");

    // Well-formed empty frame, then grow the entry to 3 doubles (24 bytes >
    // bound 2): patch the table len and append the bytes.
    let msg = FakeSeq {
        data: std::ptr::null_mut(),
        len: 0,
        elem: 8,
    };
    let mut frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    let table_base = bridge.layout.fixed_size;
    let payload_len_before = frame.len() - WireHeader::SIZE;
    let off_at = WireHeader::SIZE + table_base;
    // offset points at the (aligned) end of the current payload
    frame[off_at..off_at + 4].copy_from_slice(&(payload_len_before as u32).to_le_bytes());
    frame[off_at + 4..off_at + 8].copy_from_slice(&24u32.to_le_bytes());
    frame.extend_from_slice(&[0u8; 24]);

    PRIM_RESIZE_CALLS.store(0, Ordering::SeqCst);
    let mut out = FakeSeq {
        data: std::ptr::null_mut(),
        len: 0,
        elem: 8,
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(!ok, "over-bound prim count must be rejected");
    assert_eq!(
        PRIM_RESIZE_CALLS.load(Ordering::SeqCst),
        0,
        "resize_function must not run for an over-bound prim sequence"
    );
}

/// Front-line malformed-frame guard (type_bridge_cpp `unflatten`): a
/// payload shorter than `fixed_size + offset_table_bytes` must be
/// rejected (false), never partially-copied into the C++ message. Uses
/// the all-fixed Vector3 (fixed_size 24, no offset table) so the only
/// gate is the length check.
#[test]
fn cpp_unflatten_rejects_too_short_payload() {
    let bridge = unsafe { CppBridgedMessage::new(vector3_members_cpp()) }.expect("bridge");
    assert_eq!(bridge.layout.fixed_size, 24);

    let mut out = CppVector3::default();
    // 23 bytes < 24-byte fixed section → reject.
    let short = [0u8; 23];
    let ok = unsafe { bridge.unflatten(&short, &mut out as *mut _ as *mut c_void) };
    assert!(
        !ok,
        "payload shorter than the fixed section must be rejected"
    );
    // Empty payload → reject too (boundary).
    let ok_empty = unsafe { bridge.unflatten(&[], &mut out as *mut _ as *mut c_void) };
    assert!(!ok_empty, "empty payload must be rejected");
    // Exactly the fixed size succeeds (proves 23 was rejected by the
    // length guard, not an unrelated failure).
    let exact = [0u8; 24];
    let ok_exact = unsafe { bridge.unflatten(&exact, &mut out as *mut _ as *mut c_void) };
    assert!(ok_exact, "an exactly-fixed-size payload must decode");
}

/// Front-line malformed-frame guard (type_bridge_cpp `unflatten`
/// PrimSeq arm): a primitive-sequence entry whose byte length is NOT a
/// multiple of the element size is corrupt and must be rejected BEFORE
/// any resize_function runs (a partial element would read past the
/// decoded bytes).
#[test]
fn cpp_unflatten_rejects_prim_seq_with_non_multiple_length() {
    static RESIZED: AtomicUsize = AtomicUsize::new(0);
    unsafe extern "C" fn spy_resize(field: *mut c_void, size: usize) {
        RESIZED.fetch_add(1, Ordering::SeqCst);
        let v = &mut *(field as *mut FakeSeq);
        let mut storage = vec![0f64; size.max(1)];
        v.data = storage.as_mut_ptr() as *mut u8;
        v.len = size;
        std::mem::forget(storage);
    }
    // Unbounded vector<f64> (no bound guard in play — isolates the
    // `bytes.len() % elem_size` check).
    let mut m = member("vals", ROS_TYPE_DOUBLE, 0, true, std::ptr::null());
    m.size_function = Some(seq_size);
    m.get_const_function = Some(seq_get_const);
    m.get_function = Some(seq_get);
    m.resize_function = Some(spy_resize);
    let members = make_members(
        "test_msgs::msg",
        "CppOddPrim",
        std::mem::size_of::<FakeSeq>(),
        vec![m],
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");

    // Well-formed empty frame, then grow the entry to 7 bytes (not a multiple
    // of 8): patch the offset-table len + append the trailing bytes.
    let msg = FakeSeq {
        data: std::ptr::null_mut(),
        len: 0,
        elem: 8,
    };
    let mut frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    let table_base = bridge.layout.fixed_size;
    let off_at = WireHeader::SIZE + table_base;
    let payload_len_before = frame.len() - WireHeader::SIZE;
    frame[off_at..off_at + 4].copy_from_slice(&(payload_len_before as u32).to_le_bytes());
    frame[off_at + 4..off_at + 8].copy_from_slice(&7u32.to_le_bytes());
    frame.extend_from_slice(&[0u8; 7]);

    RESIZED.store(0, Ordering::SeqCst);
    let mut out = FakeSeq {
        data: std::ptr::null_mut(),
        len: 0,
        elem: 8,
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(
        !ok,
        "a prim-seq entry length that isn't a multiple of elem_size must be rejected"
    );
    assert_eq!(
        RESIZED.load(Ordering::SeqCst),
        0,
        "the length guard must reject before resize_function runs"
    );
}

/// FIXED `std::array<f64, N>` member (is_array_ + array_size_>0 +
/// !is_upper_bound_, NO size/resize fns — inline storage). This is the
/// covariance-array shape MoveIt messages use (`double[36]`, quaternions
/// as `double[4]`). It must take the FixedCopy path: a fixed-size span,
/// zero-copy loanable, NOT a variable entry.
#[test]
fn cpp_fixed_primitive_array_is_loanable_and_roundtrips() {
    #[repr(C)]
    #[derive(Clone, Copy, PartialEq, Debug)]
    struct CppQuat {
        vals: [f64; 4],
    }
    let mut m = member("vals", ROS_TYPE_DOUBLE, 0, true, std::ptr::null());
    m.array_size_ = 4; // FIXED array of 4 (no is_upper_bound_, no fns)
    let members = make_members(
        "geometry_msgs::msg",
        "CppQuat",
        std::mem::size_of::<CppQuat>(),
        vec![m],
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");
    // All-fixed (4×f64 = 32B inline) ⇒ zero-copy loanable.
    assert!(
        bridge.can_loan,
        "a fixed primitive array message must be loanable (no variable fields)"
    );
    assert_eq!(bridge.layout.fixed_size, 32);

    let msg = CppQuat {
        vals: [1.0, -2.5, 3.25, 4.125],
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    assert_eq!(frame.len(), WireHeader::SIZE + 32, "no variable entries");
    // Independent oracle: the fixed section is 4 packed LE f64s.
    let p = &frame[WireHeader::SIZE..];
    for (i, want) in msg.vals.iter().enumerate() {
        let got = f64::from_le_bytes(p[i * 8..i * 8 + 8].try_into().unwrap());
        assert_eq!(got, *want, "element {i}");
    }

    let mut out = CppQuat { vals: [0.0; 4] };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out, msg);
}

// =====================================================================
// Flatten-into-loan: frame_size + flatten_into through the cpp
// bridge (function-pointer containers, REAL std::string via the shim,
// bit-packed vector<bool> straight into the cursor).
// =====================================================================

/// Poison-buffer byte-identity helper — see the C bridge twin: proves
/// frame_size == flatten length, exact-end cursor, and that no prior
/// buffer contents leak into the output.
fn assert_cpp_flatten_into_identical(bridge: &CppBridgedMessage, msg: *const c_void) -> Vec<u8> {
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
    // Complementary poison,
    // mirroring the C helper. flatten() shares the cursor
    // code path, so buf==expected alone is self-comparison; two runs
    // over COMPLEMENTARY poisons must agree at every byte — the
    // deterministic unwritten-byte detector matters MOST here (the
    // cpp bridge's vector<bool> bit-fetch loop is the likeliest
    // place to miss a byte).
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
fn cpp_flatten_into_matches_flatten_for_mixed_message() {
    let bridge = unsafe { CppBridgedMessage::new(jointish_members_cpp()) }.expect("bridge");
    let mut positions = [1.5f64, -2.25, 4.125];
    let mut msg = CppJointish {
        stamp_sec: 9.0,
        position: FakeVecF64 {
            data: positions.as_mut_ptr(),
            len: positions.len(),
            cap: positions.len(),
        },
        name: StringSlot([0; 32]),
    };
    msg.name.construct("panda_joint1");
    assert_cpp_flatten_into_identical(&bridge, &msg as *const _ as *const c_void);
    msg.name.destruct();
}

/// vector<bool> through flatten_into: the bit-packed fetch loop writes
/// straight into the cursor's zeroed span — identical bytes to flatten.
#[test]
fn cpp_flatten_into_matches_flatten_for_bool_sequence() {
    #[repr(C)]
    struct CppFlags {
        flags: FakeVecBool,
    }
    let mut m = member("flags", ROS_TYPE_BOOLEAN, 0, true, std::ptr::null());
    m.size_function = Some(vecbool_size);
    m.fetch_function = Some(vecbool_fetch);
    m.assign_function = Some(vecbool_assign);
    m.resize_function = Some(vecbool_resize);
    let members = make_members(
        "test_msgs::msg",
        "FlagsInto",
        std::mem::size_of::<CppFlags>(),
        vec![m],
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");

    let pattern = [true, false, true, true, false, false, true, false, true];
    let mut bits = vec![0u8; 2];
    for (i, &b) in pattern.iter().enumerate() {
        if b {
            bits[i / 8] |= 1 << (i % 8);
        }
    }
    let msg = CppFlags {
        flags: FakeVecBool {
            bits: bits.as_mut_ptr(),
            len: pattern.len(),
        },
    };
    let frame = assert_cpp_flatten_into_identical(&bridge, &msg as *const _ as *const c_void);
    let wire: Vec<bool> = frame[frame.len() - pattern.len()..]
        .iter()
        .map(|&b| b != 0)
        .collect();
    assert_eq!(wire, pattern);
}

/// Edge: an EMPTY `vector<bool>` through flatten_into — the bit-fetch
/// loop (the spot the helper flags as "likeliest to miss a byte") must
/// take its zero-count branch and still produce a byte-identical,
/// fully-initialized frame (an empty variable entry, no fetch calls).
#[test]
fn cpp_flatten_into_matches_flatten_for_empty_bool_sequence() {
    #[repr(C)]
    struct CppFlags {
        flags: FakeVecBool,
    }
    let mut m = member("flags", ROS_TYPE_BOOLEAN, 0, true, std::ptr::null());
    m.size_function = Some(vecbool_size);
    m.fetch_function = Some(vecbool_fetch);
    m.assign_function = Some(vecbool_assign);
    m.resize_function = Some(vecbool_resize);
    let members = make_members(
        "test_msgs::msg",
        "EmptyFlagsInto",
        std::mem::size_of::<CppFlags>(),
        vec![m],
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");

    let msg = CppFlags {
        flags: FakeVecBool {
            bits: std::ptr::null_mut(),
            len: 0,
        },
    };
    // Byte-identity + complementary-poison (via the helper) over a
    // zero-length bool sequence — proves the empty branch leaves no
    // uninitialized byte.
    let frame = assert_cpp_flatten_into_identical(&bridge, &msg as *const _ as *const c_void);
    let header = WireHeader::read_from_buf(&frame).expect("header");
    assert_eq!(header.offset_table_count, 1, "one (empty) variable entry");
}

#[test]
fn cpp_flatten_into_undersized_buffer_errs_without_oob() {
    let bridge = unsafe { CppBridgedMessage::new(jointish_members_cpp()) }.expect("bridge");
    let mut positions = [3.5f64, 7.25];
    let mut msg = CppJointish {
        stamp_sec: 1.0,
        position: FakeVecF64 {
            data: positions.as_mut_ptr(),
            len: positions.len(),
            cap: positions.len(),
        },
        name: StringSlot([0; 32]),
    };
    msg.name.construct("link");
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
    assert!(res.is_err(), "undersized buffer must error");
    assert!(
        buf[size - 1..].iter().all(|&b| b == 0xCD),
        "no byte beyond the undersized slice may be touched"
    );
    msg.name.destruct();
}

/// Corrupt container sizes (hostile size_function) are rejected in the
/// SIZE PRE-PASS — before any loan could be sized from them.
#[test]
fn cpp_frame_size_rejects_corrupt_sizes() {
    let bridge = unsafe { CppBridgedMessage::new(jointish_members_cpp()) }.expect("bridge");
    let mut msg = CppJointish {
        stamp_sec: 1.0,
        position: FakeVecF64 {
            data: std::ptr::NonNull::<f64>::dangling().as_ptr(),
            len: usize::MAX, // hostile: read back by vecf64_size
            cap: 0,
        },
        name: StringSlot([0; 32]),
    };
    msg.name.construct("");
    let err = unsafe { bridge.frame_size(&msg as *const _ as *const c_void) }
        .expect_err("hostile count must be rejected by the pre-pass");
    assert!(err.to_string().contains("encode failed"), "got: {err}");
    msg.name.destruct();
}

// =====================================================================
// Uint8[] assign fast path (skips resize's value-init memset).
//
// These use a REAL std::vector<uint8_t> (via the shim fixtures) as BOTH
// the flatten source and the unflatten destination, because the decode
// fast path calls assign() directly on the C++ container -- a fake
// repr(C) header would be reinterpreted as a real std::vector and crash.
// The resize spy proves the fast path was taken (resize is NEVER called
// for the uint8 sequence). The int8 companion proves the gate holds: a
// std::vector<int8_t> stays on the resize()+copy path (never the shim).
// =====================================================================

use rmw_cerulion::ffi::introspection_cpp::{
    rmw_cerulion_vector_u8_capacity, rmw_cerulion_vector_u8_construct, rmw_cerulion_vector_u8_data,
    rmw_cerulion_vector_u8_destruct, rmw_cerulion_vector_u8_size, rmw_cerulion_vector_u8_sizeof,
};

// The production `rmw_cerulion_vector_u8_assign` extern is `pub(crate)`
// (it is a heap-corruption footgun -- a static_cast of any
// void* to std::vector<uint8_t>* -- so production reaches it only through
// the gated `assign_u8_vector` wrapper). These fixtures still need the raw
// symbol to build REAL C++ vectors, so -- like the other repr(C) fixture
// externs in this file -- they declare their OWN extern block for it. The
// symbol is exported by the compiled shim regardless of Rust visibility.
extern "C" {
    fn rmw_cerulion_vector_u8_assign(v: *mut c_void, data: *const u8, len: usize);
}

const ROS_TYPE_UINT8_LOCAL: u8 = 8;
const ROS_TYPE_INT8_LOCAL: u8 = 9;
const ROS_TYPE_OCTET_LOCAL: u8 = 7;

/// Aligned storage for one std::vector<uint8_t> (24 bytes on libstdc++
/// and libc++; over-aligned to 16 for headroom).
#[repr(C, align(16))]
struct VecU8Slot([u8; 32]);

impl VecU8Slot {
    fn construct(&mut self) {
        assert!(
            unsafe { rmw_cerulion_vector_u8_sizeof() } <= 32,
            "vector slot too small"
        );
        unsafe { rmw_cerulion_vector_u8_construct(self.0.as_mut_ptr() as *mut c_void) };
    }
    fn assign(&mut self, bytes: &[u8]) {
        unsafe {
            rmw_cerulion_vector_u8_assign(
                self.0.as_mut_ptr() as *mut c_void,
                bytes.as_ptr(),
                bytes.len(),
            )
        };
    }
    fn size(&self) -> usize {
        unsafe { rmw_cerulion_vector_u8_size(self.0.as_ptr() as *const c_void) }
    }
    fn capacity(&self) -> usize {
        unsafe { rmw_cerulion_vector_u8_capacity(self.0.as_ptr() as *const c_void) }
    }
    fn bytes(&self) -> Vec<u8> {
        let n = self.size();
        if n == 0 {
            return Vec::new();
        }
        let d = unsafe { rmw_cerulion_vector_u8_data(self.0.as_ptr() as *const c_void) };
        unsafe { std::slice::from_raw_parts(d, n) }.to_vec()
    }
    fn destruct(&mut self) {
        unsafe { rmw_cerulion_vector_u8_destruct(self.0.as_mut_ptr() as *mut c_void) }
    }
}

/// { data: std::vector<uint8_t> } -- the uint8[] payload shape.
#[repr(C)]
struct CppU8SeqMsg {
    data: VecU8Slot,
}

// Introspection accessors reading a REAL std::vector<uint8_t> (used only
// by the flatten/encode leg; the decode fast path ignores them).
unsafe extern "C" fn vecu8_size(field: *const c_void) -> usize {
    rmw_cerulion_vector_u8_size(field)
}
unsafe extern "C" fn vecu8_get_const(field: *const c_void, idx: usize) -> *const c_void {
    rmw_cerulion_vector_u8_data(field).add(idx) as *const c_void
}

static U8_RESIZE_CALLS: AtomicUsize = AtomicUsize::new(0);
unsafe extern "C" fn u8_resize_spy(_field: *mut c_void, _size: usize) {
    U8_RESIZE_CALLS.fetch_add(1, Ordering::SeqCst);
}

fn cpp_u8_seq_members() -> *const CppMessageMembers {
    let mut m = member("data", ROS_TYPE_UINT8_LOCAL, 0, true, std::ptr::null());
    m.size_function = Some(vecu8_size);
    m.get_const_function = Some(vecu8_get_const);
    // resize_function present ONLY as a spy: the uint8 fast path must not
    // call it. get_function is deliberately absent so a regression to the
    // slow path (which needs get_function) fails LOUDLY rather than
    // silently taking a different-but-correct route.
    m.resize_function = Some(u8_resize_spy);
    make_members(
        "test_msgs::msg",
        "CppU8Seq",
        std::mem::size_of::<CppU8SeqMsg>(),
        vec![m],
    )
}

#[test]
#[serial]
fn cpp_uint8_sequence_roundtrips_byte_exact_via_assign_fast_path() {
    let bridge = unsafe { CppBridgedMessage::new(cpp_u8_seq_members()) }.expect("bridge");

    // >256 bytes so a truncation/byte-swap bug crosses a byte boundary; a
    // per-index pattern so ANY wrong/stale byte mismatches.
    let payload: Vec<u8> = (0..300u32)
        .map(|i| (i.wrapping_mul(37).wrapping_add(1)) as u8)
        .collect();

    let mut src = CppU8SeqMsg {
        data: VecU8Slot([0; 32]),
    };
    src.data.construct();
    src.data.assign(&payload);
    let frame =
        unsafe { bridge.flatten(&src as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut dst = CppU8SeqMsg {
        data: VecU8Slot([0; 32]),
    };
    dst.data.construct();
    U8_RESIZE_CALLS.store(0, Ordering::SeqCst);
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut dst as *mut _ as *mut c_void,
        )
    };
    assert!(ok, "uint8[] decode must succeed");
    assert_eq!(
        U8_RESIZE_CALLS.load(Ordering::SeqCst),
        0,
        "uint8 assign fast path must NOT call resize()"
    );
    assert_eq!(dst.data.size(), payload.len(), "decoded size == count");
    assert_eq!(dst.data.bytes(), payload, "decoded content byte-exact");

    src.data.destruct();
    dst.data.destruct();
}

#[test]
#[serial]
fn cpp_uint8_assign_leaves_no_uninitialized_tail() {
    let bridge = unsafe { CppBridgedMessage::new(cpp_u8_seq_members()) }.expect("bridge");

    // Distinctive per-index payload (forced nonzero via `| 1`); a leaked
    // uninitialized/stale byte would not match.
    let payload: Vec<u8> = (0..200u32)
        .map(|i| (i.wrapping_mul(53).wrapping_add(3) | 1) as u8)
        .collect();
    let mut src = CppU8SeqMsg {
        data: VecU8Slot([0; 32]),
    };
    src.data.construct();
    src.data.assign(&payload);
    let frame =
        unsafe { bridge.flatten(&src as *const _ as *const c_void, 0, 0) }.expect("flatten");

    // Pre-DIRTY the destination: assign a LONGER sentinel buffer so the
    // vector's storage (and spare capacity) is full of 0xEE before the
    // decode. A partial copy / stale tail would leave 0xEE visible.
    let mut dst = CppU8SeqMsg {
        data: VecU8Slot([0; 32]),
    };
    dst.data.construct();
    dst.data.assign(&[0xEEu8; 512]);
    assert_eq!(dst.data.size(), 512);

    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut dst as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(dst.data.size(), payload.len(), "size shrinks to count");
    assert!(
        dst.data.capacity() >= payload.len(),
        "capacity holds the payload"
    );
    let decoded = dst.data.bytes();
    assert_eq!(decoded.len(), payload.len());
    for (i, (&got, &want)) in decoded.iter().zip(payload.iter()).enumerate() {
        assert_eq!(got, want, "byte {i} differs -- stale/uninit leak");
    }
    assert!(
        !decoded.contains(&0xEE),
        "no 0xEE sentinel byte survived the assign"
    );

    src.data.destruct();
    dst.data.destruct();
}

// Fake std::vector<int8_t> (repr(C) header + accessor fns) for the
// slow-path companion.
#[repr(C)]
struct FakeInt8Seq {
    data: *mut i8,
    len: usize,
}
static I8_RESIZE_CALLS: AtomicUsize = AtomicUsize::new(0);
unsafe extern "C" fn i8_size(field: *const c_void) -> usize {
    (*(field as *const FakeInt8Seq)).len
}
unsafe extern "C" fn i8_get_const(field: *const c_void, idx: usize) -> *const c_void {
    let v = &*(field as *const FakeInt8Seq);
    v.data.add(idx) as *const c_void
}
unsafe extern "C" fn i8_get(field: *mut c_void, idx: usize) -> *mut c_void {
    let v = &mut *(field as *mut FakeInt8Seq);
    v.data.add(idx) as *mut c_void
}
unsafe extern "C" fn i8_resize(field: *mut c_void, size: usize) {
    I8_RESIZE_CALLS.fetch_add(1, Ordering::SeqCst);
    let v = &mut *(field as *mut FakeInt8Seq);
    let mut storage = vec![0i8; size.max(1)];
    v.data = storage.as_mut_ptr();
    v.len = size;
    std::mem::forget(storage); // fixture: process-lifetime
}

/// { data: std::vector<int8_t> }
#[repr(C)]
struct CppI8SeqMsg {
    data: FakeInt8Seq,
}

/// Anti-tautology / non-regression: int8[] must STAY on the resize()+copy
/// slow path (NEVER the uint8 assign shim, which would reinterpret a
/// std::vector<int8_t> as std::vector<uint8_t> -- UB). Proven by the
/// resize spy firing exactly once and the decode being byte-exact.
#[test]
#[serial]
fn cpp_int8_sequence_stays_on_resize_slow_path() {
    let mut m = member("data", ROS_TYPE_INT8_LOCAL, 0, true, std::ptr::null());
    m.size_function = Some(i8_size);
    m.get_const_function = Some(i8_get_const);
    m.get_function = Some(i8_get);
    m.resize_function = Some(i8_resize);
    let members = make_members(
        "test_msgs::msg",
        "CppI8Seq",
        std::mem::size_of::<CppI8SeqMsg>(),
        vec![m],
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");

    let payload_i8: Vec<i8> = (0..120i32).map(|i| (i - 60) as i8).collect();
    let payload_bytes: Vec<u8> = payload_i8.iter().map(|&b| b as u8).collect();
    let mut src_storage = payload_i8.clone().into_boxed_slice();
    let src = CppI8SeqMsg {
        data: FakeInt8Seq {
            data: src_storage.as_mut_ptr(),
            len: src_storage.len(),
        },
    };
    let frame =
        unsafe { bridge.flatten(&src as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut dst = CppI8SeqMsg {
        data: FakeInt8Seq {
            data: std::ptr::null_mut(),
            len: 0,
        },
    };
    I8_RESIZE_CALLS.store(0, Ordering::SeqCst);
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut dst as *mut _ as *mut c_void,
        )
    };
    assert!(ok, "int8[] decode must succeed");
    assert_eq!(
        I8_RESIZE_CALLS.load(Ordering::SeqCst),
        1,
        "int8 must go through resize() (NOT the uint8 assign shim)"
    );
    assert_eq!(dst.data.len, payload_i8.len(), "decoded size == count");
    let decoded = unsafe { std::slice::from_raw_parts(dst.data.data as *const u8, dst.data.len) };
    assert_eq!(decoded, payload_bytes.as_slice(), "int8 content byte-exact");
}

// ---------------------------------------------------------------------
// Fake byte sequence (repr(C) header + accessor fns) for the two
// slow-path gate-boundary companions: BOUNDED uint8 (is_upper_bound_) and
// OCTET (type_id_ != UINT8). Both are byte-sized but MUST NOT reach the
// std::vector<uint8_t> assign fast path -- a fake proves it (the fast
// path would static_cast this repr(C) header as a real std::vector and
// crash / corrupt; the slow path only touches it via the fn pointers).
// ---------------------------------------------------------------------
#[repr(C)]
struct FakeU8Seq {
    data: *mut u8,
    len: usize,
}
unsafe extern "C" fn fu8_size(field: *const c_void) -> usize {
    (*(field as *const FakeU8Seq)).len
}
unsafe extern "C" fn fu8_get_const(field: *const c_void, idx: usize) -> *const c_void {
    (*(field as *const FakeU8Seq)).data.add(idx) as *const c_void
}
unsafe extern "C" fn fu8_get(field: *mut c_void, idx: usize) -> *mut c_void {
    (*(field as *mut FakeU8Seq)).data.add(idx) as *mut c_void
}
static BOUNDED_U8_RESIZE_CALLS: AtomicUsize = AtomicUsize::new(0);
unsafe extern "C" fn bounded_u8_resize(field: *mut c_void, size: usize) {
    BOUNDED_U8_RESIZE_CALLS.fetch_add(1, Ordering::SeqCst);
    let v = &mut *(field as *mut FakeU8Seq);
    let mut storage = vec![0u8; size.max(1)];
    v.data = storage.as_mut_ptr();
    v.len = size;
    std::mem::forget(storage); // fixture: process-lifetime
}
static OCTET_RESIZE_CALLS: AtomicUsize = AtomicUsize::new(0);
unsafe extern "C" fn octet_resize(field: *mut c_void, size: usize) {
    OCTET_RESIZE_CALLS.fetch_add(1, Ordering::SeqCst);
    let v = &mut *(field as *mut FakeU8Seq);
    let mut storage = vec![0u8; size.max(1)];
    v.data = storage.as_mut_ptr();
    v.len = size;
    std::mem::forget(storage); // fixture: process-lifetime
}

/// { data: <byte sequence> } behind the FakeU8Seq header.
#[repr(C)]
struct CppFakeU8SeqMsg {
    data: FakeU8Seq,
}

/// Drive one byte-sequence member through flatten→unflatten and return
/// (resize-spy count, decoded bytes). Shared by the bounded-uint8 and
/// octet gate-boundary tests.
fn roundtrip_fake_u8_seq(
    m: CppMessageMember,
    spy: &AtomicUsize,
    payload: &[u8],
) -> (usize, Vec<u8>) {
    let members = make_members(
        "test_msgs::msg",
        "CppFakeU8Seq",
        std::mem::size_of::<CppFakeU8SeqMsg>(),
        vec![m],
    );
    let bridge = unsafe { CppBridgedMessage::new(members) }.expect("bridge");

    let mut src_storage = payload.to_vec().into_boxed_slice();
    let src = CppFakeU8SeqMsg {
        data: FakeU8Seq {
            data: src_storage.as_mut_ptr(),
            len: src_storage.len(),
        },
    };
    let frame =
        unsafe { bridge.flatten(&src as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut dst = CppFakeU8SeqMsg {
        data: FakeU8Seq {
            data: std::ptr::null_mut(),
            len: 0,
        },
    };
    spy.store(0, Ordering::SeqCst);
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut dst as *mut _ as *mut c_void,
        )
    };
    assert!(ok, "byte-sequence decode must succeed");
    let calls = spy.load(Ordering::SeqCst);
    let decoded = if dst.data.data.is_null() {
        Vec::new()
    } else {
        unsafe { std::slice::from_raw_parts(dst.data.data, dst.data.len) }.to_vec()
    };
    (calls, decoded)
}

/// Bound-flipped mirror of `cpp_int8_sequence_stays_on_resize_slow_path`:
/// a BOUNDED uint8[<=N] has `type_id_ == UINT8` but `is_upper_bound_ =
/// true`, so the fast path's `!is_upper_bound_` term MUST keep it on the
/// resize()+copy slow path (a rosidl BoundedVector<uint8_t,N> is NOT a
/// plain std::vector<uint8_t> -- reinterpreting it would be UB). Proven by
/// an IN-bounds payload, the resize spy firing EXACTLY once, byte-exact.
#[test]
#[serial]
fn cpp_bounded_uint8_under_bound_stays_on_resize_slow_path() {
    let mut m = member("data", ROS_TYPE_UINT8_LOCAL, 0, true, std::ptr::null());
    m.is_upper_bound_ = true;
    m.array_size_ = 256; // bound N
    m.size_function = Some(fu8_size);
    m.get_const_function = Some(fu8_get_const);
    m.get_function = Some(fu8_get);
    m.resize_function = Some(bounded_u8_resize);

    // IN-bounds payload (100 <= 256), per-index pattern.
    let payload: Vec<u8> = (0..100u32)
        .map(|i| (i.wrapping_mul(37).wrapping_add(1)) as u8)
        .collect();
    let (calls, decoded) = roundtrip_fake_u8_seq(m, &BOUNDED_U8_RESIZE_CALLS, &payload);
    assert_eq!(
        calls, 1,
        "bounded uint8 must go through resize() -- the !is_upper_bound_ gate term is load-bearing"
    );
    assert_eq!(decoded, payload, "bounded uint8 content byte-exact");
}

/// OCTET (type_id_ == 7, NOT UINT8) is byte-sized and maps to the
/// same U8 wire field as uint8, but MUST be excluded from the assign fast
/// path -- documenting the gate boundary is UINT8-EXACT (octet's C++
/// container is std::vector<std::byte>/<unsigned char>, not
/// std::vector<uint8_t>). Proven by the resize spy firing once.
#[test]
#[serial]
fn cpp_octet_stays_on_resize_slow_path() {
    let mut m = member("data", ROS_TYPE_OCTET_LOCAL, 0, true, std::ptr::null());
    m.size_function = Some(fu8_size);
    m.get_const_function = Some(fu8_get_const);
    m.get_function = Some(fu8_get);
    m.resize_function = Some(octet_resize);

    let payload: Vec<u8> = (0..120u32)
        .map(|i| (i.wrapping_mul(53).wrapping_add(3) | 1) as u8)
        .collect();
    let (calls, decoded) = roundtrip_fake_u8_seq(m, &OCTET_RESIZE_CALLS, &payload);
    assert_eq!(
        calls, 1,
        "octet must go through resize() (the fast-path gate is UINT8-exact, octet excluded)"
    );
    assert_eq!(decoded, payload, "octet content byte-exact");
}

/// Empty edge, C++ fast path: an empty uint8[] (count == 0). The
/// fast path fires BEFORE the count==0 short-circuit, so assign(p, p) runs
/// on an empty vector -- must NOT crash, must decode to size 0, must NOT
/// call resize(). The destination is pre-dirtied so a no-op decode fails.
#[test]
#[serial]
fn cpp_uint8_empty_sequence_roundtrips_on_assign_fast_path() {
    let bridge = unsafe { CppBridgedMessage::new(cpp_u8_seq_members()) }.expect("bridge");

    let mut src = CppU8SeqMsg {
        data: VecU8Slot([0; 32]),
    };
    src.data.construct(); // empty vector, no assign
    assert_eq!(src.data.size(), 0);
    let frame =
        unsafe { bridge.flatten(&src as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut dst = CppU8SeqMsg {
        data: VecU8Slot([0; 32]),
    };
    dst.data.construct();
    dst.data.assign(&[0x7Au8; 16]); // pre-dirty: a no-op decode would leave these
    assert_eq!(dst.data.size(), 16);
    U8_RESIZE_CALLS.store(0, Ordering::SeqCst);
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut dst as *mut _ as *mut c_void,
        )
    };
    assert!(ok, "empty uint8[] decode must succeed");
    assert_eq!(
        U8_RESIZE_CALLS.load(Ordering::SeqCst),
        0,
        "empty uint8[] still takes the assign fast path (assign(p, p), no resize)"
    );
    assert_eq!(dst.data.size(), 0, "decoded empty");
    assert!(dst.data.bytes().is_empty(), "no bytes survive");

    src.data.destruct();
    dst.data.destruct();
}

/// Size representativeness: a >=1 MiB uint8[] through the C++
/// assign fast path (the other C++ tests are ~300 B; the profiled case is
/// 4 MB). Exercises a multi-page alloc+copy -- still byte-exact, still
/// resize-free.
#[test]
#[serial]
fn cpp_uint8_large_1mib_roundtrips_via_assign_fast_path() {
    let bridge = unsafe { CppBridgedMessage::new(cpp_u8_seq_members()) }.expect("bridge");

    let n = 1usize << 20; // 1 MiB
    let payload: Vec<u8> = (0..n)
        .map(|i| (i as u8).wrapping_mul(37).wrapping_add((i >> 8) as u8))
        .collect();

    let mut src = CppU8SeqMsg {
        data: VecU8Slot([0; 32]),
    };
    src.data.construct();
    src.data.assign(&payload);
    let frame =
        unsafe { bridge.flatten(&src as *const _ as *const c_void, 0, 0) }.expect("flatten");

    let mut dst = CppU8SeqMsg {
        data: VecU8Slot([0; 32]),
    };
    dst.data.construct();
    U8_RESIZE_CALLS.store(0, Ordering::SeqCst);
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut dst as *mut _ as *mut c_void,
        )
    };
    assert!(ok, "1 MiB uint8[] decode must succeed");
    assert_eq!(
        U8_RESIZE_CALLS.load(Ordering::SeqCst),
        0,
        "1 MiB uint8[] takes the assign fast path (no resize)"
    );
    assert_eq!(dst.data.size(), n, "decoded size == 1 MiB");
    assert_eq!(dst.data.bytes(), payload, "1 MiB byte-exact");

    src.data.destruct();
    dst.data.destruct();
}

/// The C++ bridge's variable-nested ELEMENT BODY is the CANONICAL
/// encoding, against a HAND-BUILT byte oracle and the independent
/// `FrameWalker`.
///
/// This closes a real gap. The C++ bridge is a token-for-token twin of the C
/// bridge's element codec and must stay in lockstep with it, but every
/// other C++ test for this shape is a self-round-trip
/// (`cpp_complex_codec_variable_nested_seq_roundtrips_deterministically`), and
/// the file's only native-reader oracles cover a fixed `Vector3` and a bare
/// `String` — neither reaches an element body. So nothing else here can tell a
/// canonical element from a packed one, and this is the bridge rclcpp and
/// MoveIt actually use.
///
/// `std_msgs/String` is the sharpest possible discriminator for this shape:
/// it has no fixed fields and one variable field, so the canonical body
/// (`[8-byte table][utf8]`) and a packed body (`[u32 count=1][u32
/// len][utf8]`) are the SAME LENGTH and differ only in whether the leading
/// `u32` is the offset `8` or the variable-member count `1`.
///
/// Fails against a packed cpp encoder (packing the
/// encoder alone is sufficient to make this test bite;
/// with BOTH the encoder and the decoder packed, the self-round-trip
/// bridge tests all still pass, which is why this byte oracle exists).
#[test]
#[serial]
fn cpp_variable_nested_element_body_is_canonical() {
    use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, FrameWalker};

    let bridge = unsafe { CppBridgedMessage::new(cpp_item_seq_members()) }.expect("bridge");
    let mut items: Vec<CppStringMsg> = (0..2)
        .map(|_| CppStringMsg {
            data: StringSlot([0; 32]),
        })
        .collect();
    items[0].data.construct("alpha");
    items[1].data.construct("beta-longer");
    let msg = CppItemSeqMsg {
        items: FakeSeq {
            data: items.as_mut_ptr() as *mut u8,
            len: 2,
            elem: std::mem::size_of::<CppStringMsg>(),
        },
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 0, 0) }.expect("flatten");
    let payload = &frame[WireHeader::SIZE..];

    // CppItemSeq has no fixed fields and one variable field, so its offset
    // table sits at payload offset 0 and the blob starts at 8.
    let off = u32::from_le_bytes(payload[0..4].try_into().unwrap()) as usize;
    let len = u32::from_le_bytes(payload[4..8].try_into().unwrap()) as usize;
    let blob = &payload[off..off + len];

    // HAND oracle — built here from the .msg shape, never via the codec.
    // std_msgs/String body: 0 fixed + one 8-byte offset entry at 0, so the
    // entry reads (8, len) and the UTF-8 follows at 8.
    let string_body = |s: &str| {
        let mut b = Vec::new();
        b.extend_from_slice(&8u32.to_le_bytes());
        b.extend_from_slice(&(s.len() as u32).to_le_bytes());
        b.extend_from_slice(s.as_bytes());
        b
    };
    let mut expected = Vec::new();
    expected.extend_from_slice(&2u32.to_le_bytes()); // element count
    for s in ["alpha", "beta-longer"] {
        let body = string_body(s);
        expected.extend_from_slice(&(body.len() as u32).to_le_bytes());
        expected.extend_from_slice(&body);
    }
    assert_eq!(
        blob, expected,
        "the cpp bridge's element bodies must be canonical byte for byte"
    );

    // Name the discriminator explicitly: bytes [8..12] of the blob are the
    // first element body's offset-table OFFSET (8). A packed encoder writes
    // the variable-member COUNT (1) here, in a body of identical length.
    assert_eq!(
        &blob[8..12],
        &8u32.to_le_bytes(),
        "element body must open with the offset-table entry's OFFSET (8), \
         not the packed variable-member count (1)"
    );

    // The INDEPENDENT decoder, over schemas parsed from .msg text rather than
    // from the introspection data the bridge encoded from.
    let schemas = vec![
        parse_rosmsg("string data\n", "String", Some("std_msgs")).expect("String"),
        parse_rosmsg("std_msgs/String[] items\n", "CppItemSeq", Some("test_msgs"))
            .expect("CppItemSeq"),
    ];
    let (walker, _w) = FrameWalker::new(schemas);
    let fv = walker
        .walk_by_hash(&frame)
        .expect("cpp frame must resolve + walk");
    let FrameValueKind::NestedArray { elements, .. } = fv.field("items").expect("items") else {
        panic!("items must decode as a canonical element list, not opaque");
    };
    assert_eq!(elements.len(), 2);
    for (i, want) in ["alpha", "beta-longer"].iter().enumerate() {
        let FrameValueKind::Nested(e) = &elements[i] else {
            panic!("element {i} must decode");
        };
        assert_eq!(e.field("data"), Some(&FrameValueKind::Str(want)));
    }

    items[0].data.destruct();
    items[1].data.destruct();
}

// =====================================================================
// Init_loaned_payload + loan_pad_ranges — C++ twins of the
// bridge_test.rs pins. The cpp arm's contract differs from C's: the
// generated ALL constructor writes EVERY member, so the bridge runs NO
// zero pre-pass (no O(payload) memset) — padding is publish's
// job (`loan_pad_ranges`).
// =====================================================================

/// bool-then-f64: 7 repr(C) padding bytes at [1, 8).
#[repr(C)]
struct CppPadFlagVal {
    flag: u8,
    value: f64,
}

/// Call COUNT plus the count of calls whose argument was NOT `ALL` —
/// asserted in the test body, never here (a panic must not cross the C
/// ABI). A last-value record would pass a bridge that called init twice,
/// or first with a wrong value and then with ALL; the pair rejects both.
static CPP_INIT_CALLS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static CPP_INIT_NON_ALL_ARGS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// 8-aligned byte buffer: the init callback dereferences the payload AS
/// the message struct (align 8), so a bare `[u8; N]` (align 1) would make
/// the test itself UB on a legal layout (the production slot is 8-aligned,
/// asserted fail-closed at borrow).
#[repr(C, align(8))]
struct Aligned8<const N: usize>([u8; N]);

/// Faithful model of rosidl_generator_cpp's `init_function`: placement-
/// new with `MessageInitialization::ALL` — EVERY member written (zeros
/// where no default is declared, the declared default otherwise; here
/// `value` declares 2.5).
unsafe extern "C" fn padded_cpp_init(msg: *mut c_void, init: u32) {
    CPP_INIT_CALLS.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    if init != rmw_cerulion::ffi::introspection_cpp::CPP_MSG_INIT_ALL {
        CPP_INIT_NON_ALL_ARGS.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    }
    let m = &mut *(msg as *mut CppPadFlagVal);
    m.flag = 0;
    m.value = 2.5;
}

fn padded_members_cpp_with_init() -> *const CppMessageMembers {
    let members = Box::leak(
        vec![
            member("flag", ROS_TYPE_BOOLEAN, 0, false, std::ptr::null()),
            member("value", ROS_TYPE_DOUBLE, 8, false, std::ptr::null()),
        ]
        .into_boxed_slice(),
    );
    Box::leak(Box::new(CppMessageMembers {
        message_namespace_: cstr("rmw_bridge__msg"),
        message_name_: cstr("PadFlagVal"),
        member_count_: members.len() as u32,
        size_of_: std::mem::size_of::<CppPadFlagVal>(),
        has_any_key_member_: false,
        members_: members.as_ptr(),
        init_function: Some(padded_cpp_init),
        fini_function: None,
    }))
}

#[test]
fn cpp_init_loaned_payload_runs_all_init_with_no_memset() {
    let bridge = unsafe { CppBridgedMessage::new(padded_members_cpp_with_init()) }.expect("bridge");
    assert!(bridge.can_loan);

    CPP_INIT_CALLS.store(0, std::sync::atomic::Ordering::SeqCst);
    CPP_INIT_NON_ALL_ARGS.store(0, std::sync::atomic::Ordering::SeqCst);
    let mut buf = Aligned8([0xAAu8; 16]);
    unsafe { bridge.init_loaned_payload(buf.0.as_mut_ptr() as *mut c_void) };
    assert_eq!(
        CPP_INIT_CALLS.load(std::sync::atomic::Ordering::SeqCst),
        1,
        "init_function must be called EXACTLY once"
    );
    assert_eq!(
        CPP_INIT_NON_ALL_ARGS.load(std::sync::atomic::Ordering::SeqCst),
        0,
        "every init_function call must carry ALL (0)"
    );
    assert_eq!(buf.0[0], 0, "ALL-init wrote the default-less bool");
    assert_eq!(
        f64::from_le_bytes(buf.0[8..16].try_into().expect("8 bytes")).to_bits(),
        2.5f64.to_bits(),
        "ALL-init wrote the declared default"
    );
    // The no-memset claim, made observable: PADDING still carries
    // the recycled-slot garbage after borrow-time init — zeroing it is
    // the publish path's job (`loan_pad_ranges`), never a whole-payload
    // memset on the cpp lane.
    assert!(
        buf.0[1..8].iter().all(|&b| b == 0xAA),
        "cpp init arm must not memset the payload"
    );
}

#[test]
fn cpp_init_loaned_payload_without_init_function_is_the_zeroed_baseline() {
    let bridge = unsafe { CppBridgedMessage::new(vector3_members_cpp()) }.expect("bridge");
    assert!(bridge.can_loan);
    let mut buf = Aligned8([0x55u8; 24]);
    unsafe { bridge.init_loaned_payload(buf.0.as_mut_ptr() as *mut c_void) };
    assert!(
        buf.0.iter().all(|&b| b == 0),
        "None-arm fallback is the documented zeroed baseline"
    );
}

#[test]
fn cpp_loan_pad_ranges_cover_exactly_the_padding() {
    let padded = unsafe { CppBridgedMessage::new(padded_members_cpp_with_init()) }.expect("bridge");
    assert_eq!(
        padded.loan_pad_ranges(),
        &[(1usize, 7usize)],
        "exactly the inter-field gap — never a field byte, never less than the gap"
    );
    let no_pad = unsafe { CppBridgedMessage::new(vector3_members_cpp()) }.expect("bridge");
    assert!(
        no_pad.loan_pad_ranges().is_empty(),
        "a gapless layout has nothing to zero"
    );
}

/// Memory safety: the adopt reuse pre-pass
/// releases a vector's buffer only when the vector OWNS one. Releasing
/// whenever `begin` is non-null would be wrong: a non-null `begin` does not
/// imply an allocation — this bridge mints the counter-example itself,
/// because `VecTriplet::forged` sets `end_of_storage == begin` for an
/// EMPTY forged entry. That leaves a live SHM address behind a capacity of
/// zero, with no hook registration (a zero-length range registers none),
/// so `::operator delete` on it reaches the interposed `free` as an
/// unknown pointer and is passed to the real `free`: an invalid
/// deallocation of shared memory, reachable by publishing an empty
/// sequence twice.
///
/// The non-owning leg aims `begin` at the MIDDLE of a live heap block,
/// which is what makes a wrong guard observable: handing a mid-allocation
/// pointer to `::operator delete` aborts the process on glibc and on
/// macOS, so restoring the `begin != 0` guard kills the test binary rather
/// than merely changing a value. The release itself is through the REAL
/// compiled shim (`rmw_cerulion_vector_pod_release`), not a stand-in.
///
/// Limit on the owning leg: "the buffer was freed" is not directly
/// observable from here — the shim's `::operator delete` goes to the
/// system allocator, which no Rust-side counter sees, and freeing it a
/// second time to prove it would itself be the double free. What that leg
/// pins is that an OWNING triplet is accepted by the guard and cleared,
/// so a guard that refuses everything (an over-correction into a leak)
/// fails this test.
#[test]
#[serial]
fn cpp_reuse_pre_pass_releases_only_a_vector_that_owns_storage() {
    let bridge =
        unsafe { CppBridgedMessage::new(jointish_forgeable_members_cpp()) }.expect("bridge");
    // The anti-vacuity premise, asserted
    // rather than assumed. With the plain
    // jointish fixture, whose `init_function`/`fini_function` are `None`,
    // bridge construction disables forging — so
    // `forged_sequence_count()` is always 0, the arm returns before
    // every assertion below, and it passes while pinning nothing. That is
    // a hard failure here: if the pre-pass has no forge-flagged field to
    // visit, this test is not testing what it claims and must say so.
    assert!(
        bridge.forged_sequence_count() > 0,
        "the fixture must be FORGEABLE or the pre-pass never visits its sequence and the \
         assertions below prove nothing"
    );
    let offset = std::mem::offset_of!(CppJointish, position);
    let triplet_at = |msg: &mut CppJointish| unsafe {
        (msg as *mut CppJointish as *mut u8).add(offset) as *mut VecTriplet
    };

    // (a) NON-OWNING: a live mid-block address with capacity 0 — exactly
    // the shape a zero-length forged entry leaves behind.
    let mut live: Vec<f64> = vec![11.0, 22.0, 33.0, 44.0];
    let mid = unsafe { live.as_mut_ptr().add(1) } as usize;
    let mut msg: CppJointish = unsafe { std::mem::zeroed() };
    unsafe {
        std::ptr::write_unaligned(
            triplet_at(&mut msg),
            VecTriplet {
                begin: mid,
                end: mid,
                end_of_storage: mid,
            },
        );
        bridge.release_forgeable_members(&mut msg as *mut _ as *mut c_void);
        let after = std::ptr::read_unaligned(triplet_at(&mut msg));
        assert_eq!(
            (after.begin, after.end, after.end_of_storage),
            (0, 0, 0),
            "the triplet must still be cleared, released or not"
        );
    }
    assert_eq!(
        live,
        vec![11.0, 22.0, 33.0, 44.0],
        "the block the non-owning triplet pointed into must be untouched"
    );

    // (b) OWNING: a real allocation with a non-zero capacity is released.
    //
    // The buffer comes from a real
    // `std::vector<uint8_t>` built through the shim, so the storage the
    // production path frees with `::operator delete` was allocated by the
    // `std::allocator` that pairs with it. Handing a Rust
    // `Box<[f64]>` to a C++ `operator delete` is an allocator mismatch that
    // is undefined behaviour and can abort, which would make this
    // arm's failures unreadable.
    let vec_bytes = unsafe { rmw_cerulion_vector_u8_sizeof() };
    let mut vec_slot: Vec<u64> = vec![0u64; vec_bytes.div_ceil(8)];
    let vec_ptr = vec_slot.as_mut_ptr() as *mut c_void;
    let payload: [u8; 32] = [7u8; 32];
    let (begin, end) = unsafe {
        rmw_cerulion_vector_u8_construct(vec_ptr);
        rmw_cerulion_vector_u8_assign(vec_ptr, payload.as_ptr(), payload.len());
        let data = rmw_cerulion_vector_u8_data(vec_ptr) as usize;
        let capacity = rmw_cerulion_vector_u8_capacity(vec_ptr);
        assert!(
            data != 0 && capacity >= payload.len(),
            "the shim-built vector must actually own storage"
        );
        (data, data + capacity)
    };
    let mut msg2: CppJointish = unsafe { std::mem::zeroed() };
    unsafe {
        std::ptr::write_unaligned(
            triplet_at(&mut msg2),
            VecTriplet {
                begin,
                end,
                end_of_storage: end,
            },
        );
        bridge.release_forgeable_members(&mut msg2 as *mut _ as *mut c_void);
        let after = std::ptr::read_unaligned(triplet_at(&mut msg2));
        assert_eq!(
            (after.begin, after.end, after.end_of_storage),
            (0, 0, 0),
            "an owning triplet is released and cleared"
        );
        // The production path has freed that buffer, so the vector object
        // must not free it again: zeroing its triplet leaves an EMPTY
        // vector, whose destructor owns nothing.
        std::ptr::write_bytes(vec_ptr as *mut u8, 0, vec_bytes);
        rmw_cerulion_vector_u8_destruct(vec_ptr);
    }

    // The predicate's boundary, both sides: capacity zero owns nothing
    // however non-null `begin` is; one byte of capacity owns something.
    assert!(!VecTriplet {
        begin: 0x1000,
        end: 0x1000,
        end_of_storage: 0x1000
    }
    .owns_storage());
    assert!(VecTriplet {
        begin: 0x1000,
        end: 0x1000,
        end_of_storage: 0x1001
    }
    .owns_storage());
}
