// SPDX-License-Identifier: AGPL-3.0-only
//! `rosidl::Buffer<uint8_t>` on Lyrical and Rolling: a Buffer on a backend
//! other than `cpu` (Isaac ROS 5.0's CUDA buffers) is published as a host
//! copy and counted, a take leaves a Buffer it fills on the CPU backend (a
//! C sequence that does not own its Buffer is detached instead), and
//! `acceptable_buffer_backends` names only installed backends.
//!
//! The Buffers are REAL `rosidl::Buffer<uint8_t>` objects built by the
//! compiled shim; the non-CPU one carries a fixture backend implemented the
//! way an accelerator backend is (its own `BufferImplBase` subclass), and
//! the C++ member accessors are the ones rosidl generates for a Buffer
//! member, compiled in the shim, so they throw on a non-CPU backend as the
//! generated ones do.
//!
//! The downgrade counter is process-wide, so every test runs `#[serial]`
//! and reads deltas.
#![cfg(all(unix, cerulion_has_is_rosidl_buffer))]

use serial_test::serial;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_void};

use cerulion_core::wire::WireHeader;
use rmw_cerulion::ffi::introspection_cpp::{CppMessageMember, CppMessageMembers};
use rmw_cerulion::ffi::{
    self, rosidl_message_type_support_t, rosidl_typesupport_introspection_c__MessageMember,
    rosidl_typesupport_introspection_c__MessageMembers, RMW_RET_OK,
};
use rmw_cerulion::rosidl_buffer::{downgrade_count, topic_downgrade_count};
use rmw_cerulion::type_bridge::BridgedMessage;
use rmw_cerulion::type_bridge_cpp::CppBridgedMessage;
use rmw_cerulion::*;

const ROS_TYPE_UINT8: u8 = 8;
const ROS_TYPE_MESSAGE: u8 = 18;
const DEVICE: &CStr = c"fixture_device";

extern "C" {
    fn rmw_cerulion_buffer_u8_sizeof() -> usize;
    fn rmw_cerulion_fixture_buffer_u8_construct(
        at: *mut c_void,
        data: *const u8,
        len: usize,
        backend: *const c_char,
        throw_on_copy: c_int,
    );
    fn rmw_cerulion_fixture_buffer_u8_destruct(b: *mut c_void);
    fn rmw_cerulion_fixture_buffer_u8_new(
        data: *const u8,
        len: usize,
        backend: *const c_char,
        throw_on_copy: c_int,
    ) -> *mut c_void;
    fn rmw_cerulion_fixture_buffer_u8_delete(b: *mut c_void);
    fn rmw_cerulion_fixture_buffer_u8_new_misreporting(
        data: *const u8,
        len: usize,
        reported_size: usize,
    ) -> *mut c_void;
    fn malloc(size: usize) -> *mut c_void;
    fn realloc(ptr: *mut c_void, size: usize) -> *mut c_void;
    fn calloc(n: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
    fn rmw_cerulion_fixture_buffer_u8_backend(b: *const c_void, out: *mut c_char, cap: usize);
    fn rmw_cerulion_fixture_buffer_u8_size_function(m: *const c_void) -> usize;
    fn rmw_cerulion_fixture_buffer_u8_get_const_function(
        m: *const c_void,
        i: usize,
    ) -> *const c_void;
    fn rmw_cerulion_fixture_buffer_u8_get_function(m: *mut c_void, i: usize) -> *mut c_void;
    fn rmw_cerulion_fixture_buffer_u8_resize_function(m: *mut c_void, size: usize);
}

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn pattern(n: usize, seed: u8) -> Vec<u8> {
    (0..n)
        .map(|i| (i as u8).wrapping_mul(31).wrapping_add(seed))
        .collect()
}

/// The backend a live Buffer reports.
unsafe fn backend_of(b: *const c_void) -> String {
    let mut out = [0 as c_char; 64];
    rmw_cerulion_fixture_buffer_u8_backend(b, out.as_mut_ptr(), out.len());
    CStr::from_ptr(out.as_ptr()).to_string_lossy().into_owned()
}

// =====================================================================
// C++ messages: the `uint8[]` member IS a rosidl::Buffer<uint8_t>
// =====================================================================

/// Pointer-aligned storage for one `rosidl::Buffer<uint8_t>` (16 bytes).
#[repr(C, align(8))]
struct BufferSlot([u8; 16]);

impl BufferSlot {
    /// A Buffer holding `bytes`: CPU-backed for `None`, else on the
    /// fixture device backend, whose host copy throws when `throws`.
    fn new(bytes: &[u8], backend: Option<&CStr>, throws: bool) -> Box<Self> {
        assert_eq!(unsafe { rmw_cerulion_buffer_u8_sizeof() }, 16);
        let mut slot = Box::new(BufferSlot([0; 16]));
        unsafe {
            rmw_cerulion_fixture_buffer_u8_construct(
                slot.as_mut_ptr(),
                bytes.as_ptr(),
                bytes.len(),
                backend.map_or(std::ptr::null(), |b| b.as_ptr()),
                throws as c_int,
            );
        }
        slot
    }
    fn as_mut_ptr(&mut self) -> *mut c_void {
        self.0.as_mut_ptr() as *mut c_void
    }
    fn as_ptr(&self) -> *const c_void {
        self.0.as_ptr() as *const c_void
    }
    fn backend(&self) -> String {
        unsafe { backend_of(self.as_ptr()) }
    }
    /// The bytes, read through the generated accessors (CPU only).
    fn cpu_bytes(&self) -> Vec<u8> {
        unsafe {
            let n = rmw_cerulion_fixture_buffer_u8_size_function(self.as_ptr());
            if n == 0 {
                return Vec::new();
            }
            let p =
                rmw_cerulion_fixture_buffer_u8_get_const_function(self.as_ptr(), 0) as *const u8;
            std::slice::from_raw_parts(p, n).to_vec()
        }
    }
}

impl Drop for BufferSlot {
    fn drop(&mut self) {
        unsafe { rmw_cerulion_fixture_buffer_u8_destruct(self.as_mut_ptr()) }
    }
}

fn buffer_member_cpp(name: &str, offset: u32) -> CppMessageMember {
    CppMessageMember {
        name_: cstr(name),
        type_id_: ROS_TYPE_UINT8,
        string_upper_bound_: 0,
        members_: std::ptr::null(),
        is_key_: false,
        is_array_: true,
        array_size_: 0,
        is_upper_bound_: false,
        offset_: offset,
        default_value_: std::ptr::null(),
        size_function: Some(rmw_cerulion_fixture_buffer_u8_size_function),
        get_const_function: Some(rmw_cerulion_fixture_buffer_u8_get_const_function),
        get_function: Some(rmw_cerulion_fixture_buffer_u8_get_function),
        fetch_function: None,
        assign_function: None,
        resize_function: Some(rmw_cerulion_fixture_buffer_u8_resize_function),
        is_rosidl_buffer_: true,
    }
}

fn cpp_members(
    name: &str,
    size_of: usize,
    members: Vec<CppMessageMember>,
) -> *const CppMessageMembers {
    let members = Box::leak(members.into_boxed_slice());
    Box::leak(Box::new(CppMessageMembers {
        message_namespace_: cstr("test_msgs::msg"),
        message_name_: cstr(name),
        member_count_: members.len() as u32,
        size_of_: size_of,
        has_any_key_member_: false,
        members_: members.as_ptr(),
        init_function: None,
        fini_function: None,
    }))
}

/// `uint8[] data`, the `sensor_msgs/Image.data` shape.
fn cpp_image_bridge() -> CppBridgedMessage {
    let members = cpp_members(
        "BufImage",
        std::mem::size_of::<BufferSlot>(),
        vec![buffer_member_cpp("data", 0)],
    );
    unsafe { CppBridgedMessage::new(members) }.expect("C++ Buffer bridge")
}

fn cpp_flatten(bridge: &CppBridgedMessage, msg: &BufferSlot) -> Result<Vec<u8>, String> {
    unsafe { bridge.flatten(msg.as_ptr(), 0, 0) }
        .map(|f| f[WireHeader::SIZE..].to_vec())
        .map_err(|e| e.to_string())
}

#[test]
#[serial]
fn a_cpp_device_buffer_publishes_the_bytes_a_cpu_buffer_would_and_is_counted_once() {
    let bridge = cpp_image_bridge();
    let bytes = pattern(4096, 7);
    let cpu = BufferSlot::new(&bytes, None, false);
    let device = BufferSlot::new(&bytes, Some(DEVICE), false);
    assert_eq!(device.backend(), "fixture_device");

    let before = downgrade_count();
    let cpu_payload = cpp_flatten(&bridge, &cpu).expect("CPU Buffer publishes");
    assert_eq!(downgrade_count(), before, "a CPU Buffer is read in place");
    let device_payload = cpp_flatten(&bridge, &device).expect("device Buffer is downgraded");
    assert_eq!(
        downgrade_count(),
        before + 1,
        "one downgrade per Buffer read"
    );
    assert_eq!(
        device_payload, cpu_payload,
        "the host copy is byte-identical"
    );
    assert_eq!(
        device.backend(),
        "fixture_device",
        "publish leaves the Buffer as it was"
    );

    // Every byte made it: the payload decodes back into a CPU Buffer.
    let mut out = BufferSlot::new(&[], None, false);
    assert!(unsafe { bridge.unflatten(&device_payload, out.as_mut_ptr()) });
    assert_eq!(out.backend(), "cpu");
    assert_eq!(out.cpu_bytes(), bytes);
}

#[test]
#[serial]
fn a_cpp_take_into_a_device_buffer_leaves_a_cpu_buffer_holding_the_frame() {
    let bridge = cpp_image_bridge();
    let bytes = pattern(300, 3);
    let source = BufferSlot::new(&bytes, None, false);
    let payload = cpp_flatten(&bridge, &source).expect("publish");

    // The caller's message holds a device Buffer whose generated resize
    // and get accessors throw: the take replaces its implementation.
    let mut dest = BufferSlot::new(&pattern(17, 99), Some(DEVICE), false);
    assert!(unsafe { bridge.unflatten(&payload, dest.as_mut_ptr()) });
    assert_eq!(dest.backend(), "cpu");
    assert_eq!(dest.cpu_bytes(), bytes);

    // An empty frame member empties a CPU Buffer too.
    let empty = cpp_flatten(&bridge, &BufferSlot::new(&[], None, false)).expect("publish empty");
    assert!(unsafe { bridge.unflatten(&empty, dest.as_mut_ptr()) });
    assert_eq!(dest.cpu_bytes(), Vec::<u8>::new());
}

#[test]
#[serial]
fn a_cpp_backend_that_throws_on_the_host_copy_refuses_the_frame_and_counts_nothing() {
    let bridge = cpp_image_bridge();
    let failing = BufferSlot::new(&pattern(64, 1), Some(DEVICE), true);
    let before = downgrade_count();
    let err = cpp_flatten(&bridge, &failing).expect_err("a failed host copy refuses");
    assert!(err.contains("backend threw"), "{err}");
    assert_eq!(downgrade_count(), before);
}

/// A backend whose `size()` disagrees with its host copy refuses the frame
/// rather than shipping a frame planned for another length, both ways.
#[test]
#[serial]
fn a_cpp_buffer_whose_size_disagrees_with_its_copy_refuses_the_frame() {
    let bridge = cpp_image_bridge();
    let bytes = pattern(40, 4);
    for reported in [bytes.len() - 1, bytes.len() + 1] {
        // The message is the Buffer itself: the one member sits at offset 0.
        let msg = unsafe {
            rmw_cerulion_fixture_buffer_u8_new_misreporting(bytes.as_ptr(), bytes.len(), reported)
        };
        let result = unsafe { bridge.flatten(msg, 0, 0) };
        assert!(
            result.is_err(),
            "size {reported} against {} copied bytes",
            bytes.len()
        );
        unsafe { rmw_cerulion_fixture_buffer_u8_delete(msg) };
    }
}

/// `Outer { Inner inner; uint8[] tail }` with `Inner { uint8[] data }`: a
/// Buffer inside a nested variable message goes through the nested codec.
#[repr(C)]
struct CppOuter {
    inner: BufferSlot,
    tail: BufferSlot,
}

#[test]
#[serial]
fn a_cpp_device_buffer_inside_a_nested_message_is_downgraded_too() {
    let inner = cpp_members(
        "BufInner",
        std::mem::size_of::<BufferSlot>(),
        vec![buffer_member_cpp("data", 0)],
    );
    let inner_ts: *const rosidl_message_type_support_t =
        Box::leak(Box::new(rosidl_message_type_support_t {
            typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
            data: inner as *const c_void,
            ..Default::default()
        }));
    let mut inner_member = buffer_member_cpp("inner", 0);
    inner_member.type_id_ = ROS_TYPE_MESSAGE;
    inner_member.is_array_ = false;
    inner_member.is_rosidl_buffer_ = false;
    inner_member.members_ = inner_ts;
    inner_member.size_function = None;
    inner_member.get_const_function = None;
    inner_member.get_function = None;
    inner_member.resize_function = None;
    let outer = cpp_members(
        "BufOuter",
        std::mem::size_of::<CppOuter>(),
        vec![inner_member, buffer_member_cpp("tail", 16)],
    );
    let bridge = unsafe { CppBridgedMessage::new(outer) }.expect("nested Buffer bridge");

    let a = pattern(200, 11);
    let b = pattern(33, 5);
    let build = |backend: Option<&CStr>| {
        let inner = BufferSlot::new(&a, backend, false);
        let tail = BufferSlot::new(&b, backend, false);
        // Move the two constructed Buffers into one struct without running
        // their destructors twice.
        let msg = CppOuter {
            inner: BufferSlot(inner.0),
            tail: BufferSlot(tail.0),
        };
        std::mem::forget(inner);
        std::mem::forget(tail);
        Box::new(msg)
    };
    let cpu = build(None);
    let device = build(Some(DEVICE));
    let flatten = |m: &CppOuter| {
        unsafe { bridge.flatten(m as *const CppOuter as *const c_void, 0, 0) }
            .map(|f| f[WireHeader::SIZE..].to_vec())
    };
    let before = downgrade_count();
    let cpu_payload = flatten(&cpu).expect("CPU publish");
    assert_eq!(downgrade_count(), before);
    let device_payload = flatten(&device).expect("device publish");
    // Each Buffer is copied once, by the write pass: the size pass measures
    // the nested message without copying its Buffer.
    assert_eq!(
        downgrade_count(),
        before + 2,
        "one host copy per Buffer per publish"
    );
    assert_eq!(device_payload, cpu_payload);

    let mut out = build(Some(DEVICE));
    assert!(unsafe {
        bridge.unflatten(&device_payload, &mut *out as *mut CppOuter as *mut c_void)
    });
    assert_eq!(out.inner.backend(), "cpu");
    assert_eq!(out.inner.cpu_bytes(), a);
    assert_eq!(out.tail.backend(), "cpu");
    assert_eq!(out.tail.cpu_bytes(), b);
}

// =====================================================================
// C messages: a flagged `uint8[]` sequence points at a Buffer
// =====================================================================

#[repr(C)]
struct CU8Seq {
    data: *mut u8,
    size: usize,
    capacity: usize,
    is_rosidl_buffer: bool,
    owns_rosidl_buffer: bool,
}

#[repr(C)]
struct CU8SeqMsg {
    data: CU8Seq,
}

impl CU8Seq {
    fn empty() -> Self {
        CU8Seq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            is_rosidl_buffer: false,
            owns_rosidl_buffer: false,
        }
    }
    /// A Buffer-backed sequence, the shape rosidl_typesupport_fastrtps
    /// writes when it deserializes one (size from the Buffer, no capacity).
    fn buffer(bytes: &[u8], backend: Option<&CStr>, throws: bool, owns: bool) -> Self {
        let data = unsafe {
            rmw_cerulion_fixture_buffer_u8_new(
                bytes.as_ptr(),
                bytes.len(),
                backend.map_or(std::ptr::null(), |b| b.as_ptr()),
                throws as c_int,
            )
        };
        CU8Seq {
            data: data as *mut u8,
            size: bytes.len(),
            capacity: 0,
            is_rosidl_buffer: true,
            owns_rosidl_buffer: owns,
        }
    }
    fn plain(bytes: &[u8]) -> Self {
        let mut boxed = bytes.to_vec().into_boxed_slice();
        let data = boxed.as_mut_ptr();
        std::mem::forget(boxed); // fixture: process lifetime
        CU8Seq {
            data,
            size: bytes.len(),
            capacity: bytes.len(),
            is_rosidl_buffer: false,
            owns_rosidl_buffer: false,
        }
    }
    unsafe fn plain_bytes(&self) -> Vec<u8> {
        assert!(!self.is_rosidl_buffer);
        if self.size == 0 {
            return Vec::new();
        }
        std::slice::from_raw_parts(self.data, self.size).to_vec()
    }
}

fn c_image_members() -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let members = Box::leak(
        vec![rosidl_typesupport_introspection_c__MessageMember {
            name_: cstr("data"),
            type_id_: ROS_TYPE_UINT8,
            is_array_: true,
            is_rosidl_buffer_: true,
            ..Default::default()
        }]
        .into_boxed_slice(),
    );
    Box::leak(Box::new(
        rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("test_msgs__msg"),
            message_name_: cstr("BufImage"),
            member_count_: 1,
            size_of_: std::mem::size_of::<CU8SeqMsg>(),
            members_: members.as_ptr(),
            ..Default::default()
        },
    ))
}

fn c_flatten(bridge: &BridgedMessage, msg: &CU8SeqMsg) -> Result<Vec<u8>, String> {
    unsafe { bridge.flatten(msg as *const CU8SeqMsg as *const c_void, 0, 0) }
        .map(|f| f[WireHeader::SIZE..].to_vec())
        .map_err(|e| e.to_string())
}

#[test]
#[serial]
fn a_c_device_buffer_sequence_publishes_the_bytes_a_plain_sequence_would() {
    let bridge = unsafe { BridgedMessage::new(c_image_members()) }.expect("C bridge");
    let bytes = pattern(5000, 9);
    let plain = CU8SeqMsg {
        data: CU8Seq::plain(&bytes),
    };
    let cpu = CU8SeqMsg {
        data: CU8Seq::buffer(&bytes, None, false, true),
    };
    let device = CU8SeqMsg {
        data: CU8Seq::buffer(&bytes, Some(DEVICE), false, true),
    };
    let before = downgrade_count();
    let plain_payload = c_flatten(&bridge, &plain).expect("plain");
    let cpu_payload = c_flatten(&bridge, &cpu).expect("a CPU Buffer is read in place");
    assert_eq!(downgrade_count(), before);
    let device_payload = c_flatten(&bridge, &device).expect("a device Buffer is downgraded");
    assert_eq!(downgrade_count(), before + 1);
    assert_eq!(cpu_payload, plain_payload);
    assert_eq!(device_payload, plain_payload);

    let mut out = CU8SeqMsg {
        data: CU8Seq::empty(),
    };
    assert!(unsafe { bridge.unflatten(&device_payload, &mut out as *mut _ as *mut c_void) });
    assert_eq!(unsafe { out.data.plain_bytes() }, bytes);
    unsafe {
        rmw_cerulion_fixture_buffer_u8_delete(cpu.data.data as *mut c_void);
        rmw_cerulion_fixture_buffer_u8_delete(device.data.data as *mut c_void);
    }
}

#[test]
#[serial]
fn a_c_take_into_an_owned_buffer_builds_a_cpu_buffer_in_place() {
    let bridge = unsafe { BridgedMessage::new(c_image_members()) }.expect("C bridge");
    let bytes = pattern(777, 2);
    let payload = c_flatten(
        &bridge,
        &CU8SeqMsg {
            data: CU8Seq::plain(&bytes),
        },
    )
    .expect("publish");

    let mut dest = CU8SeqMsg {
        data: CU8Seq::buffer(&pattern(12, 40), Some(DEVICE), false, true),
    };
    let buffer = dest.data.data as *const c_void;
    assert!(unsafe { bridge.unflatten(&payload, &mut dest as *mut _ as *mut c_void) });
    assert_eq!(
        dest.data.data as *const c_void, buffer,
        "the owned Buffer is reused"
    );
    assert!(dest.data.is_rosidl_buffer && dest.data.owns_rosidl_buffer);
    assert_eq!(dest.data.size, bytes.len());
    assert_eq!(unsafe { backend_of(buffer) }, "cpu");
    // Its bytes, read back through the bridge's own publish path.
    let again = c_flatten(&bridge, &dest).expect("republish");
    assert_eq!(again, payload);
    unsafe { rmw_cerulion_fixture_buffer_u8_delete(dest.data.data as *mut c_void) };
}

#[test]
#[serial]
fn a_c_take_into_a_borrowed_buffer_detaches_and_never_touches_it() {
    let bridge = unsafe { BridgedMessage::new(c_image_members()) }.expect("C bridge");
    let bytes = pattern(64, 8);
    let payload = c_flatten(
        &bridge,
        &CU8SeqMsg {
            data: CU8Seq::plain(&bytes),
        },
    )
    .expect("publish");

    let original = pattern(5, 1);
    let mut dest = CU8SeqMsg {
        data: CU8Seq::buffer(&original, Some(DEVICE), false, false),
    };
    let borrowed = dest.data.data as *mut c_void;
    assert!(unsafe { bridge.unflatten(&payload, &mut dest as *mut _ as *mut c_void) });
    assert!(
        !dest.data.is_rosidl_buffer,
        "the sequence no longer aliases the Buffer"
    );
    assert_ne!(dest.data.data as *mut c_void, borrowed);
    assert_eq!(unsafe { dest.data.plain_bytes() }, bytes);
    // The Buffer the sequence did not own is exactly as it was.
    assert_eq!(unsafe { backend_of(borrowed) }, "fixture_device");
    let check = CU8SeqMsg {
        data: CU8Seq {
            data: borrowed as *mut u8,
            size: original.len(),
            capacity: 0,
            is_rosidl_buffer: true,
            owns_rosidl_buffer: false,
        },
    };
    let original_payload = c_flatten(
        &bridge,
        &CU8SeqMsg {
            data: CU8Seq::plain(&original),
        },
    )
    .expect("publish");
    assert_eq!(
        c_flatten(&bridge, &check).expect("read the borrowed Buffer"),
        original_payload
    );
    unsafe { rmw_cerulion_fixture_buffer_u8_delete(borrowed) };
}

#[test]
#[serial]
fn a_c_backend_that_throws_or_a_null_buffer_refuses_the_frame() {
    let bridge = unsafe { BridgedMessage::new(c_image_members()) }.expect("C bridge");
    let before = downgrade_count();
    let failing = CU8SeqMsg {
        data: CU8Seq::buffer(&pattern(10, 0), Some(DEVICE), true, true),
    };
    assert!(c_flatten(&bridge, &failing).is_err());
    let mut null = CU8Seq::empty();
    null.is_rosidl_buffer = true;
    null.size = 3;
    assert!(c_flatten(&bridge, &CU8SeqMsg { data: null }).is_err());
    assert_eq!(downgrade_count(), before);
    unsafe { rmw_cerulion_fixture_buffer_u8_delete(failing.data.data as *mut c_void) };
}

#[test]
#[serial]
fn a_c_buffer_whose_size_disagrees_with_its_copy_refuses_the_frame() {
    let bridge = unsafe { BridgedMessage::new(c_image_members()) }.expect("C bridge");
    let bytes = pattern(40, 4);
    for reported in [bytes.len() - 1, bytes.len() + 1] {
        let buffer = unsafe {
            rmw_cerulion_fixture_buffer_u8_new_misreporting(bytes.as_ptr(), bytes.len(), reported)
        };
        let msg = CU8SeqMsg {
            data: CU8Seq {
                data: buffer as *mut u8,
                size: reported,
                capacity: 0,
                is_rosidl_buffer: true,
                owns_rosidl_buffer: true,
            },
        };
        assert!(c_flatten(&bridge, &msg).is_err(), "size {reported}");
        unsafe { rmw_cerulion_fixture_buffer_u8_delete(buffer) };
    }
}

/// `COuter { CInner inner; uint8[] tail }`, `CInner { uint8[] data }`.
#[repr(C)]
struct COuter {
    inner: CU8SeqMsg,
    tail: CU8Seq,
}

/// A Buffer inside a nested C message is copied once per publish (the size
/// pass measures it), publishes the CPU bytes, and a take refills an owned
/// nested Buffer in place and detaches from a borrowed top-level one.
#[test]
#[serial]
fn a_c_device_buffer_inside_a_nested_message_is_downgraded_and_refilled() {
    let inner_ts: *const rosidl_message_type_support_t =
        Box::leak(Box::new(rosidl_message_type_support_t {
            typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
            data: c_image_members() as *const c_void,
            ..Default::default()
        }));
    let members = Box::leak(
        vec![
            rosidl_typesupport_introspection_c__MessageMember {
                name_: cstr("inner"),
                type_id_: ROS_TYPE_MESSAGE,
                members_: inner_ts,
                ..Default::default()
            },
            rosidl_typesupport_introspection_c__MessageMember {
                name_: cstr("tail"),
                type_id_: ROS_TYPE_UINT8,
                is_array_: true,
                is_rosidl_buffer_: true,
                offset_: std::mem::offset_of!(COuter, tail) as u32,
                ..Default::default()
            },
        ]
        .into_boxed_slice(),
    );
    let outer = Box::leak(Box::new(
        rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("test_msgs__msg"),
            message_name_: cstr("BufOuter"),
            member_count_: 2,
            size_of_: std::mem::size_of::<COuter>(),
            members_: members.as_ptr(),
            ..Default::default()
        },
    ));
    let bridge = unsafe { BridgedMessage::new(outer) }.expect("nested C bridge");
    let a = pattern(300, 6);
    let b = pattern(17, 2);
    let build = |backend: Option<&CStr>, owns: bool| COuter {
        inner: CU8SeqMsg {
            data: CU8Seq::buffer(&a, backend, false, true),
        },
        tail: CU8Seq::buffer(&b, backend, false, owns),
    };
    let flatten = |m: &COuter| {
        unsafe { bridge.flatten(m as *const COuter as *const c_void, 0, 0) }
            .map(|f| f[WireHeader::SIZE..].to_vec())
    };
    let cpu = build(None, true);
    let device = build(Some(DEVICE), true);
    let before = downgrade_count();
    let cpu_payload = flatten(&cpu).expect("CPU publish");
    assert_eq!(downgrade_count(), before);
    let device_payload = flatten(&device).expect("device publish");
    assert_eq!(
        downgrade_count(),
        before + 2,
        "one host copy per Buffer per publish"
    );
    assert_eq!(device_payload, cpu_payload);

    let mut out = build(Some(DEVICE), false);
    let inner_buffer = out.inner.data.data as *const c_void;
    let borrowed_tail = out.tail.data as *mut c_void;
    assert!(unsafe { bridge.unflatten(&device_payload, &mut out as *mut COuter as *mut c_void) });
    assert_eq!(
        out.inner.data.data as *const c_void, inner_buffer,
        "owned: refilled in place"
    );
    assert_eq!(unsafe { backend_of(inner_buffer) }, "cpu");
    assert!(!out.tail.is_rosidl_buffer, "borrowed: detached");
    assert_eq!(unsafe { out.tail.plain_bytes() }, b);
    assert_eq!(unsafe { backend_of(borrowed_tail) }, "fixture_device");
    assert_eq!(flatten(&out).expect("republish"), cpu_payload);
    unsafe {
        for m in [&cpu, &device] {
            rmw_cerulion_fixture_buffer_u8_delete(m.inner.data.data as *mut c_void);
            rmw_cerulion_fixture_buffer_u8_delete(m.tail.data as *mut c_void);
        }
        rmw_cerulion_fixture_buffer_u8_delete(out.inner.data.data as *mut c_void);
        rmw_cerulion_fixture_buffer_u8_delete(borrowed_tail);
    }
}

// =====================================================================
// Through the rmw C ABI
// =====================================================================

fn unique_suffix() -> u128 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock")
        .as_nanos()
}

unsafe fn setup_node(name: &str) -> (*mut ffi::rmw_node_t, Box<ffi::rmw_init_options_t>) {
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    let node = rmw_create_node(context, cstr(name), cstr("/"));
    assert!(!node.is_null(), "node creation failed");
    (node, options)
}

fn qos() -> ffi::rmw_qos_profile_t {
    ffi::rmw_qos_profile_t {
        history: ffi::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth: 8,
        reliability: ffi::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        durability: ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE,
        deadline: ffi::rmw_time_t { sec: 0, nsec: 0 },
        lifespan: ffi::rmw_time_t { sec: 0, nsec: 0 },
        liveliness: ffi::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
        liveliness_lease_duration: ffi::rmw_time_t { sec: 0, nsec: 0 },
        avoid_ros_namespace_conventions: false,
    }
}

fn c_image_ts() -> *const rosidl_message_type_support_t {
    Box::leak(Box::new(rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: c_image_members() as *const c_void,
        ..Default::default()
    }))
}

#[test]
#[serial]
fn acceptable_buffer_backends_names_a_backend_this_rmw_lacks_and_still_gets_cpu_data() {
    unsafe {
        let suffix = unique_suffix();
        let (node, _opts) = setup_node(&format!("bufopts_{suffix}"));
        let ts = c_image_ts();
        let topic = CString::new(format!("/rosidl_buffer/opts/{suffix}")).expect("topic");
        let q = qos();
        let accepted: [Option<&CStr>; 7] = [
            None,
            Some(c""),
            Some(c"cpu"),
            Some(c" cpu ,any"),
            Some(c"any"),
            Some(c"cuda"),
            Some(c"cpu,cuda"),
        ];
        for requested in accepted {
            let mut opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
            opts.acceptable_buffer_backends = requested.map_or(std::ptr::null(), |r| r.as_ptr());
            let sub = rmw_create_subscription(node, ts, topic.as_ptr(), &q, &opts);
            assert!(!sub.is_null(), "{requested:?} is a well-formed list");
            assert_eq!(rmw_destroy_subscription(node, sub), RMW_RET_OK);
        }

        // A subscription that asks for cuda receives a device frame as CPU data.
        let mut opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        opts.acceptable_buffer_backends = c"cuda".as_ptr();
        let sub = rmw_create_subscription(node, ts, topic.as_ptr(), &q, &opts);
        assert!(!sub.is_null());
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &q, &pub_opts);
        assert!(!publisher.is_null());
        let bytes = pattern(512, 5);
        let msg = CU8SeqMsg {
            data: CU8Seq::buffer(&bytes, Some(DEVICE), false, true),
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        rmw_cerulion_fixture_buffer_u8_delete(msg.data.data as *mut c_void);
        let mut out = CU8SeqMsg {
            data: CU8Seq::empty(),
        };
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        loop {
            let mut taken = false;
            assert_eq!(
                rmw_take(
                    sub,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            if taken {
                break;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "the frame never arrived"
            );
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        assert_eq!(out.data.plain_bytes(), bytes);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, sub), RMW_RET_OK);

        // Only malformed input fails the create, with the reason on rcl's
        // error channel.
        let mut opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        opts.acceptable_buffer_backends = c"cpu;cuda".as_ptr();
        let sub = rmw_create_subscription(node, ts, topic.as_ptr(), &q, &opts);
        assert!(sub.is_null(), "a malformed list fails the create");
        let text = ffi::last_rcl_error_text().expect("the reason reaches rcl's error channel");
        assert!(
            text.contains("acceptable_buffer_backends names 'cpu;cuda'"),
            "{text}"
        );
        // The refused create left nothing behind: the same topic still takes
        // a CPU subscription, and it is the topic's only one.
        let mut opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        opts.acceptable_buffer_backends = c"cpu".as_ptr();
        let sub = rmw_create_subscription(node, ts, topic.as_ptr(), &q, &opts);
        assert!(!sub.is_null());
        let mut count = usize::MAX;
        assert_eq!(
            rmw_count_subscribers(node, topic.as_ptr(), &mut count),
            RMW_RET_OK
        );
        assert_eq!(count, 1);
        assert_eq!(rmw_destroy_subscription(node, sub), RMW_RET_OK);
    }
}

#[test]
#[serial]
fn a_device_buffer_published_through_rmw_arrives_intact_and_is_counted_on_its_topic() {
    unsafe {
        let suffix = unique_suffix();
        let (node, _opts) = setup_node(&format!("bufpub_{suffix}"));
        let ts = c_image_ts();
        let ros_topic = format!("/rosidl_buffer/e2e/{suffix}");
        let topic = CString::new(ros_topic.clone()).expect("topic");
        let q = qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &q, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &q, &pub_opts);
        assert!(!publisher.is_null());

        let frames: Vec<Vec<u8>> = (0..3).map(|i| pattern(2048 + i, i as u8)).collect();
        let before = downgrade_count();
        for bytes in &frames {
            let msg = CU8SeqMsg {
                data: CU8Seq::buffer(bytes, Some(DEVICE), false, true),
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            rmw_cerulion_fixture_buffer_u8_delete(msg.data.data as *mut c_void);
        }
        assert_eq!(downgrade_count(), before + 3);
        // The per-topic count is keyed on the Cerulion name, which is the
        // ROS name verbatim.
        assert_eq!(topic_downgrade_count(&ros_topic), 3);

        let take = |out: &mut CU8SeqMsg| {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
            loop {
                let mut taken = false;
                assert_eq!(
                    rmw_take(
                        subscription,
                        out as *mut CU8SeqMsg as *mut c_void,
                        &mut taken,
                        std::ptr::null_mut()
                    ),
                    RMW_RET_OK
                );
                if taken {
                    return;
                }
                assert!(
                    std::time::Instant::now() < deadline,
                    "a published frame never arrived"
                );
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
        };
        for (i, bytes) in frames.iter().enumerate() {
            if i == 0 {
                // The take leaves an owned device Buffer on the CPU backend.
                let mut out = CU8SeqMsg {
                    data: CU8Seq::buffer(&pattern(9, 9), Some(DEVICE), false, true),
                };
                take(&mut out);
                assert_eq!(backend_of(out.data.data as *const c_void), "cpu");
                let republished = c_flatten(
                    &BridgedMessage::new(c_image_members()).expect("bridge"),
                    &out,
                )
                .expect("read back");
                let expected = c_flatten(
                    &BridgedMessage::new(c_image_members()).expect("bridge"),
                    &CU8SeqMsg {
                        data: CU8Seq::plain(bytes),
                    },
                )
                .expect("oracle");
                assert_eq!(republished, expected, "matching bytes");
                rmw_cerulion_fixture_buffer_u8_delete(out.data.data as *mut c_void);
            } else {
                let mut out = CU8SeqMsg {
                    data: CU8Seq::empty(),
                };
                take(&mut out);
                assert_eq!(&out.data.plain_bytes(), bytes, "matching bytes");
            }
        }
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
    }
}

unsafe extern "C" fn ser_allocate(size: usize, _state: *mut c_void) -> *mut c_void {
    malloc(size)
}
unsafe extern "C" fn ser_deallocate(p: *mut c_void, _state: *mut c_void) {
    free(p)
}
unsafe extern "C" fn ser_reallocate(
    p: *mut c_void,
    size: usize,
    _state: *mut c_void,
) -> *mut c_void {
    realloc(p, size)
}
unsafe extern "C" fn ser_zero_allocate(n: usize, size: usize, _state: *mut c_void) -> *mut c_void {
    calloc(n, size)
}

/// `rmw_serialize` downgrades a device Buffer like a publish does, writes the
/// bytes a CPU Buffer serializes to, and counts the frame under its own
/// `<rmw_serialize>` key.
#[test]
#[serial]
fn rmw_serialize_downgrades_a_device_buffer_and_counts_it() {
    unsafe {
        let ts = c_image_ts();
        let bytes = pattern(1000, 3);
        let serialize = |msg: &CU8SeqMsg| {
            let mut ser: ffi::rmw_serialized_message_t = std::mem::zeroed();
            ser.allocator = ffi::rcutils_allocator_t {
                allocate: Some(ser_allocate),
                deallocate: Some(ser_deallocate),
                reallocate: Some(ser_reallocate),
                zero_allocate: Some(ser_zero_allocate),
                state: std::ptr::null_mut(),
            };
            assert_eq!(
                rmw_serialize(msg as *const _ as *const c_void, ts, &mut ser),
                RMW_RET_OK
            );
            let out = std::slice::from_raw_parts(ser.buffer, ser.buffer_length).to_vec();
            free(ser.buffer as *mut c_void);
            out
        };
        let cpu = CU8SeqMsg {
            data: CU8Seq::buffer(&bytes, None, false, true),
        };
        let device = CU8SeqMsg {
            data: CU8Seq::buffer(&bytes, Some(DEVICE), false, true),
        };
        let before = topic_downgrade_count("<rmw_serialize>");
        let cpu_bytes = serialize(&cpu);
        assert_eq!(topic_downgrade_count("<rmw_serialize>"), before);
        let device_bytes = serialize(&device);
        assert_eq!(topic_downgrade_count("<rmw_serialize>"), before + 1);
        assert_eq!(
            device_bytes[WireHeader::SIZE..],
            cpu_bytes[WireHeader::SIZE..]
        );
        rmw_cerulion_fixture_buffer_u8_delete(cpu.data.data as *mut c_void);
        rmw_cerulion_fixture_buffer_u8_delete(device.data.data as *mut c_void);
    }
}
