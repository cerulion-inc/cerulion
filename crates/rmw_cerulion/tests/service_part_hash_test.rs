// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The `std_srvs` service parts through BOTH introspection bridges, against
//! the native generated parts. The hand-built arms need no ROS installation.
//!
//! A ROS 2 peer on `rmw_cerulion` announces a service part under the hash
//! the bridge derives from rosidl's introspection data; a native node
//! announces the generated part's `SCHEMA_HASH`. The two meet on the wire
//! only when those agree, so each of the six parts is built here exactly as
//! rosidl generates it (namespace `std_srvs__srv` / `std_srvs::srv`, member
//! names `<Name>_Request` and `<Name>_Response`, and the
//! `uint8 structure_needs_at_least_one_member` placeholder rosidl gives every
//! member-less struct, because IDL forbids an empty one) and hashed through
//! `BridgedMessage::new` / `CppBridgedMessage::new`, the calls
//! `rmw_create_service` and `rmw_create_client` make.
//!
//! The member tables are written by hand from rosidl's generated C and C++
//! structs, never derived from the native schemas, so drift on either side
//! lands here.
//!
//! The `#[ignore]`d arm at the bottom reads the SAME six parts from a real
//! distro's `std_srvs` introspection libraries (found through
//! `AMENT_PREFIX_PATH`); the rmw distro lanes run it with `--ignored`.

use std::ffi::CString;
use std::os::raw::{c_char, c_void};

use rmw_cerulion::ffi::introspection_cpp::{CppMessageMember, CppMessageMembers};
use rmw_cerulion::ffi::{
    rosidl_message_type_support_t, rosidl_typesupport_introspection_c__MessageMember,
    rosidl_typesupport_introspection_c__MessageMembers,
};
use rmw_cerulion::type_bridge::{BridgeError, BridgedMessage};
use rmw_cerulion::type_bridge_cpp::CppBridgedMessage;

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::std_srvs::{
    Empty_Request, Empty_Response, SetBool_Request, SetBool_Response, Trigger_Request,
    Trigger_Response,
};

// Introspection type ids.
const ROS_TYPE_BOOLEAN: u8 = 6;
const ROS_TYPE_UINT8: u8 = 8;
const ROS_TYPE_STRING: u8 = 16;
const ROS_TYPE_MESSAGE: u8 = 18;

/// The one member rosidl puts in a struct that declares none.
const PLACEHOLDER: &str = "structure_needs_at_least_one_member";

/// Leak a C string (test fixtures live for the process).
fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

// =====================================================================
// rosidl's generated C shapes
// =====================================================================

/// `rosidl_runtime_c__String`.
#[repr(C)]
struct CRosString {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

/// What rosidl generates for a member-less struct (`Empty_Request`,
/// `Empty_Response`, `Trigger_Request`): the placeholder byte and nothing else.
#[repr(C)]
struct CPlaceholderOnly {
    structure_needs_at_least_one_member: u8,
}

/// `std_srvs__srv__SetBool_Request`.
#[repr(C)]
struct CBool {
    data: bool,
}

/// `std_srvs__srv__SetBool_Response` and `std_srvs__srv__Trigger_Response`.
#[repr(C)]
struct CBoolString {
    success: bool,
    message: CRosString,
}

fn c_member(
    name: &str,
    type_id: u8,
    offset: usize,
    nested: *const rosidl_message_type_support_t,
) -> rosidl_typesupport_introspection_c__MessageMember {
    rosidl_typesupport_introspection_c__MessageMember {
        name_: cstr(name),
        type_id_: type_id,
        members_: nested,
        offset_: offset as u32,
        ..Default::default()
    }
}

fn c_members(
    namespace: &str,
    name: &str,
    size_of: usize,
    members: Vec<rosidl_typesupport_introspection_c__MessageMember>,
) -> *const rosidl_typesupport_introspection_c__MessageMembers {
    let members = Box::leak(members.into_boxed_slice());
    Box::leak(Box::new(
        rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr(namespace),
            message_name_: cstr(name),
            member_count_: members.len() as u32,
            size_of_: size_of,
            members_: members.as_ptr(),
            ..Default::default()
        },
    ))
}

fn c_typesupport(
    members: *const rosidl_typesupport_introspection_c__MessageMembers,
) -> *const rosidl_message_type_support_t {
    Box::leak(Box::new(rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: members as *const c_void,
        ..Default::default()
    }))
}

fn c_placeholder_only(name: &str) -> *const rosidl_typesupport_introspection_c__MessageMembers {
    c_members(
        "std_srvs__srv",
        name,
        std::mem::size_of::<CPlaceholderOnly>(),
        vec![c_member(PLACEHOLDER, ROS_TYPE_UINT8, 0, std::ptr::null())],
    )
}

fn c_bool(name: &str) -> *const rosidl_typesupport_introspection_c__MessageMembers {
    c_members(
        "std_srvs__srv",
        name,
        std::mem::size_of::<CBool>(),
        vec![c_member("data", ROS_TYPE_BOOLEAN, 0, std::ptr::null())],
    )
}

fn c_bool_string(name: &str) -> *const rosidl_typesupport_introspection_c__MessageMembers {
    c_members(
        "std_srvs__srv",
        name,
        std::mem::size_of::<CBoolString>(),
        vec![
            c_member("success", ROS_TYPE_BOOLEAN, 0, std::ptr::null()),
            c_member(
                "message",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CBoolString, message),
                std::ptr::null(),
            ),
        ],
    )
}

// =====================================================================
// rosidl's generated C++ shapes
// =====================================================================

/// Storage for one `std::string` (24 bytes on libc++, 32 on libstdc++; the
/// hash arms never construct one, the bridge only needs the offset).
#[repr(C, align(8))]
struct CppStringSlot([u8; 32]);

/// `std_srvs::srv::Empty_Request_` and friends: the placeholder byte only.
#[repr(C)]
struct CppPlaceholderOnly {
    structure_needs_at_least_one_member: u8,
}

/// `std_srvs::srv::SetBool_Request_`.
#[repr(C)]
struct CppBool {
    data: bool,
}

/// `std_srvs::srv::SetBool_Response_` and `Trigger_Response_`.
#[repr(C)]
struct CppBoolString {
    success: bool,
    message: CppStringSlot,
}

fn cpp_member(name: &str, type_id: u8, offset: usize) -> CppMessageMember {
    CppMessageMember {
        name_: cstr(name),
        type_id_: type_id,
        string_upper_bound_: 0,
        members_: std::ptr::null(),
        #[cfg(cerulion_has_is_key)]
        is_key_: false,
        is_array_: false,
        array_size_: 0,
        is_upper_bound_: false,
        offset_: offset as u32,
        default_value_: std::ptr::null(),
        size_function: None,
        get_const_function: None,
        get_function: None,
        #[cfg(cerulion_has_fetch_function)]
        fetch_function: None,
        #[cfg(cerulion_has_fetch_function)]
        assign_function: None,
        resize_function: None,
        #[cfg(cerulion_has_is_rosidl_buffer)]
        is_rosidl_buffer_: false,
    }
}

fn cpp_members(
    name: &str,
    size_of: usize,
    members: Vec<CppMessageMember>,
) -> *const CppMessageMembers {
    let members = Box::leak(members.into_boxed_slice());
    Box::leak(Box::new(CppMessageMembers {
        message_namespace_: cstr("std_srvs::srv"),
        message_name_: cstr(name),
        member_count_: members.len() as u32,
        size_of_: size_of,
        #[cfg(cerulion_has_is_key)]
        has_any_key_member_: false,
        members_: members.as_ptr(),
        init_function: None,
        fini_function: None,
    }))
}

fn cpp_placeholder_only(name: &str) -> *const CppMessageMembers {
    cpp_members(
        name,
        std::mem::size_of::<CppPlaceholderOnly>(),
        vec![cpp_member(PLACEHOLDER, ROS_TYPE_UINT8, 0)],
    )
}

fn cpp_bool(name: &str) -> *const CppMessageMembers {
    cpp_members(
        name,
        std::mem::size_of::<CppBool>(),
        vec![cpp_member("data", ROS_TYPE_BOOLEAN, 0)],
    )
}

fn cpp_bool_string(name: &str) -> *const CppMessageMembers {
    cpp_members(
        name,
        std::mem::size_of::<CppBoolString>(),
        vec![
            cpp_member("success", ROS_TYPE_BOOLEAN, 0),
            cpp_member(
                "message",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CppBoolString, message),
            ),
        ],
    )
}

// =====================================================================
// The six parts
// =====================================================================

/// Qualified part name and the native generated hash, in `.srv` order.
fn native_parts() -> [(&'static str, u64); 6] {
    [
        ("std_srvs/Empty_Request", Empty_Request::SCHEMA_HASH),
        ("std_srvs/Empty_Response", Empty_Response::SCHEMA_HASH),
        ("std_srvs/SetBool_Request", SetBool_Request::SCHEMA_HASH),
        ("std_srvs/SetBool_Response", SetBool_Response::SCHEMA_HASH),
        ("std_srvs/Trigger_Request", Trigger_Request::SCHEMA_HASH),
        ("std_srvs/Trigger_Response", Trigger_Response::SCHEMA_HASH),
    ]
}

/// The C introspection of each part, as rosidl generates it.
fn c_parts() -> [*const rosidl_typesupport_introspection_c__MessageMembers; 6] {
    [
        c_placeholder_only("Empty_Request"),
        c_placeholder_only("Empty_Response"),
        c_bool("SetBool_Request"),
        c_bool_string("SetBool_Response"),
        c_placeholder_only("Trigger_Request"),
        c_bool_string("Trigger_Response"),
    ]
}

/// The C++ introspection of each part, as rosidl generates it.
fn cpp_parts() -> [*const CppMessageMembers; 6] {
    [
        cpp_placeholder_only("Empty_Request"),
        cpp_placeholder_only("Empty_Response"),
        cpp_bool("SetBool_Request"),
        cpp_bool_string("SetBool_Response"),
        cpp_placeholder_only("Trigger_Request"),
        cpp_bool_string("Trigger_Response"),
    ]
}

/// One line per part whose bridged hash differs from the native one. The
/// bridged list is in `.srv` order, like [`native_parts`].
fn drift(bridged: Vec<(String, u64)>) -> Vec<String> {
    assert_eq!(bridged.len(), native_parts().len());
    bridged
        .into_iter()
        .zip(native_parts())
        .filter_map(|((bridged_name, bridged), (name, native))| {
            assert_eq!(
                bridged_name, name,
                "the bridge names the part as ROS 2 does"
            );
            (bridged != native)
                .then(|| format!("{name}: rmw_cerulion 0x{bridged:016x}, native 0x{native:016x}"))
        })
        .collect()
}

#[test]
fn each_std_srvs_part_bridged_from_c_introspection_hashes_like_the_native_part() {
    let bridged = c_parts()
        .into_iter()
        .map(|members| {
            let bridge = unsafe { BridgedMessage::new(members) }
                .unwrap_or_else(|e| panic!("the C bridge must accept rosidl's members: {e}"));
            (bridge.qualified_name.clone(), bridge.schema_hash())
        })
        .collect();
    let drifted = drift(bridged);
    assert!(
        drifted.is_empty(),
        "a ROS 2 peer (C introspection) and a native peer would not meet on:\n{}",
        drifted.join("\n")
    );
}

#[test]
fn each_std_srvs_part_bridged_from_cpp_introspection_hashes_like_the_native_part() {
    let bridged = cpp_parts()
        .into_iter()
        .map(|members| {
            let bridge = unsafe { CppBridgedMessage::new(members) }
                .unwrap_or_else(|e| panic!("the C++ bridge must accept rosidl's members: {e}"));
            (bridge.qualified_name.clone(), bridge.schema_hash())
        })
        .collect();
    let drifted = drift(bridged);
    assert!(
        drifted.is_empty(),
        "a ROS 2 peer (C++ introspection) and a native peer would not meet on:\n{}",
        drifted.join("\n")
    );
}

// =====================================================================
// The empty parts on the wire
// =====================================================================

/// A native `Trigger_Request` frame is a bare header. The bridged one must be
/// the same frame: the placeholder byte never reaches the wire, and a take
/// never writes it (rosidl zero-initializes it; the sentinel proves the
/// bridge leaves it alone rather than copying a payload byte over it).
#[test]
fn an_empty_part_bridged_from_c_is_a_header_only_frame() {
    let bridge =
        unsafe { BridgedMessage::new(c_placeholder_only("Trigger_Request")) }.expect("bridge");
    assert!(
        !bridge.can_loan,
        "the struct is one placeholder byte, the payload is none: a loan would publish the byte"
    );
    let msg = CPlaceholderOnly {
        structure_needs_at_least_one_member: 0,
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 3, 11) }.expect("flatten");
    assert_eq!(
        frame.len(),
        WireHeader::SIZE,
        "an empty part carries no payload"
    );
    let header = WireHeader::read_from_buf(&frame).expect("header");
    assert_eq!(header.schema_hash, Trigger_Request::SCHEMA_HASH);
    assert_eq!(header.sequence, 3);
    assert_eq!(header.timestamp_ns, 11);

    let mut out = CPlaceholderOnly {
        structure_needs_at_least_one_member: 0xA5,
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok, "a header-only frame is a complete empty part");
    assert_eq!(out.structure_needs_at_least_one_member, 0xA5);
}

#[test]
fn an_empty_part_bridged_from_cpp_is_a_header_only_frame() {
    let bridge =
        unsafe { CppBridgedMessage::new(cpp_placeholder_only("Empty_Response")) }.expect("bridge");
    assert!(!bridge.can_loan);
    let msg = CppPlaceholderOnly {
        structure_needs_at_least_one_member: 0,
    };
    let frame =
        unsafe { bridge.flatten(&msg as *const _ as *const c_void, 5, 13) }.expect("flatten");
    assert_eq!(
        frame.len(),
        WireHeader::SIZE,
        "an empty part carries no payload"
    );
    let header = WireHeader::read_from_buf(&frame).expect("header");
    assert_eq!(header.schema_hash, Empty_Response::SCHEMA_HASH);

    let mut out = CppPlaceholderOnly {
        structure_needs_at_least_one_member: 0xA5,
    };
    let ok = unsafe {
        bridge.unflatten(
            &frame[WireHeader::SIZE..],
            &mut out as *mut _ as *mut c_void,
        )
    };
    assert!(ok);
    assert_eq!(out.structure_needs_at_least_one_member, 0xA5);
}

// =====================================================================
// A member-less message nested in another message
// =====================================================================

/// `rmw_probe/Holder { std_msgs/Empty e }`, laid out as rosidl does: the
/// nested struct is its one placeholder byte, so the C field is one byte
/// where the native wire has none. The bridge does not re-pack that shape
/// yet, so it must refuse the type loudly at creation; it must never bridge
/// it under a hash a native `Holder` would not announce.
#[test]
fn a_message_that_nests_a_member_less_message_is_refused_not_mis_hashed() {
    let empty = c_typesupport(c_members(
        "std_msgs__msg",
        "Empty",
        std::mem::size_of::<CPlaceholderOnly>(),
        vec![c_member(PLACEHOLDER, ROS_TYPE_UINT8, 0, std::ptr::null())],
    ));
    let holder = c_members(
        "rmw_probe__msg",
        "Holder",
        std::mem::size_of::<CPlaceholderOnly>(),
        vec![c_member("e", ROS_TYPE_MESSAGE, 0, empty)],
    );
    match unsafe { BridgedMessage::new(holder) } {
        Err(BridgeError::LayoutMismatch { message, detail }) => {
            assert_eq!(message, "rmw_probe/Holder");
            assert!(detail.contains("'e'"), "the refusal names the field: {detail}");
        }
        Err(other) => panic!("expected a layout refusal, got {other}"),
        Ok(bridge) => panic!(
            "a nested member-less message was bridged under hash 0x{:016x}; a native peer would never meet it",
            bridge.schema_hash()
        ),
    }
}
