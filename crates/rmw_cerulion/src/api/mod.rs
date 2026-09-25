// SPDX-License-Identifier: AGPL-3.0-only
//! The exported rmw C ABI surface.
//!
//! Symbol inventory mirrors `rmw_iceoryx2` (rolling) — the set
//! `rmw_implementation`'s dispatcher resolves. Functions that cannot be
//! meaningfully implemented on a SHM transport return
//! `RMW_RET_UNSUPPORTED` exactly like other rmw implementations do
//! (network flow endpoints, content filters, dynamic typesupport).
//!
//! # Safety
//!
//! Every `extern "C"` function upholds the rmw contract: null-check all
//! pointers; verify the implementation identifier on entities whose
//! `data` payload is dereferenced (pure-default getters like
//! `*_get_actual_qos` only null-check); never unwind across the FFI
//! boundary. Substantive entry points — including the init.rs bootstrap
//! surface — wrap their bodies in `ffi::ffi_guard` (catch_unwind →
//! `RMW_RET_ERROR`/null + `tracing::error!`), so even an internal logic
//! panic degrades to a per-call error instead of aborting the host
//! process. Unwrapped entry points (allocate/free
//! helpers, UNSUPPORTED stubs, count/QoS getters) are panic-free by
//! inspection: no unwrap, poison-tolerant locks, and the only slices
//! (`rmw_get_gid_for_*`'s 16-byte copy) are bounded by the per-era
//! `RMW_GID_STORAGE_SIZE` pin (24 pre-Iron / 16 Iron+, both ≥ 16).
//!
//! # Caller-memory ORDER CONTRACT (the load-time era guard)
//!
//! The rmw structs a HOST hands us are laid out by the host's distro, and
//! this `.so` was compiled for exactly one. So the PRE-INIT exports —
//! `rmw_init_options_init` / `_copy` / `_fini` and `rmw_init`, the only
//! exports a host calls before `rmw_init` has passed — order their bodies
//! null check → `refuse_on_era_mismatch` (a verdict from the baked claim
//! and `ROS_DISTRO` ONLY) → first touch of caller memory. A refusal
//! therefore reads and writes ZERO caller bytes (pinned with a PROT_NONE
//! page in `tests/rmw_era_guard_test.rs`). The two pre-init getters
//! (`rmw_get_implementation_identifier`, `rmw_get_serialization_format`)
//! touch no caller memory.
//! Every POST-INIT export needs no era guard by design: its first
//! caller-memory touch is `implementation_identifier`, the FIRST field of
//! every handle struct (node/publisher/subscription/service/client/guard
//! condition/wait set/gid/event — offset 0 in every era) or offset 8 of
//! `rmw_context_t` (behind `instance_id`, inside the layout-invariant
//! prefix), and such a handle exists only because THIS library stamped it
//! after a passed `rmw_init`; a hand-forged handle is outside the rmw
//! contract.

mod guard_wait;
mod init;
mod misc;
mod node;
mod pubsub;
mod services;

pub use guard_wait::*;
pub use init::*;
pub use misc::*;
pub use node::*;
pub use pubsub::*;
pub use services::*;

use crate::era::CppBridgeGate;
use crate::ffi;

/// Which introspection language a typesupport handle resolved to.
///
/// rclpy (and rclc) hand rmw the C typesupport; rclcpp — and therefore
/// MoveIt 2 — hands the C++ one. Frames are byte-identical either way
/// (see `bridge::AnyBridge`).
pub(crate) enum IntrospectionHandle {
    C(*const ffi::rosidl_typesupport_introspection_c__MessageMembers),
    Cpp(*const ffi::introspection_cpp::CppMessageMembers),
}

/// Resolve a (possibly wrapped) rosidl MESSAGE typesupport handle to
/// introspection data.
///
/// rclcpp/rclpy hand rmw a language dispatcher handle; its `func`
/// callback resolves the inner handle for a requested identifier.
/// The one C++ typesupport gate both resolvers consult — a
/// refusing verdict's [`CppBridgeGate::emit_refusal`] logs the
/// per-verdict constant paragraph and tells the caller to refuse the
/// C++ arm. The vendored bypass is `cfg(test)` (silent) or
/// `test-seams` (LOUD, flood-latched — a warn on the FIRST resolve and
/// at each decade of the running total, `debug!` in between, the counter
/// unconditional; test-seams is a public feature a deployable build can
/// enable, so production silence is impossible). All `cfg!` values are compile-time constants.
fn cpp_typesupport_gate() -> CppBridgeGate {
    // Every input is a typed derivation from ONE place in
    // era.rs — no positional bools a swapped `cfg!` could compile into
    // the silent shape.
    let bypass =
        crate::era::classify_cpp_bypass(crate::era::bindings_source(), crate::era::test_surface());
    if bypass == crate::era::CppBypassMode::SeamsLoud {
        crate::era::emit_cpp_bypass_warn();
    }
    crate::era::cpp_bridge_gate_for(bypass)
}

/// We probe the dispatcher's OWN language first: asking a cpp
/// dispatcher for the C identifier fails inside rosidl's
/// type_support_dispatch, which SETS the process rcutils error state —
/// rcl then reports that stale string when anything later fails.
///
/// # Safety
/// `ts` must be null or a valid rosidl message typesupport handle.
pub(crate) unsafe fn resolve_introspection(
    ts: *const ffi::rosidl_message_type_support_t,
) -> Option<IntrospectionHandle> {
    for (attempt, ident) in identifier_probe_order(if ts.is_null() {
        std::ptr::null()
    } else {
        (*ts).typesupport_identifier
    })
    .into_iter()
    .enumerate()
    {
        if attempt > 0 {
            // The failed first probe went through rosidl's
            // type_support_dispatch, which SET the process rcutils
            // error state; clear it so a SUCCESSFUL fallback doesn't
            // leave a stale string for rcl to report on the next
            // unrelated failure (rmw_fastrtps resets between probes
            // the same way). On total failure the last probe's error
            // is deliberately left in place — it is the diagnostic.
            ffi::rcutils_reset_error_best_effort();
        }
        if let Some(handle) = resolve_for_identifier(ts, ident) {
            let data = (*handle).data;
            if data.is_null() {
                continue;
            }
            if ident == ffi::INTROSPECTION_C_IDENTIFIER {
                return Some(IntrospectionHandle::C(
                    data as *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
                ));
            }
            // C++ bridge gate, bounded on BOTH edges (the
            // Jazzy/Kilted hand-mirror exactly), with the
            // bypass composite — see cpp_typesupport_gate. The C
            // arm above stays fully functional on every era.
            if cpp_typesupport_gate().emit_refusal() {
                return None;
            }
            return Some(IntrospectionHandle::Cpp(
                data as *const ffi::introspection_cpp::CppMessageMembers,
            ));
        }
    }
    None
}

/// Probe order for introspection identifiers, keyed by the top-level
/// handle's own identifier ("…_cpp" dispatchers → C++ first).
unsafe fn identifier_probe_order(top_ident: *const std::os::raw::c_char) -> [&'static [u8]; 2] {
    let prefer_cpp = matches!(ffi::cstr(top_ident), Some(id) if id.contains("cpp"));
    if prefer_cpp {
        [
            ffi::introspection_cpp::INTROSPECTION_CPP_IDENTIFIER,
            ffi::INTROSPECTION_C_IDENTIFIER,
        ]
    } else {
        [
            ffi::INTROSPECTION_C_IDENTIFIER,
            ffi::introspection_cpp::INTROSPECTION_CPP_IDENTIFIER,
        ]
    }
}

/// Which introspection language a SERVICE typesupport resolved to.
pub(crate) enum ServiceIntrospectionHandle {
    C(*const ffi::rosidl_typesupport_introspection_c__ServiceMembers),
    Cpp(*const ffi::introspection_cpp::CppServiceMembers),
}

/// Resolve a service typesupport handle (same language-first probe
/// order as [`resolve_introspection`]).
///
/// # Safety
/// `ts` must be null or a valid rosidl service typesupport handle.
pub(crate) unsafe fn resolve_service_introspection(
    ts: *const ffi::rosidl_service_type_support_t,
) -> Option<ServiceIntrospectionHandle> {
    for (attempt, ident) in identifier_probe_order(if ts.is_null() {
        std::ptr::null()
    } else {
        (*ts).typesupport_identifier
    })
    .into_iter()
    .enumerate()
    {
        if attempt > 0 {
            // Same stale-rcutils-error hygiene as
            // resolve_introspection (see the comment there).
            ffi::rcutils_reset_error_best_effort();
        }
        if let Some(data) = resolve_service_for_identifier(ts, ident) {
            if ident == ffi::INTROSPECTION_C_IDENTIFIER {
                return Some(ServiceIntrospectionHandle::C(
                    data as *const ffi::rosidl_typesupport_introspection_c__ServiceMembers,
                ));
            }
            // Same gate as the message resolver: CppServiceMembers is
            // hand-mirrored to the Jazzy/Kilted-era shape too.
            if cpp_typesupport_gate().emit_refusal() {
                return None;
            }
            return Some(ServiceIntrospectionHandle::Cpp(
                data as *const ffi::introspection_cpp::CppServiceMembers,
            ));
        }
    }
    None
}

unsafe fn resolve_service_for_identifier(
    ts: *const ffi::rosidl_service_type_support_t,
    identifier: &[u8],
) -> Option<*const std::os::raw::c_void> {
    if ts.is_null() {
        return None;
    }
    // The service typesupport struct mirrors the message one for the
    // fields we touch (identifier / data / func) — resolve identically.
    let mut current = ts;
    for _ in 0..4 {
        let ident = (*current).typesupport_identifier;
        if identifier_matches(ident, identifier) {
            let data = (*current).data;
            return if data.is_null() { None } else { Some(data) };
        }
        let func = (*current).func?;
        let next = func(current, identifier.as_ptr() as *const std::os::raw::c_char);
        if next.is_null() {
            return None;
        }
        current = next;
    }
    None
}

unsafe fn resolve_for_identifier(
    ts: *const ffi::rosidl_message_type_support_t,
    identifier: &[u8],
) -> Option<*const ffi::rosidl_message_type_support_t> {
    if ts.is_null() {
        return None;
    }
    let mut current = ts;
    // Bounded handle-chain walk (dispatcher → concrete). 4 is generous;
    // real chains are depth 1-2.
    for _ in 0..4 {
        let ident = (*current).typesupport_identifier;
        if identifier_matches(ident, identifier) {
            return Some(current);
        }
        let func = (*current).func?;
        let next = func(current, identifier.as_ptr() as *const std::os::raw::c_char);
        if next.is_null() {
            return None;
        }
        current = next;
    }
    None
}

unsafe fn identifier_matches(ident: *const std::os::raw::c_char, expected: &[u8]) -> bool {
    match ffi::cstr(ident) {
        Some(s) => s.as_bytes() == &expected[..expected.len() - 1],
        None => false,
    }
}

// =====================================================================
// Dispatch-layer tests: the C-vs-C++ language
// resolution is the seam every rclcpp/MoveIt node hits FIRST. The cpp
// codec itself is covered in tests/cpp_bridge_test.rs; the resolver
// (probe order + handle-chain walk + null-data skip + fallback) and the
// AnyBridge::Cpp arm of bridge_for are covered here. These unit tests build
// fake rosidl typesupport chains (no ROS install needed — the function-
// pointer contract IS the interface, same strategy as cpp_bridge_test).
// =====================================================================
#[cfg(test)]
mod dispatch_tests {
    use super::*;
    use crate::bridge::AnyBridge;
    use crate::ffi::introspection_cpp::{
        CppMessageMember, CppMessageMembers, INTROSPECTION_CPP_IDENTIFIER,
    };
    use std::ffi::CString;
    use std::os::raw::{c_char, c_void};

    /// Leak a NUL-terminated identifier (process-lifetime, like rosidl's).
    fn ident(s: &str) -> *const c_char {
        CString::new(s).expect("ident").into_raw()
    }

    /// A non-null `data` sentinel the resolver only ever *casts* (never
    /// derefs) when selecting an arm — a real leaked allocation so it is
    /// a valid pointer, not a fabricated address.
    fn data_sentinel() -> *const c_void {
        Box::leak(Box::new(0u8)) as *const u8 as *const c_void
    }

    /// A leaf typesupport handle that matches `identifier` and carries
    /// `data` (use `data_sentinel()` for selection-only tests).
    fn leaf(identifier: &str, data: *const c_void) -> *const ffi::rosidl_message_type_support_t {
        Box::leak(Box::new(ffi::rosidl_message_type_support_t {
            typesupport_identifier: ident(identifier),
            data,
            ..Default::default()
        }))
    }

    /// The two leaves a dispatcher can resolve to (its `data` payload).
    #[repr(C)]
    struct Inners {
        c: *const ffi::rosidl_message_type_support_t,
        cpp: *const ffi::rosidl_message_type_support_t,
    }

    /// rosidl `type_support_dispatch` shape: hand back the leaf whose
    /// language matches the REQUESTED identifier (null if absent).
    unsafe extern "C" fn dispatch(
        current: *const ffi::rosidl_message_type_support_t,
        req: *const c_char,
    ) -> *const ffi::rosidl_message_type_support_t {
        let inners = &*((*current).data as *const Inners);
        let req = ffi::cstr(req).unwrap_or("");
        // Order matters: "introspection_cpp" contains "introspection_c".
        if req.contains("introspection_cpp") {
            inners.cpp
        } else if req.contains("introspection_c") {
            inners.c
        } else {
            std::ptr::null()
        }
    }

    /// A language dispatcher handle (top-level `…_cpp`/`…_c` identifier)
    /// that resolves to either inner via [`dispatch`].
    fn dispatcher(
        top_ident: &str,
        c: *const ffi::rosidl_message_type_support_t,
        cpp: *const ffi::rosidl_message_type_support_t,
    ) -> *const ffi::rosidl_message_type_support_t {
        let inners = Box::leak(Box::new(Inners { c, cpp }));
        Box::leak(Box::new(ffi::rosidl_message_type_support_t {
            typesupport_identifier: ident(top_ident),
            data: inners as *const Inners as *const c_void,
            func: Some(dispatch),
            ..Default::default()
        }))
    }

    #[test]
    fn probe_order_prefers_the_dispatcher_own_language() {
        unsafe {
            let cpp = identifier_probe_order(ident("rosidl_typesupport_cpp"));
            assert_eq!(
                cpp[0], INTROSPECTION_CPP_IDENTIFIER,
                "cpp dispatcher → C++ first"
            );
            assert_eq!(cpp[1], ffi::INTROSPECTION_C_IDENTIFIER);

            let c = identifier_probe_order(ident("rosidl_typesupport_c"));
            assert_eq!(
                c[0],
                ffi::INTROSPECTION_C_IDENTIFIER,
                "c dispatcher → C first"
            );
            assert_eq!(c[1], INTROSPECTION_CPP_IDENTIFIER);

            // A null top identifier must not panic and defaults to C-first.
            let none = identifier_probe_order(std::ptr::null());
            assert_eq!(none[0], ffi::INTROSPECTION_C_IDENTIFIER);
        }
    }

    #[test]
    fn resolves_cpp_arm_for_a_cpp_dispatcher() {
        unsafe {
            let c = leaf("rosidl_typesupport_introspection_c", data_sentinel());
            let cpp = leaf("rosidl_typesupport_introspection_cpp", data_sentinel());
            let disp = dispatcher("rosidl_typesupport_cpp", c, cpp);
            assert!(
                matches!(resolve_introspection(disp), Some(IntrospectionHandle::Cpp(p)) if !p.is_null()),
                "cpp dispatcher must resolve to the C++ introspection arm"
            );
        }
    }

    #[test]
    fn resolves_c_arm_for_a_c_dispatcher() {
        unsafe {
            let c = leaf("rosidl_typesupport_introspection_c", data_sentinel());
            let cpp = leaf("rosidl_typesupport_introspection_cpp", data_sentinel());
            let disp = dispatcher("rosidl_typesupport_c", c, cpp);
            assert!(
                matches!(resolve_introspection(disp), Some(IntrospectionHandle::C(p)) if !p.is_null()),
                "c dispatcher must resolve to the C introspection arm"
            );
        }
    }

    #[test]
    fn falls_back_to_c_when_preferred_cpp_inner_absent() {
        // A cpp-preferring dispatcher whose ONLY inner is the C one: the
        // first (C++) probe returns null, the resolver must reset the
        // rcutils error state and fall back to C — NOT return None.
        unsafe {
            let c = leaf("rosidl_typesupport_introspection_c", data_sentinel());
            let disp = dispatcher("rosidl_typesupport_cpp", c, std::ptr::null());
            assert!(
                matches!(resolve_introspection(disp), Some(IntrospectionHandle::C(_))),
                "must fall back to the C inner when C++ is absent"
            );
        }
    }

    #[test]
    fn direct_leaf_resolves_without_a_dispatcher() {
        unsafe {
            let cpp = leaf("rosidl_typesupport_introspection_cpp", data_sentinel());
            assert!(matches!(
                resolve_introspection(cpp),
                Some(IntrospectionHandle::Cpp(_))
            ));
        }
    }

    #[test]
    fn null_data_handle_is_not_resolved() {
        // A handle with the matching identifier but NULL data is not
        // introspection data — the resolver skips it (and, with no other
        // candidate, returns None rather than handing back a null arm).
        unsafe {
            let bad = leaf("rosidl_typesupport_introspection_cpp", std::ptr::null());
            assert!(resolve_introspection(bad).is_none());
        }
    }

    #[test]
    fn null_typesupport_resolves_to_none() {
        unsafe {
            assert!(resolve_introspection(std::ptr::null()).is_none());
        }
    }

    // --- The AnyBridge::Cpp arm of bridge_for, end-to-end through the ---
    // --- resolver with REAL C++ introspection members (Vector3).      ---

    const ROS_TYPE_DOUBLE: u8 = 2;

    fn cpp_member(name: &str, offset: u32) -> CppMessageMember {
        CppMessageMember {
            name_: ident(name),
            type_id_: ROS_TYPE_DOUBLE,
            string_upper_bound_: 0,
            members_: std::ptr::null(),
            is_key_: false,
            is_array_: false,
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
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer_: false,
        }
    }

    /// geometry_msgs/Vector3 as C++ introspection (3 doubles, all fixed).
    fn vector3_cpp_members() -> *const CppMessageMembers {
        let members = Box::leak(
            vec![cpp_member("x", 0), cpp_member("y", 8), cpp_member("z", 16)].into_boxed_slice(),
        );
        Box::leak(Box::new(CppMessageMembers {
            message_namespace_: ident("geometry_msgs::msg"),
            message_name_: ident("Vector3"),
            member_count_: 3,
            size_of_: 24,
            has_any_key_member_: false,
            members_: members.as_ptr(),
            init_function: None,
            fini_function: None,
        }))
    }

    #[test]
    fn bridge_for_builds_cpp_anybridge_and_flattens_like_native() {
        use cerulion_core::message::ShmMessage;
        use cerulion_core::wire::WireHeader;
        unsafe {
            // A cpp dispatcher whose C++ inner carries real Vector3 members.
            let cpp_leaf = leaf(
                "rosidl_typesupport_introspection_cpp",
                vector3_cpp_members() as *const c_void,
            );
            let disp = dispatcher("rosidl_typesupport_cpp", std::ptr::null(), cpp_leaf);

            let bridge = super::pubsub::bridge_for(disp).expect("bridge_for must resolve cpp");
            assert!(
                matches!(&*bridge, AnyBridge::Cpp(_)),
                "cpp dispatcher must yield the AnyBridge::Cpp arm"
            );
            // recipe-3: the bridged frame carries the NATIVE schema hash,
            // proving cpp-resolved frames interop byte-for-byte.
            assert_eq!(
                bridge.schema_hash(),
                <native_ros2_messages::geometry_msgs::Vector3 as ShmMessage>::SCHEMA_HASH
            );
            assert!(bridge.can_loan(), "all-fixed Vector3 is loanable");

            #[repr(C)]
            struct V3 {
                x: f64,
                y: f64,
                z: f64,
            }
            let msg = V3 {
                x: 1.0,
                y: -2.0,
                z: 0.5,
            };
            let frame = bridge
                .flatten(&msg as *const _ as *const c_void, 4, 40)
                .expect("flatten");
            assert_eq!(frame.len(), WireHeader::SIZE + 24);
            // Oracle: the wire fixed section is 3 packed LE f64s (the
            // documented Vector3 layout) — decode independently of the
            // bridge's own unflatten path.
            let p = &frame[WireHeader::SIZE..];
            let rd = |o: usize| f64::from_le_bytes(p[o..o + 8].try_into().unwrap());
            assert_eq!(rd(0), 1.0);
            assert_eq!(rd(8), -2.0);
            assert_eq!(rd(16), 0.5);
        }
    }
}
