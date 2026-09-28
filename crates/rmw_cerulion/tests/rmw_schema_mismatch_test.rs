// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The schema-hash-mismatch flood latch at the PRODUCTION `rmw_take`
//! call site (`rmw_cerulion/src/api/pubsub.rs`).
//!
//! A wire frame whose `schema_hash` does not match the subscription's type is
//! DROPPED. That is a version SKEW between the two ends, so it fails EVERY
//! frame until somebody redeploys — and earlier each drop emitted a BARE
//! per-frame `warn!`. On a 100 Hz topic that is ~100 lines/s ≈ 860 MB/day, the
//! exact class that once filled a 234 GB robot disk and that the earlier
//! latches already fixed for their own arms.
//!
//! The suppression policy is `cerulion_core`'s shared `FailureRegimeLatch`,
//! oracle-tested in `cerulion_core/tests/failure_regime_latch_test.rs`; the
//! four `cerulion_core` service arms are pinned in `service_test.rs`. THIS
//! file pins the rmw topic-take arm end-to-end through the real C ABI.
//!
//! Its OWN binary, not an arm of `rmw_e2e_test.rs`, because `#[traced_test]`
//! installs a GLOBAL tracing subscriber: any sibling test that brings the rmw
//! runtime up first takes that slot and the capture then panics
//! (`SetGlobalDefaultError`). Same shape and same reason as
//! `rmw_transient_local_ceiling_test.rs`, where every test is `#[traced_test]`
//! too.
//!
//! ⚠️ Shares the iceoryx2 SHM singleton — run serial:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_schema_mismatch_test -- --test-threads=1
//! ```

use cerulion_core::testing::{count_at, count_at_exclusively, debug_lines_expected, never_loud};
use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};

use rmw_cerulion::ffi::introspection_cpp::{
    CppMessageMember, CppMessageMembers, CppServiceMembers, VecTriplet,
};
use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;
// no-env-filter so the capture reaches the `cerulion_core` target (the
// schema-mismatch reporter's crate), not just this test crate.
use tracing_test::traced_test;

// =====================================================================
// Fixtures (cribbed from rmw_e2e_test.rs, exactly as
// rmw_transient_local_ceiling_test.rs does)
// =====================================================================

const ROS_TYPE_DOUBLE: u8 = 2;
const ROS_TYPE_BOOLEAN: u8 = 6;
const ROS_TYPE_UINT8: u8 = 8;
const ROS_TYPE_MESSAGE: u8 = 18;

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn member(
    name: &str,
    type_id: u8,
    offset: u32,
) -> ffi::rosidl_typesupport_introspection_c__MessageMember {
    ffi::rosidl_typesupport_introspection_c__MessageMember {
        name_: cstr(name),
        type_id_: type_id,
        offset_: offset,
        ..Default::default()
    }
}

fn make_message_ts(
    namespace: &str,
    name: &str,
    size_of: usize,
    members: Vec<ffi::rosidl_typesupport_introspection_c__MessageMember>,
) -> *const ffi::rosidl_message_type_support_t {
    let members = Box::leak(members.into_boxed_slice());
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr(namespace),
            message_name_: cstr(name),
            member_count_: members.len() as u32,
            size_of_: size_of,
            members_: members.as_ptr(),
            ..Default::default()
        },
    ));
    let ts = ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    };
    Box::leak(Box::new(ts))
}

#[repr(C)]
#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct CPoint {
    x: f64,
    y: f64,
    z: f64,
}

/// Unique TYPE NAME per call, so schema hashes never collide across runs
/// against the global SHM singleton — and so two typesupports with the SAME
/// layout can carry DIFFERENT hashes, which is exactly the skew under test.
fn point_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    make_message_ts(
        "rmw_mismatch__msg",
        unique,
        std::mem::size_of::<CPoint>(),
        vec![
            member("x", ROS_TYPE_DOUBLE, 0),
            member("y", ROS_TYPE_DOUBLE, 8),
            member("z", ROS_TYPE_DOUBLE, 16),
        ],
    )
}

/// A fake `std::vector<bool>`: BIT-PACKED like the real one, so nothing here
/// can be served by a contiguous copy.
#[repr(C)]
struct FakeVecBool {
    bits: *mut u8,
    len: usize,
}

unsafe extern "C" fn vecbool_size(field: *const c_void) -> usize {
    (*(field as *const FakeVecBool)).len
}

unsafe extern "C" fn vecbool_resize(field: *mut c_void, size: usize) {
    let v = &mut *(field as *mut FakeVecBool);
    let mut storage = vec![0u8; size.div_ceil(8).max(1)];
    v.bits = storage.as_mut_ptr();
    v.len = size;
    std::mem::forget(storage);
}

/// A C++ typesupport for a one-member `bool[]` type, with `size_function` and
/// `resize_function` but NO `assign_function`: the shape every distro before
/// Humble has by construction (its generator emits no `assign` for
/// `std::vector<bool>` and leaves `get`/`get_const` null), and the shape a
/// hand fixture reaches on any era by leaving the accessor unset.
fn cpp_bool_seq_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    cpp_ts(cpp_members(
        "rmw_mismatch::msg",
        unique,
        std::mem::size_of::<FakeVecBool>(),
        vec![cpp_unwritable_bool_member()],
        None,
        None,
    ))
}

/// A C++ introspection member with every accessor unset.
fn cpp_member(name: &str, type_id: u8, offset: u32, is_array: bool) -> CppMessageMember {
    CppMessageMember {
        name_: cstr(name),
        type_id_: type_id,
        string_upper_bound_: 0,
        members_: std::ptr::null(),
        #[cfg(cerulion_has_is_key)]
        is_key_: false,
        is_array_: is_array,
        array_size_: 0,
        is_upper_bound_: false,
        offset_: offset,
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
    ns: &str,
    name: &str,
    size_of: usize,
    members: Vec<CppMessageMember>,
    init: Option<unsafe extern "C" fn(*mut c_void, u32)>,
    fini: Option<unsafe extern "C" fn(*mut c_void)>,
) -> *const CppMessageMembers {
    let members = Box::leak(members.into_boxed_slice());
    Box::leak(Box::new(CppMessageMembers {
        message_namespace_: cstr(ns),
        message_name_: cstr(name),
        member_count_: members.len() as u32,
        size_of_: size_of,
        #[cfg(cerulion_has_is_key)]
        has_any_key_member_: false,
        members_: members.as_ptr(),
        init_function: init,
        fini_function: fini,
    }))
}

fn cpp_ts(members: *const CppMessageMembers) -> *const ffi::rosidl_message_type_support_t {
    Box::leak(Box::new(ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
        data: members as *const c_void,
        ..Default::default()
    }))
}

/// The pre-Humble `bool[]` member: the element COUNT is readable and the
/// container resizes, and no element can be written, because the C++
/// generator emits no `assign` for `std::vector<bool>`.
fn cpp_unwritable_bool_member() -> CppMessageMember {
    let mut flags = cpp_member("flags", ROS_TYPE_BOOLEAN, 0, true);
    flags.size_function = Some(vecbool_size);
    flags.resize_function = Some(vecbool_resize);
    flags
}

/// `{ Inner inner }` over `Inner { bool[] flags }`: the unwritable member
/// one level DOWN, where a census over top-level members alone never sees
/// it. Every level holds one member at offset 0, so the C++ struct of both
/// is a `FakeVecBool`.
fn cpp_nested_bool_members(ns: &str, name: &str, inner_name: &str) -> *const CppMessageMembers {
    let inner = cpp_members(
        "rmw_mismatch::msg",
        inner_name,
        std::mem::size_of::<FakeVecBool>(),
        vec![cpp_unwritable_bool_member()],
        None,
        None,
    );
    let mut nested = cpp_member("inner", ROS_TYPE_MESSAGE, 0, false);
    nested.members_ = cpp_ts(inner);
    cpp_members(
        ns,
        name,
        std::mem::size_of::<FakeVecBool>(),
        vec![nested],
        None,
        None,
    )
}

fn cpp_nested_bool_seq_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    cpp_ts(cpp_nested_bool_members(
        "rmw_mismatch::msg",
        unique,
        &format!("Inner{unique}"),
    ))
}

/// The C++ SERVICE typesupport whose request hides its `bool[]` in a nested
/// message. Its request schema is the twin of [`c_nested_service_ts`]'s, so
/// a C-typesupport client (rclpy, or any C node) can put a frame carrying
/// that member on the wire for this server to refuse.
fn cpp_nested_service_ts(unique: &str) -> *const ffi::rosidl_service_type_support_t {
    let request = cpp_nested_bool_members(
        "rmw_mismatch::srv",
        &format!("{unique}_Request"),
        &format!("Inner{unique}"),
    );
    let response = cpp_members(
        "rmw_mismatch::srv",
        &format!("{unique}_Response"),
        std::mem::size_of::<f64>(),
        vec![cpp_member("ok", ROS_TYPE_DOUBLE, 0, false)],
        None,
        None,
    );
    let sm = Box::leak(Box::new(CppServiceMembers {
        service_namespace_: cstr("rmw_mismatch::srv"),
        service_name_: cstr(unique),
        request_members_: request,
        response_members_: response,
        #[cfg(cerulion_has_event_members)]
        event_members_: std::ptr::null(),
    }));
    Box::leak(Box::new(ffi::rosidl_service_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
        data: sm as *const _ as *const c_void,
        ..Default::default()
    }))
}

/// rosidl's C `bool[]`: a plain pointer, size and capacity, which the C
/// bridge reads and writes as bytes. No accessor exists or is needed, which
/// is why this condition is a C++-only one.
#[repr(C)]
struct CBoolSeq {
    data: *mut bool,
    size: usize,
    capacity: usize,
}

/// The C twin of [`cpp_nested_service_ts`]: the SAME package, type names,
/// member names and field types, so the two bridges agree on the wire
/// schema hash and a C client's frame reaches the C++ server's take.
fn c_nested_service_ts(unique: &str) -> *const ffi::rosidl_service_type_support_t {
    let mut flags = member("flags", ROS_TYPE_BOOLEAN, 0);
    flags.is_array_ = true;
    let inner = make_message_ts(
        "rmw_mismatch__msg",
        &format!("Inner{unique}"),
        std::mem::size_of::<CBoolSeq>(),
        vec![flags],
    );
    let mut nested = member("inner", ROS_TYPE_MESSAGE, 0);
    nested.members_ = inner;
    let request = make_message_ts(
        "rmw_mismatch__srv",
        &format!("{unique}_Request"),
        std::mem::size_of::<CBoolSeq>(),
        vec![nested],
    );
    let response = make_message_ts(
        "rmw_mismatch__srv",
        &format!("{unique}_Response"),
        std::mem::size_of::<f64>(),
        vec![member("ok", ROS_TYPE_DOUBLE, 0)],
    );
    let sm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__ServiceMembers {
            service_namespace_: cstr("rmw_mismatch__srv"),
            service_name_: cstr(unique),
            request_members_: unsafe { (*request).data }
                as *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
            response_members_: unsafe { (*response).data }
                as *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
            ..Default::default()
        },
    ));
    Box::leak(Box::new(ffi::rosidl_service_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: sm as *const _ as *const c_void,
        ..Default::default()
    }))
}

/// A type the LOANED take serves: one FORGEABLE `uint8[]` (unbounded,
/// primitive, never bool, so the forged take aims its `std::vector` triplet
/// at the held sample) beside the nested `bool[]` no build before Humble can
/// write. Without the forgeable member `can_loan_take` is false and the
/// loaned take is never reached at all.
#[repr(C)]
struct CppLoanable {
    data: VecTriplet,
    inner: FakeVecBool,
}

unsafe extern "C" fn loanable_init(msg: *mut c_void, _all: u32) {
    // Both members are empty containers: a zeroed triplet is the empty
    // `std::vector` the forge overwrites, and the fixture's bit-packed
    // `bool[]` is zero length. Nothing is allocated, so `fini` frees nothing.
    std::ptr::write_bytes(msg as *mut u8, 0, std::mem::size_of::<CppLoanable>());
}

unsafe extern "C" fn loanable_fini(_msg: *mut c_void) {}

fn cpp_loanable_nested_bool_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    let inner = cpp_members(
        "rmw_mismatch::msg",
        &format!("Inner{unique}"),
        std::mem::size_of::<FakeVecBool>(),
        vec![cpp_unwritable_bool_member()],
        None,
        None,
    );
    let data = cpp_member(
        "data",
        ROS_TYPE_UINT8,
        std::mem::offset_of!(CppLoanable, data) as u32,
        true,
    );
    let mut nested = cpp_member(
        "inner",
        ROS_TYPE_MESSAGE,
        std::mem::offset_of!(CppLoanable, inner) as u32,
        false,
    );
    nested.members_ = cpp_ts(inner);
    cpp_ts(cpp_members(
        "rmw_mismatch::msg",
        unique,
        std::mem::size_of::<CppLoanable>(),
        vec![data, nested],
        Some(loanable_init),
        Some(loanable_fini),
    ))
}

static UNIQUE: AtomicU64 = AtomicU64::new(0);

fn unique_suffix() -> u64 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos() as u64;
    nanos ^ UNIQUE.fetch_add(1, Ordering::Relaxed)
}

unsafe fn setup_node(
    name: &str,
) -> (
    *mut ffi::rmw_context_t,
    *mut ffi::rmw_node_t,
    Box<ffi::rmw_init_options_t>,
) {
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    let node = rmw_create_node(context, cstr(name), cstr("/"));
    assert!(!node.is_null(), "node creation failed");
    (context, node, options)
}

fn default_qos() -> ffi::rmw_qos_profile_t {
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

// =====================================================================
// The pins
// =====================================================================

/// Substring unique to the LOUD (`warn!`) arm of the mismatch report.
const HASH_LOUD: &str = "dropping a frame whose wire schema hash does not match";
/// Substring unique to the SUPPRESSED (`debug!`) arm.
const HASH_SUPPRESSED: &str = "schema-hash mismatch suppressed";
/// Substring unique to the RECOVERY (`info!`) arm.
const HASH_RECOVERY: &str = "schema hashes match again";
/// Substring unique to the LOUD (`error!`) arm of the DECODE report.
const DECODE_LOUD: &str = "dropping a frame the bridge could not decode";
/// Substring unique to the SUPPRESSED (`debug!`) arm of the decode report.
const DECODE_SUPPRESSED: &str = "decode failure suppressed";
/// Substring unique to the LOUD (`error!`) arm of the PRE-WRITE entry refusal
/// (the arm that carries `var_idx=` and `reason=`).
const ENTRY_REFUSED_LOUD: &str = "refusing a frame before decoding it";
/// Substring unique to the SUPPRESSED (`debug!`) arm of the same reporter. Its
/// own arm, not the decode reporter's: a promotion of THIS repeat to `error!`
/// is what the flood latch exists to prevent, and keying on the other
/// reporter's text would never see it.
const ENTRY_REFUSED_SUPPRESSED: &str = "frame refused before decoding (regime still open)";
/// Substring unique to the DECADE RE-ANNOUNCEMENT (`error!`) arm of the decode
/// report — the arm that exists precisely to survive a filter hiding `debug!`.
const DECODE_STILL: &str = "decode failures are STILL dropping every frame";
/// Substring unique to `rmw_deserialize`'s own pre-write refusal, which has
/// no latch and no entity: a one-shot call has no stream to flood.
const DESERIALIZE_REFUSED: &str = "rmw_deserialize refused the buffer before writing anything";

/// The value of a rendered field whose own value contains SPACES: everything
/// between `<key>=` and the key that the emission declares NEXT. A
/// whitespace-terminated read would truncate such a value at its first space,
/// and `contains` would pass on a prefix; this returns the whole value so an
/// oracle can compare it with `assert_eq!`.
fn field_value_before(line: &str, key: &str, next_key: &str) -> Option<String> {
    // `rfind`, not `find`: this reporter's own headline PROSE names its fields
    // ("reason= says which check ..."), so the first occurrence of `key=` is
    // inside the message. tracing renders the message before the fields, so
    // the field is the LAST occurrence.
    let start = line.rfind(&format!("{key}="))? + key.len() + 1;
    let rest = &line[start..];
    let end = rest.find(&format!(" {next_key}="))?;
    Some(rest[..end].to_string())
}

/// Read the subscription's hash-mismatch counter — the log-level-independent
/// Principle #3 signal.
///
/// This cast is the ONLY way to reach it: the rmw C ABI is standardized, so no
/// accessor can be added for rclcpp/rclpy. That is precisely why the shared
/// latch re-announces an open regime at each decade of the running total — the
/// log is a ROS user's whole window onto the condition.
///
/// # Safety
/// `subscription` must be a live subscription created by this implementation.
unsafe fn hash_mismatch_count(subscription: *const ffi::rmw_subscription_t) -> u64 {
    let data = &*((*subscription).data as *const rmw_cerulion::runtime::SubscriptionData);
    cerulion_core::transport::failure_regime_latch::lock_regime_latch(&data.hash_mismatches)
        .total_failures()
}

/// Read the subscription's -failure counter (the sibling latch).
///
/// # Safety
/// `subscription` must be a live subscription created by this implementation.
unsafe fn decode_failure_count(subscription: *const ffi::rmw_subscription_t) -> u64 {
    let data = &*((*subscription).data as *const rmw_cerulion::runtime::SubscriptionData);
    data.decode_failures
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .total_failures()
}

/// The wire `schema_hash` this subscription's bridge expects.
///
/// # Safety
/// `subscription` must be a live subscription created by this implementation.
unsafe fn expected_schema_hash(subscription: *const ffi::rmw_subscription_t) -> u64 {
    let data = &*((*subscription).data as *const rmw_cerulion::runtime::SubscriptionData);
    data.bridge.schema_hash()
}

/// The CERULION topic name behind an rmw subscription (the fully-qualified
/// ROS name itself — the mapping is the identity) — what a raw
/// `cerulion_core` publisher must open.
///
/// # Safety
/// `subscription` must be a live subscription created by this implementation.
unsafe fn cerulion_topic(subscription: *const ffi::rmw_subscription_t) -> String {
    let data = &*((*subscription).data as *const rmw_cerulion::runtime::SubscriptionData);
    data.topic.clone()
}

/// A hand-built wire frame: 32-byte header + `payload_len` zero bytes.
///
/// `total_size` must equal the published slice length exactly, or the
/// subscriber's own bounds check drops the frame BEFORE the hash gate and
/// neither latch moves.
fn raw_frame(schema_hash: u64, sequence: u32, payload_len: usize) -> Vec<u8> {
    use cerulion_core::wire::WireHeader;
    let total = WireHeader::SIZE + payload_len;
    let header = WireHeader {
        schema_hash,
        total_size: total as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence,
        timestamp_ns: 0,
    };
    let mut frame = vec![0u8; total];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame
}

/// A member this build's C++ typesupport gives no way to WRITE refuses the
/// frame with the MEMBER NAMED, before anything is written.
///
/// A `bool[]` needs the member's `assign` accessor: `std::vector<bool>` is
/// bit-packed, so there is no element address to copy into. Before Humble the
/// C++ generator emits no `assign` (and leaves `get`/`get_const` null), so a
/// Foxy subscriber on a `bool[]` topic can never decode a frame a rclpy or
/// C-typesupport publisher puts there. What the user must be told is which
/// member and why, not that the wire is malformed, which is what the generic
/// entry refusal says and which sends an operator to redeploy both ends for a
/// limit of this build that no redeploy changes.
///
/// Oracles, three of them, all hand-written here:
/// * the WHOLE `reason=` value, typed out in this test rather than read back
///   from the constant the bridge renders (rule 1: the oracle is the message
///   the user reads), and compared with `assert_eq!`; a `contains` would pass
///   on a prefix, and the value carries spaces, so it is extracted between its
///   own key and the next field the emission declares;
/// * the side effect that must NOT have happened (rule 5): `taken` stays
///   false and the caller's message is byte-identical to the poison it went in
///   with, because the refusal is taken BEFORE `unflatten`;
/// * the refusal is LATCHED like every other decode refusal: exactly one
///   loud line from the reporter for two frames, with the unconditional
///   counter at 2, so a Foxy subscriber on a live topic neither floods the
///   disk nor goes silent.
#[test]
#[serial]
#[traced_test]
fn a_bool_member_this_build_cannot_write_is_refused_with_the_member_named() {
    const FRAMES: u64 = 2;
    unsafe {
        let suffix = unique_suffix();
        let ts = cpp_bool_seq_ts(&format!("Unwritable{suffix}"));
        let (_, node, _opts) = setup_node(&format!("node_{suffix}"));
        let topic = CString::new(format!("/rmw_mismatch/unwritable/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        // Registration SUCCEEDS: the limit is per message, never per build.
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        assert_eq!(decode_failure_count(subscription), 0);
        let cer_topic = cerulion_topic(subscription);
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let msl = cerulion_core::wire::MaxSliceLen::try_new(4096).expect("slice len");
        let mut raw_pub = rt
            .transport
            .create_publisher_simple(&cer_topic, msl)
            .expect("raw publisher on the subscription's topic");

        // The frame carries the subscription's OWN schema hash, so the hash
        // gate passes and the refusal under test is the one that fires. A
        // producer that CAN write this member (rclpy, or any C-typesupport
        // node) is what puts such a frame on the topic; what is pinned here is
        // the SUBSCRIBER's refusal.
        let expected = expected_schema_hash(subscription);
        // The caller's message, poisoned: a refusal must not touch one byte.
        let poison = FakeVecBool {
            bits: 0xA5A5_A5A5_A5A5_A5A5u64 as *mut u8,
            len: 0xA5A5_A5A5_A5A5_A5A5,
        };
        for i in 0..FRAMES {
            raw_pub
                .publish_raw(&raw_frame(expected, i as u32, 16))
                .expect("publish the hand frame");
            let mut out = FakeVecBool {
                bits: poison.bits,
                len: poison.len,
            };
            let mut taken = true;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK,
                "frame {i}: an undecodable member is a DROP, not a take failure"
            );
            assert!(
                !taken,
                "frame {i}: nothing was written, so nothing was taken"
            );
            assert_eq!(
                out.bits as usize, poison.bits as usize,
                "frame {i}: the refusal must not write the container's data pointer"
            );
            assert_eq!(
                out.len, poison.len,
                "frame {i}: nor its length, no resize, no assign, nothing"
            );
        }
        assert_eq!(
            decode_failure_count(subscription),
            FRAMES,
            "the counter is UNCONDITIONAL: it moves on the suppressed repeat too"
        );

        // The message the user reads. Typed out here, never read back from the
        // bridge's own constant.
        let want_reason = "bool sequence member 'flags' cannot be decoded: this build's C++ \
                           typesupport has no fetch accessor for std::vector<bool>: its \
                           generator emits no fetch or assign function and leaves get and \
                           get_const null, so no element is reachable";
        logs_assert(|lines: &[&str]| {
            let heads = count_at_exclusively(lines, "ERROR", &[ENTRY_REFUSED_LOUD])?;
            if heads != 1 {
                return Err(format!(
                    "{FRAMES} undecodable frames must produce exactly ONE loud refusal, got \
                     {heads}"
                ));
            }
            let head = lines
                .iter()
                .find(|l| l.contains(ENTRY_REFUSED_LOUD))
                .ok_or_else(|| "the loud refusal line is missing".to_string())?;
            let reason = field_value_before(head, "reason", "total_failures")
                .ok_or_else(|| format!("the refusal carries no reason= field: {head}"))?;
            if reason != want_reason {
                return Err(format!(
                    "the reason= value is not the one the user must read:\n  got:  {reason}\n  \
                     want: {want_reason}"
                ));
            }
            if !head.contains("var_idx=0") {
                return Err(format!("the refusal must name WHICH entry: {head}"));
            }
            // The repeats: release-safe, because `debug!` is compiled out
            // under `release_max_level_info` and a DEBUG count then reads 0
            // whatever the code did. The expectation therefore routes through
            // the static-level helper, and the level-free TWIN below is what
            // still fails in release if the suppressed arm is ever promoted:
            // a subscriber in this state refuses every frame, so a loud repeat
            // is the disk-fill class.
            let suppressed = count_at_exclusively(lines, "DEBUG", &[ENTRY_REFUSED_SUPPRESSED])?;
            let want = debug_lines_expected(FRAMES as usize - 1);
            if suppressed != want {
                return Err(format!(
                    "{FRAMES} refusals must leave exactly {want} suppressed repeat(s) at \
                     DEBUG, got {suppressed}"
                ));
            }
            never_loud(lines, ENTRY_REFUSED_SUPPRESSED)?;
            Ok(())
        });

        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The hash-mismatch latch at the PRODUCTION `rmw_take` call site (`api/pubsub.rs`).
///
/// A version-skewed peer publishes a type this subscription does not expect:
/// the hashes disagree, so EVERY frame is dropped until somebody redeploys.
///
/// Hand oracles: N=6 skewed frames ⇒ exactly ONE loud head + 5 suppressed
/// repeats + an UNCONDITIONAL counter of 6; then a matching frame ⇒ exactly
/// one recovery line carrying the SUPPRESSED count (5, not 6 — the loud head
/// was never suppressed) and a delivered message; then a fresh skew is loud
/// again (the re-arm). `taken` stays false throughout the regime — a dropped
/// frame must never be reported as taken.
///
/// The skew is built the way a real one arises: two typesupports with the same
/// layout and DIFFERENT type names, so the schema hashes differ while the
/// frames stay structurally publishable on one topic.
#[test]
#[serial]
#[traced_test]
fn wrong_hash_takes_are_loud_once_counted_always_and_recover() {
    const SKEWED_FRAMES: usize = 6;
    unsafe {
        let suffix = unique_suffix();
        // Same layout, different type NAME ⇒ different schema hash.
        let ours = point_ts(&format!("MismatchOurs{suffix}"));
        let skewed = point_ts(&format!("MismatchSkewed{suffix}"));
        let (_, node, _opts) = setup_node(&format!("node_{suffix}"));

        let topic = CString::new(format!("/rmw_mismatch/skew/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ours, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        assert_eq!(
            hash_mismatch_count(subscription),
            0,
            "a fresh subscription must start clean"
        );

        // The skewed peer.
        let bad_publisher = rmw_create_publisher(node, skewed, topic.as_ptr(), &qos, &pub_opts);
        assert!(!bad_publisher.is_null());

        for i in 0..SKEWED_FRAMES {
            let msg = CPoint {
                x: i as f64,
                y: 0.0,
                z: 0.0,
            };
            assert_eq!(
                rmw_publish(
                    bad_publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let mut out = CPoint::default();
            let mut taken = true;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK,
                "a hash mismatch is a DROP, not a take failure"
            );
            assert!(!taken, "a wrong-hash frame must never be reported as taken");
            assert_eq!(out, CPoint::default(), "the out buffer must be untouched");
        }
        assert_eq!(
            hash_mismatch_count(subscription),
            SKEWED_FRAMES as u64,
            "the counter is UNCONDITIONAL — it must count the debug-suppressed \
             repeats too, or a persistently skewed subscription is invisible at \
             RUST_LOG=error"
        );

        // Recovery: the operator redeploys, and a matching publisher takes
        // over the topic. The skewed publisher is destroyed first so the two
        // are never live at once — the rmw subscription path leaves
        // `max_publishers` at iceoryx2's default of 2, so this is a
        // FIDELITY choice (one writer per deployment), not a cap the
        // transport would have enforced.
        assert_eq!(rmw_destroy_publisher(node, bad_publisher), RMW_RET_OK);
        let good_publisher = rmw_create_publisher(node, ours, topic.as_ptr(), &qos, &pub_opts);
        assert!(!good_publisher.is_null());
        let healed = CPoint {
            x: 42.0,
            y: -1.0,
            z: 7.5,
        };
        assert_eq!(
            rmw_publish(
                good_publisher,
                &healed as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        let mut out = CPoint::default();
        let mut taken = false;
        assert_eq!(
            rmw_take(
                subscription,
                &mut out as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(taken, "the matching frame must be delivered");
        assert_eq!(out, healed);
        assert_eq!(
            hash_mismatch_count(subscription),
            SKEWED_FRAMES as u64,
            "recovery must NEVER reset the running total"
        );

        // Re-armed: a fresh skew is loud again.
        assert_eq!(rmw_destroy_publisher(node, good_publisher), RMW_RET_OK);
        let bad_again = rmw_create_publisher(node, skewed, topic.as_ptr(), &qos, &pub_opts);
        assert!(!bad_again.is_null());
        let msg = CPoint::default();
        assert_eq!(
            rmw_publish(
                bad_again,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        let mut taken = true;
        assert_eq!(
            rmw_take(
                subscription,
                &mut out as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(!taken);
        assert_eq!(hash_mismatch_count(subscription), SKEWED_FRAMES as u64 + 1);

        logs_assert(|lines: &[&str]| {
            let warns = count_at_exclusively(lines, "WARN", &[HASH_LOUD])?;
            never_loud(lines, HASH_SUPPRESSED)?;
            let debugs = count_at_exclusively(lines, "DEBUG", &[HASH_SUPPRESSED])?;
            let recoveries = count_at_exclusively(lines, "INFO", &[HASH_RECOVERY])?;
            if warns != 2 {
                return Err(format!(
                    "expected exactly 2 WARN loud heads (the {SKEWED_FRAMES}-frame regime, \
                     then the re-armed one) — a per-frame warn would give {}, got {warns}",
                    SKEWED_FRAMES + 1
                ));
            }
            let want_debugs = debug_lines_expected(SKEWED_FRAMES - 1);
            if debugs != want_debugs {
                return Err(format!(
                    "expected {want_debugs} DEBUG suppressed repeats, got {debugs}"
                ));
            }
            if recoveries != 1 {
                return Err(format!(
                    "expected exactly 1 INFO recovery line, got {recoveries}"
                ));
            }
            // A variant that emits the suppressed arm at `warn!` keeps
            // every count above intact while suppression does nothing.
            let leaked = count_at(lines, "WARN", HASH_SUPPRESSED);
            if leaked != 0 {
                return Err(format!(
                    "the suppressed arm must be DEBUG, found {leaked} at WARN"
                ));
            }
            let rec = lines
                .iter()
                .find(|l| l.contains(HASH_RECOVERY))
                .ok_or("no recovery line")?;
            if !rec.contains(&format!("suppressed_count={}", SKEWED_FRAMES - 1)) {
                return Err(format!(
                    "recovery must report the {} SUPPRESSED (not all {SKEWED_FRAMES}): {rec}",
                    SKEWED_FRAMES - 1
                ));
            }
            // Operators grep a topic drop by `topic=`, never a generic `name=`
            // — on the RECOVERY line as much as the head, since watching one
            // key must show the regime both open and close.
            if !rec.contains("topic=/rmw_mismatch/skew/") {
                return Err(format!("recovery must log under `topic=`: {rec}"));
            }
            let head = lines
                .iter()
                .find(|l| l.contains(HASH_LOUD))
                .ok_or("no loud head")?;
            if !head.contains("topic=/rmw_mismatch/skew/") {
                return Err(format!("loud head must log under `topic=`: {head}"));
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, bad_again), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// ANTI-TAUTOLOGY control for the test above: a MATCHED pub/sub pair emits
/// none of the lines and leaves the counter at exactly zero. Without
/// it, every "exactly N" assertion above would still hold if the reporter
/// also fired on matching frames.
#[test]
#[serial]
#[traced_test]
fn a_matched_pub_sub_pair_is_silent_and_counts_zero() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("MismatchQuiet{suffix}"));
        let (_, node, _opts) = setup_node(&format!("quiet_node_{suffix}"));

        let topic = CString::new(format!("/rmw_mismatch/quiet/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        for i in 0..6 {
            let msg = CPoint {
                x: i as f64,
                y: 1.0,
                z: 2.0,
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let mut out = CPoint::default();
            let mut taken = false;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            assert!(taken);
            assert_eq!(out, msg);
        }

        assert_eq!(hash_mismatch_count(subscription), 0);
        assert_eq!(decode_failure_count(subscription), 0);
        logs_assert(|lines: &[&str]| {
            let any = lines
                .iter()
                .filter(|l| {
                    l.contains(HASH_LOUD)
                        || l.contains(HASH_SUPPRESSED)
                        || l.contains(HASH_RECOVERY)
                        || l.contains(DECODE_LOUD)
                        || l.contains(DECODE_SUPPRESSED)
                })
                .count();
            if any == 0 {
                Ok(())
            } else {
                Err(format!(
                    "a matched pair must emit no hash-mismatch / decode-failure lines, got {any}"
                ))
            }
        });

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The SEPARATENESS of the hash latch from the decode
/// latch, at the production `rmw_take` site.
///
/// `SubscriptionData` deliberately carries TWO latches: a hash mismatch (the
/// two ends disagree on the TYPE — redeploy from the same schemas) and a
/// decode failure (they agree on the type and disagree on the FRAMING —
/// redeploy from the same build) are different conditions with different
/// remedies, and one open regime must never swallow the other's loud head.
/// Until this arm, that design claim had no test: merging the two latches left
/// the whole suite green.
///
/// Built from RAW wire frames on the subscription's own iceoryx2 topic,
/// because the two conditions need frames a real rmw publisher cannot produce:
/// a payload that carries the RIGHT hash but is too short for the bridge
/// layout (24 bytes of fixed section for a 3×f64 message).
///
/// Routing the decode-failure report through `data.hash_mismatches`
/// (one latch for both conditions) fails this test twice — the head is
/// downgraded to a suppressed repeat by the decode regime already open, and
/// `decode_failure_count` never leaves 0.
#[test]
#[serial]
#[traced_test]
fn an_open_decode_regime_does_not_swallow_the_hash_mismatch_head() {
    const DECODE_FRAMES: usize = 3;
    unsafe {
        let suffix = unique_suffix();
        let ours = point_ts(&format!("MismatchSep{suffix}"));
        let (_, node, _opts) = setup_node(&format!("sep_node_{suffix}"));

        let topic = CString::new(format!("/rmw_mismatch/sep/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ours, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());

        let expected = expected_schema_hash(subscription);
        let cer_topic = cerulion_topic(subscription);
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let msl = cerulion_core::wire::MaxSliceLen::try_new(4096).expect("slice len");
        let mut raw_pub = rt
            .transport
            .create_publisher_simple(&cer_topic, msl)
            .expect("raw publisher on the subscription's topic");

        // A 3×f64 message needs 24 bytes of fixed section; 8 is far short, so
        // the bridge's unflatten refuses AFTER the hash gate has passed.
        const SHORT_PAYLOAD: usize = 8;

        let take_one = || {
            let mut out = CPoint::default();
            let mut taken = true;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            taken
        };

        // Phase A — open a DECODE regime: right hash, unusable payload.
        for seq in 0..DECODE_FRAMES {
            raw_pub
                .publish_raw(&raw_frame(expected, seq as u32, SHORT_PAYLOAD))
                .expect("publish short frame");
            assert!(!take_one(), "an undecodable frame must never be taken");
        }
        assert_eq!(decode_failure_count(subscription), DECODE_FRAMES as u64);
        assert_eq!(
            hash_mismatch_count(subscription),
            0,
            "a frame whose hash MATCHED must not touch the hash counter"
        );

        // Phase B — with that regime OPEN, a hash mismatch must still be LOUD.
        let wrong = expected ^ 0xA5A5_A5A5_A5A5_A5A5;
        raw_pub
            .publish_raw(&raw_frame(wrong, DECODE_FRAMES as u32, SHORT_PAYLOAD))
            .expect("publish skewed frame");
        assert!(!take_one());
        assert_eq!(hash_mismatch_count(subscription), 1);
        assert_eq!(
            decode_failure_count(subscription),
            DECODE_FRAMES as u64,
            "a wrong-hash frame returns before the decode arm — the decode \
             counter must not move"
        );

        // Phase C — the decode regime was never closed by phase B, so the next
        // undecodable frame is a suppressed repeat, NOT a fresh loud head.
        raw_pub
            .publish_raw(&raw_frame(
                expected,
                DECODE_FRAMES as u32 + 1,
                SHORT_PAYLOAD,
            ))
            .expect("publish short frame");
        assert!(!take_one());
        assert_eq!(decode_failure_count(subscription), DECODE_FRAMES as u64 + 1);

        logs_assert(|lines: &[&str]| {
            let decode_heads = count_at_exclusively(lines, "ERROR", &[DECODE_LOUD])?;
            never_loud(lines, DECODE_SUPPRESSED)?;
            let decode_debugs = count_at_exclusively(lines, "DEBUG", &[DECODE_SUPPRESSED])?;
            let hash_heads = count_at_exclusively(lines, "WARN", &[HASH_LOUD])?;
            never_loud(lines, HASH_SUPPRESSED)?;
            let hash_debugs = count_at_exclusively(lines, "DEBUG", &[HASH_SUPPRESSED])?;
            let hash_recoveries = count_at_exclusively(lines, "INFO", &[HASH_RECOVERY])?;
            if decode_heads != 1 {
                return Err(format!(
                    "expected exactly 1 ERROR decode head over {} undecodable frames, \
                     got {decode_heads}",
                    DECODE_FRAMES + 1
                ));
            }
            let want_decode_debugs = debug_lines_expected(DECODE_FRAMES);
            if decode_debugs != want_decode_debugs {
                return Err(format!(
                    "expected {want_decode_debugs} DEBUG decode repeats, got {decode_debugs}"
                ));
            }
            if hash_heads != 1 {
                return Err(format!(
                    "an OPEN decode regime must not swallow the hash mismatch's loud \
                     head: expected 1 WARN, got {hash_heads}"
                ));
            }
            if hash_debugs != 0 {
                return Err(format!(
                    "the single hash mismatch must be the LOUD head, not a repeat of \
                     somebody else's regime: got {hash_debugs} DEBUG lines"
                ));
            }
            if hash_recoveries != 0 {
                return Err(format!(
                    "a lone hash mismatch re-arms SILENTLY, got {hash_recoveries} \
                     recovery lines"
                ));
            }
            Ok(())
        });

        drop(raw_pub);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The DECADE RE-ANNOUNCEMENT of the decode latch, at the production
/// `rmw_take` site — the arm the rmw sites need MOST.
///
/// `SubscriptionData::decode_failures` sits behind an opaque `*mut c_void` the
/// STANDARDIZED rmw C ABI hands to rclcpp/rclpy, so no accessor can be added
/// and the only reader is the cast above. A ROS user's whole window onto a
/// subscription dropping every frame is therefore the LOG, which is why an open
/// regime re-announces loudly at each power of ten instead of going silent
/// after one line.
///
/// That arm pins only the hash reporter's twin: every other
/// drive in the suite stops at 4 failures while the first decade boundary is
/// 10, so this `error!` was reachable by no assertion and free to be a
/// `debug!` — which would put it back under exactly the filter it exists to
/// survive.
///
/// Hand oracle over 10 undecodable frames in ONE open regime: 1 `ERROR` head +
/// 8 `DEBUG` repeats + 1 `ERROR` re-announcement carrying `total_failures=10`
/// and the UNCHANGED `suppressed=8`. Changing `error!` to `debug!` on that arm
/// fails the ERROR triple AND the DEBUG-negative guard.
///
/// Same RAW-frame stimulus as the separateness arm: the right hash with a
/// payload below the bridge's 24-byte fixed section is a shape no real rmw
/// publisher can produce, so it must be hand-built.
#[test]
#[serial]
#[traced_test]
fn an_open_decode_regime_re_announces_at_the_decade_at_error() {
    /// One full decade of undecodable frames — the 10th crosses the boundary.
    const DECODE_FRAMES: usize = 10;
    /// A 3×f64 message needs 24 bytes of fixed section; 8 is far short, so the
    /// bridge's unflatten refuses AFTER the hash gate has passed.
    const SHORT_PAYLOAD: usize = 8;
    unsafe {
        let suffix = unique_suffix();
        let ours = point_ts(&format!("DecodeDecade{suffix}"));
        let (_, node, _opts) = setup_node(&format!("decade_node_{suffix}"));

        let topic = CString::new(format!("/rmw_mismatch/decade/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ours, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());

        let expected = expected_schema_hash(subscription);
        let cer_topic = cerulion_topic(subscription);
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let msl = cerulion_core::wire::MaxSliceLen::try_new(4096).expect("slice len");
        let mut raw_pub = rt
            .transport
            .create_publisher_simple(&cer_topic, msl)
            .expect("raw publisher on the subscription's topic");

        for seq in 0..DECODE_FRAMES {
            raw_pub
                .publish_raw(&raw_frame(expected, seq as u32, SHORT_PAYLOAD))
                .expect("publish short frame");
            let mut out = CPoint::default();
            let mut taken = true;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            assert!(!taken, "an undecodable frame must never be taken");
        }
        assert_eq!(decode_failure_count(subscription), DECODE_FRAMES as u64);
        assert_eq!(
            hash_mismatch_count(subscription),
            0,
            "every frame carried the RIGHT hash — the hash counter must not move"
        );

        logs_assert(|lines: &[&str]| {
            let heads = count_at_exclusively(lines, "ERROR", &[DECODE_LOUD])?;
            let still = count_at_exclusively(lines, "ERROR", &[DECODE_STILL])?;
            never_loud(lines, DECODE_SUPPRESSED)?;
            let debugs = count_at_exclusively(lines, "DEBUG", &[DECODE_SUPPRESSED])?;
            let want_debugs = debug_lines_expected(8);
            if (heads, still, debugs) != (1, 1, want_debugs) {
                return Err(format!(
                    "expected (1 ERROR head, 1 ERROR decade re-announcement, {want_debugs} DEBUG \
                     repeats) over {DECODE_FRAMES} undecodable frames, got \
                     ({heads}, {still}, {debugs})"
                ));
            }
            if count_at(lines, "DEBUG", DECODE_STILL) != 0 {
                return Err(
                    "the re-announcement must be LOUD — it is the operator's only \
                     window at the rmw sites, where the counter is unreachable"
                        .to_string(),
                );
            }
            let line = lines
                .iter()
                .find(|l| l.contains(DECODE_STILL))
                .ok_or("no re-announcement line")?;
            for needle in ["total_failures=10", "suppressed=8"] {
                if !line.contains(needle) {
                    return Err(format!("re-announcement is missing {needle}: {line}"));
                }
            }
            // The reporters used to log a GENERIC `name=`
            // while the arms three lines away logged `topic=`, so one
            // subscription's two failure conditions could not be followed with
            // one query. Every decode line on a subscription now carries the
            // same key as its hash-mismatch twin.
            let head = lines
                .iter()
                .find(|l| l.contains(DECODE_LOUD))
                .ok_or("no decode head")?;
            for l in [head, line] {
                if !l.contains("topic=/rmw_mismatch/decade/") {
                    return Err(format!("decode line must log under `topic=`: {l}"));
                }
            }
            Ok(())
        });

        drop(raw_pub);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// The same refusal for a member one level DOWN. Every path that decodes
// C++ typesupport reads the registration record BEFORE its first write, so
// the nested member must be in that record or the decode writes half the
// destination and then fails.
// =====================================================================

/// The WHOLE `reason=` value a user reads when the unwritable `bool[]` sits
/// inside a nested message. Typed out here, never read back from the
/// constant the bridge renders: the oracle is the message the user reads.
/// The path is the outer member, a dot, the nested member, so the reader is
/// sent to the field itself and not to the message that contains it.
const NESTED_REASON: &str = "bool sequence member 'inner.flags' cannot be decoded: this build's \
                             C++ typesupport has no fetch accessor for std::vector<bool>: its \
                             generator emits no fetch or assign function and leaves get and \
                             get_const null, so no element is reachable";

/// Substring unique to the loud arm of the LOAN-refusal reporter, whose
/// absence proves the loaned take put its shadow back instead of retiring
/// it or leaking it.
const LOAN_REFUSED: &str = "loaned take refused";

/// Read a service server's decode-failure counter.
///
/// # Safety
/// `service` must be a live service created by this implementation.
unsafe fn service_decode_failure_count(service: *const ffi::rmw_service_t) -> u64 {
    let data = &*((*service).data as *const rmw_cerulion::runtime::ServiceData);
    data.decode_failures
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .total_failures()
}

/// The one loud refusal line, with its `reason=` value checked against the
/// hand oracle and its `var_idx=` against the top-level entry the member
/// rides in.
fn assert_one_refusal_naming(lines: &[&str], var_idx: usize) -> Result<(), String> {
    let heads = count_at_exclusively(lines, "ERROR", &[ENTRY_REFUSED_LOUD])?;
    if heads != 1 {
        return Err(format!("expected exactly ONE loud refusal, got {heads}"));
    }
    let head = lines
        .iter()
        .find(|l| l.contains(ENTRY_REFUSED_LOUD))
        .ok_or_else(|| "the loud refusal line is missing".to_string())?;
    let reason = field_value_before(head, "reason", "total_failures")
        .ok_or_else(|| format!("the refusal carries no reason= field: {head}"))?;
    if reason != NESTED_REASON {
        return Err(format!(
            "the reason= value is not the one the user must read:\n  got:  {reason}\n  want: \
             {NESTED_REASON}"
        ));
    }
    if !head.contains(&format!("var_idx={var_idx}")) {
        return Err(format!("the refusal must name WHICH entry: {head}"));
    }
    Ok(())
}

/// The COPYING subscription take refuses a frame whose unwritable `bool[]`
/// is one level down, before writing anything.
///
/// Registration used to census the top-level members only, so this member
/// was not in the record: the take started decoding, `unflatten` recursed
/// into the nested body, resized the nested sequence and only then hit the
/// missing accessor, leaving the caller's message part new and part old on
/// a call that reports nothing taken.
///
/// Oracles, all hand-written here: the WHOLE `reason=` value including the
/// PATH ([`NESTED_REASON`]); the side effect that must not have happened,
/// the destination byte-identical to the poison it went in with and `taken`
/// false; and the latch, one loud line for two frames with the
/// unconditional counter at two.
#[test]
#[serial]
#[traced_test]
fn a_nested_bool_member_this_build_cannot_write_is_refused_with_its_path() {
    const FRAMES: u64 = 2;
    unsafe {
        let suffix = unique_suffix();
        let ts = cpp_nested_bool_seq_ts(&format!("NestedUnwritable{suffix}"));
        let (_, node, _opts) = setup_node(&format!("node_{suffix}"));
        let topic = CString::new(format!("/rmw_mismatch/nested/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        // Registration SUCCEEDS: the limit is per message, never per build.
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        assert_eq!(decode_failure_count(subscription), 0);
        let cer_topic = cerulion_topic(subscription);
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let msl = cerulion_core::wire::MaxSliceLen::try_new(4096).expect("slice len");
        let mut raw_pub = rt
            .transport
            .create_publisher_simple(&cer_topic, msl)
            .expect("raw publisher on the subscription's topic");

        let expected = expected_schema_hash(subscription);
        // The caller's message, poisoned. The outer type holds one member at
        // offset 0 and that member holds one, so the whole struct IS the
        // nested bit-packed container.
        let poison = FakeVecBool {
            bits: 0xA5A5_A5A5_A5A5_A5A5u64 as *mut u8,
            len: 0xA5A5_A5A5_A5A5_A5A5,
        };
        for i in 0..FRAMES {
            raw_pub
                .publish_raw(&raw_frame(expected, i as u32, 16))
                .expect("publish the hand frame");
            let mut out = FakeVecBool {
                bits: poison.bits,
                len: poison.len,
            };
            let mut taken = true;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK,
                "frame {i}: an undecodable member is a DROP, not a take failure"
            );
            assert!(
                !taken,
                "frame {i}: nothing was written, so nothing was taken"
            );
            assert_eq!(
                out.bits as usize, poison.bits as usize,
                "frame {i}: the refusal must not write the nested container's data pointer"
            );
            assert_eq!(
                out.len, poison.len,
                "frame {i}: nor its length, no resize, no assign, nothing"
            );
        }
        assert_eq!(
            decode_failure_count(subscription),
            FRAMES,
            "the counter is UNCONDITIONAL: it moves on the suppressed repeat too"
        );
        // `inner` is the first and only variable member, so the entry the
        // refusal names is 0.
        logs_assert(|lines: &[&str]| assert_one_refusal_naming(lines, 0));

        drop(raw_pub);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The LOANED take refuses the same frame before forging, and puts its
/// shadow back.
///
/// This path writes into an rmw-owned shadow rather than the caller's
/// message, so the partial write is not the caller's problem; what it
/// reported instead was the generic malformed-entry line, which blames the
/// WIRE for a limit of this build and sends an operator to redeploy both
/// ends, and it retired a shadow per frame.
///
/// Oracles: the same hand-written `reason=` value, the entry index of the
/// top-level member the nested one rides in (`data` is entry 0, `inner` is
/// entry 1), the loaned pointer still NULL, and the absence of any
/// loan-refusal line, which is what a retired or leaked shadow would
/// eventually produce.
#[test]
#[serial]
#[traced_test]
fn the_loaned_take_refuses_a_nested_unwritable_member_and_keeps_its_shadow() {
    unsafe {
        let suffix = unique_suffix();
        let ts = cpp_loanable_nested_bool_ts(&format!("LoanNested{suffix}"));
        let (_, node, _opts) = setup_node(&format!("node_{suffix}"));
        let topic = CString::new(format!("/rmw_mismatch/nested_loan/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let cer_topic = cerulion_topic(subscription);
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let msl = cerulion_core::wire::MaxSliceLen::try_new(4096).expect("slice len");
        let mut raw_pub = rt
            .transport
            .create_publisher_simple(&cer_topic, msl)
            .expect("raw publisher on the subscription's topic");
        let expected = expected_schema_hash(subscription);
        raw_pub
            .publish_raw(&raw_frame(expected, 0, 32))
            .expect("publish the hand frame");

        let mut loaned: *mut c_void = std::ptr::null_mut();
        let mut taken = true;
        let ret =
            rmw_take_loaned_message(subscription, &mut loaned, &mut taken, std::ptr::null_mut());
        assert_ne!(
            ret,
            ffi::RMW_RET_UNSUPPORTED,
            "the fixture must be served by the loaned take: its uint8[] member is forgeable, so \
             can_loan_take holds"
        );
        assert_eq!(ret, RMW_RET_OK, "an undecodable member is a DROP");
        assert!(!taken, "nothing was forged, so nothing was taken");
        assert!(loaned.is_null(), "a refused loan hands out no pointer");
        assert_eq!(decode_failure_count(subscription), 1);
        logs_assert(|lines: &[&str]| {
            assert_one_refusal_naming(lines, 1)?;
            if lines.iter().any(|l| l.contains(LOAN_REFUSED)) {
                return Err(
                    "the shadow must go back to the pool UNUSED: no loan refusal belongs here"
                        .to_string(),
                );
            }
            Ok(())
        });

        drop(raw_pub);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// A SERVICE server on such a request type refuses it before writing
/// anything into the caller's request message.
///
/// The producer is a real client on the C typesupport, which writes a
/// `bool[]` as plain bytes and needs no accessor: exactly the pair a user
/// hits, an rclpy or C node calling a C++ server on a build before Humble.
/// The two typesupports carry the same package, type names, member names
/// and field types, so they agree on the wire schema hash and the frame
/// reaches the server's take.
///
/// Oracles: the same hand-written `reason=` value; the request message
/// byte-identical to its poison with `taken` false; and the refusal logged
/// under `service=` with the server's own counter at one.
#[test]
#[serial]
#[traced_test]
fn a_service_request_with_a_nested_unwritable_member_is_refused_before_any_write() {
    unsafe {
        let suffix = unique_suffix();
        let unique = format!("NestedSvc{suffix}");
        let (_, node, _opts) = setup_node(&format!("svc_node_{suffix}"));
        let service_name =
            CString::new(format!("/rmw_mismatch/nested_svc/{suffix}")).expect("name");
        let qos = default_qos();

        let service = rmw_create_service(
            node,
            cpp_nested_service_ts(&unique),
            service_name.as_ptr(),
            &qos,
        );
        assert!(!service.is_null(), "service creation failed");
        let client = rmw_create_client(
            node,
            c_nested_service_ts(&unique),
            service_name.as_ptr(),
            &qos,
        );
        assert!(!client.is_null(), "client creation failed");
        let mut available = false;
        assert_eq!(
            rmw_service_server_is_available(node, client, &mut available),
            RMW_RET_OK
        );
        assert!(available, "server must be visible to the client");

        #[repr(C)]
        struct CNestedRequest {
            flags: CBoolSeq,
        }
        #[repr(C)]
        struct CRequest {
            inner: CNestedRequest,
        }
        let mut bools = [true, false, true];
        let request = CRequest {
            inner: CNestedRequest {
                flags: CBoolSeq {
                    data: bools.as_mut_ptr(),
                    size: bools.len(),
                    capacity: bools.len(),
                },
            },
        };
        let mut sequence_id: i64 = 0;
        assert_eq!(
            rmw_send_request(
                client,
                &request as *const _ as *const c_void,
                &mut sequence_id
            ),
            RMW_RET_OK,
            "the C bridge writes a bool[] as bytes, so the client can send it"
        );
        assert_eq!(sequence_id, 1);

        let poison = FakeVecBool {
            bits: 0xA5A5_A5A5_A5A5_A5A5u64 as *mut u8,
            len: 0xA5A5_A5A5_A5A5_A5A5,
        };
        let mut out = FakeVecBool {
            bits: poison.bits,
            len: poison.len,
        };
        let mut header: ffi::rmw_service_info_t = std::mem::zeroed();
        let mut taken = true;
        assert_eq!(
            rmw_take_request(
                service,
                &mut header,
                &mut out as *mut _ as *mut c_void,
                &mut taken
            ),
            RMW_RET_OK,
            "an undecodable member is a DROP, not a take failure"
        );
        assert!(!taken, "nothing was written, so nothing was taken");
        assert_eq!(
            out.bits as usize, poison.bits as usize,
            "the refusal must not write the nested container's data pointer"
        );
        assert_eq!(out.len, poison.len, "nor its length");
        assert_eq!(
            service_decode_failure_count(service),
            1,
            "the request reached the server and was counted, so the schemas agree"
        );
        let want_service = format!("service=/rmw_mismatch/nested_svc/{suffix}");
        logs_assert(move |lines: &[&str]| {
            assert_one_refusal_naming(lines, 0)?;
            let head = lines
                .iter()
                .find(|l| l.contains(ENTRY_REFUSED_LOUD))
                .ok_or_else(|| "the loud refusal line is missing".to_string())?;
            if !head.contains(&want_service) {
                return Err(format!("the refusal must log under `service=`: {head}"));
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_client(node, client), RMW_RET_OK);
        assert_eq!(rmw_destroy_service(node, service), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// `rmw_deserialize` refuses the buffer before writing anything.
///
/// It decodes into a CALLER-owned message exactly as a take does, with no
/// entity to hang a latch on, so it answers a bare error code; before the
/// refusal it answered that code over a message the caller had to treat as
/// garbage. This is also the pin the crate's notes ask for on any change to
/// this function.
///
/// Oracles: the hand-written `reason=` value, the caller's message
/// byte-identical to its poison, and `RMW_RET_ERROR`. No node and no
/// transport: this entry point owns nothing.
#[test]
#[serial]
#[traced_test]
fn rmw_deserialize_refuses_a_nested_unwritable_member_before_any_write() {
    unsafe {
        let suffix = unique_suffix();
        let ts = cpp_nested_bool_seq_ts(&format!("Deser{suffix}"));
        // The schema hash in the header is never read: the refusal is taken
        // before the bridge looks at one byte of the buffer.
        let frame = raw_frame(0, 0, 16);
        let mut input: ffi::rmw_serialized_message_t = std::mem::zeroed();
        input.buffer = frame.as_ptr() as *mut u8;
        input.buffer_length = frame.len();
        input.buffer_capacity = frame.len();

        let poison = FakeVecBool {
            bits: 0xA5A5_A5A5_A5A5_A5A5u64 as *mut u8,
            len: 0xA5A5_A5A5_A5A5_A5A5,
        };
        let mut out = FakeVecBool {
            bits: poison.bits,
            len: poison.len,
        };
        assert_eq!(
            rmw_deserialize(&input, ts, &mut out as *mut _ as *mut c_void),
            ffi::RMW_RET_ERROR
        );
        assert_eq!(out.bits as usize, poison.bits as usize);
        assert_eq!(out.len, poison.len);
        logs_assert(|lines: &[&str]| {
            let head = lines
                .iter()
                .find(|l| l.contains(DESERIALIZE_REFUSED))
                .ok_or_else(|| "rmw_deserialize logged no refusal".to_string())?;
            let reason = field_value_before(head, "reason", "type")
                .ok_or_else(|| format!("the refusal carries no reason= field: {head}"))?;
            if reason != NESTED_REASON {
                return Err(format!(
                    "the reason= value is not the one the user must read:\n  got:  {reason}\n  \
                     want: {NESTED_REASON}"
                ));
            }
            Ok(())
        });
    }
}
