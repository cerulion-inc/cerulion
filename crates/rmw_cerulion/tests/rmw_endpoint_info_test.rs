// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! `rmw_get_publishers_info_by_topic` /
//! `rmw_get_subscriptions_info_by_topic` over the process-local graph
//! cache. Drives the EXACT extern "C" surface rviz2 / MCAP tools /
//! `ros2 topic info -v` and rclpy's `get_publishers_info_by_topic` call.
//! Hand-built introspection typesupports (same pattern as
//! `rmw_e2e_test.rs`) — no ROS installation required.
//!
//! Process-LOCAL scope: these queries only see THIS process's endpoints
//! (cross-process discovery is not implemented), which is
//! exactly what the in-process test harness exercises.
//!
//! ⚠️ Shares the iceoryx2 SHM singleton + the process-global graph
//! registry — run serial:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_endpoint_info_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::collections::HashSet;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::Mutex;

use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;

// =====================================================================
// Fixtures (cribbed from rmw_e2e_test.rs)
// =====================================================================

const ROS_TYPE_DOUBLE: u8 = 2;

/// The transport's default subscriber queue depth — the depth the
/// SUBSCRIPTION round-trip arm below is really provisioned, since a
/// subscription's requested depth is never applied to its queue at all
/// and its queue depth IS its retention.
///
/// NOT the publisher arm's oracle. The earlier slot-depth reporting rule reported a publisher's
/// subscriber-slot CEILING (which for a TRANSIENT_LOCAL depth-5 publisher is
/// this same 16, since the raise `(5*4).div_ceil(3) = 7` never lowers the stock
/// default); the retention-depth refinement reports its RETENTION instead — see
/// [`RETAINED_TL_DEPTH_5`].
///
/// HAND-WRITTEN, never read back from the transport — reading
/// `subscriber_buffer_size()` would ask the production code to confirm its own
/// answer. `the_transport_default_is_still_the_hand_written_oracle` below is
/// the drift guard.
const DEFAULT_PROVISIONED_DEPTH: usize = 16;

/// The retention a TRANSIENT_LOCAL
/// depth-5 publisher is provisioned, and so the depth its endpoint info
/// reports — `5.clamp(1, 16)`, hand-computed. The second worked case
/// from that decision ("TL depth-5 → 5").
const RETAINED_TL_DEPTH_5: usize = 5;

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

/// Type name "rmw_ep__msg/<unique>" → qualified "rmw_ep/<unique>" →
/// ROS graph type "rmw_ep/msg/<unique>". A 3×f64 POD (no publish/take
/// here, so only the layout size matters).
fn ep_point_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    make_message_ts(
        "rmw_ep__msg",
        unique,
        3 * std::mem::size_of::<f64>(),
        vec![
            member("x", ROS_TYPE_DOUBLE, 0),
            member("y", ROS_TYPE_DOUBLE, 8),
            member("z", ROS_TYPE_DOUBLE, 16),
        ],
    )
}

static UNIQUE: AtomicU64 = AtomicU64::new(0);

fn unique_suffix() -> u64 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos() as u64;
    nanos ^ UNIQUE.fetch_add(1, Ordering::Relaxed)
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

// A REAL malloc-backed rcutils_allocator_t — rcl always passes one, and
// the endpoint-info fill allocates the array + strings through it.
extern "C" {
    fn malloc(size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}
unsafe extern "C" fn fixture_allocate(size: usize, _state: *mut c_void) -> *mut c_void {
    malloc(size)
}
unsafe extern "C" fn fixture_deallocate(ptr: *mut c_void, _state: *mut c_void) {
    free(ptr)
}
fn malloc_allocator() -> ffi::rcutils_allocator_t {
    let mut a: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
    a.allocate = Some(fixture_allocate);
    a.deallocate = Some(fixture_deallocate);
    a
}

unsafe fn read_cstr(p: *const c_char) -> String {
    assert!(!p.is_null(), "endpoint info string must not be null");
    CStr::from_ptr(p).to_str().expect("utf-8").to_string()
}

// =====================================================================
// Fault-injecting, leak-tracking allocator
// =====================================================================

/// Countdown fault injector + pointer ledger: `allocate` succeeds
/// `remaining_successes` times then returns null (rcutils allocators
/// report OOM as null); every successful allocation and every free is
/// recorded BY POINTER so tests can assert the OOM rollback in
/// `fill_endpoint_info_array` frees each succeeded allocation exactly
/// once (no leak, no double-free). The callbacks NEVER assert — a panic
/// there would unwind across the C ABI — tests inspect the ledger after
/// the call. Reached via the allocator's `state` pointer (no statics,
/// no cross-test bleed).
#[derive(Default)]
struct FaultAllocState {
    /// Allocations allowed to succeed before injecting null.
    remaining_successes: AtomicUsize,
    /// Every pointer handed out (in allocation order; no frees happen
    /// before the rollback in these tests, so malloc address reuse
    /// cannot alias entries within the measured window).
    allocated: Mutex<Vec<usize>>,
    /// Every pointer freed (in free order; duplicates = double-free).
    freed: Mutex<Vec<usize>>,
}

impl FaultAllocState {
    fn with_budget(n: usize) -> Self {
        Self {
            remaining_successes: AtomicUsize::new(n),
            ..Default::default()
        }
    }

    /// Exactly-once teardown check: no double-free (freed list has no
    /// duplicate pointers) and freed set == allocated set (nothing
    /// leaked, nothing foreign freed).
    fn assert_no_leak_no_double_free(&self) {
        let allocated = self.allocated.lock().expect("ledger").clone();
        let freed = self.freed.lock().expect("ledger").clone();
        let freed_set: HashSet<usize> = freed.iter().copied().collect();
        assert_eq!(
            freed.len(),
            freed_set.len(),
            "double-free detected in rollback: {freed:?}"
        );
        let allocated_set: HashSet<usize> = allocated.iter().copied().collect();
        assert_eq!(
            allocated_set, freed_set,
            "rollback must free every succeeded allocation exactly once \
             (allocated {allocated:?} vs freed {freed:?})"
        );
    }
}

unsafe extern "C" fn fault_allocate(size: usize, state: *mut c_void) -> *mut c_void {
    let st = &*(state as *const FaultAllocState);
    // Atomic countdown: succeed while the budget lasts, then null.
    if st
        .remaining_successes
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |n| n.checked_sub(1))
        .is_err()
    {
        return std::ptr::null_mut();
    }
    let p = malloc(size);
    if !p.is_null() {
        // Poison-tolerant lock: a callback must never panic (unwinding
        // out of an extern "C" fn aborts the process).
        st.allocated
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(p as usize);
    }
    p
}

unsafe extern "C" fn fault_deallocate(ptr: *mut c_void, state: *mut c_void) {
    let st = &*(state as *const FaultAllocState);
    let mut freed = st.freed.lock().unwrap_or_else(|e| e.into_inner());
    // Record FIRST, real-free at most once: a double-free regression is
    // detected by the ledger's duplicate entry WITHOUT corrupting the
    // test process's heap (freeing the same malloc pointer twice is UB
    // that could crash before the assertion runs).
    let already_freed = freed.contains(&(ptr as usize));
    freed.push(ptr as usize);
    if !already_freed {
        free(ptr);
    }
}

/// Allocator wired to a [`FaultAllocState`] via the `state` pointer;
/// `state` must outlive every call made through the returned allocator.
fn fault_allocator(state: &FaultAllocState) -> ffi::rcutils_allocator_t {
    let mut a: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
    a.allocate = Some(fault_allocate);
    a.deallocate = Some(fault_deallocate);
    a.state = state as *const FaultAllocState as *mut c_void;
    a
}

// =====================================================================
// Publisher / subscription endpoint info
// =====================================================================

/// Drift guard for [`DEFAULT_PROVISIONED_DEPTH`] — fail HERE, naming the
/// constant, rather than failing the two round-trip arms with a mystifying
/// number if the transport default ever moves.
#[test]
#[serial]
fn the_transport_default_is_still_the_hand_written_oracle() {
    let rt = rmw_cerulion::runtime::runtime().expect("runtime");
    assert_eq!(
        rt.transport.subscriber_buffer_size(),
        DEFAULT_PROVISIONED_DEPTH,
        "the transport's default subscriber buffer moved; update \
         DEFAULT_PROVISIONED_DEPTH and re-check the depth assertions"
    );
}

#[test]
#[serial]
fn publisher_endpoint_info_round_trips_all_fields() {
    unsafe {
        let suffix = unique_suffix();
        let unique = format!("EpP{suffix}");
        let ts = ep_point_ts(&unique);
        let node_name = format!("ep_pub_node_{suffix}");
        let (node, _opts) = setup_node(&node_name);

        let topic = CString::new(format!("/rmw_ep/pub/{suffix}")).expect("topic");
        let mut qos = default_qos();
        qos.durability = ffi::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        qos.depth = RETAINED_TL_DEPTH_5;
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        // Oracle for the gid: the SAME entity gid rmw hands out via the
        // dedicated getter (not a self-compare of the registry with itself).
        let mut gid: ffi::rmw_gid_t = std::mem::zeroed();
        assert_eq!(rmw_get_gid_for_publisher(publisher, &mut gid), RMW_RET_OK);

        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_publishers_info_by_topic(node, &mut allocator, topic.as_ptr(), false, &mut arr,),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 1, "exactly one local publisher");
        assert!(!arr.info_array.is_null());
        let info = &*arr.info_array;
        assert_eq!(read_cstr(info.node_name), node_name);
        assert_eq!(read_cstr(info.node_namespace), "/");
        assert_eq!(read_cstr(info.topic_type), format!("rmw_ep/msg/{unique}"));
        assert_eq!(info.endpoint_type, ffi::RMW_ENDPOINT_PUBLISHER);
        assert_eq!(
            info.endpoint_gid, gid.data,
            "gid == rmw_get_gid_for_publisher"
        );
        // QoS round-trip on the four negotiated axes.
        assert_eq!(
            info.qos_profile.reliability,
            ffi::RMW_QOS_POLICY_RELIABILITY_RELIABLE
        );
        assert_eq!(
            info.qos_profile.durability,
            ffi::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        );
        assert_eq!(
            info.qos_profile.history,
            ffi::RMW_QOS_POLICY_HISTORY_KEEP_LAST
        );
        // DEPTH IS THE PROVISIONED
        // RETENTION. TRANSIENT_LOCAL depth 5 clamps history to 5, so 5 frames
        // are what a late joiner gets and 5 is what this reports. It coincides
        // with the request here, and that is the point: the surface is not
        // echoing, it is reporting a retention that happens to equal the ask
        // (the depth-1000 arm in `rmw_transient_local_ceiling_test.rs` is where
        // the two diverge, and this create is correspondingly QUIET — pinned
        // there too). The earlier slot-depth version of this assertion read the
        // subscriber-slot ceiling `DEFAULT_PROVISIONED_DEPTH` (16, since the
        // raise `(5*4).div_ceil(3) = 7` never lowers the stock default), which
        // over-claimed by 11 frames. The other three axes above still carry the
        // REQUESTED values and are unchanged by either decision.
        assert_eq!(
            info.qos_profile.depth, RETAINED_TL_DEPTH_5,
            "endpoint info reports the depth Cerulion RETAINS ({RETAINED_TL_DEPTH_5}), \
             never the subscriber-slot ceiling {DEFAULT_PROVISIONED_DEPTH}"
        );

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

#[test]
#[serial]
fn subscription_endpoint_info_round_trips_all_fields() {
    unsafe {
        let suffix = unique_suffix();
        let unique = format!("EpS{suffix}");
        let ts = ep_point_ts(&unique);
        let node_name = format!("ep_sub_node_{suffix}");
        let (node, _opts) = setup_node(&node_name);

        let topic = CString::new(format!("/rmw_ep/sub/{suffix}")).expect("topic");
        let mut qos = default_qos();
        qos.depth = 7; // VOLATILE
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());

        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_subscriptions_info_by_topic(
                node,
                &mut allocator,
                topic.as_ptr(),
                false,
                &mut arr,
            ),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 1);
        let info = &*arr.info_array;
        assert_eq!(read_cstr(info.node_name), node_name);
        assert_eq!(read_cstr(info.node_namespace), "/");
        assert_eq!(read_cstr(info.topic_type), format!("rmw_ep/msg/{unique}"));
        assert_eq!(info.endpoint_type, ffi::RMW_ENDPOINT_SUBSCRIPTION);
        // No rmw_get_gid_for_subscription surface; assert the deterministic
        // gid is present (entity counter starts at 1, so never all-zero).
        assert_ne!(info.endpoint_gid, [0u8; 16], "subscription gid is set");
        assert_eq!(
            info.qos_profile.durability,
            ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE
        );
        // A subscription's requested depth is never applied to its
        // iceoryx2 queue, so the true number here is the
        // transport default it really got — reporting the 7 it asked for would
        // be an echo of the request, on the endpoint class where the ask
        // changed nothing at all.
        assert_eq!(
            info.qos_profile.depth, DEFAULT_PROVISIONED_DEPTH,
            "a subscription reports the queue depth it really got, never the \
             requested 7"
        );

        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

#[test]
#[serial]
fn unknown_topic_yields_empty_array_and_ok() {
    unsafe {
        let suffix = unique_suffix();
        let (node, _opts) = setup_node(&format!("ep_none_node_{suffix}"));
        // A topic no endpoint ever created.
        let topic = CString::new(format!("/rmw_ep/never/{suffix}")).expect("topic");
        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_publishers_info_by_topic(node, &mut allocator, topic.as_ptr(), false, &mut arr,),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 0, "no endpoints → empty array");
        assert!(arr.info_array.is_null(), "empty array has a null buffer");

        // Subscription side too.
        let mut arr2: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_subscriptions_info_by_topic(
                node,
                &mut allocator,
                topic.as_ptr(),
                false,
                &mut arr2,
            ),
            RMW_RET_OK
        );
        assert_eq!(arr2.size, 0);
        assert!(arr2.info_array.is_null());

        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

#[test]
#[serial]
fn two_publishers_distinct_topics_isolated_entries() {
    unsafe {
        let suffix = unique_suffix();
        let unique = format!("EpM{suffix}");
        let ts = ep_point_ts(&unique);
        let (node, _opts) = setup_node(&format!("ep_multi_node_{suffix}"));

        // Cerulion provisions graph topics single-writer, so two publishers
        // cannot share ONE topic over the SHM transport — the genuine
        // same-topic 2-entry case is covered by `two_subscriptions_...`
        // (subscriptions carry no writer cap). Here: two publishers on two
        // topics owned by ONE node each resolve to their own single entry
        // (per-topic isolation of the endpoint map).
        let topic_a = CString::new(format!("/rmw_ep/ma/{suffix}")).expect("topic");
        let topic_b = CString::new(format!("/rmw_ep/mb/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let pub_a = rmw_create_publisher(node, ts, topic_a.as_ptr(), &qos, &pub_opts);
        let pub_b = rmw_create_publisher(node, ts, topic_b.as_ptr(), &qos, &pub_opts);
        assert!(!pub_a.is_null() && !pub_b.is_null());

        let mut allocator = malloc_allocator();
        for topic in [&topic_a, &topic_b] {
            let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
            assert_eq!(
                rmw_get_publishers_info_by_topic(
                    node,
                    &mut allocator,
                    topic.as_ptr(),
                    false,
                    &mut arr,
                ),
                RMW_RET_OK
            );
            assert_eq!(arr.size, 1);
        }

        assert_eq!(rmw_destroy_publisher(node, pub_a), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, pub_b), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Two subscriptions on the SAME topic (subscriptions carry no
/// single-writer cap) → exactly two endpoint records, both gids present
/// and DISTINCT, stable order (creation order).
#[test]
#[serial]
fn two_subscriptions_same_topic_yield_two_entries() {
    unsafe {
        let suffix = unique_suffix();
        let unique = format!("EpMS{suffix}");
        let ts = ep_point_ts(&unique);
        let (node, _opts) = setup_node(&format!("ep_multisub_node_{suffix}"));

        let topic = CString::new(format!("/rmw_ep/msub/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let sub_a = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        let sub_b = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!sub_a.is_null() && !sub_b.is_null());

        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_subscriptions_info_by_topic(
                node,
                &mut allocator,
                topic.as_ptr(),
                false,
                &mut arr,
            ),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 2, "two local subscriptions");
        let g0 = (*arr.info_array.add(0)).endpoint_gid;
        let g1 = (*arr.info_array.add(1)).endpoint_gid;
        assert_ne!(g0, g1, "distinct entity gids");
        assert_ne!(g0, [0u8; 16]);
        assert_ne!(g1, [0u8; 16]);

        assert_eq!(rmw_destroy_subscription(node, sub_a), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, sub_b), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

#[test]
#[serial]
fn destroy_removes_endpoint_entry() {
    unsafe {
        let suffix = unique_suffix();
        let ts = ep_point_ts(&format!("EpD{suffix}"));
        let (node, _opts) = setup_node(&format!("ep_del_node_{suffix}"));

        let topic = CString::new(format!("/rmw_ep/del/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_publishers_info_by_topic(node, &mut allocator, topic.as_ptr(), false, &mut arr,),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 1);

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);

        let mut arr2: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_publishers_info_by_topic(
                node,
                &mut allocator,
                topic.as_ptr(),
                false,
                &mut arr2,
            ),
            RMW_RET_OK
        );
        assert_eq!(arr2.size, 0, "destroy deregistered the endpoint");
        assert!(arr2.info_array.is_null());

        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

#[test]
#[serial]
fn null_args_rejected() {
    unsafe {
        let suffix = unique_suffix();
        let (node, _opts) = setup_node(&format!("ep_null_node_{suffix}"));
        let topic = CString::new(format!("/rmw_ep/null/{suffix}")).expect("topic");
        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();

        // Null node.
        assert_eq!(
            rmw_get_publishers_info_by_topic(
                std::ptr::null(),
                &mut allocator,
                topic.as_ptr(),
                false,
                &mut arr,
            ),
            ffi::RMW_RET_INVALID_ARGUMENT
        );
        // Null out-array.
        assert_eq!(
            rmw_get_publishers_info_by_topic(
                node,
                &mut allocator,
                topic.as_ptr(),
                false,
                std::ptr::null_mut(),
            ),
            ffi::RMW_RET_INVALID_ARGUMENT
        );
        // Null allocator (the real fill needs it).
        assert_eq!(
            rmw_get_subscriptions_info_by_topic(
                node,
                std::ptr::null_mut(),
                topic.as_ptr(),
                false,
                &mut arr,
            ),
            ffi::RMW_RET_INVALID_ARGUMENT
        );
        // Null topic name.
        assert_eq!(
            rmw_get_subscriptions_info_by_topic(
                node,
                &mut allocator,
                std::ptr::null(),
                false,
                &mut arr,
            ),
            ffi::RMW_RET_INVALID_ARGUMENT
        );

        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// OOM rollback: the partial-fill teardown in
// fill_endpoint_info_array is the diff's most hazardous unsafe path and
// was structurally unreachable with a real-malloc fixture. The
// fault-injecting allocator makes both failure sites reachable.
// =====================================================================

/// Build a 2-endpoint topic (two subscriptions — publishers are capped
/// single-writer per topic) and return everything the OOM tests need.
unsafe fn setup_two_subscription_topic(
    suffix: u64,
    tag: &str,
) -> (
    *mut ffi::rmw_node_t,
    CString,
    *mut ffi::rmw_subscription_t,
    *mut ffi::rmw_subscription_t,
) {
    let ts = ep_point_ts(&format!("Ep{tag}{suffix}"));
    let (node, _opts) = setup_node(&format!("ep_{tag}_node_{suffix}"));
    let topic = CString::new(format!("/rmw_ep/{tag}/{suffix}")).expect("topic");
    let qos = default_qos();
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    let sub_a = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
    let sub_b = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
    assert!(!sub_a.is_null() && !sub_b.is_null());
    (node, topic, sub_a, sub_b)
}

/// (i) The ARRAY allocation fails (budget 0 — the very first allocation
/// in the fill is the info array): BAD_ALLOC, the out-param stays a
/// clean empty array (size 0 / null), and the ledger shows zero
/// allocations and zero frees (nothing to leak, nothing freed twice).
#[test]
#[serial]
fn oom_at_array_alloc_returns_bad_alloc_with_clean_out_param() {
    unsafe {
        let suffix = unique_suffix();
        let (node, topic, sub_a, sub_b) = setup_two_subscription_topic(suffix, "ooma");

        let state = FaultAllocState::with_budget(0);
        let mut allocator = fault_allocator(&state);
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_subscriptions_info_by_topic(
                node,
                &mut allocator,
                topic.as_ptr(),
                false,
                &mut arr,
            ),
            ffi::RMW_RET_BAD_ALLOC
        );
        assert_eq!(arr.size, 0, "failed fill must leave a clean empty array");
        assert!(arr.info_array.is_null());
        assert!(
            state.allocated.lock().expect("ledger").is_empty(),
            "budget 0: the array allocation itself must have been refused"
        );
        assert!(
            state.freed.lock().expect("ledger").is_empty(),
            "nothing succeeded, so the rollback must free nothing"
        );
        state.assert_no_leak_no_double_free();

        assert_eq!(rmw_destroy_subscription(node, sub_a), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, sub_b), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// (ii) A strdup fails PARTWAY through an INTERIOR slot. Allocation
/// order in `fill_endpoint_info_array` is
/// `[array][slot0: name, ns, type][slot1: name, ns, type]`, so budget 5
/// = array + slot 0 complete + slot 1's node_name; slot 1's
/// node_namespace hits the injected null. The rollback must free the
/// failed slot's succeeded string AND all of slot 0's strings AND the
/// array — each exactly once — and the out-param must stay zeroed.
#[test]
#[serial]
fn oom_mid_slot_rolls_back_every_succeeded_allocation_exactly_once() {
    unsafe {
        let suffix = unique_suffix();
        let (node, topic, sub_a, sub_b) = setup_two_subscription_topic(suffix, "oomb");

        let state = FaultAllocState::with_budget(5);
        let mut allocator = fault_allocator(&state);
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_subscriptions_info_by_topic(
                node,
                &mut allocator,
                topic.as_ptr(),
                false,
                &mut arr,
            ),
            ffi::RMW_RET_BAD_ALLOC
        );
        assert_eq!(arr.size, 0, "failed fill must leave a clean empty array");
        assert!(arr.info_array.is_null());
        // The budget was fully consumed at the documented sites: array +
        // 3 slot-0 strings + slot-1 node_name (pins the interior-slot
        // partial-fill shape, not some earlier failure).
        assert_eq!(
            state.allocated.lock().expect("ledger").len(),
            5,
            "exactly the array, slot 0's three strings, and slot 1's first \
             string must have succeeded before the injected OOM"
        );
        state.assert_no_leak_no_double_free();

        assert_eq!(rmw_destroy_subscription(node, sub_a), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, sub_b), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Remove-by-gid exactness (the cheap sibling pin): with TWO
/// subscriptions on one topic, destroying ONE by its rmw object must
/// remove exactly THAT endpoint's record — the sibling survives with
/// its own gid. (A remove-first/remove-any regression would leave the
/// destroyed endpoint's gid behind instead.)
#[test]
#[serial]
fn destroying_one_subscription_leaves_sibling_record_intact() {
    unsafe {
        let suffix = unique_suffix();
        let (node, topic, sub_a, sub_b) = setup_two_subscription_topic(suffix, "sib");

        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_subscriptions_info_by_topic(
                node,
                &mut allocator,
                topic.as_ptr(),
                false,
                &mut arr,
            ),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 2);
        // Records are pushed in creation order: index 0 = sub_a, 1 = sub_b.
        let gid_a = (*arr.info_array.add(0)).endpoint_gid;
        let gid_b = (*arr.info_array.add(1)).endpoint_gid;
        assert_ne!(gid_a, gid_b);

        // Destroy sub_a: exactly its record must go.
        assert_eq!(rmw_destroy_subscription(node, sub_a), RMW_RET_OK);
        let mut arr2: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_subscriptions_info_by_topic(
                node,
                &mut allocator,
                topic.as_ptr(),
                false,
                &mut arr2,
            ),
            RMW_RET_OK
        );
        assert_eq!(arr2.size, 1, "sibling endpoint must survive");
        let survivor = (*arr2.info_array).endpoint_gid;
        assert_eq!(survivor, gid_b, "the SURVIVING record must be sub_b's");
        assert_ne!(survivor, gid_a, "sub_a's record must be the one removed");

        assert_eq!(rmw_destroy_subscription(node, sub_b), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
