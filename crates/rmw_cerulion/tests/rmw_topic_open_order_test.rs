// SPDX-License-Identifier: AGPL-3.0-only
#![cfg(unix)]
//! A topic's iceoryx2 service parameters must not depend on which endpoint
//! opens the topic first (issue 617). The first opener creates the service;
//! a later opener that needs a deeper history or a larger subscriber buffer
//! ceiling used to be refused with `DoesNotSupportRequestedMinHistorySize` or
//! `DoesNotSupportRequestedMinBufferSize`. Each open-order test opens the
//! topic with the smaller endpoint first, then a TRANSIENT_LOCAL depth-16
//! publisher, and requires the second open to succeed.
//!
//! The late-joiner arms pin what a late subscription takes from the retained
//! messages, by its durability: VOLATILE none, TRANSIENT_LOCAL up to its depth
//! and the topic's history limit (16, or 12 for a type above the 16 MiB slice
//! tier), and BEST_AVAILABLE as TRANSIENT_LOCAL only when every publisher on
//! the topic retains, in this process or another. The cross-process arms
//! re-execute this binary as the publishing child
//! (`subprocess_child_publishes_then_waits`, `#[ignore]`d and inert without
//! its environment). The BEST_AVAILABLE arms build only where the rmw headers
//! declare that durability (Iron and later).
//!
//! Shares the iceoryx2 SHM singleton, so run serial:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_topic_open_order_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};

use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;

const DEEP: usize = 16;
const ROS_TYPE_DOUBLE: u8 = 2;

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

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct CPoint {
    x: f64,
    y: f64,
    z: f64,
}

fn point_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    let members = Box::leak(
        vec![
            member("x", ROS_TYPE_DOUBLE, 0),
            member("y", ROS_TYPE_DOUBLE, 8),
            member("z", ROS_TYPE_DOUBLE, 16),
        ]
        .into_boxed_slice(),
    );
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("rmw_open_order__msg"),
            message_name_: cstr(unique),
            member_count_: members.len() as u32,
            size_of_: std::mem::size_of::<CPoint>(),
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

static UNIQUE: AtomicU64 = AtomicU64::new(0);

fn unique_suffix() -> u64 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos() as u64;
    nanos ^ UNIQUE.fetch_add(1, Ordering::Relaxed)
}

unsafe fn setup_node(name: &str) -> (*mut ffi::rmw_context_t, *mut ffi::rmw_node_t) {
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    Box::leak(options);
    let node = rmw_create_node(context, cstr(name), cstr("/"));
    assert!(!node.is_null(), "node creation failed");
    (context, node)
}

fn qos(durability: u32, depth: usize) -> ffi::rmw_qos_profile_t {
    ffi::rmw_qos_profile_t {
        history: ffi::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth,
        reliability: ffi::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        durability: durability as _,
        deadline: ffi::rmw_time_t { sec: 0, nsec: 0 },
        lifespan: ffi::rmw_time_t { sec: 0, nsec: 0 },
        liveliness: ffi::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
        liveliness_lease_duration: ffi::rmw_time_t { sec: 0, nsec: 0 },
        avoid_ros_namespace_conventions: false,
    }
}

fn transient_local(depth: usize) -> ffi::rmw_qos_profile_t {
    qos(ffi::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, depth)
}

fn volatile(depth: usize) -> ffi::rmw_qos_profile_t {
    qos(ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE, depth)
}

enum First {
    Subscription(ffi::rmw_qos_profile_t),
    Publisher(ffi::rmw_qos_profile_t),
}

/// Opens `/rmw_open_order/<tag>/<unique>` with `first`, then opens a
/// TRANSIENT_LOCAL depth-16 publisher on it and returns whether that second
/// open produced a publisher.
unsafe fn second_open_succeeds(tag: &str, first: First) -> bool {
    let suffix = unique_suffix();
    let ts = point_ts(&format!("Oo{suffix}"));
    let (_, node) = setup_node(&format!("open_order_{tag}_{suffix}"));
    let topic = CString::new(format!("/rmw_open_order/{tag}/{suffix}")).expect("topic");
    let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    let (first_pub, first_sub) = match first {
        First::Subscription(q) => (
            std::ptr::null_mut(),
            rmw_create_subscription(node, ts, topic.as_ptr(), &q, &sub_opts),
        ),
        First::Publisher(q) => (
            rmw_create_publisher(node, ts, topic.as_ptr(), &q, &pub_opts),
            std::ptr::null_mut(),
        ),
    };
    assert!(
        !first_pub.is_null() || !first_sub.is_null(),
        "the FIRST opener must create the topic"
    );
    let deep = rmw_create_publisher(node, ts, topic.as_ptr(), &transient_local(DEEP), &pub_opts);
    let ok = !deep.is_null();
    if ok {
        assert_eq!(rmw_destroy_publisher(node, deep), RMW_RET_OK);
    }
    if !first_pub.is_null() {
        assert_eq!(rmw_destroy_publisher(node, first_pub), RMW_RET_OK);
    }
    if !first_sub.is_null() {
        assert_eq!(rmw_destroy_subscription(node, first_sub), RMW_RET_OK);
    }
    assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    ok
}

#[test]
#[serial]
fn a_subscription_that_opens_first_does_not_lock_out_a_deep_transient_local_publisher() {
    unsafe {
        assert!(
            second_open_succeeds("sub_first", First::Subscription(volatile(10))),
            "a VOLATILE subscription created the service at history 0; the later \
             TRANSIENT_LOCAL depth-16 publisher must still open it"
        );
    }
}

#[test]
#[serial]
fn a_shallow_transient_local_publisher_that_opens_first_does_not_lock_out_a_deeper_one() {
    unsafe {
        assert!(
            second_open_succeeds("shallow_tl", First::Publisher(transient_local(1))),
            "a depth-1 TRANSIENT_LOCAL publisher created the service at history 1; \
             the later depth-16 publisher must still open it"
        );
    }
}

#[test]
#[serial]
fn a_volatile_publisher_that_opens_first_does_not_lock_out_a_deep_transient_local_publisher() {
    unsafe {
        assert!(
            second_open_succeeds("volatile_pub", First::Publisher(volatile(10))),
            "a VOLATILE publisher created the service at history 0; the later \
             TRANSIENT_LOCAL depth-16 publisher must still open it"
        );
    }
}

/// Waits on `subscription` through `rmw_wait` (which runs the publisher
/// history pump, as an executor does) for about one second, takes everything
/// it receives, and returns how many messages it got.
unsafe fn drain(
    context: *mut ffi::rmw_context_t,
    subscription: *mut ffi::rmw_subscription_t,
) -> usize {
    drain_points(context, subscription).len()
}

/// [`drain`], returning the `x` of every message taken, in take order.
unsafe fn drain_points(
    context: *mut ffi::rmw_context_t,
    subscription: *mut ffi::rmw_subscription_t,
) -> Vec<f64> {
    let ws = rmw_create_wait_set(context, 8);
    assert!(!ws.is_null(), "wait set");
    let mut received = Vec::new();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(1);
    while std::time::Instant::now() < deadline {
        let mut sub_ptrs = [(*subscription).data];
        let mut subs = ffi::rmw_subscriptions_t {
            subscriber_count: 1,
            subscribers: sub_ptrs.as_mut_ptr(),
        };
        let timeout = ffi::rmw_time_t {
            sec: 0,
            nsec: 50_000_000,
        };
        let _ = rmw_wait(
            &mut subs,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &timeout,
        );
        loop {
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
            if !taken {
                break;
            }
            received.push(out.x);
        }
    }
    assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
    received
}

/// Creates the topic with a VOLATILE subscription, publishes `SENT` messages
/// from a publisher with `pub_qos`, then creates a late subscription with
/// `late_qos` and returns how many retained messages it received.
unsafe fn late_joiner_receives(
    tag: &str,
    pub_qos: ffi::rmw_qos_profile_t,
    late_qos: ffi::rmw_qos_profile_t,
) -> usize {
    const SENT: usize = 8;
    let suffix = unique_suffix();
    let ts = point_ts(&format!("Lj{suffix}"));
    let (context, node) = setup_node(&format!("late_joiner_{tag}_{suffix}"));
    let topic = CString::new(format!("/rmw_open_order/late/{tag}/{suffix}")).expect("topic");
    let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    let creator = rmw_create_subscription(node, ts, topic.as_ptr(), &volatile(10), &sub_opts);
    assert!(
        !creator.is_null(),
        "the VOLATILE subscription creates the topic"
    );
    let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &pub_qos, &pub_opts);
    assert!(
        !publisher.is_null(),
        "the publisher opens the subscription-created topic"
    );
    for i in 0..SENT {
        let msg = CPoint {
            x: i as f64,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
    }
    let late = rmw_create_subscription(node, ts, topic.as_ptr(), &late_qos, &sub_opts);
    assert!(!late.is_null(), "the late subscription opens the topic");
    let received = drain(context, late);
    assert_eq!(rmw_destroy_subscription(node, late), RMW_RET_OK);
    assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
    assert_eq!(rmw_destroy_subscription(node, creator), RMW_RET_OK);
    assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    received
}

#[test]
#[serial]
fn a_late_transient_local_subscription_receives_the_backlog_on_a_subscription_created_topic() {
    unsafe {
        assert_eq!(
            late_joiner_receives("tl_tl", transient_local(10), transient_local(3)),
            3,
            "of the 8 messages a TRANSIENT_LOCAL publisher retained, a late \
             TRANSIENT_LOCAL depth-3 subscription receives exactly 3 (its own history \
             request), though a VOLATILE subscription created the topic; without the \
             request iceoryx2 would deliver all 8"
        );
    }
}

#[test]
#[serial]
fn a_late_volatile_subscription_receives_no_backlog_from_a_volatile_publisher() {
    unsafe {
        assert_eq!(
            late_joiner_receives("vol_vol", volatile(10), volatile(10)),
            0,
            "every rmw topic is created with a 16-deep history, so a late VOLATILE \
             subscription must ask for none or it replays a VOLATILE publisher's \
             old messages"
        );
    }
}

#[test]
#[serial]
fn a_late_volatile_subscription_receives_no_backlog_from_a_transient_local_publisher() {
    unsafe {
        assert_eq!(
            late_joiner_receives("tl_vol", transient_local(10), volatile(10)),
            0,
            "DDS durability: a VOLATILE reader takes no historical samples, even \
             from a TRANSIENT_LOCAL writer"
        );
    }
}

/// How a native (non-rmw) subscriber attaches in [`native_late_joiner_receives`]:
/// the graph-input and Python path, or the open-only path `cerulion topic
/// echo` and `hz` use.
#[derive(Clone, Copy)]
enum Native {
    WithBuffers,
    OpenOnly,
}

/// Publishes `RETAINED` messages from a VOLATILE rmw publisher on a topic it
/// creates (or that `native_history` > 0 pre-created natively with that much
/// history), attaches a native subscriber, publishes one LIVE message (its
/// send delivers any retained history first), and returns how many messages
/// the native subscriber received.
unsafe fn native_late_joiner_receives(tag: &str, native: Native, native_history: usize) -> usize {
    const RETAINED: usize = 4;
    let suffix = unique_suffix();
    let ts = point_ts(&format!("Nl{suffix}"));
    let (_, node) = setup_node(&format!("native_late_{tag}_{suffix}"));
    let topic_str = format!("/rmw_open_order/native/{tag}/{suffix}");
    let topic = CString::new(topic_str.clone()).expect("topic");
    let transport = &rmw_cerulion::runtime::runtime().expect("runtime").transport;
    // A natively created topic with declared history: a core publisher creates
    // the service, a holder keeps it alive, and the core publisher goes away
    // so the rmw publisher below is the only writer.
    let _holder = if native_history > 0 {
        let msl = cerulion_core::wire::MaxSliceLen::try_new(4096).expect("slice len");
        let creator = transport
            .create_publisher(&topic_str, msl, native_history)
            .expect("native creator");
        let holder = transport
            .create_subscriber_open_only(&topic_str)
            .expect("service holder");
        drop(creator);
        Some(holder)
    } else {
        None
    };
    let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
    let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &volatile(10), &pub_opts);
    assert!(
        !publisher.is_null(),
        "the VOLATILE rmw publisher opens the topic"
    );
    let send = |i: usize| {
        let msg = CPoint {
            x: i as f64,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
    };
    for i in 0..RETAINED {
        send(i);
    }
    let subscriber = match native {
        Native::WithBuffers => transport
            .create_subscriber_with_buffers(&topic_str, transport.default_topic_config(), 16)
            .expect("native subscriber"),
        Native::OpenOnly => transport
            .create_subscriber_open_only(&topic_str)
            .expect("native open-only subscriber"),
    };
    send(RETAINED);
    let mut received = 0;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(1);
    while std::time::Instant::now() < deadline {
        received += subscriber.try_receive(|_| {}).expect("receive");
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    drop(subscriber);
    assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
    assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    received
}

#[test]
#[serial]
fn a_native_subscriber_receives_no_backlog_from_a_volatile_rmw_publisher() {
    unsafe {
        assert_eq!(
            native_late_joiner_receives("with_buffers", Native::WithBuffers, 0),
            1,
            "a native subscriber (graph input, Python) that attaches after a VOLATILE \
             rmw publisher sent 4 messages receives only the live one: the rmw-created \
             service's history is the topic's limit, not a retention the publisher asked for"
        );
    }
}

#[test]
#[serial]
fn an_open_only_native_subscriber_receives_no_backlog_from_a_volatile_rmw_publisher() {
    unsafe {
        assert_eq!(
            native_late_joiner_receives("open_only", Native::OpenOnly, 0),
            1,
            "`cerulion topic echo` and `hz` attach open-only; after a VOLATILE rmw \
             publisher sent 4 messages they receive only the live one"
        );
    }
}

#[test]
#[serial]
fn a_native_subscriber_still_receives_history_a_native_creator_declared() {
    unsafe {
        assert_eq!(
            native_late_joiner_receives("native_history", Native::OpenOnly, 4),
            5,
            "on a topic a native creator made with history 4, a native late joiner still \
             receives the 4 retained messages and the live one (graph latched topics keep \
             their late-joiner replay)"
        );
    }
}

// =====================================================================
// History limit 12: a variable type above the 16 MiB slice tier.
// =====================================================================

const ROS_TYPE_STRING: u8 = 16;

#[repr(C)]
struct CRosString {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

/// A message with a string field: a variable layout. Its type name is in no
/// slice ceiling table, so it takes the 128 MiB catch-all, above the 16 MiB
/// tier, and the topic's history limit is 12.
#[repr(C)]
struct CLabeled {
    id: f64,
    label: CRosString,
}

fn labeled_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    let members = Box::leak(
        vec![
            member("id", ROS_TYPE_DOUBLE, 0),
            member("label", ROS_TYPE_STRING, 8),
        ]
        .into_boxed_slice(),
    );
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("rmw_open_order__msg"),
            message_name_: cstr(unique),
            member_count_: members.len() as u32,
            size_of_: std::mem::size_of::<CLabeled>(),
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

/// Takes every `CLabeled` the subscription receives within about one second
/// and returns its `(id, label)` pairs in take order.
unsafe fn drain_labeled(
    context: *mut ffi::rmw_context_t,
    subscription: *mut ffi::rmw_subscription_t,
) -> Vec<(f64, String)> {
    let ws = rmw_create_wait_set(context, 8);
    assert!(!ws.is_null(), "wait set");
    let mut received = Vec::new();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(1);
    while std::time::Instant::now() < deadline {
        let mut sub_ptrs = [(*subscription).data];
        let mut subs = ffi::rmw_subscriptions_t {
            subscriber_count: 1,
            subscribers: sub_ptrs.as_mut_ptr(),
        };
        let timeout = ffi::rmw_time_t {
            sec: 0,
            nsec: 50_000_000,
        };
        let _ = rmw_wait(
            &mut subs,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &timeout,
        );
        loop {
            let mut out = CLabeled {
                id: -1.0,
                label: CRosString {
                    data: std::ptr::null_mut(),
                    size: 0,
                    capacity: 0,
                },
            };
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
            if !taken {
                break;
            }
            let label = if out.label.data.is_null() {
                String::new()
            } else {
                let bytes = std::slice::from_raw_parts(out.label.data, out.label.size);
                let text = String::from_utf8_lossy(bytes).into_owned();
                libc::free(out.label.data as *mut c_void);
                text
            };
            received.push((out.id, label));
        }
    }
    assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
    received
}

unsafe extern "C" fn fixture_allocate(size: usize, _state: *mut c_void) -> *mut c_void {
    libc::malloc(size)
}

unsafe extern "C" fn fixture_deallocate(ptr: *mut c_void, _state: *mut c_void) {
    libc::free(ptr)
}

/// A malloc-backed allocator for the endpoint info out-array.
fn malloc_allocator() -> ffi::rcutils_allocator_t {
    let mut allocator: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
    allocator.allocate = Some(fixture_allocate);
    allocator.deallocate = Some(fixture_deallocate);
    allocator
}

/// The depth and durability `rmw_get_publishers_info_by_topic` reports for
/// the topic's only publisher.
unsafe fn only_publisher_depth_and_durability(
    node: *mut ffi::rmw_node_t,
    topic: &CString,
) -> (usize, u32) {
    let mut allocator = malloc_allocator();
    let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
    assert_eq!(
        rmw_get_publishers_info_by_topic(node, &mut allocator, topic.as_ptr(), false, &mut arr),
        RMW_RET_OK
    );
    assert_eq!(arr.size, 1, "the topic has exactly one publisher");
    let profile = (*arr.info_array).qos_profile;
    (profile.depth, profile.durability)
}

#[test]
#[serial]
fn a_large_variable_type_topic_opened_by_a_subscription_retains_twelve_messages() {
    const SENT: usize = 14;
    unsafe {
        let suffix = unique_suffix();
        let ts = labeled_ts(&format!("Big{suffix}"));
        let (context, node) = setup_node(&format!("large_history_{suffix}"));
        let topic = CString::new(format!("/rmw_open_order/large/{suffix}")).expect("topic");
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let creator = rmw_create_subscription(node, ts, topic.as_ptr(), &volatile(10), &sub_opts);
        assert!(
            !creator.is_null(),
            "the VOLATILE subscription creates the topic"
        );
        let publisher =
            rmw_create_publisher(node, ts, topic.as_ptr(), &transient_local(DEEP), &pub_opts);
        assert!(
            !publisher.is_null(),
            "a TRANSIENT_LOCAL depth-16 publisher must open a large-type topic a \
             subscription created"
        );
        assert_eq!(
            only_publisher_depth_and_durability(node, &topic),
            (12, ffi::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
            "above the 16 MiB slice tier the topic's history limit is 12, so the \
             depth-16 publisher reports its depth clamped to 12"
        );
        for i in 0..SENT {
            let text = format!("m{i}");
            let buf = libc::malloc(text.len() + 1) as *mut u8;
            assert!(!buf.is_null(), "label buffer");
            std::ptr::copy_nonoverlapping(text.as_ptr(), buf, text.len());
            *buf.add(text.len()) = 0;
            let msg = CLabeled {
                id: i as f64,
                label: CRosString {
                    data: buf,
                    size: text.len(),
                    capacity: text.len() + 1,
                },
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            libc::free(buf as *mut c_void);
        }
        let late =
            rmw_create_subscription(node, ts, topic.as_ptr(), &transient_local(DEEP), &sub_opts);
        assert!(!late.is_null(), "the late subscription opens the topic");
        let received = drain_labeled(context, late);
        let want: Vec<(f64, String)> = (2..14).map(|i| (i as f64, format!("m{i}"))).collect();
        assert_eq!(
            received, want,
            "of the 14 messages sent, a late TRANSIENT_LOCAL depth-16 subscription \
             receives exactly the newest 12 (ids 2 to 13, labels intact): the large \
             type's history limit, not 16"
        );
        assert_eq!(rmw_destroy_subscription(node, late), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, creator), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// BEST_AVAILABLE durability (Iron and later headers only).
// =====================================================================

/// The `x` the publisher of a late-joiner arm sends AFTER the late
/// subscription took what was retained: it can only arrive live, so it is
/// the nonzero control beside every list of retained messages.
#[cfg(cerulion_has_durability_best_available)]
const LIVE_X: f64 = 1000.0;

#[cfg(cerulion_has_durability_best_available)]
fn best_available(depth: usize) -> ffi::rmw_qos_profile_t {
    qos(ffi::RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE, depth)
}

/// `start, .., end - 1` as message `x` values.
#[cfg(cerulion_has_durability_best_available)]
fn xs(range: std::ops::Range<usize>) -> Vec<f64> {
    range.map(|i| i as f64).collect()
}

/// `values`, sorted (two publishers' retained messages interleave in no
/// fixed order).
#[cfg(cerulion_has_durability_best_available)]
fn sorted(mut values: Vec<f64>) -> Vec<f64> {
    values.sort_by(f64::total_cmp);
    values
}

/// What a late subscription took in a late-joiner arm.
#[cfg(cerulion_has_durability_best_available)]
#[derive(Debug, PartialEq)]
struct LateTake {
    /// The `x` values taken before the live message was sent, in take order.
    retained: Vec<f64>,
    /// The `x` values taken after it.
    live: Vec<f64>,
}

/// Creates one publisher per entry of `publishers` on a fresh topic; each
/// sends `sent` messages (`x` = 0 to `sent - 1`). Then a subscription with
/// `late_qos` opens and takes what reaches it within about one second, the
/// FIRST publisher sends one live message (`x` = [`LIVE_X`]), and the
/// subscription takes again. Also returns the depth endpoint info reports for
/// each publisher.
#[cfg(cerulion_has_durability_best_available)]
unsafe fn late_subscription_takes(
    tag: &str,
    publishers: &[ffi::rmw_qos_profile_t],
    sent: usize,
    late_qos: ffi::rmw_qos_profile_t,
) -> (LateTake, Vec<usize>) {
    let suffix = unique_suffix();
    let ts = point_ts(&format!("Ba{suffix}"));
    let (context, node) = setup_node(&format!("best_available_{tag}_{suffix}"));
    let topic =
        CString::new(format!("/rmw_open_order/best_available/{tag}/{suffix}")).expect("topic");
    let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    let publish = |publisher: *mut ffi::rmw_publisher_t, x: f64| {
        let msg = CPoint { x, y: 0.0, z: 0.0 };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
    };
    let mut handles = Vec::new();
    for q in publishers {
        let p = rmw_create_publisher(node, ts, topic.as_ptr(), q, &pub_opts);
        assert!(!p.is_null(), "publisher {} opens the topic", handles.len());
        handles.push(p);
    }
    for &p in &handles {
        for i in 0..sent {
            publish(p, i as f64);
        }
    }
    let mut allocator = malloc_allocator();
    let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
    assert_eq!(
        rmw_get_publishers_info_by_topic(node, &mut allocator, topic.as_ptr(), false, &mut arr),
        RMW_RET_OK
    );
    let depths = (0..arr.size)
        .map(|i| (*arr.info_array.add(i)).qos_profile.depth)
        .collect();
    let late = rmw_create_subscription(node, ts, topic.as_ptr(), &late_qos, &sub_opts);
    assert!(!late.is_null(), "the late subscription opens the topic");
    let retained = drain_points(context, late);
    publish(handles[0], LIVE_X);
    let live = drain_points(context, late);
    assert_eq!(rmw_destroy_subscription(node, late), RMW_RET_OK);
    for p in handles {
        assert_eq!(rmw_destroy_publisher(node, p), RMW_RET_OK);
    }
    assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    (LateTake { retained, live }, depths)
}

#[cfg(cerulion_has_durability_best_available)]
#[test]
#[serial]
fn a_late_best_available_subscription_takes_a_transient_local_backlog() {
    unsafe {
        let (taken, _) =
            late_subscription_takes("tl", &[transient_local(DEEP)], 20, best_available(5));
        assert_eq!(
            taken,
            LateTake {
                retained: xs(15..20),
                live: vec![LIVE_X],
            },
            "every publisher on the topic offers TRANSIENT_LOCAL, so BEST_AVAILABLE \
             resolves TRANSIENT_LOCAL: of 20 retained messages the depth-5 subscription \
             takes the newest 5, then the live one"
        );
    }
}

#[cfg(cerulion_has_durability_best_available)]
#[test]
#[serial]
fn a_best_available_subscription_deeper_than_the_limit_takes_the_limit() {
    unsafe {
        let (taken, _) =
            late_subscription_takes("deep", &[transient_local(DEEP)], 20, best_available(1000));
        assert_eq!(
            taken,
            LateTake {
                retained: xs(4..20),
                live: vec![LIVE_X],
            },
            "a resolved TRANSIENT_LOCAL depth 1000 is clamped to the topic's limit of \
             16: the newest 16 of 20, then the live one"
        );
    }
}

#[cfg(cerulion_has_durability_best_available)]
#[test]
#[serial]
fn a_late_best_available_subscription_takes_no_backlog_when_a_volatile_publisher_is_on_the_topic() {
    unsafe {
        let (taken, _) = late_subscription_takes(
            "mixed",
            &[transient_local(DEEP), volatile(10)],
            4,
            best_available(10),
        );
        assert_eq!(
            taken,
            LateTake {
                retained: Vec::new(),
                live: vec![LIVE_X],
            },
            "one publisher on the topic is VOLATILE, so BEST_AVAILABLE resolves VOLATILE: \
             no retained message, only the live one"
        );
    }
}

#[cfg(cerulion_has_durability_best_available)]
#[test]
#[serial]
fn a_late_transient_local_subscription_on_the_mixed_topic_takes_both_rings() {
    unsafe {
        let (taken, _) = late_subscription_takes(
            "mixed_tl",
            &[transient_local(DEEP), volatile(10)],
            4,
            transient_local(10),
        );
        assert_eq!(
            sorted(taken.retained),
            vec![0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0],
            "control for the BEST_AVAILABLE zero: on the same topic shape a \
             TRANSIENT_LOCAL subscription takes the 4 messages each publisher's ring \
             holds, so the rings were not empty"
        );
        assert_eq!(taken.live, vec![LIVE_X]);
    }
}

#[cfg(cerulion_has_durability_best_available)]
#[test]
#[serial]
fn a_best_available_publisher_retains_like_a_transient_local_one() {
    unsafe {
        let want = LateTake {
            retained: xs(0..8),
            live: vec![LIVE_X],
        };
        let (to_tl, depths) =
            late_subscription_takes("pub_tl", &[best_available(4)], 8, transient_local(10));
        assert_eq!(
            depths,
            vec![4],
            "a BEST_AVAILABLE publisher resolves TRANSIENT_LOCAL and reports its own \
             depth, 4, not the 0 of a VOLATILE publisher"
        );
        assert_eq!(
            to_tl, want,
            "a late TRANSIENT_LOCAL depth-10 subscription takes the 8 retained \
             messages, then the live one (as from any publisher on an rmw-created topic; \
             the depth above and the BEST_AVAILABLE arm below are what pin the publisher)"
        );
        let (to_ba, _) =
            late_subscription_takes("pub_ba", &[best_available(4)], 8, best_available(10));
        assert_eq!(
            to_ba, want,
            "the BEST_AVAILABLE publisher carries the retaining mark, so a late \
             BEST_AVAILABLE subscription resolves TRANSIENT_LOCAL and takes the same"
        );
    }
}

#[cfg(cerulion_has_durability_best_available)]
#[test]
#[serial]
fn best_available_resolution_delivers_the_same_messages_on_every_run() {
    unsafe {
        let first =
            late_subscription_takes("again_a", &[transient_local(DEEP)], 20, best_available(5));
        let second =
            late_subscription_takes("again_b", &[transient_local(DEEP)], 20, best_available(5));
        assert_eq!(first, second, "two runs of one arm take identical messages");
    }
}

// =====================================================================
// BEST_AVAILABLE across processes: the publisher lives in a child process
// (this binary re-executed), the late subscription in this one.
// =====================================================================

/// Set to "1" ONLY on the spawned child, so a bare `--ignored` run no-ops.
#[cfg(cerulion_has_durability_best_available)]
const ENV_CHILD: &str = "CER_OPEN_ORDER_CHILD";
#[cfg(cerulion_has_durability_best_available)]
const ENV_TOPIC: &str = "CER_OPEN_ORDER_TOPIC";
#[cfg(cerulion_has_durability_best_available)]
const ENV_TYPE: &str = "CER_OPEN_ORDER_TYPE";
/// `transient_local` or `volatile`.
#[cfg(cerulion_has_durability_best_available)]
const ENV_DURABILITY: &str = "CER_OPEN_ORDER_DURABILITY";
#[cfg(cerulion_has_durability_best_available)]
const ENV_SENT: &str = "CER_OPEN_ORDER_SENT";
#[cfg(cerulion_has_durability_best_available)]
const CHILD_TEST: &str = "subprocess_child_publishes_then_waits";
#[cfg(cerulion_has_durability_best_available)]
const CHILD_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(120);
/// Child to parent, one per stdout line: the backlog is sent; the live
/// message is sent; the publisher and node are destroyed.
#[cfg(cerulion_has_durability_best_available)]
const READY: &str = "OPEN_ORDER_CHILD_READY";
#[cfg(cerulion_has_durability_best_available)]
const SENT_LIVE: &str = "OPEN_ORDER_CHILD_SENT_LIVE";
#[cfg(cerulion_has_durability_best_available)]
const DONE: &str = "OPEN_ORDER_CHILD_DONE";

/// THE CHILD: opens the topic with one publisher, sends the backlog, then
/// keeps its publisher pumped through `rmw_wait` (as an executor does) and
/// obeys `LIVE` (send one live message) and `EXIT` on stdin.
#[cfg(cerulion_has_durability_best_available)]
#[test]
#[ignore]
fn subprocess_child_publishes_then_waits() {
    use std::io::BufRead;
    if std::env::var(ENV_CHILD).as_deref() != Ok("1") {
        return;
    }
    let topic = std::env::var(ENV_TOPIC).expect("the parent sets the topic");
    let type_name = std::env::var(ENV_TYPE).expect("the parent sets the type name");
    let sent: usize = std::env::var(ENV_SENT)
        .expect("the parent sets the count")
        .parse()
        .expect("count");
    let pub_qos = match std::env::var(ENV_DURABILITY).as_deref() {
        Ok("transient_local") => transient_local(DEEP),
        Ok("volatile") => volatile(10),
        other => panic!("unknown durability {other:?}"),
    };
    let (lines_tx, lines_rx) = std::sync::mpsc::channel::<String>();
    std::thread::spawn(move || {
        for line in std::io::stdin().lock().lines() {
            let Ok(line) = line else { break };
            if lines_tx.send(line).is_err() {
                break;
            }
        }
    });
    unsafe {
        let ts = point_ts(&type_name);
        let (context, node) = setup_node(&format!("open_order_child_{}", std::process::id()));
        let topic = CString::new(topic).expect("topic");
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &pub_qos, &pub_opts);
        assert!(!publisher.is_null(), "child publisher");
        let publish = |x: f64| {
            let msg = CPoint { x, y: 0.0, z: 0.0 };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
        };
        for i in 0..sent {
            publish(i as f64);
        }
        println!("{READY}");
        let ws = rmw_create_wait_set(context, 4);
        assert!(!ws.is_null(), "wait set");
        let timeout = ffi::rmw_time_t {
            sec: 0,
            nsec: 20_000_000,
        };
        let deadline = std::time::Instant::now() + CHILD_TIMEOUT;
        loop {
            assert!(
                std::time::Instant::now() < deadline,
                "the parent never said EXIT"
            );
            let _ = rmw_wait(
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                ws,
                &timeout,
            );
            match lines_rx.try_recv().as_deref() {
                Ok("LIVE") => {
                    publish(LIVE_X);
                    println!("{SENT_LIVE}");
                }
                Ok("EXIT") => break,
                Ok(other) => panic!("unknown parent command {other:?}"),
                Err(std::sync::mpsc::TryRecvError::Empty) => {}
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    panic!("the parent closed stdin without EXIT")
                }
            }
        }
        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
        println!("{DONE}");
    }
}

/// The parent's view of the child, fed line by line from its stdout.
#[cfg(cerulion_has_durability_best_available)]
struct Child {
    process: std::process::Child,
    stdin: std::process::ChildStdin,
    lines: std::sync::mpsc::Receiver<String>,
}

#[cfg(cerulion_has_durability_best_available)]
impl Child {
    fn spawn(topic: &str, type_name: &str, durability: &str, sent: usize) -> Self {
        use std::io::BufRead;
        let exe = std::env::current_exe().expect("current_exe");
        let mut process = std::process::Command::new(exe)
            .args([
                "--exact",
                CHILD_TEST,
                "--ignored",
                "--nocapture",
                "--test-threads=1",
            ])
            .env(ENV_CHILD, "1")
            .env(ENV_TOPIC, topic)
            .env(ENV_TYPE, type_name)
            .env(ENV_DURABILITY, durability)
            .env(ENV_SENT, sent.to_string())
            .stdin(std::process::Stdio::piped())
            .stdout(std::process::Stdio::piped())
            .spawn()
            .expect("spawn child");
        let stdin = process.stdin.take().expect("child stdin");
        let stdout = process.stdout.take().expect("child stdout");
        let (tx, lines) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            for line in std::io::BufReader::new(stdout).lines() {
                let Ok(line) = line else { break };
                if tx.send(line).is_err() {
                    break;
                }
            }
        });
        Self {
            process,
            stdin,
            lines,
        }
    }

    /// Waits for a stdout line CONTAINING `marker` (libtest prints
    /// `test <name> ... ` without a newline before the body runs, so the first
    /// marker shares that line).
    fn expect_line(&mut self, marker: &str) {
        let deadline = std::time::Instant::now() + CHILD_TIMEOUT;
        loop {
            let left = deadline.saturating_duration_since(std::time::Instant::now());
            match self.lines.recv_timeout(left) {
                Ok(line) if line.contains(marker) => return,
                Ok(_) => {}
                Err(e) => {
                    let _ = self.process.kill();
                    let _ = self.process.wait();
                    panic!("the child never printed {marker}: {e:?}");
                }
            }
        }
    }

    fn command(&mut self, command: &str) {
        use std::io::Write;
        writeln!(self.stdin, "{command}").expect("write to child");
        self.stdin.flush().expect("flush child stdin");
    }

    fn finish(mut self) {
        self.command("EXIT");
        self.expect_line(DONE);
        let deadline = std::time::Instant::now() + CHILD_TIMEOUT;
        let status = loop {
            match self.process.try_wait().expect("try_wait") {
                Some(status) => break status,
                None if std::time::Instant::now() > deadline => {
                    let _ = self.process.kill();
                    let _ = self.process.wait();
                    panic!("the child did not exit within {CHILD_TIMEOUT:?}");
                }
                None => std::thread::sleep(std::time::Duration::from_millis(20)),
            }
        };
        assert!(status.success(), "the child failed: {status:?}");
    }
}

/// What the cross-process arms observe in THIS process.
#[cfg(cerulion_has_durability_best_available)]
struct CrossProcessTake {
    /// The late BEST_AVAILABLE depth-5 subscription.
    best_available: LateTake,
    /// A TRANSIENT_LOCAL depth-5 control subscription opened after it,
    /// taken before the live message: the newest 5 of what the child's ring
    /// held.
    control: Vec<f64>,
    /// The topic's retain marks while the child's publisher lived.
    marks_alive: Option<Vec<bool>>,
    /// The topic's retain marks after the child exited.
    marks_after: Option<Vec<bool>>,
}

/// A child process publishes `sent` messages with `durability` on a fresh
/// topic. Then a BEST_AVAILABLE depth-5 subscription opens in THIS process
/// and takes what reaches it, a TRANSIENT_LOCAL depth-5 control subscription
/// does the same, the child sends one live message, and the first
/// subscription takes again.
#[cfg(cerulion_has_durability_best_available)]
unsafe fn cross_process_best_available_takes(
    tag: &str,
    durability: &str,
    sent: usize,
) -> CrossProcessTake {
    let suffix = unique_suffix();
    let type_name = format!("Xp{suffix}");
    let topic_str = format!("/rmw_open_order/xproc/{tag}/{suffix}");
    let ts = point_ts(&type_name);
    let (context, node) = setup_node(&format!("open_order_parent_{tag}_{suffix}"));
    let mut child = Child::spawn(&topic_str, &type_name, durability, sent);
    child.expect_line(READY);
    let marks_alive = retain_marks(&topic_str);
    let topic = CString::new(topic_str.clone()).expect("topic");
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    let late = rmw_create_subscription(node, ts, topic.as_ptr(), &best_available(5), &sub_opts);
    assert!(
        !late.is_null(),
        "the late subscription opens the child's topic"
    );
    let retained = drain_points(context, late);
    let control_sub =
        rmw_create_subscription(node, ts, topic.as_ptr(), &transient_local(5), &sub_opts);
    assert!(!control_sub.is_null(), "the control subscription opens");
    let control = drain_points(context, control_sub);
    assert_eq!(rmw_destroy_subscription(node, control_sub), RMW_RET_OK);
    child.command("LIVE");
    child.expect_line(SENT_LIVE);
    let live = drain_points(context, late);
    child.finish();
    let marks_after = retain_marks(&topic_str);
    assert_eq!(rmw_destroy_subscription(node, late), RMW_RET_OK);
    assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    CrossProcessTake {
        best_available: LateTake { retained, live },
        control,
        marks_alive,
        marks_after,
    }
}

/// The retain mark of every publisher live on `topic`, in any process, as
/// this process's transport reads it.
#[cfg(cerulion_has_durability_best_available)]
fn retain_marks(topic: &str) -> Option<Vec<bool>> {
    rmw_cerulion::runtime::runtime()
        .expect("runtime")
        .transport
        .topic_publisher_retain_marks(topic)
        .ok()
}

#[cfg(cerulion_has_durability_best_available)]
fn assert_marks(got: Option<Vec<bool>>, want: Option<Vec<bool>>, what: &str) {
    assert_eq!(got, want, "{what}");
}

#[cfg(cerulion_has_durability_best_available)]
#[test]
#[serial]
fn a_best_available_subscription_takes_the_backlog_of_a_transient_local_publisher_in_another_process(
) {
    unsafe {
        let seen = cross_process_best_available_takes("tl", "transient_local", 20);
        assert_eq!(
            seen.best_available,
            LateTake {
                retained: xs(15..20),
                live: vec![LIVE_X],
            },
            "the only publisher, in another process, offers TRANSIENT_LOCAL, so \
             BEST_AVAILABLE resolves TRANSIENT_LOCAL: the newest 5 of 20, then the live one"
        );
        assert_eq!(
            seen.control,
            xs(15..20),
            "the TRANSIENT_LOCAL control takes the same newest 5"
        );
        assert_marks(
            seen.marks_alive,
            Some(vec![true]),
            "the other process's TRANSIENT_LOCAL publisher carries the retaining mark",
        );
        assert_marks(
            seen.marks_after,
            Some(Vec::new()),
            "the destroyed publisher leaves the topic's publisher list",
        );
    }
}

#[cfg(cerulion_has_durability_best_available)]
#[test]
#[serial]
fn a_best_available_subscription_takes_no_backlog_from_a_volatile_publisher_in_another_process() {
    unsafe {
        let seen = cross_process_best_available_takes("vol", "volatile", 8);
        assert_eq!(
            seen.best_available,
            LateTake {
                retained: Vec::new(),
                live: vec![LIVE_X],
            },
            "the only publisher, in another process, is VOLATILE, so BEST_AVAILABLE \
             resolves VOLATILE: none of the 8 messages its ring holds, only the live one"
        );
        assert_eq!(
            seen.control,
            xs(3..8),
            "control for that zero: a TRANSIENT_LOCAL subscription on the same topic \
             takes the newest 5 of the 8 the other process's ring holds"
        );
        assert_marks(
            seen.marks_alive,
            Some(vec![false]),
            "the other process's VOLATILE publisher carries no retaining mark",
        );
        assert_marks(
            seen.marks_after,
            Some(Vec::new()),
            "the destroyed publisher leaves the topic's publisher list",
        );
    }
}
