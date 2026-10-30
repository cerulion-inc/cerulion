// SPDX-License-Identifier: AGPL-3.0-only
#![cfg(unix)]
//! A topic's iceoryx2 service parameters must not depend on which endpoint
//! opens the topic first (issue 617). The first opener creates the service;
//! a later opener that needs a deeper history or a larger subscriber buffer
//! ceiling used to be refused with `DoesNotSupportRequestedMinHistorySize` or
//! `DoesNotSupportRequestedMinBufferSize`. Each test opens the topic with the
//! smaller endpoint first, then a TRANSIENT_LOCAL depth-16 publisher, and
//! requires the second open to succeed.
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
    let ws = rmw_create_wait_set(context, 8);
    assert!(!ws.is_null(), "wait set");
    let mut received = 0;
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
            received += 1;
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
