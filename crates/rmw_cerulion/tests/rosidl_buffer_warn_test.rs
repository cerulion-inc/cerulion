// SPDX-License-Identifier: AGPL-3.0-only
//! The operator's view of the `rosidl::Buffer` downgrade (Lyrical and
//! Rolling): a topic's first published frame that carried a non-CPU Buffer
//! logs one WARN naming the topic and the backend, later frames are only
//! counted, and the per-topic count is logged again at every power of ten.
//! A second topic gets its own first warning, and a subscription naming a
//! backend this rmw lacks warns once, naming it. Its own binary because
//! `#[traced_test]` installs the process-global subscriber.
#![cfg(all(unix, cerulion_has_is_rosidl_buffer))]

use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_void};

use rmw_cerulion::ffi::{
    self, rosidl_message_type_support_t, rosidl_typesupport_introspection_c__MessageMember,
    rosidl_typesupport_introspection_c__MessageMembers, RMW_RET_OK,
};
use rmw_cerulion::*;
use tracing_test::traced_test;

extern "C" {
    fn rmw_cerulion_fixture_buffer_u8_new(
        data: *const u8,
        len: usize,
        backend: *const c_char,
        throw_on_copy: c_int,
    ) -> *mut c_void;
    fn rmw_cerulion_fixture_buffer_u8_delete(b: *mut c_void);
}

const ROS_TYPE_UINT8: u8 = 8;
const HEADLINE: &str = "a rosidl::Buffer on a non-CPU backend was copied to host memory";
const DECADE: &str = "rosidl::Buffer CPU copies on this topic";
const SUB_IGNORED: &str = "acceptable_buffer_backends names backends rmw_cerulion does not have";

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

#[repr(C)]
struct CU8Seq {
    data: *mut u8,
    size: usize,
    capacity: usize,
    is_rosidl_buffer: bool,
    owns_rosidl_buffer: bool,
}

fn image_ts() -> *const rosidl_message_type_support_t {
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
    let mm = Box::leak(Box::new(
        rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("test_msgs__msg"),
            message_name_: cstr("BufWarnImage"),
            member_count_: 1,
            size_of_: std::mem::size_of::<CU8Seq>(),
            members_: members.as_ptr(),
            ..Default::default()
        },
    ));
    Box::leak(Box::new(rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    }))
}

unsafe fn publisher_on(
    node: *mut ffi::rmw_node_t,
    ts: *const rosidl_message_type_support_t,
    topic: &str,
) -> *mut ffi::rmw_publisher_t {
    let qos = ffi::rmw_qos_profile_t {
        history: ffi::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth: 4,
        reliability: ffi::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        durability: ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE,
        deadline: ffi::rmw_time_t { sec: 0, nsec: 0 },
        lifespan: ffi::rmw_time_t { sec: 0, nsec: 0 },
        liveliness: ffi::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
        liveliness_lease_duration: ffi::rmw_time_t { sec: 0, nsec: 0 },
        avoid_ros_namespace_conventions: false,
    };
    let opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
    let t = CString::new(topic).expect("topic");
    let p = rmw_create_publisher(node, ts, t.as_ptr(), &qos, &opts);
    assert!(!p.is_null());
    p
}

unsafe fn publish_device_frames(publisher: *mut ffi::rmw_publisher_t, n: usize) {
    let backend: &CStr = c"fixture_device";
    let bytes = [7u8; 64];
    for _ in 0..n {
        let msg = CU8Seq {
            data: rmw_cerulion_fixture_buffer_u8_new(
                bytes.as_ptr(),
                bytes.len(),
                backend.as_ptr(),
                0,
            ) as *mut u8,
            size: bytes.len(),
            capacity: 0,
            is_rosidl_buffer: true,
            owns_rosidl_buffer: true,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        rmw_cerulion_fixture_buffer_u8_delete(msg.data as *mut c_void);
    }
}

#[test]
#[traced_test]
fn a_topic_warns_on_its_first_downgrade_and_at_each_power_of_ten() {
    let suffix = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock")
        .as_nanos();
    let first = format!("/rosidl_buffer/warn/a/{suffix}");
    let second = format!("/rosidl_buffer/warn/b/{suffix}");
    unsafe {
        let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
        let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
        assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
        let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
        assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
        let node = rmw_create_node(context, cstr(&format!("bufwarn_{suffix}")), cstr("/"));
        assert!(!node.is_null());
        let ts = image_ts();
        let a = publisher_on(node, ts, &first);
        let b = publisher_on(node, ts, &second);
        publish_device_frames(a, 11);
        publish_device_frames(b, 1);
        // A subscription naming a backend this rmw lacks is created and
        // warns once, naming what it ignores.
        let mut sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        sub_opts.acceptable_buffer_backends = c"cpu, cuda".as_ptr();
        let qos = ffi::rmw_qos_profile_t {
            history: ffi::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth: 4,
            reliability: ffi::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability: ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE,
            deadline: ffi::rmw_time_t { sec: 0, nsec: 0 },
            lifespan: ffi::rmw_time_t { sec: 0, nsec: 0 },
            liveliness: ffi::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            liveliness_lease_duration: ffi::rmw_time_t { sec: 0, nsec: 0 },
            avoid_ros_namespace_conventions: false,
        };
        let t = CString::new(second.clone()).expect("topic");
        let sub = rmw_create_subscription(node, ts, t.as_ptr(), &qos, &sub_opts);
        assert!(!sub.is_null());
        assert_eq!(rmw_destroy_subscription(node, sub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, a), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, b), RMW_RET_OK);
    }
    assert_eq!(
        rmw_cerulion::rosidl_buffer::topic_downgrade_count(&first),
        11
    );
    assert_eq!(
        rmw_cerulion::rosidl_buffer::topic_downgrade_count(&second),
        1
    );
    logs_assert(|lines: &[&str]| {
        let on = |topic: &str, needle: &str| {
            lines
                .iter()
                .filter(|l| l.contains(" WARN ") && l.contains(needle))
                .filter(|l| l.split_whitespace().any(|t| t == format!("topic={topic}")))
                .copied()
                .collect::<Vec<_>>()
        };
        let first_head = on(&first, HEADLINE);
        let first_decades = on(&first, DECADE);
        let second_head = on(&second, HEADLINE);
        if first_head.len() != 1 || second_head.len() != 1 {
            return Err(format!(
                "one headline per topic, got {first_head:?} and {second_head:?}"
            ));
        }
        if !first_head[0].contains("backend=fixture_device")
            || !first_head[0].contains("topic_downgrades=1")
        {
            return Err(format!("headline fields: {}", first_head[0]));
        }
        if first_decades.len() != 1 || !first_decades[0].contains("topic_downgrades=10") {
            return Err(format!("one decade line at 10, got {first_decades:?}"));
        }
        let ignored = on(&second, SUB_IGNORED);
        if ignored.len() != 1 || !ignored[0].contains("ignored_backends=cuda") {
            return Err(format!("one ignored-backends warning, got {ignored:?}"));
        }
        Ok(())
    });
}
