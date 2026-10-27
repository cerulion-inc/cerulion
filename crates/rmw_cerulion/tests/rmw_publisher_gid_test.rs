// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The publisher gid a take reports, through the rmw C ABI over real
//! iceoryx2 shared memory.
//!
//! rclcpp drops the rmw copy of a message its intra-process path already
//! delivered when `rmw_message_info_t::publisher_gid` equals the gid
//! `rmw_get_gid_for_publisher` returned for one of its own publishers
//! (compared with `rmw_compare_gids_equal`). These tests pin that every take
//! path with an info out-parameter reports the publishing rmw publisher's gid,
//! that a non-rmw publisher reports the all-zero gid, that gids are unique
//! across processes (two child processes with equal entity numbers), and that
//! endpoint info reports the same gids.
//!
//! ⚠️ iceoryx2 shared memory is a process singleton: run with
//! `--test-threads=1`:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_publisher_gid_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::ffi::CString;
use std::io::{BufRead, BufReader};
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use cerulion_core::wire::MaxSliceLen;
use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;

const ROS_TYPE_DOUBLE: u8 = 2;

/// Set to "1" only on a spawned child, so a bare `-- --ignored` run no-ops.
const ENV_CHILD: &str = "CER_PUBLISHER_GID_CHILD";
const ENV_TOPIC: &str = "CER_PUBLISHER_GID_TOPIC";
const ENV_TYPE: &str = "CER_PUBLISHER_GID_TYPE";
const ENV_X: &str = "CER_PUBLISHER_GID_X";
const CHILD_TEST: &str = "subprocess_child_publishes_once";
const CHILD_TIMEOUT: Duration = Duration::from_secs(120);
const PROBE_GID: &str = "CHILD_GID=";
const PROBE_READY: &str = "CHILD_READY";

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
#[derive(Default, Clone, Copy, PartialEq, Debug)]
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
            message_namespace_: cstr("rmw_gid__msg"),
            message_name_: cstr(unique),
            member_count_: members.len() as u32,
            size_of_: std::mem::size_of::<CPoint>(),
            members_: members.as_ptr(),
            ..Default::default()
        },
    ));
    Box::leak(Box::new(ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    }))
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

/// The pid-namespace id the gid rule names, read independently of the crate:
/// the low 32 bits of the `/proc/self/ns/pid` inode, 0 where there is none.
fn expected_pid_namespace_id() -> u32 {
    match std::fs::read_link("/proc/self/ns/pid") {
        Ok(p) => {
            let s = p.to_string_lossy().into_owned();
            let inner = s
                .strip_prefix("pid:[")
                .and_then(|r| r.strip_suffix(']'))
                .unwrap_or_else(|| panic!("unexpected /proc/self/ns/pid target {s:?}"));
            inner.parse::<u64>().expect("pid namespace inode") as u32
        }
        Err(_) => 0,
    }
}

/// The 16 gid bytes of an `rmw_gid_t`, after asserting the storage tail
/// (24-byte storage before Iron) is zero and the identifier is this rmw's.
fn gid_bytes(gid: &ffi::rmw_gid_t) -> [u8; 16] {
    assert_eq!(
        gid.implementation_identifier,
        ffi::implementation_identifier_ptr(),
        "the gid names this implementation"
    );
    assert!(
        gid.data[16..].iter().all(|b| *b == 0),
        "storage tail is zero"
    );
    gid.data[..16].try_into().expect("16 bytes")
}

fn hex(gid: &[u8; 16]) -> String {
    gid.iter().map(|b| format!("{b:02x}")).collect()
}

fn unhex(s: &str) -> [u8; 16] {
    assert_eq!(s.len(), 32, "{s:?}");
    let mut gid = [0u8; 16];
    for (i, b) in gid.iter_mut().enumerate() {
        *b = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).expect("hex");
    }
    gid
}

/// Asserts `gid` follows the layout for THIS process: pid-namespace id,
/// process id, entity counter.
fn assert_this_process_layout(gid: &[u8; 16]) {
    assert_eq!(
        u32::from_le_bytes(gid[0..4].try_into().unwrap()),
        expected_pid_namespace_id(),
        "bytes 0..4 are the pid-namespace id"
    );
    assert_eq!(
        u32::from_le_bytes(gid[4..8].try_into().unwrap()),
        std::process::id(),
        "bytes 4..8 are the process id"
    );
}

unsafe fn publisher_gid(publisher: *const ffi::rmw_publisher_t) -> ffi::rmw_gid_t {
    let mut gid: ffi::rmw_gid_t = std::mem::zeroed();
    assert_eq!(rmw_get_gid_for_publisher(publisher, &mut gid), RMW_RET_OK);
    gid
}

unsafe fn gids_equal(a: &ffi::rmw_gid_t, b: &ffi::rmw_gid_t) -> bool {
    let mut equal = false;
    assert_eq!(rmw_compare_gids_equal(a, b, &mut equal), RMW_RET_OK);
    equal
}

unsafe fn publish(publisher: *const ffi::rmw_publisher_t, x: f64) {
    let msg = CPoint { x, y: 0.0, z: 0.0 };
    assert_eq!(
        rmw_publish(
            publisher,
            &msg as *const _ as *const c_void,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
}

unsafe fn take_with_info(
    subscription: *const ffi::rmw_subscription_t,
) -> (CPoint, ffi::rmw_message_info_t) {
    let mut out = CPoint::default();
    let mut taken = false;
    let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
    // Poison: the take must overwrite every gid byte.
    info.publisher_gid.data = [0xa5; ffi::RMW_GID_STORAGE_SIZE as usize];
    assert_eq!(
        rmw_take_with_info(
            subscription,
            &mut out as *mut _ as *mut c_void,
            &mut taken,
            &mut info,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
    assert!(taken, "a published message must be taken");
    (out, info)
}

/// The upstream shape (NVIDIA `test_cuda_image_mixed_pubsub`): one publisher,
/// and in the same process a subscription standing for rclcpp's
/// intra-process subscription's rmw side. Every take path that reports
/// message info reports the publisher's own gid, so rclcpp's
/// `rmw_compare_gids_equal` filter finds its own publisher and drops the
/// duplicate. A second rmw publisher on the topic reports its own, distinct,
/// gid, and a native (non-rmw) publisher reports the all-zero gid.
#[test]
#[serial]
fn every_take_path_reports_the_publishing_rmw_publishers_gid() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("GidA{suffix}"));
        let (node, _opts) = setup_node(&format!("gid_node_{suffix}"));
        let ros_topic = format!("/rmw_gid/paths/{suffix}");
        let topic = CString::new(ros_topic.clone()).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        assert!(
            (*subscription).can_loan_messages,
            "fixed POD type loans on take"
        );
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let second = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!second.is_null());

        let pub_gid = publisher_gid(publisher);
        let second_gid = publisher_gid(second);
        let pub_bytes = gid_bytes(&pub_gid);
        assert_this_process_layout(&pub_bytes);
        assert_this_process_layout(&gid_bytes(&second_gid));
        assert!(
            !gids_equal(&pub_gid, &second_gid),
            "two publishers, two gids"
        );

        // rmw_take: no info out-parameter, the message still arrives.
        publish(publisher, 1.0);
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
        assert_eq!(out.x, 1.0);

        // rmw_take_with_info.
        publish(publisher, 2.0);
        let (out, info) = take_with_info(subscription);
        assert_eq!(out.x, 2.0);
        assert_eq!(gid_bytes(&info.publisher_gid), pub_bytes);
        assert!(
            gids_equal(&info.publisher_gid, &pub_gid),
            "rclcpp's filter compares with rmw_compare_gids_equal"
        );
        assert!(!gids_equal(&info.publisher_gid, &second_gid));

        // The second publisher's message reports the second publisher.
        publish(second, 3.0);
        let (out, info) = take_with_info(subscription);
        assert_eq!(out.x, 3.0);
        assert!(gids_equal(&info.publisher_gid, &second_gid));
        assert!(!gids_equal(&info.publisher_gid, &pub_gid));

        // rmw_take_sequence: two messages, two publishers, in order.
        publish(publisher, 4.0);
        publish(second, 5.0);
        let mut msgs = [CPoint::default(); 2];
        let mut msg_ptrs: [*mut c_void; 2] = [
            &mut msgs[0] as *mut _ as *mut c_void,
            &mut msgs[1] as *mut _ as *mut c_void,
        ];
        let mut infos: [ffi::rmw_message_info_t; 2] = std::mem::zeroed();
        let mut seq: ffi::rmw_message_sequence_t = std::mem::zeroed();
        seq.data = msg_ptrs.as_mut_ptr();
        seq.capacity = 2;
        let mut info_seq: ffi::rmw_message_info_sequence_t = std::mem::zeroed();
        info_seq.data = infos.as_mut_ptr();
        info_seq.capacity = 2;
        let mut n = 0usize;
        assert_eq!(
            rmw_take_sequence(
                subscription,
                2,
                &mut seq,
                &mut info_seq,
                &mut n,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert_eq!(n, 2);
        assert_eq!((msgs[0].x, msgs[1].x), (4.0, 5.0));
        assert!(gids_equal(&infos[0].publisher_gid, &pub_gid));
        assert!(gids_equal(&infos[1].publisher_gid, &second_gid));

        // rmw_take_serialized_message_with_info.
        publish(publisher, 6.0);
        let mut serialized: ffi::rmw_serialized_message_t = std::mem::zeroed();
        serialized.allocator = malloc_allocator();
        let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
        let mut taken = false;
        assert_eq!(
            rmw_take_serialized_message_with_info(
                subscription,
                &mut serialized,
                &mut taken,
                &mut info,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(taken);
        assert!(gids_equal(&info.publisher_gid, &pub_gid));
        assert_eq!(gid_bytes(&info.publisher_gid), pub_bytes);
        // The frame this take returned is the native wire frame; keep it for
        // the native publisher arm below.
        let frame =
            std::slice::from_raw_parts(serialized.buffer, serialized.buffer_length).to_vec();
        free(serialized.buffer as *mut c_void);

        // rmw_take_loaned_message_with_info.
        publish(second, 7.0);
        let mut loaned: *mut c_void = std::ptr::null_mut();
        let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
        let mut taken = false;
        assert_eq!(
            rmw_take_loaned_message_with_info(
                subscription,
                &mut loaned,
                &mut taken,
                &mut info,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(taken && !loaned.is_null());
        assert_eq!((*(loaned as *const CPoint)).x, 7.0);
        assert!(gids_equal(&info.publisher_gid, &second_gid));
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, loaned),
            RMW_RET_OK
        );

        // A native publisher (no rmw port name) reports the all-zero gid,
        // which matches no rmw publisher.
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let mut native = rt
            .transport
            .create_publisher(&ros_topic, MaxSliceLen::const_new(4096), 0)
            .expect("native publisher on the rmw topic");
        native.publish_raw(&frame).expect("native publish");
        let (out, info) = take_with_info(subscription);
        assert_eq!(
            out.x, 6.0,
            "the native publisher re-sent the serialized frame"
        );
        assert_eq!(gid_bytes(&info.publisher_gid), [0u8; 16]);
        assert!(!gids_equal(&info.publisher_gid, &pub_gid));
        assert!(!gids_equal(&info.publisher_gid, &second_gid));
        drop(native);

        // The first publisher again: the cached entry still answers.
        publish(publisher, 8.0);
        let (out, info) = take_with_info(subscription);
        assert_eq!(out.x, 8.0);
        assert!(gids_equal(&info.publisher_gid, &pub_gid));

        assert_eq!(rmw_destroy_publisher(node, second), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Endpoint info reports the gid `rmw_get_gid_for_publisher` returns, and a
/// subscription's gid follows the same layout.
#[test]
#[serial]
fn endpoint_info_reports_the_same_gids() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("GidE{suffix}"));
        let (node, _opts) = setup_node(&format!("gid_ep_node_{suffix}"));
        let topic = CString::new(format!("/rmw_gid/ep/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!publisher.is_null() && !subscription.is_null());
        let pub_gid = publisher_gid(publisher);

        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_publishers_info_by_topic(node, &mut allocator, topic.as_ptr(), false, &mut arr),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 1);
        assert_eq!((*arr.info_array).endpoint_gid, pub_gid.data);

        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_subscriptions_info_by_topic(
                node,
                &mut allocator,
                topic.as_ptr(),
                false,
                &mut arr
            ),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 1);
        let sub_gid = (*arr.info_array).endpoint_gid;
        assert!(
            sub_gid[16..].iter().all(|b| *b == 0),
            "storage tail is zero"
        );
        let sub_bytes: [u8; 16] = sub_gid[..16].try_into().unwrap();
        assert_this_process_layout(&sub_bytes);
        assert_ne!(sub_gid, pub_gid.data, "publisher and subscription differ");

        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// Cross-process: two children with equal entity numbers
// =====================================================================

/// THE CHILD: creates one rmw publisher on the parent's topic, publishes one
/// message, reports its gid, and stays attached until its stdin closes.
/// `#[ignore]`d: only the parent spawns it (with `ENV_CHILD=1`).
#[test]
#[ignore = "spawned by the cross-process parent test"]
fn subprocess_child_publishes_once() {
    if std::env::var(ENV_CHILD).as_deref() != Ok("1") {
        return;
    }
    let topic = std::env::var(ENV_TOPIC).expect("topic");
    let type_name = std::env::var(ENV_TYPE).expect("type");
    let x: f64 = std::env::var(ENV_X).expect("x").parse().expect("x");
    unsafe {
        let ts = point_ts(&type_name);
        let (node, _opts) = setup_node("gid_child_node");
        let topic = CString::new(topic).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let gid = gid_bytes(&publisher_gid(publisher));
        publish(publisher, x);
        println!("{PROBE_GID}{}", hex(&gid));
        println!("{PROBE_READY}");
        let mut sink = String::new();
        let _ = std::io::Read::read_to_string(&mut std::io::stdin(), &mut sink);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

struct Child {
    process: std::process::Child,
    gid: [u8; 16],
}

/// Spawns a child, reads its gid and waits until it has published.
fn spawn_child(topic: &str, type_name: &str, x: f64) -> Child {
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
        .env(ENV_X, x.to_string())
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::inherit())
        .spawn()
        .expect("spawn child");
    let stdout = process.stdout.take().expect("child stdout");
    let (tx, rx) = std::sync::mpsc::channel();
    // The reader drains the child's stdout to its end, even after the parent
    // stops listening: a closed pipe would fail the child's own libtest
    // summary write with EPIPE and exit it 101.
    std::thread::spawn(move || {
        for line in BufReader::new(stdout).lines() {
            let Ok(line) = line else { break };
            let _ = tx.send(line);
        }
    });
    let deadline = Instant::now() + CHILD_TIMEOUT;
    let mut gid = None;
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        let line = match rx.recv_timeout(left) {
            Ok(line) => line,
            Err(e) => {
                let _ = process.kill();
                let _ = process.wait();
                panic!("child did not report ready ({e:?}); gid so far {gid:?}");
            }
        };
        // libtest prints `test <name> ... ` without a newline before the
        // body runs, so a probe can share a line with it.
        if let Some(at) = line.find(PROBE_GID) {
            gid = Some(unhex(line[at + PROBE_GID.len()..].trim()));
        }
        if line.contains(PROBE_READY) {
            break;
        }
    }
    Child {
        process,
        gid: gid.expect("the child reports its gid before ready"),
    }
}

fn finish_child(mut child: Child) {
    drop(child.process.stdin.take());
    let deadline = Instant::now() + CHILD_TIMEOUT;
    loop {
        match child.process.try_wait().expect("try_wait") {
            Some(status) => {
                assert!(status.success(), "child exited with {status}");
                return;
            }
            None if Instant::now() > deadline => {
                let _ = child.process.kill();
                let _ = child.process.wait();
                panic!("child did not exit within {CHILD_TIMEOUT:?}");
            }
            None => std::thread::sleep(Duration::from_millis(20)),
        }
    }
}

/// Two child processes run the same creation sequence, so their publishers
/// hold the same entity number; their gids still differ (process id), each
/// follows the layout for its own process, and a take in this process
/// reports each message's own publisher's gid.
#[test]
#[serial]
fn gids_are_unique_across_processes_and_a_cross_process_take_reports_the_childs_gid() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("GidX{suffix}");
        let ts = point_ts(&type_name);
        let (node, _opts) = setup_node(&format!("gid_parent_node_{suffix}"));
        let ros_topic = format!("/rmw_gid/xproc/{suffix}");
        let topic = CString::new(ros_topic.clone()).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());

        let a = spawn_child(&ros_topic, &type_name, 10.0);
        let b = spawn_child(&ros_topic, &type_name, 20.0);

        assert_eq!(
            a.gid[8..],
            b.gid[8..],
            "equal entity numbers in the two children"
        );
        assert_ne!(a.gid, b.gid, "gids differ across processes");
        assert_eq!(
            u32::from_le_bytes(a.gid[4..8].try_into().unwrap()),
            a.process.id()
        );
        assert_eq!(
            u32::from_le_bytes(b.gid[4..8].try_into().unwrap()),
            b.process.id()
        );

        for _ in 0..2 {
            let (out, info) = take_with_info(subscription);
            let expected = if out.x == 10.0 {
                a.gid
            } else {
                assert_eq!(out.x, 20.0);
                b.gid
            };
            assert_eq!(gid_bytes(&info.publisher_gid), expected);
            let mut expected_gid: ffi::rmw_gid_t = std::mem::zeroed();
            expected_gid.implementation_identifier = ffi::implementation_identifier_ptr();
            expected_gid.data[..16].copy_from_slice(&expected);
            assert!(gids_equal(&info.publisher_gid, &expected_gid));
        }

        finish_child(a);
        finish_child(b);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
