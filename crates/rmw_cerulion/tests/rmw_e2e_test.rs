// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! Full-stack rmw conformance e2e: drives the EXACT
//! extern "C" surface rcl would call — init → node → publisher /
//! subscription → publish → wait → take, the loaned zero-copy path, and
//! services — over REAL iceoryx2 shared memory. No ROS installation
//! required (typesupports are hand-built introspection data).
//!
//! ⚠️ iceoryx2 shared memory is a process singleton — run with
//! `--test-threads=1`:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_e2e_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};

use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;

// =====================================================================
// Hand-built typesupport fixtures (same pattern as bridge_test.rs)
// =====================================================================

const ROS_TYPE_DOUBLE: u8 = 2;
const ROS_TYPE_STRING: u8 = 16;

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

#[repr(C)]
struct CRosString {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

fn point_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    // Unique TYPE NAME per test run so schema hashes don't collide with
    // other runs against the global SHM singleton.
    make_message_ts(
        "rmw_e2e__msg",
        unique,
        std::mem::size_of::<CPoint>(),
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

/// Bring up context + node through the C ABI.
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
// Pub/sub through the C ABI over real iceoryx2
// =====================================================================

#[test]
#[serial]
fn c_abi_publish_take_round_trip() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("PtA{suffix}");
        let ts = point_ts(&type_name);
        let (_, node, _opts) = setup_node(&format!("rt_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/rt/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        // Matched counts through the graph API.
        let mut count = 0usize;
        assert_eq!(
            rmw_publisher_count_matched_subscriptions(publisher, &mut count),
            RMW_RET_OK
        );
        assert_eq!(count, 1);

        // Publish.
        let msg = CPoint {
            x: 1.0,
            y: -2.0,
            z: 0.5,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );

        // Wait reports readiness (subscription data pointer, as rcl does).
        let mut sub_ptrs = [(*subscription).data];
        let mut subs = ffi::rmw_subscriptions_t {
            subscriber_count: 1,
            subscribers: sub_ptrs.as_mut_ptr(),
        };
        let timeout = ffi::rmw_time_t { sec: 1, nsec: 0 };
        let ret = rmw_wait(
            &mut subs,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            rmw_create_wait_set((*node).context, 8),
            &timeout,
        );
        assert_eq!(ret, RMW_RET_OK, "wait must wake on the published sample");
        assert!(!sub_ptrs[0].is_null(), "subscription must be ready");

        // Take.
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

        // Second take: empty.
        let mut taken2 = true;
        assert_eq!(
            rmw_take(
                subscription,
                &mut out as *mut _ as *mut c_void,
                &mut taken2,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(!taken2);

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The loaned-message zero-copy path: borrow hands out a pointer INTO
/// the SHM slot; publish sends that same slot; the subscriber reads the
/// bytes rclcpp wrote — no flatten copy anywhere.
///
/// CONTRACT NOTE: borrow no longer hands out a zero-filled
/// slot — it runs the typesupport's `init_function` (rosidl defaults).
/// This fixture's typesupport has NO `init_function`, so borrow takes
/// the documented fallback (zeroed payload + breadcrumb) and the test's
/// behavior is unchanged: every field is written before publish, and the
/// cancelled borrow never ships. The init/defaults contract itself is
/// pinned by the loan-init tests below.
#[test]
#[serial]
fn c_abi_loaned_message_zero_copy_path() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("PtB{suffix}");
        let ts = point_ts(&type_name);
        let (_, node, _opts) = setup_node(&format!("loan_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/loan/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!((*publisher).can_loan_messages, "fixed POD type must loan");

        // Borrow → write fields through the raw pointer (exactly what
        // rclcpp's LoanedMessage does) → publish.
        let mut loaned: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut loaned),
            RMW_RET_OK
        );
        assert!(!loaned.is_null());
        let point = &mut *(loaned as *mut CPoint);
        point.x = 42.0;
        point.y = 43.0;
        point.z = 44.0;
        assert_eq!(
            rmw_publish_loaned_message(publisher, loaned, std::ptr::null_mut()),
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
        assert_eq!(
            out,
            CPoint {
                x: 42.0,
                y: 43.0,
                z: 44.0
            }
        );

        // Borrow + cancel releases the slot without publishing.
        let mut cancelled: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut cancelled),
            RMW_RET_OK
        );
        assert_eq!(
            rmw_return_loaned_message_from_publisher(publisher, cancelled),
            RMW_RET_OK
        );
        let mut taken3 = true;
        assert_eq!(
            rmw_take(
                subscription,
                &mut out as *mut _ as *mut c_void,
                &mut taken3,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(!taken3, "cancelled loan must not publish");

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// Loaned-message initialization correctness
//
// rclcpp's LoanedMessage does NOT placement-new MessageT on the loaned
// branch (jazzy loaned_message.hpp: static_cast only; only the heap
// fallback constructs), so the rmw's slot state IS the message's
// initial state. Borrow must therefore serve rosidl DEFAULTS, not
// zeros: a Quaternion declares `float64 w 1`, and a loan published
// untouched must read w == 1.0 exactly as on every other rmw.
// =====================================================================

/// Quaternion-shaped fixed POD: x/y/z default-less, w declares `1`.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Debug)]
struct CQuat {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

/// Faithful model of rosidl_generator_c's `__init` (called by the
/// introspection `init_function` wrapper, which ignores its `_init`
/// argument — rosidl#477): assignments are emitted ONLY for members
/// that need init. Here only `w` declares a default, so ONLY `w` is
/// written — x/y/z must come from the rmw's zero baseline, which is
/// exactly what makes this fixture prove BOTH halves of the C
/// contract (zero pre-pass + init call).
///
/// Records a call COUNT plus the count of non-ALL arguments (the cpp
/// fixture's double pin): value-oracles alone cannot see a publish path
/// that RE-RUNS init, because a re-run rewrites the same values —
/// asserted in the test bodies, never here (a panic must not cross the
/// C ABI). The real C wrapper ignores the argument; what the counter
/// pins is what the RMW passes.
static C_INIT_CALLS: AtomicU64 = AtomicU64::new(0);
static C_INIT_NON_ALL_ARGS: AtomicU64 = AtomicU64::new(0);

unsafe extern "C" fn cquat_c_init(
    msg: *mut c_void,
    init: ffi::rosidl_runtime_c__message_initialization,
) {
    C_INIT_CALLS.fetch_add(1, Ordering::SeqCst);
    if init != 0 {
        // 0 == ROSIDL_RUNTIME_C_MSG_INIT_ALL
        C_INIT_NON_ALL_ARGS.fetch_add(1, Ordering::SeqCst);
    }
    (*(msg as *mut CQuat)).w = 1.0;
}

fn make_message_ts_with_init(
    namespace: &str,
    name: &str,
    size_of: usize,
    members: Vec<ffi::rosidl_typesupport_introspection_c__MessageMember>,
    init: unsafe extern "C" fn(*mut c_void, ffi::rosidl_runtime_c__message_initialization),
) -> *const ffi::rosidl_message_type_support_t {
    let members = Box::leak(members.into_boxed_slice());
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr(namespace),
            message_name_: cstr(name),
            member_count_: members.len() as u32,
            size_of_: size_of,
            members_: members.as_ptr(),
            init_function: Some(init),
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

fn quat_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    make_message_ts_with_init(
        "rmw_e2e__msg",
        unique,
        std::mem::size_of::<CQuat>(),
        vec![
            member("x", ROS_TYPE_DOUBLE, 0),
            member("y", ROS_TYPE_DOUBLE, 8),
            member("z", ROS_TYPE_DOUBLE, 16),
            member("w", ROS_TYPE_DOUBLE, 24),
        ],
        cquat_c_init,
    )
}

/// Shared scaffolding for the loan tests: node + pub/sub over
/// one topic, asserting the type really is loanable.
unsafe fn setup_loan_pair(
    ts: *const ffi::rosidl_message_type_support_t,
    tag: &str,
    suffix: u64,
) -> (
    *mut ffi::rmw_node_t,
    *mut ffi::rmw_publisher_t,
    *mut ffi::rmw_subscription_t,
) {
    let (_, node, _opts) = setup_node(&format!("{tag}_node_{suffix}"));
    let topic = CString::new(format!("/rmw_e2e/{tag}/{suffix}")).expect("topic");
    let qos = default_qos();
    let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
    assert!(!subscription.is_null());
    let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
    assert!(!publisher.is_null());
    assert!(
        (*publisher).can_loan_messages,
        "recursively-fixed POD type must loan"
    );
    (node, publisher, subscription)
}

/// THE pin: a loan published UNTOUCHED carries the rosidl
/// DEFAULTS, not zeros. Sentinel-filled take target so the x/y/z zeros
/// are proven written by the frame, never inherited. Bit-exact oracle.
#[test]
#[serial]
fn untouched_loan_publishes_rosidl_defaults_not_zeros() {
    unsafe {
        let suffix = unique_suffix();
        let ts = quat_ts(&format!("Qa{suffix}"));
        let (node, publisher, subscription) = setup_loan_pair(ts, "qdef", suffix);

        C_INIT_CALLS.store(0, Ordering::SeqCst);
        C_INIT_NON_ALL_ARGS.store(0, Ordering::SeqCst);
        let mut loaned: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut loaned),
            RMW_RET_OK
        );
        assert!(!loaned.is_null());
        // The pointer the typesupport's init just wrote through, and that
        // rclcpp will overlay the message struct on, is 8-aligned (the
        // borrow path's fail-closed alignment gate let it through; the
        // de-facto iceoryx2 0.9.1 mechanism is chunk+40 B header @ 8, +32
        // WireHeader — see `rmw_borrow_loaned_message`).
        assert_eq!(
            (loaned as usize) % std::mem::align_of::<CQuat>(),
            0,
            "loaned payload pointer must satisfy the message struct's alignment"
        );
        // Publish with NO field writes — the rclcpp shape this bug bit:
        // LoanedMessage never constructs on the loaned branch, so what
        // borrow left in the slot is what ships.
        assert_eq!(
            rmw_publish_loaned_message(publisher, loaned, std::ptr::null_mut()),
            RMW_RET_OK
        );

        let mut out = CQuat {
            x: 9.0,
            y: 9.0,
            z: 9.0,
            w: 9.0,
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
        assert!(taken);
        assert_eq!(
            out.w.to_bits(),
            1.0f64.to_bits(),
            "declared default `w 1` must survive an untouched loan (a zeroed loan reads 0.0)"
        );
        assert_eq!(out.x.to_bits(), 0.0f64.to_bits(), "default-less x is zero");
        assert_eq!(out.y.to_bits(), 0.0f64.to_bits(), "default-less y is zero");
        assert_eq!(out.z.to_bits(), 0.0f64.to_bits(), "default-less z is zero");
        // Rechecked AFTER publish + take: the value oracle above cannot see
        // a publish path that RE-RUNS init (a re-run rewrites the same
        // values) — the count can. Init belongs to borrow, exactly once.
        assert_eq!(
            C_INIT_CALLS.load(Ordering::SeqCst),
            1,
            "publish/take must not re-run the init_function"
        );
        assert_eq!(C_INIT_NON_ALL_ARGS.load(Ordering::SeqCst), 0);

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Init runs at BORROW, before the caller's writes — a written field
/// carries the caller's value while an untouched defaulted field still
/// carries its default (init must not clobber, and must not re-run at
/// publish).
#[test]
#[serial]
fn borrow_init_does_not_clobber_caller_writes() {
    unsafe {
        let suffix = unique_suffix();
        let ts = quat_ts(&format!("Qb{suffix}"));
        let (node, publisher, subscription) = setup_loan_pair(ts, "qmix", suffix);

        let mut loaned: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut loaned),
            RMW_RET_OK
        );
        (*(loaned as *mut CQuat)).x = 2.0;
        assert_eq!(
            rmw_publish_loaned_message(publisher, loaned, std::ptr::null_mut()),
            RMW_RET_OK
        );

        let mut out = CQuat {
            x: 9.0,
            y: 9.0,
            z: 9.0,
            w: 9.0,
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
        assert!(taken);
        assert_eq!(out.x.to_bits(), 2.0f64.to_bits(), "caller write survives");
        assert_eq!(out.w.to_bits(), 1.0f64.to_bits(), "default not clobbered");
        assert_eq!(out.y.to_bits(), 0.0f64.to_bits());
        assert_eq!(out.z.to_bits(), 0.0f64.to_bits());

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// bool-then-f64: 7 repr(C) padding bytes at [1, 8). No init_function
/// (also pins the documented None-arm fallback: zeroed baseline for a
/// default-less type).
#[repr(C)]
struct CPadded {
    flag: u8,
    value: f64,
}

const ROS_TYPE_BOOLEAN: u8 = 6;

fn padded_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    make_message_ts(
        "rmw_e2e__msg",
        unique,
        std::mem::size_of::<CPadded>(),
        vec![
            member("flag", ROS_TYPE_BOOLEAN, 0),
            member("value", ROS_TYPE_DOUBLE, 8),
        ],
    )
}

/// Padding determinism: the caller stamps DIFFERENT garbage
/// over the whole payload of two loans (modeling a whole-struct
/// assignment, which copies a stack temporary's padding bytes), writes
/// identical field values, and the two frames on the wire must be
/// byte-identical to each other AND to a hand oracle with ZERO padding
/// — the publish-time `loan_pad_ranges` zeroing is what makes that
/// true. Without it, frame bytes depend on process memory (Principle #7
/// violation + memory disclosure into SHM).
#[test]
#[serial]
fn loan_padding_is_zeroed_and_frames_deterministic() {
    use cerulion_core::wire::WireHeader;
    unsafe {
        let suffix = unique_suffix();
        let ts = padded_ts(&format!("Pd{suffix}"));
        let (node, publisher, subscription) = setup_loan_pair(ts, "pad", suffix);

        let payload_len = std::mem::size_of::<CPadded>();
        for garbage in [0xAAu8, 0x55u8] {
            let mut loaned: *mut c_void = std::ptr::null_mut();
            assert_eq!(
                rmw_borrow_loaned_message(publisher, ts, &mut loaned),
                RMW_RET_OK
            );
            // Whole-payload garbage stamp, THEN field writes — the pad
            // bytes now hold `garbage` unless publish zeroes them.
            std::ptr::write_bytes(loaned as *mut u8, garbage, payload_len);
            let msg = &mut *(loaned as *mut CPadded);
            msg.flag = 1;
            msg.value = 3.25;
            assert_eq!(
                rmw_publish_loaned_message(publisher, loaned, std::ptr::null_mut()),
                RMW_RET_OK
            );
        }

        // Hand oracle for the 16-byte payload: flag=1, 7 zero pad bytes,
        // 3.25 LE.
        let mut oracle = [0u8; 16];
        oracle[0] = 1;
        oracle[8..16].copy_from_slice(&3.25f64.to_le_bytes());

        let mut frames: Vec<Vec<u8>> = Vec::new();
        for i in 0..2 {
            let mut serialized: ffi::rmw_serialized_message_t = std::mem::zeroed();
            serialized.allocator = malloc_allocator();
            let mut taken = false;
            assert_eq!(
                rmw_take_serialized_message(
                    subscription,
                    &mut serialized,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            assert!(taken, "frame {i} must be taken");
            let wire = std::slice::from_raw_parts(
                serialized.buffer as *const u8,
                serialized.buffer_length,
            );
            assert_eq!(wire.len(), WireHeader::SIZE + payload_len);
            frames.push(wire[WireHeader::SIZE..].to_vec());
            free(serialized.buffer as *mut c_void);
        }
        assert_eq!(
            frames[0], oracle,
            "payload must be fields + ZERO padding (0xAA garbage must not ship)"
        );
        assert_eq!(
            frames[1], oracle,
            "payload must be fields + ZERO padding (0x55 garbage must not ship)"
        );
        assert_eq!(
            frames[0], frames[1],
            "two loans with identical field values must be byte-identical on the wire"
        );

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Negative control: a variable type (string field) still refuses
/// borrow with UNSUPPORTED: the loan-init rework changed how a loan is
/// initialized, never WHAT is loanable.
#[test]
#[serial]
fn variable_type_still_refuses_borrow() {
    unsafe {
        let suffix = unique_suffix();
        let ts = make_message_ts(
            "rmw_e2e__msg",
            &format!("Vs{suffix}"),
            std::mem::size_of::<CRosString>(),
            vec![member("label", ROS_TYPE_STRING, 0)],
        );
        let (_, node, _opts) = setup_node(&format!("vloan_node_{suffix}"));
        let topic = CString::new(format!("/rmw_e2e/vloan/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        assert!(!(*publisher).can_loan_messages);

        let mut loaned: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut loaned),
            ffi::RMW_RET_UNSUPPORTED,
            "variable types must keep refusing the loan path"
        );
        assert!(loaned.is_null(), "a refused borrow hands out no pointer");

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Faithful model of rosidl_generator_cpp's `init_function`: placement-
/// new with `MessageInitialization::ALL`, whose constructor writes
/// EVERY member — zeros where no default is declared, the declared
/// default otherwise. (Contrast `cquat_c_init`: the C generator skips
/// default-less members.) Records a call COUNT plus the count of calls
/// whose argument was NOT `ALL`, so the test can pin that the fn ran
/// EXACTLY once and every call asked for ALL (0) — a last-value record
/// would pass a double call or a wrong-then-right sequence. Asserted in
/// the test body, never here: a panic must not cross the C ABI.
static CPP_INIT_CALLS: AtomicU64 = AtomicU64::new(0);
static CPP_INIT_NON_ALL_ARGS: AtomicU64 = AtomicU64::new(0);

unsafe extern "C" fn cquat_cpp_init(msg: *mut c_void, init: u32) {
    CPP_INIT_CALLS.fetch_add(1, Ordering::SeqCst);
    if init != rmw_cerulion::ffi::introspection_cpp::CPP_MSG_INIT_ALL {
        CPP_INIT_NON_ALL_ARGS.fetch_add(1, Ordering::SeqCst);
    }
    let q = &mut *(msg as *mut CQuat);
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
}

/// The C++-typesupport arm of the pin — the production rclcpp
/// path: the Cpp bridge dispatches borrow-time init through
/// `CppMessageMembers::init_function` with ALL (no zero pre-pass — the
/// ALL constructor writes every member), and the untouched loan carries
/// the defaults.
#[test]
#[serial]
fn cpp_typesupport_untouched_loan_serves_defaults() {
    unsafe {
        use rmw_cerulion::ffi::introspection_cpp::{CppMessageMember, CppMessageMembers};

        let suffix = unique_suffix();
        let cpp_member = |name: &str, offset: u32| CppMessageMember {
            name_: cstr(name),
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
        };
        let members = Box::leak(
            vec![
                cpp_member("x", 0),
                cpp_member("y", 8),
                cpp_member("z", 16),
                cpp_member("w", 24),
            ]
            .into_boxed_slice(),
        );
        let mm = Box::leak(Box::new(CppMessageMembers {
            message_namespace_: cstr("rmw_e2e__msg"),
            message_name_: cstr(&format!("Qcpp{suffix}")),
            member_count_: members.len() as u32,
            size_of_: std::mem::size_of::<CQuat>(),
            has_any_key_member_: false,
            members_: members.as_ptr(),
            init_function: Some(cquat_cpp_init),
            fini_function: None,
        }));
        let ts: *const ffi::rosidl_message_type_support_t =
            Box::leak(Box::new(ffi::rosidl_message_type_support_t {
                typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
                data: mm as *const _ as *const c_void,
                ..Default::default()
            }));

        let (node, publisher, subscription) = setup_loan_pair(ts, "qcpp", suffix);

        CPP_INIT_CALLS.store(0, Ordering::SeqCst);
        CPP_INIT_NON_ALL_ARGS.store(0, Ordering::SeqCst);
        let mut loaned: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut loaned),
            RMW_RET_OK
        );
        assert_eq!(
            CPP_INIT_CALLS.load(Ordering::SeqCst),
            1,
            "borrow must call the cpp init_function EXACTLY once"
        );
        assert_eq!(
            CPP_INIT_NON_ALL_ARGS.load(Ordering::SeqCst),
            0,
            "every cpp init_function call must carry ALL (0)"
        );
        assert_eq!(
            rmw_publish_loaned_message(publisher, loaned, std::ptr::null_mut()),
            RMW_RET_OK
        );

        let mut out = CQuat {
            x: 9.0,
            y: 9.0,
            z: 9.0,
            w: 9.0,
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
        assert!(taken);
        assert_eq!(
            out.w.to_bits(),
            1.0f64.to_bits(),
            "cpp ALL-init default must survive an untouched loan"
        );
        assert_eq!(out.x.to_bits(), 0.0f64.to_bits());
        assert_eq!(out.y.to_bits(), 0.0f64.to_bits());
        assert_eq!(out.z.to_bits(), 0.0f64.to_bits());
        // Rechecked AFTER publish + take: the value oracle above cannot see
        // a publish path that RE-RUNS init (a re-run rewrites the same
        // values) — the count can. Init belongs to borrow, exactly once.
        assert_eq!(
            CPP_INIT_CALLS.load(Ordering::SeqCst),
            1,
            "publish/take must not re-run the cpp init_function"
        );
        assert_eq!(CPP_INIT_NON_ALL_ARGS.load(Ordering::SeqCst), 0);

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// rmw.h (Jazzy, `rmw_borrow_loaned_message`): "RMW_RET_INVALID_ARGUMENT if
/// `*ros_message` is not NULL (to prevent leaks)". Borrow once; a second
/// borrow into the STILL-SET slot is refused, the slot is pointer-equal
/// (untouched), and NO second loan was registered — observable without
/// reaching into `pending_loans`: the outstanding loan still publishes
/// normally (exactly ONE frame reaches the subscriber, defaults intact), and
/// publishing the same handle AGAIN is INVALID_ARGUMENT. Pre-guard, the
/// second borrow would have minted a second loan under a NEW pointer,
/// orphaning the first (its slot pinned for the publisher's life) — and a
/// phantom registration under the SAME key would make that second publish
/// succeed and ship a second frame. Also pins the rmw.h NULL-`type_support`
/// arm (INVALID_ARGUMENT, slot untouched).
#[test]
#[serial]
fn borrow_into_a_non_null_slot_is_refused_to_prevent_leaks() {
    unsafe {
        let suffix = unique_suffix();
        let ts = quat_ts(&format!("Qn{suffix}"));
        let (node, publisher, subscription) = setup_loan_pair(ts, "qnn", suffix);

        C_INIT_CALLS.store(0, Ordering::SeqCst);
        C_INIT_NON_ALL_ARGS.store(0, Ordering::SeqCst);
        let mut loaned: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut loaned),
            RMW_RET_OK
        );
        let first = loaned;
        assert!(!first.is_null());

        // Second borrow into the still-set slot: refused, slot untouched,
        // and NO init ran (the guard sits BEFORE any loan is taken).
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut loaned),
            ffi::RMW_RET_INVALID_ARGUMENT,
            "a non-NULL *ros_message must be refused (rmw.h: to prevent leaks)"
        );
        assert_eq!(
            loaned, first,
            "a refused borrow must leave the caller's slot pointer-equal"
        );
        assert_eq!(
            C_INIT_CALLS.load(Ordering::SeqCst),
            1,
            "a refused borrow must not run the init_function"
        );

        // NULL type_support: INVALID_ARGUMENT, nothing loaned.
        let mut fresh: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, std::ptr::null(), &mut fresh),
            ffi::RMW_RET_INVALID_ARGUMENT
        );
        assert!(fresh.is_null(), "a refused borrow hands out no pointer");

        // The ONE outstanding loan publishes normally, defaults intact.
        assert_eq!(
            rmw_publish_loaned_message(publisher, first, std::ptr::null_mut()),
            RMW_RET_OK
        );
        let mut out = CQuat {
            x: 9.0,
            y: 9.0,
            z: 9.0,
            w: 9.0,
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
        assert!(taken);
        assert_eq!(out.w.to_bits(), 1.0f64.to_bits());

        // Exactly one frame: the queue is now empty ...
        let mut taken2 = true;
        assert_eq!(
            rmw_take(
                subscription,
                &mut out as *mut _ as *mut c_void,
                &mut taken2,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(
            !taken2,
            "a refused borrow must not have minted a second frame"
        );
        // ... and the handle is SPENT: no phantom second registration under
        // the same key survives to be published again.
        assert_eq!(
            rmw_publish_loaned_message(publisher, first, std::ptr::null_mut()),
            ffi::RMW_RET_INVALID_ARGUMENT,
            "exactly one loan was registered for that pointer"
        );

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// Services through the C ABI
// =====================================================================

fn service_ts(unique: &str) -> *const ffi::rosidl_service_type_support_t {
    // Request: { a: f64 }; Response: { sum: f64, msg: string }.
    let req = make_message_ts(
        "rmw_e2e__srv",
        &format!("{unique}_Request"),
        std::mem::size_of::<f64>(),
        vec![member("a", ROS_TYPE_DOUBLE, 0)],
    );
    #[repr(C)]
    struct CResp {
        sum: f64,
        msg: CRosString,
    }
    let resp = make_message_ts(
        "rmw_e2e__srv",
        &format!("{unique}_Response"),
        std::mem::size_of::<CResp>(),
        vec![
            member("sum", ROS_TYPE_DOUBLE, 0),
            member(
                "msg",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CResp, msg) as u32,
            ),
        ],
    );

    let req_members =
        unsafe { (*req).data } as *const ffi::rosidl_typesupport_introspection_c__MessageMembers;
    let resp_members =
        unsafe { (*resp).data } as *const ffi::rosidl_typesupport_introspection_c__MessageMembers;

    let sm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__ServiceMembers {
            service_namespace_: cstr("rmw_e2e__srv"),
            service_name_: cstr(unique),
            request_members_: req_members,
            response_members_: resp_members,
            ..Default::default()
        },
    ));

    let ts = ffi::rosidl_service_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: sm as *const _ as *const c_void,
        ..Default::default()
    };
    Box::leak(Box::new(ts))
}

#[test]
#[serial]
fn c_abi_service_round_trip() {
    unsafe {
        let suffix = unique_suffix();
        let ts = service_ts(&format!("Sum{suffix}"));
        let (_, node, _opts) = setup_node(&format!("svc_node_{suffix}"));

        let service_name = CString::new(format!("/rmw_e2e/sum/{suffix}")).expect("name");
        let qos = default_qos();

        let service = rmw_create_service(node, ts, service_name.as_ptr(), &qos);
        assert!(!service.is_null(), "service creation failed");
        let client = rmw_create_client(node, ts, service_name.as_ptr(), &qos);
        assert!(!client.is_null(), "client creation failed");

        // Server availability through the iceoryx2 global registry.
        let mut available = false;
        assert_eq!(
            rmw_service_server_is_available(node, client, &mut available),
            RMW_RET_OK
        );
        assert!(available, "server must be visible to the client");

        // Request.
        let request: f64 = 20.5;
        let mut sequence_id: i64 = 0;
        assert_eq!(
            rmw_send_request(
                client,
                &request as *const _ as *const c_void,
                &mut sequence_id
            ),
            RMW_RET_OK
        );
        assert_eq!(sequence_id, 1, "first request sequence is 1");

        // Server takes it.
        let mut req_out: f64 = 0.0;
        let mut header: ffi::rmw_service_info_t = std::mem::zeroed();
        let mut taken = false;
        assert_eq!(
            rmw_take_request(
                service,
                &mut header,
                &mut req_out as *mut _ as *mut c_void,
                &mut taken
            ),
            RMW_RET_OK
        );
        assert!(taken);
        assert_eq!(req_out, 20.5);
        assert_eq!(header.request_id.sequence_number, 1);

        // Respond (correlated by the echoed request id).
        #[repr(C)]
        struct CResp {
            sum: f64,
            msg: CRosString,
        }
        let mut response = CResp {
            sum: 41.0,
            msg: CRosString {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            },
        };
        let reply = b"twenty-point-five doubled\0";
        response.msg.data = reply.as_ptr() as *mut u8;
        response.msg.size = reply.len() - 1;
        response.msg.capacity = reply.len();
        assert_eq!(
            rmw_send_response(
                service,
                &mut header.request_id,
                &mut response as *mut _ as *mut c_void
            ),
            RMW_RET_OK
        );

        // Client takes the response.
        let mut resp_out = CResp {
            sum: 0.0,
            msg: CRosString {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            },
        };
        let mut resp_header: ffi::rmw_service_info_t = std::mem::zeroed();
        let mut resp_taken = false;
        assert_eq!(
            rmw_take_response(
                client,
                &mut resp_header,
                &mut resp_out as *mut _ as *mut c_void,
                &mut resp_taken
            ),
            RMW_RET_OK
        );
        assert!(resp_taken);
        assert_eq!(resp_out.sum, 41.0);
        assert_eq!(resp_header.request_id.sequence_number, 1);
        let msg = std::slice::from_raw_parts(resp_out.msg.data, resp_out.msg.size);
        assert_eq!(msg, b"twenty-point-five doubled");

        assert_eq!(rmw_destroy_client(node, client), RMW_RET_OK);
        assert_eq!(rmw_destroy_service(node, service), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Guard conditions wake `rmw_wait` and consume their trigger.
#[test]
#[serial]
fn c_abi_guard_condition_wakes_wait() {
    unsafe {
        let suffix = unique_suffix();
        let (context, node, _opts) = setup_node(&format!("gc_node_{suffix}"));

        let gc = rmw_create_guard_condition(context);
        assert!(!gc.is_null());
        assert_eq!(rmw_trigger_guard_condition(gc), RMW_RET_OK);

        let mut gc_ptrs = [(*gc).data];
        let mut guards = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: gc_ptrs.as_mut_ptr(),
        };
        let timeout = ffi::rmw_time_t {
            sec: 0,
            nsec: 50_000_000,
        };
        let ws = rmw_create_wait_set(context, 4);
        let ret = rmw_wait(
            std::ptr::null_mut(),
            &mut guards,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &timeout,
        );
        assert_eq!(ret, RMW_RET_OK, "triggered guard must wake the wait");
        assert!(!gc_ptrs[0].is_null());

        // Trigger consumed: next wait times out.
        let mut gc_ptrs2 = [(*gc).data];
        let mut guards2 = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: gc_ptrs2.as_mut_ptr(),
        };
        let short = ffi::rmw_time_t {
            sec: 0,
            nsec: 10_000_000,
        };
        let ret2 = rmw_wait(
            std::ptr::null_mut(),
            &mut guards2,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &short,
        );
        assert_eq!(ret2, ffi::RMW_RET_TIMEOUT);
        assert!(gc_ptrs2[0].is_null(), "not-ready guard must be nulled");

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_guard_condition(gc), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Registry mutations (publisher creation) must trigger
/// every node's graph guard condition so executors blocked in rmw_wait
/// re-run graph queries instead of progressing only via timeout.
#[test]
#[serial]
fn c_abi_graph_change_triggers_node_graph_guard() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("PtG{suffix}");
        let ts = point_ts(&type_name);
        let (context, node, _opts) = setup_node(&format!("gg_node_{suffix}"));

        let gg = rmw_node_get_graph_guard_condition(node);
        assert!(!gg.is_null());
        let ws = rmw_create_wait_set(context, 8);

        // Node creation itself announced a graph change — consume it.
        let zero = ffi::rmw_time_t { sec: 0, nsec: 0 };
        let mut ptrs = [(*gg).data];
        let mut guards = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: ptrs.as_mut_ptr(),
        };
        let _ = rmw_wait(
            std::ptr::null_mut(),
            &mut guards,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &zero,
        );

        // Quiescent now: a wait with nothing pending times out.
        let mut ptrs2 = [(*gg).data];
        let mut guards2 = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: ptrs2.as_mut_ptr(),
        };
        assert_eq!(
            rmw_wait(
                std::ptr::null_mut(),
                &mut guards2,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                ws,
                &zero,
            ),
            ffi::RMW_RET_TIMEOUT
        );

        // Mutate the graph: create a publisher → guard must fire.
        let topic = CString::new(format!("/rmw_e2e/gg/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let mut ptrs3 = [(*gg).data];
        let mut guards3 = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: ptrs3.as_mut_ptr(),
        };
        assert_eq!(
            rmw_wait(
                std::ptr::null_mut(),
                &mut guards3,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                ws,
                &zero,
            ),
            RMW_RET_OK,
            "publisher creation must trigger the node graph guard"
        );
        assert!(!ptrs3[0].is_null());

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// rcl passes saturated rmw_time_t values for
/// "effectively infinite" — `Instant + Duration` overflow must not
/// panic. A triggered guard makes the wait return immediately.
#[test]
#[serial]
fn c_abi_wait_survives_overflow_timeout() {
    unsafe {
        let suffix = unique_suffix();
        let (context, node, _opts) = setup_node(&format!("of_node_{suffix}"));

        let gc = rmw_create_guard_condition(context);
        assert!(!gc.is_null());
        assert_eq!(rmw_trigger_guard_condition(gc), RMW_RET_OK);

        let ws = rmw_create_wait_set(context, 8);
        let mut ptrs = [(*gc).data];
        let mut guards = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: ptrs.as_mut_ptr(),
        };
        let huge = ffi::rmw_time_t {
            sec: u64::MAX,
            nsec: u64::MAX,
        };
        let ret = rmw_wait(
            std::ptr::null_mut(),
            &mut guards,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &huge,
        );
        assert_eq!(ret, RMW_RET_OK, "overflow timeout must not panic/abort");

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_guard_condition(gc), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
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

/// A REAL malloc-backed `rcutils_allocator_t` — rcl always passes a
/// functional allocator; a zeroed one (all-None fn pointers) only
/// exercises the BAD_ALLOC path.
fn malloc_allocator() -> ffi::rcutils_allocator_t {
    let mut a: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
    a.allocate = Some(fixture_allocate);
    a.deallocate = Some(fixture_deallocate);
    a
}

/// rmw_take_serialized_message_with_info must fill
/// message_info (timestamps + publication sequence), matching the
/// deserializing take path. The "cerulion" serialized form IS the wire
/// frame, header included.
#[test]
#[serial]
fn c_abi_serialized_take_fills_message_info() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("PtS{suffix}");
        let ts = point_ts(&type_name);
        let (_, node, _opts) = setup_node(&format!("si_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/si/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let msg = CPoint {
            x: 4.0,
            y: 5.0,
            z: 6.0,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );

        let mut serialized: ffi::rmw_serialized_message_t = std::mem::zeroed();
        serialized.allocator = malloc_allocator();
        let mut taken = false;
        let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
        info.publication_sequence_number = 12345; // poison: must be overwritten
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
        assert!(taken, "published frame must be taken");
        assert_eq!(info.publication_sequence_number, 0, "first publish = seq 0");
        assert!(
            info.source_timestamp > 0,
            "source timestamp must be stamped"
        );
        assert!(
            info.received_timestamp > 0,
            "received timestamp must be stamped"
        );
        assert!(serialized.buffer_length > 0);

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Variable-message C ABI path: publish/take of a message with a
/// `string` field exercises flatten (scattered → flat frame) and
/// unflatten (libc-allocated assignment into the caller's struct) —
/// the path every real MoveIt message with strings/sequences takes.
#[test]
#[serial]
fn c_abi_variable_message_publish_take_round_trip() {
    #[repr(C)]
    struct CLabeled {
        id: f64,
        label: CRosString,
    }

    unsafe {
        let suffix = unique_suffix();
        let ts = make_message_ts(
            "rmw_e2e__msg",
            &format!("Lbl{suffix}"),
            std::mem::size_of::<CLabeled>(),
            vec![
                member("id", ROS_TYPE_DOUBLE, 0),
                member("label", ROS_TYPE_STRING, 8),
            ],
        );
        let (_, node, _opts) = setup_node(&format!("var_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/var/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let text = "panda_link0";
        let label_buf = malloc(text.len() + 1) as *mut u8;
        std::ptr::copy_nonoverlapping(text.as_ptr(), label_buf, text.len());
        *label_buf.add(text.len()) = 0;
        let msg = CLabeled {
            id: 42.0,
            label: CRosString {
                data: label_buf,
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

        // Take into an INITIALIZED (empty-string) message, per contract.
        let mut out = CLabeled {
            id: 0.0,
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
        assert!(taken, "variable message must be taken");
        assert_eq!(out.id, 42.0);
        assert!(!out.label.data.is_null());
        assert_eq!(out.label.size, text.len());
        let got = std::slice::from_raw_parts(out.label.data, out.label.size);
        assert_eq!(got, text.as_bytes());

        free(out.label.data as *mut c_void);
        free(label_buf as *mut c_void);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Event pump: an IDLE transient-local publisher must deliver its
/// history to a late joiner purely via the rmw_wait pump — no publish
/// happens after the subscriber exists, so the pump is the only
/// delivery mechanism (the ros2_control robot_description hang).
#[test]
#[serial]
fn transient_local_history_reaches_late_joiner_via_wait_pump() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("PtL{suffix}");
        let ts = point_ts(&type_name);
        let (context, node, _opts) = setup_node(&format!("tl_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/tl/{suffix}")).expect("topic");
        let mut qos = default_qos();
        qos.durability = ffi::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        qos.depth = 4;
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        // Publish BEFORE the subscriber exists — lands in history.
        let msg = CPoint {
            x: 7.0,
            y: 8.0,
            z: 9.0,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );

        // Late joiner. The publisher is idle from here on.
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());

        // rmw_wait loops: the pump (20ms throttle) must process the
        // SubscriberJoined event and deliver history.
        let ws = rmw_create_wait_set(context, 8);
        let mut got = false;
        for _ in 0..40 {
            let mut sub_ptrs = [(*subscription).data];
            let mut subs = ffi::rmw_subscriptions_t {
                subscriber_count: 1,
                subscribers: sub_ptrs.as_mut_ptr(),
            };
            let timeout = ffi::rmw_time_t {
                sec: 0,
                nsec: 50_000_000,
            };
            let ret = rmw_wait(
                &mut subs,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                ws,
                &timeout,
            );
            if ret == RMW_RET_OK && !sub_ptrs[0].is_null() {
                got = true;
                break;
            }
        }
        assert!(got, "history must reach the late joiner via the wait pump");

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

        // Lifecycle: destroy the publisher, then wait again — the pump
        // must never touch the freed PublisherData (unregister-first).
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
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

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Negative control for the pump test: with VOLATILE durability there
/// is no history — the late joiner must NOT receive the pre-join
/// message (proves the transient-local delivery above came from the
/// history pump, not from some stray republish).
#[test]
#[serial]
fn volatile_publisher_does_not_deliver_history_to_late_joiner() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("PtV{suffix}");
        let ts = point_ts(&type_name);
        let (context, node, _opts) = setup_node(&format!("vol_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/vol/{suffix}")).expect("topic");
        let qos = default_qos(); // VOLATILE
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        let msg = CPoint {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);

        let ws = rmw_create_wait_set(context, 8);
        let mut sub_ptrs = [(*subscription).data];
        let mut subs = ffi::rmw_subscriptions_t {
            subscriber_count: 1,
            subscribers: sub_ptrs.as_mut_ptr(),
        };
        let timeout = ffi::rmw_time_t {
            sec: 0,
            nsec: 200_000_000,
        };
        let ret = rmw_wait(
            &mut subs,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &timeout,
        );
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT, "no history for VOLATILE");

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Flatten-into-loan e2e: a 1 MiB std_msgs/UInt8MultiArray-shaped
/// message (nested complex layout + huge uint8[] payload) published
/// through the NEW loan path (VOLATILE ⇒ frame_size → uninit SHM loan →
/// flatten_into → send), then byte-compared on the subscriber side:
///
/// 1. rmw_take roundtrip — every payload byte + dim metadata survives.
/// 2. rmw_take_serialized — the wire frame on the wire is byte-identical
///    to the bridge's heap `flatten` oracle (header timestamp excluded:
///    it comes from the live clock).
#[test]
#[serial]
fn c_abi_1mb_uint8_multi_array_publish_take_byte_identical() {
    use cerulion_core::wire::WireHeader;
    use rmw_cerulion::type_bridge::BridgedMessage;

    const ROS_TYPE_UINT32: u8 = 12;
    const ROS_TYPE_UINT8: u8 = 8;
    const ROS_TYPE_MESSAGE: u8 = 18;

    #[repr(C)]
    struct CSeq {
        data: *mut c_void,
        size: usize,
        capacity: usize,
    }
    /// std_msgs/MultiArrayDimension: { label: string, size: u32, stride: u32 }
    #[repr(C)]
    struct CDim {
        label: CRosString,
        size: u32,
        stride: u32,
    }
    /// std_msgs/MultiArrayLayout: { dim: MultiArrayDimension[], data_offset: u32 }
    #[repr(C)]
    struct CLayout {
        dim: CSeq,
        data_offset: u32,
    }
    /// std_msgs/UInt8MultiArray: { layout: MultiArrayLayout, data: uint8[] }
    #[repr(C)]
    struct CU8Ma {
        layout: CLayout,
        data: CSeq,
    }

    fn full_member(
        name: &str,
        type_id: u8,
        offset: u32,
        is_array: bool,
        nested: *const ffi::rosidl_message_type_support_t,
    ) -> ffi::rosidl_typesupport_introspection_c__MessageMember {
        ffi::rosidl_typesupport_introspection_c__MessageMember {
            name_: cstr(name),
            type_id_: type_id,
            members_: nested,
            is_array_: is_array,
            offset_: offset,
            ..Default::default()
        }
    }
    fn make_members_and_ts(
        namespace: &str,
        name: &str,
        size_of: usize,
        members: Vec<ffi::rosidl_typesupport_introspection_c__MessageMember>,
    ) -> (
        *const ffi::rosidl_typesupport_introspection_c__MessageMembers,
        *const ffi::rosidl_message_type_support_t,
    ) {
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
        let ts = Box::leak(Box::new(ffi::rosidl_message_type_support_t {
            typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
            data: mm as *const _ as *const c_void,
            ..Default::default()
        }));
        (mm, ts)
    }

    unsafe {
        let suffix = unique_suffix();
        let (_, dim_ts) = make_members_and_ts(
            "rmw_e2e__msg",
            &format!("Dim{suffix}"),
            std::mem::size_of::<CDim>(),
            vec![
                full_member("label", ROS_TYPE_STRING, 0, false, std::ptr::null()),
                full_member(
                    "size",
                    ROS_TYPE_UINT32,
                    std::mem::offset_of!(CDim, size) as u32,
                    false,
                    std::ptr::null(),
                ),
                full_member(
                    "stride",
                    ROS_TYPE_UINT32,
                    std::mem::offset_of!(CDim, stride) as u32,
                    false,
                    std::ptr::null(),
                ),
            ],
        );
        let (_, layout_ts) = make_members_and_ts(
            "rmw_e2e__msg",
            &format!("Lay{suffix}"),
            std::mem::size_of::<CLayout>(),
            vec![
                full_member("dim", ROS_TYPE_MESSAGE, 0, true, dim_ts),
                full_member(
                    "data_offset",
                    ROS_TYPE_UINT32,
                    std::mem::offset_of!(CLayout, data_offset) as u32,
                    false,
                    std::ptr::null(),
                ),
            ],
        );
        let (u8ma_members, u8ma_ts) = make_members_and_ts(
            "rmw_e2e__msg",
            &format!("U8Ma{suffix}"),
            std::mem::size_of::<CU8Ma>(),
            vec![
                full_member("layout", ROS_TYPE_MESSAGE, 0, false, layout_ts),
                full_member(
                    "data",
                    ROS_TYPE_UINT8,
                    std::mem::offset_of!(CU8Ma, data) as u32,
                    true,
                    std::ptr::null(),
                ),
            ],
        );

        let (_, node, _opts) = setup_node(&format!("u8ma_node_{suffix}"));
        let topic = CString::new(format!("/rmw_e2e/u8ma/{suffix}")).expect("topic");
        let qos = default_qos(); // VOLATILE ⇒ the flatten-into-loan path
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, u8ma_ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, u8ma_ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        // 1 MiB deterministic payload + two labeled dims.
        let heap_str = |s: &str| -> CRosString {
            let buf = malloc(s.len() + 1) as *mut u8;
            std::ptr::copy_nonoverlapping(s.as_ptr(), buf, s.len());
            *buf.add(s.len()) = 0;
            CRosString {
                data: buf,
                size: s.len(),
                capacity: s.len() + 1,
            }
        };
        const PAYLOAD: usize = 1 << 20;
        let mut payload: Vec<u8> = (0..PAYLOAD)
            .map(|i| ((i.wrapping_mul(131)) ^ (i >> 8)) as u8)
            .collect();
        let mut dims = [
            CDim {
                label: heap_str("rows"),
                size: 1024,
                stride: PAYLOAD as u32,
            },
            CDim {
                label: heap_str("cols"),
                size: 1024,
                stride: 1024,
            },
        ];
        let msg = CU8Ma {
            layout: CLayout {
                dim: CSeq {
                    data: dims.as_mut_ptr() as *mut c_void,
                    size: dims.len(),
                    capacity: dims.len(),
                },
                data_offset: 0,
            },
            data: CSeq {
                data: payload.as_mut_ptr() as *mut c_void,
                size: payload.len(),
                capacity: payload.len(),
            },
        };

        // ---- Publish #1 (seq 0) → rmw_take roundtrip ----
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        let mut out = CU8Ma {
            layout: CLayout {
                dim: CSeq {
                    data: std::ptr::null_mut(),
                    size: 0,
                    capacity: 0,
                },
                data_offset: 0,
            },
            data: CSeq {
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
        assert!(taken, "1 MiB message must be taken");
        assert_eq!(out.data.size, PAYLOAD);
        let got = std::slice::from_raw_parts(out.data.data as *const u8, out.data.size);
        assert_eq!(
            got,
            &payload[..],
            "1 MiB payload must survive byte-for-byte"
        );
        assert_eq!(out.layout.dim.size, 2);
        let out_dims = std::slice::from_raw_parts(out.layout.dim.data as *const CDim, 2);
        assert_eq!(
            std::slice::from_raw_parts(out_dims[0].label.data, out_dims[0].label.size),
            b"rows"
        );
        assert_eq!(out_dims[0].size, 1024);
        assert_eq!(out_dims[0].stride, PAYLOAD as u32);
        assert_eq!(
            std::slice::from_raw_parts(out_dims[1].label.data, out_dims[1].label.size),
            b"cols"
        );

        // ---- Publish #2 (seq 1) → serialized take: the frame on the
        // wire must be byte-identical to the heap-flatten oracle ----
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        let mut serialized: ffi::rmw_serialized_message_t = std::mem::zeroed();
        serialized.allocator = malloc_allocator();
        let mut taken2 = false;
        assert_eq!(
            rmw_take_serialized_message(
                subscription,
                &mut serialized,
                &mut taken2,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(taken2, "serialized take must see publish #2");
        let wire =
            std::slice::from_raw_parts(serialized.buffer as *const u8, serialized.buffer_length);

        let oracle_bridge = BridgedMessage::new(u8ma_members).expect("oracle bridge");
        let expected = oracle_bridge
            .flatten(&msg as *const _ as *const c_void, 1, 0)
            .expect("oracle flatten");
        assert_eq!(wire.len(), expected.len(), "frame length must match");
        assert_eq!(
            &wire[WireHeader::SIZE..],
            &expected[WireHeader::SIZE..],
            "payload (fixed + offset table + variable) must be byte-identical"
        );
        let wire_h = WireHeader::read_from_buf(wire).expect("wire header");
        let exp_h = WireHeader::read_from_buf(&expected).expect("oracle header");
        assert_eq!(wire_h.schema_hash, exp_h.schema_hash);
        assert_eq!(wire_h.total_size, exp_h.total_size);
        assert_eq!(wire_h.offset_table_offset, exp_h.offset_table_offset);
        assert_eq!(wire_h.offset_table_count, exp_h.offset_table_count);
        assert_eq!(wire_h.sequence, 1, "second publish must carry seq 1");
        // timestamp_ns comes from the live clock — not compared.

        // Cleanup (libc-allocated by unflatten / fixtures).
        for d in out_dims {
            free(d.label.data as *mut c_void);
        }
        free(out.layout.dim.data);
        free(out.data.data);
        for d in &dims {
            free(d.label.data as *mut c_void);
        }
        free(serialized.buffer as *mut c_void);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// `rmw_destroy_node` must UNREGISTER its graph guard from
/// `Runtime::graph_guards` before freeing it — a failed `retain` would leave a
/// dangling `*const GuardConditionState` that the next `notify_graph_change`
/// derefs (use-after-free). The other guard tests run the unregister but never
/// assert it; this one does, with two nodes so a leaked entry is caught: the
/// registry must shrink by exactly one on destroy, and the surviving node's
/// guard must still fire afterward (exercises `notify_graph_change` over the
/// post-destroy registry — the UAF path — and proves B stayed live).
#[test]
#[serial]
fn c_abi_destroy_node_unregisters_its_graph_guard() {
    let guard_count = || {
        rmw_cerulion::runtime::runtime()
            .expect("runtime initialized")
            .graph_guards
            .lock()
            .expect("graph_guards lock")
            .len()
    };
    unsafe {
        let suffix = unique_suffix();

        // Two nodes (A + B) in one context. setup_node makes the context + A;
        // B is a second node on the same context.
        let (context, node_a, _opts) = setup_node(&format!("unreg_a_{suffix}"));
        let after_a = guard_count();
        let node_b = rmw_create_node(context, cstr(&format!("unreg_b_{suffix}")), cstr("/"));
        assert!(!node_b.is_null(), "second node creation failed");
        assert_eq!(
            guard_count(),
            after_a + 1,
            "creating node B must register exactly one more graph guard"
        );

        // Destroy A — its guard must be unregistered (registry shrinks by one).
        // A failed retain would leave A's freed guard registered (count stays
        // after_a + 1) → use-after-free on the next notify_graph_change.
        assert_eq!(rmw_destroy_node(node_a), RMW_RET_OK);
        assert_eq!(
            guard_count(),
            after_a,
            "rmw_destroy_node must unregister exactly the destroyed node's guard"
        );

        // Survivor B's guard must still fire on a graph change — proving B
        // stayed registered AND that triggering all guards after A's destroy
        // did not deref A's freed pointer.
        let gg_b = rmw_node_get_graph_guard_condition(node_b);
        assert!(!gg_b.is_null());
        let ws = rmw_create_wait_set(context, 8);
        let zero = ffi::rmw_time_t { sec: 0, nsec: 0 };

        // Drain the graph changes already pending (B's creation + A's destroy).
        let mut drain = [(*gg_b).data];
        let mut drain_gc = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: drain.as_mut_ptr(),
        };
        let _ = rmw_wait(
            std::ptr::null_mut(),
            &mut drain_gc,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &zero,
        );

        // Fresh graph change on B → B's guard must fire.
        let ts = point_ts(&format!("UnregG{suffix}"));
        let topic = CString::new(format!("/rmw_e2e/unreg/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node_b, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let mut fired = [(*gg_b).data];
        let mut fired_gc = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: fired.as_mut_ptr(),
        };
        assert_eq!(
            rmw_wait(
                std::ptr::null_mut(),
                &mut fired_gc,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                ws,
                &zero,
            ),
            RMW_RET_OK,
            "survivor node B's guard must still fire after peer A was destroyed"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node_b, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node_b), RMW_RET_OK);
        assert_eq!(
            guard_count(),
            after_a - 1,
            "both nodes now destroyed → registry back to its pre-A baseline \
             (neither test node registered)"
        );
    }
}

// =====================================================================
// Zero-copy loaned TAKE (the sample held across the C ABI)
// =====================================================================

/// Loaned take round trip: publish via the publish-side loan, take via
/// `rmw_take_loaned_message_with_info` — the pointer aims DIRECTLY into
/// the SHM frame (past the WireHeader), field values byte-exact against
/// the hand oracle, message_info exact against the wire header read back
/// through that same pointer, return releases, and the queue is then
/// empty. Also pins the unknown-pointer return arm.
#[test]
#[serial]
fn c_abi_loaned_take_round_trip_and_return() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("PtLT{suffix}");
        let ts = point_ts(&type_name);
        let (_, node, _opts) = setup_node(&format!("ltake_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/ltake/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        assert!(
            (*subscription).can_loan_messages,
            "fixed POD type must loan on the take side too"
        );
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        // Publish through the loan lane (zero-copy end to end).
        let mut loaned: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut loaned),
            RMW_RET_OK
        );
        let point = &mut *(loaned as *mut CPoint);
        point.x = 7.0;
        point.y = -8.5;
        point.z = 0.25;
        assert_eq!(
            rmw_publish_loaned_message(publisher, loaned, std::ptr::null_mut()),
            RMW_RET_OK
        );

        // Loaned take with info.
        let mut taken = false;
        let mut out: *mut c_void = std::ptr::null_mut();
        let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
        info.publication_sequence_number = 12345; // poison: must be overwritten
        assert_eq!(
            rmw_take_loaned_message_with_info(
                subscription,
                &mut out,
                &mut taken,
                &mut info,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(taken, "published frame must be taken as a loan");
        assert!(!out.is_null());
        // The pointer is 8-aligned (the de facto iceoryx2 guarantee the
        // rmw asserts fail-closed at hand-out).
        assert_eq!(out as usize % 8, 0, "loaned pointer must be 8-aligned");
        // Byte-exact values straight out of SHM.
        let got = &*(out as *const CPoint);
        assert_eq!(
            *got,
            CPoint {
                x: 7.0,
                y: -8.5,
                z: 0.25
            }
        );
        // message_info: source_timestamp is EXACTLY the wire timestamp —
        // cross-checked against the WireHeader sitting 32 bytes before the
        // loaned pointer in the SAME held frame.
        let header_bytes = std::slice::from_raw_parts(
            (out as *const u8).sub(cerulion_core::wire::WireHeader::SIZE),
            cerulion_core::wire::WireHeader::SIZE,
        );
        let header = cerulion_core::wire::WireHeader::read_from_buf(header_bytes)
            .expect("wire header before the loaned payload");
        assert_eq!(info.source_timestamp, header.timestamp_ns as i64);
        assert_eq!(info.publication_sequence_number, 0, "first publish = seq 0");
        assert_eq!(info.publication_sequence_number, u64::from(header.sequence));
        assert!(info.received_timestamp > 0, "received timestamp stamped");
        assert_eq!(info.reception_sequence_number, u64::MAX);

        // rmw.h contract: a take into an out-slot that is NOT NULL is
        // INVALID_ARGUMENT "to prevent leaks" — the slot still holds the
        // outstanding loan's only handle. Refused BEFORE any receive: a
        // second frame published now must survive the refused call, and the
        // slot must be left untouched.
        let second = CPoint {
            x: 20.0,
            y: 21.0,
            z: 22.0,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &second as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        let held_before = out;
        let mut reused_taken = true;
        assert_eq!(
            rmw_take_loaned_message(
                subscription,
                &mut out,
                &mut reused_taken,
                std::ptr::null_mut()
            ),
            ffi::RMW_RET_INVALID_ARGUMENT,
            "a non-NULL *loaned_message on entry is refused (rmw.h: to prevent leaks)"
        );
        assert_eq!(
            out, held_before,
            "the refused call must leave the slot untouched"
        );

        // Return the loan; a second return of the SAME pointer is an
        // unknown-loan INVALID_ARGUMENT (loud, never a double-free).
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, out),
            RMW_RET_OK
        );
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, out),
            ffi::RMW_RET_INVALID_ARGUMENT
        );

        // The second frame was NOT consumed by the refused call: a take into
        // a NULL slot serves it, byte-exact.
        let mut taken2 = false;
        let mut out2: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_take_loaned_message(subscription, &mut out2, &mut taken2, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert!(
            taken2,
            "the refused non-NULL-slot call must not have consumed a frame"
        );
        assert_eq!(*(out2 as *const CPoint), second);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, out2),
            RMW_RET_OK
        );

        // Queue is now empty: nothing further to take.
        let mut taken3 = true;
        let mut out3: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_take_loaned_message(subscription, &mut out3, &mut taken3, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert!(!taken3, "queue must be empty after both loaned takes");
        assert!(out3.is_null());

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// HOLD a loaned take across further publishes: the held payload is
/// byte-UNCHANGED (iceoryx2 borrow accounting pins the slot until the
/// loan returns — the no-reclaim-while-held pin), then the remaining
/// frames drain IN ORDER.
#[test]
#[serial]
fn c_abi_loaned_take_hold_across_publish_pins_the_slot() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("PtLH{suffix}");
        let ts = point_ts(&type_name);
        let (_, node, _opts) = setup_node(&format!("lhold_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/lhold/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);

        let first = CPoint {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &first as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );

        let mut taken = false;
        let mut held: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_take_loaned_message(subscription, &mut held, &mut taken, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert!(taken);
        let snapshot = *(held as *const CPoint);
        assert_eq!(snapshot, first);

        // Publish three more frames while the loan is outstanding.
        for i in 0..3u32 {
            let msg = CPoint {
                x: 10.0 + f64::from(i),
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

        // The held payload must be byte-identical to the pre-publish
        // snapshot: the slot cannot be reclaimed while the sample lives.
        assert_eq!(
            *(held as *const CPoint),
            snapshot,
            "held loan bytes must not change while later frames publish"
        );
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, held),
            RMW_RET_OK
        );

        // Drain the rest in publish order, sequence numbers ascending.
        for i in 0..3u32 {
            let mut t = false;
            let mut p: *mut c_void = std::ptr::null_mut();
            let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
            assert_eq!(
                rmw_take_loaned_message_with_info(
                    subscription,
                    &mut p,
                    &mut t,
                    &mut info,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            assert!(t, "frame {i} must still be queued");
            assert_eq!((*(p as *const CPoint)).x, 10.0 + f64::from(i), "FIFO order");
            assert_eq!(info.publication_sequence_number, 1 + u64::from(i));
            assert_eq!(
                rmw_return_loaned_message_from_subscription(subscription, p),
                RMW_RET_OK
            );
        }

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Borrow-budget exhaustion is LOUD and bounded: with the whole
/// RMW_TAKE_LOAN_BORROW_BUDGET (4) held, the next loaned take fails with
/// RMW_RET_ERROR (no silent loss, no hang — the queued frames survive);
/// returning ONE loan recovers the next take, still in FIFO order. The
/// exact 4/5 boundary also behaviorally pins that the rmw create really
/// provisioned the service at the documented budget.
#[test]
#[serial]
fn c_abi_loaned_take_budget_exhaustion_is_loud_and_recovers() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("PtLB{suffix}");
        let ts = point_ts(&type_name);
        let (_, node, _opts) = setup_node(&format!("lbudget_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/lbudget/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);

        for i in 0..5u32 {
            let msg = CPoint {
                x: f64::from(i),
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

        // Hold the whole budget: 4 concurrent loans, FIFO values 0..=3.
        let mut held: Vec<*mut c_void> = Vec::new();
        for i in 0..4u32 {
            let mut t = false;
            let mut p: *mut c_void = std::ptr::null_mut();
            assert_eq!(
                rmw_take_loaned_message(subscription, &mut p, &mut t, std::ptr::null_mut()),
                RMW_RET_OK,
                "take {i} within the budget must succeed"
            );
            assert!(t);
            assert_eq!((*(p as *const CPoint)).x, f64::from(i));
            held.push(p);
        }

        // The 5th concurrent loan is refused loudly — RMW_RET_ERROR, taken
        // stays false, and the queued 5th frame is NOT lost.
        let mut t5 = false;
        let mut p5: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_take_loaned_message(subscription, &mut p5, &mut t5, std::ptr::null_mut()),
            ffi::RMW_RET_ERROR,
            "a take past the borrow budget must fail loudly, never hang"
        );
        assert!(!t5);
        assert!(p5.is_null(), "no pointer may escape a refused take");

        // Returning ONE loan recovers; the next take serves the surviving
        // 5th frame (x = 4.0).
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, held.remove(0)),
            RMW_RET_OK
        );
        let mut t = false;
        let mut p: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_take_loaned_message(subscription, &mut p, &mut t, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert!(t, "returning a loan must recover the take path");
        assert_eq!(
            (*(p as *const CPoint)).x,
            4.0,
            "the frame behind the refused take must not have been lost"
        );
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p),
            RMW_RET_OK
        );
        for p in held {
            assert_eq!(
                rmw_return_loaned_message_from_subscription(subscription, p),
                RMW_RET_OK
            );
        }

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Control: a VARIABLE type (string member) keeps can_loan_messages =
/// false on BOTH sides and every loan verb returns RMW_RET_UNSUPPORTED —
/// rclcpp falls back to the copying paths.
#[test]
#[serial]
fn c_abi_loaned_verbs_unsupported_for_variable_types() {
    #[repr(C)]
    struct CLabeledLT {
        id: f64,
        label: CRosString,
    }

    unsafe {
        let suffix = unique_suffix();
        let ts = make_message_ts(
            "rmw_e2e__msg",
            &format!("VarLT{suffix}"),
            std::mem::size_of::<CLabeledLT>(),
            vec![
                member("id", ROS_TYPE_DOUBLE, 0),
                member(
                    "label",
                    ROS_TYPE_STRING,
                    std::mem::offset_of!(CLabeledLT, label) as u32,
                ),
            ],
        );
        let (_, node, _opts) = setup_node(&format!("lvar_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/lvar/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(
            !(*subscription).can_loan_messages,
            "variable types must not advertise take loans"
        );
        assert!(!(*publisher).can_loan_messages);

        let mut taken = false;
        let mut out: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_take_loaned_message(subscription, &mut out, &mut taken, std::ptr::null_mut()),
            ffi::RMW_RET_UNSUPPORTED
        );
        assert!(!taken);
        let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
        assert_eq!(
            rmw_take_loaned_message_with_info(
                subscription,
                &mut out,
                &mut taken,
                &mut info,
                std::ptr::null_mut()
            ),
            ffi::RMW_RET_UNSUPPORTED
        );
        assert!(!taken, "an UNSUPPORTED take must not report a take");
        assert!(
            out.is_null(),
            "an UNSUPPORTED take must not hand out a pointer"
        );
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, 0x8 as *mut c_void),
            ffi::RMW_RET_UNSUPPORTED
        );
        let mut borrow: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts, &mut borrow),
            ffi::RMW_RET_UNSUPPORTED
        );

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Destroy with an outstanding take loan: the subscription releases the
/// held sample (no leak, no abort — warned in the log), and the topic
/// stays healthy for a fresh subscription afterwards.
#[test]
#[serial]
fn c_abi_destroy_subscription_with_outstanding_loan_releases() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("PtLD{suffix}");
        let ts = point_ts(&type_name);
        let (_, node, _opts) = setup_node(&format!("ldestroy_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/ldestroy/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);

        let msg = CPoint {
            x: 5.0,
            y: 6.0,
            z: 7.0,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        let mut taken = false;
        let mut held: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_take_loaned_message(subscription, &mut held, &mut taken, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert!(taken);

        // Destroy WITH the loan outstanding: must succeed (the held sample
        // is dropped with the subscription — borrow released, never leaked)
        // and must not abort the process.
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);

        // The topic is healthy afterwards: a fresh subscription attaches
        // (the freed borrow/subscriber slots are reusable) and a fresh
        // publish round-trips through the loaned take.
        let sub2 = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(
            !sub2.is_null(),
            "fresh subscription after loan-holding destroy"
        );
        let msg2 = CPoint {
            x: 9.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg2 as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        let mut t2 = false;
        let mut p2: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_take_loaned_message(sub2, &mut p2, &mut t2, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert!(t2);
        assert_eq!((*(p2 as *const CPoint)).x, 9.0);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(sub2, p2),
            RMW_RET_OK
        );

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, sub2), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Read the subscription's DECODE-failure counter (the framing latch the
/// loaned take's refusals ride). Unconditional, log-level independent.
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

/// Serialize `msg` through the REAL `rmw_serialize` into a malloc-backed
/// buffer the test may then MUTATE — the shape a bag player hands
/// `rmw_publish_serialized_message`.
///
/// # Safety
/// `ts` must be the typesupport `msg` was built against.
unsafe fn serialize_point(
    msg: &CPoint,
    ts: *const ffi::rosidl_message_type_support_t,
) -> ffi::rmw_serialized_message_t {
    let mut out: ffi::rmw_serialized_message_t = std::mem::zeroed();
    out.allocator = malloc_allocator();
    assert_eq!(
        rmw_serialize(msg as *const _ as *const c_void, ts, &mut out),
        RMW_RET_OK,
        "rmw_serialize must produce a frame"
    );
    assert!(out.buffer_length >= cerulion_core::wire::WireHeader::SIZE);
    out
}

/// A same-hash fixed-size frame whose header carries NONZERO offset-table
/// metadata must be REFUSED by the loaned take (consumed, dropped, counted
/// on the decode latch, `taken` false, no pointer) — never handed out as a
/// zero-copy C struct. `rmw_publish_serialized_message` only validates the
/// header parse + hash, so such a frame really can reach the wire from a
/// bag player. Both arms: a nonzero `offset_table_count`, and an
/// `offset_table_offset` past the frame. The copying-take reference and a
/// clean frame afterwards prove the queue is not wedged.
#[test]
#[serial]
fn c_abi_loaned_take_refuses_nonzero_offset_table_metadata() {
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("PtLO{suffix}");
        let ts = point_ts(&type_name);
        let (_, node, _opts) = setup_node(&format!("loffset_node_{suffix}"));

        let topic = CString::new(format!("/rmw_e2e/loffset/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert_eq!(decode_failure_count(subscription), 0);

        let msg = CPoint {
            x: 3.0,
            y: 4.0,
            z: 5.0,
        };

        // Arm 1: same hash, right length, offset_table_count = 1 (bytes
        // [16..20] of the WireHeader, little-endian).
        let crafted = serialize_point(&msg, ts);
        let bytes = std::slice::from_raw_parts_mut(crafted.buffer, crafted.buffer_length);
        bytes[16..20].copy_from_slice(&1u32.to_le_bytes());
        assert_eq!(
            rmw_publish_serialized_message(publisher, &crafted, std::ptr::null_mut()),
            RMW_RET_OK,
            "the serialized-publish entry point accepts a same-hash frame"
        );
        let mut taken = true;
        let mut out: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_take_loaned_message(subscription, &mut out, &mut taken, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert!(
            !taken,
            "a frame claiming offset-table entries must not be loaned"
        );
        assert!(out.is_null(), "no pointer may escape a refused frame");
        assert_eq!(
            decode_failure_count(subscription),
            1,
            "the refusal must be counted on the decode/framing latch"
        );

        // Arm 2: count 0 but an offset_table_offset (bytes [12..16]) far
        // past the frame — malformed regardless of convention.
        let crafted2 = serialize_point(&msg, ts);
        let bytes2 = std::slice::from_raw_parts_mut(crafted2.buffer, crafted2.buffer_length);
        bytes2[12..16].copy_from_slice(&u32::MAX.to_le_bytes());
        assert_eq!(
            rmw_publish_serialized_message(publisher, &crafted2, std::ptr::null_mut()),
            RMW_RET_OK
        );
        let mut taken2 = true;
        let mut out2: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_take_loaned_message(subscription, &mut out2, &mut taken2, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert!(!taken2);
        assert!(out2.is_null());
        assert_eq!(decode_failure_count(subscription), 2);

        // Control: the refused frames were CONSUMED (not left wedging the
        // queue) and a clean frame afterwards is loaned normally — the
        // native producers' offset conventions (both count 0) pass the gate.
        let clean = serialize_point(&msg, ts);
        assert_eq!(
            rmw_publish_serialized_message(publisher, &clean, std::ptr::null_mut()),
            RMW_RET_OK
        );
        let mut taken3 = false;
        let mut out3: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_take_loaned_message(subscription, &mut out3, &mut taken3, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert!(taken3, "a clean same-hash frame must still be loaned");
        assert_eq!(*(out3 as *const CPoint), msg);
        assert_eq!(
            decode_failure_count(subscription),
            2,
            "a good frame never counts"
        );
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, out3),
            RMW_RET_OK
        );

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
