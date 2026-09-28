// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The loaned-take REFUSAL flood latch at the production
//! `rmw_take_loaned_message` site — both budget arms.
//!
//! A consumer that RETAINS loans past the budget is refused on every
//! following take, and rclcpp's executor retries at the topic's arrival
//! rate: on a 30 Hz camera topic that is 30 lines/s forever under a
//! bare `error!`. The site rides `cerulion_core`'s shared
//! `FailureRegimeLatch` through `loan_refusal_latch`. Two arms, ONE latch:
//!
//! - `kind=shadow_pool_exhausted` — a FORGED type whose four shadows are all
//!   loaned out; refused BEFORE the receive (no frame consumed);
//! - `kind=receive_failed` — a FIXED type at the transport's borrow budget
//!   (iceoryx2's `ExceedsMaxBorrows`), the arm that originally shipped unlatched.
//!
//! Hand oracles, every predicate matching the LEVEL TOKEN as well as the
//! message (a text-only filter passes a variant that emits the suppressed
//! arm at `error!`, so suppression does nothing while message and
//! counter stay intact): N refusals ⇒ exactly 1 `ERROR` head + N-1 `DEBUG`
//! repeats (+ the `ERROR` decade re-announcement when N crosses 10, which is
//! NOT counted as suppressed) + an UNCONDITIONAL counter of N; a return +
//! served take ⇒ exactly ONE `INFO` recovery carrying `suppressed_count=`
//! the suppressed number (not the total); the counter is never reset; a
//! fresh refusal is loud again. Field keys are pinned as whole tokens:
//! `topic=`, `kind=`, `total_failures=`, `suppressed_count=`.
//!
//! Its OWN binary: `#[traced_test]` installs the process-global subscriber
//! and `runtime()` installs its own with `try_init` — a non-traced test
//! running first would take the slot. Every test here is `#[traced_test]`
//! and `#[serial]`; run with `--test-threads=1` (SHM singleton):
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_loan_refusal_latch_test -- --test-threads=1
//! ```

use cerulion_core::testing::{
    count_at, count_at_exclusively, debug_lines_expected, line_level, never_loud,
};
use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};

use rmw_cerulion::ffi::{self, RMW_RET_ERROR, RMW_RET_OK};
use rmw_cerulion::runtime::SubscriptionData;
use rmw_cerulion::*;
use tracing_test::traced_test;

// =====================================================================
// Fixtures
// =====================================================================

const ROS_TYPE_DOUBLE: u8 = 2;
const ROS_TYPE_UINT8: u8 = 8;
const ROS_TYPE_UINT32: u8 = 12;

extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

static UNIQUE: AtomicU64 = AtomicU64::new(0);

fn unique_suffix() -> u64 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos() as u64;
    nanos ^ UNIQUE.fetch_add(1, Ordering::Relaxed)
}

fn c_member(
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

#[allow(clippy::type_complexity)]
fn c_ts(
    name: &str,
    size_of: usize,
    members: Vec<ffi::rosidl_typesupport_introspection_c__MessageMember>,
    init: Option<unsafe extern "C" fn(*mut c_void, ffi::rosidl_runtime_c__message_initialization)>,
    fini: Option<unsafe extern "C" fn(*mut c_void)>,
) -> *const ffi::rosidl_message_type_support_t {
    let members = Box::leak(members.into_boxed_slice());
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("rmw_refusal__msg"),
            message_name_: cstr(name),
            member_count_: members.len() as u32,
            size_of_: size_of,
            members_: members.as_ptr(),
            init_function: init,
            fini_function: fini,
            ..Default::default()
        },
    ));
    Box::leak(Box::new(ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    }))
}

/// FIXED: the transport-budget arm.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Debug)]
struct CPoint {
    x: f64,
    y: f64,
    z: f64,
}

fn point_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    c_ts(
        unique,
        std::mem::size_of::<CPoint>(),
        vec![
            c_member("x", ROS_TYPE_DOUBLE, 0),
            c_member("y", ROS_TYPE_DOUBLE, 8),
            c_member("z", ROS_TYPE_DOUBLE, 16),
        ],
        None,
        None,
    )
}

/// FORGED: `{ x: u32, data: uint8[] }` — the shadow-pool arm.
#[repr(C)]
struct CU8Seq {
    data: *mut u8,
    size: usize,
    capacity: usize,
    #[cfg(cerulion_has_is_rosidl_buffer)]
    is_rosidl_buffer: bool,
    #[cfg(cerulion_has_is_rosidl_buffer)]
    owns_rosidl_buffer: bool,
}

#[repr(C)]
struct CBlob {
    x: u32,
    data: CU8Seq,
}

unsafe extern "C" fn blob_init(
    _msg: *mut c_void,
    _init: ffi::rosidl_runtime_c__message_initialization,
) {
}

unsafe extern "C" fn blob_fini(msg: *mut c_void) {
    let m = &mut *(msg as *mut CBlob);
    if !m.data.data.is_null() {
        free(m.data.data as *mut c_void);
        m.data.data = std::ptr::null_mut();
    }
}

fn blob_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    let mut data = c_member(
        "data",
        ROS_TYPE_UINT8,
        std::mem::offset_of!(CBlob, data) as u32,
    );
    data.is_array_ = true;
    c_ts(
        unique,
        std::mem::size_of::<CBlob>(),
        vec![c_member("x", ROS_TYPE_UINT32, 0), data],
        Some(blob_init),
        Some(blob_fini),
    )
}

unsafe fn publish_blob(publisher: *const ffi::rmw_publisher_t, x: u32, n: usize) {
    let buf = calloc(n.max(1), 1) as *mut u8;
    for i in 0..n {
        *buf.add(i) = (i as u32 ^ x) as u8;
    }
    let msg = CBlob {
        x,
        data: CU8Seq {
            data: buf,
            size: n,
            capacity: n,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
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
    free(buf as *mut c_void);
}

unsafe fn setup_node(name: &str) -> *mut ffi::rmw_node_t {
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    Box::leak(options);
    let node = rmw_create_node(context, cstr(name), cstr("/"));
    assert!(!node.is_null());
    node
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

unsafe fn setup_pair(
    ts: *const ffi::rosidl_message_type_support_t,
    tag: &str,
    suffix: u64,
) -> (
    *mut ffi::rmw_node_t,
    *mut ffi::rmw_publisher_t,
    *mut ffi::rmw_subscription_t,
    String,
) {
    let node = setup_node(&format!("{tag}_node_{suffix}"));
    let ros_topic = format!("/rmw_refusal/{tag}/{suffix}");
    let topic = CString::new(ros_topic.clone()).expect("topic");
    let qos = default_qos();
    let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
    assert!(!subscription.is_null());
    let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
    assert!(!publisher.is_null());
    (
        node,
        publisher,
        subscription,
        // The Cerulion name IS the ROS name, verbatim (leading slash kept).
        ros_topic,
    )
}

unsafe fn take_loaned(
    subscription: *const ffi::rmw_subscription_t,
) -> (ffi::rmw_ret_t, bool, *mut c_void) {
    let mut taken = false;
    let mut out: *mut c_void = std::ptr::null_mut();
    let ret = rmw_take_loaned_message(subscription, &mut out, &mut taken, std::ptr::null_mut());
    (ret, taken, out)
}

unsafe fn refusal_count(subscription: *const ffi::rmw_subscription_t) -> u64 {
    (*((*subscription).data as *const SubscriptionData)).loan_refusal_count()
}

// =====================================================================
// Log helpers (the repo's level-token discipline — see
// rmw_publish_reject_test.rs for why a text-only filter is not enough)
// =====================================================================

/// Markers chosen so no one is a substring of another: every loud head opens
/// with the refusal + an em dash (each `kind=` then carries its OWN headline
/// and remedy, asserted per arm), the repeat names itself, the decade line
/// says STILL.
const LOUD: &str = "loaned take refused —";
const SUPPRESSED: &str = "(suppressed repeat)";
const STILL: &str = "loaned take STILL refused";
const RECOVERY: &str = "loaned take recovered";

fn find_at<'a>(lines: &[&'a str], level: &str, marker: &str) -> Option<&'a str> {
    lines
        .iter()
        .copied()
        .find(|l| line_level(l) == Some(level) && l.contains(marker))
}

/// EVERY captured line at `level` carrying `marker` — for arms that must hold
/// a field contract across a re-armed regime's SECOND head as well as its
/// first (a `find_at` over a two-head capture inspects only the first).
fn lines_at<'a>(lines: &[&'a str], level: &str, marker: &str) -> Vec<&'a str> {
    lines
        .iter()
        .copied()
        .filter(|l| line_level(l) == Some(level) && l.contains(marker))
        .collect()
}

/// Whole-whitespace-token `key=value` match — never a substring.
fn has_field(line: &str, key: &str, value: &str) -> bool {
    let needle = format!("{key}={value}");
    line.split_whitespace().any(|token| token == needle)
}

// =====================================================================
// Tests
// =====================================================================

/// The shadow-pool arm, driven PAST the decade: a forged type with all four
/// shadows loaned out is refused 10× ⇒ 1 loud head + 8 suppressed repeats
/// + the 10th refusal re-announced loudly (`total_failures=10`,
/// `suppressed_count=8`); nothing was consumed (all 10 queued frames are
/// still there); one return + served take ⇒ ONE recovery carrying
/// `suppressed_count=8`; the counter is never reset; a fresh refusal is loud
/// again.
#[test]
#[serial]
#[traced_test]
fn shadow_pool_refusals_are_loud_once_re_announced_at_the_decade_and_recover() {
    const REFUSALS: usize = 10;
    unsafe {
        let suffix = unique_suffix();
        let ts = blob_ts(&format!("BlobR{suffix}"));
        let (node, publisher, subscription, topic) = setup_pair(ts, "blob", suffix);
        assert!(
            (*subscription).can_loan_messages,
            "the blob type is take-loanable"
        );
        assert_eq!(refusal_count(subscription), 0);

        for i in 0..8u32 {
            publish_blob(publisher, i, 64);
        }
        let mut held = Vec::new();
        for i in 0..4u32 {
            let (ret, taken, p) = take_loaned(subscription);
            assert_eq!(ret, RMW_RET_OK, "take {i} inside the budget");
            assert!(taken);
            assert_eq!((*(p as *const CBlob)).x, i, "FIFO");
            held.push(p);
        }
        for _ in 0..REFUSALS {
            let (ret, taken, p) = take_loaned(subscription);
            assert_eq!(ret, RMW_RET_ERROR);
            assert!(!taken);
            assert!(p.is_null());
        }
        assert_eq!(
            refusal_count(subscription),
            REFUSALS as u64,
            "the counter is UNCONDITIONAL — it counts the debug-suppressed repeats too"
        );

        // Recovery: return one, the next take serves frame 4 (nothing was
        // consumed by the refusals).
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, held.remove(0)),
            RMW_RET_OK
        );
        let (ret, taken, p) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_eq!(
            (*(p as *const CBlob)).x,
            4,
            "the refused takes consumed no frame"
        );
        held.push(p);
        assert_eq!(
            refusal_count(subscription),
            REFUSALS as u64,
            "recovery never resets"
        );

        // Re-armed: with four held again, a fresh refusal is loud again.
        let (ret, taken, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_ERROR);
        assert!(!taken);
        assert_eq!(refusal_count(subscription), REFUSALS as u64 + 1);

        logs_assert(|lines: &[&str]| {
            let heads = count_at_exclusively(lines, "ERROR", &[LOUD])?;
            never_loud(lines, SUPPRESSED)?;
            let repeats = count_at_exclusively(lines, "DEBUG", &[SUPPRESSED])?;
            let stills = count_at_exclusively(lines, "ERROR", &[STILL])?;
            let recoveries = count_at_exclusively(lines, "INFO", &[RECOVERY])?;
            if heads != 2 {
                return Err(format!(
                    "expected exactly 2 ERROR loud heads (the regime, then the re-armed one) \
                     — a per-take error! would give {}; got {heads}",
                    REFUSALS + 1
                ));
            }
            let want_repeats = debug_lines_expected(REFUSALS - 2);
            if repeats != want_repeats {
                return Err(format!(
                    "expected {want_repeats} DEBUG suppressed repeats (10 refusals − head − decade), got {repeats}"
                ));
            }
            if stills != 1 {
                return Err(format!(
                    "expected exactly 1 ERROR decade re-announcement, got {stills}"
                ));
            }
            if recoveries != 1 {
                return Err(format!(
                    "expected exactly 1 INFO recovery, got {recoveries}"
                ));
            }
            if count_at(lines, "ERROR", SUPPRESSED) != 0 {
                return Err("the suppressed arm must be DEBUG, found it at ERROR".into());
            }
            let heads = lines_at(lines, "ERROR", LOUD);
            if heads.is_empty() {
                return Err("no loud head".into());
            }
            for head in heads {
                if !has_field(head, "topic", &topic)
                    || !has_field(head, "kind", "shadow_pool_exhausted")
                {
                    return Err(format!(
                        "head must carry topic= and kind=shadow_pool_exhausted: {head}"
                    ));
                }
                if !head.contains("every shadow in the pool is loaned out")
                    || !head.contains("return outstanding loans")
                {
                    return Err(format!(
                        "the pool-exhausted head must name ITS condition and ITS remedy: {head}"
                    ));
                }
                if !has_field(head, "outstanding_loans", "4") || !has_field(head, "budget", "4") {
                    return Err(format!("head must name the shape (4 held of 4): {head}"));
                }
            }
            let still = find_at(lines, "ERROR", STILL).ok_or("no decade line")?;
            if !has_field(still, "total_failures", "10")
                || !has_field(still, "suppressed_count", "8")
            {
                return Err(format!(
                    "decade line must carry total_failures=10 suppressed_count=8: {still}"
                ));
            }
            if !has_field(still, "topic", &topic) {
                return Err(format!("decade line must carry topic=: {still}"));
            }
            let rec = find_at(lines, "INFO", RECOVERY).ok_or("no INFO recovery")?;
            if !has_field(rec, "suppressed_count", "8") || !has_field(rec, "topic", &topic) {
                return Err(format!(
                    "recovery must report the 8 SUPPRESSED (not the 10 total) with topic=: {rec}"
                ));
            }
            Ok(())
        });

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

/// The transport-budget arm (a FIXED type at iceoryx2's borrow budget —
/// `ExceedsMaxBorrows`), the arm that shipped as a bare per-take `error!`:
/// 6 refusals ⇒ 1 loud head (`kind=receive_failed`, the transport's reason
/// on `error=`) + 5 suppressed + counter 6; return + served take ⇒ one
/// recovery with `suppressed_count=5`; the queued 5th frame survives.
#[test]
#[serial]
#[traced_test]
fn transport_budget_refusals_are_loud_once_counted_always_and_recover() {
    const REFUSALS: usize = 6;
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("PtR{suffix}"));
        let (node, publisher, subscription, topic) = setup_pair(ts, "pt", suffix);
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
        let mut held = Vec::new();
        for _ in 0..4 {
            let (ret, taken, p) = take_loaned(subscription);
            assert_eq!(ret, RMW_RET_OK);
            assert!(taken);
            held.push(p);
        }
        for _ in 0..REFUSALS {
            let (ret, taken, p) = take_loaned(subscription);
            assert_eq!(
                ret, RMW_RET_ERROR,
                "the 5th concurrent loan exceeds the borrow budget"
            );
            assert!(!taken);
            assert!(p.is_null());
        }
        assert_eq!(refusal_count(subscription), REFUSALS as u64);

        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, held.remove(0)),
            RMW_RET_OK
        );
        let (ret, taken, p) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK, "returning one loan recovers");
        assert!(taken);
        assert_eq!(
            (*(p as *const CPoint)).x,
            4.0,
            "the frame behind the refusals survived"
        );
        held.push(p);
        assert_eq!(refusal_count(subscription), REFUSALS as u64);

        logs_assert(|lines: &[&str]| {
            let heads = count_at_exclusively(lines, "ERROR", &[LOUD])?;
            never_loud(lines, SUPPRESSED)?;
            let repeats = count_at_exclusively(lines, "DEBUG", &[SUPPRESSED])?;
            let recoveries = count_at_exclusively(lines, "INFO", &[RECOVERY])?;
            if heads != 1 {
                return Err(format!("expected exactly 1 ERROR loud head, got {heads}"));
            }
            let want_repeats = debug_lines_expected(REFUSALS - 1);
            if repeats != want_repeats {
                return Err(format!(
                    "expected {want_repeats} DEBUG suppressed repeats, got {repeats}"
                ));
            }
            if recoveries != 1 {
                return Err(format!(
                    "expected exactly 1 INFO recovery, got {recoveries}"
                ));
            }
            if count_at(lines, "ERROR", SUPPRESSED) != 0 {
                return Err("the suppressed arm must be DEBUG, found it at ERROR".into());
            }
            let heads = lines_at(lines, "ERROR", LOUD);
            if heads.is_empty() {
                return Err("no loud head".into());
            }
            for head in heads {
                if !has_field(head, "kind", "receive_failed") || !has_field(head, "topic", &topic) {
                    return Err(format!(
                        "head must carry kind=receive_failed and topic=: {head}"
                    ));
                }
                if !head.contains("ExceedsMaxBorrows") {
                    return Err(format!(
                        "head must carry the transport's own reason (ExceedsMaxBorrows): {head}"
                    ));
                }
                if !head.contains("the transport refused the receive")
                    || head.contains("every shadow in the pool")
                {
                    return Err(format!(
                        "the receive-failed head must carry ITS headline, not the pool's: {head}"
                    ));
                }
            }
            let rec = find_at(lines, "INFO", RECOVERY).ok_or("no INFO recovery")?;
            if !has_field(rec, "suppressed_count", &(REFUSALS - 1).to_string()) {
                return Err(format!("recovery must report the suppressed count: {rec}"));
            }
            Ok(())
        });

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

/// Anti-tautology control: a consumer that never retains past the budget
/// logs NONE of the refusal vocabulary and counts zero — the "exactly N"
/// arms above would also pass a reporter that fired on served takes.
#[test]
#[serial]
#[traced_test]
fn a_consumer_inside_the_budget_logs_nothing_and_counts_zero() {
    unsafe {
        let suffix = unique_suffix();
        let ts = blob_ts(&format!("BlobQ{suffix}"));
        let (node, publisher, subscription, _) = setup_pair(ts, "blobq", suffix);
        for round in 0..3u32 {
            for i in 0..4u32 {
                publish_blob(publisher, round * 4 + i, 32);
            }
            let mut held = Vec::new();
            for _ in 0..4 {
                let (ret, taken, p) = take_loaned(subscription);
                assert_eq!(ret, RMW_RET_OK);
                assert!(taken);
                held.push(p);
            }
            for p in held {
                assert_eq!(
                    rmw_return_loaned_message_from_subscription(subscription, p),
                    RMW_RET_OK
                );
            }
        }
        assert_eq!(refusal_count(subscription), 0);
        logs_assert(|lines: &[&str]| {
            // The offenders themselves, so a failure names WHICH marker leaked
            // and the line — a bare count would leave the operator no lead.
            let noisy: Vec<&&str> = lines
                .iter()
                .filter(|l| {
                    [LOUD, SUPPRESSED, STILL, RECOVERY]
                        .iter()
                        .any(|m| l.contains(m))
                })
                .collect();
            if !noisy.is_empty() {
                return Err(format!(
                    "healthy consumer logged {} loan-refusal line(s): {noisy:?}",
                    noisy.len()
                ));
            }
            Ok(())
        });
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// The forge-FALLBACK latch (a below-floor entry served by copy)
// =====================================================================

unsafe extern "C" fn fixture_allocate(size: usize, _state: *mut c_void) -> *mut c_void {
    calloc(size.max(1), 1)
}
unsafe extern "C" fn fixture_deallocate(ptr: *mut c_void, _state: *mut c_void) {
    free(ptr)
}
unsafe extern "C" fn fixture_reallocate(
    ptr: *mut c_void,
    size: usize,
    _state: *mut c_void,
) -> *mut c_void {
    extern "C" {
        fn realloc(p: *mut c_void, n: usize) -> *mut c_void;
    }
    realloc(ptr, size)
}
unsafe extern "C" fn fixture_zero_allocate(
    n: usize,
    size: usize,
    _state: *mut c_void,
) -> *mut c_void {
    calloc(n.max(1), size.max(1))
}

fn malloc_allocator() -> ffi::rcutils_allocator_t {
    ffi::rcutils_allocator_t {
        allocate: Some(fixture_allocate),
        deallocate: Some(fixture_deallocate),
        reallocate: Some(fixture_reallocate),
        zero_allocate: Some(fixture_zero_allocate),
        state: std::ptr::null_mut(),
    }
}

/// Publish one blob through `rmw_publish_serialized_message`, with its `data`
/// entry aimed at payload offset 0 (the fixed section's `x`) when
/// `below_floor` — the placement the wire forbids and the forge must not
/// alias.
unsafe fn publish_serialized_blob(
    publisher: *const ffi::rmw_publisher_t,
    ts: *const ffi::rosidl_message_type_support_t,
    x: u32,
    below_floor: bool,
) {
    let buf = calloc(4, 1) as *mut u8;
    for i in 0..4 {
        *buf.add(i) = (i as u32 ^ x) as u8;
    }
    let msg = CBlob {
        x,
        data: CU8Seq {
            data: buf,
            size: 4,
            capacity: 4,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
    };
    let mut ser: ffi::rmw_serialized_message_t = std::mem::zeroed();
    ser.allocator = malloc_allocator();
    assert_eq!(
        rmw_serialize(&msg as *const _ as *const c_void, ts, &mut ser),
        RMW_RET_OK
    );
    if below_floor {
        let bytes = std::slice::from_raw_parts_mut(ser.buffer, ser.buffer_length);
        let entry = cerulion_core::wire::WireHeader::SIZE + 4; // fixed = x (4 bytes); data = entry 0
        bytes[entry..entry + 4].copy_from_slice(&0u32.to_le_bytes());
        bytes[entry + 4..entry + 8].copy_from_slice(&4u32.to_le_bytes());
    }
    assert_eq!(
        rmw_publish_serialized_message(publisher, &ser, std::ptr::null_mut()),
        RMW_RET_OK
    );
    free(buf as *mut c_void);
}

const FALLBACK_LOUD: &str = "forged loaned take fell back to COPYING";
const FALLBACK_SUPPRESSED: &str = "fell back to copying (suppressed repeat)";
const FALLBACK_RECOVERY: &str = "forged loaned take recovered";

/// Six below-floor frames ⇒ every take SUCCEEDS with `data` a heap COPY of
/// the bytes the entry designates (`x`'s own four bytes), ONE `WARN` head +
/// 5 `DEBUG` repeats, an unconditional count of 6; a well-placed frame ⇒ one
/// `INFO` recovery carrying `suppressed_count=5`; a fresh below-floor frame
/// is loud again. The refusal latch is untouched throughout (counts 0).
#[test]
#[serial]
#[traced_test]
fn below_floor_fallbacks_are_loud_once_counted_always_and_recover() {
    const FALLBACKS: u32 = 6;
    unsafe {
        let suffix = unique_suffix();
        let ts = blob_ts(&format!("BlobF{suffix}"));
        let (node, publisher, subscription, topic) = setup_pair(ts, "blobf", suffix);
        let fallbacks = |sub: *const ffi::rmw_subscription_t| {
            (*((*sub).data as *const SubscriptionData)).forge_fallback_count()
        };
        assert_eq!(fallbacks(subscription), 0);

        for x in 0..FALLBACKS {
            publish_serialized_blob(publisher, ts, 100 + x, true);
            let (ret, taken, p) = take_loaned(subscription);
            assert_eq!(ret, RMW_RET_OK);
            assert!(taken, "a below-floor entry is served, never refused");
            let b = &*(p as *const CBlob);
            assert_eq!(b.x, 100 + x);
            assert_eq!(
                b.data.size, 4,
                "the copy is the four bytes the entry designates"
            );
            assert_eq!(
                std::slice::from_raw_parts(b.data.data, 4),
                &(100 + x).to_le_bytes(),
                "…which are x's own bytes in the fixed section"
            );
            assert_eq!(
                rmw_return_loaned_message_from_subscription(subscription, p),
                RMW_RET_OK
            );
        }
        assert_eq!(
            fallbacks(subscription),
            u64::from(FALLBACKS),
            "the counter is UNCONDITIONAL — it counts the debug-suppressed repeats too"
        );
        assert_eq!(
            refusal_count(subscription),
            0,
            "a fallback is not a refusal"
        );

        // Recovery: a well-placed frame is forged and closes the regime.
        publish_serialized_blob(publisher, ts, 7, false);
        let (ret, taken, p) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p),
            RMW_RET_OK
        );
        assert_eq!(
            fallbacks(subscription),
            u64::from(FALLBACKS),
            "recovery never resets"
        );

        // Re-armed.
        publish_serialized_blob(publisher, ts, 8, true);
        let (ret, taken, p) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p),
            RMW_RET_OK
        );
        assert_eq!(fallbacks(subscription), u64::from(FALLBACKS) + 1);

        logs_assert(|lines: &[&str]| {
            let heads = count_at_exclusively(lines, "WARN", &[FALLBACK_LOUD])?;
            never_loud(lines, FALLBACK_SUPPRESSED)?;
            let repeats = count_at_exclusively(lines, "DEBUG", &[FALLBACK_SUPPRESSED])?;
            let recoveries = count_at_exclusively(lines, "INFO", &[FALLBACK_RECOVERY])?;
            if heads != 2 {
                return Err(format!(
                    "expected exactly 2 WARN heads (the regime, then the re-armed one), got {heads}"
                ));
            }
            let want_repeats = debug_lines_expected((FALLBACKS - 1) as usize);
            if repeats != want_repeats {
                return Err(format!(
                    "expected {want_repeats} DEBUG suppressed repeats, got {repeats}"
                ));
            }
            if recoveries != 1 {
                return Err(format!(
                    "expected exactly 1 INFO recovery, got {recoveries}"
                ));
            }
            if count_at(lines, "WARN", FALLBACK_SUPPRESSED) != 0 {
                return Err("the suppressed arm must be DEBUG, found it at WARN".into());
            }
            let heads = lines_at(lines, "WARN", FALLBACK_LOUD);
            if heads.is_empty() {
                return Err("no WARN head".into());
            }
            for head in heads {
                if !has_field(head, "topic", &topic)
                    || !has_field(head, "below_floor", "1")
                    || !has_field(head, "forgeable", "1")
                    || !has_field(head, "data_floor", "12")
                {
                    return Err(format!(
                        "head must carry topic=, below_floor=1, forgeable=1, data_floor=12: {head}"
                    ));
                }
            }
            let rec = find_at(lines, "INFO", FALLBACK_RECOVERY).ok_or("no INFO recovery")?;
            if !has_field(rec, "suppressed_count", &(FALLBACKS - 1).to_string()) {
                return Err(format!("recovery must report the suppressed count: {rec}"));
            }
            if lines.iter().any(|l| l.contains(LOUD) || l.contains(STILL)) {
                return Err("a fallback must never log through the REFUSAL latch".into());
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
