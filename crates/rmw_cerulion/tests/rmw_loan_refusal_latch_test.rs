// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The loaned take PAST the zero-copy loans its borrow ceiling allows, and the
//! take-side flood latches, at the production `rmw_take_loaned_message` site.
//!
//! A consumer that RETAINS loans (a `message_filters` cache, a latest-N
//! buffer) must keep receiving. The take keeps one borrow of the
//! subscription's ceiling (`min(RMW_TAKE_LOAN_BORROW_BUDGET, the service's
//! max_borrowed_samples)`, 4 on a service this crate creates) in reserve: once
//! `ceiling - 1` zero-copy loans are held, each further take receives into the
//! reserved borrow, copies the message into private memory, releases the
//! borrow at once and hands out the copy. Both shapes are pinned: a FIXED type
//! (the fixed section copied into aligned heap words) and a FORGED type (an
//! unpooled shadow filled by the copying decode).
//!
//! The copies ride their own `FailureRegimeLatch` through
//! `loan_refusal_latch`. Hand oracles, every predicate matching the LEVEL
//! TOKEN as well as the message (a text-only filter passes a variant that
//! emits the suppressed arm loudly, so suppression does nothing while message
//! and counter stay intact): N copies give exactly 1 `WARN` head + N-1
//! `DEBUG` repeats (+ the `WARN` decade re-announcement when N crosses 10,
//! which is NOT counted as suppressed) + an UNCONDITIONAL counter of N; a
//! return and a take served zero-copy again give exactly ONE `INFO` recovery
//! carrying `suppressed_count=` the suppressed number (not the total); the
//! counter is never reset; a fresh copy is loud again. The refusal latch
//! counts zero throughout. Field keys are pinned as whole tokens: `topic=`,
//! `held_loans=`, `ceiling=`, `total_failures=`, `suppressed_count=`.
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

use rmw_cerulion::ffi::{self, RMW_RET_OK};
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

/// Every `fini` the rmw runs over a blob: a copy's return runs it once (the
/// copy is destroyed), a zero-copy loan's return runs none (its shadow is
/// recycled).
static BLOB_FINIS: AtomicU64 = AtomicU64::new(0);

unsafe extern "C" fn blob_fini(msg: *mut c_void) {
    BLOB_FINIS.fetch_add(1, Ordering::Relaxed);
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

unsafe fn copy_count(subscription: *const ffi::rmw_subscription_t) -> u64 {
    (*((*subscription).data as *const SubscriptionData)).loan_copy_count()
}

/// The zero-copy loans the subscription holds (each pins one SHM borrow).
unsafe fn zero_copy_held(subscription: *const ffi::rmw_subscription_t) -> usize {
    let data = &*((*subscription).data as *const SubscriptionData);
    data.inner
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .pending_takes
        .len()
}

/// True when `loaned` is a zero-copy loan: one of the held SHM samples is
/// keyed by it. A copy is keyed by private heap memory and holds no sample.
unsafe fn is_zero_copy(subscription: *const ffi::rmw_subscription_t, loaned: *mut c_void) -> bool {
    let data = &*((*subscription).data as *const SubscriptionData);
    data.inner
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .pending_takes
        .iter()
        .any(|t| t.key == loaned as usize)
}

/// True when `ptr` lies inside the payload of a held SHM sample.
unsafe fn inside_a_held_sample(
    subscription: *const ffi::rmw_subscription_t,
    ptr: *const u8,
) -> bool {
    let data = &*((*subscription).data as *const SubscriptionData);
    let inner = data.inner.lock().unwrap_or_else(|e| e.into_inner());
    inner.pending_takes.iter().any(|t| {
        let payload = t.sample.payload();
        let start = payload.as_ptr() as usize;
        (start..start + payload.len()).contains(&(ptr as usize))
    })
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

/// Markers of the served-by-copy latch, none a substring of another: the head
/// names the copy and a colon, the repeat names itself, the decade line says
/// STILL, the recovery says zero-copy again.
const COPY_LOUD: &str = "loaned take served by COPY:";
const COPY_SUPPRESSED: &str = "loaned take served by COPY (suppressed repeat)";
const COPY_STILL: &str = "loaned takes STILL served by COPY";
const COPY_RECOVERY: &str = "loaned take served zero-copy again";

/// The bytes `publish_blob(_, x, 4)` writes for `x` in `0..=14`, written out by
/// hand (`data[i] = i ^ x`).
const BLOB_ORACLE: [[u8; 4]; 15] = [
    [0, 1, 2, 3],
    [1, 0, 3, 2],
    [2, 3, 0, 1],
    [3, 2, 1, 0],
    [4, 5, 6, 7],
    [5, 4, 7, 6],
    [6, 7, 4, 5],
    [7, 6, 5, 4],
    [8, 9, 10, 11],
    [9, 8, 11, 10],
    [10, 11, 8, 9],
    [11, 10, 9, 8],
    [12, 13, 14, 15],
    [13, 12, 15, 14],
    [14, 15, 12, 13],
];

/// Publish blob `x` and take it as a loan: served, taken, `x` and its four
/// bytes exactly the oracle's. Returns the loaned pointer.
unsafe fn publish_and_take_blob(
    publisher: *const ffi::rmw_publisher_t,
    subscription: *const ffi::rmw_subscription_t,
    x: u32,
) -> *mut c_void {
    publish_blob(publisher, x, 4);
    let (ret, taken, p) = take_loaned(subscription);
    assert_eq!(
        ret, RMW_RET_OK,
        "take of blob {x} is served whatever the consumer holds"
    );
    assert!(taken, "blob {x} is taken");
    assert!(!p.is_null(), "blob {x} hands out a pointer");
    let b = &*(p as *const CBlob);
    assert_eq!(b.x, x, "FIFO");
    assert_eq!(b.data.size, 4, "blob {x}");
    assert_eq!(
        std::slice::from_raw_parts(b.data.data, 4),
        &BLOB_ORACLE[x as usize],
        "blob {x}'s bytes"
    );
    p
}

/// A FORGED type whose consumer keeps every message keeps receiving. The
/// ceiling is 4, so the first three takes are zero-copy loans (`data()` inside
/// a held sample) and every take after them, the 4th and the 5th included, is
/// served by COPY: RMW_RET_OK, taken, the published contents, `data()` in
/// private memory, no borrow held for it. Ten copies give 1 WARN head + 8 DEBUG
/// repeats + the 10th re-announced at WARN (`total_failures=10`,
/// `suppressed_count=8`) and a counter of 10. Returning a copy runs the
/// typesupport's `fini` once and untracks it; after one zero-copy loan comes
/// back the next take is zero-copy again, with ONE INFO recovery carrying
/// `suppressed_count=8`; the counter is never reset and a fresh copy is loud
/// again. Nothing is refused.
#[test]
#[serial]
#[traced_test]
fn forged_takes_past_the_reserve_are_served_by_copy_loud_once_re_announced_and_recover() {
    const ZERO_COPY: u32 = 3;
    const COPIES: u32 = 10;
    unsafe {
        let suffix = unique_suffix();
        let ts = blob_ts(&format!("BlobR{suffix}"));
        let (node, publisher, subscription, topic) = setup_pair(ts, "blob", suffix);
        assert!(
            (*subscription).can_loan_messages,
            "the blob type is take-loanable"
        );
        assert_eq!(copy_count(subscription), 0);

        let mut held = Vec::new();
        for x in 0..ZERO_COPY + COPIES {
            let p = publish_and_take_blob(publisher, subscription, x);
            let data_ptr = (*(p as *const CBlob)).data.data as *const u8;
            if x < ZERO_COPY {
                assert!(
                    is_zero_copy(subscription, p),
                    "blob {x} is a zero-copy loan"
                );
                assert!(
                    inside_a_held_sample(subscription, data_ptr),
                    "a zero-copy loan's data() aims into its held sample (blob {x})"
                );
            } else {
                assert!(
                    !is_zero_copy(subscription, p),
                    "blob {x} is served by copy: the consumer already holds {ZERO_COPY} \
                     zero-copy loans"
                );
                assert!(
                    !inside_a_held_sample(subscription, data_ptr),
                    "a copy's data() is private memory, never a held sample (blob {x})"
                );
            }
            held.push(p);
        }
        assert_eq!(
            held.len(),
            13,
            "a fifth held loan, and every one after it, still receives"
        );
        assert_eq!(
            zero_copy_held(subscription),
            3,
            "zero-copy loans stop one below the ceiling of 4"
        );
        assert_eq!(
            copy_count(subscription),
            u64::from(COPIES),
            "the counter is UNCONDITIONAL: it counts the debug-suppressed repeats too"
        );
        assert_eq!(refusal_count(subscription), 0, "nothing was refused");

        // A returned copy is destroyed through the typesupport's fini and is
        // no longer outstanding.
        let last_copy = held.pop().expect("held");
        let finis = BLOB_FINIS.load(Ordering::Relaxed);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, last_copy),
            RMW_RET_OK
        );
        assert_eq!(
            BLOB_FINIS.load(Ordering::Relaxed) - finis,
            1,
            "returning a copy runs the typesupport's fini exactly once"
        );
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, last_copy),
            ffi::RMW_RET_INVALID_ARGUMENT,
            "a returned copy is no longer an outstanding loan"
        );

        // Recovery: one zero-copy loan back, and the next take is zero-copy.
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, held.remove(0)),
            RMW_RET_OK
        );
        let p = publish_and_take_blob(publisher, subscription, 13);
        assert!(
            is_zero_copy(subscription, p),
            "below the reserve again, the take is zero-copy"
        );
        held.push(p);
        assert_eq!(
            copy_count(subscription),
            u64::from(COPIES),
            "recovery never resets"
        );

        // Re-armed: three zero-copy loans held again, a fresh copy is loud.
        let p = publish_and_take_blob(publisher, subscription, 14);
        assert!(!is_zero_copy(subscription, p), "past the reserve again");
        held.push(p);
        assert_eq!(copy_count(subscription), u64::from(COPIES) + 1);
        assert_eq!(refusal_count(subscription), 0);

        logs_assert(|lines: &[&str]| {
            let heads = count_at_exclusively(lines, "WARN", &[COPY_LOUD])?;
            never_loud(lines, COPY_SUPPRESSED)?;
            let repeats = count_at_exclusively(lines, "DEBUG", &[COPY_SUPPRESSED])?;
            let stills = count_at_exclusively(lines, "WARN", &[COPY_STILL])?;
            let recoveries = count_at_exclusively(lines, "INFO", &[COPY_RECOVERY])?;
            if heads != 2 {
                return Err(format!(
                    "expected exactly 2 WARN heads (the regime, then the re-armed one); a \
                     per-take warn! would give {}; got {heads}",
                    COPIES + 1
                ));
            }
            let want_repeats = debug_lines_expected((COPIES - 2) as usize);
            if repeats != want_repeats {
                return Err(format!(
                    "expected {want_repeats} DEBUG suppressed repeats (10 copies less the head \
                     and the decade line), got {repeats}"
                ));
            }
            if stills != 1 {
                return Err(format!(
                    "expected exactly 1 WARN decade re-announcement, got {stills}"
                ));
            }
            if recoveries != 1 {
                return Err(format!(
                    "expected exactly 1 INFO recovery, got {recoveries}"
                ));
            }
            for head in lines_at(lines, "WARN", COPY_LOUD) {
                if !has_field(head, "topic", &topic)
                    || !has_field(head, "held_loans", "3")
                    || !has_field(head, "ceiling", "4")
                {
                    return Err(format!(
                        "head must carry topic=, held_loans=3 and ceiling=4: {head}"
                    ));
                }
                if !head.contains("return loans") {
                    return Err(format!("head must name the remedy: {head}"));
                }
            }
            let still = find_at(lines, "WARN", COPY_STILL).ok_or("no decade line")?;
            if !has_field(still, "total_failures", "10")
                || !has_field(still, "suppressed_count", "8")
                || !has_field(still, "topic", &topic)
            {
                return Err(format!(
                    "decade line must carry topic=, total_failures=10 and suppressed_count=8: \
                     {still}"
                ));
            }
            let rec = find_at(lines, "INFO", COPY_RECOVERY).ok_or("no INFO recovery")?;
            if !has_field(rec, "suppressed_count", "8") || !has_field(rec, "topic", &topic) {
                return Err(format!(
                    "recovery must report the 8 SUPPRESSED (not the 10 total) with topic=: {rec}"
                ));
            }
            if lines
                .iter()
                .any(|l| l.contains(LOUD) || l.contains(STILL) || l.contains(RECOVERY))
            {
                return Err(
                    "a take served by copy must never log through the REFUSAL latch".into(),
                );
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

/// The points published in order, written out by hand.
const POINT_ORACLE: [CPoint; 10] = [
    CPoint {
        x: 0.0,
        y: -1.5,
        z: 100.0,
    },
    CPoint {
        x: 1.0,
        y: -2.5,
        z: 101.0,
    },
    CPoint {
        x: 2.0,
        y: -3.5,
        z: 102.0,
    },
    CPoint {
        x: 3.0,
        y: -4.5,
        z: 103.0,
    },
    CPoint {
        x: 4.0,
        y: -5.5,
        z: 104.0,
    },
    CPoint {
        x: 5.0,
        y: -6.5,
        z: 105.0,
    },
    CPoint {
        x: 6.0,
        y: -7.5,
        z: 106.0,
    },
    CPoint {
        x: 7.0,
        y: -8.5,
        z: 107.0,
    },
    CPoint {
        x: 8.0,
        y: -9.5,
        z: 108.0,
    },
    CPoint {
        x: 9.0,
        y: -10.5,
        z: 109.0,
    },
];

/// Publish point `i` of the oracle and take it as a loan: served, taken, the
/// struct exactly the oracle's, the pointer 8-aligned. Returns the pointer.
unsafe fn publish_and_take_point(
    publisher: *const ffi::rmw_publisher_t,
    subscription: *const ffi::rmw_subscription_t,
    i: usize,
) -> *mut c_void {
    let msg = POINT_ORACLE[i];
    assert_eq!(
        rmw_publish(
            publisher,
            &msg as *const _ as *const c_void,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
    let (ret, taken, p) = take_loaned(subscription);
    assert_eq!(
        ret, RMW_RET_OK,
        "take of point {i} is served whatever the consumer holds"
    );
    assert!(taken, "point {i} is taken");
    assert!(!p.is_null(), "point {i} hands out a pointer");
    assert_eq!(p as usize % 8, 0, "point {i}'s loaned pointer is 8-aligned");
    assert_eq!(*(p as *const CPoint), POINT_ORACLE[i], "point {i}");
    p
}

/// A FIXED type at its borrow ceiling keeps receiving: three zero-copy loans
/// held, the 4th, the 5th and every take after them are served by COPY (the
/// fixed section in private, 8-aligned memory, the borrow released at once),
/// never `ExceedsMaxBorrows`. Six copies give 1 WARN head + 5 DEBUG repeats and
/// a counter of 6; a returned copy is untracked; one zero-copy loan back lets
/// the next take be zero-copy again with ONE INFO recovery carrying
/// `suppressed_count=5`.
#[test]
#[serial]
#[traced_test]
fn fixed_takes_past_the_reserve_are_served_by_copy_counted_always_and_recover() {
    const ZERO_COPY: usize = 3;
    const COPIES: usize = 6;
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("PtR{suffix}"));
        let (node, publisher, subscription, topic) = setup_pair(ts, "pt", suffix);
        let mut held = Vec::new();
        for i in 0..ZERO_COPY + COPIES {
            let p = publish_and_take_point(publisher, subscription, i);
            assert_eq!(
                is_zero_copy(subscription, p),
                i < ZERO_COPY,
                "point {i}: the first {ZERO_COPY} takes are zero-copy, the rest copies"
            );
            if i >= ZERO_COPY {
                assert!(
                    !inside_a_held_sample(subscription, p as *const u8),
                    "a copy is private memory, never a held sample (point {i})"
                );
            }
            held.push(p);
        }
        assert_eq!(
            zero_copy_held(subscription),
            3,
            "the fourth borrow stays in reserve"
        );
        assert_eq!(copy_count(subscription), COPIES as u64);
        assert_eq!(refusal_count(subscription), 0, "nothing was refused");

        // A returned copy is untracked; its bytes were the oracle's to the end.
        let copy = held.pop().expect("held");
        assert_eq!(*(copy as *const CPoint), POINT_ORACLE[8]);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, copy),
            RMW_RET_OK
        );
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, copy),
            ffi::RMW_RET_INVALID_ARGUMENT,
            "a returned copy is no longer an outstanding loan"
        );

        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, held.remove(0)),
            RMW_RET_OK
        );
        let p = publish_and_take_point(publisher, subscription, 9);
        assert!(
            is_zero_copy(subscription, p),
            "below the reserve again, the take is zero-copy"
        );
        held.push(p);
        assert_eq!(
            copy_count(subscription),
            COPIES as u64,
            "recovery never resets"
        );

        logs_assert(|lines: &[&str]| {
            let heads = count_at_exclusively(lines, "WARN", &[COPY_LOUD])?;
            never_loud(lines, COPY_SUPPRESSED)?;
            let repeats = count_at_exclusively(lines, "DEBUG", &[COPY_SUPPRESSED])?;
            let recoveries = count_at_exclusively(lines, "INFO", &[COPY_RECOVERY])?;
            if heads != 1 {
                return Err(format!("expected exactly 1 WARN loud head, got {heads}"));
            }
            let want_repeats = debug_lines_expected(COPIES - 1);
            if repeats != want_repeats {
                return Err(format!(
                    "expected {want_repeats} DEBUG suppressed repeats, got {repeats}"
                ));
            }
            if count_at(lines, "WARN", COPY_STILL) != 0 {
                return Err("six copies never reach the decade line".into());
            }
            if recoveries != 1 {
                return Err(format!(
                    "expected exactly 1 INFO recovery, got {recoveries}"
                ));
            }
            let head = find_at(lines, "WARN", COPY_LOUD).ok_or("no loud head")?;
            if !has_field(head, "topic", &topic)
                || !has_field(head, "held_loans", "3")
                || !has_field(head, "ceiling", "4")
                || !has_field(head, "total_failures", "1")
            {
                return Err(format!(
                    "head must carry topic=, held_loans=3, ceiling=4 and total_failures=1: {head}"
                ));
            }
            let rec = find_at(lines, "INFO", COPY_RECOVERY).ok_or("no INFO recovery")?;
            if !has_field(rec, "suppressed_count", &(COPIES - 1).to_string())
                || !has_field(rec, "total_failures", &COPIES.to_string())
            {
                return Err(format!(
                    "recovery must report the suppressed count and the running total: {rec}"
                ));
            }
            if lines.iter().any(|l| l.contains("ExceedsMaxBorrows")) {
                return Err("no take may reach the transport's borrow refusal".into());
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

/// One served take of the determinism sequence: `x`, whether it was zero-copy,
/// and its four bytes.
type ServedBlob = (u32, bool, [u8; 4]);

/// Run the fixed sequence on a fresh topic: six takes held, a copy and a
/// zero-copy loan returned, two takes more. Returns what was served and the
/// final copy and refusal counts.
unsafe fn run_retention_sequence(tag: &str) -> (Vec<ServedBlob>, u64, u64) {
    let suffix = unique_suffix();
    let ts = blob_ts(&format!("BlobD{tag}{suffix}"));
    let (node, publisher, subscription, _) = setup_pair(ts, tag, suffix);
    let mut served = Vec::new();
    let mut held = Vec::new();
    let mut take = |x: u32, held: &mut Vec<*mut c_void>| {
        let p = publish_and_take_blob(publisher, subscription, x);
        let b = &*(p as *const CBlob);
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(std::slice::from_raw_parts(b.data.data, 4));
        served.push((b.x, is_zero_copy(subscription, p), bytes));
        held.push(p);
    };
    for x in 0..6 {
        take(x, &mut held);
    }
    // Return the last copy and the first zero-copy loan.
    for p in [held.remove(5), held.remove(0)] {
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p),
            RMW_RET_OK
        );
    }
    take(6, &mut held);
    take(7, &mut held);
    let counts = (copy_count(subscription), refusal_count(subscription));
    for p in held {
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p),
            RMW_RET_OK
        );
    }
    assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
    assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
    assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    (served, counts.0, counts.1)
}

/// Determinism: the same retention sequence run twice serves the same
/// messages the same way (which takes are zero-copy, which are copies, the
/// bytes) with the same counts, and both runs equal the hand-written oracle.
#[test]
#[serial]
#[traced_test]
fn the_same_retention_sequence_serves_the_same_takes_twice() {
    const ZC: bool = true;
    const COPY: bool = false;
    let oracle: Vec<ServedBlob> = vec![
        (0, ZC, [0, 1, 2, 3]),
        (1, ZC, [1, 0, 3, 2]),
        (2, ZC, [2, 3, 0, 1]),
        (3, COPY, [3, 2, 1, 0]),
        (4, COPY, [4, 5, 6, 7]),
        (5, COPY, [5, 4, 7, 6]),
        // One zero-copy loan returned: the next take is zero-copy again...
        (6, ZC, [6, 7, 4, 5]),
        // ...and the one after it is past the reserve once more.
        (7, COPY, [7, 6, 5, 4]),
    ];
    unsafe {
        let first = run_retention_sequence("detA");
        let second = run_retention_sequence("detB");
        assert_eq!(first, second, "two runs of one sequence must agree");
        assert_eq!(first.0, oracle, "served takes");
        assert_eq!(first.1, 4, "four takes were served by copy");
        assert_eq!(first.2, 0, "none was refused");
    }
}

/// Anti-tautology control: a consumer that holds no more than the three
/// zero-copy loans the ceiling of 4 allows logs NONE of the refusal or copy
/// vocabulary and counts zero on both latches; the "exactly N" arms above would
/// also pass a reporter that fired on every served take.
#[test]
#[serial]
#[traced_test]
fn a_consumer_inside_the_budget_logs_nothing_and_counts_zero() {
    unsafe {
        let suffix = unique_suffix();
        let ts = blob_ts(&format!("BlobQ{suffix}"));
        let (node, publisher, subscription, _) = setup_pair(ts, "blobq", suffix);
        for round in 0..3u32 {
            for i in 0..3u32 {
                publish_blob(publisher, round * 3 + i, 32);
            }
            let mut held = Vec::new();
            for _ in 0..3 {
                let (ret, taken, p) = take_loaned(subscription);
                assert_eq!(ret, RMW_RET_OK);
                assert!(taken);
                assert!(
                    is_zero_copy(subscription, p),
                    "inside the budget, zero-copy"
                );
                held.push(p);
            }
            for p in held {
                assert_eq!(
                    rmw_return_loaned_message_from_subscription(subscription, p),
                    RMW_RET_OK
                );
            }
        }
        assert_eq!(copy_count(subscription), 0);
        assert_eq!(refusal_count(subscription), 0);
        logs_assert(|lines: &[&str]| {
            // The offenders themselves, so a failure names WHICH marker leaked
            // and the line — a bare count would leave the operator no lead.
            let noisy: Vec<&&str> = lines
                .iter()
                .filter(|l| {
                    [
                        LOUD,
                        SUPPRESSED,
                        STILL,
                        RECOVERY,
                        COPY_LOUD,
                        COPY_STILL,
                        COPY_RECOVERY,
                    ]
                    .iter()
                    .any(|m| l.contains(m))
                })
                .collect();
            if !noisy.is_empty() {
                return Err(format!(
                    "healthy consumer logged {} loan-refusal or copy line(s): {noisy:?}",
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
