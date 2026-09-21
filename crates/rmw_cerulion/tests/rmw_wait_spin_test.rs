// SPDX-License-Identifier: AGPL-3.0-only
//! `rmw_wait` spin behavioral pins, spin ENABLED.
//!
//! Every test sets `CERULION_LIVE_SPIN_US=18446744073709551615` (`u64::MAX`
//! — a "584,000-year spin" input) via an `EnvVarGuard`
//! BEFORE its wait, so each wait runs on the CLAMPED budget (the 100 ms
//! `SPIN_BUDGET_MAX_US` cap) and proves that input is benign end to end:
//! zero-timeout waits still return at once, mid-wait wakes still land,
//! nothing pins a core past the cap. The knob is the SHARED
//! `CERULION_LIVE_SPIN_US` (one parse in `cerulion_core::monitor_wait`,
//! one ceiling — since the one-knob-surface merge; the rmw-only spin env
//! is gone) and is read PER CALL, so the guard's
//! `remove_var` on drop really does restore the default for later tests —
//! which is why the spin-DISABLED arms now live beside every other
//! spin=0 pin in `rmw_wait_event_test.rs` rather than in a second binary.
//!
//! Every wait also carries a SECONDS-scale HANG guard (`elapsed <` a
//! generous ceiling; a ceiling, never a tight band): a wake that arrived
//! near the 10 s timeout, or a zero-timeout call that slept seconds
//! before TIMEOUT, must not pass on counters + return code alone.
//!
//! The pins are otherwise COUNTER-based (Principle #3), not wall bands:
//! `WaitSetData::{spin_probes, spin_wakes}` behind `rmw_wait_set_t::data`.
//! A mid-wait publish/trigger synchronizes on the OBSERVABLE "the waiter
//! has run at least one spin probe" — never on a sleep, which cannot
//! distinguish "queued before the wait" from "arrived mid-wait" — and the
//! wake arms then require `spin_wakes >= 1` (a variant that deletes the
//! spin still delivers via the fd block but scores 0 here), while the
//! zero-timeout arm requires `spin_probes == 0` (the spin never runs
//! past the caller's deadline). Under the merged contract the spin runs
//! ONCE per call, on entry, for up to the 100 ms cap: the publisher
//! thread publishes microseconds after observing the first probe, well
//! inside that single phase. Residual: a publish delayed past the
//! phase (a >100 ms stall of the publisher thread) would be served by the
//! fd block instead and fail the `spin_wakes` arm — loudly, never
//! vacuously.
//!
//! ⚠️ iceoryx2 shared memory is a process singleton — run with
//! `--test-threads=1`:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_wait_spin_test -- --test-threads=1
//! ```

#![cfg(unix)]

use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;

const SPIN_US_ENV: &str = "CERULION_LIVE_SPIN_US";
/// `u64::MAX` — see the module docs: the process runs on the clamped cap.
const SPIN_US_VALUE: &str = "18446744073709551615";

/// RAII env cleanup even on panic (repo convention). The knob is read per
/// call, so dropping the guard restores the default for whatever runs next.
struct EnvVarGuard;
impl EnvVarGuard {
    fn set(value: &str) -> Self {
        std::env::set_var(SPIN_US_ENV, value);
        Self
    }
}
impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(SPIN_US_ENV);
    }
}

// =====================================================================
// Hand-built typesupport fixtures (same pattern as rmw_e2e_test.rs)
// =====================================================================

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

fn point_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    // Unique TYPE NAME per test run so schema hashes don't collide with
    // other runs against the global SHM singleton.
    make_message_ts(
        "rmw_spin__msg",
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

/// Read the wait set's spin diagnostics (Principle #3) by casting the
/// opaque data pointer — the same shape the event-driven wait's fd
/// counters use. Sound from any thread: the fields are atomics and
/// `rmw_wait` holds only a shared reference to the struct.
unsafe fn spin_counters(ws: *mut ffi::rmw_wait_set_t) -> (u64, u64) {
    let data = &*((*ws).data as *const WaitSetData);
    (
        data.spin_probes.load(Ordering::Relaxed),
        data.spin_wakes.load(Ordering::Relaxed),
    )
}

/// Block until the waiter is provably INSIDE its spin phase (at least
/// one spin probe has run on `ws`) — the observable a mid-wait publish
/// or trigger synchronizes on instead of a sleep. Bounded so a waiter
/// that never spins fails loudly instead of hanging the thread.
unsafe fn await_spin_entry(ws: *mut ffi::rmw_wait_set_t) {
    let deadline = Instant::now() + Duration::from_secs(10);
    while spin_counters(ws).0 == 0 {
        assert!(
            Instant::now() < deadline,
            "the waiter never entered its spin phase"
        );
        std::hint::spin_loop();
    }
}

// =====================================================================
// Pins
// =====================================================================

/// (a) A message published while the taker is provably INSIDE its spin
/// wakes the wait ready and is taken — the hand oracle payload, not a
/// self-compare — and the wake was served BY the spin
/// (`spin_wakes >= 1`), which a spin-less variant cannot score.
#[test]
#[serial]
fn publish_while_waiting_wakes_the_spin_and_takes_the_oracle() {
    let _env = EnvVarGuard::set(SPIN_US_VALUE);
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("SpinA{suffix}");
        let ts = point_ts(&type_name);
        let (context, node, _opts) = setup_node(&format!("spin_a_{suffix}"));

        let topic = CString::new(format!("/rmw_spin/a/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        // Hand oracle.
        let oracle = CPoint {
            x: 15.36,
            y: -1.5,
            z: 36.0,
        };

        let ws = rmw_create_wait_set(context, 8);

        // Publish from another thread once the waiter is inside its spin.
        let pub_addr = publisher as usize;
        let ws_addr = ws as usize;
        let publisher_thread = std::thread::spawn(move || {
            await_spin_entry(ws_addr as *mut ffi::rmw_wait_set_t);
            let publisher = pub_addr as *mut ffi::rmw_publisher_t;
            let msg = CPoint {
                x: 15.36,
                y: -1.5,
                z: 36.0,
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
        });

        let mut sub_ptrs = [(*subscription).data];
        let mut subs = ffi::rmw_subscriptions_t {
            subscriber_count: 1,
            subscribers: sub_ptrs.as_mut_ptr(),
        };
        let timeout = ffi::rmw_time_t { sec: 10, nsec: 0 };
        let start = Instant::now();
        let ret = rmw_wait(
            &mut subs,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &timeout,
        );
        let elapsed = start.elapsed();
        publisher_thread.join().expect("publisher thread");

        assert_eq!(ret, RMW_RET_OK, "wait must wake on the mid-wait publish");
        // Hang guard (seconds-scale ceiling, never a band).
        assert!(
            elapsed < Duration::from_secs(5),
            "the wake must be prompt, not near the timeout (elapsed {elapsed:?})"
        );
        assert!(!sub_ptrs[0].is_null(), "subscription must be ready");
        let (probes, wakes) = spin_counters(ws);
        assert!(probes >= 1, "the waiter spun (probes {probes})");
        assert!(
            wakes >= 1,
            "the wake must be served by the spin, not the sleep poll (wakes {wakes})"
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
        assert!(taken, "the woken wait's message must be takeable");
        assert_eq!(out, oracle);

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// (d) A guard condition triggered while the waiter is provably inside
/// its spin wakes the wait (served by the spin: `spin_wakes >= 1`) and is
/// consumed exactly once (the H-2 contract preserved through the spin):
/// the woken pass reports it ready, and a subsequent zero-timeout wait
/// times out with the slot nulled AND runs no spin probe — the trigger
/// was not double-counted, not lost, and the zero timeout never spun.
#[test]
#[serial]
fn guard_triggered_mid_wait_wakes_and_is_consumed_exactly_once() {
    let _env = EnvVarGuard::set(SPIN_US_VALUE);
    unsafe {
        let suffix = unique_suffix();
        let (context, node, _opts) = setup_node(&format!("spin_d_{suffix}"));

        let gc = rmw_create_guard_condition(context);
        assert!(!gc.is_null());
        let ws = rmw_create_wait_set(context, 4);

        let gc_addr = gc as usize;
        let ws_addr = ws as usize;
        let trigger_thread = std::thread::spawn(move || {
            await_spin_entry(ws_addr as *mut ffi::rmw_wait_set_t);
            let gc = gc_addr as *const ffi::rmw_guard_condition_t;
            assert_eq!(rmw_trigger_guard_condition(gc), RMW_RET_OK);
        });

        let mut gc_ptrs = [(*gc).data];
        let mut guards = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: gc_ptrs.as_mut_ptr(),
        };
        let timeout = ffi::rmw_time_t { sec: 10, nsec: 0 };
        let start = Instant::now();
        let ret = rmw_wait(
            std::ptr::null_mut(),
            &mut guards,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &timeout,
        );
        let elapsed = start.elapsed();
        trigger_thread.join().expect("trigger thread");

        assert_eq!(ret, RMW_RET_OK, "mid-wait trigger must wake the wait");
        // Hang guard (seconds-scale ceiling, never a band).
        assert!(
            elapsed < Duration::from_secs(5),
            "the wake must be prompt, not near the timeout (elapsed {elapsed:?})"
        );
        assert!(!gc_ptrs[0].is_null(), "guard must be reported ready");
        let (probes_before, wakes) = spin_counters(ws);
        assert!(
            wakes >= 1,
            "the trigger must be observed by the spin (wakes {wakes})"
        );

        // Consumed exactly once: the next zero-timeout wait sees nothing
        // — and spins nothing (probes unchanged across the call).
        let mut gc_ptrs2 = [(*gc).data];
        let mut guards2 = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: gc_ptrs2.as_mut_ptr(),
        };
        let zero = ffi::rmw_time_t { sec: 0, nsec: 0 };
        let start2 = Instant::now();
        let ret2 = rmw_wait(
            std::ptr::null_mut(),
            &mut guards2,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &zero,
        );
        let elapsed2 = start2.elapsed();
        assert_eq!(ret2, ffi::RMW_RET_TIMEOUT, "trigger must be consumed once");
        // Hang guard (seconds-scale ceiling, never a band).
        assert!(
            elapsed2 < Duration::from_secs(1),
            "zero-timeout must return immediately (elapsed {elapsed2:?})"
        );
        assert!(gc_ptrs2[0].is_null(), "not-ready guard must be nulled");
        let (probes_after, wakes_after) = spin_counters(ws);
        assert_eq!(
            probes_after, probes_before,
            "a zero-timeout wait must run no spin probe"
        );
        assert_eq!(wakes_after, wakes, "no phantom wake on the consumed guard");

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_guard_condition(gc), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// (c) A zero-timeout `rmw_wait` with nothing ready returns
/// RMW_RET_TIMEOUT with the not-ready slot nulled and runs ZERO spin
/// probes — the spin never runs past the caller's deadline, pinned on
/// the counter rather than a wall band (this binary's cached budget is
/// the 100 ms cap, so a deadline-ignoring spin would score probes > 0).
#[test]
#[serial]
fn zero_timeout_wait_returns_timeout_immediately_without_spinning() {
    let _env = EnvVarGuard::set(SPIN_US_VALUE);
    unsafe {
        let suffix = unique_suffix();
        let type_name = format!("SpinC{suffix}");
        let ts = point_ts(&type_name);
        let (context, node, _opts) = setup_node(&format!("spin_c_{suffix}"));

        let topic = CString::new(format!("/rmw_spin/c/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());

        let mut sub_ptrs = [(*subscription).data];
        let mut subs = ffi::rmw_subscriptions_t {
            subscriber_count: 1,
            subscribers: sub_ptrs.as_mut_ptr(),
        };
        let zero = ffi::rmw_time_t { sec: 0, nsec: 0 };
        let ws = rmw_create_wait_set(context, 8);
        let start = Instant::now();
        let ret = rmw_wait(
            &mut subs,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &zero,
        );
        let elapsed = start.elapsed();

        assert_eq!(ret, ffi::RMW_RET_TIMEOUT, "nothing ready ⇒ TIMEOUT");
        // Hang guard (seconds-scale ceiling, never a band).
        assert!(
            elapsed < Duration::from_secs(1),
            "zero-timeout must return immediately (elapsed {elapsed:?})"
        );
        assert!(
            sub_ptrs[0].is_null(),
            "not-ready subscription must be nulled"
        );
        assert_eq!(
            spin_counters(ws),
            (0, 0),
            "a zero-timeout wait must run no spin probe and score no wake"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
