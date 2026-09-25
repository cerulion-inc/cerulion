// SPDX-License-Identifier: AGPL-3.0-only
//! The event-driven `rmw_wait` — blocking waits over the
//! subscriptions' existing iceoryx2 event listeners + the guard-condition
//! fd doorbells, over REAL iceoryx2 shared memory.
//!
//! What is pinned (hand oracles; wall assertions are GENEROUS — lower
//! bounds where a spin would return early, multi-second ceilings where
//! load can only delay):
//!
//! * a publish WAKES a blocked wait (functional round trip) and the wake
//!   really rides the fd path (`WaitSetData::fd_wakes` accumulated over
//!   rounds — a sleep-poll implementation scores 0);
//! * the ready set is EXACT across multiple subscriptions;
//! * a guard trigger wakes a blocked wait, is consumed exactly once, and
//!   its doorbell is DRAINED — the second wait times out with ZERO fd
//!   wakes (THE re-fire pin: skip any drain and the residual doorbell
//!   byte fires the block instantly, deterministically failing the
//!   zero-delta assert);
//! * zero-timeout waits NEVER block (`fd_blocks == 0` across 100 calls —
//!   the rcl `spin_some` / timer-ready shape);
//! * services and clients wake through their own request/reply listeners
//!   (the same fd path — a wait implementation that left them polling at
//!   the pump cap scores 0 fd wakes on their wait sets);
//! * the TRANSIENT_LOCAL pump keeps its ≤20ms cadence while a taker is
//!   PARKED (an uncapped block starves the late joiner — deterministic);
//! * `CERULION_RMW_EVENT_WAIT=off` restores the sleep-poll
//!   (`fd_blocks == 0` while the functional round trip still works);
//! * a wait with no entities spends its timeout (bounded sleep, no spin);
//! * the PARK tier: on Linux a publish wakes a parked wait through the
//!   topic DOORBELL (`park_blocks`/`park_wakes_doorbell`; the fd block is
//!   entered only for a wait the doorbell did not serve, because a FORCED
//!   park runs the bounded `FirstRungOnce` shape), off Linux the doorbell
//!   is a compile-time stub and the same waits pin the fd tier. And
//!   `CERULION_MONITOR_WAIT=0` forces the fd tier EVERYWHERE, which is
//!   why the fd-counter pins below run under it (they pin the SAME tier
//!   on every platform).
//!
//! ⚠️ iceoryx2 shared memory is a process singleton — run with
//! `--test-threads=1`:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_wait_event_test -- --test-threads=1
//! ```

#![cfg(unix)]

use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;

// =====================================================================
// Fixtures (cribbed from rmw_e2e_test.rs)
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
    make_message_ts(
        "rmw_wev__msg",
        unique,
        std::mem::size_of::<CPoint>(),
        vec![
            member("x", ROS_TYPE_DOUBLE, 0),
            member("y", ROS_TYPE_DOUBLE, 8),
            member("z", ROS_TYPE_DOUBLE, 16),
        ],
    )
}

/// Request { a: f64 } / Response { sum: f64 } — both plain f64 so the
/// take-side oracle needs no string plumbing.
fn service_ts(unique: &str) -> *const ffi::rosidl_service_type_support_t {
    let req = make_message_ts(
        "rmw_wev__srv",
        &format!("{unique}_Request"),
        std::mem::size_of::<f64>(),
        vec![member("a", ROS_TYPE_DOUBLE, 0)],
    );
    let resp = make_message_ts(
        "rmw_wev__srv",
        &format!("{unique}_Response"),
        std::mem::size_of::<f64>(),
        vec![member("sum", ROS_TYPE_DOUBLE, 0)],
    );
    let req_members =
        unsafe { (*req).data } as *const ffi::rosidl_typesupport_introspection_c__MessageMembers;
    let resp_members =
        unsafe { (*resp).data } as *const ffi::rosidl_typesupport_introspection_c__MessageMembers;
    let sm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__ServiceMembers {
            service_namespace_: cstr("rmw_wev__srv"),
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

static UNIQUE: AtomicU64 = AtomicU64::new(0);

fn unique_suffix() -> u64 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos() as u64;
    // The pid term is LOAD-BEARING, not decoration: doorbell pages are
    // `/cer_db_{ns}_{fnv(topic)}` under the SHARED `default_namespace()`,
    // and under nextest each test is its own concurrent PROCESS — so
    // cross-process page uniqueness rides the TOPIC, which this suffix
    // makes pid-scoped by construction (the serial-discipline walk's
    // requirement, carried here rather than in the namespace).
    nanos ^ (u64::from(std::process::id()) << 40) ^ UNIQUE.fetch_add(1, Ordering::Relaxed)
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

/// Panic-safe env manipulation (the repository's `EnvVarGuard` pattern):
/// removes the var on drop, panicking test or not.
struct EnvVarGuard {
    key: &'static str,
}

impl EnvVarGuard {
    fn set(key: &'static str, value: &str) -> Self {
        std::env::set_var(key, value);
        Self { key }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.key);
    }
}

/// Counter snapshot off the opaque wait-set data pointer. ONLY sound when
/// no `rmw_wait` on this wait set is in flight (read-after-join in every
/// test below).
unsafe fn wait_counters(ws: *mut ffi::rmw_wait_set_t) -> (u64, u64, u64) {
    let data = &*((*ws).data as *const WaitSetData);
    (
        data.fd_blocks.load(Ordering::Relaxed),
        data.fd_wakes.load(Ordering::Relaxed),
        data.timeout_wakes.load(Ordering::Relaxed),
    )
}

/// The blocks-entered observable: `fd_blocks + park_blocks` (each
/// increments right before its tier's block is armed), so the sync below
/// works on whichever tier this platform/env runs.
unsafe fn blocks_entered(ws: *mut ffi::rmw_wait_set_t) -> u64 {
    let data = &*((*ws).data as *const WaitSetData);
    data.fd_blocks.load(Ordering::Relaxed) + data.park_blocks.load(Ordering::Relaxed)
}

/// Block until the waiter on `ws` has entered a FRESH kernel block —
/// [`blocks_entered`] moved past `seen`. A ring/trigger/publish issued right after this lands INSIDE that
/// block and wakes it through the fd, rather than in the loop-head window
/// between blocks where the drain would eat the signal before the probe
/// serves it. A sleep cannot give that guarantee — on macOS the waiter's
/// poll timeout and a test sleep are timer-COALESCED onto the same tick,
/// so a sleep-timed signal lands in the head window far more often than a
/// random phase would (measured ~30 % here). Bounded so a waiter that
/// never blocks fails loudly.
unsafe fn await_fresh_block(ws: *mut ffi::rmw_wait_set_t, seen: u64) -> u64 {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let now = blocks_entered(ws);
        if now > seen {
            return now;
        }
        assert!(
            Instant::now() < deadline,
            "the waiter never entered a fresh block"
        );
        std::hint::spin_loop();
    }
}

/// Block until the waiter's ladder has climbed to the 20 ms cap (bounded).
unsafe fn await_cap_rung(ws: *mut ffi::rmw_wait_set_t) {
    let data = &*((*ws).data as *const WaitSetData);
    let deadline = Instant::now() + Duration::from_secs(10);
    while data.block_rung_us() < 20_000 {
        assert!(
            Instant::now() < deadline,
            "the waiter never reached the cap rung"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Run `rmw_wait` on an EXECUTOR-shaped set — one subscription, N guard
/// conditions, M services — in a thread; returns (ret, sub_ready).
unsafe fn wait_on_full_set_in_thread(
    sub_data: *mut c_void,
    guard_datas: Vec<usize>,
    service_datas: Vec<usize>,
    ws: *mut ffi::rmw_wait_set_t,
    timeout: Duration,
) -> std::thread::JoinHandle<(ffi::rmw_ret_t, bool)> {
    let sub_addr = sub_data as usize;
    let ws_addr = ws as usize;
    std::thread::spawn(move || unsafe {
        let mut sub_ptrs = [sub_addr as *mut c_void];
        let mut subs = ffi::rmw_subscriptions_t {
            subscriber_count: 1,
            subscribers: sub_ptrs.as_mut_ptr(),
        };
        let mut gc_ptrs: Vec<*mut c_void> = guard_datas.iter().map(|&a| a as *mut c_void).collect();
        let mut guards = ffi::rmw_guard_conditions_t {
            guard_condition_count: gc_ptrs.len(),
            guard_conditions: gc_ptrs.as_mut_ptr(),
        };
        let mut svc_ptrs: Vec<*mut c_void> =
            service_datas.iter().map(|&a| a as *mut c_void).collect();
        let mut svcs = ffi::rmw_services_t {
            service_count: svc_ptrs.len(),
            services: svc_ptrs.as_mut_ptr(),
        };
        let t = ffi::rmw_time_t {
            sec: timeout.as_secs(),
            nsec: u64::from(timeout.subsec_nanos()),
        };
        let ret = rmw_wait(
            &mut subs,
            &mut guards,
            &mut svcs,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws_addr as *mut ffi::rmw_wait_set_t,
            &t,
        );
        (ret, !sub_ptrs[0].is_null())
    })
}

/// Run `rmw_wait` on ONE subscription in a thread; returns (ret, ready).
unsafe fn wait_on_sub_in_thread(
    sub_data: *mut c_void,
    ws: *mut ffi::rmw_wait_set_t,
    timeout: Duration,
) -> std::thread::JoinHandle<(ffi::rmw_ret_t, bool)> {
    let sub_addr = sub_data as usize;
    let ws_addr = ws as usize;
    std::thread::spawn(move || unsafe {
        let mut sub_ptrs = [sub_addr as *mut c_void];
        let mut subs = ffi::rmw_subscriptions_t {
            subscriber_count: 1,
            subscribers: sub_ptrs.as_mut_ptr(),
        };
        let t = ffi::rmw_time_t {
            sec: timeout.as_secs(),
            nsec: u64::from(timeout.subsec_nanos()),
        };
        let ret = rmw_wait(
            &mut subs,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws_addr as *mut ffi::rmw_wait_set_t,
            &t,
        );
        (ret, !sub_ptrs[0].is_null())
    })
}

// =====================================================================
// The pins
// =====================================================================

/// A blocked wait wakes on a publish, the take succeeds, and — across
/// rounds — at least one wake rode the fd path (a sleep-poll implementation, or
/// an fd snapshot that missed subscriptions, scores `fd_wakes == 0`).
/// Rounds accumulate because any single publish can land in the tiny
/// probe window between blocks (ready-return without an fd wake) —
/// legitimate, so the pin is cumulative, not per-round. Runs under
/// `CERULION_MONITOR_WAIT=0` and DOUBLES as that flag's disable pin: with
/// the park forced off, the doorbell tier must be fully inert on every
/// platform (Linux would otherwise park here — the park half lives in
/// `a_publish_wakes_a_parked_wait_through_the_topic_doorbell`).
#[test]
#[serial]
fn a_publish_wakes_a_blocked_wait_and_take_succeeds() {
    // Spin disabled: a spin-caught publish is a legitimate non-fd wake
    // that would dilute the cumulative counter for no coverage gain (the
    // default-spin path is exercised by every other test here).
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    // Park disabled: this is the fd-tier pin (and the disable pin — see
    // the doc); the park tier has its own test below.
    let _park = EnvVarGuard::set("CERULION_MONITOR_WAIT", "0");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevA{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_a_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/a/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let ws = rmw_create_wait_set(context, 8);

        for round in 0..10u32 {
            let handle = wait_on_sub_in_thread((*subscription).data, ws, Duration::from_secs(5));
            std::thread::sleep(Duration::from_millis(30));
            let msg = CPoint {
                x: f64::from(round),
                y: -1.0,
                z: 0.25,
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let (ret, ready) = handle.join().expect("wait thread");
            assert_eq!(ret, RMW_RET_OK, "round {round}: publish must wake the wait");
            assert!(
                ready,
                "round {round}: subscription must be in the ready set"
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
            assert!(taken, "round {round}: take must succeed after the wake");
            assert_eq!(out, msg, "round {round}: payload oracle");
        }

        let (blocks, fd_wakes, _timeouts) = wait_counters(ws);
        assert!(
            blocks >= 1,
            "the waits must actually have blocked (got {blocks})"
        );
        assert!(
            fd_wakes >= 1,
            "across 10 blocked-publish rounds at least one wake must ride \
             the fd path — 0 means the block is not watching the \
             subscription's listener (sleep-poll regression)"
        );
        // The disable pin: with the park forced off the doorbell tier is
        // fully inert — on every platform, Linux included.
        let data = &*((*ws).data as *const WaitSetData);
        assert_eq!(
            data.park_blocks.load(Ordering::Relaxed),
            0,
            "CERULION_MONITOR_WAIT=0 must disable the park tier"
        );
        assert_eq!(
            data.doorbell_topics(),
            0,
            "a park-disabled wait must not map doorbell pages"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Two subscriptions attached, one published: the ready set contains
/// EXACTLY the published one (the other slot is nulled per the rmw
/// contract).
#[test]
#[serial]
fn ready_set_is_exact_across_multiple_subscriptions() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevB{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_b_{suffix}"));
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let topic1 = CString::new(format!("/rmw_wev/b1/{suffix}")).expect("topic");
        let topic2 = CString::new(format!("/rmw_wev/b2/{suffix}")).expect("topic");
        let sub1 = rmw_create_subscription(node, ts, topic1.as_ptr(), &qos, &sub_opts);
        let sub2 = rmw_create_subscription(node, ts, topic2.as_ptr(), &qos, &sub_opts);
        assert!(!sub1.is_null() && !sub2.is_null());
        let pub2 = rmw_create_publisher(node, ts, topic2.as_ptr(), &qos, &pub_opts);
        assert!(!pub2.is_null());

        let msg = CPoint {
            x: 4.0,
            y: 5.0,
            z: 6.0,
        };
        assert_eq!(
            rmw_publish(
                pub2,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );

        let ws = rmw_create_wait_set(context, 8);
        let mut sub_ptrs = [(*sub1).data, (*sub2).data];
        let mut subs = ffi::rmw_subscriptions_t {
            subscriber_count: 2,
            subscribers: sub_ptrs.as_mut_ptr(),
        };
        let timeout = ffi::rmw_time_t { sec: 2, nsec: 0 };
        let ret = rmw_wait(
            &mut subs,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &timeout,
        );
        assert_eq!(ret, RMW_RET_OK);
        assert!(sub_ptrs[0].is_null(), "unpublished sub1 must be nulled");
        assert!(!sub_ptrs[1].is_null(), "published sub2 must be ready");

        let mut out = CPoint::default();
        let mut taken = false;
        assert_eq!(
            rmw_take(
                sub2,
                &mut out as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(taken);
        assert_eq!(out, msg);

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, pub2), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, sub1), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, sub2), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// THE doorbell re-fire pin. A guard triggered from another thread wakes
/// the blocked wait and is consumed exactly once; the SECOND wait must
/// spend its full timeout with ZERO fd wakes — a skipped doorbell drain
/// leaves the ring byte level-readable, and the second wait's very first
/// block then fires instantly (deterministic `fd_wakes` delta ≥ 1).
#[test]
#[serial]
fn a_guard_trigger_wakes_a_blocked_wait_and_its_doorbell_is_drained() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    unsafe {
        let suffix = unique_suffix();
        let (context, node, _opts) = setup_node(&format!("wev_c_{suffix}"));
        let gc = rmw_create_guard_condition(context);
        assert!(!gc.is_null());
        let ws = rmw_create_wait_set(context, 4);

        // Wait in a thread; trigger while it is (very probably) blocked.
        let gc_addr = (*gc).data as usize;
        let ws_addr = ws as usize;
        let handle = std::thread::spawn(move || {
            let mut gc_ptrs = [gc_addr as *mut c_void];
            let mut guards = ffi::rmw_guard_conditions_t {
                guard_condition_count: 1,
                guard_conditions: gc_ptrs.as_mut_ptr(),
            };
            let timeout = ffi::rmw_time_t { sec: 5, nsec: 0 };
            let ret = rmw_wait(
                std::ptr::null_mut(),
                &mut guards,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                ws_addr as *mut ffi::rmw_wait_set_t,
                &timeout,
            );
            (ret, !gc_ptrs[0].is_null())
        });
        // Trigger INSIDE a cap-rung block: wait for the ladder to reach the
        // 20 ms cap, then for the waiter to enter a fresh block, then ring —
        // the trigger's doorbell byte lands in a 20 ms poll, never in the
        // loop-head window (see `await_fresh_block`).
        await_cap_rung(ws);
        await_fresh_block(ws, blocks_entered(ws));
        assert_eq!(rmw_trigger_guard_condition(gc), RMW_RET_OK);
        let (ret, ready) = handle.join().expect("wait thread");
        assert_eq!(ret, RMW_RET_OK, "the trigger must wake the blocked wait");
        assert!(ready, "the guard must be in the ready set");
        // The wake must have ridden the guard's doorbell fd: a snapshot that
        // omits guard doorbells still returns here via the 20 ms pump-cap
        // timeout + probe, but scores no fd wake.
        let (_, fd_wakes_first, _) = wait_counters(ws);
        assert!(
            fd_wakes_first >= 1,
            "the guard trigger must wake the block through its doorbell fd \
             (0 = guard doorbells are not in the fd snapshot)"
        );

        // Second wait: trigger consumed AND doorbell drained — full
        // timeout, zero fd wakes.
        let (_, wakes_before, _) = wait_counters(ws);
        let mut gc_ptrs2 = [(*gc).data];
        let mut guards2 = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: gc_ptrs2.as_mut_ptr(),
        };
        let timeout2 = ffi::rmw_time_t {
            sec: 0,
            nsec: 300_000_000,
        };
        let start = Instant::now();
        let ret2 = rmw_wait(
            std::ptr::null_mut(),
            &mut guards2,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &timeout2,
        );
        assert_eq!(ret2, ffi::RMW_RET_TIMEOUT, "consumed guard: must time out");
        assert!(gc_ptrs2[0].is_null(), "not-ready guard must be nulled");
        assert!(
            start.elapsed() >= Duration::from_millis(250),
            "the second wait must spend its timeout (generous lower bound)"
        );
        let (_, wakes_after, _) = wait_counters(ws);
        assert_eq!(
            wakes_after - wakes_before,
            0,
            "the consumed trigger's doorbell must be DRAINED — a residual \
             ring byte re-fires the block instantly"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_guard_condition(gc), RMW_RET_OK);
        // The node registers its graph guard PROCESS-globally; leaving it
        // alive would let every later graph change ring a stale guard.
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Zero-timeout waits (rcl passes 0 whenever a timer is already ready;
/// `spin_some` polls with 0) must return after ONE probe — never spin,
/// never block. 100 calls with nothing ready: all TIMEOUT, zero kernel
/// blocks, seconds-generous total wall.
#[test]
#[serial]
fn zero_timeout_waits_never_block() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevD{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_d_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/d/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let ws = rmw_create_wait_set(context, 8);

        let start = Instant::now();
        for _ in 0..100 {
            let mut sub_ptrs = [(*subscription).data];
            let mut subs = ffi::rmw_subscriptions_t {
                subscriber_count: 1,
                subscribers: sub_ptrs.as_mut_ptr(),
            };
            let zero = ffi::rmw_time_t { sec: 0, nsec: 0 };
            assert_eq!(
                rmw_wait(
                    &mut subs,
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    ws,
                    &zero,
                ),
                ffi::RMW_RET_TIMEOUT
            );
            assert!(sub_ptrs[0].is_null());
        }
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "100 zero-timeout waits must be prompt (bounded at seconds, not µs)"
        );
        let (blocks, _, _) = wait_counters(ws);
        assert_eq!(
            blocks, 0,
            "a zero-timeout wait must NEVER enter the kernel block"
        );
        let data = &*((*ws).data as *const WaitSetData);
        assert_eq!(
            data.park_blocks.load(Ordering::Relaxed),
            0,
            "…nor the park tier (a park is a block too)"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Services and clients wake through their own request/reply listeners —
/// the SAME fd path as subscriptions. Cumulative `fd_wakes` on each side's
/// wait set pins that their fds are genuinely in the snapshot: an
/// implementation that left them polling at the 20ms pump cap still
/// passes the functional asserts but scores 0 fd wakes.
#[test]
#[serial]
fn services_and_clients_wake_through_their_listeners() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    unsafe {
        let suffix = unique_suffix();
        let ts = service_ts(&format!("WevSum{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_e_{suffix}"));
        let service_name = CString::new(format!("/rmw_wev/sum/{suffix}")).expect("name");
        let qos = default_qos();
        let service = rmw_create_service(node, ts, service_name.as_ptr(), &qos);
        assert!(!service.is_null());
        let client = rmw_create_client(node, ts, service_name.as_ptr(), &qos);
        assert!(!client.is_null());

        let ws_svc = rmw_create_wait_set(context, 4);
        let ws_cli = rmw_create_wait_set(context, 4);

        for round in 0..5u32 {
            // --- Server side: park on the service, send a request. ---
            let svc_addr = (*service).data as usize;
            let ws_addr = ws_svc as usize;
            let handle = std::thread::spawn(move || {
                let mut svc_ptrs = [svc_addr as *mut c_void];
                let mut svcs = ffi::rmw_services_t {
                    service_count: 1,
                    services: svc_ptrs.as_mut_ptr(),
                };
                let timeout = ffi::rmw_time_t { sec: 5, nsec: 0 };
                let ret = rmw_wait(
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    &mut svcs,
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    ws_addr as *mut ffi::rmw_wait_set_t,
                    &timeout,
                );
                (ret, !svc_ptrs[0].is_null())
            });
            std::thread::sleep(Duration::from_millis(30));
            let request: f64 = f64::from(round) + 0.5;
            let mut sequence_id: i64 = 0;
            assert_eq!(
                rmw_send_request(
                    client,
                    &request as *const _ as *const c_void,
                    &mut sequence_id
                ),
                RMW_RET_OK
            );
            let (ret, ready) = handle.join().expect("service wait thread");
            assert_eq!(
                ret, RMW_RET_OK,
                "round {round}: request must wake the server"
            );
            assert!(ready, "round {round}: service must be in the ready set");

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
            assert_eq!(req_out, request, "round {round}: request oracle");

            // --- Client side: park on the client, send the response. ---
            let cli_addr = (*client).data as usize;
            let ws_addr = ws_cli as usize;
            let handle = std::thread::spawn(move || {
                let mut cli_ptrs = [cli_addr as *mut c_void];
                let mut cls = ffi::rmw_clients_t {
                    client_count: 1,
                    clients: cli_ptrs.as_mut_ptr(),
                };
                let timeout = ffi::rmw_time_t { sec: 5, nsec: 0 };
                let ret = rmw_wait(
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    &mut cls,
                    std::ptr::null_mut(),
                    ws_addr as *mut ffi::rmw_wait_set_t,
                    &timeout,
                );
                (ret, !cli_ptrs[0].is_null())
            });
            std::thread::sleep(Duration::from_millis(30));
            let mut response: f64 = request * 2.0;
            assert_eq!(
                rmw_send_response(
                    service,
                    &mut header.request_id,
                    &mut response as *mut _ as *mut c_void
                ),
                RMW_RET_OK
            );
            let (ret, ready) = handle.join().expect("client wait thread");
            assert_eq!(
                ret, RMW_RET_OK,
                "round {round}: response must wake the client"
            );
            assert!(ready, "round {round}: client must be in the ready set");

            let mut resp_out: f64 = 0.0;
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
            assert_eq!(resp_out, request * 2.0, "round {round}: response oracle");
        }

        let (_, svc_wakes, _) = wait_counters(ws_svc);
        let (_, cli_wakes, _) = wait_counters(ws_cli);
        assert!(
            svc_wakes >= 1,
            "service waits must wake via the request listener's fd \
             (0 = services are silently unwaited, left to the pump cap)"
        );
        assert!(
            cli_wakes >= 1,
            "client waits must wake via the reply listener's fd \
             (0 = clients are silently unwaited, left to the pump cap)"
        );

        assert_eq!(rmw_destroy_wait_set(ws_svc), RMW_RET_OK);
        assert_eq!(rmw_destroy_wait_set(ws_cli), RMW_RET_OK);
        assert_eq!(rmw_destroy_client(node, client), RMW_RET_OK);
        assert_eq!(rmw_destroy_service(node, service), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The 20ms block-cap pin: TRANSIENT_LOCAL history must reach a late
/// joiner while the only rmw_wait in the process is PARKED on an
/// unrelated, silent topic. The parked thread's ≤20ms wakeups are the
/// ONLY pump in this test (the main thread never calls rmw_wait) — an
/// uncapped block starves the late joiner deterministically.
#[test]
#[serial]
fn transient_local_pump_still_delivers_while_a_taker_is_parked() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevF{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_f_{suffix}"));
        let qos_tl = {
            let mut q = default_qos();
            q.durability = ffi::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
            q.depth = 4;
            q
        };
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        // Latched publisher, idle after one publish.
        let tl_topic = CString::new(format!("/rmw_wev/f_tl/{suffix}")).expect("topic");
        let publisher = rmw_create_publisher(node, ts, tl_topic.as_ptr(), &qos_tl, &pub_opts);
        assert!(!publisher.is_null());
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

        // A silent, unrelated topic to park on.
        let quiet_topic = CString::new(format!("/rmw_wev/f_quiet/{suffix}")).expect("topic");
        let qos = default_qos();
        let quiet_sub = rmw_create_subscription(node, ts, quiet_topic.as_ptr(), &qos, &sub_opts);
        assert!(!quiet_sub.is_null());
        let ws = rmw_create_wait_set(context, 8);
        let handle = wait_on_sub_in_thread((*quiet_sub).data, ws, Duration::from_millis(1500));
        std::thread::sleep(Duration::from_millis(50)); // let it park

        // Late joiner on the latched topic. History delivery is driven by
        // the PARKED thread's pump — nothing here pumps.
        let late_sub = rmw_create_subscription(node, ts, tl_topic.as_ptr(), &qos_tl, &sub_opts);
        assert!(!late_sub.is_null());
        let deadline = Instant::now() + Duration::from_millis(1200);
        let mut out = CPoint::default();
        let mut taken = false;
        while Instant::now() < deadline {
            assert_eq!(
                rmw_take(
                    late_sub,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            if taken {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(
            taken,
            "TRANSIENT_LOCAL history must reach the late joiner while the \
             only waiter is parked — the ≤20ms block cap is what keeps the \
             pump alive (an uncapped block starves this)"
        );
        assert_eq!(out, msg);

        let (ret, ready) = handle.join().expect("parked thread");
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT, "the quiet topic never fires");
        assert!(!ready);

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, quiet_sub), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, late_sub), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The kill-switch: `CERULION_RMW_EVENT_WAIT=off` restores the
/// sleep-poll — the functional round trip still works, and the wait
/// NEVER enters the fd block (`fd_blocks == 0`). Runs with a NONZERO
/// shared spin knob and the park's auto default, so the zero
/// spin/park/ladder asserts below are discriminating rather than
/// vacuous (folds the deleted per-knob purity tests: the legacy loop is
/// restored VERBATIM — no spin, no park, no doorbell pages, and no
/// ladder state, `block_rung_us == 0` because the ready-return reset is
/// gated on the event path).
#[test]
#[serial]
fn kill_switch_restores_the_sleep_poll() {
    let _kill = EnvVarGuard::set("CERULION_RMW_EVENT_WAIT", "off");
    // A nonzero spin budget the kill-switch must IGNORE (spin_probes == 0
    // below would be vacuous under the park-first default of unset).
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "50");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevG{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_g_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/g/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let ws = rmw_create_wait_set(context, 8);

        let handle = wait_on_sub_in_thread((*subscription).data, ws, Duration::from_secs(2));
        std::thread::sleep(Duration::from_millis(50));
        let msg = CPoint {
            x: 3.0,
            y: 2.0,
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
        let (ret, ready) = handle.join().expect("wait thread");
        assert_eq!(ret, RMW_RET_OK, "sleep-poll mode must still wake on data");
        assert!(ready);
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

        let (blocks, _, _) = wait_counters(ws);
        assert_eq!(
            blocks, 0,
            "with the kill-switch off the wait must never enter the fd block"
        );
        // Kill-switch purity: OFF restores the legacy loop VERBATIM — no
        // spin either (CERULION_LIVE_SPIN_US is ignored on this path,
        // and it is set to 50 above precisely so this is discriminating).
        let data = &*((*ws).data as *const WaitSetData);
        assert_eq!(
            data.spin_probes.load(Ordering::Relaxed),
            0,
            "the kill-switch must disable the spin front too (probe + sleep only)"
        );
        // …and no ladder state, no park tier, no doorbell pages: the Sleep
        // arm never consults the ladder, the ready-return reset is gated on
        // the event path, and `use_park_cfg` is too.
        assert_eq!(
            data.block_rung_us(),
            0,
            "the kill-switch path must never arm the ladder"
        );
        assert_eq!(
            data.park_blocks.load(Ordering::Relaxed),
            0,
            "the kill-switch path must never park"
        );
        assert_eq!(
            data.doorbell_topics(),
            0,
            "the kill-switch path must not map doorbell pages"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// A wait with NO entities is a bounded sleep to its deadline — never an
/// early return, never a spin (poll on an empty snapshot degrades to a
/// plain sleep inside the 20ms cap loop).
#[test]
#[serial]
fn a_wait_with_no_entities_spends_its_timeout() {
    unsafe {
        let suffix = unique_suffix();
        let (context, node, _opts) = setup_node(&format!("wev_h_{suffix}"));
        let ws = rmw_create_wait_set(context, 4);
        let timeout = ffi::rmw_time_t {
            sec: 0,
            nsec: 100_000_000,
        };
        let start = Instant::now();
        let ret = rmw_wait(
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &timeout,
        );
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT);
        assert!(
            start.elapsed() >= Duration::from_millis(100),
            "an empty wait must spend its timeout"
        );
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "…and return promptly after it"
        );
        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Run `rmw_wait` on ONE subscription on the CALLING thread; returns
/// (ret, ready).
unsafe fn wait_on_sub(
    sub_data: *mut c_void,
    ws: *mut ffi::rmw_wait_set_t,
    timeout: Duration,
) -> (ffi::rmw_ret_t, bool) {
    let mut sub_ptrs = [sub_data];
    let mut subs = ffi::rmw_subscriptions_t {
        subscriber_count: 1,
        subscribers: sub_ptrs.as_mut_ptr(),
    };
    let t = ffi::rmw_time_t {
        sec: timeout.as_secs(),
        nsec: u64::from(timeout.subsec_nanos()),
    };
    let ret = rmw_wait(
        &mut subs,
        std::ptr::null_mut(),
        std::ptr::null_mut(),
        std::ptr::null_mut(),
        std::ptr::null_mut(),
        ws,
        &t,
    );
    (ret, !sub_ptrs[0].is_null())
}

/// The degraded arm (the busy-spin class): a listener whose
/// notification drain FAILS while its fd stays readable must NOT be
/// blocked on. Arranged for real, not simulated: the sticky drain fault
/// makes `rmw_take` skip its own stale-event drain, so after
/// publish → take the SentSample notification is still queued (fd
/// READABLE) while the sample is gone (probe NOT ready) — the exact shape
/// that turns a level-triggered `poll(2)` wait into a spin. A wait on that
/// subscription must time out by sleep pacing: `fd_blocks == 0`,
/// `fd_wakes == 0`, the call counted in `degraded_waits`, the latch's
/// unconditional total advanced; and once the fault clears the very next
/// call is back on the fd path. Mutant: ignore `degraded` in
/// `block_strategy` ⇒ the readable fd fires the block instantly on every
/// iteration and `fd_wakes` explodes.
#[test]
#[serial]
fn a_failing_listener_drain_degrades_to_sleep_pacing_never_a_spin() {
    // Pin the fd tier explicitly: the heal-phase assert reads `fd_blocks`,
    // which the Linux park tier would leave at 0.
    let _park = EnvVarGuard::set("CERULION_MONITOR_WAIT", "0");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevI{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_i_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/i/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let ws = rmw_create_wait_set(context, 8);

        // The seam is process-wide (the subscriber's layout is ABI-pinned, so
        // it cannot carry a per-instance flag); disarm on every exit path.
        struct DrainFaultGuard;
        impl Drop for DrainFaultGuard {
            fn drop(&mut self) {
                cerulion_core::transport::subscriber::fault_inject_drain_events_err_for_test(false);
            }
        }
        let _disarm = DrainFaultGuard;
        let arm = |armed: bool| {
            cerulion_core::transport::subscriber::fault_inject_drain_events_err_for_test(armed);
        };
        arm(true);

        // Publish, then take: with the drain faulted the take leaves the
        // notification queued while consuming the sample.
        let msg = CPoint {
            x: 1.5,
            y: 2.5,
            z: 3.5,
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
        assert!(
            taken,
            "the sample must still be delivered under the drain fault"
        );
        assert_eq!(out, msg);

        // Degraded wait #1: readable fd, nothing ready ⇒ TIMEOUT by pacing.
        let start = Instant::now();
        let (ret, ready) = wait_on_sub((*subscription).data, ws, Duration::from_millis(300));
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT);
        assert!(!ready);
        assert!(
            start.elapsed() >= Duration::from_millis(250),
            "a degraded wait must still spend its timeout"
        );
        let data = &*((*ws).data as *const WaitSetData);
        assert_eq!(
            data.fd_blocks.load(Ordering::Relaxed),
            0,
            "a degraded call must never enter the fd block (a readable fd \
             nobody can drain re-fires every block — a tight spin)"
        );
        assert_eq!(data.fd_wakes.load(Ordering::Relaxed), 0);
        assert_eq!(
            data.degraded_waits.load(Ordering::Relaxed),
            1,
            "the call is counted as degraded"
        );
        assert!(
            data.drain_failure_count() >= 1,
            "the latch's unconditional total must record the failure"
        );

        // Degraded wait #2: counted per CALL, still no fd block.
        let (ret, _) = wait_on_sub((*subscription).data, ws, Duration::from_millis(100));
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT);
        let data = &*((*ws).data as *const WaitSetData);
        assert_eq!(data.degraded_waits.load(Ordering::Relaxed), 2);
        assert_eq!(data.fd_blocks.load(Ordering::Relaxed), 0);

        // Heal: the next call drains cleanly and is back on the fd path.
        arm(false);
        let (ret, _) = wait_on_sub((*subscription).data, ws, Duration::from_millis(100));
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT);
        let data = &*((*ws).data as *const WaitSetData);
        assert_eq!(
            data.degraded_waits.load(Ordering::Relaxed),
            2,
            "a healthy call is not degraded"
        );
        assert!(
            data.fd_blocks.load(Ordering::Relaxed) >= 1,
            "once the drain heals the fd path resumes on the very next call"
        );
        assert_eq!(
            data.fd_wakes.load(Ordering::Relaxed),
            0,
            "the healed drain emptied the queue — no stale wake"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The spin front: with a generous budget a publish landing
/// shortly after wait entry is caught in USER SPACE — `spin_wakes`
/// accumulates across rounds (a build without the spin scores 0) and
/// `spin_probes` proves the front ran. Cumulative, not per-round: a round
/// whose publish lands before the waiter's first probe is a legitimate
/// ready-return, and under load one could outlast the budget and block.
#[test]
#[serial]
fn the_spin_front_catches_an_imminent_publish() {
    // The cap itself (100 ms) — also exercises the at-cap admit.
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "100000");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevJ{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_j_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/j/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let ws = rmw_create_wait_set(context, 8);

        for round in 0..10u32 {
            let handle = wait_on_sub_in_thread((*subscription).data, ws, Duration::from_secs(5));
            std::thread::sleep(Duration::from_millis(10));
            let msg = CPoint {
                x: f64::from(round),
                y: 0.5,
                z: -0.5,
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let (ret, ready) = handle.join().expect("wait thread");
            assert_eq!(ret, RMW_RET_OK, "round {round}");
            assert!(ready, "round {round}");
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

        let data = &*((*ws).data as *const WaitSetData);
        assert!(
            data.spin_probes.load(Ordering::Relaxed) >= 1,
            "the spin front must have run"
        );
        assert!(
            data.spin_wakes.load(Ordering::Relaxed) >= 1,
            "across 10 rounds at least one publish must be caught by the spin \
             (0 = the spin front is inert)"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// The adaptive block ladder
// =====================================================================

/// Ladder pin (i) — ping-pong stays on the FIRST rung: every round's
/// publish arrives well inside the idleness threshold, so each wake resets
/// the ladder (`block_rung_resets >= rounds`) and the rung is back at the
/// first one (200 µs) after the last wake. `block_backoffs` is PRINTED,
/// not asserted zero: the threshold is the fixed internal 50 empties
/// (~10 ms at the first rung — the raise-it-for-the-test knob is gone),
/// so a loaded runner descheduling the publisher >10 ms can legitimately
/// buy a backoff mid-run; the property under test is that traffic RETURNS
/// the ladder to the first rung, which holds for any N. Runs under
/// `CERULION_MONITOR_WAIT=0` so `fd_blocks` pins the same (fd) tier on
/// every platform.
#[test]
#[serial]
fn ping_pong_traffic_keeps_the_block_ladder_on_its_first_rung() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    let _park = EnvVarGuard::set("CERULION_MONITOR_WAIT", "0");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevK{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_k_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/k/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let ws = rmw_create_wait_set(context, 8);

        const ROUNDS: u32 = 20;
        for round in 0..ROUNDS {
            let handle = wait_on_sub_in_thread((*subscription).data, ws, Duration::from_secs(5));
            // Ping-pong cadence: the "reply" lands within a few first-rung
            // blocks (1 ms ≈ 5 blocks at the 200 µs first rung — far under
            // the 50-empty threshold), so the waiter really BLOCKS at the
            // first rung before it wakes, rather than finding the sample at
            // its very first probe.
            std::thread::sleep(Duration::from_millis(1));
            let msg = CPoint {
                x: f64::from(round),
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
            let (ret, ready) = handle.join().expect("wait thread");
            assert_eq!(ret, RMW_RET_OK, "round {round}");
            assert!(ready);
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

        let data = &*((*ws).data as *const WaitSetData);
        let resets = data.block_rung_resets.load(Ordering::Relaxed);
        let backoffs = data.block_backoffs.load(Ordering::Relaxed);
        eprintln!(
            "ladder(i) ping-pong: rounds={ROUNDS} block_rung_resets={resets} \
             block_backoffs={backoffs} fd_blocks={} fd_wakes={}",
            data.fd_blocks.load(Ordering::Relaxed),
            data.fd_wakes.load(Ordering::Relaxed)
        );
        assert!(
            resets >= u64::from(ROUNDS),
            "every ready-return must reset the ladder (got {resets} resets over {ROUNDS} rounds)"
        );
        assert_eq!(
            data.block_rung_us(),
            200,
            "after the last wake the ladder must be back at the first rung — \
             whatever a mid-run stall bought ({backoffs} backoffs along the \
             way), the wake resets it"
        );
        assert!(
            data.fd_blocks.load(Ordering::Relaxed) >= 1,
            "the waiter must actually have blocked (at the first rung) across the rounds"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Ladder pin (ii) — an IDLE wait set backs off: over a 250 ms wait on a
/// silent topic the ladder doubles (`block_backoffs > 0`) and the number
/// of kernel blocks — `fd_blocks + park_blocks`: on Linux a subscription-
/// only wait set maps its bell (the consumer-side open creates the page)
/// and PARKS, elsewhere it blocks on the fd; the ladder bounds both tiers
/// identically — stays FAR below the flat-100 µs count (2500 here):
/// the ladder's schedule is 50 empties at the first rung, a handful of
/// doublings, then the 20 ms cap ≈ 70 blocks. Asserted `< 500` — a
/// CPU-cost pin with a generous bound, not a wall band.
#[test]
#[serial]
fn an_idle_wait_set_climbs_the_ladder_and_stops_burning_wakeups() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    // Park FORCED on so the Linux `park_yields` arm below is meaningful on
    // x86 CI too (the x86 default is no park); the ladder bounds both
    // tiers identically, so the block-count pin is tier-agnostic.
    let _park = EnvVarGuard::set("CERULION_MONITOR_WAIT", "1");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevL{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_l_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/l/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let ws = rmw_create_wait_set(context, 8);

        let start = Instant::now();
        let (ret, ready) = wait_on_sub((*subscription).data, ws, Duration::from_millis(250));
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT);
        assert!(!ready);
        assert!(start.elapsed() >= Duration::from_millis(200));

        let data = &*((*ws).data as *const WaitSetData);
        let backoffs = data.block_backoffs.load(Ordering::Relaxed);
        let blocks = blocks_entered(ws);
        eprintln!(
            "ladder(ii) idle 250ms: blocks={blocks} block_backoffs={backoffs} \
             timeout_wakes={} (flat 100µs would be ~2500)",
            data.timeout_wakes.load(Ordering::Relaxed)
        );
        assert!(
            backoffs > 0,
            "an idle wait set must back off (got 0 backoffs)"
        );
        assert!(
            blocks < 500,
            "the ladder must cut the idle wakeup count well below the flat-100µs \
             2500 (got {blocks} blocks in 250ms)"
        );
        assert!(blocks >= 1, "…while still having blocked at all");
        // The park is OS-cooperative: on Linux this subscription-only set
        // parks under the forced hatch (the consumer-side open creates the
        // bell page) and every timed-out slice must have yielded the core;
        // off Linux the park is stubbed.
        let yields = data.park_yields.load(Ordering::Relaxed);
        if cfg!(target_os = "linux") {
            assert!(
                yields >= 1,
                "the call's park must yield the core per timed-out slice (got {yields})"
            );
        } else {
            assert_eq!(yields, 0, "no park off Linux ⇒ no yields");
        }

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Ladder pin (iii) — a publish AFTER backoff still wakes promptly via the
/// fd. Each round: the waiter idles long enough to climb to the 20 ms cap
/// (`block_backoffs` grows), then a publish must return well inside a
/// generous hang guard (< 50 ms after the publish — the wake is an fd, not
/// the cap timer) and reset the ladder. The fd-wake count is asserted
/// CUMULATIVELY: `1 <= fd_wakes delta <= rounds` — at least one round's
/// wake rode the fd (a single publish can land in the tens-of-µs loop-head
/// window between two cap blocks and be seen by the probe instead, a
/// legitimate ready-return), and never MORE than one per round (a
/// level-readable residue would re-fire the block, the busy-spin class).
#[test]
#[serial]
fn a_publish_after_backoff_still_wakes_promptly_through_the_fd() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    // This is the fd-tier promptness pin — the Linux park tier would serve
    // the wake through the doorbell and leave `fd_wakes` at 0.
    let _park = EnvVarGuard::set("CERULION_MONITOR_WAIT", "0");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevM{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_m_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/m/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let ws = rmw_create_wait_set(context, 8);
        let data = &*((*ws).data as *const WaitSetData);

        const ROUNDS: u64 = 3;
        let wakes_at_start = data.fd_wakes.load(Ordering::Relaxed);
        for round in 0..ROUNDS {
            let backoffs_before = data.block_backoffs.load(Ordering::Relaxed);
            let handle = wait_on_sub_in_thread((*subscription).data, ws, Duration::from_secs(10));
            // Idle past the threshold and up to the cap (50 × 100 µs + 8
            // doublings ≈ 45 ms); 300 ms is plenty.
            std::thread::sleep(Duration::from_millis(300));
            let backoffs_idle = data.block_backoffs.load(Ordering::Relaxed);
            assert!(
                backoffs_idle > backoffs_before,
                "round {round}: the waiter must have backed off before the publish"
            );

            let msg = CPoint {
                x: 9.0 + round as f64,
                y: 8.0,
                z: 7.0,
            };
            let published_at = Instant::now();
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let (ret, ready) = handle.join().expect("wait thread");
            let woke_after = published_at.elapsed();
            eprintln!(
                "ladder(iii) round {round}: backoffs_during_idle={} woke_after={woke_after:?} \
                 fd_wakes_so_far={} block_rung_resets={}",
                backoffs_idle - backoffs_before,
                data.fd_wakes.load(Ordering::Relaxed) - wakes_at_start,
                data.block_rung_resets.load(Ordering::Relaxed)
            );
            assert_eq!(ret, RMW_RET_OK, "round {round}");
            assert!(ready);
            assert!(
                woke_after < Duration::from_millis(50),
                "round {round}: a backed-off waiter must still wake on the fd, not the \
                 cap timer (woke {woke_after:?} after the publish)"
            );
            assert!(
                data.block_rung_resets.load(Ordering::Relaxed) > round,
                "round {round}: the wake must reset the ladder"
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

        let fd_wake_delta = data.fd_wakes.load(Ordering::Relaxed) - wakes_at_start;
        assert!(
            fd_wake_delta >= 1,
            "across {ROUNDS} rounds at least one post-backoff publish must wake through \
             the fd (0 = the cap rung is not fd-armed)"
        );
        assert!(
            fd_wake_delta <= ROUNDS,
            "never more than one fd wake per publish (got {fd_wake_delta} over {ROUNDS} \
             rounds — a level-readable residue re-firing the block)"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// Spin once per call; the recovery-spin bound
// =====================================================================

/// The shared knob: the spin ceiling is per call, but
/// an idle executor calls `rmw_wait` in a LOOP — at the shared
/// `CERULION_LIVE_SPIN_US` cap (100 ms, safe on the native loop's
/// imminence-gated spin) every idle iteration would burn a full
/// busy-probe before its first block, so the per-call bound alone bounds no
/// cumulative idle CPU. The entry spin is therefore armed only by a
/// PRODUCTIVE previous call: N consecutive empty waits run exactly ONE
/// spin phase (the fresh wait set's first call), a delivered message
/// re-arms, and the call after it spins again. Mutant: drop the disarm ⇒
/// one phase per idle call (N total).
#[test]
#[serial]
fn an_idle_executor_spins_once_per_idle_period_not_per_call() {
    // The cap itself — the value under test.
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "100000");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevN{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_n_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/n/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let ws = rmw_create_wait_set(context, 8);
        let data = &*((*ws).data as *const WaitSetData);

        // N consecutive EMPTY waits: only the FIRST (fresh, armed) spins.
        const IDLE_CALLS: u32 = 5;
        for call in 0..IDLE_CALLS {
            let (ret, _) = wait_on_sub((*subscription).data, ws, Duration::from_millis(60));
            assert_eq!(ret, ffi::RMW_RET_TIMEOUT, "idle call {call}");
        }
        let phases = data.spin_phases.load(Ordering::Relaxed);
        eprintln!(
            "idle-period spin: idle_calls={IDLE_CALLS} spin_phases={phases} spin_probes={}",
            data.spin_probes.load(Ordering::Relaxed)
        );
        assert_eq!(
            phases, 1,
            "across {IDLE_CALLS} consecutive empty waits the spin must run exactly \
             ONCE (the fresh wait set's first call) — one phase per idle CALL means \
             the per-call ceiling bounds no cumulative idle CPU (got {phases})"
        );

        // A DELIVERED message re-arms: the delivered call itself is still
        // disarmed (its predecessor was empty), the call AFTER it spins.
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
        let (ret, ready) = wait_on_sub((*subscription).data, ws, Duration::from_secs(2));
        assert_eq!(ret, RMW_RET_OK);
        assert!(ready);
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
            data.spin_phases.load(Ordering::Relaxed),
            1,
            "the delivered call itself is disarmed (its predecessor was empty)"
        );
        let (ret, _) = wait_on_sub((*subscription).data, ws, Duration::from_millis(60));
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT);
        assert_eq!(
            data.spin_phases.load(Ordering::Relaxed),
            2,
            "the call AFTER a delivered one must spin again (the re-arm proof)"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The post-wake recovery spin is bounded at ONE per call. A readable
/// doorbell does not guarantee the probe finds anything — here the guard's
/// doorbell is RUNG six times without a trigger (`Doorbell::ring` on the
/// public state), each ring waking the block
/// (`fd_wakes` counts them) with nothing ready. Per call the spin may run
/// on entry and at most ONCE more after such a wake: `spin_phases <= 2`.
/// Mutant: re-arming the recovery spin on every fd wake scores one phase
/// per ring (8 here) and fails.
#[test]
#[serial]
fn notification_without_readiness_never_re_enters_the_spin_more_than_once_per_call() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "50");
    unsafe {
        let suffix = unique_suffix();
        let (context, node, _opts) = setup_node(&format!("wev_q_{suffix}"));
        let gc = rmw_create_guard_condition(context);
        assert!(!gc.is_null());
        let ws = rmw_create_wait_set(context, 4);
        let data = &*((*ws).data as *const WaitSetData);

        let gc_addr = (*gc).data as usize;
        let ws_addr = ws as usize;
        let handle = std::thread::spawn(move || {
            let mut gc_ptrs = [gc_addr as *mut c_void];
            let mut guards = ffi::rmw_guard_conditions_t {
                guard_condition_count: 1,
                guard_conditions: gc_ptrs.as_mut_ptr(),
            };
            let timeout = ffi::rmw_time_t { sec: 10, nsec: 0 };
            let ret = rmw_wait(
                std::ptr::null_mut(),
                &mut guards,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                ws_addr as *mut ffi::rmw_wait_set_t,
                &timeout,
            );
            (ret, !gc_ptrs[0].is_null())
        });

        // Each ring lands INSIDE a fresh block (see `await_fresh_block`): the
        // ladder resets to the first rung on every fd wake, so the sync is
        // what keeps a ring from being drained in the loop-head window.
        const RINGS: u64 = 6;
        let state = &*((*gc).data as *const rmw_cerulion::runtime::GuardConditionState);
        let bell = state.doorbell.as_ref().expect("guard doorbell");
        let mut seen = blocks_entered(ws);
        for _ in 0..RINGS {
            seen = await_fresh_block(ws, seen);
            bell.ring(); // a notification with NOTHING ready
                         // Let the wake be served before the next sync (a ring racing
                         // the previous wake's drain would coalesce into one fd wake).
            std::thread::sleep(Duration::from_millis(5));
        }
        await_fresh_block(ws, blocks_entered(ws));
        assert_eq!(rmw_trigger_guard_condition(gc), RMW_RET_OK);
        let (ret, ready) = handle.join().expect("wait thread");
        assert_eq!(ret, RMW_RET_OK);
        assert!(ready);

        let fd_wakes = data.fd_wakes.load(Ordering::Relaxed);
        let phases = data.spin_phases.load(Ordering::Relaxed);
        eprintln!("spin-bound: rings={RINGS} fd_wakes={fd_wakes} spin_phases={phases}");
        // Floor at half the rings: a ring can still be lost to a >rung
        // deschedule of THIS thread between the sync and the ring (the
        // loaded-runner class); a doorbell-less snapshot scores 0 regardless.
        assert!(
            fd_wakes >= RINGS / 2,
            "the untriggered rings must have woken the block (got {fd_wakes} fd wakes \
             for {RINGS} rings; 0 = guard doorbells are not in the fd snapshot)"
        );
        assert!(
            phases <= 2,
            "the spin may run on entry and at most ONCE more after a wake per call \
             (got {phases} phases for {fd_wakes} fd wakes)"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_guard_condition(gc), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// An EMPTY wait set (no subscriptions/guards/services/clients) can never
/// observe readiness, so the spin must not run on it at all — an unguarded
/// 200 ms empty wait at the 100 ms cap measures ~480k probes
/// and ~100 ms of thread CPU. Pin: at the cap budget, an empty wait
/// returns TIMEOUT at the requested wall with `spin_phases == 0` and
/// `spin_probes == 0`; the CONTROL is the same budget with one (untriggered)
/// guard attached, which must spin (`spin_phases >= 1`) — so the pin cannot
/// pass on a build whose spin is simply broken. Mutant: resolving the
/// budget without the attached-entity gate scores 1 phase on the empty arm.
#[test]
#[serial]
fn an_empty_wait_never_spins_under_a_nonzero_budget() {
    // The cap: the most an attacker-shaped knob can ask for.
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "100000");
    unsafe {
        let suffix = unique_suffix();
        let (context, node, _opts) = setup_node(&format!("wev_r_{suffix}"));
        let ws = rmw_create_wait_set(context, 4);
        let data = &*((*ws).data as *const WaitSetData);

        // Empty arm.
        let timeout = ffi::rmw_time_t {
            sec: 0,
            nsec: 200_000_000,
        };
        let start = Instant::now();
        let ret = rmw_wait(
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &timeout,
        );
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT);
        assert!(
            start.elapsed() >= Duration::from_millis(150),
            "an empty wait must still spend its timeout"
        );
        let phases = data.spin_phases.load(Ordering::Relaxed);
        let probes = data.spin_probes.load(Ordering::Relaxed);
        eprintln!("empty-spin: spin_phases={phases} spin_probes={probes}");
        assert_eq!(phases, 0, "an empty wait set must never enter the spin");
        assert_eq!(probes, 0, "…and must run no probe at all");

        // Control: one attached (untriggered) guard under the same budget
        // MUST spin — the empty-arm zero is a decision, not a broken spin.
        // On a FRESH wait set: the empty arm above ended EMPTY and so
        // DISARMED `ws`'s next entry spin (the across-calls idle bound —
        // pinned by `an_idle_executor_spins_once_per_idle_period…`), and
        // this control asserts observability gating, not arming.
        let ws2 = rmw_create_wait_set(context, 4);
        let data2 = &*((*ws2).data as *const WaitSetData);
        let gc = rmw_create_guard_condition(context);
        assert!(!gc.is_null());
        let mut gc_ptrs = [(*gc).data];
        let mut guards = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: gc_ptrs.as_mut_ptr(),
        };
        let ret = rmw_wait(
            std::ptr::null_mut(),
            &mut guards,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws2,
            &timeout,
        );
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT);
        assert!(
            data2.spin_phases.load(Ordering::Relaxed) >= 1,
            "the control (one attached guard, fresh wait set) must spin under \
             the same budget"
        );
        assert!(data2.spin_probes.load(Ordering::Relaxed) >= 1);

        assert_eq!(rmw_destroy_wait_set(ws2), RMW_RET_OK);
        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_guard_condition(gc), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The park tier: a publish wakes a PARKED wait through the topic
/// DOORBELL — the SHM line the rmw publisher armed at create and rings on
/// every send (`notify_sent_sample`). Default env
/// (`CERULION_MONITOR_WAIT` unset = auto ON): on Linux the wait must map
/// exactly the one subscription topic's bell, take the park tier, score at
/// least one doorbell wake across the waits, and enter the fd tier only
/// for a wait the doorbell did not serve; off Linux the doorbell is a
/// compile-time stub, the park tier must never engage, and the SAME waits
/// must wake through the fd path instead. Both halves are asserted, so the
/// test is meaningful on every platform (the Linux half proves the
/// publisher's arming and the wait's bells meet on one page). The take +
/// payload oracle rides along each wait: the park is a WAKE tier, never a
/// data path.
///
/// The fd count is BOUNDED rather than zero for the same reason as the
/// two-publisher survivor arm below: a FORCED park on x86 runs the
/// `FirstRungOnce` shape, parking once for the ladder's first rung and
/// handing the rest of the idle to the fd tier, so a cross-thread publish
/// landing after that rung wakes an fd block with the product behaving
/// correctly.
#[test]
#[serial]
fn a_publish_wakes_a_parked_wait_through_the_topic_doorbell() {
    // Spin disabled: a spin-caught publish would be a legitimate non-park
    // wake that dilutes the tier counters for no coverage gain.
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    // Park FORCED on: the x86 default is no park (measured slower), so the
    // Linux arm pins the park tier under the explicit hatch; aarch64 parks
    // by default and off Linux the force is inert (doorbell stub).
    let _park = EnvVarGuard::set("CERULION_MONITOR_WAIT", "1");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevR{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_r_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/r/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        // Created BEFORE the first wait: the create arms the topic doorbell
        // (Linux), so the very first `refresh_park_bells` maps the page.
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let ws = rmw_create_wait_set(context, 8);

        const WAITS: u32 = 10;
        for round in 0..WAITS {
            let seen = blocks_entered(ws);
            let handle = wait_on_sub_in_thread((*subscription).data, ws, Duration::from_secs(5));
            // Publish INSIDE a fresh block (park or fd — whichever tier
            // this platform runs), not in the loop-head probe window.
            await_fresh_block(ws, seen);
            let msg = CPoint {
                x: f64::from(round),
                y: 6.0,
                z: -6.0,
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let (ret, ready) = handle.join().expect("wait thread");
            assert_eq!(ret, RMW_RET_OK, "round {round}: publish must wake the wait");
            assert!(ready, "round {round}");
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
            assert!(taken, "round {round}");
            assert_eq!(out, msg, "round {round}: payload oracle");
        }

        let data = &*((*ws).data as *const WaitSetData);
        let park_blocks = data.park_blocks.load(Ordering::Relaxed);
        let park_wakes = data.park_wakes_doorbell.load(Ordering::Relaxed);
        let fd_wakes = data.fd_wakes.load(Ordering::Relaxed);
        eprintln!(
            "park-tier: doorbell_topics={} park_blocks={park_blocks} \
             park_wakes_doorbell={park_wakes} fd_blocks={} fd_wakes={fd_wakes}",
            data.doorbell_topics(),
            data.fd_blocks.load(Ordering::Relaxed),
        );
        if cfg!(target_os = "linux") {
            assert_eq!(
                data.doorbell_topics(),
                1,
                "the one subscription topic's doorbell page must be mapped \
                 (0 = the publisher never armed it, or the wait never refreshed)"
            );
            assert!(
                park_blocks >= 1,
                "with a mapped bell the wait must take the park tier"
            );
            assert!(
                park_wakes >= 1,
                "across {WAITS} parked-publish waits at least one wake must ride \
                 the doorbell (0 = the publisher's ring never reaches the \
                 park, which then times out at the rung cadence instead)"
            );
            let fd_blocks = data.fd_blocks.load(Ordering::Relaxed);
            let unserved = u64::from(WAITS).saturating_sub(park_wakes);
            assert!(
                fd_blocks <= unserved,
                "a parked wait set falls back to the fd block only for a wait the \
                 doorbell did not serve: fd_blocks={fd_blocks} against {unserved} \
                 unserved wait(s) of {WAITS}"
            );
        } else {
            assert_eq!(
                data.doorbell_topics(),
                0,
                "off Linux the doorbell is a stub — no pages mapped"
            );
            assert_eq!(park_blocks, 0, "off Linux the park tier never engages");
            assert_eq!(park_wakes, 0);
            assert!(
                fd_wakes >= 1,
                "off Linux the same rounds must wake through the fd path"
            );
        }

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// A poisoned entity mutex makes the entity
/// permanently invisible to `check_ready` and absent from the fd
/// snapshot, so a wait set holding ONLY wedged entities is observably
/// EMPTY — it must not buy a spin budget (a non-null count would burn
/// the WHOLE budget in front of an empty fd wait, every call). The
/// healthy-entity control is
/// `an_empty_wait_never_spins_under_a_nonzero_budget`'s attached-guard
/// arm (something observable => the spin runs), so this pin cannot pass
/// on a spin that never runs at all. The wait itself must still be a
/// bounded TIMEOUT — a wedge degrades, never hangs, never returns ready.
#[test]
#[serial]
fn a_wait_set_of_only_wedged_entities_never_spins() {
    // The cap — the budget the old count would have burned per call.
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "100000");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevS{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_s_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/s/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let ws = rmw_create_wait_set(context, 8);

        // Poison the subscription's entity mutex: a thread panics while
        // holding it — the exact wedge `lock_unpoisoned` defends the rmw
        // against (never re-enter torn SHM bookkeeping).
        let data_addr = (*subscription).data as usize;
        let poisoner = std::thread::spawn(move || {
            // (Lexically inside the test's `unsafe` block, so no nested one.)
            let data = &*(data_addr as *const runtime::SubscriptionData);
            let _guard = data.inner.lock().expect("first lock is healthy");
            panic!("poison the entity mutex (deliberate)");
        });
        assert!(poisoner.join().is_err(), "the poisoner must have panicked");

        let t0 = Instant::now();
        let (ret, ready) = wait_on_sub((*subscription).data, ws, Duration::from_millis(200));
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT, "a wedged entity is never ready");
        assert!(!ready);
        assert!(
            t0.elapsed() >= Duration::from_millis(150),
            "the wait must still spend its timeout (bounded degrade, no hang)"
        );
        let data = &*((*ws).data as *const WaitSetData);
        assert_eq!(
            data.spin_phases.load(Ordering::Relaxed),
            0,
            "a wait set of only wedged entities must not buy a spin budget \
             (nothing can be observed — the spin would burn the whole \
             100 ms cap in front of an empty fd wait)"
        );
        assert_eq!(data.spin_probes.load(Ordering::Relaxed), 0);

        // Teardown is safe under the wedge: destroy skips the poisoned
        // count but still frees (pinned in rmw_destroy_subscription).
        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The stale-bell RE-MAP: when a bell's page is REPLACED under a wait set
/// — an OWNED creator (the native graph build's bell, or a stale-name
/// cleanup) unlinked the name and the next publisher minted a fresh inode
/// — the wait set still maps the old, never-rung page. The first frame
/// then reaches the listener (fd path — one recheck slice, never the 20 ms
/// timer) with the bell silent; `reconcile_stale_bells` sees
/// frame-without-ring and re-maps, and every later publish rings this wait
/// set again. (rmw publishers themselves open the bell UNOWNED and never
/// unlink — see the survivor pin — so this test unlinks the name by hand
/// between the two lives; without that step life 2 would simply join the
/// same page and nothing would be stale.) Linux arm: after the re-create `park_bell_reopens >= 1`
/// and the post-re-create rounds score DOORBELL wakes (a build that skips
/// the re-open scores fd wakes only on every one of them).
/// Off Linux the park tier is stubbed: the same rounds must keep waking
/// via the fd, with the re-open bookkeeping provably inert.
#[test]
#[serial]
fn a_recreated_publisher_re_rings_the_parked_wait() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    // Park FORCED on (see the park pin above).
    let _park = EnvVarGuard::set("CERULION_MONITOR_WAIT", "1");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevT{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_t_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/t/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let ws = rmw_create_wait_set(context, 8);
        let data = &*((*ws).data as *const WaitSetData);

        // One publish-wake-take round inside a fresh block.
        let round = |publisher: *mut ffi::rmw_publisher_t, x: f64| {
            let seen = blocks_entered(ws);
            let handle = wait_on_sub_in_thread((*subscription).data, ws, Duration::from_secs(5));
            await_fresh_block(ws, seen);
            let msg = CPoint { x, y: 1.5, z: -1.5 };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let (ret, ready) = handle.join().expect("wait thread");
            assert_eq!(ret, RMW_RET_OK, "x={x}: the publish must wake the wait");
            assert!(ready);
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
        };

        // Life 1: the original publisher, three rounds (Linux: the wait maps
        // its bell and parks on it).
        let first = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!first.is_null());
        for i in 0..3 {
            round(first, f64::from(i));
        }
        let reopens_before = data.park_bell_reopens.load(Ordering::Relaxed);
        let bells_before = data.doorbell_topics();

        // Destroy + re-create on the SAME topic: the old page's name is
        // unlinked, the new publisher mints a fresh inode under it.
        assert_eq!(rmw_destroy_publisher(node, first), RMW_RET_OK);
        // Model the OWNED-creator lifecycle: unlink the page's name between
        // the two lives (an owned open + drop unlinks it), so the replacement
        // publisher's create-if-absent mints a fresh inode.
        {
            let cer_topic = runtime::ros_topic_to_cerulion(&format!("/rmw_wev/t/{suffix}"))
                .expect("canonical topic name");
            // DELIBERATELY the shared production namespace (declared in
            // `serial_discipline_test`'s `SHM_NS_EXEMPTIONS`): this block
            // must unlink THE page the rmw publisher armed under
            // `default_namespace()` — a pid-scoped namespace would unlink a
            // page nobody armed and the stale-bell pin below would never
            // fire. Cross-process page uniqueness rides the topic instead
            // (`unique_suffix()` embeds `process::id()`).
            let shared_publisher_ns = cerulion_core::doorbell::default_namespace();
            drop(
                cerulion_core::doorbell::Doorbell::open_owned(&shared_publisher_ns, &cer_topic)
                    .expect("owned open"),
            );
        }
        let second = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!second.is_null());

        // Life 2: the first round can only wake via the fd (stale bell);
        // the re-open must land there, and the rest ring again.
        let doorbell_wakes_before = data.park_wakes_doorbell.load(Ordering::Relaxed);
        let fd_wakes_before = data.fd_wakes.load(Ordering::Relaxed);
        const LIFE2_ROUNDS: u32 = 6;
        for i in 0..LIFE2_ROUNDS {
            round(second, 100.0 + f64::from(i));
        }
        let reopens = data.park_bell_reopens.load(Ordering::Relaxed) - reopens_before;
        let doorbell_wakes =
            data.park_wakes_doorbell.load(Ordering::Relaxed) - doorbell_wakes_before;
        let fd_wakes = data.fd_wakes.load(Ordering::Relaxed) - fd_wakes_before;
        eprintln!(
            "re-create: bells_before={bells_before} bells_now={} reopens={reopens} \
             life2 doorbell_wakes={doorbell_wakes} fd_wakes={fd_wakes} park_blocks={}",
            data.doorbell_topics(),
            data.park_blocks.load(Ordering::Relaxed),
        );
        if cfg!(target_os = "linux") {
            assert_eq!(bells_before, 1, "life 1 must have mapped the topic's bell");
            assert!(
                reopens >= 1,
                "the stale bell must be re-mapped after the re-create (0 = the \
                 frame-without-ring evidence was not acted on)"
            );
            assert!(
                doorbell_wakes >= 1,
                "after the re-map the new publisher's rings must wake this wait \
                 set again (0 = every life-2 wake rode the fd — the stale mapping \
                 was never replaced)"
            );
            assert!(
                fd_wakes <= u64::from(LIFE2_ROUNDS),
                "never more than one fd-path wake per life-2 round"
            );
        } else {
            assert_eq!(bells_before, 0, "off Linux no bell is ever mapped");
            assert_eq!(reopens, 0, "off Linux the re-open bookkeeping is inert");
            assert_eq!(data.park_blocks.load(Ordering::Relaxed), 0);
            assert!(
                fd_wakes >= 1,
                "off Linux the re-created publisher still wakes through the fd"
            );
        }

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, second), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Wake-cost pin on a stock-executor-shaped set — ONE subscription plus
/// TWO guard conditions plus SIX services (rclcpp's parameter services):
/// per hop (a publish waking a blocked wait) the wait may DRAIN at most 3
/// wake signals (`drain_calls`, each at least one syscall) and run at most
/// 3 post-wake SHM probes (`probes_wake`), while every call's ENTRY pass
/// probes each SHM entity exactly once (`probes_entry == 7` — the one full
/// pass the contract needs; see the next test). A loop that drains
/// every listener and probes every entity twice per wake costs 9 drains at
/// entry + 9 at the wake, ~21 probes — most of the measured ping-pong RTT.
/// Mutant: restore the unconditional drain-all at the loop head ⇒ 18
/// drains per hop.
#[test]
#[serial]
fn a_nine_entity_hop_drains_and_probes_only_what_fired() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevU{suffix}"));
        let sts = service_ts(&format!("WevUsvc{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_u_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/u/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let gc1 = rmw_create_guard_condition(context);
        let gc2 = rmw_create_guard_condition(context);
        assert!(!gc1.is_null() && !gc2.is_null());
        let mut services = Vec::new();
        let mut names = Vec::new();
        for k in 0..6 {
            let name = CString::new(format!("/rmw_wev/u/svc{k}/{suffix}")).expect("name");
            let svc = rmw_create_service(node, sts, name.as_ptr(), &qos);
            assert!(!svc.is_null(), "service {k}");
            services.push(svc);
            names.push(name);
        }
        let ws = rmw_create_wait_set(context, 16);
        let data = &*((*ws).data as *const WaitSetData);
        let guard_datas = vec![(*gc1).data as usize, (*gc2).data as usize];
        let service_datas: Vec<usize> = services.iter().map(|&s| (*s).data as usize).collect();

        const HOPS: u32 = 5;
        let drains0 = data.drain_calls.load(Ordering::Relaxed);
        let pe0 = data.probes_entry.load(Ordering::Relaxed);
        let pw0 = data.probes_wake.load(Ordering::Relaxed);
        for hop in 0..HOPS {
            let seen = blocks_entered(ws);
            let handle = wait_on_full_set_in_thread(
                (*subscription).data,
                guard_datas.clone(),
                service_datas.clone(),
                ws,
                Duration::from_secs(5),
            );
            await_fresh_block(ws, seen);
            let msg = CPoint {
                x: f64::from(hop),
                y: 9.0,
                z: -9.0,
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let (ret, ready) = handle.join().expect("wait thread");
            assert_eq!(ret, RMW_RET_OK, "hop {hop}");
            assert!(
                ready,
                "hop {hop}: the subscription must be the ready entity"
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
        let drains = data.drain_calls.load(Ordering::Relaxed) - drains0;
        let probes_entry = data.probes_entry.load(Ordering::Relaxed) - pe0;
        let probes_wake = data.probes_wake.load(Ordering::Relaxed) - pw0;
        eprintln!(
            "nine-entity: hops={HOPS} drains={drains} probes_entry={probes_entry} \
             probes_wake={probes_wake} park_blocks={} fd_blocks={}",
            data.park_blocks.load(Ordering::Relaxed),
            data.fd_blocks.load(Ordering::Relaxed)
        );
        assert!(
            drains <= 3 * u64::from(HOPS),
            "at most 3 drains per hop on a 9-entity set (got {drains} over {HOPS} hops — \
             a drain-all loop scores 18 per hop)"
        );
        assert!(
            probes_wake <= 3 * u64::from(HOPS),
            "at most 3 post-wake probes per hop (got {probes_wake} over {HOPS} hops)"
        );
        assert_eq!(
            probes_entry,
            7 * u64::from(HOPS),
            "each call's entry pass probes every SHM entity exactly once (1 sub + 6 services)"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        for svc in services {
            assert_eq!(rmw_destroy_service(node, svc), RMW_RET_OK);
        }
        assert_eq!(rmw_destroy_guard_condition(gc1), RMW_RET_OK);
        assert_eq!(rmw_destroy_guard_condition(gc2), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The rmw contract the ENTRY full probe exists for: rclcpp takes ONE
/// message per ready subscription per spin, so a SECOND queued sample has
/// had its notification drained already and no fd will ever fire for it.
/// Publish twice, wait, take ONE, wait again: the second wait must return
/// READY at once — served by the entry probe, with ZERO blocks entered —
/// and the second take must deliver the second payload. A fired-only
/// entry pass would strand that sample until the next publish.
#[test]
#[serial]
fn a_second_queued_sample_is_ready_at_the_next_wait_without_a_wake() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevV{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_v_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/v/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let ws = rmw_create_wait_set(context, 8);

        let first = CPoint {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let second = CPoint {
            x: 4.0,
            y: 5.0,
            z: 6.0,
        };
        for m in [&first, &second] {
            assert_eq!(
                rmw_publish(
                    publisher,
                    m as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
        }
        // Wait 1: ready (either the entry probe or a wake), take ONE.
        let (ret, ready) = wait_on_sub((*subscription).data, ws, Duration::from_secs(2));
        assert_eq!(ret, RMW_RET_OK);
        assert!(ready);
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
        assert_eq!(out, first, "FIFO: the first sample first");

        // Wait 2: the second sample has NO notification left (both were
        // drained at wait 1) — only the entry full probe can find it.
        let seen = blocks_entered(ws);
        let (ret, ready) = wait_on_sub((*subscription).data, ws, Duration::from_secs(2));
        assert_eq!(
            ret, RMW_RET_OK,
            "a queued second sample must make the next wait READY (fired-only entry \
             probing would strand it until the next publish)"
        );
        assert!(ready);
        assert_eq!(
            blocks_entered(ws),
            seen,
            "…served by the entry probe: no block was entered"
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
        assert_eq!(out, second);

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Bell ownership: a ROS topic is
/// provisioned at TWO publishers (the `/rosout` shape), so a bell that any
/// one publisher OWNED and unlinked on drop would vanish under the
/// survivor — it keeps ringing its old inode while a wait set created
/// afterwards maps a fresh page and never hears it. rmw publishers
/// therefore open the bell UNOWNED (created if absent, never unlinked).
/// Pin: two publishers on one topic, destroy the FIRST, then a NEW
/// subscription + wait set, publishes from the survivor: under the forced
/// park (Linux) at least one wake rides the doorbell, and the fd tier is
/// entered only for a wait the park did not serve; off Linux the same
/// waits wake through the fd. Mutant: restore the owned open ⇒ the new
/// wait set scores fd wakes only, so `park_wakes_doorbell` reads 0 and the
/// arm fails (a Linux-only kill: the park is stubbed off Linux).
///
/// Why the fd count is BOUNDED rather than zero: a FORCED park on x86 runs
/// the `FirstRungOnce` shape, which parks once for the ladder's first rung
/// (a few hundred µs) and hands the rest of the idle to the fd tier. A
/// cross-thread publish that lands after that rung therefore wakes an fd
/// block, legitimately and with the product behaving correctly, so a
/// `fd_blocks == 0` claim is a claim about the machine's scheduling rather
/// than about bell ownership. Bounding the fd blocks by the number of
/// waits the doorbell did NOT serve keeps the tier claim exactly as strong
/// where it is decidable. Residual, stated in the source: one page per
/// topic can outlive every publisher on the machine until the next
/// creator.
#[test]
#[serial]
fn two_publishers_one_topic_the_survivor_still_rings_a_new_wait_set() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    let _park = EnvVarGuard::set("CERULION_MONITOR_WAIT", "1");
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("WevW{suffix}"));
        let (context, node, _opts) = setup_node(&format!("wev_w_{suffix}"));
        let topic = CString::new(format!("/rmw_wev/w/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let first = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!first.is_null());
        let second = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(
            !second.is_null(),
            "a ROS topic is provisioned at two publishers (the /rosout shape)"
        );
        // The first publisher dies. An OWNED bell would unlink the name here.
        assert_eq!(rmw_destroy_publisher(node, first), RMW_RET_OK);

        // Only NOW the consumer: a LATER subscription + wait set.
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let ws = rmw_create_wait_set(context, 8);
        let data = &*((*ws).data as *const WaitSetData);

        const WAITS: u32 = 6;
        for round in 0..WAITS {
            let seen = blocks_entered(ws);
            let handle = wait_on_sub_in_thread((*subscription).data, ws, Duration::from_secs(5));
            await_fresh_block(ws, seen);
            let msg = CPoint {
                x: 200.0 + f64::from(round),
                y: 2.5,
                z: -2.5,
            };
            assert_eq!(
                rmw_publish(
                    second,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let (ret, ready) = handle.join().expect("wait thread");
            assert_eq!(
                ret, RMW_RET_OK,
                "round {round}: the survivor's publish must wake the wait"
            );
            assert!(ready);
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

        let park_wakes = data.park_wakes_doorbell.load(Ordering::Relaxed);
        let fd_wakes = data.fd_wakes.load(Ordering::Relaxed);
        eprintln!(
            "survivor: doorbell_topics={} park_blocks={} park_wakes_doorbell={park_wakes} \
             fd_blocks={} fd_wakes={fd_wakes}",
            data.doorbell_topics(),
            data.park_blocks.load(Ordering::Relaxed),
            data.fd_blocks.load(Ordering::Relaxed),
        );
        if cfg!(target_os = "linux") {
            assert_eq!(data.doorbell_topics(), 1);
            assert!(data.park_blocks.load(Ordering::Relaxed) >= 1);
            assert!(
                park_wakes >= 1,
                "the survivor's rings must reach a wait set created AFTER the first \
                 publisher died (0 = the dead publisher unlinked the page under it)"
            );
            let fd_blocks = data.fd_blocks.load(Ordering::Relaxed);
            let unserved = u64::from(WAITS).saturating_sub(park_wakes);
            assert!(
                fd_blocks <= unserved,
                "a parked wait set falls back to the fd block only for a wait the \
                 doorbell did not serve: fd_blocks={fd_blocks} against {unserved} \
                 unserved wait(s) of {WAITS}"
            );
        } else {
            assert_eq!(data.park_blocks.load(Ordering::Relaxed), 0);
            assert!(fd_wakes >= 1, "off Linux the survivor wakes the fd tier");
        }

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, second), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Lost-wake race pin: a guard trigger landing after
/// the readiness probe read the flag and BEFORE the consuming pass must
/// SURVIVE to the next call (Principle #6) — probe-and-clear is ONE
/// atomic swap, and the consuming pass never blanket-clears the flag.
/// The `test-seams` hook fires one extra trigger in exactly that window
/// (its fired-counter makes the injection attributable); the next wait
/// must return the guard ready with NO new trigger. Mutant: restore the
/// unconditional flag clear in `consume_from_mask` ⇒ the late trigger is
/// erased and the second wait times out.
#[test]
#[serial]
fn a_trigger_landing_between_probe_and_consume_is_not_lost() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
    unsafe {
        let suffix = unique_suffix();
        let (context, node, _opts) = setup_node(&format!("wev_x_{suffix}"));
        let gc = rmw_create_guard_condition(context);
        assert!(!gc.is_null());
        let ws = rmw_create_wait_set(context, 4);
        let state = (*gc).data as *const runtime::GuardConditionState;

        // Trigger once so the entry probe finds readiness; the armed seam
        // then fires the SECOND trigger between that probe and the consume.
        let fired_before = test_seams::guard_race_triggers_fired();
        let _race = test_seams::GuardRaceTriggerGuard::arm(state);
        assert_eq!(rmw_trigger_guard_condition(gc), RMW_RET_OK);
        let mut gc_ptrs = [(*gc).data];
        let mut guards = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: gc_ptrs.as_mut_ptr(),
        };
        let t = ffi::rmw_time_t { sec: 2, nsec: 0 };
        let ret = rmw_wait(
            std::ptr::null_mut(),
            &mut guards,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &t,
        );
        assert_eq!(ret, RMW_RET_OK);
        assert!(!gc_ptrs[0].is_null(), "the first trigger is served");
        assert_eq!(
            test_seams::guard_race_triggers_fired() - fired_before,
            1,
            "the seam must have fired IN the probe→consume window (else this \
             pin proves nothing)"
        );

        // The LATE trigger must still be pending: ready with NO new trigger.
        let mut gc_ptrs2 = [(*gc).data];
        let mut guards2 = ffi::rmw_guard_conditions_t {
            guard_condition_count: 1,
            guard_conditions: gc_ptrs2.as_mut_ptr(),
        };
        let t2 = ffi::rmw_time_t {
            sec: 0,
            nsec: 300_000_000,
        };
        let ret2 = rmw_wait(
            std::ptr::null_mut(),
            &mut guards2,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &t2,
        );
        assert_eq!(
            ret2, RMW_RET_OK,
            "a trigger landing between probe and consume was LOST (the consuming \
             pass blanket-cleared the flag)"
        );
        assert!(!gc_ptrs2[0].is_null());

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_guard_condition(gc), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The idle-spin bound must survive a notification-WITHOUT-readiness
/// storm (the arming rule): productive = the call returned
/// READY, not "a wake happened". Untriggered doorbell rings wake blocks
/// but deliver nothing; under a wake-counts-as-productive rule every
/// storm call would re-arm the next call's entry spin and the idle bound
/// would be defeated at the 100 ms cap. Arms: one plain empty wait spends
/// the fresh set's single idle-period spin and disarms; N ring-storm
/// calls (each WOKEN, none ready — `fd_wakes` proves the storm was real)
/// must add ZERO spin phases; a real delivery re-arms (the call after it
/// spins). Mutant: re-arm on any wake ⇒ the storm calls spin.
#[test]
#[serial]
fn a_no_readiness_notify_storm_does_not_rearm_the_idle_spin() {
    let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "100000");
    unsafe {
        let suffix = unique_suffix();
        let (context, node, _opts) = setup_node(&format!("wev_y_{suffix}"));
        let gc = rmw_create_guard_condition(context);
        assert!(!gc.is_null());
        let ws = rmw_create_wait_set(context, 4);
        let data = &*((*ws).data as *const WaitSetData);
        let state = &*((*gc).data as *const runtime::GuardConditionState);
        let gc_addr = (*gc).data as usize;
        let ws_addr = ws as usize;

        // Call 0: fresh set, empty — the ONE idle-period spin; disarms.
        let wait_once = |nanos: u64| {
            let mut gc_ptrs = [gc_addr as *mut c_void];
            let mut guards = ffi::rmw_guard_conditions_t {
                guard_condition_count: 1,
                guard_conditions: gc_ptrs.as_mut_ptr(),
            };
            let t = ffi::rmw_time_t {
                sec: 0,
                nsec: nanos,
            };
            let ret = rmw_wait(
                std::ptr::null_mut(),
                &mut guards,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                ws_addr as *mut ffi::rmw_wait_set_t,
                &t,
            );
            (ret, !gc_ptrs[0].is_null())
        };
        let (ret, _) = wait_once(60_000_000);
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT);
        assert_eq!(data.spin_phases.load(Ordering::Relaxed), 1);

        // The storm: N calls, each woken mid-block by an untriggered ring.
        let fd_wakes_before = data.fd_wakes.load(Ordering::Relaxed);
        const STORM_CALLS: u32 = 4;
        for call in 0..STORM_CALLS {
            let seen = blocks_entered(ws);
            let handle = std::thread::spawn(move || {
                let mut gc_ptrs = [gc_addr as *mut c_void];
                let mut guards = ffi::rmw_guard_conditions_t {
                    guard_condition_count: 1,
                    guard_conditions: gc_ptrs.as_mut_ptr(),
                };
                let t = ffi::rmw_time_t {
                    sec: 0,
                    nsec: 150_000_000,
                };
                // (Lexically inside the test's `unsafe` block — no nested one.)
                {
                    rmw_wait(
                        std::ptr::null_mut(),
                        &mut guards,
                        std::ptr::null_mut(),
                        std::ptr::null_mut(),
                        std::ptr::null_mut(),
                        ws_addr as *mut ffi::rmw_wait_set_t,
                        &t,
                    )
                }
            });
            await_fresh_block(ws, seen);
            state.doorbell.as_ref().expect("doorbell").ring(); // NOTHING ready
            let ret = handle.join().expect("storm wait");
            assert_eq!(ret, ffi::RMW_RET_TIMEOUT, "storm call {call}");
        }
        let storm_wakes = data.fd_wakes.load(Ordering::Relaxed) - fd_wakes_before;
        let phases = data.spin_phases.load(Ordering::Relaxed);
        eprintln!(
            "notify-storm: calls={STORM_CALLS} fd_wakes_delta={storm_wakes} spin_phases={phases}"
        );
        assert!(
            storm_wakes >= 1,
            "the storm must really have woken blocks (else this pin is vacuous)"
        );
        assert_eq!(
            phases, 1,
            "woken-but-empty calls must NOT re-arm the entry spin — a \
             notification-without-readiness storm would otherwise burn a full \
             budget per call at the 100 ms cap (got {phases} phases)"
        );

        // A real delivery re-arms: the call AFTER it spins again.
        assert_eq!(rmw_trigger_guard_condition(gc), RMW_RET_OK);
        let (ret, ready) = wait_once(500_000_000);
        assert_eq!(ret, RMW_RET_OK);
        assert!(ready);
        let (ret, _) = wait_once(60_000_000);
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT);
        assert_eq!(
            data.spin_phases.load(Ordering::Relaxed),
            2,
            "a DELIVERED call re-arms the next call's entry spin"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_guard_condition(gc), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
