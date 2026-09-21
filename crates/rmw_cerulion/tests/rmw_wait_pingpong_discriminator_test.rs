// SPDX-License-Identifier: AGPL-3.0-only
//! The ping-pong DISCRIMINATOR for the event-driven `rmw_wait`: two wait
//! sets on two threads (client + server) over REAL iceoryx2, the exact
//! shape of an rclcpp ping-pong between two processes (a callback-paced
//! client, a republishing server, `rclcpp::spin` on both sides), run once
//! with the spin ON and once OFF, reporting every wait-set counter on BOTH
//! sides, a per-round WAKE-MODE tally (fd / spin / probe / after-timeout),
//! and the RTT distribution.
//!
//! Why it exists: in measurement the ladder + spin=50 put roughly half the
//! processes into a per-LIFETIME ~123 µs mode (tight; = one client spin +
//! one server spin + ~22 µs of real path) in the stock and C0 postures,
//! never under a C1 cap, while spin=0 sessions did 30 µs. The counters
//! separate "the wait is timeout-paced" (a loop bug — `timeout_wakes`
//! dominate) from "the fd fires but the woken peer runs late" (a
//! scheduling effect — `fd_wakes` dominate, latency still high). The
//! Linux-only `same_core_*` arm (`--ignored`, box-run) pins BOTH threads to
//! one CPU to reproduce the co-located pair deterministically: on a build
//! whose spin does not yield, spin=50 there costs ~100 µs over spin=0.
//!
//! The harness also creates the server's subscription and spins its
//! executor BEFORE the client attaches; the server here runs several waits
//! before the client's publisher exists and prints its wait-set state per
//! call (degraded, blocks, wakes, watched fds, rung).
//!
//! Functional asserts only in the portable arm (the numbers are evidence
//! for `--nocapture`); ⚠️ SHM singleton — `--test-threads=1`:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_wait_pingpong_discriminator_test -- --test-threads=1 --nocapture
//! # box (Linux): the same-core pin arm
//! cargo test -p rmw_cerulion --test rmw_wait_pingpong_discriminator_test -- --ignored --test-threads=1 --nocapture
//! ```

#![cfg(unix)]

use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;

// --- fixtures (cribbed from rmw_wait_event_test.rs) --------------------

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
        "rmw_disc__msg",
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

/// One `rmw_wait` on ONE subscription on the calling thread.
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

unsafe fn take_point(sub: *const ffi::rmw_subscription_t) -> Option<CPoint> {
    let mut out = CPoint::default();
    let mut taken = false;
    assert_eq!(
        rmw_take(
            sub,
            &mut out as *mut _ as *mut c_void,
            &mut taken,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
    taken.then_some(out)
}

unsafe fn publish_point(publisher: *const ffi::rmw_publisher_t, msg: &CPoint) {
    assert_eq!(
        rmw_publish(
            publisher,
            msg as *const _ as *const c_void,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
}

// --- counters ---------------------------------------------------------

#[derive(Clone, Copy, Default, Debug)]
struct Counters {
    fd_blocks: u64,
    fd_wakes: u64,
    timeout_wakes: u64,
    spin_phases: u64,
    spin_probes: u64,
    spin_wakes: u64,
    rung_resets: u64,
    backoffs: u64,
    degraded_waits: u64,
    park_blocks: u64,
    park_wakes: u64,
    park_yields: u64,
    drains: u64,
    probes_wake: u64,
}

impl Counters {
    /// Sound only while no `rmw_wait` on `ws` is mutating non-atomic state;
    /// the counters themselves are atomics, so a concurrent read is a race
    /// on VALUES only, never UB — which is what lets the tally read them
    /// while the peer thread is waiting.
    unsafe fn read(ws: *mut ffi::rmw_wait_set_t) -> Self {
        let d = &*((*ws).data as *const WaitSetData);
        let ld = |a: &AtomicU64| a.load(Ordering::Relaxed);
        Self {
            fd_blocks: ld(&d.fd_blocks),
            fd_wakes: ld(&d.fd_wakes),
            timeout_wakes: ld(&d.timeout_wakes),
            spin_phases: ld(&d.spin_phases),
            spin_probes: ld(&d.spin_probes),
            spin_wakes: ld(&d.spin_wakes),
            rung_resets: ld(&d.block_rung_resets),
            backoffs: ld(&d.block_backoffs),
            degraded_waits: ld(&d.degraded_waits),
            park_blocks: ld(&d.park_blocks),
            park_wakes: ld(&d.park_wakes_doorbell),
            park_yields: ld(&d.park_yields),
            drains: ld(&d.drain_calls),
            probes_wake: ld(&d.probes_wake),
        }
    }

    fn delta(self, before: Self) -> Self {
        Self {
            fd_blocks: self.fd_blocks - before.fd_blocks,
            fd_wakes: self.fd_wakes - before.fd_wakes,
            timeout_wakes: self.timeout_wakes - before.timeout_wakes,
            spin_phases: self.spin_phases - before.spin_phases,
            spin_probes: self.spin_probes - before.spin_probes,
            spin_wakes: self.spin_wakes - before.spin_wakes,
            rung_resets: self.rung_resets - before.rung_resets,
            backoffs: self.backoffs - before.backoffs,
            degraded_waits: self.degraded_waits - before.degraded_waits,
            park_blocks: self.park_blocks - before.park_blocks,
            park_wakes: self.park_wakes - before.park_wakes,
            park_yields: self.park_yields - before.park_yields,
            drains: self.drains - before.drains,
            probes_wake: self.probes_wake - before.probes_wake,
        }
    }
}

/// How ONE wait returned ready, read off the counter deltas around it.
#[derive(Default, Debug, Clone, Copy)]
struct WakeModes {
    /// The park block saw a topic doorbell ring (Linux park tier).
    doorbell: u64,
    /// The block returned `Fired` (the fd path did its job).
    fd: u64,
    /// The spin front observed readiness.
    spin: u64,
    /// Ready at the first probe, with no block at all (data was already
    /// there on entry).
    probe_immediate: u64,
    /// Ready at a probe AFTER at least one empty timeout and no fd wake —
    /// the "timeout-paced" mode: the wake did not ride the fd.
    after_timeout: u64,
}

impl WakeModes {
    fn classify(&mut self, d: Counters) {
        if d.park_wakes > 0 {
            self.doorbell += 1;
        } else if d.fd_wakes > 0 {
            self.fd += 1;
        } else if d.spin_wakes > 0 {
            self.spin += 1;
        } else if d.timeout_wakes > 0 {
            self.after_timeout += 1;
        } else {
            self.probe_immediate += 1;
        }
    }
}

fn percentile(sorted: &[Duration], pct: f64) -> Duration {
    if sorted.is_empty() {
        return Duration::ZERO;
    }
    let idx = ((sorted.len() - 1) as f64 * pct).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

// --- the ping-pong ----------------------------------------------------

struct PingPong {
    node: *mut ffi::rmw_node_t,
    ping_pub: *mut ffi::rmw_publisher_t,
    ping_sub: *mut ffi::rmw_subscription_t,
    pong_pub: *mut ffi::rmw_publisher_t,
    pong_sub: *mut ffi::rmw_subscription_t,
    ws_client: *mut ffi::rmw_wait_set_t,
    ws_server: *mut ffi::rmw_wait_set_t,
}

struct PhaseReport {
    label: String,
    rtt_p50: Duration,
    rtt_p90: Duration,
    rtt_p99: Duration,
    client: Counters,
    server: Counters,
    client_modes: WakeModes,
    server_modes: WakeModes,
}

impl PhaseReport {
    fn print(&self) {
        eprintln!(
            "discriminator[{}] RTT p50={:?} p90={:?} p99={:?}",
            self.label, self.rtt_p50, self.rtt_p90, self.rtt_p99
        );
        eprintln!(
            "discriminator[{}] CLIENT {:?} modes {:?}",
            self.label, self.client, self.client_modes
        );
        eprintln!(
            "discriminator[{}] SERVER {:?} modes {:?}",
            self.label, self.server, self.server_modes
        );
    }
}

/// Run `rounds` of ping-pong: the server thread waits on the ping
/// subscription, republishes on pong; the client (this thread) paces
/// `gap` (sleeping, as the harness's callback does), publishes a ping,
/// waits on the pong subscription, records the RTT. `pin_cpu` (Linux
/// only) pins BOTH threads to that CPU first.
unsafe fn run_phase(
    pp: &PingPong,
    label: &str,
    rounds: u32,
    gap: Duration,
    pin_cpu: Option<usize>,
) -> PhaseReport {
    let client_before = Counters::read(pp.ws_client);
    let server_before = Counters::read(pp.ws_server);

    let ping_sub_data = (*pp.ping_sub).data as usize;
    let ping_sub = pp.ping_sub as usize;
    let pong_pub = pp.pong_pub as usize;
    let ws_server = pp.ws_server as usize;
    let server = std::thread::spawn(move || {
        if let Some(cpu) = pin_cpu {
            assert!(pin::pin_current_thread(cpu), "server pin failed");
        }
        let mut modes = WakeModes::default();
        for round in 0..rounds {
            let ws = ws_server as *mut ffi::rmw_wait_set_t;
            let before = Counters::read(ws);
            let (ret, ready) =
                wait_on_sub(ping_sub_data as *mut c_void, ws, Duration::from_secs(10));
            assert_eq!(ret, RMW_RET_OK, "server round {round}");
            assert!(ready);
            modes.classify(Counters::read(ws).delta(before));
            let msg = take_point(ping_sub as *const ffi::rmw_subscription_t)
                .expect("server take after wake");
            publish_point(pong_pub as *const ffi::rmw_publisher_t, &msg);
        }
        modes
    });

    if let Some(cpu) = pin_cpu {
        assert!(pin::pin_current_thread(cpu), "client pin failed");
    }
    let mut rtts = Vec::with_capacity(rounds as usize);
    let mut client_modes = WakeModes::default();
    for round in 0..rounds {
        std::thread::sleep(gap);
        let msg = CPoint {
            x: f64::from(round),
            y: 0.5,
            z: -0.5,
        };
        let before = Counters::read(pp.ws_client);
        let t0 = Instant::now();
        publish_point(pp.ping_pub, &msg);
        let (ret, ready) = wait_on_sub((*pp.pong_sub).data, pp.ws_client, Duration::from_secs(10));
        let rtt = t0.elapsed();
        assert_eq!(ret, RMW_RET_OK, "client round {round}");
        assert!(ready);
        client_modes.classify(Counters::read(pp.ws_client).delta(before));
        let got = take_point(pp.pong_sub).expect("client take after wake");
        assert_eq!(got, msg, "round {round}: pong must echo the ping");
        rtts.push(rtt);
    }
    let server_modes = server.join().expect("server thread");
    rtts.sort();

    PhaseReport {
        label: label.to_string(),
        rtt_p50: percentile(&rtts, 0.50),
        rtt_p90: percentile(&rtts, 0.90),
        rtt_p99: percentile(&rtts, 0.99),
        client: Counters::read(pp.ws_client).delta(client_before),
        server: Counters::read(pp.ws_server).delta(server_before),
        client_modes,
        server_modes,
    }
}

/// Build the topology in the harness's ORDER: server side first (ping
/// subscription + pong publisher), the server's wait set driven through
/// several calls with NO matched ping publisher yet (printing its state per
/// call), THEN the client side attaches.
unsafe fn build(suffix: u64) -> PingPong {
    let ts = point_ts(&format!("Disc{suffix}"));
    let (context, node, _opts) = setup_node(&format!("disc_{suffix}"));
    let ping_topic = CString::new(format!("/rmw_disc/ping/{suffix}")).expect("topic");
    let pong_topic = CString::new(format!("/rmw_disc/pong/{suffix}")).expect("topic");
    let qos = default_qos();
    let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

    // Server side first.
    let ping_sub = rmw_create_subscription(node, ts, ping_topic.as_ptr(), &qos, &sub_opts);
    assert!(!ping_sub.is_null());
    let pong_pub = rmw_create_publisher(node, ts, pong_topic.as_ptr(), &qos, &pub_opts);
    assert!(!pong_pub.is_null());
    let ws_server = rmw_create_wait_set(context, 8);

    // The harness spins the server's executor before the client exists:
    // several waits with no matched publisher, state printed per call.
    for call in 0..5 {
        let (ret, ready) = wait_on_sub((*ping_sub).data, ws_server, Duration::from_millis(20));
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT, "pre-attach call {call}");
        assert!(!ready);
        let d = &*((*ws_server).data as *const WaitSetData);
        eprintln!(
            "discriminator[pre-attach] server call {call}: degraded_waits={} fd_blocks={} \
             fd_wakes={} timeout_wakes={} watched_fds={} rung_us={} consecutive_empty={} \
             spin_phases={}",
            d.degraded_waits.load(Ordering::Relaxed),
            d.fd_blocks.load(Ordering::Relaxed),
            d.fd_wakes.load(Ordering::Relaxed),
            d.timeout_wakes.load(Ordering::Relaxed),
            d.watched_fds(),
            d.block_rung_us(),
            d.consecutive_empty(),
            d.spin_phases.load(Ordering::Relaxed),
        );
        assert_eq!(
            d.watched_fds(),
            1,
            "the ping subscription's listener fd must be watched"
        );
        assert_eq!(d.degraded_waits.load(Ordering::Relaxed), 0);
    }

    // Client side attaches.
    let pong_sub = rmw_create_subscription(node, ts, pong_topic.as_ptr(), &qos, &sub_opts);
    assert!(!pong_sub.is_null());
    let ping_pub = rmw_create_publisher(node, ts, ping_topic.as_ptr(), &qos, &pub_opts);
    assert!(!ping_pub.is_null());
    let ws_client = rmw_create_wait_set(context, 8);

    PingPong {
        node,
        ping_pub,
        ping_sub,
        pong_pub,
        pong_sub,
        ws_client,
        ws_server,
    }
}

unsafe fn teardown(pp: PingPong) {
    assert_eq!(rmw_destroy_wait_set(pp.ws_client), RMW_RET_OK);
    assert_eq!(rmw_destroy_wait_set(pp.ws_server), RMW_RET_OK);
    assert_eq!(rmw_destroy_publisher(pp.node, pp.ping_pub), RMW_RET_OK);
    assert_eq!(rmw_destroy_publisher(pp.node, pp.pong_pub), RMW_RET_OK);
    assert_eq!(rmw_destroy_subscription(pp.node, pp.ping_sub), RMW_RET_OK);
    assert_eq!(rmw_destroy_subscription(pp.node, pp.pong_sub), RMW_RET_OK);
    assert_eq!(rmw_destroy_node(pp.node), RMW_RET_OK);
}

/// Thread → CPU pinning. Linux: `sched_setaffinity(2)` on the calling
/// thread (a hand extern — this crate carries no `libc` dependency,
/// following the `type_bridge` malloc shim precedent). Elsewhere: no
/// portable API to force two threads onto one core, so `None`.
mod pin {
    #[cfg(target_os = "linux")]
    mod sys {
        extern "C" {
            pub fn sched_setaffinity(pid: i32, cpusetsize: usize, mask: *const u8) -> i32;
            pub fn sched_getaffinity(pid: i32, cpusetsize: usize, mask: *mut u8) -> i32;
        }
        /// glibc's `cpu_set_t` is 1024 bits.
        pub const SET_BYTES: usize = 128;
    }

    /// The first CPU this process may run on (the affinity mask is what a
    /// container / CI runner grants, so the pin is always permitted).
    #[cfg(target_os = "linux")]
    pub fn first_allowed_cpu() -> Option<usize> {
        let mut mask = [0u8; sys::SET_BYTES];
        // SAFETY: `mask` is a live 128-byte buffer, the size passed.
        if unsafe { sys::sched_getaffinity(0, sys::SET_BYTES, mask.as_mut_ptr()) } != 0 {
            return None;
        }
        (0..sys::SET_BYTES * 8).find(|&i| mask[i / 8] & (1 << (i % 8)) != 0)
    }

    #[cfg(target_os = "linux")]
    pub fn pin_current_thread(cpu: usize) -> bool {
        let mut mask = [0u8; sys::SET_BYTES];
        mask[cpu / 8] |= 1 << (cpu % 8);
        // SAFETY: `mask` is a live 128-byte buffer, the size passed; pid 0
        // = the calling thread.
        unsafe { sys::sched_setaffinity(0, sys::SET_BYTES, mask.as_ptr()) == 0 }
    }

    #[cfg(not(target_os = "linux"))]
    pub fn first_allowed_cpu() -> Option<usize> {
        None
    }

    #[cfg(not(target_os = "linux"))]
    pub fn pin_current_thread(_cpu: usize) -> bool {
        false
    }
}

const ROUNDS: u32 = 400;
const GAP: Duration = Duration::from_millis(2);

/// The portable discriminator: spin ON then OFF, unpinned, counters and
/// wake modes per side. Functional asserts only — the numbers are the
/// evidence (`--nocapture`).
#[test]
#[serial]
fn ping_pong_counters_with_the_spin_on_and_off() {
    unsafe {
        let pp = build(unique_suffix());

        let on = {
            let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "50");
            run_phase(&pp, "spin=50 unpinned", ROUNDS, GAP, None)
        };
        let off = {
            let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
            run_phase(&pp, "spin=0 unpinned", ROUNDS, GAP, None)
        };
        on.print();
        off.print();

        for (label, r) in [("spin=50", &on), ("spin=0", &off)] {
            assert_eq!(r.client.degraded_waits, 0, "{label}: client degraded");
            assert_eq!(r.server.degraded_waits, 0, "{label}: server degraded");
            assert_eq!(
                r.server_modes.doorbell
                    + r.server_modes.fd
                    + r.server_modes.spin
                    + r.server_modes.probe_immediate
                    + r.server_modes.after_timeout,
                u64::from(ROUNDS),
                "{label}: every server wake classified"
            );
        }
        assert_eq!(off.client.spin_phases, 0, "spin=0 must run no spin phase");
        assert!(
            on.client.spin_phases <= u64::from(ROUNDS) * 2,
            "spin=50: at most one spin phase per call plus one per fd wake (got {})",
            on.client.spin_phases
        );

        teardown(pp);
    }
}

/// The same-core arm (Linux, box-run): both threads pinned to one CPU —
/// the co-located pair CFS wake-affinity produces in ~half the harness
/// sessions — spin ON vs OFF. A spin that does not yield keeps the
/// just-woken peer off the CPU for its whole budget on BOTH hops, so
/// spin=50 costs ~100 µs over spin=0 (the 9013dbcb9 ~123 µs mode); a
/// yielding spin hands the core over at the first probe. The pin is the
/// RATIO, both sides measured under the same load on the same core.
#[test]
#[serial]
#[ignore = "box-run (Linux): pins both threads to one CPU; see the module docs"]
fn same_core_pair_spin_on_must_not_cost_a_spin_budget_per_hop() {
    let Some(cpu) = pin::first_allowed_cpu() else {
        eprintln!("discriminator[same-core] no CPU affinity API on this platform — skipped");
        return;
    };
    let cpu = std::env::var("PIN_CORE")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(cpu);
    unsafe {
        let pp = build(unique_suffix());
        let on = {
            let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "50");
            run_phase(
                &pp,
                &format!("spin=50 pinned cpu{cpu}"),
                ROUNDS,
                GAP,
                Some(cpu),
            )
        };
        let off = {
            let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
            run_phase(
                &pp,
                &format!("spin=0 pinned cpu{cpu}"),
                ROUNDS,
                GAP,
                Some(cpu),
            )
        };
        on.print();
        off.print();
        let on_us = on.rtt_p50.as_micros() as f64;
        let off_us = off.rtt_p50.as_micros() as f64;
        assert!(
            on_us <= 2.0 * off_us + 20.0,
            "same core: spin=50 p50 {on_us}µs vs spin=0 p50 {off_us}µs — the spin is \
             starving the co-located peer (a non-yielding spin costs one budget per hop)"
        );
        teardown(pp);
    }
}

/// The park is OS-COOPERATIVE: with BOTH threads pinned to one CPU, spin
/// OFF and the park ON, the RTT must stay under 100 µs. Measured data at the
/// unbounded park (691db49dd): p50 6.997 ms — a CFS wakeup-granularity
/// wall, because a parked thread is RUNNING to the scheduler and the
/// co-located publisher got the core only on a tick. The per-slice yield
/// and the bounded horizon bring it back to the spin's ~20 µs. Linux:
/// pinned (box-run, `--ignored` like its sibling). Off Linux there is no
/// affinity API and no park: the SAME rounds run UNPINNED on the fd tier
/// against a 1 ms starvation bound — the desk-runnable half of the arm.
/// Mutant: an unbounded, non-yielding park (Linux-only kill).
#[test]
#[serial]
#[ignore = "box-run (Linux): pins both threads to one CPU; see the module docs"]
fn same_core_pair_park_on_spin_off_stays_under_100us() {
    let cpu = pin::first_allowed_cpu().map(|c| {
        std::env::var("PIN_CORE")
            .ok()
            .and_then(|v| v.parse::<usize>().ok())
            .unwrap_or(c)
    });
    unsafe {
        let pp = build(unique_suffix());
        let r = {
            let _spin = EnvVarGuard::set("CERULION_LIVE_SPIN_US", "0");
            // The park FORCED on: this arm pins the park's per-slice YIELD,
            // and the x86 default no longer parks.
            let _park = EnvVarGuard::set("CERULION_MONITOR_WAIT", "1");
            let label = match cpu {
                Some(c) => format!("spin=0 park=on pinned cpu{c}"),
                None => "spin=0 park=on unpinned (no affinity API; fd tier)".to_string(),
            };
            run_phase(&pp, &label, ROUNDS, GAP, cpu)
        };
        r.print();
        let p50_us = r.rtt_p50.as_micros() as f64;
        if cpu.is_some() {
            assert!(
                p50_us < 100.0,
                "same core, park ON, spin 0: p50 {p50_us}µs — the park is holding the \
                 core against its own publisher (an unbounded / non-yielding park \
                 reads ~7 ms here)"
            );
        } else {
            assert!(
                p50_us < 1000.0,
                "unpinned fd tier: p50 {p50_us}µs (a starvation guard, not a band)"
            );
        }
        teardown(pp);
    }
}
