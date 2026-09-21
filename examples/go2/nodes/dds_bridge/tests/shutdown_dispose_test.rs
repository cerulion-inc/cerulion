// SPDX-License-Identifier: AGPL-3.0-only
//! The DISPOSE-OBSERVABILITY pin — the decisive proof that the
//! bridge's `shutdown()` seam actually sends the DDS participant dispose (what
//! removes ghost readers, not just "the thread exits").
//!
//! # What this proves
//!
//! A bridge that exits without dropping its participant (a `pkill`/SIGTERM of
//! `cerulion graph run attach` with the drain thread detached) orphans every
//! DDS reader with NO unregistration — the rustdds participant never sends its
//! SPDP participant-dispose / SEDP endpoint-disposes, so the robot's writers
//! hold our ghost readers until (or past) the lease. This test wires up the
//! bridge's REAL DDS path (a [`BridgePump`] driven exactly as the node's
//! `ExternalSource::Blocking` closure drives it — the shared shutdown handle
//! retained BEFORE the pump is moved into the closure, then signalled from a
//! separate thread, precisely as [`DdsBridge::shutdown`] does) and asserts a
//! SECOND participant OBSERVES the dispose.
//!
//! # The oracle (never a self-compare)
//!
//! rustdds distinguishes an EXPLICIT dispose from a lease timeout at the
//! discovery seam: `process_participant_dispose` fires
//! `DomainParticipantStatusEvent::ParticipantLost { reason: LostReason::Disposed }`,
//! while a lease timeout fires `LostReason::Timeout`. Asserting the reason is
//! **Disposed** (not Timeout) proves participant A ACTIVELY sent its dispose on
//! Drop — i.e. `shutdown()` ran the drain thread's participant Drop
//! (`DomainParticipantDisc::drop` → `on_participant_shutting_down`) — rather
//! than us merely waiting out the lease. With the drain thread detached for
//! the whole process the participant is never dropped, so
//! this event would NEVER arrive (the ghost persists until lease ageout).
//!
//! Participant A goes through the production [`cerulion_go2_dds::Go2Participant`]
//! (the one-per-process slot); participant B is a RAW rustdds
//! `DomainParticipant` (no slot) subscribed to A's disposes via its
//! `status_listener`. Both run in-process on a unique DDS domain.
//!
//! # Hermetic discovery: pin BOTH participants to ONE shared multicast interface
//!
//! (The rustdds fork this workspace pins adds unicast `same_host_loopback` SPDP
//! peers, which give an independent localhost discovery path. The
//! multicast-interface pinning below is kept as the host-agnostic hermetic path:
//! it holds on stock rustdds too, and the rest of this section describes that
//! multicast-only behavior.)
//!
//! rustdds 0.13.1 has NO unicast initial-peers knob; same-host discovery is
//! otherwise multicast-based, and it CANNOT multicast over `lo` (Linux `lo` lacks
//! `IFF_MULTICAST`, so rustdds's interface filter drops it — macOS `lo0` IS
//! multicast-capable, which is why DEFAULT interfaces pass on macOS
//! but FAIL on Linux). On a multi-homed Linux host (for example `enp*` +
//! `docker0` + `tailscale0`) the default all-interfaces path lets B discover A but
//! reliably DROPS A's participant dispose (the SPDP writer sends it over an
//! interface where same-host delivery does not land).
//!
//! The test pins BOTH A (`only_networks` in its `BridgeConfig`) and B
//! (`DomainParticipantBuilder::with_only_networks`) to ONE non-loopback,
//! multicast-capable, UP interface ([`shared_discovery_iface`], enumerated the
//! same way rustdds itself selects multicast interfaces). Both then join + send
//! multicast on the SAME interface, so discovery + the dispose ride that
//! interface's KERNEL-LOCAL `IP_MULTICAST_LOOP` path — same-host delivery that
//! never traverses the NIC/switch, with no multi-homed ambiguity — hermetic on
//! any host with a NIC.
//!
//! # Best-effort SPDP dispose → bounded retry (a real regression still fails)
//!
//! SPDP is a BEST-EFFORT builtin (no heartbeat/retransmit), and rustdds 0.13.1
//! emits the participant dispose ONCE from the participant's Drop path (its
//! send-failure is swallowed: `…dispose(…).unwrap_or(())`). Measured, that
//! single datagram is genuinely lost ~30% of the time even same-host on the best
//! single interface — a discovery-settle before shutdown does NOT change it
//! (the loss is in the teardown send, not reciprocal-discovery timing). A single
//! best-effort send therefore CANNOT gate a CI run reliably.
//!
//! So the test retries the WHOLE best-effort scenario (fresh A + fresh B,
//! discover, shutdown, observe) up to [`MAX_DISPOSE_ATTEMPTS`] times and passes
//! on the FIRST attempt that observes `Disposed`. A real regression still
//! fails, WITHOUT weakening the oracle:
//!
//! - The regression (drain thread detached, participant NEVER dropped)
//!   makes every attempt's `shutdown()` return a non-`Joined` outcome — the
//!   per-attempt `assert_eq!(… Joined)` fails FAST on attempt 1.
//! - A subtler regression where the join happens but no dispose is ever sent
//!   makes EVERY attempt observe nothing → all attempts exhaust →
//!   [`MAX_DISPOSE_ATTEMPTS`] failures → the test panics. A genuine best-effort
//!   loss on all `MAX_DISPOSE_ATTEMPTS` attempts is astronomically unlikely
//!   (≈`p^N` for a measured per-attempt loss `p` ≈ 0.3–0.55), so
//!   retry-exhaustion means "never sent", not "unlucky".
//! - The oracle stays byte-identical: `LostReason::Disposed`, never `Timeout`.
//!
//! Hermetic — no robot, no external peer, no dependency on switch/LAN multicast.

use std::net::IpAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use dds_bridge::config::BridgeConfig;
use dds_bridge::pump::{BridgePump, ShutdownOutcome, DRAIN_JOIN_DEADLINE};
use dds_bridge::queue::SampleQueue;

use cerulion_go2_dds::participant::reset_participant_slot_for_test;
use cerulion_go2_dds::ros2_client::rustdds::{
    DomainParticipantBuilder, DomainParticipantStatusEvent, LostReason, StatusEvented,
};

/// Bounded window for B to discover A over same-host UDP discovery (per attempt;
/// same-host discovery converges in well under a second, so this is a generous
/// upper bound rarely approached).
const DISCOVER_TIMEOUT: Duration = Duration::from_secs(20);
/// Bounded window for B to observe A's dispose after `shutdown()`, PER ATTEMPT.
/// A delivered dispose lands in ~0.2s same-host; 3s is a wide margin that keeps a
/// LOST-dispose attempt cheap to retry (see the module doc on the bounded retry).
const DISPOSE_ATTEMPT_WINDOW: Duration = Duration::from_secs(3);
/// How many times to retry the whole best-effort dispose scenario before failing
/// (see the module doc). Measured per-attempt loss is ~30% on an idle Linux
/// host and up to ~55% under load, so even at the pessimistic 0.55 the odds of a
/// HEALTHY bridge exhausting all 15 (`0.55^15` ≈ 1e-4) are negligible, while a real
/// regression (no dispose ever) fails every one. Worst-case cost is paid ONLY on
/// a failing run, and its true upper bound is the DISCOVERY path, not the
/// dispose one: a dispose-loss regression pays 15 × [`DISPOSE_ATTEMPT_WINDOW`]
/// (≈ 45s), but a DEGRADED host that discovers A on no attempt pays
/// 15 × [`DISCOVER_TIMEOUT`] (≈ 5 minutes). A healthy run succeeds on one of the
/// first few attempts in a second or two.
const MAX_DISPOSE_ATTEMPTS: u32 = 15;

/// A unique-ish DDS domain per test run, kept in the port-safe range so stale
/// participants from an earlier run (or unrelated DDS on domain 0) do not
/// collide with this hermetic pair.
fn unique_domain() -> u16 {
    100 + (std::process::id() % 100) as u16
}

/// Pick ONE non-loopback, multicast-capable, UP interface's IPv4 address to pin
/// BOTH participants' discovery to (see the module doc). Enumerated the SAME way
/// stock rustdds selects its multicast interfaces — `pnet::datalink::interfaces`
/// (rustdds's own transitive dep) filtered by `is_up()` + `is_multicast()` +
/// non-loopback, first IPv4 — so the chosen address is guaranteed to be one
/// rustdds will actually use for multicast joins/sends. Point-to-point tunnels
/// (e.g. `tailscale0`) are excluded: they are valid multicast interfaces but
/// their same-host loopback delivery was the least reliable of the measured NICs,
/// so skipping them lowers the per-attempt best-effort loss. Called ONCE and
/// pinned on both A and B, so the two always agree on the interface.
///
/// Returns `None` only on a host with no such interface (e.g. a loopback-only
/// CI container). The caller treats `None` as an ENVIRONMENTAL precondition
/// failure and panics FAST (a multicast-capable non-P2P NIC is required):
/// same-host DDS discovery is impossible there with stock rustdds regardless
/// (loopback is not multicast-capable on Linux), so falling back to default
/// interfaces would only buy a ~5-minute discovery timeout before the same
/// failure. A desktop macOS or Linux x86 machine has a qualifying NIC.
fn shared_discovery_iface() -> Option<IpAddr> {
    pnet_datalink::interfaces()
        .into_iter()
        .filter(|iface| {
            iface.is_up()
                && iface.is_multicast()
                && !iface.is_loopback()
                && !iface.is_point_to_point()
        })
        .flat_map(|iface| iface.ips)
        .map(|ip_net| ip_net.ip())
        .find(IpAddr::is_ipv4)
}

/// Poll B's participant-status stream (bounded) for the FIRST
/// `ParticipantDiscovered` — on this unique domain the only remote B can
/// discover is A (a participant never discovers itself). Returns A's GUID
/// prefix (the key `ParticipantLost` reports).
fn wait_for_discovery(
    listener: &cerulion_go2_dds::ros2_client::rustdds::DomainParticipantStatusListener,
    timeout: Duration,
) -> Option<cerulion_go2_dds::ros2_client::rustdds::GUID> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        while let Some(ev) = listener.try_recv_status() {
            if let DomainParticipantStatusEvent::ParticipantDiscovered { dpd } = ev {
                return Some(dpd.guid);
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    None
}

/// Poll B's participant-status stream (bounded) for `ParticipantLost` of A's
/// prefix. Returns the observed `LostReason` (so the caller can assert it is
/// `Disposed`, NOT a lease `Timeout` — the anti-tautology on the dispose).
fn wait_for_participant_lost(
    listener: &cerulion_go2_dds::ros2_client::rustdds::DomainParticipantStatusListener,
    a_guid: cerulion_go2_dds::ros2_client::rustdds::GUID,
    timeout: Duration,
) -> Option<LostReason> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        while let Some(ev) = listener.try_recv_status() {
            if let DomainParticipantStatusEvent::ParticipantLost { id, reason } = ev {
                if id == a_guid.prefix {
                    return Some(reason);
                }
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    None
}

/// Outcome of one best-effort dispose scenario attempt.
enum AttemptOutcome {
    /// B observed A's EXPLICIT dispose — the decisive success (the oracle).
    Disposed,
    /// B discovered A and `shutdown()` joined, but the best-effort dispose was
    /// not observed within [`DISPOSE_ATTEMPT_WINDOW`] — a lost datagram, retry.
    DiscoveredNoDispose,
    /// B never discovered A within [`DISCOVER_TIMEOUT`] — retry (unexpected on a
    /// host with a usable multicast interface).
    NoDiscovery,
}

/// Run ONE fully self-contained best-effort dispose scenario: fresh B (raw
/// participant), fresh A (the bridge's real `BridgePump`/`Go2Participant` path
/// driven by a detached helper exactly like the node's `ExternalSource::Blocking`
/// closure), discover, then signal + JOIN the drain thread (== `DdsBridge::shutdown`)
/// and watch for the dispose. Every participant is created + torn down inside
/// this call, so attempts are isolated (each A has a fresh GUID; B never carries
/// a prior attempt's events). Asserts that hold on EVERY attempt of a healthy
/// bridge — `shutdown` JOINs, the helper survives its
/// post-shutdown no-op iterates, and a second shutdown is
/// idempotent — run here so a regression in any of them fails fast, independent
/// of the best-effort dispose.
fn run_dispose_attempt(domain: u16, iface: IpAddr) -> AttemptOutcome {
    // Clean slot regardless of prior state (previous attempt / prior test).
    reset_participant_slot_for_test();

    // ---- Participant B: a RAW rustdds participant (no one-per-process slot) --
    // watching A's disposes. Created FIRST so its status channel captures A's
    // ParticipantDiscovered. Pinned to `iface` via `with_only_networks` (mirrors
    // A's `only_networks` config below). `iface` is always a real interface — the
    // caller fast-fails on `None` (no default-interface fallback).
    let b = DomainParticipantBuilder::new(domain)
        .with_only_networks([iface])
        .build()
        .expect("raw rustdds participant B (single-interface discovery)");
    let b_listener = b.status_listener();

    // ---- Participant A: the bridge's REAL path (Go2Participant via the pump) --
    // A typed-only mapping needs no transport (BridgePump::new). `only_networks`
    // pins A to the SAME `iface` as B: the config threads to
    // `ParticipantConfig::only_networks` -> `DomainParticipantBuilder::with_only_networks`,
    // so both participants join + send multicast on that ONE interface and the
    // dispose lands over its same-host IP_MULTICAST_LOOP path (see module doc).
    let only_networks_line = format!("only_networks: [\"{iface}\"]\n");
    let cfg_yaml = format!(
        "domain_id: {domain}\n{only_networks_line}mappings:\n  - dds_topic: /twist\n    \
         ros_type: geometry_msgs/Twist\n    cerulion_topic: /twist_out\n"
    );
    let cfg = BridgeConfig::from_yaml(&cfg_yaml, "dispose test").expect("config validates");
    let queue = Arc::new(Mutex::new(SampleQueue::default()));
    let mut pump = BridgePump::new(cfg, queue);
    let stats = pump.stats();

    // Production shape: retain the shared shutdown handle BEFORE the pump is
    // moved into the (Blocking-closure-equivalent) helper thread — the ONLY
    // path the "node" keeps to signal + join the drain thread once the pump is
    // owned elsewhere. This is exactly what DdsBridge::pump_blocking_source does.
    let shutdown = pump.shutdown_handle();

    // Drive the pump on a dedicated thread, exactly like the host's detached
    // ExternalSource::Blocking helper. It KEEPS calling the closure even after
    // shutdown — post-shutdown iterates must no-op gracefully.
    let running = Arc::new(AtomicBool::new(true));
    let running_helper = Arc::clone(&running);
    let helper = std::thread::Builder::new()
        .name("blocking-helper".to_string())
        .spawn(move || {
            while running_helper.load(Ordering::Acquire) {
                // == the ExternalSource::Blocking closure: pump.run_helper_iteration().
                let _ring = pump.run_helper_iteration();
            }
        })
        .expect("spawn blocking helper");

    // ---- Precondition: B discovers A (proves A's participant is up + announcing).
    let a_guid = match wait_for_discovery(&b_listener, DISCOVER_TIMEOUT) {
        Some(g) => g,
        None => {
            running.store(false, Ordering::Release);
            let _ = helper.join();
            // Tear down A's drain thread too — without this the attempt LEAKS a
            // live participant onto the shared per-pid domain, and a later
            // attempt's `wait_for_discovery` (first-discovered trust) can latch
            // onto the stale GUID and mis-attribute disposes. Best-effort (no
            // Joined assert): this arm already retries; the happy path's
            // per-attempt assert covers the detached-thread regression class.
            let _ = shutdown.shutdown(DRAIN_JOIN_DEADLINE);
            drop(b);
            return AttemptOutcome::NoDiscovery;
        }
    };

    // ---- The seam under test: signal + join the drain thread (== DdsBridge::shutdown).
    // This holds on EVERY attempt of a healthy bridge; a detached-thread regression
    // returns a non-Joined outcome and fails here on attempt 1.
    let outcome = shutdown.shutdown(DRAIN_JOIN_DEADLINE);
    assert_eq!(
        outcome,
        ShutdownOutcome::Joined,
        "shutdown() must stop + JOIN the drain thread (participant dropped, disposes sent) \
         within the deadline; pump stats: {}",
        stats.render()
    );

    // ---- The decisive oracle: B observes A's EXPLICIT dispose (not a timeout).
    let observed = wait_for_participant_lost(&b_listener, a_guid, DISPOSE_ATTEMPT_WINDOW);

    // ---- Post-shutdown no-op: the helper kept calling the closure post-shutdown; it
    // must have no-oped gracefully (never respawned, never panicked). Stop it now.
    running.store(false, Ordering::Release);
    helper
        .join()
        .expect("the blocking helper survives post-shutdown no-op iterates (never panics)");

    // A second shutdown is idempotent (host teardown can call it twice).
    assert_eq!(
        shutdown.shutdown(DRAIN_JOIN_DEADLINE),
        ShutdownOutcome::AlreadyShutDown
    );

    drop(b);

    match observed {
        // Disposed (explicit) vs Timeout (lease ageout) is the anti-tautology: the
        // shutdown must ACTIVELY dispose, not fall back to waiting out the lease.
        // (LostReason has no PartialEq, so we match rather than assert_eq.)
        Some(LostReason::Disposed) => AttemptOutcome::Disposed,
        Some(LostReason::Timeout { lease, elapsed }) => panic!(
            "A was lost by lease TIMEOUT (lease={lease:?}, elapsed={elapsed:?}), NOT an \
             explicit dispose — shutdown() did not run the participant Drop (the dispose path \
             regressed)"
        ),
        None => AttemptOutcome::DiscoveredNoDispose,
    }
}

#[test]
fn shutdown_sends_participant_dispose_observed_by_a_second_participant() {
    // Clean slot regardless of prior state (this is the only real-DDS test in
    // the binary, so it is the sole slot toucher).
    reset_participant_slot_for_test();

    let domain = unique_domain();
    // Pin BOTH participants to ONE shared multicast interface so same-host SPDP
    // discovery + the dispose ride its kernel-local IP_MULTICAST_LOOP path.
    // Resolved ONCE and applied to A and B identically (see the module doc).
    // A host with no multicast-capable NIC is an ENVIRONMENTAL precondition
    // failure — fail FAST + LEGIBLY here rather than falling back to default
    // interfaces and eating a ~5-minute (15 × DISCOVER_TIMEOUT) discovery timeout.
    let iface = match shared_discovery_iface() {
        Some(ip) => ip,
        None => panic!(
            "PRECONDITION: no multicast-capable, non-loopback, non-point-to-point network \
             interface found on this host — same-host DDS discovery is multicast-only under \
             stock rustdds, so this dispose-observability test cannot run. This is an \
             ENVIRONMENTAL precondition, NOT a code failure (a NIC-less / loopback-only host \
             must skip it)."
        ),
    };

    // Retry the whole best-effort scenario; pass on the first observed Disposed.
    // A LostReason::Timeout inside an attempt panics immediately (a real dispose
    // regression), so the loop only ever retries a LOST best-effort datagram.
    let mut discovered_at_least_once = false;
    for attempt in 1..=MAX_DISPOSE_ATTEMPTS {
        match run_dispose_attempt(domain, iface) {
            AttemptOutcome::Disposed => {
                reset_participant_slot_for_test();
                return;
            }
            AttemptOutcome::DiscoveredNoDispose => {
                discovered_at_least_once = true;
                eprintln!(
                    "dispose attempt {attempt}/{MAX_DISPOSE_ATTEMPTS}: discovered A but \
                     best-effort dispose not observed within {DISPOSE_ATTEMPT_WINDOW:?}; retrying"
                );
            }
            AttemptOutcome::NoDiscovery => {
                eprintln!(
                    "dispose attempt {attempt}/{MAX_DISPOSE_ATTEMPTS}: B did not discover A \
                     within {DISCOVER_TIMEOUT:?}; retrying"
                );
            }
        }
    }

    reset_participant_slot_for_test();
    if discovered_at_least_once {
        panic!(
            "B never observed A's ParticipantLost across {MAX_DISPOSE_ATTEMPTS} attempts (each \
             discovered A + joined the drain thread) — the SPDP dispose was NEVER sent/received \
             (the ghost-reader regression this test pins; a mere best-effort loss on all \
             {MAX_DISPOSE_ATTEMPTS} attempts is astronomically unlikely)"
        );
    }
    // Reached ONLY with a usable interface selected (the no-iface case
    // fast-failed above), so do NOT claim "no usable multicast interface" — a
    // usable one WAS found; discovery simply never converged over it.
    panic!(
        "B never discovered A across {MAX_DISPOSE_ATTEMPTS} attempts on the selected multicast \
         interface {iface} (each attempt waited up to {DISCOVER_TIMEOUT:?}) — a usable interface \
         WAS found, but same-host SPDP discovery never converged over it. Degraded-host / \
         environmental (e.g. multicast blocked or filtered on that NIC), not a bridge regression."
    );
}
