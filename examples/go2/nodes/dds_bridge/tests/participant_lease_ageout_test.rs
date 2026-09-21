// SPDX-License-Identifier: AGPL-3.0-only
//! The SIGKILL-AGEOUT timing pin — the decisive proof that the SHORT
//! participant lease bounds the damage from the UNCATCHABLE teardown
//! (SIGKILL / power-cut / panic), where no signal handler can send a DDS
//! dispose.
//!
//! # What this proves (the uncatchable half of teardown)
//!
//! The graceful path makes CATCHABLE signals clean: SIGINT/SIGTERM/SIGHUP route
//! through it so the participant Drop sends its SPDP dispose and
//! remotes drop our readers immediately (pinned by `shutdown_dispose_test.rs`'s
//! `LostReason::Disposed` oracle). But SIGKILL sends NO dispose — the process
//! just vanishes — so the only bound on how long our orphaned readers linger on
//! the robot's writers is the PARTICIPANT LEASE those writers learned from our
//! SPDP announce. Stock rustdds hardcodes that at `5 * SPDP_PUBLISH_PERIOD`
//! (~50 s); Cerulion participants advertise
//! [`DEFAULT_LEASE_DURATION`] (10 s) instead, so a SIGKILLed bridge's ghosts age
//! out in ~10 s, not ~50 s.
//!
//! This test SIGKILLs a real Go2 participant (built through the production
//! [`cerulion_go2_dds::participant::Go2Participant`] path, in a SUBPROCESS so the
//! kill is real and no Drop runs) and asserts a second participant B evicts it by
//! LEASE TIMEOUT with the CONFIGURED ~10 s lease, within a wall bound far below
//! the stock ~50 s.
//!
//! # The oracle (never a self-compare), two independent halves
//!
//! 1. **Advertised lease (deterministic).** On discovery, B's
//!    `ParticipantDiscovered { dpd }` carries `dpd.lease_duration` — the lease A
//!    put on the SPDP wire. Asserting it is ~10 s (not the stock ~50 s) proves
//!    the builder knob (`DomainParticipantBuilder::participant_lease_duration`,
//!    fed by `ParticipantConfig::lease_duration`) reached the wire. This half
//!    needs no timing and cannot flake.
//! 2. **Ageout by TIMEOUT, fast (behavioral).** After SIGKILL, B observes
//!    `ParticipantLost { reason: LostReason::Timeout { lease, elapsed } }` — the
//!    lease-ageout reason, NOT `Disposed` (a SIGKILLed process sends no dispose).
//!    Asserting the reason is `Timeout` with `lease` ~10 s AND the WALL time from
//!    kill to observation is well under the stock ~50 s proves the SHORT lease
//!    actually bounds the ghost window. A regression to the stock ~50 s lease
//!    fails BOTH halves (wrong advertised lease; ageout well past the bound).
//!
//! # Needs a NIC (`#[ignore]`) — wall-clock + real multi-process DDS
//!
//! This is a real two-process DDS scenario over real UDP discovery with a
//! ~10 s+ wall-clock wait, so it is `#[ignore]`d (run by hand, like the barrier
//! subprocess tests). The CI-safe pins are
//! the config-default value pins (`cerulion_go2_dds`/`cerulion_dds` unit tests)
//! and the fork's SPDP serialize-seam test. Run on a machine with a NIC:
//!
//! ```text
//! cargo test -p dds_bridge --test participant_lease_ageout_test -- --ignored --nocapture
//! ```
//!
//! # Hermetic discovery: pin BOTH participants to ONE shared multicast interface
//!
//! Same rationale as `shutdown_dispose_test`: same-host SPDP discovery is
//! multicast-based, `lo` is not multicast-capable on Linux, and a multi-homed
//! host can send/receive discovery over different interfaces. Pinning A and B to
//! ONE non-loopback, multicast-capable, UP, non-point-to-point interface makes
//! discovery ride that interface's kernel-local `IP_MULTICAST_LOOP` path —
//! hermetic on any host with a NIC. (The fork's `same_host_loopback` unicast SPDP
//! peers add an independent localhost discovery path on top; the multicast pin is
//! kept as the proven belt-and-suspenders.)

use std::net::IpAddr;
use std::process::{Child, Command};
use std::time::{Duration, Instant};

use cerulion_go2_dds::participant::{
    reset_participant_slot_for_test, Go2Participant, ParticipantConfig, DEFAULT_LEASE_DURATION,
};
use cerulion_go2_dds::ros2_client::rustdds::{
    DomainParticipantBuilder, DomainParticipantStatusEvent, DomainParticipantStatusListener,
    LostReason, StatusEvented, GUID,
};

/// Env switch: when set on the re-exec'd child process, it runs the participant-A
/// entrypoint (build a real `Go2Participant`, announce, block until SIGKILLed)
/// instead of no-oping.
const CHILD_ENV: &str = "AGEOUT_CHILD";
/// Env carrying the DDS domain the child A must join (must equal B's).
const CHILD_DOMAIN_ENV: &str = "AGEOUT_DOMAIN";
/// Env carrying the interface IP the child A must pin discovery to (== B's).
const CHILD_IFACE_ENV: &str = "AGEOUT_IFACE";
/// The name of the child-entrypoint `#[test]` the parent re-execs via `--exact`.
const CHILD_ENTRYPOINT: &str = "ageout_child_process_participant";

/// Bounded window for B to discover A over same-host UDP discovery. Same-host
/// discovery converges in well under a second; this is a generous upper bound.
const DISCOVER_TIMEOUT: Duration = Duration::from_secs(20);
/// Bounded window for B to observe A's lease-timeout eviction AFTER SIGKILL. B
/// needs up to the lease (~10 s) plus the ~2 s participant-cleanup granularity to
/// notice, so ~12 s is the real floor; 25 s is a wide margin that still sits
/// far below the stock ~50 s lease the short lease replaces.
const AGEOUT_OBSERVE_WINDOW: Duration = Duration::from_secs(25);
/// The wall bound the observed ageout must beat — comfortably below the stock
/// `5 * SPDP_PUBLISH_PERIOD` (~50 s) lease and above the real ~12 s floor.
const AGEOUT_WALL_CEILING: Duration = Duration::from_secs(20);

/// A unique-ish DDS domain per test run in the port-safe range (crib of
/// `shutdown_dispose_test::unique_domain`), so stale participants from an earlier
/// run (or unrelated DDS on domain 0) do not collide with this pair.
fn unique_domain() -> u16 {
    100 + (std::process::id() % 100) as u16
}

/// Pick ONE non-loopback, multicast-capable, UP, non-point-to-point interface's
/// IPv4 address to pin BOTH participants' discovery to. Enumerated the SAME way
/// stock rustdds selects multicast interfaces (`pnet_datalink`, rustdds's own
/// transitive dep), so the chosen address is one rustdds will actually use.
/// Verbatim crib of `shutdown_dispose_test::shared_discovery_iface`.
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

/// SIGKILL + reap the child on Drop, so a panicking parent never leaks the
/// subprocess. `std::process::Child::kill` sends SIGKILL on Unix (exactly the
/// uncatchable teardown this test exercises).
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Poll B's participant-status stream (bounded) for the FIRST
/// `ParticipantDiscovered` on this unique domain — the only remote B can find is
/// A. Returns A's GUID and the lease A ADVERTISED in SPDP (converted to
/// `std::time::Duration`), so the caller can pin the wire lease value.
fn wait_for_discovery(
    listener: &DomainParticipantStatusListener,
    timeout: Duration,
) -> Option<(GUID, Option<Duration>)> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        while let Some(ev) = listener.try_recv_status() {
            if let DomainParticipantStatusEvent::ParticipantDiscovered { dpd } = ev {
                let advertised = dpd.lease_duration.map(Duration::from);
                return Some((dpd.guid, advertised));
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    None
}

/// Poll B's participant-status stream (bounded) for `ParticipantLost` of A's
/// prefix. Returns the observed `LostReason` (so the caller asserts it is a lease
/// `Timeout`, NOT `Disposed` — the anti-tautology: a SIGKILL sends no dispose, so
/// the ONLY way A leaves is the lease ageout the short lease bounds).
fn wait_for_participant_lost(
    listener: &DomainParticipantStatusListener,
    a_guid: GUID,
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

/// Assert an observed/advertised lease is ~[`DEFAULT_LEASE_DURATION`] (10 s) —
/// tight tolerance because 10 s is `{seconds: 10, fraction: 0}` in RTPS
/// `Duration` and round-trips exactly. The point is to distinguish 10 s from the
/// stock ~50 s, so ±0.5 s is plenty and robust to any fraction rounding.
fn assert_near_default_lease(observed: Duration, what: &str) {
    let expected = DEFAULT_LEASE_DURATION;
    let low = expected.saturating_sub(Duration::from_millis(500));
    let high = expected + Duration::from_millis(500);
    assert!(
        observed >= low && observed <= high,
        "{what} = {observed:?}, expected ~{expected:?} (the SHORT lease); a value near \
         the stock ~50 s means the participant_lease_duration knob did not reach the wire"
    );
}

/// The child-process entrypoint: when re-exec'd with [`CHILD_ENV`] set, build a
/// REAL `Go2Participant` (the production path, lease = 10 s default) on the
/// domain + interface the parent passes, announce, and block until the parent
/// SIGKILLs it. When run as a normal (ignored) test WITHOUT the env — i.e. as a
/// sibling of the parent in a `--ignored` batch — it must no-op immediately.
#[test]
#[ignore = "box-only child entrypoint; driven via re-exec by the parent ageout test"]
fn ageout_child_process_participant() {
    if std::env::var(CHILD_ENV).as_deref() != Ok("1") {
        // Not the re-exec'd child — do nothing (this is the parent's own suite
        // running the ignored sibling).
        return;
    }

    let domain: u16 = std::env::var(CHILD_DOMAIN_ENV)
        .expect("child: domain env")
        .parse()
        .expect("child: domain parse");
    let iface: IpAddr = std::env::var(CHILD_IFACE_ENV)
        .expect("child: iface env")
        .parse()
        .expect("child: iface parse");

    // Fresh process => the slot is unclaimed, but reset defensively.
    reset_participant_slot_for_test();

    // Production path: ParticipantConfig::new sets the 10 s default lease; pin
    // discovery to the parent-chosen interface.
    let cfg = ParticipantConfig::new(domain, vec![iface]);
    let _participant = Go2Participant::new(&cfg).expect("child: build Go2Participant");
    eprintln!(
        "ageout child: Go2Participant up (domain={domain}, iface={iface}, lease={:?}); \
         announcing until SIGKILLed",
        cfg.lease_duration
    );

    // Block "forever" — the parent SIGKILLs us once B has discovered us. The
    // bound is a safety net so a stranded child cannot linger past the whole
    // suite. No Drop-dispose runs when we are SIGKILLed (that is the point).
    std::thread::sleep(Duration::from_secs(120));
}

#[test]
#[ignore = "box-only: real two-process DDS + ~10s wall-clock lease ageout"]
fn sigkilled_participant_ages_out_within_configured_lease() {
    // This binary's only real-DDS parent test; clean the slot defensively (the
    // child runs in its OWN process, so it never contends for this slot).
    reset_participant_slot_for_test();

    let domain = unique_domain();
    // A multicast-capable NIC is an environmental precondition (as in the
    // dispose test): fail FAST + LEGIBLY rather than eat a long timeout.
    let iface = match shared_discovery_iface() {
        Some(ip) => ip,
        None => panic!(
            "PRECONDITION: no multicast-capable, non-loopback, non-point-to-point network \
             interface found — same-host DDS discovery is multicast-based, so this ageout test \
             cannot run here. ENVIRONMENTAL, not a code failure: run it on a host with such a NIC."
        ),
    };

    // ---- Participant B: a RAW rustdds participant (no one-per-process slot) ---
    // Created FIRST so its status channel captures A's ParticipantDiscovered.
    // Pinned to `iface` (mirrors A). B's own lease is irrelevant — B stays alive
    // the whole test and evicts A using A's ADVERTISED lease.
    let b = DomainParticipantBuilder::new(domain)
        .with_only_networks([iface])
        .build()
        .expect("raw rustdds participant B (single-interface discovery)");
    let b_listener = b.status_listener();

    // ---- Participant A: the production Go2Participant, in a SUBPROCESS so the
    // SIGKILL is real (a same-process thread cannot be SIGKILLed without taking
    // the test down). Re-exec THIS test binary at the child entrypoint.
    let exe = std::env::current_exe().expect("current_exe");
    let child = Command::new(exe)
        .args([
            "--exact",
            CHILD_ENTRYPOINT,
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(CHILD_ENV, "1")
        .env(CHILD_DOMAIN_ENV, domain.to_string())
        .env(CHILD_IFACE_ENV, iface.to_string())
        .spawn()
        .expect("spawn child participant A");
    let mut guard = ChildGuard(child);

    // ---- Precondition + oracle half 1: B discovers A and reads A's ADVERTISED
    // lease off the SPDP wire.
    let (a_guid, advertised) = match wait_for_discovery(&b_listener, DISCOVER_TIMEOUT) {
        Some(found) => found,
        None => panic!(
            "B did not discover A (subprocess) within {DISCOVER_TIMEOUT:?} on interface {iface}, \
             domain {domain} — same-host SPDP discovery never converged (degraded host / \
             multicast filtered), not a lease regression."
        ),
    };
    let advertised = advertised.expect(
        "A must advertise a participant lease in SPDP (PID_PARTICIPANT_LEASE_DURATION) — a \
         missing lease means the knob did not reach the wire",
    );
    assert_near_default_lease(advertised, "A's SPDP-advertised lease");

    // ---- The uncatchable teardown: SIGKILL A. No Drop, no dispose.
    let kill_at = Instant::now();
    guard.0.kill().expect("SIGKILL child A");
    let _ = guard.0.wait(); // reap now; ChildGuard::drop is then a no-op

    // ---- Oracle half 2: B evicts A by LEASE TIMEOUT (not Disposed), fast.
    let reason = wait_for_participant_lost(&b_listener, a_guid, AGEOUT_OBSERVE_WINDOW)
        .unwrap_or_else(|| {
            panic!(
                "B never observed A's ParticipantLost within {AGEOUT_OBSERVE_WINDOW:?} after \
                 SIGKILL — the lease ageout did not fire (a stock ~50 s lease would exceed this \
                 window; that IS the regression this test guards)"
            )
        });
    let observed_wall = kill_at.elapsed();

    match reason {
        LostReason::Timeout { lease, elapsed } => {
            // The lease B timed out against is A's advertised lease: ~10 s, not
            // the stock ~50 s.
            assert_near_default_lease(Duration::from(lease), "A's lease at timeout");
            // And the whole ageout completed well under the stock ~50 s window.
            assert!(
                observed_wall < AGEOUT_WALL_CEILING,
                "A aged out {observed_wall:?} after SIGKILL (elapsed-since-contact={elapsed:?}); \
                 expected < {AGEOUT_WALL_CEILING:?} (the SHORT lease). A stock ~50 s lease \
                 would take ~5x longer."
            );
            eprintln!(
                "ageout PASS: A (lease {:?}) SIGKILLed, evicted by TIMEOUT \
                 {observed_wall:?} later (elapsed-since-contact {elapsed:?})",
                Duration::from(lease)
            );
        }
        LostReason::Disposed => panic!(
            "A was lost by explicit DISPOSE — impossible after SIGKILL (no Drop runs), so either \
             the child was not actually SIGKILLed or discovery latched onto the wrong participant"
        ),
    }

    reset_participant_slot_for_test();
    drop(guard);
}
