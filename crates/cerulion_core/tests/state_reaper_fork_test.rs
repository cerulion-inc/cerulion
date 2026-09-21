// SPDX-License-Identifier: AGPL-3.0-only
//! The REAPER over real children, and audit **amendment 7**'s
//! `kill(pid, 0)` predicate driven through the PRODUCTION arm-word sweep.
//!
//! # What only a real child can show
//!
//! The in-module oracles cover the predicate's refusals and the escalation rule. They
//! cannot see the three things that decide whether a robot's black box survives a bad
//! encoder: that the watchdog's SIGKILL is reported BY ITS CAUSE rather than as an
//! unexplained signal 9, that `ECHILD` is an outcome rather than a hang, and that a
//! DEAD worker's claim is actually reclaimed so its peers' cadences un-freeze.
//!
//! That last one is amendment 7's whole point. Under the shipping `--peer-loss
//! continue` a worker dying between its claim and its reap freezes every SURVIVOR's
//! cadence as `StillEncoding` forever, turning the all-or-nothing anchor into
//! nothing-forever-SILENTLY. The arm word takes the liveness predicate as an argument
//! precisely so the RULE can be oracle-tested without a syscall; this file is the other
//! half — the production predicate, over a really-dead process, through the real sweep.
//!
//! # Clock-free
//!
//! `ChildReaper::poll` takes the elapsed time since the caller's previous pass, so a
//! stall is driven by handing it one large interval rather than by sleeping through
//! `STATE_STALL_TIMEOUT_NS`. No arm here asserts a wall.
//!
//! `#![cfg(unix)]`; parallel-safe (per-test arm-word tags, anonymous pages, targeted
//! reaps).

#![cfg(unix)]

use cerulion_core::state_arm::MappedStateArm;
use cerulion_core::state_carrier::{
    child_main, claimant_is_alive, resolve_fd_ceiling, ChildBreadcrumb, ChildOutcome, ChildPhase,
    ChildReaper, ChildSetup, KeepFds, MappedBreadcrumb, NodeCaptureOutcome, NodeEncoder,
    STATE_STALL_TIMEOUT_NS,
};

const CEILING: std::time::Duration = std::time::Duration::from_secs(30);

fn self_pid() -> i32 {
    unsafe { libc::getpid() }
}

/// Drive the reaper until it produces an outcome, under a liveness ceiling.
///
/// `elapsed_ns` is what each pass is TOLD; the wall only bounds the loop.
fn drive(reaper: &mut ChildReaper, crumb: &ChildBreadcrumb, elapsed_ns: u64) -> ChildOutcome {
    let deadline = std::time::Instant::now() + CEILING;
    loop {
        if let Some(outcome) = reaper.poll(crumb, elapsed_ns) {
            return outcome;
        }
        if std::time::Instant::now() >= deadline {
            unsafe { libc::kill(reaper.pid(), libc::SIGKILL) };
            let mut st: libc::c_int = 0;
            unsafe { libc::waitpid(reaper.pid(), &mut st, 0) };
            panic!("the reaper produced no outcome within {CEILING:?}");
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
}

/// An encoder that does exactly what the test's fixture needs and nothing else.
struct Fixture {
    count: u32,
    /// If set, the child blocks here forever instead of finishing this node.
    wedge_at: Option<u32>,
    /// The phase to leave stamped while wedged (amendment 8's discriminator).
    wedge_phase: ChildPhase,
    /// If set, the child panics at this node.
    panic_at: Option<u32>,
}

impl NodeEncoder for Fixture {
    fn len(&self) -> u32 {
        self.count
    }

    fn encode(&mut self, idx: u32, crumb: &ChildBreadcrumb) -> NodeCaptureOutcome {
        crumb.bump();
        if self.panic_at == Some(idx) {
            crumb.enter_field(3);
            panic!("encoder blew up on node {idx}");
        }
        if self.wedge_at == Some(idx) {
            crumb.set_phase(self.wedge_phase);
            // The wedged-encoder shape: alive, stamping nothing further.
            loop {
                unsafe { libc::pause() };
            }
        }
        NodeCaptureOutcome::Done
    }
}

fn spawn_child(crumb: &MappedBreadcrumb, fixture: Fixture) -> i32 {
    let setup = ChildSetup {
        parent_pid: self_pid(),
        keep_fds: KeepFds::none(),
        fd_ceiling: resolve_fd_ceiling().ceiling,
    };
    // SAFETY: the child runs the production `child_main` and leaves via `_exit`.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        let mut f = fixture;
        child_main(crumb, &setup, &mut f);
    }
    pid
}

#[test]
fn a_completed_child_reports_completed() {
    // The anti-tautology control for every arm below: a healthy child must NOT be
    // killed, however many reaper passes it sees, and its outcome must be Completed
    // rather than any of the failure vocabulary.
    let crumb = MappedBreadcrumb::create().expect("map");
    let pid = spawn_child(
        &crumb,
        Fixture {
            count: 2,
            wedge_at: None,
            wedge_phase: ChildPhase::Encoding,
            panic_at: None,
        },
    );
    let mut reaper = ChildReaper::new(pid);
    // Each pass is told ZERO elapsed time, so the watchdog can never trip: whatever
    // this arm reports, it is not the watchdog's doing.
    let outcome = drive(&mut reaper, &crumb, 0);
    assert_eq!(outcome, ChildOutcome::Completed);
    assert!(!outcome.blames_the_node());
}

#[test]
fn a_panicking_child_is_reported_with_the_node_and_field_the_breadcrumb_held() {
    // The containment produces an EXIT, not a signal, and the diagnosis has to come
    // from the breadcrumb because the child cannot format one.
    let crumb = MappedBreadcrumb::create().expect("map");
    let pid = spawn_child(
        &crumb,
        Fixture {
            count: 3,
            wedge_at: None,
            wedge_phase: ChildPhase::Encoding,
            panic_at: Some(1),
        },
    );
    let mut reaper = ChildReaper::new(pid);
    let outcome = drive(&mut reaper, &crumb, 0);
    assert_eq!(
        outcome,
        ChildOutcome::Panicked {
            node_idx: 1,
            field_idx: 3
        },
        "a contained panic must be reported as Panicked, naming what the child was \
         inside — not as an unexplained exit code"
    );
    assert!(outcome.blames_the_node());
}

#[test]
fn amendment_8_a_wedged_child_is_killed_and_reported_by_its_cause_not_by_signal_nine() {
    // The two conditions are driven with IDENTICAL fixtures differing only in the phase
    // the child leaves stamped, so nothing but amendment 8's discriminator can separate
    // them — and BOTH must be reported by their cause rather than as `Signal(SIGKILL)`,
    // which is what `waitpid` actually says after the watchdog acts.
    for (phase, expect_backpressure) in
        [(ChildPhase::RingFull, true), (ChildPhase::Encoding, false)]
    {
        let crumb = MappedBreadcrumb::create().expect("map");
        let pid = spawn_child(
            &crumb,
            Fixture {
                count: 2,
                wedge_at: Some(0),
                wedge_phase: phase,
                panic_at: None,
            },
        );
        let mut reaper = ChildReaper::new(pid);

        // Wait until the child has actually wedged, so the kill lands on a stalled
        // counter rather than before the child started. A CONDITION under a ceiling.
        let deadline = std::time::Instant::now() + CEILING;
        while crumb.phase() != phase {
            assert!(
                std::time::Instant::now() < deadline,
                "the child never reached the {phase:?} phase"
            );
            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        // ONE pass carrying the whole stall window: clock-free, and instant.
        let outcome = drive(&mut reaper, &crumb, STATE_STALL_TIMEOUT_NS);
        match (outcome, expect_backpressure) {
            (
                ChildOutcome::Backpressured {
                    node_idx,
                    stalled_for_ns,
                    ..
                },
                true,
            ) => {
                assert_eq!(node_idx, 0);
                assert!(stalled_for_ns >= STATE_STALL_TIMEOUT_NS);
                assert!(
                    !outcome.blames_the_node(),
                    "a dead recorder stalls every node equally; escalating names innocents"
                );
            }
            (
                ChildOutcome::Stalled {
                    node_idx,
                    phase: reported,
                    stalled_for_ns,
                    ..
                },
                false,
            ) => {
                assert_eq!(node_idx, 0);
                assert_eq!(reported, ChildPhase::Encoding);
                assert!(stalled_for_ns >= STATE_STALL_TIMEOUT_NS);
                assert!(outcome.blames_the_node());
            }
            (other, _) => panic!(
                "a child wedged in {phase:?} must be reported by its CAUSE; got {other:?} \
                 (a bare Signal(9) here means the reaper forgot that IT did the killing)"
            ),
        }
    }
}

#[test]
fn a_child_reaped_by_something_else_is_gone_status_unknown_not_a_hang() {
    // The `SIG_IGN`'d-SIGCHLD class this repo has already been bitten by
    // (cerulion_netd/src/client.rs). Reproduced by reaping the child OURSELVES first,
    // so the reaper's own `waitpid` gets ECHILD — the same errno an auto-reap produces.
    let crumb = MappedBreadcrumb::create().expect("map");
    let pid = spawn_child(
        &crumb,
        Fixture {
            count: 1,
            wedge_at: None,
            wedge_phase: ChildPhase::Encoding,
            panic_at: None,
        },
    );
    let mut status: libc::c_int = 0;
    assert_eq!(
        unsafe { libc::waitpid(pid, &mut status, 0) },
        pid,
        "the test must reap it first, so the reaper sees ECHILD"
    );

    let mut reaper = ChildReaper::new(pid);
    assert_eq!(
        reaper.poll(&crumb, 0),
        Some(ChildOutcome::GoneStatusUnknown),
        "ECHILD must be an OUTCOME — a reaper that treated it as 'still running' would \
         poll a dead pid forever and hold its claim with it"
    );
}

#[test]
fn amendment_7_a_dead_workers_claim_is_reclaimed_by_the_production_predicate() {
    // The rule is oracle-tested in `state_arm` against an INJECTED predicate. This is
    // the other half: the PRODUCTION `kill(pid, 0)` predicate, over a process that is
    // really dead, through the real sweep — which is what makes the fix non-inert.
    let tag = format!("reaper_{}_{}", self_pid(), line!());
    let arm = MappedStateArm::create_owned(&tag).expect("create arm word");

    // A real child, reaped, so its pid is genuinely gone.
    let crumb = MappedBreadcrumb::create().expect("map");
    let dead_pid = spawn_child(
        &crumb,
        Fixture {
            count: 1,
            wedge_at: None,
            wedge_phase: ChildPhase::Encoding,
            panic_at: None,
        },
    );
    let mut status: libc::c_int = 0;
    assert_eq!(unsafe { libc::waitpid(dead_pid, &mut status, 0) }, dead_pid);
    assert!(
        !claimant_is_alive(dead_pid),
        "the reaped child must read as dead, or this arm proves nothing"
    );

    // Two claims: one held by this (live) process, one by the dead child.
    let live_slot = arm
        .claim(self_pid(), 4096)
        .expect("claim a slot for ourselves");
    let dead_slot = arm
        .claim(dead_pid, 8192)
        .expect("claim a slot for the dead child");
    assert_ne!(live_slot, dead_slot);
    let (before_live, before_bytes) = arm.reservation();
    assert_eq!(before_live, 2, "both claims must be published");
    assert_eq!(before_bytes, 4096 + 8192);

    let sweep = arm.sweep_stale_claims(claimant_is_alive);

    assert_eq!(
        sweep.cleared, 1,
        "exactly the DEAD claimant's slot must be reclaimed"
    );
    assert_eq!(sweep.live, 1, "and exactly the live one must survive");
    assert_eq!(
        sweep.freed_bytes, 8192,
        "the reclaimed reservation must be given back — a ghost reservation blocks \
         every future fork on this machine"
    );
    let (after_live, after_bytes) = arm.reservation();
    assert_eq!(after_live, 1);
    assert_eq!(
        after_bytes, 4096,
        "the SURVIVOR's reservation must be untouched: a sweep that cleared both would \
         let peers fork against memory this process has committed to"
    );

    // And the survivor is genuinely still ours to release.
    //
    // The releaser pid is the process RUNNING the teardown, not the claimant: when
    // the PARENT reaps its child, the two differ by construction, and a `Releasing`
    // slot's pid names whoever is doing the tearing down — which is the party
    // `claimant_is_alive` must test for such a slot.
    assert_eq!(arm.release(live_slot, self_pid()), Some(4096));
    assert_eq!(arm.reservation(), (0, 0));
}

// ===========================================================================
// Teardown with a child STILL IN FLIGHT
// ===========================================================================

/// A carrier torn down while its capture child is still encoding must TERMINATE the
/// child and give its claim back.
///
/// `CaptureReaper::stop` used to set a flag and join. The run loop's final pass is
/// `WNOHANG`, so a child still encoding was simply not reaped — and once the thread
/// was joined, nothing would ever poll it again.
///
/// The durable damage is the CLAIM, not the orphan. A claim is taken with the
/// PARENT's pid and released only on the reap path, so an unreaped child leaks one
/// of `STATE_CLAIM_SLOTS` permanently. Amendment 7's stale sweep cannot recover it:
/// the sweep reclaims a slot whose OWNER pid is dead, and the owner here is the very
/// process that tore down and carried on. Enough teardowns with a child in flight
/// and the table is full, at which point no worker on the machine can anchor again.
///
/// Both halves are asserted, because they fail independently: a stop that killed the
/// child but skipped the release still leaks, and one that released without killing
/// leaves an orphan writing into a ring nobody owns.
#[test]
fn a_carrier_torn_down_mid_capture_kills_its_child_and_releases_its_claim() {
    use cerulion_core::state_carrier::CaptureReaper;
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    let name = format!("tdown_{}", self_pid());
    let arm = Arc::new(MappedStateArm::create_owned(&name).expect("arm word"));
    let crumb = Arc::new(MappedBreadcrumb::create().expect("breadcrumb"));
    let mut reaper = CaptureReaper::start(Arc::clone(&crumb), Arc::clone(&arm));

    // A child that will NOT finish on its own inside this test.
    // SAFETY: the child's whole body is async-signal-safe (`sleep` + `_exit`).
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork: {}", std::io::Error::last_os_error());
    if pid == 0 {
        // SAFETY: child of a just-`fork`ed process; both calls are async-signal-safe.
        unsafe {
            libc::sleep(300);
            libc::_exit(0);
        }
    }

    let slot = arm.claim(self_pid(), 0).expect("a fresh table has a slot");
    reaper.note_fork(pid, slot, 7, vec![0]);
    assert!(reaper.child_in_flight(), "the child is in flight");
    assert_eq!(arm.busy_workers(), 1, "and its claim is held");

    // THE ARM: tear the carrier down while that child is still running.
    reaper.stop();

    assert_eq!(
        arm.busy_workers(),
        0,
        "the claim must be given back — it was taken with THIS process's pid, so the \
         stale sweep can never reclaim it while this process lives, and the table is \
         finite"
    );

    // And the child must be gone. Bounded: a SIGKILLed, already-reaped child's pid
    // answers ESRCH at once; this loop only decides how long a WEDGE takes to report.
    let start = Instant::now();
    let mut gone = false;
    while start.elapsed() < CEILING {
        // SAFETY: signal 0 probes for existence and delivers nothing.
        if unsafe { libc::kill(pid, 0) } != 0 {
            gone = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    if !gone {
        // Never leave a 300-second orphan behind, whatever the verdict.
        // SAFETY: signalling the child this test forked.
        unsafe {
            libc::kill(pid, libc::SIGKILL);
            let mut st: libc::c_int = 0;
            libc::waitpid(pid, &mut st, 0);
        }
    }
    assert!(
        gone,
        "the child must be terminated and reaped by the teardown — an orphan outlives \
         its carrier and keeps writing into a ring nobody is draining"
    );
}
