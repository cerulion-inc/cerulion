// SPDX-License-Identifier: AGPL-3.0-only
//! The parent-side REAPER — the targeted `waitpid`, the progress
//! watchdog's kill, the outcome vocabulary, and the stale-claim sweep's production
//! liveness predicate.
//!
//! # `waitpid(pid, WNOHANG)`, never `waitpid(-1)`
//!
//! Not hypothetical hygiene. The monolith graph process already owns
//! `std::process::Child` handles for its workers, for `bagd` and for the gateway
//! (`graph_cmd.rs:2355, 5756, 9260`, each reaped via a targeted `try_wait()`), so a
//! wildcard wait here would steal their exit status or theirs would steal ours — and an
//! exit status stolen from `bagd` is a recording that reports the wrong thing about its
//! own finalisation.
//!
//! `ECHILD` is handled EXPLICITLY as "gone, status unknown", never as a hang. The
//! class is real: under a `SIG_IGN`'d `SIGCHLD`
//! (`crates/cerulion_netd/src/client.rs:718, 1642, 2078`) an auto-reaped child leaves a
//! waiter staring at a process that no longer exists.
//!
//! # A killed child must be reported by its CAUSE, not by its signal
//!
//! When the watchdog SIGKILLs a stalled child, `waitpid` afterwards reports
//! `WTERMSIG == SIGKILL` — which is true and useless: it says the child was killed
//! without saying that WE killed it, or why. The reaper therefore remembers the verdict
//! it acted on and reports [`ChildOutcome::Stalled`] / [`ChildOutcome::Backpressured`],
//! carrying the node and phase the breadcrumb held. Losing that distinction is what
//! sends an operator hunting an external SIGKILL that never happened.
//!
//! # The liveness predicate, and the one value it must refuse
//!
//! [`claimant_is_alive`] is `kill(pid, 0) != ESRCH`, and it REFUSES `pid <= 0` before
//! the syscall. `kill(0, 0)` signals the CALLER'S OWN PROCESS GROUP and reports
//! success, so a zero pid would read as "this claimant is alive" and its stale claim
//! would never be swept — defeating the stale-claim sweep by an argument value rather than by a
//! missing rule. The arm word refuses to STORE such a pid for the same reason; this
//! refuses to BELIEVE one.
//!
//! Without the stale-claim sweep, a worker dying between its claim and its reap — the ordinary
//! shape under the shipping `--peer-loss continue` — freezes every SURVIVOR's cadence
//! as `StillEncoding` forever, which converts the all-or-nothing anchor into
//! nothing-forever-SILENTLY on precisely the degraded robot an incident recorder exists
//! for.
//!
//! # WHICH pid the predicate is asked about, per slot state
//!
//! Not always the claimant. `StateArmWord::release` takes the pid of the process
//! RUNNING the teardown, so a slot in `Releasing` names the RELEASER — which in the
//! shipping shape is the PARENT reaping its child, a different process from the one
//! that claimed. Testing the claimant there would call a live teardown abandoned and
//! reclaim a slot mid-tail. The arm word applies the injected predicate to whichever
//! party each state names; this module only has to supply a predicate that is right
//! about any pid it is handed, which is why it is written as a plain function of one
//! pid rather than of a slot.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on the
//! `pub mod state_carrier;` declaration in `lib.rs`.

use super::breadcrumb::{ChildBreadcrumb, ChildPhase};
use super::hook::{CHILD_EXIT_CAPTURE_FAILED, CHILD_EXIT_OK, CHILD_EXIT_PANIC};
use super::watchdog::{ProgressReading, StallVerdict, StallWatch};

/// THE LIVENESS PREDICATE: is the process holding a claim still alive?
///
/// `kill(pid, 0)` performs the permission and existence checks without sending a
/// signal: `ESRCH` means gone, `EPERM` means alive-but-not-ours, success means alive.
///
/// A non-positive pid is refused BEFORE the syscall, because `kill(0, 0)` addresses the
/// caller's own process group and succeeds — so a zero would report a dead claimant as
/// alive and its reservation would block every future fork on this machine for the life
/// of the run.
pub fn claimant_is_alive(pid: i32) -> bool {
    if pid <= 0 {
        return false;
    }
    // SAFETY: `kill` with signal 0 sends nothing; it only performs the checks.
    if unsafe { libc::kill(pid, 0) } == 0 {
        return true;
    }
    // EPERM means the process EXISTS but is not ours to signal — alive. Anything other
    // than ESRCH is likewise not evidence of death, and treating an unknown errno as
    // "dead" would let a sweep steal a live peer's reservation.
    std::io::Error::last_os_error().raw_os_error() != Some(libc::ESRCH)
}

/// How a capture child ended, in the vocabulary the run-time failure table reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChildOutcome {
    /// Every node in the fork set encoded.
    Completed,
    /// At least one encoder refused; the rest of the fork set still encoded.
    CaptureFailed,
    /// The child panicked and left through the hook without unwinding.
    Panicked {
        /// The node the breadcrumb named.
        node_idx: u32,
        /// The field, where the encoder reported one.
        field_idx: u32,
    },
    /// The child died on a signal nobody here sent — a SIGSEGV in a user encoder, or
    /// macOS's CoreFoundation/ObjC post-fork abort, which is deliberately NOT
    /// labelled a timeout.
    Signal(i32),
    /// The watchdog killed it, and it was blocked on a FULL state ring.
    /// The remedy is the RECORDER, not the node.
    Backpressured {
        /// The node the breadcrumb named.
        node_idx: u32,
        /// How far it got.
        progress: u64,
        /// How long the counter stood still.
        stalled_for_ns: u64,
    },
    /// The watchdog killed it for a stalled progress counter in any other phase.
    Stalled {
        /// The node the breadcrumb named — this is what makes it a diagnosis.
        node_idx: u32,
        /// The phase it was in.
        phase: ChildPhase,
        /// How far it got.
        progress: u64,
        /// How long the counter stood still.
        stalled_for_ns: u64,
    },
    /// `ECHILD` — something else reaped it (a `SIG_IGN`'d `SIGCHLD`). Gone, status
    /// unknown. **Explicitly not a hang.**
    GoneStatusUnknown,
}

impl ChildOutcome {
    /// Whether this outcome should count toward the per-NODE stall escalation.
    ///
    /// `Backpressured` is excluded for the same reason it is in
    /// [`StallVerdict::blames_the_node`]: a dead recorder stalls every node equally, so
    /// escalating would name a set of nodes none of which is broken.
    pub fn blames_the_node(self) -> bool {
        matches!(self, Self::Stalled { .. } | Self::Panicked { .. })
    }
}

/// One reaper, for one live child.
///
/// Driven from the reaper THREAD on its existing cadence — never from the node thread,
/// which is what keeps the executor's only blocking primitive a `try_lock`.
#[derive(Debug)]
pub struct ChildReaper {
    pid: i32,
    watch: StallWatch,
    /// The verdict we SIGKILLed on, so the eventual `WTERMSIG == SIGKILL` is reported
    /// by its cause rather than as an unexplained signal.
    killed_for: Option<StallVerdict>,
}

impl ChildReaper {
    /// Start watching a freshly forked child.
    pub fn new(pid: i32) -> Self {
        Self {
            pid,
            watch: StallWatch::new(),
            killed_for: None,
        }
    }

    /// The child's pid.
    pub fn pid(&self) -> i32 {
        self.pid
    }

    /// The highest progress the child was observed to reach.
    pub fn progress_watermark(&self) -> u64 {
        self.watch.watermark()
    }

    /// One reaper pass. `None` means the child is still running.
    ///
    /// `elapsed_ns` is the time since the caller's PREVIOUS pass; the reaper thread owns
    /// the clock, this owns the decision.
    pub fn poll(&mut self, crumb: &ChildBreadcrumb, elapsed_ns: u64) -> Option<ChildOutcome> {
        let mut status: libc::c_int = 0;
        // SAFETY: a targeted, non-blocking wait on this reaper's own child.
        let r = unsafe { libc::waitpid(self.pid, &mut status, libc::WNOHANG) };
        if r == self.pid {
            return Some(self.classify_exit(status, crumb));
        }
        if r < 0 {
            let errno = std::io::Error::last_os_error().raw_os_error();
            if errno == Some(libc::EINTR) {
                // A signal interrupted the wait. Nothing is known yet; try again next
                // pass rather than inventing an outcome.
                return None;
            }
            // ECHILD (or anything else the wait cannot recover from): the child is not
            // ours to reap any more. Explicitly an OUTCOME, never a hang.
            return Some(ChildOutcome::GoneStatusUnknown);
        }

        // Still running: this is where the watchdog lives.
        let verdict = self.watch.observe(
            ProgressReading {
                progress: crumb.progress(),
                phase: crumb.phase(),
            },
            elapsed_ns,
        );
        if verdict.should_kill() && self.killed_for.is_none() {
            self.killed_for = Some(verdict);
            // SAFETY: signalling this reaper's own child.
            unsafe { libc::kill(self.pid, libc::SIGKILL) };
        }
        None
    }

    /// Map a reaped status onto the outcome vocabulary.
    fn classify_exit(&self, status: libc::c_int, crumb: &ChildBreadcrumb) -> ChildOutcome {
        if libc::WIFSIGNALED(status) {
            // If WE killed it, report the CAUSE. `WTERMSIG == SIGKILL` alone is true
            // and useless — it does not say who, or why.
            if let Some(verdict) = self.killed_for {
                return match verdict {
                    StallVerdict::Backpressured { stalled_for_ns } => ChildOutcome::Backpressured {
                        node_idx: crumb.node_idx(),
                        progress: crumb.progress(),
                        stalled_for_ns,
                    },
                    StallVerdict::Stalled {
                        stalled_for_ns,
                        phase,
                    } => ChildOutcome::Stalled {
                        node_idx: crumb.node_idx(),
                        phase,
                        progress: crumb.progress(),
                        stalled_for_ns,
                    },
                    // Only the terminal arms ever reach `killed_for`.
                    StallVerdict::Progressing | StallVerdict::Waiting { .. } => {
                        ChildOutcome::Signal(libc::WTERMSIG(status))
                    }
                };
            }
            return ChildOutcome::Signal(libc::WTERMSIG(status));
        }
        match libc::WEXITSTATUS(status) {
            CHILD_EXIT_OK => ChildOutcome::Completed,
            CHILD_EXIT_CAPTURE_FAILED => ChildOutcome::CaptureFailed,
            CHILD_EXIT_PANIC => ChildOutcome::Panicked {
                node_idx: crumb.node_idx(),
                field_idx: crumb.field_idx(),
            },
            // A code this build does not mint. Reported as-is rather than folded into
            // one of ours: an unrecognised code means something OTHER than our child
            // ran, and saying "completed" there would be a fabricated verdict.
            other => ChildOutcome::Signal(-other),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_liveness_predicate_refuses_the_pid_that_would_signal_our_own_group() {
        // `kill(0, 0)` addresses the CALLER'S OWN process group and succeeds, so a zero
        // reaching the syscall reads as "alive" and the stale claim is never swept.
        // Negative pids address groups too. Both are refused BEFORE the call.
        assert!(
            !claimant_is_alive(0),
            "pid 0 is the caller's own process group"
        );
        assert!(!claimant_is_alive(-1), "negative pids address groups");
        assert!(
            !claimant_is_alive(-getpid_i32()),
            "our own group, spelled negatively"
        );

        // The anti-tautology half: the predicate is not simply always false.
        assert!(
            claimant_is_alive(getpid_i32()),
            "this process is alive, so the predicate must say so"
        );
    }

    #[test]
    fn only_esrch_is_evidence_of_death_so_a_live_peer_we_cannot_signal_reads_alive() {
        // The arm nothing else reaches, and the one whose failure is silent: `kill`
        // fails with EPERM for a process that EXISTS but is not ours to signal. Reading
        // that as death would let a sweep steal a LIVE peer's reservation — freeing
        // memory another worker has already committed a fork to, which is the OOM the memory
        // gate exists to prevent, arriving through the mechanism meant to prevent it.
        //
        // pid 1 is the portable witness: it always exists (init/launchd), and it is
        // root-owned, so an ordinary test run gets EPERM while a root run gets success.
        // BOTH must read ALIVE, which is exactly the property under test — the arm is
        // therefore meaningful whichever way the suite is run.
        assert!(
            claimant_is_alive(1),
            "pid 1 always exists; a predicate that called it dead is treating a \
             non-ESRCH errno as evidence of death"
        );

        // Its complement, measured rather than asserted about a made-up pid: a child we
        // really did reap is really gone.
        let pid = unsafe { libc::fork() };
        assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
        if pid == 0 {
            unsafe { libc::_exit(0) };
        }
        let mut status: libc::c_int = 0;
        assert_eq!(unsafe { libc::waitpid(pid, &mut status, 0) }, pid);
        assert!(
            !claimant_is_alive(pid),
            "a reaped child is ESRCH, and THAT is what a stale claim looks like"
        );
    }

    #[test]
    fn a_backpressured_outcome_does_not_blame_the_node() {
        // The stall escalation exists to surface a node whose encoder cannot complete. A
        // dead recorder stalls every node equally, so counting it would escalate the
        // whole graph on the first cadence after `bagd` died.
        let bp = ChildOutcome::Backpressured {
            node_idx: 2,
            progress: 10,
            stalled_for_ns: 1,
        };
        let st = ChildOutcome::Stalled {
            node_idx: 2,
            phase: ChildPhase::Encoding,
            progress: 10,
            stalled_for_ns: 1,
        };
        assert!(!bp.blames_the_node());
        assert!(st.blames_the_node());
        assert!(ChildOutcome::Panicked {
            node_idx: 0,
            field_idx: 0
        }
        .blames_the_node());
        assert!(!ChildOutcome::Completed.blames_the_node());
        assert!(!ChildOutcome::GoneStatusUnknown.blames_the_node());
        // A signal nobody here sent is not evidence about the node's encoder either —
        // the macOS CoreFoundation abort lands here and is explicitly not a timeout.
        assert!(!ChildOutcome::Signal(libc::SIGSEGV).blames_the_node());
    }

    fn getpid_i32() -> i32 {
        // SAFETY: `getpid` takes no arguments and cannot fail.
        unsafe { libc::getpid() }
    }
}
