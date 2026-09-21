// SPDX-License-Identifier: AGPL-3.0-only
//! Shared multi-process record-harness helpers for the `cerulion` CLI mp
//! acceptance tests. LIFTED verbatim from
//! `mp_record_e2e_test.rs` so a second test binary (the mp record→replay
//! exit-0 e2e) can drive the SAME proven recipe — hand-built tempdir workspace,
//! real-binary `graph run --record` spawn, directed SIGINT to the supervisor,
//! `ChildGuard`/`BagdGuard` leak reaping, bounded waits, per-rank manifest +
//! trace-provenance readers — without copy-paste drift.
//!
//! Test binaries can't share crate code, so each consuming binary compiles this
//! module in via `mod mp_support;` (the repo's `tests/common/mod.rs` pattern).
//! `#![allow(dead_code)]`: not every binary uses every helper (the record→replay
//! test skips the departure/killed-bagd-only helpers), so unused-in-one-binary
//! items must not trip the `-D warnings` gate.
//!
//! Prerequisites (the repo's fixture pattern — the helpers PANIC with the exact
//! instruction if missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`

#![allow(dead_code)]

use std::collections::BTreeMap;
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use cerulion_bag::BagReader;
use cerulion_core::shm_ring::ring_shm_name;
use cerulion_core::trace_ring::{
    TraceRingConsumer, TraceRingRecord, RECORD_TYPE_DEPARTURE, RECORD_TYPE_STEP_BOUNDARY,
};

/// The departure ring's header rank (`u32::MAX`) — must match
/// `graph_cmd::DEPARTURE_RING_RANK` (pinned there against `worker_ring_rank`;
/// restated here as the on-bag sentinel the acceptance checks).
pub const DEPARTURE_SENTINEL: u32 = u32::MAX;

/// The supervisor child, torn down by PROCESS GROUP, with an orphan verdict.
///
/// A guard that `kill()`s the supervisor alone is not enough — a SIGKILLed supervisor
/// cannot reap its workers, so every arm that panics mid-window (a
/// `wait_for_bag` timeout, say) would LEAK its `graph run-worker` processes onto
/// the desk: ppid 1, ~54 iceoryx2 fds each, a 50 ms ticker free-running for
/// hours, from arms that have already "finished".
///
/// The supervisor is therefore spawned into its OWN process group, and teardown
/// signals the GROUP: SIGTERM first so the production shutdown path runs and the
/// workers exit cleanly, a bounded wait, then SIGKILL to the group for anything
/// still standing.
///
/// # Why the child is PRIVATE and the constructors are named
///
/// The group kill is `kill(-pid)`, which only reaches a group if the child is
/// its own group LEADER — and that is true only when the spawn called
/// `setpgid`. With a `pub struct ChildGuard(pub Child)`, that
/// requirement would live in the `Drop` body and in prose, and any caller could
/// wrap a hand-rolled `cmd.spawn()` and get a guard whose teardown silently
/// degrades: `kill(-pid)` returns ESRCH, the error is discarded, only the
/// supervisor dies, and its workers are re-parented to init — the exact leak
/// this type exists to prevent.
///
/// The invariant is therefore in the TYPE. There is no way to build a guard
/// without saying which kind it is:
///
/// * [`ChildGuard::spawn_group_leader`] performs the `setpgid` itself, so a
///   group-killing guard can only ever wrap a real group leader.
/// * [`ChildGuard::single_process`] is for a child with no worker subtree (a
///   `cerulion replay`, a `topic echo`); its teardown kills the PID, because
///   signalling a group it does not lead would hit the runner's own group.
///
/// `Deref<Target = Child>` keeps the ordinary `id()` / `try_wait()` / `kill()`
/// calls working unchanged — the field is hidden to stop incorrect
/// CONSTRUCTION, not to hide the child.
///
/// # The orphan verdict, and why it is not taken in `Drop`
///
/// The guard also answers "did this run leak a worker?", because a per-arm
/// `for pid in &before { assert!(pid_is_gone(pid)) }` is a guarantee each caller
/// must remember and one already had not. Two things make that answer real:
///
/// * The pid set is captured while the supervisor is ALIVE. A first version
///   snapshotted it inside `Drop`, which runs AFTER the arm's own
///   `wait_bounded` has reaped the supervisor — `pgrep -P <dead pid>` is then
///   empty, the loop ran zero times, and every arm "passed" having checked
///   nothing. [`ChildGuard::wait_bounded`] is therefore a METHOD that notes the
///   workers before it waits, so an arm cannot reap behind the guard's back
///   through `Deref`, and [`ChildGuard::note_workers`] is there for an arm that
///   wants the set earlier.
/// * The verdict is RETURNED, never panicked. A destructor that panics while
///   the thread is already unwinding ABORTS the process and erases the failure
///   that mattered — and a `precreate_failure` arm, which aborts deployment
///   before spawning any worker, would have been failed by a verdict it has no
///   way to satisfy. `Drop` therefore only reports (to stderr) and
///   [`ChildGuard::orphan_verdict`] is `#[must_use]` for the arms that assert.
pub struct ChildGuard {
    child: Child,
    /// Does this child lead its own process group? Set by the constructor, so
    /// it cannot disagree with how the child was actually spawned.
    group_leader: bool,
    /// Worker pids seen while the supervisor was still ALIVE. `None` means
    /// nobody ever looked, which is NOT the same as "there were none".
    noted_workers: Option<Vec<u32>>,
    /// Has a verdict already been taken and reported?
    ///
    /// `finish()` tears down and renders; `Drop` then runs on the same guard.
    /// Without this the teardown — and, on a leaking arm, the full
    /// [`ORPHAN_GRACE`] poll — would be paid TWICE, and one run's leak would be
    /// reported twice.
    verdict_taken: bool,
    /// How many signals this guard has aimed at its OWN child's pid or pgid.
    ///
    /// The observable for the reaped-pid rule below: an arm that has already
    /// reaped its child must tear down with this still at ZERO. Without a
    /// counter the rule is only assertable by intercepting `kill(2)`, which is
    /// platform-specific — and a rule nothing portable can check is a rule that rots.
    /// Signals aimed at a LEAKED WORKER's own pid are deliberately not counted:
    /// those pids are re-read live from the process table at teardown, never
    /// stored, so they carry no reuse hazard.
    signals_sent: usize,
}

/// What became of the worker pids this guard noted while its supervisor lived.
///
/// `#[must_use]` so an arm that asks cannot then ignore the answer.
#[must_use]
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OrphanVerdict {
    /// Every noted worker is gone.
    Clean { checked: usize },
    /// Nobody noted a worker set — a single-process guard, or a run that
    /// aborted before spawning. Deliberately distinct from `Clean`: it is the
    /// absence of evidence, not evidence of absence.
    NotChecked,
    /// These pids were still alive after the group SIGKILL and the grace.
    Leaked(Vec<u32>),
}

impl OrphanVerdict {
    /// Fail the arm unless every noted worker is gone.
    ///
    /// `NotChecked` passes: a `single_process` guard and an aborted deployment
    /// both legitimately have no workers, and turning "nothing to check" into a
    /// failure is how a harness guarantee becomes a harness obstacle. An arm
    /// that needs the stronger claim asserts the COUNT at `note_workers`.
    pub fn assert_clean(self) {
        if let Self::Leaked(pids) = self {
            panic!(
                "ORPHANED worker pids {pids:?} survived teardown — they hold iceoryx2 \
                 fds and keep stepping after this arm finished"
            );
        }
    }
}

impl ChildGuard {
    /// Spawn `cmd` as its OWN process-group leader, then guard it.
    ///
    /// Use this for anything that spawns a worker subtree — a `graph run` on a
    /// `process_groups:` graph above all, where the workers are the processes
    /// that leak.
    pub fn spawn_group_leader(cmd: &mut Command) -> std::io::Result<Self> {
        // SAFETY: `pre_exec` runs between fork and exec, where only
        // async-signal-safe calls are allowed; `setpgid(0, 0)` is one. It
        // touches no memory and allocates nothing.
        unsafe {
            cmd.pre_exec(|| {
                if libc::setpgid(0, 0) == -1 {
                    return Err(std::io::Error::last_os_error());
                }
                Ok(())
            });
        }
        Ok(Self {
            child: cmd.spawn()?,
            group_leader: true,
            noted_workers: None,
            verdict_taken: false,
            signals_sent: 0,
        })
    }

    /// Guard a child that has NO worker subtree, so its teardown targets the
    /// PID and never a group.
    ///
    /// Named rather than `From<Child>` so the choice is legible at the call
    /// site: reading `single_process(child)` tells you the author asserted
    /// there is nothing below it, which is the claim a reader has to check.
    pub fn single_process(child: Child) -> Self {
        Self {
            child,
            group_leader: false,
            noted_workers: None,
            verdict_taken: false,
            signals_sent: 0,
        }
    }

    /// Record the worker pids alive RIGHT NOW, while the supervisor still is.
    ///
    /// Returns them so an arm can assert the count — which is the only way to
    /// tell "no workers leaked" from "`pgrep` found none to begin with".
    pub fn note_workers(&mut self) -> &[u32] {
        let pids = if self.group_leader {
            worker_pids_of(self.child.id())
        } else {
            Vec::new()
        };
        // NEVER CACHE AN EMPTY NOTE. A supervisor spawns its workers over the
        // following hundreds of milliseconds, and the first caller is often a
        // log-line poll one line after the spawn — so an unconditional cache
        // stored `Some([])` at t≈0 and every later site short-circuited on
        // `is_none()`, leaving the verdict permanently `NotChecked`. That is how
        // `plain_run_resim`'s guard went vacuous the moment its polls were routed
        // through the guard: the check was present, ran, and could never fail.
        //
        // Leaving it `None` while empty makes every subsequent site RE-NOTE, so
        // the set is captured as soon as the workers exist. The cost is one
        // `pgrep` per poll until then, bounded by the poll loop that ends when
        // the line it waits for appears. A `single_process` guard has no subtree
        // and stays `None` forever, which is `NotChecked` — correct, not a gap.
        if !pids.is_empty() {
            self.noted_workers = Some(pids);
        }
        self.noted_workers.as_deref().unwrap_or_default()
    }

    /// TEST SEAM: note an EXPLICIT pid set instead of discovering one.
    ///
    /// The discovery half (`worker_pids_of`'s `pgrep`) is exercised by the e2e
    /// arms, which have a real supervisor with real `run-worker` children. What
    /// no e2e arm can drive on demand is the VERDICT half — `Leaked`, the grace
    /// loop timing out, `assert_clean`'s panic — because producing a genuine
    /// orphan inside a PASSING test means deliberately leaking one. This seam
    /// lets `mp_support_verdict_test` drive that half over real processes whose
    /// liveness it controls exactly.
    pub fn note_workers_explicit(&mut self, pids: Vec<u32>) {
        self.noted_workers = Some(pids);
    }

    /// A liveness PROBE that notes the live worker set first.
    ///
    /// `try_wait` REAPS a child that has exited, so a bare probe through `Deref`
    /// is the same hazard as a bare reap: it can collect the supervisor before
    /// anything has looked at its children, leaving the verdict nothing to check.
    /// Callers that poll for early death (a log-line wait) go through here.
    ///
    /// Such a caller typically polls from t≈0, before any worker exists, which is
    /// why [`ChildGuard::note_workers`] must not cache an empty set — see the
    /// note there. This method re-notes on every call until it finds one.
    pub fn try_wait_noting(&mut self) -> std::io::Result<Option<ExitStatus>> {
        if self.noted_workers.is_none() {
            self.note_workers();
        }
        self.child.try_wait()
    }

    /// [`wait_bounded`] as a METHOD, noting the live worker set first.
    ///
    /// A free function taking `&mut *guard` let an arm reap the supervisor
    /// through `Deref` before anything had looked at its children, which is
    /// exactly how the orphan check came to be vacuous. Going through the guard
    /// means the lifecycle is the TYPE's business, not each caller's.
    pub fn wait_bounded(&mut self, timeout: Duration) -> Option<ExitStatus> {
        if self.noted_workers.is_none() {
            self.note_workers();
        }
        wait_bounded(&mut self.child, timeout)
    }

    /// Have the noted workers gone? Polls with a bounded grace.
    ///
    /// The grace is load-bearing: `Drop` SIGKILLs the group and a just-killed
    /// child stays visible to `kill(pid, 0)` until its parent reaps it, so an
    /// immediate check reports a perfectly clean teardown as a leak.
    fn render_orphan_verdict(&self) -> OrphanVerdict {
        let Some(noted) = self.noted_workers.as_ref() else {
            return OrphanVerdict::NotChecked;
        };
        if noted.is_empty() {
            return OrphanVerdict::NotChecked;
        }
        let deadline = Instant::now() + ORPHAN_GRACE;
        loop {
            let alive: Vec<u32> = noted.iter().copied().filter(|p| !pid_is_gone(*p)).collect();
            if alive.is_empty() {
                return OrphanVerdict::Clean {
                    checked: noted.len(),
                };
            }
            if Instant::now() >= deadline {
                return OrphanVerdict::Leaked(alive);
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    /// Tear the child down NOW and return the orphan verdict.
    ///
    /// For an arm that wants the verdict as an assertion rather than a stderr
    /// note. `Drop` afterwards is a no-op on an already-reaped child.
    pub fn finish(&mut self) -> OrphanVerdict {
        let verdict = self.teardown();
        self.verdict_taken = true;
        verdict
    }

    /// The verdict without tearing down — for an arm that already waited.
    pub fn orphan_verdict(&self) -> OrphanVerdict {
        self.render_orphan_verdict()
    }

    /// How many signals this guard has aimed at its own child's pid or pgid.
    ///
    /// The portable oracle for the reaped-pid rule in [`ChildGuard::teardown`]:
    /// tear down a guard whose child was already reaped and this must still be
    /// `0`. Intercepting `kill(2)` would prove the same thing and only on Linux.
    pub fn signals_sent(&self) -> usize {
        self.signals_sent
    }

    /// Signal the child's whole process group. `kill(-pgid)` — a group-leader
    /// child's pid IS the pgid.
    fn signal_group(&mut self, sig: libc::c_int) {
        self.signals_sent += 1;
        unsafe { libc::kill(-(self.child.id() as libc::pid_t), sig) };
    }

    /// Teardown, shared by `Drop` and [`ChildGuard::finish`]: group-signal a
    /// leader, pid-signal a single process.
    ///
    /// Notes the worker set first IF nobody has yet — a last chance that only
    /// helps while the supervisor is still alive, which is why the real capture
    /// point is [`ChildGuard::wait_bounded`].
    fn teardown(&mut self) -> OrphanVerdict {
        if self.noted_workers.is_none() {
            self.note_workers();
        }

        // THE RULE, stated once, for every signal this function aims at the
        // child: NEVER signal a pid or a pgid whose leader we have already
        // REAPED. Once `wait(2)` has collected the child, the kernel is free to
        // hand that pid — and a leader's pid IS its pgid — to an unrelated
        // process, so the signal would land on somebody else. On a shared machine
        // that somebody can be another user's build or test run.
        //
        // This is not a hypothetical reachable only by contrivance: `finish()`
        // and `Drop` are BOTH routinely called after `wait_bounded()` has
        // already reaped the child — this file's own docs say so — which is the
        // ordinary shape of a passing arm, not a failure path.
        //
        // The SIGKILL arm below already carried this rule (`group_leader &&
        // !exited`); the two SIGTERM arms did not, so the function contradicted
        // itself and the contradiction ran first. `try_wait()` is the test:
        // after `wait()`, `std` caches the status and returns `Ok(Some(..))`
        // for every later call (verified empirically, not assumed), so a
        // guard whose child is gone reports it here.
        let already_reaped = matches!(self.child.try_wait(), Ok(Some(_)));

        if already_reaped {
            // Nothing to signal and nothing to wait for. Fall through to the
            // verdict, which is the part that still matters: the WORKERS may
            // have outlived the supervisor, and they are swept by their own
            // freshly-read pids below.
        } else if self.group_leader {
            // TERM the GROUP: the supervisor's own handler forwards to its
            // workers and they exit through the normal path.
            self.signal_group(libc::SIGTERM);
        } else {
            // Best-effort BY DESIGN: a child that is exiting but not yet reaped
            // is the normal case here, so this must not go through the
            // asserting helpers. `ESRCH` on a still-unreaped pid is impossible
            // (a zombie is still signallable), which is exactly why the reap
            // check above — not an `ESRCH` check — is what makes this safe.
            // SAFETY: kill(2) with a valid pid + signal; no memory is touched.
            self.signals_sent += 1;
            unsafe {
                libc::kill(self.child.id() as libc::pid_t, libc::SIGTERM);
            }
        }
        let deadline = Instant::now() + TEARDOWN_TERM_GRACE;
        let mut exited = already_reaped;
        while !exited {
            match self.child.try_wait() {
                Ok(Some(_)) => {
                    exited = true;
                    break;
                }
                Ok(None) if Instant::now() < deadline => {
                    std::thread::sleep(Duration::from_millis(50));
                }
                // Bounded: a wedged supervisor must not hang the suite.
                _ => break,
            }
        }
        if !exited {
            // SAY SO. A silent expiry here means the production shutdown path
            // did not finish in `TEARDOWN_TERM_GRACE` and everything below is a
            // SIGKILL — which is worth knowing when an arm's bag or ring
            // assertions then look odd.
            eprintln!(
                "mp_support: supervisor pid {} did not exit within {:?} of SIGTERM — \
                 escalating to SIGKILL (its graceful shutdown did not complete)",
                self.child.id(),
                TEARDOWN_TERM_GRACE
            );
        }
        // Per the rule at the top of this function: the GROUP kill runs only
        // while the leader is unreaped.
        if self.group_leader && !exited {
            self.signal_group(libc::SIGKILL);
        }
        let _ = self.child.kill();
        let _ = self.child.wait();

        // THE VERDICT IS RENDERED BEFORE THE CLEANUP BELOW, and the order is the
        // whole point: sweeping the survivors first would leave every verdict
        // reading `Clean` no matter what leaked.
        let verdict = self.render_orphan_verdict();

        // Now leave the desk clean. A leaked worker is killed by its OWN pid —
        // unambiguous, and it is the case that actually matters, since a
        // supervisor can exit cleanly and still leave a worker stepping.
        if let OrphanVerdict::Leaked(pids) = &verdict {
            for pid in pids {
                send_signal(*pid, libc::SIGKILL);
            }
        }
        verdict
    }
}

impl std::ops::Deref for ChildGuard {
    type Target = Child;
    fn deref(&self) -> &Child {
        &self.child
    }
}

impl std::ops::DerefMut for ChildGuard {
    fn deref_mut(&mut self) -> &mut Child {
        &mut self.child
    }
}

/// How long teardown waits for the production shutdown path after SIGTERM.
const TEARDOWN_TERM_GRACE: Duration = Duration::from_secs(5);

/// How long a noted worker is given to disappear before it is called a leak.
///
/// A just-SIGKILLed child stays visible to `kill(pid, 0)` until its parent reaps
/// it, so a verdict taken with no grace reports a clean teardown as a leak.
const ORPHAN_GRACE: Duration = Duration::from_secs(2);

impl Drop for ChildGuard {
    fn drop(&mut self) {
        if self.verdict_taken {
            // `finish()` already tore this guard down and reported. Re-running
            // would pay the teardown and (on a leak) the whole grace a second
            // time, and report one run's leak twice.
            return;
        }
        let verdict = self.teardown();
        // REPORT, never panic. A destructor that panics while the thread is
        // already unwinding ABORTS the process and erases the failure that
        // mattered — and an arm that aborts deployment before spawning any
        // worker (`precreate_failure_aborts_deployment_before_any_spawn`) has no
        // way to satisfy a verdict at all. The teardown above has already killed
        // the survivors, so this line is the record, not the remedy; an arm that
        // wants the verdict as an ASSERTION calls
        // `finish()`/`orphan_verdict()`, which are `#[must_use]`.
        if let OrphanVerdict::Leaked(pids) = &verdict {
            eprintln!(
                "mp_support: ORPHANED worker pids {pids:?} survived the teardown of \
                 supervisor pid {} — they held iceoryx2 fds and kept stepping after the \
                 arm finished; killed now. Call `finish()` to fail the arm that leaks.",
                self.child.id()
            );
        }
    }
}

/// Every `graph run-worker` process still alive under `supervisor_pid`.
///
/// Used by the departure arms to assert the teardown left NO orphan, the way
/// `network_gateway_e2e_test` asserts `kill(pid, 0) == ESRCH` after its own.
pub fn worker_pids_of(supervisor_pid: u32) -> Vec<u32> {
    Command::new("pgrep")
        .args(["-P", &supervisor_pid.to_string(), "-f", "run-worker"])
        .output()
        .ok()
        .map(|o| {
            String::from_utf8_lossy(&o.stdout)
                .lines()
                .filter_map(|l| l.trim().parse::<u32>().ok())
                .collect()
        })
        .unwrap_or_default()
}

/// `true` when `pid` no longer exists (`kill(pid, 0)` ⇒ ESRCH).
pub fn pid_is_gone(pid: u32) -> bool {
    // The `unsafe {}` result is BOUND rather than compared inline: at the head
    // of an expression statement, `unsafe { .. } == -1` parses the block as a
    // statement and then chokes on the `==`.
    let rc = unsafe { libc::kill(pid as libc::pid_t, 0) };
    rc == -1 && std::io::Error::last_os_error().raw_os_error() == Some(libc::ESRCH)
}

/// bagd GRANDCHILD leak guard: bagd runs in its OWN process group, so killing the
/// supervisor on a mid-window panic orphans it. Armed with
/// [`BagdGuard::arm`], which snapshots the daemons ALREADY running; on Drop it
/// SIGKILLs the process group of every bagd that appeared since — never one it
/// did not start. Best-effort; init reaps the orphan.
pub struct BagdGuard {
    /// bagd pids that were ALREADY running when this guard was armed.
    ///
    /// The needle is the graph name, and two test binaries in this tree record
    /// the same `mpdemo` graph (`mp_record_e2e_test` and
    /// `credit_death_e2e_test`), so the doc claim that it "is unique to this
    /// file's graph name" was false. A guard that SIGKILLs every match would
    /// then reap a sibling binary's live daemon mid-arm if the two ever run
    /// concurrently. Snapshotting at arm time makes the guard reap only what
    /// appeared on ITS watch, which is the property the needle was supposed to
    /// provide and does not.
    pre_existing: Vec<i32>,
}

impl BagdGuard {
    /// Arm the guard, recording the daemons that are already running.
    pub fn arm() -> Self {
        Self {
            pre_existing: bagd_pids(),
        }
    }
}

/// Every bagd pid recording THIS module's graph, by cmdline needle.
///
/// A `pgrep` that cannot RUN is a harness fault, not "no daemons": swallowing it
/// would make the arm-time snapshot empty, and an empty snapshot turns the guard
/// into the indiscriminate reaper it was changed to stop being. `pgrep` exits 1
/// with no match, which is the ordinary empty answer and not an error.
fn bagd_pids() -> Vec<i32> {
    let out = Command::new("pgrep")
        .args(["-f", "bagd --out recordings/mpdemo_"])
        .output()
        .expect("run pgrep to find bagd daemons");
    match out.status.code() {
        Some(0) | Some(1) => String::from_utf8_lossy(&out.stdout)
            .lines()
            .filter_map(|l| l.trim().parse::<i32>().ok())
            .collect(),
        other => panic!(
            "pgrep failed with status {other:?} — cannot tell which bagd \
                         daemons were already running"
        ),
    }
}

impl Drop for BagdGuard {
    fn drop(&mut self) {
        for pid in bagd_pids()
            .into_iter()
            .filter(|pid| !self.pre_existing.contains(pid))
        {
            // SAFETY: killpg(2) on the grandchild's own process group
            // (pgid == pid — spawned with process_group(0)); no memory is
            // touched. ESRCH after a clean exit is the expected no-op.
            unsafe {
                libc::killpg(pid, libc::SIGKILL);
            }
        }
    }
}

/// Poll `try_wait` until the child exits or `timeout` elapses.
///
/// PRIVATE, and that is the enforcement. Reaping a guarded child through this
/// function leaves nothing for the orphan verdict to check — it was how the
/// check came to be vacuous — so every reap goes through
/// [`ChildGuard::wait_bounded`], which notes the live worker set first. A test
/// file that tries to call this one does not COMPILE, which a convention cannot
/// promise; the walk in `credit_death_e2e_test` carries the same rule as
/// belt-and-braces. A raw one-shot child is wrapped in
/// [`ChildGuard::single_process`] rather than waited on bare.
fn wait_bounded(child: &mut Child, timeout: Duration) -> Option<ExitStatus> {
    let start = Instant::now();
    loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => return Some(status),
            None if start.elapsed() > timeout => return None,
            None => std::thread::sleep(Duration::from_millis(20)),
        }
    }
}

/// Signal ONE process; returns whether the signal LANDED.
///
/// `kill(0, sig)` signals the caller's whole process group — the test runner
/// included — and `kill(-1, ..)` every process the user owns, so a pid that
/// arrived from a failed `pgrep` parse must never reach `kill(2)`: both are
/// refused outright.
///
/// The outcome is RETURNED rather than asserted, because plenty of sites
/// legitimately signal a process that may already have exited (a SIGINT to a
/// supervisor whose last worker just died; the teardown path). A caller whose
/// premise is "this process is alive" uses [`signal_live_process`], which turns a
/// miss into a named failure instead of a later "the log never said X".
pub fn send_signal(pid: u32, sig: libc::c_int) -> bool {
    assert!(
        pid > 1,
        "refusing to signal pid {pid}: 0 means the runner's own process group \
         and 1 is init — this is a lookup that FAILED, not a target"
    );
    // SAFETY: kill(2) with a valid pid + signal; no memory is touched.
    let rc = unsafe { libc::kill(pid as libc::pid_t, sig) };
    rc == 0
}

/// [`send_signal`] where the target MUST be alive — the arm's oracle depends on
/// this signal landing, so a miss is a failure with a name rather than a later
/// "the log never said X".
pub fn signal_live_process(pid: u32, sig: libc::c_int) {
    assert!(
        send_signal(pid, sig),
        "kill({pid}, {sig}) did not land: {} — the arm's oracle assumes this \
         process was alive and received it",
        std::io::Error::last_os_error()
    );
}

/// The platform cdylib filename for a crate/node name.
pub fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

/// A prebuilt fixture cdylib by crate name. PANICS with the build instruction
/// if missing (the repo's fixture pattern).
pub fn fixture_cdylib(crate_name: &str) -> PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
}

/// Hand-build the multi-process recording workspace in `root`:
/// a `[workspace]` Cargo.toml + `graphs/mpdemo.yaml` (the 3-node chain split
/// into `p0:[ticker,relay]` / `p1:[sink]`) + per-type `nodes/<t>/src/lib.rs`
/// fixture-source copies (for the metadata/staleness walkers) + prebuilt
/// fixture cdylibs under `target/debug/lib<t>.*` (no in-test cargo build).
///
/// `prefix` must be unique per test — the deployment's DATA plane runs on the
/// DEFAULT iceoryx2 namespace (shared with the supervisor's PLANNING
/// build; nothing is minted per-run),
/// so topic names must not collide across tests.
pub fn build_mp_workspace(root: &Path, prefix: &str) {
    // Default: the graph FILE basename and its internal `name:` coincide
    // (`graphs/mpdemo.yaml`, `name: mpdemo`).
    build_mp_workspace_named(root, prefix, "mpdemo", "mpdemo");
}

/// [`build_mp_workspace`] with the graph's FILE basename (`graph_file` →
/// `graphs/<graph_file>.yaml`) and its LEGACY `name:` key set INDEPENDENTLY.
///
/// `name:` is optional-and-ignored, so this helper's job
/// is to make the two DISAGREE: it builds the one shape that can tell a
/// file-stemmed identity apart from a `name:`-stemmed one, which is what
/// `mp_record_stems_the_bag_by_the_file_not_the_declared_name` drives.
pub fn build_mp_workspace_named(root: &Path, prefix: &str, graph_file: &str, internal_name: &str) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    // ticker = the period fixture; relay + sink = two node TYPES that are both
    // copies of the data-trigger forwarder fixture (input `trigger_in`,
    // output `cmd` mirroring x).
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures");
    let sources = [
        ("ticker", "test_node_macro_period_cdylib"),
        ("relay", "test_node_macro_data_trigger_cdylib"),
        ("sink", "test_node_macro_data_trigger_cdylib"),
    ];
    for (node_type, fixture) in sources {
        std::fs::create_dir_all(root.join(format!("nodes/{node_type}/src"))).unwrap();
        std::fs::copy(
            fixtures.join(fixture).join("src/lib.rs"),
            root.join(format!("nodes/{node_type}/src/lib.rs")),
        )
        .expect("copy fixture src");
        std::fs::copy(
            fixture_cdylib(fixture),
            root.join("target/debug").join(dylib_file(node_type)),
        )
        .expect("copy fixture cdylib");
    }
    // The pg graph: declaration order gives p0 rank 0, p1 rank 1. Written under
    // `graphs/<graph_file>.yaml` with the (possibly distinct) internal name.
    std::fs::write(
        root.join(format!("graphs/{graph_file}.yaml")),
        format!(
            "name: {internal_name}\n\
             prefix: {prefix}\n\
             process_groups:\n\
             \x20 p0:\n\
             \x20 - ticker\n\
             \x20 - relay\n\
             \x20 p1:\n\
             \x20 - sink\n\
             nodes:\n\
             - id: ticker\n\
             \x20 type: ticker\n\
             \x20 inputs: []\n\
             \x20 outputs:\n\
             \x20 - name: cmd\n\
             \x20\x20\x20 schema: geometry_msgs/Vector3\n\
             - id: relay\n\
             \x20 type: relay\n\
             \x20 inputs:\n\
             \x20 - name: trigger_in\n\
             \x20\x20\x20 source: ticker/cmd\n\
             \x20 outputs:\n\
             \x20 - name: cmd\n\
             \x20\x20\x20 schema: geometry_msgs/Vector3\n\
             - id: sink\n\
             \x20 type: sink\n\
             \x20 inputs:\n\
             \x20 - name: trigger_in\n\
             \x20\x20\x20 source: relay/cmd\n\
             \x20 outputs:\n\
             \x20 - name: cmd\n\
             \x20\x20\x20 schema: geometry_msgs/Vector3\n"
        ),
    )
    .unwrap();
}

/// Spawn `cerulion graph run mpdemo --record=recordings [extra...]` in `root`,
/// stdout+stderr redirected to files (readable while the child runs).
pub fn spawn_mp_record(root: &Path, extra: &[&str]) -> (ChildGuard, PathBuf, PathBuf) {
    spawn_mp_record_graph(root, "mpdemo", extra, &[])
}

/// [`spawn_mp_record`] with extra ENVIRONMENT for the spawned supervisor (and,
/// inherited, its workers) — the `CERULION_EXECUTION_MODE=free_run`
/// opt-in is an env knob, not a flag.
pub fn spawn_mp_record_with_env(
    root: &Path,
    extra: &[&str],
    envs: &[(&str, &str)],
) -> (ChildGuard, PathBuf, PathBuf) {
    spawn_mp_record_graph(root, "mpdemo", extra, envs)
}

/// [`spawn_mp_record`] with the graph the CLI is pointed at
/// (`cerulion graph run <graph_file> ...`) parameterized: the file-stem-identity arm
/// runs a graph whose FILE name differs from its legacy `name:` key.
pub fn spawn_mp_record_graph(
    root: &Path,
    graph_file: &str,
    extra: &[&str],
    envs: &[(&str, &str)],
) -> (ChildGuard, PathBuf, PathBuf) {
    let stdout_path = root.join("run.stdout");
    let stderr_path = root.join("run.stderr");
    // NO `--no-validate`: a recording made with the schema checks off is
    // refused at parse (by design — such a bag cannot be reliably
    // replay-verified). The workspace this module builds validates, so the
    // flag bought the harness nothing.
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(["graph", "run", graph_file, "--record=recordings"])
        .args(extra)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps it LOCAL-ONLY).
        .env("CERULION_NETWORK", "off")
        .env(
            "RUST_LOG",
            "cerulion=info,cerulion_cli_engine=info,cerulion_bagd=info",
        )
        // HERMETIC on the execution mode. The supervisor reads
        // `CERULION_EXECUTION_MODE` from ITS environment, which this spawn
        // forwards, so a developer running the suite with `free_run` exported
        // would flip every lockstep arm to free-run and test the wrong
        // contract. Pinned to the default
        // here; an arm that WANTS free-run overrides it through `envs`,
        // which is applied after this line.
        .env("CERULION_EXECUTION_MODE", "lockstep")
        .envs(envs.iter().copied())
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()));
    // OWN PROCESS GROUP, so teardown can signal the whole tree — performed by
    // the constructor, which is the only place `setpgid` lives.
    let guard = ChildGuard::spawn_group_leader(&mut cmd)
        .expect("spawn cerulion graph run --record (multi-process)");
    (guard, stdout_path, stderr_path)
}

/// Block until the recordings dir contains a `.mcap` or `timeout` elapses.
pub fn wait_for_bag(recordings: &Path, timeout: Duration) -> Option<PathBuf> {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if let Ok(rd) = std::fs::read_dir(recordings) {
            for e in rd.flatten() {
                let p = e.path();
                if p.extension().and_then(|x| x.to_str()) == Some("mcap") {
                    return Some(p);
                }
            }
        }
        std::thread::sleep(Duration::from_millis(30));
    }
    None
}

/// How many recorded step boundaries PER RANK count as a healthy window.
///
/// The fixed windows this replaces were 2-5 s of a 50 ms ticker (~40-100 steps)
/// and no assertion anywhere read the count. 20 is reachable in about one chunk
/// flush and still leaves the one arm that reads the SHAPE of the stream — the
/// free-run "boundary deltas are not all equal" discriminator, which needs 3
/// boundaries — a 19-delta sample.
pub const RECORDED_WINDOW_BOUNDARIES: usize = 20;

/// The bound on a wait-for-recorded-evidence poll.
///
/// Deliberately generous: it replaces a fixed window, so it must never be the
/// thing that fails on a loaded machine. Reaching it means the recording never
/// produced the evidence the assertions afterwards need, which is a real
/// failure and is reported as one.
pub const RECORDED_WINDOW_TIMEOUT: Duration = Duration::from_secs(60);

/// What a mid-run read of the bag can see: the messages and the scheduler-trace
/// records that are already DURABLE in the file.
pub struct BagSnapshot {
    pub msgs: Vec<cerulion_bag::BagMessage>,
    pub trace: Vec<TraceRingRecord>,
    /// Why the trace could not be decoded YET, if it could not. Early in a run
    /// there is legitimately nothing to read; at a deadline this is the thing
    /// worth printing, so it is carried rather than swallowed.
    pub trace_error: Option<String>,
}

impl BagSnapshot {
    /// Frames already recorded on `topic`.
    pub fn frames_on(&self, topic: &str) -> usize {
        self.msgs.iter().filter(|m| m.topic == topic).count()
    }

    /// Step-boundary records already recorded for `rank` (bagd stamps the rank
    /// into `reserved`).
    pub fn boundaries_for_rank(&self, rank: u32) -> usize {
        self.trace
            .iter()
            .filter(|r| r.reserved == rank && r.record_type == RECORD_TYPE_STEP_BOUNDARY)
            .count()
    }

    /// How many USER topics already carry at least `min` recorded frames.
    ///
    /// Reserved channels (`__cerulion/…` — the scheduler trace, the
    /// nondeterminism log) are excluded: they are recorded whether or not a
    /// single node published anything, so counting them would let a window
    /// "pass" on a graph whose data plane never moved.
    pub fn user_topics_with_frames(&self, min: usize) -> usize {
        let mut per_topic: BTreeMap<&str, usize> = BTreeMap::new();
        for m in &self.msgs {
            if !m.topic.starts_with(cerulion_bag::RESERVED_PREFIX) {
                *per_topic.entry(m.topic.as_str()).or_default() += 1;
            }
        }
        per_topic.values().filter(|n| **n >= min).count()
    }

    /// DEPARTURE records already recorded for the worker whose rank is `rank`
    /// (departure provenance is the sentinel ring; the DEAD rank rides
    /// `node_idx`).
    pub fn departures_of_rank(&self, rank: u32) -> usize {
        self.trace
            .iter()
            .filter(|r| {
                r.record_type == RECORD_TYPE_DEPARTURE
                    && r.reserved == DEPARTURE_SENTINEL
                    && r.node_idx == rank
            })
            .count()
    }
}

/// A snapshot read of a bag that bagd is STILL WRITING.
///
/// `std::fs::read` + [`BagReader::from_bytes`], deliberately NOT
/// `BagReader::open`: `open` MMAPS the file, and memmap2's soundness contract
/// excludes a file a writer is concurrently appending to — `reader.rs`'s own
/// SAFETY note says replay reads "finalized, quiescent bags; concurrent writers
/// are out of contract". A `read()` is a point-in-time COPY of a prefix, and a
/// prefix that ends mid-record is exactly what `recover_messages`'s `TornTail`
/// arm exists for.
///
/// Mid-run this sees only what bagd has FLUSHED (a chunk closes at
/// `cerulion_bagd::CHUNK_TIME_FLOOR_MS` or 4 MiB, whichever comes first), so it
/// is a conservative lower bound on what the finalized bag will carry — which is
/// the right direction for a wait: it can be late, never early.
pub fn bag_snapshot(bag: &Path) -> Option<BagSnapshot> {
    let bytes = std::fs::read(bag).ok()?;
    if bytes.is_empty() {
        return None;
    }
    let reader = BagReader::from_bytes(bytes);
    // Both halves must be the CRASH-TOLERANT ones, and that is not a detail:
    // `BagReader::scheduler_trace` (and `trace_records`) require a FINALIZED bag
    // — they walk from the footer's `summary_start`, which a bag still being
    // written does not have yet. MEASURED: using it here returned Err on every
    // poll, which a tolerant `unwrap_or_default()` turned into "no trace yet", so
    // every per-rank condition sat false until its 60 s deadline and all eight
    // `mp_record_e2e_test` arms failed on the timeout. `recover_scheduler_trace`
    // is the documented twin for exactly this case: it decodes from every
    // COMPLETE chunk of a possibly-truncated bag.
    let (msgs, _completeness) = reader.recover_messages().ok()?;
    let (trace, trace_error) = match reader.recover_scheduler_trace() {
        Ok((trace, _completeness)) => (trace, None),
        Err(e) => (Vec::new(), Some(e.to_string())),
    };
    Some(BagSnapshot {
        msgs,
        trace,
        trace_error,
    })
}

/// Wait until a mid-run read of `bag` satisfies `ready`, and return that
/// snapshot. Panics with `what` and the last counts seen if `timeout` expires.
///
/// This replaces "sleep long enough that the window is surely healthy" with
/// "wait until the evidence the assertions rest on is actually in the bag": the
/// same oracle, reached as soon as it is true, and a LOUD failure rather than a
/// thin window if the recorder never gets there.
pub fn wait_for_bag_state(
    bag: &Path,
    what: &str,
    timeout: Duration,
    ready: impl Fn(&BagSnapshot) -> bool,
) -> BagSnapshot {
    let start = Instant::now();
    let mut last: Option<BagSnapshot> = None;
    while start.elapsed() < timeout {
        if let Some(snap) = bag_snapshot(bag) {
            if ready(&snap) {
                return snap;
            }
            last = Some(snap);
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    let seen = match &last {
        Some(s) => {
            // A count of zero says "not there" and nothing about WHY. The
            // histogram says which record kinds, stamped with which rank, DID
            // reach the bag — which is the difference between "the recorder is
            // not running", "this rank never appeared" and "I am filtering on
            // the wrong field".
            let mut kinds: BTreeMap<(u32, u32), usize> = BTreeMap::new();
            for r in &s.trace {
                *kinds.entry((r.record_type, r.reserved)).or_default() += 1;
            }
            let hist: Vec<String> = kinds
                .iter()
                .map(|((kind, rank), n)| format!("(type {kind}, reserved {rank}) x{n}"))
                .collect();
            format!(
                "{} message(s), {} trace record(s) [{}]{}",
                s.msgs.len(),
                s.trace.len(),
                hist.join(", "),
                match &s.trace_error {
                    Some(e) => format!(" (the trace could not be decoded: {e})"),
                    None => String::new(),
                }
            )
        }
        None => "nothing readable in the bag at all".to_string(),
    };
    panic!(
        "waited {timeout:?} for {what} in {} and it never happened — last saw {seen}. \
         A mid-run read sees only FLUSHED chunks (a chunk closes at \
         `cerulion_bagd::CHUNK_TIME_FLOOR_MS`), so a healthy recorder reaches this \
         within about a chunk; if nothing is readable, the recording itself is the \
         problem.",
        bag.display()
    )
}
/// The file's text, LOSSY.
///
/// `read_to_string` aborts the whole read on the first non-UTF-8 byte and
/// leaves `s` empty, so a single stray byte anywhere in a supervisor log made
/// every `wait_for_log` needle vanish — a test would then fail claiming the
/// line was never logged, pointing the reader at the production code instead of
/// at the log. Reading BYTES and converting lossily keeps the rest of the log
/// readable, which is all these matchers need.
pub fn read_file(p: &Path) -> String {
    match std::fs::read(p) {
        Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
        Err(_) => String::new(),
    }
}

/// Find the `cerulion bagd` grandchild's pid by its unique bag FILENAME
/// (bounded pgrep poll — the sibling `graph_record_e2e_test` pattern). The
/// grandchild is in its OWN process group, so it is only reachable this way.
pub fn find_bagd_pid(bag_path: &Path, timeout: Duration) -> Option<u32> {
    let needle = bag_path
        .file_name()
        .expect("bag filename")
        .to_string_lossy()
        .to_string();
    let start = Instant::now();
    while start.elapsed() < timeout {
        let out = Command::new("pgrep")
            .args(["-f", &needle])
            .output()
            .expect("pgrep");
        if let Some(pid) = String::from_utf8_lossy(&out.stdout)
            .lines()
            .filter_map(|l| l.trim().parse::<u32>().ok())
            .next()
        {
            return Some(pid);
        }
        std::thread::sleep(Duration::from_millis(30));
    }
    None
}

/// The worker pid running a SPECIFIC group's plan, or `None`.
///
/// The supervisor's direct child whose argv holds `plan_<group>.json` (the
/// `mp_supervisor_box_test` pattern: `pgrep -P <supervisor> -f plan_p1.json`; the
/// plan filename is group-unique within a deployment and parent-scoping keeps
/// concurrent deployments apart). Bounded poll.
///
/// `None` CONFLATES three outcomes and a caller must say which it means: the
/// worker never spawned, it spawned and already EXITED, or `pgrep` itself
/// failed. An arm that `.expect()`s this should name all three in its message —
/// "could not locate the p1 worker" reads as the first, and the second is what
/// a genuinely broken run produces.
///
/// A pid is returned only when `> 1`: `pgrep` output that fails to parse would
/// otherwise yield 0, which every `kill` treats as "my own process group".
pub fn worker_pid_for_group(supervisor_pid: u32, group: &str, timeout: Duration) -> Option<u32> {
    let pat = format!("plan_{group}.json");
    let start = Instant::now();
    while start.elapsed() < timeout {
        if let Ok(out) = Command::new("pgrep")
            .args(["-P", &supervisor_pid.to_string(), "-f", &pat])
            .output()
        {
            if let Some(pid) = String::from_utf8_lossy(&out.stdout)
                .lines()
                .find_map(|l| l.trim().parse::<u32>().ok())
            {
                if pid > 1 {
                    return Some(pid);
                }
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    None
}

/// The stamped ring tags for a `mpdemo` run under supervisor `sup_pid` — the
/// EXACT scheme the supervisor mints (`stamp_recording_rings` /
/// `departure_ring_tag`; pinned unit-side by
/// `recording_ring_tags_are_distinct_and_pid_scoped`, and breadcrumbed at
/// stamp time). Used by the ring-sweep asserts.
pub fn expected_ring_tags(sup_pid: u32) -> [String; 3] {
    [
        format!("cer_rec_mpdemo_{sup_pid}_r0"),
        format!("cer_rec_mpdemo_{sup_pid}_r1"),
        format!("cer_rec_mpdemo_{sup_pid}_dep"),
    ]
}

/// Portable (macOS-safe, no /dev/shm listing) ring-sweep assert: opening any
/// of the run's ring names must FAIL after the supervisor exits —
/// cleanly-exited workers unlinked their own, and the supervisor's
/// `WorkerRingSweeper` + departure-ring drop covered every other one
/// (SIGKILLed workers included).
pub fn assert_rings_swept(sup_pid: u32) {
    for tag in expected_ring_tags(sup_pid) {
        assert!(
            TraceRingConsumer::open(&ring_shm_name(&tag)).is_err(),
            "ring `{tag}` must be unlinked after the supervisor exits \
             (no /dev/shm leak on any exit path)"
        );
    }
}

/// Parse a `__cerulion/trace_manifest_rank{N}.json` attachment: `(rank,
/// node_ids)`. `None` if the attachment is absent.
pub fn read_manifest(reader: &BagReader, rank: u32) -> Option<(u64, Vec<String>)> {
    let att = reader
        .attachment(&format!("__cerulion/trace_manifest_rank{rank}.json"))
        .expect("read attachments")?;
    let v: serde_json::Value = serde_json::from_slice(&att.data).expect("manifest json parses");
    let got_rank = v["rank"].as_u64().expect("manifest rank field");
    let node_ids = v["node_ids"]
        .as_array()
        .expect("manifest node_ids array")
        .iter()
        .map(|s| s.as_str().expect("node id string").to_string())
        .collect();
    Some((got_rank, node_ids))
}

/// Group trace records by their bagd-stamped `reserved` rank, preserving
/// per-rank stream order (SPSC ring FIFO ⇒ in-rank order == push order).
pub fn by_rank(trace: &[TraceRingRecord]) -> BTreeMap<u32, Vec<&TraceRingRecord>> {
    let mut m: BTreeMap<u32, Vec<&TraceRingRecord>> = BTreeMap::new();
    for r in trace {
        m.entry(r.reserved).or_default().push(r);
    }
    m
}

/// Assert a replay `--report`'s ALWAYS-serialized
/// `read_log` block reports the redundant per-edge read-log verifier as
/// `verified_clean` over at least one compared edge.
///
/// This is the assertion that makes the verifier's outcome CI-VISIBLE. The
/// verifier is REPORT-ONLY (it never touches `passed` or the exit code), so
/// without this assert a divergence, or a verifier that silently went inert,
/// would leave the run GREEN with its `warn!` in libtest's discarded stderr,
/// and a green run would be evidence of nothing. With it, a green run means
/// the verifier compared at least one edge and agreed: a diverging or
/// non-exercised verifier fails here.
///
/// EVERY other status FAILS, which is the vacuity guard rather than
/// strictness — each is a state in which the verifier compared nothing, or
/// compared and disagreed:
///
/// - `not_exercised` — active (kind-6 records + a usable input table) but the
///   replay ended before ANY read was paired.
/// - `inert` — the bag carries no kind-6 READ-OUTCOME records at all (every
///   `trace_format` <= 2 bag, and any graph with no read edges).
/// - `disabled` — the verifier stood down, loud-warned (unusable manifest
///   input table, capacity skew, a broken mid-run invariant).
/// - `diverged` — at least one edge disagreed with the re-derivation.
///
/// `edges_compared` is asserted SEPARATELY even though the engine's `finalize`
/// cannot mint `verified_clean` with zero (it degrades to `not_exercised`): a
/// clean claim must say WHAT it compared, and schema drift must not pass
/// vacuously — the same posture the sibling `node_failures` / `violations`
/// asserts take. An ABSENT or non-string `status` likewise fails.
///
/// `context` names the recording under test, so a failure in one of the
/// several e2es carrying this assert is attributable without a backtrace.
pub fn assert_read_log_verified_clean(report_json: &serde_json::Value, context: &str) {
    let read_log = &report_json["read_log"];
    assert_eq!(
        read_log["status"].as_str(),
        Some("verified_clean"),
        "{context} must leave the redundant read-log verifier \
         VERIFIED CLEAN. The verifier's promotion-evidence window counts GREEN main-CI \
         cycles from this assert — a diverging OR non-exercised verifier must FAIL \
         here, not pass silently, or the window counts cycles that prove nothing. \
         read_log: {read_log}\nfull report: {report_json}"
    );
    assert!(
        read_log["edges_compared"].as_u64().unwrap_or(0) > 0,
        "a `verified_clean` claim must say WHAT it compared — \
         {context} reported zero compared edges, which verifies nothing and must \
         never count toward the verifier's promotion-evidence window. \
         read_log: {read_log}\nfull report: {report_json}"
    );
}

/// The exactly-one-`.mcap` acceptance: multi-process recording writes ONE bag.
pub fn assert_single_bag(recordings: &Path) -> PathBuf {
    let bags: Vec<PathBuf> = std::fs::read_dir(recordings)
        .expect("recordings dir")
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|x| x.to_str()) == Some("mcap"))
        .collect();
    assert_eq!(
        bags.len(),
        1,
        "multi-process recording must write exactly ONE bag, found {bags:?}"
    );
    bags.into_iter().next().unwrap()
}
