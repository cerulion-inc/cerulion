// SPDX-License-Identifier: AGPL-3.0-only
//! The LIVENESS ORACLE a run-directory sweeper reads.
//!
//! # The question, and why a pid cannot answer it
//!
//! A SIGKILLed run leaves POSIX SHM objects behind (`cer_rg_*` trace and state
//! rings, `cer_sta_*` arm words) whose names are FNV hashes, so `/dev/shm` is
//! unattributable on its own — the run directory's `run.json` is the only
//! ledger that says which names belonged to which run. Reclaiming them means
//! answering one question about each run directory: **is that run still
//! running?**
//!
//! The two obvious answers are both wrong:
//!
//! * `kill(supervisor_pid, 0)` — pids are REUSED. A sweeper that believes a
//!   recycled pid classifies a LIVE run as dead, and unlinking a live run's ring
//!   name makes it unattachable to `cerulion bag record --run` for the rest of
//!   its life. That is the failure this module exists to make unreachable, so
//!   the pid is corroboration in a log line and never a verdict.
//! * the `/__cerulion/runs` registry — `Ending` is a best-effort LAST WORD
//!   (MEASURED 0/8 at a 250 ms poll, `run_dir::RunDescriptor::drop`), so a
//!   polling reader cannot tell a graceful exit from a crash, and a run that
//!   could not publish at all (`is_discoverable() == false`) is not in the
//!   registry while being perfectly alive.
//!
//! `flock(2)` answers it exactly, because the KERNEL releases the lock — on
//! `close`, on exit, and on SIGKILL, with no cooperation from the dying process.
//! A held lock is therefore proof of life that a crashed run cannot fake, and a
//! free lock is proof of death that a live run cannot accidentally present.
//!
//! # Fail toward NOT sweeping
//!
//! The two error directions are not symmetric, and the whole module is shaped by
//! that asymmetry:
//!
//! | Mistake | Cost |
//! |---|---|
//! | sweeping a LIVE run | its rings become unattachable by name; a mid-run `bag record --run` silently loses the scheduler trace it came for |
//! | leaking a DEAD run | bounded SHM (~40 MiB per trace ring, ~64 MiB per state ring) until the next run or a reboot, and it is WARNED |
//!
//! So every ambiguity resolves to "leave it alone": an absent lock file is
//! [`LockState::Absent`] and NEVER sweepable (a run from a build that predates
//! this module, or one killed inside the `mkdir`→flock window, is UNKNOWN — not
//! dead), and an unopenable lock is [`LockState::Unreadable`], likewise
//! untouched. Only an ACQUIRED lock authorises reclaiming anything.
//!
//! # Ordering is the contract
//!
//! `run.lock` is created and flocked BEFORE `run.json` is rendered
//! (`run_dir::start_run_descriptor`), and each worker flocks
//! `worker-<rank>.lock` BEFORE its READY sentinel. That ordering is what makes
//! the oracle sound rather than merely available: a sweeper that can SEE a
//! ledger is looking at a directory whose owner already holds its lock, so the
//! `mkdir`→flock window contains nothing to sweep and cannot be mis-classified.
//! Reversing it — rendering `run.json` first — reopens exactly that race, and is
//! the mutation `run_sweep`'s start-race test kills.
//!
//! # Scope
//!
//! Unix only, like the run directory itself. `flock` is POSIX-advisory and is
//! honoured on Linux and macOS local filesystems; on an NFS-backed
//! `~/.cerulion/runs` it degrades (documented limitation U14 — a network home is
//! not a supported robot deployment).

#[cfg(unix)]
use std::fs::File;
use std::path::{Path, PathBuf};

/// The run OWNER's lock file: created + flocked before `run.json` exists.
pub const RUN_LOCK_FILE: &str = "run.lock";

/// One worker's lock file, `worker-<rank>.lock`: flocked before READY.
#[must_use]
pub fn worker_lock_file(rank: usize) -> String {
    format!("worker-{rank}.lock")
}

/// One mid-run attacher's lock file, `attach-<pid>.lock`.
///
/// Keyed by PID rather than by a counter because an attacher is an INDEPENDENT
/// process that may crash: a name it derives itself needs no coordination with
/// any other attacher, and a stale one left by a crash is exactly what the
/// flock probe is for (the file survives, the lock does not).
#[must_use]
pub fn attach_lock_file(pid: u32) -> String {
    format!("attach-{pid}.lock")
}

/// Is `name` one of this module's lock files?
///
/// Used by the sweeper's report and by tests; kept here so the three spellings
/// above have exactly one recogniser.
#[must_use]
pub fn is_lock_file(name: &str) -> bool {
    name == RUN_LOCK_FILE
        || (name.starts_with("worker-") && name.ends_with(".lock"))
        || (name.starts_with("attach-") && name.ends_with(".lock"))
}

/// What a probe of one lock file found.
///
/// Only [`LockState::Free`] licenses reclaiming; every other arm means "leave it
/// alone" for a DIFFERENT reason, and the sweeper's log distinguishes them
/// because they call for different operator action (a `Held` run is working
/// normally; an `Absent` one is an older build's leftover; an `Unreadable` one is
/// a permissions problem).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LockState {
    /// The lock is HELD by a live process — the run is running.
    Held,
    /// The lock file exists and we acquired it — its holder is gone.
    Free,
    /// No lock file at all. UNKNOWN, never dead: a directory written by a build
    /// that predates this module has no lock and its run may still be live.
    Absent,
    /// The lock file exists but could not be opened (permissions, a foreign
    /// owner, a full fd table). UNKNOWN.
    Unreadable(String),
}

impl LockState {
    /// Does this state PROVE the holder is gone?
    ///
    /// Exactly one arm does. Written as a method rather than a `matches!` at the
    /// call sites so the "only `Free`" rule has one home and a future arm cannot
    /// be silently folded into the sweepable side.
    #[must_use]
    pub fn proves_dead(&self) -> bool {
        matches!(self, LockState::Free)
    }
}

/// An ACQUIRED, held `flock(LOCK_EX | LOCK_NB)` — released when dropped, and by
/// the kernel if the process dies without dropping it.
///
/// The `File` is the lock: `flock` is owned by the open file DESCRIPTION, so
/// holding this struct is holding the lock, and there is no explicit unlock
/// call to forget.
#[cfg(unix)]
#[derive(Debug)]
#[must_use = "the lock is released the moment this value drops — bind it for the run's lifetime"]
pub struct RunLock {
    /// Kept solely to HOLD the `flock` — never read, and underscored to say so.
    ///
    /// `flock` is owned by the open file DESCRIPTION, so this handle is the
    /// lock: closing it (by dropping this struct, or by the process dying)
    /// releases it, and there is no unlock call anybody can forget.
    _file: File,
    path: PathBuf,
}

#[cfg(unix)]
impl RunLock {
    /// Create (if needed) and EXCLUSIVELY lock `path`, without blocking.
    ///
    /// # Errors
    ///
    /// [`std::io::ErrorKind::WouldBlock`] when somebody else already holds it —
    /// which for `run.lock` means a live run owns this directory, and for
    /// `worker-<rank>.lock` means two workers claimed one rank. Any other error
    /// is the underlying `open`/`flock` failure.
    pub fn acquire(path: &Path) -> std::io::Result<Self> {
        // 0600 like every other run-directory artifact: the lock file's mere
        // NAME leaks nothing, but it sits inside a 0700 directory whose whole
        // rule is owner-only, and a file that escapes it (a copy, a backup
        // sweep) should carry its own restriction.
        //
        // CREATE_NEW first, then fall back to opening an existing file, so this
        // call KNOWS whether it minted the artifact. That knowledge is the whole
        // point — see the failure path below.
        let mut opts = std::fs::OpenOptions::new();
        opts.write(true).create_new(true);
        {
            use std::os::unix::fs::OpenOptionsExt;
            opts.mode(0o600);
        }
        let (file, we_created_it) = match opts.open(path) {
            Ok(f) => (f, true),
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => (
                // Somebody else's file (a live holder's, or a dead run's
                // leftover). Never truncate it: its CONTENT is irrelevant but
                // truncating another process's open file is a gratuitous write
                // to a path we may not own the lock on.
                std::fs::OpenOptions::new().write(true).open(path)?,
                false,
            ),
            Err(e) => return Err(e),
        };
        if let Err(e) = flock_exclusive_nonblocking(&file) {
            // A FAILED ACQUIRE MUST NOT LEAVE A FREE LOCK FILE BEHIND.
            //
            // The sweeper's oracle is "the file exists and I could lock it ⇒ its
            // holder is gone". A create that succeeded followed by a flock that
            // did NOT would leave exactly that shape while the caller runs on
            // unprotected — an artifact that reads as proof of death for a
            // process that is very much alive. Removing it restores the
            // `Absent`-is-UNKNOWN arm, which is the never-sweep answer.
            //
            // Only what THIS call created is removed. An `AlreadyExists` path
            // belongs to somebody else — usually a live holder we just lost the
            // race to — and unlinking it would strip a live run of its own
            // liveness evidence, which is the very hazard this arm exists to
            // prevent.
            if we_created_it {
                let _ = std::fs::remove_file(path);
            }
            return Err(e);
        }
        Ok(Self {
            _file: file,
            path: path.to_path_buf(),
        })
    }

    /// The locked path (for log lines and tests).
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }
}

/// Probe one lock file WITHOUT keeping it.
///
/// The acquired lock is dropped before returning, so a probe never blocks a
/// subsequent legitimate acquisition — the answer is a snapshot, which is all a
/// sweeper needs: the run it is about to reclaim is one whose owner is already
/// gone, and a run cannot come back to life under a path whose directory is
/// about to be removed.
///
/// # Self-probing is SAFE, and load-bearing
///
/// `flock` contends across independent open file DESCRIPTIONS, so this call
/// contends with a lock THIS process already holds through another `File`.
/// That is deliberate: the sweeping run holds its own `run.lock` before it
/// sweeps, so its own directory probes [`LockState::Held`] and is structurally
/// excluded — the sweeper cannot delete itself, and that exclusion is a kernel
/// property rather than a path comparison somebody has to remember to write.
#[cfg(unix)]
#[must_use]
pub fn probe_lock(path: &Path) -> LockState {
    // Deliberately NOT `create(true)`: a probe must never MINT a lock file. If
    // it did, every ledger-less directory would gain one on first sweep and the
    // `Absent`-is-unknown rule would quietly become "absent until observed,
    // then free" — which is the live-run hazard wearing a different hat.
    let file = match std::fs::OpenOptions::new().write(true).open(path) {
        Ok(f) => f,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return LockState::Absent,
        Err(e) => return LockState::Unreadable(e.to_string()),
    };
    match flock_exclusive_nonblocking(&file) {
        Ok(()) => LockState::Free,
        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => LockState::Held,
        Err(e) => LockState::Unreadable(e.to_string()),
    }
}

/// `flock(LOCK_EX | LOCK_NB)` on `f`.
///
/// Cribbed from `cerulion_netd::hygiene`'s daemon-singleton lock — the same
/// primitive for the same reason (the kernel releases it on SIGKILL, so it
/// cannot lie about a dead holder). Not shared with that crate because netd is
/// not a dependency of this one and a two-line `libc` call is not worth a new
/// package edge.
#[cfg(unix)]
fn flock_exclusive_nonblocking(f: &File) -> std::io::Result<()> {
    use std::os::unix::io::AsRawFd;
    // Test-only fault seam: the create-succeeded-then-flock-FAILED path is the
    // one this module's cleanup arm exists for, and it cannot be produced by
    // ordinary input — a contended lock always means the file already existed,
    // so the natural `WouldBlock` never reaches the cleanup. Real causes (ENOLCK
    // on an exhausted lock table, EINTR) are rare and unreachable from a test.
    #[cfg(test)]
    if FAULT_FLOCK.with(std::cell::Cell::get) {
        return Err(std::io::Error::from_raw_os_error(libc::ENOLCK));
    }
    // SAFETY: `f` is a live, open file owned by the caller for the whole call,
    // so its raw fd is valid; `flock` only reads it.
    let rc = unsafe { libc::flock(f.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) };
    if rc != 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}

// Arms `flock_exclusive_nonblocking`'s fault seam. See its comment.
//
// THREAD-SCOPED, not process-global, and that is isolation rather than
// tidiness: libtest runs tests in parallel, so a process-wide flag armed by one
// arm makes every CONCURRENT sibling's ordinary `RunLock::acquire` fail with a
// fault it never asked for. That is not hypothetical — it was measured here as
// `Unreadable("No locks available (os error 77)")` inside an unrelated probe
// test. `#[serial]` on the arming test is NOT sufficient: it serialises against
// other `#[serial]` tests only, and the victim was an ordinary one. (The
// `CountingAllocator` thread-scoping lesson, applied to a fault seam.)
#[cfg(test)]
thread_local! {
    static FAULT_FLOCK: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// RAII arming guard, so a panicking arm cannot leave the seam armed for the
/// rest of the thread.
#[cfg(test)]
fn fault_inject_flock() -> impl Drop {
    struct Guard;
    impl Drop for Guard {
        fn drop(&mut self) {
            FAULT_FLOCK.with(|f| f.set(false));
        }
    }
    FAULT_FLOCK.with(|f| f.set(true));
    Guard
}

/// Non-Unix: there are no run directories, so there is nothing to lock. The
/// probe reports UNKNOWN, which is the never-sweep answer.
#[cfg(not(unix))]
#[must_use]
pub fn probe_lock(_path: &Path) -> LockState {
    LockState::Unreadable("run locks are Unix-only".to_string())
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;

    /// Hand oracle for the three lock-file spellings — the sweeper's directory
    /// walk skips them by name, so a rename that this recogniser does not know
    /// about would make it try to parse a lock file as a ledger.
    #[test]
    fn the_lock_file_recogniser_matches_exactly_the_three_spellings() {
        assert!(is_lock_file(RUN_LOCK_FILE));
        assert!(is_lock_file(&worker_lock_file(0)));
        assert!(is_lock_file(&worker_lock_file(17)));
        assert!(is_lock_file(&attach_lock_file(4242)));
        // …and nothing else in a run directory.
        for other in [
            "run.json",
            "graph.yaml",
            "env.json",
            "recorder.json",
            "worker-0.json",
            "attach.lock.bak",
            "lock",
        ] {
            assert!(!is_lock_file(other), "`{other}` must not read as a lock");
        }
    }

    #[test]
    fn worker_and_attach_lock_names_carry_their_key() {
        assert_eq!(worker_lock_file(3), "worker-3.lock");
        assert_eq!(attach_lock_file(99), "attach-99.lock");
    }

    /// The three probe verdicts against real files, plus the ONE that licenses
    /// a sweep. Anti-tautology: the same path answers `Held` then `Free`
    /// depending only on whether the lock is still bound, so the probe is
    /// reading the kernel rather than the filesystem.
    #[test]
    fn a_probe_reads_the_lock_not_the_file() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join(RUN_LOCK_FILE);

        assert_eq!(
            probe_lock(&path),
            LockState::Absent,
            "no file at all is UNKNOWN, never dead"
        );
        assert!(!LockState::Absent.proves_dead());

        let held = RunLock::acquire(&path).expect("acquire");
        assert_eq!(
            probe_lock(&path),
            LockState::Held,
            "a held lock must contend even from the SAME process — this is what \
             makes a sweeping run unable to delete itself"
        );
        assert!(!LockState::Held.proves_dead());

        drop(held);
        assert_eq!(
            probe_lock(&path),
            LockState::Free,
            "the file survives the holder; the LOCK does not"
        );
        assert!(LockState::Free.proves_dead());
    }

    /// A second acquisition of a held lock fails with `WouldBlock` rather than
    /// blocking — the sweeper's walk must be bounded.
    #[test]
    fn a_second_acquire_of_a_held_lock_would_block_rather_than_wait() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join(RUN_LOCK_FILE);
        let _held = RunLock::acquire(&path).expect("first acquire");
        let err = RunLock::acquire(&path).expect_err("second acquire must fail");
        assert_eq!(err.kind(), std::io::ErrorKind::WouldBlock);
    }

    /// A probe must not CREATE the lock file it probes. Without this, the first
    /// sweep would mint locks for every ledger-less directory and the
    /// `Absent`-is-unknown rule would decay into "free on the second sweep".
    #[test]
    fn a_probe_never_mints_the_lock_file() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join(RUN_LOCK_FILE);
        assert_eq!(probe_lock(&path), LockState::Absent);
        assert!(
            !path.exists(),
            "probing an absent lock must leave it absent"
        );
    }

    /// A FAILED acquire must not leave the lock file it
    /// created behind.
    ///
    /// The sweeper's oracle is "the file exists and I could lock it ⇒ its holder
    /// is gone". A create that succeeded followed by a flock that did NOT would
    /// leave exactly that shape while the caller runs on unprotected — an
    /// artifact that reads as PROOF OF DEATH for a live process. Removing it
    /// restores `Absent`, which is the never-sweep answer.
    #[test]
    #[serial_test::serial]
    fn a_failed_acquire_removes_the_lock_file_it_created() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join(RUN_LOCK_FILE);
        {
            let _fault = fault_inject_flock();
            let err = RunLock::acquire(&path).expect_err("the injected flock must fail");
            assert_eq!(err.raw_os_error(), Some(libc::ENOLCK));
        }
        assert!(
            !path.exists(),
            "a failed acquire must not leave a lock file that reads as Free"
        );
        assert_eq!(
            probe_lock(&path),
            LockState::Absent,
            "…and the sweeper must therefore see UNKNOWN, never a sweepable run"
        );
        // Anti-tautology: with the fault disarmed the same call DOES create the
        // file, so the assertion above is about the failure path rather than
        // about `acquire` never creating anything.
        let held = RunLock::acquire(&path).expect("acquire");
        assert!(path.exists());
        assert_eq!(probe_lock(&path), LockState::Held);
        drop(held);
    }

    /// The OTHER direction of the same fix, and the dangerous one: a failed
    /// acquire must NOT remove a lock file it did not create.
    ///
    /// `WouldBlock` means somebody else is holding it — usually a live run we
    /// just lost the race to. Unlinking there would strip a LIVE run of its own
    /// liveness evidence, turning a conservative refusal into the exact
    /// catastrophic sweep this module exists to prevent.
    #[test]
    #[serial_test::serial]
    fn a_contended_acquire_leaves_the_holders_lock_file_alone() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join(RUN_LOCK_FILE);
        let _holder = RunLock::acquire(&path).expect("the holder acquires");
        let err = RunLock::acquire(&path).expect_err("the contender must fail");
        assert_eq!(err.kind(), std::io::ErrorKind::WouldBlock);
        assert!(
            path.exists(),
            "the HOLDER's lock file must survive a contender's failed acquire"
        );
        assert_eq!(
            probe_lock(&path),
            LockState::Held,
            "and the run must still read as LIVE"
        );
    }

    /// A dead run's leftover lock file is re-acquirable in place — the
    /// `create_new`-then-fall-back path. Without the fallback, `create_new`
    /// would `AlreadyExists` on every restart and no run could ever take a lock
    /// in a directory that had one before.
    #[test]
    #[serial_test::serial]
    fn a_leftover_free_lock_file_is_re_acquired_in_place() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join(RUN_LOCK_FILE);
        drop(RunLock::acquire(&path).expect("first holder"));
        assert!(path.exists(), "the file outlives its holder");
        let _second = RunLock::acquire(&path).expect("a leftover file must be re-acquirable");
        assert_eq!(probe_lock(&path), LockState::Held);
    }

    /// Only `Free` proves death — pinned as a set so a new arm cannot be folded
    /// onto the sweepable side without this failing.
    #[test]
    fn exactly_one_lock_state_proves_death() {
        let all = [
            LockState::Held,
            LockState::Free,
            LockState::Absent,
            LockState::Unreadable("boom".to_string()),
        ];
        let dead: Vec<_> = all.iter().filter(|s| s.proves_dead()).collect();
        assert_eq!(dead, vec![&LockState::Free]);
    }
}
