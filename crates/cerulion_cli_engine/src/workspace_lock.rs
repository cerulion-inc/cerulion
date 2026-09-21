// SPDX-License-Identifier: AGPL-3.0-only
//! Inter-process serialization for workspace mutations.
//!
//! The lock is a kernel `flock(2)` on `<root>/.cerulion/workspace.lock`, so it
//! serializes across PROCESSES (the `cerulion` CLI and `cerulion-wsd` share it)
//! as well as across threads. Within one thread it is reentrant: a nested
//! [`WorkspaceLock::acquire`] on the same workspace reuses the held file
//! description instead of dead-locking on a second `flock`.
//!
//! What creates the lock file — what writes a TRACKED file, and what never
//! does — is part of the contract. THREE constructors, and the tracked-file
//! side effect belongs to exactly one of them:
//!
//! * [`WorkspaceLock::acquire`] is the DEFAULT: it creates `.cerulion/` and
//!   the lock file on first use, blocks in the kernel while another writer
//!   holds it, and writes NO tracked file. A default-named constructor cannot
//!   modify a file the user's VCS tracks — that is the whole of the naming
//!   rule here.
//! * [`WorkspaceLock::acquire_and_track_gitignore`] is `acquire` PLUS the one
//!   tracked-file side effect: when THIS acquisition creates `.cerulion/` it
//!   also appends `.cerulion/` to an EXISTING `<root>/.gitignore` that lacks
//!   the entry, so a workspace scaffolded before the lock existed does not
//!   grow an untracked directory. A workspace with no `.gitignore` gets none.
//!   It is for the verbs that OWN `<root>/.cerulion` — `node`, `graph`,
//!   `schema`, `ros2 attach`, `graph partition`, and `cerulion-wsd`'s
//!   mutation dispatch — and its name says what it does, so no caller reaches
//!   it by writing the shortest thing that compiles.
//! * [`WorkspaceLock::acquire_interruptibly`] serializes EXACTLY like
//!   `acquire` — same file, same kernel lock, same reentrancy, same absence of
//!   tracked writes — and WAITS DIFFERENTLY: `acquire` blocks in the kernel,
//!   which a handler installed with `SA_RESTART` (as `cerulion_cli`'s `ctrlc`
//!   does) makes uninterruptible, while this one polls so the caller's own
//!   interrupt predicate is observed. BOTH contention paths honour it: the
//!   kernel `flock` is retried non-blockingly, and a same-process peer is
//!   waited out on the reentrancy registry's condvar with `wait_timeout`, the
//!   predicate asked between parks with the registry mutex released.
//!   `cerulion ros2 migrate` takes it: its wait happens after the user has
//!   consented to apply, and that verb promises an interruptible write window.
//! * [`WorkspaceLock::acquire_read`] (a READ) never creates anything: it takes
//!   a shared lock when the lock file already exists and otherwise returns an
//!   unlocked guard, so inspecting a workspace leaves no trace in it.
//!
//! The top-up is tied to CREATING `.cerulion/`, and "created" means THIS
//! acquisition's own `create_dir` returned `Ok` — not "the directory was
//! absent a moment ago". Two processes racing a fresh workspace can BOTH
//! see an absent directory, both succeed at `create_dir_all` (which is `Ok`
//! for a directory that already exists), and both run the top-up — which
//! happens BEFORE the lock file is opened and is therefore unserialised.
//! `create_dir` gives exactly one of them `Ok` and the other `AlreadyExists`,
//! so at most one acquisition per creation reports it and at most one runs the
//! top-up.
//!
//! WHAT THAT RACE COSTS, stated at its real strength: a DUPLICATED
//! `.cerulion/` line in a TRACKED file is NOT reachable.
//! `ensure_gitignore_entry` reads the whole file, returns early if any line
//! already matches, and writes the whole file back with `fs::write`, which
//! TRUNCATES. Two racers therefore cannot leave two matching lines by any
//! interleaving. What they can do is lose content: `fs::write` truncates
//! before it writes, so a racer reading in that window sees an empty or
//! partial file and writes back a `.gitignore` missing everything the other
//! one had. A tracked file silently losing a line is the hazard; `create_dir`
//! removes it by letting only one acquisition reach the read-modify-write at
//! all. It also refuses rather than RE-CREATING a `<root>` deleted between the
//! `canonicalize` and the create, which `create_dir_all` would have rebuilt.
//!
//! Whichever acquisition creates the directory still decides for the
//! workspace: once an `acquire` — or anything else that writes there, such as
//! migrate's manifest — has created it, a later
//! `acquire_and_track_gitignore` finds the directory present and tops nothing
//! up. Pinned by `the_gitignore_top_up_is_tied_to_creating_the_lock_directory`
//! and, for the race, by
//! `a_racer_that_creates_the_lock_directory_first_leaves_the_top_up_to_nobody_else`.
//!
//! [`WorkspaceLock::lock_dir_origin`] reports which of THREE states this guard
//! is in — [`LockDirOrigin`] — because a bool cannot tell a nested acquire
//! inside a run that created the directory apart from one that found a
//! directory someone else created last week, and a top-up rule has to.
//!
//! Contention is LOUD on every shape, once per contended WAIT and never once
//! per poll. An exclusive acquire first tries a non-blocking `flock`; when
//! another PROCESS holds the lock it emits one `tracing::warn!` naming the
//! lock file, then waits. A peer THREAD of this process is waited out on the
//! reentrancy registry's condvar below, where the kernel `flock` is never
//! reached — that wait warns too, in its own words, because `cerulion-wsd` is
//! the multi-threaded consumer and the in-process shape is its common one.
//! Per WAIT, not per acquire: one call that queues behind a peer thread and
//! then behind another process emits two lines, one for each thing it waited
//! on.
//!
//! The `#[cfg(not(unix))]` arms in this module return guards that lock
//! nothing, and say so. Read them as VESTIGIAL rather than as a supported
//! platform: `AcquireError` names `CliError`, whose import here is itself
//! `#[cfg(unix)]`, and twenty-odd other modules in this crate use
//! `std::os::unix` ungated — the crate does not build off Unix today.
//!
//! The lock is per-ROOT. Two writers serialize only when they resolve to the
//! same directory: `cerulion ros2 migrate`'s `--workspace` is a COLCON root,
//! while `node`/`graph`/`schema`/`ros2 attach` lock the CERULION workspace
//! root, so those verbs exclude each other only in a layout where the two
//! coincide.
//!
//! An exclusive acquire refuses a SYMLINKED `<root>/.cerulion` rather than
//! following it, and opens the lock file `O_NOFOLLOW`: a planted link would
//! otherwise have the lock created — and flocked — outside the workspace.
//! [`WorkspaceLock::acquire_read`] carries the same walk: a
//! read creates nothing, so an absent `.cerulion` or lock file is still an
//! unlocked guard — but a LINK is refused there too, because a reader that
//! followed one would take a shared lock on another tree's file and then
//! report this workspace as safely readable.
//!
//! That walk opens the DIRECTORY with the platform's SEARCH-ONLY flag —
//! `O_SEARCH` on Apple, `O_PATH` on Linux, `O_RDONLY` elsewhere (see
//! `LOCK_DIR_OPEN_FLAGS`) — because a plain pathname open needs
//! only search permission on `.cerulion` while `O_DIRECTORY|O_RDONLY` needs
//! READ, so a `.cerulion` at mode `0311` would go from working to refused.
//!
//! A read NEVER waits on a lock this thread itself holds. `flock` locks belong
//! to an open file DESCRIPTION, so a thread holding the exclusive lock that
//! calls [`WorkspaceLock::acquire_read`] on the same root would contend with
//! itself and park in the kernel forever. That is refused with a named
//! internal-bug error instead; contention from another
//! THREAD or another PROCESS still waits, which is the legitimate case.
//!
//! THE CONVERSE IS NOT REFUSED, and saying so is the point of writing it down:
//! a thread holding a [`WorkspaceReadLock`] that then acquires EXCLUSIVELY on
//! the same root still parks in `flock(LOCK_EX)` forever. A read registers
//! nothing — it is deliberately free of registry state, so there is nothing
//! for the exclusive path to consult — which makes this direction structurally
//! undetectable where the other one was merely undetected. No caller does it
//! (`cerulion-wsd`'s dispatch takes exactly one lock per request and never
//! writes under a read guard), so it is a carve-out, not a guarantee. Closing
//! it needs a thread-local record of held read guards, which is a design
//! change rather than this fix.

#[cfg(unix)]
use std::collections::HashMap;
use std::path::Path;
#[cfg(unix)]
use std::path::PathBuf;
#[cfg(unix)]
use std::sync::{Arc, Condvar, Mutex, OnceLock, Weak};
#[cfg(unix)]
use std::thread::ThreadId;

#[cfg(unix)]
use crate::error::CliError;
use crate::error::CliResult;

#[cfg(unix)]
use std::ffi::CString;
#[cfg(unix)]
use std::fs::{self, File};
#[cfg(unix)]
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
#[cfg(unix)]
use std::os::unix::ffi::OsStrExt;
#[cfg(unix)]
use std::time::Duration;

/// The lock directory inside a workspace root.
pub const LOCK_DIR: &str = ".cerulion";
/// The lock file inside [`LOCK_DIR`].
pub const LOCK_FILE: &str = "workspace.lock";

/// Where `<root>/.cerulion/` came from, as far as THIS guard is concerned.
///
/// Three states rather than a bool, and the third is the point: a nested
/// acquire inside a run that created the directory is a different fact from a
/// directory someone else created last week, and a `.gitignore` top-up rule is
/// exactly the kind of rule that has to tell them apart. A bool collapses them
/// and then reports the same `false` for both.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LockDirOrigin {
    /// THIS acquisition's own `create_dir` returned `Ok`, so it created
    /// `<root>/.cerulion/`. At most one acquisition in the world reports this
    /// per creation: `create_dir` (not `create_dir_all`) gives every other
    /// racer `AlreadyExists`.
    CreatedHere,
    /// This guard created nothing, but the acquisition it NESTS inside — an
    /// outer guard this thread still holds — created the directory during this
    /// run.
    CreatedByAnOuterAcquisition,
    /// The directory was already there when this run reached it.
    PreExisting,
}

/// Whether a guard OPENED the file description it holds, or is reusing one an
/// outer guard on the same thread already had.
///
/// A named type rather than a `bool`, because it sits next to a second
/// truth-valued fact (did this run create the directory) at
/// [`lock_dir_origin`]'s call site: two adjacent `bool` parameters can be
/// swapped silently, and the swap is exactly the mistake that would make a
/// nested guard claim the creation.
#[cfg(unix)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Nesting {
    /// This guard opened the file description.
    Outermost,
    /// It reuses one this thread already holds, so it created nothing itself.
    InsideAnOuterAcquisition,
}

/// Which of the three origins a guard reports.
///
/// Pure and total over its four inputs, so it can be mutated and oracle-tested
/// without touching a filesystem. All four are reached — nested or not × the
/// run created the directory or not — but they are NOT all reached from the
/// same place, and it is worth being exact about that rather than claiming
/// more:
///
/// * the `.gitignore` top-up hangs off the OUTERMOST half. Its call site passes
///   [`Nesting::Outermost`] literally (a nested acquire returned long before),
///   so collapsing `(Outermost, true)` changes what is written to a TRACKED
///   file — but collapsing the NESTED arm changes nothing that is written
///   anywhere.
/// * the nested arm exists for the REPORT: it is the state a bool cannot hold,
///   and the one a future top-up rule must tell apart from "somebody created
///   this last week". Today its only consumer is
///   [`WorkspaceLock::lock_dir_origin`] and the tests that pin it.
///
/// The whole collapse is not tracked-file-visible.
///
/// `created_by_the_run` is whether the acquisition that OPENED the file
/// description this guard holds created `<root>/.cerulion/`.
#[cfg(unix)]
fn lock_dir_origin(nesting: Nesting, created_by_the_run: bool) -> LockDirOrigin {
    match (nesting, created_by_the_run) {
        (Nesting::Outermost, true) => LockDirOrigin::CreatedHere,
        (Nesting::InsideAnOuterAcquisition, true) => LockDirOrigin::CreatedByAnOuterAcquisition,
        (_, false) => LockDirOrigin::PreExisting,
    }
}

/// The flags `<root>/.cerulion` is opened with to anchor the descriptor walk.
///
/// The walk needs a directory descriptor to `openat` through, and it needs to
/// refuse a link — but it does NOT need to read the directory, and asking for
/// read permission is a real narrowing: a plain pathname open
/// needs only SEARCH on `.cerulion`, so a workspace whose lock directory is
/// mode `0311` works with one and is refused by the other. Each
/// platform spells "open a directory for search only" differently:
///
/// * Apple: `O_SEARCH`, which is itself `O_EXEC | O_DIRECTORY`. MEASURED on
///   macOS 26.0.1 (Darwin 25) as an unprivileged uid: `0311` opens, and `openat` + `flock`
///   through the descriptor both work, while `O_RDONLY|O_DIRECTORY` fails
///   `EACCES` — and a symlink, a dangling symlink and a regular file still
///   fail `ENOTDIR`, byte-for-byte the errnos the refusal wording keys on.
/// * Linux (and Android, which carries the same constant): `O_PATH`, which
///   opens the object without checking permission on it and is explicitly
///   documented as usable as the `dirfd` of `openat`. `O_DIRECTORY` and
///   `O_NOFOLLOW` are two of the three flags `O_PATH` still honours (the third
///   is `O_CLOEXEC`, and this constant must not grow a fourth — `O_PATH`
///   silently IGNORES every other flag, so a fourth would be INERT rather than
///   rejected, which is a stronger reason for the rule than a loud errno would
///   be), so the refusals survive — though by
///   a different MECHANISM than elsewhere, worth stating exactly: under
///   `O_PATH|O_NOFOLLOW` a trailing symlink does not fail, it yields a
///   descriptor for the LINK ITSELF, and it is `O_DIRECTORY` that then rejects
///   it with `ENOTDIR`. The refusal arms key on `ELOOP | ENOTDIR`, so both
///   spellings land in the same place. The two platforms also differ on
///   PERMISSION, which the "search-only" shorthand hides: `O_SEARCH` checks
///   execute at open time, `O_PATH` checks nothing on the object, so a
///   `.cerulion` this process cannot search is refused at the DIRECTORY open on
///   Apple and at the lock-FILE open on Linux — same outcome, a different path
///   named in the message; that arm is reasoned, not measured. A kernel
///   without `O_PATH` degrades safely rather than failing: Linux IGNORES
///   unrecognised open flags, so the arm falls back to a plain `O_RDONLY`
///   directory open.
/// * Anything else Unix: `O_RDONLY`, the plain directory open, because there
///   is no portable third spelling and a wrong guess is worse than the
///   narrowing.
///
/// `O_DIRECTORY | O_NOFOLLOW` are the refusal, and they are unconditional:
/// they are what makes a swapped-in link retarget a NAME rather than the
/// descriptor the lock file is opened relative to.
#[cfg(all(unix, target_vendor = "apple"))]
const LOCK_DIR_OPEN_FLAGS: libc::c_int =
    libc::O_SEARCH | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC;
#[cfg(all(unix, any(target_os = "linux", target_os = "android")))]
const LOCK_DIR_OPEN_FLAGS: libc::c_int =
    libc::O_PATH | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC;
#[cfg(all(
    unix,
    not(target_vendor = "apple"),
    not(any(target_os = "linux", target_os = "android"))
))]
const LOCK_DIR_OPEN_FLAGS: libc::c_int =
    libc::O_RDONLY | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC;

/// Whether an acquisition that CREATES `.cerulion/` may also write a TRACKED
/// file (today: top up an existing `<root>/.gitignore`).
#[cfg(unix)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum TrackedFilePolicy {
    /// Append `.cerulion/` to an existing `.gitignore` that lacks it.
    TopUpGitignore,
    /// Leave every tracked file exactly as it was.
    Leave,
}

/// How to wait out another holder.
#[cfg(unix)]
enum ContentionWait<'a> {
    /// Block in the kernel until the holder releases. Simple, and correct for
    /// a caller with nothing to do meanwhile — but `ctrlc` installs its
    /// handler with `SA_RESTART`, so the blocking `flock` is auto-restarted
    /// and a signal cannot end the wait.
    Blocking,
    /// Poll, re-asking the caller's predicate between attempts, so a wait the
    /// user wants out of can end — on BOTH contention paths: the registry
    /// condvar (a same-process peer) parks with `wait_timeout` at the same
    /// cadence, and the kernel `flock` is retried non-blockingly. Costs one
    /// sleep or one bounded park, plus at most one `flock`, per
    /// [`CONTENTION_POLL`].
    PollingUntil(&'a dyn Fn() -> bool),
}

#[cfg(all(test, unix))]
thread_local! {
    /// Test seam: runs between the `.cerulion` metadata check and the descriptor
    /// walk that follows it. That gap is exactly where a swap would have to land to
    /// beat a path-based open, so the pin for the walk has to be able to act there.
    /// Never compiled into a shipped build.
    ///
    /// THREAD-LOCAL, not a process-global static, and that is not a style choice.
    /// libtest runs these tests in PARALLEL, so a global hook fires inside EVERY
    /// other test's acquire — measured: it planted a symlink into an unrelated
    /// test's tempdir (`AlreadyExists` from that test's own fixture) and spent its
    /// own one-shot guard doing it, so the test that installed the hook then
    /// observed no swap at all and its refusal assertion failed. Two failures, one
    /// leak. A thread-local confines the seam to the thread that installed it,
    /// which is the thread whose acquire it is there to interrupt. It also removes
    /// the mutex entirely: a `RefCell` borrow released during unwind cannot poison,
    /// so a hook that panics can no longer wedge the path it observes.
    static PRE_OPEN_HOOK: std::cell::RefCell<Option<Box<dyn Fn()>>> =
        std::cell::RefCell::new(None);
}

/// Gated `all(test, unix)`, not `test`: the only callers are `acquire_with`
/// (itself `#[cfg(unix)]`) and the unix-gated test module, so a bare
/// `#[cfg(test)]` leaves this userless on a non-Unix test build — and
/// `dead_code` is deny at the crate root, so that is a BUILD failure, not a
/// warning. The same gate is on the thread-local and on `set_pre_open_hook`.
///
/// A hook must not reinstall itself from inside its own body (the borrow is
/// held across the call). Nothing needs to, and the alternative — taking the
/// hook out and putting it back — would skip the restore on a panicking hook.
#[cfg(all(test, unix))]
fn run_pre_open_hook() {
    PRE_OPEN_HOOK.with(|slot| {
        if let Some(hook) = slot.borrow().as_ref() {
            hook();
        }
    });
}

/// Install (or clear, with `None`) the seam above ON THIS THREAD. Returns the
/// previous value so a test can restore it.
#[cfg(all(test, unix))]
fn set_pre_open_hook(hook: Option<Box<dyn Fn()>>) -> Option<Box<dyn Fn()>> {
    PRE_OPEN_HOOK.with(|slot| std::mem::replace(&mut *slot.borrow_mut(), hook))
}

#[cfg(all(test, unix))]
thread_local! {
    /// Test seam: runs immediately BEFORE the lock directory is created.
    ///
    /// That window is where the `create_dir_all` TOCTOU lived — a racing
    /// process creating `.cerulion/` there is what made two acquisitions both
    /// believe they created it — and it is the only place a test can stand in
    /// for that racer without actually racing one. THREAD-LOCAL for the same
    /// reason as [`PRE_OPEN_HOOK`]: libtest runs these in parallel and a
    /// process-global hook would fire inside every sibling test's acquire.
    /// Never compiled into a shipped build.
    static PRE_CREATE_HOOK: std::cell::RefCell<Option<Box<dyn Fn()>>> =
        const { std::cell::RefCell::new(None) };
}

#[cfg(all(test, unix))]
thread_local! {
    /// Test seam: how many of the NEXT lock-file opens should behave as though
    /// the kernel had returned the spurious `ENOENT` that macOS/APFS produces
    /// while a peer process is creating the same directory.
    ///
    /// The real condition needs a genuine multi-process race, which is
    /// probabilistic — a test built on one proves nothing on the run where the
    /// racers happen to miss each other, and it cannot say which happened.
    /// This injects the exact observable instead, so the retry that absorbs it
    /// has a deterministic oracle. THREAD-LOCAL for the same reason as the
    /// other seams here: libtest runs these in parallel.
    static FAIL_LOCK_OPEN_ENOENT: std::cell::Cell<u32> = const { std::cell::Cell::new(0) };
}

/// Arm the seam above ON THIS THREAD for the next `count` opens.
#[cfg(all(test, unix))]
fn fail_next_lock_opens_with_enoent(count: u32) {
    FAIL_LOCK_OPEN_ENOENT.with(|slot| slot.set(count));
}

#[cfg(all(test, unix))]
thread_local! {
    /// Test seam: runs inside the lock-FILE open, i.e. AFTER the directory
    /// descriptor has been taken and BEFORE the `openat` through it.
    ///
    /// That window is the only place the frozen-descriptor bug lives: a peer's
    /// `.gitignore` give-back landing here leaves this call holding a
    /// descriptor for a directory that no longer exists, and an `openat`
    /// through such a descriptor fails `ENOENT` unconditionally — so a retry
    /// that reuses it retries a corpse. THREAD-LOCAL for the same reason as the
    /// other seams: libtest runs these in parallel.
    static PRE_FILE_OPEN_HOOK: std::cell::RefCell<Option<Box<dyn Fn()>>> =
        const { std::cell::RefCell::new(None) };
}

#[cfg(all(test, unix))]
fn set_pre_file_open_hook(hook: Option<Box<dyn Fn()>>) -> Option<Box<dyn Fn()>> {
    PRE_FILE_OPEN_HOOK.with(|slot| std::mem::replace(&mut *slot.borrow_mut(), hook))
}

#[cfg(all(test, unix))]
fn run_pre_create_hook() {
    PRE_CREATE_HOOK.with(|slot| {
        if let Some(hook) = slot.borrow().as_ref() {
            hook();
        }
    });
}

/// Install (or clear, with `None`) the pre-create seam ON THIS THREAD.
#[cfg(all(test, unix))]
fn set_pre_create_hook(hook: Option<Box<dyn Fn()>>) -> Option<Box<dyn Fn()>> {
    PRE_CREATE_HOOK.with(|slot| std::mem::replace(&mut *slot.borrow_mut(), hook))
}

/// Gap between polls of the interrupt predicate while an exclusive acquire is
/// contended (the polling variant of `ContentionWait` — a private type, so no
/// intra-doc link). Short enough that a Ctrl-C feels immediate, long enough
/// that a minutes-long wait is ~600 polls a minute rather than a spin. Public
/// so a test that anchors on the wait's cadence (the interrupted-wait arms in
/// `tests/ros2_migrate_test.rs`) reads the value the loop sleeps, not a
/// literal that can drift from it.
#[cfg(unix)]
pub const CONTENTION_POLL: Duration = Duration::from_millis(100);

/// Drift guard: the cadence is a UX/cost trade-off with no test that would
/// notice it being retuned. Below ~10 ms the wait is a spin; above ~500 ms a
/// Ctrl-C stops feeling immediate.
#[cfg(unix)]
const _: () = assert!(
    CONTENTION_POLL.as_millis() >= 10 && CONTENTION_POLL.as_millis() <= 500,
    "CONTENTION_POLL must stay between 10ms and 500ms"
);

#[cfg(unix)]
struct Inner {
    lock: File,
    depth: Mutex<usize>,
    root: PathBuf,
    /// Whether the acquisition that OPENED this file description created
    /// `<root>/.cerulion/`. Lives here rather than on the guard because a
    /// NESTED guard has to report it — as
    /// [`LockDirOrigin::CreatedByAnOuterAcquisition`], the state a bool cannot
    /// express — and the nested guard is not the one that created anything.
    created_lock_dir: bool,
}

#[cfg(unix)]
struct RegistryEntry {
    owner: ThreadId,
    /// `None` means this thread has reserved the workspace and is acquiring
    /// the kernel lock; `Some` is a completed acquisition.
    inner: Option<Weak<Inner>>,
}

#[cfg(unix)]
static REGISTRY: OnceLock<(Mutex<HashMap<PathBuf, RegistryEntry>>, Condvar)> = OnceLock::new();

/// Whether the registry entry under a root is still THIS thread's unpromoted
/// reservation — i.e. the thing a failed or interrupted acquire must remove.
///
/// Pure, so it can be mutated and oracle-tested without touching the live
/// registry: the two conjuncts guard different mistakes. Dropping the owner
/// check lets a thread tear down a PEER's reservation; dropping the
/// `inner.is_none()` check lets it tear down its own COMPLETED acquisition,
/// which would hand the workspace to a second writer while the first still
/// holds the kernel lock.
#[cfg(unix)]
fn reservation_is_ours(entry: Option<&RegistryEntry>, owner: ThreadId) -> bool {
    entry.is_some_and(|entry| entry.owner == owner && entry.inner.is_none())
}

/// How many times each of the exclusive path's two opens — the lock DIRECTORY
/// and the lock FILE — may be attempted before a failure is reported.
///
/// ONE budget rather than two, because the two are the same class of transient:
/// an `ENOENT` that cannot mean "absent" because this call has just established
/// the thing exists. See each call site for what makes it transient there.
#[cfg(unix)]
const LOCK_OPEN_ATTEMPTS: u32 = 5;

/// Drift guard: at 1 there is no retry at all and both `ENOENT` bugs are back;
/// a large value would turn a refusal into an unbounded spin.
#[cfg(unix)]
const _: () = assert!(
    LOCK_OPEN_ATTEMPTS >= 2 && LOCK_OPEN_ATTEMPTS <= 16,
    "LOCK_OPEN_ATTEMPTS must retry at least once and stay bounded"
);

/// Open (creating if absent) the lock FILE, relative to the verified directory
/// descriptor.
///
/// A function rather than an inline `unsafe` block so its CALLER — the retry
/// loop in `acquire_with` — reads as a decision rather than as syscall
/// plumbing, and so the spurious `ENOENT` that loop exists for can be injected
/// deterministically instead of waited for in a real race.
#[cfg(unix)]
fn open_lock_file(dir: &OwnedFd, file_name: &CString) -> Result<libc::c_int, std::io::Error> {
    #[cfg(test)]
    PRE_FILE_OPEN_HOOK.with(|slot| {
        if let Some(hook) = slot.borrow().as_ref() {
            hook();
        }
    });
    #[cfg(test)]
    if FAIL_LOCK_OPEN_ENOENT.with(|slot| {
        let left = slot.get();
        if left == 0 {
            return false;
        }
        slot.set(left - 1);
        true
    }) {
        // Before the syscall, so the injection cannot leak a descriptor.
        return Err(std::io::Error::from_raw_os_error(libc::ENOENT));
    }
    // SAFETY: `dir` is live for the call and `file_name` is NUL-terminated.
    let fd = unsafe {
        libc::openat(
            dir.as_raw_fd(),
            file_name.as_ptr(),
            libc::O_RDWR | libc::O_CREAT | libc::O_NOFOLLOW | libc::O_CLOEXEC,
            0o666 as libc::c_uint,
        )
    };
    if fd < 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(fd)
}

/// Take a BLOCKING `flock`, retrying `EINTR`.
///
/// `flock(2)`'s blocking wait is interruptible, and an unretried `EINTR` turns
/// a legitimate contended wait into a hard refusal reading "Interrupted system
/// call" — which `ros2 migrate` then dresses in the FILESYSTEM remedy, sending
/// the user to audit a `.cerulion` that is fine. The `cerulion` CLI is mostly
/// shielded because `ctrlc` installs its handler with `SA_RESTART`, but
/// `cerulion-wsd` is a tokio process that installs its own handlers and a test
/// binary has no protection at all. BOTH shipped blocking waits go through
/// this — the exclusive one and `acquire_read`'s shared one, which is the
/// longest-parked of the two. The non-blocking `LOCK_NB` probes need no
/// equivalent: the first probe of each path is in no loop, and the poll loop
/// returns hard on any errno but `EWOULDBLOCK`. `flock(2)` documents `EINTR`
/// only for a call that WAITS, and a non-blocking `flock` never waits.
///
/// Returns the `errno` of the failure, or `None` once the lock is held.
///
/// SAFETY (caller): `fd` must be a live, open descriptor for the whole call.
#[cfg(unix)]
unsafe fn flock_blocking(fd: libc::c_int, operation: libc::c_int) -> Option<std::io::Error> {
    loop {
        // SAFETY: the caller guarantees `fd` is live for the whole call.
        if unsafe { libc::flock(fd, operation) } == 0 {
            return None;
        }
        let error = std::io::Error::last_os_error();
        if error.raw_os_error() != Some(libc::EINTR) {
            return Some(error);
        }
    }
}

/// Open the lock DIRECTORY for the descriptor walk, with the one fallback the
/// platform-conditional flags make necessary.
///
/// [`LOCK_DIR_OPEN_FLAGS`] asks for a search-only open. Linux IGNORES
/// unrecognised open flags, so a kernel without `O_PATH` silently degrades to
/// a plain `O_RDONLY` open — but a rejected flag is a real possibility on the
/// Apple arm, which `target_vendor = "apple"` spreads across macOS, iOS, tvOS
/// and watchOS: an `O_EXEC` the kernel refuses comes back `EINVAL`, which is
/// neither `ELOOP` nor `ENOTDIR`, so every acquire in the process would fail
/// with an opaque "opening the directory of" message and no way back.
///
/// `EINVAL` — and ONLY `EINVAL` — therefore retries once with the earlier
/// flags. It costs nothing on a platform that accepts the search-only open (one
/// extra syscall on a path that was already failing), and it cannot weaken the
/// refusals: the retry keeps `O_DIRECTORY|O_NOFOLLOW`, so a link or a
/// non-directory still fails exactly as before. What it gives up on such a
/// platform is only item 17's narrowing — the `0311` case goes back to
/// `EACCES`, which is where it was.
#[cfg(unix)]
fn open_lock_dir(dir_path: &CString) -> libc::c_int {
    // SAFETY: `dir_path` is a NUL-terminated C string that outlives the call.
    let fd = unsafe { libc::open(dir_path.as_ptr(), LOCK_DIR_OPEN_FLAGS) };
    if fd >= 0 || std::io::Error::last_os_error().raw_os_error() != Some(libc::EINVAL) {
        return fd;
    }
    // SAFETY: as above.
    unsafe {
        libc::open(
            dir_path.as_ptr(),
            libc::O_RDONLY | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC,
        )
    }
}

/// Whether a READ on this root would be waiting on a lock THIS THREAD holds.
///
/// `flock` locks belong to an open file DESCRIPTION, so the shared lock
/// [`WorkspaceLock::acquire_read`] opens contends with the exclusive one this
/// thread may already hold exactly as another process's would — and the only
/// thing that could release it is this thread, which would be parked in the
/// kernel. The wait is therefore not slow, it is permanent.
///
/// Pure, so it can be mutated and oracle-tested without a live registry. BOTH
/// shapes of a same-thread entry are refused, and they are refused for
/// different reasons:
///
/// * a COMPLETED acquisition (`inner` is `Some`) deadlocks immediately and
///   deterministically — this is the carve-out the module docs named;
/// * this thread's own UNPROMOTED reservation means the read is running from
///   INSIDE this thread's in-flight `acquire` (an `interrupted` predicate, a
///   `tracing` subscriber the contention warn reaches). That read SUCCEEDS
///   today and moves the deadlock one step later, onto the outer acquire's
///   `LOCK_EX` against the shared lock its own thread now holds — but only if
///   the guard outlives the acquire, which cannot be known here. "Refused
///   read the caller restructures" and "daemon thread parked forever" are not
///   comparable costs.
///
/// The OWNER check is the whole of the discrimination: an entry owned by
/// ANOTHER thread is ordinary contention, which must WAIT and then succeed.
///
/// "Owner" is the thread that REGISTERED the acquisition, which is the one
/// that will release it because [`WorkspaceLock`] is `!Send` by construction —
/// see its `_thread_bound` field. Without that the two could differ and this
/// check would be keyed on the wrong thread in both directions.
#[cfg(unix)]
fn read_waits_on_this_thread(entry: Option<&RegistryEntry>, reader: ThreadId) -> bool {
    entry.is_some_and(|entry| entry.owner == reader)
}

/// Removes this thread's unpromoted reservation unless it is disarmed.
///
/// RAII rather than a removal at each exit, because the exits are not all
/// visible: `interrupted()` is caller-supplied code called while the
/// reservation is outstanding, and a panic unwinding out of it skips every
/// hand-rolled cleanup, leaving the workspace un-acquirable for the life of
/// the process. Inert for the CLI (its predicate is an atomic load) and for a
/// panic that aborts; `cerulion-wsd` is long-lived and shares this primitive.
///
/// The `Drop` must never panic: it can run DURING an unwind, where a second
/// panic aborts the process. So a poisoned registry mutex is recovered rather
/// than propagated — a cleanup that cannot run is worse than a poisoned map.
#[cfg(unix)]
struct ReservationGuard<'a> {
    root: &'a PathBuf,
    owner: ThreadId,
    armed: bool,
}

#[cfg(unix)]
impl ReservationGuard<'_> {
    /// The acquisition completed and the entry now carries a live `inner`;
    /// removing it here would be removing a real lock.
    fn disarm(&mut self) {
        self.armed = false;
    }
}

#[cfg(unix)]
impl Drop for ReservationGuard<'_> {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }
        let Some((registry, wake)) = REGISTRY.get() else {
            return;
        };
        let mut entries = registry.lock().unwrap_or_else(|e| e.into_inner());
        if reservation_is_ours(entries.get(self.root), self.owner) {
            entries.remove(self.root);
        }
        drop(entries);
        wake.notify_all();
    }
}

/// Why an acquire did not yield a lock.
///
/// Deliberately NOT `CliResult`, and deliberately WITHOUT a
/// `From<AcquireError> for CliError`: the whole point is that `?` must not
/// compile here. The predecessor returned `CliResult<Option<Self>>`, where
/// `Ok(None)` meant "your interrupt predicate ended the wait" — visible at the
/// call site, but bindable and ignorable, which is the same class of mistake
/// the workspace lock exists to prevent. An exhaustive match on this enum
/// cannot fall through to running unlocked, and each arm carries the CALLER's
/// own vocabulary rather than a generic one minted here.
///
/// It carries `Display` and `source` (but NOT `From<AcquireError> for
/// CliError`) because without them the cheapest thing a caller can reach for
/// is `.expect(..)` on a `Debug` value — the exact pressure the type exists to
/// remove. `?` stays closed: `CliError` has no blanket `From<E: Error>`.
#[derive(Debug, thiserror::Error)]
pub enum AcquireError {
    /// The caller's `interrupted` predicate returned true while waiting, so no
    /// lock was taken and the caller should refuse in its own words.
    ///
    /// No TRACKED file was written. Whether the UNTRACKED `.cerulion/` and its
    /// lock file exist depends on WHERE the interrupt landed, and there are
    /// three sites: the registry park (a same-process peer holds the lock) is
    /// reached BEFORE this call creates anything, while both kernel-`flock`
    /// checks are reached after — and the kernel one is the wait a
    /// single-threaded CLI writer meets, having no peer thread to queue
    /// behind. So, like [`AcquireError::Failed`], treat the directory as
    /// possibly created.
    #[error(
        "the wait for the workspace lock was ended by the caller's interrupt \
         predicate before the lock was taken"
    )]
    Interrupted,
    /// The acquire itself failed on something about the WORKSPACE (permissions,
    /// a symlinked `.cerulion`, a filesystem that cannot lock). The lock was
    /// not taken, though the attempt may have created the untracked lock
    /// directory.
    ///
    /// Distinct from [`AcquireError::Internal`] because the two have different
    /// remedies and the caller writes the remedy: this one is a path the user
    /// can fix.
    #[error("{0}")]
    Failed(#[source] CliError),
    /// The acquire refused because of a BUG IN THIS CRATE, not because of
    /// anything about the caller's workspace — this thread re-entered its own
    /// in-flight acquire, or the lock's own registry reservation vanished
    /// mid-acquire. Nothing was locked and no tracked file was written.
    ///
    /// It is a separate variant rather than another `Failed` because a caller
    /// that folds them together tells the user to go and inspect a `.cerulion`
    /// that is perfectly fine: the interpolated cause named
    /// the real reason, but the REMEDY beside it pointed at their filesystem.
    #[error("{0}")]
    Internal(#[source] CliError),
}

/// What is wrong with a path the workspace lock has to walk through.
///
/// Three conditions, three sentences. Each condition gets its own wording,
/// so a plain regular file at `<root>/.cerulion` is diagnosed on its own
/// terms, never as "a SYMLINK … its target lies outside the workspace".
#[cfg(unix)]
#[derive(Clone, Copy)]
enum LockPathProblem {
    /// A link. The lock is never taken through one.
    Symlink,
    /// Not a directory and NOT a link: a regular file, a fifo, a device.
    ///
    /// Which of these two a failed `O_DIRECTORY|O_NOFOLLOW` open means is NOT
    /// decidable from its errno — MEASURED, macOS reports a symlinked
    /// `.cerulion` as `ENOTDIR` where Linux reports `ELOOP` — so the call site
    /// refuses on the errno and then looks once more, by name, to choose
    /// between these. That look picks a sentence; it never picks the outcome.
    NotADirectory,
    /// A path the operating system cannot be handed at all (an interior NUL).
    /// Unreachable from `canonicalize()` + a `const` component; it exists so
    /// that if it ever fires it does not claim to be a symlink.
    Unusable,
}

/// The one refusal both acquire paths give for a lock path that is not what it
/// has to be.
#[cfg(unix)]
fn lock_path_refusal(what: &Path, problem: LockPathProblem) -> CliError {
    let (what_it_is, why) = match problem {
        LockPathProblem::Symlink => (
            "a SYMLINK",
            "the workspace lock is never taken through a link (its target lies \
             outside the workspace)",
        ),
        LockPathProblem::NotADirectory => ("NOT A DIRECTORY", "the workspace lock lives inside it"),
        LockPathProblem::Unusable => (
            "NOT A USABLE PATH",
            "it cannot be handed to the operating system (an interior NUL byte)",
        ),
    };
    CliError::Validation(format!(
        "'{}' is {what_it_is} — {why}. Move it aside and re-run.",
        what.display()
    ))
}

/// Which of the two path problems a failed `O_DIRECTORY|O_NOFOLLOW` open means.
///
/// The REFUSAL is the errno's business; the WORDING is not, because the errno
/// cannot say which condition this is. MEASURED on macOS: a symlinked
/// `.cerulion` under `O_DIRECTORY|O_NOFOLLOW` reports `ENOTDIR`, and so does a
/// DANGLING one. Linux is DOCUMENTED to report `ELOOP` for a trailing link
/// under `O_NOFOLLOW` and was not measured here — which is itself why the
/// sentence cannot ride the errno: one arm of the old split was an assumption.
///
/// So both acquire paths refuse on the errno and then take ONE more,
/// name-based look purely to choose the sentence. That look is not part of the
/// decision and cannot weaken it — the open has already failed and no lock is
/// taken. If the path VANISHED between the two the errno picks the wording; if
/// it changed into something else the second look wins and names the new thing
/// (a symlink swapped for a real directory is worded "NOT A DIRECTORY"). Both
/// are wording-only, on a path that is already refused.
#[cfg(unix)]
fn classify_lock_dir(lock_dir: &Path, error: &std::io::Error) -> LockPathProblem {
    match fs::symlink_metadata(lock_dir) {
        Ok(meta) if meta.file_type().is_symlink() => LockPathProblem::Symlink,
        Ok(_) => LockPathProblem::NotADirectory,
        Err(_) if error.raw_os_error() == Some(libc::ELOOP) => LockPathProblem::Symlink,
        Err(_) => LockPathProblem::NotADirectory,
    }
}

/// What a failed open of the lock FILE means on the READ path.
#[cfg(unix)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReadOpen {
    /// Nothing has ever mutated this workspace through the engine: readable,
    /// and the guard is simply unlocked.
    Unlocked,
    /// This process may not WRITE the file. A read-only open still takes a real
    /// shared lock.
    RetryReadOnly,
    /// A link. The lock is never taken through one.
    Symlink,
    /// Anything else is a real error.
    Fail,
}

/// The read path's errno table, as a pure function so it can be oracle-tested
/// and mutated.
///
/// It exists because the interesting mutation here is a NARROWING, not a
/// deletion: deleting the whole read-only retry is caught by a fixture, but
/// dropping one errno from it is not — `chmod` cannot produce `EPERM`, and the
/// alternatives (`chattr +i`) need root. As a table, dropping any entry fails a
/// named vector.
///
/// `EPERM` sits beside `EACCES` deliberately: `std` decodes BOTH to
/// `ErrorKind::PermissionDenied`, which is exactly what the path-based
/// predecessor matched, so listing only `EACCES` silently turned an immutable
/// or LSM-guarded lock file from "degrades to a real shared read" into "hard
/// error on every read verb".
#[cfg(unix)]
fn read_open_outcome(errno: Option<i32>) -> ReadOpen {
    match errno {
        Some(libc::ENOENT) => ReadOpen::Unlocked,
        Some(libc::ELOOP) => ReadOpen::Symlink,
        Some(libc::EACCES) | Some(libc::EPERM) | Some(libc::EROFS) => ReadOpen::RetryReadOnly,
        _ => ReadOpen::Fail,
    }
}

/// A lock-path I/O failure, carrying the path and the operation that failed.
///
/// The inner `io::Error`'s `Display` is a bare `"Permission denied (os error
/// 13)"`, and `CliError::Io` only prefixes it with `"I/O error: "` — neither
/// names the path or the operation.
/// `ros2 migrate` wraps that with the lock path at its own call site, but
/// `cerulion-wsd` forwards the string to its client verbatim — so the one
/// consumer that CANNOT add context was the one getting none. The `ErrorKind`
/// is preserved deliberately: `cerulion_wsd`'s `lock_error` keys its
/// `workspace_not_found` reply off `ErrorKind::NotFound`. What is NOT
/// preserved is `raw_os_error()` (nor a `source` chain): `io::Error::new`
/// mints a custom error. Nothing reads either off a `CliError` — this module's
/// own errno branches all run on the raw syscall error, before it reaches here.
#[cfg(unix)]
fn lock_io_refusal(what: &Path, doing: &str, error: std::io::Error) -> CliError {
    CliError::Io(std::io::Error::new(
        error.kind(),
        format!(
            "{doing} the workspace lock at '{}': {error}",
            what.display()
        ),
    ))
}

/// Owns the exclusive workspace mutation lock.
///
/// Dropping it RELEASES the lock, so a bare `WorkspaceLock::acquire(&root)?;`
/// in statement position would take the lock and give it back on the same
/// line. `clippy::let_underscore_lock` knows only std guard types, so nothing
/// else catches that.
///
/// Nested acquisition on the same thread and workspace reuses the existing
/// open file description. Acquisitions from another thread or process still
/// block on the kernel flock.
#[must_use = "dropping the guard releases the workspace lock immediately"]
pub struct WorkspaceLock {
    /// Makes the guard `!Send`, which is a CORRECTNESS requirement rather than
    /// tidiness: the reentrancy registry records the thread that REGISTERED the
    /// acquisition, and [`WorkspaceLock::acquire_read`]'s refusal keys on it.
    /// A guard moved to another thread would give the registering thread a
    /// false refusal on a read that should have waited, and the HOLDING thread
    /// a false allow — which parks it in `flock(LOCK_SH)` forever, the exact
    /// hang the self-wait refusal exists to prevent. `Arc<Inner>` is otherwise
    /// `Send + Sync`, so nothing else stops that; this does.
    ///
    /// Costs nothing where it is used: `cerulion-wsd` builds and drops its
    /// guard inside one `spawn_blocking` closure, so the CLOSURE is still
    /// `Send` — only the guard may not itself cross.
    _thread_bound: std::marker::PhantomData<*const ()>,
    #[cfg(unix)]
    inner: Arc<Inner>,
    /// Whether this guard opened `inner`'s file description or is reusing one.
    ///
    /// The guard stores the fact it OWNS and nothing else:
    /// [`WorkspaceLock::lock_dir_origin`] DERIVES the reported origin from this
    /// plus `Inner::created_lock_dir` through the one classifier, so there is
    /// no second copy of the answer that could disagree with it, and no way to
    /// build a guard whose reported origin the classifier would not have
    /// produced.
    #[cfg(unix)]
    nesting: Nesting,
}

/// Owns a shared workspace read lock when the workspace has a lock file.
///
/// On Unix the guard is unlocked in exactly ONE case: `<root>/.cerulion` or the
/// lock file inside it does not exist, because nothing has ever mutated this
/// workspace through the engine. (A non-Unix build takes no lock at all.) That is safe for the reason it sounds like —
/// no cooperating writer can hold a lock that does not exist.
///
/// Everything else is a real lock or a refusal. A checkout this reader cannot
/// WRITE still yields a genuine shared lock, opened read-only; any other open
/// failure is an `Err`, never a quietly unlocked guard: a writer can
/// absolutely hold a lock on a file you may not open.
///
/// Note what `is_locked()` does NOT promise even when true: it says a shared
/// lock was taken, not that the workspace will stay unmutated. A writer that
/// starts AFTER this guard exists blocks on it; one that started before is
/// what this guard waited for.
#[must_use = "dropping the guard releases the shared workspace lock immediately"]
pub struct WorkspaceReadLock {
    #[cfg(unix)]
    lock: Option<File>,
    #[cfg(unix)]
    root: PathBuf,
}

/// Append `.cerulion/` to an EXISTING `<root>/.gitignore` that lacks it.
///
/// Called only when [`WorkspaceLock::acquire_and_track_gitignore`] has just
/// CREATED the lock directory — `create_dir` returned `Ok` to THIS
/// acquisition, so at most one racer reaches here per creation — so a pre-lock
/// workspace gains the entry `workspace create` writes for new ones. A workspace without a `.gitignore` is left without one — the
/// engine never invents a file the user did not ask for. Idempotent: a second
/// call leaves the file byte-identical.
#[cfg(unix)]
fn ensure_gitignore_entry(root: &Path) -> std::io::Result<()> {
    let gitignore_path = root.join(".gitignore");
    let mut gitignore = match fs::read_to_string(&gitignore_path) {
        Ok(contents) => contents,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(()),
        Err(error) => return Err(error),
    };
    let entry = format!("{LOCK_DIR}/");
    if gitignore.lines().any(|line| line.trim() == entry) {
        return Ok(());
    }
    if !gitignore.is_empty() && !gitignore.ends_with('\n') {
        gitignore.push('\n');
    }
    gitignore.push_str(&entry);
    gitignore.push('\n');
    fs::write(&gitignore_path, gitignore)?;
    tracing::info!(
        path = %gitignore_path.display(),
        "added `.cerulion/` to the workspace .gitignore (the workspace lock lives there)"
    );
    Ok(())
}

/// Create `<root>/.cerulion` and, when the caller asked for the tracked write
/// AND THIS CALL created it, top up `<root>/.gitignore`.
///
/// Returns whether this call created the directory.
///
/// ONE function for both the first attempt and the recovery remake below,
/// because the two must not drift: a remake that re-creates the directory
/// but skips the top-up is exactly that drift, and with it
/// a tracking acquisition could end up holding a lock on a `.cerulion` that is
/// permanently absent from a tracked `.gitignore` — the forfeiture the
/// give-back exists to prevent, re-entered through the recovery. Whoever
/// creates the directory that SURVIVES owes the tracked write, however many
/// attempts it took.
#[cfg(unix)]
fn create_lock_dir_and_track(
    root: &Path,
    lock_dir: &Path,
    tracked: TrackedFilePolicy,
) -> CliResult<bool> {
    let mut created = false;
    // Test seam: fires immediately before the create, so a racer stands exactly
    // where one would land in production. It belongs HERE and not at the call
    // site: a `!is_dir()`-then-create shape has the
    // whole bug between those two statements — a seam above this function
    // fires before any such check and so reproduces nothing.
    #[cfg(test)]
    run_pre_create_hook();
    match fs::create_dir(lock_dir) {
        Ok(()) => created = true,
        // Somebody else got there first — this run included, on a
        // second acquisition. Not an error, and NOT a creation: the
        // walk below is what decides whether what is there is usable.
        Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {}
        Err(error) => {
            return Err(lock_io_refusal(
                lock_dir,
                "creating the directory of",
                error,
            ))
        }
    }
    // ONE decision point for the tracked write, and it is the same
    // classifier the guard reports: a variant that collapses the three
    // origins changes what lands in `.gitignore`, not just what an
    // accessor says.
    if tracked == TrackedFilePolicy::TopUpGitignore
        && lock_dir_origin(Nesting::Outermost, created) == LockDirOrigin::CreatedHere
    {
        // Named, because the caller asked for this write BY NAME and a
        // bare `CliError::Io` would say "Permission denied (os error
        // 13)" without saying which file or what was being done to it.
        if let Err(error) = ensure_gitignore_entry(root) {
            // GIVE THE DIRECTORY BACK before refusing. The top-up runs
            // exactly once per CREATION and this acquisition is the
            // creator, so leaving the directory behind would make every
            // later `acquire_and_track_gitignore` take the
            // `AlreadyExists` arm and never retry — turning one loud
            // refusal into a workspace whose `.cerulion/` stays
            // untracked FOREVER while every later verb succeeds
            // quietly. The directory is empty as far as THIS thread is
            // concerned (it has not opened the lock file yet) — a racer
            // that took the `AlreadyExists` arm may already have created
            // one, in which case the `rmdir` fails `ENOTEMPTY` and takes
            // the arm below, which is correct: a live lock is never
            // removed. So this is a plain `rmdir`; it
            // is best-effort because the refusal stands either way, and
            // a failure to give it back is exactly the "will not retry"
            // state, so it is logged rather than swallowed.
            // `warn!`, NOT `debug!`: `cerulion_core` enables `tracing`'s
            // `release_max_level_info`, so a `debug!` does not EXIST in a
            // shipped binary at any `RUST_LOG` — and the one-shot verbs
            // that take this constructor default to `warn` anyway. The
            // state being reported is permanent and otherwise invisible.
            // No `created = false` here, and that is deliberate
            // rather than an omission: this arm returns unconditionally
            // two statements down, so nothing ever reads the flag again.
            // Setting it here would be an assignment that SURVIVES
            // every test, which is what an inert
            // statement does, and the dead-code rule applies to a line
            // that merely looks careful as much as to one that looks
            // unused.
            let gave_back = match fs::remove_dir(lock_dir) {
                Ok(()) => true,
                Err(cleanup) => {
                    tracing::warn!(
                        path = %lock_dir.display(),
                        error = %cleanup,
                        "could not give back the lock directory after a failed \
                         .gitignore top-up; a later acquire will find it present and \
                         will NOT retry — add `.cerulion/` to the workspace .gitignore \
                         by hand"
                    );
                    false
                }
            };
            // The consequence rides the REFUSAL too: the log is the
            // wrong place for it alone, because this string is what the
            // user actually reads.
            let consequence = if gave_back {
                "nothing was locked; re-run once that path is writable and the entry \
                 will be added then"
            } else {
                "nothing was locked, and the untracked lock directory could not be \
                 removed — a later run will not retry the entry, so add `.cerulion/` \
                 to that file by hand"
            };
            return Err(CliError::Io(std::io::Error::new(
                error.kind(),
                format!(
                    "adding `{LOCK_DIR}/` to '{}': {error} — {consequence}",
                    root.join(".gitignore").display()
                ),
            )));
        }
    }
    Ok(created)
}

impl WorkspaceLock {
    /// Return the canonical workspace root protected by this lock.
    pub fn root(&self) -> &Path {
        #[cfg(unix)]
        {
            &self.inner.root
        }

        #[cfg(not(unix))]
        {
            Path::new("")
        }
    }

    /// Acquire the workspace lock, waiting for another writer to finish.
    ///
    /// Creates `<root>/.cerulion/workspace.lock` on first use and writes NO
    /// TRACKED file. If another PROCESS holds the lock this logs one `warn!`
    /// naming the lock file and then blocks until it is released; a
    /// same-thread nested acquire returns immediately.
    ///
    /// The DEFAULT name has no tracked-file side effect on purpose. Were it
    /// to top up `<root>/.gitignore`, the shortest constructor to
    /// write would also be the one that modifies a file the user's VCS tracks —
    /// invisible at every call site, and a `dry-run` contract away from being a
    /// bug. That side effect lives in
    /// [`WorkspaceLock::acquire_and_track_gitignore`], whose name says so.
    ///
    /// NO SHIPPED CALLER reaches this today: every workspace-owning verb takes
    /// the tracking constructor and `ros2 migrate` takes
    /// [`WorkspaceLock::acquire_interruptibly`]. It exists so that a future
    /// writer which must not touch a tracked file has a correctly-named door to
    /// reach for — the alternative being that it reaches for the tracking one
    /// because it is the shorter name.
    pub fn acquire(root: &Path) -> CliResult<Self> {
        #[cfg(unix)]
        {
            Self::acquire_blocking(root, TrackedFilePolicy::Leave)
        }

        #[cfg(not(unix))]
        {
            let _ = root;
            Ok(Self {
                _thread_bound: std::marker::PhantomData,
            })
        }
    }

    /// [`WorkspaceLock::acquire`], plus the one TRACKED-file side effect:
    /// when THIS acquisition creates `<root>/.cerulion/` it appends
    /// `.cerulion/` to an EXISTING `<root>/.gitignore` that lacks the entry.
    ///
    /// For the verbs that OWN `<root>/.cerulion` — `node`, `graph`, `schema`,
    /// `ros2 attach`, `graph partition`, and `cerulion-wsd`'s mutation
    /// dispatch. A workspace with no `.gitignore` gets none: the engine never
    /// invents a file the user did not ask for.
    ///
    /// Exactly one acquisition in the world runs the top-up per creation of the
    /// directory, because the creation test is `create_dir` returning `Ok` to
    /// THIS call rather than "the directory was absent a moment ago". A
    /// failure to write `.gitignore` refuses the acquire rather than being
    /// swallowed — the caller asked for the tracked write by name.
    ///
    /// On a non-Unix build this is a DECLARED-WEAKER no-op guard that locks
    /// nothing and writes nothing, exactly like [`WorkspaceLock::acquire`].
    pub fn acquire_and_track_gitignore(root: &Path) -> CliResult<Self> {
        #[cfg(unix)]
        {
            Self::acquire_blocking(root, TrackedFilePolicy::TopUpGitignore)
        }

        #[cfg(not(unix))]
        {
            let _ = root;
            Ok(Self {
                _thread_bound: std::marker::PhantomData,
            })
        }
    }

    /// The body both blocking constructors share, so the two differ in exactly
    /// the tracked-file policy and nothing else.
    ///
    /// It FLATTENS [`AcquireError`] into a [`CliError`], which is not a loss:
    /// both failure variants already say in words which they are, and a
    /// blocking caller has nowhere to put the distinction — it is the polling
    /// constructor's callers that write per-arm remedies.
    #[cfg(unix)]
    fn acquire_blocking(root: &Path, tracked: TrackedFilePolicy) -> CliResult<Self> {
        match Self::acquire_with(root, tracked, ContentionWait::Blocking) {
            Ok(lock) => Ok(lock),
            // Unreachable: only `ContentionWait::PollingUntil` can report an
            // interrupt. A refusal rather than a panic because `cerulion-wsd`
            // is long-lived and shares this primitive — a bug here should cost
            // one request, not the daemon.
            Err(AcquireError::Interrupted) => Err(CliError::Validation(
                "the workspace lock reported an interrupted wait to a caller that \
                 asked to BLOCK. This is a bug in cerulion_cli_engine, not a \
                 condition of your workspace; nothing was locked."
                    .to_string(),
            )),
            Err(AcquireError::Failed(error)) | Err(AcquireError::Internal(error)) => Err(error),
        }
    }

    /// Acquire the workspace lock with a wait the caller can END.
    ///
    /// Identical to [`WorkspaceLock::acquire`] in every respect that
    /// serializes — the same `<root>/.cerulion/workspace.lock`, the same
    /// exclusive `flock`, the same loud first warn, the same same-thread
    /// reentrancy, so the two exclude each other — and equally free of TRACKED
    /// writes. The ONE difference is the wait: `acquire` blocks in the kernel,
    /// which `SA_RESTART` makes uninterruptible, while this one POLLS,
    /// re-asking `interrupted` between attempts — on the kernel `flock` AND on
    /// the reentrancy registry's condvar, so a same-process peer's hold is
    /// interruptible too.
    ///
    /// The NAME is that difference, and nothing else. A name such as
    /// `acquire_without_tracked_writes` would fit only if `acquire` topped up
    /// `<root>/.gitignore` — with the tracked write living in
    /// [`WorkspaceLock::acquire_and_track_gitignore`], a name promising the
    /// ABSENCE of tracked writes would distinguish this variant from nothing at
    /// all, while implying the default one writes them.
    ///
    /// [`AcquireError::Interrupted`] means exactly one thing: `interrupted`
    /// returned true while waiting, so the lock was never taken and the caller
    /// should refuse in its own words. It is an error VARIANT rather than an
    /// `Ok(None)` because the predecessor's `Option` was visible but BINDABLE:
    /// `?` folded the error half away and left `None` to be matched or not.
    /// Stated at its real strength — `AcquireError` has no `From` into
    /// `CliError`, so `?` does not compile here and the case cannot be skipped
    /// silently; a caller who writes `.ok()` still gets an `Option`, and that
    /// is a deliberate act rather than the default one.
    ///
    /// [`AcquireError::Failed`] and [`AcquireError::Internal`] are separated
    /// for the caller's REMEDY: the first is a path the user can fix, the
    /// second is a bug in this crate and telling the user to go and inspect
    /// their `.cerulion` for it wastes their time.
    ///
    /// Use it from a writer that promises an interruptible wait (`cerulion
    /// ros2 migrate`: its acquire lands after the user has consented to
    /// apply). It still CREATES `.cerulion/` and the lock file, which are
    /// untracked.
    ///
    /// On a non-Unix build this is a DECLARED-WEAKER no-op guard that locks
    /// nothing, exactly like [`WorkspaceLock::acquire`].
    pub fn acquire_interruptibly(
        root: &Path,
        interrupted: &dyn Fn() -> bool,
    ) -> Result<Self, AcquireError> {
        #[cfg(unix)]
        {
            Self::acquire_with(
                root,
                TrackedFilePolicy::Leave,
                ContentionWait::PollingUntil(interrupted),
            )
        }

        #[cfg(not(unix))]
        {
            let _ = (root, interrupted);
            Ok(Self {
                _thread_bound: std::marker::PhantomData,
            })
        }
    }

    /// Where `<root>/.cerulion/` came from, as far as THIS guard is concerned.
    ///
    /// Three states, not a bool, because the one a bool loses is the one a
    /// top-up rule most needs: a NESTED acquire inside a run that created the
    /// directory reports [`LockDirOrigin::CreatedByAnOuterAcquisition`], which
    /// a bool would render as the same `false` it gives a directory somebody
    /// else created last week.
    ///
    /// Stated at its real strength: the ENGINE does not read this today — the
    /// top-up decides from the outermost half inside `acquire_with`, and no
    /// other caller consults it. It is an observable (Principle #3) and the
    /// shape a later rule needs, not a value the current code branches on.
    ///
    /// RACE-EXACT, not best-effort: [`LockDirOrigin::CreatedHere`] means this
    /// acquisition's own `create_dir` returned `Ok`. Two processes racing a
    /// fresh workspace can both see an absent directory and both succeed at
    /// `create_dir_all` — which is `Ok` for a directory that already exists —
    /// so both would report the creation and both run the `.gitignore` top-up,
    /// which happens before the lock file is opened and is therefore
    /// unserialised. `create_dir` hands `AlreadyExists` to every racer but one.
    /// See the module docs for what that race actually costs (content LOSS
    /// through a truncating rewrite — not a duplicated line, which this
    /// module's idempotence check makes unreachable).
    ///
    /// On a non-Unix build there is no lock directory at all, so the guard
    /// reports [`LockDirOrigin::PreExisting`] — it created nothing.
    pub fn lock_dir_origin(&self) -> LockDirOrigin {
        #[cfg(unix)]
        {
            lock_dir_origin(self.nesting, self.inner.created_lock_dir)
        }

        #[cfg(not(unix))]
        {
            LockDirOrigin::PreExisting
        }
    }

    /// [`AcquireError::Interrupted`] is reachable only under
    /// [`ContentionWait::PollingUntil`], and means its predicate ended the wait.
    ///
    /// It returns the PUBLIC failure type rather than a private one plus an
    /// `Option`: the interrupt is a way of not getting the lock, exactly like
    /// the other two, and an `Ok(None)` beside them is the bindable shape
    /// that the public surface deliberately does not offer.
    #[cfg(unix)]
    fn acquire_with(
        root: &Path,
        tracked: TrackedFilePolicy,
        wait: ContentionWait<'_>,
    ) -> Result<Self, AcquireError> {
        // Mapped rather than `?`d only because this fn does not return
        // `CliResult`; the refusal itself is the plain `CliError`.
        let root = root
            .canonicalize()
            .map_err(|error| AcquireError::Failed(CliError::from(error)))?;
        let (registry, wake) =
            REGISTRY.get_or_init(|| (Mutex::new(HashMap::new()), Condvar::new()));
        let owner = std::thread::current().id();
        // Every registry lock in this module RECOVERS a poisoned mutex rather
        // than propagating it. A panic anywhere under this mutex would
        // otherwise make every later acquire in the process panic too — the
        // workspace lock would be gone for the life of a `cerulion-wsd`, which
        // is a strictly worse outcome than operating on a map whose worst case
        // is a stale reservation the owner checks already reject.
        let mut entries = registry
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        let mut warned_same_process = false;
        while let Some(entry) = entries.get(&root) {
            if entry.owner == owner {
                if let Some(inner) = entry.inner.as_ref().and_then(Weak::upgrade) {
                    // Recovered rather than propagated: a poisoned depth would
                    // otherwise panic while the REGISTRY mutex is held, which
                    // poisons that too and wedges every later acquire in the
                    // process (`cerulion-wsd` included). The value is a count.
                    *inner
                        .depth
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner()) += 1;
                    return Ok(Self {
                        _thread_bound: std::marker::PhantomData,
                        inner,
                        nesting: Nesting::InsideAnOuterAcquisition,
                    });
                }
                if entry.inner.is_some() {
                    entries.remove(&root);
                    break;
                }
                // Our own UNPROMOTED reservation: this thread is already
                // inside its own acquire for this root, so something this
                // acquire CALLED came back in — an `interrupted` predicate on
                // the polling path, or any other caller code that runs while
                // the reservation is outstanding (a `tracing` subscriber
                // reached from the contention `warn!`, say, which is how
                // `acquire` can arrive here without having a predicate at
                // all). Waiting would park on a notify only this thread could
                // send (`Blocking`) or recurse without bound
                // (`PollingUntil`), so refuse instead of hanging. The
                // reservation is this thread's, and its own guard gives it back.
                return Err(AcquireError::Internal(CliError::Validation(format!(
                    "re-entered the workspace lock for '{}' from inside that acquire — code it \
                     called (an `interrupted` predicate, a `tracing` subscriber) took the \
                     workspace lock again. Nothing was locked.",
                    root.display()
                ))));
            }
            // A same-process peer is waited out HERE, on the condvar — the
            // kernel `flock` below is never reached — so a wait here with NO
            // signal of any kind, while the cross-process one warns, would be
            // an asymmetry facing the wrong way: `cerulion-wsd` is the one
            // multi-threaded consumer, so the in-process shape is its COMMON
            // case and must not be the silent one.
            //
            // Both arms follow the kernel arm's discipline exactly: ask the
            // caller's predicate FIRST, warn SECOND. A caller whose predicate
            // is already set never waits, and telling it "waiting for it to
            // finish" would be a line about a wait that never happened. The
            // loop is only entered when an entry EXISTS, so reaching here
            // already means real contention.
            //
            // The warn names the workspace ROOT, not the lock file: this wait
            // can be the FIRST contended acquire on a workspace, where the
            // file does not exist yet.
            //
            // NEITHER arm emits it while the registry mutex is HELD. A
            // `tracing` subscriber runs synchronously on the emitting thread
            // and is caller code: one that re-enters the workspace lock — for
            // ANY root, since this registry is process-global and its mutex is
            // not reentrant — would block on a mutex its own thread holds and
            // deadlock outright, never reaching `wake.wait` (which is what
            // would have released it). So each arm drops the guard, warns, and
            // re-takes it; the loop's own condition then re-checks, so a peer
            // that released in the gap costs nothing.
            match wait {
                ContentionWait::Blocking => {
                    if !warned_same_process {
                        warned_same_process = true;
                        drop(entries);
                        tracing::warn!(
                            workspace = %root.display(),
                            "workspace is locked by another request in this process — waiting for it to finish"
                        );
                        entries = registry
                            .lock()
                            .unwrap_or_else(|poisoned| poisoned.into_inner());
                        continue;
                    }
                    entries = wake
                        .wait(entries)
                        .unwrap_or_else(|poisoned| poisoned.into_inner());
                }
                ContentionWait::PollingUntil(interrupted) => {
                    // The predicate is CALLER code. Run it with the registry
                    // mutex RELEASED — a panic out of it under the lock would
                    // poison the map for every later acquire in the process
                    // (observed directly by
                    // `a_predicate_that_panics_in_the_registry_park_leaves_the_map_usable`,
                    // which asserts `Mutex::is_poisoned` rather than an outcome
                    // the poison recovery would absorb), and caller code
                    // holding this mutex also blocks every OTHER root's
                    // acquire and deadlocks outright if it takes the workspace
                    // lock itself.
                    drop(entries);
                    if interrupted() {
                        return Err(AcquireError::Interrupted);
                    }
                    if !warned_same_process {
                        warned_same_process = true;
                        tracing::warn!(
                            workspace = %root.display(),
                            "workspace is locked by another request in this process — waiting for it to finish"
                        );
                    }
                    let relocked = registry
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner());
                    // The peer may have released while the mutex was down;
                    // parking anyway would cost a whole CONTENTION_POLL on a
                    // lock that is already free.
                    entries = if relocked.contains_key(&root) {
                        // `wait_timeout` bounds the park at the same cadence
                        // the cross-process poll uses, so one constant governs
                        // how promptly a Ctrl-C is felt either way.
                        let (next, _timed_out) = wake
                            .wait_timeout(relocked, CONTENTION_POLL)
                            .unwrap_or_else(|poisoned| poisoned.into_inner());
                        next
                    } else {
                        relocked
                    };
                }
            }
        }
        entries.insert(root.clone(), RegistryEntry { owner, inner: None });
        drop(entries);
        // From here to the promotion below, every exit — including a panic
        // unwinding out of the caller's `interrupted()` — must give the
        // reservation back.
        let mut reservation = ReservationGuard {
            root: &root,
            owner,
            armed: true,
        };

        let lock_dir = root.join(LOCK_DIR);
        let lock_path = lock_dir.join(LOCK_FILE);
        let mut created_lock_dir = false;
        let lock = match (|| -> CliResult<Option<File>> {
            // The lock directory is reached WITHOUT following a link. A
            // planted `<root>/.cerulion -> elsewhere` would otherwise have
            // this create the lock file at a path of somebody else's choosing
            // and flock a file that is not this workspace's — the shape
            // `ros2_migrate` already refuses for its manifest and its patch,
            // and since `ros2 migrate --write` takes this lock BEFORE its
            // first anchored write, the refusal has to live here.
            //
            // This metadata check is the cheap, early refusal with a good
            // message. It is NOT the guarantee — a path check and a later
            // path-based open are two operations, and a `.cerulion` swapped
            // for a symlink in between would have the open land on an outside
            // inode, so two writers could hold DIFFERENT locks while mutating
            // one workspace. The guarantee is the descriptor walk below.
            match fs::symlink_metadata(&lock_dir) {
                Ok(meta) if meta.file_type().is_symlink() => {
                    return Err(lock_path_refusal(&lock_dir, LockPathProblem::Symlink));
                }
                // It exists and is neither a link nor a directory — a regular
                // file, a fifo, a device. `create_dir_all` below would refuse
                // it too, but with a bare `AlreadyExists` that names neither
                // the path nor what is wrong with it; the descriptor walk's
                // own `ENOTDIR` arm is never reached because the create fails
                // first. Say it here, in the same words the read path uses.
                Ok(meta) if !meta.is_dir() => {
                    return Err(lock_path_refusal(&lock_dir, LockPathProblem::NotADirectory));
                }
                _ => {}
            }
            // `create_dir`, NOT `create_dir_all`, for two independent reasons.
            //
            // (1) The top-up's exactly-once property. `create_dir_all` returns
            // `Ok` for a directory that already exists, so two processes racing
            // a fresh workspace both believed they created it and BOTH ran the
            // tracked-file top-up below — which happens before the lock file is
            // opened and flocked, so nothing serialises it. The cost is CONTENT
            // LOSS, not a duplicated line: `ensure_gitignore_entry` rewrites
            // the whole file with `fs::write`, which truncates, so a racer
            // reading inside that window writes back a file missing the other's
            // content. `create_dir` gives exactly one racer `Ok`.
            //
            // (2) `<root>` was canonicalized above, so it existed then — but if
            // it is deleted in between, `create_dir_all` would silently REBUILD
            // the workspace root under a path nothing owns any more, while
            // `create_dir` returns `ENOENT` and refuses. There was never a
            // missing ancestor for `create_dir_all` to be covering.
            // ONE retry span covering create -> open the directory -> open the
            // file, and the span is the point.
            //
            // Two independent retry loops that freeze the
            // directory descriptor between them miss a case:
            // the `.gitignore` give-back's `remove_dir` can land
            // AFTER a peer opened its directory descriptor and BEFORE its
            // `openat`, and an `openat` through a descriptor whose directory has
            // been removed fails `ENOENT` UNCONDITIONALLY — MEASURED:
            // three attempts through the frozen descriptor, all `ENOENT`,
            // while remaking the directory and RE-DERIVING the descriptor
            // succeeds. So a file-open retry that reuses the same descriptor
            // retries a corpse, and `workspace_not_found` stays reachable one
            // step past the directory open.
            //
            // Retrying the whole span re-derives the descriptor by construction,
            // and one counter bounds the lot. Every `ENOENT` inside the span is
            // transient by the same argument: this attempt has just established
            // the directory exists, so absence means somebody removed it, and
            // the only remover is this module's own give-back.
            let dir_path = CString::new(lock_dir.as_os_str().as_bytes())
                .map_err(|_| lock_path_refusal(&lock_dir, LockPathProblem::Unusable))?;
            let file_name = CString::new(LOCK_FILE).expect("LOCK_FILE has no NUL");
            let mut attempt = 0;
            let (dir, fd) = loop {
                attempt += 1;
                let exhausted = attempt >= LOCK_OPEN_ATTEMPTS;

                if create_lock_dir_and_track(&root, &lock_dir, tracked)? {
                    created_lock_dir = true;
                }

                #[cfg(test)]
                run_pre_open_hook();

                // THE GUARANTEE for `<root>/.cerulion` and the lock file inside
                // it — NOT for `root`'s own ancestors, which were resolved by
                // the `canonicalize` above and are not re-anchored here. That is
                // the scope the threat model needs: the planted component is the
                // one a workspace's own contents can introduce.
                //
                // The reason this is not two path opens: open `.cerulion` itself
                // with `O_DIRECTORY|O_NOFOLLOW`, then open the lock file
                // RELATIVE TO THAT DESCRIPTOR. A link swapped in after this
                // point retargets a name, not a descriptor, so the file we flock
                // is the one inside the directory we verified — whatever happens
                // to the path afterwards. `ros2_migrate`'s `anchored` module
                // makes the same move for the manifest and the patch.
                let dirfd = open_lock_dir(&dir_path);
                if dirfd < 0 {
                    let error = std::io::Error::last_os_error();
                    if matches!(
                        error.raw_os_error(),
                        Some(libc::ELOOP) | Some(libc::ENOTDIR)
                    ) {
                        return Err(lock_path_refusal(
                            &lock_dir,
                            classify_lock_dir(&lock_dir, &error),
                        ));
                    }
                    if error.raw_os_error() == Some(libc::ENOENT) && !exhausted {
                        continue;
                    }
                    return Err(lock_io_refusal(
                        &lock_dir,
                        "opening the directory of",
                        error,
                    ));
                }
                // SAFETY: `dirfd` is a fresh, valid descriptor this call owns.
                let dir = unsafe { OwnedFd::from_raw_fd(dirfd) };

                // `openat` with `O_CREAT` also returns a SPURIOUS `ENOENT` on
                // macOS/APFS while a peer process is creating that same
                // directory — the shape this lock is FOR. MEASURED on macOS
                // with a C model of exactly this syscall sequence, children
                // released together by a gate file: 12 of 50 children failed
                // with two racers, 41 of 100 with four and 73 of 200 with eight
                // on a single attempt, and 0 of 160 with a second. Left
                // unretried it is worse than a wrong message: `lock_io_refusal`
                // preserves `ErrorKind`, so the refusal is `Io(NotFound)`, and
                // `cerulion_wsd`'s `lock_error` keys `workspace_not_found` off
                // exactly that.
                //
                // Every OTHER errno — `ELOOP`, `EACCES`, `ENOTDIR` — returns on
                // the first attempt.
                match open_lock_file(&dir, &file_name) {
                    Ok(fd) => break (dir, fd),
                    Err(error) => {
                        if error.raw_os_error() == Some(libc::ELOOP) {
                            return Err(lock_path_refusal(&lock_path, LockPathProblem::Symlink));
                        }
                        if error.raw_os_error() == Some(libc::ENOENT) && !exhausted {
                            continue;
                        }
                        return Err(lock_io_refusal(&lock_path, "opening", error));
                    }
                }
            };
            // `dir` is held only so the descriptor the file was opened through
            // outlives that open; nothing below reads it.
            drop(dir);
            // SAFETY: `fd` is a fresh, valid descriptor this call owns.
            let lock = unsafe { File::from_raw_fd(fd) };
            // SAFETY: `lock` is a live, open file owned by this guard throughout this call.
            let result = unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) };
            if result == 0 {
                return Ok(Some(lock));
            }
            let error = std::io::Error::last_os_error();
            if error.raw_os_error() != Some(libc::EWOULDBLOCK) {
                return Err(lock_io_refusal(&lock_path, "locking", error));
            }
            // The warn lives in each arm, not above the match: a caller whose
            // predicate is ALREADY set never waits, and telling it "waiting for
            // it to finish" would be a line about a wait that never happened.
            match wait {
                ContentionWait::Blocking => {
                    tracing::warn!(
                        lock = %lock_path.display(),
                        "workspace is locked by another process (a `cerulion` command or `cerulion-wsd` mid-mutation) — waiting for it to finish"
                    );
                    // SAFETY: `lock` stays open for the whole blocking wait.
                    if let Some(error) = unsafe { flock_blocking(lock.as_raw_fd(), libc::LOCK_EX) }
                    {
                        return Err(lock_io_refusal(&lock_path, "locking", error));
                    }
                    Ok(Some(lock))
                }
                ContentionWait::PollingUntil(interrupted) => {
                    if interrupted() {
                        return Ok(None);
                    }
                    tracing::warn!(
                        lock = %lock_path.display(),
                        "workspace is locked by another process (a `cerulion` command or `cerulion-wsd` mid-mutation) — waiting for it to finish"
                    );
                    loop {
                        std::thread::sleep(CONTENTION_POLL);
                        // SAFETY: as above.
                        let result =
                            unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) };
                        if result == 0 {
                            return Ok(Some(lock));
                        }
                        let error = std::io::Error::last_os_error();
                        if error.raw_os_error() != Some(libc::EWOULDBLOCK) {
                            return Err(lock_io_refusal(&lock_path, "locking", error));
                        }
                        if interrupted() {
                            return Ok(None);
                        }
                    }
                }
            }
        })() {
            Ok(Some(lock)) => lock,
            outcome => {
                // Either the wait was interrupted (`Ok(None)`) or the acquire
                // failed. `reservation` gives the entry back on the way out, so
                // there is nothing to undo by hand here. Everything the closure
                // can fail on is a condition of the WORKSPACE — the two
                // internal-bug refusals are minted outside it.
                return match outcome {
                    Ok(_) => Err(AcquireError::Interrupted),
                    Err(error) => Err(AcquireError::Failed(error)),
                };
            }
        };

        let inner = Arc::new(Inner {
            lock,
            depth: Mutex::new(1),
            root: root.clone(),
            created_lock_dir,
        });
        let mut entries = registry
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        let Some(entry) = entries.get_mut(&root) else {
            // Unreachable — `retain` in `Drop` preserves every reservation
            // (`None => true`) and `reservation_is_ours` gates every other
            // removal on the owner. A refusal rather than a panic because this
            // runs with the registry mutex HELD, and a panic there poisons the
            // map for every later acquire in the process. The guard gives the
            // reservation back on the way out, and dropping `inner` closes the
            // descriptor, which releases the kernel lock.
            return Err(AcquireError::Internal(CliError::Validation(format!(
                "the workspace lock's own reservation for '{}' disappeared mid-acquire. This \
                 is a bug in cerulion_cli_engine, not a condition of your workspace; nothing \
                 was locked.",
                root.display()
            ))));
        };
        entry.inner = Some(Arc::downgrade(&inner));
        reservation.disarm();
        Ok(Self {
            _thread_bound: std::marker::PhantomData,
            inner,
            nesting: Nesting::Outermost,
        })
    }

    /// Acquire a shared lock for a workspace read.
    ///
    /// Never creates the lock directory or file. When
    /// `<root>/.cerulion/workspace.lock` exists and can be opened the guard
    /// holds a shared `flock`, so a read cannot interleave with a writer's
    /// commit; when it does not exist the guard is unlocked and the read
    /// proceeds. A checkout this process cannot WRITE still takes a real
    /// shared lock, opened read-only.
    ///
    /// The wait is UNINTERRUPTIBLE and unbounded. After one `warn!` the shared
    /// `flock` blocks in the kernel for as long as the writer holds it, and
    /// `ctrlc` installs its handler with `SA_RESTART`, so no ordinary signal
    /// ends it — there is no polling variant of this call. `cerulion-wsd`
    /// serves reads on a blocking-pool thread, so one is parked for the length
    /// of a whole `ros2 migrate --write`.
    ///
    /// NOT reentrant with [`WorkspaceLock::acquire`]: this opens its own file
    /// description, and `flock` descriptions conflict within one process just
    /// as they do across processes, so a thread that already holds the
    /// exclusive lock on a root would wait on itself. That case is
    /// REFUSED, by name, before anything is opened — otherwise it would park in
    /// the kernel forever after one `warn!`. The refusal reads
    /// as the internal bug it is; no shipped caller reaches it, because the
    /// `cerulion-wsd` dispatch picks exactly one of the two.
    ///
    /// Refused in BOTH shapes: a completed acquisition on this thread, and this
    /// thread's own in-flight acquire (a read taken from an `interrupted`
    /// predicate or a `tracing` subscriber). The second succeeds today and
    /// moves the deadlock onto the outer acquire; see
    /// `read_waits_on_this_thread` for why both are refused rather than one.
    /// Contention from another THREAD or another PROCESS is untouched: it
    /// still warns once and waits.
    ///
    /// The descriptor walk opens `.cerulion` with the platform's SEARCH-ONLY
    /// flag (`LOCK_DIR_OPEN_FLAGS`) rather than for reading, so a `.cerulion`
    /// at mode `0311` — which the path-based open handled and
    /// `O_DIRECTORY|O_RDONLY` refused with `EACCES` — reads again.
    pub fn acquire_read(root: &Path) -> CliResult<WorkspaceReadLock> {
        #[cfg(unix)]
        {
            let root = root.canonicalize()?;
            // BEFORE anything is opened: a read that would wait on a lock this
            // very thread holds can never be woken. Refuse
            // it by name instead of parking in the kernel forever. Contention
            // from another THREAD or another PROCESS is not this case and
            // still waits — `read_waits_on_this_thread` keys on the OWNER.
            //
            // The registry is only consulted, never modified: a read takes no
            // reservation, so there is nothing here to give back and no new
            // way for this call to leak an entry.
            if let Some((registry, _)) = REGISTRY.get() {
                let reader = std::thread::current().id();
                let entries = registry
                    .lock()
                    .unwrap_or_else(|poisoned| poisoned.into_inner());
                let waits_on_us = read_waits_on_this_thread(entries.get(&root), reader);
                drop(entries);
                if waits_on_us {
                    return Err(CliError::Validation(format!(
                        "read-locking the workspace at '{}' from a thread that already holds — \
                         or is inside its own acquire of — that workspace's EXCLUSIVE lock. \
                         `flock` locks belong to an open file description, so this read would \
                         wait on a lock only this thread can release, forever. This is a bug \
                         in cerulion_cli_engine, not a condition of your workspace; nothing \
                         was locked. Please report it with the command you ran.",
                        root.display()
                    )));
                }
            }
            let lock_dir = root.join(LOCK_DIR);
            let lock_path = lock_dir.join(LOCK_FILE);
            // The SAME descriptor walk the exclusive path uses, with one
            // difference that runs all the way through it: a READ creates
            // nothing, so every "it is not there" is an unlocked guard rather
            // than an error. What is NOT softened is a LINK: a reader that
            // followed `<root>/.cerulion -> elsewhere` would take a shared lock
            // on a file that is not this workspace's and then report the
            // workspace as safely readable, which is worse than refusing.
            let dir_path = CString::new(lock_dir.as_os_str().as_bytes())
                .map_err(|_| lock_path_refusal(&lock_dir, LockPathProblem::Unusable))?;
            let dirfd = open_lock_dir(&dir_path);
            if dirfd < 0 {
                let error = std::io::Error::last_os_error();
                return match error.raw_os_error() {
                    // Nothing has ever mutated this workspace through the
                    // engine: readable, unlocked, exactly as before.
                    //
                    // NOT the same class as the exclusive path's `ENOENT`, and
                    // deliberately not retried: there, this call had just
                    // created the directory or been told it existed, so absence
                    // was impossible and meant a racer had removed it. HERE a
                    // read creates nothing, so absence is the ordinary answer
                    // for a workspace no writer has touched. Retrying it by
                    // symmetry would buy four syscalls on the commonest read
                    // there is.
                    Some(libc::ENOENT) => Ok(WorkspaceReadLock { lock: None, root }),
                    // Refuse on the errno, then take ONE name-based look purely
                    // to choose the sentence — the errno cannot say which
                    // condition this is (MEASURED: macOS reports a symlinked
                    // `.cerulion` as ENOTDIR under `O_DIRECTORY` where Linux
                    // reports ELOOP), and calling a plain regular file "a
                    // SYMLINK whose target lies outside the workspace" names a
                    // cause that does not exist.
                    Some(libc::ELOOP) | Some(libc::ENOTDIR) => Err(lock_path_refusal(
                        &lock_dir,
                        classify_lock_dir(&lock_dir, &error),
                    )),
                    _ => Err(lock_io_refusal(
                        &lock_dir,
                        "opening the directory of",
                        error,
                    )),
                };
            }
            // SAFETY: `dirfd` is a fresh descriptor this call owns.
            let dir = unsafe { OwnedFd::from_raw_fd(dirfd) };
            let file_name = CString::new(LOCK_FILE).expect("LOCK_FILE has no NUL");
            let open_at = |flags: libc::c_int| {
                // SAFETY: `dir` is live for the call and `file_name` is NUL-terminated.
                unsafe {
                    libc::openat(
                        dir.as_raw_fd(),
                        file_name.as_ptr(),
                        flags | libc::O_NOFOLLOW | libc::O_CLOEXEC,
                    )
                }
            };
            // Never `O_CREAT`: a read leaves no trace in a workspace.
            let mut fd = open_at(libc::O_RDWR);
            if fd < 0 {
                let error = std::io::Error::last_os_error();
                match read_open_outcome(error.raw_os_error()) {
                    ReadOpen::Unlocked => return Ok(WorkspaceReadLock { lock: None, root }),
                    ReadOpen::Symlink => {
                        return Err(lock_path_refusal(&lock_path, LockPathProblem::Symlink))
                    }
                    ReadOpen::RetryReadOnly => {
                        fd = open_at(libc::O_RDONLY);
                        if fd < 0 {
                            let retry = std::io::Error::last_os_error();
                            return match read_open_outcome(retry.raw_os_error()) {
                                ReadOpen::Unlocked => Ok(WorkspaceReadLock { lock: None, root }),
                                ReadOpen::Symlink => {
                                    Err(lock_path_refusal(&lock_path, LockPathProblem::Symlink))
                                }
                                // The read-only open IS the fallback, so a
                                // second permission failure has nothing left to
                                // try.
                                _ => Err(lock_io_refusal(&lock_path, "opening", retry)),
                            };
                        }
                    }
                    ReadOpen::Fail => return Err(lock_io_refusal(&lock_path, "opening", error)),
                }
            }
            // SAFETY: `fd` is a fresh descriptor this call owns.
            let lock = unsafe { File::from_raw_fd(fd) };
            // Contention is LOUD here too. This wait blocks in the kernel for
            // as long as the writer holds the lock, and `cerulion-wsd` serves a
            // read on a blocking-pool thread — so a read concurrent with a
            // `ros2 migrate --write` parked one of those threads for the whole
            // migration and said NOTHING, while the exclusive path three
            // screens up goes to real trouble to try `LOCK_NB` first and name
            // the file. One line per contended read, same as there.
            //
            // SAFETY: `lock` is a live, open file owned by this guard throughout this call.
            let probed = unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_SH | libc::LOCK_NB) };
            if probed != 0 {
                let error = std::io::Error::last_os_error();
                if error.raw_os_error() != Some(libc::EWOULDBLOCK) {
                    return Err(lock_io_refusal(&lock_path, "read-locking", error));
                }
                // Every case that REACHES this line is a genuine wait: a peer
                // THREAD of this process (the registry knew, and waiting is the
                // right answer) or another process. The same-thread case is
                // refused ~100 lines above, so this warn need not hedge about
                // it: it cannot get here.
                tracing::warn!(
                    lock = %lock_path.display(),
                    "workspace is being mutated (a `cerulion` command, `cerulion-wsd`, or another request in this process holds the write lock) — waiting to read it"
                );
                // SAFETY: as above — `lock` stays open for the whole wait.
                // Through `flock_blocking`, like the exclusive path: this is
                // the LONGEST blocking wait in the module (a read concurrent
                // with a `ros2 migrate --write` parks for the whole migration),
                // so it is the one with the largest `EINTR` window, and an
                // unretried `EINTR` here refuses a read that should have waited.
                if let Some(error) = unsafe { flock_blocking(lock.as_raw_fd(), libc::LOCK_SH) } {
                    return Err(lock_io_refusal(&lock_path, "read-locking", error));
                }
            }
            Ok(WorkspaceReadLock {
                lock: Some(lock),
                root,
            })
        }

        #[cfg(not(unix))]
        {
            let _ = root;
            Ok(WorkspaceReadLock {})
        }
    }
}

impl WorkspaceReadLock {
    /// Return the canonical workspace root protected by this lock.
    pub fn root(&self) -> &Path {
        #[cfg(unix)]
        {
            &self.root
        }

        #[cfg(not(unix))]
        {
            Path::new("")
        }
    }

    /// Whether this guard actually holds the shared kernel lock.
    ///
    /// On Unix, `false` means exactly one thing: `<root>/.cerulion` or its lock
    /// file does not exist. A read-only checkout is NOT that case — it takes a
    /// real shared lock through a read-only open — and every other failure is
    /// an `Err` rather than an unlocked guard. On a non-Unix build it is always
    /// `false`, because that guard locks nothing (see the module header).
    pub fn is_locked(&self) -> bool {
        #[cfg(unix)]
        {
            self.lock.is_some()
        }

        #[cfg(not(unix))]
        {
            false
        }
    }
}

#[cfg(unix)]
impl Drop for WorkspaceReadLock {
    fn drop(&mut self) {
        if let Some(lock) = self.lock.take() {
            // SAFETY: `lock` remains open until this method returns. The
            // result is discarded because `lock` is closed on the next line
            // and closing a descriptor releases its `flock` unconditionally —
            // a failure here leaves nothing to repair, and a `Drop` must not
            // panic.
            let _ = unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_UN) };
        }
    }
}

#[cfg(unix)]
impl Drop for WorkspaceLock {
    fn drop(&mut self) {
        // A `Drop` must never panic: it can run DURING an unwind, where a
        // second panic ABORTS the process — and this is the guard held across
        // arbitrary caller code, so it is the one that runs during an unwind
        // most often. Same rule, and the same recovery, as `ReservationGuard`.
        let mut depth = self
            .inner
            .depth
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        *depth -= 1;
        if *depth != 0 {
            return;
        }
        drop(depth);

        // The kernel lock and the registry entry go under ONE critical
        // section, so nothing ever observes half of a release. Either order
        // ALONE is wrong, in opposite directions:
        //
        // * entry first, lock second — publishes "this workspace is free"
        //   while it is not. A peer thread waking in that window finds no
        //   entry, falls through to the kernel `flock`, gets EWOULDBLOCK and
        //   reports "locked by another PROCESS" for a peer in its own; it then
        //   sleeps a full CONTENTION_POLL, and a Ctrl-C landing there refuses
        //   a lock that was already free.
        // * lock first, entry second — the inverse: the entry still upgrades
        //   (this `Arc<Inner>` is alive until `drop` returns), so the reuse
        //   arm above would hand out a `WorkspaceLock` for a workspace this
        //   process no longer flocks, whose own `Drop` then unlocks a lock it
        //   never took.
        //
        // Under the mutex the invariant is simply: an entry that upgrades
        // means the kernel lock is held. `LOCK_UN` does not block, so holding
        // the mutex across it costs nothing.
        let unlock = |inner: &Arc<Inner>| {
            // SAFETY: `inner.lock` stays open for the whole call — this is the
            // last `Arc`, and it outlives the closure. The result is discarded
            // because there is nothing to do about it and a `Drop` must not
            // panic: the descriptor closes moments later, which releases the
            // lock unconditionally. (A failure here does not preserve the
            // ordering above — it defers the release to that close — but there
            // is no second way to unlock.)
            let _ = unsafe { libc::flock(inner.lock.as_raw_fd(), libc::LOCK_UN) };
        };
        match REGISTRY.get() {
            Some((registry, wake)) => {
                let mut entries = registry
                    .lock()
                    .unwrap_or_else(|poisoned| poisoned.into_inner());
                unlock(&self.inner);
                entries.retain(|_, entry| match &entry.inner {
                    None => true,
                    Some(inner) => inner
                        .upgrade()
                        .is_some_and(|inner| !Arc::ptr_eq(&inner, &self.inner)),
                });
                drop(entries);
                wake.notify_all();
            }
            None => unlock(&self.inner),
        }
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::{
        fail_next_lock_opens_with_enoent, lock_dir_origin, read_waits_on_this_thread,
        reservation_is_ours, set_pre_create_hook, set_pre_file_open_hook, set_pre_open_hook,
        CliError, CliResult, Inner, LockDirOrigin, Nesting, RegistryEntry, WorkspaceLock, LOCK_DIR,
        LOCK_DIR_OPEN_FLAGS, LOCK_FILE,
    };
    use std::os::fd::AsRawFd;
    use std::os::unix::ffi::OsStrExt;
    use std::os::unix::fs::PermissionsExt;
    use std::path::Path;
    use std::sync::mpsc;
    use std::sync::Weak;
    use std::time::Duration;

    /// `expect_err` would need `Debug` on the Ok type, and the guard
    /// deliberately has none — a lock is not a value to print.
    fn refusal_text<T>(outcome: CliResult<T>, what: &str) -> String {
        match outcome {
            Ok(_) => panic!("{what} must refuse"),
            Err(error) => error.to_string(),
        }
    }

    /// Nothing is interrupting these tests.
    fn never_interrupted() -> &'static dyn Fn() -> bool {
        &|| false
    }

    /// Take the INTERRUPTIBLE lock, asserting it was actually taken.
    fn acquire_quiet(root: &Path) -> WorkspaceLock {
        WorkspaceLock::acquire_interruptibly(root, never_interrupted())
            .expect("an uninterrupted acquire always yields the lock")
    }

    /// Open `<root>/.cerulion/workspace.lock` as a FOREIGN open file
    /// description. `flock(2)` locks belong to a description, so this contends
    /// exactly as another PROCESS would — and unlike a second `WorkspaceLock`
    /// on this thread, it is not reentrant, so it can observe whether the lock
    /// is really held and at what strength.
    fn foreign_description(root: &Path) -> std::fs::File {
        std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(root.join(LOCK_DIR).join(LOCK_FILE))
            .expect("the lock file exists")
    }

    /// A deadline the contention ARMS own themselves: every other test here is
    /// `recv_timeout`-bounded, but an arm that blocks the test thread inside an
    /// acquire would, if its release never runs, poll forever and HANG CI
    /// rather than fail. The predicate ends the wait, which turns that into a
    /// named failure at the arm's own `expect`.
    const POLL_DEADLINE: usize = 300;

    /// A thread-local `tracing` subscriber that WATCHES for one warn, counts
    /// it, and optionally re-enters the workspace lock from inside `event()`.
    ///
    /// It exists so a contention arm can gate its release on the warn HAVING
    /// FIRED rather than on a wall clock. The warn is emitted at the moment the
    /// acquire has provably reached contention — it is emitted only after a
    /// `LOCK_NB` probe failed, or with a registry entry in hand — which is
    /// exactly the readiness a fixed `sleep` was standing in for, and unlike a
    /// sleep it cannot be outrun by a loaded runner.
    ///
    /// `reenter` is for the deadlock arm: caller code that takes the workspace
    /// lock from inside a subscriber. The readiness signal is sent BEFORE that
    /// re-entry, so the arm still hands over its handshake in the very case
    /// where the re-entry never returns.
    struct WarnWatch {
        needle: &'static str,
        hits: std::sync::Arc<std::sync::atomic::AtomicUsize>,
        ready: std::sync::Mutex<Option<mpsc::Sender<()>>>,
        reenter: Option<std::path::PathBuf>,
    }

    impl tracing::Subscriber for WarnWatch {
        fn enabled(&self, _: &tracing::Metadata<'_>) -> bool {
            true
        }
        fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::span::Id {
            tracing::span::Id::from_u64(1)
        }
        fn record(&self, _: &tracing::span::Id, _: &tracing::span::Record<'_>) {}
        fn record_follows_from(&self, _: &tracing::span::Id, _: &tracing::span::Id) {}
        fn event(&self, event: &tracing::Event<'_>) {
            struct Find {
                needle: &'static str,
                found: bool,
            }
            impl tracing::field::Visit for Find {
                fn record_debug(
                    &mut self,
                    field: &tracing::field::Field,
                    value: &dyn std::fmt::Debug,
                ) {
                    if field.name() == "message" && format!("{value:?}").contains(self.needle) {
                        self.found = true;
                    }
                }
            }
            let mut find = Find {
                needle: self.needle,
                found: false,
            };
            event.record(&mut find);
            if !find.found {
                return;
            }
            if self.hits.fetch_add(1, std::sync::atomic::Ordering::SeqCst) != 0 {
                return;
            }
            // Hand over the handshake FIRST: a re-entry that deadlocks must
            // still have told the test it got this far.
            if let Some(tx) = self
                .ready
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner())
                .take()
            {
                tx.send(()).ok();
            }
            if let Some(root) = &self.reenter {
                drop(WorkspaceLock::acquire(root).expect("the subscriber's own acquire"));
            }
        }
        fn enter(&self, _: &tracing::span::Id) {}
        fn exit(&self, _: &tracing::span::Id) {}
    }

    /// Whether a foreign description can take `mode` right now (it is released
    /// again immediately, so the probe never changes what is held).
    fn foreign_can_take(f: &std::fs::File, mode: libc::c_int) -> bool {
        // SAFETY: `f` is a live open file owned by the caller across this call.
        if unsafe { libc::flock(f.as_raw_fd(), mode | libc::LOCK_NB) } == 0 {
            // SAFETY: as above.
            unsafe { libc::flock(f.as_raw_fd(), libc::LOCK_UN) };
            true
        } else {
            false
        }
    }

    #[test]
    fn nested_acquisition_on_one_thread_reuses_the_lock() {
        let temp = tempfile::tempdir().unwrap();
        let outer = WorkspaceLock::acquire(temp.path()).unwrap();
        let inner = WorkspaceLock::acquire(temp.path()).unwrap();
        drop(inner);
        drop(outer);
        // Both guards are gone, so the root is acquirable again. `drop(..)`
        // rather than a bare statement: the guard is `#[must_use]`, and a lock
        // taken and released on one line is the shape that attribute exists to
        // stop somebody writing by accident.
        drop(WorkspaceLock::acquire(temp.path()).unwrap());
    }

    /// The second thread announces itself BEFORE calling `acquire`, so the test
    /// can observe that it does NOT acquire while the first guard is held and
    /// DOES once it is dropped — a no-op lock fails the first observation.
    #[test]
    fn separate_threads_serialize_on_the_workspace_lock() {
        let temp = tempfile::tempdir().unwrap();
        let first = WorkspaceLock::acquire(temp.path()).unwrap();
        let (about_to_acquire_tx, about_to_acquire_rx) = mpsc::channel();
        let (acquired_tx, acquired_rx) = mpsc::channel();
        let root = temp.path().to_path_buf();
        let thread = std::thread::spawn(move || {
            about_to_acquire_tx.send(()).unwrap();
            let _lock = WorkspaceLock::acquire(&root).unwrap();
            acquired_tx.send(()).unwrap();
        });
        about_to_acquire_rx.recv().unwrap();
        assert!(
            acquired_rx
                .recv_timeout(Duration::from_millis(300))
                .is_err(),
            "the second thread acquired the lock while the first guard was still held"
        );
        drop(first);
        acquired_rx
            .recv_timeout(Duration::from_secs(10))
            .expect("the second thread never acquired after the first guard dropped");
        thread.join().unwrap();
    }

    /// `flock(2)` locks belong to an OPEN FILE DESCRIPTION, so a second `open()`
    /// of the lock file contends exactly like another process would. Holding an
    /// exclusive flock on such a description must block `acquire`, and releasing
    /// it must let `acquire` through — the cross-process half of the contract,
    /// pinned without spawning a process.
    #[test]
    fn the_lock_is_a_kernel_flock_that_a_foreign_description_contends() {
        let temp = tempfile::tempdir().unwrap();
        drop(WorkspaceLock::acquire(temp.path()).unwrap());
        let foreign = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(temp.path().join(LOCK_DIR).join(LOCK_FILE))
            .unwrap();
        // SAFETY: `foreign` is open for the whole test.
        assert_eq!(0, unsafe {
            libc::flock(foreign.as_raw_fd(), libc::LOCK_EX)
        });
        let (acquired_tx, acquired_rx) = mpsc::channel();
        let root = temp.path().to_path_buf();
        let thread = std::thread::spawn(move || {
            let _lock = WorkspaceLock::acquire(&root).unwrap();
            acquired_tx.send(()).unwrap();
        });
        assert!(
            acquired_rx
                .recv_timeout(Duration::from_millis(300))
                .is_err(),
            "acquire went through while a foreign flock held the file"
        );
        // SAFETY: as above.
        assert_eq!(0, unsafe {
            libc::flock(foreign.as_raw_fd(), libc::LOCK_UN)
        });
        acquired_rx
            .recv_timeout(Duration::from_secs(10))
            .expect("acquire never completed after the foreign flock was released");
        thread.join().unwrap();
    }

    #[test]
    fn a_read_never_creates_the_lock_directory() {
        let temp = tempfile::tempdir().unwrap();
        let guard = WorkspaceLock::acquire_read(temp.path()).unwrap();
        assert!(!guard.is_locked());
        assert!(!temp.path().join(LOCK_DIR).exists());
        drop(guard);
        drop(WorkspaceLock::acquire(temp.path()).unwrap());
        let guard = WorkspaceLock::acquire_read(temp.path()).unwrap();
        assert!(
            guard.is_locked(),
            "once the lock file exists a read holds the shared flock"
        );
    }

    #[test]
    fn the_first_mutation_tops_up_an_existing_gitignore_once() {
        let temp = tempfile::tempdir().unwrap();
        let gitignore = temp.path().join(".gitignore");
        std::fs::write(&gitignore, "target/").unwrap();
        drop(WorkspaceLock::acquire_and_track_gitignore(temp.path()).unwrap());
        assert_eq!(
            std::fs::read_to_string(&gitignore).unwrap(),
            "target/\n.cerulion/\n"
        );
        drop(WorkspaceLock::acquire_and_track_gitignore(temp.path()).unwrap());
        assert_eq!(
            std::fs::read_to_string(&gitignore).unwrap(),
            "target/\n.cerulion/\n",
            "a second acquire must leave .gitignore byte-identical"
        );
    }

    /// The THREE constructors on IDENTICAL fixtures, so the only variable is
    /// which one was called. The tracking control is what stops the other two
    /// assertions passing on a build where NOTHING tops up; the two silent
    /// arms are what stop a build where EVERYTHING does.
    ///
    /// The DEFAULT-named constructor is in here deliberately: since the
    /// tracked write moved to `acquire_and_track_gitignore` the thing most
    /// likely to regress is `acquire` quietly getting it back.
    #[test]
    fn only_the_named_tracking_constructor_tops_up_an_existing_gitignore() {
        for (label, take) in [
            (
                "acquire_interruptibly",
                (|root: &Path| {
                    drop(acquire_quiet(root));
                }) as fn(&Path),
            ),
            ("acquire", |root: &Path| {
                drop(WorkspaceLock::acquire(root).unwrap());
            }),
        ] {
            let untracked = tempfile::tempdir().unwrap();
            std::fs::write(untracked.path().join(".gitignore"), "target/").unwrap();
            take(untracked.path());
            assert_eq!(
                std::fs::read_to_string(untracked.path().join(".gitignore")).unwrap(),
                "target/",
                "{label} must leave a tracked file byte-identical"
            );
            assert!(
                untracked.path().join(LOCK_DIR).join(LOCK_FILE).exists(),
                "{label} must still create the lock it serializes on"
            );
        }

        let topping = tempfile::tempdir().unwrap();
        std::fs::write(topping.path().join(".gitignore"), "target/").unwrap();
        drop(WorkspaceLock::acquire_and_track_gitignore(topping.path()).unwrap());
        assert_eq!(
            std::fs::read_to_string(topping.path().join(".gitignore")).unwrap(),
            "target/\n.cerulion/\n",
            "the constructor that says it tracks `.gitignore` must actually do it"
        );
    }

    /// The guarantee `ros2 migrate --write` rests on, asserted against the
    /// KERNEL rather than against this process's reentrancy registry: while
    /// the interruptible variant is held, a foreign description can take
    /// NEITHER an exclusive NOR a shared lock — the second half is what
    /// separates a real `LOCK_EX` from a `LOCK_SH` that would let a second
    /// migration straight through while every intra-process arm stayed green
    /// (intra-process contention is resolved in the registry and never reaches
    /// `flock` at all).
    #[test]
    fn acquire_interruptibly_takes_an_exclusive_kernel_flock() {
        let temp = tempfile::tempdir().unwrap();
        drop(acquire_quiet(temp.path()));
        let foreign = foreign_description(temp.path());
        assert!(
            foreign_can_take(&foreign, libc::LOCK_EX),
            "nothing is held yet"
        );

        let held = acquire_quiet(temp.path());
        assert!(
            !foreign_can_take(&foreign, libc::LOCK_EX),
            "a foreign exclusive lock went through while the guard was held"
        );
        assert!(
            !foreign_can_take(&foreign, libc::LOCK_SH),
            "a foreign SHARED lock went through — the guard is not exclusive"
        );
        drop(held);
        assert!(
            foreign_can_take(&foreign, libc::LOCK_EX),
            "the guard did not release the kernel lock on drop"
        );
    }

    /// THE guarantee the descriptor walk buys, driven at the only moment that
    /// can distinguish it from a path-based open: `.cerulion` is replaced by a
    /// symlink AFTER the metadata check has passed and BEFORE the lock file is
    /// opened. A path open would follow it and flock an outside inode — two
    /// writers would then hold different locks while mutating one workspace.
    /// Opening the directory `O_NOFOLLOW` and the file `openat`-relative to it
    /// turns that swap into a refusal.
    #[test]
    fn a_directory_swapped_after_the_check_is_refused_not_followed() {
        let temp = tempfile::tempdir().unwrap();
        let outside = tempfile::tempdir().unwrap();
        let lock_dir = temp.path().join(LOCK_DIR);
        let outside_path = outside.path().to_path_buf();
        let swapped = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));

        let armed = swapped.clone();
        let previous = set_pre_open_hook(Some(Box::new(move || {
            // Once only: the acquire runs this after the check, and a retry
            // must not keep re-swapping.
            if armed.swap(true, std::sync::atomic::Ordering::SeqCst) {
                return;
            }
            let _ = std::fs::remove_dir_all(&lock_dir);
            std::os::unix::fs::symlink(&outside_path, &lock_dir).unwrap();
        })));
        let outcome = WorkspaceLock::acquire(temp.path());
        set_pre_open_hook(previous);

        assert!(
            swapped.load(std::sync::atomic::Ordering::SeqCst),
            "the seam never fired — the test proved nothing"
        );
        let err = refusal_text(outcome, "a directory swapped after the check");
        assert!(err.contains("SYMLINK"), "got: {err}");
        assert!(
            std::fs::read_dir(outside.path()).unwrap().next().is_none(),
            "the swap was followed — a lock file was created outside the workspace"
        );
    }

    /// `reservation_is_ours` decides what a failed acquire may tear down, and
    /// both conjuncts guard a different disaster, so both are pinned. Oracle
    /// vectors, not a live registry: this is the pure half, which is what makes
    /// it safe to mutate.
    #[test]
    fn a_reservation_is_ours_only_when_this_thread_left_it_unpromoted() {
        let ours = std::thread::current().id();
        let theirs = std::thread::spawn(|| std::thread::current().id())
            .join()
            .unwrap();
        assert_ne!(ours, theirs, "the fixture needs two distinct thread ids");

        // The only removable shape: our own reservation, not yet promoted.
        assert!(reservation_is_ours(
            Some(&RegistryEntry {
                owner: ours,
                inner: None
            }),
            ours
        ));
        // A PEER's reservation — removing it would hand their workspace away.
        assert!(!reservation_is_ours(
            Some(&RegistryEntry {
                owner: theirs,
                inner: None
            }),
            ours
        ));
        // Our own COMPLETED acquisition — removing it would let a second
        // writer in while we still hold the kernel lock.
        let inner_placeholder: Weak<Inner> = Weak::new();
        assert!(!reservation_is_ours(
            Some(&RegistryEntry {
                owner: ours,
                inner: Some(inner_placeholder.clone())
            }),
            ours
        ));
        // A peer's completed acquisition, and an absent entry.
        assert!(!reservation_is_ours(
            Some(&RegistryEntry {
                owner: theirs,
                inner: Some(inner_placeholder)
            }),
            ours
        ));
        assert!(!reservation_is_ours(None, ours));
    }

    /// THE item-3 pin: `interrupted` is caller-supplied code called while this
    /// thread's reservation is outstanding, so a panic unwinding out of it used
    /// to skip the hand-rolled cleanup and leave the workspace un-acquirable
    /// for the life of the process. The RAII guard gives the reservation back
    /// on the way out.
    ///
    /// The recovery assertion runs on a SEPARATE thread with a deadline: a
    /// leaked reservation parks the next acquire on the registry condvar
    /// forever, so asserting inline would hang the binary instead of failing.
    #[test]
    fn a_panicking_interrupt_predicate_gives_the_reservation_back() {
        let temp = tempfile::tempdir().unwrap();
        drop(acquire_quiet(temp.path()));
        let foreign = foreign_description(temp.path());
        // SAFETY: `foreign` is open for the whole test.
        assert_eq!(0, unsafe {
            libc::flock(foreign.as_raw_fd(), libc::LOCK_EX)
        });

        // The predicate panics on its FIRST call, which happens while the
        // reservation is outstanding and before any lock is held.
        let root = temp.path().to_path_buf();
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _ = WorkspaceLock::acquire_interruptibly(&root, &|| {
                panic!("the caller's predicate blew up")
            });
        }));
        assert!(panicked.is_err(), "the fixture must actually panic");

        // SAFETY: as above.
        assert_eq!(0, unsafe {
            libc::flock(foreign.as_raw_fd(), libc::LOCK_UN)
        });
        let (tx, rx) = mpsc::channel();
        let root = temp.path().to_path_buf();
        std::thread::spawn(move || {
            tx.send(WorkspaceLock::acquire_interruptibly(&root, &|| false).is_ok())
                .unwrap();
        });
        assert!(
            rx.recv_timeout(Duration::from_secs(10))
                .expect("the panicking predicate leaked the registry reservation"),
            "the workspace must be acquirable after a panicking predicate"
        );
    }

    /// The READ path's half of the descriptor walk. A read
    /// creates nothing, so the two "not there" shapes must stay unlocked
    /// guards, while a LINK must refuse: a reader that followed one would take
    /// a shared lock on another tree's file and then report THIS workspace as
    /// safely readable, which is worse than an error.
    #[test]
    fn a_read_refuses_a_symlinked_lock_path_but_tolerates_an_absent_one() {
        // Absent `.cerulion` — unlocked, no error, nothing created.
        let absent = tempfile::tempdir().unwrap();
        let guard = WorkspaceLock::acquire_read(absent.path()).expect("an absent lock dir reads");
        assert!(!guard.is_locked());
        assert!(!absent.path().join(LOCK_DIR).exists());

        // `.cerulion` exists, lock file does not — still unlocked, still no
        // creation (the exclusive path is what mints the file).
        let empty_dir = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(empty_dir.path().join(LOCK_DIR)).unwrap();
        let guard =
            WorkspaceLock::acquire_read(empty_dir.path()).expect("an absent lock file reads");
        assert!(!guard.is_locked());
        assert!(!empty_dir.path().join(LOCK_DIR).join(LOCK_FILE).exists());

        // A symlinked `.cerulion` — refused, and NO shared lock taken on the
        // file at the far end.
        //
        // The target holds a REAL `workspace.lock`. Against an EMPTY target the
        // old assertion could not fail at all: `acquire_read` creates nothing
        // on any path, so the pre-change code (which followed the link, found
        // no lock file and returned an unlocked guard) left it just as empty as
        // the fixed code does.
        //
        // Scope of the replacement: the headline variant — a walk that
        // FOLLOWS the link — is killed by `refusal_text` below, which panics on
        // any `Ok`. What the foreign `LOCK_EX` probe adds is the narrower
        // property that a refusal leaves NOTHING locked behind, on a file that
        // is not this workspace's.
        let linked_dir = tempfile::tempdir().unwrap();
        let outside = tempfile::tempdir().unwrap();
        std::fs::write(outside.path().join(LOCK_FILE), b"").unwrap();
        std::os::unix::fs::symlink(outside.path(), linked_dir.path().join(LOCK_DIR)).unwrap();
        let err = refusal_text(
            WorkspaceLock::acquire_read(linked_dir.path()),
            "a read through a symlinked .cerulion",
        );
        assert!(err.contains("SYMLINK"), "got: {err}");
        let outsider = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(outside.path().join(LOCK_FILE))
            .unwrap();
        assert!(
            foreign_can_take(&outsider, libc::LOCK_EX),
            "the read took a shared lock on a file OUTSIDE this workspace"
        );

        // A symlinked lock FILE inside a real directory — refused by
        // O_NOFOLLOW. Same discipline: the target EXISTS and is lockable, so
        // a walk that followed the link would hold `LOCK_SH` on it.
        let linked_file = tempfile::tempdir().unwrap();
        let target = linked_file.path().join("elsewhere");
        std::fs::write(&target, b"").unwrap();
        std::fs::create_dir_all(linked_file.path().join(LOCK_DIR)).unwrap();
        std::os::unix::fs::symlink(&target, linked_file.path().join(LOCK_DIR).join(LOCK_FILE))
            .unwrap();
        let err = refusal_text(
            WorkspaceLock::acquire_read(linked_file.path()),
            "a read through a symlinked lock file",
        );
        assert!(err.contains("SYMLINK"), "got: {err}");
        let outsider = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(&target)
            .unwrap();
        assert!(
            foreign_can_take(&outsider, libc::LOCK_EX),
            "the read took a shared lock on the link's target"
        );

        // DANGLING links, both shapes. MEASURED on macOS: a dangling
        // `.cerulion` reports ENOTDIR and a dangling lock file reports ELOOP,
        // so both are refused. Neither reports ENOENT — which matters, because
        // ENOENT is the ONLY errno on this path that yields an unlocked-but-Ok
        // guard, so a platform that answered it there would silently degrade a
        // symlinked lock path to "readable". This arm is what would fail if one
        // ever did.
        let dangling_dir = tempfile::tempdir().unwrap();
        std::os::unix::fs::symlink(
            dangling_dir.path().join("nowhere"),
            dangling_dir.path().join(LOCK_DIR),
        )
        .unwrap();
        let err = refusal_text(
            WorkspaceLock::acquire_read(dangling_dir.path()),
            "a read through a DANGLING .cerulion link",
        );
        assert!(err.contains("SYMLINK"), "got: {err}");

        let dangling_file = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(dangling_file.path().join(LOCK_DIR)).unwrap();
        std::os::unix::fs::symlink(
            dangling_file.path().join("nowhere"),
            dangling_file.path().join(LOCK_DIR).join(LOCK_FILE),
        )
        .unwrap();
        let err = refusal_text(
            WorkspaceLock::acquire_read(dangling_file.path()),
            "a read through a DANGLING lock-file link",
        );
        assert!(err.contains("SYMLINK"), "got: {err}");

        // A REGULAR FILE at `.cerulion` is not a link, and saying it is names
        // a cause that does not exist. Reachable only on the read path: the
        // exclusive path's own pre-check refuses it before the walk.
        let plain = tempfile::tempdir().unwrap();
        std::fs::write(plain.path().join(LOCK_DIR), b"not a directory").unwrap();
        let err = refusal_text(
            WorkspaceLock::acquire_read(plain.path()),
            "a read through a regular-file .cerulion",
        );
        assert!(
            err.contains("NOT A DIRECTORY") && !err.contains("SYMLINK"),
            "a regular file was diagnosed as a symlink: {err}"
        );

        // ANTI-TAUTOLOGY: a healthy workspace still yields a REAL shared lock,
        // so the refusals above are about links and not about the walk being
        // broken for everyone.
        let healthy = tempfile::tempdir().unwrap();
        drop(acquire_quiet(healthy.path()));
        let guard = WorkspaceLock::acquire_read(healthy.path()).expect("a healthy read");
        assert!(
            guard.is_locked(),
            "a real lock file must yield a held guard"
        );
    }

    /// EPERM, specifically — the errno the rewrite dropped and `chmod 0o444`
    /// does not produce.
    ///
    /// The sibling arm above chmods the file, which yields EACCES, so it kills
    /// "delete the whole retry" but NOT "list EACCES and forget EPERM":
    /// omitting EPERM survives it. `std` decodes both errnos to
    /// `ErrorKind::PermissionDenied`, which is exactly what the path-based
    /// predecessor matched, so the omission silently turned an immutable lock
    /// file from "degrades to a real shared read" into "hard error on every
    /// read verb".
    ///
    /// macOS only, and that is its actual reach: `chflags(UF_IMMUTABLE)` gives
    /// EPERM on an `O_RDWR` open as an ORDINARY user (measured: `O_RDWR` =>
    /// EPERM, `O_RDONLY` => ok), while the Linux equivalent (`chattr +i`) needs
    /// `CAP_LINUX_IMMUTABLE`. CI runs a macOS job, so this is covered there
    /// rather than nowhere.
    #[cfg(target_os = "macos")]
    #[test]
    fn a_read_of_an_immutable_lock_file_still_takes_a_shared_lock() {
        /// Clears the flag on the way out, including during a panic — an
        /// immutable file left behind defeats the tempdir's own cleanup.
        struct Immutable(std::ffi::CString);
        impl Immutable {
            fn set(path: &Path, flags: libc::c_uint) -> Self {
                use std::os::unix::ffi::OsStrExt;
                let c = std::ffi::CString::new(path.as_os_str().as_bytes()).unwrap();
                // SAFETY: `c` is NUL-terminated and outlives the call.
                assert_eq!(0, unsafe { libc::chflags(c.as_ptr(), flags) });
                Self(c)
            }
        }
        impl Drop for Immutable {
            fn drop(&mut self) {
                // SAFETY: as above.
                unsafe { libc::chflags(self.0.as_ptr(), 0) };
            }
        }

        let temp = tempfile::tempdir().unwrap();
        drop(acquire_quiet(temp.path()));
        let lock_path = temp.path().join(LOCK_DIR).join(LOCK_FILE);
        let _immutable = Immutable::set(&lock_path, libc::UF_IMMUTABLE);
        assert!(
            std::fs::OpenOptions::new()
                .write(true)
                .open(&lock_path)
                .is_err(),
            "the fixture did not make the lock file unwritable, so this arm proves nothing"
        );

        let guard = WorkspaceLock::acquire_read(temp.path())
            .expect("an immutable lock file must still read");
        assert!(
            guard.is_locked(),
            "EPERM must take the read-only retry, not become a hard error"
        );
    }

    /// The read path's errno table, as a vector oracle.
    ///
    /// The fixtures below kill "delete the whole read-only retry"; they cannot
    /// kill "drop one errno from it", because `chmod` yields `EACCES` and the
    /// alternatives need root. Against the table, dropping any entry fails a
    /// named vector.
    #[test]
    fn the_read_open_errno_table_is_exact() {
        use super::{read_open_outcome, ReadOpen};
        assert_eq!(
            read_open_outcome(Some(libc::ENOENT)),
            ReadOpen::Unlocked,
            "an absent lock file is a readable workspace, not an error"
        );
        assert_eq!(
            read_open_outcome(Some(libc::ELOOP)),
            ReadOpen::Symlink,
            "a link is refused, never followed"
        );
        for errno in [libc::EACCES, libc::EPERM, libc::EROFS] {
            assert_eq!(
                read_open_outcome(Some(errno)),
                ReadOpen::RetryReadOnly,
                "errno {errno} must take the read-only retry — `std` decodes EACCES and \
                 EPERM alike to PermissionDenied, which is what the predecessor matched"
            );
        }
        for errno in [libc::EISDIR, libc::EIO, libc::ENOTDIR, libc::EMFILE] {
            assert_eq!(
                read_open_outcome(Some(errno)),
                ReadOpen::Fail,
                "errno {errno} is a real failure and must not degrade to a readable guard"
            );
        }
        assert_eq!(
            read_open_outcome(None),
            ReadOpen::Fail,
            "an unknown failure must not be treated as absence"
        );
    }

    /// A read that has to WAIT says so, exactly once. The read path blocks in
    /// the kernel for as long as the writer holds the lock, and `cerulion-wsd`
    /// serves reads on a blocking-pool thread — so a read concurrent with a
    /// `ros2 migrate --write` would otherwise park one of those threads for the
    /// whole migration in complete silence, while the exclusive path three screens
    /// away tries `LOCK_NB` first and names the file.
    ///
    /// The read runs on THIS thread because `tracing_test` attributes captured
    /// lines by the span around the test function; the writer is the helper.
    #[test]
    fn a_contended_read_warns_once_before_it_blocks() {
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path().to_path_buf();
        let (holding_tx, holding_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel::<()>();
        let holder = std::thread::spawn(move || {
            let held = acquire_quiet(&root);
            holding_tx.send(()).unwrap();
            let _ = release_rx.recv_timeout(Duration::from_secs(30));
            drop(held);
        });
        holding_rx
            .recv_timeout(Duration::from_secs(10))
            .expect("the writer never took the lock");

        // The lock really is held RIGHT NOW, so the read below really will
        // contend. This replaces a wall-clock assertion on how long the read
        // took — a floor like that fails on healthy code whenever a loaded
        // runner deschedules the reader past the holder's sleep.
        let probe = foreign_description(temp.path());
        assert!(
            !foreign_can_take(&probe, libc::LOCK_SH),
            "the writer's lock was not actually held, so this read would not contend"
        );

        // THE HANDSHAKE. A fixed timer here would race it: a stall between
        // the reader's "about to read" signal and the `flock` below would
        // leave the read uncontended, no warn fires, and the arm fails on
        // healthy code. The warn itself is
        // the readiness: it is emitted only after the `LOCK_SH|LOCK_NB` probe
        // has ALREADY failed, so a loaded runner can make it later but cannot
        // outrun it. The peer is released on that signal and nothing else.
        //
        // The same subscriber counts the warns, which is why this arm does not
        // use `#[traced_test]`: one mechanism, and it is the one the release
        // depends on, so a broken capture fails the handshake rather than
        // silently vacating the count.
        let hits = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let (ready_tx, ready_rx) = mpsc::channel::<()>();
        let releaser = std::thread::spawn(move || {
            let _ = ready_rx.recv_timeout(Duration::from_secs(30));
            release_tx.send(()).ok();
        });
        let watch = WarnWatch {
            needle: "waiting to read it",
            hits: hits.clone(),
            ready: std::sync::Mutex::new(Some(ready_tx)),
            reenter: None,
        };
        let guard = tracing::subscriber::with_default(watch, || {
            WorkspaceLock::acquire_read(temp.path()).expect("the read must complete, not fail")
        });
        assert!(guard.is_locked(), "a contended read still ends up holding");
        drop(guard);
        releaser.join().unwrap();
        holder.join().unwrap();
        assert_eq!(
            hits.load(std::sync::atomic::Ordering::SeqCst),
            1,
            "expected exactly 1 contended-read warn — one per contended read, never one \
             per poll, and never none (which would mean the read did not contend)"
        );
    }

    /// A lock file this reader may not WRITE still yields a REAL shared lock,
    /// through a read-only open. Nothing covered that retry: deleting the
    /// whole `EACCES`/`EPERM`/`EROFS` block left the suite green, which is how
    /// the rewrite quietly dropped `EPERM` (the path-based predecessor matched
    /// `ErrorKind::PermissionDenied`, which `std` decodes from BOTH errnos) and
    /// turned an immutable lock file into a hard error on every read verb.
    #[test]
    fn a_read_of_an_unwritable_lock_file_still_takes_a_shared_lock() {
        use std::os::unix::fs::PermissionsExt;
        let temp = tempfile::tempdir().unwrap();
        drop(acquire_quiet(temp.path()));
        let lock_path = temp.path().join(LOCK_DIR).join(LOCK_FILE);
        std::fs::set_permissions(&lock_path, std::fs::Permissions::from_mode(0o444)).unwrap();

        // As root the fixture is INERT — `open(O_RDWR)` on a 0444 file succeeds,
        // the retry is never entered, and the arm would pass having proven
        // nothing. Say so out loud and stop, rather than reporting coverage
        // this run does not have. (The errno-table oracle above is
        // root-independent and still covers the classification.)
        if std::fs::OpenOptions::new()
            .write(true)
            .open(&lock_path)
            .is_ok()
        {
            std::fs::set_permissions(&lock_path, std::fs::Permissions::from_mode(0o644)).unwrap();
            eprintln!(
                "DEGRADE: running as a user who can write a 0444 file (root?) — the \
                 read-only retry is not exercised by this arm on this runner"
            );
            return;
        }

        let guard = WorkspaceLock::acquire_read(temp.path())
            .expect("an unwritable lock file must still read");
        assert!(
            guard.is_locked(),
            "the read-only retry must yield a REAL shared lock, not an unlocked guard"
        );
        // And it really is SHARED, not merely reported: a writer must be
        // excluded while the guard lives and admitted the moment it dies.
        //
        // The mode goes back to 0644 FIRST, because the probe opens the file
        // read+write and would otherwise fail on the fixture instead of on the
        // lock. Restoring it cannot affect the guard — that lock rides an open
        // descriptor, and permissions are checked at open time.
        std::fs::set_permissions(&lock_path, std::fs::Permissions::from_mode(0o644)).unwrap();
        let foreign = foreign_description(temp.path());
        assert!(
            !foreign_can_take(&foreign, libc::LOCK_EX),
            "the guard reports a shared lock it does not hold"
        );
        drop(guard);
        assert!(
            foreign_can_take(&foreign, libc::LOCK_EX),
            "the shared lock outlived its guard"
        );
    }

    /// The contention `warn!` is the ONLY thing a user sees
    /// during a multi-minute wait, and the wait is a LOOP — so the line must
    /// fire once per contended acquire, not once per poll. At the shipped
    /// 100 ms cadence a per-poll warn is ~600 lines a minute per waiter, the
    /// log-flood disk-fill class.
    ///
    /// The release is driven by the predicate's OWN call count, so nothing
    /// wall-clock decides the outcome on a healthy run: the foreign lock is
    /// held until the wait has provably polled several times, which is exactly
    /// when a per-poll warn would have emitted several lines. The one wall is
    /// the releaser's 30 s escape hatch, so a wedged run fails at the
    /// `polled >= POLLS_BEFORE_RELEASE` assertion instead of hanging CI.
    #[tracing_test::traced_test]
    #[test]
    fn a_contended_acquire_warns_once_not_once_per_poll() {
        const POLLS_BEFORE_RELEASE: usize = 5;
        let temp = tempfile::tempdir().unwrap();
        drop(acquire_quiet(temp.path()));
        let foreign = foreign_description(temp.path());
        // SAFETY: `foreign` is open for the whole test.
        assert_eq!(0, unsafe {
            libc::flock(foreign.as_raw_fd(), libc::LOCK_EX)
        });

        // The ACQUIRE runs on the test's own thread and the RELEASE on the
        // helper, not the other way round: `tracing_test` attributes captured
        // lines by the span it opens around the test FUNCTION, so a warn
        // emitted on a spawned thread is invisible to `logs_assert` — measured,
        // the line appeared in the captured stdout while the assert counted 0.
        //
        // The helper owns a DUPLICATE descriptor rather than a raw fd number:
        // this test returns in well under a second, and a detached thread
        // holding a bare number could unlock whatever a parallel test had since
        // opened at it.
        let releaser_fd = foreign.try_clone().expect("dup the lock descriptor");
        let polls = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let polls_helper = polls.clone();
        let releaser = std::thread::spawn(move || {
            let deadline = std::time::Instant::now() + Duration::from_secs(30);
            while polls_helper.load(std::sync::atomic::Ordering::SeqCst) < POLLS_BEFORE_RELEASE {
                if std::time::Instant::now() >= deadline {
                    break;
                }
                std::thread::sleep(Duration::from_millis(20));
            }
            // SAFETY: the helper owns `releaser_fd` for the whole call.
            unsafe { libc::flock(releaser_fd.as_raw_fd(), libc::LOCK_UN) };
        });

        let lock = WorkspaceLock::acquire_interruptibly(temp.path(), &|| {
            polls.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1 > POLL_DEADLINE
        })
        .expect(
            "the contended acquire never completed — the foreign lock was not released \
             within the poll deadline",
        );
        releaser.join().unwrap();
        drop(lock);

        let polled = polls.load(std::sync::atomic::Ordering::SeqCst);
        assert!(
            polled >= POLLS_BEFORE_RELEASE,
            "the wait polled {polled} times — fewer than the {POLLS_BEFORE_RELEASE} a \
             per-poll warn needs to be distinguishable, so the arm would prove nothing"
        );
        logs_assert(|lines: &[&str]| {
            let warns = lines
                .iter()
                .filter(|line| line.contains("workspace is locked by another process"))
                .count();
            if warns == 1 {
                Ok(())
            } else {
                Err(format!(
                    "expected exactly 1 contention warn for 1 contended acquire, got {warns} \
                     (the wait polled {polled} times)"
                ))
            }
        });
    }

    /// `lock_io_refusal`'s `ErrorKind` preservation is a CROSS-CRATE contract:
    /// `cerulion_wsd` forwards the rendered string to its client verbatim and
    /// keys its `workspace_not_found` reply off `ErrorKind::NotFound`. Nothing
    /// asserted it, and `io::Error::other(..)` — the shortest thing anyone
    /// would write — compiles, reads identically, and silently reclassifies
    /// every lock I/O failure as `Other`.
    #[test]
    fn a_lock_io_refusal_keeps_its_kind_and_names_the_path_and_the_operation() {
        for kind in [
            std::io::ErrorKind::NotFound,
            std::io::ErrorKind::PermissionDenied,
            std::io::ErrorKind::AlreadyExists,
        ] {
            let refusal = super::lock_io_refusal(
                Path::new("/ws/.cerulion/workspace.lock"),
                "opening",
                std::io::Error::new(kind, "the raw cause"),
            );
            match &refusal {
                CliError::Io(error) => assert_eq!(
                    error.kind(),
                    kind,
                    "the kind must survive — wsd keys workspace_not_found off NotFound"
                ),
                other => panic!("expected CliError::Io, got {other:?}"),
            }
            let rendered = refusal.to_string();
            assert!(
                rendered.contains("/ws/.cerulion/workspace.lock"),
                "the refusal must name the path; got: {rendered}"
            );
            assert!(
                rendered.contains("opening"),
                "the refusal must name the operation; got: {rendered}"
            );
            assert!(
                rendered.contains("the raw cause"),
                "the refusal must keep the underlying cause; got: {rendered}"
            );
        }
    }

    /// A `tracing` subscriber is CALLER CODE, and it runs synchronously on the
    /// emitting thread. A contention `warn!` emitted while the
    /// process-global registry mutex is HELD means a subscriber that re-enters
    /// the workspace lock — for ANY root, the registry being global and its
    /// mutex not reentrant — blocks on a mutex its own thread holds and
    /// deadlocks outright, never reaching the `wake.wait` that would have
    /// released it. The in-process warn is exactly where that
    /// deadlock would sit.
    ///
    /// The subscriber acquires a DIFFERENT root, so this pins the global-mutex
    /// half specifically: the same-root case is the separate re-entrancy
    /// refusal below. The acquire runs on a worker with a deadline, because a
    /// regression HANGS rather than fails.
    ///
    /// NO WALL. The release is gated on the subscriber having FIRED, which is
    /// the moment the acquire provably reached contention; a fixed sleep of
    /// 200 ms instead is one a loaded runner can outrun — the
    /// worker has to spawn, install the subscriber, canonicalize, take the
    /// registry mutex and probe the lock before the warn can happen, and if the
    /// holder released first there is no contention, no warn, and a spurious
    /// failure. The same discipline holds here and in
    /// `a_contended_read_warns_once_before_it_blocks`, which has the same shape.
    #[test]
    fn a_subscriber_that_re_enters_the_lock_does_not_deadlock_the_warn() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        let contended = tempfile::tempdir().unwrap();
        let unrelated = tempfile::tempdir().unwrap();
        // Scaffold the unrelated lock directory up front, so the work inside
        // the subscriber is an acquire and nothing else.
        drop(acquire_quiet(unrelated.path()));

        let held_root = contended.path().to_path_buf();
        let (holding_tx, holding_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel::<()>();
        let holder = std::thread::spawn(move || {
            let held = acquire_quiet(&held_root);
            holding_tx.send(()).unwrap();
            let _ = release_rx.recv_timeout(Duration::from_secs(30));
            drop(held);
        });
        holding_rx
            .recv_timeout(Duration::from_secs(10))
            .expect("the peer never took the lock");

        let hits = std::sync::Arc::new(AtomicUsize::new(0));
        let hits_worker = hits.clone();
        let (ready_tx, ready_rx) = mpsc::channel::<()>();
        let other_root = unrelated.path().to_path_buf();
        let contended_root = contended.path().to_path_buf();
        let (done_tx, done_rx) = mpsc::channel();
        std::thread::spawn(move || {
            let subscriber = WarnWatch {
                needle: "locked by another request in this process",
                hits: hits_worker,
                ready: std::sync::Mutex::new(Some(ready_tx)),
                reenter: Some(other_root),
            };
            // Thread-local, so it cannot disturb any sibling test.
            tracing::subscriber::with_default(subscriber, || {
                // BLOCKING variant: it is the one `cerulion-wsd` uses, and the
                // one whose warn sat under the mutex.
                let lock = WorkspaceLock::acquire(&contended_root);
                done_tx.send(lock.is_ok()).ok();
            });
        });

        // THE HANDSHAKE: the worker has warned, so it is provably inside the
        // contended wait. Only now does the peer let go.
        ready_rx
            .recv_timeout(Duration::from_secs(20))
            .expect("the contended acquire never warned, so it never reached contention");
        release_tx.send(()).ok();

        assert!(
            done_rx.recv_timeout(Duration::from_secs(20)).expect(
                "the acquire never returned — a subscriber re-entering the lock \
                         deadlocked the thread that was warning about contention"
            ),
            "the contended acquire must still complete"
        );
        holder.join().unwrap();
        assert!(
            hits.load(Ordering::SeqCst) > 0,
            "the subscriber never saw the contention warn, so this arm proves nothing"
        );
    }

    /// The re-entrancy refusal, which replaced a park nothing could ever wake.
    ///
    /// Reaching it needs THIS thread's own UNPROMOTED reservation, which exists
    /// only between the reservation and the promotion — i.e. while the caller's
    /// `interrupted` predicate runs. The predicate is consulted before the
    /// first sleep, so the arm has no wall in it, and it ends the outer wait
    /// itself so a reverted refusal fails here instead of hanging.
    #[test]
    fn re_entering_the_lock_from_the_interrupt_predicate_is_refused_not_parked() {
        let temp = tempfile::tempdir().unwrap();
        drop(acquire_quiet(temp.path()));
        let foreign = foreign_description(temp.path());
        // SAFETY: `foreign` is open for the whole test.
        assert_eq!(0, unsafe {
            libc::flock(foreign.as_raw_fd(), libc::LOCK_EX)
        });

        let inner_text = std::sync::Mutex::new(None::<String>);
        let outcome = WorkspaceLock::acquire_interruptibly(temp.path(), &|| {
            if inner_text.lock().unwrap().is_none() {
                let re_entered =
                    WorkspaceLock::acquire_interruptibly(temp.path(), never_interrupted());
                *inner_text.lock().unwrap() = Some(match re_entered {
                    Ok(_) => "the re-entrant acquire SUCCEEDED".to_string(),
                    Err(super::AcquireError::Interrupted) => "Interrupted".to_string(),
                    // The re-entrancy refusal is an INTERNAL-BUG arm;
                    // the assertions below read the text, so
                    // both failure arms hand it over the same way.
                    Err(super::AcquireError::Failed(error))
                    | Err(super::AcquireError::Internal(error)) => error.to_string(),
                });
            }
            true
        });
        assert!(
            matches!(outcome, Err(super::AcquireError::Interrupted)),
            "the outer acquire must end on its own predicate"
        );

        let text = inner_text
            .lock()
            .unwrap()
            .clone()
            .expect("the predicate ran");
        assert!(
            text.contains("re-entered the workspace lock"),
            "a re-entrant acquire must be REFUSED, not parked on a notify only this thread \
             could send; got: {text}"
        );
        assert!(
            text.contains(&temp.path().canonicalize().unwrap().display().to_string()),
            "the refusal must name the root; got: {text}"
        );

        // SAFETY: as above.
        assert_eq!(0, unsafe {
            libc::flock(foreign.as_raw_fd(), libc::LOCK_UN)
        });
        let (tx, rx) = mpsc::channel();
        let root = temp.path().to_path_buf();
        std::thread::spawn(move || {
            tx.send(WorkspaceLock::acquire(&root).is_ok()).ok();
        });
        assert!(
            rx.recv_timeout(Duration::from_secs(10))
                .expect("the refusal leaked the reservation"),
            "the workspace must still be lockable after a refused re-entry"
        );
    }

    /// A SECOND contended acquire must warn AGAIN. Without this, a warn
    /// latched process-globally — the obvious "fix" for a flood — passes
    /// `a_contended_acquire_warns_once_not_once_per_poll` unchanged while
    /// leaving every later waiter with no signal at all.
    #[tracing_test::traced_test]
    #[test]
    fn a_second_contended_acquire_warns_again() {
        let temp = tempfile::tempdir().unwrap();
        drop(acquire_quiet(temp.path()));
        let foreign = foreign_description(temp.path());

        for round in 1..=2 {
            // SAFETY: `foreign` is open for the whole test.
            assert_eq!(0, unsafe {
                libc::flock(foreign.as_raw_fd(), libc::LOCK_EX)
            });
            let releaser_fd = foreign.try_clone().expect("dup the lock descriptor");
            let polls = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
            let polls_helper = polls.clone();
            let releaser = std::thread::spawn(move || {
                let deadline = std::time::Instant::now() + Duration::from_secs(30);
                while polls_helper.load(std::sync::atomic::Ordering::SeqCst) < 2
                    && std::time::Instant::now() < deadline
                {
                    std::thread::sleep(Duration::from_millis(20));
                }
                // SAFETY: the helper owns `releaser_fd` for the whole call.
                unsafe { libc::flock(releaser_fd.as_raw_fd(), libc::LOCK_UN) };
            });
            let lock = WorkspaceLock::acquire_interruptibly(temp.path(), &|| {
                // Same self-owned deadline as the warn-once arm: a release
                // that never lands must FAIL this test, not hang CI.
                polls.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1 > POLL_DEADLINE
            })
            .unwrap_or_else(|_| {
                panic!("round {round} never acquired — the foreign lock was never released")
            });
            assert!(
                polls.load(std::sync::atomic::Ordering::SeqCst) >= 2,
                "round {round} never waited, so a missing warn would be about the fixture \
                 rather than about the latch"
            );
            releaser.join().unwrap();
            drop(lock);
        }

        logs_assert(|lines: &[&str]| {
            let warns = lines
                .iter()
                .filter(|line| line.contains("workspace is locked by another process"))
                .count();
            if warns == 2 {
                Ok(())
            } else {
                Err(format!(
                    "expected 1 warn per contended acquire = 2, got {warns}"
                ))
            }
        });
    }

    /// The OTHER half of the contention `warn!`: a same-process peer is waited out on
    /// the registry condvar, where the kernel `flock` is never reached — so
    /// that wait had no signal of any kind while the cross-process one warned.
    /// `cerulion-wsd` is the multi-threaded consumer, so the silent shape was
    /// its common one.
    ///
    /// The `cross_process == 0` assertion below is NOT a pin for the release
    /// ORDER in `WorkspaceLock::drop`. Swapping that order still has the
    /// notify land after the kernel release, so a woken waiter finds the lock
    /// already free; observing the window needs a `wait_timeout` expiry
    /// landing inside a sub-microsecond gap and then four syscalls completing
    /// before one, and a variant that swaps the order wins that race
    /// essentially always. What the assertion does pin is the WORDING: a
    /// same-process peer is never reported as another process.
    ///
    /// The holder runs on a helper and the contender on THIS thread:
    /// `tracing_test` attributes
    /// captured lines by the span around the test FUNCTION, so the acquire
    /// whose warn is counted has to be the one on the test thread.
    #[tracing_test::traced_test]
    #[test]
    fn an_in_process_wait_warns_once_and_names_this_process() {
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path().to_path_buf();
        let (holding_tx, holding_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel::<()>();
        let holder = std::thread::spawn(move || {
            let held = acquire_quiet(&root);
            holding_tx.send(()).unwrap();
            let _ = release_rx.recv_timeout(Duration::from_secs(30));
            drop(held);
        });
        holding_rx
            .recv_timeout(Duration::from_secs(10))
            .expect("the holder never took the lock");

        // Release on a timer the CONTENDED acquire itself drives: the
        // predicate is called once per registry park, so a few parks prove the
        // wait was real before the holder lets go.
        let polls = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let polls_helper = polls.clone();
        let timer = std::thread::spawn(move || {
            let deadline = std::time::Instant::now() + Duration::from_secs(30);
            while polls_helper.load(std::sync::atomic::Ordering::SeqCst) < 3
                && std::time::Instant::now() < deadline
            {
                std::thread::sleep(Duration::from_millis(20));
            }
            release_tx.send(()).ok();
        });

        let lock = WorkspaceLock::acquire_interruptibly(temp.path(), &|| {
            polls.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            false
        })
        .expect("the in-process wait must complete once the peer releases");
        drop(lock);
        timer.join().unwrap();
        holder.join().unwrap();

        let polled = polls.load(std::sync::atomic::Ordering::SeqCst);
        assert!(
            polled >= 3,
            "the registry park ran {polled} times — too few for a per-park warn to be \
             distinguishable from a per-acquire one"
        );
        logs_assert(|lines: &[&str]| {
            let same_process = lines
                .iter()
                .filter(|line| line.contains("locked by another request in this process"))
                .count();
            let cross_process = lines
                .iter()
                .filter(|line| line.contains("locked by another process"))
                .count();
            if same_process != 1 {
                return Err(format!(
                    "expected exactly 1 in-process contention warn, got {same_process} \
                     (the park ran {polled} times)"
                ));
            }
            if cross_process != 0 {
                return Err(format!(
                    "a same-process peer was reported as another PROCESS {cross_process} time(s)"
                ));
            }
            Ok(())
        });
    }

    /// The registry park runs the caller's predicate with the registry mutex
    /// RELEASED. A panic out of it under that mutex would poison the map, and
    /// every later acquire in the process — including the one inside
    /// `WorkspaceLock::drop`, where a panic during an unwind ABORTS — would
    /// then fail. That is strictly worse than the leak item 3 fixes.
    ///
    /// `a_panicking_interrupt_predicate_gives_the_reservation_back` cannot see
    /// it: its fixture uses a FOREIGN holder, so the panic lands on the kernel
    /// arm, which never held the registry mutex. This one panics from inside
    /// the PARK.
    ///
    /// The OUTCOME alone cannot discriminate, because the poison recovery
    /// added by the same sweep absorbs it — so this arm asserts the poison
    /// DIRECTLY, with `Mutex::is_poisoned`. The outcome half is kept as the
    /// recovery pin; each half alone lets a different variant slip through,
    /// so both run together.
    #[test]
    fn a_predicate_that_panics_in_the_registry_park_leaves_the_map_usable() {
        let temp = tempfile::tempdir().unwrap();
        let held = acquire_quiet(temp.path());
        let root = temp.path().to_path_buf();
        let calls = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let calls_worker = calls.clone();
        let worker = std::thread::spawn(move || {
            let _ = WorkspaceLock::acquire_interruptibly(&root, &|| {
                // Panic on the SECOND call: the first is the park's own, so
                // the panic provably comes from inside the wait.
                if calls_worker.fetch_add(1, std::sync::atomic::Ordering::SeqCst) > 0 {
                    panic!("the caller's predicate blew up inside the registry park");
                }
                false
            });
        });
        assert!(
            worker.join().is_err(),
            "the fixture must actually panic, or this arm proves nothing"
        );
        assert!(
            calls.load(std::sync::atomic::Ordering::SeqCst) >= 2,
            "the panic must come from the PARK, not from a pre-park call"
        );

        // THE discriminating assertion. Under a variant that calls
        // `interrupted()` above `drop(entries)`, the predicate panics with
        // the registry guard live, which poisons the map. Every lock in this
        // module recovers from that, so nothing downstream fails — the
        // poison itself is the only observable.
        let (registry, _) = super::REGISTRY
            .get()
            .expect("the registry exists once acquired");
        assert!(
            !registry.is_poisoned(),
            "a panic in the caller's predicate poisoned the registry — it ran while the \
             registry mutex was HELD"
        );

        // And the recovery half: the workspace is still usable afterwards.
        drop(held);

        let (tx, rx) = mpsc::channel();
        let root = temp.path().to_path_buf();
        std::thread::spawn(move || {
            tx.send(WorkspaceLock::acquire(&root).is_ok()).ok();
        });
        assert!(
            rx.recv_timeout(Duration::from_secs(10))
                .expect("the workspace lock stopped working after a panic in the park"),
            "a panicking predicate must not take the workspace lock away from the process"
        );
    }

    /// A REGULAR FILE at `<root>/.cerulion` is not a symlink, and telling the
    /// user it is names a cause that does not exist.
    ///
    /// On the EXCLUSIVE path the `symlink_metadata` pre-check refuses it before
    /// the descriptor walk ever runs, so this arm pins THAT wording — without
    /// the pre-check the walk reaches `create_dir_all` and surfaces a bare
    /// `AlreadyExists`, which is what `NOT A DIRECTORY` replaces here. The arm
    /// that reaches `classify_lock_dir` is the read path's twin, inside
    /// `a_read_refuses_a_symlinked_lock_path_but_tolerates_an_absent_one`.
    #[test]
    fn a_regular_file_at_the_lock_directory_is_not_reported_as_a_symlink() {
        let temp = tempfile::tempdir().unwrap();
        std::fs::write(temp.path().join(LOCK_DIR), b"not a directory").unwrap();
        let err = refusal_text(
            WorkspaceLock::acquire(temp.path()),
            "an acquire through a regular-file .cerulion",
        );
        assert!(
            err.contains("NOT A DIRECTORY"),
            "the refusal must name the real condition; got: {err}"
        );
        assert!(
            !err.contains("SYMLINK"),
            "a regular file was diagnosed as a symlink: {err}"
        );
        assert!(
            err.contains(LOCK_DIR),
            "the refusal must name the offending path; got: {err}"
        );
    }

    /// THE three-state pin. Every state is produced by a real acquisition, and
    /// the two that a bool would merge are asserted to DIFFER — which is the
    /// assertion a `created_lock_dir() -> bool` cannot carry at all.
    #[test]
    fn the_lock_dir_origin_separates_a_nested_reuse_from_an_earlier_runs_directory() {
        let temp = tempfile::tempdir().unwrap();
        let first = acquire_quiet(temp.path());
        assert_eq!(
            first.lock_dir_origin(),
            LockDirOrigin::CreatedHere,
            "the acquisition that creates `.cerulion/` must say so"
        );
        // NESTED, inside a run that created the directory.
        let nested = acquire_quiet(temp.path());
        let nested_over_created = nested.lock_dir_origin();
        assert_eq!(
            nested.lock_dir_origin(),
            LockDirOrigin::CreatedByAnOuterAcquisition,
            "a nested acquire created nothing itself, but THIS RUN created the directory — \
             the state a bool collapses into the same `false` it gives a week-old directory"
        );
        drop(nested);
        drop(first);
        // A later, independent acquisition finds the directory already there.
        let second = acquire_quiet(temp.path());
        assert_eq!(
            second.lock_dir_origin(),
            LockDirOrigin::PreExisting,
            "only the creating acquisition reports CreatedHere"
        );
        // A nest INSIDE that one inherits `PreExisting`, not the nested state:
        // the nesting is real, the creation is not.
        let nested_over_existing = acquire_quiet(temp.path());
        assert_eq!(
            nested_over_existing.lock_dir_origin(),
            LockDirOrigin::PreExisting,
            "nesting does not invent a creation this run never made"
        );
        // THE DISCRIMINATOR, stated as its own assertion so the intent cannot
        // be read out of the suite by deleting one `assert_eq!` above.
        assert_ne!(
            nested_over_created,
            nested_over_existing.lock_dir_origin(),
            "two nested acquires must NOT report the same origin when one run created the \
             directory and the other found it"
        );
    }

    /// The pure classifier behind all three states, against a hand-written
    /// oracle over its whole domain.
    ///
    /// Scope, stated the way the production docs state it: the two OUTERMOST
    /// inputs are reached in production (the top-up decision passes
    /// `Nesting::Outermost` literally, so one of them decides what lands in a
    /// TRACKED file). The two NESTED inputs are reached only through
    /// `WorkspaceLock::lock_dir_origin`, which the engine does not read today —
    /// they are the REPORT, and the oracle is what keeps the report correct
    /// until something consumes it.
    #[test]
    fn the_lock_dir_origin_classifier_matches_its_oracle_on_every_input() {
        let oracle = [
            ((Nesting::Outermost, true), LockDirOrigin::CreatedHere),
            (
                (Nesting::InsideAnOuterAcquisition, true),
                LockDirOrigin::CreatedByAnOuterAcquisition,
            ),
            ((Nesting::Outermost, false), LockDirOrigin::PreExisting),
            (
                (Nesting::InsideAnOuterAcquisition, false),
                LockDirOrigin::PreExisting,
            ),
        ];
        for ((nesting, created_by_the_run), want) in oracle {
            assert_eq!(
                lock_dir_origin(nesting, created_by_the_run),
                want,
                "lock_dir_origin({nesting:?}, created_by_the_run={created_by_the_run})"
            );
        }
        // Anti-collapse: a bool cannot hold three values across these four
        // inputs, so a classifier that lost a state would make one of these
        // pairs equal. Asserted directly rather than inferred from the table.
        assert_ne!(
            lock_dir_origin(Nesting::InsideAnOuterAcquisition, true),
            lock_dir_origin(Nesting::InsideAnOuterAcquisition, false),
            "the two NESTED inputs are the pair a bool merges"
        );
        assert_ne!(
            lock_dir_origin(Nesting::Outermost, true),
            lock_dir_origin(Nesting::InsideAnOuterAcquisition, true),
            "a nested guard must not claim the creation its outer guard made"
        );
    }

    /// A wait the caller cannot end is the hazard this variant exists to
    /// avoid: `ctrlc` installs its handler with `SA_RESTART`, so the blocking
    /// `flock` that `acquire` uses is auto-restarted and answers no signal.
    /// The predicate is consulted BEFORE the first sleep, so this test has no
    /// wall in it.
    #[tracing_test::traced_test]
    #[test]
    fn an_interrupted_wait_reports_none_and_takes_no_lock() {
        let temp = tempfile::tempdir().unwrap();
        drop(acquire_quiet(temp.path()));
        let foreign = foreign_description(temp.path());
        // SAFETY: `foreign` is open for the whole test.
        assert_eq!(0, unsafe {
            libc::flock(foreign.as_raw_fd(), libc::LOCK_EX)
        });

        let outcome = WorkspaceLock::acquire_interruptibly(temp.path(), &|| true);
        assert!(
            matches!(outcome, Err(super::AcquireError::Interrupted)),
            "an interrupted wait must report Interrupted, never a lock it does not hold"
        );
        // The production code puts the contention `warn!` INSIDE each wait
        // arm, below the predicate gate, so a caller whose interrupt is
        // already set is never told it is "waiting for it to finish" — a line
        // about a wait that never happened. Hoisting the warn above that gate
        // passed the whole suite until this assertion existed.
        //
        // The count is checked ONCE, at the end, against a capture that is
        // proven live by a REAL contended wait below. An `== 0` guard on its
        // own would also pass if the capture were broken (the `no-env-filter`
        // feature dropped, the subscriber slot taken), which is the vacuity
        // this repo pairs every absence guard against.

        // ANTI-TAUTOLOGY, and the reservation-release pin: with the foreign
        // lock gone and no interrupt the SAME call acquires. A leaked registry
        // reservation would park this forever, so it runs on a thread with a
        // deadline and fails rather than hanging the binary.
        // SAFETY: as above.
        assert_eq!(0, unsafe {
            libc::flock(foreign.as_raw_fd(), libc::LOCK_UN)
        });
        // ANTI-VACUITY for the log assertion, and the reservation-release pin
        // in one: re-take the foreign lock, contend for real on THIS thread
        // (so the capture can see it), and release from a helper. Exactly one
        // warn must come out of the whole test — none from the interrupted
        // call above, one from this genuine wait.
        // SAFETY: as above.
        assert_eq!(0, unsafe {
            libc::flock(foreign.as_raw_fd(), libc::LOCK_EX)
        });
        let releaser_fd = foreign.try_clone().expect("dup the lock descriptor");
        let polls = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let polls_helper = polls.clone();
        let releaser = std::thread::spawn(move || {
            let deadline = std::time::Instant::now() + Duration::from_secs(30);
            while polls_helper.load(std::sync::atomic::Ordering::SeqCst) < 2
                && std::time::Instant::now() < deadline
            {
                std::thread::sleep(Duration::from_millis(20));
            }
            // SAFETY: the helper owns `releaser_fd` for the whole call.
            unsafe { libc::flock(releaser_fd.as_raw_fd(), libc::LOCK_UN) };
        });
        let lock = WorkspaceLock::acquire_interruptibly(temp.path(), &|| {
            polls.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1 > POLL_DEADLINE
        })
        .expect("the lock must be takeable again after an interrupted wait");
        releaser.join().unwrap();
        drop(lock);

        logs_assert(|lines: &[&str]| {
            let warns = lines
                .iter()
                .filter(|line| line.contains("workspace is locked by"))
                .count();
            if warns == 1 {
                Ok(())
            } else {
                Err(format!(
                    "expected exactly 1 warn — none from the ALREADY-interrupted call, one \
                     from the genuine wait — got {warns}: {lines:?}"
                ))
            }
        });
    }

    /// The lock is never taken THROUGH a link: a planted
    /// `<root>/.cerulion -> elsewhere` would otherwise put the lock file at a
    /// path of somebody else's choosing and flock a file that is not this
    /// workspace's. Both variants refuse, and the planted target stays empty —
    /// the outcome, not merely the error text.
    #[test]
    fn a_symlinked_lock_path_is_refused_and_never_followed() {
        let temp = tempfile::tempdir().unwrap();
        let outside = tempfile::tempdir().unwrap();
        std::os::unix::fs::symlink(outside.path(), temp.path().join(LOCK_DIR)).unwrap();
        for (label, err) in [
            (
                "acquire",
                refusal_text(WorkspaceLock::acquire(temp.path()), "a symlinked .cerulion"),
            ),
            (
                "acquire_interruptibly",
                match WorkspaceLock::acquire_interruptibly(temp.path(), never_interrupted()) {
                    Ok(_) => panic!("a symlinked .cerulion must refuse"),
                    Err(super::AcquireError::Interrupted) => {
                        unreachable!("nothing interrupted this")
                    }
                    Err(super::AcquireError::Internal(error)) => {
                        panic!(
                            "a symlinked .cerulion is the WORKSPACE's condition, not our \
                                bug; got: {error}"
                        )
                    }
                    Err(super::AcquireError::Failed(error)) => error.to_string(),
                },
            ),
        ] {
            assert!(
                err.contains("SYMLINK"),
                "{label} must name the condition; got: {err}"
            );
        }
        assert!(
            std::fs::read_dir(outside.path()).unwrap().next().is_none(),
            "the link was followed — something was created through it"
        );

        // And the final component: a planted `workspace.lock -> …` is refused
        // by O_NOFOLLOW rather than created at the link's target — through
        // BOTH variants, and with the SAME crafted message as the directory
        // half. Left as a bare `?`, ELOOP reached every caller but migrate as
        // "I/O error: Too many levels of symbolic links": no path, no remedy.
        for (label, take) in [
            (
                "acquire",
                &(|root: &Path| WorkspaceLock::acquire(root)) as &dyn Fn(&Path) -> CliResult<_>,
            ),
            (
                "acquire_interruptibly",
                &(|root: &Path| match WorkspaceLock::acquire_interruptibly(
                    root,
                    never_interrupted(),
                ) {
                    Ok(lock) => Ok(lock),
                    Err(super::AcquireError::Interrupted) => unreachable!("not interrupted"),
                    Err(super::AcquireError::Internal(error)) => {
                        panic!(
                            "a planted link is the WORKSPACE's condition, not our bug; \
                                got: {error}"
                        )
                    }
                    Err(super::AcquireError::Failed(error)) => Err(error),
                }) as &dyn Fn(&Path) -> CliResult<_>,
            ),
        ] {
            let planted = tempfile::tempdir().unwrap();
            let target = planted.path().join("elsewhere");
            std::fs::create_dir_all(planted.path().join(LOCK_DIR)).unwrap();
            std::os::unix::fs::symlink(&target, planted.path().join(LOCK_DIR).join(LOCK_FILE))
                .unwrap();
            let err = refusal_text(take(planted.path()), "a symlinked lock file");
            assert!(
                err.contains("SYMLINK"),
                "{label} must name the condition for the final component too; got: {err}"
            );
            assert!(
                !target.exists(),
                "{label}: the lock file link was followed into a create"
            );
        }
    }

    /// An IN-PROCESS wait is interruptible too. Waiting out
    /// a same-process peer on the registry condvar without
    /// consulting the predicate would ignore a Ctrl-C during that wait, so
    /// only a cross-process contender could be escaped. The holder here is
    /// another THREAD, so the kernel `flock` is never reached and the registry
    /// park is the only thing under test.
    ///
    /// No wall in it: the predicate trips on its second call, and the holder is
    /// released only after the refusal has been observed, so a slow machine
    /// lengthens the park without changing the verdict.
    #[test]
    fn an_in_process_wait_honours_the_interrupt_predicate() {
        let temp = tempfile::tempdir().unwrap();
        let held = acquire_quiet(temp.path());

        let calls = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let root = temp.path().to_path_buf();
        let calls_worker = calls.clone();
        let (done_tx, done_rx) = mpsc::channel();
        let worker = std::thread::spawn(move || {
            let outcome = WorkspaceLock::acquire_interruptibly(&root, &|| {
                calls_worker.fetch_add(1, std::sync::atomic::Ordering::SeqCst) > 0
            });
            done_tx.send(matches!(outcome, Err(super::AcquireError::Interrupted)))
        });

        assert!(
            done_rx
                .recv_timeout(Duration::from_secs(10))
                .expect("an in-process wait never returned — the park ignored the predicate"),
            "an interrupted in-process wait must report Interrupted"
        );
        assert!(
            calls.load(std::sync::atomic::Ordering::SeqCst) >= 2,
            "the predicate must be consulted by the PARK, not once before it"
        );
        drop(held);
        worker.join().unwrap().unwrap();
    }

    /// Whichever variant is asked for, it is ONE lock: the second acquisition
    /// waits for the first, in both orders. (This arm is INTRA-process, so it
    /// is resolved in the reentrancy registry and never reaches `flock` — the
    /// kernel half is `acquire_interruptibly_takes_an_exclusive_kernel_flock`.)
    ///
    /// The negative window is paired with a HAPPENS-BEFORE:
    /// a 300 ms "did not acquire" can be satisfied by a thread that was merely
    /// descheduled, so the acquirer also reads a flag the main thread sets
    /// immediately before releasing. Reading it as set proves the acquire
    /// completed AFTER the release rather than merely late.
    #[test]
    fn both_acquire_variants_wait_for_each_other() {
        for quiet_first in [false, true] {
            let temp = tempfile::tempdir().unwrap();
            let held = if quiet_first {
                acquire_quiet(temp.path())
            } else {
                WorkspaceLock::acquire(temp.path()).unwrap()
            };
            let (about_to_acquire_tx, about_to_acquire_rx) = mpsc::channel();
            let (acquired_tx, acquired_rx) = mpsc::channel();
            let root = temp.path().to_path_buf();
            let released = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
            let released_worker = released.clone();
            let thread = std::thread::spawn(move || {
                about_to_acquire_tx.send(()).unwrap();
                let _lock = if quiet_first {
                    WorkspaceLock::acquire(&root).unwrap()
                } else {
                    acquire_quiet(&root)
                };
                // Read IMMEDIATELY after acquiring: the main thread sets this
                // just before it releases, so a 1 here is proof the acquire
                // happened after the release and not merely late.
                acquired_tx
                    .send(released_worker.load(std::sync::atomic::Ordering::SeqCst))
                    .unwrap();
            });
            about_to_acquire_rx.recv().unwrap();
            assert!(
                acquired_rx
                    .recv_timeout(Duration::from_millis(300))
                    .is_err(),
                "quiet_first={quiet_first}: the other variant acquired while a guard was held"
            );
            released.store(1, std::sync::atomic::Ordering::SeqCst);
            drop(held);
            let saw_release = acquired_rx
                .recv_timeout(Duration::from_secs(10))
                .unwrap_or_else(|e| {
                    panic!("quiet_first={quiet_first}: never acquired after release: {e}")
                });
            assert_eq!(
                saw_release, 1,
                "quiet_first={quiet_first}: the acquire completed BEFORE the release — the \
                 negative window above was satisfied by a descheduled thread, not by the lock"
            );
            thread.join().unwrap();
        }
    }

    /// Reentrancy is a property of the lock, not of one constructor: nesting
    /// the variants in either order reuses the held file description instead
    /// of dead-locking on a second `flock`. The foreign probes are what make
    /// this more than a no-deadlock smoke test — they pin that dropping the
    /// INNER guard does not release the kernel lock the outer one still owns.
    #[test]
    fn every_pair_of_constructors_nests_on_one_thread_in_either_order() {
        // THREE constructors, so every ORDERED pair — the module doc says
        // reentrancy is a property of the lock, not of one constructor, and a
        // two-of-three loop left the tracking one out of a claim made about all
        // of them.
        type Take = fn(&Path) -> WorkspaceLock;
        let takes: [(&str, Take); 3] = [
            ("acquire", |root| WorkspaceLock::acquire(root).unwrap()),
            ("acquire_interruptibly", acquire_quiet),
            ("acquire_and_track_gitignore", |root| {
                WorkspaceLock::acquire_and_track_gitignore(root).unwrap()
            }),
        ];

        let temp = tempfile::tempdir().unwrap();
        drop(WorkspaceLock::acquire(temp.path()).unwrap());
        let foreign = foreign_description(temp.path());

        for (outer_name, take_outer) in takes {
            for (inner_name, take_inner) in takes {
                let pair = format!("{outer_name} outside {inner_name}");
                let outer = take_outer(temp.path());
                let inner = take_inner(temp.path());
                drop(inner);
                assert!(
                    !foreign_can_take(&foreign, libc::LOCK_EX),
                    "{pair}: dropping the inner guard released the lock the outer one holds"
                );
                drop(outer);
                assert!(
                    foreign_can_take(&foreign, libc::LOCK_EX),
                    "{pair}: dropping the outer guard did not release the lock"
                );
            }
        }
    }

    /// The documented consequence of tying the top-up to CREATING
    /// `.cerulion/`, pinned so it cannot change unnoticed: whichever
    /// acquisition creates the directory decides for the workspace. The
    /// tracked-write-free variant creates it, so a later `acquire` finds it
    /// present and tops nothing up — as does anything else that writes there
    /// first (`cerulion ros2 migrate` writes its manifest into
    /// `.cerulion/` with no lock involved).
    #[test]
    fn the_gitignore_top_up_is_tied_to_creating_the_lock_directory() {
        let temp = tempfile::tempdir().unwrap();
        std::fs::write(temp.path().join(".gitignore"), "target/\n").unwrap();
        drop(acquire_quiet(temp.path()));
        drop(WorkspaceLock::acquire_and_track_gitignore(temp.path()).unwrap());
        assert_eq!(
            std::fs::read_to_string(temp.path().join(".gitignore")).unwrap(),
            "target/\n",
            "the top-up is creation-tied, and another constructor was the creator"
        );

        // CONTROL: where the topping acquire IS the creator it still tops up,
        // so the assertion above is about ORDER, not about a removed top-up.
        let fresh = tempfile::tempdir().unwrap();
        std::fs::write(fresh.path().join(".gitignore"), "target/\n").unwrap();
        drop(WorkspaceLock::acquire_and_track_gitignore(fresh.path()).unwrap());
        assert_eq!(
            std::fs::read_to_string(fresh.path().join(".gitignore")).unwrap(),
            "target/\n.cerulion/\n"
        );
    }

    #[test]
    fn a_workspace_without_a_gitignore_gets_none() {
        let temp = tempfile::tempdir().unwrap();
        drop(WorkspaceLock::acquire_and_track_gitignore(temp.path()).unwrap());
        assert!(temp.path().join(LOCK_DIR).join(LOCK_FILE).exists());
        assert!(!temp.path().join(".gitignore").exists());
    }

    /// THE RACE the three states were designed for, driven DETERMINISTICALLY at
    /// the seam rather than by racing real processes.
    ///
    /// `create_dir_all` returns `Ok` for a directory that already exists, so
    /// two acquisitions that both passed a `!lock_dir.is_dir()` check before
    /// either created it BOTH reported the creation, and both ran the
    /// `.gitignore` top-up — which happens before the lock file is opened and
    /// flocked and is therefore unserialised across processes. See the module
    /// docs for what that actually costs (content LOSS through a truncating
    /// rewrite, not a duplicated line).
    ///
    /// A test that really races loses most runs — the window is microseconds
    /// wide — and, worse, a race that never happened produces the SAME passing
    /// state as one that did, so its mutation-killing power has no lower bound.
    /// The seam stands the winning racer up exactly where it would have landed:
    /// `PRE_CREATE_HOOK` fires immediately before this acquisition's create
    /// attempt, i.e. AFTER the point at which an `!is_dir()` check would have
    /// already decided the directory was absent. With `create_dir` the racer's
    /// directory makes this acquisition's create return `AlreadyExists`, so it
    /// is not the creator and tops nothing up; with `!is_dir()` +
    /// `create_dir_all` the create returns `Ok` to BOTH and the tracked file is
    /// rewritten by two unserialised writers.
    ///
    /// SCOPE: this pins the CREATE-window semantics. That the top-up
    /// sits before the flock — i.e. that two processes are unserialised there
    /// at all — is visible by reading the function, and no test here asserts it.
    #[test]
    fn a_racer_that_creates_the_lock_directory_first_leaves_the_top_up_to_nobody_else() {
        struct HookGuard;
        impl Drop for HookGuard {
            fn drop(&mut self) {
                set_pre_create_hook(None);
            }
        }

        let raced = tempfile::tempdir().unwrap();
        let gitignore = raced.path().join(".gitignore");
        std::fs::write(&gitignore, "target/\n").unwrap();
        let racer_target = raced.path().join(LOCK_DIR);
        let guard = HookGuard;
        set_pre_create_hook(Some(Box::new(move || {
            // The racer that won. `create_dir`, not `create_dir_all`, so a
            // second firing could not be mistaken for a fresh creation.
            let _ = std::fs::create_dir(&racer_target);
        })));
        let raced_lock = WorkspaceLock::acquire_and_track_gitignore(raced.path()).unwrap();
        let raced_origin = raced_lock.lock_dir_origin();
        drop(raced_lock);
        drop(guard);

        assert_eq!(
            raced_origin,
            LockDirOrigin::PreExisting,
            "another writer created the lock directory first, so this acquisition created \
             NOTHING — reporting `CreatedHere` here is the whole of the TOCTOU"
        );
        assert_eq!(
            std::fs::read_to_string(&gitignore).unwrap(),
            "target/\n",
            "and it must therefore top nothing up: two writers each rewriting a TRACKED \
             file, unserialised, is how one of them loses the other's content"
        );
        assert!(
            raced.path().join(LOCK_DIR).join(LOCK_FILE).exists(),
            "it must still have taken the lock it serializes on"
        );

        // CONTROL, same fixture, hook OFF: this acquisition IS the creator, so
        // it reports the creation and tops up. Without it the assertions above
        // pass on a build where NOTHING ever creates or tops up.
        let uncontested = tempfile::tempdir().unwrap();
        std::fs::write(uncontested.path().join(".gitignore"), "target/\n").unwrap();
        let control = WorkspaceLock::acquire_and_track_gitignore(uncontested.path()).unwrap();
        assert_eq!(control.lock_dir_origin(), LockDirOrigin::CreatedHere);
        drop(control);
        assert_eq!(
            std::fs::read_to_string(uncontested.path().join(".gitignore")).unwrap(),
            "target/\n.cerulion/\n",
            "the control: an uncontested creation still tops up"
        );
    }

    /// A FAILED top-up refuses AND gives the lock directory back, so a later
    /// acquire retries instead of forfeiting the entry forever.
    ///
    /// The failure is injected by making `.gitignore` a DIRECTORY, so
    /// `read_to_string` fails with something that is neither `NotFound` (which
    /// `ensure_gitignore_entry` treats as "this workspace has none") nor
    /// exempt under root — a permission bit would be inert in the container
    /// that runs as root, which is the environment that ships.
    ///
    /// The SECOND half is the mutation-killer. Without `created_lock_dir =
    /// false` and the `remove_dir`, the retry below takes the `AlreadyExists`
    /// arm, reports `PreExisting`, and tops nothing up — so `.cerulion/` stays
    /// out of a tracked `.gitignore` for the life of the workspace while every
    /// later verb succeeds quietly. The first half alone would pass on a build
    /// that never created the directory at all.
    #[test]
    fn a_failed_gitignore_top_up_gives_the_lock_directory_back_so_a_later_acquire_retries() {
        let temp = tempfile::tempdir().unwrap();
        std::fs::create_dir(temp.path().join(".gitignore")).unwrap();

        let text = refusal_text(
            WorkspaceLock::acquire_and_track_gitignore(temp.path()),
            "a top-up that cannot read .gitignore",
        );
        assert!(
            text.contains(".gitignore"),
            "the refusal must name the file it could not write; got: {text}"
        );
        assert!(
            text.contains("re-run once that path is writable"),
            "…and must tell the user what happens next; got: {text}"
        );
        assert!(
            !temp.path().join(LOCK_DIR).exists(),
            "the creator must GIVE THE DIRECTORY BACK; otherwise the top-up is forfeited \
             for the life of the workspace"
        );

        // THE RETRY. Fix the cause, and the next acquire is a fresh CREATION
        // that tops up — which is the whole point of giving the directory back.
        std::fs::remove_dir(temp.path().join(".gitignore")).unwrap();
        std::fs::write(temp.path().join(".gitignore"), "target/\n").unwrap();
        let lock = WorkspaceLock::acquire_and_track_gitignore(temp.path()).unwrap();
        assert_eq!(
            lock.lock_dir_origin(),
            LockDirOrigin::CreatedHere,
            "the retry must be a fresh creation, not an AlreadyExists"
        );
        drop(lock);
        assert_eq!(
            std::fs::read_to_string(temp.path().join(".gitignore")).unwrap(),
            "target/\n.cerulion/\n",
            "and it must actually top up the entry the refused acquire could not"
        );
    }

    /// A racer's give-back landing between this acquisition's DIRECTORY
    /// DESCRIPTOR and its `openat` is recovered too — which needs the
    /// descriptor RE-DERIVED, not just the directory remade.
    ///
    /// Same severe class as the sibling arm below, one step later, and the one
    /// a retry loop gets wrong by default: an `openat` through a descriptor
    /// whose directory has been removed fails `ENOENT` UNCONDITIONALLY
    /// (MEASURED: three attempts through the frozen descriptor,
    /// all `ENOENT`; remaking the directory AND re-deriving the descriptor
    /// succeeds). A file-open retry that reuses the descriptor therefore
    /// retries a corpse and still reports `workspace_not_found`.
    ///
    /// `PRE_FILE_OPEN_HOOK` fires inside the lock-file open, after the
    /// descriptor is taken and before the `openat` — the only window the bug
    /// lives in.
    #[test]
    fn a_give_back_after_the_directory_descriptor_is_recovered_by_re_deriving_it() {
        struct HookGuard;
        impl Drop for HookGuard {
            fn drop(&mut self) {
                set_pre_file_open_hook(None);
            }
        }

        let temp = tempfile::tempdir().unwrap();
        std::fs::write(temp.path().join(".gitignore"), "target/\n").unwrap();
        let lock_dir = temp.path().join(LOCK_DIR);
        let fired = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let fired_hook = fired.clone();
        let target = lock_dir.clone();
        let guard = HookGuard;
        set_pre_file_open_hook(Some(Box::new(move || {
            if !fired_hook.swap(true, std::sync::atomic::Ordering::SeqCst) {
                // The peer's give-back, landing AFTER this call took its
                // directory descriptor.
                let _ = std::fs::remove_dir(&target);
            }
        })));
        let outcome = WorkspaceLock::acquire_and_track_gitignore(temp.path());
        drop(guard);

        assert!(
            fired.load(std::sync::atomic::Ordering::SeqCst),
            "the seam never fired, so this arm proves nothing"
        );
        let lock = outcome.expect(
            "a directory removed after the descriptor was taken must be remade AND the \
             descriptor re-derived — retrying the same descriptor retries a corpse and \
             still tells a second user their workspace does not exist",
        );
        drop(lock);
        assert!(
            lock_dir.join(LOCK_FILE).exists(),
            "and the recovery must leave a real lock behind"
        );
        assert_eq!(
            std::fs::read_to_string(temp.path().join(".gitignore")).unwrap(),
            "target/\n.cerulion/\n",
            "the attempt that created the surviving directory still owes the tracked write"
        );
    }

    /// A racer's give-back landing between this acquisition's create and its
    /// directory open is RECOVERED, not reported.
    ///
    /// The window is the one the `.gitignore` give-back itself opens: A
    /// creates the directory, A's top-up fails, A removes the directory — while
    /// B, which took the `AlreadyExists` arm, is sitting between its create and
    /// its open. Reported rather than recovered it surfaces as `Io(NotFound)`,
    /// which `cerulion_wsd` maps to `workspace_not_found`: a daemon telling B's
    /// user their workspace does not exist because A's `.gitignore` was
    /// unwritable.
    ///
    /// `PRE_OPEN_HOOK` fires immediately before the directory open, which is
    /// exactly where A's `remove_dir` lands, so the race is reproduced
    /// deterministically instead of waited for. The hook removes the directory
    /// ONCE; the recovery must remake it and carry on.
    #[test]
    fn a_racers_give_back_between_the_create_and_the_open_is_recovered_not_reported() {
        struct HookGuard;
        impl Drop for HookGuard {
            fn drop(&mut self) {
                set_pre_open_hook(None);
            }
        }

        let temp = tempfile::tempdir().unwrap();
        // A `.gitignore` WITHOUT the entry, because the second half of this arm
        // is that a TRACKING acquisition which remakes the directory still owes
        // the tracked write.
        std::fs::write(temp.path().join(".gitignore"), "target/\n").unwrap();
        let lock_dir = temp.path().join(LOCK_DIR);
        let removed = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let removed_hook = removed.clone();
        let target = lock_dir.clone();
        let guard = HookGuard;
        set_pre_open_hook(Some(Box::new(move || {
            if !removed_hook.swap(true, std::sync::atomic::Ordering::SeqCst) {
                // A's give-back, landing in B's window.
                let _ = std::fs::remove_dir(&target);
            }
        })));
        let outcome = WorkspaceLock::acquire_and_track_gitignore(temp.path());
        drop(guard);

        assert!(
            removed.load(std::sync::atomic::Ordering::SeqCst),
            "the seam never fired, so this arm proves nothing"
        );
        let lock = outcome.expect(
            "a directory removed between the create and the open must be REMADE, not \
             reported — reporting it tells a second user their workspace does not exist",
        );
        assert_eq!(
            lock.lock_dir_origin(),
            LockDirOrigin::CreatedHere,
            "after the remake THIS acquisition is the one that created the directory that \
             survives, and it must say so"
        );
        drop(lock);
        assert!(
            lock_dir.join(LOCK_FILE).exists(),
            "and the recovery must leave a real lock behind"
        );
        // THE SECOND HALF: a recovery that
        // remade the directory but skipped the top-up leaves `.cerulion/`
        // permanently absent from a TRACKED file — the forfeiture the give-back
        // exists to prevent, re-entered through the recovery.
        assert_eq!(
            std::fs::read_to_string(temp.path().join(".gitignore")).unwrap(),
            "target/\n.cerulion/\n",
            "whoever creates the directory that survives owes the tracked write, however \
             many attempts it took"
        );
    }

    /// The RECOVERY attempt owes the tracked write when IT is the one that
    /// creates the surviving directory.
    ///
    /// The sibling arm above cannot see this. There the FIRST attempt creates
    /// the directory, so the top-up has already happened by the time the
    /// give-back lands, and its `.gitignore` assertion is satisfied by attempt
    /// one no matter what the recovery does. A variant that makes only
    /// the recovery skip the top-up passes it.
    ///
    /// The sequence this arm reproduces: a tracking acquisition receives
    /// `AlreadyExists` — so it writes NOTHING — and only then does the creator
    /// give the directory back. The acquisition that remakes it is the first
    /// one here to create anything, and if it does not top up, `.cerulion/`
    /// stays permanently absent from a TRACKED `.gitignore` while every later
    /// verb succeeds quietly: the forfeiture the give-back exists to prevent,
    /// re-entered through the recovery.
    ///
    /// Both seams are needed, and they fire at different points: `PRE_CREATE`
    /// (once) stands the racer up so attempt one takes `AlreadyExists`, and
    /// `PRE_OPEN` (once) is the give-back landing in this call's window.
    #[test]
    fn a_recovery_that_creates_the_surviving_directory_owes_the_tracked_write() {
        struct HookGuard;
        impl Drop for HookGuard {
            fn drop(&mut self) {
                set_pre_create_hook(None);
                set_pre_open_hook(None);
            }
        }

        let temp = tempfile::tempdir().unwrap();
        std::fs::write(temp.path().join(".gitignore"), "target/\n").unwrap();
        let lock_dir = temp.path().join(LOCK_DIR);
        let guard = HookGuard;

        // The racer wins the FIRST attempt only, so this acquisition takes the
        // `AlreadyExists` arm and owes nothing yet.
        let created_by_racer = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let racer_flag = created_by_racer.clone();
        let racer_target = lock_dir.clone();
        set_pre_create_hook(Some(Box::new(move || {
            if !racer_flag.swap(true, std::sync::atomic::Ordering::SeqCst) {
                let _ = std::fs::create_dir(&racer_target);
            }
        })));

        // Then the racer gives it back, in this call's create-to-open window.
        let removed = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let removed_hook = removed.clone();
        let remove_target = lock_dir.clone();
        set_pre_open_hook(Some(Box::new(move || {
            if !removed_hook.swap(true, std::sync::atomic::Ordering::SeqCst) {
                let _ = std::fs::remove_dir(&remove_target);
            }
        })));

        let lock = WorkspaceLock::acquire_and_track_gitignore(temp.path())
            .expect("the directory must be remade, not reported");
        let origin = lock.lock_dir_origin();
        drop(lock);
        drop(guard);

        assert!(
            created_by_racer.load(std::sync::atomic::Ordering::SeqCst)
                && removed.load(std::sync::atomic::Ordering::SeqCst),
            "both seams must have fired, or this arm proves nothing"
        );
        assert_eq!(
            origin,
            LockDirOrigin::CreatedHere,
            "the recovery created the directory that survives, so this acquisition is its \
             creator even though its first attempt was told it already existed"
        );
        assert_eq!(
            std::fs::read_to_string(temp.path().join(".gitignore")).unwrap(),
            "target/\n.cerulion/\n",
            "and the creator of the surviving directory owes the tracked write — a recovery \
             that remakes the directory but skips the top-up re-enters the permanent \
             forfeiture the give-back exists to prevent"
        );
    }

    /// The spurious `ENOENT` the lock-file open retries, injected rather than
    /// raced for.
    ///
    /// MEASURED with a C model of exactly this syscall sequence
    /// (lstat, mkdir, open the directory, `openat` the file), children released
    /// together by a gate file: at 8 racers, 61 of 160 children failed with ONE
    /// attempt and 0 of 160 failed with two — 73 of them needing the second.
    /// It reproduces at TWO processes, which is `cerulion node create` racing
    /// `cerulion-wsd` on the first mutation of any workspace. Left unretried it
    /// is worse than a wrong message: the refusal keeps `ErrorKind::NotFound`,
    /// which is what `cerulion_wsd` maps to `workspace_not_found` — a daemon
    /// telling a user their workspace does not exist while they look at it.
    ///
    /// The race itself is probabilistic, so a test built on one proves nothing
    /// on the run where the racers miss each other. The seam injects the exact
    /// observable, which makes the oracle deterministic in BOTH directions:
    /// within the budget the acquire must succeed, and past it the failure must
    /// still be reported rather than spun on.
    #[test]
    fn a_spurious_enoent_from_the_lock_file_open_is_absorbed_within_the_budget() {
        // RAII, mirroring `HookGuard`: every arm below drains the counter to
        // zero, but an acquire that failed BEFORE reaching `open_lock_file`
        // would leave it armed, and the discipline should not depend on that.
        struct SeamGuard;
        impl Drop for SeamGuard {
            fn drop(&mut self) {
                fail_next_lock_opens_with_enoent(0);
            }
        }
        let _seam = SeamGuard;

        let temp = tempfile::tempdir().unwrap();
        // One injected failure: the shape every measured trial recovered from.
        fail_next_lock_opens_with_enoent(1);
        let lock = WorkspaceLock::acquire(temp.path())
            .expect("a spurious ENOENT must be retried, not reported");
        drop(lock);
        assert!(
            temp.path().join(LOCK_DIR).join(LOCK_FILE).exists(),
            "and the retry must actually have created the lock file"
        );

        // The whole budget minus one: still absorbed.
        let edge = tempfile::tempdir().unwrap();
        fail_next_lock_opens_with_enoent(super::LOCK_OPEN_ATTEMPTS - 1);
        drop(
            WorkspaceLock::acquire(edge.path())
                .expect("the budget must absorb up to its last attempt"),
        );

        // PAST the budget the refusal STANDS. Without this the retry could be
        // an unbounded spin and every assertion above would still pass.
        let past = tempfile::tempdir().unwrap();
        fail_next_lock_opens_with_enoent(super::LOCK_OPEN_ATTEMPTS);
        let text = refusal_text(
            WorkspaceLock::acquire(past.path()),
            "an ENOENT that outlasts the budget",
        );
        assert!(
            text.contains("opening the workspace lock at"),
            "a condition that outlasts the budget must be REPORTED, naming the path; got: {text}"
        );
        // And the seam is left disarmed, so a sibling acquire on this thread is
        // unaffected.
        drop(WorkspaceLock::acquire(past.path()).expect("the seam is spent"));
    }

    /// The REGRESSION IS A HANG, so the deadline is the
    /// oracle: the whole scenario runs on a spawned thread and the test thread
    /// bounds it with `recv_timeout`. A reverted refusal parks that thread in
    /// `flock(LOCK_SH)` forever — nothing can wake it, because the only holder
    /// of the exclusive lock is the parked thread itself — and this fails with
    /// a named timeout instead of wedging the suite.
    ///
    /// The ANTI-TAUTOLOGY half is in the same body and is the more important
    /// one: a read contending with ANOTHER thread must still WAIT and then
    /// SUCCEED. Without it, "refuse every read while anything is locked" would
    /// pass the first half perfectly.
    #[test]
    fn a_read_from_the_thread_that_holds_the_write_lock_is_refused_not_parked() {
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path().to_path_buf();
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let held = WorkspaceLock::acquire(&root).expect("the exclusive lock");
            // SAME THREAD, SAME ROOT: a second file description contending
            // with the one this thread holds.
            let outcome = WorkspaceLock::acquire_read(&root);
            tx.send(match outcome {
                Ok(guard) => format!("SUCCEEDED, is_locked={}", guard.is_locked()),
                Err(error) => error.to_string(),
            })
            .ok();
            drop(held);
        });
        let text = rx.recv_timeout(Duration::from_secs(20)).expect(
            "the read never returned — a thread that holds the exclusive lock parked forever \
             on its own lock instead of being refused",
        );
        assert!(
            text.contains("already holds") && text.contains("EXCLUSIVE lock"),
            "the refusal must name what the caller did; got: {text}"
        );
        assert!(
            text.contains("bug in cerulion_cli_engine"),
            "a caller cannot fix this by changing their workspace, so the refusal must not \
             send them to look at one; got: {text}"
        );

        // ANTI-TAUTOLOGY: a read from a DIFFERENT thread is ordinary
        // contention. It must wait for the writer and then take a real shared
        // lock — not inherit the refusal above.
        let holding = tempfile::tempdir().unwrap();
        let held_root = holding.path().to_path_buf();
        let (holding_tx, holding_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel::<()>();
        let writer = std::thread::spawn(move || {
            let held = WorkspaceLock::acquire(&held_root).expect("the exclusive lock");
            holding_tx.send(()).unwrap();
            let _ = release_rx.recv_timeout(Duration::from_secs(30));
            drop(held);
        });
        holding_rx
            .recv_timeout(Duration::from_secs(10))
            .expect("the writer never took the lock");
        // THAT a read on another thread is not REFUSED is the claim here, and
        // that it ends up holding a real shared lock. That it WAITED is
        // asserted against the KERNEL rather than a wall: a foreign description
        // cannot take `LOCK_SH` while this writer holds `LOCK_EX`, so any read
        // reaching the flock must block. Stated precisely because the earlier
        // shape — a negative `recv_timeout` window — proved less than it looked:
        // on a loaded runner it is satisfied by a reader thread that never got
        // scheduled at all.
        let foreign = foreign_description(holding.path());
        assert!(
            !foreign_can_take(&foreign, libc::LOCK_SH),
            "the writer must genuinely exclude a shared lock, or nothing below is a wait"
        );
        let read_root = holding.path().to_path_buf();
        let (read_tx, read_rx) = mpsc::channel();
        std::thread::spawn(move || {
            read_tx
                .send(WorkspaceLock::acquire_read(&read_root).map(|guard| guard.is_locked()))
                .ok();
        });
        release_tx.send(()).ok();
        let locked = read_rx
            .recv_timeout(Duration::from_secs(20))
            .expect("the read never returned after the writer released")
            .expect("a read contending with ANOTHER thread must succeed, not be refused");
        assert!(locked, "and it must hold a real shared lock");
        writer.join().unwrap();
    }

    /// The pure half of item 9, over its whole domain. The OWNER check is what
    /// separates "this thread would wait on itself" from ordinary contention,
    /// and dropping it turns every contended read into a refusal.
    #[test]
    fn a_read_waits_on_this_thread_only_for_this_threads_own_entry() {
        let me = std::thread::current().id();
        let elsewhere = std::thread::spawn(|| std::thread::current().id())
            .join()
            .unwrap();
        let promoted = |owner| RegistryEntry {
            owner,
            // A `Weak` with no `Arc` behind it never upgrades, which is what
            // makes this constructible without a real lock; the function keys
            // on the OWNER, not on whether it upgrades.
            inner: Some(Weak::<Inner>::new()),
        };
        let reserved = |owner| RegistryEntry { owner, inner: None };

        assert!(
            !read_waits_on_this_thread(None, me),
            "no entry at all is not contention"
        );
        assert!(
            read_waits_on_this_thread(Some(&promoted(me)), me),
            "this thread's COMPLETED acquisition is the deadlock the refusal exists for"
        );
        assert!(
            read_waits_on_this_thread(Some(&reserved(me)), me),
            "this thread's own in-flight acquire counts too — the read succeeds and moves \
             the deadlock onto the outer acquire"
        );
        assert!(
            !read_waits_on_this_thread(Some(&promoted(elsewhere)), me),
            "a PEER thread's lock is ordinary contention and must be waited out"
        );
        assert!(
            !read_waits_on_this_thread(Some(&reserved(elsewhere)), me),
            "a peer's in-flight acquire is ordinary contention too"
        );
    }

    /// The lock directory is opened for SEARCH, not for
    /// READING, so a `.cerulion` at mode `0311` still works.
    ///
    /// TWO halves, and the first is deliberately environment-INDEPENDENT: the
    /// flags themselves must carry this platform's search-only bit. Running as
    /// root (the ros2-bench container does) bypasses permission checks entirely, so
    /// a purely behavioural arm would turn a false FAIL into a false PASS in
    /// exactly the environment that ships — this half kills reverting to
    /// `O_RDONLY` everywhere, uid included.
    ///
    /// The second half is behavioural and PROVES ITS OWN PRECONDITION rather
    /// than skipping on `id -u`: it first attempts the OLD open shape on the
    /// same `0311` directory. If that is refused, permission bits are being
    /// enforced for this process and the arm asserts the discriminating fact
    /// (the new shape opens where the old one could not). If it SUCCEEDS,
    /// nothing here enforces permissions and the arm still asserts the new
    /// shape works — a weaker claim, stated as such, on a run that cannot make
    /// the stronger one.
    #[test]
    fn the_lock_directory_is_opened_for_search_not_for_reading() {
        #[cfg(target_vendor = "apple")]
        assert_eq!(
            LOCK_DIR_OPEN_FLAGS & libc::O_SEARCH,
            libc::O_SEARCH,
            "Apple's search-only flag must be set, or a 0311 `.cerulion` fails EACCES"
        );
        #[cfg(any(target_os = "linux", target_os = "android"))]
        assert_eq!(
            LOCK_DIR_OPEN_FLAGS & libc::O_PATH,
            libc::O_PATH,
            "Linux's search-only flag must be set, or a 0311 `.cerulion` fails EACCES"
        );
        assert_eq!(
            LOCK_DIR_OPEN_FLAGS & libc::O_DIRECTORY,
            libc::O_DIRECTORY,
            "the walk still has to refuse a non-directory"
        );
        assert_eq!(
            LOCK_DIR_OPEN_FLAGS & libc::O_NOFOLLOW,
            libc::O_NOFOLLOW,
            "the walk still has to refuse a link"
        );

        // THE CALL-SITE HALF. The flag assertions above pin the CONSTANT; they
        // say nothing about a call site that stops using it, and under root the
        // behavioural half below cannot discriminate at all (permission bits are
        // not enforced, so a read-requiring open succeeds too).
        //
        // Every needle is SPLIT with `concat!` so it does not occur verbatim in
        // this file: a walk whose own needle is a literal counts itself, and the
        // count IS the assertion. Splitting is not sufficient on its own — see
        // the SCOPING below, which is the half that is easy to get wrong.
        //
        // The view is WHITESPACE-COLLAPSED because rustfmt decides where a call
        // wraps: a line-based count reads 1 only because the fallback's arguments
        // happen to land on two lines, leaving the guard blind to exactly the
        // shape a real second call site would take.
        let source = include_str!("workspace_lock.rs");
        let collapsed: String = source.chars().filter(|c| !c.is_whitespace()).collect();
        let dir_opens = collapsed
            .matches(concat!("libc::", "open(dir_path.as_ptr()"))
            .count();
        assert_eq!(
            dir_opens, 2,
            "the lock directory is opened in exactly TWO places, BOTH inside `open_lock_dir`: \
             the search-only open and its documented EINVAL fallback. A third is a call site \
             that can drift away from LOCK_DIR_OPEN_FLAGS; found {dir_opens}"
        );
        assert!(
            collapsed.contains(concat!(
                "libc::",
                "open(dir_path.as_ptr(),LOCK_DIR_OPEN_",
                "FLAGS)"
            )),
            "the primary open must pass LOCK_DIR_OPEN_FLAGS — a call site that spells the \
             flags out again is exactly the revert the constant assertions above cannot see"
        );
        // The EINVAL fallback, asserted rather than allowed silently — and
        // SCOPED to `open_lock_dir`'s BODY. Over the whole file this assertion
        // is TAUTOLOGICAL: the precondition probe further down this very test
        // spells the same flag list, so deleting the production fallback left it
        // green. The `concat!` split stops a needle matching its own literal; it
        // does nothing about the test's own CODE matching it.
        //
        // Sliced on the UNcollapsed source, because a top-level function body
        // ends at the first column-0 `}` and collapsing erases that landmark.
        let after_signature = source
            .split_once(concat!("fn open_lock_", "dir(dir_path: &CString)"))
            .expect("open_lock_dir is where the directory open lives")
            .1;
        let body: String = after_signature
            .split_once("\n}\n")
            .expect("a top-level fn body ends at the first column-0 brace")
            .0
            .chars()
            .filter(|c| !c.is_whitespace())
            .collect();
        assert!(
            body.contains(concat!(
                "libc::O_RDONLY|libc::O_DIRECTORY|libc::",
                "O_NOFOLLOW|libc::O_CLOEXEC,"
            )),
            "the EINVAL fallback must still exist IN `open_lock_dir`, or a platform that \
             rejects the search-only flag has no way back"
        );

        let temp = tempfile::tempdir().unwrap();
        // Create the lock through the ordinary path, then narrow the directory.
        drop(WorkspaceLock::acquire(temp.path()).unwrap());
        let lock_dir = temp.path().join(LOCK_DIR);
        let narrow = std::fs::Permissions::from_mode(0o311);
        std::fs::set_permissions(&lock_dir, narrow).unwrap();

        // The precondition probe: the shape this fix replaced.
        let c_dir = std::ffi::CString::new(lock_dir.as_os_str().as_bytes()).unwrap();
        // SAFETY: `c_dir` is NUL-terminated and outlives the call.
        let old_shape = unsafe {
            libc::open(
                c_dir.as_ptr(),
                libc::O_RDONLY | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC,
            )
        };
        let enforced = if old_shape < 0 {
            true
        } else {
            // SAFETY: a descriptor this call owns.
            unsafe { libc::close(old_shape) };
            false
        };

        // Both outcomes are captured BEFORE anything can panic, and the
        // directory is widened again immediately: an assertion that fired while
        // `.cerulion` was still `0311` would leave a directory the tempdir
        // cannot remove.
        let read = WorkspaceLock::acquire_read(temp.path()).map(|guard| guard.is_locked());
        let write = WorkspaceLock::acquire(temp.path()).map(drop);
        std::fs::set_permissions(&lock_dir, std::fs::Permissions::from_mode(0o755)).unwrap();

        assert!(
            read.as_ref().is_ok_and(|locked| *locked),
            "a 0311 lock directory must still be readable AND take a real shared lock — it \
             needs SEARCH, not READ; got: {read:?}"
        );
        assert!(
            write.is_ok(),
            "a 0311 lock directory must still be writable; got: {write:?}"
        );
        if !enforced {
            // Not a skip: every assertion above ran. This one line records
            // that the DISCRIMINATING comparison was unavailable here.
            eprintln!(
                "note: this environment does not enforce directory permissions for this \
                 process (root?), so the old open shape succeeded too; the flags assertions \
                 above are what pin the fix here"
            );
        }
    }
}
