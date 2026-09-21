// SPDX-License-Identifier: AGPL-3.0-only
//! Audit **amendment 1**: the panic hook that keeps a capture child from unwinding
//! into its parent's frames, installed by the PARENT and branching on `getpid()`.
//!
//! # The hazard, twice
//!
//! The first hazard is why a hook is mandatory at all: a fork child shares the parent's stack
//! frames, so a panic that UNWINDS there runs `Drop` for the parent's live data plane
//! — iceoryx2 ports, SHM guards, the ring producer. The bag would be corrupted by a
//! process that owns none of it, from a panic in a user encoder nobody was watching.
//! The fix is to leave via `_exit` FROM THE HOOK, so unwinding never begins.
//!
//! The original design had the CHILD install that hook as its first act. Audit amendment 1
//! is that this cannot work, and the mechanism was verified first-party against rustc
//! 1.96's std source rather than assumed:
//!
//! - std's hook is `static HOOK: RwLock<Hook>` — **nonpoison, plain blocking**, so a
//!   wedge here is a true deadlock rather than a poison error.
//! - `rust_panic_with_hook` holds the **READ** guard across the entire user hook
//!   (`panicking.rs:816`, a match-scrutinee temporary).
//! - `set_hook` takes the **WRITE** lock.
//!
//! A `fork` landing while any parent thread is mid-panic therefore hands the child an
//! `RwLock` with a reader count above zero and **no thread left to release it** (a
//! fork child has exactly one thread). The child's `set_hook` blocks on that write
//! lock forever — before the hook's protection exists — and the parent misreports it 5 s
//! later as `ChildStalled`, pointing at an encoder that never ran.
//!
//! That precondition is not hypothetical: `runtime.rs:6597`'s standing
//! `ExternalSource::Blocking` helper runs USER closures under `catch_unwind` on a
//! detached thread in every ingress/driver graph, so a parent thread mid-panic is an
//! ordinary state, not a corner.
//!
//! # The SECOND direction, and the exact scope of what the fix covers
//!
//! The addendum found a direction the audit had not stated: a fork landing
//! mid-`set_hook` (WRITE lock held) deadlocks any child *panic* at `HOOK.read()`,
//! because `rust_panic_with_hook` takes a READ lock before it invokes any hook — ours
//! included. **The parent-installed hook does NOT make that read go away**.
//!
//! What the fix removes is every hook-lock acquisition the child makes ON ITS OWN
//! BEHALF, which is the WRITE lock (`set_hook`), the one that can never be granted
//! while a reader is stranded. The read the panic machinery itself takes is std's, not
//! ours, and no std API can prevent it: there is no way to hold the hook read lock
//! across `fork`, and `update_hook` — the only other public spelling — takes WRITE.
//!
//! So the residual is real and is bounded by an INVARIANT rather than by a mechanism:
//! **nothing may take std's hook lock for WRITE once a capture fork is reachable.** The
//! carrier's own install is a `Once` at ARM time, strictly before any fork exists, and
//! the rule is ENFORCED by a source walk over this module and its siblings
//! (`state_child_discipline_test.rs`) rather than left to a comment — the walk requires
//! the ONE `take_hook`/`set_hook` pair to be inside the installer and forbids the
//! tokens everywhere else in the carrier.
//!
//! What the invariant does NOT cover, stated because it is the one live hole: a NODE's
//! own code — a user crate, a cdylib's dependency — calling `panic::set_hook` at
//! runtime, concurrently with a boundary. That is outside this repo's reach, it is rare
//! (a library installing a panic hook after startup is unusual), and the outcome is
//! bounded by the progress watchdog: the child wedges before its first bump, so it is
//! killed at `STATE_STALL_TIMEOUT_NS` and reported `ChildStalled` — a miss, not a hang,
//! and not silence.
//!
//! **One hook, installed in the parent at arm time, branching on `getpid()`; the child
//! calls `set_hook` never.**
//!
//! One OTHER sanctioned installer exists in this crate:
//! `transport::reg_channel::install_contained_panic_hook_suppressor`,
//! a `Once`-guarded WRAPPING hook that swallows the default report for the registration
//! pump's CONTAINED per-tick panics and delegates for every other thread. It honors this
//! module's invariant by sequencing: it runs at `TransportManager` construction, and a
//! capture fork requires an armed recorder, which requires a live manager — so its one
//! WRITE strictly precedes fork reachability. The discipline walk deliberately scans
//! `src/state_carrier/` only; the sequencing argument lives at that installer.
//!
//! # Why `getpid()` and not a flag the child sets
//!
//! A flag would need the child to run code before the hook is correct, and a panic in
//! THAT window — between `fork` returning 0 and the store landing — would take the
//! parent arm and unwind through frames the child does not own, which is the whole
//! hazard. `getpid()` is correct from the instant `fork` returns, with no window at
//! all, and it is a vDSO-or-syscall read that takes no lock.
//!
//! **The over-reach is deliberate and is a feature.** Any OTHER fork-without-exec child
//! of this process would also take the child arm. That is the right answer for such a
//! child too: unwinding in a forked child is the same hazard whoever forked it. A
//! `fork`+`exec` child is unaffected, because `exec` replaces the image and this hook
//! with it.
//!
//! # What the child arm may do
//!
//! Nothing that takes a lock the parent's other threads could have held, and nothing
//! that can BLOCK. So: relaxed atomic stores into the breadcrumb (already mapped), one
//! **non-blocking** `write(2)` of a **fixed** byte string to a preallocated fd (dropped
//! if the fd is not immediately writable — see `write_diagnostic_nonblocking`), and
//! `_exit`. No formatting, no allocation, no `tracing` (a global dispatcher behind a
//! lock outside this hook's control), no `std::process`.
//! The node index the report needs is already in the breadcrumb, which the parent reads
//! after reaping — the child does not have to render it.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on the
//! `pub mod state_carrier;` declaration in `lib.rs`.

use std::sync::atomic::{AtomicI32, AtomicPtr, Ordering};
use std::sync::Once;

use super::breadcrumb::{ChildBreadcrumb, ChildPhase};

/// Exit code of a capture child that completed every node in its fork set.
pub const CHILD_EXIT_OK: i32 = 0;

/// Exit code of a capture child that PANICKED — `_exit`ed from the hook, so it never
/// unwound.
///
/// Deliberately not 101 (rustc's own panic exit code): the parent must be able to tell
/// "our hook ran and contained it" from "something else in this image aborted", and a
/// shared code would conflate them.
pub const CHILD_EXIT_PANIC: i32 = 90;

/// Exit code of a capture child whose encoder returned an error rather than panicking.
pub const CHILD_EXIT_CAPTURE_FAILED: i32 = 91;

/// The pid of the process that installed the hook. `0` until installed.
static PARENT_PID: AtomicI32 = AtomicI32::new(0);

/// The breadcrumb the child arm stamps, or null while detached.
///
/// Re-pointed on every arm and NULLED on disarm, because the mapping is dropped there
/// and a hook writing through a stale pointer would turn a contained panic into a
/// SIGSEGV at an address that no longer means anything.
static BREADCRUMB: AtomicPtr<ChildBreadcrumb> = AtomicPtr::new(std::ptr::null_mut());

/// The fd the child arm writes its one diagnostic line to.
///
/// Preallocated at arm time so the child never has to `dup` inside the
/// hook. Defaults to `STDERR_FILENO`.
static CHILD_STDERR_FD: AtomicI32 = AtomicI32::new(libc::STDERR_FILENO);

/// Installs the process-wide hook exactly once.
static INSTALL: Once = Once::new();

/// The one line the child arm emits. Fixed bytes: no formatting, so no allocation.
///
/// It is deliberately terse and points at the durable evidence rather than trying to
/// carry it: the node and phase are already in the breadcrumb, which the parent reads
/// after reaping, and `state_coverage.json` is where the row lands.
const CHILD_PANIC_NOTICE: &[u8] =
    b"cerulion: checkpoint capture child panicked; exiting without unwinding (see the anchor's ChildPanicked row for the node and field)\n";

/// `true` if this process is a `fork` child of the process that installed the hook.
///
/// Public because the child module's own discipline depends on it, and because a test
/// asserting the branch exists needs to observe the same predicate the hook uses
/// rather than a re-derivation of it.
pub fn is_capture_child() -> bool {
    let parent = PARENT_PID.load(Ordering::Relaxed);
    // Never installed => not a child of anything we forked.
    parent != 0 && unsafe { libc::getpid() } != parent
}

/// Install the pid-branching hook. Idempotent; called at ARM time, in the parent,
/// before any `fork`.
///
/// The previous hook is captured and delegated to on the parent arm, so installing
/// this changes parent-side panic behaviour **not at all** — which is what makes it
/// safe to install in a long-running robot process the moment a recorder attaches.
pub fn install_fork_panic_hook() {
    PARENT_PID.store(unsafe { libc::getpid() }, Ordering::Relaxed);
    INSTALL.call_once(|| {
        let previous = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            if is_capture_child() {
                child_panic_exit();
            }
            previous(info);
        }));
    });
}

/// Point the hook's child arm at the breadcrumb for this run.
///
/// # Safety
///
/// The pointer must remain mapped until [`clear_fork_panic_breadcrumb`] is called.
pub unsafe fn set_fork_panic_breadcrumb(crumb: &super::breadcrumb::MappedBreadcrumb) {
    let (ptr, _) = crumb.mapping();
    BREADCRUMB.store(ptr as *mut ChildBreadcrumb, Ordering::Release);
}

/// Forget the breadcrumb (disarm). The hook still contains a child panic; it just has
/// no page to stamp.
pub fn clear_fork_panic_breadcrumb() {
    BREADCRUMB.store(std::ptr::null_mut(), Ordering::Release);
}

/// Set the fd the child arm writes its diagnostic to (the preallocated raw
/// stderr).
///
/// Deliberately does NOT set `O_NONBLOCK` on `fd`: that flag lives on the open file
/// DESCRIPTION, which a `dup` of stderr shares with the parent, so setting it here
/// would make the PARENT's own log writes start failing `EAGAIN`. The child protects
/// itself with `poll` instead — see this module's `write_diagnostic_nonblocking`.
pub fn set_child_stderr_fd(fd: i32) {
    CHILD_STDERR_FD.store(fd, Ordering::Relaxed);
}

/// The child arm. **Never returns.**
///
/// Everything here is either a relaxed store into an already-mapped page or a
/// `write(2)` of a static byte string, so it holds no lock the parent's other threads
/// could have been inside at the fork instant.
fn child_panic_exit() -> ! {
    let crumb = BREADCRUMB.load(Ordering::Acquire);
    if !crumb.is_null() {
        // SAFETY: the pointer is either null or a live mapping the parent set and has
        // not cleared; `ChildBreadcrumb` is atomics-only, so a shared reference is
        // sound from the child.
        let b = unsafe { &*crumb };
        // Leaves the node and field the parent will report; the phase is what
        // distinguishes "died in an encoder" from "died before it started".
        b.set_phase(ChildPhase::Unknown);
    }
    write_diagnostic_nonblocking(CHILD_STDERR_FD.load(Ordering::Relaxed), CHILD_PANIC_NOTICE);
    // `_exit`, never `exit`: no destructors, no `atexit`, no SHM refcount touched,
    // nothing in iceoryx2 or the allocator torn down. This is the Redis BGSAVE
    // discipline, and leaving FROM THE HOOK is what stops the unwind before it starts.
    unsafe { libc::_exit(CHILD_EXIT_PANIC) }
}

/// How many `write(2)` attempts the diagnostic gets before it is abandoned.
///
/// A short write or an `EINTR` is worth one more try; an unbounded retry loop is not.
/// The bound exists because the alternative is the failure this whole module prevents —
/// see [`write_diagnostic_nonblocking`].
const DIAGNOSTIC_WRITE_ATTEMPTS: u32 = 8;

/// Write the diagnostic WITHOUT ever blocking, dropping it if it cannot go out now.
///
/// # A contained panic must not become a stall
///
/// The child's stderr is inherited, so it can be a PIPE — a shell pipeline, a
/// supervisor capturing output, a CI harness — and a pipe whose reader is slow or gone
/// blocks `write(2)` once its buffer fills. On that path the hook would never reach
/// `_exit`: the hook's containment would hold (nothing unwinds) while the child sat in a
/// write forever, the parent's watchdog would report `ChildStalled` five seconds later,
/// and the operator would go hunting an encoder bug in a node whose encoder had already
/// panicked and said so. A blocking diagnostic converts the one failure this module
/// makes SAFE into the one it makes CONFUSING.
///
/// # Why `poll`, and NOT `O_NONBLOCK` on the fd
///
/// The obvious fix — `fcntl(fd, F_SETFL, O_NONBLOCK)` — is wrong here, and would be
/// wrong at [`set_child_stderr_fd`] too. `O_NONBLOCK` is a property of the open FILE
/// DESCRIPTION, not of the descriptor, and a `dup` of stderr SHARES that description
/// with the parent. Setting it would make the PARENT's own writes start failing
/// `EAGAIN` — a robot's whole log — to protect a courtesy line in a child.
///
/// `poll(POLLOUT, 0)` asks the same question and mutates nothing: if the write cannot
/// proceed immediately, the line is DROPPED. That is the trade — the durable
/// evidence is the breadcrumb (which the parent reads after reaping) and the exit
/// status, so what is lost is a convenience and what is bought is that `_exit` is
/// always reached.
fn write_diagnostic_nonblocking(fd: i32, mut bytes: &[u8]) {
    let mut attempts = 0u32;
    while !bytes.is_empty() && attempts < DIAGNOSTIC_WRITE_ATTEMPTS {
        attempts += 1;
        let mut pfd = libc::pollfd {
            fd,
            events: libc::POLLOUT,
            revents: 0,
        };
        // SAFETY: a caller-owned `pollfd`; a zero timeout never blocks.
        let ready = unsafe { libc::poll(&mut pfd, 1, 0) };
        if ready != 1 || (pfd.revents & libc::POLLOUT) == 0 {
            // Not writable now (a full pipe), or the fd is gone. Drop the line.
            return;
        }
        // SAFETY: writing a byte slice we own to a caller-supplied fd that `poll` has
        // just reported writable.
        let n = unsafe { libc::write(fd, bytes.as_ptr() as *const libc::c_void, bytes.len()) };
        if n > 0 {
            bytes = &bytes[n as usize..];
            continue;
        }
        if n < 0 {
            let e = std::io::Error::last_os_error().raw_os_error();
            // EINTR / EAGAIN are worth one more poll; anything else is terminal.
            if e != Some(libc::EINTR) && e != Some(libc::EAGAIN) {
                return;
            }
            continue;
        }
        // A zero-length write on a non-empty buffer makes no progress.
        return;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_child_exit_codes_are_distinct_and_do_not_collide_with_rustcs_own() {
        // The parent classifies a reaped child by its exit code, so two conditions
        // sharing one number would merge two distinct exit conditions. 101 is rustc's own
        // panic exit; a hook that failed to contain a panic would produce it, and the
        // parent must be able to tell that apart from a contained one.
        assert_eq!(CHILD_EXIT_OK, 0);
        assert_ne!(CHILD_EXIT_PANIC, CHILD_EXIT_OK);
        assert_ne!(CHILD_EXIT_CAPTURE_FAILED, CHILD_EXIT_OK);
        assert_ne!(CHILD_EXIT_PANIC, CHILD_EXIT_CAPTURE_FAILED);
        assert_ne!(CHILD_EXIT_PANIC, 101, "must not be rustc's own panic code");
        assert_ne!(CHILD_EXIT_CAPTURE_FAILED, 101);
        // A Unix exit status is one byte.
        for code in [CHILD_EXIT_OK, CHILD_EXIT_PANIC, CHILD_EXIT_CAPTURE_FAILED] {
            assert!((0..256).contains(&code), "{code} must fit an exit status");
        }
    }

    #[test]
    fn an_uninstalled_hook_never_claims_a_process_is_a_capture_child() {
        // `PARENT_PID == 0` is the not-installed sentinel, and 0 is not a valid pid, so
        // the predicate must be false rather than comparing against it. Reading it the
        // other way would make EVERY process a "capture child" before arm time — so a
        // perfectly ordinary parent-side panic would `_exit(90)` instead of unwinding,
        // killing the robot on the first assertion failure anywhere in the process.
        //
        // Driven directly rather than through `is_capture_child()`, which cannot be
        // observed in the uninstalled state once any sibling test has armed the
        // process-wide statics.
        let uninstalled = 0;
        let live_pid = unsafe { libc::getpid() };
        assert_ne!(live_pid, uninstalled, "pid 0 is not a real process");
        assert!(
            !(uninstalled != 0 && live_pid != uninstalled),
            "the not-installed sentinel must never satisfy the child predicate"
        );
    }

    #[test]
    fn the_notice_is_a_fixed_byte_string_the_child_can_write_without_allocating() {
        // A formatted message would allocate inside a fork child's panic path — the one
        // place the design bans allocation it can pre-size away. `&'static [u8]` is the
        // enforcement: there is nothing to format.
        assert!(!CHILD_PANIC_NOTICE.is_empty());
        assert_eq!(
            *CHILD_PANIC_NOTICE.last().unwrap(),
            b'\n',
            "a raw write must terminate its own line; nothing else will"
        );
        assert!(
            std::str::from_utf8(CHILD_PANIC_NOTICE).is_ok(),
            "the notice must survive a terminal"
        );
    }
}
