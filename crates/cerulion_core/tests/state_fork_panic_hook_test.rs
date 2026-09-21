// SPDX-License-Identifier: AGPL-3.0-only
//! The parent-installed, pid-branching panic
//! hook, driven over REAL `fork(2)` children — including a fork taken while std's hook
//! lock is genuinely READ-HELD, which is the precondition the parent-side install exists for.
//!
//! # Why ONE test body
//!
//! Everything here is process-global: `std::panic::set_hook` replaces the process's
//! only hook, `install_fork_panic_hook` captures whatever hook it finds as its
//! delegate, and the headline precondition (a thread parked INSIDE a hook,
//! holding the read guard) is a one-way state. libtest gives `#[serial]` a mutex but
//! never an ORDER, so the phases are folded into one `#[test]` — the template this
//! repo already uses for exactly this hazard (`cdylib_tick_lifecycle_codes_1_and_2`).
//! Its own binary for the same reason: a sibling test panicking through a hook this
//! file installed would be deciding its own behaviour.
//!
//! # The oracle for "unwinding never begins" is a DROP that must not happen
//!
//! The hazard is not that a child panic is untidy — it is that unwinding a fork
//! child runs `Drop` for the PARENT's live data plane (iceoryx2 ports, SHM guards, the
//! ring producer) in a process that owns none of it. An exit code alone cannot see
//! that: a child that unwound and then happened to exit 90 would look identical. So the
//! child registers a guard whose `Drop` writes a byte to a pipe the parent holds, and
//! the parent asserts the pipe is EMPTY. That is a direct observation of "no destructor
//! ran", not a proxy for it.
//!
//! # The park is BOUNDED, and the reason is itself evidence
//!
//! To work, phase 5 needs a thread parked INSIDE the hook so the fork inherits a held read
//! guard. Parking it forever wedges the test binary at TEARDOWN — libtest's own
//! `test_main` calls `std::panicking::take_hook()` when the run ends, which takes the
//! WRITE lock. With an unbounded park the test reports `ok`, then the
//! process never exits, with `test_main -> take_hook` blocked. That
//! is the same hook-lock wedge arriving from the other direction, in the test harness. So
//! the parker is released after phase 5 and the test WAITS until it has left the hook,
//! with an absolute backstop so a panic in any earlier phase cannot wedge CI either.
//!
//! `#![cfg(unix)]`; parallel-safe against other FILES (own binary, own pipes, anonymous
//! pages, targeted reaps).

#![cfg(unix)]

use std::cell::Cell;
use std::sync::atomic::{AtomicBool, Ordering};

use cerulion_core::state_carrier::{
    clear_fork_panic_breadcrumb, install_fork_panic_hook, is_capture_child,
    set_fork_panic_breadcrumb, ChildPhase, MappedBreadcrumb, CHILD_EXIT_PANIC,
};

/// A generous liveness ceiling. Load can delay a child; nothing can make a correct one
/// exceed this, and a DEADLOCKED one never finishes at any ceiling — which is the
/// condition phase 5 exists to catch.
const CEILING: std::time::Duration = std::time::Duration::from_secs(20);

thread_local! {
    /// Armed only by the parker thread (phase 5). Everything else delegates normally,
    /// so an assertion failure in any other phase still panics like an ordinary test.
    static PARK_IN_HOOK: Cell<bool> = const { Cell::new(false) };
}

/// Set once the parker thread is provably INSIDE the hook, holding std's read guard.
static PARKED_IN_HOOK: AtomicBool = AtomicBool::new(false);

/// Set by the test to let the parker leave the hook and release std's read guard.
static RELEASE_PARKER: AtomicBool = AtomicBool::new(false);

/// Set by the parker once it has RETURNED from the hook (guard released).
static PARKER_LEFT_HOOK: AtomicBool = AtomicBool::new(false);

/// An absolute backstop on the park, so a panic anywhere in the test can never leave
/// libtest wedged (see the module docs on `take_hook`). It is far longer than the
/// phase it serves, so it cannot end the park early on a loaded machine.
const PARK_BACKSTOP: std::time::Duration = std::time::Duration::from_secs(120);

fn reap(pid: libc::pid_t, what: &str) -> libc::c_int {
    let deadline = std::time::Instant::now() + CEILING;
    loop {
        let mut status: libc::c_int = 0;
        let r = unsafe { libc::waitpid(pid, &mut status, libc::WNOHANG) };
        if r == pid {
            return status;
        }
        assert!(
            r >= 0,
            "waitpid failed: {}",
            std::io::Error::last_os_error()
        );
        if std::time::Instant::now() >= deadline {
            unsafe { libc::kill(pid, libc::SIGKILL) };
            let mut st: libc::c_int = 0;
            unsafe { libc::waitpid(pid, &mut st, 0) };
            panic!("{what}: child {pid} did not exit within {CEILING:?} (a DEADLOCKED child never exits at any ceiling — this is the hook-lock wedge)");
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
}

fn exit_code(status: libc::c_int) -> Option<libc::c_int> {
    if libc::WIFEXITED(status) {
        Some(libc::WEXITSTATUS(status))
    } else {
        None
    }
}

/// A pipe whose read end reports whether ANYTHING was written to it.
struct Probe {
    read_fd: libc::c_int,
    write_fd: libc::c_int,
}

impl Probe {
    fn new() -> Self {
        let mut fds = [0 as libc::c_int; 2];
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0, "pipe failed");
        Self {
            read_fd: fds[0],
            write_fd: fds[1],
        }
    }

    /// PARENT: has anything been written? Non-blocking — the child is already reaped by
    /// the time this is asked, so a byte in flight has already arrived.
    ///
    /// `EINTR` is RETRIED and any other error PANICS, because this is the oracle for
    /// "no destructor ran": treating a failed `poll` as "nothing written" would make the
    /// no-unwind assertion pass on a signal, silently, which is the one direction a safety
    /// oracle must never fail in.
    fn fired(&self) -> bool {
        loop {
            let mut pfd = libc::pollfd {
                fd: self.read_fd,
                events: libc::POLLIN,
                revents: 0,
            };
            let ready = unsafe { libc::poll(&mut pfd, 1, 0) };
            if ready >= 0 {
                return ready == 1 && (pfd.revents & libc::POLLIN) != 0;
            }
            let err = std::io::Error::last_os_error();
            assert_eq!(
                err.raw_os_error(),
                Some(libc::EINTR),
                "poll on the unwind-tattle pipe failed ({err}); treating that as \
                 'nothing was written' would pass the no-unwind oracle on an error"
            );
        }
    }
}

impl Drop for Probe {
    fn drop(&mut self) {
        unsafe {
            libc::close(self.read_fd);
            libc::close(self.write_fd);
        }
    }
}

/// A guard whose `Drop` writes to a pipe. Registered on the CHILD's stack, so it runs
/// if and only if the child UNWINDS.
struct UnwindTattle(libc::c_int);

impl Drop for UnwindTattle {
    fn drop(&mut self) {
        // `*b"D"` rather than `[b'D']` — clippy's `byte_char_slices` on CI's newer
        // stable (the toolchain-drift class AGENTS.md warns about).
        let b = *b"D";
        unsafe { libc::write(self.0, b.as_ptr() as *const libc::c_void, 1) };
    }
}

/// Fill a pipe's buffer so any further write to it would BLOCK.
///
/// The write end is put in non-blocking mode for the FILL only and restored, so the
/// child inherits an ordinary blocking descriptor — which is the whole point: the
/// production code must not depend on the fd's flags.
fn fill_pipe(write_fd: libc::c_int) -> usize {
    let flags = unsafe { libc::fcntl(write_fd, libc::F_GETFL) };
    assert!(flags >= 0, "F_GETFL: {}", std::io::Error::last_os_error());
    assert_eq!(
        unsafe { libc::fcntl(write_fd, libc::F_SETFL, flags | libc::O_NONBLOCK) },
        0,
        "F_SETFL O_NONBLOCK"
    );
    let chunk = [0u8; 4096];
    let mut written = 0usize;
    loop {
        let n =
            unsafe { libc::write(write_fd, chunk.as_ptr() as *const libc::c_void, chunk.len()) };
        if n <= 0 {
            break;
        }
        written += n as usize;
        assert!(written < 64 * 1024 * 1024, "pipe never filled");
    }
    // Restore blocking mode: the child must face the ORDINARY descriptor.
    assert_eq!(
        unsafe { libc::fcntl(write_fd, libc::F_SETFL, flags) },
        0,
        "F_SETFL restore"
    );
    written
}

/// PHASE 6, run from the one test body below (the hook is process-global — see the
/// module docs on why this file has a single `#[test]`).
fn phase_6_a_full_diagnostic_pipe_does_not_turn_a_contained_panic_into_a_stall(
    crumb: &MappedBreadcrumb,
) {
    // The child's stderr is INHERITED, so it can be a pipe —
    // a shell pipeline, a supervisor capturing output, a CI harness — and once that
    // pipe's buffer fills, `write(2)` blocks. A hook that wrote the diagnostic
    // unconditionally would never reach `_exit`: the panic containment would hold while
    // the child sat in the write forever, and the parent's watchdog would report
    // ChildStalled five seconds later, sending the operator to hunt an encoder bug in a
    // node whose encoder had already panicked and said so.
    //
    // The POSITIONED SEAM is the pipe state: the fd is genuinely unwritable at the
    // instant the hook runs, and it is a BLOCKING descriptor (the fill restores the
    // flags), so nothing about the fixture makes the write cheap.
    let mut fds = [0 as libc::c_int; 2];
    assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0, "pipe failed");
    let (read_fd, write_fd) = (fds[0], fds[1]);
    let filled = fill_pipe(write_fd);
    assert!(filled > 0, "the fixture must actually fill the pipe");

    // Prove it really would block: a further blocking write cannot proceed. Asked with
    // `poll`, which is exactly what the production path asks.
    let mut probe = libc::pollfd {
        fd: write_fd,
        events: libc::POLLOUT,
        revents: 0,
    };
    let ready = unsafe { libc::poll(&mut probe, 1, 0) };
    assert!(
        ready == 0 || (probe.revents & libc::POLLOUT) == 0,
        "the pipe must be unwritable, or this arm cannot see the hazard"
    );

    cerulion_core::state_carrier::set_child_stderr_fd(write_fd);

    // SAFETY: `crumb` is the caller's live mapping and is cleared by the caller.
    unsafe { set_fork_panic_breadcrumb(crumb) };
    crumb.rearm();

    let tattle = Probe::new();
    // SAFETY: the child registers a stack guard and panics; the hook must `_exit`.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        crumb.enter_node(4);
        let _tattle = UnwindTattle(tattle.write_fd);
        panic!("encoder panicked while the diagnostic pipe was full");
    }

    let status = reap(pid, "full-pipe diagnostic");

    // Restore the process default before asserting, so a failure here cannot leave a
    // full pipe wired in as this process's child-diagnostic fd.
    cerulion_core::state_carrier::set_child_stderr_fd(libc::STDERR_FILENO);
    unsafe {
        libc::close(read_fd);
        libc::close(write_fd);
    }

    assert_eq!(
        exit_code(status),
        Some(CHILD_EXIT_PANIC),
        "the child must still leave through the hook with its own panic code; a \
         blocking diagnostic write turns a CONTAINED panic into a five-second \
         ChildStalled that blames an innocent node"
    );
    assert!(
        !tattle.fired(),
        "dropping the diagnostic must not cost the no-unwind guarantee"
    );
    assert_eq!(
        crumb.node_idx(),
        4,
        "the breadcrumb is the DURABLE evidence — it is what survives the dropped line"
    );
}

#[test]
fn amendment_1_the_parent_installed_hook_contains_child_panics_without_unwinding() {
    // ---------------------------------------------------------------- phase 0: install
    //
    // A delegate that PARKS when this thread armed it, and behaves normally otherwise.
    // Installed BEFORE `install_fork_panic_hook`, because the production installer
    // captures whatever hook it finds as its parent-arm delegate and does so exactly
    // once (`Once`) — so phase 5's precondition has to be in place from the start.
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        if PARK_IN_HOOK.with(|c| c.get()) {
            // Inside std's hook => this thread holds the READ guard. Announce, then
            // never leave: the read guard is now held by a thread that will not
            // release it, which is exactly the state a `fork` hands its child.
            PARKED_IN_HOOK.store(true, Ordering::SeqCst);
            let until = std::time::Instant::now() + PARK_BACKSTOP;
            while !RELEASE_PARKER.load(Ordering::SeqCst) && std::time::Instant::now() < until {
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
            PARKER_LEFT_HOOK.store(true, Ordering::SeqCst);
            return;
        }
        default_hook(info);
    }));

    let crumb = MappedBreadcrumb::create().expect("map breadcrumb");
    install_fork_panic_hook();
    // SAFETY: `crumb` outlives every fork below and is cleared before it is dropped.
    unsafe { set_fork_panic_breadcrumb(&crumb) };

    assert!(
        !is_capture_child(),
        "the installing process is not its own child"
    );

    // ------------------------------------------- phase 1: the parent arm is UNCHANGED
    //
    // The anti-tautology half. If the hook took the child arm unconditionally, this
    // panic would `_exit(90)` and take the whole test binary with it — a robot dying on
    // the first assertion failure anywhere in the process. It must unwind normally.
    let caught = std::panic::catch_unwind(|| panic!("parent-side panic must still unwind"));
    assert!(
        caught.is_err(),
        "a parent panic must unwind through the previous hook, not _exit"
    );

    // ------------- phase 2+3: a child panic is contained, and NOTHING is dropped
    let tattle = Probe::new();
    const CHILD_NODE: u32 = 5;
    crumb.rearm();

    // SAFETY: the child registers a stack guard and panics; the hook leaves via `_exit`
    // before any unwinding starts.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        crumb.enter_node(CHILD_NODE);
        crumb.bump();
        let _tattle = UnwindTattle(tattle.write_fd);
        panic!("capture encoder blew up");
    }
    let status = reap(pid, "phase 2");

    assert_eq!(
        exit_code(status),
        Some(CHILD_EXIT_PANIC),
        "a panicking child must _exit with the carrier's own panic code, so the parent \
         can tell a contained panic from any other abort in this image"
    );
    assert!(
        !tattle.fired(),
        "the child's Drop RAN, so it UNWOUND — in a fork child that runs Drop for the \
         PARENT's live data plane: iceoryx2 ports, SHM guards, the ring producer, \
         torn down by a process that owns none of them"
    );
    // The breadcrumb survives the child and carries the diagnosis (phase 3).
    assert_eq!(
        crumb.node_idx(),
        CHILD_NODE,
        "the node the child was inside must survive it — that is the ChildPanicked row"
    );
    assert_eq!(
        crumb.phase(),
        ChildPhase::Unknown,
        "the hook stamps a phase of its own, so a panic is distinguishable from a child \
         that merely stopped where it was"
    );

    // ------------------------- phase 4: a cleared breadcrumb must not become a SIGSEGV
    //
    // The disarm path drops the mapping, so a hook still holding the pointer would
    // write through a stale address and turn a CONTAINED panic into a segfault — a
    // strictly worse outcome than the one it was preventing.
    clear_fork_panic_breadcrumb();
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        panic!("panic with no breadcrumb armed");
    }
    let status = reap(pid, "phase 4");
    assert_eq!(
        exit_code(status),
        Some(CHILD_EXIT_PANIC),
        "with no breadcrumb the hook must still contain the panic, not crash"
    );
    // SAFETY: re-arming the same live mapping for phase 5.
    unsafe { set_fork_panic_breadcrumb(&crumb) };

    // ---------------- phase 5: THE headline — fork while std's hook lock is READ-held
    //
    // A thread panics into the hook and parks there, holding the guard
    // `rust_panic_with_hook` takes across the whole user hook. A `fork` now hands the
    // child an RwLock with a reader count above zero and NO thread left to release it.
    //
    // A child that called `panic::set_hook` as its FIRST act would take the WRITE
    // lock, which can never be granted, so the child would wedge BEFORE its panic
    // protection exists and the parent would misreport it as `ChildStalled` five
    // seconds later — pointing at an encoder that never ran. So the child
    // acquires the hook lock NEVER: the hook is already installed, in the parent.
    std::thread::spawn(|| {
        PARK_IN_HOOK.with(|c| c.set(true));
        panic!("parking inside the hook to hold std's read guard across the fork");
    });
    let deadline = std::time::Instant::now() + CEILING;
    while !PARKED_IN_HOOK.load(Ordering::SeqCst) {
        assert!(
            std::time::Instant::now() < deadline,
            "the parker thread never reached the hook; phase 5's precondition is not set up"
        );
        std::thread::sleep(std::time::Duration::from_millis(2));
    }

    let tattle = Probe::new();
    crumb.rearm();
    // SAFETY: as phase 2 — the child only panics, and the hook `_exit`s.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        crumb.enter_node(9);
        let _tattle = UnwindTattle(tattle.write_fd);
        panic!("child panic with the inherited hook lock read-held");
    }
    let status = reap(pid, "phase 5 (fork taken mid-panic)");
    assert_eq!(
        exit_code(status),
        Some(CHILD_EXIT_PANIC),
        "a child forked while std's hook lock is read-held must still exit through the \
         hook — it takes no hook lock of its own"
    );
    assert!(
        !tattle.fired(),
        "phase 5: the child unwound even though the hook ran"
    );
    assert_eq!(crumb.node_idx(), 9);

    // Let the parker leave the hook, and WAIT until it has.
    //
    // This is not tidiness: libtest calls `std::panicking::take_hook()` at the end of
    // its run, which takes the WRITE lock — so a reader parked forever wedges the
    // HARNESS at teardown, after the test has already reported `ok`. The blocked
    // frame is `test_main -> take_hook`: the same hook-lock wedge,
    // arriving from the other direction.
    RELEASE_PARKER.store(true, Ordering::SeqCst);
    let deadline = std::time::Instant::now() + CEILING;
    while !PARKER_LEFT_HOOK.load(Ordering::SeqCst) {
        assert!(
            std::time::Instant::now() < deadline,
            "the parker never left the hook; libtest's teardown take_hook() would wedge"
        );
        std::thread::sleep(std::time::Duration::from_millis(2));
    }

    // ---------------- phase 6: a FULL diagnostic pipe must not become a stall
    phase_6_a_full_diagnostic_pipe_does_not_turn_a_contained_panic_into_a_stall(&crumb);

    clear_fork_panic_breadcrumb();
}
