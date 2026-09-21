// SPDX-License-Identifier: AGPL-3.0-only
//! `child_main` and its discipline over a REAL `fork(2)` child.
//!
//! The sibling source walk (`state_child_discipline_test.rs`) proves what the child
//! module does NOT do. This one proves what it DOES: the fd close really closes, the
//! keep list really keeps, the signal mask really comes back empty, the encoder is
//! driven in declaration order, and each exit condition maps to its own code — none of
//! which a source walk can see.
//!
//! # Everything is reported through a shared PAGE, not a pipe
//!
//! Step 5 closes every inherited fd, so a child reporting through a pipe would be
//! reporting through the very thing under test. A `MAP_SHARED | MAP_ANONYMOUS` page
//! survives the close (it is a MAPPING, not a descriptor) and is what the
//! production breadcrumb uses for the same reason. The
//! one pipe here is deliberate and is the OBJECT of a test: it proves the keep list
//! keeps and that a non-kept sibling is gone.
//!
//! `#![cfg(unix)]`; parallel-safe (anonymous pages, own pipes, targeted reaps).

#![cfg(unix)]

use cerulion_core::state_carrier::{
    child_main, resolve_fd_ceiling, ChildBreadcrumb, ChildPhase, ChildSetup, KeepFds,
    MappedBreadcrumb, NodeCaptureOutcome, NodeEncoder, CHILD_EXIT_CAPTURE_FAILED, CHILD_EXIT_OK,
};

const PAGE: usize = 4096;
const CEILING: std::time::Duration = std::time::Duration::from_secs(30);

/// A `MAP_SHARED` scratch page the child writes its findings into.
///
/// Slot layout (u64 each), fixed by hand so the child needs no formatting:
/// 0: nodes encoded · 1: encode-order fingerprint · 2: kept-fd write result
/// 3: closed-fd write result · 4: blocked signals found in the mask · 5: bumps
struct Scratch(*mut u64);

impl Scratch {
    const NODES: usize = 0;
    const ORDER: usize = 1;
    const KEPT_WRITE: usize = 2;
    const CLOSED_WRITE: usize = 3;
    const BLOCKED_SIGS: usize = 4;
    const BUMPS: usize = 5;

    fn new() -> Self {
        let p = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                PAGE,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED | libc::MAP_ANONYMOUS,
                -1,
                0,
            )
        };
        assert_ne!(p, libc::MAP_FAILED, "scratch mmap failed");
        Self(p as *mut u64)
    }

    fn set(&self, slot: usize, v: u64) {
        unsafe { self.0.add(slot).write_volatile(v) };
    }

    fn get(&self, slot: usize) -> u64 {
        unsafe { self.0.add(slot).read_volatile() }
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        unsafe { libc::munmap(self.0 as *mut std::ffi::c_void, PAGE) };
    }
}

fn reap(pid: libc::pid_t) -> libc::c_int {
    let deadline = std::time::Instant::now() + CEILING;
    loop {
        let mut status: libc::c_int = 0;
        let r = unsafe { libc::waitpid(pid, &mut status, libc::WNOHANG) };
        if r == pid {
            return status;
        }
        assert!(r >= 0, "waitpid: {}", std::io::Error::last_os_error());
        if std::time::Instant::now() >= deadline {
            unsafe { libc::kill(pid, libc::SIGKILL) };
            let mut st: libc::c_int = 0;
            unsafe { libc::waitpid(pid, &mut st, 0) };
            panic!("child did not exit within {CEILING:?}");
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

/// An encoder that records what it was asked to do, into the shared page.
struct RecordingEncoder<'a> {
    scratch: &'a Scratch,
    count: u32,
    /// Node index whose encode returns `Failed`, if any.
    fail_at: Option<u32>,
}

impl NodeEncoder for RecordingEncoder<'_> {
    fn len(&self) -> u32 {
        self.count
    }

    fn encode(&mut self, idx: u32, crumb: &ChildBreadcrumb) -> NodeCaptureOutcome {
        self.scratch
            .set(Scratch::NODES, self.scratch.get(Scratch::NODES) + 1);
        // An ORDER-SENSITIVE fingerprint: a positional mix, so encoding {0,1,2} in any
        // other order yields a different value. A plain sum or set would not.
        let mixed = self.scratch.get(Scratch::ORDER) * 31 + u64::from(idx) + 1;
        self.scratch.set(Scratch::ORDER, mixed);
        // The encoder bumps from inside its own work, which is the contract the
        // watchdog depends on for a giant whose sort emits no ring records.
        for _ in 0..10 {
            crumb.bump();
        }
        self.scratch.set(Scratch::BUMPS, crumb.progress());
        if self.fail_at == Some(idx) {
            NodeCaptureOutcome::Failed
        } else {
            NodeCaptureOutcome::Done
        }
    }
}

/// The order fingerprint the parent expects for `0..count` encoded in order.
fn expected_order(count: u32) -> u64 {
    let mut acc = 0u64;
    for idx in 0..count {
        acc = acc * 31 + u64::from(idx) + 1;
    }
    acc
}

#[test]
fn a_child_encodes_its_fork_set_in_order_and_exits_zero() {
    const NODES: u32 = 3;
    let scratch = Scratch::new();
    let crumb = MappedBreadcrumb::create().expect("map breadcrumb");
    let setup = ChildSetup {
        parent_pid: unsafe { libc::getpid() },
        keep_fds: KeepFds::none(),
        fd_ceiling: resolve_fd_ceiling().ceiling,
    };

    // SAFETY: the child runs the production `child_main`, which applies its own
    // discipline and leaves via `_exit`.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        let mut enc = RecordingEncoder {
            scratch: &scratch,
            count: NODES,
            fail_at: None,
        };
        child_main(&crumb, &setup, &mut enc);
    }
    let status = reap(pid);

    assert_eq!(
        exit_code(status),
        Some(CHILD_EXIT_OK),
        "clean fork set exits 0"
    );
    assert_eq!(scratch.get(Scratch::NODES), u64::from(NODES));
    assert_eq!(
        scratch.get(Scratch::ORDER),
        expected_order(NODES),
        "the fork set must be encoded in declaration order — the anchor's bytes are \
         order-dependent, and a set-shaped assertion would not see a reordering"
    );
    // The breadcrumb carries the LAST node and the finishing phase, which is what a
    // report names when a child dies near the end.
    assert_eq!(crumb.node_idx(), NODES - 1);
    assert_eq!(crumb.phase(), ChildPhase::Finishing);
    assert_eq!(
        crumb.progress(),
        u64::from(NODES) * 10,
        "every encoder bump must reach the PARENT's page; a child bumping into a \
         private copy is a watchdog that kills healthy children"
    );
}

#[test]
fn one_nodes_failure_costs_only_that_node_and_is_its_own_exit_code() {
    // An anchor missing a part is already rejected as PartialAnchor anyway, so a child that
    // ABORTED the fork set on the first failure would throw away every remaining node's
    // part for no gain — and the parent would not be able to tell "one encoder refused"
    // from "the child died".
    const NODES: u32 = 4;
    let scratch = Scratch::new();
    let crumb = MappedBreadcrumb::create().expect("map breadcrumb");
    let setup = ChildSetup {
        parent_pid: unsafe { libc::getpid() },
        keep_fds: KeepFds::none(),
        fd_ceiling: resolve_fd_ceiling().ceiling,
    };

    // SAFETY: as above.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        let mut enc = RecordingEncoder {
            scratch: &scratch,
            count: NODES,
            fail_at: Some(1),
        };
        child_main(&crumb, &setup, &mut enc);
    }
    let status = reap(pid);

    assert_eq!(
        exit_code(status),
        Some(CHILD_EXIT_CAPTURE_FAILED),
        "an encoder refusal is its OWN exit code — not 0, and not the panic code"
    );
    assert_eq!(
        scratch.get(Scratch::NODES),
        u64::from(NODES),
        "the remaining nodes must still be encoded after one refuses"
    );
    assert_eq!(scratch.get(Scratch::ORDER), expected_order(NODES));
}

/// Serializes the ONE arm that raises this process's `RLIMIT_NOFILE`.
///
/// The limit is process-global; raising it is benign for a concurrent sibling (its
/// child merely loops further), but the arm restores it, and two arms racing the
/// raise/restore pair could leave it lowered under a sibling that had already sized its
/// ceiling.
static RLIMIT_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[test]
fn a_descriptor_above_the_old_fixed_bound_is_closed_because_the_bound_is_the_real_limit() {
    // THE finding: the loop used to stop at a hard-coded 4096, so on any machine whose soft
    // limit was raised — `ulimit -n 65536` is ordinary on a server or a robot — every
    // descriptor above it was INHERITED and stayed open. A capture child can live for
    // minutes under the liveness watchdog (the whole point of a stall-based watchdog
    // rather than a duration cap), so a socket it holds blocks the peer's shutdown for
    // that long.
    //
    // Driven at a descriptor number the old bound could not reach, with the SAME probe
    // shape as the keep-list arm: the child writes and reports the result.
    const HIGH_FD: libc::c_int = 5000;
    let _guard = RLIMIT_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    let mut original: libc::rlimit = unsafe { std::mem::zeroed() };
    assert_eq!(
        unsafe { libc::getrlimit(libc::RLIMIT_NOFILE, &mut original) },
        0,
        "getrlimit"
    );
    // Raise the soft limit so HIGH_FD is a legal descriptor at all — macOS ships a soft
    // limit of 256, where `dup2` to 5000 fails outright.
    let want = (HIGH_FD as libc::rlim_t) + 1024;
    if original.rlim_max != libc::RLIM_INFINITY && original.rlim_max < want {
        eprintln!(
            "SKIP: hard RLIMIT_NOFILE {} is below {want}",
            original.rlim_max
        );
        return;
    }
    let raised = libc::rlimit {
        rlim_cur: want,
        rlim_max: original.rlim_max,
    };
    assert_eq!(
        unsafe { libc::setrlimit(libc::RLIMIT_NOFILE, &raised) },
        0,
        "setrlimit to {want} failed: {}",
        std::io::Error::last_os_error()
    );

    struct RestoreRlimit(libc::rlimit);
    impl Drop for RestoreRlimit {
        fn drop(&mut self) {
            unsafe { libc::setrlimit(libc::RLIMIT_NOFILE, &self.0) };
        }
    }
    let _restore = RestoreRlimit(original);

    // The ceiling must now genuinely exceed the old fixed bound, or the arm is vacuous.
    let ceiling = resolve_fd_ceiling();
    assert!(
        ceiling.ceiling > HIGH_FD,
        "the resolved ceiling {} must cover the probe descriptor {HIGH_FD}",
        ceiling.ceiling
    );

    let scratch = Scratch::new();
    let crumb = MappedBreadcrumb::create().expect("map breadcrumb");
    let mut fds = [0 as libc::c_int; 2];
    assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0, "pipe failed");
    assert_eq!(
        unsafe { libc::dup2(fds[1], HIGH_FD) },
        HIGH_FD,
        "dup2 to {HIGH_FD} failed: {}",
        std::io::Error::last_os_error()
    );

    let setup = ChildSetup {
        parent_pid: unsafe { libc::getpid() },
        keep_fds: KeepFds::none(),
        fd_ceiling: ceiling.ceiling,
    };

    // SAFETY: the child probes one descriptor and leaves via `_exit` inside `child_main`.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        struct HighProbe<'a> {
            scratch: &'a Scratch,
        }
        impl NodeEncoder for HighProbe<'_> {
            fn len(&self) -> u32 {
                1
            }
            fn encode(&mut self, _idx: u32, crumb: &ChildBreadcrumb) -> NodeCaptureOutcome {
                let byte = [1u8];
                let r = unsafe { libc::write(HIGH_FD, byte.as_ptr() as *const libc::c_void, 1) };
                self.scratch.set(Scratch::CLOSED_WRITE, (r as i64) as u64);
                crumb.bump();
                NodeCaptureOutcome::Done
            }
        }
        let mut enc = HighProbe { scratch: &scratch };
        child_main(&crumb, &setup, &mut enc);
    }
    let status = reap(pid);

    unsafe {
        libc::close(fds[0]);
        libc::close(fds[1]);
        libc::close(HIGH_FD);
    }

    assert_eq!(exit_code(status), Some(CHILD_EXIT_OK));
    assert_eq!(
        scratch.get(Scratch::CLOSED_WRITE) as i64,
        -1,
        "fd {HIGH_FD} survived into the child: the close loop stopped below the \
         process's real descriptor space, so a raised `ulimit -n` leaks every socket \
         and bag handle above the old fixed bound for the child's whole lifetime"
    );
}

#[test]
fn the_child_closes_inherited_fds_but_keeps_the_ones_it_was_told_to() {
    // Two pipes, identical in every way except that one write end is on the keep list.
    // Without the closed sibling this test would prove only that `close_range` did not
    // close everything; without the kept one it would prove only that the child cannot
    // write at all.
    let scratch = Scratch::new();
    let crumb = MappedBreadcrumb::create().expect("map breadcrumb");

    let mut kept = [0 as libc::c_int; 2];
    let mut closed = [0 as libc::c_int; 2];
    assert_eq!(unsafe { libc::pipe(kept.as_mut_ptr()) }, 0);
    assert_eq!(unsafe { libc::pipe(closed.as_mut_ptr()) }, 0);
    let (kept_w, closed_w) = (kept[1], closed[1]);

    let setup = ChildSetup {
        parent_pid: unsafe { libc::getpid() },
        keep_fds: KeepFds::none().with(kept_w),
        fd_ceiling: resolve_fd_ceiling().ceiling,
    };

    // Block SIGTERM in the PARENT, so the child inherits a non-empty mask. Step 4 must
    // clear it: a child deaf to the signals the reaper uses leaves SIGKILL as the only
    // thing that still works.
    unsafe {
        let mut block: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut block);
        libc::sigaddset(&mut block, libc::SIGTERM);
        libc::sigprocmask(libc::SIG_BLOCK, &block, std::ptr::null_mut());
    }

    // SAFETY: the child runs the production discipline, probes two descriptors and its
    // own signal mask, and leaves via `_exit` inside `child_main`.
    let pid = unsafe { libc::fork() };
    assert!(pid >= 0, "fork failed: {}", std::io::Error::last_os_error());
    if pid == 0 {
        struct Probe<'a> {
            scratch: &'a Scratch,
            kept_w: libc::c_int,
            closed_w: libc::c_int,
        }
        impl NodeEncoder for Probe<'_> {
            fn len(&self) -> u32 {
                1
            }
            fn encode(&mut self, _idx: u32, crumb: &ChildBreadcrumb) -> NodeCaptureOutcome {
                let byte = [1u8];
                let k =
                    unsafe { libc::write(self.kept_w, byte.as_ptr() as *const libc::c_void, 1) };
                let c =
                    unsafe { libc::write(self.closed_w, byte.as_ptr() as *const libc::c_void, 1) };
                self.scratch.set(Scratch::KEPT_WRITE, (k as i64) as u64);
                self.scratch.set(Scratch::CLOSED_WRITE, (c as i64) as u64);

                let mut mask: libc::sigset_t = unsafe { std::mem::zeroed() };
                unsafe {
                    libc::sigprocmask(libc::SIG_SETMASK, std::ptr::null(), &mut mask);
                }
                let mut blocked = 0u64;
                for sig in 1..32 {
                    if unsafe { libc::sigismember(&mask, sig) } == 1 {
                        blocked += 1;
                    }
                }
                self.scratch.set(Scratch::BLOCKED_SIGS, blocked);
                crumb.bump();
                NodeCaptureOutcome::Done
            }
        }
        let mut enc = Probe {
            scratch: &scratch,
            kept_w,
            closed_w,
        };
        child_main(&crumb, &setup, &mut enc);
    }
    let status = reap(pid);

    // Restore the parent's own mask before asserting, so a failure here cannot leave
    // the test binary with SIGTERM blocked.
    unsafe {
        let mut block: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut block);
        libc::sigaddset(&mut block, libc::SIGTERM);
        libc::sigprocmask(libc::SIG_UNBLOCK, &block, std::ptr::null_mut());
        libc::close(kept[0]);
        libc::close(kept[1]);
        libc::close(closed[0]);
        libc::close(closed[1]);
    }

    assert_eq!(exit_code(status), Some(CHILD_EXIT_OK));
    assert_eq!(
        scratch.get(Scratch::KEPT_WRITE),
        1,
        "the fd on the keep list must survive step 5 — the child's one diagnostic \
         channel is exactly such an fd"
    );
    assert_eq!(
        scratch.get(Scratch::CLOSED_WRITE) as i64,
        -1,
        "a descriptor NOT on the keep list must be gone: the child inherits the whole \
         graph process's fd table, including sockets and bag handles it must never touch"
    );
    assert_eq!(
        scratch.get(Scratch::BLOCKED_SIGS),
        0,
        "step 4 must clear the inherited signal MASK; the parent had SIGTERM blocked, \
         and a child that stayed deaf to it leaves SIGKILL as the reaper's only tool"
    );
}
