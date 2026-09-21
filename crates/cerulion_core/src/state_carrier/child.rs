// SPDX-License-Identifier: AGPL-3.0-only
//! The fork CHILD — everything that runs after `fork(2)` returns 0, and
//! the ONLY file the source-walk gate polices.
//!
//! # What a fork child may do, and why the list is so short
//!
//! A fork child has exactly ONE thread: whatever any other parent thread held at the
//! fork instant is held forever in the child's image, by nobody. So the child is banned
//! from `tracing` (a global dispatcher behind an uncontrolled lock), any iceoryx2
//! call, `std::process`, `exit`/destructors/`atexit`, any fd not on its keep list, and
//! any allocation that can be pre-sized away. Diagnostics are raw `write(2)`.
//!
//! That is not a style rule, and it is not enforced by review: it is enforced by a
//! comment-stripped SOURCE WALK over this file (`state_child_discipline_test.rs`), the
//! precedent this repo already uses for exactly this class of "nothing in this
//! module may call X". A hand-maintained list of allowed calls would reproduce the
//! failure mode that precedent exists to close; the walk reads the file.
//!
//! # The steps, in order, and what each one is for
//!
//! 1. **No `panic::set_hook`.** The hook is already installed, by the
//!    PARENT, branching on `getpid()`. A child that touched std's hook lock could wedge
//!    on a reader the fork inherited. This step is the ABSENCE of code, which is why the
//!    source walk asserts it rather than a runtime check.
//! 2. **`prctl(PR_SET_PDEATHSIG, SIGKILL)`** on Linux, so a SIGKILLed parent takes the
//!    child with it. macOS has no equivalent, so the child compares `getppid()` against
//!    the recorded parent pid at its checkpoints. Without either, the child is
//!    reparented to init still holding a full CoW image, and a graph restarted on a
//!    supervisor's SIGTERM contends with the previous run's orphan.
//! 3. **`oom_score_adj = 1000`** on Linux. The OOM killer's badness score is dominated
//!    by RSS, parent and child share most of theirs, and nothing otherwise biases the
//!    kernel's choice toward the disposable process. This is what makes the disposable
//!    process disposable *to the kernel*. No macOS equivalent.
//! 4. **Signal dispositions to `SIG_DFL`, and the signal MASK reset.**
//!    The child inherited the CLI's SIGINT handler and must die on
//!    Ctrl-C rather than run a shutdown path in a process that owns nothing; and an
//!    inherited BLOCKED set would leave it deaf to the very signals the reaper uses.
//! 5. **Close every inherited fd except the keep list** — a bounded `close(2)` loop, on
//!    every platform, over the process's REAL descriptor space (`RLIMIT_NOFILE`'s soft
//!    limit, resolved in the parent by [`resolve_fd_ceiling`]). A fixed bound would leak
//!    every descriptor above it wherever the soft limit was raised — `ulimit -n 65536`
//!    is ordinary — and a capture child can live for MINUTES under the liveness
//!    watchdog, so a socket it holds open blocks the peer's shutdown for that whole
//!    time. The keep list is fds only: the ring and the breadcrumb are MAPPINGS, not
//!    fds, so they need no entry and cannot
//!    be closed by accident. Linux 5.9+'s `close_range(2)` would do it in one syscall
//!    and is deliberately not used: it needs its own
//!    arm (the keep list is not contiguous, so the range form has to be driven in
//!    segments). The LOOP covers the whole space it claims to.
//! 6. **Encode**, stamping the breadcrumb per node and per push.
//! 7. **`_exit`**, never `exit`: no destructors, no `atexit`, nothing in iceoryx2 or the
//!    allocator torn down, no SHM refcount touched. The Redis BGSAVE discipline, and
//!    what makes the child structurally unable to damage the parent's data plane.
//!
//! # Where the macOS parent check runs, and why not at every bump
//!
//! The macOS `getppid()` check could run at each progress bump. A progress
//! bump is per *element* — a 30 M-entry map bumps 30 M times — so a syscall there would
//! cost 30 M syscalls in the inner loop, contradicting the rule "no syscall
//! per unit of work" and making the liveness signal itself the dominant cost. The check
//! therefore runs at NODE boundaries and around ring pushes, both of which are already
//! coarse. The cost is latency in noticing a dead parent: bounded by one node's encode
//! rather than by one element. On Linux `PR_SET_PDEATHSIG` makes the question moot,
//! and macOS is the dev platform, not the robot.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on the
//! `pub mod state_carrier;` declaration in `lib.rs`.

use super::breadcrumb::{ChildBreadcrumb, ChildPhase};
use super::hook::{CHILD_EXIT_CAPTURE_FAILED, CHILD_EXIT_OK};

/// The fallback bound for the close loop, used only when `getrlimit` FAILS.
///
/// It is not the ceiling: the real one is derived from `RLIMIT_NOFILE` (see
/// [`resolve_fd_ceiling`]), because a hard-coded 4096 leaks every descriptor above it
/// on a machine whose soft limit was raised — `ulimit -n 65536` is ordinary on a server or
/// a robot, and a capture child can live for MINUTES under the liveness watchdog, so a
/// socket it holds open blocks the peer's shutdown for that whole time.
const FD_CLOSE_FALLBACK: i32 = 4096;

/// An absolute cap on the close loop, for a soft limit of `RLIM_INFINITY`.
///
/// The loop costs one `close(2)` per descriptor, so an unbounded limit has to stop
/// somewhere. A million closes is ~1 s in a process that is about to spend far longer
/// encoding, and it is orders of magnitude above any real `RLIMIT_NOFILE`. What it
/// leaves is a RESIDUAL, not a silent one: [`FdCeilingSource::RlimitUnbounded`] names
/// it, and `close_range(2)` would remove the loop's cost basis entirely.
const FD_CLOSE_CEILING_MAX: i32 = 1 << 20;

// A fallback below a stock soft limit would leave descriptors open in the child even on
// an ordinary machine; one absurdly above it spends a syscall per fd for nothing. Pinned at
// compile time rather than in a test, because it is a property of the constants.
const _: () = assert!(FD_CLOSE_FALLBACK >= 1024);
const _: () = assert!(FD_CLOSE_FALLBACK <= 65536);
const _: () = assert!(FD_CLOSE_CEILING_MAX >= FD_CLOSE_FALLBACK);

/// Where the close loop's bound came from.
///
/// Three-way rather than a bare number, because the PARENT logs it and the two
/// non-ideal arms need different operator lines: a failed `getrlimit` means the bound is
/// a guess, an unbounded limit means the loop is capped and descriptors above the cap
/// survive. Reported rather than inferred — the child cannot log (its ban list), so
/// the resolution happens in the parent and rides [`ChildSetup`] by value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FdCeilingSource {
    /// `RLIMIT_NOFILE`'s soft limit — the process's real descriptor space.
    Rlimit,
    /// The soft limit is `RLIM_INFINITY` (or beyond `i32`); capped at this module's
    /// absolute maximum (1<<20 descriptors).
    RlimitUnbounded,
    /// `getrlimit` failed with this `errno`; the fallback is a guess and says so.
    GetrlimitFailed(i32),
}

impl FdCeilingSource {
    /// Whether the bound really covers this process's descriptor space.
    pub fn is_exact(self) -> bool {
        matches!(self, Self::Rlimit)
    }
}

/// The close loop's bound and where it came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FdCeiling {
    /// One past the highest descriptor the loop visits.
    pub ceiling: i32,
    /// Provenance, for the parent's log line.
    pub source: FdCeilingSource,
}

/// Resolve the close loop's bound from `RLIMIT_NOFILE`. **Called in the PARENT**, before
/// the fork, so a degraded answer can be logged.
///
/// The soft limit IS the descriptor space: a process cannot hold an fd at or above it,
/// so looping to it is exactly complete — which a fixed constant never is.
pub fn resolve_fd_ceiling() -> FdCeiling {
    // SAFETY: `getrlimit` fills a caller-owned `rlimit` and takes no ownership.
    let mut lim: libc::rlimit = unsafe { std::mem::zeroed() };
    if unsafe { libc::getrlimit(libc::RLIMIT_NOFILE, &mut lim) } != 0 {
        let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
        return FdCeiling {
            ceiling: FD_CLOSE_FALLBACK,
            source: FdCeilingSource::GetrlimitFailed(errno),
        };
    }
    let soft = lim.rlim_cur;
    if soft == libc::RLIM_INFINITY || soft > FD_CLOSE_CEILING_MAX as libc::rlim_t {
        return FdCeiling {
            ceiling: FD_CLOSE_CEILING_MAX,
            source: FdCeilingSource::RlimitUnbounded,
        };
    }
    FdCeiling {
        // Below the fallback the loop would be SHORTER than a stock box needs, but the
        // soft limit is authoritative: a descriptor at or above it cannot exist.
        ceiling: soft as i32,
        source: FdCeilingSource::Rlimit,
    }
}

/// The fds a child keeps open past step 5.
///
/// A fixed-size array rather than a slice of a `Vec`, because building the list must not
/// allocate: it is assembled by the PARENT before the fork and read by the child after
/// it. `-1` marks an unused entry.
#[derive(Debug, Clone, Copy)]
pub struct KeepFds([i32; Self::MAX]);

impl KeepFds {
    /// How many fds a child can be told to keep.
    ///
    /// Two is the shipping shape (a preallocated raw stderr, and one spare); the ring
    /// and the breadcrumb are MAPPINGS and need no entry.
    pub const MAX: usize = 4;

    /// An empty keep list.
    pub fn none() -> Self {
        Self([-1; Self::MAX])
    }

    /// Add an fd. Ignores negatives and silently saturates at [`KeepFds::MAX`] — the
    /// caller is the carrier, which knows its own list length at compile time.
    pub fn with(mut self, fd: i32) -> Self {
        if fd < 0 {
            return self;
        }
        for slot in self.0.iter_mut() {
            if *slot == -1 {
                *slot = fd;
                return self;
            }
        }
        self
    }

    /// Whether `fd` is on the list.
    ///
    /// A negative `fd` is never on it, even though `-1` is what fills the unused
    /// slots. Without that guard the empty marker reads as a KEPT descriptor — found by
    /// this module's own oracle. It is unreachable from the shipping close loop (which
    /// walks upward from 3), but an API whose answer depends on the caller never asking
    /// the awkward question is one edit away from being wrong.
    pub fn contains(&self, fd: i32) -> bool {
        fd >= 0 && self.0.contains(&fd)
    }
}

impl Default for KeepFds {
    fn default() -> Self {
        Self::none()
    }
}

/// What the parent decided before forking, handed to the child by value.
#[derive(Debug, Clone, Copy)]
pub struct ChildSetup {
    /// The pid the child must still see as its parent (the macOS `PR_SET_PDEATHSIG`
    /// substitute).
    pub parent_pid: i32,
    /// The fds to keep open.
    pub keep_fds: KeepFds,
    /// One past the highest descriptor step 5 closes, resolved by
    /// [`resolve_fd_ceiling`] in the PARENT — the child cannot log a degraded answer.
    pub fd_ceiling: i32,
}

impl ChildSetup {
    /// The ordinary constructor: this process is the parent, and the bound comes from
    /// `RLIMIT_NOFILE`. Returns the provenance so the caller can log a degraded one.
    pub fn resolve(keep_fds: KeepFds) -> (Self, FdCeilingSource) {
        let fd = resolve_fd_ceiling();
        (
            Self {
                // SAFETY: `getpid` takes no arguments and cannot fail.
                parent_pid: unsafe { libc::getpid() },
                keep_fds,
                fd_ceiling: fd.ceiling,
            },
            fd.source,
        )
    }
}

/// How one node's encode ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeCaptureOutcome {
    /// Encoded and pushed.
    Done,
    /// The encoder refused. The child records it and moves on: one node's failure must
    /// not cost the rest of the fork set their parts, and the reader rejects an anchor
    /// missing a part as `PartialAnchor` anyway.
    Failed,
}

/// What the child does for one node. Supplied by the carrier.
///
/// Takes the breadcrumb so the encoder can stamp fields and bump progress from inside
/// its own inner loop — the liveness signal has to come from where the work is, not
/// from a wrapper around it. A giant's canonical sort emits no ring records for minutes,
/// so a wrapper that bumped only between nodes would let the watchdog kill
/// exactly the node the fork carrier exists to serve.
pub trait NodeEncoder {
    /// How many nodes are in the fork set.
    fn len(&self) -> u32;

    /// Whether the fork set is empty.
    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Encode node `idx`.
    fn encode(&mut self, idx: u32, crumb: &ChildBreadcrumb) -> NodeCaptureOutcome;
}

/// Apply setup steps 2-5 of the module docs. Called by [`child_main`]; separated so a test can drive the
/// discipline without an encoder.
///
/// Every failure here is IGNORED rather than reported: the child has no channel for a
/// setup error that is not the breadcrumb, and a child that refused to encode because
/// `oom_score_adj` was unwritable would trade a real anchor for a hygiene warning. What
/// the parent sees instead is the anchor arriving, or the watchdog.
pub fn apply_child_discipline(setup: &ChildSetup) {
    #[cfg(target_os = "linux")]
    {
        // Step 2: a SIGKILLed parent takes the child with it.
        // SAFETY: `prctl` with PR_SET_PDEATHSIG takes one integer argument.
        unsafe { libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGKILL) };
        // Step 2b, the post-prctl race: if the parent died BETWEEN the fork and the
        // prctl, the signal was already sent and will never arrive. Re-check.
        if unsafe { libc::getppid() } != setup.parent_pid {
            unsafe { libc::_exit(CHILD_EXIT_OK) };
        }
        // Step 3: make the disposable process disposable to the kernel too.
        write_oom_score_adj();
    }
    #[cfg(not(target_os = "linux"))]
    {
        // No PR_SET_PDEATHSIG here; the parent check is all there is (see the module
        // docs on where it runs).
        if unsafe { libc::getppid() } != setup.parent_pid {
            unsafe { libc::_exit(CHILD_EXIT_OK) };
        }
    }

    reset_signals();
    close_inherited_fds(&setup.keep_fds, setup.fd_ceiling);
}

/// Step 3 (Linux): bias the OOM killer toward this process.
#[cfg(target_os = "linux")]
fn write_oom_score_adj() {
    const PATH: &[u8] = b"/proc/self/oom_score_adj\0";
    const VALUE: &[u8] = b"1000\n";
    // SAFETY: a NUL-terminated literal path and a fixed byte string. No allocation, and
    // `open`/`write`/`close` are async-signal-safe.
    unsafe {
        let fd = libc::open(PATH.as_ptr() as *const libc::c_char, libc::O_WRONLY);
        if fd < 0 {
            return;
        }
        libc::write(fd, VALUE.as_ptr() as *const libc::c_void, VALUE.len());
        libc::close(fd);
    }
}

/// Step 4: dispositions to `SIG_DFL`, and the inherited signal MASK cleared.
///
/// Both halves matter. The dispositions are why the child dies on Ctrl-C rather than
/// running the CLI's shutdown path in a process that owns nothing. The MASK matters
/// because a child that inherited a blocked set would be deaf to exactly
/// the signals the reaper uses to end it, so the watchdog's SIGKILL would be the only
/// thing that still worked — and only because SIGKILL cannot be blocked.
fn reset_signals() {
    // SAFETY: `signal`/`sigprocmask` with well-formed arguments; both are
    // async-signal-safe.
    unsafe {
        let mut sig = 1;
        while sig < 32 {
            // SIGKILL and SIGSTOP cannot be re-dispositioned; asking is harmless and
            // skipping them keeps the loop free of special cases at the call site.
            if sig != libc::SIGKILL && sig != libc::SIGSTOP {
                libc::signal(sig, libc::SIG_DFL);
            }
            sig += 1;
        }
        let mut empty: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut empty);
        libc::sigprocmask(libc::SIG_SETMASK, &empty, std::ptr::null_mut());
    }
}

/// Step 5: close every inherited fd except the keep list.
///
/// The ring and the breadcrumb are MAPPINGS, not fds, so they are not on the list and
/// cannot be closed here, which is the reason
/// this loop needs no knowledge of them at all.
fn close_inherited_fds(keep: &KeepFds, ceiling: i32) {
    let mut fd = 3; // 0/1/2 are handled by the keep list or left alone deliberately.
    while fd < ceiling {
        if !keep.contains(fd) {
            // SAFETY: closing a descriptor that may not exist returns EBADF and is
            // otherwise a no-op.
            unsafe { libc::close(fd) };
        }
        fd += 1;
    }
}

/// Run the child to completion. **Never returns.**
///
/// The whole body is inside a `catch_unwind` with `_exit` on BOTH arms, which is
/// belt-and-braces over the parent-installed hook: the hook leaves from inside the panic, so
/// the `Err` arm here should be unreachable — but "should be unreachable" is exactly
/// the claim that must not be the only thing standing between a user encoder and
/// the parent's data plane.
pub fn child_main(crumb: &ChildBreadcrumb, setup: &ChildSetup, encoder: &mut dyn NodeEncoder) -> ! {
    apply_child_discipline(setup);

    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let mut failures = 0u32;
        let count = encoder.len();
        let mut idx = 0u32;
        while idx < count {
            // A dead parent means nobody will ever drain this anchor. Checked per NODE
            // rather than per progress bump — see the module docs.
            if unsafe { libc::getppid() } != setup.parent_pid {
                unsafe { libc::_exit(CHILD_EXIT_OK) };
            }
            crumb.enter_node(idx);
            if encoder.encode(idx, crumb) == NodeCaptureOutcome::Failed {
                failures += 1;
            }
            idx += 1;
        }
        crumb.set_phase(ChildPhase::Finishing);
        failures
    }));

    match result {
        Ok(0) => unsafe { libc::_exit(CHILD_EXIT_OK) },
        Ok(_) => unsafe { libc::_exit(CHILD_EXIT_CAPTURE_FAILED) },
        // Unreachable while the parent-installed hook is in place; kept because the unwinding
        // hazard must not rest on one mechanism.
        Err(_) => unsafe { libc::_exit(super::hook::CHILD_EXIT_PANIC) },
    }
}

/// Stamp [`ChildPhase::RingFull`] for the duration of a push that can block, then put
/// the phase back.
///
/// This is the recorder-vs-encoder split's entire cost on the child side: one relaxed store
/// before, one after. A stall observed while the phase reads `RingFull` is the RECORDER
/// having stopped draining, not the encoder having deadlocked, and the two need
/// different operator lines.
///
/// The phase is RESTORED rather than left set, because a stall that begins just after a
/// push returned must not inherit the previous push's phase and blame a healthy
/// recorder. (A transient `RingFull` reading is harmless on its own: the watchdog
/// classifies only when progress has ALSO been frozen for the whole timeout.)
pub fn push_with_backpressure_phase<T>(crumb: &ChildBreadcrumb, push: impl FnOnce() -> T) -> T {
    crumb.set_phase(ChildPhase::RingFull);
    let out = push();
    crumb.set_phase(ChildPhase::Encoding);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_keep_list_is_fixed_size_and_refuses_negatives() {
        // It is read by a child after `fork`, so it must not be backed by an allocation
        // the child would have to touch. Saturation is the only sane overflow behaviour
        // for a fixed array whose caller knows its own length at compile time.
        let k = KeepFds::none();
        assert!(!k.contains(2), "an empty list keeps nothing");
        assert!(
            !k.contains(-1),
            "the empty marker must not read as a kept fd"
        );

        let k = k.with(2).with(7);
        assert!(k.contains(2) && k.contains(7));
        assert!(!k.contains(3));

        // Negatives are ignored rather than stored, or they would collide with the
        // `-1` empty marker and make every unused slot look kept.
        let k = k.with(-5);
        assert!(!k.contains(-5));
        assert!(!k.contains(-1));

        // Saturating: past MAX, further fds are dropped rather than overwriting one
        // the caller asked to keep.
        let mut full = KeepFds::none();
        for fd in 10..(10 + KeepFds::MAX as i32) {
            full = full.with(fd);
        }
        for fd in 10..(10 + KeepFds::MAX as i32) {
            assert!(full.contains(fd), "fd {fd} must survive a full list");
        }
        let full = full.with(99);
        assert!(!full.contains(99), "an over-full list drops the newcomer");
        assert!(full.contains(10), "and never evicts an earlier keeper");
    }

    #[test]
    fn the_fd_ceiling_covers_this_processs_real_descriptor_space() {
        // The bound must be the process's OWN limit, not a constant: a machine with a
        // raised soft limit is exactly where a fixed 4096 leaks descriptors, and a
        // capture child holding one can block a peer's shutdown for minutes.
        let resolved = resolve_fd_ceiling();
        let mut lim: libc::rlimit = unsafe { std::mem::zeroed() };
        assert_eq!(
            unsafe { libc::getrlimit(libc::RLIMIT_NOFILE, &mut lim) },
            0,
            "getrlimit must work on the test host, or this arm proves nothing"
        );

        match resolved.source {
            FdCeilingSource::Rlimit => {
                assert_eq!(
                    i64::from(resolved.ceiling),
                    lim.rlim_cur as i64,
                    "an exact bound must BE the soft limit — a descriptor at or above \
                     it cannot exist, and anything below it can"
                );
                assert!(resolved.source.is_exact());
            }
            FdCeilingSource::RlimitUnbounded => {
                assert_eq!(resolved.ceiling, FD_CLOSE_CEILING_MAX);
                assert!(
                    lim.rlim_cur == libc::RLIM_INFINITY
                        || lim.rlim_cur > FD_CLOSE_CEILING_MAX as libc::rlim_t,
                    "the capped arm may only be taken for a genuinely unbounded limit"
                );
                assert!(!resolved.source.is_exact());
            }
            FdCeilingSource::GetrlimitFailed(_) => {
                panic!("getrlimit succeeded above, so this arm is unreachable here")
            }
        }
        assert!(
            resolved.ceiling > 3,
            "the loop must visit at least one descriptor"
        );
    }

    #[test]
    fn a_degraded_fd_ceiling_is_never_reported_as_exact() {
        // The two non-ideal arms are what the parent logs about. Collapsing either into
        // "exact" is how a bound that covers less than it claims goes unnoticed.
        assert!(FdCeilingSource::Rlimit.is_exact());
        assert!(!FdCeilingSource::RlimitUnbounded.is_exact());
        assert!(!FdCeilingSource::GetrlimitFailed(libc::EPERM).is_exact());
        // And the fallback is only ever a GUESS — never silently presented as the space.
        let guessed = FdCeiling {
            ceiling: FD_CLOSE_FALLBACK,
            source: FdCeilingSource::GetrlimitFailed(libc::EPERM),
        };
        assert!(!guessed.source.is_exact());
    }

    #[test]
    fn the_resolving_constructor_names_this_process_as_the_parent() {
        // `ChildSetup::resolve` is the ordinary path; a wrong parent pid makes the
        // macOS PDEATHSIG substitute exit the child immediately.
        let (setup, source) = ChildSetup::resolve(KeepFds::none().with(2));
        assert_eq!(setup.parent_pid, unsafe { libc::getpid() });
        assert!(setup.keep_fds.contains(2));
        assert_eq!(setup.fd_ceiling, resolve_fd_ceiling().ceiling);
        assert_eq!(source, resolve_fd_ceiling().source);
    }

    #[test]
    fn the_backpressure_phase_is_restored_so_a_later_stall_is_not_misattributed() {
        // A stall in RingFull blames the RECORDER. If the phase were
        // left set after a push returned, an encoder that wedged one instruction later
        // would be reported as a dead recorder — the exact misdiagnosis the phase
        // exists to remove, with its sign flipped.
        let crumb = super::super::breadcrumb::MappedBreadcrumb::create().expect("map");
        crumb.enter_node(1);
        assert_eq!(crumb.phase(), ChildPhase::Encoding);

        let observed_during_push = push_with_backpressure_phase(&crumb, || crumb.phase());
        assert_eq!(
            observed_during_push,
            ChildPhase::RingFull,
            "the phase must be RingFull WHILE the push can block"
        );
        assert_eq!(
            crumb.phase(),
            ChildPhase::Encoding,
            "and must be restored the moment it cannot"
        );
    }
}
