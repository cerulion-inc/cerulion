// SPDX-License-Identifier: AGPL-3.0-only
//! The **public wake source** — a listener-only "a frame may
//! have arrived on this topic" signal, and a multiplexer that blocks on a set of
//! them with a timeout.
//!
//! # Why this exists
//!
//! An observer that reads a topic through a listener-less
//! [`DataOnlySubscriber`](crate::transport::subscriber::DataOnlySubscriber) has
//! no way to learn that a frame arrived — the tap is structurally invisible to
//! the producer's notifier, which is the whole point of that class. So
//! every such consumer has had to POLL, and `cerulion-vizd`'s frame drain polls
//! on a fixed 16 ms cadence that was measured as **11.8 ms p50 / 19.4 ms p90**
//! of the desk's entire frame latency against 78 µs of real work.
//!
//! This module gives that consumer a wake WITHOUT giving up the data-only tap:
//! the [`WakeSource`] is a **sibling** of the tap, not a replacement. The tap
//! keeps reading through `DataOnlySubscriber` (same bytes, same order, same
//! drain path); the listener only says *when* to look.
//!
//! # One multiplexer, not two
//!
//! [`WakeSet`] is a thin public face over the SAME `graph::waitset`
//! `WaitSetReactor` the live loop already blocks on:
//! it adds no reactor, no second attach discipline, and no
//! second determinism contract. In particular the fired set comes back in
//! **declaration order** (ascending index into the caller's slice) because the
//! reactor sorts it that way, independent of iceoryx2's internal callback order.
//!
//! # What a wake means, and what it does not
//!
//! A wake is a SIGNAL, never a count. iceoryx2 event notifications coalesce, so
//! a single wake can stand for any number of frames — including, legitimately,
//! **zero** (a connection-lifecycle event, a coalesced notify whose frame the
//! caller already drained). The buffer is still the SHM queue; the loss boundary
//! is unchanged. A caller must therefore:
//!
//! * drain BEFORE waiting (a frame committed between the last drain and the
//!   arming of the wait would otherwise sit until the timeout — the same
//!   send→notify race `CerulionSubscriber::wait_for_message` names), and
//! * treat the timeout as the fallback that makes the loop a strict SUPERSET of
//!   a polling one: a missed, dropped or never-sent wake degrades to exactly the
//!   old cadence, never to a stall.
//!
//! # Who pays
//!
//! Attaching a listener is not free: the producer's notifier now has one more
//! connection to `sendto`. Two decisions bound that cost and the caller — not this
//! module — is responsible for respecting them:
//!
//! * **An undrained listener** used to be the expensive case: under iceoryx2
//!   0.9.1 it filled its `AF_UNIX SOCK_DGRAM` socket and every later notify to
//!   it took a failure path that flooded the log. 0.10 removed that (a full
//!   doorbell is swallowed, and a notify into an already-notified listener
//!   skips the send), so an undrained listener now costs the producer LESS than
//!   a drained one. A wake-driven consumer drains its listener on every wake by
//!   construction, so what it costs the producer is one doorbell send per
//!   publish — present-vs-absent, which is what the next point is about.
//! * **Notify elision** means a GRAPH publisher with zero listeners
//!   skips notify work entirely. Attaching here un-arms that for the topic.
//!   Attach a wake only where the publisher is one you are willing to bill.
//!
//! Both are policy for the caller; this module states them and enforces neither.

use std::time::Duration;

use iceoryx2::port::listener::Listener;

use crate::error::{TransportError, TransportResult};
use crate::graph::waitset::{WaitSetReactor, WaitSource};
use crate::graph::WAITSET_MAX_ATTACHMENTS;
use crate::transport::CerService;

/// One topic's wake channel: an iceoryx2 event `Listener` on `{topic}/event`,
/// and nothing else — no subscriber port, no data queue, no notifier.
///
/// Minted by
/// [`TransportManager::create_wake_listener`](crate::transport::TransportManager::create_wake_listener),
/// which is the only constructor: a wake source is meaningless without the
/// manager's iceoryx2 node, and routing every mint through the manager keeps the
/// open-only discipline (a missing topic is a loud error, never a phantom
/// service) in one place.
///
/// `Send + Sync` (iceoryx2's `ipc_threadsafe` service), so a source can be
/// minted on a control thread and waited on by a drain thread — which is exactly
/// the shape a daemon needs.
pub struct WakeSource {
    listener: Listener<CerService>,
    topic: String,
}

impl WakeSource {
    /// Wrap an already-created listener. Crate-internal: see the type docs for
    /// why [`TransportManager::create_wake_listener`](crate::transport::TransportManager::create_wake_listener)
    /// is the public door.
    pub(crate) fn new(listener: Listener<CerService>, topic: String) -> Self {
        Self { listener, topic }
    }

    /// The topic this source wakes for. Used as the label in the multiplexer's
    /// attach-failure diagnostics, so an operator sees a TOPIC rather than an
    /// opaque slice index.
    pub fn topic(&self) -> &str {
        &self.topic
    }
}

impl std::fmt::Debug for WakeSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The listener has no useful Debug and printing it would leak iceoryx2
        // internals into a daemon's logs; the topic is the whole identity.
        f.debug_struct("WakeSource")
            .field("topic", &self.topic)
            .finish()
    }
}

/// The reason a [`WakeSet::wait`] refuses a source slice
/// outright, or `None` when the slice is within capacity.
///
/// PURE and split out from [`WakeSet::wait`] so the boundary is testable at the
/// exact value without provisioning a thousand real iceoryx2 listeners — and so
/// the refusal is one expression rather than a condition duplicated between the
/// check and its message.
fn refuse_over_capacity(len: usize) -> Option<TransportError> {
    if len <= WAITSET_MAX_ATTACHMENTS {
        return None;
    }
    Some(TransportError::GraphError {
        reason: format!(
            "a wake set was asked to wait on {len} sources, but the iceoryx2 WaitSet \
             supports at most {WAITSET_MAX_ATTACHMENTS} attachments (it is FD-set \
             bounded, ~FD_SETSIZE); past the cap sources silently fail to attach and \
             those topics lose their wake, degrading to the caller's timeout cadence \
             — wait on fewer sources, or split them across wake sets"
        ),
    })
}

/// A multiplexer that blocks until any of a set of [`WakeSource`]s fires, or a
/// timeout elapses, and reports WHICH fired.
///
/// The set is passed **per call**, not owned: a daemon's tap set changes under a
/// lock on one thread while the drain thread waits on it, so binding the sources
/// into the multiplexer would force the wait to hold that lock. Owning only the
/// reactor's reusable buffers keeps the wait lock-free by construction.
///
/// See the [module docs](self) for what a wake means (and does not).
pub struct WakeSet {
    reactor: WaitSetReactor,
}

impl WakeSet {
    /// Build a wake set. `capacity_hint` pre-sizes the reactor's fired-set
    /// buffers so a steady-state [`wait`](Self::wait) records into already-
    /// reserved space; it is a hint, not a cap (the cap is
    /// [`WAITSET_MAX_ATTACHMENTS`], enforced per call in [`wait`](Self::wait)).
    pub fn new(capacity_hint: usize) -> TransportResult<Self> {
        let reactor =
            WaitSetReactor::new(capacity_hint).map_err(|e| TransportError::GraphError {
                reason: format!("could not create a wake set (iceoryx2 WaitSet): {e}"),
            })?;
        Ok(Self { reactor })
    }

    /// Block until one of `sources` fires or `timeout` elapses, then return the
    /// INDICES (into `sources`) of every source that fired, ascending.
    ///
    /// * An EMPTY slice returns immediately with an empty fired set — it does
    ///   not sleep. A caller with nothing to wait on must pace itself (see
    ///   [`last_wait_blocked`](Self::last_wait_blocked)), exactly as the live
    ///   loop does.
    /// * A fired index means "there MAY be data" — see the [module docs](self).
    /// * The fired set is **empty on timeout**, which is not an error.
    ///
    /// The only `Err` is a slice larger than [`WAITSET_MAX_ATTACHMENTS`], which
    /// is refused BEFORE anything is attached (a partial attach would silently
    /// drop the tail's wakes).
    pub fn wait(
        &mut self,
        sources: &[&WakeSource],
        timeout: Duration,
    ) -> TransportResult<&[usize]> {
        if let Some(err) = refuse_over_capacity(sources.len()) {
            return Err(err);
        }
        // The reactor labels each source for its attach-failure diagnostics. A
        // topic is the label an operator can act on, so pay one small Vec per
        // wait to carry it. This is a daemon control loop (tens of waits per
        // second), not a frame path — the frames never come through here.
        let pairs: Vec<(WaitSource<'_>, std::sync::Arc<str>)> = sources
            .iter()
            .map(|s| {
                (
                    WaitSource::Listener(&s.listener),
                    std::sync::Arc::from(s.topic.as_str()),
                )
            })
            .collect();
        self.reactor.run_once(&pairs, timeout);
        Ok(self.reactor.fired_indices())
    }

    /// Did the most recent [`wait`](Self::wait) actually BLOCK — i.e. attach at
    /// least one source and spend up to the timeout on it?
    ///
    /// `false` when it short-circuited (an empty slice, or every attach failed),
    /// in which case the call consumed no time at all. A loop that treats the
    /// timeout as its pacing MUST check this, or an empty source set turns it
    /// into an unbounded spin, the class where an event loop with an
    /// unspent budget burns a core.
    pub fn last_wait_blocked(&self) -> bool {
        self.reactor.last_wait_blocked()
    }
}

impl std::fmt::Debug for WakeSet {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("WakeSet")
            .field("last_wait_blocked", &self.reactor.last_wait_blocked())
            .finish()
    }
}

// =====================================================================
// Poll(2)-backed fd wakes — `Doorbell` + `FdWakeSet`
// =====================================================================
//
// The listener-based [`WakeSet`] above rides the iceoryx2 WaitSet, which
// needs `&Listener` borrows for the whole block and (on macOS) watches
// fds with `select`, aborting on any fd number >= FD_SETSIZE. `rmw_wait`
// cannot use either property: it must block on the event
// listeners of entities whose iceoryx2 state lives behind per-entity
// mutexes shared with concurrent take/publish threads — so the wait may
// hold NO entity lock across the block — and it runs inside foreign ROS
// processes whose fd numbers it does not control. The two types below are
// the poll(2) face of the same wake vocabulary:
//
// * a caller LOCKS an entity, snapshots its listener's raw fd (a `Copy`
//   integer — `CerulionSubscriber::event_listener_fd`), UNLOCKS, and
//   blocks on the snapshot;
// * `poll(2)` has no fd-NUMBER ceiling on any unix (the existing
//   precedent: the monitor-wait park polls high fds for exactly this
//   reason), so the select-path FD_SETSIZE abort is structurally
//   impossible here rather than guarded;
// * nothing is registered with the listener, so there is no double-attach
//   question — two waiters polling one fd both see level-triggered
//   readability.
//
// Everything the module docs say about what a wake MEANS applies
// unchanged: a wake is a SIGNAL, never a count; drain before waiting;
// the timeout is the fallback that keeps a loop a strict superset of a
// polling one.

#[cfg(unix)]
use std::os::unix::io::{AsRawFd, FromRawFd, OwnedFd, RawFd};
#[cfg(unix)]
use std::time::Instant;

/// A self-wake channel: one fd a waiter polls, one a trigger writes.
///
/// Linux: a single `eventfd(2)` (both roles). Other unix (macOS): a
/// nonblocking `pipe(2)`. Built for the rmw guard conditions — a
/// trigger [`ring`](Self::ring)s from any thread; a blocked
/// [`FdWakeSet::wait`] watching [`poll_fd`](Self::poll_fd) wakes; the
/// waiter [`drain`](Self::drain)s before re-arming (an undrained doorbell
/// is level-readable and would re-fire every block, the spin class).
///
/// The doorbell is a WAKE, never the truth: callers pair it with their own
/// state (an atomic flag, a queue probe) that is written BEFORE the ring,
/// so a waiter that drains the ring and then reads the state still sees
/// it — drain-then-probe is race-free by that ordering.
#[cfg(unix)]
pub struct Doorbell {
    /// The end a waiter polls (POLLIN) and drains.
    read_fd: OwnedFd,
    /// The end a trigger writes. `None` on Linux, where the eventfd plays
    /// both roles.
    write_fd: Option<OwnedFd>,
}

#[cfg(unix)]
impl Doorbell {
    /// Create a doorbell. Fails only on fd exhaustion / syscall failure —
    /// callers should degrade loudly (a trigger without a doorbell still
    /// sets its own state; a blocked waiter notices at its timeout cap
    /// instead of instantly).
    pub fn new() -> TransportResult<Self> {
        #[cfg(target_os = "linux")]
        {
            // SAFETY: plain syscall; on success the fd is fresh and
            // uniquely ours to own.
            let fd = unsafe { libc::eventfd(0, libc::EFD_NONBLOCK | libc::EFD_CLOEXEC) };
            if fd < 0 {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "doorbell eventfd creation failed: {}",
                        std::io::Error::last_os_error()
                    ),
                });
            }
            // SAFETY: fresh fd from eventfd(2), uniquely owned.
            Ok(Self {
                read_fd: unsafe { OwnedFd::from_raw_fd(fd) },
                write_fd: None,
            })
        }
        #[cfg(not(target_os = "linux"))]
        {
            let mut fds = [0i32; 2];
            // SAFETY: `fds` is a live 2-slot array, as pipe(2) requires.
            if unsafe { libc::pipe(fds.as_mut_ptr()) } != 0 {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "doorbell pipe creation failed: {}",
                        std::io::Error::last_os_error()
                    ),
                });
            }
            // SAFETY: fresh fds from pipe(2), uniquely owned (and closed
            // by OwnedFd's Drop on every path below, error included).
            let read_fd = unsafe { OwnedFd::from_raw_fd(fds[0]) };
            let write_fd = unsafe { OwnedFd::from_raw_fd(fds[1]) };
            set_nonblocking_cloexec(read_fd.as_raw_fd()).map_err(|e| {
                TransportError::GraphError {
                    reason: format!("doorbell pipe read-end fcntl failed: {e}"),
                }
            })?;
            set_nonblocking_cloexec(write_fd.as_raw_fd()).map_err(|e| {
                TransportError::GraphError {
                    reason: format!("doorbell pipe write-end fcntl failed: {e}"),
                }
            })?;
            Ok(Self {
                read_fd,
                write_fd: Some(write_fd),
            })
        }
    }

    /// Ring the doorbell (any thread). Infallible and silent BY CONTRACT:
    ///
    /// * `write(2)` is async-signal-safe and ring is reachable from signal
    ///   paths (rmw guard triggers), so it must not log or allocate;
    /// * `WouldBlock` means the channel already holds pending wakes — the
    ///   doorbell is level-observed, one pending byte is enough;
    /// * any other failure degrades to the waiter's timeout cadence — the
    ///   caller's own state (written before the ring) is the truth, the
    ///   ring is only WHEN to look.
    pub fn ring(&self) {
        let fd = self.write_fd.as_ref().unwrap_or(&self.read_fd).as_raw_fd();
        // eventfd requires exactly one u64; a pipe accepts the same 8
        // bytes — one uniform write.
        let buf = 1u64.to_ne_bytes();
        loop {
            // SAFETY: `buf` is live for the call; the fd is owned by self.
            let n = unsafe { libc::write(fd, buf.as_ptr().cast(), buf.len()) };
            if n >= 0 {
                return;
            }
            if std::io::Error::last_os_error().kind() == std::io::ErrorKind::Interrupted {
                continue;
            }
            return;
        }
    }

    /// Drain every pending ring (nonblocking; read until empty). A waiter
    /// calls this each iteration BEFORE probing its own state — see the
    /// type docs for why that order is race-free.
    pub fn drain(&self) {
        let fd = self.read_fd.as_raw_fd();
        let mut buf = [0u8; 64];
        loop {
            // SAFETY: `buf` is live for the call; the fd is owned by self.
            let n = unsafe { libc::read(fd, buf.as_mut_ptr().cast(), buf.len()) };
            if n > 0 {
                continue; // more may be queued (pipe arm)
            }
            if n < 0 && std::io::Error::last_os_error().kind() == std::io::ErrorKind::Interrupted {
                continue;
            }
            return; // 0, WouldBlock (= drained), or a hard error
        }
    }

    /// The fd a waiter watches (POLLIN) — hand it to [`FdWakeSet::watch`].
    /// Valid while this doorbell lives; never read, write or close it.
    pub fn poll_fd(&self) -> RawFd {
        self.read_fd.as_raw_fd()
    }
}

#[cfg(unix)]
impl std::fmt::Debug for Doorbell {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Doorbell")
            .field("poll_fd", &self.read_fd.as_raw_fd())
            .finish()
    }
}

#[cfg(all(unix, not(target_os = "linux")))]
fn set_nonblocking_cloexec(fd: RawFd) -> std::io::Result<()> {
    // SAFETY: fcntl flag reads/writes on an fd the caller owns.
    unsafe {
        let flags = libc::fcntl(fd, libc::F_GETFL);
        if flags < 0 || libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) < 0 {
            return Err(std::io::Error::last_os_error());
        }
        let fdflags = libc::fcntl(fd, libc::F_GETFD);
        if fdflags < 0 || libc::fcntl(fd, libc::F_SETFD, fdflags | libc::FD_CLOEXEC) < 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(())
}

/// Outcome of one [`FdWakeSet::wait`].
#[cfg(unix)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FdWake {
    /// At least one watched fd became readable. A SIGNAL, never a count —
    /// the caller drains + probes its own state and decides.
    Fired,
    /// The full timeout elapsed with no readiness. Not an error; the
    /// polling-superset fallback.
    TimedOut,
}

/// A `poll(2)`-backed multi-fd blocking wait with a reusable buffer.
///
/// The poll(2) sibling of [`WakeSet`]: raw fd snapshots instead of
/// `&Listener` borrows (so no lock is held across the block), no
/// FD_SETSIZE ceiling (see the section comment above), and degradation
/// instead of aborts — a watched fd that dies mid-wait (`POLLNVAL`, or an
/// error state with nothing readable) is dropped from the set for the
/// rest of its life in this set, where iceoryx2's select path would
/// `fatal_panic` the whole process on EBADF.
#[cfg(unix)]
#[derive(Default)]
pub struct FdWakeSet {
    fds: Vec<libc::pollfd>,
    /// Indices (in `watch` order) of the fds the LAST `wait`/`check`
    /// found readable — recorded before `revents` is cleared, so a caller
    /// can drain/probe ONLY what fired instead of every watched entity.
    fired: Vec<usize>,
}

#[cfg(unix)]
impl FdWakeSet {
    /// An empty set. [`wait`](Self::wait) on it is a plain bounded sleep.
    pub fn new() -> Self {
        Self::default()
    }

    /// Forget every watched fd (buffer capacity retained).
    pub fn clear(&mut self) {
        self.fds.clear();
        self.fired.clear();
    }

    /// The fds the LAST [`Self::wait`] / [`Self::check`] found readable, as
    /// indices in `watch` order (empty after a timeout, after `clear`, or
    /// before any wait). A caller that keeps a parallel table of what it
    /// watched can drain and probe exactly the entities that woke it —
    /// the difference between O(fired) and O(entities) syscalls per wake.
    pub fn fired(&self) -> &[usize] {
        &self.fired
    }

    /// Watch `fd` for readability (POLLIN). The fd is a NON-OWNING
    /// snapshot: the caller guarantees it stays open for as long as it is
    /// watched — a violation degrades (the entry is dropped on
    /// `POLLNVAL`), never aborts.
    pub fn watch(&mut self, fd: RawFd) {
        // hot-path-alloc-ok: wait-set (re)build on the rmw wait path, not
        // the frame path — the buffer is reused across iterations/calls.
        self.fds.push(libc::pollfd {
            fd,
            events: libc::POLLIN,
            revents: 0,
        });
    }

    /// Number of watched fds (dead, neutralized entries included).
    pub fn len(&self) -> usize {
        self.fds.len()
    }

    /// True when nothing is watched.
    pub fn is_empty(&self) -> bool {
        self.fds.is_empty()
    }

    /// Nonblocking readiness sweep: one `poll(…, 0)` over the watched
    /// fds. [`FdWake::Fired`] iff any fd is readable RIGHT NOW; dead fds
    /// are neutralized exactly as in [`Self::wait`]. This is the park
    /// tier's per-slice fd check — a parked rmw wait re-checks listener
    /// and doorbell fds each recheck slice with this instead of blocking
    /// on them.
    pub fn check(&mut self) -> FdWake {
        // `fired` describes THIS call only — cleared before any early
        // return, so a caller draining "what fired" after an empty or
        // timed-out sweep never re-drains the previous wake's entities.
        self.fired.clear();
        if self.fds.is_empty() {
            return FdWake::TimedOut;
        }
        // SAFETY: the pointer/len pair is `self.fds`, live across the
        // call; poll(2) only reads fd/events and writes revents.
        let rc = unsafe { libc::poll(self.fds.as_mut_ptr(), self.fds.len() as libc::nfds_t, 0) };
        if rc <= 0 {
            return FdWake::TimedOut;
        }
        let mut fired = false;
        for (i, p) in self.fds.iter_mut().enumerate() {
            if p.revents & libc::POLLIN != 0 {
                fired = true;
                self.fired.push(i);
            } else if p.revents != 0 {
                p.fd = -1;
            }
            p.revents = 0;
        }
        if fired {
            FdWake::Fired
        } else {
            FdWake::TimedOut
        }
    }

    /// Block until a watched fd is readable or `timeout` elapses.
    ///
    /// * An EMPTY set sleeps the timeout and reports [`FdWake::TimedOut`];
    ///   it never returns early and never spins (the unbounded-spin class).
    /// * **Linux: `ppoll(2)`, nanosecond timeout** — a sub-millisecond
    ///   budget is a REAL fd-armed block (the rmw wait's adaptive ladder
    ///   starts at 100 µs; a near timer also keeps the core in a shallow
    ///   C-state, which is what makes an fd wake arrive in µs rather than
    ///   after a deep-idle exit). Subject to the thread's timer slack like
    ///   any timed block.
    /// * **Other unix: `poll(2)`, whole milliseconds** — a sub-millisecond
    ///   budget (or tail) is polled at ONE millisecond, never at 0 (a
    ///   nonblocking probe would turn the loop into a spin) and never
    ///   slept (a sleep leaves the fd unarmed): the fd wake stays
    ///   immediate; the cost is up to a millisecond of overshoot on a
    ///   sub-ms tail, which callers with a deadline absorb by re-probing
    ///   after the wait (the rmw H-2 pass).
    /// * `EINTR` retries with the remaining budget; a persistent poll
    ///   failure burns the remaining budget as a sleep (degrade to the
    ///   caller's timeout cadence, never a hot error loop) and is
    ///   `debug!`-logged.
    pub fn wait(&mut self, timeout: Duration) -> FdWake {
        // See `check`: `fired` is THIS call's report, cleared before the
        // zero-budget, empty-set and timed-out early returns.
        self.fired.clear();
        let start = Instant::now();
        loop {
            let Some(remaining) = timeout.checked_sub(start.elapsed()) else {
                return FdWake::TimedOut;
            };
            if remaining.is_zero() {
                return FdWake::TimedOut;
            }
            if self.fds.is_empty() {
                std::thread::sleep(remaining);
                return FdWake::TimedOut;
            }
            let rc = self.poll_once(remaining);
            if rc > 0 {
                let mut fired = false;
                self.fired.clear();
                for (i, p) in self.fds.iter_mut().enumerate() {
                    if p.revents & libc::POLLIN != 0 {
                        fired = true;
                        self.fired.push(i);
                    } else if p.revents != 0 {
                        // POLLNVAL / POLLERR / POLLHUP with nothing
                        // readable: the fd died under us. Stop watching it
                        // (poll(2) ignores negative fds) so a dead entry
                        // cannot re-fire every block until the caller's
                        // deadline — the select path would have aborted
                        // the process here instead.
                        p.fd = -1;
                    }
                    p.revents = 0;
                }
                if fired {
                    return FdWake::Fired;
                }
                continue; // only dead entries fired: re-poll the rest
            }
            if rc == 0 {
                // A timed-out slice (poll's ms floor can expire up to 1ms
                // before the real deadline); the loop head re-derives the
                // remainder and returns TimedOut once it is spent.
                continue;
            }
            let err = std::io::Error::last_os_error();
            if err.kind() == std::io::ErrorKind::Interrupted {
                continue;
            }
            tracing::debug!(
                error = %err,
                "FdWakeSet poll failed; degrading to timeout pacing"
            );
            std::thread::sleep(remaining);
            return FdWake::TimedOut;
        }
    }

    /// One kernel block of at most `remaining` — `ppoll(2)` with a
    /// nanosecond timespec, so a sub-ms budget is a real fd-armed block.
    /// Returns the raw poll result (`>0` fired, `0` timed out, `<0` error
    /// with errno set).
    #[cfg(target_os = "linux")]
    fn poll_once(&mut self, remaining: Duration) -> libc::c_int {
        let ts = libc::timespec {
            tv_sec: remaining.as_secs() as libc::time_t,
            tv_nsec: libc::c_long::from(remaining.subsec_nanos()),
        };
        // SAFETY: the pointer/len pair is `self.fds`, live across the call;
        // `ts` is live for the call; a null sigmask means "no mask change".
        // ppoll only reads fd/events and writes revents.
        unsafe {
            libc::ppoll(
                self.fds.as_mut_ptr(),
                self.fds.len() as libc::nfds_t,
                &ts,
                std::ptr::null(),
            )
        }
    }

    /// One kernel block of at most `remaining` on the `poll(2)` ms floor.
    /// macOS has no `ppoll`; this is the macOS fallback arm.
    ///
    /// A sub-millisecond budget is polled at ONE millisecond — never at 0
    /// (a nonblocking probe that turns the loop into a spin) and never
    /// slept: a sleep leaves the fd UNARMED for its whole length, and
    /// macOS timer coalescing stretches a 100 µs sleep to ~1 ms, so the
    /// rmw ladder's first rungs were both fd-blind and slow here (measured
    /// in the ping-pong discriminator: 399/400 server wakes `after_timeout`,
    /// RTT ≈ 1 ms). Polling at 1 ms keeps the fd wake immediate; the cost
    /// is up to 1 ms of overshoot on a sub-ms tail, which the sleep it
    /// replaces already paid in practice. Callers with a hard deadline
    /// re-probe after the wait (the rmw H-2 pass).
    #[cfg(not(target_os = "linux"))]
    fn poll_once(&mut self, remaining: Duration) -> libc::c_int {
        let ms = remaining.as_millis().clamp(1, i32::MAX as u128) as libc::c_int;
        // SAFETY: the pointer/len pair is `self.fds`, live across the call;
        // poll(2) only reads fd/events and writes revents.
        unsafe { libc::poll(self.fds.as_mut_ptr(), self.fds.len() as libc::nfds_t, ms) }
    }
}

#[cfg(unix)]
impl std::fmt::Debug for FdWakeSet {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FdWakeSet")
            .field("watched", &self.fds.len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The capacity refusal is a THRESHOLD, pinned on both sides at the exact
    /// value. Hand-written oracle: the cap itself is admitted, one past it is
    /// refused. (`WAITSET_MAX_ATTACHMENTS` is read from the shipped constant so
    /// this cannot drift into testing a number nobody uses.)
    #[test]
    fn the_wake_set_capacity_refusal_is_a_threshold_pinned_on_both_sides() {
        assert!(refuse_over_capacity(0).is_none(), "an empty set is legal");
        assert!(
            refuse_over_capacity(WAITSET_MAX_ATTACHMENTS - 1).is_none(),
            "one under the cap must be admitted"
        );
        assert!(
            refuse_over_capacity(WAITSET_MAX_ATTACHMENTS).is_none(),
            "the cap itself must be admitted (it is a maximum, not an exclusive bound)"
        );
        let over = refuse_over_capacity(WAITSET_MAX_ATTACHMENTS + 1)
            .expect("one past the cap must be refused");
        let msg = over.to_string();
        // The message must name the OFFENDING count and the cap — an operator
        // reading it should not have to look the constant up.
        assert!(
            msg.contains(&format!("{}", WAITSET_MAX_ATTACHMENTS + 1)),
            "the refusal must name the offending count: {msg}"
        );
        assert!(
            msg.contains(&format!("{WAITSET_MAX_ATTACHMENTS}")),
            "the refusal must name the cap: {msg}"
        );
    }

    // --- Doorbell + FdWakeSet (pure fd mechanics, no iceoryx2 —
    // --- parallel-safe). Wall assertions are LOWER bounds only (a spin
    // --- that returns early is the failure class; load can only lengthen).

    #[cfg(unix)]
    #[test]
    fn a_rung_doorbell_wakes_the_fd_set_and_a_drained_one_times_out() {
        let bell = Doorbell::new().expect("doorbell");
        let mut set = FdWakeSet::new();
        set.watch(bell.poll_fd());

        // Level-triggered: a ring BEFORE the wait is not lost.
        bell.ring();
        assert_eq!(
            set.wait(Duration::from_secs(5)),
            FdWake::Fired,
            "a pending ring must fire the wait"
        );

        // Drained: the same set must now spend its full timeout.
        bell.drain();
        let start = Instant::now();
        assert_eq!(set.wait(Duration::from_millis(60)), FdWake::TimedOut);
        assert!(
            start.elapsed() >= Duration::from_millis(60),
            "a quiet drained set must spend its timeout, not return early"
        );
    }

    #[cfg(unix)]
    #[test]
    fn multiple_rings_coalesce_and_one_drain_clears_them_all() {
        let bell = Doorbell::new().expect("doorbell");
        let mut set = FdWakeSet::new();
        set.watch(bell.poll_fd());
        bell.ring();
        bell.ring();
        bell.ring();
        assert_eq!(set.wait(Duration::from_secs(5)), FdWake::Fired);
        bell.drain();
        // The re-fire pin at the primitive level: no residue survives a
        // drain, so the next wait times out instead of instantly waking.
        let start = Instant::now();
        assert_eq!(set.wait(Duration::from_millis(50)), FdWake::TimedOut);
        assert!(start.elapsed() >= Duration::from_millis(50));
    }

    #[cfg(unix)]
    #[test]
    fn an_empty_set_sleeps_its_timeout() {
        let mut set = FdWakeSet::new();
        assert!(set.is_empty());
        let start = Instant::now();
        assert_eq!(set.wait(Duration::from_millis(60)), FdWake::TimedOut);
        assert!(
            start.elapsed() >= Duration::from_millis(60),
            "an empty set must be a bounded sleep, never an early return \
             (the caller paces on the timeout — the unbounded-spin class)"
        );
    }

    #[cfg(unix)]
    #[test]
    fn a_ring_from_another_thread_wakes_a_blocked_wait() {
        let bell = std::sync::Arc::new(Doorbell::new().expect("doorbell"));
        let mut set = FdWakeSet::new();
        set.watch(bell.poll_fd());
        let ringer = {
            let bell = std::sync::Arc::clone(&bell);
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(50));
                bell.ring();
            })
        };
        let start = Instant::now();
        assert_eq!(
            set.wait(Duration::from_secs(10)),
            FdWake::Fired,
            "a cross-thread ring must wake the blocked wait"
        );
        // Generous ceiling: the wake must come from the ring, not the 10s
        // timeout (no tight band — load only delays, never falsifies).
        assert!(start.elapsed() < Duration::from_secs(9));
        ringer.join().expect("ringer");
    }

    #[cfg(unix)]
    #[test]
    fn a_dead_fd_is_neutralized_not_spun_on() {
        // Watch an fd NUMBER that can never name a live descriptor (far
        // above any RLIMIT_NOFILE), so the kernel deterministically
        // reports POLLNVAL — a closed REAL fd would race parallel tests
        // re-using the number. The wait must spend its budget: a dead
        // entry is neutralized, never converted into an instant return.
        let mut set = FdWakeSet::new();
        set.watch(0x7FFF_FF00);
        let start = Instant::now();
        assert_eq!(set.wait(Duration::from_millis(60)), FdWake::TimedOut);
        assert!(
            start.elapsed() >= Duration::from_millis(60),
            "a dead watched fd must degrade to the timeout, never an early return"
        );
    }

    /// `fired()` must describe the LAST `wait`/`check` ONLY. Every early
    /// return — a timed-out wait, a zero-budget wait, an empty-set check —
    /// must leave it empty even when the previous call reported a wake: a
    /// stale index makes a fired-only drainer re-drain (and re-probe) an
    /// entity that did not wake this iteration. Mutant: drop the
    /// top-of-call clears ⇒ the stale index survives every early return.
    #[test]
    #[cfg(unix)]
    fn fired_is_cleared_before_every_early_return() {
        let bell = Doorbell::new().expect("doorbell");
        let mut set = FdWakeSet::new();
        set.watch(bell.poll_fd());

        // Fired → a timed-out wait.
        bell.ring();
        assert_eq!(set.wait(Duration::from_millis(200)), FdWake::Fired);
        assert_eq!(set.fired(), &[0]);
        bell.drain();
        assert_eq!(set.wait(Duration::from_millis(5)), FdWake::TimedOut);
        assert!(
            set.fired().is_empty(),
            "a timed-out wait must not report the previous wake's indices"
        );

        // Fired → a ZERO-budget wait (the early return before any poll).
        bell.ring();
        assert_eq!(set.wait(Duration::from_millis(200)), FdWake::Fired);
        assert_eq!(set.fired(), &[0]);
        bell.drain();
        assert_eq!(set.wait(Duration::ZERO), FdWake::TimedOut);
        assert!(set.fired().is_empty(), "a zero-budget wait must clear too");

        // Fired → check() on an EMPTY set, and wait() on an EMPTY set: the
        // stale state is planted by hand (nothing else can reach it, which
        // is exactly why the clears must sit BEFORE the early returns).
        let mut empty = FdWakeSet::new();
        empty.fired.push(7);
        assert_eq!(empty.check(), FdWake::TimedOut);
        assert!(empty.fired().is_empty(), "an empty-set check must clear");
        empty.fired.push(7);
        assert_eq!(empty.wait(Duration::from_millis(1)), FdWake::TimedOut);
        assert!(empty.fired().is_empty(), "an empty-set wait must clear");
    }
}
