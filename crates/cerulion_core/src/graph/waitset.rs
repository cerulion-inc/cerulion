// SPDX-License-Identifier: AGPL-3.0-only
//! The WaitSet reactor (event multiplexer) + its
//! determinism firewall — now PRODUCTION.
//!
//! # What this is
//!
//! [`WaitSetReactor`] wraps an iceoryx2 [`WaitSet`] and exposes a single
//! [`WaitSetReactor::run_once`] call that blocks (up to a timeout) until one
//! of the supplied [`Listener`]s receives an event, then **records which
//! sources fired** (in deterministic declaration order — see
//! [`WaitSetReactor::run_once`]) into a pre-sized `Vec` and returns it. It is
//! the reactor half of the event-driven live loop: the other half is
//! the production
//! [`crate::graph::GraphRuntime::run_live`] driver, which blocks on this
//! reactor for a real wakeup and then calls the EXISTING
//! [`crate::graph::GraphRuntime::step`] — driving the step loop from real
//! iceoryx2 events instead of fixed-`delta` polling. The reactor is the
//! "WHEN to step"; `step`/`drain_level` remains the "WHAT fires".
//!
//! # Determinism firewall (NON-NEGOTIABLE)
//!
//! The reactor only changes WHEN `step()` is called, never its body. The
//! production firing path (`Scheduler::step` / `evaluate_node` / `fire_node`,
//! `GraphRuntime::step` / its per-level `drain_level` passes) is byte-identical
//! whether or not the reactor drives it — none of those functions call into
//! this module, and `run_live` ignores
//! WHICH sources the reactor reports (the fired-set is a record-only
//! artifact, validated by the still-gated
//! [`crate::graph::runtime::GraphRuntime::run_waitset_reactor_once_for_test`]
//! seam). A wakeup is purely a "step now" signal; the deterministic
//! per-level `drain_level` passes do the real `signal_data` + firing. So the
//! live path's firing sequence is byte-identical to the polled path's.
//!
//! The reactor's `wait_and_process` callback is **record-only**: it appends
//! the fired source's index to `self.fired_idx`, drains that listener's EVENT
//! queue (the notification channel), and returns. After the wait completes,
//! `run_once` sorts the recorded indices ascending and materialises the node
//! ids into `self.fired` (declaration order). It MUST NEVER call
//! `signal_data` / `signal_input_received` / `signal_sync_input` /
//! `trigger_external` / `fire_node` or any scheduler mutation.
//!
//! Draining the listener's EVENT queue is NOT a firewall violation: the event
//! queue is iceoryx2's notification channel, entirely separate from the SHM
//! MESSAGE queue. Recording (and clearing the wakeup notification) does NOT
//! consume the iceoryx2 data sample — the deterministic `drain_level`
//! (`try_receive` off the message queue) remains the sole reader of the data.
//! Clearing the notification is what the iceoryx2 WaitSet doc pattern itself
//! does (`listener.try_wait_one()` in the callback) and is required so a
//! subsequent `run_once` on a quiet graph does not re-report a stale
//! connection-lifecycle event (`SubscriberConnected` / `PublisherConnected`
//! are queued on the listener at build, independent of any data publish).
//!
//! This separation is what lets the WaitSet observe the same `Listener`s the
//! scheduler's data path will later drain, without perturbing the
//! deterministic trace.
//!
//! # The record-only PRE-block spin
//!
//! The live loop runs a scheduled spin-then-block
//! ([`crate::graph::GraphRuntime::spin_sources`]): before each blocking
//! `run_once`, the loop busy-polls these SAME `Listener`s for a short budget
//! (so an imminent wake is caught in user space, skipping the WaitSet's
//! `epoll` / C-state-exit round-trip). It drains the SAME notification/event
//! queue under the SAME firewall as the reactor's callback — notification-only
//! (`listener.try_wait_one()`), NEVER the SHM message queue, NEVER any
//! scheduler mutation or clock advance. Like the reactor, it only changes WHEN
//! the loop wakes, never WHAT fires or the fire order (that is decided entirely
//! inside `step()`/`drain_level`). The firewall above applies to it verbatim.
//!
//! # Mixed Listener + raw-fd wake sources
//!
//! [`WaitSetReactor::run_once`] now accepts a heterogeneous [`WaitSource`] slice
//! — each entry is either an iceoryx2 [`Listener`] (as before) or a non-owning
//! device [`FdSource`] (a driver-ingress external trigger). Both attach to the
//! same [`WaitSet`] (iceoryx2's `attach_notification` accepts any
//! `SynchronousMultiplexing` type) and demux through the same
//! `WaitSetAttachmentId` `BTreeMap`, so mixing them needs no new bookkeeping.
//!
//! **The firewall extends unchanged to fd sources, and is if anything STRICTER
//! for them:** a device fd is WAKE-only and LEVEL-triggered. When one fires the
//! callback records its index ONLY — it NEVER reads the fd. The external node's
//! own `tick` drains its device; the runtime must never consume device data
//! (Principle #2 — data is truth). The fired-set stays the record-only artifact
//! `run_live` IGNORES (it blocks purely for the wakeup, then `step`s), so fd
//! sources change only WHEN the loop wakes, never WHAT fires. This module is the
//! reactor ONLY: no scheduler marking, no macro wiring — a listener-
//! only graph runs byte-identically to one built before fd sources existed.
//!
//! **Two abort hazards are guarded at the boundary** (both would abort the whole
//! process): a stale fd would trip iceoryx2's `select` EBADF `fatal_panic`, so
//! `run_once` probes [`FdSource::is_valid`] (`fcntl F_GETFD`) and SKIPS an
//! invalid fd before attaching (a best-effort TOCTOU narrowing — see the
//! HAZARD 1 note on [`FdSource`]); and an fd `>= FD_SETSIZE` would be
//! out-of-bounds in iceoryx2's unchecked `FD_SET`, so `FdSource::non_owning`
//! (the test-only SELECT-path constructor) rejects it at construction, and the
//! production [`classify`] refuses it as [`FdVerdict::AboveSelectLimit`]. See
//! [`FdSource`] for the full contract.
//!
//! **The ceiling is a SELECT-path concern only.** The `FD_SETSIZE`
//! refusal exists because THIS reactor watches fds with `select`/`FD_SET`. The
//! monitor-wait park / barrier idle path
//! ([`crate::graph::GraphRuntime::monitor_wait_block`]) never reaches this
//! reactor — it polls each external fd with `poll(2)`, which has NO fd-number
//! ceiling — so a LIVE fd `>= FD_SETSIZE` is watchable there. [`classify`]
//! splits the three refusal causes `non_owning` flattens into one `None` so the
//! park/barrier validation site can lift ONLY the ceiling (via
//! `classify(raw, /* polled_not_selected */ true)`) while a negative or dead fd
//! is still refused on every path.

use std::collections::BTreeMap;
use std::os::unix::io::RawFd;
use std::sync::Arc;
use std::time::Duration;

use iceoryx2::port::listener::Listener;
use iceoryx2::prelude::{
    CallbackProgression, SignalHandlingMode, WaitSet, WaitSetAttachmentId, WaitSetBuilder,
};
use iceoryx2::waitset::WaitSetCreateError;
use iceoryx2_bb_posix::file_descriptor::{FileDescriptor, FileDescriptorBased};
use iceoryx2_bb_posix::file_descriptor_set::SynchronousMultiplexing;

use crate::transport::CerService;

/// A NON-OWNING raw file descriptor wrapped so it can be
/// attached to the iceoryx2 [`WaitSet`] alongside iceoryx2 [`Listener`]s.
///
/// This is the reactor foundation for the driver-ingress external trigger (an
/// `#[cerulion_node(external)]` node that self-triggers off a device fd or a
/// blocking SDK call). A wired-in `FdSource` lets the live loop BLOCK on the
/// device fd in the SAME WaitSet as the graph's data-trigger listeners, so a
/// readable device wakes the loop as promptly as a published message.
///
/// # Non-owning contract (Principle #11 — no leaks / no double-close)
///
/// The inner [`FileDescriptor`] is built with [`FileDescriptor::non_owning_new`]:
/// its `Drop` NEVER calls `close(2)`, so the device fd's lifetime stays with
/// whoever opened it (the node / SDK). `FdSource` therefore does NOT impl
/// `Clone` — the owning `FileDescriptor::clone` `dup(2)`s to an OWNED fd whose
/// `Drop` WOULD close, re-introducing the double-close hazard. Copying a
/// `WaitSource::Fd(&FdSource)` reference is fine (it is `&FdSource`, not a clone).
///
/// # Hazards this newtype guards (both would ABORT the whole process)
///
/// - **HAZARD 1 — stale fd ⇒ EBADF fatal abort.** iceoryx2's `select`-backed
///   wait maps `EBADF` to a `fatal_panic` (whole-process abort). A device fd
///   that was valid at construction but has since been closed would trip that
///   abort on the next wait. [`Self::is_valid`] probes the fd (`fcntl F_GETFD`)
///   so [`WaitSetReactor::run_once`] can SKIP a source that is already stale at
///   attach time. This is a BEST-EFFORT TOCTOU mitigation: it narrows the
///   stale-fd window to probe→select within one cycle — an fd closed in that
///   window still trips the upstream abort. The fd-lifetime contract on
///   `ExternalSource::Fd` documents that nodes must not close a handed-over fd
///   outside `tick()`.
/// - **HAZARD 2 — fd value ≥ `FD_SETSIZE` ⇒ out-of-bounds UB (SELECT path only).**
///   iceoryx2's `FileDescriptorSet::add` calls `FD_SET` without bounds-checking
///   the fd value, so an fd `>= libc::FD_SETSIZE` (1024) is out-of-bounds UB when
///   the WaitSet watches it with `select`. The test-only `Self::non_owning`
///   REJECTS any raw `>= FD_SETSIZE` (returns `None`) so such an fd can never
///   reach `FD_SET`; the production [`classify`] reports it as
///   [`FdVerdict::AboveSelectLimit`] on the select path.
///   This ceiling is enforced ONLY on the select path. The monitor-wait
///   park / barrier idle path polls each external fd via `poll(2)` (no fd-number
///   ceiling), so [`classify`] with `polled_not_selected == true` lets a LIVE high
///   fd through — the ceiling refusal is scoped to callers that actually `FD_SET`.
///
/// The PRODUCTION constructor is wired: `FdSource::non_owning` is
/// now called from [`crate::graph::GraphRuntime`]'s `collect_external_sources`
/// (run_live entry) for every `#[cerulion_node(external)]` node that returns
/// [`crate::graph::node::ExternalSource::Fd`], and the resulting `FdSource`s are
/// chained into the live loop's WaitSet source list + polled by
/// `sweep_external_sources`. So this type is genuinely constructed and every
/// method exercised in every build mode — it needs no `allow(dead_code)`
/// scaffold.
#[derive(Debug)]
pub(crate) struct FdSource(FileDescriptor);

// Compile-time guard: FdSource must NEVER be Clone/Copy —
// `FileDescriptor::clone` dup(2)s to an OWNED fd whose Drop closes,
// silently converting non-owning to owning (double-close hazard). If a
// future edit derives Clone/Copy on FdSource, `some_item` below becomes
// ambiguous (two applicable impls) and this file stops compiling.
// (Dep-free variant of static_assertions' `assert_not_impl_any!`. The
// Clone impl carries no `?Sized` — `Clone: Sized` makes it redundant and
// clippy's `needless_maybe_sized` rejects it.)
const _: () = {
    trait AmbiguousIfClone<A> {
        fn some_item() {}
    }
    impl<T: ?Sized> AmbiguousIfClone<()> for T {}
    struct Invalid;
    impl<T: Clone> AmbiguousIfClone<Invalid> for T {}
    let _ = <FdSource as AmbiguousIfClone<_>>::some_item;
};

/// The classified outcome of validating a raw fd as a WaitSet source.
///
/// The test-only `FdSource::non_owning` collapses every refusal into a single
/// `None`, which is exactly right for the SELECT-backed reactor callers (a high fd is as
/// unwatchable there as a dead one). But the park/barrier idle path polls fds via
/// `poll(2)`, which has NO `FD_SETSIZE` ceiling, so a LIVE high fd IS watchable
/// there. This verdict keeps the three refusal causes DISTINCT so the caller can
/// lift ONLY the select-ceiling refusal when it will poll rather than select,
/// while still refusing a negative or dead fd on every path.
pub(crate) enum FdVerdict {
    /// A live fd (in-range for select, or any live fd on the poll path) wrapped as
    /// a non-owning WaitSet source.
    Usable(FdSource),
    /// `raw < 0` — not a real fd on ANY path.
    Negative,
    /// `raw >= libc::FD_SETSIZE` (1024) AND the caller SELECTs — HAZARD 2:
    /// out-of-bounds in iceoryx2's unchecked `FD_SET`. Produced ONLY on the select
    /// path; a `poll(2)` caller (`polled_not_selected == true`) lifts the ceiling
    /// and falls through to the liveness probe instead.
    AboveSelectLimit,
    /// Not a currently-open descriptor (HAZARD 1 — `FileDescriptor::non_owning_new`
    /// / `fcntl F_GETFD` failed). Dead on ANY path.
    Dead,
}

/// Classify `raw` as a WaitSet source, splitting the three refusal
/// causes the test-only `FdSource::non_owning` flattens into one `None`.
///
/// `polled_not_selected` is the caller's contract about HOW the fd will be
/// watched:
/// - `false` = the SELECT-backed iceoryx2 [`WaitSet`] (the default live-loop
///   reactor + every test reactor seam), which cannot watch an fd `>= FD_SETSIZE`
///   (HAZARD 2 — out-of-bounds `FD_SET`);
/// - `true` = the monitor-wait park / barrier idle path, which polls each fd via
///   `poll(2)` and so has NO fd-number ceiling.
///
/// The KEY property: with `polled_not_selected == true` a high fd FALLS THROUGH
/// to the liveness probe — only the ceiling lifts. A negative fd (`Negative`) and
/// a dead fd (`Dead`) are still refused on EVERY path (the park polls the fd, but
/// a `poll(2)` on a closed fd returns `POLLNVAL` forever — never watch a dead fd).
pub(crate) fn classify(raw: RawFd, polled_not_selected: bool) -> FdVerdict {
    if raw < 0 {
        return FdVerdict::Negative;
    }
    // HAZARD 2: iceoryx2's `FileDescriptorSet::add` never guards the fd value
    // before `FD_SET`, so an fd at/above FD_SETSIZE is out-of-bounds UB on the
    // select path. The park/barrier path polls via `poll(2)` (no ceiling), so it
    // skips this refusal and lets a live high fd through to the liveness probe.
    if !polled_not_selected && (raw as usize) >= libc::FD_SETSIZE {
        return FdVerdict::AboveSelectLimit;
    }
    // HAZARD 1: `non_owning_new` probes `fcntl F_GETFD`, so a dead fd is refused
    // here on EVERY path.
    match FileDescriptor::non_owning_new(raw) {
        Some(fd) => FdVerdict::Usable(FdSource(fd)),
        None => FdVerdict::Dead,
    }
}

impl FdSource {
    /// Wrap `raw` as a NON-OWNING WaitSet source for a SELECT-backed caller, or
    /// `None` if it can never be attached safely.
    ///
    /// Rejects (returns `None`):
    /// - `raw < 0` (not a real fd);
    /// - `raw >= libc::FD_SETSIZE` (HAZARD 2 — would be out-of-bounds in
    ///   iceoryx2's `FD_SET`);
    /// - a `raw` that is not currently a valid open fd (via
    ///   [`FileDescriptor::non_owning_new`], which itself probes `fcntl F_GETFD`).
    ///
    /// The returned `FdSource` is non-owning: dropping it never closes `raw`.
    ///
    /// This is the SELECT-path constructor, now used ONLY by the
    /// test-only reactor seams (which watch their fds with `select`/`FD_SET`), so
    /// it is gated behind `#[cfg(any(test, feature = "test-helpers"))]`. It keeps
    /// the full `FD_SETSIZE` ceiling by delegating to [`classify`] with
    /// `polled_not_selected == false` — negative / above-limit / dead all collapse
    /// to `None`, byte-identical to the earlier three-guard body. The PRODUCTION
    /// park/barrier validation site in `GraphRuntime::collect_external_sources`
    /// calls [`classify`] DIRECTLY with the poll-path flag (so it can lift ONLY the
    /// ceiling for a live high fd and report the distinct `AboveSelectLimit`), which
    /// is why this thin `Option` wrapper no longer has a production caller.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn non_owning(raw: RawFd) -> Option<Self> {
        match classify(raw, false) {
            FdVerdict::Usable(fd) => Some(fd),
            FdVerdict::Negative | FdVerdict::AboveSelectLimit | FdVerdict::Dead => None,
        }
    }

    /// HAZARD 1 probe: is the wrapped fd still a valid open descriptor?
    ///
    /// `fcntl(fd, F_GETFD)` is the cheapest liveness check — it returns `-1`
    /// (setting `errno` to `EBADF`) for a closed/invalid fd and `>= 0`
    /// otherwise. [`WaitSetReactor::run_once`] calls this before attaching an
    /// `FdSource` so a stale fd is skipped rather than tripping iceoryx2's
    /// `select` EBADF fatal abort.
    pub(crate) fn is_valid(&self) -> bool {
        // SAFETY: `native_handle` reads the wrapped fd's raw value transiently
        // for the immediately-following `fcntl`; it is not stored (upholding the
        // accessor's contract). `fcntl(_, F_GETFD)` is a read-only query with no
        // side effects.
        unsafe { libc::fcntl(self.0.native_handle(), libc::F_GETFD) != -1 }
    }

    /// The wrapped fd's raw value, for the non-blocking
    /// `poll(2)` readiness probe in `GraphRuntime::sweep_external_sources`.
    ///
    /// Non-owning like [`Self::is_valid`]: the value is read transiently and
    /// never stored, so exposing it does not affect the fd's lifetime (the node
    /// owns the device; the runtime never reads or closes it).
    pub(crate) fn raw(&self) -> RawFd {
        // SAFETY: transient read of the wrapped fd's raw value for the
        // immediately-following `poll(2)`; not stored (upholds the accessor's
        // contract, same as `is_valid`'s `fcntl` probe).
        unsafe { self.0.native_handle() }
    }
}

impl FileDescriptorBased for FdSource {
    fn file_descriptor(&self) -> &FileDescriptor {
        &self.0
    }
}

// Zero-method marker: opts `FdSource` into `WaitSet::attach_notification`
// (bounded by `SynchronousMultiplexing`). `FileDescriptorBased` above supplies
// the fd the WaitSet's reactor watches.
impl SynchronousMultiplexing for FdSource {}

/// A single WaitSet wake source — either an iceoryx2
/// [`Listener`] (a graph data-trigger / sync input's notification channel) or a
/// non-owning raw device fd ([`FdSource`], the driver-ingress external trigger).
///
/// Both variants attach to the SAME iceoryx2 [`WaitSet`] via
/// `attach_notification` (which accepts any `SynchronousMultiplexing` type) and
/// demux back to their source index through the SAME `WaitSetAttachmentId`
/// [`BTreeMap`] in [`WaitSetReactor::run_once`] — the reactor treats a device-fd
/// wake and a listener wake uniformly as a "step now" signal.
///
/// `Copy` because it wraps only a shared reference (no owned fd) — copying it
/// never dups or closes anything.
#[derive(Clone, Copy)]
pub(crate) enum WaitSource<'a> {
    /// An iceoryx2 event listener (a graph trigger/sync input's wake channel).
    Listener(&'a Listener<CerService>),
    /// A non-owning raw device fd (a driver-ingress external trigger).
    ///
    /// This is constructed from the live loop's external-binding
    /// wiring (`GraphRuntime::collect_external_sources` chains each `DeviceFd`
    /// binding as `WaitSource::Fd` into `live_step`'s source list) in addition to
    /// the `#[cfg(test/test-helpers)]` reactor seams, so it is production-live in
    /// every build — see [`FdSource`] (it needs no `allow(dead_code)`
    /// scaffold).
    Fd(&'a FdSource),
}

/// Conservative cross-platform upper bound on the number of
/// iceoryx2 WaitSet notification attachments the live loop can hold at once.
///
/// iceoryx2's `WaitSet` is FD-set bounded on the `select`-backed path
/// (`FD_SETSIZE`, 1024 on Linux/macOS); past that bound `attach_notification`
/// silently drops sources. The live loop
/// ([`crate::graph::GraphRuntime::run_live`]) attaches exactly one
/// notification per `DataTrigger` trigger input and per `Sync` input — i.e.
/// `data_trigger_bindings.len() + sync_input_bindings.len()` attachments. A
/// graph exceeding this would lose the prompt event-driven wakeup for the
/// dropped sources, degrading them to the 250 ms liveliness-sweep cadence — a
/// latency regression, NOT data loss: the level executor still calls
/// `drain_level` for every level on each `step`, so every binding is drained
/// once per step in aggregate, regardless of WaitSet attachment. The graph build
/// rejects such graphs up front via [`check_waitset_attachment_capacity`] so
/// the COUNT-driven failure is a loud, actionable build error instead of a
/// quiet runtime degradation (a per-`run_once` "failed to attach a listener"
/// warn storm).
///
/// This guard rules out only the FD-set-*count* cause. Other attach failures —
/// e.g. a process `RLIMIT_NOFILE` tightened BELOW this cap, so a build that
/// passes the guard still can't open all its FDs at run time — remain a
/// per-`run_once` `tracing::warn!` (still loud, still not data loss: the
/// affected sources just fall back to the heartbeat cadence). The cap is
/// `FD_SETSIZE`-conservative, so on an `epoll`/`kqueue`-backed iceoryx2 build
/// the *effective* runtime limit can be higher (the guard may reject a graph
/// that would have attached) — a deliberate fail-closed direction: it can
/// produce a false rejection, never a false acceptance.
pub const WAITSET_MAX_ATTACHMENTS: usize = 1024;

/// Fail-fast build-time guard that the live-loop WaitSet
/// attachment count (`data_trigger_count + sync_input_count +
/// external_source_count`) stays within [`WAITSET_MAX_ATTACHMENTS`].
///
/// Pure (no transport, no iceoryx2) so the boundary contract is directly
/// unit-testable without provisioning thousands of real ports. Called once
/// per graph build from `GraphRuntime::build_with_scheduler` after the binding
/// vectors are populated — the single choke point every build path funnels
/// through. `saturating_add` keeps the count itself panic-free even for
/// pathological inputs (the cap rejects them anyway).
///
/// `external_source_count` is the number of driver-ingress
/// raw-fd wake sources (`WaitSource::Fd`) the live loop will attach — each
/// consumes one WaitSet attachment slot exactly like a listener. The graph build
/// passes a conservative upper bound (one per External-policy node), because the
/// exact set is only known when the live loop collects its sources.
pub fn check_waitset_attachment_capacity(
    data_trigger_count: usize,
    sync_input_count: usize,
    external_source_count: usize,
) -> crate::error::TransportResult<()> {
    let total = data_trigger_count
        .saturating_add(sync_input_count)
        .saturating_add(external_source_count);
    if total > WAITSET_MAX_ATTACHMENTS {
        return Err(crate::error::TransportError::GraphError {
            reason: format!(
                "graph needs {total} live-loop WaitSet event attachments \
                 ({data_trigger_count} data-trigger inputs + {sync_input_count} \
                 sync inputs + {external_source_count} external fd sources), but \
                 the WaitSet supports at most {WAITSET_MAX_ATTACHMENTS} (iceoryx2 \
                 is FD-set bounded, ~FD_SETSIZE); past the cap, trigger listeners \
                 silently fail to attach and those nodes lose prompt event-driven \
                 wakeups, degrading to the 250 ms liveliness cadence — split the \
                 graph across processes to stay under the cap"
            ),
        });
    }
    Ok(())
}

#[cfg(any(test, feature = "test-helpers"))]
static FORCE_ATTACH_FAILURE: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Test-only: force every `attach_notification`
/// in `run_once` to be skipped (as if it failed), so `guards.is_empty()` →
/// `last_wait_blocked` stays `false`. Lets a test mutation-verify the 25a
/// busy-spin guard WITHOUT real FD exhaustion. Process-global; callers must
/// `#[serial]` and reset to false after use.
#[cfg(any(test, feature = "test-helpers"))]
pub(crate) fn set_force_attach_failure_for_test(on: bool) {
    FORCE_ATTACH_FAILURE.store(on, std::sync::atomic::Ordering::Relaxed);
}

/// An iceoryx2 [`WaitSet`]-backed event multiplexer that
/// blocks until one of a set of [`Listener`]s receives an event and records
/// which sources fired (record-only — see the module-level firewall note).
/// The production [`crate::graph::GraphRuntime::run_live`] driver uses it for
/// the BLOCK (the "when to step"); the fired-set itself is consumed only by
/// the still-gated `run_waitset_reactor_once_for_test` firewall seam.
///
/// The fired-set is returned in **declaration order** (ascending source index
/// — the order entries appear in the `sources` slice, which `run_live` builds
/// from the graph's `data_trigger_bindings` wiring order). This is a
/// deterministic, replay-stable contract: dispatch order is independent of
/// iceoryx2's internal attachment / callback order, which is undocumented and
/// could change between releases. A stable order keeps any future fired-set
/// consumer bit-for-bit reproducible (Principle #7: Replay = Live).
///
/// The [`WaitSet`] is owned for the reactor's life; the per-source
/// `WaitSetGuard`s (which borrow BOTH the waitset and the listeners) are kept
/// strictly LOCAL to [`Self::run_once`] so the struct never becomes
/// self-referential (the guards detach when `run_once` returns).
pub(crate) struct WaitSetReactor {
    /// The owned event multiplexer. Built with
    /// [`SignalHandlingMode::Disabled`] — `cerulion graph run` owns SIGINT
    /// separately, so the reactor must not swallow termination signals.
    waitset: WaitSet<CerService>,
    /// Recorded fired-source node ids from the most recent [`Self::run_once`],
    /// in declaration order. Pre-sized at construction and cleared (not
    /// reallocated) at the start of each `run_once`, so steady-state runs are
    /// alloc-free.
    fired: Vec<Arc<str>>,
    /// Scratch buffer of fired-source INDICES (into the `sources` slice),
    /// recorded by the callback in iceoryx2's internal order, then
    /// sorted ascending so `fired` is materialised in declaration order.
    /// Pre-sized + cleared (not reallocated) per `run_once` — alloc-free in
    /// steady state.
    fired_idx: Vec<usize>,
    /// Did the most recent [`Self::run_once`] actually BLOCK
    /// on `wait_and_process_*` (i.e. attach ≥1 source)? `false` when it
    /// short-circuited — no sources, or every `attach_notification` failed — in
    /// which case it returned WITHOUT consuming `timeout`. The
    /// [`crate::graph::GraphRuntime::run_live`] loop reads this so it can sleep
    /// the heartbeat itself in that case rather than busy-spin at 100% CPU.
    /// Reset to `false` at the top of every `run_once`.
    last_wait_blocked: bool,
}

impl WaitSetReactor {
    /// Build a reactor with an empty [`WaitSet`]. Sources are attached
    /// per-call in [`Self::run_once`] (their guards are local to that call).
    ///
    /// `capacity_hint` pre-sizes BOTH the `fired` and `fired_idx` buffers so a
    /// steady-state `run_once` records into already-reserved space.
    // hot-path-alloc-ok-fn: cold: reactor CONSTRUCTION — the whole point of `capacity_hint` is that
    // these two buffers are reserved HERE so `run_once` records into already-reserved space
    pub(crate) fn new(capacity_hint: usize) -> Result<Self, WaitSetCreateError> {
        let waitset = WaitSetBuilder::new()
            .signal_handling_mode(SignalHandlingMode::Disabled)
            .create::<CerService>()?;
        Ok(Self {
            waitset,
            fired: Vec::with_capacity(capacity_hint),
            fired_idx: Vec::with_capacity(capacity_hint),
            last_wait_blocked: false,
        })
    }

    /// Block until one of `sources` receives an event (or `timeout` elapses),
    /// then return the node ids of every source that fired, in **declaration
    /// order** (ascending index into `sources`).
    ///
    /// Each entry in `sources` pairs a [`WaitSource`] (an iceoryx2 [`Listener`]
    /// OR a non-owning device [`FdSource`]) with the node id to record when that
    /// source fires. The reactor attaches every source as a WaitSet notification
    /// (collecting the guards in a LOCAL `Vec`), builds a LOCAL [`BTreeMap`] from
    /// attachment id → index into `sources` (the iceoryx2-documented lookup
    /// pattern), runs one `wait_and_process_once_with_timeout`, and in the
    /// callback looks up the fired attachment and pushes its `sources` INDEX into
    /// `self.fired_idx`. After the wait returns, the recorded indices are sorted
    /// ascending and mapped back to their node ids in `self.fired`.
    ///
    /// **Determinism:** iceoryx2 invokes the callback in an internal,
    /// undocumented order. Recording the index and sorting it ascending makes
    /// the returned fired-set follow the caller's declaration order regardless
    /// of that internal order — a replay-stable dispatch sequence
    /// (Principle #7).
    ///
    /// **Firewall (record-only, both source kinds):** the callback NEVER mutates
    /// the scheduler. For a [`WaitSource::Listener`] it appends to
    /// `self.fired_idx` and clears the fired listener's EVENT queue (the
    /// notification channel, NOT the SHM message queue — the iceoryx2 data sample
    /// stays queued for the deterministic `drain_level`). For a
    /// [`WaitSource::Fd`] it appends to `self.fired_idx` ONLY — it NEVER reads
    /// the raw fd: the fd is level-triggered and the external node's own tick
    /// drains its device, so the runtime must never consume device data. The
    /// returned slice borrows `self.fired`, valid until the next `run_once`. The
    /// local guards drop at end of scope (detaching every source) so the
    /// [`WaitSet`] is empty again on return.
    ///
    /// **Stale-fd safety (HAZARD 1):** before attaching a [`WaitSource::Fd`] the
    /// reactor probes [`FdSource::is_valid`]; an invalid (stale/closed) fd is
    /// SKIPPED with a `tracing::warn!` and never attached. Best-effort: this
    /// narrows the stale-fd window to probe→select within one cycle — an fd
    /// closed in that window still trips iceoryx2's `select` EBADF fatal abort
    /// (the fd-lifetime contract on `ExternalSource::Fd`: nodes must not close
    /// a handed-over fd outside `tick()`). Listener sources are iceoryx2-owned
    /// and cannot go stale this way, so they skip the probe.
    ///
    /// `run_live` ignores the returned slice — it blocks on
    /// this call purely for the wakeup, then steps. The fired-set remains the
    /// record-only artifact the gated firewall seam asserts on.
    pub(crate) fn run_once(
        &mut self,
        sources: &[(WaitSource<'_>, Arc<str>)],
        timeout: Duration,
    ) -> &[Arc<str>] {
        self.fired.clear();
        self.fired_idx.clear();
        // Reset the blocked flag; set true only once we have ≥1 attached source
        // (the path that actually calls `wait_and_process_*` and consumes
        // `timeout`). Every short-circuit below leaves it `false`.
        self.last_wait_blocked = false;

        // iceoryx2 0.9.1 `wait_and_process_once_with_timeout` returns
        // `Err(WaitSetRunError::NoAttachments)` on an empty WaitSet (it rejects
        // the zero-attachment call — it does not hang). With nothing attached
        // there is nothing to fire, so short-circuit to the empty record and
        // skip that error/warn path.
        if sources.is_empty() {
            return &self.fired;
        }

        // Attach every source as a notification. The guards borrow BOTH the
        // waitset and the listeners; keeping them in this LOCAL `Vec` (not on
        // the struct) is what avoids the self-referential-struct trap — they
        // detach when this `Vec` drops at end of `run_once`.
        // hot-path-alloc-known: one `Vec` per WAIT on the live reactor loop. The guards must be a
        // LOCAL collection — they borrow both the waitset and the listeners, which is what avoids
        // the self-referential-struct trap — so hoisting them onto the struct is not a rename.
        // Fixing it needs a different ownership shape for the attachment guards; recorded rather
        // than claimed cold
        let mut guards = Vec::with_capacity(sources.len());
        // attachment id → index into `sources`. iceoryx2 documents this
        // BTreeMap lookup pattern (`WaitSetAttachmentId: Ord`) for resolving a
        // fired attachment back to its source.
        // hot-path-alloc-known: the SIBLING of the guards `Vec` above — one `BTreeMap` per WAIT
        // on the live reactor loop, keyed by an attachment id that only exists once the guards
        // are attached, so it cannot outlive them any more than they can. Surfaced by the
        // hot-path allocation lint's pattern widening, which added the map
        // constructors the old regex was missing. Same fix as the guards: a different ownership
        // shape for the attachments; recorded rather than claimed cold
        let mut by_attachment: BTreeMap<WaitSetAttachmentId<CerService>, usize> = BTreeMap::new();
        for (idx, (source, node_id)) in sources.iter().enumerate() {
            #[cfg(any(test, feature = "test-helpers"))]
            if FORCE_ATTACH_FAILURE.load(std::sync::atomic::Ordering::Relaxed) {
                tracing::warn!(
                    source_idx = idx,
                    node_id = %node_id,
                    "WaitSetReactor: forced attach failure (test seam)"
                );
                continue;
            }
            // HAZARD 1: iceoryx2's `select`-backed wait maps EBADF to a
            // `fatal_panic` (WHOLE-PROCESS abort). A raw device fd that has gone
            // stale (closed since `FdSource` construction) would trip that abort
            // on the next wait — so probe validity BEFORE attaching and SKIP one
            // that is already invalid. Best-effort TOCTOU mitigation: it narrows
            // the stale-fd window to probe→select within one cycle; an fd closed
            // in that window still trips the upstream abort (the
            // `ExternalSource::Fd` lifetime contract: nodes must not close a handed-over fd outside
            // tick()). Listener sources are iceoryx2-owned and cannot go stale
            // this way, so they skip the probe.
            if let WaitSource::Fd(fd) = source {
                if !fd.is_valid() {
                    tracing::warn!(
                        source_idx = idx,
                        node_id = %node_id,
                        reason = "stale/invalid fd skipped to avoid iceoryx2 select EBADF fatal abort",
                        "WaitSetReactor: skipping raw-fd source for this run_once cycle"
                    );
                    continue;
                }
            }
            let attach_result = match source {
                WaitSource::Listener(listener) => self.waitset.attach_notification(*listener),
                WaitSource::Fd(fd) => self.waitset.attach_notification(*fd),
            };
            match attach_result {
                Ok(guard) => {
                    by_attachment.insert(WaitSetAttachmentId::from_guard(&guard), idx);
                    guards.push(guard);
                }
                Err(e) => {
                    // A failed attach means this source can't be observed this
                    // cycle. Record-only contract holds (nothing fires); the
                    // already-attached sources still wait. Loud-warn so a
                    // capacity/resource exhaustion surfaces — with the NODE ID,
                    // since the operational consequence is that THIS node
                    // degrades to the 250 ms liveliness cadence and the operator
                    // needs to know which one (`source_idx` is an opaque index).
                    // (iceoryx2 maps BOTH an over-capacity attach and a genuine
                    // duplicate attach to `WaitSetAttachmentError::AlreadyAttached`,
                    // so either is a possible cause here.)
                    tracing::warn!(
                        error = %e,
                        source_idx = idx,
                        node_id = %node_id,
                        "WaitSetReactor: failed to attach a source (AlreadyAttached \
                         may mean either a duplicate attach OR capacity exceeded); \
                         this source is not observed for this run_once cycle"
                    );
                }
            }
        }

        // If EVERY attach failed the waitset is still empty — short-circuit
        // before `wait_and_process_*` (which would return
        // `WaitSetRunError::NoAttachments`) and return the empty record.
        if guards.is_empty() {
            return &self.fired;
        }

        // ≥1 source attached, so we WILL block on `wait_and_process_*` below
        // (consuming up to `timeout`). Mark it so `run_live` knows this
        // iteration blocked and need not sleep the heartbeat itself.
        self.last_wait_blocked = true;

        // RECORD-ONLY callback. `fired_idx` is captured by &mut; the callback
        // looks the fired attachment up, records the source's INDEX (so the
        // post-wait sort can put the fired-set in declaration order), and
        // drains that listener's EVENT queue (mirroring the iceoryx2 WaitSet
        // doc pattern, which calls `listener.try_wait_one()` in the callback).
        //
        // FIREWALL: draining the listener's event queue is NOT a scheduler
        // mutation and is NOT a fire. The event queue is iceoryx2's
        // notification channel — entirely separate from the SHM message queue
        // that the deterministic `drain_level` reads via `try_receive`. The
        // data sample is never consumed here; only the wakeup notification is
        // cleared (so a subsequent `run_once` on a quiet graph doesn't
        // re-report the same stale connection-lifecycle event — e.g. the
        // `SubscriberConnected`/`PublisherConnected` events queued at build).
        // The callback performs NO `signal_data`/`signal_input_received`/
        // `trigger_external`/`fire_node`.
        let fired_idx = &mut self.fired_idx;
        let on_event = |attachment_id: WaitSetAttachmentId<CerService>| -> CallbackProgression {
            if let Some(&idx) = by_attachment.get(&attachment_id) {
                match &sources[idx].0 {
                    WaitSource::Listener(listener) => {
                        fired_idx.push(idx);
                        // Drain the EVENT queue (notifications), NOT the data queue.
                        while let Ok(Some(_event_id)) = listener.try_wait_one() {}
                    }
                    WaitSource::Fd(_fd) => {
                        // A device fd is WAKE-only and
                        // LEVEL-triggered. Record the index; NEVER read the fd —
                        // the external node's own tick drains its device, so the
                        // runtime must not consume device data. (Not draining a
                        // level-triggered fd is safe here because this is the
                        // `_once_` wait variant: the callback fires once per
                        // cycle and returns, so a still-readable fd cannot spin.)
                        fired_idx.push(idx);
                    }
                }
            }
            CallbackProgression::Continue
        };

        if let Err(e) = self
            .waitset
            .wait_and_process_once_with_timeout(on_event, timeout)
        {
            tracing::warn!(
                error = %e,
                "WaitSetReactor: wait_and_process_once_with_timeout failed; \
                 fired-set may be incomplete for this cycle"
            );
        }

        // Sort the recorded indices ascending and materialise the node ids in
        // DECLARATION order (independent of iceoryx2's internal callback
        // order). `sort_unstable` is alloc-free and the order of equal keys is
        // irrelevant — `attach_notification` yields one attachment per source,
        // so the indices are distinct.
        self.fired_idx.sort_unstable();
        // The two receivers are bound first so the `extend` fits on ONE line: the
        // lint's block-above rule arms the NEXT line, and rustfmt's split of the
        // longer form puts a bare `self.fired` there — which allocates nothing, so
        // the marker would arm a line the pattern set cannot match and would land
        // in the unmatched-annotation banner instead of covering the real call.
        let fired_idx = &self.fired_idx;
        let fired = &mut self.fired;
        // hot-path-alloc-ok: `fired` is CLEARED at the top of every `run_once` and
        // REUSED, pre-sized by `capacity_hint` in `WaitSetReactor::new` for exactly
        // this — so at steady state the extend records into already-reserved space
        // and allocates nothing. The fired set is bounded by the attachment count,
        // so it can grow at most once, when a cycle first exceeds the hint. The
        // `Arc::clone` inside is a refcount bump, not an allocation.
        fired.extend(fired_idx.iter().map(|&i| Arc::clone(&sources[i].1)));

        // `guards` drop here, detaching every source from the waitset.
        &self.fired
    }

    /// The most recent [`Self::run_once`]'s fired sources as
    /// INDICES into the `sources` slice, ascending — the same record
    /// [`Self::run_once`] maps to node ids, before that mapping.
    ///
    /// It exists so the public [`crate::wake::WakeSet`] can demux a fired set
    /// without inventing a second multiplexer: `WakeSet`'s callers key their
    /// sources positionally (a vizd tap set), not by a graph node id, and the
    /// node-id vector `run_once` returns would force them to round-trip through
    /// strings they never had. The indices are ALREADY sorted ascending by
    /// `run_once` (the declaration-order determinism contract), so this
    /// accessor inherits it rather than re-deriving it.
    pub(crate) fn fired_indices(&self) -> &[usize] {
        &self.fired_idx
    }

    /// Did the most recent [`Self::run_once`] actually block
    /// on the WaitSet (attach ≥1 source and call `wait_and_process_*`)?
    ///
    /// `false` if it short-circuited (no sources, or every `attach_notification`
    /// failed) and therefore returned WITHOUT consuming `timeout`.
    /// [`crate::graph::GraphRuntime::run_live`] uses this to sleep the heartbeat
    /// in that case instead of busy-spinning at 100% CPU.
    pub(crate) fn last_wait_blocked(&self) -> bool {
        self.last_wait_blocked
    }
}

#[cfg(test)]
mod tests {
    use super::{classify, FdVerdict};
    use std::os::unix::io::RawFd;

    /// Raise the `RLIMIT_NOFILE` soft limit to at least `at_least` (a no-op if it
    /// is already high enough) so a `dup2` onto a high target fd is legal. Loud on
    /// any libc failure — a silently-unraised limit would make the dup2 below fail,
    /// not the classify assertion, so surface the real cause here.
    fn raise_nofile_to(at_least: libc::rlim_t) {
        // SAFETY: `getrlimit`/`setrlimit` write/read a local `rlimit` we own.
        unsafe {
            let mut rl = libc::rlimit {
                rlim_cur: 0,
                rlim_max: 0,
            };
            assert_eq!(
                libc::getrlimit(libc::RLIMIT_NOFILE, &mut rl),
                0,
                "getrlimit(RLIMIT_NOFILE) must succeed"
            );
            if rl.rlim_cur < at_least {
                rl.rlim_cur = at_least.min(rl.rlim_max);
                assert_eq!(
                    libc::setrlimit(libc::RLIMIT_NOFILE, &rl),
                    0,
                    "setrlimit raising the NOFILE soft limit to {} must succeed",
                    rl.rlim_cur
                );
                assert!(
                    rl.rlim_cur >= at_least,
                    "the hard NOFILE limit ({}) is below the {at_least} this test needs",
                    rl.rlim_max
                );
            }
        }
    }

    /// A live pipe whose read end is duplicated onto a chosen high fd number; the
    /// original ends are closed on `Drop`. `high` is the dup target (must be free).
    struct HighFd {
        orig_read: RawFd,
        orig_write: RawFd,
        high: RawFd,
    }

    impl HighFd {
        fn new(high: RawFd) -> Self {
            raise_nofile_to((high as libc::rlim_t) + 16);
            let mut fds = [0 as libc::c_int; 2];
            // SAFETY: `fds` is a 2-element array; `pipe` writes exactly two fds.
            let rc = unsafe { libc::pipe(fds.as_mut_ptr()) };
            assert_eq!(rc, 0, "libc::pipe must succeed");
            // The high target must be free so dup2 does not clobber a live fd.
            // SAFETY: F_GETFD is a read-only liveness probe.
            assert_eq!(
                unsafe { libc::fcntl(high, libc::F_GETFD) },
                -1,
                "the high fd {high} must be free before dup2"
            );
            // SAFETY: `fds[0]` is a valid open fd; `high` is a free in-range target.
            let rc = unsafe { libc::dup2(fds[0], high) };
            assert_eq!(rc, high, "dup2 onto the high fd must return the target");
            HighFd {
                orig_read: fds[0],
                orig_write: fds[1],
                high,
            }
        }

        /// Close the high dup target out of band (models a dead high fd) and mark it
        /// relinquished so `Drop` does not double-close.
        fn close_high(&mut self) {
            if self.high >= 0 {
                // SAFETY: sole close of the dup target we created.
                unsafe { libc::close(self.high) };
                self.high = -1;
            }
        }
    }

    impl Drop for HighFd {
        fn drop(&mut self) {
            // SAFETY: each is an fd this HighFd opened and has not relinquished.
            unsafe {
                if self.high >= 0 {
                    libc::close(self.high);
                }
                libc::close(self.orig_read);
                libc::close(self.orig_write);
            }
        }
    }

    /// The classify verdict matrix — the three refusal causes `non_owning`
    /// flattens into one `None` must be distinguishable, and the poll-path flag
    /// must lift ONLY the `FD_SETSIZE` ceiling (never the negative/dead refusals).
    #[test]
    fn classify_splits_negative_above_limit_dead_and_usable() {
        // raw < 0 → Negative on EITHER flag (the negative guard is first).
        assert!(matches!(classify(-1, false), FdVerdict::Negative));
        assert!(matches!(classify(-1, true), FdVerdict::Negative));

        // A low, live fd is Usable on both paths (the common case). A plain pipe's
        // read end is a small fd number (well below FD_SETSIZE).
        let mut low_fds = [0 as libc::c_int; 2];
        // SAFETY: `low_fds` is a 2-element array; `pipe` writes exactly two fds.
        assert_eq!(
            unsafe { libc::pipe(low_fds.as_mut_ptr()) },
            0,
            "pipe for the low-fd case must succeed"
        );
        let low_live = low_fds[0];
        assert!(
            (low_live as usize) < libc::FD_SETSIZE,
            "a fresh pipe's read fd {low_live} must be below FD_SETSIZE"
        );
        assert!(matches!(classify(low_live, false), FdVerdict::Usable(_)));
        assert!(matches!(classify(low_live, true), FdVerdict::Usable(_)));
        // SAFETY: sole close of the two fds this pipe opened (the non-owning
        // `FdSource` from `classify` above never closed them).
        unsafe {
            libc::close(low_fds[0]);
            libc::close(low_fds[1]);
        }

        // A LIVE high fd: the select path (polled_not_selected == false) REFUSES it
        // as AboveSelectLimit; the poll path (true) FALLS THROUGH to the liveness
        // probe and finds it Usable — the key property.
        let high_alive = HighFd::new(libc::FD_SETSIZE as RawFd + 76);
        assert!(matches!(
            classify(high_alive.high, false),
            FdVerdict::AboveSelectLimit
        ));
        assert!(matches!(
            classify(high_alive.high, true),
            FdVerdict::Usable(_)
        ));

        // A DEAD high fd is refused on EVERY path — the poll path lifts the ceiling
        // but a closed fd is still Dead (never watch a POLLNVAL fd).
        let mut high_dead = HighFd::new(libc::FD_SETSIZE as RawFd + 108);
        let dead_num = high_dead.high;
        high_dead.close_high();
        assert!(matches!(classify(dead_num, true), FdVerdict::Dead));
    }
}
