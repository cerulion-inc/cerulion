// SPDX-License-Identifier: AGPL-3.0-only
//! Flood-suppression latch + counter for UNDELIVERED `SentSample` notifies.
//!
//! Model mirrors the sibling
//! [`OutputDiscardLatch`](super::output_discard_latch::OutputDiscardLatch)
//! and the pre-step-drain
//! [`DrainWarnLatch`](crate::graph::drain_latch::DrainWarnLatch): a
//! pure state machine so the suppression policy is unit-testable with no
//! transport, no SHM and no mock.
//!
//! # Why (the failure it makes observable)
//!
//! iceoryx2 delivers an event notification over a per-listener `AF_UNIX
//! SOCK_DGRAM` doorbell, and — because `notify_with_custom_event_id` passes
//! `skip_self_deliver = false` — a publisher's notify is delivered to EVERY
//! listener on the topic's event service, **including the publisher's own**
//! (each [`CerulionPublisher`](super::publisher::CerulionPublisher) owns one,
//! to hear `SubscriberConnected`).
//!
//! A listener that cannot be delivered to is a genuinely degraded wake path
//! (Principle #6 — the consumer stops being woken and falls back to the
//! heartbeat), and iceoryx2's own complaint about it is filtered out by the
//! log-level default (`init_iceoryx_log_level`), which is correct for the disk
//! and exactly why this latch exists: Cerulion must carry the signal ITSELF or
//! the condition is invisible (Principle #3).
//!
//! # Detection
//!
//! `Notifier::notify*` returns the number of listeners it actually triggered.
//! Comparing that against the topic event service's live listener count is an
//! exact, cheap test: `triggered < listeners` ⇒ at least one connection could
//! not be delivered to.
//!
//! **A stale registration** is the condition that produces it: the listener's
//! process died (SIGKILL, crash) without deregistering, so its doorbell has no
//! reader. The next notify's send is refused, iceoryx2 drops that connection
//! and does not count it, and the topic's dynamic-config entry is still there.
//! Remedy: nothing from the producer; a dead-node sweep removes the entry.
//! `notify_shortfall_iox2_test` drives exactly that and is where the numbers
//! below were measured.
//!
//! ## The condition this latch was BUILT for, and no longer sees
//!
//! Until iceoryx2 0.10 the event id rode IN the datagram, so a listener nobody
//! drained filled its socket and every later notify to it failed and was logged
//! once per publish — measured at ~2500 lines/s ≈ 5 MB/s on a robot, enough to
//! fill a root disk. 0.10 removed that at the source: the id and its repeat
//! count live in a shared-memory counting bitset, the doorbell carries one
//! byte, a full doorbell is SWALLOWED rather than refused, and a notify into a
//! listener that already holds an unconsumed wake skips the send entirely. A
//! live listener nobody drains is therefore reached forever and counted as
//! reached, and no drain anywhere exists to prevent it.
//!
//! ## And one shape it cannot see
//!
//! A listener killed while holding an UNCONSUMED wake sits in the notified
//! state, where a notify returns success without touching the doorbell. That
//! registration reads as reached for as long as it survives. A consumer that
//! was draining when it died leaves the state idle and IS seen, which is the
//! common shape; the blind one is a consumer that was already not draining.
//!
//! # Race windows (both directions, and what absorbs each)
//!
//! The `listeners` count and the notify are two separate operations, so a
//! listener can join or leave in between. WHICH direction is possible depends on
//! WHICH side of the notify the count was read — that is exactly what
//! [`ListenerCountTiming`] tells this latch:
//!
//! * [`ListenerCountTiming::BeforeNotify`] (the caller already had the count —
//!   the elision gate, the boundary resweep). A listener that
//!   ATTACHES in the window can only raise `triggered`, never fabricate a
//!   shortfall. A listener that DEREGISTERS in the window shows up once as
//!   `undelivered` — one `warn!`, one counter bump — and then the latch re-arms
//!   SILENTLY on the next healthy notify: a lone-warn regime leaves
//!   `suppressed == 0`, which is exactly the case [`NotifyDeliveryLatch::on_notify`]
//!   deliberately does NOT announce (the same rule as
//!   `OutputDiscardLatch::on_complete`), so NO `Recovered` follows and the
//!   counter simply stays at its bumped value. A lingering nonzero count after
//!   such a blip is the healthy resting state; growth is the signal.
//! * [`ListenerCountTiming::AfterNotify`] (no caller-supplied count, so this
//!   module's consumer reads it right after the notify — every elision-UNARMED
//!   publisher: raw/service/rmw publishers and the netd/gateway mirror
//!   re-inject publishers, whose consumers attach and detach constantly). Here
//!   the race is INVERTED: a listener that ATTACHES between the notify and the
//!   read raises `listeners` without raising `triggered` and looks exactly like
//!   an undelivered notify. That would be a loud `warn!` plus a permanent
//!   counter bump on a perfectly healthy graph, so this latch does NOT accept
//!   it: an `AfterNotify` shortfall observed while the latch is healthy only
//!   ARMS a suspicion (nothing logged, nothing counted) and forces the NEXT
//!   notify to be classified. The regime opens only if the shortfall REPEATS
//!   there. An attach race heals on that very next notify; a dead-but-unreaped
//!   listener does not.
//!
//! # Cost discipline (the notify is on the publish path)
//!
//! Reading `listeners` is a shared-memory container `len()` on the topic's
//! dynamic config — cheap, but NOT free, and `notify_sent_sample` runs on every
//! publish of every publisher, including ones the elision gate never
//! armed (which therefore never paid that read before). So the latch
//! exposes [`NotifyDeliveryLatch::needs_classification`]: a publisher whose
//! notify reached exactly as many listeners as the last CLASSIFIED one, while
//! the latch is healthy and holds no armed suspicion, is indistinguishable from
//! that last classification and can be skipped WITHOUT reading the listener
//! count. Steady state is then three relaxed loads of latch-local atomics.
//!
//! That gate is a COST gate and nothing else, so the caller must consult it ONLY
//! when the read it avoids would actually be a NEW read. A caller that already
//! holds the count (either `BeforeNotify` site) has nothing to save and must
//! classify unconditionally — skipping there would trade real detection for
//! zero saving, and would silence the resweep, the one site that observes a
//! quiescent producer's foreign listener at all.
//!
//! What detection costs on the `AfterNotify` path — TWO
//! distinct costs, one from the gate and one from the persistence rule:
//!
//! 1. **The cost gate.** A listener this publisher has NEVER once reached —
//!    i.e. one already unreachable at the moment it joined, while `triggered`
//!    happens to stay constant — is not classified until `triggered` next
//!    moves. The regime that matters (a listener that WAS reachable and then
//!    stops being, which is every observed case: `triggered` drops) is always
//!    classified, and the first notify of a publisher's life is always
//!    classified (the sentinel).
//! 2. **The persistence rule**, whose confirming notify is PUBLISH-CADENCE
//!    BOUND and therefore unbounded in time. An `AfterNotify` shortfall only
//!    arms a suspicion; it is confirmed (counted, warned) by the NEXT
//!    classified notify, which arrives when the producer next publishes. On a
//!    quiescent or one-shot producer — a netd/gateway mirror of `/tf_static`,
//!    a latched TRANSIENT_LOCAL rmw publisher, a service reply port —
//!    that notify may be far away or never come, and until it does
//!    `total_undelivered` reads 0 and nothing is logged even though a real
//!    shortfall was observed. The boundary resweep does NOT rescue
//!    this: it returns early unless notify elision is ARMED, and armed
//!    publishers are exactly the `BeforeNotify` population — so the
//!    `AfterNotify` population has no boundary observer at all. The trade is
//!    deliberate (the alternative is a `warn!` plus a permanent counter bump on
//!    every attach race of every mirror publisher), but it means a 0 on a
//!    low-cadence unarmed publisher reads "nothing CONFIRMED", not "nothing
//!    seen".
//!
//! # State machine
//!
//! ```text
//!               on_notify(triggered < listeners) -> Warn
//!   Healthy ───────────────────────────────────────────► Degraded (suppressed = 0)
//!      ▲                                                    │  on_notify(triggered < listeners)
//!      │ on_notify(triggered >= listeners) -> Recovered     │    -> Debug { suppressed += 1 }
//!      │   Some iff suppressed > 0, else None (silent)      │
//!      └────────────────────────────────────────────────────┘
//!
//!   Healthy + on_notify(delivered) -> None  (steady state: one relaxed load)
//! ```
//!
//! One latch per [`CerulionPublisher`](super::publisher::CerulionPublisher), so
//! there is no key lookup and no lock on the publish path. The healthy path is
//! a single relaxed atomic load plus a compare — the counters are `&self`
//! atomics (not `&mut` fields like the sibling latches) because
//! `notify_sent_sample` takes `&self`.
//!
//! Counts are diagnostics: under a hypothetical concurrent notify on one
//! publisher the `suppressed` tally can skew by a few, which never changes the
//! Warn/Debug/Recovered CLASSIFICATION and never affects data flow.

use core::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};

/// Sentinel for [`NotifyDeliveryLatch::last_classified`]: no notify has been
/// classified yet, so the next one MUST be (a publisher's very first notify can
/// itself be the failing one). `usize::MAX` is unreachable as a real listener
/// count.
const NEVER_CLASSIFIED: usize = usize::MAX;

/// WHEN the caller read the `listeners` count relative to the notify it is
/// reporting. Selects which race window [`NotifyDeliveryLatch::on_notify`] has
/// to defend against — see the module docs' race-window section.
///
/// A typed enum rather than a bool because the two arms are trivially
/// invertible and the inversion is SILENT: reporting an `AfterNotify` read as
/// `BeforeNotify` re-opens the false-positive `warn!` this type exists to
/// suppress, and the opposite mislabel delays a real detection by one notify.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ListenerCountTiming {
    /// The count was read BEFORE the notify (the caller already had it: the
    /// elision gate, the boundary resweep). An attaching
    /// listener can only raise `triggered`, so a shortfall is classified
    /// immediately.
    BeforeNotify,
    /// The count was read AFTER the notify (no caller-supplied count). An
    /// attaching listener can fabricate a one-observation shortfall, so a
    /// shortfall must PERSIST across two classified notifies before it opens a
    /// regime.
    AfterNotify,
}

/// What the caller should log for one notify, as decided by
/// [`NotifyDeliveryLatch::on_notify`].
///
/// A typed enum (rather than a bool) so call sites read as
/// `NotifyDeliveryAction::Warn { .. }` instead of an easily-inverted flag — the
/// same inversion-regression surface the sibling
/// [`DiscardLogLevel`](super::output_discard_latch::DiscardLogLevel) exists to
/// pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NotifyDeliveryAction {
    /// Nothing to log: either the notify reached every listener and the latch
    /// was already healthy, or it closed a regime that suppressed nothing.
    None,
    /// First undelivered notify of a regime — log at `tracing::warn!`.
    Warn {
        /// Listeners this notify could NOT be delivered to.
        undelivered: usize,
    },
    /// A repeat while the latch is set — log at `tracing::debug!`, carrying the
    /// number of undelivered notifies DOWNGRADED so far in this regime (the
    /// loud first one is not counted; it was not suppressed).
    Debug {
        /// Listeners this notify could NOT be delivered to.
        undelivered: usize,
        /// Undelivered notifies downgraded to `debug!` since the regime opened.
        suppressed: u64,
    },
    /// A fully-delivered notify closed a regime that had suppressed at least
    /// one repeat — log ONCE at `tracing::info!`, then the latch is re-armed.
    Recovered {
        /// Undelivered notifies that were downgraded during the closed regime.
        suppressed: u64,
    },
}

/// Pure flood-suppression state machine for undelivered `SentSample` notifies.
/// One per [`CerulionPublisher`](super::publisher::CerulionPublisher).
///
/// Holds no transport state and emits no logs itself — the caller maps the
/// returned decision onto `tracing` events (see
/// [`CerulionPublisher::notify_sent_sample`](super::publisher::CerulionPublisher::notify_sent_sample)).
#[derive(Debug)]
pub struct NotifyDeliveryLatch {
    /// True while in the Degraded state (≥1 undelivered notify since the last
    /// fully-delivered one).
    degraded: AtomicBool,
    /// Undelivered notifies downgraded to `debug!` since the regime opened. The
    /// FIRST (loud `warn!`) one is NOT counted. Reset at each regime open and
    /// at recovery.
    suppressed: AtomicU64,
    /// Running total of listener-notifies that could not be delivered, across
    /// ALL regimes — bumped on EVERY undelivered notify the latch CONFIRMS,
    /// regardless of log-level regime, NEVER reset on recovery. This is the
    /// Principle #3 queryability signal: with iceoryx2's own `warn!` correctly
    /// filtered at `IOX2_LOG_LEVEL=error` and sustained repeats
    /// `debug!`-downgraded here, a persistently unreachable listener would
    /// otherwise be completely invisible. Surfaced through
    /// [`CerulionPublisher::notify_undelivered_count`](super::publisher::CerulionPublisher::notify_undelivered_count).
    ///
    /// "Confirmed" is the ONE deliberate exclusion, and it is what keeps the
    /// counter's headline promise (zero on a healthy graph) true: the FIRST
    /// [`ListenerCountTiming::AfterNotify`] shortfall of a regime is a
    /// suspicion, not a fact — it is equally explained by a listener attaching
    /// in the notify→read window — so it is counted only when the next
    /// classified notify reproduces it (see [`Self::pending_shortfall`]). A
    /// genuine regime therefore counts every undelivered notify from its
    /// SECOND observation on; an attach race counts none.
    total_undelivered: AtomicU64,
    /// True while an [`ListenerCountTiming::AfterNotify`] shortfall has been
    /// observed on a HEALTHY latch but not yet confirmed by a second classified
    /// notify. Purely a suspicion: nothing has been logged or counted for it.
    ///
    /// It also forces [`Self::needs_classification`] — without that, a
    /// persistent shortfall whose `triggered` never moves again (an unreachable
    /// listener at a steady publish rate) would be skipped by the cost gate
    /// forever and the suspicion would never be confirmed.
    pending_shortfall: AtomicBool,
    /// The `triggered` count of the last notify [`Self::on_notify`] actually
    /// classified, or the private `NEVER_CLASSIFIED` sentinel before the first
    /// one. Drives
    /// [`Self::needs_classification`] — see the module docs' cost-discipline
    /// section for why the caller is allowed to skip the listener-count read.
    last_classified: AtomicUsize,
}

/// Hand-written (NOT derived) so `Default` is byte-identical to
/// [`NotifyDeliveryLatch::new`]: `last_classified` starts at the private
/// `NEVER_CLASSIFIED` sentinel, not at `0`. A derived `Default` would start it
/// at `0`, which is a REAL triggered count — a publisher whose first notify
/// reaches zero listeners while it has one would then be skipped as "already
/// classified healthy", exactly the undelivered-notify condition going unseen.
impl Default for NotifyDeliveryLatch {
    fn default() -> Self {
        Self::new()
    }
}

impl NotifyDeliveryLatch {
    /// Construct a latch in the Healthy state.
    pub const fn new() -> Self {
        Self {
            degraded: AtomicBool::new(false),
            suppressed: AtomicU64::new(0),
            total_undelivered: AtomicU64::new(0),
            pending_shortfall: AtomicBool::new(false),
            last_classified: AtomicUsize::new(NEVER_CLASSIFIED),
        }
    }

    /// Should the caller READ the topic's live listener count and call
    /// [`Self::on_notify`] for this notify?
    ///
    /// **Only ask when the read would be a NEW one.** This is a COST gate: a
    /// caller that already holds the count (see
    /// [`ListenerCountTiming::BeforeNotify`]) saves nothing by skipping and
    /// must classify unconditionally.
    ///
    /// `false` — the skip — only when ALL of these hold:
    ///
    /// * the latch is healthy (a degraded regime must keep classifying, so
    ///   [`Self::total_undelivered`] counts every undelivered notify of the
    ///   regime and the recovery edge is not missed),
    /// * no [`ListenerCountTiming::AfterNotify`] shortfall is awaiting
    ///   confirmation (the suspicion must be resolved, and a persistent
    ///   shortfall's `triggered` typically does NOT move again), and
    /// * `triggered` equals the last classified notify's `triggered`, so this
    ///   notify's outcome is indistinguishable from one already classified healthy.
    ///
    /// Three relaxed loads, no store, no alloc.
    #[inline]
    #[must_use]
    pub fn needs_classification(&self, triggered: usize) -> bool {
        self.degraded.load(Ordering::Relaxed)
            || self.pending_shortfall.load(Ordering::Relaxed)
            || self.last_classified.load(Ordering::Relaxed) != triggered
    }

    /// Record the outcome of one notify and return what the caller should log.
    ///
    /// `triggered` is the listener count `Notifier::notify*` reported;
    /// `listeners` is the topic event service's live listener count, and
    /// `timing` says whether the caller read it before or after the notify (see
    /// [`ListenerCountTiming`] — it selects which race window is defended
    /// against). `triggered >= listeners` is the healthy case.
    ///
    /// Steady state is one relaxed load and a compare — no store, no alloc, no
    /// lock. Callers that would have to READ `listeners` gate this on
    /// [`Self::needs_classification`] so the read itself is only paid when it
    /// can change the verdict; callers that already hold the count call this
    /// unconditionally.
    #[inline]
    pub fn on_notify(
        &self,
        triggered: usize,
        listeners: usize,
        timing: ListenerCountTiming,
    ) -> NotifyDeliveryAction {
        // Remember what we classified, so an identical subsequent notify can skip
        // the listener-count read entirely (see `needs_classification`).
        self.last_classified.store(triggered, Ordering::Relaxed);
        if triggered >= listeners {
            // Healthy. Any armed suspicion was an artifact — the listener that
            // inflated the previous count is reachable now (or gone), which is
            // exactly how an attach race resolves. Drop it.
            // Load-before-store so the steady state never writes.
            if self.pending_shortfall.load(Ordering::Relaxed) {
                self.pending_shortfall.store(false, Ordering::Relaxed);
            }
            if self.degraded.load(Ordering::Relaxed) {
                self.degraded.store(false, Ordering::Relaxed);
                let suppressed = self.suppressed.swap(0, Ordering::Relaxed);
                // A lone-warn regime (one `warn!`, immediate recovery) re-arms
                // SILENTLY: the single warning already told the whole story, and
                // announcing recovery there would double the log volume of a
                // publisher flapping every other publish. Mirrors
                // `OutputDiscardLatch::on_complete`.
                if suppressed > 0 {
                    return NotifyDeliveryAction::Recovered { suppressed };
                }
            }
            return NotifyDeliveryAction::None;
        }

        // A shortfall. On the `AfterNotify` path the count was read AFTER the
        // notify, so a listener that ATTACHED in that window inflates
        // `listeners` without raising `triggered` and is indistinguishable from
        // a genuinely undelivered notify AT THIS OBSERVATION. Require
        // PERSISTENCE before believing it: the first such observation on a
        // healthy latch only arms the suspicion — nothing logged, nothing
        // counted — and `needs_classification` then forces the next notify to
        // be classified. A dead-but-unreaped listener reproduces
        // the shortfall there and opens the regime one notify later; an attach
        // race resolves healthy and costs nothing. Inside an OPEN regime every
        // repeat is believed (the condition is already established), and a
        // `BeforeNotify` count cannot be inflated by an attach at all.
        if timing == ListenerCountTiming::AfterNotify
            && !self.degraded.load(Ordering::Relaxed)
            && !self.pending_shortfall.swap(true, Ordering::Relaxed)
        {
            return NotifyDeliveryAction::None;
        }
        self.pending_shortfall.store(false, Ordering::Relaxed);

        let undelivered = listeners - triggered;
        // Every CONFIRMED undelivered notify bumps the queryable total,
        // independent of the log-level regime (Principle #3). Cold path only.
        self.total_undelivered
            .fetch_add(undelivered as u64, Ordering::Relaxed);
        if self.degraded.load(Ordering::Relaxed) {
            let suppressed = self.suppressed.fetch_add(1, Ordering::Relaxed) + 1;
            NotifyDeliveryAction::Debug {
                undelivered,
                suppressed,
            }
        } else {
            self.degraded.store(true, Ordering::Relaxed);
            self.suppressed.store(0, Ordering::Relaxed);
            NotifyDeliveryAction::Warn { undelivered }
        }
    }

    /// Running total of CONFIRMED undelivered listener-notifies on this
    /// publisher, across all regimes (never reset on recovery). See the
    /// `total_undelivered` field docs for what "confirmed" excludes.
    pub fn total_undelivered(&self) -> u64 {
        self.total_undelivered.load(Ordering::Relaxed)
    }

    /// Returns `true` while the latch is in the Degraded state.
    ///
    /// A test-oracle accessor for this pure state machine; production drives the
    /// latch purely through [`on_notify`](Self::on_notify).
    pub fn is_degraded(&self) -> bool {
        self.degraded.load(Ordering::Relaxed)
    }

    /// Returns `true` while an [`ListenerCountTiming::AfterNotify`] shortfall is
    /// armed but unconfirmed (nothing logged, nothing counted for it yet).
    ///
    /// A test-oracle accessor; production observes the same state only through
    /// [`Self::needs_classification`], which it forces.
    pub fn has_pending_shortfall(&self) -> bool {
        self.pending_shortfall.load(Ordering::Relaxed)
    }
}

// ---------------------------------------------------------------------------
// This module's half of the ABI LAYOUT PIN (see `crate::abi_layout`).
//
// `abi_pin_struct!` expands to an exhaustive destructuring pattern with no `..`
// rest pattern, so adding or removing a field of one of these structs is a
// COMPILE ERROR naming the struct and the field; it also measures
// size/align/`offset_of!`, which `crate::abi_layout` compares against the
// snapshot table keyed to `CERULION_ABI_VERSION`. `abi_pin_enum!` does the
// same for a variant set (an enum carries no stable field offsets).
// ---------------------------------------------------------------------------
#[cfg(test)]
pub(crate) fn abi_layout_pins() -> Vec<crate::abi_layout::MeasuredStruct> {
    use crate::abi_layout::abi_pin_struct;
    vec![abi_pin_struct!(NotifyDeliveryLatch {
        degraded,
        suppressed,
        total_undelivered,
        pending_shortfall,
        last_classified
    })]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Shorthand for the armed/resweep timing (a count read BEFORE the notify),
    /// where a shortfall is believed at its first observation.
    const BEFORE: ListenerCountTiming = ListenerCountTiming::BeforeNotify;
    /// Shorthand for the unarmed timing (a count read AFTER the notify), where a
    /// shortfall must persist across two classified notifies.
    const AFTER: ListenerCountTiming = ListenerCountTiming::AfterNotify;

    /// The canonical regime: warn → debug(1) → debug(2) → recovery(2) → warn.
    /// Hand-written oracle, never a self-compare.
    #[test]
    fn canonical_regime_cycle() {
        let latch = NotifyDeliveryLatch::new();
        // Steady healthy state logs nothing and stays armed.
        assert_eq!(latch.on_notify(2, 2, BEFORE), NotifyDeliveryAction::None);
        assert!(!latch.is_degraded());
        assert_eq!(latch.total_undelivered(), 0);

        // Regime opens loudly.
        assert_eq!(
            latch.on_notify(1, 2, BEFORE),
            NotifyDeliveryAction::Warn { undelivered: 1 }
        );
        assert!(latch.is_degraded());
        assert_eq!(latch.total_undelivered(), 1);

        // Repeats are downgraded with a running suppressed count.
        assert_eq!(
            latch.on_notify(1, 2, BEFORE),
            NotifyDeliveryAction::Debug {
                undelivered: 1,
                suppressed: 1
            }
        );
        assert_eq!(
            latch.on_notify(0, 2, BEFORE),
            NotifyDeliveryAction::Debug {
                undelivered: 2,
                suppressed: 2
            }
        );
        // Total counts LISTENERS missed, not calls: 1 + 1 + 2.
        assert_eq!(latch.total_undelivered(), 4);

        // Recovery reports once, then re-arms.
        assert_eq!(
            latch.on_notify(2, 2, BEFORE),
            NotifyDeliveryAction::Recovered { suppressed: 2 }
        );
        assert!(!latch.is_degraded());
        // The total is NEVER reset on recovery (Principle #3 queryability).
        assert_eq!(latch.total_undelivered(), 4);

        // Re-armed: the next failure is loud again.
        assert_eq!(
            latch.on_notify(0, 1, BEFORE),
            NotifyDeliveryAction::Warn { undelivered: 1 }
        );
        assert_eq!(latch.total_undelivered(), 5);
    }

    /// A lone-warn regime re-arms SILENTLY (no recovery `info!`) — the
    /// every-other-publish flapper must not double its log volume.
    #[test]
    fn lone_warn_regime_rearms_silently() {
        let latch = NotifyDeliveryLatch::new();
        assert_eq!(
            latch.on_notify(0, 1, BEFORE),
            NotifyDeliveryAction::Warn { undelivered: 1 }
        );
        assert_eq!(latch.on_notify(1, 1, BEFORE), NotifyDeliveryAction::None);
        assert!(!latch.is_degraded());
        // Re-armed regardless: loud again.
        assert_eq!(
            latch.on_notify(0, 1, BEFORE),
            NotifyDeliveryAction::Warn { undelivered: 1 }
        );
        assert_eq!(latch.total_undelivered(), 2);
    }

    /// Inversion regressions: an always-warn latch and a never-warn latch both
    /// fail this. 1000 undelivered notifies ⇒ EXACTLY one `Warn`, 999 `Debug`.
    #[test]
    fn sustained_regime_is_one_warn_and_n_minus_one_debug() {
        let latch = NotifyDeliveryLatch::new();
        let mut warns = 0_u32;
        let mut debugs = 0_u32;
        for _ in 0..1000 {
            match latch.on_notify(0, 1, BEFORE) {
                NotifyDeliveryAction::Warn { .. } => warns += 1,
                NotifyDeliveryAction::Debug { .. } => debugs += 1,
                other => panic!("unexpected action in a sustained regime: {other:?}"),
            }
        }
        assert_eq!(warns, 1, "exactly one loud head per regime");
        assert_eq!(debugs, 999);
        assert_eq!(latch.total_undelivered(), 1000);
    }

    /// `triggered > listeners` (a listener attached between the count read and
    /// the notify) is HEALTHY, not a spurious recovery/warn — on BOTH timings.
    #[test]
    fn more_triggered_than_counted_is_healthy() {
        for timing in [BEFORE, AFTER] {
            let latch = NotifyDeliveryLatch::new();
            assert_eq!(latch.on_notify(3, 2, timing), NotifyDeliveryAction::None);
            assert!(!latch.is_degraded());
            assert!(!latch.has_pending_shortfall());
            assert_eq!(latch.total_undelivered(), 0);
        }
    }

    /// A publisher with ZERO listeners is the notify-elision steady state (and
    /// the quiescent-topic case): `0 >= 0` ⇒ healthy, nothing logged, nothing
    /// counted, on BOTH timings.
    #[test]
    fn zero_listeners_is_healthy() {
        for timing in [BEFORE, AFTER] {
            let latch = NotifyDeliveryLatch::new();
            for _ in 0..10 {
                assert_eq!(latch.on_notify(0, 0, timing), NotifyDeliveryAction::None);
            }
            assert!(!latch.is_degraded());
            assert!(!latch.has_pending_shortfall());
            assert_eq!(latch.total_undelivered(), 0);
        }
    }

    /// `Default` must equal `new()` — a Healthy, zeroed latch that has
    /// classified nothing (so its FIRST notify is always classified) and holds
    /// no suspicion.
    #[test]
    fn default_matches_new() {
        let latch = NotifyDeliveryLatch::default();
        assert!(!latch.is_degraded());
        assert!(!latch.has_pending_shortfall());
        assert_eq!(latch.total_undelivered(), 0);
        assert!(
            latch.needs_classification(0),
            "a latch that has classified nothing must classify its first notify"
        );
        assert_eq!(latch.on_notify(1, 1, BEFORE), NotifyDeliveryAction::None);
    }

    /// The cost gate ([`NotifyDeliveryLatch::needs_classification`]) — hand
    /// oracle over the exact sequence a steady healthy publisher produces, then
    /// a shortfall, then a recovery.
    ///
    /// A `false` here means the caller skips reading the topic's listener count
    /// from shared memory, so this is the pin that the skip only ever happens on
    /// a repeat of an already-healthy verdict.
    #[test]
    fn needs_classification_skips_only_repeats_of_a_healthy_verdict() {
        let latch = NotifyDeliveryLatch::new();

        // 1. Nothing classified yet ⇒ classify, whatever the count.
        assert!(latch.needs_classification(0));
        assert!(latch.needs_classification(7));

        // 2. First classification (healthy, 1 of 1).
        assert_eq!(latch.on_notify(1, 1, BEFORE), NotifyDeliveryAction::None);
        // A repeat of the SAME triggered count is indistinguishable ⇒ skip.
        assert!(!latch.needs_classification(1));
        // A DIFFERENT count is a change ⇒ classify (either direction).
        assert!(latch.needs_classification(0), "a drop must be classified");
        assert!(latch.needs_classification(2), "a rise must be classified");

        // 3. Saturation: triggered drops, so the gate lets it through and the
        //    regime opens.
        assert_eq!(
            latch.on_notify(0, 1, BEFORE),
            NotifyDeliveryAction::Warn { undelivered: 1 }
        );
        // While DEGRADED every notify is classified, even an identical repeat —
        // otherwise the total would stop counting and the recovery edge could
        // be missed.
        assert!(
            latch.needs_classification(0),
            "a degraded latch must classify every notify, including repeats"
        );
        assert_eq!(
            latch.on_notify(0, 1, BEFORE),
            NotifyDeliveryAction::Debug {
                undelivered: 1,
                suppressed: 1
            }
        );

        // 4. Recovery re-arms the skip: the healed verdict is remembered, so a
        //    steady healthy publisher goes back to paying nothing.
        assert_eq!(
            latch.on_notify(1, 1, BEFORE),
            NotifyDeliveryAction::Recovered { suppressed: 1 }
        );
        assert!(!latch.needs_classification(1));
        assert_eq!(latch.total_undelivered(), 2);
    }

    /// The attach-race absorber: on the `AfterNotify`
    /// path the listener count is read AFTER the notify, so a listener that
    /// ATTACHES in that window inflates `listeners` without raising `triggered`
    /// and looks exactly like an undelivered notify. A SINGLE such observation
    /// must produce NO log and NO count — the very next notify (where the new
    /// listener is reachable) resolves it healthy.
    ///
    /// Hand oracle: the netd/gateway mirror shape — one own
    /// listener, vizd attaches (triggered 2 of a then-3 count), then everything
    /// is reachable. Total must be 0: nothing was ever undelivered.
    #[test]
    fn after_notify_single_shortfall_is_absorbed_not_counted() {
        let latch = NotifyDeliveryLatch::new();

        // The apparent shortfall (2 reached, 3 counted after the attach).
        assert_eq!(
            latch.on_notify(2, 3, AFTER),
            NotifyDeliveryAction::None,
            "a first AfterNotify shortfall is a suspicion, not a verdict — no warn"
        );
        assert!(
            !latch.is_degraded(),
            "no regime may open on one observation"
        );
        assert!(
            latch.has_pending_shortfall(),
            "the suspicion must be armed so the next notify is classified"
        );
        assert_eq!(
            latch.total_undelivered(),
            0,
            "a healthy graph's attach race must never bump the operator counter"
        );

        // Next notify: the attached listener is reachable ⇒ resolved healthy,
        // silently (no Recovered — no regime was ever opened).
        assert_eq!(latch.on_notify(3, 3, AFTER), NotifyDeliveryAction::None);
        assert!(!latch.has_pending_shortfall(), "the suspicion is dropped");
        assert!(!latch.is_degraded());
        assert_eq!(latch.total_undelivered(), 0);
    }

    /// The other half of the persistence rule: a shortfall that REPEATS on the
    /// next classified notify is real (an unreachable listener, or a dead one whose
    /// registration lingers) and opens the regime loudly at that second
    /// observation.
    #[test]
    fn after_notify_persistent_shortfall_warns_and_counts() {
        let latch = NotifyDeliveryLatch::new();

        assert_eq!(latch.on_notify(1, 2, AFTER), NotifyDeliveryAction::None);
        assert!(latch.has_pending_shortfall());

        assert_eq!(
            latch.on_notify(1, 2, AFTER),
            NotifyDeliveryAction::Warn { undelivered: 1 },
            "a shortfall that survives a second classified notify is real"
        );
        assert!(latch.is_degraded());
        assert!(
            !latch.has_pending_shortfall(),
            "confirming the suspicion consumes it"
        );
        assert_eq!(latch.total_undelivered(), 1);

        // From here the regime behaves exactly like any other: repeats are
        // downgraded and counted unconditionally.
        assert_eq!(
            latch.on_notify(1, 2, AFTER),
            NotifyDeliveryAction::Debug {
                undelivered: 1,
                suppressed: 1
            }
        );
        assert_eq!(latch.total_undelivered(), 2);
    }

    /// The pin for the `pending_shortfall` term in
    /// [`NotifyDeliveryLatch::needs_classification`]: without it, an unreachable
    /// listener at a steady publish rate is invisible FOREVER — the suspicion is
    /// armed, `triggered` never moves again, the cost gate skips every
    /// subsequent notify, and the suspicion is never confirmed.
    #[test]
    fn pending_shortfall_forces_classification_of_an_identical_repeat() {
        let latch = NotifyDeliveryLatch::new();
        assert_eq!(latch.on_notify(1, 2, AFTER), NotifyDeliveryAction::None);

        // Healthy latch, and `triggered` is EXACTLY the last classified value —
        // the two conditions that normally authorise the skip.
        assert!(!latch.is_degraded());
        assert!(
            latch.needs_classification(1),
            "an armed suspicion must force classification even on an identical, \
             not-degraded repeat — otherwise it can never be confirmed"
        );
    }

    /// The armed / resweep path is UNCHANGED by the persistence rule: a
    /// `BeforeNotify` count cannot be inflated by an attaching listener, so its
    /// first shortfall is believed immediately.
    #[test]
    fn before_notify_shortfall_is_believed_immediately() {
        let latch = NotifyDeliveryLatch::new();
        assert_eq!(
            latch.on_notify(1, 2, BEFORE),
            NotifyDeliveryAction::Warn { undelivered: 1 }
        );
        assert!(latch.is_degraded());
        assert!(!latch.has_pending_shortfall());
        assert_eq!(latch.total_undelivered(), 1);
    }

    /// The persistence rule applies ONLY at regime OPEN. Once a regime is open
    /// the condition is established, so an `AfterNotify` repeat is believed at
    /// once — deferring inside a regime would under-count the very signal the
    /// operator is watching.
    #[test]
    fn after_notify_repeats_inside_an_open_regime_are_not_deferred() {
        let latch = NotifyDeliveryLatch::new();
        // Open the regime on the armed path (immediate), then switch timings.
        assert_eq!(
            latch.on_notify(0, 1, BEFORE),
            NotifyDeliveryAction::Warn { undelivered: 1 }
        );
        assert_eq!(
            latch.on_notify(0, 1, AFTER),
            NotifyDeliveryAction::Debug {
                undelivered: 1,
                suppressed: 1
            },
            "inside an open regime an AfterNotify shortfall is counted at once"
        );
        assert_eq!(latch.total_undelivered(), 2);
        assert!(!latch.has_pending_shortfall());
    }

    /// A publisher whose consumers attach and detach constantly (the netd mirror
    /// shape) can produce the one-observation shortfall over and over. It must
    /// stay silent and uncounted every single time — the false-positive class
    /// must not accumulate.
    #[test]
    fn repeated_attach_races_never_open_a_regime_or_count() {
        let latch = NotifyDeliveryLatch::new();
        for _ in 0..50 {
            assert_eq!(latch.on_notify(1, 2, AFTER), NotifyDeliveryAction::None);
            assert_eq!(latch.on_notify(2, 2, AFTER), NotifyDeliveryAction::None);
        }
        assert!(!latch.is_degraded());
        assert!(!latch.has_pending_shortfall());
        assert_eq!(
            latch.total_undelivered(),
            0,
            "50 attach races on a healthy publisher must leave the operator \
             counter at exactly zero"
        );
    }

    /// The skip gate must not change WHAT a driven sequence classifies: driving
    /// the same stimulus through the gate yields exactly the actions a
    /// gate-less drive does for the classified notifies, and the total is
    /// identical.
    ///
    /// Hand oracle for the `AfterNotify` (elision-unarmed) path: 3 healthy
    /// (1 of 1) → 4 unreachable (0 of 1) → 3 healthy again. The FIRST unreachable
    /// observation is deferred by the persistence rule, so the regime opens on
    /// the second and the total is 3 — one less than the four shortfalls
    /// observed.
    #[test]
    fn gated_after_notify_drive_matches_ungated_totals_and_actions() {
        let stimulus: &[(usize, usize)] = &[
            (1, 1),
            (1, 1),
            (1, 1),
            (0, 1),
            (0, 1),
            (0, 1),
            (0, 1),
            (1, 1),
            (1, 1),
            (1, 1),
        ];

        let gated = NotifyDeliveryLatch::new();
        // hot-path-alloc-ok: `#[cfg(test)]` oracle vector — the lint
        // sweeps all of transport/src including in-module test mods.
        let mut gated_actions = Vec::new();
        let mut skipped = 0_usize;
        for (triggered, listeners) in stimulus {
            if gated.needs_classification(*triggered) {
                gated_actions.push(gated.on_notify(*triggered, *listeners, AFTER));
            } else {
                skipped += 1;
            }
        }

        let ungated = NotifyDeliveryLatch::new();
        let ungated_actions: Vec<_> = stimulus
            .iter()
            .map(|(t, l)| ungated.on_notify(*t, *l, AFTER))
            .filter(|a| !matches!(a, NotifyDeliveryAction::None))
            .collect();

        // HAND ORACLE, not a self-compare: the exact logged sequence.
        // hot-path-alloc-ok: `#[cfg(test)]` oracle vector — the lint
        // sweeps all of transport/src including in-module test mods.
        let oracle = vec![
            NotifyDeliveryAction::Warn { undelivered: 1 },
            NotifyDeliveryAction::Debug {
                undelivered: 1,
                suppressed: 1,
            },
            NotifyDeliveryAction::Debug {
                undelivered: 1,
                suppressed: 2,
            },
            NotifyDeliveryAction::Recovered { suppressed: 2 },
        ];
        let gated_logged: Vec<_> = gated_actions
            .iter()
            .filter(|a| !matches!(a, NotifyDeliveryAction::None))
            .cloned()
            .collect();
        assert_eq!(
            gated_logged, oracle,
            "gated drive must match the hand oracle"
        );
        assert_eq!(
            ungated_actions, oracle,
            "the cost gate must not change any LOGGED verdict"
        );
        // 4 shortfalls observed, the first deferred as an attach-race suspicion.
        assert_eq!(gated.total_undelivered(), 3);
        assert_eq!(ungated.total_undelivered(), 3);
        // And it really skipped work (anti-tautology: 2 healthy repeats in the
        // opening run + 2 in the closing run). The deferred observation is NOT
        // among them — a pending suspicion forces classification.
        assert_eq!(skipped, 4, "the gate must actually skip healthy repeats");
    }

    /// The SAME stimulus on the armed / resweep (`BeforeNotify`) path, where the
    /// caller already holds the count: every shortfall is believed at once, so
    /// the regime opens one notify earlier and the total is the full 4.
    ///
    /// The contrast with the `AfterNotify` oracle above IS the persistence rule,
    /// stated as two hand-written numbers (4 vs 3) rather than a self-compare.
    #[test]
    fn before_notify_drive_counts_every_shortfall_from_the_first() {
        let stimulus: &[(usize, usize)] = &[
            (1, 1),
            (1, 1),
            (1, 1),
            (0, 1),
            (0, 1),
            (0, 1),
            (0, 1),
            (1, 1),
            (1, 1),
            (1, 1),
        ];
        let latch = NotifyDeliveryLatch::new();
        // hot-path-alloc-ok: `#[cfg(test)]` oracle vector — the lint
        // sweeps all of transport/src including in-module test mods.
        let logged: Vec<_> = stimulus
            .iter()
            .map(|(t, l)| latch.on_notify(*t, *l, BEFORE))
            .filter(|a| !matches!(a, NotifyDeliveryAction::None))
            .collect();
        assert_eq!(
            logged,
            // hot-path-alloc-ok: `#[cfg(test)]` oracle vector — the lint
            // sweeps all of transport/src including in-module test mods.
            vec![
                NotifyDeliveryAction::Warn { undelivered: 1 },
                NotifyDeliveryAction::Debug {
                    undelivered: 1,
                    suppressed: 1,
                },
                NotifyDeliveryAction::Debug {
                    undelivered: 1,
                    suppressed: 2,
                },
                NotifyDeliveryAction::Debug {
                    undelivered: 1,
                    suppressed: 3,
                },
                NotifyDeliveryAction::Recovered { suppressed: 3 },
            ]
        );
        assert_eq!(latch.total_undelivered(), 4);
    }
}
