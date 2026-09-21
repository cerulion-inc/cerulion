// SPDX-License-Identifier: AGPL-3.0-only
//! Runtime tap manager for the dynamic viz daemon.
//!
//! [`TapManager`] is the TRANSPORT-backed half of the daemon's tap plane: it
//! owns a set of listener-less [`DataOnlySubscriber`] taps it can add/remove at
//! runtime and drains them into per-tick [`InputFrames`] batches for the
//! [`VizLogWorker`](crate::worker::VizLogWorker).
//!
//! It decides WHAT the tap set is AND does it: `self.taps` (a `BTreeMap` keyed by
//! topic) IS the set, so attach is idempotent and `list` is ordered by
//! construction. There is no separate pure `TapSet` state machine for
//! the deciding half: nothing consumed one, so this type is not a
//! "concrete counterpart" to anything.
//! The semantics that matter are pinned over real iceoryx2 in
//! `tests/tap_manager_test.rs` (`attach_twice_opens_exactly_one_tap`,
//! `detach_frees_the_introspection_slot`), not in a unit test of a parallel
//! type.
//!
//! # Why a data-only tap (the observability invariant)
//!
//! [`TransportManager::create_data_only_subscriber`] opens ONLY the topic's
//! `{topic}/data` service (never its event service), so the tap registers NO
//! event listener and is structurally invisible to the producer's notifier loop
//! — zero perturbation to graph timing/determinism. This is the
//! already-sanctioned OBSERVER class (the `bagd` recorder + the network gateway
//! use it), NOT a second graph-plane consumer read path (it does NOT violate the
//! no-second-read-path decision: an observer tap never reads through the
//! graph's queue).
//!
//! # Semantics
//!
//! - **Attach** is idempotent: attaching an already-tapped topic opens NO second
//!   tap ([`AttachOutcome::AlreadyAttached`]) — the introspection-slot budget is
//!   scarce (only [`INTROSPECTION_SUBSCRIBER_HEADROOM`] taps per topic, shared
//!   with `bagd`/`topic echo`/`hz`).
//! - **Attach errors are per-topic and NON-FATAL**: a missing topic surfaces the
//!   "topic does not exist" error; a slot-exhausted topic surfaces the exact
//!   "no free introspection slot" error — neither panics, and neither leaves a
//!   phantom entry (the tap set is unchanged on failure).
//! - **No history**: a data-only tap has no late-joiner history, so frames
//!   published BEFORE attach are dropped — attach, THEN the producer publishes.
//! - **Detach** drops the tap, releasing its introspection slot; a subsequent
//!   [`attach`](TapManager::attach) opens a FRESH tap (so a re-created producer
//!   service is reachable again).
//! - **Poll** drains each tap DRAIN-TO-LIVE (no daemon-side history): it copies
//!   each frame out of SHM and releases the borrow immediately, so a poll never
//!   pins more than one borrow-budget's worth of slots. A per-tap drain error is
//!   non-fatal: the partial fill is still forwarded and the tap stays attached
//!   (the next poll retries), and the error is FLOOD-LATCHED per tap (the house
//!   [`FieldsWarnLatch`] contract, matching worker.rs's reconnect/panic latches):
//!   the first error of a regime `warn!`s, sustained errors downgrade to `debug!`
//!   carrying a running suppressed count, and the first successful drain after a
//!   failing regime emits one recovery `info!`. So a persistent receive failure
//!   (a crashed producer, an iceoryx2 `ConnectionFailure`) at the ~60 Hz poll
//!   rate never storms stderr — the run-killer class.
//!
//! [`FieldsWarnLatch`]: crate::pointcloud::FieldsWarnLatch
//!
//! [`INTROSPECTION_SUBSCRIBER_HEADROOM`]: cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM

use std::collections::BTreeMap;
use std::sync::Arc;

use cerulion_core::transport::subscriber::{DataOnlySubscriber, OwnedInboundSample};
use cerulion_core::transport::TransportManager;
use cerulion_core::wake::WakeSource;
use cerulion_core::TransportResult;

use crate::pointcloud::{FieldsLogAction, FieldsWarnLatch};
use crate::sink::route_key_for_topic;
use crate::worker::InputFrames;

/// How a tap learns that a frame arrived.
///
/// This is a per-topic COST decision, not a preference, and it is the caller's
/// to make — see [`TapManager::attach`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeMode {
    /// Poll only. The tap registers ZERO event listeners (the listener-less shape,
    /// byte-identical to every tap before wake listeners existed) and is drained on whatever
    /// cadence the caller's loop runs at.
    Timer,
    /// Poll AND wake. The tap additionally holds a listener-only
    /// [`WakeSource`], so a drain loop can block on it and see a frame within a
    /// wake latency instead of within a poll interval.
    ///
    /// The listener is a **sibling** of the tap, never a replacement: the
    /// `DataOnlySubscriber` and the drain path are untouched, so the frames, their
    /// order and their bytes are identical to [`WakeMode::Timer`]. What it costs
    /// is one `sendto` per frame PUBLISHED, and — if that producer is a GRAPH
    /// publisher — its notify elision while the tap is held.
    ///
    /// "Per frame published" is a **correction**, not a restatement. On a
    /// graph publisher this listener also arms the publisher's live-loop boundary
    /// resweep, which used to fire a real notify on every `live_step` pass the
    /// producer did NOT publish — a bill set by the graph's loop rate rather than
    /// the topic's frame rate (measured 198 notifies for 2 frames), which also
    /// woke the topic's own in-graph consumer and free-ran that graph's live loop.
    /// The resweep now announces an un-announced frame, so a quiescent producer
    /// costs nothing at all.
    Listener,
}

/// One tapped topic's frames drained in a single [`TapManager::poll_detailed`]
/// pass, carrying BOTH the absolute `topic` (so the daemon can key per-topic Hz /
/// schema stats; that keying needs the topic, which the route-keyed
/// [`InputFrames`] loses) AND the resolved `route_key` (the render key the worker
/// batch is keyed by). Only taps that yielded ≥1 frame are returned.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolledTap {
    /// The absolute topic this tap reads (the stats key).
    pub topic: String,
    /// The [`route_key_for_topic`] result forwarded as the [`InputFrames::name`].
    /// OPAQUE: it now encodes the topic AND any `entity` override, so it
    /// carries a separator that is not printable — use
    /// [`crate::sink::route_key_topic`] for anything human- or wire-facing.
    pub route_key: String,
    /// The full wire frames (WireHeader + payload) drained this pass, in arrival
    /// order (drain-to-live; no daemon-side history).
    pub frames: Vec<Vec<u8>>,
}

/// The outcome of an [`TapManager::attach`] — whether a new tap was opened or the
/// topic was already tapped (idempotent no-op). The daemon's control responder
/// reports it back to Studio. The route key is DIAGNOSTIC — the UI's
/// entity label comes from `crate::sink::reported_entity_for`, not from this key.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AttachOutcome {
    /// A new tap was opened. Carries the resolved render [route key](route_key_for_topic)
    /// the poll batch is keyed by — now the WHOLE topic (not its last
    /// path segment), plus any `entity` override encoded alongside it.
    Attached {
        /// The absolute topic now tapped.
        topic: String,
        /// The route key forwarded to the worker as the [`InputFrames::name`].
        route_key: String,
    },
    /// The topic was ALREADY tapped — no second tap was opened (idempotent).
    AlreadyAttached {
        /// The absolute topic (already tapped).
        topic: String,
    },
}

/// One attached tap: the listener-less SHM subscriber, the resolved render route
/// key, and a reusable drain scratch buffer.
struct AttachedTap {
    /// The listener-less data-only SHM reader.
    sub: DataOnlySubscriber,
    /// The [`route_key_for_topic`] result — the [`InputFrames::name`] this tap's
    /// frames are forwarded under (selects the render entity + coalescing scratch
    /// on the worker side). Resolved once at attach.
    route_key: String,
    /// Reused across polls so the steady-state drain allocates no new buffer.
    /// ALWAYS emptied before [`TapManager::poll`] returns (owned samples pin SHM
    /// pool slots + a borrow-budget unit while held — see [`OwnedInboundSample`]).
    scratch: Vec<OwnedInboundSample>,
    /// Per-tap loud-once flood latch for `drain_owned` receive errors (the house
    /// [`FieldsWarnLatch`](crate::pointcloud::FieldsWarnLatch) pattern, matching
    /// worker.rs's reconnect/panic latches). A persistent receive failure
    /// (crashed producer, iceoryx2 `ConnectionFailure`) at the ~60 Hz poll rate
    /// would otherwise `warn!` ~60×/s — the stderr-storm run-killer class.
    /// First error of a regime `warn!`, repeats `debug!` with a running suppressed
    /// count, one recovery `info!` on the first successful drain after a regime.
    drain_latch: FieldsWarnLatch,
    /// This tap's optional wake channel — `Some` under
    /// [`WakeMode::Listener`], `None` under [`WakeMode::Timer`] (and after a
    /// [`TapManager::demote_to_timer`]).
    ///
    /// `Arc` because the WAIT must happen OUTSIDE the lock this manager lives
    /// under: a drain thread collects the sources
    /// ([`TapManager::wake_sources`]), releases the lock, and blocks on them,
    /// while control threads keep attaching and detaching. A detach concurrent
    /// with a wait therefore keeps the listener alive for at most one more wait —
    /// harmless (nothing reads it, and the multiplexer drains whatever it finds),
    /// and strictly better than holding the state lock across a blocking wait.
    wake: Option<Arc<WakeSource>>,
}

/// The dynamic viz daemon's set of runtime taps over real iceoryx2. Owns a
/// [`DataOnlySubscriber`] per attached topic (keyed by absolute topic name, so
/// [`list`](Self::list) is deterministically sorted) and drains them into
/// per-tick [`InputFrames`] batches. See the [module docs](self) for the full
/// semantics + the observability invariant.
#[derive(Default)]
pub struct TapManager {
    /// Attached topic → its tap. The map keys ARE the tap set (BTreeMap → a
    /// deterministic sorted `list`).
    taps: BTreeMap<String, AttachedTap>,
}

impl TapManager {
    /// A fresh manager with no taps.
    pub fn new() -> Self {
        Self::default()
    }

    /// Attach a listener-less tap to `topic`. Idempotent: an already-attached
    /// topic opens NO second tap ([`AttachOutcome::AlreadyAttached`]).
    ///
    /// `entity_override` (the daemon's `attach{entity: "..."}`) selects the render
    /// ENTITY; the topic still decides the route's semantic knobs, and the key
    /// encodes both halves — see [`route_key_for_topic`].
    ///
    /// The error is per-topic and NON-FATAL, surfaced verbatim from
    /// [`TransportManager::create_data_only_subscriber`]: a missing topic → the
    /// "topic does not exist" error; a slot-exhausted topic → the "subscriber
    /// slots are attached / stop a tool" error. On ANY error the tap set is
    /// UNCHANGED (no phantom entry) — the topic can be attached again later.
    ///
    /// # `wake`
    ///
    /// [`WakeMode::Listener`] additionally opens a listener-only
    /// [`WakeSource`] so a drain loop can BLOCK on this tap instead of polling
    /// it. That is a decision to BILL THE PRODUCER one `sendto` per frame
    /// PUBLISHED (and, on a graph publisher, its notify elision), so the
    /// caller — which is the only layer that knows whose producer it is —
    /// chooses. Per frame PUBLISHED is the exact arithmetic: without the debt
    /// rule, a listener on a graph publisher also buys that publisher's live-loop
    /// boundary resweep a notify on every silent pass — see [`WakeMode::Listener`].
    ///
    /// `vizd` passes `Listener` on EVERY attach seam by
    /// design: a remote/mirror topic bills only the desk's own netd, and
    /// a LOCAL graph topic bills its own producer for as long as a human is
    /// watching it — which the decision holds is worth it, because the alternative
    /// left `cerulion viz` ON A ROBOT on the ~16 ms poll floor while the same
    /// data viewed from a desk ran at ~0.03 ms. This manager enforces neither
    /// policy; it does what it is told and reports the cost via
    /// [`has_wake`](Self::has_wake).
    ///
    /// **A wake that cannot be opened is NOT an attach failure.** The tap is the
    /// point; the wake is an optimisation. If the listener cannot be created
    /// (event-listener slots exhausted, a service racing away) the tap attaches
    /// anyway in [`WakeMode::Timer`] with a loud `warn!` naming the topic and the
    /// consequence — that topic keeps delivering, one poll interval later, rather
    /// than disappearing from the viewer over a latency optimisation.
    pub fn attach(
        &mut self,
        manager: &TransportManager,
        topic: &str,
        entity_override: Option<&str>,
        wake: WakeMode,
    ) -> TransportResult<AttachOutcome> {
        // Idempotent: never open a second tap for one topic (the introspection
        // slot budget is scarce). Checked BEFORE the fallible open, so a repeat
        // attach never even touches the transport.
        if self.taps.contains_key(topic) {
            return Ok(AttachOutcome::AlreadyAttached {
                topic: topic.to_string(),
            });
        }
        // The fallible open. On Err the map is untouched (checked above, inserted
        // below only on success) — no phantom entry, the topic stays re-attachable.
        let sub = manager.create_data_only_subscriber(topic)?;
        let route_key = route_key_for_topic(topic, entity_override);
        // The wake is opened AFTER the tap, so a failed tap never mints a
        // listener, and a failed WAKE never costs the tap (see the doc above).
        let wake_source = match wake {
            WakeMode::Timer => None,
            WakeMode::Listener => match manager.create_wake_listener(topic) {
                Ok(source) => Some(Arc::new(source)),
                Err(e) => {
                    tracing::warn!(
                        topic = %topic,
                        error = %e,
                        "viz tap attached WITHOUT its wake listener — this topic falls back \
                         to the poll cadence (frames still arrive, one poll interval later); \
                         a later re-attach retries the wake"
                    );
                    None
                }
            },
        };
        tracing::debug!(
            topic = %topic,
            route_key = %crate::sink::route_key_topic(&route_key),
            wake = wake_source.is_some(),
            "viz tap attached (data-only SHM tap; `wake` is the listener sibling)"
        );
        self.taps.insert(
            topic.to_string(),
            AttachedTap {
                sub,
                route_key: route_key.clone(),
                scratch: Vec::new(),
                drain_latch: FieldsWarnLatch::new(),
                wake: wake_source,
            },
        );
        Ok(AttachOutcome::Attached {
            topic: topic.to_string(),
            route_key,
        })
    }

    /// Detach `topic`, dropping its tap and releasing its introspection slot.
    /// Returns `true` if a tap was removed, `false` if the topic was not tapped
    /// (detaching an unknown topic is a no-op, never an error).
    pub fn detach(&mut self, topic: &str) -> bool {
        let removed = self.taps.remove(topic).is_some();
        if removed {
            tracing::debug!(topic = %topic, "viz tap detached (slot released)");
        }
        removed
    }

    /// Drain up to `max` frames from EACH attached tap into a per-topic
    /// [`InputFrames`] batch (keyed by the tap's resolved route key), returning
    /// only the taps that yielded at least one frame.
    ///
    /// DRAIN-TO-LIVE: each frame is copied out of SHM and its borrow released
    /// immediately, so the manager holds no frame history and never pins more
    /// than one borrow-budget's worth of pool slots. A per-tap drain error is
    /// non-fatal — the partial fill is forwarded and the tap stays attached
    /// (the next poll retries). `max == 0` drains nothing.
    ///
    /// The returned batch is fed to [`VizLogWorker::try_enqueue`](crate::worker::VizLogWorker::try_enqueue).
    pub fn poll(&mut self, max: usize) -> Vec<InputFrames> {
        self.poll_detailed(max)
            .into_iter()
            .map(|p| InputFrames {
                name: p.route_key,
                frames: p.frames,
            })
            .collect()
    }

    /// Drain each attached tap exactly like [`poll`](Self::poll), but return a
    /// [`PolledTap`] per yielding tap carrying the absolute TOPIC alongside the
    /// route key. The daemon uses this so it can key per-topic Hz / schema stats
    /// (which the route-keyed [`InputFrames`] discards) while still building the
    /// worker batch. [`poll`](Self::poll) is defined in terms of this (one drain
    /// path), so their frame semantics are byte-identical.
    pub fn poll_detailed(&mut self, max: usize) -> Vec<PolledTap> {
        let mut batch = Vec::with_capacity(self.taps.len());
        for (topic, tap) in self.taps.iter_mut() {
            let frames = drain_tap(topic, tap, max);
            if !frames.is_empty() {
                batch.push(PolledTap {
                    topic: topic.clone(),
                    route_key: tap.route_key.clone(),
                    frames,
                });
            }
        }
        batch
    }

    /// The currently-attached topics in deterministic (sorted) order.
    pub fn list(&self) -> Vec<String> {
        self.taps.keys().cloned().collect()
    }

    /// Every attached tap's wake source, in the SAME
    /// deterministic sorted order as [`list`](Self::list) — the set a drain loop
    /// blocks on.
    ///
    /// Returned by VALUE (cloned `Arc`s) rather than by reference precisely so
    /// the caller can DROP THE LOCK before waiting: a blocking wait held under
    /// this manager's mutex would stall every control verb for the length of the
    /// wait, which is the problem the wake set exists to solve, not a new way to have it.
    ///
    /// [`WakeMode::Timer`] taps contribute nothing, so an all-Timer daemon gets
    /// an empty vector and its loop must pace itself (a wake set with no sources
    /// does not block — see `WakeSet::last_wait_blocked`).
    pub fn wake_sources(&self) -> Vec<Arc<WakeSource>> {
        self.taps.values().filter_map(|t| t.wake.clone()).collect()
    }

    /// Observability (Principle #3): does `topic` currently hold a wake
    /// listener? `false` for an unattached topic, a [`WakeMode::Timer`] tap, a
    /// tap whose wake could not be opened, and one demoted by
    /// [`demote_to_timer`](Self::demote_to_timer).
    ///
    /// This is the observable that makes "which taps are billing their producer"
    /// answerable from outside — without it, the cost of the wake would
    /// be invisible to the daemon's own status surface.
    pub fn has_wake(&self, topic: &str) -> bool {
        self.taps.get(topic).is_some_and(|t| t.wake.is_some())
    }

    /// Drop `topic`'s wake listener, leaving the tap attached
    /// and delivering on the timer path. Returns `true` if a wake was actually
    /// dropped.
    ///
    /// This is the wedge fallback's lever: it un-bills the producer (its own
    /// elision gate re-arms within one publish) and costs the topic only
    /// latency, never frames — the `DataOnlySubscriber` and the SHM queue behind
    /// it are untouched.
    pub fn demote_to_timer(&mut self, topic: &str) -> bool {
        self.taps
            .get_mut(topic)
            .and_then(|t| t.wake.take())
            .is_some()
    }

    /// True if `topic` is currently tapped.
    pub fn contains(&self, topic: &str) -> bool {
        self.taps.contains_key(topic)
    }

    /// The number of attached taps.
    pub fn len(&self) -> usize {
        self.taps.len()
    }

    /// True when no taps are attached.
    pub fn is_empty(&self) -> bool {
        self.taps.is_empty()
    }

    /// Test seam (compiled OUT of production builds): arm the tap on `topic` so
    /// its next `drain_owned` receive FAILS after `n` successful drains, letting
    /// an integration test exercise the NON-FATAL drain-error path (the partial
    /// fill is forwarded, the tap survives, and the error is flood-latched — see
    /// `drain_tap`). Delegates to the underlying `DataOnlySubscriber`'s own
    /// receive-fault seam (`fault_inject_receive_after_for_test`); a no-op if
    /// `topic` is not attached. Cfg-gated (`test` / `test-helpers`) exactly like
    /// the core seam it forwards to, so it is never present in a production build.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fault_inject_tap_receive_after(&mut self, topic: &str, n: usize) {
        if let Some(tap) = self.taps.get_mut(topic) {
            tap.sub.fault_inject_receive_after_for_test(n as u32);
        }
    }
}

/// Drain up to `max` frames from ONE tap DRAIN-TO-LIVE. Drains in borrow-budget
/// chunks, copying each owned sample's full wire frame out and CLEARING the
/// scratch between chunks (so the held borrow never exceeds one chunk). A drain
/// error is non-fatal: the partial fill already copied out is returned and the
/// tap keeps its subscriber (the caller keeps it attached; the next poll retries).
///
/// The error is FLOOD-LATCHED via the tap's `drain_latch`
/// ([`FieldsWarnLatch`](crate::pointcloud::FieldsWarnLatch), matching worker.rs's
/// reconnect/panic latches): the first error of a regime `warn!`s (topic +
/// error), sustained errors downgrade to `debug!` with a running suppressed
/// count, and the first FULLY SUCCESSFUL drain after a failing regime emits one
/// recovery `info!` (and re-arms). A healthy tap that never errors is silent
/// every poll (`on_decoded` on an already-armed latch is a no-op).
fn drain_tap(topic: &str, tap: &mut AttachedTap, max: usize) -> Vec<Vec<u8>> {
    let mut frames: Vec<Vec<u8>> = Vec::new();
    if max == 0 {
        return frames;
    }
    // Never borrow more than the tap's borrow budget at once (iceoryx2 fails a
    // receive that exceeds `subscriber_max_borrowed_samples`). One chunk is
    // held → copied out → released before the next chunk.
    let budget = tap.sub.max_borrowed_samples().max(1);
    let mut remaining = max;
    while remaining > 0 {
        let want = remaining.min(budget);
        tap.scratch.clear();
        let n = match tap.sub.drain_owned(want, &mut tap.scratch) {
            Ok(n) => n,
            Err(e) => {
                // NON-FATAL: forward whatever was already drained (kept in
                // scratch on Err) and keep the tap attached — never fail the poll
                // or drop the tap. The receive failure is FLOOD-LATCHED (the house
                // FieldsWarnLatch contract): loud `warn!` on the first error of a
                // regime, `debug!` with a running suppressed count on repeats — so
                // a persistent failure at the ~60 Hz poll rate never storms stderr.
                for sample in tap.scratch.iter() {
                    frames.push(sample.payload().to_vec());
                }
                tap.scratch.clear();
                match tap.drain_latch.on_inferred() {
                    FieldsLogAction::WarnFirst => tracing::warn!(
                        topic = %topic,
                        error = %e,
                        "viz tap drain failed — forwarding the partial fill + keeping the tap \
                         (retry next poll; repeats log at debug until it drains again)"
                    ),
                    FieldsLogAction::DebugSuppressed { suppressed } => tracing::debug!(
                        topic = %topic,
                        suppressed,
                        error = %e,
                        "viz tap drain still failing (partial fill forwarded; tap kept; warn suppressed)"
                    ),
                }
                return frames;
            }
        };
        for sample in tap.scratch.iter() {
            // `OwnedInboundSample::payload()` is the FULL wire frame (32-byte
            // header + body), already sliced to `total_size` — exactly what the
            // FrameWalker decodes. One small per-frame copy (the terminal copy),
            // then the borrow is released when scratch is cleared next iteration.
            frames.push(sample.payload().to_vec());
        }
        remaining -= n;
        if n < want {
            // Fewer than requested → the SHM queue is drained.
            break;
        }
    }
    // Release every borrow before returning (drain-to-live: hold no frame past
    // the poll).
    tap.scratch.clear();
    // Reached here → the drain fully succeeded (no receive error this poll). Heal
    // a failing regime with one recovery `info!` carrying the total suppressed
    // count; `None` (already armed, or a lone-warn regime) is silent, so a healthy
    // tap that never errors logs nothing every poll.
    if let Some(suppressed) = tap.drain_latch.on_decoded() {
        tracing::info!(
            topic = %topic,
            suppressed_count = suppressed,
            "viz tap drain recovered — receive succeeding again (drain-error regime healed)"
        );
    }
    frames
}
