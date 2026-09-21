// SPDX-License-Identifier: AGPL-3.0-only
//! The NODE-DEATH ledger.
//!
//! # What this is
//!
//! `TriggerKind::PanicDisable` had a wire byte, a posture switch, a
//! `CaptureRequest` constructor and ZERO producers. This is the half a producer
//! needs that neither the scheduler nor the runtime can do alone: a place to
//! RECORD "this node has stopped" at the instant it is observed, so that the
//! publish can happen somewhere else entirely.
//!
//! Splitting the mint from the publish is not tidiness. The scheduler's disable
//! edge runs inside `fire_node_into`, which the wide (`>= PARALLEL_FIRE_THRESHOLD`)
//! path drives ON A RAYON WORKER holding a disjoint `&mut ScheduledNode` — so
//! anything at that site must be `Send`, must touch no `Scheduler` state, and
//! absolutely must not open an iceoryx2 node. What it CAN do is push a record
//! into a shared ledger; the step boundary, where every tick has returned and
//! rayon is scope-joined, is where that ledger is read.
//!
//! # TWO sites, because one of them is unreachable on a real graph
//!
//! The obvious site is the scheduler's circuit breaker: `MAX_CONSECUTIVE_PANICS`
//! consecutive panics and the node is `disabled`. On a bare [`Scheduler`] that is
//! exactly what happens. Through a [`GraphRuntime`](crate::graph::GraphRuntime) it
//! is UNREACHABLE, and the reason is structural rather than incidental: the
//! runtime's tick callback locks the node's `Arc<Mutex<Box<dyn NodeEntry>>>` and
//! calls `tick()` holding the guard, so the FIRST panic POISONS that mutex and
//! every later fire's `lock()` returns `Err` — the callback logs, skips, and
//! never re-enters `tick()`. `panic_count` sticks at 1, `consecutive_panics` never
//! reaches 3, and the disable edge never fires.
//!
//! So the poison transition IS what a real node death looks like, and it gets its
//! own cause. `TriggerKind::ProcessFault`'s own doc reserves contained panics for
//! `PanicDisable`, which is what makes both of these the same KIND with the same
//! regime key (the node id) rather than two.
//!
//! # ARMING is the replay firewall
//!
//! The polled `step()` seam reaches the same step boundary the live loop does, and
//! a replay must never publish a live capture request. That is not left to
//! reasoning: [`NodeDeathLedger::record`] is a no-op until something ARMS the
//! ledger, and the only thing that arms it is `run_live`. A replay drives
//! `step()`, never `run_live`, so on that path the ledger is not merely undrained
//! — it is never written, which also keeps the cost of the whole feature at one
//! atomic load per step boundary.
//!
//! # Bounded, because a ledger nobody drains must not grow
//!
//! Deaths are deduplicated per `(node, cause)` and capped at
//! [`MAX_TRACKED_NODE_DEATHS`]. Both bounds matter for the same reason the
//! trigger gate's own cause list is capped: the ledger is written by a
//! failure path, and a failure path is exactly where "it cannot happen often" is
//! the assumption that breaks. Past the cap the ledger COUNTS what it dropped
//! rather than growing, and says so.
//!
//! `TriggerKind::PanicDisable` and `CaptureRequest` are deliberately named in
//! plain backticks rather than linked: they live in `crate::flashback`, which is
//! `#[cfg(unix)]` (POSIX SHM all the way down), while THIS module is portable —
//! it is a pure ledger and nothing in it touches a plane. A doc LINK from a
//! portable item into a gated module fails CI's Documentation job on a non-unix
//! build with `rustdoc::broken_intra_doc_links`, which
//! `cfg_audit_test::no_portable_item_references_a_unix_only_module` walks for. A
//! prose mention is resolved by nobody and is what that gate explicitly allows.
//!
//! [`Scheduler`]: super::Scheduler

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Mutex;

/// How many DISTINCT node deaths one ledger remembers.
///
/// Generous against the shape it bounds — a graph with more than this many nodes
/// dying in one run has a problem no capture describes — and small enough that a
/// pathological run cannot turn a diagnostic into a leak. Past it, deaths are
/// COUNTED (see [`NodeDeathLedger::dropped`]) rather than stored.
pub const MAX_TRACKED_NODE_DEATHS: usize = 64;

/// WHY a node stopped.
///
/// Two variants rather than one because the two are observed at different sites
/// and mean subtly different things to whoever reads the capture: one is the
/// scheduler's own circuit breaker opening, the other is a node that panicked
/// ONCE and can never run again. They mint the same `TriggerKind::PanicDisable`
/// request — the remedy is identical and the regime key is the node — but the
/// DETAIL says which was seen. (Backticks, not a link: see the module docs.)
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum NodeDeathCause {
    /// The scheduler's circuit breaker opened: `MAX_CONSECUTIVE_PANICS`
    /// consecutive panics and the node was `disabled`.
    ///
    /// Reachable on a bare `Scheduler` and on any host that does not poison a
    /// node's entry on the first panic.
    DisabledAfterPanics,
    /// The node's entry mutex is POISONED — it panicked once inside
    /// `GraphRuntime`'s tick callback, which holds the guard across `tick()`, so
    /// every later fire skips it.
    ///
    /// This is what a node death looks like on a real graph run. The node is not
    /// flagged `disabled` (the scheduler never saw three panics), but it will
    /// never execute again, which is the same fact for an operator.
    EntryPoisoned,
    /// A CDYLIB node reported a panic-class failure across the FFI boundary —
    /// `DylibNodeEntry::tick` returning `TransportError::NodeTickPanicked` for
    /// FFI code 2 (the cdylib's own `catch_unwind` caught a panic) or 3 (its
    /// `NODES` mutex is poisoned by an earlier one).
    ///
    /// Its own variant, and NOT [`EntryPoisoned`](Self::EntryPoisoned), because
    /// on this path the HOST's entry mutex is not poisoned at all: the panic was
    /// caught on the far side of the FFI and comes back as a returned `Err`, so
    /// nothing unwound through the host's guard. Reporting it as an entry poison
    /// would put a false mechanism in the bag's detail, which is exactly what
    /// this enum's per-cause `detail()` exists to prevent.
    ///
    /// TERMINAL for the same reason `EntryPoisoned` is: code 2 leaves the
    /// cdylib's own `NODES` mutex poisoned, after which every later tick returns
    /// code 3 forever, and nothing calls `clear_poison` on either side.
    ///
    /// This is the SHIPPING node shape — every node built by
    /// `cerulion node build` is a cdylib, and before this cause existed it was
    /// recorded by NEITHER mint site: the returned `Err` never poisons the host
    /// entry (so the poison arm never runs) and never unwinds (so the scheduler's
    /// `consecutive_panics` resets to 0 on the callback's clean return and the
    /// circuit breaker never opens).
    CdylibPanicked,
}

impl NodeDeathCause {
    /// The word this cause uses in a capture's DETAIL and in logs — ONE spelling,
    /// so a `tracing` field and the bag's own manifest cannot drift (the single-copy
    /// rule applied to a vocabulary).
    pub fn as_wire(self) -> &'static str {
        match self {
            Self::DisabledAfterPanics => "disabled_after_consecutive_panics",
            Self::EntryPoisoned => "entry_poisoned_after_panic",
            Self::CdylibPanicked => "cdylib_panicked",
        }
    }

    /// The sentence a capture carries.
    pub fn detail(self) -> &'static str {
        match self {
            Self::DisabledAfterPanics => {
                "the scheduler disabled this node after consecutive panics — it will not fire \
                 again unless something resets it"
            }
            Self::EntryPoisoned => {
                "this node panicked and its entry is poisoned — every later fire is skipped, so \
                 it will not execute again in this run"
            }
            Self::CdylibPanicked => {
                "this cdylib node panicked across the FFI boundary and its own node registry is \
                 poisoned — every later tick returns the poisoned code, so it will not execute \
                 again in this run"
            }
        }
    }
}

/// One recorded death.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeDeath {
    /// The node that stopped. This is the capture request's SUBJECT — the regime
    /// key — so two deaths of one node coalesce while two nodes dying are two
    /// conditions.
    pub node_id: String,
    /// What was observed.
    pub cause: NodeDeathCause,
}

/// The shared per-run ledger both mint sites write and the step boundary reads.
///
/// Minted with a fresh default in `Scheduler::add_node` — so a bare scheduler with
/// no runtime has a per-node ledger nothing shares and nothing drains, which is
/// the `discard_signal` precedent — and OVERWRITTEN by the runtime with the one
/// shared instance it also hands to every tick callback.
#[derive(Debug, Default)]
pub struct NodeDeathLedger {
    /// Set when a live loop attaches. UNARMED is the replay firewall AND the
    /// zero-cost state: [`record`](Self::record) returns on one atomic load.
    armed: AtomicBool,
    /// The step-boundary drain's fast gate — one atomic load, no lock, no alloc.
    pending: AtomicBool,
    /// Distinct deaths, in observation order. Order is stable so a capture's
    /// cause list is deterministic across runs of one recording.
    deaths: Mutex<Vec<NodeDeath>>,
    /// Deaths refused by [`MAX_TRACKED_NODE_DEATHS`]. Never reset — a bound that
    /// has been reached is a fact about the answer, so a reader can tell "no
    /// further deaths" from "further deaths I refused to hold".
    dropped: AtomicU32,
}

impl NodeDeathLedger {
    /// A fresh, UNARMED ledger.
    pub fn new() -> Self {
        Self::default()
    }

    /// Arm the ledger — the live loop's declaration that a publisher exists.
    ///
    /// Idempotent: `run_live` is resumable, and re-arming must not disturb a
    /// death already recorded.
    pub fn arm(&self) {
        self.armed.store(true, Ordering::Release);
    }

    /// Whether anything will be recorded. Principle #3: the replay firewall is
    /// only checkable because this is observable.
    pub fn is_armed(&self) -> bool {
        self.armed.load(Ordering::Acquire)
    }

    /// Whether a drain would find anything — the step-boundary gate.
    ///
    /// One atomic load (`Acquire`, not `Relaxed` — see the accessors), which is
    /// the whole cost of producer 2 on a graph
    /// whose nodes are healthy (and on every graph at all while unarmed).
    pub fn has_pending(&self) -> bool {
        self.pending.load(Ordering::Acquire)
    }

    /// How many deaths the cap refused.
    pub fn dropped(&self) -> u32 {
        self.dropped.load(Ordering::Acquire)
    }

    /// Record one death — a NO-OP unless armed, and deduplicated per
    /// `(node, cause)`.
    ///
    /// Deduplication lives HERE rather than at each site because the two sites
    /// dedup for different reasons and only one of them can do it locally: the
    /// scheduler edge is guarded on `!node.disabled` so it fires once per
    /// TRANSITION (and `reset_node` re-enables, so a reset plus three more panics
    /// is a legitimate SECOND transition), while the poison arm runs at fire rate
    /// forever once a node is dead. Neither guard can see the other's site, and a
    /// capture keyed on the node must not be minted twice for one condition.
    ///
    /// Returns whether this call recorded something NEW.
    pub fn record(&self, node_id: &str, cause: NodeDeathCause) -> bool {
        // The unarmed early-out is the replay firewall and the zero-cost path.
        if !self.is_armed() {
            return false;
        }
        let mut deaths = match self.deaths.lock() {
            Ok(g) => g,
            // A poisoned diagnostic must never wedge the path it observes — the
            // `lock_regime_latch` rule. Recovering the guard keeps the ledger
            // usable; the worst case is one duplicate entry, which the gate's own
            // latch absorbs.
            Err(e) => e.into_inner(),
        };
        if deaths
            .iter()
            .any(|d| d.node_id == node_id && d.cause == cause)
        {
            return false;
        }
        if deaths.len() >= MAX_TRACKED_NODE_DEATHS {
            self.dropped.fetch_add(1, Ordering::Release);
            return false;
        }
        // The one allocation on this path, and it is reached only when a node has
        // STOPPED: never on a replay (unarmed), never on a healthy graph, never
        // on a repeat (the dedup above), and never more than
        // `MAX_TRACKED_NODE_DEATHS` times in one run (the cap above).
        let node_id = node_id.to_string(); // hot-path-alloc-ok: cold: one String per DISTINCT node death
        deaths.push(NodeDeath { node_id, cause });
        drop(deaths);
        self.pending.store(true, Ordering::Release);
        true
    }

    /// Take everything recorded so far, clearing the pending gate.
    ///
    /// The dedup is scoped to ONE DRAIN WINDOW, and that is the whole of it: the
    /// memory IS `deaths`, and `mem::take` below empties it, so nothing here
    /// remembers a node once its death has been handed to the publisher.
    ///
    /// Repeats ACROSS drains are prevented at the source instead — the scheduler
    /// edge fires once per disable TRANSITION, and the poison arm latches inside
    /// its own callback closure. Keeping a separate, un-taken key set here would
    /// be a second copy of a fact those two sites already own, and it would have
    /// to be bounded and aged on its own.
    pub fn take(&self) -> Vec<NodeDeath> {
        if !self.has_pending() {
            // hot-path-alloc-ok: `Vec::new()` allocates NOTHING (it is a dangling-pointer
            // empty vec); this is the step boundary's fast return
            return Vec::new();
        }
        let mut deaths = match self.deaths.lock() {
            Ok(g) => g,
            Err(e) => e.into_inner(),
        };
        self.pending.store(false, Ordering::Release);
        std::mem::take(&mut deaths)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_unarmed_ledger_records_nothing_which_is_the_replay_firewall() {
        // The whole reason a replay cannot publish a capture request: not that
        // nothing drains the ledger, but that nothing WRITES it.
        let ledger = NodeDeathLedger::new();
        assert!(!ledger.is_armed());
        assert!(!ledger.record("cam", NodeDeathCause::EntryPoisoned));
        assert!(
            !ledger.has_pending(),
            "an unarmed ledger has nothing to drain"
        );
        assert!(ledger.take().is_empty());
        // ANTI-TAUTOLOGY: the same call after arming DOES record, so the arm
        // above is capable of failing.
        ledger.arm();
        assert!(ledger.record("cam", NodeDeathCause::EntryPoisoned));
        assert!(ledger.has_pending());
    }

    #[test]
    fn a_repeat_of_one_node_and_cause_is_recorded_once() {
        // The poison arm runs at FIRE RATE once a node is dead, so without this a
        // 1 kHz graph mints a thousand capture requests a second.
        let ledger = NodeDeathLedger::new();
        ledger.arm();
        assert!(ledger.record("cam", NodeDeathCause::EntryPoisoned));
        assert!(!ledger.record("cam", NodeDeathCause::EntryPoisoned));
        assert!(!ledger.record("cam", NodeDeathCause::EntryPoisoned));
        let taken = ledger.take();
        assert_eq!(
            taken,
            vec![NodeDeath {
                node_id: "cam".to_string(),
                cause: NodeDeathCause::EntryPoisoned,
            }]
        );
    }

    #[test]
    fn two_causes_on_one_node_and_two_nodes_are_distinct_records() {
        // A node can BOTH be disabled by the breaker and be poisoned (different
        // hosts, different paths), and the DETAIL differs — so the pair is the
        // dedup key, not the node alone.
        let ledger = NodeDeathLedger::new();
        ledger.arm();
        assert!(ledger.record("cam", NodeDeathCause::EntryPoisoned));
        assert!(ledger.record("cam", NodeDeathCause::DisabledAfterPanics));
        assert!(ledger.record("imu", NodeDeathCause::EntryPoisoned));
        let taken = ledger.take();
        // HAND oracle, in observation order.
        assert_eq!(
            taken,
            vec![
                NodeDeath {
                    node_id: "cam".to_string(),
                    cause: NodeDeathCause::EntryPoisoned
                },
                NodeDeath {
                    node_id: "cam".to_string(),
                    cause: NodeDeathCause::DisabledAfterPanics
                },
                NodeDeath {
                    node_id: "imu".to_string(),
                    cause: NodeDeathCause::EntryPoisoned
                },
            ]
        );
    }

    #[test]
    fn a_take_clears_the_pending_gate_and_a_second_take_is_empty() {
        let ledger = NodeDeathLedger::new();
        ledger.arm();
        ledger.record("cam", NodeDeathCause::EntryPoisoned);
        assert!(ledger.has_pending());
        assert_eq!(ledger.take().len(), 1);
        assert!(!ledger.has_pending(), "the gate closes with the drain");
        assert!(ledger.take().is_empty());
    }

    #[test]
    fn the_cap_refuses_and_counts_rather_than_growing() {
        // A ledger written by a failure path must not be the leak.
        let ledger = NodeDeathLedger::new();
        ledger.arm();
        for i in 0..MAX_TRACKED_NODE_DEATHS {
            assert!(ledger.record(&format!("n{i}"), NodeDeathCause::EntryPoisoned));
        }
        assert_eq!(ledger.dropped(), 0, "nothing refused at the cap itself");
        assert!(!ledger.record("one_too_many", NodeDeathCause::EntryPoisoned));
        assert!(!ledger.record("and_another", NodeDeathCause::EntryPoisoned));
        assert_eq!(ledger.dropped(), 2, "refusals are COUNTED, never silent");
        assert_eq!(ledger.take().len(), MAX_TRACKED_NODE_DEATHS);
    }

    #[test]
    fn the_two_causes_carry_distinct_words_and_distinct_sentences() {
        // The DETAIL is what an operator reads off the capture, and the two
        // conditions have genuinely different meanings (one node stopped after
        // three panics; one node panicked once and can never run again).
        assert_ne!(
            NodeDeathCause::DisabledAfterPanics.as_wire(),
            NodeDeathCause::EntryPoisoned.as_wire()
        );
        assert_ne!(
            NodeDeathCause::DisabledAfterPanics.detail(),
            NodeDeathCause::EntryPoisoned.detail()
        );
        assert!(NodeDeathCause::EntryPoisoned.detail().contains("poisoned"));
        assert!(NodeDeathCause::DisabledAfterPanics
            .detail()
            .contains("disabled"));
    }
}
