// SPDX-License-Identifier: AGPL-3.0-only
//! — the ERASED CARRIER SEAM: `NodeEntry::{inline_safe,
//! cer_probe}`, the two answers the boundary walk reads before it touches a
//! node.
//!
//! # What this file exists to catch
//!
//! `walk_inline` sees nodes through `InlineCaptureTarget`, whose three
//! questions are "may I run your capture on the node thread?", "is any lock in
//! your declared state held right now?" and "encode yourself". The third landed
//! with the D6 keystone; the first two are these forwards, and WITHOUT them
//! every macro node inherits `NodeEntry`'s safe defaults — `inline_safe =
//! false` — so every node takes the fork carrier and the entire inline path is
//! dead code on a real graph. That failure is SILENT: anchors still appear,
//! with identical bytes, just via `fork(2)` every cadence.
//!
//! So the oracle here is not "the seam compiles". It is that the answers TRACK
//! THE NODE'S OWN FIELDS — a plain-data node says `true`, a lock-carrying node
//! says `false`, and a node with no `CerulionState` at all says `false` — each
//! against a hand-written expectation, never against another node.
//!
//! # Why the lock is an `Arc<Mutex<T>>` and not a bare `Mutex<T>`
//!
//! Two reasons, and the second is the one that matters. It is the decided
//! shape — the ordinary way a node shares a costmap or an input queue with a
//! helper thread, and the exact field the carrier must keep off the node
//! thread. And it is the only shape whose lock the test can HOLD: the generated
//! `{Name}Entry` keeps `inner` private, so a bare `Mutex` field would be
//! unreachable once the node is moved in, and the probe arm would have to be
//! written against the user struct instead — which would exercise
//! `CerulionState::cer_probe` while leaving the `NodeEntry` FORWARD, the thing
//! this file exists for, unasserted.
//!
//! # Why `cer_probe` needs a node whose probe actually walks
//!
//! The derive short-circuits `cer_probe` to `true` when `INLINE_SAFE` is set,
//! because an inline-safe type carries no lock to probe. A probe arm written
//! over a plain-data node therefore proves nothing about probing: it would pass
//! against a `fn cer_probe(&self) -> bool { true }` stub. The lock-carrying node
//! is the only fixture that can tell the forward from the stub, which is why
//! both halves — held and released — are asserted on it.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test state_carrier_seam_test
//! ```

use std::sync::{Arc, Mutex};

use cerulion_core::error::TransportResult;
use cerulion_core::graph::node::{ClosureNodeEntry, MacroPolicy, NodeContext, NodeEntry, NodeInfo};
use cerulion_core::prelude::*;
use cerulion_core::state::CerulionState;
use native_ros2_messages::geometry_msgs::Vector3;

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// Plain data only: every field is in the inventory's `INLINE_SAFE = true`
/// group, so the fold-in's AND over the field list folds to `true`.
#[cerulion_node(period_ms = 10)]
struct PlainState {
    #[output]
    out: Vector3,
    count: u64,
    label: String,
}

#[cerulion_node_impl]
impl PlainState {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        self.out.x = self.count as f64;
        Ok(())
    }
}

/// Carries a lock, in the decided shape. `Mutex<T>` is captured (through
/// `try_lock`, never serde's blocking `lock()`) but declares `INLINE_SAFE =
/// false`, so the AND over this node's fields folds to `false` however plain
/// its other fields are — which is the property under test: ONE lock
/// anywhere in the state graph keeps the whole node off the node thread.
#[cerulion_node(period_ms = 10)]
struct LockedState {
    #[output]
    out: Vector3,
    plain: u64,
    shared: Arc<Mutex<u64>>,
}

#[cerulion_node_impl]
impl LockedState {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.plain += 1;
        self.out.x = self.plain as f64;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// `inline_safe`
// ---------------------------------------------------------------------------

#[test]
fn a_plain_data_node_answers_inline_safe_and_a_lock_carrying_one_does_not() {
    // THE forward. Both answers come from the SAME generated method, so a
    // deleted forward makes the first assertion fail (the node falls back to
    // `NodeEntry`'s `false`) while a forward hardcoded to `true` makes the
    // second fail. Neither broken variant can satisfy both, which is why they are one
    // test rather than two.
    let plain = PlainStateEntry::new();
    let locked = LockedStateEntry::new();

    assert!(
        plain.inline_safe(),
        "a node whose every field is plain data must be inline-eligible: its capture \
         runs only generated code over data that cannot block the node thread"
    );
    assert!(
        !locked.inline_safe(),
        "one `Mutex` anywhere in the declared state must keep the WHOLE node off the \
         node thread (§0.5) — the fork carrier captures it just as completely"
    );
}

#[test]
fn the_entrys_answer_is_the_state_impls_answer_not_an_independent_one() {
    // The forward must READ the fold-in's const rather than restate it. Asserted
    // against `CerulionState::INLINE_SAFE` read directly off the user struct —
    // the one place the value is derived — so a forward that hardcoded either
    // answer diverges here even if it happened to match on one fixture.
    assert_eq!(
        PlainStateEntry::new().inline_safe(),
        <PlainState as CerulionState>::INLINE_SAFE,
        "the erased answer must be the type's own const"
    );
    assert_eq!(
        LockedStateEntry::new().inline_safe(),
        <LockedState as CerulionState>::INLINE_SAFE,
        "the erased answer must be the type's own const"
    );
    // Anti-tautology: the two consts must actually DIFFER, or the equality above
    // would hold for a forward that returned a constant.
    assert_ne!(
        <PlainState as CerulionState>::INLINE_SAFE,
        <LockedState as CerulionState>::INLINE_SAFE,
        "the fixtures must disagree, or this test cannot see a constant forward"
    );
}

#[test]
fn a_node_with_no_cerulion_state_takes_the_safe_default() {
    // A `ClosureNodeEntry` has no `CerulionState` at all — it is the population
    // `NodeEntry`'s defaults exist for. `inline_safe` must be FALSE (the safe
    // direction: it costs a different carrier, never a wrong capture) and
    // `cer_probe` TRUE (sound because the only impls that can answer `false` are
    // ones this crate writes; anything else is covered by the child's progress
    // watchdog, not by a claim made here).
    let info =
        NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::Period { period_ms: 10 });
    let closure = ClosureNodeEntry::new(info, |_ctx: &mut NodeContext| -> TransportResult<()> {
        Ok(())
    })
    .with_label("probe_closure");

    assert!(
        !closure.inline_safe(),
        "a node that has not proved the property must not silently claim it"
    );
    assert!(
        closure.cer_probe(),
        "the probe's default is TRUE: it reports a lock it can SEE, and it can see none"
    );
}

// ---------------------------------------------------------------------------
// `cer_probe`
// ---------------------------------------------------------------------------

#[test]
fn the_probe_reports_a_held_lock_through_the_entry_and_clears_when_released() {
    // The whole purpose: a contended lock at the anchor boundary means a
    // FOREIGN holder (the graph's own threads provably hold none), and forking
    // into it would leave the child holding it forever — a fork child has one
    // thread, so a lock another thread held at the fork instant is held by
    // nobody in the child's image.
    //
    // The contention is REAL and it is reached the way a helper thread reaches
    // it: a clone of the node's own `Arc`, taken before the node moves into its
    // entry, locked by this thread while the ENTRY is asked. That is what makes
    // this an assertion about the `NodeEntry` forward rather than about
    // `CerulionState::cer_probe`, which the entry could ignore.
    let node = LockedState::default();
    let shared = Arc::clone(&node.shared);
    let entry = LockedStateEntry::with_state(node);

    assert!(
        entry.cer_probe(),
        "an uncontended node must probe clean — without this the anchor would be \
         skipped forever and the `false` arm below would prove nothing"
    );

    {
        let _held = shared.lock().expect("uncontended at this point");
        assert!(
            !entry.cer_probe(),
            "a held lock in the DECLARED state graph must make the probe refuse, so the \
             boundary skips the anchor instead of forking into it"
        );
    }

    assert!(
        entry.cer_probe(),
        "and it must CLEAR when the lock is released — a probe that latched would \
         cost the node every subsequent anchor for the life of the process"
    );
}

#[test]
fn a_plain_nodes_probe_is_the_const_short_circuit_not_a_walk() {
    // The derive short-circuits `cer_probe` to `true` when `INLINE_SAFE` is set,
    // because an inline-safe type carries no lock to probe. Pinned so the
    // short-circuit is a DECISION rather than an accident of the fixtures: this
    // is the arm that would fail if `INLINE_SAFE` ever folded `true` for a type
    // that does carry a lock, which is the one way the two consts can disagree
    // dangerously.
    // The precondition is a COMPILE-time fact about the fixture, so it is
    // const-asserted rather than checked at run time: if `PlainState` ever
    // stopped being inline-safe this test would silently start asserting
    // something else (that a WALKING probe returns true), and the arm below
    // would no longer pin the short circuit at all.
    const _: () = assert!(<PlainState as CerulionState>::INLINE_SAFE);

    let plain = PlainStateEntry::new();
    assert!(
        plain.cer_probe(),
        "an inline-safe node's probe is unconditionally true — it has nothing to probe"
    );
}

// ---------------------------------------------------------------------------
// The seam composes with the D6 keystone
// ---------------------------------------------------------------------------

#[test]
fn every_carrier_answer_is_present_on_the_same_node() {
    // The walk asks all three questions of one node. A node that answered two of
    // them and defaulted the third would be captured by the wrong carrier or not
    // at all, so the composition is pinned rather than assumed — `state_shape`
    // and the two carrier answers must agree that
    // this node has state and may be captured where it stands.
    let plain = PlainStateEntry::new();

    assert!(
        plain.state_shape().is_some(),
        "the node declares state (D6's forward)"
    );
    assert!(plain.inline_safe(), "and it may be captured inline");
    assert!(plain.cer_probe(), "and nothing is holding it up");
}
