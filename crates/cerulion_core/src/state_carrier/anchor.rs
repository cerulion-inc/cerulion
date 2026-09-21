// SPDX-License-Identifier: AGPL-3.0-only
//! `take_anchor`'s BOUNDED INLINE WALK — the bounded attempt that
//! decides, per node and without reading a clock, which carrier each node takes.
//!
//! # The two filters, and why neither is a timer
//!
//! At a due boundary the executor walks the process's nodes in DECLARATION order.
//! A node is attempted inline only if `CerulionState::INLINE_SAFE` — the const saying
//! its capture runs only framework-generated code over data that cannot block it
//! — and it stays inline only while its encode FITS the boundary's single arena.
//! Everything else joins a fork set, and one `fork(2)` covers the whole set.
//!
//! `Instant::now()` at a step boundary is both a determinism hazard and, on macOS
//! under background QoS, unreliable: one run measured a nominal 150 ms charged
//! as 1100-1696 ms. So the budget is expressed in BYTES, and `INLINE_SAFE` is what
//! turns that byte bound into a TIME bound: the node thread acquires no lock, makes no
//! syscall, runs no user-defined control flow and cannot recurse into a cycle, so its
//! only unbounded dimension is bytes.
//!
//! # Three properties are load-bearing
//!
//! **The `INLINE_SAFE` gate is FIRST.** An ineligible node costs zero arena bytes and
//! zero user-code execution on the node thread. Testing it after building a sink would
//! put user-code execution back on the hot loop for exactly the nodes the const exists to
//! keep off it.
//!
//! **The budget is SHARED across the boundary, and aborted bytes still count.** An
//! overflowing node's consumed capacity is charged to the running total, so the TOTAL
//! inline encoding at any boundary is hard bounded no matter how many nodes overflow —
//! a 40-node worker cannot spend 40 budgets. `BoundedSink::consumed` charges a REFUSED
//! sink its whole granted capacity, so the first overflow effectively pushes the rest
//! of that boundary's nodes into the fork set (they get a zero-byte sink and refuse at
//! their first write). That is not a loss: one fork covers the whole set, and the
//! fork's cost is per-PROCESS, not per-node.
//!
//! **The carrier split is not part of the bag's contract.** Both carriers call the same
//! generated encoder and emit identical bytes, so which nodes went inline and which
//! forked is invisible in the recording. It is deterministic anyway — declaration order
//! over identical state — but nothing depends on that.
//!
//! # A capture that PANICS must not poison the node's mutex
//!
//! `catch_unwind` here is mandatory, not defensive. The scheduler holds each node
//! behind a mutex and a panic escaping a capture that held the guard would poison it,
//! killing **every subsequent tick of that node for the process lifetime** and
//! misreporting the cause as a tick panic that never happened. The caller therefore
//! resolves its guard and passes `&T`, keeping the catch INSIDE the guard's scope so
//! the guard drops normally.
//!
//! The fork carrier does not have this hazard at all — user code never runs on a node
//! thread there — which is a genuine advantage of the fork, worth stating beside the
//! inline path rather than burying.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on the
//! `pub mod state_carrier;` declaration in `lib.rs`.

use crate::state::{BoundedSink, StateError};

/// One node, as the inline walk sees it.
///
/// An erased view rather than `CerulionState` itself, because that trait is not object
/// safe (associated consts, and a `cer_read` returning `Self`) and the walk must be
/// able to iterate a heterogeneous graph. The derive supplies the real
/// implementation; a hand-written one is exactly as trusted as its `INLINE_SAFE`
/// declaration, which defaults to `false` for that reason.
pub trait InlineCaptureTarget {
    /// `CerulionState::INLINE_SAFE`, surfaced per node.
    ///
    /// `false` sends the node to the fork carrier BEFORE anything else happens to it.
    fn inline_safe(&self) -> bool;

    /// `CerulionState::cer_probe` — non-blocking, total over the declared state.
    fn cer_probe(&self) -> bool;

    /// Encode into `sink`.
    fn capture(&self, sink: &mut BoundedSink<'_>) -> Result<(), StateError>;
}

/// Why a node was deferred to the fork carrier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ForkReason {
    /// `INLINE_SAFE` is `false` — the inline-safety gate, applied before any encoding.
    NotInlineSafe,
    /// The encode did not fit what was left of the boundary budget.
    ArenaOverflow,
    /// The encoder returned an error.
    EncoderError,
    /// The encoder PANICKED. Caught, so the node's mutex is not poisoned.
    CaptureUnwound,
}

/// What one boundary's inline walk did.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct InlineWalkStats {
    /// Nodes captured inline.
    pub inline_nodes: u32,
    /// Nodes deferred to the fork carrier.
    pub forked_nodes: u32,
    /// Bytes of the shared budget consumed, INCLUDING capacity charged to aborted
    /// attempts.
    pub bytes_used: usize,
    /// Nodes whose encoder panicked (a subset of `forked_nodes`).
    pub unwound_nodes: u32,
    /// Nodes refused entry to the arena WITHOUT any encoding being attempted — the
    /// `INLINE_SAFE` gate (a subset of `forked_nodes`).
    pub gated_nodes: u32,
}

/// The up-front quiescence probe, run ONCE before the walk.
///
/// Returns the index of the first node whose declared state graph holds a contended
/// lock. A contended lock at this boundary is genuinely anomalous — the graph's own
/// threads provably hold none — so it means a FOREIGN holder, and the whole
/// anchor is skipped rather than attempted: forking into a held lock would leave the
/// child holding it forever (a fork child has one thread), and the watchdog would
/// SIGKILL it five seconds later. Skipping converts that into an unforked run whose
/// manifest still names the node.
///
/// Running it ONCE UP FRONT rather than per node is what saves the wasted inline
/// encoding: a contended lock means no anchor this cadence either way.
pub fn probe_quiescent<T: InlineCaptureTarget>(nodes: &[T]) -> Option<usize> {
    nodes.iter().position(|n| !n.cer_probe())
}

/// The bounded inline walk.
///
/// `arena` IS the boundary budget: nothing is written past it and nothing is charged
/// past it. `fork_set` is CLEARED and refilled, so the caller can reuse one allocation
/// across boundaries — the walk itself allocates nothing.
///
/// `on_part` receives each successful node's bytes while they are still in the arena,
/// so a part is never copied out to be handed on.
pub fn walk_inline<T: InlineCaptureTarget>(
    nodes: &[T],
    arena: &mut [u8],
    fork_set: &mut Vec<(u32, ForkReason)>,
    mut on_part: impl FnMut(u32, &[u8]),
) -> InlineWalkStats {
    fork_set.clear();
    let mut stats = InlineWalkStats::default();
    let mut used = 0usize;

    for (idx, node) in nodes.iter().enumerate() {
        let idx = idx as u32;

        // THE `INLINE_SAFE` GATE, FIRST. An ineligible node costs zero arena bytes and zero
        // user-code execution on the node thread.
        if !node.inline_safe() {
            fork_set.push((idx, ForkReason::NotInlineSafe));
            stats.forked_nodes += 1;
            stats.gated_nodes += 1;
            continue;
        }

        let mut refused_or_failed: Option<ForkReason> = None;
        {
            // THE SHARED BUDGET, and it is the SLICE that expresses it: each node is
            // granted only what is LEFT of the arena, so once that reaches zero every
            // later node gets a zero-byte sink and refuses at its first write.
            //
            // Deliberately `new` over `with_capacity(.., budget - used)`. The two are
            // the same number, and `with_capacity` CLAMPS its argument to the slice
            // anyway — so the second form states the bound twice while only one of them
            // is load-bearing. A mutation run proved the cost of that: replacing the
            // capacity argument with the FULL budget changed nothing and survived the
            // whole suite, because the slice was doing the work all along. One
            // expression of the bound is one thing to get wrong.
            let mut sink = BoundedSink::new(&mut arena[used..]);
            // MANDATORY: a panic escaping here would poison the node's mutex and
            // kill every subsequent tick of that node for the process lifetime, while
            // reporting a tick panic that never happened.
            let outcome =
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| node.capture(&mut sink)));
            match outcome {
                Ok(Ok(())) if !sink.refused() => {
                    let n = sink.len();
                    on_part(idx, sink.written());
                    used += n;
                    stats.inline_nodes += 1;
                }
                Ok(Ok(())) => {
                    // A sink that reports success while having REFUSED a write is a
                    // partial encoding claiming to be whole. Charged and forked rather
                    // than trusted — the bytes are discarded either way.
                    used += sink.consumed();
                    refused_or_failed = Some(ForkReason::ArenaOverflow);
                }
                Ok(Err(e)) => {
                    used += sink.consumed();
                    refused_or_failed =
                        Some(if matches!(e, StateError::SinkFull) || sink.refused() {
                            ForkReason::ArenaOverflow
                        } else {
                            ForkReason::EncoderError
                        });
                }
                Err(_) => {
                    used += sink.consumed();
                    refused_or_failed = Some(ForkReason::CaptureUnwound);
                }
            }
        }
        if let Some(reason) = refused_or_failed {
            if reason == ForkReason::CaptureUnwound {
                stats.unwound_nodes += 1;
            }
            fork_set.push((idx, reason));
            stats.forked_nodes += 1;
        }
    }

    stats.bytes_used = used;
    stats
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::StateSink;

    /// A target whose behaviour is spelled out by the test rather than derived.
    struct Target {
        inline_safe: bool,
        probe: bool,
        /// Bytes to write, in one call.
        bytes: usize,
        behaviour: Behaviour,
        /// Counts every time `capture` was ENTERED — how the gate's "zero user code"
        /// claim is observed.
        entered: std::cell::Cell<u32>,
    }

    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Behaviour {
        Ok,
        Err,
        Panic,
        /// Writes past the grant and SWALLOWS the refusal, returning `Ok(())`.
        ///
        /// Unreachable through a generated encoder (they propagate with `?`), and
        /// exactly what a hand-written `impl CerulionState` can do by accident.
        OkAfterRefusal,
    }

    impl Target {
        fn ok(bytes: usize) -> Self {
            Self {
                inline_safe: true,
                probe: true,
                bytes,
                behaviour: Behaviour::Ok,
                entered: std::cell::Cell::new(0),
            }
        }
        fn gated() -> Self {
            Self {
                inline_safe: false,
                ..Self::ok(4)
            }
        }
        fn with(mut self, b: Behaviour) -> Self {
            self.behaviour = b;
            self
        }
        fn contended(mut self) -> Self {
            self.probe = false;
            self
        }
    }

    impl InlineCaptureTarget for Target {
        fn inline_safe(&self) -> bool {
            self.inline_safe
        }
        fn cer_probe(&self) -> bool {
            self.probe
        }
        fn capture(&self, sink: &mut BoundedSink<'_>) -> Result<(), StateError> {
            self.entered.set(self.entered.get() + 1);
            match self.behaviour {
                Behaviour::Panic => panic!("encoder blew up"),
                Behaviour::Err => Err(StateError::NonUtf8Path),
                Behaviour::Ok => {
                    let buf = vec![0xABu8; self.bytes];
                    sink.write(&buf)?;
                    Ok(())
                }
                Behaviour::OkAfterRefusal => {
                    let buf = vec![0xABu8; self.bytes];
                    // The swallow: the sink refused, and this reports success anyway.
                    let _ = sink.write(&buf);
                    Ok(())
                }
            }
        }
    }

    /// `(stats, fork set, published parts as (node, byte count))`.
    type WalkResult = (InlineWalkStats, Vec<(u32, ForkReason)>, Vec<(u32, usize)>);

    fn walk(nodes: &[Target], arena_len: usize) -> WalkResult {
        let mut arena = vec![0u8; arena_len];
        let mut fork_set = Vec::new();
        let mut parts = Vec::new();
        let stats = walk_inline(nodes, &mut arena, &mut fork_set, |idx, bytes| {
            parts.push((idx, bytes.len()))
        });
        (stats, fork_set, parts)
    }

    #[test]
    fn the_inline_safe_gate_runs_first_and_costs_zero_bytes_and_zero_user_code() {
        // The gate's whole point: an ineligible node must not have its encoder ENTERED at
        // all — testing the const after building a sink would put the hazard back on
        // the node thread for exactly the nodes the const exists to keep off it. The
        // entry counter is the only way to see that; a byte count alone would be
        // satisfied by an encoder that ran and wrote nothing.
        let nodes = [Target::gated(), Target::ok(8)];
        let (stats, fork_set, parts) = walk(&nodes, 1024);

        assert_eq!(
            nodes[0].entered.get(),
            0,
            "a gated node's encoder must NEVER run"
        );
        assert_eq!(nodes[1].entered.get(), 1, "an eligible node's must");
        assert_eq!(stats.gated_nodes, 1);
        assert_eq!(stats.inline_nodes, 1);
        assert_eq!(stats.bytes_used, 8, "the gated node cost zero arena bytes");
        assert_eq!(fork_set, vec![(0, ForkReason::NotInlineSafe)]);
        assert_eq!(parts, vec![(1, 8)]);
    }

    #[test]
    fn the_budget_is_shared_so_forty_nodes_cannot_spend_forty_budgets() {
        // THE bound. Ten nodes each wanting the WHOLE arena: the first fits, and the
        // total charged must never exceed one budget however many follow. A per-node
        // budget would charge 10x here and put ~10 arenas of work on one boundary.
        const ARENA: usize = 64;
        let nodes: Vec<Target> = (0..10).map(|_| Target::ok(ARENA)).collect();
        let (stats, fork_set, parts) = walk(&nodes, ARENA);

        assert_eq!(parts, vec![(0, ARENA)], "exactly the first node fits");
        assert_eq!(stats.inline_nodes, 1);
        assert_eq!(stats.forked_nodes, 9);
        assert_eq!(
            stats.bytes_used, ARENA,
            "the TOTAL inline encoding at a boundary is hard bounded at one budget"
        );
        assert!(
            fork_set
                .iter()
                .all(|(_, r)| *r == ForkReason::ArenaOverflow),
            "every node after the arena filled must be deferred as an overflow: {fork_set:?}"
        );
    }

    #[test]
    fn an_aborted_attempts_capacity_is_charged_to_the_shared_budget() {
        // The shared budget's consequence, stated as a number: the first overflow charges its WHOLE
        // grant, so the rest of the boundary's nodes get a zero-byte sink and refuse at
        // their first write. Without the charge, an overflowing node would cost nothing
        // and a boundary could retry the full budget once per node.
        const ARENA: usize = 100;
        let nodes = [Target::ok(10), Target::ok(1000), Target::ok(4)];
        let (stats, fork_set, parts) = walk(&nodes, ARENA);

        assert_eq!(
            parts,
            vec![(0, 10)],
            "only the first node's part is published"
        );
        assert_eq!(
            stats.bytes_used, ARENA,
            "10 written + the overflowing node's remaining 90 charged = the whole budget"
        );
        assert_eq!(
            fork_set,
            vec![
                (1, ForkReason::ArenaOverflow),
                (2, ForkReason::ArenaOverflow)
            ],
            "and the node AFTER the overflow is pushed to the fork set by the empty budget"
        );
        assert_eq!(
            nodes[2].entered.get(),
            1,
            "it was still offered a (zero-byte) sink"
        );
    }

    #[test]
    fn a_panicking_encoder_is_caught_forked_and_named_rather_than_ending_the_walk() {
        // An escaping panic would poison the node's mutex and kill every
        // subsequent tick of that node for the process lifetime, reporting a tick panic
        // that never happened. The walk must also CONTINUE — the nodes after it are
        // innocent, and one fork covers them all anyway.
        let nodes = [
            Target::ok(4),
            Target::ok(4).with(Behaviour::Panic),
            Target::ok(4),
        ];
        let (stats, fork_set, parts) = walk(&nodes, 1024);

        assert_eq!(stats.unwound_nodes, 1);
        assert_eq!(fork_set, vec![(1, ForkReason::CaptureUnwound)]);
        assert_eq!(
            parts,
            vec![(0, 4), (2, 4)],
            "the walk must continue past a panicking node"
        );
        assert_eq!(stats.inline_nodes, 2);
        assert_eq!(stats.bytes_used, 8, "the panicking node wrote nothing");
    }

    #[test]
    fn an_encoder_error_is_distinguished_from_an_overflow() {
        // Both defer the node, and they need different operator lines: an overflow is a
        // node that will be served by the fork carrier every cadence, an encoder error
        // is a bug. Collapsing them would hide the second inside the first.
        let nodes = [Target::ok(4).with(Behaviour::Err)];
        let (stats, fork_set, parts) = walk(&nodes, 1024);
        assert_eq!(fork_set, vec![(0, ForkReason::EncoderError)]);
        assert_eq!(stats.unwound_nodes, 0, "an Err is not an unwind");
        assert!(parts.is_empty());
    }

    #[test]
    fn the_probe_names_the_first_contended_node_and_a_quiet_graph_names_none() {
        // A contended lock at this boundary means a FOREIGN holder, because the
        // graph's own threads provably hold none. The anchor is skipped whole rather
        // than attempted — forking into a held lock leaves the child holding it forever.
        let quiet = [Target::ok(4), Target::ok(4)];
        assert_eq!(probe_quiescent(&quiet), None);

        let contended = [
            Target::ok(4),
            Target::ok(4).contended(),
            Target::ok(4).contended(),
        ];
        assert_eq!(
            probe_quiescent(&contended),
            Some(1),
            "the FIRST contended node is what the manifest names"
        );
    }

    #[test]
    fn the_walk_reuses_the_callers_fork_set_rather_than_allocating_one() {
        // The walk runs at a step boundary, so it must allocate nothing. The contract
        // is that the caller's vector is CLEARED and refilled — a walk that appended
        // would grow it without bound across a run's boundaries.
        let mut arena = vec![0u8; 1024];
        let mut fork_set = vec![(99u32, ForkReason::EncoderError)];
        let nodes = [Target::gated()];
        walk_inline(&nodes, &mut arena, &mut fork_set, |_, _| {});
        assert_eq!(
            fork_set,
            vec![(0, ForkReason::NotInlineSafe)],
            "a stale entry from the previous boundary must not survive into this one"
        );
    }

    #[test]
    fn an_encoder_that_swallows_a_refusal_is_not_published_as_a_whole_part() {
        // A hand-written `impl CerulionState` can write past the grant, ignore the
        // `Err`, and return `Ok(())`. Trusting that return value would publish a
        // PARTIAL encoding as a complete part — a node's state silently truncated in
        // the bag, which is worse than no anchor for that node. Generated encoders
        // propagate with `?` and cannot reach this, which is exactly why the guard
        // needs its own arm: no other fixture in this file can see it.
        let nodes = [
            Target::ok(4),
            Target::ok(1000).with(Behaviour::OkAfterRefusal),
        ];
        let (stats, fork_set, parts) = walk(&nodes, 100);

        assert_eq!(
            parts,
            vec![(0, 4)],
            "the swallowing node must NOT publish a part, however cheerful its return"
        );
        assert_eq!(fork_set, vec![(1, ForkReason::ArenaOverflow)]);
        assert_eq!(stats.inline_nodes, 1);
        assert_eq!(
            stats.bytes_used, 100,
            "and its refused grant is still charged to the shared budget"
        );
    }

    #[test]
    fn an_empty_graph_walks_to_nothing() {
        let nodes: [Target; 0] = [];
        let (stats, fork_set, parts) = walk(&nodes, 64);
        assert_eq!(stats, InlineWalkStats::default());
        assert!(fork_set.is_empty());
        assert!(parts.is_empty());
    }
}
