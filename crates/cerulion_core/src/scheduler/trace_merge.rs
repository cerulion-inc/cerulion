// SPDX-License-Identifier: AGPL-3.0-only
//! Cross-process deterministic trace merge.
//!
//! When a graph is partitioned across processes (graph/partition.rs), each
//! per-process [`crate::graph::GraphRuntime`] produces a flat
//! `Vec<TraceEntry>` in EMISSION order, where every entry already carries the
//! logical [`TraceEntry::step`] it fired in and the GLOBAL DAG
//! [`TraceEntry::global_level`] it fired at (both stamped deterministically by
//! the levelized executor, both INCLUDED in `TraceEntry`'s `PartialEq`/`Eq`).
//!
//! [`merge_partition_traces`] stitches those per-process traces back into the
//! single global fire sequence a monolith run would have produced. It is the
//! productized replacement for the positional hand-merge that used to live in
//! `tests/barrier_level_gate_iox2_test.rs` (the now-deleted `tag_trace`/
//! `merge_traces`), which reconstructed each fire's global level POSITIONALLY
//! (`i / per_step`) and therefore only worked for single-node-per-level graphs
//! that fired EVERY level EVERY step. This function instead reads
//! `step`/`global_level` straight off each entry, so it is correct for
//! multi-node levels and conditional (non-every-step) fires.
//!
//! The merge is a PURE function (no transport, no clock, no I/O): it sorts by a
//! TOTAL-ordered 4-tuple, so the result is deterministic and therefore
//! REPLAY-STABLE (Principle #7) — the actual moat.

use crate::scheduler::handle::TraceEntry;

/// One process's contribution to the cross-process trace merge: its
/// `rank` (from [`crate::graph::partition::ProcessGroup::rank`]) and its full
/// per-process fire trace in EMISSION ORDER (exactly what
/// `GraphRuntime::trace()` returns).
///
/// Borrows the trace rather than owning it — the caller keeps each
/// `GraphRuntime`'s trace alive across the merge and pays no copy until the
/// merged `Vec` is built.
#[derive(Debug, Clone, Copy)]
pub struct ProcessTrace<'a> {
    /// The process's cross-process rank — unique per process, so it is a
    /// valid (and load-bearing) tiebreaker between two processes that own the
    /// SAME global level (a wide split).
    pub rank: usize,
    /// The process's fire trace in EMISSION order. The entry's index within
    /// this slice is the within-`(step, global_level, rank)` `seq` tiebreaker.
    pub trace: &'a [TraceEntry],
}

/// Merge per-process partitioned traces into the single global fire sequence.
///
/// Sorts every fire by `(step, global_level, rank, seq)` where `seq` is the
/// entry's index within its own process trace (the within-`(step, level,
/// rank)` tiebreaker, DERIVED here from emission position — never reconstructed
/// positionally from a per-step modulus). This 4-tuple is a TOTAL order (`rank`
/// is unique per process, `seq` unique within a process), so the result is
/// deterministic regardless of sort stability — and therefore REPLAY-STABLE
/// (Principle #7), the actual moat.
///
/// # Contract
///
/// - **Multi-node levels + conditional fires.** Correct for a global level
///   with ≥2 nodes and for nodes that fire only on some steps — unlike the
///   positional reconstruction it replaces (which assumes one node per level
///   firing every step).
/// - **`step` is the primary key, NOT `fire_time_ns`.** Keying on the logical
///   step counter (not the clock) is what keeps the merge correct for a `Period`
///   catch-up burst: a sub-step catch-up fires at a `fire_time_ns` BEFORE the
///   step's `current_time_ns`, so a `fire_time_ns`-keyed sort would float it
///   ahead of an earlier level's fire in the SAME logical step. The roles are
///   distinct: `step` (primary) groups each logical step's fires together so the
///   secondary key, `global_level`, orders the levels WITHIN a step — neither
///   uses the misleading sub-step `fire_time_ns`.
/// - **Pipeline (rank-contiguous) split == monolith.** When each global level
///   is owned by exactly one process, the merged sequence equals the monolith
///   trace exactly.
/// - **Wide split == deterministic, not byte-equal to declaration order.** When
///   two processes own the SAME global level, the merge canonicalizes the
///   shared level as `[rank0's fires…, rank1's fires…]`. This equals the
///   monolith's graph-declaration order iff that order is rank-contiguous
///   within the shared level (which the auto-partitioner produces). Within-level
///   order is semantically irrelevant by DAG construction (there are no
///   intra-level edges), so the real requirement is that the merge be
///   DETERMINISTIC — which it is unconditionally — not that it be byte-equal to
///   the monolith's arbitrary declaration order.
/// - **Caller owns lockstep; beyond rank-uniqueness the merge VALIDATES NOTHING
///   about cross-process step alignment.** Apart from the always-on
///   rank-uniqueness precondition (see `# Errors`), the merge is a pure re-sort
///   of whatever entries it is handed — it cannot detect a cross-process
///   step-range desync, because a
///   fire dropped by a desynced peer is indistinguishable from a legitimately
///   conditional non-fire (Contract bullet 1). The CALLER owns lockstep
///   stepping — every process must call `step()` the same number of times
///   (guaranteed by the barrier model, which gates level advance across
///   processes) — and the CALLER provides the end-to-end oracle that proves
///   alignment (e.g. the monolith firewall in
///   `tests/barrier_level_gate_iox2_test.rs`). Without lockstep the `step`
///   primary key no longer names the same logical step across processes and the
///   merge silently produces a well-ordered but WRONG sequence. Lockstep is
///   necessary but not sufficient: because `step` is an ABSOLUTE counter that
///   `clear_trace` does NOT reset, the caller must also retain traces
///   symmetrically — an asymmetric `clear_trace()` (one partition clears its
///   warmup, another does not) leaves phantom early-step entries the merge would
///   silently interleave. The barrier model gives equal step COUNTS; the caller
///   owns equal trace RETENTION.
///
/// # Errors
/// Returns [`crate::error::TransportError`]`::GraphError` if the `ProcessTrace.rank`
/// values are not pairwise distinct. Rank is the load-bearing tiebreaker in the
/// `(step, global_level, rank, seq)` total order, so a duplicate would silently
/// degrade the merge to input-slice order. This is an ALWAYS-ON check (not a
/// `debug_assert!`), so a release-built cross-process caller cannot skip it.
// hot-path-alloc-ok-fn: cold: OFFLINE k-way merge of recorded per-rank traces, run once when a
// multi-process recording is assembled
pub fn merge_partition_traces(
    processes: &[ProcessTrace<'_>],
) -> crate::error::TransportResult<Vec<TraceEntry>> {
    // `rank` is the load-bearing tiebreaker in the `(step, global_level, rank,
    // seq)` total order, so it MUST be pairwise distinct across processes. We
    // validate that with an always-on `Err(TransportError::GraphError)` — NOT a
    // `debug_assert!`: this is the un-skippable replacement the prior
    // FOLLOWUP note anticipated. A release-build duplicate rank would
    // otherwise silently degrade the merge to input-slice order and make a
    // future `replay == live` self-compare falsely PASS (it would mis-order
    // replay AND live identically), so the check must run in EVERY build.
    let mut ranks: Vec<usize> = processes.iter().map(|p| p.rank).collect();
    ranks.sort_unstable();
    ranks.dedup();
    if ranks.len() != processes.len() {
        return Err(crate::error::TransportError::GraphError {
            reason: "merge_partition_traces: ProcessTrace.rank values must be pairwise distinct \
                     (they are the load-bearing tiebreaker in the (step, global_level, rank, seq) \
                     total order); duplicate ranks would silently degrade the merge to input-slice order"
                .to_string(),
        });
    }

    // Tag each entry with its total-order key `(step, global_level, rank, seq)`,
    // sort, then clone out the entries in sorted order. `seq` is the entry's
    // emission index within its own process trace.
    let mut tagged: Vec<(u64, usize, usize, usize, &TraceEntry)> = processes
        .iter()
        .flat_map(|p| {
            p.trace
                .iter()
                .enumerate()
                .map(move |(seq, entry)| (entry.step, entry.global_level, p.rank, seq, entry))
        })
        .collect();

    tagged.sort_by_key(|(step, global_level, rank, seq, _)| (*step, *global_level, *rank, *seq));

    Ok(tagged
        .into_iter()
        .map(|(.., entry)| entry.clone())
        .collect())
}
