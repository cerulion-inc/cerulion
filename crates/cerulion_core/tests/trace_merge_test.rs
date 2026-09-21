// SPDX-License-Identifier: AGPL-3.0-only
//! Pure (no-iceoryx2) oracle-vector tests for the
//! cross-process deterministic trace merge `merge_partition_traces`.
//!
//! Every assertion compares the merged node-id sequence against a HAND-WRITTEN
//! expected sequence (oracle-vector, NOT a self-compare of two merge calls) —
//! except `determinism_idempotent`, which is explicitly about idempotence.
//!
//! `TraceEntry` and `ProcessTrace` are constructed via struct literals here
//! (both are `pub`, non-`#[non_exhaustive]`), so these tests need no transport,
//! clock, or graph build.

use std::sync::Arc;

use cerulion_core::{merge_partition_traces, ProcessTrace, TraceEntry};

/// Build a `TraceEntry` for `node` firing in logical `step` at global level
/// `gl` with `fire_time_ns = t` (and `duration_ns: 0` — wall time is excluded
/// from `TraceEntry`'s `PartialEq`/`Eq` anyway).
fn e(node: &str, step: u64, gl: usize, t: u64) -> TraceEntry {
    TraceEntry {
        node_id: Arc::from(node),
        step,
        fire_time_ns: t,
        global_level: gl,
        duration_ns: 0,
        discarded: false,
    }
}

/// The merged fire sequence as a `Vec` of node-id `String`s (the oracle target).
fn ids(trace: &[TraceEntry]) -> Vec<String> {
    trace
        .iter()
        .map(|entry| entry.node_id.to_string())
        .collect()
}

/// 1. One process, already ordered by `(step, global_level)`: the merge
///    returns it unchanged (same node_id sequence).
#[test]
fn single_process_passthrough() {
    let trace = vec![
        e("a", 0, 0, 100),
        e("b", 0, 1, 200),
        e("c", 1, 0, 300),
        e("d", 1, 1, 400),
    ];
    let merged = merge_partition_traces(&[ProcessTrace {
        rank: 0,
        trace: &trace,
    }])
    .expect("ranks are distinct in this fixture");
    assert_eq!(ids(&merged), vec!["a", "b", "c", "d"]);
}

/// 2. Pipeline (rank-contiguous) split: P0 (rank 0) owns global levels 0,1;
///    P1 (rank 1) owns 2,3 — across 3 steps. The merged sequence must equal the
///    step-major, then-global_level monolith order. `fire_time_ns` is set
///    ADVERSARIALLY (all P0 fires earlier than all P1 fires) so that a merge
///    keyed on `fire_time_ns` instead of `step` would group all of P0 ahead of
///    all of P1 — the assertion proves `step` is the primary key.
#[test]
fn two_process_pipeline_interleave() {
    // P0 fire times 10..16 (small); P1 fire times 100..106 (large).
    let p0 = vec![
        e("n0", 0, 0, 10),
        e("n1", 0, 1, 11),
        e("n0", 1, 0, 12),
        e("n1", 1, 1, 13),
        e("n0", 2, 0, 14),
        e("n1", 2, 1, 15),
    ];
    let p1 = vec![
        e("n2", 0, 2, 100),
        e("n3", 0, 3, 101),
        e("n2", 1, 2, 102),
        e("n3", 1, 3, 103),
        e("n2", 2, 2, 104),
        e("n3", 2, 3, 105),
    ];
    let merged = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &p0,
        },
        ProcessTrace {
            rank: 1,
            trace: &p1,
        },
    ])
    .expect("ranks are distinct in this fixture");
    // Monolith order: step0 L0,L1,L2,L3 ; step1 L0,L1,L2,L3 ; step2 …
    let expected = vec![
        "n0", "n1", "n2", "n3", // step 0
        "n0", "n1", "n2", "n3", // step 1
        "n0", "n1", "n2", "n3", // step 2
    ];
    assert_eq!(ids(&merged), expected);
}

/// 3. Multi-node levels. Part A: a single process whose global level 1 fires
///    TWO nodes — they must stay in EMISSION (seq) order within the level. Part
///    B: the SAME level split across two processes at the same global level —
///    the merge canonicalizes it as `[rank0's fires…, rank1's fires…]` (the
///    case the positional reconstruction could not handle at all).
#[test]
fn multi_node_level() {
    // Part A — both nodes of level 1 live in one process.
    let p0 = vec![e("root", 0, 0, 0), e("a", 0, 1, 0), e("b", 0, 1, 0)];
    let merged = merge_partition_traces(&[ProcessTrace {
        rank: 0,
        trace: &p0,
    }])
    .expect("ranks are distinct in this fixture");
    assert_eq!(ids(&merged), vec!["root", "a", "b"]);

    // Part B — level 1 split: rank0 owns {a,b}, rank1 owns {c,d}.
    let p_ab = vec![e("a", 0, 1, 0), e("b", 0, 1, 0)];
    let p_cd = vec![e("c", 0, 1, 0), e("d", 0, 1, 0)];
    let merged = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &p_ab,
        },
        ProcessTrace {
            rank: 1,
            trace: &p_cd,
        },
    ])
    .expect("ranks are distinct in this fixture");
    assert_eq!(ids(&merged), vec!["a", "b", "c", "d"]);
}

/// 4. Conditional (non-every-step) fires. `cond` fires only on steps {0,2}
///    (a gap at step 1, so its trace is shorter); `always` fires every step at
///    level 0; `sink` (a second process) fires every step at level 2. The merge
///    must interleave them correctly by step — the positional `i % per_step`
///    reconstruction would mis-segment the shorter `cond` stream.
#[test]
fn conditional_fire_gaps() {
    // P0 emits: s0 L0 always, s0 L1 cond, s1 L0 always (cond SKIPPED),
    //           s2 L0 always, s2 L1 cond.
    let p0 = vec![
        e("always", 0, 0, 0),
        e("cond", 0, 1, 0),
        e("always", 1, 0, 0),
        e("always", 2, 0, 0),
        e("cond", 2, 1, 0),
    ];
    // P1 emits sink every step at level 2.
    let p1 = vec![e("sink", 0, 2, 0), e("sink", 1, 2, 0), e("sink", 2, 2, 0)];
    let merged = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &p0,
        },
        ProcessTrace {
            rank: 1,
            trace: &p1,
        },
    ])
    .expect("ranks are distinct in this fixture");
    // step0: always,cond,sink ; step1: always,sink (no cond) ; step2: always,cond,sink
    let expected = vec![
        "always", "cond", "sink", // step 0
        "always", "sink", // step 1 — cond's gap is honored
        "always", "cond", "sink", // step 2
    ];
    assert_eq!(ids(&merged), expected);
}

/// 5. `rank` is load-bearing in the sort key. Two processes own
///    the SAME global level (step0/level1). With rank0={a,b}, rank1={c,d} the
///    merge is `[a,b,c,d]`; SWAP the ranks (rank1={a,b}, rank0={c,d}) and the
///    merge becomes `[c,d,a,b]`. The result CHANGES when only the ranks flip —
///    a regression that dropped `rank` from the key would (under the stable
///    sort) keep slice order and produce `[a,b,c,d]` in BOTH calls, failing the
///    swapped assertion. Both calls pass `{a,b}` FIRST in the slice, so the
///    flipped output cannot come from slice order — only from `rank`.
#[test]
fn wide_split_rank_is_load_bearing() {
    let p_ab = vec![e("a", 0, 1, 0), e("b", 0, 1, 0)];
    let p_cd = vec![e("c", 0, 1, 0), e("d", 0, 1, 0)];

    // rank0 = {a,b}, rank1 = {c,d}  ->  [a,b,c,d]
    let merged = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &p_ab,
        },
        ProcessTrace {
            rank: 1,
            trace: &p_cd,
        },
    ])
    .expect("ranks are distinct in this fixture");
    assert_eq!(ids(&merged), vec!["a", "b", "c", "d"]);

    // SWAP ranks ({a,b} -> rank1, {c,d} -> rank0), keep {a,b} first in slice.
    let merged_swapped = merge_partition_traces(&[
        ProcessTrace {
            rank: 1,
            trace: &p_ab,
        },
        ProcessTrace {
            rank: 0,
            trace: &p_cd,
        },
    ])
    .expect("ranks are distinct in this fixture");
    assert_eq!(ids(&merged_swapped), vec!["c", "d", "a", "b"]);

    // Anti-tautology guard: the two orders are genuinely different, so the
    // rank flip really moved data (kills a "both happened to be equal" pass).
    assert_ne!(ids(&merged), ids(&merged_swapped));
}

/// 6. Determinism: merging the SAME inputs twice yields byte-identical
///    `Vec<TraceEntry>` (compared via `TraceEntry`'s replay-deterministic
///    `PartialEq`, which includes `step`/`global_level`, excludes wall time).
///    Also pinned against a hand oracle so it is not a pure self-compare of the
///    ordering itself.
#[test]
fn determinism_idempotent() {
    let p0 = vec![e("n0", 0, 0, 5), e("n1", 0, 1, 6), e("n0", 1, 0, 7)];
    let p1 = vec![e("n2", 0, 2, 8), e("n2", 1, 2, 9)];
    let procs = [
        ProcessTrace {
            rank: 0,
            trace: &p0,
        },
        ProcessTrace {
            rank: 1,
            trace: &p1,
        },
    ];

    let first = merge_partition_traces(&procs).expect("ranks are distinct in this fixture");
    let second = merge_partition_traces(&procs).expect("ranks are distinct in this fixture");

    // Idempotence: byte-identical across two runs.
    assert_eq!(first, second);
    // Hand oracle so the ordering is also pinned (step-major, then level).
    assert_eq!(ids(&first), vec!["n0", "n1", "n2", "n0", "n2"]);
}

/// 7. Empty inputs. No processes → empty vec; an empty trace contributes
///    nothing; all-empty → empty.
#[test]
fn empty() {
    // No processes at all.
    assert!(merge_partition_traces(&[]).expect("empty is ok").is_empty());

    // An empty trace alongside a real one contributes nothing.
    let empty_trace: Vec<TraceEntry> = Vec::new();
    let real = vec![e("x", 0, 0, 0), e("y", 1, 0, 0)];
    let merged = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &empty_trace,
        },
        ProcessTrace {
            rank: 1,
            trace: &real,
        },
    ])
    .expect("ranks are distinct in this fixture");
    assert_eq!(ids(&merged), vec!["x", "y"]);

    // All-empty processes → empty.
    let merged = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &empty_trace,
        },
        ProcessTrace {
            rank: 1,
            trace: &empty_trace,
        },
    ])
    .expect("ranks are distinct in this fixture");
    assert!(merged.is_empty());
}

/// 8. `global_level` is load-bearing as the secondary key, the
///    Period-catch-up PROXY. A single process where a HIGHER global level's
///    fires carry EARLIER `fire_time_ns` than a LOWER level's — exactly what a
///    `Period` catch-up burst produces (sub-step catch-up fires at a
///    `fire_time_ns` BEFORE the step's `current_time_ns`) — but emitted in level
///    order. The merge must order by `global_level`, NOT `fire_time_ns`.
///
///    This is the Period-correctness proxy: the documented claim is that `step`
///    is primary and `global_level` orders within a step so that a sub-step
///    catch-up does NOT float ahead of an earlier level's same-step fire. A
///    regression that swapped `global_level` for `fire_time_ns` as the secondary
///    key would yield `[hi@100, hi@300, lo@500]` = ["hi","hi","lo"] — WRONG.
///    The existing 7 fixtures all set `fire_time_ns` monotonic with
///    `global_level`, so they pass under that regression; this fixture inverts the
///    two and is the first to KILL it.
#[test]
fn within_step_global_level_orders_over_fire_time() {
    // Step 0: level-0 `lo` fires at current_time 500; a level-1 `Period` node
    // `hi` catches up at sub-step times 100, 300 (BEFORE lo) — but emitted AFTER
    // lo because emission is in level order.
    let p0 = vec![e("lo", 0, 0, 500), e("hi", 0, 1, 100), e("hi", 0, 1, 300)];
    let merged = merge_partition_traces(&[ProcessTrace {
        rank: 0,
        trace: &p0,
    }])
    .expect("ranks are distinct in this fixture");
    // `global_level` orders within the step → lo (L0) then both hi (L1).
    assert_eq!(ids(&merged), vec!["lo", "hi", "hi"]);
}

/// 9. `global_level` is load-bearing across RANKS too, via
///    NON-CONTIGUOUS level ownership. rank 0 owns global levels {0, 2}; rank 1
///    owns {1}. Emission order within each process is level order, so the
///    per-process `seq` does NOT encode the cross-process global order.
///
///    Only the FULL `(step, global_level, rank, seq)` key yields L0,L1,L2.
///    Dropping `global_level` → `(step, rank, seq)` would sort rank 0's two
///    fires (L0 seq0, L2 seq1) ahead of rank 1's (L1 seq0), giving
///    ["L0","L2","L1"] — WRONG. The pipeline fixture (#2) can't catch this
///    because there each rank owns a CONTIGUOUS level block, so `(rank, seq)`
///    happens to agree with `global_level`. This non-contiguous split is the
///    first to make them disagree.
#[test]
fn interleaved_level_ownership_orders_by_global_level() {
    let p0 = vec![e("L0", 0, 0, 0), e("L2", 0, 2, 0)]; // rank 0, seq 0,1
    let p1 = vec![e("L1", 0, 1, 0)]; // rank 1, seq 0
    let merged = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &p0,
        },
        ProcessTrace {
            rank: 1,
            trace: &p1,
        },
    ])
    .expect("ranks are distinct in this fixture");
    assert_eq!(ids(&merged), vec!["L0", "L1", "L2"]);
}

/// 10. `seq` tiebreaker is EMISSION order, not sorted-by-node_id. Two entries
///     sharing the same `(step, global_level, rank)` are emitted in REVERSE
///     alphabetical order (`b` then `a`); the merge must preserve EMISSION
///     (seq) order, NOT re-sort by node_id. A regression that keyed the merge on
///     node_id (instead of the emission-position `seq`) would yield ["a","b"].
#[test]
fn seq_tiebreaker_is_emission_order_not_node_id() {
    let p0 = vec![e("b", 0, 0, 0), e("a", 0, 0, 0)]; // emitted b then a
    let merged = merge_partition_traces(&[ProcessTrace {
        rank: 0,
        trace: &p0,
    }])
    .expect("ranks are distinct in this fixture");
    assert_eq!(ids(&merged), vec!["b", "a"]);
}

/// 11. Fix-1 guard — duplicate `rank`s return a LOUD, always-on `Err`. Two
///     `ProcessTrace`s sharing rank 0 violate the unique-rank precondition that
///     the `(step, global_level, rank, seq)` total order relies on. Unlike the
///     prior `debug_assert!` (which compiled out in release and let a duplicate
///     rank silently degrade the merge to input-slice order), the merge now
///     returns `Err(TransportError::GraphError)` in EVERY build — this test runs
///     and asserts in release too (no `#[cfg(debug_assertions)]`/`should_panic`).
///     Paired with `distinct_ranks_merge_ok` as the anti-tautology control.
#[test]
fn duplicate_rank_returns_loud_err() {
    let p0 = vec![e("a", 0, 0, 0)];
    let p1 = vec![e("b", 0, 1, 0)];
    // Both ProcessTraces share rank 0 — violates the unique-rank precondition.
    let err = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &p0,
        },
        ProcessTrace {
            rank: 0,
            trace: &p1,
        },
    ])
    .expect_err("duplicate ranks must be rejected with an Err, not silently merged");

    // `TransportError` is not `PartialEq`, so match the variant + substring-check
    // the reason rather than comparing the error by value.
    match err {
        cerulion_core::error::TransportError::GraphError { reason } => {
            assert!(
                reason.contains("pairwise distinct"),
                "reason must explain the pairwise-distinct contract, got: {reason}"
            );
        }
        other => panic!("expected TransportError::GraphError, got {other:?}"),
    }
}

/// 11b. Anti-tautology control for `duplicate_rank_returns_loud_err`: a
///      DISTINCT-rank input (ranks 0 and 1) returns `Ok` with the expected
///      merged order. Proves the rank check does not reject everything — the
///      apparatus actually moves.
#[test]
fn distinct_ranks_merge_ok() {
    let p0 = vec![e("a", 0, 0, 0)];
    let p1 = vec![e("b", 0, 1, 0)];
    let merged = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &p0,
        },
        ProcessTrace {
            rank: 1,
            trace: &p1,
        },
    ])
    .expect("distinct ranks (0 and 1) must merge Ok");
    assert_eq!(ids(&merged), vec!["a", "b"]);
}

/// 12. THREE-process (k-way, k ≥ 3) merge — the first ≥3-stream coverage. The
///     2-process fixtures above can't exercise a global level OWNED BY MORE THAN
///     ONE rank simultaneously across three streams. Here rank 0 owns global
///     levels {0, 2}; rank 1 AND rank 2 BOTH own global level 1, so WITHIN that
///     shared level the `rank` tiebreaker must order rank 1 ahead of rank 2.
///     `fire_time_ns` is set ADVERSARIALLY (rank 0 LATEST, rank 1 EARLIEST, rank 2
///     in the MIDDLE) so a merge keyed on `fire_time_ns` would scramble the order
///     entirely — proving the `(step, global_level, rank, seq)` key.
///
///     The `expected` below is a LITERAL hand-built oracle, NOT computed by
///     re-sorting in the test (no self-compare).
#[test]
fn three_process_distinct_ranks_merge_ok() {
    // rank 0 owns global levels 0 and 2, across 2 steps; fire times LARGEST.
    let p0 = vec![
        e("a0", 0, 0, 901), // step 0, global level 0
        e("a2", 0, 2, 902), // step 0, global level 2
        e("b0", 1, 0, 903), // step 1, global level 0
        e("b2", 1, 2, 904), // step 1, global level 2
    ];
    // rank 1 owns global level 1; fire times SMALLEST.
    let p1 = vec![
        e("c1", 0, 1, 100), // step 0, global level 1
        e("d1", 1, 1, 101), // step 1, global level 1
    ];
    // rank 2 ALSO owns global level 1 (shared with rank 1); fire times MIDDLE.
    let p2 = vec![
        e("e1", 0, 1, 500), // step 0, global level 1
        e("f1", 1, 1, 501), // step 1, global level 1
    ];
    let merged = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &p0,
        },
        ProcessTrace {
            rank: 1,
            trace: &p1,
        },
        ProcessTrace {
            rank: 2,
            trace: &p2,
        },
    ])
    .expect("ranks 0,1,2 are pairwise distinct");

    // LITERAL hand oracle — (step, global_level, rank, seq) order:
    //   step 0: L0 r0 a0 ; L1 r1 c1 ; L1 r2 e1 ; L2 r0 a2
    //   step 1: L0 r0 b0 ; L1 r1 d1 ; L1 r2 f1 ; L2 r0 b2
    // (the rank tiebreaker puts rank1's c1/d1 before rank2's e1/f1 in shared L1).
    let expected = vec!["a0", "c1", "e1", "a2", "b0", "d1", "f1", "b2"];
    assert_eq!(ids(&merged), expected);
}

/// 13. THREE-process merge with a NON-ADJACENT duplicate rank `[0, 1, 0]` (the two
///     0s are separated by rank 1). The merge sort-then-dedups the ranks, so it
///     catches a collision that is NOT adjacent in the input slice.
///
///     THIS IS THE DISCRIMINATING TEST: the existing 2-process `[0, 0]` test
///     (`duplicate_rank_returns_loud_err`) would STILL pass an "adjacent-dedup-
///     without-sort" regression because its dups ARE adjacent; this `[0, 1, 0]`
///     layout would NOT (a naive `windows(2)` adjacency check sees `0!=1` then
///     `1!=0` and wrongly accepts).
#[test]
fn three_process_non_adjacent_duplicate_rank_returns_err() {
    let p0 = vec![e("a", 0, 0, 0)];
    let p1 = vec![e("b", 0, 1, 0)];
    let p2 = vec![e("c", 0, 2, 0)];
    // ranks [0, 1, 0] — the duplicate 0s are NON-ADJACENT (rank 1 sits between).
    let err = merge_partition_traces(&[
        ProcessTrace {
            rank: 0,
            trace: &p0,
        },
        ProcessTrace {
            rank: 1,
            trace: &p1,
        },
        ProcessTrace {
            rank: 0,
            trace: &p2,
        },
    ])
    .expect_err("a non-adjacent duplicate rank must still be rejected");

    match err {
        cerulion_core::error::TransportError::GraphError { reason } => {
            assert!(
                reason.contains("pairwise distinct"),
                "reason must explain the pairwise-distinct contract, got: {reason}"
            );
        }
        other => panic!("expected TransportError::GraphError, got {other:?}"),
    }
}
