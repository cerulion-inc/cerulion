// SPDX-License-Identifier: AGPL-3.0-only
//! The PURE, PORTABLE multi-process deployment planner.
//!
//! Given a full [`GraphConfig`] whose `process_groups:` is declared, this
//! module produces a deterministic [`DeploymentPlan`] — one [`WorkerPlan`] per
//! process group — that the spawner (iceoryx2 `Config` minting,
//! `MappedBarrier` creation, child-process fork/exec, READY-sentinel handshake)
//! consumes verbatim. This module is PLANNING ONLY: pure functions + serde
//! types + unit tests, no iceoryx2 / SHM / spawn / process / barrier runtime.
//!
//! # The GLOBAL-levels crux
//!
//! The cross-process barrier rendezvouses every worker at every
//! GLOBAL DAG-level boundary so all processes advance in lockstep (Principle
//! #7: replay = live). Each worker therefore carries a `global_level_map`
//! defined against the ONE global levelization of the FULL graph — NOT a
//! per-group re-levelization. The planner takes that global [`Levels`] and
//! feeds it straight to [`derive_process_groups`], which mints each group's
//! participant-map.
//!
//! # Why `levels` + `tightest_timing_ns` are INPUTS
//!
//! Building the correct global [`Levels`] needs the trigger-edge
//! classification (which `#[input]`s are triggering vs latest-value reads) —
//! that lives in each node's macro-declared `NodeInfo`, read over the cdylib
//! FFI boundary at graph build, NOT in the YAML. Likewise the graph's tightest
//! timing (`Scheduler::tightest_timing_ns`) reads macro-declared `period_ms` /
//! `tick_within_ms` / `expect_within_ms` / `promise_within_ms`, none of which
//! are in the YAML. Both are therefore IMPURE to derive.
//!
//! To keep this planning core pure and portable (it runs and unit-tests on
//! macOS with zero iceoryx2), the caller builds the full-graph
//! runtime ONCE, extracts `runtime.tightest_timing_ns()` + the global
//! `Levels`, and hands both here. The field docs below say where the spawner
//! does the runtime wiring and the iceoryx2 `Config` minting.

use cerulion_core::credit::ProducerSlot;
use std::collections::HashSet;

use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

use cerulion_core::error::{TransportError, TransportResult};
use cerulion_core::graph::config::GraphConfig;
use cerulion_core::graph::partition::{derive_process_groups, ProcessGroup};
use cerulion_core::graph::topology::Levels;
use cerulion_core::graph::{resolve_output_topic, resolve_source};
// `FileName` + the `SemanticString` trait (its `new` /
// `push_bytes` are trait methods) reach us via the iceoryx2 prelude — both are
// re-exported there (`iceoryx2::prelude::{FileName, SemanticString}`), so
// minting the SHM-namespace `Config::global.prefix` needs no new dependency
// beyond the `iceoryx2 = "=0.9.1"` this crate already pins.
use iceoryx2::prelude::{FileName, SemanticString};

/// The handed-quantum floor: 1ms in nanoseconds.
///
/// The global gating quantum is `tightest_timing_ns.unwrap_or(FLOOR).max(FLOOR)`
/// — exactly the derivation `build_live_deterministic_with_manager_and_barrier`
/// applies to the supervisor-handed quantum in
/// `cerulion_core::graph::runtime`. A graph with no declared timing (all
/// data-triggered) falls back to this 1ms heartbeat floor.
pub const HANDED_QUANTUM_FLOOR_NS: u64 = 1_000_000;

/// Per-worker READY budget (ms): the max the SUPERVISOR waits for ONE spawned
/// worker to signal READY before killing the deployment. The supervisor's
/// `READY_DEADLINE` (graph_cmd) derives from this constant — SINGLE SOURCE —
/// because the same figure is folded into every worker's
/// [`WorkerPlan::go_deadline_ms`]: the supervisor spawns workers SEQUENTIALLY,
/// READY-gated, so the FIRST-spawned worker legitimately waits up to
/// `(n-1) * READY_BUDGET_MS` between its own READY and GO. If the supervisor's
/// per-worker deadline and this budget drifted apart, a slow-but-legal
/// deployment could pass every READY gate yet blow an early worker's GO
/// deadline (a spurious fail-loud).
pub const READY_BUDGET_MS: u64 = 60_000;

/// Base GO-wait headroom (ms) layered ON TOP of the sequential-spawn READY
/// budget in [`WorkerPlan::go_deadline_ms`] — covers the supervisor's own
/// post-READY work (GO-sentinel write, filesystem + scheduling noise) so the
/// composed deadline `GO_BASE_MS + n * READY_BUDGET_MS` strictly dominates the
/// worst legitimate wait.
pub const GO_BASE_MS: u64 = 120_000;

/// One rank's wedge-page binding — the SHM tag and the slot order.
///
/// The two travel together because the tag alone is unusable: a slot index means
/// a node only under an agreed ordering, and the supervisor (which reads the page
/// and names the offender) is the side that owns it. See
/// [`WorkerPlan::wedge_page`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct WedgePagePlan {
    /// The deployment tag the page is named by, with this worker's `rank`.
    pub tag: String,
    /// The node ids in SLOT ORDER: `nodes[i]` occupies slot `i`.
    ///
    /// A `Vec` rather than a map because the ordering IS the contract, and a map
    /// would let the two sides agree on the pairs while disagreeing on which slot
    /// a node sits in — the same distinction that makes this field's order
    /// load-bearing rather than incidental.
    pub nodes: Vec<String>,
}

/// How the workers of ONE multi-process deployment coordinate
/// their steps — the plan-field SWITCH the worker branches on.
///
/// `Lockstep` is the shipped shape: every rank advances the same handed
/// quantum per step behind the shared `MappedBarrier`. `FreeRun` is the
/// flow-mode substrate: no barrier, no handed GATING quantum (the
/// plan still carries `handed_quantum_ns`; the capture-plane cadence reads it
/// on both routes), each rank on its own wall-following clock.
/// The DEFAULT is `Lockstep` on
/// every route; a `FreeRun` plan exists only when the
/// supervisor was OPTED IN (`graph_cmd::resolve_run_execution_mode`), so
/// nothing a user runs today changes behaviour.
///
/// `#[serde(rename_all = "snake_case")]` so the plan file and the bag's
/// `coordination` stamp spell the two modes the same way (`lockstep` /
/// `free_run` — the `replay_engine::CoordinationMode` vocabulary); the two
/// enums are kept DISTINCT because one is an execution decision and the other
/// is a recording contract, and `resolve_run_coordination` is the ONE map
/// between them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExecutionMode {
    /// Barrier lockstep (the shipped default): a shared `MappedBarrier`
    /// gates every global-level advance and every rank's gating clock
    /// advances by the same handed quantum.
    #[default]
    Lockstep,
    /// Free-run: no barrier, no handed GATING quantum — the non-record worker
    /// runs on the real clock, the `--record` worker on a controlled clock
    /// that is placed at a shared `real_ns()` epoch and then follows the wall.
    FreeRun,
}

/// One CROSS-PROCESS `block` edge's credit-word binding —
/// the identity the supervisor mints a `MappedCredit` segment under, plus the
/// ranks on either side of it.
///
/// The SHAPE lands with the switch (this commit) so that the plan file's
/// vocabulary is complete before anything reads it; the supervisor mints the
/// entries (from the LOADED planning topology — `ConsumerEdge.depth`, never
/// a source-parsed or defaulted depth) and the worker opens the words in the
/// credit-wiring commit. Until then every plan carries an EMPTY list, which
/// is exactly the earlier behaviour (no cross-process `block` edge can
/// run at all — the refusal still stands).
///
/// `depth` is `u32` because that is what `MappedCredit::create_owned` stamps
/// into the word; converting at the plan boundary keeps the one-recipe
/// discipline (the producer's defer-read and the consumer's ceiling read the
/// SAME number out of the SAME word).
///
/// `deny_unknown_fields`: an entry is minted by the supervisor and read by
/// the worker it just spawned — same binary, same run — so an unknown key here
/// is a skew between the two halves of ONE build, and the loud refusal is the
/// right answer (the enclosing `WorkerPlan` stays LENIENT at its top level for
/// the additive-field contract; that leniency does not extend into an entry).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CreditEdgePlan {
    /// The resolved ABSOLUTE topic name.
    pub topic: String,
    /// The consuming node id.
    pub consumer_node: String,
    /// The consuming input name.
    pub consumer_input: String,
    /// The consumer's declared `#[input(depth = N)]` ceiling, from the loaded
    /// planning topology.
    pub depth: u32,
    /// The rank(s) owning the producer. The single-producer fence means exactly
    /// one entry; the `Vec` pre-fits the CAS extension.
    pub producer_ranks: Vec<usize>,
    /// The rank owning the consumer.
    pub consumer_rank: usize,
}

/// Mint the deployment's [`CreditEdgePlan`] set — one entry per
/// `block` edge whose producer and consumer land in DIFFERENT ranks.
///
/// PURE (topology + a node→rank map in, plans out), so the whole mint is
/// oracle-testable without a supervisor, a transport or a filesystem.
///
/// # The three conditions, and why each one is a fence rather than a filter
///
/// An entry is minted only for a flow that satisfies ALL of:
///
/// * **exactly one producer.** A multi-producer credit edge is unserializable
///   at decide time — the fused-decide argument cannot cross processes
///   — and the sound fix is a CAS reservation this scope fence
///   deliberately does not build. Minting one anyway would hand a producer a
///   claim another producer can consume between the read and the publish, i.e.
///   silent overshoot on an edge declared LOSSLESS (Principle #6).
/// * **every consumer on the topic is `block`.** A mixed topic already degrades
///   its `block` consumers to `drop_oldest` (deferring the producer would
///   starve the non-block sibling), so a word there would describe an
///   occupancy nothing gates on.
/// * **the producer's rank differs from the consumer's.** A co-located edge
///   keeps its process-local word: zero syscalls, zero `/dev/shm` objects, and
///   the identical arithmetic.
///
/// # Provenance of `depth`
///
/// `ConsumerEdge.depth` from the LOADED planning topology — never a
/// source-parsed value and never `DEFAULT_CONSUMER_DEPTH`. The word STAMPS this
/// number and both sides read it back out of the word, so a depth ABOVE the
/// consumer's real declaration would make the producer defer LATE and the
/// consumer's behaviourally-real queue EVICT on a lossless edge.
///
/// # Refusals
///
/// A depth of 0, or one that does not fit `u32`, is an `Err` naming the edge.
/// Both are structurally unreachable — `GraphTopology::validate` enforces
/// `1 <= depth <= MAX_CONSUMER_DEPTH` on every build path — which is exactly
/// why they are refused rather than clamped: a value arriving here outside that
/// range means the topology this was minted from is not the one the run
/// validated, and `MappedCredit::create_owned` would either refuse (0) or stamp
/// a truncated ceiling (overflow) one layer down, where the diagnostic can no
/// longer name the graph edge.
///
/// A node in NEITHER of the map's ranks (an unplaced node — `validate_partition`
/// reports that separately) is SKIPPED, not refused: this function's job is to
/// describe the edges a valid partition splits, not to re-adjudicate the
/// partition.
pub fn credit_edges_for(
    topo: &cerulion_core::graph::GraphTopology,
    node_rank: &std::collections::HashMap<&str, usize>,
) -> Result<Vec<CreditEdgePlan>, String> {
    let mut out: Vec<CreditEdgePlan> = Vec::new();
    for flow in topo.topics() {
        // The ONE body of the creditability rule
        // (`TopicFlow::credit_bar`), which `partition::validate_block_colocation`
        // also calls. Spelling the condition out here a second time is exactly
        // how the mint and the validator would drift — and drift in the
        // permissive direction means a partition PASSES plan-time validation
        // and then dies in the consumer's worker.
        if !flow.is_creditable() {
            continue;
        }
        let Some(&producer_rank) = node_rank.get(flow.producers[0].as_str()) else {
            continue;
        };
        for edge in &flow.consumers {
            let Some(&consumer_rank) = node_rank.get(edge.node_id.as_str()) else {
                continue;
            };
            if producer_rank == consumer_rank {
                continue;
            }
            if edge.depth == 0 {
                return Err(format!(
                    "cross-process `block` edge '{}' -> '{}.{}' declares depth 0; a credit word \
                     stamped depth 0 reads FULL unconditionally, so its producer would defer \
                     entry to every tick forever. GraphTopology::validate enforces depth >= 1 on \
                     every build path, so this is a planning-topology bug",
                    flow.topic, edge.node_id, edge.input
                ));
            }
            let depth = u32::try_from(edge.depth).map_err(|_| {
                format!(
                    "cross-process `block` edge '{}' -> '{}.{}' declares depth {}, which does not \
                     fit the u32 a credit word stamps. GraphTopology::validate caps depth at \
                     MAX_CONSUMER_DEPTH, so this is a planning-topology bug",
                    flow.topic, edge.node_id, edge.input, edge.depth
                )
            })?;
            out.push(CreditEdgePlan {
                topic: flow.topic.clone(),
                consumer_node: edge.node_id.clone(),
                consumer_input: edge.input.clone(),
                depth,
                producer_ranks: vec![producer_rank],
                consumer_rank,
            });
        }
    }
    Ok(out)
}

/// The slot `dead_rank` occupies on `edge`, or `None` if it is not
/// one of that edge's producers.
///
/// The `parked` bitmask is PER EDGE, so the bit a dead producer left set is at
/// its position in that edge's producer list, NOT at its barrier rank. Every
/// edge this build mints has exactly one producer (the single-producer fence), so the
/// SLOT is normally 0 (the fence pins the slot, not the rank — a lone producer
/// can sit at any rank), so a confusion between them is usually invisible —
/// which is exactly why the distinction is a TYPE.
///
/// The type is [`cerulion_core::credit::ProducerSlot`], and it is
/// threaded all the way into `credit.rs` itself: `park_enter` / `park_exit` /
/// `clear_parked_producer` / `ParkedEdgeGuard::enter` take nothing else, so the
/// guarantee does not stop at this crate's boundary (none of those four
/// takes a bare `u32`).
///
/// A free function rather than an inherent `ProducerSlot::of`, because the type
/// now lives in another crate and [`CreditEdgePlan`] lives in this one — the
/// mint belongs beside the plan it reads.
///
/// The rank is taken as a `usize` — the type `producer_ranks` already holds —
/// so the COMPARISON is exact. An earlier shape compared `r as u32 == rank`,
/// which on a deployment with more than `u32::MAX` ranks would alias two
/// different ranks onto one slot; unreachable in practice, and free to remove.
///
/// The `.unwrap_or(u32::MAX)` on the RESULT still saturates rather than
/// truncating (mirroring `graph_cmd.rs`'s rank handling): a position past
/// `u32::MAX` cannot address a bit in a 32-bit mask anyway, and `as u32` would
/// wrap it onto a low bit belonging to a real producer. A saturated slot fails
/// `fits_mask`, so it degrades to the slice-timeout cadence rather than
/// stealing another producer's bit.
pub(crate) fn producer_slot_of(edge: &CreditEdgePlan, rank: usize) -> Option<ProducerSlot> {
    edge.producer_ranks
        .iter()
        .position(|&r| r == rank)
        .map(|i| ProducerSlot::new(u32::try_from(i).unwrap_or(u32::MAX)))
}

/// Which role a DEAD rank played on one credit edge, and
/// therefore what the supervisor owes that edge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum CreditDeathRole {
    /// The dead rank was the edge's CONSUMER, and at least one producer of
    /// that edge is NOT KNOWN DEAD. Those producers' credit word will never be
    /// drained again, so they are deferred at the credit gate on every tick
    /// from now on, publishing nothing on this topic.
    ///
    /// "not known dead" is the exact strength: a producer that dies between
    /// the emitting pass and the next sweep is named on a line already in the
    /// log. `CreditDeathWatch` emits a RETRACTION when that happens — it
    /// cannot unsay the first line, only correct it.
    ///
    /// Carries the SURVIVORS (the edge's producer ranks minus every rank known
    /// dead), because the warn names them and calling a dead rank "alive and
    /// healthy" is worse than saying nothing. A consumer death with NO
    /// surviving producer yields no action at all: the whole edge is gone, it
    /// stranded nobody, and the ordinary departure line already names both
    /// groups.
    Consumer { deferred_producers: Vec<usize> },
    /// The dead rank was a PRODUCER. If it died inside its park its
    /// `ParkedEdgeGuard::Drop` never ran, leaving its bit set in the word's
    /// `parked` mask forever; every freeing drain would then pay a no-op
    /// kernel wake for the rest of the run.
    ///
    /// A rank can in principle be both ends of one edge; the mint's
    /// rank-differs fence makes that unreachable today, and this arm and the
    /// `Consumer` arm are independent, so such an edge would yield both.
    ///
    /// The setter is LIVE as of the wake plumbing: a producer
    /// deferred on a mapped credit edge takes a `ParkedEdgeGuard` around its
    /// kernel block, so this sweep now clears a bit production really sets.
    /// (It shipped one PR AHEAD of that setter, deliberately, so the first
    /// parked producer met a supervisor that already cleaned up after it.)
    Producer { slot: ProducerSlot },
}

/// One thing the supervisor must do about one credit edge
/// because one rank died.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CreditDeathAction {
    /// Index into the deployment's watched edges.
    pub(crate) edge_idx: usize,
    pub(crate) dead_rank: u32,
    pub(crate) role: CreditDeathRole,
}

/// What a set of worker deaths means for the deployment's
/// cross-process `block` credit edges.
///
/// # Why this exists at all
///
/// A dead CONSUMER rank permanently wedges its surviving peer producers, and
/// the wedge alarm is blind to it BY CONSTRUCTION: the alarm reports a node that
/// entered a tick and did not return, while a credit-parked producer has not
/// entered a tick — it is deferred at the pre-fire gate, which is a normal
/// state the wedge alarm must not fire on. Without this, the loudest thing a
/// permanently-wedged deployment produces is the ordinary
/// `peer-loss=continue` "continuing degraded" line, which names the DEAD group
/// and says nothing about the SURVIVOR just silently retired.
///
/// A dead PRODUCER rank is the other direction — a stale `parked` bit.
///
/// # `dead_so_far` is CUMULATIVE, and that is the correctness point
///
/// The survivor set is the edge's producers minus every rank known dead — not
/// merely those in THIS batch. Both shapes are real:
///
/// * WITHIN one batch: `sweep_additional_deaths` folds every death in the pass
///   window, so a 2-rank deployment losing both ranks arrives as one call;
/// * ACROSS passes: the producer dies in pass 1 and the consumer in pass 2, so
///   this batch names a rank that died a moment ago.
///
/// Either way, naming a dead rank as a deferred survivor tells the operator to
/// go look at a process that no longer exists.
///
/// # Pure
///
/// Every decision is here; the caller only performs the effects. Deterministic:
/// edges in plan order, dead ranks in the order given.
pub(crate) fn credit_death_actions(
    edges: &[CreditEdgePlan],
    dead_ranks: &[u32],
    dead_so_far: &std::collections::BTreeSet<u32>,
) -> Vec<CreditDeathAction> {
    let mut out = Vec::new();
    for (edge_idx, edge) in edges.iter().enumerate() {
        for &dead_rank in dead_ranks {
            if edge.consumer_rank as u32 == dead_rank {
                // SURVIVORS only: every producer not known dead, in this batch
                // or any earlier one.
                let deferred_producers: Vec<usize> = edge
                    .producer_ranks
                    .iter()
                    .copied()
                    .filter(|r| {
                        let r32 = *r as u32;
                        !dead_so_far.contains(&r32) && !dead_ranks.contains(&r32)
                    })
                    .collect();
                // A fully-dead edge stranded nobody — SUPPRESSED.
                if !deferred_producers.is_empty() {
                    out.push(CreditDeathAction {
                        edge_idx,
                        dead_rank,
                        role: CreditDeathRole::Consumer { deferred_producers },
                    });
                }
            }
            if let Some(slot) = producer_slot_of(edge, dead_rank as usize) {
                out.push(CreditDeathAction {
                    edge_idx,
                    dead_rank,
                    role: CreditDeathRole::Producer { slot },
                });
            }
        }
    }
    out
}

/// One credit edge's IDENTITY — `(topic, consumer node, consumer input)`,
/// which is also what `credit_edge_id` hashes into the SHM object name.
///
/// The reconciliation compares KEYS, never whole plans, and that is
/// deliberate: `source_entry_infos` cannot know a node's declared `depth` (it
/// parses the source, where the macro attribute may be absent) and stamps
/// `DEFAULT_CONSUMER_DEPTH`, so a full-plan equality would refuse EVERY
/// deployment whose `block` input declares a non-default depth. The question
/// the reconciliation asks is "do the two views agree on WHICH edges get a
/// word", and the depth that ends up in the word is the LOADED one by design,
/// pinned by
/// `a_depth_that_differs_between_the_two_views_is_not_a_drift_refusal`.
type CreditEdgeKey = (String, String, String);

fn credit_edge_keys(edges: &[CreditEdgePlan]) -> std::collections::BTreeSet<CreditEdgeKey> {
    edges
        .iter()
        .map(|e| {
            (
                e.topic.clone(),
                e.consumer_node.clone(),
                e.consumer_input.clone(),
            )
        })
        .collect()
}

/// Render a credit-edge list for an operator-facing log line (D1).
///
/// `credit_edges = 1` does not answer "did MY split edge get a word?", which
/// is the only question anyone asks this line. Deterministic: plan order.
pub fn render_credit_edge_list(edges: &[CreditEdgePlan]) -> String {
    edges
        .iter()
        .map(|e| format!("{} -> {}.{}", e.topic, e.consumer_node, e.consumer_input))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Reconcile the SOURCE-judged acceptance against the
/// LOADED-metadata mint, over two topologies built from the two infos maps.
///
/// Extracted from `graph_run_supervisor` because the wiring shipped
/// INERT-TESTABLE: every arm drove one topology through both halves, so a
/// deleted call or swapped arguments changed nothing any test could see.
/// `node_rank` is the caller's ONE map (the same one the mint used), and the
/// source infos are the caller's FIRST read — this adds no second parse and no
/// second rank derivation.
///
/// Returns `None` when the two agree.
pub(crate) fn reconcile_credit_edges(
    source_topo: &cerulion_core::graph::GraphTopology,
    loaded_topo: &cerulion_core::graph::GraphTopology,
    node_rank: &std::collections::HashMap<&str, usize>,
    graph: &str,
) -> Result<Option<String>, String> {
    let source = credit_edges_for(source_topo, node_rank)?;
    let loaded = credit_edges_for(loaded_topo, node_rank)?;
    Ok(credit_edge_drift_refusal(graph, &source, &loaded))
}

/// Refuse when the credit edges the plan-time check
/// ACCEPTED are not the ones the supervisor MINTED.
///
/// # Why the two can differ at all
///
/// `validate_partition` judges a hand-written `process_groups:` on
/// SOURCE-parsed metadata — so a partition is judgeable without
/// building anything — while the mint reads the LOADED cdylibs, because the
/// credit word must stamp the depth the run will really enforce. Both are the
/// right input for their own job, and a STALE cdylib makes them disagree about
/// a topic's `is_all_block()`.
///
/// Neither direction is safe to let through, and neither is loud on its own:
/// an edge accepted but not minted dies in the consumer's worker blaming an
/// external publisher that does not exist; an edge minted but not accepted was
/// never checked for the shapes the credit word cannot describe. So ANY set
/// difference refuses, before a worker is spawned.
///
/// Returns `None` when the two agree. Pure; deterministic (`BTreeSet` order).
pub fn credit_edge_drift_refusal(
    graph: &str,
    source_accepted: &[CreditEdgePlan],
    loaded_minted: &[CreditEdgePlan],
) -> Option<String> {
    let accepted = credit_edge_keys(source_accepted);
    let minted = credit_edge_keys(loaded_minted);
    if accepted == minted {
        return None;
    }
    let render = |set: &std::collections::BTreeSet<CreditEdgeKey>| {
        set.iter()
            .map(|(t, n, i)| format!("'{t}' -> '{n}.{i}'"))
            .collect::<Vec<_>>()
            .join(", ")
    };
    let accepted_only: std::collections::BTreeSet<CreditEdgeKey> =
        accepted.difference(&minted).cloned().collect();
    let minted_only: std::collections::BTreeSet<CreditEdgeKey> =
        minted.difference(&accepted).cloned().collect();
    let mut parts = Vec::new();
    if !accepted_only.is_empty() {
        parts.push(format!(
            "accepted at plan time but NOT minted: {} — the BUILT node's port metadata \
             disagrees with its SOURCE on {} (a stale build, or a raw-FFI node whose \
             `// CERULION:INFO_START` block drifted from its `cerulion_node_info()`), so \
             the partition was allowed to split an edge the run carries no credit word \
             for, and the consumer's worker would refuse to build, reported by the \
             supervisor as an opaque \"exited before signaling READY\"",
            render(&accepted_only),
            if accepted_only.len() == 1 {
                "it"
            } else {
                "them"
            }
        ));
    }
    if !minted_only.is_empty() {
        parts.push(format!(
            "minted but NOT accepted at plan time: {} — the BUILT cdylib declares `block` \
             where the SOURCE does not, so nothing checked this edge against the shapes a \
             credit word cannot describe",
            render(&minted_only)
        ));
    }
    Some(format!(
        "graph run (multi-process): graph '{graph}''s partition was validated against the \
         node SOURCES but its cross-process `block` credit words are minted from the BUILT \
         cdylibs, and the two disagree — {}. This is a STALE BUILD: rebuild the node(s) \
         (`cerulion node build <type>`, or `--release` to match the run) so the source and \
         the cdylib declare the same `#[input(backpressure = …)]`; a raw-FFI node \
         additionally needs its `// CERULION:INFO_START` marker kept in step with its \
         `cerulion_node_info()`. Refused before any worker was spawned",
        parts.join("; ")
    ))
}

/// The deterministic deployment plan for one process group.
///
/// Everything the spawner needs to launch ONE worker process: its
/// [`subgraph`](Self::subgraph) config, its cross-process `rank`, its barrier
/// participant-map (`global_level_map`), the shared handed quantum, and the
/// plain-string barrier / ready-sentinel identifiers.
///
/// Not `PartialEq`-derivable because [`GraphConfig`] is not `PartialEq`; tests
/// compare via serde round-trip + field-level assertions instead.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkerPlan {
    /// The declared process-group name (e.g. `"perception"`).
    pub group: String,
    /// The SUBGRAPH's resolved identity (`"{graph}_{group}"`, derived from the
    /// parent graph's file stem).
    ///
    /// It rides the plan rather than the subgraph because
    /// [`GraphConfig::identity`] is `#[serde(skip)]`: it describes how a config
    /// was LOADED, not the document, so it cannot survive this JSON hop on its
    /// own. `graph_run_worker` restores it onto `subgraph` before building, so
    /// a worker's `graph=` log fields name the run rather than `unnamed`.
    /// `#[serde(default)]` only so a plan written by another build still
    /// parses; the supervisor always stamps it.
    #[serde(default)]
    pub graph_identity: String,
    /// The cross-process rank (index in `process_group_order`, else the
    /// `process_groups` declaration order) — the barrier ordering + trace-merge
    /// tiebreaker. Mirrors [`ProcessGroup::rank`].
    pub rank: usize,
    /// The barrier participant-map against the GLOBAL levelization: length =
    /// the global level count. `Some(local)` at global level `g` iff this group
    /// owns ≥1 node at `g` (`local` increments per owned global level, ascending
    /// `g`); `None` where the group only rendezvouses. Mirrors
    /// [`ProcessGroup::global_level_map`]; the non-`None` entries are a
    /// strictly-increasing bijection onto `0..local_count` by construction.
    pub global_level_map: Vec<Option<usize>>,
    /// The per-global-level MID-LEVEL barrier flags — same length and
    /// same indexing as [`Self::global_level_map`], `true` on each global level
    /// carrying a same-level NON-TRIGGER edge this partition SPLITS across two
    /// process groups.
    ///
    /// A flagged level crosses TWO barrier generations per step (one between
    /// its snapshot and tick halves, then the ordinary end-of-level one), which
    /// is what restores the monolith's snapshot-before-any-tick guarantee across
    /// processes. Every other level crosses exactly one, so a graph with no
    /// split pair pays nothing — see
    /// [`crate::multiprocess::mid_level_barrier_flags`].
    ///
    /// **IDENTICAL on every worker, by construction.** The flags decide how many
    /// generations a level burns, so two workers disagreeing on one level
    /// desynchronise the shared generation counter for the rest of the run. The
    /// SUPERVISOR computes the vector once (from the LOADED cdylib metadata,
    /// which is what the run was levelized from) and stamps the same value into
    /// every plan — it is deliberately NOT derived per worker.
    ///
    /// `#[serde(default)]` so a plan file written before this field existed deserialises
    /// to the empty vector. That is NOT a silently-wrong default: an empty
    /// vector fails `install_barrier_participant`'s length check LOUDLY on a
    /// multi-level graph, and the supervisor and worker ship together.
    #[serde(default)]
    pub mid_level_barrier: Vec<bool>,
    /// The GLOBAL gating quantum (ns) — the tightest timing over the FULL graph,
    /// floored at [`HANDED_QUANTUM_FLOOR_NS`]. IDENTICAL across every worker
    /// (the whole point: all workers advance their gating clocks in lockstep).
    pub handed_quantum_ns: u64,
    /// The iceoryx2 node name for this worker: `cerulion_{graph}_{group}`.
    /// Distinguishes co-resident workers of the same graph on one SHM root.
    pub node_name: String,
    /// The per-deployment SHM NAMESPACE (`cerdep_{graph}_{nonce}`),
    /// historically named for the barrier because that was its only tenant.
    /// IDENTICAL across every worker in the deployment. The
    /// spawner maps this into a real `MappedBarrier` namespace + iceoryx2
    /// `Config`; here it is only a deterministic string.
    ///
    /// The cross-process `block` CREDIT words share this namespace
    /// (`MappedCredit::create_owned(&barrier_ns, &credit_edge_id(..), depth)`);
    /// the `/cer_bar_` / `/cer_crd_` family prefixes keep the two apart. The
    /// FIELD is deliberately NOT renamed — it is not `#[serde(default)]`, so a
    /// rename would refuse every plan file written by another build.
    pub barrier_ns: String,
    /// The shared barrier id within [`barrier_ns`](Self::barrier_ns)
    /// (`{graph}_levelgate`). IDENTICAL across every worker.
    pub barrier_id: String,
    /// This worker's READY-sentinel path
    /// (`cerulion_{graph}_{group}_{nonce}.ready`, a bare filename). The
    /// spawner roots this under the deployment's working directory and the
    /// worker touches it once its graph-owned handoff topics exist.
    pub ready_path: String,
    /// This group's SUBGRAPH — ONLY its own nodes (+ their verbatim wiring),
    /// with the parent `prefix` / `multi_publisher_topics` preserved so
    /// cross-group topic edges still resolve to the same absolute names, and
    /// `process_groups` / `process_group_order` cleared (a worker is a
    /// single-process monolith). This is the config the supervisor writes out
    /// and spawns the worker against.
    pub subgraph: GraphConfig,
    /// The serialized iceoryx2 [`Config`](iceoryx2::config::Config) (JSON) every
    /// worker of this deployment shares — the DEFAULT iceoryx2
    /// namespace, snapshotted ONCE by the SUPERVISOR from the host's resolved
    /// global config (`Config::global_config().clone()`; `iox2_` on a
    /// config-file-free host) so all co-resident workers land on ONE namespace,
    /// their cross-group pub/sub connect, AND `topic echo/hz` / cross-graph
    /// absolute topics / external publishers interoperate. IDENTICAL across every
    /// worker in the deployment.
    ///
    /// `plan_deployment` (the PURE planner) leaves this `String::new()` —
    /// minting a `Config` is IMPURE (it resolves the host's iceoryx2 global
    /// config), so the supervisor injects the real JSON here AFTER
    /// planning, right before it serializes each `WorkerPlan` to disk and
    /// spawns the worker. The WORKER (`graph_cmd::graph_run_worker`)
    /// errors LOUDLY if this is empty — a worker cannot initialize its shared
    /// transport without its Config.
    pub ix_config_json: String,
    /// The deployment-wide GO sentinel — ONE file shared by ALL workers of a
    /// deployment (the planner mints the bare filename
    /// `cerulion_{graph}_{nonce}.go`; the supervisor roots it under the plan
    /// directory, like [`ready_path`](Self::ready_path)). The worker blocks
    /// between writing its READY sentinel and entering `run_live` until this
    /// file appears — the supervisor touches it only once EVERY worker is
    /// built + READY, so no worker publishes before every worker's subscribers
    /// exist. Closes two startup races: (a) an early worker's first publishes
    /// racing a later worker's subscriber connect (run-to-run startup
    /// nondeterminism — Principle #7), and (b) a sequential-spawn gap exceeding
    /// the ~5s barrier boundary timeout spuriously poisoning an earlier worker
    /// blocked at its first level boundary.
    pub go_path: String,
    /// Max wall time (ms) the worker waits for the GO sentinel after signaling
    /// READY, set by the PLANNER to `GO_BASE_MS + n_workers * READY_BUDGET_MS`
    /// (the planner knows the group count — kept pure). The composition
    /// matters: the supervisor spawns SEQUENTIALLY (READY-gated, up to
    /// [`READY_BUDGET_MS`] per worker), so the FIRST-spawned worker's
    /// legitimate GO wait spans EVERY later sibling's READY window — a fixed
    /// deadline would spuriously fail-loud a >=4-group slow deployment.
    /// IDENTICAL across every worker of a deployment.
    pub go_deadline_ms: u64,
    /// Whether the worker's cdylib resolution prefers the `target/release`
    /// artifact (mirrors `graph run --release`). Injected by the supervisor
    /// post-plan (like [`ix_config_json`](Self::ix_config_json)) so every worker
    /// loads the SAME profile the supervisor's `graph run` selected. The pure
    /// planner sets `false`; the worker (`graph_run_worker`) threads this into
    /// `load_node_factories` instead of a hard-coded `false`.
    pub prefer_release: bool,
    /// The in-memory fire-trace ring cap (entries) every
    /// worker applies (`GraphRuntime::set_trace_limit`), stamped by the
    /// supervisor from `graph run --trace-limit` post-plan (like
    /// [`prefer_release`](Self::prefer_release); the pure planner sets the
    /// default). `#[serde(default)]` to the production default so an OLD plan
    /// file written before this field existed still deserializes (back-compat:
    /// absent key → `PRODUCTION_TRACE_LIMIT`). The worker still SKIPS the cap
    /// entirely under the `CERULION_MP_TRACE_DIR` diagnostic seam.
    #[serde(default = "default_trace_limit")]
    pub trace_limit: usize,
    /// The RESOLVED live-loop monitor-wait park flag every worker applies
    /// (`cerulion_core::MonitorWaitPolicy`), the multi-process latency fix.
    /// Stamped by the SUPERVISOR post-plan from the SAME resolver the monolith
    /// live arm uses (`graph_cmd::resolve_monitor_wait_policy` — flags + env +
    /// primitive probe, resolved ONCE on the deployment host), so every worker
    /// parks identically. `#[serde(default)]` (= `false` → the
    /// park-off behavior) so an OLD plan file without the field still
    /// deserializes. The pure planner leaves it `false`.
    #[serde(default)]
    pub monitor_wait: bool,
    /// The doorbell half of the resolved park policy (producer-side
    /// ringing plus the consumer registry — the cross-process publish wake).
    /// Same stamping / serde-default contract as
    /// [`monitor_wait`](Self::monitor_wait); `MonitorWaitPolicy::new` re-coerces
    /// the doorbell-implies-monitor-wait invariant at the worker.
    #[serde(default)]
    pub doorbell: bool,
    /// The user's explicit `--no-cpu-dma-lock` opt-out,
    /// stamped by the SUPERVISOR post-plan (`graph_cmd::stamp_cap_mode`) so a
    /// worker's (8.7) C-state cap honors the SAME `CpuDmaLockMode` the monolith
    /// arms honor — without it a worker would default to `Auto` and silently ignore the
    /// flag. `#[serde(default)]` (= `false` → `Auto`) so
    /// an OLD plan file without the field still deserializes. The pure planner
    /// leaves it `false`.
    #[serde(default)]
    pub cap_disabled: bool,
    /// The coordination this worker runs under — see
    /// [`ExecutionMode`]. Stamped by the SUPERVISOR post-plan
    /// (`graph_cmd::stamp_execution_mode`) from the ONE resolution point
    /// (`graph_cmd::resolve_run_execution_mode`), the same way
    /// [`monitor_wait`](Self::monitor_wait) carries a resolved `graph run`
    /// decision; the pure planner leaves the default.
    ///
    /// `#[serde(default)]` = `Lockstep`, and that default is the whole
    /// mergeability argument: an OLD plan file, a plan from a supervisor that
    /// never stamped it, and every un-opted-in run all read as the shipped
    /// barrier-lockstep worker — the `mid_level_barrier` additive-field
    /// precedent. The worker resolves this field ONCE into its
    /// `WorkerBuildPath` and branches on that at six places (clock mint,
    /// barrier open, build call, recording configuration, epoch arming, cohort
    /// leave); the supervisor branches on it too (no shared barrier is created
    /// and no dead-peer cohort repair runs for a free-run deployment).
    #[serde(default)]
    pub execution_mode: ExecutionMode,
    /// The per-topic iceoryx2 provisioning REQUIREMENTS the SUPERVISOR
    /// harvested from its FULL-graph planning build
    /// (`GraphRuntime::topic_requirements`) and stamped here post-plan
    /// (`graph_cmd::stamp_topic_requirements`) — min borrowed-samples / buffer /
    /// subscriber count / event-listener slots per topic, unioned over ALL
    /// process groups' consumers.
    ///
    /// # Why this exists (the split loses the monolith's cross-group union)
    ///
    /// A monolith build provisions each topic's service by reducing over EVERY
    /// consumer in the whole graph. A `process_groups:` split runs each
    /// group in its own process against ONLY its subgraph, so the producer-owning
    /// worker never sees a consumer in another group — e.g. a non-trigger
    /// latest-value (snapshot) input needing `subscriber_max_borrowed_samples >=
    /// 3`. It then creates the topic's service under-provisioned and refuses that
    /// cross-group consumer at open
    /// (`DoesNotSupportRequestedMinSubscriberBorrowedSamples`, also the buffer /
    /// subscriber-count axes). The producer-owning worker consumes THIS map
    /// (threaded into `build_live_deterministic_with_manager_and_barrier`) and
    /// provisions `max(local view, stamped union)` per owned topic.
    ///
    /// The PURE planner (`plan_deployment`) leaves this EMPTY — harvesting needs
    /// the impure full-graph build (like `ix_config_json` / `prefer_release`); the
    /// supervisor stamps it post-plan. `#[serde(default)]` = old-plan-file
    /// back-compat: an absent key deserializes to an empty map ⇒ current
    /// single-process-view behavior (a no-op union). Keyed by resolved absolute
    /// topic name; `BTreeMap` keeps the plan file byte-stable (deterministic).
    #[serde(default)]
    pub topic_requirements: std::collections::BTreeMap<String, cerulion_core::TopicRequirements>,
    /// The absolute topics this group CONSUMES that a SIBLING group of the same
    /// graph PRODUCES: its cross-group input edges, by topic.
    ///
    /// The subgraph split rewrites each of those inputs to the absolute topic and
    /// drops the producer from this worker's view, so to the worker's own
    /// validation the source is an in-prefix absolute name with no producer,
    /// which draws the "is this a typo?" warning on a graph the supervisor had
    /// just validated. Only the supervisor holds the whole graph, so it names
    /// the topics here and the worker hands them to validation
    /// (`ValidationOptions::sibling_topics`). A source NOT in this set still
    /// warns.
    ///
    /// Planned by the PURE planner (it needs only the config). `#[serde(default)]`
    /// so a plan file without the field still parses; the empty set it yields
    /// restores the warning, which is the loud direction.
    #[serde(default)]
    pub sibling_topics: std::collections::BTreeSet<String>,
    /// The cross-process `block` edges this deployment
    /// backs with a `MappedCredit` word — see [`CreditEdgePlan`]. EMPTY on
    /// every plan until the credit-wiring commit teaches the supervisor to
    /// mint them from the loaded planning topology; `#[serde(default)]` so an
    /// old plan file reads as "no credit-backed edge", which is the
    /// earlier behaviour exactly. Stamped on BOTH execution modes when
    /// it is stamped at all (the word is what makes a split
    /// `block` edge representable, independent of how the ranks coordinate).
    #[serde(default)]
    pub credit_edges: Vec<CreditEdgePlan>,
    /// Multi-process recording: the trace-ring tag this worker
    /// creates its scheduler-trace ring under
    /// (`cerulion_core::trace_ring::TraceRingOwner::create`), set by the
    /// SUPERVISOR post-plan when `graph run --record` is active (like
    /// [`ix_config_json`](Self::ix_config_json); the pure planner leaves it
    /// `None`). `Some(tag)` ⇒ the worker creates the ring — header rank =
    /// [`rank`](Self::rank), manifest = its SUBGRAPH node ids in config order —
    /// BEFORE touching its READY sentinel (READY implies ring-exists: the
    /// supervisor opens every worker ring for `bagd` only after all READYs) and
    /// installs the minted producer on its scheduler. `None` ⇒ not recording —
    /// the single-process shape. `#[serde(default)]` (= `None`) so an
    /// OLD plan file written before this field existed still deserializes
    /// (back-compat: absent key → not recording).
    #[serde(default)]
    pub recording_ring: Option<String>,
    /// The scheduler-trace ring tag this worker creates its ring
    /// under on a run that is NOT recording — the ALWAYS-ON half of
    /// [`recording_ring`](Self::recording_ring).
    ///
    /// # Why this is a SECOND field and not a reuse of `recording_ring`
    ///
    /// Because `recording_ring` is not a tag — it is a MODE. Its `Some`/`None`
    /// is read as "am I recording" by the worker (`configure_recording_runtime_mp`
    /// flips Mode-B per-tick durations ON) and by the supervisor's recording
    /// bring-up (which builds bagd's `--ring` set from it and treats a missing
    /// tag as an internal error). Stamping it on every run would therefore turn
    /// per-tick `Instant` reads on for every node of every graph the machine
    /// runs — a hot-path cost nobody asked for — and would make "this plan
    /// records" unaskable.
    ///
    /// So the TAG and the MODE are split. At most ONE of the two is `Some` on
    /// any given plan: `--record` stamps `recording_ring` exactly as before, and
    /// every other multi-process run stamps this one. A worker holding this
    /// field creates the identical ring (same capacity, same header rank, same
    /// node/input/publisher manifest) and installs the identical producer — what
    /// it does NOT do is flip the recording runtime configuration.
    ///
    /// Creation failure is a DEGRADE here and a FAILURE under `recording_ring`,
    /// which is the same asymmetry for the same reason: a recording that
    /// silently loses its scheduler trace is a corrupt deliverable, while a
    /// serving graph that loses its black-box trace must still serve.
    ///
    /// `#[serde(default)]` (= `None`) so a plan file written by another build
    /// still deserializes — an absent key reads as "this run mints no trace
    /// ring", which is exactly the earlier behaviour.
    #[serde(default)]
    pub trace_ring: Option<String>,
    /// The checkpoint arm word's SHM tag this worker must
    /// open, or `None` when nothing is checkpointing the run.
    ///
    /// Stamped by the SUPERVISOR post-plan from the resolved
    /// `CERULION_STATE_ARM_TAG` (the pure planner leaves it `None`), on the
    /// `recording_ring` / `ix_config_json` precedent. It is stamped rather than
    /// re-read from the environment per worker because a `WorkerPlan` is
    /// self-contained by this struct's own doctrine, and because every rank must
    /// open the SAME word: two ranks resolving their own tags could clamp
    /// against two different onsets.
    ///
    /// An OLD plan file written before this field existed still deserializes,
    /// as `None` — absent key ⇒ not checkpointed, which is the earlier
    /// behaviour exactly. That back-compat is carried by the TYPE, not by the
    /// attribute: serde's derive resolves a missing field through
    /// `missing_field`, which succeeds for any `Option<T>`, so `#[serde(default)]`
    /// here is a no-op (MEASURED — removing it fails no test). It is kept for
    /// consistency with the sibling post-plan fields and because it states the
    /// intent at the declaration; the PROPERTY is pinned by
    /// `graph_cmd`'s `the_worker_plans_state_arm_tag_is_additive_in_both_directions`,
    /// which drives a key-less plan through a real deserialize.
    #[serde(default)]
    pub state_arm_tag: Option<String>,
    /// This run's DIRECTORY, so the worker can flock its own
    /// `worker-<rank>.lock` inside it before signalling READY.
    ///
    /// A worker needs it for exactly one reason, and it is a SAFETY one: nothing
    /// in this repo installs `PDEATHSIG`, so a SIGKILLed supervisor does not
    /// necessarily take its workers with it. A run directory can therefore hold
    /// a FREE `run.lock` beside a rank that is still stepping and still writing
    /// its rings, and a sweeper keying on the supervisor's lock alone would
    /// unlink a ring under a live writer. The worker's own lock is the only
    /// thing that can say "rank 3 is still here".
    ///
    /// The worker WRITES no `run.json` bytes — the manifest already truncates
    /// mid-rewrite, and N workers rewriting one document is a race with no
    /// upside. It creates and holds ONE empty file, never rewritten.
    ///
    /// `None` — an old plan file, or a run whose description could not be
    /// written at all (that write degrades rather than failing) — means the worker
    /// locks nothing, which is the earlier behaviour. Additive and
    /// `Option`, the [`state_arm_tag`](Self::state_arm_tag) precedent.
    #[serde(default)]
    pub run_dir: Option<String>,
    /// This rank's WEDGE PAGE — the tag its
    /// `cerulion_core::wedge_page::MappedWedgePage` is named by, and the SLOT
    /// ORDER its scheduler must bind its nodes to.
    ///
    /// `None` — an old plan file, a supervisor that could not create the page, a
    /// build where the alarm is unavailable — means this worker opens nothing and
    /// writes nothing, i.e. exactly the earlier behaviour. Additive and
    /// `Option`, the [`state_arm_tag`](Self::state_arm_tag) precedent.
    ///
    /// # Why the tag and the slot order are ONE field
    ///
    /// The supervisor is the process that READS the page, so it decides which
    /// slot means which node: the barrier-flag rule (the supervisor computes the
    /// mid-level barrier flags; a worker never re-derives them) applied to the
    /// same class of hazard. Deriving the order independently on both sides from
    /// `subgraph.nodes` would make a disagreement possible, and a disagreement
    /// here does not crash: the supervisor names node B while node A is wedged,
    /// which is a wrong answer from the feature whose entire job is to name the
    /// offender. Carrying them together means a tag can never arrive without the
    /// ordering it must be read with.
    #[serde(default)]
    pub wedge_page: Option<WedgePagePlan>,
}

/// The serde default for [`WorkerPlan::trace_limit`] — the single-source
/// production default (`graph_cmd::PRODUCTION_TRACE_LIMIT`).
fn default_trace_limit() -> usize {
    crate::graph_cmd::PRODUCTION_TRACE_LIMIT
}

/// The full multi-process deployment plan for a graph.
///
/// One [`WorkerPlan`] per process group (in rank order) plus the deployment-wide
/// shared barrier identifiers and the `expected` participant count the barrier
/// owner rendezvouses on.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeploymentPlan {
    /// One worker per process group, in rank order (`workers[i].rank == i`).
    pub workers: Vec<WorkerPlan>,
    /// The shared barrier namespace — echoed on every [`WorkerPlan::barrier_ns`].
    pub barrier_ns: String,
    /// The shared barrier id — echoed on every [`WorkerPlan::barrier_id`].
    pub barrier_id: String,
    /// The barrier participant count = the number of workers = the number of
    /// process groups. The spawner passes this to
    /// `MappedBarrier::create_owned(ns, id, expected)`.
    pub expected: usize,
}

/// Floor the global tightest timing to the handed quantum.
///
/// `tightest_timing_ns.unwrap_or(FLOOR).max(FLOOR)` — a graph with no declared
/// timing (`None`) uses the 1ms heartbeat floor, and any sub-ms value is
/// clamped up. Mirrors the runtime's handed-quantum derivation exactly.
///
/// The GLOBAL min-over-groups property is inherent in the input: the caller's
/// `Scheduler::tightest_timing_ns()` over the FULL graph is by definition the
/// MIN over all nodes, hence the min over all groups.
pub fn handed_quantum_ns(tightest_timing_ns: Option<u64>) -> u64 {
    tightest_timing_ns
        .unwrap_or(HANDED_QUANTUM_FLOOR_NS)
        .max(HANDED_QUANTUM_FLOOR_NS)
}

/// Reject a group that owns NON-ADJACENT global DAG levels.
///
/// The cross-process barrier's participant-map is a CONTIGUOUS-split index map:
/// each worker owns a contiguous BAND of global levels. The runtime's
/// `GraphRuntime::install_barrier_participant` enforces the exact contract
/// (`global_level_map`'s non-`None` entries must be a strictly-increasing
/// bijection onto `0..local_count`, where `local_count` is the SUBGRAPH's own
/// level count) at build time — but a plan-time check gives the user a clear
/// error at PLAN time instead of a deep runtime build failure.
///
/// # What this catches (and what it does NOT)
///
/// [`derive_process_groups`] ALWAYS emits a strictly-increasing contiguous
/// LOCAL index (`Some(0), Some(1), …`) by construction, so the runtime's
/// bijection-onto-`0..local_count` clause can only fail when the group's owned
/// global-level COUNT differs from the subgraph's own level count. The clearest
/// such misconfiguration is an INTERLEAVED partition — a group owning global
/// levels `{0, 2}` while another owns `{1}` — which surfaces here as a `None`
/// GAP between two `Some`s in `global_level_map`. This check is a PURE,
/// PLAN-TIME NECESSARY condition for a clean contiguous split; it is NOT
/// sufficient (a contiguous band whose bridge node lives in another group can
/// still collapse the subgraph's level count), so the runtime build remains the
/// full authority and still returns a clean `Err`.
///
/// # The BRIDGE axis is caught pre-spawn by `validate_partition`
///
/// This fn is the CHEAP, PORTABLE backstop covering ONLY the contiguity axis (a
/// gap between the first/last owned global level). The `graph run` supervisor
/// (`graph_cmd::graph_run_supervisor`) additionally runs the FULL
/// [`cerulion_core::graph::validate_partition`] (contiguity + BRIDGE re-
/// levelization) right after [`plan_deployment`] returns — BEFORE any worker is
/// spawned — so the bridge defect this check cannot see is refused at plan time
/// (not at the deep worker `install_barrier_participant` build). That validator
/// lives in the CALLER (not here) because it needs the impure `entry_infos` +
/// `trigger_edges` this pure planner deliberately does NOT take (see the module
/// docs on why `levels` is an input). Predicate relationship: `validate_partition`
/// ⊇ this check on the contiguity axis AND adds the bridge axis, so the two can
/// never drift — the backstop can never accept what the full validator rejects.
pub fn validate_contiguous_ownership(group: &ProcessGroup) -> TransportResult<()> {
    let map = &group.global_level_map;
    let first = map.iter().position(|o| o.is_some());
    let last = map.iter().rposition(|o| o.is_some());
    if let (Some(f), Some(l)) = (first, last) {
        if !map[f..=l].iter().all(|o| o.is_some()) {
            return Err(TransportError::GraphError {
                reason: format!(
                    "process group '{}' owns non-adjacent global DAG levels (its \
                     participant-map {:?} has a gap between owned levels {f} and {l}); \
                     the cross-process barrier only supports a CONTIGUOUS-split partition \
                     — give each group a contiguous band of the graph's pipeline stages \
                     (interleaved splits are not supported)",
                    group.name, map
                ),
            });
        }
    }
    Ok(())
}

/// The node metadata PARSED FROM SOURCE (`nodes/<type>/src/lib.rs`
/// via `parse_node_metadata`).
///
/// A newtype, not a bare `&IndexMap`, because
/// [`classify_trigger_metadata_drift`] takes BOTH provenances and a silent
/// swap would invert the entire point of the cross-check. Same discipline as
/// `MaxSliceLen` / `ListenerCountTiming`: make the wrong call a compile error
/// rather than a comment. It is deliberately absent from
/// [`classify_split_same_level_non_trigger_pairs`]'s signature — that seam
/// takes ONLY the loaded truth, so there is nothing there to swap.
#[derive(Debug, Clone, Copy)]
pub struct SourceNodeMetadata<'a>(pub &'a IndexMap<String, cerulion_core::NodeInfo>);

/// The node metadata the RUNTIME LOADED — each planning factory's
/// own `NodeEntry::info()`, i.e. what the BUILT cdylib actually declares.
///
/// This is the authority for anything compared against `runtime.levels()`,
/// because the planning runtime derived those levels from exactly these
/// `NodeInfo`s. See [`SourceNodeMetadata`] for why it is a newtype.
#[derive(Debug, Clone, Copy)]
pub struct LoadedNodeMetadata<'a>(pub &'a IndexMap<String, cerulion_core::NodeInfo>);

/// The policy-accurate spelling of workaround (a) for one reported
/// pair — "make the edge triggering" is NOT one gesture.
///
/// The macro's `validate_trigger_inference` rejects several of the spellings a
/// generic "add `#[input(trigger)]`" would send an operator to write, so a
/// single fixed remedy string would advertise a COMPILE ERROR on three of the
/// five reachable consumer policies. The variant is derived from the
/// consumer's own [`cerulion_core::MacroPolicy`] by
/// [`TriggerRemedy::for_policy`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerRemedy {
    /// `#[cerulion_node(period_ms = N)]` consumer. The macro REJECTS a trigger
    /// input combined with `period_ms`, so the fix is a POLICY change: drop
    /// `period_ms` and mark the input `#[input(trigger)]` (exactly the demo
    /// retarget).
    ReplacePeriodWithTrigger,
    /// `DataTrigger` consumer — it already has exactly ONE `#[input(trigger)]`
    /// field, and the macro REJECTS two trigger inputs without a sync policy,
    /// so the mark must be paired with `sync_window_ms = N` / `unbounded_sync`.
    MarkTriggerAndAddSyncWindow,
    /// `Sync { .. }` / `UnboundedSync` consumer — it already carries a sync
    /// policy, so marking one more input `#[input(trigger)]` is legal and
    /// simply widens the alignment set.
    MarkTrigger,
    /// `#[cerulion_node(external)]` consumer. There is NO trigger-mark remedy:
    /// the macro rejects a trigger input combined with `external`, and a
    /// data-triggered node is no longer a self-triggering ingress node.
    ExternalNoTriggerGesture,
    /// The consumer declares no macro policy at all (a closure entry, a
    /// raw-FFI node with no `"policy"` key). UNREACHABLE through the
    /// production seam — such a node falls back to `TriggerPolicy::Data`, so
    /// `build_trigger_edges` marks EVERY wired input triggering and guard 1
    /// excludes the pair. Present for totality, and pinned as such.
    NoDeclaredPolicy,
}

// The two workaround spellings moved to `cerulion_core::graph`, so
// this advisory and the plan-time `block` co-location REFUSAL (which lives in
// core, beside the partitioner) read ONE definition instead of two that can
// drift. The wording is byte-unchanged; only its home moved.
use cerulion_core::graph::{REMEDY_CO_LOCATE, REMEDY_SINGLE_PROCESS};

impl TriggerRemedy {
    /// Classify the consumer's remedy from its macro policy.
    ///
    /// Pure and total. Mirrors `cerulion_macros::validate`'s
    /// `validate_trigger_inference` — that function is the authority on which
    /// `#[input(trigger)]` spellings COMPILE, and this is the only place the
    /// warn's advice is allowed to disagree with it.
    pub fn for_policy(policy: Option<&cerulion_core::MacroPolicy>) -> Self {
        use cerulion_core::MacroPolicy as MP;
        match policy {
            Some(MP::Period { .. }) => Self::ReplacePeriodWithTrigger,
            Some(MP::DataTrigger { .. }) => Self::MarkTriggerAndAddSyncWindow,
            Some(MP::Sync { .. }) | Some(MP::UnboundedSync) => Self::MarkTrigger,
            Some(MP::External) => Self::ExternalNoTriggerGesture,
            None => Self::NoDeclaredPolicy,
        }
    }

    /// A short, stable, greppable token naming this remedy — the `remedy=`
    /// structured field on the warn line. Distinct per variant, so an operator
    /// (or a fleet log query) can count "how many of my splits need a policy
    /// change" without parsing prose.
    pub fn token(self) -> &'static str {
        match self {
            Self::ReplacePeriodWithTrigger => "replace_period_with_trigger",
            Self::MarkTriggerAndAddSyncWindow => "mark_trigger_and_add_sync_window",
            Self::MarkTrigger => "mark_trigger",
            Self::ExternalNoTriggerGesture => "co_locate_only_external",
            Self::NoDeclaredPolicy => "co_locate_only_no_policy",
        }
    }

    /// Render the whole `Fix by …` sentence for one finding.
    ///
    /// The two no-gesture variants enumerate only TWO workarounds and say why
    /// — an operator must not be handed an `(a)` that cannot compile, and a
    /// placeholder `(a) not available` slot would be worse than renumbering.
    pub fn fix_sentence(self, consumer: &str, input: &str) -> String {
        match self {
            Self::ReplacePeriodWithTrigger => format!(
                "Fix by (a) changing `{consumer}`'s node policy to DATA-TRIGGERED — drop \
                 `period_ms` from its `#[cerulion_node(..)]` attribute and mark `{input}` \
                 `#[input(trigger)]`, which levelizes the consumer below the producer (the \
                 macro REJECTS `#[input(trigger)]` combined with `period_ms`, so the mark \
                 alone will not compile), OR (b) {REMEDY_CO_LOCATE}, OR (c) \
                 {REMEDY_SINGLE_PROCESS}."
            ),
            Self::MarkTriggerAndAddSyncWindow => format!(
                "Fix by (a) adding `{input}` to `{consumer}`'s trigger set — mark it \
                 `#[input(trigger)]` AND add `#[cerulion_node(sync_window_ms = N)]` (or \
                 `unbounded_sync`), which levelizes the consumer below the producer (the \
                 macro REJECTS two `#[input(trigger)]` fields with no sync policy, so the \
                 mark alone will not compile), OR (b) {REMEDY_CO_LOCATE}, OR (c) \
                 {REMEDY_SINGLE_PROCESS}."
            ),
            Self::MarkTrigger => format!(
                "Fix by (a) marking `{input}` `#[input(trigger)]` on `{consumer}`, which adds \
                 it to that node's sync alignment set and levelizes the consumer below the \
                 producer, OR (b) {REMEDY_CO_LOCATE}, OR (c) {REMEDY_SINGLE_PROCESS}."
            ),
            Self::ExternalNoTriggerGesture => format!(
                "Fix by (a) {REMEDY_CO_LOCATE}, OR (b) {REMEDY_SINGLE_PROCESS} — there is NO \
                 `#[input(trigger)]` remedy here: `{consumer}` is a self-triggering \
                 `external` ingress node, the macro REJECTS `#[input(trigger)]` combined \
                 with `external`, and a data-triggered node is no longer an ingress node."
            ),
            Self::NoDeclaredPolicy => format!(
                "Fix by (a) {REMEDY_CO_LOCATE}, OR (b) {REMEDY_SINGLE_PROCESS} — there is NO \
                 `#[input(trigger)]` remedy here: `{consumer}` declares no macro trigger \
                 policy, so the runtime already fires it on ANY input arrival."
            ),
        }
    }
}

/// One same-level, non-trigger producer→consumer edge that the
/// partition puts in TWO different process groups.
///
/// See [`split_same_level_non_trigger_pairs`] for why this shape is
/// nondeterministic and [`report_split_same_level_non_trigger_pairs`] for what
/// the operator is told.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SplitNonTriggerPair {
    /// The resolved topic carrying the edge.
    pub topic: String,
    /// The producing node's id.
    pub producer: String,
    /// The process group owning the producer.
    pub producer_group: String,
    /// The consuming node's id.
    pub consumer: String,
    /// The consuming node's non-trigger `#[input]` field name.
    pub consumer_input: String,
    /// The process group owning the consumer.
    pub consumer_group: String,
    /// The global DAG level BOTH nodes occupy (the whole point — a shared
    /// level is exactly what the end-of-level barrier cannot order).
    pub level: usize,
    /// The policy-accurate spelling of workaround (a) for THIS consumer —
    /// derived from its own `MacroPolicy`, because a generic "mark the input
    /// `#[input(trigger)]`" is a COMPILE ERROR on a `period_ms`, `external`,
    /// or already-`DataTrigger` consumer.
    pub remedy: TriggerRemedy,
}

/// One consumer edge whose TRIGGER classification differs between
/// the node's SOURCE metadata and the metadata the runtime LOADED from the
/// built cdylib — i.e. a STALE BUILD (or a raw-FFI node whose
/// `// CERULION:INFO_START` marker has drifted from its real
/// `cerulion_node_info()`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TriggerClassificationDrift {
    /// The consuming node's id.
    pub consumer: String,
    /// The resolved topic carrying the edge.
    pub topic: String,
    /// What `nodes/<type>/src/lib.rs` says today.
    pub source_says_triggering: bool,
    /// What the BUILT cdylib says — the classification the planning runtime
    /// actually levelized with, and therefore the one this detector trusts.
    pub loaded_says_triggering: bool,
}

/// One consumer input whose declared BACKPRESSURE policy differs
/// between the node's SOURCE metadata and the metadata the runtime LOADED from
/// the built cdylib — i.e. a STALE BUILD (or a raw-FFI node whose
/// `// CERULION:INFO_START` marker has drifted from its real
/// `cerulion_node_info()`).
///
/// Its own type beside [`TriggerClassificationDrift`] rather than a field on
/// it, because the two disagreements have different GRAINS: a trigger mark is
/// a `(consumer, topic)` DAG-edge property (`TriggerEdges`' own key), while a
/// backpressure policy is declared PER INPUT — one node reading one topic
/// through two inputs can legitimately declare `block` on one and
/// `drop_oldest` on the other. Merging them would force one of the two onto
/// the wrong key.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BackpressureClassificationDrift {
    /// The consuming node's id.
    pub consumer: String,
    /// The consuming node's input field name.
    pub input: String,
    /// The resolved topic that input reads.
    pub topic: String,
    /// What `nodes/<type>/src/lib.rs` says today: the truth
    /// the auto-partitioner co-located (or did not co-locate) on.
    pub source_policy: cerulion_core::graph::node::BackpressurePolicy,
    /// What the BUILT cdylib says — the policy `GraphTopology::validate`
    /// enforces inside each worker.
    pub loaded_policy: cerulion_core::graph::node::BackpressurePolicy,
}

/// The topology both plan-time diagnostics walk, built from the
/// LOADED metadata — or `None`, logged and swallowed.
///
/// # Why this cannot be `?`
///
/// Both callers are DIAGNOSTICS, and one of them runs BEFORE the refusing
/// `validate_partition`. A `?` here would let an advisory decide which error
/// the operator sees — turning "your partition is invalid, here is the bridge
/// node" into "topology build failed" on the drift path, and aborting an
/// otherwise legal run on the pair path. A diagnostic must never wedge or
/// re-label the path it observes (the `lock_regime_latch` precedent).
///
/// UNREACHABLE in production: the planning `GraphRuntime::build_*` already ran
/// `GraphTopology::build` over this exact `config` + these exact LOADED infos
/// and succeeded, or the supervisor returned long before here. If it ever
/// fires, having NO findings is the correct answer — the diagnostics have
/// nothing they can correctly say — and `validate_partition`, which builds its
/// own view, will speak for itself.
fn diagnostic_topology(
    config: &GraphConfig,
    loaded_entry_infos: LoadedNodeMetadata<'_>,
) -> Option<cerulion_core::graph::GraphTopology> {
    match cerulion_core::graph::GraphTopology::build(config, loaded_entry_infos.0) {
        Ok(topology) => Some(topology),
        Err(error) => {
            tracing::debug!(
                graph = %config.identity(),
                error = ?error,
                "the plan-time diagnostics could not build a topology from the \
                 loaded node metadata, so they report nothing this run; the partition \
                 validator builds its own view and is unaffected"
            );
            None
        }
    }
}

/// Find every same-level non-trigger edge this partition SPLITS
/// across process groups.
///
/// # Why this shape is nondeterministic
///
/// A non-trigger (latest-value) `#[input]` is NOT a DAG edge — `derive_levels`
/// skips it — so a `period_ms` producer and a `period_ms` consumer joined only
/// by a plain `#[input]` share ONE global level. Within a single process that
/// is still safe: `run_level` drains, decides, snapshots ALL inputs, and only
/// then ticks ALL nodes, so the consumer provably reads the value as of
/// step-start. Across TWO process groups co-owning that level the guarantee is
/// gone: the cross-process barrier rendezvous sits at the level's END, so
/// nothing orders the producer's tick+publish against the consumer's
/// `snapshot_inputs`. OS scheduling decides the pairing, the bag records no
/// consumption edge that could reproduce it, and replay re-executes as a
/// monolith where the safe ordering applies — so the live run is
/// nondeterministic AND replay is structurally unable to match it.
///
/// # The predicate (all four must hold)
///
/// 1. the edge is NON-triggering — `trigger_edges` is the level DAG's OWN
///    predicate, so this asks exactly "did levelization decline to order
///    this?";
/// 2. producer and consumer sit at the SAME global level — a trigger edge, or
///    any other path that levelizes the consumer below the producer, is
///    ordered by the end-of-level barrier and is NOT reported;
/// 3. they are in DIFFERENT process groups — co-located nodes keep the
///    monolith's snapshot-before-tick guarantee (a self-edge is therefore
///    never reported, since a node is always in its own group);
/// 4. both nodes are actually placed and levelized.
///
/// # Provenance: every input must describe ONE build
///
/// `trigger_edges`, `entry_infos` and `levels` are three views of the same
/// per-node macro metadata, and mixing provenances silently inverts verdicts:
/// `levels` comes from the planning runtime, which levelized the LOADED cdylib
/// info, so a `trigger_edges` derived from SOURCE would (after an
/// `#[input(trigger)]` edit with no rebuild) classify an edge the runtime
/// ordered as unordered, and vice versa. Production callers must go through
/// [`classify_split_same_level_non_trigger_pairs`], which derives all three
/// from the LOADED metadata alone; this function stays parameterised so the
/// unit tests can feed deliberately-inconsistent inputs.
///
/// # Guards 1 and 2 are output-equivalent TODAY — stated, not hidden
///
/// `GraphTopology::level_invariant_violation` rejects any levelization in which
/// a triggering edge is not STRICTLY level-increasing, on both the derived
/// (Kahn) and the `level_assignments:` path. So today `is_triggering(edge)`
/// already implies `producer_level < consumer_level`, and guard 1 can never
/// change the result that guard 2 alone would give: no mutation of the shipped
/// code kills guard 1 through a *reachable* input, and the arm that pins it
/// (`..._a_triggering_edge_is_never_reported_even_at_one_level`) says so and
/// feeds inputs today's levelization refuses to mint.
///
/// It is kept because it is the SEMANTICALLY primary predicate — the question
/// is "did the level DAG decline to order this pair?", and `TriggerEdges` is
/// literally what `derive_levels` filters on. Dropping it would leave this
/// detector's correctness resting on a non-local invariant in another crate:
/// if that invariant were ever relaxed, a same-level trigger edge would start
/// being reported as unordered when the barrier does order it. Guard 2 is NOT
/// redundant in the other direction — a non-trigger edge whose consumer is
/// pushed to another level by some OTHER trigger edge is barrier-ordered and
/// is excluded only by guard 2.
///
/// # Purity + scope
///
/// Pure and total: no I/O, no transport, no spawn. It is an ADVISORY, not a
/// validator — it never refuses a run, which is why it lives here beside the
/// planner rather than in `cerulion_core::graph::partition` (the home of the
/// refusing `validate_partition`).
///
/// `TriggerEdges` keys on `(node, topic)`, not `(node, input)`. That is the
/// right grain here rather than a limitation: it is precisely the key
/// `derive_levels` filters on, so a node reading one topic through BOTH a
/// trigger and a plain input is levelized BELOW the producer and condition 2
/// excludes the pair — which is correct, because the barrier then orders it.
///
/// Returns pairs in deterministic order (topology topic order, then graph
/// order within a topic's consumers and producers).
pub fn split_same_level_non_trigger_pairs(
    topology: &cerulion_core::graph::GraphTopology,
    trigger_edges: &cerulion_core::graph::TriggerEdges,
    levels: &Levels,
    process_groups: &IndexMap<String, Vec<String>>,
    entry_infos: LoadedNodeMetadata<'_>,
) -> Vec<SplitNonTriggerPair> {
    // node -> owning group. `derive_process_groups` already rejects
    // double-assignment, so first-wins can only bind an already-refused graph.
    let mut group_of: IndexMap<&str, &str> = IndexMap::new();
    for (group, members) in process_groups {
        for member in members {
            group_of.entry(member.as_str()).or_insert(group.as_str());
        }
    }

    let mut found = Vec::new();
    for flow in topology.topics() {
        for consumer in &flow.consumers {
            // (1) a triggering edge IS a DAG edge — barrier-ordered.
            if trigger_edges.is_triggering(&consumer.node_id, &flow.topic) {
                continue;
            }
            let (Some(consumer_group), Some(consumer_level)) = (
                group_of.get(consumer.node_id.as_str()),
                levels.level_of(&consumer.node_id),
            ) else {
                continue; // (4) unplaced or unlevelized — nothing to claim.
            };
            // Workaround (a) depends on the CONSUMER's own macro policy: the
            // macro rejects `#[input(trigger)]` beside `period_ms`/`external`,
            // and rejects a second trigger input with no sync policy.
            let consumer_policy = entry_infos
                .0
                .get(consumer.node_id.as_str())
                .and_then(|i| i.policy());
            let remedy = TriggerRemedy::for_policy(consumer_policy.as_ref());
            for producer in &flow.producers {
                let (Some(producer_group), Some(producer_level)) =
                    (group_of.get(producer.as_str()), levels.level_of(producer))
                else {
                    continue; // (4)
                };
                // (3) co-located: the monolith ordering still holds.
                if producer_group == consumer_group {
                    continue;
                }
                // (2) different levels: the end-of-level barrier orders them.
                if producer_level != consumer_level {
                    continue;
                }
                found.push(SplitNonTriggerPair {
                    topic: flow.topic.clone(),
                    producer: producer.clone(),
                    producer_group: (*producer_group).to_string(),
                    consumer: consumer.node_id.clone(),
                    consumer_input: consumer.input.clone(),
                    consumer_group: (*consumer_group).to_string(),
                    level: consumer_level,
                    remedy,
                });
            }
        }
    }
    found
}

/// Every consumer edge the SOURCE metadata and the LOADED cdylib
/// metadata classify differently.
///
/// Pure. Both sides are produced by the SAME `build_trigger_edges`, so a
/// disagreement is never a classification-logic difference — it is evidence
/// that `nodes/<type>/src/lib.rs` has moved since the cdylib was built (or, on
/// a raw-FFI node, that the hand-written `// CERULION:INFO_START` marker no
/// longer matches its real `cerulion_node_info()`).
///
/// Grain is `(consumer, topic)` — the `TriggerEdges` key — walked in topology
/// order and deduplicated, so a node reading one topic through two inputs
/// reports the disagreement once.
pub fn trigger_classification_drift(
    topology: &cerulion_core::graph::GraphTopology,
    source_edges: &cerulion_core::graph::TriggerEdges,
    loaded_edges: &cerulion_core::graph::TriggerEdges,
) -> Vec<TriggerClassificationDrift> {
    let mut seen: HashSet<(&str, &str)> = HashSet::new();
    let mut found = Vec::new();
    for flow in topology.topics() {
        for consumer in &flow.consumers {
            let source_says = source_edges.is_triggering(&consumer.node_id, &flow.topic);
            let loaded_says = loaded_edges.is_triggering(&consumer.node_id, &flow.topic);
            if source_says == loaded_says {
                continue;
            }
            if !seen.insert((consumer.node_id.as_str(), flow.topic.as_str())) {
                continue;
            }
            found.push(TriggerClassificationDrift {
                consumer: consumer.node_id.clone(),
                topic: flow.topic.clone(),
                source_says_triggering: source_says,
                loaded_says_triggering: loaded_says,
            });
        }
    }
    found
}

/// The production entry point for the STALE-BUILD cross-check —
/// compare the two metadata provenances over the whole graph.
///
/// # Why this is separate from the split-pair classifier, and runs EARLIER
///
/// The two diagnostics answer different questions at different moments:
///
/// * this one asks "do the source and the built cdylibs describe the same
///   graph?", which needs ONLY the two metadata maps; and
/// * [`classify_split_same_level_non_trigger_pairs`] asks "does the partition
///   this run is about to execute split an unordered pair?", which needs the
///   planning runtime's `levels`.
///
/// They were briefly one call, emitted together AFTER the pre-spawn
/// `validate_partition`. That ordering had a real hole: `validate_partition`
/// reads the SOURCE metadata, so a stale cdylib can make it REFUSE a
/// partition the loaded runtime would happily run — and because that call is
/// `?`-propagated, the supervisor returned with a bare partition rejection and
/// NO hint that a stale build was the cause. The drift warn went missing on
/// exactly the shape where it matters most. Splitting the seam lets the
/// cross-check run BEFORE the validator, so the disagreement is named on every
/// path, refusing ones included.
///
/// Which truth REFUSES is deliberately unchanged — `validate_partition` still
/// reads source metadata. Only the DIAGNOSTIC moved.
///
/// Infallible by design: the topology build is swallowed into a `debug!`
/// rather than propagated (see `diagnostic_topology`) — a diagnostic that ran
/// BEFORE the refusing validator and could itself fail would re-label the
/// validator's refusal as its own error.
pub fn classify_trigger_metadata_drift(
    config: &GraphConfig,
    source_entry_infos: SourceNodeMetadata<'_>,
    loaded_entry_infos: LoadedNodeMetadata<'_>,
) -> Vec<TriggerClassificationDrift> {
    let Some(topology) = diagnostic_topology(config, loaded_entry_infos) else {
        return Vec::new();
    };
    let source_edges = cerulion_core::graph::build_trigger_edges(config, source_entry_infos.0);
    let loaded_edges = cerulion_core::graph::build_trigger_edges(config, loaded_entry_infos.0);
    trigger_classification_drift(&topology, &source_edges, &loaded_edges)
}

/// The production entry point for the SPLIT-PAIR advisory — classify
/// this partition's unordered pairs from the LOADED metadata.
///
/// # Provenance
///
/// This function takes ONLY the loaded truth, and that is the point: `levels`
/// is the planning runtime's, derived from these exact `NodeInfo`s, so nothing
/// here can be compared against a build the runtime did not execute. The
/// source-parsed metadata is not an input — it has one job (the drift
/// cross-check above) and no business in the classification. Owning the
/// topology + trigger-edge derivation here rather than at the call site is
/// what keeps that true; the supervisor was previously one variable name away
/// from classifying against source-parsed metadata.
///
/// Infallible by design: an ADVISORY must never refuse a run, so the topology
/// build is swallowed into a `debug!` rather than propagated (see
/// `diagnostic_topology`).
pub fn classify_split_same_level_non_trigger_pairs(
    config: &GraphConfig,
    loaded_entry_infos: LoadedNodeMetadata<'_>,
    levels: &Levels,
) -> Vec<SplitNonTriggerPair> {
    let Some(topology) = diagnostic_topology(config, loaded_entry_infos) else {
        return Vec::new();
    };
    let loaded_edges = cerulion_core::graph::build_trigger_edges(config, loaded_entry_infos.0);
    split_same_level_non_trigger_pairs(
        &topology,
        &loaded_edges,
        levels,
        &config.process_groups,
        loaded_entry_infos,
    )
}

/// Report every source-vs-loaded trigger disagreement, one loud
/// `warn!` each.
///
/// This is NEVER a silent pick of one side. The detector must classify from
/// the LOADED metadata (that is what `runtime.levels()` describes), but a
/// disagreement means the operator is reading a verdict about a build that no
/// longer matches the source in front of them — and it equally means the
/// pre-spawn `validate_partition`, which runs on the SOURCE metadata, judged a
/// different graph. Both are worth a line before any worker spawns.
///
/// A freshly built workspace logs NOTHING.
pub fn warn_trigger_classification_drift(graph_name: &str, drift: &[TriggerClassificationDrift]) {
    for d in drift {
        tracing::warn!(
            graph = %graph_name,
            consumer = %d.consumer,
            topic = %d.topic,
            source_says_triggering = d.source_says_triggering,
            loaded_says_triggering = d.loaded_says_triggering,
            "STALE BUILD — this node's source and its BUILT cdylib disagree on \
             whether this input is a `#[input(trigger)]` DAG edge. The run uses the LOADED \
             (built) classification, which is what the levelization and every \
             verdict below describe, while the pre-spawn partition validation read the \
             SOURCE. Rebuild the node (`cerulion node build <type>`, or `--release` to match \
             the run) so the two agree; a raw-FFI node additionally needs its \
             `// CERULION:INFO_START` marker kept in step with its `cerulion_node_info()`."
        );
    }
}

/// Every consumer input whose declared BACKPRESSURE policy differs
/// between the SOURCE metadata and the LOADED cdylib metadata.
///
/// # Why this seam, and why it is a SIBLING of the trigger cross-check
///
/// The auto-partitioner co-locates a `block` topic's producers with
/// its `block` consumers, and it decides that from SOURCE (derivation runs
/// well ahead of the first cdylib load). The run it produces is then levelized
/// and GATED from the LOADED cdylibs. So a stale build carries two costs, in
/// opposite directions:
///
/// * source says `block`, the `.so` says `drop_oldest` — a needlessly large
///   process group. Harmless, and the run works.
/// * source says `drop_oldest`, the `.so` says `block` — the partition did NOT
///   co-locate, `subgraph_for` drops the foreign producer from the consumer's
///   subgraph, and that worker dies at `GraphTopology::validate` with a
///   message blaming an external publisher the operator does not have.
///
/// The second is exactly the failure that co-location exists to remove, re-created by
/// a build that is behind its source — so it must be NAMED rather than left as
/// an unexplained worker death. This is the same class, the same seam and the
/// same hoisting rationale as [`classify_trigger_metadata_drift`] (which runs
/// deliberately BEFORE the refusing `validate_partition`, so the stale-build
/// line survives a refusal); it is a separate function only because the two
/// findings have different grains — see [`BackpressureClassificationDrift`].
///
/// Grain is `(consumer, input)`, walked in topology order and deduplicated.
///
/// Infallible by design, for the same reason as its sibling: a diagnostic that
/// ran BEFORE the refusing validator and could itself fail would re-label the
/// validator's refusal as its own error (see `diagnostic_topology`).
pub fn classify_backpressure_metadata_drift(
    config: &GraphConfig,
    source_entry_infos: SourceNodeMetadata<'_>,
    loaded_entry_infos: LoadedNodeMetadata<'_>,
) -> Vec<BackpressureClassificationDrift> {
    let Some(topology) = diagnostic_topology(config, loaded_entry_infos) else {
        return Vec::new();
    };
    let mut seen: HashSet<(&str, &str)> = HashSet::new();
    let mut found = Vec::new();
    for flow in topology.topics() {
        for consumer in &flow.consumers {
            // The topology was built from the LOADED infos, so the edge's own
            // policy IS the loaded truth — no second topology build.
            let loaded_policy = consumer.policy;
            let Some(source_policy) = source_entry_infos
                .0
                .get(consumer.node_id.as_str())
                .and_then(|info| {
                    info.input_meta()
                        .iter()
                        .find(|m| m.name == consumer.input)
                        .map(|m| m.backpressure)
                })
            else {
                // A node (or an input) the SOURCE parse does not know about is
                // not a DISAGREEMENT — it is an absence, and `validate_graph`
                // speaks for it. Reporting a drift here would invent a
                // `drop_oldest` claim the source never made.
                continue;
            };
            if source_policy == loaded_policy {
                continue;
            }
            if !seen.insert((consumer.node_id.as_str(), consumer.input.as_str())) {
                continue;
            }
            found.push(BackpressureClassificationDrift {
                consumer: consumer.node_id.clone(),
                input: consumer.input.clone(),
                topic: flow.topic.clone(),
                source_policy,
                loaded_policy,
            });
        }
    }
    found
}

/// Report every source-vs-loaded BACKPRESSURE disagreement, one loud
/// `warn!` each.
///
/// Like its trigger sibling this is NEVER a silent pick of one side: the run
/// is gated by the LOADED policy while the PARTITION was derived from the
/// SOURCE one, so the operator is reading a partition that describes a build
/// they no longer have. A freshly built workspace logs NOTHING.
pub fn warn_backpressure_classification_drift(
    graph_name: &str,
    drift: &[BackpressureClassificationDrift],
) {
    for d in drift {
        tracing::warn!(
            graph = %graph_name,
            consumer = %d.consumer,
            input = %d.input,
            topic = %d.topic,
            source_policy = ?d.source_policy,
            loaded_policy = ?d.loaded_policy,
            "STALE BUILD — this input's source and its BUILT cdylib disagree on \
             its `#[input(backpressure = …)]` policy. The multi-process partition is \
             derived and VALIDATED from the SOURCE policy, while the run — and the \
             supervisor's cross-process credit words — are decided by the LOADED one, so \
             the two disagree in BOTH directions. Source `drop_oldest` against a cdylib \
             that says `block` will kill this node's worker at graph build (nothing co-located \
             or credited an edge the source never showed). Source `block` against a cdylib \
             that says `drop_oldest` is the opposite: the partition was accepted for a \
             `block` edge the run does not have, so the edge simply runs LOSSY. Rebuild the \
             node (`cerulion node build <type>`, or `--release` to match the run) so the \
             two agree; a raw-FFI node additionally needs its `// CERULION:INFO_START` \
             marker kept in step with its `cerulion_node_info()`."
        );
    }
}

/// Report every pair [`split_same_level_non_trigger_pairs`] found,
/// one loud `warn!` each, at PLAN time — before any worker is spawned.
///
/// The condition is otherwise SILENT at record time and surfaces much later as
/// an unexplained payload byte in a re-execution diff, so this follows the
/// repo's loud-inference rule: the runtime is about to run a shape whose
/// pairing it cannot order, and it says so every time. The line names the
/// consequence AND all three workarounds, because an operator hitting this
/// needs to choose between them, not go look it up.
///
/// The consequence names `cerulion bag play <bag> --resim all
/// --verify`, and the `--verify` is LOAD-BEARING rather than decorative — a
/// bare `--resim all` is neutral and exits 0 on any completed re-execution, so
/// it is precisely the invocation that would NOT surface this. (The old
/// spelling, `cerulion replay`, is a removed verb: pointing an operator at it
/// would have handed them a command that prints a removal notice and exits
/// non-zero for a reason unrelated to their actual problem.)
///
/// Emitted once per offending `(producer, consumer, input, topic)` tuple — the
/// grain of the fix, since each tuple is repaired by a different `trigger`
/// mark or co-location. A clean partition logs NOTHING.
///
/// # The remedy is not a constant
///
/// "Make the input triggering" is FIVE different gestures depending on the
/// consumer's own macro policy, and three of them make the naive
/// `#[input(trigger)]` a COMPILE ERROR (`cerulion_macros::validate`'s
/// `validate_trigger_inference` rejects a trigger input beside
/// `period_ms`/`external`, and a second trigger input with no sync policy). So
/// the `Fix by …` clause is composed per finding from
/// [`TriggerRemedy::fix_sentence`], while the CONSEQUENCE paragraph stays one
/// literal at one call site — the alternative, five `warn!` arms each carrying
/// its own copy of the consequence prose, is the drift hazard this repo keeps
/// finding. The remedy is ALSO carried as a stable `remedy=` token so the
/// operator-facing grep key does not depend on prose.
///
/// This WAS the interim floor. The conditional mid-level barrier has since
/// landed, so the same finding is now the INPUT to a fix rather than an
/// unactioned advisory — see [`mid_level_barrier_flags`] and the level/message
/// rationale on this function.
pub fn report_split_same_level_non_trigger_pairs(graph_name: &str, pairs: &[SplitNonTriggerPair]) {
    for pair in pairs {
        tracing::info!(
            graph = %graph_name,
            topic = %pair.topic,
            producer = %pair.producer,
            producer_group = %pair.producer_group,
            consumer = %pair.consumer,
            input = %pair.consumer_input,
            consumer_group = %pair.consumer_group,
            level = pair.level,
            remedy = %pair.remedy.token(),
            "this partition SPLITS a same-level non-trigger edge across process \
             groups. Global level {} therefore takes a MID-LEVEL barrier rendezvous (two \
             generations per step instead of one), which orders every group's step-start \
             snapshots before any group ticks — restoring the monolith's guarantee, so the \
             live run is deterministic and its `--record` bag re-executes. RESIDUAL: if \
             `{}` declares a `block` input or publishes onto a `block` topic, its own \
             snapshot runs inside the fused block group, AFTER this rendezvous, and is NOT \
             ordered — only in that case is a remedy still needed. {}",
            pair.level,
            pair.consumer,
            pair
                .remedy
                .fix_sentence(&pair.consumer, &pair.consumer_input)
        );
    }
}

/// Turn the classified split pairs into the per-global-level
/// MID-LEVEL barrier flag vector the supervisor stamps into every
/// [`WorkerPlan::mid_level_barrier`].
///
/// `flags[g]` is `true` iff some pair sits at global level `g`. `global_levels`
/// is the GLOBAL level count (the length of every worker's
/// `global_level_map`), so the result is index-compatible with it by
/// construction — which is exactly what
/// `GraphRuntime::install_barrier_participant` checks.
///
/// # Why a level out of range is DROPPED, not clamped or panicked
///
/// The two inputs come from the same planning levelization, so a pair's level
/// is always in range. If that ever stopped holding, the right answer is the
/// earlier behaviour for that level (no extra rendezvous — the run is no
/// worse than it was) rather than a panic that kills a legal deployment, or a
/// clamp that would flag the WRONG level and desynchronise nothing but confuse
/// everything. It is a `debug_assert!` so a test build still fails loudly.
///
/// An empty `pairs` yields an all-`false` vector: every level crosses exactly
/// one generation per step and the run pays nothing, which is the
/// overwhelmingly common case.
pub fn mid_level_barrier_flags(pairs: &[SplitNonTriggerPair], global_levels: usize) -> Vec<bool> {
    let mut flags = vec![false; global_levels];
    for pair in pairs {
        match flags.get_mut(pair.level) {
            Some(slot) => *slot = true,
            None => {
                debug_assert!(
                    false,
                    "mid_level_barrier_flags: pair at level {} is outside the {} global levels — the classifier and the planner read the SAME levelization",
                    pair.level, global_levels
                );
                tracing::error!(
                    level = pair.level,
                    global_levels,
                    producer = %pair.producer,
                    consumer = %pair.consumer,
                    "a split same-level non-trigger pair reports a global level \
                     outside the planned level count; its level gets no mid-level barrier"
                );
            }
        }
    }
    flags
}

/// Produce the deterministic multi-process deployment plan.
///
/// * `config` — the FULL graph (must declare `process_groups:`).
/// * `levels` — the ONE GLOBAL levelization of the full graph (see the module
///   docs on why this is an input, not derived here).
/// * `tightest_timing_ns` — the full graph's `Scheduler::tightest_timing_ns()`
///   (`None` when no node declares timing); floored to the handed quantum.
/// * `nonce` — a per-deployment uniqueness token folded into the barrier
///   namespace + ready-sentinel names (a run id, PID, timestamp — the caller's
///   choice; kept a plain `&str` here).
///
/// # Steps
///
/// Validate + [`derive_process_groups`] (structural validation + participant-map
/// derivation), floor the quantum, then per-group contiguity guard + subgraph
/// split before assembling the plan. Errors surface via
/// [`TransportError::GraphError`], matching the rest of the graph layer.
pub fn plan_deployment(
    config: &GraphConfig,
    levels: &Levels,
    tightest_timing_ns: Option<u64>,
    nonce: &str,
) -> TransportResult<DeploymentPlan> {
    if !config.has_process_groups() {
        return Err(TransportError::GraphError {
            reason: format!(
                "plan_deployment: graph '{}' declares no `process_groups:` — a \
                 multi-process deployment needs at least one process group (add \
                 `process_groups:` to the graph, or run it single-process)",
                config.identity()
            ),
        });
    }

    // Structural validation (orphans / dangling refs / double-assignment / empty
    // groups+names / order-permutation) + the participant-map derivation live in
    // `derive_process_groups`. It also rejects the UNDERSIZED config↔levels
    // direction (a member with no level → error). The OVERSIZED/stale direction
    // is guarded below.
    let groups = derive_process_groups(config, levels)?;

    // Sanitized group-name uniqueness. Group names key
    // worker plan files (`plan_{sanitize_ns(group)}.json`) and READY sentinels
    // (`ready_path`), both through `sanitize_ns` — two DISTINCT
    // validation-passing groups ("g.1"/"g_1", "left arm"/"left_arm") that
    // sanitize to ONE token would silently OVERWRITE each other's plan files
    // (one subgraph runs twice, the other never — wrong topology) and
    // cross-trip READY gating (the second worker declared READY off the
    // first's sentinel). Reject at PLAN time, loudly, so every downstream
    // group-keyed filename is collision-free by construction.
    {
        let mut seen: std::collections::HashMap<String, &str> = std::collections::HashMap::new();
        for group in &groups {
            let token = sanitize_ns(&group.name);
            if let Some(prev) = seen.insert(token.clone(), &group.name) {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "process groups '{prev}' and '{}' both sanitize to '{token}' — \
                         rename one process group; sanitized names must be unique because \
                         they key worker plan files and READY sentinels",
                        group.name
                    ),
                });
            }
        }
    }

    // The handed `levels` MUST be the levelization of THIS exact graph — not a
    // stale one or one built from a superset. Kahn levelization fills levels
    // `0..len` contiguously (every level is occupied by ≥1 node), so the highest
    // level any node occupies must be exactly `levels.len() - 1`. A larger
    // `levels.len()` means the caller handed a mismatched levelization; without
    // this guard the plan would carry phantom trailing-`None` global levels
    // (extra empty barrier rounds every step) or — worse, if a node is
    // misplaced — silently shifted ownership bands. Fail loud instead.
    let occupied_level_count = config
        .nodes
        .iter()
        .filter_map(|n| levels.level_of(&n.id))
        .max()
        .map(|hi| hi + 1)
        .unwrap_or(0);
    if occupied_level_count != levels.len() {
        return Err(TransportError::GraphError {
            reason: format!(
                "plan_deployment: graph '{}' occupies {occupied_level_count} global DAG \
                 levels but the handed `levels` reports {} — the levelization does not \
                 match this graph (stale, or built from a superset). Rebuild `levels` from \
                 this exact config.",
                config.identity(),
                levels.len()
            ),
        });
    }

    let handed_quantum_ns = handed_quantum_ns(tightest_timing_ns);
    let barrier_ns = deployment_ns(config.identity(), nonce);
    let barrier_id = deployment_barrier_id(config.identity());
    // ONE deployment-wide GO sentinel — identical on every worker (the start gate).
    let go = go_path(config.identity(), nonce);
    // The GO deadline composes with the group count: the first-spawned worker's
    // legitimate GO wait spans every later sibling's sequential READY window
    // (see `WorkerPlan::go_deadline_ms`). The planner knows the count — pure.
    let go_deadline_ms = GO_BASE_MS + (groups.len() as u64) * READY_BUDGET_MS;

    let mut workers = Vec::with_capacity(groups.len());
    for group in &groups {
        // Plan-time proxy for the runtime's contiguous-split precondition.
        validate_contiguous_ownership(group)?;

        // `derive_process_groups` keyed the group off `config.process_groups`, so
        // the reverse lookup always resolves; guard rather than panic anyway.
        let members =
            config
                .process_groups
                .get(&group.name)
                .ok_or_else(|| TransportError::GraphError {
                    reason: format!(
                        "plan_deployment: derived group '{}' has no members entry in \
                     process_groups (internal inconsistency)",
                        group.name
                    ),
                })?;

        workers.push(WorkerPlan {
            group: group.name.clone(),
            rank: group.rank,
            global_level_map: group.global_level_map.clone(),
            // All-`false` here, and that is the PLANNER's real
            // answer rather than a placeholder. `plan_deployment` is PURE (see
            // the module docs) — it takes no `NodeInfo`s, so it cannot know
            // which edges are triggering and therefore cannot classify a split
            // same-level non-trigger pair. The SUPERVISOR, which does hold the
            // loaded metadata, stamps the real vector over this before the plans
            // are serialised (`stamp_mid_level_barrier`). A run that never
            // reaches that stamp keeps exactly the earlier behaviour: one
            // generation per global level.
            mid_level_barrier: vec![false; group.global_level_map.len()],
            handed_quantum_ns,
            node_name: worker_node_name(config.identity(), &group.name),
            barrier_ns: barrier_ns.clone(),
            barrier_id: barrier_id.clone(),
            ready_path: ready_path(config.identity(), &group.name, nonce),
            graph_identity: format!("{}_{}", config.identity(), group.name),
            subgraph: subgraph_for(config, &group.name, members),
            // The pure planner leaves this empty; the SUPERVISOR owns
            // minting + serializing the shared iceoryx2 `Config` and injects the
            // JSON into every worker post-plan (a snapshot of the host's
            // resolved global config — the DEFAULT `iox2_` data-plane namespace
            // on a config-file-free host). The worker (`graph_run_worker`)
            // rejects an empty value LOUDLY.
            ix_config_json: String::new(),
            go_path: go.clone(),
            go_deadline_ms,
            // The supervisor injects its `graph run --release` preference
            // post-plan (like `ix_config_json`); the pure planner defaults false.
            prefer_release: false,
            // The supervisor stamps `graph run --trace-limit`
            // post-plan; the pure planner sets the production default.
            trace_limit: default_trace_limit(),
            // The supervisor stamps the resolved park policy post-plan
            // (`graph_cmd::stamp_park_policy`); the pure planner defaults off.
            monitor_wait: false,
            doorbell: false,
            // The supervisor stamps `--no-cpu-dma-lock` post-plan
            // (`graph_cmd::stamp_cap_mode`); the pure planner defaults false
            // (= the worker's `Auto` cap mode).
            cap_disabled: false,
            // The supervisor stamps the RESOLVED mode post-plan
            // (`graph_cmd::stamp_execution_mode`); the pure planner leaves the
            // default, which is the shipped barrier-lockstep worker.
            execution_mode: ExecutionMode::default(),
            // Harvesting per-topic requirements needs the IMPURE
            // full-graph build, so the pure planner leaves this empty; the
            // supervisor stamps it post-plan (`stamp_topic_requirements`). Empty =
            // no cross-group union (a no-op) — the single-process-view behavior.
            topic_requirements: std::collections::BTreeMap::new(),
            sibling_topics: sibling_topics_for(config, members),
            // Minted by the supervisor from the LOADED planning
            // topology in the credit-wiring commit; the pure planner holds no
            // `NodeInfo`s and cannot see a `block` policy, so it mints none.
            credit_edges: Vec::new(),
            // The supervisor stamps the recording-ring tag
            // post-plan when `--record` is active; the pure planner
            // leaves it None (not recording).
            recording_ring: None,
            trace_ring: None,
            // Stamped post-plan by the supervisor, like
            // `recording_ring` — the pure planner reads no environment.
            state_arm_tag: None,
            run_dir: None,
            // The pure planner mints no wedge page; the SUPERVISOR
            // creates it and stamps the binding post-plan (the `state_arm_tag` /
            // `recording_ring` precedent — a page is a real SHM object, which a
            // pure planner must not create).
            wedge_page: None,
        });
    }

    let expected = workers.len();
    tracing::debug!(
        graph = %config.identity(),
        workers = expected,
        handed_quantum_ns,
        barrier_ns = %barrier_ns,
        "planned multi-process deployment"
    );

    Ok(DeploymentPlan {
        workers,
        barrier_ns,
        barrier_id,
        expected,
    })
}

/// Split the full graph into ONE group's subgraph: only this group's nodes,
/// parent `prefix` / `multi_publisher_topics` preserved (so topic edges resolve
/// to the same absolute names), and `process_groups` / `process_group_order`
/// cleared (a worker is a single-process monolith).
///
/// # Input-source rewriting (the cross-group edge fix)
///
/// Inputs are kept VERBATIM for IN-GROUP edges but ABSOLUTIZED for CROSS-GROUP
/// edges. A member input whose source is:
///
/// * already ABSOLUTE (`/...`) — passes through unchanged (an external topic or
///   another graph's; also the shape a `topic:` override reference always has,
///   since `validate_graph` forces overrides absolute).
/// * RELATIVE and resolving to a topic THIS group's members produce — an
///   in-group edge; kept verbatim (`n0/out`) so it resolves locally.
/// * RELATIVE and resolving to a topic NO group member produces — a CROSS-GROUP
///   edge whose producer lives in another worker; rewritten to its resolved
///   absolute topic (`/{prefix}/{producer}/{out}`). Without this rewrite the
///   worker's `validate_graph` HARD-fails ("references non-existent source"),
///   since the producer node is FOREIGN to (dropped from) this subgraph; as an
///   absolute source it instead validates + provisions as an external
///   source, which is exactly the cross-process handoff a peer worker publishes.
///
/// Nodes are filtered in the parent `config.nodes` order (Principle #5/#7 —
/// graph order is the source of truth), so the subgraph is deterministic.
///
/// Precondition: `config` is pre-validated (`validate_graph` ran — the
/// supervisor validates before planning), so a RELATIVE reference to a
/// `topic:`-overridden output cannot reach here (`validate_graph` rejects it
/// with the reference-it-as-absolute hint).
/// `pub(crate)` because [`crate::replay_rank`]'s per-rank replay
/// planner restricts each RECORDED rank's subgraph through this SAME function
/// — the cross-group absolutization rule above is exactly the cross-rank edge
/// shape bag-fed injection serves, and a second copy of it would be free to
/// drift from this one (one restriction rule, one place).
pub(crate) fn subgraph_for(
    config: &GraphConfig,
    group_name: &str,
    members: &[String],
) -> GraphConfig {
    let member_set: HashSet<&str> = members.iter().map(String::as_str).collect();

    // The absolute topics THIS group's members PRODUCE (honoring each output's
    // `topic:` override via `resolve_output_topic`). A relative input source
    // resolving into this set is an in-group edge; one that does not is
    // cross-group and gets absolutized below.
    let produced: HashSet<String> = config
        .nodes
        .iter()
        .filter(|n| member_set.contains(n.id.as_str()))
        .flat_map(|n| {
            n.outputs
                .iter()
                .map(move |o| resolve_output_topic(&config.prefix, &n.id, o))
        })
        .collect();

    let nodes = config
        .nodes
        .iter()
        .filter(|n| member_set.contains(n.id.as_str()))
        .map(|n| {
            let mut n = n.clone();
            for input in &mut n.inputs {
                // Absolute sources pass through verbatim. A relative source is
                // absolutized ONLY when its resolved topic is NOT produced
                // in-group (a cross-group edge); in-group relatives stay verbatim.
                if !input.source.starts_with('/') {
                    let resolved = resolve_source(&config.prefix, &input.source);
                    if !produced.contains(&resolved) {
                        input.source = resolved;
                    }
                }
            }
            n
        })
        .collect();

    GraphConfig {
        // A parent `level_assignments` override is
        // RESTRICTED to this group's members and COMPRESSED to the group's
        // 0-based local band (`compress_group_level_assignments` — the SAME
        // fn `subgraph_local_levels`' plan-time bijection model uses, so the
        // worker and the validator cannot drift). The worker's own
        // `resolve_levels` then runs EXACTLY the local levels the
        // supervisor's global plan banded — a worker re-Kahning its subgraph
        // independently (the earlier behavior, kept for `None`) could
        // collapse levels the refined global assignment deliberately
        // separated. Parent `None` ⇒ `None` ⇒ the worker Kahn-levelizes,
        // byte-identical to the earlier behavior.
        level_assignments: config.level_assignments.as_ref().map(|a| {
            let member_refs: Vec<&str> = members.iter().map(String::as_str).collect();
            cerulion_core::graph::compress_group_level_assignments(a, &member_refs)
        }),
        // The per-process subgraph drops the network block (like it
        // drops `process_groups`). The full graph's `network:` is validated
        // once at the top level, and the workers stay network-free: the
        // separate gateway process owns the network plane.
        network: None,
        // By design, a worker's subgraph is a derived graph, never a
        // file, so it carries an identity and no `name:` key. The identity
        // does not survive the plan's JSON hop (it is `#[serde(skip)]` —
        // it describes how a config was LOADED, not the document), so
        // `WorkerPlan::graph_identity` carries it across and
        // `graph_run_worker` restores it.
        name: None,
        identity: format!("{}_{group_name}", config.identity()),
        prefix: config.prefix.clone(),
        nodes,
        multi_publisher_topics: config.multi_publisher_topics.clone(),
        process_groups: IndexMap::new(),
        process_group_order: Vec::new(),
    }
}

/// The cross-group input edges of one group, by topic: every absolute topic a
/// member CONSUMES that a NON-member of the same graph PRODUCES. See
/// [`WorkerPlan::sibling_topics`].
///
/// Sources resolve exactly as [`subgraph_for`] resolves them, so the names
/// here are the names the worker's subgraph carries. An absolute source the
/// author wrote by hand counts too: in the full graph it wires to its in-graph
/// producer, and only the split makes it look external.
fn sibling_topics_for(
    config: &GraphConfig,
    members: &[String],
) -> std::collections::BTreeSet<String> {
    let member_set: HashSet<&str> = members.iter().map(String::as_str).collect();
    let is_member = |n: &&cerulion_core::graph::config::NodeDef| member_set.contains(n.id.as_str());
    let sibling_produced: HashSet<String> = config
        .nodes
        .iter()
        .filter(|n| !is_member(n))
        .flat_map(|n| {
            n.outputs
                .iter()
                .map(move |o| resolve_output_topic(&config.prefix, &n.id, o))
        })
        .collect();
    config
        .nodes
        .iter()
        .filter(is_member)
        .flat_map(|n| &n.inputs)
        .map(|input| {
            if input.source.starts_with('/') {
                input.source.clone()
            } else {
                resolve_source(&config.prefix, &input.source)
            }
        })
        .filter(|topic| sibling_produced.contains(topic))
        .collect()
}

/// The iceoryx2 node name for a worker: `cerulion_{graph}_{group}`, SANITIZED
/// (`sanitize_ns`): the value feeds iceoryx2's `NodeName` in transport init — an
/// arbitrary charset (a `/` or space in the YAML graph name) would fail the
/// worker there. Sanitizing keeps a legal graph name from costing a deployment
/// its run at all (that conversion is a typed refusal, so a bad name
/// cannot PANIC the worker). Group-name
/// uniqueness AFTER sanitization is already guaranteed by `plan_deployment`'s
/// plan-time collision guard.
fn worker_node_name(graph: &str, group: &str) -> String {
    format!("cerulion_{}_{}", sanitize_ns(graph), sanitize_ns(group))
}

/// The shared per-deployment SHM namespace, SANITIZED (`sanitize_ns`): the NS
/// lands VERBATIM in the `/dev/shm` filename of every segment the deployment
/// mints under it (`MappedBarrier`'s `shm_open` name is
/// `/cer_bar_{ns}_{hash-of-id}`; `MappedCredit` uses
/// `/cer_crd_{ns}_{hash-of-id}`), so a `/`-bearing graph name would
/// ENOENT/EINVAL `create_owned` — the same class the sentinel filenames were
/// fixed for. `nonce` keeps concurrent deployments of the same graph from
/// colliding on a `/dev/shm` object.
///
/// Historically named for the barrier, which was the only tenant; the credit
/// words share it because they share the deployment, and the per-family
/// PREFIX (`cer_bar_` vs `cer_crd_`) is what keeps the two apart.
fn deployment_ns(graph: &str, nonce: &str) -> String {
    format!("cerdep_{}_{}", sanitize_ns(graph), sanitize_ns(nonce))
}

/// The shared per-deployment barrier id within the namespace. One barrier
/// gates the whole DAG, so a single stable id suffices. Deliberately NOT
/// sanitized: `barrier_shm_name` FNV-HASHES the id into the `/dev/shm`
/// filename (only the NS goes in raw), so an arbitrary charset here is
/// filename-safe by construction.
fn deployment_barrier_id(graph: &str) -> String {
    format!("{graph}_levelgate")
}

/// A worker's READY-sentinel filename (a bare name the supervisor roots under
/// the plan directory). Components are SANITIZED (`sanitize_ns`): a graph or
/// group name carrying a path separator (`name: my/graph` in YAML) would
/// otherwise inject an intermediate directory the supervisor never creates —
/// the worker's READY write then ENOENTs and the failure surfaces as a
/// misleading "exited before signaling READY".
fn ready_path(graph: &str, group: &str, nonce: &str) -> String {
    format!(
        "cerulion_{}_{}_{}.ready",
        sanitize_ns(graph),
        sanitize_ns(group),
        sanitize_ns(nonce)
    )
}

/// The deployment-wide GO-sentinel filename (bare name; the supervisor roots it
/// under the plan directory). ONE file per deployment — every worker carries the
/// SAME value (no `group` component), so one supervisor `fs::write` releases
/// them all. Components sanitized like [`ready_path`] (a raw `/` in the graph
/// name would break the supervisor's rooted GO write the same way). See
/// [`WorkerPlan::go_path`] for the start-gate contract.
fn go_path(graph: &str, nonce: &str) -> String {
    format!("cerulion_{}_{}.go", sanitize_ns(graph), sanitize_ns(nonce))
}

/// The order the supervisor must SPAWN workers in — indices
/// into [`DeploymentPlan::workers`], sorted ascending by each worker's MINIMUM
/// owned global DAG level (the first index where its `global_level_map` entry
/// is `Some`). Ties (which cannot occur under a contiguous-split partition, but
/// are handled anyway) break by `rank`.
///
/// Producer-owning groups (lower global levels) still spawn + READY before
/// consumer groups, but this ordering is now only bring-up latency
/// HYGIENE, not the correctness mechanism: correctness against consumer-first
/// creation is owned by the supervisor's data-plane PRE-CREATION
/// (`graph_run_supervisor` step (9.5)), which creates every graph-owned
/// topic's iceoryx2 services at the FULL finalized config before any worker
/// spawns — a consumer group that comes up first can only OPEN the
/// already-correct service, never win the create race and under-provision it
/// (the consumer-first spawn death). Level ordering and the READY handshake (each
/// worker touches its sentinel once its owned publishers attach) are kept
/// byte-unchanged as spawn staging.
pub fn spawn_order(plan: &DeploymentPlan) -> Vec<usize> {
    let mut order: Vec<usize> = (0..plan.workers.len()).collect();
    order.sort_by_key(|&i| {
        let w = &plan.workers[i];
        let min_level = w
            .global_level_map
            .iter()
            .position(|slot| slot.is_some())
            .unwrap_or(usize::MAX);
        (min_level, w.rank)
    });
    order
}

/// Map any character outside `[A-Za-z0-9_]` to `_` — THE ONE sanitize charset
/// for every multi-process-derived name: the iceoryx2 `FileName` prefix body,
/// sentinel filenames (`ready_path`/`go_path`), the supervisor's plan
/// directory + per-group plan filenames, and the worker trace-dump filenames.
/// `pub(crate)` so graph_cmd's supervisor uses the SAME charset (a second
/// sanitizer with a divergent charset — e.g. one keeping `-` — would let
/// "planner-a"/"planner_a" collide on sentinels but not plan files, or vice
/// versa). Group-token uniqueness under this map is enforced at plan time by
/// `plan_deployment`'s collision guard.
pub(crate) fn sanitize_ns(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

/// FNV-1a 64 (offset `0xcbf29ce484222325`, prime `0x100000001b3`) — the same
/// constants as `cerulion_core`'s barrier name derivation (`barrier.rs`). A
/// tiny LOCAL copy, deliberately NOT imported from cerulion_core internals
/// (the barrier's hasher is not pub API; the algorithm is 5 lines and stable).
fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}

/// The ONE shared iceoryx2 `Config` every worker of a multi-process
/// deployment joins — a snapshot of the host's RESOLVED global config
/// (`Config::global_config().clone()`), i.e. the DEFAULT iceoryx2 namespace.
/// On a config-file-free host (Cerulion ships no iceoryx2 config file) this IS
/// `Config::default()` (`global.prefix == "iox2_"`).
///
/// # Why the default namespace
///
/// Previously the deployment minted a run-scoped `cer_d_{fnv:016x}` prefix,
/// which quarantined the whole DATA plane: `topic echo/hz/list/info` (which
/// attach via `Config::global_config()`), cross-graph absolute topics, and
/// external publishers were all INVISIBLE to a default `graph run` deployment.
/// Moving the data plane onto the default namespace makes all of that
/// interoperate — the same zero-config introspection story a single-process
/// `graph run` gives.
///
/// # Why `global_config()`, not a literal `Config::default()`
///
/// Every OTHER component resolves `Config::global_config()` — `topic
/// list/echo/hz` (`topic_cmd.rs`), the monolith/default transport-init path,
/// and the dead-node cleanup sweep (`ipc_cleanup.rs`) — and that resolution
/// honors an operator-installed iceoryx2 config file. A literal
/// `Config::default()` here would let such a file SILENTLY split the mp data
/// plane from everything else — exactly the invisibility class the default namespace kills.
/// Snapshotting the supervisor's resolved config (once, at mint) and shipping
/// it to every worker + bagd keeps all components on the identical namespace
/// whatever the host's config resolves to. (The mint runs inside `graph_run`,
/// AFTER `graph_run`'s dead-node sweep has already forced the first
/// `global_config()` resolution — so iceoryx2's first-resolution "No config
/// file was loaded" warn cannot originate from HERE at any log level. It is
/// additionally invisible at the default level: `graph_run` now
/// applies `init_iceoryx_log_level_from_env()` rather than a hardcoded
/// `set_log_level(Error)`, so the level is `IOX2_LOG_LEVEL` if set — an
/// operator running at `warn` or lower WILL see that warn from wherever the
/// first resolution actually happens.)
///
/// `graph` + `nonce` are IGNORED (the signature + serialization plumbing are
/// kept intact — the supervisor still mints this ONCE and injects the JSON
/// into every [`WorkerPlan::ix_config_json`], and the worker still rejects an
/// EMPTY `ix_config_json` loudly; the resolved config serializes to NON-EMPTY
/// JSON, so that guard keeps working). Concurrent runs of the SAME graph now
/// share one namespace and collide LOUDLY via the single-writer publisher
/// checks — the DESIGNED protection, not a silent overlap. Only the
/// deployment's INFRA stays run-scoped elsewhere: the barrier (`cerdep_*` POSIX
/// SHM), the trace rings (`cer_rec_*`/`cer_rg_*` POSIX SHM), the doorbells
/// (`/cer_db_<$USER>_*`, already `$USER`-scoped), and the supervisor's
/// `cer_p_{hex}` PLANNING namespace (see [`planning_ix_config`]).
pub fn mint_deployment_ix_config(
    _graph: &str,
    _nonce: &str,
) -> TransportResult<iceoryx2::config::Config> {
    Ok(iceoryx2::config::Config::global_config().clone())
}

/// The supervisor's THROWAWAY planning namespace, a
/// FIXED-LENGTH `cer_p_{fnv:016x}` prefix where `fnv` is FNV-1a 64 over
/// `{graph}\x00{nonce}` (the NUL separator keeps `("ab","c")`/`("a","bc")`
/// distinct).
///
/// The supervisor builds the full graph ONCE (to fail-fast on validation and to
/// extract the global `Levels` + tightest timing the planner needs). That
/// planning build attaches REAL single-writer publishers for every graph-owned
/// topic; sharing the workers' DEFAULT (`iox2_`) data-plane namespace would
/// collide with the workers' own single-writer publishers. This distinct
/// planning prefix keeps it inert — so the planning namespace STAYS run-scoped
/// even though the data plane moved to the default.
///
/// # Why the `cer_p_` prefix is safe against the `Node::list` panic
///
/// iceoryx2 0.9.1's `Node::list` enumerates node-monitor files by STRING-PREFIX
/// match against `global.prefix` and parses the remainder as a bare `u128` node
/// id (`node/mod.rs` `parse::<u128>().unwrap()`). Any namespace whose prefix is
/// a STRICT PREFIX of another's therefore lists the other's files, fails the
/// u128 parse, and PANICS the process (observed: workers died pre-READY at
/// exit 101). The default data-plane prefix `iox2_` and this `cer_p_…` prefix
/// DIVERGE AT CHARACTER 0 (`i` vs `c`), so NEITHER can ever string-prefix the
/// other — the whole collision class is structurally dead. (Previously this
/// invariant was maintained by making `cer_d_` and `cer_p_` EQUAL LENGTH; the
/// data plane is now the resolved default, so char-0 divergence is the
/// mechanism. The guarantee is stated against the config-file-free resolution
/// `iox2_`; an operator config file overriding `global.prefix` to something
/// `cer_p`-adjacent is outside it and would surface via iceoryx2's own errors.)
///
/// Everything except the prefix comes from the host's RESOLVED global config
/// (`Config::global_config().clone()` — crucially `global.root_path`
/// must ride an operator config file's root exactly like the data plane and
/// the cleanup sweep do; on a config-file-free host this is `Config::default()`),
/// so every supervisor process that mints from the SAME `(graph, nonce)` on one
/// host converges on the SAME planning namespace. Hex is always FileName-legal
/// (`sanitize_ns` is not needed here — it still guards sentinel + plan
/// filenames); the `FileName` error arm is unreachable for this fixed 22-char
/// hex shape but kept as a cheap belt-and-suspenders guard. The supervisor logs
/// the graph → prefix mapping at mint + spawn time for `/dev/shm` correlation.
pub fn planning_ix_config(graph: &str, nonce: &str) -> TransportResult<iceoryx2::config::Config> {
    let mut hash_input = Vec::with_capacity(graph.len() + 1 + nonce.len());
    hash_input.extend_from_slice(graph.as_bytes());
    hash_input.push(0);
    hash_input.extend_from_slice(nonce.as_bytes());
    let fnv = fnv1a64(&hash_input);
    let prefix_str = format!("cer_p_{fnv:016x}");
    let prefix = FileName::new(prefix_str.as_bytes()).map_err(|e| {
        TransportError::InvalidTransportConfig {
            reason: format!(
                "cannot build the iceoryx2 SHM-namespace prefix from graph '{graph}' + \
                 nonce '{nonce}' (candidate '{prefix_str}', {} bytes): iceoryx2 FileName rejects \
                 it ({e:?})",
                prefix_str.len()
            ),
        }
    })?;
    let mut config = iceoryx2::config::Config::global_config().clone();
    config.global.prefix = prefix;
    Ok(config)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::graph::config::{InputDef, NodeDef, OutputDef};
    use cerulion_core::graph::topology::{GraphTopology, TriggerEdges};
    use cerulion_core::graph::NodeInfo;

    // ===================================================================
    // Fixtures: mirror `crates/cerulion_core/tests/partition_test.rs` so the
    // GLOBAL levelization we feed the planner is built the SAME way the
    // runtime builds it (topology + trigger edges), not fabricated.
    // ===================================================================

    const PREFIX: &str = "p";

    /// One node: id, (input_name, source) pairs, output names.
    fn node(id: &str, inputs: &[(&str, &str)], outputs: &[&str]) -> NodeDef {
        NodeDef {
            ros2: None,
            id: id.to_string(),
            node_type: id.to_string(),
            inputs: inputs
                .iter()
                .map(|(name, source)| InputDef {
                    name: name.to_string(),
                    source: source.to_string(),
                })
                .collect(),
            outputs: outputs
                .iter()
                .map(|name| OutputDef {
                    name: name.to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                })
                .collect(),
        }
    }

    /// A producer whose single output is overridden onto an ABSOLUTE topic —
    /// the only way two nodes can produce ONE topic (`GraphTopology::build`
    /// derives `<prefix>/<node>/<output>` otherwise, so two producers of one
    /// topic are unconstructible without the override plus the
    /// `multi_publisher_topics:` permission).
    fn node_out_at(id: &str, topic: &str) -> NodeDef {
        let mut n = node(id, &[], &["out"]);
        n.outputs[0].topic = Some(topic.to_string());
        n
    }

    // ===================================================================
    // `credit_edges_for` — the cross-process `block` credit mint.
    //
    // PURE oracle vectors: a hand-built topology + a hand-built node->rank map
    // in, a hand-written expected plan list out. No supervisor, no transport,
    // no filesystem.
    // ===================================================================

    /// One `block` input's metadata at an explicit depth.
    fn block_meta(name: &str, depth: usize) -> cerulion_core::graph::node::InputMeta {
        cerulion_core::graph::node::InputMeta {
            name: name.to_string(),
            schema_hash: 0,
            trigger: true,
            depth,
            backpressure: cerulion_core::graph::node::BackpressurePolicy::Block,
            expect_within_ms: None,
        }
    }

    /// One non-`block` input's metadata (the mixed-topic sibling).
    fn plain_meta(name: &str) -> cerulion_core::graph::node::InputMeta {
        cerulion_core::graph::node::InputMeta {
            name: name.to_string(),
            schema_hash: 0,
            trigger: true,
            depth: 4,
            backpressure: cerulion_core::graph::node::BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }
    }

    /// `(node id, rank)` pairs as the map `credit_edges_for` takes.
    fn ranks<'a>(pairs: &[(&'a str, usize)]) -> std::collections::HashMap<&'a str, usize> {
        pairs.iter().copied().collect()
    }

    /// Render a plan list to a comparable, hand-writable shape.
    fn shape_edges(
        edges: &[CreditEdgePlan],
    ) -> Vec<(String, String, String, u32, Vec<usize>, usize)> {
        edges
            .iter()
            .map(|e| {
                (
                    e.topic.clone(),
                    e.consumer_node.clone(),
                    e.consumer_input.clone(),
                    e.depth,
                    e.producer_ranks.clone(),
                    e.consumer_rank,
                )
            })
            .collect()
    }

    /// A one-producer, one-`block`-consumer graph plus a second, non-block
    /// consumer on a DIFFERENT topic (so the mint has something to ignore).
    fn simple_block_topology(depth: usize) -> GraphTopology {
        let config = config_with(vec![
            node("prod", &[], &["out"]),
            node("sink", &[("gate", "prod/out")], &[]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "sink".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", depth)], vec![]),
        );
        GraphTopology::build(&config, &infos).expect("topology")
    }

    fn config_with(nodes: Vec<NodeDef>) -> GraphConfig {
        GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "credit".to_string(),
            prefix: PREFIX.to_string(),
            nodes,
            multi_publisher_topics: Vec::new(),
            process_groups: IndexMap::new(),
            process_group_order: Vec::new(),
        }
    }

    /// THE oracle: a single-producer, all-`block` flow split across two ranks
    /// mints exactly one entry, carrying the LOADED topology's depth.
    #[test]
    fn a_split_single_producer_block_edge_mints_exactly_one_credit_entry() {
        let topo = simple_block_topology(7);
        let edges = credit_edges_for(&topo, &ranks(&[("prod", 0), ("sink", 1)])).expect("mint");
        assert_eq!(
            shape_edges(&edges),
            vec![(
                "/p/prod/out".to_string(),
                "sink".to_string(),
                "gate".to_string(),
                7,
                vec![0],
                1
            )],
            "one entry, the resolved absolute topic, the declared depth, and both ranks"
        );
    }

    /// A CO-LOCATED edge mints NOTHING — it keeps its process-local word, which
    /// costs no syscall and no `/dev/shm` object.
    #[test]
    fn a_co_located_block_edge_mints_nothing() {
        let topo = simple_block_topology(7);
        let edges = credit_edges_for(&topo, &ranks(&[("prod", 0), ("sink", 0)])).expect("mint");
        assert!(
            edges.is_empty(),
            "same rank on both ends needs no cross-process word, got {edges:?}"
        );
    }

    /// A MIXED topic (one `block` consumer, one `drop_oldest` sibling) mints
    /// NOTHING even when split: the `block` consumer there already degrades to
    /// `drop_oldest`, so a word would describe an occupancy nothing gates on.
    #[test]
    fn a_split_mixed_topic_mints_nothing() {
        let config = config_with(vec![
            node("prod", &[], &["out"]),
            node("sink", &[("gate", "prod/out")], &[]),
            node("other", &[("plain", "prod/out")], &[]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "sink".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", 4)], vec![]),
        );
        infos.insert(
            "other".to_string(),
            NodeInfo::with_meta(vec![plain_meta("plain")], vec![]),
        );
        let topo = GraphTopology::build(&config, &infos).expect("topology");
        let edges = credit_edges_for(&topo, &ranks(&[("prod", 0), ("sink", 1), ("other", 1)]))
            .expect("mint");
        assert!(
            edges.is_empty(),
            "a mixed topic's `block` consumer degrades to drop_oldest; crediting it would \
             describe an occupancy nothing gates on, got {edges:?}"
        );
    }

    /// A MULTI-PRODUCER flow mints NOTHING even when split — the single-producer scope
    /// fence. A credit CLAIM is a plain read under one writer; with two, a
    /// producer's claim can be consumed by the other between the read and the
    /// publish, which is silent overshoot on an edge declared LOSSLESS.
    #[test]
    fn a_split_multi_producer_block_edge_mints_nothing() {
        let mut config = config_with(vec![
            node("p0", &[], &["out"]),
            node("p1", &[], &["out"]),
            node("sink", &[("gate", "/shared")], &[]),
        ]);
        // Both producers publish the SAME absolute topic, which is only legal
        // for a listed topic.
        config.nodes[0].outputs[0].topic = Some("/shared".to_string());
        config.nodes[1].outputs[0].topic = Some("/shared".to_string());
        config.multi_publisher_topics = vec!["/shared".to_string()];
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("p0".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert("p1".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "sink".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", 4)], vec![]),
        );
        let topo = GraphTopology::build(&config, &infos).expect("topology");
        let edges =
            credit_edges_for(&topo, &ranks(&[("p0", 0), ("p1", 0), ("sink", 1)])).expect("mint");
        assert!(
            edges.is_empty(),
            "a multi-producer cross-process edge is outside the single-producer fence, got {edges:?}"
        );
    }

    /// Q4 (this is by design): a `multi_publisher_topics:`-listed topic with
    /// exactly ONE in-graph producer IS credited.
    ///
    /// The listing is an opt-in for a topic that MAY carry several writers; the
    /// fence is about how many actually do IN THIS GRAPH. With one in-graph
    /// producer the claim is serialisable by construction, exactly as on an
    /// unlisted topic, so the listing alone must not cost the edge its word.
    #[test]
    fn a_listed_topic_with_one_in_graph_producer_is_still_credited() {
        let mut config = config_with(vec![
            node("prod", &[], &["out"]),
            node("sink", &[("gate", "/listed")], &[]),
        ]);
        config.nodes[0].outputs[0].topic = Some("/listed".to_string());
        config.multi_publisher_topics = vec!["/listed".to_string()];
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "sink".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", 5)], vec![]),
        );
        let topo = GraphTopology::build(&config, &infos).expect("topology");
        let edges = credit_edges_for(&topo, &ranks(&[("prod", 0), ("sink", 1)])).expect("mint");
        assert_eq!(
            shape_edges(&edges),
            vec![(
                "/listed".to_string(),
                "sink".to_string(),
                "gate".to_string(),
                5,
                vec![0],
                1
            )],
            "a LISTED topic with one in-graph producer is credited like any other"
        );
    }

    /// One producer, TWO `block` consumers in two FOREIGN ranks ⇒ two entries,
    /// each carrying its own consumer's depth and rank.
    #[test]
    fn one_producer_with_two_foreign_block_consumers_mints_two_entries() {
        let config = config_with(vec![
            node("prod", &[], &["out"]),
            node("s1", &[("gate", "prod/out")], &[]),
            node("s2", &[("gate", "prod/out")], &[]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "s1".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", 2)], vec![]),
        );
        infos.insert(
            "s2".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", 3)], vec![]),
        );
        let topo = GraphTopology::build(&config, &infos).expect("topology");
        let edges =
            credit_edges_for(&topo, &ranks(&[("prod", 0), ("s1", 1), ("s2", 2)])).expect("mint");
        assert_eq!(
            shape_edges(&edges),
            vec![
                (
                    "/p/prod/out".to_string(),
                    "s1".to_string(),
                    "gate".to_string(),
                    2,
                    vec![0],
                    1
                ),
                (
                    "/p/prod/out".to_string(),
                    "s2".to_string(),
                    "gate".to_string(),
                    3,
                    vec![0],
                    2
                ),
            ],
            "one entry per SPLIT consumer edge, each with its own depth and rank"
        );
    }

    /// A producer co-located with ONE consumer and split from ANOTHER mints
    /// exactly the split one.
    #[test]
    fn a_partly_co_located_block_topic_mints_only_the_split_edge() {
        let config = config_with(vec![
            node("prod", &[], &["out"]),
            node("near", &[("gate", "prod/out")], &[]),
            node("far", &[("gate", "prod/out")], &[]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "near".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", 2)], vec![]),
        );
        infos.insert(
            "far".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", 6)], vec![]),
        );
        let topo = GraphTopology::build(&config, &infos).expect("topology");
        let edges =
            credit_edges_for(&topo, &ranks(&[("prod", 0), ("near", 0), ("far", 1)])).expect("mint");
        assert_eq!(
            shape_edges(&edges),
            vec![(
                "/p/prod/out".to_string(),
                "far".to_string(),
                "gate".to_string(),
                6,
                vec![0],
                1
            )],
            "only the SPLIT consumer edge needs a word"
        );
    }

    /// An UNPLACED node (in neither rank) is SKIPPED, not refused —
    /// `validate_partition` reports an unplaced node separately, and this
    /// function's job is to describe the edges a valid partition splits.
    #[test]
    fn an_unplaced_node_is_skipped_rather_than_refused() {
        let topo = simple_block_topology(4);
        // Producer unplaced.
        assert!(credit_edges_for(&topo, &ranks(&[("sink", 1)]))
            .expect("mint")
            .is_empty());
        // Consumer unplaced.
        assert!(credit_edges_for(&topo, &ranks(&[("prod", 0)]))
            .expect("mint")
            .is_empty());
    }

    /// A depth of 0 is REFUSED loudly naming the edge, not clamped: a word
    /// stamped depth 0 reads FULL unconditionally, so its producer would defer
    /// forever with no drain that can recover it.
    #[test]
    fn a_zero_depth_edge_is_refused_naming_the_edge() {
        let topo = simple_block_topology(0);
        let err = credit_edges_for(&topo, &ranks(&[("prod", 0), ("sink", 1)]))
            .expect_err("depth 0 must be refused");
        for needle in ["/p/prod/out", "sink", "gate", "depth 0"] {
            assert!(
                err.contains(needle),
                "the refusal must name the edge (missing {needle:?}): {err}"
            );
        }
    }

    /// A depth that does not fit the `u32` a credit
    /// word stamps is refused, naming the edge.
    ///
    /// Unlike the runtime's twin — which sits below `GraphTopology::validate`'s
    /// `MAX_CONSUMER_DEPTH` cap and is therefore unreachable from a validated
    /// topology — the MINT is handed a topology directly, so this branch is
    /// reachable and worth an oracle. A saturating conversion here would stamp
    /// a ceiling the consumer never declared into a page a peer process then
    /// believes.
    #[test]
    fn a_depth_that_does_not_fit_a_u32_is_refused_naming_the_edge() {
        let config = config_with(vec![
            node("prod", &[], &["out"]),
            node("sink", &[("gate", "prod/out")], &[]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "sink".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", u32::MAX as usize + 1)], vec![]),
        );
        let topo = GraphTopology::build(&config, &infos).expect("topology");
        let err = credit_edges_for(&topo, &ranks(&[("prod", 0), ("sink", 1)]))
            .expect_err("a depth past u32 must be refused");
        for needle in ["/p/prod/out", "sink", "gate", "does not fit"] {
            assert!(
                err.contains(needle),
                "the refusal must name {needle:?}: {err}"
            );
        }
    }

    /// The mint's ORDER is the topology's deterministic graph order, and two
    /// runs over one topology are byte-identical (Principle #7: the plan file
    /// this feeds must be stable).
    #[test]
    fn the_mint_is_deterministic_and_follows_graph_order() {
        let config = config_with(vec![
            node("prod", &[], &["out"]),
            node("zeta", &[("gate", "prod/out")], &[]),
            node("alpha", &[("gate", "prod/out")], &[]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        for id in ["zeta", "alpha"] {
            infos.insert(
                id.to_string(),
                NodeInfo::with_meta(vec![block_meta("gate", 2)], vec![]),
            );
        }
        let topo = GraphTopology::build(&config, &infos).expect("topology");
        let r = ranks(&[("prod", 0), ("zeta", 1), ("alpha", 1)]);
        let a = credit_edges_for(&topo, &r).expect("mint");
        let b = credit_edges_for(&topo, &r).expect("mint");
        assert_eq!(shape_edges(&a), shape_edges(&b), "two mints are identical");
        assert_eq!(
            a.iter()
                .map(|e| e.consumer_node.as_str())
                .collect::<Vec<_>>(),
            vec!["zeta", "alpha"],
            "DECLARATION order, not alphabetical — a sort here would be a second, \
             silently-different answer to `which edge is which`"
        );
    }

    /// Ranks derived FROM the groups, exactly as the mint site does
    /// (`graph_cmd.rs`: one rank per worker, workers in declaration order).
    ///
    /// A seam arm that hand-writes both the groups AND the rank map would
    /// leave the group⟺rank lemma the validator delegates to its `pg == cg`
    /// arm ASSUMED by the very test that should cover it.
    fn ranks_from(
        groups: &IndexMap<String, Vec<String>>,
    ) -> std::collections::HashMap<&str, usize> {
        let mut out = std::collections::HashMap::new();
        for (rank, (_, members)) in groups.iter().enumerate() {
            for m in members {
                out.insert(m.as_str(), rank);
            }
        }
        out
    }

    // =================================================================
    // What a worker death means for the credit edges.
    // =================================================================

    fn death_edge(
        topic: &str,
        consumer: &str,
        producers: &[usize],
        consumer_rank: usize,
    ) -> CreditEdgePlan {
        CreditEdgePlan {
            topic: topic.to_string(),
            consumer_node: consumer.to_string(),
            consumer_input: "gate".to_string(),
            depth: 4,
            producer_ranks: producers.to_vec(),
            consumer_rank,
        }
    }

    fn no_prior_deaths() -> std::collections::BTreeSet<u32> {
        std::collections::BTreeSet::new()
    }

    /// P1/P5/P6: a dead CONSUMER is reported on its own edge only.
    #[test]
    fn a_dead_consumer_rank_is_reported_on_its_own_edge_only() {
        let edges = vec![
            death_edge("/a", "sink_a", &[0], 1),
            death_edge("/b", "sink_b", &[0], 2),
        ];
        assert_eq!(
            credit_death_actions(&edges, &[1], &no_prior_deaths()),
            vec![CreditDeathAction {
                edge_idx: 0,
                dead_rank: 1,
                role: CreditDeathRole::Consumer {
                    deferred_producers: vec![0]
                },
            }]
        );
        // P5: a rank on NO credit edge yields nothing — the negative control
        // without which "exactly one" passes against a reporter that fires on
        // every edge for every death.
        assert!(credit_death_actions(&edges, &[7], &no_prior_deaths()).is_empty());
        // P6: empty inputs, both ways.
        assert!(credit_death_actions(&[], &[1], &no_prior_deaths()).is_empty());
        assert!(credit_death_actions(&edges, &[], &no_prior_deaths()).is_empty());
    }

    /// P2: a dead PRODUCER yields the EDGE-LOCAL slot, not the global rank.
    ///
    /// The producer sits at rank 2 so the two numbers differ — at rank 0 this
    /// arm passes identically against a variant that hands
    /// `clear_parked_producer` the global rank, and every edge this build
    /// mints has exactly one producer, so slot 0 is the norm.
    #[test]
    fn a_dead_producer_rank_yields_its_edge_local_slot_not_the_rank() {
        let edges = vec![death_edge("/a", "sink", &[2], 1)];
        let actions = credit_death_actions(&edges, &[2], &no_prior_deaths());
        assert_eq!(actions.len(), 1);
        match &actions[0].role {
            CreditDeathRole::Producer { slot } => assert_eq!(
                slot.index(),
                0,
                "the SLOT is the position in `producer_ranks` (0) while the RANK is 2"
            ),
            other => panic!("expected a Producer action, got {other:?}"),
        }
    }

    /// P3: one rank, both roles, two actions — each with the right slot.
    ///
    /// The discriminator for a sweep keyed on `consumer_rank` instead of
    /// `producer_ranks`.
    #[test]
    fn a_rank_that_is_producer_on_one_edge_and_consumer_on_another_yields_both() {
        let edges = vec![
            // rank 2 PRODUCES here (slot 1 of two producers)...
            death_edge("/produced", "far_sink", &[5, 2], 3),
            // ...and CONSUMES here, with rank 0 surviving.
            death_edge("/consumed", "near_sink", &[0], 2),
        ];
        let actions = credit_death_actions(&edges, &[2], &no_prior_deaths());
        assert_eq!(actions.len(), 2, "both roles: {actions:?}");
        match &actions[0].role {
            CreditDeathRole::Producer { slot } => assert_eq!(
                slot.index(),
                1,
                "the POSITION of rank 2 among [5, 2], not the rank"
            ),
            other => panic!("expected Producer first, got {other:?}"),
        }
        assert_eq!(
            actions[1].role,
            CreditDeathRole::Consumer {
                deferred_producers: vec![0]
            }
        );
    }

    /// P4: two edges to ONE dead consumer ⇒ two actions, in plan order.
    #[test]
    fn two_edges_to_one_dead_consumer_are_both_reported_in_plan_order() {
        let edges = vec![
            death_edge("/zeta", "sink", &[0], 1),
            death_edge("/alpha", "sink", &[0], 1),
        ];
        assert_eq!(
            credit_death_actions(&edges, &[1], &no_prior_deaths())
                .iter()
                .map(|a| a.edge_idx)
                .collect::<Vec<_>>(),
            vec![0, 1],
            "PLAN order, not sorted — a sort here would be a second, silently \
             different answer to `which edge is which`"
        );
    }

    /// P7: a co-death in one batch retires nobody, so it
    /// yields no Consumer action.
    ///
    /// `sweep_additional_deaths` folds every death in the pass window into one
    /// batch, so a 2-rank `--peer-loss continue` deployment losing both ranks
    /// arrives here as a single call. Reporting "producer rank 0 is
    /// permanently deferred" would send the operator to look at a process that
    /// died in the same instant; the ordinary departure line already names
    /// both groups.
    #[test]
    fn a_consumer_death_with_no_surviving_producer_is_suppressed() {
        let edges = vec![death_edge("/a", "sink", &[0], 1)];
        assert!(
            credit_death_actions(&edges, &[0, 1], &no_prior_deaths())
                .iter()
                .all(|a| !matches!(a.role, CreditDeathRole::Consumer { .. })),
            "a fully-dead edge stranded nobody"
        );
        // ANTI-VACUITY: the PRODUCER half still fires for rank 0 — the
        // suppression is of the consumer claim, not of the whole batch.
        assert!(
            credit_death_actions(&edges, &[0, 1], &no_prior_deaths())
                .iter()
                .any(|a| matches!(a.role, CreditDeathRole::Producer { .. })),
            "the dead producer's stale bit must still be swept"
        );
    }

    /// P8: the dead set is cumulative across passes.
    ///
    /// The producer dies in pass 1 and the consumer in pass 2. Only a
    /// cumulative set can tell that the "surviving" producer died a moment
    /// ago; a within-batch filter alone names a corpse.
    #[test]
    fn a_producer_that_died_in_an_earlier_pass_is_not_named_as_a_survivor() {
        let edges = vec![death_edge("/a", "sink", &[0], 1)];
        let mut dead_so_far = no_prior_deaths();
        // Pass 1: the producer dies.
        let _ = credit_death_actions(&edges, &[0], &dead_so_far);
        dead_so_far.insert(0);
        // Pass 2: the consumer dies. Nobody is left to defer.
        assert!(
            credit_death_actions(&edges, &[1], &dead_so_far)
                .iter()
                .all(|a| !matches!(a.role, CreditDeathRole::Consumer { .. })),
            "the producer died in pass 1 — it cannot be a surviving deferred producer"
        );
        // Anti-vacuity: with an empty prior set the same pass 2 call does
        // report it, so this arm is about the cumulative set and nothing else.
        assert!(
            credit_death_actions(&edges, &[1], &no_prior_deaths())
                .iter()
                .any(|a| matches!(a.role, CreditDeathRole::Consumer { .. })),
            "without the cumulative set the very same batch names the dead producer"
        );
    }

    /// P9: with two producers and one dead, the warn names
    /// ONLY the live one.
    #[test]
    fn a_consumer_death_names_only_the_producers_still_alive() {
        // `multi_publisher_topics` shape: two in-graph producers.
        let edges = vec![death_edge("/a", "sink", &[0, 4], 1)];
        let mut dead_so_far = no_prior_deaths();
        dead_so_far.insert(0);
        let actions = credit_death_actions(&edges, &[1], &dead_so_far);
        let consumer = actions
            .iter()
            .find_map(|a| match &a.role {
                CreditDeathRole::Consumer { deferred_producers } => Some(deferred_producers),
                _ => None,
            })
            .expect("one producer survives, so the edge DID retire somebody");
        assert_eq!(
            consumer,
            &vec![4],
            "only rank 4 is not known dead; naming rank 0 would point at a dead process"
        );
        // ANTI-VACUITY: with an EMPTY prior set the SAME batch names BOTH
        // producers — so the filtering above is the dead set doing its job,
        // not the fixture happening to have one producer.
        let both = credit_death_actions(&edges, &[1], &no_prior_deaths());
        let named = both
            .iter()
            .find_map(|a| match &a.role {
                CreditDeathRole::Consumer { deferred_producers } => Some(deferred_producers),
                _ => None,
            })
            .expect("with no prior deaths the edge strands both producers");
        assert_eq!(named, &vec![0, 4], "both, when neither is known dead");
    }

    /// The reconciliation over TWO metadata views:
    /// the seam the wiring shipped without.
    ///
    /// Every earlier arm drove ONE topology through both halves, so the
    /// reconciliation could be deleted or have its arguments swapped and no
    /// test would notice. This builds two topologies from two DIFFERENT infos
    /// maps — the shape a stale cdylib really produces — and asserts the
    /// refusal names the edge AND the direction.
    #[test]
    fn a_stale_cdylib_that_disagrees_with_its_source_refuses_pre_spawn() {
        let config = config_with(vec![
            node("prod", &[], &["out"]),
            node("sink", &[("gate", "prod/out")], &[]),
        ]);
        let mut infos_block: IndexMap<String, NodeInfo> = IndexMap::new();
        infos_block.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos_block.insert(
            "sink".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", 5)], vec![]),
        );
        let mut infos_plain = infos_block.clone();
        infos_plain.insert(
            "sink".to_string(),
            NodeInfo::with_meta(vec![plain_meta("gate")], vec![]),
        );
        let topo_block = GraphTopology::build(&config, &infos_block).expect("topology");
        let topo_plain = GraphTopology::build(&config, &infos_plain).expect("topology");
        let ranks = ranks(&[("prod", 0), ("sink", 1)]);

        // SOURCE says `block`, the BUILT cdylib says `drop_oldest`: accepted,
        // nothing minted, and the consumer's worker would die at build.
        let msg = reconcile_credit_edges(&topo_block, &topo_plain, &ranks, "g")
            .expect("mint")
            .expect("the two views disagree — this must refuse");
        assert!(msg.contains("'/p/prod/out' -> 'sink.gate'"), "{msg}");
        assert!(
            msg.contains("accepted at plan time but NOT minted"),
            "{msg}"
        );
        assert!(!msg.contains("minted but NOT accepted"), "{msg}");

        // The MIRROR direction: source `drop_oldest`, cdylib `block`. Nothing
        // was accepted, but the RUN treats the edge as `block` — so it would
        // run lossy on an edge declared lossless.
        let msg = reconcile_credit_edges(&topo_plain, &topo_block, &ranks, "g")
            .expect("mint")
            .expect("the mirror direction must refuse too");
        assert!(
            msg.contains("minted but NOT accepted at plan time"),
            "{msg}"
        );
        assert!(
            !msg.contains("accepted at plan time but NOT minted"),
            "{msg}"
        );

        // CONTROL: the two views AGREE ⇒ no refusal. Without this, a
        // reconciliation that refused unconditionally passes both arms above.
        assert!(
            reconcile_credit_edges(&topo_block, &topo_block, &ranks, "g")
                .expect("mint")
                .is_none(),
            "agreeing views must not refuse"
        );
        assert!(
            reconcile_credit_edges(&topo_plain, &topo_plain, &ranks, "g")
                .expect("mint")
                .is_none(),
            "two agreeing NON-block views mint nothing on both sides"
        );
    }

    /// The comparison is deliberately KEY-ONLY, and this is why.
    ///
    /// `source_entry_infos` parses the source, where the `depth` attribute may
    /// be absent, and stamps `DEFAULT_CONSUMER_DEPTH`. A full-plan equality
    /// would therefore refuse EVERY deployment whose `block` input declares a
    /// non-default depth — a refusal with no defect behind it. The word is
    /// stamped from the LOADED depth by design.
    #[test]
    fn a_depth_that_differs_between_the_two_views_is_not_a_drift_refusal() {
        let config = config_with(vec![
            node("prod", &[], &["out"]),
            node("sink", &[("gate", "prod/out")], &[]),
        ]);
        let build = |depth: usize| {
            let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
            infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
            infos.insert(
                "sink".to_string(),
                NodeInfo::with_meta(vec![block_meta("gate", depth)], vec![]),
            );
            GraphTopology::build(&config, &infos).expect("topology")
        };
        let ranks = ranks(&[("prod", 0), ("sink", 1)]);
        // Same EDGE, different declared depth on each side.
        assert!(
            reconcile_credit_edges(&build(10), &build(64), &ranks, "g")
                .expect("mint")
                .is_none(),
            "a depth difference is not an edge-set difference and must not refuse"
        );
        // ANTI-VACUITY: the depths really did differ in the minted plans.
        assert_ne!(
            credit_edges_for(&build(10), &ranks).expect("mint")[0].depth,
            credit_edges_for(&build(64), &ranks).expect("mint")[0].depth,
        );
    }

    /// The accepted-vs-minted reconciliation.
    ///
    /// Hand oracles over the four shapes: agreement, each direction alone, and
    /// both at once. The two directions have DIFFERENT consequences and the
    /// message must say which one happened — an operator reading "accepted but
    /// not minted" is about to lose a worker, while "minted but not accepted"
    /// means nothing checked the edge at all.
    #[test]
    fn credit_edge_drift_refusal_reports_each_direction_with_its_own_consequence() {
        let edge = |topic: &str, node: &str| CreditEdgePlan {
            topic: topic.to_string(),
            consumer_node: node.to_string(),
            consumer_input: "gate".to_string(),
            depth: 4,
            producer_ranks: vec![0],
            consumer_rank: 1,
        };
        let a = edge("/a", "sink_a");
        let b = edge("/b", "sink_b");

        // AGREEMENT — including the empty case, which is every healthy run.
        assert!(credit_edge_drift_refusal("g", &[], &[]).is_none());
        assert!(
            credit_edge_drift_refusal("g", std::slice::from_ref(&a), std::slice::from_ref(&a))
                .is_none()
        );
        // Order must not matter: these are SETS.
        assert!(
            credit_edge_drift_refusal("g", &[a.clone(), b.clone()], &[b.clone(), a.clone()])
                .is_none(),
            "the comparison is a set, not a sequence"
        );

        // ACCEPTED but NOT minted — the direction that kills a worker.
        let msg = credit_edge_drift_refusal("g", std::slice::from_ref(&a), &[])
            .expect("an accepted-but-unminted edge must refuse");
        assert!(msg.contains("'/a' -> 'sink_a.gate'"), "{msg}");
        assert!(
            msg.contains("accepted at plan time but NOT minted"),
            "{msg}"
        );
        assert!(
            msg.contains("exited before signaling READY"),
            "the message must name the failure the operator would otherwise see: {msg}"
        );
        assert!(
            !msg.contains("minted but NOT accepted"),
            "only the direction that happened may be reported: {msg}"
        );

        // MINTED but NOT accepted — the direction that runs unchecked.
        let msg = credit_edge_drift_refusal("g", &[], std::slice::from_ref(&a))
            .expect("a minted-but-unaccepted edge must refuse");
        assert!(
            msg.contains("minted but NOT accepted at plan time"),
            "{msg}"
        );
        assert!(
            !msg.contains("accepted at plan time but NOT minted"),
            "{msg}"
        );

        // BOTH at once: both clauses, each naming its own edge.
        let msg =
            credit_edge_drift_refusal("g", std::slice::from_ref(&a), std::slice::from_ref(&b))
                .expect("a two-way disagreement must refuse");
        assert!(msg.contains("'/a' -> 'sink_a.gate'"), "{msg}");
        assert!(msg.contains("'/b' -> 'sink_b.gate'"), "{msg}");
        assert!(
            msg.contains("accepted at plan time but NOT minted"),
            "{msg}"
        );
        assert!(
            msg.contains("minted but NOT accepted at plan time"),
            "{msg}"
        );

        // Every refusal carries the STALE-BUILD diagnosis and the remedy, or
        // the operator is told a fact with no action attached.
        for m in [
            credit_edge_drift_refusal("g", std::slice::from_ref(&a), &[]).unwrap(),
            credit_edge_drift_refusal("g", &[], std::slice::from_ref(&a)).unwrap(),
        ] {
            assert!(m.contains("STALE BUILD"), "{m}");
            assert!(m.contains("cerulion node build"), "{m}");
            assert!(m.contains("CERULION:INFO_START"), "{m}");
            assert!(
                m.contains("Refused before any worker was spawned"),
                "the refusal must say it cost nothing: {m}"
            );
        }
    }

    /// The refusal text is DETERMINISTIC — two calls on the same disagreement
    /// render byte-identically, whatever order the plans arrive in.
    #[test]
    fn credit_edge_drift_refusal_is_deterministic() {
        let edge = |topic: &str| CreditEdgePlan {
            topic: topic.to_string(),
            consumer_node: "sink".to_string(),
            consumer_input: "gate".to_string(),
            depth: 4,
            producer_ranks: vec![0],
            consumer_rank: 1,
        };
        let accepted = vec![edge("/z"), edge("/a"), edge("/m")];
        let one = credit_edge_drift_refusal("g", &accepted, &[]).unwrap();
        let mut shuffled = accepted.clone();
        shuffled.reverse();
        let two = credit_edge_drift_refusal("g", &shuffled, &[]).unwrap();
        assert_eq!(one, two, "the refusal must not depend on plan order");
        assert!(
            one.find("'/a'").unwrap() < one.find("'/m'").unwrap()
                && one.find("'/m'").unwrap() < one.find("'/z'").unwrap(),
            "and it renders in sorted order: {one}"
        );
    }

    /// The D1 edge list names topic AND consumer, in plan order.
    #[test]
    fn the_credit_edge_list_names_each_edge_not_a_count() {
        let edge = |topic: &str, node: &str| CreditEdgePlan {
            topic: topic.to_string(),
            consumer_node: node.to_string(),
            consumer_input: "gate".to_string(),
            depth: 4,
            producer_ranks: vec![0],
            consumer_rank: 1,
        };
        assert_eq!(render_credit_edge_list(&[]), "");
        assert_eq!(
            render_credit_edge_list(&[edge("/a", "one"), edge("/b", "two")]),
            "/a -> one.gate, /b -> two.gate"
        );
    }

    /// Plan-time ACCEPTANCE and the MINT must agree, edge
    /// for edge, over REAL topologies and REAL group→rank derivation.
    ///
    /// The two halves live in different crates —
    /// `partition::validate_block_colocation` decides whether the partition is
    /// refused, `credit_edges_for` decides whether a word is minted — and each
    /// crate's own oracle pins only its own side. This arm is the seam.
    ///
    /// Accepting more than the mint produces is the dangerous direction: the
    /// partition passes, the supervisor stamps nothing, and the consumer's
    /// worker dies at `GraphTopology::validate` blaming an external publisher
    /// the operator does not have.
    ///
    /// Ranks come from `ranks_from(&groups)`, not a hand-written map, so the
    /// group⟺rank lemma the validator delegates to its `pg == cg` arm is
    /// EXERCISED rather than assumed — and edge IDENTITY is compared via
    /// `shape_edges`, not a count, so a mint that produced the right NUMBER of
    /// wrong words fails.
    #[test]
    fn a_partition_the_validator_accepts_is_exactly_one_the_mint_credits() {
        use cerulion_core::graph::partition::validate_partition;
        use cerulion_core::graph::topology::TriggerEdges;

        // ---- (a) THE creditable shape: accepted, one word, the RIGHT word.
        let config = config_with(vec![
            node("prod", &[], &["out"]),
            node("sink", &[("gate", "prod/out")], &[]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "sink".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", 5)], vec![]),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("sink", "/p/prod/out");
        let split = groups(&[("front", &["prod"]), ("back", &["sink"])]);
        validate_partition(&split, &config, &infos, &edges)
            .expect("a single-producer all-`block` split edge is accepted");
        let topo = GraphTopology::build(&config, &infos).expect("topology");
        assert_eq!(
            shape_edges(&credit_edges_for(&topo, &ranks_from(&split)).expect("mint")),
            vec![(
                "/p/prod/out".to_string(),
                "sink".to_string(),
                "gate".to_string(),
                5u32,
                vec![0usize],
                1usize,
            )],
            "the accepted split must mint EXACTLY the word it was accepted for"
        );

        // ---- (b) the same graph CO-LOCATED: accepted, and nothing minted.
        // The mint's rank term is the validator's `pg == cg` arm, so this is
        // the row that proves they are the same condition rather than two that
        // happen to agree on the split case.
        let together = groups(&[("one", &["prod", "sink"])]);
        validate_partition(&together, &config, &infos, &edges).expect("co-located is fine");
        assert!(
            credit_edges_for(&topo, &ranks_from(&together))
                .expect("mint")
                .is_empty(),
            "a co-located edge needs no cross-process word"
        );

        // ---- (c) FAN-OUT: one producer, two consumers in two FOREIGN groups
        // ⇒ accepted, and TWO words (one per consumer).
        let fan_config = config_with(vec![
            node("prod", &[], &["out"]),
            node("s1", &[("gate", "prod/out")], &[]),
            node("s2", &[("gate", "prod/out")], &[]),
        ]);
        let mut fan_infos: IndexMap<String, NodeInfo> = IndexMap::new();
        fan_infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        for c in ["s1", "s2"] {
            fan_infos.insert(
                c.to_string(),
                NodeInfo::with_meta(vec![block_meta("gate", 2)], vec![]),
            );
        }
        let mut fan_edges = TriggerEdges::new();
        fan_edges.insert("s1", "/p/prod/out");
        fan_edges.insert("s2", "/p/prod/out");
        let fan_split = groups(&[("g0", &["prod"]), ("g1", &["s1"]), ("g2", &["s2"])]);
        validate_partition(&fan_split, &fan_config, &fan_infos, &fan_edges)
            .expect("a fanned-out creditable topic is accepted");
        let fan_topo = GraphTopology::build(&fan_config, &fan_infos).expect("topology");
        let fan_minted = credit_edges_for(&fan_topo, &ranks_from(&fan_split)).expect("mint");
        assert_eq!(
            fan_minted
                .iter()
                .map(|e| (e.consumer_node.as_str(), e.consumer_rank))
                .collect::<Vec<_>>(),
            vec![("s1", 1), ("s2", 2)],
            "one word per SPLIT consumer, in declaration order"
        );

        // ---- (d) MIXED partial co-location: the producer sits with ONE
        // consumer and is split from the other. Still creditable (one
        // producer, all-`block`), so accepted, and exactly ONE word — for the
        // consumer that is actually across a boundary.
        let partial = groups(&[("g0", &["prod", "s1"]), ("g1", &["s2"])]);
        validate_partition(&partial, &fan_config, &fan_infos, &fan_edges).expect("accepted");
        let partial_minted = credit_edges_for(&fan_topo, &ranks_from(&partial)).expect("mint");
        assert_eq!(
            partial_minted
                .iter()
                .map(|e| e.consumer_node.as_str())
                .collect::<Vec<_>>(),
            vec!["s2"],
            "only the SPLIT consumer needs a word"
        );

        // ---- (e) MULTI-PRODUCER: refused, and nothing minted.
        let mp_config = GraphConfig {
            multi_publisher_topics: vec!["/shared".to_string()],
            ..config_with(vec![
                node_out_at("pa", "/shared"),
                node_out_at("pb", "/shared"),
                node("sink", &[("gate", "/shared")], &[]),
            ])
        };
        let mut mp_infos: IndexMap<String, NodeInfo> = IndexMap::new();
        for p in ["pa", "pb"] {
            mp_infos.insert(p.to_string(), NodeInfo::with_meta(vec![], vec![]));
        }
        mp_infos.insert(
            "sink".to_string(),
            NodeInfo::with_meta(vec![block_meta("gate", 3)], vec![]),
        );
        let mut mp_edges = TriggerEdges::new();
        mp_edges.insert("sink", "/shared");
        let mp_split = groups(&[("front", &["pa", "pb"]), ("back", &["sink"])]);
        validate_partition(&mp_split, &mp_config, &mp_infos, &mp_edges)
            .expect_err("a split MULTI-PRODUCER block edge is not creditable");
        let mp_topo = GraphTopology::build(&mp_config, &mp_infos).expect("topology");
        assert!(
            credit_edges_for(&mp_topo, &ranks_from(&mp_split))
                .expect("mint")
                .is_empty(),
            "the refused split must mint nothing"
        );

        // ---- (f) MIXED topic: refused, and nothing minted. Same producer,
        // same `block` consumer, one extra `drop_oldest` sibling — so the ONLY
        // difference from (a) is the conjunct both halves key on.
        let mixed_config = config_with(vec![
            node("prod", &[], &["out"]),
            node("sink", &[("gate", "prod/out")], &[]),
            node("lossy", &[("watch", "prod/out")], &[]),
        ]);
        let mut mixed_infos = infos.clone();
        mixed_infos.insert(
            "lossy".to_string(),
            NodeInfo::with_meta(vec![plain_meta("watch")], vec![]),
        );
        let mut mixed_edges = TriggerEdges::new();
        mixed_edges.insert("sink", "/p/prod/out");
        mixed_edges.insert("lossy", "/p/prod/out");
        let mixed_split = groups(&[("front", &["prod", "lossy"]), ("back", &["sink"])]);
        validate_partition(&mixed_split, &mixed_config, &mixed_infos, &mixed_edges)
            .expect_err("a MIXED topic's split `block` edge is not creditable");
        let mixed_topo = GraphTopology::build(&mixed_config, &mixed_infos).expect("topology");
        assert!(
            credit_edges_for(&mixed_topo, &ranks_from(&mixed_split))
                .expect("mint")
                .is_empty(),
            "the refused split must mint nothing"
        );
    }

    fn groups(pairs: &[(&str, &[&str])]) -> IndexMap<String, Vec<String>> {
        pairs
            .iter()
            .map(|(name, members)| {
                (
                    name.to_string(),
                    members.iter().map(|s| s.to_string()).collect(),
                )
            })
            .collect()
    }

    /// A 5-node single chain n0 -> n1 -> n2 -> n3 -> n4 (global levels 0..4)
    /// with the given `process_groups`.
    fn chain_config(process_groups: IndexMap<String, Vec<String>>) -> GraphConfig {
        GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "chain".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                node("n0", &[], &["out"]),
                node("n1", &[("inp", "n0/out")], &["out"]),
                node("n2", &[("inp", "n1/out")], &["out"]),
                node("n3", &[("inp", "n2/out")], &["out"]),
                node("n4", &[("inp", "n3/out")], &[]),
            ],
            multi_publisher_topics: vec!["/tf".to_string()],
            process_groups,
            process_group_order: Vec::new(),
        }
    }

    /// A 4-node DIAMOND n0 -> {n1, n2} -> n3 (global levels 0,1,1,2).
    fn diamond_config(process_groups: IndexMap<String, Vec<String>>) -> GraphConfig {
        GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "diamond".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                node("n0", &[], &["out"]),
                node("n1", &[("inp", "n0/out")], &["out"]),
                node("n2", &[("inp", "n0/out")], &["out"]),
                node("n3", &[("a", "n1/out"), ("b", "n2/out")], &[]),
            ],
            multi_publisher_topics: Vec::new(),
            process_groups,
            process_group_order: Vec::new(),
        }
    }

    /// Build the GLOBAL Kahn levelization for `config`, treating each listed
    /// `(consumer, resolved-topic)` pair as a TRIGGERING edge. Mirrors
    /// `partition_test.rs::levels_for`. NodeInfo is empty (topology consumer
    /// edges come from the YAML `inputs`, not macro metadata).
    fn levels_for(config: &GraphConfig, trigger_edges: &[(&str, &str)]) -> Levels {
        let entry_infos: IndexMap<String, NodeInfo> = config
            .nodes
            .iter()
            .map(|n| (n.id.clone(), NodeInfo::with_meta(Vec::new(), Vec::new())))
            .collect();
        let topo = GraphTopology::build(config, &entry_infos).expect("topology build");
        let mut edges = TriggerEdges::new();
        for (consumer, topic) in trigger_edges {
            edges.insert(*consumer, *topic);
        }
        topo.derive_levels(&edges).expect("derive levels")
    }

    fn chain_levels(config: &GraphConfig) -> Levels {
        levels_for(
            config,
            &[
                ("n1", "/p/n0/out"),
                ("n2", "/p/n1/out"),
                ("n3", "/p/n2/out"),
                ("n4", "/p/n3/out"),
            ],
        )
    }

    fn diamond_levels(config: &GraphConfig) -> Levels {
        levels_for(
            config,
            &[
                ("n1", "/p/n0/out"),
                ("n2", "/p/n0/out"),
                ("n3", "/p/n1/out"),
                ("n3", "/p/n2/out"),
            ],
        )
    }

    /// Assert a participant-map's non-`None` entries are a strictly-increasing
    /// bijection onto `0..count` (the contiguous-split contract).
    fn assert_contiguous_local_bijection(map: &[Option<usize>]) {
        let locals: Vec<usize> = map.iter().filter_map(|o| *o).collect();
        let expected: Vec<usize> = (0..locals.len()).collect();
        assert_eq!(
            locals,
            expected,
            "non-None entries must be a strictly-increasing bijection onto 0..{}",
            locals.len()
        );
    }

    fn subgraph_node_ids(w: &WorkerPlan) -> Vec<String> {
        w.subgraph.nodes.iter().map(|n| n.id.clone()).collect()
    }

    // ===================================================================
    // Oracle-vector tests.
    // ===================================================================

    /// A worker's identity survives the plan's JSON hop.
    ///
    /// `GraphConfig::identity` is `#[serde(skip)]` — it describes how a config
    /// was LOADED, not the document — so the subgraph the supervisor stamps
    /// into a `WorkerPlan` arrives at `graph run-worker` with its identity GONE.
    /// `WorkerPlan::graph_identity` is what carries it across, and this test
    /// drives the exact round trip a worker really performs: the supervisor
    /// serializes the plan to `plan_<rank>.json`, the worker deserializes it.
    ///
    /// Without the carry, EVERY multi-process worker logs `graph=unnamed` for
    /// the whole run — a silent, run-long identity loss on the DEFAULT Unix
    /// path, and one no monolith test can see.
    #[test]
    fn a_workers_identity_survives_the_plan_json_hop() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "hopnonce").expect("plan");

        let w0 = &plan.workers[0];
        // The supervisor stamps `{graph}_{group}` — the parent's FILE-stemmed
        // identity, not its deprecated key.
        assert_eq!(w0.graph_identity, "chain_P0");
        assert_eq!(
            w0.subgraph.identity(),
            "chain_P0",
            "the in-memory subgraph carries it too"
        );

        // THE hop: exactly what `graph run-worker` reads back off disk.
        let json = serde_json::to_string(w0).expect("serialize plan");
        let back: WorkerPlan = serde_json::from_str(&json).expect("deserialize plan");

        // The property that makes the carry NECESSARY — stated so a future
        // reader cannot mistake `graph_identity` for redundant bookkeeping.
        assert_eq!(
            back.subgraph.identity(),
            cerulion_core::graph::UNNAMED_GRAPH,
            "`identity` is `serde(skip)`, so the subgraph loses it across the hop"
        );
        assert!(
            back.subgraph.name.is_none(),
            "a DERIVED subgraph carries no deprecated key to fall back on either"
        );
        // …and the property that makes it SUFFICIENT.
        assert_eq!(back.graph_identity, "chain_P0");
        assert!(
            json.contains("\"graph_identity\":\"chain_P0\""),
            "the identity must really be ON the wire, got:\n{json}"
        );
    }

    /// 2-way split of the 5-node chain against the HAND oracle hand-pasted in
    /// `barrier_level_gate_iox2_test.rs` (MAP_A / MAP_B).
    #[test]
    fn plan_two_way_chain_matches_oracle() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        assert_eq!(levels.len(), 5, "chain must levelize to 5 global levels");

        let plan = plan_deployment(&config, &levels, None, "nonceX").expect("plan");

        assert_eq!(plan.workers.len(), 2);
        assert_eq!(plan.expected, 2);
        assert_eq!(plan.barrier_ns, "cerdep_chain_nonceX");
        assert_eq!(plan.barrier_id, "chain_levelgate");

        // Worker 0 (P0) — owns globals 0,1.
        let w0 = &plan.workers[0];
        assert_eq!(w0.group, "P0");
        assert_eq!(w0.rank, 0);
        assert_eq!(
            w0.global_level_map,
            vec![Some(0), Some(1), None, None, None],
            "MAP_A oracle"
        );
        assert_contiguous_local_bijection(&w0.global_level_map);
        assert_eq!(w0.node_name, "cerulion_chain_P0");
        assert_eq!(w0.ready_path, "cerulion_chain_P0_nonceX.ready");
        assert_eq!(w0.handed_quantum_ns, HANDED_QUANTUM_FLOOR_NS); // no timing -> 1ms floor
        assert_eq!(subgraph_node_ids(w0), vec!["n0", "n1"]);

        // Worker 1 (P1) — owns globals 2,3,4.
        let w1 = &plan.workers[1];
        assert_eq!(w1.group, "P1");
        assert_eq!(w1.rank, 1);
        assert_eq!(
            w1.global_level_map,
            vec![None, None, Some(0), Some(1), Some(2)],
            "MAP_B oracle"
        );
        assert_contiguous_local_bijection(&w1.global_level_map);
        assert_eq!(w1.node_name, "cerulion_chain_P1");
        assert_eq!(subgraph_node_ids(w1), vec!["n2", "n3", "n4"]);

        // Shared barrier identifiers echo on every worker.
        for w in &plan.workers {
            assert_eq!(w.barrier_ns, plan.barrier_ns);
            assert_eq!(w.barrier_id, plan.barrier_id);
            assert_eq!(w.handed_quantum_ns, HANDED_QUANTUM_FLOOR_NS);
        }
    }

    /// 3-way split of the 5-node chain: {A:[n0], B:[n1,n2], C:[n3,n4]} against a
    /// hand oracle — every group owns a CONTIGUOUS band.
    #[test]
    fn plan_three_way_chain_matches_oracle() {
        let config = chain_config(groups(&[
            ("A", &["n0"]),
            ("B", &["n1", "n2"]),
            ("C", &["n3", "n4"]),
        ]));
        let levels = chain_levels(&config);

        let plan = plan_deployment(&config, &levels, None, "n").expect("plan");
        assert_eq!(plan.workers.len(), 3);
        assert_eq!(plan.expected, 3);

        // Per-worker hand oracle (name, rank, participant-map, owned node ids).
        let check =
            |w: &WorkerPlan, name: &str, rank: usize, map: &[Option<usize>], ids: &[&str]| {
                assert_eq!(w.group, name);
                assert_eq!(w.rank, rank);
                assert_eq!(w.global_level_map, map);
                assert_contiguous_local_bijection(&w.global_level_map);
                assert_eq!(subgraph_node_ids(w), ids);
                assert_eq!(w.node_name, format!("cerulion_chain_{name}"));
            };
        check(
            &plan.workers[0],
            "A",
            0,
            &[Some(0), None, None, None, None],
            &["n0"],
        );
        check(
            &plan.workers[1],
            "B",
            1,
            &[None, Some(0), Some(1), None, None],
            &["n1", "n2"],
        );
        check(
            &plan.workers[2],
            "C",
            2,
            &[None, None, None, Some(0), Some(1)],
            &["n3", "n4"],
        );
    }

    /// A WIDE global level (diamond: n1 AND n2 at level 1) split across two
    /// groups — both groups own the SAME global level 1 (legal: a level may be
    /// co-owned; the barrier still rendezvouses both there).
    #[test]
    fn plan_diamond_shared_wide_level() {
        // P0:[n0,n1] owns globals 0,1 ; P1:[n2,n3] owns globals 1,2.
        let config = diamond_config(groups(&[("P0", &["n0", "n1"]), ("P1", &["n2", "n3"])]));
        let levels = diamond_levels(&config);
        assert_eq!(levels.len(), 3, "diamond levelizes to 3 global levels");

        let plan = plan_deployment(&config, &levels, None, "d").expect("plan");
        assert_eq!(plan.workers.len(), 2);

        // P0 owns {0,1} contiguous.
        assert_eq!(
            plan.workers[0].global_level_map,
            vec![Some(0), Some(1), None]
        );
        assert_contiguous_local_bijection(&plan.workers[0].global_level_map);
        // P1 owns {1,2} contiguous.
        assert_eq!(
            plan.workers[1].global_level_map,
            vec![None, Some(0), Some(1)]
        );
        assert_contiguous_local_bijection(&plan.workers[1].global_level_map);
    }

    /// `process_group_order` overrides the rank order (not declaration order).
    #[test]
    fn plan_honors_explicit_process_group_order() {
        let mut config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        // Reverse the ranks explicitly.
        config.process_group_order = vec!["P1".to_string(), "P0".to_string()];
        let levels = chain_levels(&config);

        let plan = plan_deployment(&config, &levels, None, "z").expect("plan");
        // Rank AND the full per-worker payload must follow the reordered rank —
        // NOT declaration order. Pinning only rank/group would miss a desync where
        // rank follows `process_group_order` while the participant-map / subgraph /
        // node_name follow declaration order (a cross-process lockstep corruption).
        // workers[0] = P1 (rank 0): MAP_B + P1's nodes + P1's node_name.
        assert_eq!(plan.workers[0].group, "P1");
        assert_eq!(plan.workers[0].rank, 0);
        assert_eq!(
            plan.workers[0].global_level_map,
            vec![None, None, Some(0), Some(1), Some(2)],
            "reordered worker[0]=P1 must carry MAP_B, not P0's MAP_A"
        );
        assert_eq!(subgraph_node_ids(&plan.workers[0]), vec!["n2", "n3", "n4"]);
        assert_eq!(plan.workers[0].node_name, "cerulion_chain_P1");
        // workers[1] = P0 (rank 1): MAP_A + P0's nodes + P0's node_name.
        assert_eq!(plan.workers[1].group, "P0");
        assert_eq!(plan.workers[1].rank, 1);
        assert_eq!(
            plan.workers[1].global_level_map,
            vec![Some(0), Some(1), None, None, None],
            "reordered worker[1]=P0 must carry MAP_A, not P1's MAP_B"
        );
        assert_eq!(subgraph_node_ids(&plan.workers[1]), vec!["n0", "n1"]);
        assert_eq!(plan.workers[1].node_name, "cerulion_chain_P0");
    }

    /// Subgraph split preserves each group's node set + verbatim wiring, drops
    /// foreign nodes, preserves prefix + multi_publisher_topics, and clears the
    /// partition fields.
    #[test]
    fn subgraph_split_preserves_wiring_and_drops_foreign() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "s").expect("plan");

        let sub0 = &plan.workers[0].subgraph;
        // Only P0's nodes, in graph order; foreign nodes dropped.
        assert_eq!(subgraph_node_ids(&plan.workers[0]), vec!["n0", "n1"]);
        // Verbatim wiring: n1 still consumes n0/out.
        let n1 = sub0.nodes.iter().find(|n| n.id == "n1").unwrap();
        assert_eq!(n1.inputs.len(), 1);
        assert_eq!(n1.inputs[0].source, "n0/out");
        assert_eq!(n1.outputs.len(), 1);
        assert_eq!(n1.outputs[0].name, "out");
        // Preserved parent fields.
        assert_eq!(sub0.prefix, "p");
        assert_eq!(sub0.multi_publisher_topics, vec!["/tf".to_string()]);
        // Cleared partition fields (worker is a monolith).
        assert!(sub0.process_groups.is_empty());
        assert!(sub0.process_group_order.is_empty());
        // The subgraph's identity carries the group; the file-stem rule leaves
        // the deprecated key absent on a DERIVED graph (there is no file to
        // round-trip it back to).
        assert_eq!(sub0.identity(), "chain_P0");
        assert!(sub0.name.is_none());

        // A cross-group consumer (P1's n2 reads n1/out, produced in P0) has its
        // source ABSOLUTIZED to `/p/n1/out` — the producer n1 is FOREIGN to P1,
        // so a verbatim relative `n1/out` would hard-fail P1's `validate_graph`;
        // the resolved absolute topic instead validates as an external
        // (cross-process) source that P0's worker publishes.
        let sub1 = &plan.workers[1].subgraph;
        let n2 = sub1.nodes.iter().find(|n| n.id == "n2").unwrap();
        assert_eq!(n2.inputs[0].source, "/p/n1/out");
        // Absolutization AND foreignness pinned TOGETHER: n2's source is rewritten
        // to `/p/n1/out` AND the producer n1 is DROPPED from P1's subgraph — that
        // is precisely what makes `/p/n1/out` an EXTERNAL (cross-group) source
        // rather than an in-group edge. A regression pulling referenced-but-
        // unassigned producers into the subgraph would break this.
        assert!(
            sub1.nodes.iter().all(|n| n.id != "n1"),
            "the cross-group producer n1 must be FOREIGN to P1's subgraph"
        );
        assert_eq!(sub1.prefix, "p");
    }

    /// `subgraph_for` (direct): a CROSS-GROUP relative source is rewritten to its
    /// resolved absolute topic (incl. the full multi-segment `/prefix/node/out`
    /// path), while IN-GROUP relative sources stay verbatim.
    #[test]
    fn subgraph_for_absolutizes_cross_group_relative_source() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        // Split P1 directly (members n2,n3,n4).
        let members = vec!["n2".to_string(), "n3".to_string(), "n4".to_string()];
        let sub = subgraph_for(&config, "P1", &members);

        // n2 reads n1/out — n1 is FOREIGN to P1 ⇒ absolutized to the full topic.
        let n2 = sub.nodes.iter().find(|n| n.id == "n2").unwrap();
        assert_eq!(
            n2.inputs[0].source, "/p/n1/out",
            "cross-group relative source must become its resolved absolute topic"
        );
        // n3 reads n2/out — n2 IS in P1 ⇒ verbatim relative.
        let n3 = sub.nodes.iter().find(|n| n.id == "n3").unwrap();
        assert_eq!(
            n3.inputs[0].source, "n2/out",
            "in-group edge stays relative"
        );
        // n4 reads n3/out — n3 IS in P1 ⇒ verbatim relative.
        let n4 = sub.nodes.iter().find(|n| n.id == "n4").unwrap();
        assert_eq!(
            n4.inputs[0].source, "n3/out",
            "in-group edge stays relative"
        );
    }

    /// The plan names each worker's cross-group input edges, and only those.
    ///
    /// P1's `n2` reads `n1/out` from P0, so P1 carries `/p/n1/out`; P0 consumes
    /// nothing from a sibling and carries nothing. `n3` and `n4` read in-group,
    /// so their topics are absent: a set that listed every sibling OUTPUT would
    /// also pass the validation test below, and fail here.
    #[test]
    fn the_plan_names_each_workers_cross_group_input_topics() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let plan = plan_deployment(&config, &chain_levels(&config), None, "st").expect("plan");
        let named = |w: &WorkerPlan| w.sibling_topics.iter().cloned().collect::<Vec<_>>();
        assert_eq!(named(&plan.workers[0]), Vec::<String>::new());
        assert_eq!(named(&plan.workers[1]), vec!["/p/n1/out".to_string()]);

        // An absolute source written by hand counts when a sibling produces it
        // (`/p/n0/out`), and an external one (`/ext/cam`) never does.
        let mut by_hand = config.clone();
        by_hand.nodes[4].inputs.push(InputDef {
            name: "by_hand".to_string(),
            source: "/p/n0/out".to_string(),
        });
        by_hand.nodes[4].inputs.push(InputDef {
            name: "cam".to_string(),
            source: "/ext/cam".to_string(),
        });
        let members = vec!["n2".to_string(), "n3".to_string(), "n4".to_string()];
        assert_eq!(
            sibling_topics_for(&by_hand, &members)
                .into_iter()
                .collect::<Vec<_>>(),
            vec!["/p/n0/out".to_string(), "/p/n1/out".to_string()]
        );
    }

    /// THE symptom, on the REAL worker view: a correct graph, split, validated
    /// the way `graph run-worker` validates it, must not be told its wiring may
    /// be a typo. The control is the same view validated with no set, which is
    /// what every worker did before and what an old plan file still yields.
    #[test]
    #[tracing_test::traced_test]
    fn a_planned_worker_view_validates_without_the_typo_warning() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let plan = plan_deployment(&config, &chain_levels(&config), None, "tw").expect("plan");
        let worker = &plan.workers[1];
        let warns = |lines: &[&str]| {
            lines
                .iter()
                .filter(|l| l.contains("matches no declared output"))
                .count()
        };

        cerulion_core::graph::validate_graph_with(
            &worker.subgraph,
            cerulion_core::graph::ValidationOptions {
                sibling_topics: Some(&worker.sibling_topics),
                ..Default::default()
            },
        )
        .expect("the worker view validates");
        logs_assert(|lines: &[&str]| match warns(lines) {
            0 => Ok(()),
            n => Err(format!("a correct split graph drew {n} typo warning(s)")),
        });

        cerulion_core::graph::validate_graph(&worker.subgraph).expect("validates");
        logs_assert(|lines: &[&str]| match warns(lines) {
            1 => Ok(()),
            n => Err(format!(
                "the no-set control must warn exactly once, got {n}"
            )),
        });
    }

    /// `subgraph_for` (direct): ABSOLUTE sources pass through verbatim — both a
    /// plain external topic (`/ext/cam`) and a `topic:`-override reference
    /// (`/tf`, which `validate_graph` forces absolute in the full config), so the
    /// single passthrough assertion covers the override case too.
    #[test]
    fn subgraph_for_passes_absolute_sources_through_unchanged() {
        let config = GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "abs".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![NodeDef {
                ros2: None,
                id: "sink".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![
                    InputDef {
                        name: "cam".to_string(),
                        source: "/ext/cam".to_string(),
                    },
                    InputDef {
                        name: "tf".to_string(),
                        source: "/tf".to_string(),
                    },
                ],
                outputs: Vec::new(),
            }],
            multi_publisher_topics: vec!["/tf".to_string()],
            process_groups: groups(&[("G", &["sink"])]),
            process_group_order: Vec::new(),
        };
        let members = vec!["sink".to_string()];
        let sub = subgraph_for(&config, "G", &members);
        let sink = sub.nodes.iter().find(|n| n.id == "sink").unwrap();
        assert_eq!(
            sink.inputs[0].source, "/ext/cam",
            "absolute external source unchanged"
        );
        assert_eq!(
            sink.inputs[1].source, "/tf",
            "absolute topic-override source unchanged"
        );
    }

    /// `spawn_order`: workers already in level order (rank == level order) spawn
    /// producers-first.
    #[test]
    fn spawn_order_follows_min_owned_global_level() {
        let config = chain_config(groups(&[
            ("A", &["n0"]),       // owns global 0
            ("B", &["n1", "n2"]), // owns globals 1,2
            ("C", &["n3", "n4"]), // owns globals 3,4
        ]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "so").expect("plan");
        assert_eq!(spawn_order(&plan), vec![0, 1, 2]);
    }

    /// `spawn_order`: even when `process_group_order` REVERSES rank vs level
    /// order, spawn order follows the MIN owned global level (producers first),
    /// NOT rank — so `workers[1]` (P0, level 0) spawns before `workers[0]` (P1,
    /// level 2).
    #[test]
    fn spawn_order_is_level_order_even_when_rank_reversed() {
        let mut config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),       // owns globals 0,1
            ("P1", &["n2", "n3", "n4"]), // owns globals 2,3,4
        ]));
        config.process_group_order = vec!["P1".to_string(), "P0".to_string()];
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "so").expect("plan");
        // workers[0] = P1 (rank 0, min level 2); workers[1] = P0 (rank 1, min 0).
        assert_eq!(plan.workers[0].group, "P1");
        assert_eq!(plan.workers[1].group, "P0");
        assert_eq!(
            spawn_order(&plan),
            vec![1, 0],
            "producer group P0 (index 1, level 0) must spawn before consumer P1 (index 0, level 2)"
        );
    }

    /// `spawn_order`: a single group is the trivial `[0]`.
    #[test]
    fn spawn_order_single_group() {
        let config = chain_config(groups(&[("only", &["n0", "n1", "n2", "n3", "n4"])]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "so").expect("plan");
        assert_eq!(spawn_order(&plan), vec![0]);
    }

    /// Every worker of a deployment shares ONE deployment-wide GO sentinel (the
    /// start gate the supervisor releases once ALL workers are READY), and it
    /// differs from EVERY per-worker READY sentinel (per-group names).
    #[test]
    fn go_path_is_shared_and_distinct_from_every_ready_path() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "gp").expect("plan");

        // Exact oracle for the bare filename.
        assert_eq!(plan.workers[0].go_path, "cerulion_chain_gp.go");
        // Shared: identical on every worker (one fs::write releases them all).
        for w in &plan.workers {
            assert_eq!(
                w.go_path, plan.workers[0].go_path,
                "all workers must share ONE deployment-wide GO sentinel"
            );
        }
        // Distinct from EVERY ready_path (cross-product, not just each worker's own).
        for w in &plan.workers {
            for peer in &plan.workers {
                assert_ne!(
                    w.go_path, peer.ready_path,
                    "the GO sentinel must never collide with a READY sentinel"
                );
            }
        }
    }

    /// `go_deadline_ms` composes with the GROUP COUNT (oracle arithmetic:
    /// `GO_BASE_MS + n * READY_BUDGET_MS`), dominating `n * READY_BUDGET_MS` —
    /// the first-spawned worker's legitimate GO wait spans every later
    /// sibling's sequential READY window, so a fixed deadline would spuriously
    /// fail-loud a >=4-group slow deployment.
    #[test]
    fn go_deadline_composes_with_group_count() {
        // 2 groups: 120_000 + 2 * 60_000 = 240_000.
        let config2 = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels2 = chain_levels(&config2);
        let plan2 = plan_deployment(&config2, &levels2, None, "gd").expect("plan");
        for w in &plan2.workers {
            assert_eq!(w.go_deadline_ms, 240_000, "2-group oracle");
            assert!(
                w.go_deadline_ms > 2 * READY_BUDGET_MS,
                "the GO deadline must dominate n * READY_BUDGET_MS"
            );
        }

        // 5 groups (one node each): 120_000 + 5 * 60_000 = 420_000.
        let config5 = chain_config(groups(&[
            ("A", &["n0"]),
            ("B", &["n1"]),
            ("C", &["n2"]),
            ("D", &["n3"]),
            ("E", &["n4"]),
        ]));
        let levels5 = chain_levels(&config5);
        let plan5 = plan_deployment(&config5, &levels5, None, "gd").expect("plan");
        for w in &plan5.workers {
            assert_eq!(w.go_deadline_ms, 420_000, "5-group oracle");
            assert!(
                w.go_deadline_ms > 5 * READY_BUDGET_MS,
                "the GO deadline must dominate n * READY_BUDGET_MS"
            );
        }
    }

    /// Sentinel FILENAMES are sanitized: a graph/group/nonce carrying a path
    /// separator or space must not leak into the bare sentinel name (a raw '/'
    /// would name an intermediate directory the supervisor never creates —
    /// worker ENOENT before READY, surfacing as a misleading "exited before
    /// signaling READY").
    #[test]
    fn sentinel_names_are_sanitized() {
        let r = ready_path("my/graph x", "g 1", "n once");
        assert_eq!(r, "cerulion_my_graph_x_g_1_n_once.ready");
        assert!(
            !r.contains('/') && !r.contains(' '),
            "ready sentinel must carry no '/' or space: {r}"
        );

        let g = go_path("my/graph x", "n once");
        assert_eq!(g, "cerulion_my_graph_x_n_once.go");
        assert!(
            !g.contains('/') && !g.contains(' '),
            "GO sentinel must carry no '/' or space: {g}"
        );
    }

    /// `sanitize_ns` — direct oracle: keeps `[A-Za-z0-9_]`, maps EVERYTHING
    /// else (including `-`) to `_`. The `-` row pins the unified charset: a
    /// second sanitizer that kept `-` let "planner-a"/"planner_a" collide on
    /// sentinels but not plan files.
    #[test]
    fn sanitize_ns_oracle() {
        assert_eq!(sanitize_ns("myGraph_2"), "myGraph_2");
        assert_eq!(sanitize_ns("my-graph"), "my_graph");
        assert_eq!(sanitize_ns("my/graph x!"), "my_graph_x_");
        assert_eq!(sanitize_ns(""), "");
    }

    /// Two distinct validation-passing groups whose names
    /// sanitize to the same token ("g.1"/"g_1") are rejected at plan time —
    /// downstream they key worker plan files and READY sentinels, where a
    /// collision silently overwrites one subgraph with the other (wrong
    /// topology) and cross-trips READY gating.
    #[test]
    fn sanitized_group_collision_rejected_at_plan_time() {
        let config = chain_config(groups(&[
            ("g.1", &["n0", "n1"]),
            ("g_1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let err = plan_deployment(&config, &levels, None, "c").unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("g.1") && msg.contains("g_1") && msg.contains("'g_1'"),
            "collision rejection must name BOTH raw groups and the shared token, got: {msg}"
        );
        assert!(
            msg.contains("rename one process group"),
            "collision rejection must carry the rename remedy, got: {msg}"
        );

        // CONTROL: distinct-token groups ("g.1"/"g.2" -> g_1/g_2) still plan fine.
        let ok_config = chain_config(groups(&[
            ("g.1", &["n0", "n1"]),
            ("g.2", &["n2", "n3", "n4"]),
        ]));
        let ok_levels = chain_levels(&ok_config);
        let plan = plan_deployment(&ok_config, &ok_levels, None, "c").expect("control plan");
        assert_eq!(plan.workers.len(), 2);
        assert_ne!(
            plan.workers[0].ready_path, plan.workers[1].ready_path,
            "distinct tokens must yield distinct READY sentinels"
        );
    }

    /// The `-` collision class: "planner-a"/"planner_a" sanitize to one token
    /// under the UNIFIED charset (`-` maps to `_`) and are rejected. Pins that
    /// no group-keyed filename path keeps `-` while another maps it.
    #[test]
    fn dash_underscore_group_collision_rejected() {
        let config = chain_config(groups(&[
            ("planner-a", &["n0", "n1"]),
            ("planner_a", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let err = plan_deployment(&config, &levels, None, "c").unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("planner-a") && msg.contains("planner_a"),
            "dash/underscore collision must name both raw groups, got: {msg}"
        );
    }

    /// The barrier namespace
    /// lands verbatim in the `/dev/shm` `shm_open` filename and node names
    /// feed iceoryx2 `NodeName` (panic-shaped `expect` in transport init) —
    /// a hostile graph name ("my/graph x") / nonce ("1/2") / group ("g 1")
    /// must therefore mint SANITIZED barrier_ns + node_names (exact oracles),
    /// like the sentinel filenames already do.
    #[test]
    fn hostile_names_mint_sanitized_barrier_ns_and_node_names() {
        let mut config = chain_config(groups(&[
            ("g 1", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        config.identity = "my/graph x".to_string();
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "1/2").expect("plan");

        // Exact-string oracles for the sanitized shapes.
        assert_eq!(plan.barrier_ns, "cerdep_my_graph_x_1_2");
        assert_eq!(plan.workers[0].node_name, "cerulion_my_graph_x_g_1");

        // No '/' or space anywhere a filename / NodeName consumes.
        assert!(
            !plan.barrier_ns.contains('/') && !plan.barrier_ns.contains(' '),
            "barrier_ns must be sanitized: {}",
            plan.barrier_ns
        );
        for w in &plan.workers {
            assert!(
                !w.node_name.contains('/') && !w.node_name.contains(' '),
                "node_name must be sanitized: {}",
                w.node_name
            );
        }
    }

    /// The deployment data plane joins the DEFAULT iceoryx2 namespace —
    /// `mint_deployment_ix_config` returns a snapshot of the host's RESOLVED
    /// global config (`Config::global_config().clone()`, the SAME resolution
    /// `topic echo/hz`, the monolith transport init, and the cleanup sweep
    /// use). Asserted BOTH structurally (== the live `*Config::global_config()`,
    /// so the pin tracks whatever the host resolves) AND against the literal
    /// `"iox2_"` prefix — valid on config-file-free dev/CI machines (Cerulion
    /// ships no iceoryx2 config file), and it fails LOUDLY if iceoryx2's
    /// default prefix ever drifts on a dependency bump (the property
    /// `topic echo/hz`, cross-graph absolute topics, and external publishers
    /// all rely on).
    #[test]
    fn mint_deployment_ix_config_is_the_resolved_default_namespace() {
        let cfg = mint_deployment_ix_config("mygraph", "123_456").expect("mint");
        assert_eq!(
            &cfg,
            iceoryx2::config::Config::global_config(),
            "the mp data plane must be the host's resolved global config (the default namespace)"
        );
        assert_eq!(
            cfg.global.prefix.to_string(),
            "iox2_",
            "the resolved global.prefix must be `iox2_` on a config-file-free host — a drift \
             here silently re-isolates the mp data plane"
        );
    }

    /// `graph` + `nonce` are IGNORED — arbitrary inputs (path
    /// separators, spaces, unicode, empty, 300-char monsters) ALL mint the
    /// IDENTICAL resolved-global Config. This is the PRECONDITION for the
    /// designed concurrent-same-graph collision: two runs of one graph mint the
    /// SAME (default) namespace and collide LOUDLY on the single-writer
    /// publisher checks, rather than silently running side-by-side on disjoint
    /// run-scoped namespaces (an invisible collision).
    #[test]
    fn mint_deployment_ignores_graph_and_nonce_the_collision_precondition() {
        let baseline = mint_deployment_ix_config("baseline", "0").expect("mint");
        let cases: [(String, String); 5] = [
            ("my-graph/x!".into(), "n".into()),
            ("spaces here".into(), "a b".into()),
            ("uníçode".into(), "🚀".into()),
            (String::new(), String::new()),
            ("g".repeat(300), "n".repeat(300)),
        ];
        for (graph, nonce) in &cases {
            let cfg = mint_deployment_ix_config(graph, nonce).expect("mint");
            assert_eq!(
                cfg, baseline,
                "graph={graph:?} nonce={nonce:?} must mint the SAME resolved Config (inputs ignored)"
            );
        }
    }

    /// `mint_deployment_ix_config`: same inputs mint EQUAL configs — the
    /// cross-process convergence contract (every worker computes the same
    /// namespace independently). Now this is the per-process
    /// `global_config()` static's determinism (resolved once, then cached),
    /// but the contract the workers rely on is unchanged.
    #[test]
    fn mint_config_is_deterministic() {
        let a = mint_deployment_ix_config("g", "n").expect("mint");
        let b = mint_deployment_ix_config("g", "n").expect("mint");
        assert_eq!(a, b, "same (graph, nonce) must mint an identical Config");
    }

    /// `mint_deployment_ix_config`: the minted Config survives a serde JSON
    /// round-trip AND serializes to a NON-EMPTY string. The non-empty pin is
    /// load-bearing: the worker rejects an EMPTY `ix_config_json` loudly
    /// (`graph_run_worker`), and the supervisor injects THIS JSON — so a
    /// resolved Config that serialized empty would silently trip the worker
    /// guard. This pins that the snapshot clears the `.is_empty()` guard.
    #[test]
    fn mint_config_serde_round_trips_and_is_non_empty() {
        let cfg = mint_deployment_ix_config("g", "n").expect("mint");
        let json = serde_json::to_string(&cfg).expect("serialize");
        assert!(
            !json.is_empty(),
            "the deployment Config JSON must be non-empty (the worker empty-guard depends on it)"
        );
        let back: iceoryx2::config::Config = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(
            cfg, back,
            "iceoryx2 Config JSON round-trip must preserve the minted Config"
        );
    }

    /// The deployment data plane is the DEFAULT `iox2_` namespace while
    /// the supervisor's PLANNING namespace stays run-scoped (`cer_p_{hex}`, still
    /// FNV-1a over `{graph}\x00{nonce}`). The two must be DISTINCT and — the
    /// observed iceoryx2 `Node::list` panic class — NEITHER may string-prefix
    /// the other. The guarantee is CHAR-0 DIVERGENCE (`iox2_` vs
    /// `cer_p_…`: `i` != `c`), not an equal-length trick, so both
    /// `starts_with` directions are asserted explicitly (there is no equal-length
    /// assertion — the strings are not equal length).
    ///
    /// SCOPE: the deploy prefix is now the host's RESOLVED global config, so this
    /// invariant is enforced against the config-file-free resolution (`iox2_` —
    /// every dev/CI machine; Cerulion ships no iceoryx2 config file). An operator
    /// config file overriding `global.prefix` to something `cer_p`-adjacent sits
    /// outside the structural guarantee; that collision would surface via
    /// iceoryx2's own errors at planning-build time, not silently.
    #[test]
    fn deployment_default_ns_and_planning_ns_neither_prefixes_the_other() {
        let deploy = mint_deployment_ix_config("g", "n")
            .expect("mint")
            .global
            .prefix
            .to_string();
        let planning = planning_ix_config("g", "n")
            .expect("mint")
            .global
            .prefix
            .to_string();
        assert_eq!(deploy, "iox2_", "deployment data plane = default namespace");
        // Planning is still FNV-scoped per (graph, nonce) — structural + literal.
        assert_eq!(
            planning,
            format!("cer_p_{:016x}", fnv1a64(b"g\x00n")),
            "planning prefix shape (still run-scoped)"
        );
        assert_eq!(
            planning, "cer_p_d4806b18fa57bd34",
            "planning literal pin for (\"g\", \"n\")"
        );
        assert_ne!(deploy, planning, "namespaces must be distinct");
        assert!(
            !deploy.starts_with(&planning) && !planning.starts_with(&deploy),
            "neither prefix may string-prefix the other (iceoryx2 Node::list panic class); \
             `iox2_` and `cer_p_…` diverge at char 0"
        );
    }

    /// `plan_deployment` floors/passes-through the handed global quantum, and
    /// stamps it IDENTICALLY on every worker.
    ///
    /// NOTE ON SCOPE: this module contains NO min-over-groups logic — deriving
    /// the global tightest is the caller's impure job (`Scheduler::tightest_timing_ns`
    /// over the full graph; see the module docs). This test MODELS
    /// that derivation in the test body (an explicit `min` over an oracle vector)
    /// purely to feed a realistic scalar; the assertions exercise only
    /// `plan_deployment`'s flooring + uniform stamping, not a min derivation that
    /// lives here.
    #[test]
    fn global_quantum_is_min_over_group_tightests() {
        let config = chain_config(groups(&[
            ("A", &["n0"]),
            ("B", &["n1", "n2"]),
            ("C", &["n3", "n4"]),
        ]));
        let levels = chain_levels(&config);

        // Per-group tightests: 4ms, 1ms, none. Global min = 1ms.
        let per_group: [Option<u64>; 3] = [Some(4_000_000), Some(1_000_000), None];
        let global_min = per_group.into_iter().flatten().min();
        assert_eq!(global_min, Some(1_000_000));

        let plan = plan_deployment(&config, &levels, global_min, "q").expect("plan");
        for w in &plan.workers {
            assert_eq!(w.handed_quantum_ns, 1_000_000);
        }

        // All-`None` (no declared timing) -> 1ms floor.
        let plan_none = plan_deployment(&config, &levels, None, "q").expect("plan");
        for w in &plan_none.workers {
            assert_eq!(w.handed_quantum_ns, HANDED_QUANTUM_FLOOR_NS);
        }

        // A larger min (5ms) flows through unfloored.
        let plan_5 = plan_deployment(&config, &levels, Some(5_000_000), "q").expect("plan");
        for w in &plan_5.workers {
            assert_eq!(w.handed_quantum_ns, 5_000_000);
        }
    }

    /// The `handed_quantum_ns` flooring helper — direct oracle vector.
    #[test]
    fn handed_quantum_helper_floors_correctly() {
        assert_eq!(handed_quantum_ns(None), 1_000_000, "no timing -> 1ms floor");
        assert_eq!(handed_quantum_ns(Some(0)), 1_000_000, "0 clamped to floor");
        assert_eq!(
            handed_quantum_ns(Some(500_000)),
            1_000_000,
            "sub-ms clamped to floor"
        );
        assert_eq!(
            handed_quantum_ns(Some(1_000_000)),
            1_000_000,
            "exactly 1ms passes"
        );
        assert_eq!(
            handed_quantum_ns(Some(16_000_000)),
            16_000_000,
            "16ms passes through"
        );
    }

    /// An INTERLEAVED partition (a group owning non-adjacent global levels) is
    /// rejected at PLAN time with the contiguous-split diagnostic.
    #[test]
    fn interleaved_partition_rejected_at_plan_time() {
        // Diamond globals: n0=0, n1=1, n2=1, n3=2.
        // P0:[n0,n3] owns {0,2} (GAP at level 1) ; P1:[n1,n2] owns {1}.
        let config = diamond_config(groups(&[("P0", &["n0", "n3"]), ("P1", &["n1", "n2"])]));
        let levels = diamond_levels(&config);

        let err = plan_deployment(&config, &levels, None, "bad").unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("non-adjacent global DAG levels") && msg.contains("P0"),
            "expected contiguous-split rejection naming P0, got: {msg}"
        );
    }

    /// The contiguity helper — direct oracle vectors over hand-built maps.
    #[test]
    fn contiguity_helper_oracle() {
        let ok = |map: Vec<Option<usize>>| ProcessGroup {
            name: "g".to_string(),
            rank: 0,
            global_level_map: map,
        };
        // Contiguous span (trailing None) -> Ok.
        assert!(validate_contiguous_ownership(&ok(vec![Some(0), Some(1), None])).is_ok());
        // Leading None, contiguous -> Ok.
        assert!(validate_contiguous_ownership(&ok(vec![None, None, Some(0), Some(1)])).is_ok());
        // Single owned level, sandwiched -> Ok (no gap between owned levels).
        assert!(validate_contiguous_ownership(&ok(vec![None, Some(0), None])).is_ok());
        // Vacuous: a map with NO owned level (all None) -> Ok (nothing to gap).
        assert!(validate_contiguous_ownership(&ok(vec![None, None])).is_ok());
        // Vacuous: an empty map -> Ok (the `first`/`last == None` early skip).
        assert!(validate_contiguous_ownership(&ok(vec![])).is_ok());
        // GAP between owned levels -> Err.
        assert!(validate_contiguous_ownership(&ok(vec![Some(0), None, Some(1)])).is_err());
        // Two gaps -> Err.
        assert!(
            validate_contiguous_ownership(&ok(vec![Some(0), None, Some(1), None, Some(2)]))
                .is_err()
        );
    }

    /// A graph without `process_groups:` is rejected loudly.
    #[test]
    fn empty_process_groups_rejected() {
        let config = chain_config(IndexMap::new());
        let levels = chain_levels(&config);
        let err = plan_deployment(&config, &levels, None, "n").unwrap_err();
        assert!(
            err.to_string().contains("declares no `process_groups:`"),
            "got: {err}"
        );
    }

    /// An ORPHAN node (declared but in no group) is rejected via
    /// `derive_process_groups` -> `validate_process_groups`.
    #[test]
    fn orphan_node_rejected_via_derive() {
        // n4 is left out of every group.
        let config = chain_config(groups(&[("P0", &["n0", "n1"]), ("P1", &["n2", "n3"])]));
        let levels = chain_levels(&config);
        let err = plan_deployment(&config, &levels, None, "n").unwrap_err();
        assert!(
            err.to_string().contains("n4") && err.to_string().contains("process group"),
            "expected orphan rejection naming n4, got: {err}"
        );
    }

    /// A node assigned to TWO groups is rejected via `derive_process_groups`.
    #[test]
    fn double_assigned_node_rejected_via_derive() {
        // n2 is in both P0 and P1.
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1", "n2"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let err = plan_deployment(&config, &levels, None, "n").unwrap_err();
        assert!(
            err.to_string().contains("more than one process group"),
            "got: {err}"
        );
    }

    /// A config/levels MISMATCH (levels smaller than the config) surfaces the
    /// `derive_process_groups` "no level" error rather than panicking.
    #[test]
    fn config_levels_mismatch_rejected() {
        let full = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        // Levels built from a SMALLER graph (only n0,n1) — n2..n4 have no level.
        let small = chain_config(groups(&[("P0", &["n0", "n1"])]));
        let small = GraphConfig {
            level_assignments: None,
            network: None,
            nodes: small.nodes.into_iter().take(2).collect(),
            ..small
        };
        let small_levels = levels_for(&small, &[("n1", "/p/n0/out")]);

        let err = plan_deployment(&full, &small_levels, None, "n").unwrap_err();
        assert!(
            err.to_string().contains("no level"),
            "expected level-mismatch error, got: {err}"
        );
    }

    /// An OVERSIZED / stale `levels` (MORE global levels than the config
    /// occupies) is rejected at plan time — guards the direction
    /// `derive_process_groups` does NOT (it only rejects the undersized
    /// direction, above). Without the guard the plan would carry phantom empty
    /// global levels or silently-shifted ownership.
    #[test]
    fn oversized_levels_rejected() {
        // Planned config: the standard 5-node chain (n0..n4 -> 5 global levels).
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        // `levels` built from a 6-node SUPERSET chain (n0..n5 -> 6 levels): every
        // planned node still resolves a level (0..4), so the undersized check
        // passes, but `levels.len() == 6 > 5` occupied.
        let superset = GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "chain".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                node("n0", &[], &["out"]),
                node("n1", &[("inp", "n0/out")], &["out"]),
                node("n2", &[("inp", "n1/out")], &["out"]),
                node("n3", &[("inp", "n2/out")], &["out"]),
                node("n4", &[("inp", "n3/out")], &["out"]),
                node("n5", &[("inp", "n4/out")], &[]),
            ],
            multi_publisher_topics: Vec::new(),
            process_groups: IndexMap::new(),
            process_group_order: Vec::new(),
        };
        let big_levels = levels_for(
            &superset,
            &[
                ("n1", "/p/n0/out"),
                ("n2", "/p/n1/out"),
                ("n3", "/p/n2/out"),
                ("n4", "/p/n3/out"),
                ("n5", "/p/n4/out"),
            ],
        );
        assert_eq!(big_levels.len(), 6);

        let err = plan_deployment(&config, &big_levels, None, "n").unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("occupies 5 global DAG levels") && msg.contains("reports 6"),
            "expected oversized-levels rejection, got: {msg}"
        );
    }

    /// WorkerPlan serde round-trips (serialize -> deserialize -> serialize is
    /// byte-stable). GraphConfig is not PartialEq, so we compare serializations.
    #[test]
    fn worker_plan_serde_round_trips() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let mut plan = plan_deployment(&config, &levels, Some(2_000_000), "rt").expect("plan");

        // `plan_deployment` leaves `ix_config_json` empty (the pure planner does
        // not mint an iceoryx2 Config). Simulate the supervisor injecting
        // a minted-Config JSON so the round-trip exercises a NON-empty value.
        let injected = r#"{"root_path":"/tmp/rt","some":"config"}"#.to_string();
        plan.workers[0].ix_config_json = injected.clone();

        let w = &plan.workers[0];
        let s1 = serde_json::to_string(w).expect("serialize");
        let w2: WorkerPlan = serde_json::from_str(&s1).expect("deserialize");
        let s2 = serde_json::to_string(&w2).expect("re-serialize");
        assert_eq!(s1, s2, "WorkerPlan serde round-trip must be byte-stable");
        // Spot-check a few reconstructed fields.
        assert_eq!(w2.group, "P0");
        assert_eq!(
            w2.global_level_map,
            vec![Some(0), Some(1), None, None, None]
        );
        assert_eq!(w2.handed_quantum_ns, 2_000_000);
        assert_eq!(w2.subgraph.nodes.len(), 2);
        // The injected `ix_config_json` survives the round-trip verbatim.
        assert_eq!(
            w2.ix_config_json, injected,
            "the supervisor-injected iceoryx2 Config JSON must round-trip verbatim"
        );
    }

    /// BACK-COMPAT: an OLD plan-file JSON written BEFORE the
    /// `trace_limit` field existed (the key absent entirely) must still
    /// deserialize, with the field defaulting to the production trace cap
    /// (`PRODUCTION_TRACE_LIMIT` = 100_000) via `#[serde(default)]`.
    #[test]
    fn worker_plan_without_trace_limit_key_defaults_to_production_cap() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "bc").expect("plan");

        // Serialize a modern plan, then STRIP the `trace_limit` key to fabricate
        // the exact on-disk shape an old supervisor wrote.
        let mut v = serde_json::to_value(&plan.workers[0]).expect("to_value");
        let obj = v
            .as_object_mut()
            .expect("WorkerPlan serializes to an object");
        assert!(
            obj.remove("trace_limit").is_some(),
            "modern WorkerPlan JSON must carry trace_limit (sanity)"
        );
        let old_json = serde_json::to_string(&v).expect("old-shape JSON");

        let w: WorkerPlan = serde_json::from_str(&old_json)
            .expect("an old plan file without trace_limit must still deserialize");
        assert_eq!(
            w.trace_limit,
            crate::graph_cmd::PRODUCTION_TRACE_LIMIT,
            "absent trace_limit key must default to the production cap (100_000)"
        );
        // Sanity: the planner's own default matches the same single source.
        assert_eq!(
            plan.workers[0].trace_limit,
            crate::graph_cmd::PRODUCTION_TRACE_LIMIT
        );
    }

    /// BACK-COMPAT: an OLD plan-file JSON written BEFORE the
    /// `monitor_wait` / `doorbell` fields existed (keys absent entirely) must
    /// still deserialize, with BOTH defaulting to `false` — the
    /// park-off behavior — via `#[serde(default)]`.
    #[test]
    fn worker_plan_without_park_keys_defaults_to_off() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "mwbc").expect("plan");

        // Serialize a modern plan, then STRIP both park keys to fabricate the
        // exact on-disk shape an old supervisor wrote.
        let mut v = serde_json::to_value(&plan.workers[0]).expect("to_value");
        let obj = v
            .as_object_mut()
            .expect("WorkerPlan serializes to an object");
        assert!(
            obj.remove("monitor_wait").is_some() && obj.remove("doorbell").is_some(),
            "modern WorkerPlan JSON must carry both park keys (sanity)"
        );
        let old_json = serde_json::to_string(&v).expect("old-shape JSON");

        let w: WorkerPlan = serde_json::from_str(&old_json)
            .expect("an old plan file without the park keys must still deserialize");
        assert!(
            !w.monitor_wait && !w.doorbell,
            "absent park keys must default to false (park off)"
        );
    }

    /// BACK-COMPAT: an OLD plan-file JSON written BEFORE the
    /// `cap_disabled` field existed (key absent entirely) must still
    /// deserialize, with the field defaulting to `false` — the worker's `Auto`
    /// cap mode — via `#[serde(default)]`. Also pins the
    /// pure planner's default (`false`; the supervisor's `stamp_cap_mode` is
    /// the only writer of `true`).
    #[test]
    fn worker_plan_without_cap_disabled_key_defaults_to_auto() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "capbc").expect("plan");

        // Planner-default pin: the pure planner leaves the opt-out unstamped.
        for w in &plan.workers {
            assert!(
                !w.cap_disabled,
                "the pure planner must default cap_disabled to false (Auto)"
            );
        }

        // Serialize a modern plan, then STRIP the key to fabricate the exact
        // on-disk shape an old supervisor wrote.
        let mut v = serde_json::to_value(&plan.workers[0]).expect("to_value");
        let obj = v
            .as_object_mut()
            .expect("WorkerPlan serializes to an object");
        assert!(
            obj.remove("cap_disabled").is_some(),
            "modern WorkerPlan JSON must carry cap_disabled (sanity)"
        );
        let old_json = serde_json::to_string(&v).expect("old-shape JSON");

        let w: WorkerPlan = serde_json::from_str(&old_json)
            .expect("an old plan file without cap_disabled must still deserialize");
        assert!(
            !w.cap_disabled,
            "absent cap_disabled key must default to false (the worker's Auto cap mode)"
        );
    }

    /// The `recording_ring` seam field.
    ///
    /// Three pins in one shape (mirrors the trace_limit/park/cap back-compat
    /// tests above):
    /// 1. the PURE planner leaves it `None` (not recording — the supervisor is
    ///    the only writer);
    /// 2. a supervisor-stamped `Some(tag)` survives the plan-file JSON
    ///    round-trip VERBATIM (the worker reads exactly the tag the supervisor
    ///    will hand to bagd);
    /// 3. an OLD plan-file JSON written BEFORE the field existed (key absent
    ///    entirely) still deserializes, defaulting to `None` — the
    ///    not-recording shape — via `#[serde(default)]`.
    #[test]
    fn worker_plan_recording_ring_roundtrips_and_absent_key_defaults_to_none() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let mut plan = plan_deployment(&config, &levels, None, "rr").expect("plan");

        // (1) planner-default pin: nothing sets the field yet.
        for w in &plan.workers {
            assert_eq!(
                w.recording_ring, None,
                "the pure planner must leave recording_ring None (not recording)"
            );
        }

        // (2) a stamped Some(tag) round-trips verbatim.
        let tag = "cer_rec_demo_12345_r0".to_string();
        plan.workers[0].recording_ring = Some(tag.clone());
        let json = serde_json::to_string(&plan.workers[0]).expect("serialize stamped plan");
        let w: WorkerPlan = serde_json::from_str(&json).expect("deserialize stamped plan");
        assert_eq!(
            w.recording_ring.as_deref(),
            Some(tag.as_str()),
            "a supervisor-stamped recording_ring must round-trip verbatim"
        );

        // (3) old-plan back-compat: STRIP the key to fabricate the exact
        // on-disk shape an old supervisor wrote.
        let mut v = serde_json::to_value(&plan.workers[0]).expect("to_value");
        let obj = v
            .as_object_mut()
            .expect("WorkerPlan serializes to an object");
        assert!(
            obj.remove("recording_ring").is_some(),
            "modern WorkerPlan JSON must carry recording_ring (sanity)"
        );
        let old_json = serde_json::to_string(&v).expect("old-shape JSON");
        let w: WorkerPlan = serde_json::from_str(&old_json)
            .expect("an old plan file without recording_ring must still deserialize");
        assert_eq!(
            w.recording_ring, None,
            "absent recording_ring key must default to None (not recording)"
        );
    }

    /// `plan_deployment` leaves every worker's `ix_config_json` empty (the pure
    /// planner does not mint an iceoryx2 Config; the supervisor does).
    #[test]
    fn plan_leaves_ix_config_json_empty() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "e").expect("plan");
        for w in &plan.workers {
            assert!(
                w.ix_config_json.is_empty(),
                "the pure planner must leave ix_config_json empty for the supervisor to inject"
            );
        }
    }

    /// DeploymentPlan serde round-trips.
    #[test]
    fn deployment_plan_serde_round_trips() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "rt").expect("plan");

        let s1 = serde_json::to_string(&plan).expect("serialize");
        let plan2: DeploymentPlan = serde_json::from_str(&s1).expect("deserialize");
        let s2 = serde_json::to_string(&plan2).expect("re-serialize");
        assert_eq!(
            s1, s2,
            "DeploymentPlan serde round-trip must be byte-stable"
        );
        assert_eq!(plan2.workers.len(), 2);
        assert_eq!(plan2.expected, 2);
        assert_eq!(plan2.barrier_ns, "cerdep_chain_rt");
    }

    /// The planner is deterministic — same inputs -> byte-identical plan.
    #[test]
    fn plan_is_deterministic() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let a = plan_deployment(&config, &levels, Some(3_000_000), "det").expect("plan");
        let b = plan_deployment(&config, &levels, Some(3_000_000), "det").expect("plan");
        assert_eq!(
            serde_json::to_string(&a).unwrap(),
            serde_json::to_string(&b).unwrap(),
            "planner must be deterministic"
        );
    }

    /// The PURE planner leaves `topic_requirements` empty — harvesting
    /// needs the impure full-graph build, so the supervisor stamps it post-plan
    /// (`graph_cmd::stamp_topic_requirements`), exactly like `ix_config_json` /
    /// `prefer_release`.
    #[test]
    fn plan_deployment_leaves_topic_requirements_empty() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "tr").expect("plan");
        for w in &plan.workers {
            assert!(
                w.topic_requirements.is_empty(),
                "the pure planner must leave topic_requirements empty for the supervisor to stamp"
            );
        }
    }

    /// BACK-COMPAT: an OLD plan-file JSON written BEFORE the
    /// `topic_requirements` field existed (key absent entirely) must still
    /// deserialize, with the field defaulting to an EMPTY map via
    /// `#[serde(default)]` — the current single-process-view behavior (a no-op
    /// union). Mirrors the `cap_disabled` / park-key back-compat pins.
    #[test]
    fn worker_plan_without_topic_requirements_key_defaults_to_empty() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let plan = plan_deployment(&config, &levels, None, "trbc").expect("plan");

        // Serialize a modern plan, then STRIP the key to fabricate the exact
        // on-disk shape an old supervisor wrote.
        let mut v = serde_json::to_value(&plan.workers[0]).expect("to_value");
        let obj = v
            .as_object_mut()
            .expect("WorkerPlan serializes to an object");
        assert!(
            obj.remove("topic_requirements").is_some(),
            "modern WorkerPlan JSON must carry topic_requirements (sanity)"
        );
        let old_json = serde_json::to_string(&v).expect("old-shape JSON");

        let w: WorkerPlan = serde_json::from_str(&old_json)
            .expect("an old plan file without topic_requirements must still deserialize");
        assert!(
            w.topic_requirements.is_empty(),
            "absent topic_requirements key must default to an empty map (no-op union)"
        );
    }

    /// A STAMPED `topic_requirements` map round-trips through serde
    /// byte-stably (the supervisor stamps it, the worker deserializes it back)
    /// and reconstructs each per-topic requirement verbatim.
    #[test]
    fn worker_plan_with_topic_requirements_round_trips() {
        let config = chain_config(groups(&[
            ("P0", &["n0", "n1"]),
            ("P1", &["n2", "n3", "n4"]),
        ]));
        let levels = chain_levels(&config);
        let mut plan = plan_deployment(&config, &levels, None, "trrt").expect("plan");

        // Simulate the supervisor stamping a harvested requirement (a cross-group
        // snapshot topic needing borrow-3) into worker 0.
        let mut stamped: std::collections::BTreeMap<String, cerulion_core::TopicRequirements> =
            std::collections::BTreeMap::new();
        stamped.insert(
            "/p/n0/out".to_string(),
            cerulion_core::TopicRequirements {
                min_borrowed_samples: 3,
                min_buffer: 16,
                min_subscribers: 6,
                min_event_listeners: 1,
            },
        );
        plan.workers[0].topic_requirements = stamped.clone();

        let w = &plan.workers[0];
        let s1 = serde_json::to_string(w).expect("serialize");
        let w2: WorkerPlan = serde_json::from_str(&s1).expect("deserialize");
        let s2 = serde_json::to_string(&w2).expect("re-serialize");
        assert_eq!(s1, s2, "WorkerPlan serde round-trip must be byte-stable");
        // The stamped map reconstructs verbatim.
        assert_eq!(w2.topic_requirements, stamped);
        let req = &w2.topic_requirements["/p/n0/out"];
        assert_eq!(req.min_borrowed_samples, 3);
        assert_eq!(req.min_buffer, 16);
        assert_eq!(req.min_subscribers, 6);
        assert_eq!(req.min_event_listeners, 1);
    }

    // ======================================================================
    // Worker sub-configs carry the COMPRESSED local band.
    // ======================================================================

    /// A parent `level_assignments` override is restricted to each group's
    /// members and rank-compressed to the group's 0-based local band; a
    /// parent WITHOUT the block keeps `None` (the worker Kahn-levelizes,
    /// byte-identical to the earlier behavior).
    #[test]
    fn subgraph_for_compresses_parent_level_assignments_per_group() {
        // Chain n0..n4 with a REFINED assignment: n1 delayed one level
        // (n0:0, n1:2, n2:1 is illegal — keep edges increasing). Use the
        // identity-shifted shape: n0:0 n1:1 n2:2 n3:3 n4:4 (Kahn) is a no-op
        // for compression interest, so instead give g1 a GAPPED-owner shape
        // via grouping: g0=[n0,n1], g1=[n2,n4], g2=[n3] — g1 owns global
        // {2,4}, which must compress to local {0,1}.
        let mut config = chain_config(groups(&[
            ("g0", &["n0", "n1"]),
            ("g1", &["n2", "n4"]),
            ("g2", &["n3"]),
        ]));
        let mut assignments = IndexMap::new();
        for (node, level) in [("n0", 0usize), ("n1", 1), ("n2", 2), ("n3", 3), ("n4", 4)] {
            assignments.insert(node.to_string(), level);
        }
        config.level_assignments = Some(assignments);

        let g0 = subgraph_for(&config, "g0", &["n0".into(), "n1".into()]);
        let g0_map = g0
            .level_assignments
            .expect("g0 carries the compressed band");
        assert_eq!(
            g0_map
                .iter()
                .map(|(k, v)| (k.as_str(), *v))
                .collect::<Vec<_>>(),
            vec![("n0", 0), ("n1", 1)],
            "g0: contiguous global {{0,1}} compresses to the identity"
        );

        let g1 = subgraph_for(&config, "g1", &["n2".into(), "n4".into()]);
        let g1_map = g1
            .level_assignments
            .expect("g1 carries the compressed band");
        assert_eq!(
            g1_map
                .iter()
                .map(|(k, v)| (k.as_str(), *v))
                .collect::<Vec<_>>(),
            vec![("n2", 0), ("n4", 1)],
            "g1: GAPPED global {{2,4}} rank-compresses to local {{0,1}}"
        );

        let g2 = subgraph_for(&config, "g2", &["n3".into()]);
        let g2_map = g2
            .level_assignments
            .expect("g2 carries the compressed band");
        assert_eq!(
            g2_map
                .iter()
                .map(|(k, v)| (k.as_str(), *v))
                .collect::<Vec<_>>(),
            vec![("n3", 0)],
            "a singleton group compresses to local 0"
        );

        // Parent None => worker None (byte-identical to the earlier behavior).
        let plain = chain_config(groups(&[("g0", &["n0", "n1"])]));
        let sub = subgraph_for(&plain, "g0", &["n0".into(), "n1".into()]);
        assert!(sub.level_assignments.is_none(), "no override => no block");
    }

    // ===================================================================
    // The plan-time split-same-level-non-trigger-pair detector.
    //
    // Every arm is an ORACLE-VECTOR test — the expected `Vec<
    // SplitNonTriggerPair>` is written out by hand from the fixture, never
    // compared against a second run of the same function.
    //
    // The fixtures below are a minimal synthetic equivalent of the
    // `obstacle_avoidance` shape (two `period_ms` nodes joined by ONE plain
    // `#[input]`, split process-per-node). `build_trigger_edges` emits NO
    // trigger edges for a `period_ms` node, which is exactly what an EMPTY
    // `TriggerEdges` models here — so `levels_for(cfg, &[])` puts both nodes
    // at global level 0 the same way the real pipeline does.
    //
    // Every node in these fixtures carries a REAL `MacroPolicy` (default
    // `Period`, see `infos_for`), because the reported workaround (a) is
    // derived from the CONSUMER's policy: an empty `NodeInfo` would silently
    // exercise the unreachable `NoDeclaredPolicy` arm instead of the shape the
    // detector was written for.
    // ===================================================================

    /// The canonical unordered shape: `producer` publishes `out`, `consumer`
    /// reads it through a PLAIN (non-trigger) input `plan`. Both are level-0
    /// roots.
    fn pair_config(process_groups: IndexMap<String, Vec<String>>) -> GraphConfig {
        GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "obstacle".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                node("producer", &[], &["out"]),
                node("consumer", &[("plan", "producer/out")], &[]),
            ],
            multi_publisher_topics: Vec::new(),
            process_groups,
            process_group_order: Vec::new(),
        }
    }

    const PAIR_TOPIC: &str = "/p/producer/out";

    /// A node whose single output publishes to an ABSOLUTE topic override
    /// (used for the `multi_publisher_topics` arm, where two nodes must
    /// publish the SAME topic).
    fn abs_node(id: &str, inputs: &[(&str, &str)], out_topic: Option<&str>) -> NodeDef {
        NodeDef {
            ros2: None,
            id: id.to_string(),
            node_type: id.to_string(),
            inputs: inputs
                .iter()
                .map(|(name, source)| InputDef {
                    name: name.to_string(),
                    source: source.to_string(),
                })
                .collect(),
            outputs: out_topic
                .into_iter()
                .map(|t| OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: Some(t.to_string()),
                })
                .collect(),
        }
    }

    /// A per-node policy + trigger-mark override for [`infos_for`]:
    /// `(node_id, policy, trigger-marked input names)`.
    type MetaSpec<'a> = (&'a str, Option<cerulion_core::MacroPolicy>, &'a [&'a str]);

    /// Build per-node `NodeInfo` the way BOTH provenances do — a macro policy
    /// plus one `InputMeta` per YAML-wired input carrying its declared
    /// `#[input(trigger)]` mark. Nodes not named in `spec` default to
    /// `Period { period_ms: 10 }` with no trigger marks, i.e. the demo's
    /// `obstacle_avoidance` shape.
    ///
    /// This is the shape `source_entry_infos` builds and the shape a cdylib's
    /// `info()` returns, so `build_trigger_edges` over the result is the REAL
    /// classification — which is what makes
    /// `classify_split_same_level_non_trigger_pairs` testable end-to-end.
    fn infos_for(config: &GraphConfig, spec: &[MetaSpec<'_>]) -> IndexMap<String, NodeInfo> {
        config
            .nodes
            .iter()
            .map(|n| {
                let over = spec.iter().find(|(id, _, _)| *id == n.id.as_str());
                let policy = match over {
                    Some((_, policy, _)) => policy.clone(),
                    None => Some(cerulion_core::MacroPolicy::Period { period_ms: 10 }),
                };
                let triggers: &[&str] = over.map(|(_, _, t)| *t).unwrap_or(&[]);
                let input_meta: Vec<cerulion_core::graph::node::InputMeta> = n
                    .inputs
                    .iter()
                    .map(|i| cerulion_core::graph::node::InputMeta {
                        name: i.name.clone(),
                        schema_hash: 0,
                        trigger: triggers.contains(&i.name.as_str()),
                        depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
                        backpressure: cerulion_core::graph::node::BackpressurePolicy::default(),
                        expect_within_ms: None,
                    })
                    .collect();
                let mut info = NodeInfo::with_meta(input_meta, Vec::new());
                if let Some(policy) = policy {
                    info = info.with_policy(policy);
                }
                (n.id.clone(), info)
            })
            .collect()
    }

    /// A hand-written expected finding. `remedy` defaults to
    /// [`TriggerRemedy::ReplacePeriodWithTrigger`] because [`detect`]'s
    /// fixtures are all-`Period`; the policy-varied arms use [`pair_r`].
    fn pair(
        producer: &str,
        producer_group: &str,
        consumer: &str,
        consumer_input: &str,
        consumer_group: &str,
        topic: &str,
        level: usize,
    ) -> SplitNonTriggerPair {
        pair_r(
            producer,
            producer_group,
            consumer,
            consumer_input,
            consumer_group,
            topic,
            level,
            TriggerRemedy::ReplacePeriodWithTrigger,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn pair_r(
        producer: &str,
        producer_group: &str,
        consumer: &str,
        consumer_input: &str,
        consumer_group: &str,
        topic: &str,
        level: usize,
        remedy: TriggerRemedy,
    ) -> SplitNonTriggerPair {
        SplitNonTriggerPair {
            topic: topic.to_string(),
            producer: producer.to_string(),
            producer_group: producer_group.to_string(),
            consumer: consumer.to_string(),
            consumer_input: consumer_input.to_string(),
            consumer_group: consumer_group.to_string(),
            level,
            remedy,
        }
    }

    /// Run the detector over `config` with the given triggering edges, all
    /// nodes `Period`.
    fn detect(config: &GraphConfig, trigger: &[(&str, &str)]) -> Vec<SplitNonTriggerPair> {
        detect_with_meta(config, trigger, &[])
    }

    /// [`detect`] with per-node policy overrides.
    fn detect_with_meta(
        config: &GraphConfig,
        trigger: &[(&str, &str)],
        spec: &[MetaSpec<'_>],
    ) -> Vec<SplitNonTriggerPair> {
        let entry_infos = infos_for(config, spec);
        let topology = GraphTopology::build(config, &entry_infos).expect("topology build");
        let mut edges = TriggerEdges::new();
        for (consumer, topic) in trigger {
            edges.insert(*consumer, *topic);
        }
        let levels = topology.derive_levels(&edges).expect("derive levels");
        split_same_level_non_trigger_pairs(
            &topology,
            &edges,
            &levels,
            &config.process_groups,
            LoadedNodeMetadata(&entry_infos),
        )
    }

    /// HEADLINE (positive): process-per-node on the `obstacle_avoidance`
    /// shape reports exactly the one split pair, every field hand-oracled.
    #[test]
    fn a_split_same_level_non_trigger_pair_is_reported() {
        let config = pair_config(groups(&[("g0", &["producer"]), ("g1", &["consumer"])]));
        assert_eq!(
            detect(&config, &[]),
            vec![pair(
                "producer", "g0", "consumer", "plan", "g1", PAIR_TOPIC, 0
            )],
            "two level-0 nodes joined by a plain input, in different groups, is \
             exactly the unordered shape this warn exists to name"
        );
    }

    /// GUARD 3 (same group): co-locating the pair keeps the monolith's
    /// snapshot-before-tick ordering, so there is nothing to warn about.
    /// Dropping the `producer_group == consumer_group` check fails here.
    #[test]
    fn a_co_located_pair_is_not_reported() {
        let config = pair_config(groups(&[("g0", &["producer", "consumer"])]));
        assert_eq!(
            detect(&config, &[]),
            Vec::new(),
            "one group owns both nodes — `run_level` orders them in-process"
        );
    }

    /// The "SOME ordering guard survives" oracle: make the SAME edge
    /// triggering and the consumer levelizes to 1, so the end-of-level barrier
    /// orders the pair even though the groups differ.
    ///
    /// This arm is killed by NEITHER
    /// single-guard variant — with `is_triggering` deleted the level check still
    /// excludes it (levels 0 vs 1), and with the level check deleted
    /// `is_triggering` still does. Only dropping BOTH fails it (measured:
    /// 4 arms fail, this one among them). It is kept for exactly that reason —
    /// it is the arm that notices the detector losing its whole ordering story
    /// at once, which the two single-guard arms cannot see.
    #[test]
    fn a_pair_at_different_levels_is_not_reported() {
        let config = pair_config(groups(&[("g0", &["producer"]), ("g1", &["consumer"])]));
        let trigger = [("consumer", PAIR_TOPIC)];
        // Anti-vacuity: the fixture really did move the consumer down a level
        // (otherwise this would pass for the wrong reason).
        let entry_infos: IndexMap<String, NodeInfo> = config
            .nodes
            .iter()
            .map(|n| (n.id.clone(), NodeInfo::with_meta(Vec::new(), Vec::new())))
            .collect();
        let topology = GraphTopology::build(&config, &entry_infos).expect("topology");
        let mut edges = TriggerEdges::new();
        edges.insert("consumer", PAIR_TOPIC);
        let levels = topology.derive_levels(&edges).expect("levels");
        assert_eq!(levels.level_of("producer"), Some(0));
        assert_eq!(levels.level_of("consumer"), Some(1));

        assert_eq!(
            detect(&config, &trigger),
            Vec::new(),
            "a trigger edge IS a DAG edge — the barrier between levels 0 and 1 \
             orders the publish against the snapshot"
        );
    }

    /// GUARD 2, the half a trigger edge cannot reach: a NON-trigger edge whose
    /// consumer is pushed to another level by some OTHER trigger edge. The
    /// pair is barrier-ordered and must not be reported, and only the LEVEL
    /// check can see it (guard 1 declines to fire — the edge really is
    /// non-triggering).
    #[test]
    fn a_non_trigger_edge_spanning_two_levels_is_not_reported() {
        // `src` -TRIGGERS-> `consumer` (level 1); `producer` (level 0) also
        // feeds `consumer` through a PLAIN input.
        let config = GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "spanning".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                node("src", &[], &["out"]),
                node("producer", &[], &["out"]),
                node(
                    "consumer",
                    &[("trig", "src/out"), ("plan", "producer/out")],
                    &[],
                ),
            ],
            multi_publisher_topics: Vec::new(),
            process_groups: groups(&[("g0", &["src", "producer"]), ("g1", &["consumer"])]),
            process_group_order: Vec::new(),
        };
        assert_eq!(
            detect(&config, &[("consumer", "/p/src/out")]),
            Vec::new(),
            "the plain `producer` -> `consumer` edge crosses a level boundary, so \
             the end-of-level barrier already orders it"
        );
    }

    /// GUARD 1 (the edge is triggering), pinned on inputs today's levelization
    /// REFUSES to mint.
    ///
    /// `GraphTopology::level_invariant_violation` rejects any levelization in
    /// which a trigger edge is not strictly level-increasing, so no reachable
    /// graph can present a same-level triggering pair — which means guard 1 is
    /// output-equivalent to guard 2 on every real input, and no plausible
    /// mutation of the shipped code kills it through the production seam. It
    /// is kept as the SEMANTICALLY primary predicate (see the fn docs), so it
    /// is pinned the only way it can be: as the pure function's stated
    /// contract, by handing it a `levels` and a `trigger_edges` that disagree.
    /// A refactor that deletes the `is_triggering` check fails here.
    #[test]
    fn a_triggering_edge_is_never_reported_even_at_one_level() {
        let config = pair_config(groups(&[("g0", &["producer"]), ("g1", &["consumer"])]));
        let entry_infos = infos_for(&config, &[]);
        let topology = GraphTopology::build(&config, &entry_infos).expect("topology");
        // Levels derived with NO trigger edges => both at level 0 ...
        let levels = topology
            .derive_levels(&TriggerEdges::new())
            .expect("levels");
        // ... while the classification handed to the detector says the edge
        // IS triggering. Deliberately inconsistent; see the doc above.
        let mut edges = TriggerEdges::new();
        edges.insert("consumer", PAIR_TOPIC);
        assert_eq!(
            split_same_level_non_trigger_pairs(
                &topology,
                &edges,
                &levels,
                &config.process_groups,
                LoadedNodeMetadata(&entry_infos),
            ),
            Vec::new(),
            "a triggering edge is a DAG edge and is never this detector's business"
        );
    }

    /// GUARD 4: a node absent from `process_groups` is unplaced, so no claim
    /// can be made about it.
    #[test]
    fn an_unplaced_node_is_not_reported() {
        let config = pair_config(groups(&[("g1", &["consumer"])]));
        assert_eq!(
            detect(&config, &[]),
            Vec::new(),
            "`producer` is in no group — nothing to attribute the race to"
        );
    }

    /// A `multi_publisher_topics` topic reports EACH split producer and only
    /// those: the co-located producer is silent, the cross-group one is not.
    /// Kills a "first producer only" / "any producer" regression.
    #[test]
    fn a_multi_publisher_topic_reports_only_its_split_producers() {
        let config = GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "tf".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                abs_node("pa", &[], Some("/tf")),
                abs_node("pb", &[], Some("/tf")),
                abs_node("c", &[("tf_in", "/tf")], None),
            ],
            multi_publisher_topics: vec!["/tf".to_string()],
            process_groups: groups(&[("g0", &["pa", "c"]), ("g1", &["pb"])]),
            process_group_order: Vec::new(),
        };
        assert_eq!(
            detect(&config, &[]),
            vec![pair("pb", "g1", "c", "tf_in", "g0", "/tf", 0)],
            "`pa` shares `c`'s group and is ordered; only `pb` crosses"
        );
    }

    /// GRAIN: one report per `(producer, consumer, input, topic)` tuple — two
    /// plain inputs of one node on one topic are two separate `trigger` marks
    /// to add, so they are two separate lines.
    #[test]
    fn two_plain_inputs_on_one_topic_are_reported_once_each() {
        let config = GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "twoin".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                node("producer", &[], &["out"]),
                node(
                    "consumer",
                    &[("plan", "producer/out"), ("plan_b", "producer/out")],
                    &[],
                ),
            ],
            multi_publisher_topics: Vec::new(),
            process_groups: groups(&[("g0", &["producer"]), ("g1", &["consumer"])]),
            process_group_order: Vec::new(),
        };
        assert_eq!(
            detect(&config, &[]),
            vec![
                pair("producer", "g0", "consumer", "plan", "g1", PAIR_TOPIC, 0),
                pair("producer", "g0", "consumer", "plan_b", "g1", PAIR_TOPIC, 0),
            ],
            "each unordered input is its own finding, in graph order"
        );
    }

    /// ANTI-TAUTOLOGY: a pure trigger chain, split process-per-node, reports
    /// NOTHING. Without this arm every "is_empty" assertion above would also
    /// pass a detector that never fires.
    #[test]
    fn a_clean_partition_reports_nothing() {
        let config = chain_config(groups(&[
            ("g0", &["n0"]),
            ("g1", &["n1"]),
            ("g2", &["n2"]),
            ("g3", &["n3"]),
            ("g4", &["n4"]),
        ]));
        let levels = chain_levels(&config);
        let entry_infos = infos_for(&config, &[]);
        let topology = GraphTopology::build(&config, &entry_infos).expect("topology");
        let mut edges = TriggerEdges::new();
        for (c, t) in [
            ("n1", "/p/n0/out"),
            ("n2", "/p/n1/out"),
            ("n3", "/p/n2/out"),
            ("n4", "/p/n3/out"),
        ] {
            edges.insert(c, t);
        }
        assert_eq!(
            split_same_level_non_trigger_pairs(
                &topology,
                &edges,
                &levels,
                &config.process_groups,
                LoadedNodeMetadata(&entry_infos),
            ),
            Vec::new(),
            "every edge in a trigger chain is barrier-ordered — the default \
             partition of a normal pipeline must stay silent"
        );
    }

    /// Determinism (Principle #7): the report order is the topology's, not a
    /// hash iteration accident.
    #[test]
    fn findings_are_reported_in_deterministic_order() {
        let config = GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "fan".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                node("pa", &[], &["out"]),
                node("pb", &[], &["out"]),
                node("c", &[("a", "pa/out"), ("b", "pb/out")], &[]),
            ],
            multi_publisher_topics: Vec::new(),
            process_groups: groups(&[("g0", &["pa", "pb"]), ("g1", &["c"])]),
            process_group_order: Vec::new(),
        };
        let oracle = vec![
            pair("pa", "g0", "c", "a", "g1", "/p/pa/out", 0),
            pair("pb", "g0", "c", "b", "g1", "/p/pb/out", 0),
        ];
        assert_eq!(detect(&config, &[]), oracle);
        assert_eq!(detect(&config, &[]), oracle, "and again, identically");
    }

    // -------------------------------------------------------------------
    // The REPORTER (`report_split_same_level_non_trigger_pairs`).
    // -------------------------------------------------------------------

    /// Count the WARN lines carrying `needle`, matching the LEVEL as a whole
    /// token — a reporter demoted to `debug!` would leave the operator with
    /// exactly the silence this warn exists to end while every field assertion
    /// still passed.
    fn warn_lines_containing<'a>(lines: &[&'a str], needle: &str) -> Vec<&'a str> {
        lines_at(lines, "WARN", needle)
    }

    /// The same predicate for the SPLIT-PAIR reporter, which is now
    /// `info!` rather than `warn!` — the finding is no longer an unactioned
    /// advisory but the INPUT to the mid-level barrier that FIXES it, so a
    /// warning would cry wolf on every run of a shape the runtime now orders.
    /// Still matched as a whole LEVEL TOKEN: demoted to `debug!` the breadcrumb
    /// vanishes from a default-`info` robot log, and the operator loses the only
    /// line saying a level is paying two rendezvous per step.
    fn info_lines_containing<'a>(lines: &[&'a str], needle: &str) -> Vec<&'a str> {
        lines_at(lines, "INFO", needle)
    }

    fn lines_at<'a>(lines: &[&'a str], level: &str, needle: &str) -> Vec<&'a str> {
        lines
            .iter()
            .filter(|l| l.split_whitespace().any(|t| t == level) && l.contains(needle))
            .copied()
            .collect()
    }

    /// One loud WARN per finding, each naming the producer, the consumer +
    /// input, the topic, the CONSEQUENCE, and the workarounds.
    #[tracing_test::traced_test]
    #[test]
    fn the_reporter_reports_once_per_pair_and_names_every_fix() {
        let pairs = vec![
            pair("producer", "g0", "consumer", "plan", "g1", PAIR_TOPIC, 0),
            pair("pb", "g1", "c", "tf_in", "g0", "/tf", 2),
        ];
        report_split_same_level_non_trigger_pairs("obstacle", &pairs);

        logs_assert(|lines: &[&str]| {
            let warns = info_lines_containing(lines, "SPLITS a same-level non-trigger edge");
            if warns.len() != 2 {
                return Err(format!(
                    "expected 2 INFO lines, got {}: {:?}",
                    warns.len(),
                    warns
                ));
            }
            Ok(())
        });

        // Field set, per finding — an operator greps by these keys.
        for needle in [
            "graph=obstacle",
            "producer=producer",
            "producer_group=g0",
            "consumer=consumer",
            "input=plan",
            "consumer_group=g1",
            "level=0",
            "remedy=replace_period_with_trigger",
            &format!("topic={PAIR_TOPIC}"),
            // ... and the second finding is genuinely distinct, not a repeat.
            "producer=pb",
            "input=tf_in",
            "topic=/tf",
            "level=2",
        ] {
            assert!(logs_contain(needle), "missing structured field `{needle}`");
        }

        // The line's CLAIM: it
        // must not say the pairing is "decided by OS scheduling" or that the bag can
        // fail its own re-execution; both are FALSE for this shape, and a
        // line that said them would send an operator chasing a
        // nondeterminism the runtime has just ordered. What it must say instead
        // is what the run now DOES — and the extra rendezvous is the operator's
        // only visible evidence that this level costs two generations a step.
        assert!(logs_contain("MID-LEVEL barrier rendezvous"));
        assert!(logs_contain("two generations per step instead of one"));
        assert!(logs_contain(
            "orders every group's step-start snapshots before any group ticks"
        ));
        // The RESIDUAL is pinned as hard as the claim, because it is the half a
        // reader is most likely to drop when re-wording: the fused block group
        // snapshots AFTER this rendezvous, so a block-involved consumer is still
        // unordered and still needs one of the workarounds.
        assert!(logs_contain("RESIDUAL"));
        assert!(logs_contain(
            "its own snapshot runs inside the fused block group, AFTER this rendezvous, and is \
             NOT ordered"
        ));
        // The workarounds survive VERBATIM — they are what the residual points
        // at — in the POLICY-ACCURATE spelling for a `period_ms` consumer (see
        // `a_period_consumers_fix_is_a_policy_change`).
        assert!(logs_contain("`#[input(trigger)]`"));
        assert!(logs_contain(
            "co-locating both nodes in ONE `process_groups:` group"
        ));
        assert!(logs_contain("`--single-process`"));
        // NEGATIVE half: the retired nondeterminism claims must be GONE. Without
        // this, prose carrying the retired claims passes every assertion above (the
        // markers could simply be appended to it) and the run would keep warning
        // about a hazard it does not have.
        for retired in [
            "decided by OS scheduling",
            "two live runs of this graph",
            "conditional mid-level barrier removes",
        ] {
            assert!(
                !logs_contain(retired),
                "the reporter still carries the pre-barrier claim `{retired}` — the mid-level \
                 barrier orders this shape, so that sentence is now false"
            );
        }
    }

    /// ANTI-TAUTOLOGY quiet control: a clean partition logs NOTHING. Without
    /// it, the "exactly 2" arm above would pass a reporter that also fired on
    /// well-ordered graphs.
    #[tracing_test::traced_test]
    #[test]
    fn the_reporter_is_silent_on_a_clean_partition() {
        report_split_same_level_non_trigger_pairs("clean", &[]);
        warn_trigger_classification_drift("clean", &[]);
        assert!(
            !logs_contain("STALE BUILD") && !logs_contain("this partition SPLITS"),
            "no findings must mean no output at all"
        );
    }

    // -------------------------------------------------------------------
    // The FLAG VECTOR (`mid_level_barrier_flags`) — the seam that
    // turns a finding into the runtime's extra rendezvous. Hand-written
    // oracles; the vector is index-compatible with `global_level_map` by
    // construction, which is what `install_barrier_participant` checks.
    // -------------------------------------------------------------------

    /// A clean partition flags NOTHING — the overwhelmingly common case, and
    /// the one that must stay free. This is also the ANTI-TAUTOLOGY control for
    /// every arm below: without it, "flags[g] is true" is satisfied by a helper
    /// that flags every level unconditionally.
    #[test]
    fn a_clean_partition_flags_no_level_for_a_mid_level_barrier() {
        assert_eq!(mid_level_barrier_flags(&[], 4), vec![false; 4]);
        // A zero-level graph is not constructible, but the helper must not
        // panic on one either.
        assert_eq!(mid_level_barrier_flags(&[], 0), Vec::<bool>::new());
    }

    /// ONLY the levels carrying a pair are flagged, and the vector is exactly
    /// as long as the level count it was sized from. The oracle is written by
    /// hand rather than derived from the input, so a helper that flagged
    /// `pair.level + 1` (or every level from 0 to the max) fails here.
    #[test]
    fn only_the_levels_carrying_a_split_pair_are_flagged() {
        let pairs = vec![
            pair("producer", "g0", "consumer", "plan", "g1", PAIR_TOPIC, 2),
            pair("pb", "g1", "c", "tf_in", "g0", "/tf", 0),
        ];
        assert_eq!(
            mid_level_barrier_flags(&pairs, 5),
            vec![true, false, true, false, false]
        );
    }

    /// TWO pairs on ONE level flag it ONCE. The flags decide how many
    /// generations a level burns, so a per-PAIR count would make a level with
    /// two split edges cross three generations while its peers crossed two —
    /// a desynchronised barrier, not a slower one. `Vec<bool>` makes that
    /// unrepresentable; this pins the property rather than the type.
    #[test]
    fn two_pairs_on_one_level_flag_it_exactly_once() {
        let pairs = vec![
            pair("pa", "g0", "c", "a", "g1", "/p/pa/out", 1),
            pair("pb", "g0", "c", "b", "g1", "/p/pb/out", 1),
        ];
        let flags = mid_level_barrier_flags(&pairs, 3);
        assert_eq!(flags, vec![false, true, false]);
        assert_eq!(
            flags.iter().filter(|f| **f).count(),
            1,
            "one level takes ONE extra rendezvous however many pairs sit on it"
        );
    }

    /// A pair reporting a level outside the planned count is DROPPED, not
    /// clamped and not panicked — the earlier behaviour for that level.
    /// Unreachable in production (both inputs come from one levelization), so
    /// it is a `debug_assert!`; this arm drives the RELEASE branch by calling
    /// the helper the way production would if the invariant ever broke.
    ///
    /// Run under `cfg(debug_assertions)` the helper would fire its assert, so
    /// the arm is gated to release — the assertion itself is the debug-build
    /// pin, and this is the "what does a shipped binary do" half.
    #[test]
    #[cfg(not(debug_assertions))]
    fn a_pair_outside_the_planned_levels_is_dropped_not_clamped() {
        let pairs = vec![pair("p", "g0", "c", "i", "g1", "/t", 9)];
        // NOT `vec![false, false, true]` (clamped onto the last level) and not
        // a panic: the level simply keeps one generation per step.
        assert_eq!(mid_level_barrier_flags(&pairs, 3), vec![false; 3]);
    }

    // -------------------------------------------------------------------
    // The REMEDY (workaround (a) must not advertise a COMPILE ERROR).
    //
    // `cerulion_macros::validate::validate_trigger_inference` rejects
    // `#[input(trigger)]` beside `period_ms`/`external`, and rejects a second
    // trigger input with no sync policy — so a fixed "mark the input
    // `#[input(trigger)]`" remedy is wrong on THREE of the five reachable
    // consumer policies.
    // -------------------------------------------------------------------

    /// ORACLE VECTOR: every `MacroPolicy` maps to its own remedy, and the
    /// tokens are pairwise distinct (an operator counting `remedy=` keys must
    /// be able to tell the classes apart).
    #[test]
    fn the_remedy_follows_the_consumers_macro_policy() {
        use cerulion_core::MacroPolicy as MP;
        let oracle: Vec<(Option<MP>, TriggerRemedy, &str)> = vec![
            (
                Some(MP::Period { period_ms: 10 }),
                TriggerRemedy::ReplacePeriodWithTrigger,
                "replace_period_with_trigger",
            ),
            (
                Some(MP::DataTrigger {
                    input_name: "trig".to_string(),
                }),
                TriggerRemedy::MarkTriggerAndAddSyncWindow,
                "mark_trigger_and_add_sync_window",
            ),
            (
                Some(MP::Sync { window_ms: 25 }),
                TriggerRemedy::MarkTrigger,
                "mark_trigger",
            ),
            (
                Some(MP::UnboundedSync),
                TriggerRemedy::MarkTrigger,
                "mark_trigger",
            ),
            (
                Some(MP::External),
                TriggerRemedy::ExternalNoTriggerGesture,
                "co_locate_only_external",
            ),
            (
                None,
                TriggerRemedy::NoDeclaredPolicy,
                "co_locate_only_no_policy",
            ),
        ];
        for (policy, want, token) in &oracle {
            let got = TriggerRemedy::for_policy(policy.as_ref());
            assert_eq!(got, *want, "wrong remedy for policy {policy:?}");
            assert_eq!(got.token(), *token, "wrong token for policy {policy:?}");
        }
        // Distinctness: 5 variants ⇒ 5 tokens.
        let tokens: HashSet<&str> = oracle.iter().map(|(_, r, _)| r.token()).collect();
        assert_eq!(tokens.len(), 5, "remedy tokens must be pairwise distinct");
    }

    /// A `period_ms` consumer must be told to change its POLICY. A remedy
    /// text ("making the input triggering (`#[input(trigger)]`)") is a COMPILE
    /// ERROR here, so the line must name the `period_ms` removal AND say the
    /// mark alone will not compile.
    #[tracing_test::traced_test]
    #[test]
    fn a_period_consumers_fix_is_a_policy_change_not_a_bare_trigger_mark() {
        report_split_same_level_non_trigger_pairs(
            "obstacle",
            &[pair_r(
                "producer",
                "g0",
                "consumer",
                "plan",
                "g1",
                PAIR_TOPIC,
                0,
                TriggerRemedy::ReplacePeriodWithTrigger,
            )],
        );
        assert!(logs_contain("remedy=replace_period_with_trigger"));
        assert!(logs_contain(
            "changing `consumer`'s node policy to DATA-TRIGGERED"
        ));
        assert!(logs_contain(
            "drop `period_ms` from its `#[cerulion_node(..)]` attribute"
        ));
        assert!(
            logs_contain("the mark alone will not compile"),
            "the operator must be told WHY the bare mark is not the fix"
        );
        // All three workarounds are still offered.
        assert!(logs_contain("OR (c) running `--single-process`."));
    }

    /// A `DataTrigger` consumer already has ONE trigger input, so a second
    /// mark needs a sync policy — the macro rejects two trigger inputs
    /// without one.
    #[tracing_test::traced_test]
    #[test]
    fn a_data_trigger_consumers_fix_pairs_the_mark_with_a_sync_window() {
        report_split_same_level_non_trigger_pairs(
            "fusion",
            &[pair_r(
                "producer",
                "g0",
                "consumer",
                "plan",
                "g1",
                PAIR_TOPIC,
                0,
                TriggerRemedy::MarkTriggerAndAddSyncWindow,
            )],
        );
        assert!(logs_contain("remedy=mark_trigger_and_add_sync_window"));
        assert!(logs_contain("`#[cerulion_node(sync_window_ms = N)]`"));
        assert!(logs_contain("`unbounded_sync`"));
        assert!(logs_contain("the mark alone will not compile"));
    }

    /// A `Sync` consumer CAN just take the mark — and must not be told to
    /// remove a `period_ms` it does not have.
    #[tracing_test::traced_test]
    #[test]
    fn a_sync_consumers_fix_is_the_bare_trigger_mark() {
        report_split_same_level_non_trigger_pairs(
            "fusion",
            &[pair_r(
                "producer",
                "g0",
                "consumer",
                "plan",
                "g1",
                PAIR_TOPIC,
                0,
                TriggerRemedy::MarkTrigger,
            )],
        );
        assert!(logs_contain("remedy=mark_trigger"));
        assert!(logs_contain(
            "marking `plan` `#[input(trigger)]` on `consumer`"
        ));
        assert!(logs_contain("sync alignment set"));
        assert!(
            !logs_contain("period_ms"),
            "a Sync consumer has no `period_ms` to drop — advising it would be nonsense"
        );
        assert!(
            !logs_contain("will not compile"),
            "the bare mark DOES compile here; saying otherwise would deter the real fix"
        );
    }

    /// An `external` consumer has NO trigger-mark remedy at all: the macro
    /// rejects `#[input(trigger)]` beside `external`, and a data-triggered
    /// node is no longer an ingress node. It must be offered exactly TWO
    /// workarounds, not three.
    #[tracing_test::traced_test]
    #[test]
    fn an_external_consumer_is_offered_only_the_two_fixes_that_exist() {
        report_split_same_level_non_trigger_pairs(
            "attach",
            &[pair_r(
                "producer",
                "g0",
                "bridge",
                "cfg",
                "g1",
                PAIR_TOPIC,
                0,
                TriggerRemedy::ExternalNoTriggerGesture,
            )],
        );
        assert!(logs_contain("remedy=co_locate_only_external"));
        assert!(logs_contain("there is NO `#[input(trigger)]` remedy here"));
        assert!(logs_contain("self-triggering `external` ingress node"));
        assert!(logs_contain(
            "Fix by (a) co-locating both nodes in ONE `process_groups:` group"
        ));
        assert!(
            !logs_contain("OR (c)"),
            "only TWO workarounds exist for an `external` consumer — a phantom (c) \
             implies a third"
        );
    }

    /// The unreachable-but-total arm: a consumer with no macro policy is told
    /// the truth about ITS shape (the runtime already fires it on any input),
    /// never the `external` prose.
    #[tracing_test::traced_test]
    #[test]
    fn a_policy_less_consumer_is_not_described_as_external() {
        report_split_same_level_non_trigger_pairs(
            "closure",
            &[pair_r(
                "producer",
                "g0",
                "consumer",
                "plan",
                "g1",
                PAIR_TOPIC,
                0,
                TriggerRemedy::NoDeclaredPolicy,
            )],
        );
        assert!(logs_contain("remedy=co_locate_only_no_policy"));
        assert!(logs_contain("declares no macro trigger policy"));
        assert!(!logs_contain("ingress node"));
    }

    /// The remedy is per CONSUMER, not per graph: one partition can split a
    /// `period_ms`, a `Sync`, an `external` and a `DataTrigger` consumer at
    /// once, and each gets its own gesture. Kills a "one remedy for the whole
    /// report" regression that the single-consumer arms above cannot see, and
    /// is the ONLY arm that drives `external` / `DataTrigger` consumers
    /// through the DETECTOR (the production path) rather than through
    /// `TriggerRemedy::for_policy` alone.
    #[test]
    fn two_consumers_of_different_policies_get_different_remedies() {
        let config = GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "mixed".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                node("producer", &[], &["out"]),
                node("periodic", &[("plan", "producer/out")], &[]),
                node(
                    "synced",
                    &[("plan", "producer/out"), ("a", "producer/out")],
                    &[],
                ),
                node("bridge", &[("cfg", "producer/out")], &[]),
                node(
                    "fuser",
                    &[("plan", "producer/out"), ("trig", "producer/out")],
                    &[],
                ),
            ],
            multi_publisher_topics: Vec::new(),
            process_groups: groups(&[
                ("g0", &["producer"]),
                ("g1", &["periodic", "synced", "bridge", "fuser"]),
            ]),
            process_group_order: Vec::new(),
        };
        // `bridge` is `external` (no trigger gesture) and `fuser` is a
        // `DataTrigger` on `trig` — so `fuser`'s OTHER input `plan` is a
        // non-trigger read whose remedy must pair the mark with a sync window.
        // (The `TriggerEdges` key is `(node, topic)`, so `fuser`'s declared
        // `trig` edge is deliberately NOT listed as triggering in the arms
        // below: both of its inputs read the SAME topic, and a triggering key
        // there would exclude the node entirely — a real graph would have
        // `trig` on a different topic, which changes nothing about the remedy
        // this arm is pinning.)
        let mixed: Vec<MetaSpec<'_>> = vec![
            ("bridge", Some(cerulion_core::MacroPolicy::External), &[]),
            (
                "fuser",
                Some(cerulion_core::MacroPolicy::DataTrigger {
                    input_name: "trig".to_string(),
                }),
                &["trig"],
            ),
        ];
        // `synced` is a Sync node whose `a` input IS trigger-marked; with the
        // (node, topic) key triggering, guard 1 excludes BOTH of its edges.
        let mut spec = mixed.clone();
        spec.push((
            "synced",
            Some(cerulion_core::MacroPolicy::Sync { window_ms: 25 }),
            &["a"],
        ));
        let found = detect_with_meta(&config, &[("synced", PAIR_TOPIC)], &spec);
        assert_eq!(
            found,
            vec![
                pair_r(
                    "producer",
                    "g0",
                    "periodic",
                    "plan",
                    "g1",
                    PAIR_TOPIC,
                    0,
                    TriggerRemedy::ReplacePeriodWithTrigger,
                ),
                pair_r(
                    "producer",
                    "g0",
                    "bridge",
                    "cfg",
                    "g1",
                    PAIR_TOPIC,
                    0,
                    TriggerRemedy::ExternalNoTriggerGesture,
                ),
                pair_r(
                    "producer",
                    "g0",
                    "fuser",
                    "plan",
                    "g1",
                    PAIR_TOPIC,
                    0,
                    TriggerRemedy::MarkTriggerAndAddSyncWindow,
                ),
                pair_r(
                    "producer",
                    "g0",
                    "fuser",
                    "trig",
                    "g1",
                    PAIR_TOPIC,
                    0,
                    TriggerRemedy::MarkTriggerAndAddSyncWindow,
                ),
            ],
            "the Sync consumer's edge IS triggering (guard 1) and is excluded; every other \
             consumer is reported with the remedy ITS OWN policy allows"
        );

        // Now un-trigger the Sync node's mark on that topic: it joins the
        // report too, with the remedy only a Sync node gets.
        let mut spec = mixed.clone();
        spec.push((
            "synced",
            Some(cerulion_core::MacroPolicy::Sync { window_ms: 25 }),
            &[],
        ));
        let both = detect_with_meta(&config, &[], &spec);
        let remedies: Vec<(&str, &str, TriggerRemedy)> = both
            .iter()
            .map(|p| (p.consumer.as_str(), p.consumer_input.as_str(), p.remedy))
            .collect();
        assert_eq!(
            remedies,
            vec![
                ("periodic", "plan", TriggerRemedy::ReplacePeriodWithTrigger),
                ("synced", "plan", TriggerRemedy::MarkTrigger),
                ("synced", "a", TriggerRemedy::MarkTrigger),
                ("bridge", "cfg", TriggerRemedy::ExternalNoTriggerGesture),
                ("fuser", "plan", TriggerRemedy::MarkTriggerAndAddSyncWindow),
                ("fuser", "trig", TriggerRemedy::MarkTriggerAndAddSyncWindow),
            ],
            "each consumer carries ITS OWN remedy — FOUR distinct classes in one report"
        );
    }

    // -------------------------------------------------------------------
    // PROVENANCE (`classify_split_same_level_non_trigger_pairs`): the verdict
    // must describe the build the runtime LOADED, never the source tree.
    // -------------------------------------------------------------------

    /// Levels the way the PLANNING RUNTIME derives them: from the LOADED
    /// metadata's own trigger classification.
    fn levels_from_loaded(config: &GraphConfig, loaded: &IndexMap<String, NodeInfo>) -> Levels {
        let topology = GraphTopology::build(config, loaded).expect("topology");
        let edges = cerulion_core::graph::build_trigger_edges(config, loaded);
        topology.derive_levels(&edges).expect("levels")
    }

    /// THE STALE-BUILD ARM. Source has just gained `#[input(trigger)]` on
    /// `plan` (so the SOURCE classification orders the pair), while the BUILT
    /// cdylib is still `period_ms` (so the LOADED classification — and
    /// `runtime.levels()` with it — does not).
    ///
    /// The verdict must follow the LOADED side: the run really is unordered,
    /// and a source-classified detector would stay silent about it:
    /// classifying from `source_entry_infos` yields an EMPTY `pairs` here.
    #[test]
    fn a_stale_build_classifies_from_the_loaded_metadata_not_the_source() {
        let config = pair_config(groups(&[("g0", &["producer"]), ("g1", &["consumer"])]));
        // BUILT cdylib: still `period_ms`, no trigger marks.
        let loaded = infos_for(&config, &[]);
        // SOURCE: the developer marked `plan` `#[input(trigger)]` and dropped
        // `period_ms` — the exact demo retarget, not yet rebuilt.
        let source = infos_for(
            &config,
            &[(
                "consumer",
                Some(cerulion_core::MacroPolicy::DataTrigger {
                    input_name: "plan".to_string(),
                }),
                &["plan"],
            )],
        );
        // Anti-vacuity: the two provenances really do classify differently.
        assert!(
            cerulion_core::graph::build_trigger_edges(&config, &source)
                .is_triggering("consumer", PAIR_TOPIC),
            "fixture bug: SOURCE must classify the edge as triggering"
        );
        assert!(
            !cerulion_core::graph::build_trigger_edges(&config, &loaded)
                .is_triggering("consumer", PAIR_TOPIC),
            "fixture bug: LOADED must classify the edge as non-triggering"
        );

        let levels = levels_from_loaded(&config, &loaded);
        assert_eq!(
            classify_split_same_level_non_trigger_pairs(
                &config,
                LoadedNodeMetadata(&loaded),
                &levels
            ),
            vec![pair(
                "producer", "g0", "consumer", "plan", "g1", PAIR_TOPIC, 0
            )],
            "the RUN is unordered — the verdict must describe the LOADED build, and the \
             remedy must be the LOADED policy's (`period_ms`), not the source's"
        );
        assert_eq!(
            classify_trigger_metadata_drift(
                &config,
                SourceNodeMetadata(&source),
                LoadedNodeMetadata(&loaded)
            ),
            vec![TriggerClassificationDrift {
                consumer: "consumer".to_string(),
                topic: PAIR_TOPIC.to_string(),
                source_says_triggering: true,
                loaded_says_triggering: false,
            }],
            "and the disagreement is NAMED, never a silent pick of one side"
        );
    }

    /// The OTHER direction: source dropped the trigger mark, the built cdylib
    /// still has it. Nothing is reported (the running build orders the pair —
    /// guard 1 AND the loaded levelization both exclude it), but the stale
    /// build is still named, because the operator is otherwise reading a
    /// verdict about code they are not looking at.
    #[test]
    fn a_stale_build_is_named_even_when_the_verdict_is_clean() {
        let config = pair_config(groups(&[("g0", &["producer"]), ("g1", &["consumer"])]));
        let loaded = infos_for(
            &config,
            &[(
                "consumer",
                Some(cerulion_core::MacroPolicy::DataTrigger {
                    input_name: "plan".to_string(),
                }),
                &["plan"],
            )],
        );
        let source = infos_for(&config, &[]);
        let levels = levels_from_loaded(&config, &loaded);
        assert_eq!(
            classify_split_same_level_non_trigger_pairs(
                &config,
                LoadedNodeMetadata(&loaded),
                &levels
            ),
            Vec::new(),
            "the RUNNING build orders the pair"
        );
        assert_eq!(
            classify_trigger_metadata_drift(
                &config,
                SourceNodeMetadata(&source),
                LoadedNodeMetadata(&loaded)
            ),
            vec![TriggerClassificationDrift {
                consumer: "consumer".to_string(),
                topic: PAIR_TOPIC.to_string(),
                source_says_triggering: false,
                loaded_says_triggering: true,
            }]
        );
    }

    /// ANTI-TAUTOLOGY: a freshly built workspace (identical provenances)
    /// reports NO drift — and still finds the split pair, so the arms above
    /// cannot be passing on a detector that reports drift unconditionally or
    /// on one that never fires.
    #[test]
    fn a_fresh_build_reports_no_drift_and_still_finds_the_pair() {
        let config = pair_config(groups(&[("g0", &["producer"]), ("g1", &["consumer"])]));
        let infos = infos_for(&config, &[]);
        let levels = levels_from_loaded(&config, &infos);
        assert_eq!(
            classify_trigger_metadata_drift(
                &config,
                SourceNodeMetadata(&infos),
                LoadedNodeMetadata(&infos)
            ),
            Vec::new(),
            "matching metadata is not drift"
        );
        assert_eq!(
            classify_split_same_level_non_trigger_pairs(
                &config,
                LoadedNodeMetadata(&infos),
                &levels
            ),
            vec![pair(
                "producer", "g0", "consumer", "plan", "g1", PAIR_TOPIC, 0
            )]
        );
    }

    /// The drift grain is `(consumer, topic)` — the `TriggerEdges` key — so a
    /// node reading ONE topic through TWO inputs is reported ONCE, not twice.
    #[test]
    fn drift_is_deduplicated_per_consumer_and_topic() {
        let config = GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "twoin".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                node("producer", &[], &["out"]),
                node(
                    "consumer",
                    &[("plan", "producer/out"), ("plan_b", "producer/out")],
                    &[],
                ),
            ],
            multi_publisher_topics: Vec::new(),
            process_groups: groups(&[("g0", &["producer"]), ("g1", &["consumer"])]),
            process_group_order: Vec::new(),
        };
        let loaded = infos_for(&config, &[]);
        let source = infos_for(
            &config,
            &[(
                "consumer",
                Some(cerulion_core::MacroPolicy::Sync { window_ms: 25 }),
                &["plan", "plan_b"],
            )],
        );
        let topology = GraphTopology::build(&config, &loaded).expect("topology");
        let drift = trigger_classification_drift(
            &topology,
            &cerulion_core::graph::build_trigger_edges(&config, &source),
            &cerulion_core::graph::build_trigger_edges(&config, &loaded),
        );
        assert_eq!(
            drift,
            vec![TriggerClassificationDrift {
                consumer: "consumer".to_string(),
                topic: PAIR_TOPIC.to_string(),
                source_says_triggering: true,
                loaded_says_triggering: false,
            }],
            "two inputs on one topic are ONE `(node, topic)` classification"
        );
    }

    /// The drift REPORTER: one loud WARN per disagreement, naming both sides
    /// and the rebuild.
    #[tracing_test::traced_test]
    #[test]
    fn the_drift_reporter_warns_once_per_disagreeing_edge() {
        warn_trigger_classification_drift(
            "obstacle",
            &[
                TriggerClassificationDrift {
                    consumer: "consumer".to_string(),
                    topic: PAIR_TOPIC.to_string(),
                    source_says_triggering: true,
                    loaded_says_triggering: false,
                },
                TriggerClassificationDrift {
                    consumer: "other".to_string(),
                    topic: "/tf".to_string(),
                    source_says_triggering: false,
                    loaded_says_triggering: true,
                },
            ],
        );
        logs_assert(|lines: &[&str]| {
            let warns = warn_lines_containing(lines, "STALE BUILD — this node's");
            if warns.len() != 2 {
                return Err(format!("expected 2 WARN lines, got {}", warns.len()));
            }
            Ok(())
        });
        for needle in [
            "graph=obstacle",
            "consumer=consumer",
            "source_says_triggering=true",
            "loaded_says_triggering=false",
            "consumer=other",
            "source_says_triggering=false",
            "loaded_says_triggering=true",
        ] {
            assert!(logs_contain(needle), "missing structured field `{needle}`");
        }
        assert!(logs_contain("cerulion node build"));
        assert!(
            logs_contain("The run uses the LOADED (built) classification"),
            "the operator must be told WHICH side the verdicts below describe"
        );
    }

    /// The two diagnostics are INDEPENDENT: each reports its own findings and
    /// neither swallows nor duplicates the other's. This is what lets the
    /// stale-build half be emitted before the refusing `validate_partition`
    /// while the split-pair half stays after it.
    #[tracing_test::traced_test]
    #[test]
    fn the_two_diagnostics_report_independently() {
        warn_trigger_classification_drift(
            "obstacle",
            &[TriggerClassificationDrift {
                consumer: "consumer".to_string(),
                topic: PAIR_TOPIC.to_string(),
                source_says_triggering: true,
                loaded_says_triggering: false,
            }],
        );
        logs_assert(|lines: &[&str]| {
            let stale = warn_lines_containing(lines, "STALE BUILD — this node's").len();
            let split = info_lines_containing(lines, "SPLITS a same-level non-trigger edge").len();
            if (stale, split) != (1, 0) {
                return Err(format!(
                    "the drift reporter must emit ONLY its own line; got ({stale} stale, \
                     {split} split)"
                ));
            }
            Ok(())
        });

        report_split_same_level_non_trigger_pairs(
            "obstacle",
            &[pair(
                "producer", "g0", "consumer", "plan", "g1", PAIR_TOPIC, 0,
            )],
        );
        logs_assert(|lines: &[&str]| {
            let stale = warn_lines_containing(lines, "STALE BUILD — this node's").len();
            let split = info_lines_containing(lines, "SPLITS a same-level non-trigger edge").len();
            if (stale, split) != (1, 1) {
                return Err(format!(
                    "the pair reporter must ADD its line without re-emitting the drift one; \
                     got ({stale} stale, {split} split)"
                ));
            }
            Ok(())
        });
    }

    /// A config whose topology cannot be built yields NO findings from either
    /// diagnostic and never refuses — the hoisted drift check runs BEFORE
    /// `validate_partition`, so a fallible diagnostic there would re-label the
    /// validator's refusal as its own error, and the split-pair advisory would
    /// abort a run it has no business refusing.
    ///
    /// The fixture is a genuinely unbuildable graph: two nodes publishing `/tf`
    /// with `/tf` NOT listed in `multi_publisher_topics`, which
    /// `GraphTopology::build` rejects.
    #[test]
    fn an_unbuildable_topology_yields_no_findings_and_never_refuses() {
        let config = GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "double".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                abs_node("pa", &[], Some("/tf")),
                abs_node("pb", &[], Some("/tf")),
                abs_node("c", &[("tf_in", "/tf")], None),
            ],
            // `/tf` deliberately NOT listed => double-producer => build Err.
            multi_publisher_topics: Vec::new(),
            process_groups: groups(&[("g0", &["pa", "c"]), ("g1", &["pb"])]),
            process_group_order: Vec::new(),
        };
        let infos = infos_for(&config, &[]);
        // Anti-vacuity: the fixture really is unbuildable (otherwise this arm
        // would pass for the wrong reason).
        assert!(
            GraphTopology::build(&config, &infos).is_err(),
            "fixture bug: the topology must fail to build"
        );
        // Both diagnostics are INFALLIBLE — there is no `?` a caller could get
        // wrong — and both stay silent rather than guess.
        assert_eq!(
            classify_trigger_metadata_drift(
                &config,
                SourceNodeMetadata(&infos),
                LoadedNodeMetadata(&infos)
            ),
            Vec::new()
        );
        // Any `Levels` will do — the function returns before it reads them.
        // Borrowed from the buildable twin (`/tf` listed) so the fixture is a
        // real levelization rather than a fabricated one.
        let buildable = GraphConfig {
            multi_publisher_topics: vec!["/tf".to_string()],
            ..config.clone()
        };
        let levels = levels_from_loaded(&buildable, &infos_for(&buildable, &[]));
        assert_eq!(
            classify_split_same_level_non_trigger_pairs(
                &config,
                LoadedNodeMetadata(&infos),
                &levels
            ),
            Vec::new()
        );
    }

    // -------------------------------------------------------------------
    // The BACKPRESSURE half of the stale-build cross-check
    // (`classify_backpressure_metadata_drift` +
    // `warn_backpressure_classification_drift`).
    //
    // Its trigger sibling above is pinned by four behavioural arms; without
    // these, every feature-deleted variant survives the
    // whole suite — the body reduced to `Vec::new()`, the
    // `source_policy == loaded_policy` guard INVERTED (warning on every
    // healthy input while missing every drifted one), the reporter demoted
    // to `debug!`. These arms mirror the sibling's construction: hand-built
    // source-vs-loaded provenances, hand-written oracles on every field,
    // and a paired anti-tautology so a classifier that fires on everything
    // cannot pass a positive arm.
    // -------------------------------------------------------------------

    use cerulion_core::graph::node::BackpressurePolicy;

    /// A per-input backpressure declaration for [`infos_bp`]:
    /// `(node_id, input_name, policy)`.
    type BackpressureSpec<'a> = (&'a str, &'a str, BackpressurePolicy);

    /// [`infos_for`]'s backpressure sibling. Every node is
    /// `Period { period_ms: 10 }` with no trigger marks (the demo's
    /// `obstacle_avoidance` shape), and each `(node, input)` named in `spec`
    /// carries the declared policy instead of the `DropOldest` default.
    ///
    /// It exists because `infos_for` hardcodes
    /// `BackpressurePolicy::default()`, so it cannot build the two
    /// DISAGREEING provenances this cross-check compares — and
    /// `NodeInfo::input_meta` is `pub(crate)` to `cerulion_core`, so the
    /// metas must be built carrying the policy rather than patched after.
    fn infos_bp(config: &GraphConfig, spec: &[BackpressureSpec<'_>]) -> IndexMap<String, NodeInfo> {
        config
            .nodes
            .iter()
            .map(|n| {
                let input_meta: Vec<cerulion_core::graph::node::InputMeta> = n
                    .inputs
                    .iter()
                    .map(|i| cerulion_core::graph::node::InputMeta {
                        name: i.name.clone(),
                        schema_hash: 0,
                        trigger: false,
                        depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
                        backpressure: spec
                            .iter()
                            .find(|(node_id, input, _)| {
                                *node_id == n.id.as_str() && *input == i.name.as_str()
                            })
                            .map(|(_, _, policy)| *policy)
                            .unwrap_or_default(),
                        expect_within_ms: None,
                    })
                    .collect();
                (
                    n.id.clone(),
                    NodeInfo::with_meta(input_meta, Vec::new())
                        .with_policy(cerulion_core::MacroPolicy::Period { period_ms: 10 }),
                )
            })
            .collect()
    }

    /// A hand-written expected backpressure finding.
    fn bp_drift(
        consumer: &str,
        input: &str,
        topic: &str,
        source_policy: BackpressurePolicy,
        loaded_policy: BackpressurePolicy,
    ) -> BackpressureClassificationDrift {
        BackpressureClassificationDrift {
            consumer: consumer.to_string(),
            input: input.to_string(),
            topic: topic.to_string(),
            source_policy,
            loaded_policy,
        }
    }

    /// A consumer reading TWO producers through FOUR inputs — two of them on
    /// ONE topic, which is exactly the shape a `(consumer, topic)` grain
    /// (the trigger sibling's key) would collapse.
    ///
    /// Topics are created in producer-declaration order and consumers appended
    /// in input-declaration order (`GraphTopology::build`'s two passes), so the
    /// report order is `/p/pa/out`'s inputs then `/p/pb/out`'s — deterministic,
    /// and asserted as a whole vector rather than a set.
    fn bp_config() -> GraphConfig {
        GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "obstacle".to_string(),
            prefix: PREFIX.to_string(),
            nodes: vec![
                node("pa", &[], &["out"]),
                node("pb", &[], &["out"]),
                node(
                    "consumer",
                    &[
                        ("plan", "pa/out"),
                        ("plan_b", "pa/out"),
                        ("cfg", "pb/out"),
                        ("cfg_ok", "pb/out"),
                    ],
                    &[],
                ),
            ],
            multi_publisher_topics: Vec::new(),
            process_groups: groups(&[("g0", &["pa", "pb"]), ("g1", &["consumer"])]),
            process_group_order: Vec::new(),
        }
    }

    const BP_TOPIC_A: &str = "/p/pa/out";
    const BP_TOPIC_B: &str = "/p/pb/out";

    /// HEADLINE (positive): the DANGEROUS direction — source says
    /// `drop_oldest` (so the auto-partitioner did NOT co-locate) while
    /// the BUILT cdylib says `block` (so the worker dies at
    /// `GraphTopology::validate` blaming an external publisher the operator
    /// does not have). Every field hand-oracled: consumer, input, topic, and
    /// BOTH policies.
    ///
    /// A body reduced to `Vec::new()` yields nothing here; the
    /// INVERTED `source_policy == loaded_policy` guard skips exactly this
    /// record while reporting the healthy sibling inputs instead.
    #[test]
    fn a_backpressure_disagreement_is_reported_with_both_sides() {
        let config = pair_config(groups(&[("g0", &["producer"]), ("g1", &["consumer"])]));
        // BUILT cdylib: `#[input(backpressure = block)]`.
        let loaded = infos_bp(&config, &[("consumer", "plan", BackpressurePolicy::Block)]);
        // SOURCE: still the `drop_oldest` default — the partition was derived
        // from THIS, and it did not co-locate the pair.
        let source = infos_bp(&config, &[]);

        // Anti-vacuity: the two provenances really do declare different
        // policies (otherwise this arm would pass on a classifier that
        // reports drift unconditionally).
        assert_eq!(
            source["consumer"].input_meta()[0].backpressure,
            BackpressurePolicy::DropOldest,
            "fixture bug: SOURCE must declare `drop_oldest`"
        );
        assert_eq!(
            loaded["consumer"].input_meta()[0].backpressure,
            BackpressurePolicy::Block,
            "fixture bug: LOADED must declare `block`"
        );

        assert_eq!(
            classify_backpressure_metadata_drift(
                &config,
                SourceNodeMetadata(&source),
                LoadedNodeMetadata(&loaded)
            ),
            vec![bp_drift(
                "consumer",
                "plan",
                PAIR_TOPIC,
                BackpressurePolicy::DropOldest,
                BackpressurePolicy::Block,
            )],
            "the run is gated by the LOADED `block` while the partition was \
             derived from the SOURCE `drop_oldest` — the disagreement must be \
             NAMED, with both sides on the record"
        );
    }

    /// ANTI-TAUTOLOGY: a freshly built workspace logs NOTHING — the doc's
    /// explicit claim, and the arm without which the headline above passes a
    /// classifier that reports every input.
    ///
    /// The discriminator is IN BODY on the SAME fixture: flipping ONLY the
    /// loaded side must produce exactly one record, so "zero" cannot be the
    /// answer of a classifier that is simply inert here (a `Vec::new()` body
    /// passes the first half and fails the second).
    #[test]
    fn a_freshly_built_workspace_reports_no_backpressure_drift() {
        let config = pair_config(groups(&[("g0", &["producer"]), ("g1", &["consumer"])]));
        let agreed = infos_bp(&config, &[("consumer", "plan", BackpressurePolicy::Block)]);
        assert_eq!(
            classify_backpressure_metadata_drift(
                &config,
                SourceNodeMetadata(&agreed),
                LoadedNodeMetadata(&agreed)
            ),
            Vec::new(),
            "matching declarations are not drift — a freshly built workspace \
             logs NOTHING"
        );

        // The same fixture, ONE side changed: the classifier is live here.
        let drifted = infos_bp(&config, &[]);
        assert_eq!(
            classify_backpressure_metadata_drift(
                &config,
                SourceNodeMetadata(&agreed),
                LoadedNodeMetadata(&drifted)
            ),
            vec![bp_drift(
                "consumer",
                "plan",
                PAIR_TOPIC,
                BackpressurePolicy::Block,
                BackpressurePolicy::DropOldest,
            )],
            "and the harmless direction (source `block`, cdylib `drop_oldest` — \
             a needlessly large process group) is still NAMED"
        );
    }

    /// The GRAIN is `(consumer, input)`, not `(consumer, topic)` — the design
    /// claim on `BackpressureClassificationDrift` itself: one node reading one
    /// topic through two inputs may legitimately declare `block` on one and
    /// `drop_oldest` on the other, so merging them onto the topic key would
    /// force one of the two onto the wrong row.
    ///
    /// Drives THREE distinct disagreements in one report — both directions
    /// across the `block`/`drop_oldest` boundary plus a same-variant
    /// `sample(N)` change of N — beside a FOURTH input that AGREES and must
    /// contribute nothing.
    #[test]
    fn each_input_carries_its_own_disagreement_and_direction() {
        let config = bp_config();
        let source = infos_bp(
            &config,
            &[
                ("consumer", "plan", BackpressurePolicy::Block),
                // `plan_b` left at the `drop_oldest` default.
                ("consumer", "cfg", BackpressurePolicy::Sample(5)),
                ("consumer", "cfg_ok", BackpressurePolicy::Block),
            ],
        );
        let loaded = infos_bp(
            &config,
            &[
                // `plan` left at the `drop_oldest` default.
                ("consumer", "plan_b", BackpressurePolicy::Block),
                ("consumer", "cfg", BackpressurePolicy::Sample(9)),
                ("consumer", "cfg_ok", BackpressurePolicy::Block),
            ],
        );

        assert_eq!(
            classify_backpressure_metadata_drift(
                &config,
                SourceNodeMetadata(&source),
                LoadedNodeMetadata(&loaded)
            ),
            vec![
                bp_drift(
                    "consumer",
                    "plan",
                    BP_TOPIC_A,
                    BackpressurePolicy::Block,
                    BackpressurePolicy::DropOldest,
                ),
                bp_drift(
                    "consumer",
                    "plan_b",
                    BP_TOPIC_A,
                    BackpressurePolicy::DropOldest,
                    BackpressurePolicy::Block,
                ),
                bp_drift(
                    "consumer",
                    "cfg",
                    BP_TOPIC_B,
                    BackpressurePolicy::Sample(5),
                    BackpressurePolicy::Sample(9),
                ),
            ],
            "two inputs on ONE topic are TWO findings (the `(consumer, input)` \
             grain), a same-variant `sample(N)` change of N is a real \
             disagreement, and the agreeing fourth input contributes nothing"
        );
    }

    /// An input the SOURCE parse does not know about is an ABSENCE, not a
    /// disagreement — the classifier's own documented answer, and the reason
    /// it `continue`s rather than inventing a `drop_oldest` claim the source
    /// never made (`validate_graph` speaks for a genuinely missing port).
    ///
    /// Both shapes: the whole NODE unknown to the source, and the node known
    /// with the INPUT absent from its metadata (a raw-FFI node, whose
    /// `parse_info_json` carries input NAMES only, so `input_meta` arrives
    /// empty). Paired with an in-body discriminator so "zero" is not the
    /// answer of an inert classifier.
    #[test]
    fn an_input_the_source_does_not_know_is_an_absence_not_a_disagreement() {
        let config = pair_config(groups(&[("g0", &["producer"]), ("g1", &["consumer"])]));
        let loaded = infos_bp(&config, &[("consumer", "plan", BackpressurePolicy::Block)]);

        // (a) the whole node is unknown to the source parse.
        let mut node_absent = infos_bp(&config, &[]);
        node_absent.shift_remove("consumer");
        assert_eq!(
            classify_backpressure_metadata_drift(
                &config,
                SourceNodeMetadata(&node_absent),
                LoadedNodeMetadata(&loaded)
            ),
            Vec::new(),
            "a node the SOURCE does not know is an absence, not a drift"
        );

        // (b) the node is known but declares no input metadata at all.
        let mut input_absent = infos_bp(&config, &[]);
        input_absent.insert(
            "consumer".to_string(),
            NodeInfo::with_meta(Vec::new(), Vec::new())
                .with_policy(cerulion_core::MacroPolicy::Period { period_ms: 10 }),
        );
        assert_eq!(
            classify_backpressure_metadata_drift(
                &config,
                SourceNodeMetadata(&input_absent),
                LoadedNodeMetadata(&loaded)
            ),
            Vec::new(),
            "an input the SOURCE does not declare is an absence, not a drift"
        );

        // DISCRIMINATOR: the same loaded side against a source that DOES know
        // the input reports the disagreement — so the two empties above are
        // not an inert classifier's answer.
        assert_eq!(
            classify_backpressure_metadata_drift(
                &config,
                SourceNodeMetadata(&infos_bp(&config, &[])),
                LoadedNodeMetadata(&loaded)
            ),
            vec![bp_drift(
                "consumer",
                "plan",
                PAIR_TOPIC,
                BackpressurePolicy::DropOldest,
                BackpressurePolicy::Block,
            )],
        );
    }

    /// The backpressure drift REPORTER: one loud WARN per disagreeing input,
    /// naming both sides and the rebuild — matched at the LEVEL TOKEN as well
    /// as the message, because a text-only predicate passes a `warn!` →
    /// `debug!` demotion while every field assertion still holds.
    ///
    /// The healthy case (an EMPTY finding list logs NOTHING) is driven FIRST,
    /// in this same body, so the absence guard cannot be vacuous: the positive
    /// count that follows proves the capture really sees this reporter's lines.
    #[tracing_test::traced_test]
    #[test]
    fn the_backpressure_reporter_warns_once_per_disagreeing_input() {
        warn_backpressure_classification_drift("obstacle", &[]);
        logs_assert(|lines: &[&str]| {
            let warns = warn_lines_containing(lines, "STALE BUILD — this input's").len();
            if warns != 0 {
                return Err(format!(
                    "a freshly built workspace must log NOTHING; got {warns} WARN lines"
                ));
            }
            Ok(())
        });

        warn_backpressure_classification_drift(
            "obstacle",
            &[
                bp_drift(
                    "consumer",
                    "plan",
                    PAIR_TOPIC,
                    BackpressurePolicy::DropOldest,
                    BackpressurePolicy::Block,
                ),
                bp_drift(
                    "other",
                    "cfg",
                    "/tf",
                    BackpressurePolicy::Sample(5),
                    BackpressurePolicy::Sample(9),
                ),
            ],
        );
        logs_assert(|lines: &[&str]| {
            let warns = warn_lines_containing(lines, "STALE BUILD — this input's").len();
            if warns != 2 {
                return Err(format!("expected 2 WARN lines, got {warns}"));
            }
            Ok(())
        });
        let pair_topic_field = format!("topic={PAIR_TOPIC}");
        for needle in [
            "graph=obstacle",
            "consumer=consumer",
            "input=plan",
            pair_topic_field.as_str(),
            "source_policy=DropOldest",
            "loaded_policy=Block",
            "consumer=other",
            "input=cfg",
            "topic=/tf",
            "source_policy=Sample(5)",
            "loaded_policy=Sample(9)",
        ] {
            assert!(logs_contain(needle), "missing structured field `{needle}`");
        }
        assert!(
            logs_contain("cerulion node build"),
            "the operator must be told how to make the two agree"
        );
        assert!(
            logs_contain("will kill this node's worker at graph build"),
            "the DANGEROUS direction's consequence must be named — that is why \
             this cross-check exists"
        );
    }
}
