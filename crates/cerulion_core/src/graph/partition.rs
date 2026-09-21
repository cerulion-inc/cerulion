// SPDX-License-Identifier: AGPL-3.0-only
//! Cross-process partition derivation.
//! Turns a `GraphConfig::process_groups` declaration + the GLOBAL levelization
//! (Kahn-derived, or the `level_assignments:`-refined assignment when
//! the graph carries one) into per-group barrier participant-maps. The SOURCE of the
//! groups is intentionally decoupled (a `&GraphConfig` today; a deployment
//! file or the auto-partitioner later) so only this module changes.
//!
//! # What a participant-map is
//!
//! The cross-process barrier rendezvouses every context at every
//! GLOBAL DAG-level boundary so all processes advance in lockstep (Principle
//! #7: replay = live). Each group needs a `global_level_map: Vec<Option<usize>>`
//! of length = the global level count: `Some(local_idx)` if the group OWNS ≥1
//! node at that global level (the `local_idx` increments per owned global
//! level, in ascending global order), `None` if the group only RENDEZVOUSES
//! there (owns nothing). This is exactly the contract documented on
//! `GraphRuntime::set_barrier_participant_for_test` — the non-`None` entries
//! are a strictly-increasing bijection onto `0..local_count` by construction
//! (a contiguous-split index map).
//!
//! # Scope
//!
//! Config field + validation + this derivation API + tests. The runtime/barrier
//! WIRING (feeding these maps into `GraphRuntime`) is a LATER PR — this module
//! only produces a tested `pub` value.
//!
//! # The cost-aware auto-partitioner
//!
//! [`auto_partition`] DERIVES the `process_groups` shape (which
//! [`derive_process_groups`] above consumes) from a graph's dataflow + a cost
//! snapshot, instead of reading a hand-written declaration. The policy is
//! **process-per-node BASELINE + validated GREEDY FUSION**: every
//! node starts in its own group (maximum fault isolation), and tightly-coupled
//! low-latency chains are fused under a per-group compute budget, minimizing
//! cross-process topic edges.
//!
//! Fusion score `coupling(edge) = rate(edge) × (cross_ns − intra_ns)` (the
//! per-second latency saved by keeping the edge in-process); edges are fused
//! greedily in descending coupling.
//!
//! ## The re-levelization crux (the REAL constraint)
//!
//! Plan-time contiguity of the participant-map is NECESSARY BUT NOT SUFFICIENT.
//! A group process re-levelizes its OWN induced subgraph at spawn — and
//! `GraphRuntime::install_barrier_participant` REQUIRES that local level count
//! to equal the number of GLOBAL levels the group owns (`locals ==
//! 0..local_count`). A contiguous global band whose bridge node lives in a
//! FOREIGN group collapses that re-levelization: the downstream member loses
//! its in-group predecessor and re-levelizes to a lower local level, so
//! `local_count < owned_global_count` and the spawner rejects the map. Every
//! candidate fusion in [`auto_partition`] must therefore pass
//! [`validate_partition`]'s per-group subgraph re-levelization bijection check
//! (which reuses the REAL [`GraphTopology::derive_levels`] on the group's
//! induced sub-topology) or it is not fused; a caller-supplied partition that
//! fails is rejected with a diagnostic NAMING THE BRIDGE NODE. The bijection
//! check alone is NOT sufficient for contiguity, though: a group
//! owning NON-CONTIGUOUS global levels does NOT always fail re-levelization —
//! a DIRECT in-group edge spanning the gap (owned `{0,4}` with a direct `0→4`
//! edge) re-levelizes to a gap-free local band `{0,1}`, so `local_count ==
//! owned_global_count` and the bijection PASSES even though the owned band is
//! gapped. The barrier's participant-map is a CONTIGUOUS-split index map that
//! cannot represent a gapped band, so `check_group` applies an EXPLICIT
//! contiguity test (the owned global levels must span a gap-free band) IN
//! ADDITION to the bijection; a gapped group is rejected `NonContiguous`.
//! Passing BOTH checks also guarantees the coarsened group-level graph is
//! acyclic, so no separate cycle check is needed.
//!
//! ## Determinism (Principle #7)
//!
//! Integer-only cost arithmetic (ns + millihertz — no float ordering),
//! graph-order tiebreaks, and `IndexMap`/`BTreeMap` iteration mean the same
//! inputs yield a byte-identical [`AutoPartition`]. The output freezes the cost
//! snapshot it was derived from alongside the grouping.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::num::NonZeroUsize;

use indexmap::{IndexMap, IndexSet};

use crate::error::{TransportError, TransportResult};
use crate::graph::config::{GraphConfig, NodeDef};
use crate::graph::node::{BackpressurePolicy, NodeInfo};
use crate::graph::topology::{resolve_levels, CreditBar, GraphTopology, Levels, TriggerEdges};
use crate::scheduler::TraceEntry;

/// One process group's derived barrier participation.
///
/// Produced by [`derive_process_groups`] from a validated
/// [`GraphConfig::process_groups`] + the graph's global [`Levels`]. The
/// [`global_level_map`](Self::global_level_map) is the per-context barrier
/// participant-map (see the module docs + the contract on
/// `GraphRuntime::set_barrier_participant_for_test`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProcessGroup {
    /// The declared group name (e.g. `"perception"`).
    pub name: String,
    /// The group's cross-process rank — the index of `name` in the rank
    /// order: the `process_groups` DECLARATION (listing) order by default, or
    /// the explicit `process_group_order` list when one is provided. The
    /// cross-process trace-merge tiebreaker + barrier ordering.
    pub rank: usize,
    /// The barrier participant-map: length = the global level count.
    /// `Some(local)` at global level `g` iff this group owns ≥1 node at `g`
    /// (`local` increments per owned global level, in ascending `g` order);
    /// `None` if the group only rendezvouses at `g`. The non-`None` entries
    /// form a strictly-increasing bijection onto `0..local_count` by
    /// construction.
    pub global_level_map: Vec<Option<usize>>,
}

/// Validate a graph's `process_groups` declaration.
///
/// When `process_groups` is non-empty:
/// * every node id in `config.nodes` appears in EXACTLY ONE group (rejects an
///   ORPHAN — an unlisted node — and a node listed in ≥2 groups);
/// * every node id LISTED in a group EXISTS in `config.nodes` (rejects a
///   DANGLING reference);
/// * when `config.process_group_order` is non-empty it MUST be a permutation
///   of the group names — every group named EXACTLY once (rejects an UNKNOWN
///   group name, a MISSING group, and a DUPLICATE name in the order list).
///
/// An empty map ⇒ `Ok` (treated as absent — single-process monolith). Errors
/// name the offending node/group via [`TransportError::GraphError`], matching
/// the rest of `validate_graph`.
pub fn validate_process_groups(config: &GraphConfig) -> TransportResult<()> {
    if config.process_groups.is_empty() {
        // `process_groups` and `process_group_order` are independent
        // `#[serde(default)]` fields, so an order list can be set with NO
        // groups. That config passed validation silently AND made
        // `derive_process_groups` panic (the override branch's `.get(name)`
        // would `.expect()` on the empty map). Reject it loudly before the
        // truly-empty (monolith) Ok path.
        if !config.process_group_order.is_empty() {
            return Err(TransportError::GraphError {
                reason: "process_group_order is set but process_groups is empty; declare groups \
                         in process_groups or remove process_group_order"
                    .to_string(),
            });
        }
        // Absent ⇒ single-process monolith; nothing to validate.
        return Ok(());
    }

    // Only NATIVE nodes join process groups: a `ros2:` entry is spawned by
    // `graph run` as its own supervised process (no barrier participation, no
    // rank), so it is neither required below (the orphan check) nor allowed
    // here — a group listing one is refused BY NAME with a suggested fix, rather than
    // as a dangling reference.
    let declared_nodes: HashSet<&str> = config.native_nodes().map(|n| n.id.as_str()).collect();
    let ros2_nodes: HashSet<&str> = config.ros2_nodes().map(|n| n.id.as_str()).collect();

    // (1) Every LISTED node exists, and no node is listed in ≥2 groups.
    // `assignment` maps a node id to the FIRST group that claimed it, so the
    // second claim names both groups in the diagnostic.
    let mut assignment: std::collections::HashMap<&str, &str> = std::collections::HashMap::new();
    for (group_name, members) in &config.process_groups {
        // An empty/whitespace-only name is a real rank-carrying ghost group
        // that later feeds barrier name derivation. Reject it before anything
        // else.
        if group_name.trim().is_empty() {
            return Err(TransportError::GraphError {
                reason: "process group name is empty or whitespace-only; give every group a \
                     non-empty name (e.g. 'perception', 'local_planner')"
                    .to_string(),
            });
        }
        // An empty group owns nothing yet still consumes a rank, silently
        // shifting every later group's `rank` (a ghost barrier participant +
        // silent renumbering = a determinism footgun). Reject it before
        // iterating members.
        if members.is_empty() {
            return Err(TransportError::GraphError {
                reason: format!(
                    "process group '{group_name}' is empty; assign at least one node or remove it \
                     (an empty group owns nothing and silently shifts later groups' ranks)"
                ),
            });
        }
        for node_id in members {
            if ros2_nodes.contains(node_id.as_str()) {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "process_groups group '{group_name}' lists '{node_id}', which is a \
                         `ros2:` entry — ROS 2 processes are spawned by `cerulion graph run` as \
                         their own supervised processes and cannot join a process group; remove \
                         it from the group"
                    ),
                });
            }
            if !declared_nodes.contains(node_id.as_str()) {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "process_groups group '{group_name}' lists node '{node_id}', which is \
                         not declared in the graph's `nodes:` (dangling reference)"
                    ),
                });
            }
            if let Some(prev) = assignment.insert(node_id.as_str(), group_name.as_str()) {
                // Distinguish a SAME-group duplicate (listed twice in ONE group)
                // from a cross-group double-assignment — the cross-group message
                // ("'P0' and 'P0'") is self-contradictory for the same-group case.
                let reason = if prev == group_name.as_str() {
                    format!(
                        "node '{node_id}' is listed twice in the same process group \
                         '{group_name}'; each node must appear exactly once"
                    )
                } else {
                    format!(
                        "node '{node_id}' is listed in more than one process group \
                         ('{prev}' and '{group_name}'); every node must appear in EXACTLY \
                         one group"
                    )
                };
                return Err(TransportError::GraphError { reason });
            }
        }
    }

    // (2) Every DECLARED native node appears in some group (no orphans). A
    // `ros2:` entry is exempt — it is spawned outside the partition.
    for node in config.native_nodes() {
        if !assignment.contains_key(node.id.as_str()) {
            return Err(TransportError::GraphError {
                reason: format!(
                    "node '{}' is not assigned to any process group; when \
                     `process_groups:` is present every node must appear in EXACTLY one \
                     group (add it to a group, or remove `process_groups:` for a \
                     single-process graph)",
                    node.id
                ),
            });
        }
    }

    // (3) When an explicit `process_group_order` is given it must be a
    // PERMUTATION of the group names — every group named exactly once. An
    // off-by-one here silently reorders cross-process ranks (a determinism
    // footgun), so reject UNKNOWN names, MISSING groups, and DUPLICATES loudly.
    if !config.process_group_order.is_empty() {
        let mut seen: HashSet<&str> = HashSet::new();
        for name in &config.process_group_order {
            // Unknown group name in the order list.
            if !config.process_groups.contains_key(name) {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "process_group_order lists '{name}', which is not a declared group in \
                         `process_groups`; the order list must name only declared groups"
                    ),
                });
            }
            // Duplicate name in the order list.
            if !seen.insert(name.as_str()) {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "process_group_order lists '{name}' more than once; each group must \
                         appear EXACTLY once (the order list is a permutation of the groups)"
                    ),
                });
            }
        }
        // Missing group: a declared group never named in the order list.
        for group_name in config.process_groups.keys() {
            if !seen.contains(group_name.as_str()) {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "process group '{group_name}' is missing from process_group_order; when \
                         the order list is present it must name EXACTLY one of every declared \
                         group (add '{group_name}', or remove process_group_order to use \
                         declaration order)"
                    ),
                });
            }
        }
    }

    Ok(())
}

/// Derive each group's barrier participant-map from the
/// graph's `process_groups` declaration + the GLOBAL [`Levels`] (Kahn-derived,
/// or the `level_assignments:`-refined assignment — the walk is
/// count-agnostic either way).
///
/// Iterates the groups in RANK order — the explicit `config.process_group_order`
/// when it is non-empty, else the `config.process_groups` declaration (listing)
/// order (the insertion-ordered `IndexMap`); `rank` is the position in that
/// order. For each group it builds a `global_level_map` of length `levels.len()`:
/// at global level `g`,
/// `Some(local)` iff the group owns ≥1 node at `g` (a node is at `g` iff
/// `levels.level_of(node) == Some(g)`), with `local` incrementing per owned
/// global level in ascending `g` order; else `None`. Returns the groups in rank
/// order: `groups[i].rank == i` as returned.
///
/// This is a `pub` reuse surface, so it FIRST runs [`validate_process_groups`]
/// (structural — orphans / dangling refs / double-assignment / empty groups) on
/// the config (idempotent, O(nodes); production also validates via
/// `validate_graph`, so it double-validates cheaply). Validation is structural
/// and does NOT check levels, so a config/levels MISMATCH — e.g. a stale or
/// smaller [`Levels`] missing a partition member — is still reachable; that
/// member-with-no-level case returns a [`TransportError`] rather than panicking.
pub fn derive_process_groups(
    config: &GraphConfig,
    levels: &Levels,
) -> TransportResult<Vec<ProcessGroup>> {
    // Structural validation first (safe `pub` reuse surface); the level-mismatch
    // check below is the only inconsistency validation cannot catch.
    validate_process_groups(config)?;

    let level_count = levels.len();
    let mut groups: Vec<ProcessGroup> = Vec::with_capacity(config.process_groups.len());

    // Rank order: the explicit `process_group_order` when present, else the
    // `process_groups` declaration (listing) order (insertion-ordered IndexMap).
    // Validation above proved a non-empty order list is a permutation of the
    // keys, so every lookup here resolves.
    let ordered: Vec<(&String, &Vec<String>)> = if config.process_group_order.is_empty() {
        config.process_groups.iter().collect()
    } else {
        config
            .process_group_order
            .iter()
            .map(|name| {
                let members = config.process_groups.get(name).expect(
                    "validate_process_groups proved process_group_order is a permutation of the \
                     group names",
                );
                (name, members)
            })
            .collect()
    };

    for (rank, (name, members)) in ordered.into_iter().enumerate() {
        // The set of global levels this group owns ≥1 node at.
        let mut owned_levels: HashSet<usize> = HashSet::new();
        for node_id in members {
            // A member with no level is an internal inconsistency (validation
            // proved it exists in `config.nodes`; the levelization places every
            // declared node — even a disconnected one lands at level 0). Fail
            // loudly rather than silently dropping it from the map.
            let lvl = levels
                .level_of(node_id)
                .ok_or_else(|| TransportError::GraphError {
                    reason: format!(
                        "process_groups group '{name}' member '{node_id}' has no level in the \
                     graph levelization (the config was not validated against these levels)"
                    ),
                })?;
            owned_levels.insert(lvl);
        }

        // Walk global levels in ascending order; assign a contiguous local
        // index per owned level, `None` elsewhere.
        let mut global_level_map: Vec<Option<usize>> = Vec::with_capacity(level_count);
        let mut next_local = 0usize;
        for g in 0..level_count {
            if owned_levels.contains(&g) {
                global_level_map.push(Some(next_local));
                next_local += 1;
            } else {
                global_level_map.push(None);
            }
        }

        groups.push(ProcessGroup {
            name: name.clone(),
            rank,
            global_level_map,
        });
    }

    Ok(groups)
}

// ===========================================================================
// Cost-aware auto-partitioner (pure).
// ===========================================================================

/// Measured/configured hop-cost constants driving the fusion score.
///
/// Both are PER-PLATFORM and caller-overridable (measured constants: intra-hop
/// ~1µs M3 / 2.1–3.1µs Jetson; cross-process boundary tax ~6.8µs Jetson). Stored
/// in ns as integers so the fusion score is byte-reproducible.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HopCosts {
    /// Intra-process hop cost (ns): a FUSED edge stays in one process.
    pub intra_ns: u64,
    /// Cross-process boundary tax (ns): a CUT edge crosses a process boundary.
    /// When `cross_ns <= intra_ns` fusion is never profitable (every coupling
    /// is 0), so the partitioner leaves the graph process-per-node.
    pub cross_ns: u64,
}

impl HopCosts {
    /// The per-platform DEFAULT hop costs, cfg-selected at
    /// compile time (mirrors the `DEFAULT_BARRIER_SPIN_US` per-OS precedent in
    /// `barrier.rs`).
    ///
    /// These are STARTING points, not the only channel: a deployment's
    /// profiler artifact file overrides them with values measured on that machine.
    /// They exist so the partitioner has a sane cross/intra spread on a fresh
    /// graph before any artifact is written.
    ///
    /// Values (measured):
    /// * `intra_ns` — the in-process hop: ~1000ns on Apple silicon (M3),
    ///   ~600ns on x86_64 Linux, ~2100ns on aarch64 Linux (Jetson).
    /// * `cross_ns` — the cross-process boundary tax: ~6800ns on Linux,
    ///   ~9000ns on macOS.
    ///
    /// Un-measured targets fall back to the Apple-silicon intra + the Linux
    /// cross (a conservative spread that still profits fusion), so
    /// `cross_ns > intra_ns` holds on every target.
    pub fn platform_default() -> Self {
        // intra-process hop (per arch/OS).
        #[cfg(target_os = "macos")]
        let intra_ns = 1_000;
        #[cfg(all(target_os = "linux", target_arch = "x86_64"))]
        let intra_ns = 600;
        #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
        let intra_ns = 2_100;
        #[cfg(not(any(
            target_os = "macos",
            all(target_os = "linux", target_arch = "x86_64"),
            all(target_os = "linux", target_arch = "aarch64"),
        )))]
        let intra_ns = 1_000; // conservative fallback for un-measured targets

        // cross-process boundary tax (per OS).
        #[cfg(target_os = "macos")]
        let cross_ns = 9_000;
        #[cfg(not(target_os = "macos"))]
        let cross_ns = 6_800;

        Self { intra_ns, cross_ns }
    }
}

/// The frozen cost snapshot [`auto_partition`] derives a grouping from.
///
/// Integer-only (ns + millihertz) so scoring AND ordering are deterministic —
/// no float non-determinism leaks into the grouping (Principle #7). The whole
/// snapshot is cloned into [`AutoPartition::costs`]: a grouping is only
/// meaningful paired with the costs it optimized against, and freezing it makes
/// the output self-describing + byte-reproducible.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PartitionCosts {
    /// Per-node p50 tick duration in ns (from `TraceEntry::duration_ns`),
    /// keyed by node id. EVERY graph node MUST have an entry — a missing node
    /// is a loud [`TransportError`] (never a silent default). The sum over a
    /// group's members is the group's compute load, gated by the budget.
    pub node_p50_ns: BTreeMap<String, u64>,
    /// Per trigger-edge fire rate in MILLIHERTZ (1 Hz = 1000 mHz), keyed by the
    /// `(producer_node, consumer_node)` pair. A pair with NO entry defaults to
    /// rate 0 (coupling 0 ⇒ never fused) — an unknown rate is treated as "do
    /// not fuse", the conservative choice. Millihertz keeps the score an exact
    /// integer.
    pub edge_rate_mhz: BTreeMap<(String, String), u64>,
    /// Hop-cost constants (the fusion score's `cross_ns − intra_ns` term).
    pub hop: HopCosts,
}

/// The output of [`harvest_costs`] — a frozen
/// [`PartitionCosts`] snapshot PLUS the set of under-sampled nodes that were
/// deliberately left WITHOUT a cost.
///
/// The two fields are the exact pair [`auto_partition`] consumes: pass
/// `costs` as its cost snapshot and `isolated` as its isolated set. Their
/// union covers every node the profiler observed — a node is either
/// well-sampled (a `costs.node_p50_ns` entry) or isolated (in `isolated`,
/// no cost), never both and never neither. Deterministic (BTree-ordered
/// throughout), so the same inputs yield a byte-identical `ProfileResult`
/// (Principle #7).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProfileResult {
    /// The frozen cost snapshot (per-node p50 + per-edge rate + hop costs).
    /// Contains a cost entry for exactly the WELL-SAMPLED nodes.
    pub costs: PartitionCosts,
    /// Under-sampled nodes (fire count below the node's PER-NODE target in
    /// `fires_targets` — or the node absent from `fires_targets` entirely — or
    /// well-sampled by fire count but with NO usable duration sample in the
    /// trace: no sample at all, or a ZERO lower-median, which is structurally
    /// indistinguishable from B-dur recording being off). They carry NO cost
    /// (fabricating one would violate Principle #13) and stay singleton process
    /// groups when fed to [`auto_partition`].
    pub isolated: BTreeSet<String>,
}

/// The policy that derives each node's PER-NODE fire target from a
/// short warm-up observation (see [`derive_fire_targets`]).
///
/// The auto-partition profiler runs a graph, counts each node's fires, and
/// isolates any node short of a fire target ([`harvest_costs`]). A single
/// SCALAR target blanket-isolates every node in a low-rate graph (a 1 Hz
/// control loop can never reach, say, 1000 fires within a 30 s cap). This
/// policy instead SCALES each node's target to its OWN observed rate: from a
/// warm-up fire count it projects how many fires the node would reach over a
/// projection horizon (the duration cap for the profiler's stop gate; the
/// actual observed window for its harvest gate — see [`derive_fire_targets`]),
/// keeps a fraction of that projection as the target (the `fraction_num /
/// fraction_den` rate-droop tolerance — a run may sample slower than warm-up),
/// then clamps into `[min_samples, max_samples]`.
///
/// Integer-only (see [`derive_fire_targets`]) so the derived targets are
/// byte-reproducible (Principle #7). The default (`min 20`, `max 1000`,
/// fraction `1/2`) is the shipped policy; the constants live here so tests pin
/// them explicitly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FireTargetPolicy {
    /// FLOOR a derived target clamps UP to: a node whose projected fire count
    /// is tiny still needs a minimum sample count for a meaningful p50.
    pub min_samples: u64,
    /// CEILING a derived target clamps DOWN to: a high-rate node needs no more
    /// than this many samples, and capping keeps the profiling window bounded.
    pub max_samples: u64,
    /// Numerator of the rate-droop tolerance fraction applied to the projected
    /// fire count (`fraction_num / fraction_den`).
    pub fraction_num: u64,
    /// Denominator of the rate-droop tolerance fraction. MUST be non-zero —
    /// enforced at (const-)evaluation time by [`FireTargetPolicy::new`];
    /// [`derive_fire_targets`] additionally treats a hand-constructed zero as
    /// no-derivation to avoid a div-by-zero.
    pub fraction_den: u64,
}

impl Default for FireTargetPolicy {
    /// The default policy: `min 20`, `max 1000`, fraction `1/2` (a ÷2
    /// rate-droop tolerance). Delegates to [`FireTargetPolicy::new`] so the
    /// non-zero-denominator invariant is honored through one path.
    fn default() -> Self {
        Self::new(20, 1000, 1, 2)
    }
}

impl FireTargetPolicy {
    /// Construct a policy, asserting the `fraction_den != 0` invariant at
    /// (const-)evaluation time.
    ///
    /// This is a `const fn`, so building a policy from const arguments makes a
    /// zero denominator a COMPILE-TIME error (a const-eval panic) rather than a
    /// runtime div-by-zero — the compile-time-prevention discipline. A runtime
    /// call with a zero denominator panics loudly on the same `assert!`.
    pub const fn new(
        min_samples: u64,
        max_samples: u64,
        fraction_num: u64,
        fraction_den: u64,
    ) -> Self {
        assert!(
            fraction_den != 0,
            "FireTargetPolicy::fraction_den must be non-zero (division by zero)"
        );
        Self {
            min_samples,
            max_samples,
            fraction_num,
            fraction_den,
        }
    }
}

/// ONE node's warm-up observation, measured over
/// the window that starts at **that node's FIRST FIRE**.
///
/// [`derive_fire_targets`] observes every node over ONE shared window running
/// from profile start. That window contains the graph BRING-UP: a `period_ms =
/// N` node owes a fire for every `N` ms of scheduler time since its last one,
/// the build (cdylib load + SHM service creation) is elapsed time in which it
/// cannot fire, and the debt is replayed as a CATCH-UP BURST on the first live
/// step. Counting that burst as rate makes the projected fire target
/// unreachable, and the node is then ISOLATED for firing at exactly its
/// declared period (measured on main CI run 31364973342 as a 58.45 Hz
/// "warm-up rate" for a 20 Hz node).
///
/// The burst lands entirely in the node's FIRST observed counter movement (the
/// scheduler's catch-up loop replays the whole debt inside one `step`), so
/// anchoring the observation at the first fire — and counting only what came
/// AFTER it — measures the node's SUSTAINED rate and nothing else.
///
/// This is a profile-mode MEASUREMENT change only: no scheduler semantics move,
/// so there is no Principle #7 interaction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WarmupObservation {
    /// Fires observed strictly AFTER the node's first sighting — i.e. with the
    /// bring-up burst (and the first fire it arrived with) excluded.
    ///
    /// `0` is a legitimate observation, NOT an absence: it means "this node
    /// fired, and produced no further fire inside `window_ns`", which is what a
    /// node slower than the warm-up window looks like. See
    /// [`derive_fire_targets_from_observations`] for how it is projected.
    pub fires: u64,
    /// The node's own ACTIVE window (ns): from its first observed fire to the
    /// warm-up end. `0` when the first fire was seen at the warm-up boundary
    /// itself — no rate is derivable from it, but the node still fired.
    pub window_ns: u64,
}

/// One greedy fusion that was APPLIED, in application order — an audit
/// trail making the grouping's derivation inspectable and pinning the
/// greedy-descending order in tests.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FusionRecord {
    /// The producer node of the trigger edge whose coupling drove the fusion.
    pub producer: String,
    /// The consumer node of that trigger edge.
    pub consumer: String,
    /// `coupling = rate_mhz × (cross_ns − intra_ns)` — the score it ranked by.
    pub coupling: u128,
}

/// Why a candidate fusion was NOT applied — makes budget starvation,
/// bridge collapse, and unprofitability distinguishable from each other AND
/// from "no candidates at all" (an empty `rejections` + empty `fusions`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FusionRejectionReason {
    /// `coupling == 0` — a zero fire rate, or `cross_ns <= intra_ns` (fusing
    /// saves nothing).
    Unprofitable,
    /// The merged group's compute load would exceed the per-group budget.
    OverBudget {
        /// Σ of the merged members' p50 durations (saturating).
        load: u64,
        /// The `budget_ns` the load was gated against.
        budget: u64,
    },
    /// The merged group fails the re-levelization bijection check — a foreign
    /// node bridges its dependency chain (see the module "re-levelization
    /// crux").
    Bridged {
        /// The foreign trigger-predecessor named by the check (see the
        /// internal `find_bridge`'s first-foreign-predecessor semantics —
        /// a deterministic diagnostic HINT, not a proven-causal bridge).
        bridge: String,
    },
    /// The merged group would own NON-ADJACENT global DAG levels. A
    /// DIRECT in-group edge spanning the gap keeps the re-levelization
    /// bijection intact (owned `{0,4}` with a `0→4` edge collapses to local
    /// `{0,1}`), so this is NOT caught by [`FusionRejectionReason::Bridged`];
    /// the barrier's contiguous-split participant-map still cannot represent a
    /// gapped band. The candidate edge that would have created the gap is on
    /// the enclosing [`FusionRejection`]'s `producer`/`consumer`.
    NonContiguous {
        /// The sorted, deduped global levels the merged group would own —
        /// non-contiguous (a gap exists between `owned.first()` and
        /// `owned.last()`).
        owned: Vec<usize>,
    },
}

/// One candidate fusion that was REJECTED, in consideration
/// (descending-coupling) order — the audit-trail twin of [`FusionRecord`].
/// `graph levels` does not print these; the partition emitter consumes them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FusionRejection {
    /// The producer node of the rejected candidate edge.
    pub producer: String,
    /// The consumer node of the rejected candidate edge.
    pub consumer: String,
    /// The candidate's coupling score at consideration time.
    pub coupling: u128,
    /// Why the fusion was not applied.
    pub reason: FusionRejectionReason,
}

/// The result of [`auto_partition`] — the derived grouping plus the
/// frozen cost snapshot it was derived from.
///
/// [`groups`](Self::groups) is directly consumable as
/// [`GraphConfig::process_groups`] and by [`derive_process_groups`]. Same
/// `(config, entry_infos, trigger_edges, costs, budget)` ⇒ byte-identical
/// `AutoPartition` (Principle #7).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AutoPartition {
    /// group name → member node ids (in graph order). Group order = pipeline
    /// order (each group's lead node's global level, then graph order), which
    /// becomes the cross-process rank order when fed to
    /// [`derive_process_groups`]. Names are lead-node-derived (`grp_<node>`,
    /// sanitized + uniquified).
    pub groups: IndexMap<String, Vec<String>>,
    /// The cost snapshot this grouping was derived from (a clone of the input).
    pub costs: PartitionCosts,
    /// The fusions applied, in greedy (descending-coupling) order.
    pub fusions: Vec<FusionRecord>,
    /// The candidate fusions NOT applied, in consideration order (an
    /// already-same-group candidate is neither a fusion nor a rejection — it
    /// asked for nothing).
    pub rejections: Vec<FusionRejection>,
}

// ===========================================================================
// `block` edges are a HARD CO-LOCATION CONSTRAINT on the partition.
// ===========================================================================

/// The co-location workaround, spelled once for the WHOLE repo.
///
/// This wording was minted for the split-pair advisory
/// (`cerulion_cli_engine::multiprocess`); the plan-time refusal offers
/// the same gesture, so it reads the same const rather than a second copy that
/// can drift. It lives HERE because `cerulion_cli_engine` depends on
/// `cerulion_core` and not the other way round.
pub const REMEDY_CO_LOCATE: &str = "co-locating both nodes in ONE `process_groups:` group";
/// The monolith workaround, spelled once for the WHOLE repo (see
/// [`REMEDY_CO_LOCATE`]).
pub const REMEDY_SINGLE_PROCESS: &str = "running `--single-process`";

/// One topic whose flow carries a `block` consumer AND an in-graph
/// producer — i.e. one co-location constraint the partition must satisfy.
///
/// Every listed node is unioned into ONE process group by the DERIVATION — a
/// preference it does not decline, because crossing a process boundary costs a
/// real hop whether or not the edge is correct.
///
/// It is not an absolute law of the design, and since the credit exemption it is not
/// one of the plan-time refusal either. A co-located edge's defer mirror is a
/// process-local heap word, which a producer in another OS process cannot
/// observe; the CROSS-PROCESS form — one `MappedCredit` SHM page both address
/// spaces operate on, carried by `crate::credit::CreditWord` — makes a SPLIT
/// edge lossless, and the supervisor mints one for every CREDITABLE topic
/// (exactly one in-graph producer, no non-`block` consumers:
/// [`crate::graph::topology::TopicFlow::credit_bar`]). A hand-written `process_groups:` splitting such
/// an edge is therefore ACCEPTED; splitting any other `block` topic's flow is
/// still refused pre-spawn, naming the bar it hit.
///
/// The constraint covers the WHOLE flow — producers AND every consumer, `block`
/// or not. See [`block_colocation_seeds`] for why the non-`block` siblings of a
/// MIXED topic are not optional.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BlockColocationSeed {
    /// The resolved topic name.
    pub topic: String,
    /// Every in-graph producer node id (graph order). More than one only on a
    /// `multi_publisher_topics` topic — where EVERY producer receives the same
    /// `BlockDeferEdge`, so the constraint is genuinely the whole flow.
    pub producers: Vec<String>,
    /// Every `(node, input)` that declared `block` on this topic (graph order).
    pub block_consumers: Vec<(String, String)>,
    /// Every OTHER `(node, input)` consuming this topic (graph order) — the
    /// non-`block` siblings that make the topic MIXED. Empty on an all-`block`
    /// topic, and the reason a mixed topic's group is the whole flow: see
    /// [`block_colocation_seeds`].
    pub other_consumers: Vec<(String, String)>,
    /// Whether this topic's `block` edges can carry a
    /// cross-process credit word, and if not, why.
    ///
    /// PRIVATE and stamped from [`crate::graph::topology::TopicFlow::credit_bar`] at construction —
    /// the ONE body of the rule, which the mint calls too. Re-deriving it here
    /// from the public fields would be a second implementation free to drift
    /// from the mint's; a caller that wants the answer asks
    /// [`BlockColocationSeed::credit_bar`].
    credit_bar: Option<CreditBar>,
}

impl BlockColocationSeed {
    /// Why this topic's `block` edges cannot be credited —
    /// `None` means they can. Stamped at construction from
    /// [`crate::graph::topology::TopicFlow::credit_bar`]; see that function for why the rule has ONE
    /// body.
    pub fn credit_bar(&self) -> Option<&CreditBar> {
        self.credit_bar.as_ref()
    }

    /// Every node this seed constrains — producers, then block consumers, then
    /// the non-block siblings, in graph order, with duplicates possible (a node
    /// may both produce and consume; the union-find absorbs the repeat).
    fn nodes(&self) -> impl Iterator<Item = &str> {
        self.producers
            .iter()
            .map(String::as_str)
            .chain(self.block_consumers.iter().map(|(n, _)| n.as_str()))
            .chain(self.other_consumers.iter().map(|(n, _)| n.as_str()))
    }

    /// `producer -> consumer.input` renderings, for a diagnostic.
    fn edge_labels(&self) -> Vec<String> {
        let mut out = Vec::new();
        for p in &self.producers {
            for (c, i) in &self.block_consumers {
                out.push(format!("{p} -> {c}.{i}"));
            }
        }
        out
    }
}

/// Every co-location constraint this graph's topology carries.
///
/// # The predicate is `has_block_consumer()`, NOT `is_all_block()`
///
/// The gate that refuses is PER CONSUMER (`GraphTopology::validate`:
/// `matches!(c.policy, Block) && flow.producers.is_empty()`) and knows nothing
/// about mixedness. `is_all_block()` is consulted only LATER, at the runtime's
/// defer-INSTALL site, which degrades a mixed topic's `block` consumers to
/// `drop_oldest` with a warn — so on a split mixed topic the degrade can never
/// rescue anything, because the build is already dead.
///
/// It is worse than symmetric: in the consumer's worker subgraph the SIBLING
/// `drop_oldest` consumer is a foreign node and is dropped, so LOCALLY the flow
/// reads as all-block with no producer. Global mixedness is structurally
/// invisible to the check that refuses. Seeding on `is_all_block()` would
/// therefore leave a legal, documented, separately-tested shape (one `block` +
/// one `drop_oldest` consumer on one topic) falling straight through into the
/// exact defect this constraint exists to close.
///
/// # The seed is the WHOLE flow — every consumer, not just the `block` ones
///
/// Seeding producers + `block` consumers ALONE is not merely a smaller group,
/// it is a DIFFERENT RUN. That same worker-local invisibility cuts the other
/// way: co-locate `{producer, block consumer}` and leave the `drop_oldest`
/// sibling in its own group, and the co-located worker's `TopicFlow` reads
/// consumers = `[block]` ⇒ `is_all_block()` TRUE ⇒ the runtime INSTALLS the
/// defer mirror (`runtime.rs`, the `!flow.is_all_block()` gate), while the
/// monolith and `--single-process` see the mixed flow and DEGRADE `block` to
/// `drop_oldest` with a warn. The degrade warn then fires in NO process (the
/// co-located worker is locally all-block; the sibling's worker sees no `block`
/// consumer at all), so the divergence is silent, and the producer is throttled
/// to the `block` consumer's drain rate — starving the sibling, which is
/// exactly the starvation `TopicFlow::is_all_block`'s decision K exists to
/// prevent. `--record` then captures a different stream per mode from one
/// graph.
///
/// So the seed takes EVERY consumer of a `block`-carrying flow. That makes the
/// worker's LOCAL mixedness equal the graph's GLOBAL mixedness, so the worker
/// degrades exactly like the monolith. The cost is a marginally larger group on
/// a topic whose `block` is degraded anyway; the alternative was not a smaller
/// group, it was two modes of one graph disagreeing in silence.
///
/// A flow with NO in-graph producer is skipped: there is nothing to co-locate
/// WITH, and such a graph is refused at build with its own specific message,
/// identically under `--single-process`. Manufacturing a plan-time refusal for
/// it would carry a partition remedy for a partition that is not the problem.
///
/// Pure; deterministic (topology order in, graph order within a flow).
pub fn block_colocation_seeds(topo: &GraphTopology) -> Vec<BlockColocationSeed> {
    let mut seeds = Vec::new();
    for flow in topo.topics() {
        if flow.producers.is_empty() || !flow.has_block_consumer() {
            continue;
        }
        let (block_consumers, other_consumers) = flow
            .consumers
            .iter()
            .map(|c| {
                (
                    matches!(c.policy, BackpressurePolicy::Block),
                    (c.node_id.clone(), c.input.clone()),
                )
            })
            .fold(
                (Vec::new(), Vec::new()),
                |(mut block, mut other), (is_block, edge)| {
                    if is_block {
                        block.push(edge);
                    } else {
                        other.push(edge);
                    }
                    (block, other)
                },
            );
        seeds.push(BlockColocationSeed {
            topic: flow.topic.clone(),
            producers: flow.producers.clone(),
            block_consumers,
            other_consumers,
            // The SAME call the mint makes, on the SAME value — so the
            // validator cannot answer differently from `credit_edges_for`.
            credit_bar: flow.credit_bar(),
        });
    }
    seeds
}

/// Union every seed's nodes, then REPAIR the result to a
/// spawner-consumable partition by absorption. Returns the anchor index of
/// each seeded group (deduplicated by root, ascending) for the caller's budget
/// accounting.
///
/// # The window is load-bearing, and it is stated as an invariant
///
/// This runs IMMEDIATELY after `UnionFind::new`, BEFORE any greedy fusion. Two
/// consequences the rest of the design leans on:
///
/// 1. the greedy loop's `rp == rc { continue }` arm then treats a
///    block-co-located pair as *already fused, asks nothing* — no spurious
///    `Unprofitable` rejection record — and its budget fold (which folds
///    `node_cost` over the CURRENT merged set) sees the real member set;
/// 2. every non-seed group is still a singleton, so absorbing a foreign node
///    cannot leave a half-emptied donor behind.
///
/// Point 2 is belt-and-braces rather than the actual guarantee: union-find can
/// only MERGE classes, never split one, so `uf.union(anchor, foreign)` pulls
/// the foreign node's WHOLE class in and the donor class disappears entirely.
/// A donor can never be left non-contiguous by losing a member, in this window
/// or any other.
///
/// # Repair, not refusal
///
/// A seeded union bypasses `check_group` by construction, so it can produce a
/// `Bridged` or `NonContiguous` group — which `validate_partition` would then
/// refuse, turning today's confusing worker death into a plan-time refusal of a
/// graph that runs fine under `--single-process`. So the group GROWS instead:
/// absorb the named bridge (or the nodes occupying the gap in a non-contiguous
/// band) and re-check. Every absorption is announced. In the pathological limit
/// the group reaches the whole graph — exactly the `--single-process` shape the
/// operator is forced into today, but derived and announced rather than
/// discovered from a dead worker.
///
/// # Termination
///
/// Every repair round performs at least one `union`, and a union strictly
/// reduces the number of equivalence classes, so at most `n - 1` can ever
/// happen. The outer loop is bounded at `n + 1` passes and returns a loud `Err`
/// naming the offending block edges if it somehow does not converge — a
/// defensive arm, never a silent proceed.
///
/// Deterministic: seeds in topology order, nodes within a seed in graph order,
/// absorption candidates scanned ascending `0..n`.
#[allow(clippy::too_many_arguments)]
fn seed_and_repair_block_colocation(
    uf: &mut UnionFind,
    seeds: &[BlockColocationSeed],
    node_ids: &[&str],
    id_to_idx: &HashMap<&str, usize>,
    config: &GraphConfig,
    entry_infos: &IndexMap<String, NodeInfo>,
    trigger_edges: &TriggerEdges,
    topo: &GraphTopology,
    global_levels: &Levels,
    isolated: &BTreeSet<String>,
) -> TransportResult<Vec<usize>> {
    if seeds.is_empty() {
        return Ok(Vec::new());
    }
    let n = node_ids.len();
    let mut anchors: Vec<usize> = Vec::new();
    for seed in seeds {
        let members: Vec<usize> = seed
            .nodes()
            .filter_map(|id| id_to_idx.get(id).copied())
            .collect();
        let Some(&first) = members.first() else {
            continue;
        };
        // Co-location OVERRIDES isolation, loudly. `graph profile`
        // isolates a node that missed its fire target, and a `block` consumer
        // is BY CONSTRUCTION the slow node the producer is being deferred for
        // — the single most likely member of that set. Isolation is a cost
        // POLICY ("keep it a singleton, we have no measurement for it");
        // co-location is a CORRECTNESS constraint, so it wins. It is never
        // silent, because the alternative is a run that cannot build.
        let overridden: Vec<&str> = members
            .iter()
            .map(|&i| node_ids[i])
            .filter(|id| isolated.contains(*id))
            .collect();
        if !overridden.is_empty() {
            // WARN, not INFO: this OVERRIDES a policy the profiler recorded, on
            // verbs (`graph partition`, `graph levels`) whose default filter is
            // `cerulion=warn` — see the budget-override site for the full
            // argument. "It is never silent" has to be true at the shipped
            // default, not only under an explicit `RUST_LOG`.
            tracing::warn!(
                graph = %config.identity(),
                topic = %seed.topic,
                isolated_members = %overridden.join(", "),
                "block co-location OVERRIDES isolation for this topic — an \
                 under-sampled node named in the profile's isolated set is a `block` \
                 endpoint, and `block` is a correctness constraint rather than a cost \
                 preference, so it is grouped with the rest of the flow instead of staying \
                 a singleton. It carries NO measured cost, so it contributes NOTHING to \
                 this group's reported budget load"
            );
        }
        tracing::info!(
            graph = %config.identity(),
            topic = %seed.topic,
            producers = %seed.producers.join(", "),
            block_consumers = %seed
                .block_consumers
                .iter()
                .map(|(c, i)| format!("{c}.{i}"))
                .collect::<Vec<_>>()
                .join(", "),
            mixed_siblings = %if seed.other_consumers.is_empty() {
                "none".to_string()
            } else {
                seed.other_consumers
                    .iter()
                    .map(|(c, i)| format!("{c}.{i}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            },
            "co-locating this topic's producer(s) and `block` consumer(s) in ONE \
             process group. `block` defers the producer's tick through a PROCESS-LOCAL \
             mirror, and crossing a process boundary costs a real hop, so the derivation \
             keeps the flow together. A hand-written split is not automatically refused: \
             where the topic is CREDITABLE (one in-graph producer, no non-`block` \
             consumers) the supervisor mints it a cross-process credit word and it runs \
             losslessly. Any `mixed_siblings` (non-`block` consumers of the same topic) \
             join the group too: a worker that cannot SEE them reads its flow as \
             all-`block` and installs the defer where the monolith degrades it"
        );
        for &m in &members[1..] {
            uf.union(first, m);
        }
        anchors.push(first);
    }
    // Deduplicate anchors by their CURRENT root (two seeds sharing a node are
    // ONE group) — ascending, so the repair walk is deterministic.
    //
    // By ROOT, not by index: a transitive chain (A block-> B on one topic,
    // B block-> C on another) yields anchors idx(A) and idx(B), two DIFFERENT
    // indices resolving to ONE union-find root, so an index dedup keeps both.
    // Grouping is unaffected (union-find only merges), but every downstream
    // walk over `anchors` runs twice for that group — including
    // `auto_partition`'s over-budget reporting loop, which then emits the
    // identical override line TWICE for one group, against the loud-ONCE
    // intent. Unions are all done by here, so `find` is stable for the rest of
    // this function; the repair below only ever MERGES classes, so two anchors
    // that share a root now can never stop sharing one.
    // `(root, index)` so the SURVIVING anchor of a shared root is deterministic
    // too (`sort_unstable` reorders equal keys freely, and this vector is
    // returned to the caller).
    anchors.sort_unstable_by_key(|&a| (uf.find(a), a));
    anchors.dedup_by_key(|a| uf.find(*a));

    // ---- repair by absorption -------------------------------------------
    let mut pass = 0usize;
    loop {
        pass += 1;
        if pass > n + 1 {
            return Err(TransportError::GraphError {
                reason: format!(
                    "auto_partition: could not derive a spawner-consumable partition that \
                     co-locates the graph's `block` edges ({}) — the repair loop did not \
                     converge in {} passes. Run with `--single-process`, or declare a \
                     `process_groups:` block by hand that puts each `block` topic's \
                     producer(s) and `block` consumer(s) in one group",
                    seeds
                        .iter()
                        .flat_map(|s| s.edge_labels())
                        .collect::<Vec<_>>()
                        .join(", "),
                    n + 1
                ),
            });
        }
        let mut changed = false;
        for &anchor in &anchors {
            let root = uf.find(anchor);
            let members: Vec<usize> = (0..n).filter(|&i| uf.find(i) == root).collect();
            let member_ids: Vec<&str> = members.iter().map(|&i| node_ids[i]).collect();
            let member_set: HashSet<usize> = members.iter().copied().collect();
            match check_group(
                &member_ids,
                config,
                entry_infos,
                trigger_edges,
                topo,
                global_levels,
            )? {
                GroupCheck::Valid => {}
                GroupCheck::Bridged { bridge, diagnostic } => {
                    // Absorb the named foreign bridge. `union` pulls its whole
                    // class in, so the donor never survives half-drained.
                    let absorbed = bridge
                        .as_deref()
                        .and_then(|b| id_to_idx.get(b).copied())
                        .filter(|i| !member_set.contains(i));
                    let Some(idx) = absorbed else {
                        return Err(TransportError::GraphError {
                            reason: format!(
                                "auto_partition: a `block` co-location group is not \
                                 spawner-consumable and the repair could not name a node to \
                                 absorb: {diagnostic}. Run with `--single-process`, or \
                                 declare a `process_groups:` block by hand"
                            ),
                        });
                    };
                    // WARN: a node the operator never asked to group lands in
                    // a group this verb WRITES to their file. Same reason as
                    // the budget override — `graph partition` defaults to
                    // `cerulion=warn`, so an `info!` here is invisible on the
                    // one surface where it matters.
                    tracing::warn!(
                        graph = %config.identity(),
                        absorbed = %node_ids[idx],
                        group_members = %member_ids.join(", "),
                        "absorbing a foreign bridge node into a `block` \
                         co-location group so the group re-levelizes as a \
                         contiguous-split band the spawner can consume. The group is \
                         larger than the cost model asked for; that is the price of a \
                         `block` edge the partition may not split"
                    );
                    uf.union(anchor, idx);
                    changed = true;
                }
                GroupCheck::NonContiguous { owned, diagnostic } => {
                    // Absorb every node occupying a global level INSIDE the
                    // group's owned band but owned by somebody else.
                    let (lo, hi) = (owned[0], owned[owned.len() - 1]);
                    let gap: HashSet<usize> = (lo..=hi).filter(|g| !owned.contains(g)).collect();
                    let mut absorbed_any = false;
                    for (i, id) in node_ids.iter().enumerate() {
                        if member_set.contains(&i) {
                            continue;
                        }
                        let Some(g) = global_levels.level_of(id) else {
                            continue;
                        };
                        if !gap.contains(&g) {
                            continue;
                        }
                        // WARN, for the same reason as the bridge absorption
                        // above: a node the operator never grouped is written
                        // into a group by this verb.
                        tracing::warn!(
                            graph = %config.identity(),
                            absorbed = %id,
                            level = g,
                            group_members = %member_ids.join(", "),
                            "absorbing a node that occupies a gap inside a \
                             `block` co-location group's owned level band — the \
                             cross-process barrier's participant map is a \
                             CONTIGUOUS-split index map and cannot represent a gapped \
                             band"
                        );
                        uf.union(anchor, i);
                        absorbed_any = true;
                    }
                    if !absorbed_any {
                        return Err(TransportError::GraphError {
                            reason: format!(
                                "auto_partition: a `block` co-location group owns a \
                                 non-contiguous level band and the repair found no node to \
                                 absorb: {diagnostic}. Run with `--single-process`, or \
                                 declare a `process_groups:` block by hand"
                            ),
                        });
                    }
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
    Ok(anchors)
}

/// Derive process groups from a graph's dataflow + a cost
/// snapshot via **process-per-node baseline + validated greedy fusion**.
///
/// Starts every node in its own group (max fault isolation), then fuses the
/// most tightly-coupled trigger edges first (`coupling = rate × (cross_ns −
/// intra_ns)`, greedy descending). A fusion is COMMITTED only when the merged
/// group both (a) fits the per-group compute budget (`Σ member p50 ≤
/// budget_ns`) and (b) passes the per-group re-levelization bijection check
/// (see [`validate_partition`] and the module "re-levelization crux") — so the
/// emitted mapping is spawner-consumable by construction. When no fusion is
/// profitable (`cross_ns ≤ intra_ns`, or every rate is 0) the graph is left
/// process-per-node.
///
/// Pure: no I/O, no clock, no randomness. Reuses the REAL
/// [`GraphTopology::build`] + [`GraphTopology::derive_levels`] for both the
/// global and per-group induced-subgraph levelizations.
///
/// # Errors
///
/// * a graph node missing from `costs.node_p50_ns`;
/// * the graph's trigger DAG contains an algebraic cycle (surfaced from
///   [`GraphTopology::derive_levels`]);
/// * `config`/`entry_infos` fail [`GraphTopology::build`].
///
/// # Warnings (stray cost-snapshot entries)
///
/// A MISSING node cost is a loud error (above); an EXTRA one is only a
/// `tracing::warn!`. Any `node_p50_ns` key not naming a graph node, or any
/// `edge_rate_mhz` key not naming an actual producer→consumer trigger edge,
/// emits one structured warn per stray key and is otherwise ignored. It is a
/// warn rather than an error on purpose: a snapshot may legitimately be a
/// SUPERSET re-used across graph edits (a node/edge deleted from the graph but
/// still in the frozen snapshot). But a stray key is ALSO the fingerprint of a
/// typo — a mistyped edge key means the INTENDED edge silently keeps rate 0
/// (never fused), and a mistyped node key means the intended node keeps no
/// cost — so every unmatched key is surfaced loudly instead of vanishing.
pub fn auto_partition(
    config: &GraphConfig,
    entry_infos: &IndexMap<String, NodeInfo>,
    trigger_edges: &TriggerEdges,
    costs: &PartitionCosts,
    isolated: &BTreeSet<String>,
    budget_ns: u64,
) -> TransportResult<AutoPartition> {
    // `isolated` names the online profiler's UNDER-SAMPLED
    // nodes. An isolated node (a) SKIPS the missing-cost error below (it carries
    // NO fabricated cost — Principle #13), (b) is never a fusion candidate
    // (excluded from `pairs`/`candidates`), so it stays a singleton group. Every
    // isolated name must be a REAL graph node — a typo would silently exempt the
    // WRONG node from the cost requirement, so reject it loudly (kills typos).
    let node_id_set: HashSet<&str> = config.native_nodes().map(|n| n.id.as_str()).collect();
    for iso in isolated {
        if !node_id_set.contains(iso.as_str()) {
            return Err(TransportError::GraphError {
                reason: format!(
                    "auto_partition: isolated node '{iso}' is not a node in this graph; the \
                     isolated set must name only real graph nodes (a typo would silently \
                     exempt the wrong node from the per-node cost requirement)"
                ),
            });
        }
    }

    // Every NON-isolated node needs a cost — loud, not silent (Principle #13 /
    // no silent inference). Checked before any structural work so the message is
    // direct. An isolated node holds a placeholder 0 in `node_cost` that is
    // NEVER read (isolated nodes are never fusion candidates, so their cost is
    // never summed into a group's budget load).
    let mut node_cost: Vec<u64> = Vec::with_capacity(config.nodes.len());
    for node in config.native_nodes() {
        if isolated.contains(node.id.as_str()) {
            node_cost.push(0);
            continue;
        }
        match costs.node_p50_ns.get(&node.id) {
            Some(&c) => node_cost.push(c),
            None => {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "auto_partition: node '{}' has no p50 duration in the cost snapshot; \
                         every graph node must carry a measured/estimated per-node duration \
                         (or be listed in the isolated set)",
                        node.id
                    ),
                })
            }
        }
    }

    let (topo, global_levels) = build_global(config, entry_infos, trigger_edges)?;

    // node id ↔ graph-order index (the deterministic tiebreak key throughout).
    let node_ids: Vec<&str> = config.native_nodes().map(|n| n.id.as_str()).collect();
    let id_to_idx: HashMap<&str, usize> = node_ids
        .iter()
        .enumerate()
        .map(|(i, id)| (*id, i))
        .collect();
    let n = node_ids.len();

    // Candidate fusion edges: unique (producer, consumer) node pairs joined by
    // a TRIGGER edge (self-edges excluded — a node can't fuse with itself). The
    // cost model keys rate per node pair, so per-topic multiplicity collapses
    // to one candidate.
    let mut pairs: IndexSet<(usize, usize)> = IndexSet::new();
    for flow in topo.topics() {
        for c in &flow.consumers {
            if !trigger_edges.is_triggering(&c.node_id, &flow.topic) {
                continue;
            }
            // An isolated (under-sampled) consumer is never a
            // fusion candidate — it stays a singleton group.
            if isolated.contains(c.node_id.as_str()) {
                continue;
            }
            let Some(&ci) = id_to_idx.get(c.node_id.as_str()) else {
                continue;
            };
            for p in &flow.producers {
                // Likewise an isolated producer contributes no candidate edge.
                if isolated.contains(p.as_str()) {
                    continue;
                }
                if let Some(&pi) = id_to_idx.get(p.as_str()) {
                    if pi != ci {
                        pairs.insert((pi, ci));
                    }
                }
            }
        }
    }

    // Warn (never error) on stray cost-snapshot entries. A
    // MISSING node cost is a loud error (checked above); an EXTRA key is only a
    // warn — the snapshot may legitimately be a SUPERSET re-used across graph
    // edits. But a stray key is also the fingerprint of a TYPO (a mistyped edge
    // key means the intended edge silently keeps rate 0), so surface every
    // unmatched key loudly instead of letting it vanish.
    for key in costs.node_p50_ns.keys() {
        if !node_id_set.contains(key.as_str()) {
            tracing::warn!(
                key = %key,
                "auto_partition: stray cost-snapshot entry — node_p50_ns names a node not in \
                 this graph (ignored; a typo means the intended node silently keeps no cost — \
                 harmless if the snapshot is a superset re-used across graph edits)"
            );
        }
    }
    // The only edge keys scoring ever consults are the candidate trigger pairs;
    // any other `edge_rate_mhz` key is never read → stray.
    let consulted_edges: HashSet<(&str, &str)> = pairs
        .iter()
        .map(|&(pi, ci)| (node_ids[pi], node_ids[ci]))
        .collect();
    for (producer, consumer) in costs.edge_rate_mhz.keys() {
        if !consulted_edges.contains(&(producer.as_str(), consumer.as_str())) {
            tracing::warn!(
                producer = %producer,
                consumer = %consumer,
                "auto_partition: stray cost-snapshot entry — edge_rate_mhz names a pair that is \
                 not a producer→consumer trigger edge in this graph (ignored; a typo means the \
                 intended edge silently keeps rate 0 — harmless if the snapshot is a superset \
                 re-used across graph edits)"
            );
        }
    }

    // Score each pair. saving = latency reclaimed per crossing avoided.
    let saving = u128::from(costs.hop.cross_ns.saturating_sub(costs.hop.intra_ns));
    let mut candidates: Vec<(u128, usize, usize)> = pairs
        .iter()
        .map(|&(pi, ci)| {
            let rate = u128::from(edge_rate(costs, node_ids[pi], node_ids[ci]));
            (rate.saturating_mul(saving), pi, ci)
        })
        .collect();
    // Greedy DESCENDING coupling; ties broken by (producer, consumer) graph
    // order so the fusion sequence is fully deterministic.
    candidates.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));

    // Process-per-node baseline; fuse in-place with union-find keyed by index.
    let mut uf = UnionFind::new(n);

    // SEED the hard `block` co-location constraints BEFORE the greedy
    // loop, and repair the result to a spawner-consumable shape. Ordering is
    // load-bearing (see `seed_and_repair_block_colocation`): the loop's
    // already-same-group `continue` then treats a co-located pair as asking for
    // nothing, and its budget fold sees the real member set.
    let seeds = block_colocation_seeds(&topo);
    let seed_anchors = seed_and_repair_block_colocation(
        &mut uf,
        &seeds,
        &node_ids,
        &id_to_idx,
        config,
        entry_infos,
        trigger_edges,
        &topo,
        &global_levels,
        isolated,
    )?;
    // A seed BYPASSES the budget gate by construction — the gate
    // guards CANDIDATE fusions inside the loop, and co-location is not a
    // candidate. That is the intended semantic (a `block` edge the partition
    // may not split outranks a latency budget), but it must not be silent.
    //
    // The reported load is a FLOOR, not a total: an isolated member carries a
    // placeholder 0 that is not a measurement (Principle #13 — nothing here
    // fabricates a cost for it), and co-location can pull an isolated node in.
    // The line says so rather than presenting the sum as complete.
    //
    // WARN, not INFO: `graph partition` (the only surface where an operator can
    // set `--budget-ns` at all — the run preflight passes `budget_ns: None`) is
    // classified OneShot, whose default filter is `cerulion=warn`, so an `info!`
    // here does not print on a default invocation and the "never silently"
    // claim would be false on the one verb it is about. This is a POLICY
    // OVERRIDE of a value the operator supplied, which is the repo's `warn`
    // row ("recoverable issues") and matches the sibling
    // `warn_order_block_removed` on this same verb.
    for &anchor in &seed_anchors {
        let root = uf.find(anchor);
        let members: Vec<usize> = (0..n).filter(|&i| uf.find(i) == root).collect();
        let load: u64 = members
            .iter()
            .fold(0u64, |acc, &i| acc.saturating_add(node_cost[i]));
        if load <= budget_ns {
            continue;
        }
        let unmeasured: Vec<&str> = members
            .iter()
            .map(|&i| node_ids[i])
            .filter(|id| isolated.contains(*id))
            .collect();
        tracing::warn!(
            graph = %config.identity(),
            group_members = %members
                .iter()
                .map(|&i| node_ids[i])
                .collect::<Vec<_>>()
                .join(", "),
            load_ns = load,
            budget_ns = budget_ns,
            unmeasured_members = %if unmeasured.is_empty() {
                "none".to_string()
            } else {
                unmeasured.join(", ")
            },
            block_topics = %seeds
                .iter()
                .map(|s| s.topic.as_str())
                .collect::<Vec<_>>()
                .join(", "),
            "a `block` co-location group exceeds the per-group compute budget. \
             Co-location WINS — `block` is a correctness constraint and a split one cannot \
             build — so the group is kept and the budget is overridden. `load_ns` is a \
             FLOOR: any member listed in `unmeasured_members` was isolated by the profiler \
             and carries no measured cost, so it contributes nothing to this sum"
        );
    }

    let mut fusions: Vec<FusionRecord> = Vec::new();
    let mut rejections: Vec<FusionRejection> = Vec::new();
    let reject = |rejections: &mut Vec<FusionRejection>,
                  pi: usize,
                  ci: usize,
                  coupling: u128,
                  reason: FusionRejectionReason| {
        rejections.push(FusionRejection {
            producer: node_ids[pi].to_string(),
            consumer: node_ids[ci].to_string(),
            coupling,
            reason,
        });
    };

    for &(coupling, pi, ci) in &candidates {
        let rp = uf.find(pi);
        let rc = uf.find(ci);
        if rp == rc {
            // Already in the same group: the candidate asks for nothing —
            // neither a fusion nor a rejection. This precedes the coupling==0
            // gate so a zero-coupling candidate whose endpoints were already
            // transitively fused (via other rated edges) is NOT mis-recorded as
            // an Unprofitable rejection.
            continue;
        }
        // Only profitable fusions (coupling 0 covers cross ≤ intra AND rate 0).
        if coupling == 0 {
            reject(
                &mut rejections,
                pi,
                ci,
                coupling,
                FusionRejectionReason::Unprofitable,
            );
            continue;
        }
        // Candidate merged member set = both current groups, in graph order.
        let merged: Vec<usize> = (0..n)
            .filter(|&i| uf.find(i) == rp || uf.find(i) == rc)
            .collect();
        // (a) budget gate. Saturating fold — same overflow discipline as the
        // coupling arithmetic (a pathological snapshot must clamp, not wrap).
        let load: u64 = merged
            .iter()
            .fold(0u64, |acc, &i| acc.saturating_add(node_cost[i]));
        if load > budget_ns {
            reject(
                &mut rejections,
                pi,
                ci,
                coupling,
                FusionRejectionReason::OverBudget {
                    load,
                    budget: budget_ns,
                },
            );
            continue;
        }
        // (b) re-levelization gate (the REAL spawner constraint).
        let member_ids: Vec<&str> = merged.iter().map(|&i| node_ids[i]).collect();
        match check_group(
            &member_ids,
            config,
            entry_infos,
            trigger_edges,
            &topo,
            &global_levels,
        )? {
            GroupCheck::Valid => {
                uf.union(pi, ci);
                fusions.push(FusionRecord {
                    producer: node_ids[pi].to_string(),
                    consumer: node_ids[ci].to_string(),
                    coupling,
                });
            }
            // A fusion that would collapse the group's re-levelization is not
            // taken — the two nodes stay in separate processes, and the
            // rejection records the named bridge.
            GroupCheck::Bridged { bridge, .. } => {
                reject(
                    &mut rejections,
                    pi,
                    ci,
                    coupling,
                    FusionRejectionReason::Bridged {
                        // A mismatch always has a foreign trigger-predecessor
                        // (an all-in-group ancestry would re-levelize to the
                        // owned rank); the fallback keeps the record total
                        // rather than panicking on the unreachable arm.
                        bridge: bridge.unwrap_or_else(|| "<unresolved>".to_string()),
                    },
                );
            }
            // A fusion that would give the merged group a NON-CONTIGUOUS
            // owned band (a direct in-group edge spanning the gap keeps the
            // bijection intact) is not taken — the candidate edge (`pi`→`ci`) is
            // on the recorded `FusionRejection`, so the audit trail names it.
            GroupCheck::NonContiguous { owned, .. } => {
                reject(
                    &mut rejections,
                    pi,
                    ci,
                    coupling,
                    FusionRejectionReason::NonContiguous { owned },
                );
            }
        }
    }

    let groups = assemble_groups(&mut uf, &node_ids, &global_levels);

    Ok(AutoPartition {
        groups,
        costs: costs.clone(),
        fusions,
        rejections,
    })
}

/// The process-per-node BASELINE grouping — every node in its
/// own group, EXCEPT where the `block` co-location constraints say
/// otherwise — named and ordered by the SAME internal `assemble_groups` logic
/// [`auto_partition`] uses, so the no-costs emit path and the fused path share
/// ONE naming/ordering source of truth.
///
/// # It is not unconditionally process-per-node
///
/// A `block` topic's whole flow shares one group here exactly as it does on the
/// fused path — this IS the default path (`cerulion graph run` on an
/// unpartitioned graph with no cost snapshot), so before the co-location repair every `block`
/// edge in every unprofiled graph was split and its consumer's worker died at
/// build. There is no cost model here, so no budget can be overridden; the
/// seed and repair diagnostics are the ones `seed_and_repair_block_colocation`
/// emits (private — see its own docs), and the repair can absorb
/// further nodes (up to, in the limit, the whole graph). A graph with no
/// `block` input is untouched and really is one group per node.
///
/// This is exactly the state [`auto_partition`] starts from before any fusion
/// (and the state it returns when no fusion is profitable), factored out so the
/// emitter can produce it WITHOUT a cost snapshot. The no-costs path must NOT
/// synthesize a degenerate cost snapshot to force this shape out of
/// [`auto_partition`] — fabricating per-node durations would violate Principle
/// #13; deriving the baseline directly keeps the "no measured costs ⇒ maximal
/// fault isolation" fallback free of invented costs.
///
/// Returns `group name → member node ids` (a single-member `grp_<node>` per
/// node, except for the seeded `block` groups above), in pipeline order (each
/// group's lead node's global level, then graph order) — directly consumable as
/// [`GraphConfig::process_groups`]. Same `(config, entry_infos, trigger_edges)`
/// ⇒ byte-identical map (Principle #7).
///
/// Pure; reuses the REAL `build_global` levelizer only for the group ORDER.
///
/// # Errors
///
/// The structural errors of [`auto_partition`]'s build phase — a graph that
/// fails [`GraphTopology::build`], or a trigger DAG with an algebraic cycle
/// (surfaced from [`GraphTopology::derive_levels`]) — plus, since the co-location repair landed,
/// `seed_and_repair_block_colocation`'s three loud arms: a repair loop that
/// did not converge, and either absorption finding no node to absorb.
pub fn baseline_process_per_node(
    config: &GraphConfig,
    entry_infos: &IndexMap<String, NodeInfo>,
    trigger_edges: &TriggerEdges,
) -> TransportResult<IndexMap<String, Vec<String>>> {
    let (topo, global_levels) = build_global(config, entry_infos, trigger_edges)?;
    let node_ids: Vec<&str> = config.native_nodes().map(|n| n.id.as_str()).collect();
    let id_to_idx: HashMap<&str, usize> = node_ids
        .iter()
        .enumerate()
        .map(|(i, id)| (*id, i))
        .collect();
    // A fresh union-find with NO unions leaves every node in its own group; the
    // shared `assemble_groups` then applies the identical `grp_<lead>` naming +
    // pipeline ordering the fused path uses.
    let mut uf = UnionFind::new(node_ids.len());
    // Except for the hard `block` co-location constraints — see this
    // function's own doc for why the baseline honours them too.
    let seeds = block_colocation_seeds(&topo);
    seed_and_repair_block_colocation(
        &mut uf,
        &seeds,
        &node_ids,
        &id_to_idx,
        config,
        entry_infos,
        trigger_edges,
        &topo,
        &global_levels,
        &BTreeSet::new(),
    )?;
    Ok(assemble_groups(&mut uf, &node_ids, &global_levels))
}

/// The pure online-profiler harness — turn a run's observed
/// trace + cap-immune fire counts into the [`ProfileResult`] ([`auto_partition`]'s
/// `(costs, isolated)` input).
///
/// Mirrors [`auto_partition`]'s signature style (`config` + `entry_infos` +
/// `trigger_edges`) so the two compose directly: the same three graph args feed
/// both, and this function's `ProfileResult` splits straight into
/// `auto_partition`'s `costs` + `isolated` params.
///
/// * **Per-node p50** — for each WELL-SAMPLED node (fire count `>=` the node's
///   PER-NODE target in `fires_targets`), the integer LOWER median of that
///   node's `duration_ns` samples in `trace` (see `lower_median`). A node's p50
///   is an OBSERVED sample, never an average — no fabricated intermediate value.
/// * **Isolation** — a node whose fire count is BELOW its per-node target in
///   `fires_targets`, OR that has NO entry in `fires_targets` (an unknown
///   target is treated as unmet — the conservative choice), is ISOLATED: it
///   gets NO cost entry and
///   goes into [`ProfileResult::isolated`] (fabricating a cost from too few
///   samples would violate Principle #13). A node that IS well-sampled by fire
///   count but has NO usable duration sample in the trace is ALSO isolated for
///   the same reason — there is no measured p50 to emit, and leaving it un-costed
///   but non-isolated would make `auto_partition` reject the snapshot. "No
///   usable sample" covers both NO sample at all (e.g. every sample
///   ring-evicted) AND a ZERO lower-median: `TraceEntry::duration_ns` is hard-0
///   when B-dur recording is off while fires still push trace entries, so a
///   zero median is structurally indistinguishable from recording-off — costing
///   it 0 would fabricate a "free" node that always fuses. (A genuinely sub-ns
///   tick is impossible on real hardware; it would be isolated conservatively.)
/// * **Per-edge rate** — for each `(producer, consumer)` TRIGGER edge where
///   NEITHER endpoint is isolated, `rate_mHz = producer_fires × 10^12 /
///   window_ns` (`rate_mhz`). A producer publishes one frame per fire, so
///   every consumer of a topic inherits the producer's rate. Self-edges and
///   edges touching an isolated node are omitted (they would never be consulted
///   by `auto_partition`, which excludes isolated nodes from its candidates).
///
/// `window_ns == 0` (a zero-length / absent observation window) is handled
/// without a div-by-zero: every edge rate defaults to 0 = "unknown rate, do not
/// fuse" (see `rate_mhz`). p50 and isolation are unaffected (they do not
/// depend on the window).
///
/// Deterministic (BTree-ordered maps/sets, insertion-ordered topic iteration,
/// a stable-sorted median) → the same inputs yield a byte-identical
/// `ProfileResult` (Principle #7). Pure: no I/O, no clock, no randomness.
///
/// # Errors
///
/// Surfaces a [`TransportError`] only from the internal
/// [`GraphTopology::build`] (a malformed graph) — the profiling maths
/// themselves are total.
// The three graph args mirror `auto_partition`; the trace + fire counts are the
// observed run data; window_ns / hop / fires_targets are the profiler knobs.
// Each is genuinely needed and flat is clearer than a bundle struct (same
// judgement as `NodeHandle::new`).
#[allow(clippy::too_many_arguments)]
pub fn harvest_costs(
    config: &GraphConfig,
    entry_infos: &IndexMap<String, NodeInfo>,
    trigger_edges: &TriggerEdges,
    trace: &[TraceEntry],
    fire_counts: &BTreeMap<String, u64>,
    window_ns: u64,
    hop: HopCosts,
    fires_targets: &BTreeMap<String, u64>,
) -> TransportResult<ProfileResult> {
    // (1) Split the observed nodes by the sampling-adequacy gate. A node absent
    // from `fire_counts` is unknown to the profiler and is NOT enumerated here;
    // the caller must feed every node's cap-immune counter so the union of
    // `isolated` + `costs.node_p50_ns` covers the whole graph (else
    // `auto_partition` will reject the un-costed node).
    let mut isolated: BTreeSet<String> = BTreeSet::new();
    let mut well_sampled: BTreeSet<String> = BTreeSet::new();
    for (node, &count) in fire_counts {
        // Per-node gate: a node is under-sampled if it is ABSENT from
        // `fires_targets` (unknown target ⇒ treat as unmet, the conservative
        // choice) OR its fire count is strictly below its own target. The
        // boundary stays `<` — a count EQUAL to the target is KEPT.
        let under_sampled = match fires_targets.get(node) {
            Some(&target) => count < target,
            None => true,
        };
        if under_sampled {
            isolated.insert(node.clone());
        } else {
            well_sampled.insert(node.clone());
        }
    }

    // (2) Per-node p50 from the trace's duration_ns samples, for the
    // well-sampled nodes only. Gather each well-sampled node's durations in one
    // pass over the trace.
    let mut durations: BTreeMap<&str, Vec<u64>> = BTreeMap::new();
    for entry in trace {
        let id = entry.node_id.as_ref();
        if well_sampled.contains(id) {
            durations.entry(id).or_default().push(entry.duration_ns);
        }
    }
    let mut node_p50_ns: BTreeMap<String, u64> = BTreeMap::new();
    for node in &well_sampled {
        match durations.get(node.as_str()) {
            Some(samples) if !samples.is_empty() => {
                // A usable p50 must be NONZERO: `TraceEntry::duration_ns` is
                // hard-0 whenever B-dur recording is OFF (a fire ALWAYS pushes
                // a trace entry regardless), so a ZERO lower-median is
                // structurally indistinguishable from a recording-off trace.
                // Costing it 0 would fabricate a "free" node that always fuses
                // (Principle #13). A genuinely sub-ns tick is impossible on
                // real hardware; if one ever produced a zero median it is
                // isolated CONSERVATIVELY (no cost, singleton group) rather
                // than costed 0.
                let p50 = lower_median(samples);
                if p50 > 0 {
                    node_p50_ns.insert(node.clone(), p50);
                } else {
                    isolated.insert(node.clone());
                }
            }
            _ => {
                // Well-sampled by fire count but NO duration sample at all
                // (e.g. ring-evicted): there is no measured p50 to emit
                // (fabricating one would violate Principle #13), so DEMOTE to
                // isolated. This keeps the harness → auto_partition contract
                // intact (every node is either costed or isolated, never
                // un-costed-and-non-isolated).
                isolated.insert(node.clone());
            }
        }
    }

    // (3) Per-edge fire rate from producer fire counts, over the trigger edges.
    // Reuses the REAL topology (the same builder auto_partition uses).
    let topo = GraphTopology::build(config, entry_infos)?;
    let mut edge_rate_mhz: BTreeMap<(String, String), u64> = BTreeMap::new();
    for flow in topo.topics() {
        for c in &flow.consumers {
            if !trigger_edges.is_triggering(&c.node_id, &flow.topic) {
                continue;
            }
            // Isolated endpoints are never fusion candidates, so a rate there
            // would be a stray key downstream — omit it.
            if isolated.contains(&c.node_id) {
                continue;
            }
            for p in &flow.producers {
                if p == &c.node_id {
                    continue; // self-edge (a node can't fuse with itself)
                }
                if isolated.contains(p) {
                    continue;
                }
                // A producer publishes 1 frame per fire, so every consumer of the
                // topic inherits the producer's fire rate.
                let fires = fire_counts.get(p).copied().unwrap_or(0);
                edge_rate_mhz.insert((p.clone(), c.node_id.clone()), rate_mhz(fires, window_ns));
            }
        }
    }

    Ok(ProfileResult {
        costs: PartitionCosts {
            node_p50_ns,
            edge_rate_mhz,
            hop,
        },
        isolated,
    })
}

/// Derive each node's PER-NODE fire target from a SHARED warm-up
/// window.
///
/// **This is no longer the profiler's warm-up
/// path.** A window shared by every node starts at profile start, which counts
/// the graph bring-up's catch-up burst as rate — see [`WarmupObservation`] for
/// the defect and [`derive_fire_targets_from_observations`] for the per-node
/// form the profiler's warm-up snapshot now uses. This shared-window form
/// survives for the profiler's NO-WARM-UP FALLBACK (a run that ended before
/// warm-up completed), where `warmup_ns == horizon_ns` makes the target
/// `clamp(fires × fraction, min, max)` — at most half the node's OWN observed
/// fires under the default policy, so that path is structurally immune to the
/// burst it also contains. It delegates the projection arithmetic to the
/// per-node form, so there is exactly ONE implementation of the clamp.
///
/// For each node in `warmup_fires` with a NON-ZERO warm-up fire count, projects
/// the fires it would reach over the `horizon_ns` window and keeps a fraction
/// of that projection, clamped into the policy's `[min_samples, max_samples]`:
///
/// ```text
/// total_expected = warmup_fires * horizon_ns / warmup_ns      (u128, saturating)
/// target         = clamp(total_expected * fraction_num / fraction_den, min, max)
/// ```
///
/// **Two horizons.** The profiler calls this with two different
/// horizons for two different questions:
/// * `horizon_ns = the duration cap` — the watcher's STOP GATE ("has a FULL
///   run collected enough?");
/// * `horizon_ns = the ACTUAL observed window` — the harvest's isolation gate
///   ("did this node sample as much as THIS window could have delivered?").
///   A run stopped early (Ctrl-C) must not judge nodes against a cap-length
///   projection they were never given time to reach. Window-horizon targets
///   are `<=` cap-horizon ones (window `<=` cap), so a stop-gate-met run
///   passes the harvest gate a fortiori.
///
/// Integer-only (u128 intermediates, saturating multiply, then a lossless cast
/// back to `u64` — the same overflow discipline as `rate_mhz`), so the derived
/// targets are byte-reproducible (Principle #7). Deterministic (BTree-ordered)
/// and pure (no I/O, clock, or randomness).
///
/// Contracts:
/// * A node whose warm-up fire count is `0` gets NO entry (silent contract): it
///   has no rate to project from, so it is left out — [`harvest_costs`] then
///   ISOLATES it (a node absent from the target map is under-sampled).
/// * `warmup_ns == 0` (a zero-length warm-up) carries no rate information, so
///   NO target is derivable and the returned map is EMPTY. The engine never
///   passes `0`; this only guards against a div-by-zero.
/// * `warmup_ns == horizon_ns` (the no-warm-up fallback: the whole observed
///   window IS the observation) degenerates to `target = clamp(fires ×
///   fraction, min, max)` — e.g. `clamp(fires/2, 20, 1000)` at the default
///   policy.
/// * A degenerate `policy.fraction_den == 0` (reachable only by bypassing
///   [`FireTargetPolicy::new`] with a struct literal) is likewise treated as
///   no-derivation (empty map) to avoid a div-by-zero.
pub fn derive_fire_targets(
    warmup_fires: &BTreeMap<String, u64>,
    warmup_ns: u64,
    horizon_ns: u64,
    policy: &FireTargetPolicy,
) -> BTreeMap<String, u64> {
    // A zero warm-up window carries no derivable target — return an empty map
    // rather than divide by zero. (The zero-denominator guard lives in the
    // shared implementation below.)
    if warmup_ns == 0 {
        return BTreeMap::new();
    }
    // Every node shares ONE window, and a zero fire count means the node never
    // fired at all — the silent contract — so it is filtered out here rather
    // than handed to the per-node form, whose MEMBERSHIP means "this node
    // fired" (see [`derive_fire_targets_from_observations`]).
    let observations: BTreeMap<String, WarmupObservation> = warmup_fires
        .iter()
        .filter(|(_, &fires)| fires != 0)
        .map(|(node, &fires)| {
            (
                node.clone(),
                WarmupObservation {
                    fires,
                    window_ns: warmup_ns,
                },
            )
        })
        .collect();
    derive_fire_targets_from_observations(&observations, horizon_ns, policy)
}

/// Derive each node's PER-NODE fire target from
/// its OWN [`WarmupObservation`] — the window that starts at that node's FIRST
/// FIRE, so the graph bring-up's catch-up burst is not counted as rate.
///
/// The projection arithmetic is [`derive_fire_targets`]'s, unchanged — this is
/// where it lives, and the shared-window form delegates here:
///
/// ```text
/// total_expected = obs.fires * horizon_ns / obs.window_ns   (u128, saturating)
/// target         = clamp(total_expected * fraction_num / fraction_den, min, max)
/// ```
///
/// **MEMBERSHIP is the evidence the node fired.** A node absent from
/// `observations` was SILENT through the warm-up: it has no rate to project
/// from, so it gets NO target and [`harvest_costs`] isolates it with the
/// distinct "silent through warm-up" marker. That contract is byte-unchanged
/// from the shared-window form; only the SHAPE of the observation moved.
///
/// **A present node with `fires == 0` (or a zero `window_ns`) is NOT silent.**
/// It fired — that is why it is in the map — and merely produced no further
/// fire inside its own active window, which is exactly what a node slower than
/// the warm-up looks like (a 1 Hz node's first fire can land 200 ms into a 1 s
/// warm-up, leaving 800 ms with nothing further). Dropping it would re-label a
/// firing node "silent through warm-up" and isolate it, which is a REGRESSION
/// against the low-rate contract. Instead it projects ZERO and the
/// policy FLOOR carries it to `min_samples` — the same target the earlier
/// shared-window path produced for such a node, where `clamp(1 fire scaled,
/// min, max)` also floored.
///
/// Same integer-only discipline (u128 intermediates, saturating multiply,
/// lossless cast back), so targets stay byte-reproducible (Principle #7);
/// deterministic (BTree-ordered) and pure. A degenerate
/// `policy.fraction_den == 0` yields an EMPTY map rather than a div-by-zero.
pub fn derive_fire_targets_from_observations(
    observations: &BTreeMap<String, WarmupObservation>,
    horizon_ns: u64,
    policy: &FireTargetPolicy,
) -> BTreeMap<String, u64> {
    let mut targets: BTreeMap<String, u64> = BTreeMap::new();
    // A degenerate zero denominator carries no derivable target — return an
    // empty map rather than divide by zero.
    if policy.fraction_den == 0 {
        return targets;
    }
    let min = u128::from(policy.min_samples);
    let max = u128::from(policy.max_samples);
    for (node, obs) in observations {
        // No positive rate is measurable — but the node DID fire (membership
        // is that evidence), so it must NOT be re-labelled silent. Project
        // ZERO and let the policy floor carry it.
        let scaled = if obs.fires == 0 || obs.window_ns == 0 {
            0u128
        } else {
            // Project the fire count over the horizon from the node's own
            // ACTIVE rate, then keep the droop-tolerated fraction. u128
            // throughout; saturating multiply guards a pathological fire count
            // (mirrors `rate_mhz`).
            let total_expected = u128::from(obs.fires).saturating_mul(u128::from(horizon_ns))
                / u128::from(obs.window_ns);
            total_expected.saturating_mul(u128::from(policy.fraction_num))
                / u128::from(policy.fraction_den)
        };
        // Manual clamp (avoids `u128::clamp`'s panic if a malformed policy has
        // min > max): floor UP to min, then cap DOWN to max.
        let clamped = scaled.max(min).min(max);
        // `clamped <= max = u128::from(max_samples)` always ⇒ the cast is
        // lossless; `unwrap_or` mirrors `rate_mhz`'s saturating discipline.
        targets.insert(node.clone(), u64::try_from(clamped).unwrap_or(u64::MAX));
    }
    targets
}

/// The DEFAULT per-group compute budget derived from the profiling
/// machine's core count — `ceil(total_compute_ns / cores)`, floor 1.
///
/// Rationale: [`auto_partition`]'s budget caps each fused group's summed p50
/// (`Σ member p50 ≤ budget_ns`). With an UNBOUNDED default, dense graphs fuse
/// hot chains into a near-monolith with no per-core load balancing; dividing
/// the graph's TOTAL costed compute across the cores it can actually schedule
/// on makes the default budget "one core's fair share" — fusion never packs
/// more compute into one process than an even spread across the permitted
/// cores would allow.
///
/// **Freeze contract:** computed ONCE at PROFILE time from the PROFILING
/// machine's permitted core count and frozen into the cost-snapshot artifact
/// (`derived_budget_ns`). Readers (the partition verb, the `graph run`
/// preflight) consume the frozen value; they never re-derive it from the
/// READING machine's cores — the budget is a property of the measurement, and a
/// deployment machine with different cores would otherwise silently reshape a
/// reviewed partition.
///
/// Integer-only and deterministic: ceil-division computed in u128 (no
/// overflow for any `u64` total), result `<= total` so the cast back is
/// lossless. `total_compute_ns == 0` (nothing costed) yields the `max(1)`
/// floor — a 0 budget is rejected downstream (`--budget-ns` forbids 0), and
/// callers avoid FREEZING that degenerate case at all (an all-isolated
/// profile writes no budget).
pub fn derive_default_budget_ns(total_compute_ns: u64, cores: NonZeroUsize) -> u64 {
    // usize -> u128 is lossless on every target; NonZero guarantees >= 1.
    let cores = cores.get() as u128;
    let ceil = u128::from(total_compute_ns).div_ceil(cores);
    // ceil(total / cores) <= total for total >= 1 (and 0 -> 0 -> max(1)),
    // so the u64 cast is lossless; `unwrap_or` mirrors the file's
    // saturating discipline.
    u64::try_from(ceil).unwrap_or(u64::MAX).max(1)
}

/// The integer LOWER median of `samples` — the per-node p50
/// used as the fusion cost.
///
/// Sorts a copy ascending and returns the element at index `(n - 1) / 2`:
/// * ODD `n` → the exact middle sample;
/// * EVEN `n` → the LOWER of the two central samples (index `(n-1)/2`), NOT
///   their average. Returning an OBSERVED sample keeps the p50 an exact integer
///   with no rounding and no fabricated intermediate value (Principle #13). For
///   example `[10, 20, 30, 40]` → `20` (index 1), not `25`.
///
/// Callers guarantee `!samples.is_empty()` (an empty slice would index out of
/// bounds); [`harvest_costs`] only calls this for a node with ≥1 sample.
fn lower_median(samples: &[u64]) -> u64 {
    debug_assert!(
        !samples.is_empty(),
        "lower_median requires a non-empty slice"
    );
    let mut sorted = samples.to_vec();
    sorted.sort_unstable();
    sorted[(sorted.len() - 1) / 2]
}

/// An edge's fire rate in MILLIHERTZ from the producer's fire
/// count over the profiling window.
///
/// `rate_mHz = fires × 10^12 / window_ns` (ns→s is ÷10^9, Hz→mHz is ×10^3, so
/// the combined factor is 10^12). Computed in `u128` so a high fire count
/// cannot overflow, then saturating-cast to `u64` — a pathological value clamps
/// to `u64::MAX` rather than wrapping (the same overflow discipline as the
/// coupling arithmetic in [`auto_partition`]).
///
/// `window_ns == 0` (a zero-length / absent observation window) carries NO rate
/// information, so the rate is 0 = "unknown rate, do not fuse" — the
/// conservative choice (matching [`PartitionCosts::edge_rate_mhz`]'s no-entry
/// default) that also guards against an integer div-by-zero.
fn rate_mhz(fires: u64, window_ns: u64) -> u64 {
    if window_ns == 0 {
        return 0;
    }
    let raw = u128::from(fires).saturating_mul(1_000_000_000_000u128) / u128::from(window_ns);
    u64::try_from(raw).unwrap_or(u64::MAX)
}

/// Validate that EVERY group in `groups` is
/// spawner-consumable — its induced-subgraph re-levelization is a
/// contiguous-split bijection onto the global levels it owns.
///
/// This is the REAL constraint (`GraphRuntime::install_barrier_participant`):
/// a group process re-levelizes its OWN subgraph, and that local level count
/// MUST equal the number of global levels the group owns, IN ORDER. A group
/// whose contiguous global band is bridged by a FOREIGN node collapses — the
/// error NAMES THE BRIDGE node. Runs [`validate_process_groups`]' structural
/// checks (orphans / dangling refs / double-assignment / empty groups) first.
///
/// [`auto_partition`] uses the same per-group check internally, so a
/// partition it emits always passes this validator by construction; the
/// validator additionally serves caller-supplied (hand-written / emitted)
/// partitions.
pub fn validate_partition(
    groups: &IndexMap<String, Vec<String>>,
    config: &GraphConfig,
    entry_infos: &IndexMap<String, NodeInfo>,
    trigger_edges: &TriggerEdges,
) -> TransportResult<()> {
    // Structural validation via the existing checker — build a config view
    // carrying these groups (the passed `config` may have none of its own).
    let mut structural = config.clone();
    structural.process_groups = groups.clone();
    structural.process_group_order = Vec::new();
    validate_process_groups(&structural)?;

    let (topo, global_levels) = build_global(config, entry_infos, trigger_edges)?;

    for (name, members) in groups {
        let member_ids: Vec<&str> = members.iter().map(|s| s.as_str()).collect();
        match check_group(
            &member_ids,
            config,
            entry_infos,
            trigger_edges,
            &topo,
            &global_levels,
        )? {
            GroupCheck::Valid => {}
            GroupCheck::Bridged { diagnostic, .. } => {
                return Err(TransportError::GraphError {
                    reason: format!("process group '{name}': {diagnostic}"),
                });
            }
            // Same one-voice wording as the runtime's
            // `install_barrier_participant` + `validate_contiguous_ownership`
            // plan-time guard, plus the offending owned-level set.
            GroupCheck::NonContiguous { diagnostic, .. } => {
                return Err(TransportError::GraphError {
                    reason: format!("process group '{name}': {diagnostic}"),
                });
            }
        }
    }

    // Last, so a partition that is BOTH structurally broken and
    // block-splitting still reports the structural fault first (every existing
    // bridge/contiguity verdict is byte-unchanged).
    validate_block_colocation(groups, &topo).verdict?;
    Ok(())
}

/// Refuse a partition that splits a `block`-carrying topic's flow —
/// its producer(s), its `block` consumer(s), AND (on a MIXED topic) its
/// non-`block` siblings — across process groups, EXCEPT where the credit exemption
/// gives the split edge a cross-process credit word.
///
/// # Why a REFUSAL, and why there is no third option
///
/// `block` is a producer-side pre-fire defer driven by a mirror the CONSUMER's
/// subscriber decrements. On a co-located edge that mirror is a process-local
/// heap word, and a producer in another OS process holds a different one (or
/// none), so it cannot observe the consumer's queue through it.
///
/// That is a statement about the WORD, and the credit exemption changed which words
/// exist: a split edge can be backed by a `MappedCredit` SHM page both address
/// spaces operate on (`crate::credit::CreditWord`), which the supervisor mints
/// and the runtime wires from the plan. Such an edge is lossless across the
/// boundary, so refusing it would refuse a shape that runs — and the exemption
/// is [`crate::graph::topology::TopicFlow::credit_bar`] — the ONE body of the rule, stamped onto the
/// seed at construction and called by `credit_edges_for` too, so the two
/// halves cannot drift.
///
/// For every OTHER shape the refusal stands, and the third option stays the
/// wrong one: the only available "degrade" would be silently rewriting `block`
/// to `drop_oldest`, which is a DATA-LOSS semantic change the operator never
/// asked for (Principle #6, and the loud-over-silent-inference rule). So the
/// partition is refused, at plan time, before any worker is spawned.
///
/// That is strictly better than what happens without it: `subgraph_for` drops
/// the foreign producer from the consumer's subgraph, the consumer's worker
/// sees a `Block` consumer on a producer-less topic, and it dies at
/// `GraphTopology::validate` with a message blaming an external publisher the
/// operator does not have — reported by the supervisor as an opaque
/// "exited before signaling READY".
///
/// # The predicate is the same one the DERIVATION seeds on
///
/// `has_block_consumer() && !producers.is_empty()`, over the WHOLE flow — see
/// [`block_colocation_seeds`] for why it is not `is_all_block()` and why the
/// non-`block` siblings are not optional. The two halves MUST agree: a refusal
/// predicate NARROWER than the seed's would refuse partitions the derivation
/// itself produces, and a WIDER one would let a shape the derivation declines
/// to co-locate die in the worker.
///
/// # The two violations are DIFFERENT faults and say so
///
/// A split producer↔`block`-consumer edge KILLS the consumer's worker at build
/// (a `Block` consumer on a locally producer-less topic). A split MIXED sibling
/// does the opposite: every worker builds, and the one holding the producer and
/// the `block` consumer reads its local flow as all-`block`, INSTALLS the defer
/// mirror, and throttles the producer to the `block` consumer's drain rate —
/// starving the sibling — while `--single-process` degrades to `drop_oldest`
/// with a warn. The degrade warn fires in NO process, so one graph runs two
/// semantics in silence. Same remedies, different cause: reporting the
/// build-death text for a shape that builds fine would send the operator
/// looking for a crash that never happens.
///
/// They also take OPPOSITE remedies. Either endpoint of a split `block` edge
/// may move, but a split MIXED sibling has its producer and `block` consumer
/// already together — so moving the PRODUCER to the sibling's group splits that
/// edge and is refused in turn. Only the sibling may move.
///
/// And a `BlockEdge` violation is the HEADLINE wherever it was found: the
/// per-seed ordering below is followed by a global stable partition, because a
/// mixed sibling on an earlier topic would otherwise headline a refusal whose
/// diagnosis says every worker builds while a later split `block` edge kills
/// one.
///
/// A flow with NO in-graph producer is SKIPPED. Such a graph is already refused
/// at build with its own specific message, identically under
/// `--single-process`, so there is nothing here for a partition remedy to fix
/// and a plan-time double would only mislead.
///
/// # The seed is UNCHANGED by the credit exemption
///
/// [`block_colocation_seeds`] still seeds every `block`-carrying flow, so the
/// DERIVED partition keeps co-locating whole flows: crossing a process boundary
/// costs a real hop whether or not a credit word makes it correct, and the
/// derivation optimises for latency. The exemption only stops REFUSING a
/// hand-written partition that chose the boundary deliberately — accepting more
/// than the derivation produces, which is the safe direction (the reverse would
/// refuse partitions the derivation itself writes).
///
/// Reached from BOTH partition paths: `graph run`'s pre-spawn gate and the
/// read-only `graph levels` verdict both go through [`validate_partition`], and
/// by then a DERIVED partition and a hand-written one are indistinguishable.
fn validate_block_colocation(
    groups: &IndexMap<String, Vec<String>>,
    topo: &GraphTopology,
) -> BlockColocationOutcome {
    let mut credited: Vec<CreditableSplitEdge> = Vec::new();
    // node id → owning group name.
    let mut owner: HashMap<&str, &str> = HashMap::new();
    for (name, members) in groups {
        for m in members {
            owner.insert(m.as_str(), name.as_str());
        }
    }
    let mut violations: Vec<BlockColocationViolation> = Vec::new();
    for seed in block_colocation_seeds(topo) {
        // BLOCK edges before siblings WITHIN a seed. This ordering alone is not
        // enough — see the global stable partition below.
        let block_pairs = seed
            .block_consumers
            .iter()
            .map(|edge| (edge, BlockColocationKind::BlockEdge));
        let sibling_pairs = seed
            .other_consumers
            .iter()
            .map(|edge| (edge, BlockColocationKind::MixedSibling));
        for ((consumer, input), kind) in block_pairs.chain(sibling_pairs) {
            for producer in &seed.producers {
                if producer == consumer {
                    continue;
                }
                let (Some(&pg), Some(&cg)) =
                    (owner.get(producer.as_str()), owner.get(consumer.as_str()))
                else {
                    // An unplaced node is a STRUCTURAL fault the checks above
                    // already own; inventing a co-location verdict for it would
                    // re-label somebody else's error.
                    continue;
                };
                if pg == cg {
                    continue;
                }
                // Credit exemption (C5): a SPLIT `block` edge is buildable when the
                // supervisor mints it a cross-process credit word — and only
                // then. Asked AFTER the `pg == cg` arm, so this stands for the
                // mint's rank-inequality term as well.
                // Credit exemption: a SPLIT `block` edge is buildable when the
                // supervisor mints it a cross-process credit word — and only
                // then. Asked AFTER the `pg == cg` arm, so this stands for the
                // mint's rank-inequality term as well. The rule itself lives
                // on `TopicFlow` (`credit_bar`), stamped onto the seed at
                // construction, so the validator and the mint cannot drift.
                let vkind = match kind {
                    BlockColocationKind::BlockEdge => match seed.credit_bar() {
                        None => {
                            // ACCEPTED. Recorded in the walk that decided it,
                            // so the operator-facing line and the verdict
                            // behind it are one value, not two derivations.
                            credited.push(CreditableSplitEdge {
                                topic: seed.topic.clone(),
                                consumer_node: consumer.clone(),
                                consumer_input: input.clone(),
                                producer_group: pg.to_string(),
                                consumer_group: cg.to_string(),
                            });
                            continue;
                        }
                        Some(bar) => BlockColocationViolationKind::BlockEdge { bar: bar.clone() },
                    },
                    BlockColocationKind::MixedSibling => BlockColocationViolationKind::MixedSibling,
                };
                let line = match kind {
                    BlockColocationKind::BlockEdge => format!(
                        "topic '{}': producer '{producer}' (group '{pg}') is split from \
                         `block` input '{consumer}.{input}' (group '{cg}')",
                        seed.topic
                    ),
                    BlockColocationKind::MixedSibling => format!(
                        "topic '{}': producer '{producer}' (group '{pg}') is split from its \
                         non-`block` consumer '{consumer}.{input}' (group '{cg}') on a topic \
                         that ALSO carries `block` input(s) {}",
                        seed.topic,
                        seed.block_consumers
                            .iter()
                            .map(|(c, i)| format!("'{c}.{i}'"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                };
                violations.push(BlockColocationViolation {
                    kind: vkind,
                    line,
                    topic: seed.topic.clone(),
                    producer: producer.clone(),
                    consumer: consumer.clone(),
                    input: input.clone(),
                    producer_group: pg.to_string(),
                    consumer_group: cg.to_string(),
                    block_inputs: seed
                        .block_consumers
                        .iter()
                        .map(|(c, i)| format!("{c}.{i}"))
                        .collect(),
                });
            }
        }
    }
    if violations.is_empty() {
        // Sorted + deduped so the rendered order is stable regardless of
        // topology order. (A creditable topic has exactly one producer, so the
        // per-(edge, producer) walk already yields one entry per edge; the
        // dedup is belt-and-braces against a future widening.)
        credited.sort();
        credited.dedup();
        return BlockColocationOutcome {
            credited,
            verdict: Ok(()),
        };
    }
    // A REFUSAL DOES NOT UNMAKE THE EDGES IT DID NOT REFUSE. The walk is
    // per-seed, so one partition can hold a creditable split AND, on a
    // different topic, a violation. While this returned `Result<Vec, _>` the
    // credited vec was destroyed by the `Err`, and `creditable_split_block_edges`
    // — whose whole job is to REPORT those edges — turned that into
    // `unwrap_or_default()`, i.e. "no creditable edges here". `partition_emit`
    // believed it and overwrote a legal hand-written split with no
    // "REPLACES that choice" warning at all, which is the silent loss the
    // comment at its own call site says to name. So the verdict and the
    // credited set are now returned TOGETHER and the caller chooses.
    credited.sort();
    credited.dedup();
    // A `BlockEdge` violation is the headline WHEREVER it was found, not merely
    // wherever it beat a sibling inside its own seed. Ordering block-first per
    // seed is not enough: a MIXED sibling on an EARLIER topic would otherwise
    // claim the headline, and its diagnosis says every worker builds — while
    // the later split `block` edge kills one. Report the arm an operator can
    // act on. A STABLE partition, so all violations survive and graph order is
    // preserved within each class; the FIRST record is therefore always the
    // headline's own line.
    let (mut ordered, siblings): (Vec<_>, Vec<_>) = violations
        .into_iter()
        .partition(|v| matches!(v.kind, BlockColocationViolationKind::BlockEdge { .. }));
    ordered.extend(siblings);
    let more = if ordered.len() > 1 {
        format!(
            "\n  (and {} more split `block`-topic edge(s):\n  - {})",
            ordered.len() - 1,
            ordered[1..]
                .iter()
                .map(|v| v.rendered_with_bar())
                .collect::<Vec<_>>()
                .join("\n  - ")
        )
    } else {
        String::new()
    };
    let BlockColocationViolation {
        kind,
        line: headline,
        topic,
        producer,
        consumer,
        input,
        producer_group: pg,
        consumer_group: cg,
        block_inputs,
        ..
    } = ordered.into_iter().next().expect("non-empty checked above");
    let block_input_list = block_inputs
        .iter()
        .map(|e| format!("'{e}'"))
        .collect::<Vec<_>>()
        .join(" / ");
    let tail = format!(
        "OR (b) {REMEDY_SINGLE_PROCESS}, OR (c) dropping `block` from {block_input_list} if \
         losing frames on '{topic}' is acceptable. Deleting the `process_groups:` block \
         entirely also works: the derived partition co-locates a `block` topic's whole flow \
         automatically"
    );
    // The remedy ladder is NOT shared, because option (a) is DIRECTIONAL and
    // the two kinds point OPPOSITE ways.
    //
    // On a `BlockEdge` violation either endpoint may move: the two groups hold
    // one end each, so joining them fixes it whichever way round it is done.
    //
    // On a `MixedSibling` violation they do NOT. `pg` already holds the
    // producer AND its `block` consumer(s) — it must, or this violation would
    // have been a `BlockEdge` one and the ordering above would have made THAT
    // the headline — while `cg` holds only the non-`block` sibling. So moving
    // the PRODUCER into `cg` splits the very `block` edge this partition
    // currently has intact, and the result is refused again on the next
    // validation. Only moving the SIBLING (or the whole flow together) fixes
    // it, and the message says which way and why.
    let remedies = match kind {
        BlockColocationViolationKind::BlockEdge { .. } => format!(
            "Fix by (a) {REMEDY_CO_LOCATE} — move EVERY producer of '{topic}' into group \
             '{cg}' with '{consumer}' (or '{consumer}' into '{pg}') — {tail}"
        ),
        BlockColocationViolationKind::MixedSibling => format!(
            "Fix by (a) {REMEDY_CO_LOCATE} — move '{consumer}' into group '{pg}', which \
             already holds '{producer}' and its `block` input(s) {block_input_list}, so the \
             whole flow ends up in ONE group. Do NOT move '{producer}' into '{cg}' instead: \
             that would SPLIT the `block` edge this partition currently has intact, and the \
             result is refused in turn — {tail}"
        ),
    };
    let reason = match &kind {
        BlockColocationViolationKind::BlockEdge { bar } => format!(
            "this partition SPLITS a `block` edge across process groups:\n  - {}{more}\n  \
             `block` defers the producer's tick through a mirror the consumer's subscriber \
             decrements. A SPLIT edge can carry that mirror in shared memory — a \
             cross-process credit word — but only for a topic with exactly ONE in-graph \
             producer and NO non-`block` consumers, and {}. So this mirror stays \
             PROCESS-LOCAL: a producer in another OS process can never observe \
             '{consumer}.{input}''s queue, and the worker owning '{consumer}' would refuse \
             to build. Rewriting it to `drop_oldest` would silently LOSE data, so this is \
             refused instead. {remedies}",
            headline,
            render_credit_bar(&topic, bar)
        ),
        BlockColocationViolationKind::MixedSibling => format!(
            "this partition SPLITS a MIXED `block` topic's non-`block` consumer from the \
             rest of its flow:\n  - {}{more}\n  A topic carrying BOTH `block` and non-`block` \
             consumers degrades its `block` consumers to `drop_oldest` — deferring the \
             producer would starve the non-`block` sibling. That degrade is decided PER \
             PROCESS from the worker's OWN topology, and a worker cannot see a consumer it \
             does not own: split like this, the worker holding '{producer}' reads its flow \
             as all-`block`, INSTALLS the defer, and throttles '{producer}' to the `block` \
             consumer's drain rate — starving '{consumer}.{input}' — while \
             `--single-process` degrades and warns. The mixed-topic warn fires in NEITHER \
             process, so one graph would run two semantics in silence. {remedies}",
            headline
        ),
    };
    BlockColocationOutcome {
        credited,
        verdict: Err(TransportError::GraphError { reason }),
    }
}

/// What the co-location walk found: the split `block` edges it CREDITED, and
/// whether the partition is refused — INDEPENDENT of each other.
///
/// They are independent because the walk is per-seed: a partition may credit a
/// split on one topic and be refused for an unrelated topic in the same pass,
/// and the credited edges are still genuinely creditable. Folding them into one
/// `Result` discarded the credited set on any refusal, which made the reporting
/// surface claim there were none.
struct BlockColocationOutcome {
    /// Sorted + deduped, whatever the verdict.
    credited: Vec<CreditableSplitEdge>,
    /// `Err` names the FIRST violation, with its remedy ladder.
    verdict: TransportResult<()>,
}

/// One split `block` edge the plan-time check ACCEPTED because it
/// carries a cross-process credit word.
///
/// Reported so the acceptance can be ANNOUNCED where it is decided: before
/// this, a credited split and an ordinary co-located flow produced the same
/// `partition: spawner-consumable` and the same silence, and an operator could
/// not tell whether their split edge had been credited or quietly co-located
/// by the derivation. Ordering is by `topic` then consumer, via `Ord`.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct CreditableSplitEdge {
    pub topic: String,
    pub consumer_node: String,
    pub consumer_input: String,
    pub producer_group: String,
    pub consumer_group: String,
}

impl std::fmt::Display for CreditableSplitEdge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} -> {}.{} ({} -> {})",
            self.topic,
            self.consumer_node,
            self.consumer_input,
            self.producer_group,
            self.consumer_group
        )
    }
}

/// Every split `block` edge this partition carries a credit word
/// for — the ACCEPTANCE, reported for the operator-facing lines.
///
/// Delegates to the same CO-LOCATION walk [`validate_partition`] runs, so the
/// reported set is that decision itself rather than a second derivation.
///
/// SCOPE, stated exactly: this runs `validate_block_colocation` ALONE. It does
/// NOT run `validate_process_groups` or the per-group bridge/contiguity
/// checks, so a partition refused for one of THOSE reasons still reports a
/// non-empty set — the edges really are creditable; the partition is
/// unrunnable for an unrelated structural reason, which its own refusal names.
///
/// A partition refused by the CO-LOCATION walk itself also reports the edges
/// that walk credited. It used to report none, and that was a bug rather than a
/// scope statement: the walk is per-seed, so one partition can credit a split
/// on one topic and be refused over a different one, and reporting zero made
/// `partition_emit` overwrite a legal hand-written split without the warning
/// that names it. The edges are returned beside the verdict now, by the private
/// `BlockColocationOutcome` the walk hands back (named in plain text, NOT as an
/// intra-doc link: this fn is `pub` and that type is not, so a link here fails
/// the `RUSTDOCFLAGS=-D warnings` docs gate. It is deliberately not made `pub`
/// to satisfy the link — it appears in no public signature, so exporting it
/// would add a type no caller can obtain).
///
/// A topology that will not BUILD reports `Vec::new()` — there is no correct
/// answer to give, and every caller here is a reporting surface whose own
/// verdict comes from a path that surfaces the build error properly.
pub fn creditable_split_block_edges(
    groups: &IndexMap<String, Vec<String>>,
    config: &GraphConfig,
    entry_infos: &IndexMap<String, NodeInfo>,
    trigger_edges: &TriggerEdges,
) -> Vec<CreditableSplitEdge> {
    let Ok((topo, _)) = build_global(config, entry_infos, trigger_edges) else {
        return Vec::new();
    };
    // `.credited`, NOT the verdict: an edge this walk credited stays credited
    // even when the SAME walk refused the partition over an unrelated topic.
    validate_block_colocation(groups, &topo).credited
}

/// Which co-location constraint a partition broke, as the pairs iterator
/// labels it — see [`validate_block_colocation`] for why the two are reported
/// apart. A `BlockEdge` label becomes a VIOLATION only when the seed is not
/// creditable; [`BlockColocationViolationKind`] is what a recorded violation
/// carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BlockColocationKind {
    /// A producer split from a `block` consumer: the consumer's worker cannot
    /// build.
    BlockEdge,
    /// A MIXED topic's non-`block` consumer split from the flow: every worker
    /// builds, and the one holding the `block` consumer silently installs a
    /// defer the monolith degrades.
    MixedSibling,
}

/// A RECORDED violation's kind. The `BlockEdge` arm carries its
/// [`CreditBar`] by VALUE, not as an `Option`: a split `block` edge is
/// recorded only when the seed's bar is `Some`, so there is no "no bar" state
/// to render, and an `Option` here would be an unreachable arm the compiler
/// forces every reader past.
#[derive(Debug, Clone, PartialEq, Eq)]
enum BlockColocationViolationKind {
    BlockEdge { bar: CreditBar },
    MixedSibling,
}

/// The clause naming WHY a split `block` edge could not be credited.
///
/// The refusal owes the operator this sentence now that some split edges run:
/// "a split `block` edge cannot be honoured" stopped being true, so a message
/// without it sends the reader looking for a limitation that no longer exists.
///
/// `NoInGraphProducer` and `NoBlockConsumer` cannot reach here — a seed
/// requires both — but they RENDER rather than panicking, because a diagnostic
/// must never crash the verb it is explaining.
fn render_credit_bar(topic: &str, bar: &CreditBar) -> String {
    match bar {
        CreditBar::MultipleProducers(n) => format!(
            "topic '{topic}' has {n} in-graph producers, and a credit word counts ONE \
             producer's outstanding frames — two writers would each spend the other's credit"
        ),
        CreditBar::MixedTopic { .. } => format!(
            "topic '{topic}' ALSO carries non-`block` consumer(s) {}, which degrade its \
             `block` consumers to `drop_oldest` — there is no lossless defer left to credit",
            bar.siblings()
                .iter()
                .map(|e| format!("'{e}'"))
                .collect::<Vec<_>>()
                .join(" / ")
        ),
        CreditBar::NoInGraphProducer => {
            format!("topic '{topic}' has no in-graph producer, so there is nothing to defer")
        }
        CreditBar::NoBlockConsumer => {
            format!("topic '{topic}' has no `block` consumer, so there is no defer to credit")
        }
    }
}

/// One violation, kept whole so the headline can render the right diagnosis
/// (and the right REMEDY DIRECTION) without re-deriving either from a formatted
/// line, and so the reporting ORDER can be chosen after every violation is in.
struct BlockColocationViolation {
    /// The recorded kind — a `BlockEdge` carries the bar it hit, so the
    /// diagnosis and the decision that produced it are ONE value.
    kind: BlockColocationViolationKind,
    /// This violation's own rendered bullet.
    line: String,
    topic: String,
    producer: String,
    consumer: String,
    input: String,
    producer_group: String,
    consumer_group: String,
    block_inputs: Vec<String>,
}

impl BlockColocationViolation {
    /// This violation's bullet WITH its bar.
    ///
    /// The `(and N more)` tail used to render `line` alone, so an operator
    /// fixing a multi-violation partition learned one diagnosis per run: the
    /// headline named its bar and every other line named none, even when they
    /// hit DIFFERENT bars (a multi-producer topic and a mixed one can both be
    /// split by one partition). Each line now carries its own.
    fn rendered_with_bar(&self) -> String {
        match &self.kind {
            BlockColocationViolationKind::BlockEdge { bar } => {
                format!("{} — {}", self.line, render_credit_bar(&self.topic, bar))
            }
            BlockColocationViolationKind::MixedSibling => self.line.clone(),
        }
    }
}

/// Look up an edge's fire rate (mHz); a pair with no entry is rate 0.
fn edge_rate(costs: &PartitionCosts, producer: &str, consumer: &str) -> u64 {
    costs
        .edge_rate_mhz
        .get(&(producer.to_string(), consumer.to_string()))
        .copied()
        .unwrap_or(0)
}

/// Build the GLOBAL topology + trigger-aware levelization (the REAL code).
///
/// Routes through [`resolve_levels`] — the single
/// level-resolution seam — so a yaml `level_assignments:` override reaches
/// the partitioner's plan-time global levels exactly as it reaches the
/// runtime executor build. A partitioner levelizing independently of the
/// runtime would band process groups over levels the runtime never runs
/// (mp runs disagreeing with the yaml); one seam makes that divergence
/// structurally impossible. The algebraic-cycle diagnostic is likewise the
/// shared `resolve_levels` wording (one voice).
fn build_global(
    config: &GraphConfig,
    entry_infos: &IndexMap<String, NodeInfo>,
    trigger_edges: &TriggerEdges,
) -> TransportResult<(GraphTopology, Levels)> {
    let topo = GraphTopology::build(config, entry_infos)?;
    let levels = resolve_levels(config, &topo, trigger_edges)?;
    Ok((topo, levels))
}

/// The verdict of the per-group re-levelization bijection check.
enum GroupCheck {
    /// The group's induced-subgraph re-levelization is a contiguous-split
    /// bijection onto its owned global levels — spawner-consumable.
    Valid,
    /// The group's contiguous global band is bridged by a foreign node.
    Bridged {
        /// The named foreign trigger-predecessor (see [`find_bridge`]);
        /// `None` only on the unreachable no-predecessor arm.
        bridge: Option<String>,
        /// The full human-readable diagnostic (names the collapsed member,
        /// the level mismatch, the bridge, and the fix).
        diagnostic: String,
    },
    /// The group owns NON-ADJACENT global DAG levels. The per-member
    /// re-levelization bijection can pass here (a direct in-group edge spanning
    /// the gap collapses the owned band into a gap-free local band), so this is
    /// a DISTINCT verdict from [`GroupCheck::Bridged`]: the contiguous-split
    /// participant-map cannot represent a gapped band even when the bijection
    /// holds.
    NonContiguous {
        /// The sorted, deduped owned global levels — non-contiguous.
        owned: Vec<usize>,
        /// The full human-readable diagnostic (names the gapped owned band and
        /// the fix); reused verbatim by [`validate_partition`].
        diagnostic: String,
    },
}

/// Re-levelize a group's induced subgraph via the REAL
/// [`GraphTopology::build`] + [`GraphTopology::derive_levels`] and check that
/// each member's LOCAL level equals its rank among the group's OWNED global
/// levels — the exact `install_barrier_participant` precondition
/// (`local_count == owned_global_count`, in order). On failure the offending
/// member's foreign predecessor (the BRIDGE) is named.
///
/// Modular by design: a group's verdict depends only on its own members + which
/// global levels they occupy, so validating each group independently validates
/// the whole partition. A cross-group input to a member becomes a producer-less
/// (external) topic in the sub-topology — exactly how the group process sees it
/// at spawn — so a member fed only from outside re-levelizes as a local root,
/// which is what surfaces a bridge collapse.
fn check_group(
    member_ids: &[&str],
    config: &GraphConfig,
    entry_infos: &IndexMap<String, NodeInfo>,
    trigger_edges: &TriggerEdges,
    topo: &GraphTopology,
    global_levels: &Levels,
) -> TransportResult<GroupCheck> {
    let member_set: HashSet<&str> = member_ids.iter().copied().collect();

    // Owned global levels, sorted + deduped → local rank per owned level.
    let mut owned: Vec<usize> = Vec::with_capacity(member_ids.len());
    for id in member_ids {
        let g = global_levels
            .level_of(id)
            .ok_or_else(|| TransportError::GraphError {
                reason: format!(
                    "auto_partition: group member '{id}' has no level in the graph levelization \
                     (config was not levelized against this member)"
                ),
            })?;
        owned.push(g);
    }
    owned.sort_unstable();
    owned.dedup();
    let owned_rank: HashMap<usize, usize> =
        owned.iter().enumerate().map(|(r, g)| (*g, r)).collect();

    // Local re-levelization of the induced subgraph (reuses the real levelizer).
    let local = subgraph_local_levels(member_ids, config, entry_infos, trigger_edges)?;

    for id in member_ids {
        let g = global_levels
            .level_of(id)
            .expect("owned lookup above succeeded");
        let expected = owned_rank[&g];
        let actual = local
            .level_of(id)
            .expect("member is present in its own subgraph levelization");
        if actual != expected {
            let bridge = find_bridge(topo, trigger_edges, id, &member_set);
            let bridge_clause = match &bridge {
                Some(b) => format!("foreign bridge node '{b}'"),
                None => "a foreign node".to_string(),
            };
            let diagnostic = format!(
                "node '{id}' owns global level {g} (group-local rank {expected}) but re-levelizes \
                 to local level {actual} in the group's induced subgraph — {bridge_clause} breaks \
                 its in-group dependency chain, so the group's local level count would not match \
                 the {owned_count} global levels it owns (a valid cross-process split must be \
                 contiguous with no foreign bridge; move the bridge into this group or split the \
                 group at the bridge)",
                owned_count = owned.len()
            );
            return Ok(GroupCheck::Bridged { bridge, diagnostic });
        }
    }

    // The per-member bijection loop above PASSES for a group whose
    // owned global levels are NON-CONTIGUOUS as long as a DIRECT in-group edge
    // spans the gap — owned `{0,4}` with a direct `0→4` edge re-levelizes to
    // locals `{0,1}`, collapsing the gap so `local_count == owned_count` and
    // every `actual == expected`. The barrier's participant-map is a
    // CONTIGUOUS-split index map, so a gapped band is not representable
    // regardless of the bijection. PLACEMENT IS LOAD-BEARING: this test runs
    // AFTER the bridge loop, so a gapped group whose gap is caused by a FOREIGN
    // bridge (the bijection FAILS) still returns `Bridged` first — every
    // existing `Bridged` oracle is byte-identical. `owned` is sorted+deduped,
    // so a contiguous band satisfies `last - first + 1 == len`.
    if let (Some(&lo), Some(&hi)) = (owned.first(), owned.last()) {
        if hi - lo + 1 != owned.len() {
            let missing = (hi - lo + 1) - owned.len();
            let diagnostic = format!(
                "owns non-adjacent global DAG levels {owned:?} (spanning global levels \
                 {lo}..={hi}, but {missing} intermediate level(s) in that band are owned by \
                 other groups); the cross-process barrier only supports a CONTIGUOUS-split \
                 partition — give each group a contiguous band of the graph's pipeline \
                 stages"
            );
            return Ok(GroupCheck::NonContiguous { owned, diagnostic });
        }
    }

    Ok(GroupCheck::Valid)
}

/// Build a group's induced sub-topology (sub-config restricted to the group's
/// members) and levelize it with the REAL [`GraphTopology::derive_levels`].
/// Cross-group producers become external (producer-less) topics, so an
/// externally-fed member is a local root — mirroring the group process at spawn.
fn subgraph_local_levels(
    member_ids: &[&str],
    config: &GraphConfig,
    entry_infos: &IndexMap<String, NodeInfo>,
    trigger_edges: &TriggerEdges,
) -> TransportResult<Levels> {
    let member_set: HashSet<&str> = member_ids.iter().copied().collect();
    let sub_nodes: Vec<NodeDef> = config
        .nodes
        .iter()
        .filter(|nd| member_set.contains(nd.id.as_str()))
        .cloned()
        .collect();
    let sub_config = GraphConfig {
        // When the parent graph carries a `level_assignments`
        // override, the sub-config gets the COMPRESSED local band
        // ([`compress_group_level_assignments`]) — modelling EXACTLY what the
        // worker runs at spawn (`subgraph_for` in cerulion_cli_engine's
        // multiprocess planner sets the same compression on the worker's
        // sub-config; the two sites are in lockstep BY SHARING the one
        // compression fn). The worker no longer re-Kahns independently, so
        // the local side of the bijection check below models told-levels;
        // levelizing through `resolve_levels` also re-validates the
        // compressed band (contiguity + edge order) exactly as the worker
        // build will. Parent `None` ⇒ `None` ⇒ Kahn — byte-identical to the
        // earlier behavior, including the foreign-bridge refusal.
        level_assignments: config
            .level_assignments
            .as_ref()
            .map(|a| compress_group_level_assignments(a, member_ids)),
        // The plan-time subgraph drops the network block (like it
        // drops `process_groups`); network wiring is not a plan-time concern.
        network: None,
        // A plan-time subgraph is the SAME graph, so it
        // inherits the parent's resolved identity. The deprecated `name:`
        // key is dropped — a subgraph is never written back to disk, so it
        // has nothing to round-trip.
        name: None,
        identity: config.identity.clone(),
        prefix: config.prefix.clone(),
        nodes: sub_nodes,
        multi_publisher_topics: config.multi_publisher_topics.clone(),
        process_groups: IndexMap::new(),
        process_group_order: Vec::new(),
    };
    let sub_infos: IndexMap<String, NodeInfo> = entry_infos
        .iter()
        .filter(|(k, _)| member_set.contains(k.as_str()))
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();
    let sub_topo = GraphTopology::build(&sub_config, &sub_infos)?;
    resolve_levels(&sub_config, &sub_topo, trigger_edges).map_err(|e| TransportError::GraphError {
        reason: format!("auto_partition: group subgraph levelization failed: {e}"),
    })
}

/// Restrict a GLOBAL `level_assignments` map to one process
/// group's members and COMPRESS the member levels to the group's 0-based
/// LOCAL band — the level assignment the group's WORKER runs at spawn.
///
/// Local level = the member's rank among the group's sorted, deduped OWNED
/// global levels (exactly the `install_barrier_participant` /
/// per-group-bijection contract: "each member's LOCAL level equals its rank
/// among the group's owned global levels"). Well-defined for ANY member
/// level multiset, and the output always satisfies the level-assignment
/// invariants on the induced subgraph:
///
/// * **contiguous 0..K, no empty level** — rank-compression fills every rank
///   by construction (gaps in the global band collapse);
/// * **in-group trigger edges stay strictly increasing** — `g(m1) < g(m2)`
///   globally (the global assignment was validated) implies
///   `rank(g(m1)) < rank(g(m2))`. Cross-group edges become external topics
///   in the subgraph and constrain nothing locally.
///
/// Whether the group's owned GLOBAL band is itself contiguous (the
/// barrier participant-map requirement) is deliberately NOT this function's
/// concern — `validate_partition` / `validate_contiguous_ownership` gate
/// that axis on the GLOBAL side.
///
/// Members absent from `assignments` are skipped (upstream validation
/// requires full coverage; a genuinely missing member then fails the
/// worker's own coverage validation loudly rather than silently here).
/// Deterministic: output keys in `members` (graph) order.
pub fn compress_group_level_assignments(
    assignments: &IndexMap<String, usize>,
    members: &[&str],
) -> IndexMap<String, usize> {
    let mut owned: Vec<usize> = members
        .iter()
        .filter_map(|m| assignments.get(*m).copied())
        .collect();
    owned.sort_unstable();
    owned.dedup();
    let rank: HashMap<usize, usize> = owned.iter().enumerate().map(|(r, g)| (*g, r)).collect();
    members
        .iter()
        .filter_map(|m| assignments.get(*m).map(|g| ((*m).to_string(), rank[g])))
        .collect()
}

/// Backward BFS over the GLOBAL trigger DAG from `node`, returning the first
/// FOREIGN (non-member) predecessor — a bridge whose foreign placement
/// collapsed the group's re-levelization. Deterministic: predecessors are
/// visited in topic/producer insertion order.
///
/// # Limitation: first-foreign-predecessor, not necessarily the causal bridge
///
/// This returns the FIRST foreign node reached in a deterministic
/// insertion-order BFS, NOT a proven-minimal or uniquely-causal bridge. When a
/// group's re-levelization is collapsed by MULTIPLE foreign predecessors on
/// different chains (or a foreign node that is reachable but not actually the
/// one that shifted THIS member's local level), the name returned is whichever
/// foreign node the ordered BFS hits first — which may be a non-causal foreign
/// predecessor rather than "the" bridge. The name is therefore a diagnostic
/// HINT ("a foreign node here breaks the chain — look near this one"), not a
/// guaranteed root cause. This is intentional (a heuristic that names SOME
/// concrete foreign node beats "a foreign node" with no name); it is not
/// tightened because a minimal-bridge search would be more code for a
/// diagnostic-only string, and the fix ("move the bridge into this group or
/// split the group at the bridge") is the same regardless of which bridge is
/// named. Determinism is preserved either way (Principle #7).
fn find_bridge(
    topo: &GraphTopology,
    trigger_edges: &TriggerEdges,
    node: &str,
    member_set: &HashSet<&str>,
) -> Option<String> {
    let mut visited: HashSet<String> = HashSet::new();
    let mut queue: Vec<String> = trigger_predecessors(topo, trigger_edges, node);
    let mut head = 0;
    while head < queue.len() {
        let cur = queue[head].clone();
        head += 1;
        if !visited.insert(cur.clone()) {
            continue;
        }
        if !member_set.contains(cur.as_str()) {
            return Some(cur);
        }
        for p in trigger_predecessors(topo, trigger_edges, &cur) {
            queue.push(p);
        }
    }
    None
}

/// The immediate TRIGGER predecessors of `node` in the global topology — the
/// producers of every topic `node` triggers on.
fn trigger_predecessors(
    topo: &GraphTopology,
    trigger_edges: &TriggerEdges,
    node: &str,
) -> Vec<String> {
    let mut preds: Vec<String> = Vec::new();
    for flow in topo.topics() {
        let consumes = flow.consumers.iter().any(|c| c.node_id == node);
        if consumes && trigger_edges.is_triggering(node, &flow.topic) {
            for p in &flow.producers {
                if p != node {
                    preds.push(p.clone());
                }
            }
        }
    }
    preds
}

/// Collapse the union-find into a named grouping. Group ORDER = pipeline order
/// (each group's lead node's global level, then graph order); the NAME is
/// `grp_<lead>` (sanitized + uniquified), where the lead is the member with the
/// lowest global level (graph-order tiebreak). Members are in graph order.
fn assemble_groups(
    uf: &mut UnionFind,
    node_ids: &[&str],
    global_levels: &Levels,
) -> IndexMap<String, Vec<String>> {
    let n = node_ids.len();
    // rep → member indices (graph order, because we scan 0..n ascending).
    let mut by_rep: IndexMap<usize, Vec<usize>> = IndexMap::new();
    for i in 0..n {
        let r = uf.find(i);
        by_rep.entry(r).or_default().push(i);
    }

    // (lead_level, lead_idx, member indices) per group.
    let mut groups: Vec<(usize, usize, Vec<usize>)> = by_rep
        .into_values()
        .map(|members| {
            let lead = *members
                .iter()
                .min_by_key(|&&i| (global_levels.level_of(node_ids[i]).unwrap_or(usize::MAX), i))
                .expect("group is non-empty");
            let lead_level = global_levels.level_of(node_ids[lead]).unwrap_or(usize::MAX);
            (lead_level, lead, members)
        })
        .collect();
    // Pipeline order: lead global level, then graph order of the lead.
    groups.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));

    let mut out: IndexMap<String, Vec<String>> = IndexMap::new();
    let mut used: HashSet<String> = HashSet::new();
    for (_lead_level, lead_idx, members) in groups {
        let name = unique_group_name(node_ids[lead_idx], &mut used);
        let member_ids: Vec<String> = members.iter().map(|&i| node_ids[i].to_string()).collect();
        out.insert(name, member_ids);
    }
    out
}

/// `grp_<sanitized-lead>`, made unique by appending `_2`, `_3`, … on collision.
fn unique_group_name(lead: &str, used: &mut HashSet<String>) -> String {
    let base = format!("grp_{}", sanitize_ident(lead));
    if used.insert(base.clone()) {
        return base;
    }
    let mut suffix = 2usize;
    loop {
        let candidate = format!("{base}_{suffix}");
        if used.insert(candidate.clone()) {
            return candidate;
        }
        suffix += 1;
    }
}

/// Keep `[A-Za-z0-9_]`, map everything else to `_`; empty → `node`.
fn sanitize_ident(id: &str) -> String {
    let s: String = id
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    if s.is_empty() {
        "node".to_string()
    } else {
        s
    }
}

/// Minimal union-find over node indices; the group representative is always the
/// SMALLEST member index (keeps assembly deterministic).
struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }

    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]]; // path halving
            x = self.parent[x];
        }
        x
    }

    fn union(&mut self, a: usize, b: usize) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra == rb {
            return;
        }
        // Point the larger rep at the smaller so `find` yields the min index.
        let (keep, drop) = if ra < rb { (ra, rb) } else { (rb, ra) };
        self.parent[drop] = keep;
    }
}

// ===========================================================================
// Inline unit tests for the pure profiler helpers
// (`lower_median` + `rate_mhz`). Pure — no transport, no clock.
// ===========================================================================
#[cfg(test)]
mod chunk_c_helper_tests {
    use super::{lower_median, rate_mhz, HopCosts};

    #[test]
    fn platform_default_cross_exceeds_intra_and_pins_constants() {
        let hop = HopCosts::platform_default();
        // Universal invariant (read on EVERY target): the default must make
        // fusion profitable, else no edge ever fuses. Also guards against an
        // unused `hop` on an un-pinned fallback target.
        assert!(
            hop.cross_ns > hop.intra_ns,
            "cross ({}) must exceed intra ({})",
            hop.cross_ns,
            hop.intra_ns
        );
        // Per-OS/arch pins (measured), mirroring the
        // barrier.rs DEFAULT_BARRIER_SPIN_US per-OS assertion precedent.
        #[cfg(target_os = "macos")]
        {
            assert_eq!(hop.intra_ns, 1_000);
            assert_eq!(hop.cross_ns, 9_000);
        }
        #[cfg(all(target_os = "linux", target_arch = "x86_64"))]
        {
            assert_eq!(hop.intra_ns, 600);
            assert_eq!(hop.cross_ns, 6_800);
        }
        #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
        {
            assert_eq!(hop.intra_ns, 2_100);
            assert_eq!(hop.cross_ns, 6_800);
        }
    }

    #[test]
    fn lower_median_single_sample() {
        assert_eq!(lower_median(&[42]), 42);
    }

    #[test]
    fn lower_median_odd_count_is_exact_middle() {
        // sorted [1,3,5,7,9], middle index (5-1)/2 = 2 => 5.
        assert_eq!(lower_median(&[9, 1, 7, 3, 5]), 5);
    }

    #[test]
    fn lower_median_even_count_is_lower_of_two_central() {
        // sorted [10,20,30,40], index (4-1)/2 = 1 => 20 (the LOWER central, not 25).
        assert_eq!(lower_median(&[40, 10, 30, 20]), 20);
        // sorted [1,2] => index 0 => 1 (the lower of the pair, not 1.5→1).
        assert_eq!(lower_median(&[2, 1]), 1);
    }

    #[test]
    fn lower_median_is_order_independent() {
        // The result depends only on the multiset, not the input order.
        assert_eq!(
            lower_median(&[5, 1, 9, 3, 7]),
            lower_median(&[1, 3, 5, 7, 9])
        );
    }

    #[test]
    fn lower_median_with_duplicates() {
        // sorted [4,4,4,4] => index 1 => 4.
        assert_eq!(lower_median(&[4, 4, 4, 4]), 4);
        // sorted [1,2,2,9] => index 1 => 2.
        assert_eq!(lower_median(&[9, 2, 1, 2]), 2);
    }

    #[test]
    fn rate_mhz_one_khz_over_one_second() {
        // 1000 fires over 1s = 1000 Hz = 1_000_000 mHz.
        assert_eq!(rate_mhz(1_000, 1_000_000_000), 1_000_000);
    }

    #[test]
    fn rate_mhz_thirty_hz_over_one_second() {
        // 30 fires over 1s = 30 Hz = 30_000 mHz.
        assert_eq!(rate_mhz(30, 1_000_000_000), 30_000);
    }

    #[test]
    fn rate_mhz_sub_hertz() {
        // 1 fire over 2s = 0.5 Hz = 500 mHz.
        assert_eq!(rate_mhz(1, 2_000_000_000), 500);
    }

    #[test]
    fn rate_mhz_zero_fires_is_zero() {
        assert_eq!(rate_mhz(0, 1_000_000_000), 0);
    }

    #[test]
    fn rate_mhz_zero_window_is_zero_not_panic() {
        // The div-by-zero guard: a zero-length window yields rate 0 (do not
        // fuse), NOT a panic, even with a huge fire count.
        assert_eq!(rate_mhz(1_000_000, 0), 0);
        assert_eq!(rate_mhz(0, 0), 0);
    }

    #[test]
    fn rate_mhz_saturates_on_overflow() {
        // u64::MAX fires over a 1ns window would be ~1.8e31 mHz, far beyond
        // u64::MAX (~1.8e19) — it must CLAMP, never wrap.
        assert_eq!(rate_mhz(u64::MAX, 1), u64::MAX);
    }
}

// ===========================================================================
// In-module oracles for the `block` co-location constraint —
// the budget override, the absorption repair, and the plan-time refusal.
//
// Pure: hand-built `GraphConfig` + `NodeInfo` + `TriggerEdges`, no transport,
// no clock. Hand-written expected group maps throughout.
// ===========================================================================
#[cfg(test)]
mod block_colocation_tests {
    use super::*;
    // `flow_shape` builds the PRODUCTION value the rule is defined on.
    use crate::graph::config::{InputDef, OutputDef};
    use crate::graph::node::InputMeta;
    use crate::graph::topology::DEFAULT_CONSUMER_DEPTH;
    use crate::graph::topology::{ConsumerEdge, TopicFlow};

    fn meta(name: &str, trigger: bool, backpressure: BackpressurePolicy) -> InputMeta {
        InputMeta {
            name: name.to_string(),
            schema_hash: 0,
            trigger,
            depth: DEFAULT_CONSUMER_DEPTH,
            backpressure,
            expect_within_ms: None,
        }
    }

    fn out(name: &str) -> OutputDef {
        OutputDef {
            name: name.to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }
    }

    fn inp(name: &str, source: &str) -> InputDef {
        InputDef {
            name: name.to_string(),
            source: source.to_string(),
        }
    }

    /// An output publishing onto an ABSOLUTE topic — the only way two nodes
    /// can produce ONE topic (`GraphTopology::build` derives the topic from
    /// `<prefix>/<node>/<output>` otherwise, so two producers of one topic are
    /// unconstructible without an override).
    fn out_at(name: &str, topic: &str) -> OutputDef {
        OutputDef {
            topic: Some(topic.to_string()),
            ..out(name)
        }
    }

    /// A config with `topics` opted into `multi_publisher_topics:`.
    ///
    /// Named for what it DOES. The listing is a PERMISSION, not a producer
    /// count: `GraphTopology::build` refuses a second in-graph producer on an
    /// unlisted topic, but a listed topic with ONE producer is perfectly
    /// ordinary — and is Q4's creditable case, which is one of this helper's
    /// call sites. Takes a SLICE because a graph can list several, and the
    /// single-topic version made a two-topic arm override the field it had
    /// just set. Kept local rather than widening `config_of`, which ~40
    /// single-producer arms already use.
    fn multi_publisher_listed_config_of(nodes: Vec<NodeDef>, topics: &[&str]) -> GraphConfig {
        GraphConfig {
            multi_publisher_topics: topics.iter().map(|t| t.to_string()).collect(),
            ..config_of(nodes)
        }
    }

    fn node(id: &str, inputs: Vec<InputDef>, outputs: Vec<OutputDef>) -> NodeDef {
        NodeDef {
            ros2: None,
            id: id.to_string(),
            node_type: id.to_string(),
            inputs,
            outputs,
        }
    }

    fn config_of(nodes: Vec<NodeDef>) -> GraphConfig {
        GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "bp".to_string(),
            prefix: "p".to_string(),
            nodes,
            multi_publisher_topics: Vec::new(),
            process_groups: IndexMap::new(),
            process_group_order: Vec::new(),
        }
    }

    fn shape(map: &IndexMap<String, Vec<String>>) -> Vec<(String, Vec<String>)> {
        map.iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }

    fn line_is_warn(line: &str) -> bool {
        line.split_whitespace().any(|t| t == "WARN")
    }

    /// At least one captured line carries `$marker` AND the `WARN` level.
    ///
    /// The level is matched as a WHOLE WHITESPACE TOKEN: `tracing_test` renders
    /// the span name (the test function's own name) into every line, so a bare
    /// `line.contains("WARN")` is a substring test over text the test itself
    /// controls, not a level check. A MACRO rather than a fn because
    /// `#[traced_test]` injects `logs_assert` into the test body's scope, where
    /// it closes over that body's span name.
    macro_rules! assert_warn_line {
        ($marker:expr, $why:expr) => {
            logs_assert(|lines: &[&str]| {
                if lines.iter().any(|l| l.contains($marker) && line_is_warn(l)) {
                    Ok(())
                } else {
                    Err(format!(
                        "{}: expected a WARN line containing `{}`; captured: {:#?}",
                        $why, $marker, lines
                    ))
                }
            })
        };
    }

    /// As [`assert_warn_line`], for a structured field: `key=value` matched as a
    /// whole whitespace token, so `load_ns=900` cannot be satisfied by
    /// `load_ns=9000` (the whole-token `has_field` rule).
    macro_rules! assert_warn_field {
        ($key:expr, $value:expr, $why:expr) => {
            logs_assert(|lines: &[&str]| {
                let want = format!("{}={}", $key, $value);
                if lines
                    .iter()
                    .any(|l| line_is_warn(l) && l.split_whitespace().any(|t| t == want))
                {
                    Ok(())
                } else {
                    Err(format!(
                        "{}: expected a WARN line carrying the whole token `{}`; captured: {:#?}",
                        $why, want, lines
                    ))
                }
            })
        };
    }

    /// `a -(trigger via /p/a/feed)-> b -(trigger)-> c`, with `a -(block via
    /// /p/a/cmd)-> c` as well. `b` is the FOREIGN BRIDGE: `a` and `c` alone own
    /// global levels {0, 2} and `c` re-levelizes to local 1 instead of the
    /// rank-1 it owns only because `b` sits between them — so the seeded
    /// `{a, c}` group is `Bridged` and must ABSORB `b`.
    ///
    /// `b` consumes a DIFFERENT topic (`/p/a/feed`) on purpose: the block topic
    /// `/p/a/cmd` must stay all-`block` so `b` is a genuine foreign bridge
    /// rather than a seeded mixed sibling. Wire `b` onto `/p/a/cmd` and the
    /// seed pulls it in directly, the repair never runs, and this arm silently
    /// stops testing absorption at all.
    fn bridged_block_graph() -> (GraphConfig, IndexMap<String, NodeInfo>, TriggerEdges) {
        let config = config_of(vec![
            node("a", vec![], vec![out("cmd"), out("feed")]),
            node("b", vec![inp("t", "a/feed")], vec![out("cmd")]),
            node("c", vec![inp("t", "b/cmd"), inp("gate", "a/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("a".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "b".to_string(),
            NodeInfo::with_meta(
                vec![meta("t", true, BackpressurePolicy::DropOldest)],
                vec![],
            ),
        );
        infos.insert(
            "c".to_string(),
            NodeInfo::with_meta(
                vec![
                    meta("t", true, BackpressurePolicy::DropOldest),
                    meta("gate", false, BackpressurePolicy::Block),
                ],
                vec![],
            ),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("b", "/p/a/feed");
        edges.insert("c", "/p/b/cmd");
        (config, infos, edges)
    }

    /// The seed can produce a group the spawner cannot consume; it is REPAIRED
    /// by absorption, never refused (refusing would reject a graph that runs
    /// fine under `--single-process`).
    #[tracing_test::traced_test]
    #[test]
    fn a_bridged_block_group_absorbs_its_bridge_instead_of_refusing() {
        let (config, infos, edges) = bridged_block_graph();
        let groups = baseline_process_per_node(&config, &infos, &edges).expect("derive");
        assert_eq!(
            shape(&groups),
            vec![(
                "grp_a".to_string(),
                vec!["a".to_string(), "b".to_string(), "c".to_string()]
            )],
            "the seeded {{a, c}} group is bridged by `b`, so `b` is absorbed"
        );
        // And the result really is spawner-consumable — the whole point of the
        // repair (a `Bridged` group would be refused by this very validator).
        validate_partition(&groups, &config, &infos, &edges)
            .expect("the repaired partition must pass the spawner-consumability check");
        assert_warn_line!(
            "absorbing a foreign bridge node",
            "every absorption must be announced, at a level the partition verb prints"
        );
    }

    /// ANTI-TAUTOLOGY: the same graph with `drop_oldest` in place of `block`
    /// derives the untouched process-per-node baseline. Without this, the arm
    /// above is satisfied by a repair that fuses everything unconditionally.
    #[test]
    fn the_same_graph_without_block_keeps_the_process_per_node_baseline() {
        let (config, mut infos, edges) = bridged_block_graph();
        infos.insert(
            "c".to_string(),
            NodeInfo::with_meta(
                vec![
                    meta("t", true, BackpressurePolicy::DropOldest),
                    meta("gate", false, BackpressurePolicy::DropOldest),
                ],
                vec![],
            ),
        );
        let groups = baseline_process_per_node(&config, &infos, &edges).expect("derive");
        assert_eq!(
            shape(&groups),
            vec![
                ("grp_a".to_string(), vec!["a".to_string()]),
                ("grp_b".to_string(), vec!["b".to_string()]),
                ("grp_c".to_string(), vec!["c".to_string()]),
            ]
        );
    }

    /// The budget gate guards CANDIDATE fusions, and a seed is not a candidate
    /// — so co-location bypasses it by construction. That is the intended
    /// semantic (a `block` edge the partition may not split outranks a latency
    /// budget), and it must be ANNOUNCED rather than silent.
    ///
    /// The isolated member is the point of the `load_ns` wording: it holds a
    /// placeholder 0 that is not a measurement, so the reported load is a FLOOR
    /// and the line must say which members are unmeasured (Principle #13 — a
    /// fabricated cost would be worse than a labelled floor).
    #[tracing_test::traced_test]
    #[test]
    fn a_block_group_over_budget_is_kept_and_the_override_is_announced() {
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("cons", vec![inp("gate", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("cons", "/p/prod/cmd");

        let costs = PartitionCosts {
            node_p50_ns: [("prod".to_string(), 900u64)].into_iter().collect(),
            edge_rate_mhz: BTreeMap::new(),
            hop: HopCosts::platform_default(),
        };
        // `cons` is isolated ⇒ no measured cost. Budget 100ns < prod's 900ns,
        // so the seeded group is over budget on the measured members alone.
        let isolated: BTreeSet<String> = ["cons".to_string()].into_iter().collect();
        let out = auto_partition(&config, &infos, &edges, &costs, &isolated, 100).expect("derive");
        assert_eq!(
            shape(&out.groups),
            vec![(
                "grp_prod".to_string(),
                vec!["prod".to_string(), "cons".to_string()]
            )],
            "co-location WINS over the budget — a split block edge cannot build"
        );
        // LEVEL TOKENS, not bare text: `traced_test` installs a subscriber with
        // NO env filter, so a `warn!` demoted to `debug!` still satisfies every
        // `logs_contain` while vanishing at the `cerulion=warn` default these
        // verbs actually run under (the demoted-level regression class).
        assert_warn_line!(
            "exceeds the per-group compute budget",
            "the override must be announced, and LOUDLY — `graph partition` defaults to \
             cerulion=warn"
        );
        assert_warn_field!(
            "load_ns",
            "900",
            "the reported load is the sum of the MEASURED members"
        );
        assert_warn_field!(
            "unmeasured_members",
            "cons",
            "an isolated member contributes no cost and the line must say so"
        );
        assert_warn_line!(
            "OVERRIDES isolation",
            "overriding the isolated-stays-singleton rule must be loud"
        );
    }

    /// The budget line must NOT fire on a group that fits — otherwise the arm
    /// above would pass against an unconditional log.
    #[tracing_test::traced_test]
    #[test]
    fn a_block_group_inside_the_budget_announces_no_override() {
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("cons", vec![inp("gate", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("cons", "/p/prod/cmd");
        let costs = PartitionCosts {
            node_p50_ns: [("prod".to_string(), 10u64), ("cons".to_string(), 10u64)]
                .into_iter()
                .collect(),
            edge_rate_mhz: BTreeMap::new(),
            hop: HopCosts::platform_default(),
        };
        let out = auto_partition(&config, &infos, &edges, &costs, &BTreeSet::new(), 1_000_000)
            .expect("derive");
        assert_eq!(out.groups.len(), 1);
        assert!(
            !logs_contain("exceeds the per-group compute budget"),
            "a group inside its budget must announce no override"
        );
    }

    /// The seed must run BEFORE the greedy loop. Seeding after it would fold
    /// the budget over the wrong member set and mis-record the co-located pair
    /// as an `Unprofitable` rejection (the loop's already-same-group arm exists
    /// precisely so a seeded pair asks for nothing).
    #[test]
    fn a_co_located_pair_is_never_recorded_as_a_fusion_rejection() {
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("cons", vec![inp("gate", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("cons", "/p/prod/cmd");
        let costs = PartitionCosts {
            node_p50_ns: [("prod".to_string(), 10u64), ("cons".to_string(), 10u64)]
                .into_iter()
                .collect(),
            // rate 0 ⇒ coupling 0 ⇒ the loop would reject it as Unprofitable
            // if the seed had not already fused the pair.
            edge_rate_mhz: BTreeMap::new(),
            hop: HopCosts::platform_default(),
        };
        let out = auto_partition(&config, &infos, &edges, &costs, &BTreeSet::new(), 1_000)
            .expect("derive");
        assert!(
            out.rejections.is_empty(),
            "a seeded pair must ask the greedy loop for nothing; got {:?}",
            out.rejections
        );
        assert!(
            out.fusions.is_empty(),
            "and it is not a FUSION either — it was never a candidate"
        );
    }

    // ---- The credit exemption (C5), as a pure table --------

    /// Build a `TopicFlow` by SHAPE, so the oracle below enumerates the whole
    /// space instead of the shapes some graph happens to produce — and drives
    /// the PRODUCTION decision (`TopicFlow::credit_bar`), not a test copy of
    /// it.
    fn flow_shape(producers: usize, block: usize, other: usize) -> TopicFlow {
        let consumer = |id: String, policy| ConsumerEdge {
            node_id: id,
            input: "inp".to_string(),
            policy,
            depth: 4,
        };
        TopicFlow {
            topic: "/t".to_string(),
            producers: (0..producers).map(|i| format!("p{i}")).collect(),
            consumers: (0..block)
                .map(|i| consumer(format!("b{i}"), BackpressurePolicy::Block))
                .chain(
                    (0..other).map(|i| consumer(format!("o{i}"), BackpressurePolicy::DropOldest)),
                )
                .collect(),
        }
    }

    /// `TopicFlow::credit_bar` is the ONE body of the creditability rule, and
    /// this is its table: which shapes can carry a cross-process credit word,
    /// and for the rest, WHICH bar they hit.
    ///
    /// The whole `(producers, block, other)` space up to 3 each, not a
    /// hand-picked six — the `0` rows are exactly the ones the previous
    /// seed-shaped predicate could REPRESENT but never answered correctly
    /// (`producers = 0` rendered "has 0 in-graph producers … two writers would
    /// each spend the other's credit"; `block = 0` read as creditable while
    /// `is_all_block()` was false).
    ///
    /// The bar VALUE is asserted, not just its presence: a classifier that
    /// always answered `MixedTopic` satisfies a totality-only check.
    #[test]
    fn the_credit_bar_answers_every_flow_shape_and_names_the_right_bar() {
        for producers in 0..=3 {
            for block in 0..=3 {
                for other in 0..=3 {
                    let flow = flow_shape(producers, block, other);
                    let want = match (producers, block, other) {
                        // No in-graph producer: nothing to defer, and
                        // `validate` refuses `block` here with its own message.
                        (0, _, _) => Some(CreditBar::NoInGraphProducer),
                        // Multi-producer OUTRANKS the mixed bar: it is the one
                        // an operator cannot fix by moving a node.
                        (n, _, _) if n > 1 => Some(CreditBar::MultipleProducers(n)),
                        // One producer and NO `block` consumer — whether or
                        // not non-`block` ones exist. There is nothing to
                        // defer, so it is NOT "mixed": calling it that renders
                        // a sentence about degrading `block` consumers the
                        // topic does not have. (This row is the C5 review
                        // correction; the cube previously CODIFIED the wrong
                        // answer at `(1, 0, >0)`.)
                        (_, 0, _) => Some(CreditBar::NoBlockConsumer),
                        // One producer, `block` consumer(s) AND a non-`block`
                        // sibling ⇒ MIXED, naming every sibling in graph order.
                        (_, _, o) if o > 0 => Some(CreditBar::MixedTopic {
                            first: "o0.inp".to_string(),
                            rest: (1..o).map(|i| format!("o{i}.inp")).collect(),
                        }),
                        // THE creditable shape, at 1 and at N block consumers
                        // (the mint emits one word per consumer).
                        _ => None,
                    };
                    assert_eq!(
                        flow.credit_bar(),
                        want,
                        "producers={producers} block={block} other={other}"
                    );
                    assert_eq!(
                        flow.is_creditable(),
                        want.is_none(),
                        "`is_creditable` must BE `credit_bar().is_none()`, not agree with it"
                    );
                    // The cross-crate pin, stated at its real strength: this
                    // is the condition the mint WAS written in terms of before
                    // the hoist. Both halves now call `is_creditable()`, so
                    // this is no longer the thing keeping them together — it
                    // is an INDEPENDENT cross-check that the hoisted rule
                    // still means what the mint used to mean.
                    assert_eq!(
                        flow.is_creditable(),
                        flow.producers.len() == 1 && flow.is_all_block(),
                        "creditability must equal the mint's own condition at \
                         producers={producers} block={block} other={other}"
                    );
                }
            }
        }
    }

    /// The seed STAMPS the flow's answer rather than re-deriving one.
    ///
    /// `BlockColocationSeed::credit_bar` is what the validator consults, and it
    /// is filled at construction from the same call the mint makes. A seed that
    /// recomputed the rule from its own public fields would be a second
    /// implementation free to drift — this drives the PRODUCTION constructor
    /// (`block_colocation_seeds`) over a real topology and requires the stamped
    /// answer to equal the flow's.
    #[test]
    fn the_seed_stamps_the_flows_own_credit_bar() {
        // One creditable topic, one mixed, one multi-producer — all three seeded.
        let config = multi_publisher_listed_config_of(
            vec![
                node("prod", vec![], vec![out("clean"), out("mixed")]),
                node("mp_a", vec![], vec![out_at("s", "/shared")]),
                node("mp_b", vec![], vec![out_at("s", "/shared")]),
                node("c_clean", vec![inp("gate", "prod/clean")], vec![]),
                node("c_mixed", vec![inp("gate", "prod/mixed")], vec![]),
                node("c_lossy", vec![inp("watch", "prod/mixed")], vec![]),
                node("c_shared", vec![inp("gate", "/shared")], vec![]),
            ],
            &["/shared"],
        );
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        for p in ["prod", "mp_a", "mp_b"] {
            infos.insert(p.to_string(), NodeInfo::with_meta(vec![], vec![]));
        }
        for c in ["c_clean", "c_mixed", "c_shared"] {
            infos.insert(
                c.to_string(),
                NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
            );
        }
        infos.insert(
            "c_lossy".to_string(),
            NodeInfo::with_meta(
                vec![meta("watch", true, BackpressurePolicy::DropOldest)],
                vec![],
            ),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("c_clean", "/p/prod/clean");
        edges.insert("c_mixed", "/p/prod/mixed");
        edges.insert("c_lossy", "/p/prod/mixed");
        edges.insert("c_shared", "/shared");
        let (topo, _) = build_global(&config, &infos, &edges).expect("topology");

        let seeds = block_colocation_seeds(&topo);
        assert_eq!(seeds.len(), 3, "all three block-carrying flows are seeded");
        for seed in &seeds {
            let flow = topo.topic(&seed.topic).expect("the seed's own flow");
            assert_eq!(
                seed.credit_bar(),
                flow.credit_bar().as_ref(),
                "seed '{}' must carry the FLOW's answer, not its own",
                seed.topic
            );
        }
        // And the three answers really are different, or the arm above would
        // pass against a constructor that stamped `None` on everything.
        let bars: Vec<Option<&CreditBar>> = seeds.iter().map(|s| s.credit_bar()).collect();
        assert!(
            bars.contains(&None)
                && bars
                    .iter()
                    .any(|b| matches!(b, Some(CreditBar::MixedTopic { .. })))
                && bars
                    .iter()
                    .any(|b| matches!(b, Some(CreditBar::MultipleProducers(2)))),
            "the fixture must exercise all three answers; got {bars:?}"
        );
    }

    /// Credit exemption (C5 pass 2): the REPORTING seam's own oracle.
    ///
    /// `creditable_split_block_edges` feeds three operator surfaces (the
    /// acceptance `info!`, the `graph levels` verdict, the `graph partition`
    /// overwrite warning) and had no direct test — every arm reached it only
    /// through a rendered string. Hand-written expected vectors here, so a
    /// wrong SET is caught at the source rather than as a puzzling diff in
    /// three different messages.
    #[test]
    fn creditable_split_block_edges_reports_exactly_the_split_creditable_edges() {
        // (a) THE shape: one producer, one `block` consumer, split.
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("cons", vec![inp("gate", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("cons", "/p/prod/cmd");
        let mut split: IndexMap<String, Vec<String>> = IndexMap::new();
        split.insert("front".to_string(), vec!["prod".to_string()]);
        split.insert("back".to_string(), vec!["cons".to_string()]);
        assert_eq!(
            creditable_split_block_edges(&split, &config, &infos, &edges),
            vec![CreditableSplitEdge {
                topic: "/p/prod/cmd".to_string(),
                consumer_node: "cons".to_string(),
                consumer_input: "gate".to_string(),
                producer_group: "front".to_string(),
                consumer_group: "back".to_string(),
            }]
        );
        // The rendered form the three surfaces all print.
        assert_eq!(
            creditable_split_block_edges(&split, &config, &infos, &edges)[0].to_string(),
            "/p/prod/cmd -> cons.gate (front -> back)"
        );

        // MIXED PARTITION: one CREDITABLE split plus, on an UNRELATED topic, an
        // edge the walk REFUSES. The creditable edge must still be reported.
        //
        // This is the regression the `Result<Vec, _>` shape caused: the credited
        // vec was destroyed by the `Err`, `creditable_split_block_edges` turned
        // that into `unwrap_or_default()`, and the reporting surface then said
        // "no creditable edges" about a partition that holds one. `partition_emit`
        // believed it and overwrote a legal hand-written split with no warning
        // naming what it replaced — silent loss of the operator's own choice.
        //
        // The refused half is a MIXED `block` topic: one producer whose topic
        // carries BOTH a `block` consumer and a non-`block` sibling, split. Its
        // `block` consumers degrade to `drop_oldest`, so there is no lossless
        // defer left to credit and the walk refuses it. It shares no topic, no
        // node and no edge with the creditable one above.
        let mixed_cfg = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("cons", vec![inp("gate", "prod/cmd")], vec![]),
            node("prod2", vec![], vec![out("tele")]),
            node("blocker", vec![inp("gate2", "prod2/tele")], vec![]),
            node("lossy", vec![inp("watch", "prod2/tele")], vec![]),
        ]);
        let mut mx_infos: IndexMap<String, NodeInfo> = IndexMap::new();
        mx_infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        mx_infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        mx_infos.insert("prod2".to_string(), NodeInfo::with_meta(vec![], vec![]));
        mx_infos.insert(
            "blocker".to_string(),
            NodeInfo::with_meta(vec![meta("gate2", true, BackpressurePolicy::Block)], vec![]),
        );
        mx_infos.insert(
            "lossy".to_string(),
            NodeInfo::with_meta(
                vec![meta("watch", true, BackpressurePolicy::DropOldest)],
                vec![],
            ),
        );
        let mut mx_edges = TriggerEdges::new();
        mx_edges.insert("cons", "/p/prod/cmd");
        mx_edges.insert("blocker", "/p/prod2/tele");
        mx_edges.insert("lossy", "/p/prod2/tele");
        let mut mx_groups: IndexMap<String, Vec<String>> = IndexMap::new();
        mx_groups.insert(
            "front".to_string(),
            vec!["prod".to_string(), "prod2".to_string(), "lossy".to_string()],
        );
        mx_groups.insert(
            "back".to_string(),
            vec!["cons".to_string(), "blocker".to_string()],
        );

        // PRECONDITION, asserted rather than assumed: this partition really IS
        // refused. Without it the test could pass on a partition that is simply
        // legal, which would prove nothing about surviving a refusal.
        let (mx_topo, _) = build_global(&mixed_cfg, &mx_infos, &mx_edges).expect("topology builds");
        let outcome = validate_block_colocation(&mx_groups, &mx_topo);
        assert!(
            outcome.verdict.is_err(),
            "precondition: the MIXED-topic split must be REFUSED, or this test \
             proves nothing"
        );
        // THE POINT: the credited edge survives that refusal.
        assert_eq!(
            creditable_split_block_edges(&mx_groups, &mixed_cfg, &mx_infos, &mx_edges)
                .iter()
                .map(|e| e.to_string())
                .collect::<Vec<_>>(),
            vec!["/p/prod/cmd -> cons.gate (front -> back)".to_string()],
            "a refusal on an UNRELATED topic must not erase the edges this same \
             walk credited"
        );

        // CO-LOCATED: creditable, but nothing is SPLIT, so nothing is reported.
        let mut together: IndexMap<String, Vec<String>> = IndexMap::new();
        together.insert(
            "one".to_string(),
            vec!["prod".to_string(), "cons".to_string()],
        );
        assert!(creditable_split_block_edges(&together, &config, &infos, &edges).is_empty());

        // FAN-OUT: one producer, two `block` consumers in two foreign groups
        // ⇒ BOTH reported, in topology then graph order.
        let fan = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("s1", vec![inp("gate", "prod/cmd")], vec![]),
            node("s2", vec![inp("gate", "prod/cmd")], vec![]),
        ]);
        let mut fan_infos: IndexMap<String, NodeInfo> = IndexMap::new();
        fan_infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        for c in ["s1", "s2"] {
            fan_infos.insert(
                c.to_string(),
                NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
            );
        }
        let mut fan_edges = TriggerEdges::new();
        fan_edges.insert("s1", "/p/prod/cmd");
        fan_edges.insert("s2", "/p/prod/cmd");
        let mut fan_groups: IndexMap<String, Vec<String>> = IndexMap::new();
        fan_groups.insert("g0".to_string(), vec!["prod".to_string()]);
        fan_groups.insert("g1".to_string(), vec!["s1".to_string()]);
        fan_groups.insert("g2".to_string(), vec!["s2".to_string()]);
        assert_eq!(
            creditable_split_block_edges(&fan_groups, &fan, &fan_infos, &fan_edges)
                .iter()
                .map(|e| (e.consumer_node.as_str(), e.consumer_group.as_str()))
                .collect::<Vec<_>>(),
            vec![("s1", "g1"), ("s2", "g2")]
        );

        // REFUSED BY THE CO-LOCATION WALK (a MIXED split) ⇒ none. This is the
        // half of the documented scope that IS enforced here; a partition
        // refused for a STRUCTURAL reason still reports its creditable edges,
        // which the fn's doc states rather than pretends otherwise.
        let mixed = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("blocker", vec![inp("gate", "prod/cmd")], vec![]),
            node("lossy", vec![inp("watch", "prod/cmd")], vec![]),
        ]);
        let mut mixed_infos: IndexMap<String, NodeInfo> = IndexMap::new();
        mixed_infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        mixed_infos.insert(
            "blocker".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        mixed_infos.insert(
            "lossy".to_string(),
            NodeInfo::with_meta(
                vec![meta("watch", true, BackpressurePolicy::DropOldest)],
                vec![],
            ),
        );
        let mut mixed_edges = TriggerEdges::new();
        mixed_edges.insert("blocker", "/p/prod/cmd");
        mixed_edges.insert("lossy", "/p/prod/cmd");
        let mut mixed_groups: IndexMap<String, Vec<String>> = IndexMap::new();
        mixed_groups.insert(
            "front".to_string(),
            vec!["prod".to_string(), "lossy".to_string()],
        );
        mixed_groups.insert("back".to_string(), vec!["blocker".to_string()]);
        assert!(
            creditable_split_block_edges(&mixed_groups, &mixed, &mixed_infos, &mixed_edges)
                .is_empty()
        );
    }

    // ---- the plan-time refusal (ii) ------------------------------------

    /// The credit exemption (C5) INVERTS the co-location headline: a HAND-WRITTEN
    /// partition that splits a SINGLE-PRODUCER, ALL-`block` edge is now
    /// ACCEPTED, because the supervisor mints that edge a cross-process credit
    /// word and the defer crosses the boundary losslessly.
    ///
    /// This arm was `..._is_refused` until the credit exemption. The refusal it pinned
    /// was a statement about the WORD — a process-local heap cell one address
    /// space cannot show another — and 2d changed which words exist. The shape
    /// it uses is EXACTLY the mint's (`credit_edges_for`: one in-graph
    /// producer, `is_all_block()`, consumer rank != producer rank), which is
    /// why it flips and the multi-producer twin below does not.
    #[test]
    fn a_hand_written_partition_that_splits_a_creditable_block_edge_is_accepted() {
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("cons", vec![inp("gate", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("cons", "/p/prod/cmd");

        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert("front".to_string(), vec!["prod".to_string()]);
        groups.insert("back".to_string(), vec!["cons".to_string()]);
        validate_partition(&groups, &config, &infos, &edges)
            .expect("a single-producer all-`block` split edge carries a credit word");

        // PRECONDITION, asserted rather than assumed: the seed really is the
        // creditable shape. Without this the arm would also pass if the
        // topology had stopped seeing the `block` consumer at all.
        let (topo, _) = build_global(&config, &infos, &edges).expect("topology");
        let seeds = block_colocation_seeds(&topo);
        assert_eq!(seeds.len(), 1, "one `block`-carrying flow");
        assert!(
            seeds[0].credit_bar().is_none(),
            "this arm proves nothing unless the seed is the creditable shape"
        );
    }

    /// Q4 (decided): a topic LISTED in `multi_publisher_topics:` but carrying
    /// exactly ONE in-graph producer is still creditable, so its split `block`
    /// edge is accepted.
    ///
    /// The listing is a permission, not a count. `producers` holds the
    /// IN-GRAPH producers, and the word describes exactly those — an external
    /// publisher on such a topic cannot be deferred at all, co-located or not,
    /// which is a pre-existing property of `block` + `multi_publisher_topics:`
    /// that C5 neither creates nor worsens. Refusing on the LISTING would
    /// refuse a shape that runs, and reading the listing as "many producers"
    /// is the exact conflation this arm exists to prevent.
    #[test]
    fn a_multi_publisher_listed_topic_with_one_in_graph_producer_is_still_creditable() {
        let config = multi_publisher_listed_config_of(
            vec![
                node("prod", vec![], vec![out_at("cmd", "/shared/cmd")]),
                node("cons", vec![inp("gate", "/shared/cmd")], vec![]),
            ],
            &["/shared/cmd"],
        );
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("cons", "/shared/cmd");

        // PRECONDITION: the topic really is listed, so the arm is about the
        // LISTING and not about an ordinary single-producer topic.
        assert_eq!(
            config.multi_publisher_topics,
            vec!["/shared/cmd".to_string()],
            "this arm proves nothing unless the topic is multi-publisher-listed"
        );
        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert("front".to_string(), vec!["prod".to_string()]);
        groups.insert("back".to_string(), vec!["cons".to_string()]);
        validate_partition(&groups, &config, &infos, &edges)
            .expect("one IN-GRAPH producer is creditable however the topic is listed");
    }

    /// The twin that did NOT flip: a MULTI-PRODUCER split `block` edge is
    /// still refused, and the refusal now says WHY it could not be credited —
    /// naming producer, consumer + input, topic, both groups, the literal
    /// group edit, and `--single-process`.
    #[test]
    fn a_hand_written_partition_that_splits_a_multi_producer_block_edge_is_refused() {
        let config = multi_publisher_listed_config_of(
            vec![
                node("prod_a", vec![], vec![out_at("cmd", "/shared/cmd")]),
                node("prod_b", vec![], vec![out_at("cmd", "/shared/cmd")]),
                node("cons", vec![inp("gate", "/shared/cmd")], vec![]),
            ],
            &["/shared/cmd"],
        );
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        for p in ["prod_a", "prod_b"] {
            infos.insert(p.to_string(), NodeInfo::with_meta(vec![], vec![]));
        }
        infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("cons", "/shared/cmd");

        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert(
            "front".to_string(),
            vec!["prod_a".to_string(), "prod_b".to_string()],
        );
        groups.insert("back".to_string(), vec!["cons".to_string()]);
        let err = validate_partition(&groups, &config, &infos, &edges)
            .expect_err("a split MULTI-PRODUCER block edge must still be refused");
        let msg = err.to_string();
        for needle in [
            "/shared/cmd",
            "prod_a",
            // BOTH producers: "names both producers" was asserted by the
            // message's own remedy ("move EVERY producer") and pinned by
            // neither half until now.
            "prod_b",
            "cons.gate",
            "'front'",
            "'back'",
            REMEDY_CO_LOCATE,
            REMEDY_SINGLE_PROCESS,
        ] {
            assert!(
                msg.contains(needle),
                "the refusal must name `{needle}`; got: {msg}"
            );
        }
        // The credit-exemption half: it must say why THIS edge could not be
        // credited, or it sends the operator looking for a limitation that no
        // longer exists.
        assert!(
            msg.contains("has 2 in-graph producers"),
            "the refusal must name the actual bar and its COUNT; got: {msg}"
        );
        assert!(
            !msg.contains("ALSO carries non-`block` consumer(s)"),
            "the MIXED bar is the wrong diagnosis for a multi-producer edge; got: {msg}"
        );
        // WHOLE SENTENCES. Every needle above sits inside ONE unbroken
        // segment, which is how this literal shipped carrying runs of 13-22
        // embedded spaces mid-sentence (a botched line-join baked the
        // continuation indentation into the string; `cargo fmt` never rewrites
        // a string's contents). A needle spanning a clause boundary fails on a
        // re-break, so these are what pin the RENDERED text.
        for sentence in [
            "A SPLIT edge can carry that mirror in shared memory — a cross-process credit \
             word — but only for a topic with exactly ONE in-graph producer and NO \
             non-`block` consumers",
            "Rewriting it to `drop_oldest` would silently LOSE data, so this is refused \
             instead.",
            "Deleting the `process_groups:` block entirely also works: the derived partition \
             co-locates a `block` topic's whole flow automatically",
        ] {
            assert!(
                msg.contains(sentence),
                "the refusal renders mangled whitespace — expected `{sentence}`; got: {msg}"
            );
        }
    }

    /// The OTHER still-refused shape gets the OTHER sentence: a split
    /// `block` edge on a MIXED topic is refused because the degrade leaves no
    /// lossless defer to credit — and the message names the sibling that
    /// caused it, which is the node the operator has to move.
    #[test]
    fn a_split_block_edge_on_a_mixed_topic_is_refused_naming_the_sibling_that_bars_credit() {
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("blocker", vec![inp("gate", "prod/cmd")], vec![]),
            node("lossy", vec![inp("watch", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "blocker".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        infos.insert(
            "lossy".to_string(),
            NodeInfo::with_meta(
                vec![meta("watch", true, BackpressurePolicy::DropOldest)],
                vec![],
            ),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("blocker", "/p/prod/cmd");
        edges.insert("lossy", "/p/prod/cmd");
        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert(
            "front".to_string(),
            vec!["prod".to_string(), "lossy".to_string()],
        );
        groups.insert("back".to_string(), vec!["blocker".to_string()]);
        let msg = validate_partition(&groups, &config, &infos, &edges)
            .expect_err("a MIXED topic's split block edge is not creditable")
            .to_string();
        assert!(
            msg.contains("ALSO carries non-`block` consumer(s) 'lossy.watch'"),
            "the mixed bar must name the sibling the operator has to move; got: {msg}"
        );
        assert!(
            !msg.contains("in-graph producers"),
            "the multi-producer bar is the wrong diagnosis here; got: {msg}"
        );
    }

    /// The rewritten refusal is DETERMINISTIC: two calls render byte-identical
    /// text.
    ///
    /// C5 rewrote the reason, the remedy and the `(and N more)` tail, and the
    /// tail now renders a per-line bar drawn from a `Vec` walk — exactly the
    /// shape where an accidental set/hash iteration would leak run-to-run
    /// order into an operator-facing message (and into any log diff).
    #[test]
    fn the_rewritten_block_colocation_refusal_is_deterministic() {
        let config = multi_publisher_listed_config_of(
            vec![
                node("prod_a", vec![], vec![out_at("cmd", "/shared/cmd")]),
                node("prod_b", vec![], vec![out_at("cmd", "/shared/cmd")]),
                node("cons", vec![inp("gate", "/shared/cmd")], vec![]),
            ],
            &["/shared/cmd"],
        );
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        for p in ["prod_a", "prod_b"] {
            infos.insert(p.to_string(), NodeInfo::with_meta(vec![], vec![]));
        }
        infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("cons", "/shared/cmd");
        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert(
            "front".to_string(),
            vec!["prod_a".to_string(), "prod_b".to_string()],
        );
        groups.insert("back".to_string(), vec!["cons".to_string()]);

        let one = validate_partition(&groups, &config, &infos, &edges)
            .expect_err("refused")
            .to_string();
        let two = validate_partition(&groups, &config, &infos, &edges)
            .expect_err("refused")
            .to_string();
        assert_eq!(one, two, "the refusal text must not vary run to run");
        // And the tail really is exercised, or this arm pins determinism of a
        // string with no variable part.
        assert!(
            one.contains("(and 1 more split `block`-topic edge(s)"),
            "the fixture must produce a tail: {one}"
        );
        assert!(
            one.matches("has 2 in-graph producers").count() >= 2,
            "each line renders its OWN bar, headline and tail alike: {one}"
        );
    }

    /// An UNPLACED node is a structural fault the group checks already own —
    /// inventing a co-location verdict for it would re-label somebody else's
    /// error, so the walk `continue`s.
    ///
    /// The `continue` was unpinned: nothing reached it, so deleting it (and
    /// panicking on the `None`, or fabricating a verdict) was invisible.
    #[test]
    fn an_unplaced_node_is_not_reported_as_a_colocation_violation() {
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("cons", vec![inp("gate", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("cons", "/p/prod/cmd");
        // `cons` is in NO group. `validate_partition` refuses this for its own
        // structural reason; the point is that the message is the STRUCTURAL
        // one, not a fabricated co-location verdict about an edge whose
        // consumer has no group to be split from.
        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert("front".to_string(), vec!["prod".to_string()]);
        let msg = validate_partition(&groups, &config, &infos, &edges)
            .expect_err("an unplaced node is refused")
            .to_string();
        assert!(
            !msg.contains("SPLITS a `block` edge"),
            "an unplaced node must not be reported as a co-location violation: {msg}"
        );
        // The direct half: the walk itself yields no violation for it.
        let (topo, _) = build_global(&config, &infos, &edges).expect("topology");
        assert!(
            validate_block_colocation(&groups, &topo).verdict.is_ok(),
            "the co-location walk skips an unplaced node rather than judging it"
        );
    }

    /// The SAME partition with the block edge co-located passes — the refusal
    /// is about the split, not about `block`.
    #[test]
    fn a_hand_written_partition_that_co_locates_the_block_edge_passes() {
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("cons", vec![inp("gate", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("cons", "/p/prod/cmd");
        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert(
            "one".to_string(),
            vec!["prod".to_string(), "cons".to_string()],
        );
        validate_partition(&groups, &config, &infos, &edges).expect("co-located ⇒ valid");
    }

    /// The refusal predicate MUST match the seed's: a MIXED block topic split
    /// across groups is refused too, because the gate that kills the worker is
    /// per-consumer and never sees the mixedness.
    #[test]
    fn a_split_mixed_block_topic_is_refused_too() {
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("blocker", vec![inp("gate", "prod/cmd")], vec![]),
            node("lossy", vec![inp("watch", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "blocker".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        infos.insert(
            "lossy".to_string(),
            NodeInfo::with_meta(
                vec![meta("watch", true, BackpressurePolicy::DropOldest)],
                vec![],
            ),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("blocker", "/p/prod/cmd");
        edges.insert("lossy", "/p/prod/cmd");
        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert(
            "front".to_string(),
            vec!["prod".to_string(), "lossy".to_string()],
        );
        groups.insert("back".to_string(), vec!["blocker".to_string()]);
        let err = validate_partition(&groups, &config, &infos, &edges)
            .expect_err("a split MIXED block topic must be refused too");
        let msg = err.to_string();
        assert!(msg.contains("blocker.gate"), "got: {msg}");
        assert!(
            msg.contains("SPLITS a `block` edge across process groups"),
            "the ROOT fault here is the split block edge, so that is the diagnosis; got: {msg}"
        );
    }

    /// A partition that keeps the `block` edge together but splits the MIXED
    /// topic's NON-`block` sibling is refused too — and with its OWN diagnosis.
    ///
    /// This shape BUILDS on every worker, which is exactly why it needs the
    /// refusal: the worker holding `{prod, blocker}` sees consumers =
    /// `[blocker(Block)]`, reads `is_all_block()` TRUE, and INSTALLS the defer
    /// mirror, while the monolith sees the mixed flow and degrades to
    /// `drop_oldest` with a warn. The mixed-topic warn fires in NEITHER process
    /// (the other worker has no `block` consumer at all), so one graph runs two
    /// semantics silently and `lossy` is throttled to `blocker`'s drain rate.
    #[test]
    fn a_split_mixed_sibling_is_refused_with_its_own_diagnosis() {
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("blocker", vec![inp("gate", "prod/cmd")], vec![]),
            node("lossy", vec![inp("watch", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "blocker".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        infos.insert(
            "lossy".to_string(),
            NodeInfo::with_meta(
                vec![meta("watch", true, BackpressurePolicy::DropOldest)],
                vec![],
            ),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("blocker", "/p/prod/cmd");
        edges.insert("lossy", "/p/prod/cmd");
        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert(
            "front".to_string(),
            vec!["prod".to_string(), "blocker".to_string()],
        );
        groups.insert("back".to_string(), vec!["lossy".to_string()]);
        let err = validate_partition(&groups, &config, &infos, &edges)
            .expect_err("a split MIXED sibling must be refused");
        let msg = err.to_string();
        for needle in [
            "/p/prod/cmd",
            "lossy.watch",
            "blocker.gate",
            "'front'",
            "'back'",
            REMEDY_CO_LOCATE,
            REMEDY_SINGLE_PROCESS,
        ] {
            assert!(
                msg.contains(needle),
                "the refusal must name `{needle}`; got: {msg}"
            );
        }
        assert!(
            msg.contains("INSTALLS the defer"),
            "it must name the DIVERGENCE (a worker installing what the monolith degrades); \
             got: {msg}"
        );
        assert!(
            msg.contains("in NEITHER process"),
            "it must say the degrade warn fires nowhere, which is why this is silent; got: {msg}"
        );
        // The DISCRIMINATOR: every worker here BUILDS, so borrowing the split
        // block edge's text would send the operator hunting a crash that never
        // happens.
        assert!(
            !msg.contains("would refuse to build"),
            "a split sibling builds fine — it must not claim a build death; got: {msg}"
        );
        // THE REMEDY DIRECTION. 'front' already holds the producer AND the
        // `block` consumer; 'back' holds only the sibling. So the fix is to
        // move the SIBLING — telling the operator to move the PRODUCER instead
        // splits the `block` edge that is currently intact and the partition is
        // refused again on the next validation. A remedy that leaves the user
        // no better off is the misleading-surface class, not a nit.
        assert!(
            msg.contains("move 'lossy' into group 'front'"),
            "the remedy must move the SIBLING into the producer's group; got: {msg}"
        );
        assert!(
            !msg.contains("move 'prod' into group 'back'"),
            "the remedy must NOT offer to move the producer into the sibling's group — that \
             SPLITS the intact `block` edge; got: {msg}"
        );
        assert!(
            msg.contains("Do NOT move 'prod' into 'back'"),
            "it must say WHY the other direction is wrong, not merely omit it; got: {msg}"
        );
    }

    /// A split `block` edge is the HEADLINE wherever it is found — including on
    /// a LATER topic than a mixed-sibling violation.
    ///
    /// Block-before-sibling ordering inside one seed is not enough: the mixed
    /// topic here is enumerated FIRST (asserted, so this arm cannot go vacuous
    /// if topology order ever changes), and its diagnosis claims every worker
    /// builds — while `blocker_z`'s worker, split from its producer, dies at
    /// build. The refusal must report the arm the operator can act on, and
    /// still list both.
    ///
    /// Credit exemption (C5) RE-TARGET: the later topic must now be MULTI-PRODUCER,
    /// or its split edge is credited and there is no `BlockEdge` violation
    /// left to headline. The ordering claim under test is unchanged.
    #[test]
    fn a_split_block_edge_outranks_an_earlier_topics_mixed_sibling_as_the_headline() {
        let config = multi_publisher_listed_config_of(
            vec![
                node("prod", vec![], vec![out("aaa"), out_at("zzz", "/zzz")]),
                node("blocker_a", vec![inp("gate", "prod/aaa")], vec![]),
                node("lossy_a", vec![inp("watch", "prod/aaa")], vec![]),
                node("prod_z2", vec![], vec![out_at("zzz", "/zzz")]),
                node("blocker_z", vec![inp("gate", "/zzz")], vec![]),
            ],
            &["/zzz"],
        );
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        for p in ["prod", "prod_z2"] {
            infos.insert(p.to_string(), NodeInfo::with_meta(vec![], vec![]));
        }
        for b in ["blocker_a", "blocker_z"] {
            infos.insert(
                b.to_string(),
                NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
            );
        }
        infos.insert(
            "lossy_a".to_string(),
            NodeInfo::with_meta(
                vec![meta("watch", true, BackpressurePolicy::DropOldest)],
                vec![],
            ),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("blocker_a", "/p/prod/aaa");
        edges.insert("lossy_a", "/p/prod/aaa");
        edges.insert("blocker_z", "/zzz");

        // PRECONDITION, asserted rather than assumed: the MIXED topic really is
        // enumerated first, so a per-seed-only ordering really would headline
        // the sibling.
        let (topo, _) = build_global(&config, &infos, &edges).expect("topology");
        let seeds = block_colocation_seeds(&topo);
        assert_eq!(
            seeds.iter().map(|s| s.topic.as_str()).collect::<Vec<_>>(),
            vec!["/p/prod/aaa", "/zzz"],
            "this arm needs the MIXED topic enumerated FIRST or it proves nothing"
        );
        // SECOND precondition (credit exemption): the later edge must be one the
        // credit exemption REJECTS, or this arm silently becomes a test that
        // no violation is reported at all.
        assert!(
            seeds[1].credit_bar().is_some(),
            "the headline edge must be uncreditable or there is nothing to headline"
        );

        // front holds the mixed topic's block edge INTACT (prod + blocker_a);
        // back splits the OTHER topic's block edge and holds the sibling.
        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert(
            "front".to_string(),
            vec![
                "prod".to_string(),
                "blocker_a".to_string(),
                "prod_z2".to_string(),
            ],
        );
        groups.insert(
            "back".to_string(),
            vec!["lossy_a".to_string(), "blocker_z".to_string()],
        );
        let msg = validate_partition(&groups, &config, &infos, &edges)
            .expect_err("both violations must refuse")
            .to_string();
        // `to_string()` prefixes the error kind, so match the headline clause
        // rather than the start of the string — and pair it with the ABSENCE of
        // the sibling headline, which is the real discriminator (the sibling's
        // BULLET is expected below; only its HEADLINE must be gone).
        assert!(
            msg.contains("this partition SPLITS a `block` edge across process groups"),
            "the BLOCK EDGE is the headline even though the mixed topic sorts first; got: {msg}"
        );
        assert!(
            !msg.contains("SPLITS a MIXED"),
            "the mixed sibling must NOT headline — it sorts first but every worker builds \
             under it, while the later split `block` edge kills one; got: {msg}"
        );
        assert!(
            msg.contains("would refuse to build"),
            "the headline must be the build-death diagnosis; got: {msg}"
        );
        // Both survive, and the headline's own bullet leads.
        assert!(
            msg.contains("blocker_z.gate") && msg.contains("lossy_a.watch"),
            "every violation must still be listed; got: {msg}"
        );
        // TWO more, not one: `/zzz` now has two producers (the C5 re-target),
        // and the inner loop records one violation PER producer — so the tail
        // carries the second producer's split edge as well as the sibling.
        assert!(
            msg.contains("(and 2 more split `block`-topic edge(s)"),
            "the second producer's edge and the sibling must both ride the 'more' list; \
             got: {msg}"
        );
        let head = msg.split("(and 2 more").next().expect("head");
        assert!(
            head.contains("blocker_z.gate") && !head.contains("lossy_a.watch"),
            "the headline's own bullet must be the BLOCK EDGE's line; got: {msg}"
        );
    }

    /// ANTI-TAUTOLOGY for the arm above: the SAME graph with the whole flow in
    /// one group passes. Without it, "a mixed topic is refused" is satisfied by
    /// a validator that refuses every mixed topic however it is grouped.
    #[test]
    fn a_mixed_block_topic_whose_whole_flow_is_co_located_passes() {
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("blocker", vec![inp("gate", "prod/cmd")], vec![]),
            node("lossy", vec![inp("watch", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "blocker".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        infos.insert(
            "lossy".to_string(),
            NodeInfo::with_meta(
                vec![meta("watch", true, BackpressurePolicy::DropOldest)],
                vec![],
            ),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("blocker", "/p/prod/cmd");
        edges.insert("lossy", "/p/prod/cmd");
        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert(
            "one".to_string(),
            vec![
                "prod".to_string(),
                "blocker".to_string(),
                "lossy".to_string(),
            ],
        );
        validate_partition(&groups, &config, &infos, &edges)
            .expect("the whole flow co-located ⇒ valid");
    }

    /// The seed carries the WHOLE flow, split by policy — the field-level
    /// oracle behind both the derivation and the refusal.
    #[test]
    fn a_mixed_flows_seed_carries_its_block_consumers_and_its_siblings_apart() {
        let config = config_of(vec![
            node("prod", vec![], vec![out("cmd")]),
            node("blocker", vec![inp("gate", "prod/cmd")], vec![]),
            node("lossy", vec![inp("watch", "prod/cmd")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "blocker".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        infos.insert(
            "lossy".to_string(),
            NodeInfo::with_meta(
                vec![meta("watch", true, BackpressurePolicy::DropOldest)],
                vec![],
            ),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("blocker", "/p/prod/cmd");
        edges.insert("lossy", "/p/prod/cmd");
        let (topo, _) = build_global(&config, &infos, &edges).expect("topology");
        let seeds = block_colocation_seeds(&topo);
        assert_eq!(seeds.len(), 1, "one block-carrying topic; got {seeds:?}");
        assert_eq!(seeds[0].topic, "/p/prod/cmd");
        assert_eq!(seeds[0].producers, vec!["prod".to_string()]);
        assert_eq!(
            seeds[0].block_consumers,
            vec![("blocker".to_string(), "gate".to_string())]
        );
        assert_eq!(
            seeds[0].other_consumers,
            vec![("lossy".to_string(), "watch".to_string())],
            "the non-`block` sibling rides the seed — it is what makes the worker's local \
             mixedness equal the graph's"
        );
    }

    /// A genuinely EXTERNAL `block` topic (no in-graph producer) is SKIPPED:
    /// it is already refused at build with its own specific message,
    /// identically under `--single-process`, so a plan-time double carrying a
    /// partition remedy would only mislead.
    #[test]
    fn a_producer_less_block_topic_is_not_a_partition_refusal() {
        let config = config_of(vec![
            node("cons", vec![inp("gate", "/outside/feed")], vec![]),
            node("spare", vec![], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert(
            "cons".to_string(),
            NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
        );
        infos.insert("spare".to_string(), NodeInfo::with_meta(vec![], vec![]));
        let edges = TriggerEdges::new();
        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert("g0".to_string(), vec!["cons".to_string()]);
        groups.insert("g1".to_string(), vec!["spare".to_string()]);
        validate_partition(&groups, &config, &infos, &edges)
            .expect("an external block topic is somebody else's refusal");
        // And the seed likewise declines to constrain it.
        let topo = GraphTopology::build(&config, &infos).expect("topology");
        assert!(block_colocation_seeds(&topo).is_empty());
    }

    /// EVERY partition the derivation emits passes the refusal it shares a
    /// predicate with. A refusal predicate narrower than the seed's would
    /// refuse partitions the derivation itself produces; a wider one would let
    /// a shape the derivation declines to co-locate die in the worker.
    #[test]
    fn a_derived_partition_always_passes_its_own_block_colocation_check() {
        let (config, infos, edges) = bridged_block_graph();
        let groups = baseline_process_per_node(&config, &infos, &edges).expect("derive");
        validate_partition(&groups, &config, &infos, &edges).expect("derived ⇒ valid");
    }

    // ---- the branches nothing reached -----------------------------------

    /// The MULTI-violation branch of the refusal. Every other refusal arm in
    /// the tree constructs exactly ONE violation, so mangling or dropping
    /// `violations[1..]` was invisible.
    ///
    /// Credit exemption (C5) RE-TARGET: BOTH topics are now multi-producer. With
    /// single producers both edges are credited, the partition is accepted,
    /// and the multi-violation branch is unreachable — the arm would pass
    /// while testing nothing.
    #[test]
    fn a_partition_that_splits_two_block_edges_names_both() {
        let config = multi_publisher_listed_config_of(
            vec![
                node(
                    "prod",
                    vec![],
                    vec![out_at("cmd", "/cmd"), out_at("aux", "/aux")],
                ),
                node(
                    "prod2",
                    vec![],
                    vec![out_at("cmd", "/cmd"), out_at("aux", "/aux")],
                ),
                node("c1", vec![inp("gate", "/cmd")], vec![]),
                node("c2", vec![inp("gate", "/aux")], vec![]),
            ],
            &["/cmd", "/aux"],
        );
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        for p in ["prod", "prod2"] {
            infos.insert(p.to_string(), NodeInfo::with_meta(vec![], vec![]));
        }
        for c in ["c1", "c2"] {
            infos.insert(
                c.to_string(),
                NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
            );
        }
        let mut edges = TriggerEdges::new();
        edges.insert("c1", "/cmd");
        edges.insert("c2", "/aux");
        let mut groups: IndexMap<String, Vec<String>> = IndexMap::new();
        groups.insert(
            "front".to_string(),
            vec!["prod".to_string(), "prod2".to_string()],
        );
        groups.insert("back".to_string(), vec!["c1".to_string(), "c2".to_string()]);
        let err = validate_partition(&groups, &config, &infos, &edges)
            .expect_err("two split MULTI-PRODUCER block edges must be refused");
        let msg = err.to_string();
        // Two topics x two producers = four violations, so THREE ride the
        // tail. The point of the arm is that `violations[1..]` renders at all.
        assert!(
            msg.contains("(and 3 more split `block`-topic edge(s)"),
            "the refusal must count the rest; got: {msg}"
        );
        for needle in ["c1.gate", "c2.gate", "/cmd", "/aux"] {
            assert!(
                msg.contains(needle),
                "BOTH violations must be named — missing `{needle}`; got: {msg}"
            );
        }
    }

    /// The `GroupCheck::NonContiguous` absorption arm — the repair branch the
    /// bridged arm cannot reach.
    ///
    /// `a` drives BOTH `b` and `c` (so `{a, c}` carries a DIRECT in-group
    /// trigger edge and the per-member bijection PASSES), while `b` pushes `c`
    /// to global level 2. The seeded `{a, c}` therefore owns a non-contiguous
    /// band `{0, 2}` — which is the shape the cross-process barrier's
    /// contiguous-split participant map cannot represent — and `b` occupies
    /// the gap.
    #[tracing_test::traced_test]
    #[test]
    fn a_non_contiguous_block_group_absorbs_the_gap_instead_of_refusing() {
        let config = config_of(vec![
            node("a", vec![], vec![out("cmd"), out("t")]),
            node("b", vec![inp("trig", "a/t")], vec![out("mid")]),
            node(
                "c",
                vec![
                    inp("trig1", "a/t"),
                    inp("trig2", "b/mid"),
                    inp("gate", "a/cmd"),
                ],
                vec![],
            ),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("a".to_string(), NodeInfo::with_meta(vec![], vec![]));
        infos.insert(
            "b".to_string(),
            NodeInfo::with_meta(
                vec![meta("trig", true, BackpressurePolicy::DropOldest)],
                vec![],
            ),
        );
        infos.insert(
            "c".to_string(),
            NodeInfo::with_meta(
                vec![
                    meta("trig1", true, BackpressurePolicy::DropOldest),
                    meta("trig2", true, BackpressurePolicy::DropOldest),
                    meta("gate", false, BackpressurePolicy::Block),
                ],
                vec![],
            ),
        );
        let mut edges = TriggerEdges::new();
        edges.insert("b", "/p/a/t");
        edges.insert("c", "/p/a/t");
        edges.insert("c", "/p/b/mid");

        let groups = baseline_process_per_node(&config, &infos, &edges).expect("derive");
        assert_eq!(
            shape(&groups),
            vec![(
                "grp_a".to_string(),
                vec!["a".to_string(), "b".to_string(), "c".to_string()]
            )],
            "the gap node must be ABSORBED, not refused"
        );
        validate_partition(&groups, &config, &infos, &edges)
            .expect("the repaired partition must be spawner-consumable");
        assert_warn_line!(
            "occupies a gap inside",
            "a gap absorption must be announced at the level the partition verb prints"
        );
    }

    /// A TRANSITIVE block chain (`a -block-> b`, `b -block-> c`) is TWO seeds
    /// sharing node `b`, so their anchors are two DIFFERENT indices resolving
    /// to ONE union-find root. Deduping by INDEX left both standing and the
    /// over-budget reporting loop announced the identical override TWICE for
    /// one group — against the loud-ONCE intent.
    #[tracing_test::traced_test]
    #[test]
    fn a_transitive_block_chain_announces_its_over_budget_group_once() {
        let config = config_of(vec![
            node("a", vec![], vec![out("cmd")]),
            node("b", vec![inp("gate", "a/cmd")], vec![out("mid")]),
            node("c", vec![inp("gate", "b/mid")], vec![]),
        ]);
        let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
        infos.insert("a".to_string(), NodeInfo::with_meta(vec![], vec![]));
        for n in ["b", "c"] {
            infos.insert(
                n.to_string(),
                NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
            );
        }
        let mut edges = TriggerEdges::new();
        edges.insert("b", "/p/a/cmd");
        edges.insert("c", "/p/b/mid");

        let costs = PartitionCosts {
            node_p50_ns: [
                ("a".to_string(), 900u64),
                ("b".to_string(), 10u64),
                ("c".to_string(), 10u64),
            ]
            .into_iter()
            .collect(),
            edge_rate_mhz: BTreeMap::new(),
            hop: HopCosts::platform_default(),
        };
        let out =
            auto_partition(&config, &infos, &edges, &costs, &BTreeSet::new(), 100).expect("derive");
        assert_eq!(
            shape(&out.groups),
            vec![(
                "grp_a".to_string(),
                vec!["a".to_string(), "b".to_string(), "c".to_string()]
            )],
            "a transitive block chain is ONE group"
        );
        logs_assert(|lines: &[&str]| {
            let hits = lines
                .iter()
                .filter(|l| l.contains("exceeds the per-group compute budget"))
                .count();
            if hits == 1 {
                Ok(())
            } else {
                Err(format!(
                    "ONE group must announce its budget override ONCE; got {hits} lines: \
                     {lines:#?}"
                ))
            }
        });
    }
}
