// SPDX-License-Identifier: AGPL-3.0-only
//! iceoryx2 stale-resource cleanup helpers.
//!
//! The problem this module solves (surfaced in review): a
//! blanket `remove_dir_all("/tmp/iceoryx2")` corrupts the state of
//! any other live Cerulion process that happens to be running
//! alongside (e.g. a perception graph in one terminal and a logging
//! graph in another). iceoryx2 already exposes a smart cleanup:
//! [`Node::try_cleanup_dead_nodes`](iceoryx2::node::Node::try_cleanup_dead_nodes)
//! (renamed from `cleanup_dead_nodes` in iceoryx2 0.9)
//! enumerates the registry, identifies entries whose owning
//! processes have died (the
//! [`iceoryx2::node::NodeState::Dead`] variant),
//! and removes only their stale resources. Live nodes — and
//! `Inaccessible` / `Undefined` nodes — are left untouched.
//!
//! This module wraps that API for use from the three consumers:
//! - The graceful-EXIT pass ([`run_exit_hygiene`], armed by
//!   [`ExitHygieneGuard`] in `graph_run`). Same machinery as the two below,
//!   run at the one moment where hygiene delays no work.
//! - The explicit `cerulion clean` command in
//!   `crates/cerulion_cli/src/main.rs`. Replaced the prior per-entry
//!   `/tmp/iceoryx2/{services,nodes}/*` removal walk that wiped
//!   live and dead state alike.
//! - The implicit cleanup at the top of `graph_run` (in
//!   `crates/cerulion_cli_engine/src/graph_cmd.rs`). Replaced the prior
//!   `remove_dir_all("/tmp/iceoryx2")` blanket wipe that
//!   clobbered every sibling Cerulion process on the host.
//!
//! `graph_run` continues to call this explicitly even though
//! iceoryx2's `NodeBuilder::create` ALSO runs the same sweep when
//! `cleanup_dead_nodes_on_creation: true` is set (the iceoryx2
//! default — see `iceoryx2-0.8.1/src/config.rs:189-204`). The
//! explicit call is the load-bearing line: it keeps the dead-node
//! sweep happening regardless of any user-supplied
//! `iceoryx2.toml` that flips the on-creation default to `false`.
//! The double-call is correct: a second sweep on a freshly-cleaned
//! registry finds no `Dead` entries and is a directory walk plus
//! constant work.
//!
//! # Why the STARTUP sweep is BOUNDED and `cerulion clean` is not
//!
//! The sweep's cost is `O(dead nodes)` times the cost of removing ONE dead
//! node's stale resources — and that inner cost is NOT constant. It walks the
//! dead node's services, and for each one iceoryx2 lists the shared-memory
//! namespace (`stale_resource_cleanup::connections` →
//! `Connection::list_cfg` → `SharedMemory::list` → the PAL's `shm_list()`).
//!
//! On **Linux** `shm_list()` reads `/dev/shm`, which contains shared memory and
//! nothing else. On **macOS** there is no such filesystem, so
//! `iceoryx2-pal-posix` keeps one `<name>.shm_state` file per segment in
//! `TEMP_DIRECTORY` — literally `/tmp/` — and `shm_list()` is a full `readdir`
//! of `/tmp`. A segment whose owner was SIGKILLed leaks its `.shm_state` file,
//! so on a long-lived desk that directory grows without bound and every listing
//! grows with it.
//!
//! MEASURED on a development machine (2026-08-22): `/tmp` held **73,290 entries,
//! 72,707 of them `.shm_state`**, and one full scan took **393 s** under load
//! (13 s quiet). After removing the stale files: **898 entries, 0.00 s**. The
//! same `cerulion_cli` test binary went from `1 of 8` arms passing in 668 s to
//! **8 of 8 in 51 s**, with the failures reported as
//! `bagd never created the bag` and EMPTY stdout AND stderr — the supervisor
//! had not reached its first log line, because it was still in this sweep.
//!
//! So an unbounded sweep at the top of `graph_run` can silently delay step 0 by
//! minutes on a host whose only sin is having run a lot of graphs. That is a
//! ROBOT-facing failure, not a test-harness one: `/tmp` on a robot accumulates
//! the same way, and a crash-looping node is exactly the shape that fills it.
//! [`cleanup_dead_iceoryx2_nodes_bounded`] therefore gives the startup sweep a
//! wall budget and REPORTS what it left behind; `cerulion clean` — the verb
//! whose entire job is the sweep — keeps the unbounded
//! [`cleanup_dead_iceoryx2_nodes`].
//!
//! **Scope of the bound**, in two parts, because a reader who takes
//! "startup is bounded" at face value would be wrong twice.
//!
//! 1. **Per attempt.** The budget is consulted BEFORE each dead node and
//!    `blocking_remove_stale_resources` has no cancellation, so the guarantee is
//!    `wall <= budget + one attempt` — it stops a run from paying a pathological
//!    per-node cost `N` times, it cannot make one attempt cheap. Nothing is lost
//!    by deferring: a dead node is still dead next time, and the next
//!    `graph run` (or `cerulion clean`) picks it up.
//!
//! 2. **This is not the only sweep a `graph run` pays.**
//!    `cerulion_core::transport`'s `startup_dead_node_sweep` calls the same
//!    unbounded `try_cleanup_dead_nodes` once per process at
//!    `TransportManager::init` (crash recovery was restored there after
//!    disabling iceoryx2's automatic reaping, which is why the on-creation sweep
//!    is NOT a third copy — it is switched off). That call runs in every graph
//!    process, every `run-worker`, `bagd` and `netd`, it is unbounded, and
//!    bounding it is a change to `cerulion_core` rather than to this module. So
//!    on a host pathological enough to exhaust the budget here, startup can
//!    still stall there — the budget below covers the sweep `graph_run` pays
//!    FIRST (measured to be the one stalling: a `sample` of a wedged supervisor
//!    showed the whole stack inside `cleanup_dead_iceoryx2_nodes`), not every
//!    sweep in the process.

use core::time::Duration;

use cerulion_core::iceoryx_logger::CapturedLog;
use iceoryx2::config::Config;
use iceoryx2::node::{CleanupState, NodeBuilder};

// Route stale-resource cleanup through the same
// Service type the core transport now uses (`ipc_threadsafe::Service`). The
// Service type parameter is purely an in-process port-wrapping concern
// (`ArcThreadSafetyPolicy`: SingleThreaded vs MutexProtected) — the on-SHM
// service identity (`ServiceHash`) and the node registry walked by
// `try_cleanup_dead_nodes` are identical across both variants, so this matches
// core for consistency rather than out of cross-process necessity.
use iceoryx2::service::ipc_threadsafe::Service as CerService;

/// Per-failure diagnostic for `cerulion clean` reporting.
///
/// Built from iceoryx2's per-failure trace messages (captured via
/// the [`cerulion_core::iceoryx_logger`] bridge) to give the user
/// actionable feedback about WHY a cleanup failed, not just that
/// it did. iceoryx2's `CleanupState` only carries flat counts;
/// the per-cause breakdown comes from parsing the trace strings.
#[derive(Debug, Clone)]
pub struct CleanupReport {
    /// Number of dead nodes successfully cleaned (mirrors
    /// iceoryx2's `CleanupState::cleanups`, which is `u64` as of
    /// iceoryx2 0.9).
    pub cleanups: u64,
    /// Number of cleanup attempts that failed (mirrors
    /// iceoryx2's `CleanupState::failed_cleanups`, `u64` as of
    /// iceoryx2 0.9).
    pub failed_cleanups: u64,
    /// Cause-classified breakdown of `failed_cleanups`. Keys are
    /// human-readable cause categories (`"permission denied"`,
    /// `"version mismatch"`, etc.). Values are counts. The sum
    /// of values may be less than `failed_cleanups` if iceoryx2
    /// emits a failure cause that doesn't match a known
    /// pattern; in that case the unmatched count is in
    /// `unclassified`.
    pub failures_by_cause: std::collections::BTreeMap<String, usize>,
    /// Failures whose cause didn't match any known pattern.
    /// Carries the raw iceoryx2 message excerpts so the user
    /// can inspect them directly.
    pub unclassified: Vec<String>,
    /// One entry per dead node iceoryx2 REFUSED to remove, carrying
    /// the node's identity, the `NodeCleanupFailure` variant, and every
    /// sub-cause line iceoryx2 emitted about that node on the way to the
    /// refusal. `failures_by_cause` answers "how many, of what kind";
    /// this answers "WHICH node, and WHY" — the question a nightly canary
    /// that finds exactly one permanently-stranded node has to answer
    /// before anyone can attribute it to a suite. See
    /// [`classify_cleanup_failures`] for the attribution rule.
    pub failures: Vec<FailedNodeCleanup>,
    /// Every line the sweep logged about the REGISTRY WALK ITSELF
    /// rather than about one node — the scan could not list the registry, or
    /// aborted part-way (`is_registry_wide_line` enumerates the exact 0.9.1
    /// shapes). Verbatim, in emission order. Such a line is NEVER a node's
    /// cause: it names no node, and filing it under whichever refusal
    /// happens to follow would render a GLOBAL failure as that node's fault
    /// — the wrong-culprit class this report exists to prevent. Non-empty
    /// also means the counters above are a LOWER bound: a dead node the walk
    /// never reached is still registered and appears in neither.
    pub registry_errors: Vec<String>,
}

/// One dead node iceoryx2 could not remove, with the sub-causes it logged
/// on the way.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FailedNodeCleanup {
    /// The node identity exactly as iceoryx2 rendered it in the
    /// `Unable to remove dead node {id:?} (…)` line — for 0.9.1 that is
    /// `UniqueNodeId(UniqueSystemId { value: …, pid: …, creation_time: … })`.
    /// Kept verbatim: it is the token every other line about the node
    /// carries, and shortening it here would make a reader's grep against
    /// a raw trace miss.
    pub node: String,
    /// The `NodeCleanupFailure` variant name (`InternalError`,
    /// `InsufficientPermissions`, …). Empty when the line did not carry a
    /// parenthesised variant at all — never fabricated.
    pub variant: String,
    /// Every captured sub-cause message attributed to this node, in
    /// emission order. Empty when iceoryx2 gave the refusal no
    /// explanation (or the bridge captured none).
    pub causes: Vec<String>,
}

/// The report parts derived from the captured iceoryx2 log lines, kept
/// apart from the [`CleanupState`] counters so the derivation is a PURE
/// function of the capture and oracle-testable without a registry.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ClassifiedFailures {
    /// See [`CleanupReport::failures_by_cause`].
    pub failures_by_cause: std::collections::BTreeMap<String, usize>,
    /// See [`CleanupReport::unclassified`].
    pub unclassified: Vec<String>,
    /// See [`CleanupReport::failures`].
    pub failures: Vec<FailedNodeCleanup>,
    /// See [`CleanupReport::registry_errors`].
    pub registry_errors: Vec<String>,
}

/// The line iceoryx2 emits once per dead node it could not remove
/// (`iceoryx2-0.9.1/src/node/mod.rs:1231`):
/// `Unable to remove dead node {node_id:?} ({failure:?}).`
const FAILURE_LINE_MARKER: &str = "Unable to remove dead node";

/// The line the sweep emits once per dead node it DID remove
/// (`iceoryx2-0.9.1/src/node/mod.rs:1227`). A block boundary for the
/// adjacency rule: sub-causes logged before it belonged to that node's
/// attempt, not to whatever fails next.
const SUCCESS_LINE_MARKER: &str = "was successfully removed";

/// The message heads of every line the 0.9.1 dead-node sweep can emit that
/// is scoped to the REGISTRY WALK rather than to one node — read off
/// `iceoryx2-0.9.1/src/node/mod.rs`: the sweep entry `blocking_cleanup_dead_nodes`
/// (`:1210-1245`, everything outside its per-node `cleanup_call`), `Node::list`
/// (`:1116-1150`) and `list_all_nodes` (`:1250-1269`). Five lines, three heads,
/// each interpolating only a `NodeListFailure` variant — never a node:
///
/// - `:1242`, origin `Node::<…>::cleanup_dead_nodes()`: `Unable to perform a
///   full scan for dead nodes since the all existing nodes could not be listed
///   ({e:?}).` — logged AFTER `Node::list` returned `Err`.
/// - `:1145`, origin `Node::list()`: `Unable to iterate over Node list since
///   the node list could not be acquired ({e:?}).` — `list_all_nodes` failed
///   BEFORE the per-node loop; no node was visited.
/// - `:1138`, origin `Node::list()`: `Unable to iterate over Node list since
///   the following error occurred ({e:?}).` — `NodeState::new` failed for a
///   visited node and the walk ABORTED there. Only `Interrupt` reaches it:
///   `NodeState::new` (`:409-413`) maps `InsufficientPermissions` to
///   `Inaccessible` and `InternalError` to `Undefined` and keeps walking.
/// - `:1261` / `:1265`, origin `Node::list_all_nodes({config:?})` (a `Config`
///   rendering, no node): `Unable to list all nodes due to insufficient
///   permissions while listing all nodes.` / `… due to an internal failure
///   while listing all nodes.`
///
/// `Unable to list all nodes` (plural, then ` due`) is NOT a prefix of the
/// per-node `Unable to list all node detail storages …` (`:783-792`), so the
/// third head cannot claim that line. Everything else the sweep logs is
/// per-node: `DeadNodeView`'s `from self` lines, `remove_node` /
/// `remove_node_details_directory` (`:825`/`:845`, token in the origin), the
/// `service_tags`/`port_tags` string origins (`:1401`/`:1434`, emitted inside
/// the node's block), `service/stale_resource_cleanup.rs` (port origins), and
/// the walk lines `get_node_state`/`open_node_storage`/`get_node_details`
/// (`:1310`/`:1345`/`:1377`, the visited node's token in the origin — the
/// foreign-token guard's business).
const REGISTRY_WIDE_LINE_HEADS: [&str; 3] = [
    "Unable to perform a full scan for dead nodes",
    "Unable to iterate over Node list",
    "Unable to list all nodes",
];

/// Is this captured line about the registry walk itself, not about one node?
/// Exactly the [`REGISTRY_WIDE_LINE_HEADS`] shapes. Such a line reads as a
/// sub-cause to [`is_sub_cause_line`] (it carries ` since ` / ` due to `) and
/// carries no node token, so without this predicate the adjacency arm would
/// hand it to whatever refusal follows.
fn is_registry_wide_line(log: &CapturedLog) -> bool {
    REGISTRY_WIDE_LINE_HEADS
        .iter()
        .any(|head| log.message.contains(head))
}

/// The origin heads of lines the registry walk logs about a VISITED node
/// WITHOUT that node's token — node-scoped, yet invisible to the id arm and
/// to the foreign-token guard alike. One head in 0.9.1:
///
/// - `Node::state_from_monitor(` (`iceoryx2-0.9.1/src/node/mod.rs:1271-1297`):
///   `ProcessMonitor::state()` failed for the node `Node::list` is
///   classifying. The origin renders the MONITOR — `ProcessMonitor {
///   state_path: …, owner_lock_path: …, context_path: … }`, three file paths
///   that name the node only by file name, no `UniqueNodeId(` span — and the
///   message is sub-cause-shaped (`Unable to acquire node state from monitor
///   due to …`). `NodeState::new` (`:395-413`) turns the failure into
///   `Inaccessible` / `Undefined` — a node that is never `Dead`, never
///   attempted, never refused — or aborts the walk (`Interrupt`, a
///   registry-wide line of its own). Either way the line can never be a
///   refused node's cause; it is token-free, so without this guard the
///   adjacency arm hands it to whatever refusal follows — the foreign-token
///   wrong-culprit class, one rendering over.
const NODE_SCOPED_TOKENLESS_ORIGIN_HEADS: [&str; 1] = ["Node::state_from_monitor("];

/// Is this line about a node the walk visited, rendered without that node's
/// token? Exactly the [`NODE_SCOPED_TOKENLESS_ORIGIN_HEADS`] origins. Such a
/// line belongs to NOBODY in the refusal listing.
fn is_node_scoped_tokenless_line(log: &CapturedLog) -> bool {
    NODE_SCOPED_TOKENLESS_ORIGIN_HEADS
        .iter()
        .any(|head| log.origin.starts_with(head))
}

/// Does this captured line carry iceoryx2 sub-cause text?
///
/// Every explanatory line on the path to a refusal is a `fail!`/`debug!`
/// shaped `"{msg} since …"` or `"{msg} due to …"` — but the word after
/// `since` is NOT always `the`. The 0.9.1 vocabulary
/// (`iceoryx2-0.9.1/src/node/mod.rs`, plus `service/stale_resource_cleanup.rs`):
///
/// - `since the …` — the resource-removal arms (`:545-736`, `:1351-1391`),
///   e.g. `… since the port tags could not be read due to an internal error.`
/// - `since another instance …` — the cleaner-lock family, which is what two
///   concurrently-sweeping cerulion processes produce (every `graph run`
///   startup sweeps, and `try_cleanup_dead_nodes` blocks for ZERO time):
///   `:598` `… since another instance is already cleaning up the dead nodes
///   resources.` (the intra-process `IN_CLEANUP_SECTION` guard), `:758` `…
///   since another instance is already cleaning up all resources.`, `:762`
///   `… since another instance has already cleaned up all resources.`
///   (`ResourcesAlreadyCleanedUp`), `:570` `… since another instance
///   requires longer than {timeout:?} to cleanup the resources.` (the
///   timeout that turns `:598`/`:758` into the
///   `AnotherInstanceIsCleaningUpTheNode` refusal).
/// - `since an …` / `since a …` — the signal arms: `:766` `… since an
///   interrupt signal was received.` (`Interrupt`), `:1318` `… since an
///   interrupt was received while acquiring the node state.`, `:1323` `…
///   since an internal failure occurred while acquiring the node state.`
/// - `since connection (…) …` — `stale_resource_cleanup.rs:106`.
/// - `due to …` — every permission / internal-error arm.
///
/// The refusal line itself (`Unable to remove dead node … (…)`) and the
/// sweep's bookkeeping lines (`Dead node (…) detected`, `… was successfully
/// removed`) carry neither word, so matching the bare ` since ` / ` due to `
/// changes nothing about what is EXCLUDED. A predicate that required
/// ` since the ` missed the whole cleaner-lock family, and the listing then
/// rendered `(no sub-cause captured …)` for a refusal whose explanation WAS
/// in the capture — an affirmatively false absence.
fn is_sub_cause_line(log: &CapturedLog) -> bool {
    !log.message.contains(FAILURE_LINE_MARKER)
        && (log.message.contains(" since ") || log.message.contains(" due to "))
}

/// The head of a node identity as iceoryx2 0.9.1 renders it under `{:?}`:
/// `UniqueNodeId` is a tuple struct over `UniqueSystemId` with a derived
/// `Debug` (`iceoryx2-0.9.1/src/identifiers.rs:163`), so every rendering —
/// the refusal line's, a `DeadNodeView` origin's, a string origin's
/// `{node_id:?}` — opens with this and closes at the paren that balances it.
const NODE_TOKEN_PREFIX: &str = "UniqueNodeId(";

/// Every node token embedded in `text`: each `UniqueNodeId(` … `)` span,
/// closed at the paren that BALANCES its opener. The inner
/// `UniqueSystemId { value, pid, creation_time: Time { … } }` carries braces
/// but no parens, so balancing is what turns `Dead node (UniqueNodeId(…))
/// detected` into the same token the refusal line prints. An unbalanced
/// prefix (a truncated line) yields nothing rather than a guessed token.
fn node_tokens(text: &str) -> impl Iterator<Item = &str> {
    text.match_indices(NODE_TOKEN_PREFIX)
        .filter_map(move |(start, _)| {
            let mut depth = 0usize;
            for (offset, ch) in text[start..].char_indices() {
                match ch {
                    '(' => depth += 1,
                    ')' => {
                        depth = depth.saturating_sub(1);
                        if depth == 0 {
                            return Some(&text[start..start + offset + ch.len_utf8()]);
                        }
                    }
                    _ => {}
                }
            }
            None
        })
}

/// Does this line name a node OTHER than `node`, in its origin or message?
/// The adjacency arm's guard: a line that carries any node token is not
/// ownerless, so it must not be handed to the next refusal by position.
fn carries_a_foreign_node_token(log: &CapturedLog, node: &str) -> bool {
    node_tokens(&log.origin)
        .chain(node_tokens(&log.message))
        .any(|token| token != node)
}

/// Split a refusal line into `(node, variant)`.
///
/// The variant is the LAST ` (…)` token — space-then-paren, because the node
/// identity before it is itself full of parens and braces
/// (`UniqueNodeId(UniqueSystemId { … })`) none of which follow a space, and
/// every `NodeCleanupFailure` variant is a unit variant, so the token
/// contains no parens of its own. A line with no such tail yields the whole
/// remainder as the node and an EMPTY variant, never a guessed one.
fn parse_failure_line(message: &str) -> Option<(String, String)> {
    let start = message.find(FAILURE_LINE_MARKER)? + FAILURE_LINE_MARKER.len();
    let rest = message[start..].trim().trim_end_matches('.').trim_end();
    let Some(open) = rest.rfind(" (").map(|sep| sep + 1) else {
        return Some((rest.to_string(), String::new()));
    };
    let Some(close) = rest[open..].find(')') else {
        return Some((rest.to_string(), String::new()));
    };
    let variant = rest[open + 1..open + close].trim();
    let node = rest[..open].trim();
    if node.is_empty() {
        // `(Variant)` with nothing before it is not a node token — keep the
        // whole remainder as the identity rather than inventing an empty one.
        return Some((rest.to_string(), String::new()));
    }
    Some((node.to_string(), variant.to_string()))
}

/// Derive the report parts from the captured iceoryx2 log lines. PURE:
/// no registry, no transport, no clock — the same capture always yields
/// the same parts, which is what lets hand-built vectors pin it.
///
/// **`failures_by_cause` and `unclassified`** are derived as the earlier
/// loop did: one refusal line per failure, classified by substring on the
/// variant name, unmatched lines pushed verbatim to `unclassified`. The
/// 0.9.1 `NodeCleanupFailure` (`iceoryx2-0.9.1/src/node/mod.rs:269-282`) has
/// exactly six variants — `Interrupt`, `InternalError`,
/// `InsufficientPermissions`, `VersionMismatch`, `ResourcesAlreadyCleanedUp`,
/// `AnotherInstanceIsCleaningUpTheNode` — and the rules classify four of
/// them: permissions, version, internal error, and (since this pass)
/// `AnotherInstanceIsCleaningUpTheNode` as **lock contention**. The legacy
/// `InCleanupSection` / `MonitoringResources` patterns name NO 0.9.1 variant
/// and can never fire against this iceoryx2; they are kept only so an older
/// bridge's lines classify the same way. `Interrupt` and
/// `ResourcesAlreadyCleanedUp` stay `unclassified` deliberately: neither is a
/// condition the remediation table has a remedy for.
///
/// **`registry_errors`** holds every registry-wide line (`is_registry_wide_line`
/// enumerates the shapes), verbatim and in order, independent of whether any
/// refusal exists — a scan that could not list the registry refuses nothing and
/// still failed.
///
/// **`failures`** is the newer addition: one entry per refusal line,
/// carrying every sub-cause line about that node emitted BEFORE it. The
/// attribution rule has two arms, in this order, and one exclusion that
/// precedes both:
///
/// 0. **Registry-wide lines belong to no node.** A line the sweep logged
///    about the walk itself (the full-scan failure, `Node::list`'s own lines,
///    `list_all_nodes`) is filed under `registry_errors` and is never a
///    cause, wherever it sits. It is token-free and sub-cause-shaped, so it is
///    exactly what the adjacency arm would otherwise claim; rendering it
///    beneath a node presents a global failure as that node's fault — the
///    sibling of the foreign-token defect below. In 0.9.1 the walk logs these
///    AFTER the refusals it managed (the scan failure is reported once `list`
///    returns) or with NO refusal at all (nothing was listed), so a
///    registry-wide line PRECEDING a refusal needs a capture spanning more
///    than one sweep or a future ordering — the rule is position-independent
///    so neither can ever re-open the hole.
/// 1. **By node id.** The sub-cause lines are `debug!(from self, …)` on the
///    `DeadNodeView`, so their `origin` is that view's Debug rendering,
///    which embeds the same `UniqueNodeId(…)` token the refusal line
///    prints. A sub-cause line whose `origin` or `message` contains a
///    refusal line's node token belongs to THAT node, wherever it sits in
///    the capture — which is what keeps two interleaved nodes apart.
/// 2. **By adjacency.** A sub-cause line carrying NO node token at all
///    (logged from a string origin deeper in the service layer, say) is
///    attributed to the next refusal line after it, provided no refusal or
///    success line lies between — those are the boundaries between one
///    node's attempt and the next. Each such line is claimed at most once.
///
/// A line that carries a node token the refusals do NOT name — a `UniqueNodeId(…)`
/// span in its `origin` or `message` (see `node_tokens`) — is attributed to
/// NOBODY: an id is evidence, adjacency is a fallback for its absence, and a
/// line that names some other node is not ownerless, it is someone else's. The
/// registry walk emits exactly such lines about nodes that are never refused:
/// `Node::list` (`mod.rs:1116`) builds a `NodeState` for EVERY registered node
/// before the sweep's callback sees it, and both `get_node_state` (`:1310`,
/// origin `Node::get_node_state({config:?}, {node_id:?})` — the
/// `InsufficientPermissions` arm that makes another user's node `Inaccessible`)
/// and `open_node_storage` (`:1345`, an unreadable ALIVE node's config storage)
/// log with the VISITED node's token in the origin. Such a line sits in the
/// capture with no boundary between it and the next refusal, so without this
/// rule the cross-user desk shape — another user's node registered beside a
/// stranded one — would file the other user's permission line under the
/// stranded node's entry: the wrong-culprit class this attribution exists to
/// prevent.
///
/// The same rule, one rendering over: a line whose ORIGIN names a visited
/// node WITHOUT a token (`is_node_scoped_tokenless_line` — `Node::state_from_monitor(…)`,
/// whose origin is the node's `ProcessMonitor` paths) is attributed to NOBODY
/// too. It is emitted while `Node::list` classifies a node that then reads
/// `Inaccessible` or `Undefined` — never `Dead`, never refused — so it can
/// never be a refused node's cause, and it is token-free, so the adjacency
/// arm would otherwise claim it for the next refusal.
#[must_use]
pub fn classify_cleanup_failures(captured: &[CapturedLog]) -> ClassifiedFailures {
    let mut parts = ClassifiedFailures::default();

    // Every node token named by a refusal line, in order. A sub-cause
    // line's owner is the first of these it carries.
    let refusals: Vec<(usize, String, String)> = captured
        .iter()
        .enumerate()
        .filter_map(|(i, log)| {
            parse_failure_line(&log.message).map(|(node, variant)| (i, node, variant))
        })
        .collect();
    let owner_of = |log: &CapturedLog| -> Option<&str> {
        refusals
            .iter()
            .map(|(_, node, _)| node.as_str())
            .find(|node| log.origin.contains(node) || log.message.contains(node))
    };
    let mut claimed = vec![false; captured.len()];

    // Arm 0: the walk's own failure lines, verbatim, whether or not any node
    // was refused. Collected apart from the per-refusal loop below so a scan
    // that listed nothing still reports WHY.
    parts.registry_errors = captured
        .iter()
        .filter(|log| is_registry_wide_line(log))
        .map(|log| log.message.clone())
        .collect();

    for (i, log) in captured.iter().enumerate() {
        // The classification half — the earlier loop plus the
        // lock-contention rule below.
        if !log.message.contains(FAILURE_LINE_MARKER) {
            continue;
        }
        let cause = if log.message.contains("Permission")
            || log.message.contains("InsufficientPermissions")
        {
            "permission denied"
        } else if log.message.contains("VersionMismatch") {
            "version mismatch"
        } else if log.message.contains("AnotherInstanceIsCleaningUpTheNode")
            || log.message.contains("InCleanupSection")
        {
            // `AnotherInstanceIsCleaningUpTheNode` is the 0.9.1 spelling of a
            // cleaner-lock refusal (`node/mod.rs:281`; raised at `:569`,
            // `:597`, `:757`) — the routine outcome of two `graph run`
            // startup sweeps racing over one dead node. `InCleanupSection`
            // names no variant in the 0.9.1 `NodeCleanupFailure`
            // (`:269-282`), so that pattern alone could never fire against
            // this iceoryx2 and every such refusal landed in `unclassified`
            // with a "see the raw lines" hint for a condition the table has
            // a remedy for. Kept as the second alternative for an older
            // bridge's lines.
            "lock contention"
        } else if log.message.contains("MonitoringResources") {
            "monitoring resource still in use"
        } else if log.message.contains("InternalError") {
            "iceoryx2 internal error"
        } else {
            parts.unclassified.push(log.message.clone());
            // Still fall through to the attribution half below: an
            // unclassified VARIANT is exactly the refusal whose sub-causes
            // a reader most needs, so it gets an entry too.
            ""
        };
        if !cause.is_empty() {
            *parts
                .failures_by_cause
                .entry(cause.to_string())
                .or_insert(0) += 1;
        }

        // The attribution half.
        let Some((_, node, variant)) = refusals.iter().find(|(idx, _, _)| *idx == i) else {
            continue;
        };
        let block_start = captured[..i]
            .iter()
            .rposition(|earlier| {
                earlier.message.contains(FAILURE_LINE_MARKER)
                    || earlier.message.contains(SUCCESS_LINE_MARKER)
            })
            .map_or(0, |b| b + 1);
        let mut causes = Vec::new();
        for (j, earlier) in captured[..i].iter().enumerate() {
            // Arm 0 sits here, ahead of both attribution arms: a registry-wide
            // line is sub-cause-shaped and token-free, so nothing below would
            // stop the adjacency arm from claiming it.
            if claimed[j] || !is_sub_cause_line(earlier) || is_registry_wide_line(earlier) {
                continue;
            }
            let attributed = match owner_of(earlier) {
                Some(owner) => owner == node,
                None => {
                    j >= block_start
                        && !carries_a_foreign_node_token(earlier, node)
                        && !is_node_scoped_tokenless_line(earlier)
                }
            };
            if attributed {
                claimed[j] = true;
                causes.push(earlier.message.clone());
            }
        }
        parts.failures.push(FailedNodeCleanup {
            node: node.clone(),
            variant: variant.clone(),
            causes,
        });
    }

    parts
}

/// Walk iceoryx2's node registry and remove the stale resources of
/// every dead node. Same behaviour as
/// [`cleanup_dead_iceoryx2_nodes`] but additionally captures
/// iceoryx2's per-failure trace messages and classifies them by
/// cause for actionable user-facing diagnostics.
///
/// Replaces the opaque
/// "Cleaned N dead iceoryx2 node(s); M cleanup(s) failed (typically
/// permission errors)" with a structured per-cause breakdown.
/// Common causes:
/// - **permission denied** — the dead node's resources are owned
///   by a different uid; user can't remove them. Run as the
///   owning user, or `chmod` the resources.
/// - **version mismatch** — the dead node's on-disk state was
///   produced by a different iceoryx2 version. Run
///   `rm -rf /tmp/iceoryx2/` to recover (the live nodes will
///   recreate their state).
/// - **lock contention** — another process is racing the same
///   cleanup. Retry; usually self-resolves on the next run.
///
/// Requires the iceoryx2 → tracing bridge to be installed (call
/// [`cerulion_core::iceoryx_logger::install_iceoryx2_tracing_bridge`]
/// at process startup). Without the bridge, iceoryx2's traces
/// go to its built-in console logger and the capture buffer
/// stays empty, so this function falls back to the flat
/// `CleanupState` counts and `unclassified` is empty.
pub fn cleanup_dead_iceoryx2_nodes_with_diagnostics() -> CleanupReport {
    cleanup_dead_iceoryx2_nodes_with_diagnostics_with_config(Config::global_config())
}

/// [`cleanup_dead_iceoryx2_nodes_with_diagnostics`] over an EXPLICIT iceoryx2
/// config instead of the global one.
///
/// The global config is what every CLI path wants; the explicit one is what
/// lets the orphan-port-tag pin (`tests/clean_orphan_port_tag_test.rs`) run the
/// SAME sweep-and-classify path over an isolated registry root, so a real
/// dead node can be minted, refused, reclaimed and re-swept without touching
/// the desk's `/tmp/iceoryx2`. Same log-level guard, same capture, same
/// classifier — the global variant is this one applied to
/// `Config::global_config()`.
pub fn cleanup_dead_iceoryx2_nodes_with_diagnostics_with_config(config: &Config) -> CleanupReport {
    use cerulion_core::iceoryx_logger::{capture_iceoryx_logs, init_iceoryx_log_level_from_env};
    use iceoryx2::prelude::{set_log_level, LogLevel};

    /// RAII guard: bumps iceoryx2's log level to Trace on
    /// construction and restores it on drop, even if the
    /// scoped call panics. `Node::try_cleanup_dead_nodes` is documented
    /// to potentially panic on a registry corrupted by a non-iceoryx2
    /// writer; without the guard, that panic would leave the global
    /// log level pinned at Trace for the rest of the process
    /// lifetime, flooding stderr with iceoryx2 internal traces.
    ///
    /// The restore is ENV-DERIVED, not a hardcoded `Error`. The old
    /// hardcoded restore meant a `cerulion graph run` under
    /// `IOX2_LOG_LEVEL=warn` silently lost the operator's level the moment the
    /// startup dead-node sweep ran, which made the env knob inert. Absent
    /// / empty env resolves to `error`, so the default is unchanged.
    struct LogLevelGuard;
    impl Drop for LogLevelGuard {
        fn drop(&mut self) {
            init_iceoryx_log_level_from_env();
        }
    }

    set_log_level(LogLevel::Trace);
    let _guard = LogLevelGuard;
    // The sweep needs a node in the namespace it is cleaning: iceoryx2 carries
    // `try_cleanup_dead_nodes` on `&Node` rather than on a bare config. That
    // node is transient and is dropped the moment the sweep returns, and it is
    // never a candidate for its own sweep (only DEAD nodes are).
    //
    // Creating it can fail, and the namespace this module is asked to clean is
    // exactly where it is most likely to: a corrupted registry refuses the
    // creation, which is the case the sweep exists for. Returning a 0/0
    // `CleanupState` there would read as "nothing was dead", so the refusal is
    // reported as a REGISTRY error instead — the field whose contract is
    // already "the scan could not run, so the counters are a lower bound".
    let sweep_config = cerulion_core::transport::dead_node_sweep::sweep_node_config(config);
    let ((state, node_refusal), captured) =
        capture_iceoryx_logs(
            || match NodeBuilder::new().config(&sweep_config).create::<CerService>() {
            Ok(node) => (node.try_cleanup_dead_nodes(), None),
            Err(e) => (
                CleanupState {
                    cleanups: 0,
                    failed_cleanups: 0,
                },
                Some(format!(
                    "Unable to perform a full scan for dead nodes since a node could not \
                     be created in the namespace being swept ({e:?})."
                )),
            ),
            },
        );
    // `_guard` restores the ENV-DERIVED level on drop at end-of-scope (or on
    // panic-unwind through `_guard`'s Drop) — `IOX2_LOG_LEVEL` if set, else
    // `error`. See the guard's own docs: a hardcoded `Error` restore here is
    // the inert-knob bug, not the contract.

    let ClassifiedFailures {
        failures_by_cause,
        unclassified,
        failures,
        mut registry_errors,
    } = classify_cleanup_failures(&captured);
    if let Some(refusal) = node_refusal {
        registry_errors.push(refusal);
    }

    CleanupReport {
        cleanups: state.cleanups,
        failed_cleanups: state.failed_cleanups,
        failures_by_cause,
        unclassified,
        failures,
        registry_errors,
    }
}

/// Walk iceoryx2's node registry and remove the stale resources of
/// every dead node. Live nodes are not touched.
///
/// Returns a [`CleanupState`] reporting how many dead-node cleanups
/// succeeded and how many failed (typically permission errors when
/// the calling process doesn't own the dead node's resources, e.g.
/// running `cerulion clean` after a different user's `cerulion
/// graph run` died on the same host).
///
/// Panic containment: iceoryx2's `Node::list` walks the
/// node-monitor directory and `unwrap()`s each filename through `from_utf8`
/// and `parse::<u128>()` (`iceoryx2-0.9.1/src/node/mod.rs:1128`). A stray or
/// foreign file in the shared namespace (a non-iceoryx2 writer, a
/// strict-prefix namespace overlap — the class the fixed-length-hex
/// prefixes killed) would panic what is a BEST-EFFORT hygiene sweep and take
/// the whole `graph run` startup down with it. The sweep is ADVISORY, so the
/// panic is caught here, surfaced as ONE loud `tracing::warn!`, and the run
/// continues with a zeroed [`CleanupState`].
///
/// `AssertUnwindSafe` is sound: on unwind we observe nothing from inside the
/// closure — everything is discarded and a zeroed state returned. NOTE: the
/// iceoryx2-INTERNAL creation-time sweep (inside `NodeBuilder::create`, run
/// when `cleanup_dead_nodes_on_creation` is set) canNOT be wrapped this way;
/// the hex-prefix fix removed its only known trigger.
///
/// No panic-injection unit test: the panic lives inside iceoryx2's listing
/// walk with no seam short of planting files in the SHARED global iceoryx2
/// namespace (which would race sibling Cerulion processes on the host);
/// per-project policy forgoes new injection infrastructure for this —
/// the wrap is pinned by review.
pub fn cleanup_dead_iceoryx2_nodes() -> CleanupState {
    // The GLOBAL config is resolved INSIDE the boundary: `Config::global_config()`
    // reads and parses `iceoryx2.toml` on first use and can panic on a corrupt
    // file, and a panic there is exactly the advisory-sweep failure this wrap
    // exists to contain (pinned by `the_global_sweep_resolves_its_config_inside_the_panic_boundary`).
    // The sweep is a node method, so this mints a transient node on the global
    // config inside the SAME panic boundary (see the diagnostics variant for
    // why the node is needed and why its refusal must not read as a clean
    // sweep). This entry point returns a bare `CleanupState` with nowhere to
    // put a refusal, so a node that cannot be created is ONE loud warning: the
    // advisory sweep did not run, and a caller that saw 0/0 must not conclude
    // the namespace was clean.
    contained_sweep(|| {
        let sweep_config =
            cerulion_core::transport::dead_node_sweep::sweep_node_config(Config::global_config());
        match NodeBuilder::new()
            .config(&sweep_config)
            .create::<CerService>()
        {
            Ok(node) => node.try_cleanup_dead_nodes(),
            Err(e) => {
                tracing::warn!(
                    error = ?e,
                    "dead-node sweep skipped: no iceoryx2 node could be created in the \
                     namespace being swept, so stale resources were neither found nor \
                     removed (this is not an empty registry). Run `cerulion clean` for \
                     the classified report, or remove the stale iceoryx2 shared-memory \
                     artifacts by hand"
                );
                CleanupState {
                    cleanups: 0,
                    failed_cleanups: 0,
                }
            }
        }
    })
}

/// The ONE panic boundary both sweep entry points share: run the sweep, and on
/// a panic surface ONE loud warning and return the zeroed advisory state.
fn contained_sweep(sweep: impl FnOnce() -> CleanupState) -> CleanupState {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(sweep)) {
        Ok(state) => state,
        Err(payload) => {
            // Extract the panic message when it is a string (the common
            // `panic!`/`unwrap` shapes); anything else gets a marker.
            let panic_msg = payload
                .downcast_ref::<&str>()
                .map(|s| (*s).to_string())
                .or_else(|| payload.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "<non-string panic payload>".to_string());
            tracing::warn!(
                panic = %panic_msg,
                "iceoryx2 dead-node sweep panicked while listing the shared namespace (likely a \
                 stray/foreign file in the iceoryx2 root) — skipping the advisory sweep; the run \
                 continues"
            );
            CleanupState {
                cleanups: 0,
                failed_cleanups: 0,
            }
        }
    }
}

// The POLICY lives in `cerulion_core::transport::dead_node_sweep` and this
// module DELEGATES to it. There are two startup sweeps in the system — this one
// and the transport's, which runs in every process that builds a
// `TransportManager` — and bounding one alone leaves the stall exactly where it
// was: a run that clears this sweep walks straight into that one. Two copies of
// one policy is how they drift, so the budget, the decision
// and the report have a single definition and both callers import it.
pub use cerulion_core::transport::dead_node_sweep::{
    run_budgeted_sweep, sweep_step, BoundedCleanup, SweepStep, STARTUP_DEAD_NODE_SWEEP_BUDGET,
};

/// The bounded sweep over the GLOBAL iceoryx2 config — what every CLI startup
/// path wants, and the one thing this module still adds over the core policy.
///
/// The core function takes its config explicitly because `cerulion_core`'s own
/// caller (`TransportManager`) has already resolved one and must sweep THAT
/// namespace, not the global one. Here there is no manager yet, so the global
/// config is the answer, and binding it once keeps every CLI call site from
/// re-deciding it.
pub fn cleanup_dead_iceoryx2_nodes_bounded(budget: Duration) -> BoundedCleanup {
    cerulion_core::transport::dead_node_sweep::cleanup_dead_nodes_bounded(
        Config::global_config(),
        budget,
    )
}

/// The TOTAL wall a graceful `cerulion graph run` teardown may spend on hygiene.
///
/// One budget across both passes, not one each. The agreed design prices the exit pass
/// at ~2 s; two independent 2 s bounds would silently make a Ctrl+C take four,
/// which is a different (and worse) deal than the one that was agreed.
pub const EXIT_HYGIENE_BUDGET: Duration = crate::shm_state::SHM_STATE_EXIT_RECLAIM_BUDGET;

/// Run the graceful-exit hygiene pass: reclaim what a dead process
/// left behind, under [`EXIT_HYGIENE_BUDGET`], and say so only if it did
/// something.
///
/// # Why exit, and why here
///
/// Both populations this touches are DEAD residue — files and registry entries
/// whose owning processes are gone — and both were reclaimed only when someone
/// typed `cerulion clean`, i.e. on the desks of people who already knew about
/// the defect. At exit the graph is over, so a bounded pass delays no work: it
/// costs the prompt and nothing else. A run that is SIGKILLed skips this by
/// nature; its residue is swept by the next run that ends gracefully.
///
/// # Order, and the shared budget
///
/// The dead-node sweep goes FIRST and the state files SECOND, and that order is
/// a CORRECTNESS constraint, not a preference — it is the same order
/// `cerulion clean` has always run in, and for the reason its own doc gives:
/// *"a reclamation running first would unlink objects the sweep was about to
/// inspect."*
///
/// On macOS a `/tmp/<name>.shm_state` file is the ONLY mapping from an
/// iceoryx2 resource name to the real POSIX object behind it
/// (`iceoryx2-pal-posix`'s `get_real_shm_name`, `macos/mman.rs`). Remove the
/// file and every later `shm_open` of that name answers `ENOENT` — so a dead
/// node whose registry entry is still standing can never be reaped again:
/// iceoryx2 cannot read its details, cannot deregister it from its services,
/// and the entry survives every future sweep. Measured, with a control, in
/// `tests/reclaim_ordering_test.rs`.
///
/// # The convergence gate
///
/// Order alone is not enough. A state file whose CREATOR is dead is exactly the
/// file a still-standing dead-node entry needs, so the reclamation runs only
/// when the sweep left the registry CONVERGED — nothing deferred, nothing
/// failed. If any dead node is still on the host, its resources are still
/// claimed and the only safe thing to do with its name mappings is to leave
/// them alone; the next run (or `cerulion clean`) reclaims once the registry is
/// clear.
///
/// The sweep therefore gets the WHOLE budget and the file pass gets whatever is
/// left, which is the reverse of the original split. That costs the file pass
/// wall on a busy desk — it is bounded, it reports its own floor, and the next
/// graceful exit picks up the remainder.
///
/// **Scope**, three parts. (1) The dead-node bound is `budget + one
/// attempt` (see this module's header). (2) This is best-effort: every failure
/// is a log line, never an exit code — a graph that ran correctly must not
/// report failure because the desk could not be tidied. (3) The gate is
/// necessary but not sufficient: a state file can also name a segment a LIVE
/// service still uses whose CREATING process happens to be dead (a
/// multi-process graph's worker), which the "creator is gone" evidence cannot
/// see. That hazard is shared with `cerulion clean`'s reclamation, which already
/// carries it, and this pass does not close it.
pub fn run_exit_hygiene() {
    exit_hygiene_pass(
        EXIT_HYGIENE_BUDGET,
        cleanup_dead_iceoryx2_nodes_bounded,
        |_remaining| {
            // `#[cfg(unix)]` AND the runtime predicate — they say different
            // things and neither substitutes for the other.
            //
            // The cfg is a COMPILE gate: `LibcProbe` and its `SystemProbe` impl
            // are themselves `#[cfg(unix)]`, so a non-Unix build would fail to
            // TYPE-CHECK this reference no matter what
            // `platform_uses_shm_state_files()` would have answered at run time
            // — a runtime `false` does not remove code. The predicate is a
            // BEHAVIOUR gate: every platform that keeps state files is Unix, but
            // not every Unix keeps them (Linux opens POSIX shared memory
            // directly and leaves none), so on Linux this must still do nothing.
            #[cfg(unix)]
            if crate::shm_state::platform_uses_shm_state_files() {
                let report = crate::shm_state::reclaim_at_exit(
                    std::path::Path::new(crate::shm_state::SHM_STATE_DIRECTORY),
                    _remaining,
                    &crate::shm_state::LibcProbe,
                );
                if let Some(line) = crate::shm_state::render_exit_reclaim_line(&report) {
                    // The composed sentence rides a FIELD and the message stays
                    // constant, per the repo's tracing discipline — interpolating
                    // it into the message would make the two arms ungreppable and
                    // is gated by `cerulion_core`'s
                    // `tracing_field_discipline_test`.
                    if line.failed {
                        tracing::warn!(
                            summary = %line.text,
                            reclaimed = report.reclaimed_files,
                            failures = report.reclaim_failures.len() as u64
                                + report.reclaim_failures_elided,
                            "exit hygiene: could not reclaim every state file it proved dead"
                        );
                    } else {
                        // Housekeeping that succeeded: `debug!`, so the last
                        // line of a run stays the run's own result.
                        tracing::debug!(
                            summary = %line.text,
                            reclaimed = report.reclaimed_files,
                            released_bytes = report.reclaimed_object_bytes,
                            "exit hygiene: reclaimed stale shared-memory state left by earlier runs"
                        );
                    }
                }
            }
        },
    );
}

/// One step of the exit pass, in the order it ran.
///
/// The ORDER is a correctness property (see [`run_exit_hygiene`]), and a
/// structural source walk can only pin the order of two CALLS in a body — it
/// cannot see a gate that lets the second one run anyway. This is what a test
/// observes instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HygieneStep {
    /// The bounded dead-node sweep ran.
    SweptDeadNodes,
    /// The registry had converged, so the state-file pass was REACHED. (Whether
    /// it then does anything is a platform question the caller owns.)
    ReachedStateFileReclamation,
}

/// The exit pass's ORDER, GATE and budget arithmetic, over injected work.
///
/// Extracted so the contract is testable: the production pass reads and deletes
/// from the developer's real `/tmp` and cannot be executed in a unit test, but
/// the decision it embodies can. Returns the steps it ran, in order.
fn exit_hygiene_pass<S, R>(budget: Duration, sweep: S, reclaim: R) -> Vec<HygieneStep>
where
    S: FnOnce(Duration) -> BoundedCleanup,
    R: FnOnce(Duration),
{
    let started = std::time::Instant::now();
    let mut steps = Vec::new();

    // PASS 1 — the dead-node sweep, on the WHOLE budget. It must run before
    // anything removes a state file: iceoryx2 removes its own name mappings
    // through `shm_unlink`, which takes the object AND the file together and in
    // the only order that cannot strand either.
    let swept = sweep(budget);
    steps.push(HygieneStep::SweptDeadNodes);
    // The level mirrors the state-file half below, and the STARTUP path's own
    // split: a removal that FAILED is a fault with an operator remedy
    // (permissions, or version skew from an upgrade), while a removal that
    // succeeded is housekeeping. Filing the first under the second is
    // under-reporting a condition the startup sweep already warns about.
    //
    // `deferred` alone stays quiet either way: it is the ordinary outcome of a
    // bounded pass on a busy desk, the next graceful exit picks it up, and
    // saying so on every shutdown would be exactly the noise the state-file
    // half deliberately avoids. Its operator-actionable form already exists on
    // the STARTUP path, which names `cerulion clean`.
    if swept.failed_cleanups > 0 {
        tracing::warn!(
            cleanups = swept.cleanups,
            failed_cleanups = swept.failed_cleanups,
            deferred = swept.deferred,
            budget_ms = budget.as_millis() as u64,
            "exit hygiene: could not reclaim some dead iceoryx2 node state (typically permission \
             errors when the dead node belonged to a different user, or post-upgrade version skew)"
        );
    } else if swept.cleanups > 0 {
        // Housekeeping that succeeded: `debug!` (the failure arm above stays
        // a warn), so the last line of a run stays the run's own result.
        tracing::debug!(
            cleanups = swept.cleanups,
            deferred = swept.deferred,
            budget_ms = budget.as_millis() as u64,
            "exit hygiene: reclaimed dead iceoryx2 node state left by earlier runs"
        );
    }

    // PASS 2 — the state files, on what is LEFT of the one budget, and only
    // over a CONVERGED registry.
    if !registry_converged(&swept) {
        // Deliberately NOT a warn: the sweep above already said its piece about
        // the condition, at the right level, and a desk that ends every run with
        // a second line about a consequence of it is the noise this pass exists
        // to avoid. Declining is the SAFE outcome.
        tracing::debug!(
            failed_cleanups = swept.failed_cleanups,
            deferred = swept.deferred,
            "exit hygiene: leaving stale shared-memory state files alone — dead iceoryx2 \
             nodes are still registered and removing a name mapping they still need would \
             make them permanently unreclaimable"
        );
        return steps;
    }

    reclaim(budget.saturating_sub(started.elapsed()));
    steps.push(HygieneStep::ReachedStateFileReclamation);
    steps
}

/// Did the sweep leave the iceoryx2 node registry with no dead entries at all?
///
/// The precondition for touching a `.shm_state` file. Both terms mean the same
/// thing — a dead node is still registered — and differ only in why: `deferred`
/// is a node the budget never reached, `failed_cleanups` one that was tried and
/// refused. Either way its resources are still claimed, and a name mapping
/// removed underneath it can never be given back.
///
/// Pure, so the gate is oracle-testable without a registry.
#[must_use]
pub fn registry_converged(swept: &BoundedCleanup) -> bool {
    swept.deferred == 0 && swept.failed_cleanups == 0
}

/// RAII: runs [`run_exit_hygiene`] when a `graph run` scope ends, however it
/// ends.
///
/// A guard rather than a call at the bottom of `graph_run`, because
/// `graph_run` does not HAVE one bottom: the multi-process arm `return`s from
/// inside a match (it is the Unix default), and every `?` on the way is another
/// exit. A guard constructed once covers all of them, and Rust's
/// reverse-declaration drop order puts it after the runtime, the gateway child
/// and every other resource declared below it — which is the ordering the
/// decision asks for.
///
/// Deliberately NOT constructed by `graph run-worker` or `graph run-gateway`: a
/// multi-process run would then have every worker sweeping one `/tmp` at once,
/// racing each other over the same directory for no gain. The supervisor's own
/// exit — which is `graph_run`'s — is the single place, and it happens after
/// the workers have been reaped, so their residue is exactly what it finds.
pub struct ExitHygieneGuard;

/// How many exit-hygiene guards have been ARMED in this process.
///
/// The observable behind the "a refused run does no hygiene" arm.
/// Counting the ARMING rather than the work is deliberate — the work deletes
/// from the one directory `iceoryx2-pal-posix` hardcodes (`/tmp/`), so a test
/// that had to let it run would race every other lane on the desk. A run that
/// arms nothing can do no hygiene, which is exactly the claim.
#[cfg(any(test, feature = "test-seams"))]
static EXIT_GUARDS_ARMED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Read the arming counter (test seam — see [`EXIT_GUARDS_ARMED`]).
#[cfg(any(test, feature = "test-seams"))]
#[must_use]
pub fn exit_guards_armed() -> u64 {
    EXIT_GUARDS_ARMED.load(std::sync::atomic::Ordering::SeqCst)
}

impl ExitHygieneGuard {
    /// Arm the guard. The work happens on `Drop`, never here — constructing
    /// this on the startup path must cost nothing.
    #[must_use]
    pub fn armed() -> Self {
        #[cfg(any(test, feature = "test-seams"))]
        EXIT_GUARDS_ARMED.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        Self
    }
}

impl Drop for ExitHygieneGuard {
    fn drop(&mut self) {
        run_exit_hygiene();
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// The REAL sweep runs against the REAL registry without panicking, and a
    /// ZERO budget attempts nothing there either.
    ///
    /// SCOPE: on a host with no dead nodes this is an implication whose
    /// premise is empty — every counter is legitimately zero and a
    /// budget-ignoring implementation passes it. It pins the WIRING (the real
    /// call reaches the real registry and returns a well-formed report); the
    /// POLICY is pinned by `the_budget_stops_attempting_and_starts_counting`.
    #[test]
    fn a_zero_budget_attempts_nothing_over_the_real_registry() {
        let report = cleanup_dead_iceoryx2_nodes_bounded(Duration::ZERO);
        assert_eq!(
            (report.cleanups, report.failed_cleanups),
            (0, 0),
            "a ZERO budget must not attempt a single removal, got {report:?}"
        );
    }

    /// The global entry point's config resolution sits INSIDE the panic
    /// boundary. `Config::global_config()` parses `iceoryx2.toml` on first use
    /// and can panic on a corrupt file; a refactor once moved that call
    /// outside `catch_unwind`, so a config-load panic escaped an ADVISORY
    /// sweep that documents zeroed containment. The module has no
    /// panic-injection seam (planting files in the shared registry would race
    /// sibling processes), so the wrap is pinned structurally: within
    /// `cleanup_dead_iceoryx2_nodes`, `Config::global_config()` appears only
    /// inside the closure handed to `contained_sweep`, and `contained_sweep`
    /// is the `catch_unwind`.
    #[test]
    fn the_global_sweep_resolves_its_config_inside_the_panic_boundary() {
        let src: String = include_str!("ipc_cleanup.rs")
            .lines()
            .map(|l| match l.find("//") {
                Some(i) => &l[..i],
                None => l,
            })
            .collect::<Vec<_>>()
            .join("\n");
        let body = brace_body(&src, "pub fn cleanup_dead_iceoryx2_nodes()");
        let closure = body
            .find("contained_sweep(||")
            .expect("the global entry point hands a closure to `contained_sweep`");
        let config = body
            .find("Config::global_config()")
            .expect("the global entry point resolves the global config");
        assert!(
            closure < config,
            "`Config::global_config()` must be resolved INSIDE the closure `contained_sweep` runs \
             under `catch_unwind`, never before it; body was:\n{body}"
        );
        assert_eq!(
            body.matches("Config::global_config()").count(),
            1,
            "exactly one resolution, inside the boundary; body was:\n{body}"
        );
        let boundary = brace_body(&src, "fn contained_sweep(");
        assert!(
            boundary.contains("catch_unwind("),
            "`contained_sweep` must be the `catch_unwind`; body was:\n{boundary}"
        );
        // ANTI-TAUTOLOGY: the stripped view still holds the code it claims to
        // read (a stripper that emptied it would make the absences vacuous).
        assert!(src.contains("fn contained_sweep(") && src.len() > 1000);
    }

    /// The brace-matched body of the first `fn` whose signature starts with
    /// `sig` (braces inside string literals are balanced in this file).
    fn brace_body(src: &str, sig: &str) -> String {
        let start = src
            .find(sig)
            .unwrap_or_else(|| panic!("`{sig}` must exist"));
        let open = src[start..].find('{').expect("a body") + start;
        let mut depth = 0usize;
        for (i, b) in src.as_bytes().iter().enumerate().skip(open) {
            match b {
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        return src[open..=i].to_string();
                    }
                }
                _ => {}
            }
        }
        panic!("`{sig}` body is not brace-balanced");
    }

    /// Smoke test: the cleanup call must not panic when no
    /// iceoryx2 state exists, when only live state exists, or
    /// when stale state exists. We run the call once and assert
    /// it returns a `CleanupState` whose counters are
    /// well-formed (just access them; their absolute values
    /// depend on system state and are not pinned here to avoid
    /// flakiness when other Cerulion processes are alive on the
    /// same host).
    ///
    /// Currently safe to run in parallel with other
    /// `cerulion_cli_engine` lib tests because no other test in
    /// this crate creates an iceoryx2 Node. If a future test
    /// lands that does, the new test plus this one MUST be
    /// coordinated via `--test-threads=1` (see the serial-test
    /// list in the project docs for the pattern used in
    /// `crates/cerulion_core/tests/`).
    #[test]
    fn cleanup_dead_iceoryx2_nodes_returns_state_without_panic() {
        let state = cleanup_dead_iceoryx2_nodes();
        // Access both counters so a future regression that drops
        // a field from `CleanupState` shows up as a compile error
        // here, not a runtime mystery.
        let _cleanups: u64 = state.cleanups;
        let _failed: u64 = state.failed_cleanups;
    }

    /// The ORDER and the GATE at the production
    /// composition, not at a source walk: a walk can pin which of two CALLS
    /// appears first, but not that a gate actually stops the second one. Both
    /// halves run through `exit_hygiene_pass`, the function `run_exit_hygiene`
    /// delegates to.
    #[test]
    fn the_exit_pass_sweeps_first_and_only_then_reaches_the_state_files() {
        let converged = BoundedCleanup {
            cleanups: 3,
            failed_cleanups: 0,
            deferred: 0,
        };

        // ORDER: the sweep runs, and only afterwards is the file pass reached.
        let order = std::sync::Mutex::new(Vec::new());
        let steps = exit_hygiene_pass(
            Duration::from_secs(2),
            |_| {
                order.lock().expect("order").push("sweep");
                converged
            },
            |_| order.lock().expect("order").push("reclaim"),
        );
        assert_eq!(
            *order.lock().expect("order"),
            vec!["sweep", "reclaim"],
            "the dead-node sweep must run BEFORE anything removes a name mapping it needs"
        );
        assert_eq!(
            steps,
            vec![
                HygieneStep::SweptDeadNodes,
                HygieneStep::ReachedStateFileReclamation
            ]
        );

        // GATE: each non-converged shape stops the file pass being reached at
        // all — asserted per shape, so neither term can be dropped.
        for (label, swept) in [
            (
                "a refused cleanup",
                BoundedCleanup {
                    cleanups: 0,
                    failed_cleanups: 1,
                    deferred: 0,
                },
            ),
            (
                "a deferred node",
                BoundedCleanup {
                    cleanups: 0,
                    failed_cleanups: 0,
                    deferred: 1,
                },
            ),
        ] {
            let reached = std::sync::atomic::AtomicBool::new(false);
            let steps = exit_hygiene_pass(
                Duration::from_secs(2),
                |_| swept,
                |_| reached.store(true, std::sync::atomic::Ordering::SeqCst),
            );
            assert!(
                !reached.load(std::sync::atomic::Ordering::SeqCst),
                "{label} leaves an entry registered, so the file pass must not run"
            );
            assert_eq!(steps, vec![HygieneStep::SweptDeadNodes], "{label}");
        }
    }

    /// The file pass gets what is LEFT of the ONE budget, never a fresh one —
    /// the same "a bound you can pay twice is not a bound" rule as the registry
    /// walk. Driven with a sweep that really does consume wall.
    #[test]
    fn the_state_file_pass_inherits_what_the_sweep_left_of_the_budget() {
        let budget = Duration::from_millis(300);
        let spent = Duration::from_millis(120);
        let handed = std::sync::Mutex::new(None);
        exit_hygiene_pass(
            budget,
            |_| {
                std::thread::sleep(spent);
                BoundedCleanup {
                    cleanups: 0,
                    failed_cleanups: 0,
                    deferred: 0,
                }
            },
            |remaining| *handed.lock().expect("handed") = Some(remaining),
        );
        let handed = handed
            .lock()
            .expect("handed")
            .expect("the pass was reached");
        assert!(
            handed <= budget - spent,
            "the file pass was handed {handed:?} of a {budget:?} budget the sweep had \
             already spent {spent:?} of — that is a second budget, not a remainder"
        );
    }

    /// The gate that decides whether a `.shm_state` file may be
    /// touched at all. Hand-written vector — BOTH terms must block, because
    /// both mean the same thing (a dead node is still registered) and differ
    /// only in why: `deferred` was never reached, `failed_cleanups` was tried
    /// and refused. Either one left behind an entry whose name mappings are
    /// still needed, and a mapping removed underneath it can never be given
    /// back (`tests/reclaim_ordering_test.rs` measures that, with a
    /// control).
    #[test]
    fn only_a_registry_with_no_dead_nodes_left_admits_state_file_reclamation() {
        let case = |cleanups, failed_cleanups, deferred| {
            registry_converged(&BoundedCleanup {
                cleanups,
                failed_cleanups,
                deferred,
            })
        };

        // Converged: nothing left, whether or not the sweep did work.
        assert!(case(0, 0, 0), "an empty registry is converged");
        assert!(case(7, 0, 0), "a fully successful sweep is converged");

        // Not converged — each term ALONE must block, so neither can be
        // dropped without a failing arm.
        assert!(
            !case(0, 1, 0),
            "one refused cleanup leaves an entry standing"
        );
        assert!(!case(0, 0, 1), "one deferred node leaves an entry standing");
        assert!(!case(9, 1, 0), "successes do not offset a refusal");
        assert!(!case(9, 0, 1), "successes do not offset a deferral");
        assert!(!case(0, 3, 4), "both at once is still refused");
    }

    // ── The pure classification + attribution over hand-built captures ──

    use iceoryx2::prelude::LogLevel;

    /// The sweep's own string origin. Re-derived against the live library by
    /// `the_sweep_origin_matches_the_live_rendering`.
    pub(crate) const SWEEP_ORIGIN: &str =
        "Node::<iceoryx2::service::ipc_threadsafe::Service>::cleanup_dead_nodes()";

    /// A node token exactly as iceoryx2 renders `UniqueNodeId` under `{:?}` —
    /// parens AND braces inside, which is why the refusal line is parsed from
    /// its end.
    ///
    /// The inner type changed in 0.10 (`UniqueSystemId { value, pid,
    /// creation_time }` became `UniqueId { value }`), which is exactly the kind
    /// of drift that leaves a hand fixture describing a shape the library no
    /// longer emits. `the_node_token_fixture_matches_the_live_rendering` builds
    /// a REAL node and compares, so the next such rename fails a test.
    pub(crate) fn node_token(value: u128) -> String {
        format!("UniqueNodeId(UniqueId {{ value: {value} }})")
    }

    /// The `from self` origin of a `DeadNodeView` — the derived Debug
    /// rendering, which embeds the node token.
    pub(crate) fn view_origin(node: &str) -> String {
        format!("DeadNodeView(AliveNodeView {{ id: {node}, details: None, _service: PhantomData<iceoryx2::service::ipc_threadsafe::Service> }})")
    }

    /// The hand fixtures above describe text a LIBRARY emits, so they rot
    /// silently when that text moves: the classifier keeps parsing, the tests
    /// keep passing, and `cerulion clean` attributes a refusal to the wrong
    /// node (or to none). These two arms re-derive both halves against the
    /// linked iceoryx2 rather than against a comment.
    ///
    /// The id half is the one that actually moved: `UniqueNodeId` wrapped
    /// `UniqueSystemId { value, pid, creation_time: Time { .. } }` and now
    /// wraps `UniqueId { value }`. The parser survives it because it balances
    /// parens rather than matching the inner type, and this arm is what says
    /// so out loud.
    #[test]
    fn the_node_token_fixture_matches_the_live_rendering() {
        let config = iceoryx2::testing::generate_isolated_config();
        let node = iceoryx2::node::NodeBuilder::new()
            .config(&config)
            .create::<CerService>()
            .expect("a node on an isolated config");
        let live = format!("{:?}", node.id());

        assert!(
            live.starts_with(NODE_TOKEN_PREFIX),
            "the node identity no longer renders as `{NODE_TOKEN_PREFIX}…` but as \
             `{live}` — every refusal line, sub-cause origin and detection line is \
             keyed on that prefix, so the classifier would stop attributing anything"
        );

        // The token the parser would lift out of a real refusal line is the
        // WHOLE identity, not a prefix of it.
        let refusal_line = format!("Unable to remove dead node {live} (InternalError).");
        let lifted: Vec<&str> = node_tokens(&refusal_line).collect();
        assert_eq!(
            lifted,
            vec![live.as_str()],
            "paren balancing must lift the whole live identity out of a refusal line"
        );
        let (parsed_node, variant) =
            parse_failure_line(&refusal_line).expect("a refusal line parses");
        assert_eq!(parsed_node, live, "the parsed node must be the live identity");
        assert_eq!(variant, "InternalError");

        // And the hand fixture must be the same SHAPE, so every oracle built
        // on it is still describing reality. Compared structurally (the live
        // value is whatever the process was handed), not byte for byte.
        let fixture = node_token(4242);
        let shape = |t: &str| -> String {
            t.chars()
                .map(|c| if c.is_ascii_digit() { '#' } else { c })
                .collect()
        };
        assert_eq!(
            shape(&fixture),
            shape(&live),
            "the hand node-token fixture no longer has the shape iceoryx2 renders \
             (fixture `{fixture}`, live `{live}`) — re-derive it before trusting any \
             oracle built on it"
        );
    }

    /// The sweep's origin string, re-derived. It is `Node::<{type_name}>::\
    /// cleanup_dead_nodes()` inside iceoryx2, so half of it is mechanical and
    /// a `Service` swap on Cerulion's side moves it without touching iceoryx2.
    #[test]
    fn the_sweep_origin_matches_the_live_rendering() {
        let derived = format!(
            "Node::<{}>::cleanup_dead_nodes()",
            core::any::type_name::<CerService>()
        );
        assert_eq!(
            SWEEP_ORIGIN, derived,
            "the sweep origin the fixtures build refusal lines with is not the one \
             the linked iceoryx2 would print"
        );
    }

    pub(crate) fn line(
        level: LogLevel,
        origin: impl Into<String>,
        message: impl Into<String>,
    ) -> CapturedLog {
        CapturedLog {
            level,
            origin: origin.into(),
            message: message.into(),
        }
    }

    pub(crate) fn sub_cause(node: &str, tail: &str) -> CapturedLog {
        line(
            LogLevel::Debug,
            view_origin(node),
            format!("Unable to remove stale resources since the {tail}"),
        )
    }

    pub(crate) fn refusal(node: &str, variant: &str) -> CapturedLog {
        line(
            LogLevel::Trace,
            SWEEP_ORIGIN,
            format!("Unable to remove dead node {node} ({variant})."),
        )
    }

    pub(crate) fn detected(node: &str) -> CapturedLog {
        line(
            LogLevel::Debug,
            SWEEP_ORIGIN,
            format!("Dead node ({node}) detected"),
        )
    }

    pub(crate) fn removed(node: &str) -> CapturedLog {
        line(
            LogLevel::Trace,
            SWEEP_ORIGIN,
            format!("The dead node ({node}) was successfully removed."),
        )
    }

    /// The motivating shape: ONE node refused with `InternalError`,
    /// preceded by the two sub-cause lines iceoryx2 logs on the
    /// stale-port path (`mod.rs:709` + the corrupted-remainder arm at
    /// `:659`). The counts must be what the earlier loop produced and the
    /// new entry must carry BOTH sub-causes, in order, under the node's
    /// verbatim token and the bare variant name.
    #[test]
    fn one_internal_error_node_carries_its_two_preceding_sub_causes() {
        let node = node_token(4242);
        let c1 = "corrupted service remainders to could not be removed due to an internal error (InternalError).";
        let c2 = "stale resources of the port PortId(9) could not be removed due to an internal failure.";
        let captured = vec![
            detected(&node),
            sub_cause(&node, c1),
            sub_cause(&node, c2),
            refusal(&node, "InternalError"),
        ];

        let parts = classify_cleanup_failures(&captured);

        assert_eq!(
            parts.failures_by_cause,
            std::collections::BTreeMap::from([("iceoryx2 internal error".to_string(), 1)])
        );
        assert!(parts.unclassified.is_empty(), "{:?}", parts.unclassified);
        assert_eq!(
            parts.failures,
            vec![FailedNodeCleanup {
                node: node.clone(),
                variant: "InternalError".to_string(),
                causes: vec![
                    format!("Unable to remove stale resources since the {c1}"),
                    format!("Unable to remove stale resources since the {c2}"),
                ],
            }]
        );
    }

    /// Two nodes whose lines are INTERLEAVED (a shape the real sequential
    /// sweep never produces — which is exactly why it is the discriminating
    /// oracle for the id arm: adjacency alone would hand every line to
    /// whichever refusal came first). Each node must receive ONLY its own
    /// sub-causes, and the per-cause counts must both be 1.
    #[test]
    fn two_interleaved_nodes_are_kept_apart_by_their_ids() {
        let a = node_token(1);
        let b = node_token(2);
        let captured = vec![
            detected(&a),
            sub_cause(&a, "service tags could not be read due to an internal error."),
            detected(&b),
            sub_cause(&b, "dead node was using a different iceoryx2 version."),
            sub_cause(&a, "port tags could not be read due to an internal error."),
            sub_cause(&b, "stale resources of the port PortId(3) could not be removed since the iceoryx2 version does not match."),
            refusal(&a, "InternalError"),
            refusal(&b, "VersionMismatch"),
        ];

        let parts = classify_cleanup_failures(&captured);

        assert_eq!(
            parts.failures_by_cause,
            std::collections::BTreeMap::from([
                ("iceoryx2 internal error".to_string(), 1),
                ("version mismatch".to_string(), 1),
            ])
        );
        assert!(parts.unclassified.is_empty());
        assert_eq!(parts.failures.len(), 2, "{:?}", parts.failures);
        let (fa, fb) = (&parts.failures[0], &parts.failures[1]);
        assert_eq!(
            (fa.node.as_str(), fa.variant.as_str()),
            (a.as_str(), "InternalError")
        );
        assert_eq!(
            (fb.node.as_str(), fb.variant.as_str()),
            (b.as_str(), "VersionMismatch")
        );
        assert_eq!(
            fa.causes,
            vec![captured[1].message.clone(), captured[4].message.clone()],
            "node A must get exactly its own two sub-causes, in order"
        );
        assert_eq!(
            fb.causes,
            vec![captured[3].message.clone(), captured[5].message.clone()],
            "node B must get exactly its own two sub-causes, in order"
        );
    }

    /// A permission-denied refusal, with the adjacency arm exercised on
    /// BOTH sides of a boundary: an ownerless sub-cause logged during an
    /// EARLIER node's successful removal must NOT leak across that
    /// success line, while an ownerless line after it (a string-origin
    /// service-layer line about this node) IS attributed by adjacency.
    #[test]
    fn a_permission_denied_node_takes_ownerless_lines_only_from_its_own_block() {
        let earlier = node_token(5);
        let node = node_token(6);
        let stray_before_boundary = line(
            LogLevel::Debug,
            "Service::remove_node",
            "Unable to update the service state since the lock could not be acquired.",
        );
        let ownerless_in_block = line(
            LogLevel::Debug,
            "Service::remove_node",
            "Unable to unlink the port resources since the process does not have sufficient permissions.",
        );
        let captured = vec![
            detected(&earlier),
            stray_before_boundary.clone(),
            removed(&earlier),
            detected(&node),
            ownerless_in_block.clone(),
            sub_cause(&node, "stale resources of the port PortId(1) could not be removed due to insufficient permissions."),
            refusal(&node, "InsufficientPermissions"),
        ];

        let parts = classify_cleanup_failures(&captured);

        assert_eq!(
            parts.failures_by_cause,
            std::collections::BTreeMap::from([("permission denied".to_string(), 1)])
        );
        assert!(parts.unclassified.is_empty());
        assert_eq!(parts.failures.len(), 1);
        let f = &parts.failures[0];
        assert_eq!(
            (f.node.as_str(), f.variant.as_str()),
            (node.as_str(), "InsufficientPermissions")
        );
        assert_eq!(
            f.causes,
            vec![
                ownerless_in_block.message.clone(),
                captured[5].message.clone()
            ],
            "the ownerless line INSIDE this node's block is adjacency-attributed; the one \
             before the earlier node's success line is not"
        );
        assert!(
            !f.causes.iter().any(|c| c == &stray_before_boundary.message),
            "a success line is a block boundary the adjacency arm must not cross"
        );
    }

    /// A variant no substring rule knows: the refusal line lands VERBATIM
    /// in `unclassified` and counts under NO cause (the earlier behaviour,
    /// unchanged) — and it STILL gets a `failures` entry, because an
    /// unclassified variant is the refusal whose sub-causes a reader most
    /// needs.
    #[test]
    fn an_unclassified_variant_is_reported_raw_and_still_attributed() {
        let node = node_token(9);
        // The VERBATIM line `ResourcesAlreadyCleanedUp` actually produces
        // (`acquire_cleaner_lock`, `mod.rs:762`) — not a `since the …` line
        // borrowed from another arm, which is how a `since the`-only
        // predicate stayed green while dropping the real explanation.
        let captured = vec![
            line(
                LogLevel::Debug,
                view_origin(&node),
                CLEANER_LOCK_ALREADY_CLEANED_UP,
            ),
            refusal(&node, "ResourcesAlreadyCleanedUp"),
        ];

        let parts = classify_cleanup_failures(&captured);

        assert!(
            parts.failures_by_cause.is_empty(),
            "{:?}",
            parts.failures_by_cause
        );
        assert_eq!(parts.unclassified, vec![captured[1].message.clone()]);
        assert_eq!(
            parts.failures,
            vec![FailedNodeCleanup {
                node,
                variant: "ResourcesAlreadyCleanedUp".to_string(),
                causes: vec![captured[0].message.clone()],
            }]
        );
    }

    /// No bridge installed ⇒ nothing captured ⇒ every part empty. The
    /// `Default` of the parts IS the oracle, so a future field cannot
    /// arrive pre-populated.
    #[test]
    fn an_empty_capture_yields_empty_everything() {
        let parts = classify_cleanup_failures(&[]);
        assert_eq!(parts, ClassifiedFailures::default());
        assert!(parts.failures_by_cause.is_empty());
        assert!(parts.unclassified.is_empty());
        assert!(parts.failures.is_empty());
        assert!(parts.registry_errors.is_empty());
    }

    /// Sub-cause lines with NO refusal after them attribute to nothing —
    /// a successful sweep that logged explanations produces no entry.
    #[test]
    fn sub_causes_without_a_refusal_produce_no_entry() {
        let node = node_token(3);
        let captured = vec![
            detected(&node),
            sub_cause(&node, "service itself is corrupted. Trying to remove the corrupted remainders of the service."),
            removed(&node),
        ];
        assert_eq!(
            classify_cleanup_failures(&captured),
            ClassifiedFailures::default()
        );
    }

    /// The refusal-line parser, on the real 0.9.1 shape and on the two
    /// degenerate ones: no parenthesised tail (whole remainder = node,
    /// EMPTY variant, never a guess) and a message that is not a refusal
    /// at all (`None`).
    #[test]
    fn the_refusal_line_parser_takes_the_last_parenthesised_token_as_the_variant() {
        let node = node_token(11);
        assert_eq!(
            parse_failure_line(&format!(
                "Unable to remove dead node {node} (InternalError)."
            )),
            Some((node.clone(), "InternalError".to_string()))
        );
        // No trailing period — a future iceoryx2 might drop it.
        assert_eq!(
            parse_failure_line(&format!(
                "Unable to remove dead node {node} (VersionMismatch)"
            )),
            Some((node.clone(), "VersionMismatch".to_string()))
        );
        assert_eq!(
            parse_failure_line("Unable to remove dead node NodeId(7)."),
            Some(("NodeId(7)".to_string(), String::new())),
            "a lone parenthesised token is the node, not a variant"
        );
        assert_eq!(
            parse_failure_line("Unable to remove dead node 7."),
            Some(("7".to_string(), String::new()))
        );
        assert_eq!(parse_failure_line("Dead node (x) detected"), None);
    }

    /// The origin `get_node_state` logs from (`iceoryx2-0.9.1/src/node/mod.rs:1310`)
    /// — a string origin naming the VISITED node, which the registry walk
    /// emits for every node it cannot monitor, refused or not.
    fn get_node_state_origin(node: &str) -> String {
        format!("Node::get_node_state(Config {{ global: Global {{ root_path: \"/tmp/iceoryx2/\", .. }}, .. }}, {node})")
    }

    /// The line an other-user node produces on the walk (`mod.rs:1314`; the
    /// arm `NodeState::new` maps to `Inaccessible`).
    const INACCESSIBLE_LINE: &str =
        "Unable to acquire node monitor due to insufficient permissions while acquiring the node state.";

    /// The line an unreadable ALIVE node produces on the walk (`mod.rs:1351`).
    const UNREADABLE_STORAGE_LINE: &str =
        "Unable to open node config storage since the node config storage could not be read.";

    /// The cross-user desk shape: two nodes the sweep never refuses — one
    /// another user's (`Inaccessible`), one alive with an unreadable config
    /// storage — log their own walk lines with THEIR tokens in the origin,
    /// right before a stranded node's block, with no refusal or success
    /// boundary in between. Neither line may land under the stranded node:
    /// its entry must carry exactly its own explanation.
    #[test]
    fn a_non_refused_nodes_line_is_not_blamed_on_the_next_refusal() {
        let other_user = node_token(31);
        let alive_unreadable = node_token(32);
        let stranded = node_token(33);
        let own = "port tags could not be read due to an internal error.";
        let captured = vec![
            line(
                LogLevel::Debug,
                get_node_state_origin(&other_user),
                INACCESSIBLE_LINE,
            ),
            line(
                LogLevel::Debug,
                format!("open_node_storage(Config {{ .. }}, {alive_unreadable})"),
                UNREADABLE_STORAGE_LINE,
            ),
            detected(&stranded),
            sub_cause(&stranded, own),
            refusal(&stranded, "InternalError"),
        ];

        let parts = classify_cleanup_failures(&captured);

        assert_eq!(
            parts.failures,
            vec![FailedNodeCleanup {
                node: stranded.clone(),
                variant: "InternalError".to_string(),
                causes: vec![format!("Unable to remove stale resources since the {own}")],
            }],
            "a line naming a node that was never refused belongs to nobody — \
             it must not be filed under the next refusal by position"
        );
        assert_eq!(
            parts.failures_by_cause,
            std::collections::BTreeMap::from([("iceoryx2 internal error".to_string(), 1)]),
            "the foreign lines are not refusals either — the counts see one failure"
        );
    }

    /// The control that isolates the discriminating variable: the IDENTICAL
    /// walk-line shape, with the refused node's OWN token in the origin, IS
    /// attributed — by the id arm, not by adjacency — so the guard above
    /// rejects lines for WHOSE token they carry, never for the shape of the
    /// origin they come from.
    #[test]
    fn the_same_walk_line_carrying_the_refused_nodes_own_token_is_attributed_by_id() {
        let stranded = node_token(34);
        let own_walk_line = line(
            LogLevel::Debug,
            get_node_state_origin(&stranded),
            INACCESSIBLE_LINE,
        );
        let captured = vec![
            own_walk_line.clone(),
            detected(&stranded),
            refusal(&stranded, "InsufficientPermissions"),
        ];

        let parts = classify_cleanup_failures(&captured);

        assert_eq!(parts.failures.len(), 1);
        assert_eq!(
            parts.failures[0].causes,
            vec![own_walk_line.message.clone()],
            "the token is the refused node's own, so the id arm claims the line"
        );
    }

    /// A foreign-token line is not merely withheld from the NEXT refusal —
    /// it is withheld from every refusal, and a genuinely ownerless line in
    /// the same block is still adjacency-attributed beside it (the guard
    /// narrows the adjacency arm to token-free lines, it does not disable
    /// it).
    #[test]
    fn a_foreign_token_line_beside_an_ownerless_one_withholds_only_itself() {
        let other = node_token(35);
        let stranded = node_token(36);
        let ownerless = line(
            LogLevel::Debug,
            "Service::remove_node",
            "Unable to unlink the port resources since the process does not have sufficient permissions.",
        );
        let captured = vec![
            detected(&stranded),
            line(
                LogLevel::Debug,
                get_node_state_origin(&other),
                INACCESSIBLE_LINE,
            ),
            ownerless.clone(),
            refusal(&stranded, "InsufficientPermissions"),
        ];

        let parts = classify_cleanup_failures(&captured);

        assert_eq!(parts.failures.len(), 1);
        assert_eq!(
            parts.failures[0].causes,
            vec![ownerless.message.clone()],
            "{:?}",
            parts.failures[0]
        );
    }

    /// A `Node::state_from_monitor(…)` line — node-scoped, TOKEN-FREE — sits
    /// right before a refused node's block (the inaccessible neighbour
    /// `Node::list` classified just before the dead node was attempted; on a
    /// real walk nothing separates the two). It must NOT become the refused
    /// node's cause, and it is not a registry-wide line either. The node's
    /// own explanation and a genuinely ownerless service-layer line beside it
    /// are still attributed — the guard narrows the adjacency arm to lines
    /// ownerless in FACT, not merely in form. Keyed on the origin head: the
    /// same message from the node's own `DeadNodeView` origin is the node's
    /// (the id arm), so a guard reading the message would be wrong.
    #[test]
    fn a_monitor_state_line_about_a_visited_node_is_never_attributed_by_adjacency() {
        let node = node_token(6);
        let monitor_line = line(
            LogLevel::Debug,
            "Node::state_from_monitor(ProcessMonitor { state_path: FilePath { value: \"/tmp/iceoryx2/nodes/iox2_5.node_monitor\" }, owner_lock_path: FilePath { value: \"/tmp/iceoryx2/nodes/iox2_5.node_monitor_owner_lock\" }, context_path: FilePath { value: \"/tmp/iceoryx2/nodes/iox2_5.node_monitor_context\" } })",
            "Unable to acquire node state from monitor due to insufficient permissions to acquire the nodes state.",
        );
        let ownerless = line(
            LogLevel::Debug,
            "Service::remove_node",
            "Unable to unlink the port resources since the process does not have sufficient permissions.",
        );
        let own = sub_cause(
            &node,
            "port tags could not be read due to an internal error.",
        );
        let captured = vec![
            monitor_line.clone(),
            detected(&node),
            ownerless.clone(),
            own.clone(),
            refusal(&node, "InternalError"),
        ];

        let parts = classify_cleanup_failures(&captured);

        assert_eq!(parts.failures.len(), 1, "{:?}", parts.failures);
        assert_eq!(
            parts.failures[0].causes,
            vec![ownerless.message.clone(), own.message.clone()],
            "the visited node's monitor line is nobody's; the ownerless line and the node's own \
             are still attributed"
        );
        assert!(
            parts.registry_errors.is_empty(),
            "a visited node's monitor failure is not a registry-wide line: {:?}",
            parts.registry_errors
        );

        // The control for the origin key: the same MESSAGE from the node's own
        // view origin carries its token and IS its cause.
        let from_own_view = line(
            LogLevel::Debug,
            view_origin(&node),
            monitor_line.message.clone(),
        );
        let captured = vec![
            detected(&node),
            from_own_view.clone(),
            refusal(&node, "InternalError"),
        ];
        assert_eq!(
            classify_cleanup_failures(&captured).failures[0].causes,
            vec![from_own_view.message.clone()],
            "the guard reads the origin head, never the message"
        );
    }

    /// `node_tokens` on the three renderings the classifier meets: a bare
    /// token, a `Dead node (…) detected` line (the token wrapped in the
    /// sweep's own parens — balancing must stop at the token's closer, not
    /// the wrapper's), a view origin, an origin naming TWO nodes, and text
    /// with no token — plus a truncated token, which yields nothing.
    #[test]
    fn node_tokens_extracts_each_balanced_unique_node_id_span() {
        let a = node_token(41);
        let b = node_token(42);
        let collect = |text: &str| node_tokens(text).map(str::to_string).collect::<Vec<_>>();

        assert_eq!(collect(&a), vec![a.clone()]);
        assert_eq!(
            collect(&format!("Dead node ({a}) detected")),
            vec![a.clone()]
        );
        assert_eq!(collect(&view_origin(&a)), vec![a.clone()]);
        assert_eq!(
            collect(&format!("moved {a} beside {b}")),
            vec![a.clone(), b.clone()]
        );
        assert!(collect("Service::remove_node").is_empty());
        assert!(collect("Config::global_config()").is_empty());
        let truncated = &a[..a.len() - 4];
        assert!(
            collect(truncated).is_empty(),
            "an unbalanced prefix is not a token: {truncated}"
        );
    }

    /// `acquire_cleaner_lock` (`iceoryx2-0.9.1/src/node/mod.rs:741-778`), the
    /// three refusal-producing arms, verbatim. `msg` there is
    /// `"Unable to acquire monitor cleaner"`.
    const CLEANER_LOCK_ANOTHER_INSTANCE: &str =
        "Unable to acquire monitor cleaner since another instance is already cleaning up all resources.";
    const CLEANER_LOCK_ALREADY_CLEANED_UP: &str =
        "Unable to acquire monitor cleaner since another instance has already cleaned up all resources.";
    const CLEANER_LOCK_INTERRUPT: &str =
        "Unable to acquire monitor cleaner since an interrupt signal was received.";
    /// The intra-process `IN_CLEANUP_SECTION` guard in
    /// `remove_stale_resources_impl` (`mod.rs:598`); `msg` is
    /// `"Unable to remove stale resources"`.
    const IN_CLEANUP_SECTION_LINE: &str =
        "Unable to remove stale resources since another instance is already cleaning up the dead nodes resources.";
    /// `blocking_remove_stale_resources` giving up (`mod.rs:570`) — under the
    /// sweep's ZERO timeout this follows `:598`/`:758` immediately and is the
    /// line that becomes the `AnotherInstanceIsCleaningUpTheNode` refusal.
    const BLOCKING_TIMEOUT_LINE: &str =
        "Unable to block until the stale resources of the dead node are removed since another instance requires longer than 0ns to cleanup the resources.";

    /// The cleaner-lock CONTENTION shape — two sweeps racing over one dead
    /// node, the routine outcome of two `graph run`s starting together. The
    /// explanation is `mod.rs:758` followed by the `:570` give-up; both must
    /// land under the refusal, in emission order. This was the probe that
    /// failed on the `since the`-only predicate (`left: []`).
    ///
    /// The count half is the code-review P1 oracle: the refusal counts under
    /// `lock contention` — the cause the remediation table already has a
    /// remedy for — and NOT under `unclassified`, where it landed while the
    /// only rule for it named `InCleanupSection`, a variant 0.9.1 does not
    /// have (`mod.rs:269-282`).
    #[test]
    fn a_cleaner_lock_contention_refusal_carries_its_verbatim_explanation() {
        let node = node_token(51);
        let captured = vec![
            detected(&node),
            line(
                LogLevel::Debug,
                view_origin(&node),
                CLEANER_LOCK_ANOTHER_INSTANCE,
            ),
            line(LogLevel::Debug, view_origin(&node), BLOCKING_TIMEOUT_LINE),
            refusal(&node, "AnotherInstanceIsCleaningUpTheNode"),
        ];

        let parts = classify_cleanup_failures(&captured);

        assert_eq!(
            parts.failures_by_cause,
            std::collections::BTreeMap::from([("lock contention".to_string(), 1)]),
            "a cleaner-lock refusal is lock contention, not an unknown variant"
        );
        assert!(parts.unclassified.is_empty(), "{:?}", parts.unclassified);
        assert_eq!(
            parts.failures,
            vec![FailedNodeCleanup {
                node,
                variant: "AnotherInstanceIsCleaningUpTheNode".to_string(),
                causes: vec![
                    CLEANER_LOCK_ANOTHER_INSTANCE.to_string(),
                    BLOCKING_TIMEOUT_LINE.to_string(),
                ],
            }]
        );
    }

    /// One arm per refusal-producing message that does NOT read `since the`:
    /// each, alone before its refusal, is attributed as exactly `[that line]`.
    /// Restoring the `since the`-only predicate fails every row.
    /// The third column is the variant's classification: the two
    /// cleaner-lock refusals count under `lock contention`; `Interrupt` and
    /// `ResourcesAlreadyCleanedUp` stay `unclassified` (verbatim, counted
    /// under no cause) — the table has no remedy for either.
    #[test]
    fn each_cleaner_lock_and_signal_arm_is_attributed_verbatim() {
        let rows: [(&str, &str, Option<&str>); 4] = [
            (
                IN_CLEANUP_SECTION_LINE,
                "AnotherInstanceIsCleaningUpTheNode",
                Some("lock contention"),
            ),
            (
                CLEANER_LOCK_ANOTHER_INSTANCE,
                "AnotherInstanceIsCleaningUpTheNode",
                Some("lock contention"),
            ),
            (
                CLEANER_LOCK_ALREADY_CLEANED_UP,
                "ResourcesAlreadyCleanedUp",
                None,
            ),
            (CLEANER_LOCK_INTERRUPT, "Interrupt", None),
        ];
        for (k, (message, variant, cause)) in rows.iter().enumerate() {
            let node = node_token(60 + k as u128);
            let captured = vec![
                detected(&node),
                line(LogLevel::Debug, view_origin(&node), *message),
                refusal(&node, variant),
            ];

            let parts = classify_cleanup_failures(&captured);

            assert_eq!(parts.failures.len(), 1, "row {k}: {parts:?}");
            assert_eq!(
                parts.failures[0].causes,
                vec![message.to_string()],
                "row {k} ({variant}): the verbatim explanation must be the one cause"
            );
            match cause {
                Some(cause) => {
                    assert_eq!(
                        parts.failures_by_cause,
                        std::collections::BTreeMap::from([(cause.to_string(), 1)]),
                        "row {k} ({variant}) must count under `{cause}`"
                    );
                    assert!(parts.unclassified.is_empty(), "row {k}: {parts:?}");
                }
                None => {
                    assert!(parts.failures_by_cause.is_empty(), "row {k}: {parts:?}");
                    assert_eq!(
                        parts.unclassified,
                        vec![captured[2].message.clone()],
                        "row {k} ({variant}) stays unclassified, verbatim"
                    );
                }
            }
        }
    }

    /// Every distinct PER-NODE sub-cause shape iceoryx2 0.9.1 can log on the
    /// way to a refusal — read off `mod.rs` 540-835
    /// (`remove_stale_resources_impl`, `acquire_cleaner_lock`, `remove_node`,
    /// `blocking_remove_stale_resources`) and 1299-1391 (the per-node walk
    /// helpers) plus `service/stale_resource_cleanup.rs`. Placeholders (`0ns`,
    /// port `9`, `(InternalError)`) stand in for the `{:?}` interpolations.
    const PER_NODE_SUB_CAUSE_SHAPES: &[&str] = &[
        // remove_stale_resources_impl / blocking_remove_stale_resources
        "Unable to block until the stale resources of the dead node are removed since the adaptive wait builder could not be initiated.",
        "Unable to block until the stale resources of the dead node are removed since the current system time could not be acquired.",
        "Unable to block until the stale resources of the dead node are removed since the adaptive wait failed.",
        "Unable to block until the stale resources of the dead node are removed due to a failure while acquiring the elapsed time.",
        BLOCKING_TIMEOUT_LINE,
        IN_CLEANUP_SECTION_LINE,
        "Unable to remove stale resources since the monitor cleaner lock could not be acquired.",
        "Unable to remove stale resources since the dead node was using a different iceoryx2 version.",
        "Unable to remove stale resources since the service itself is corrupted. Trying to remove the corrupted remainders of the service.",
        "Unable to remove stale resources since the corrupted service remainders to could not be removed due to insufficient permissions.",
        "Unable to remove stale resources since the corrupted service remainders to could not be removed due to an internal error (InternalError).",
        "Unable to remove stale resources due to an internal error while removing the node from the service (InternalError).",
        "Unable to remove stale resources since the service tags could not be read due to insufficent permissions.",
        "Unable to remove stale resources since the service tags could not be read due to an internal error.",
        "Unable to remove stale resources since the stale resources of the port 9 could not be removed due to insufficient permissions.",
        "Unable to remove stale resources since the stale resources of the port 9 could not be removed since the iceoryx2 version does not match.",
        "Unable to remove stale resources since the stale resources of the port 9 could not be removed due to an internal failure.",
        "Unable to remove stale resources since the port tags could not be read due to insufficent permissions.",
        "Unable to remove stale resources since the port tags could not be read due to an internal error.",
        "Unable to remove stale resources since the node itself could not be removed.",
        // acquire_cleaner_lock
        CLEANER_LOCK_ANOTHER_INSTANCE,
        CLEANER_LOCK_ALREADY_CLEANED_UP,
        CLEANER_LOCK_INTERRUPT,
        "Unable to acquire monitor cleaner due to an internal error while acquiring monitoring cleaner.",
        // the node-detail storage arms (:783-836) — note `node detail
        // storages`, one character from the registry-wide `nodes`
        "Unable to list all node detail storages due to insufficient permissions.",
        "Unable to list all node detail storages due to an internal error.",
        "Unable to remove node detail storage entry due to insufficient permissions.",
        "Unable to remove node detail storage entry due to an internal failure.",
        "Unable to remove node details directory due to insufficient permissions.",
        "Unable to remove node details directory due to an internal error.",
        // the per-node walk helpers (:1272-1391)
        "Unable to acquire node state from monitor due to insufficient permissions to acquire the nodes state.",
        "Unable to acquire node state from monitor due to an interrupt signal while acquiring the nodes state.",
        "Unable to acquire node state from monitor due to an internal error while acquiring the nodes state.",
        "Unable to acquire node monitor due to insufficient permissions while acquiring the node state.",
        "Unable to acquire node monitor since an interrupt was received while acquiring the node state.",
        "Unable to acquire node monitor since an internal failure occurred while acquiring the node state.",
        "Unable to open node config storage since the node config storage could not be read.",
        "Unable to open node config storage since the node config storage seems to be uninitialized but the state should always be present.",
        "Unable to open node config storage due to an internal failure while opening the node config storage.",
        "Unable to read node details since the content of the node config storage could not be read.",
        "Unable to read node details since the contents of the node config storage is corrupted.",
        // service/stale_resource_cleanup.rs
        "Failed to remove stale port resources due to insufficient permissions to list all connections.",
        "Failed to remove stale port resources due to an internal error while listing all connections.",
        "Failed to remove stale port resources since connection (\"c\") has a different iceoryx2 version.",
        "Failed to remove stale port resources due to insufficient permissions to remove the connection (\"c\").",
        "Failed to remove stale port resources due to insufficient permissions to remove the ports data segment.",
        "Failed to remove stale port resources due to an internal error while removing the ports data segment.",
        "Failed to remove stale port resources due to insufficient permissions to remove the port from its connections.",
        "Failed to remove stale port resources since the port could not be removed from its connection since iceoryx2 version does not match.",
        "Failed to remove stale port resources due to an internal error while removing the port from its connection.",
    ];

    /// The five REGISTRY-WIDE lines, one per row of [`REGISTRY_WIDE_LINE_HEADS`]'s
    /// enumeration (`mod.rs:1242`, `:1145`, `:1138`, `:1261`, `:1265`), paired
    /// with the origin each is logged from; the `{e:?}` interpolation is
    /// rendered as the `NodeListFailure` variant that reaches it.
    const REGISTRY_WIDE_SHAPES: [(&str, &str); 5] = [
        (
            SWEEP_ORIGIN,
            "Unable to perform a full scan for dead nodes since the all existing nodes could not be listed (InsufficientPermissions).",
        ),
        (
            "Node::list()",
            "Unable to iterate over Node list since the node list could not be acquired (InsufficientPermissions).",
        ),
        (
            "Node::list()",
            "Unable to iterate over Node list since the following error occurred (Interrupt).",
        ),
        (
            "Node::list_all_nodes(Config { global: Global { root_path: \"/tmp/iceoryx2/\", .. }, .. })",
            "Unable to list all nodes due to insufficient permissions while listing all nodes.",
        ),
        (
            "Node::list_all_nodes(Config { global: Global { root_path: \"/tmp/iceoryx2/\", .. }, .. })",
            "Unable to list all nodes due to an internal failure while listing all nodes.",
        ),
    ];

    /// Every per-node AND every registry-wide shape is recognised as
    /// sub-cause text; the three non-explanatory lines the sweep emits are
    /// not. The registry-wide rows are DELIBERATELY in the positive half:
    /// they read as sub-causes to this predicate, which is exactly what makes
    /// the registry guard in the adjacency arm load-bearing rather than
    /// redundant.
    #[test]
    fn the_sub_cause_predicate_covers_every_0_9_1_shape_and_only_those() {
        let node = node_token(70);
        for shape in PER_NODE_SUB_CAUSE_SHAPES
            .iter()
            .copied()
            .chain(REGISTRY_WIDE_SHAPES.iter().map(|(_, shape)| *shape))
        {
            assert!(
                is_sub_cause_line(&line(LogLevel::Debug, view_origin(&node), shape)),
                "not recognised as a sub-cause: {shape}"
            );
        }
        for bookkeeping in [
            detected(&node),
            removed(&node),
            refusal(&node, "AnotherInstanceIsCleaningUpTheNode"),
        ] {
            assert!(
                !is_sub_cause_line(&bookkeeping),
                "a bookkeeping line must never read as a sub-cause: {}",
                bookkeeping.message
            );
        }
    }

    /// The registry-wide predicate over the whole enumeration, both halves:
    /// each of the five walk lines (from its real origin) is registry-wide;
    /// every per-node shape and every bookkeeping line is not — so a head
    /// widened to swallow a per-node line (`Unable to list all node detail
    /// storages …` is one character from `Unable to list all nodes`) fails
    /// here, and a dropped head fails on its row.
    #[test]
    fn the_registry_wide_predicate_matches_exactly_the_five_walk_lines() {
        for (k, (origin, shape)) in REGISTRY_WIDE_SHAPES.iter().enumerate() {
            assert!(
                is_registry_wide_line(&line(LogLevel::Debug, *origin, *shape)),
                "row {k}: not recognised as registry-wide: {shape}"
            );
        }
        let node = node_token(71);
        for shape in PER_NODE_SUB_CAUSE_SHAPES {
            assert!(
                !is_registry_wide_line(&line(LogLevel::Debug, view_origin(&node), *shape)),
                "a per-node line must never read as registry-wide: {shape}"
            );
        }
        for bookkeeping in [
            detected(&node),
            removed(&node),
            refusal(&node, "InternalError"),
        ] {
            assert!(
                !is_registry_wide_line(&bookkeeping),
                "a bookkeeping line must never read as registry-wide: {}",
                bookkeeping.message
            );
        }
    }

    /// THE finding: the full-scan line (`mod.rs:1242`) sitting IMMEDIATELY
    /// before a refusal — token-free and sub-cause-shaped, everything the
    /// adjacency arm looks for. It must NOT become that node's cause, and
    /// MUST be in `registry_errors`. The node's own explanation and a
    /// genuinely ownerless service-layer line in the same block are still
    /// attributed (the control: the guard narrows the adjacency arm to
    /// registry lines, it does not disable it), and the counts see ONE
    /// failure (the registry line is not a refusal). Reverting the
    /// `is_registry_wide_line` exclusion in the adjacency arm fails the
    /// FIRST assertion.
    #[test]
    fn a_registry_wide_line_before_a_refusal_is_global_not_that_nodes_cause() {
        let node = node_token(80);
        let (scan_origin, scan_shape) = REGISTRY_WIDE_SHAPES[0];
        let full_scan = line(LogLevel::Debug, scan_origin, scan_shape);
        let ownerless = line(
            LogLevel::Debug,
            "Service::remove_node",
            "Unable to unlink the port resources since the process does not have sufficient permissions.",
        );
        let own = "port tags could not be read due to an internal error.";
        let captured = vec![
            full_scan.clone(),
            detected(&node),
            ownerless.clone(),
            sub_cause(&node, own),
            refusal(&node, "InternalError"),
        ];

        let parts = classify_cleanup_failures(&captured);

        assert_eq!(parts.failures.len(), 1, "{parts:?}");
        assert!(
            !parts.failures[0]
                .causes
                .iter()
                .any(|cause| cause == &full_scan.message),
            "a registry-wide line must never be a node's cause — a global failure \
             rendered as this node's fault; got {:?}",
            parts.failures[0].causes
        );
        assert_eq!(
            parts.registry_errors,
            vec![full_scan.message.clone()],
            "the line belongs to the registry walk, verbatim"
        );
        assert_eq!(
            parts.failures[0].causes,
            vec![
                ownerless.message.clone(),
                format!("Unable to remove stale resources since the {own}"),
            ],
            "the node keeps its own explanation and the ownerless service-layer line beside it"
        );
        assert_eq!(
            parts.failures_by_cause,
            std::collections::BTreeMap::from([("iceoryx2 internal error".to_string(), 1)]),
            "the registry line is not a refusal — the counts see one failure"
        );
    }

    /// The realistic 0.9.1 shape for a registry that cannot be LISTED —
    /// `list_all_nodes` (`:1261`) → `Node::list` (`:1145`) → the sweep entry
    /// (`:1242`), in that order: NO refusal, NO node, and the three lines
    /// verbatim under `registry_errors`. The sweep's counters read 0/0 here,
    /// which without this field is indistinguishable from "nothing to clean".
    #[test]
    fn a_registry_that_cannot_be_listed_reports_its_chain_and_no_node() {
        let chain: Vec<CapturedLog> = [3usize, 1, 0]
            .into_iter()
            .map(|row| {
                let (origin, shape) = REGISTRY_WIDE_SHAPES[row];
                line(LogLevel::Debug, origin, shape)
            })
            .collect();

        let parts = classify_cleanup_failures(&chain);

        assert_eq!(
            parts.registry_errors,
            chain.iter().map(|l| l.message.clone()).collect::<Vec<_>>(),
            "every line of the chain, in emission order"
        );
        assert!(parts.failures.is_empty(), "{:?}", parts.failures);
        assert!(parts.failures_by_cause.is_empty());
        assert!(parts.unclassified.is_empty());
    }

    /// The other reachable 0.9.1 shape: the walk ABORTS on `Interrupt` AFTER
    /// refusing a node (`NodeState::new` `:413` → `Node::list` `:1138` → the
    /// sweep entry `:1242`). The registry lines FOLLOW the refusal, so the
    /// adjacency arm never considers them — yet both must be reported, apart:
    /// the node with exactly its own cause, the walk with its two lines.
    #[test]
    fn an_aborted_walk_after_a_refusal_reports_both_halves_apart() {
        let node = node_token(81);
        let own = "node itself could not be removed.";
        let (list_origin, list_shape) = REGISTRY_WIDE_SHAPES[2];
        let (scan_origin, _) = REGISTRY_WIDE_SHAPES[0];
        let captured = vec![
            detected(&node),
            sub_cause(&node, own),
            refusal(&node, "InternalError"),
            line(LogLevel::Debug, list_origin, list_shape),
            line(
                LogLevel::Debug,
                scan_origin,
                "Unable to perform a full scan for dead nodes since the all existing nodes could not be listed (Interrupt).",
            ),
        ];

        let parts = classify_cleanup_failures(&captured);

        assert_eq!(
            parts.failures,
            vec![FailedNodeCleanup {
                node: node.clone(),
                variant: "InternalError".to_string(),
                causes: vec![format!("Unable to remove stale resources since the {own}")],
            }]
        );
        assert_eq!(
            parts.registry_errors,
            vec![captured[3].message.clone(), captured[4].message.clone()]
        );
    }
}
