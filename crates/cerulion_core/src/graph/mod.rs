// SPDX-License-Identifier: AGPL-3.0-only
//! Graph configuration loading and runtime execution.
//!
//! The graph YAML file (`graphs/<name>.yaml` in a workspace) is the single
//! source of truth for TOPOLOGY (Principle #5): node instances, their types
//! and the wiring between their ports. That is what enables static validation
//! and zero-discovery startup. It carries no trigger policy: a node's policy
//! is part of its TYPE, declared on the `#[cerulion_node]` macro, and a
//! `policy:` key in a graph file is refused as an unknown field.
//!
//! This module is the runtime side of that file. A user authors the YAML with
//! `cerulion graph create` and `cerulion node stage` and runs it with
//! `cerulion graph run`; nothing here is called from node code.
//!
//! # Topic Resolution
//!
//! `/{prefix}/{node_id}/{output_name}` (e.g., `/perception/camera/image`),
//! unless the output declares an absolute `topic:` override.
//!
//! # Execution Model
//!
//! `GraphRuntime::step()` performs:
//! 1. Pre-step drain: check all data-triggered subscriber inputs
//! 2. Bridge received data to `scheduler.signal_data()`
//! 3. Call `scheduler.step(delta)` to evaluate triggers and fire nodes

pub mod chain;
pub mod config;
pub mod drain_latch;
pub mod node;
pub mod partition;
pub mod runtime;
// Replay exit-3 widening: per-node tick-failure flood-suppression latch
// (the DrainWarnLatch sibling — pure state machine, mapped to tracing
// levels in runtime.rs).
pub mod tick_failure_latch;
pub mod topology;
pub mod validation;
// WaitSet reactor (event multiplexer) + determinism
// firewall — PRODUCTION code, not gated behind `#[cfg(test |
// test-helpers)]`: it is rooted by the production `GraphRuntime::run_live`
// live loop (which drives `step()` from real iceoryx2 wakeups instead of
// fixed-`delta` polling). The `run_waitset_reactor_once_for_test`
// seam stays gated (the firewall-proof harness); the reactor itself ships.
pub(crate) mod waitset;

pub use chain::{
    census_chains, ChainBar, ChainCensus, Colocation, ConsumerEdgeVerdict, ConsumerFireClass,
    FusedChain, MAX_FUSED_CHAIN_NODES,
};
pub use config::{
    GraphConfig, InputDef, NetworkBlock, NetworkMode, NodeDef, OutputDef, UNNAMED_GRAPH,
};
#[cfg(feature = "fuzz-helpers")]
pub use node::fuzz_parse_info_json;
#[cfg(any(test, feature = "test-helpers"))]
pub use node::ClosureNodeEntry;
pub use node::{DylibNodeEntry, NodeContext, NodeEntry, NodeInfo};
pub use partition::{
    auto_partition, baseline_process_per_node, block_colocation_seeds,
    compress_group_level_assignments, creditable_split_block_edges, derive_default_budget_ns,
    derive_fire_targets, derive_fire_targets_from_observations, derive_process_groups,
    harvest_costs, validate_partition, validate_process_groups, AutoPartition, BlockColocationSeed,
    CreditableSplitEdge, FireTargetPolicy, FusionRecord, HopCosts, PartitionCosts, ProcessGroup,
    ProfileResult, WarmupObservation, REMEDY_CO_LOCATE, REMEDY_SINGLE_PROCESS,
};
pub use runtime::{
    build_trigger_edges, compute_gateway_plan, validate_network_ingress, BuildPurpose,
    CreditBinding, CreditRole, CrossProcessWiring, GraphRuntime, BARRIER_BOUNDARY_TIMEOUT,
    EXTERNAL_TOPIC_SILENCE_GRACE_MS,
};
pub use topology::{
    resolve_levels, CreditBar, CycleError, GraphTopology, Level, LevelGrowth, Levels, RefineInputs,
    TriggerEdges,
};
pub use validation::{validate_graph, validate_graph_with, ValidationOptions};
pub use waitset::{check_waitset_attachment_capacity, WAITSET_MAX_ATTACHMENTS};

use crate::error::{TransportError, TransportResult};

// ─── Graph Parsing ───────────────────────────────────────────

/// Parse a YAML string into a `GraphConfig`.
///
/// When the YAML omits `prefix:`, it is resolved to the host's hostname
/// (with any trailing `.local` stripped, since macOS Bonjour names like
/// `robot.local` are noisy when used as topic prefixes). If the
/// `hostname` command is unavailable, falls back to `"localhost"`.
///
/// Use [`parse_graph_raw`] when round-tripping YAML through CLI tools
/// that should preserve the user-authored shape (e.g. keep an absent
/// `prefix:` line absent on rewrite).
pub fn parse_graph(yaml: &str) -> TransportResult<GraphConfig> {
    let mut config = parse_graph_raw(yaml)?;
    if config.prefix.is_empty() {
        config.prefix = default_prefix(config.identity());
    }
    Ok(config)
}

/// Parse a YAML string into a `GraphConfig` without filling in any
/// defaults. The returned config is exactly what was authored — empty
/// `prefix` stays empty so a subsequent `serde_yaml::to_string` does
/// not silently introduce a `prefix:` line the user did not write.
///
/// Older graph YAML carried a `policy:` block on each node. `NodeDef` no
/// longer has a `policy` field, and every graph-YAML type carries
/// `#[serde(deny_unknown_fields)]`, so such a file is now REFUSED by serde
/// with an error naming the unknown key and listing the accepted ones.
/// serde's message cannot say WHERE the setting went, though, so this
/// function still scans the raw YAML first and warns once per `policy:`
/// occurrence with the migration instruction — the warn is the remedy, the
/// parse error is the stop.
pub fn parse_graph_raw(yaml: &str) -> TransportResult<GraphConfig> {
    warn_on_legacy_policy_block(yaml);
    let mut config: GraphConfig =
        serde_yaml::from_str(yaml).map_err(|e| TransportError::GraphParseError {
            reason: e.to_string(),
        })?;
    // The file stem is the graph name, but a string has no
    // file, so a stemless parse SEEDS the identity from the deprecated `name:`
    // key. That is the only identity such a config can have, and it is what
    // keeps an older bag's embedded `graph.yaml` (which always carries one)
    // reading exactly as it did before.
    //
    // This is ONE of the two places `name` is touched at all — the other is
    // `adopt_file_stem_identity`, which OVERWRITES this seed the moment a stem
    // is known, so on every CLI path the seed is transient.
    // hot-path-alloc-ok: cold: graph YAML PARSE — once per `graph run`/`replay`, before any
    // node exists
    config.identity = config.name.clone().unwrap_or_default();
    Ok(config)
}

/// The key named by a `deny_unknown_fields` failure in a graph-YAML parse
/// error, if that is what the error is.
///
/// Every graph-YAML type denies unknown fields (the CI hardening
/// pass), so a parse error is one of two very different things: a key the
/// format no longer defines (repairable, which is what `bag migrate` is for),
/// or real YAML damage, which nothing can repair. Three call sites have to tell
/// them apart to decide what remedy to offer: [`crate::graph`]'s own callers in
/// `cerulion_cli_engine::replay_cmd` (the resim refusal), `bag_migrate` (the
/// strip loop), and `cerulion_bagd`'s capture judge.
///
/// It lives HERE, beside [`parse_graph_raw`], because it reads that function's
/// own error text: the two must move together, and three private copies —
/// which is exactly what this replaces — could not. `cerulion_bagd` in
/// particular cannot import the CLI engine (the engine already depends on
/// bagd), so a shared home had to be a crate both of them already depend on.
///
/// Tolerant of a wrapper: the caller may pass serde's raw message or
/// [`TransportError::GraphParseError`]'s rendered text, since the marker it
/// looks for survives either.
pub fn unknown_field_key(parse_error: &str) -> Option<String> {
    const MARKER: &str = "unknown field `";
    let at = parse_error.find(MARKER)? + MARKER.len();
    let rest = &parse_error[at..];
    let end = rest.find('`')?;
    let key = &rest[..end];
    // hot-path-alloc-ok: cold: this function is only ever reached from a graph-YAML
    // parse that has ALREADY FAILED — `serde_yaml` refused the document, and the
    // three callers are deciding which remedy to print. A run that parses its graph
    // never enters here at all, and a run that does not parse it has no nodes and no
    // hot path. `graph/mod.rs` is a scanned file (only `config.rs`/`node.rs`/
    // `partition.rs`/`topology.rs`/`validation.rs` are excluded), so a cold `String`
    // here is a deliberate, annotated decision like its sibling in `parse_graph_raw`.
    (!key.is_empty()).then(|| key.to_string())
}

/// Adopt the file stem as the graph's identity.
///
/// THE one place a resolved graph name enters a [`GraphConfig`]. Every CLI
/// verb resolves a graph by joining `graphs/{stem}.yaml`, so the stem is the
/// name the user typed and the name on disk; every identity surface derives
/// from it (run directories, bag stems, ring tags, Flashback capture names,
/// `graph=` log fields, cost-snapshot keys).
///
/// A legacy `name:` that DIVERGES from the stem gets ONE loud `warn!` naming
/// both values and saying which one is in force — a deprecation notice, never
/// a refusal. An ABSENT `name:` is silent (the shape `graph create` writes
/// from here on), and so is one that AGREES, since neither tells the author
/// anything they can act on.
pub fn adopt_file_stem_identity(config: &mut GraphConfig, stem: &str) {
    if let Some(declared) = config.name.as_deref() {
        if declared != stem {
            tracing::warn!(
                graph = %stem,
                declared_name = %declared,
                "graph YAML declares a `name:` that DIFFERS from the file stem, but a graph \
                 is named by its FILE — this run uses the stem (see `graph=`) and `name:` \
                 (see `declared_name=`) is IGNORED. Delete the `name:` line to silence this; \
                 every identity surface (run directory, bag file stem, ring tags, `graph=` \
                 log fields) already uses the stem."
            );
        }
    }
    // hot-path-alloc-ok: cold: graph identity RESOLUTION — once per verb, at load time
    config.identity = stem.to_string();
}

/// Walk the raw YAML text looking for a `policy:` line indented as
/// a node-level field (matches the `crates/cerulion_core/fixtures/test_graph.yaml`
/// indentation). Emits a `tracing::warn!` for each occurrence so a graph
/// written against the old format is told WHERE its trigger policy went — the
/// parse that follows then refuses the file (`deny_unknown_fields`).
fn warn_on_legacy_policy_block(yaml: &str) {
    for (lineno, raw_line) in yaml.lines().enumerate() {
        let line = raw_line.trim_end();
        // Match `   policy:` (any leading whitespace; trailing
        // colon optionally followed by inline content like `policy: {...}`).
        let trimmed = line.trim_start();
        if !(trimmed == "policy:"
            || trimmed.starts_with("policy:") && {
                // Reject `policy_xyz:` false-matches by requiring the
                // char immediately after `policy` to be `:`.
                trimmed.as_bytes().get(6) == Some(&b':')
            })
        {
            continue;
        }
        tracing::warn!(
            line = lineno + 1,
            "graph YAML carries a `policy:` block (the `line` field locates it). Trigger \
             policy lives on the macro side only \
             (`#[cerulion_node(period_ms = N)]`, `#[input(trigger)]`, etc.) — this block \
             is REJECTED (it used to be silently ignored). Move the policy declaration \
             into `nodes/<type>/src/lib.rs` and delete the YAML `policy:` block."
        );
    }
}

/// Compute the default topic prefix for a graph: hostname without any
/// `.local` suffix. If the `hostname` command is unavailable (e.g. a
/// stripped sandbox), falls back to `"localhost"` — never the graph
/// name, which would silently mask a misconfigured environment.
///
/// `_graph_name` is accepted for API stability with earlier versions
/// that fell back to it; it is intentionally unused.
// hot-path-alloc-ok-fn: cold: graph-LOAD topic-prefix derivation, once per graph
pub fn default_prefix(_graph_name: &str) -> String {
    hostname_sans_local().unwrap_or_else(|| "localhost".to_string())
}

/// The env-var name that overrides the robot's network identity.
///
/// Kept HERE (the single source of the identity-resolution contract) so the
/// gateway (`cerulion_cli_engine::graph_cmd::resolve_robot_identity`) and the
/// remote-plane daemon (`cerulion_remoted`, which cannot reach the CLI engine)
/// resolve the SAME override — otherwise the LAN and remote planes could
/// self-attribute a robot under two different names.
pub const ROBOT_IDENTITY_ENV: &str = "CERULION_ROBOT_IDENTITY";

/// Resolve the robot's network identity: the `CERULION_ROBOT_IDENTITY` env
/// override (trimmed, non-empty wins) → else the hostname ([`default_prefix`],
/// `.local`-stripped, `localhost` fallback).
///
/// This is the SINGLE override-aware resolver every plane must call. The gateway
/// resolves identity through the SAME precedence (env override → `default_prefix`;
/// `default_prefix` ignores its graph-name argument, so env → hostname is the
/// identical contract), and the remote wire plane's catalog (`CatalogReply.robot`)
/// calls THIS so a fleet image that pins `CERULION_ROBOT_IDENTITY` announces one
/// consistent identity across the LAN gateway AND the remote (iroh) plane.
///
/// Identity is a NETWORK-only concern — it never appears in topic names or bags
/// (those use the graph prefix), so resolving it at runtime from the environment
/// is replay-safe.
// hot-path-alloc-ok-fn: cold: reads the robot-identity env once at graph LOAD
pub fn robot_identity_from_env() -> String {
    if let Ok(v) = std::env::var(ROBOT_IDENTITY_ENV) {
        let trimmed = v.trim();
        if !trimmed.is_empty() {
            return trimmed.to_string();
        }
    }
    default_prefix("")
}

// hot-path-alloc-ok-fn: cold: hostname normalisation, once at graph LOAD
fn hostname_sans_local() -> Option<String> {
    let raw = std::process::Command::new("hostname")
        .output()
        .ok()
        .and_then(|o| {
            if o.status.success() {
                Some(String::from_utf8_lossy(&o.stdout).trim().to_string())
            } else {
                None
            }
        })?;
    if raw.is_empty() {
        return None;
    }
    Some(raw.strip_suffix(".local").unwrap_or(&raw).to_string())
}

// ─── Topic Resolution ────────────────────────────────────────

/// Resolve an output to its topic: the `topic:` override verbatim when set
/// (an absolute name, validated at graph load), else the derived
/// `/{prefix}/{node_id}/{name}` (canonical leading-slash form, ROS2
/// convention — the slash is part of the iceoryx2 service name and
/// doubles as the absolute-reference escape marker in graph YAML).
///
/// This is the only producer-side TOPIC RESOLUTION: the raw
/// `resolve_topic` building block was deleted-by-inlining so a
/// producer-side call site structurally cannot bypass the override
/// (visibility demotion could not protect the graph-internal callers —
/// Rust visibility is subtree-based). One deliberate formula duplicate
/// exists: validation's in-prefix did-you-mean re-derives the DERIVED
/// spelling of an overridden output for comparison only — it cannot
/// route through this resolver (which would return the override).
// hot-path-alloc-ok-fn: cold: resolves one output port's topic NAME at graph BUILD
pub fn resolve_output_topic(prefix: &str, node_id: &str, output: &config::OutputDef) -> String {
    match &output.topic {
        Some(t) => t.clone(),
        None => format!("/{}/{}/{}", prefix, node_id, output.name),
    }
}

/// One shape predicate for absolute topic
/// names (`source:` references, `topic:` overrides) shared by every
/// ingress — validation previously carried two identical hand-rolled
/// copies (the prefix guard is a deliberately separate sibling: prefixes
/// are relative names with their own edge rules).
/// Returns the rejection reason for malformed shapes. Zenoh-reserved
/// characters are rejected too: topic names become zenoh key chunks when
/// bridging, and graph-load is a better failure point than a runtime
/// zenoh put error.
///
/// `pub` (was `pub(crate)`) because `cerulion ros2 attach`'s
/// `--topic-prefix` validation PROBES this exact predicate (prefix +
/// `"/probe"`) so its reject set can never drift NARROWER than graph
/// load's — a prefix that would doom every generated `topic:` override is
/// refused before any file is written, with this predicate's reason
/// surfaced verbatim (single source of truth, no second hand-rolled list).
pub fn malformed_absolute_name(s: &str) -> Option<&'static str> {
    if s.len() == 1 {
        return Some("a bare '/' names no topic");
    }
    if s.ends_with('/') {
        return Some("trailing '/' (empty final segment)");
    }
    if s.contains("//") {
        return Some("empty segment ('//')");
    }
    if s.contains(['*', '?', '#', '$', '@']) {
        return Some(
            "zenoh-reserved character ('*', '?', '#', '$', '@') — topic \
             names must be valid zenoh key chunks for network bridging \
             ('@' marks a verbatim chunk with special matching semantics)",
        );
    }
    None
}

/// Resolve an input source reference to a full topic path.
///
/// Relative source `{source_node_id}/{output_name}` resolves to
/// `/{prefix}/{source_node_id}/{output_name}` (canonical leading-slash
/// form). A source starting with `/` is an ABSOLUTE reference and passes
/// through verbatim — it bypasses the prefix entirely, naming either
/// another graph's topic or an external one (the
/// runtime provisions producer-less external topics with the
/// buffer-ceiling-only `PublisherProvisioning::External` shape).
// hot-path-alloc-ok-fn: cold: resolves one input edge's source topic NAME at graph BUILD
pub fn resolve_source(prefix: &str, source: &str) -> String {
    if source.starts_with('/') {
        source.to_string()
    } else {
        format!("/{}/{}", prefix, source)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tracing_test::traced_test;

    // ── The shared unknown-key reader ──────────────────────────

    /// It reads the key, and it reads it out of the REAL error text —
    /// including the wrapped [`TransportError::GraphParseError`] rendering,
    /// which is what two of the three call sites actually hold.
    #[test]
    fn the_unknown_key_reader_reads_the_key_and_nothing_else() {
        // Raw serde, top level and nested (serde prefixes the container path).
        assert_eq!(
            unknown_field_key("unknown field `policy`, expected one of `name`").as_deref(),
            Some("policy")
        );
        assert_eq!(
            unknown_field_key("nodes[0].outputs[0]: unknown field `dpeth`, expected").as_deref(),
            Some("dpeth")
        );

        // The WRAPPED form, produced by the parser this reader lives beside —
        // built by really failing a parse rather than by pasting a string, so
        // a change to either half fails here.
        let wrapped = parse_graph_raw("nodes:\n  - id: a\n    type: t\n    policy: 1\n")
            .expect_err("an undefined key must refuse")
            .to_string();
        assert!(
            wrapped.contains("Failed to parse graph YAML"),
            "the wrapper must still be there, or this arm proves nothing: {wrapped}"
        );
        assert_eq!(unknown_field_key(&wrapped).as_deref(), Some("policy"));

        // NOT an unknown-key failure: real YAML damage, and a type error. Both
        // must answer None, because each call site branches on exactly that.
        let damaged = parse_graph_raw("nodes: [")
            .expect_err("malformed YAML must refuse")
            .to_string();
        assert_eq!(unknown_field_key(&damaged), None, "got: {damaged}");
        assert_eq!(
            unknown_field_key("invalid type: string \"x\", expected a sequence at line 3"),
            None
        );
        // A truncated marker names no key.
        assert_eq!(unknown_field_key("unknown field ``, expected"), None);
        assert_eq!(unknown_field_key("unknown field `"), None);
    }

    // ── The file stem is the graph name ──────────────────────────────────
    //
    // These arms drive the two — and only two — places the deprecated `name:`
    // key is touched at all: `parse_graph_raw`'s stemless SEED and
    // `adopt_file_stem_identity`'s divergence warn. Every oracle is
    // hand-written; nothing here compares one parse against another.

    /// The HEADLINE: a `name:` that disagrees with the file stem is IGNORED,
    /// and says so ONCE.
    ///
    /// The stem wins on the value AND the warning names both sides, because an
    /// operator who sees `graph=nav2` in a log while their file says
    /// `name: nav2_mobile_autonomy` has no way to connect the two otherwise.
    /// The level token is matched as well as the message: a deprecation the
    /// default filter drops is a deprecation nobody is told about.
    #[test]
    #[traced_test]
    fn a_declared_name_that_diverges_from_the_file_stem_is_ignored_and_warned_once() {
        let yaml = "name: nav2_mobile_autonomy\nnodes: []\n";
        let mut config = parse_graph_raw(yaml).expect("legacy YAML must still parse");
        // The stemless SEED: with no file in sight the declared name is the
        // only identity there is.
        assert_eq!(
            config.identity(),
            "nav2_mobile_autonomy",
            "a stemless parse seeds the identity from the legacy key"
        );

        adopt_file_stem_identity(&mut config, "nav2");

        assert_eq!(config.identity(), "nav2", "the FILE STEM wins");
        assert_eq!(
            config.name.as_deref(),
            Some("nav2_mobile_autonomy"),
            "the deprecated key is KEPT so `node stage` round-trips it"
        );

        logs_assert(|lines: &[&str]| {
            let warns = count_at(lines, "WARN", "nav2_mobile_autonomy");
            if warns == 1 {
                Ok(())
            } else {
                Err(format!(
                    "exactly ONE deprecation warn, at WARN; got {warns}"
                ))
            }
        });
        assert!(
            logs_contain("nav2"),
            "the warn must name the stem that is actually in force"
        );
    }

    /// An ABSENT `name:` is the shape `graph create` writes from here on, so it
    /// must be SILENT — a deprecation notice on a file that already complies is
    /// noise an operator cannot act on.
    #[test]
    #[traced_test]
    fn a_graph_with_no_declared_name_adopts_the_stem_silently() {
        let mut config = parse_graph_raw("nodes: []\n").expect("a nameless graph must parse");
        assert_eq!(
            config.identity(),
            crate::graph::UNNAMED_GRAPH,
            "nothing could name it yet"
        );
        assert!(config.name.is_none());

        adopt_file_stem_identity(&mut config, "perception");

        assert_eq!(config.identity(), "perception");
        assert!(config.name.is_none(), "nothing is fabricated into the key");
        logs_assert(
            |lines: &[&str]| match count_at(lines, "WARN", "perception") {
                0 => Ok(()),
                n => Err(format!(
                    "an absent `name:` must warn about nothing; got {n}"
                )),
            },
        );
    }

    /// The other silent case, and the ANTI-TAUTOLOGY control for the headline:
    /// without it, "exactly one warn" would also be satisfied by a warn that
    /// fires on every graph carrying the key at all.
    #[test]
    #[traced_test]
    fn a_declared_name_that_agrees_with_the_stem_is_silent() {
        let mut config = parse_graph_raw("name: perception\nnodes: []\n").unwrap();
        adopt_file_stem_identity(&mut config, "perception");
        assert_eq!(config.identity(), "perception");
        logs_assert(
            |lines: &[&str]| match count_at(lines, "WARN", "perception") {
                0 => Ok(()),
                n => Err(format!(
                    "a `name:` that already matches its file tells the author nothing; got {n}"
                )),
            },
        );
    }

    /// BACK-COMPAT, stated as bytes: an old graph round-trips its `name:` line
    /// and a new one emits none. `graph_write` / `node stage` re-serialize the
    /// whole config, so this is what keeps a line from being silently deleted
    /// out from under an author — and what keeps one from appearing.
    #[test]
    fn the_deprecated_key_round_trips_and_the_identity_never_serializes() {
        let legacy = parse_graph_raw("name: legacy\nnodes: []\n").unwrap();
        let out = serde_yaml::to_string(&legacy).expect("serialize");
        assert!(
            out.contains("name: legacy"),
            "a legacy `name:` must survive a rewrite, got:\n{out}"
        );

        let mut fresh = parse_graph_raw("nodes: []\n").unwrap();
        adopt_file_stem_identity(&mut fresh, "fresh");
        let out = serde_yaml::to_string(&fresh).expect("serialize");
        assert!(
            !out.contains("name:"),
            "a nameless graph must emit no `name:`, got:\n{out}"
        );
        // The identity itself must not leak into the document EITHER, under any
        // spelling. `graph_write` (`node stage`) and `render_effective_graph_yaml`
        // both re-serialize the whole config, so a serializing identity would
        // inject a key nobody wrote into the user's own file and into every bag's
        // `graph.yaml`. This assertion is the ONLY thing that sees it: the
        // stemless SEED unconditionally overwrites `identity` on the way back in,
        // so a round-trip through `parse_graph_raw` cannot tell the two apart.
        assert!(
            !out.contains("identity"),
            "the resolved identity must NEVER be written into a graph file, got:\n{out}"
        );
        // Re-reading gives no identity back — which is exactly why every load
        // seam calls `adopt_file_stem_identity`.
        assert_eq!(
            parse_graph_raw(&out).unwrap().identity(),
            crate::graph::UNNAMED_GRAPH
        );
    }

    /// An OLD BAG's embedded `graph.yaml` always carries a `name:`, and nothing
    /// stems it — so `parse_graph`'s seed is what keeps an older recording
    /// reading exactly as it did. Pinned on the parser replay actually uses.
    #[test]
    fn a_pre_ruling_bags_embedded_graph_still_names_itself() {
        let embedded = "name: recorded_run\nprefix: robot\nnodes: []\n";
        let config = parse_graph(embedded).expect("an old bag's graph must still parse");
        assert_eq!(config.identity(), "recorded_run");
        assert_eq!(config.prefix, "robot");
    }

    /// Count log lines that carry BOTH a level token and a needle.
    ///
    /// PURE, and called from inside each test's own `logs_assert` closure —
    /// `tracing-test` injects `logs_assert`/`logs_contain` into the annotated
    /// FUNCTION, so a module-level helper cannot reach them. The level is read
    /// as a whole whitespace token out of the line header:
    /// `tracing-test` renders the span name — the test fn's own name — into
    /// every line, so a bare `contains("WARN")` is not a level check.
    fn count_at(lines: &[&str], level: &str, needle: &str) -> usize {
        lines
            .iter()
            .filter(|l| l.split_whitespace().take(4).any(|t| t == level) && l.contains(needle))
            .count()
    }

    #[test]
    fn test_parse_minimal_graph() {
        let yaml = r#"
name: test
nodes:
  - id: pub1
    type: test_pub
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: 1024
"#;
        let config = parse_graph(yaml).unwrap();
        assert_eq!(config.identity(), "test");
        assert_eq!(config.prefix, default_prefix("test"));
        assert!(!config.prefix.is_empty());
        assert_eq!(config.nodes.len(), 1);
        assert_eq!(config.nodes[0].id, "pub1");
    }

    #[test]
    fn test_parse_prefix_defaults_to_hostname_or_name() {
        let yaml = r#"
name: perception
nodes:
  - id: cam
    type: camera
"#;
        let config = parse_graph(yaml).unwrap();
        assert_eq!(config.prefix, default_prefix("perception"));
        assert!(!config.prefix.is_empty());
        assert!(
            !config.prefix.ends_with(".local"),
            "prefix should not carry a `.local` suffix: got {}",
            config.prefix
        );
    }

    #[test]
    fn test_parse_explicit_prefix() {
        let yaml = r#"
name: perception
prefix: custom_prefix
nodes:
  - id: cam
    type: camera
"#;
        let config = parse_graph(yaml).unwrap();
        assert_eq!(config.prefix, "custom_prefix");
    }

    #[test]
    fn test_resolve_output_topic() {
        // Derived arm (hand-pasted literal oracle — resolve_topic was
        // deleted by inlining).
        let derived = config::OutputDef {
            name: "image".to_string(),
            schema: String::new(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        };
        assert_eq!(
            resolve_output_topic("perception", "camera", &derived),
            "/perception/camera/image"
        );
        // Override arm: verbatim.
        let overridden = config::OutputDef {
            topic: Some("/tf".to_string()),
            ..derived
        };
        assert_eq!(
            resolve_output_topic("perception", "camera", &overridden),
            "/tf"
        );
    }

    #[test]
    fn test_resolve_source() {
        assert_eq!(
            resolve_source("perception", "camera/image"),
            "/perception/camera/image"
        );
        // Absolute references bypass the prefix verbatim.
        assert_eq!(resolve_source("perception", "/ext/cam"), "/ext/cam");
        assert_eq!(
            resolve_source("perception", "/perception/camera/image"),
            "/perception/camera/image"
        );
    }

    #[test]
    fn test_parse_invalid_yaml_rejected() {
        let yaml = "{{{{invalid yaml!!!";
        let result = parse_graph(yaml);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_node_with_inputs_and_outputs() {
        // Trigger policy lives only on the macro side, so
        // graph YAML carries no `policy:` block. The two-node graph
        // below is the smallest non-trivial topology that shows
        // input/output wiring still parses cleanly with no policy
        // hint at the YAML level.
        let yaml = r#"
name: test
nodes:
  - id: pub1
    type: pub
    outputs:
      - name: data
        max_slice_len: 1024
  - id: sub1
    type: sub
    inputs:
      - name: data
        source: pub1/data
"#;
        let config = parse_graph(yaml).unwrap();
        assert_eq!(config.nodes.len(), 2);
        assert_eq!(config.nodes[1].inputs.len(), 1);
        assert_eq!(config.nodes[1].inputs[0].source, "pub1/data");
    }

    /// RAII guard that restores `CERULION_ROBOT_IDENTITY` to its prior state
    /// (present-with-value or absent) on drop — panic-safe, so a failing
    /// assertion never leaks the override into a sibling test.
    struct RobotIdentityEnvGuard {
        prior: Option<String>,
    }

    impl RobotIdentityEnvGuard {
        fn set(value: &str) -> Self {
            let prior = std::env::var(ROBOT_IDENTITY_ENV).ok();
            std::env::set_var(ROBOT_IDENTITY_ENV, value);
            Self { prior }
        }

        fn unset() -> Self {
            let prior = std::env::var(ROBOT_IDENTITY_ENV).ok();
            std::env::remove_var(ROBOT_IDENTITY_ENV);
            Self { prior }
        }
    }

    impl Drop for RobotIdentityEnvGuard {
        fn drop(&mut self) {
            match &self.prior {
                Some(v) => std::env::set_var(ROBOT_IDENTITY_ENV, v),
                None => std::env::remove_var(ROBOT_IDENTITY_ENV),
            }
        }
    }

    /// `robot_identity_from_env`: env unset → hostname (== `default_prefix("")`);
    /// env set → the trimmed override; whitespace-only / empty → falls back to
    /// hostname. `#[serial]` (the env is process-global); env restored on drop.
    #[test]
    #[serial_test::serial]
    fn robot_identity_env_override_then_hostname_fallback() {
        // Unset → hostname (the same value default_prefix resolves).
        {
            let _g = RobotIdentityEnvGuard::unset();
            assert_eq!(
                robot_identity_from_env(),
                default_prefix(""),
                "unset override must resolve to the hostname"
            );
        }
        // Set to a concrete fleet name → that exact name.
        {
            let _g = RobotIdentityEnvGuard::set("fleet-bot");
            assert_eq!(robot_identity_from_env(), "fleet-bot");
        }
        // A value with surrounding whitespace is TRIMMED (non-empty wins).
        {
            let _g = RobotIdentityEnvGuard::set("  padded-bot  ");
            assert_eq!(robot_identity_from_env(), "padded-bot");
        }
        // Whitespace-only → treated as empty → hostname fallback.
        {
            let _g = RobotIdentityEnvGuard::set("   ");
            assert_eq!(
                robot_identity_from_env(),
                default_prefix(""),
                "a whitespace-only override falls back to the hostname"
            );
        }
        // Empty string → hostname fallback.
        {
            let _g = RobotIdentityEnvGuard::set("");
            assert_eq!(
                robot_identity_from_env(),
                default_prefix(""),
                "an empty override falls back to the hostname"
            );
        }
    }
}
