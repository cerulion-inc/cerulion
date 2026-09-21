// SPDX-License-Identifier: AGPL-3.0-only
//! PURE tests for the `cerulion graph profile` artifact
//! layer — the [`ProfileArtifact`] serde schema (frozen on-disk shape),
//! the lossless `ProfileResult → artifact → (costs, isolated)` round-trip,
//! the loud hand-edit validation arms, and the report/path helpers.
//!
//! No iceoryx2, no transport, no clock — parallel-safe. The live harness
//! (`graph_profile` itself) is covered by `graph_profile_iox2_test.rs`
//! (run separately: it needs prebuilt fixture cdylibs and `--test-threads=1`).

use std::collections::{BTreeMap, BTreeSet};
use std::num::NonZeroUsize;
use std::path::Path;
use std::time::Duration;

use cerulion_cli_engine::graph_cmd::{
    all_targets_met, build_warmup_observations, default_artifact_path, parse_profile_artifact,
    poll_warmup_sightings, profile_warmup_duration, warmup_burst_fires, IsolatedNodeReport,
    NodeWarmupSighting, ProfileArtifact, ProfileEdge, ProfileReport, PROFILE_ARTIFACT_VERSION,
};
use cerulion_core::graph::{HopCosts, PartitionCosts, ProfileResult, WarmupObservation};

/// A hand-built mixed `ProfileResult`: two costed nodes, one edge, one
/// isolated node — the canonical round-trip fixture.
fn sample_profile() -> ProfileResult {
    ProfileResult {
        costs: PartitionCosts {
            node_p50_ns: [("sink".to_string(), 850), ("ticker".to_string(), 1200)]
                .into_iter()
                .collect(),
            edge_rate_mhz: [(("ticker".to_string(), "sink".to_string()), 20_000)]
                .into_iter()
                .collect(),
            // Find-pass #6: DISTINCT from every `HopCosts::platform_default()`
            // variant (macOS 1000/9000, x86_64-linux 600/6800, aarch64-linux
            // 2100/6800) so the to_costs Some-arm is mutation-proof on ALL
            // platforms — a revert to platform_default() can never coincide.
            hop: HopCosts {
                intra_ns: 12_345,
                cross_ns: 67_890,
            },
        },
        isolated: ["laggard".to_string()].into_iter().collect(),
    }
}

/// The STUBBED profiling-box core count for tests — from_profile
/// takes cores as a parameter precisely so tests never call
/// `available_parallelism` and assert a literal.
fn cores(n: usize) -> NonZeroUsize {
    NonZeroUsize::new(n).expect("test core count is nonzero")
}

// ==========================================================================
// The frozen on-disk schema. The YAML string is a HAND-WRITTEN oracle (never
// derived from the serializer under test) — pins field names, nesting, and
// declaration order so an accidental rename/reorder is a loud test failure,
// not a silent format break for every existing artifact on disk.
// ==========================================================================

const SAMPLE_YAML: &str = "\
version: 2
graph: demo
window_ns: 5000000000
nodes:
  sink: 850
  ticker: 1200
edges:
- producer: ticker
  consumer: sink
  rate_mhz: 20000
isolated:
- laggard
hop:
  intra_ns: 12345
  cross_ns: 67890
derived_budget_ns: 513
profile_cores: 4
";

#[test]
fn artifact_serializes_to_the_frozen_yaml_shape() {
    // Cores STUBBED at 4 => derived_budget_ns = ceil((850+1200)/4)
    // = ceil(2050/4) = 513 in the frozen YAML (hand-computed).
    let artifact =
        ProfileArtifact::from_profile("demo", 5_000_000_000, &sample_profile(), cores(4));
    let yaml = serde_yaml::to_string(&artifact).expect("serialize");
    assert_eq!(
        yaml, SAMPLE_YAML,
        "the on-disk schema is FROZEN — a field rename/reorder breaks every artifact on disk"
    );
}

#[test]
fn artifact_yaml_round_trip_is_lossless() {
    // ProfileResult → artifact → YAML → artifact → (costs, isolated) must
    // reproduce the ORIGINAL (costs, isolated) pair exactly.
    let profile = sample_profile();
    let artifact = ProfileArtifact::from_profile("demo", 5_000_000_000, &profile, cores(4));
    let yaml = serde_yaml::to_string(&artifact).expect("serialize");
    let read_back: ProfileArtifact = serde_yaml::from_str(&yaml).expect("deserialize");
    assert_eq!(read_back, artifact, "artifact survives the YAML round-trip");

    let (costs, isolated) = read_back.to_costs().expect("to_costs");
    assert_eq!(costs, profile.costs, "costs are lossless through the file");
    // Find-pass #6: the file's hop values (distinct from EVERY platform
    // default) must flow through EXACTLY — an implementation that substitutes
    // platform_default() fails here on every platform.
    assert_eq!(costs.hop.intra_ns, 12_345, "file hop intra flows through");
    assert_eq!(costs.hop.cross_ns, 67_890, "file hop cross flows through");
    assert_ne!(
        costs.hop,
        HopCosts::platform_default(),
        "fixture hop must stay distinct from this host's platform default (anti-vacuity guard for \
         the assert above)"
    );
    assert_eq!(
        isolated, profile.isolated,
        "the isolated set is lossless through the file"
    );
    // Provenance fields travel too.
    assert_eq!(read_back.graph, "demo");
    assert_eq!(read_back.window_ns, 5_000_000_000);
    assert_eq!(read_back.version, PROFILE_ARTIFACT_VERSION);
    // The frozen budget + its core-count provenance survive the file
    // round-trip exactly (Σ 2050 / stubbed 4 cores, ceil => 513).
    assert_eq!(read_back.derived_budget_ns, Some(513));
    assert_eq!(read_back.profile_cores, Some(4));
}

#[test]
fn hand_written_yaml_without_hop_falls_back_to_platform_default() {
    // A hand-edited file may DELETE the hop block; the reader then uses the
    // READING host's platform default (the freshly-written file always carries
    // the block — only a hand-edit reaches this arm).
    let yaml = "\
version: 1
graph: demo
window_ns: 1000000000
nodes:
  a: 100
edges: []
isolated: []
";
    let artifact: ProfileArtifact = serde_yaml::from_str(yaml).expect("deserialize");
    assert!(artifact.hop.is_none(), "the hop block is genuinely absent");
    let (costs, _) = artifact.to_costs().expect("to_costs");
    assert_eq!(
        costs.hop,
        HopCosts::platform_default(),
        "a missing hop block falls back to the reading host's platform default"
    );
}

// ==========================================================================
// deny_unknown_fields — a typo'd key in a hand-edited file is a LOUD parse
// error (the project's loud-over-silent rule), at every nesting level.
// ==========================================================================

#[test]
fn unknown_top_level_field_is_rejected() {
    // `isolatd` (typo'd `isolated`) must be a loud parse error, not a
    // silently-ignored key that leaves the intended list empty.
    let yaml = "\
version: 1
graph: demo
window_ns: 1
nodes: {}
edges: []
isolatd:
- laggard
";
    let err = serde_yaml::from_str::<ProfileArtifact>(yaml)
        .expect_err("a typo'd top-level key must be rejected")
        .to_string();
    assert!(
        err.contains("isolatd"),
        "the error must name the unknown field; got: {err}"
    );
}

#[test]
fn unknown_edge_field_is_rejected() {
    // `rate_hz` (typo'd `rate_mhz`) inside an edge entry must also reject —
    // silently defaulting the intended rate to 0 would mean "never fuse".
    let yaml = "\
version: 1
graph: demo
window_ns: 1
nodes: {}
edges:
- producer: a
  consumer: b
  rate_hz: 5
isolated: []
";
    let err = serde_yaml::from_str::<ProfileArtifact>(yaml)
        .expect_err("a typo'd edge key must be rejected")
        .to_string();
    assert!(
        err.contains("rate_hz"),
        "the error must name the unknown edge field; got: {err}"
    );
}

#[test]
fn unknown_hop_field_is_rejected() {
    let yaml = "\
version: 1
graph: demo
window_ns: 1
nodes: {}
edges: []
isolated: []
hop:
  intra_ns: 1000
  cross_nanos: 9000
";
    let err = serde_yaml::from_str::<ProfileArtifact>(yaml)
        .expect_err("a typo'd hop key must be rejected")
        .to_string();
    assert!(
        err.contains("cross_nanos"),
        "the error must name the unknown hop field; got: {err}"
    );
}

// ==========================================================================
// to_costs validation arms — loud, never a silent repair.
// ==========================================================================

#[test]
fn unsupported_version_is_rejected_naming_supported() {
    // Version 2 is CURRENT, so the too-new example is 3; the message names
    // the file's version AND the supported RANGE (1..=2 — v1 stays readable).
    let yaml = "\
version: 3
graph: demo
window_ns: 1
nodes: {}
edges: []
isolated: []
";
    let artifact: ProfileArtifact = serde_yaml::from_str(yaml).expect("parses (shape is fine)");
    let err = artifact
        .to_costs()
        .expect_err("an unknown version must be rejected at consumption")
        .to_string();
    assert!(
        err.contains("version 3") && err.contains("1..=2"),
        "the error must name both the file's version and the supported range; got: {err}"
    );
}

// ==========================================================================
// The frozen core-count budget in the v2 artifact — exact freeze
// pins (stubbed cores, hand oracles), v1 back-compat, the total==0 None
// contract, and the version-FIRST parse helper.
// ==========================================================================

#[test]
fn from_profile_freezes_ceil_budget_and_cores_exactly() {
    // Σ p50 = 500 + 501 = 1001; stubbed cores = 4 => ceil(1001/4) = 251
    // (floor division would freeze 250). Hand oracle, never derived from the
    // machinery under test.
    let profile = ProfileResult {
        costs: PartitionCosts {
            node_p50_ns: [("a".to_string(), 500), ("b".to_string(), 501)]
                .into_iter()
                .collect(),
            edge_rate_mhz: BTreeMap::new(),
            hop: HopCosts {
                intra_ns: 1,
                cross_ns: 2,
            },
        },
        isolated: BTreeSet::new(),
    };
    let artifact = ProfileArtifact::from_profile("demo", 1, &profile, cores(4));
    assert_eq!(
        artifact.derived_budget_ns,
        Some(251),
        "ceil(1001/4) = 251, frozen at profile time"
    );
    assert_eq!(
        artifact.profile_cores,
        Some(4),
        "the stubbed core count is frozen as provenance"
    );
    assert_eq!(artifact.version, PROFILE_ARTIFACT_VERSION, "written as v2");
}

#[test]
fn v1_artifact_parses_with_both_budget_fields_none() {
    // A pre-budget file (version: 1, no budget fields) must parse under the
    // v2 reader with both fields None AND still thaw through to_costs.
    let yaml = "\
version: 1
graph: demo
window_ns: 1000000000
nodes:
  a: 100
edges: []
isolated: []
";
    let artifact = parse_profile_artifact(yaml).expect("a v1 file parses under the v2 reader");
    assert_eq!(artifact.version, 1);
    assert_eq!(
        artifact.derived_budget_ns, None,
        "v1 carries no frozen budget"
    );
    assert_eq!(artifact.profile_cores, None, "v1 carries no cores field");
    let (costs, _) = artifact
        .to_costs()
        .expect("v1 thaws (version gate accepts 1)");
    assert_eq!(costs.node_p50_ns.get("a"), Some(&100));
}

#[test]
fn all_isolated_profile_freezes_no_budget() {
    // total == 0 (nothing costed — every node isolated): BOTH fields None.
    // Freezing the degenerate max(1) floor would smuggle a meaningless
    // budget into the artifact.
    let profile = ProfileResult {
        costs: PartitionCosts {
            node_p50_ns: BTreeMap::new(),
            edge_rate_mhz: BTreeMap::new(),
            hop: HopCosts {
                intra_ns: 1,
                cross_ns: 2,
            },
        },
        isolated: ["lonely".to_string()].into_iter().collect(),
    };
    let artifact = ProfileArtifact::from_profile("demo", 1, &profile, cores(8));
    assert_eq!(
        artifact.derived_budget_ns, None,
        "no costed compute => no frozen budget"
    );
    assert_eq!(
        artifact.profile_cores, None,
        "cores provenance is only written alongside a budget"
    );
    // And the serialized YAML simply omits both keys (skip_serializing_if).
    let yaml = serde_yaml::to_string(&artifact).expect("serialize");
    assert!(
        !yaml.contains("derived_budget_ns") && !yaml.contains("profile_cores"),
        "absent fields are OMITTED from the file, not written as nulls; got:\n{yaml}"
    );
}

#[test]
fn parse_helper_refuses_future_version_on_version_message_not_serde() {
    // A FUTURE-version file carrying a field this build has never heard of:
    // the strict deny_unknown_fields parse alone would die on a raw serde
    // unknown-field error, burying the diagnosis. The version-FIRST probe
    // must surface the VERSION message instead.
    let yaml = "\
version: 3
graph: demo
window_ns: 1
nodes: {}
edges: []
isolated: []
some_v3_field: 42
";
    let err = parse_profile_artifact(yaml)
        .expect_err("a future version must be refused")
        .to_string();
    assert!(
        err.contains("version 3") && err.contains("1..=2"),
        "the refusal must be the VERSION message, not a serde unknown-field error; got: {err}"
    );
    assert!(
        !err.contains("some_v3_field"),
        "the unknown field must not be the headline diagnosis; got: {err}"
    );

    // Control (anti-tautology): a SUPPORTED-version file with a typo'd key
    // still fails LOUDLY on the strict parse — the probe does not launder
    // hand-edit typos.
    let typo = "\
version: 2
graph: demo
window_ns: 1
nodes: {}
edges: []
isolated: []
derived_budget_nz: 42
";
    let err = parse_profile_artifact(typo)
        .expect_err("a typo'd key in a supported-version file must still be rejected")
        .to_string();
    assert!(
        err.contains("malformed"),
        "supported version + unknown key => the strict-parse malformed error; got: {err}"
    );
}

#[test]
fn duplicate_edge_is_rejected_loudly() {
    // The Vec encoding makes duplicates representable; last-one-wins would
    // silently drop a rate a hand-editor thought they set.
    let artifact = ProfileArtifact {
        version: 1,
        graph: "demo".to_string(),
        window_ns: 1,
        nodes: BTreeMap::new(),
        edges: vec![
            ProfileEdge {
                producer: "a".to_string(),
                consumer: "b".to_string(),
                rate_mhz: 10,
            },
            ProfileEdge {
                producer: "a".to_string(),
                consumer: "b".to_string(),
                rate_mhz: 99,
            },
        ],
        isolated: Vec::new(),
        hop: None,
        derived_budget_ns: None,
        profile_cores: None,
    };
    let err = artifact
        .to_costs()
        .expect_err("a duplicate (producer, consumer) edge must be rejected")
        .to_string();
    assert!(
        err.contains('a') && err.contains('b') && err.contains("more than once"),
        "the error must name the duplicated pair; got: {err}"
    );
}

#[test]
fn node_in_both_nodes_and_isolated_is_rejected() {
    // Contradiction: an isolated node must carry NO cost.
    let artifact = ProfileArtifact {
        version: 1,
        graph: "demo".to_string(),
        window_ns: 1,
        nodes: [("dual".to_string(), 100)].into_iter().collect(),
        edges: Vec::new(),
        isolated: vec!["dual".to_string()],
        hop: None,
        derived_budget_ns: None,
        profile_cores: None,
    };
    let err = artifact
        .to_costs()
        .expect_err("a node in both nodes: and isolated: must be rejected")
        .to_string();
    assert!(
        err.contains("dual") && err.contains("BOTH"),
        "the error must name the contradictory node; got: {err}"
    );
}

#[test]
fn duplicate_isolated_entries_dedup_to_set_semantics() {
    // The isolated list has SET semantics — a duplicated entry changes
    // nothing (unlike a duplicated edge, which silently overrides a rate),
    // so it is deduped rather than rejected.
    let artifact = ProfileArtifact {
        version: 1,
        graph: "demo".to_string(),
        window_ns: 1,
        nodes: BTreeMap::new(),
        edges: Vec::new(),
        isolated: vec!["x".to_string(), "x".to_string()],
        hop: None,
        derived_budget_ns: None,
        profile_cores: None,
    };
    let (_, isolated) = artifact.to_costs().expect("to_costs");
    assert_eq!(
        isolated,
        ["x".to_string()].into_iter().collect::<BTreeSet<_>>()
    );
}

// ==========================================================================
// Helpers: default path + report shape.
// ==========================================================================

#[test]
fn default_artifact_path_is_costs_yaml_next_to_the_graph() {
    let path = default_artifact_path(Path::new("/ws"), "demo");
    assert_eq!(path, Path::new("/ws/graphs/demo.costs.yaml"));
}

#[test]
fn report_carries_the_run_headline() {
    // The report struct is the binary's print surface — pin its field
    // semantics (path + sorted isolated + counts) with a literal.
    let report = ProfileReport {
        artifact_path: default_artifact_path(Path::new("/ws"), "demo"),
        backup_path: None,
        isolated: vec![IsolatedNodeReport {
            node: "laggard".to_string(),
            fires: 7,
            target: Some(1000),
            starved_trigger_inputs: Vec::new(),
        }],
        fires_override: Some(1000),
        node_count: 3,
        window_ns: 2_000_000_000,
    };
    assert_eq!(
        report.artifact_path,
        Path::new("/ws/graphs/demo.costs.yaml")
    );
    assert_eq!(
        report.isolated,
        vec![IsolatedNodeReport {
            node: "laggard".to_string(),
            fires: 7,
            target: Some(1000),
            starved_trigger_inputs: Vec::new(),
        }],
        "the isolated entry carries the node, its observed fire count, AND its own target"
    );
    assert_eq!(
        report.fires_override,
        Some(1000),
        "Some = the uniform --fires mode"
    );
    assert_eq!(report.node_count, 3);
    assert_eq!(report.window_ns, 2_000_000_000);
    assert!(report.backup_path.is_none(), "fresh write => no backup");
}

// ==========================================================================
// The AUTO-mode pure helpers — the per-node stop gate
// (`all_targets_met`) and the warm-up clamp (`profile_warmup_duration`).
// Oracle-vector style: every expectation is a hand-computed literal.
// ==========================================================================

fn counts(pairs: &[(&str, u64)]) -> BTreeMap<String, u64> {
    pairs.iter().map(|(id, c)| (id.to_string(), *c)).collect()
}

#[test]
fn all_targets_met_true_when_every_targeted_node_reaches_its_own_target() {
    let fire_counts = counts(&[("a", 50), ("b", 20)]);
    let targets = counts(&[("a", 50), ("b", 20)]);
    assert!(
        all_targets_met(&fire_counts, &targets),
        "count == target is MET (the >= boundary mirrors harvest_costs' keep-at-target gate)"
    );
    let over = counts(&[("a", 51), ("b", 900)]);
    assert!(all_targets_met(&over, &targets), "over-target is met too");
}

#[test]
fn all_targets_met_false_when_one_node_is_short() {
    let fire_counts = counts(&[("a", 50), ("b", 19)]);
    let targets = counts(&[("a", 50), ("b", 20)]);
    assert!(
        !all_targets_met(&fire_counts, &targets),
        "b at target-1 holds the gate open (per-node, not aggregate)"
    );
}

#[test]
fn all_targets_met_excludes_silent_nodes_without_a_target_entry() {
    // `silent` fired 0 and has NO target entry (silent through warm-up) — it
    // must NOT hold the run hostage: the gate closes on the targeted nodes.
    let fire_counts = counts(&[("a", 50), ("silent", 0)]);
    let targets = counts(&[("a", 50)]);
    assert!(
        all_targets_met(&fire_counts, &targets),
        "a node absent from the target map is EXCLUDED from the stop gate"
    );
}

#[test]
fn all_targets_met_empty_targets_is_false_until_warmup() {
    // THE pinned semantics: before derivation exists (or when every node was
    // silent through warm-up ⇒ an EMPTY derived map) the gate must NOT stop
    // the run — vacuous-true would stop it instantly at warm-up end,
    // forfeiting exactly the remaining cap a slow node needs.
    let fire_counts = counts(&[("a", 1_000_000), ("b", 1_000_000)]);
    let targets: BTreeMap<String, u64> = BTreeMap::new();
    assert!(
        !all_targets_met(&fire_counts, &targets),
        "empty targets => FALSE (never vacuous-true), regardless of the counts"
    );
}

#[test]
fn all_targets_met_targeted_node_missing_from_counts_is_not_met() {
    // A target-bearing node absent from the counts map counts as 0 fires.
    let fire_counts = counts(&[("a", 50)]);
    let targets = counts(&[("a", 50), ("ghost", 1)]);
    assert!(
        !all_targets_met(&fire_counts, &targets),
        "a targeted node with no recorded count is treated as 0 => not met"
    );
}

#[test]
fn profile_warmup_duration_clamp_oracle() {
    // cap/10 clamped into [1 s, 3 s] — hand-computed vectors.
    let d = |s| Duration::from_secs(s);
    assert_eq!(
        profile_warmup_duration(d(30)),
        d(3),
        "30s cap => 3s (exact tenth)"
    );
    assert_eq!(
        profile_warmup_duration(d(5)),
        d(1),
        "5s cap => 0.5s tenth clamps UP to the 1s floor"
    );
    assert_eq!(
        profile_warmup_duration(d(20)),
        d(2),
        "20s cap => 2s (exact tenth)"
    );
    assert_eq!(
        profile_warmup_duration(d(60)),
        d(3),
        "60s cap => 6s tenth clamps DOWN to the 3s ceiling"
    );
    assert_eq!(
        profile_warmup_duration(d(1)),
        d(1),
        "1s cap (the CLI minimum) => warm-up == the whole cap"
    );
}

// ==========================================================================
// The artifact composes with auto_partition end-to-end (pure): thawed costs +
// isolated feed the partitioner; the isolated node stays singleton.
// ==========================================================================

#[test]
fn thawed_artifact_feeds_auto_partition() {
    use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
    use cerulion_core::graph::{auto_partition, NodeInfo, TriggerEdges};
    use indexmap::IndexMap;

    // ticker -> sink (trigger edge), laggard disconnected + isolated.
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        name: None,
        identity: "demo".to_string(),
        prefix: "p".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "ticker".to_string(),
                node_type: "ticker".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "cmd".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "sink".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![InputDef {
                    name: "trigger_in".to_string(),
                    source: "ticker/cmd".to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                ros2: None,
                id: "laggard".to_string(),
                node_type: "laggard".to_string(),
                inputs: vec![],
                outputs: vec![],
            },
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: IndexMap::new(),
        process_group_order: Vec::new(),
    };
    let infos: IndexMap<String, NodeInfo> = config
        .nodes
        .iter()
        .map(|n| (n.id.clone(), NodeInfo::with_meta(Vec::new(), Vec::new())))
        .collect();
    let mut t = TriggerEdges::new();
    t.insert("sink", "/p/ticker/cmd");

    let artifact =
        ProfileArtifact::from_profile("demo", 5_000_000_000, &sample_profile(), cores(4));
    let (costs, isolated) = artifact.to_costs().expect("to_costs");
    let part = auto_partition(&config, &infos, &t, &costs, &isolated, 100_000)
        .expect("the thawed artifact must feed auto_partition without error");

    let shape: Vec<(String, Vec<String>)> = part
        .groups
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();
    assert_eq!(
        shape,
        vec![
            (
                "grp_ticker".to_string(),
                vec!["ticker".to_string(), "sink".to_string()]
            ),
            ("grp_laggard".to_string(), vec!["laggard".to_string()]),
        ],
        "the rated edge fuses; the isolated (cost-less) laggard stays singleton"
    );
}

// ==========================================================================
// An `edges:` entry naming an `isolated:` node is a
// LOUD to_costs reject (the fourth arm) — otherwise the rate silently
// degrades to a downstream stray-key warn at partition time.
// ==========================================================================

#[test]
fn edge_naming_isolated_node_is_rejected() {
    // The consumer endpoint is isolated => reject, naming edge + endpoint.
    let artifact = ProfileArtifact {
        version: 1,
        graph: "demo".to_string(),
        window_ns: 1,
        nodes: [("a".to_string(), 100)].into_iter().collect(),
        edges: vec![ProfileEdge {
            producer: "a".to_string(),
            consumer: "slow".to_string(),
            rate_mhz: 10,
        }],
        isolated: vec!["slow".to_string()],
        hop: None,
        derived_budget_ns: None,
        profile_cores: None,
    };
    let err = artifact
        .to_costs()
        .expect_err("an edge into an isolated node must be rejected")
        .to_string();
    assert!(
        err.contains("slow") && err.contains("isolated"),
        "the error must name the isolated endpoint; got: {err}"
    );

    // Producer-side twin: the isolated node produces the edge.
    let artifact = ProfileArtifact {
        version: 1,
        graph: "demo".to_string(),
        window_ns: 1,
        nodes: [("b".to_string(), 100)].into_iter().collect(),
        edges: vec![ProfileEdge {
            producer: "slow".to_string(),
            consumer: "b".to_string(),
            rate_mhz: 10,
        }],
        isolated: vec!["slow".to_string()],
        hop: None,
        derived_budget_ns: None,
        profile_cores: None,
    };
    assert!(
        artifact.to_costs().is_err(),
        "an edge FROM an isolated node must also be rejected"
    );
}

// ==========================================================================
// The atomic + backed-up artifact writer. The artifact
// is a documented hand-edit surface — a mid-write failure must never corrupt
// the prior file, and an overwrite must never silently clobber hand edits.
// ==========================================================================

/// Directory must hold EXACTLY the expected file names afterward — pins that
/// no temp file leaks (the atomic-rename shape).
fn dir_file_names(dir: &Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .expect("read_dir")
        .map(|e| e.expect("entry").file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    names
}

#[test]
fn atomic_write_fresh_no_backup_no_stray_tmp() {
    use cerulion_cli_engine::graph_cmd::write_profile_artifact_atomically;
    let tmp = tempfile::tempdir().expect("tempdir");
    let dest = tmp.path().join("demo.costs.yaml");

    let backup = write_profile_artifact_atomically(&dest, "version: 1\n").expect("write");
    assert!(backup.is_none(), "fresh write => no backup");
    assert_eq!(
        std::fs::read_to_string(&dest).expect("dest readable"),
        "version: 1\n",
        "destination carries the new contents"
    );
    assert_eq!(
        dir_file_names(tmp.path()),
        vec!["demo.costs.yaml".to_string()],
        "no .bak and no leaked temp file"
    );
}

#[tracing_test::traced_test]
#[test]
fn atomic_write_existing_backs_up_old_bytes_and_warns() {
    use cerulion_cli_engine::graph_cmd::write_profile_artifact_atomically;
    let tmp = tempfile::tempdir().expect("tempdir");
    let dest = tmp.path().join("demo.costs.yaml");

    // The "hand-edited" prior artifact.
    let old = "version: 1\n# hand edit: precious\n";
    std::fs::write(&dest, old).expect("seed prior artifact");

    let new = "version: 1\nnodes: {}\n";
    let backup = write_profile_artifact_atomically(&dest, new).expect("write");

    let bak = backup.expect("an existing destination must yield a backup path");
    assert_eq!(
        bak,
        tmp.path().join("demo.costs.yaml.bak"),
        "backup is <path>.bak next to the artifact"
    );
    assert_eq!(
        std::fs::read_to_string(&bak).expect("backup readable"),
        old,
        "the backup carries the OLD content byte-identical (hand edits preserved)"
    );
    assert_eq!(
        std::fs::read_to_string(&dest).expect("dest readable"),
        new,
        "the destination carries the NEW content"
    );
    assert!(
        logs_contain("previous file (including any hand edits) was backed up"),
        "the overwrite must warn loudly"
    );
    assert_eq!(
        dir_file_names(tmp.path()),
        vec![
            "demo.costs.yaml".to_string(),
            "demo.costs.yaml.bak".to_string()
        ],
        "exactly artifact + backup — no leaked temp file"
    );
}

#[test]
fn atomic_write_creates_missing_parent_dirs() {
    use cerulion_cli_engine::graph_cmd::write_profile_artifact_atomically;
    let tmp = tempfile::tempdir().expect("tempdir");
    let dest = tmp.path().join("deep/nested/demo.costs.yaml");
    let backup = write_profile_artifact_atomically(&dest, "version: 1\n").expect("write");
    assert!(backup.is_none());
    assert_eq!(
        std::fs::read_to_string(&dest).expect("dest readable"),
        "version: 1\n"
    );
}

#[tracing_test::traced_test]
#[test]
fn atomic_write_second_overwrite_replaces_backup_with_latest_prior() {
    use cerulion_cli_engine::graph_cmd::write_profile_artifact_atomically;
    let tmp = tempfile::tempdir().expect("tempdir");
    let dest = tmp.path().join("demo.costs.yaml");

    write_profile_artifact_atomically(&dest, "gen: one\n").expect("first");
    write_profile_artifact_atomically(&dest, "gen: two\n").expect("second");
    let backup = write_profile_artifact_atomically(&dest, "gen: three\n").expect("third");

    // The backup always holds the IMMEDIATELY-PRIOR generation (one level of
    // undo — documented as `<path>.bak`, not a history).
    assert_eq!(
        std::fs::read_to_string(backup.expect("backup")).expect("bak readable"),
        "gen: two\n"
    );
    assert_eq!(
        std::fs::read_to_string(&dest).expect("dest readable"),
        "gen: three\n"
    );
}

/// The ENGINE rejects `fires_override == Some(0)`
/// loudly BEFORE touching the workspace or transport — defense-in-depth behind
/// the CLI's `range(1..)` parse gate (the CLI-and-engine-must-enforce-the-same-
/// invariant discipline). Pins the remedy text naming auto-derive.
#[test]
fn engine_rejects_some_zero_fires_override_before_workspace_access() {
    use std::sync::atomic::AtomicBool;
    use std::sync::Arc;
    let err = cerulion_cli_engine::graph_cmd::graph_profile(
        Path::new("/nonexistent/workspace/for/this/pin"),
        "nope",
        Arc::new(AtomicBool::new(true)),
        Duration::from_secs(1),
        Some(0),
        None,
    )
    .expect_err("Some(0) must be rejected");
    let msg = err.to_string();
    assert!(
        msg.contains("--fires must be > 0"),
        "wrong rejection message: {msg}"
    );
    assert!(
        msg.contains("omit --fires to auto-derive"),
        "remedy must name auto-derive: {msg}"
    );
}

// ==========================================================================
// The WATCHER's anchoring arithmetic:
// `build_warmup_observations` turns per-node first-fire sightings into the
// per-node windows the fire targets are projected from. PURE (no `Instant`,
// no run), hand-written oracles throughout.
//
// The engine half — how those observations project into targets, including
// the floor a fired-but-sampleless node lands on — is oracle-pinned in
// `cerulion_core/tests/auto_partition_test.rs`.
// ==========================================================================

/// A sighting builder: `(node, first_fire_at_ns, fires_at_first_sighting,
/// fires_at_warmup_end)`.
fn sighting(
    node: &str,
    first_fire_at_ns: Option<u64>,
    fires_at_first_sighting: u64,
    fires_at_warmup_end: u64,
) -> NodeWarmupSighting {
    NodeWarmupSighting {
        node: node.to_string(),
        first_fire_at_ns,
        fires_at_first_sighting,
        fires_at_warmup_end,
    }
}

#[test]
fn a_late_first_fire_is_measured_over_its_own_window_not_the_elapsed_wall() {
    // The decision-85 pin. Three nodes, ONE 1 s warm-up, and each must be
    // measured over ITS OWN active window:
    //   early : first fire at   0 ms, 21 fires by the end ->  20 over 1000 ms
    //   late  : first fire at 900 ms,  3 fires by the end ->   2 over  100 ms
    //   burst : first fire at   0 ms, 59 fires by the end (39 already standing
    //           at the sighting — the bring-up debt) ->        20 over 1000 ms
    //
    // `late` is the discriminator: over the elapsed WALL it reads 2 fires per
    // second, over its own window 20 — an order of magnitude apart, and only
    // the second is a rate the node ever ran at. `burst` is the bring-up burst shape:
    // identical raw count to a 59 Hz node, identical answer to `early` once the
    // debt it arrived with is excluded.
    const WARMUP_END_NS: u64 = 1_000_000_000;
    let observations = build_warmup_observations(
        &[
            sighting("early", Some(0), 1, 21),
            sighting("late", Some(900_000_000), 1, 3),
            sighting("burst", Some(0), 39, 59),
        ],
        WARMUP_END_NS,
    );
    let oracle: BTreeMap<String, WarmupObservation> = [
        (
            "early".to_string(),
            WarmupObservation {
                fires: 20,
                window_ns: 1_000_000_000,
            },
        ),
        (
            "late".to_string(),
            WarmupObservation {
                fires: 2,
                window_ns: 100_000_000,
            },
        ),
        (
            "burst".to_string(),
            WarmupObservation {
                fires: 20,
                window_ns: 1_000_000_000,
            },
        ),
    ]
    .into_iter()
    .collect();
    assert_eq!(
        observations, oracle,
        "each node's window runs from ITS OWN first fire, with the fires already \
         standing there excluded"
    );
}

#[test]
fn a_node_that_never_fired_yields_no_observation_at_all() {
    // The SILENT contract, unmoved: no first fire ⇒ no observation ⇒
    // no target ⇒ the distinct "silent through warm-up" isolation marker. The
    // live sibling in the same call is the anti-tautology half (an empty result
    // would satisfy the absence assertion on its own).
    let observations = build_warmup_observations(
        &[
            sighting("silent", None, 0, 0),
            sighting("live", Some(10_000_000), 1, 21),
        ],
        1_000_000_000,
    );
    assert!(
        !observations.contains_key("silent"),
        "a node that never fired must carry NO observation; got {observations:?}"
    );
    assert_eq!(
        observations["live"],
        WarmupObservation {
            fires: 20,
            window_ns: 990_000_000,
        },
        "the firing sibling is still observed over its own window"
    );
}

#[test]
fn a_node_whose_first_fire_lands_in_the_boundary_poll_is_observed_not_silenced() {
    // Driven through the WATCHER SEAM, not through `build_warmup_observations`
    // directly, because the property is about the POLL's own order: the anchor
    // sweep must run BEFORE the warm-up-end snapshot, so a node whose counter
    // moves for the first time in the boundary poll is anchored AT the boundary
    // (zero-length window ⇒ floored by the policy) instead of falling through
    // into the "never fired" contract and being isolated as silent through
    // warm-up. Handing a pre-populated sighting to the builder cannot see that —
    // it passes whichever order the watcher uses.
    //
    // `counts` is ONE reading per poll, shared by the sweep and the snapshot,
    // which is also what makes "warm-up fires with no anchor" unreachable.
    let mut sightings = vec![
        sighting("boundary", None, 0, 0),
        sighting("early", None, 0, 0),
    ];

    // Poll 1 at 500 ms — mid-warm-up: `early` has fired 10 times (a bring-up
    // burst plus its own cadence), `boundary` has not moved at all.
    assert!(
        poll_warmup_sightings(&mut sightings, &[0, 10], 500_000_000, false).is_none(),
        "a poll before the boundary produces no observations"
    );

    // Poll 2 IS the warm-up end at 1000 ms — `boundary`'s counter has moved for
    // the FIRST time (4 fires, all of them its bring-up debt).
    let observations = poll_warmup_sightings(&mut sightings, &[4, 20], 1_000_000_000, true)
        .expect("the boundary poll returns the warm-up observations");

    let oracle: BTreeMap<String, WarmupObservation> = [
        (
            // Anchored AT the boundary: zero active window, zero active fires.
            // PRESENT is the whole assertion — absent would mean isolated.
            "boundary".to_string(),
            WarmupObservation {
                fires: 0,
                window_ns: 0,
            },
        ),
        (
            // Anchored at poll 1: 20 − 10 fires over 1000 − 500 ms.
            "early".to_string(),
            WarmupObservation {
                fires: 10,
                window_ns: 500_000_000,
            },
        ),
    ]
    .into_iter()
    .collect();
    assert_eq!(
        observations, oracle,
        "the boundary node is observed with a zero-length window, not dropped"
    );
    assert!(
        observations.contains_key("boundary"),
        "a node that fired in the boundary poll must never be reported silent"
    );
}

#[test]
fn a_counter_that_never_moves_is_never_anchored_by_the_sweep() {
    // The silent contract from the SEAM's side (its builder-side twin is
    // `a_node_that_never_fired_yields_no_observation_at_all`): the sweep anchors
    // on a counter that MOVED, so a node reading zero at every poll — including
    // the boundary — carries no anchor and therefore no observation, which is
    // what `harvest_costs` reads as "silent through warm-up". The live sibling
    // in the same body is the anti-tautology half: an empty result would satisfy
    // the absence assertion on its own.
    let mut sightings = vec![sighting("silent", None, 0, 0), sighting("live", None, 0, 0)];
    for (poll, live) in [(200_000_000u64, 0u64), (600_000_000, 3)] {
        assert!(poll_warmup_sightings(&mut sightings, &[0, live], poll, false).is_none());
    }
    let observations = poll_warmup_sightings(&mut sightings, &[0, 12], 1_000_000_000, true)
        .expect("the boundary poll returns the warm-up observations");

    assert!(
        !observations.contains_key("silent"),
        "a counter that never moved must carry NO observation; got {observations:?}"
    );
    assert_eq!(
        observations["live"],
        // Anchored at 600 ms carrying 3; 12 − 3 fires over 1000 − 600 ms.
        WarmupObservation {
            fires: 9,
            window_ns: 400_000_000,
        },
        "the firing sibling is observed over its own window"
    );
}

#[test]
fn the_excluded_burst_is_accounted_for_every_node_that_fired() {
    // `burst_fires_excluded` is the operator's window onto what the anchoring
    // removed (Principle #3). It sums the fires ALREADY STANDING at each
    // anchored node's first sighting, and a never-fired node contributes
    // nothing — so the total is structurally >= 1 per anchored node.
    let sightings = [
        sighting("ticker", Some(0), 39, 59),
        sighting("sink", Some(1_000_000), 2, 20),
        sighting("silent", None, 0, 0),
    ];
    assert_eq!(
        warmup_burst_fires(&sightings),
        41,
        "39 + 2, with the never-fired node contributing nothing"
    );
    assert_eq!(
        warmup_burst_fires(&[sighting("silent", None, 0, 0)]),
        0,
        "a run in which nothing fired excluded nothing"
    );
}

// ==========================================================================
// The watcher reads each counter ONCE
// per poll. STRUCTURAL, because the defect it closes is a RACE — a fire
// landing between two reads leaves a node with warm-up fires and no anchor,
// and no test can schedule that interleaving on demand. MEASURED: restoring
// the two-read shape at the call site passes the entire pure suite.
//
// One reading makes `count > 0` and `fires_at_warmup_end > 0` the same
// predicate, so the bad state is unreachable rather than repaired. That is a
// property of the CALL SITE, which `poll_warmup_sightings` (which takes the
// reading as a parameter) structurally cannot assert about itself.
// ==========================================================================

/// The source with comments removed — line AND block, depth-tracked because
/// Rust block comments NEST. Load-bearing here: the watcher's own comments
/// explain the single-read rule and name the expression it governs, so a raw
/// text count would read them as code.
fn code_only(src: &str) -> String {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let (mut i, mut depth, mut line_comment) = (0usize, 0usize, false);
    while i < bytes.len() {
        // Compared as BYTES, never as a `&src[i..i + 2]` slice: this source is
        // full of em-dashes, and slicing two bytes at a multi-byte boundary
        // panics (it did, the first time this ran).
        let (b0, b1) = (bytes[i], bytes.get(i + 1).copied().unwrap_or(0));
        if depth == 0 && !line_comment && b0 == b'/' && b1 == b'/' {
            line_comment = true;
            i += 2;
        } else if line_comment {
            if b0 == b'\n' {
                line_comment = false;
                out.push('\n');
            }
            i += 1;
        } else if b0 == b'/' && b1 == b'*' {
            depth += 1;
            i += 2;
        } else if depth > 0 && b0 == b'*' && b1 == b'/' {
            depth -= 1;
            i += 2;
        } else if depth > 0 {
            if bytes[i] == b'\n' {
                out.push('\n');
            }
            i += 1;
        } else {
            let ch = src[i..].chars().next().expect("char boundary");
            out.push(ch);
            i += ch.len_utf8();
        }
    }
    out
}

/// The body of the watcher closure, brace-matched from its `spawn` header — a
/// textual `};` boundary is the class this repo has already been bitten by.
fn watcher_closure_body(code: &str) -> &str {
    const HEADER: &str = ".spawn(move || -> Option<ProfileWarmupSnapshot> {";
    let start = code
        .find(HEADER)
        .expect("the profile watcher's spawn header must be findable")
        + HEADER.len();
    let mut depth = 1usize;
    for (offset, ch) in code[start..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return &code[start..start + offset];
                }
            }
            _ => {}
        }
    }
    panic!("the watcher closure body is not brace-balanced");
}

/// Every `.load(` RECEIVER in `body`, in source order — the expression text
/// immediately preceding each call.
///
/// The guard counts fire-counter loads, and it must not be keyed on ONE
/// SPELLING: `counter.load(` is what the watcher happens to write today, so a
/// second read through any other binding (`c.load(`) or through an expression
/// (`handle.fire_counter().load(`) would sail past a lexical needle while being
/// exactly the race the guard exists to close.
///
/// Walks BACKWARDS over the bytes a receiver expression can be made of. Every
/// accepted byte is ASCII, so the resulting index is always a char boundary —
/// this source is full of em-dashes and the walk must never slice into one.
fn atomic_load_receivers(body: &str) -> Vec<String> {
    const NEEDLE: &str = ".load(";
    let bytes = body.as_bytes();
    body.match_indices(NEEDLE)
        .map(|(call, _)| {
            let mut start = call;
            while start > 0 {
                let b = bytes[start - 1];
                let part_of_receiver = b.is_ascii_alphanumeric()
                    || matches!(b, b'_' | b'.' | b'(' | b')' | b'[' | b']');
                if !part_of_receiver {
                    break;
                }
                start -= 1;
            }
            body[start..call].to_string()
        })
        .collect()
}

/// The atomics in the watcher body that are NOT fire counters, each
/// with the reason it is not one.
///
/// A DECLARED inventory rather than a filter: anything else the walk finds is
/// counted as a fire-counter load, so a new atomic added to the poll fails the
/// guard until somebody classifies it — the same discipline as this repo's
/// other structural walks (`upstream_drift_test`'s waivers,
/// `completions_test`'s forbidden-token list).
const NON_COUNTER_ATOMIC_LOADS: &[(&str, &str)] = &[(
    "watcher_running",
    "the shared `running` flag — the Ctrl-C / external-stop seam, read once at \
     the top of the poll to decide whether to keep going. Not a per-node fire \
     counter, and re-reading it cannot desynchronise a node's warm-up \
     bookkeeping from its own fire count.",
)];

#[test]
fn the_watcher_reads_each_fire_counter_once_per_poll() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/graph_cmd.rs"))
        .expect("the engine source is readable from its own test");
    let code = code_only(&src);
    let body = watcher_closure_body(&code);

    // ANTI-TAUTOLOGY: the extraction really found the loop, so a mis-resolved
    // boundary cannot make the count vacuously right.
    for marker in [
        "poll_warmup_sightings(",
        "all_targets_met(",
        "warmup_deadline",
    ] {
        assert!(
            body.contains(marker),
            "the extracted watcher body must contain `{marker}` — the boundary resolved wrong"
        );
    }

    let receivers = atomic_load_receivers(body);

    // A stale exclusion pre-authorises an atomic the poll no longer has, which
    // is how an inventory rots into a licence. Every declared entry must be
    // REACHED.
    for (receiver, reason) in NON_COUNTER_ATOMIC_LOADS {
        assert!(
            receivers.iter().any(|r| r.as_str() == *receiver),
            "`{receiver}` is declared a non-counter atomic load ({reason}) but the watcher \
             body no longer loads it — drop the entry rather than leaving it to \
             pre-authorise something else"
        );
    }

    let counter_loads: Vec<&String> = receivers
        .iter()
        .filter(|r| {
            !NON_COUNTER_ATOMIC_LOADS
                .iter()
                .any(|(excluded, _)| *excluded == r.as_str())
        })
        .collect();

    assert_eq!(
        counter_loads.len(),
        1,
        "the watcher must read each fire counter exactly ONCE per poll and share that \
         reading with the warm-up bookkeeping AND the stop gate: a second load \
         lets a fire land between the two, leaving a node with warm-up fires and no \
         first-fire anchor, which is then reported 'silent through warm-up' and isolated. \
         Every `.load(` in the watcher body whose receiver is not in \
         NON_COUNTER_ATOMIC_LOADS counts, so a second read through ANY binding or \
         expression is caught, not just the spelling used today. Found these \
         counter-load receivers: {counter_loads:?} (all load receivers: {receivers:?})."
    );
}

#[test]
fn the_structural_helpers_answer_their_hand_written_vectors() {
    // The walk asserts THROUGH these, so a helper that answers too easily makes
    // it vacuous without failing anything.
    assert_eq!(code_only("a // b\nc"), "a \nc", "line comments are removed");
    assert_eq!(
        code_only("a /* b */ c"),
        "a  c",
        "block comments are removed"
    );
    assert_eq!(
        code_only("a /* b /* c */ d */ e"),
        "a  e",
        "block comments NEST"
    );
    assert_eq!(
        code_only("a // /* b\nc"),
        "a \nc",
        "a block opener inside a line comment opens nothing"
    );
    assert_eq!(
        code_only("a — b // c\nd"),
        "a — b \nd",
        "multi-byte characters survive (the first run panicked slicing one)"
    );
    // `atomic_load_receivers` is what makes the guard spelling-independent, so
    // it is pinned on the shapes a second read could actually take — the ones a
    // `counter.load(` needle would miss.
    assert_eq!(
        atomic_load_receivers("let a = counter.load(Ordering::Acquire);"),
        vec!["counter".to_string()],
        "a plain binding is its own receiver"
    );
    assert_eq!(
        atomic_load_receivers("let a = c.load(o); let b = fire_counter.load(o);"),
        vec!["c".to_string(), "fire_counter".to_string()],
        "a DIFFERENT binding is still a receiver — the dodge the reviewer named"
    );
    assert_eq!(
        atomic_load_receivers("handle.fire_counter().load(Ordering::Acquire)"),
        vec!["handle.fire_counter()".to_string()],
        "an EXPRESSION receiver is captured whole — the other half of the dodge"
    );
    assert_eq!(
        atomic_load_receivers("counters[idx].2.load(o)"),
        vec!["counters[idx].2".to_string()],
        "indexing and tuple access belong to the receiver"
    );
    assert_eq!(
        atomic_load_receivers("map(|(_, counter, _)| counter.load(o))"),
        vec!["counter".to_string()],
        "a closure pattern before the receiver is not swallowed"
    );
    assert!(
        atomic_load_receivers("let x = 1; // nothing here\n").is_empty(),
        "no call, no receivers"
    );

    assert_eq!(
        watcher_closure_body("x.spawn(move || -> Option<ProfileWarmupSnapshot> {a{b}c}y"),
        "a{b}c",
        "the body is brace-MATCHED, not cut at the first close"
    );
}
