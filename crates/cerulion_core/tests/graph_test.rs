// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for graph loading, validation, and runtime under the
//! SHM-backed pub/sub API.
//!
//! Tests the full pipeline: YAML parsing → validation → GraphRuntime build → step.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test graph_test -- --test-threads=1
//! ```
//!
//! Must run single-threaded when the `build` (iceoryx2) variants are exercised:
//! the iceoryx2 singleton + shared memory require serial access across
//! transport tests. The `build_for_test` variants are technically parallel-
//! safe but the shared `--test-threads=1` invocation still applies.
//!
//! # Drops vs. legacy file
//!
//! Tests that exercised the deleted heap-struct `publisher.publish(&msg)` /
//! `subscriber.try_receive_typed` path inside the runtime were rewritten to
//! use `loan_proxy::<T>()`. Tests that purely verified parse/validate/scheduler
//! plumbing translated unchanged.

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::graph::node::{ClosureNodeEntry, NodeEntry, NodeInfo};
use cerulion_core::graph::{
    default_prefix, parse_graph, resolve_source, validate_graph, validate_graph_with, GraphRuntime,
    ValidationOptions,
};
use cerulion_core::transport::TransportManager;
use cerulion_core::MacroPolicy;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Monotonic counter guaranteeing unique topics even within the same nanosecond.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Generate a unique graph prefix for test isolation.
fn unique_prefix(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("gtest/{}/{}/{}", base, nanos, id)
}

// ============================================================
// Pure-config tests (no transport): translate unchanged.
// ============================================================

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
    // Missing `prefix:` resolves to hostname (sans `.local`) with the
    // graph name as a fallback when `hostname` is unavailable.
    assert_eq!(config.prefix, default_prefix("test"));
    assert!(!config.prefix.is_empty());
    assert_eq!(config.nodes.len(), 1);
    assert_eq!(config.nodes[0].id, "pub1");
    assert_eq!(config.nodes[0].node_type, "test_pub");
    assert_eq!(config.nodes[0].outputs.len(), 1);
    assert_eq!(config.nodes[0].outputs[0].name, "data");
    assert_eq!(config.nodes[0].outputs[0].max_slice_len, Some(1024));
}

#[test]
fn test_resolve_source_from_node_binding() {
    assert_eq!(
        resolve_source("perception", "camera/image"),
        "/perception/camera/image"
    );
}

#[test]
fn test_validate_graph_missing_source_node() {
    let yaml = r#"
name: test
nodes:
  - id: sub1
    type: consumer
    inputs:
      - name: data
        source: ghost/output
"#;
    let config = parse_graph(yaml).unwrap();
    let result = validate_graph(&config);
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("non-existent"),
        "error should mention non-existent source: {}",
        err
    );
}

#[test]
fn test_validate_graph_absolute_external_source_allowed() {
    // An ABSOLUTE source (leading '/') matching no
    // in-graph output is an EXTERNAL topic — exempt from the
    // non-existent-source rejection (GraphTopology::validate still
    // rejects `block` consumers on it; the runtime provisions it
    // External). A relative dangling source stays a hard error
    // (test_validate_graph_missing_source_node above).
    let yaml = r#"
name: test
nodes:
  - id: sub1
    type: consumer
    inputs:
      - name: data
        source: /vendor/cam
"#;
    let config = parse_graph(yaml).unwrap();
    validate_graph(&config).expect("absolute external source must validate");
}

#[test]
#[tracing_test::traced_test]
fn test_in_prefix_absolute_miss_warns_and_validates_as_external() {
    // This first shipped as a rejection; flipped
    // to a loud warn: two same-host graphs legitimately share
    // the hostname-default prefix — prefix is a namespace, not a graph
    // identity — so an in-prefix absolute miss may be another graph's
    // topic. The graph must VALIDATE (the topic provisions External) and
    // the typo diagnosis must arrive as exactly one warn. The
    // data-integrity half (two graphs actually publishing the same
    // topic) errors at publisher creation — pinned end-to-end in
    // cross_graph_collision_iox2_test.rs.
    let yaml = r#"
name: test
prefix: p
nodes:
  - id: camera
    type: camera
    outputs:
      - name: image
        schema: test/Data
        max_slice_len: 1024
  - id: viewer
    type: viewer
    inputs:
      - name: img
        source: /p/camera/imge
"#;
    let config = parse_graph(yaml).unwrap();
    validate_graph(&config).expect("in-prefix absolute miss must validate as external");
    // The HIT case: the absolute spelling of an existing derived output
    // wires in-graph, validates fine, and must NOT warn (it never reaches
    // the in-prefix branch — the output-set check short-circuits first).
    let yaml_ok = yaml.replace("/p/camera/imge", "/p/camera/image");
    let config = parse_graph(&yaml_ok).unwrap();
    validate_graph(&config).expect("absolute spelling of a real in-graph output validates");
    // The OUT-OF-PREFIX miss: a genuinely external topic EVALUATES the
    // in_prefix predicate (false → the silent external `continue`) and
    // must NOT warn — this arm catches a variant that always warns
    // (`if in_prefix` → `if true`), which the HIT case alone cannot (the HIT
    // short-circuits at the output-set check before the predicate).
    let yaml_ext = yaml.replace("/p/camera/imge", "/vendor/cam");
    let config = parse_graph(&yaml_ext).unwrap();
    validate_graph(&config).expect("out-of-prefix external source validates silently");
    logs_assert(|lines: &[&str]| {
        // Both substrings: keyed to THIS warn specifically, so a future
        // warn elsewhere borrowing "matches no declared output" cannot
        // flip the count for an unrelated reason.
        let warns = lines
            .iter()
            .filter(|l| {
                l.contains("matches no declared output")
                    && l.contains("treating it as an EXTERNAL topic")
            })
            .count();
        if warns == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 in-prefix-miss warn, got {warns}"
            ))
        }
    });
}

/// A multi-process WORKER's view of a correct graph: only `viewer`, its
/// cross-group input already rewritten to the absolute topic, and `camera`
/// (the producer) in a sibling group's process. The supervisor names that
/// topic, so validating the view must not suggest the wiring is a typo.
const WORKER_VIEW_YAML: &str = r#"
name: test
prefix: p
nodes:
  - id: viewer
    type: viewer
    inputs:
      - name: img
        source: /p/camera/image
"#;

fn in_prefix_miss_warns(lines: &[&str]) -> Vec<String> {
    lines
        .iter()
        .filter(|l| {
            l.contains("matches no declared output")
                && l.contains("treating it as an EXTERNAL topic")
        })
        .map(|l| l.to_string())
        .collect()
}

#[test]
#[tracing_test::traced_test]
fn a_worker_view_whose_source_a_sibling_group_produces_does_not_warn() {
    let config = parse_graph(WORKER_VIEW_YAML).unwrap();
    let siblings = std::collections::BTreeSet::from(["/p/camera/image".to_string()]);
    validate_graph_with(
        &config,
        ValidationOptions {
            sibling_topics: Some(&siblings),
            ..Default::default()
        },
    )
    .expect("a worker view validates");
    logs_assert(
        |lines: &[&str]| match in_prefix_miss_warns(lines).as_slice() {
            [] => Ok(()),
            warns => Err(format!(
                "a sibling-produced source must not warn: {warns:?}"
            )),
        },
    );
}

#[test]
#[tracing_test::traced_test]
fn a_worker_view_still_warns_for_a_source_no_sibling_produces() {
    // Anti-tautology for the test above, in the SAME view with the SAME set:
    // a second input one letter off a sibling topic is in no set, so it warns,
    // once, naming itself. A check that silenced every in-prefix source in a
    // worker would pass the test above and fail here.
    let yaml = format!("{WORKER_VIEW_YAML}      - name: typo\n        source: /p/camera/imge\n");
    let config = parse_graph(&yaml).unwrap();
    let siblings = std::collections::BTreeSet::from(["/p/camera/image".to_string()]);
    validate_graph_with(
        &config,
        ValidationOptions {
            sibling_topics: Some(&siblings),
            ..Default::default()
        },
    )
    .expect("an in-prefix miss validates as external");
    // The same view with NO set is the pre-split behaviour: both sources warn.
    validate_graph(&config).expect("validates without the set too");
    logs_assert(|lines: &[&str]| {
        let warns = in_prefix_miss_warns(lines);
        let typo = warns
            .iter()
            .filter(|l| l.contains("/p/camera/imge"))
            .count();
        let sibling = warns
            .iter()
            .filter(|l| l.contains("/p/camera/image"))
            .count();
        // typo: once per validation. sibling: only the validation with no set.
        if typo == 2 && sibling == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected typo=2 sibling=1, got typo={typo} sibling={sibling}: {warns:?}"
            ))
        }
    });
}

#[test]
#[tracing_test::traced_test]
fn test_in_prefix_multi_segment_prefix_miss_still_warns() {
    // Regression: a graph prefix
    // may itself contain `/` — the prefix validator permits internal single
    // slashes, so `my/robot` is a VALID prefix. The in-prefix detection must
    // match the full `/{prefix}/` boundary, not just the first `/`-segment:
    // the old `source[1..].split('/').next() == prefix` compare yielded
    // `"my" == "my/robot"` → false, silently misclassifying an in-prefix
    // typo as an external topic and SUPPRESSING the diagnostic warn. This
    // drives exactly that case (a typo'd `/my/robot/camera/imge`) and pins
    // that the warn still fires for a multi-segment prefix.
    let yaml = r#"
name: test
prefix: my/robot
nodes:
  - id: camera
    type: camera
    outputs:
      - name: image
        schema: test/Data
        max_slice_len: 1024
  - id: viewer
    type: viewer
    inputs:
      - name: img
        source: /my/robot/camera/imge
"#;
    let config = parse_graph(yaml).unwrap();
    validate_graph(&config)
        .expect("in-prefix (multi-segment) absolute miss must validate as external");
    logs_assert(|lines: &[&str]| {
        let warns = lines
            .iter()
            .filter(|l| {
                l.contains("matches no declared output")
                    && l.contains("treating it as an EXTERNAL topic")
            })
            .count();
        if warns == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 in-prefix-miss warn for a multi-segment prefix, got {warns}"
            ))
        }
    });
}

#[test]
#[tracing_test::traced_test]
fn test_in_prefix_derived_spelling_of_override_warn_hints_at_absolute() {
    // The derived spelling of an output whose topic is OVERRIDDEN is an
    // in-prefix miss — the graph validates (warn, not rejection, per the
    // flip above) and the warn must carry the override did-you-mean.
    let yaml = r#"
name: test
prefix: p
nodes:
  - id: bc
    type: tf_broadcaster
    outputs:
      - name: tf
        schema: test/Data
        max_slice_len: 1024
        topic: /tf
  - id: loc
    type: localizer
    inputs:
      - name: tf
        source: /p/bc/tf
"#;
    let config = parse_graph(yaml).unwrap();
    validate_graph(&config).expect("derived spelling of an override validates (warn, not error)");
    assert!(
        logs_contain("reference it as `source: /tf`"),
        "the warn must carry the override did-you-mean"
    );
}

#[test]
fn test_validate_graph_malformed_absolute_sources_rejected() {
    // Shape guards: a malformed absolute reference
    // would produce empty path segments (invalid in zenoh key
    // expressions and nonsensical as canonical names).
    for bad in ["/", "/x/", "/x//y", "/x/*y", "/x/c#m", "/x/@v"] {
        let yaml = format!(
            r#"
name: test
nodes:
  - id: sub1
    type: consumer
    inputs:
      - name: data
        source: "{bad}"
"#
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config)
            .expect_err(&format!(
                "malformed absolute source '{bad}' must be rejected"
            ))
            .to_string();
        assert!(
            err.contains("malformed"),
            "'{bad}' rejection must say malformed: {err}"
        );
    }
}

#[test]
fn test_validate_graph_bad_prefix_rejected() {
    // Canonical names are /{prefix}/... — a prefix
    // with a leading/trailing slash (or empty segments) would produce
    // '//' in every derived topic. Empty prefixes never reach
    // validate_graph via parse_graph (it fills the default), so the
    // empty arm uses a hand-built config.
    for bad in ["/p", "p/", "a//b"] {
        let yaml = format!(
            r#"
name: test
prefix: "{bad}"
nodes:
  - id: src
    type: producer
    outputs:
      - name: out
        schema: test/Data
        max_slice_len: 1024
"#
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config)
            .expect_err(&format!("prefix '{bad}' must be rejected"))
            .to_string();
        assert!(err.contains("prefix"), "'{bad}' rejection: {err}");
    }
    let mut config = parse_graph(
        r#"
name: test
nodes:
  - id: src
    type: producer
    outputs:
      - name: out
        schema: test/Data
        max_slice_len: 1024
"#,
    )
    .unwrap();
    config.prefix = String::new();
    let err = validate_graph(&config)
        .expect_err("empty prefix must be rejected")
        .to_string();
    assert!(err.contains("prefix"), "empty-prefix rejection: {err}");
}

#[test]
fn test_validate_graph_prefix_with_zenoh_reserved_char_rejected() {
    // The prefix is part of every derived topic
    // name, so a zenoh-reserved char (`*`, `?`, `#`, `$`, `@`) must be rejected
    // loudly at graph-load — not left to fail opaquely at runtime. Mirrors
    // `malformed_absolute_name`'s rejection for absolute `topic:`/`source:`
    // names. The message must name the REAL problem (reserved char), not the
    // structural slash rule.
    for bad in ["cam*", "a?b", "p#1", "x$y", "n@m"] {
        let yaml = format!(
            r#"
name: test
prefix: "{bad}"
nodes:
  - id: src
    type: producer
    outputs:
      - name: out
        schema: test/Data
        max_slice_len: 1024
"#
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config)
            .expect_err(&format!("prefix '{bad}' must be rejected"))
            .to_string();
        assert!(
            err.contains("zenoh-reserved"),
            "'{bad}' must be rejected as a zenoh-reserved-char prefix with an \
             accurate message, not the structural slash one: {err}"
        );
    }

    // A clean prefix must STILL validate (no false rejection from the new gate).
    let ok = parse_graph(
        r#"
name: test
prefix: robot1
nodes:
  - id: src
    type: producer
    outputs:
      - name: out
        schema: test/Data
        max_slice_len: 1024
"#,
    )
    .unwrap();
    validate_graph(&ok).expect("a clean prefix must validate");
}

#[test]
fn test_output_topic_override_parses_and_validates() {
    // `topic:` on an output overrides the derived
    // name with an absolute one; an absolute `source:` matching it
    // validates (the producer exists in-graph).
    let yaml = r#"
name: test
prefix: p
nodes:
  - id: broadcaster
    type: tf_broadcaster
    outputs:
      - name: tf
        schema: test/Data
        max_slice_len: 1024
        topic: /tf
  - id: localizer
    type: localizer
    inputs:
      - name: tf
        source: /tf
"#;
    let config = parse_graph(yaml).unwrap();
    assert_eq!(config.nodes[0].outputs[0].topic.as_deref(), Some("/tf"));
    validate_graph(&config).expect("override + absolute consumer must validate");
    // Round-trip: absent `topic:` keys stay absent.
    let yaml_no_override = r#"
name: test
prefix: p
nodes:
  - id: src
    type: producer
    outputs:
      - name: out
        schema: test/Data
        max_slice_len: 1024
"#;
    let config = parse_graph(yaml_no_override).unwrap();
    assert_eq!(config.nodes[0].outputs[0].topic, None);
    let reserialized = serde_yaml::to_string(&config).expect("serialize");
    assert!(
        !reserialized.contains("topic:"),
        "absent overrides must not serialize: {reserialized}"
    );
}

#[test]
fn test_relative_topic_override_rejected_with_did_you_mean() {
    let yaml = r#"
name: test
prefix: p
nodes:
  - id: broadcaster
    type: tf_broadcaster
    outputs:
      - name: tf
        schema: test/Data
        max_slice_len: 1024
        topic: tf
"#;
    let config = parse_graph(yaml).unwrap();
    let err = validate_graph(&config)
        .expect_err("relative override")
        .to_string();
    assert!(
        err.contains("must be ABSOLUTE") && err.contains("did you mean `topic: /tf`"),
        "got: {err}"
    );
}

#[test]
fn test_malformed_topic_overrides_rejected() {
    for bad in ["/", "/tf/", "/a//b", "/t$f", "/t?f", "/@tf"] {
        let yaml = format!(
            r#"
name: test
prefix: p
nodes:
  - id: broadcaster
    type: tf_broadcaster
    outputs:
      - name: tf
        schema: test/Data
        max_slice_len: 1024
        topic: "{bad}"
"#
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config)
            .expect_err(&format!("malformed override '{bad}'"))
            .to_string();
        assert!(err.contains("malformed"), "'{bad}': {err}");
    }
}

#[test]
fn test_relative_ref_to_overridden_output_gets_actionable_error() {
    // A relative short-ref naming an overridden output
    // misses the topic set (the derived name no longer exists) — the
    // error must point at the absolute name, not claim non-existence.
    let yaml = r#"
name: test
prefix: p
nodes:
  - id: broadcaster
    type: tf_broadcaster
    outputs:
      - name: tf
        schema: test/Data
        max_slice_len: 1024
        topic: /tf
  - id: localizer
    type: localizer
    inputs:
      - name: tf
        source: broadcaster/tf
"#;
    let config = parse_graph(yaml).unwrap();
    let err = validate_graph(&config)
        .expect_err("relative ref")
        .to_string();
    assert!(
        err.contains("publishes the absolute topic '/tf'") && err.contains("`source: /tf`"),
        "got: {err}"
    );
}

#[test]
fn test_colliding_topic_overrides_rejected() {
    // Two outputs overriding to the same absolute name collide via the
    // existing duplicate-topic check (override-vs-derived collisions ride
    // the same set).
    let yaml = r#"
name: test
prefix: p
nodes:
  - id: a
    type: t1
    outputs:
      - name: x
        schema: test/Data
        max_slice_len: 1024
        topic: /tf
  - id: b
    type: t2
    outputs:
      - name: y
        schema: test/Data
        max_slice_len: 1024
        topic: /tf
"#;
    let config = parse_graph(yaml).unwrap();
    let err = validate_graph(&config)
        .expect_err("colliding overrides")
        .to_string();
    assert!(
        err.contains("duplicate output topic") && err.contains("/tf"),
        "got: {err}"
    );
}

#[test]
fn test_validate_graph_duplicate_input_name_rejected() {
    // Two same-named inputs on one
    // node collide on every (node_id, input) wiring key — the second
    // silently overwrites the first in the subscriber map and the per-input
    // backpressure maps (for `block`, orphaning the first edge's
    // outstanding mirror → the producer defers forever with no failure at
    // the wiring assert, since each registration lands on a fresh
    // subscriber). The validator must reject it at graph load.
    let yaml = r#"
name: test
nodes:
  - id: src_a
    type: producer
    outputs:
      - name: out
        schema: Vector3
  - id: src_b
    type: producer
    outputs:
      - name: out
        schema: Vector3
  - id: sink
    type: consumer
    inputs:
      - name: data
        source: src_a/out
      - name: data
        source: src_b/out
"#;
    let config = parse_graph(yaml).unwrap();
    let err = validate_graph(&config)
        .expect_err("validate must reject duplicate input names on one node")
        .to_string();
    assert!(
        err.contains("duplicate input name"),
        "error should say `duplicate input name`: {}",
        err
    );
    // Role-binding: pin which substring is the node and which is
    // the input — `contains("sink") && contains("data")` alone survives a
    // swapped format-arg mutation.
    assert!(
        err.contains("node 'sink'") && err.contains("input name 'data'"),
        "error must bind the node and input names to the right roles: {}",
        err
    );
}

#[test]
fn test_parse_perception_fixture() {
    let yaml = include_str!("../fixtures/test_graph.yaml");
    let config = parse_graph(yaml).unwrap();
    validate_graph(&config).unwrap();
    assert_eq!(config.nodes.len(), 6, "fixture should have 6 nodes");

    let ids: Vec<&str> = config.nodes.iter().map(|n| n.id.as_str()).collect();
    assert!(ids.contains(&"camera"), "missing camera node");
    assert!(ids.contains(&"detector"), "missing detector node");
    assert!(ids.contains(&"imu"), "missing imu node");
    assert!(ids.contains(&"fusion"), "missing fusion node");
    assert!(ids.contains(&"tracker"), "missing tracker node");
    assert!(ids.contains(&"diagnostics"), "missing diagnostics node");

    // The fixture demonstrates the absolute-topic
    // surface — a `topic:` override and an absolute external source —
    // and both pass validation (the override is absolute; the external
    // source is exempt from the in-graph-producer requirement).
    let detector = config.nodes.iter().find(|n| n.id == "detector").unwrap();
    let detections = detector
        .outputs
        .iter()
        .find(|o| o.name == "detections")
        .expect("detector must declare the detections output");
    assert_eq!(
        detections.topic.as_deref(),
        Some("/detections"),
        "detector output must carry the absolute override"
    );
    let diagnostics = config.nodes.iter().find(|n| n.id == "diagnostics").unwrap();
    let health = diagnostics
        .inputs
        .iter()
        .find(|i| i.name == "health")
        .expect("diagnostics must declare the health input");
    assert_eq!(
        health.source, "/external/health",
        "diagnostics must consume the absolute external source"
    );
    // The fixture demonstrates the multi-publisher
    // opt-in surface — the listed absolute topic parses and validates.
    assert_eq!(
        config.multi_publisher_topics,
        vec!["/detections".to_string()],
        "fixture must carry the multi_publisher_topics opt-in"
    );
}

/// Hand-derived oracle for the trigger-aware DAG
/// levels of `fixtures/test_graph.yaml`.
///
/// The fixture's trigger policies come from the node MACROS (the YAML is
/// topology-only); this oracle reflects them as documented in the fixture
/// comments:
///
/// | node | policy (macro) | triggering inputs |
/// |---|---|---|
/// | camera | source (period) | none → root |
/// | detector | data-trigger on `image` | camera/image |
/// | imu | source (period) | none → root |
/// | fusion | Sync (macro decides) | camera/image AND imu/data |
/// | tracker | (no inputs/outputs) | none → level-0 root |
/// | diagnostics | External | none → root |
///
/// Triggering edges: detector←camera/image, fusion←camera/image,
/// fusion←imu/data.
///
/// `tracker` has no inputs and no outputs, so it has NO topic edges — but
/// `GraphTopology` captures the full `config.nodes` set at build, so it is
/// still placed as a level-0 root (in-degree 0). The levelization is
/// COMPLETE: every node has a level, so the p3/p4 executor can never
/// silently skip a disconnected (e.g. side-effect-only) node.
#[test]
fn test_fixture_trigger_aware_levels() {
    use cerulion_core::graph::{GraphTopology, TriggerEdges};

    let yaml = include_str!("../fixtures/test_graph.yaml");
    // Pin the prefix so the resolved topic names are deterministic in the
    // oracle (the fixture omits `prefix:`, which would default to the
    // hostname).
    let mut config = parse_graph(yaml).unwrap();
    config.prefix = "perception".to_string();
    validate_graph(&config).unwrap();

    // Topology is policy-free; supply empty NodeInfo per node (build only
    // reads input_meta for policy/depth, which the oracle does not exercise
    // — the trigger classification is supplied separately, mirroring the
    // runtime's `build_trigger_edges`).
    let entry_infos: IndexMap<String, NodeInfo> = config
        .nodes
        .iter()
        .map(|n| (n.id.clone(), NodeInfo::from_names(vec![], vec![])))
        .collect();
    let topo = GraphTopology::build(&config, &entry_infos).expect("topology builds");

    // Hand-built trigger-edge set, FAITHFUL to what `build_trigger_edges`
    // would emit for the fixture's macro policies (data-trigger detector +
    // Sync fusion → these three edges; Period camera/imu, input-less tracker,
    // and External diagnostics → NO triggering edges).
    let mut edges = TriggerEdges::new();
    edges.insert("detector", "/perception/camera/image");
    edges.insert("fusion", "/perception/camera/image");
    edges.insert("fusion", "/perception/imu/data");
    // NOTE: diagnostics is External (and consumes the producer-less
    // `/external/health`), so `build_trigger_edges` emits NO triggering edge
    // for it — it is a level-0 root with in-degree 0. We deliberately do NOT
    // hand-insert a `("diagnostics", "/external/health")` edge here: External
    // policy never yields a triggering edge, so doing so would model an edge
    // the real runtime cannot produce. The orthogonal "a TRIGGERING edge to a
    // producer-less topic adds no DAG edge (consumer stays L0)" contract is
    // covered cleanly — without this External inconsistency — by
    // `topology::tests::levels_external_trigger_on_producerless_topic_is_root`.

    let levels = topo.derive_levels(&edges).expect("fixture is acyclic");

    // Hand-derived oracle vectors. Within-level order is `config.nodes`
    // order: camera, detector, imu, fusion, tracker, diagnostics.
    let actual: Vec<Vec<String>> = levels.iter().map(|l| l.nodes.clone()).collect();
    assert_eq!(
        actual,
        vec![
            vec![
                "camera".to_string(),
                "imu".to_string(),
                "tracker".to_string(),
                "diagnostics".to_string(),
            ],
            vec!["detector".to_string(), "fusion".to_string()],
        ],
        "fixture levels (tracker is a level-0 root — disconnected but complete)"
    );

    // Spot checks.
    assert_eq!(levels.level_of("camera"), Some(0));
    assert_eq!(levels.level_of("imu"), Some(0));
    assert_eq!(levels.level_of("diagnostics"), Some(0));
    assert_eq!(levels.level_of("detector"), Some(1));
    assert_eq!(
        levels.level_of("fusion"),
        Some(1),
        "fusion depends on camera (L0) and imu (L0) → max+1 = 1"
    );
    // tracker is fully disconnected (no edges) but still placed — the
    // levelization is complete (config.nodes-seeded) → it is a level-0 root.
    assert_eq!(
        levels.level_of("tracker"),
        Some(0),
        "a node with no edges is still placed as a level-0 root (complete \
         levelization — the executor never silently skips it)"
    );
    assert_eq!(
        levels.len(),
        2,
        "two levels (the longest chain is length 2)"
    );
}

// ============================================================
// Runtime tests: SHM-backed publish path inside ClosureNodeEntry.
//
// Legacy used `ctx.publisher_mut("data").publish(&heap_struct)`. New
// equivalent: `ctx.publisher_mut("data").loan_proxy::<Vector3>()` and
// write into the SHM-backed proxy.
// ============================================================

#[test]
fn test_run_simple_graph() {
    let prefix = unique_prefix("simple");
    let yaml = format!(
        r#"
name: simple_graph
prefix: {prefix}
nodes:
  - id: publisher
    type: test_counter
    outputs:
      - name: data
        schema: geometry_msgs/Vector3
        max_slice_len: 256
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let pub_counter = Arc::new(AtomicU32::new(0));
    let pub_counter_clone = Arc::clone(&pub_counter);

    let pub_node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["data".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let n = pub_counter_clone.fetch_add(1, Ordering::Relaxed) + 1;
            if let Some(publisher) = ctx.publisher_mut("data") {
                let mut proxy = publisher.loan_proxy::<Vector3>()?;
                proxy.x = n as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
                // Drop publishes.
            }
            Ok(())
        },
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("publisher".to_string(), Box::new(pub_node));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build runtime");

    runtime.step(Duration::from_millis(10));
    {
        let handle = runtime.node_handle("publisher").unwrap();
        assert_eq!(
            handle.fire_count(),
            1,
            "publisher should fire once at t=10ms"
        );
    }
    assert_eq!(pub_counter.load(Ordering::Relaxed), 1);

    runtime.step(Duration::from_millis(10));
    {
        let handle = runtime.node_handle("publisher").unwrap();
        assert_eq!(
            handle.fire_count(),
            2,
            "publisher should fire twice at t=20ms"
        );
    }
}

/// Production trace cap: `GraphRuntime::set_trace_limit(4)`
/// turns the execution trace into a ring keeping the NEWEST 4 entries. A
/// Period(10ms) node stepped 8×10ms fires 8 times (fire_count carries the full
/// truth) while the trace retains exactly the LAST 4 consecutive fires — the
/// newest, not the oldest / a sample. This is the runtime-level pin for the
/// cap `graph run` / `graph run-worker` apply after every production build
/// (the scheduler-level builder-equivalence pin lives in
/// `scheduler/mod.rs::set_trace_limit_matches_builder`).
#[test]
fn test_set_trace_limit_caps_runtime_trace_to_newest() {
    let prefix = unique_prefix("tracecap");
    let yaml = format!(
        r#"
name: tracecap_graph
prefix: {prefix}
nodes:
  - id: ticker
    type: test_ticker
    outputs:
      - name: data
        schema: geometry_msgs/Vector3
        max_slice_len: 256
"#
    );
    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["data".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        |ctx| {
            if let Some(publisher) = ctx.publisher_mut("data") {
                let mut proxy = publisher.loan_proxy::<Vector3>()?;
                proxy.x = 1.0;
            }
            Ok(())
        },
    );
    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("ticker".to_string(), Box::new(node));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build runtime");
    runtime.set_trace_limit(4);

    for _ in 0..8 {
        runtime.step(Duration::from_millis(10));
    }
    {
        let handle = runtime.node_handle("ticker").unwrap();
        assert_eq!(handle.fire_count(), 8, "all 8 fires happened");
    }

    let trace = runtime.trace();
    assert_eq!(trace.len(), 4, "trace ring capped at 4");
    assert!(
        trace.iter().all(|e| &*e.node_id == "ticker"),
        "every retained entry is the ticker's"
    );
    let steps: Vec<u64> = trace.iter().map(|e| e.step).collect();
    let max = *steps.last().expect("non-empty");
    // Anchor `max` to the FINAL step absolutely (8 steps, zero-indexed -> 7):
    // without this, `[max-3..=max]` also passes for an oldest-4 ring (steps
    // 0..=3, max == 3) — the relative window alone cannot distinguish the
    // eviction direction.
    assert_eq!(
        max, 7,
        "newest-4 retention: the last retained step is the 8th fire (step 7)"
    );
    assert_eq!(
        steps,
        vec![max - 3, max - 2, max - 1, max],
        "retained entries must be the NEWEST 4 consecutive fires (ring drops the oldest)"
    );
}

#[test]
fn test_replay_identical_to_live() {
    let trace1 = run_deterministic_graph("replay_1");
    let trace2 = run_deterministic_graph("replay_2");

    assert_eq!(
        trace1.len(),
        trace2.len(),
        "traces must have same number of entries"
    );

    for (i, (t1, t2)) in trace1.iter().zip(trace2.iter()).enumerate() {
        assert_eq!(
            t1.0, t2.0,
            "trace[{}] node_id mismatch: {:?} vs {:?}",
            i, t1.0, t2.0
        );
        assert_eq!(
            t1.1, t2.1,
            "trace[{}] fire_time mismatch: {} vs {}",
            i, t1.1, t2.1
        );
    }
}

/// Build and step a deterministic graph with two no-IO nodes; return the trace
/// as `(node_id, fire_time_ns)` pairs. Used to verify replay-equivalence
/// (Principle #7).
fn run_deterministic_graph(suffix: &str) -> Vec<(String, u64)> {
    let prefix = unique_prefix(suffix);
    let yaml = format!(
        r#"
name: replay_test
prefix: {prefix}
nodes:
  - id: fast
    type: fast_ticker
  - id: slow
    type: slow_ticker
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let fast_node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::Period { period_ms: 10 }),
        |_ctx| Ok(()),
    );

    let slow_node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::Period { period_ms: 20 }),
        |_ctx| Ok(()),
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("fast".to_string(), Box::new(fast_node));
    nodes.insert("slow".to_string(), Box::new(slow_node));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    for _ in 0..5 {
        runtime.step(Duration::from_millis(10));
    }

    runtime
        .trace()
        .iter()
        .map(|entry| (entry.node_id.to_string(), entry.fire_time_ns))
        .collect()
}

#[test]
fn test_graph_two_node_pipeline() {
    let prefix = unique_prefix("pipeline");
    let yaml = format!(
        r#"
name: pipeline
prefix: {prefix}
nodes:
  - id: producer
    type: counter
    outputs:
      - name: data
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: consumer
    type: echo
    inputs:
      - name: data
        source: producer/data
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let producer_fires = Arc::new(AtomicU32::new(0));
    let producer_fires_clone = Arc::clone(&producer_fires);
    let consumer_fires = Arc::new(AtomicU32::new(0));
    let consumer_fires_clone = Arc::clone(&consumer_fires);

    let producer = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["data".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let n = producer_fires_clone.fetch_add(1, Ordering::Relaxed) + 1;
            if let Some(pub_port) = ctx.publisher_mut("data") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = n as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    let consumer = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["data".to_string()], vec![]),
        move |_ctx| {
            consumer_fires_clone.fetch_add(1, Ordering::Relaxed);
            Ok(())
        },
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("producer".to_string(), Box::new(producer));
    nodes.insert("consumer".to_string(), Box::new(consumer));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    for _ in 0..3 {
        runtime.step(Duration::from_millis(10));
    }

    assert_eq!(
        producer_fires.load(Ordering::Relaxed),
        3,
        "producer should fire 3 times"
    );

    // Consumer fire count depends on data-trigger bridging and iceoryx2
    // delivery timing. We verify wiring succeeds and the producer ran;
    // the data-trigger consumer plumbing is covered end-to-end in
    // `in_process_test.rs` and `output_proxy_test.rs`.
}

/// A graph with TWO nodes of the SAME node `type` (distinct ids).
///
/// Multi-camera / multi-IMU / multi-lidar stacks are the norm in real robots: a
/// stereo rig wires two instances of one node type to different output topics.
/// Yet every OTHER test graph here uses nodes of DISTINCT types, so an id-keyed
/// and a type-keyed factory map would be indistinguishable. The hazard —
/// `node_factories` keyed by node *type* instead of node *id* — would pass all
/// of them and break ONLY same-type graphs: the second same-type lookup would
/// miss its already-`swap_remove`d factory (build error), or both instances
/// would collapse onto a single shared factory. The current contract keys by
/// node id (`runtime.rs`: `node_factories.swap_remove(&node_def.id)`); this test
/// pins it.
///
/// NOTE: the in-process heap backend no longer exists, so the once-separate
/// `build()` (IPC) vs `build_in_process()` paths have collapsed to ONE iceoryx2
/// build path (`GraphRuntime::build`) — exercised here.
///
/// Oracle (NOT a self-compare): two `camera` instances publish DISTINCT,
/// id-derived value series (`cam_left` → `1000 + tick`, `cam_right` →
/// `2000 + tick`) to their OWN topics; two `sink` instances each data-trigger on
/// (and read LIVE from) ONE camera, recording what they saw. We assert each sink
/// observed EXACTLY its own producer's current-tick value. A crossed value
/// (left↔right), a stale value, an equal value, or MISSING would all mean the
/// two same-type instances failed to get independent, correctly-routed
/// publishers. (Each sink reads its TRIGGER input, which is served live —
/// non-trigger inputs are step-boundary-frozen, so a single
/// two-input sink would read one input stale; two single-trigger sinks keep both
/// reads current-tick deterministic.)
#[test]
fn test_graph_duplicate_node_type_instances_tick_and_route_independently() {
    const WARMUP: u32 = 3;
    const MEASURED: u32 = 4;
    // Sentinel for a "ctx subscriber read None" so a missed read stays LOUD
    // instead of silently colliding with a real 0 payload.
    const MISSING: f64 = -1.0;

    let prefix = unique_prefix("dup_type");
    // cam_left and cam_right share `type: camera`; sink_left and sink_right share
    // `type: sink`. Same-type instances on BOTH the producer and consumer side.
    let yaml = format!(
        r#"
name: dup_type_graph
prefix: {prefix}
nodes:
  - id: cam_left
    type: camera
    outputs:
      - name: image
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: cam_right
    type: camera
    outputs:
      - name: image
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: sink_left
    type: sink
    inputs:
      - name: in
        source: cam_left/image
  - id: sink_right
    type: sink
    inputs:
      - name: in
        source: cam_right/image
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    // Harness-set per-step base; each camera adds its own offset so the two
    // topics carry provably-distinct data.
    let tick_base = Arc::new(AtomicU64::new(0));
    // Per-instance fire counters PROVE each same-type instance has its OWN boxed
    // state (a type-keyed/shared factory would collapse or drop one box).
    let left_fires = Arc::new(AtomicU32::new(0));
    let right_fires = Arc::new(AtomicU32::new(0));
    // What each sink read from its routed input.
    let left_seen = Arc::new(AtomicU64::new(0));
    let right_seen = Arc::new(AtomicU64::new(0));

    // Build a Period `camera` instance that publishes `offset + tick_base`.
    let mk_camera = |offset: u64, fires: Arc<AtomicU32>, base: Arc<AtomicU64>| {
        ClosureNodeEntry::new(
            NodeInfo::from_names(vec![], vec!["image".to_string()])
                .with_policy(MacroPolicy::Period { period_ms: 10 }),
            move |ctx| {
                fires.fetch_add(1, Ordering::Relaxed);
                let v = (offset + base.load(Ordering::Relaxed)) as f64;
                if let Some(pub_port) = ctx.publisher_mut("image") {
                    let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                    proxy.x = v;
                    proxy.y = 0.0;
                    proxy.z = 0.0;
                }
                Ok(())
            },
        )
    };

    // Build a `sink` instance that data-triggers on `in` and records its LIVE
    // (current-tick) read into `seen`.
    let mk_sink = |seen: Arc<AtomicU64>| {
        ClosureNodeEntry::new(
            NodeInfo::from_names(vec!["in".to_string()], vec![]).with_policy(
                MacroPolicy::DataTrigger {
                    input_name: "in".to_string(),
                },
            ),
            move |ctx| {
                let v = ctx
                    .subscriber_mut("in")
                    .and_then(|s| s.try_view::<Vector3, _>(|view| view.x).ok().flatten())
                    .unwrap_or(MISSING);
                seen.store(v as i64 as u64, Ordering::Relaxed);
                Ok(())
            },
        )
    };

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert(
        "cam_left".to_string(),
        Box::new(mk_camera(
            1000,
            Arc::clone(&left_fires),
            Arc::clone(&tick_base),
        )),
    );
    nodes.insert(
        "cam_right".to_string(),
        Box::new(mk_camera(
            2000,
            Arc::clone(&right_fires),
            Arc::clone(&tick_base),
        )),
    );
    nodes.insert(
        "sink_left".to_string(),
        Box::new(mk_sink(Arc::clone(&left_seen))),
    );
    nodes.insert(
        "sink_right".to_string(),
        Box::new(mk_sink(Arc::clone(&right_seen))),
    );

    // Acceptance #1: the SAME-TYPE graph BUILDS. A type-keyed factory map would
    // fail right here — the second `camera`/`sink` lookup would miss its factory.
    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    // WARMUP: establish iceoryx2 connections so measured reads can't hit the
    // cold-start race (mirrors test_p3_chain_propagates_value_same_tick).
    for _ in 0..WARMUP {
        runtime.step(Duration::from_millis(10));
    }

    // MEASURED: each step both cameras publish their DISTINCT id-derived value;
    // assert each sink read EXACTLY its own producer's current-tick value.
    for tick in 0..MEASURED {
        tick_base.store(tick as u64, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));

        let l = left_seen.load(Ordering::Relaxed) as i64 as f64;
        let r = right_seen.load(Ordering::Relaxed) as i64 as f64;

        assert_ne!(l, MISSING, "sink_left read None on a MEASURED step");
        assert_ne!(r, MISSING, "sink_right read None on a MEASURED step");
        assert_eq!(
            l,
            (1000 + tick as u64) as f64,
            "sink_left must carry cam_left's current-tick value — a crossed/stale \
             value means the two same-type instances did not get independent publishers"
        );
        assert_eq!(
            r,
            (2000 + tick as u64) as f64,
            "sink_right must carry cam_right's current-tick value — a crossed/stale \
             value means the two same-type instances did not get independent publishers"
        );
    }

    // Acceptance #2: BOTH same-type instances ticked INDEPENDENTLY — equal fire
    // counts via the scheduler's observable state AND via each instance's own
    // captured counter (proving distinct boxes, not a shared one).
    let total = (WARMUP + MEASURED) as u64;
    assert_eq!(
        runtime.node_handle("cam_left").unwrap().fire_count(),
        total,
        "cam_left should fire every step"
    );
    assert_eq!(
        runtime.node_handle("cam_right").unwrap().fire_count(),
        total,
        "cam_right should fire every step"
    );
    assert_eq!(
        left_fires.load(Ordering::Relaxed) as u64,
        total,
        "cam_left's own closure must have run every step (independent box state)"
    );
    assert_eq!(
        right_fires.load(Ordering::Relaxed) as u64,
        total,
        "cam_right's own closure must have run every step (independent box state)"
    );
}

#[test]
fn test_graph_shutdown_clean() {
    let prefix = unique_prefix("shutdown");
    let yaml = format!(
        r#"
name: shutdown_test
prefix: {prefix}
nodes:
  - id: node1
    type: test
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::Period { period_ms: 10 }),
        |_ctx| Ok(()),
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("node1".to_string(), Box::new(node));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");
    runtime.step(Duration::from_millis(10));

    runtime.shutdown();
}

#[test]
fn test_graph_config_accessor() {
    let prefix = unique_prefix("accessor");
    let yaml = format!(
        r#"
name: my_graph
prefix: {prefix}
nodes:
  - id: node1
    type: test
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::Period { period_ms: 50 }),
        |_ctx| Ok(()),
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("node1".to_string(), Box::new(node));

    let runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    assert_eq!(runtime.config().identity(), "my_graph");
    assert!(runtime.node_handle("node1").is_some());
    assert!(runtime.node_handle("nonexistent").is_none());
}

#[test]
fn test_graph_build_missing_node_entry() {
    let prefix = unique_prefix("missing_entry");
    let yaml = format!(
        r#"
name: test
prefix: {prefix}
nodes:
  - id: node1
    type: test
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let result = GraphRuntime::build(config, nodes, &mgr, clock);
    assert!(result.is_err(), "build should fail with missing node entry");
}

#[test]
fn test_graph_external_trigger() {
    let prefix = unique_prefix("external");
    let yaml = format!(
        r#"
name: test
prefix: {prefix}
nodes:
  - id: ext_node
    type: test
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let fires = Arc::new(AtomicU32::new(0));
    let fires_clone = Arc::clone(&fires);

    let node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::External),
        move |_ctx| {
            fires_clone.fetch_add(1, Ordering::Relaxed);
            Ok(())
        },
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("ext_node".to_string(), Box::new(node));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    runtime.step(Duration::from_millis(100));
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "external node should not fire without trigger"
    );
}

#[test]
fn test_graph_trace_clear() {
    let prefix = unique_prefix("trace_clear");
    let yaml = format!(
        r#"
name: test
prefix: {prefix}
nodes:
  - id: node1
    type: test
"#
    );

    let config = parse_graph(&yaml).unwrap();
    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::Period { period_ms: 10 }),
        |_ctx| Ok(()),
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("node1".to_string(), Box::new(node));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    runtime.step(Duration::from_millis(10));
    assert!(!runtime.trace().is_empty(), "trace should have entries");

    runtime.clear_trace();
    assert!(
        runtime.trace().is_empty(),
        "trace should be empty after clear"
    );
}

#[test]
fn test_simulated_clock_not_wall_clock() {
    let prefix = unique_prefix("simclock");
    let yaml = format!(
        r#"
name: simclock_test
prefix: {prefix}
nodes:
  - id: tick_node
    type: test
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    assert_eq!(clock.now_ns(), 0, "simulated clock should start at 0");

    let node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::Period { period_ms: 10 }),
        |_ctx| Ok(()),
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("tick_node".to_string(), Box::new(node));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, Arc::clone(&clock)).expect("build");

    runtime.step(Duration::from_millis(10));

    assert_eq!(
        clock.now_ns(),
        10_000_000,
        "simulated clock should be exactly 10ms"
    );

    let trace = runtime.trace();
    assert_eq!(trace.len(), 1, "one fire expected");
    assert_eq!(
        trace[0].fire_time_ns, 10_000_000,
        "fire_time_ns should be from VirtualClock (10ms), not wall clock"
    );
}

#[test]
fn test_multi_publisher_topics_entry_validation() {
    // List entries must be absolute, well-formed, and
    // unique — each rejection carries its own actionable text.
    let base = r#"
name: test
prefix: p
nodes:
  - id: bc
    type: tf_broadcaster
    outputs:
      - name: tf
        schema: test/Data
        max_slice_len: 1024
        topic: /tf
multi_publisher_topics:
"#;
    // Relative entry: did-you-mean.
    let yaml = format!("{base}  - tf\n");
    let err = validate_graph(&parse_graph(&yaml).unwrap())
        .expect_err("relative entry")
        .to_string();
    assert!(
        err.contains("must be ABSOLUTE") && err.contains("did you mean '/tf'"),
        "got: {err}"
    );
    // Malformed entry.
    let yaml = format!("{base}  - /tf//x\n");
    let err = validate_graph(&parse_graph(&yaml).unwrap())
        .expect_err("malformed entry")
        .to_string();
    assert!(err.contains("is malformed"), "got: {err}");
    // Duplicate entry.
    let yaml = format!("{base}  - /tf\n  - /tf\n");
    let err = validate_graph(&parse_graph(&yaml).unwrap())
        .expect_err("duplicate entry")
        .to_string();
    assert!(err.contains("more than once"), "got: {err}");
    // The happy entry validates.
    let yaml = format!("{base}  - /tf\n");
    validate_graph(&parse_graph(&yaml).unwrap()).expect("well-formed listing validates");
}

#[test]
fn test_multi_publisher_listing_relaxes_duplicate_output_topic() {
    // Two outputs overriding to the same absolute topic
    // are rejected UNLESS listed — and the unlisted rejection names the
    // opt-in remedy.
    let base = r#"
name: test
prefix: p
nodes:
  - id: bc_a
    type: tf_broadcaster
    outputs:
      - name: tf
        schema: test/Data
        max_slice_len: 1024
        topic: /tf
  - id: bc_b
    type: tf_broadcaster
    outputs:
      - name: tf
        schema: test/Data
        max_slice_len: 1024
        topic: /tf
"#;
    let err = validate_graph(&parse_graph(base).unwrap())
        .expect_err("unlisted double-producer")
        .to_string();
    assert!(
        err.contains("duplicate output topic") && err.contains("multi_publisher_topics"),
        "the rejection must carry the opt-in remedy: {err}"
    );
    let yaml = format!("{base}multi_publisher_topics:\n  - /tf\n");
    validate_graph(&parse_graph(&yaml).unwrap()).expect("listed double-producer must validate");
}

#[test]
#[tracing_test::traced_test]
fn test_multi_publisher_unused_listing_warns() {
    // Loud over silent: a listed topic nothing
    // references is almost certainly a list typo — exactly one warn; a
    // referenced listing must NOT warn (the exactly-1 count catches a
    // variant that always warns).
    let yaml = r#"
name: test
prefix: p
nodes:
  - id: bc
    type: tf_broadcaster
    outputs:
      - name: tf
        schema: test/Data
        max_slice_len: 1024
        topic: /tf
  - id: sink
    type: sink
    inputs:
      - name: inp
        source: /consumed/only
multi_publisher_topics:
  - /tf
  - /typo/nothing/references
  - /consumed/only
"#;
    // /tf is PRODUCED (override), /consumed/only is CONSUMED via an
    // absolute source (resolution-aware reference check),
    // /typo/... is unreferenced. Exactly ONE warn.
    validate_graph(&parse_graph(yaml).unwrap()).expect("validates with a warn");
    logs_assert(|lines: &[&str]| {
        let warns = lines
            .iter()
            .filter(|l| l.contains("no output publishes") && l.contains("no input sources"))
            .count();
        if warns == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 unused-listing warn, got {warns}"
            ))
        }
    });
}

// ============================================================
// Drain-between-levels executor proofs.
//
// `GraphRuntime::step()` advances the clock once, then
// for each trigger-edge DAG level in order: drain that level's
// trigger inputs, then fire that level — BEFORE the next level. So a
// producer's same-tick publish is visible to its (next-level)
// data-triggered consumer in the SAME `step()`. These tests are the
// oracle proof that the whole trigger chain collapses to ONE tick.
//
// SERIAL: real iceoryx2 via `GraphRuntime::build`.
// ============================================================

/// THE headline proof: a 3-node data-trigger chain
/// (Period source → DataTrigger relay → DataTrigger sink) collapses
/// to a single tick per `step()`. The oracle is the step count K: a
/// collapsed chain fires source == relay == sink == K.
///
/// Under drain-before-fire this would be source=K, relay=K-1,
/// sink=K-2 (one hop per tick); drain-between-levels makes
/// it source==relay==sink==K — the whole chain fires in a single
/// step().
#[test]
fn test_p3_trigger_chain_collapses_to_one_tick() {
    const K: u32 = 5;

    let prefix = unique_prefix("p3_chain");
    let yaml = format!(
        r#"
name: p3_chain
prefix: {prefix}
nodes:
  - id: source
    type: counter
    outputs:
      - name: data
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: relay
    type: relay
    inputs:
      - name: in
        source: source/data
    outputs:
      - name: mid
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: sink
    type: sink
    inputs:
      - name: in
        source: relay/mid
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let source_fires = Arc::new(AtomicU32::new(0));
    let source_fires_clone = Arc::clone(&source_fires);
    let relay_fires = Arc::new(AtomicU32::new(0));
    let relay_fires_clone = Arc::clone(&relay_fires);
    let sink_fires = Arc::new(AtomicU32::new(0));
    let sink_fires_clone = Arc::clone(&sink_fires);

    // Level 0: Period source. Publishes to `data` every 10 ms step.
    let source = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["data".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let n = source_fires_clone.fetch_add(1, Ordering::Relaxed) + 1;
            if let Some(pub_port) = ctx.publisher_mut("data") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = n as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    // Level 1: DataTrigger on `in` (← source/data). Re-publishes to `mid`.
    let relay = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec!["mid".to_string()]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        move |ctx| {
            let n = relay_fires_clone.fetch_add(1, Ordering::Relaxed) + 1;
            if let Some(pub_port) = ctx.publisher_mut("mid") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = n as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    // Level 2: DataTrigger on `in` (← relay/mid). Terminal.
    let sink = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec![]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        move |_ctx| {
            sink_fires_clone.fetch_add(1, Ordering::Relaxed);
            Ok(())
        },
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("source".to_string(), Box::new(source));
    nodes.insert("relay".to_string(), Box::new(relay));
    nodes.insert("sink".to_string(), Box::new(sink));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    for _ in 0..K {
        runtime.step(Duration::from_millis(10));
    }

    let source_n = source_fires.load(Ordering::Relaxed);
    let relay_n = relay_fires.load(Ordering::Relaxed);
    let sink_n = sink_fires.load(Ordering::Relaxed);

    // Period source fires exactly once per 10 ms step.
    assert_eq!(
        source_n, K,
        "source (Period 10ms) must fire once per step: expected {K}, got {source_n}"
    );

    // The collapse: every level rides the same tick. If the executor were
    // NOT collapsing the chain we'd see relay == K-1 and sink == K-2
    // (one hop per tick) — that lag would FAIL these asserts. This is a
    // COLD-START proof: the lag only shows from step 1, because in steady
    // state the one-hop-per-tick lag is a constant offset that washes out after a
    // warmup+counter-reset (see `test_p3_chain_propagates_value_same_tick`,
    // which deliberately warms up and so CANNOT use the fire-count oracle).
    assert_eq!(
        relay_n,
        K,
        "relay (DataTrigger ← source/data) must fire in the SAME step as source: \
         expected {K}, got {relay_n} (a value of {} would mean a one-tick lag — \
         the executor not collapsing the chain)",
        K - 1
    );
    assert_eq!(
        sink_n,
        K,
        "sink (DataTrigger ← relay/mid) must fire in the SAME step as relay: \
         expected {K}, got {sink_n} (a value of {} would mean a two-tick lag — \
         the executor not collapsing the chain)",
        K - 2
    );

    // THE oracle (non-tautological): the head and the tail of the chain
    // fired the same number of times — the whole chain collapsed to ONE
    // tick per step().
    assert_eq!(
        sink_n, source_n,
        "chain must collapse to one tick: sink_fires ({sink_n}) == source_fires ({source_n})"
    );
}

/// VALUE propagation companion to `test_p3_trigger_chain_collapses_to_one_tick`.
/// Proves the source's CURRENT-tick value reaches the sink within the SAME
/// `step()` (not just that the sink fired the right number of times). Closes
/// the "right schedule, wrong/stale payload" blind spot end-to-end:
/// source writes V, relay forwards what it read, sink records what it read.
///
/// WARMUP IS CORRECT HERE: we step ~3 times before measuring so that BOTH the
/// drain/trigger subscriber AND the body-read `ctx` subscriber establish their
/// iceoryx2 connections. Without the warmup the sink's `ctx` subscriber can
/// miss the relay's FIRST publish before the connection is established (a
/// cold-start connection-warmup race) — that is precisely the flake this split
/// removes. Warmup is the right fix for a VALUE test because value propagation
/// is a steady-state property: the collapse delivers V the same step regardless
/// of cold start.
///
/// WARMUP WOULD BE WRONG for the fire-count proof
/// (`test_p3_trigger_chain_collapses_to_one_tick`): the uncollapsed one-hop-per-
/// tick lag is a CONSTANT offset that washes out in steady state. After a
/// warmup+counter-reset, even a lagged (uncollapsed) executor shows
/// source==relay==sink==K. Only from a COLD start does lagged = [K, K-1, K-2]
/// differ from collapsed = [K, K, K]. So that proof MUST stay cold-start, and
/// this VALUE proof must NOT (the connection race forces a warmup).
///
/// Parallel-safe via `unique_prefix` (per-test SHM root), matching the rest of
/// this file's real-iceoryx2 `GraphRuntime::build` tests.
#[test]
fn test_p3_chain_propagates_value_same_tick() {
    const WARMUP: u32 = 3;
    const MEASURED: u32 = 4;
    // A sentinel the source never publishes; if a measured-step `try_view`
    // returns None (no data — the warmup should make this unreachable), the
    // relay/sink record this and the final assert REJECTS it. This keeps the
    // test from silently degrading to a tautology (e.g. `unwrap_or(0.0)`).
    const MISSING: f64 = -1.0;

    let prefix = unique_prefix("p3_chain_value");
    let yaml = format!(
        r#"
name: p3_chain_value
prefix: {prefix}
nodes:
  - id: source
    type: counter
    outputs:
      - name: data
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: relay
    type: relay
    inputs:
      - name: in
        source: source/data
    outputs:
      - name: mid
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: sink
    type: sink
    inputs:
      - name: in
        source: relay/mid
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    // The value the source publishes this tick (set by the harness before each
    // measured step). The source forwards it to `data`; the relay forwards what
    // it read; the sink records what it read.
    let source_value = Arc::new(AtomicU64::new(0));
    let source_value_src = Arc::clone(&source_value);
    let relay_seen = Arc::new(AtomicU64::new(0));
    let relay_seen_clone = Arc::clone(&relay_seen);
    let sink_seen = Arc::new(AtomicU64::new(0));
    let sink_seen_clone = Arc::clone(&sink_seen);

    // Level 0: Period source. Publishes the harness-chosen value to `data`.
    let source = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["data".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let v = source_value_src.load(Ordering::Relaxed) as f64;
            if let Some(pub_port) = ctx.publisher_mut("data") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = v;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    // Level 1: DataTrigger. FORWARDS the value it read from `in` to `mid`. If
    // the input view is None (no data), forward the MISSING sentinel so a
    // measured-step miss is LOUD downstream rather than silently 0.
    let relay = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec!["mid".to_string()]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        move |ctx| {
            let received = ctx
                .subscriber_mut("in")
                .and_then(|s| s.try_view::<Vector3, _>(|view| view.x).ok().flatten())
                .unwrap_or(MISSING);
            relay_seen_clone.store(received as i64 as u64, Ordering::Relaxed);
            if let Some(pub_port) = ctx.publisher_mut("mid") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = received;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    // Level 2: DataTrigger. RECORDS the value it read from `in`. None → MISSING.
    let sink = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec![]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        move |ctx| {
            let received = ctx
                .subscriber_mut("in")
                .and_then(|s| s.try_view::<Vector3, _>(|view| view.x).ok().flatten())
                .unwrap_or(MISSING);
            sink_seen_clone.store(received as i64 as u64, Ordering::Relaxed);
            Ok(())
        },
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("source".to_string(), Box::new(source));
    nodes.insert("relay".to_string(), Box::new(relay));
    nodes.insert("sink".to_string(), Box::new(sink));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    // WARMUP: establish iceoryx2 connections (drain + ctx subscribers) so the
    // measured steps cannot hit the cold-start connection race. The source
    // publishes a benign warmup value.
    source_value.store(7, Ordering::Relaxed);
    for _ in 0..WARMUP {
        runtime.step(Duration::from_millis(10));
    }

    // MEASURED steps: each one publishes a fresh, known value and asserts the
    // sink saw EXACTLY the source's CURRENT-tick value (same-tick propagation).
    // A lagged executor would show the value from two ticks ago; a missed read
    // would show MISSING — both rejected.
    for tick in 0..MEASURED {
        let v: u64 = 100 + tick as u64;
        source_value.store(v, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));

        let relay_v = relay_seen.load(Ordering::Relaxed) as i64 as f64;
        let sink_v = sink_seen.load(Ordering::Relaxed) as i64 as f64;

        // LOUD-on-None: a measured-step miss surfaces as MISSING, not silent 0.
        assert_ne!(
            relay_v, MISSING,
            "relay read None on a MEASURED step (warmup should make this unreachable): \
             the ctx subscriber missed the source's publish"
        );
        assert_ne!(
            sink_v, MISSING,
            "sink read None on a MEASURED step (warmup should make this unreachable): \
             the ctx subscriber missed the relay's publish"
        );
        // Same-tick value propagation: source's CURRENT value reaches both the
        // relay and the sink within THIS step. A lagged executor would show
        // v - 1 (relay) / v - 2 (sink).
        assert_eq!(
            relay_v, v as f64,
            "relay must read the SOURCE's current-tick value {v}, got {relay_v} \
             (a stale value would mean the payload lagged the schedule)"
        );
        assert_eq!(
            sink_v, v as f64,
            "sink must read the SOURCE's current-tick value {v} end-to-end, got {sink_v} \
             (a stale value would mean the payload lagged the schedule)"
        );
    }
}

/// The drain-between-levels reorder must be deterministic run-to-run.
/// Build the SAME multi-level trigger graph twice (fresh prefix each),
/// run the same steps, and assert the two traces are byte-identical
/// (same length; per-entry `node_id` + `fire_time_ns` equal). This
/// pins the level-grouped firing order as reproducible.
#[test]
fn test_p3_level_order_trace_is_deterministic() {
    let trace1 = run_p3_level_graph("p3_level_1");
    let trace2 = run_p3_level_graph("p3_level_2");

    assert_eq!(
        trace1.len(),
        trace2.len(),
        "level-grouped traces must have the same number of entries: {} vs {}",
        trace1.len(),
        trace2.len()
    );
    // Non-vacuous: a multi-level chain over several steps must have fired.
    assert!(
        !trace1.is_empty(),
        "trace must be non-empty (the chain should have fired)"
    );

    for (i, (t1, t2)) in trace1.iter().zip(trace2.iter()).enumerate() {
        assert_eq!(
            t1.0, t2.0,
            "trace[{i}] node_id mismatch: {:?} vs {:?}",
            t1.0, t2.0
        );
        assert_eq!(
            t1.1, t2.1,
            "trace[{i}] fire_time_ns mismatch: {} vs {}",
            t1.1, t2.1
        );
    }
}

/// Build and step the p3 multi-level trigger graph
/// (Period source → DataTrigger relay → DataTrigger sink); return the
/// trace as `(node_id, fire_time_ns)` pairs. Mirrors
/// `run_deterministic_graph` but exercises the trigger-edge DAG that
/// the level-grouped executor reorders.
fn run_p3_level_graph(suffix: &str) -> Vec<(String, u64)> {
    let prefix = unique_prefix(suffix);
    let yaml = format!(
        r#"
name: p3_level_test
prefix: {prefix}
nodes:
  - id: source
    type: counter
    outputs:
      - name: data
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: relay
    type: relay
    inputs:
      - name: in
        source: source/data
    outputs:
      - name: mid
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: sink
    type: sink
    inputs:
      - name: in
        source: relay/mid
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let source = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["data".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        |ctx| {
            if let Some(pub_port) = ctx.publisher_mut("data") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = 1.0;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    let relay = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec!["mid".to_string()]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        |ctx| {
            if let Some(pub_port) = ctx.publisher_mut("mid") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = 2.0;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    let sink = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec![]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        |_ctx| Ok(()),
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("source".to_string(), Box::new(source));
    nodes.insert("relay".to_string(), Box::new(relay));
    nodes.insert("sink".to_string(), Box::new(sink));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    for _ in 0..5 {
        runtime.step(Duration::from_millis(10));
    }

    runtime
        .trace()
        .iter()
        .map(|entry| (entry.node_id.to_string(), entry.fire_time_ns))
        .collect()
}

/// A wider level — two nodes sharing one level — must fire
/// in GRAPH (IndexMap insertion) order within the level, NOT sorted / HashMap
/// order. This is the executor analog of `on_event_multi_handler`'s
/// declaration-order pin: the linear chain tests (`*_collapses_to_one_tick`)
/// can never reach this because every level there has exactly one node, so
/// `decide_fires`/`tick_decided` only ever run over a single-element slice. Here
/// a single Period source `src` fans out to TWO data-trigger consumers `beta`
/// and `alpha` (declared `beta` FIRST, deliberately reverse-alphabetical) that
/// share level 1; the trace must show `beta` firing before `alpha` within each
/// tick. A sort-by-name or HashMap-random regression in `decide_fires`/
/// `tick_decided` (or in the level-node ordering) would flip this to `alpha`,
/// `beta`.
///
/// Parallel-safe via `unique_prefix` (per-test SHM root), matching the rest of
/// this file's real-iceoryx2 `GraphRuntime::build` tests.
#[test]
fn test_p3_wider_level_fires_in_graph_order() {
    const K: u32 = 4;

    let prefix = unique_prefix("p3_wide");
    // `beta` is declared BEFORE `alpha` (reverse-alphabetical) on purpose:
    // graph order is beta->alpha, so a sorted/HashMap firing would FLIP it.
    let yaml = format!(
        r#"
name: p3_wide
prefix: {prefix}
nodes:
  - id: src
    type: counter
    outputs:
      - name: data
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: beta
    type: sink
    inputs:
      - name: in
        source: src/data
  - id: alpha
    type: sink
    inputs:
      - name: in
        source: src/data
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let src_fires = Arc::new(AtomicU32::new(0));
    let src_fires_clone = Arc::clone(&src_fires);
    let beta_fires = Arc::new(AtomicU32::new(0));
    let beta_fires_clone = Arc::clone(&beta_fires);
    let alpha_fires = Arc::new(AtomicU32::new(0));
    let alpha_fires_clone = Arc::clone(&alpha_fires);

    // Level 0: Period source publishing to `data` every step.
    let src = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["data".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let n = src_fires_clone.fetch_add(1, Ordering::Relaxed) + 1;
            if let Some(pub_port) = ctx.publisher_mut("data") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = n as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    // Level 1, declared FIRST: `beta`, a data-trigger consumer of src/data.
    let beta = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec![]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        move |_ctx| {
            beta_fires_clone.fetch_add(1, Ordering::Relaxed);
            Ok(())
        },
    );

    // Level 1, declared SECOND: `alpha`, also a data-trigger consumer of
    // src/data.
    let alpha = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec![]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        move |_ctx| {
            alpha_fires_clone.fetch_add(1, Ordering::Relaxed);
            Ok(())
        },
    );

    // Insertion order MUST mirror the YAML declaration order (src, beta, alpha)
    // - IndexMap is the graph's source of truth for ordering.
    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("src".to_string(), Box::new(src));
    nodes.insert("beta".to_string(), Box::new(beta));
    nodes.insert("alpha".to_string(), Box::new(alpha));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    for _ in 0..K {
        runtime.step(Duration::from_millis(10));
    }

    // Same-tick collapse for the fan-out: each level-1 consumer fires once per
    // step, in lockstep with the source.
    let src_n = src_fires.load(Ordering::Relaxed);
    let beta_n = beta_fires.load(Ordering::Relaxed);
    let alpha_n = alpha_fires.load(Ordering::Relaxed);
    assert_eq!(
        src_n, K,
        "src (Period) must fire once per step: got {src_n}"
    );
    assert_eq!(
        beta_n, K,
        "beta (data-trigger <- src/data) must fire in the SAME step as src: got {beta_n}"
    );
    assert_eq!(
        alpha_n, K,
        "alpha (data-trigger <- src/data) must fire in the SAME step as src: got {alpha_n}"
    );

    // The within-level ORDER pin: filter the trace to the level-1 nodes and
    // assert that on EVERY tick `beta` (declared first) fired before `alpha`
    // (declared second). A sorted/HashMap-random regression flips this.
    let level1_order: Vec<&str> = runtime
        .trace()
        .iter()
        .filter_map(|e| {
            let id = e.node_id.as_ref();
            (id == "beta" || id == "alpha").then_some(id)
        })
        .collect();
    // K ticks x 2 level-1 nodes.
    assert_eq!(
        level1_order.len(),
        (K as usize) * 2,
        "expected {} level-1 trace entries (K ticks x 2 nodes), got {}: {:?}",
        (K as usize) * 2,
        level1_order.len(),
        level1_order
    );
    // The pattern must be [beta, alpha, beta, alpha, ...] - graph order within
    // each tick. NOT [alpha, beta, ...] (sorted) and NOT interleaved randomly.
    let expected: Vec<&str> = (0..K as usize).flat_map(|_| ["beta", "alpha"]).collect();
    assert_eq!(
        level1_order, expected,
        "level-1 nodes must fire in GRAPH (declaration) order beta->alpha within each tick \
         (a value starting [alpha, beta, ...] would mean decide_fires/tick_decided sorted by \
         name or iterated a HashMap)"
    );
}

/// A diamond fan-in collapses to one tick and the fan-in
/// path works through the level executor.
///
/// Shape: `src(Period) -> {left, right}(DataTrigger, both re-publish) ->
/// join(Sync <- left + right)`. `join` is a bounded `Sync`, NOT a
/// `DataTrigger`, because a fan-in JOIN must wait for BOTH branches:
/// `MacroPolicy::DataTrigger` names a SINGLE input and fires when THAT input
/// arrives (it would fire on whichever branch arrives first, NOT requiring both
/// to co-arrive — so it simply can't express "wait for both"). `Sync` requires
/// ALL its inputs within the window and fires EXACTLY ONCE on co-arrival —
/// which `left` and `right` deliver every tick (both publish on the same
/// level-1 sweep, stamped at the same sim time under `VirtualClock`). So the
/// diamond collapses to one join fire per src fire. The level executor drains
/// all of level 1 before firing level 2, so `join` sees both branches'
/// same-tick publishes.
///
/// Parallel-safe via `unique_prefix` (per-test SHM root), matching the rest of
/// this file's real-iceoryx2 `GraphRuntime::build` tests.
#[test]
fn test_p3_diamond_fan_in_collapses() {
    const K: u32 = 5;

    let prefix = unique_prefix("p3_diamond");
    let yaml = format!(
        r#"
name: p3_diamond
prefix: {prefix}
nodes:
  - id: src
    type: counter
    outputs:
      - name: data
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: left
    type: relay
    inputs:
      - name: in
        source: src/data
    outputs:
      - name: out
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: right
    type: relay
    inputs:
      - name: in
        source: src/data
    outputs:
      - name: out
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: join
    type: sync
    inputs:
      - name: l
        source: left/out
      - name: r
        source: right/out
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let src_fires = Arc::new(AtomicU32::new(0));
    let src_fires_clone = Arc::clone(&src_fires);
    let left_fires = Arc::new(AtomicU32::new(0));
    let left_fires_clone = Arc::clone(&left_fires);
    let right_fires = Arc::new(AtomicU32::new(0));
    let right_fires_clone = Arc::clone(&right_fires);
    let join_fires = Arc::new(AtomicU32::new(0));
    let join_fires_clone = Arc::clone(&join_fires);

    // Level 0: Period source.
    let src = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["data".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let n = src_fires_clone.fetch_add(1, Ordering::Relaxed) + 1;
            if let Some(pub_port) = ctx.publisher_mut("data") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = n as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    // Level 1, branch A: data-trigger <- src/data -> republish to `out`.
    let left = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec!["out".to_string()]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        move |ctx| {
            left_fires_clone.fetch_add(1, Ordering::Relaxed);
            if let Some(pub_port) = ctx.publisher_mut("out") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = 1.0;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    // Level 1, branch B: data-trigger <- src/data -> republish to `out`.
    let right = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec!["out".to_string()]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        move |ctx| {
            right_fires_clone.fetch_add(1, Ordering::Relaxed);
            if let Some(pub_port) = ctx.publisher_mut("out") {
                let mut proxy = pub_port.loan_proxy::<Vector3>()?;
                proxy.x = 2.0;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    // Level 2: Sync over both branches (`l` <- left/out, `r` <- right/out).
    // Fires once when BOTH arrive within the 1000 ms window - every tick.
    // Sync aligns ONLY `#[input(trigger)]`-marked inputs, so this
    // closure declares trigger-marked InputMeta for both (the all-trigger
    // shape — semantics identical to the earlier all-inputs contract;
    // depth/backpressure stay the topology defaults `from_names` implied).
    let sync_meta = |name: &str| cerulion_core::graph::node::InputMeta {
        name: name.to_string(),
        schema_hash: 0,
        trigger: true,
        depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        backpressure: cerulion_core::graph::node::BackpressurePolicy::default(),
        expect_within_ms: None,
    };
    let join = ClosureNodeEntry::new(
        NodeInfo::with_meta(vec![sync_meta("l"), sync_meta("r")], vec![])
            .with_policy(MacroPolicy::Sync { window_ms: 1000 }),
        move |_ctx| {
            join_fires_clone.fetch_add(1, Ordering::Relaxed);
            Ok(())
        },
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("src".to_string(), Box::new(src));
    nodes.insert("left".to_string(), Box::new(left));
    nodes.insert("right".to_string(), Box::new(right));
    nodes.insert("join".to_string(), Box::new(join));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    for _ in 0..K {
        runtime.step(Duration::from_millis(10));
    }

    let src_n = src_fires.load(Ordering::Relaxed);
    let left_n = left_fires.load(Ordering::Relaxed);
    let right_n = right_fires.load(Ordering::Relaxed);
    let join_n = join_fires.load(Ordering::Relaxed);

    assert_eq!(
        src_n, K,
        "src (Period) must fire once per step: got {src_n}"
    );
    // Both branches collapse to the same tick as the source.
    assert_eq!(
        left_n, K,
        "left (data-trigger <- src/data) must fire in the SAME step as src: got {left_n}"
    );
    assert_eq!(
        right_n, K,
        "right (data-trigger <- src/data) must fire in the SAME step as src: got {right_n}"
    );
    // The diamond fan-in collapse: join fires once per src fire (both branches'
    // same-tick publishes drained into level 2 before join fires). A non-
    // collapsing executor would lag join by one or two ticks (join_n < K).
    assert_eq!(
        join_n, K,
        "join (Sync <- left/out + right/out) must fire once per src fire - the diamond \
         collapses to one tick: expected {K}, got {join_n}"
    );
}

// ═══════════ `level_assignments:` yaml block (PURE) ═══════════
//
// Parse + load-time-validation pins for the baked level-assignment block.
// No transport — these run standalone via
// `cargo test -p cerulion_core --test graph_test level_assignments`.

/// The block parses into `GraphConfig::level_assignments` (Some), preserving
/// the yaml-authored key order (IndexMap — deterministic error reporting).
#[test]
fn level_assignments_block_parses_into_config() {
    let yaml = r#"
name: test
prefix: p
level_assignments:
  cam: 1
  imu: 0
  fuse: 2
nodes:
  - id: imu
    type: imu
    outputs:
      - name: out
        schema: test/Data
        max_slice_len: 64
  - id: cam
    type: cam
    outputs:
      - name: out
        schema: test/Data
        max_slice_len: 64
  - id: fuse
    type: fuse
    inputs:
      - name: a
        source: imu/out
      - name: b
        source: cam/out
"#;
    let config = parse_graph(yaml).unwrap();
    let assignments = config
        .level_assignments
        .as_ref()
        .expect("the block must parse into Some");
    assert_eq!(assignments.len(), 3);
    assert_eq!(assignments["cam"], 1);
    assert_eq!(assignments["imu"], 0);
    assert_eq!(assignments["fuse"], 2);
    // Yaml-authored key order preserved (IndexMap, not a sorted/hashed map).
    let keys: Vec<&str> = assignments.keys().map(|k| k.as_str()).collect();
    assert_eq!(keys, vec!["cam", "imu", "fuse"]);
}

/// Absent block ⇒ `None` — the byte-identical Kahn-fallback contract's
/// config half.
#[test]
fn level_assignments_absent_parses_none() {
    let yaml = r#"
name: test
prefix: p
nodes:
  - id: solo
    type: solo
    outputs:
      - name: out
        schema: test/Data
        max_slice_len: 64
"#;
    let config = parse_graph(yaml).unwrap();
    assert!(
        config.level_assignments.is_none(),
        "no block in the yaml must parse to None"
    );
}

/// Round-trip: Some serializes to a `level_assignments:` block that re-parses
/// to the same map; None serializes to NO such key (a re-serialized untouched
/// graph gains no phantom block).
#[test]
fn level_assignments_round_trips_through_serialize() {
    let yaml = r#"
name: test
prefix: p
level_assignments:
  a: 0
  b: 1
nodes:
  - id: a
    type: a
    outputs:
      - name: out
        schema: test/Data
        max_slice_len: 64
  - id: b
    type: b
    inputs:
      - name: i
        source: a/out
"#;
    let config = parse_graph(yaml).unwrap();
    let reserialized = serde_yaml::to_string(&config).expect("serialize");
    assert!(
        reserialized.contains("level_assignments:"),
        "Some must serialize the block: {reserialized}"
    );
    let reparsed = parse_graph(&reserialized).unwrap();
    assert_eq!(
        reparsed.level_assignments, config.level_assignments,
        "the block round-trips"
    );

    // None ⇒ the key is absent on re-serialize (skip_serializing_if).
    let mut without = config;
    without.level_assignments = None;
    let reserialized = serde_yaml::to_string(&without).expect("serialize");
    assert!(
        !reserialized.contains("level_assignments"),
        "None must not serialize a phantom block: {reserialized}"
    );
}

/// The early load-time gate: `validate_graph` rejects the config-only
/// violation classes (unknown key / partial coverage) — same text as the
/// build-time enforcement in `levels_from_assignments` (one voice,
/// defense-in-depth at both boundaries).
#[test]
fn level_assignments_validate_graph_gates_unknown_and_missing() {
    let base = r#"
name: test
prefix: p
level_assignments:
  a: 0
  ghost: 1
nodes:
  - id: a
    type: a
    outputs:
      - name: out
        schema: test/Data
        max_slice_len: 64
  - id: b
    type: b
    inputs:
      - name: i
        source: a/out
"#;
    let config = parse_graph(base).unwrap();
    let err = validate_graph(&config).expect_err("unknown node must gate at load");
    let msg = format!("{err}");
    assert!(msg.contains("unknown node(s) [ghost]"), "got: {msg}");

    // Partial coverage (drop `ghost`, leave `b` unassigned).
    let mut partial = config;
    let mut map = indexmap::IndexMap::new();
    map.insert("a".to_string(), 0usize);
    partial.level_assignments = Some(map);
    let err = validate_graph(&partial).expect_err("partial coverage must gate at load");
    let msg = format!("{err}");
    assert!(msg.contains("must cover EVERY node"), "got: {msg}");
    assert!(msg.contains("missing [b]"), "got: {msg}");

    // Full coverage passes the load gate (edge/contiguity checks are the
    // build's job — validate_graph is config-only).
    let mut full = parse_graph(base).unwrap();
    let mut map = indexmap::IndexMap::new();
    map.insert("a".to_string(), 0usize);
    map.insert("b".to_string(), 1usize);
    full.level_assignments = Some(map);
    validate_graph(&full).expect("full coverage passes the config-only gate");
}

/// DAG-level LOCKSTEP: a Sync node's NON-trigger input is classified
/// NON-triggering by `build_trigger_edges`, so it is not a DAG dependency and
/// `derive_levels` places the node from its TRIGGER inputs only.
///
/// Topology (pure — no transport):
///
/// ```text
///   src (Period)  --raw-->  mid (DataTrigger on `in`)  --refined-->  (fuse.ctx, NON-trigger)
///        \--raw------------------------------------------------->  (fuse.a,   #[input(trigger)])
/// ```
///
/// HAND ORACLE: the levels are `[[src], [mid, fuse]]`; fuse's
/// only DAG edge is its trigger input `a ← src/raw`, so it sits at level 1
/// beside `mid`. Before the flip, Sync classified EVERY input as triggering, so
/// `ctx ← mid/refined` was an edge and fuse sat at level 2 — this test FAILS
/// on the pre-flip classification (level_of("fuse") == Some(2)), pinning the
/// fire-semantics/DAG-levels lockstep (both narrow through
/// `trigger_marked_input_names` in the same change).
#[test]
fn test_sync_non_trigger_input_is_not_a_dag_edge() {
    use cerulion_core::graph::{build_trigger_edges, GraphTopology};

    let yaml = r#"
name: levels
prefix: syncl
nodes:
  - id: src
    type: cam
    outputs:
      - name: raw
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: mid
    type: refiner
    inputs:
      - name: in
        source: src/raw
    outputs:
      - name: refined
        schema: geometry_msgs/Vector3
        max_slice_len: 256
  - id: fuse
    type: fuser
    inputs:
      - name: a
        source: src/raw
      - name: ctx
        source: mid/refined
"#;
    let config = parse_graph(yaml).unwrap();
    validate_graph(&config).unwrap();

    let meta = |name: &str, trigger: bool| cerulion_core::graph::node::InputMeta {
        name: name.to_string(),
        schema_hash: 0,
        trigger,
        depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        backpressure: cerulion_core::graph::node::BackpressurePolicy::default(),
        expect_within_ms: None,
    };
    let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
    infos.insert(
        "src".to_string(),
        NodeInfo::from_names(vec![], vec!["raw".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
    );
    infos.insert(
        "mid".to_string(),
        NodeInfo::from_names(vec!["in".to_string()], vec!["refined".to_string()]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
    );
    infos.insert(
        "fuse".to_string(),
        NodeInfo::with_meta(vec![meta("a", true), meta("ctx", false)], vec![])
            .with_policy(MacroPolicy::Sync { window_ms: 50 }),
    );

    let edges = build_trigger_edges(&config, &infos);
    // Edge classification: the trigger input IS a DAG edge; the non-trigger
    // input is NOT (earlier both were).
    assert!(
        edges.is_triggering("fuse", "/syncl/src/raw"),
        "fuse's #[input(trigger)] `a` must remain a triggering DAG edge"
    );
    assert!(
        !edges.is_triggering("fuse", "/syncl/mid/refined"),
        "fuse's plain #[input] `ctx` must NOT be a triggering DAG edge \
         (pre-flip Sync classified every input as triggering)"
    );
    assert!(
        edges.is_triggering("mid", "/syncl/src/raw"),
        "mid's DataTrigger edge is untouched by the Sync trigger-scope flip"
    );

    // Level assignment: fuse derives from its trigger input only.
    let topology = GraphTopology::build(&config, &infos).expect("topology builds");
    let levels = topology.derive_levels(&edges).expect("acyclic");
    assert_eq!(
        levels.len(),
        2,
        "HAND ORACLE: [[src], [mid, fuse]] — 2 levels (pre-flip: 3)"
    );
    assert_eq!(levels.level_of("src"), Some(0), "src is the Period root");
    assert_eq!(levels.level_of("mid"), Some(1), "mid triggers off src");
    assert_eq!(
        levels.level_of("fuse"),
        Some(1),
        "the trigger-scope flip: fuse's level derives from its TRIGGER input (src/raw) \
         only — the non-trigger ctx <- mid/refined read no longer forces it below \
         mid (pre-flip fuse sat at level 2)"
    );
}

// ───────────────────────────────────────────────────────────────────────────
// `ros2:` graph entries — the mixed-stack shape. Parse + validate oracles
// (config-only; nothing here touches a transport). Every rejection is pinned
// by the substring that names the FIX, so a regression to a generic message
// fails loudly.
// ───────────────────────────────────────────────────────────────────────────

const ROS2_MIXED_GRAPH: &str = r#"
prefix: robot
nodes:
  - id: detector
    type: yolo_node
    outputs:
      - name: boxes
        schema: test/Data
        max_slice_len: 64
  - id: move_group
    ros2:
      package: moveit_ros_move_group
      executable: move_group
      args: ["--log-level", "warn"]
      params_file: config/move_group.yaml
      params:
        rate: 10
        name: go2
        fast: true
  - id: bringup
    ros2:
      launch: launch/robot.launch.py
      args: ["use_rviz:=false"]
"#;

fn ros2_reject(yaml: &str) -> String {
    let config = parse_graph(yaml).expect("shape must PARSE — the rejection is validation's");
    let err = validate_graph(&config).expect_err("must be rejected");
    format!("{err}")
}

/// ACCEPT: a native node beside a `ros2 run`-form and a `ros2 launch`-form
/// entry parses, validates, and splits cleanly — `take_ros2_nodes` hands back
/// the two ROS 2 entries in declaration order and leaves a plain native graph.
#[test]
fn ros2_entries_parse_validate_and_split_from_the_native_graph() {
    let mut config = parse_graph(ROS2_MIXED_GRAPH).expect("mixed graph parses");
    validate_graph(&config).expect("mixed graph validates");
    assert!(config.has_ros2_nodes());
    assert_eq!(
        config
            .ros2_nodes()
            .map(|n| n.id.as_str())
            .collect::<Vec<_>>(),
        ["move_group", "bringup"]
    );
    assert_eq!(
        config
            .native_nodes()
            .map(|n| n.id.as_str())
            .collect::<Vec<_>>(),
        ["detector"]
    );

    let ros2 = config.take_ros2_nodes();
    assert_eq!(
        ros2.iter().map(|n| n.id.as_str()).collect::<Vec<_>>(),
        ["move_group", "bringup"]
    );
    assert!(!config.has_ros2_nodes());
    assert_eq!(config.nodes.len(), 1);
    assert_eq!(config.nodes[0].id, "detector");
    validate_graph(&config).expect("the native remainder is a valid graph on its own");
}

/// The argv each shape spawns with, against HAND oracles: the run form
/// renders `run <pkg> <exe> [args] --ros-args --params-file <f> -p k:=v ...`
/// in declaration order (numbers/bools canonical); the launch form renders
/// `launch <resolved file> [args]`; a run form with no params carries NO
/// `--ros-args`.
#[test]
fn ros2_argv_renders_run_and_launch_forms_to_the_hand_oracle() {
    let config = parse_graph(ROS2_MIXED_GRAPH).expect("parses");
    let move_group = config.nodes[1].ros2.as_ref().expect("ros2 block");
    assert_eq!(
        move_group.argv(None),
        [
            "run",
            "moveit_ros_move_group",
            "move_group",
            "--log-level",
            "warn",
            "--ros-args",
            "--params-file",
            "config/move_group.yaml",
            "-p",
            "rate:=10",
            "-p",
            "name:=go2",
            "-p",
            "fast:=true",
        ]
    );
    let bringup = config.nodes[2].ros2.as_ref().expect("ros2 block");
    assert_eq!(
        bringup.argv(Some("/ws/launch/robot.launch.py")),
        ["launch", "/ws/launch/robot.launch.py", "use_rviz:=false"]
    );
    assert_eq!(
        bringup.argv(None),
        ["launch", "launch/robot.launch.py", "use_rviz:=false"],
        "no resolved path ⇒ the declared one, verbatim"
    );

    let bare = parse_graph(
        r#"
prefix: p
nodes:
  - id: a
    type: a
  - id: talker
    ros2:
      package: demo_nodes_cpp
      executable: talker
"#,
    )
    .expect("parses");
    assert_eq!(
        bare.nodes[1].ros2.as_ref().expect("ros2").argv(None),
        ["run", "demo_nodes_cpp", "talker"],
        "no params ⇒ no --ros-args"
    );
}

/// A `ros2:` entry round-trips through serialize: no `type:` key is emitted
/// for it, the block survives, and the native sibling keeps its `type:`.
#[test]
fn ros2_entries_round_trip_through_serialize() {
    let config = parse_graph(ROS2_MIXED_GRAPH).expect("parses");
    let yaml = serde_yaml::to_string(&config).expect("serializes");
    assert!(
        !yaml.contains("type: ''"),
        "no empty type is emitted: {yaml}"
    );
    let again = parse_graph(&yaml).expect("re-parses");
    validate_graph(&again).expect("re-validates");
    assert_eq!(again.nodes.len(), 3);
    assert_eq!(again.nodes[0].node_type, "yolo_node");
    assert!(again.nodes[1].is_ros2());
    assert_eq!(
        again.nodes[1].ros2.as_ref().unwrap().argv(None),
        config.nodes[1].ros2.as_ref().unwrap().argv(None)
    );
}

/// A misspelled key INSIDE the `ros2:` block is a loud parse error
/// (`deny_unknown_fields`), never a silently dropped setting.
#[test]
fn ros2_block_rejects_unknown_keys_at_parse() {
    let err = parse_graph(
        r#"
prefix: p
nodes:
  - id: a
    type: a
  - id: mg
    ros2:
      package: x
      executable: y
      parms: {a: 1}
"#,
    )
    .expect_err("unknown key must fail to parse");
    assert!(format!("{err}").contains("parms"), "got: {err}");
}

/// REJECT matrix: every malformed shape names the entry and the fix.
#[test]
fn ros2_entry_shape_violations_are_rejected_by_name() {
    let native = "  - id: a\n    type: a\n";
    let cases: [(&str, &str); 9] = [
        // both type and ros2
        (
            "  - id: mg\n    type: t\n    ros2:\n      package: x\n      executable: y\n",
            "declares BOTH `type:` and `ros2:`",
        ),
        // ports on a ros2 entry
        (
            "  - id: mg\n    ros2:\n      package: x\n      executable: y\n    outputs:\n      - name: o\n        schema: test/Data\n",
            "cannot declare `inputs:` / `outputs:`",
        ),
        (
            "  - id: mg\n    ros2:\n      package: x\n      executable: y\n    inputs:\n      - name: i\n        source: a/o\n",
            "cannot declare `inputs:` / `outputs:`",
        ),
        // launch + package
        (
            "  - id: mg\n    ros2:\n      launch: f.launch.py\n      package: x\n",
            "use ONE shape",
        ),
        // package without executable
        (
            "  - id: mg\n    ros2:\n      package: x\n",
            "needs BOTH `package:` and `executable:`",
        ),
        // empty block
        ("  - id: mg\n    ros2: {}\n", "declares nothing to run"),
        // launch + params
        (
            "  - id: mg\n    ros2:\n      launch: f.launch.py\n      params:\n        a: 1\n",
            "cannot carry `params:` / `params_file:`",
        ),
        // non-scalar param
        (
            "  - id: mg\n    ros2:\n      package: x\n      executable: y\n      params:\n        tree:\n          leaf: 1\n",
            "param 'tree' is not a scalar",
        ),
        // native entry with no type at all
        ("  - id: bare\n", "missing `type:`"),
    ];
    for (entry, expected) in cases {
        let yaml = format!("prefix: p\nnodes:\n{native}{entry}");
        let msg = ros2_reject(&yaml);
        assert!(
            msg.contains(expected),
            "entry:\n{entry}\nexpected `{expected}`, got: {msg}"
        );
    }
}

/// A graph of ONLY `ros2:` entries is refused and points at the right tool.
#[test]
fn ros2_only_graph_is_refused_and_names_ros2_run() {
    let msg = ros2_reject(
        r#"
prefix: p
nodes:
  - id: mg
    ros2:
      launch: f.launch.py
"#,
    );
    assert!(msg.contains("cerulion ros2 run"), "got: {msg}");
}

/// `process_groups:` — a ros2 entry is REFUSED in a group by name, and a
/// partition covering only the native nodes is COMPLETE (the orphan check
/// exempts ros2 entries).
#[test]
fn ros2_entries_are_refused_in_process_groups_and_exempt_from_coverage() {
    let base = r#"
prefix: p
nodes:
  - id: a
    type: a
  - id: mg
    ros2:
      package: x
      executable: y
"#;
    let msg = ros2_reject(&format!("{base}process_groups:\n  g0: [a, mg]\n"));
    assert!(msg.contains("is a `ros2:` entry"), "got: {msg}");
    assert!(msg.contains("cannot join a process group"), "got: {msg}");

    let ok = parse_graph(&format!("{base}process_groups:\n  g0: [a]\n")).expect("parses");
    validate_graph(&ok).expect("native-only coverage is complete");
}

/// `level_assignments:` — a ros2 entry is REFUSED in the block by name, and a
/// block covering only the native nodes is COMPLETE.
#[test]
fn ros2_entries_are_refused_in_level_assignments_and_exempt_from_coverage() {
    let base = r#"
prefix: p
nodes:
  - id: a
    type: a
  - id: mg
    ros2:
      launch: f.launch.py
"#;
    let msg = ros2_reject(&format!("{base}level_assignments:\n  a: 0\n  mg: 1\n"));
    assert!(msg.contains("is a `ros2:` entry"), "got: {msg}");
    assert!(msg.contains("not scheduled into a DAG level"), "got: {msg}");

    let ok = parse_graph(&format!("{base}level_assignments:\n  a: 0\n")).expect("parses");
    validate_graph(&ok).expect("native-only coverage is complete");
}
