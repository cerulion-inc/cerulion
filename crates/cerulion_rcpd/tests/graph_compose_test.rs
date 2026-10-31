// SPDX-License-Identifier: AGPL-3.0-only
//! The pure `robot://graph` composer against hand-written oracles.

use cerulion_core::transport::cerulion_q::{CatalogEntry, CatalogProvenance};
use cerulion_core::transport::mirror_registry::GatherCompleteness;
use cerulion_core::transport::run_registry::{RunGather, RunRecord, RunState};
use cerulion_rcpd::graph::{
    compose, GraphInputs, NodeView, RunStateView, RunView, RunsCompleteness, TopicObservation,
};
use cerulion_rcpd::manifest::{DeclaredNode, RobotIdentity};

fn identity() -> RobotIdentity {
    RobotIdentity {
        robot: "bot".into(),
        nodes: vec![DeclaredNode {
            node_id: "cam".into(),
            node_type: "camera".into(),
        }],
    }
}

fn record(run_id: u128, state: RunState) -> RunRecord {
    RunRecord {
        run_id,
        supervisor_pid: 42,
        run_started_at_ns: 9,
        state,
        graph_name: "g".into(),
        run_dir: "runs/x".into(),
    }
}

fn inputs() -> GraphInputs {
    GraphInputs {
        now_ns: 123,
        runs: Ok(RunGather {
            records: vec![record(2, RunState::Ending), record(1, RunState::Live)],
            completeness: GatherCompleteness::Settled,
        }),
        topics: Ok(vec![
            TopicObservation {
                topic: "/b".into(),
                producer_count: Some(1),
                liveness: None,
            },
            TopicObservation {
                topic: "/a".into(),
                producer_count: None,
                liveness: None,
            },
        ]),
        monitors: Vec::new(),
    }
}

#[test]
fn the_document_carries_runs_nodes_and_a_sorted_catalog() {
    let g = compose(&identity(), inputs());
    assert_eq!(g.version, 1);
    assert_eq!(g.robot, "bot");
    assert_eq!(g.refreshed_at_ns, 123);
    assert_eq!(g.runs.completeness, RunsCompleteness::Settled);
    assert_eq!(
        g.runs.records,
        vec![
            RunView {
                run_id: "0x00000000000000000000000000000001".into(),
                graph_name: "g".into(),
                state: RunStateView::Live,
                supervisor_pid: 42,
                run_started_at_ns: 9,
            },
            RunView {
                run_id: "0x00000000000000000000000000000002".into(),
                graph_name: "g".into(),
                state: RunStateView::Ending,
                supervisor_pid: 42,
                run_started_at_ns: 9,
            },
        ]
    );
    assert_eq!(
        g.nodes,
        vec![NodeView {
            node_id: "cam".into(),
            node_type: "camera".into(),
            counters: None,
        }]
    );
    assert_eq!(g.catalog.robot, "bot");
    assert_eq!(g.catalog.error, None);
    assert_eq!(g.topics_error, None);
    assert_eq!(
        g.catalog.entries,
        vec![
            CatalogEntry {
                topic: "/a".into(),
                schema_hash: None,
                schema_name: None,
                provenance: CatalogProvenance::Runtime,
                producer_count: None,
                liveness: None,
            },
            CatalogEntry {
                topic: "/b".into(),
                schema_hash: None,
                schema_name: None,
                provenance: CatalogProvenance::Runtime,
                producer_count: Some(1),
                liveness: None,
            },
        ]
    );
}

#[test]
fn node_counters_serialize_as_null() {
    let json = serde_json::to_value(compose(&identity(), inputs())).expect("json");
    assert_eq!(
        json["nodes"],
        serde_json::json!([{ "node_id": "cam", "node_type": "camera", "counters": null }])
    );
    assert_eq!(
        json["runs"]["completeness"],
        serde_json::json!({ "kind": "settled" })
    );
    assert!(
        json["runs"]["records"][0].get("run_dir").is_none(),
        "no local paths"
    );
}

#[test]
fn two_compositions_are_byte_identical() {
    let a = serde_json::to_string(&compose(&identity(), inputs())).expect("json");
    let b = serde_json::to_string(&compose(&identity(), inputs())).expect("json");
    assert_eq!(a, b);
}

#[test]
fn an_incomplete_gather_is_reported_not_hidden() {
    let mut i = inputs();
    i.runs = Ok(RunGather {
        records: Vec::new(),
        completeness: GatherCompleteness::Incomplete {
            live_writers: 3,
            writers_heard: 1,
        },
    });
    assert_eq!(
        compose(&identity(), i).runs.completeness,
        RunsCompleteness::Incomplete {
            live_writers: 3,
            writers_heard: 1,
        }
    );
}

#[test]
fn a_failed_gather_is_unavailable_with_its_reason() {
    let mut i = inputs();
    i.runs = Err("registry closed".into());
    let g = compose(&identity(), i);
    assert_eq!(
        g.runs.completeness,
        RunsCompleteness::Unavailable {
            error: "registry closed".into(),
        }
    );
    assert!(g.runs.records.is_empty());
}

#[test]
fn a_failed_topic_listing_is_a_topics_error_not_a_catalog_refusal() {
    let mut i = inputs();
    i.topics = Err("listing failed".into());
    let g = compose(&identity(), i);
    assert_eq!(g.topics_error.as_deref(), Some("listing failed"));
    assert_eq!(
        g.catalog.error, None,
        "catalog.error means an authorization refusal"
    );
    assert!(g.catalog.entries.is_empty());
}
