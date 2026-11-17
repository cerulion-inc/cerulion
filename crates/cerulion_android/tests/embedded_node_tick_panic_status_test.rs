// SPDX-License-Identifier: AGPL-3.0-only
//! Status reports a linked node whose tick panicked.
//!
//! The graph runtime contains a tick panic: the node stops, the rest of the
//! graph keeps running, and status keeps reporting `running`. `node_failures`
//! names each such node with its first panic message, read while the graph
//! thread runs. Two linked nodes panic on their third tick; a third node
//! stays healthy and keeps publishing. It runs under both transports: the
//! default cross-process one, and the in-process one an app uses
//! (`--features local-transport`). The node registry and the transport are
//! process state, so this file is its own binary and holds ONE test that
//! runs its steps in order.

use std::time::{Duration, Instant};

use cerulion_android::{codes, EmbeddedRuntime, NodeFailure, RuntimeState};
use cerulion_core::transport::TransportManager;
use serde_json::json;
use tempfile::tempdir;

cerulion_android::embed_nodes! {
    "test_node_macro_period_cdylib" => test_node_macro_period_cdylib::PeriodNodeEntry,
    "test_node_macro_period_panic_cdylib" => test_node_macro_period_panic_cdylib::PeriodNodeEntry,
}

const JNI_VERSION_1_6: i32 = 0x0001_0006;
const VECTOR3: &str = "geometry_msgs/Vector3";
/// The panic fixture's message for its third tick (its source, by hand).
const PANIC_MESSAGE: &str = "injected candidate crash at tick 3 (exit-3 twin)";

fn period_node(id: &str, node_type: &str) -> String {
    format!(
        "  - id: {id}
    type: {node_type}
    outputs:
      - name: cmd
        schema: {VECTOR3}
"
    )
}

/// Two panicking nodes (`zeta`, `alpha`, in that graph order, so the list
/// order is neither name nor panic order by chance) and one healthy node
/// (`steady`).
fn panic_graph(prefix: &str) -> String {
    format!(
        "prefix: {prefix}\nnodes:\n{}{}{}",
        period_node("zeta", "test_node_macro_period_panic_cdylib"),
        period_node("alpha", "test_node_macro_period_panic_cdylib"),
        period_node("steady", "test_node_macro_period_cdylib"),
    )
}

fn healthy_graph(prefix: &str) -> String {
    format!(
        "prefix: {prefix}\nnodes:\n{}",
        period_node("steady", "test_node_macro_period_cdylib")
    )
}

fn start(runtime: &mut EmbeddedRuntime, graph_yaml: &str) {
    runtime
        .start(&json!({"graph_yaml": graph_yaml}).to_string())
        .expect("start the graph");
}

fn node_failures(runtime: &EmbeddedRuntime) -> Vec<NodeFailure> {
    runtime.status_handle().lock().unwrap().node_failures.list()
}

fn wait_for_failures(runtime: &EmbeddedRuntime, wanted: usize) -> Vec<NodeFailure> {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let failures = node_failures(runtime);
        if failures.len() >= wanted || Instant::now() >= deadline {
            return failures;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// The hand-written oracle: both panicking nodes, in graph order.
fn expected_failures() -> Vec<NodeFailure> {
    ["zeta", "alpha"]
        .iter()
        .map(|node_id| NodeFailure {
            node_id: (*node_id).to_string(),
            message: PANIC_MESSAGE.to_string(),
        })
        .collect()
}

/// Frames delivered on `topic` from now on, up to `wanted` or 10 s.
fn frames_delivered(topic: &str, wanted: usize) -> usize {
    let manager = TransportManager::get().expect("transport manager");
    let subscriber = manager
        .create_subscriber(topic)
        .expect("subscribe to the healthy node's output");
    let mut frames = 0;
    let deadline = Instant::now() + Duration::from_secs(10);
    while frames < wanted && Instant::now() < deadline {
        subscriber
            .wait_for_message(Duration::from_millis(500), |_| frames += 1)
            .expect("wait for the healthy node's output");
    }
    frames
}

/// Start the panic graph, wait for both panics, and read them while the
/// graph keeps running. Returns the failures status reported.
fn panics_reported_while_running(runtime: &mut EmbeddedRuntime, prefix: &str) -> Vec<NodeFailure> {
    start(runtime, &panic_graph(prefix));
    let failures = wait_for_failures(runtime, 2);
    assert_eq!(failures, expected_failures());

    let status = runtime.status_handle().lock().unwrap().clone();
    assert_eq!(status.state, RuntimeState::Running, "{status:?}");
    assert_eq!(status.nodes, 3, "{status:?}");
    assert_eq!(status.last_error, None, "{status:?}");
    let status_json = serde_json::to_value(&*runtime.status_handle().lock().unwrap()).unwrap();
    assert_eq!(
        status_json["node_failures"],
        json!([
            {"node_id": "zeta", "message": PANIC_MESSAGE},
            {"node_id": "alpha", "message": PANIC_MESSAGE},
        ])
    );

    // The graph keeps running: the healthy node still delivers after both
    // panics, and a panicked node is reported once.
    assert!(frames_delivered(&format!("/{prefix}/steady/cmd"), 3) >= 3);
    assert_eq!(node_failures(runtime), expected_failures());
    failures
}

#[test]
fn status_reports_contained_node_tick_panics_while_the_graph_runs() {
    assert_eq!(
        JNI_OnLoad(std::ptr::null_mut(), std::ptr::null_mut()),
        JNI_VERSION_1_6
    );
    let temp = tempdir().unwrap();
    let pid = std::process::id();
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), "android_node_panics".into())
        .expect("runtime");

    // Before any start there is nothing to report, and the field is an
    // empty list in the status JSON.
    assert!(node_failures(&runtime).is_empty());
    let idle_json = serde_json::to_value(&*runtime.status_handle().lock().unwrap()).unwrap();
    assert_eq!(idle_json["node_failures"], json!([]));

    let first_run = panics_reported_while_running(&mut runtime, &format!("panic_one_{pid}"));
    runtime.stop().expect("stop the panic graph");
    // The report outlives the graph thread: still readable after stop.
    let status = runtime.status_handle().lock().unwrap().clone();
    assert_eq!(status.state, RuntimeState::Stopped, "{status:?}");
    assert_eq!(status.node_failures.list(), expected_failures());

    // A second start of the same graph reports the same panics.
    let second_run = panics_reported_while_running(&mut runtime, &format!("panic_two_{pid}"));
    assert_eq!(first_run, second_run);
    runtime.stop().expect("stop the second panic graph");

    // A start refused before its config parsed leaves the report as it was.
    let error = runtime
        .start("{not json")
        .expect_err("unparsable config must be refused");
    assert_eq!(error.code(), codes::INVALID_ARGUMENT);
    assert_eq!(node_failures(&runtime), expected_failures());

    // A start that is refused after its config parsed still clears the
    // earlier run's report: the failure is the start's, not a node's.
    let unlinked = panic_graph(&format!("refused_{pid}"))
        .replace("type: test_node_macro_period_cdylib", "type: unlinked_node");
    let error = runtime
        .start(&json!({"graph_yaml": unlinked}).to_string())
        .expect_err("an unlinked node type must be refused");
    assert_eq!(error.code(), codes::GRAPH);
    let status = runtime.status_handle().lock().unwrap().clone();
    assert_eq!(status.state, RuntimeState::Failed, "{status:?}");
    assert!(status.node_failures.list().is_empty(), "{status:?}");
    let status_json = serde_json::to_value(&*runtime.status_handle().lock().unwrap()).unwrap();
    assert_eq!(status_json["node_failures"], json!([]));

    // A healthy graph reports none.
    let prefix = format!("healthy_{pid}");
    start(&mut runtime, &healthy_graph(&prefix));
    assert!(frames_delivered(&format!("/{prefix}/steady/cmd"), 6) >= 6);
    assert!(node_failures(&runtime).is_empty());
    let status_json = serde_json::to_value(&*runtime.status_handle().lock().unwrap()).unwrap();
    assert_eq!(status_json["node_failures"], json!([]));
    runtime.stop().expect("stop the healthy graph");
}
