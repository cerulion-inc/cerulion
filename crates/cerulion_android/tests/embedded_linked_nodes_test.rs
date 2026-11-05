// SPDX-License-Identifier: AGPL-3.0-only
//! Graph nodes linked into the library run from the node registry.
//!
//! This binary declares its nodes with `cerulion_android::embed_nodes!`, the
//! same line an app's runtime library uses, and calls the `JNI_OnLoad` the
//! macro makes. It needs no prebuilt node library. It runs under both
//! transports: the default cross-process one, and the in-process one an app
//! uses (`--features local-transport`). The registry is process state, so
//! this file is its own binary and holds ONE test that runs its steps in
//! order.

use std::path::Path;
use std::time::{Duration, Instant};

use cerulion_android::{codes, EmbeddedRuntime, RuntimeState};
use cerulion_bag::BagReader;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::WireHeader;
use serde_json::json;
use tempfile::tempdir;

cerulion_android::embed_nodes! {
    "test_node_macro_period_cdylib" => test_node_macro_period_cdylib::PeriodNodeEntry,
    "test_node_macro_relay_rlib" => test_node_macro_relay_rlib::RelayNodeEntry,
}

const JNI_VERSION_1_6: i32 = 0x0001_0006;
const VECTOR3: &str = "geometry_msgs/Vector3";

fn graph_yaml(prefix: &str, source_type: &str) -> String {
    graph_yaml_with_relay(prefix, source_type, "test_node_macro_relay_rlib")
}

fn graph_yaml_with_relay(prefix: &str, source_type: &str, relay_type: &str) -> String {
    format!(
        "prefix: {prefix}
nodes:
  - id: source
    type: {source_type}
    outputs:
      - name: cmd
        schema: {VECTOR3}
  - id: relay
    type: {relay_type}
    inputs:
      - name: cmd
        source: source/cmd
    outputs:
      - name: echo
        schema: {VECTOR3}
"
    )
}

fn config(prefix: &str, file: &str) -> String {
    json!({
        "graph_yaml": graph_yaml(prefix, "test_node_macro_period_cdylib"),
        "record": {"file": file},
    })
    .to_string()
}

/// `(x, z)` of each Vector3 frame the relay sent: `x` is copied from the
/// input frame, `z` counts the relay's ticks.
fn relay_frames_live(topic: &str, wanted: usize) -> Vec<(f64, f64)> {
    let manager = TransportManager::get().expect("transport manager");
    let subscriber = manager
        .create_subscriber(topic)
        .expect("subscribe to the relay output");
    let mut frames = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(10);
    while frames.len() < wanted && Instant::now() < deadline {
        subscriber
            .wait_for_message(Duration::from_millis(500), |message| {
                let payload = message.payload();
                let x = f64::from_le_bytes(payload[0..8].try_into().expect("x bytes"));
                let z = f64::from_le_bytes(payload[16..24].try_into().expect("z bytes"));
                frames.push((x, z));
            })
            .expect("wait for relay output");
    }
    frames
}

/// The relay's tick count (`z`) of each frame recorded on `topic`, in order.
fn bag_relay_ticks(path: &Path, topic: &str) -> Vec<f64> {
    let reader = BagReader::open(path).expect("open finalized bag");
    reader
        .user_message_index()
        .expect("index messages")
        .get(topic)
        .map(|spans| {
            spans
                .iter()
                .map(|span| {
                    let frame = reader.frame(span);
                    let payload = &frame[WireHeader::SIZE..];
                    f64::from_le_bytes(payload[16..24].try_into().expect("z bytes"))
                })
                .collect()
        })
        .unwrap_or_default()
}

fn bag_frame_count(path: &Path, topic: &str) -> usize {
    let reader = BagReader::open(path).expect("open finalized bag");
    let channel = reader
        .channels()
        .expect("read channels")
        .into_iter()
        .find(|channel| channel.topic == topic)
        .unwrap_or_else(|| panic!("bag has no channel {topic}"));
    assert_eq!(channel.schema_name, VECTOR3);
    reader
        .user_message_index()
        .expect("index messages")
        .get(topic)
        .map_or(0, |spans| {
            for span in spans {
                assert!(reader.frame(span).len() >= WireHeader::SIZE + 24);
            }
            spans.len()
        })
}

fn assert_relay_counts_up(frames: &[(f64, f64)]) {
    assert!(
        frames.len() >= 3,
        "expected at least 3 relay frames, got {frames:?}"
    );
    for pair in frames.windows(2) {
        assert!(
            pair[1].1 > pair[0].1,
            "relay tick count must rise strictly: {frames:?}"
        );
    }
    // The period node writes x = 0.0 into every frame; the relay copies it.
    assert!(frames.iter().all(|(x, z)| *x == 0.0 && *z >= 1.0));
}

fn run_once(runtime: &mut EmbeddedRuntime, files_dir: &Path, prefix: &str, file: &str) {
    let relay_topic = format!("/{prefix}/relay/echo");
    let source_topic = format!("/{prefix}/source/cmd");
    runtime
        .start(&config(prefix, file))
        .expect("start the linked-node graph");
    let status = runtime.status_handle().lock().unwrap().clone();
    assert_eq!(status.state, RuntimeState::Running);
    assert_eq!(status.nodes, 2);
    let frames = relay_frames_live(&relay_topic, 3);
    assert_relay_counts_up(&frames);
    let summary = runtime.stop().expect("stop").expect("recorder summary");
    assert!(summary.per_topic.get(&relay_topic).copied().unwrap_or(0) > 0);
    let bag = files_dir.join("bags").join(file);
    assert!(bag_frame_count(&bag, &source_topic) > 0);
    assert!(bag_frame_count(&bag, &relay_topic) > 0);
    // Each start makes a fresh relay from the registry, so the recorded
    // tick count starts again at 1 (a reused instance would carry on).
    let ticks = bag_relay_ticks(&bag, &relay_topic);
    assert_eq!(
        ticks.first().copied(),
        Some(1.0),
        "relay ticks in {file}: {ticks:?}"
    );
}

/// The relay declares the output `echo`. A graph that lists the relay's
/// input and no `outputs:` cannot publish it, so the start fails with the
/// graph code, the node and the output named, and the status shows the same
/// error; the graph never reports running.
fn an_output_the_graph_does_not_list_fails_the_start(runtime: &mut EmbeddedRuntime, prefix: &str) {
    let unlisted_prefix = format!("{prefix}_unlisted");
    let graph = format!(
        "prefix: {unlisted_prefix}
nodes:
  - id: source
    type: test_node_macro_period_cdylib
    outputs:
      - name: cmd
        schema: {VECTOR3}
  - id: relay
    type: test_node_macro_relay_rlib
    inputs:
      - name: cmd
        source: source/cmd
"
    );
    let error = runtime
        .start(&json!({"graph_yaml": graph}).to_string())
        .expect_err("a node output the graph does not list must fail the start");
    assert_eq!(error.code(), codes::GRAPH);
    let message = error.to_string();
    assert_eq!(
        message,
        "node 'relay' output 'echo': the node declares this output and the graph does not list \
         it, so the node has no publisher for it; add `- name: echo` with its `schema:` under \
         the node's `outputs:`"
    );
    let status = runtime.status_handle().lock().unwrap().clone();
    assert_eq!(status.state, RuntimeState::Failed, "{status:?}");
    let last_error = status
        .last_error
        .as_ref()
        .expect("a failed start records its error");
    assert_eq!(last_error.code, codes::GRAPH, "{status:?}");
    assert_eq!(last_error.message, message, "{status:?}");
}

#[test]
fn linked_nodes_run_record_restart_and_refuse_unlinked_types() {
    assert_eq!(
        JNI_OnLoad(std::ptr::null_mut(), std::ptr::null_mut()),
        JNI_VERSION_1_6
    );
    assert_eq!(
        cerulion_android::nodes::linked_node_types(),
        [
            "test_node_macro_period_cdylib",
            "test_node_macro_relay_rlib"
        ]
    );

    let temp = tempdir().unwrap();
    let files_dir = temp.path();
    let prefix = format!("linked_{}", std::process::id());
    let mut runtime =
        EmbeddedRuntime::new(files_dir.to_path_buf(), "android_linked".into()).expect("runtime");
    assert_eq!(
        runtime.status_handle().lock().unwrap().linked_nodes,
        [
            "test_node_macro_period_cdylib",
            "test_node_macro_relay_rlib"
        ]
    );
    let status_json = serde_json::to_value(&*runtime.status_handle().lock().unwrap()).unwrap();
    assert_eq!(
        status_json["linked_nodes"],
        json!([
            "test_node_macro_period_cdylib",
            "test_node_macro_relay_rlib"
        ])
    );

    // Run, stop, and run again in the same process: the second start makes
    // its nodes again from the registry, and frames must flow and be
    // recorded again.
    run_once(&mut runtime, files_dir, &prefix, "first.mcap");
    run_once(&mut runtime, files_dir, &prefix, "second.mcap");

    // A type that is not linked is refused with the graph code, and the
    // message names the type and the linked list.
    let unlinked = json!({"graph_yaml": graph_yaml(&prefix, "timer")}).to_string();
    let error = runtime
        .start(&unlinked)
        .expect_err("an unlinked node type must be refused");
    assert_eq!(error.code(), codes::GRAPH);
    assert_eq!(
        error.to_string(),
        "node type 'timer' is not linked into this app library (linked: test_node_macro_period_cdylib, test_node_macro_relay_rlib); add the node crate to the app runtime crate and list it in cerulion_android::embed_nodes!"
    );

    // A type registered after the runtime was made is listed by the next
    // start, and its factory may read the registry while it builds the node.
    cerulion_android::nodes::register(&[cerulion_android::nodes::LinkedNode {
        node_type: "late_relay",
        factory: || {
            assert!(
                cerulion_android::nodes::linked_node_types().contains(&"late_relay".to_string())
            );
            Box::new(test_node_macro_relay_rlib::RelayNodeEntry::new())
        },
    }])
    .expect("register a type after the runtime was made");
    let late = json!({
        "graph_yaml": graph_yaml_with_relay(&prefix, "test_node_macro_period_cdylib", "late_relay"),
    })
    .to_string();
    runtime
        .start(&late)
        .expect("start a graph with the late type");
    let status = runtime.status_handle().lock().unwrap().clone();
    assert_eq!(status.state, RuntimeState::Running);
    assert_eq!(status.nodes, 2);
    assert_eq!(
        status.linked_nodes,
        [
            "late_relay",
            "test_node_macro_period_cdylib",
            "test_node_macro_relay_rlib"
        ]
    );
    assert_relay_counts_up(&relay_frames_live(&format!("/{prefix}/relay/echo"), 3));
    runtime.stop().expect("stop the late-type graph");

    an_output_the_graph_does_not_list_fails_the_start(&mut runtime, &prefix);

    // Under the in-process transport a graph with native nodes refuses
    // node_lib_dir: nodes cannot be loaded from separate libraries there.
    #[cfg(feature = "local-transport")]
    {
        let with_dir = json!({
            "graph_yaml": graph_yaml(&prefix, "test_node_macro_period_cdylib"),
            "node_lib_dir": files_dir,
        })
        .to_string();
        let error = runtime
            .start(&with_dir)
            .expect_err("node_lib_dir must be refused in-process");
        assert_eq!(error.code(), codes::GRAPH);
        assert_eq!(
            error.to_string(),
            "node_lib_dir is not supported by the in-process app runtime; link the nodes into the app library with cerulion_android::embed_nodes!"
        );
    }

    // A second registration of a linked type is refused, and from then on
    // every start with a graph reports it.
    let duplicate = cerulion_android::nodes::register(&[cerulion_android::nodes::LinkedNode {
        node_type: "test_node_macro_relay_rlib",
        factory: || Box::new(test_node_macro_relay_rlib::RelayNodeEntry::new()),
    }])
    .expect_err("a duplicate node type must be refused");
    assert_eq!(duplicate.code(), codes::GRAPH);
    let error = runtime
        .start(&config(&prefix, "third.mcap"))
        .expect_err("a start with a graph must report the duplicate");
    assert_eq!(error.code(), codes::GRAPH);
    assert_eq!(error.to_string(), duplicate.to_string());
}
