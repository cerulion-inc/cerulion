// SPDX-License-Identifier: AGPL-3.0-only
//! A long-running embedded graph keeps a bounded execution trace.
//!
//! The scheduler adds one trace entry per node fire. `graph run` caps that
//! trace at its default `--trace-limit`, and the embedded runtime must cap
//! it at the same value, so an app graph that runs for days keeps flat
//! memory. A linked relay node reads an app topic on a trigger input (one
//! fire per frame) and writes its fire count into each echo, so the echoes
//! count the fires independently of the trace. The node registry and the
//! transport are process state, so this file is its own binary and holds
//! ONE test.

use std::sync::mpsc;
use std::time::{Duration, Instant};

use cerulion_android::{EmbeddedRuntime, RuntimeState};
use cerulion_cli_engine::graph_cmd::PRODUCTION_TRACE_LIMIT;
use cerulion_core::real_ns;
use cerulion_core::transport::TransportManager;
use serde_json::json;
use tempfile::tempdir;

cerulion_android::embed_nodes! {
    "test_node_macro_relay_rlib" => test_node_macro_relay_rlib::RelayNodeEntry,
}

const JNI_VERSION_1_6: i32 = 0x0001_0006;
const VECTOR3: &str = "geometry_msgs/Vector3";
/// Fires past the cap: the oldest this many entries must be gone.
const FIRES_PAST_CAP: usize = 500;
/// Frames published before the test waits for their echoes.
const BURST: usize = 4;
/// The fire after which the test reads the clock. A fire's time is read when
/// its step begins, so the cut sits this many bursts before the first kept
/// fire: a step that began before the cut has long ended when that fire runs.
const CUT_AT_FIRE: usize = FIRES_PAST_CAP - 25 * BURST;

/// What the trace kept at the end of the run.
struct KeptTrace {
    len: usize,
    nodes_other_than_logic: usize,
    oldest_fire_ns: u64,
}

#[test]
fn a_long_run_keeps_the_newest_trace_entries_up_to_the_graph_run_cap() {
    assert_eq!(
        JNI_OnLoad(std::ptr::null_mut(), std::ptr::null_mut()),
        JNI_VERSION_1_6
    );
    let temp = tempdir().unwrap();
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), "android_trace_cap".into())
        .expect("runtime");
    let pid = std::process::id();
    let app_topic = format!("/trace_cap_{pid}/cmd");
    let prefix = format!("trace_cap_graph_{pid}");
    let echo_topic = format!("/{prefix}/logic/echo");
    let graph = format!(
        "prefix: {prefix}
nodes:
  - id: logic
    type: test_node_macro_relay_rlib
    inputs:
      - name: cmd
        source: {app_topic}
    outputs:
      - name: echo
        schema: {VECTOR3}
"
    );
    let (kept_tx, kept_rx) = mpsc::channel();
    runtime.set_graph_exit_hook_for_test(move |trace| {
        let kept = KeptTrace {
            len: trace.len(),
            nodes_other_than_logic: trace.iter().filter(|e| &*e.node_id != "logic").count(),
            oldest_fire_ns: trace.first().map_or(0, |e| e.fire_time_ns),
        };
        kept_tx.send(kept).expect("send the kept trace");
    });
    runtime
        .start(
            &json!({
                "graph_yaml": graph,
                "publish": [{"topic": app_topic, "schema": VECTOR3}],
            })
            .to_string(),
        )
        .expect("a graph with a linked relay node starts");
    let status = runtime.status_handle().lock().unwrap().clone();
    assert_eq!(status.state, RuntimeState::Running, "{status:?}");

    let subscriber = TransportManager::get()
        .expect("transport manager")
        .create_subscriber(&echo_topic)
        .expect("subscribe to the relay output");
    let total = PRODUCTION_TRACE_LIMIT + FIRES_PAST_CAP;
    let mut published = 0usize;
    let mut fires = 0usize;
    // Taken once fire `CUT_AT_FIRE` has echoed and before any later frame is
    // published: every kept fire is later.
    let mut cut_ns = None;
    let deadline = Instant::now() + Duration::from_secs(120);
    while fires < total {
        let burst = BURST.min(total - published);
        for _ in 0..burst {
            published += 1;
            runtime
                .publish(
                    &app_topic,
                    &json!({"x": published as f64, "y": 0.0, "z": 0.0}).to_string(),
                )
                .unwrap_or_else(|error| panic!("publish frame {published}: {error}"));
        }
        while fires < published {
            assert!(
                Instant::now() < deadline,
                "{fires} of {published} published frames echoed before the deadline"
            );
            subscriber
                .wait_for_message(Duration::from_millis(200), |message| {
                    let payload = message.payload();
                    let x = f64::from_le_bytes(payload[0..8].try_into().expect("x bytes"));
                    let z = f64::from_le_bytes(payload[16..24].try_into().expect("z bytes"));
                    fires += 1;
                    // One fire per frame, in publish order.
                    assert_eq!((x, z), (fires as f64, fires as f64));
                })
                .expect("wait for relay output");
        }
        if fires == CUT_AT_FIRE {
            cut_ns = Some(real_ns());
        }
    }
    runtime.stop().expect("stop the graph");
    let end_ns = real_ns();

    let kept = kept_rx
        .recv_timeout(Duration::from_secs(10))
        .expect("the graph thread reports its trace on exit");
    assert_eq!(
        kept.len, PRODUCTION_TRACE_LIMIT,
        "{total} fires must leave exactly the graph run cap in the trace"
    );
    assert_eq!(kept.nodes_other_than_logic, 0);
    let cut_ns = cut_ns.expect("the cut was taken");
    // Both bounds, so a fire time on another clock than `real_ns` fails.
    assert!(
        cut_ns < kept.oldest_fire_ns && kept.oldest_fire_ns <= end_ns,
        "the trace must keep the newest fires: oldest kept {} not in ({cut_ns}, {end_ns}]",
        kept.oldest_fire_ns
    );
}
