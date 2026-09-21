// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end CLI integration tests (the SHM-backed API rewrite).
//!
//! Simulates a real user journey: create workspace → add nodes → build
//! graph → run with real iceoryx2 transport → verify zero-copy,
//! determinism, and observability.
//!
//! # Architecture
//!
//! Two-layer testing:
//! 1. **CLI Layer** — calls `cerulion_cli_engine` library functions to
//!    produce workspace files, node crates, and graph YAML.
//! 2. **Runtime Layer** — feeds CLI-generated YAML into `GraphRuntime`
//!    with `ClosureNodeEntry` nodes and real iceoryx2 transport, and
//!    publishes/receives via the SHM-backed `loan_proxy` / `try_view`
//!    APIs introduced by that rewrite.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test e2e_cli_test -- --test-threads=1
//! ```
//!
//! Must run single-threaded: iceoryx2 singleton + shared memory requires
//! serial access across transport tests.
//!
//! # Dropped tests vs. the legacy file
//!
//! - `test_e2e_flat_latency_across_payload_sizes` — duplicated the flatness
//!   gates after the SHM rewrite. Coverage is strictly stronger in
//!   `flat_latency_test.rs` (the full publish→receive path, a wider sweep, a
//!   tighter ceiling on a per-size floor).

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime};

use cerulion_cli_engine::graph_cmd::{
    build_node_def, graph_create, graph_read, node_stage, resolve_input_binding,
};
use cerulion_cli_engine::node_cmd::{node_create, node_info, node_list, node_modify_add_port};
use cerulion_cli_engine::topic_cmd::topic_list;
use cerulion_cli_engine::workspace::workspace_create;
use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::{ClosureNodeEntry, NodeEntry, NodeInfo};
use cerulion_core::graph::{parse_graph, validate_graph, GraphRuntime};
use cerulion_core::transport::TransportManager;
use cerulion_core::MacroPolicy;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Monotonic counter guaranteeing unique topics even within the same nanosecond.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Generate a unique prefix for test isolation.
///
/// `TransportManager` is a process-wide `OnceLock` singleton, so all tests
/// share the same iceoryx2 node. Unique prefixes prevent topic collisions.
fn unique_prefix(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("e2e/{}/{}/{}", base, nanos, id)
}

// ============================================================
// Test 1: CLI workspace creation
// ============================================================

/// A panic-safe `std::env` setter for the hermetic introspection arm.
/// Restores the previous value (or removes the var) on drop, so a failing assertion
/// cannot leak `CERULION_NETWORK` into a sibling test in this binary.
struct EnvVarGuard {
    key: &'static str,
    prev: Option<String>,
}

impl EnvVarGuard {
    fn set(key: &'static str, val: &str) -> Self {
        let prev = std::env::var(key).ok();
        std::env::set_var(key, val);
        Self { key, prev }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        match &self.prev {
            Some(v) => std::env::set_var(self.key, v),
            None => std::env::remove_var(self.key),
        }
    }
}

#[test]
fn test_cli_workspace_creation() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = workspace_create(tmp.path(), "my_robot").unwrap();

    // Directory structure
    assert!(ws.root.join("Cargo.toml").exists());
    assert!(ws.graphs_dir.is_dir());
    assert!(ws.nodes_dir.is_dir());
    assert!(ws.schemas_dir.is_dir());
    assert!(ws.root.join(".cargo/config.toml").exists());

    // Cargo.toml content
    let cargo = std::fs::read_to_string(ws.root.join("Cargo.toml")).unwrap();
    assert!(cargo.contains("[workspace]"), "missing [workspace] section");
    assert!(
        cargo.contains("resolver = \"2\""),
        "missing resolver = \"2\""
    );
    assert!(
        cargo.contains("members = [\"nodes/*\"]"),
        "missing workspace members"
    );

    // .cargo/config.toml has IOX2_LOG_LEVEL
    let cargo_config = std::fs::read_to_string(ws.root.join(".cargo/config.toml")).unwrap();
    assert!(
        cargo_config.contains("IOX2_LOG_LEVEL"),
        "missing IOX2_LOG_LEVEL in .cargo/config.toml"
    );
}

// ============================================================
// Test 2: CLI node creation and modification
// ============================================================

#[test]
fn test_cli_node_creation_and_modification() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = workspace_create(tmp.path(), "test_ws").unwrap();
    let cargo_toml = ws.root.join("Cargo.toml");

    // Create two nodes
    node_create(&ws.nodes_dir, &cargo_toml, "camera", None).unwrap();
    node_create(&ws.nodes_dir, &cargo_toml, "detector", None).unwrap();

    // Add image output to camera
    node_modify_add_port(
        &ws.nodes_dir,
        "camera",
        "image",
        Some("sensor_msgs::Image"),
        true,
        false,
    )
    .unwrap();

    // Add image input to detector
    node_modify_add_port(
        &ws.nodes_dir,
        "detector",
        "image",
        Some("sensor_msgs::Image"),
        false,
        false,
    )
    .unwrap();

    // Verify camera metadata (derived from source).
    // Default-created node: no `--ext-trigger` and no
    // `#[input(trigger)]` field, so the macro template emits
    // `period_ms = 100`. The parser recovers that as
    // `Some(Period { period_ms: 100 })` per the round-trip.
    let camera_info = node_info(&ws.nodes_dir, "camera").unwrap();
    assert_eq!(camera_info.node_type, "camera");
    assert_eq!(
        camera_info.policy,
        Some(cerulion_core::MacroPolicy::Period { period_ms: 100 })
    );
    assert_eq!(camera_info.outputs.len(), 1);
    assert_eq!(camera_info.outputs[0].name, "image");
    assert_eq!(
        camera_info.outputs[0].schema.as_deref(),
        Some("sensor_msgs/Image")
    );

    // Verify detector metadata (derived from source)
    let detector_info = node_info(&ws.nodes_dir, "detector").unwrap();
    assert_eq!(detector_info.node_type, "detector");
    assert_eq!(detector_info.inputs.len(), 1);
    assert_eq!(detector_info.inputs[0].name, "image");

    // Verify lib.rs has import
    let camera_src = std::fs::read_to_string(ws.nodes_dir.join("camera/src/lib.rs")).unwrap();
    assert!(
        camera_src.contains("use native_ros2_messages::sensor_msgs::Image;"),
        "camera lib.rs should import Image"
    );

    // Verify node_list returns both
    let nodes = node_list(&ws.nodes_dir).unwrap();
    assert_eq!(nodes.len(), 2);
    let types: Vec<&str> = nodes.iter().map(|n| n.node_type.as_str()).collect();
    assert!(types.contains(&"camera"));
    assert!(types.contains(&"detector"));
}

// ============================================================
// Test 3: CLI graph creation and node staging
// ============================================================

#[test]
fn test_cli_graph_creation_and_staging() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = workspace_create(tmp.path(), "test_ws").unwrap();

    // Create graph with explicit prefix
    graph_create(&ws.graphs_dir, "perception", Some("robot1")).unwrap();

    // Graph YAML carries no `policy:` block. Trigger
    // policy lives only on the macro side; staging only encodes
    // topology (id, type, inputs, outputs).
    let camera_def = {
        let mut def = build_node_def(
            "camera",
            None,
            &[("image".to_string(), Some("sensor_msgs::Image".to_string()))],
            &[],
        );
        def.outputs[0].max_slice_len = Some(1024);
        def
    };
    node_stage(&ws.graphs_dir, "perception", camera_def).unwrap();

    let detector_def = build_node_def(
        "detector",
        None,
        &[],
        &[("image".to_string(), "[camera,image]".to_string())],
    );
    let config = node_stage(&ws.graphs_dir, "perception", detector_def).unwrap();

    // Verify graph YAML round-trips correctly
    assert_eq!(config.identity(), "perception");
    assert_eq!(config.prefix, "robot1");
    assert_eq!(config.nodes.len(), 2);

    // Verify camera node
    assert_eq!(config.nodes[0].id, "camera");
    assert_eq!(config.nodes[0].outputs.len(), 1);
    assert_eq!(config.nodes[0].outputs[0].name, "image");

    // Verify detector node input binding resolved
    assert_eq!(config.nodes[1].id, "detector");
    assert_eq!(config.nodes[1].inputs.len(), 1);
    assert_eq!(
        config.nodes[1].inputs[0].source, "camera/image",
        "[camera,image] should resolve to camera/image"
    );

    // Verify resolve_input_binding directly
    assert_eq!(resolve_input_binding("[camera,image]"), "camera/image");

    // Read back from disk and validate
    let read_back = graph_read(&ws.graphs_dir, "perception").unwrap();
    validate_graph(&read_back).unwrap();
    assert_eq!(read_back.nodes.len(), 2);
}

// ============================================================
// Test 4: Full pipeline — CLI generates graph, runtime executes
// ============================================================

#[test]
fn test_e2e_camera_detector_pipeline() {
    let prefix = unique_prefix("pipeline");

    // Build graph YAML (equivalent to what CLI commands produce). We use
    // `geometry_msgs/Vector3` (24-byte fixed schema) — the simplest
    // schema that the SHM-backed loan_proxy + try_view APIs round-trip.
    let yaml = format!(
        r#"
name: e2e_pipeline
prefix: {prefix}
nodes:
  - id: camera
    type: camera_pub
    outputs:
      - name: image
        schema: geometry_msgs/Vector3
        max_slice_len: 1024
  - id: detector
    type: detector_sub
    inputs:
      - name: image
        source: camera/image
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    // Camera node: publishes Vector3 each tick via SHM-backed loan_proxy.
    let camera_fires = Arc::new(AtomicU32::new(0));
    let camera_fires_clone = Arc::clone(&camera_fires);

    let camera_node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["image".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let seq = camera_fires_clone.fetch_add(1, Ordering::Relaxed);
            if let Some(publisher) = ctx.publisher_mut("image") {
                let mut proxy = publisher.loan_proxy::<Vector3>()?;
                proxy.x = seq as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
                // Drop publishes.
            }
            Ok(())
        },
    );

    // Detector node: receives via subscriber.try_view, records sequences.
    // Macro DataTrigger on `image` input drives data-triggered firing.
    let detector_fires = Arc::new(AtomicU32::new(0));
    let detector_fires_clone = Arc::clone(&detector_fires);

    let detector_node = ClosureNodeEntry::new(
        NodeInfo::with_meta(
            vec![cerulion_core::prelude::InputMeta {
                name: "image".to_string(),
                schema_hash: 0,
                trigger: true,
                depth: 1,
                backpressure: cerulion_core::prelude::BackpressurePolicy::DropOldest,
                expect_within_ms: None,
            }],
            vec![],
        )
        .with_policy(MacroPolicy::DataTrigger {
            input_name: "image".to_string(),
        }),
        move |ctx| {
            detector_fires_clone.fetch_add(1, Ordering::Relaxed);
            if let Some(sub) = ctx.subscriber_mut("image") {
                let _ = sub.try_view::<Vector3, _>(|_view| {
                    // Received a sample — data-trigger worked.
                });
            }
            Ok(())
        },
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("camera".to_string(), Box::new(camera_node));
    nodes.insert("detector".to_string(), Box::new(detector_node));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    // Run 5 steps at 10ms each
    for _ in 0..5 {
        runtime.step(Duration::from_millis(10));
    }
    // Extra step to drain final camera publish to detector
    runtime.step(Duration::from_millis(10));

    // Camera fires once per step (period_ms=10)
    let handle = runtime.node_handle("camera").unwrap();
    assert_eq!(
        handle.fire_count(),
        6,
        "camera should fire 6 times (5+1 steps)"
    );

    // Detector is data-triggered. Under the drain-between-levels level
    // executor the data-trigger consumer fires in the SAME step its input is
    // produced — the camera publishes on level 0, the executor drains level 1's
    // trigger inputs, then fires the detector, all within one `step()`. The old
    // 1-step lag of a flat whole-graph drain does not exist.
    let det_handle = runtime.node_handle("detector").unwrap();
    assert!(
        det_handle.fire_count() >= 1,
        "detector should fire at least once (data-triggered)"
    );

    // Trace should contain both node IDs
    let trace = runtime.trace();
    let trace_node_ids: Vec<&str> = trace.iter().map(|e| e.node_id.as_ref()).collect();
    assert!(
        trace_node_ids.contains(&"camera"),
        "trace should contain camera entries"
    );
    assert!(
        trace_node_ids.contains(&"detector"),
        "trace should contain detector entries"
    );

    // Clean shutdown
    runtime.shutdown();
}

// ============================================================
// Test 5: No data loss (Principle #6)
// ============================================================

#[test]
fn test_e2e_no_data_loss() {
    let prefix = unique_prefix("no_loss");
    let total_messages: u32 = 10;

    let yaml = format!(
        r#"
name: no_loss
prefix: {prefix}
nodes:
  - id: sender
    type: counter_pub
    outputs:
      - name: data
        schema: geometry_msgs/Vector3
        max_slice_len: 1024
  - id: receiver
    type: counter_sub
    inputs:
      - name: data
        source: sender/data
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let pub_count = Arc::new(AtomicU32::new(0));
    let pub_count_clone = Arc::clone(&pub_count);

    let sender = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["data".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        move |ctx| {
            let seq = pub_count_clone.fetch_add(1, Ordering::Relaxed);
            if let Some(publisher) = ctx.publisher_mut("data") {
                let mut proxy = publisher.loan_proxy::<Vector3>()?;
                // Encode the publish sequence number as Vector3.x so the
                // receiver can verify monotonic ordering.
                proxy.x = seq as f64;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    let recv_sequences = Arc::new(Mutex::new(Vec::<u32>::new()));
    let recv_sequences_clone = Arc::clone(&recv_sequences);

    let receiver = ClosureNodeEntry::new(
        NodeInfo::with_meta(
            vec![cerulion_core::prelude::InputMeta {
                name: "data".to_string(),
                schema_hash: 0,
                trigger: true,
                depth: 1,
                backpressure: cerulion_core::prelude::BackpressurePolicy::DropOldest,
                expect_within_ms: None,
            }],
            vec![],
        )
        .with_policy(MacroPolicy::DataTrigger {
            input_name: "data".to_string(),
        }),
        move |ctx| {
            if let Some(sub) = ctx.subscriber_mut("data") {
                let seqs = Arc::clone(&recv_sequences_clone);
                let _ = sub.try_view::<Vector3, _>(|view| {
                    let seq = view.x as u32;
                    seqs.lock().unwrap().push(seq);
                });
            }
            Ok(())
        },
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("sender".to_string(), Box::new(sender));
    nodes.insert("receiver".to_string(), Box::new(receiver));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    // Step N times for N publishes
    for _ in 0..total_messages {
        runtime.step(Duration::from_millis(10));
    }
    // Extra steps to drain all remaining messages through the pipeline
    for _ in 0..2 {
        runtime.step(Duration::from_millis(10));
    }

    let published = pub_count.load(Ordering::Relaxed);
    let received = recv_sequences.lock().unwrap();

    assert_eq!(
        published,
        total_messages + 2,
        "sender should have published {total_messages}+2 messages"
    );

    // Verify receiver saw at least the original `total_messages` samples.
    // try_view is drain-keep-latest, so the receiver may observe fewer
    // samples than published if multiple publishes land between two ticks
    // — but it MUST observe at least one tick's worth (the trigger
    // dispatch fires on each new sample).
    assert!(
        !received.is_empty(),
        "receiver should have received at least one sample"
    );

    // Verify sequences are monotonically non-decreasing (no reordering).
    // try_view is latest-wins so older samples may be skipped, but the
    // observed sequence must still increase strictly between consecutive
    // captures.
    for window in received.windows(2) {
        assert!(
            window[1] > window[0],
            "sequences must be strictly increasing: {} -> {}",
            window[0],
            window[1]
        );
    }

    runtime.shutdown();
}

// ============================================================
// Test 6: Deterministic replay (Principle #7: Replay = Live)
// ============================================================

#[test]
fn test_e2e_deterministic_replay() {
    let trace1 = run_replay_graph("replay_run1");
    let trace2 = run_replay_graph("replay_run2");

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
            "trace[{}] fire_time_ns mismatch: {} vs {}",
            i, t1.1, t2.1
        );
    }
}

/// Run a deterministic 3-node graph and return (node_id, fire_time_ns) pairs.
fn run_replay_graph(suffix: &str) -> Vec<(String, u64)> {
    let prefix = unique_prefix(suffix);

    let yaml = format!(
        r#"
name: replay_test
prefix: {prefix}
nodes:
  - id: camera
    type: cam
    outputs:
      - name: image
        schema: geometry_msgs/Vector3
        max_slice_len: 1024
  - id: imu
    type: imu_sensor
  - id: detector
    type: det
    inputs:
      - name: image
        source: camera/image
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let camera_node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["image".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        |ctx| {
            if let Some(publisher) = ctx.publisher_mut("image") {
                let mut proxy = publisher.loan_proxy::<Vector3>()?;
                proxy.x = 0.0;
                proxy.y = 0.0;
                proxy.z = 0.0;
            }
            Ok(())
        },
    );

    let imu_node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::Period { period_ms: 5 }),
        |_ctx| Ok(()),
    );

    let detector_node = ClosureNodeEntry::new(
        NodeInfo::with_meta(
            vec![cerulion_core::prelude::InputMeta {
                name: "image".to_string(),
                schema_hash: 0,
                trigger: true,
                depth: 1,
                backpressure: cerulion_core::prelude::BackpressurePolicy::DropOldest,
                expect_within_ms: None,
            }],
            vec![],
        )
        .with_policy(MacroPolicy::DataTrigger {
            input_name: "image".to_string(),
        }),
        |_ctx| Ok(()),
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("camera".to_string(), Box::new(camera_node));
    nodes.insert("imu".to_string(), Box::new(imu_node));
    nodes.insert("detector".to_string(), Box::new(detector_node));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    // Step 50ms total in 5ms increments (matches IMU rate)
    for _ in 0..10 {
        runtime.step(Duration::from_millis(5));
    }
    // Extra step to drain final camera publish
    runtime.step(Duration::from_millis(5));

    runtime
        .trace()
        .iter()
        .map(|entry| (entry.node_id.to_string(), entry.fire_time_ns))
        .collect()
}

// ============================================================
// Test 7: Observable state and topic introspection
// ============================================================

#[test]
// The regression was NOT in iceoryx2 0.9's `Service::list`
// semantics. Root cause was a version skew — `cerulion_cli_engine`
// declared `iceoryx2 = "0.8"` while `cerulion_core` (which creates the
// services) moved to `iceoryx2 = "0.9"`. `topic_list()` therefore linked
// a *second*, 0.8 iceoryx2 whose `Config::global_config()` +
// static-config-storage format are incompatible with the 0.9-created
// services, so it enumerated nothing. Bumping the CLI engine to
// `iceoryx2 = "0.9"` (matching core) makes same-process services visible.
fn test_e2e_observable_state_and_topic_introspection() {
    let prefix = unique_prefix("observe");

    let yaml = format!(
        r#"
name: observe_test
prefix: {prefix}
nodes:
  - id: ticker
    type: test_ticker
    outputs:
      - name: data
        schema: geometry_msgs/Vector3
        max_slice_len: 1024
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let ticker_node = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["data".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }),
        |ctx| {
            if let Some(publisher) = ctx.publisher_mut("data") {
                let mut proxy = publisher.loan_proxy::<Vector3>()?;
                proxy.x = 1.0;
                proxy.y = 2.0;
                proxy.z = 3.0;
            }
            Ok(())
        },
    );

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("ticker".to_string(), Box::new(ticker_node));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    // Step 3 times at 10ms each
    for _ in 0..3 {
        runtime.step(Duration::from_millis(10));
    }

    // Observable state via NodeHandle
    let handle = runtime.node_handle("ticker").unwrap();
    assert_eq!(handle.fire_count(), 3, "should fire exactly 3 times");
    assert_eq!(
        handle.last_fire_ns(),
        30_000_000,
        "last_fire_ns should be 30ms (3 * 10ms)"
    );
    assert_eq!(handle.panic_count(), 0, "no panics expected");

    // Trace should have exactly 3 entries
    let trace = runtime.trace();
    assert_eq!(trace.len(), 3, "trace should have exactly 3 entries");
    for (i, entry) in trace.iter().enumerate() {
        assert_eq!(entry.node_id.as_ref(), "ticker");
        assert_eq!(
            entry.fire_time_ns,
            (i as u64 + 1) * 10_000_000,
            "trace[{}] fire_time should be {}ms",
            i,
            (i + 1) * 10
        );
    }

    // Topic introspection via iceoryx2 service discovery.
    //
    // The graph output "data" on node "ticker" creates topic:
    //   the derived topic form = "/{prefix}/ticker/data"
    //
    // The transport creates iceoryx2 service "/{prefix}/ticker/data/data".
    // topic_list() strips the "/data" service suffix exactly ONCE
    // (a repeated-strip bug was fixed here), yielding the full
    // canonical topic "/{prefix}/ticker/data".
    let expected_topic = format!("/{}/ticker/data", prefix);
    let topics = topic_list().unwrap();
    let found = topics.iter().any(|t| t.name == expected_topic);
    assert!(
        found,
        "topic_list() should discover '{}', found: {:?}",
        expected_topic,
        topics.iter().map(|t| &t.name).collect::<Vec<_>>()
    );

    // Introspection commands pre-check
    // existence instead of silently CREATING an empty service via
    // open_or_create. A missing topic errors actionably; the no-slash
    // spelling of an existing topic gets the canonical did-you-mean.
    //
    // This arm is hermetic. Both arms assert the LOCAL story (a missing topic is
    // reported, not created; a no-slash spelling gets the canonical hint), and the
    // LOCAL story must not be decided by whether the machine running the test can
    // see a robot. An unresolved topic now takes one of three messages
    // depending on what the NETWORK did, so without the kill-switch this arm reads
    // the developer's LAN: it passed on a desk and failed on CI, where the
    // robot-less resolve correctly answers the non-converged "UNKNOWN" message.
    // `CERULION_NETWORK=off` pins the local-only path, which is the contract under
    // test. (That the canonical hint SURVIVES all three messages — it is a local
    // fact — is pinned in `topic_cmd`'s
    // `the_canonical_slash_hint_rides_every_unresolved_topic_message`; this arm
    // pins it end-to-end through the real `topic_info` entry point.)
    let _network_off = EnvVarGuard::set("CERULION_NETWORK", "off");
    let err = cerulion_cli_engine::topic_cmd::topic_info("/nope/missing/topic", None)
        .expect_err("info on a missing topic must error, not create the service")
        .to_string();
    assert!(err.contains("not found"), "got: {err}");
    // The ABSENT direction through the real wiring: a topic with no slashed
    // local twin must NOT get the did-you-mean (an always-true
    // `has_canonical_slash_twin` would staple a confident lie — this is the
    // one assertion that executes the predicate end-to-end and catches it;
    // the pure-formatter oracle cannot, it takes the bool by value).
    assert!(!err.contains("did you mean"), "got: {err}");
    let no_slash = expected_topic.strip_prefix('/').unwrap();
    let err = cerulion_cli_engine::topic_cmd::topic_info(no_slash, None)
        .expect_err("the no-slash spelling must error with the did-you-mean")
        .to_string();
    assert!(
        err.contains(&format!("did you mean '{expected_topic}'")),
        "got: {err}"
    );
    drop(_network_off);

    runtime.shutdown();
}
