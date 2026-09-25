// SPDX-License-Identifier: AGPL-3.0-only
//! Env-snapshot isolation property test.
//!
//! Locks the contract that `NodeContext::env_snapshot` (captured at
//! `GraphRuntime::build` / `build_for_test` time via
//! `std::env::vars().collect()`) ISOLATES nodes from post-build env
//! mutations. Without this file the property exists in code but has no
//! test exercising it end-to-end — a regression that reverts to live
//! `std::env::var` reads (e.g. dropping the snapshot wiring) would
//! silently re-break determinism without any test firing.
//!
//! There is no
//! "without a snapshot" code path. Every NodeContext
//! constructor produces an `Arc<HashMap>` snapshot; the only
//! difference is whether it's empty (test-only `for_tests`) or
//! pre-populated (production `with_runtime_env`). Tests below
//! exclusively exercise the populated-snapshot isolation property
//! through `GraphRuntime::build_for_test`, which captures live
//! env once at build time.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::SystemTime;

use cerulion_core::clock::VirtualClock;
use cerulion_core::error::TransportResult;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo};
use indexmap::IndexMap;
use serial_test::serial;

// Tests in this file mutate process env vars
// (`CER_TEST_ENV_ISO_*`). Each `#[test]` is annotated with `#[serial]`
// (from the `serial_test` dev-dep) so they run one at a time within
// this binary.

static NODE_ID_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_id(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = NODE_ID_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{base}_{nanos}_{id}")
}

/// Build a single-node runtime whose tick captures the result of
/// `ctx.env_str(key, default)` into the shared `recorded_value`.
fn build_runtime_recording_env_var(
    key: &'static str,
    default: &'static str,
    recorded_value: Arc<Mutex<Vec<String>>>,
) -> GraphRuntime {
    let info = NodeInfo::from_names(vec![], vec!["noop".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    let recorded_for_tick = Arc::clone(&recorded_value);
    let entry = ClosureNodeEntry::new(info, move |ctx: &mut NodeContext| {
        let v = ctx.env_str(key, default);
        recorded_for_tick.lock().unwrap().push(v);
        Ok(())
    });

    let node_id = unique_id("env_iso");
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "env_iso".to_string(),
        prefix: "envt".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: node_id.clone(),
            node_type: node_id,
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "noop".to_string(),
                schema: "u8".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(config.nodes[0].id.clone(), Box::new(entry));

    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 4).expect("build_for_test")
}

/// Step the runtime once: advance simulated time by 1ms and run one
/// scheduler step so the period-1ms node fires.
fn tick_once(rt: &mut GraphRuntime) -> TransportResult<()> {
    rt.step(std::time::Duration::from_millis(1));
    Ok(())
}

/// Happy path: build runtime → mutate env → tick → assert tick saw the
/// PRE-BUILD value (i.e. the snapshot froze at build time).
#[test]
#[serial]
fn env_snapshot_isolates_post_build_env_mutations() {
    let key = "CER_TEST_ENV_ISO_SET";
    // Defensive cleanup at test ENTRY (in case a prior panicking test
    // leaked the var). Mirrors `chunk_c_ffi_error_test.rs` pattern.
    std::env::remove_var(key);
    std::env::set_var(key, "before_build");

    let recorded = Arc::new(Mutex::new(Vec::<String>::new()));
    let mut rt = build_runtime_recording_env_var(key, "default", Arc::clone(&recorded));

    // Mutate env AFTER build. The snapshot was captured at build time
    // (line ~182 / 351 of runtime.rs via std::env::vars().collect()).
    std::env::set_var(key, "after_build");

    tick_once(&mut rt).unwrap();
    drop(rt);

    let captured = recorded.lock().unwrap();
    assert_eq!(
        captured.len(),
        1,
        "expected exactly one tick capture; got {captured:?}"
    );
    assert_eq!(
        captured[0], "before_build",
        "node must observe the env value at the time of GraphRuntime::build_for_test, \
         NOT the post-build mutation; got: {:?}",
        captured[0]
    );

    std::env::remove_var(key);
}

/// Edge: env var REMOVED after build — node must still observe the
/// snapshotted value, not "default".
#[test]
#[serial]
fn env_snapshot_unaffected_by_remove_var_post_build() {
    let key = "CER_TEST_ENV_ISO_REMOVE";
    std::env::remove_var(key); // defensive entry cleanup
    std::env::set_var(key, "captured");

    let recorded = Arc::new(Mutex::new(Vec::<String>::new()));
    let mut rt = build_runtime_recording_env_var(key, "default_when_unset", Arc::clone(&recorded));

    std::env::remove_var(key);

    tick_once(&mut rt).unwrap();
    drop(rt);

    let captured = recorded.lock().unwrap();
    assert_eq!(
        captured[0], "captured",
        "post-build remove_var must NOT cause the snapshot's value to disappear; \
         got: {:?}",
        captured[0]
    );
}

/// Adversarial: env var was UNSET at build time → snapshot lacks the
/// key → tick must return the user-supplied default. Then setting the
/// var post-build must NOT cause the node to suddenly see it.
#[test]
#[serial]
fn env_snapshot_returns_default_when_var_unset_at_build_time() {
    let key = "CER_TEST_ENV_ISO_UNSET";
    std::env::remove_var(key); // ensure unset

    let recorded = Arc::new(Mutex::new(Vec::<String>::new()));
    let mut rt = build_runtime_recording_env_var(key, "default_at_build", Arc::clone(&recorded));

    // Set the var post-build. Snapshot must NOT see it.
    std::env::set_var(key, "set_after_build");

    tick_once(&mut rt).unwrap();
    drop(rt);

    let captured = recorded.lock().unwrap();
    assert_eq!(
        captured[0], "default_at_build",
        "env_str must return user-supplied default for keys absent from \
         the snapshot, regardless of post-build mutations; got: {:?}",
        captured[0]
    );

    std::env::remove_var(key);
}

/// Determinism: TWO ticks of the same node observe the SAME snapshot
/// value, even when env mutates between ticks. (Locks that the snapshot
/// is read fresh on each tick from the FROZEN map, not snapshotted at
/// init and re-read.)
#[test]
#[serial]
fn env_snapshot_value_is_stable_across_ticks_under_env_churn() {
    let key = "CER_TEST_ENV_ISO_CHURN";
    std::env::remove_var(key); // defensive entry cleanup
    std::env::set_var(key, "stable");

    let recorded = Arc::new(Mutex::new(Vec::<String>::new()));
    let mut rt = build_runtime_recording_env_var(key, "default", Arc::clone(&recorded));

    tick_once(&mut rt).unwrap();

    std::env::set_var(key, "churned_between_ticks");
    tick_once(&mut rt).unwrap();

    std::env::remove_var(key);
    tick_once(&mut rt).unwrap();

    drop(rt);

    let captured = recorded.lock().unwrap();
    assert_eq!(captured.len(), 3, "got: {captured:?}");
    assert!(
        captured.iter().all(|v| v == "stable"),
        "all ticks must observe the snapshotted value 'stable' regardless \
         of env churn between ticks; got: {captured:?}"
    );

    std::env::remove_var(key);
}
