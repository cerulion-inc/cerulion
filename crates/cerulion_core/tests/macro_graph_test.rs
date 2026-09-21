// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for macro-generated nodes in GraphRuntime.
//!
//! Tests that `#[cerulion_node]` generated nodes work correctly when
//! wired into the graph runtime with scheduler and transport.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test macro_graph_test -- --test-threads=1
//! ```
//!
//! Must run single-threaded: iceoryx2 singleton + shared memory requires
//! serial access across transport tests.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::{parse_graph, validate_graph, GraphRuntime};
use cerulion_core::prelude::*;
use cerulion_core::transport::TransportManager;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Monotonic counter for unique topic prefixes.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_prefix(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("mgtest/{}/{}/{}", base, nanos, id)
}

// ============================================================
// Macro node definitions for graph tests (
// declarative-only, period-driven sources).
// ============================================================

#[cerulion_node(period_ms = 10)]
struct GraphCounterNode {
    count: u32,
    #[output]
    data: Vector3,
}

#[cerulion_node_impl]
impl GraphCounterNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        self.data.x = self.count as f64;
        Ok(())
    }
}

#[cerulion_node(period_ms = 10)]
struct GraphNoopNode {
    #[output]
    tick_marker: Vector3,
}

#[cerulion_node_impl]
impl GraphNoopNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Marker output keeps the node declarative without doing real work.
        self.tick_marker.x = 0.0;
        Ok(())
    }
}

// ============================================================
// M.21: test_macro_node_in_graph
// ============================================================

#[test]
fn test_macro_node_in_graph() {
    let prefix = unique_prefix("macro_graph");
    let yaml = format!(
        r#"
name: macro_test
prefix: {prefix}
nodes:
  - id: counter
    type: graph_counter
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

    let counter_node = GraphCounterNodeEntry::new();

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("counter".to_string(), Box::new(counter_node));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    // Step 30ms — should fire 3 times (period_ms=10)
    for _ in 0..3 {
        runtime.step(Duration::from_millis(10));
    }

    let handle = runtime.node_handle("counter").unwrap();
    assert_eq!(handle.fire_count(), 3, "counter should fire 3 times");
}

// ============================================================
// M.22: test_macro_node_deterministic_replay
// ============================================================

/// One fire of a `run_macro_graph` trace, reduced to the replay-DETERMINISTIC
/// fields: `(node_id, step, fire_time_ns, global_level)`. Those are exactly the
/// four fields `TraceEntry`'s hand-written `PartialEq` compares — the
/// wall-clock `duration_ns` and the `discarded` data annotation are excluded
/// there because neither is replayable (Principle #7).
type Fire = (String, u64, u64, usize);

/// What ONE `run_macro_graph` observed: the fire sequence off the trace, plus
/// each node's `fire_count()` read off its `NodeHandle`. The counters are a
/// SECOND, independent accounting path (bumped in `fire_node`, not derived from
/// the trace), so checking both against one oracle is two observations of the
/// run rather than one restated.
struct MacroGraphRun {
    fires: Vec<Fire>,
    fire_counts: Vec<(String, u64)>,
}

/// The fire sequence both runs must produce, written out BY HAND from the
/// declarations rather than read off a run — which is the whole point of this
/// arm. Derivation:
///
/// * `fast` (`graph_counter`) and `slow` (`graph_noop`) are both
///   `#[cerulion_node(period_ms = 10)]` sources with no inputs, so both are
///   graph ROOTS and fire at DAG `global_level` 0. A 2-wide level is below
///   `PARALLEL_FIRE_THRESHOLD`, so it fires SERIALLY in registration order —
///   the graph file's order, `fast` then `slow` (Principle #5: execution order
///   is derivable from the graph).
/// * `Scheduler::begin_step` advances the clock BEFORE any node is evaluated,
///   and `add_node` baselines a `Period` node's deadline at `now + interval`.
///   With a `VirtualClock` starting at 0, the 0-based step `k` therefore runs
///   at `(k + 1) * 10 ms` and the deadline due there is exactly that instant.
/// * The step delta EQUALS the period, so exactly one interval is due per step:
///   one fire per node per step, never a catch-up burst.
///
/// Hence 5 steps x 2 nodes = 10 fires, `fire_time_ns` climbing 10 ms -> 50 ms.
fn expected_fires() -> Vec<Fire> {
    vec![
        ("fast".to_string(), 0, 10_000_000, 0),
        ("slow".to_string(), 0, 10_000_000, 0),
        ("fast".to_string(), 1, 20_000_000, 0),
        ("slow".to_string(), 1, 20_000_000, 0),
        ("fast".to_string(), 2, 30_000_000, 0),
        ("slow".to_string(), 2, 30_000_000, 0),
        ("fast".to_string(), 3, 40_000_000, 0),
        ("slow".to_string(), 3, 40_000_000, 0),
        ("fast".to_string(), 4, 50_000_000, 0),
        ("slow".to_string(), 4, 50_000_000, 0),
    ]
}

#[test]
fn test_macro_node_deterministic_replay() {
    let expected = expected_fires();
    let expected_counts = vec![("fast".to_string(), 5_u64), ("slow".to_string(), 5_u64)];

    let run_a = run_macro_graph("replay_a");
    let run_b = run_macro_graph("replay_b");

    // Each run is checked against the HAND oracle first. Comparing the two runs
    // to each other and nothing else — what this test used to do — passes for
    // any pair of identically-wrong runs, the empty trace included: a graph
    // whose nodes never fire at all is perfectly reproducible.
    assert_eq!(
        run_a.fires, expected,
        "run A's fire sequence must equal the hand oracle"
    );
    assert_eq!(
        run_b.fires, expected,
        "run B's fire sequence must equal the hand oracle"
    );
    assert_eq!(
        run_a.fire_counts, expected_counts,
        "run A's NodeHandle fire counts must equal the hand oracle"
    );
    assert_eq!(
        run_b.fire_counts, expected_counts,
        "run B's NodeHandle fire counts must equal the hand oracle"
    );

    // Principle #7 (Replay = Live) stated explicitly. Implied by the two oracle
    // arms above, and kept because it is the contract this test is named for —
    // any future edit that loosens the oracle still has to answer to it.
    assert_eq!(
        run_a.fires, run_b.fires,
        "two runs of the same declarations must produce identical traces"
    );
}

fn run_macro_graph(suffix: &str) -> MacroGraphRun {
    let prefix = unique_prefix(suffix);
    let yaml = format!(
        r#"
name: replay_macro
prefix: {prefix}
nodes:
  - id: fast
    type: graph_counter
    outputs:
      - name: data
        schema: geometry_msgs/Vector3
        max_slice_len: 1024
  - id: slow
    type: graph_noop
    outputs:
      - name: tick_marker
        schema: geometry_msgs/Vector3
        max_slice_len: 1024
"#
    );

    let config = parse_graph(&yaml).unwrap();
    validate_graph(&config).unwrap();

    let mgr = TransportManager::get_or_init().expect("init");
    let clock = Arc::new(VirtualClock::new());

    let fast = GraphCounterNodeEntry::new();
    let slow = GraphNoopNodeEntry::new();

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("fast".to_string(), Box::new(fast));
    nodes.insert("slow".to_string(), Box::new(slow));

    let mut runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build");

    for _ in 0..5 {
        runtime.step(Duration::from_millis(10));
    }

    let fires = runtime
        .trace()
        .iter()
        .map(|e| {
            (
                e.node_id.to_string(),
                e.step,
                e.fire_time_ns,
                e.global_level,
            )
        })
        .collect();
    let fire_counts = ["fast", "slow"]
        .into_iter()
        .map(|id| {
            let handle = runtime
                .node_handle(id)
                .unwrap_or_else(|| panic!("node handle for `{id}`"));
            (id.to_string(), handle.fire_count())
        })
        .collect();

    MacroGraphRun { fires, fire_counts }
}
