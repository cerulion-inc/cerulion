// SPDX-License-Identifier: AGPL-3.0-only
//! Group 5+6: end-to-end clock-model contracts over real
//! iceoryx2. RUN SERIAL (`--test-threads=1`): iceoryx2's SHM singleton.
//!
//! - Test 5 (whole-runtime-on-external-time): a graph built on an
//!   `ExternalClock` observes node-side `now_ns()` EQUAL to the fed external
//!   sequence, with NO node code change, and two runs of the same fed sequence
//!   produce byte-identical observed-time traces (determinism).
//! - Test 6 (publish-stamp follows the ACTIVE clock — the determinism
//!   lock): under a `VirtualClock` runtime, a published message's
//!   `WireHeader.timestamp_ns` equals the virtual-clock value at publish, NOT
//!   wall `real_ns()`. This pins that `publisher.rs` stamps from the active
//!   clock (`self.clock.now_ns()`), the foundation of replay determinism.
//!
//! Clock injection: `build_for_test` is `VirtualClock`-only, so the External
//! runtime uses `GraphRuntime::build_live` (accepts any `Arc<dyn Clock>`) over
//! an isolated per-test `TransportManager::init_for_test` carrying the same
//! clock. Both halves share the clock so the scheduler, node shim, and
//! publisher all read ONE time source.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::{real_ns, Clock, ExternalClock, VirtualClock};
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::WireHeader;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

// ===========================================================================
// Test 5 — whole runtime on external time
// ===========================================================================

#[derive(Debug, Default)]
struct TimeTrace {
    now_ns: Mutex<Vec<u64>>,
}

static TIME_TRACE: std::sync::OnceLock<Arc<TimeTrace>> = std::sync::OnceLock::new();

/// Vanilla period node — UNCHANGED node code. It records `self.now_ns()` (the
/// active clock) each tick. The whole point of test 5: this node has no idea
/// it's on external time; it just reads `now_ns()`.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct ExtTimeSink {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl ExtTimeSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let t = self.now_ns();
        self.out.x = t as f64;
        TIME_TRACE
            .get()
            .expect("trace set by test")
            .now_ns
            .lock()
            .unwrap()
            .push(t);
        Ok(())
    }
}

fn ext_time_config() -> GraphConfig {
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_time".to_string(),
        prefix: "ext_time".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "sink".to_string(),
            node_type: "ext_time_sink".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    }
}

/// Run the graph on a FRESH ExternalClock, feeding `fed` between steps, and
/// return the node-observed `now_ns()` trace.
fn run_ext_time(fed: &[u64]) -> Vec<u64> {
    let trace = Arc::new(TimeTrace::default());
    // OnceLock is set-once; install on first run, reuse + clear afterwards.
    let _ = TIME_TRACE.set(trace.clone());
    let installed = TIME_TRACE.get().expect("trace present");
    installed.now_ns.lock().unwrap().clear();

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(ExtTimeSinkEntry::new()));

    let ext = Arc::new(ExternalClock::new());
    let clock_dyn: Arc<dyn Clock> = ext.clone();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ext_time_test".into(),
            clock: clock_dyn.clone(),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test");
    let mut runtime = GraphRuntime::build_live(ext_time_config(), factories, &mgr, clock_dyn)
        .expect("build_live");

    for &t in fed {
        ext.set_external(t);
        runtime.step(Duration::from_millis(1));
    }

    installed.now_ns.lock().unwrap().clone()
}

#[test]
#[serial]
fn whole_runtime_observes_fed_external_time() {
    // Monotonically-increasing fed sequence (period node fires once per step
    // as long as the external clock crossed the 1ms deadline).
    let fed = [1_000_000u64, 2_000_000, 3_000_000, 4_000_000, 5_000_000];
    let observed = run_ext_time(&fed);

    assert_eq!(
        observed.len(),
        fed.len(),
        "period node should fire once per fed external timestamp"
    );
    // Node-observed now_ns() equals EXACTLY the fed external value — NO node
    // code change, the runtime is fully driven by the external master.
    assert_eq!(
        observed,
        fed.to_vec(),
        "node-observed now_ns() must equal the fed external sequence verbatim"
    );
}

#[test]
#[serial]
fn external_time_observed_trace_is_deterministic() {
    let fed = [1_000_000u64, 2_000_000, 3_000_000, 4_000_000, 5_000_000];
    let a = run_ext_time(&fed);
    let b = run_ext_time(&fed);
    assert_eq!(
        a, b,
        "two runs of the same fed external sequence must produce byte-identical \
         observed-time traces (Principle #7: Replay = Live)"
    );
}

// ===========================================================================
// Test 6 — publish stamp follows the ACTIVE clock (F4 lock)
// ===========================================================================

/// Period producer that publishes one Vector3 per tick. The publisher stamps
/// the WireHeader from the runtime's ACTIVE clock; under a VirtualClock the
/// stamp must equal the virtual-clock value at loan time, NOT wall time.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct StampProducer {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl StampProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

#[test]
#[serial]
fn publish_stamp_follows_active_virtual_clock_not_wall() {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "stamp".to_string(),
        prefix: "stamp".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "prod".to_string(),
            node_type: "stamp_producer".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("prod".to_string(), Box::new(StampProducerEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock.clone(), 8).expect("build_for_test");

    // Subscribe to the producer's derived topic via the SAME isolated
    // transport BEFORE the producer fires (history is disabled, so a
    // late-joining subscriber would miss the frame).
    let mgr = runtime.test_transport().expect("test transport parked");
    let topic = "/stamp/prod/out"; // /{prefix}/{node_id}/{output_name}
    let subscriber = mgr.create_subscriber(topic).expect("create subscriber");

    // Step the runtime by 7ms. For a VirtualClock runtime, `step(delta)`
    // ADVANCES the virtual clock by `delta` (= 7ms here) and then fires the
    // period node within the same step — so the publisher loans/stamps with
    // the active clock at exactly 7ms. (Do NOT also call `advance_ms` here:
    // that would double-advance the clock to 14ms before the fire.) This lands
    // a distinctive small value far below any plausible wall `real_ns()`
    // (which is CLOCK_MONOTONIC ns since boot — minutes-to-hours of ns), so a
    // regression that stamped wall time would read orders of magnitude larger.
    let wall_at_publish = real_ns();
    runtime.step(Duration::from_millis(7));

    // Read the received frame's WireHeader.timestamp_ns.
    let mut stamped: Option<u64> = None;
    // A couple of receive attempts cover any iceoryx2 delivery latency.
    for _ in 0..50 {
        let got = subscriber
            .try_receive_one(|msg| {
                stamped = Some(msg.header().timestamp_ns);
            })
            .expect("try_receive_one");
        if got {
            break;
        }
        std::thread::sleep(Duration::from_millis(2));
    }

    let stamped = stamped.expect("subscriber must receive the published frame");
    assert_eq!(
        stamped, 7_000_000,
        "WireHeader.timestamp_ns must equal the VIRTUAL clock at publish (7ms), \
         proving the publisher stamps from the ACTIVE clock — got {stamped}"
    );
    // Belt-and-suspenders: the stamp is NOT wall time. Wall `real_ns()` is
    // CLOCK_MONOTONIC ns since boot — vastly larger than 7ms. A regression
    // that stamped wall time would land near `wall_at_publish`, not 7ms.
    assert!(
        stamped < wall_at_publish,
        "the virtual stamp ({stamped}) must NOT be wall time (>= {wall_at_publish})"
    );

    // Sanity: WireHeader::SIZE is the 32-byte header these bytes carry.
    assert_eq!(WireHeader::SIZE, 32, "WireHeader is 32 bytes");
}
