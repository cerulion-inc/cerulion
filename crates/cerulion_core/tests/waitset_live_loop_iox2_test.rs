// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end WaitSet LIVE LOOP over real iceoryx2.
//!
//! Drives [`GraphRuntime::run_live_step_once_for_test`] — the
//! single-iteration seam of the production [`GraphRuntime::run_live`] live
//! loop — over a `single data-trigger consumer` graph whose trigger topic is
//! an absolute EXTERNAL source. An out-of-graph test publisher (created via
//! the parked `test_transport`) writes onto that exact topic, decoupling the
//! publish from any in-graph period producer.
//!
//! Unlike the record-only `run_waitset_reactor_once_for_test` seam,
//! `run_live_step_once_for_test` DOES advance the graph: it runs one
//! `live_step` (block on the WaitSet for a real wakeup or timeout, then the
//! EXISTING `step` / `drain_level` does the real firing). This exercises
//! the production live path one iteration at a time on the main thread (no
//! `GraphRuntime: Send` requirement).
//!
//! Contracts, all `#[serial]` (the live loop builds an iceoryx2 WaitSet over
//! the process-global shared-memory singleton):
//!
//! 1. **Positive** — after the external publisher writes one frame, ONE
//!    `live_step` wakes on the `SentSample` event and `step`/`drain_level`
//!    fires the consumer (its `fire_count` increments).
//! 2. **Negative / blocking** — with NO publish, ONE `live_step` returns after
//!    ~timeout (the reactor blocked, nothing arrived) with the consumer NOT
//!    fired.
//! 3. **Correctness** — N publishes interleaved with N `live_step` calls fires
//!    the consumer EXACTLY N times (no lost / dup).
//! 4. **Determinism** — two independent builds given identical treatment yield
//!    the same fire counts (Principle #7).
//!
//! ## On iceoryx2 event-listener semantics (priming)
//!
//! A trigger input's iceoryx2 event `Listener` multiplexes CONNECTION
//! lifecycle events (`SubscriberConnected` / `PublisherConnected`, queued at
//! build / on a publisher attach) alongside data `SentSample` events. So a
//! freshly-built graph's listener already has pending events BEFORE any data
//! publish, and attaching the external publisher queues another. A `live_step`
//! that wakes on connection noise still calls `step` — but `drain_level`
//! finds no data, so the consumer does NOT fire. The tests PRIME (one
//! `live_step` after wiring the external publisher) to drain that noise before
//! asserting on a specific data publish, so a connection-noise wake never
//! masks a missing/extra data fire.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The absolute external topic the consumer's trigger input subscribes — no
/// in-graph producer, so the graph provisions it as `External` (buffer-ceiling
/// only, no single-writer cap) and an out-of-graph publisher attaches freely.
const EXT_TOPIC: &str = "/live/ext/cam";

/// A short wake timeout: long enough that a published event wakes the reactor
/// well before it elapses (positive path), short enough that the no-publish
/// blocking path returns quickly (negative path).
const WAKE_TIMEOUT: Duration = Duration::from_millis(150);

/// Data-trigger consumer of an absolute external topic. Counts its fires in a
/// shared `Arc<AtomicU64>` so the test can read the count without a handle.
#[cerulion_node]
#[derive(Default)]
struct LiveConsumer {
    #[input(trigger)]
    inp: Vector3,
    fires: Arc<AtomicU64>,
    sum: f64,
}

#[cerulion_node_impl]
impl LiveConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// Build a single-consumer graph whose trigger input is the absolute external
/// topic `EXT_TOPIC`.
fn live_graph(fires: Arc<AtomicU64>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "waitset_live_loop_test".to_string(),
        prefix: "wsl".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "consumer".to_string(),
            node_type: "live_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: EXT_TOPIC.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "consumer".to_string(),
        Box::new(LiveConsumerEntry::with_state(LiveConsumer {
            fires,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// Build the graph over an isolated test transport and return the runtime plus
/// the shared fire counter.
fn build() -> (GraphRuntime, Arc<AtomicU64>) {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = live_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build waitset live-loop graph");
    (runtime, fires)
}

/// Attach an out-of-graph publisher on `EXT_TOPIC` via the runtime's parked
/// test transport.
fn external_publisher(runtime: &GraphRuntime) -> cerulion_core::CerulionPublisher {
    let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
    mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("external publisher must attach to the absolute external topic")
}

/// Publish exactly ONE frame onto `EXT_TOPIC` (the proxy publishes on drop).
fn publish_one(pubr: &mut cerulion_core::CerulionPublisher, x: f64) {
    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
    proxy.x = x;
    drop(proxy); // publish
}

/// Read the consumer's fire count two ways and assert they agree, returning it.
/// `fires` is the in-node `Arc<AtomicU64>`; `node_handle` is the scheduler's
/// own counter — both must track the same firing path.
fn fire_count(runtime: &GraphRuntime, fires: &Arc<AtomicU64>) -> u64 {
    let counter = fires.load(Ordering::Relaxed);
    let handle = runtime
        .node_handle("consumer")
        .expect("consumer handle")
        .fire_count();
    assert_eq!(
        counter, handle,
        "the in-node fire counter ({counter}) and the scheduler's fire_count \
         ({handle}) must agree — both reflect the same `step`/`drain_level` \
         firing path"
    );
    counter
}

/// Drain build-time + attach-time connection-lifecycle noise so subsequent
/// assertions are attributable to a data publish, not connection events. One
/// `live_step` with no data published wakes on the noise, then `step` finds no
/// data and does NOT fire the consumer.
fn prime(runtime: &mut GraphRuntime, fires: &Arc<AtomicU64>) {
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        fire_count(runtime, fires),
        0,
        "priming (no data published) must NOT fire the consumer — a connection \
         -noise wake calls `step`, but `drain_level` finds no data"
    );
}

#[test]
#[serial]
fn live_step_fires_consumer_on_external_publish() {
    let (mut runtime, fires) = build();
    let mut pubr = external_publisher(&runtime);
    prime(&mut runtime, &fires);

    // Publish one frame, then ONE live iteration: the reactor wakes on the
    // `SentSample` event and `step`/`drain_level` fires the consumer.
    publish_one(&mut pubr, 1.0);
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);

    assert_eq!(
        fire_count(&runtime, &fires),
        1,
        "after one external publish, one `live_step` must wake on the event and \
         fire the data-trigger consumer exactly once"
    );
}

#[test]
#[serial]
fn live_step_does_not_fire_without_publish() {
    let (mut runtime, fires) = build();
    let _pubr = external_publisher(&runtime);
    prime(&mut runtime, &fires);

    // No publish. One `live_step` blocks the reactor for ~WAKE_TIMEOUT, then
    // `step` runs but `drain_level` finds nothing → consumer does NOT fire.
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);

    assert_eq!(
        fire_count(&runtime, &fires),
        0,
        "with NO publish, the reactor blocks for the full timeout and the \
         consumer must NOT fire (nothing arrived)"
    );
}

#[test]
#[serial]
fn live_step_fires_n_times_for_n_publishes() {
    let (mut runtime, fires) = build();
    let mut pubr = external_publisher(&runtime);
    prime(&mut runtime, &fires);

    // N publishes, each followed by ONE live iteration → exactly N fires, no
    // lost / dup. The payload increments so a dropped/duplicated frame would
    // also corrupt the running `sum`, but the fire COUNT is the load-bearing
    // assertion here.
    const N: u64 = 6;
    for i in 1..=N {
        publish_one(&mut pubr, i as f64);
        runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
        assert_eq!(
            fire_count(&runtime, &fires),
            i,
            "after publish #{i} + one `live_step`, the consumer must have fired \
             exactly {i} time(s) (no lost / dup)"
        );
    }

    assert_eq!(
        fire_count(&runtime, &fires),
        N,
        "N publishes interleaved with N `live_step` calls must fire the \
         consumer exactly N={N} times"
    );
}

/// Build a fresh graph, attach the external publisher, prime, then run N
/// publish+live_step cycles and return the final fire count — the determinism
/// oracle.
fn drive_n_publish_cycles(n: u64) -> u64 {
    let (mut runtime, fires) = build();
    let mut pubr = external_publisher(&runtime);
    prime(&mut runtime, &fires);
    for i in 1..=n {
        publish_one(&mut pubr, i as f64);
        runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    }
    fire_count(&runtime, &fires)
}

#[test]
#[serial]
fn live_loop_is_deterministic_across_runs() {
    // Two independent builds given identical treatment yield the same fire
    // count — bit-identical firing (Principle #7: Replay = Live).
    const N: u64 = 5;
    let first = drive_n_publish_cycles(N);
    let second = drive_n_publish_cycles(N);
    assert_eq!(
        first, second,
        "two independently-built graphs given identical treatment must yield \
         the same fire count"
    );
    // Hand-written exact oracle: N publishes, one fire each → exactly N.
    assert_eq!(
        first, N,
        "the fire count must be exactly N={N} (one fire per publish); first = {first}"
    );
}
