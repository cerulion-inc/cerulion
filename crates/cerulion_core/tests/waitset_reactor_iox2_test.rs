// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end WaitSet reactor over real iceoryx2.
//!
//! Drives [`GraphRuntime::run_waitset_reactor_once_for_test`] — the
//! record-only WaitSet seam — over data-trigger consumer graphs built with
//! `build_for_test` (per-test isolated SHM root). Each consumer's trigger
//! input subscribes an ABSOLUTE EXTERNAL topic (a leading-`/` source with no
//! in-graph producer), so the graph provisions it as `External` (buffer-ceiling
//! only, NO single-writer cap) and an OUT-OF-GRAPH publisher — created via the
//! parked `test_transport` — can attach freely and publish onto the exact topic
//! the consumer subscribes. This is the reactor's real role: an external /
//! cross-process publisher waking the loop.
//!
//! ## Why the publish is out-of-graph, not `runtime.step()`
//!
//! The drain-BEFORE-fire executor was replaced with a
//! drain-between-LEVELS level executor: one `step()` now COLLAPSES the
//! producer→consumer chain — level-0 fires the producer (which publishes +
//! notifies the consumer's trigger `Listener`), then level-1's `drain_level`
//! calls `try_receive` (→ `drain_stale_events`, which CONSUMES the listener's
//! `SentSample` notification) and FIRES the consumer in the SAME `step()`. So
//! after a `step()` there is NO pending notification left for the reactor to
//! observe. The fix: never `step()` to set up the reactor's observation —
//! publish DIRECTLY onto the consumer's external trigger topic via an
//! out-of-graph publisher (the reactor's listener gets exactly one fresh
//! `SentSample`, and nothing in the graph drains it because the consumer's
//! level is never run). A subsequent real `step()` (in the firewall test) then
//! still finds the un-consumed SHM data sample and fires the consumer once —
//! proving the reactor's event-queue drain did NOT touch the data queue.
//!
//! Contracts, all `#[serial]` (the seam builds an iceoryx2 WaitSet over the
//! process-global shared-memory singleton):
//!
//! 1. **Positive** — after the out-of-graph publisher writes onto the
//!    consumer's trigger topic, the reactor's fired-set CONTAINS the consumer
//!    node id (its input `Listener` got a notification).
//! 2. **Negative** — on a quiet graph with NO new publish (after the build's
//!    connection-lifecycle events have been drained), the fired-set is EMPTY.
//! 3. **Determinism** — two independently-built graphs given identical
//!    treatment (drain connection noise → publish one frame → run reactor)
//!    yield the same fired-set, EXACTLY `["consumer"]` (one data-trigger
//!    input — hand-written oracle).
//! 4. **Firewall** — calling the seam does NOT advance the consumer's
//!    `fire_count`, AND the reactor's event-queue drain does NOT consume the
//!    SHM DATA sample (a subsequent real `step` still fires the consumer once).
//! 5. **Multiplexing (declaration order)** — a TWO-consumer graph with
//!    consumers declared REVERSE-alphabetically pins the fired-set to
//!    DECLARATION order: both-published → `["zeta", "alpha"]`, selective
//!    single-publish routes to exactly one consumer, deterministic across
//!    runs.
//! 6. **Empty graph** — a producer-only graph (no data-trigger bindings) hits
//!    the `sources.is_empty()` early-return: an EMPTY fired-set, no hang.
//!
//! ## On iceoryx2 event-listener semantics
//!
//! A trigger input's iceoryx2 event `Listener` multiplexes CONNECTION
//! lifecycle events (`SubscriberConnected` / `PublisherConnected`, queued at
//! build / on a publisher attach) alongside data `SentSample` events. A raw
//! WaitSet attachment wakes for ANY of them. So a freshly-built graph's
//! listener already has pending events BEFORE any data publish, and attaching
//! the out-of-graph publisher queues another — the reactor records the consumer
//! on the first cycle purely from connection noise. The reactor's callback
//! drains the listener's EVENT queue (the notification channel, not the SHM
//! data queue) after recording, so a quiet next cycle reports nothing.
//! Tests that need a clean baseline therefore PRIME the reactor once (draining
//! the build + attach noise) before asserting on a specific data publish.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::CerulionPublisher;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The single consumer's absolute external trigger topic — no in-graph
/// producer, so the graph provisions it `External` (buffer-ceiling only, no
/// single-writer cap) and the out-of-graph test publisher attaches freely.
const EXT_TOPIC: &str = "/wsr/ext/cam";

/// Period producer kept ONLY for the producer-only empty-graph test (contract
/// 6). The reactor-observation tests publish out-of-graph onto `EXT_TOPIC`
/// instead, so the consumer graphs carry no in-graph producer.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct WsProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl WsProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Data-trigger consumer: fires whenever its `inp` input receives data.
#[cerulion_node]
#[derive(Default)]
struct WsConsumer {
    #[input(trigger)]
    inp: Vector3,
    sum: f64,
}

#[cerulion_node_impl]
impl WsConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        Ok(())
    }
}

/// A data-trigger consumer with
/// `sample(2)` backpressure. `sample(N)`'s read-decimation gates the FIRE rate,
/// so the body-subscriber unification is INELIGIBLE — this consumer
/// keeps the legacy dual-subscriber path and so its live WaitSet wake source is
/// a `TriggerSubscriber::Ipc` (NOT the unified `ListenerOnly`). Co-locating it
/// with a plain (unified → `ListenerOnly`) `WsConsumer` on the SAME topic puts
/// BOTH `TriggerSubscriber` variants in one live source list — the case the
/// `.map`-over-both-variants source builder must handle. Replicated here
/// (rather than imported) because test binaries are separate crates.
#[cerulion_node]
#[derive(Default)]
struct WsSampleConsumer {
    #[input(trigger, backpressure = sample(2))]
    inp: Vector3,
    sum: f64,
}

#[cerulion_node_impl]
impl WsSampleConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        Ok(())
    }
}

/// Build a single data-trigger consumer subscribing the absolute external
/// topic `EXT_TOPIC`.
fn ws_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "waitset_reactor_test".to_string(),
        prefix: "wsr".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "consumer".to_string(),
            node_type: "ws_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: EXT_TOPIC.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("consumer".to_string(), Box::new(WsConsumerEntry::new()));
    (config, factories)
}

/// Build the graph over an isolated test transport.
fn build() -> GraphRuntime {
    let (config, factories) = ws_graph();
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("build waitset graph")
}

/// Attach an out-of-graph publisher on `topic` via the runtime's parked test
/// transport. The topic must be the consumer's absolute external trigger topic
/// (provisioned `External`, so no single-writer cap blocks this attach).
fn external_publisher(runtime: &GraphRuntime, topic: &str) -> CerulionPublisher {
    let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
    mgr.create_publisher(topic, MaxSliceLen::const_new(256), 0)
        .expect("external publisher must attach to the absolute external topic")
}

/// Drain the connection-lifecycle events (`SubscriberConnected` /
/// `PublisherConnected`, including the out-of-graph publisher's attach) queued
/// on the consumer's trigger listener, by running the reactor once and
/// discarding its result.
fn prime_drain_connection_noise(runtime: &mut GraphRuntime) {
    let _ = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(200));
}

/// Publish exactly ONE frame onto `pubr`'s topic (the proxy publishes on drop).
/// This leaves a fresh `SentSample` notification on the consumer's trigger
/// `Listener` AND the data sample on its SHM queue — neither consumed, because
/// the consumer's level is never drained/fired here (we do NOT call `step()`).
fn publish_one(pubr: &mut CerulionPublisher, x: f64) {
    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
    proxy.x = x;
    drop(proxy); // publish
}

#[test]
#[serial]
fn reactor_records_consumer_when_trigger_topic_published() {
    let mut runtime = build();
    // Hold the out-of-graph publisher for the whole test so its attach event is
    // a one-time priming cost, not a per-publish disconnect/reconnect churn.
    let mut pubr = external_publisher(&runtime, EXT_TOPIC);
    // Clear the build + attach connection-lifecycle events so the assertion
    // below is attributable to the data publish, not connection noise.
    prime_drain_connection_noise(&mut runtime);

    // Publish onto the consumer's trigger topic WITHOUT stepping the graph —
    // the consumer's level never drains, so the `SentSample` stays pending for
    // the reactor.
    publish_one(&mut pubr, 1.0);

    let fired = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(200));
    assert!(
        fired.iter().any(|id| id == "consumer"),
        "reactor should record the data-trigger consumer after its trigger \
         topic was published; fired = {fired:?}"
    );
}

#[test]
#[serial]
fn reactor_records_nothing_on_quiet_graph() {
    let mut runtime = build();
    // Attach (and hold) the out-of-graph publisher, then drain its attach event
    // plus the build's connection events.
    let _pubr = external_publisher(&runtime, EXT_TOPIC);
    // First cycle records the consumer from the build/attach connection events
    // AND drains them.
    prime_drain_connection_noise(&mut runtime);

    // No new publish. The listener's event queue is now empty, so a fresh
    // short-timeout cycle records NOTHING.
    let fired = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(50));
    assert!(
        fired.is_empty(),
        "on a quiet graph (build noise drained, no new publish) the reactor \
         must record an EMPTY fired-set; fired = {fired:?}"
    );
}

/// Build a fresh graph, attach + drain connection noise, publish one frame,
/// then run the reactor once — returning its fired-set. The determinism oracle.
fn drive_one_publish_cycle() -> Vec<String> {
    let mut runtime = build();
    let mut pubr = external_publisher(&runtime, EXT_TOPIC);
    prime_drain_connection_noise(&mut runtime);
    publish_one(&mut pubr, 1.0);
    runtime.run_waitset_reactor_once_for_test(Duration::from_millis(200))
}

#[test]
#[serial]
fn reactor_is_deterministic_across_runs() {
    // Two independent builds given identical treatment yield the same
    // fired-set — bit-identical dispatch (Principle #7).
    let first = drive_one_publish_cycle();
    let second = drive_one_publish_cycle();
    assert_eq!(
        first, second,
        "two independently-built graphs given identical treatment must yield \
         the same fired-set"
    );
    // Hand-written exact oracle: this graph has exactly ONE data-trigger input
    // (`consumer.inp`), so the fired-set is provably the singleton
    // `["consumer"]`. Pinning the exact vector (not a weak `contains` check)
    // kills a regression that would record extra/wrong ids or reorder.
    assert_eq!(
        first,
        vec!["consumer".to_string()],
        "the fired-set must be exactly [\"consumer\"] (one data-trigger input); \
         first = {first:?}"
    );
}

#[test]
#[serial]
fn reactor_does_not_fire_the_consumer_firewall() {
    let mut runtime = build();
    // Hold the publisher across BOTH reactor calls and the subsequent step so
    // the SHM data sample stays delivered (drop adds no further sample).
    let mut pubr = external_publisher(&runtime, EXT_TOPIC);
    prime_drain_connection_noise(&mut runtime);
    publish_one(&mut pubr, 1.0);

    // Snapshot the consumer's fire count BEFORE the reactor runs. Nothing has
    // fired the consumer yet (we published out-of-graph, never stepped), so
    // this is its baseline.
    let before = runtime
        .node_handle("consumer")
        .expect("consumer handle")
        .fire_count();

    // Run the reactor twice — record-only, must not fire the consumer.
    let _ = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(200));
    let _ = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(200));

    let after = runtime
        .node_handle("consumer")
        .expect("consumer handle")
        .fire_count();

    assert_eq!(
        before, after,
        "FIREWALL: run_waitset_reactor_once_for_test must NOT advance the \
         consumer's fire_count (the reactor records, it does not fire)"
    );

    // LOAD-BEARING event-drain-≠-data-consume proof: the reactor drained the
    // listener's EVENT queue (the wakeup notification), but it must NOT have
    // consumed the SHM DATA sample. Prove that by stepping the real graph: the
    // deterministic `drain_level`/`try_receive` path still finds the queued
    // frame and fires the consumer exactly once. If the reactor had consumed
    // the data sample, the frame would be gone and the consumer would NOT fire
    // here (after_step == before).
    runtime.step(Duration::from_millis(10));
    let after_step = runtime
        .node_handle("consumer")
        .expect("consumer handle")
        .fire_count();
    assert_eq!(
        after_step,
        before + 1,
        "the reactor's event-queue drain must NOT consume the SHM DATA sample: \
         the queued frame must survive for the real `drain_level`, firing \
         the consumer exactly once (before={before}, after_step={after_step})"
    );
}

// ---------------------------------------------------------------------------
// Multi-consumer multiplexing.
//
// The reactor's core value-add — multiplexing N listeners and routing each
// fired attachment back to the right source via the `BTreeMap` — was only
// exercised at N=1 above. These types build a TWO-consumer graph whose
// consumers are declared in REVERSE-alphabetical order (`zeta` BEFORE
// `alpha`) so the fired-set ordering pins DECLARATION order (source index),
// not name order. A sort-by-NAME regression would surface as `["alpha",
// "zeta"]` instead of the contract `["zeta", "alpha"]`.
//
// Each consumer subscribes a DISTINCT absolute external topic, so out-of-graph
// publishes can target one or both independently — the lever for both the
// completeness/order contract and the selective single-publish routing
// contract. (No in-graph producers: the reactor only observes the consumers'
// listeners, and out-of-graph publishes set up every observation.)
// ---------------------------------------------------------------------------

/// The two consumers' distinct absolute external trigger topics.
const EXT_TOPIC_ZETA: &str = "/wsm/ext/zeta";
const EXT_TOPIC_ALPHA: &str = "/wsm/ext/alpha";

/// Build a two-consumer graph. Node order (and hence `data_trigger_bindings`
/// / source-index order) is:
///   1. `zeta`  (data-trigger consumer of `EXT_TOPIC_ZETA`,  source index 0)
///   2. `alpha` (data-trigger consumer of `EXT_TOPIC_ALPHA`, source index 1)
///
/// Consumers are declared REVERSE-alphabetically (`zeta` before `alpha`) so
/// declaration order ≠ name order — the lever that kills a sort-by-name
/// regression.
fn ws_multi_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "waitset_reactor_multi_test".to_string(),
        prefix: "wsm".to_string(),
        nodes: vec![
            // REVERSE-alphabetical declaration: `zeta` first (source index 0),
            // `alpha` second (source index 1).
            NodeDef {
                fuse: None,
                ros2: None,
                id: "zeta".to_string(),
                node_type: "ws_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: EXT_TOPIC_ZETA.to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "alpha".to_string(),
                node_type: "ws_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: EXT_TOPIC_ALPHA.to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("zeta".to_string(), Box::new(WsConsumerEntry::new()));
    factories.insert("alpha".to_string(), Box::new(WsConsumerEntry::new()));
    (config, factories)
}

/// Build the multi-consumer graph over an isolated test transport.
fn build_multi() -> GraphRuntime {
    let (config, factories) = ws_multi_graph();
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("build multi waitset graph")
}

#[test]
#[serial]
fn reactor_multiplexes_two_consumers_in_declaration_order() {
    // ---- Contract (a): BOTH published → completeness + ORDER. ----
    let mut runtime = build_multi();
    let mut pub_zeta = external_publisher(&runtime, EXT_TOPIC_ZETA);
    let mut pub_alpha = external_publisher(&runtime, EXT_TOPIC_ALPHA);
    prime_drain_connection_noise(&mut runtime);

    // Publish onto BOTH consumers' trigger topics without stepping — both
    // listeners hold a fresh `SentSample`, neither consumer-side `drain_level`
    // has run, so both notifications stay pending for the reactor.
    publish_one(&mut pub_zeta, 1.0);
    publish_one(&mut pub_alpha, 2.0);

    let fired = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(200));
    assert_eq!(
        fired,
        vec!["zeta".to_string(), "alpha".to_string()],
        "both consumers' trigger topics were published, so the fired-set must \
         be EXACTLY [\"zeta\", \"alpha\"] in DECLARATION order (source index); \
         a sort-by-name regression would yield [\"alpha\", \"zeta\"]. \
         fired = {fired:?}"
    );

    // ---- Contract (b): selective single-publish → routing. ----
    // Fresh build. Publish onto ONLY `zeta`'s trigger topic, so ONLY `zeta`'s
    // listener holds a `SentSample`. The reactor must route that single
    // notification back to `zeta` alone.
    let mut runtime = build_multi();
    let mut pub_zeta = external_publisher(&runtime, EXT_TOPIC_ZETA);
    let _pub_alpha = external_publisher(&runtime, EXT_TOPIC_ALPHA);
    prime_drain_connection_noise(&mut runtime);
    publish_one(&mut pub_zeta, 1.0);

    let fired = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(200));
    assert_eq!(
        fired,
        vec!["zeta".to_string()],
        "only `zeta`'s topic was published, so the fired-set must be EXACTLY \
         [\"zeta\"]. fired = {fired:?}"
    );
}

/// Drive the multi-consumer graph through the contract-(a) both-published
/// cycle and return the fired-set. The determinism oracle for the
/// multiplexing path.
fn drive_multi_both_publish_cycle() -> Vec<String> {
    let mut runtime = build_multi();
    let mut pub_zeta = external_publisher(&runtime, EXT_TOPIC_ZETA);
    let mut pub_alpha = external_publisher(&runtime, EXT_TOPIC_ALPHA);
    prime_drain_connection_noise(&mut runtime);
    publish_one(&mut pub_zeta, 1.0);
    publish_one(&mut pub_alpha, 2.0);
    runtime.run_waitset_reactor_once_for_test(Duration::from_millis(200))
}

#[test]
#[serial]
fn reactor_multiplexing_is_deterministic_across_runs() {
    // ---- Contract (c): determinism. ----
    // Two independent builds given identical treatment yield the same exact
    // ordered fired-set (Principle #7).
    let first = drive_multi_both_publish_cycle();
    let second = drive_multi_both_publish_cycle();
    assert_eq!(
        first, second,
        "two independently-built multi-consumer graphs given identical \
         treatment must yield the same exact ordered fired-set"
    );
    assert_eq!(
        first,
        vec!["zeta".to_string(), "alpha".to_string()],
        "the ordered fired-set must be exactly [\"zeta\", \"alpha\"]; \
         first = {first:?}"
    );
}

// ---------------------------------------------------------------------------
// Empty-graph early-return.
//
// A producer-ONLY graph has no data-trigger bindings, so the seam builds an
// empty `sources` slice and `run_once` must hit the `sources.is_empty()`
// early-return: returning an EMPTY Vec WITHOUT calling
// `wait_and_process_once_with_timeout` (which would reject the
// zero-attachment WaitSet with `WaitSetRunError::NoAttachments`). If that
// early-return were broken the test would hang — a CI-visible failure.
// ---------------------------------------------------------------------------

/// Build a producer-ONLY graph (a lone period producer, no consumer).
fn ws_producer_only_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "waitset_reactor_empty_test".to_string(),
        prefix: "wse".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "producer".to_string(),
            node_type: "ws_producer".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(WsProducerEntry::new()));
    (config, factories)
}

#[test]
#[serial]
fn reactor_returns_empty_on_producer_only_graph() {
    let (config, factories) = ws_producer_only_graph();
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build producer-only graph");

    // No data-trigger bindings → empty `sources` → `sources.is_empty()`
    // early-return. A normal timeout is fine: a correct early-return returns
    // IMMEDIATELY with an empty Vec; a broken one would reach
    // `wait_and_process_*` on a zero-attachment WaitSet and the test would
    // hang (CI-visible).
    let fired = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(50));
    assert!(
        fired.is_empty(),
        "a producer-only graph has no data-trigger inputs, so the reactor must \
         return an EMPTY fired-set; fired = {fired:?}"
    );
}

// ---------------------------------------------------------------------------
// BOTH `TriggerSubscriber` variants
// coexist in ONE live WaitSet source list.
//
// The source builder uses a `.map`, not a `filter_map` (which would drop UNIFIED
// bindings entirely), over BOTH `TriggerSubscriber` variants —
// `ListenerOnly` (a unified `drop_oldest` trigger input, whose body subscriber
// absorbed the trigger-drain sub, leaving a standalone listener as wake source)
// and `Ipc` (an INELIGIBLE input — here `sample(2)`, which keeps the legacy
// dual-subscriber path so its trigger-drain sub IS the wake source). This test
// builds TWO consumers on the SAME external trigger topic — one of each variant —
// and pins that BOTH appear in the reactor's FIRED-SET after a single publish.
//
// Why the reactor harness (not the e2e `live_step`): the reactor's fired-set is
// the set of sources that ACTUALLY fired the WaitSet — a timing-independent
// membership oracle. A source dropped from the live list simply will NOT appear
// in the fired-set, so dropping EITHER variant fails this test. (The e2e
// `polled_vs_live_iox2_test::live_step_wakes_both_unified_and_ipc_sources` only
// asserts fire-counts, which `drain_level` drains every step regardless of which
// source woke the loop — so a single dropped variant still fires via the
// heartbeat fallback and goes undetected there. This is the genuine structural
// per-variant pin.)
// ---------------------------------------------------------------------------

/// The shared absolute external trigger topic both mixed-variant consumers
/// subscribe (no in-graph producer → provisioned `External`, so an out-of-graph
/// publisher attaches freely AND two consumers may share it).
const EXT_TOPIC_MIXED: &str = "/wsx/ext/cam";

/// Build a two-consumer graph on the SAME external trigger topic:
///   1. `unified` — plain `#[input(trigger)]` (drop_oldest → UNIFIED →
///      `TriggerSubscriber::ListenerOnly` wake source).
///   2. `sampled` — `#[input(trigger, backpressure = sample(2))]` (INELIGIBLE →
///      legacy dual-subscriber → `TriggerSubscriber::Ipc` wake source).
///
/// So the live WaitSet source list carries BOTH variants — the
/// `.map`-over-both dispatch under test.
fn ws_mixed_variant_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "waitset_reactor_mixed_test".to_string(),
        prefix: "wsx".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "unified".to_string(),
                node_type: "ws_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: EXT_TOPIC_MIXED.to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sampled".to_string(),
                node_type: "ws_sample_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: EXT_TOPIC_MIXED.to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("unified".to_string(), Box::new(WsConsumerEntry::new()));
    factories.insert(
        "sampled".to_string(),
        Box::new(WsSampleConsumerEntry::new()),
    );
    (config, factories)
}

#[test]
#[serial]
fn reactor_records_both_unified_and_ipc_sources() {
    let (config, factories) = ws_mixed_variant_graph();
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build mixed-variant waitset graph");

    // Hold the out-of-graph publisher across the whole test so its attach event
    // is a one-time priming cost, not per-publish churn.
    let mut pubr = external_publisher(&runtime, EXT_TOPIC_MIXED);
    // Drain the build + attach connection-lifecycle events so the assertion below
    // is attributable to the data publish, not connection noise.
    prime_drain_connection_noise(&mut runtime);

    // Publish ONE frame onto the shared trigger topic WITHOUT stepping the graph:
    // BOTH consumers' listeners hold a fresh `SentSample`, and neither level is
    // drained, so both notifications stay pending for the reactor.
    publish_one(&mut pubr, 1.0);

    let fired = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(200));

    // STRUCTURAL per-variant pin: BOTH the unified (`ListenerOnly`) and sampled
    // (`Ipc`) sources are present in the live source list, so BOTH appear in the
    // fired-set. Dropping EITHER variant from the source builder's `.map` removes that
    // consumer's source → it is absent from the fired-set → this fails. `contains`
    // (not an ordered exact-vector check) because the fired-set's ordering across
    // distinct `TriggerSubscriber` variants is not a contract this test pins —
    // membership of both variants is.
    assert!(
        fired.iter().any(|id| id == "unified"),
        "the UNIFIED (ListenerOnly) source must be in the live source list and \
         fire — fired = {fired:?}"
    );
    assert!(
        fired.iter().any(|id| id == "sampled"),
        "the INELIGIBLE (Ipc) source must be in the live source list and fire — \
         dropping it from the source builder's `.map` over both variants makes it absent \
         from the fired-set; fired = {fired:?}"
    );
}
