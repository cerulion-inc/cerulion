// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end proof of the REAL liveliness producer
//! (graceful-disconnect path) over real iceoryx2 via
//! `GraphRuntime::build_for_test` (per-test SHM root — parallel-safe; serial
//! iceoryx2 tests).
//!
//! The runtime liveliness SWEEP (`GraphRuntime::liveliness_sweep`, driven from
//! `step()`) polls each watched input topic's live publisher count and, on a
//! transition, mints a `LivelinessEvent` into the consuming node's
//! `QosEventStore` so the `#[on_event(input = "...")]`
//! `LivelinessEvent` handler fires. It also bumps the per-node
//! `NodeHandle::publisher_disconnects_observed_count` on every `Lost` edge.
//!
//! This file covers the GRACEFUL disconnect path only: a same-process external
//! publisher's `Drop` decrements iceoryx2's dynamic publisher count, which the
//! sweep observes as a `Lost` transition. The true-CRASH path (forked child +
//! `try_cleanup_dead_nodes`) lives in `liveliness_crash_iox2_test.rs` — NOT covered here.
//!
//! Determinism contract (LIVE-ONLY observe + record): the sweep cadence and
//! each event's `changed_at_ns` are deterministic (sim-clock driven), so a
//! controlled-drop-timing run is reproducible. The publisher-count OBSERVATION
//! itself is live-only / not bit-reproducible from a free re-run — the tests
//! pin ONLY what IS deterministic (controlled attach/drop at the same step).
//!
//! No fake data (Principle #13): the consumer's handler records REAL handler
//! execution into shared atomics; the publishers are REAL iceoryx2 ports whose
//! attach/drop the sweep observes via `number_of_publishers()`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Shared observation surface the consumer's handler writes and the test reads.
#[derive(Default, cerulion_core::state::CerulionState)]
struct LiveObs {
    /// Number of `Lost` (`PublisherDisconnected`) handler firings.
    lost_fires: AtomicU64,
    /// Number of `Alive` (`PublisherConnected`) handler firings.
    alive_fires: AtomicU64,
    /// `publisher_count` from the most recent event (1 + state code so the
    /// test can read back the last edge: see `last_state` for the discriminant).
    last_publisher_count: AtomicU64,
    /// `count_total` from the most recent event.
    last_count_total: AtomicU64,
    /// `changed_at_ns` from the most recent event.
    last_changed_at_ns: AtomicU64,
}

/// Periodic (5 ms) consumer of an absolute external topic with a generic
/// `LivelinessEvent` handler that records (not asserts) the edge — so it works
/// for BOTH `Lost` and `Alive` transitions minted by the real sweep.
///
/// Periodic (not data-trigger) on purpose: the `#[on_event]` dispatch runs on
/// the tick Ok-path, so the node must tick EVERY step to drain a pending
/// liveliness event — a data-trigger node would never tick (and so never
/// dispatch the `Lost` handler) once its publisher drops and data stops.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct LiveConsumer {
    #[input]
    inp: Vector3,
    sum: f64,
    obs: Arc<LiveObs>,
}

#[cerulion_node_impl]
impl LiveConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        Ok(())
    }

    /// Routed to `take_liveliness_event` by the `LivelinessEvent` param type.
    #[on_event(input = "inp")]
    fn on_live(&mut self, event: LivelinessEvent) {
        assert_eq!(
            event.input_name.as_ref(),
            "inp",
            "the event must carry its input name"
        );
        self.obs
            .last_publisher_count
            .store(event.publisher_count as u64, Ordering::Relaxed);
        self.obs
            .last_count_total
            .store(event.count_total, Ordering::Relaxed);
        self.obs
            .last_changed_at_ns
            .store(event.changed_at_ns, Ordering::Relaxed);
        match event.state {
            LivelinessState::Lost => {
                assert_eq!(
                    event.cause,
                    LivelinessCause::PublisherDisconnected,
                    "Lost must carry PublisherDisconnected"
                );
                assert_eq!(event.publisher_count, 0, "Lost means no publishers remain");
                self.obs.lost_fires.fetch_add(1, Ordering::Relaxed);
            }
            LivelinessState::Alive => {
                assert_eq!(
                    event.cause,
                    LivelinessCause::PublisherConnected,
                    "Alive must carry PublisherConnected"
                );
                assert!(
                    event.publisher_count >= 1,
                    "Alive means at least one publisher present"
                );
                self.obs.alive_fires.fetch_add(1, Ordering::Relaxed);
            }
            _ => unreachable!("only Lost/Alive exist"),
        }
    }
}

/// A single-consumer graph whose ONLY input is an absolute external topic
/// (`/live/cam`) — no in-graph producer, so the test attaches/drops external
/// publishers to drive real liveliness transitions.
fn live_graph(obs: Arc<LiveObs>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "liveliness_sweep_test".to_string(),
        prefix: "lsw".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "cons".to_string(),
            node_type: "live_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/live/cam".to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "cons".to_string(),
        Box::new(LiveConsumerEntry::with_state(LiveConsumer {
            obs,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// Build the graph with a shortened (5 ms) liveliness sweep cadence so a single
/// `step_ms(5)` runs exactly one sweep.
fn build_runtime(obs: Arc<LiveObs>) -> GraphRuntime {
    let (config, factories) = live_graph(obs);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build liveliness graph");
    // 5 ms cadence: one sweep per 5 ms step.
    runtime.set_liveliness_sweep_period_for_test(5);
    runtime
}

/// `step()` order is: the level executor drains each level's trigger inputs
/// (`drain_level`) and fires that level (ticking the node, draining any PENDING
/// liveliness event), then → liveliness_sweep (detects a transition and
/// PUSHES the event). So a transition the sweep pushes on step N is dispatched
/// by the tick on step N+1. This helper does the two steps: the first lets the
/// sweep observe + push the transition, the second lets the next tick drain it.
/// (The second sweep sees no NEW transition, so it pushes nothing.)
fn observe_transition(runtime: &mut GraphRuntime) {
    runtime.step_ms(5); // sweep observes the transition, pushes the event
    runtime.step_ms(5); // next tick drains + dispatches the pushed event
}

/// Attach an external publisher to `/live/cam` via the runtime's parked test
/// transport.
fn attach_publisher(
    runtime: &GraphRuntime,
) -> cerulion_core::transport::publisher::CerulionPublisher {
    runtime
        .test_transport()
        .expect("test transport parked")
        .create_publisher("/live/cam", MaxSliceLen::const_new(256), 0)
        .expect("external publisher attaches to /live/cam")
}

#[test]
#[serial]
fn lost_fires_on_graceful_drop() {
    // Controlled sequence: build (baseline 0) → attach publisher → sweep
    // (Alive) → drop publisher → sweep (Lost). Exactly one Lost handler fire;
    // the per-node disconnect counter == 1.
    let obs = Arc::new(LiveObs::default());
    let mut runtime = build_runtime(Arc::clone(&obs));

    let pubr = attach_publisher(&runtime);
    // Sweep observes the publisher present (Alive edge — baseline was 0); the
    // second step's tick drains the pushed event.
    observe_transition(&mut runtime);
    assert_eq!(
        obs.alive_fires.load(Ordering::Relaxed),
        1,
        "attaching a publisher after a 0-baseline build is an Alive transition"
    );
    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        0,
        "no Lost yet — the publisher is still attached"
    );

    // Drop the publisher: the next sweep observes the Lost edge.
    drop(pubr);
    observe_transition(&mut runtime);

    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        1,
        "dropping the last publisher must fire exactly one Lost handler"
    );
    assert_eq!(
        obs.last_publisher_count.load(Ordering::Relaxed),
        0,
        "the Lost event reports zero live publishers"
    );
    let handle = runtime.node_handle("cons").expect("consumer node handle");
    assert_eq!(
        handle.publisher_disconnects_observed_count(),
        1,
        "the per-node disconnect counter bumps once on the Lost transition"
    );
}

#[test]
#[serial]
fn quiet_while_publisher_alive() {
    // With a publisher attached and never dropped, many sweeps fire EXACTLY
    // one Alive (the baseline-0 → present edge) and ZERO Lost; the disconnect
    // counter stays 0.
    let obs = Arc::new(LiveObs::default());
    let mut runtime = build_runtime(Arc::clone(&obs));
    let _pubr = attach_publisher(&runtime);

    for _ in 0..20 {
        runtime.step_ms(5);
    }

    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        0,
        "no Lost while the publisher stays alive"
    );
    assert_eq!(
        obs.alive_fires.load(Ordering::Relaxed),
        1,
        "exactly one Alive edge (the initial attach), no re-fires while steady"
    );
    let handle = runtime.node_handle("cons").expect("consumer node handle");
    assert_eq!(
        handle.publisher_disconnects_observed_count(),
        0,
        "the disconnect counter stays 0 while the publisher is alive"
    );
}

#[test]
#[serial]
fn reattach_fires_alive_through_real_producer() {
    // Drive a full Lost → Alive cycle through the REAL sweep (no injection):
    // attach → Alive → drop → Lost → re-attach → Alive again.
    let obs = Arc::new(LiveObs::default());
    let mut runtime = build_runtime(Arc::clone(&obs));

    let pubr = attach_publisher(&runtime);
    observe_transition(&mut runtime); // Alive #1
    drop(pubr);
    observe_transition(&mut runtime); // Lost #1
    assert_eq!(obs.lost_fires.load(Ordering::Relaxed), 1, "one Lost so far");

    // A NEW external publisher re-appears: the next sweep fires Alive again.
    let _pubr2 = attach_publisher(&runtime);
    observe_transition(&mut runtime); // Alive #2

    assert_eq!(
        obs.alive_fires.load(Ordering::Relaxed),
        2,
        "re-attaching a publisher fires a SECOND Alive edge through the real producer"
    );
    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        1,
        "still exactly one Lost (the single drop)"
    );
    // The Alive event reports the live publisher count and the cumulative
    // transition count (Alive#1, Lost#1, Alive#2 = 3).
    assert_eq!(
        obs.last_publisher_count.load(Ordering::Relaxed),
        1,
        "the re-attach Alive event reports one live publisher"
    );
    assert_eq!(
        obs.last_count_total.load(Ordering::Relaxed),
        3,
        "cumulative transition count is 3 (Alive, Lost, Alive)"
    );
}

/// Run the controlled attach→Alive→drop→Lost sequence and return
/// `(alive_fires, lost_fires, last_changed_at_ns, disconnects)`.
fn run_controlled_cycle() -> (u64, u64, u64, u64) {
    let obs = Arc::new(LiveObs::default());
    let mut runtime = build_runtime(Arc::clone(&obs));
    let pubr = attach_publisher(&runtime);
    observe_transition(&mut runtime); // Alive: sweep at 5 ms, drained at 10 ms
    drop(pubr);
    observe_transition(&mut runtime); // Lost: sweep at 15 ms, drained at 20 ms
    let handle = runtime.node_handle("cons").expect("consumer node handle");
    (
        obs.alive_fires.load(Ordering::Relaxed),
        obs.lost_fires.load(Ordering::Relaxed),
        obs.last_changed_at_ns.load(Ordering::Relaxed),
        handle.publisher_disconnects_observed_count(),
    )
}

#[test]
#[serial]
fn controlled_timing_is_deterministic() {
    // The cadence + `changed_at_ns` are sim-clock driven, so two runs with the
    // drop at the SAME controlled step yield identical fire counts AND an
    // identical `changed_at_ns` (the deterministic part of the live-only
    // contract). The Lost transition is observed at the 15 ms sweep (attach
    // Alive at 5 ms, drained at 10 ms; drop Lost at 15 ms), so the LAST event
    // (Lost) carries `changed_at_ns == 15_000_000`.
    let a = run_controlled_cycle();
    let b = run_controlled_cycle();
    assert_eq!(
        a, b,
        "identical controlled sequence ⇒ identical observation"
    );
    // Anchor to the known values (not a tautological a==b on scalars a
    // regression-to-constant would also satisfy).
    assert_eq!(a.0, 1, "exactly one Alive");
    assert_eq!(a.1, 1, "exactly one Lost");
    assert_eq!(
        a.2, 15_000_000,
        "the Lost edge's changed_at_ns is the 15 ms sweep's sim-clock (deterministic)"
    );
    assert_eq!(a.3, 1, "exactly one disconnect observed");
}

// ===========================================================================
// (G) Internal producer→consumer edge, CONSUMER declared FIRST — no spurious
//     Alive (the regression test for the post-build baseline re-read).
// ===========================================================================

/// In-graph periodic producer feeding an internal topic so the consumer's edge
/// has a real, build-time-present publisher.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct InternalProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl InternalProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

#[test]
#[serial]
fn internal_edge_consumer_first_no_spurious_alive() {
    // PINS the baseline re-read: an INTERNAL producer→consumer edge where the CONSUMER is
    // declared BEFORE the producer in `config.nodes`. `config.nodes` has no
    // topological sort and each node attaches its own publisher during its own
    // build iteration, so a build-loop-time baseline read for the consumer
    // would observe 0 publishers (the producer's publisher is not attached
    // until its later iteration) and the first sweep would then see count=1 →
    // a SPURIOUS `Alive`. With the baseline re-read at the post-build
    // finalize pass (when every producer is attached) the baseline is already
    // 1, so NO Alive edge fires. Without the re-seed this asserts alive==0
    // against an observed alive==1, and fails.
    let obs = Arc::new(LiveObs::default());

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "liveliness_internal_order_test".to_string(),
        prefix: "lio".to_string(),
        // NOTE: consumer "cons" declared BEFORE producer "prod" — the whole
        // point of this test (reverse of topological order).
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "cons".to_string(),
                node_type: "live_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "prod/out".to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                ros2: None,
                id: "prod".to_string(),
                node_type: "internal_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("prod".to_string(), Box::new(InternalProducerEntry::new()));
    factories.insert(
        "cons".to_string(),
        Box::new(LiveConsumerEntry::with_state(LiveConsumer {
            obs: Arc::clone(&obs),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build internal-edge graph");
    runtime.set_liveliness_sweep_period_for_test(5);

    // Run several sweeps. The producer is present at build (just declared
    // later), so the baseline is 1 and there is NEVER a transition.
    for _ in 0..10 {
        runtime.step_ms(5);
    }

    assert_eq!(
        obs.alive_fires.load(Ordering::Relaxed),
        0,
        "no spurious Alive — the internal producer was present at build, only \
         declared after the consumer (baseline re-read post-build)"
    );
    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        0,
        "the producer never drops, so no Lost"
    );
    let handle = runtime.node_handle("cons").expect("consumer node handle");
    assert_eq!(
        handle.publisher_disconnects_observed_count(),
        0,
        "no disconnect observed"
    );
}

// ===========================================================================
// (H) Data-trigger consumer: the Lost HANDLER does NOT fire (no tick) but the
//     disconnect COUNTER does (the documented dispatch limitation).
// ===========================================================================

/// Data-triggered consumer of the external `/live/cam` topic with the same
/// `LivelinessEvent` handler. Unlike `LiveConsumer` (period), this fires its
/// tick ONLY on data arrival — so once the publisher drops and data stops, the
/// node never ticks and its `Lost` handler never dispatches. The runtime sweep
/// still bumps the per-node disconnect COUNTER regardless of whether it ticks.
#[cerulion_node]
#[derive(Default)]
struct DataTriggerLiveConsumer {
    #[input(trigger)]
    inp: Vector3,
    sum: f64,
    obs: Arc<LiveObs>,
}
#[cerulion_node_impl]
impl DataTriggerLiveConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        Ok(())
    }

    #[on_event(input = "inp")]
    fn on_live(&mut self, event: LivelinessEvent) {
        match event.state {
            LivelinessState::Lost => {
                self.obs.lost_fires.fetch_add(1, Ordering::Relaxed);
            }
            LivelinessState::Alive => {
                self.obs.alive_fires.fetch_add(1, Ordering::Relaxed);
            }
            _ => unreachable!("only Lost/Alive exist"),
        }
    }
}

#[test]
#[serial]
fn data_trigger_lost_handler_silent_but_counter_fires() {
    // Pins the documented limitation: a data-trigger node stops ticking when its
    // input goes silent, so its `Lost` HANDLER does NOT dispatch — but the
    // always-on observable, `publisher_disconnects_observed_count`, DOES bump
    // because the sweep increments it regardless of whether the node ticks.
    let obs = Arc::new(LiveObs::default());

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "liveliness_data_trigger_test".to_string(),
        prefix: "ldt".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "cons".to_string(),
            node_type: "data_trigger_live_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/live/cam".to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "cons".to_string(),
        Box::new(DataTriggerLiveConsumerEntry::with_state(
            DataTriggerLiveConsumer {
                obs: Arc::clone(&obs),
                ..Default::default()
            },
        )),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build data-trigger graph");
    runtime.set_liveliness_sweep_period_for_test(5);

    // Attach a publisher after build (finalize baseline = 0), so the first sweep
    // fires an Alive edge (0 → 1) which this test does NOT assert. The point here
    // is the LOST path: publish a couple frames so the data-trigger node ticks at
    // least once (proving the dispatch path is wired), then DROP the publisher.
    // A data-trigger node stops ticking once its data stops, so its Lost handler
    // never dispatches (lost_fires == 0) — yet the sweep's disconnect COUNTER
    // still bumps (the always-on observable, independent of the tick).
    let mut pubr = attach_publisher(&runtime);
    let publish = |p: &mut cerulion_core::transport::publisher::CerulionPublisher, v: f64| {
        let mut proxy = p.loan_proxy::<Vector3>().expect("loan");
        proxy.x = v;
        drop(proxy);
    };
    publish(&mut pubr, 1.0);
    runtime.step_ms(5); // sweep sees Alive edge (baseline 0 → 1) + node ticks on data
    publish(&mut pubr, 2.0);
    runtime.step_ms(5); // node ticks again on data; drains any pending event

    // DROP the publisher: data stops, so the data-trigger node never ticks
    // again. Pump several sweeps — each bumps the disconnect counter on the
    // Lost edge it observes (only the FIRST sweep sees the transition; later
    // sweeps see steady count==0, no further bump).
    drop(pubr);
    for _ in 0..6 {
        runtime.step_ms(5);
    }

    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        0,
        "no data ⇒ no tick ⇒ the Lost HANDLER never dispatches (documented limitation)"
    );
    let handle = runtime.node_handle("cons").expect("consumer node handle");
    assert_eq!(
        handle.publisher_disconnects_observed_count(),
        1,
        "the sweep bumps the disconnect COUNTER exactly once regardless of ticking"
    );
}

// ===========================================================================
// (I) Multi-transition counter accuracy across attach→drop→reattach→drop.
// ===========================================================================

#[test]
#[serial]
fn multi_transition_counter_accuracy() {
    // attach→Alive→drop→Lost→reattach→Alive→drop→Lost over the REAL sweep on a
    // PERIOD consumer (so every edge's handler dispatches). Pin every counter.
    let obs = Arc::new(LiveObs::default());
    let mut runtime = build_runtime(Arc::clone(&obs));

    let pubr1 = attach_publisher(&runtime);
    observe_transition(&mut runtime); // Alive #1 (count_total = 1)
    drop(pubr1);
    observe_transition(&mut runtime); // Lost  #1 (count_total = 2)
    let pubr2 = attach_publisher(&runtime);
    observe_transition(&mut runtime); // Alive #2 (count_total = 3)
    drop(pubr2);
    observe_transition(&mut runtime); // Lost  #2 (count_total = 4)

    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        2,
        "two drops ⇒ two Lost handler fires"
    );
    assert_eq!(
        obs.alive_fires.load(Ordering::Relaxed),
        2,
        "two attaches ⇒ two Alive handler fires"
    );
    let handle = runtime.node_handle("cons").expect("consumer node handle");
    assert_eq!(
        handle.publisher_disconnects_observed_count(),
        2,
        "the disconnect counter bumps once per Lost ⇒ 2"
    );
    // The LAST observed event is Lost #2: A=1, L=2, A=3, L=4 ⇒ count_total = 4.
    assert_eq!(
        obs.last_count_total.load(Ordering::Relaxed),
        4,
        "cumulative transition count at the final (Lost #2) edge is 4"
    );
}

// ===========================================================================
// (J) Multi-publisher latch: `Lost` fires only when the LAST publisher leaves.
// ===========================================================================

#[test]
#[serial]
fn multi_publisher_lost_only_on_last_leave() {
    // Two external publishers on the same external topic /live/cam. With NO
    // in-graph producer the topic takes the External provisioning arm, which uses
    // iceoryx2's create-default publisher ceiling (2) — so both attach WITHOUT a
    // `multi_publisher_topics` opt-in (that opt-in admits multiple IN-GRAPH
    // producers; listing a producer-less external topic is a no-op). The sweep
    // observes Alive with publisher_count == 2. Dropping ONE leaves a publisher
    // (count 2 → 1, still alive): NO Lost, counter stays 0. Dropping the SECOND
    // (count 1 → 0) fires exactly one Lost — the latch only releases when the
    // LAST publisher leaves.
    let obs = Arc::new(LiveObs::default());

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "liveliness_multi_pub_test".to_string(),
        prefix: "lmp".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "cons".to_string(),
            node_type: "live_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/live/cam".to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "cons".to_string(),
        Box::new(LiveConsumerEntry::with_state(LiveConsumer {
            obs: Arc::clone(&obs),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build multi-publisher liveliness graph");
    runtime.set_liveliness_sweep_period_for_test(5);

    // Two external publishers attach to the opted-in topic.
    let pub_a = attach_publisher(&runtime);
    let pub_b = attach_publisher(&runtime);
    observe_transition(&mut runtime); // Alive edge: count 0 → 2

    assert_eq!(
        obs.alive_fires.load(Ordering::Relaxed),
        1,
        "the (0 → present) edge fires exactly one Alive"
    );
    assert_eq!(
        obs.last_publisher_count.load(Ordering::Relaxed),
        2,
        "the Alive event reports BOTH live publishers"
    );

    // Drop ONE publisher: count 2 → 1, still alive ⇒ NO Lost.
    drop(pub_a);
    observe_transition(&mut runtime);
    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        0,
        "one publisher remains ⇒ no Lost (the latch holds)"
    );
    let handle = runtime.node_handle("cons").expect("consumer node handle");
    assert_eq!(
        handle.publisher_disconnects_observed_count(),
        0,
        "disconnect counter stays 0 while a publisher remains"
    );

    // Drop the SECOND: count 1 → 0 ⇒ exactly one Lost.
    drop(pub_b);
    observe_transition(&mut runtime);
    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        1,
        "dropping the LAST publisher fires exactly one Lost"
    );
    let handle = runtime.node_handle("cons").expect("consumer node handle");
    assert_eq!(
        handle.publisher_disconnects_observed_count(),
        1,
        "the disconnect counter bumps once on the final leave"
    );
}

// ===========================================================================
// (K) Dead-node CLEANUP runs on its OWN thread at
//     its own cadence, the live-loop sweep does not drive it, AND the
//     graceful path is unperturbed by it.
// ===========================================================================

#[test]
#[serial]
fn cleanup_runs_on_its_own_thread_not_per_sweep_and_graceful_path_unbroken() {
    // `LivelinessCleaner::cleanup_dead_nodes` reclaims a
    // CRASHED publisher's stale iceoryx2 port (forcing `number_of_publishers()`
    // to drop). It runs on a dedicated reclaim thread (not inline in the
    // liveliness sweep on the live-loop thread) at
    // `LIVELINESS_CLEANUP_PERIOD_MS`. This test pins THREE contracts over the
    // REAL machinery on real iceoryx2:
    //
    //   (1) The thread exists and reclaims: with the cadence
    //       shortened through the test seam, `liveliness_cleanup_call_count()`
    //       leaves 0 within a bounded WALL wait, with NO sweep driven at all. A
    //       runtime that never starts the thread, or a thread that never reaches
    //       the reclaim call, stays at 0.
    //
    //   (2) The sweep does not walk the registry: with the
    //       thread PARKED, K sim-clock sweeps leave the count UNCHANGED. Re-adding
    //       the per-sweep `cleanup_dead_nodes()` call bumps it by K.
    //
    //   (3) NON-REGRESSION: the graceful-disconnect path is
    //       untouched: attach + drop a publisher and confirm the `Lost` handler
    //       STILL fires exactly once and the per-node disconnect counter STILL
    //       bumps once, with the count STILL unchanged (the sweeps of that cycle
    //       ran no walk either).
    let obs = Arc::new(LiveObs::default());
    let mut runtime = build_runtime(Arc::clone(&obs)); // 5 ms sweep cadence

    // (1) A short cadence, then a bounded wall wait for the thread's first pass.
    // No `step_ms` here: the pass must come from the thread, not from a sweep.
    runtime.set_liveliness_cleanup_period_for_test(20);
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
    while runtime.liveliness_cleanup_call_count() == 0 && std::time::Instant::now() < deadline {
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    assert!(
        runtime.liveliness_cleanup_call_count() >= 1,
        "the dead-node reclaim thread must run cleanup_dead_nodes on its own cadence \
         (a runtime that never starts the thread stays at 0)"
    );

    // (2) PARK the thread, snapshot, drive K sweeps: the count must not move.
    // Parking first makes the snapshot race-free: the counter is bumped at the
    // START of a pass, so any pass in flight when we park has already counted,
    // and none starts for an hour.
    runtime.set_liveliness_cleanup_period_for_test(3_600_000);
    let parked = runtime.liveliness_cleanup_call_count();
    const K: u64 = 7;
    for _ in 0..K {
        runtime.step_ms(5);
    }
    assert_eq!(
        runtime.liveliness_cleanup_call_count(),
        parked,
        "the live-loop sweep must NOT drive dead-node cleanup (\
         re-adding the per-sweep call bumps this by {K})"
    );

    // (3) The graceful Lost path is unbroken: attach a publisher (Alive), drop
    // it (Lost), and confirm the handler + counter behave exactly as in the graceful tests above.
    let pubr = attach_publisher(&runtime);
    observe_transition(&mut runtime); // Alive edge (baseline 0 → 1)
    assert_eq!(
        obs.alive_fires.load(Ordering::Relaxed),
        1,
        "attaching a publisher still fires exactly one Alive edge"
    );

    drop(pubr); // crashless graceful drop — Drop decrements the count immediately
    observe_transition(&mut runtime); // Lost edge (1 → 0)
    assert_eq!(
        obs.lost_fires.load(Ordering::Relaxed),
        1,
        "the graceful Lost handler STILL fires exactly once"
    );
    let handle = runtime.node_handle("cons").expect("consumer node handle");
    assert_eq!(
        handle.publisher_disconnects_observed_count(),
        1,
        "the per-node disconnect counter STILL bumps once on the graceful drop"
    );
    assert_eq!(
        runtime.liveliness_cleanup_call_count(),
        parked,
        "the sweeps of the graceful cycle ran no registry walk either"
    );
}
