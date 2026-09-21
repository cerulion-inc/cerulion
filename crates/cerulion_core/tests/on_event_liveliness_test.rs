// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end proof that the type-routed `#[on_event]` macro
//! handler fires for the NEW input-scoped `LivelinessEvent` over real iceoryx2
//! via `GraphRuntime::build_for_test` (per-test SHM root — parallel-safe; serial
//! iceoryx2 tests).
//!
//! `LivelinessEvent` is an `#[on_event]` `EventKind` wired exactly
//! how `ExpectWithinEvent`/`PromiseWithinEvent` are:
//! routed by the handler's event PARAMETER TYPE to
//! `ctx.take_liveliness_event(name)`, with a MANDATORY `input = "..."` filter.
//!
//! The REAL crash-detection producer that mints liveliness transitions is a
//! later chunk — NOT this one. Here the event is injected via the scheduler
//! test seam (`GraphRuntime::push_liveliness_event_for_test`, mirroring the
//! production `step()` push site) so the dispatch path can be proven now.
//!
//! Determinism (Principle #7): the injected event carries a FIXED
//! `changed_at_ns` and the dispatch is driven by the deterministic
//! `VirtualClock`, so the fire count is bit-identical across runs.
//!
//! No fake data (Principle #13): the handler increments a shared
//! `Arc<AtomicU64>` injected via the macro-generated `with_state(inner)`
//! constructor, so the count is observed from REAL handler execution; the graph
//! is a real iceoryx2 graph and the inject mirrors the real push site.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

// ===========================================================================
// Producer + liveliness consumer
// ===========================================================================

/// Periodic (5 ms) producer feeding the consumer's input so the topic has a
/// real publisher and the consumer's input edge is wired.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct FeedProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl FeedProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Periodic (5 ms) consumer with a `LivelinessEvent` handler on its input. The
/// node ticks every step (so the Ok-path dispatch runs every tick); the handler
/// fires only on a step where a `LivelinessEvent` is pending for `inp`.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct LivelinessConsumer {
    #[input]
    inp: Vector3,
    last: f64,
    /// Shared with the test harness so it can observe handler firings.
    lost_fires: Arc<AtomicU64>,
}
#[cerulion_node_impl]
impl LivelinessConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last = self.inp.x;
        Ok(())
    }

    /// Liveliness handler — type-routed to `take_liveliness_event` via the
    /// `LivelinessEvent` param type.
    #[on_event(input = "inp")]
    fn on_liveliness(&mut self, event: LivelinessEvent) {
        // Foundation invariants pinned here so a regression in the event payload
        // or in the routing surfaces in this declarative-handler test too.
        assert_eq!(
            event.input_name.as_ref(),
            "inp",
            "LivelinessEvent must carry its input name"
        );
        assert_eq!(
            event.state,
            LivelinessState::Lost,
            "the injected event is a Lost transition"
        );
        assert_eq!(event.cause, LivelinessCause::PublisherDisconnected);
        // Full-payload read-back: the injected `lost_event(123_456_789)` carries
        // count_total=1, changed_at_ns=123_456_789, publisher_count=0. Asserting
        // every field here catches a bug that garbles them in transit (they are
        // minted by `new_for_test` but otherwise never read on the handler side).
        assert_eq!(
            event.count_total, 1,
            "the injected Lost event carries count_total=1"
        );
        assert_eq!(
            event.changed_at_ns, 123_456_789,
            "the injected Lost event carries the fixed changed_at_ns"
        );
        assert_eq!(
            event.publisher_count, 0,
            "the injected Lost event carries publisher_count=0"
        );
        self.lost_fires.fetch_add(1, Ordering::Relaxed);
    }
}

fn liveliness_graph(
    lost_fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "on_event_liveliness_test".to_string(),
        prefix: "oel".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "prod".to_string(),
                node_type: "feed_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "cons".to_string(),
                node_type: "liveliness_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "prod/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("prod".to_string(), Box::new(FeedProducerEntry::new()));
    let consumer = LivelinessConsumer {
        lost_fires,
        ..Default::default()
    };
    factories.insert(
        "cons".to_string(),
        Box::new(LivelinessConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// A `Lost` liveliness event with a FIXED `changed_at_ns` (deterministic — never
/// a wall read), as the real `step()` push site will mint. `LivelinessEvent` is
/// `#[non_exhaustive]`, so this integration-test crate mints it via the
/// `new_for_test` seam rather than a struct literal.
fn lost_event(changed_at_ns: u64) -> LivelinessEvent {
    LivelinessEvent::new_for_test(
        Arc::from("inp"),
        LivelinessState::Lost,
        LivelinessCause::PublisherDisconnected,
        1,
        changed_at_ns,
        0,
    )
}

/// Run the graph for `steps` 5 ms steps; if `inject` is true, push ONE `Lost`
/// liveliness event into the consumer before the last step (so a tick
/// runs after the inject and dispatches the handler). Return the number of
/// `LivelinessEvent` firings the declarative handler observed.
fn run_liveliness(steps: usize, inject: bool) -> u64 {
    let lost_fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = liveliness_graph(Arc::clone(&lost_fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build liveliness graph");
    assert!(steps >= 2, "need >= 2 steps to inject before the last");
    let inject_before = steps - 1;
    for i in 0..steps {
        if inject && i == inject_before {
            // FIXED changed_at_ns keeps the event deterministic regardless of
            // the scheduler clock at this step.
            runtime.push_liveliness_event_for_test("cons", lost_event(123_456_789));
        }
        runtime.step(Duration::from_millis(5));
    }
    lost_fires.load(Ordering::Relaxed)
}

#[test]
fn on_event_liveliness_handler_fires_on_injected_lost() {
    // A single injected `Lost` transition must reach the type-routed handler
    // exactly once (the consumer ticks every 5 ms, so the Ok-path dispatch runs
    // after the inject and drains the pending event).
    let fires = run_liveliness(10, true);
    assert_eq!(
        fires, 1,
        "#[on_event] LivelinessEvent handler must fire EXACTLY once for a single \
         injected Lost transition — newest-wins single slot, drained once by the \
         next tick (got {fires})"
    );
}

#[test]
fn on_event_liveliness_handler_quiet_without_inject() {
    // No inject → no liveliness transition → the handler must stay silent.
    let fires = run_liveliness(10, false);
    assert_eq!(
        fires, 0,
        "#[on_event] LivelinessEvent handler must stay quiet with no injected \
         transition (got {fires})"
    );
}

#[test]
fn on_event_liveliness_dispatch_is_deterministic() {
    // Principle #7: bit-identical fire count across two runs (the injected event
    // carries a fixed changed_at_ns and dispatch is driven by the deterministic
    // VirtualClock, never wall time).
    let a = run_liveliness(10, true);
    let b = run_liveliness(10, true);
    // Anchored to the known single-inject count (1) on BOTH runs rather than a
    // tautological a==b on a scalar (which a regression to a constant would also
    // satisfy). Determinism intent: identical step sequences yield the identical
    // exact fire count, never a wall-clock-dependent value.
    assert_eq!(
        a, 1,
        "#[on_event] LivelinessEvent fire count must be exactly 1 on the first run \
         (a={a})"
    );
    assert_eq!(
        b, 1,
        "#[on_event] LivelinessEvent fire count must be exactly 1 on the second run, \
         deterministically matching the first (b={b})"
    );
}

// ===========================================================================
// Multi-handler routing: a LivelinessEvent handler AND an ExpectWithinEvent
// handler on the SAME input both fire, routed to DISTINCT accessors. This
// proves the new `(port, kind)` ALLOW-case routes correctly (a routing-collapse
// regression onto one accessor would drop one).
// ===========================================================================

/// Slow producer (100 ms) — far outside the consumer's 30 ms expect window, so
/// the watchdog misses windows between arrivals and the `ExpectWithinEvent`
/// handler fires. We ALSO inject a `Lost` liveliness event so the
/// `LivelinessEvent` handler fires — both on the SAME input.
#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct SlowFeedProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl SlowFeedProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Data-triggered consumer whose SINGLE input carries a 30 ms `expect_within_ms`
/// watchdog AND a `LivelinessEvent` handler. Both handlers are on `inp` but route
/// to DISTINCT accessors (`take_expect_within_event` / `take_liveliness_event`).
#[cerulion_node]
#[derive(Default)]
struct DualLivelinessConsumer {
    #[input(trigger, expect_within_ms = 30)]
    inp: Vector3,
    last: f64,
    /// Bumped inside the LivelinessEvent handler.
    live_fires: Arc<AtomicU64>,
    /// Bumped inside the ExpectWithinEvent handler.
    stale_fires: Arc<AtomicU64>,
}
#[cerulion_node_impl]
impl DualLivelinessConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last = self.inp.x;
        Ok(())
    }

    /// Routed to `take_liveliness_event` by the `LivelinessEvent` type.
    #[on_event(input = "inp")]
    fn on_live(&mut self, event: LivelinessEvent) {
        assert_eq!(event.input_name.as_ref(), "inp");
        assert_eq!(event.state, LivelinessState::Lost);
        self.live_fires.fetch_add(1, Ordering::Relaxed);
    }

    /// Routed to `take_expect_within_event` by the `ExpectWithinEvent` type.
    #[on_event(input = "inp")]
    fn on_stale(&mut self, event: ExpectWithinEvent) {
        assert_eq!(event.input_name.as_ref(), "inp");
        assert_eq!(event.expect_within_ms, 30);
        self.stale_fires.fetch_add(1, Ordering::Relaxed);
    }
}

fn dual_graph(
    live_fires: Arc<AtomicU64>,
    stale_fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "on_event_liveliness_dual".to_string(),
        prefix: "oeld".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "prod".to_string(),
                node_type: "slow_feed_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "cons".to_string(),
                node_type: "dual_liveliness_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "prod/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("prod".to_string(), Box::new(SlowFeedProducerEntry::new()));
    let consumer = DualLivelinessConsumer {
        live_fires,
        stale_fires,
        ..Default::default()
    };
    factories.insert(
        "cons".to_string(),
        Box::new(DualLivelinessConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

#[test]
fn liveliness_and_expect_within_handlers_on_same_input_both_fire() {
    // The slow (100 ms) producer over a 30 ms watchdog drives the
    // `ExpectWithinEvent` handler; we ALSO inject a `Lost` liveliness event to
    // drive the `LivelinessEvent` handler. BOTH handlers are on the SAME input
    // and must fire, routed to DISTINCT accessors — a routing-collapse
    // regression would zero one of them.
    let live_fires = Arc::new(AtomicU64::new(0));
    let stale_fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = dual_graph(Arc::clone(&live_fires), Arc::clone(&stale_fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build dual graph");
    // 80 × 5 ms = 400 ms of wire time → the 100 ms producer starves the 30 ms
    // window several times (ExpectWithinEvent fires). Inject a Lost transition
    // partway so the data-triggered consumer ticks afterward and dispatches it.
    for i in 0..80 {
        if i == 40 {
            runtime.push_liveliness_event_for_test("cons", lost_event(987_654_321));
        }
        runtime.step(Duration::from_millis(5));
    }
    let live = live_fires.load(Ordering::Relaxed);
    let stale = stale_fires.load(Ordering::Relaxed);
    assert!(
        live >= 1,
        "the LivelinessEvent handler on `inp` must fire on the injected Lost \
         transition (got {live})"
    );
    assert!(
        stale >= 1,
        "the ExpectWithinEvent handler on `inp` must ALSO fire — a 100 ms producer \
         starves the 30 ms window (got {stale})"
    );
}
