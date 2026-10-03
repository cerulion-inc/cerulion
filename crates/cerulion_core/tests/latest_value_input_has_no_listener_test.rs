// SPDX-License-Identifier: AGPL-3.0-only
//! A latest-value input is built with no event listener, read from the PORT SET.
//!
//! # The shape this guards
//!
//! An input that declares no trigger is read on its own node's fire, by the step's
//! snapshot. Nothing wakes it: no wake source attaches its listener and no
//! execution path waits on it. The port that would sit in every publisher's
//! notifier send loop for it is therefore built not at all, and these arms say so
//! from what exists on the topic rather than from the type.
//!
//! # Why the port set, and not a publisher's failed-notify count
//!
//! On 0.9.1 the cost of an unread listener was visible to the publisher: its socket
//! filled and every later notify took the failed-delivery path. That instrument is
//! GONE on 0.10 and this file measured it going: a tap nobody reads absorbed four
//! thousand publishes with the publisher's undelivered count never leaving zero.
//! `notify_delivery_latch`'s own module doc says why, that the event id and its
//! repeat count live in a shared-memory counting bitset, the doorbell carries one
//! byte, a full doorbell is swallowed rather than refused, and a notify into a
//! listener that already holds an unconsumed wake skips the send, so "a live
//! listener nobody drains is reached forever and counted as reached".
//!
//! So the observable is the PORT SET on the topic's event service, read two ways
//! that must agree: the live listener count the publisher sees, and the expected
//! in-process total the elision gate keeps. A latest-value input contributes to
//! NEITHER. Give it back its listener and the live count rises above the expected
//! total by one, which is what the arms assert against.
//!
//! # Two things the fixture needs
//!
//! The test's publisher is built on the GRAPH'S transport, because
//! `build_for_test` runs on an isolated iceoryx2 configuration and a publisher from
//! the process singleton would open a different service of the same topic name. And
//! the graph is built before it, so the graph's own ports provision the services.
//!
//! The elision gate compares its expected total against the live count for
//! EQUALITY, so an over-count is not cosmetic: it holds the live count below the
//! expected one for the life of the process and the topic never elides again. That
//! is why the arms assert the relation rather than printing the numbers.
//!
//! ```bash
//! cargo test -p cerulion_core --test latest_value_input_has_no_listener_test \
//!     -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::{parse_graph, validate_graph, GraphRuntime};
use cerulion_core::prelude::*;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use indexmap::IndexMap;
use native_ros2_messages::std_msgs::Int32;
use serial_test::serial;

/// Publishes in the window, enough that an unread port would be visible in any
/// instrument: the arms read the port SET, so the number only has to be more than
/// one step's worth of traffic.
const WINDOW_PUBLISHES: usize = 96;

/// The topic under test: the graph producer's output, which the sink reads as a
/// latest value and the test's own publisher also publishes onto.
///
/// It carries the LEADING SLASH, because that is what the runtime resolves a node's
/// output and a YAML input source to; `resolve_source` returns a slash-prefixed
/// source unchanged, so a publisher on the unslashed spelling is a publisher on a
/// DIFFERENT service.
const FEED_TOPIC: &str = "/test/no_listener/producer/out";

/// What the graph producer publishes on its next fire.
static FEED_VALUE: AtomicI32 = AtomicI32::new(0);

/// The last value the sink saw, and how many times its BODY ran.
static SINK_VALUE: AtomicI32 = AtomicI32::new(0);
static SINK_FIRES: AtomicI32 = AtomicI32::new(0);

fn unique_topic(arm: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .expect("a clock after the epoch")
        .as_nanos();
    format!("/test/no_listener/{arm}/{nanos}")
}

fn slot_len() -> MaxSliceLen {
    MaxSliceLen::const_new((WireHeader::SIZE + 64) as u32)
}

/// Publish one `Int32` frame from a publisher of the test's own.
fn publish(publisher: &mut CerulionPublisher, value: i32) {
    let mut proxy = publisher.loan_proxy::<Int32>().expect("a loan");
    proxy.data = value;
}

// ---------------------------------------------------------------------------
// The graph: a producer fired by the test, and a sink whose ONLY input is a
// latest value on the producer's topic. Both are external so the test controls
// exactly which steps publish and which steps read.
// ---------------------------------------------------------------------------

#[cerulion_node(external)]
#[derive(Default)]
struct Feed {
    #[output]
    out: Int32,
}

#[cerulion_node_impl]
impl Feed {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.data = FEED_VALUE.load(Ordering::Relaxed);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[cerulion_node(external)]
#[derive(Default)]
struct Sink {
    #[input]
    value: Int32,
}

#[cerulion_node_impl]
impl Sink {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Serves the frozen slot the snapshot filled. A fire whose snapshot
        // found no delivery collapses the whole tick to a no-op and neither
        // store below runs, which is what makes SINK_FIRES "the body ran".
        SINK_VALUE.store(self.value.data, Ordering::Relaxed);
        SINK_FIRES.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

const GRAPH: &str = r#"
prefix: test/no_listener
multi_publisher_topics:
  - /test/no_listener/producer/out
nodes:
  - id: producer
    type: feed
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: sink
    type: sink
    inputs:
      - name: value
        source: /test/no_listener/producer/out
"#;

fn build_graph() -> GraphRuntime {
    let config = parse_graph(GRAPH).expect("the fixture graph must parse");
    validate_graph(&config).expect("the fixture graph must validate");
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    // Keyed by node ID, not node type.
    factories.insert("producer".to_string(), Box::new(FeedEntry::new()));
    factories.insert("sink".to_string(), Box::new(SinkEntry::new()));
    let clock = std::sync::Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("the fixture must build")
}

const STEP: Duration = Duration::from_millis(1);

/// Publish `value` from the GRAPH producer, then fire ONLY the sink until its
/// body runs and reads that value.
///
/// A fire whose snapshot finds no delivery leaves the generated tick a no-op: the
/// scheduler counts the fire, the body never runs and nothing is recorded. The
/// producer and the sink therefore fire on separate steps, and the sink's fire is
/// a bounded loop, which is the shape the cross-step hold arms use to tolerate
/// the delivery's surfacing latency.
fn feed_and_read(runtime: &mut GraphRuntime, value: i32, what: &str) {
    FEED_VALUE.store(value, Ordering::Relaxed);
    runtime
        .trigger_external("producer")
        .expect("trigger the producer");
    runtime.step(STEP);
    read_expecting(runtime, value, what);
}

/// Fire ONLY the sink until its body runs, and assert what it read.
///
/// Separate from [`feed_and_read`] because after the window the frame this input
/// serves is the TEST publisher's, not the producer's: the latest-wins drain keeps
/// the LAST frame it pops, and this publish is the last one onto the topic before
/// the read, with no producer publish after it. Reading its value is also what
/// proves the two share a service, since a publisher on another instance of the
/// same topic name could not put a frame in this input at all.
fn read_expecting(runtime: &mut GraphRuntime, value: i32, what: &str) {
    for attempt in 1..=200 {
        SINK_FIRES.store(0, Ordering::Relaxed);
        SINK_VALUE.store(0, Ordering::Relaxed);
        runtime.trigger_external("sink").expect("trigger the sink");
        runtime.step(STEP);
        if SINK_FIRES.load(Ordering::Relaxed) > 0 {
            assert_eq!(
                SINK_VALUE.load(Ordering::Relaxed),
                value,
                "{what}: the fire must read the NEWEST frame on the topic, which is what a \
                 latest-value input means: a listener is not what makes the read work"
            );
            return;
        }
        assert!(
            attempt < 200,
            "{what}: the sink's body never ran in 200 fires. A counted fire whose body did not \
             run is a fire whose snapshot found no delivery, so the input read nothing"
        );
    }
}

/// The run both arms share. `elision_off` sets the knob as well, so the result is
/// shown not to depend on the gate's state.
fn run_arm(elision_off: bool) {
    SINK_VALUE.store(0, Ordering::Relaxed);
    SINK_FIRES.store(0, Ordering::Relaxed);

    if elision_off {
        std::env::set_var("CERULION_NOTIFY_ELISION", "off");
    } else {
        std::env::remove_var("CERULION_NOTIFY_ELISION");
    }

    // The GRAPH is built first, and the test's publisher is built on the GRAPH'S OWN
    // transport. `build_for_test` runs on an isolated iceoryx2 configuration that is
    // deliberately not the process singleton, so a publisher taken from
    // `TransportManager::get_or_init` would open a DIFFERENT service of the same
    // topic name: its frames would reach no subscriber of this graph, and the
    // undelivered count it reports would be its own listener's, which its send path
    // drains before every notify. That shape reads zero for a reason that has
    // nothing to do with what an input was built with, so it would read the same
    // whatever this arm is pointed at.
    let mut runtime = build_graph();
    let mgr = Arc::clone(
        runtime
            .test_transport()
            .expect("build_for_test parks a test transport"),
    );
    let mut publisher = mgr
        .create_publisher_simple(FEED_TOPIC, slot_len())
        .expect("a publisher of the test's own on the topic under test");
    std::thread::sleep(Duration::from_millis(50));

    // PORT CONTROL, before anything is measured: the sink reads one frame. This
    // is what keeps every assertion below from passing on a topic the node has no
    // port on.
    feed_and_read(&mut runtime, 11, "the port control");

    // The elision bookkeeping and the live port count must AGREE. The gate is an
    // equality test on these two numbers, so a topic whose expected total counts a
    // listener that was never created can never elide again: every publish on it
    // pays a real send for the life of the process. Asserting them equal is what
    // catches that, and it is the one assertion here that moves when the routing
    // above changes.
    //
    // Only when the gate is ARMED. Its kill switch skips every elision touch
    // including the bookkeeping, so with the knob off the expected total is absent
    // by design and there is nothing to compare it to.
    if !elision_off {
        // A DELTA, not an absolute. The graph's expected total counts the ports the
        // GRAPH minted; this test then added one of its own, the publisher's own
        // listener, which the graph never counted and never should. So the live
        // count must be exactly one above the expected total: equal would mean the
        // graph counted a port it did not create, and the gate compares these two
        // numbers for equality, so such an over-count holds the live count below
        // the expected one forever and the topic never elides again.
        let live = publisher.event_listener_count_for_test();
        let expected = runtime
            .notify_elision_expected_for_test(FEED_TOPIC)
            .expect("an armed gate records an expected total for a graph-owned topic");
        assert_eq!(
            expected + 1,
            live,
            "the topic's expected in-process listener total ({expected}) plus this test's \
             own publisher listener must equal the listeners that exist on it ({live}). A \
             latest-value input counted into the total while holding no listener would make \
             expected too high, and since the elision gate compares the two for equality \
             that disengages elision on this topic permanently"
        );
    }
    // The window: real notifies from a publisher that never elides, with the sink
    // NEVER fired, so no read of the INPUT happens on this topic. What it proves on
    // 0.10 is that the port set does not change under traffic: a listener is not
    // created lazily on first notify, so the count read before the window still
    // holds after it.
    SINK_FIRES.store(0, Ordering::Relaxed);
    for i in 0..WINDOW_PUBLISHES {
        publish(&mut publisher, i as i32 + 1);
        runtime.step(STEP);
    }
    assert_eq!(
        SINK_FIRES.load(Ordering::Relaxed),
        0,
        "the sink must not have run during the window, or the arm would be measuring a read \
         rather than an input that holds no listener"
    );

    // THE OBSERVABLE, after the traffic: the live listener count on the topic. The
    // graph minted one listener here, the producer publisher's own, and this test
    // added one more, its publisher's. The latest-value input minted NONE, so the
    // count is two. Give the input its listener back and this reads three, which
    // reds this arm.
    //
    // This replaces the 0.9.1 observable, a publisher's failed-notify count, which
    // 0.10 removed at the source: this file measured a tap nobody reads absorbing
    // four thousand publishes with that count never leaving zero.
    let live_after = publisher.event_listener_count_for_test();
    assert_eq!(
        live_after, 2,
        "the topic must carry exactly two listeners after the window, the graph producer's \
         own and this test publisher's own, and NONE for the latest-value input. {live_after} \
         means the input was built with a port nothing waits on"
    );

    // And the input still works. The value expected is the TEST publisher's last,
    // because the latest-wins drain keeps the LAST frame it pops and this publish is
    // the last one onto the topic before the read, with no producer publish after
    // it. Reading it is also what proves the two share one service: a publisher on
    // another instance of the same topic name could put no frame in this input.
    let last = WINDOW_PUBLISHES as i32 + 1;
    publish(&mut publisher, last);
    read_expecting(&mut runtime, last, "after the window");

    std::env::remove_var("CERULION_NOTIFY_ELISION");
}

/// The REFUSAL, which is the other half of building an input without a listener:
/// `wait_for_message` on such a subscriber names the input and the reason instead
/// of returning a silent zero or blocking until its timeout on a listener that
/// can never be signalled.
///
/// This call serves subscribers a tool builds for itself; the `cerulion topic`
/// observer is its caller in this tree. It is not how a node reads a declared
/// input.
#[test]
#[serial]
fn wait_for_message_on_a_listenerless_subscriber_refuses_by_name() {
    let mgr = TransportManager::get_or_init().expect("a transport manager");
    let topic = unique_topic("refusal");
    let _publisher = mgr
        .create_publisher_simple(&topic, slot_len())
        .expect("a publisher");
    let subscriber = mgr
        .create_subscriber_no_listener_for_test(&topic)
        .expect("a subscriber built the way a latest-value input is built");

    let err = subscriber
        .wait_for_message(Duration::from_millis(10), |_| {})
        .expect_err(
            "a subscriber with no listener has nothing to wait on, so the call must refuse rather than report that nothing arrived",
        );
    let text = err.to_string();
    for needle in [topic.as_str(), "declares no trigger"] {
        assert!(
            text.contains(needle),
            "the refusal must name {needle}, so the caller can see WHICH input and WHY; got: {text}"
        );
    }
}

/// ANTI-TAUTOLOGY for the arm above: the same call on a subscriber built WITH a
/// listener does not refuse. Without this, "it refuses" is satisfied by a
/// `wait_for_message` that refuses on every subscriber there is.
#[test]
#[serial]
fn wait_for_message_on_a_subscriber_with_a_listener_does_not_refuse() {
    let mgr = TransportManager::get_or_init().expect("a transport manager");
    let topic = unique_topic("no_refusal");
    let _publisher = mgr
        .create_publisher_simple(&topic, slot_len())
        .expect("a publisher");
    let subscriber = mgr
        .create_subscriber(&topic)
        .expect("a subscriber with its listener");

    let read = subscriber
        .wait_for_message(Duration::from_millis(10), |_| {})
        .expect("a subscriber WITH a listener waits rather than refusing");
    assert_eq!(
        read, 0,
        "nothing was published, so the wait must end at its timeout having read nothing: a zero here is the waited-and-saw-nothing answer the refusal above would otherwise be confused with"
    );
}

/// A real notify from a publisher that never elides does not create a listener for
/// the input: the port set on the topic is unchanged after the window.
#[test]
#[serial]
fn a_latest_value_input_mints_no_listener_under_real_notifies() {
    run_arm(false);
}

/// The same with elision turned OFF by its knob, so the result is shown not to
/// depend on the gate's state.
#[test]
#[serial]
fn a_latest_value_input_mints_no_listener_with_elision_off() {
    run_arm(true);
}
