// SPDX-License-Identifier: AGPL-3.0-only
//! A notify that reaches fewer listeners than the topic has must be counted and
//! reported, and the condition that produces one must be REACHABLE, end to end
//! over real iceoryx2.
//!
//! # What changed under the publisher, and why these tests look like this
//!
//! iceoryx2 delivers an event notification over a per-listener `AF_UNIX
//! SOCK_DGRAM` doorbell, and a publisher's notify is delivered to every
//! listener on the topic's event service, its own included (each publisher owns
//! one, to hear `SubscriberConnected`).
//!
//! Until iceoryx2 0.10 the event id rode IN the datagram, so a listener nobody
//! drained filled its socket and every later notify to it failed and logged.
//! That is the flood that filled a robot's disk, and the apparatus in this file
//! used to reproduce it by notifying an undrained listener twenty thousand
//! times.
//!
//! 0.10 removed the condition at the source. The event id and its repeat count
//! now live in a shared-memory counting bitset, the doorbell carries one byte,
//! a full doorbell is swallowed rather than refused, and a notify into a
//! listener that already holds an unconsumed wake skips the send entirely. So
//! notifying an undrained listener forever costs nothing and reports nothing,
//! and an apparatus built on saturation would assert on a state the library can
//! no longer enter. A test that passes because its stimulus stopped working is
//! not a passing test, so the apparatus MOVED rather than being loosened.
//!
//! # The condition that IS reachable, and it is the one that mattered
//!
//! The shortfall detector compares the listeners a notify actually triggered
//! against the count the event service reports. The latch's own documentation
//! always named two conditions that produce a shortfall; the first (saturation)
//! is gone, and the second is not:
//!
//! > a stale registration: the listener's process died and iceoryx2 removed the
//! > dead connection from the notifier's connection list while the topic's
//! > dynamic-config entry has not been reaped yet.
//!
//! A `SIGKILL`ed consumer leaves its listener registered and its doorbell
//! socket without a reader. The next notify's send is refused, iceoryx2 drops
//! that connection and does not count it, and the publisher sees
//! `triggered < listeners` — a genuinely degraded wake path, with a remedy (a
//! dead-node sweep) an operator can act on. That is what every arm below drives.
//!
//! MEASURED on this apparatus rather than argued: twenty thousand notifies into
//! an undrained live listener report zero undelivered, and ten notifies after a
//! killed consumer report nine (the first arms the persistence rule, the rest
//! are counted).
//!
//! # One hole this leaves, stated because it is not obvious
//!
//! A listener killed while holding an UNCONSUMED wake sits in the notified
//! state, and a notify into that state returns success without touching the
//! doorbell. Such a registration is counted as reached for as long as it
//! survives, so the detector does not see it. A consumer that is draining when
//! it dies (the common shape, and the one these tests drive) leaves the state
//! idle and IS seen.
//!
//! # What the tests assert
//!
//! * [`an_undrained_live_listener_is_never_a_shortfall`] — the 0.10 contract,
//!   as a number rather than a silence: twenty thousand undrained notifies, all
//!   delivered, zero counted.
//! * [`an_ingress_publisher_reports_nothing_undelivered_across_a_long_run`] —
//!   the same contract on the raw ingress path that produced the original
//!   flood, through real loans and sends rather than bare notifies.
//! * [`a_killed_consumers_registration_is_counted_as_undelivered`] — the
//!   apparatus arm: the condition is real and the detector fires.
//! * [`the_latch_recovers_when_the_dead_registration_is_reaped`] — the regime
//!   lifecycle over real transport: degraded, then a dead-node sweep removes
//!   the stale registration, then the count stops growing and the loud path is
//!   re-armed.
//! * [`notify_delivery_log_arms_map_warn_then_debug_then_recovery`] — the
//!   `tracing` LEVEL mapping through the production call site (an inverted
//!   warn/debug mapping is invisible to the counter-only arms above).
//!
//! `#[serial]`: each arm spawns a child process over a shared iceoryx2 root.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test notify_shortfall_iox2_test -- --test-threads=1
//! ```

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::RealClock;
use cerulion_core::testing::{
    child_iceoryx_config, count_at_exclusively, debug_level_compiled_in, kill_child_holding_ports,
    line_level, IsolatedRoot,
};
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use serial_test::serial;
use tracing_test::traced_test;

/// Notifies to issue at a live undrained listener. The old apparatus needed
/// this many to exceed a datagram socket's capacity; it is kept because the
/// claim under test is now the opposite one (that no amount of them degrades
/// anything), and a small number would not say that.
const UNDRAINED_NOTIFIES: usize = 20_000;

/// The topic the parent publishes on and the child subscribes to. One name,
/// because the child is handed its namespace, not its topic.
const TOPIC: &str = "/shortfall/probe";

/// Raw publishes on the ingress arm. Each is a full loan, copy, send and
/// notify, so the count is lower than the bare-notify arm's and still far past
/// any queue depth in the path.
const GUARANTEE_PUBLISHES: usize = 5_000;

/// Printed by the child once its subscriber exists and its listener has been
/// drained to idle. The parent kills the child only after seeing it, so a child
/// that died early can never be mistaken for the condition under test.
const CHILD_READY: &str = "CHILD_SUBSCRIBER_READY";

/// Ticks the child spends draining before its parent kills it. It must outlast
/// every arm below; the parent always kills it, so the bound only governs a run
/// whose parent itself died.
const CHILD_DRAIN_TICKS: usize = 120_000;

/// Time given to the namespace after the kill. The child's death is
/// asynchronous, and the arms read the listener count immediately after.
const SETTLE: Duration = Duration::from_millis(200);

/// The child: hold a real subscriber on `TOPIC` over the parent's namespace and
/// keep draining its event listener until killed.
///
/// The draining is not incidental. A listener killed while holding an
/// unconsumed wake sits in iceoryx2's notified state, where a later notify
/// returns success without touching the doorbell, and the shortfall would never
/// be observed. Draining keeps it idle, which is the shape a live consumer
/// actually has.
///
/// `#[ignore]` so a normal suite pass never runs it; the parents invoke it by
/// exact name with the namespace in the environment.
#[test]
#[ignore = "child process entry point — driven by the parent tests in this file"]
fn subprocess_child_holds_a_subscriber() {
    let Some(config) = child_iceoryx_config() else {
        // Run directly with `-- --ignored`: do nothing rather than sleep.
        return;
    };
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: "shortfall_child".to_string(),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 4,
            network: None,
        },
        config,
    )
    .expect("child transport on the parent's namespace");
    let subscriber = transport
        .create_subscriber(TOPIC)
        .expect("child subscriber on the parent's topic");
    subscriber
        .drain_event_notifications()
        .expect("drain the child's listener to idle");

    println!("{CHILD_READY}");
    use std::io::Write;
    std::io::stdout().flush().expect("flush the ready marker");

    for _ in 0..CHILD_DRAIN_TICKS {
        let _ = subscriber.drain_event_notifications();
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// Hand-build a minimal raw wire frame (32-byte little-endian header, payload).
fn make_wire_frame(schema_hash: u64, seq: u32, payload: &[u8]) -> Vec<u8> {
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: 0,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

/// A parent publisher on a namespace a child can be handed.
fn parent_publisher(tag: &str) -> (IsolatedRoot, Arc<TransportManager>, CerulionPublisher) {
    let root = IsolatedRoot::mint(tag);
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("{tag}_parent"),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 4,
            network: None,
        },
        root.config(),
    )
    .expect("parent transport");
    let publisher = transport
        .create_publisher(TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("parent publisher");
    (root, transport, publisher)
}

/// Remove every dead node's stale resources from a namespace, which is what
/// deregisters a killed consumer's listener. iceoryx2 carries the sweep on a
/// node, so this mints one, exactly as `cerulion clean` does.
fn reap_dead_nodes(root: &IsolatedRoot) {
    let sweeper = iceoryx2::node::NodeBuilder::new()
        .config(&root.config())
        .create::<iceoryx2::service::ipc_threadsafe::Service>()
        .expect("a node in the namespace to sweep from");
    let state = sweeper.try_cleanup_dead_nodes();
    assert_eq!(
        state.failed_cleanups, 0,
        "the sweep refused a dead node ({state:?}), so the stale registration is still \
         there and the recovery below would be testing nothing"
    );
}

/// Establish the condition: a consumer attaches, drains, and is killed.
fn kill_a_live_consumer(root: &IsolatedRoot, publisher: &CerulionPublisher) {
    kill_child_holding_ports(
        "subprocess_child_holds_a_subscriber",
        root,
        CHILD_READY,
        SETTLE,
    );
    assert!(
        publisher.event_listener_count_for_test() >= 2,
        "precondition: the killed consumer's listener must still be registered on the \
         topic's event service (its own listener plus the publisher's) — with the \
         registration already reaped there is no stale registration to detect"
    );
}

/// The 0.10 contract, asserted as a number: a listener nobody drains is not a
/// degraded wake path and must never be counted as one.
///
/// This is the arm that would have FAILED on 0.9.1, where the same stimulus
/// filled a datagram socket within a few hundred notifies. It is what licenses
/// the read path and the publish path to stop draining listeners they do not
/// wait on.
#[test]
#[serial]
fn an_undrained_live_listener_is_never_a_shortfall() {
    let (_root, _transport, publisher) = parent_publisher("undrained");

    assert_eq!(
        publisher.notify_undelivered_count(),
        0,
        "a fresh publisher must have zero undelivered notifies"
    );

    // The publisher's own listener is a real, connected recipient: without
    // that, every count below would be vacuous.
    let first = publisher.notify_sent_sample().expect("first notify");
    assert_eq!(
        first, 1,
        "the publisher's OWN listener is the one recipient of its own notify"
    );

    for i in 0..UNDRAINED_NOTIFIES {
        let triggered = publisher.notify_sent_sample().expect("notify");
        assert_eq!(
            triggered, 1,
            "notify {i} reached {triggered} listeners, not 1: a listener nobody drains \
             must stay reachable forever (the doorbell carries one byte, a full \
             doorbell is swallowed, and a notify into an already-notified listener \
             skips the send)"
        );
    }

    assert_eq!(
        publisher.notify_undelivered_count(),
        0,
        "{UNDRAINED_NOTIFIES} notifies at a listener nobody drained must report ZERO \
         undelivered — a nonzero count here means the wake path degrades on an \
         undrained listener again, and every drain this tree removed has to come back"
    );
}

/// The same contract on the RAW INGRESS path, which is the one that produced
/// the original flood: roughly ninety of these run inside one attached bridge,
/// each publishing at line rate, and each auto-notifying from `publish_raw`.
///
/// This arm is what would catch the gate on the publisher's self drain breaking
/// delivery: it is a real loan, copy, send and notify per iteration, with
/// nothing draining the publisher's own listener at any point.
#[test]
#[serial]
fn an_ingress_publisher_reports_nothing_undelivered_across_a_long_run() {
    let root = IsolatedRoot::mint("ingress");
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ingress_parent".into(),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 4,
            network: None,
        },
        root.config(),
    )
    .expect("parent transport");
    let mut ingress = transport
        .create_ingress_publisher(TOPIC, MaxSliceLen::const_new(256))
        .expect("ingress publisher");

    let payload = [0xAAu8, 0xBB, 0xCC, 0xDD];
    for seq in 0..GUARANTEE_PUBLISHES {
        let frame = make_wire_frame(0x1234_5678_9ABC_DEF0, seq as u32, &payload);
        ingress.publish_raw(&frame).expect("publish_raw");
    }

    // EXACTLY 0 is a legitimate oracle here, not an over-tight one. The one
    // accepted false positive the accessor documents is a listener
    // DEREGISTERING between an elision-armed publisher's count read and its
    // notify, and this environment excludes it twice over: an ingress publisher
    // is never elision-armed, so every classification takes the self-read path;
    // and the topic's only listener is the publisher's own, created before the
    // loop and alive after it, so nothing attaches or detaches inside the
    // window.
    assert_eq!(
        ingress.notify_undelivered_count(),
        0,
        "{GUARANTEE_PUBLISHES} raw ingress publishes, each notifying a listener nobody \
         drains, must report ZERO undelivered"
    );
}

/// The apparatus arm: the condition the detector exists for is reachable, and
/// the detector fires on it.
///
/// Without this arm every "zero undelivered" assertion elsewhere could pass
/// because nothing can ever be counted.
#[test]
#[serial]
fn a_killed_consumers_registration_is_counted_as_undelivered() {
    let (root, _transport, publisher) = parent_publisher("killed");

    kill_a_live_consumer(&root, &publisher);

    let listeners = publisher.event_listener_count_for_test();
    let mut triggered_seq = Vec::new();
    for _ in 0..10 {
        triggered_seq.push(publisher.notify_sent_sample().expect("post-kill notify"));
    }

    assert!(
        triggered_seq.iter().all(|&t| t < listeners),
        "the killed consumer's registration must not be reachable: the event service \
         reports {listeners} listeners and the notifies triggered {triggered_seq:?}"
    );
    assert!(
        publisher.notify_undelivered_count() > 0,
        "a notify that reached fewer listeners than the topic reports must be COUNTED \
         — the count is the only queryable signal for a degraded wake path once \
         iceoryx2's own complaint is filtered"
    );
}

/// The regime lifecycle over real transport, and the proof that REAPING the
/// stale registration is what heals it: degraded, then a dead-node sweep, then
/// the count stops growing (it is never reset) and the loud path is re-armed.
#[test]
#[serial]
fn the_latch_recovers_when_the_dead_registration_is_reaped() {
    let (root, _transport, publisher) = parent_publisher("recover");

    kill_a_live_consumer(&root, &publisher);
    for _ in 0..10 {
        let _ = publisher.notify_sent_sample();
    }
    let degraded_total = publisher.notify_undelivered_count();
    assert!(
        degraded_total > 0,
        "precondition: the degraded regime must have opened"
    );

    // Reap the dead node. This is the same sweep `cerulion clean` and the
    // startup hygiene pass run, and it is the operator's remedy here.
    reap_dead_nodes(&root);
    let healed = publisher.event_listener_count_for_test();
    assert_eq!(
        healed, 1,
        "the sweep must remove the dead consumer's registration, leaving only the \
         publisher's own listener; found {healed}"
    );

    // The notify reaches everything the service now reports.
    let triggered = publisher.notify_sent_sample().expect("post-sweep notify");
    assert_eq!(
        triggered, healed,
        "after the reap every registered listener is reachable again"
    );

    // Recovery neither resets the unconditional total nor counts a new failure.
    assert_eq!(
        publisher.notify_undelivered_count(),
        degraded_total,
        "a recovered notify must neither reset nor increment the undelivered total"
    );

    // Re-armed: the healed state is durable, not a one-shot.
    for _ in 0..200 {
        publisher.notify_sent_sample().expect("healed notify");
    }
    assert_eq!(
        publisher.notify_undelivered_count(),
        degraded_total,
        "with every registration reachable the count must never grow again"
    );
}

/// The LOG-ARM MAPPING through the production `notify_sent_sample` call site:
/// the first undelivered notify of a regime is exactly one `warn!`, sustained
/// repeats are `debug!`, and the delivered notify that closes the regime is
/// exactly one recovery `info!`.
///
/// The pure latch state machine is oracle-tested in
/// `transport/notify_delivery_latch.rs`; this pins that the publisher maps each
/// returned action onto the RIGHT `tracing` level. An inverted mapping
/// (warn against debug, or a recovery emitted per notify) would ship green
/// without it, because the counter-only arms above cannot see log levels.
#[test]
#[serial]
#[traced_test]
fn notify_delivery_log_arms_map_warn_then_debug_then_recovery() {
    let (root, _transport, publisher) = parent_publisher("logarms");

    kill_a_live_consumer(&root, &publisher);

    // Open the regime, then two more undelivered notifies for the sustained
    // (downgraded) arm. The counter is the level-free leg: it must count them
    // whatever the log did.
    let mut opened = false;
    for _ in 0..10 {
        let _ = publisher.notify_sent_sample();
        if publisher.notify_undelivered_count() > 0 {
            opened = true;
            break;
        }
    }
    assert!(opened, "precondition: the degraded regime must open");
    let at_open = publisher.notify_undelivered_count();
    for _ in 0..2 {
        let _ = publisher.notify_sent_sample();
    }
    assert_eq!(
        publisher.notify_undelivered_count(),
        at_open + 2,
        "the sustained arm must still COUNT every undelivered notify — the counter is \
         independent of log level and never reset by recovery"
    );

    // Reap and notify: the regime closes with one recovery report.
    reap_dead_nodes(&root);
    publisher.notify_sent_sample().expect("post-sweep notify");

    let topic = TOPIC.to_string();
    logs_assert(move |lines: &[&str]| {
        let mine: Vec<&&str> = lines.iter().filter(|l| l.contains(&topic)).collect();
        // Each count is level-token-matched AND paired with the level-free
        // total of its own conjunction (this topic plus this marker), so neither
        // a line demoted to another level nor a duplicate of it at another level
        // reads as the one line.
        let warns = count_at_exclusively(lines, "WARN", &["reached fewer listeners", &topic])?;
        // Level-free: a suppressed repeat must never be LOUD. This is the half
        // of the contract that survives `release_max_level_info`, where the
        // DEBUG count below reads 0 and its lower bound cannot bite. FIRST, so
        // it is this arm — which names the condition — that fires on a promoted
        // repeat, rather than the exclusive DEBUG count's generic complaint.
        let loud_repeats = mine
            .iter()
            .filter(|l| {
                matches!(line_level(l), Some("WARN" | "INFO" | "ERROR"))
                    && l.contains("still undeliverable (suppressed)")
            })
            .count();
        if loud_repeats != 0 {
            return Err(format!(
                "a sustained repeat was emitted at a LOUD level on {topic} \
                 ({loud_repeats} line(s)) — the downgrade to debug! is the flood suppression"
            ));
        }
        let debugs = count_at_exclusively(
            lines,
            "DEBUG",
            &["still undeliverable (suppressed)", &topic],
        )?;
        let recoveries =
            count_at_exclusively(lines, "INFO", &["notify delivery recovered", &topic])?;
        if warns != 1 {
            return Err(format!(
                "expected EXACTLY one loud regime-opening warn! on {topic}, got {warns} \
                 (a per-notify warn is the flood the latch exists to prevent; 0 means the \
                 loud head was downgraded)"
            ));
        }
        if debug_level_compiled_in() && debugs < 1 {
            return Err(format!(
                "expected the sustained repeats to be downgraded to debug! on {topic}, \
                 got {debugs}"
            ));
        }
        if recoveries != 1 {
            return Err(format!(
                "expected EXACTLY one recovery info! when the regime closed on {topic}, \
                 got {recoveries}"
            ));
        }
        Ok(())
    });
}
