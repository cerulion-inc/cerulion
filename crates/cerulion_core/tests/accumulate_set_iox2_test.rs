// SPDX-License-Identifier: AGPL-3.0-only
//! `AccumulatorSet` over a hand-built iceoryx2 publisher and subscriber: the
//! error arms of the collect, the registration refusals, the event latch and
//! the read bound, each against hand-written values.
//!
//! The graph-level behaviour (two writers, the generated tick, the
//! `#[on_event]` handler) is in `accumulate_input_iox2_test.rs`.
//!
//! ```bash
//! cargo test -p cerulion_core --test accumulate_set_iox2_test -- --test-threads=1
//! ```

use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use cerulion_core::graph::node::{AnySubscriber, NodeContext};
use cerulion_core::message::ShmMessage;
use cerulion_core::scheduler::BackpressureCounters;
use cerulion_core::testing::{count_at_exclusively, debug_lines_expected, never_loud};
use cerulion_core::transport::accumulate::AccumulatorSet;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

const INPUT: &str = "tf";

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .expect("a clock after the epoch")
        .as_nanos() as u64
}

fn unique_topic(arm: &str) -> String {
    format!("test/accumulate_set/{arm}/{}", now_ns())
}

/// A publisher and a subscriber on a topic of this arm's own.
fn pair(arm: &str) -> (CerulionPublisher, CerulionSubscriber, String) {
    let mgr = TransportManager::get_or_init().expect("a transport manager");
    let topic = unique_topic(arm);
    let publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("a publisher");
    let subscriber = mgr.create_subscriber(&topic).expect("a subscriber");
    std::thread::sleep(Duration::from_millis(50));
    (publisher, subscriber, topic)
}

fn context(sub: CerulionSubscriber) -> NodeContext {
    let mut subs = IndexMap::new();
    subs.insert(INPUT.to_string(), AnySubscriber::Ipc(sub));
    NodeContext::for_tests(IndexMap::new(), subs)
}

fn publish(publisher: &mut CerulionPublisher, x: f64) {
    let mut proxy = publisher.loan_proxy::<Vector3>().expect("a loan");
    proxy.x = x;
}

/// A well-formed frame of another schema (hash `0xF0E1`), stamped after every
/// frame published before it.
fn publish_foreign(publisher: &mut CerulionPublisher) {
    let ts = now_ns();
    let body = Vector3::WIRE_FIXED_SIZE;
    let mut header = WireHeader::new(0xF0E1, 999, ts);
    header.total_size = (WireHeader::SIZE + body) as u32;
    let mut frame = vec![0u8; WireHeader::SIZE + body];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    publisher.publish_raw(&frame).expect("a raw publish");
}

/// One collect and one serve: the `x` of every frame the fire received.
fn fire(set: &mut AccumulatorSet, ctx: &mut NodeContext) -> (Vec<f64>, u64) {
    set.snapshot(ctx, &[INPUT.to_string()]);
    set.mark_served(INPUT);
    let view = set.view::<Vector3>(INPUT);
    (view.iter().map(|f| f.x).collect(), view.dropped())
}

fn take_event(ctx: &mut NodeContext) -> Option<(u64, u64, usize)> {
    match ctx.subscriber_mut(INPUT).expect("the input") {
        AnySubscriber::Ipc(s) => s
            .try_take_backpressure_event()
            .map(|e| (e.dropped, e.count_total, e.buffer_capacity)),
    }
}

#[test]
#[serial]
#[traced_test]
fn a_foreign_schema_frame_between_two_valid_frames_is_refused_and_the_rest_are_served() {
    let (mut publisher, sub, topic) = pair("foreign");
    let mut ctx = context(sub);
    let mut set = AccumulatorSet::new();
    set.register::<Vector3>(&mut ctx, INPUT, 4)
        .expect("register");

    // Two refused frames in a row: the first is loud, the second is
    // counted at debug, and the valid frame after them closes the regime.
    publish(&mut publisher, 1.0);
    publish_foreign(&mut publisher);
    publish_foreign(&mut publisher);
    publish(&mut publisher, 2.0);
    assert_eq!(
        fire(&mut set, &mut ctx),
        (vec![1.0, 2.0], 0),
        "both valid frames, in order; the refused frames are neither served nor drops"
    );
    let mismatch = format!("Schema mismatch on topic '{topic}'");
    let head = "accumulate input refused a frame";
    let repeat = "accumulate input still refusing frames";
    logs_assert(|lines: &[&str]| {
        // Level-free: the repeat is never loud, and the head is exactly one
        // WARN line with no copy at any other level. Both hold in release.
        never_loud(lines, repeat)?;
        let loud = count_at_exclusively(lines, "WARN", &[head])?;
        // The repeat is `debug!`: it exists only where `debug!` is compiled in.
        let repeats = count_at_exclusively(lines, "DEBUG", &[repeat])?;
        let named = lines
            .iter()
            .filter(|l| l.contains(head) || l.contains(repeat))
            .all(|l| l.contains(&mismatch));
        if loud == 1 && repeats == debug_lines_expected(1) && named {
            Ok(())
        } else {
            Err(format!(
                "want one WARN head and {} DEBUG repeat(s), each naming the wired topic; \
                 got {loud} head(s) and {repeats} repeat(s)",
                debug_lines_expected(1)
            ))
        }
    });
    assert!(
        logs_contain("accumulate input reads frames again"),
        "the valid frame after the refusals closes the failure regime"
    );
}

#[test]
#[serial]
#[traced_test]
fn a_receive_fault_ends_the_collect_and_the_next_collect_reads_on() {
    let (mut publisher, mut sub, _) = pair("fault");
    // The first read pops a frame; the second read faults.
    sub.fault_inject_receive_after_for_test(1);
    let mut ctx = context(sub);
    let mut set = AccumulatorSet::new();
    set.register::<Vector3>(&mut ctx, INPUT, 4)
        .expect("register");

    publish(&mut publisher, 1.0);
    publish(&mut publisher, 2.0);
    publish(&mut publisher, 3.0);
    assert_eq!(fire(&mut set, &mut ctx), (vec![1.0], 0));
    assert!(logs_contain("accumulate input refused a frame"));
    assert!(logs_contain("fault-injected receive error"));
    assert_eq!(
        fire(&mut set, &mut ctx),
        (vec![2.0, 3.0], 0),
        "the fault popped nothing: the next collect reads the rest"
    );
}

#[test]
#[serial]
fn registration_refuses_a_second_registration_and_an_out_of_range_cap() {
    let mut ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    let mut set = AccumulatorSet::new();
    set.register::<Vector3>(&mut ctx, INPUT, 4).expect("first");
    assert!(set.contains(INPUT));
    assert_eq!(
        set.register::<Vector3>(&mut ctx, INPUT, 4)
            .unwrap_err()
            .to_string(),
        "Invalid transport configuration: accumulate input 'tf' is registered twice"
    );
    for cap in [0usize, 65] {
        assert_eq!(
            set.register::<Vector3>(&mut ctx, "other", cap)
                .unwrap_err()
                .to_string(),
            format!(
                "Invalid transport configuration: accumulate cap {cap} is out of range: \
                 the accepted range is 1..=64"
            )
        );
    }
    assert!(
        !set.contains("other"),
        "a refused registration holds nothing"
    );
}

#[test]
#[serial]
fn an_event_still_pending_keeps_the_next_regime_armed_and_cap_drops_reach_the_counter() {
    let (mut publisher, mut sub, _) = pair("latch");
    let counters = Arc::new(BackpressureCounters::new());
    sub.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("reader"),
        Arc::from(INPUT),
        2,
    );
    let mut ctx = context(sub);
    let mut set = AccumulatorSet::new();
    set.register::<Vector3>(&mut ctx, INPUT, 2)
        .expect("register");

    // Regime 1: 3 frames into a cap of 2, one drop, one event. Not taken.
    for x in [1.0, 2.0, 3.0] {
        publish(&mut publisher, x);
    }
    assert_eq!(fire(&mut set, &mut ctx), (vec![2.0, 3.0], 1));
    // A clean fire re-arms.
    publish(&mut publisher, 4.0);
    assert_eq!(fire(&mut set, &mut ctx), (vec![4.0], 0));
    // Regime 2 overflows while regime 1's event is still pending: its event
    // is not queued, so the regime stays armed.
    for x in [5.0, 6.0, 7.0] {
        publish(&mut publisher, x);
    }
    assert_eq!(fire(&mut set, &mut ctx), (vec![6.0, 7.0], 1));
    assert_eq!(take_event(&mut ctx), Some((1, 1, 2)), "regime 1's event");
    assert_eq!(take_event(&mut ctx), None);
    // Still owed: the next collect of regime 2 delivers regime 2's OPENING
    // event (1 dropped, 2 in total, stamped before this mark), not one built
    // from its own 2 drops and later frames.
    // The publisher stamps frames with the real clock this mark reads.
    let mark = cerulion_core::clock::real_ns();
    for x in [8.0, 9.0, 10.0, 11.0] {
        publish(&mut publisher, x);
    }
    assert_eq!(fire(&mut set, &mut ctx), (vec![10.0, 11.0], 2));
    let opening = match ctx.subscriber_mut(INPUT).expect("the input") {
        AnySubscriber::Ipc(s) => s.try_take_backpressure_event(),
    }
    .expect("regime 2's event, delivered once the slot is free");
    assert_eq!(
        (
            opening.dropped,
            opening.count_total,
            opening.buffer_capacity
        ),
        (1, 2, 2),
        "the opening collect's counts"
    );
    assert!(
        opening.regime_started_at_ns < mark,
        "the opening collect's timestamp, not the retry's: {} >= {mark}",
        opening.regime_started_at_ns
    );
    assert_eq!(take_event(&mut ctx), None, "one event per regime");
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Acquire),
        4,
        "every cap drop reaches the input's drop_oldest counter"
    );
}

#[test]
#[serial]
#[traced_test]
fn an_overflow_regime_logs_loud_once_counts_repeats_and_reports_its_recovery() {
    let (mut publisher, sub, _) = pair("regime");
    let mut ctx = context(sub);
    let mut set = AccumulatorSet::new();
    set.register::<Vector3>(&mut ctx, INPUT, 2)
        .expect("register");
    let warn = "accumulate input dropped its oldest frames";
    let repeat = "accumulate input still dropping frames";
    let recovered = "accumulate input collects without dropping frames again";

    // Two overflowing collects in a row: one loud head, one counted repeat.
    for _ in 0..2 {
        for x in [1.0, 2.0, 3.0] {
            publish(&mut publisher, x);
        }
        assert_eq!(fire(&mut set, &mut ctx), (vec![2.0, 3.0], 1));
    }
    assert!(!logs_contain(recovered), "still dropping: no recovery yet");
    // A collect that drops nothing closes the regime and says so once.
    publish(&mut publisher, 4.0);
    assert_eq!(fire(&mut set, &mut ctx), (vec![4.0], 0));
    publish(&mut publisher, 5.0);
    assert_eq!(fire(&mut set, &mut ctx), (vec![5.0], 0));
    logs_assert(|lines: &[&str]| {
        // Level-free: the repeat is never loud; the head and the recovery
        // are one line each at their own level. All of it holds in release.
        never_loud(lines, repeat)?;
        let heads = count_at_exclusively(lines, "WARN", &[warn])?;
        let recoveries = count_at_exclusively(lines, "INFO", &[recovered])?;
        // The repeat is `debug!`: it exists only where `debug!` is compiled in.
        let repeats = count_at_exclusively(lines, "DEBUG", &[repeat])?;
        if (heads, recoveries) != (1, 1) || repeats != debug_lines_expected(1) {
            return Err(format!(
                "want 1 loud head, {} repeat(s) and 1 recovery, got {heads}, {repeats} and \
                 {recoveries}",
                debug_lines_expected(1)
            ));
        }
        if !lines
            .iter()
            .any(|l| l.contains(recovered) && l.contains("suppressed=1"))
        {
            return Err("the recovery line counts the one suppressed repeat".to_string());
        }
        Ok(())
    });
    // The regime raised exactly one event.
    assert_eq!(take_event(&mut ctx), Some((1, 1, 2)));
    assert_eq!(take_event(&mut ctx), None);
}

#[test]
#[serial]
fn a_frozen_held_frame_ends_the_collect_instead_of_reading_it_forever() {
    let (mut publisher, sub, _) = pair("held");
    let mut ctx = context(sub);
    let mut set = AccumulatorSet::new();
    set.register::<Vector3>(&mut ctx, INPUT, 4)
        .expect("register");

    publish(&mut publisher, 1.0);
    // A latest-value freeze through the public context call holds the
    // frame; a read of a held slot re-offers it without popping.
    ctx.snapshot_inputs(&[INPUT.to_string()]);
    // The freeze took the frame off the queue into the held slot, and the
    // collect stops at the held view instead of inserting it on every read.
    assert_eq!(fire(&mut set, &mut ctx), (vec![], 0));
}
