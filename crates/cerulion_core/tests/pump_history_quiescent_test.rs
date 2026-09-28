// SPDX-License-Identifier: AGPL-3.0-only
//! Quiescent-publisher history delivery.
//!
//! Native iceoryx2 history is delivered to a late joiner by
//! `publisher.update_connections()`, which Cerulion drives from
//! `deliver_history` — but ONLY off the publish path (`loan_proxy` →
//! `check_subscriber_events`). So a publisher that filled its history queue
//! and then went QUIESCENT (never sends again) would leave a late joiner with
//! nothing, because the `SubscriberConnected` event is never drained.
//!
//! `CerulionPublisher::pump_history()` is a public, publish-free
//! driver the runtime calls on a cadence (in `live_step`, off the firing
//! path) so quiescent publishers still service late joiners. These tests pin
//! the publisher-level mechanism directly over an isolated `TestTransport`
//! (per-test SHM root → parallel-safe).
//!
//! Contrast with `history_test.rs`, whose tests pump delivery by doing "one
//! more `loan_proxy()`+drop" — here we do NO further publish and assert
//! delivery happens solely because of `pump_history()`.

use cerulion_core::wire::MaxSliceLen;
use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime};

use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use native_ros2_messages::geometry_msgs::Vector3;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/pump_hist/{base}/{nanos}/{id}")
}

fn publish_vec3(publisher: &mut CerulionPublisher, x: f64) {
    let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
    proxy.x = x;
    proxy.y = 0.0;
    proxy.z = 0.0;
}

fn drain_all_sequences(sub: &mut CerulionSubscriber) -> BTreeSet<u32> {
    let mut seqs = BTreeSet::new();
    for _ in 0..4 {
        let _ = sub
            .wait_for_message(Duration::from_millis(200), |msg| {
                seqs.insert(msg.header().sequence);
            })
            .expect("wait_for_message");
    }
    seqs
}

// ============================================================
// The core fix: a QUIESCENT publisher delivers history via pump_history()
// ============================================================

/// history_size = 3, late subscriber depth >= history. Publish three frames
/// (sequences 0,1,2), attach a late joiner, then — CRUCIALLY — publish NOTHING
/// more. Driving `pump_history()` alone must deliver all three retained
/// frames. This is the quiescent-publisher gap the chunk closes: without the
/// pump, the `SubscriberConnected` event is never drained and the late joiner
/// gets nothing.
#[test]
fn pump_history_delivers_retained_history_to_a_quiescent_publishers_late_joiner() {
    let topic = unique_topic("quiescent_full");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 3);

    // Fill the native history queue, then go quiescent.
    for x in [10.0_f64, 20.0, 30.0] {
        publish_vec3(&mut publisher, x);
    }

    // Late joiner attaches AFTER the publisher stopped publishing.
    let mut late = tt.subscriber(&topic);

    // NO further publish — the ONLY driver is the runtime-style pump.
    publisher.pump_history();

    let seqs = drain_all_sequences(&mut late);
    for expected in [0u32, 1, 2] {
        assert!(
            seqs.contains(&expected),
            "pump_history() on a quiescent publisher must deliver retained \
             native-history sequence {expected} to the late joiner; got {seqs:?}"
        );
    }
    // No frame was published after connect, so there is no sequence 3.
    assert!(
        !seqs.contains(&3),
        "no frame was published after connect — there must be no sequence 3; got {seqs:?}"
    );
}

/// A second `pump_history()` after the late joiner has already drained its
/// history is a harmless no-op: no `SubscriberConnected` event is pending, so
/// nothing is re-delivered (history is not replayed twice).
#[test]
fn pump_history_does_not_redeliver_after_the_join_was_serviced() {
    let topic = unique_topic("quiescent_no_redeliver");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 3);
    for x in [10.0_f64, 20.0, 30.0] {
        publish_vec3(&mut publisher, x);
    }
    let mut late = tt.subscriber(&topic);
    publisher.pump_history();
    let first = drain_all_sequences(&mut late);
    assert!(
        first.contains(&0) && first.contains(&1) && first.contains(&2),
        "first pump must deliver the retained history; got {first:?}"
    );

    // A second pump with no NEW subscriber connect drains no pending event and
    // delivers nothing further.
    publisher.pump_history();
    let second = drain_all_sequences(&mut late);
    assert!(
        second.is_empty(),
        "a second pump_history() with no pending SubscriberConnected must \
         deliver nothing (history is not replayed); got {second:?}"
    );
}

/// `pump_history()` on a history-disabled (size 0) publisher is a harmless
/// no-op even with a late joiner present: nothing was retained, so nothing is
/// delivered, and the call neither panics nor errors.
#[test]
fn pump_history_is_harmless_noop_when_history_disabled() {
    let topic = unique_topic("quiescent_disabled");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    for x in [10.0_f64, 20.0, 30.0] {
        publish_vec3(&mut publisher, x);
    }
    let mut late = tt.subscriber(&topic);

    // Must not panic; delivers nothing because history_size = 0 retained
    // nothing.
    publisher.pump_history();

    let seqs = drain_all_sequences(&mut late);
    assert!(
        seqs.is_empty(),
        "history-disabled publisher must replay nothing on pump_history(); got {seqs:?}"
    );
}

// ============================================================
// The gate that decides whether a pump drains at all
// ============================================================

/// A listener count change arms MORE THAN ONE drain.
///
/// The gate in `check_subscriber_events` is edge triggered on
/// `number_of_listeners()`, and it deliberately arms a BUDGET rather than a
/// single drain: the count rises when a subscriber's listener is created and
/// the `SubscriberConnected` notify lands after it, so one gated drain could
/// fall inside that window and see nothing. Nothing pinned the budget, so
/// shrinking it to one, which removes exactly that behaviour, was green.
///
/// A publisher with no subscriber yet is the clean stage: the first pass sees
/// the count move off its `usize::MAX` sentinel to one (the publisher's own
/// listener) and arms, and no transition is pending to spend the arming early,
/// so the budget is observable by counting the passes it survives.
#[test]
fn a_listener_count_change_arms_more_than_one_drain() {
    let topic = unique_topic("arming_budget");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 3);

    // One pass: the count moves off the sentinel, the budget is armed, and this
    // pass spends one of it. No subscriber exists, so no transition is drained.
    publisher.pump_history();
    let after_first = publisher.self_drains_armed_for_test();
    assert!(
        after_first > 1,
        "a listener count change must arm a BUDGET of drains, not one: after \
         the pass that observed the change {after_first} remained armed"
    );

    // Spend it down and count the passes. A budget of one would be exhausted by
    // the first pass above and this loop would run zero times.
    let mut passes = 1_u32;
    while publisher.self_drains_armed_for_test() > 0 {
        publisher.pump_history();
        passes += 1;
        assert!(
            passes < 10_000,
            "the arming budget is not being spent: {passes} passes and \
             {} still armed",
            publisher.self_drains_armed_for_test()
        );
    }
    assert!(
        passes > 1,
        "the budget must survive more than the pass that armed it; it was \
         exhausted after {passes}"
    );
}

/// A net zero listener swap on a quiescent publisher still delivers history.
///
/// This is the interleaving the call budget alone cannot cover, staged exactly:
///
/// 1. a subscriber attaches, is serviced, and the observed transition spends
///    the whole arming budget (asserted below, so the stage is real);
/// 2. that subscriber detaches and another attaches before the next pass, so
///    the listener COUNT is unchanged and the edge triggered gate never fires;
/// 3. the new subscriber's `SubscriberConnected` sits in the publisher's
///    listener with nothing left to drain it.
///
/// On a publisher that still sends, iceoryx2 rescues this itself: `send_sample`
/// calls `update_connections`, which delivers history to every newly connected
/// subscriber. On a QUIESCENT publisher there is no send, so without a time
/// bound the late joiner waits for the count to move again, which on a quiet
/// topic may be never. The idle deadline is what closes it, and this arm fails
/// the moment that deadline is removed from `pump_history`.
#[test]
fn a_net_zero_listener_swap_still_delivers_history_once_the_idle_deadline_passes() {
    let topic = unique_topic("net_zero_swap");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 3);
    for x in [10.0_f64, 20.0, 30.0] {
        publish_vec3(&mut publisher, x);
    }

    // Stage 1: the first joiner is serviced, and that observed transition
    // zeroes the budget. Every later pass is the cheap load until the count
    // moves again.
    let first = tt.subscriber(&topic);
    publisher.pump_history();
    assert_eq!(
        publisher.self_drains_armed_for_test(),
        0,
        "servicing the first joiner must spend the arming, otherwise the swap \
         below is covered by leftover budget and this arm proves nothing"
    );

    // Stage 2: the swap. The count returns to what the gate last recorded, so
    // no arming fires, and stage 1 left nothing to spend.
    drop(first);
    let mut late = tt.subscriber(&topic);

    // Stage 3: the idle pass, run as if the deadline had elapsed rather than
    // sleeping for it. This is the ONLY thing that can drain the swap.
    publisher.pump_history_past_the_idle_deadline_for_test();

    let seqs = drain_all_sequences(&mut late);
    for expected in [0u32, 1, 2] {
        assert!(
            seqs.contains(&expected),
            "the idle deadline must deliver retained sequence {expected} to a \
             late joiner that arrived on a net zero listener swap; got {seqs:?}"
        );
    }
}
