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
