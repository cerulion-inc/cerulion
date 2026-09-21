// SPDX-License-Identifier: AGPL-3.0-only
//! Behavior-level coverage for NATIVE late-joiner
//! history over an isolated `TestTransport` (iceoryx2 SHM).
//!
//! # Rewrite from the heap-replay era
//!
//! This file used to pin Cerulion's heap `HistoryBuffer` replay
//! loop: `deliver_history` iterated a `VecDeque<Vec<u8>>` and re-published
//! each frame through `publish_raw`, with a break-on-first-error +
//! structured-`error!`-event contract exercised via the
//! `test_inject_history_frame` / `test_deliver_history_now` /
//! `fault_inject_publish_raw_after` hooks. That heap path is DELETED:
//! history is now iceoryx2-native (the publisher port retains the last N
//! sent frames by SHM offset and delivers them automatically to a late
//! joiner). The heap-replay break-vs-continue and per-frame-error contracts
//! no longer exist, so those tests are gone — they pinned a mechanism that
//! was removed, not behavior worth keeping.
//!
//! These tests instead assert the OBSERVABLE native-history contract: a
//! publisher with `history_size = N` that sent M frames, joined late by a
//! subscriber of depth D, delivers it the `min(N, D)` MOST-RECENT frames
//! (zero-copy, via native delivery + the `SentHistory` wake), and an
//! `history_size = 0` publisher delivers no retained frames.
//!
//! # Counting by wire `sequence`
//!
//! Each successful publish stamps a monotonic per-publisher
//! `WireHeader::sequence`. Counting distinct delivered sequences is a
//! layout-independent, deterministic way to assert exactly which retained
//! frames reached the late joiner.

use cerulion_core::testing::TestTransport;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;
use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime};

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/deliver_history/{base}/{nanos}/{id}")
}

/// Publish one `Vector3` frame carrying `x` and let the proxy drop (send).
fn publish(pubr: &mut CerulionPublisher, x: f64) {
    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan_proxy");
    proxy.x = x;
    proxy.y = 0.0;
    proxy.z = 0.0;
}

/// Drain every distinct delivered wire `sequence` from `sub` (a short
/// blocking window absorbs the send→notify race; native history + the
/// freshly-sent live frame may straddle two wakeups).
fn drain_sequences(sub: &mut CerulionSubscriber) -> BTreeSet<u32> {
    let mut seqs = BTreeSet::new();
    for _ in 0..4 {
        let _ = sub
            .wait_for_message(Duration::from_millis(200), |msg| {
                // The parsed WireHeader is on `msg.header()`; `msg.payload()`
                // is only the post-header bytes.
                seqs.insert(msg.header().sequence);
            })
            .expect("wait_for_message");
    }
    seqs
}

// =============================================================
// Happy path: full native delivery when depth >= history
// =============================================================

#[test]
fn native_history_full_delivery_when_depth_ge_history() {
    let topic = unique_topic("full");
    let tt = TestTransport::with_buffer_size(16);
    let mut pubr = tt.publisher(&topic, MaxSliceLen::const_new(4096), 5);

    // Five frames retained (sequences 0..4); depth 16 >= history 5.
    for i in 0..5 {
        publish(&mut pubr, i as f64);
    }

    let mut late = tt.subscriber(&topic);
    // Pump the SubscriberConnected handler (native delivery) + one live
    // frame (sequence 5).
    publish(&mut pubr, 5.0);

    let seqs = drain_sequences(&mut late);
    for expected in 0u32..=5 {
        assert!(
            seqs.contains(&expected),
            "depth>=history: late joiner must receive native-history sequence \
             {expected}; got {seqs:?}"
        );
    }
}

// =============================================================
// Truncation: depth < history → only the newest `depth` retained
// =============================================================

#[test]
fn native_history_truncates_oldest_when_depth_lt_history() {
    let topic = unique_topic("trunc");
    let tt = TestTransport::with_buffer_size(16);
    // History holds 6; the late joiner's queue is only 2 deep.
    let mut pubr = tt.publisher(&topic, MaxSliceLen::const_new(4096), 6);

    for i in 0..6 {
        publish(&mut pubr, i as f64);
    }

    let mut shallow = tt
        .subscriber_with_buffers(&topic, tt.default_topic_config(), 2)
        .expect("shallow subscriber");

    // Pump native delivery + newest live frame (sequence 6).
    publish(&mut pubr, 6.0);

    let seqs = drain_sequences(&mut shallow);
    // The OLDEST history sequences must NOT reach a depth-2 consumer.
    for dropped in [0u32, 1, 2, 3] {
        assert!(
            !seqs.contains(&dropped),
            "depth-2 consumer must NOT receive old history sequence {dropped} \
             (native per-consumer truncation); got {seqs:?}"
        );
    }
    // The history-delivery pin: native delivery sends
    // the newest min(history=6, buffer=2)=2 retained frames {4,5} oldest-first,
    // then the live frame 6 evicts 4 (drop_oldest), leaving exactly {5,6}.
    // Asserting sequence 5 kills the silent-drop regression where delivery
    // sends nothing and the consumer ends with just {6} — which would pass
    // every OTHER assertion in this test.
    assert!(
        seqs.contains(&5),
        "depth-2 consumer must receive the newest RETAINED history frame \
         (sequence 5) — proves per-consumer native history delivery ran; \
         got {seqs:?}"
    );
    // The live frame 6 evicts the older delivered history frame (4); the
    // depth-2 consumer ends with its newest 2 = {5,6}.
    assert!(
        !seqs.contains(&4),
        "depth-2 buffer holds only its newest 2 frames; the live frame 6 must \
         evict the older delivered history frame (4); got {seqs:?}"
    );
    assert!(
        seqs.contains(&6),
        "depth-2 consumer must receive the newest live frame (sequence 6); got {seqs:?}"
    );
}

// =============================================================
// History disabled: a late joiner receives nothing retained
// =============================================================

#[test]
fn native_history_disabled_replays_nothing() {
    let topic = unique_topic("disabled");
    let tt = TestTransport::with_buffer_size(16);
    let mut pubr = tt.publisher(&topic, MaxSliceLen::const_new(4096), 0);

    // Pre-connect frames are NOT retained (sequences 0,1,2).
    for i in 0..3 {
        publish(&mut pubr, i as f64);
    }

    let mut late = tt.subscriber(&topic);
    publish(&mut pubr, 3.0); // sequence 3, post-connect live

    let seqs = drain_sequences(&mut late);
    for retained in [0u32, 1, 2] {
        assert!(
            !seqs.contains(&retained),
            "history-disabled publisher must NOT replay pre-connect sequence \
             {retained}; got {seqs:?}"
        );
    }
    assert!(
        seqs.contains(&3),
        "late joiner must still receive the post-connect live frame (sequence 3); \
         got {seqs:?}"
    );
}

// =============================================================
// Bounded retention: only the newest `history_size` frames are kept
// =============================================================

#[test]
fn native_history_retains_only_newest_history_size_frames() {
    let topic = unique_topic("bounded");
    let tt = TestTransport::with_buffer_size(16);
    // history_size 3, but publish 8 frames (sequences 0..7) before connect.
    let mut pubr = tt.publisher(&topic, MaxSliceLen::const_new(4096), 3);

    for i in 0..8 {
        publish(&mut pubr, i as f64);
    }

    let mut late = tt.subscriber(&topic);
    publish(&mut pubr, 8.0); // sequence 8, live

    let seqs = drain_sequences(&mut late);
    // Only the newest 3 retained (sequences 5,6,7) reach the late joiner;
    // sequences 0..4 were evicted from the bounded native history queue.
    for evicted in 0u32..=4 {
        assert!(
            !seqs.contains(&evicted),
            "bounded history (size 3) must NOT retain evicted sequence {evicted}; got {seqs:?}"
        );
    }
    for retained in [5u32, 6, 7] {
        assert!(
            seqs.contains(&retained),
            "bounded history (size 3) must retain newest sequence {retained}; got {seqs:?}"
        );
    }
}
