// SPDX-License-Identifier: AGPL-3.0-only
//! Behavior-level coverage for NATIVE late-joiner
//! history on the **iceoryx2** backend (the process singleton).
//!
//! # What `deliver_history` can fail on
//!
//! There is no per-frame replay loop, so there is nothing per-frame to pin:
//! no `SentHistory` suppression on a truncated
//! replay, no break-on-first-error, no structured `frame_index` error
//! events.
//! History is iceoryx2-native (the publisher port
//! retains the last N sent frames by SHM offset and delivers them
//! automatically on `update_connections()`), so
//! `deliver_history` has NO per-frame failure mode to break on or suppress
//! over: it drives `publisher.update_connections()` (zero-copy native
//! delivery) and then fires `SentHistory` to wake the late joiner — but ONLY
//! when `history_size > 0` (a `history_size == 0` / VOLATILE service retained
//! nothing, so the wake would be spurious and is suppressed).
//!
//! What is pinned end-to-end over real iceoryx2:
//! - a late joiner of depth >= history receives ALL retained frames;
//! - `SentHistory` fires on `SubscriberConnected` (the wake that lets a
//!   late joiner on a quiescent publisher drain the natively-delivered
//!   frames — the key reason `deliver_history` still exists);
//! - a depth < history late joiner receives only its newest `depth` frames
//!   (per-consumer truncation);
//! - history_size = 0 retains nothing.
//!
//! Serial-test required: touches the iceoryx2 singleton.

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime};

use cerulion_core::transport::events::PubSubEvent;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/deliver_history_iox2/{base}/{nanos}/{id}")
}

fn publish(pubr: &mut CerulionPublisher, x: f64) {
    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan_proxy");
    proxy.x = x;
    proxy.y = 0.0;
    proxy.z = 0.0;
}

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
// Full native delivery + SentHistory wake (the contract that keeps
// deliver_history alive on a quiescent publisher)
// =============================================================

#[test]
fn iox2_native_history_full_delivery_and_sent_history_wake() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("full");
    let mut pubr = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(4096), 5)
        .expect("create publisher");

    // Five frames retained natively (sequences 0..4).
    for i in 0..5u32 {
        publish(&mut pubr, i as f64);
    }

    // Late joiner attaches (deep enough for full delivery).
    let mut late = mgr.create_subscriber(&topic).expect("subscriber");

    // Pump the publisher's SubscriberConnected handler: drives native
    // `update_connections()` delivery AND fires `SentHistory`. One live
    // frame (sequence 5) also goes out.
    publish(&mut pubr, 5.0);

    // The SentHistory wake reached the subscriber's listener.
    let events = late.test_drain_events();
    assert!(
        events.contains(&PubSubEvent::SentHistory),
        "deliver_history must fire SentHistory on SubscriberConnected (the wake \
         a quiescent-publisher late joiner needs to drain native history); \
         got events: {events:?}"
    );

    let seqs = drain_sequences(&mut late);
    for expected in 0u32..=5 {
        assert!(
            seqs.contains(&expected),
            "depth>=history late joiner must receive native-history sequence \
             {expected}; got {seqs:?}"
        );
    }
}

// =============================================================
// NEGATIVE TWIN of the full-delivery test: when `update_connections`
// fails (forced via the test seam), `deliver_history` SUPPRESSES the
// SentHistory wake AND delivers zero history frames.
//
// The Cerulion API can't induce a real `update_connections` failure, so
// `force_next_update_connections_fail_for_test` arms the next
// `deliver_history` to take the Err-path gate. This pins the
// SentHistory-suppression contract end-to-end.
//
// Mutation-kill: deleting the real `return` in `deliver_history` (so the
// SentHistory notify fires unconditionally) makes this test FAIL — the
// subscriber would then see `SentHistory` in its drained events.
// =============================================================

#[test]
fn iox2_native_history_update_connections_failure_suppresses_sent_history() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("uc_fail");
    let mut pubr = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(4096), 3)
        .expect("create publisher");

    // Three frames retained natively (sequences 0..2) BEFORE the late joiner
    // attaches — so there IS history that, on a successful refresh, WOULD fire a
    // wake. That makes the "no wake" assertion below meaningful (not vacuously
    // true because there was nothing to deliver in the first place).
    for i in 0..3u32 {
        publish(&mut pubr, i as f64);
    }

    // Late joiner attaches. Its constructor enqueues a `SubscriberConnected`
    // event on the publisher's listener, pending until the publisher drains it.
    let late = mgr.create_subscriber(&topic).expect("subscriber");

    // Arm the forced-fail seam, THEN drive the `SubscriberConnected` handler via
    // `pump_history`. The seam ORs `forced` into the SAME gate the real
    // `update_connections` `Err` flows through (`if uc.is_err() || forced
    // { ...; return; }`), so it takes the wake-SUPPRESSION path WITHOUT a real
    // transport failure (which the Cerulion API can't induce).
    //
    // NOTE: the seam fires AFTER `update_connections` (so that removing the gate's
    // `return` is caught — see the mutation-kill note above). That real
    // `update_connections` SUCCEEDS here, so the native history frames DO land in
    // the subscriber's queue — we therefore do NOT assert "zero frames". What 30.6
    // gates is the WAKE, and that is exactly what we assert: a forced gate-failure
    // suppresses `SentHistory`. (That a REAL `update_connections` `Err` delivers
    // zero frames is iceoryx2 0.9.1 behavior established from source, not
    // re-litigated here — the seam runs the real, succeeding call.)
    pubr.force_next_update_connections_fail_for_test();
    pubr.pump_history();

    // The SentHistory wake was SUPPRESSED — the negative of the full-delivery
    // test's positive `contains(&SentHistory)` assertion. Mutation-kill:
    // deleting the gate's `return` makes the notify fire unconditionally →
    // `SentHistory` would appear here → this test fails.
    let events = late.test_drain_events();
    assert!(
        !events.contains(&PubSubEvent::SentHistory),
        "a forced update_connections gate-failure must SUPPRESS the SentHistory \
         wake; got events: {events:?}"
    );
}

// =============================================================
// SentHistory is gated on `history_size > 0`: a
// VOLATILE (history_size == 0) service retained nothing, so a SentHistory
// wake would be spurious — it is SUPPRESSED. A history-bearing service
// (history_size > 0) still fires it. Both arms live in ONE test so it catches
// both breakages: deleting the `if self.history_size == 0 { return; }` gate
// (the zero arm then sees SentHistory → fails) AND broadening it to fire
// never / always (the >0 arm then loses SentHistory → fails).
//
// This is the positive pin the backpressure ↔ rmw merge added: the rmw native-
// history port routes VOLATILE publishers through deliver_history, so the
// empty-delivery wake became reachable and is now gated, not merely
// documented. (deliver_history_failure_iox2_test.rs)
// =============================================================

#[test]
fn iox2_native_history_size_zero_suppresses_sent_history_but_nonzero_fires() {
    let mgr = TransportManager::get_or_init().expect("init");

    // --- Arm 1: history_size == 0 (VOLATILE) → SentHistory SUPPRESSED ---
    let topic0 = unique_topic("gate_zero");
    let mut pub0 = mgr
        .create_publisher(&topic0, MaxSliceLen::const_new(4096), 0)
        .expect("create history-0 publisher");
    // Publish BEFORE the late joiner so a (hypothetical) non-empty delivery
    // would have something to wake about — makes the "no wake" assertion
    // meaningful rather than vacuous.
    for i in 0..3u32 {
        publish(&mut pub0, i as f64);
    }
    let late0 = mgr
        .create_subscriber(&topic0)
        .expect("history-0 subscriber");
    // Drive the SubscriberConnected handler → deliver_history. update_connections
    // SUCCEEDS (connection established) but history_size == 0 means zero frames,
    // so the zero-frames gate suppresses the wake.
    pub0.pump_history();
    let events0 = late0.test_drain_events();
    assert!(
        !events0.contains(&PubSubEvent::SentHistory),
        "history_size == 0 (VOLATILE) must SUPPRESS the SentHistory wake \
         (no wake on an empty delivery); got events: {events0:?}"
    );

    // --- Arm 2: history_size > 0 → SentHistory STILL FIRES (control) ---
    let topic1 = unique_topic("gate_nonzero");
    let mut pub1 = mgr
        .create_publisher(&topic1, MaxSliceLen::const_new(4096), 2)
        .expect("create history-2 publisher");
    for i in 0..2u32 {
        publish(&mut pub1, i as f64);
    }
    let late1 = mgr
        .create_subscriber(&topic1)
        .expect("history-2 subscriber");
    pub1.pump_history();
    let events1 = late1.test_drain_events();
    assert!(
        events1.contains(&PubSubEvent::SentHistory),
        "history_size > 0 must STILL fire SentHistory (the gate is exactly \
         `history_size > 0`, not always-suppress); got events: {events1:?}"
    );
}

// =============================================================
// Per-consumer truncation: depth < history
// =============================================================

#[test]
fn iox2_native_history_truncates_to_subscriber_depth() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("trunc");
    let mut pubr = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(4096), 6)
        .expect("create publisher");

    for i in 0..6u32 {
        publish(&mut pubr, i as f64);
    }

    // Shallow late joiner (depth 2 < history 6).
    let mut shallow = mgr
        .create_subscriber_with_buffers(&topic, mgr.default_topic_config(), 2)
        .expect("shallow subscriber");

    publish(&mut pubr, 6.0); // newest live, sequence 6

    let seqs = drain_sequences(&mut shallow);
    // Truncation: the OLD history frames are dropped. `4` is dropped too, but
    // for a DIFFERENT reason than 0..3 — see the `!contains(&4)` pin below.
    for dropped in [0u32, 1, 2, 3] {
        assert!(
            !seqs.contains(&dropped),
            "depth-2 consumer must NOT receive old history sequence {dropped}; got {seqs:?}"
        );
    }
    // The history-delivery pin: native delivery sends
    // the newest min(history=6, buffer=2)=2 retained frames {4,5} oldest-first,
    // then the live frame 6 evicts 4 (drop_oldest), leaving the consumer with
    // exactly {5,6}. Sequence 5 came from native history — asserting it kills
    // the silent-drop regression where `update_connections()` delivers nothing
    // and the consumer ends with just {6} (which would pass every OTHER
    // assertion in this test).
    assert!(
        seqs.contains(&5),
        "depth-2 consumer must receive the newest RETAINED history frame \
         (sequence 5) — proves native update_connections() history delivery \
         actually ran; got {seqs:?}"
    );
    // The live frame 6 evicts the older of the two delivered history frames (4)
    // from the depth-2 buffer, so the consumer ends with its newest 2 = {5,6}.
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
// history_size = 0 retains nothing AND suppresses the SentHistory wake
// (no wake on an empty delivery). See the dedicated
// SentHistory-gating pins below.
// =============================================================

#[test]
fn iox2_native_history_disabled_retains_nothing() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("disabled");
    let mut pubr = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(4096), 0)
        .expect("create publisher");

    for i in 0..3u32 {
        publish(&mut pubr, i as f64); // sequences 0,1,2 — NOT retained
    }

    let mut late = mgr.create_subscriber(&topic).expect("subscriber");
    publish(&mut pubr, 3.0); // post-connect live, sequence 3

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
// Bounded retention: only the newest `history_size` frames survive
// =============================================================

#[test]
fn iox2_native_history_retains_only_newest_history_size() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("bounded");
    let mut pubr = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(4096), 3)
        .expect("create publisher");

    // 7 frames before connect (sequences 0..6); only newest 3 retained.
    for i in 0..7u32 {
        publish(&mut pubr, i as f64);
    }

    let mut late = mgr.create_subscriber(&topic).expect("subscriber");
    publish(&mut pubr, 7.0); // live, sequence 7

    let seqs = drain_sequences(&mut late);
    for evicted in 0u32..=3 {
        assert!(
            !seqs.contains(&evicted),
            "bounded history (size 3) must NOT retain evicted sequence {evicted}; got {seqs:?}"
        );
    }
    for retained in [4u32, 5, 6] {
        assert!(
            seqs.contains(&retained),
            "bounded history (size 3) must retain newest sequence {retained}; got {seqs:?}"
        );
    }
}

// =============================================================
// Dup + content robustness (deferral 3): the other history tests key off a
// `BTreeSet<u32>` of wire sequences, which collapses duplicates and ignores
// payload bytes. This one test drains into a `Vec<(sequence, x)>` — so a
// DUPLICATE delivery is observable — AND reads each frame's payload, so wrong
// CONTENT is observable. Publishing `x == sequence` makes the two checkable
// together: the drained pairs must be EXACTLY the retained set, each once,
// each carrying its own value.
// =============================================================

/// Drain delivered frames as `(wire sequence, payload x)` pairs. Unlike
/// `drain_sequences`'s `BTreeSet`, a `Vec` preserves duplicates (so a
/// double-delivery is visible), and it captures the payload `x` (so wrong
/// content is visible). `Vector3` is a FIXED schema (no offset table), so the
/// payload is the fixed section directly and `x` is its first `f64` (LE).
fn drain_seq_and_x(sub: &mut CerulionSubscriber) -> Vec<(u32, f64)> {
    let mut out = Vec::new();
    for _ in 0..4 {
        let _ = sub
            .wait_for_message(Duration::from_millis(200), |msg| {
                let seq = msg.header().sequence;
                let x = f64::from_le_bytes(
                    msg.payload()[0..8]
                        .try_into()
                        .expect("Vector3 fixed payload is >= 8 bytes"),
                );
                out.push((seq, x));
            })
            .expect("wait_for_message");
    }
    out
}

#[test]
fn iox2_native_history_delivers_each_frame_once_with_correct_payload() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("dup_content");
    let mut pubr = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(4096), 4)
        .expect("create publisher");

    // Frame i carries `x = i`, and (from a fresh publisher) is stamped wire
    // sequence i — so `x == sequence` for every frame.
    for i in 0..4u32 {
        publish(&mut pubr, i as f64);
    }

    // Deep-enough late joiner (default buffer >> 4 → full delivery).
    let mut late = mgr.create_subscriber(&topic).expect("subscriber");
    // Drive native history delivery WITHOUT a live frame, so EXACTLY the 4
    // retained frames are delivered (a live publish would add a 5th).
    pubr.pump_history();

    let frames = drain_seq_and_x(&mut late);

    // (1) NO DUPLICATES — each sequence appears at most once. (`drain_sequences`'s
    // set could not see this; the Vec can.)
    let seqs: Vec<u32> = frames.iter().map(|(s, _)| *s).collect();
    let unique: BTreeSet<u32> = seqs.iter().copied().collect();
    assert_eq!(
        seqs.len(),
        unique.len(),
        "no retained history frame may be delivered more than once; got {frames:?}"
    );

    // (2) EXACTLY the 4 retained frames, each carrying its OWN payload. A frame
    // delivered with a mismatched payload would break `x == sequence`; a missing
    // or extra frame would break the exact-vector equality.
    let mut sorted = frames.clone();
    sorted.sort_by_key(|(s, _)| *s);
    let expected: Vec<(u32, f64)> = (0..4u32).map(|i| (i, i as f64)).collect();
    assert_eq!(
        sorted, expected,
        "each retained frame must arrive exactly once with its own payload \
         (x == sequence); got {frames:?}"
    );
}
