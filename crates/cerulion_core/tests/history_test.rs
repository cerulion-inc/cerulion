// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for iceoryx2 NATIVE late-joiner history (the data
//! service's own `history_size`).
//!
//! # Why native
//!
//! Cerulion keeps NO heap `HistoryBuffer` (a `VecDeque<Vec<u8>>`
//! that would copy every wire frame on the publish hot path and re-publish
//! copies to late joiners). History is iceoryx2-NATIVE: the data
//! SERVICE is created with `history_size(N)`, so the publisher port retains
//! the last N sent frames by SHM offset (zero-copy) and delivers them
//! automatically to a late subscriber on `update_connections()` (driven by
//! the publisher's `SubscriberConnected` handler) and on every `send()`.
//!
//! These tests therefore assert BEHAVIOR — "a late subscriber receives the
//! retained frames" — there is no `history_len()`/`has_history()`
//! accessor to read. Delivery is per-consumer truncated to
//! `min(history_size, subscriber.buffer_size)` most-recent frames (the
//! intended semantics).
//!
//! # Delivery mechanism exercised
//!
//! A late subscriber sends `SubscriberConnected` on creation. The publisher
//! drains that event inside its next `loan_proxy()` call
//! (`check_subscriber_events` → `deliver_history` →
//! `publisher.update_connections()` delivers the native history queue into
//! the late joiner's data queue, then fires `SentHistory` to wake it). The
//! tests drive that by doing one more `loan_proxy()`+drop on the publisher
//! after the late subscriber attaches, then draining the subscriber.
//!
//! # Why frames are counted by wire `sequence`
//!
//! Each successful publish stamps a monotonically increasing per-publisher
//! `WireHeader::sequence` (0,1,2,…). Counting DISTINCT delivered sequences
//! is a layout-independent, deterministic way to assert exactly which
//! retained frames reached the late joiner — no fragile fixed-field byte
//! offsets.

use cerulion_core::wire::MaxSliceLen;
use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime};

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::Image;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/hist/{base}/{nanos}/{id}")
}

/// Publish one `Vector3` frame carrying `x` and let the proxy drop (send).
fn publish_vec3(publisher: &mut CerulionPublisher, x: f64) {
    let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
    proxy.x = x;
    proxy.y = 0.0;
    proxy.z = 0.0;
}

/// Drain every wire `sequence` currently deliverable on `sub` (a short
/// blocking window absorbs the send→notify race) and return the SET. Native
/// history + the freshly-sent live frame may straddle two notify wakeups, so
/// we drain a few times.
fn drain_all_sequences(sub: &mut CerulionSubscriber) -> BTreeSet<u32> {
    let mut seqs = BTreeSet::new();
    for _ in 0..4 {
        let _ = sub
            .wait_for_message(Duration::from_millis(200), |msg| {
                // `msg.header()` is the parsed WireHeader; `msg.payload()` is
                // only the POST-header bytes, so read the sequence from the
                // header, not by re-parsing the payload.
                seqs.insert(msg.header().sequence);
            })
            .expect("wait_for_message");
    }
    seqs
}

// ============================================================
// Native history: full delivery to a late joiner
// ============================================================

/// history_size = 3, late subscriber depth = 8 (>= history). Publish three
/// frames (sequences 0,1,2) BEFORE the late joiner, attach it, then publish
/// one more frame (sequence 3) which pumps the publisher's
/// SubscriberConnected handler → native history delivery + SentHistory wake.
/// The late joiner (depth >= history) must receive ALL three retained
/// sequences plus the new one.
#[test]
fn native_history_full_delivery_to_late_joiner_fixed_schema() {
    let topic = unique_topic("native_full_fixed");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 3);

    // Three frames retained in the publisher's native history queue.
    for x in [10.0_f64, 20.0, 30.0] {
        publish_vec3(&mut publisher, x);
    }

    // Late joiner attaches (depth 8 >= history 3 → full delivery).
    let mut late = tt.subscriber(&topic);

    // Pump the publisher's SubscriberConnected handler (delivers native
    // history) and emit one more live frame (sequence 3).
    publish_vec3(&mut publisher, 40.0);

    let seqs = drain_all_sequences(&mut late);
    // Full history (sequences 0,1,2) must all reach the late joiner, plus
    // the post-connect live frame (sequence 3).
    for expected in [0u32, 1, 2, 3] {
        assert!(
            seqs.contains(&expected),
            "late joiner (depth>=history) must receive native-history sequence \
             {expected}; got {seqs:?}"
        );
    }
}

/// Per-consumer truncation: history_size = 5, late subscriber
/// depth = 2. iceoryx2 delivers only the `min(5, 2)` MOST-RECENT history
/// frames to this shallow consumer — older history (sequences 0,1,2) is not
/// delivered to it.
#[test]
fn native_history_truncates_to_subscriber_depth() {
    let topic = unique_topic("native_trunc");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    // Publisher history holds 5; the late subscriber's queue is only 2 deep.
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 5);

    // Sequences 0..4 retained in native history.
    for x in [1.0_f64, 2.0, 3.0, 4.0, 5.0] {
        publish_vec3(&mut publisher, x);
    }

    // Shallow late joiner: buffer_size 2.
    let mut shallow = tt
        .subscriber_with_buffers(&topic, tt.default_topic_config(), 2)
        .expect("shallow subscriber");

    // Pump native delivery + emit a newest live frame (sequence 5).
    publish_vec3(&mut publisher, 6.0);

    let seqs = drain_all_sequences(&mut shallow);
    // The shallow consumer must NOT receive the OLDEST history sequences —
    // native truncation to depth dropped them for this consumer.
    for dropped in [0u32, 1, 2] {
        assert!(
            !seqs.contains(&dropped),
            "depth-2 consumer must NOT receive old history sequence {dropped} \
             (native per-consumer truncation); got {seqs:?}"
        );
    }
    // It DOES receive the newest live frame (sequence 5).
    assert!(
        seqs.contains(&5),
        "depth-2 consumer must receive the newest live frame (sequence 5); got {seqs:?}"
    );
}

/// history disabled (size 0): a late joiner receives NO retained frames —
/// only frames published AFTER it connected.
#[test]
fn native_history_disabled_delivers_nothing_retained() {
    let topic = unique_topic("native_disabled");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);

    // Frames published before the late joiner are NOT retained (history 0):
    // sequences 0,1,2.
    for x in [100.0_f64, 200.0, 300.0] {
        publish_vec3(&mut publisher, x);
    }

    let mut late = tt.subscriber(&topic);

    // One live frame after connect (sequence 3).
    publish_vec3(&mut publisher, 400.0);

    let seqs = drain_all_sequences(&mut late);
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

// ============================================================
// Native history: variable schema round-trips
// ============================================================

/// A variable-schema (Image) frame retained in native history reaches a
/// late joiner intact — fixed fields, encoding string, and data bytes all
/// survive the zero-copy SHM-offset retention + native delivery. Read via
/// the typed `try_view` (latest-wins): after the pump, the newest frame the
/// shallow-1 late joiner holds IS the retained mono8 history frame, because
/// we deliver history then DON'T overwrite it with a live frame on this
/// consumer (history_size=1, depth=1: the single retained frame is what it
/// sees first).
#[test]
fn native_history_variable_schema_round_trips() {
    let topic = unique_topic("native_variable");

    let max_slice =
        (WireHeader::SIZE + <Image as ShmMessage>::WIRE_FIXED_SIZE + 8 * 3 + 256) as u32;
    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(max_slice), 1);

    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.height = 8;
        proxy.width = 8;
        proxy.is_bigendian = 0;
        proxy.step = 8;
        proxy.set_header_bytes(&[]).expect("set header_bytes");
        proxy.set_encoding("mono8").expect("set encoding");
        proxy.set_data(&[1, 2, 3, 4]).expect("set data");
    }

    let late = tt.subscriber(&topic);

    // Pump native delivery of the retained Image frame with one more publish.
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.height = 9;
        proxy.width = 9;
        proxy.is_bigendian = 0;
        proxy.step = 9;
        proxy.set_header_bytes(&[]).expect("set header_bytes");
        proxy.set_encoding("rgb8").expect("set encoding");
        proxy.set_data(&[9, 9]).expect("set data");
    }

    // Collect every Image delivered (the retained mono8 history frame must be
    // present, byte-intact).
    let mut seen: Vec<(u32, String, Vec<u8>)> = Vec::new();
    for _ in 0..4 {
        let _ = late
            .wait_for_message(Duration::from_millis(200), |msg| {
                // `msg.payload()` is already the POST-HEADER payload region
                // (matches `try_view`'s `build_reader` construction).
                let reader = <Image as ShmMessage>::build_reader(msg.payload());
                let enc = reader.encoding().unwrap_or("").to_string();
                seen.push((reader.height, enc, reader.data().to_vec()));
            })
            .expect("wait_for_message");
    }

    let mono = seen
        .iter()
        .find(|(_, enc, _)| enc == "mono8")
        .expect("late joiner must receive the retained mono8 history Image");
    assert_eq!(mono.0, 8, "height preserved through native history");
    assert_eq!(mono.2, vec![1u8, 2, 3, 4], "data bytes preserved");
}
