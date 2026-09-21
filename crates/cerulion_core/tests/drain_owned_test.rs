// SPDX-License-Identifier: AGPL-3.0-only
//! The sample-held-beyond-callback recording tap that the
//! `bagd` recorder binary consumes.
//!
//! Covers three pieces added to `transport/subscriber.rs`:
//!   1. [`OwnedInboundSample`] — an owned, zero-copy inbound frame held past
//!      the receive callback (`payload()` = the full published frame,
//!      `wire_header()` = the parsed header).
//!   2. [`DataOnlySubscriber::drain_owned`] — batch-drain up to `max` frames as
//!      owned samples, honoring the per-subscriber
//!      `subscriber_max_borrowed_samples` budget. (This is the KEPT tap after
//!      the `wait_any`-family + `CerulionSubscriber::drain_owned` deletion; the
//!      borrow-budget service is provisioned by a throwaway listener-full
//!      subscriber, then read via the listener-less data-only tap — see
//!      `sub_with_borrow`.)
//!   3. [`DataOnlySubscriber::max_borrowed_samples`] — borrow-ceiling
//!      visibility.
//!
//! # Isolation & parallel-safety
//!
//! Every test uses [`TestTransport`], which owns an iceoryx2
//! `TransportManager` rooted at a UNIQUE per-instance SHM prefix (via
//! `iceoryx_test_config`). Two `TestTransport`s never collide even on identical
//! topic names, so this file is PARALLEL-SAFE and is deliberately NOT marked
//! `#[serial]` — it does not touch the global `TransportManager` singleton or
//! any process-global state.
//!
//! # Oracles, never self-compares
//!
//! `drain_owned` frames are published via `publish_raw` (byte-exact — the
//! publisher loans exactly `data.len()`), so the received bytes are compared
//! against HAND-BUILT expected wire frames, not a second run of the code under
//! test. `publish_raw` does NOT emit a `SentSample` event, which is fine:
//! `drain_owned` reads the SHM message queue directly. The
//! `payload_truncates_slot_padding_to_total_size` test uses `loan_proxy` (which
//! loans the full `max_slice_len`) to exercise slot-padding truncation.

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime};

use cerulion_core::testing::TestTransport;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::{
    CerulionSubscriber, DataOnlySubscriber, OwnedInboundSample,
};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use native_ros2_messages::geometry_msgs::Vector3;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/drain_owned/{base}/{nanos}/{id}")
}

/// Hand-build a wire frame: a valid 32-byte [`WireHeader`] (with `total_size`
/// set to the full frame length) followed by `body`.
fn build_frame(schema_hash: u64, sequence: u32, timestamp_ns: u64, body: &[u8]) -> Vec<u8> {
    let total = WireHeader::SIZE + body.len();
    let mut header = WireHeader::new(schema_hash, sequence, timestamp_ns);
    header.total_size = total as u32;
    let mut frame = vec![0u8; total];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(body);
    frame
}

/// A record/replay data-only tap ([`DataOnlySubscriber`]) provisioned at an
/// explicit `subscriber_max_borrowed_samples` (so it can hold many owned
/// samples at once). The data-only tap is OPEN-ONLY, so this creates the topic
/// service first with a listener-full provisioner subscriber (which sets the
/// borrow budget + buffer), then attaches the listener-less tap. Both are
/// created BEFORE the test's publisher, so the tap is connected when the
/// publisher publishes. The provisioner is kept alive by the returned wrapper
/// so the service (and its borrow budget) never tears down mid-test.
struct BorrowTap {
    _provisioner: CerulionSubscriber,
    tap: DataOnlySubscriber,
}

impl std::ops::Deref for BorrowTap {
    type Target = DataOnlySubscriber;
    fn deref(&self) -> &DataOnlySubscriber {
        &self.tap
    }
}

impl std::ops::DerefMut for BorrowTap {
    fn deref_mut(&mut self) -> &mut DataOnlySubscriber {
        &mut self.tap
    }
}

fn sub_with_borrow(tt: &TestTransport, topic: &str, borrow: usize, buffer: usize) -> BorrowTap {
    let mut cfg = tt.default_topic_config();
    cfg.subscriber_max_borrowed_samples = Some(borrow);
    let provisioner = tt
        .subscriber_with_buffers(topic, cfg, buffer)
        .expect("subscriber_with_buffers (provisions the borrow-budget service)");
    let tap = tt
        .data_only_subscriber(topic)
        .expect("data-only tap opens on the provisioned service");
    BorrowTap {
        _provisioner: provisioner,
        tap,
    }
}

/// iceoryx2 delivery is synchronous (a returned `send()` has already enqueued
/// the sample on every connected subscriber), so this settle is pure paranoia
/// margin for CI-VM surfacing latency, not a correctness dependency.
fn settle() {
    std::thread::sleep(Duration::from_millis(30));
}

// ============================================================
// 1. Happy path — 5 hand-built frames drain byte-identically, in FIFO order.
// ============================================================

#[test]
fn drain_owned_happy_path_byte_matches_hand_built_frames() {
    const N: usize = 5;
    let tt = TestTransport::with_buffer_size(16);
    let topic = unique_topic("happy");

    // Subscriber first (borrow 16 so all 5 can be held at once), then publisher.
    let mut sub = sub_with_borrow(&tt, &topic, 16, 8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);

    // Hand-build + publish 5 distinct frames (distinct schema hash, sequence,
    // timestamp, AND body bytes/length).
    let mut expected: Vec<Vec<u8>> = Vec::new();
    for i in 0..N {
        let body = vec![0xA0u8 + i as u8; 8 + i]; // distinct bytes and lengths
        let frame = build_frame(
            0x1111_2222_3333_4444 + i as u64,
            i as u32,
            1000 + i as u64,
            &body,
        );
        publisher.publish_raw(&frame).expect("publish_raw");
        expected.push(frame);
    }
    settle();

    let mut held: Vec<OwnedInboundSample> = Vec::new();
    let n = sub.drain_owned(10, &mut held).expect("drain_owned");
    assert_eq!(n, N, "drain_owned(10) must return all {N} queued frames");
    assert_eq!(held.len(), N);

    // No extra frames beyond the 5 published.
    let extra = sub.drain_owned(10, &mut held).expect("drain_owned");
    assert_eq!(extra, 0, "no frames beyond the {N} published");
    assert_eq!(held.len(), N);

    // Byte-match each frame against its hand-built oracle, in FIFO order, and
    // confirm the header parses with the expected fields.
    for (i, owned) in held.iter().enumerate() {
        assert_eq!(
            owned.payload(),
            expected[i].as_slice(),
            "frame {i} payload bytes must match the hand-built frame"
        );
        let h = owned.wire_header().expect("wire_header must parse");
        assert_eq!(h.sequence, i as u32, "frame {i} wire sequence");
        assert_eq!(
            h.schema_hash,
            0x1111_2222_3333_4444 + i as u64,
            "frame {i} schema hash"
        );
        assert_eq!(
            h.total_size as usize,
            expected[i].len(),
            "frame {i} total_size == frame length"
        );
        // payload() is bounded to total_size (internal consistency).
        assert_eq!(owned.payload().len(), h.total_size as usize);
    }
}

// ============================================================
// 2. Max cap + FIFO no-loss: drain 3 then the remaining 2.
// ============================================================

#[test]
fn drain_owned_max_cap_and_fifo_no_loss() {
    const N: usize = 5;
    let tt = TestTransport::with_buffer_size(16);
    let topic = unique_topic("cap");

    let mut sub = sub_with_borrow(&tt, &topic, 16, 8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);

    for i in 0..N {
        let frame = build_frame(0xDEAD_0000 + i as u64, i as u32, i as u64, &[i as u8; 4]);
        publisher.publish_raw(&frame).expect("publish_raw");
    }
    settle();

    // drain_owned(3) caps at 3 even though 5 are queued.
    let mut first: Vec<OwnedInboundSample> = Vec::new();
    let n1 = sub.drain_owned(3, &mut first).expect("drain_owned");
    assert_eq!(n1, 3, "drain_owned(3) must cap at 3 with 5 queued");

    // The NEXT drain returns the remaining 2 (no loss).
    let mut second: Vec<OwnedInboundSample> = Vec::new();
    let n2 = sub.drain_owned(10, &mut second).expect("drain_owned");
    assert_eq!(n2, 2, "the remaining 2 frames survive (FIFO, no loss)");

    // FIFO ordering across the two drains: sequences 0,1,2 then 3,4.
    let seqs: Vec<u32> = first
        .iter()
        .chain(second.iter())
        .map(|o| o.wire_header().expect("header").sequence)
        .collect();
    assert_eq!(seqs, vec![0, 1, 2, 3, 4], "FIFO order preserved, no loss");
}

// ============================================================
// 3. max == 0 is a no-op (does not touch the port), a full drain still sees all.
// ============================================================

#[test]
fn drain_owned_max_zero_is_noop() {
    const N: usize = 5;
    let tt = TestTransport::with_buffer_size(16);
    let topic = unique_topic("zero");

    let mut sub = sub_with_borrow(&tt, &topic, 16, 8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);

    for i in 0..N {
        let frame = build_frame(0xF00D, i as u32, i as u64, &[0x11; 4]);
        publisher.publish_raw(&frame).expect("publish_raw");
    }
    settle();

    let mut out: Vec<OwnedInboundSample> = Vec::new();
    let n0 = sub.drain_owned(0, &mut out).expect("drain_owned(0)");
    assert_eq!(n0, 0, "max == 0 returns Ok(0)");
    assert!(out.is_empty(), "max == 0 pushes nothing");

    // The port was untouched — a full drain still sees every frame.
    let n = sub.drain_owned(100, &mut out).expect("drain_owned");
    assert_eq!(n, N, "the untouched port still yields all {N} frames");
    assert_eq!(out.len(), N);
}

// ============================================================
// 4. Held-across-publish: a held sample's SHM bytes stay valid + unchanged
//    while the publisher keeps publishing.
// ============================================================

#[test]
fn owned_sample_held_across_later_publishes_stays_valid() {
    let tt = TestTransport::with_buffer_size(16);
    let topic = unique_topic("held");

    let mut sub = sub_with_borrow(&tt, &topic, 16, 8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);

    // Publish the first 2 frames, drain + HOLD them.
    let f0 = build_frame(0xAA, 0, 0, &[0x01, 0x02, 0x03, 0x04]);
    let f1 = build_frame(0xBB, 1, 1, &[0x05, 0x06, 0x07, 0x08]);
    publisher.publish_raw(&f0).expect("publish_raw");
    publisher.publish_raw(&f1).expect("publish_raw");
    settle();

    let mut held: Vec<OwnedInboundSample> = Vec::new();
    let n = sub.drain_owned(2, &mut held).expect("drain_owned");
    assert_eq!(n, 2);
    // Snapshot the held bytes BEFORE the later publishes.
    let held0_before = held[0].payload().to_vec();
    let held1_before = held[1].payload().to_vec();
    assert_eq!(held0_before, f0);
    assert_eq!(held1_before, f1);

    // Publisher keeps publishing 3 MORE frames while the first 2 are held.
    for i in 2..5u32 {
        let frame = build_frame(0xCC + i as u64, i, i as u64, &[0xFF; 8]);
        publisher.publish_raw(&frame).expect("publish_raw");
    }
    settle();

    // Re-read the HELD payloads: the pinned SHM slots are unchanged (the
    // publisher could not reclaim them, so the later publishes used other
    // slots). This is the SHM-slot-pinned semantics.
    assert_eq!(
        held[0].payload(),
        held0_before.as_slice(),
        "held frame 0 bytes must be unchanged after later publishes"
    );
    assert_eq!(
        held[1].payload(),
        held1_before.as_slice(),
        "held frame 1 bytes must be unchanged after later publishes"
    );
}

// ============================================================
// 5. Borrow-budget contract: holding budget-many samples makes the next
//    drain Err (loud), and dropping them recovers. max_borrowed_samples() reports
//    the provisioned value.
// ============================================================

#[test]
fn borrow_budget_exhaustion_errors_then_recovers() {
    const N: usize = 5;
    let tt = TestTransport::with_buffer_size(16);
    let topic = unique_topic("borrow");

    // Provision a SMALL borrow budget (3). Subscriber creates the service.
    let mut sub = sub_with_borrow(&tt, &topic, 3, 8);
    assert_eq!(
        sub.max_borrowed_samples(),
        3,
        "max_borrowed_samples() reports the provisioned value"
    );

    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    for i in 0..N {
        let frame = build_frame(0x5150, i as u32, i as u64, &[i as u8; 4]);
        publisher.publish_raw(&frame).expect("publish_raw");
    }
    settle();

    // Hold budget-many (3) owned samples.
    let mut held: Vec<OwnedInboundSample> = Vec::new();
    let n = sub.drain_owned(3, &mut held).expect("drain_owned");
    assert_eq!(n, 3, "3 frames held (== the borrow budget)");

    // A further drain must ERROR loudly (ExceedsMaxBorrows), not hang or
    // silently drop.
    let mut over: Vec<OwnedInboundSample> = Vec::new();
    let err = sub.drain_owned(10, &mut over);
    assert!(
        err.is_err(),
        "draining beyond the borrow budget must return Err (got {err:?})"
    );
    assert!(over.is_empty(), "the over-budget drain pushed nothing");

    // Drop the held samples → budget freed → the next drain succeeds and
    // recovers the remaining 2 frames (the failed pop did NOT consume them).
    drop(held);
    let mut rest: Vec<OwnedInboundSample> = Vec::new();
    let n2 = sub
        .drain_owned(10, &mut rest)
        .expect("drain_owned after drop");
    assert_eq!(
        n2, 2,
        "after freeing the budget, the remaining 2 frames drain (no loss)"
    );
    // FIFO across the failure: sequences 3,4 survived.
    let seqs: Vec<u32> = rest
        .iter()
        .map(|o| o.wire_header().expect("header").sequence)
        .collect();
    assert_eq!(
        seqs,
        vec![3, 4],
        "the un-consumed frames survive in FIFO order"
    );
}

// ============================================================
// Shared helper: a notifying publish (used by the slot-padding test below).
// ============================================================

/// loan_proxy publish (loans a full `Vector3` slot from `max_slice_len`).
fn publish_notify(publisher: &mut CerulionPublisher, x: f64) {
    let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
    proxy.x = x;
    proxy.y = 0.0;
    proxy.z = 0.0;
}

// ============================================================
// 7. Send-at-runtime: move a drained OwnedInboundSample to another thread and
//    read its bytes there.
// ============================================================

#[test]
fn owned_sample_is_send_across_thread() {
    let tt = TestTransport::with_buffer_size(16);
    let topic = unique_topic("send");

    let mut sub = sub_with_borrow(&tt, &topic, 16, 8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);

    let frame = build_frame(
        0x1234_5678_9ABC_DEF0,
        7,
        42,
        &[0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x11],
    );
    publisher.publish_raw(&frame).expect("publish_raw");
    settle();

    let mut held: Vec<OwnedInboundSample> = Vec::new();
    let n = sub.drain_owned(1, &mut held).expect("drain_owned");
    assert_eq!(n, 1);
    let owned = held.pop().expect("one sample");

    let expected = frame.clone();
    // MOVE the owned sample across a thread boundary (compiles only because
    // OwnedInboundSample: Send) and read its payload there.
    let bytes = std::thread::spawn(move || owned.payload().to_vec())
        .join()
        .expect("reader thread");
    assert_eq!(
        bytes, expected,
        "the frame read on another thread must match the published bytes"
    );
    // Keep the transport + subscriber alive until after the join.
    drop(sub);
    drop(tt);
}

// ============================================================
// 8. payload() truncates trailing loan-slot capacity to total_size.
// ============================================================

#[test]
fn payload_truncates_slot_padding_to_total_size() {
    let tt = TestTransport::with_buffer_size(16);
    let topic = unique_topic("trunc");

    let mut sub = sub_with_borrow(&tt, &topic, 16, 8);
    // A cold publisher loans the FULL max_slice_len (256) even for a small
    // Vector3 frame (~56 bytes) — so the received slot has trailing padding.
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    publish_notify(&mut publisher, 1.5); // Vector3 via loan_proxy (delivers on drop)
    settle();

    let mut held: Vec<OwnedInboundSample> = Vec::new();
    let n = sub.drain_owned(1, &mut held).expect("drain_owned");
    assert_eq!(n, 1);
    let owned = &held[0];

    let total = owned.wire_header().expect("header parses").total_size as usize;
    assert!(
        total >= WireHeader::SIZE,
        "total_size must include the header, got {total}"
    );
    // payload() is bounded to the frame (total_size), NOT the 256-byte loan
    // slot — if truncation regressed, payload().len() would be 256.
    assert_eq!(
        owned.payload().len(),
        total,
        "payload() must be bounded to total_size, not the loan slot"
    );
    assert!(
        owned.payload().len() < 256,
        "payload() must exclude the 256-byte slot's trailing padding, got {}",
        owned.payload().len()
    );
}

// ============================================================
// 9. Partial-fill kept on a MID-drain Err (review #17): frames pushed
//    before the failing receive stay in `out`, and nothing queued is lost.
// ============================================================

#[test]
fn drain_owned_mid_drain_err_keeps_partial_fill_and_loses_nothing() {
    // borrow=3: hold 2, so the 3rd receive inside one drain_owned succeeds
    // (borrow 3 == cap) and the 4th receive attempt fails (would exceed).
    //
    // NOTE (deviation from the naive publish-3 recipe): with only 3 published,
    // the drain after f3 sees an EMPTY queue and returns Ok — no Err fires.
    // Publishing 4 guarantees a frame is PENDING at the failing receive, which
    // is the documented contract's scenario (kept partial fill + pending tail).
    const N: usize = 4;
    let tt = TestTransport::with_buffer_size(16);
    let topic = unique_topic("partial");

    let mut sub = sub_with_borrow(&tt, &topic, 3, 8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);

    let mut frames: Vec<Vec<u8>> = Vec::new();
    for i in 0..N {
        let frame = build_frame(0x9A97, i as u32, i as u64, &[(i as u8) + 1; 6]);
        publisher.publish_raw(&frame).expect("publish_raw");
        frames.push(frame);
    }
    settle();

    // Hold 2 of the 4 (borrows 1,2).
    let mut held: Vec<OwnedInboundSample> = Vec::new();
    let n = sub.drain_owned(2, &mut held).expect("drain_owned(2)");
    assert_eq!(n, 2);

    // drain_owned(5): f3 succeeds (borrow 3), the receive for f4 fails
    // (ExceedsMaxBorrows) — the partial fill (f3) must be KEPT in out2.
    let mut out2: Vec<OwnedInboundSample> = Vec::new();
    let err = sub.drain_owned(5, &mut out2);
    assert!(
        err.is_err(),
        "the 4th borrow must fail mid-drain, got {err:?}"
    );
    assert_eq!(
        out2.len(),
        1,
        "the frame drained BEFORE the failure is KEPT (the kept-on-Err contract)"
    );
    assert_eq!(
        out2[0].payload(),
        frames[2].as_slice(),
        "the kept frame byte-matches the 3rd hand-built frame"
    );

    // Recovery: drop the first two held (frees borrows) — the pending 4th
    // frame drains cleanly; nothing was lost across the partial-fill Err.
    drop(held);
    let mut rest: Vec<OwnedInboundSample> = Vec::new();
    let n3 = sub.drain_owned(10, &mut rest).expect("drain after drop");
    assert_eq!(n3, 1, "the pending tail frame survives the Err");
    assert_eq!(
        rest[0].payload(),
        frames[3].as_slice(),
        "FIFO across the failure: the 4th frame is next"
    );
}
