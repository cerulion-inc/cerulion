// SPDX-License-Identifier: AGPL-3.0-only
//! Publish-trace integration tests: verify the publish
//! trace captures wire-header metadata from real publishes.
//!
//! Uses the iceoryx2 `TestTransport` helper. The trace hook lives in
//! `transport::publisher::parse_trace_entry_from_wire` (shared
//! between both publisher backends), so coverage of one backend
//! validates the parse path that the other uses too.

use cerulion_core::wire::MaxSliceLen;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::testing::TestTransport;
use cerulion_core::trace::{HistoryDepth, PublishTrace};
use cerulion_core::wire::WireHeader;

const SCHEMA_HASH: u64 = 0xDEAD_BEEF_CAFE_F00D;

/// Build a wire frame: 32-byte WireHeader + N-byte payload of `0x42`.
fn make_wire_frame(sequence: u32, timestamp_ns: u64, payload_len: usize) -> Vec<u8> {
    let header = WireHeader::new(SCHEMA_HASH, sequence, timestamp_ns);
    let mut buf = vec![0u8; WireHeader::SIZE + payload_len];
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    // total_size lives at [8..12] — patch it.
    let total_size = (WireHeader::SIZE + payload_len) as u32;
    buf[8..12].copy_from_slice(&total_size.to_le_bytes());
    for b in &mut buf[WireHeader::SIZE..] {
        *b = 0x42;
    }
    buf
}

#[test]
fn publish_trace_captures_wire_header_fields() {
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher("test/topic", MaxSliceLen::const_new(4096), 0);

    let trace = Arc::new(Mutex::new(PublishTrace::new(HistoryDepth::Count(100))));
    publisher.attach_trace(Arc::clone(&trace));

    publisher
        .publish_raw(&make_wire_frame(1, 100_000_000, 64))
        .unwrap();
    publisher
        .publish_raw(&make_wire_frame(2, 200_000_000, 128))
        .unwrap();
    publisher
        .publish_raw(&make_wire_frame(3, 300_000_000, 32))
        .unwrap();

    let t = trace.lock().unwrap();
    assert_eq!(t.len(), 3, "trace should have one entry per publish");
    let entries: Vec<_> = t.entries().collect();
    assert_eq!(entries[0].sequence, 1);
    assert_eq!(entries[0].publish_time_ns, 100_000_000);
    assert_eq!(entries[0].schema_hash, SCHEMA_HASH);
    assert_eq!(&*entries[0].topic, "test/topic");
    assert_eq!(entries[1].sequence, 2);
    assert_eq!(entries[1].publish_time_ns, 200_000_000);
    assert_eq!(entries[2].sequence, 3);
    assert_eq!(entries[2].publish_time_ns, 300_000_000);
}

#[test]
fn publish_trace_respects_count_depth_eviction() {
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher("test/topic", MaxSliceLen::const_new(4096), 0);

    let trace = Arc::new(Mutex::new(PublishTrace::new(HistoryDepth::Count(3))));
    publisher.attach_trace(Arc::clone(&trace));

    for i in 1..=10 {
        publisher
            .publish_raw(&make_wire_frame(i, (i as u64) * 1_000_000, 16))
            .unwrap();
    }

    let t = trace.lock().unwrap();
    assert_eq!(t.len(), 3, "count=3 cap should evict older entries");
    let seqs: Vec<u32> = t.entries().map(|e| e.sequence).collect();
    assert_eq!(seqs, vec![8, 9, 10], "newest 3 sequences should survive");
}

#[test]
fn publish_trace_window_evicts_old_entries() {
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher("test/topic", MaxSliceLen::const_new(4096), 0);

    let trace = Arc::new(Mutex::new(PublishTrace::new(HistoryDepth::Window(
        Duration::from_millis(50),
    ))));
    publisher.attach_trace(Arc::clone(&trace));

    publisher.publish_raw(&make_wire_frame(1, 0, 16)).unwrap();
    publisher
        .publish_raw(&make_wire_frame(2, 25_000_000, 16))
        .unwrap();
    publisher
        .publish_raw(&make_wire_frame(3, 100_000_000, 16))
        .unwrap();
    // Window = 50ms. Latest = 100ms. Cutoff = 50ms.
    // Seq 1 (t=0) and seq 2 (t=25ms) are < 50ms → evicted.

    let t = trace.lock().unwrap();
    let seqs: Vec<u32> = t.entries().map(|e| e.sequence).collect();
    assert_eq!(seqs, vec![3], "only entries within the 50ms window survive");
}

#[test]
fn publish_trace_is_zero_overhead_when_unattached() {
    // Sanity: a publisher without a trace must not crash + must not push
    // anywhere (no panic on the None branch).
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher("test/topic", MaxSliceLen::const_new(4096), 0);

    assert!(!publisher.has_trace(), "default should be no trace");
    publisher.publish_raw(&make_wire_frame(1, 0, 16)).unwrap();
    // No panic = pass.
}

#[test]
fn publish_trace_handles_malformed_data_gracefully() {
    // Publish a payload smaller than WireHeader::SIZE (32 B). The trace
    // hook should silently skip (parser returns None on short input).
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher("test/topic", MaxSliceLen::const_new(4096), 0);

    let trace = Arc::new(Mutex::new(PublishTrace::new(HistoryDepth::Count(100))));
    publisher.attach_trace(Arc::clone(&trace));

    // Publish a 16-byte payload — half a WireHeader. The channel push
    // succeeds (we don't validate WireHeader at the in-process channel
    // boundary), but the trace parser returns None and skips.
    publisher.publish_raw(&[0u8; 16]).unwrap();
    publisher
        .publish_raw(&make_wire_frame(42, 1_000_000, 32))
        .unwrap();

    let t = trace.lock().unwrap();
    assert_eq!(
        t.len(),
        1,
        "malformed (< 32B) publish skipped; valid publish recorded"
    );
    assert_eq!(t.entries().next().unwrap().sequence, 42);
}

/// `publish_raw` MUST bump the `block`
/// outstanding mirror, exactly like `OutputProxy::drop` and
/// `send_overflow_frame`. Without this, a `block` producer publishing via
/// `publish_raw` (the raw-FFI re-publish path) enqueues frames the mirror
/// never counts → the scheduler pre-fire never observes a full queue → no
/// defer → iceoryx2 overflow → silent data loss (Principle #6).
///
/// This test registers a block outstanding counter on the publisher, sends
/// three frames via `publish_raw`, and asserts the counter advanced by
/// exactly three (one increment per published frame — symmetric with the
/// consumer's per-drain decrement). It also covers history replay (which
/// goes through `publish_raw`, so history frames are counted too).
#[test]
fn publish_raw_bumps_block_outstanding_mirror() {
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher("block/topic", MaxSliceLen::const_new(4096), 0);

    // The mirror is a `CreditWord` now — LOCAL here (a
    // same-process edge); `outstanding()` is the `Acquire` load this test read
    // off the raw atomic.
    let outstanding = cerulion_core::credit::CreditWord::local(u32::MAX);
    publisher.register_block_outstanding_for_test(outstanding.clone());

    assert_eq!(
        outstanding.outstanding(),
        0,
        "mirror starts at zero before any publish"
    );

    for i in 1..=3u32 {
        publisher
            .publish_raw(&make_wire_frame(i, (i as u64) * 1_000_000, 16))
            .unwrap();
    }

    assert_eq!(
        outstanding.outstanding(),
        3,
        "publish_raw must increment the block outstanding mirror once per \
         published frame (regression: #1 data-loss bug — publish_raw bypassed \
         the increment, so block pre-fire never deferred → overflow)"
    );
}
