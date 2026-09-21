// SPDX-License-Identifier: AGPL-3.0-only
//! Adversarial tests for wire-frame bounds validation on the receive paths.
//!
//! # These are REAL iceoryx2 receive paths
//!
//! There is no in-process heap backend: `TestTransport` is a real iceoryx2
//! transport holding its own `TransportManager` rooted at a unique per-instance
//! SHM prefix. Every test below drives that `TestTransport`, so the paths under
//! test are the ordinary shared-memory ones; "in-process" here means
//! ISOLATION, not a second transport implementation. The file needs
//! neither `#[serial]` nor `--test-threads=1`.
//!
//! The single-file, multi-node pipeline programs live as
//! test code in `in_code_pipeline_test.rs`, which also pins the single clock
//! advance per `step` (item 4 below) against a hand-written oracle instead of a
//! completion marker on stdout.
//!
//! Behaviour the tests defend against:
//!
//! 1. `total_size = 0` / `total_size < WireHeader::SIZE` — a clamp such as
//!    `header.total_size as usize.max(WireHeader::SIZE)` would
//!    silently coerce any sub-header `total_size` up to 32. The frame
//!    would then be handed to the callback as a zero-byte payload (or the
//!    typed `try_view` would happily succeed on garbage).
//! 2. `total_size > raw.len()` — would compute an out-of-bounds slice.
//!    The drain rejects this rather than truncating.
//! 3. Underfill — publisher loaned `max_slice_len = 256` but the
//!    finalised frame was 56 bytes (`WireHeader::SIZE + Vector3 (24)`).
//!    Without the clamp the callback's payload slice would span
//!    `raw[32..raw.len()]` (224 bytes including pad). With it, it spans
//!    `raw[32..total_size]` (24 bytes — just the Vector3).
//! 4. An in-code pipeline must not call `clock.advance(N)` after
//!    `runtime.step(N)`: that double-counts time. The scheduler advances the
//!    clock internally. That regression is
//!    pinned in `in_code_pipeline_test.rs`, where those pipelines live.

use cerulion_core::wire::MaxSliceLen;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::SystemTime;

use cerulion_core::error::TransportError;
use cerulion_core::message::ShmMessage;
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::geometry_msgs::Vector3;

/// Monotonic counter so parallel-test instances don't collide on topic names.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/chunk_a_bounds/{base}/{nanos}/{id}")
}

/// Build a fabricated wire frame of `frame_len` bytes whose `WireHeader`
/// claims `total_size` and uses `Vector3::SCHEMA_HASH` so schema-validating
/// paths get past the schema check and into the bounds check.
fn build_frame(total_size: u32, frame_len: usize) -> Vec<u8> {
    let mut buf = vec![0u8; frame_len.max(WireHeader::SIZE)];
    let header = WireHeader {
        schema_hash: Vector3::SCHEMA_HASH,
        total_size,
        offset_table_offset: WireHeader::SIZE as u32 + Vector3::WIRE_FIXED_SIZE as u32,
        offset_table_count: Vector3::VARIABLE_FIELD_COUNT as u32,
        sequence: 0,
        timestamp_ns: 0,
    };
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    buf.truncate(frame_len);
    buf
}

// ─── Case 1: total_size == 0 ────────────────────────────────────────

#[test]
fn test_try_view_rejects_zero_total_size() {
    let topic = unique_topic("zero_total_size_try_view");
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let mut subscriber = tt.subscriber(&topic);

    // Frame is large enough to hold a real WireHeader, but the header
    // *claims* total_size == 0 — sub-header lower-bound violation.
    let frame = build_frame(0, WireHeader::SIZE + Vector3::WIRE_FIXED_SIZE);
    publisher.publish_raw(&frame).expect("publish_raw");

    let result = subscriber.try_view::<Vector3, _>(|_view| {
        panic!("closure must NOT run when total_size = 0");
    });
    match result {
        Err(TransportError::Deserialization { reason, .. }) => {
            assert!(
                reason.contains("total_size"),
                "error reason should mention total_size, got: {reason}"
            );
        }
        other => panic!("expected Deserialization error for total_size = 0, got {other:?}"),
    }
}

#[test]
fn test_drain_samples_skips_zero_total_size() {
    let topic = unique_topic("zero_total_size_drain");
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let subscriber = tt.subscriber(&topic);

    let frame = build_frame(0, WireHeader::SIZE + Vector3::WIRE_FIXED_SIZE);
    publisher.publish_raw(&frame).expect("publish_raw");

    let mut callback_runs = 0usize;
    let count = subscriber
        .try_receive(|_msg| {
            callback_runs += 1;
        })
        .expect("try_receive should not error on a malformed frame; it skips");
    assert_eq!(
        count, 0,
        "drain should skip the malformed frame (returned count != 0)"
    );
    assert_eq!(
        callback_runs, 0,
        "callback must not fire on a frame whose total_size == 0"
    );
}

// ─── Case 2: total_size == WireHeader::SIZE - 1 (sub-header) ────────

#[test]
fn test_try_view_rejects_sub_header_total_size() {
    let topic = unique_topic("sub_header_try_view");
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let mut subscriber = tt.subscriber(&topic);

    // Header claims total_size = 31 (one byte short of the header itself).
    let frame = build_frame(
        (WireHeader::SIZE - 1) as u32,
        WireHeader::SIZE + Vector3::WIRE_FIXED_SIZE,
    );
    publisher.publish_raw(&frame).expect("publish_raw");

    let result = subscriber.try_view::<Vector3, _>(|_view| {
        panic!("closure must NOT run when total_size < WireHeader::SIZE");
    });
    match result {
        Err(TransportError::Deserialization { reason, .. }) => {
            assert!(
                reason.contains("total_size") || reason.contains("31"),
                "error reason should mention bounds, got: {reason}"
            );
        }
        other => panic!("expected Deserialization for sub-header total_size, got {other:?}"),
    }
}

#[test]
fn test_drain_samples_skips_sub_header_total_size() {
    let topic = unique_topic("sub_header_drain");
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let subscriber = tt.subscriber(&topic);

    let frame = build_frame(
        (WireHeader::SIZE - 1) as u32,
        WireHeader::SIZE + Vector3::WIRE_FIXED_SIZE,
    );
    publisher.publish_raw(&frame).expect("publish_raw");

    let mut callback_runs = 0usize;
    let count = subscriber
        .try_receive(|_msg| {
            callback_runs += 1;
        })
        .expect("try_receive must not error on the malformed frame");
    assert_eq!(count, 0, "drain should skip sub-header frame");
    assert_eq!(callback_runs, 0, "callback must not fire");
}

// ─── Case 3: total_size > raw.len() (oversized) ─────────────────────

#[test]
fn test_try_view_rejects_oversized_total_size() {
    let topic = unique_topic("oversized_try_view");
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let mut subscriber = tt.subscriber(&topic);

    // Frame is 56 bytes, but the header claims 1024 — would have read
    // 968 bytes past the buffer end if not bounds-checked.
    let frame_len = WireHeader::SIZE + Vector3::WIRE_FIXED_SIZE;
    let frame = build_frame(1024, frame_len);
    publisher.publish_raw(&frame).expect("publish_raw");

    let result = subscriber.try_view::<Vector3, _>(|_view| {
        panic!("closure must NOT run when total_size > raw.len()");
    });
    match result {
        Err(TransportError::Deserialization { reason, .. }) => {
            assert!(
                reason.contains("total_size") || reason.contains("1024"),
                "error reason should mention bounds, got: {reason}"
            );
        }
        other => panic!("expected Deserialization for oversized total_size, got {other:?}"),
    }
}

#[test]
fn test_drain_samples_skips_oversized_total_size() {
    let topic = unique_topic("oversized_drain");
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let subscriber = tt.subscriber(&topic);

    let frame_len = WireHeader::SIZE + Vector3::WIRE_FIXED_SIZE;
    let frame = build_frame(1024, frame_len);
    publisher.publish_raw(&frame).expect("publish_raw");

    let mut callback_runs = 0usize;
    let count = subscriber
        .try_receive(|_msg| {
            callback_runs += 1;
        })
        .expect("try_receive must not error on oversized frame");
    assert_eq!(count, 0);
    assert_eq!(callback_runs, 0);
}

// ─── Case 4: legitimate underfill — clamp must trim trailing pad ────

/// This is the load-bearing distinguishing test for the `total_size` clamp.
///
/// Without the clamp (`payload_len = raw.len() - WireHeader::SIZE` or
/// `&raw[WireHeader::SIZE..]`): callback would observe
/// `max_slice_len - 32 = 224` bytes of trailing pad.
/// With it (`payload_len = total_size - WireHeader::SIZE`,
/// `&raw[WireHeader::SIZE..total_size]`): callback observes exactly
/// `Vector3::WIRE_FIXED_SIZE = 24` bytes.
#[test]
fn test_drain_samples_clamps_payload_to_total_size() {
    let topic = unique_topic("underfill_clamp");
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let subscriber = tt.subscriber(&topic);

    // A realistic underfill: publisher loaned 256 bytes (max_slice_len)
    // but the actual finalised frame is just `WireHeader + Vector3`.
    let max_slice_len: usize = 256;
    let actual_total: usize = WireHeader::SIZE + Vector3::WIRE_FIXED_SIZE; // 56
    assert!(
        actual_total < max_slice_len,
        "test setup invalid: pad must be > 0"
    );

    // Build the frame at the FULL max_slice_len, but the WireHeader
    // says total_size = 56. Fill the pad with a recognisable byte so
    // a regression test can prove no pad bytes leak.
    const PAD_SENTINEL: u8 = 0xCD;
    let mut frame = vec![PAD_SENTINEL; max_slice_len];
    let header = WireHeader {
        schema_hash: Vector3::SCHEMA_HASH,
        total_size: actual_total as u32,
        offset_table_offset: WireHeader::SIZE as u32 + Vector3::WIRE_FIXED_SIZE as u32,
        offset_table_count: Vector3::VARIABLE_FIELD_COUNT as u32,
        sequence: 0,
        timestamp_ns: 0,
    };
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    // Zero out the Vector3 fixed-section so the legitimate payload
    // bytes are distinct from the pad sentinel.
    for byte in &mut frame[WireHeader::SIZE..actual_total] {
        *byte = 0;
    }
    publisher.publish_raw(&frame).expect("publish_raw");

    let mut observed_payload_len: Option<usize> = None;
    let mut observed_has_sentinel = false;
    let count = subscriber
        .try_receive(|msg| {
            observed_payload_len = Some(msg.payload().len());
            observed_has_sentinel = msg.payload().contains(&PAD_SENTINEL);
        })
        .expect("try_receive");

    assert_eq!(count, 1, "the legitimate frame should be delivered");
    assert_eq!(
        observed_payload_len,
        Some(Vector3::WIRE_FIXED_SIZE),
        "callback must see EXACTLY Vector3::WIRE_FIXED_SIZE bytes \
         (an unclamped slice would leak {} bytes of pad)",
        max_slice_len - actual_total
    );
    assert!(
        !observed_has_sentinel,
        "callback payload must not contain the 0xCD pad sentinel \
         (`&raw[WireHeader::SIZE..]` would include pad)"
    );
}

/// Same property on the typed `try_view` path — the underlying SHM
/// reader is bounded by `payload_len = total_size - WireHeader::SIZE`
/// so a Vector3 reader sees exactly its 24-byte fixed section.
#[test]
fn test_try_view_clamps_payload_to_total_size() {
    let topic = unique_topic("underfill_try_view");
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let mut subscriber = tt.subscriber(&topic);

    // Same construction as the drain test: 256 byte buffer, 56 byte total_size.
    let max_slice_len: usize = 256;
    let actual_total: usize = WireHeader::SIZE + Vector3::WIRE_FIXED_SIZE;
    let mut frame = vec![0xCDu8; max_slice_len];
    let header = WireHeader {
        schema_hash: Vector3::SCHEMA_HASH,
        total_size: actual_total as u32,
        offset_table_offset: WireHeader::SIZE as u32 + Vector3::WIRE_FIXED_SIZE as u32,
        offset_table_count: Vector3::VARIABLE_FIELD_COUNT as u32,
        sequence: 0,
        timestamp_ns: 0,
    };
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    // Write a known Vector3 (1.0, 2.0, 3.0) into the fixed section
    // just so the typed read returns predictable values.
    frame[WireHeader::SIZE..WireHeader::SIZE + 8].copy_from_slice(&1.0_f64.to_le_bytes());
    frame[WireHeader::SIZE + 8..WireHeader::SIZE + 16].copy_from_slice(&2.0_f64.to_le_bytes());
    frame[WireHeader::SIZE + 16..WireHeader::SIZE + 24].copy_from_slice(&3.0_f64.to_le_bytes());
    publisher.publish_raw(&frame).expect("publish_raw");

    let observed = subscriber
        .try_view::<Vector3, _>(|view| (view.x, view.y, view.z))
        .expect("try_view");
    assert_eq!(
        observed,
        Some((1.0, 2.0, 3.0)),
        "typed read must surface the Vector3 written into the underfilled buffer"
    );
}
