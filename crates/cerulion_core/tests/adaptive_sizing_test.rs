// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for the adaptive `loan_proxy`
//! sizing.
//!
//! These tests exercise the publisher-side sliding window through real
//! `loan_proxy` + publish cycles using the iceoryx2 backend via
//! `TestTransport` (each test owns an isolated per-instance SHM prefix,
//! so they remain parallel-test-safe). iceoryx2 is the only backend
//! (the in-process heap one was deleted), so this file is the direct
//! coverage for the production sliding window; `adaptive_sizing_iceoryx2_test.rs`
//! mirrors the load-bearing arms on the process-global singleton.
//!
//! # Coverage
//!
//! Per the adaptive-sizing test plan: sliding window happy path,
//! window shrink/grow, window-per-publisher independence, ring-buffer
//! wrap, determinism, error variant introduction. The in-tick
//! overflow-redirect tests (heap fallback, `PayloadTooLarge` from the
//! overflow path) live in the `overflow_redirect_*` test files.

use cerulion_core::wire::MaxSliceLen;

use cerulion_core::testing::TestTransport;
use cerulion_core::transport::adaptive_sizer::{HEADROOM_DEN, HEADROOM_NUM, WINDOW_SIZE};
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::std_msgs::String as RosString;

/// Captured wire frame: `(header, payload_bytes)`. Reassemble via
/// `header.write_to_buf` + payload to get the full byte slice.
type CapturedFrame = (WireHeader, Vec<u8>);

/// Reassemble a captured frame into a full wire-byte vector.
fn reassemble(frame: &CapturedFrame) -> Vec<u8> {
    let mut buf = vec![0u8; WireHeader::SIZE + frame.1.len()];
    frame.0.write_to_buf(&mut buf[..WireHeader::SIZE]);
    buf[WireHeader::SIZE..].copy_from_slice(&frame.1);
    buf
}

// ============================================================
// Helpers
// ============================================================

/// Build an iceoryx2 publisher with the given `max_slice_len`. No
/// history. Returns the owning `TestTransport` alongside the publisher
/// — the caller must keep `tt` alive for the publisher's lifetime
/// (`let (_tt, mut pubr) = make_publisher(...)`).
fn make_publisher(max_slice_len: u32) -> (TestTransport, CerulionPublisher) {
    let tt = TestTransport::with_buffer_size(16);
    let pubr = tt.publisher("test/adaptive", MaxSliceLen::const_new(max_slice_len), 0);
    (tt, pubr)
}

/// Build with `TestTransport`'s deterministic VirtualClock so wire-byte
/// determinism tests don't see jitter from timestamps. The transport's
/// internal clock starts at t=0 and never advances, so `timestamp_ns` is
/// 0 for every publish — and identical across two separate
/// `TestTransport`s (the determinism contract these tests rely on).
///
/// `subscriber_buf` is the capacity the caller's `tt.subscriber(...)`
/// channels need (the iceoryx2 subscriber buffer
/// capacity). It is supplied per-test so a
/// publish-all-then-drain test can size the buffer to hold every frame.
fn make_buffered_publisher(
    topic: &str,
    max_slice_len: u32,
    subscriber_buf: usize,
) -> (TestTransport, CerulionPublisher) {
    let tt = TestTransport::with_buffer_size(subscriber_buf);
    let pubr = tt.publisher(topic, MaxSliceLen::const_new(max_slice_len), 0);
    (tt, pubr)
}

/// Publish a fixed-schema `Vector3` (24-byte fixed payload) once.
/// Returns the wire frame size (= header + WIRE_FIXED_SIZE).
fn publish_vector3(pubr: &mut CerulionPublisher, x: f64) -> usize {
    use cerulion_core::message::ShmMessage;
    {
        let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
        // Vector3's writer derefs to `&mut Vector3Shm`; field access
        // lands directly in SHM.
        proxy.x = x;
        proxy.y = 0.0;
        proxy.z = 0.0;
        // proxy drops here; publish + record happens.
    }
    WireHeader::SIZE + <Vector3 as ShmMessage>::WIRE_FIXED_SIZE
}

/// Publish a variable-schema `RosString` with the given payload bytes.
/// Returns the actual wire frame size (= header + offset table + bytes).
fn publish_string(pubr: &mut CerulionPublisher, payload: &[u8]) -> usize {
    let mut proxy = pubr.loan_proxy::<RosString>().expect("loan");
    proxy
        .set_data(std::str::from_utf8(payload).expect("ascii"))
        .expect("set_data fits in loan");
    drop(proxy);
    // Wire frame: header + WIRE_FIXED_SIZE (0 for std_msgs/String)
    // + 8-byte offset table entry + payload bytes.
    WireHeader::SIZE + 8 + payload.len()
}

// ============================================================
// Cold publisher: no shrink before window fills
// ============================================================

#[test]
fn adaptive_first_tick_uses_max_slice_len() {
    let (_tt, pubr) = make_publisher(16 * 1024 * 1024);
    // Cold publisher (zero records) returns max_slice_len for any
    // min_required ≤ max_slice_len.
    assert_eq!(
        pubr.adaptive_loan_size_for_min_required(40),
        16 * 1024 * 1024
    );
    assert!(!pubr.sizer_warm());
}

#[test]
fn adaptive_one_tick_short_of_warm_still_max_slice_len() {
    let (_tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    for _ in 0..(WINDOW_SIZE - 1) {
        publish_vector3(&mut pubr, 1.0);
    }
    assert!(!pubr.sizer_warm());
    assert_eq!(
        pubr.adaptive_loan_size_for_min_required(40),
        16 * 1024 * 1024
    );
}

#[test]
fn adaptive_warms_after_window_size_publishes() {
    let (_tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    for _ in 0..WINDOW_SIZE {
        publish_vector3(&mut pubr, 1.0);
    }
    assert!(pubr.sizer_warm());
}

// ============================================================
// Warm publisher: shrinks toward recent_max × headroom
// ============================================================

#[test]
fn adaptive_steady_state_loan_smaller_than_max_after_warmup() {
    use cerulion_core::message::ShmMessage;

    let (_tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    // Vector3 wire frame is exactly header + WIRE_FIXED_SIZE.
    let frame = WireHeader::SIZE + <Vector3 as ShmMessage>::WIRE_FIXED_SIZE;
    for _ in 0..WINDOW_SIZE {
        publish_vector3(&mut pubr, 1.0);
    }
    assert!(pubr.sizer_warm());
    assert_eq!(pubr.sizer_recent_max(), frame as u32);
    let expected_loan = (frame as u64 * HEADROOM_NUM / HEADROOM_DEN) as u32;
    // `min_required` computed in u32 to match the
    // sizer's new u32 interface.
    let min_required: u32 = (WireHeader::SIZE + <Vector3 as ShmMessage>::WIRE_FIXED_SIZE) as u32;
    assert_eq!(
        pubr.adaptive_loan_size_for_min_required(min_required),
        expected_loan.max(min_required)
    );
    // Sanity: loan dropped 100×+ from the 16 MiB ceiling.
    assert!(
        pubr.adaptive_loan_size_for_min_required(min_required) < 16 * 1024,
        "warm Vector3 loan should be ≪ 16 KiB, got {}",
        pubr.adaptive_loan_size_for_min_required(min_required)
    );
}

#[test]
fn adaptive_steady_state_loan_constant_tick_over_tick() {
    let (_tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    for _ in 0..WINDOW_SIZE {
        publish_vector3(&mut pubr, 1.0);
    }
    let first = pubr.adaptive_loan_size_for_min_required(64);
    // Publish 50 more — same payload size — and the loan size must
    // not drift.
    for _ in 0..50 {
        publish_vector3(&mut pubr, 1.0);
        assert_eq!(pubr.adaptive_loan_size_for_min_required(64), first);
    }
}

#[test]
fn adaptive_in_process_loan_shrinks_after_warmup() {
    // The loan the publisher asks iceoryx2 for should shrink along
    // with the sliding window. (The `in_process` in the test name
    // is a leftover: there is a single iceoryx2 backend, and
    // the shrinking quantity is the loan requested from it.)
    // This pins the regression adaptive sizing prevents:
    // a full-size loan (16 MiB per loan when YAML omits
    // max_slice_len) on every publish.
    let (_tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    let cold_loan = pubr.adaptive_loan_size_for_min_required(40);
    for _ in 0..WINDOW_SIZE {
        publish_vector3(&mut pubr, 1.0);
    }
    let warm_loan = pubr.adaptive_loan_size_for_min_required(40);
    assert!(
        warm_loan < cold_loan,
        "warm loan ({warm_loan}) should be ≪ cold loan ({cold_loan})"
    );
    assert!(
        warm_loan < cold_loan / 100,
        "warm loan should drop 100× from cold; got cold={cold_loan} warm={warm_loan}"
    );
}

// ============================================================
// Window grows + shrinks back
// ============================================================

#[test]
fn adaptive_window_grows_with_payload_size_growth() {
    // This path (no in-tick overflow redirect) requires that any
    // single tick's payload fits in the current adaptive loan. To
    // exercise window growth without forcing a `ProxyBufferTooSmall`
    // failure, publish a sequence of payloads that grows by less
    // than the headroom factor (1.5×) per tick. Each successful
    // publish records its size, the next loan grows, and the next
    // payload still fits.
    let (_tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    // Warm with a baseline so the publisher leaves the cold path.
    for _ in 0..WINDOW_SIZE {
        publish_string(&mut pubr, b"warmup-payload");
    }
    let baseline_loan = pubr.adaptive_loan_size_for_min_required(40);

    // Publish a series of growing payloads. Each ~1.3× the previous
    // — well within the 1.5× headroom multiplier so each one still
    // fits in the prior tick's loan.
    let mut size = 16usize;
    for _ in 0..10 {
        let payload = vec![b'L'; size];
        publish_string(&mut pubr, &payload);
        size = (size * 13) / 10; // grow by 1.3×
    }
    let grown_loan = pubr.adaptive_loan_size_for_min_required(40);
    assert!(
        grown_loan > baseline_loan,
        "loan should grow as recent payloads grow: baseline={baseline_loan} grown={grown_loan}"
    );
}

#[test]
fn adaptive_window_shrinks_back_after_large_payload_overwritten() {
    // Warm at a LARGER baseline first, then shrink the publish
    // sizes. The window's `recent_max` should drop as the larger
    // payloads age out of the ring buffer. (This path can't
    // exercise a jump-from-small without a ProxyBufferTooSmall
    // failure — see `adaptive_window_grows_with_payload_size_growth`
    // for the convergence-via-growth-sequence pattern.)
    let (_tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    let large_payload = vec![b'L'; 500];
    // Warm with the LARGE payload first — establishes a high
    // recent_max + a generous loan size.
    for _ in 0..WINDOW_SIZE {
        publish_string(&mut pubr, &large_payload);
    }
    let large_loan = pubr.adaptive_loan_size_for_min_required(40);

    // WINDOW_SIZE small publishes. The large payloads age out of the
    // ring buffer one by one; once all are evicted, recent_max falls
    // to the small payload's frame size and the loan shrinks.
    for _ in 0..WINDOW_SIZE {
        publish_string(&mut pubr, b"s");
    }
    let recovered_loan = pubr.adaptive_loan_size_for_min_required(40);
    assert!(
        recovered_loan < large_loan,
        "after WINDOW_SIZE small publishes the loan must shrink: large={large_loan} recovered={recovered_loan}"
    );
    // Quantitative: small payload frame = header + offset_table + 1 = 41
    // bytes; recent_max = 41; loan = 41 × 1.5 = 61. Floor at min_required.
    assert!(
        recovered_loan < 200,
        "recovered loan should drop to ~60-100 bytes; got {recovered_loan}"
    );
}

#[test]
fn adaptive_window_floor_at_min_required() {
    // Even on a tiny-payload-only history, the loan never drops below
    // min_required (the schema's wire header + fixed + offset table).
    let (_tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    for _ in 0..WINDOW_SIZE {
        publish_string(&mut pubr, b"");
    }
    let min_required = 40;
    assert!(
        pubr.adaptive_loan_size_for_min_required(min_required) >= min_required,
        "loan must respect min_required floor"
    );
}

#[test]
fn adaptive_max_slice_len_caps_loan() {
    // Pathologically high recorded sizes — the headroom multiplier
    // would push the loan past max_slice_len. Cap to max.
    let (_tt, mut pubr) = make_publisher(64 * 1024);
    // Publish 16 strings just under the cap. 16 × ~64 KiB recent_max →
    // 1.5× = 96 KiB → caps at 64 KiB.
    for _ in 0..WINDOW_SIZE {
        let payload = vec![b'P'; 60 * 1024];
        publish_string(&mut pubr, &payload);
    }
    assert_eq!(
        pubr.adaptive_loan_size_for_min_required(40),
        64 * 1024,
        "loan capped at max_slice_len"
    );
}

// ============================================================
// Per-publisher independence
// ============================================================

#[test]
fn adaptive_per_publisher_window_independent() {
    // Two publishers on different topics, different size profiles —
    // one steady at small payloads, one at large. Each maintains its
    // own sliding window; they don't cross-pollute.
    let (_small_tt, mut small_pubr) = make_publisher(16 * 1024 * 1024);
    let (_large_tt, mut large_pubr) = make_publisher(16 * 1024 * 1024);

    for _ in 0..WINDOW_SIZE {
        publish_string(&mut small_pubr, b"x");
        publish_string(&mut large_pubr, &vec![b'L'; 8000]);
    }

    let small_loan = small_pubr.adaptive_loan_size_for_min_required(40);
    let large_loan = large_pubr.adaptive_loan_size_for_min_required(40);
    assert!(
        large_loan > small_loan * 100,
        "publishers must maintain independent windows: small={small_loan} large={large_loan}"
    );
}

// ============================================================
// Determinism
// ============================================================

#[test]
fn adaptive_byte_for_byte_determinism_within_process() {
    // Two publishers, identical payload sequences, identical clocks.
    // Per-tick wire bytes must be byte-equal even though both
    // publishers' sliding windows transition cold → warm in lockstep.
    let (tt_a, mut pub_a) = make_buffered_publisher("t/a", 16 * 1024 * 1024, 2 * WINDOW_SIZE);
    let (tt_b, mut pub_b) = make_buffered_publisher("t/b", 16 * 1024 * 1024, 2 * WINDOW_SIZE);
    let sub_a = tt_a.subscriber("t/a");
    let sub_b = tt_b.subscriber("t/b");

    let payloads: Vec<Vec<u8>> = (0..(2 * WINDOW_SIZE))
        .map(|i| vec![i as u8 + 0x20; (i % 17) + 1])
        .collect();

    for p in &payloads {
        publish_string(&mut pub_a, p);
        publish_string(&mut pub_b, p);
    }

    // Drain both subscribers' frames and compare bit-for-bit AFTER
    // masking out the timestamp_ns (which depends on wall-clock and
    // would differ across two simulated clocks).
    let mut frames_a: Vec<CapturedFrame> = Vec::new();
    let _ = sub_a
        .try_receive(|msg| frames_a.push((*msg.header(), msg.payload().to_vec())))
        .expect("recv");
    let mut frames_b: Vec<CapturedFrame> = Vec::new();
    let _ = sub_b
        .try_receive(|msg| frames_b.push((*msg.header(), msg.payload().to_vec())))
        .expect("recv");
    assert_eq!(frames_a.len(), payloads.len());
    assert_eq!(frames_b.len(), payloads.len());
    // VirtualClock starts at 0 for both; timestamps should be
    // identical → full byte-equality.
    for (i, (a, b)) in frames_a.iter().zip(frames_b.iter()).enumerate() {
        assert_eq!(
            reassemble(a),
            reassemble(b),
            "frame {i} differs across publishers"
        );
    }
}

#[test]
fn adaptive_window_state_does_not_leak_into_wire_bytes() {
    // One publisher cold, one publisher pre-warmed with garbage —
    // same input data must produce the same wire frame. This is the
    // load-bearing determinism contract for adaptive sizing (sliding-window
    // state is internal and does NOT leak into wire bytes).
    let (tt_cold, mut cold_pub) = make_buffered_publisher("t/cold", 16 * 1024 * 1024, 4);
    let (tt_warm, mut warm_pub) = make_buffered_publisher("t/warm", 16 * 1024 * 1024, 4);
    let cold_sub = tt_cold.subscriber("t/cold");
    let warm_sub = tt_warm.subscriber("t/warm");

    // Pre-warm only `warm_pub` with WINDOW_SIZE × 1000-byte publishes
    // before draining its subscriber channel (so the recorded state
    // diverges from `cold_pub`).
    for _ in 0..WINDOW_SIZE {
        publish_string(&mut warm_pub, &vec![b'W'; 1000]);
    }
    // Drain the warmup frames out of warm_sub so they don't pollute
    // the comparison below.
    let _ = warm_sub.try_receive(|_| {}).expect("drain");
    assert!(warm_pub.sizer_warm());
    assert!(!cold_pub.sizer_warm());

    // Now publish the SAME input on both. Cold-pub still loans
    // max_slice_len; warm-pub loans recent_max × 1.5. The wire frame
    // bytes (after `OutputProxy::Drop` truncation) must be identical.
    let payload = b"hello-world";
    let mut cold_pubr = cold_pub.loan_proxy::<RosString>().expect("loan");
    cold_pubr
        .set_data(std::str::from_utf8(payload).unwrap())
        .expect("set");
    drop(cold_pubr);
    let mut warm_pubr = warm_pub.loan_proxy::<RosString>().expect("loan");
    warm_pubr
        .set_data(std::str::from_utf8(payload).unwrap())
        .expect("set");
    drop(warm_pubr);

    let mut cold_frames: Vec<CapturedFrame> = Vec::new();
    let _ = cold_sub
        .try_receive(|m| cold_frames.push((*m.header(), m.payload().to_vec())))
        .expect("recv");
    let mut warm_frames: Vec<CapturedFrame> = Vec::new();
    let _ = warm_sub
        .try_receive(|m| warm_frames.push((*m.header(), m.payload().to_vec())))
        .expect("recv");
    assert_eq!(cold_frames.len(), 1);
    assert_eq!(warm_frames.len(), 1);
    // Sequence numbers + timestamps may differ (different publisher
    // state); compare structurally. The load-bearing assertion is
    // payload equality + header field parity (schema_hash,
    // total_size, offset_table_offset, offset_table_count).
    let (cold_h, cold_p) = &cold_frames[0];
    let (warm_h, warm_p) = &warm_frames[0];
    assert_eq!(
        cold_p, warm_p,
        "payload bytes differ — sliding-window state leaked into wire bytes!"
    );
    assert_eq!(cold_h.schema_hash, warm_h.schema_hash);
    assert_eq!(cold_h.total_size, warm_h.total_size);
    assert_eq!(cold_h.offset_table_offset, warm_h.offset_table_offset);
    assert_eq!(cold_h.offset_table_count, warm_h.offset_table_count);
}

#[test]
fn adaptive_payload_round_trips_byte_for_byte() {
    // Direct: publish a payload through the adaptive path and verify
    // the subscriber sees exactly what was written, regardless of
    // sliding-window state. This covers the basic correctness
    // property the determinism contract relies on.
    //
    // Warmup payload size must be at least as large as the canary
    // (this path has no in-tick overflow redirect, so a canary
    // larger than the warm loan would hit ProxyBufferTooSmall).
    let (tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    let sub = tt.subscriber("test/adaptive");
    let canary_payload = b"the-quick-brown-fox-jumps-over-the-lazy-dog";
    // Warm with a payload at least as large as the canary so the
    // adaptive loan stays large enough.
    let warmup_payload = vec![b'W'; canary_payload.len()];
    for _ in 0..WINDOW_SIZE {
        publish_string(&mut pubr, &warmup_payload);
    }
    let _ = sub.try_receive(|_| {}).expect("drain");

    // Now publish the canary payload.
    let mut proxy = pubr.loan_proxy::<RosString>().expect("loan");
    proxy
        .set_data(std::str::from_utf8(canary_payload).unwrap())
        .expect("set");
    drop(proxy);

    let mut received: Vec<CapturedFrame> = Vec::new();
    let _ = sub
        .try_receive(|m| received.push((*m.header(), m.payload().to_vec())))
        .expect("recv");
    assert_eq!(received.len(), 1);
    let payload_bytes = &received[0].1;
    // payload_bytes is everything after the 32-byte WireHeader:
    // WIRE_FIXED_SIZE (0 for std_msgs/String) + offset table (8) +
    // user payload. The string bytes live at offset 8.
    let user_offset = 8;
    assert_eq!(&payload_bytes[user_offset..], canary_payload);
}

// ============================================================
// Drop-path coverage: record_payload_size is NOT called on
// failed/aborted ticks (the convergence policy).
// ============================================================

#[test]
fn adaptive_does_not_record_on_missing_variable_field_drop() {
    // The convergence policy is "learn
    // from successful publishes only." A `OutputProxy` dropped
    // without writing all required variable fields should NOT
    // contribute to the sliding window — otherwise a partial-write
    // tick (e.g., user `?`-returns midway) would record a too-small
    // payload size and shrink the next loan inappropriately.
    //
    // `RosString` has 1 variable field (`data`). Loan + drop without
    // calling `set_data` triggers the missing-variable-field path
    // in `OutputProxy::Drop` (logged at error level; record skipped).
    let (_tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    {
        let _proxy = pubr.loan_proxy::<RosString>().expect("loan");
        // Drop without set_data — `all_variables_written` returns
        // false, Drop early-returns before record_payload_size.
    }
    assert_eq!(
        pubr.sizer_recent_max(),
        0,
        "record_payload_size must NOT fire on missing-variable-field Drop"
    );
}

#[test]
fn adaptive_does_not_warm_with_only_failed_drops() {
    // Stronger version of the above: WINDOW_SIZE failed drops in a
    // row must not warm the publisher. The sliding window stays
    // empty and the loan returns max_slice_len.
    let (_tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    for _ in 0..(WINDOW_SIZE * 2) {
        let _proxy = pubr.loan_proxy::<RosString>().expect("loan");
        // Drop without set_data.
    }
    assert!(
        !pubr.sizer_warm(),
        "publisher must not warm from failed drops"
    );
    assert_eq!(
        pubr.adaptive_loan_size_for_min_required(40),
        16 * 1024 * 1024,
        "loan size must remain max_slice_len when no successful drops have occurred"
    );
}

// ============================================================
// max_slice_len() accessor stability: post-warmup the configured
// ceiling is unchanged.
// ============================================================

#[test]
fn adaptive_max_slice_len_accessor_unchanged_post_warmup() {
    // The publisher's configured `max_slice_len()` reflects the
    // graph-time YAML value, NOT the adaptive loan size. A regression
    // where adaptive logic accidentally rewrites the ceiling would
    // surface here. The `max_slice_len()` accessor lives on the
    // publisher itself; concrete-type test fixture so we read
    // directly without `AnyPublisher` indirection.
    let (_tt, mut pubr) = make_publisher(16 * 1024 * 1024);
    assert_eq!(pubr.max_slice_len().get(), 16 * 1024 * 1024);
    for _ in 0..WINDOW_SIZE {
        publish_string(&mut pubr, b"x");
    }
    assert!(pubr.sizer_warm());
    // Adaptive loan is much smaller now, but max_slice_len() must
    // still report the configured ceiling.
    assert_eq!(pubr.max_slice_len().get(), 16 * 1024 * 1024);
    assert!(pubr.adaptive_loan_size_for_min_required(40) < 16 * 1024);
}

// ============================================================
// Multi-publish cold-vs-warm determinism
// ============================================================

#[test]
fn adaptive_cold_vs_warm_same_input_multi_publish_determinism() {
    // Stronger than the single-publish cold/warm test:
    // `adaptive_window_state_does_not_leak_into_wire_bytes` only
    // covered ONE post-warmup publish. This extends to FIVE
    // back-to-back publishes — catches a hypothetical bug where the
    // first publish post-warmup is byte-equal but the second
    // diverges (e.g. if the cold publisher's window started warming
    // mid-comparison and shifted the loan size). All five frames
    // must be byte-equal across the cold and warm publishers.
    let (tt_cold, mut cold_pub) = make_buffered_publisher("t/cold", 16 * 1024 * 1024, 16);
    let (tt_warm, mut warm_pub) = make_buffered_publisher("t/warm", 16 * 1024 * 1024, 16);
    let cold_sub = tt_cold.subscriber("t/cold");
    let warm_sub = tt_warm.subscriber("t/warm");
    // Pre-warm only `warm_pub`.
    for _ in 0..WINDOW_SIZE {
        publish_string(&mut warm_pub, &[b'W'; 200]);
    }
    let _ = warm_sub.try_receive(|_| {}).expect("drain warmup");

    let canary = b"deterministic-canary-payload";
    for _ in 0..5 {
        let mut cold_proxy = cold_pub.loan_proxy::<RosString>().expect("loan");
        cold_proxy
            .set_data(std::str::from_utf8(canary).unwrap())
            .expect("set");
        drop(cold_proxy);
        let mut warm_proxy = warm_pub.loan_proxy::<RosString>().expect("loan");
        warm_proxy
            .set_data(std::str::from_utf8(canary).unwrap())
            .expect("set");
        drop(warm_proxy);
    }
    let mut cold_frames: Vec<CapturedFrame> = Vec::new();
    let _ = cold_sub
        .try_receive(|m| cold_frames.push((*m.header(), m.payload().to_vec())))
        .expect("recv cold");
    let mut warm_frames: Vec<CapturedFrame> = Vec::new();
    let _ = warm_sub
        .try_receive(|m| warm_frames.push((*m.header(), m.payload().to_vec())))
        .expect("recv warm");
    assert_eq!(cold_frames.len(), 5);
    assert_eq!(warm_frames.len(), 5);
    for i in 0..5 {
        assert_eq!(
            cold_frames[i].1, warm_frames[i].1,
            "frame {i}: payload bytes differ between cold and warm publisher"
        );
        assert_eq!(
            cold_frames[i].0.total_size, warm_frames[i].0.total_size,
            "frame {i}: total_size differs (sliding-window state leaked into header!)"
        );
    }
}

// ============================================================
// Codegen-string assertions: error variant introduction
// ============================================================
//
// (These tests live in `error_message_test.rs` lib unit tests, which
// already covers `PayloadTooLarge` / `AllocationFailed` Display
// semantics. No additional codegen-string assertions are needed here
// — the codegen emits `ProxyBufferTooSmall` at those sites.)
