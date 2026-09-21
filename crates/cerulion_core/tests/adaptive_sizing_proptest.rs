// SPDX-License-Identifier: AGPL-3.0-only
//! Property-based adversarial coverage for
//! adaptive `loan_proxy` sizing.
//!
//! The deterministic suite in `adaptive_sizing_test.rs` pins specific
//! sliding-window transitions (warm/cold, grow/shrink, ring wrap). These
//! `proptest` cases fuzz the two invariants those hand-picked sequences
//! can only sample:
//!
//! 1. **No panic across arbitrary size sequences.** A random sequence of
//!    payload sizes driven through the real `loan_proxy` + publish path
//!    must never panic — each tick either publishes or returns a clean
//!    `TransportError` (`PayloadTooLarge` / `AllocationFailed`). This
//!    surfaces ring-buffer-wrap × loan-size interactions the fixed
//!    sequences miss.
//! 2. **Byte-for-byte round-trip.** Arbitrary payload bytes published
//!    through the adaptive path must arrive at the subscriber unchanged,
//!    regardless of the publisher's sliding-window state.
//!
//! # Transport (later)
//!
//! The in-process heap backend (`InProcessPublisher`) was deleted by
//! These properties now drive the real iceoryx2 transport via the
//! isolated per-test `TestTransport` (a fresh `TransportManager` over its
//! own SHM root — the same pattern as `overflow_redirect_e2e_test.rs`),
//! so each case publishes + receives over shared memory. Because every
//! `cargo test` invocation shares the process-global iceoryx2
//! `TransportManager` singleton state, each proptest fn is `#[serial]`
//! (serial_test dev-dep) and every case publishes on a process-unique
//! topic name (`unique_topic`) so concurrent/successive cases never
//! collide on a topic.
//!
//! The payload is `std_msgs::ByteMultiArray`, whose `data` variable field
//! accepts arbitrary `u8` bytes (unlike `std_msgs::String` /
//! `set_data(&str)`, which requires UTF-8). `ByteMultiArray` has two
//! variable fields (`layout`, then `data`); we write `layout` empty and
//! `data` with the payload, in declaration order — the established
//! write-both-fields pattern from `overflow_redirect_test.rs`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use cerulion_core::testing::TestTransport;
use cerulion_core::transport::adaptive_sizer::WINDOW_SIZE;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::std_msgs::ByteMultiArray;
use proptest::prelude::*;
use serial_test::serial;

/// 16 MiB test ceiling — far above the proptest payload domain (≤ 16384
/// bytes; note the runtime DEFAULT_MAX_SLICE_LEN tier-3 default is now
/// 128 MiB later, this const is an independent test param), so the
/// adaptive loan never hits the overflow / `PayloadTooLarge` path inside
/// these properties. The "returns a clean error" arm is exercised
/// separately by the deterministic error-variant tests; here we assert
/// the happy path never panics and the bytes match.
const MAX_SLICE_LEN: u32 = 16 * 1024 * 1024;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

/// A process-unique topic name so successive proptest cases (which share
/// the global iceoryx2 `TransportManager` state across `TestTransport`
/// instances) never collide. Combines wall-clock nanos with a monotonic
/// counter.
fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system time after epoch")
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/adaptive_proptest/{base}/{nanos}/{id}")
}

/// Build a fresh isolated iceoryx2 transport plus a publisher on a unique
/// topic. Each property case gets its own `TestTransport` (own SHM root)
/// and topic so sliding-window state never leaks across cases. `buf` is
/// the subscriber buffer size. Returns the transport (which MUST outlive
/// the publisher + subscriber) alongside the publisher.
fn make_publisher(buf: usize) -> (TestTransport, String, CerulionPublisher) {
    let tt = TestTransport::with_buffer_size(buf);
    let topic = unique_topic("p");
    let publisher = tt.publisher(&topic, MaxSliceLen::const_new(MAX_SLICE_LEN), 0);
    (tt, topic, publisher)
}

/// Publish one `ByteMultiArray` carrying `payload` as its `data` field.
/// Writes `layout` empty (field 0) then `data` (field 1) in declaration
/// order, then drops the proxy to publish. Returns the `set_*` result so
/// callers can assert it is `Ok` (or a clean `Err`) — never a panic.
fn try_publish(
    pubr: &mut CerulionPublisher,
    payload: &[u8],
) -> Result<(), cerulion_core::TransportError> {
    let mut proxy = pubr.loan_proxy::<ByteMultiArray>()?;
    proxy.set_layout_bytes(&[])?;
    proxy.set_data(payload)?;
    // proxy drops here → publish + sliding-window record.
    Ok(())
}

proptest! {
    // Trim the case count from proptest's default 256. Each case drives
    // up to 64 real publish+receive cycles at payloads up to 16 KiB on a
    // debug build, so the default would dominate `cargo test --workspace`
    // wall-clock. 32 cases still samples the size domain densely enough to
    // surface ring-wrap / loan-sizing edge interactions while keeping this
    // file's debug-build wall-clock modest. The deterministic suite in
    // `adaptive_sizing_test.rs` carries the exhaustive corner cases.
    #![proptest_config(ProptestConfig::with_cases(32))]

    /// A random sequence of payload sizes (0..16384 bytes, up to 64
    /// ticks) driven through the adaptive path must NEVER panic. Every
    /// tick must resolve to either a successful publish or a clean
    /// `TransportError` — proptest fails on any panic (e.g. an arithmetic
    /// overflow or slice OOB in the sliding-window / loan-sizing math).
    ///
    /// A bounded subscriber is drained each tick so the transport's
    /// receive queue can't grow without bound over the sequence.
    #[test]
    #[serial]
    fn adaptive_property_random_payload_sequence_never_panics(
        sizes in prop::collection::vec(0..16384usize, 0..64)
    ) {
        let (tt, topic, mut pubr) = make_publisher(4);
        let mut sub = tt.subscriber(&topic);
        for size in sizes {
            let payload = vec![0xABu8; size];
            // Within the proptest domain (size ≤ 16384 ≪ 16 MiB) this is
            // always Ok; an Err would still be a CLEAN outcome (not a
            // panic), which is the property under test. We tolerate Err
            // without failing — the invariant is "no panic", and a typed
            // error is a valid non-panicking outcome.
            let _ = try_publish(&mut pubr, &payload);
            // Drain to keep the bounded receive queue from filling.
            let _ = sub.try_view::<ByteMultiArray, _>(|_| ());
        }
    }

    /// Arbitrary payload bytes round-trip byte-for-byte through the
    /// subscriber regardless of sliding-window state. We first WARM the
    /// publisher (so the adaptive loan is in steady-state, not the cold
    /// `max_slice_len` path), publishing warmup payloads at least as large
    /// as the test payload so the warm loan stays big enough (this path
    /// has no in-tick overflow redirect). Then the random payload is
    /// published and read back via the typed `ByteMultiArray` reader.
    #[test]
    #[serial]
    fn adaptive_property_published_bytes_match_user_writes(
        payload in prop::collection::vec(any::<u8>(), 0..16384usize)
    ) {
        let (tt, topic, mut pubr) = make_publisher(2 * WINDOW_SIZE);
        let mut sub = tt.subscriber(&topic);

        // Warm the sliding window with payloads no smaller than the test
        // payload, so the steady-state adaptive loan can hold it.
        let warmup = vec![b'W'; payload.len()];
        for _ in 0..WINDOW_SIZE {
            try_publish(&mut pubr, &warmup).expect("warmup publish must succeed");
        }
        // `try_view` is drain-to-latest (latest-wins): each call discards all
        // but the newest queued sample (see `CerulionSubscriber::try_view`).
        // So publishing the canary LAST and reading once returns the canary —
        // the warmup frames are drained past, no explicit pre-drain needed.
        // (Warmup + canary = WINDOW_SIZE + 1 ≤ the 2*WINDOW_SIZE buffer, so
        // nothing is evicted before the read either way.)
        try_publish(&mut pubr, &payload).expect("canary publish must succeed");
        let received = sub
            .try_view::<ByteMultiArray, _>(|view| view.data().to_vec())
            .expect("try_view must not error")
            .expect("a frame must be available after publish");
        prop_assert_eq!(
            received,
            payload,
            "published ByteMultiArray data must round-trip byte-for-byte"
        );
    }
}
