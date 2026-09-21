// SPDX-License-Identifier: AGPL-3.0-only
//! Iceoryx2 tests for the overflow re-loan
//! path.
//!
//! Pin that an OVERFLOW tick (a payload spike that spills past the warm
//! adaptive loan and re-loans a larger sample in `OutputProxy::Drop` →
//! `send_overflow_frame`) is retained for late joiners exactly like a
//! steady-state tick. History is
//! iceoryx2-native (there is no heap `HistoryBuffer`): the overflow tick's `sample.send()` feeds the
//! native publisher history queue (by SHM offset, zero-copy), so a late
//! joiner receives the spike frame with NO Cerulion-side history copy. A
//! refactor that lost the overflow send (or its native retention) would
//! make the late joiner miss the spike (Replay = Live violation,
//! Principle #7) — this test fails on that.
//!
//! # Serial-only
//!
//! These tests share the global iceoryx2 singleton. The parent
//! `cargo test` invocation must run with `--test-threads=1`.

use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::sensor_msgs::Image;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::SystemTime;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/overflow_iox2/{base}/{nanos}/{id}")
}

#[test]
fn iceoryx2_overflow_tick_retained_in_native_history() {
    // An OVERFLOW tick (spike that spills →
    // send_overflow_frame's larger re-loan + `sample.send()`) feeds the
    // NATIVE iceoryx2 publisher history queue, so a late joiner connecting
    // after the spike receives the spike frame (Replay = Live for the
    // overflow path). A regression dropping the overflow send (or its
    // native retention) would make the late joiner see only warm-up-sized
    // frames (~1 KiB), failing the size assertion below.
    use std::time::Duration;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("history_parity");
    // history_size 4 keeps the last 4 frames — the spike is the newest, so
    // it survives the bounded native history.
    let mut publisher = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(16 * 1024), 4)
        .expect("create publisher with history");

    // Warm sizer with 16 small publishes (1 KiB each).
    for i in 0..16u32 {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.height = i;
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&vec![0u8; 1024]).expect("warmup data");
    }

    // Spike! 12 KiB payload exceeds the warm ~1.5 KiB loan → spill →
    // OutputProxy::Drop → send_overflow_frame → native history retains it.
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.height = 999;
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        let spike_payload = vec![0xa5u8; 12 * 1024];
        proxy.set_data(&spike_payload).expect("spike via spill");
        assert!(proxy.has_overflow(), "spike MUST trigger spill");
    }

    // Late-joiner subscribes AFTER the spike. Its SubscriberConnected
    // event drives the publisher's native history delivery on the
    // publisher's next loan_proxy (below) — `update_connections()` delivers
    // the retained spike frame, then SentHistory wakes the late joiner.
    let late_joiner = mgr.create_subscriber(&topic).expect("create late joiner");

    // One more publish to pump the SubscriberConnected handler (native
    // delivery + wake). A small frame so it does not itself look like a
    // 12 KiB spike — the only >= 12 KiB frame the late joiner can see is
    // the retained overflow tick.
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.height = 1000;
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&vec![0u8; 256]).expect("pump frame");
    }

    // Drain the late-joiner PER-FRAME (try_receive, not try_view — try_view
    // is latest-wins and would discard the spike if a newer frame is queued)
    // and track the MAX wire total_size seen. The retained 12 KiB overflow
    // tick MUST be present.
    let mut max_total_size = 0usize;
    let mut frames_received = 0usize;
    for _ in 0..50 {
        let _ = late_joiner
            .try_receive(|msg| {
                // The parsed WireHeader is on `msg.header()`; `msg.payload()`
                // is only the post-header bytes.
                max_total_size = max_total_size.max(msg.header().total_size as usize);
                frames_received += 1;
            })
            .expect("try_receive");
        std::thread::sleep(Duration::from_millis(2));
    }

    assert!(
        frames_received > 0,
        "late-joiner must receive native history (none received)"
    );
    // total_size = WireHeader::SIZE + payload; the 12 KiB spike dominates.
    assert!(
        max_total_size >= 12 * 1024,
        "late-joiner must see the spike's 12 KiB frame via native history — \
         got max total_size={} bytes. A regression losing the overflow send (or \
         its native retention) would leave only warm-up sizes (~1 KiB).",
        max_total_size
    );
}

#[test]
fn iceoryx2_overflow_with_history_disabled_delivers_live_without_drop() {
    // With history disabled (history_size = 0 via
    // create_publisher_simple), an overflow tick still delivers LIVE to a
    // connected subscriber with no frame dropped — the native history queue
    // simply retains nothing for future late joiners. Pins that the
    // send_overflow_frame path is unaffected by history being off.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("history_disabled");
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(16 * 1024))
        .expect("create publisher without history");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Warm sizer.
    for _ in 0..16u32 {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&vec![0u8; 1024]).expect("ok");
    }

    // Spike → spill → send_overflow_frame.
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy
            .set_data(&vec![0xbbu8; 12 * 1024])
            .expect("spike via spill");
        assert!(proxy.has_overflow());
    }

    // No frame dropped on the overflow path.
    assert_eq!(publisher.frames_dropped_overflow(), 0);

    // Live subscriber received the 12 KiB spike (delivery unaffected by
    // history being disabled).
    std::thread::sleep(std::time::Duration::from_millis(100));
    let observed = subscriber
        .try_view::<Image, _>(|view| view.data().len())
        .expect("try_view")
        .expect("live frame present");
    assert_eq!(
        observed,
        12 * 1024,
        "history-disabled overflow still delivers the full 12 KiB live"
    );
}

#[test]
fn iceoryx2_overflow_increments_dropped_counter_on_re_loan_failure() {
    // Pin the counter + log on Drop's overflow-Err arm over the global
    // iceoryx2 manager (the mirror of the isolated-root counter test),
    // driven by the fire-once
    // `fault_inject_send_overflow_frame_after` seam.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("dropped_counter");
    let mut publisher = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(16 * 1024), 2)
        .expect("create publisher");
    let _subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Warm sizer.
    for _ in 0..16u32 {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&[1u8]).expect("set_data warm-up");
    }
    assert_eq!(publisher.frames_dropped_overflow(), 0);

    // Arm fault → next send_overflow_frame returns LoanCapacity.
    publisher.fault_inject_send_overflow_frame_after(0);

    // Spike → spill → re-loan FAILS → counter++ + tracing::error!
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy
            .set_data(&vec![0xddu8; 8 * 1024])
            .expect("spike → spill (set_data succeeds; the FAILURE is at Drop time)");
    }

    assert_eq!(
        publisher.frames_dropped_overflow(),
        1,
        "iceoryx2 Drop's overflow-Err arm must increment frames_dropped_overflow"
    );

    // Recovery: subsequent ticks succeed (fault was fire-once).
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&[42u8]).expect("recovery tick");
    }
    assert_eq!(publisher.frames_dropped_overflow(), 1, "no double-count");
}

#[test]
fn iceoryx2_spill_allocation_failed_surfaces_to_setter_call() {
    // iceoryx2 backend parity for the in-process test
    // `e2e_spill_allocation_failed_surfaces_to_setter_call`.
    //
    // The `spill_fault_injection::arm()` hook is checked at the TOP of
    // codegen-emitted `<Name>Shm::spill_to_overflow`, which is
    // backend-agnostic. The Err returned by `set_data` propagates
    // through the iceoryx2 `OutputProxy` identically to in-process.
    // This test pins that contract: a future refactor that gates the
    // fault check by backend (or moves it post-allocation in a way
    // that diverges) would break in-process and iceoryx2
    // independently — both need direct test coverage.
    use cerulion_core::error::TransportError;
    use cerulion_core::spill_fault_injection;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("spill_oom_iox2");
    let mut publisher = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(16 * 1024), 0)
        .expect("create publisher");
    let _subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Warm sizer with small payloads so the next loan is small.
    for _ in 0..16u32 {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&[1u8]).expect("set_data warm-up");
    }

    // Arm the in-tick allocation-failure fault. The next setter that
    // would spill returns `AllocationFailed`.
    spill_fault_injection::arm();

    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");

        // Spike that would spill: 8 KiB write into ~1.5 KiB warm loan,
        // would fit in 16 KiB ceiling. The fault check fires first.
        let big = vec![0xeeu8; 8 * 1024];
        let err = proxy
            .set_data(&big)
            .expect_err("set_data with spill fault armed must Err");
        match err {
            TransportError::AllocationFailed { topic, requested } => {
                assert!(
                    topic.starts_with("test/overflow_iox2/spill_oom_iox2/"),
                    "AllocationFailed must carry the publisher's topic (got: {topic})"
                );
                // requested == max_capacity == max_slice_len - WireHeader::SIZE = 16K - 32.
                assert_eq!(
                    requested,
                    16 * 1024 - 32,
                    "AllocationFailed.requested must reflect max_capacity, not the setter's bytes_needed"
                );
            }
            other => panic!("expected AllocationFailed, got {other:?}"),
        }
        // Proxy drops here — has_overflow=false (fault fired before
        // spill), all_variables_written=false (set_data Err'd before
        // mark_written), so Drop skips the publish.
    }

    // Verify recovery: next tick succeeds (fault is fire-once).
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&[42u8]).expect("post-fault recovery");
    }
}

// NOTE: there is no history-push failure test. Native history retains the SHM frame by
// offset on `send()` — there is no Cerulion-side history allocation to fail,
// so there is no `history_pushes_failed` counter, fault hook, or contract to pin.

#[test]
fn iceoryx2_invariant_violation_counter_is_zero_in_normal_operation() {
    // Pin that the
    // `frames_dropped_invariant_violation` counter stays at 0 during
    // normal operation (no Drop reaches the documented-unreachable
    // defensive branches in the current production code path).
    //
    // The three branches are:
    // - Contract violation `T::has_overflow == true && overflow_view_bytes == None`
    //   (requires a hand-written buggy `impl ShmMessage`)
    // - Small original_bytes at overflow drop (< WireHeader::SIZE)
    //   (requires loan_proxy to skip stamping the header)
    // - Small buf at steady-state drop (< 12 bytes)
    //   (requires a publisher to return an undersized sample)
    //
    // None are reachable today. This test pins that contract — a
    // future refactor breaking one of those invariants would flip
    // this counter and FAIL the test.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("invariant_zero");
    let mut publisher = mgr
        .create_publisher(&topic, MaxSliceLen::const_new(16 * 1024), 0)
        .expect("create publisher");
    let _subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Exercise both steady-state and overflow paths.
    for _ in 0..16u32 {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&[1u8]).expect("set_data warm-up");
    }
    // Spike to exercise the overflow path.
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy
            .set_data(&vec![0xeeu8; 12 * 1024])
            .expect("spike via spill");
    }

    assert_eq!(
        publisher.frames_dropped_invariant_violation(),
        0,
        "no invariant violation Drop branch should fire in normal operation"
    );
}
