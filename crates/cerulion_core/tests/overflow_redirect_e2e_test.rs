// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end overflow-redirect tests via the iceoryx2
//! `TestTransport` helper.
//!
//! These tests verify the full pipeline from `loan_proxy` → spill →
//! `OutputProxy::Drop` re-loan → subscriber receives, over an isolated
//! per-test iceoryx2 transport (`TestTransport`) rather than the
//! process-global singleton. iceoryx2 is the only backend, so this is the
//! production `OutputProxy::Drop` overflow path, just on its own SHM root.
//!
//! # What's tested
//!
//! - **Round-trip via spill**: publisher with tiny adaptive loan +
//!   payload spike → subscriber receives the full payload, byte-equal.
//! - **Post-overflow convergence**: after a spike tick, the sliding
//!   window grows, so the next tick at the same size NO LONGER spills.
//! - **Determinism**: byte-equal frame between the spill path and a
//!   direct-max-loan path on the same input data.
//! - **Partial-write + spill + drop**: dropping the OutputProxy without
//!   writing all variable fields does NOT publish the frame, even when
//!   the spill is active. The `all_variables_written` gate is independent
//!   of the spill path.
//! - **Multi-field spill**: a spike that triggers spill, followed by a
//!   write to a DIFFERENT variable field, lands the second field in the
//!   spill buffer with the correct offset table entry.
//! - **PayloadTooLarge does NOT publish**: error path leaves no frame in
//!   the subscriber's channel.
//! - **Overflow-path discard recovery**:
//!   a discard regime healed by a SPILLING complete publish fires the
//!   `OutputProxy::Drop` overflow send-success recovery `info!` + latch re-arm.

use cerulion_core::error::TransportError;
use cerulion_core::testing::{count_at_exclusively, line_level, TestTransport};
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::sensor_msgs::Image;
use tracing_test::traced_test;

/// Create an isolated iceoryx2 transport plus a publisher on `topic`.
///
/// Returns the `TestTransport` alongside the publisher: the transport
/// owns the iceoryx2 `TransportManager` and MUST outlive the publisher
/// and any subscribers, so the caller binds it as a local for the rest
/// of the test fn (e.g. `let (tt, mut pub_) = fresh_publisher(...);`).
/// `buf` is the subscriber buffer size applied to every subscriber
/// created from this transport.
fn fresh_publisher(
    topic: &str,
    max_slice_len: u32,
    buf: usize,
) -> (TestTransport, CerulionPublisher) {
    let tt = TestTransport::with_buffer_size(buf);
    let publisher = tt.publisher(topic, MaxSliceLen::const_new(max_slice_len), 0);
    (tt, publisher)
}

/// Drop a writer immediately by scoping it. Returns nothing — used to
/// trigger `OutputProxy::Drop` deterministically.
fn publish_tick<F>(pub_: &mut CerulionPublisher, f: F)
where
    F: FnOnce(&mut cerulion_core::transport::output_proxy::OutputProxy<'_, Image>),
{
    let mut proxy = pub_.loan_proxy::<Image>().expect("loan_proxy");
    f(&mut proxy);
    // `proxy` drops here — publishes the frame (steady or via spill).
}

#[test]
fn e2e_spill_round_trip_subscriber_sees_full_payload() {
    // For a spill (not PayloadTooLarge) we need a WARM sizer (`recent_max`
    // small enough that the next adaptive loan is much smaller than the
    // payload) plus a payload that fits in `max_capacity`. Setup:
    //   1. Warm the sizer with 16 small (1 KiB) publishes.
    //   2. Spike to 12 KiB — exceeds the ~1.5 KiB warm loan, fits in the
    //      16 KiB ceiling. Spill fires.
    let (tt, mut pub_) = fresh_publisher("e2e/spike", 16 * 1024, 64); // 16 KiB ceiling
    let mut sub = tt.subscriber("e2e/spike");

    // Step 1: warm the sizer with 16 small publishes (1 KiB each).
    for i in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.height = i;
            proxy.width = 1;
            proxy.step = 1;
            proxy.is_bigendian = 0;
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&vec![0u8; 1024]).expect("data warmup");
        });
    }

    // Drain warm-up subscriber buffer.
    let _ = sub.try_view::<Image, _>(|_| ()).expect("drain");

    // Step 2: spike! Publish 12 KiB payload. Current adaptive loan is
    // ~1024 * 1.5 = 1.5 KiB; 12 KiB exceeds the loan but fits in 16 KiB
    // ceiling. Spill should fire.
    let spike_payload = vec![0x42u8; 12 * 1024];
    publish_tick(&mut pub_, |proxy| {
        proxy.height = 999;
        proxy.width = 1;
        proxy.step = 1;
        proxy.is_bigendian = 0;
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&spike_payload).expect("spike via spill");
    });

    // Subscriber sees the spike.
    let observed = sub
        .try_view::<Image, _>(|view| (view.height, view.data().to_vec()))
        .expect("try_view")
        .expect("frame present");

    assert_eq!(observed.0, 999, "spike tick's height field round-trips");
    assert_eq!(
        observed.1.len(),
        12 * 1024,
        "subscriber sees the full 12 KiB payload (not truncated to the small loan size)"
    );
    assert_eq!(
        observed.1, spike_payload,
        "subscriber sees byte-equal payload"
    );
}

#[test]
fn e2e_post_overflow_convergence_grows_next_loan() {
    let (tt, mut pub_) = fresh_publisher("e2e/converge", 64 * 1024, 64);
    let _sub = tt.subscriber("e2e/converge");

    // Warm with 16 small (1 KiB) publishes.
    for i in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.height = i;
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&vec![0u8; 1024]).expect("set_data warm-up");
        });
    }

    // Sanity: current adaptive loan size is small (~1.5 KiB after
    // warmup at 1 KiB writes).
    let pre_spike_loan = pub_.adaptive_loan_size_for_min_required(40);
    assert!(
        pre_spike_loan < 4 * 1024,
        "pre-spike loan should be small (<4 KiB), got {}",
        pre_spike_loan
    );

    // Spike! 32 KiB payload. Spill fires.
    publish_tick(&mut pub_, |proxy| {
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&vec![0xaau8; 32 * 1024]).expect("spill");
    });

    // After the overflow tick recorded the actual payload size, the
    // next adaptive loan should be sized to fit the spike (~48 KiB).
    let post_spike_loan = pub_.adaptive_loan_size_for_min_required(40);
    assert!(
        post_spike_loan > 32 * 1024,
        "post-spike adaptive loan should accommodate the spike (>32 KiB), got {} \
         — overflow-tick recording (the convergence policy) failed",
        post_spike_loan
    );
}

#[test]
fn e2e_partial_write_with_spill_does_not_publish() {
    // A cold publisher with max_slice_len = 64 KiB would not do here:
    // cold loan = max_slice_len = 64 KiB; 8 KiB set_data fits → NO
    // SPILL, and the test would pass for the wrong reason (just exercising the
    // missing-var-field gate). To actually trigger spill, warm the sizer
    // first so the next loan is ~1.5 KiB. Then 8 KiB set_data spills.
    // The OutputProxy::Drop `all_variables_written` check then gates the
    // publish BEFORE the overflow re-loan path runs — frame is NOT
    // published, but the spill DID happen.
    let (tt, mut pub_) = fresh_publisher("e2e/partial", 64 * 1024, 64);
    let mut sub = tt.subscriber("e2e/partial");

    // Warm sizer.
    for _ in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[1u8]).expect("set_data warm-up");
        });
    }
    while sub
        .try_view::<Image, _>(|_| ())
        .expect("try_view drain")
        .is_some()
    {}

    {
        let mut proxy = pub_.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        // Skip encoding — leave it unwritten.
        // 8 KiB exceeds the warm ~1.5 KiB loan → spill fires.
        proxy.set_data(&vec![0u8; 8 * 1024]).expect("data spill");
        assert!(
            proxy.has_overflow(),
            "spill MUST fire for this test to exercise spill+partial-write interaction"
        );
        // proxy drops here. Drop body checks `all_variables_written`
        // FIRST (encoding unwritten → fails), then would check
        // `has_overflow` SECOND. The all-vars gate fires first → no
        // publish reaches the subscriber.
    }

    let observed = sub.try_view::<Image, _>(|_| ()).expect("try_view");
    assert!(
        observed.is_none(),
        "no frame should reach the subscriber when a variable field was unwritten — \
         even though the spill HAS fired (all_variables_written check runs BEFORE overflow path)"
    );
}

#[test]
fn e2e_payload_too_large_does_not_publish() {
    // max_slice_len = 4 KiB. set_data 16 KiB → PayloadTooLarge.
    // OutputProxy::Drop sees the missing-var-field gate (encoding never
    // written either), but more importantly the frame is NEVER
    // delivered because (a) set_data Err propagation, (b) Drop's
    // missing-var-field check still runs.
    let (tt, mut pub_) = fresh_publisher("e2e/too_large", 4096, 16);
    let mut sub = tt.subscriber("e2e/too_large");

    {
        let mut proxy = pub_.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        let err = proxy
            .set_data(&vec![0u8; 16 * 1024])
            .expect_err("payload exceeds 4 KiB ceiling");
        assert!(matches!(err, TransportError::PayloadTooLarge { .. }));
        // proxy drops with missing variable field `data` (set_data
        // returned Err, mark_written NOT set). Drop's gate skips publish.
    }

    let observed = sub.try_view::<Image, _>(|_| ()).expect("try_view");
    assert!(
        observed.is_none(),
        "PayloadTooLarge must NOT result in a published frame"
    );
}

#[test]
fn e2e_determinism_spill_path_matches_direct_loan_path() {
    // Same input data → same wire bytes, regardless of whether the
    // publisher hit the spill path or the direct loan path.
    //
    // Path A: tiny ceiling so the first loan = ceiling, payload spills.
    //   Actually for a spill, we need max_slice_len > payload, otherwise
    //   PayloadTooLarge. Use 64 KiB ceiling, force spill by warming
    //   sizer with tiny payloads first.
    //
    // Path B: ceiling = payload size, no spill (direct loan of exactly
    //   the payload size).
    //
    // Both should produce the same wire bytes (modulo sequence number
    // and timestamp, which differ between publishers).

    let payload = vec![0x77u8; 4 * 1024];
    let captured_a: Vec<u8>;
    let captured_b: Vec<u8>;

    {
        // Path A: warm + spike.
        let (tt, mut pub_) = fresh_publisher("e2e/det_a", 64 * 1024, 16);
        let mut sub = tt.subscriber("e2e/det_a");
        // Warm with 16 small publishes.
        for _ in 0..16 {
            publish_tick(&mut pub_, |proxy| {
                proxy.set_header_bytes(&[]).expect("set_header_bytes");
                proxy.set_encoding("g").expect("set_encoding");
                proxy.set_data(&[1u8]).expect("set_data warm-up");
            });
        }
        // Drain warmup.
        loop {
            if sub
                .try_view::<Image, _>(|_| ())
                .expect("try_view drain")
                .is_none()
            {
                break;
            }
        }
        // Spike (causes spill).
        publish_tick(&mut pub_, |proxy| {
            proxy.height = 42;
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&payload).expect("spike");
        });
        // Capture the wire bytes.
        captured_a = sub
            .try_view::<Image, _>(|view| view.data().to_vec())
            .expect("try_view")
            .expect("frame present");
    }

    {
        // Path B: ceiling exactly = payload size, no spill.
        // 4 KiB + header(32) + offset table(24) + fixed section(16) = 4168
        let (tt, mut pub_) = fresh_publisher("e2e/det_b", 8 * 1024, 16);
        let mut sub = tt.subscriber("e2e/det_b");
        // Single tick (cold sizer → loans full max_slice_len).
        publish_tick(&mut pub_, |proxy| {
            proxy.height = 42;
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&payload).expect("set_data payload");
        });
        captured_b = sub
            .try_view::<Image, _>(|view| view.data().to_vec())
            .expect("try_view")
            .expect("frame present");
    }

    // The `data` field bytes must be byte-equal between the two paths.
    assert_eq!(
        captured_a, captured_b,
        "spill path and direct-loan path must produce byte-equal `data` field"
    );
    assert_eq!(captured_a, payload, "round-trip is byte-equal to input");
}

#[test]
fn e2e_multi_field_spill_all_fields_in_heap() {
    // After a spill triggered by one field, subsequent writes to a
    // DIFFERENT variable field must also land in the spill buffer with
    // correct offset table entries.
    let (tt, mut pub_) = fresh_publisher("e2e/multi", 64 * 1024, 16);
    let mut sub = tt.subscriber("e2e/multi");

    // Warm sizer with 16 tiny publishes.
    for _ in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[1u8]).expect("set_data warm-up");
        });
    }
    // Drain.
    while sub
        .try_view::<Image, _>(|_| ())
        .expect("try_view drain")
        .is_some()
    {}

    // Spike: encoding (long string) triggers spill mid-way; then
    // set_data afterwards writes into the spill buffer.
    let long_encoding = "x".repeat(8 * 1024);
    let data_payload = vec![0x55u8; 4 * 1024];
    publish_tick(&mut pub_, |proxy| {
        proxy.height = 13;
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy
            .set_encoding(&long_encoding)
            .expect("encoding via spill");
        proxy.set_data(&data_payload).expect("data into spill");
    });

    let observed = sub
        .try_view::<Image, _>(|view| {
            (
                view.height,
                view.encoding().expect("encoding").to_owned(),
                view.data().to_vec(),
            )
        })
        .expect("try_view")
        .expect("frame present");

    assert_eq!(observed.0, 13);
    assert_eq!(
        observed.1, long_encoding,
        "encoding (which triggered spill) round-trips"
    );
    assert_eq!(
        observed.2, data_payload,
        "data (written AFTER spill) lands at correct offset and round-trips"
    );
}

#[test]
fn e2e_exact_boundary_loan_equals_max_capacity() {
    // Pins the spill-trigger boundary on a COLD publisher where the
    // first-tick loan equals `max_slice_len` (max_capacity) exactly —
    // the adaptive sizer has no history yet, so it returns the ceiling.
    // Under those conditions:
    //   cursor + bytes_needed == self.len → fits, no spill.
    //   cursor + bytes_needed == self.len + 1 → spills.
    //
    // A hot publisher hides this boundary because the sizer + 1.5x
    // headroom obscure the loan size; a cold publisher exposes it.
    //
    // max_slice_len = 256. WireHeader = 32. Loan post-header = 224.
    // Image fixed section + offset table = 16 + 24 = 40. So cursor
    // starts at 40, with 224 - 40 = 184 bytes for variable payload.
    //
    // Write header_bytes(empty), encoding("x"). Then set_data(183) fits
    // (cursor 40 + 1 [encoding] + 183 = 224, exactly self.len). No spill.
    // set_data(184) exceeds by 1 → triggers spill.
    let (tt, mut pub_) = fresh_publisher("e2e/boundary", 256, 16);
    let mut sub = tt.subscriber("e2e/boundary");

    // Tick 1: exactly fits.
    publish_tick(&mut pub_, |proxy| {
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("x").expect("set_encoding");
        // 224 (loan post-header) - 40 (fixed+offset) - 1 (encoding) = 183.
        // Setter advances cursor to 224 == self.len. No spill.
        proxy.set_data(&[1u8; 183]).expect("exact-fit no spill");
    });
    // Drain.
    let obs1 = sub
        .try_view::<Image, _>(|view| view.data().to_vec())
        .expect("try_view")
        .expect("frame present");
    assert_eq!(obs1.len(), 183);

    // Tick 2: explicitly try 185 bytes via set_data on a fresh proxy
    // (cursor 40 + 185 = 225 > 224 = self.len, AND > max_capacity = 224).
    // PayloadTooLarge.
    let mut proxy = pub_.loan_proxy::<Image>().expect("loan_proxy");
    let r = proxy.set_data(&[1u8; 185]);
    match r {
        Err(TransportError::PayloadTooLarge { requested, max, .. }) => {
            assert_eq!(max, 224, "max_capacity = max_slice_len - WireHeader::SIZE");
            assert!(
                requested >= 40 + 185,
                "requested includes fixed-section offset + write size"
            );
        }
        other => panic!("expected PayloadTooLarge, got {:?}", other),
    }
    drop(proxy);
}

#[test]
fn e2e_spill_on_first_variable_write() {
    // Spill triggered on the FIRST
    // variable write (smallest possible written_so_far: only the prefix
    // = fixed section + offset table, no prior variable payload).
    // Exercises the memcpy boundary at the lower end.
    let (tt, mut pub_) = fresh_publisher("e2e/first_field", 32 * 1024, 64);
    let mut sub = tt.subscriber("e2e/first_field");

    // Warm with 16 tiny publishes.
    for _ in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[1u8]).expect("set_data warm-up");
        });
    }
    while sub
        .try_view::<Image, _>(|_| ())
        .expect("try_view drain")
        .is_some()
    {}

    // Spike on FIRST field — header_bytes (8 KiB) with a warm ~1.5 KiB loan.
    let big_header = vec![0xeeu8; 8 * 1024];
    publish_tick(&mut pub_, |proxy| {
        proxy.height = 7;
        proxy
            .set_header_bytes(&big_header)
            .expect("first-field spike via spill");
        // Pin that the spill
        // actually fired. Without this assertion, the test would pass
        // even if the warm-up was ineffective (loan stayed large) and
        // no spill occurred — the same wrong-reason pass that
        // `e2e_partial_write_with_spill_does_not_publish` guards against.
        assert!(
            proxy.has_overflow(),
            "first-field spike MUST trigger spill (otherwise warmup was ineffective)"
        );
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&[0u8; 1]).expect("set_data minimal");
    });

    let observed = sub
        .try_view::<Image, _>(|view| {
            (
                view.height,
                view.header_bytes().to_vec(),
                view.encoding().expect("encoding").to_owned(),
                view.data().to_vec(),
            )
        })
        .expect("try_view")
        .expect("frame present");
    assert_eq!(observed.0, 7);
    assert_eq!(
        observed.1, big_header,
        "first-field spike round-trips byte-equal"
    );
    assert_eq!(observed.2, "g");
    assert_eq!(observed.3, &[0u8; 1]);
}

#[test]
fn e2e_overflow_preserves_sequence_and_timestamp_from_loan_time() {
    // The Drop overflow path copies
    // the WireHeader from the ORIGINAL sample's [0..32] (stamped at
    // loan_proxy time) and patches ONLY total_size. Sequence + timestamp
    // must carry over unchanged — otherwise overflow ticks would have
    // different metadata than steady-state ticks, breaking subscribers
    // that rely on monotonic sequence numbers or stable timestamps.
    let (tt, mut pub_) = fresh_publisher("e2e/seq", 16 * 1024, 64);
    let sub = tt.subscriber("e2e/seq");

    // Warm sizer.
    for _ in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[1u8]).expect("set_data warm-up");
        });
    }

    // Drain warmup; capture last sequence number via try_receive.
    let mut last_seq: Option<u32> = None;
    sub.try_receive(|msg| {
        last_seq = Some(msg.header().sequence);
    })
    .expect("drain warmup");
    let pre_spike_seq = last_seq.expect("at least one warm-up frame");

    // Spike (forces spill).
    publish_tick(&mut pub_, |proxy| {
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy
            .set_data(&vec![0xa5u8; 8 * 1024])
            .expect("set_data spike");
    });

    let mut spike_seq: Option<u32> = None;
    let mut spike_ts: Option<u64> = None;
    let mut spike_schema: Option<u64> = None;
    sub.try_receive(|msg| {
        spike_seq = Some(msg.header().sequence);
        spike_ts = Some(msg.header().timestamp_ns);
        spike_schema = Some(msg.header().schema_hash);
    })
    .expect("receive spike frame");

    assert_eq!(
        spike_seq.expect("spike frame present"),
        pre_spike_seq + 1,
        "overflow tick's sequence must be loan-time sequence (pre_spike_seq + 1), NOT a re-stamped value"
    );
    // VirtualClock returns 0 → timestamp_ns from loan is 0. We can't
    // distinguish loan-time vs re-stamp purely on the value here, but the
    // sequence assertion above is the load-bearing pin: re-stamping at
    // Drop would also reset the sequence counter (or skip it), breaking
    // monotonicity. Schema_hash must also be preserved.
    use cerulion_core::message::ShmMessage;
    assert_eq!(
        spike_schema.expect("spike frame present"),
        <Image as ShmMessage>::SCHEMA_HASH,
        "schema_hash from loan-time stamp must be preserved across overflow path"
    );
    // Timestamp comes from VirtualClock; assert it's the same as
    // warm-up's (both 0 with VirtualClock starting at 0). Real clock
    // would give monotonic non-zero values; this assert holds for both.
    let _ = spike_ts.expect("spike frame present");
}

#[test]
fn e2e_send_overflow_frame_err_increments_dropped_counter() {
    // When `send_overflow_frame` returns Err (re-loan failure or send
    // failure), `OutputProxy::Drop` increments `frames_dropped_overflow`
    // on the publisher AND emits `tracing::error!`. The counter is the
    // metrics surface; the error log is grep-only. Both must fire.
    use tracing_test::traced_test;

    #[traced_test]
    fn body() {
        let (tt, mut pub_) = fresh_publisher("e2e/drop_err", 16 * 1024, 64);
        let mut sub = tt.subscriber("e2e/drop_err");

        // Warm sizer.
        for _ in 0..16 {
            publish_tick(&mut pub_, |proxy| {
                proxy.set_header_bytes(&[]).expect("set_header_bytes");
                proxy.set_encoding("g").expect("set_encoding");
                proxy.set_data(&[1u8]).expect("set_data warm-up");
            });
        }
        while sub
            .try_view::<Image, _>(|_| ())
            .expect("try_view drain")
            .is_some()
        {}

        assert_eq!(pub_.frames_dropped_overflow(), 0, "no drops pre-arm");

        // Arm fault-injection: the next send_overflow_frame call fails.
        pub_.fault_inject_send_overflow_frame_after(0);

        // Spike → spill fires → Drop calls send_overflow_frame → fails →
        // counter increments + error logged + frame is LOST.
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy
                .set_data(&vec![0xbbu8; 8 * 1024])
                .expect("spike via spill");
        });

        // Counter incremented.
        assert_eq!(
            pub_.frames_dropped_overflow(),
            1,
            "Drop's overflow-Err arm must increment frames_dropped_overflow"
        );

        // The drop is an `error!`: the level token is matched as well as the
        // two substrings, so a demotion to `warn!`/`info!` cannot pass as it.
        logs_assert(|lines: &[&str]| {
            if lines.iter().any(|l| {
                line_level(l) == Some("ERROR")
                    && l.contains("frame DROPPED")
                    && l.contains("frames_dropped_overflow counter incremented")
            }) {
                Ok(())
            } else {
                Err(
                    "tracing::error! must fire with 'frame DROPPED' and name the \
                     frames_dropped_overflow counter (operator searchability) on ONE \
                     ERROR line"
                        .to_string(),
                )
            }
        });

        // Subscriber receives NO frame (data was LOST).
        let observed = sub.try_view::<Image, _>(|_| ()).expect("try_view drain");
        assert!(
            observed.is_none(),
            "fault-injected overflow re-loan failure must drop the frame"
        );

        // Subsequent ticks recover (fault was fire-once).
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[42u8]).expect("set_data recovery");
        });
        let recovered = sub
            .try_view::<Image, _>(|view| view.data().to_vec())
            .expect("try_view")
            .expect("recovered frame");
        assert_eq!(
            recovered,
            &[42u8],
            "publisher recovers after fire-once fault clears"
        );

        // Counter stays at 1 (recovery tick was steady-state, no overflow).
        assert_eq!(pub_.frames_dropped_overflow(), 1);
    }
    body();
}

#[test]
fn e2e_spill_allocation_failed_surfaces_to_setter_call() {
    // Fault-inject `AllocationFailed` in
    // `<Name>Shm::spill_to_overflow`. Setter calls that would spill
    // surface the error directly to the caller (NOT to `Drop`'s
    // overflow re-loan path — that's `send_overflow_frame`'s
    // `LoanCapacity` failure, see test
    // `e2e_send_overflow_frame_err_increments_dropped_counter`).
    //
    // Mechanism: thread-local fire-once hook (`spill_fault_injection::arm`)
    // armed before the setter call. The codegen-emitted
    // `spill_to_overflow` checks the thread-local at function top and
    // returns `AllocationFailed { topic, requested: max_cap }` when
    // armed. Subsequent setter calls succeed (fire-once auto-clears).
    use cerulion_core::error::TransportError;
    use cerulion_core::testing::spill_fault_injection;

    let (tt, mut pub_) = fresh_publisher("e2e/spill_oom", 16 * 1024, 64);
    let mut sub = tt.subscriber("e2e/spill_oom");

    // Warm sizer with small payloads so the next loan is small.
    for _ in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[1u8]).expect("set_data warm-up");
        });
    }
    while sub
        .try_view::<Image, _>(|_| ())
        .expect("try_view drain")
        .is_some()
    {}

    // Arm the in-tick allocation-failure fault. The next setter that
    // would spill returns `AllocationFailed`.
    spill_fault_injection::arm();

    // Spike payload that would overflow the warm loan + force spill.
    // The setter call surfaces AllocationFailed directly.
    {
        let mut proxy = pub_.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");

        let big = vec![0xeeu8; 8 * 1024];
        let err = proxy
            .set_data(&big)
            .expect_err("set_data with spill fault must Err");
        match err {
            TransportError::AllocationFailed { topic, requested } => {
                assert_eq!(
                    topic, "e2e/spill_oom",
                    "AllocationFailed must carry the publisher's topic"
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
        // proxy still drops here — but since spill failed, the writer
        // has no overflow buffer, AND the data-field bit is NOT set
        // (set_data returned Err before mark_written). Drop's
        // all_variables_written gate skips the publish.
    }

    // Subscriber receives nothing (no frame published).
    let observed = sub.try_view::<Image, _>(|_| ()).expect("try_view drain");
    assert!(
        observed.is_none(),
        "AllocationFailed must skip the publish — no frame in subscriber channel"
    );

    // Subsequent ticks succeed (fire-once cleared the hook).
    publish_tick(&mut pub_, |proxy| {
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy
            .set_data(&[42u8])
            .expect("post-fault tick succeeds (fault is fire-once)");
    });
    let recovered = sub
        .try_view::<Image, _>(|view| view.data().to_vec())
        .expect("try_view")
        .expect("recovery frame present");
    assert_eq!(recovered, &[42u8], "publisher recovers after fault clears");
}

#[test]
fn e2e_send_overflow_frame_err_still_records_payload_for_convergence() {
    // Convergence-on-failure: when
    // `send_overflow_frame` Err's, `OutputProxy::Drop` still records
    // `total_size` into the adaptive sizer BEFORE bumping the
    // dropped-frame counter. This pins the next-tick-converges
    // invariant even on delivery failure: a single overflow whose
    // re-loan races with SHM exhaustion must NOT perpetuate (next
    // tick attempts same payload → same too-small loan → spills
    // again → same dropped frame → ad infinitum).
    //
    // Without the Err-arm record the sizer is unchanged on Err, so the next
    // loan stays cold-sized. With it, the next loan
    // accommodates the spike so the recovery tick is steady-state.
    let (tt, mut pub_) = fresh_publisher("e2e/converge_on_err", 64 * 1024, 64);
    let _sub = tt.subscriber("e2e/converge_on_err");

    // Warm sizer with small payloads so the next adaptive loan is small.
    for _ in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[1u8]).expect("set_data warm-up");
        });
    }

    // Pre-spike loan is small (~1.5 KiB on a 1B payload warm-up).
    let pre_spike = pub_.adaptive_loan_size_for_min_required(40);
    assert!(
        pre_spike < 4 * 1024,
        "pre-spike loan should be small (<4 KiB), got {}",
        pre_spike
    );

    // Arm fault: next send_overflow_frame fails (fire-once).
    pub_.fault_inject_send_overflow_frame_after(0);

    // Spike that would spill. Drop calls send_overflow_frame → fails →
    // the Err arm records `total_size` before bumping the counter.
    publish_tick(&mut pub_, |proxy| {
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy
            .set_data(&vec![0xccu8; 32 * 1024])
            .expect("spike via spill (will fault on re-deliver)");
    });

    // Both effects must have fired:
    // 1. Counter incremented (frame was lost).
    assert_eq!(
        pub_.frames_dropped_overflow(),
        1,
        "frames_dropped_overflow must increment when re-deliver Err's"
    );
    // 2. Sizer recorded the spike — next loan accommodates it.
    let post_spike = pub_.adaptive_loan_size_for_min_required(40);
    assert!(
        post_spike > 32 * 1024,
        "post-spike adaptive loan should accommodate the spike (>32 KiB), got {} \
         — Err-arm record_payload_size (convergence-on-failure) failed; \
         pre_spike was {}",
        post_spike,
        pre_spike
    );
}

#[test]
fn e2e_fill_from_producer_err_after_spill_preserves_state() {
    // fill_from triggers spill (cursor >= self.len), then producer
    // returns Err. The Err arm must NOT commit cursor / mark_written —
    // the writer's user-observable state is unchanged from before the
    // fill_from call. The internal spill (overflow Vec + repointed ptr)
    // IS committed, but that's an implementation detail.
    let (tt, mut pub_) = fresh_publisher("e2e/fill_err", 16 * 1024, 16);
    let mut sub = tt.subscriber("e2e/fill_err");

    // Warm sizer.
    for _ in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[1u8]).expect("set_data warm-up");
        });
    }
    while sub
        .try_view::<Image, _>(|_| ())
        .expect("try_view drain")
        .is_some()
    {}

    // Now: in a single proxy, write header + encoding (small), then
    // call fill_from_data with a producer that returns Err. The fill_from
    // would spill (cursor near loan exhaustion → ensure_capacity_for
    // fires). After producer Err, mark_written for `data` should NOT
    // be set. Drop sees missing variable field → no publish.
    {
        let mut proxy = pub_.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy
            .set_encoding(&"x".repeat(4096))
            .expect("set_encoding large");
        // fill_from_data: producer returns Err immediately.
        let r = proxy.fill_from_data(|_dst: &mut [u8]| {
            Err(TransportError::Internal {
                reason: "simulated producer Err".into(),
            })
        });
        assert!(r.is_err(), "fill_from must propagate producer Err");
        // Drop sees missing `data` → no publish.
    }

    let observed = sub.try_view::<Image, _>(|_| ()).expect("try_view drain");
    assert!(
        observed.is_none(),
        "producer Err must not result in a published frame"
    );
}

#[test]
fn e2e_n_consecutive_overflow_drops_increments_counter_monotonically() {
    // Pin that
    // `frames_dropped_overflow` is monotonically incremented across
    // multiple consecutive fault-injected re-loan failures. Catches a
    // regression where the counter resets, double-counts, or fails to
    // observe a subsequent failure after a successful intervening tick.
    // Ceiling large enough to fit 3 successive spike doublings.
    // Convergence-on-failure grows the loan each
    // iteration; we need headroom for 2x spike growth × 3 iterations.
    let (tt, mut pub_) = fresh_publisher("e2e/monotonic", 256 * 1024, 64);
    let _sub = tt.subscriber("e2e/monotonic");

    // Warm sizer with tiny payloads so the first adaptive loan is small.
    for _ in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[1u8]).expect("set_data warm-up");
        });
    }
    assert_eq!(pub_.frames_dropped_overflow(), 0);

    // Sequence: fault → spike → counter=1; success → counter unchanged;
    // fault → spike → counter=2; success → counter unchanged; fault →
    // spike → counter=3.
    //
    // Convergence-on-failure interaction:
    // because the sizer records the spike size even on Err arms,
    // each iteration's loan grows to fit the prior spike. Query the
    // current loan size and overshoot it by 2x so this tick still
    // spills even after the previous Err's record_payload_size grew
    // the window.
    for expected_count in 1..=3u64 {
        let current_loan = pub_.adaptive_loan_size_for_min_required(40);
        let spike_size = (current_loan as usize * 2).max(8 * 1024);
        pub_.fault_inject_send_overflow_frame_after(0);
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy
                .set_data(&vec![0xccu8; spike_size])
                .expect("set_data spike");
        });
        assert_eq!(
            pub_.frames_dropped_overflow(),
            expected_count,
            "counter must increment on each fault-injected re-loan failure \
             (iteration {expected_count}, spike size {spike_size}, prior loan {current_loan})"
        );
        // Steady-state tick between faults: counter unchanged.
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[2u8]).expect("set_data steady-state");
        });
        assert_eq!(
            pub_.frames_dropped_overflow(),
            expected_count,
            "counter must stay constant across steady-state ticks"
        );
    }
    assert_eq!(pub_.frames_dropped_overflow(), 3);
}

#[test]
fn e2e_partial_write_with_spill_does_not_increment_overflow_counter() {
    // Pin the semantic
    // distinction between "drop because of missing variable field" (a
    // user bug — should not increment `frames_dropped_overflow`) and
    // "drop because of overflow re-loan failure" (a system condition —
    // should increment). The missing-var gate runs BEFORE the overflow
    // path, so the counter must stay 0 on a partial-write + spill tick.
    let (tt, mut pub_) = fresh_publisher("e2e/partial_no_counter", 64 * 1024, 64);
    let mut sub = tt.subscriber("e2e/partial_no_counter");

    for _ in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[1u8]).expect("set_data warm-up");
        });
    }
    while sub
        .try_view::<Image, _>(|_| ())
        .expect("try_view drain")
        .is_some()
    {}

    {
        let mut proxy = pub_.loan_proxy::<Image>().expect("loan_proxy");
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        // Skip encoding → missing variable field.
        // But trigger spill on set_data so has_overflow=true at Drop.
        proxy
            .set_data(&vec![0u8; 8 * 1024])
            .expect("set_data spike");
        assert!(
            proxy.has_overflow(),
            "spill must fire to exercise the interaction"
        );
    }

    assert_eq!(
        pub_.frames_dropped_overflow(),
        0,
        "missing-variable-field drop must NOT increment the overflow counter; \
         the all_variables_written check runs BEFORE the overflow path"
    );
}

#[test]
fn e2e_invariant_violation_counter_is_zero_in_normal_operation() {
    // In-process mirror of
    // `iceoryx2_invariant_violation_counter_is_zero_in_normal_operation`.
    //
    // `frames_dropped_invariant_violation` is wired in
    // `OutputProxy::Drop` for THREE defensive branches:
    // - Contract violation `T::has_overflow == true && overflow_view_bytes == None`
    //   (hand-written buggy `impl ShmMessage`)
    // - Small original_bytes at overflow drop (< WireHeader::SIZE)
    //   (loan_proxy refactor break)
    // - Small loaned buf at steady-state drop (< 12 bytes)
    //   (publisher returned undersized sample)
    //
    // All three are documented-unreachable in production. This test
    // pins that contract over a `TestTransport` SHM root; the
    // `overflow_iox2_test.rs` mirror pins it on the process-global
    // singleton. A future refactor breaking one of those invariants
    // would flip the counter and FAIL this test.
    let (tt, mut pub_) = fresh_publisher("e2e/invariant_zero", 64 * 1024, 64);
    let _sub = tt.subscriber("e2e/invariant_zero");

    assert_eq!(pub_.frames_dropped_invariant_violation(), 0);

    // Exercise both steady-state and overflow paths through normal
    // Drop sequences. Neither should bump
    // `frames_dropped_invariant_violation`.
    for _ in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&[1u8]).expect("set_data warm-up");
        });
    }
    // Spike — exercises overflow Drop path.
    publish_tick(&mut pub_, |proxy| {
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy
            .set_data(&vec![0xeeu8; 32 * 1024])
            .expect("spike via spill");
    });

    assert_eq!(
        pub_.frames_dropped_invariant_violation(),
        0,
        "no invariant-violation Drop branch should fire in normal operation \
         over an isolated `TestTransport` SHM root"
    );
    // There is no `history_pushes_failed` counter to assert on:
    // native history retains the SHM frame by offset on send() (no
    // Cerulion-side history allocation).
}

// ============================================================
// The OVERFLOW send-success recovery site
// (`OutputProxy::Drop`, the spill else-branch calling `record_output_complete`)
// is exercised here; the STEADY-STATE recovery site is exercised by
// `output_proxy_test`'s recovery test. This drives a discard regime and then heals
// it with a COMPLETE publish whose payload takes the SPILL path, so the recovery
// `info!` + latch re-arm fire from the overflow site specifically.
// ============================================================

/// Open a discard regime (3 incomplete drops), then heal it with a payload spike
/// that SPILLS — proving the overflow send-success arm reports recovery once and
/// re-arms the loud discard path. Attribution to the overflow site: the ONLY
/// complete publish after the regime is the spike, and the subscriber receives
/// its full spilled payload (so the overflow re-loan + send succeeded → the
/// else-branch that hosts `record_output_complete` ran).
#[test]
#[traced_test]
fn overflow_path_recovery_info_and_rearm_e2e() {
    // 16 KiB ceiling, buffer 64 — same shape as `e2e_spill_round_trip`.
    let (tt, mut pub_) = fresh_publisher("e2e/overflow_recovery", 16 * 1024, 64);
    let mut sub = tt.subscriber("e2e/overflow_recovery");

    // Step 1: warm the adaptive sizer with 16 small (1 KiB) COMPLETE publishes
    // so the loan settles ~1.5 KiB (a later 12 KiB payload will exceed it →
    // spill). These are complete publishes on a HEALTHY latch → no recovery log.
    for i in 0..16 {
        publish_tick(&mut pub_, |proxy| {
            proxy.height = i;
            proxy.set_header_bytes(&[]).expect("set_header_bytes");
            proxy.set_encoding("g").expect("set_encoding");
            proxy.set_data(&vec![0u8; 1024]).expect("warmup");
        });
    }
    // Drain warm-up frames.
    while sub.try_view::<Image, _>(|_| ()).expect("drain").is_some() {}

    // Step 2: open a discard regime — 3 incomplete drops (no `data` written →
    // the all-variables gate discards on drop). #1 error!, #2/#3 debug{1,2}.
    for _ in 0..3 {
        publish_tick(&mut pub_, |_proxy| {
            // Write nothing → incomplete → discard on drop.
        });
    }
    assert_eq!(
        pub_.output_discard_count(),
        3,
        "the 3 incomplete drops opened a discard regime"
    );

    // Step 3: HEAL via a SPILL. 12 KiB payload exceeds the ~1.5 KiB warm loan
    // but fits the 16 KiB ceiling → the overflow re-loan path runs, and its
    // send-success arm calls `record_output_complete` → recovery info!.
    let spike_payload = vec![0x42u8; 12 * 1024];
    {
        let mut proxy = pub_.loan_proxy::<Image>().expect("loan_proxy");
        proxy.height = 999;
        proxy.set_header_bytes(&[]).expect("set_header_bytes");
        proxy.set_encoding("g").expect("set_encoding");
        proxy.set_data(&spike_payload).expect("heal via spill");
        // As in `e2e_partial_write_with_spill_does_not_publish`
        // / `e2e_spill_on_first_variable_write`, assert the SPILL/overflow path
        // was actually taken WHILE the proxy is alive. Without this, an adaptive-
        // sizer change that grows the warm loan to >= 12 KiB would fit the spike
        // in the DIRECT loan (no spill) — the full payload would still deliver and
        // `frames_dropped_overflow()` would still be 0, so the test would SILENTLY
        // degrade to re-covering the STEADY-STATE recovery site instead of the
        // OVERFLOW site it claims to exercise. `has_overflow()` makes that regression
        // fail loudly.
        assert!(
            proxy.has_overflow(),
            "the 12 KiB heal spike MUST take the spill/overflow path (warm loan ~1.5 KiB) — \
             otherwise the recovery info! below is NOT attributable to the overflow site"
        );
        // proxy drops here → publishes the spilled frame; the send-success arm
        // calls `record_output_complete` → recovery info!.
    }

    // The spilled heal frame DELIVERED in full — proves the overflow re-loan +
    // send succeeded (the branch that hosts the recovery `record_output_complete`),
    // so the recovery below is attributable to the OVERFLOW site, not steady-state.
    let observed = sub
        .try_view::<Image, _>(|view| (view.height, view.data().to_vec()))
        .expect("try_view")
        .expect("healed spill frame present");
    assert_eq!(observed.0, 999, "heal spike height round-trips");
    assert_eq!(
        observed.1, spike_payload,
        "subscriber sees the full 12 KiB spilled payload (overflow path taken)"
    );
    assert_eq!(
        pub_.frames_dropped_overflow(),
        0,
        "the heal spill SUCCEEDED (no overflow drop) — the success arm ran"
    );

    // Step 4: re-arm check — a fresh incomplete drop must log error! again.
    publish_tick(&mut pub_, |_proxy| {});
    assert_eq!(
        pub_.output_discard_count(),
        4,
        "the re-armed discard is counted (3 regime-1 + 1 post-recovery)"
    );

    // Recovery `info!` fired EXACTLY ONCE from the overflow site, carrying the
    // closed regime's suppressed count (2 debug-downgraded). Per-line predicate
    // (both substrings on one line) so a wrong-count regression fails.
    logs_assert(|lines: &[&str]| {
        let recovery = count_at_exclusively(
            lines,
            "INFO",
            &["OutputProxy: output recovered", "suppressed_count=2"],
        )?;
        // Head discard (#1) + re-armed discard (step 4) = 2 loud error!s; the
        // sustained middle discards were debug-downgraded (proving the latch, not
        // a flood, drove the overflow-path recovery).
        // Matched WITH the level token AND against the level-free total of the
        // same marker: a head demoted to `warn!`/`info!` would otherwise still
        // count as a loud error!, and so would a second copy of it at another
        // level.
        let errors = count_at_exclusively(
            lines,
            "ERROR",
            &["dropped without writing all declared variable fields"],
        )?;
        if recovery != 1 {
            return Err(format!(
                "expected exactly 1 overflow-path 'output recovered' info! carrying \
                 suppressed_count=2, got {recovery}"
            ));
        }
        if errors != 2 {
            return Err(format!(
                "expected exactly 2 loud discard error!s (regime head + re-armed), got \
                 {errors} — a flood means the latch never downgraded; <2 means no re-arm"
            ));
        }
        Ok(())
    });
}
