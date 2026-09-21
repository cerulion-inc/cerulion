// SPDX-License-Identifier: AGPL-3.0-only
//! The record/replay capture tap is a listener-less
//! [`DataOnlySubscriber`](cerulion_core::transport::subscriber::DataOnlySubscriber).
//!
//! # What this pins
//!
//! - **(A) The event-level ZERO proof (the headline).** A producer + a data-only
//!   tap: the producer's event service `number_of_listeners()` stays at the
//!   same-process consumer count (the tap contributes ZERO). Attaching the OLD
//!   listener-full open-only tap raises the count by one, so reverting the
//!   recorder/replay tap wiring fails test A. Because iceoryx2's
//!   `FailedToDeliverSignal` storm can only exist when a listener is connected,
//!   `listener-count == consumer-count` is a DIRECT, log-independent proof of
//!   zero failed sends to the tap, which is the bar this tap must clear.
//! - **(B) Record completeness (Principle #6/#7).** The data-only tap captures a
//!   published burst byte-identical to a hand oracle via `drain_owned` polling
//!   alone (no listener), including a STALLED-then-resumed drain, byte-identical
//!   across two runs.
//! - **(C) `has_samples()` is a NON-CONSUMING emptiness query.** The
//!   query the gateway's one-budget-per-pass egress drain closes its liveness
//!   advancement baseline on, asked against a queue it is still FORWARDING from.
//!   The boolean verdicts alone cannot see the contract — a consuming probe
//!   answers `true` just as happily while swallowing an egress frame — so the pin
//!   is that a subsequent `drain_all` still yields ALL the published frames,
//!   byte-identical to a hand oracle.
//!
//! Parallel-safe: each test uses its own isolated iceoryx2 config (per-test SHM
//! root). No `#[serial]` needed (no process-global state).
//!
//! ```bash
//! cargo test -p cerulion_core --test data_only_tap_iox2_test -- --test-threads=1
//! ```

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};

fn manager_with(name: &str, buffer: usize, ix: iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: buffer,
            network: None,
        },
        ix,
    )
    .expect("init isolated transport manager")
}

/// A valid wire frame (32-byte header + body) so the tap's
/// `OwnedInboundSample::payload()` slices to exactly `total_size` — the frame we
/// published, byte-for-byte (a padded loan slot would otherwise trail capacity).
fn wire_frame(seq: u32, body: &[u8]) -> Vec<u8> {
    let total = WireHeader::SIZE + body.len();
    let mut header = WireHeader::new(0x1, seq, seq as u64);
    header.total_size = total as u32;
    let mut frame = vec![0u8; total];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(body);
    frame
}

// ===========================================================================
// (A) THE EVENT-LEVEL ZERO PROOF + mutation control.
// ===========================================================================

#[test]
fn data_only_tap_registers_no_event_listener_but_listener_full_tap_does() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = manager_with("zero_proof", 64, ix);
    let topic = "/zero/topic";
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(64), 0)
        .expect("publisher creates the topic (data + event services)");

    // Baseline: the publisher's event service starts with the publisher's OWN
    // listener (a Cerulion publisher listens for SubscriberConnected/Disconnected
    // to drive late-joiner history). No data consumer exists yet, so this is the
    // "same-process consumer count" reference the tap must not perturb.
    let base = publisher.event_listener_count_for_test();

    // HEADLINE: a DATA-ONLY capture tap opens ONLY the data service — it
    // registers NO event listener, so the producer's live listener count is
    // UNCHANGED. With no connected listener, iceoryx2's notifier send loop has
    // nothing to send to → zero failed sends, structurally, at default sysctls.
    let data_tap = mgr
        .create_data_only_subscriber(topic)
        .expect("data-only capture tap attaches");
    assert_eq!(
        publisher.event_listener_count_for_test(),
        base,
        "a DATA-ONLY tap must register ZERO event listeners (the listener-less fix); \
         count stayed at the baseline"
    );

    // Publishing + notifying with a data-only tap attached never fails a send.
    // (The publisher's OWN listener is the only one connected — the tap adds
    // none — so no NEW listener socket exists to fill; the tap storm is
    // structurally impossible.)
    for i in 0..256u32 {
        publisher
            .publish_raw(&wire_frame(i, &[(i & 0xff) as u8]))
            .expect("publish");
        publisher.notify_sent_sample().expect("notify never errors");
    }
    // The listener count is STILL `base` after a full burst — the tap never grew
    // a listener (the load-bearing invariant).
    assert_eq!(
        publisher.event_listener_count_for_test(),
        base,
        "the data-only tap remains listener-less across the whole burst"
    );

    // The OLD listener-full open-only tap (a `CerulionSubscriber`) DOES create an
    // event listener → the count rises by exactly one. This is what the
    // listener-less tap removed; reverting the headline call above from
    // `create_data_only_subscriber` to `create_subscriber_open_only` (the
    // constructor change this test guards against) makes test A's headline
    // assert fail. (The recorder/replay call sites are wired to
    // `DataOnlySubscriber` by TYPE — `TapState.sub` / `Capture.subscriber` — so
    // a reverted constructor there is a compile error, not a runtime miss.)
    let listener_full_tap = mgr
        .create_subscriber_open_only(topic)
        .expect("listener-full open-only tap attaches");
    assert_eq!(
        publisher.event_listener_count_for_test(),
        base + 1,
        "a listener-full (`create_subscriber_open_only`) tap registers exactly ONE \
         event listener — the connection that made the tap storm possible"
    );

    drop(data_tap);
    drop(listener_full_tap);
}

// ===========================================================================
// (B) Record completeness (Principle #6) + stall/resume + determinism (#7).
// ===========================================================================

/// Capture every frame `drain_owned` yields until the SHM queue is empty,
/// copying each frame's bytes out and releasing the borrow before re-draining
/// (so held borrows never exceed the budget — exactly the bagd/replay pattern).
fn drain_all(tap: &mut cerulion_core::transport::subscriber::DataOnlySubscriber) -> Vec<Vec<u8>> {
    let budget = tap.max_borrowed_samples().saturating_sub(1).max(1);
    let mut scratch = Vec::new();
    let mut got = Vec::new();
    loop {
        scratch.clear();
        let n = tap.drain_owned(budget, &mut scratch).expect("drain_owned");
        for sample in scratch.drain(..) {
            got.push(sample.payload().to_vec());
        }
        if n < budget {
            break; // queue drained.
        }
    }
    got
}

/// One capture run: publish `n` distinct frames (STALLING the drain until every
/// frame is queued — no listener socket to fill, so a stall is safe), then drain
/// the whole backlog via the data-only tap. Returns the captured frame bytes.
fn capture_run(name: &str, n: u32) -> Vec<Vec<u8>> {
    let ix = cerulion_core::testing::iceoryx_test_config();
    // Queue deep enough to hold the whole burst across the stall (no drop-oldest).
    let mgr = manager_with(name, (n as usize) + 16, ix);
    let topic = "/complete/topic";
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(64), 0)
        .expect("publisher");
    let mut tap = mgr
        .create_data_only_subscriber(topic)
        .expect("data-only tap");

    // STALL: publish + notify the whole burst WITHOUT draining. With no listener
    // there is no notification socket to fill — the storm is impossible even in
    // principle; the SHM data queue simply holds the frames.
    for i in 0..n {
        publisher
            .publish_raw(&wire_frame(i, &[(i & 0xff) as u8, (i >> 8) as u8]))
            .expect("publish");
        publisher.notify_sent_sample().expect("notify");
    }

    // RESUME: drain the whole backlog off the SHM queue.
    drain_all(&mut tap)
}

#[test]
fn data_only_tap_captures_the_full_burst_byte_identical_and_deterministically() {
    const N: u32 = 200;

    // Hand oracle (NOT a self-compare): the exact frames the producer published.
    let oracle: Vec<Vec<u8>> = (0..N)
        .map(|i| wire_frame(i, &[(i & 0xff) as u8, (i >> 8) as u8]))
        .collect();

    let run1 = capture_run("complete_1", N);
    assert_eq!(
        run1, oracle,
        "the data-only tap must capture EVERY published frame byte-identical, via \
         drain_owned polling alone (no listener), even after a full-burst stall"
    );

    // Determinism (Principle #7): a second independent run yields the identical
    // capture == the same hand oracle.
    let run2 = capture_run("complete_2", N);
    assert_eq!(
        run2, oracle,
        "a second capture run is byte-identical to the hand oracle (deterministic)"
    );
}

// ===========================================================================
// (C) `has_samples()` is a NON-CONSUMING emptiness query.
// ===========================================================================

/// The network gateway's one-budget-per-pass egress drain closes its advancement
/// baseline on `has_samples()` (`DrainObservation::queue_emptied`), so the query
/// runs against a queue the caller is still FORWARDING from. Non-consuming is the
/// load-bearing property, and the boolean verdicts alone cannot distinguish it: a
/// probe that drained a frame to answer the same question answers `true` just as
/// happily while silently swallowing an egress frame (Principle #6).
///
/// Implementing `has_samples()` as "receive one and discard" makes the
/// captured burst come back short by exactly `PROBES` frames.
#[test]
fn has_samples_answers_without_consuming_a_frame() {
    const N: u32 = 8;
    const PROBES: usize = 3;

    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = manager_with("has_samples", (N as usize) + 16, ix);
    let topic = "/has_samples/topic";
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(64), 0)
        .expect("publisher");
    let mut tap = mgr
        .create_data_only_subscriber(topic)
        .expect("data-only tap");

    // EMPTY: nothing has been published — the correct answer is `false`.
    assert!(
        !tap.has_samples().expect("has_samples on an empty queue"),
        "an empty receive queue must answer false"
    );

    for i in 0..N {
        publisher
            .publish_raw(&wire_frame(i, &[(i & 0xff) as u8, (i >> 8) as u8]))
            .expect("publish");
    }

    // NON-EMPTY, asked repeatedly. Every probe answers `true`; a CONSUMING probe
    // would answer `true` too, which is why the verdicts are not the pin.
    for probe in 0..PROBES {
        assert!(
            tap.has_samples().expect("has_samples with frames queued"),
            "probe {probe} must see the queued frames"
        );
    }

    // THE PIN: the drain still yields ALL N frames, byte-identical to the hand
    // oracle. A consuming probe leaves N - PROBES here.
    let oracle: Vec<Vec<u8>> = (0..N)
        .map(|i| wire_frame(i, &[(i & 0xff) as u8, (i >> 8) as u8]))
        .collect();
    let got = drain_all(&mut tap);
    assert_eq!(
        got, oracle,
        "{PROBES} emptiness queries must not have moved the data plane — the tap \
         still holds every published frame"
    );

    // And once drained it flips back to `false`.
    assert!(
        !tap.has_samples().expect("has_samples after the drain"),
        "a drained queue must answer false again"
    );
}
