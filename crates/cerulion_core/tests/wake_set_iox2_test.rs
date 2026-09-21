// SPDX-License-Identifier: AGPL-3.0-only
//! The public wake source
//! ([`WakeSource`](cerulion_core::wake::WakeSource) +
//! [`WakeSet`](cerulion_core::wake::WakeSet)) over REAL iceoryx2.
//!
//! # Why the publisher here is `create_ingress_publisher`
//!
//! The headline arms publish through
//! [`TransportManager::create_ingress_publisher`](cerulion_core::transport::TransportManager::create_ingress_publisher)
//! — netd's OWN desk-side mirror publisher, the exact producer a `cerulion-vizd`
//! wake will observe (the event-driven design). Using `create_publisher` instead would prove
//! the multiplexer works against a shape the desk never sees, and would also skip
//! the `arm_publish_raw_notify` that makes `publish_raw` wake anybody at
//! all. So these arms double as the no-inert-shipping proof that the production
//! mirror path really notifies.
//!
//! # What is pinned
//!
//! * **A publish wakes the set**, and the wake really is a WAKE — the wait
//!   returns well inside a timeout long enough that a timing-out implementation
//!   would be caught by the clock.
//! * **Demux is by DECLARATION ORDER.** N sources, one publisher fired → exactly
//!   that index. This is the arm a `wait` that always reports index 0 fails.
//! * **A quiet set spends its timeout and reports nothing** — asserted on the
//!   WALL as a lower bound (a timeout that returns instantly is the
//!   unbounded-spin class), never as a band load could invert.
//! * **An empty source slice does NOT block** and says so
//!   (`last_wait_blocked() == false`), which is what a caller must key its own
//!   pacing on.
//! * **The listener is REAL** — the producer's live listener count rises by
//!   exactly one while the source is held and falls back on drop (the listener-less tap's
//!   event-level proof, run in reverse: here a listener is the POINT, so its
//!   presence is the anti-tautology for every wake assertion in the file).
//! * **Open-only discipline** — a missing topic is a loud error naming the
//!   remedy, and mints nothing.
//! * **Capacity is refused before anything attaches**, driven at the real
//!   boundary through the production `wait` (a partial attach would silently drop
//!   the tail's wakes).
//!
//! Parallel-safe: per-test isolated iceoryx2 config (per-test SHM root), no
//! process-global state, no `#[serial]`.
//!
//! ```bash
//! cargo test -p cerulion_core --test wake_set_iox2_test
//! ```

use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wake::{WakeSet, WakeSource};
use cerulion_core::wire::{MaxSliceLen, WireHeader};

fn manager(name: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport manager")
}

/// A valid wire frame (32-byte header + body) — `create_ingress_publisher`'s
/// `publish_raw` takes the caller's frame verbatim.
fn wire_frame(seq: u32, body: &[u8]) -> Vec<u8> {
    let total = WireHeader::SIZE + body.len();
    let mut header = WireHeader::new(0x1, seq, seq as u64);
    header.total_size = total as u32;
    let mut frame = vec![0u8; total];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(body);
    frame
}

/// Wait once and return the fired indices as an owned Vec (the borrow ends with
/// the call, so an assertion can outlive it).
fn wait_fired(set: &mut WakeSet, sources: &[&WakeSource], timeout: Duration) -> Vec<usize> {
    set.wait(sources, timeout)
        .expect("a within-capacity wait never errors")
        .to_vec()
}

/// Drain any event queued before the arms below (a connection-lifecycle
/// notification the reactor's callback clears). Returns once the set is quiet,
/// so a later "quiet" assertion is about the ABSENCE of a publish rather than
/// about startup noise. Bounded: at most a handful of settle waits.
fn settle(set: &mut WakeSet, sources: &[&WakeSource]) {
    for _ in 0..8 {
        if wait_fired(set, sources, Duration::from_millis(30)).is_empty() {
            return;
        }
    }
    panic!("the wake set never went quiet — a source is firing with no publisher");
}

// ===========================================================================
// The headline: a publish on the PRODUCTION mirror-publisher shape wakes the set.
// ===========================================================================

#[test]
fn a_publish_on_an_ingress_publisher_wakes_the_set_promptly() {
    let mgr = manager("wake_headline");
    let topic = "/wake/headline";
    let mut publisher = mgr
        .create_ingress_publisher(topic, MaxSliceLen::const_new(64))
        .expect("the mirror publisher creates the topic");

    let source = mgr
        .create_wake_listener(topic)
        .expect("a wake listener attaches to a live topic");
    let mut set = WakeSet::new(1).expect("wake set");
    let sources = [&source];
    settle(&mut set, &sources);

    // A generous timeout: if the wake did NOT work the wait would consume all of
    // it, and the elapsed assertion below would fail by an order of magnitude.
    // A CEILING is load-safe here — contention can only make a wake LATER, so a
    // loaded runner cannot turn a broken wake into a passing one.
    const TIMEOUT: Duration = Duration::from_secs(2);
    publisher
        .publish_raw(&wire_frame(0, b"hello"))
        .expect("publish");

    let started = Instant::now();
    let fired = wait_fired(&mut set, &sources, TIMEOUT);
    let elapsed = started.elapsed();

    assert_eq!(
        fired,
        vec![0],
        "the one source must fire (a publish on netd's own ingress publisher \
         notifies — publish_raw is armed to notify on it)"
    );
    assert!(
        set.last_wait_blocked(),
        "the wait attached its source and blocked on it"
    );
    assert!(
        elapsed < Duration::from_millis(500),
        "the wake must arrive promptly, not by timing out: waited {elapsed:?} of {TIMEOUT:?}"
    );
}

// ===========================================================================
// Demux: N sources → the RIGHT indices, in declaration order.
// ===========================================================================

#[test]
fn n_sources_demux_to_the_firing_indices_in_declaration_order() {
    let mgr = manager("wake_demux");
    let topics = ["/wake/demux/a", "/wake/demux/b", "/wake/demux/c"];
    let mut publishers: Vec<_> = topics
        .iter()
        .map(|t| {
            mgr.create_ingress_publisher(t, MaxSliceLen::const_new(64))
                .expect("mirror publisher")
        })
        .collect();
    let owned: Vec<WakeSource> = topics
        .iter()
        .map(|t| mgr.create_wake_listener(t).expect("wake listener"))
        .collect();
    let sources: Vec<&WakeSource> = owned.iter().collect();
    let mut set = WakeSet::new(sources.len()).expect("wake set");
    settle(&mut set, &sources);

    // Fire the MIDDLE source only. A `wait` that always reports index 0 — or one
    // that reports every attached source — fails here against a hand oracle.
    publishers[1]
        .publish_raw(&wire_frame(0, b"b"))
        .expect("publish");
    assert_eq!(
        wait_fired(&mut set, &sources, Duration::from_secs(2)),
        vec![1],
        "only the source whose topic was published on may fire"
    );

    // Fire the OUTER two. The fired set must come back ASCENDING regardless of
    // the order iceoryx2 happened to invoke the callback in.
    publishers[2]
        .publish_raw(&wire_frame(1, b"c"))
        .expect("publish");
    publishers[0]
        .publish_raw(&wire_frame(1, b"a"))
        .expect("publish");
    let fired = wait_fired(&mut set, &sources, Duration::from_secs(2));
    assert_eq!(
        fired,
        vec![0, 2],
        "both fired sources report, in DECLARATION order (ascending index), and \
         the un-published middle source does not"
    );

    // The topic label rides along for diagnostics — a fired index is only useful
    // if the caller can name it.
    assert_eq!(owned[1].topic(), topics[1]);
}

// ===========================================================================
// Quiet + empty: the two shapes a pacing caller depends on.
// ===========================================================================

#[test]
fn a_quiet_set_spends_its_timeout_and_reports_nothing() {
    let mgr = manager("wake_quiet");
    let topic = "/wake/quiet";
    let _publisher = mgr
        .create_ingress_publisher(topic, MaxSliceLen::const_new(64))
        .expect("mirror publisher");
    let source = mgr.create_wake_listener(topic).expect("wake listener");
    let mut set = WakeSet::new(1).expect("wake set");
    let sources = [&source];
    settle(&mut set, &sources);

    // Nothing is published. The wait must consume its timeout — a LOWER bound on
    // the wall, which load can only lengthen. An implementation that returned
    // instantly would turn a caller's loop into the unbounded spin, and
    // that is exactly what this catches.
    const TIMEOUT: Duration = Duration::from_millis(300);
    let started = Instant::now();
    let fired = wait_fired(&mut set, &sources, TIMEOUT);
    let elapsed = started.elapsed();

    assert!(
        fired.is_empty(),
        "a quiet set reports no fired source, got {fired:?}"
    );
    assert!(
        set.last_wait_blocked(),
        "the source WAS attached, so the wait genuinely blocked"
    );
    assert!(
        elapsed >= TIMEOUT.mul_f64(0.8),
        "the wait must spend its timeout on a quiet set, not return instantly: \
         waited {elapsed:?} of {TIMEOUT:?}"
    );
}

#[test]
fn an_empty_source_slice_does_not_block_and_says_so() {
    let mut set = WakeSet::new(0).expect("wake set");
    let started = Instant::now();
    let fired = wait_fired(&mut set, &[], Duration::from_secs(5));
    let elapsed = started.elapsed();

    assert!(fired.is_empty(), "nothing attached, nothing fires");
    assert!(
        !set.last_wait_blocked(),
        "an empty slice must report that it did NOT block — a caller keying its \
         pacing on the timeout would otherwise spin at 100% CPU with no taps"
    );
    assert!(
        elapsed < Duration::from_millis(500),
        "an empty wait must return immediately (it consumed {elapsed:?} of a 5s timeout)"
    );
}

// ===========================================================================
// The listener is REAL (anti-tautology for every wake assertion above).
// ===========================================================================

#[test]
fn a_wake_source_holds_exactly_one_listener_and_releases_it_on_drop() {
    let mgr = manager("wake_listener_count");
    let topic = "/wake/count";
    let mut publisher = mgr
        .create_ingress_publisher(topic, MaxSliceLen::const_new(64))
        .expect("mirror publisher");

    let base = publisher.event_listener_count_for_test();
    let source = mgr.create_wake_listener(topic).expect("wake listener");
    assert_eq!(
        publisher.event_listener_count_for_test(),
        base + 1,
        "a wake source registers EXACTLY ONE event listener — this is the cost \
         the notify-elision and listener-less-tap rulings are about, and it must be visible"
    );

    // A data-only tap alongside it changes nothing (the invariant still
    // holds for the TAP — the wake is a sibling, not a replacement).
    let _tap = mgr
        .create_data_only_subscriber(topic)
        .expect("data-only tap");
    assert_eq!(
        publisher.event_listener_count_for_test(),
        base + 1,
        "the data-only tap contributes ZERO listeners; only the wake source does"
    );

    drop(source);
    // The count is read through the publisher's own live view, which refreshes on
    // its next event sweep; a publish drives that sweep deterministically.
    publisher
        .publish_raw(&wire_frame(0, b"x"))
        .expect("publish");
    assert_eq!(
        publisher.event_listener_count_for_test(),
        base,
        "dropping the wake source releases its listener (so the producer's notify \
         elision gate can re-arm within one publish)"
    );
}

// ===========================================================================
// Open-only discipline + capacity.
// ===========================================================================

#[test]
fn a_wake_listener_on_a_missing_topic_fails_loudly_and_mints_nothing() {
    let mgr = manager("wake_missing");
    let topic = "/wake/never/created";

    let err = mgr
        .create_wake_listener(topic)
        .expect_err("a wake listener never creates a topic");
    let msg = err.to_string();
    assert!(
        msg.contains(topic),
        "the error must name the offending topic: {msg}"
    );
    assert!(
        msg.contains("does not exist"),
        "the error must say the topic does not exist: {msg}"
    );
    assert!(
        msg.contains("cerulion topic list"),
        "the error must name the remedy verb: {msg}"
    );
    assert!(
        mgr.data_service_missing(topic),
        "the failed open must leave NO phantom data service behind"
    );
}

#[test]
fn a_wait_past_the_attachment_cap_is_refused_before_anything_attaches() {
    let mgr = manager("wake_capacity");
    let topic = "/wake/capacity";
    let _publisher = mgr
        .create_ingress_publisher(topic, MaxSliceLen::const_new(64))
        .expect("mirror publisher");
    let source = mgr.create_wake_listener(topic).expect("wake listener");
    let mut set = WakeSet::new(1).expect("wake set");

    // The cap is on ATTACHMENTS, so one real listener referenced N times is a
    // faithful over-cap slice — and it makes the boundary reachable without
    // provisioning a thousand ports (iceoryx2's event default is 16 listeners,
    // so a thousand REAL sources on one topic is not even constructible).
    let cap = cerulion_core::graph::WAITSET_MAX_ATTACHMENTS;
    let at_cap: Vec<&WakeSource> = (0..cap).map(|_| &source).collect();
    let over_cap: Vec<&WakeSource> = (0..cap + 1).map(|_| &source).collect();

    let err = set
        .wait(&over_cap, Duration::from_millis(1))
        .expect_err("one past the cap must be refused");
    let msg = err.to_string();
    assert!(
        msg.contains(&format!("{}", cap + 1)) && msg.contains(&format!("{cap}")),
        "the refusal must name both the offending count and the cap: {msg}"
    );
    assert!(
        !set.last_wait_blocked(),
        "a refused wait must not have attached or blocked"
    );

    // ANTI-TAUTOLOGY: the cap itself is admitted, so the refusal is a threshold
    // rather than a blanket rejection of every large slice. (iceoryx2 rejects a
    // duplicate attach, so most of these fail to attach and warn — the point here
    // is only that `wait` itself did not refuse the call.)
    let _ = set
        .wait(&at_cap, Duration::from_millis(1))
        .expect("a slice AT the cap is admitted");
}
