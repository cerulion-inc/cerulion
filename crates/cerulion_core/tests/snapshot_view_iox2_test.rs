// SPDX-License-Identifier: AGPL-3.0-only
//! Step-boundary snapshot R1 "accounting-once" contract.
//!
//! `CerulionSubscriber` gained a step-boundary snapshot pair:
//!
//! - [`CerulionSubscriber::snapshot_latest`] drains to the latest sample AND
//!   runs ALL backpressure accounting (drop_oldest eviction count, sample(N)
//!   decimation, the `block` outstanding mirror, the expect_within anchor)
//!   EXACTLY ONCE, then freezes the result.
//! - the following [`CerulionSubscriber::try_view`] serves the frozen slot
//!   WITHOUT re-draining / re-accounting.
//!
//! # The R1 contract these tests prove
//!
//! The drain + accounting runs EXACTLY ONCE per `(input, snapshot)`. A
//! re-drain in `try_view` would silently DOUBLE a backpressure counter. We
//! prove this against an ORACLE / CONTROL (never a self-compare): every
//! `snapshot_latest()`-then-`try_view()` result is compared against a direct
//! single-drain control on an identical stream, and the count after the view
//! is asserted UNCHANGED from the count after the snapshot (the anti-double-
//! count mutation pin — a re-drain would bump it again).
//!
//! The runtime does NOT yet call `snapshot_latest` (executor wiring is chunk
//! 4), so these tests drive `snapshot_latest` / `try_view` DIRECTLY on raw
//! subscribers.
//!
//! # Running
//!
//! iceoryx2 SHM singleton → serial, one thread:
//!
//! ```bash
//! cargo test -p cerulion_core --test snapshot_view_iox2_test -- --test-threads=1
//! ```

use cerulion_core::scheduler::BackpressureCounters;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

/// Monotonic counter so re-runs don't collide on the iceoryx2 service name.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/snapshot_view/{base}/{nanos}/{id}")
}

/// Let iceoryx2 surface published samples on the subscriber side.
fn settle() {
    std::thread::sleep(Duration::from_millis(50));
}

/// Publish a single `Vector3` whose `x` is `val` (deterministic per-frame
/// stamp so the body read can assert WHICH sample is served). The loan is
/// dropped (and the frame sent) immediately, freeing the publisher loan slot
/// so a publish loop never exhausts the pool.
fn publish_x(publisher: &mut cerulion_core::CerulionPublisher, val: f64) {
    let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
    proxy.x = val;
    proxy.y = 0.0;
    proxy.z = 0.0;
}

// ---------------------------------------------------------------------------
// 1. snapshot_then_view serves the same latest-wins sample as a direct drain.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn snapshot_then_view_serves_latest_byte_identical_to_direct_drain() {
    // Two subscribers on the SAME topic+stream. SubA (CONTROL) drains live via
    // try_view; SubB snapshot_latest()s then try_view()s. Both must serve the
    // SAME latest-wins sample, byte-identical — proving the snapshot captures
    // the identical latest sample a direct drain would.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("latest_identical");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut sub_a = mgr.create_subscriber(&topic).expect("create sub_a");
    let mut sub_b = mgr.create_subscriber(&topic).expect("create sub_b");

    // A short deterministic stream; latest-wins should pick the LAST value.
    for v in [1.0, 2.0, 3.0, 4.0] {
        publish_x(&mut publisher, v);
    }
    settle();

    // CONTROL: direct live drain.
    let control = sub_a
        .try_view::<Vector3, _>(|view| (view.x, view.y, view.z))
        .expect("control try_view")
        .expect("control sees the latest sample");

    // FROZEN: snapshot then view.
    sub_b.snapshot_latest();
    let frozen = sub_b
        .try_view::<Vector3, _>(|view| (view.x, view.y, view.z))
        .expect("frozen try_view")
        .expect("frozen serves the latest sample");

    assert_eq!(
        control, frozen,
        "snapshot+view must serve the byte-identical latest-wins sample the \
         direct drain serves (control {control:?} vs frozen {frozen:?})"
    );
    assert_eq!(
        frozen.0, 4.0,
        "latest-wins must serve the LAST published value (4.0), got {}",
        frozen.0
    );
}

// ---------------------------------------------------------------------------
// 2. drop_oldest eviction accounting runs exactly once.
// ---------------------------------------------------------------------------

/// Build a `drop_oldest`-probed subscriber on `topic`, establish its baseline
/// with one warm-up frame + drain, then publish `flood` frames (overflowing
/// the default 16-deep subscriber queue so iceoryx2 evicts the oldest), and
/// run `final_drain` over it. Returns the post-drain `drop_oldest_count`.
///
/// `final_drain` is the variable under test: a plain `try_view` (the CONTROL,
/// one drain) vs. `snapshot_latest()` + `try_view()` (the FROZEN path, whose
/// view must add NOTHING).
fn probed_drop_oldest_count(
    mgr: &TransportManager,
    topic: &str,
    flood: u32,
    mut final_drain: impl FnMut(&mut cerulion_core::CerulionSubscriber, &Arc<BackpressureCounters>),
) -> u64 {
    let mut publisher = mgr
        .create_publisher_simple(topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(topic).expect("create subscriber");
    let counters = Arc::new(BackpressureCounters::new());
    subscriber.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("snapshot_node"),
        Arc::from("probed_in"),
        16,
    );

    // Warm-up: establish the per-stream baseline (this drain counts no
    // eviction — first sighting baseline-establishes uncounted).
    publish_x(&mut publisher, 0.0);
    settle();
    let warmed = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("warm-up try_view");
    assert!(warmed.is_some(), "warm-up frame must arrive");
    assert_eq!(
        counters.drop_oldest_count.load(Ordering::Acquire),
        0,
        "the baseline-establishing drain counts no eviction"
    );

    // Flood: overflow the 16-deep subscriber queue between drains so iceoryx2
    // silently evicts the oldest — the wire-seq gap the probe must count.
    for i in 0..flood {
        publish_x(&mut publisher, (i + 1) as f64);
    }
    settle();

    final_drain(&mut subscriber, &counters);
    counters.drop_oldest_count.load(Ordering::Acquire)
}

#[test]
#[serial]
fn snapshot_accounts_drop_oldest_exactly_once() {
    let mgr = TransportManager::get_or_init().expect("init");
    // 40 frames into a 16-deep queue → ~24 evictions. The exact number depends
    // on iceoryx2 surfacing; we assert >0 on each stream and the same-stream
    // accounting-once pin (frozen == count_after_snapshot).
    let flood = 40;

    // CONTROL: a plain single try_view drain on an identical stream.
    let control_topic = unique_topic("drop_control");
    let control_n = probed_drop_oldest_count(&mgr, &control_topic, flood, |sub, _c| {
        sub.try_view::<Vector3, _>(|view| view.x)
            .expect("control drain")
            .expect("control sees the surviving latest sample");
    });
    assert!(
        control_n > 0,
        "the flood must force real iceoryx2 evictions (control counted {control_n})"
    );

    // FROZEN: snapshot drains+accounts; the view must add NOTHING.
    let frozen_topic = unique_topic("drop_frozen");
    let mut count_after_snapshot = 0u64;
    let frozen_n = probed_drop_oldest_count(&mgr, &frozen_topic, flood, |sub, c| {
        sub.snapshot_latest();
        // Accounting happened at snapshot time.
        count_after_snapshot = c.drop_oldest_count.load(Ordering::Acquire);
        // The view serves the frozen slot — re-drain would DOUBLE the count.
        let served = sub
            .try_view::<Vector3, _>(|view| view.x)
            .expect("frozen drain")
            .expect("frozen serves the surviving latest sample");
        assert!(served > 0.0, "the served sample is a real flooded frame");
    });

    assert!(
        count_after_snapshot > 0,
        "snapshot_latest must run the eviction accounting (got {count_after_snapshot})"
    );
    assert_eq!(
        frozen_n, count_after_snapshot,
        "the try_view after snapshot must account NOTHING — count must be \
         UNCHANGED from snapshot time ({count_after_snapshot}) but was \
         {frozen_n} (a re-drain double-counted)"
    );
    // NB: we deliberately do NOT assert exact cross-service equality
    // (`frozen_n == control_n`). The control and frozen streams are TWO
    // independent iceoryx2 services; their exact eviction counts can differ
    // under scheduler/surfacing jitter on a loaded CI runner (same flood, but
    // iceoryx2 surfaces samples on each service independently). The real
    // accounting-once contract is the SAME-STREAM pin above
    // (`frozen_n == count_after_snapshot`): the view added nothing on the
    // frozen stream. The control's `control_n > 0` only proves the flood
    // evicts at all — not that the two services evict an identical amount.
}

// ---------------------------------------------------------------------------
// 3. sample(N) decimation accounting runs exactly once.
// ---------------------------------------------------------------------------

/// Build a `sample(N)`-gated subscriber, accept one frame (the gate's first
/// accept), publish a second frame well inside the gate window (it will
/// decimate), then run `final_drain`. Returns the post-drain `sampled_count`.
fn sample_gate_decimate_count(
    mgr: &TransportManager,
    topic: &str,
    mut final_drain: impl FnMut(
        &mut cerulion_core::CerulionSubscriber,
        &Arc<BackpressureCounters>,
    ) -> Option<f64>,
) -> u64 {
    let mut publisher = mgr
        .create_publisher_simple(topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(topic).expect("create subscriber");
    let counters = Arc::new(BackpressureCounters::new());
    // 10s interval: any second frame published ~50ms later falls inside the
    // window and is decimated.
    subscriber.register_sample_gate_for_test(
        10_000,
        Arc::clone(&counters),
        Arc::from("snapshot_node"),
        Arc::from("sampled_in"),
        16,
    );

    // Frame 1 → first accept (no decimation).
    publish_x(&mut publisher, 1.0);
    settle();
    let accepted = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("accept try_view");
    assert_eq!(accepted, Some(1.0), "the first frame must be accepted");
    assert_eq!(
        counters.sampled_count.load(Ordering::Acquire),
        0,
        "the accept path decimates nothing"
    );

    // Frame 2, well inside the 10s window → decimated.
    publish_x(&mut publisher, 2.0);
    settle();

    let served = final_drain(&mut subscriber, &counters);
    assert_eq!(
        served, None,
        "a decimated frame is served as Ok(None), got {served:?}"
    );
    counters.sampled_count.load(Ordering::Acquire)
}

#[test]
#[serial]
fn snapshot_accounts_sample_gate_exactly_once() {
    let mgr = TransportManager::get_or_init().expect("init");

    // CONTROL: a plain single try_view drain decimates frame 2 exactly once.
    let control_topic = unique_topic("sample_control");
    let control_n = sample_gate_decimate_count(&mgr, &control_topic, |sub, _c| {
        sub.try_view::<Vector3, _>(|view| view.x)
            .expect("control decimate drain")
    });
    assert_eq!(
        control_n, 1,
        "a single drain decimates the one too-soon frame exactly once"
    );

    // FROZEN: snapshot drains+decimates; the view must add NOTHING.
    let frozen_topic = unique_topic("sample_frozen");
    let mut count_after_snapshot = 0u64;
    let frozen_n = sample_gate_decimate_count(&mgr, &frozen_topic, |sub, c| {
        sub.snapshot_latest();
        count_after_snapshot = c.sampled_count.load(Ordering::Acquire);
        sub.try_view::<Vector3, _>(|view| view.x)
            .expect("frozen decimate drain")
    });

    assert_eq!(
        count_after_snapshot, 1,
        "snapshot_latest must run the sample(N) decimation (got {count_after_snapshot})"
    );
    assert_eq!(
        frozen_n, count_after_snapshot,
        "the try_view after snapshot must decimate NOTHING — sampled_count \
         must be UNCHANGED from snapshot time ({count_after_snapshot}) but \
         was {frozen_n} (a re-drain double-counted)"
    );
    assert_eq!(
        frozen_n, control_n,
        "snapshot+view must decimate the SAME count a single drain does \
         (control {control_n} vs frozen {frozen_n})"
    );
}

// ---------------------------------------------------------------------------
// 4. block outstanding-mirror accounting runs exactly once.
// ---------------------------------------------------------------------------

/// Build a `block`-probed subscriber whose shared `outstanding` mirror starts
/// at `pre`, publish `n` frames, then run `final_drain`. Returns the
/// post-drain `outstanding` value. The drain decrements the mirror by the
/// number of frames it removed (`record_block_drained`) — that decrement is
/// the per-drain block accounting that must run exactly once.
fn block_drain_outstanding(
    mgr: &TransportManager,
    topic: &str,
    pre: u64,
    n: u32,
    mut final_drain: impl FnMut(&mut cerulion_core::CerulionSubscriber),
) -> u64 {
    let mut publisher = mgr
        .create_publisher_simple(topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(topic).expect("create subscriber");
    // The mirror is a `CreditWord` now, LOCAL here, since this
    // test models a same-process edge. `record_published_n(pre)` is what a
    // producer's `pre` publishes would have left behind.
    let outstanding = cerulion_core::credit::CreditWord::local(u32::MAX);
    outstanding.record_published_n(pre);
    let counters = Arc::new(BackpressureCounters::new());
    subscriber.register_block_probe_for_test(
        outstanding.clone(),
        // Threshold above `pre` so the at-threshold event arm doesn't fire —
        // we isolate the outstanding-mirror decrement as the accounting.
        pre + 100,
        Arc::clone(&counters),
        Arc::from("blocked_in"),
        16,
    );

    for i in 0..n {
        publish_x(&mut publisher, (i + 1) as f64);
    }
    settle();

    final_drain(&mut subscriber);
    outstanding.outstanding()
}

#[test]
#[serial]
fn snapshot_accounts_block_mirror_exactly_once() {
    let mgr = TransportManager::get_or_init().expect("init");
    let pre = 50u64;
    let n = 4u32; // 4 frames, well under the 16-deep queue → all 4 drained.

    // CONTROL: a single drain decrements the mirror by exactly `removed`.
    let control_topic = unique_topic("block_control");
    let control_outstanding = block_drain_outstanding(&mgr, &control_topic, pre, n, |sub| {
        let served = sub
            .try_view::<Vector3, _>(|view| view.x)
            .expect("control block drain");
        assert!(
            served.is_some(),
            "the block consumer sees the latest sample"
        );
    });
    let control_removed = pre - control_outstanding;
    assert!(
        control_removed > 0,
        "the drain must remove (and account for) the queued block frames \
         (pre {pre} → post {control_outstanding})"
    );

    // FROZEN: snapshot decrements the mirror; the view must decrement NOTHING.
    // The decrement happens entirely at snapshot_latest() time; the try_view
    // that follows serves the frozen slot and runs `record_block_drained`
    // zero more times. We prove accounting-once by comparing the TOTAL
    // decrement to the control's single-drain decrement (a re-drain would
    // double it).
    let frozen_topic = unique_topic("block_frozen");
    let frozen_outstanding = block_drain_outstanding(&mgr, &frozen_topic, pre, n, |sub| {
        sub.snapshot_latest();
        let served = sub
            .try_view::<Vector3, _>(|view| view.x)
            .expect("frozen block drain");
        assert!(served.is_some(), "frozen serves the latest sample");
    });

    let frozen_removed = pre - frozen_outstanding;
    assert!(
        frozen_removed > 0,
        "snapshot_latest must run the block outstanding-mirror decrement \
         (pre {pre} → post {frozen_outstanding})"
    );
    assert_eq!(
        frozen_removed, control_removed,
        "snapshot+view must decrement the block mirror by the SAME amount a \
         single drain does (control removed {control_removed} vs frozen \
         removed {frozen_removed}) — a re-drain in try_view would double the \
         decrement"
    );
}

// ---------------------------------------------------------------------------
// 5. frozen Empty serves Ok(None).
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn frozen_empty_serves_ok_none() {
    // snapshot_latest() on an EMPTY queue freezes FrozenSlot::Empty; the next
    // try_view serves Ok(None) — no panic, no spurious value.
    // This ALSO pins the pre-first-delivery WAIT gate — a never-delivered
    // queue keeps held_sample == None, so the Empty drain stays Empty (NOT a
    // replayed Held) → Ok(None) → the node waits, never a fabricated default.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("empty");

    // Create the publisher so the service exists (matched endpoints), but
    // publish nothing.
    let _publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");
    settle();

    subscriber.snapshot_latest();
    let served = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("empty try_view must not error");
    assert_eq!(
        served, None,
        "an empty-queue snapshot serves Ok(None), got {served:?}"
    );
}

// ---------------------------------------------------------------------------
// 6. frozen Err is replayed once, then drains live.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn frozen_err_replayed_once_then_drains_live() {
    // fault_inject_receive_after(0) errors the very next receive() inside the
    // snapshot drain. snapshot_latest() stores FrozenSlot::Err; the next
    // try_view replays that Err ONCE; a SUBSEQUENT try_view drains LIVE (the
    // frozen slot was consumed) and serves a freshly published good sample.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("err_replay");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Publish one frame so the drain has something to receive (the fault fires
    // on the receive call regardless).
    publish_x(&mut publisher, 1.0);
    settle();

    // Arm: the next receive() inside the snapshot drain errors once.
    subscriber.fault_inject_receive_after_for_test(0);
    subscriber.snapshot_latest();

    // The stored Err is replayed exactly once.
    let replayed = subscriber.try_view::<Vector3, _>(|view| view.x);
    assert!(
        replayed.is_err(),
        "the frozen Err must be replayed by the first try_view, got {replayed:?}"
    );

    // The frozen slot is now consumed (try_view took it); the fault hook has
    // cleared (fire-once). A fresh good sample must drain LIVE and be served.
    publish_x(&mut publisher, 9.0);
    settle();
    let live = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("after the replayed Err, try_view drains live without error")
        .expect("the freshly published good sample is served");
    assert_eq!(
        live, 9.0,
        "the post-Err live drain must serve the fresh sample (9.0), got {live}"
    );
}

// ---------------------------------------------------------------------------
// 7. clear-then-capture: double snapshot must not double-borrow.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn clear_then_capture_double_snapshot_no_double_borrow() {
    // snapshot_latest() then snapshot_latest() again with NO try_view between.
    // The second snapshot still serves the FRESHER sample. Note that the
    // old "never hold 2 borrowed" premise is STALE — `held_sample` is now
    // RETAINED across the drain (held + latest + receive-transient → peak 3,
    // budgeted by subscriber_max_borrowed_samples = 3 on snapshot-source
    // topics), so a fast producer no longer drops the prior sample before the
    // next drain. The burst-borrow peak is proven by the new burst test (later
    // chunk); here we just pin that the second snapshot serves the fresher value.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("double_snapshot");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // First sample.
    publish_x(&mut publisher, 1.0);
    settle();
    subscriber.snapshot_latest(); // freezes sample x=1.0

    // A FRESHER sample lands.
    publish_x(&mut publisher, 2.0);
    settle();
    // Second snapshot: `held_sample` is RETAINED (there is no
    // clear-then-capture anymore); with a single queued sample on this default-2
    // service the drain peaks at 2 borrows (held + receive-transient), so the
    // second snapshot simply UPDATES the hold to the fresher value. The borrow-3
    // burst case (held + latest + receive-transient) is pinned by
    // `non_trigger_hold_iox2_test::held_value_survives_multi_sample_burst`.
    subscriber.snapshot_latest(); // updates held 1.0 → 2.0, freezes x=2.0

    let served = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("double-snapshot try_view")
        .expect("the second snapshot serves a sample");
    assert_eq!(
        served, 2.0,
        "the second snapshot must serve the FRESHER sample (2.0), not the \
         first frozen one (1.0), got {served}"
    );
}

// ---------------------------------------------------------------------------
// 8. The frozen slot is SERVE-MANY — a serve does not consume it,
//    and only the next CAPTURE supersedes it.
// ---------------------------------------------------------------------------

/// The frozen-slot reuse contract, at the seam it is implemented on.
///
/// # What this replaced, and why the oracle INVERTED
///
/// This arm used to be `view_after_serving_frozen_drains_live`, and it pinned the
/// exact opposite: `try_view` took the frozen slot (`self.frozen.take()`), so a
/// SECOND read fell through to a live drain and served whatever had landed since.
///
/// That is the defect, not a feature. A `Data`-trigger node bursts `fire_count`
/// fires inside ONE step (`Scheduler::tick_data_burst`) while its plain
/// non-trigger `#[input]` context is snapshotted ONCE per level pass — so the
/// first fire consumed the frozen context and every later fire of that burst
/// found the slot empty and FELL TO A LIVE DRAIN, with two outcomes and neither
/// wanted. On a QUIET context topic the live drain found the queue empty and
/// returned `Ok(None)`, so the tick chain collapsed at the macro's
/// declaration-ordered `try_view`, capping the common controller shape (trigger
/// + config/map context) at one message per step. On a NON-quiet one it really
/// popped, leaking a same-step publish into the middle of a burst so the fires
/// of ONE step disagreed about their context (Principle #7) — measured on the
/// Period catch-up shape in `read_outcome_capture_iox2_test`'s arm (g). The
/// collapse residual is documented on `fifo_burst_within_step_iox2_test`'s
/// collapsed-tick arm.
///
/// A non-trigger input is latest-value BY CONTRACT, so serving the identical
/// bytes to every fire of one step is correct — and it is strictly MORE
/// deterministic than the fall-through it replaces, which is exactly what the
/// mid-step publish below is here to demonstrate.
///
/// # The two halves, and why neither alone is enough
///
/// * SERVE-MANY: a second read inside the step serves the SAME frozen value even
///   though a fresher sample is sitting in the queue. Reverting to
///   `self.frozen.take()` fails here with `2.0`.
/// * NOT-FOREVER (the anti-tautology half): the very next CAPTURE supersedes the
///   slot, so the third read serves the fresher sample. Without it, a `try_view`
///   hard-wired to replay the first value it ever served would pass the first
///   half.
#[test]
#[serial]
fn frozen_slot_serves_many_until_the_next_capture() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("serve_many");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // The step's capture.
    publish_x(&mut publisher, 1.0);
    settle();
    subscriber.snapshot_latest();
    let first = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("frozen try_view")
        .expect("frozen serves the snapshot sample");
    assert_eq!(
        first, 1.0,
        "the frozen view serves the snapshot sample (1.0)"
    );

    // A FRESHER sample lands mid-step — the same-step publish a burst's later
    // fires must NOT be able to observe.
    publish_x(&mut publisher, 2.0);
    settle();
    let second = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("the re-served frozen slot never errors")
        .expect("the frozen slot is still there to serve");
    assert_eq!(
        second, 1.0,
        "a SECOND read inside the step must serve the SAME frozen value (1.0), \
         never the sample that landed after the capture — got {second}. This is \
         what lets every fire of a Data burst read one consistent context"
    );

    // Anti-tautology: the slot is scoped to the STEP, not frozen forever. The
    // next capture supersedes it and the fresher sample is served.
    subscriber.snapshot_latest();
    let after_capture = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("post-capture try_view")
        .expect("the new capture serves a sample");
    assert_eq!(
        after_capture, 2.0,
        "the NEXT capture supersedes the slot — a re-served value must not \
         outlive its step, got {after_capture}"
    );
}

/// The `Empty` half of serve-many — the twin of `frozen_empty_serves_ok_none`,
/// which covers only the FIRST read.
///
/// # What this test catches
///
/// `try_view`'s serve-many arm returns in place for `Held` AND `Empty`. Dropping
/// just the `Empty` case (leaving `Held` sticky) passes every burst arm in
/// `fifo_burst_within_step_iox2_test`, because a burst on an EMPTY context
/// collapses at fire 1 and never reaches a second read — so nothing over there
/// can see it.
///
/// It still matters, and the shape is ordinary: a node that declares its TRIGGER
/// before a context input that has never delivered consumes its trigger frame on
/// fire 1, collapses at the context read, and the refill then gives it fire 2. If
/// `Empty` were consumed by its serve, fire 2 would live-drain — so a context
/// frame landing mid-step would be observed by fire 2 and not by fire 1, and the
/// fires of ONE step would disagree about whether the context exists at all.
/// Pinned at the subscriber, where the state is reachable directly.
#[test]
#[serial]
fn a_frozen_empty_is_re_served_and_never_falls_through_to_a_live_read() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("serve_many_empty");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");
    settle();

    // Capture on an empty queue: nothing has EVER been delivered, so the slot
    // is `Empty` and `held_sample` is None (the pre-first-delivery WAIT).
    subscriber.snapshot_latest();
    let first = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("empty try_view must not error");
    assert_eq!(
        first, None,
        "the first read of an Empty slot serves Ok(None)"
    );

    // A sample lands mid-step.
    publish_x(&mut publisher, 5.0);
    settle();
    let second = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("the re-served Empty slot must not error");
    assert_eq!(
        second, None,
        "a SECOND read inside the same step must still observe Ok(None) — \
         every fire of one step agrees about whether the context exists. If \
         `Empty` were consumed by its serve this reads Some(5.0), and the fires \
         of one burst would disagree",
    );

    // Anti-tautology: the sample really was there to be read, so the None above
    // is the FREEZE and not an empty queue. The next capture serves it.
    subscriber.snapshot_latest();
    let after_capture = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("post-capture try_view")
        .expect("the new capture serves the sample that had landed");
    assert_eq!(
        after_capture, 5.0,
        "the mid-step sample is served by the NEXT capture — proving it was \
         queued and readable all along, got {after_capture}"
    );
}

// ---------------------------------------------------------------------------
// 9. expect_within anchor accounting runs exactly once.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn snapshot_writes_expect_within_anchor_exactly_once() {
    // R1 contract: snapshot_latest()'s drain writes the DELIVERED sample's wire
    // timestamp into the expect_within anchor EXACTLY ONCE (at snapshot time).
    // The following try_view serves the frozen slot and must NOT re-write the
    // anchor — a re-drain (the double-count bug) would store the wire ts again.
    // Deterministic: the wire ts is data (publisher-stamped), so no wall clock.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("expect_within_anchor");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Install a fresh anchor (0 = "no data seen yet").
    let anchor = Arc::new(AtomicU64::new(0));
    subscriber.register_expect_within_anchor_for_test(Arc::clone(&anchor));

    // One delivered frame: the publisher stamps a non-zero wire timestamp.
    publish_x(&mut publisher, 1.0);
    settle();

    // The snapshot drain delivers the sample → writes its wire ts into the
    // anchor (the expect_within accounting ran at SNAPSHOT time).
    subscriber.snapshot_latest();
    let after_snapshot = anchor.load(Ordering::Acquire);
    assert_ne!(
        after_snapshot, 0,
        "snapshot_latest must write the delivered sample's wire timestamp into \
         the expect_within anchor (still 0 → the anchor accounting did NOT run \
         at snapshot time)"
    );

    // The view serves the frozen slot WITHOUT re-draining — the anchor must be
    // UNCHANGED (a re-drain would re-store the wire ts → double accounting).
    let served = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("frozen try_view")
        .expect("frozen serves the snapshot sample");
    assert_eq!(
        served, 1.0,
        "the frozen view serves the snapshot sample (1.0)"
    );
    assert_eq!(
        anchor.load(Ordering::Acquire),
        after_snapshot,
        "the try_view after snapshot must NOT re-write the expect_within anchor \
         — it must be UNCHANGED from snapshot time ({after_snapshot}): try_view \
         serves the frozen slot via frozen.take() and never re-enters the drain, \
         so the anchor's write site is not reached again"
    );
}
