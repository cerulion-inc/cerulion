// SPDX-License-Identifier: AGPL-3.0-only
//! Tests for the QoS deadline knobs
//! (`expect_within_ms` / `promise_within_ms` / `tick_within_ms`) as
//! port-level + node-level QoS.
//!
//! Verifies:
//! 1. `Scheduler::set_expect_within` configures per-input deadline
//!    tracking; `step()` increments `expect_within_missed_count`
//!    when elapsed > deadline.
//! 2. `Scheduler::set_tick_within` configures per-node tick deadline;
//!    `fire_node` increments `tick_within_missed_count` when
//!    the tick callback's wall-clock duration exceeds the limit.
//! 3. `Scheduler::set_promise_within` + `signal_output_published`
//!    increments `promise_within_missed_count` (the publisher-side
//!    API surface).
//! 4. The three counters are independent — incrementing one doesn't
//!    affect the others.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::scheduler::{
    ExpectWithinEvent, LivelinessCause, LivelinessEvent, LivelinessState, NodeConfig,
    PromiseWithinEvent, Scheduler, TriggerPolicy,
};
use tracing_test::traced_test;

fn counting_callback() -> (Box<dyn FnMut() + Send>, Arc<AtomicU64>) {
    let count = Arc::new(AtomicU64::new(0));
    let count_clone = count.clone();
    let cb = Box::new(move || {
        count_clone.fetch_add(1, Ordering::Relaxed);
    });
    (cb, count)
}

#[test]
fn expect_within_miss_increments_counter() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));

    let (cb, _fires) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "consumer".to_string(),
            // Periodic policy so step() always evaluates — orthogonal
            // to the input-deadline check.
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(1_000_000),
                max_catchup: None,
            },
            callback: cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("consumer").unwrap();

    // Configure: expect data on `image` every 50 ms. the
    // window anchor is a caller-supplied shared `Arc<AtomicU64>` (the runtime
    // wires the same handle into the input's subscriber); init to the build
    // clock (t=0 here).
    scheduler
        .set_expect_within(
            "consumer",
            "image",
            50,
            Arc::new(AtomicU64::new(clock.now_ns())),
            false,
            None,
        )
        .unwrap();
    assert_eq!(
        handle.expect_within_missed_count(),
        0,
        "no misses at setup time"
    );

    // Step 30 ms — within the 50 ms window; no miss.
    scheduler.step(Duration::from_millis(30));
    assert_eq!(handle.expect_within_missed_count(), 0);

    // Step another 30 ms — total 60 ms without data; one miss expected.
    scheduler.step(Duration::from_millis(30));
    assert_eq!(
        handle.expect_within_missed_count(),
        1,
        "60 ms > 50 ms deadline should increment expect_within counter once"
    );

    // Signal fresh data — should reset the tracker.
    let now = clock.now_ns();
    scheduler
        .signal_input_received("consumer", "image", now)
        .unwrap();
    scheduler.step(Duration::from_millis(30));
    assert_eq!(
        handle.expect_within_missed_count(),
        1,
        "fresh data within window should NOT increment again"
    );

    // Step past the deadline again.
    scheduler.step(Duration::from_millis(40));
    assert_eq!(
        handle.expect_within_missed_count(),
        2,
        "another 70 ms without data should add a second miss"
    );
}

#[test]
fn tick_within_miss_increments_counter() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));

    let slow_cb: Box<dyn FnMut() + Send> = Box::new(|| {
        // 5 ms sleep — far exceeds the 1 ms deadline below.
        std::thread::sleep(Duration::from_millis(5));
    });
    scheduler
        .add_node(NodeConfig {
            id: "slow".to_string(),
            policy: TriggerPolicy::External,
            callback: slow_cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("slow").unwrap();

    scheduler.set_tick_within("slow", 1).unwrap();

    scheduler.trigger_external("slow").unwrap();
    scheduler.step(Duration::from_millis(1));

    assert_eq!(
        handle.tick_within_missed_count(),
        1,
        "5 ms tick > 1 ms deadline should increment tick_within counter"
    );
    assert_eq!(handle.fire_count(), 1, "node fired once");
}

#[test]
fn tick_within_not_exceeded_no_miss() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));

    let (cb, _fires) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fast".to_string(),
            policy: TriggerPolicy::External,
            callback: cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("fast").unwrap();

    // 100 ms deadline; counting_callback returns immediately.
    scheduler.set_tick_within("fast", 100).unwrap();
    scheduler.trigger_external("fast").unwrap();
    scheduler.step(Duration::from_millis(1));

    assert_eq!(handle.tick_within_missed_count(), 0);
    assert_eq!(handle.fire_count(), 1);
}

#[test]
fn promise_within_miss_counter_increments_on_repeated_misses() {
    // Production-path test: each step past the deadline (without a
    // publish) bumps the counter exactly once + resets the tracker.
    // Two missed windows → counter = 2.
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));

    let (cb, _fires) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "publisher".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(1_000_000),
                max_catchup: None,
            },
            callback: cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("publisher").unwrap();
    scheduler
        .set_promise_within(
            "publisher",
            "cmd_vel",
            50,
            Arc::new(AtomicU64::new(clock.now_ns())),
        )
        .unwrap();

    assert_eq!(handle.promise_within_missed_count(), 0);
    scheduler.step(Duration::from_millis(60));
    assert_eq!(handle.promise_within_missed_count(), 1);
    // Tracker reset to t=60ms. Stepping another 60ms (no publish) →
    // elapsed 60ms > 50ms → second miss.
    scheduler.step(Duration::from_millis(60));
    assert_eq!(handle.promise_within_missed_count(), 2);
}

// Verify the three miss paths emit
// `tracing::warn!` with structured fields so operators see the
// violation, not just a silent counter bump. Without these logs the
// observable counter is invisible until something queries it.

#[test]
#[traced_test]
fn expect_within_miss_emits_structured_warn() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));

    let (cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "n".to_string(),
            policy: TriggerPolicy::External,
            callback: cb,
        })
        .unwrap();
    scheduler
        .set_expect_within(
            "n",
            "image",
            50,
            Arc::new(AtomicU64::new(clock.now_ns())),
            false,
            None,
        )
        .unwrap();
    scheduler.step(Duration::from_millis(60));

    assert!(
        logs_contain("no fresh data within the expected window"),
        "expect_within_ms miss must emit a tracing::warn!"
    );
    assert!(
        logs_contain("node_id=n"),
        "warn must carry structured node_id field"
    );
    assert!(
        logs_contain("input=image"),
        "warn must carry structured input field"
    );
    assert!(
        logs_contain("expect_within_ms=50"),
        "warn must carry structured expect_within_ms field"
    );
}

#[test]
#[traced_test]
fn tick_within_miss_emits_structured_warn() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));

    let slow_cb: Box<dyn FnMut() + Send> = Box::new(|| {
        std::thread::sleep(Duration::from_millis(5));
    });
    scheduler
        .add_node(NodeConfig {
            id: "slow".to_string(),
            policy: TriggerPolicy::External,
            callback: slow_cb,
        })
        .unwrap();
    scheduler.set_tick_within("slow", 1).unwrap();
    scheduler.trigger_external("slow").unwrap();
    scheduler.step(Duration::from_millis(1));

    assert!(
        logs_contain("tick took longer than the execution budget"),
        "tick_within_ms miss must emit a tracing::warn!"
    );
    assert!(
        logs_contain("node_id=slow"),
        "warn must carry structured node_id field"
    );
    assert!(
        logs_contain("tick_within_ms=1"),
        "warn must carry structured tick_within_ms field"
    );
}

#[test]
#[traced_test]
fn promise_within_miss_emits_structured_warn_via_production_path() {
    // Exercises the PRODUCTION path end-to-end. Two scenarios pinned
    // back-to-back:
    //
    //   (a) Step past the deadline without publishing → miss + warn
    //       with structured fields (node_id / output / promise_within_ms /
    //       elapsed_ms).
    //   (b) Publish BEFORE the deadline would expire → tracker resets;
    //       subsequent step within the publish-window must NOT miss.
    //       The publish-call is structurally load-bearing: deleting
    //       it would let `step()` see elapsed > deadline and bump the
    //       counter, failing this test. (It also rules out a
    //       version where `signal_output_published` is effectively a
    //       no-op because the prior miss-and-reset has already
    //       advanced last_publish_ns to current_time.)
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));

    let (cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "publisher".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(1_000_000),
                max_catchup: None,
            },
            callback: cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("publisher").unwrap();

    // Configure: commit to publishing on `cmd_vel` every 50 ms.
    scheduler
        .set_promise_within(
            "publisher",
            "cmd_vel",
            50,
            Arc::new(AtomicU64::new(clock.now_ns())),
        )
        .unwrap();
    assert_eq!(handle.promise_within_missed_count(), 0);

    // (a) Step past the deadline without publishing → miss + warn.
    scheduler.step(Duration::from_millis(60));
    assert_eq!(
        handle.promise_within_missed_count(),
        1,
        "60 ms > 50 ms deadline without publish should increment promise_within counter"
    );
    assert!(
        logs_contain("no publish within the promised window"),
        "promise_within_ms miss must emit a tracing::warn!"
    );
    assert!(
        logs_contain("node_id=publisher"),
        "warn must carry structured node_id field"
    );
    assert!(
        logs_contain("output=cmd_vel"),
        "warn must carry structured output field"
    );
    assert!(
        logs_contain("promise_within_ms=50"),
        "warn must carry structured promise_within_ms field"
    );

    // (b) Publish-reset path: structured so that the publish call is
    // load-bearing. After the step(60ms) above, last_publish_ns is
    // at t=60ms (miss-and-reset). Advance another 40 ms (no publish);
    // elapsed since reset = 40 ms < 50 ms → no new miss (counter == 1).
    // Then signal a publish; last_publish_ns moves to t=100ms.
    // Advance 40 ms more (t=140ms); elapsed since publish = 40 ms
    // < 50 ms → still no miss (counter == 1). If
    // `signal_output_published` were a no-op, the
    // 40+40 advance from t=60ms would be elapsed = 80 ms > 50 ms
    // and the counter would jump to 2 — this test would fail
    // loudly. The intermediate `step(40)` ensures we cross the
    // 50-ms boundary measured from t=60ms, so the publish-reset
    // is the only thing keeping the counter at 1.
    scheduler.step(Duration::from_millis(40));
    assert_eq!(
        handle.promise_within_missed_count(),
        1,
        "elapsed 40 ms since reset must not miss yet"
    );
    scheduler
        .signal_output_published("publisher", "cmd_vel")
        .unwrap();
    scheduler.step(Duration::from_millis(40));
    assert_eq!(
        handle.promise_within_missed_count(),
        1,
        "publish-reset must keep elapsed at 40 ms (since publish at 100 ms), \
         not 80 ms (since reset at 60 ms) — counter must stay at 1. \
         If this assertion fails with count == 2, `signal_output_published` \
         did not reset the tracker."
    );
}

#[test]
fn three_qos_within_counters_are_independent() {
    // Trigger ONLY the promise_within counter via the production path
    // (set_promise_within + step past deadline). Verify the other
    // two counters stay at 0 — they're independent observation
    // surfaces and must not bleed into each other.
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));

    let (cb, _fires) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "n".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(1_000_000),
                max_catchup: None,
            },
            callback: cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("n").unwrap();
    scheduler
        .set_promise_within("n", "out", 50, Arc::new(AtomicU64::new(clock.now_ns())))
        .unwrap();
    scheduler.step(Duration::from_millis(60));

    assert_eq!(handle.promise_within_missed_count(), 1);
    assert_eq!(handle.expect_within_missed_count(), 0);
    assert_eq!(handle.tick_within_missed_count(), 0);
}

// ─── c2: reactable QoS watchdog EVENT surface ─────────────
//
// The edge-triggered `ExpectWithinEvent` / `PromiseWithinEvent` fire ONCE
// per silence regime (the first miss after data goes quiet) and rearm on a
// real arrival/publish. The COUNTER cadence is unchanged — every missed
// window still bumps the counter — so these tests assert the EVENT is
// strictly edge-gated relative to the counter.

#[test]
fn expect_within_event_fires_once_per_regime_and_rearms() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));

    let (cb, _fires) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            // Periodic-but-never-fires policy so step() always runs the
            // watchdog without the node's trigger interfering.
            id: "n".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(1_000_000),
                max_catchup: None,
            },
            callback: cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("n").unwrap();
    scheduler.install_qos_event_store_for_test("n").unwrap();
    scheduler
        .set_expect_within(
            "n",
            "image",
            50,
            Arc::new(AtomicU64::new(clock.now_ns())),
            false,
            None,
        )
        .unwrap();

    // Inside the 50 ms window: no miss, no event.
    scheduler.step(Duration::from_millis(30)); // t=30 ms
    assert_eq!(handle.expect_within_missed_count(), 0);
    assert!(scheduler.take_expect_within_event("n", "image").is_none());

    // Cross the window: regime-1 first miss → counter 1 AND event 1.
    scheduler.step(Duration::from_millis(30)); // t=60 ms, elapsed 60 > 50
    assert_eq!(handle.expect_within_missed_count(), 1);
    let ev = scheduler
        .take_expect_within_event("n", "image")
        .expect("the first miss of a silence regime fires an event");
    assert_eq!(&*ev.input_name, "image");
    assert_eq!(ev.expect_within_ms, 50);
    assert_eq!(ev.elapsed_ms, 60);
    assert_eq!(ev.count_total, 1);
    assert_eq!(
        ev.missed_at_ns, 60_000_000,
        "missed_at_ns is the step clock"
    );
    // Drained — taking again yields None.
    assert!(scheduler.take_expect_within_event("n", "image").is_none());

    // Still in regime 1 (no arrival): the counter bumps again but the
    // EVENT is edge-gated — no new event fires.
    scheduler.step(Duration::from_millis(60)); // t=120 ms, elapsed 60 > 50
    assert_eq!(handle.expect_within_missed_count(), 2);
    assert!(
        scheduler.take_expect_within_event("n", "image").is_none(),
        "a second miss in the SAME silence regime must NOT fire a new event"
    );

    // A real arrival (wire ts t=119 ms — distinct from the scheduler's last
    // miss-write at t=120, modelling data published just before observation)
    // rearms the latch.
    scheduler
        .signal_input_received("n", "image", 119_000_000)
        .unwrap();
    scheduler.step(Duration::from_millis(30)); // t=150 ms, elapsed since 119 = 31 < 50
    assert_eq!(handle.expect_within_missed_count(), 2);
    assert!(scheduler.take_expect_within_event("n", "image").is_none());

    // New silence regime crosses the window → the rearmed latch fires again.
    scheduler.step(Duration::from_millis(60)); // t=210 ms, elapsed since 119 = 91 > 50
    assert_eq!(handle.expect_within_missed_count(), 3);
    let ev2 = scheduler
        .take_expect_within_event("n", "image")
        .expect("a fresh arrival rearms the latch — the next regime fires again");
    assert_eq!(ev2.count_total, 3);
    assert_eq!(ev2.missed_at_ns, 210_000_000);
}

#[test]
fn promise_within_event_fires_once_per_regime_and_rearms() {
    // Output twin of the expect test. `signal_output_published` resets the
    // window using the scheduler clock (not a caller ts), so the publish is
    // separated from the prior miss-write by an intervening step to give it
    // a distinct timestamp (modelling time passing before the publish).
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));

    let (cb, _fires) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "p".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(1_000_000),
                max_catchup: None,
            },
            callback: cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("p").unwrap();
    scheduler.install_qos_event_store_for_test("p").unwrap();
    scheduler
        .set_promise_within("p", "out", 50, Arc::new(AtomicU64::new(clock.now_ns())))
        .unwrap();

    // Regime-1 first miss → counter 1 AND event 1.
    scheduler.step(Duration::from_millis(60)); // t=60 ms
    assert_eq!(handle.promise_within_missed_count(), 1);
    let ev = scheduler
        .take_promise_within_event("p", "out")
        .expect("first promise miss fires an event");
    assert_eq!(&*ev.output_name, "out");
    assert_eq!(ev.promise_within_ms, 50);
    assert_eq!(ev.elapsed_ms, 60);
    assert_eq!(ev.count_total, 1);
    assert_eq!(ev.missed_at_ns, 60_000_000);
    assert!(scheduler.take_promise_within_event("p", "out").is_none());

    // Same regime: counter bumps, event edge-gated.
    scheduler.step(Duration::from_millis(60)); // t=120 ms
    assert_eq!(handle.promise_within_missed_count(), 2);
    assert!(
        scheduler.take_promise_within_event("p", "out").is_none(),
        "a second miss in the SAME regime must NOT fire a new promise event"
    );

    // Intervening step advances the clock so the publish lands at a distinct
    // ts (t=130) vs the last miss-write (t=120), then publish → rearm.
    scheduler.step(Duration::from_millis(10)); // t=130 ms, elapsed 10 < 50, no miss
    assert_eq!(handle.promise_within_missed_count(), 2);
    scheduler.signal_output_published("p", "out").unwrap(); // anchor = 130 ms
    scheduler.step(Duration::from_millis(30)); // t=160 ms, elapsed since 130 = 30 < 50
    assert_eq!(handle.promise_within_missed_count(), 2);
    assert!(scheduler.take_promise_within_event("p", "out").is_none());

    // New silence regime → rearmed event fires.
    scheduler.step(Duration::from_millis(60)); // t=220 ms, elapsed since 130 = 90 > 50
    assert_eq!(handle.promise_within_missed_count(), 3);
    let ev2 = scheduler
        .take_promise_within_event("p", "out")
        .expect("a publish rearms the latch — the next regime fires again");
    assert_eq!(ev2.count_total, 3);
    assert_eq!(ev2.missed_at_ns, 220_000_000);
}

#[test]
fn qos_events_are_deterministic_across_runs() {
    // Principle #7: the same step sequence yields a bit-identical sequence
    // of (fired?, count_total, missed_at_ns) — no wall-clock reads leak in.
    fn run() -> (
        Vec<Option<ExpectWithinEvent>>,
        Vec<Option<PromiseWithinEvent>>,
    ) {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));
        let (cb, _fires) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "n".to_string(),
                policy: TriggerPolicy::Period {
                    interval: Duration::from_millis(1_000_000),
                    max_catchup: None,
                },
                callback: cb,
            })
            .unwrap();
        scheduler.install_qos_event_store_for_test("n").unwrap();
        scheduler
            .set_expect_within(
                "n",
                "image",
                50,
                Arc::new(AtomicU64::new(clock.now_ns())),
                false,
                None,
            )
            .unwrap();
        scheduler
            .set_promise_within("n", "out", 50, Arc::new(AtomicU64::new(clock.now_ns())))
            .unwrap();
        let mut expects = Vec::new();
        let mut promises = Vec::new();
        for _ in 0..8 {
            scheduler.step(Duration::from_millis(40));
            expects.push(scheduler.take_expect_within_event("n", "image"));
            promises.push(scheduler.take_promise_within_event("n", "out"));
        }
        (expects, promises)
    }
    let (e1, p1) = run();
    let (e2, p2) = run();
    assert_eq!(e1, e2, "expect_within event sequence must be deterministic");
    assert_eq!(
        p1, p2,
        "promise_within event sequence must be deterministic"
    );
    // Sanity: at least one event actually fired (the run isn't vacuously
    // all-None).
    assert!(
        e1.iter().any(Option::is_some),
        "the deterministic run must have fired at least one expect event"
    );
    // Mirror the non-vacuity guard on the promise side: without it a
    // regression that made the promise edge-trigger never push would let
    // the `assert_eq!(p1, p2)` above pass VACUOUSLY (both runs all-None).
    assert!(
        p1.iter().any(Option::is_some),
        "the deterministic run must have fired at least one promise event"
    );
}

#[test]
fn tick_within_miss_emits_no_qos_event() {
    // LOCKED decision: `tick_within_ms` stays counter-only — no reactable
    // event (it keys off wall-clock `Instant`, which would break replay).
    // A blown tick budget bumps the counter but pushes NOTHING to the QoS
    // event store.
    //
    // Non-vacuity: we ALSO register a real `expect_within` input + a real
    // `promise_within` output with LONG (never-missed) windows, so the
    // store has live, drainable slots for those ports. Draining them after
    // a tick miss must still be `None` — i.e. the tick path does not
    // spuriously push onto a real QoS port. (That the store DOES capture
    // expect/promise events is proven by the `*_fires_once_per_regime_*`
    // tests; here those windows deliberately stay quiet.)
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));
    let slow_cb: Box<dyn FnMut() + Send> = Box::new(|| {
        std::thread::sleep(Duration::from_millis(5));
    });
    scheduler
        .add_node(NodeConfig {
            id: "slow".to_string(),
            policy: TriggerPolicy::External,
            callback: slow_cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("slow").unwrap();
    scheduler.install_qos_event_store_for_test("slow").unwrap();
    scheduler.set_tick_within("slow", 1).unwrap();
    // Long windows (10 s) so neither QoS port misses during the 1 ms step —
    // any event in their store slots could only come from the tick path.
    scheduler
        .set_expect_within(
            "slow",
            "img",
            10_000,
            Arc::new(AtomicU64::new(clock.now_ns())),
            false,
            None,
        )
        .unwrap();
    scheduler
        .set_promise_within(
            "slow",
            "cmd",
            10_000,
            Arc::new(AtomicU64::new(clock.now_ns())),
        )
        .unwrap();
    scheduler.trigger_external("slow").unwrap();
    scheduler.step(Duration::from_millis(1));

    assert_eq!(
        handle.tick_within_missed_count(),
        1,
        "the slow tick must blow the 1 ms budget"
    );
    assert_eq!(
        handle.expect_within_missed_count(),
        0,
        "the 10 s expect window must not miss in a 1 ms step"
    );
    assert_eq!(
        handle.promise_within_missed_count(),
        0,
        "the 10 s promise window must not miss in a 1 ms step"
    );
    assert!(
        scheduler.take_expect_within_event("slow", "img").is_none(),
        "tick_within is counter-only — a tick miss must not push an event onto \
         the real (registered, quiet) expect port"
    );
    assert!(
        scheduler.take_promise_within_event("slow", "cmd").is_none(),
        "tick_within is counter-only — a tick miss must not push an event onto \
         the real (registered, quiet) promise port"
    );
}

#[test]
fn expect_within_arrival_at_prior_miss_ts_still_rearms() {
    // The arrival-only-anchor design ELIMINATES the
    // equal-timestamp missed-rearm edge: because the scheduler NEVER
    // writes the shared anchor, an arrival whose wire-ts happens to equal a
    // value a miss-writing scheduler would author on a miss is still an
    // unambiguous real arrival (the anchor's only writers are arrivals), so
    // the latch rearms and the next regime fires. This test drives exactly
    // that would-be aliasing collision and asserts the rearm WORKS.
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));
    let (cb, _fires) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "n".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(1_000_000),
                max_catchup: None,
            },
            callback: cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("n").unwrap();
    scheduler.install_qos_event_store_for_test("n").unwrap();
    scheduler
        .set_expect_within(
            "n",
            "image",
            50,
            Arc::new(AtomicU64::new(clock.now_ns())),
            false,
            None,
        )
        .unwrap();

    // Regime 1: first miss → event 1, then a same-regime miss at t=120 ms
    // (the timestamp the OLD scheduler would have authored into the anchor).
    scheduler.step(Duration::from_millis(60)); // t=60: miss 1, event 1
    assert!(scheduler.take_expect_within_event("n", "image").is_some());
    scheduler.step(Duration::from_millis(60)); // t=120: miss 2, edge-gated
    assert_eq!(handle.expect_within_missed_count(), 2);
    assert!(scheduler.take_expect_within_event("n", "image").is_none());

    // The would-be aliasing arrival: wire-ts EXACTLY t=120 ms. Under a
    // miss-writing anchor this collides with the scheduler's miss-write and
    // suppresses the rearm; under the arrival-only anchor it is detected as a real arrival.
    scheduler
        .signal_input_received("n", "image", 120_000_000)
        .unwrap();

    // New silence regime crosses the window → the REARMED latch fires again.
    scheduler.step(Duration::from_millis(60)); // t=180: miss 3
    assert_eq!(
        handle.expect_within_missed_count(),
        3,
        "the counter advances every window regardless"
    );
    let ev = scheduler.take_expect_within_event("n", "image").expect(
        "an arrival at the prior miss timestamp must STILL rearm the latch \
         (arrival-only anchor → no aliasing) — the next regime fires an event",
    );
    assert_eq!(ev.count_total, 3);
    assert_eq!(ev.missed_at_ns, 180_000_000);
}

// ─── Scheduler-direct LivelinessEvent store seam ──────────
//
// Mirrors the expect/promise scheduler-direct pattern above (add_node +
// install_qos_event_store_for_test + take_*), but routes THROUGH the
// liveliness seams `push_liveliness_event_for_test` + `take_liveliness_event`
// (the latter is otherwise unexercised). Pins two store-level contracts the
// e2e `#[on_event]` test can't isolate:
//
//   (1) drain-once-then-None — a single push surfaces once with the FULL
//       payload asserted, and a second drain yields None (once-per-edge /
//       quiet-after-drain).
//   (2) newest-wins overwrite — pushing a second event for the SAME input
//       before draining surfaces the SECOND (Alive) event, proving the slot
//       is newest-wins, not first-wins / append. This also exercises the
//       Alive / PublisherConnected edge (the e2e test only injects Lost).
//
// The two `changed_at_ns` constants (A != B) are picked so a wrong-field
// mutation (e.g. surfacing the stale Lost event, or reading the wrong field)
// fails visibly.
#[test]
fn liveliness_store_drains_once_and_is_newest_wins() {
    const A_LOST_NS: u64 = 111_000_111;
    const B_ALIVE_NS: u64 = 222_000_222;

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));

    let (cb, _fires) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "live_node".to_string(),
            policy: TriggerPolicy::External,
            callback: cb,
        })
        .unwrap();
    scheduler
        .install_qos_event_store_for_test("live_node")
        .unwrap();

    // (1) drain-once-then-None: push a Lost event, drain it with FULL payload
    // assertions, then prove the slot is empty.
    scheduler.push_liveliness_event_for_test(
        "live_node",
        LivelinessEvent::new_for_test(
            Arc::from("link"),
            LivelinessState::Lost,
            LivelinessCause::PublisherDisconnected,
            1,
            A_LOST_NS,
            0,
        ),
    );
    let lost = scheduler
        .take_liveliness_event("live_node", "link")
        .expect("the pushed Lost liveliness event must drain through the scheduler seam");
    assert_eq!(lost.input_name.as_ref(), "link");
    assert_eq!(lost.state, LivelinessState::Lost);
    assert_eq!(lost.cause, LivelinessCause::PublisherDisconnected);
    assert_eq!(lost.count_total, 1);
    assert_eq!(lost.changed_at_ns, A_LOST_NS);
    assert_eq!(lost.publisher_count, 0);
    assert!(
        scheduler
            .take_liveliness_event("live_node", "link")
            .is_none(),
        "a second drain must yield None — the slot is once-per-edge / quiet-after-drain"
    );

    // (2) newest-wins overwrite (also exercises Alive / PublisherConnected):
    // push a Lost then, WITHOUT draining, push an Alive for the SAME input. A
    // single drain must surface the SECOND (Alive) event — proving newest-wins,
    // not first-wins / append.
    scheduler.push_liveliness_event_for_test(
        "live_node",
        LivelinessEvent::new_for_test(
            Arc::from("link"),
            LivelinessState::Lost,
            LivelinessCause::PublisherDisconnected,
            1,
            A_LOST_NS,
            0,
        ),
    );
    scheduler.push_liveliness_event_for_test(
        "live_node",
        LivelinessEvent::new_for_test(
            Arc::from("link"),
            LivelinessState::Alive,
            LivelinessCause::PublisherConnected,
            2,
            B_ALIVE_NS,
            1,
        ),
    );
    let surfaced = scheduler
        .take_liveliness_event("live_node", "link")
        .expect("the overwritten slot must still surface an event");
    assert_eq!(
        surfaced.state,
        LivelinessState::Alive,
        "newest-wins: the second (Alive) event must surface, not the stale Lost"
    );
    assert_eq!(surfaced.cause, LivelinessCause::PublisherConnected);
    assert_eq!(surfaced.count_total, 2);
    assert_eq!(
        surfaced.changed_at_ns, B_ALIVE_NS,
        "newest-wins: a wrong-field/first-wins mutation would surface A_LOST_NS here"
    );
    assert_eq!(surfaced.publisher_count, 1);
    assert!(
        scheduler
            .take_liveliness_event("live_node", "link")
            .is_none(),
        "after the single newest-wins drain the slot is empty again"
    );
}

// ===========================================================================
// A lapsed window on a BACKLOGGED FIFO trigger input is not a miss
// ===========================================================================

/// The scoping oracle, at the scheduler seam where it is sharpest: a Data node
/// with TWO watched inputs, only one of which is its per-message FIFO trigger.
/// One `signal_data` gives the node a backlog of exactly ONE arrival, and a
/// single 60 ms step lapses BOTH 50 ms windows.
///
/// Hand oracle: the marked input's window is BACKLOG (`backlogged == 1`), the
/// unmarked one's is genuine silence (`missed == 1`). The node-level pending
/// count is shared by both inputs, so a guard that dropped the `fifo_trigger`
/// conjunct would suppress the unmarked input too and drive `missed` to 0.
///
/// This is also the ONLY arm that can see an off-by-one in the backlog
/// predicate: the backlog here is exactly 1, so a guard written as
/// `pending_data_count > 1` counts the marked input as silent and yields
/// `missed == 2` / `backlogged == 0`. Every end-to-end shape runs with a
/// backlog of tens, where `> 0` and `> 1` are indistinguishable.
///
/// `run_qos_windows` runs BEFORE `decide_node` in the same step, so the
/// arrival this step will consume is still pending at the QoS check — which
/// is what makes a single step enough and keeps the arm independent of how
/// many pending arrivals one step consumes.
#[test]
fn backlog_suppresses_only_the_marked_fifo_trigger_input() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));
    let (cb, _fires) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "n".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("n").unwrap();
    scheduler
        .set_expect_within(
            "n",
            "trig",
            50,
            Arc::new(AtomicU64::new(clock.now_ns())),
            true,
            None,
        )
        .unwrap();
    scheduler
        .set_expect_within(
            "n",
            "ctx",
            50,
            Arc::new(AtomicU64::new(clock.now_ns())),
            false,
            None,
        )
        .unwrap();

    scheduler.signal_data("n").unwrap();
    assert_eq!(
        handle.pending_data_count(),
        1,
        "precondition: exactly ONE pending arrival, so the off-by-one above is \
         observable"
    );
    scheduler.step(Duration::from_millis(60));

    assert_eq!(
        handle.expect_within_missed_count(),
        1,
        "the UNMARKED input's window lapsed with no data of its own — that is \
         silence and must still count a miss"
    );
    assert_eq!(
        handle.expect_within_backlogged_count(),
        1,
        "the MARKED FIFO trigger input's window lapsed while its own arrival \
         was still pending — that is backlog, not silence"
    );
}

/// A suppressed window advances the window start exactly as a counted miss
/// does, so the report stays ONE PER WINDOW instead of degenerating into one
/// per step — the flood the whole change exists to remove.
///
/// The backlog is held open by a pre-fire check that always defers, so
/// nothing is ever consumed and the arm does not depend on how many pending
/// arrivals a step takes. 30 steps of 25 ms against a 50 ms window: a window
/// lapses only once the clock is more than 50 ms past the last advance, i.e.
/// every THIRD step (75 ms, 150 ms, … 750 ms) — ten times.
///
/// Without the advance the branch stays true from step 3 onward and the count
/// is 28, so the oracle separates the two by 18.
#[test]
fn a_suppressed_window_advances_once_per_window_not_once_per_step() {
    const WINDOWS: u64 = 10;
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));
    let (cb, fires) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "n".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();
    let handle = scheduler.node_handle("n").unwrap();
    // Always defer: `decide_node` returns before the policy match, so the
    // pending arrival is never consumed and the backlog stays open for the
    // whole sweep.
    scheduler.set_pre_fire_check("n", |_| true).unwrap();
    scheduler
        .set_expect_within(
            "n",
            "trig",
            50,
            Arc::new(AtomicU64::new(clock.now_ns())),
            true,
            None,
        )
        .unwrap();
    scheduler.signal_data("n").unwrap();

    for _ in 0..(3 * WINDOWS) {
        scheduler.step(Duration::from_millis(25));
    }

    assert_eq!(
        handle.expect_within_backlogged_count(),
        WINDOWS,
        "a suppressed window must advance the window start, so the backlog is \
         reported once per WINDOW (10), not once per step (28)"
    );
    assert_eq!(
        handle.expect_within_missed_count(),
        0,
        "every lapse here happened with the arrival still pending — none may \
         be counted as a miss"
    );
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "anti-vacuity: the pre-fire check really did defer every step, so the \
         backlog really was open throughout"
    );
    assert_eq!(
        handle.pending_data_count(),
        1,
        "and the pending arrival was never consumed"
    );
}
