// SPDX-License-Identifier: AGPL-3.0-only
//! Scheduler Tests
//!
//! All tests use `VirtualClock` for deterministic assertions.
//! No iceoryx2 services needed — the scheduler operates on callbacks, not transport.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::error::TransportError;
use cerulion_core::scheduler::{NodeConfig, Scheduler, TraceEntry, TriggerPolicy};
use serial_test::serial;
use tracing_test::traced_test;

/// Helper: create a callback that increments a counter.
fn counting_callback() -> (Box<dyn FnMut() + Send>, Arc<AtomicU64>) {
    let count = Arc::new(AtomicU64::new(0));
    let count_clone = count.clone();
    let cb = Box::new(move || {
        count_clone.fetch_add(1, Ordering::Relaxed);
    });
    (cb, count)
}

/// Helper: create a Period policy with default (unlimited) catch-up.
fn period(interval_ms: u64) -> TriggerPolicy {
    TriggerPolicy::Period {
        interval: Duration::from_millis(interval_ms),
        max_catchup: None,
    }
}

// ---------------------------------------------------------------------------
// 4.0: Dead code audit (run separately via script)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 4.1: Period trigger with exact timing
// ---------------------------------------------------------------------------
#[test]
fn test_period_trigger_exact_timing() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    let _handle = scheduler
        .add_node(NodeConfig {
            id: "camera".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // No time passed — no fires
    assert_eq!(count.load(Ordering::Relaxed), 0);

    // Step exactly 10ms — fires once
    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 1);

    // Step 5ms — no fire (only 5ms since last)
    scheduler.step_ms(5);
    assert_eq!(count.load(Ordering::Relaxed), 1);

    // Step 5ms more — fires again (now 10ms since last fire)
    scheduler.step_ms(5);
    assert_eq!(count.load(Ordering::Relaxed), 2);
}

// ---------------------------------------------------------------------------
// 4.2: Multiple nodes fire in deterministic insertion order
// ---------------------------------------------------------------------------
#[test]
fn test_multiple_nodes_deterministic_order() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    // Add three nodes with same period
    for name in &["alpha", "beta", "gamma"] {
        let (cb, _) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: name.to_string(),
                policy: period(10),
                callback: cb,
            })
            .unwrap();
    }

    scheduler.step_ms(10);

    let trace = scheduler.trace();
    assert_eq!(trace.len(), 3);
    assert_eq!(&*trace[0].node_id, "alpha");
    assert_eq!(&*trace[1].node_id, "beta");
    assert_eq!(&*trace[2].node_id, "gamma");
}

// ---------------------------------------------------------------------------
// 4.3: Different periods interleave correctly
// ---------------------------------------------------------------------------
#[test]
fn test_different_periods_interleave() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    // A fires every 5ms, B fires every 10ms
    let (cb_a, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "A".to_string(),
            policy: period(5),
            callback: cb_a,
        })
        .unwrap();

    let (cb_b, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "B".to_string(),
            policy: period(10),
            callback: cb_b,
        })
        .unwrap();

    // Step 10ms: A fires at 5ms and 10ms, B fires at 10ms
    scheduler.step_ms(10);

    let trace = scheduler.trace();
    assert_eq!(trace.len(), 3);
    // A fires first (insertion order), then B
    // A@5ms, A@10ms, B@10ms
    assert_eq!(
        trace[0],
        TraceEntry {
            node_id: Arc::from("A"),
            step: 0,
            fire_time_ns: 5_000_000,
            global_level: 0,
            duration_ns: 0,
            discarded: false,
        }
    );
    assert_eq!(
        trace[1],
        TraceEntry {
            node_id: Arc::from("A"),
            step: 0,
            fire_time_ns: 10_000_000,
            global_level: 0,
            duration_ns: 0,
            discarded: false,
        }
    );
    assert_eq!(
        trace[2],
        TraceEntry {
            node_id: Arc::from("B"),
            step: 0,
            fire_time_ns: 10_000_000,
            global_level: 0,
            duration_ns: 0,
            discarded: false,
        }
    );
}

// ---------------------------------------------------------------------------
// 4.4: Replay produces identical traces
// ---------------------------------------------------------------------------
#[test]
fn test_replay_identical_to_live() {
    fn run_scenario() -> Vec<TraceEntry> {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);

        let (cb_a, _) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "sensor".to_string(),
                policy: TriggerPolicy::Period {
                    interval: Duration::from_millis(10),
                    max_catchup: None,
                },
                callback: cb_a,
            })
            .unwrap();

        let (cb_b, _) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "processor".to_string(),
                policy: TriggerPolicy::Data,
                callback: cb_b,
            })
            .unwrap();

        // Simulate: sensor fires at 10ms, then data signal, then step 10ms more
        scheduler.step_ms(10);
        scheduler.signal_data("processor").unwrap();
        scheduler.step_ms(10);

        scheduler.trace().to_vec()
    }

    let trace1 = run_scenario();
    let trace2 = run_scenario();
    assert_eq!(trace1, trace2, "Replay must produce identical trace");
}

// ---------------------------------------------------------------------------
// 4.6: Period catch-up on large step
// ---------------------------------------------------------------------------
#[test]
fn test_period_catchup_on_large_step() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fast".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // Step 30ms — fires 3 times (at 10ms, 20ms, 30ms)
    scheduler.step_ms(30);
    assert_eq!(count.load(Ordering::Relaxed), 3);

    let trace = scheduler.trace();
    assert_eq!(trace[0].fire_time_ns, 10_000_000);
    assert_eq!(trace[1].fire_time_ns, 20_000_000);
    assert_eq!(trace[2].fire_time_ns, 30_000_000);
}

// ---------------------------------------------------------------------------
// 4.6b: the polled gating clock advances by a RUN-INDEPENDENT
// quantum (contract test for `ClockInner::advance` →
// `VirtualClock::advance_by_recorded`).
// ---------------------------------------------------------------------------
//
// Read this before "strengthening" this test:
// reverting the production change from `advance_by_recorded(delta_ns)` back to
// `advance(delta_ns)` in `ClockInner::advance` keeps this test GREEN BY DESIGN
// — the two `VirtualClock` methods are byte-identical (same `fetch_add` with
// `Release`). This test does NOT pin the rename; it pins the RUN-INDEPENDENCE
// CONTRACT: under a fixed polled `step_ms(10)` sequence the gating clock lands
// on the fixed-quantum oracle (10/20/30/40/50 ms), independent of the run. A
// future regression INSIDE the advance SEAM (`ClockInner::advance` /
// `VirtualClock::advance_by_recorded` transforming the delta by a run-DEPENDENT
// quantity — wall elapsed, max-of-peer durations) breaks the oracle below.
// SCOPE NOTE: the run-dependent *delta SOURCE* (a live caller deriving `delta`
// from wall elapsed) is OUT of scope here — `step_ms()` hard-codes the delta;
// that caller-side boundary is pinned by `polled_vs_live_iox2_test`. Do NOT
// rewrite this into something that fails on the byte-identical `advance()`
// revert — that would be pinning the mechanism, not the contract.
#[test]
fn test_polled_gating_clock_fire_time_run_independent() {
    fn run_scenario() -> Vec<(String, u64)> {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);

        let (cb, _) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "ticker".to_string(),
                policy: period(10),
                callback: cb,
            })
            .unwrap();

        // FIXED polled sequence: five 10ms ticks. The gating-clock delta per
        // step is the run-independent poll quantum, so fire_time lands on the
        // fixed-quantum oracle regardless of how (or how many times) we run.
        for _ in 0..5 {
            scheduler.step_ms(10);
        }

        scheduler
            .trace()
            .iter()
            .map(|e| (e.node_id.to_string(), e.fire_time_ns))
            .collect()
    }

    let run1 = run_scenario();
    let run2 = run_scenario();

    // (1) Run-independence: two runs of the fixed polled sequence agree.
    assert_eq!(
        run1, run2,
        "polled gating clock must produce a run-independent trace"
    );

    // (2) Non-tautological oracle: the gating clock lands on the FIXED
    // run-independent quantum (10/20/30/40/50 ms), not merely that two runs
    // agree. This is what would break if the gating clock were ever advanced
    // by a run-dependent (wall / max-of-peers) quantity.
    let oracle = vec![
        ("ticker".to_string(), 10_000_000),
        ("ticker".into(), 20_000_000),
        ("ticker".into(), 30_000_000),
        ("ticker".into(), 40_000_000),
        ("ticker".into(), 50_000_000),
    ];
    assert_eq!(
        run1, oracle,
        "fire_time must land on the fixed run-independent polled quantum"
    );
}

// ---------------------------------------------------------------------------
// 4.7: Observable state via NodeHandle
// ---------------------------------------------------------------------------
#[test]
fn test_state_observable() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    let handle = scheduler
        .add_node(NodeConfig {
            id: "obs".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    assert_eq!(handle.id(), "obs");
    assert_eq!(handle.fire_count(), 0);
    assert_eq!(handle.last_fire_ns(), 0);

    scheduler.step_ms(10);
    assert_eq!(handle.fire_count(), 1);
    assert_eq!(handle.last_fire_ns(), 10_000_000);

    scheduler.step_ms(10);
    assert_eq!(handle.fire_count(), 2);
    assert_eq!(handle.last_fire_ns(), 20_000_000);
}

// ---------------------------------------------------------------------------
// NodeHandle::fire_counter hands an observer thread a clone
// of the SAME cap-immune fire counter — it ALIASES step()'s bumps, is
// monotonic, and never resets. (The online profiler reads it to build the
// fires_target gate.)
// ---------------------------------------------------------------------------
#[test]
fn test_fire_counter_aliases_and_is_monotonic() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    let handle = scheduler
        .add_node(NodeConfig {
            id: "prof".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // The observer's clone reads the live count lock-free.
    let counter = handle.fire_counter();
    assert_eq!(counter.load(Ordering::Acquire), 0);

    // Each step bumps the SAME atomic the accessor reads — aliasing proof.
    scheduler.step_ms(10);
    assert_eq!(counter.load(Ordering::Acquire), 1);
    assert_eq!(
        counter.load(Ordering::Acquire),
        handle.fire_count(),
        "fire_counter() aliases fire_count() — same atomic"
    );

    // Monotonic: only ever climbs.
    scheduler.step_ms(10);
    scheduler.step_ms(10);
    assert_eq!(counter.load(Ordering::Acquire), 3);

    // A second clone observes the same value (shared Arc, not a snapshot).
    let counter2 = handle.fire_counter();
    assert_eq!(counter2.load(Ordering::Acquire), 3);
    assert!(
        Arc::ptr_eq(&counter, &counter2),
        "both clones share one atomic"
    );
}

// ---------------------------------------------------------------------------
// 4.8: Same inputs → same NodeHandle values (deterministic state)
// ---------------------------------------------------------------------------
#[test]
fn test_state_deterministic() {
    fn run_and_observe() -> (u64, u64) {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);

        let (cb, _) = counting_callback();
        let handle = scheduler
            .add_node(NodeConfig {
                id: "det".to_string(),
                policy: TriggerPolicy::Period {
                    interval: Duration::from_millis(10),
                    max_catchup: None,
                },
                callback: cb,
            })
            .unwrap();

        scheduler.step_ms(25);
        (handle.fire_count(), handle.last_fire_ns())
    }

    let (fc1, lf1) = run_and_observe();
    let (fc2, lf2) = run_and_observe();
    assert_eq!(fc1, fc2);
    assert_eq!(lf1, lf2);
}

// ---------------------------------------------------------------------------
// 4.9: FIFO ordering preserved across all trigger types
// ---------------------------------------------------------------------------
#[test]
fn test_fifo_ordering() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    // Add nodes with different trigger types in specific order
    let (cb1, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "period_node".to_string(),
            policy: period(10),
            callback: cb1,
        })
        .unwrap();

    let (cb2, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "data_node".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb2,
        })
        .unwrap();

    let (cb3, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "external_node".to_string(),
            policy: TriggerPolicy::External,
            callback: cb3,
        })
        .unwrap();

    // Signal all non-period nodes
    scheduler.signal_data("data_node").unwrap();
    scheduler.trigger_external("external_node").unwrap();

    scheduler.step_ms(10);

    let trace = scheduler.trace();
    assert_eq!(trace.len(), 3);
    // Insertion order: period, data, external
    assert_eq!(&*trace[0].node_id, "period_node");
    assert_eq!(&*trace[1].node_id, "data_node");
    assert_eq!(&*trace[2].node_id, "external_node");
}

// ---------------------------------------------------------------------------
// 4.10: Data trigger fires on signal
// ---------------------------------------------------------------------------
#[test]
fn test_data_trigger_fires() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "detector".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();

    // No signal → no fire
    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 0);

    // Signal data then step → fires
    scheduler.signal_data("detector").unwrap();
    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 1);
}

// ---------------------------------------------------------------------------
// 4.11: External trigger fires once
// ---------------------------------------------------------------------------
#[test]
fn test_external_trigger_fires() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "manual".to_string(),
            policy: TriggerPolicy::External,
            callback: cb,
        })
        .unwrap();

    // No trigger → no fire
    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 0);

    // Trigger then step → fires once
    scheduler.trigger_external("manual").unwrap();
    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 1);

    // Next step without trigger → no fire
    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 1);
}

// ---------------------------------------------------------------------------
// 4.12: Sync trigger waits for all inputs
// ---------------------------------------------------------------------------
#[test]
fn test_sync_trigger_waits_for_all_inputs() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["camera".to_string(), "imu".to_string()],
                window: Some(Duration::from_millis(50)),
            },
            callback: cb,
        })
        .unwrap();

    // Only camera data → no fire
    scheduler
        .signal_sync_input("fusion", "camera", 10_000_000)
        .unwrap();
    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 0);

    // Add IMU data within window → fires
    scheduler
        .signal_sync_input("fusion", "imu", 15_000_000)
        .unwrap();
    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 1);
}

// ---------------------------------------------------------------------------
// 4.13: Sync trigger rejects stale timestamps
// ---------------------------------------------------------------------------
#[test]
fn test_sync_trigger_rejects_stale() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["camera".to_string(), "imu".to_string()],
                window: Some(Duration::from_millis(10)), // 10ms window
            },
            callback: cb,
        })
        .unwrap();

    // Camera at 0ms, IMU at 100ms — outside 10ms window
    scheduler.signal_sync_input("fusion", "camera", 0).unwrap();
    scheduler
        .signal_sync_input("fusion", "imu", 100_000_000)
        .unwrap();
    scheduler.step_ms(10);
    assert_eq!(
        count.load(Ordering::Relaxed),
        0,
        "stale timestamps should not trigger sync"
    );
}

// ---------------------------------------------------------------------------
// 4.13b: Unbounded sync — fires when all triggers have
// data regardless of timestamp spread; the bounded sync's stale-rejection
// path is intentionally absent.
// ---------------------------------------------------------------------------
#[test]
fn test_unbounded_sync_fires_regardless_of_spread() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["camera".to_string(), "imu".to_string()],
                window: None, // unbounded — no spread check
            },
            callback: cb,
        })
        .unwrap();

    // Camera at 0ms, IMU at 100ms — a bounded sync with 10ms window
    // would reject this (see test_sync_trigger_rejects_stale). Unbounded
    // sync fires because both inputs have unconsumed messages.
    scheduler.signal_sync_input("fusion", "camera", 0).unwrap();
    scheduler
        .signal_sync_input("fusion", "imu", 100_000_000)
        .unwrap();
    scheduler.step_ms(10);
    assert_eq!(
        count.load(Ordering::Relaxed),
        1,
        "unbounded sync should fire once when both triggers have data, \
         regardless of timestamp spread"
    );
}

#[test]
fn test_unbounded_sync_waits_for_all_inputs() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["camera".to_string(), "imu".to_string()],
                window: None,
            },
            callback: cb,
        })
        .unwrap();

    // Only one input → no fire (loose AND still requires all triggers
    // to have data; the difference vs bounded sync is only the spread
    // check, not the all-inputs check).
    scheduler
        .signal_sync_input("fusion", "camera", 10_000_000)
        .unwrap();
    scheduler.step_ms(10);
    assert_eq!(
        count.load(Ordering::Relaxed),
        0,
        "unbounded sync should NOT fire when only one trigger has data"
    );

    // Second input arrives → fires
    scheduler
        .signal_sync_input("fusion", "imu", 9_999_999_999)
        .unwrap(); // far future
    scheduler.step_ms(10);
    assert_eq!(
        count.load(Ordering::Relaxed),
        1,
        "unbounded sync fires once both inputs are present (no timing bound)"
    );
}

// ---------------------------------------------------------------------------
// 4.14: Data trigger doesn't double-fire
// ---------------------------------------------------------------------------
#[test]
fn test_data_trigger_no_double_fire() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "once".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();

    scheduler.signal_data("once").unwrap();

    // First step fires
    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 1);

    // Second step without new signal → no fire
    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 1);
}

// ---------------------------------------------------------------------------
// 4.17: Remove node — no longer fires
// ---------------------------------------------------------------------------
#[test]
fn test_remove_node() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "temp".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 1);

    scheduler.remove_node("temp").unwrap();
    assert_eq!(scheduler.node_count(), 0);

    scheduler.step_ms(10);
    assert_eq!(
        count.load(Ordering::Relaxed),
        1,
        "removed node should not fire"
    );
}

// ---------------------------------------------------------------------------
// 4.18: Duplicate node rejected
// ---------------------------------------------------------------------------
#[test]
fn test_duplicate_node_rejected() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb1, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "dup".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb1,
        })
        .unwrap();

    let (cb2, _) = counting_callback();
    let result = scheduler.add_node(NodeConfig {
        id: "dup".to_string(),
        policy: TriggerPolicy::Data,
        callback: cb2,
    });

    match result {
        Err(TransportError::DuplicateNode { node_id }) => assert_eq!(node_id, "dup"),
        _ => panic!("expected DuplicateNode error"),
    }
}

// ===========================================================================
// NEW TESTS: T1-T12 — edge cases and missing coverage
// ===========================================================================

// ---------------------------------------------------------------------------
// T1: Zero-interval Period is rejected
// ---------------------------------------------------------------------------
#[test]
fn test_zero_interval_period_rejected() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    let result = scheduler.add_node(NodeConfig {
        id: "bad".to_string(),
        policy: TriggerPolicy::Period {
            interval: Duration::ZERO,
            max_catchup: None,
        },
        callback: cb,
    });

    match result {
        Err(TransportError::SchedulerError { reason: msg }) => {
            assert!(
                msg.contains("interval must be > 0"),
                "unexpected error: {}",
                msg
            );
        }
        _ => panic!("expected SchedulerError for zero interval"),
    }
}

// ---------------------------------------------------------------------------
// T2: Zero-window Sync still fires when all timestamps are identical
// ---------------------------------------------------------------------------
#[test]
fn test_zero_window_sync_fires_on_identical_timestamps() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "sync_zero".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["a".to_string(), "b".to_string()],
                window: Some(Duration::ZERO),
            },
            callback: cb,
        })
        .unwrap();

    // Same timestamp for both → spread = 0 ≤ window(0) → fires
    scheduler
        .signal_sync_input("sync_zero", "a", 100_000)
        .unwrap();
    scheduler
        .signal_sync_input("sync_zero", "b", 100_000)
        .unwrap();
    scheduler.step_ms(1);
    assert_eq!(count.load(Ordering::Relaxed), 1);

    // Different timestamps → spread > 0, window = 0 → no fire
    scheduler
        .signal_sync_input("sync_zero", "a", 200_000)
        .unwrap();
    scheduler
        .signal_sync_input("sync_zero", "b", 200_001)
        .unwrap();
    scheduler.step_ms(1);
    assert_eq!(
        count.load(Ordering::Relaxed),
        1,
        "spread > 0 should not fire with zero window"
    );
}

// ---------------------------------------------------------------------------
// T3: Empty sync inputs is rejected
// ---------------------------------------------------------------------------
#[test]
fn test_empty_sync_inputs_rejected() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    let result = scheduler.add_node(NodeConfig {
        id: "bad_sync".to_string(),
        policy: TriggerPolicy::Sync {
            inputs: vec![],
            window: Some(Duration::from_millis(50)),
        },
        callback: cb,
    });

    match result {
        Err(TransportError::SchedulerError { reason: msg }) => {
            // The terminal empty-set error was reworded to name
            // the CAUSE (zero `#[input(trigger)]`-marked wired inputs — the
            // node could never fire) and the FIX (mark >=2 inputs trigger,
            // or pick a different policy), plus the offending node id.
            assert!(
                msg.contains("Sync trigger set is empty"),
                "error must name the cause; got: {}",
                msg
            );
            assert!(
                msg.contains("bad_sync"),
                "error must name the node; got: {}",
                msg
            );
            assert!(
                msg.contains("Mark >=2 wired inputs `#[input(trigger)]`"),
                "error must state the fix; got: {}",
                msg
            );
        }
        _ => panic!("expected SchedulerError for empty sync inputs"),
    }
}

// ---------------------------------------------------------------------------
// T4: signal_data() on wrong policy types is rejected
// ---------------------------------------------------------------------------
#[test]
fn test_signal_data_wrong_policy_rejected() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    // Period node
    let (cb1, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "period_node".to_string(),
            policy: period(10),
            callback: cb1,
        })
        .unwrap();

    // External node
    let (cb2, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "ext_node".to_string(),
            policy: TriggerPolicy::External,
            callback: cb2,
        })
        .unwrap();

    // Sync node
    let (cb3, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "sync_node".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["a".to_string()],
                window: Some(Duration::from_millis(10)),
            },
            callback: cb3,
        })
        .unwrap();

    // signal_data() should fail for all non-Data nodes
    assert!(scheduler.signal_data("period_node").is_err());
    assert!(scheduler.signal_data("ext_node").is_err());
    assert!(scheduler.signal_data("sync_node").is_err());
}

// ---------------------------------------------------------------------------
// T5: Sync invalid input name is rejected
// ---------------------------------------------------------------------------
#[test]
fn test_sync_invalid_input_name_rejected() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["camera".to_string(), "imu".to_string()],
                window: Some(Duration::from_millis(50)),
            },
            callback: cb,
        })
        .unwrap();

    // Valid input
    assert!(scheduler
        .signal_sync_input("fusion", "camera", 10_000_000)
        .is_ok());

    // Invalid input name — should be rejected
    let result = scheduler.signal_sync_input("fusion", "lidar", 10_000_000);
    match result {
        Err(TransportError::SchedulerError { reason: msg }) => {
            assert!(msg.contains("not declared"), "unexpected error: {}", msg);
        }
        _ => panic!("expected SchedulerError for invalid sync input name"),
    }
}

// ---------------------------------------------------------------------------
// T8: Period large catch-up is capped with max_catchup
// ---------------------------------------------------------------------------
#[test]
fn test_period_large_catchup_capped() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "capped".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(1),
                max_catchup: Some(5),
            },
            callback: cb,
        })
        .unwrap();

    // Step 1000ms with 1ms period → would be 1000 fires, capped to 5
    scheduler.step_ms(1000);
    assert_eq!(
        count.load(Ordering::Relaxed),
        5,
        "catch-up should be capped at max_catchup"
    );

    // Next step should resume from current time, not re-fire 995 missed ones
    scheduler.step_ms(1);
    assert_eq!(
        count.load(Ordering::Relaxed),
        6,
        "after cap, scheduler should resume normally"
    );
}

// ---------------------------------------------------------------------------
// T9: NodeHandle cross-thread read
// ---------------------------------------------------------------------------
#[test]
fn test_node_handle_cross_thread_read() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    let handle = scheduler
        .add_node(NodeConfig {
            id: "threaded".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    scheduler.step_ms(10);

    // Clone handle and read from another thread
    let handle_clone = handle.clone();
    let thread_result = std::thread::spawn(move || {
        (
            handle_clone.fire_count(),
            handle_clone.last_fire_ns(),
            handle_clone.id().to_string(),
        )
    })
    .join()
    .expect("thread should not panic");

    assert_eq!(thread_result.0, 1);
    assert_eq!(thread_result.1, 10_000_000);
    assert_eq!(thread_result.2, "threaded");
}

// ---------------------------------------------------------------------------
// T10: RealClock scheduler smoke test
// ---------------------------------------------------------------------------
#[test]
fn test_real_clock_scheduler_smoke() {
    let mut scheduler = Scheduler::new();

    let (cb, count) = counting_callback();
    let _handle = scheduler
        .add_node(NodeConfig {
            id: "real".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();

    // Signal data and step (RealClock reads wall time)
    scheduler.signal_data("real").unwrap();
    scheduler.step(Duration::from_millis(1));
    assert_eq!(count.load(Ordering::Relaxed), 1);
}

// ---------------------------------------------------------------------------
// T11: clear_trace()
// ---------------------------------------------------------------------------
#[test]
fn test_clear_trace() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "traced".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    scheduler.step_ms(10);
    assert_eq!(scheduler.trace().len(), 1);

    scheduler.clear_trace();
    assert_eq!(
        scheduler.trace().len(),
        0,
        "trace should be empty after clear"
    );

    scheduler.step_ms(10);
    assert_eq!(
        scheduler.trace().len(),
        1,
        "trace should accumulate after clear"
    );
}

// ---------------------------------------------------------------------------
// T12: node_handle("nonexistent") returns None
// ---------------------------------------------------------------------------
#[test]
fn test_node_handle_nonexistent_returns_none() {
    let clock = Arc::new(VirtualClock::new());
    let scheduler = Scheduler::with_virtual_clock(clock);

    assert!(
        scheduler.node_handle("does_not_exist").is_none(),
        "node_handle for nonexistent ID should return None"
    );
}

// ---------------------------------------------------------------------------
// T13: Circuit breaker disables node after 3 consecutive panics
// ---------------------------------------------------------------------------
#[test]
fn test_circuit_breaker_disables_after_panics() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let cb: Box<dyn FnMut() + Send> = Box::new(|| {
        panic!("intentional test panic");
    });
    let handle = scheduler
        .add_node(NodeConfig {
            id: "panicker".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // Fire 3 times → 3 panics → disabled
    scheduler.step_ms(10); // panic 1
    scheduler.step_ms(10); // panic 2
    scheduler.step_ms(10); // panic 3 → disabled

    assert_eq!(handle.panic_count(), 3);
    assert_eq!(handle.fire_count(), 3); // fire_count includes panicked fires

    // Step again — node should NOT fire (disabled)
    scheduler.step_ms(10);
    assert_eq!(handle.fire_count(), 3, "disabled node should not fire");
}

// ---------------------------------------------------------------------------
// Node-death ledger, SITE (a): the disable edge MINTS a node death
// ---------------------------------------------------------------------------

/// A bare `Scheduler` is the only place the circuit breaker's disable edge can
/// be driven at all — through a `GraphRuntime` the first panic poisons the node's
/// entry mutex, so `tick()` is never re-entered and `consecutive_panics` never
/// reaches three (see `cerulion_core::scheduler::node_death`). So this arm owns
/// the mint, and the runtime's poison arm is pinned separately.
#[test]
fn the_disable_edge_records_one_node_death() {
    use cerulion_core::scheduler::{NodeDeath, NodeDeathCause, NodeDeathLedger};

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let cb: Box<dyn FnMut() + Send> = Box::new(|| panic!("intentional test panic"));
    let handle = scheduler
        .add_node(NodeConfig {
            id: "panicker".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();
    let ledger = Arc::new(NodeDeathLedger::new());
    // ARMED, because an unarmed ledger records nothing — which is the replay
    // firewall, and is also what makes the "nothing yet" assertion below capable
    // of failing for the right reason rather than for that one.
    ledger.arm();
    scheduler
        .set_node_death_ledger_for_test("panicker", Arc::clone(&ledger))
        .unwrap();

    // ANTI-TAUTOLOGY: two panics is NOT a death. Without this, "exactly one
    // death" is satisfied by a mint on every panic that happens to be drained
    // once.
    scheduler.step_ms(10);
    scheduler.step_ms(10);
    assert_eq!(handle.panic_count(), 2);
    assert!(
        !ledger.has_pending(),
        "the breaker has not opened yet — nothing has died"
    );

    scheduler.step_ms(10); // panic 3 → disabled
    assert!(ledger.has_pending(), "the disable edge must record a death");

    // HAND oracle on the whole record: the SUBJECT is the node id (the regime
    // key a capture is latched on) and the cause is the breaker's, not the
    // runtime's poison arm.
    assert_eq!(
        ledger.take(),
        vec![NodeDeath {
            node_id: "panicker".to_string(),
            cause: NodeDeathCause::DisabledAfterPanics,
        }]
    );
    assert_eq!(ledger.dropped(), 0);
}

/// The guard is `!node.disabled`, so the mint is once per TRANSITION — and a
/// transition really can happen twice, because `reset_node` re-enables.
///
/// `TriggerKind::PanicDisable`'s doc USED to claim this "fires at most once per
/// node per run". It was corrected to name the TRANSITION instead; this arm is
/// what keeps that correction checkable rather than a comment.
#[test]
fn a_reset_and_three_more_panics_is_a_second_node_death() {
    use cerulion_core::scheduler::{NodeDeathCause, NodeDeathLedger};

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let cb: Box<dyn FnMut() + Send> = Box::new(|| panic!("intentional test panic"));
    scheduler
        .add_node(NodeConfig {
            id: "resettable".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();
    let ledger = Arc::new(NodeDeathLedger::new());
    ledger.arm();
    scheduler
        .set_node_death_ledger_for_test("resettable", Arc::clone(&ledger))
        .unwrap();

    for _ in 0..3 {
        scheduler.step_ms(10);
    }
    assert_eq!(ledger.take().len(), 1, "the first death");

    // A disabled node does not fire, so these steps cannot mint anything — which
    // is the half a `>=`-guarded mint would get wrong if any future path reached
    // `fire_node_into` on a disabled node.
    for _ in 0..5 {
        scheduler.step_ms(10);
    }
    assert!(
        !ledger.has_pending(),
        "a node that is already dead must not die again"
    );

    scheduler.reset_node("resettable").unwrap();
    for _ in 0..3 {
        scheduler.step_ms(10);
    }
    let second = ledger.take();
    assert_eq!(
        second.len(),
        1,
        "a reset node that dies again is a NEW transition, not a repeat"
    );
    assert_eq!(second[0].cause, NodeDeathCause::DisabledAfterPanics);
    assert_eq!(second[0].node_id, "resettable");
}

/// The replay firewall, at the SCHEDULER site: an unarmed ledger records nothing
/// even when the breaker genuinely opens.
///
/// This is the property that makes the polled `step()` seam safe. A replay drives
/// `step()`, `run_live` is the only thing that arms, so a node that panics during
/// a replay mints no capture request — not because nothing drains the ledger, but
/// because nothing writes it.
#[test]
fn an_unarmed_ledger_records_no_disable_edge() {
    use cerulion_core::scheduler::NodeDeathLedger;

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let cb: Box<dyn FnMut() + Send> = Box::new(|| panic!("intentional test panic"));
    let handle = scheduler
        .add_node(NodeConfig {
            id: "panicker".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();
    let ledger = Arc::new(NodeDeathLedger::new());
    // NOT armed.
    scheduler
        .set_node_death_ledger_for_test("panicker", Arc::clone(&ledger))
        .unwrap();

    for _ in 0..3 {
        scheduler.step_ms(10);
    }
    // The breaker really DID open — asserted, so "nothing recorded" cannot pass
    // because the stimulus failed.
    assert_eq!(handle.panic_count(), 3);
    scheduler.step_ms(10);
    assert_eq!(handle.fire_count(), 3, "the node really is disabled");

    assert!(!ledger.has_pending());
    assert!(ledger.take().is_empty());
}

// ---------------------------------------------------------------------------
// T13b: Circuit breaker reset via reset_node()
// ---------------------------------------------------------------------------
#[test]
fn test_circuit_breaker_reset() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let call_count = Arc::new(AtomicU64::new(0));
    let call_count_clone = call_count.clone();

    // Always panics
    let cb: Box<dyn FnMut() + Send> = Box::new(move || {
        call_count_clone.fetch_add(1, Ordering::Relaxed);
        panic!("intentional test panic");
    });
    let handle = scheduler
        .add_node(NodeConfig {
            id: "resettable".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // Fire 3 times → 3 panics → disabled
    scheduler.step_ms(10);
    scheduler.step_ms(10);
    scheduler.step_ms(10);
    assert_eq!(handle.panic_count(), 3);
    assert_eq!(handle.fire_count(), 3);

    // Node should NOT fire (disabled)
    scheduler.step_ms(10);
    assert_eq!(handle.fire_count(), 3, "disabled node should not fire");

    // Reset the circuit breaker
    scheduler.reset_node("resettable").unwrap();

    // Node should fire again (still panics, but it fires)
    scheduler.step_ms(10);
    assert_eq!(
        handle.fire_count(),
        4,
        "node should fire after circuit breaker reset"
    );
    assert_eq!(call_count.load(Ordering::Relaxed), 4);
}

// ---------------------------------------------------------------------------
// T14: Circuit breaker resets on successful callback
// ---------------------------------------------------------------------------
#[test]
fn test_circuit_breaker_resets_on_success() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let panic_count = Arc::new(AtomicU64::new(0));
    let panic_count_clone = panic_count.clone();

    // Panic on first 2 calls, succeed on 3rd
    let cb: Box<dyn FnMut() + Send> = Box::new(move || {
        let n = panic_count_clone.fetch_add(1, Ordering::Relaxed);
        if n < 2 {
            panic!("intentional panic #{}", n + 1);
        }
        // Success on call 3+
    });

    let handle = scheduler
        .add_node(NodeConfig {
            id: "recoverer".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    scheduler.step_ms(10); // panic 1, consecutive = 1
    scheduler.step_ms(10); // panic 2, consecutive = 2
    scheduler.step_ms(10); // success! consecutive resets to 0

    assert_eq!(handle.panic_count(), 2);
    assert_eq!(handle.fire_count(), 3);

    // Node should still be active (not disabled)
    scheduler.step_ms(10);
    assert_eq!(handle.fire_count(), 4);
}

// ---------------------------------------------------------------------------
// T14b: the Data burst stops at the panic circuit breaker
//
// `decide_node`'s `disabled` gate runs BEFORE the burst, so the burst loop is
// the ONE place a node can keep running after `fire_node_into` opened the
// breaker. Pre-FIFO a Data node fired once per step, so an always-panicking
// callback stopped after `MAX_CONSECUTIVE_PANICS` STEPS; a burst that never
// re-reads the flag runs the whole batch inside ONE step — `catch_unwind`
// cycles and trace entries alike, all of them after the breaker opened.
// ---------------------------------------------------------------------------
#[test]
fn a_data_burst_stops_at_the_panic_circuit_breaker() {
    // The scheduler's `MAX_CONSECUTIVE_PANICS`, which is private: this is a
    // HAND oracle, not a value read back out of the code under test.
    const PANIC_LIMIT: u64 = 3;
    // Comfortably past the limit, so a burst that ignores the flag is
    // unmistakable rather than off by one.
    const ARRIVALS: u64 = 8;

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let calls = Arc::new(AtomicU64::new(0));
    let calls_cb = Arc::clone(&calls);
    let cb: Box<dyn FnMut() + Send> = Box::new(move || {
        calls_cb.fetch_add(1, Ordering::Relaxed);
        panic!("intentional test panic");
    });
    let handle = scheduler
        .add_node(NodeConfig {
            id: "panicker".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();

    for _ in 0..ARRIVALS {
        scheduler.signal_data("panicker").unwrap();
    }
    scheduler.step_ms(1);

    assert_eq!(
        calls.load(Ordering::Relaxed),
        PANIC_LIMIT,
        "the burst must stop the instant the breaker opens — one that re-reads \
         nothing runs all {ARRIVALS} arrivals inside this single step"
    );
    assert_eq!(
        handle.panic_count(),
        PANIC_LIMIT,
        "one panic per fire, and no fire after the breaker opened"
    );
    assert_eq!(
        handle.fire_count(),
        PANIC_LIMIT,
        "fire_count records panicked fires too — so it is the burst's length"
    );
    let entries = scheduler
        .trace()
        .iter()
        .filter(|e| e.node_id.as_ref() == "panicker")
        .count() as u64;
    assert_eq!(
        entries, PANIC_LIMIT,
        "the trace carries one entry per fire, so a burst that ran on past the \
         breaker writes {ARRIVALS} entries for a node the breaker disabled"
    );

    // The node is DISABLED, not merely out of arrivals: the burst handed its
    // unfired remainder back to `pending_data_count`, and a further step must
    // still fire it zero times.
    scheduler.step_ms(1);
    assert_eq!(
        calls.load(Ordering::Relaxed),
        PANIC_LIMIT,
        "a disabled node does not fire again, carried backlog or not"
    );
    assert_eq!(handle.fire_count(), PANIC_LIMIT);
}

// ---------------------------------------------------------------------------
// T14c: the Period catch-up burst stops at the panic circuit breaker
//
// The Data burst and the Period catch-up are two loops with the SAME hole:
// `decide_node`'s `disabled` gate runs BEFORE either of them, so both can keep
// running a callback the breaker has already disabled. A `period_ms` node that
// fell behind (a slow step, a resumed run, a coarse `step_ms`) is handed a
// multi-fire burst and, without the break, an always-panicking one burns the WHOLE
// burst inside one step — `catch_unwind` cycles and trace entries alike.
//
// This drives `Scheduler::tick_node` (the serial path `step_ms` takes).
// `tick_node_into` — the COLLECT mirror the rayon level executor uses — carries
// the identical break; it is not reachable from this bare-`Scheduler` seam, so
// this arm does not claim to cover it.
// ---------------------------------------------------------------------------
#[test]
fn a_period_catchup_burst_stops_at_the_panic_circuit_breaker() {
    // The scheduler's `MAX_CONSECUTIVE_PANICS`, which is private: a HAND
    // oracle, not a value read back out of the code under test.
    const PANIC_LIMIT: u64 = 3;
    const INTERVAL_MS: u64 = 10;
    // 10 intervals fall due in ONE step — comfortably past the limit, so a
    // burst that ignores the flag is unmistakable rather than off by one.
    const CATCHUP_MS: u64 = 100;
    const BURST: u64 = CATCHUP_MS / INTERVAL_MS;

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let calls = Arc::new(AtomicU64::new(0));
    let calls_cb = Arc::clone(&calls);
    let cb: Box<dyn FnMut() + Send> = Box::new(move || {
        calls_cb.fetch_add(1, Ordering::Relaxed);
        panic!("intentional test panic");
    });
    let handle = scheduler
        .add_node(NodeConfig {
            id: "panicker".to_string(),
            // `period()` leaves `max_catchup` unbounded, so the whole backlog
            // is handed to ONE tick as a `fire_count`-of-10 burst.
            policy: period(INTERVAL_MS),
            callback: cb,
        })
        .unwrap();

    scheduler.step_ms(CATCHUP_MS);

    assert_eq!(
        calls.load(Ordering::Relaxed),
        PANIC_LIMIT,
        "the catch-up burst must stop the instant the breaker opens — one that \
         re-reads nothing runs all {BURST} due intervals inside this single step"
    );
    assert_eq!(
        handle.panic_count(),
        PANIC_LIMIT,
        "one panic per fire, and no fire after the breaker opened"
    );
    assert_eq!(
        handle.fire_count(),
        PANIC_LIMIT,
        "fire_count records panicked fires too — so it is the burst's length"
    );
    let entries = scheduler
        .trace()
        .iter()
        .filter(|e| e.node_id.as_ref() == "panicker")
        .count() as u64;
    assert_eq!(
        entries, PANIC_LIMIT,
        "the trace carries one entry per fire, so a burst that ran on past the \
         breaker writes {BURST} entries for a node the breaker disabled"
    );

    // The node is DISABLED, and it does NOT resume: the disabled break leaves
    // `next_fire_ns` where `decide_node` advanced it (past the whole burst),
    // which is `reset_node`'s documented no-catch-up contract.
    scheduler.step_ms(INTERVAL_MS);
    assert_eq!(
        calls.load(Ordering::Relaxed),
        PANIC_LIMIT,
        "a disabled node does not fire again, due intervals or not"
    );
    assert_eq!(handle.fire_count(), PANIC_LIMIT);
}

// ===========================================================================
// Bug fix tests: pending_data_count, deadline ordering, bounded trace
// ===========================================================================

// ---------------------------------------------------------------------------
// B1a: Data signal count tracks multiple signals (Bug 1 fix)
// ---------------------------------------------------------------------------
#[test]
fn test_data_signal_count_tracks_multiple_signals() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    let handle = scheduler
        .add_node(NodeConfig {
            id: "multi".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();

    // Signal 5 times between steps
    for _ in 0..5 {
        scheduler.signal_data("multi").unwrap();
    }

    // Handle should report 5 pending signals
    assert_eq!(handle.pending_data_count(), 5);

    // Per-message FIFO firing serves the WHOLE signalled backlog in
    // ONE step — one fire per arrival, "fire on each message arriving". A
    // collapse-to-one would fire once and reset the count to 0, silently
    // discarding the other 4 arrivals; a one-fire-per-step carry would keep
    // them instead, losing no frame but capping a data-trigger node's
    // THROUGHPUT at one frame per scheduler step — behind a 1 kHz producer
    // that is a permanent, queue-deep lag it can never work off.
    scheduler.step_ms(10);
    assert_eq!(
        count.load(Ordering::Relaxed),
        5,
        "one step serves every signalled arrival — five fires, not one"
    );
    assert_eq!(
        handle.pending_data_count(),
        0,
        "the backlog is drained by the step that saw it, not carried"
    );

    // No new signals → no fire
    scheduler.step_ms(10);
    assert_eq!(count.load(Ordering::Relaxed), 5);
}

// ---------------------------------------------------------------------------
// The Data burst loop's three exits, driven directly.
//
// These are PURE (no transport): they drive `signal_data` / the refill hook /
// `set_pre_fire_check` against hand oracles, so each of the loop's stopping
// conditions is pinned on its own rather than only in whatever combination a
// real graph happens to reach.
// ---------------------------------------------------------------------------

/// A burst's fires all belong to the STEP, not to invented sub-step instants:
/// k fires share ONE `fire_time_ns` and produce k separate `TraceEntry`s.
///
/// The shared instant is the `Period` catch-up precedent — a burst served
/// inside one step is served AT that step — and it is what keeps a recorded
/// trace replayable (the polled and live paths both derive it from the
/// scheduler's clock, never from a wall reading taken per fire). Separate
/// entries are what keep the trace ACCURATE about how many times the node ran.
#[test]
fn a_data_burst_records_one_trace_entry_per_fire_at_one_fire_time() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "sink".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();

    for _ in 0..4 {
        scheduler.signal_data("sink").unwrap();
    }
    scheduler.step_ms(7);

    assert_eq!(
        count.load(Ordering::Relaxed),
        4,
        "four arrivals, four fires"
    );
    let entries: Vec<_> = scheduler
        .trace()
        .iter()
        .filter(|e| e.node_id.as_ref() == "sink")
        .collect();
    assert_eq!(
        entries.len(),
        4,
        "each fire of the burst is its own trace entry — a burst recorded as \
         ONE entry would under-report how many times the node ran"
    );
    let times: Vec<u64> = entries.iter().map(|e| e.fire_time_ns).collect();
    assert_eq!(
        times,
        vec![7_000_000; 4],
        "every fire of the burst carries the STEP's gating time (the Period \
         catch-up precedent), not a per-fire wall reading"
    );
}

/// The per-step fire CAP bounds the work one step can do, and what it leaves
/// behind reports due-NOW so the live loop comes straight back for it.
///
/// Reachable only through the refill hook: `pending_data_count` saturates at
/// the same constant, so a signalled backlog can never exceed the cap. The hook
/// here yields more frames than the cap deliberately — a real Unified input is
/// bounded by its queue depth, which is that same constant, so the cap is a
/// structural bound on a shape today's depths cannot reach. Pinning it anyway
/// is what stops "the loop terminates" resting on the queue's depth.
#[test]
fn the_per_step_fire_cap_bounds_a_refilled_burst_and_the_remainder_reports_due_now() {
    use cerulion_core::graph::topology::MAX_CONSUMER_DEPTH;

    let cap = MAX_CONSUMER_DEPTH as u64;
    let offered = cap + 5;

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "sink".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();

    // A refill hook standing in for a Unified input holding `offered` frames:
    // each call pops one (returning `popped = 1`) until the supply runs out.
    let remaining = Arc::new(AtomicU64::new(offered - 1)); // one is signalled below
    let remaining_hook = Arc::clone(&remaining);
    scheduler
        .set_trigger_refill("sink", "inp", move || {
            let left = remaining_hook.load(Ordering::Relaxed);
            if left == 0 {
                (0, None)
            } else {
                remaining_hook.store(left - 1, Ordering::Relaxed);
                (1, Some(left))
            }
        })
        .unwrap();

    // The boundary drain's one signalled arrival opens the burst.
    scheduler.signal_data("sink").unwrap();
    scheduler.step_ms(1);

    assert_eq!(
        count.load(Ordering::Relaxed),
        cap,
        "one step fires at most the per-step cap, however many frames the \
         input still holds"
    );
    // The hook was called once per fire after the first, PLUS once more at the
    // cap: the loop asks for the next frame and only then finds it has run out
    // of budget, so that frame is popped and held un-fired. The input therefore
    // holds `offered - 1 - cap`, with one more in hand.
    assert_eq!(
        remaining.load(Ordering::Relaxed),
        offered - 1 - cap,
        "exactly the un-popped remainder is still in the input"
    );
    assert_eq!(
        scheduler.ns_until_next_fire(1_000_000),
        Some(0),
        "the frame the cap stopped us firing is popped and held with nothing \
         having signalled it — `pending_data_count` deliberately cannot \
         describe it (the next boundary drain re-offers a held head and mints \
         its signal then, so counting it now would fire twice for one frame), \
         so the live loop must be told to come back some other way or it \
         would wait out the liveliness cap for it"
    );

    // The next step serves the rest, so the cap DELAYS work rather than
    // dropping it. The signal stands in for what the boundary drain does on a
    // real graph: re-offer the held head and mint its arrival.
    scheduler.signal_data("sink").unwrap();
    scheduler.step_ms(1);
    assert_eq!(
        count.load(Ordering::Relaxed),
        offered,
        "the capped remainder is served by the following step — no loss"
    );
    // ...and the hint is RELEASED. Without this the wake floor is a one-way
    // door: `data_backlog_hint` is the only term of `ns_until_next_fire` that
    // no drain clears on its own, so a hint that survives its own backlog pins
    // the live loop at its 1 ms floor for the life of the process on an
    // otherwise idle Data node — the unbounded-poll class.
    //
    // Deleting `data_backlog_hint = false`
    // from `decide_node`'s Data arm fails here (`Some(0)`), and so does dropping
    // the `refilled_unfired = false` that each fire performs (the flag then
    // survives the fire it describes and re-arms the hint on a burst that
    // drained completely). Every OTHER assertion in this arm passes under both.
    assert_eq!(
        scheduler.ns_until_next_fire(2_000_000),
        None,
        "a served backlog stops shortening the wake window — the hint the cap \
         raised is cleared by the step that consumes what it described"
    );
}

/// A refilled burst that drains COMPLETELY raises no wake hint at all.
///
/// The cap arm above pins the hint's release AFTER it was raised; this pins that
/// it is never raised in the first place on the ordinary shape — a queue the
/// step emptied. These are two different regressions: the hint here is set only if a
/// fire fails to clear the `refilled_unfired` flag the refill before it set, so
/// this arm sees a burst-loop bookkeeping bug that a step-boundary clear would
/// paper over.
///
/// Dropping `refilled_unfired = false` after
/// `fire_node_into` fails exactly here (`Some(0)`), while the cap arm's
/// remaining assertions and every other arm in this file stay green.
#[test]
fn a_fully_served_refilled_burst_raises_no_wake_hint() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let (cb, count) = counting_callback();
    let handle = scheduler
        .add_node(NodeConfig {
            id: "sink".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();

    // Three frames behind the one the boundary drain signalled — well inside
    // the per-step cap, so the step drains the input completely.
    let remaining = Arc::new(AtomicU64::new(3));
    let remaining_hook = Arc::clone(&remaining);
    scheduler
        .set_trigger_refill("sink", "inp", move || {
            let left = remaining_hook.load(Ordering::Relaxed);
            if left == 0 {
                (0, None)
            } else {
                remaining_hook.store(left - 1, Ordering::Relaxed);
                (1, Some(left))
            }
        })
        .unwrap();

    scheduler.signal_data("sink").unwrap();
    scheduler.step_ms(1);

    assert_eq!(
        count.load(Ordering::Relaxed),
        4,
        "one signalled arrival plus three refilled frames, all fired in the \
         one step that saw them"
    );
    assert_eq!(remaining.load(Ordering::Relaxed), 0, "the input is drained");
    assert_eq!(
        handle.pending_data_count(),
        0,
        "nothing was carried — the burst was served, not deferred"
    );
    assert_eq!(
        scheduler.ns_until_next_fire(1_000_000),
        None,
        "a burst the step drained COMPLETELY leaves no reason to come back \
         early: an idle Data node has no deadline, and a hint raised here \
         would clamp the live loop's wake window to its floor forever"
    );
}

/// A mid-burst pre-fire defer stops the burst and CARRIES what it did not
/// fire — `throttle_ms` means "at most one fire per step", and it must cost
/// nothing.
///
/// The gate here is the throttle's own shape: allow while the node has never
/// fired, defer once it has. Without the per-fire re-check the burst would fire
/// all four in one step and the cap would be the only thing bounding a
/// throttled node.
#[test]
fn a_mid_burst_pre_fire_defer_fires_once_and_carries_the_rest() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let (cb, count) = counting_callback();
    let handle = scheduler
        .add_node(NodeConfig {
            id: "sink".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();

    let gate_handle = handle.clone();
    scheduler
        .set_pre_fire_check("sink", move |_now_ns| gate_handle.fire_count() > 0)
        .unwrap();

    for _ in 0..4 {
        scheduler.signal_data("sink").unwrap();
    }
    scheduler.step_ms(1);

    assert_eq!(
        count.load(Ordering::Relaxed),
        1,
        "a throttled node fires at most ONCE per step, whatever the backlog"
    );
    assert_eq!(
        handle.pending_data_count(),
        3,
        "the arrivals the defer left unfired are given back — deferring must \
         never lose a fire"
    );
    assert_eq!(
        scheduler.ns_until_next_fire(1_000_000),
        Some(0),
        "a carried remainder still reports due-NOW"
    );
}

// ---------------------------------------------------------------------------
// B3a: Bounded trace drops oldest entries (Bug 3 fix)
// ---------------------------------------------------------------------------
#[test]
fn test_trace_bounded_drops_oldest() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock).with_trace_limit(5);

    let (cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "ring".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // Fire 10 times (at 10ms, 20ms, ..., 100ms)
    scheduler.step_ms(100);

    let trace = scheduler.trace();
    assert_eq!(trace.len(), 5, "trace should be capped at limit");

    // Should contain the LAST 5 entries (60ms, 70ms, 80ms, 90ms, 100ms)
    assert_eq!(trace[0].fire_time_ns, 60_000_000);
    assert_eq!(trace[1].fire_time_ns, 70_000_000);
    assert_eq!(trace[2].fire_time_ns, 80_000_000);
    assert_eq!(trace[3].fire_time_ns, 90_000_000);
    assert_eq!(trace[4].fire_time_ns, 100_000_000);
}

// ---------------------------------------------------------------------------
// B3b: Unbounded trace (default) retains all entries (Bug 3 backward compat)
// ---------------------------------------------------------------------------
#[test]
fn test_trace_unbounded_default() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "unbounded".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // Fire 100 times
    scheduler.step_ms(1000);

    let trace = scheduler.trace();
    assert_eq!(
        trace.len(),
        100,
        "default (unbounded) trace should retain all entries"
    );
}

// ---------------------------------------------------------------------------
// Period catch-up burst is CLAMPED by the
// block pre-fire (overflow + mirror-desync + livelock guard).
//
// A pre-fire checked ONCE at the top of evaluate_node,
// BEFORE the Period catch-up loop, is not enough: a deferred Period block-producer that then
// resumed (queue had room for ~1) would fire N accumulated ticks in ONE step,
// publishing N frames into a queue with room for 1 → iceoryx2 overflow (data
// loss) AND the outstanding mirror would jump by N with no matching drain →
// outstanding stays >= threshold forever → the producer defers forever
// (livelock).
//
// The catch-up loop re-checks the pre-fire BEFORE EACH catch-up fire and BREAKs (rewinding
// next_fire_ns) the moment the predicate defers again. This test models the
// publish-into-a-bounded-queue mirror with a shared atomic and asserts:
//   (a) the mirror NEVER exceeds the threshold (no overflow), even on a big
//       multi-interval catch-up step;
//   (b) no livelock — once the queue drains, the producer fires the remaining
//       catch-up ticks (fire_count climbs past the threshold over time);
//   (c) every fire corresponds to exactly one increment (mirror stays in
//       lock-step: total fires == total publishes).
// ---------------------------------------------------------------------------
#[test]
fn test_period_catchup_clamped_by_block_pre_fire() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    // `outstanding` mirrors a bounded consumer queue; THRESHOLD is its
    // capacity (defer when outstanding >= THRESHOLD).
    const THRESHOLD: u64 = 3;
    let outstanding = Arc::new(AtomicU64::new(0));
    let published = Arc::new(AtomicU64::new(0));

    // Callback = "publish one frame": bump both the live mirror and a total
    // publish counter. (In production OutputProxy::drop / publish_raw bump the
    // mirror; here we model it directly.)
    let outstanding_cb = Arc::clone(&outstanding);
    let published_cb = Arc::clone(&published);
    let cb = Box::new(move || {
        outstanding_cb.fetch_add(1, Ordering::SeqCst);
        published_cb.fetch_add(1, Ordering::SeqCst);
    });

    scheduler
        .add_node(NodeConfig {
            id: "producer".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // Pre-fire: defer when the mirror is at/over capacity.
    let outstanding_pf = Arc::clone(&outstanding);
    scheduler
        .set_pre_fire_check("producer", move |_now_ns| {
            outstanding_pf.load(Ordering::SeqCst) >= THRESHOLD
        })
        .unwrap();

    // Track the max the mirror ever reached across the whole run.
    let mut max_outstanding = 0u64;

    // Step 1: a single BIG step spanning 10 intervals (100ms) with NO drain.
    // Without the #4 fix this would fire all 10 catch-up ticks at once,
    // driving `outstanding` to 10 (>> THRESHOLD) — overflow. With the fix the
    // burst breaks as soon as outstanding hits THRESHOLD.
    scheduler.step_ms(100);
    max_outstanding = max_outstanding.max(outstanding.load(Ordering::SeqCst));
    assert!(
        outstanding.load(Ordering::SeqCst) <= THRESHOLD,
        "(a) overflow: mirror exceeded the queue capacity during a Period catch-up \
         burst (got {}, threshold {THRESHOLD}) — the pre-fire was not re-checked \
         inside the catch-up loop",
        outstanding.load(Ordering::SeqCst)
    );
    let fires_after_burst = published.load(Ordering::SeqCst);
    assert_eq!(
        fires_after_burst, THRESHOLD,
        "the clamped catch-up burst should fire exactly THRESHOLD times before \
         the mirror fills (got {fires_after_burst})"
    );

    // Step 2: drain one slot, then step again. The producer must RESUME and
    // fire (no livelock) — proving next_fire_ns was rewound, not consumed.
    for _ in 0..5 {
        // Drain one frame (consumer reads one).
        outstanding.fetch_sub(1, Ordering::SeqCst);
        scheduler.step_ms(10);
        max_outstanding = max_outstanding.max(outstanding.load(Ordering::SeqCst));
        assert!(
            outstanding.load(Ordering::SeqCst) <= THRESHOLD,
            "(a) overflow on resume: mirror exceeded capacity after a drain+step \
             (got {}, threshold {THRESHOLD})",
            outstanding.load(Ordering::SeqCst)
        );
    }

    // (b) No livelock: the producer fired the rewound catch-up ticks as the
    // queue drained — total publishes climbed past the initial THRESHOLD.
    let total_fires = published.load(Ordering::SeqCst);
    assert!(
        total_fires > THRESHOLD,
        "(b) livelock: producer never resumed after draining (got {total_fires} \
         total fires, expected > {THRESHOLD})"
    );

    // (c) Mirror stayed in lock-step: every fire was exactly one publish, and
    // the mirror never overflowed across the entire run.
    assert!(
        max_outstanding <= THRESHOLD,
        "(c) mirror overflowed at some point (max {max_outstanding} > {THRESHOLD})"
    );
}

// Determinism companion: the clamped Period
// catch-up must be bit-identical across two runs (Principle #7). The pre-fire
// reads deterministic atomics and the closure receives the deterministic
// VirtualClock time, so the fire timeline is replay-stable.
#[test]
fn test_period_catchup_clamp_is_deterministic() {
    fn run() -> Vec<TraceEntry> {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        const THRESHOLD: u64 = 2;
        let outstanding = Arc::new(AtomicU64::new(0));
        let outstanding_cb = Arc::clone(&outstanding);
        let cb = Box::new(move || {
            outstanding_cb.fetch_add(1, Ordering::SeqCst);
        });
        scheduler
            .add_node(NodeConfig {
                id: "p".to_string(),
                policy: period(10),
                callback: cb,
            })
            .unwrap();
        let outstanding_pf = Arc::clone(&outstanding);
        scheduler
            .set_pre_fire_check("p", move |_now_ns| {
                outstanding_pf.load(Ordering::SeqCst) >= THRESHOLD
            })
            .unwrap();
        // Big catch-up step, then alternate drain+step.
        scheduler.step_ms(50);
        for _ in 0..4 {
            outstanding.fetch_sub(1, Ordering::SeqCst);
            scheduler.step_ms(10);
        }
        scheduler.trace().to_vec()
    }
    let a = run();
    let b = run();
    assert_eq!(
        a, b,
        "clamped Period catch-up fire timeline must be bit-identical across runs \
         (Principle #7)"
    );
    assert!(!a.is_empty(), "the node actually fired");
}

// Capped-rewind regression pin.
//
// This is the regression test for the capped-rewind bug. It is DISTINCT from
// `test_period_catchup_clamp_is_deterministic` above, which uses
// `max_catchup: None` and therefore NEVER enters the cap-skip loop where the
// bug lived.
//
// The bug: with `max_catchup: Some(N)`, a large step collects only the first N
// due intervals, then the cap-skip loop OVER-ADVANCES `next_fire_ns` past the
// whole burst to drop the stale (uncollected) intervals. If the pre-fire then
// defers MID-BURST (after firing some, but not all, of the COLLECTED N), the
// resume target must be the earliest un-fired COLLECTED interval. The buggy
// version arithmetic-rewound from the over-advanced post-cap `next_fire_ns`,
// landing it in the STALE region and silently DROPPING the un-fired collected
// intervals. The fix sets `*next_fire = fire_time` directly.
//
// Setup: interval=10ms, max_catchup=Some(2). An "outstanding" counter + an
// "allow" latch model a draining consumer. The pre-fire defers (returns true)
// once outstanding >= 2.
#[test]
fn test_period_capped_rewind_resumes_at_earliest_unfired_collected_interval() {
    const INTERVAL_MS: u64 = 10;
    const MAX_CATCHUP: u32 = 2;
    const THRESHOLD: u64 = 2; // defer once outstanding reaches max_catchup

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    // The producer "publishes" once per fire by bumping the outstanding mirror.
    let outstanding = Arc::new(AtomicU64::new(0));
    let outstanding_cb = Arc::clone(&outstanding);
    let cb = Box::new(move || {
        outstanding_cb.fetch_add(1, Ordering::SeqCst);
    });
    scheduler
        .add_node(NodeConfig {
            id: "p".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(INTERVAL_MS),
                max_catchup: Some(MAX_CATCHUP),
            },
            callback: cb,
        })
        .unwrap();

    // Pre-fire defers once the consumer is "full" (outstanding >= THRESHOLD).
    // A draining consumer is simulated externally by decrementing `outstanding`.
    let outstanding_pf = Arc::clone(&outstanding);
    scheduler
        .set_pre_fire_check("p", move |_now_ns| {
            outstanding_pf.load(Ordering::SeqCst) >= THRESHOLD
        })
        .unwrap();

    // Big step: 100ms => 10 intervals due (10ms..=100ms). Capped to 2 collected
    // (10ms, 20ms); the cap-skip loop over-advances `next_fire_ns` to 110ms.
    // The burst drains: fire 10ms (outstanding 0->1, pre-fire still allows),
    // then before 20ms the pre-fire sees outstanding==1 (<2) and ALLOWS, fires
    // 20ms (outstanding 1->2). Both collected intervals fire this step; the
    // mid-burst defer is exercised on the NEXT large step below.
    scheduler.step_ms(100);

    // Assertion (1): the node fired AT MOST max_catchup ticks in the burst —
    // bounded by the cap. The pre-fire allowed both because each fire only
    // pushed outstanding to the threshold AFTER firing (defer is `>=`, checked
    // BEFORE the fire), so exactly 2 fired here.
    {
        let trace = scheduler.trace();
        assert!(
            trace.len() <= MAX_CATCHUP as usize,
            "burst must be capped to max_catchup ({MAX_CATCHUP}); got {} fires",
            trace.len()
        );
        assert_eq!(
            trace.len(),
            2,
            "both collected intervals fired before outstanding hit the defer threshold"
        );
        assert_eq!(
            trace[0].fire_time_ns, 10_000_000,
            "first collected interval"
        );
        assert_eq!(
            trace[1].fire_time_ns, 20_000_000,
            "second collected interval"
        );
        // outstanding is now 2 -> the pre-fire will DEFER on the next step.
        assert_eq!(outstanding.load(Ordering::SeqCst), 2);
    }

    // Now exercise the MID-BURST defer + capped-rewind resume. Step again: the
    // node is at next_fire=110ms (over-advanced by the cap-skip on the prior
    // step). With outstanding==2 the pre-fire defers at the TOP of evaluate_node
    // (before the Period arm), so no new collection happens and next_fire stays
    // at 110ms. No new fires.
    scheduler.clear_trace();
    scheduler.step_ms(100); // clock now at 200ms; pre-fire defers at the top
    assert!(
        scheduler.trace().is_empty(),
        "fully-deferred step must not fire (next_fire untouched)"
    );

    // Drain the consumer fully so the pre-fire ALLOWS again.
    outstanding.store(0, Ordering::SeqCst);

    // Step once more. next_fire is 110ms <= 200ms, so the burst re-collects
    // from 110ms. This is the steady-state resume path. The KEY mutation-
    // sensitive property the capped-rewind fix guarantees is exercised below
    // via a direct mid-burst defer, so first confirm resume fires from the
    // earliest un-fired interval (110ms), not a skipped-ahead stale value.
    scheduler.clear_trace();
    scheduler.step_ms(0); // re-evaluate at 200ms without advancing
    {
        let trace = scheduler.trace();
        assert!(!trace.is_empty(), "node must RESUME firing after drain");
        assert_eq!(
            trace[0].fire_time_ns, 110_000_000,
            "resume must fire the earliest un-fired interval (110ms), not a stale skipped value"
        );
    }
}

// Capped-rewind regression pin, direct.
//
// The most surgical pin of the fix: force a defer EXACTLY at the boundary
// between the two collected intervals, then assert the resume fire_time is the
// un-fired COLLECTED interval (the buggy arithmetic-rewind landed it in the
// stale region and DROPPED it). Uses an `allow` latch flipped externally so we
// control precisely when the burst defers mid-stream.
#[test]
fn test_period_capped_rewind_mid_burst_defer_does_not_drop_collected_interval() {
    const INTERVAL_MS: u64 = 10;
    const MAX_CATCHUP: u32 = 2;

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    // Count fires AND record how many fires the pre-fire has already permitted,
    // so we can defer precisely after the FIRST collected interval fires.
    let fires = Arc::new(AtomicU64::new(0));
    let fires_cb = Arc::clone(&fires);
    let cb = Box::new(move || {
        fires_cb.fetch_add(1, Ordering::SeqCst);
    });
    scheduler
        .add_node(NodeConfig {
            id: "p".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(INTERVAL_MS),
                max_catchup: Some(MAX_CATCHUP),
            },
            callback: cb,
        })
        .unwrap();

    // The pre-fire defers as soon as one fire has happened in this burst window.
    // `allow` gates whether we permit any firing at all. With allow=true and the
    // fires counter, the closure defers after the first fire => mid-burst defer
    // on the SECOND collected interval (20ms), leaving it un-fired.
    let allow = Arc::new(std::sync::atomic::AtomicBool::new(true));
    let allow_pf = Arc::clone(&allow);
    let fires_pf = Arc::clone(&fires);
    // baseline captured at the start of each evaluate to count fires within the burst
    let burst_baseline = Arc::new(AtomicU64::new(0));
    let burst_baseline_pf = Arc::clone(&burst_baseline);
    scheduler
        .set_pre_fire_check("p", move |_now_ns| {
            if !allow_pf.load(Ordering::SeqCst) {
                return true; // hard-deferred
            }
            // Defer once at least one fire has occurred since the burst baseline.
            let fired_in_burst =
                fires_pf.load(Ordering::SeqCst) - burst_baseline_pf.load(Ordering::SeqCst);
            fired_in_burst >= 1
        })
        .unwrap();

    // Big step: 10 intervals due, capped to 2 collected (10ms, 20ms); cap-skip
    // over-advances next_fire to 110ms. Drain: fire 10ms (now fired_in_burst=1),
    // then the pre-fire DEFERS before 20ms. The fix sets next_fire = 20ms (the
    // un-fired collected interval). The bug would arithmetic-rewind from 110ms
    // and land in the stale region, DROPPING 20ms.
    burst_baseline.store(fires.load(Ordering::SeqCst), Ordering::SeqCst);
    scheduler.step_ms(100);

    {
        let trace = scheduler.trace();
        assert_eq!(
            trace.len(),
            1,
            "exactly one fire before the mid-burst defer"
        );
        assert_eq!(
            trace[0].fire_time_ns, 10_000_000,
            "first collected interval"
        );
    }
    assert_eq!(fires.load(Ordering::SeqCst), 1);

    // "Drain" + re-arm the burst baseline so the pre-fire allows again.
    burst_baseline.store(fires.load(Ordering::SeqCst), Ordering::SeqCst);
    scheduler.clear_trace();

    // Re-evaluate WITHOUT advancing time. If the fix is in place, next_fire is
    // 20ms (the un-fired collected interval) and 20ms <= 100ms, so it re-fires
    // at 20ms. If the bug were present, next_fire would be ~100ms or 110ms (the
    // stale-region landing), so the resume fire_time would be 100_000_000 (or
    // the node would not fire 20ms at all) — the assertion below catches it.
    scheduler.step_ms(0);
    {
        let trace = scheduler.trace();
        assert!(
            !trace.is_empty(),
            "resume must re-fire the un-fired collected interval, not drop it"
        );
        assert_eq!(
            trace[0].fire_time_ns, 20_000_000,
            "MUTATION-SENSITIVE: resume fire_time must be the earliest un-fired COLLECTED \
             interval (20ms). The buggy arithmetic-rewind landed next_fire in the stale \
             region (~100ms) and DROPPED the 20ms interval — this assertion fails under that bug."
        );
    }
}

/// Coverage for a review test gap: the FireKind::Period arithmetic
/// reconstruction is a RUNNING ADD (`fire_time += interval_ns`) across the burst.
/// The capped-rewind test above fires ONE interval then defers, so it only pins
/// the first reconstructed value + the rewind. This UNCAPPED (`max_catchup: None`)
/// variant fires THREE intervals (10/20/30ms) before deferring, pinning that the
/// running add ACCUMULATES correctly across multiple intervals (an off-by-one in
/// `first_fire_ns + k*interval` would mis-place the 2nd/3rd) and that the mid-burst
/// defer then rewinds to the 4th (un-fired) interval (40ms), not the over-advanced
/// next_fire.
#[test]
fn test_period_uncapped_multi_interval_reconstruction_then_defer_rewinds() {
    const INTERVAL_MS: u64 = 10;

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let fires = Arc::new(AtomicU64::new(0));
    let fires_cb = Arc::clone(&fires);
    scheduler
        .add_node(NodeConfig {
            id: "p".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(INTERVAL_MS),
                max_catchup: None, // UNCAPPED — the whole burst is collected
            },
            callback: Box::new(move || {
                fires_cb.fetch_add(1, Ordering::SeqCst);
            }),
        })
        .unwrap();

    // Defer once THREE fires have happened in the burst window → fires 10/20/30ms,
    // then the pre-fire defers before the 4th collected interval (40ms).
    let fires_pf = Arc::clone(&fires);
    let burst_baseline = Arc::new(AtomicU64::new(0));
    let burst_baseline_pf = Arc::clone(&burst_baseline);
    scheduler
        .set_pre_fire_check("p", move |_now_ns| {
            fires_pf.load(Ordering::SeqCst) - burst_baseline_pf.load(Ordering::SeqCst) >= 3
        })
        .unwrap();

    // Big step: 10 intervals due, uncapped → all collected. Drain fires 10/20/30ms
    // (running add), then defers before 40ms, leaving next_fire = 40ms.
    burst_baseline.store(fires.load(Ordering::SeqCst), Ordering::SeqCst);
    scheduler.step_ms(100);
    {
        let trace = scheduler.trace();
        let times: Vec<u64> = trace.iter().map(|e| e.fire_time_ns).collect();
        assert_eq!(
            times,
            vec![10_000_000, 20_000_000, 30_000_000],
            "MUTATION-SENSITIVE: the reconstruction must place the first three collected \
             intervals at exactly 10/20/30ms — an off-by-one in `first_fire_ns + k*interval` \
             would mis-place the 2nd/3rd"
        );
    }
    assert_eq!(fires.load(Ordering::SeqCst), 3);

    // Re-arm + re-evaluate without advancing time: resume must fire the un-fired
    // 4th collected interval (40ms), proving the mid-burst defer rewound next_fire
    // to the earliest un-fired reconstructed interval (not the over-advanced end).
    burst_baseline.store(fires.load(Ordering::SeqCst), Ordering::SeqCst);
    scheduler.clear_trace();
    scheduler.step_ms(0);
    {
        let trace = scheduler.trace();
        assert!(
            !trace.is_empty(),
            "resume must re-fire the un-fired 40ms interval"
        );
        assert_eq!(
            trace[0].fire_time_ns, 40_000_000,
            "MUTATION-SENSITIVE: resume fire_time must be the earliest un-fired reconstructed \
             interval (40ms) after a 3-fire burst — pins both the running-add accumulation and \
             the rewind-to-fire_time on defer"
        );
    }
}

// ===========================================================================
// Group 1: `Scheduler::ns_until_next_fire` UNIT tests.
//
// `ns_until_next_fire(now_ns)` is a PURE READ returning the ns until the
// soonest pending `Period` fire among ENABLED nodes (None if no enabled
// Period node has a pending deadline). It feeds `GraphRuntime::live_timeout`
// (the live-loop WaitSet timeout). Period `next_fire_ns` is initialized at
// `add_node` to `clock.now_ns() + interval` — under `with_virtual_clock`
// (clock reads 0 at build) that is `0 + interval = interval`.
//
// All assertions are ORACLE values (exact, never self-comparison).
// ===========================================================================

// Per-message FIFO firing: a Data node with a
// carried pending backlog reports due-NOW through `ns_until_next_fire`, so
// the live loop's wake window clamps to its 1 ms floor and a burst drains
// one frame per millisecond instead of one per 250 ms liveliness window
// (nothing external rings again for an already-drained notification). A
// Data node with NO backlog contributes nothing — the pre-FIFO behavior.
#[test]
fn data_backlog_reports_due_now_and_an_idle_data_node_reports_nothing() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "sink".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();

    // Idle Data node: no deadline of any kind.
    assert_eq!(
        scheduler.ns_until_next_fire(0),
        None,
        "an idle Data node has no pending deadline"
    );

    // Three signalled arrivals: the backlog is due NOW.
    for _ in 0..3 {
        scheduler.signal_data("sink").unwrap();
    }
    assert_eq!(
        scheduler.ns_until_next_fire(0),
        Some(0),
        "a carried FIFO backlog is due NOW — the live wake window clamps \
         to its floor until the backlog drains"
    );

    // The step that sees the backlog SERVES it, so one step returns
    // the node to idle. The due-NOW term still matters — it covers what a step
    // could not serve (a remainder past the per-step fire cap, or arrivals a
    // mid-burst `block`/`throttle` defer left unfired) plus the Unified path's
    // frozen-but-unfired head — but a burst is no longer paid for one wake at
    // a time.
    scheduler.step_ms(1);
    assert_eq!(
        count.load(Ordering::Relaxed),
        3,
        "the backlog was SERVED, not discarded — one fire per signalled \
         arrival. Without this, a step that cleared or dropped the backlog \
         while firing zero or one callback satisfies the wake-window \
         assertion below just as well"
    );
    assert_eq!(
        scheduler.ns_until_next_fire(1_000_000),
        None,
        "a drained backlog stops shortening the wake window"
    );
}

// G1.1: Empty scheduler → None (no Period node at all).
#[test]
fn test_ns_until_next_fire_empty_is_none() {
    let clock = Arc::new(VirtualClock::new());
    let scheduler = Scheduler::with_virtual_clock(clock);
    assert_eq!(
        scheduler.ns_until_next_fire(0),
        None,
        "an empty scheduler has no pending Period deadline → None"
    );
}

// G1.2: One Period{10ms} node, no step. The deadline is at 10ms.
//   - at now=0      → 10ms remaining
//   - at now=3ms    → 7ms remaining
//   - at now=10ms   → 0 (deadline reached exactly)
//   - at now=15ms   → 0 (past-due; saturating_sub, NOT a huge wraparound)
// The now=15ms case is the load-bearing `saturating_sub` pin.
#[test]
fn test_ns_until_next_fire_single_period_countdown_and_saturation() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let (cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "p".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    assert_eq!(
        scheduler.ns_until_next_fire(0),
        Some(10_000_000),
        "at now=0 the 10ms deadline is 10ms away"
    );
    assert_eq!(
        scheduler.ns_until_next_fire(3_000_000),
        Some(7_000_000),
        "at now=3ms the 10ms deadline is 7ms away"
    );
    assert_eq!(
        scheduler.ns_until_next_fire(10_000_000),
        Some(0),
        "at now=10ms the deadline is reached → 0"
    );
    assert_eq!(
        scheduler.ns_until_next_fire(15_000_000),
        Some(0),
        "PAST-DUE saturating pin: now=15ms is 5ms past the 10ms deadline → \
         saturating_sub yields 0, NOT a near-u64::MAX wraparound"
    );
}

// G1.3: Two Period nodes (10ms + 20ms) → the MIN remaining.
//   - at now=0  → min(10ms, 20ms) = 10ms
//   - at now=12ms → the 10ms node is past-due (Some(0)); min(0, 8ms) = 0
#[test]
fn test_ns_until_next_fire_two_periods_returns_min() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let (cb_a, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fast".to_string(),
            policy: period(10),
            callback: cb_a,
        })
        .unwrap();
    let (cb_b, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "slow".to_string(),
            policy: period(20),
            callback: cb_b,
        })
        .unwrap();

    assert_eq!(
        scheduler.ns_until_next_fire(0),
        Some(10_000_000),
        "the soonest of {{10ms, 20ms}} deadlines is 10ms"
    );
    assert_eq!(
        scheduler.ns_until_next_fire(12_000_000),
        Some(0),
        "at now=12ms the 10ms node is past-due (0), so the min across the two \
         is 0 (the 20ms node is still 8ms out)"
    );
}

// G1.4: Non-Period policies do NOT contribute a deadline.
//   - Data-only → None
//   - Sync-only → None
//   - External-only → None
//   - a graph with BOTH a Data node AND a Period{10ms} node → Some(10ms)
//     (only the Period node counts).
#[test]
fn test_ns_until_next_fire_ignores_non_period_policies() {
    // Data-only.
    {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        let (cb, _) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "d".to_string(),
                policy: TriggerPolicy::Data,
                callback: cb,
            })
            .unwrap();
        assert_eq!(
            scheduler.ns_until_next_fire(0),
            None,
            "a Data-only node has no Period deadline → None"
        );
    }
    // Sync-only.
    {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        let (cb, _) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "s".to_string(),
                policy: TriggerPolicy::Sync {
                    inputs: vec!["a".to_string(), "b".to_string()],
                    window: Some(Duration::from_millis(50)),
                },
                callback: cb,
            })
            .unwrap();
        assert_eq!(
            scheduler.ns_until_next_fire(0),
            None,
            "a Sync-only node has no Period deadline → None"
        );
    }
    // External-only.
    {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        let (cb, _) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "e".to_string(),
                policy: TriggerPolicy::External,
                callback: cb,
            })
            .unwrap();
        assert_eq!(
            scheduler.ns_until_next_fire(0),
            None,
            "an External-only node has no Period deadline → None"
        );
    }
    // Data + Period{10ms} → only the Period counts.
    {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        let (cb_d, _) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "d".to_string(),
                policy: TriggerPolicy::Data,
                callback: cb_d,
            })
            .unwrap();
        let (cb_p, _) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "p".to_string(),
                policy: period(10),
                callback: cb_p,
            })
            .unwrap();
        assert_eq!(
            scheduler.ns_until_next_fire(0),
            Some(10_000_000),
            "with a Data node and a Period{{10ms}} node, only the Period \
             contributes a deadline → Some(10ms)"
        );
    }
}

// G1.5: PURITY / firewall — the accessor mutates NOTHING.
//
// Build a scheduler with a Period{10ms} node; call `ns_until_next_fire`
// THREE times; then `step(10ms)`. The node must fire EXACTLY once (as if the
// accessor calls never happened). A CONTROL scheduler with no accessor calls
// must reach the identical fire_count after the identical step.
#[test]
fn test_ns_until_next_fire_is_pure_no_mutation() {
    // Subject: poll the accessor 3× before stepping.
    let subject_count = {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        let (cb, count) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "p".to_string(),
                policy: period(10),
                callback: cb,
            })
            .unwrap();
        // Three PURE reads — must not advance, consume, or rewind any deadline.
        let _ = scheduler.ns_until_next_fire(5_000_000);
        let _ = scheduler.ns_until_next_fire(5_000_000);
        let _ = scheduler.ns_until_next_fire(5_000_000);
        scheduler.step(Duration::from_millis(10));
        count.load(Ordering::Relaxed)
    };

    // Control: identical scheduler, NO accessor calls.
    let control_count = {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        let (cb, count) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "p".to_string(),
                policy: period(10),
                callback: cb,
            })
            .unwrap();
        scheduler.step(Duration::from_millis(10));
        count.load(Ordering::Relaxed)
    };

    assert_eq!(
        subject_count, 1,
        "a single 10ms step fires the 10ms-period node exactly once, regardless \
         of prior accessor calls (the accessor is a pure read)"
    );
    assert_eq!(
        subject_count, control_count,
        "PURITY: a scheduler polled by ns_until_next_fire 3× must reach the SAME \
         fire_count after a 10ms step as an un-polled control — the accessor \
         mutates no scheduler state"
    );
}

// ===========================================================================
// Group 2: RealClock + Period NO-EXPLOSION regression.
//
// Pins the `add_node` clock-RELATIVE Period init. Without it, a
// Period node under RealClock would have `next_fire_ns` initialized
// absolute-from-0; the first `step` would see `now = real_ns()` (a huge value)
// and catch-up-fire up to `u32::MAX` times in one step → hang/explosion.
//
// The LOAD-BEARING assertion is `count == 0` on the FIRST step: a regression
// to the absolute-from-0 init would make the first step hang or explode.
// ===========================================================================
#[test]
#[serial] // RealClock + a real wall sleep; keep isolated from parallel timing.
fn test_real_clock_period_no_catchup_explosion_on_first_step() {
    let mut scheduler = Scheduler::new(); // RealClock

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "p".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(10),
                max_catchup: None,
            },
            callback: cb,
        })
        .unwrap();

    // First step ~immediately after add_node: next_fire = build_wall + 10ms,
    // which is still in the FUTURE relative to `now` → 0 fires. This MUST
    // return promptly (no hang). Under the buggy absolute-from-0 init this
    // step would attempt ~real_ns()/10ms catch-up fires and hang/explode.
    scheduler.step(Duration::from_millis(1));
    assert_eq!(
        count.load(Ordering::Relaxed),
        0,
        "REGRESSION PIN: a fresh RealClock Period node must NOT catch-up-fire on \
         the first step (clock-relative next_fire init). A revert to \
         absolute-from-0 init would hang/explode here."
    );

    // After ~25ms of real time, ≥2 period deadlines (10ms, 20ms) have passed.
    // The node is `max_catchup: None` (UNBOUNDED) ON PURPOSE — this pins that
    // the clock-relative init protects even the worst case. The load-bearing
    // contrast is "a SMALL BOUNDED number of catch-up fires" vs the ~4.29-BILLION
    // (u32::MAX) the absolute-from-0 init would attempt — so the upper bound is
    // deliberately LOOSE (< 100_000 ≈ 16 min of stall at 10ms/fire) to never flake
    // under arbitrary CI load while still failing loud on the explosion regression.
    // (An earlier ≤15 bound flaked on a heavily-loaded macOS CI runner that
    // stretched the wall window to 16 fires — the tight bound tested CI load, not
    // the clamp.) The truly load-bearing pin is the `count == 0` FIRST-step
    // assertion above; this is the secondary "bounded, not unbounded" guard.
    std::thread::sleep(Duration::from_millis(25));
    scheduler.step(Duration::from_millis(1));
    let c = count.load(Ordering::Relaxed);
    assert!(
        c >= 1,
        "after a ≥25ms real sleep, an UNBOUNDED 10ms-period node must have caught \
         up at least once; got {c}"
    );
    assert!(
        c < 100_000,
        "catch-up must be BOUNDED, NOT a u32::MAX (~4.29-billion) explosion from \
         absolute-from-0 init; got {c}"
    );
}

// ---------------------------------------------------------------------------
// The `signal_*` desync-rejection counter
//
// `NodeHandle::signal_failed_count()` makes the wrong-policy / not-declared
// `Err` arms of `signal_data` / `signal_sync_input` observable. It is a RAW
// per-call count (NOT latched like the runtime's per-regime warn — see the
// field doc). The Ok firing path is byte-identical — the no-bump tests below
// pin that the counter never touches a successful signal (the replay firewall
// is the other guard).
// ---------------------------------------------------------------------------

/// The canonical deferral case: `signal_data()` on a Period node is rejected
/// and counted. The bump is cumulative across repeated desyncs.
#[test]
fn test_signal_data_on_period_node_bumps_signal_failed_count() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    let handle = scheduler
        .add_node(NodeConfig {
            id: "camera".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    assert_eq!(handle.signal_failed_count(), 0, "no desync yet");

    // signal_data is invalid for a Period node → Err + one count.
    let err = scheduler.signal_data("camera");
    assert!(
        matches!(err, Err(TransportError::SchedulerError { .. })),
        "signal_data on a Period node must be a SchedulerError, got {err:?}"
    );
    assert_eq!(handle.signal_failed_count(), 1, "one desync counted");

    // Cumulative: a second rejection increments again.
    let _ = scheduler.signal_data("camera");
    assert_eq!(handle.signal_failed_count(), 2, "desyncs accumulate");
}

/// The Ok path is byte-identical: a valid `signal_data()` on a Data node
/// succeeds and NEVER bumps the desync counter.
#[test]
fn test_signal_data_on_data_node_does_not_bump_signal_failed_count() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    let handle = scheduler
        .add_node(NodeConfig {
            id: "detector".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();

    scheduler.signal_data("detector").unwrap();
    scheduler.signal_data("detector").unwrap();
    assert_eq!(
        handle.signal_failed_count(),
        0,
        "a successful signal_data must not touch the desync counter"
    );
}

/// `signal_sync_input()` on a non-Sync node is rejected and counted.
#[test]
fn test_signal_sync_input_wrong_policy_bumps_signal_failed_count() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    let handle = scheduler
        .add_node(NodeConfig {
            id: "camera".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    let err = scheduler.signal_sync_input("camera", "imu", 1_000_000);
    assert!(
        matches!(err, Err(TransportError::SchedulerError { .. })),
        "signal_sync_input on a Period node must be a SchedulerError, got {err:?}"
    );
    assert_eq!(handle.signal_failed_count(), 1);
}

/// On a Sync node, an UN-declared input is rejected and counted; a declared
/// input succeeds and does NOT bump the counter.
#[test]
fn test_signal_sync_input_undeclared_input_bumps_but_valid_does_not() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    let handle = scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["camera".to_string(), "imu".to_string()],
                window: Some(Duration::from_millis(50)),
            },
            callback: cb,
        })
        .unwrap();

    // "lidar" is not a declared sync input → not-declared Err arm.
    let err = scheduler.signal_sync_input("fusion", "lidar", 1_000_000);
    assert!(
        matches!(err, Err(TransportError::SchedulerError { .. })),
        "undeclared sync input must be a SchedulerError, got {err:?}"
    );
    assert_eq!(handle.signal_failed_count(), 1, "undeclared input counted");

    // A declared input succeeds — the Ok path leaves the counter untouched.
    scheduler
        .signal_sync_input("fusion", "camera", 2_000_000)
        .unwrap();
    assert_eq!(
        handle.signal_failed_count(),
        1,
        "a valid sync input must not bump the desync counter"
    );
}

/// The counter is per-node: a desync on one node never leaks into another.
#[test]
fn test_signal_failed_count_is_per_node() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb_a, _) = counting_callback();
    let handle_a = scheduler
        .add_node(NodeConfig {
            id: "alpha".to_string(),
            policy: period(10),
            callback: cb_a,
        })
        .unwrap();
    let (cb_b, _) = counting_callback();
    let handle_b = scheduler
        .add_node(NodeConfig {
            id: "beta".to_string(),
            policy: period(10),
            callback: cb_b,
        })
        .unwrap();

    let _ = scheduler.signal_data("alpha");
    assert_eq!(
        handle_a.signal_failed_count(),
        1,
        "alpha counted its desync"
    );
    assert_eq!(handle_b.signal_failed_count(), 0, "beta untouched");
}

/// `signal_data` / `signal_sync_input` on an External node also hit the
/// wrong-policy catch-all arm and are counted. Guards against a future
/// `match` that special-cases `External` ahead of the `_ =>` arm and silently
/// stops counting these desyncs.
#[test]
fn test_signal_on_external_node_bumps_signal_failed_count() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    let handle = scheduler
        .add_node(NodeConfig {
            id: "ext".to_string(),
            policy: TriggerPolicy::External,
            callback: cb,
        })
        .unwrap();

    assert!(scheduler.signal_data("ext").is_err());
    assert!(scheduler
        .signal_sync_input("ext", "any", 1_000_000)
        .is_err());
    assert_eq!(
        handle.signal_failed_count(),
        2,
        "both signal_data and signal_sync_input desyncs counted on an External node"
    );
}

// ---------------------------------------------------------------------------
// B-dur: TraceEntry::duration_ns (Mode B wall-tick telemetry)
// ---------------------------------------------------------------------------

/// Pure oracle for the HAND-WRITTEN `PartialEq`: `duration_ns` is EXCLUDED
/// from equality (non-replayable wall time, Principle #7) while `node_id`
/// and `fire_time_ns` still distinguish entries.
#[test]
fn test_trace_entry_partial_eq_ignores_duration_ns() {
    let base = TraceEntry {
        node_id: Arc::from("n"),
        step: 0,
        fire_time_ns: 1_000,
        global_level: 0,
        duration_ns: 5,
        discarded: false,
    };

    // Same id + fire_time, WILDLY different duration → EQUAL (excluded).
    let differ_duration = TraceEntry {
        node_id: Arc::from("n"),
        step: 0,
        fire_time_ns: 1_000,
        global_level: 0,
        duration_ns: 999_999,
        discarded: false,
    };
    assert_eq!(base, differ_duration);

    // Same id + fire_time, DIFFERENT `discarded` → EQUAL (excluded, a
    // data-frame annotation exactly like `duration_ns`). This pins the
    // documented invariant: the exit-6 divergence key must NOT trip on
    // `discarded`, and the flat-vs-level / rayon byte-identity assertions must
    // stay green. Regression guard — adding `&& self.discarded ==
    // other.discarded` to `eq` compiles and passes every existing test (no
    // byte-identity test discards), silently rotting the invariant; this case
    // catches it.
    let differ_discarded = TraceEntry {
        node_id: Arc::from("n"),
        step: 0,
        fire_time_ns: 1_000,
        global_level: 0,
        duration_ns: 5,
        discarded: true,
    };
    assert_eq!(base, differ_discarded);

    // Both non-replayable annotations differing AT ONCE → still EQUAL.
    let differ_both_annotations = TraceEntry {
        node_id: Arc::from("n"),
        step: 0,
        fire_time_ns: 1_000,
        global_level: 0,
        duration_ns: 999_999,
        discarded: true,
    };
    assert_eq!(base, differ_both_annotations);

    // Different fire_time → NOT equal (still distinguishes).
    let differ_fire_time = TraceEntry {
        node_id: Arc::from("n"),
        step: 0,
        fire_time_ns: 2_000,
        global_level: 0,
        duration_ns: 5,
        discarded: false,
    };
    assert_ne!(base, differ_fire_time);

    // Different node_id → NOT equal (still distinguishes).
    let differ_node_id = TraceEntry {
        node_id: Arc::from("m"),
        step: 0,
        fire_time_ns: 1_000,
        global_level: 0,
        duration_ns: 5,
        discarded: false,
    };
    assert_ne!(base, differ_node_id);
}

/// The HAND-WRITTEN `PartialEq` INCLUDES `global_level`
/// (replay-deterministic DAG topology) — the dual of the exclusion oracle above.
/// Two entries equal in `(node_id, fire_time_ns)` but differing ONLY in
/// `global_level` must compare `!=` (proving inclusion); re-affirm that differing
/// ONLY in `duration_ns` still compares `==` (exclusion holds alongside).
#[test]
fn test_trace_entry_partial_eq_includes_global_level() {
    let base = TraceEntry {
        node_id: Arc::from("n"),
        step: 0,
        fire_time_ns: 1_000,
        global_level: 0,
        duration_ns: 5,
        discarded: false,
    };

    // Same id + fire_time, DIFFERENT global_level → NOT equal (INCLUDED).
    let differ_global_level = TraceEntry {
        node_id: Arc::from("n"),
        step: 0,
        fire_time_ns: 1_000,
        global_level: 3,
        duration_ns: 5,
        discarded: false,
    };
    assert_ne!(
        base, differ_global_level,
        "global_level is replay-deterministic and MUST be part of equality"
    );

    // Same id + fire_time + global_level, different duration → EQUAL (duration
    // still EXCLUDED even with global_level now in the eq).
    let differ_duration_only = TraceEntry {
        node_id: Arc::from("n"),
        step: 0,
        fire_time_ns: 1_000,
        global_level: 0,
        duration_ns: 999_999,
        discarded: false,
    };
    assert_eq!(
        base, differ_duration_only,
        "duration_ns stays EXCLUDED from equality (non-replayable wall time)"
    );
}

/// The HAND-WRITTEN `PartialEq` INCLUDES `step` (the
/// 0-based logical-step counter — replay-deterministic, the cross-process merge's
/// primary sort key). Two entries equal in EVERY other compared field but
/// differing ONLY in `step` must compare `!=` (proving inclusion); re-affirm that
/// differing ONLY in `duration_ns` still compares `==` (exclusion holds alongside).
#[test]
fn test_trace_entry_partial_eq_includes_step() {
    let base = TraceEntry {
        node_id: Arc::from("n"),
        step: 0,
        fire_time_ns: 1_000,
        global_level: 0,
        duration_ns: 5,
        discarded: false,
    };

    // Same id + fire_time + global_level, DIFFERENT step → NOT equal (INCLUDED).
    let differ_step = TraceEntry {
        node_id: Arc::from("n"),
        step: 7,
        fire_time_ns: 1_000,
        global_level: 0,
        duration_ns: 5,
        discarded: false,
    };
    assert_ne!(
        base, differ_step,
        "step is replay-deterministic and MUST be part of equality"
    );

    // Same id + fire_time + global_level + step, different duration → EQUAL
    // (duration still EXCLUDED even with step now in the eq).
    let differ_duration_only = TraceEntry {
        node_id: Arc::from("n"),
        step: 0,
        fire_time_ns: 1_000,
        global_level: 0,
        duration_ns: 999_999,
        discarded: false,
    };
    assert_eq!(
        base, differ_duration_only,
        "duration_ns stays EXCLUDED from equality (non-replayable wall time)"
    );
}

/// The flat (`Scheduler::step`) path increments the
/// per-step `TraceEntry.step` index by exactly 1 per `step()`, independent of
/// iceoryx2 / the level executor. A single `period(5)` node stepped 3× at 5ms
/// fires once per step; the emitted trace must carry `step` 0, 1, 2 in order —
/// directly pinning the `begin_step`/`current_step` per-step bump contract that
/// the cross-process merge's PRIMARY sort key depends on.
#[test]
fn test_flat_path_step_increments_per_step() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "p".to_string(),
            policy: period(5),
            callback: cb,
        })
        .unwrap();

    // period 5ms, delta 5ms × 3 ⇒ exactly one fire per step.
    scheduler.step_ms(5);
    scheduler.step_ms(5);
    scheduler.step_ms(5);

    let trace = scheduler.trace();
    assert_eq!(trace.len(), 3, "one fire per step ⇒ 3 fires across 3 steps");
    assert_eq!(trace[0].step, 0, "first step stamps step index 0");
    assert_eq!(trace[1].step, 1, "second step stamps step index 1");
    assert_eq!(trace[2].step, 2, "third step stamps step index 2");
}

/// Gating: when `set_record_tick_durations` is NOT called, every emitted
/// trace entry has `duration_ns == 0` (no `Instant` measurement, zero tax).
#[test]
fn test_duration_ns_zero_by_default() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    for name in &["alpha", "beta"] {
        let (cb, _) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: name.to_string(),
                policy: period(10),
                callback: cb,
            })
            .unwrap();
    }

    // Step so both fire several times.
    scheduler.step_ms(30);

    let trace = scheduler.trace();
    assert!(!trace.is_empty(), "nodes should have fired");
    for entry in trace {
        assert_eq!(
            entry.duration_ns, 0,
            "duration recording is OFF by default ⇒ duration_ns must be 0"
        );
    }
}

/// When recording is enabled, a tick that sleeps a real ≥2ms records a
/// `duration_ns` of at least 1ms (liveness; sleep guarantees at-least, so
/// non-flaky). Fires exactly once via a single `External` trigger.
#[test]
fn test_duration_ns_recorded_when_enabled() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let cb: Box<dyn FnMut() + Send> = Box::new(move || {
        std::thread::sleep(Duration::from_millis(2));
    });
    scheduler
        .add_node(NodeConfig {
            id: "slow".to_string(),
            policy: TriggerPolicy::External,
            callback: cb,
        })
        .unwrap();

    // Enable AFTER add_node — the setter must reach existing nodes too.
    scheduler.set_record_tick_durations(true);

    scheduler.trigger_external("slow").unwrap();
    scheduler.step_ms(10);

    let trace = scheduler.trace();
    assert_eq!(trace.len(), 1, "External node fires exactly once");
    assert!(
        trace[0].duration_ns >= 1_000_000,
        "a ≥2ms sleeping tick should record duration_ns ≥ 1ms, got {}ns",
        trace[0].duration_ns
    );
}

/// Determinism firewall: two runs of the same scenario WITH durations on
/// compare EQUAL even though their wall durations differ run-to-run — the
/// `Vec<TraceEntry>` / element `PartialEq` exclusion holds end-to-end.
#[test]
fn test_two_runs_compare_equal_despite_differing_durations() {
    fn run_scenario() -> Vec<TraceEntry> {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);

        let cb: Box<dyn FnMut() + Send> = Box::new(move || {
            std::thread::sleep(Duration::from_millis(1));
        });
        scheduler
            .add_node(NodeConfig {
                id: "sensor".to_string(),
                policy: period(10),
                callback: cb,
            })
            .unwrap();

        scheduler.set_record_tick_durations(true);

        // Fire a few times so the trace has multiple entries.
        scheduler.step_ms(30);

        scheduler.trace().to_vec()
    }

    let trace1 = run_scenario();
    let trace2 = run_scenario();

    assert!(!trace1.is_empty(), "scenario should fire the period node");
    // The equality must be MEANINGFUL: durations were genuinely recorded
    // (else trace1==trace2 would be a vacuous "both all-zero" pass).
    assert!(
        trace1.iter().any(|e| e.duration_ns > 0),
        "durations must actually be recorded when enabled (else equality is vacuous)"
    );
    // Equality MUST hold regardless of per-run wall-duration jitter — the
    // determinism firewall (duration_ns excluded from PartialEq).
    assert_eq!(
        trace1, trace2,
        "traces must compare equal despite run-to-run duration_ns differences"
    );
    // Anti-tautology (deterministic, not jitter-dependent): mutate a copy's
    // duration_ns by a huge delta and it STILL compares equal — proving the
    // exclusion is load-bearing even when durations genuinely differ.
    let mut mutated = trace1.clone();
    mutated[0].duration_ns = mutated[0].duration_ns.wrapping_add(1_000_000_000);
    assert_eq!(
        trace1, mutated,
        "a wildly different duration_ns must NOT break TraceEntry equality"
    );
}

/// `add_node` inheritance branch (mod.rs `add_node` seeding
/// `record_durations: self.record_tick_durations`): enabling recording
/// BEFORE any node is added must make a LATER-added node record durations
/// too. Proves the master gate is sticky across `add_node`, not only the
/// retroactive `set_record_tick_durations` loop over existing nodes.
#[test]
fn test_duration_inherited_when_enabled_before_add_node() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    // Enable BEFORE adding any node — the master gate must seed future nodes.
    scheduler.set_record_tick_durations(true);

    let cb: Box<dyn FnMut() + Send> = Box::new(move || {
        std::thread::sleep(Duration::from_millis(2));
    });
    scheduler
        .add_node(NodeConfig {
            id: "late".to_string(),
            policy: TriggerPolicy::External,
            callback: cb,
        })
        .unwrap();

    scheduler.trigger_external("late").unwrap();
    scheduler.step_ms(10);

    let trace = scheduler.trace();
    assert_eq!(trace.len(), 1, "External node fires exactly once");
    assert!(
        trace[0].duration_ns >= 1_000_000,
        "a node added AFTER enabling must inherit recording: ≥2ms sleep ⇒ \
         duration_ns ≥ 1ms, got {}ns",
        trace[0].duration_ns
    );
}

/// Merged single-`Instant` refactor (`fire_node_into`): the ONE `Instant`
/// captured when `tick_within_ns.is_some() || record_durations` feeds BOTH
/// the `tick_within` budget check AND `TraceEntry::duration_ns`. With both
/// enabled, a tick that overruns its 1ms budget must (a) bump the
/// per-node tick-within miss counter AND (b) record a real `duration_ns` —
/// proving the two consumers share one measurement, neither path lost.
#[test]
fn test_tick_within_and_duration_both_recorded() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let cb: Box<dyn FnMut() + Send> = Box::new(move || {
        std::thread::sleep(Duration::from_millis(5));
    });
    let handle = scheduler
        .add_node(NodeConfig {
            id: "overrun".to_string(),
            policy: TriggerPolicy::External,
            callback: cb,
        })
        .unwrap();

    // 1ms budget; the 5ms sleep overruns it. Both gates on.
    scheduler.set_tick_within("overrun", 1).unwrap();
    scheduler.set_record_tick_durations(true);

    scheduler.trigger_external("overrun").unwrap();
    scheduler.step_ms(10);

    let trace = scheduler.trace();
    assert_eq!(trace.len(), 1, "External node fires exactly once");
    assert!(
        handle.tick_within_missed_count() > 0,
        "a 5ms tick over a 1ms budget must bump the tick-within miss counter"
    );
    assert!(
        trace[0].duration_ns >= 1_000_000,
        "the SAME fire must also record duration_ns ≥ 1ms (≥5ms sleep), got {}ns",
        trace[0].duration_ns
    );
}

/// Gated-contract pin (the `duration_ns` write is gated on `record_durations`
/// SPECIFICALLY, not on whether the `Instant` was taken): a `tick_within_ms`
/// node captures an `Instant` for its BUDGET check, but with B-dur recording
/// OFF its `duration_ns` MUST stay 0. Proves the `tick_within` measurement
/// does not leak into the telemetry field — "0 unless recording on" holds even
/// for budgeted nodes, and a non-zero `duration_ns` always implies recording
/// was on. (Without the gate, this node would emit a real ~5ms duration.)
#[test]
fn test_tick_within_only_does_not_populate_duration_when_recording_off() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let cb: Box<dyn FnMut() + Send> = Box::new(move || {
        std::thread::sleep(Duration::from_millis(5));
    });
    let handle = scheduler
        .add_node(NodeConfig {
            id: "budgeted".to_string(),
            policy: TriggerPolicy::External,
            callback: cb,
        })
        .unwrap();

    // tick_within budget ON (captures an Instant) but B-dur recording OFF.
    scheduler.set_tick_within("budgeted", 1).unwrap();

    scheduler.trigger_external("budgeted").unwrap();
    scheduler.step_ms(10);

    let trace = scheduler.trace();
    assert_eq!(trace.len(), 1, "External node fires exactly once");
    // The budget path still measured + flagged the overrun...
    assert!(
        handle.tick_within_missed_count() > 0,
        "the tick_within budget check must still fire (the Instant IS taken)"
    );
    // ...but duration_ns is gated OFF → 0, despite the ~5ms tick.
    assert_eq!(
        trace[0].duration_ns, 0,
        "duration_ns must be 0 when B-dur recording is off, even though the \
         tick_within budget measured a ~5ms tick; got {}ns",
        trace[0].duration_ns
    );
}

// ---------------------------------------------------------------------------
// `restore_period_schedule`
//
// A resumed replay places the clock at the anchor instant and then restores
// each `Period` node's deadline. The recording can NAME that deadline only for
// a node that fires in the recorded suffix; a review after the fold found
// that a node with no fire in it was left on the deadline `add_node` derived at
// build time — which sits BELOW the anchor instant and fires on the first
// resumed step. These pin both halves against a hand-computed oracle.
//
// # Every named baseline is PHASE-DISTINGUISHING, deliberately
//
// The two branches produce the SAME number on a carelessly chosen fixture, and
// an oracle that cannot separate them pins nothing. A 10 ms node built at the
// origin and re-phased past a 105 ms anchor lands on 110 ms — so a recorded
// baseline of 110 ms is satisfied by an implementation that DISCARDS the
// recording entirely, which is the whole behaviour under test. Every baseline
// below is therefore off the node's own origin-phased grid:
//
//   * `named` records 109 ms — the 10 ms grid from the origin never lands
//     there, so adopting it and re-phasing to it are distinct outcomes.
//   * `named_at_anchor` records 105 ms, EXACTLY the anchor instant (the
//     ordinary shape of a node whose first recorded fire is the boundary the
//     resume rendezvouses on). Adoption leaves it due immediately; a stray
//     re-phase AFTER adoption — dropping the `continue` — would advance it to
//     115 ms, and the node would miss the fire the recording shows.
//
// Discarding `baselines` and adopting-then-re-phasing
// each fail this test, and each passed it before the baselines were re-chosen.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn restore_period_schedule_sets_a_named_node_and_re_phases_an_unnamed_one() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock.clone());

    let (named_cb, named_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "named".to_string(),
            policy: period(10),
            callback: named_cb,
        })
        .unwrap();
    let (anchor_cb, anchor_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "named_at_anchor".to_string(),
            policy: period(10),
            callback: anchor_cb,
        })
        .unwrap();
    let (unnamed_cb, unnamed_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "unnamed".to_string(),
            policy: period(30),
            callback: unnamed_cb,
        })
        .unwrap();
    // Built at the clock's origin, so every deadline is one interval in.
    assert_eq!(scheduler.ns_until_next_fire(0), Some(10_000_000));

    // The resume: the clock is placed at the anchor instant FIRST (105 ms — the
    // engine reads it off the recording's first boundary), then the schedule is
    // restored with only the nodes the recording names.
    clock.set(105_000_000);
    let mut baselines = std::collections::BTreeMap::new();
    baselines.insert("named".to_string(), 109_000_000u64);
    baselines.insert("named_at_anchor".to_string(), 105_000_000u64);
    scheduler.restore_period_schedule(&baselines);

    // Hand oracle for the post-restore schedule:
    //   named            109 ms  (recorded, verbatim — NOT the 110 ms a 10 ms
    //                             re-phase from the origin would produce)
    //   named_at_anchor  105 ms  (recorded, verbatim — due at once, NOT the
    //                             115 ms a re-phase past `now` would produce)
    //   unnamed          120 ms  (un-named: 30, 60, 90, 120 — exactly where the
    //                             live run's schedule stood at 105 ms)
    assert_eq!(
        scheduler.ns_until_next_fire(105_000_000),
        Some(0),
        "`named_at_anchor`'s recorded baseline IS the anchor instant, so it is \
         due immediately; a re-phase past `now` would push it to 115 ms"
    );
    // 106 ms: only the node whose recorded deadline already passed fires.
    scheduler.step_ms(1);
    assert_eq!(
        anchor_count.load(Ordering::Relaxed),
        1,
        "the node recorded as due AT the anchor must fire on the first resumed \
         step — 0 here means its recorded baseline was not adopted verbatim"
    );
    assert_eq!(named_count.load(Ordering::Relaxed), 0);
    assert_eq!(unnamed_count.load(Ordering::Relaxed), 0);
    // 108 ms: nothing is due (109 / 115 / 120 all still ahead).
    scheduler.step_ms(2);
    assert_eq!(named_count.load(Ordering::Relaxed), 0);
    assert_eq!(anchor_count.load(Ordering::Relaxed), 1);
    assert_eq!(
        unnamed_count.load(Ordering::Relaxed),
        0,
        "the un-named node must NOT fire on the first resumed steps — a \
         build-time deadline of 30 ms would have burst through 30/60/90 here"
    );
    // 109 ms: `named`'s RECORDED deadline arrives. A discarded baseline would
    // have re-phased it to 110 ms, so a 0 here is that discard.
    scheduler.step_ms(1);
    assert_eq!(
        named_count.load(Ordering::Relaxed),
        1,
        "`named` must fire at its recorded 109 ms, not at the 110 ms a \
         re-phase from the origin produces"
    );
    assert_eq!(unnamed_count.load(Ordering::Relaxed), 0);
    // 110 ms: the instant a discarded baseline WOULD have fired at. `named` is
    // now on 119 ms (109 + 10), so it must stay put — the other side of the
    // same phase pin.
    scheduler.step_ms(1);
    assert_eq!(
        named_count.load(Ordering::Relaxed),
        1,
        "`named` must NOT fire again at 110 ms — its phase is the recorded \
         one (109, 119, ...), not the origin's (110, 120, ...)"
    );
    assert_eq!(anchor_count.load(Ordering::Relaxed), 1);
    // 115 ms: `named_at_anchor`'s own next interval (105 + 10).
    scheduler.step_ms(5);
    assert_eq!(anchor_count.load(Ordering::Relaxed), 2);
    assert_eq!(named_count.load(Ordering::Relaxed), 1);
    assert_eq!(unnamed_count.load(Ordering::Relaxed), 0);
    // 119 ms: `named` again, and `unnamed` is still one millisecond short of
    // its re-phased 120 ms — the lower side of that deadline.
    scheduler.step_ms(4);
    assert_eq!(named_count.load(Ordering::Relaxed), 2);
    assert_eq!(
        unnamed_count.load(Ordering::Relaxed),
        0,
        "the un-named node's re-phased deadline is exactly 120 ms, not 119"
    );
    // 120 ms: `unnamed`'s re-phased deadline arrives, exactly once.
    scheduler.step_ms(1);
    assert_eq!(unnamed_count.load(Ordering::Relaxed), 1);
    assert_eq!(named_count.load(Ordering::Relaxed), 2);
    assert_eq!(anchor_count.load(Ordering::Relaxed), 2);
}

// ---------------------------------------------------------------------------
// The re-phase is O(1) ARITHMETIC, not a loop.
//
// `restore_period_schedule` advanced an un-named `Period` node with
// `while *next_fire <= now { *next_fire += interval }` — O((now − next_fire) /
// interval) ITERATIONS. That was fine while the only caller was checkpoint
// resume, where `now` is an anchor a few seconds above a build-time deadline.
// Free-run replay added a second caller and broke the assumption: a FREE-RUN bag's
// per-rank clocks are WALL-FAITHFUL, so a from-start free-run replay places the
// clock on the recording's first boundary — ~1.7e18 ns since the epoch — and
// then calls this. A 1 ms node the recording never fired inside its window is
// exactly the un-named case, i.e. ~1.7e12 iterations. That is a HANG, not a
// slow path.
//
// HOW THIS IS PINNED, and the trade:
// a regression turns COMPLETION into NON-COMPLETION, which libtest cannot assert
// on — a `#[should_panic]` cannot catch a hang, and a wall-clock budget tight
// enough to separate "one multiply" from "1.7e12 adds" is the load-inversion class
// that a loaded runner inverts. So the ORACLE is the VALUE: the exact deadline
// the arithmetic must produce, hand-computed from the epoch and the interval.
// Under the correct code it is instant; under the broken loop the test never
// reaches its first assertion at all, and the FAILURE MODE IS THE HANG ITSELF
// (verified by reasoning about the iteration count, not by running it — running
// it is what the fix exists to prevent). The value oracle additionally catches
// every WRONG-ANSWER regression, which a wall gate never could: an off-by-one in
// the step count, a `<` for `<=`, a phase-losing `now + interval`.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn restore_period_schedule_re_phases_past_a_wall_epoch_clock_in_one_step() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock.clone());

    // A 1 ms node — the free-run shape: a control-rate node the recorded window
    // never fired, so the engine's `first_recorded_fire_times` names it not at
    // all and it takes the re-phase arm.
    let (fast_cb, fast_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fast".to_string(),
            policy: period(1),
            callback: fast_cb,
        })
        .unwrap();
    // A 33 ms sibling (~30 Hz), likewise un-named, so the arm is exercised at
    // two interval scales in one restore.
    //
    // 33 rather than a tidy 30 is LOAD-BEARING, and it is what makes the phase
    // half of this test mean anything: the wall epoch below is an exact multiple
    // of 1 ms AND of 30 ms, so on either of those grids the phase-preserving
    // answer and a phase-DISCARDING `now + interval` COINCIDE, and no assertion
    // over them can tell the two apart. 33 ms does not divide it
    // (1_755_000_000_000_000_000 = 27 ms mod 33 ms), so the correct deadline is
    // WALL + 6 ms while `now + interval` gives WALL + 33 ms.
    let (slow_cb, _slow_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "slow".to_string(),
            policy: period(33),
            callback: slow_cb,
        })
        .unwrap();
    // Built at the clock's ORIGIN: both deadlines are one interval in, which is
    // what makes them sit ~1.7e18 ns BELOW the clock a moment from now.
    assert_eq!(scheduler.ns_until_next_fire(0), Some(1_000_000));

    // A REAL wall epoch — `SystemTime::UNIX_EPOCH`-scale nanoseconds, the value
    // a free-run recording's first STEP_BOUNDARY carries. Not a synthetic large
    // number: this is the magnitude the defect was found at.
    const WALL_EPOCH_NS: u64 = 1_755_000_000_000_000_000;
    clock.set(WALL_EPOCH_NS);

    // No baselines at all: EVERY node takes the re-phase arm.
    let baselines = std::collections::BTreeMap::new();
    scheduler.restore_period_schedule(&baselines);

    // HAND ORACLE. The rule is "advance in whole intervals until strictly past
    // `now`, preserving phase", so the answer is the smallest value above `now`
    // congruent to the build-time deadline modulo the interval:
    //
    //   next = next_fire + (floor((now − next_fire) / interval) + 1) * interval
    //
    // computed here from the constants, independently of the implementation.
    let expect_next = |interval_ns: u64| -> u64 {
        let built = interval_ns; // `add_node` baselines at origin + interval
        let behind = WALL_EPOCH_NS - built;
        built + (behind / interval_ns + 1) * interval_ns
    };
    let fast_next = expect_next(1_000_000);
    let slow_next = expect_next(33_000_000);

    // Both are STRICTLY past the clock, and by LESS than one interval — the two
    // halves of "the earliest un-fired deadline". These BOUNDS on their own can
    // catch NO phase regression: `now + interval` lands exactly one interval past
    // `now` and satisfies both, on every node. What catches it is the EXACT
    // value asserted against the SCHEDULER below — and only on `slow`, whose
    // grid misses the epoch.
    assert!(fast_next > WALL_EPOCH_NS && fast_next - WALL_EPOCH_NS <= 1_000_000);
    assert!(slow_next > WALL_EPOCH_NS && slow_next - WALL_EPOCH_NS <= 33_000_000);

    assert_eq!(
        scheduler.ns_until_next_fire(WALL_EPOCH_NS),
        Some(fast_next - WALL_EPOCH_NS),
        "the 1 ms node's deadline is the hand-computed whole-interval advance — \
         under a one-interval-at-a-time loop this line is never reached (~1.7e12 iterations)"
    );

    // The SLOW node's restored deadline, read back off the scheduler ITSELF.
    // `ns_until_next_fire` reports the MINIMUM over every node, so it can only
    // ever speak for `fast` — and `fast`'s grid passes through the epoch, so its
    // answer is phase-blind. Without this line `slow_next` is computed and then
    // compared against nothing, and the one node that can discriminate a
    // phase-discarding advance is never asked.
    assert_eq!(
        scheduler
            .node_framework_state("slow")
            .expect("the slow node is in this scheduler")
            .next_fire_ns,
        Some(slow_next),
        "the 33 ms node's deadline PRESERVES its phase — WALL_EPOCH_NS + {} ns, \
         not the WALL_EPOCH_NS + 33 ms a phase-discarding `now + interval` produces",
        slow_next - WALL_EPOCH_NS
    );

    // PHASE PRESERVED, stated as the property rather than as one number: each
    // deadline is congruent to its build-time deadline modulo its interval.
    assert_eq!(fast_next % 1_000_000, 1_000_000 % 1_000_000);
    assert_eq!(slow_next % 33_000_000, 33_000_000 % 33_000_000);

    // …and the schedule really RUNS from there: advancing to the 1 ms node's
    // re-phased deadline FIRES it exactly once, and leaves it due one interval
    // later. Without this the value assertions describe a field nothing reads.
    scheduler.step(Duration::from_nanos(fast_next - WALL_EPOCH_NS));
    assert_eq!(
        fast_count.load(Ordering::Relaxed),
        1,
        "the re-phased deadline is a REAL deadline: advancing to it fires the node ONCE \
         (a burst here would mean the re-phase left the deadline below the clock — the \
         catch-up storm this fix exists to prevent)"
    );
    assert_eq!(
        scheduler.ns_until_next_fire(fast_next),
        Some(1_000_000),
        "after firing at its re-phased deadline the node is due one interval later"
    );

    // ANTI-TAUTOLOGY: a node whose deadline is ALREADY past `now` is a NO-OP —
    // the arithmetic must not advance it by a spurious extra interval. Built
    // fresh so its baseline is clean.
    let mut scheduler2 = Scheduler::with_virtual_clock(clock.clone());
    let (ahead_cb, _ahead_count) = counting_callback();
    scheduler2
        .add_node(NodeConfig {
            id: "ahead".to_string(),
            policy: period(1),
            callback: ahead_cb,
        })
        .unwrap();
    // Built at `fast_next` (the clock's current value), so its deadline is
    // `fast_next + 1 ms` — already strictly past `now`.
    scheduler2.restore_period_schedule(&std::collections::BTreeMap::new());
    assert_eq!(
        scheduler2.ns_until_next_fire(fast_next),
        Some(1_000_000),
        "a deadline already past `now` is left ALONE — an unconditional advance \
         would push it to 2 ms"
    );

    // EQUALITY BOUNDARY. The rule is "advance until STRICTLY past `now`", so a
    // deadline sitting EXACTLY on `now` is behind, not ahead, and must be pushed
    // one whole interval.
    //
    // The arm above cannot reach this and its `<`-for-`<=` claim was empty: its
    // node is built while the clock already sits at `fast_next`, so its deadline
    // is `fast_next + 1 ms` — comfortably in the FUTURE, where `<` and `<=` are
    // both no-ops and agree. Only `next_fire == now` separates them, so the
    // clock is moved ONTO the deadline here.
    let mut scheduler3 = Scheduler::with_virtual_clock(clock.clone());
    let (edge_cb, edge_count) = counting_callback();
    let built_at = clock.now_ns();
    scheduler3
        .add_node(NodeConfig {
            id: "edge".to_string(),
            policy: period(1),
            callback: edge_cb,
        })
        .unwrap();
    // `add_node` baselines at `clock.now_ns() + interval`, so this is the node's
    // deadline to the nanosecond.
    let on_the_deadline = built_at + 1_000_000;
    clock.set(on_the_deadline);
    scheduler3.restore_period_schedule(&std::collections::BTreeMap::new());
    assert_eq!(
        scheduler3
            .node_framework_state("edge")
            .expect("the edge node is in this scheduler")
            .next_fire_ns,
        Some(on_the_deadline + 1_000_000),
        "a deadline sitting EXACTLY on `now` is advanced one whole interval — a \
         `<` for `<=` slip leaves it AT `now`, i.e. already due"
    );
    assert_eq!(
        scheduler3.ns_until_next_fire(on_the_deadline),
        Some(1_000_000),
        "…and the live loop's own wake-sizing seam agrees: one whole interval of \
         headroom, never the `Some(0)` due-NOW a `<` for `<=` slip reports"
    );
    // Behavioural half: the advanced deadline is a REAL one — nothing fires
    // until the clock reaches it, and then exactly once. Under `<` the node is
    // due at `now` and this first sub-interval step already fires it.
    scheduler3.step(Duration::from_nanos(999_999));
    assert_eq!(
        edge_count.load(Ordering::Relaxed),
        0,
        "the re-phased edge node is NOT due before its advanced deadline"
    );
    scheduler3.step(Duration::from_nanos(1));
    assert_eq!(
        edge_count.load(Ordering::Relaxed),
        1,
        "…and fires exactly once when the clock reaches it"
    );
}

#[test]
#[serial]
fn restore_period_schedule_leaves_a_non_period_node_and_an_unknown_id_alone() {
    // The anti-tautology half: the sweep must not invent a deadline for a
    // trigger that carries no timing state, and must not fire one.
    //
    // A `Period` control rides in the SAME scheduler, and it is what makes this
    // an anti-tautology test rather than a vacuous one. Without it the whole
    // body of `restore_period_schedule` could be `return;` and every assertion
    // below would still hold — `ns_until_next_fire` reads `next_fire_ns` only
    // on the `Period` arm, so a scheduler holding nothing else reports `None`
    // however the sweep behaved, and an `External` node fires from
    // `trigger_external` rather than from a deadline. The control's adopted
    // baseline is the one observable that proves the sweep RAN.
    //
    // Its 505 ms baseline is off its own origin-phased 10 ms grid (which passes
    // through 500 and 510) for the same reason as the test above: a
    // grid-aligned value is satisfied by an implementation that discards the
    // recording.
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock.clone());
    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "external".to_string(),
            policy: TriggerPolicy::External,
            callback: cb,
        })
        .unwrap();
    let (ticker_cb, ticker_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "ticker".to_string(),
            policy: period(10),
            callback: ticker_cb,
        })
        .unwrap();

    clock.set(500_000_000);
    // The recording names all three classes the doc calls out: a `Period` node
    // (adopted), a non-`Period` node (left alone — the past-dated instant is
    // the shape that would fire it if a deadline were stamped on), and an id
    // this graph does not contain (a no-op, not a panic).
    let mut baselines = std::collections::BTreeMap::new();
    baselines.insert("ticker".to_string(), 505_000_000u64);
    baselines.insert("external".to_string(), 400_000_000u64);
    baselines.insert("ghost".to_string(), 450_000_000u64);
    scheduler.restore_period_schedule(&baselines);

    assert_eq!(
        scheduler.ns_until_next_fire(500_000_000),
        Some(5_000_000),
        "only the Period control contributes a deadline, and it is the recorded \
         505 ms — a discarded baseline would re-phase it to 510 ms, and a sweep \
         that never ran would leave it at the build-time 10 ms"
    );
    // 505 ms: the control fires on its recorded deadline; the External node,
    // whose recorded baseline is 100 ms in the PAST, does not.
    scheduler.step_ms(5);
    assert_eq!(ticker_count.load(Ordering::Relaxed), 1);
    assert_eq!(count.load(Ordering::Relaxed), 0);
    // A full second later the External node has still never fired: it holds no
    // deadline, so no amount of clock advance can reach one.
    scheduler.step_ms(1000);
    assert_eq!(
        count.load(Ordering::Relaxed),
        0,
        "a non-Period node must never acquire a deadline from the restore, \
         however the recording names it"
    );
}

// ---------------------------------------------------------------------------
// The FRAMEWORK SECTION: capture and restore
//
// `ScheduledNode` is not serializable (a boxed callback and shared `Arc`s),
// but three of its fields are plain data nothing else in a recording states:
// `next_fire_ns`, `pending_data_count` and `sync_input_timestamps`. These pin
// the seam that reads them off a live scheduler and the seam that puts the
// third one back, both against hand-computed oracles.
// ---------------------------------------------------------------------------

#[test]
#[serial]
fn the_framework_section_reports_each_triggers_own_plain_data() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (period_cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "camera".to_string(),
            policy: period(16),
            callback: period_cb,
        })
        .unwrap();
    let (data_cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "detector".to_string(),
            policy: TriggerPolicy::Data,
            callback: data_cb,
        })
        .unwrap();
    let (sync_cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["camera".to_string(), "imu".to_string()],
                window: Some(Duration::from_millis(50)),
            },
            callback: sync_cb,
        })
        .unwrap();
    let (ext_cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "driver".to_string(),
            policy: TriggerPolicy::External,
            callback: ext_cb,
        })
        .unwrap();

    // Drive each trigger to a DISTINCT, hand-known state. `detector` is
    // signalled three times and never stepped, so its count is still standing
    // — the shape a `block`/`throttle` defer or a tripped breaker leaves at a
    // real boundary.
    for _ in 0..3 {
        scheduler.signal_data("detector").unwrap();
    }
    scheduler
        .signal_sync_input("fusion", "camera", 7_000_000)
        .unwrap();

    let camera = scheduler
        .node_framework_state("camera")
        .expect("known node");
    assert_eq!(
        camera.next_fire_ns,
        Some(16_000_000),
        "a Period node built on a clock at its origin is due one interval in"
    );
    assert_eq!(camera.pending_data_count, 0);
    assert!(camera.sync_input_timestamps.is_empty());

    let detector = scheduler
        .node_framework_state("detector")
        .expect("known node");
    assert_eq!(
        detector.next_fire_ns, None,
        "a Data trigger carries no timing state, and None must not read as \
         Some(0) — a deadline of zero is due immediately"
    );
    assert_eq!(detector.pending_data_count, 3);

    let fusion = scheduler
        .node_framework_state("fusion")
        .expect("known node");
    assert_eq!(fusion.next_fire_ns, None);
    assert_eq!(fusion.pending_data_count, 0);
    assert_eq!(
        fusion.sync_input_timestamps,
        [("camera".to_string(), 7_000_000u64)]
            .into_iter()
            .collect::<std::collections::BTreeMap<_, _>>(),
        "the half-satisfied alignment §6.5's worked example is about: /cam \
         arrived, /imu did not"
    );

    let driver = scheduler
        .node_framework_state("driver")
        .expect("known node");
    assert_eq!(
        driver,
        Default::default(),
        "an External trigger states nothing"
    );
    assert!(driver.is_empty());

    assert_eq!(
        scheduler.node_framework_state("not-in-this-graph"),
        None,
        "an unknown id is None, not a fabricated empty section"
    );

    // And the values TRACK the scheduler rather than being read once at build:
    // one step serves the whole signalled backlog (per-message FIFO
    // firing — one fire per arrival, within the step) and advances the
    // Period deadline.
    scheduler.step_ms(16);
    assert_eq!(
        scheduler
            .node_framework_state("detector")
            .expect("known node")
            .pending_data_count,
        0,
        "the step served all three arrivals; nothing carries"
    );
    assert_eq!(
        scheduler
            .node_framework_state("camera")
            .expect("known node")
            .next_fire_ns,
        Some(32_000_000)
    );
}

#[test]
#[serial]
fn restore_sync_input_timestamps_sets_a_named_node_and_leaves_the_rest_alone() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (named_cb, named_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["camera".to_string(), "lidar".to_string()],
                window: Some(Duration::from_millis(50)),
            },
            callback: named_cb,
        })
        .unwrap();
    let (unnamed_cb, unnamed_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "other_fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["camera".to_string(), "lidar".to_string()],
                window: Some(Duration::from_millis(50)),
            },
            callback: unnamed_cb,
        })
        .unwrap();
    let (period_cb, period_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "ticker".to_string(),
            policy: period(10),
            callback: period_cb,
        })
        .unwrap();

    // The recording says /camera had arrived at the
    // anchor and /lidar had not. `other_fusion` is NOT named — the recording
    // states nothing about it — and `ticker` is a non-Sync node the sweep must
    // not write into.
    let mut per_node = std::collections::BTreeMap::new();
    per_node.insert(
        "fusion".to_string(),
        [("camera".to_string(), 20_000_000u64)]
            .into_iter()
            .collect::<std::collections::BTreeMap<_, _>>(),
    );
    per_node.insert(
        "ticker".to_string(),
        [("camera".to_string(), 20_000_000u64)]
            .into_iter()
            .collect::<std::collections::BTreeMap<_, _>>(),
    );
    scheduler.restore_sync_input_timestamps(&per_node);

    assert_eq!(
        scheduler
            .node_framework_state("fusion")
            .expect("known node")
            .sync_input_timestamps,
        [("camera".to_string(), 20_000_000u64)]
            .into_iter()
            .collect::<std::collections::BTreeMap<_, _>>()
    );
    assert!(
        scheduler
            .node_framework_state("other_fusion")
            .expect("known node")
            .sync_input_timestamps
            .is_empty(),
        "a node the recording does not name is left as built"
    );
    assert!(
        scheduler
            .node_framework_state("ticker")
            .expect("known node")
            .sync_input_timestamps
            .is_empty(),
        "a Period node must not acquire sync state from the restore, however \
         the recording names it"
    );

    // The BEHAVIOURAL half, which is the whole point: /lidar alone now
    // completes the restored alignment, while the un-named node still needs
    // both. Without the restore `fusion` would wait for a second /camera the
    // recording never sent.
    scheduler
        .signal_sync_input("fusion", "lidar", 25_000_000)
        .unwrap();
    scheduler
        .signal_sync_input("other_fusion", "lidar", 25_000_000)
        .unwrap();
    scheduler.step_ms(30);
    assert_eq!(
        named_count.load(Ordering::Relaxed),
        1,
        "the restored /camera arrival must satisfy its half of the window"
    );
    assert_eq!(
        unnamed_count.load(Ordering::Relaxed),
        0,
        "the un-named node still holds only /lidar"
    );
    assert_eq!(period_count.load(Ordering::Relaxed), 3, "10ms over 30ms");
}

#[test]
#[serial]
fn a_restored_alignment_still_respects_the_sync_window() {
    // The restore puts back a TIMESTAMP, not a fire: a recorded arrival far
    // outside the window must not make the node fire on an arrival that is
    // nowhere near it. The anti-tautology twin of the arm above — without it,
    // an implementation that restored a "this input is satisfied" FLAG instead
    // of the instant would pass.
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["camera".to_string(), "lidar".to_string()],
                window: Some(Duration::from_millis(10)),
            },
            callback: cb,
        })
        .unwrap();

    let mut per_node = std::collections::BTreeMap::new();
    per_node.insert(
        "fusion".to_string(),
        [("camera".to_string(), 0u64)]
            .into_iter()
            .collect::<std::collections::BTreeMap<_, _>>(),
    );
    scheduler.restore_sync_input_timestamps(&per_node);

    scheduler
        .signal_sync_input("fusion", "lidar", 100_000_000)
        .unwrap();
    scheduler.step_ms(100);
    assert_eq!(
        count.load(Ordering::Relaxed),
        0,
        "a restored arrival 100ms before a 10ms window must not align"
    );
}

// ===========================================================================
// The TRACE-DRIVEN fire plan.
//
// Under free-run, replay drives fires FROM THE RECORDING and re-derivation
// becomes the verifier's job. These arms drive the plan through the
// FLAT `Scheduler::step` seam — no transport, no graph — because the property
// under test is the scheduler's: "a plan-driven scheduler never consults a
// trigger". The level seams (`decide_fires`, `evaluate_nodes_fused`) route
// through the SAME `take_planned_fire` body, and the block-fused half's own
// behavioural pin lives in `backpressure_block_iox2_test.rs` (it needs a real
// `block` edge).
// ===========================================================================

/// **Every `Period` deadline advance in the scheduler DELEGATES to the one
/// implementation — pinned STRUCTURALLY, because its failure shapes hang.**
///
/// `catchup_clamp::period_advance` replaced three hand-written advances. Two of
/// them (`reset_node`, and the forward arm of `restore_period_schedule`) drive
/// it at `cap == 0` — "carry the deadline strictly past `now`, mint no fires".
///
/// A behavioural pin is not available for the two shapes that motivated the
/// extraction, and that is the whole reason this walk exists:
///
/// * `interval_ns == 0` makes the loop spin FOREVER (the deadline never moves
///   past `now`), and
/// * a wall-faithful clock (~1.7e18 ns) against a 1 ms interval is ~1.7e12
///   iterations.
///
/// Both HANG rather than fail, so a test that exercised them would wedge CI
/// instead of reporting. `catchup_clamp`'s own oracle asserts the O(1) answers
/// for exactly those shapes directly; what cannot be checked there is whether
/// the SCHEDULER still calls it. This is that check.
///
/// Restoring either `while *next_fire <= now { *next_fire += i }`
/// fails here. It passes every behavioural `reset_node` arm, because the loop
/// and the O(1) form agree wherever the loop terminates — which is every shape
/// a test can safely run.
#[test]
fn every_period_deadline_advance_delegates_to_the_shared_implementation() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/scheduler/mod.rs"))
        .expect("the scheduler's own source");

    /// The function body, from its signature to the next item at the same
    /// indent, with `//` comments stripped — the docs here NAME the loop this
    /// walk forbids, so an unstripped view would flag its own explanation.
    fn body_of<'a>(src: &'a str, signature: &str) -> &'a str {
        let start = src
            .find(signature)
            .unwrap_or_else(|| panic!("the walk must find `{signature}`"));
        let rest = &src[start..];
        let end = rest[1..]
            .find("\n    pub fn ")
            .or_else(|| rest[1..].find("\n    /// "))
            .map_or(rest.len(), |i| i + 1);
        &rest[..end]
    }
    fn code_only(body: &str) -> String {
        body.lines()
            .map(|l| l.split_once("//").map_or(l, |(code, _)| code))
            .collect::<Vec<_>>()
            .join("\n")
    }

    for sig in ["pub fn reset_node(", "pub fn restore_period_schedule("] {
        let body = code_only(body_of(&src, sig));
        assert!(
            body.contains("catchup_clamp::period_advance("),
            "`{sig}` must advance through the ONE implementation, not its own arithmetic"
        );
        assert!(
            !body.contains("+= interval_ns"),
            "`{sig}` must not carry a hand-written running add:\n{body}"
        );
    }

    // ANTI-TAUTOLOGY: the walk must really be reading bodies, or every
    // assertion above is satisfied by an empty string.
    assert!(
        code_only(body_of(&src, "pub fn reset_node(")).contains("consecutive_panics = 0"),
        "the extracted body must be `reset_node`'s own"
    );
    assert!(
        code_only(body_of(&src, "pub fn restore_period_schedule(")).contains("baselines.get("),
        "the extracted body must be `restore_period_schedule`'s own"
    );
}

/// Removing a node keeps the installed fire plan ALIGNED with
/// the nodes that remain.**
///
/// The plan is keyed by node INDEX and `remove_node` is a `shift_remove`, so
/// every later node moves down by one. A plan left as installed would pair
/// the node AFTER the removed one with the removed node's slot and leave the
/// last node's slot naming nothing — a fire the recording holds, lost with no
/// report (`unconsumed_replay_fires` filters an index past the node map).
/// Three nodes, a plan naming only the LAST; remove the MIDDLE; the last
/// node's fire must survive as ITS OWN, both in the unconsumed report before
/// the step and as a real fire in it.
#[test]
#[serial]
fn removing_a_node_keeps_the_fire_plan_aligned_with_the_remaining_nodes() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let mut counts = Vec::new();
    for id in ["alpha", "beta", "gamma"] {
        let (cb, count) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: id.to_string(),
                policy: TriggerPolicy::Data,
                callback: cb,
            })
            .unwrap();
        counts.push(count);
    }
    scheduler
        .set_replay_fire_plan(
            0,
            &[cerulion_core::scheduler::ReplayFire {
                node_id: "gamma",
                first_fire_ns: 4242,
                fire_count: 1,
                interval_ns: 0,
            }],
        )
        .unwrap();
    assert_eq!(
        scheduler.unconsumed_replay_fires(),
        vec!["gamma"],
        "before anything: the plan holds ONE unconsumed fire, gamma's"
    );

    scheduler.remove_node("beta").unwrap();

    // The slot followed gamma down to index 1 — still gamma's, still unconsumed.
    assert_eq!(
        scheduler.unconsumed_replay_fires(),
        vec!["gamma"],
        "after removing the node BEFORE it, gamma's planned fire is still gamma's — a \
         plan left unshifted pairs the slot with no node and reports NOTHING"
    );
    scheduler.step_ms(5);
    assert_eq!(
        (
            counts[0].load(Ordering::Relaxed),
            counts[2].load(Ordering::Relaxed)
        ),
        (0, 1),
        "the step performs gamma's planned fire and nothing else"
    );
    assert!(
        scheduler.unconsumed_replay_fires().is_empty(),
        "…and the plan is fully consumed"
    );
}

/// **The plan is authoritative for EVERY policy, not just `Period`.**
///
/// Every other arm in this section drives `Period` nodes, and `Period` is the
/// one policy whose live decide arm was rewritten for free-run — so an
/// implementation that consulted the plan there and fell through to the trigger
/// on `Data` / `Sync` would pass all of them. Both halves are asserted in ONE
/// body, in BOTH directions:
///
/// * a `Data` node with an UNCONSUMED arrival and a `Sync` node whose inputs are
///   ALIGNED (both of which the live trigger would fire) do NOT fire, because
///   an EMPTY plan is a step with no fires; and
/// * on the next step, with their trigger state deliberately left un-signalled,
///   they DO fire — at the RECORDED instants, which no re-derivation of either
///   trigger could produce.
#[test]
#[serial]
fn a_trace_driven_plan_is_authoritative_for_data_and_sync_nodes_too() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (data_cb, data_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "relay".to_string(),
            policy: TriggerPolicy::Data,
            callback: data_cb,
        })
        .unwrap();
    let (sync_cb, sync_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["a".to_string(), "b".to_string()],
                window: Some(Duration::from_millis(50)),
            },
            callback: sync_cb,
        })
        .unwrap();

    // Step 0: BOTH triggers are satisfied — an arrival for the Data node, an
    // in-window aligned pair for the Sync node — and the recording holds
    // NOTHING. An empty plan must not fall through to either trigger.
    scheduler.signal_data("relay").unwrap();
    scheduler
        .signal_sync_input("fusion", "a", 1_000_000)
        .unwrap();
    scheduler
        .signal_sync_input("fusion", "b", 1_100_000)
        .unwrap();
    scheduler.set_replay_fire_plan(0, &[]).unwrap();
    scheduler.step_ms(5);
    assert_eq!(
        (
            data_count.load(Ordering::Relaxed),
            sync_count.load(Ordering::Relaxed)
        ),
        (0, 0),
        "an EMPTY plan is a step with no fires for a Data or a Sync node either"
    );

    // Step 1: nothing further is signalled. A trigger-driven scheduler fires
    // neither (the Data arrival is spent, the Sync pair consumed), and the
    // recording says BOTH fired — at instants a re-derivation could not mint.
    scheduler
        .set_replay_fire_plan(
            1,
            &[
                cerulion_core::scheduler::ReplayFire {
                    node_id: "relay",
                    first_fire_ns: 4242,
                    fire_count: 1,
                    interval_ns: 0,
                },
                cerulion_core::scheduler::ReplayFire {
                    node_id: "fusion",
                    first_fire_ns: 9999,
                    fire_count: 1,
                    interval_ns: 0,
                },
            ],
        )
        .unwrap();
    scheduler.clear_trace();
    scheduler.step_ms(5);
    assert_eq!(
        (
            data_count.load(Ordering::Relaxed),
            sync_count.load(Ordering::Relaxed)
        ),
        (1, 1),
        "the plan fires a Data and a Sync node whose triggers say they should not"
    );
    let mut fired: Vec<(String, u64)> = scheduler
        .trace()
        .iter()
        .map(|e| (e.node_id.to_string(), e.fire_time_ns))
        .collect();
    fired.sort();
    assert_eq!(
        fired,
        vec![("fusion".to_string(), 9999), ("relay".to_string(), 4242),],
        "each fire carries the RECORDED instant, not the step clock"
    );
    assert!(scheduler.unconsumed_replay_fires().is_empty());
}

/// The headline, driven in BOTH directions in one body — either alone is
/// satisfiable by a scheduler that ignores the plan on half its inputs:
///
/// * a node the TRIGGER would not fire (3 ms into a 10 ms period) FIRES,
///   because the recording says it did — at the RECORDED instant, not the
///   step's clock; and
/// * on the next step a node the TRIGGER WOULD fire (the period elapses)
///   does NOT, because the recording holds no fire for it.
#[test]
#[serial]
fn a_trace_driven_plan_fires_the_recording_not_the_trigger() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cam_cb, cam_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: cam_cb,
        })
        .unwrap();
    let (imu_cb, imu_count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "imu".to_string(),
            policy: period(10),
            callback: imu_cb,
        })
        .unwrap();

    // Step 0: the recording holds ONE fire, for `cam`, at an instant that is
    // deliberately NOT this step's clock (3 ms) — a re-derived Period fire could
    // never produce 777.
    scheduler
        .set_replay_fire_plan(
            0,
            &[cerulion_core::scheduler::ReplayFire {
                node_id: "cam",
                first_fire_ns: 777,
                fire_count: 1,
                interval_ns: 0,
            }],
        )
        .unwrap();
    assert!(scheduler.is_replay_driven());
    scheduler.step_ms(3);

    assert_eq!(
        cam_count.load(Ordering::Relaxed),
        1,
        "the plan says `cam` fired at step 0 — its 10 ms period says it did not"
    );
    assert_eq!(
        imu_count.load(Ordering::Relaxed),
        0,
        "the plan holds no fire for `imu`"
    );
    let fired: Vec<(String, u64)> = scheduler
        .trace()
        .iter()
        .map(|e| (e.node_id.to_string(), e.fire_time_ns))
        .collect();
    assert_eq!(
        fired,
        vec![("cam".to_string(), 777)],
        "the fire is stamped with the RECORDED instant, not the step clock"
    );
    assert!(scheduler.unconsumed_replay_fires().is_empty());

    // Step 1: the period elapses (3 + 10 = 13 ms ≥ both deadlines), so a
    // trigger-driven scheduler fires BOTH. The recording holds nothing.
    scheduler.set_replay_fire_plan(1, &[]).unwrap();
    scheduler.step_ms(10);
    assert_eq!(
        (
            cam_count.load(Ordering::Relaxed),
            imu_count.load(Ordering::Relaxed)
        ),
        (1, 0),
        "an EMPTY plan is a step with no fires — never a fall-through to the trigger"
    );

    // …and clearing it hands the schedule back to the triggers, which is what
    // shows every non-replay path (and this assertion) that the branch
    // is a branch rather than a one-way door.
    //
    // Both nodes catch up TWO intervals here (deadlines 10 and 20 ms, clock now
    // 23 ms), which is the documented consequence of `take_planned_fire` leaving
    // `next_fire_ns` alone: a plan-driven fire does not advance the Period
    // schedule, because the plan — not the schedule — is authoritative while it
    // is installed. `cam` therefore ends at 1 (planned) + 2 (catch-up).
    scheduler.clear_replay_fire_plan();
    assert!(!scheduler.is_replay_driven());
    scheduler.step_ms(10);
    assert_eq!(
        (
            cam_count.load(Ordering::Relaxed),
            imu_count.load(Ordering::Relaxed)
        ),
        (3, 2),
        "with the plan cleared both periods fire again (each catching up the two \
         intervals that elapsed), and the planned fire did NOT advance the schedule"
    );
}

/// A `FireKind::Replay` burst pays the pre-fire gate ONCE — the reason it is not
/// re-encoded as a synthetic `Period`.
///
/// `tick_node`'s `Period` arm re-evaluates `run_pre_fire_check` BEFORE EACH
/// catch-up fire, and that re-check has counter side effects (the block gate's
/// `block_fires_deferred_count` bump and its once-per-regime warn) plus a
/// `next_fire_ns` rewind on defer. A trace-driven fire must pay neither, so the
/// ORACLE is the gate's own invocation count: exactly 1 for a 3-fire burst
/// (the decide-seam gate), never 1 + 3.
///
/// The fire INSTANTS are the second half: the burst walks the recorded
/// progression by a running add, so 3 fires at 100 / 105 / 110 ns.
#[test]
#[serial]
fn a_replay_burst_pays_the_pre_fire_gate_once_not_once_per_fire() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();
    let gate_calls = Arc::new(AtomicU64::new(0));
    let gate_calls_cb = Arc::clone(&gate_calls);
    scheduler
        .set_pre_fire_check("cam", move |_now| {
            gate_calls_cb.fetch_add(1, Ordering::Relaxed);
            false // never defer — the count is the oracle, not the outcome
        })
        .unwrap();

    scheduler
        .set_replay_fire_plan(
            0,
            &[cerulion_core::scheduler::ReplayFire {
                node_id: "cam",
                first_fire_ns: 100,
                fire_count: 3,
                interval_ns: 5,
            }],
        )
        .unwrap();
    scheduler.step_ms(1);

    assert_eq!(
        count.load(Ordering::Relaxed),
        3,
        "the recording holds 3 fires for this step"
    );
    assert_eq!(
        gate_calls.load(Ordering::Relaxed),
        1,
        "the pre-fire gate is evaluated ONCE per planned burst — a synthetic \
         `Period` re-encoding would re-check it before every catch-up fire (4)"
    );
    let times: Vec<u64> = scheduler.trace().iter().map(|e| e.fire_time_ns).collect();
    assert_eq!(
        times,
        vec![100, 105, 110],
        "the burst walks the recorded progression by a running add"
    );
}

/// The plan-driven pre-fire gate is evaluated at the
/// ADVANCED STEP CLOCK, exactly as the live decide evaluates it — never at the
/// fire's own recorded instant.
///
/// The design keeps `throttle_ms` LIVE under replay (it is rank-local and
/// re-derivable, so it must reproduce the recording), which makes its INPUTS
/// part of the contract: a gate handed a different clock is a different rule
/// wearing the same name. For every trigger that stamps its fire with the step
/// clock — Data, Sync, External — the recorded `first_fire_ns` IS the step
/// clock, so the two are indistinguishable; a Period CATCH-UP burst is the one
/// shape where they differ, its `first_fire_ns` being the earliest un-fired
/// interval DEADLINE and therefore up to `fire_count * interval - 1` ns below
/// the step clock. Handed to `throttle_defers` that reads as "the node fired
/// more recently than it did".
///
/// The oracle is the PRODUCTION predicate at the PRODUCTION values, so the two
/// clocks are not merely different numbers — they give opposite answers, and
/// the arm asserts that separation itself before asserting the outcome.
///
/// Passing `planned.first_fire_ns` to the gate
/// fails this at zero fires with `unconsumed_replay_fires() == ["cam"]` — a
/// divergence the candidate never caused.
#[test]
#[serial]
fn the_plan_driven_pre_fire_gate_reads_the_step_clock_not_the_fire_instant() {
    use cerulion_core::graph::runtime::throttle_defers;

    // A `throttle_ms = 25` node whose last fire was at 5 ms, stepped to 30 ms
    // with two catch-up deadlines (20 ms, 25 ms) recorded for this step.
    const LAST_FIRE_NS: u64 = 5_000_000;
    const THROTTLE_NS: u64 = 25_000_000;
    const FIRST_FIRE_NS: u64 = 20_000_000;
    const STEP_CLOCK_NS: u64 = 30_000_000;

    // THE DISCRIMINATOR, spelled out before the run: the same rule at the same
    // values answers differently for the two clocks, so this arm cannot pass by
    // the gate being irrelevant.
    assert!(
        !throttle_defers(1, STEP_CLOCK_NS, LAST_FIRE_NS, THROTTLE_NS),
        "at the STEP CLOCK the throttle window has elapsed — live allows the burst"
    );
    assert!(
        throttle_defers(1, FIRST_FIRE_NS, LAST_FIRE_NS, THROTTLE_NS),
        "at the FIRE INSTANT it has not — which is the wrong answer"
    );

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // The gate is the production closure's shape: the real predicate, over a
    // node whose prior fire the scheduler has not itself performed (the values
    // the live gate reads off the node's handle are supplied by hand, because
    // this node has fired zero times in THIS scheduler).
    let seen_now = Arc::new(AtomicU64::new(u64::MAX));
    let seen_cb = Arc::clone(&seen_now);
    scheduler
        .set_pre_fire_check("cam", move |now| {
            seen_cb.store(now, Ordering::Relaxed);
            throttle_defers(1, now, LAST_FIRE_NS, THROTTLE_NS)
        })
        .unwrap();

    scheduler
        .set_replay_fire_plan(
            0,
            &[cerulion_core::scheduler::ReplayFire {
                node_id: "cam",
                first_fire_ns: FIRST_FIRE_NS,
                fire_count: 2,
                interval_ns: 5_000_000,
            }],
        )
        .unwrap();
    scheduler.step(std::time::Duration::from_nanos(STEP_CLOCK_NS));

    assert_eq!(
        seen_now.load(Ordering::Relaxed),
        STEP_CLOCK_NS,
        "the gate must be handed the ADVANCED STEP CLOCK, as `decide_node` hands it live"
    );
    assert_eq!(
        count.load(Ordering::Relaxed),
        2,
        "the recording's burst is performed — the throttle window really had elapsed"
    );
    assert!(
        scheduler.unconsumed_replay_fires().is_empty(),
        "…so nothing is reported unconsumed: {:?}",
        scheduler.unconsumed_replay_fires()
    );
    // …and the fire is still stamped with the RECORDED instants, not the clock
    // the gate was evaluated at — the two are separate questions.
    let times: Vec<u64> = scheduler.trace().iter().map(|e| e.fire_time_ns).collect();
    assert_eq!(times, vec![FIRST_FIRE_NS, FIRST_FIRE_NS + 5_000_000]);
}

#[test]
#[serial]
fn a_gate_that_defers_at_the_step_clock_still_defers_under_a_plan() {
    // ANTI-TAUTOLOGY for the arm above: reading the step clock must not amount
    // to neutering the gate. Same shape, with the last fire moved inside the
    // throttle window AT THE STEP CLOCK — the burst is deferred and reported.
    use cerulion_core::graph::runtime::throttle_defers;

    const LAST_FIRE_NS: u64 = 20_000_000;
    const THROTTLE_NS: u64 = 25_000_000;
    const STEP_CLOCK_NS: u64 = 30_000_000;
    assert!(throttle_defers(1, STEP_CLOCK_NS, LAST_FIRE_NS, THROTTLE_NS));

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();
    scheduler
        .set_pre_fire_check("cam", move |now| {
            throttle_defers(1, now, LAST_FIRE_NS, THROTTLE_NS)
        })
        .unwrap();
    scheduler
        .set_replay_fire_plan(
            0,
            &[cerulion_core::scheduler::ReplayFire {
                node_id: "cam",
                first_fire_ns: 20_000_000,
                fire_count: 2,
                interval_ns: 5_000_000,
            }],
        )
        .unwrap();
    scheduler.step(std::time::Duration::from_nanos(STEP_CLOCK_NS));

    assert_eq!(
        count.load(Ordering::Relaxed),
        0,
        "the gate deferred the burst"
    );
    assert_eq!(
        scheduler.unconsumed_replay_fires(),
        vec!["cam"],
        "…and a planned fire the gate refuses is REPORTED, never silently dropped"
    );
}

/// The pre-fire gate is still CONSULTED under a plan (`throttle_ms` is
/// rank-local and re-derivable, so it must reproduce the recording), and a
/// planned fire it defers is REPORTED rather than silently dropped.
///
/// The control in the same body is what makes the report mean something: with
/// the gate open the same plan is consumed and the list is empty.
#[test]
#[serial]
fn a_deferred_planned_fire_does_not_fire_and_is_reported_unconsumed() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();
    let defer = Arc::new(std::sync::atomic::AtomicBool::new(true));
    let defer_cb = Arc::clone(&defer);
    scheduler
        .set_pre_fire_check("cam", move |_now| defer_cb.load(Ordering::Relaxed))
        .unwrap();

    let planned = [cerulion_core::scheduler::ReplayFire {
        node_id: "cam",
        first_fire_ns: 42,
        fire_count: 1,
        interval_ns: 0,
    }];
    scheduler.set_replay_fire_plan(0, &planned).unwrap();
    scheduler.step_ms(1);
    assert_eq!(
        count.load(Ordering::Relaxed),
        0,
        "the gate deferred, so the planned fire did not happen"
    );
    assert_eq!(
        scheduler.unconsumed_replay_fires(),
        vec!["cam"],
        "a planned fire nothing performed is REPORTED (never silently dropped)"
    );

    // CONTROL: same plan, gate open ⇒ consumed, and the list is empty.
    defer.store(false, Ordering::Relaxed);
    scheduler.set_replay_fire_plan(1, &planned).unwrap();
    scheduler.step_ms(1);
    assert_eq!(count.load(Ordering::Relaxed), 1);
    assert!(
        scheduler.unconsumed_replay_fires().is_empty(),
        "a performed plan reports nothing — else the report is vacuous"
    );
}

/// The panic circuit breaker outranks the plan: a candidate that opened it is
/// not forced to keep firing, and the fires the recording holds for it are
/// reported as unconsumed (which is the node failure a replay verdict reports).
#[test]
#[serial]
fn a_disabled_node_is_not_forced_to_fire_by_the_plan() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let cb: Box<dyn FnMut() + Send> = Box::new(|| panic!("intentional test panic"));
    let handle = scheduler
        .add_node(NodeConfig {
            id: "panicker".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();
    // Three consecutive panics open the breaker (the T13 shape).
    scheduler.step_ms(10);
    scheduler.step_ms(10);
    scheduler.step_ms(10);
    assert_eq!(handle.fire_count(), 3, "precondition: the breaker is open");

    scheduler
        .set_replay_fire_plan(
            3,
            &[cerulion_core::scheduler::ReplayFire {
                node_id: "panicker",
                first_fire_ns: 1,
                fire_count: 2,
                interval_ns: 0,
            }],
        )
        .unwrap();
    scheduler.step_ms(10);
    assert_eq!(
        handle.fire_count(),
        3,
        "a disabled node stays disabled under a plan"
    );
    assert_eq!(scheduler.unconsumed_replay_fires(), vec!["panicker"]);
}

/// A plan installed for a DIFFERENT step fires NOTHING and says so — never a
/// fall-through to live deciding, which would fabricate fires the recording
/// does not hold.
#[test]
#[serial]
fn a_stale_plan_fires_nothing_and_counts_the_mismatch() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // Installed for step 5; the runtime is about to execute steps 0 and 1.
    scheduler
        .set_replay_fire_plan(
            5,
            &[cerulion_core::scheduler::ReplayFire {
                node_id: "cam",
                first_fire_ns: 1,
                fire_count: 1,
                interval_ns: 0,
            }],
        )
        .unwrap();
    scheduler.step_ms(10);
    scheduler.step_ms(10);

    assert_eq!(
        count.load(Ordering::Relaxed),
        0,
        "a stale plan is not a licence to decide freely — the period elapsed twice"
    );
    assert_eq!(
        scheduler.replay_plan_mismatches(),
        2,
        "one per decide seam that ran against it (one node × two steps)"
    );
}

/// The install-time refusals. Each leaves the plan ARMED and EMPTY, so the
/// refused step fires nothing rather than silently under- or over-firing.
#[test]
#[serial]
fn a_malformed_plan_is_refused_and_leaves_the_step_firing_nothing() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // (1) an unknown node — a fire this graph cannot perform.
    let err = scheduler
        .set_replay_fire_plan(
            0,
            &[
                cerulion_core::scheduler::ReplayFire {
                    node_id: "cam",
                    first_fire_ns: 1,
                    fire_count: 1,
                    interval_ns: 0,
                },
                cerulion_core::scheduler::ReplayFire {
                    node_id: "ghost",
                    first_fire_ns: 1,
                    fire_count: 1,
                    interval_ns: 0,
                },
            ],
        )
        .unwrap_err();
    match err {
        TransportError::NodeNotFound { node_id } => assert_eq!(node_id, "ghost"),
        other => panic!("expected NodeNotFound, got {other:?}"),
    }
    assert!(
        scheduler.is_replay_driven(),
        "a refused plan stays ARMED — falling back to live deciding would be worse"
    );
    scheduler.step_ms(10);
    assert_eq!(
        count.load(Ordering::Relaxed),
        0,
        "a refused plan is EMPTY, not PARTIAL: the `cam` entry accepted before \
         the bad one must not fire either"
    );

    // (2) a zero fire count — a node that did not fire carries no entry.
    assert!(scheduler
        .set_replay_fire_plan(
            1,
            &[cerulion_core::scheduler::ReplayFire {
                node_id: "cam",
                first_fire_ns: 1,
                fire_count: 0,
                interval_ns: 0,
            }],
        )
        .is_err());

    // (3) a node named twice — one (node, step) is ONE arithmetic progression.
    assert!(scheduler
        .set_replay_fire_plan(
            2,
            &[
                cerulion_core::scheduler::ReplayFire {
                    node_id: "cam",
                    first_fire_ns: 1,
                    fire_count: 1,
                    interval_ns: 0,
                },
                cerulion_core::scheduler::ReplayFire {
                    node_id: "cam",
                    first_fire_ns: 2,
                    fire_count: 1,
                    interval_ns: 0,
                },
            ],
        )
        .is_err());
}

/// The re-phase runs BACKWARD too.
///
/// Sequential per-rank replay builds one runtime per rank on the ONE clock the
/// engine carries, so rank R+1's `add_node` baselines its `Period` deadlines at
/// `rank R's epoch + interval` and the engine then places the clock at rank
/// R+1's OWN first recorded boundary — which is EARLIER. A placement that treated that as a
/// no-op ("already past now") would leave the deadline a whole recorded run in the
/// future: the node would not fire for the length of the previous rank's
/// window, i.e. the rank produces nothing.
///
/// The interval is 33 ms for the same reason the forward arm uses it: both
/// epochs below are exact multiples of 1 ms and of 30 ms, so on those grids a
/// phase-PRESERVING answer and a phase-discarding `now + interval` coincide and
/// no assertion can tell them apart. 33 ms divides neither.
#[test]
#[serial]
fn restore_period_schedule_re_phases_a_clock_placed_backward_onto_the_same_grid() {
    // Rank R's last boundary — where the clock sits while rank R+1 is BUILT.
    const PREV_RANK_EPOCH_NS: u64 = 1_755_000_000_000_000_000;
    // Rank R+1's own first recorded boundary: 12 s EARLIER (a rank that starts
    // later in the recording than the one replayed before it ended).
    const OWN_FIRST_BOUNDARY_NS: u64 = PREV_RANK_EPOCH_NS - 12_000_000_000;
    const INTERVAL_NS: u64 = 33_000_000;

    let clock = Arc::new(VirtualClock::new());
    clock.set(PREV_RANK_EPOCH_NS);
    let mut scheduler = Scheduler::with_virtual_clock(clock.clone());

    let (cb, count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "scan".to_string(),
            policy: period(33),
            callback: cb,
        })
        .unwrap();
    // Built at the PREVIOUS rank's epoch — the state a no-op placement
    // would call "already correct".
    let built = scheduler
        .node_framework_state("scan")
        .expect("node present")
        .next_fire_ns
        .expect("a Period node carries a deadline");
    assert_eq!(built, PREV_RANK_EPOCH_NS + INTERVAL_NS);

    // The engine places the clock at THIS rank's first boundary, then restores.
    clock.set(OWN_FIRST_BOUNDARY_NS);
    scheduler.restore_period_schedule(&std::collections::BTreeMap::new());

    // HAND ORACLE: the unique V with `built ≡ V (mod interval)` and
    // `now < V <= now + interval`, computed from the constants.
    let ahead = built - OWN_FIRST_BOUNDARY_NS;
    let expected = built - ((ahead - 1) / INTERVAL_NS) * INTERVAL_NS;
    assert!(
        expected > OWN_FIRST_BOUNDARY_NS && expected - OWN_FIRST_BOUNDARY_NS <= INTERVAL_NS,
        "the re-phased deadline is the EARLIEST un-fired one at the placed instant"
    );
    assert_eq!(
        scheduler
            .node_framework_state("scan")
            .expect("node present")
            .next_fire_ns,
        Some(expected),
        "a deadline a whole rank ABOVE the placed clock is pulled back onto its \
         own grid — as a no-op the node stays silent for 12 s"
    );
    // PHASE PRESERVED, stated as the property rather than as one number.
    assert_eq!(expected % INTERVAL_NS, built % INTERVAL_NS);
    // …and the schedule really RUNS from there: one step to the re-phased
    // deadline fires exactly once. Without this the value above describes a
    // field nothing reads.
    scheduler.step(Duration::from_nanos(expected - OWN_FIRST_BOUNDARY_NS));
    assert_eq!(count.load(Ordering::Relaxed), 1);
    // A no-op placement would need ~12 s of stepping to reach its first
    // fire; one interval more here fires again, on the grid.
    scheduler.step(Duration::from_nanos(INTERVAL_NS));
    assert_eq!(count.load(Ordering::Relaxed), 2);
}

/// The BOUNDARY of the backward arm: a deadline already inside one interval of
/// the placed clock is left EXACTLY as it was.
///
/// This is what keeps checkpoint resume (and the from-start free-run replay)
/// byte-unchanged: both build at the clock's ORIGIN, where an un-named node's
/// deadline is exactly one interval up, so the pull-back must be a no-op by
/// construction. Both sides of the boundary are asserted.
#[test]
#[serial]
fn the_backward_re_phase_is_a_no_op_within_one_interval() {
    const INTERVAL_NS: u64 = 33_000_000;

    /// Build a 33 ms node with the clock at `built_at`, then place the clock at
    /// `anchor` and restore. Returns the resulting deadline.
    fn rephased(built_at: u64, anchor: u64) -> u64 {
        let clock = Arc::new(VirtualClock::new());
        clock.set(built_at);
        let mut scheduler = Scheduler::with_virtual_clock(clock.clone());
        let (cb, _count) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "scan".to_string(),
                policy: period(33),
                callback: cb,
            })
            .unwrap();
        assert_eq!(
            scheduler
                .node_framework_state("scan")
                .unwrap()
                .next_fire_ns
                .unwrap(),
            built_at + INTERVAL_NS,
            "precondition: `add_node` baselines at the build clock + interval"
        );
        clock.set(anchor);
        scheduler.restore_period_schedule(&std::collections::BTreeMap::new());
        scheduler
            .node_framework_state("scan")
            .unwrap()
            .next_fire_ns
            .unwrap()
    }

    // Built at the origin, anchored at the origin: exactly one interval up —
    // the checkpoint-resume shape, and the original no-op.
    assert_eq!(rephased(0, 0), INTERVAL_NS);
    // One nanosecond inside the interval: still untouched.
    assert_eq!(rephased(0, INTERVAL_NS - 1), INTERVAL_NS);
    // THREE intervals up over a clock at the origin: pulled back by exactly two,
    // onto the same grid.
    assert_eq!(rephased(2 * INTERVAL_NS, 0), INTERVAL_NS);
    // THE THRESHOLD, pinned on both sides. `ahead == interval` is the last
    // untouched case (above), and `ahead == interval + 1` is the FIRST that
    // moves: built one ns up, anchored at the origin ⇒ pulled back exactly one
    // interval, landing one ns above the clock.
    assert_eq!(rephased(1, 0), 1);
}

// ---------------------------------------------------------------------------
// The INTRA-STEP injection seam
//
// A before-step injection bucket puts every foreign frame in the consumer FIFO
// ahead of ALL of a step's local fires, so a recorded serve order of
// `[local, FOREIGN, local]` on one shared topic is unreproducible by
// construction. `set_replay_intra_step_pauses` + `set_replay_injection_hook`
// are the sub-step vocabulary that makes it reproducible.
//
// Every arm below journals FIRES and PAUSES into ONE shared Vec, so the oracle
// is the INTERLEAVING — not a pair of counts, which a hook that ran every
// invocation after the whole burst would also satisfy.
//
// ONE deliberate coverage hole, recorded so a future reader does not "fix" it:
// `tick_decided_parallel`'s narrow-walk disjunct reads the ARMED FLAG rather
// than `!replay_paused_nodes.is_empty()`, and no test can tell those two
// spellings apart. They differ only for an ARMED-BUT-EMPTY install, where the
// serial and parallel walks are byte-identical by invariant A (same fires, same
// order, same trace) — so the difference is unobservable BY DESIGN, which is
// exactly why the flag is the safer read. Adding a `#[cfg(test)]` seam to make
// that distinction testable would be adding a production hole to satisfy a test; the
// pin is the comment at the disjunct, not an arm here.
// ---------------------------------------------------------------------------

/// A journal shared by the node callbacks and the injection hook, so one
/// ordered `Vec` records how fires and pauses interleaved.
type Journal = Arc<std::sync::Mutex<Vec<String>>>;

fn journal() -> Journal {
    Arc::new(std::sync::Mutex::new(Vec::new()))
}

fn journal_read(j: &Journal) -> Vec<String> {
    j.lock().expect("journal not poisoned").clone()
}

/// A node callback that appends `fire(<id>)` to the journal.
fn journalling_callback(j: &Journal, id: &str) -> Box<dyn FnMut() + Send> {
    let j = Arc::clone(j);
    let id = id.to_string();
    Box::new(move || {
        j.lock()
            .expect("journal not poisoned")
            .push(format!("fire({id})"));
    })
}

/// An injection hook that appends `pause(<id>,<k>)` to the journal — standing
/// in for the engine's real injector publish.
fn journalling_hook(j: &Journal) -> cerulion_core::scheduler::ReplayInjectionHook {
    let j = Arc::clone(j);
    Arc::new(move |id: &str, after_fire: u32| {
        j.lock()
            .expect("journal not poisoned")
            .push(format!("pause({id},{after_fire})"));
    })
}

fn replay_fire(node_id: &str, fire_count: u32) -> cerulion_core::scheduler::ReplayFire<'_> {
    cerulion_core::scheduler::ReplayFire {
        node_id,
        first_fire_ns: 1_000,
        fire_count,
        interval_ns: 100,
    }
}

/// THE headline: the hook runs BETWEEN two fires of one burst, at the slots the
/// recording names, with the completed-fire count that identifies them.
///
/// A 4-fire burst with pauses after fires 1 and 3 must journal
/// `[fire, pause@1, fire, fire, pause@3, fire]` — a HAND oracle, and one that a
/// hook draining after the whole burst (`[fire × 4, pause@1, pause@3]`) or
/// before it cannot satisfy. Both a paused and an UNPAUSED slot appear, so the
/// arm also refuses a hook that fires after every fire.
#[test]
#[serial]
fn an_intra_step_pause_hands_control_between_the_fires_of_a_replayed_burst() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "cam"),
        })
        .unwrap();

    scheduler.set_replay_injection_hook(journalling_hook(&j));
    assert!(scheduler.has_replay_injection_hook());
    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[
                // Deliberately OUT of order at install: the seam sorts, and the
                // per-fire consult is one indexed compare against the cursor.
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 3,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 1,
                },
            ],
        )
        .unwrap();
    assert!(scheduler.is_replay_pause_armed());
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("cam", 4)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec![
            "fire(cam)".to_string(),
            "pause(cam,1)".to_string(),
            "fire(cam)".to_string(),
            "fire(cam)".to_string(),
            "pause(cam,3)".to_string(),
            "fire(cam)".to_string(),
        ],
        "the hook must run BETWEEN the fires the recording names — not before \
         the burst, not after it, and not after every fire"
    );
    assert!(
        scheduler.unconsumed_replay_pauses().is_empty(),
        "both pauses were reached: {:?}",
        scheduler.unconsumed_replay_pauses()
    );
    assert_eq!(scheduler.replay_pause_mismatches(), 0);
}

/// A pause naming the burst's LAST fire is still delivered.
///
/// The consult sits after the fire and before the burst loop's `disabled`
/// break, so `after_fire == fire_count` is a real slot — the recording published
/// those frames at that instant, and the next step's before-step bucket is a
/// DIFFERENT place in the serve order.
#[test]
#[serial]
fn a_pause_after_the_final_fire_of_a_burst_is_still_delivered() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "cam"),
        })
        .unwrap();

    scheduler.set_replay_injection_hook(journalling_hook(&j));
    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "cam",
                after_fire: 2,
            }],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("cam", 2)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec![
            "fire(cam)".to_string(),
            "fire(cam)".to_string(),
            "pause(cam,2)".to_string(),
        ],
        "a slot after the burst's last fire is delivered, not dropped"
    );
    assert!(scheduler.unconsumed_replay_pauses().is_empty());
}

/// ZERO behaviour delta when nothing is paused — the seam changes WHEN control
/// leaves the burst, never WHAT fires.
///
/// Three legs of the SAME 3-fire plan: (a) the seam never armed, (b) armed with
/// an EMPTY list, (c) armed with a list and a hook that is never reached. All
/// three must produce the SAME trace, and that trace must equal a HAND oracle
/// (`fire_time_ns` 1000 / 1100 / 1200 from the plan's first + interval) — never
/// a two-run self-compare.
#[test]
#[serial]
fn an_unpaused_replayed_step_is_byte_identical_with_and_without_the_pause_seam() {
    // The hand oracle: the plan's own arithmetic progression, three fires.
    let oracle: Vec<(String, u64)> = vec![
        ("cam".to_string(), 1_000),
        ("cam".to_string(), 1_100),
        ("cam".to_string(), 1_200),
    ];

    let run = |arm: u8| -> (Vec<(String, u64)>, Vec<String>) {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        let j = journal();
        scheduler
            .add_node(NodeConfig {
                id: "cam".to_string(),
                policy: period(10),
                callback: journalling_callback(&j, "cam"),
            })
            .unwrap();
        match arm {
            // (a) the seam is never touched at all.
            0 => {}
            // (b) armed with an EMPTY list.
            1 => {
                scheduler.set_replay_injection_hook(journalling_hook(&j));
                scheduler.set_replay_intra_step_pauses(0, &[]).unwrap();
            }
            // (c) armed with a pause the 3-fire burst never reaches.
            _ => {
                scheduler.set_replay_injection_hook(journalling_hook(&j));
                scheduler
                    .set_replay_intra_step_pauses(
                        0,
                        &[cerulion_core::scheduler::IntraStepPause {
                            node_id: "cam",
                            after_fire: 9,
                        }],
                    )
                    .unwrap();
            }
        }
        scheduler
            .set_replay_fire_plan(0, &[replay_fire("cam", 3)])
            .unwrap();
        scheduler.step_ms(3);
        let trace = scheduler
            .trace()
            .iter()
            .map(|e| (e.node_id.to_string(), e.fire_time_ns))
            .collect();
        (trace, journal_read(&j))
    };

    for arm in 0..3u8 {
        let (trace, journalled) = run(arm);
        assert_eq!(trace, oracle, "arm {arm}: the fire schedule is the plan's");
        assert_eq!(
            journalled,
            vec![
                "fire(cam)".to_string(),
                "fire(cam)".to_string(),
                "fire(cam)".to_string()
            ],
            "arm {arm}: no pause is reached, so the hook must never run"
        );
    }

    // Determinism: the armed-and-unreached arm run twice is bit-identical to
    // itself AND to the oracle (so neither run is the other's oracle).
    assert_eq!(run(2).0, oracle);
    assert_eq!(run(2).0, oracle);
}

/// A pause the burst never REACHES is reported, never silently dropped — the
/// silent never-fire class, and the whole reason the seam keeps a cursor.
///
/// Two pauses on a 2-fire burst: `after_fire 1` is reached, `after_fire 5` is
/// not, because the recording's burst is shorter than the pause list claims.
#[test]
#[serial]
fn a_pause_the_burst_never_reaches_is_reported_not_dropped() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "cam"),
        })
        .unwrap();

    scheduler.set_replay_injection_hook(journalling_hook(&j));
    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 5,
                },
            ],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("cam", 2)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec![
            "fire(cam)".to_string(),
            "pause(cam,1)".to_string(),
            "fire(cam)".to_string(),
        ],
        "only the REACHED slot is delivered"
    );
    assert_eq!(
        scheduler.unconsumed_replay_pauses(),
        vec![("cam", 5)],
        "the unreached slot is the seam's report, not its silence"
    );
    assert_eq!(
        scheduler.replay_pause_mismatches(),
        0,
        "an unreached pause is not a step-pairing slip"
    );
}

/// A pause on a node the step's FIRE PLAN does not hold: reported, never a
/// panic.
///
/// The two installs are independent — the pause list does not know the fire
/// plan, and either order is legal — so a node that never bursts simply leaves
/// every one of its pauses unconsumed. The node that DID burst is unaffected
/// (no cross-node leakage of the cursor).
#[test]
#[serial]
fn a_pause_on_a_node_the_fire_plan_does_not_hold_is_reported_never_a_panic() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "cam"),
        })
        .unwrap();
    scheduler
        .add_node(NodeConfig {
            id: "imu".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "imu"),
        })
        .unwrap();

    scheduler.set_replay_injection_hook(journalling_hook(&j));
    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "imu",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 1,
                },
            ],
        )
        .unwrap();
    // The plan holds a burst for `cam` ONLY.
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("cam", 1)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec!["fire(cam)".to_string(), "pause(cam,1)".to_string()],
        "`imu` never bursts, so its pause never delivers — and `cam`'s is unaffected"
    );
    assert_eq!(
        scheduler.unconsumed_replay_pauses(),
        vec![("imu", 1)],
        "the pause on the non-bursting node is REPORTED"
    );
}

/// A pause list installed for a DIFFERENT step delivers NOTHING, counts every
/// consult, and logs once — the stale-plan rule `set_replay_fire_plan` states,
/// applied to the pause half.
///
/// Without the step tag a pairing slip would inject a step's foreign frames
/// into the wrong step's serve order — silently, since the trace cannot show it.
#[test]
#[serial]
fn a_stale_pause_list_delivers_nothing_and_counts_every_consult() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "cam"),
        })
        .unwrap();

    scheduler.set_replay_injection_hook(journalling_hook(&j));
    // Pauses for step 5; the plan (and the step) are 0.
    scheduler
        .set_replay_intra_step_pauses(
            5,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "cam",
                after_fire: 1,
            }],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("cam", 2)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec!["fire(cam)".to_string(), "fire(cam)".to_string()],
        "a stale list must never deliver (no fall-through to the wrong step)"
    );
    assert_eq!(
        scheduler.replay_pause_mismatches(),
        2,
        "the consult runs once per fire, and the counter is UNCONDITIONAL \
         (the log is latched once per install; this is not)"
    );
    assert_eq!(
        scheduler.unconsumed_replay_pauses(),
        vec![("cam", 1)],
        "nothing was delivered, so nothing is consumed"
    );
}

/// Clearing the HOOK out from under installed pauses loses the slot's frames —
/// so the seam refuses to record it as served: the cursor does not advance and
/// the pause reports as UNCONSUMED.
///
/// The install-time refusal (below) makes this reachable only this way, and it
/// is the one place "reached" and "delivered" can differ.
#[test]
#[serial]
fn a_pause_reached_with_the_hook_cleared_is_unconsumed_not_silently_served() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "cam"),
        })
        .unwrap();

    scheduler.set_replay_injection_hook(journalling_hook(&j));
    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "cam",
                after_fire: 1,
            }],
        )
        .unwrap();
    scheduler.clear_replay_injection_hook();
    assert!(!scheduler.has_replay_injection_hook());
    assert!(
        scheduler.is_replay_pause_armed(),
        "clearing the hook deliberately leaves the pauses armed — the loss must \
         surface, not be tidied away"
    );
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("cam", 2)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec!["fire(cam)".to_string(), "fire(cam)".to_string()],
    );
    assert_eq!(
        scheduler.unconsumed_replay_pauses(),
        vec![("cam", 1)],
        "reached but never delivered is UNCONSUMED, not served"
    );
}

/// Every install refusal, and the ARMED-but-EMPTY rule each one leaves behind:
/// a refused list must not deliver PART of a step's injections, and must not
/// disarm either.
#[test]
#[serial]
fn a_malformed_pause_list_is_refused_loudly_and_leaves_the_seam_armed_but_empty() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "cam"),
        })
        .unwrap();

    // (0) pauses with NO hook installed — nothing to hand control to.
    let err = scheduler
        .set_replay_intra_step_pauses(
            0,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "cam",
                after_fire: 1,
            }],
        )
        .unwrap_err();
    match &err {
        TransportError::GraphError { reason } => assert!(
            reason.contains("set_replay_injection_hook"),
            "the refusal must name the fix, got: {reason}"
        ),
        other => panic!("expected GraphError, got {other:?}"),
    }
    // An EMPTY list with no hook is legal — it is how "no pauses this step" is
    // declared, and it must not require a hook that has nothing to do.
    scheduler.set_replay_intra_step_pauses(0, &[]).unwrap();

    scheduler.set_replay_injection_hook(journalling_hook(&j));

    // (1) an unknown node — the mirror of the fire plan's NodeNotFound.
    let err = scheduler
        .set_replay_intra_step_pauses(
            0,
            &[
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "ghost",
                    after_fire: 1,
                },
            ],
        )
        .unwrap_err();
    match err {
        TransportError::NodeNotFound { node_id } => assert_eq!(node_id, "ghost"),
        other => panic!("expected NodeNotFound, got {other:?}"),
    }
    assert!(
        scheduler.is_replay_pause_armed(),
        "a refused list stays ARMED — falling back to before-step-only delivery \
         would be worse"
    );
    assert!(
        scheduler.unconsumed_replay_pauses().is_empty(),
        "a refused list is EMPTY, not PARTIAL: the `cam` entry accepted before \
         the bad one is retired too"
    );
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("cam", 2)])
        .unwrap();
    scheduler.step_ms(3);
    assert_eq!(
        journal_read(&j),
        vec!["fire(cam)".to_string(), "fire(cam)".to_string()],
        "the entry accepted before the refusal must not deliver either"
    );

    // (2) after_fire 0 — the count is 1-based and means COMPLETED fires. The
    //     list carries a LEGAL pause FIRST, so the arm can see the ARMED-and-
    //     EMPTY rule rather than merely the error text: a refusal that kept the
    //     entry it had already accepted would deliver PART of a step's
    //     injections.
    let err = scheduler
        .set_replay_intra_step_pauses(
            1,
            &[
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 0,
                },
            ],
        )
        .unwrap_err();
    match &err {
        TransportError::GraphError { reason } => assert!(
            reason.contains("after_fire 0"),
            "the refusal must name the offending value, got: {reason}"
        ),
        other => panic!("expected GraphError, got {other:?}"),
    }
    assert!(
        scheduler.is_replay_pause_armed(),
        "the after_fire-0 refusal ARMS the seam like the other three"
    );
    assert!(
        scheduler.unconsumed_replay_pauses().is_empty(),
        "the accepted (cam, 1) is retired with the rest: {:?}",
        scheduler.unconsumed_replay_pauses()
    );
    // …and behaviourally, which the accessor alone cannot show: the refused
    // list was tagged for step 1 and the run below IS step 1, so a surviving
    // (cam, 1) would MATCH the burst's first fire and deliver.
    scheduler
        .set_replay_fire_plan(1, &[replay_fire("cam", 2)])
        .unwrap();
    scheduler.step_ms(3);
    assert_eq!(
        journal_read(&j),
        vec![
            "fire(cam)".to_string(),
            "fire(cam)".to_string(),
            "fire(cam)".to_string(),
            "fire(cam)".to_string(),
        ],
        "step 1 fires twice more and pauses NOWHERE — the entry accepted before \
         the after_fire-0 refusal must not deliver"
    );

    // (3) the same (node, after_fire) twice — two injections at ONE slot are
    //     ONE slot carrying both frames.
    let err = scheduler
        .set_replay_intra_step_pauses(
            2,
            &[
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 2,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 2,
                },
            ],
        )
        .unwrap_err();
    match &err {
        TransportError::GraphError { reason } => assert!(
            reason.contains("twice") && reason.contains("cam"),
            "the refusal must name the node and the doubled slot, got: {reason}"
        ),
        other => panic!("expected GraphError, got {other:?}"),
    }
    assert!(
        scheduler.is_replay_pause_armed(),
        "the duplicate-slot refusal ARMS the seam like the other three — this is \
         the LAST arm, and the duplicate check runs in its own post-loop pass, \
         so nothing else covers its arming"
    );
    assert!(scheduler.unconsumed_replay_pauses().is_empty());
}

/// A re-install retires the previous step's list, and `clear` disarms the seam
/// entirely — the two halves of the per-step lifecycle, driven across three
/// steps against one hand oracle.
#[test]
#[serial]
fn a_re_install_retires_the_previous_steps_pauses_and_clear_disarms_the_seam() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "cam"),
        })
        .unwrap();
    scheduler.set_replay_injection_hook(journalling_hook(&j));

    // Step 0: a pause at 1, reached.
    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "cam",
                after_fire: 1,
            }],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("cam", 1)])
        .unwrap();
    scheduler.step_ms(3);

    // Step 1: a DIFFERENT pause. The step-0 entry must be gone — a list that
    // accumulated would inject one slot's frames on every later step.
    scheduler
        .set_replay_intra_step_pauses(
            1,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "cam",
                after_fire: 2,
            }],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(1, &[replay_fire("cam", 2)])
        .unwrap();
    scheduler.step_ms(3);

    // Step 2: disarmed. Nothing pauses, and the accessor reports nothing.
    scheduler.clear_replay_intra_step_pauses();
    assert!(!scheduler.is_replay_pause_armed());
    assert!(scheduler.unconsumed_replay_pauses().is_empty());
    scheduler
        .set_replay_fire_plan(2, &[replay_fire("cam", 2)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec![
            // step 0
            "fire(cam)".to_string(),
            "pause(cam,1)".to_string(),
            // step 1 — the step-0 slot is retired, the step-1 slot is served
            "fire(cam)".to_string(),
            "fire(cam)".to_string(),
            "pause(cam,2)".to_string(),
            // step 2 — disarmed
            "fire(cam)".to_string(),
            "fire(cam)".to_string(),
        ],
        "each step serves ITS OWN slots, and a disarmed seam serves none"
    );
    assert_eq!(scheduler.replay_pause_mismatches(), 0);
}

/// A node added AFTER the hook is installed inherits it — otherwise a graph
/// assembled in the wrong order would be silently pause-deaf on exactly the
/// nodes added last.
#[test]
#[serial]
fn a_node_added_after_the_hook_is_installed_inherits_it() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler.set_replay_injection_hook(journalling_hook(&j));
    scheduler
        .add_node(NodeConfig {
            id: "late".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "late"),
        })
        .unwrap();

    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "late",
                after_fire: 1,
            }],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("late", 1)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec!["fire(late)".to_string(), "pause(late,1)".to_string()],
        "the hook installed before the node reaches the node"
    );
}

/// The seam sits BEFORE the burst loop's `disabled` break — so the LAST fire of
/// a burst the panic circuit breaker just ended still serves its slot.
///
/// The placement carries a comment stating why ("withholding would replace a
/// node failure with an input divergence") and, before this arm, NO test drove a
/// node the breaker opens MID-BURST: `a_disabled_node_is_not_forced_to_fire_by_
/// the_plan` disables the node BEFORE the plan is installed, so the decide seam
/// refuses the fire and the burst never runs at all.
///
/// Two panics are banked first, so the burst's fire 0 is the THIRD consecutive
/// panic (`MAX_CONSECUTIVE_PANICS`): it disables the node, the pause at
/// `after_fire 1` must STILL be delivered, and only then does the burst break.
/// The node's callback panics, so it can journal nothing — which makes the
/// journal a clean single-element oracle, and the fire count the proof the break
/// really happened (3, not the plan's 2 + 3 = 5).
#[test]
#[serial]
fn a_pause_after_the_fire_that_opened_the_panic_breaker_is_still_delivered() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    let cb: Box<dyn FnMut() + Send> = Box::new(|| panic!("intentional test panic"));
    let handle = scheduler
        .add_node(NodeConfig {
            id: "panicker".to_string(),
            policy: period(10),
            callback: cb,
        })
        .unwrap();

    // Bank TWO consecutive panics — one short of the breaker.
    //
    // COUPLED to the scheduler's private `MAX_CONSECUTIVE_PANICS` (3): this
    // count is `MAX - 1`, so that the burst's FIRST fire is the one that opens
    // the breaker. Raise the constant and this arm stops testing what it
    // claims — the burst would run to completion and the pause would be
    // delivered by the ordinary path rather than by the placement above the
    // `disabled` break. The precondition assert below is what fails first if
    // that happens (it would read 2 fires but the burst would not break at 3).
    scheduler.step_ms(10);
    scheduler.step_ms(10);
    assert_eq!(
        handle.fire_count(),
        2,
        "precondition: two panics banked, the breaker is still CLOSED (so the \
         decide seam will mint the burst below)"
    );

    scheduler.set_replay_injection_hook(journalling_hook(&j));
    scheduler
        .set_replay_intra_step_pauses(
            2,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "panicker",
                after_fire: 1,
            }],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(2, &[replay_fire("panicker", 3)])
        .unwrap();
    scheduler.step_ms(10);

    assert_eq!(
        journal_read(&j),
        vec!["pause(panicker,1)".to_string()],
        "the slot after the fire that OPENED the breaker is delivered — moving \
         the seam below the `disabled` break would journal nothing"
    );
    assert_eq!(
        handle.fire_count(),
        3,
        "the burst broke after the disabling fire: 2 banked + 1, never 2 + 3"
    );
    assert!(
        scheduler.unconsumed_replay_pauses().is_empty(),
        "the one installed slot was reached: {:?}",
        scheduler.unconsumed_replay_pauses()
    );
    assert_eq!(scheduler.replay_hook_panics(), 0, "the HOOK did not panic");
}

/// A panicking injection hook is CAUGHT, COUNTED and REPORTED — never an unwind
/// out of `step()`.
///
/// The hook is engine code called from inside a node's burst. An unguarded panic
/// would unwind past the node tick two frames away and out of `step()` with no
/// counter, no report and no verdict — the one loss on this seam that leaves
/// nothing behind. The node-tick `catch_unwind` does NOT cover it: that frame
/// has already returned by the time the pause is consulted, which is why the
/// catch at the seam is a LOCAL one and not a delegation to the circuit breaker.
///
/// The caught arm keeps the seam's own rule: the cursor does NOT advance, so the
/// slot reports as UNCONSUMED (its frames really were not injected). A stuck
/// cursor also blocks that node's LATER slots, which is why `after_fire 3` is
/// unconsumed too — the real consequence of a broken injector, and identical
/// to what a cleared hook already does.
#[test]
#[serial]
fn a_panicking_injection_hook_is_caught_counted_and_leaves_its_slot_unconsumed() {
    use std::sync::atomic::AtomicBool;

    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "cam"),
        })
        .unwrap();

    let poisoned = Arc::new(AtomicBool::new(true));
    let hook = {
        let j = Arc::clone(&j);
        let poisoned = Arc::clone(&poisoned);
        let hook: cerulion_core::scheduler::ReplayInjectionHook =
            Arc::new(move |id: &str, after_fire: u32| {
                assert!(
                    !poisoned.load(Ordering::Relaxed),
                    "the injector exploded (deliberate)"
                );
                j.lock()
                    .expect("journal not poisoned")
                    .push(format!("pause({id},{after_fire})"));
            });
        hook
    };
    scheduler.set_replay_injection_hook(hook);

    // Step 0: the hook panics at the FIRST slot it is handed.
    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 3,
                },
            ],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("cam", 3)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec![
            "fire(cam)".to_string(),
            "fire(cam)".to_string(),
            "fire(cam)".to_string(),
        ],
        "the burst CONTINUES past a panicking hook — all three planned fires \
         happen, and the hook delivered nothing"
    );
    assert_eq!(
        scheduler.replay_hook_panics(),
        1,
        "counted ONCE: the cursor did not advance, so the consult at completed \
         2 and 3 no longer matched the head entry and never re-entered the hook"
    );
    assert_eq!(
        scheduler.unconsumed_replay_pauses(),
        vec![("cam", 1), ("cam", 3)],
        "the panicking slot is UNCONSUMED, and a stuck cursor blocks the node's \
         later slots too — the accurate report of a broken injector"
    );
    assert_eq!(
        scheduler.replay_pause_mismatches(),
        0,
        "a hook panic is not a step-pairing slip"
    );

    // Step 1: a healthy hook, a re-install — the seam re-arms and delivers.
    poisoned.store(false, Ordering::Relaxed);
    scheduler
        .set_replay_intra_step_pauses(
            1,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "cam",
                after_fire: 1,
            }],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(1, &[replay_fire("cam", 2)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j)[3..],
        [
            "fire(cam)".to_string(),
            "pause(cam,1)".to_string(),
            "fire(cam)".to_string(),
        ],
        "a re-install with a healthy hook delivers normally"
    );
    assert!(scheduler.unconsumed_replay_pauses().is_empty());
    assert_eq!(
        scheduler.replay_hook_panics(),
        1,
        "the counter is CUMULATIVE — a re-install never resets it"
    );
}

/// The no-hook refusal leaves the seam ARMED and EMPTY, exactly like the other
/// three — including when a PREVIOUS step's pauses are already installed.
///
/// That arm used to `return` before the retire loop and before the arming flag,
/// so it was the one refusal that did NOT honour the contract the fn documents:
/// the previous step's list stayed installed (and, from a virgin scheduler, the
/// seam stayed UNARMED). It is reachable exactly this way — install, then
/// `clear_replay_injection_hook` (which deliberately leaves pauses armed), then
/// a re-install that is refused.
#[test]
#[serial]
fn the_no_hook_refusal_retires_the_previous_steps_pauses_and_leaves_the_seam_armed() {
    // Leg 1 — a VIRGIN scheduler: the refusal must still ARM the seam. "No
    // pauses this step" and "this scheduler is not pause-driven" are different
    // facts, and a refusal must not leave the caller reading the second.
    {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        let j = journal();
        scheduler
            .add_node(NodeConfig {
                id: "cam".to_string(),
                policy: period(10),
                callback: journalling_callback(&j, "cam"),
            })
            .unwrap();
        scheduler
            .set_replay_intra_step_pauses(
                0,
                &[cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 1,
                }],
            )
            .unwrap_err();
        assert!(
            scheduler.is_replay_pause_armed(),
            "the no-hook refusal ARMS the seam like the other three — an early \
             return would leave it unarmed"
        );
        // Scope: on a VIRGIN scheduler this emptiness kills only a variant that
        // returns early (nothing was ever installed, so every arm answers
        // empty). The load-bearing "a refusal RETIRES what was already there"
        // pin is leg 2 below, which has a previous install to lose.
        assert!(scheduler.unconsumed_replay_pauses().is_empty());
    }

    // Leg 2 — the refusal must RETIRE the previous install. Reachable exactly
    // this way: install, then `clear_replay_injection_hook` (which deliberately
    // leaves pauses armed), then a re-install that is refused.
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "cam"),
        })
        .unwrap();

    scheduler.set_replay_injection_hook(journalling_hook(&j));
    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "cam",
                after_fire: 1,
            }],
        )
        .unwrap();
    scheduler.clear_replay_injection_hook();

    let err = scheduler
        .set_replay_intra_step_pauses(
            0,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "cam",
                after_fire: 2,
            }],
        )
        .unwrap_err();
    match &err {
        TransportError::GraphError { reason } => assert!(
            reason.contains("set_replay_injection_hook"),
            "the refusal must name the fix, got: {reason}"
        ),
        other => panic!("expected GraphError, got {other:?}"),
    }

    assert!(
        scheduler.is_replay_pause_armed(),
        "a refused list stays ARMED — falling back to before-step-only delivery \
         would be worse"
    );
    assert!(
        scheduler.unconsumed_replay_pauses().is_empty(),
        "the PREVIOUS step's list is retired by the refusal too: {:?}",
        scheduler.unconsumed_replay_pauses()
    );

    // …and behaviourally. The retired list was tagged for step 0 and the run
    // below IS step 0, so a surviving entry would MATCH and deliver — this is
    // the arm that sees the difference, not merely the accessor.
    scheduler.set_replay_injection_hook(journalling_hook(&j));
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("cam", 2)])
        .unwrap();
    scheduler.step_ms(3);
    assert_eq!(
        journal_read(&j),
        vec!["fire(cam)".to_string(), "fire(cam)".to_string()],
        "a refused install serves NOTHING — not the refused list, and not the \
         one it replaced"
    );
    assert_eq!(scheduler.replay_pause_mismatches(), 0);

    // …and the seam RECOVERS. Every assertion above is an ABSENCE, which a seam
    // that a refusal had permanently wedged would satisfy just as well — so the
    // arm ends on the positive: the first SUCCESSFUL install after a refusal
    // delivers normally.
    scheduler
        .set_replay_intra_step_pauses(
            1,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "cam",
                after_fire: 1,
            }],
        )
        .expect("a well-formed install after a refusal must succeed");
    scheduler
        .set_replay_fire_plan(1, &[replay_fire("cam", 2)])
        .unwrap();
    scheduler.step_ms(3);
    assert_eq!(
        journal_read(&j)[2..],
        [
            "fire(cam)".to_string(),
            "pause(cam,1)".to_string(),
            "fire(cam)".to_string(),
        ],
        "the install after the refusal delivers at its own slot"
    );
    assert!(scheduler.unconsumed_replay_pauses().is_empty());
    assert_eq!(scheduler.replay_pause_mismatches(), 0);
    assert_eq!(scheduler.replay_hook_panics(), 0);
}

/// A PAUSED step is deterministic: two runs of one fixture produce the same
/// trace AND the same fire/pause interleaving, both equal to a hand oracle.
///
/// `an_unpaused_replayed_step_is_byte_identical_with_and_without_the_pause_seam`
/// pins determinism only where the hook never runs. This is the arm where it
/// does — the hook order IS the serve order of the frames the engine injects, so
/// Principle #7 binds on the journal, not just the trace.
#[test]
#[serial]
fn a_paused_replayed_step_is_deterministic_across_runs() {
    let trace_oracle: Vec<(String, u64)> = vec![
        ("cam".to_string(), 1_000),
        ("cam".to_string(), 1_100),
        ("cam".to_string(), 1_200),
        ("cam".to_string(), 1_300),
    ];
    let journal_oracle: Vec<String> = vec![
        "fire(cam)".to_string(),
        "pause(cam,1)".to_string(),
        "fire(cam)".to_string(),
        "fire(cam)".to_string(),
        "pause(cam,3)".to_string(),
        "fire(cam)".to_string(),
    ];

    let run = || -> (Vec<(String, u64)>, Vec<String>) {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        let j = journal();
        scheduler
            .add_node(NodeConfig {
                id: "cam".to_string(),
                policy: period(10),
                callback: journalling_callback(&j, "cam"),
            })
            .unwrap();
        scheduler.set_replay_injection_hook(journalling_hook(&j));
        scheduler
            .set_replay_intra_step_pauses(
                0,
                &[
                    cerulion_core::scheduler::IntraStepPause {
                        node_id: "cam",
                        after_fire: 3,
                    },
                    cerulion_core::scheduler::IntraStepPause {
                        node_id: "cam",
                        after_fire: 1,
                    },
                ],
            )
            .unwrap();
        scheduler
            .set_replay_fire_plan(0, &[replay_fire("cam", 4)])
            .unwrap();
        scheduler.step_ms(3);
        let trace = scheduler
            .trace()
            .iter()
            .map(|e| (e.node_id.to_string(), e.fire_time_ns))
            .collect();
        (trace, journal_read(&j))
    };

    // Each run is compared to the ORACLE, never to the other run — so neither
    // is the other's oracle.
    for attempt in 0..2 {
        let (trace, journalled) = run();
        assert_eq!(trace, trace_oracle, "attempt {attempt}: fire schedule");
        assert_eq!(
            journalled, journal_oracle,
            "attempt {attempt}: fire/pause interleaving"
        );
    }
}

/// Across NODES the hook runs in INSERTION order, and the unconsumed report
/// reads the same way — node insertion order, then ascending count.
///
/// Three nodes are inserted in REVERSE-alphabetical order and their pauses are
/// installed in a SCRAMBLED order, so neither a name sort nor the install order
/// can masquerade as the contract. Each node carries one slot the burst reaches
/// and one it does not, so the ordering claim is pinned on both the journal and
/// the report.
#[test]
#[serial]
fn the_hook_and_the_unconsumed_report_read_in_node_insertion_order() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    for id in ["zeta", "mid", "alpha"] {
        scheduler
            .add_node(NodeConfig {
                id: id.to_string(),
                policy: period(10),
                callback: journalling_callback(&j, id),
            })
            .unwrap();
    }
    scheduler.set_replay_injection_hook(journalling_hook(&j));

    // Scrambled: `alpha` (insertion idx 2) is named FIRST, so an unsorted
    // touched list would report it first.
    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "alpha",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "zeta",
                    after_fire: 5,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "mid",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "alpha",
                    after_fire: 5,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "zeta",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "mid",
                    after_fire: 5,
                },
            ],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(
            0,
            &[
                replay_fire("zeta", 2),
                replay_fire("mid", 2),
                replay_fire("alpha", 2),
            ],
        )
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec![
            "fire(zeta)".to_string(),
            "pause(zeta,1)".to_string(),
            "fire(zeta)".to_string(),
            "fire(mid)".to_string(),
            "pause(mid,1)".to_string(),
            "fire(mid)".to_string(),
            "fire(alpha)".to_string(),
            "pause(alpha,1)".to_string(),
            "fire(alpha)".to_string(),
        ],
        "nodes are walked in INSERTION order, and each node's hook call sits \
         between its own fires — never a name sort, never the install order"
    );
    assert_eq!(
        scheduler.unconsumed_replay_pauses(),
        vec![("zeta", 5), ("mid", 5), ("alpha", 5)],
        "the report reads in node INSERTION order — dropping the touched-list \
         sort reads the INSTALL order (alpha, zeta, mid), and a name sort reads \
         (alpha, mid, zeta)"
    );
}

/// A hook panic is CONTAINED to the node whose slot it exploded on: the SIBLING
/// node's pause still delivers in the same step.
///
/// `a_panicking_injection_hook_is_caught_counted_and_leaves_its_slot_unconsumed`
/// drives ONE node, so it cannot tell "the panic parks THIS node's cursor" from
/// "the panic disarms the seam for the rest of the step" — and the second would
/// silently drop every later node's foreign frames while reporting exactly the
/// same counter. Two nodes, one hook that explodes only for `cam`, separates
/// them.
///
/// The second step pins the counter as an INCREMENT rather than merely
/// "not reset by a re-install": a saturating `= 1` would satisfy the cumulative
/// arm of the single-node test and fail here.
#[test]
#[serial]
fn a_hook_panic_on_one_node_does_not_stop_a_siblings_pause_from_delivering() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    for id in ["cam", "imu"] {
        scheduler
            .add_node(NodeConfig {
                id: id.to_string(),
                policy: period(10),
                callback: journalling_callback(&j, id),
            })
            .unwrap();
    }

    // Explodes for `cam` only; `imu`'s slot journals normally.
    let hook: cerulion_core::scheduler::ReplayInjectionHook = {
        let j = Arc::clone(&j);
        Arc::new(move |id: &str, after_fire: u32| {
            assert!(id != "cam", "the injector exploded on cam (deliberate)");
            j.lock()
                .expect("journal not poisoned")
                .push(format!("pause({id},{after_fire})"));
        })
    };
    scheduler.set_replay_injection_hook(hook);

    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "imu",
                    after_fire: 1,
                },
            ],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(0, &[replay_fire("cam", 2), replay_fire("imu", 2)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec![
            "fire(cam)".to_string(),
            // cam's slot exploded — no `pause(cam,1)`, and the burst continues.
            "fire(cam)".to_string(),
            "fire(imu)".to_string(),
            "pause(imu,1)".to_string(),
            "fire(imu)".to_string(),
        ],
        "the panic is contained to `cam`: `imu`'s slot still delivers BETWEEN \
         its own two fires, in the same step"
    );
    assert_eq!(
        scheduler.unconsumed_replay_pauses(),
        vec![("cam", 1)],
        "only the exploded slot is unconsumed — `imu`'s was served"
    );
    assert_eq!(scheduler.replay_hook_panics(), 1);
    assert_eq!(
        scheduler.replay_pause_mismatches(),
        0,
        "a hook panic is not a step-pairing slip"
    );

    // Step 1: the same explosion again. The counter must INCREMENT.
    scheduler
        .set_replay_intra_step_pauses(
            1,
            &[cerulion_core::scheduler::IntraStepPause {
                node_id: "cam",
                after_fire: 1,
            }],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(1, &[replay_fire("cam", 1)])
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        scheduler.replay_hook_panics(),
        2,
        "a second panicking step ACCUMULATES — a latch-shaped `= 1` would read 1"
    );
    assert_eq!(
        scheduler.unconsumed_replay_pauses(),
        vec![("cam", 1)],
        "the re-installed slot exploded too, so it is unconsumed too"
    );
}

/// Removing a node RE-DERIVES the pause seam's index list, so the survivors'
/// pauses stay reportable.
///
/// `remove_node`'s `shift_remove` re-points every LATER insertion index, and
/// `replay_paused_nodes` holds insertion indices — so a stale entry can ALIAS a
/// different, still-in-range node. The aliasing is invisible unless the paused
/// set has a GAP at the removal point: with `beta` unpaused, the stale list
/// `[0, 2, 3]` re-reads as `alpha`, `delta`, out-of-range — silently DROPPING
/// `gamma`, which is the exact under-report `unconsumed_replay_pauses` exists to
/// prevent (and which would then let the next install's retire loop clear an
/// innocent node's list).
#[test]
#[serial]
fn removing_a_node_keeps_the_surviving_pauses_reportable() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    for id in ["alpha", "beta", "gamma", "delta"] {
        scheduler
            .add_node(NodeConfig {
                id: id.to_string(),
                policy: period(10),
                callback: journalling_callback(&j, id),
            })
            .unwrap();
    }
    scheduler.set_replay_injection_hook(journalling_hook(&j));

    // `beta` (insertion idx 1) is deliberately UNPAUSED — the gap that makes the
    // aliasing observable.
    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "alpha",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "gamma",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "delta",
                    after_fire: 1,
                },
            ],
        )
        .unwrap();

    scheduler.remove_node("beta").expect("beta is in the graph");
    scheduler
        .remove_node("beta")
        .expect_err("a second removal is still NodeNotFound");

    assert_eq!(
        scheduler.unconsumed_replay_pauses(),
        vec![("alpha", 1), ("gamma", 1), ("delta", 1)],
        "every survivor's installed pause is still named, in insertion order — \
         a bare `shift_remove` reads (alpha, delta) and loses gamma's"
    );

    // …and the seam still WORKS afterwards: a clean re-install over the shrunk
    // graph delivers at every slot. (With a stale list, the re-install's retire
    // loop clears the wrong nodes, leaving `gamma` holding entries that are
    // neither retirable nor reportable — the state the install's `debug_assert`
    // names.)
    scheduler
        .set_replay_intra_step_pauses(
            0,
            &[
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "alpha",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "gamma",
                    after_fire: 1,
                },
                cerulion_core::scheduler::IntraStepPause {
                    node_id: "delta",
                    after_fire: 1,
                },
            ],
        )
        .unwrap();
    scheduler
        .set_replay_fire_plan(
            0,
            &[
                replay_fire("alpha", 2),
                replay_fire("gamma", 2),
                replay_fire("delta", 2),
            ],
        )
        .unwrap();
    scheduler.step_ms(3);

    assert_eq!(
        journal_read(&j),
        vec![
            "fire(alpha)".to_string(),
            "pause(alpha,1)".to_string(),
            "fire(alpha)".to_string(),
            "fire(gamma)".to_string(),
            "pause(gamma,1)".to_string(),
            "fire(gamma)".to_string(),
            "fire(delta)".to_string(),
            "pause(delta,1)".to_string(),
            "fire(delta)".to_string(),
        ],
        "`beta` is gone and every survivor still pauses between its own fires"
    );
    assert!(scheduler.unconsumed_replay_pauses().is_empty());
}

/// The caught hook panic REPORTS ITS CAUSE — the payload is rendered under
/// `error=`, not dropped.
///
/// `replay_hook_panics` says a panic happened and `unconsumed_replay_pauses`
/// says where; neither says WHAT, and the injection hook is engine code whose
/// failure an operator has no other window onto (the counter is not reachable
/// from a replay log). The seam's two sibling `catch_unwind` sites in the same
/// file — `fire_node_into` and the pre-fire check — both render the payload
/// through a `&str` / `String` / "unknown panic" downcast ladder; this one
/// dropped it entirely, so a replay whose injector exploded reported a count
/// and a position and no cause at all.
///
/// All THREE ladder arms are driven, because a partial ladder is the easy
/// regression: a `&str` payload (a bare `panic!("literal")`), a `String`
/// payload (a formatted `panic!`), and a payload that is NEITHER
/// (`panic_any`), which must fall back rather than render nothing.
#[test]
#[serial]
#[traced_test]
fn a_caught_hook_panic_logs_the_panic_payload_as_its_cause() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let j = journal();
    scheduler
        .add_node(NodeConfig {
            id: "cam".to_string(),
            policy: period(10),
            callback: journalling_callback(&j, "cam"),
        })
        .unwrap();

    // Step 0 renders a `&str` payload, step 1 a `String`, step 2 a `u8` that
    // matches neither downcast.
    let hook: cerulion_core::scheduler::ReplayInjectionHook =
        Arc::new(move |_id: &str, after_fire: u32| match after_fire {
            1 => panic!("str payload"),
            2 => panic!("string payload {}", 7),
            _ => std::panic::panic_any(42u8),
        });
    scheduler.set_replay_injection_hook(hook);

    for (step, after_fire) in [(0u64, 1u32), (1, 2), (2, 3)] {
        scheduler
            .set_replay_intra_step_pauses(
                step,
                &[cerulion_core::scheduler::IntraStepPause {
                    node_id: "cam",
                    after_fire,
                }],
            )
            .unwrap();
        scheduler
            .set_replay_fire_plan(step, &[replay_fire("cam", after_fire)])
            .unwrap();
        scheduler.step_ms(3);
    }

    assert_eq!(
        scheduler.replay_hook_panics(),
        3,
        "precondition: all three payload shapes really did panic and were caught"
    );
    // `error=` and not a bare substring: the payload must be carried as the
    // structured field the other two ladders use, which is what an operator
    // greps. A message that merely mentioned the text would not be enough.
    assert!(
        logs_contain("error=str payload"),
        "the &str arm of the downcast ladder must render the payload"
    );
    assert!(
        logs_contain("error=string payload 7"),
        "the String arm of the downcast ladder must render the payload"
    );
    assert!(
        logs_contain("error=unknown panic"),
        "a payload that is neither &str nor String falls back to the ladder's \
         final arm — never to silence"
    );
}

/// A THROTTLE-deferred node reports its throttle DEADLINE,
/// not `Some(0)`.
///
/// # The bug this closes
///
/// `throttle_ms` defers a node while `now - last_fire < N`, but every signal
/// the scheduler can see says the node is due: `pending_data_count > 0`. So the
/// Data arm reports `Some(0)`, `live_timeout` clamps to its 1 ms floor, and the
/// live loop wakes a thousand times a second for the whole window — each time
/// only to decide to defer again. On a `throttle_ms = 100` node that is ~100
/// wakeups per window, on a plane whose entire purpose is to stop paying
/// wakeups for nothing.
///
/// # Oracles, and the control
///
/// Exact and hand-written, never a self-compare. With a 50 ms throttle and a
/// fire at t = 10 ms the node is deferred until t = 60 ms, so at t = 10 ms the
/// answer is 50 ms, at 35 ms it is 25 ms, at 59 ms it is 1 ms. Three DIFFERENT
/// numbers, none of them `Some(0)`, so an implementation reporting a constant
/// of any value fails.
///
/// The control is a SECOND scheduler holding an identical node with NO
/// `throttle_ms`, driven with the identical stimulus, which must report
/// `Some(0)` at every one of those instants. Two schedulers rather than two
/// nodes in one, because `ns_until_next_fire` takes the MIN across nodes: an
/// unthrottled sibling in the same scheduler would pull every reading to zero
/// and the throttled answer would be unobservable.
#[test]
fn a_throttled_node_reports_its_throttle_deadline_and_an_unthrottled_one_still_reports_due_now() {
    const THROTTLE_NS: u64 = 50_000_000;
    const FIRED_AT_NS: u64 = 10_000_000;

    /// One scheduler holding one Data node, fired once at `FIRED_AT_NS` and
    /// then given a carried backlog — the shape both halves of the oracle are
    /// read from. `throttle_ns: None` builds the control.
    fn armed(throttle_ns: Option<u64>) -> Scheduler {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(Arc::clone(&clock));
        let (cb, _count) = counting_callback();
        scheduler
            .add_node(NodeConfig {
                id: "n".to_string(),
                policy: TriggerPolicy::Data,
                callback: cb,
            })
            .unwrap();
        if let Some(ns) = throttle_ns {
            scheduler.set_throttle_ns("n", ns).unwrap();
        }
        // Fire once at t = 10 ms so `last_fire_ns` is real scheduler state, not
        // something the test stamped: the throttle reads what the scheduler
        // RECORDED, and a test that wrote that field by hand would not prove
        // the two agree.
        clock.set(FIRED_AT_NS);
        scheduler.signal_data("n").unwrap();
        scheduler.step(Duration::from_millis(0));
        // Then a carried backlog, so the node answers due-NOW on every other
        // ground and the ONLY thing that can change the answer is the throttle.
        for _ in 0..3 {
            scheduler.signal_data("n").unwrap();
        }
        scheduler
    }

    let throttled = armed(Some(THROTTLE_NS));
    let free = armed(None);

    let handle = throttled.node_handle("n").expect("the node's handle");
    assert_eq!(
        handle.fire_count(),
        1,
        "precondition: the node really fired — with `fire_count == 0` the rule \
         deliberately does not defer (no prior fire to throttle against), so a \
         zero here would make every reading below vacuous"
    );
    assert_eq!(
        handle.last_fire_ns(),
        FIRED_AT_NS,
        "precondition: the scheduler recorded the fire at the instant the \
         oracles below are computed from"
    );

    for (now_ms, expected_ns) in [(10u64, 50_000_000u64), (35, 25_000_000), (59, 1_000_000)] {
        let now_ns = now_ms * 1_000_000;
        assert_eq!(
            throttled.ns_until_next_fire(now_ns),
            Some(expected_ns),
            "at t = {now_ms} ms a node throttled 50 ms after a fire at 10 ms is due \
             in {expected_ns} ns, not NOW"
        );
        assert_eq!(
            free.ns_until_next_fire(now_ns),
            Some(0),
            "the UNTHROTTLED control carrying the identical backlog is still due NOW \
             at t = {now_ms} ms — so the deadline above is attributable to the \
             throttle, not to a change in how a backlog is reported"
        );
    }

    // The window CLOSES: the rule is `<`, not `<=`, so at exactly
    // `last_fire + throttle` the throttle stops deferring and the carried
    // backlog's due-NOW answer returns. Pinned on both sides.
    assert_eq!(
        throttled.ns_until_next_fire(FIRED_AT_NS + THROTTLE_NS - 1),
        Some(1),
        "one ns before the deadline the node is still deferred, by one ns"
    );
    assert_eq!(
        throttled.ns_until_next_fire(FIRED_AT_NS + THROTTLE_NS),
        Some(0),
        "AT the deadline the throttle no longer defers, so the backlog reports \
         due-NOW again"
    );
    assert_eq!(
        throttled.ns_until_next_fire(FIRED_AT_NS + THROTTLE_NS + 1),
        Some(0),
        "and past it"
    );
}

/// A zero throttle window is REFUSED rather than read as "no cap".
///
/// The macro rejects `throttle_ms = 0` at compile time, so reaching the
/// scheduler with one means a caller COMPUTED the window instead of reading the
/// declaration. Storing it would be the worst outcome available: `Some(0)` and
/// `None` behave identically in `throttle_remaining_ns`, so the mistake would
/// be invisible for the life of the run.
#[test]
fn a_zero_throttle_window_is_refused_and_so_is_an_unknown_node() {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);
    let (cb, _count) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "n".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb,
        })
        .unwrap();
    let text = scheduler
        .set_throttle_ns("n", 0)
        .expect_err("a zero window is not a rate cap")
        .to_string();
    assert!(
        text.contains("throttle_ms") && text.contains("'n'"),
        "the refusal must name the knob and the node: {text}"
    );
    assert!(
        scheduler.set_throttle_ns("missing", 1_000_000).is_err(),
        "an unknown node is refused, exactly as `set_pre_fire_check` refuses one"
    );
    // And a legitimate window is accepted on the same scheduler — the
    // anti-tautology half, without which both refusals above would be
    // satisfied by a method that refused everything.
    scheduler
        .set_throttle_ns("n", 1_000_000)
        .expect("a real window is accepted");
}
