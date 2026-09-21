// SPDX-License-Identifier: AGPL-3.0-only
//! Per-input backpressure counter scaffolding.
//!
//! Pins the contract that:
//!   1. `Scheduler::register_backpressure_input` is idempotent and
//!      returns a stable `Arc<BackpressureCounters>`.
//!   2. `Scheduler::signal_backpressure_event` bumps the per-policy
//!      counter visible via `NodeHandle::backpressure_*_count(input)`.
//!   3. Unknown inputs read as 0 without panic.
//!   4. The three policy variants (`DropOldest`, `Block`, `Sample(N)` —
//!      `DropNewest` was cut) increment **independent** counters
//!      (mirrors the deadline counters' `three_deadline_counters_are_independent`).
//!   5. Counters are per-input, NOT per-node (the semantic the backpressure
//!      design specifies, vs the deadline counters' per-node pattern).
//!
//! The transport-layer enforcement paths are NOT exercised here — those
//! land in the iceoryx2 e2e suites (`backpressure_block_iox2_test.rs`,
//! `backpressure_sample_iox2_test.rs`, `backpressure_throttle_iox2_test.rs`),
//! which drive the actual enforcement through real subscribers.

use std::sync::atomic::Ordering;
use std::time::Duration;

use cerulion_core::graph::node::BackpressurePolicy;
use cerulion_core::scheduler::{NodeConfig, Scheduler, TriggerPolicy};
use cerulion_core::testing::{count_at_exclusively, debug_lines_expected, never_loud};

fn fixture(node_id: &str) -> (Scheduler, cerulion_core::NodeHandle) {
    let mut scheduler = Scheduler::new();
    let handle = scheduler
        .add_node(NodeConfig {
            id: node_id.to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(1_000_000),
                max_catchup: None,
            },
            callback: Box::new(|| {}),
        })
        .expect("add_node");
    (scheduler, handle)
}

#[test]
fn unregistered_input_reads_zero_without_panic() {
    let (_scheduler, handle) = fixture("n");
    assert_eq!(handle.backpressure_drop_oldest_count("unregistered"), 0);
    assert_eq!(
        handle.backpressure_block_fires_deferred_count("unregistered"),
        0
    );
    assert_eq!(handle.backpressure_sampled_count("unregistered"), 0);
    assert_eq!(
        handle.backpressure_block_defer_regimes_count("unregistered"),
        0
    );
}

#[test]
fn register_backpressure_input_is_idempotent() {
    let (mut scheduler, _handle) = fixture("n");
    let c1 = scheduler
        .register_backpressure_input("n", "image")
        .expect("register first time");
    let c2 = scheduler
        .register_backpressure_input("n", "image")
        .expect("register second time should not fail or reset");
    // Bumping via the second Arc must be visible via the first — the
    // re-registration must NOT have allocated a fresh counter.
    c2.drop_oldest_count.fetch_add(7, Ordering::Release);
    assert_eq!(c1.drop_oldest_count.load(Ordering::Acquire), 7);
}

#[test]
fn signal_each_policy_bumps_its_own_counter_via_handle() {
    let (mut scheduler, handle) = fixture("n");
    scheduler
        .register_backpressure_input("n", "image")
        .expect("register");
    scheduler
        .signal_backpressure_event("n", "image", BackpressurePolicy::DropOldest)
        .expect("DropOldest signal");
    scheduler
        .signal_backpressure_event("n", "image", BackpressurePolicy::Block)
        .expect("Block signal");
    scheduler
        .signal_backpressure_event("n", "image", BackpressurePolicy::Sample(50))
        .expect("Sample signal");
    assert_eq!(handle.backpressure_drop_oldest_count("image"), 1);
    assert_eq!(handle.backpressure_block_fires_deferred_count("image"), 1);
    assert_eq!(handle.backpressure_sampled_count("image"), 1);
    // A signalled Block EVENT is a deferred step, not a regime opening: only
    // the producer-side defer edge's arm→fire transition (`open_block_regime`,
    // in the runtime) opens a regime. A bump here would make `regimes ==
    // defers` and defeat the once-per-regime oracle that compares them.
    assert_eq!(handle.backpressure_block_defer_regimes_count("image"), 0);
}

#[test]
fn each_policy_counter_is_independent() {
    let (mut scheduler, handle) = fixture("n");
    scheduler
        .register_backpressure_input("n", "image")
        .expect("register");
    // Fire ONLY DropOldest; the other counters must stay at 0.
    for _ in 0..10 {
        scheduler
            .signal_backpressure_event("n", "image", BackpressurePolicy::DropOldest)
            .expect("signal");
    }
    assert_eq!(handle.backpressure_drop_oldest_count("image"), 10);
    assert_eq!(handle.backpressure_block_fires_deferred_count("image"), 0);
    assert_eq!(handle.backpressure_sampled_count("image"), 0);
    assert_eq!(handle.backpressure_block_defer_regimes_count("image"), 0);
}

#[test]
fn counters_are_per_input_not_per_node() {
    // Pins the per-input semantic the backpressure design specifies, vs the
    // deadline counters' per-node pattern. A drop on "image" must NOT show up
    // on a sibling input "lidar".
    let (mut scheduler, handle) = fixture("n");
    scheduler
        .register_backpressure_input("n", "image")
        .expect("register image");
    scheduler
        .register_backpressure_input("n", "lidar")
        .expect("register lidar");
    scheduler
        .signal_backpressure_event("n", "image", BackpressurePolicy::DropOldest)
        .expect("image drop");
    assert_eq!(handle.backpressure_drop_oldest_count("image"), 1);
    assert_eq!(
        handle.backpressure_drop_oldest_count("lidar"),
        0,
        "lidar drop count must NOT be incremented by an image event"
    );
}

#[test]
fn signal_on_unknown_node_returns_err() {
    let (scheduler, _handle) = fixture("n");
    let res =
        scheduler.signal_backpressure_event("missing", "image", BackpressurePolicy::DropOldest);
    assert!(res.is_err());
}

#[test]
fn signal_on_unregistered_input_is_noop_not_panic() {
    // Per the API contract: signaling an event on an unregistered
    // input is a silent no-op. The counter stays at 0 (because the
    // input was never registered, so there's no atomic to bump).
    let (scheduler, handle) = fixture("n");
    scheduler
        .signal_backpressure_event("n", "image", BackpressurePolicy::DropOldest)
        .expect("signal should succeed even for unregistered input");
    assert_eq!(handle.backpressure_drop_oldest_count("image"), 0);
}

#[test]
fn cached_arc_bypasses_scheduler_for_hot_path() {
    // Pins the documented hot-path optimization: the transport layer
    // can cache the Arc from register_backpressure_input and bump
    // counters directly without re-entering the scheduler's RwLock.
    let (mut scheduler, handle) = fixture("n");
    let counters = scheduler
        .register_backpressure_input("n", "image")
        .expect("register");
    // Bump directly via the cached Arc (this is what transport-layer
    // hot paths do).
    counters.drop_oldest_count.fetch_add(42, Ordering::Release);
    assert_eq!(handle.backpressure_drop_oldest_count("image"), 42);
}

// ============================================================
// The `emit_warn == false` (sustained-regime) arms for the
// `Block` and `Sample` policies of `record_backpressure_event_n`. Since the
// flood fix these are PRODUCTION-reachable (the `block` pre-fire defer and
// the `sample(N)` decimate both pass `false` on sustained steps of a regime) —
// so these direct-call tests are now redundant-but-harmless UNIT pins over the
// production behavior, complementing the e2e regime pins in
// `backpressure_event_iox2_test.rs`. They keep the contract pinned at the
// helper's own seam: the sustained event logs at `debug!` (present under
// `no-env-filter`), the loud `warn!` does NOT fire, and the per-policy counter
// still bumps unconditionally (Principle #3).
// ============================================================

use cerulion_core::scheduler::{record_backpressure_event_n, BackpressureCounters};
use tracing_test::traced_test;

#[test]
#[traced_test]
fn block_emit_warn_false_logs_debug_not_warn_and_bumps_counter() {
    let counters = BackpressureCounters::new();
    // emit_warn == false ⇒ the sustained (debug) arm.
    record_backpressure_event_n("n", "image", BackpressurePolicy::Block, &counters, 1, false);

    // The counter bump is UNCONDITIONAL (Principle #3), regardless of log level.
    assert_eq!(
        counters.block_fires_deferred_count.load(Ordering::Acquire),
        1,
        "block counter must bump even on the emit_warn=false arm"
    );
    // The sustained debug event fired (its message is unique to the debug arm).
    // The sustained line is `debug!`: it exists only where `debug!` is compiled in.
    // Level-free twin: the sustained Block event must never be LOUD — the half of the contract
    // that survives `release_max_level_info`, where the gated count below reads 0.
    logs_assert(|lines: &[&str]| {
        never_loud(lines, "backpressure event (sustained")?;
        // Checked AT DEBUG, not by text: the sustained line re-emitted at
        // `trace!` still satisfies a `logs_contain`, and the loud sweep above
        // permits TRACE. `count_at_exclusively` asserts the level and, in the
        // same call, that no copy of the line sits at any other level.
        let sustained = count_at_exclusively(lines, "DEBUG", &["backpressure event (sustained"])?;
        if sustained == debug_lines_expected(1) {
            Ok(())
        } else {
            Err(format!(
                "the emit_warn=false Block arm must log the sustained event at DEBUG EXACTLY \
                 once, got {sustained}"
            ))
        }
    });
    // The loud warn arm did NOT fire (its message is unique to the warn arm).
    assert!(
        !logs_contain("sole-Block-consumer-full"),
        "the loud warn! must NOT fire when emit_warn=false"
    );
}

#[test]
#[traced_test]
fn sample_emit_warn_false_logs_debug_not_warn_and_bumps_counter() {
    let counters = BackpressureCounters::new();
    // emit_warn == false ⇒ the sustained (debug) arm. n == 3 also pins the
    // batch bump.
    record_backpressure_event_n(
        "n",
        "image",
        BackpressurePolicy::Sample(50),
        &counters,
        3,
        false,
    );

    assert_eq!(
        counters.sampled_count.load(Ordering::Acquire),
        3,
        "sample counter must bump by n even on the emit_warn=false arm"
    );
    // The sustained line is `debug!`: it exists only where `debug!` is compiled in.
    // Level-free twin: the sustained Sample event must never be LOUD — the half of the contract
    // that survives `release_max_level_info`, where the gated count below reads 0.
    logs_assert(|lines: &[&str]| {
        never_loud(lines, "backpressure event (sustained")?;
        // Checked AT DEBUG, not by text: the sustained line re-emitted at
        // `trace!` still satisfies a `logs_contain`, and the loud sweep above
        // permits TRACE. `count_at_exclusively` asserts the level and, in the
        // same call, that no copy of the line sits at any other level.
        let sustained = count_at_exclusively(lines, "DEBUG", &["backpressure event (sustained"])?;
        if sustained == debug_lines_expected(1) {
            Ok(())
        } else {
            Err(format!(
                "the emit_warn=false Sample arm must log the sustained event at DEBUG EXACTLY \
                 once, got {sustained}"
            ))
        }
    });
    // The warn arm's message opens with `backpressure event: dropped …`, whereas
    // the debug arm opens with `backpressure event (sustained …`, so this
    // substring is unique to the loud warn form.
    assert!(
        !logs_contain("backpressure event: dropped message arrived"),
        "the loud warn! must NOT fire when emit_warn=false"
    );
}
