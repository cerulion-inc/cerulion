// SPDX-License-Identifier: AGPL-3.0-only
//! The never-block drop-latch LOG discipline over a REAL blocking sink.
//!
//! A `CallbackSink` whose callback blocks on a gate the test holds reproduces
//! a wedged-sink failure mode with the REAL Rerun log pipeline: the worker's
//! `rec.log` genuinely blocks (batcher fills behind the wedged sink), so the
//! sink's enqueue path is exercised exactly as in production. Pins:
//!
//! - the tick/enqueue never blocks even though the worker is stuck in a REAL
//!   `rec.log` (every `try_enqueue` stays far under a tick budget);
//! - viz frames are DROPPED (counted) while wedged;
//! - the loud-once log discipline (the house `FieldsWarnLatch` contract): the
//!   first drop of a regime `warn!`s ONCE, repeats `debug!`, and when the gate
//!   releases + the worker drains, a successful enqueue emits ONE recovery
//!   `info!`.
//!
//! Single `#[test]` in its own binary so `#[traced_test]`'s capture is isolated.
//!
//! **The timed window excludes the harness.** The enqueue-never-blocks
//! assert used to time all 60 enqueues with the `traced_test` capture live, which
//! made it a coin flip: MEASURED at 7 failures in 15 runs on a desk under build
//! load, worst 96.3 ms against its own 50 ms ceiling (69.8 ms on an idle desk).
//! Nothing was wrong with the production path. The window held TWO costs that
//! are not the enqueue: building the batch, and — the dominant one —
//! `try_enqueue`'s single `tracing` event, which under `#[traced_test]`'s
//! no-env-filter GLOBAL subscriber contends for one process-global capture
//! buffer with the `re_chunk::batcher` thread's TRACE flood (a line per FLUSH,
//! which under this test's `ALWAYS_TEST_ONLY` batcher is a line per row).
//! Everything else in `try_enqueue` is a non-blocking `try_send`. The window now
//! measures the enqueue rather than the instrumentation (see the phase-2
//! comment); the 50 ms ceiling is unchanged. That flake had never been seen
//! because NO CI job ran this binary — the gap the viz test job closes.

mod common;

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::{InputFrames, VizLogWorker};
use common::{build_twist_at, builtin_walker};
use rerun::log::ChunkBatcherConfig;
use rerun::sink::CallbackSink;
use tracing::subscriber::NoSubscriber;
use tracing_test::traced_test;

/// Batches fed with the `traced_test` capture LIVE, so the loud-once discipline
/// is observable over as much of the run as possible.
///
/// Deliberately the BULK of the run, not a prefix: the queue holds
/// `VIZ_QUEUE_CAP` (8) batches, so this window drops roughly
/// `REGIME_BATCHES - 8` of them, and the `logs_assert` below is only as sharp as
/// that number. The earlier test captured all 60 enqueues (~50 drops); a
/// small phase 1 would have narrowed the oracle enough that a latch RE-ARMING
/// every N drops could slip through (4 warns over 50 drops fails; 1 warn over 11
/// passes). Keeping phase 1 large preserves the original sharpness — the timed
/// window needs only a handful of samples to bound a `try_send`.
const REGIME_BATCHES: u64 = 50;

/// Total batches fed while wedged. `REGIME_BATCHES..TOTAL_BATCHES` is the TIMED
/// window (see the phase-2 comment for why it is instrumented differently).
const TOTAL_BATCHES: u64 = 60;

/// One tick's batch = one Twist frame on the `twist` input (dispatches to six
/// `Scalars` `rec.log` calls).
///
/// `seq` advances the frame's WIRE timestamp by 10 ms, modelling a 100 Hz
/// publisher. That advancement is load-bearing: a plot topic is
/// rate-gated on its publisher's own clock, so a fixture stamping every frame
/// identically would render exactly one of them and the sink would never wedge —
/// this test's whole premise.
fn twist_batch(seq: u64) -> Vec<InputFrames> {
    vec![InputFrames {
        name: "twist".to_string(),
        frames: vec![build_twist_at(
            [1.0, 2.0, 3.0],
            [0.1, 0.2, 0.3],
            seq * 10_000_000,
        )],
    }]
}

#[traced_test]
#[test]
fn tick_never_blocks_and_drop_latch_is_loud_once_then_recovers_over_a_real_blocking_sink() {
    // A gate the callback blocks on; the test holds it LOCKED to wedge the sink.
    let gate = Arc::new(Mutex::new(()));
    let sends = Arc::new(AtomicU64::new(0));
    let cb_gate = Arc::clone(&gate);
    let cb_sends = Arc::clone(&sends);
    let sink = CallbackSink::new(move |msgs| {
        cb_sends.fetch_add(msgs.len() as u64, Ordering::Relaxed);
        // Block while the test holds the gate — the REAL sink wedge.
        let _held = cb_gate.lock().expect("gate");
    });

    // A tiny always-flush batcher so the pipeline fills (and the worker blocks
    // in rec.log) after only a couple of frames instead of ~100 MB.
    let tiny = ChunkBatcherConfig {
        max_bytes_in_flight: 1024,
        ..ChunkBatcherConfig::ALWAYS_TEST_ONLY
    };
    // Build a stream carrying the tiny batcher (explicitly set, so the swap
    // below keeps it), then install the blocking CallbackSink.
    let rec = rerun::RecordingStreamBuilder::new("go2_test")
        .recording_id("drop_latch_log")
        .batcher_config(tiny)
        .buffered()
        .expect("buffered stream");
    rec.set_sink(Box::new(sink));

    // Hold the gate: every callback send now blocks → the worker wedges.
    let held = gate.lock().expect("hold gate");

    let mut worker =
        VizLogWorker::spawn(rec, builtin_walker(), SinkState::new()).expect("spawn worker");

    // PHASE 1 — INSTRUMENTED, UNMEASURED. Open the drop regime with the live
    // `traced_test` capture in place, so the loud-once discipline below is
    // observable. Nothing here is timed.
    for seq in 0..REGIME_BATCHES {
        worker.try_enqueue(twist_batch(seq));
    }

    // PHASE 2 — MEASURED. Two things are held OUT of the timed window, because
    // wiring the viz CI job measured them dominating it and turning this assert into a
    // ~50 %-under-load coin flip (see the module docs).
    //
    //  1. The batches are BUILT UP FRONT — `twist_batch` allocates and encodes a
    //     wire frame, which is not what "the enqueue never blocks" is about.
    //  2. THIS THREAD's `tracing` events are routed to `NoSubscriber` for the
    //     duration. `try_enqueue` is a non-blocking `try_send` plus ONE
    //     `tracing` event, so the only thing in it that can cost tens of
    //     milliseconds is that event — and under `#[traced_test]`'s
    //     no-env-filter global subscriber it contends, for one process-global
    //     capture buffer, with the `re_chunk::batcher` thread emitting a TRACE
    //     line per flushed row. That is the HARNESS's cost, not the sink wedge's.
    //     Measured worst cases against the 50 ms ceiling: 69.8 ms on an idle
    //     desk, 96.3 ms under load.
    //
    // The ceiling is UNCHANGED at 50 ms and is now a real gate rather than a
    // noise band: what it bounds is a `try_send` on a full channel, which is
    // microseconds. The first phase carries the log-discipline pin, and is kept LARGE
    // for that reason (see `REGIME_BATCHES`): a latch regression that warned on
    // every DROPPED batch would emit ~`REGIME_BATCHES - VIZ_QUEUE_CAP` warns
    // there and fail the `logs_assert` below.
    //
    // SCOPE of the split: log events emitted DURING phase 2 are invisible
    // to `logs_assert`. That window is 10 enqueues at the tail of an established
    // regime, so the only thing it can hide is a heal→re-open cycle occurring
    // inside it (a `try_send` that unexpectedly succeeds takes the `Ok` arm and
    // calls `on_decoded`). The first phase is where the regime's shape is asserted.
    let measured: Vec<Vec<InputFrames>> =
        (REGIME_BATCHES..TOTAL_BATCHES).map(twist_batch).collect();
    let mut max_enqueue = Duration::ZERO;
    tracing::subscriber::with_default(NoSubscriber::default(), || {
        for batch in measured {
            let t0 = Instant::now();
            worker.try_enqueue(batch);
            max_enqueue = max_enqueue.max(t0.elapsed());
        }
    });
    assert!(
        max_enqueue < Duration::from_millis(50),
        "the enqueue must never block even with the worker stuck in a REAL rec.log \
         (worst {max_enqueue:?})"
    );
    assert!(
        worker.dropped_frames() > 0,
        "viz frames must be dropped while the real sink is wedged (dropped={})",
        worker.dropped_frames()
    );

    // Loud-once discipline while wedged: exactly ONE warn, >=1 debug repeats.
    logs_assert(|lines: &[&str]| {
        let warns = lines
            .iter()
            .filter(|l| l.contains("DROPPING viz frames to keep"))
            .count();
        let debugs = lines
            .iter()
            .filter(|l| l.contains("viz frames still dropping"))
            .count();
        if warns != 1 {
            return Err(format!("expected exactly ONE loud drop warn, got {warns}"));
        }
        if debugs < 1 {
            return Err(format!(
                "expected >=1 suppressed debug repeats, got {debugs}"
            ));
        }
        Ok(())
    });

    // Release the gate → the worker drains; a subsequent enqueue heals the regime
    // with ONE recovery info.
    drop(held);
    worker.sync(); // wait for the worker to drain the backlog
    worker.try_enqueue(twist_batch(TOTAL_BATCHES)); // succeeds now → heal
    worker.sync();

    logs_assert(|lines: &[&str]| {
        let recoveries = lines
            .iter()
            .filter(|l| l.contains("viz queue draining again"))
            .count();
        if recoveries != 1 {
            return Err(format!(
                "expected exactly ONE recovery info after the regime healed, got {recoveries}"
            ));
        }
        Ok(())
    });

    // Sanity: the callback actually ran (the wedge was real, not a no-op).
    assert!(
        sends.load(Ordering::Relaxed) > 0,
        "the real sink callback must have been invoked"
    );
}
