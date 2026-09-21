// SPDX-License-Identifier: AGPL-3.0-only
//! The SYNTHETIC FIREHOSE benchmark.
//!
//! Reproduces, in process, the shape that cost the Go2 recording 17,720 of
//! 71,453 frames: many topics provisioned by their OWN producers at iceoryx2's
//! stock `subscriber_max_borrowed_samples = 2` and stock-depth receive queues —
//! what a `ros2 attach` bridge route, or any publisher that has never heard of
//! the recorder, gets.
//!
//! Previously that provisioning forced the recorder off its dedicated writer
//! thread and onto the drain thread, where a tap's held budget was
//! `max_borrowed - 1` = ONE frame. The drain therefore took one frame per topic
//! per pass, and each such pass paid a synchronous `writev` plus a chunk close —
//! so the recorder's aggregate frame rate was capped by its own flush rate, and
//! the producer's queue absorbed the difference until it lapped.
//!
//! `#[ignore]`d — a MEASUREMENT, not a gate. It is written to run unchanged on
//! both sides of the change, so the recorder is the only variable:
//!
//! ```text
//! cargo test -p cerulion_bagd --test firehose_bench -- --ignored --nocapture
//! ```
//!
//! Shape knobs (env, so one binary sweeps): `FH_TOPICS`, `FH_HZ`, `FH_BODY`,
//! `FH_QUEUE`, `FH_SECS`.
//!
//! # Measured (macOS/M-series desk, debug build, NVMe)
//!
//! | shape | offered | loss before | loss after |
//! |---|---|---|---|
//! | 75 topics x 40 Hz x 900 B (Go2-LIKE) | 2.65 MB/s | 0.00 % (360 chunks) | 0.00 % (4 chunks) |
//! | 64 topics x 2000 Hz x 64 B | 11.6 MB/s | 0.47 % (11,573 chunks) | 0.00 % (14 chunks) |
//! | 64 topics x 3000 Hz x 64 B | 17.4 MB/s | **10.33 %** (8,055 chunks) | **0.00 %** (19 chunks) |
//! | 24 topics x 2000 Hz x 1400 B | 65.4 MB/s | 33.09 % | 41.80 % |
//!
//! CAVEAT ON THE LOSS COLUMN: those figures are the GAP-based reading
//! (`frames_lost / published`), which under-reads — a queue overflow that
//! discards a contiguous PREFIX before its tap has a baseline is invisible to
//! gap detection. The run now also prints the conservation identity
//! (`published - recorded`), which cannot miss anything; read THAT as the loss,
//! and treat the table above as a lower bound on the saturated rows.
//!
//! Read plainly, three things:
//!
//! 1. **This desk cannot reproduce the Go2 failure at Go2 RATES.** At the
//!    measured ~2.8 MB/s both sides record everything — the Jetson's eMMC makes
//!    a flush an order of magnitude more expensive than this machine's NVMe, and
//!    the old cap is a FLUSH-RATE cap. The chunk counts are the mechanism
//!    showing through even where the loss does not: 360 synchronous flushes
//!    against 4.
//! 2. **The mechanism is real and reproducible once the FRAME rate outruns the
//!    flush rate** — small frames at a high rate, which is the Go2's 500 Hz
//!    `/lowstate` in miniature. That is the third row: 10.33 % lost against
//!    0.00 %, on identical input, with the recorder the only difference.
//! 3. **Past saturation, batching does not help and can hurt.** At 65 MB/s
//!    neither side keeps up and the batching recorder loses MORE (41.80 % vs
//!    33.09 %). Both numbers are of a system far beyond its throughput; the
//!    supportable claim for the batching writer is the band where a recorder can keep up at all,
//!    not the band where nothing can.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_bagd::{run_bagd, TapSpec};

use common::*;

/// iceoryx2's stock budget for a service nobody provisioned for recording.
const STOCK_BORROW: usize = 2;

/// The producer's `subscriber_max_borrowed_samples`, exposed so the
/// bench can SWEEP it instead of arguing about it.
///
/// This is the whole of `RECORDING_SUBSCRIBER_MAX_BORROWED`'s effect on the
/// recorder — `graph run --record` raises a recorded topic's service to that
/// value, and the recorder's tap then inherits it — so sweeping the producer's
/// provisioning measures the constant WITHOUT mutating it. Defaults to
/// `STOCK_BORROW`, so the documented rows above are byte-unchanged.
fn borrow_budget() -> usize {
    env_or("FH_BORROW", STOCK_BORROW as u64) as usize
}

fn env_or(key: &str, default: u64) -> u64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

#[test]
#[ignore = "benchmark: run explicitly with --ignored --nocapture"]
fn firehose_at_the_stock_borrow_budget() {
    // Defaults = the discriminating row above.
    let topics_n = env_or("FH_TOPICS", 64) as usize;
    let hz = env_or("FH_HZ", 3000);
    let body_len = env_or("FH_BODY", 64) as usize;
    let queue_depth = env_or("FH_QUEUE", 16) as usize;
    let secs = env_or("FH_SECS", 3);

    let borrow = borrow_budget();

    let mgr = make_manager(16);
    let out = unique_out("firehose");
    let ready = unique_out("firehose_ready");

    let topics: Vec<String> = (0..topics_n)
        .map(|i| unique_topic(&format!("fh{i}")))
        .collect();
    let mut publishers: Vec<_> = topics
        .iter()
        .map(|t| publisher_with_provisioning(&mgr, t, borrow, queue_depth, (body_len + 64) as u32))
        .collect();

    let mut cfg =
        cerulion_bagd::BagdConfig::new(out.clone(), topics.iter().map(TapSpec::attach).collect());
    cfg.ready_file = Some(ready.clone());
    cfg.status_period = None;

    let shutdown = Arc::new(AtomicBool::new(false));
    let shutdown_c = shutdown.clone();
    let mgr_c = mgr.clone();
    let bagd = std::thread::spawn(move || run_bagd(mgr_c, cfg, shutdown_c));
    assert!(
        wait_for_file(&ready, Duration::from_secs(10)),
        "ready-file never appeared"
    );
    settle();

    // One paced publisher thread per topic — identical code on both sides of
    // the change, so the only variable is the recorder.
    let go = Arc::new(AtomicBool::new(false));
    let period = Duration::from_nanos(1_000_000_000 / hz);
    let frames_each = hz * secs;
    let mut threads = Vec::with_capacity(topics_n);
    for (i, mut pubr) in publishers.drain(..).enumerate() {
        let go_c = go.clone();
        threads.push(std::thread::spawn(move || {
            while !go_c.load(Ordering::Relaxed) {
                std::hint::spin_loop();
            }
            let start = Instant::now();
            let body = vec![(i as u8).wrapping_add(1); body_len];
            for k in 0..frames_each {
                pubr.publish_raw(&build_frame(0xF14E, k as u32, k, &body))
                    .expect("publish");
                let due = start + period.mul_f64((k + 1) as f64);
                let now = Instant::now();
                if due > now {
                    std::thread::sleep(due - now);
                }
            }
            frames_each
        }));
    }
    let wall_start = Instant::now();
    go.store(true, Ordering::Relaxed);
    let published: u64 = threads
        .into_iter()
        .map(|t| t.join().expect("publisher thread"))
        .sum();
    let publish_wall = wall_start.elapsed();

    // Let the recorder drain whatever it can before the shutdown boundary.
    std::thread::sleep(Duration::from_millis(500));
    shutdown.store(true, Ordering::Relaxed);
    let summary = bagd
        .join()
        .expect("join bagd")
        .expect("run_bagd Ok — a firehose must not be an ERROR");

    let bytes = published * (body_len as u64 + 32);
    let rate_mb_s = bytes as f64 / publish_wall.as_secs_f64() / (1024.0 * 1024.0);
    // TWO loss readings, because they measure different things and the
    // gap-based one UNDER-reads.
    //
    // `frames_lost` is the wire-sequence GAP count, which needs a
    // BASELINE: a queue that discards a contiguous PREFIX before its tap ever
    // drained leaves the tap's first observation carrying a nonzero sequence,
    // indistinguishable from attaching mid-stream, so that loss is invisible to
    // it (see `Recorder::drain_taps`; the recorder accounts for it separately, as
    // `prefix_lost` in the coverage manifest). MISSING is the conservation
    // identity — published minus recorded — which needs no baseline and cannot
    // miss anything, so it is the number to read as THE loss. They agree once
    // every tap has a baseline; where they disagree the difference is the
    // un-baselined prefix.
    let lost_pct = 100.0 * summary.frames_lost as f64 / published as f64;
    let missing = published.saturating_sub(summary.messages);
    let missing_pct = 100.0 * missing as f64 / published as f64;
    println!("\n=== synthetic firehose ===");
    println!(
        "topics={topics_n} borrow={borrow} queue={queue_depth} body={body_len}B \
         target={hz}Hz/topic window={secs}s"
    );
    println!("publish wall      : {publish_wall:?}");
    println!("published frames  : {published}");
    println!("offered rate      : {rate_mb_s:.2} MB/s");
    println!("recorded          : {}", summary.messages);
    println!("frames_lost       : {}", summary.frames_lost);
    println!("dropped_unwritten: {}", summary.dropped_unwritten);
    println!("LOSS (gap-based)  : {lost_pct:.2} %");
    println!("LOSS (published-recorded) : {missing} frames = {missing_pct:.2} %");
    println!("chunks            : {}", summary.chunks);
    println!("=================================\n");

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}
