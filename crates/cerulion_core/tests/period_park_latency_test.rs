// SPDX-License-Identifier: AGPL-3.0-only
//! The period-park's wall-clock bounds, release only, on Linux.
//!
//! Three claims about time, each measured where the period-park engages (a CPU
//! with a monitor-wait primitive and, for an input written from another
//! runtime, a doorbell wake word):
//!
//! - **A ring ends a kernel nap at once**: the slowest of 16 frames from another
//!   runtime, 3 ms apart, is read within 2 ms of its write.
//! - **A frame on a second input is read within about one nap**: its median
//!   delay is at most five naps (500 us) and its slowest at most 5 ms.
//! - **A 1 kHz node keeps its grid under the period-park**: after the start-up
//!   catch-up, at most 2 % of fires land less than half a period after the
//!   previous one, and the median gap is 850 to 1150 us.
//!
//! All three judge wall time, so they live in the release latency tier: the
//! `test-latency` job runs this file on pushes to main
//! (`cargo test -p cerulion_core --test period_park_latency_test --release --
//! --nocapture --test-threads=1`), and no pull-request shard does, because a
//! loaded shared runner delays a wake and stretches every bound. The counts
//! and the order of the same runs are pinned on every pull request by
//! `period_park_iox2_test`. A green run prints each arm's measured figures.

#![cfg(all(not(debug_assertions), target_os = "linux"))]

#[path = "period_park_support/mod.rs"]
mod support;

use std::time::Duration;

use cerulion_core::graph::PeriodParkStats;
use serial_test::serial;
use support::{
    applies_to_a_foreign_consumer, engages_here, ticker_run, two_runtimes, DUE_PARK_TIMEOUT,
};

/// The longest the period-park sleeps in the kernel at once (`period_park::NAP`).
const NAP: Duration = Duration::from_micros(100);

fn median(mut v: Vec<u64>) -> u64 {
    v.sort_unstable();
    v[v.len() / 2]
}

#[test]
#[serial]
fn a_ring_from_another_runtime_is_read_within_two_milliseconds() {
    const FRAMES: u64 = 16;
    let c = two_runtimes(
        "ringlat",
        &["/ppk/ringlat/feed"],
        "/ppk/ringlat/feed",
        FRAMES,
        Duration::from_millis(3),
    );
    assert!(
        c.seen.len() as u64 >= FRAMES,
        "read {} of {FRAMES} frames",
        c.seen.len()
    );
    assert_eq!(
        c.timeouts_while_due, 0,
        "no wait while a frame was due ended on its {DUE_PARK_TIMEOUT:?} timeout"
    );
    let delays: Vec<u64> = c.seen.iter().map(|(_, d)| *d).collect();
    let worst = Duration::from_nanos(delays.iter().max().copied().unwrap_or(0));
    println!(
        "period-park ring: median delay {:?}, worst {worst:?}, {:?}",
        Duration::from_nanos(median(delays.clone())),
        c.stats
    );
    if applies_to_a_foreign_consumer() {
        assert!(
            worst <= Duration::from_millis(2),
            "the slowest frame was read {worst:?} after it was written"
        );
    } else {
        assert_eq!(c.stats, PeriodParkStats::default());
    }
}

#[test]
#[serial]
fn a_frame_on_a_second_input_is_read_within_about_one_nap() {
    const FRAMES: u64 = 16;
    let c = two_runtimes(
        "secondlat",
        &["/ppk/secondlat/a", "/ppk/secondlat/b"],
        "/ppk/secondlat/b",
        FRAMES,
        Duration::from_millis(3),
    );
    assert!(
        c.seen.len() as u64 >= FRAMES,
        "read {} of {FRAMES} frames",
        c.seen.len()
    );
    assert_eq!(
        c.timeouts_while_due, 0,
        "no wait while a frame was due ended on its {DUE_PARK_TIMEOUT:?} timeout"
    );
    let delays: Vec<u64> = c.seen.iter().map(|(_, d)| *d).collect();
    let med = Duration::from_nanos(median(delays.clone()));
    let worst = Duration::from_nanos(delays.iter().max().copied().unwrap_or(0));
    println!(
        "period-park second input: median delay {med:?}, worst {worst:?}, {:?}",
        c.stats
    );
    if applies_to_a_foreign_consumer() {
        // One nap plus the timer's slack and the step: a generous bound.
        assert!(
            med <= NAP * 5,
            "the median frame on the second input was read {med:?} after it was written"
        );
        assert!(
            worst <= Duration::from_millis(5),
            "the slowest was read {worst:?} after"
        );
    } else {
        assert_eq!(c.stats, PeriodParkStats::default());
    }
}

#[test]
#[serial]
fn a_period_node_under_the_period_park_stays_on_its_grid() {
    const WARMUP: usize = 100;
    const FIRES: usize = 800;
    let (ticks, _, stats) = ticker_run("gridlat", true, WARMUP + FIRES);
    assert!(ticks.len() >= WARMUP + FIRES, "ran {} fires", ticks.len());
    let gaps: Vec<u64> = ticks[WARMUP..]
        .windows(2)
        .map(|w| w[1].saturating_sub(w[0]))
        .collect();
    let doubles = gaps.iter().filter(|g| **g < 500_000).count();
    let med = Duration::from_nanos(median(gaps.clone()));
    println!(
        "period-park grid: {doubles} of {} fires under half a period after the last, median gap {med:?}, {stats:?}",
        gaps.len()
    );
    if engages_here() {
        assert!(
            doubles * 50 <= gaps.len(),
            "{doubles} of {} fires came less than half a period after the previous one",
            gaps.len()
        );
        assert!(
            med >= Duration::from_micros(850) && med <= Duration::from_micros(1_150),
            "the median gap is {med:?}"
        );
    } else {
        assert_eq!(stats, PeriodParkStats::default());
    }
}
