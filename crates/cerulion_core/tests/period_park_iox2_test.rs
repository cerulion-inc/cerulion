// SPDX-License-Identifier: AGPL-3.0-only
//! The live loop's period-park (`CERULION_PERIOD_PARK=1`) over real iceoryx2.
//!
//! The switch is forced per runtime through `set_period_park_for_test`, so the
//! process environment is never touched. Each arm is judged against the host
//! facts the period-park is gated on: it engages only where the CPU carries a
//! monitor-wait primitive (and, for an input written from another runtime, the
//! doorbell has a wake word). On any other host the arm asserts the switch did
//! nothing, so the binary runs a real test everywhere. The refusal arm judges a
//! runtime's wiring alone, with the CPU's monitor-wait primitive taken as held,
//! so it asserts the same reasons on every host.
//!
//! Every arm here judges counts and order, never wall time: how long after its
//! write a frame is read, and whether a 1 kHz node keeps its grid, are measured
//! in the release latency tier by `period_park_latency_test` (release only, on
//! Linux, run by the `test-latency` job on pushes to main). Per-test SHM roots;
//! `#[serial]` because the arms run live.

#[path = "period_park_support/mod.rs"]
mod support;

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::{GraphRuntime, PeriodParkStats};
use cerulion_core::prelude::*;
use cerulion_core::{MonitorWaitPolicy, RealClock};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use support::{
    applies_to_a_foreign_consumer, config, engages_here, input, node, ticker_run, transport,
    two_runtimes, DUE_PARK_TIMEOUT,
};

/// A data-triggered node that reads its input and does nothing else.
#[cerulion_node]
#[derive(Default)]
struct Sink {
    #[input(trigger)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl Sink {
    fn tick(&mut self) -> Result<(), NodeError> {
        Ok(())
    }
}

/// The reason a requested period-park would not engage on a runtime whose one
/// node reads `feed` (a topic another graph writes), with the doorbell on or off.
fn refusal_reading(tag: &str, feed: &str, doorbell: bool) -> Option<&'static str> {
    let mgr = transport(
        &format!("ppk_{tag}"),
        cerulion_core::testing::iceoryx_test_config(),
        Arc::new(RealClock),
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "sink".to_string(),
        Box::new(SinkEntry::with_state(Sink::default())),
    );
    let mut rt = GraphRuntime::build_live_with_schema_hashes_and_policy(
        config(
            &format!("ppk_{tag}"),
            &format!("ppk{tag}"),
            vec![node("sink", "sink", vec![input(feed)], vec![])],
        ),
        factories,
        &mgr,
        Arc::new(RealClock),
        None,
        MonitorWaitPolicy::new(true, doorbell, format!("ppk{}{tag}", std::process::id())),
    )
    .expect("build the graph");
    let reason = rt.period_park_wiring_refusal_for_test();
    rt.shutdown();
    drop(mgr);
    reason
}

/// An input written from outside the runtime with no doorbell wake word (the
/// doorbell off) refuses the period-park with `no_data_wake`, as its warning
/// says: no line is armed, so nothing could end a nap on that input. The gate
/// counts the inputs written from outside at build, whatever the doorbell
/// policy. Builds only, and the wiring is judged on every host: the seam takes
/// the CPU's monitor-wait primitive as held.
#[test]
#[serial]
fn an_input_written_from_outside_with_the_doorbell_off_refuses_the_period_park() {
    assert_eq!(
        refusal_reading("dboff", "/ppk/dboff/feed", false),
        Some("no_data_wake")
    );
    // The doorbell on: the input's line is armed, and the wake word decides.
    let want = if cerulion_core::doorbell::wake_word_block_primitive_available() {
        None
    } else {
        Some("no_data_wake")
    };
    assert_eq!(refusal_reading("dbon", "/ppk/dbon/feed", true), want);
}

/// A ring from another runtime ends the consumer's kernel nap at once: every
/// frame is read in order, no wait ends on its long park timeout, and a ring
/// demonstrably cut a nap short.
#[test]
#[serial]
fn a_ring_from_another_runtime_ends_a_kernel_nap() {
    const FRAMES: u64 = 16;
    let c = two_runtimes(
        "ring",
        &["/ppk/ring/feed"],
        "/ppk/ring/feed",
        FRAMES,
        Duration::from_millis(3),
    );
    let numbers: Vec<f64> = c.seen.iter().map(|(n, _)| *n).collect();
    let expected: Vec<f64> = (1..=numbers.len() as u64).map(|n| n as f64).collect();
    assert!(
        numbers.len() as u64 >= FRAMES,
        "read {} of {FRAMES} frames",
        numbers.len()
    );
    assert_eq!(numbers, expected, "every frame, in order");
    assert_eq!(
        c.timeouts_while_due, 0,
        "every wait while a frame was due ended on a data wake, not on its \
         {DUE_PARK_TIMEOUT:?} timeout"
    );
    eprintln!("ring: {:?}", c.stats);
    if applies_to_a_foreign_consumer() {
        assert!(
            c.stats.blocks > 0 && c.stats.kernel_sleeps > 0,
            "{:?}",
            c.stats
        );
        assert!(
            c.stats.kernel_sleeps_cut_by_data > 0,
            "a producer ring must end a kernel nap before its end: {:?}",
            c.stats
        );
    } else {
        assert_eq!(
            c.stats,
            PeriodParkStats::default(),
            "the period-park does not apply here"
        );
    }
}

/// A ring on an input topic other than the armed one does not end a kernel
/// nap (one process blocks on one wake word); the loop top reads every input
/// after each nap, so every frame on that input is still read, in order.
#[test]
#[serial]
fn a_ring_on_a_second_input_does_not_end_a_nap_and_every_frame_is_read() {
    const FRAMES: u64 = 16;
    let c = two_runtimes(
        "second",
        &["/ppk/second/a", "/ppk/second/b"],
        "/ppk/second/b",
        FRAMES,
        Duration::from_millis(3),
    );
    let numbers: Vec<f64> = c.seen.iter().map(|(n, _)| *n).collect();
    let expected: Vec<f64> = (1..=numbers.len() as u64).map(|n| n as f64).collect();
    assert!(
        numbers.len() as u64 >= FRAMES,
        "read {} of {FRAMES} frames",
        numbers.len()
    );
    assert_eq!(
        numbers, expected,
        "every frame on the second input, in order"
    );
    eprintln!("second input: {:?}", c.stats);
    if applies_to_a_foreign_consumer() {
        assert_eq!(
            c.stats.kernel_sleeps_cut_by_data, 0,
            "a ring on a line the nap is not blocked on cannot end the nap: {:?}",
            c.stats
        );
        assert!(c.stats.kernel_sleeps > 0, "{:?}", c.stats);
    } else {
        assert_eq!(
            c.stats,
            PeriodParkStats::default(),
            "the period-park does not apply here"
        );
    }
}

/// Under the period-park a 1 kHz node sleeps between its ticks and wakes for
/// its deadlines: the period-park planned blocks, slept in the kernel and
/// returned on the timer. Where it does not apply, the switch did nothing.
#[test]
#[serial]
fn a_period_node_under_the_period_park_sleeps_between_its_ticks() {
    const FIRES: usize = 300;
    let (ticks, _, stats) = ticker_run("sleeps", true, FIRES);
    assert!(ticks.len() >= FIRES, "ran {} fires", ticks.len());
    eprintln!("sleeps: {stats:?}");
    if engages_here() {
        assert!(
            stats.blocks > 0 && stats.kernel_sleeps > 0 && stats.timer_wakes > 0,
            "{stats:?}"
        );
    } else {
        assert_eq!(
            stats,
            PeriodParkStats::default(),
            "the period-park does not apply here"
        );
    }
}

/// With the switch off the period-park never runs: its counters stay at zero.
#[test]
#[serial]
fn with_the_switch_off_the_period_park_never_runs() {
    let (ticks, _, stats) = ticker_run("off", false, 60);
    assert!(ticks.len() >= 60, "apparatus: ran {} fires", ticks.len());
    assert_eq!(stats, PeriodParkStats::default());
}

/// The period-park changes when the loop re-polls, never what fires: past the
/// start-up catch-up (whose burst can overrun the consumer's queue in either
/// mode), the consumer reads every numbered frame, in order, with the switch on
/// as with it off.
#[test]
#[serial]
fn the_period_park_changes_when_not_what() {
    const FIRES: usize = 300;
    const WARMUP: f64 = 100.0;
    for on in [true, false] {
        let (ticks, seen, _) = ticker_run(if on { "whaton" } else { "whatoff" }, on, FIRES);
        assert!(ticks.len() >= FIRES, "apparatus: ran {} fires", ticks.len());
        let after: Vec<f64> = seen.iter().copied().filter(|n| *n > WARMUP).collect();
        assert!(
            after.len() >= FIRES - 120,
            "switch on={on}: read {} frames past the warm-up",
            after.len()
        );
        let first = after[0];
        let expected: Vec<f64> = (0..after.len()).map(|k| first + k as f64).collect();
        assert_eq!(
            after, expected,
            "switch on={on}: every frame past the warm-up, in order"
        );
    }
}
