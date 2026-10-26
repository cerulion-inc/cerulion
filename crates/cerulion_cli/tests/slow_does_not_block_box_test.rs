// SPDX-License-Identifier: AGPL-3.0-only
//! SLOW DOES NOT BLOCK: a slow node never holds a fast one unless a `block`
//! edge between them says so.
//!
//! The graph: `pace` (Period 1 ms) publishes `out` and `side`, each frame
//! stamped with its publish time; `fast` fires on every `out` frame and
//! records `(fire time, publish-to-fire latency)`; `slow` fires on every `side`
//! frame and sleeps `ri_sleep_us` per fire. The fast node's fire RATE and
//! latency P99 are measured over a FIXED window of each run, taken on the
//! host monotonic clock the nodes stamp with: from [`WARMUP`] after the run is
//! up to [`COOLDOWN`] before the Ctrl-C. The rate is the fires in that window
//! over its length, so a fast node that stalls or starts late anywhere in it
//! reads slow; its first and last fires must lie within [`EDGE_SLACK`] of the
//! window's ends.
//!
//! ## The arms
//!
//! * `a_slow_node_in_another_process_does_not_slow_the_fast_node`: three
//!   processes, three legs on one host, back to back:
//!   1. BASELINE, `slow` sleeping 0;
//!   2. CONTROL, the slow node behind a declared `block` edge
//!      (`ri_slow_block`, depth 2), so `pace` is deferred while two `side`
//!      frames are unserved. The control must cut the fast node's rate to at
//!      most half the baseline: that is what proves the measurement can see a
//!      block at all;
//!   3. SLOW, `slow` sleeping 50 ms on the default `drop_oldest` input. It must
//!      keep at least 90% of the baseline rate and a P99 within
//!      `2 x baseline + 1 ms`.
//! * `a_slow_node_in_the_same_process_holds_the_fast_node_today`: the same
//!   graph under `--single-process`, BASELINE then SLOW. Nodes that share a
//!   process run on one level-by-level step loop, so a 50 ms tick holds every
//!   node of the process; the arm asserts that coupling (SLOW at most half the
//!   baseline rate).
//!
//! Each leg prints one line:
//!
//! ```text
//! SLOW_NB arm=<cross_process|same_process> leg=<baseline|slow|control> fires=<n> rate_hz=<r> p50_us=<u> p99_us=<u>
//! ```
//!
//! ## Scope and how to run
//!
//! Timing arms: `#[ignore]` (they need a quiet Linux host) and `#[serial]`.
//!
//! ```bash
//! cargo test -p cerulion_cli --test slow_does_not_block_box_test -- --ignored --nocapture --test-threads=1
//! ```

#![cfg(not(target_os = "macos"))]
// WAIVED WHOLE on macOS: upstream iceoryx2 0.10.0 defect 2034. Every arm spawns a
// `cerulion` supervisor child that loads plugin nodes, and on macOS such a process
// cannot create any further event resource. The mechanism, the derivation that
// selects this file, and the coverage this costs are stated once in
// `cerulion_core/tests/upstream_waivers_test.rs`. Runs normally on Linux.
#![cfg(unix)]

mod det_scaffold;
mod mp_support;
mod read_identity;

use std::path::Path;
use std::time::{Duration, Instant};

use cerulion_core::graph::params::ParamValue;
use read_identity::{graph_config, node_def, unique_names, write_graph, SAMPLES_DIR_ENV};
use serial_test::serial;

/// How long the slow node sleeps per fire.
const SLOW_SLEEP_US: u64 = 50_000;
/// Bring-up allowance before the measured window starts.
const BRING_UP: Duration = Duration::from_secs(5);
/// The measured window.
const WINDOW: Duration = Duration::from_secs(6);
/// The measured window starts this long after the run is up.
const WARMUP: Duration = Duration::from_secs(2);
/// The measured window ends this long before the Ctrl-C.
const COOLDOWN: Duration = Duration::from_millis(500);
/// How far inside the window's ends the fast node's first and last fires in
/// it must lie (a 1 kHz node fires many times inside this; a stalled one
/// does not).
const EDGE_SLACK: Duration = Duration::from_millis(100);
/// A baseline leg below this many steady samples measured nothing.
const MIN_BASELINE_SAMPLES: usize = 2000;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Slow {
    /// The `ri_sink` node, drop_oldest, sleeping this long per fire.
    DropOldest(u64),
    /// The `ri_slow_block` node behind a `block` edge, sleeping this long.
    Block(u64),
}

/// The fast node's steady-state numbers for one leg.
#[derive(Debug, Clone, Copy)]
struct Leg {
    fires: usize,
    rate_hz: f64,
    p50_ns: u64,
    p99_ns: u64,
    /// The first and last fires in the window lie within [`EDGE_SLACK`] of
    /// its ends.
    edges_ok: bool,
}

/// The fast node's numbers over `[from_ns, to_ns]`, from `fast_<pid>.samples`
/// under `dir` (exactly one file).
fn leg_numbers(dir: &Path, from_ns: u64, to_ns: u64) -> Leg {
    let files: Vec<_> = std::fs::read_dir(dir)
        .expect("RI_HARNESS: samples dir")
        .flatten()
        .filter(|e| e.file_name().to_string_lossy().ends_with(".samples"))
        .collect();
    assert_eq!(
        files.len(),
        1,
        "RI_HARNESS: expected one fast node sample file under {}",
        dir.display()
    );
    let text = std::fs::read_to_string(files[0].path()).expect("RI_HARNESS: read samples");
    let steady: Vec<(u64, u64)> = text
        .lines()
        .filter_map(|l| {
            let (t, lat) = l.split_once(' ')?;
            Some((t.parse().ok()?, lat.parse().ok()?))
        })
        .filter(|(t, _)| *t >= from_ns && *t <= to_ns)
        .collect();
    let slack = EDGE_SLACK.as_nanos() as u64;
    let edges_ok = steady.first().is_some_and(|f| f.0 <= from_ns + slack)
        && steady.last().is_some_and(|l| l.0 + slack >= to_ns);
    let span = (to_ns - from_ns) as f64 / 1e9;
    let mut lat: Vec<u64> = steady.iter().map(|(_, l)| *l).collect();
    lat.sort_unstable();
    let pct = |p: f64| {
        lat.get(((lat.len().max(1) - 1) as f64 * p).round() as usize)
            .copied()
            .unwrap_or(u64::MAX)
    };
    Leg {
        fires: steady.len(),
        rate_hz: steady.len() as f64 / span,
        p50_ns: pct(0.50),
        p99_ns: pct(0.99),
        edges_ok,
    }
}

/// Run one leg: the graph with `slow`, as declared (three processes) or
/// `--single-process`, and return the fast node's numbers.
fn run_leg(arm: &str, leg: &str, slow: Slow, single_process: bool) -> Leg {
    let ws = read_identity::workspace();
    let (graph, prefix) = unique_names("nb");
    let slow_node = match slow {
        Slow::DropOldest(us) => node_def(
            "slow",
            "ri_sink",
            &[("inp", "pace/side")],
            &[],
            &[
                ("ri_id", ParamValue::Str("slow".into())),
                ("ri_sleep_us", ParamValue::UInt(us)),
            ],
        ),
        Slow::Block(us) => node_def(
            "slow",
            "ri_slow_block",
            &[("inp", "pace/side")],
            &[],
            &[("ri_sleep_us", ParamValue::UInt(us))],
        ),
    };
    let nodes = vec![
        node_def("pace", "ri_pace", &[], &["out", "side"], &[]),
        node_def("fast", "ri_fast", &[("inp", "pace/out")], &[], &[]),
        slow_node,
    ];
    let groups = vec![
        ("g_pace", vec!["pace"]),
        ("g_fast", vec!["fast"]),
        ("g_slow", vec!["slow"]),
    ];
    write_graph(ws, &graph_config(&graph, &prefix, nodes, &groups, vec![]));
    let dir = tempfile::tempdir().expect("RI_HARNESS: run dir");
    let samples = dir.path().join("samples");
    std::fs::create_dir_all(&samples).expect("RI_HARNESS: samples dir");
    let extra: &[&str] = if single_process {
        &["--single-process"]
    } else {
        &[]
    };
    let start = Instant::now();
    let ready = move || start.elapsed() >= BRING_UP;
    let live = read_identity::run_live(
        ws,
        &graph,
        extra,
        &[(SAMPLES_DIR_ENV, &samples)],
        dir.path(),
        &ready,
        WINDOW,
    );
    let n = leg_numbers(
        &samples,
        live.ready_ns + WARMUP.as_nanos() as u64,
        live.stop_ns - COOLDOWN.as_nanos() as u64,
    );
    eprintln!(
        "SLOW_NB arm={arm} leg={leg} fires={} rate_hz={:.1} p50_us={} p99_us={} edges_ok={}",
        n.fires,
        n.rate_hz,
        n.p50_ns / 1000,
        n.p99_ns / 1000,
        n.edges_ok
    );
    n
}

fn assert_baseline_measured(base: &Leg) {
    assert!(
        base.edges_ok && base.fires >= MIN_BASELINE_SAMPLES && base.rate_hz >= 500.0,
        "RI_HARNESS: the baseline fast node fired {} times at {:.1} Hz; a 1 ms pacer should \
         give about 1 kHz, so this host measured nothing",
        base.fires,
        base.rate_hz
    );
}

#[test]
#[ignore = "needs a quiet Linux host: timing"]
#[serial]
fn a_slow_node_in_another_process_does_not_slow_the_fast_node() {
    const ARM: &str = "cross_process";
    let base = run_leg(ARM, "baseline", Slow::DropOldest(0), false);
    assert_baseline_measured(&base);
    let control = run_leg(ARM, "control", Slow::Block(SLOW_SLEEP_US), false);
    assert!(
        control.rate_hz <= base.rate_hz * 0.5,
        "RI_HARNESS: the block control did not slow the fast node ({:.1} Hz against a {:.1} Hz \
         baseline), so this measurement cannot see a block and proves nothing about the \
         slow leg",
        control.rate_hz,
        base.rate_hz
    );
    let slow = run_leg(ARM, "slow", Slow::DropOldest(SLOW_SLEEP_US), false);
    assert!(
        slow.edges_ok,
        "the fast node did not fire across the whole window beside a slow node in another \
         process (it started late or stalled)"
    );
    assert!(
        slow.rate_hz >= base.rate_hz * 0.9,
        "a slow node in another process cut the fast node's rate: {:.1} Hz against a {:.1} Hz \
         baseline",
        slow.rate_hz,
        base.rate_hz
    );
    let bound = base.p99_ns * 2 + 1_000_000;
    assert!(
        slow.p99_ns <= bound,
        "a slow node in another process raised the fast node's latency p99 to {} us (baseline \
         {} us, bound {} us)",
        slow.p99_ns / 1000,
        base.p99_ns / 1000,
        bound / 1000
    );
}

#[test]
#[ignore = "needs a quiet Linux host: timing"]
#[serial]
fn a_slow_node_in_the_same_process_holds_the_fast_node_today() {
    const ARM: &str = "same_process";
    let base = run_leg(ARM, "baseline", Slow::DropOldest(0), true);
    assert_baseline_measured(&base);
    let slow = run_leg(ARM, "slow", Slow::DropOldest(SLOW_SLEEP_US), true);
    assert!(
        slow.rate_hz <= base.rate_hz * 0.5,
        "the slow node did not hold the fast node in one process ({:.1} Hz against a {:.1} Hz \
         baseline): nodes in one process do not share one step loop",
        slow.rate_hz,
        base.rate_hz
    );
}
