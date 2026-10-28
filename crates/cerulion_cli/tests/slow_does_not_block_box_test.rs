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
//! Every leg's window must also measure something before any rate is
//! compared (`read_identity::check_fire_window`): a fire in each of its four
//! equal parts, and a fire-count floor. A full-rate leg (BASELINE, and SLOW in
//! another process) needs half the fires its reference rate gives over the
//! window (the 1 kHz pacer for BASELINE, the measured baseline rate for SLOW);
//! a leg that slows the fast node (CONTROL, SLOW in one process) needs
//! [`SLOWED_MIN_FIRES`]. An empty or stalled leg fails `RI_HARNESS:`, so it
//! never reads as "cut" or "held".
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
//! SLOW_NB arm=<cross_process|same_process> leg=<baseline|slow|control> fires=<n> rate_hz=<r> p50_us=<u> p99_us=<u> edges_ok=<b> parts=[<n>; 4]
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
mod scaffold_dir;

use std::path::Path;
use std::time::{Duration, Instant};

use cerulion_core::graph::params::ParamValue;
use read_identity::{
    check_fire_window, fire_window, graph_config, node_def, unique_names, write_graph, FireWindow,
    Workspace, SAMPLES_DIR_ENV,
};
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
/// The pacer's rate: one `out` frame per 1 ms period.
const PACER_HZ: f64 = 1000.0;
/// A leg that keeps the fast node at full rate must hold at least this share
/// of the fires the reference rate gives over its window.
const FULL_RATE_FLOOR: f64 = 0.5;
/// A leg that slows the fast node (a 50 ms sleep gives about 20 Hz, some 70
/// fires per window) must still hold this many fires in its window.
const SLOWED_MIN_FIRES: usize = 10;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Slow {
    /// The `ri_sink` node, drop_oldest, sleeping this long per fire.
    DropOldest(u64),
    /// The `ri_slow_block` node behind a `block` edge, sleeping this long.
    Block(u64),
}

/// The fast node's [`FireWindow`] over `[from_ns, to_ns]`, from
/// `fast_<pid>.samples` under `dir` (exactly one file).
fn leg_numbers(dir: &Path, from_ns: u64, to_ns: u64) -> FireWindow {
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
    let samples: Vec<(u64, u64)> = text
        .lines()
        .filter_map(|l| {
            let (t, lat) = l.split_once(' ')?;
            Some((t.parse().ok()?, lat.parse().ok()?))
        })
        .collect();
    fire_window(&samples, from_ns, to_ns, EDGE_SLACK.as_nanos() as u64)
}

/// The fires `rate_hz` gives over `w`'s window, times [`FULL_RATE_FLOOR`].
fn full_rate_floor(w: &FireWindow, rate_hz: f64) -> usize {
    let secs = (w.to_ns - w.from_ns) as f64 / 1e9;
    (rate_hz * secs * FULL_RATE_FLOOR) as usize
}

/// Panic `RI_HARNESS:` unless the leg's window passes [`check_fire_window`]
/// at `floor`.
fn require_window(arm: &str, leg: &str, w: &FireWindow, floor: usize) {
    if let Err(e) = check_fire_window(w, floor) {
        panic!(
            "RI_HARNESS: arm {arm} leg {leg}: the fast node's window failed its fire check: {e}"
        );
    }
}

/// Run one leg in `ws`: the graph with `slow`, as declared (three processes)
/// or `--single-process`, and return the fast node's numbers.
fn run_leg(ws: &Workspace, arm: &str, leg: &str, slow: Slow, single_process: bool) -> FireWindow {
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
        "SLOW_NB arm={arm} leg={leg} fires={} rate_hz={:.1} p50_us={} p99_us={} edges_ok={} parts={:?}",
        n.fires,
        n.rate_hz,
        n.p50_ns / 1000,
        n.p99_ns / 1000,
        n.edges_ok,
        n.parts
    );
    n
}

fn assert_baseline_measured(arm: &str, base: &FireWindow) {
    require_window(arm, "baseline", base, full_rate_floor(base, PACER_HZ));
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
    let ws = &read_identity::workspace();
    let base = run_leg(ws, ARM, "baseline", Slow::DropOldest(0), false);
    assert_baseline_measured(ARM, &base);
    let control = run_leg(ws, ARM, "control", Slow::Block(SLOW_SLEEP_US), false);
    require_window(ARM, "control", &control, SLOWED_MIN_FIRES);
    assert!(
        control.rate_hz <= base.rate_hz * 0.5,
        "RI_HARNESS: the block control did not slow the fast node ({:.1} Hz against a {:.1} Hz \
         baseline), so this measurement cannot see a block and proves nothing about the \
         slow leg",
        control.rate_hz,
        base.rate_hz
    );
    let slow = run_leg(ws, ARM, "slow", Slow::DropOldest(SLOW_SLEEP_US), false);
    require_window(ARM, "slow", &slow, full_rate_floor(&slow, base.rate_hz));
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
    let ws = &read_identity::workspace();
    let base = run_leg(ws, ARM, "baseline", Slow::DropOldest(0), true);
    assert_baseline_measured(ARM, &base);
    let slow = run_leg(ws, ARM, "slow", Slow::DropOldest(SLOW_SLEEP_US), true);
    require_window(ARM, "slow", &slow, SLOWED_MIN_FIRES);
    assert!(
        slow.rate_hz <= base.rate_hz * 0.5,
        "the slow node did not hold the fast node in one process ({:.1} Hz against a {:.1} Hz \
         baseline): nodes in one process do not share one step loop",
        slow.rate_hz,
        base.rate_hz
    );
}
