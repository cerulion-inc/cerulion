// SPDX-License-Identifier: AGPL-3.0-only
//! EXECUTOR UNITS: nodes that share a process run on independent executors, so
//! a slow node in the process does not hold a fast one unless a `block` edge
//! between them says so; and a run on those executors replays.
//!
//! The graph: `pace` (Period 1 ms) publishes `out` and `side`; `fast` fires on
//! every `out` frame and records `(fire time, publish-to-fire latency)`; `slow`
//! fires on every `side` frame and sleeps `ri_sleep_us` per fire. All three
//! nodes are ONE declared process group, so one worker process hosts them.
//! `pace` fans out to two nodes, so the process splits into three units:
//! `pace`, `fast` and `slow`, each on its own thread.
//!
//! ## The arms
//!
//! * `a_slow_node_in_the_same_process_does_not_slow_the_fast_node`: three legs
//!   back to back on one host:
//!   1. BASELINE, `slow` sleeping 0;
//!   2. CONTROL, `slow` behind a declared `block` edge (`ri_slow_block`, depth
//!      2): the block flow keeps `pace` and `slow` in one unit, and `pace` is
//!      deferred while two `side` frames are unserved, so the fast node's rate
//!      must drop to at most half the baseline. This proves the measurement can
//!      see a hold at all;
//!   3. SLOW, `slow` sleeping 50 ms on its default `drop_oldest` input: the
//!      fast node keeps at least 90% of the baseline rate and a latency P99
//!      within `2 x baseline + 1 ms`.
//! * `the_unit_collapse_mutant_is_caught` (`oracle-mutants` feature): the SLOW
//!   leg with `CERULION_UNIT_COLLAPSE_MUTANT` set, which runs the process as
//!   one level loop again. The SLOW bound must fail: at most half the baseline
//!   rate.
//! * `without_a_profile_a_slow_tail_does_not_slow_the_chain_that_feeds_it`:
//!   `pace -> fast -> slow` as one linear chain in one process, no profile.
//!   Every data-triggered node is a unit of its own, so the SLOW bound holds;
//!   `the_unprofiled_merge_mutant_is_caught` (`oracle-mutants`) merges the
//!   chain without a profile and must fail it.
//! * `a_run_on_executor_units_replays_like_the_same_graph_in_separate_processes`:
//!   the fan-out and three-rank fixtures recorded with every node in one
//!   process group, re-executed with `bag play --resim all`, and judged with the
//!   read-identity oracle. Every stream's first live read must be its writer's
//!   first frame, and every stream that evicted nothing live must judge the
//!   same as the same fixture recorded process per node, whose bag has the same
//!   rank shape (an evicting stream is printed, not compared: a replay through
//!   a real queue reproduces no eviction under any placement).
//!
//! Each timing leg prints one line:
//!
//! ```text
//! UNITS_NB leg=<baseline|control|slow|mutant> fires=<n> rate_hz=<r> p50_us=<u> p99_us=<u>
//! ```
//!
//! ## Scope and how to run
//!
//! Every arm is `#[ignore]` (box-only, a quiet Linux host) and `#[serial]`.
//!
//! ```bash
//! cargo test -p cerulion_cli --test executor_units_box_test -- --ignored --nocapture --test-threads=1
//! cargo test -p cerulion_cli --features oracle-mutants --test executor_units_box_test \
//!     the_unit_collapse_mutant_is_caught -- --ignored --nocapture --test-threads=1
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
use read_identity::{
    graph_config, judge_replayed, node_def, unique_names, write_graph, Fixture, Floors, Placement,
    ReplayMode, RowVerdict, SAMPLES_DIR_ENV,
};
use serial_test::serial;

/// How long the slow node sleeps per fire.
const SLOW_SLEEP_US: u64 = 50_000;
/// Bring-up allowance before the measured window starts.
const BRING_UP: Duration = Duration::from_secs(5);
/// The measured window.
const WINDOW: Duration = Duration::from_secs(6);
/// Samples dropped from the start of the fast node's record.
const WARMUP: Duration = Duration::from_secs(2);
/// Samples dropped from the end of the fast node's record.
const COOLDOWN: Duration = Duration::from_millis(500);
/// A baseline leg below this many steady samples measured nothing.
const MIN_BASELINE_SAMPLES: usize = 2000;
/// A slow leg below this many steady samples measured nothing either.
const MIN_SLOW_SAMPLES: usize = 1000;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Slow {
    /// The `ri_sink` node, drop_oldest, sleeping this long per fire.
    DropOldest(u64),
    /// The `ri_slow_block` node behind a `block` edge, sleeping this long.
    Block(u64),
}

#[derive(Debug, Clone, Copy)]
struct Leg {
    fires: usize,
    rate_hz: f64,
    p50_ns: u64,
    p99_ns: u64,
}

/// Parse `fast_<pid>.samples` under `dir` (exactly one file) into the steady
/// part's numbers.
fn leg_numbers(dir: &Path) -> Leg {
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
    assert!(
        samples.len() >= 2,
        "RI_HARNESS: the fast node recorded {} sample(s)",
        samples.len()
    );
    let first = samples[0].0 + WARMUP.as_nanos() as u64;
    let last = samples[samples.len() - 1]
        .0
        .saturating_sub(COOLDOWN.as_nanos() as u64);
    let steady: Vec<(u64, u64)> = samples
        .into_iter()
        .filter(|(t, _)| *t >= first && *t <= last)
        .collect();
    assert!(
        steady.len() >= 2,
        "RI_HARNESS: no steady samples (the run was shorter than its warmup)"
    );
    let span = (steady[steady.len() - 1].0 - steady[0].0) as f64 / 1e9;
    let mut lat: Vec<u64> = steady.iter().map(|(_, l)| *l).collect();
    lat.sort_unstable();
    let pct = |p: f64| lat[((lat.len() - 1) as f64 * p).round() as usize];
    Leg {
        fires: steady.len(),
        rate_hz: (steady.len() - 1) as f64 / span,
        p50_ns: pct(0.50),
        p99_ns: pct(0.99),
    }
}

/// Run one leg with all three nodes in one process group and return the fast
/// node's numbers. `env` adds variables to the run.
fn run_leg(leg: &str, slow: Slow, env: &[(&str, &Path)]) -> Leg {
    let ws = read_identity::workspace();
    let (graph, prefix) = unique_names("units");
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
    let groups = vec![("g_all", vec!["pace", "fast", "slow"])];
    write_graph(ws, &graph_config(&graph, &prefix, nodes, &groups, vec![]));
    let dir = tempfile::tempdir().expect("RI_HARNESS: run dir");
    let samples = dir.path().join("samples");
    std::fs::create_dir_all(&samples).expect("RI_HARNESS: samples dir");
    let start = Instant::now();
    let ready = move || start.elapsed() >= BRING_UP;
    let mut vars: Vec<(&str, &Path)> = vec![(SAMPLES_DIR_ENV, &samples)];
    vars.extend_from_slice(env);
    let _ = read_identity::run_live(ws, &graph, &[], &vars, dir.path(), &ready, WINDOW);
    let n = leg_numbers(&samples);
    eprintln!(
        "UNITS_NB leg={leg} fires={} rate_hz={:.1} p50_us={} p99_us={}",
        n.fires,
        n.rate_hz,
        n.p50_ns / 1000,
        n.p99_ns / 1000
    );
    n
}

fn baseline() -> Leg {
    let base = run_leg("baseline", Slow::DropOldest(0), &[]);
    assert!(
        base.fires >= MIN_BASELINE_SAMPLES && base.rate_hz >= 500.0,
        "RI_HARNESS: the baseline fast node fired {} times at {:.1} Hz; a 1 ms pacer should \
         give about 1 kHz, so this host measured nothing",
        base.fires,
        base.rate_hz
    );
    base
}

/// The SLOW bound: the fast node keeps its rate and its P99.
fn slow_within_bound(base: &Leg, slow: &Leg) -> Result<(), String> {
    if slow.rate_hz < base.rate_hz * 0.9 {
        return Err(format!(
            "a slow node in the same process cut the fast node's rate: {:.1} Hz against a \
             {:.1} Hz baseline",
            slow.rate_hz, base.rate_hz
        ));
    }
    let bound = base.p99_ns * 2 + 1_000_000;
    if slow.p99_ns > bound {
        return Err(format!(
            "a slow node in the same process raised the fast node's latency p99 to {} us \
             (baseline {} us, bound {} us)",
            slow.p99_ns / 1000,
            base.p99_ns / 1000,
            bound / 1000
        ));
    }
    Ok(())
}

#[test]
#[ignore = "box-only: timing on a quiet Linux host"]
#[serial]
fn a_slow_node_in_the_same_process_does_not_slow_the_fast_node() {
    let base = baseline();
    let control = run_leg("control", Slow::Block(SLOW_SLEEP_US), &[]);
    assert!(
        control.rate_hz <= base.rate_hz * 0.5,
        "RI_HARNESS: the block control did not slow the fast node ({:.1} Hz against a {:.1} Hz \
         baseline), so this measurement cannot see a hold and proves nothing about the slow leg",
        control.rate_hz,
        base.rate_hz
    );
    let slow = run_leg("slow", Slow::DropOldest(SLOW_SLEEP_US), &[]);
    assert!(
        slow.fires >= MIN_SLOW_SAMPLES,
        "RI_HARNESS: the slow leg's fast node fired only {} times",
        slow.fires
    );
    if let Err(e) = slow_within_bound(&base, &slow) {
        panic!("{e}");
    }
}

#[cfg(feature = "oracle-mutants")]
#[test]
#[ignore = "box-only: timing on a quiet Linux host"]
#[serial]
fn the_unit_collapse_mutant_is_caught() {
    let base = baseline();
    let mutant = run_leg(
        "mutant",
        Slow::DropOldest(SLOW_SLEEP_US),
        &[(
            cerulion_cli_engine::multiprocess::UNIT_COLLAPSE_MUTANT_ENV,
            Path::new("1"),
        )],
    );
    assert!(
        slow_within_bound(&base, &mutant).is_err(),
        "the oracle passed with the process collapsed to one level loop: the slow-does-not-block \
         bound cannot see the coupling it exists to catch ({:.1} Hz against {:.1} Hz)",
        mutant.rate_hz,
        base.rate_hz
    );
    assert!(
        mutant.rate_hz <= base.rate_hz * 0.5,
        "the collapsed process held the fast node less than expected: {:.1} Hz against a {:.1} Hz \
         baseline",
        mutant.rate_hz,
        base.rate_hz
    );
}

/// Run one leg of the CHAIN shape, all three nodes in one process group and
/// no profile: `pace -> fast -> slow` is one linear chain, so only the
/// no-profile rule keeps `slow` off `fast`'s thread.
fn run_chain_leg(leg: &str, env: &[(&str, &Path)]) -> Leg {
    let ws = read_identity::workspace();
    let (graph, prefix) = unique_names("units_chain");
    let nodes = vec![
        node_def("pace", "ri_pace", &[], &["out", "side"], &[]),
        node_def(
            "fast",
            "ri_fast_relay",
            &[("inp", "pace/out")],
            &["out"],
            &[],
        ),
        node_def(
            "slow",
            "ri_sink",
            &[("inp", "fast/out")],
            &[],
            &[
                ("ri_id", ParamValue::Str("slow".into())),
                ("ri_sleep_us", ParamValue::UInt(SLOW_SLEEP_US)),
            ],
        ),
    ];
    let groups = vec![("g_all", vec!["pace", "fast", "slow"])];
    write_graph(ws, &graph_config(&graph, &prefix, nodes, &groups, vec![]));
    let dir = tempfile::tempdir().expect("RI_HARNESS: run dir");
    let samples = dir.path().join("samples");
    std::fs::create_dir_all(&samples).expect("RI_HARNESS: samples dir");
    let start = Instant::now();
    let ready = move || start.elapsed() >= BRING_UP;
    let mut vars: Vec<(&str, &Path)> = vec![(SAMPLES_DIR_ENV, &samples)];
    vars.extend_from_slice(env);
    let _ = read_identity::run_live(ws, &graph, &[], &vars, dir.path(), &ready, WINDOW);
    let n = leg_numbers(&samples);
    eprintln!(
        "UNITS_NB leg={leg} fires={} rate_hz={:.1} p50_us={} p99_us={}",
        n.fires,
        n.rate_hz,
        n.p50_ns / 1000,
        n.p99_ns / 1000
    );
    n
}

/// Without a profile every data-triggered node is a unit of its own, so a
/// slow tail of a linear chain does not hold the fast node at its head.
#[test]
#[ignore = "box-only: timing on a quiet Linux host"]
#[serial]
fn without_a_profile_a_slow_tail_does_not_slow_the_chain_that_feeds_it() {
    let base = baseline();
    let chain = run_chain_leg("chain_slow", &[]);
    assert!(
        chain.fires >= MIN_SLOW_SAMPLES,
        "RI_HARNESS: the chain leg's fast node fired only {} times",
        chain.fires
    );
    if let Err(e) = slow_within_bound(&base, &chain) {
        panic!("{e}");
    }
}

/// The merge mutant joins a link no profile proves safe: the chain is one
/// unit again, and the bound must fail.
#[cfg(feature = "oracle-mutants")]
#[test]
#[ignore = "box-only: timing on a quiet Linux host"]
#[serial]
fn the_unprofiled_merge_mutant_is_caught() {
    let base = baseline();
    let mutant = run_chain_leg(
        "merge_mutant",
        &[(
            cerulion_core::graph::executor::MERGE_MUTANT_ENV,
            Path::new("1"),
        )],
    );
    assert!(
        slow_within_bound(&base, &mutant).is_err(),
        "the oracle passed with an unprofiled chain merged into one unit ({:.1} Hz against \
         {:.1} Hz)",
        mutant.rate_hz,
        base.rate_hz
    );
}

/// Record `fixture` under `placement`, resim it, judge and print.
fn record_and_judge(
    fixture: Fixture,
    placement: Placement,
) -> (read_identity::RecordedRun, read_identity::Verdict) {
    let run = read_identity::record_live(fixture, placement);
    let replayed = read_identity::resim(&run, ReplayMode::ResimNeutral, None);
    let outcome = judge_replayed(fixture, &run.live, &replayed, Floors::default())
        .unwrap_or_else(|e| panic!("RI_HARNESS: {}: {e}", fixture.name()));
    outcome.print(
        fixture.name(),
        &format!("{}/{}", placement.label(), ReplayMode::ResimNeutral.label()),
    );
    let verdict = outcome
        .verdict()
        .cloned()
        .unwrap_or_else(|| panic!("RI_HARNESS: {}: the neutral resim refused", fixture.name()));
    (run, verdict)
}

/// Live reads that skip frames of their writer: a read whose sequence is not
/// the previous read's plus one from the same writer. A skip is an eviction,
/// which a replay that feeds a real queue does not reproduce whatever the
/// placement.
fn sequence_gaps(run: &read_identity::RecordedRun, node: &str, input: &str) -> usize {
    run.live
        .streams
        .get(&(node.to_string(), input.to_string()))
        .map_or(0, |reads| {
            reads
                .windows(2)
                .filter(|w| w[0].writer == w[1].writer && w[1].seq != w[0].seq + 1)
                .count()
        })
}

fn kind(v: &RowVerdict) -> &'static str {
    match v {
        RowVerdict::Equal => "equal",
        RowVerdict::Diverged { .. } => "diverged",
        RowVerdict::Short => "short",
        RowVerdict::Long => "long",
    }
}

#[test]
#[ignore = "box-only: real runs on a quiet Linux host"]
#[serial]
fn a_run_on_executor_units_replays_like_the_same_graph_in_separate_processes() {
    let mut compared = 0usize;
    for fixture in [Fixture::FanOut, Fixture::Ranks3] {
        let (run, units) = record_and_judge(fixture, Placement::OneGroup);
        let (_, processes) = record_and_judge(fixture, Placement::Declared);
        assert!(
            !units.rows.is_empty(),
            "RI_HARNESS: {} judged no stream",
            fixture.name()
        );
        for row in &units.rows {
            // Every unit had its ports built before any unit's first step: a
            // consumer's first live read is its writer's first frame.
            let reads = &run.live.streams[&(row.node.clone(), row.input.clone())];
            assert_eq!(
                reads.first().map(|r| r.seq),
                Some(1),
                "{}: {}.{}'s first live read is not its writer's first frame: a unit of the \
                 host started before its consumer's ports existed",
                fixture.name(),
                row.node,
                row.input
            );
            let gaps = sequence_gaps(&run, &row.node, &row.input);
            if gaps > 0 {
                eprintln!(
                    "UNITS_REPLAY fixture={} node={} input={} live_gaps={gaps} verdict={} \
                     (an evicting stream: replay through a real queue does not reproduce it \
                     under any placement, so it is not compared here)",
                    fixture.name(),
                    row.node,
                    row.input,
                    kind(&row.verdict)
                );
                continue;
            }
            let other = processes
                .rows
                .iter()
                .find(|r| r.node == row.node && r.input == row.input)
                .unwrap_or_else(|| {
                    panic!(
                        "RI_HARNESS: {}: stream {}.{} judged on executor units only",
                        fixture.name(),
                        row.node,
                        row.input
                    )
                });
            assert_eq!(
                kind(&row.verdict),
                kind(&other.verdict),
                "{}: stream {}.{} replays {} on executor units in one process but {} with one \
                 process per node\nunits:\n{}\nprocesses:\n{}",
                fixture.name(),
                row.node,
                row.input,
                kind(&row.verdict),
                kind(&other.verdict),
                units.render(),
                processes.render()
            );
            compared += 1;
        }
    }
    assert!(
        compared >= 3,
        "RI_HARNESS: only {compared} gap-free stream(s) were compared; the arm proves nothing"
    );
}
