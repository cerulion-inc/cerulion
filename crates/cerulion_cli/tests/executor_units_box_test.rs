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
//! * `a_run_on_executor_units_replays_like_the_same_graph_in_separate_processes_under_load`:
//!   the same comparison while busy-loop children (`CER_UNITS_LOAD_CHILDREN`,
//!   default half the CPUs) hold the host's CPUs busy; it prints one
//!   `UNDER_LOAD stressor=...` line naming the load.
//! * `a_co_hosted_relays_first_fire_costs_what_it_costs_in_its_own_process`:
//!   the three-rank chain recorded both ways; the relay's first fire co-hosted
//!   must take at most twice its own-process twin. A first fire longer than
//!   the relay's queue depth in source periods drains frames that arrived
//!   during it. `the_descriptor_presize_mutant_is_caught` (`oracle-mutants`)
//!   skips the descriptor-table pre-size of a process hosting several units
//!   and must fail it.
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
mod scaffold_dir;

use std::path::Path;
use std::time::{Duration, Instant};

use cerulion_core::graph::params::ParamValue;
use read_identity::{
    graph_config, judge_replayed, node_def, unique_names, write_graph, Fixture, Floors, Placement,
    ReplayMode, RowVerdict, Workspace, SAMPLES_DIR_ENV,
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
/// Live eviction runs a stream may show with its units in one process beyond
/// what the same stream shows with one process per node: none. A unit's queue
/// holds the same frames whichever process hosts it, so an extra eviction is a
/// consumer that ran later in its host than in a process of its own, which is
/// the co-hosting slow-down this comparison surfaces. Whether a consumer runs
/// later depends on the host's CPU contention, so an excess is printed
/// (`EVICTION_EXCESS`) and counted, never failed.
const EVICTION_TOLERANCE: usize = 0;
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
fn run_leg(ws: &Workspace, leg: &str, slow: Slow, env: &[(&str, &Path)]) -> Leg {
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
    let dir = scaffold_dir::ScaffoldDir::new("units_leg");
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

fn baseline(ws: &Workspace) -> Leg {
    let base = run_leg(ws, "baseline", Slow::DropOldest(0), &[]);
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
    let ws = read_identity::workspace();
    let base = baseline(&ws);
    let control = run_leg(&ws, "control", Slow::Block(SLOW_SLEEP_US), &[]);
    assert!(
        control.rate_hz <= base.rate_hz * 0.5,
        "RI_HARNESS: the block control did not slow the fast node ({:.1} Hz against a {:.1} Hz \
         baseline), so this measurement cannot see a hold and proves nothing about the slow leg",
        control.rate_hz,
        base.rate_hz
    );
    let slow = run_leg(&ws, "slow", Slow::DropOldest(SLOW_SLEEP_US), &[]);
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
    let ws = read_identity::workspace();
    let base = baseline(&ws);
    let mutant = run_leg(
        &ws,
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
fn run_chain_leg(ws: &Workspace, leg: &str, env: &[(&str, &Path)]) -> Leg {
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
    let dir = scaffold_dir::ScaffoldDir::new("units_leg");
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
    let ws = read_identity::workspace();
    let base = baseline(&ws);
    let chain = run_chain_leg(&ws, "chain_slow", &[]);
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
    let ws = read_identity::workspace();
    let base = baseline(&ws);
    let mutant = run_chain_leg(
        &ws,
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
    ws: &Workspace,
    fixture: Fixture,
    placement: Placement,
) -> (read_identity::RecordedRun, read_identity::Verdict) {
    let run = read_identity::record_live(ws, fixture, placement);
    let replayed = read_identity::resim(ws, &run, ReplayMode::ResimNeutral, None);
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
/// that writer's previous read plus one (its first read counts from seq 1). A
/// skip is an eviction, which a replay that feeds a real queue does not
/// reproduce whatever the placement.
fn sequence_gaps(run: &read_identity::RecordedRun, node: &str, input: &str) -> usize {
    let Some(reads) = run.live.streams.get(&(node.to_string(), input.to_string())) else {
        return 0;
    };
    let mut last: std::collections::BTreeMap<u64, u64> = std::collections::BTreeMap::new();
    let mut gaps = 0;
    for read in reads {
        let expected = last.get(&read.writer).map_or(1, |s| s + 1);
        if read.seq != expected {
            gaps += 1;
        }
        last.insert(read.writer, read.seq);
    }
    gaps
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
    replays_like_separate_processes("idle");
}

/// The comparison above with half the host's CPUs held busy for the whole
/// arm, so the units' threads and the recorder compete for CPU time.
#[test]
#[ignore = "box-only: real runs on a Linux host, loaded by the arm itself"]
#[serial]
fn a_run_on_executor_units_replays_like_the_same_graph_in_separate_processes_under_load() {
    let _load = CpuStressor::start(stressor_children());
    replays_like_separate_processes("loaded");
}

/// Busy-loop children for the under-load arm: `CER_UNITS_LOAD_CHILDREN`, else
/// half the CPUs this process may run on (at least one).
fn stressor_children() -> usize {
    std::env::var("CER_UNITS_LOAD_CHILDREN")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or_else(|| std::thread::available_parallelism().map_or(1, |n| (n.get() / 2).max(1)))
}

/// Child processes each spinning `while :; do :; done` in `sh`, unpinned,
/// killed and reaped when the guard drops.
struct CpuStressor {
    children: Vec<std::process::Child>,
}

impl CpuStressor {
    fn start(count: usize) -> Self {
        let children = (0..count)
            .map(|_| {
                std::process::Command::new("sh")
                    .args(["-c", "while :; do :; done"])
                    .stdin(std::process::Stdio::null())
                    .stdout(std::process::Stdio::null())
                    .stderr(std::process::Stdio::null())
                    .spawn()
                    .expect("RI_HARNESS: spawn a busy-loop child")
            })
            .collect();
        let cpus = std::thread::available_parallelism().map_or(0, std::num::NonZeroUsize::get);
        eprintln!(
            "UNDER_LOAD stressor=sh_busy_loop children={count} cpus_available={cpus} pinned=no"
        );
        Self { children }
    }
}

impl Drop for CpuStressor {
    fn drop(&mut self) {
        for child in &mut self.children {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

/// Record the fan-out and three-rank fixtures in one process group and with
/// one process per node, resim both, and require every gap-free stream to
/// judge the same under both placements. `leg` names the host condition in
/// the printed lines.
fn replays_like_separate_processes(leg: &str) {
    let ws = &read_identity::workspace();
    let mut compared = 0usize;
    let mut excess = 0usize;
    for fixture in [Fixture::FanOut, Fixture::Ranks3] {
        let (run, units) = record_and_judge(ws, fixture, Placement::OneGroup);
        let (declared_run, processes) = record_and_judge(ws, fixture, Placement::Declared);
        assert!(
            !units.rows.is_empty(),
            "RI_HARNESS: {} judged no stream",
            fixture.name()
        );
        for row in &units.rows {
            // Frames each placement evicted live on this stream. Hosting the
            // units in one process must not make a consumer late enough to
            // evict where one process per node does not.
            let gaps = sequence_gaps(&run, &row.node, &row.input);
            let declared_gaps = sequence_gaps(&declared_run, &row.node, &row.input);
            eprintln!(
                "UNITS_REPLAY leg={leg} fixture={} node={} input={} one_group_gaps={gaps} \
                 declared_gaps={declared_gaps} one_group_verdict={}",
                fixture.name(),
                row.node,
                row.input,
                kind(&row.verdict)
            );
            if gaps > declared_gaps + EVICTION_TOLERANCE {
                excess += 1;
                eprintln!(
                    "EVICTION_EXCESS stream={}:{}.{} one_group={gaps} declared={declared_gaps}: \
                     hosting the units together made this consumer late",
                    fixture.name(),
                    row.node,
                    row.input
                );
            }
            // An evicting stream is not compared: a replay that feeds a real
            // queue reproduces no eviction under any placement. When the
            // replay serves each input the frames it was served live, judge
            // these streams too, under `ReplayMode::ResimVerify`, and require
            // equality.
            if gaps > 0 || declared_gaps > 0 {
                continue;
            }
            // A stream that evicted nothing: every unit had its ports built
            // before any unit's first step, so its first live read is its
            // writer's first frame.
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
            if kind(&row.verdict) != kind(&other.verdict) {
                for (label, verdict_row, bag) in [
                    ("one_group", &row.verdict, &run.bag),
                    ("declared", &other.verdict, &declared_run.bag),
                ] {
                    if let RowVerdict::Diverged { live, .. } = verdict_row {
                        print_read_window(bag, label, live.seq);
                    }
                }
                if let RowVerdict::Diverged { live, .. } = &row.verdict {
                    print_read_window(&declared_run.bag, "declared_same_seq", live.seq);
                }
            }
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
    eprintln!(
        "UNITS_REPLAY_SUMMARY leg={leg} compared={compared} eviction_excess_streams={excess}"
    );
    assert!(
        compared >= 3,
        "RI_HARNESS: only {compared} gap-free stream(s) were compared; the arm proves nothing"
    );
}

/// Copy `input` to `output` with its embedded `graph.yaml` replaced by `yaml`:
/// every channel, frame and other attachment as recorded, through the bag
/// writer, the same way `bag migrate` writes its copy.
fn rewrite_embedded_graph(input: &Path, output: &Path, yaml: &str) {
    use cerulion_bag::{BagReader, BagWriter, BagWriterConfig};
    let reader = BagReader::open(input).expect("RI_HARNESS: open the recorded bag");
    let channels = reader.channels().expect("RI_HARNESS: channel table");
    let topics = cerulion_cli_engine::bag_migrate::user_topic_schemas(&channels)
        .expect("RI_HARNESS: topic schemas");
    let config = BagWriterConfig {
        provisioning: channels
            .iter()
            .filter(|c| !c.provisioning.is_empty())
            .map(|c| (c.topic.clone(), c.provisioning))
            .collect(),
        ..Default::default()
    };
    let mut writer =
        BagWriter::create(output, config, &topics).expect("RI_HARNESS: create the rewritten bag");
    for m in reader.messages().expect("RI_HARNESS: frames") {
        let m = m.expect("RI_HARNESS: a frame");
        writer
            .write_message(&m.topic, m.sequence, m.log_time, m.publish_time, &[&m.data])
            .expect("RI_HARNESS: write a frame");
    }
    let mut replaced = 0;
    for a in reader.attachments().expect("RI_HARNESS: attachments") {
        let data: &[u8] = if a.name == "graph.yaml" {
            replaced += 1;
            yaml.as_bytes()
        } else {
            &a.data
        };
        writer
            .write_attachment(&a.name, &a.media_type, a.log_time, a.create_time, data)
            .expect("RI_HARNESS: write an attachment");
    }
    assert_eq!(
        replaced, 1,
        "RI_HARNESS: the bag must embed exactly one graph.yaml"
    );
    writer
        .finalize()
        .expect("RI_HARNESS: finalize the rewritten bag");
}

/// The embedded graph of `bag`.
fn embedded_graph(bag: &Path) -> String {
    let reader = cerulion_bag::BagReader::open(bag).expect("RI_HARNESS: open the bag");
    let att = reader
        .attachments()
        .expect("RI_HARNESS: attachments")
        .into_iter()
        .find(|a| a.name == "graph.yaml")
        .expect("RI_HARNESS: the bag embeds graph.yaml");
    String::from_utf8(att.data).expect("RI_HARNESS: graph.yaml is UTF-8")
}

/// A bag whose embedded graph carries the retired `fuse: false` key replays:
/// `bag play --resim all` exits 0, warns exactly once naming the key, and
/// reproduces every read the unrewritten bag reproduces. Under the
/// `oracle-mutants` feature, `CERULION_RECORDED_FUSE_MUTANT` makes the replay
/// refuse the key, and this arm must fail.
#[test]
#[ignore = "box-only: real runs on a quiet Linux host"]
#[serial]
fn a_bag_recorded_with_the_retired_fuse_key_still_replays() {
    let ws = read_identity::workspace();
    let mut run = read_identity::record_live(&ws, Fixture::Ranks2, Placement::Declared);
    let reference = read_identity::resim(&ws, &run, ReplayMode::ResimNeutral, None);
    let read_identity::Replayed::Reads { log: reference, .. } = reference else {
        panic!("RI_HARNESS: the unrewritten bag's replay refused");
    };

    // `fuse: false` on the relay, the old spelling of what `isolate: true` says.
    let mut doc: serde_yaml::Value =
        serde_yaml::from_str(&embedded_graph(&run.bag)).expect("RI_HARNESS: graph.yaml parses");
    let relay = doc["nodes"]
        .as_sequence_mut()
        .expect("RI_HARNESS: nodes")
        .iter_mut()
        .find(|n| n["id"].as_str() == Some("relay"))
        .expect("RI_HARNESS: the fixture has a relay");
    relay
        .as_mapping_mut()
        .expect("RI_HARNESS: a node is a mapping")
        .insert("fuse".into(), false.into());
    let yaml = serde_yaml::to_string(&doc).expect("RI_HARNESS: render graph.yaml");
    let rewritten = run.dir.path().join("fuse_false.mcap");
    rewrite_embedded_graph(&run.bag, &rewritten, &yaml);
    assert!(embedded_graph(&rewritten).contains("fuse: false"));
    run.bag = rewritten;

    let replayed = read_identity::resim(&ws, &run, ReplayMode::ResimNeutral, None);
    let log = match replayed {
        read_identity::Replayed::Reads { exit, log } => {
            assert_eq!(
                exit, 0,
                "the replay of a bag carrying `fuse: false` must exit 0"
            );
            log
        }
        read_identity::Replayed::Refused { stderr } => {
            panic!("the replay refused a bag carrying the retired `fuse:` key:\n{stderr}")
        }
    };
    // The newest stderr file under the run directory is this replay's.
    let stderr_path = std::fs::read_dir(run.dir.path())
        .expect("RI_HARNESS: run dir")
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "stderr"))
        .max_by_key(|p| std::fs::metadata(p).and_then(|m| m.modified()).ok())
        .expect("RI_HARNESS: the replay's stderr file");
    let stderr = std::fs::read_to_string(&stderr_path).expect("RI_HARNESS: read stderr");
    let warnings = stderr
        .lines()
        .filter(|l| l.contains("retired `fuse:` key"))
        .count();
    assert_eq!(
        warnings, 1,
        "exactly one warning must name the retired key; got {warnings}:\n{stderr}"
    );
    assert_eq!(
        log.streams, reference.streams,
        "the bag carrying `fuse: false` must replay every read the unrewritten bag replays"
    );
    assert!(
        log.total() >= 50,
        "RI_HARNESS: the replay read only {} frame(s)",
        log.total()
    );
}

#[cfg(feature = "oracle-mutants")]
#[test]
#[ignore = "box-only: real runs on a quiet Linux host"]
#[serial]
fn the_recorded_fuse_refusal_mutant_is_caught() {
    // The `bag play` child inherits the variable; `#[serial]` keeps every
    // other arm of this binary from running while it is set.
    std::env::set_var(cerulion_core::graph::RECORDED_FUSE_MUTANT_ENV, "1");
    let caught = std::panic::catch_unwind(a_bag_recorded_with_the_retired_fuse_key_still_replays);
    std::env::remove_var(cerulion_core::graph::RECORDED_FUSE_MUTANT_ENV);
    let message = match caught {
        Ok(()) => panic!("the fuse-on-replay arm passed with the replay refusing the retired key"),
        Err(payload) => payload
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
            .unwrap_or_default(),
    };
    assert!(
        message.contains("refused a bag carrying the retired `fuse:` key"),
        "the arm failed, but not on the refusal the mutant makes: {message}"
    );
}

/// Print, from `bag`'s scheduler trace, every record of the ranks that served a
/// frame within 4 of `seq`, over the steps those reads span plus one either
/// side: `READ_WINDOW` lines carrying rank, step, record kind, node index, and
/// for a read its input index, outcome kind, served sequence and popped count.
/// Fires per step are the `fire` lines of one rank and step.
fn print_read_window(bag: &Path, label: &str, seq: u64) {
    let reader = match cerulion_bag::BagReader::open(bag) {
        Ok(r) => r,
        Err(e) => {
            eprintln!(
                "READ_WINDOW label={label} cannot open {}: {e}",
                bag.display()
            );
            return;
        }
    };
    let mut recs: Vec<(u32, u64, u32, u32, u32, u64, u64)> = Vec::new();
    let Ok(messages) = reader.messages() else {
        eprintln!("READ_WINDOW label={label} cannot walk {}", bag.display());
        return;
    };
    for m in messages.flatten() {
        if m.topic != cerulion_bag::SCHEDULER_TRACE_TOPIC {
            continue;
        }
        for r in m.data.chunks_exact(40) {
            let u64_at = |o: usize| u64::from_le_bytes(r[o..o + 8].try_into().expect("8 bytes"));
            let u32_at = |o: usize| u32::from_le_bytes(r[o..o + 4].try_into().expect("4 bytes"));
            // (rank, step, type, node_idx, level/meta, slot1, slot2)
            recs.push((
                u32_at(36) & 0xFFFF,
                u64_at(0),
                u32_at(32),
                u32_at(24),
                u32_at(28),
                u64_at(8),
                u64_at(16),
            ));
        }
    }
    let near = |served: u64| served != u64::MAX && served + 4 >= seq && served <= seq + 4;
    let hits: Vec<(u32, u64)> = recs
        .iter()
        .filter(|r| r.2 == 6 && near(r.5))
        .map(|r| (r.0, r.1))
        .collect();
    if hits.is_empty() {
        eprintln!("READ_WINDOW label={label} no read served a frame within 4 of seq {seq}");
        return;
    }
    for rank in hits
        .iter()
        .map(|h| h.0)
        .collect::<std::collections::BTreeSet<_>>()
    {
        let steps: Vec<u64> = hits.iter().filter(|h| h.0 == rank).map(|h| h.1).collect();
        let lo = steps.iter().min().copied().unwrap_or(0).saturating_sub(1);
        let hi = steps.iter().max().copied().unwrap_or(0) + 1;
        for r in recs
            .iter()
            .filter(|r| r.0 == rank && r.1 >= lo && r.1 <= hi)
        {
            match r.2 {
                1 => eprintln!(
                    "READ_WINDOW label={label} rank={} step={} kind=fire node={}",
                    r.0, r.1, r.3
                ),
                6 => {
                    eprintln!(
                    "READ_WINDOW label={label} rank={} step={} kind=read node={} in={} outcome={} \
                     seq={} popped={}",
                    r.0,
                    r.1,
                    r.3,
                    r.4 >> 16,
                    r.4 & 0x3FFF,
                    if r.5 == u64::MAX { "-".to_string() } else { r.5.to_string() },
                    r.6 & 0xFFFF_FFFF
                )
                }
                _ => {}
            }
        }
    }
}

/// The discriminating probe: the three-rank chain with one process per node and
/// a relay late enough that its fires bunch, re-executed neutral. If its relay
/// stream diverges like the one-group run's, the divergence is the replay's
/// (it feeds a real queue of fixed depth), not the executor units'. It prints;
/// its floors are the only assertions.
#[test]
#[ignore = "box-only: real runs on a quiet Linux host"]
#[serial]
fn a_late_relay_with_one_process_per_node_replay_probe() {
    let ws = read_identity::workspace();
    let fixture = Fixture::Ranks3LateRelay;
    let (run, verdict) = record_and_judge(&ws, fixture, Placement::Declared);
    assert!(
        verdict.compared_floor(Floors::default()).is_ok(),
        "RI_HARNESS: {} compared too few reads\n{}",
        fixture.name(),
        verdict.render()
    );
    for row in &verdict.rows {
        let gaps = sequence_gaps(&run, &row.node, &row.input);
        eprintln!(
            "LATE_RELAY_PROBE node={} input={} live_gaps={gaps} verdict={}",
            row.node,
            row.input,
            kind(&row.verdict)
        );
        if let RowVerdict::Diverged { live, .. } = &row.verdict {
            print_read_window(&run.bag, "late_relay_declared", live.seq);
        }
    }
}

/// Set, the first-fire arm samples the executor-unit threads of each run
/// (`UNIT_THREAD` lines). The sampler reads `/proc` every 100 us for the
/// unit threads, which only the one-group run has, so a bounded run leaves
/// it unset.
const SAMPLE_THREADS_ENV: &str = "CER_UNITS_SAMPLE_THREADS";

/// Runs of each placement the first-fire arm records.
const FIRST_FIRE_REPS: usize = 3;
/// The co-hosted first fire may take at most this many times its
/// own-process twin of the same rep.
const FIRST_FIRE_RATIO: u64 = 2;

/// One rank's first fire as the bag records it.
#[derive(Debug, Clone)]
struct FirstFire {
    rank: u32,
    node: String,
    /// The FIRE record's duration.
    dur_ns: u64,
    /// How many fires the rank's first fired step holds.
    first_step_fires: usize,
}

/// Every worker rank's first fire, rank-ascending, from `bag`'s scheduler
/// trace and its per-rank manifests.
fn first_fires(bag: &Path) -> Vec<FirstFire> {
    let reader = cerulion_bag::BagReader::open(bag).expect("RI_HARNESS: open the bag");
    let mut fires: std::collections::BTreeMap<u32, Vec<(u64, u64)>> = Default::default();
    for m in reader
        .messages()
        .expect("RI_HARNESS: walk the bag")
        .flatten()
    {
        if m.topic != cerulion_bag::SCHEDULER_TRACE_TOPIC {
            continue;
        }
        for r in m.data.chunks_exact(40) {
            let u64_at = |o: usize| u64::from_le_bytes(r[o..o + 8].try_into().expect("8 bytes"));
            let u32_at = |o: usize| u32::from_le_bytes(r[o..o + 4].try_into().expect("4 bytes"));
            if u32_at(32) == 1 {
                fires
                    .entry(u32_at(36) & 0xFFFF)
                    .or_default()
                    .push((u64_at(0), u64_at(16)));
            }
        }
    }
    fires
        .into_iter()
        .map(|(rank, list)| {
            let (step, dur_ns) = list[0];
            let name = format!("__cerulion/trace_manifest_rank{rank}.json");
            let manifest = reader
                .attachment(&name)
                .expect("RI_HARNESS: read attachments")
                .unwrap_or_else(|| panic!("RI_HARNESS: the bag carries no {name}"));
            let manifest: serde_json::Value = serde_json::from_slice(&manifest.data)
                .unwrap_or_else(|e| panic!("RI_HARNESS: {name} does not parse: {e}"));
            let node = manifest["node_ids"]
                .as_array()
                .map(|ids| {
                    ids.iter()
                        .filter_map(serde_json::Value::as_str)
                        .collect::<Vec<_>>()
                        .join(",")
                })
                .unwrap_or_default();
            FirstFire {
                rank,
                node,
                dur_ns,
                first_step_fires: list.iter().filter(|f| f.0 == step).count(),
            }
        })
        .collect()
}

/// The relay's timed ticks (`relay_*.times` under `live_dir`): per tick
/// `[index, entry ns, exit ns, entry snapshot (6), exit snapshot (6)]`, the
/// snapshot being `[cpu ns, runqueue-wait ns, timeslices, cpu, voluntary
/// switches, involuntary switches]`.
fn relay_ticks(live_dir: &Path) -> Vec<Vec<u64>> {
    let Ok(entries) = std::fs::read_dir(live_dir) else {
        return Vec::new();
    };
    let mut ticks = Vec::new();
    for e in entries.flatten() {
        let path = e.path();
        if path.extension().and_then(|x| x.to_str()) != Some("times") {
            continue;
        }
        for line in mp_support::read_file(&path).lines() {
            let f: Vec<u64> = line
                .split_whitespace()
                .skip(1)
                .filter_map(|w| w.parse().ok())
                .collect();
            if f.len() == 15 {
                ticks.push(f);
            }
        }
    }
    ticks.sort_by_key(|t| t[0]);
    ticks
}

/// Print each timed tick's body (`RELAY_TICK`, entry to exit) and each gap
/// between one tick's exit and the next tick's entry (`RELAY_GAP`), and, for
/// every gap, the unit-thread samples taken inside it (`UNIT_THREAD`).
fn print_relay_ticks(run: &read_identity::RecordedRun, rep: usize, placement: &str) {
    let ticks = relay_ticks(&run.live_dir);
    let Some(first) = ticks.first().map(|t| t[1]) else {
        eprintln!("RELAY_TICK rep={rep} placement={placement} no timed ticks were written");
        return;
    };
    let d = |a: u64, b: u64| b.saturating_sub(a) / 1000;
    let span = |label: &str, from: &[u64], to: &[u64], t0: u64, t1: u64, tick: u64| {
        eprintln!(
            "{label} rep={rep} placement={placement} tick={tick} at_us={} wall_us={} cpu_us={} \
             runqueue_wait_us={} slices={} voluntary={} involuntary={} cpu_from={} cpu_to={}",
            d(first, t0),
            d(t0, t1),
            d(from[0], to[0]),
            d(from[1], to[1]),
            to[2].saturating_sub(from[2]),
            to[4].saturating_sub(from[4]),
            to[5].saturating_sub(from[5]),
            from[3],
            to[3]
        );
    };
    let samples = mp_support::read_file(&run.dir.path().join(read_identity::UNIT_THREAD_SAMPLES));
    for (i, t) in ticks.iter().enumerate() {
        span("RELAY_TICK", &t[3..9], &t[9..15], t[1], t[2], t[0]);
        let Some(next) = ticks.get(i + 1) else {
            continue;
        };
        span("RELAY_GAP", &t[9..15], &next[3..9], t[2], next[1], t[0]);
        for line in samples.lines() {
            let Some(at) = line
                .split_whitespace()
                .next()
                .and_then(|w| w.parse::<u64>().ok())
            else {
                continue;
            };
            if at >= t[2] && at <= next[1] {
                eprintln!(
                    "UNIT_THREAD rep={rep} placement={placement} gap_after_tick={} at_us={} {}",
                    t[0],
                    d(first, at),
                    line.split_once(' ').map_or("", |(_, rest)| rest)
                );
            }
        }
    }
}

/// A node's first fire costs the same whether its unit shares a process or has
/// one of its own. The three-rank chain is recorded with one process per node
/// and with every node in one process, `FIRST_FIRE_REPS` times each, and each
/// rank's first fire is read off the bag. In every rep the co-hosted relay's
/// first fire must take at most `FIRST_FIRE_RATIO` times the relay's first
/// fire in its own process. Each run prints
///
/// ```text
/// FIRST_FIRE rep=<r> placement=<p> rank=<k> node=<id> dur_us=<u> first_step_fires=<n>
/// RELAY_TICK rep=<r> placement=<p> tick=<i> at_us=<u> wall_us=<u> cpu_us=<u> runqueue_wait_us=<u> slices=<n> voluntary=<n> involuntary=<n> cpu_from=<c> cpu_to=<c>
/// RELAY_GAP  (the same fields, from one tick's exit to the next tick's entry)
/// UNIT_THREAD rep=<r> placement=<p> gap_after_tick=<i> at_us=<u> <pid> <tid> <comm> <state> <wchan> <syscall>
/// ```
///
/// `RELAY_TICK` covers the relay's own tick body (its first four ticks) and
/// `RELAY_GAP` what runs between two of them: the publish of the tick's
/// output, the framework, and the next frame's refill. A gap with no CPU time
/// and no runqueue wait is a blocked thread; with `CER_UNITS_SAMPLE_THREADS`
/// set, the `UNIT_THREAD` samples inside it name where each unit thread was
/// waiting.
#[test]
#[ignore = "box-only: timing on a quiet Linux host"]
#[serial]
fn a_co_hosted_relays_first_fire_costs_what_it_costs_in_its_own_process() {
    let ws = read_identity::workspace();
    /// Turns the unit-thread sampler on for this arm only.
    struct Sampling;
    impl Drop for Sampling {
        fn drop(&mut self) {
            read_identity::SAMPLE_UNIT_THREADS.store(false, std::sync::atomic::Ordering::Relaxed);
        }
    }
    let sampling = std::env::var_os(SAMPLE_THREADS_ENV).is_some();
    read_identity::SAMPLE_UNIT_THREADS.store(sampling, std::sync::atomic::Ordering::Relaxed);
    let _sampling = Sampling;
    let mut failures = Vec::new();
    for rep in 0..FIRST_FIRE_REPS {
        let mut relay = std::collections::BTreeMap::new();
        for placement in [Placement::Declared, Placement::OneGroup] {
            let run = read_identity::record_live(&ws, Fixture::Ranks3, placement);
            let fires = first_fires(&run.bag);
            assert_eq!(
                fires.len(),
                3,
                "RI_HARNESS: the three-rank chain recorded fires on {} rank(s)",
                fires.len()
            );
            for f in &fires {
                eprintln!(
                    "FIRST_FIRE rep={rep} placement={} rank={} node={} dur_us={} \
                     first_step_fires={}",
                    placement.label(),
                    f.rank,
                    f.node,
                    f.dur_ns / 1000,
                    f.first_step_fires
                );
            }
            print_relay_ticks(&run, rep, placement.label());
            let r = fires
                .iter()
                .find(|f| f.node == "relay")
                .expect("RI_HARNESS: no rank hosts the relay alone");
            relay.insert(placement.label(), r.dur_ns);
        }
        let alone = relay[Placement::Declared.label()];
        let hosted = relay[Placement::OneGroup.label()];
        let bound = alone * FIRST_FIRE_RATIO;
        if hosted > bound {
            failures.push(format!(
                "rep {rep}: co-hosted relay first fire {} us > bound {} us (own process {} us)",
                hosted / 1000,
                bound / 1000,
                alone / 1000
            ));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The probe with the descriptor pre-size skipped
/// (`DESCRIPTOR_PRESIZE_MUTANT_ENV`): the co-hosted relay's first fire grows
/// the shared descriptor table, and the probe's bound must fail on it.
#[cfg(feature = "oracle-mutants")]
#[test]
#[ignore = "box-only: timing on a quiet Linux host"]
#[serial]
fn the_descriptor_presize_mutant_is_caught() {
    // The `graph run` children inherit the variable; `#[serial]` keeps every
    // other arm of this binary from running while it is set.
    std::env::set_var(
        cerulion_cli_engine::graph_cmd::DESCRIPTOR_PRESIZE_MUTANT_ENV,
        "1",
    );
    let caught = std::panic::catch_unwind(
        a_co_hosted_relays_first_fire_costs_what_it_costs_in_its_own_process,
    );
    std::env::remove_var(cerulion_cli_engine::graph_cmd::DESCRIPTOR_PRESIZE_MUTANT_ENV);
    let message = match caught {
        Ok(()) => panic!("the first-fire probe passed with the descriptor pre-size skipped"),
        Err(payload) => payload
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
            .unwrap_or_default(),
    };
    assert!(
        message.contains("co-hosted relay first fire"),
        "the probe failed, but not on its first-fire bound: {message}"
    );
}
