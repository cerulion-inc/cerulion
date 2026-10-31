// SPDX-License-Identifier: AGPL-3.0-only
//! The READ-IDENTITY ORACLE over real runs: each fixture recorded live with
//! `graph run --record`, re-executed with `bag play --resim all`, and the two
//! read logs judged per node and per input (see `read_identity/mod.rs` for the
//! record and the verdict).
//!
//! ## What each arm asserts
//!
//! * `an_evicting_consumer_resim_diverges_today`: the EXPECTED DIVERGENCE. A
//!   consumer that evicts live reads frames replay cannot reproduce while the
//!   replay feeds a real queue (it pops by count, and live popped after
//!   evictions), so at least one stream must judge non-equal, and the live run
//!   must show the evictions (sequence gaps on the slow consumer's reads).
//! * `*_verdict_probe`: the fan-out, multi-writer and rank-split fixtures, judged
//!   and printed with no assertion on the direction of the verdict. Their floors
//!   are asserted on both sides (`Verdict::compared_floor`), so a probe that
//!   compared nothing fails.
//! * `an_evicting_consumer_under_verify_probe`: the evicting fixture replayed
//!   with `--resim all --verify`, printed as judged or refused.
//! * `a_planted_misread_is_caught`: the oracle's end-to-end MUTANT. One
//!   recording is resimmed twice, the second time with the sink writing one
//!   read with another writer (`CER_RI_PLANT`); judged against each other, the
//!   oracle must report exactly that read on exactly that stream.
//!
//! Each arm prints one `READ_IDENTITY` line per stream:
//!
//! ```text
//! READ_IDENTITY fixture=<f> leg=<placement>/<mode> node=<n> input=<i> verdict=<equal|diverged|short|long> live=<n> other=<n> compared=<n> [first_diff=<i> live_read=(w,s,h) other_read=(w,s,h)]
//! ```
//!
//! A failure that is not a verdict (bring-up, a lossy recording after the
//! retries, an unfinished resim, a floor) panics `RI_HARNESS:`.
//!
//! ## Scope and how to run
//!
//! Every arm is `#[ignore]` (it needs a quiet Linux host and runs for
//! tens of seconds) and `#[serial]`. On Linux:
//!
//! ```bash
//! cargo test -p cerulion_cli --test read_identity_box_test -- --ignored --nocapture --test-threads=1
//! ```
//!
//! Each arm builds the harness workspace in a scaffold directory it holds
//! for its whole body, removed when the arm ends, passing or failing (`CERULION_KEEP_TEST_WS=1`
//! keeps a failed arm's). Every workspace of this binary builds into one
//! target directory, so the dependencies compile once per binary.

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

use read_identity::{
    judge_replayed, Fixture, Floors, Outcome, Placement, ReplayMode, RowVerdict, Verdict, Workspace,
};
use serial_test::serial;

/// Record `fixture` under `placement`, replay it under `mode`, judge and
/// print. Returns the live run and the outcome.
fn record_and_judge_mode(
    ws: &Workspace,
    fixture: Fixture,
    placement: Placement,
    mode: ReplayMode,
    plant: Option<(&str, usize)>,
) -> (read_identity::RecordedRun, Outcome) {
    let run = read_identity::record_live(ws, fixture, placement);
    let replayed = read_identity::resim(ws, &run, mode, plant);
    let outcome = judge_replayed(fixture, &run.live, &replayed, Floors::default())
        .unwrap_or_else(|e| panic!("RI_HARNESS: {}: {e}", fixture.name()));
    outcome.print(
        fixture.name(),
        &format!("{}/{}", placement.label(), mode.label()),
    );
    if let Some(verdict) = outcome.verdict() {
        verdict
            .compared_floor(Floors::default())
            .unwrap_or_else(|e| panic!("RI_HARNESS: {}: {e}", fixture.name()));
    }
    (run, outcome)
}

/// [`record_and_judge_mode`] under the neutral resim, which must run.
fn record_and_judge(
    ws: &Workspace,
    fixture: Fixture,
    placement: Placement,
    plant: Option<(&str, usize)>,
) -> (read_identity::RecordedRun, Verdict) {
    let (run, outcome) =
        record_and_judge_mode(ws, fixture, placement, ReplayMode::ResimNeutral, plant);
    match outcome {
        Outcome::Judged { verdict, .. } => (run, verdict),
        Outcome::Refused { stderr } => {
            panic!("RI_HARNESS: the neutral resim refused the recording:\n{stderr}")
        }
    }
}

/// Sequence gaps on one stream: the indices of reads whose sequence is not the
/// previous read's plus one (same writer), i.e. reads after frames the
/// consumer never read.
fn sequence_gaps(run: &read_identity::RecordedRun, node: &str, input: &str) -> Vec<usize> {
    let reads = &run.live.streams[&(node.to_string(), input.to_string())];
    reads
        .windows(2)
        .enumerate()
        .filter(|(_, w)| w[0].writer == w[1].writer && w[1].seq != w[0].seq + 1)
        .map(|(i, _)| i + 1)
        .collect()
}

/// Live evictions must be present (the fixture did what it names), and the
/// resim must not reproduce the reads.
#[test]
#[ignore = "needs a quiet Linux host: real multi-process runs"]
#[serial]
fn an_evicting_consumer_resim_diverges_today() {
    let ws = read_identity::workspace();
    let (run, verdict) = record_and_judge(&ws, Fixture::SlowEvict, Placement::Declared, None);
    let gaps = sequence_gaps(&run, "slow", "inp");
    eprintln!(
        "READ_IDENTITY_EVICTIONS fixture=slow_evict gaps={} first_gap_read={}",
        gaps.len(),
        gaps.first().map_or("-".to_string(), |i| i.to_string())
    );
    assert!(
        gaps.len() >= 10,
        "RI_HARNESS: the slow consumer skipped only {} frame run(s) live; the fixture did \
         not evict, so the arm would judge a case it does not name",
        gaps.len()
    );
    let slow = verdict
        .rows
        .iter()
        .find(|r| r.node == "slow")
        .expect("RI_HARNESS: the slow consumer's stream was judged");
    assert!(
        matches!(slow.verdict, RowVerdict::Diverged { .. }),
        "EXPECTED DIVERGENCE not seen: the neutral resim served the evicting consumer the \
         reads it made live (or only a missing tail differs).\n{}",
        verdict.render()
    );
}

/// The evicting fixture under the verdict mode (`--resim all --verify`):
/// printed, refused or judged, with no assertion on the direction.
#[test]
#[ignore = "needs a quiet Linux host: real multi-process runs"]
#[serial]
fn an_evicting_consumer_under_verify_probe() {
    let ws = read_identity::workspace();
    record_and_judge_mode(
        &ws,
        Fixture::SlowEvict,
        Placement::Declared,
        ReplayMode::ResimVerify,
        None,
    );
}

#[test]
#[ignore = "needs a quiet Linux host: real multi-process runs"]
#[serial]
fn fan_out_verdict_probe() {
    let ws = read_identity::workspace();
    record_and_judge(&ws, Fixture::FanOut, Placement::Declared, None);
}

/// The multi-writer fixture also proves its shape: three writers on the one
/// stream, and batches of five or more frames per fire.
#[test]
#[ignore = "needs a quiet Linux host: real multi-process runs"]
#[serial]
fn multi_writer_verdict_probe() {
    let ws = read_identity::workspace();
    let (run, _verdict) = record_and_judge(&ws, Fixture::MultiWriter, Placement::Declared, None);
    let reads = &run.live.streams[&("tfbuf".to_string(), "tf".to_string())];
    let mut writers: Vec<u64> = reads.iter().map(|r| r.writer).collect();
    writers.sort_unstable();
    writers.dedup();
    assert_eq!(
        writers,
        vec![1, 2, 3],
        "RI_HARNESS: the batch reader must read all three writers"
    );
    let batches = read_identity::batch_sizes(&run.live_dir, "tfbuf");
    let big = batches.iter().filter(|&&n| n >= 5).count();
    eprintln!(
        "READ_IDENTITY_BATCHES fixture=multi_writer fires={} ge5={big} max={}",
        batches.len(),
        batches.iter().max().copied().unwrap_or(0)
    );
    assert!(
        big >= 50,
        "RI_HARNESS: only {big} fire(s) drained five or more frames; the fixture did not \
         batch"
    );
}

#[test]
#[ignore = "needs a quiet Linux host: real multi-process runs"]
#[serial]
fn ranks2_verdict_probe() {
    let ws = read_identity::workspace();
    record_and_judge(&ws, Fixture::Ranks2, Placement::Declared, None);
}

#[test]
#[ignore = "needs a quiet Linux host: real multi-process runs"]
#[serial]
fn ranks3_verdict_probe() {
    let ws = read_identity::workspace();
    record_and_judge(&ws, Fixture::Ranks3, Placement::Declared, None);
}

#[test]
#[ignore = "needs a quiet Linux host: real runs"]
#[serial]
fn ranks2_single_process_verdict_probe() {
    let ws = read_identity::workspace();
    record_and_judge(&ws, Fixture::Ranks2, Placement::SingleProcess, None);
}

/// One recording resimmed twice, once with the sink's read 25 planted: judged
/// planted against unplanted, the sink's stream must diverge exactly there,
/// with the planted writer, and no other stream may differ.
#[test]
#[ignore = "needs a quiet Linux host: real multi-process runs"]
#[serial]
fn a_planted_misread_is_caught() {
    const AT: usize = 25;
    let fixture = Fixture::Ranks2;
    let ws = read_identity::workspace();
    let run = read_identity::record_live(&ws, fixture, Placement::Declared);
    let reads = |plant| match read_identity::resim(&ws, &run, ReplayMode::ResimNeutral, plant) {
        read_identity::Replayed::Reads { log, .. } => log,
        read_identity::Replayed::Refused { stderr } => {
            panic!("RI_HARNESS: the neutral resim refused the recording:\n{stderr}")
        }
    };
    let clean = reads(None);
    let planted = reads(Some(("sink", AT)));
    let verdict = read_identity::judge(fixture, &clean, &planted, Floors::default())
        .unwrap_or_else(|e| panic!("RI_HARNESS: {e}"));
    verdict.print(fixture.name(), "declared/resim_neutral_planted");
    let divergent = verdict.divergent();
    assert_eq!(
        divergent.len(),
        1,
        "only the sink's stream differs between the two resims\n{}",
        verdict.render()
    );
    match &divergent[0].verdict {
        RowVerdict::Diverged { index, live, other } if divergent[0].node == "sink" => {
            assert_eq!(
                *index,
                AT,
                "the plant is reported at its read\n{}",
                verdict.render()
            );
            assert_eq!(other.writer, live.writer + 1000, "the planted writer");
        }
        v => panic!(
            "the oracle missed the planted misread at sink read {AT}: {v:?}\n{}",
            verdict.render()
        ),
    }
}
