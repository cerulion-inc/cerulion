// SPDX-License-Identifier: AGPL-3.0-only
//! The READ-IDENTITY ORACLE over real runs, aimed at the private per-input
//! replay feeds: each fixture recorded live with `graph run --record`, replayed
//! with `bag play --resim all --verify`, and the two read logs judged per node
//! and per input (see `read_identity/mod.rs` for the record and the verdict).
//!
//! ## What each arm asserts
//!
//! * `an_evicting_consumer_replays_read_for_read_under_verify`: a slow
//!   drop-oldest consumer evicts live, and the `--verify` replay feeds each of
//!   its reads the frames it was served, so every stream judges equal. The
//!   live run must show the evictions.
//! * `an_evicting_consumer_still_diverges_under_a_neutral_resim`: the CONTROL.
//!   A resim without `--verify` keeps each consumer's queue, so the same
//!   fixture still diverges there.
//! * `fan_out_replays_read_for_read_under_verify`, `ranks2_...`, `ranks3_...`:
//!   equal under `--verify`.
//! * `multi_writer_reads_replay_read_for_read_under_verify`: three writers on
//!   one topic, read by an accumulating input that stages one read record per
//!   frame, each with its own producer annotation, so every read names its
//!   frame's writer and the stream judges equal. (A read that pops several
//!   frames of several writers is refused by name,
//!   `read_identity_batch_unattributed`; the engine's
//!   `a_batch_on_a_multi_writer_edge_is_refused_by_name` pins that.)
//!
//! Each arm prints one `READ_IDENTITY` line per stream (format in
//! `read_identity/mod.rs`). A stream that is not equal, or a verify replay
//! that exits 6, also prints `RI_EVIDENCE` lines: the replay's violations and
//! trace divergence, the recorded read-log records around the first
//! difference, and the live and replayed fires per step over that window. A
//! failure that is not a verdict panics `RI_HARNESS:`. With
//! `CERULION_KEEP_TEST_WS=1` a failing arm keeps its run's directory (the bag,
//! the live and replayed read dirs, each replay's stderr, `--report` JSON and
//! re-recording) and prints its path.
//!
//! ## Scope and how to run
//!
//! Every arm is `#[ignore]` (needs Linux and a quiet host, tens of seconds per
//! arm) and `#[serial]`. On Linux:
//!
//! ```bash
//! cargo test -p cerulion_cli --test read_identity_feed_box_test -- --ignored --nocapture --test-threads=1
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

use read_identity::{judge_replayed, Fixture, Floors, Outcome, Placement, ReplayMode, Verdict};
use serial_test::serial;

/// Record `fixture`, replay it under `mode`, judge and print.
fn record_and_judge(
    fixture: Fixture,
    placement: Placement,
    mode: ReplayMode,
) -> (read_identity::RecordedRun, Outcome) {
    let ws = read_identity::workspace();
    let run = read_identity::record_live(&ws, fixture, placement);
    let replayed = read_identity::resim(&ws, &run, mode, None);
    let outcome = judge_replayed(fixture, &run.live, &replayed, Floors::default())
        .unwrap_or_else(|e| panic!("RI_HARNESS: {}: {e}", fixture.name()));
    outcome.print(
        fixture.name(),
        &format!("{}/{}", placement.label(), mode.label()),
    );
    if let Some(verdict) = outcome.verdict() {
        read_identity::print_divergence_evidence(&run, &replayed, verdict);
    }
    if let Some(verdict) = outcome.verdict() {
        verdict
            .compared_floor(Floors::default())
            .unwrap_or_else(|e| panic!("RI_HARNESS: {}: {e}", fixture.name()));
    }
    (run, outcome)
}

/// The verdict of a replay that must have run, not refused.
fn judged(fixture: Fixture, outcome: Outcome) -> Verdict {
    match outcome {
        Outcome::Judged { verdict, .. } => verdict,
        Outcome::Refused { stderr } => panic!(
            "the replay of {} refused the recording:\n{stderr}",
            fixture.name()
        ),
    }
}

/// Reads whose sequence is not the previous one plus one (same writer): the
/// frames the live consumer never read.
fn sequence_gaps(run: &read_identity::RecordedRun, node: &str, input: &str) -> usize {
    let reads = &run.live.streams[&(node.to_string(), input.to_string())];
    reads
        .windows(2)
        .filter(|w| w[0].writer == w[1].writer && w[1].seq != w[0].seq + 1)
        .count()
}

fn assert_equal(fixture: Fixture, verdict: &Verdict) {
    assert!(
        verdict.is_equal(),
        "a --verify replay of {} must reproduce every read:\n{}",
        fixture.name(),
        verdict.render()
    );
}

#[test]
#[ignore = "needs Linux: real multi-process runs on a quiet host"]
#[serial]
fn an_evicting_consumer_replays_read_for_read_under_verify() {
    let (run, outcome) = record_and_judge(
        Fixture::SlowEvict,
        Placement::Declared,
        ReplayMode::ResimVerify,
    );
    let gaps = sequence_gaps(&run, "slow", "inp");
    eprintln!("READ_IDENTITY_EVICTIONS fixture=slow_evict gaps={gaps}");
    assert!(
        gaps >= 10,
        "RI_HARNESS: the slow consumer skipped only {gaps} frame run(s) live; the fixture \
         did not evict"
    );
    assert_equal(Fixture::SlowEvict, &judged(Fixture::SlowEvict, outcome));
}

#[test]
#[ignore = "needs Linux: real multi-process runs on a quiet host"]
#[serial]
fn an_evicting_consumer_still_diverges_under_a_neutral_resim() {
    let (_run, outcome) = record_and_judge(
        Fixture::SlowEvict,
        Placement::Declared,
        ReplayMode::ResimNeutral,
    );
    let verdict = judged(Fixture::SlowEvict, outcome);
    assert!(
        !verdict.is_equal(),
        "CONTROL: a neutral resim keeps each consumer's queue, so an evicting consumer's \
         reads diverge there\n{}",
        verdict.render()
    );
}

#[test]
#[ignore = "needs Linux: real multi-process runs on a quiet host"]
#[serial]
fn fan_out_replays_read_for_read_under_verify() {
    let (_run, outcome) = record_and_judge(
        Fixture::FanOut,
        Placement::Declared,
        ReplayMode::ResimVerify,
    );
    assert_equal(Fixture::FanOut, &judged(Fixture::FanOut, outcome));
}

#[test]
#[ignore = "needs Linux: real multi-process runs on a quiet host"]
#[serial]
fn ranks2_replays_read_for_read_under_verify() {
    let (_run, outcome) = record_and_judge(
        Fixture::Ranks2,
        Placement::Declared,
        ReplayMode::ResimVerify,
    );
    assert_equal(Fixture::Ranks2, &judged(Fixture::Ranks2, outcome));
}

#[test]
#[ignore = "needs Linux: real multi-process runs on a quiet host"]
#[serial]
fn ranks3_replays_read_for_read_under_verify() {
    let (_run, outcome) = record_and_judge(
        Fixture::Ranks3,
        Placement::Declared,
        ReplayMode::ResimVerify,
    );
    assert_equal(Fixture::Ranks3, &judged(Fixture::Ranks3, outcome));
}

#[test]
#[ignore = "needs Linux: real multi-process runs on a quiet host"]
#[serial]
fn multi_writer_reads_replay_read_for_read_under_verify() {
    let (_run, outcome) = record_and_judge(
        Fixture::MultiWriter,
        Placement::Declared,
        ReplayMode::ResimVerify,
    );
    assert_equal(Fixture::MultiWriter, &judged(Fixture::MultiWriter, outcome));
}
