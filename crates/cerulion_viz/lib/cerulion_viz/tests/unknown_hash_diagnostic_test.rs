// SPDX-License-Identifier: AGPL-3.0-only
//! The unknown-`schema_hash` diagnostic at the PRODUCTION
//! render site (`cerulion_viz::sink::dispatch_frame` → `classify_and_route`).
//!
//! The pure classifier and the reporting layer are oracle-tested in
//! `cerulion_core::codegen::unknown_hash`; this file pins that the SINK really
//! drives them — the no-inert-shipping half. A site that warns ONCE
//! per distinct hash forever, with no counter, no re-announcement and no
//! recovery, leaves an operator who missed the line no way back to it.
//!
//! The stimulus is the REAL walker-swap shape, not a synthetic one: the same
//! `geometry_msgs/Twist` frame is dispatched first against a walker that holds
//! NO schemas (so the hash resolves to nothing — exactly what a desk that never
//! compiled a robot's type sees) and then against the built-in walker, which is
//! precisely what `VizMsg::SwapWalker` does when a robot's `.msg` closure is
//! seeded mid-run.
//!
//! Its own binary: `#[traced_test]` installs a global subscriber, and every
//! predicate here counts lines.

mod common;

use cerulion_core::codegen::FrameWalker;
use cerulion_viz::sink::{dispatch_frame, SinkState};
use common::{build_twist_at, builtin_walker};
use tracing_test::traced_test;

/// A walker holding NO schemas — every frame's hash resolves to nothing. This
/// is the desk that has never compiled the robot's type, and (after a
/// vendored-hash bump) the desk whose build disagrees with the robot's.
fn empty_walker() -> FrameWalker {
    FrameWalker::new(Vec::new()).0
}

fn memory() -> rerun::RecordingStream {
    rerun::RecordingStreamBuilder::new("test")
        .recording_id("unknown_hash")
        .memory()
        .expect("memory sink")
        .0
}

/// Whole whitespace-token `key=value` match — a bare `contains` would let
/// `suppressed=5` match `suppressed=50`, and any key be a prefix of another.
fn has_field(line: &str, key: &str, value: &str) -> bool {
    let want = format!("{key}={value}");
    line.split_whitespace().any(|t| t == want)
}

/// The level read as a whole token out of the line header. `tracing-test`
/// renders the SPAN NAME — this test function's own name — into every line, so
/// a bare substring match on "WARN" is not safe.
fn line_level(line: &str) -> Option<&str> {
    line.split_whitespace()
        .find(|t| matches!(*t, "TRACE" | "DEBUG" | "INFO" | "WARN" | "ERROR"))
}

fn lines_at<'a>(logs: &'a [&'a str], level: &str, needle: &str) -> Vec<&'a str> {
    logs.iter()
        .filter(|l| line_level(l) == Some(level) && l.contains(needle))
        .copied()
        .collect()
}

/// How many DEBUG-level lines an assertion may demand — `n` in debug, `0` in
/// release.
///
/// `tracing`'s `release_max_level_info` feature compiles `debug!` OUT entirely
/// when `debug_assertions` is off, and several workspace crates enable it, which
/// Cargo unifies across the whole build. A DEBUG-line count therefore reads 0 in
/// release however the latch behaves, so an assertion demanding a nonzero count
/// fails there deterministically — the class that turned main red from the
/// push-only `cargo test -p cerulion_core --lib --release` job.
///
/// The debug pin is NOT weakened — it still demands exactly `n`.
///
/// # What pins the contract in RELEASE
///
/// NOT this number. Expecting `0` here is nearly vacuous in release, and
/// measurably so: mutating the suppressed arm to `warn!` fails both debug arms
/// and, with only this guard, PASSED in release — its line carries the
/// suppressed MESSAGE, which no WARN-keyed count looks for.
///
/// So each caller ALSO asserts, unconditionally and at every loud level, that
/// the suppressed message never appears loudly ([`suppressed_never_loud`]).
/// That holds in both modes and is what actually catches that regression in release.
/// `SinkState::undecodable_frame_count` is the third, level-free leg.
fn suppressed_debug_lines(n: usize) -> usize {
    if cfg!(debug_assertions) {
        n
    } else {
        0
    }
}

/// UNCONDITIONAL, level-independent: a suppressed repeat must never be LOUD.
///
/// The half of the suppression contract that survives `release_max_level_info`.
/// See [`suppressed_debug_lines`] for why the DEBUG count cannot carry it alone.
fn suppressed_never_loud(logs: &[&str], needle: &str) -> Result<(), String> {
    for level in ["WARN", "INFO", "ERROR"] {
        let loud = lines_at(logs, level, needle);
        if !loud.is_empty() {
            return Err(format!(
                "a suppressed repeat was emitted at {level} ({} line(s)): {loud:?}",
                loud.len()
            ));
        }
    }
    Ok(())
}

const HEAD: &str = "dropping a frame this build cannot decode";
const DECADE: &str = "STILL dropping every frame";
const QUIET: &str = "undecodable frame suppressed";
const RECOVERY: &str = "frames on this topic decode again";

/// THE headline: six undecodable frames on one topic cost ONE loud line, not
/// six — and the count survives the suppression. Then the walker swap heals it
/// and the regime re-arms.
#[traced_test]
#[test]
fn an_undecodable_topic_is_loud_once_counted_always_and_heals_on_a_walker_swap() {
    let rec = memory();
    let mut state = SinkState::new();
    let blind = empty_walker();
    // Stamps ADVANCE at 10 Hz: once the topic decodes it classifies as a plot
    // topic, and the series pre-walk rate gate would drop an identically-stamped
    // frame BEFORE `walk_by_hash` ever runs — which would silently make the
    // re-arm arm below unreachable rather than failing it.
    let frame_at = |i: u64| build_twist_at([1.0, 2.0, 3.0], [0.1, 0.2, 0.3], i * 100_000_000);

    for i in 0..6 {
        dispatch_frame(&rec, &blind, "twist", &frame_at(i), &mut state);
    }
    // The counter is the part that survives log filtering (Principle #3).
    assert_eq!(state.undecodable_frame_count("/twist"), 6);
    // A topic that never failed is not fabricated into the map.
    assert_eq!(state.undecodable_frame_count("/cloud"), 0);

    logs_assert(|logs: &[&str]| {
        let loud = lines_at(logs, "WARN", HEAD);
        if loud.len() != 1 {
            return Err(format!(
                "expected 1 WARN head, got {}: {loud:?}",
                loud.len()
            ));
        }
        // The sink cannot name the type (the wire carries a hash and the render
        // worker receives only frames + a walker), so it must say so rather
        // than guess — and must point somewhere the user can act.
        let head = loud[0];
        for (k, v) in [
            ("topic", "/twist"),
            ("vantage", "render"),
            ("kind", "schema_unidentified"),
            ("total_failures", "1"),
        ] {
            if !has_field(head, k, v) {
                return Err(format!("head missing {k}={v}: {head}"));
            }
        }
        if !head.contains("schema_hash=0x") {
            return Err(format!("head does not carry the wire hash: {head}"));
        }
        if head.contains("schema=") {
            return Err(format!("head claimed a schema name it cannot know: {head}"));
        }
        suppressed_never_loud(logs, QUIET)?;
        let quiet = lines_at(logs, "DEBUG", QUIET);
        let want_quiet = suppressed_debug_lines(5);
        if quiet.len() != want_quiet {
            return Err(format!(
                "expected {want_quiet} DEBUG repeats, got {}",
                quiet.len()
            ));
        }
        Ok(())
    });

    // The walker swap — the REAL recovery path (`VizMsg::SwapWalker` after a
    // robot's `.msg` closure is seeded). The identical frame now decodes.
    dispatch_frame(&rec, &builtin_walker(), "twist", &frame_at(6), &mut state);
    // Recovery does NOT reset the unconditional total.
    assert_eq!(state.undecodable_frame_count("/twist"), 6);

    logs_assert(|logs: &[&str]| {
        let rec_lines = lines_at(logs, "INFO", RECOVERY);
        if rec_lines.len() != 1 {
            return Err(format!(
                "expected 1 INFO recovery, got {}: {rec_lines:?}",
                rec_lines.len()
            ));
        }
        // The recovery reports what the operator MISSED (the suppressed count),
        // not the total, and names the topic so it can be grepped alongside the
        // head that opened the regime.
        if !has_field(rec_lines[0], "suppressed_count", "5")
            || !has_field(rec_lines[0], "topic", "/twist")
            // ROLL-IN 2: the recovery text is byte-identical across vantages, so
            // without this key an operator cannot tell WHICH observer healed.
            || !has_field(rec_lines[0], "vantage", "render")
        {
            return Err(format!("recovery line wrong: {}", rec_lines[0]));
        }
        Ok(())
    });

    // A fresh regime is LOUD again.
    dispatch_frame(&rec, &blind, "twist", &frame_at(7), &mut state);
    assert_eq!(state.undecodable_frame_count("/twist"), 7);
    logs_assert(|logs: &[&str]| {
        let loud = lines_at(logs, "WARN", HEAD);
        if loud.len() != 2 {
            return Err(format!("expected a re-armed head, got {}", loud.len()));
        }
        Ok(())
    });
}

/// The DECADE re-announcement at the production site. This is the arm that
/// matters most at the sink: its counter is reachable only from inside the
/// render worker's `SinkState`, so for anyone watching the daemon's log the
/// re-announcement IS the answer to "how bad has this got?".
#[traced_test]
#[test]
fn an_open_regime_re_announces_at_the_decade_from_the_render_site() {
    let rec = memory();
    let mut state = SinkState::new();
    let frame = build_twist_at([1.0, 2.0, 3.0], [0.1, 0.2, 0.3], 10_000_000);
    let blind = empty_walker();

    for _ in 0..10 {
        dispatch_frame(&rec, &blind, "twist", &frame, &mut state);
    }
    assert_eq!(state.undecodable_frame_count("/twist"), 10);

    logs_assert(|logs: &[&str]| {
        let head = lines_at(logs, "WARN", HEAD);
        let again = lines_at(logs, "WARN", DECADE);
        let quiet = lines_at(logs, "DEBUG", QUIET);
        suppressed_never_loud(logs, QUIET)?;
        let want_quiet = suppressed_debug_lines(8);
        if (head.len(), again.len(), quiet.len()) != (1, 1, want_quiet) {
            return Err(format!(
                "expected (1 head, 1 decade, {want_quiet} suppressed), got ({}, {}, {})",
                head.len(),
                again.len(),
                quiet.len()
            ));
        }
        // The re-announcement exists for the operator who MISSED the head, so
        // it may not serve a thinner field set than the head did.
        for (k, v) in [
            ("topic", "/twist"),
            ("vantage", "render"),
            ("kind", "schema_unidentified"),
            ("total_failures", "10"),
            ("suppressed", "8"),
        ] {
            if !has_field(again[0], k, v) {
                return Err(format!("decade line missing {k}={v}: {}", again[0]));
            }
        }
        if !again[0].contains("schema_hash=0x") {
            return Err(format!("decade line lost the wire hash: {}", again[0]));
        }
        // A DEBUG-level line must never be mistaken for the re-announcement.
        if !lines_at(logs, "DEBUG", DECADE).is_empty() {
            return Err("the decade re-announcement emitted at DEBUG".to_string());
        }
        Ok(())
    });
}

/// Two topics failing at once keep INDEPENDENT regimes and counters — one open
/// regime must never swallow another topic's loud head, which is exactly what
/// a robot whose whole `visualization_msgs` package skewed would produce.
#[traced_test]
#[test]
fn two_undecodable_topics_keep_independent_regimes_and_counters() {
    let rec = memory();
    let mut state = SinkState::new();
    let frame = build_twist_at([1.0, 2.0, 3.0], [0.1, 0.2, 0.3], 10_000_000);
    let blind = empty_walker();

    dispatch_frame(&rec, &blind, "marker", &frame, &mut state);
    dispatch_frame(&rec, &blind, "plan", &frame, &mut state);
    dispatch_frame(&rec, &blind, "marker", &frame, &mut state);

    assert_eq!(state.undecodable_frame_count("/marker"), 2);
    assert_eq!(state.undecodable_frame_count("/plan"), 1);

    logs_assert(|logs: &[&str]| {
        let loud = lines_at(logs, "WARN", HEAD);
        if loud.len() != 2 {
            return Err(format!(
                "each topic owes its own loud head; got {}: {loud:?}",
                loud.len()
            ));
        }
        for want in ["/marker", "/plan"] {
            if !loud.iter().any(|l| has_field(l, "topic", want)) {
                return Err(format!("no loud head for topic={want}"));
            }
        }
        Ok(())
    });
}

/// ANTI-TAUTOLOGY: a desk whose frames all decode logs NOTHING about unknown hashes
/// and counts zero. Without this, every "exactly N" arm above would also pass a
/// reporter that fired on healthy frames — and the promise this fix makes to a
/// working robot is precisely silence.
#[traced_test]
#[test]
fn a_desk_whose_frames_all_decode_is_silent_and_counts_zero() {
    let rec = memory();
    let mut state = SinkState::new();
    let walker = builtin_walker();

    for i in 0..20u64 {
        let frame = build_twist_at([1.0, 2.0, 3.0], [0.1, 0.2, 0.3], i * 10_000_000);
        dispatch_frame(&rec, &walker, "twist", &frame, &mut state);
    }
    // The positive half, asserted FIRST: a broken capture would read zero on
    // the absence guard below and pass vacuously, so the count is the anchor.
    assert_eq!(state.undecodable_frame_count("/twist"), 0);

    logs_assert(|logs: &[&str]| {
        // A forbidden line is forbidden at every level, so this predicate is
        // deliberately level-free.
        let noisy: Vec<_> = logs
            .iter()
            .filter(|l| l.contains("dropping a frame"))
            .collect();
        if !noisy.is_empty() {
            return Err(format!("a healthy desk logged: {noisy:?}"));
        }
        Ok(())
    });
}
