// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion bag info` READS the bag's coverage manifest.
//!
//! The manifest (`__cerulion/record_coverage.json`) is written into every
//! finalized bag, and `bag info` is its one reader in the shipped
//! product — without it the "a recorder must tap what exists or SAY what it is
//! not tapping" argument is unreachable from any user command. This file
//! drives the production `bag_cmd::bag_info` over REAL bags written by
//! `cerulion_bag::BagWriter`.
//!
//! The rendering itself is oracle-tested purely in `bag_cmd`'s own unit tests
//! (`render_coverage` / `render_coverage_section`). What only a real bag can
//! pin is the READ — which of the four [`CoverageReading`] arms a given bag
//! lands on. That classification is the half a pure test is structurally blind
//! to, and getting it wrong is exactly how "no manifest" would start reading as
//! "nothing was missed".
//!
//! No transport: `bag_info` is file IO plus a schema walker. Per-test tempdirs
//! ⇒ parallel-safe, no `#[serial]`.

#![cfg(unix)]

use std::path::Path;

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};
use cerulion_cli_engine::bag_cmd;
use cerulion_core::wire::WireHeader;

const TOPIC: &str = "/imu";
const SCHEMA: &str = "geometry_msgs/Vector3";
const HASH: u64 = 0x0942_0942_0942_0942;

/// One 24-byte `Vector3` payload behind a real wire header.
fn frame(seq: u32) -> Vec<u8> {
    let payload = [0u8; 24];
    let mut out = vec![0u8; WireHeader::SIZE + payload.len()];
    let header = WireHeader {
        schema_hash: HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: 1_000_000_000 + u64::from(seq) * 10_000_000,
    };
    header.write_to_buf(&mut out[..WireHeader::SIZE]);
    out[WireHeader::SIZE..].copy_from_slice(&payload);
    out
}

/// Write a finalized bag on [`TOPIC`], carrying `coverage` as the record-coverage
/// attachment when `Some`.
fn write_bag(path: &Path, coverage: Option<&cerulion_bagd::RecordCoverage>) {
    let mut w = BagWriter::create(
        path,
        BagWriterConfig::default(),
        &[TopicSchema {
            topic: TOPIC.to_string(),
            schema_name: SCHEMA.to_string(),
            schema_hash: HASH,
            wire_fixed_size: 24,
        }],
    )
    .expect("create bag");
    if let Some(c) = coverage {
        let bytes = serde_json::to_vec(c).expect("encode coverage");
        w.write_attachment(
            cerulion_bagd::RECORD_COVERAGE_ATTACHMENT,
            "application/json",
            0,
            0,
            &bytes,
        )
        .expect("write coverage attachment");
    }
    // Frames outlive the chunk closure (the writer borrows them for the whole
    // scope), so they are built up front.
    let frames: Vec<Vec<u8>> = (0..4u32).map(frame).collect();
    w.write_chunk(|c| {
        for (seq, f) in frames.iter().enumerate() {
            let seq = seq as u32;
            let ts = 1_000_000_000 + u64::from(seq) * 10_000_000;
            c.write_message(TOPIC, seq, ts, ts, &[&f[..]])?;
        }
        Ok(())
    })
    .expect("write frames");
    w.finalize().expect("finalize");
}

/// A manifest describing the mixed-coverage shape: a declared tap, a discovered one
/// that attached mid-run, and one live producer the bag does NOT contain.
fn coverage() -> cerulion_bagd::RecordCoverage {
    let mut c = cerulion_bagd::RecordCoverage {
        version: cerulion_bagd::RECORD_COVERAGE_VERSION,
        enumerated: true,
        discovery_requested: true,
        ..Default::default()
    };
    c.tapped.insert(
        TOPIC.to_string(),
        cerulion_bagd::TappedTopic {
            source: cerulion_bagd::TapSource::Declared,
            frames_recorded: 4,
            attached_late: false,
            prefix_lost: None,
            schema_source: None,
        },
    );
    c.tapped.insert(
        "/lowstate".to_string(),
        cerulion_bagd::TappedTopic {
            source: cerulion_bagd::TapSource::Discovered,
            frames_recorded: 91,
            attached_late: true,
            prefix_lost: None,
            schema_source: None,
        },
    );
    c.untapped.insert(
        "/h264".to_string(),
        cerulion_bagd::UntappedReason::AppearedAfterBagCreation,
    );
    c
}

#[test]
fn bag_info_reports_a_bags_discovered_taps_and_the_producers_it_does_not_contain() {
    let dir = tempfile::tempdir().expect("tempdir");
    let path = dir.path().join("covered.mcap");
    write_bag(&path, Some(&coverage()));

    let text = bag_info(&path);
    // The pre-existing channel table is untouched — the coverage block is
    // ADDITIVE, so a reader's existing output must still be there.
    assert!(text.contains(TOPIC), "{text}");
    // What the bag contains, and how each tap got there.
    assert!(
        text.contains("2 topic(s) tapped (1 declared, 1 discovered)"),
        "{text}"
    );
    assert!(text.contains("/lowstate"), "{text}");
    assert!(
        text.contains("no back-fill"),
        "a mid-run tap must say its topic is not covered from the start: {text}"
    );
    // And what it does NOT contain — the half `record_health.json` cannot
    // answer, since a producer outside the tapped set loses no frames.
    assert!(
        text.contains("coverage: INCOMPLETE — 1 live producer(s)"),
        "{text}"
    );
    assert!(text.contains("/h264"), "{text}");
    assert!(text.contains("appeared_after_bag_creation"), "{text}");
    assert!(text.contains("--discovery-settle-ms"), "{text}");
}

/// `bag info` says whether the manifest it is
/// rendering describes a rolling WINDOW or a continuous recording.
///
/// The flag qualifies every count in the block, so leaving it unrendered is not
/// a cosmetic gap: the FRAMES column would read as a recorder's lifetime
/// account, and a capture's absent head-loss markers — which a window can never
/// claim, because it BEGINS mid-stream on every topic by construction — would
/// read as a proof that no head loss occurred.
///
/// The RECORDING half is in the same body on purpose. A renderer that printed
/// the capture sentence unconditionally satisfies the first assertion and is
/// caught by the second, and an absence assertion with no positive twin beside
/// it proves nothing about a renderer that prints neither.
#[test]
fn bag_info_says_a_capture_is_a_window_and_says_a_recording_is_not() {
    const CAPTURE_MARKER: &str = "this is a FLASHBACK CAPTURE, not a continuous recording";
    let dir = tempfile::tempdir().expect("tempdir");

    // THE PIN: a capture-scoped manifest.
    let mut capture_coverage = coverage();
    capture_coverage.window_capture = true;
    let capture = dir.path().join("capture.mcap");
    write_bag(&capture, Some(&capture_coverage));
    let capture_text = bag_info(&capture);
    assert!(
        capture_text.contains(CAPTURE_MARKER),
        "a capture's coverage block must say what kind of artifact it describes: {capture_text}"
    );
    assert!(
        capture_text.contains("makes no claim about the START of any stream"),
        "…and WHY its head-loss column is empty — otherwise absence reads as proof: \
         {capture_text}"
    );
    assert!(
        capture_text.contains(cerulion_bagd::FLASHBACK_ATTACHMENT),
        "…and where the rest of the capture's story lives: {capture_text}"
    );
    // The rest of the block is UNCHANGED — the flag qualifies the rows, it does
    // not replace them.
    assert!(
        capture_text.contains("2 topic(s) tapped (1 declared, 1 discovered)"),
        "{capture_text}"
    );

    // THE CONTROL: the same manifest, not a capture.
    let recording = dir.path().join("recording.mcap");
    write_bag(&recording, Some(&coverage()));
    let recording_text = bag_info(&recording);
    assert!(
        !recording_text.contains(CAPTURE_MARKER),
        "a continuous recording must NOT be described as a window capture: {recording_text}"
    );
    assert!(
        recording_text.contains("2 topic(s) tapped (1 declared, 1 discovered)"),
        "ANTI-VACUITY: the control really did render a coverage block: {recording_text}"
    );
}

/// **A capture never prints `coverage: COMPLETE`.**
///
/// The window caveat is a paragraph; the VERDICT is the line an operator stops
/// reading at, and this block's own comment says so. "Every live producer the
/// recorder enumerated is in this bag" is a sentence about a RECORDING — a
/// capture holds a rolling WINDOW of each of those producers. Printed three
/// lines under the caveat it contradicts it, and it would be an UPGRADE on the
/// earlier state, where a capture carried no manifest and was told
/// explicitly that this is "an ABSENCE of information, NOT a clean-coverage
/// claim".
///
/// So a capture routes to the existing `NO GAPS` arm, whose whole shape
/// is "held back from COMPLETE by the line(s) that qualify it".
///
/// The RECORDING half is the anti-tautology control, in the same body over the
/// same fixture: without it, a renderer that printed NO GAPS for everything
/// satisfies the capture assertions and nothing catches it.
#[test]
fn a_capture_is_held_back_from_complete_while_a_recording_earns_it() {
    let dir = tempfile::tempdir().expect("tempdir");

    // A CLEAN producer picture — the only state in which COMPLETE is reachable
    // at all. The shared fixture carries an untapped producer, which would take
    // the INCOMPLETE arm and make both assertions below vacuous.
    let mut clean = coverage();
    clean.untapped.clear();
    assert_eq!(clean.gap_count(), 0, "the fixture must have no gaps");

    // THE CONTROL: a continuous recording with a clean picture earns COMPLETE.
    let recording = dir.path().join("recording.mcap");
    write_bag(&recording, Some(&clean));
    let recording_text = bag_info(&recording);
    assert!(
        recording_text.contains("coverage: COMPLETE"),
        "ANTI-TAUTOLOGY: a clean RECORDING must still earn COMPLETE, or the capture assertion \
         below is satisfied by a renderer that never prints it: {recording_text}"
    );
    // …and it keeps its per-row back-fill marker, which the capture drops.
    assert!(
        recording_text.contains("no back-fill"),
        "a recording's mid-run tap still says its topic is not covered from the start: \
         {recording_text}"
    );

    // THE PIN: the same picture, on a capture.
    let mut capture_coverage = clean.clone();
    capture_coverage.window_capture = true;
    let capture = dir.path().join("capture.mcap");
    write_bag(&capture, Some(&capture_coverage));
    let capture_text = bag_info(&capture);
    assert!(
        !capture_text.contains("coverage: COMPLETE"),
        "a capture holds a WINDOW of every producer, so it may not claim they are `in this bag` \
         — and a bag that carried no manifest at all was told that is not a clean-coverage \
         claim, so acquiring one must not upgrade it to one: {capture_text}"
    );
    assert!(
        capture_text.contains("coverage: NO GAPS"),
        "…and the verdict is WITHHELD, not ABSENT: an absent `coverage:` line already means \
         something else in this block (a caller who opted out of discovery), and one blank \
         meaning two things is what the contradicting-verdict rule forbids: {capture_text}"
    );
    assert!(
        capture_text.contains("held back from COMPLETE by the WINDOW"),
        "…and it names WHY, pointing at the caveat above rather than at lines below that a \
         capture does not have: {capture_text}"
    );
    // The per-row marker is SUPPRESSED. `attached_late` means "after this
    // RECORDER armed", and the always-on window recorder outlives the graphs it
    // watches, so on a capture essentially every row would carry it while saying
    // nothing about whether the WINDOW begins at that topic's first frame — it
    // never does, on any row. A marker that is universally true teaches an
    // operator to skim rows.
    assert!(
        !capture_text.contains("no back-fill"),
        "a capture states the no-back-fill fact ONCE, in its window caveat, not on every row: \
         {capture_text}"
    );
    // …and the row itself is still there with its count, so suppression removed
    // a MARKER and not a topic.
    assert!(capture_text.contains("/lowstate"), "{capture_text}");
    assert!(capture_text.contains("91"), "{capture_text}");
}

#[test]
fn a_bag_with_no_coverage_manifest_says_so_and_does_not_error() {
    // Every bag written before the coverage manifest existed is in this state
    // by construction. It must render cleanly, must NOT fail, and must NOT read
    // as a clean bill of health.
    let dir = tempfile::tempdir().expect("tempdir");
    let path = dir.path().join("pre942.mcap");
    write_bag(&path, None);

    let text = bag_info(&path);
    assert!(
        text.contains(TOPIC),
        "the rest of `bag info` still works: {text}"
    );
    assert!(
        text.contains("carries no `__cerulion/record_coverage.json`"),
        "{text}"
    );
    assert!(text.contains("NOT a clean-coverage claim"), "{text}");
    assert!(!text.contains("coverage: COMPLETE"), "{text}");
    assert!(!text.contains("coverage: INCOMPLETE"), "{text}");
}

#[test]
fn a_malformed_coverage_manifest_is_reported_and_the_bag_still_reads() {
    // WARN-NEVER-REFUSE: coverage is additive reporting, so a manifest that
    // will not decode must not turn a perfectly readable bag into an error.
    let dir = tempfile::tempdir().expect("tempdir");
    let path = dir.path().join("malformed.mcap");
    let mut w = BagWriter::create(
        &path,
        BagWriterConfig::default(),
        &[TopicSchema {
            topic: TOPIC.to_string(),
            schema_name: SCHEMA.to_string(),
            schema_hash: HASH,
            wire_fixed_size: 24,
        }],
    )
    .expect("create bag");
    w.write_attachment(
        cerulion_bagd::RECORD_COVERAGE_ATTACHMENT,
        "application/json",
        0,
        0,
        b"{ this is not json",
    )
    .expect("write attachment");
    let f = frame(0);
    w.write_chunk(|c| c.write_message(TOPIC, 0, 1_000_000_000, 1_000_000_000, &[&f[..]]))
        .expect("write frames");
    w.finalize().expect("finalize");

    let text = bag_info(&path);
    assert!(text.contains(TOPIC), "{text}");
    assert!(text.contains("MALFORMED"), "{text}");
    assert!(!text.contains("coverage: COMPLETE"), "{text}");
}

/// The descriptor verdict is its OWN sentence, and the coverage
/// sentence never blames a cause that did not happen.
///
/// `is_incomplete()` gained a descriptor term, and this renderer had no arm for
/// it — so a run whose enumeration RAN and found everything fell through to
/// "the recorder could not establish what was live", two lines under this
/// block's own text saying enumeration succeeded. That is the
/// contradicting-verdict class on a third path.
///
/// Both halves are asserted in ONE body because they are one claim: the schema
/// verdict must APPEAR, the false coverage sentence must NOT, and the
/// unqualified COMPLETE must be withheld — an operator stops reading at
/// COMPLETE.
#[test]
fn a_bag_whose_schemas_none_resolved_says_so_without_blaming_coverage() {
    let dir = tempfile::tempdir().expect("tempdir");
    let path = dir.path().join("unresolved.mcap");
    let mut c = coverage();
    c.untapped.clear(); // coverage itself is CLEAN — the point of the arm.
    c.schema_demand_requested = true;
    c.replay_grade = Some(cerulion_bagd::ReplayGrade::Observability);
    assert!(
        c.is_incomplete(),
        "precondition: this is the state the run-binding term escalates"
    );
    write_bag(&path, Some(&c));

    let text = bag_info(&path);
    assert!(
        text.contains("schemas: UNRESOLVED"),
        "the descriptor verdict must be stated in its own words: {text}"
    );
    assert!(
        !text.contains("could not establish what was live"),
        "enumeration RAN on this bag — the coverage sentence must not claim otherwise: {text}"
    );
    assert!(
        !text.contains("coverage: COMPLETE"),
        "an unqualified COMPLETE is the line an operator stops reading at: {text}"
    );

    // ANTI-TAUTOLOGY, same fixture with the one field changed: a bag whose
    // schemas DID resolve renders the ordinary clean verdict and no schema line.
    let ok_path = dir.path().join("resolved.mcap");
    let mut ok = c.clone();
    ok.replay_grade = Some(cerulion_bagd::ReplayGrade::Full);
    assert!(!ok.is_incomplete());
    write_bag(&ok_path, Some(&ok));
    let ok_text = bag_info(&ok_path);
    assert!(
        !ok_text.contains("schemas: UNRESOLVED"),
        "a bag whose schemas resolved must not carry the verdict: {ok_text}"
    );
    assert!(
        ok_text.contains("coverage: COMPLETE"),
        "…and must still get its clean coverage verdict: {ok_text}"
    );
}

/// A coverage cause and unresolved schemas COEXIST — and the bag reports BOTH.
///
/// A coverage predicate written as `is_incomplete() &&
/// !schemas_unresolved()` MASKS instead of subtracting: a run carrying
/// both causes renders NO coverage line at all, while the recorder's own
/// terminal still fires one. That is precisely the "bag info and the recording
/// that produced the bag cannot disagree" rule the surrounding block exists to
/// keep, broken for one cause while being kept for another.
///
/// Driven over the reachable combinations rather than one sample: each coverage
/// cause is asserted to survive the presence of the schema verdict.
#[test]
fn a_coverage_cause_and_unresolved_schemas_are_both_reported() {
    let dir = tempfile::tempdir().expect("tempdir");

    // One per coverage term the generic arm covers.
    type Cause = fn(&mut cerulion_bagd::RecordCoverage);
    let causes: [(&str, Cause); 2] = [
        ("enumeration_failures", |c| c.enumeration_failures = 3),
        ("discovery_requested_but_not_enumerated", |c| {
            c.discovery_requested = true;
            c.enumerated = false;
        }),
    ];

    for (label, apply) in causes {
        let mut c = coverage();
        c.untapped.clear();
        apply(&mut c);
        c.schema_demand_requested = true;
        c.replay_grade = Some(cerulion_bagd::ReplayGrade::Observability);

        let path = dir.path().join(format!("both_{label}.mcap"));
        write_bag(&path, Some(&c));
        let text = bag_info(&path);

        assert!(
            text.contains("schemas: UNRESOLVED"),
            "[{label}] the descriptor verdict must still be stated: {text}"
        );
        assert!(
            text.contains("coverage: INCOMPLETE"),
            "[{label}] the coverage cause must NOT be masked by the schema verdict — the \
             recorder's own terminal reports it, and the two surfaces cannot disagree: {text}"
        );
        assert!(
            !text.contains("coverage: COMPLETE"),
            "[{label}] and COMPLETE is certainly wrong here: {text}"
        );
    }

    // ANTI-TAUTOLOGY: the same coverage cause WITHOUT the schema verdict still
    // renders exactly one coverage line and no schema line — so the assertions
    // above are measuring coexistence, not a renderer that says everything.
    let mut only_coverage = coverage();
    only_coverage.untapped.clear();
    only_coverage.enumeration_failures = 3;
    let path = dir.path().join("only_coverage.mcap");
    write_bag(&path, Some(&only_coverage));
    let text = bag_info(&path);
    assert!(text.contains("coverage: INCOMPLETE"), "{text}");
    assert!(!text.contains("schemas: UNRESOLVED"), "{text}");
}

// ---------------------------------------------------------------------------
// `bag info` READS the producer attribution
// ---------------------------------------------------------------------------

/// Write a finalized bag on [`TOPIC`] carrying `health` as the record-health
/// attachment.
///
/// The document is built as JSON rather than as a `RecordHealth` value on
/// purpose: every producer-attribution field is additive and `#[serde(default)]`, so a
/// hand-written minimal document is exactly what a real recorder emits for these
/// topics — and it exercises the DECODE the reader actually performs.
fn write_bag_with_health(path: &Path, health: &serde_json::Value) {
    let mut w = BagWriter::create(
        path,
        BagWriterConfig::default(),
        &[TopicSchema {
            topic: TOPIC.to_string(),
            schema_name: SCHEMA.to_string(),
            schema_hash: HASH,
            wire_fixed_size: 24,
        }],
    )
    .expect("create bag");
    w.write_attachment(
        cerulion_bagd::RECORD_HEALTH_ATTACHMENT,
        "application/json",
        0,
        0,
        &serde_json::to_vec(health).expect("encode health"),
    )
    .expect("write health attachment");
    let f = frame(0);
    w.write_chunk(|c| c.write_message(TOPIC, 0, 1_000_000_000, 1_000_000_000, &[&f[..]]))
        .expect("write frames");
    w.finalize().expect("finalize");
}

/// Write a bag carrying `bytes` under `attachment` verbatim — including bytes
/// that are not JSON at all, which is how the MALFORMED arm is driven.
fn write_bag_with_attachment(path: &Path, attachment: &str, bytes: &[u8]) {
    let mut w = BagWriter::create(
        path,
        BagWriterConfig::default(),
        &[TopicSchema {
            topic: TOPIC.to_string(),
            schema_name: SCHEMA.to_string(),
            schema_hash: HASH,
            wire_fixed_size: 24,
        }],
    )
    .expect("create bag");
    w.write_attachment(attachment, "application/json", 0, 0, bytes)
        .expect("write attachment");
    let f = frame(0);
    w.write_chunk(|c| c.write_message(TOPIC, 0, 1_000_000_000, 1_000_000_000, &[&f[..]]))
        .expect("write frames");
    w.finalize().expect("finalize");
}

/// A `record_health.json` body carrying ONE absorbance
/// verdict, hand-built as JSON so the fixture does not depend on the producer —
/// which is also what lets [`health_doc_with_contradictory_absorbance`] below
/// write a combination the producer cannot.
fn health_doc_with_absorbance(
    basis: &str,
    verdict: &str,
    short_evaluations: u64,
) -> serde_json::Value {
    serde_json::json!({
        "version": 2,
        "dropped_unwritten": 0,
        "loss_counting_basis": basis,
        "topics": {
            TOPIC: {
                "frames_recorded": 10,
                "frames_lost": 0,
                "gap_events": 0,
                "absorbance": {
                    "tap_buffer_depth": 2,
                    "rate_mhz": 400_000,
                    "absorbance_us": 5_000,
                    "measured_tail_us": 25_000,
                    "required_depth": 10,
                    "shortfall_at_least_us": 20_000,
                    "verdict": verdict,
                    "short_evaluations": short_evaluations,
                },
            }
        },
    })
}

/// A health body whose absorbance row DISAGREES WITH ITSELF —
/// `verdict: absorbs` carrying a nonzero `shortfall_at_least_us`.
///
/// `absorbance_verdict` cannot produce this (a `debug_assert` at its one exit
/// forbids it), which is the point: the row a READER must defend against comes
/// off a bag written by an unknown robot, a hand edit, or a writer from a
/// version nobody here has. Built as raw JSON for exactly that reason — going
/// through the producer would make the fixture impossible to write.
fn health_doc_with_contradictory_absorbance() -> serde_json::Value {
    serde_json::json!({
        "version": 2,
        "dropped_unwritten": 0,
        "loss_counting_basis": "prefix_proven",
        "topics": {
            TOPIC: {
                "frames_recorded": 10,
                "frames_lost": 0,
                "gap_events": 0,
                "absorbance": {
                    "tap_buffer_depth": 9,
                    "rate_mhz": 100_000,
                    "absorbance_us": 90_000,
                    "measured_tail_us": 25_000,
                    "required_depth": 3,
                    // THE CONTRADICTION: it absorbs, and it fell short.
                    "shortfall_at_least_us": 20_000,
                    "verdict": "absorbs",
                    "short_evaluations": 0,
                },
            }
        },
    })
}

/// A health body whose absorbance row is a
/// TAIL-LESS `Short` — the ladder-OVERFLOW shape — with the absorbance placed
/// where the caller asks and the recording's own ladder stated (or not).
///
/// The overflow arm is the one rule that is not arithmetic on the row's own
/// numbers: the producer emits it only while the absorbance is at most the
/// ladder's LAST edge, because above that nothing bounds the unserved tail from
/// below and the verdict is the silent `Unrankable`. So where the absorbance
/// sits relative to that edge is what decides whether the row is producible at
/// all — a reader that counted any such row as a genuine shortfall would
/// print its invented magnitude.
///
/// `edges_us` is the RECORDING's, not this build's, which is the whole reason
/// `DrainGapHistogram` carries its edges beside its counts.
///
/// The depth is DERIVED from the requested absorbance rather
/// than hardcoded. The validator also checks that the absorbance FOLLOWS
/// from the row's own depth and rate, so a fixture that moves the absorbance
/// while pinning the depth writes a row convicted for the wrong reason — and the
/// arm asserting such a row stays SOUND (the taller-ladder one) would fail
/// outright. At the fixed 100 Hz here one slot is 10 ms, so the caller's
/// absorbance must land on that grid.
fn health_doc_with_overflow_short(
    absorbance_us: u64,
    edges_us: Option<&[u64]>,
) -> serde_json::Value {
    const RATE_MHZ: u64 = 100_000;
    const US_PER_SLOT: u64 = 1_000_000_000 / RATE_MHZ;
    assert_eq!(
        absorbance_us % US_PER_SLOT,
        0,
        "an absorbance of {absorbance_us} us does not follow from any integer depth at \
         {RATE_MHZ} mHz, so the row would be convicted by the derivation rule rather than \
         by the ladder arm this fixture exists to drive"
    );
    let tap_buffer_depth = absorbance_us / US_PER_SLOT;
    // The claim is the DERIVED bound the producer emits — one past the ladder's
    // last edge, minus the absorbance (the validator holds tail-less
    // claims to that derivation).
    let derived_shortfall = edges_us
        .and_then(|e| e.last().copied())
        .unwrap_or(*cerulion_bagd::DRAIN_GAP_BUCKET_EDGES_US.last().unwrap())
        .saturating_add(1)
        .saturating_sub(absorbance_us);
    let mut doc = serde_json::json!({
        "version": 2,
        "dropped_unwritten": 0,
        "loss_counting_basis": "prefix_proven",
        "topics": {
            TOPIC: {
                "frames_recorded": 10,
                "frames_lost": 0,
                "gap_events": 0,
                "absorbance": {
                    "tap_buffer_depth": tap_buffer_depth,
                    "rate_mhz": RATE_MHZ,
                    "absorbance_us": absorbance_us,
                    // NO `measured_tail_us`: the overflow arm knows only that
                    // the tail exceeded the ladder.
                    "shortfall_at_least_us": derived_shortfall,
                    "verdict": "short",
                    "short_evaluations": 0,
                },
            }
        },
    });
    if let Some(edges) = edges_us {
        // Every gap in the OVERFLOW bucket, which is the state that produces a
        // tail-less row in the first place.
        let mut counts = vec![0u64; edges.len()];
        counts.push(100);
        doc["drain_gaps"] = serde_json::json!({ "edges_us": edges, "counts": counts });
    }
    doc
}

/// A `record_health.json` body with one entry per `(topic, labels, catch_up)`.
fn health_doc(topics: &[(&str, u64, bool)]) -> serde_json::Value {
    let mut map = serde_json::Map::new();
    for (topic, labels, catch_up) in topics {
        map.insert(
            (*topic).to_string(),
            serde_json::json!({
                "frames_recorded": 10,
                "frames_lost": 0,
                "gap_events": 0,
                "producer_labels": labels,
                "label_catch_up": catch_up,
            }),
        );
    }
    serde_json::json!({
        "version": 2,
        "dropped_unwritten": 0,
        "topics": serde_json::Value::Object(map),
    })
}

/// The PRODUCERS block is RENDERED by the real verb over a real bag.
///
/// The renderer is oracle-tested purely in `bag_cmd`'s own unit tests; what only
/// a real bag can pin is the CALL SITE — that `bag_info` reads the manifest and
/// pushes the block onto its output at all. Without this, `let _ =
/// read_producer_labels(..)` still lets the whole suite pass: every rendering
/// assertion in the repo drives the renderer directly.
///
/// Both halves in one body, because the silence is the half that matters most:
/// a single-writer recording earns no labels, so a block that appeared anyway
/// would appear on very nearly every bag ever made.
#[test]
fn bag_info_renders_the_producer_attribution_a_recording_earned() {
    let dir = tempfile::tempdir().expect("tempdir");

    // DECLARED (labels, no catch-up), OBSERVED (labels + catch-up), and a
    // single-writer topic that earned nothing — the FILTER's three inputs.
    let path = dir.path().join("labelled.mcap");
    write_bag_with_health(
        &path,
        &health_doc(&[
            ("/tf", 6, false),
            ("/joint_states", 12, true),
            ("/imu", 0, false),
        ]),
    );
    let text = bag_info(&path);
    assert!(
        text.contains("__cerulion/frame_producers"),
        "the block must name the channel a reader would go and decode: {text}"
    );
    let row = |topic: &str| -> Option<String> {
        text.lines()
            .find(|l| l.contains(topic) && l.contains("labelled frame(s)"))
            .map(str::to_string)
    };
    let declared = row("/tf").unwrap_or_else(|| panic!("no /tf row: {text}"));
    assert!(declared.contains("6 labelled frame(s)"), "{declared}");
    assert!(
        !declared.contains("catch-up"),
        "a DECLARED topic owes no catch-up: {declared}"
    );
    let observed = row("/joint_states").unwrap_or_else(|| panic!("no /joint_states row: {text}"));
    assert!(observed.contains("12 labelled frame(s)"), "{observed}");
    assert!(
        observed.contains("catch-up record attributing the run before the second writer"),
        "the marker is what says the plurality was OBSERVED rather than declared: {observed}"
    );
    assert!(
        row("/imu").is_none(),
        "a topic that earned no attribution must not get a row: {text}"
    );

    // The `|| label_catch_up` half of the filter, which a labels-only filter
    // drops: a catch-up was written but no frame of THIS batch was labelled.
    let only_catch_up = dir.path().join("catchup_only.mcap");
    write_bag_with_health(&only_catch_up, &health_doc(&[("/tf", 0, true)]));
    let text = bag_info(&only_catch_up);
    assert!(
        text.contains("/tf") && text.contains("catch-up record"),
        "a topic whose only attribution is its catch-up must still be reported: {text}"
    );

    // SILENCE: a health document in which nothing earned attribution renders no
    // block at all.
    let quiet = dir.path().join("single_writer.mcap");
    write_bag_with_health(
        &quiet,
        &health_doc(&[("/tf", 0, false), ("/imu", 0, false)]),
    );
    let text = bag_info(&quiet);
    assert!(
        text.contains(TOPIC),
        "the rest of `bag info` still works: {text}"
    );
    assert!(
        !text.contains("__cerulion/frame_producers"),
        "a single-writer recording must render NOTHING: {text}"
    );
}

/// Neither a bag that predates the health manifest nor a TORN one is reported as having no
/// producer attribution.
///
/// SCOPE: `Absent` and `IndexUnreadable` render IDENTICALLY (nothing), so
/// this arm cannot tell them apart — that discrimination is pinned where it is
/// observable, by `bag_cmd`'s
/// `the_producer_label_reading_separates_a_finalized_bag_from_a_torn_one`. What
/// this arm covers is the consequence: both states must render cleanly, must not
/// error, and must not print a claim about a bag nobody could read.
#[test]
fn a_bag_with_no_readable_health_manifest_claims_nothing_about_producers() {
    let dir = tempfile::tempdir().expect("tempdir");

    // FINALIZED, no manifest: every bag that predates the health manifest.
    let path = dir.path().join("pre597.mcap");
    write_bag(&path, None);
    let text = bag_info(&path);
    assert!(text.contains(TOPIC), "{text}");
    assert!(!text.contains("__cerulion/frame_producers"), "{text}");

    // TORN — the writer never finalized, so the attachment index cannot be
    // walked at all.
    let torn = dir.path().join("torn.mcap");
    let mut w = BagWriter::create(
        &torn,
        BagWriterConfig::default(),
        &[TopicSchema {
            topic: TOPIC.to_string(),
            schema_name: SCHEMA.to_string(),
            schema_hash: HASH,
            wire_fixed_size: 24,
        }],
    )
    .expect("create bag");
    let f = frame(0);
    w.write_chunk(|c| c.write_message(TOPIC, 0, 1_000_000_000, 1_000_000_000, &[&f[..]]))
        .expect("write frames");
    drop(w); // no `finalize` — no summary, no attachment index.
    let text = bag_info(&torn);
    assert!(
        !text.contains("__cerulion/frame_producers"),
        "an un-lookable index is not evidence that nothing was labelled: {text}"
    );
}

/// A CAPTURE-shaped bag: real producer records on the reserved channel and NO
/// `record_health.json` — which is exactly what a flashback capture is, since
/// the health manifest belongs to the continuous recorder and a capture carries
/// `__cerulion/flashback.json` instead.
fn write_capture_shaped_bag(path: &Path, labels: &[(u64, u128)], catch_up: Option<(u64, u128)>) {
    let mut w = BagWriter::create(
        path,
        BagWriterConfig::default(),
        &[TopicSchema {
            topic: TOPIC.to_string(),
            schema_name: SCHEMA.to_string(),
            schema_hash: HASH,
            wire_fixed_size: 24,
        }],
    )
    .expect("create bag");
    let label_channel = w.frame_producers_channel_id();
    let data_channel = w
        .channel_id(TOPIC)
        .expect("the data channel is registered by `create`");
    let records: Vec<Vec<u8>> = catch_up
        .into_iter()
        .map(|(prefix_len, publisher_id)| {
            cerulion_bag::ProducerRecord {
                attribution: cerulion_bag::ProducerAttribution::CatchUpPrefix { prefix_len },
                channel_id: data_channel,
                publisher_id,
            }
            .encode()
            .to_vec()
        })
        .chain(labels.iter().map(|(frame_index, publisher_id)| {
            cerulion_bag::ProducerRecord {
                attribution: cerulion_bag::ProducerAttribution::FrameLabel {
                    frame_index: *frame_index,
                },
                channel_id: data_channel,
                publisher_id: *publisher_id,
            }
            .encode()
            .to_vec()
        }))
        .collect();
    let f = frame(0);
    w.write_chunk(|c| {
        for (i, r) in records.iter().enumerate() {
            c.write_message(
                label_channel,
                i as u32,
                1_000_000_000,
                1_000_000_000,
                &[&r[..]],
            )?;
        }
        c.write_message(TOPIC, 0, 1_000_000_000, 1_000_000_000, &[&f[..]])
    })
    .expect("write frames + labels");
    w.finalize().expect("finalize");
}

/// A Flashback capture's producer attribution is
/// SURFACED, even though a capture carries no `record_health.json`.
///
/// # Why the reader needs a second source at all
///
/// The producers block reads `record_health.json`, which the
/// CONTINUOUS recorder writes. A capture carries `__cerulion/flashback.json`
/// instead, so without a second source the capture writer's real records
/// would be durable and reported by nothing — the "an artifact no shipped
/// command surfaces is a claim nobody can check" shape the block exists to close.
///
/// The fallback counts the bag's OWN records, which is ground truth rather than
/// a second manifest that could describe a different bag.
#[test]
fn a_capture_with_no_health_manifest_still_reports_the_labels_it_carries() {
    let dir = tempfile::tempdir().expect("tempdir");
    const A: u128 = 0x1111_2222_3333_4444_5555_6666_7777_8888;
    const B: u128 = 0x9999_AAAA_BBBB_CCCC_DDDD_EEEE_FFFF_0000;

    // A capture of a DECLARED shared topic: labels, no catch-up.
    let declared = dir.path().join("capture_declared.mcap");
    write_capture_shaped_bag(&declared, &[(0, A), (1, B), (2, A)], None);
    let text = bag_info(&declared);
    assert!(
        text.contains("producers:"),
        "a capture carrying producer records must render the producers block: {text}"
    );
    assert!(
        text.contains(TOPIC) && has_count(&text, 3),
        "…naming the topic and the count it actually carries: {text}"
    );
    assert!(
        !text.contains("catch-up record"),
        "a capture with no CatchUpPrefix must not claim one: {text}"
    );

    // A capture whose plurality was OBSERVED: the catch-up marker appears.
    let observed = dir.path().join("capture_observed.mcap");
    write_capture_shaped_bag(&observed, &[(5, B), (6, A)], Some((5, A)));
    let text = bag_info(&observed);
    assert!(
        has_count(&text, 2),
        "a CatchUpPrefix is not a frame label and must not be counted as one: {text}"
    );
    assert!(
        text.contains("catch-up record"),
        "an observed-plurality capture must say its leading run was attributed in one record: \
         {text}"
    );

    // ANTI-TAUTOLOGY: the same bag shape with NO records renders nothing. Without
    // it, a fallback that printed the block unconditionally would pass both
    // halves above.
    let single = dir.path().join("capture_single_writer.mcap");
    write_capture_shaped_bag(&single, &[], None);
    let text = bag_info(&single);
    assert!(
        !text.contains("producers:"),
        "a single-writer capture earns no labels, so it must render no producers block at all: \
         {text}"
    );
}

/// Does some line claim EXACTLY `n` of `what`, with the count read as a whole
/// whitespace TOKEN immediately before it?
///
/// This discipline belongs to EVERY count the file asserts, the producer
/// labels and the absorbance arms alike: a bare
/// `contains("1 fall SHORT")` is satisfied by "11 fall SHORT", inverting an
/// exact-count oracle into one a wrong renderer passes. Same prefix-collision
/// class the bagd suite pins with `has_field`; one helper, so a new arm
/// cannot reintroduce it. Its own oracle is
/// `a_count_is_matched_as_a_whole_token_not_a_substring`.
fn has_count_of(text: &str, n: u64, what: &str) -> bool {
    let count = n.to_string();
    let tail: Vec<&str> = what.split_whitespace().collect();
    assert!(
        !tail.is_empty(),
        "the phrase after the count cannot be empty"
    );
    text.lines().any(|line| {
        let toks: Vec<&str> = line.split_whitespace().collect();
        toks.windows(1 + tail.len())
            .any(|w| w[0] == count && w[1..] == tail[..])
    })
}

/// Does the rendered block claim EXACTLY `n` labelled frames for some topic?
fn has_count(text: &str, n: u64) -> bool {
    has_count_of(text, n, "labelled")
}

/// The helper's own oracle: the superstring a bare `contains` waves through.
#[test]
fn a_count_is_matched_as_a_whole_token_not_a_substring() {
    let doctored = "  absorbance: 11 fall SHORT of them";
    assert!(
        doctored.contains("1 fall SHORT"),
        "precondition: this is exactly the string the old substring form accepted"
    );
    assert!(
        !has_count_of(doctored, 1, "fall SHORT"),
        "a report printing ELEVEN must not satisfy an assertion that ONE fell short"
    );
    assert!(
        has_count_of(doctored, 11, "fall SHORT"),
        "…and the true count still matches (anti-tautology: the helper is not just false)"
    );
    // The phrase must FOLLOW the count, not merely share the line with it.
    assert!(!has_count_of(
        "1 topic(s); 3 fall SHORT of them",
        1,
        "fall SHORT"
    ));
    // …and multi-word phrases match across a real multi-line report.
    assert!(has_count_of(
        "absorbance (from record_health.json):\n  0 absorb the drain stalls it measured\n",
        0,
        "absorb the drain stalls"
    ));
}

/// A capture-shaped bag whose reserved channel carries a MIX of readable and
/// unreadable records — `(frame_index, publisher_id, skewed)`.
///
/// `skewed` flips the VERSION byte only, so the record keeps the right length
/// and the arm pins the version gate rather than a length check.
fn write_skewed_bag(path: &Path, records: &[(u64, u128, bool)]) {
    let mut w = BagWriter::create(
        path,
        BagWriterConfig::default(),
        &[TopicSchema {
            topic: TOPIC.to_string(),
            schema_name: SCHEMA.to_string(),
            schema_hash: HASH,
            wire_fixed_size: 24,
        }],
    )
    .expect("create bag");
    let label_channel = w.frame_producers_channel_id();
    let data_channel = w.channel_id(TOPIC).expect("data channel");
    let encoded: Vec<[u8; cerulion_bag::PRODUCER_RECORD_SIZE]> = records
        .iter()
        .map(|(frame_index, publisher_id, skewed)| {
            let mut r = cerulion_bag::ProducerRecord {
                attribution: cerulion_bag::ProducerAttribution::FrameLabel {
                    frame_index: *frame_index,
                },
                channel_id: data_channel,
                publisher_id: *publisher_id,
            }
            .encode();
            if *skewed {
                r[0] = cerulion_bag::PRODUCER_RECORD_VERSION
                    .checked_add(1)
                    .expect("a next version exists");
            }
            r
        })
        .collect();
    let f = frame(0);
    w.write_chunk(|c| {
        for (i, r) in encoded.iter().enumerate() {
            c.write_message(
                label_channel,
                i as u32,
                1_000_000_000,
                1_000_000_000,
                &[&r[..]],
            )?;
        }
        c.write_message(TOPIC, 0, 1_000_000_000, 1_000_000_000, &[&f[..]])
    })
    .expect("write frames + labels");
    w.finalize().expect("finalize");
}

/// A capture whose records this build cannot read is
/// reported as unreadable, never as a capture that labels nothing.
///
/// # The event this is for
///
/// `ProducerRecord::decode` refuses an unknown `PRODUCER_RECORD_VERSION`, which
/// is precisely what the version byte exists to anticipate: a capture written by
/// a NEWER binary and read by this one. Every record then fails to decode. If
/// those failures were merely skipped, `rows` would end up empty and the reader
/// would return `Present(vec![])` — the STRONGEST arm the enum has, stating that
/// a bag carrying complete per-frame attribution carries none. A fabricated
/// absence, on the one artifact whose whole purpose is post-incident
/// attribution.
#[test]
fn a_capture_whose_records_this_build_cannot_read_says_so_rather_than_claiming_none() {
    let dir = tempfile::tempdir().expect("tempdir");
    const A: u128 = 0x1111_2222_3333_4444_5555_6666_7777_8888;

    // A capture-shaped bag whose producer records carry a FUTURE version byte.
    // Built by hand rather than via `ProducerRecord::encode`, because `encode`
    // can only ever write the version this build knows.
    let future = dir.path().join("future_version.mcap");
    {
        let mut w = BagWriter::create(
            &future,
            BagWriterConfig::default(),
            &[TopicSchema {
                topic: TOPIC.to_string(),
                schema_name: SCHEMA.to_string(),
                schema_hash: HASH,
                wire_fixed_size: 24,
            }],
        )
        .expect("create bag");
        let label_channel = w.frame_producers_channel_id();
        let data_channel = w.channel_id(TOPIC).expect("data channel");
        // A well-formed record of the RIGHT LENGTH whose only defect is its
        // version byte — so this pins the VERSION gate and not a length check.
        let mut rec = cerulion_bag::ProducerRecord {
            attribution: cerulion_bag::ProducerAttribution::FrameLabel { frame_index: 0 },
            channel_id: data_channel,
            publisher_id: A,
        }
        .encode();
        rec[0] = cerulion_bag::PRODUCER_RECORD_VERSION
            .checked_add(1)
            .expect("a next version exists");
        let f = frame(0);
        w.write_chunk(|c| {
            c.write_message(label_channel, 0, 1_000_000_000, 1_000_000_000, &[&rec[..]])?;
            c.write_message(TOPIC, 0, 1_000_000_000, 1_000_000_000, &[&f[..]])
        })
        .expect("write frames + labels");
        w.finalize().expect("finalize");
    }

    let text = bag_info(&future);
    assert!(
        text.contains("producers:") && text.contains("could not be read by this build"),
        "a capture whose records this build cannot decode must SAY so: {text}"
    );
    // Rejecting a rendered ZERO is too weak — a renderer that
    // printed "1 labelled frame(s)" for a bag whose only record it could not read
    // would pass it. NO per-topic row may be rendered at all.
    assert!(
        !text.contains("labelled frame(s)"),
        "an all-undecodable stream must render NO per-topic count row of any value — every row \
         would be a claim about records this build could not read: {text}"
    );
    // THE ARTIFACT. A capture carries NO `record_health.json` — its absence is
    // the very reason the fallback ran — so naming it here would tell an
    // operator that a file the bag does not contain is "present but MALFORMED".
    // The damage is on the reserved channel, and that is what must be named.
    assert!(
        !text.contains(cerulion_bagd::RECORD_HEALTH_ATTACHMENT),
        "the skew line must NOT name `record_health.json`: a capture has none, and its absence \
         is why this path ran at all: {text}"
    );
    assert!(
        text.contains(cerulion_bag::FRAME_PRODUCERS_TOPIC),
        "…it must name the channel the unreadable records are actually on: {text}"
    );

    // PARTIAL SKEW: one readable record beside one this build cannot read. The
    // rows that survived still render — withholding a real report over one stray
    // record would be worse — but every count becomes a FLOOR and must say so.
    // `decode` refuses on length, version AND an unknown KIND, and the kind axis
    // needs no version bump, so this mixed stream is reachable without one.
    let partial = dir.path().join("partial_skew.mcap");
    write_skewed_bag(&partial, &[(0, A, false), (1, A, true)]);
    let text = bag_info(&partial);
    assert!(
        text.contains("at least 1 labelled frame(s)"),
        "a partially-readable stream must render its surviving row as a FLOOR, never as an exact \
         total: {text}"
    );
    assert!(
        text.contains("could not be read by this build"),
        "…and must say how many records it could not account for: {text}"
    );

    // ANTI-TAUTOLOGY: the SAME bag shape with a readable version renders a real
    // row. Without it, a reader that always printed the unreadable line would
    // pass the half above.
    let readable = dir.path().join("readable_version.mcap");
    write_capture_shaped_bag(&readable, &[(0, A)], None);
    let text = bag_info(&readable);
    assert!(
        has_count(&text, 1) && !text.contains("could not be read by this build"),
        "a capture this build CAN read must render its row and no skew line: {text}"
    );
}

/// Run the production verb, failing loudly rather than returning an error the
/// caller could accidentally assert nothing about.
fn bag_info(path: &Path) -> String {
    bag_cmd::bag_info(path, None).unwrap_or_else(|e| panic!("bag info failed: {e}"))
}

/// The ABSORBANCE block is RENDERED by the real verb over a
/// real bag, and it says WHOSE run the verdicts describe.
///
/// The renderer is oracle-tested purely in `bag_cmd`'s own unit tests, which
/// hand-construct an `AbsorbanceReading`; what only a real bag can pin is the
/// CALL SITE — that `bag_info` goes looking for the attachment, decodes it, and
/// pushes the block onto its output. Without this, `let _ = read_absorbance(..)`
/// still lets the whole suite pass.
///
/// The two SCOPES ride in one body because the thing that distinguishes them is
/// the attachment NAME, and a reader that searched only one name, or paired the
/// names with the wrong scopes, produces a plausible-looking block that is
/// affirmatively wrong about whose run it describes — which is the entire reason
/// `CAPTURE_RECORDER_HEALTH_ATTACHMENT` is a separate constant.
#[test]
fn bag_info_renders_the_absorbance_verdicts_and_says_whose_run_they_describe() {
    let dir = tempfile::tempdir().expect("tempdir");

    // (1) A RECORDING: the verdicts are this bag's own.
    let recording = dir.path().join("recording.mcap");
    write_bag_with_attachment(
        &recording,
        cerulion_bagd::RECORD_HEALTH_ATTACHMENT,
        &serde_json::to_vec(&health_doc_with_absorbance("prefix_invisible", "short", 4))
            .expect("encode"),
    );
    let text = bag_info(&recording);
    assert!(
        text.contains("absorbance: 1 topic(s) carry a verdict"),
        "the block must reach the output at all: {text}"
    );
    assert!(
        text.contains("this recording"),
        "a recording's verdicts describe THIS recording: {text}"
    );
    assert!(
        !text.contains("the RECORDER that took this capture"),
        "…and must not claim to be a capture's: {text}"
    );
    assert!(
        text.contains("fell short 4 time(s) this run"),
        "the sticky count is what says the episode happened: {text}"
    );
    // The COUNTING CAVEAT, on the basis that earned it.
    assert!(
        text.contains("nothing was COUNTED"),
        "a prefix-invisible recording must state what its zeros cannot see: {text}"
    );

    // (2) A CAPTURE: the same document under the capture's own name describes
    //     the RECORDER's whole run, not this capture's window.
    let capture = dir.path().join("capture.mcap");
    write_bag_with_attachment(
        &capture,
        cerulion_bagd::CAPTURE_RECORDER_HEALTH_ATTACHMENT,
        &serde_json::to_vec(&health_doc_with_absorbance("prefix_proven", "short", 4))
            .expect("encode"),
    );
    let text = bag_info(&capture);
    assert!(
        text.contains("the RECORDER that took this capture"),
        "a capture's verdicts are the RECORDER's, over its whole run: {text}"
    );
    assert!(
        !text.contains("absorb the drain stalls this recording measured"),
        "…and must not be attributed to the capture: {text}"
    );
    // PrefixProven is the ONE basis that does not print the caveat, so this half
    // is also the caveat's own negative control.
    assert!(
        !text.contains("nothing was COUNTED"),
        "an armed-before-producers recorder CAN account for a head loss, so the \
         caveat must not print: {text}"
    );

    // (3) ANTI-TAUTOLOGY: a bag with NO health attachment renders no block at
    //     all. Without this, every assertion above is satisfied by a verb that
    //     prints the heading unconditionally.
    let bare = dir.path().join("bare.mcap");
    write_bag(&bare, None);
    let text = bag_info(&bare);
    assert!(
        !text.contains("absorbance:"),
        "an ABSENT verdict is UNKNOWN — a heading over no rows would read as \
         'this recording was checked': {text}"
    );

    // (4) A MALFORMED capture document names the attachment the bag ACTUALLY
    //     holds. Naming the recording's would send an operator to something
    //     provably not in their bag.
    let broken = dir.path().join("broken.mcap");
    write_bag_with_attachment(
        &broken,
        cerulion_bagd::CAPTURE_RECORDER_HEALTH_ATTACHMENT,
        b"{ this is not json",
    );
    let text = bag_info(&broken);
    assert!(
        text.contains("MALFORMED"),
        "an unreadable report must be reported, never silently skipped: {text}"
    );
    assert!(
        text.contains(cerulion_bagd::CAPTURE_RECORDER_HEALTH_ATTACHMENT),
        "…naming the attachment this bag holds: {text}"
    );
    assert!(
        !text.contains(cerulion_bagd::RECORD_HEALTH_ATTACHMENT),
        "…and not the one it does not: {text}"
    );
}

/// A decoded row whose verdict contradicts its
/// own numbers is reported by the REAL VERB over a REAL BAG — named, and not
/// counted as healthy.
///
/// The renderer half is oracle-tested in `bag_cmd`'s own unit arms; what only a
/// real bag can pin is that the validation runs on the DECODE path at all —
/// that `read_absorbance` does not collect externally supplied rows straight
/// into an aggregate. Were `verdict` to drive every count while the
/// numbers are read separately, this bag — which says a tap fell 20 ms short
/// — would be summarised as one where every tap absorbed, with the row filtered
/// off the actionable list by a predicate reading the very fields in dispute.
#[test]
fn bag_info_reports_an_absorbance_row_that_disagrees_with_itself() {
    let dir = tempfile::tempdir().expect("tempdir");
    let path = dir.path().join("contradictory.mcap");
    write_bag_with_attachment(
        &path,
        cerulion_bagd::RECORD_HEALTH_ATTACHMENT,
        &serde_json::to_vec(&health_doc_with_contradictory_absorbance()).expect("encode"),
    );

    let text = bag_info(&path);
    assert!(
        has_count_of(&text, 1, "DISAGREE WITH THEMSELVES"),
        "the contradiction must reach the output: {text}"
    );
    assert!(
        text.contains("DISAGREE WITH THEMSELVES and are counted in none of the above"),
        "…and stays out of every other bucket: {text}"
    );
    assert!(
        has_count_of(&text, 0, "absorb the drain stalls"),
        "…and must NOT be counted as healthy: {text}"
    );
    let row = text
        .lines()
        .find(|l| l.contains("INCONSISTENT"))
        .unwrap_or_else(|| panic!("no inconsistent row: {text}"));
    assert!(row.contains(TOPIC), "the row must name the topic: {row}");
    assert!(
        row.contains("carries a shortfall"),
        "…and what disagrees: {row}"
    );

    // ANTI-TAUTOLOGY, over the SAME verb: a sound document says nothing about
    // disagreement, so the clause is not printed on every bag.
    let sound = dir.path().join("sound.mcap");
    write_bag_with_attachment(
        &sound,
        cerulion_bagd::RECORD_HEALTH_ATTACHMENT,
        &serde_json::to_vec(&health_doc_with_absorbance("prefix_proven", "short", 4))
            .expect("encode"),
    );
    let text = bag_info(&sound);
    assert!(!text.contains("DISAGREE"), "{text}");
    assert!(!text.contains("INCONSISTENT"), "{text}");
    assert!(
        has_count_of(&text, 1, "fall SHORT"),
        "…while its own counts are unaffected: {text}"
    );
}

/// At the user-facing surface: `bag info` must not
/// count a TAIL-LESS `Short` row as a genuine shortfall when its absorbance sits
/// above the ladder that row was ranked against.
///
/// The unit oracle in `cerulion_bagd::absorbance` pins the rule; this pins the
/// CONSEQUENCE — the count, the row, and the ladder the reader judged against,
/// which no unit test can see because the ladder is read out of the bag.
#[test]
fn bag_info_refuses_a_tail_less_short_row_above_the_recordings_own_ladder() {
    let dir = tempfile::tempdir().expect("tempdir");
    let edges = cerulion_bagd::DRAIN_GAP_BUCKET_EDGES_US;
    let edge = *edges.last().expect("a ladder has a top");
    // ONE DEPTH STEP past the edge: the absorbance must still follow from
    // the row's own depth and rate, and at the fixture's 100 Hz the depth
    // quantum is 10 ms — so the nearest producible point above the edge is a
    // deeper queue, not a microsecond. Nothing is lost: the `>` boundary itself
    // is pinned from below by the `at_edge` arm, whose absorbance IS the edge
    // and which must stay sound.
    let past_edge = edge + 10_000;
    assert!(past_edge > edge);

    let info = |name: &str, doc: serde_json::Value| -> String {
        let path = dir.path().join(name);
        write_bag_with_attachment(
            &path,
            cerulion_bagd::RECORD_HEALTH_ATTACHMENT,
            &serde_json::to_vec(&doc).expect("encode"),
        );
        bag_info(&path)
    };

    // AT the edge: the row the producer really emits there. SOUND, and counted.
    let at_edge = info(
        "at_edge.mcap",
        health_doc_with_overflow_short(edge, Some(edges)),
    );
    assert!(
        !at_edge.contains("DISAGREE") && !at_edge.contains("INCONSISTENT"),
        "the boundary row is producible and must stay sound: {at_edge}"
    );
    assert!(
        has_count_of(&at_edge, 1, "fall SHORT"),
        "…and must still be counted: {at_edge}"
    );

    // ONE DEPTH STEP past it: unproducible, so neither believed nor discarded.
    let past = info(
        "past_edge.mcap",
        health_doc_with_overflow_short(past_edge, Some(edges)),
    );
    assert!(
        past.contains("DISAGREE WITH THEMSELVES and are counted in none of the above"),
        "a row above the ladder describes a comparison nobody made: {past}"
    );
    assert!(
        has_count_of(&past, 0, "fall SHORT"),
        "…and must NOT be counted as a genuine shortfall: {past}"
    );
    let row = past
        .lines()
        .find(|l| l.contains("INCONSISTENT"))
        .unwrap_or_else(|| panic!("no inconsistent row: {past}"));
    assert!(row.contains(TOPIC), "the row must name the topic: {row}");
    assert!(
        row.contains("above the highest gap the ladder can rank"),
        "…and what disagrees: {row}"
    );

    // THE LADDER IS THE RECORDING'S. The identical row, ranked by a bag whose
    // own ladder reaches higher, is sound on its own terms — a reader must not
    // convict it on this build's constant. (This pins the plumbing: a
    // reader that always uses `DRAIN_GAP_BUCKET_EDGES_US` fails here.)
    let taller: Vec<u64> = edges
        .iter()
        .copied()
        .chain(std::iter::once(edge * 4))
        .collect();
    let foreign = info(
        "taller.mcap",
        health_doc_with_overflow_short(past_edge, Some(&taller)),
    );
    assert!(
        !foreign.contains("INCONSISTENT") && has_count_of(&foreign, 1, "fall SHORT"),
        "a bag carrying its own taller ladder states a producible row: {foreign}"
    );

    // …and a document that states NO ladder is not thereby exempt: it is judged
    // against the only vocabulary its reader speaks.
    let ladderless = info(
        "no_ladder.mcap",
        health_doc_with_overflow_short(past_edge, None),
    );
    assert!(
        ladderless.contains("INCONSISTENT"),
        "a bag that names no ladder still gets checked against ours: {ladderless}"
    );
}
