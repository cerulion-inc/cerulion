// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle-vector pins for `cerulion_core::state_restore`,
//! the pure decision layer a restore runs BEFORE any byte reaches a node.
//!
//! Every assertion here is against a HAND-WRITTEN expected value, never
//! against a second call of the function under test. That matters more than
//! usual for this module: its whole job is to REFUSE recordings that would
//! replay to a confident wrong answer, so a test that merely agrees with the
//! implementation would pass just as happily against the defect.
//!
//! Three groups are called out because they exist to close specific measured
//! holes rather than to cover a surface:
//!
//! - **the backlog frame gate** (`plan_backlog_admission`) — a consumer with an undrained
//!   backlog at the anchor must be re-fed exactly the frames it was captured
//!   holding, and two consumers of one topic that disagree must be REFUSED
//!   rather than served the maximum.
//! - **the seed ladder** (`seed_publisher_sequence`) — a re-executed topic with no
//!   frame at or before the anchor is the ordinary shape of a window bag, and
//!   the one-line rule "last recorded sequence before the anchor, plus 1" is
//!   undefined there.
//! - **the shape gate** (`AnchorBlob` + `classify_shape`) — an anchor blob
//!   carries the capture-time `STATE_SHAPE` because nothing else in the
//!   recording does, and an UNFRAMED blob is refused rather than sniffed.
//!
//! No transport, no SHM, no bag — parallel-safe, no `#[serial]`, and **no
//! `#![cfg(unix)]`**: the module under test decides whether a RECORDED bag may
//! be replayed, which a desk that never runs the unix-only capture carrier must
//! be able to do. `the_restore_decision_layer_names_no_unix_only_module` is the
//! structural half of that claim (a cross-target `cargo check` is not runnable
//! here — the iceoryx2 POSIX PAL's bindgen step needs the target's own C
//! headers — so the guard walks the source instead).

use cerulion_core::state::{CerulionState, SkipCause, StateCursor, VecSink};
use cerulion_core::state_restore::{
    capture_anchor_blob, capture_anchor_blob_with_framework, classify_shape, enforce_strict_state,
    plan_backlog_admission, plan_restore, refuse_lossy_reexecuted_topics, seed_publisher_sequence,
    AnchorBlob, AnchorFact, AnchorOutcome, BacklogClaim, InputServiceCursor, NodeAnchorProblem,
    NodeFrameworkState, RestorePlan, RestoreRefusal, RestoreRequest, SeedEvidence, SeedRung,
    TopicFrameCount, TopicLoss, TopicWriters, ANCHOR_BLOB_HEADER_SIZE, ANCHOR_BLOB_MAGIC,
    ANCHOR_BLOB_MAGIC_V2, ANCHOR_BLOB_V2_HEADER_SIZE, FRAMEWORK_SECTION_MIN_READABLE,
    FRAMEWORK_SECTION_WRITE_VERSION,
};

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

fn names(list: &[&str]) -> Vec<String> {
    list.iter().map(|s| s.to_string()).collect()
}

fn complete(run_id: u64, step: u64, node: &str) -> AnchorFact {
    AnchorFact {
        run_id,
        step,
        node: node.to_string(),
        outcome: AnchorOutcome::Complete,
    }
}

fn torn(run_id: u64, step: u64, node: &str) -> AnchorFact {
    AnchorFact {
        run_id,
        step,
        node: node.to_string(),
        outcome: AnchorOutcome::Torn,
    }
}

fn skipped(run_id: u64, step: u64, node: &str, cause: SkipCause) -> AnchorFact {
    AnchorFact {
        run_id,
        step,
        node: node.to_string(),
        outcome: AnchorOutcome::Skipped(cause),
    }
}

const RUN: u64 = 0x0000_C1E5_0000_0009;
const OTHER_RUN: u64 = 0x0000_C1E5_0000_00FF;

// ===========================================================================
// AnchorBlob — the framing that carries the shape
// ===========================================================================

#[test]
fn the_anchor_header_is_the_ascii_magic_then_the_shape_little_endian() {
    // The header is a WIRE format, so its bytes are pinned by hand rather than
    // by round-tripping them through the decoder that reads them.
    let mut sink = VecSink::new();
    AnchorBlob::write_header(0x0102_0304_0506_0708, &mut sink).expect("header fits");
    let bytes = sink.into_inner();

    assert_eq!(bytes.len(), ANCHOR_BLOB_HEADER_SIZE, "header is 16 bytes");
    assert_eq!(&bytes[0..8], b"CERSTATE", "magic is the ASCII word");
    assert_eq!(
        &bytes[8..16],
        &[0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01],
        "the shape is little-endian, matching every other Cerulion wire field"
    );
    assert_eq!(
        ANCHOR_BLOB_MAGIC.to_le_bytes(),
        *b"CERSTATE",
        "the exported constant and the emitted bytes are the same word"
    );
}

#[test]
fn a_framed_capture_decodes_to_the_declared_shape_and_the_unframed_payload() {
    // The payload oracle is an INDEPENDENT capture of the same value through
    // the bare encoder: the framing must add a header and change nothing else.
    let value: u32 = 0xDEAD_BEEF;

    let mut framed = VecSink::new();
    capture_anchor_blob(&value, &mut framed).expect("capture fits");
    let framed = framed.into_inner();

    let mut bare = VecSink::new();
    value.cer_capture(&mut bare).expect("capture fits");
    let bare = bare.into_inner();

    let blob = AnchorBlob::decode("node", &framed).expect("framed");
    assert_eq!(blob.state_shape, u32::STATE_SHAPE);
    assert_eq!(blob.payload, &bare[..], "the payload is the unframed bytes");
    assert_eq!(framed.len(), ANCHOR_BLOB_HEADER_SIZE + bare.len());

    // And the payload still decodes to the original value, so the framing is
    // not merely the right LENGTH.
    let mut cursor = StateCursor::new(blob.payload);
    let read = u32::cer_read(&mut cursor).expect("decode");
    cursor.finish().expect("payload fully consumed");
    assert_eq!(read, value);
}

#[test]
fn a_zero_byte_state_is_a_real_anchor_not_an_absent_one() {
    // A node whose state encodes to nothing is captured, and its anchor must
    // survive the framing round trip — otherwise it is indistinguishable from
    // a node that was never captured at all.
    let mut sink = VecSink::new();
    AnchorBlob::write_header(0xABCD, &mut sink).expect("header fits");
    let bytes = sink.into_inner();

    let blob = AnchorBlob::decode("stateless", &bytes).expect("framed");
    assert_eq!(blob.state_shape, 0xABCD);
    assert!(blob.payload.is_empty(), "no payload, but a valid anchor");
}

#[test]
fn an_unframed_blob_is_refused_by_name_rather_than_sniffed() {
    // Sixteen bytes that could be read as a shape if the decoder guessed.
    let unframed = [7u8; 16];
    let err = AnchorBlob::decode("slam", &unframed).expect_err("no magic");
    match &err {
        RestoreRefusal::UnframedAnchor { node, bytes } => {
            assert_eq!(node, "slam");
            assert_eq!(*bytes, 16);
        }
        other => panic!("expected UnframedAnchor, got {other:?}"),
    }
    let text = err.to_string();
    assert!(text.contains("slam"), "names the node: {text}");
    assert!(
        text.contains("capture_anchor_blob"),
        "names the fix: {text}"
    );
}

#[test]
fn a_blob_shorter_than_its_own_header_is_refused_with_its_length() {
    // Fifteen bytes that OPEN with the right magic — the truncation is the
    // only thing wrong with them, and it must not be read as a shape.
    let mut short = ANCHOR_BLOB_MAGIC.to_le_bytes().to_vec();
    short.extend_from_slice(&[0u8; 7]);
    assert_eq!(short.len(), ANCHOR_BLOB_HEADER_SIZE - 1);

    let err = AnchorBlob::decode("truncated", &short).expect_err("too short");
    assert_eq!(
        err,
        RestoreRefusal::UnframedAnchor {
            node: "truncated".to_string(),
            bytes: 15,
        }
    );

    // The empty blob is the boundary's other side.
    assert_eq!(
        AnchorBlob::decode("empty", &[]).expect_err("too short"),
        RestoreRefusal::UnframedAnchor {
            node: "empty".to_string(),
            bytes: 0,
        }
    );
}

#[test]
fn one_wrong_byte_in_the_magic_is_enough_to_refuse() {
    let mut nearly = ANCHOR_BLOB_MAGIC.to_le_bytes().to_vec();
    nearly[0] ^= 0x01;
    nearly.extend_from_slice(&0u64.to_le_bytes());
    nearly.extend_from_slice(b"payload");

    assert!(matches!(
        AnchorBlob::decode("nearly", &nearly),
        Err(RestoreRefusal::UnframedAnchor { .. })
    ));
}

// ===========================================================================
// The framework section
// ===========================================================================
//
// The section states the SCHEDULER's own view of a node at the anchor, which
// is the half of a checkpoint a rebuilt `GraphRuntime` cannot recover: its
// callbacks and shared counters are wiring the build re-makes, while
// `next_fire_ns` / `pending_data_count` / `sync_input_timestamps` are plain
// data nothing else in the recording states.
//
// Every arm below is a BYTE oracle or a hand-written value, never a
// round-trip compared against itself, because the whole point of the format is
// that a bag written by one build is read by another.

/// A section with all three fields populated, and every value distinct so a
/// field read out of the wrong offset cannot pass.
fn populated_section() -> NodeFrameworkState {
    let mut sync = std::collections::BTreeMap::new();
    sync.insert("/lidar".to_string(), 0x0000_0000_1111_2222u64);
    sync.insert("/cam".to_string(), 0x0000_0000_3333_4444u64);
    NodeFrameworkState {
        next_fire_ns: Some(0x0000_0000_0102_0304),
        pending_data_count: 0x0000_0000_0000_0007,
        sync_input_timestamps: sync,
        // Deliberately a VERSION-1 section: it states no service
        // table, so every arm built on it keeps exercising the v1 layout the
        // whole earlier bag population is written in.
        input_service: None,
    }
}

/// [`populated_section`] plus a service table — a VERSION-2 section,
/// with both cursor cases present and every value distinct.
fn populated_v2_section() -> NodeFrameworkState {
    let mut cursors = std::collections::BTreeMap::new();
    cursors.insert("cam".to_string(), Some(0x0055_6677_u32));
    cursors.insert("lidar".to_string(), None);
    NodeFrameworkState {
        input_service: Some(cursors),
        ..populated_section()
    }
}

#[test]
fn a_v1_anchor_carries_no_framework_section_and_is_byte_unchanged() {
    // THE back-compat pin. Every bag recorded without a framework section is
    // framed with the v1 magic, and this build must read one exactly as a
    // v1-only build does: same shape, same payload, and NO section — not an empty one it
    // then acts on.
    let value: u32 = 0xDEAD_BEEF;
    let mut framed = VecSink::new();
    capture_anchor_blob(&value, &mut framed).expect("capture fits");
    let framed = framed.into_inner();

    let blob = AnchorBlob::decode("legacy", &framed).expect("framed");
    assert_eq!(blob.state_shape, u32::STATE_SHAPE);
    assert!(
        blob.framework.is_empty(),
        "a v1 blob declares no framework section"
    );
    assert_eq!(
        blob.framework_state("legacy").expect("no section to parse"),
        None,
        "absent must read as None — not as a section of zeros, which would be \
         the positive claim `this node had nothing pending`"
    );
    assert_eq!(framed.len(), ANCHOR_BLOB_HEADER_SIZE + 4);
}

#[test]
fn a_framework_section_rides_between_the_header_and_the_payload_as_pinned_bytes() {
    // The section is a WIRE format, so its bytes are pinned by hand rather
    // than by round-tripping them through the decoder that reads them.
    let value: u32 = 0xDEAD_BEEF;
    let framework = populated_section();

    let mut framed = VecSink::new();
    capture_anchor_blob_with_framework(&value, &framework, &mut framed).expect("capture fits");
    let framed = framed.into_inner();

    let mut bare = VecSink::new();
    value.cer_capture(&mut bare).expect("capture fits");
    let bare = bare.into_inner();

    // Hand-built oracle for the whole blob, byte for byte.
    let mut expected: Vec<u8> = Vec::new();
    expected.extend_from_slice(b"CERSTAT2");
    expected.extend_from_slice(&u32::STATE_SHAPE.to_le_bytes());
    // section length: 25-byte v1 prefix + (4 + 4 + 8) for "/cam" + (4 + 6 + 8)
    // for "/lidar" = 25 + 16 + 18 = 59.
    expected.extend_from_slice(&59u32.to_le_bytes());
    expected.extend_from_slice(&1u32.to_le_bytes()); // section version
    expected.push(1); // next_fire present
    expected.extend_from_slice(&0x0000_0000_0102_0304u64.to_le_bytes());
    expected.extend_from_slice(&7u64.to_le_bytes()); // pending_data_count
    expected.extend_from_slice(&2u32.to_le_bytes()); // sync count
                                                     // SORTED by name, so "/cam" precedes "/lidar" whatever order the live
                                                     // scheduler's insertion-ordered map held them in.
    expected.extend_from_slice(&4u32.to_le_bytes());
    expected.extend_from_slice(b"/cam");
    expected.extend_from_slice(&0x0000_0000_3333_4444u64.to_le_bytes());
    expected.extend_from_slice(&6u32.to_le_bytes());
    expected.extend_from_slice(b"/lidar");
    expected.extend_from_slice(&0x0000_0000_1111_2222u64.to_le_bytes());
    expected.extend_from_slice(&bare);

    assert_eq!(framed, expected, "the framed blob is the hand-built bytes");
    assert_eq!(
        ANCHOR_BLOB_MAGIC_V2.to_le_bytes(),
        *b"CERSTAT2",
        "the exported constant and the emitted bytes are the same word"
    );
    assert_eq!(
        framework.encoded_len(),
        59,
        "encoded_len is computed analytically and must equal what encode wrote"
    );

    // And the split the decoder makes lands on the same three regions.
    let blob = AnchorBlob::decode("fusion", &framed).expect("framed");
    assert_eq!(blob.state_shape, u32::STATE_SHAPE);
    assert_eq!(blob.payload, &bare[..], "the payload is the unframed bytes");
    assert_eq!(
        blob.framework_state("fusion").expect("readable section"),
        Some(framework),
        "the section decodes to the value that was captured"
    );
}

#[test]
fn the_sections_byte_order_does_not_depend_on_the_schedulers_insertion_order() {
    // These bytes land in a bag, so two captures of the same logical
    // state must be byte-identical. The scheduler holds sync inputs in an
    // insertion-ordered `IndexMap`, and two graphs can wire the same two
    // inputs in either order.
    let mut forward = std::collections::BTreeMap::new();
    forward.insert("/cam".to_string(), 11u64);
    forward.insert("/lidar".to_string(), 22u64);
    let mut reverse = std::collections::BTreeMap::new();
    reverse.insert("/lidar".to_string(), 22u64);
    reverse.insert("/cam".to_string(), 11u64);

    let encode = |sync: std::collections::BTreeMap<String, u64>| {
        let mut sink = VecSink::new();
        NodeFrameworkState {
            next_fire_ns: None,
            pending_data_count: 0,
            sync_input_timestamps: sync,
            input_service: None,
        }
        .encode(&mut sink)
        .expect("fits");
        sink.into_inner()
    };
    let bytes = encode(forward);
    assert_eq!(bytes, encode(reverse));
    // Anti-tautology: the encoding really does carry both names, so the
    // equality above is not two empty vectors agreeing.
    assert!(bytes.windows(4).any(|w| w == b"/cam"));
    assert!(bytes.windows(6).any(|w| w == b"/lidar"));
}

#[test]
fn a_non_period_nodes_absent_deadline_survives_the_round_trip_as_none() {
    // `next_fire_ns` is `Some` only for a `Period` trigger, and the difference
    // between "no deadline" and "a deadline of 0" is the difference between
    // leaving a node alone and making it due at the clock's origin.
    let section = NodeFrameworkState {
        next_fire_ns: None,
        pending_data_count: 3,
        sync_input_timestamps: std::collections::BTreeMap::new(),
        input_service: None,
    };
    let mut sink = VecSink::new();
    section.encode(&mut sink).expect("fits");
    let bytes = sink.into_inner();

    assert_eq!(bytes[4], 0, "the presence byte says absent");
    assert_eq!(
        &bytes[5..13],
        &0u64.to_le_bytes(),
        "an absent deadline is written as zero, so the presence byte is the \
         only thing that can distinguish it"
    );
    let read = NodeFrameworkState::decode("relay", &bytes).expect("readable");
    assert_eq!(read.next_fire_ns, None, "zero must not decode to Some(0)");
    assert_eq!(read.pending_data_count, 3);

    // The other side of the same pin: a REAL deadline of zero round-trips as
    // `Some(0)`, so the presence byte is load-bearing in both directions.
    let due_at_origin = NodeFrameworkState {
        next_fire_ns: Some(0),
        ..Default::default()
    };
    let mut sink = VecSink::new();
    due_at_origin.encode(&mut sink).expect("fits");
    let bytes = sink.into_inner();
    assert_eq!(bytes[4], 1);
    assert_eq!(
        NodeFrameworkState::decode("origin", &bytes)
            .expect("readable")
            .next_fire_ns,
        Some(0)
    );
}

#[test]
fn an_empty_section_is_reported_so_a_carrier_can_skip_the_v2_framing() {
    assert!(NodeFrameworkState::default().is_empty());
    for populated in [
        NodeFrameworkState {
            next_fire_ns: Some(0),
            ..Default::default()
        },
        NodeFrameworkState {
            pending_data_count: 1,
            ..Default::default()
        },
        NodeFrameworkState {
            sync_input_timestamps: [("/cam".to_string(), 0u64)].into_iter().collect(),
            ..Default::default()
        },
    ] {
        assert!(
            !populated.is_empty(),
            "each field alone makes the section worth writing: {populated:?}"
        );
    }
}

#[test]
fn a_future_sections_trailing_fields_are_skipped_rather_than_refused() {
    // The additive contract: a newer section is THIS build's highest layout
    // plus more, so this build reads every structure it knows — the version-1
    // prefix AND the service table — and ignores the rest. The
    // section's declared LENGTH is what bounds the skip, not the version.
    let mut sink = VecSink::new();
    populated_v2_section().encode(&mut sink).expect("fits");
    let mut bytes = sink.into_inner();
    bytes[0..4].copy_from_slice(&(FRAMEWORK_SECTION_WRITE_VERSION + 1).to_le_bytes());
    bytes.extend_from_slice(b"a field this build has never heard of");

    let read = NodeFrameworkState::decode("future", &bytes).expect("readable prefix");
    assert_eq!(
        read,
        populated_v2_section(),
        "a newer section still yields every field this build knows, the \
         service table included"
    );
}

#[test]
fn a_v2_section_round_trips_both_cursor_cases_through_the_one_reader() {
    // The service table survives encode/decode with its three-valued
    // reading intact: a served input, a nothing-served input, and an input the
    // table does not name each answer differently through `service_cursor` —
    // the ONE reader every consumer of the table goes through.
    let section = populated_v2_section();
    let mut sink = VecSink::new();
    section.encode(&mut sink).expect("fits");
    let read = NodeFrameworkState::decode("fusion", &sink.into_inner()).expect("well-formed");
    assert_eq!(read, section, "byte round-trip");
    assert_eq!(
        read.service_cursor("cam"),
        InputServiceCursor::Served(0x0055_6677),
        "a stored cursor reads back as the frame identity it named"
    );
    assert_eq!(
        read.service_cursor("lidar"),
        InputServiceCursor::NothingServed,
        "a named-but-unserved input is a positive claim, not silence"
    );
    assert_eq!(
        read.service_cursor("imu"),
        InputServiceCursor::Absent,
        "an input the table does not name gets no claim at all"
    );
}

#[test]
fn a_v1_section_read_by_this_build_states_no_service_table() {
    // The at-risk direction of the version split: every anchor written before
    // the service table existed is a version-1 section, and this build must read
    // one as saying NOTHING about service cursors — `None`, distinguishable
    // from a version-2 section positively stating an empty table. Collapsing
    // the two would make every earlier bag silently claim its consumers
    // had no FIFO inputs, which routes them to the wrong reader rule.
    //
    // `populated_section()` IS the v1 writer: the write version is
    // content-dependent, so a section with no table stamps version 1 —
    // byte-for-byte what the earlier build wrote (the pinned-bytes arm
    // above holds those exact bytes).
    let mut sink = VecSink::new();
    populated_section().encode(&mut sink).expect("fits");
    let bytes = sink.into_inner();
    assert_eq!(
        u32::from_le_bytes(bytes[0..4].try_into().expect("4 bytes")),
        1,
        "a section with no table is WRITTEN as version 1"
    );
    let read = NodeFrameworkState::decode("legacy", &bytes).expect("v1 stays readable");
    assert_eq!(
        read.input_service, None,
        "a v1 section decodes with the table ABSENT"
    );
    assert_eq!(read.service_cursor("inp"), InputServiceCursor::Absent);

    // The distinguishable twin: a section that STATES an empty table is a
    // version-2 section and decodes to `Some({})` — the positive claim "none
    // of my inputs is a per-message FIFO trigger".
    let stated_empty = NodeFrameworkState {
        input_service: Some(std::collections::BTreeMap::new()),
        ..populated_section()
    };
    let mut sink = VecSink::new();
    stated_empty.encode(&mut sink).expect("fits");
    let bytes = sink.into_inner();
    assert_eq!(
        u32::from_le_bytes(bytes[0..4].try_into().expect("4 bytes")),
        FRAMEWORK_SECTION_WRITE_VERSION,
        "a stated table — even an empty one — is a version-2 section"
    );
    let read = NodeFrameworkState::decode("modern", &bytes).expect("well-formed");
    assert_eq!(
        read.input_service,
        Some(std::collections::BTreeMap::new()),
        "the empty claim survives as an empty claim, never as silence"
    );
}

#[test]
fn a_table_of_nothing_leaves_the_section_empty_but_a_served_cursor_does_not() {
    // `is_empty` gates whether an anchor writes a section AT ALL, so it is the
    // seam that keeps every idle data-trigger node on the earlier v1
    // header. A table whose every cursor is `None` states nothing a reader
    // can act on and must not force a section into every anchor in the repo;
    // ONE served cursor is a real claim and must.
    let all_none = NodeFrameworkState {
        input_service: Some([("inp".to_string(), None)].into_iter().collect()),
        ..NodeFrameworkState::default()
    };
    assert!(
        all_none.is_empty(),
        "an all-None table alone does not make the section worth writing"
    );
    let served = NodeFrameworkState {
        input_service: Some([("inp".to_string(), Some(0u32))].into_iter().collect()),
        ..NodeFrameworkState::default()
    };
    assert!(
        !served.is_empty(),
        "a served cursor — sequence 0 included — is a claim the section must carry"
    );
}

#[test]
fn a_section_this_build_has_no_layout_for_is_refused_by_name() {
    // A version BELOW the first one that ever existed is not an older format —
    // it is bytes with no layout, and reading `next_fire_ns` out of them would
    // restore a deadline from something else entirely.
    let mut sink = VecSink::new();
    populated_section().encode(&mut sink).expect("fits");
    let mut bytes = sink.into_inner();
    bytes[0..4].copy_from_slice(&0u32.to_le_bytes());

    let err = NodeFrameworkState::decode("planner", &bytes).expect_err("no layout");
    match &err {
        RestoreRefusal::MalformedFrameworkSection { node, reason } => {
            assert_eq!(node, "planner");
            assert!(reason.contains("version 0"), "names the version: {reason}");
        }
        other => panic!("expected MalformedFrameworkSection, got {other:?}"),
    }
    let text = err.to_string();
    assert!(text.contains("planner"), "names the node: {text}");
    assert!(
        text.contains("re-record") || text.contains("build the recording was made with"),
        "states the fix: {text}"
    );
}

#[test]
fn a_truncated_section_is_refused_rather_than_read_short() {
    // Three truncations, each cutting a different structure, because a decoder
    // that bounds only its fixed prefix reads the variable table off the end.
    let mut sink = VecSink::new();
    populated_section().encode(&mut sink).expect("fits");
    let full = sink.into_inner();

    for cut in [0usize, 12, 24, 26, 30, full.len() - 1] {
        let err = NodeFrameworkState::decode("slam", &full[..cut])
            .expect_err("a truncated section must refuse");
        assert!(
            matches!(err, RestoreRefusal::MalformedFrameworkSection { .. }),
            "cut at {cut} gave {err:?}"
        );
    }
    // Anti-tautology: the UNCUT bytes are readable, so the loop above is not
    // refusing a section that was malformed to begin with.
    assert_eq!(
        NodeFrameworkState::decode("slam", &full).expect("readable"),
        populated_section()
    );
}

#[test]
fn a_section_declaring_more_bytes_than_the_anchor_holds_is_refused() {
    // The section length is UNTRUSTED — it comes off a bag — so an
    // over-declared one must refuse rather than slice past the payload.
    let value: u32 = 7;
    let mut framed = VecSink::new();
    capture_anchor_blob_with_framework(&value, &populated_section(), &mut framed)
        .expect("capture fits");
    let mut framed = framed.into_inner();
    let real_len = u32::from_le_bytes(framed[16..20].try_into().unwrap());
    framed[16..20].copy_from_slice(&(real_len + 1_000).to_le_bytes());

    let err = AnchorBlob::decode("greedy", &framed).expect_err("over-declared");
    match &err {
        RestoreRefusal::MalformedFrameworkSection { node, reason } => {
            assert_eq!(node, "greedy");
            assert!(
                reason.contains(&(real_len + 1_000).to_string()),
                "names the declared length: {reason}"
            );
        }
        other => panic!("expected MalformedFrameworkSection, got {other:?}"),
    }

    // A v2 blob too short to carry its own length word is a MALFORMED SECTION,
    // not an unframed anchor: its magic was recognised, so the blob IS framed
    // and what is truncated is the section's framing. Reporting it as unframed
    // would send the reader after the wrong thing.
    let stub = ANCHOR_BLOB_MAGIC_V2
        .to_le_bytes()
        .iter()
        .chain(0u64.to_le_bytes().iter())
        .copied()
        .collect::<Vec<u8>>();
    assert_eq!(stub.len(), ANCHOR_BLOB_V2_HEADER_SIZE - 4);
    match AnchorBlob::decode("stub", &stub) {
        Err(RestoreRefusal::MalformedFrameworkSection { node, reason }) => {
            assert_eq!(node, "stub");
            assert!(reason.contains("too short"), "{reason}");
        }
        other => panic!("expected MalformedFrameworkSection, got {other:?}"),
    }
}

#[test]
fn the_declared_section_length_is_pinned_across_its_whole_boundary_set() {
    // A v2 blob's `section_len` is UNTRUSTED — it comes off a bag — and the
    // reader's behaviour is pinned at every point of its range in ONE body, so
    // no arm of the boundary can be moved without a test noticing.
    //
    // The floor is DERIVED from the writer rather than written down: a section
    // always carries at least its own version header, so the smallest thing any
    // capture can emit is an all-default section's length. Below it, every
    // value is unreachable from a legitimate v2 writer.
    let floor = NodeFrameworkState::default().encoded_len();
    assert_eq!(
        floor, 25,
        "the v1 prefix is version(4) + presence(1) + next_fire(8) + pending(8) + sync count(4)"
    );

    let framed = |section_len: u32, section: &[u8], payload: &[u8]| -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&ANCHOR_BLOB_MAGIC_V2.to_le_bytes());
        out.extend_from_slice(&0xABCDu64.to_le_bytes());
        out.extend_from_slice(&section_len.to_le_bytes());
        out.extend_from_slice(section);
        out.extend_from_slice(payload);
        out
    };
    let mut sink = VecSink::new();
    NodeFrameworkState::default()
        .encode(&mut sink)
        .expect("fits");
    let default_section = sink.into_inner();
    assert_eq!(default_section.len(), floor);

    // (a) ZERO — the value that would otherwise read as an ABSENT section.
    //     A v1 blob signals absence with an empty framework slice, so a
    //     `section_len` of 0 arrives at `framework_state` indistinguishable
    //     from one, takes the absent arm, and the resume falls silently back to
    //     trace-derived scheduling — discarding a claim the recording DID make
    //     and then reporting the divergence against the candidate.
    let zero = framed(0, &[], b"payload");
    match AnchorBlob::decode("zero", &zero) {
        Err(RestoreRefusal::MalformedFrameworkSection { node, reason }) => {
            assert_eq!(node, "zero");
            assert!(
                reason.contains("0-byte") && reason.contains(&floor.to_string()),
                "names the declared length AND the floor: {reason}"
            );
        }
        other => panic!("a zero-length section must refuse, got {other:?}"),
    }

    // (b) EVERY value between zero and the floor — the same rule, so a reader
    //     that special-cased only zero is caught here.
    for section_len in 1..floor {
        let blob = framed(section_len as u32, &vec![0u8; section_len], b"payload");
        assert!(
            matches!(
                AnchorBlob::decode("short", &blob),
                Err(RestoreRefusal::MalformedFrameworkSection { .. })
            ),
            "a {section_len}-byte section must refuse — no capture can write one"
        );
    }

    // (c) EXACTLY the floor — the smallest LEGITIMATE section, an all-default
    //     one. It decodes, and it decodes to a POSITIVE statement (this node
    //     had no deadline, no pending arrivals, no alignment), which is a
    //     different fact from a v1 blob's silence.
    let at_floor = framed(floor as u32, &default_section, b"payload");
    let blob = AnchorBlob::decode("floor", &at_floor).expect("the floor is legitimate");
    assert_eq!(blob.framework.len(), floor);
    assert_eq!(blob.payload, b"payload");
    assert_eq!(
        blob.framework_state("floor").expect("readable"),
        Some(NodeFrameworkState::default()),
        "the smallest legitimate section is a STATEMENT, not an absence"
    );

    // (d) A length that FITS the anchor but cuts the section's own sync table.
    //     The framing accepts it — it is within the blob — and the section
    //     decoder refuses it, so the two halves of the bound compose.
    let mut sink = VecSink::new();
    populated_section().encode(&mut sink).expect("fits");
    let populated = sink.into_inner();
    assert!(
        populated.len() > floor,
        "the fixture has a sync table to cut"
    );
    let cut = framed(floor as u32, &populated, b"payload");
    assert!(
        matches!(
            AnchorBlob::decode("cut", &cut).and_then(|b| b.framework_state("cut")),
            Err(RestoreRefusal::MalformedFrameworkSection { .. })
        ),
        "a section whose declared length cuts its own table is refused"
    );

    // (e) A length BEYOND the anchor — pinned in full by
    //     `a_section_declaring_more_bytes_than_the_anchor_holds_is_refused`,
    //     asserted here too so the range is covered end to end in one place.
    let over = framed((floor + 1_000) as u32, &default_section, b"payload");
    assert!(matches!(
        AnchorBlob::decode("over", &over),
        Err(RestoreRefusal::MalformedFrameworkSection { .. })
    ));
}

#[test]
fn no_capture_can_write_a_section_below_the_readers_floor() {
    // The other half of (a)-(b) above: those arms assert the READER refuses a
    // range, and this asserts the WRITER cannot produce it — so the refusal
    // costs no legitimate recording anything. Driven over every shape a
    // section can take rather than over the default alone.
    let floor = NodeFrameworkState::default().encoded_len();
    for section in [
        NodeFrameworkState::default(),
        NodeFrameworkState {
            next_fire_ns: Some(0),
            ..Default::default()
        },
        NodeFrameworkState {
            pending_data_count: u64::MAX,
            ..Default::default()
        },
        NodeFrameworkState {
            sync_input_timestamps: [(String::new(), 0u64)].into_iter().collect(),
            ..Default::default()
        },
        populated_section(),
    ] {
        let mut sink = VecSink::new();
        AnchorBlob::write_header_with_framework(0, &section, &mut sink).expect("fits");
        let bytes = sink.into_inner();
        let declared = u32::from_le_bytes(bytes[16..20].try_into().unwrap()) as usize;
        assert_eq!(
            declared,
            section.encoded_len(),
            "the framing declares what the encoder writes: {section:?}"
        );
        assert!(
            declared >= floor,
            "a capture can never declare below the reader's floor: {section:?} -> {declared}"
        );
    }
}

#[test]
fn a_section_naming_one_input_twice_is_refused_rather_than_silently_deduped() {
    // A `BTreeMap` cannot represent the duplicate, so decoding one would
    // silently keep whichever timestamp came last — a recording stating two
    // arrival instants for one input is corrupt, and saying so beats picking.
    let mut bytes: Vec<u8> = Vec::new();
    bytes.extend_from_slice(&FRAMEWORK_SECTION_MIN_READABLE.to_le_bytes());
    bytes.push(0);
    bytes.extend_from_slice(&0u64.to_le_bytes());
    bytes.extend_from_slice(&0u64.to_le_bytes());
    bytes.extend_from_slice(&2u32.to_le_bytes());
    for ts in [11u64, 22u64] {
        bytes.extend_from_slice(&4u32.to_le_bytes());
        bytes.extend_from_slice(b"/cam");
        bytes.extend_from_slice(&ts.to_le_bytes());
    }

    let err = NodeFrameworkState::decode("fusion", &bytes).expect_err("duplicate input");
    match err {
        RestoreRefusal::MalformedFrameworkSection { reason, .. } => {
            assert!(reason.contains("/cam"), "names the input: {reason}");
        }
        other => panic!("expected MalformedFrameworkSection, got {other:?}"),
    }
}

#[test]
fn a_sync_input_name_that_is_not_utf8_is_refused() {
    let mut bytes: Vec<u8> = Vec::new();
    bytes.extend_from_slice(&FRAMEWORK_SECTION_MIN_READABLE.to_le_bytes());
    bytes.push(0);
    bytes.extend_from_slice(&0u64.to_le_bytes());
    bytes.extend_from_slice(&0u64.to_le_bytes());
    bytes.extend_from_slice(&1u32.to_le_bytes());
    bytes.extend_from_slice(&2u32.to_le_bytes());
    bytes.extend_from_slice(&[0xFF, 0xFE]);
    bytes.extend_from_slice(&5u64.to_le_bytes());

    assert!(matches!(
        NodeFrameworkState::decode("fusion", &bytes),
        Err(RestoreRefusal::MalformedFrameworkSection { .. })
    ));
}

// ===========================================================================
// plan_restore — the rendezvous
// ===========================================================================

#[test]
fn a_recording_that_begins_at_step_zero_restores_nothing() {
    // The step-0 free anchor. Facts are deliberately PRESENT and
    // deliberately incomplete: neither may change the answer, because a
    // from-start bag replays exactly as it always has.
    let facts = vec![torn(RUN, 0, "slam")];
    let plan = plan_restore(&RestoreRequest {
        run_id: RUN,
        required_nodes: &names(&["slam", "planner"]),
        first_recorded_step: 0,
        facts: &facts,
    })
    .expect("step 0 never refuses");
    assert_eq!(plan, RestorePlan::FromStart);
}

#[test]
fn a_complete_anchor_one_step_below_the_first_boundary_is_the_resume_point() {
    let facts = vec![
        complete(RUN, 100, "slam"),
        complete(RUN, 100, "planner"),
        complete(RUN, 100, "unselected"),
    ];
    let plan = plan_restore(&RestoreRequest {
        run_id: RUN,
        required_nodes: &names(&["slam", "planner"]),
        first_recorded_step: 101,
        facts: &facts,
    })
    .expect("complete");

    let anchor = match plan {
        RestorePlan::FromAnchor(a) => a,
        other => panic!("expected FromAnchor, got {other:?}"),
    };
    assert_eq!(anchor.step, 100, "S = first_recorded_step - 1");
    assert_eq!(anchor.run_id, RUN);
    assert_eq!(
        anchor.first_replay_step(),
        101,
        "checkpoint@S == trace-gate@S+1"
    );
}

#[test]
fn an_anchor_at_any_other_step_cannot_satisfy_the_rendezvous() {
    // The neighbours on BOTH sides, so the rule cannot be satisfied by a
    // one-sided comparison. A restore that accepted step 99 would execute step
    // 100 with no recorded boundary for it; one that accepted 101 would
    // silently discard a recorded step.
    for wrong_step in [99u64, 101] {
        let facts = vec![complete(RUN, wrong_step, "slam")];
        let err = plan_restore(&RestoreRequest {
            run_id: RUN,
            required_nodes: &names(&["slam"]),
            first_recorded_step: 101,
            facts: &facts,
        })
        .expect_err("wrong step");
        match err {
            RestoreRefusal::AnchorIncomplete {
                anchor_step,
                problems,
                complete_nodes,
            } => {
                assert_eq!(anchor_step, 100);
                assert_eq!(problems, vec![("slam".into(), NodeAnchorProblem::Missing)]);
                assert_eq!(complete_nodes, 0);
            }
            other => panic!("expected AnchorIncomplete, got {other:?}"),
        }
    }
}

#[test]
fn another_runs_anchor_at_the_same_step_does_not_count() {
    // A machine-wide `bag record` carries every live run's anchors on one
    // channel, and the steps of two runs are unrelated numbers.
    let facts = vec![complete(OTHER_RUN, 100, "slam")];
    let err = plan_restore(&RestoreRequest {
        run_id: RUN,
        required_nodes: &names(&["slam"]),
        first_recorded_step: 101,
        facts: &facts,
    })
    .expect_err("wrong run");
    assert!(matches!(err, RestoreRefusal::AnchorIncomplete { .. }));
}

#[test]
fn a_torn_or_skipped_anchor_voids_the_resume_and_keeps_its_cause() {
    // Decision: a capture failure voids the anchor. The cause survives into
    // the refusal because "the carrier declined because a lock was held" and
    // "a ring record was lost" have different remedies.
    let facts = vec![
        complete(RUN, 100, "slam"),
        torn(RUN, 100, "planner"),
        skipped(RUN, 100, "costmap", SkipCause::Contended),
    ];
    let err = plan_restore(&RestoreRequest {
        run_id: RUN,
        required_nodes: &names(&["slam", "planner", "costmap"]),
        first_recorded_step: 101,
        facts: &facts,
    })
    .expect_err("voided");

    match err {
        RestoreRefusal::AnchorIncomplete {
            anchor_step,
            problems,
            complete_nodes,
        } => {
            assert_eq!(anchor_step, 100);
            assert_eq!(complete_nodes, 1, "slam did have an anchor");
            assert_eq!(
                problems,
                vec![
                    ("planner".to_string(), NodeAnchorProblem::Torn),
                    (
                        "costmap".to_string(),
                        NodeAnchorProblem::Skipped(SkipCause::Contended)
                    ),
                ],
                "every offender, in the order the replay declares its nodes"
            );
        }
        other => panic!("expected AnchorIncomplete, got {other:?}"),
    }
}

#[test]
fn a_node_the_replay_does_not_execute_cannot_veto_the_anchor() {
    // The narrowing half of that decision: all-or-nothing scopes to what the anchor is
    // asked to COVER. A giant with no carrier yet narrows which selections an
    // anchor serves; it does not void one it is not part of.
    let facts = vec![
        complete(RUN, 100, "slam"),
        torn(RUN, 100, "giant"),
        skipped(RUN, 100, "bridge", SkipCause::StillEncoding),
    ];
    let plan = plan_restore(&RestoreRequest {
        run_id: RUN,
        required_nodes: &names(&["slam"]),
        first_recorded_step: 101,
        facts: &facts,
    })
    .expect("the selection is covered");
    assert_eq!(
        plan,
        RestorePlan::FromAnchor(cerulion_core::state_restore::ResumeAnchor {
            run_id: RUN,
            step: 100
        })
    );
}

#[test]
fn a_bag_with_no_anchors_at_all_names_every_node_it_cannot_cover() {
    let err = plan_restore(&RestoreRequest {
        run_id: RUN,
        required_nodes: &names(&["a", "b", "c"]),
        first_recorded_step: 7,
        facts: &[],
    })
    .expect_err("no anchors");

    match &err {
        RestoreRefusal::AnchorIncomplete {
            anchor_step,
            problems,
            complete_nodes,
        } => {
            assert_eq!(*anchor_step, 6);
            assert_eq!(*complete_nodes, 0);
            assert_eq!(problems.len(), 3, "all three, in one error");
        }
        other => panic!("expected AnchorIncomplete, got {other:?}"),
    }

    let text = err.to_string();
    assert!(text.contains("3 of the 3 node(s)"), "true ratio: {text}");
    assert!(
        text.contains("graph run --record"),
        "names the from-step-0 fix: {text}"
    );
    // The OTHER way to reach zero anchors, which the
    // from-step-0 fix cannot address. A mid-run attach that DECLINED the run's
    // node-state rings (a Flashback recorder was already draining them) records
    // none at all, so "re-record with a recorder attached long enough" is
    // impossible advice for that bag — it names `cerulion bag info` instead,
    // which is where the decision is printed.
    assert!(
        text.contains("cerulion bag info") && text.contains("DECLINED"),
        "names the declined-attach case and where to read it: {text}"
    );
    // The hedge is load-bearing: the attach records what the run REPORTED at
    // launch, and nothing un-declares that word if the recorder later stopped —
    // so this must not promise a capture exists.
    assert!(
        text.contains("IF that recorder captured any"),
        "…without asserting a capture nobody observed: {text}"
    );
}

#[test]
fn the_first_replay_step_saturates_rather_than_wrapping() {
    let anchor = cerulion_core::state_restore::ResumeAnchor {
        run_id: RUN,
        step: u64::MAX,
    };
    assert_eq!(anchor.first_replay_step(), u64::MAX);
}

// ===========================================================================
// classify_shape
// ===========================================================================

#[test]
fn an_identical_shape_restores_and_a_different_one_is_terminal() {
    classify_shape("slam", 0x1234, 0x1234).expect("identical shapes restore");

    let err = classify_shape("slam", 0x1234, 0x5678).expect_err("drift");
    assert_eq!(
        err,
        RestoreRefusal::ShapeDrift {
            node: "slam".to_string(),
            recorded: 0x1234,
            current: 0x5678,
        }
    );
    let text = err.to_string();
    assert!(
        text.contains("0x0000000000001234") && text.contains("0x0000000000005678"),
        "BOTH hashes are printed so the reader can tell which side moved: {text}"
    );
    assert!(text.contains("re-record"), "names a fix: {text}");
}

/// The shape refusal states its SCOPE and its TOLERANCE TIER.
///
/// # Why these two sentences are load-bearing rather than prose
///
/// Both were true of the landed code and stated NOWHERE a user could see them.
///
/// SCOPE: an operator reading "this anchor does not match" reasonably tries an
/// EARLIER anchor from the same recording. Every one of them will refuse
/// identically — the shape is a property of the TYPE, not of the moment — so
/// without this sentence the refusal invites a search that cannot succeed, on a
/// bag that may hold hours of anchors.
///
/// TOLERANCE: a reader may expect a per-field, name-keyed drift table
/// where a reorder is fine and an added field defaults. The code does not build
/// that — the identity is ONE hash over the field list and `cer_capture` writes
/// POSITIONALLY — so the reachable table has two rows and the behaviour
/// is STRICTER than that expectation in both directions. An operator who
/// hits this message would otherwise conclude their reorder
/// should have been tolerated and go looking for a bug.
///
/// Byte-pinned: the message is the ONLY place either fact reaches a user (the
/// analysis lives in `classify_shape`'s doc comment, which no operator reads),
/// so a rewrite that drops one silently loses it.
#[test]
fn the_shape_refusal_states_its_scope_and_its_tolerance_tier() {
    let text = classify_shape("slam", 0x1234, 0x5678)
        .expect_err("drift")
        .to_string();

    // SCOPE — the claim, and the reason it holds.
    assert!(
        text.contains("voids EVERY anchor for this node in the recording"),
        "the refusal must say the edit voids every anchor for this node, not just this one — \
         otherwise it invites a search through the bag that cannot succeed: {text}"
    );
    assert!(
        text.contains("no earlier anchor is any more applicable"),
        "…and say WHY, or the scope claim reads as an arbitrary rule: {text}"
    );

    // TOLERANCE TIER — the disclosure, and the two rows an operator most needs.
    assert!(
        text.contains("TOLERANCE: there is none"),
        "the refusal must disclose that NO drift is tolerated: {text}"
    );
    assert!(
        text.contains("REORDERED") && text.contains("a reorder is refused"),
        "a REORDER is the edit a reader most expects to be tolerated, so the message must name \
         it explicitly: {text}"
    );
    assert!(
        text.contains("added field is refused rather than defaulted"),
        "…and so is an ADDED field, which a name-keyed table would default: {text}"
    );
    assert!(
        text.contains("POSITIONALLY"),
        "the reason the tier is what it is — positional encoding under one whole-type hash — is \
         what makes the rule predictable rather than arbitrary: {text}"
    );
}

// ===========================================================================
// The lossy-bag refusal
// ===========================================================================

fn loss(topic: &str, frames_lost: u64, prefix_lost: u64) -> TopicLoss {
    TopicLoss {
        topic: topic.to_string(),
        frames_lost,
        prefix_lost,
    }
}

#[test]
fn a_lossy_re_executed_topic_refuses_on_either_loss_term() {
    // Both terms independently, because they come from DIFFERENT attachments
    // (`record_health.json` and `record_coverage.json`) and a reader that
    // wired only one would look correct against the other's fixture.
    for l in [
        loss("/scan", 3, 0),
        loss("/scan", 0, 5),
        loss("/scan", 2, 2),
    ] {
        let err = refuse_lossy_reexecuted_topics(&names(&["/scan"]), std::slice::from_ref(&l))
            .expect_err("lossy");
        match err {
            RestoreRefusal::LossyTopics { topics } => assert_eq!(topics, vec![l.clone()]),
            other => panic!("expected LossyTopics, got {other:?}"),
        }
    }
}

#[test]
fn a_clean_re_executed_topic_and_a_lossy_injected_one_both_pass() {
    // The injected half is the load-bearing one: an injected topic is replayed
    // verbatim with its own recorded sequences, so refusing over a lossy
    // bystander would make the feature unusable on any real machine-wide bag.
    refuse_lossy_reexecuted_topics(
        &names(&["/cmd_vel"]),
        &[loss("/cmd_vel", 0, 0), loss("/noisy_bystander", 900, 12)],
    )
    .expect("only re-executed topics are judged");
}

#[test]
fn every_lossy_re_executed_topic_is_named_in_one_refusal() {
    let err = refuse_lossy_reexecuted_topics(
        &names(&["/a", "/b", "/c"]),
        &[loss("/a", 1, 0), loss("/b", 0, 0), loss("/c", 0, 4)],
    )
    .expect_err("lossy");
    let text = err.to_string();
    assert!(text.contains("/a") && text.contains("/c"), "{text}");
    assert!(
        !text.contains("/b"),
        "the clean topic is not accused: {text}"
    );
    assert!(text.contains("2 re-executed topic(s)"), "{text}");
}

// ===========================================================================
// The backlog frame gate
// ===========================================================================

fn claim(topic: &str, consumer: &str, pending: u32) -> BacklogClaim {
    BacklogClaim {
        topic: topic.to_string(),
        consumer: consumer.to_string(),
        pending,
    }
}

fn have(topic: &str, n: u32) -> TopicFrameCount {
    TopicFrameCount {
        topic: topic.to_string(),
        frames_at_or_before_anchor: n,
    }
}

#[test]
fn a_graph_with_no_undrained_backlog_admits_nothing_before_the_anchor() {
    // The plain rendezvous rule stays the answer for every fully-drained topic.
    let admission = plan_backlog_admission(&[], &[have("/scan", 12)]).expect("no claims");
    assert!(admission.is_empty());
    assert_eq!(admission.admitted("/scan"), 0);
}

#[test]
fn a_captured_backlog_admits_exactly_that_many_pre_anchor_frames() {
    let admission = plan_backlog_admission(
        &[claim("/scan", "slam", 3), claim("/odom", "slam", 1)],
        &[have("/scan", 40), have("/odom", 9)],
    )
    .expect("covered");

    assert_eq!(admission.admitted("/scan"), 3);
    assert_eq!(admission.admitted("/odom"), 1);
    assert_eq!(admission.per_topic.len(), 2, "no phantom entries");
}

#[test]
fn a_zero_backlog_is_absent_from_the_admission_rather_than_an_entry_of_zero() {
    // A consumer that WAS fully drained is the majority case; recording it as
    // an explicit zero would make every topic look like it needs a decision.
    let admission = plan_backlog_admission(&[claim("/scan", "slam", 0)], &[have("/scan", 40)])
        .expect("drained");
    assert!(admission.is_empty());
    assert_eq!(admission.admitted("/scan"), 0);
}

#[test]
fn two_consumers_that_agree_produce_one_admission() {
    let admission = plan_backlog_admission(
        &[claim("/scan", "slam", 2), claim("/scan", "costmap", 2)],
        &[have("/scan", 5)],
    )
    .expect("agreement");
    assert_eq!(admission.admitted("/scan"), 2);
}

#[test]
fn two_consumers_that_disagree_are_refused_rather_than_served_the_maximum() {
    // THE backlog-gate pin. Serving 5 over-feeds `slam` by two frames; serving
    // 3 starves `costmap` by two. Either diverges a node while reporting
    // success, so the disagreement is terminal and both claims are named.
    let err = plan_backlog_admission(
        &[claim("/scan", "slam", 3), claim("/scan", "costmap", 5)],
        &[have("/scan", 40)],
    )
    .expect_err("disagreement");

    match &err {
        RestoreRefusal::BacklogDisagreement { topic, claims } => {
            assert_eq!(topic, "/scan");
            assert_eq!(
                claims,
                &vec![("costmap".to_string(), 5u32), ("slam".to_string(), 3u32)],
                "sorted, so the diagnostic does not depend on capture order"
            );
        }
        other => panic!("expected BacklogDisagreement, got {other:?}"),
    }
    let text = err.to_string();
    assert!(
        text.contains("slam=3") && text.contains("costmap=5"),
        "{text}"
    );
}

#[test]
fn a_recording_that_cannot_cover_the_captured_backlog_is_refused_with_the_shortfall() {
    let err = plan_backlog_admission(&[claim("/scan", "slam", 5)], &[have("/scan", 2)])
        .expect_err("short");
    assert_eq!(
        err,
        RestoreRefusal::BacklogShortfall {
            topic: "/scan".to_string(),
            claimed: 5,
            available: 2,
        }
    );
    assert!(err.to_string().contains("3 frame(s) short"), "{err}");
}

#[test]
fn a_topic_absent_from_the_recording_reads_as_zero_available_not_as_unbounded() {
    // The fail-CLOSED direction: an unknown topic must not silently satisfy a
    // backlog claim.
    let err = plan_backlog_admission(&[claim("/ghost", "slam", 1)], &[]).expect_err("absent");
    assert_eq!(
        err,
        RestoreRefusal::BacklogShortfall {
            topic: "/ghost".to_string(),
            claimed: 1,
            available: 0,
        }
    );
}

#[test]
fn the_shortfall_boundary_is_pinned_on_both_sides() {
    // Exactly enough is enough; one fewer is not.
    plan_backlog_admission(&[claim("/scan", "slam", 4)], &[have("/scan", 4)]).expect("exact");
    assert!(matches!(
        plan_backlog_admission(&[claim("/scan", "slam", 4)], &[have("/scan", 3)]),
        Err(RestoreRefusal::BacklogShortfall { .. })
    ));
}

// ===========================================================================
// The publisher-sequence seed ladder
// ===========================================================================

#[test]
fn the_ladder_prefers_the_last_recorded_frame_before_the_anchor() {
    // All three rungs present: the recorded predecessor wins, and the other
    // two values are chosen so that either of them answering is visible.
    let seed = seed_publisher_sequence(
        "/scan",
        TopicWriters::SingleWriter,
        &SeedEvidence {
            last_at_or_before_anchor: Some(7),
            first_after_anchor: Some(42),
            captured_last_commit: Some(100),
        },
    )
    .expect("rung i");
    assert_eq!(seed.seed, 8, "last + 1");
    assert_eq!(seed.rung, SeedRung::LastFrameBeforeAnchor);
    assert_eq!(seed.topic, "/scan");
}

#[test]
fn a_window_bag_with_no_pre_anchor_frame_seeds_from_the_first_frame_after_it_verbatim() {
    // THE seed-ladder pin. The rule "last recorded sequence before S, plus 1" is
    // undefined here, and the `prefix_lost` refusal is structurally silent on
    // exactly this shape. The first post-anchor frame IS the next one the
    // re-executed publisher emits, so its sequence is the seed — NOT that
    // sequence plus one, which is the off-by-one this rung exists to avoid.
    let seed = seed_publisher_sequence(
        "/scan",
        TopicWriters::SingleWriter,
        &SeedEvidence {
            last_at_or_before_anchor: None,
            first_after_anchor: Some(42),
            captured_last_commit: Some(100),
        },
    )
    .expect("rung ii");
    assert_eq!(seed.seed, 42, "verbatim, not 43");
    assert_eq!(seed.rung, SeedRung::FirstFrameAfterAnchor);
}

#[test]
fn a_topic_the_recording_never_carried_falls_to_the_captured_commit_counter() {
    let seed = seed_publisher_sequence(
        "/silent",
        TopicWriters::SingleWriter,
        &SeedEvidence {
            last_at_or_before_anchor: None,
            first_after_anchor: None,
            captured_last_commit: Some(9),
        },
    )
    .expect("rung iii");
    assert_eq!(seed.seed, 10, "captured + 1");
    assert_eq!(seed.rung, SeedRung::CapturedCommit);
}

#[test]
fn a_topic_with_no_evidence_at_all_is_refused_by_name() {
    let err = seed_publisher_sequence(
        "/silent",
        TopicWriters::SingleWriter,
        &SeedEvidence::default(),
    )
    .expect_err("no rung");
    assert_eq!(
        err,
        RestoreRefusal::SequenceSeedUnavailable {
            topic: "/silent".to_string()
        }
    );
    let text = err.to_string();
    assert!(text.contains("/silent"), "{text}");
    assert!(
        text.contains("byte mismatch"),
        "names the consequence an operator would otherwise misread: {text}"
    );
}

#[test]
fn a_multi_publisher_topic_is_refused_rather_than_seeded_from_an_interleaved_stream() {
    // The evidence is deliberately COMPLETE — all three rungs answer — so the
    // refusal cannot be mistaken for "no rung was available". A
    // `multi_publisher_topics:` topic interleaves one counter per publisher
    // and a frame carries no publisher identity, so the stream's "last
    // recorded sequence" is nobody's next sequence.
    let full = SeedEvidence {
        last_at_or_before_anchor: Some(7),
        first_after_anchor: Some(42),
        captured_last_commit: Some(100),
    };

    let err = seed_publisher_sequence("/tf", TopicWriters::MultiPublisher, &full)
        .expect_err("one seed cannot serve two counters");
    assert_eq!(
        err,
        RestoreRefusal::MultiPublisherTopicNotSeedable {
            topic: "/tf".to_string()
        }
    );
    let text = err.to_string();
    assert!(text.contains("/tf"), "names the topic: {text}");
    assert!(
        text.contains("multi_publisher_topics"),
        "names WHY this topic is different: {text}"
    );
    assert!(
        text.contains("injected rather than re-executed"),
        "names the remedy: {text}"
    );

    // The anti-tautology half, in the same body and over the SAME evidence:
    // without it, "MultiPublisher refuses" is satisfied by a ladder that
    // refuses everything.
    let ok = seed_publisher_sequence("/tf", TopicWriters::SingleWriter, &full)
        .expect("one writer is the ordinary shape");
    assert_eq!(ok.seed, 8);
    assert_eq!(ok.rung, SeedRung::LastFrameBeforeAnchor);
}

#[test]
fn the_seed_wraps_with_the_wire_counter_rather_than_saturating() {
    // The wire `sequence` is a `u32` that really wraps; a saturating seed
    // would stall a long run's counter at u32::MAX and mismatch every frame.
    let wrapped = seed_publisher_sequence(
        "/scan",
        TopicWriters::SingleWriter,
        &SeedEvidence {
            last_at_or_before_anchor: Some(u32::MAX),
            ..SeedEvidence::default()
        },
    )
    .expect("rung i");
    assert_eq!(wrapped.seed, 0);

    let wrapped_commit = seed_publisher_sequence(
        "/scan",
        TopicWriters::SingleWriter,
        &SeedEvidence {
            captured_last_commit: Some(u32::MAX),
            ..SeedEvidence::default()
        },
    )
    .expect("rung iii");
    assert_eq!(wrapped_commit.seed, 0);
}

// ===========================================================================
// --strict-state
// ===========================================================================

#[test]
fn strict_state_refuses_only_when_asked_and_only_when_a_node_declares_none() {
    // Default: a stateless node is normal, so an unrestored node is not an
    // error.
    enforce_strict_state(&names(&["ticker"]), false).expect("default tolerates");
    // Strict with full coverage is silent.
    enforce_strict_state(&[], true).expect("nothing missing");

    let err = enforce_strict_state(&names(&["ticker", "relay"]), true).expect_err("strict");
    assert_eq!(
        err,
        RestoreRefusal::StrictStateUnsatisfied {
            nodes: names(&["ticker", "relay"]),
        }
    );
    let text = err.to_string();
    assert!(text.contains("ticker") && text.contains("relay"), "{text}");
    assert!(
        text.contains("CerulionState"),
        "names the derive that fixes it: {text}"
    );
}

// ===========================================================================
// portability (the decision layer must not name a unix-only module)
// ===========================================================================

/// The unix-gated modules of `cerulion_core` (see `lib.rs`). A decision layer
/// that names any of them inherits their gate.
const UNIX_ONLY_MODULES: &[&str] = &["state_ring", "state_arm", "shm_ring", "trace_ring"];

/// `src` with `//` line comments and (nesting) `/* … */` block comments blanked.
///
/// String literals are deliberately NOT modelled, and the direction of that gap
/// is the reason it is acceptable: an unmodelled literal can only make a
/// FORBIDDEN token appear where there is no code, i.e. a false FAILURE, never a
/// false pass. Comments, by contrast, must be stripped — this module's own docs
/// name `state_ring` in prose precisely to explain why it does not depend on it.
fn code_only(src: &str) -> String {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut block_depth = 0usize;
    while i < bytes.len() {
        if block_depth > 0 {
            if bytes[i..].starts_with(b"/*") {
                block_depth += 1;
                out.push_str("  ");
                i += 2;
                continue;
            }
            if bytes[i..].starts_with(b"*/") {
                block_depth -= 1;
                out.push_str("  ");
                i += 2;
                continue;
            }
            out.push(if bytes[i] == b'\n' { '\n' } else { ' ' });
            i += 1;
            continue;
        }
        if bytes[i..].starts_with(b"/*") {
            block_depth = 1;
            out.push_str("  ");
            i += 2;
            continue;
        }
        if bytes[i..].starts_with(b"//") {
            while i < bytes.len() && bytes[i] != b'\n' {
                out.push(' ');
                i += 1;
            }
            continue;
        }
        out.push(bytes[i] as char);
        i += 1;
    }
    out
}

fn core_src(rel: &str) -> String {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(rel);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

#[test]
fn the_restore_decision_layer_names_no_unix_only_module() {
    // The gate this pins is not cosmetic: `graph/runtime.rs` names
    // `crate::state_restore` UNCONDITIONALLY (`restore_node_states`), so a
    // gated module and an ungated caller disagree about which targets they
    // exist on. Measured by configuring the four unix modules out and building
    // the crate: WITH the gate, three extra errors at `graph/runtime.rs`
    // 9075/9076/9098; without it, none.
    //
    // Structural rather than behavioural because a cross-target `cargo check`
    // is not runnable from this repo's dev/CI hosts — the iceoryx2 POSIX PAL's
    // bindgen step needs the target's own C headers and panics without them.
    let raw = core_src("src/state_restore.rs");
    let code = code_only(&raw);

    // Anti-tautology, and the stripper's own proof of work in one assertion:
    // the module DOES discuss `state_ring` in prose, so a stripper that did
    // nothing would fail here rather than making the negative below vacuous.
    assert!(
        raw.contains("state_ring"),
        "the module's prose is expected to mention state_ring (it explains the \
         dependency it does NOT have); if that changed, this guard's \
         anti-tautology needs a new anchor"
    );
    assert!(
        code.contains("pub fn plan_restore"),
        "the stripped view must still hold the module's code"
    );

    for module in UNIX_ONLY_MODULES {
        assert!(
            !code.contains(module),
            "state_restore.rs names the unix-only module `{module}` in CODE, which \
             re-gates the whole restore side on the OS that runs the CAPTURE side. \
             Fix: move the shared item to a portable module (as `SkipCause` was \
             moved to `crate::state`) rather than gating this one"
        );
    }
}

#[test]
fn the_restore_decision_layer_is_declared_without_a_cfg_gate() {
    let lib = code_only(&core_src("src/lib.rs"));
    let decl = "pub mod state_restore;";
    let idx = lib
        .find(decl)
        .unwrap_or_else(|| panic!("lib.rs must declare `{decl}`"));

    // The attribute that would gate it sits on the line(s) immediately above.
    // Read backwards over the stripped view to the previous non-blank line.
    let preceding = lib[..idx]
        .lines()
        .rev()
        .find(|l| !l.trim().is_empty())
        .unwrap_or("")
        .trim()
        .to_string();
    assert!(
        !preceding.starts_with("#[cfg"),
        "`{decl}` is gated by `{preceding}`. The restore DECISION layer must be \
         available wherever a bag is read — `graph/runtime.rs` names it \
         unconditionally, so a gate here is a build failure on any target the \
         gate excludes"
    );
}
