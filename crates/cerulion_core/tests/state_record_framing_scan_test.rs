// SPDX-License-Identifier: AGPL-3.0-only
//! The state RECORD FRAMING literals, swept mechanically.
//!
//! `STATE_RECORD_PAYLOAD` is DERIVED from the record size and the header size
//! (`state_ring.rs`), so every SYMBOLIC use of it re-derived for free when the
//! header grew from 32 bytes to 40 at state record format version 1. What did not
//! re-derive is every place a human wrote the number down: a doc sentence, a
//! framing example, a hand-built oracle array, an `expect` string. A stale one is
//! not a compile error and not a test failure. It is a shipped page that
//! contradicts the code beside it, which is the failure this file exists to make
//! loud.
//!
//! # Why a SOURCE walk rather than an assertion
//!
//! An assertion can only see the numbers the code computes. These are numbers the
//! code does not compute: they are prose. The only thing that can check prose
//! against a constant is a reader, and this file is that reader.
//!
//! # The two halves, and why the control half is the important one
//!
//! The REFUSAL half fails a file carrying a stale framing literal. On its own it
//! is worthless: a scan whose region was mis-spelled, or whose file moved, matches
//! nothing and reads as a pass. So every file also carries a CONTROL, a count of
//! the current framing markers it is expected to hold, and a file that drops out
//! of the scan fails its control rather than silently stopping being scanned.

use std::path::{Path, PathBuf};

/// The repo root, from this crate's manifest directory.
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("crates/<crate>/ sits two levels under the repo root")
        .to_path_buf()
}

/// A file in the scan region: its path, and the minimum number of CURRENT
/// framing markers it must carry.
///
/// The control is a MINIMUM rather than an exact count on purpose: an exact count
/// turns every added sentence into a failing gate, which is how a control gets
/// deleted. A minimum still fails the case it exists for, a file that left the
/// region or lost its framing prose entirely, because that reads zero.
///
/// Each minimum below is the count MEASURED at this tree once `512-byte` left
/// the marker set, so a file that loses one of its framing sentences fails while
/// one that gains a sentence still passes. The previous numbers were floors set
/// well under the counts a `512-byte` marker inflated, which is how a file could
/// have lost most of its framing prose and still cleared them.
struct Scanned {
    path: &'static str,
    min_markers: usize,
}

/// The file SET the framing sweep covered, each with its own control.
///
/// It is a SET rather than one file because a framing sentence lives outside
/// `state_ring.rs` too: `state_carrier/fork.rs` states the payload region in
/// prose, the SHM ring the state plane rides states the record COUNT a 500 MB
/// anchor takes, and a scan of one file reaches none of those nor the ring room
/// precheck's arena term. Adding a file here is how a new framing sentence
/// joins the gate.
const REGION: &[Scanned] = &[
    Scanned {
        path: "crates/cerulion_core/src/state_ring.rs",
        min_markers: 33,
    },
    Scanned {
        path: "crates/cerulion_core/src/state_carrier/fork.rs",
        min_markers: 1,
    },
    // TWO, and where they sit matters more than how many there are. This
    // file's only 40-byte sentence used to be the trace ring's, on a plane this
    // sweep does not cover, so the minimum was satisfied by prose that could
    // stay while every state record sentence in the file went. That is closed,
    // but a count is still a count: it is met by a framing sentence ANYWHERE in
    // the file. The framing is now stated in the doc of the function that
    // DECLARES the state channel as well as beside the descriptor line, the
    // minimum counts both, and
    // `the_writers_state_channel_doc_carries_the_framing_the_control_rests_on`
    // proves the declaring function's doc is load bearing by cutting it out and
    // watching this minimum fail.
    Scanned {
        path: "crates/cerulion_bag/src/writer.rs",
        min_markers: 2,
    },
    Scanned {
        path: "crates/cerulion_bagd/src/anchor_window.rs",
        min_markers: 5,
    },
    // ONE, and the drop from four is the point rather than a weakening. Three
    // of the four this file used to count were trace ring sentences, so a
    // trace-side edit reddened the state framing gate while the state side's
    // single sentence could go unnoticed. Those three now name their own plane
    // and the minimum counts the state record sentence alone.
    Scanned {
        path: "crates/cerulion_bagd/src/lib.rs",
        min_markers: 1,
    },
    Scanned {
        path: "crates/cerulion_bagd/src/state_coverage.rs",
        min_markers: 1,
    },
    Scanned {
        path: "crates/cerulion_core/tests/state_ring_test.rs",
        min_markers: 4,
    },
    // The SHM ring is a GENERIC primitive, but two of its sentences state the
    // record count a 500 MB state anchor takes, which is derived from the state
    // record payload region and moved with the header. Its behavioural test
    // states the same count a third time.
    Scanned {
        path: "crates/cerulion_core/src/shm_ring.rs",
        min_markers: 4,
    },
    Scanned {
        path: "crates/cerulion_core/tests/shm_ring_backpressure_test.rs",
        min_markers: 2,
    },
    // The ring room precheck's hand oracle writes the arena's record count down
    // (64 KiB of arena is 139 records). That number is DERIVED from the payload
    // region, so it moves with the header, and it is a framing number written in
    // a file that carries no other state record prose at all.
    Scanned {
        path: "crates/cerulion_core/src/graph/runtime.rs",
        min_markers: 2,
    },
];

/// A character that continues a WORD.
fn is_word_char(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_'
}

/// A character that continues a NUMBER.
///
/// The decimal point is in this set and not in [`is_word_char`], so `1.09 M`
/// cannot match inside `11.09 M` while `480` still matches at the end of a
/// sentence.
fn is_number_char(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_' || c == '.'
}

/// Is `needle` in `line` with nothing that continues a number in FRONT of it?
///
/// The leading edge is the one that decides whether a marker is the number it
/// claims to be: with it open, `140-byte` and `240-byte` stand in for the
/// header width, and a file that lost every framing sentence keeps a green
/// control on an unrelated size.
///
/// The trailing edge is left open ON PURPOSE, and that is the whole reason this
/// is not [`contains_token`]. `40-byte` and `40-bytes` are the same framing
/// sentence, and a trailing [`is_number_char`] bound stops counting the plural,
/// which trades a silent stand-in for a silent gap rather than closing one.
fn starts_a_token(line: &str, needle: &str, boundary: fn(char) -> bool) -> bool {
    let bytes = line.as_bytes();
    let mut i = 0;
    while let Some(at) = line[i..].find(needle) {
        let s = i + at;
        if s == 0 || !boundary(bytes[s - 1] as char) {
            return true;
        }
        i = s + needle.len();
    }
    false
}

/// Is `needle` in `line` standing on its own, by `boundary`?
///
/// The boundary test is the whole point: it keeps `480` out of `4801`, `1.09 M`
/// out of `11.09 M`, and the frame carve-out below off `framework`.
fn contains_token(line: &str, needle: &str, boundary: fn(char) -> bool) -> bool {
    let bytes = line.as_bytes();
    let mut i = 0;
    while let Some(at) = line[i..].find(needle) {
        let s = i + at;
        let e = s + needle.len();
        let before_ok = s == 0 || !boundary(bytes[s - 1] as char);
        let after_ok = e == bytes.len() || !boundary(bytes[e] as char);
        if before_ok && after_ok {
            return true;
        }
        i = e;
    }
    false
}

/// Does this line SAY it is describing the layout that preceded this one?
///
/// The docs must keep describing format version 0: a reader who meets a refusal
/// naming it needs somewhere to look it up. A line that says so may state ANY of
/// that format's numbers, so this excuses all three detector arms.
fn names_format_version_0(line: &str) -> bool {
    let lower = line.to_ascii_lowercase();
    lower.contains("format version 0") || lower.contains("format_version_0")
}

/// Does this line name the WIRE FRAME plane?
///
/// That plane is a different one with a 32-byte header of its own, which this
/// sweep must not touch. The allowance is WORD BOUNDED: as a substring test it
/// also exempted `framework` and `frameless`, which have nothing to do with the
/// wire frame, so a stale number on a line that happened to say `framework`
/// walked straight through.
fn names_the_wire_frame_plane(line: &str) -> bool {
    let lower = line.to_ascii_lowercase();
    contains_token(&lower, "frame", is_word_char) || contains_token(&lower, "frames", is_word_char)
}

/// Does this line name the TRACE RING plane?
///
/// That plane's record is 40 bytes WHOLE, the same width this format's state
/// record header grew to, and that coincidence is what made the control half
/// vacuous for two files in the region: `cerulion_bag/src/writer.rs` and
/// `cerulion_bagd/src/lib.rs` each cleared their minimum on a sentence about
/// the trace ring, so deleting the state record framing prose beside it left
/// the count unmoved and the sweep reported nothing.
///
/// The carve-out is SCOPED to the header width arm, for the same reason the
/// wire frame one is scoped to the 32-byte arm: the payload region and the
/// derived record counts are numbers only the state plane states, so a
/// sentence that mentions the trace ring while stating one of those is still
/// state record framing prose and still counts.
fn names_the_trace_ring_plane(line: &str) -> bool {
    // A plain substring rather than a word bound, because the plane is named
    // as `TraceRingRecord`, `scheduler_trace` and `trace ring` in turn, and a
    // word bound reaches none of the first two.
    line.to_ascii_lowercase().contains("trace")
}

/// The DERIVED record counts format version 0 stated, retired at version 1.
///
/// A 500 MB anchor was 1_092_267 records at the 480-byte payload region and is
/// 1_110_780 at 472, which `state_ring.rs`'s own `parts_for_len` oracle asserts.
/// Nothing in the compiler notices a derived number: it is arithmetic a human
/// did once and wrote down, so it is exactly the class this file reads for.
const RETIRED_DERIVED_COUNTS: &[&str] = &["1.09 M", "1_092_267", "1092267"];

/// The contiguous `///` doc block immediately above the line that introduces
/// `item`, joined back into one string, or `None` when the item carries none.
///
/// This is the STRUCTURAL reader, and it exists because a count is a weak
/// anchor. A file's minimum is met by a framing sentence ANYWHERE in it, so the
/// doc that actually carries the fact can be reworded away while an unrelated
/// sentence elsewhere holds the count up. A doc block is tied to the item it
/// describes by the language itself, so a control anchored on one fails when
/// the fact leaves the declaration that owns it.
fn doc_block_above(text: &str, item: &str) -> Option<String> {
    let lines: Vec<&str> = text.lines().collect();
    let at = lines
        .iter()
        .position(|l| l.trim_start().starts_with(item))?;
    let mut first = at;
    while first > 0 && lines[first - 1].trim_start().starts_with("///") {
        first -= 1;
    }
    if first == at {
        return None;
    }
    Some(lines[first..at].join("\n"))
}

/// Does this line carry a state record framing number that moved, EXCUSES ASIDE?
fn carries_a_stale_framing_literal(line: &str) -> bool {
    carries_a_stale_header_size(line)
        || carries_a_stale_payload_region(line)
        || carries_a_retired_derived_count(line)
}

/// Arm one: the 32-byte header the state record grew out of.
fn carries_a_stale_header_size(line: &str) -> bool {
    line.contains("32-byte")
}

/// Arm two: a bare 480, the previous payload region, as its own word.
fn carries_a_stale_payload_region(line: &str) -> bool {
    contains_token(line, "480", is_word_char)
}

/// Arm three: a record COUNT derived from that payload region.
fn carries_a_retired_derived_count(line: &str) -> bool {
    RETIRED_DERIVED_COUNTS
        .iter()
        .any(|n| contains_token(line, n, is_number_char))
}

/// Does this line carry a stale framing number that nothing on it EXCUSES?
///
/// The excuses are scoped to the arm each one justifies, which they were not
/// before: the wire frame carve-out is about a 32-byte header on another plane,
/// so it excuses arm one and nothing else. Applied to the whole line it also
/// suppressed a bare 480 and a retired record count whenever the sentence
/// happened to mention a frame, which is a state record number hiding behind an
/// unrelated word. Only the format version 0 excuse, which names this exact
/// layout, reaches all three.
fn carries_an_unexcused_stale_framing_literal(line: &str) -> bool {
    if names_format_version_0(line) {
        return false;
    }
    (carries_a_stale_header_size(line) && !names_the_wire_frame_plane(line))
        || carries_a_stale_payload_region(line)
        || carries_a_retired_derived_count(line)
}

/// The DERIVED record counts this format states, the mirror of
/// [`RETIRED_DERIVED_COUNTS`].
///
/// These are the control half's only reach into two files. `graph/runtime.rs`
/// writes the arena term down as a record count and nothing else about the state
/// framing, and the SHM ring's 500 MB sentences state the count rather than the
/// payload region. Without them those files hold no current marker at all and
/// their control reads zero, which is the vacuity the control exists to catch.
const CURRENT_DERIVED_COUNTS: &[&str] = &["1.11 M", "1_110_780", "139"];

/// Does this line RESERVE everything from `first` upward?
///
/// The claim is made of TWO parts, the range and the word, and this asks for both
/// rather than for one spelling of the sentence that joins them. Every range
/// spelling is bound against a number boundary on its LEADING edge, which is the
/// edge that does the work here: it is what keeps `14+` and `240 and above`,
/// which reserve a range nothing here mints, from being swept up with it. Three
/// of the four are bound on the trailing edge too; the `and above` spelling is
/// bound in front only, because what follows it is a word rather than a digit
/// and a bound there would buy nothing.
///
/// `first` is a PARAMETER because the detector and the control that proves it
/// found the right sentence are the same claim about two different numbers: the
/// range this build mints out of must not be reserved, and the range above it
/// must be. A control built from a literal phrase instead reds a correct tree the
/// moment the doc is reworded, and the reword it has to survive is one the
/// detector's own control list already calls legitimate.
fn reserves_everything_from(line: &str, first: &str) -> bool {
    let lower = line.to_ascii_lowercase();
    names_the_range_from(&lower, first) && lower.contains("reserv")
}

/// Does this lowercased line NAME the range running upward from `first`?
///
/// Three spellings, the last written with and without its space. That is wider
/// than one wording and NARROWER than the claim, which is the honest way to read
/// it: `4 and up`, `4 or higher` and `4 onward` reserve the same range in the
/// same breath and every one of them walks through. The first of those is not
/// merely unread, it is deliberately ABSENT and that is a stated residual rather
/// than an oversight: `state_ring.rs` says "its own doc already reserved 4 and
/// up for" in the PAST tense, recording what the previous format's doc claimed,
/// and a detector that reads that history as a live reservation reds a correct
/// tree. The other two are unread and nothing here forces them to stay so.
fn names_the_range_from(lower: &str, first: &str) -> bool {
    contains_token(lower, &format!("{first}+"), is_number_char)
        || starts_a_token(lower, &format!("{first} and above"), is_number_char)
        || contains_token(lower, &format!(">= {first}"), is_number_char)
        || contains_token(lower, &format!(">={first}"), is_number_char)
}

/// Does this line RESERVE the kind range this build mints out of?
fn reserves_the_range_this_build_mints(line: &str) -> bool {
    reserves_everything_from(line, "4")
}

/// Does this line reserve the range ABOVE the kinds this build mints?
///
/// This is the sentence the doc is SUPPOSED to carry, and it is the positive
/// control for the scan that looks for its opposite.
fn reserves_the_range_above_the_kinds_this_build_mints(line: &str) -> bool {
    reserves_everything_from(line, "7")
}

/// Does this line carry a CURRENT framing marker, for the control half?
///
/// `512-byte` is NOT in this set, and leaving it in was the defect. The record
/// SIZE did not move at format version 1: only the header width and the payload
/// region did. So a file that lost every one of its `40-byte` and `472`
/// sentences, which are the ones this format actually rewrote, still cleared its
/// minimum on the strength of a number that was already there before the format
/// existed. The set now holds only numbers this format MINTED, so a file that
/// drops its framing prose reads zero and fails its control.
///
/// `472` is TOKEN BOUND on a number boundary for the same reason the stale arm
/// is: as a plain substring it matched `1472` and `4720`, so an unrelated number
/// could stand in for the payload region and keep a stripped file's control
/// green.
///
/// `40-byte` is the same class, bounded on its LEADING edge only. Unbounded it
/// matched `140-byte`, letting an unrelated size stand in for the header width;
/// bounded on BOTH edges it would stop counting the plural `40-bytes`, which is
/// the same framing sentence. See [`starts_a_token`].
///
/// `40-byte` is also the one marker another PLANE states, so it carries the
/// trace ring carve-out ([`names_the_trace_ring_plane`]). A 40-byte trace ring
/// record is not state record framing prose, and while it counted, two files
/// in the region cleared their whole minimum on one and could have lost every
/// state record sentence they had without moving their count.
fn carries_a_current_framing_marker(line: &str) -> bool {
    (starts_a_token(line, "40-byte", is_number_char) && !names_the_trace_ring_plane(line))
        || contains_token(line, "472", is_number_char)
        || CURRENT_DERIVED_COUNTS
            .iter()
            .any(|n| contains_token(line, n, is_number_char))
}

/// No file in the framing region carries a state record number that moved, and
/// every one of them still carries the numbers that replaced them.
#[test]
fn the_state_record_framing_literals_are_swept_and_the_scan_is_not_vacuous() {
    let root = repo_root();
    let mut stale: Vec<String> = Vec::new();
    let mut controls: Vec<String> = Vec::new();

    for f in REGION {
        let full = root.join(f.path);
        let text = std::fs::read_to_string(&full)
            .unwrap_or_else(|e| panic!("the scan region must exist: {} ({e})", full.display()));
        let mut markers = 0usize;
        for (no, line) in text.lines().enumerate() {
            if carries_a_current_framing_marker(line) {
                markers += 1;
            }
            if carries_an_unexcused_stale_framing_literal(line) {
                stale.push(format!("{}:{}: {}", f.path, no + 1, line.trim()));
            }
        }
        if markers < f.min_markers {
            controls.push(format!(
                "{}: {markers} current framing markers, expected at least {}",
                f.path, f.min_markers
            ));
        }
    }

    assert!(
        controls.is_empty(),
        "THE CONTROL FAILED, so the refusal below proves nothing: a file in the scan region \
         carries none of the framing numbers it should, which is what a moved or renamed file \
         looks like.\n{}",
        controls.join("\n")
    );
    assert!(
        stale.is_empty(),
        "a state record framing number that moved at format version 1 is still written down. \
         The header is 40 bytes and the payload region is 472; a line that means the PREVIOUS \
         layout must say `format version 0` so a reader can tell the two apart.\n{}",
        stale.join("\n")
    );
}

/// The DETECTOR's own control: every literal this format retired fires, every
/// number that replaced one does not, and the prose carve-outs exempt only the
/// words they name.
///
/// The sweep above is a walk over files, so a detector arm that matches nothing
/// in the region reads there as a pass: the arm that catches a number moving is
/// worthless until something proves it can fire at all. This test is that proof,
/// and it runs in both directions, because an arm that fires on everything is
/// the same kind of useless.
///
/// `1.09 s` and `1.098` are lines that really are elsewhere in this repo and
/// mean something else entirely, so they are the negatives rather than invented
/// ones. This FILE is deliberately outside the scan region: it has to write the
/// retired numbers down to check for them.
#[test]
fn the_stale_literal_detector_fires_on_every_number_this_format_retired() {
    let must_fire = [
        "//! a 500 MB anchor is ~1.09 M records through a fixed ring",
        "/// the wait-free policy silently laps a 1.09 M-record anchor",
        "const FORMAT_VERSION_0_PARTS: u64 = 1_092_267;",
        "// 1092267 records, at the payload region that preceded this one",
        "/// the header is 32-byte and the rest is payload",
        "/// the payload region is 480 bytes",
    ];
    let must_not_fire = [
        "//! a 500 MB anchor is ~1.11 M records through a fixed ring",
        "const PARTS: u64 = 1_110_780;",
        "/// the header is 40-byte and the payload region is 472",
        "/// the binary goes from 1.09 s to 5.08 s, but nothing fails",
        "/// run 4   K=3 1.098  K=8 1.233",
        "/// a 4801 byte blob",
        "/// 11.09 M is a different number",
    ];
    let missed: Vec<&str> = must_fire
        .iter()
        .copied()
        .filter(|l| !carries_a_stale_framing_literal(l))
        .collect();
    assert!(
        missed.is_empty(),
        "the detector does not see a number this format retired, so the sweep \
         above proves nothing about it: {missed:?}"
    );
    let over: Vec<&str> = must_not_fire
        .iter()
        .copied()
        .filter(|l| carries_a_stale_framing_literal(l))
        .collect();
    assert!(
        over.is_empty(),
        "the detector fires on a number that is CURRENT or on an unrelated one, \
         which is how a gate gets deleted: {over:?}"
    );
}

/// The CURRENT marker set holds only numbers this format minted, and it reads
/// each of them as a number rather than as a substring.
///
/// The control half is the one that decides whether the sweep proved anything, so
/// a marker that fires on the wrong thing is worse than a missing one: it keeps a
/// stripped file green. Both halves are here. `512-byte` must NOT count, because
/// the record size did not move at format version 1 and a file may state it
/// without carrying one current framing sentence. `1472` and `4720` must not
/// count either, which is what the number boundary on `472` is for.
///
/// `40-byte` carries BOTH of those halves in one marker, and they pull opposite
/// ways, so each is pinned against the other. `140-byte` must not count, or a
/// slot size stands in for the header width. `40-bytes` must still count, or
/// the plural of the very sentence this marker looks for reads as its absence.
/// That is why the bound is on the leading edge alone.
#[test]
fn the_current_marker_set_holds_only_numbers_this_format_minted() {
    for marker in [
        "/// the header is 40-byte wide",
        "/// the header is 40-bytes wide",
        "/// the payload region is 472 bytes",
        "/// a 500 MB anchor is ~1.11 M records",
        "const PARTS: u64 = 1_110_780;",
        "// 64 KiB of arena is 139 records",
    ] {
        assert!(
            carries_a_current_framing_marker(marker),
            "a number this format minted must count toward a file's control: {marker}"
        );
    }
    for not_a_marker in [
        "// every message on it is exactly one 512-byte record",
        "/// the slot is 140-byte wide",
        "/// a 240-byte slot on another plane",
        "/// the slot is 1472 bytes wide",
        "/// 4720 records fit",
        "/// a 500 MB anchor is ~1.09 M records",
        "/// the header is 32-byte and the payload region is 480",
        "/// 1390 records, which is a different number",
    ] {
        assert!(
            !carries_a_current_framing_marker(not_a_marker),
            "a number this format did NOT mint must not stand in for one, or a file \
             that lost its framing prose keeps a green control: {not_a_marker}"
        );
    }
}

/// The frame carve-out exempts the wire frame plane, nothing that merely starts
/// with the same five letters, and NO arm but the 32-byte header arm it argues
/// for.
///
/// Its own control is the pair: the lines that MUST stay exempt, beside the ones
/// that must not, so neither a carve-out that stopped working nor one that
/// swallowed the region can read as a pass. The last two negatives are the
/// scoping itself. A sentence about frames that also states a retired payload
/// region or a retired record count is a state record number hiding behind an
/// unrelated word, and the whole-line carve-out this file used to apply let both
/// of them through.
#[test]
fn the_frame_carve_out_is_word_bounded_and_scoped_to_its_own_arm() {
    for exempt in [
        "/// the wire frame prefix carries a 32-byte header of its own",
        "/// rides a 32-byte slot on every StagedFrame and the `frames` vector is",
        "/// at format version 0 the payload region read 480",
        "/// at format version 0 a 500 MB anchor was ~1.09 M records",
    ] {
        assert!(
            !carries_an_unexcused_stale_framing_literal(exempt),
            "a line that names the frame plane, or format version 0, must stay \
             exempt: {exempt}"
        );
    }
    for caught in [
        "/// the framework writes the 32-byte header down",
        "/// a frameless record leaves 480 bytes of payload",
        "/// every frame in the ring leaves 480 bytes of payload",
        "/// a 500 MB anchor is ~1.09 M records, one per frame",
    ] {
        assert!(
            carries_a_stale_framing_literal(caught),
            "the control is only meaningful if the line is stale to begin \
             with: {caught}"
        );
        assert!(
            carries_an_unexcused_stale_framing_literal(caught),
            "the carve-out is for the wire FRAME's own 32-byte header: it must \
             not reach a word that merely begins with it, nor the payload region \
             and record count arms: {caught}"
        );
    }
}

/// The trace ring carve-out takes the header width arm and no other, so a
/// sentence about the other plane cannot stand in for state record framing
/// prose while a number only this plane states still counts wherever it falls.
///
/// Its own control is the pair, and the first half is the anti-vacuity one: the
/// carved-out lines must be lines the header width arm WOULD have counted, or
/// the carve-out is being credited with excluding sentences that were never in
/// the set to begin with.
#[test]
fn the_trace_ring_carve_out_is_scoped_to_the_header_width_arm() {
    for other_plane in [
        "/// the 40-byte `TraceRingRecord` wire form is appended to the arena",
        "// a trace ring's 40-byte records land on `__cerulion/scheduler_trace`",
        "// a fixed 40-bytes stride over the scheduler_trace buffer",
    ] {
        assert!(
            starts_a_token(other_plane, "40-byte", is_number_char),
            "the control is only meaningful if the header width arm would have \
             counted this line: {other_plane}"
        );
        assert!(
            !carries_a_current_framing_marker(other_plane),
            "a 40-byte record on the trace plane is not state record framing \
             prose, and while it counted a file could lose every state sentence \
             it had without moving its count: {other_plane}"
        );
    }
    for still_a_marker in [
        "/// a trace record rides beside the 472-byte state payload region",
        "/// the state record header is 40-byte wide",
        "/// a 500 MB anchor is ~1.11 M records, drained beside the trace ring",
    ] {
        assert!(
            carries_a_current_framing_marker(still_a_marker),
            "the carve-out is for the header WIDTH arm: the payload region and \
             the derived counts are numbers only this plane states, so naming \
             the other one must not discount them: {still_a_marker}"
        );
    }
}

/// The kind space doc says what the kind space IS, and no longer reserves a range
/// this build mints out of.
///
/// `state_ring.rs` reserved "4+" before format version 1 took 4, 5 and 6. Left
/// standing, that sentence tells the next reader those three values are free, and
/// the next format would mint straight over this one's records. A doc is the only
/// thing that carries a reservation, so a doc is the only thing that can be wrong
/// about it.
#[test]
fn the_reserved_kind_range_doc_names_the_range_that_is_actually_reserved() {
    let text = std::fs::read_to_string(repo_root().join("crates/cerulion_core/src/state_ring.rs"))
        .expect("state_ring.rs");
    let offenders: Vec<&str> = text
        .lines()
        .filter(|l| reserves_the_range_this_build_mints(l))
        .collect();
    assert!(
        offenders.is_empty(),
        "the kind doc still reserves a range this build mints out of: {offenders:?}"
    );
    // The control: the doc DOES state a reservation, so this is a scan that found
    // the right sentence rather than one that matched nothing at all.
    //
    // It is built from the SAME predicate shape as the offenders scan, one number
    // over, and that is the repair. It was a literal phrase pair, `7+ are
    // RESERVED` and `values 7+ are RESERVED`, whose second arm could never fire
    // that the first did not (any string holding the longer one holds the
    // shorter), so it was one wording wearing two. Worse, `7+ is reserved` is a
    // wording this file's own control list below declares legitimate, and it
    // reddened this gate on a correct tree.
    let reservations: Vec<&str> = text
        .lines()
        .filter(|l| reserves_the_range_above_the_kinds_this_build_mints(l))
        .collect();
    assert!(
        !reservations.is_empty(),
        "the doc must still reserve the range above the kinds this build mints"
    );
}

/// The reserved-range detector reads the CLAIM rather than one wording of it.
///
/// It was two literal phrases, `4+ reserved` and `Values 4+ are RESERVED`, and a
/// doc saying `4+ is reserved` means exactly the same thing and walked through
/// both. A phrase list is the wrong shape for a sentence a human writes freely,
/// so the detector now asks for the two things the claim is made of, the range
/// and the word, and the control below is the pair: every wording that makes the
/// claim fires, and the sentence the doc is SUPPOSED to carry does not.
#[test]
fn the_reserved_range_detector_reads_the_claim_and_not_one_wording_of_it() {
    for offender in [
        "/// Values 4+ are RESERVED.",
        "/// 4+ reserved.",
        "/// 4+ is reserved.",
        "/// 4+ are reserved for a later format.",
        "// kind 4+ remains reserved",
        // THE EVASION the `4+` spelling left open, and the reason the range
        // half reads three spellings rather than one. A doc that says this
        // beside the `7+` sentence reserves three kinds this build mints, and
        // the scan read empty while the control still found its sentence.
        "/// kinds 4 and above are reserved for a later format",
        "/// kinds >= 4 are reserved for a later format",
        "/// kinds >=4 are reserved for a later format",
    ] {
        assert!(
            reserves_the_range_this_build_mints(offender),
            "a doc that reserves the range this build mints out of must be caught, \
             however it is worded: {offender}"
        );
    }
    for allowed in [
        "// version 1 record this build mints; values 7+ are RESERVED.",
        "/// 7+ is reserved.",
        "/// kinds 7 and above are reserved for a later format",
        "/// kind 4 is the rank-bearing final record",
        "/// 14+ is reserved, a range nothing here mints",
        "/// kinds 14 and above are reserved, a range nothing here mints",
        "/// its own doc already reserved 4 and up for, which this one took",
    ] {
        assert!(
            !reserves_the_range_this_build_mints(allowed),
            "the detector must not fire on the reservation the doc is supposed to \
             carry, or the gate reads red on a correct tree: {allowed}"
        );
    }

    // The POSITIVE control's predicate is the same claim one number over, so it
    // gets the same pair. Without this it could stop firing and the offenders
    // scan above would go unwitnessed: an empty scan and a control that finds
    // nothing look identical from the outside, which is the whole failure this
    // file exists to prevent.
    for reservation in [
        "// version 1 record this build mints; values 7+ are RESERVED.",
        "/// 7+ is reserved.",
        "/// kinds 7 and above are reserved for a later format",
        "/// kinds >= 7 are reserved for a later format",
    ] {
        assert!(
            reserves_the_range_above_the_kinds_this_build_mints(reservation),
            "the sentence the doc is supposed to carry must be found however it \
             is worded: {reservation}"
        );
    }
    for not_that_reservation in [
        "/// Values 4+ are RESERVED.",
        "/// 17+ is reserved, a range nothing here mints",
        "/// kind 7 is a kind this build mints",
    ] {
        assert!(
            !reserves_the_range_above_the_kinds_this_build_mints(not_that_reservation),
            "the positive control must not be satisfied by a sentence that is not \
             the reservation, or it stops witnessing anything: {not_that_reservation}"
        );
    }
}

/// `cerulion_bag/src/writer.rs` states the state channel's framing in the DOC
/// of the function that declares the channel, and the control is that cutting
/// that doc out fails the sweep.
///
/// What this closes is not a wrong count, it is what the count RESTS ON. The
/// whole of this file's minimum used to be carried by one comment in the middle
/// of a function body: prose a reflow, a tidy or an unrelated refactor can take
/// away without touching a line of code, and the sweep would have read the loss
/// as a file that simply says less. The framing is now stated on the doc of
/// `reserved_channels`, the item that declares `__cerulion/state` and the
/// record width its descriptor carries, so the fact sits on the declaration the
/// language ties it to.
///
/// Both halves are here. The doc must name the header width, the payload region
/// and the record size; and with the doc block cut out of the text, the file's
/// marker count must fall UNDER its own region minimum, which is what proves
/// the minimum is resting on this doc rather than on prose elsewhere in the
/// file.
#[test]
fn the_writers_state_channel_doc_carries_the_framing_the_control_rests_on() {
    let path = "crates/cerulion_bag/src/writer.rs";
    let text = std::fs::read_to_string(repo_root().join(path)).expect("writer.rs");
    let doc = doc_block_above(&text, "fn reserved_channels(")
        .expect("the function that declares the reserved channels must carry a doc block");
    assert!(
        doc.lines().any(carries_a_current_framing_marker),
        "the declaring function's own doc must state the framing it declares:\n{doc}"
    );
    for number in ["40-byte", "472", "512-byte"] {
        assert!(
            doc.contains(number),
            "the doc must name the record shape whole, and {number} is missing \
             from it:\n{doc}"
        );
    }

    // THE CONTROL: cut that doc block out and the file falls under the minimum
    // the region declares for it. Without this the test above is satisfied by a
    // doc that says the right thing while the minimum leans on something else,
    // which is the exact shape of the vacuity this file exists to catch.
    let min = REGION
        .iter()
        .find(|f| f.path == path)
        .expect("writer.rs is in the scan region")
        .min_markers;
    let without = text.replace(&doc, "");
    assert!(
        without.len() < text.len(),
        "the doc block must really have been cut out of the text"
    );
    let left = without
        .lines()
        .filter(|l| carries_a_current_framing_marker(l))
        .count();
    assert!(
        left < min,
        "cutting the declaring function's doc must FAIL the sweep, or this \
         file's control is resting on prose somewhere else in it: {left} \
         markers left, minimum {min}"
    );
}
