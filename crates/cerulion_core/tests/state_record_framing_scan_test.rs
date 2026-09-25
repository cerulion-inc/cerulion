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
struct Scanned {
    path: &'static str,
    min_markers: usize,
}

/// The file SET the framing sweep covered, each with its own control.
///
/// It is a SET rather than one file because a framing sentence lives outside
/// `state_ring.rs` too: `state_carrier/fork.rs` states the payload region in
/// prose, and a scan of one file reaches neither it nor the ring room
/// precheck's arena term. Adding a file here is how a new framing sentence
/// joins the gate.
const REGION: &[Scanned] = &[
    Scanned {
        path: "crates/cerulion_core/src/state_ring.rs",
        min_markers: 8,
    },
    Scanned {
        path: "crates/cerulion_core/src/state_carrier/fork.rs",
        min_markers: 1,
    },
    Scanned {
        path: "crates/cerulion_bag/src/writer.rs",
        min_markers: 1,
    },
    Scanned {
        path: "crates/cerulion_bagd/src/anchor_window.rs",
        min_markers: 4,
    },
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
        min_markers: 3,
    },
    // The ring room precheck's hand oracle writes the arena's record count down
    // (64 KiB of arena is 139 records). That number is DERIVED from the payload
    // region, so it moves with the header, and it is a framing number written in
    // a file that carries no other state record prose at all.
    Scanned {
        path: "crates/cerulion_core/src/graph/runtime.rs",
        min_markers: 1,
    },
];

/// A line may still carry a previous format's number when it SAYS so.
///
/// Two allowances, both narrow. A line naming format version 0 is talking about
/// the layout that preceded this one, which the docs must keep doing: a reader
/// who meets a refusal mentioning it needs somewhere to look it up. A line naming
/// a FRAME is about the wire frame prefix, a different plane with its own
/// 32-byte header that this sweep must not touch.
fn is_allowed(line: &str) -> bool {
    let lower = line.to_ascii_lowercase();
    lower.contains("format version 0")
        || lower.contains("format_version_0")
        || lower.contains("frame")
}

/// Does this line carry a state record framing number that moved?
fn carries_a_stale_framing_literal(line: &str) -> bool {
    if line.contains("32-byte") {
        return true;
    }
    // A bare 480, the previous payload region, as its own word.
    let bytes = line.as_bytes();
    let mut i = 0;
    while let Some(at) = line[i..].find("480") {
        let s = i + at;
        let e = s + 3;
        let before_ok = s == 0 || !(bytes[s - 1] as char).is_ascii_alphanumeric();
        let after_ok = e == bytes.len() || !(bytes[e] as char).is_ascii_alphanumeric();
        if before_ok && after_ok {
            return true;
        }
        i = e;
    }
    false
}

/// Does this line carry a CURRENT framing marker, for the control half?
fn carries_a_current_framing_marker(line: &str) -> bool {
    line.contains("40-byte") || line.contains("472") || line.contains("512-byte")
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
            if carries_a_stale_framing_literal(line) && !is_allowed(line) {
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
        .filter(|l| l.contains("4+ reserved") || l.contains("Values 4+ are RESERVED"))
        .collect();
    assert!(
        offenders.is_empty(),
        "the kind doc still reserves a range this build mints out of: {offenders:?}"
    );
    // The control: the doc DOES state a reservation, so this is a scan that found
    // the right sentence rather than one that matched nothing at all.
    assert!(
        text.contains("7+ are RESERVED") || text.contains("values 7+ are RESERVED"),
        "the doc must still reserve the range above the kinds this build mints"
    );
}
