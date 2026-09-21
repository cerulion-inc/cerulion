#![cfg(unix)]
//! The socket-directory rule is stated in TWO places, and neither is here.
//!
//! `cerulion_wsd`, `cerulion-netd` and `cerulion-vizd` share one socket
//! lifecycle (`cerulion_hygiene`) precisely so there is one rule to read and
//! one to change. It is written down twice on purpose and no more: once
//! code-side, in `cerulion_hygiene`'s module docs, and once user-facing, in
//! USER_API's socket-path cell — and `cerulion_hygiene`'s
//! `user_api_doc_test.rs` keeps those two in agreement. A THIRD copy has
//! nothing keeping it in agreement, which is what this file forbids for this crate.
//! It carried one in TWO places —
//! `AGENTS.md` and `src/hygiene.rs`, byte-identical — and when the shared crate
//! changed the rule, both copies went on describing the old shape. What the
//! rule says today is `cerulion_hygiene`'s module docs' business, and this file
//! deliberately does not say it — including here: a pin that forbids a further
//! copy and then carries one is the same failure it exists to catch, one file
//! further out. `AGENTS.md` is the file an agent working in this crate reads as
//! the contract, so the stale copy did not merely rot: it instructed.
//!
//! `cerulion_hygiene/tests/user_api_doc_test.rs` already forbids exactly this
//! for USER_API's wsd row (`the_wsd_row_defers_to_the_one_rule_instead_of_
//! restating_it`). This is the same pin, one directory over: the crate's own
//! docs may POINT at the rule and must not RESTATE it. The walk therefore
//! covers this crate's `tests/` as well as its `src/` — this file included,
//! which is why the marker is assembled from parts it never spells.

use std::path::{Path, PathBuf};

/// The word every copy of the rule has used and a pointer never needs: it
/// names the arm the rule's owner may change, and it is a VERB about
/// behaviour, not a name a deferring sentence has any reason to spell.
///
/// Assembled from two halves so this file — which the walk below now reads,
/// like every other Rust source in the crate — does not spell the marker
/// anywhere in its own source. An exclusion for "the line that defines the
/// constant" would be the alternative, and it exempts exactly the file whose
/// job is to be subject to its own rule.
const RESTATEMENT_MARKER: &str = concat!("tight", "ened");

/// This crate's own documentation surface: the instruction file plus every
/// Rust source file under `src/` and `tests/` (module docs, item docs and
/// comments alike). `tests/` is in the walk because a test file's module doc
/// is documentation an agent reads and can be wrong in exactly the same way —
/// which is how THIS file shipped a stale copy of the rule while pinning
/// everyone else against one.
fn wsd_doc_files() -> Vec<PathBuf> {
    let crate_root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut files = vec![crate_root.join("AGENTS.md")];
    let mut stack = vec![crate_root.join("src"), crate_root.join("tests")];
    while let Some(dir) = stack.pop() {
        for entry in std::fs::read_dir(&dir).expect("read a source directory of the crate") {
            let path = entry.expect("read a directory entry").path();
            if path.is_dir() {
                stack.push(path);
            } else if path.extension().and_then(|e| e.to_str()) == Some("rs") {
                files.push(path);
            }
        }
    }
    files.sort();
    files
}

#[test]
fn no_wsd_doc_restates_the_socket_directory_rule() {
    let files = wsd_doc_files();
    // Anti-vacuity: the walk must really be reading the files that carried the
    // copies, or "no file says it" would be satisfied by finding nothing.
    // `doc_deference_test.rs` is this file: the walk covers `tests/` too, so
    // the pin applies to itself — and a `tests/` directory dropped from the
    // walk fails HERE rather than going quietly green.
    for required in ["AGENTS.md", "hygiene.rs", "doc_deference_test.rs"] {
        assert!(
            files
                .iter()
                .any(|f| f.file_name().and_then(|n| n.to_str()) == Some(required)),
            "the walk must reach {required} — the first two carried the copies, and the \
             third is this file, which must be subject to its own rule; found {files:?}"
        );
    }

    let offenders: Vec<String> = files
        .iter()
        .filter(|path| {
            std::fs::read_to_string(path)
                .expect("read a wsd doc file")
                .contains(RESTATEMENT_MARKER)
        })
        .map(|path| path.display().to_string())
        .collect();
    assert!(
        offenders.is_empty(),
        "a second copy of the socket-directory rule is back in {offenders:?} (it says \
         {RESTATEMENT_MARKER:?}). Two copies is how it drifted: when `cerulion_hygiene` \
         changed the rule, both of this crate's copies went on describing the old shape. \
         POINT at the rule — `cerulion_hygiene`'s module docs and USER_API's socket-path \
         cell — and do not restate it here."
    );
}

#[test]
fn the_wsd_docs_point_at_the_one_rule() {
    // The complement, and it is what stops the pin above being satisfied by
    // deleting the mention altogether: an agent must still be able to FIND the
    // rule from either file.
    for name in ["AGENTS.md", "src/hygiene.rs"] {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(name);
        let text = std::fs::read_to_string(&path).expect("read a wsd doc file");
        assert!(
            text.contains("socket-directory rule"),
            "{name} must still name the socket-directory rule so a reader knows one exists"
        );
        assert!(
            text.contains("cerulion_hygiene") && text.contains("USER_API"),
            "{name} must point at BOTH places a reader can find the rule — \
             `cerulion_hygiene`'s module docs, where it is stated, and USER_API's \
             socket-path cell, where a user reads it"
        );
    }
}
