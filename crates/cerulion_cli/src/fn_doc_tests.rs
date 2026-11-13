// SPDX-License-Identifier: AGPL-3.0-only
//! Each `cerulion pair` dispatch function and the shutdown-signal handler in
//! `main.rs` carry their own `///` doc.
//!
//! A function inserted between another's `///` block and its `fn`, with no
//! blank line, joins its own doc onto that block: rustdoc and every reader
//! then give the first function's text to the inserted one, and the first is
//! left with none. No lint sees it, as these functions are private to a
//! binary, so this reads `main.rs` and checks the run of `///` lines directly
//! above each named `fn`, the attributes between them skipped.

const MAIN: &str = include_str!("main.rs");

/// The `///` lines directly above the `nth` line of `src` that starts with
/// `signature`, attributes between them skipped, in source order.
fn doc_above<'a>(src: &'a str, signature: &str, nth: usize) -> Vec<&'a str> {
    let lines: Vec<&str> = src.lines().collect();
    let at = lines
        .iter()
        .enumerate()
        .filter(|(_, line)| line.starts_with(signature))
        .nth(nth)
        .unwrap_or_else(|| panic!("no line {nth} starting with {signature:?}"))
        .0;
    let mut doc = Vec::new();
    for line in lines[..at].iter().rev() {
        let trimmed = line.trim_start();
        if trimmed.starts_with("///") {
            doc.push(*line);
        } else if !(trimmed.starts_with("#[") && doc.is_empty()) {
            break;
        }
    }
    doc.reverse();
    doc
}

#[test]
fn each_pair_dispatch_fn_and_the_signal_handler_carry_their_own_doc() {
    let cases = [
        (
            "fn setup_ctrlc_handler()",
            0,
            "/// Install the process-wide shutdown-signal handler and return the shared",
        ),
        (
            "fn pair_start_exit_code()",
            0,
            "/// `cerulion pair --start`: one pairing code made on this robot and followed",
        ),
        (
            "fn pair_start_exit_code()",
            1,
            "/// `cerulion pair --start` where the robot daemons do not build.",
        ),
        (
            "fn pair_exit_code(",
            0,
            "/// The desk side of `cerulion pair`: resolve and spawn `cerulion-connectd",
        ),
    ];
    for (signature, nth, first) in cases {
        assert_eq!(
            doc_above(MAIN, signature, nth).first().copied(),
            Some(first),
            "the doc above {signature} number {nth}"
        );
    }
    // The handler's doc ends on its signal-safety note: nothing was appended.
    assert_eq!(
        doc_above(MAIN, "fn setup_ctrlc_handler()", 0)
            .last()
            .copied(),
        Some("/// run in signal context. Making the feature explicit left it BYTE-IDENTICAL.")
    );
}

#[test]
fn a_doc_appended_to_another_fns_doc_is_seen_and_the_other_left_bare() {
    let joined = "\
/// The handler's own doc.
/// Its last line.
/// The inserted function's doc.
#[cfg(unix)]
fn inserted() {}

fn handler() {}
";
    assert_eq!(
        doc_above(joined, "fn inserted()", 0),
        vec![
            "/// The handler's own doc.",
            "/// Its last line.",
            "/// The inserted function's doc.",
        ]
    );
    assert!(doc_above(joined, "fn handler()", 0).is_empty());
}
