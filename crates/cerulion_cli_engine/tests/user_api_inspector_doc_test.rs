// SPDX-License-Identifier: AGPL-3.0-only
//! `USER_API.md`'s `graph.validate` paragraph is the only place a user is told
//! what the daemon's node inspector does — and every number in it was a
//! hand-copied literal that nothing read.
//!
//! The in-code verdicts derive their intervals from the constants
//! (`PipeEnd::note` renders `READER_GRACE.as_millis()`), so they cannot rot;
//! the reference sentence carried `30 s`, `1 MiB` and "half a second" as prose.
//! A review that proposed a longer clean-exit bound would have staled the
//! last of those the moment it landed, with the log and the reference
//! disagreeing about how long the daemon waited and no gate moving.
//!
//! So this file reads the paragraph and derives every number from the shipped
//! constant, exactly as the code does. It is a CONTENT pin, not a formatting
//! one: the text is whitespace-normalized first, because the file is
//! hard-wrapped prose and a phrase is routinely split across a line break.
//!
//! Parallel-safe — a file read, no transport, no process spawn.

use cerulion_cli_engine::node_inspector::{
    DEFAULT_INSPECT_TIMEOUT, MAX_INSPECT_OUTPUT_BYTES, READER_GRACE,
};

/// The `graph.validate` paragraph, whitespace-normalized.
fn validate_paragraph() -> String {
    let raw = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs/user-api.md"),
    )
    .expect("USER_API.md must be readable from the crate root");
    let doc = raw.split_whitespace().collect::<Vec<_>>().join(" ");
    // ANTI-TAUTOLOGY: without a reachable anchor every assertion below is
    // vacuous against a renamed, moved or deleted paragraph.
    let start = unique_anchor(
        &doc,
        "`graph.validate` runs the engine's full `graph validate`",
        "the `graph.validate` paragraph",
    );
    let rest = &doc[start..];
    let closing = "in its own one-shot process.)";
    let end = unique_anchor(rest, closing, "the paragraph's closing parenthetical") + closing.len();
    rest[..end].to_string()
}

/// The byte offset of `needle` in `doc`, REQUIRING it to occur exactly once:
/// with a plain `find` a valid first paragraph followed by a stale duplicate
/// would leave the duplicate unchecked and the gate green.
fn unique_anchor(doc: &str, needle: &str, what: &str) -> usize {
    let hits: Vec<usize> = doc.match_indices(needle).map(|(i, _)| i).collect();
    match hits.as_slice() {
        [one] => *one,
        [] => panic!("{what} moved or was renamed — this file is pinning nothing until its anchor is updated"),
        many => panic!(
            "{what} appears {} times — a duplicate copy would be checked by nothing; keep ONE",
            many.len()
        ),
    }
}

#[test]
fn a_duplicated_paragraph_is_refused_not_silently_half_checked() {
    let once = "a b ANCHOR c d";
    assert_eq!(unique_anchor(once, "ANCHOR", "x"), 4);
    let twice = "a ANCHOR b ANCHOR c";
    let err = std::panic::catch_unwind(|| unique_anchor(twice, "ANCHOR", "the anchor"))
        .expect_err("two copies must be refused");
    let msg = err.downcast_ref::<String>().cloned().unwrap_or_default();
    assert!(msg.contains("appears 2 times"), "{msg}");
    let none = std::panic::catch_unwind(|| unique_anchor("nothing", "ANCHOR", "the anchor"))
        .expect_err("a missing anchor must be refused");
    let msg = none.downcast_ref::<String>().cloned().unwrap_or_default();
    assert!(msg.contains("moved or was renamed"), "{msg}");
}

/// Every number the paragraph quotes is DERIVED here from the constant that
/// produces it, so a constant that moves fails this arm instead of leaving the
/// reference quietly wrong.
#[test]
fn the_validate_paragraph_quotes_the_inspectors_shipped_bounds() {
    let para = validate_paragraph();

    let deadline = format!("killed after {} s", DEFAULT_INSPECT_TIMEOUT.as_secs());
    assert!(
        para.contains(&deadline),
        "the inspection deadline is {DEFAULT_INSPECT_TIMEOUT:?}, so the paragraph must say \
         {deadline:?}:\n{para}"
    );

    // The cap is quoted in the unit a user reads. Deriving the rendering (not
    // the literal "1 MiB") is what makes a change to the constant fail here.
    assert_eq!(
        MAX_INSPECT_OUTPUT_BYTES % (1 << 20),
        0,
        "the paragraph renders the cap in whole MiB; a cap that is not a whole \
         number of MiB needs a new rendering here AND in the doc"
    );
    let cap = format!("each pipe capped at {} MiB", MAX_INSPECT_OUTPUT_BYTES >> 20);
    assert!(
        para.contains(&cap),
        "the retention cap is {MAX_INSPECT_OUTPUT_BYTES} bytes, so the paragraph must say \
         {cap:?}:\n{para}"
    );

    let grace = format!(
        "has still not closed {} ms after the child is reaped",
        READER_GRACE.as_millis()
    );
    assert!(
        para.contains(&grace),
        "the collection grace is {READER_GRACE:?}, so the paragraph must say {grace:?}:\n{para}"
    );
}

/// The paragraph must also state the two rules the code actually implements,
/// not the earlier half of each: an unfinished document channel is reported as
/// an OBSERVATION with the possibilities named and none of them asserted (a
/// reader that was merely not scheduled is not an escapee, and neither is an
/// in-group member the SIGKILL could not end in time), and stderr is fatal in
/// exactly one case — a flood — while held-open or merely chatty stderr is
/// logged.
#[test]
fn the_validate_paragraph_states_the_observation_and_the_one_fatal_stderr_case() {
    let para = validate_paragraph();
    let flood = format!(
        "FLOODS past the same {} MiB cap is a load-time defect and does fail the check",
        MAX_INSPECT_OUTPUT_BYTES >> 20
    );
    for (needle, why) in [
        (
            "the pipe may still be held (by a process outside the group, or by one the kill \
             could not end in time), or the reader thread was not scheduled",
            "the verdict lists the POSSIBILITIES, open-ended (`PipeEnd::note` hedges with \
             \"for example\", because an in-group member the kill could not end in time is a \
             holder too); a doc that asserts the escapee sends an operator hunting for a \
             process that need not exist, and a doc that CLOSES the list sends the next one \
             hunting for a cause the verdict never claimed to have enumerated",
        ),
        (
            "close-on-exec, so anything the library `exec`s cannot hold that channel",
            "the SCOPE of the held-open verdict: a library that exec's is structurally \
             incapable of it, so a doc that omits this sends the reader looking for the \
             wrong kind of child",
        ),
        (
            "a constructor that `fork()`s WITHOUT exec'ing (`daemon(3)`) still can, and is \
             one of the holders the verdict names",
            "…and the reachable shape, named as ONE of the possibilities. The paragraph said \
             two sentences earlier that the verdict asserts none of them, then asserted this \
             one — sending a user whose healthy library failed on a saturated runner to audit \
             constructors for a `daemon(3)` that is not there. It must not over-count either: \
             'ONE of the two causes' re-closes the list the sentence before it opened",
        ),
        (
            "logged and never fatal",
            "held-open or chatty stderr does not veto a good document",
        ),
        (
            "whatever it wrote to stdout is quoted instead, labelled as stdout",
            "a failing child that spoke only on stdout is quoted — the `fd 2 is closed` \
             refusal is exactly that shape",
        ),
        (
            flood.as_str(),
            "the ONE case where stderr IS fatal — the earlier wording said stderr never is, \
             while a test requires exactly this failure",
        ),
    ] {
        assert!(para.contains(needle), "{why}; missing {needle:?}:\n{para}");
    }
}
