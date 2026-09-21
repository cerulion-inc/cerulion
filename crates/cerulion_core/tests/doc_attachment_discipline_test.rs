//! A `///` block belongs to the item BELOW it — a structural guard for
//! the doc-ANNEX class.
//!
//! # The class, and why it earned a test
//!
//! Inserting an item between an existing `///` block and the item it documents
//! silently re-parents the doc: the prose now describes its NEIGHBOUR, and the
//! item that wrote it is left bare. Nothing complains. `rustdoc` renders the
//! wrong text under the wrong name, a reader is told something false about the
//! code in front of them, and the item whose contract was written down loses it.
//!
//! It has occurred in three distinct shapes:
//!
//! | # | shape | instances |
//! |---|---|---|
//! | A | attributes (`#[inline]`/`#[must_use]`) separated from their `fn` by another item's doc | 1 |
//! | B | a BLANK line between a `///` block and its item | 3 |
//! | C | a new item's doc APPENDED to an existing run, no blank line, no attributes | 2 |
//!
//! # What this guard checks — A and B only, and C deliberately NOT
//!
//! Shapes **A and B are caught**, each with a failing fixture below, and each
//! measured to flag NOTHING on the real tree.
//!
//! Shape **C is not caught, and that is a deliberate narrowing rather than an
//! oversight.** The first version of this guard tried, with a textual heuristic
//! (a `///` run that opens more than one issue-citation paragraph). It was wrong
//! in both directions, and the numbers are recorded here so nobody re-litigates
//! it from intuition:
//!
//! * **37 false positives** across the files below. Opening two citation
//!   paragraphs in one doc is ordinary and correct in this tree — a doc that
//!   explains a decision and then cites the later ticket that revised it does it
//!   all the time.
//! * **and it still MISSED both real instances** (measured: it counts 1 on the
//!   real instance-C shape). The appended paragraph's heading follows a non-empty
//!   `///` line with no separator, so it is not in "paragraph-start" position,
//!   which is the only position that heuristic counted.
//!
//! The inverse rule — a citation heading on a line whose predecessor is a
//! non-empty `///` line — does catch the real shape, and flags **86** sites,
//! nearly all of them ordinary text where a line wrap happened to put a citation
//! at the start of a continuation line.
//!
//! So the signal for shape C is not in the text, and a guard that cries wolf 37
//! or 86 times is worse than no guard: it gets suppressed, and its green stops
//! meaning anything. Shape C is therefore not detected (a real fix needs the token
//! stream or a rustdoc pass, not a line regex). What ships is a guard whose every
//! report is real.
//!
//! It does NOT check that a doc DESCRIBES its item (undecidable here), that every
//! item HAS one (`missing_docs` is a lint, and this repo does not want it on
//! private items), or anything about `//!` module docs.

/// The files this guard covers — the surface across all three crates.
///
/// The set follows the PR rather than the first four instances, because two of the
/// six happened in crates the original set excluded — a guard scoped to where the
/// bug was last seen is a guard that misses where it goes next.
const COVERED: &[&str] = &[
    // `cerulion_core`, relative to this crate's manifest dir.
    "src/read_outcome.rs",
    "src/trace_ring.rs",
    "src/shm_ring.rs",
    "src/graph/runtime.rs",
    "tests/read_outcome_capture_iox2_test.rs",
    // …and the two crates where the class recurred.
    "../cerulion_cli_engine/src/replay_engine.rs",
    "../cerulion_cli_engine/tests/replay_engine_test.rs",
    "../cerulion_bagd/src/lib.rs",
];

/// A `///` line, an attribute line, a blank, or an item — the only distinction the
/// rules need.
#[derive(Debug, PartialEq, Eq)]
enum Line {
    Doc,
    Attr,
    Blank,
    Other,
}

fn classify(line: &str) -> Line {
    let t = line.trim_start();
    if t.is_empty() {
        Line::Blank
    } else if t.starts_with("///") {
        Line::Doc
    } else if t.starts_with("#[") || t.starts_with("#!") {
        Line::Attr
    } else {
        Line::Other
    }
}

/// Every offence in one file, as `(line number, what was found)`.
///
/// Walks each `///` run and looks at what FOLLOWS it. Attributes pass through —
/// that is the normal shape. A BLANK line, or a second `///` run after
/// attributes, means the block has been separated from its item.
fn offences(src: &str) -> Vec<(usize, String)> {
    let lines: Vec<&str> = src.lines().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < lines.len() {
        if classify(lines[i]) != Line::Doc {
            i += 1;
            continue;
        }
        let doc_start = i;
        while i < lines.len() && classify(lines[i]) == Line::Doc {
            i += 1;
        }
        let mut saw_attr = false;
        while i < lines.len() && classify(lines[i]) == Line::Attr {
            saw_attr = true;
            i += 1;
        }
        match lines.get(i).map(|l| classify(l)) {
            // The normal shape: doc [+ attrs] then the item.
            Some(Line::Other) | None => {}
            // SHAPE B — a blank line between a doc block and its item.
            Some(Line::Blank) => out.push((
                doc_start + 1,
                "a blank line separates this `///` block from the item below it".to_string(),
            )),
            // SHAPE A — doc, attributes, then ANOTHER doc run: the attributes and
            // the item below belong to the SECOND block, so this one documents
            // nothing and the attributes have been taken from their own `fn`.
            Some(Line::Doc) if saw_attr => out.push((
                doc_start + 1,
                "this `///` block is followed by attributes and then a SECOND `///` \
                 block — the attributes and the item below belong to the second, so \
                 this one documents nothing"
                    .to_string(),
            )),
            // Two adjacent runs with no attributes between them is shape C, which
            // this guard deliberately does not judge (see the module docs).
            Some(Line::Doc) => {}
            Some(Line::Attr) => unreachable!("attributes were consumed above"),
        }
    }
    out
}

#[test]
fn a_doc_block_is_never_separated_from_the_item_it_documents() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut all: Vec<String> = Vec::new();
    for rel in COVERED {
        let path = root.join(rel);
        let src = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("covered file {rel} must be readable: {e}"));
        for (line, why) in offences(&src) {
            all.push(format!("{rel}:{line}: {why}"));
        }
    }
    assert!(
        all.is_empty(),
        "a `///` block must be followed by attributes and then its item, and \
         nothing else — inserting an item in between silently re-parents the \
         prose onto its neighbour and leaves the original bare (three shapes of it \
         have occurred; see the module docs):\n  {}",
        all.join("\n  ")
    );
}

/// The anti-tautology half: the rules really fire on the shapes they claim.
///
/// Without this, a walker that classified everything as "fine" — or one whose
/// file list silently read nothing — would pass the guard above and report
/// nothing forever. Each fixture is the shape of a REAL instance.
#[test]
fn the_guard_fires_on_each_shape_it_claims() {
    let shapes: &[(&str, &str)] = &[
        (
            "SHAPE A (instance 1) — `annotation_folds` inserted between \
             `records_fold`'s doc + `#[inline]`/`#[must_use]` and `records_fold`",
            "\
/// The first function's contract.
#[inline]
#[must_use]
/// The second function's doc.
fn second() {}
",
        ),
        (
            "SHAPE B (instances 2-4) — a blank line between a doc and its item",
            "\
/// A module's rationale.

#[cfg(test)]
mod something_else {}
",
        ),
    ];
    for (what, src) in shapes {
        assert!(
            !offences(src).is_empty(),
            "this shape has occurred in real code and must be reported — {what}:\n{src}"
        );
    }

    // SHAPE C, stated as a KNOWN GAP rather than left for a reader to discover:
    // this is the real instance-5 text and the guard does NOT report it. Asserting
    // the gap keeps the module docs accurate — if someone later finds a zero-false-
    // positive rule, this assertion fails and tells them to update the docs.
    let shape_c = "\
/// The wire format of section 4.
///
/// Rows are `(a, b, c)`, little-endian.
/// The row-key rule, in one place.
///
/// Both ends call this.
fn reject_duplicates() {}
";
    assert!(
        offences(shape_c).is_empty(),
        "shape C is a DOCUMENTED gap (the module docs say why: 37 false positives \
         one way, 86 the other, and the narrower rule misses this very text). If \
         it is now caught, that is good news — update the module docs and move \
         this into `shapes` above:\n{:?}",
        offences(shape_c)
    );

    // …and the CORRECT shapes are silent, or the guard is noise.
    for ok in [
        "/// Documented.\nfn f() {}\n",
        "/// Documented.\n#[inline]\nfn f() {}\n",
        "/// Documented.\n///\n/// With a blank doc line.\n#[must_use]\nfn f() {}\n",
        "// a plain comment\n\nfn f() {}\n",
        "/// Two items, each with its own doc.\nfn a() {}\n\n/// The second.\nfn b() {}\n",
        // The false-positive class the dropped heuristic fired on, 37 times: a
        // doc that explains a decision and then cites the ticket that revised it.
        "/// TICKET-1111: one heading.\n///\n/// TICKET-2222 revised it.\nfn f() {}\n",
    ] {
        assert!(
            offences(ok).is_empty(),
            "this shape is correct and must not be reported: {ok:?} -> {:?}",
            offences(ok)
        );
    }
}
