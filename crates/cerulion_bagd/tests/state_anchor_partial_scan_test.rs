// SPDX-License-Identifier: AGPL-3.0-only
//! The word PARTIAL, swept mechanically over the state anchor plane's sources.
//!
//! PARTIAL is a RESIM VERDICT word, and the resim has no verdict by that name:
//! its exit contract runs 0 to 6, and a capture whose ranks did not all restore
//! is refused as not replay-grade with exit 2. The word used to name a
//! different fact in these files as well, a run whose rank published no state
//! ring, so one `bag info` session could hand an operator one word for two
//! things, neither of which the resim answers with. Those sentences now state
//! the fact instead: every anchor of the run LACKS that rank's records, and the
//! resim refuses a capture of it. This file is what keeps them stated that way.
//!
//! # A POSITIVE scan, and why the conditional one it replaces was not enough
//!
//! The obvious scan is conditional: refuse `partial` only where it appears
//! beside the rank vocabulary. A lone `partial` then walks straight through,
//! and a sentence reworded to drop the second token stops being scanned
//! without anything failing. So the rule here is the other way round: EVERY
//! occurrence of the word in these files is a failure unless the file's own
//! ALLOW LIST names it. A new sentence has to be read by a person and added
//! here, which is the whole point.
//!
//! # The allow list is also the CONTROL, and that is the important half
//!
//! A refusal-only scan whose region moved, whose file was renamed, or whose
//! read failed matches nothing and reads as a pass. So every allow entry must
//! MATCH something: an entry that matches no line means the sentence it was
//! written for is gone or reworded, and the scan fails rather than quietly
//! covering less. An emptied or truncated allow list therefore fails loudly
//! instead of passing vacuously, and so does a file that dropped out of the
//! region.
//!
//! # What is scanned
//!
//! PROSE: doc comments, line comments and any line carrying a string literal.
//! An identifier named `partial` in a test is not operator surface and not
//! reader surface, and gating it would make the allow list a list of variable
//! names rather than of sentences.

use std::path::{Path, PathBuf};

/// The repo root, from this crate's manifest directory.
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("crates/<crate>/ sits two levels under the repo root")
        .to_path_buf()
}

/// One file in the scan region, with the occurrences it is allowed to carry.
struct Scanned {
    path: &'static str,
    /// A distinguishing fragment of each allowed sentence, as it appears.
    ///
    /// A FRAGMENT rather than the whole line, so re-wrapping a doc comment does
    /// not fail the scan, and long enough to name one sentence rather than a
    /// word, so an entry cannot quietly permit a new one.
    allowed: &'static [&'static str],
}

/// The state anchor plane's NAMED source set.
///
/// Named rather than globbed. A glob over `src/` would grow the region every
/// time an unrelated crate learned the word, and the failure this file exists
/// to catch lives in exactly these nine files: the recorder's anchor plane, the
/// capture manifest, the coverage manifest, the graph side that arms a rank's
/// ring, and the ring itself.
const REGION: &[Scanned] = &[
    Scanned {
        path: "crates/cerulion_bagd/src/anchor_window.rs",
        allowed: &[
            "PARTIAL buffer a refused anchor had",
            "`StateAssembler::feed` drops the partial anchor when",
            "exactly as permanently as one whose partial",
            "partial anchor when a skip arrives, so",
            "partial buffer a refused anchor held.",
            "partial buffer an in-flight anchor had",
            "partial buffer for the newcomer and answers `false`,",
            "partial buffer of its own.",
            "partial buffer that can never",
            "partial checkpoint IS an answer\");",
            "partial checkpoint and an all-declined one both are.",
            "partial head anchor its cursor landed inside, and serves the",
            "partial head anchor the live cursor landed inside),",
            "partial one forever: the",
            "partial shape is written and stamped; this shape is not writ",
            "partial.len() >= 4, \"precondition: several chunks in flight\"",
            "than throw the capture away: a partial",
            "…and STRICTLY NEVER SILENTLY PARTIAL",
        ],
    },
    Scanned {
        path: "crates/cerulion_bagd/src/flashback_plane.rs",
        allowed: &[
            "partial history can only fail to withhold budget.",
            "partial listing it happily evicts real captures to satisfy a",
            "partial listing makes the plan DELETE captures it should",
            // READ BY A PERSON, and kept rather than reworded: the word here is
            // the NAME of the recorder's rule, whose canonical statement is the
            // `anchor_window.rs` module-doc heading this list already allows on
            // that file's line. The sentence CITES the rule; it does not give
            // the resim verdict's word a second meaning.
            "rule is STRICTLY NEVER SILENTLY PARTIAL. A member that reached",
        ],
    },
    Scanned {
        path: "crates/cerulion_bagd/src/lib.rs",
        allowed: &[
            "\"\\\"partial\\\"\",",
            "PARTIAL STEP heading the trace. A mid-run attach",
            "PARTIAL: some live-topic scans failed, so the untapped list",
            "Partial, and warning on that would train the operator",
            "Partial` deliberately does NOT",
            "Partial`, and escalating on that would WARN on essentially e",
            "Partial`. A discovery run that",
            "head-step gate discards the leading partial",
            "index-aligned with `rings`, so a partial",
            "least {mib} MiB (a floor from a PARTIAL anchor",
            "never `Partial`.",
            "partial buffer is evidence of one",
            "partial checkpoint's capture read as clean coverage of a sma",
            "partial head step is discarded**",
            "partial is wrong at record 1",
            "partial one, and those are",
            "partial one, because without the cause",
            "partial set cannot be committed without",
            "partial state is strictly more useful",
            "partial) and the DEPARTURE ring is not (it carries",
            "partial-bag path. `cerulion_cli_engine` derives its own",
            "sound attach: its head anchor is partial by",
            "tell \"no trace at all\" from \"a partial one\".",
            "told it failed to resolve, and Partial/Full are",
        ],
    },
    Scanned {
        path: "crates/cerulion_bagd/src/state_coverage.rs",
        allowed: &[
            "\"the mid-run attach discards its partial head\",",
            "dropped as a mid-run attach's partial head anchor.",
            "ledger is still discarding a partial head anchor.",
            "mid-run attach discarded as a partial head anchor.",
            "partial head anchor of a mid-run attach. Monotone",
            "partial head anchor the live cursor landed inside is",
            "partial head](Self::head_records_discarded) is in the bag wh",
        ],
    },
    Scanned {
        path: "crates/cerulion_bagd/src/trace_window.rs",
        allowed: &[],
    },
    Scanned {
        path: "crates/cerulion_cli_engine/src/graph_cmd.rs",
        allowed: &[
            "PARTIAL payload, parse the",
            "PARTIAL projection says which direction it is wrong in. \"12",
            "PARTIAL: `fully_removed_names`",
            "`Err` rather than yielding a partial",
            "at k >= 5 deaths … cascade a partial loss into",
            "no rolling window at all: {partial:?}",
            "partial bag (the guard kills + reaps).",
            "partial bag may exist at `{}` (un-finalized",
            "partial derivation would produce",
            "partial loss cascades into an all-crashed failure.",
            "partial loss into all-crashed).",
            "partial map would manufacture a false failure on a graph tha",
            "partial output): missing graph, YAML parse failure,",
            "partial trace. A write",
            "partial\", &over_refs, &[\"drive\"]);",
            "partial-bag path a graceful shutdown",
            "partial.env_json.is_some(), \"{partial:?}\");",
            "partial.graph_yaml.is_some(), \"{partial:?}\");",
            "partial.process_groups = [(\"p0\".to_string(), vec![config.nod",
        ],
    },
    Scanned {
        path: "crates/cerulion_cli_engine/src/replay_state.rs",
        allowed: &["discarded as a mid-run attach's partial head."],
    },
    Scanned {
        path: "crates/cerulion_cli_engine/src/state_arm_attach.rs",
        allowed: &["partial-capture design.", "partial-capture reporting is"],
    },
    Scanned {
        path: "crates/cerulion_core/src/state_ring.rs",
        allowed: &[
            "[`StateAssembler::torn_drains`] counts it. Handing on a partially",
            "assembler is still discarding a partial head anchor.",
            "partial head anchor is DISCARDED, not misreported as",
            "partial head anchor is dropped rather than misreported as co",
            "partial head anchor of a mid-run attach: a record whose `par",
            "partial head anchor this cursor lands inside.",
            "partial record in an inline `[u8; 472]`, encodes onto a",
            "partial record lives inline and each emitted record is a",
            "partially assembled for this anchor is void, and the writer",
            "partially assembled right now.",
            "records were discarded as the partial head anchor.",
            "rolling back would mean cloning partially",
        ],
    },
];

/// Whether a line is PROSE: a comment, or a line carrying a string literal.
fn is_prose(line: &str) -> bool {
    let t = line.trim_start();
    t.starts_with("//") || line.contains('"')
}

/// Whether `line` carries the word, case-insensitively and word-bounded on the
/// left, so `impartial` does not match and `PARTIAL` does.
fn carries_the_word(line: &str) -> bool {
    let lower = line.to_ascii_lowercase();
    let bytes = lower.as_bytes();
    let mut from = 0;
    while let Some(at) = lower[from..].find("partial") {
        let at = from + at;
        let before_is_word =
            at > 0 && (bytes[at - 1].is_ascii_alphanumeric() || bytes[at - 1] == b'_');
        if !before_is_word {
            return true;
        }
        from = at + 1;
    }
    false
}

/// THE REFUSAL HALF: every occurrence of the word is named by its file's allow
/// list, or the scan fails with the file, the line number and the sentence.
#[test]
fn every_use_of_the_word_partial_in_the_anchor_plane_is_on_its_files_allow_list() {
    let root = repo_root();
    let mut unlisted: Vec<String> = Vec::new();
    let mut scanned_files = 0usize;
    for file in REGION {
        let text = std::fs::read_to_string(root.join(file.path))
            .unwrap_or_else(|e| panic!("{} must be readable: {e}", file.path));
        scanned_files += 1;
        for (i, line) in text.lines().enumerate() {
            if !carries_the_word(line) || !is_prose(line) {
                continue;
            }
            if file.allowed.iter().any(|a| line.contains(a)) {
                continue;
            }
            unlisted.push(format!("{}:{}: {}", file.path, i + 1, line.trim()));
        }
    }
    assert_eq!(
        scanned_files,
        REGION.len(),
        "every file in the region must have been read"
    );
    assert!(
        unlisted.is_empty(),
        "PARTIAL is the resim verdict's word. These sentences use it for something else, \
         or are new uses nobody has read yet. Either state the fact instead (every anchor of \
         this run LACKS rank N's records) or add the sentence to this file's allow list:\n{}",
        unlisted.join("\n")
    );
}

/// THE CONTROL HALF: every allow entry matches a line, and the region carries
/// the count this scan was written against.
///
/// Without it the test above passes on an empty region, a renamed file, a read
/// that returned nothing, or an allow list somebody emptied to make a red go
/// away. Each of those reads ZERO unlisted occurrences, which is
/// indistinguishable from a clean tree until the control is asked.
#[test]
fn every_allow_entry_matches_a_line_and_the_region_is_not_empty() {
    let root = repo_root();
    let mut stale: Vec<String> = Vec::new();
    let mut entries = 0usize;
    for file in REGION {
        let text = std::fs::read_to_string(root.join(file.path))
            .unwrap_or_else(|e| panic!("{} must be readable: {e}", file.path));
        for allow in file.allowed {
            entries += 1;
            if !text.lines().any(|l| l.contains(allow)) {
                stale.push(format!("{}: {allow:?}", file.path));
            }
        }
    }
    assert!(
        stale.is_empty(),
        "these allow entries match nothing, so the sentence they were written for was \
         reworded, moved or deleted and the scan is covering less than it claims. Re-read \
         the sentence and update the entry, or remove it:\n{}",
        stale.join("\n")
    );
    assert_eq!(
        entries, ALLOW_LIST_ENTRIES,
        "the allow list is the control: a list that shrank without this number moving is a \
         scan that quietly stopped covering something"
    );
}

/// The allow entries this scan was written against, counted.
///
/// A LITERAL, and the only number in this file. It moves only when a person has
/// read the sentence that moved it, which is what makes it a control rather
/// than a second copy of the list's length.
const ALLOW_LIST_ENTRIES: usize = 87;

/// ORACLE 15, arm (a): the SIX operator-facing sentences say the new fact, in
/// their new wording, naming the rank whose records are LACKING and the exit
/// code a resim of such a capture really returns.
///
/// Asserted as literal text read off the source, because these are the four
/// warns and errors and the two scope sentences an operator actually meets, and
/// the scan above can only say the old WORD is gone. A sentence that dropped
/// the word and also dropped the fact would pass the scan and fail here.
///
/// The rank is what makes each of them actionable: on a deployment with a dozen
/// ranks, "every anchor of this run lacks a rank's records" without saying which
/// rank is a sentence an operator cannot act on, so each site's own rank term is
/// asserted beside the sentence.
///
/// # The CONSEQUENCE half, and what pins it to the behaviour
///
/// These sentences also tell an operator what a resim of such a capture does,
/// and that half was wrong: they named a PARTIAL verdict and an exit 8, neither
/// of which exists. `cerulion bag play --resim` has a 0 to 6 contract, and a
/// capture whose ranks did not all restore is refused as not replay-grade with
/// exit 2, while a capture whose window reaches step 0 needs no anchor and
/// resims. The BEHAVIOUR those two clauses now name is driven end to end by
/// `cerulion_cli_engine`'s own
/// `a_capture_stamped_with_a_missing_rank_is_refused_rather_than_given_a_verdict`,
/// which builds such a capture and reads the exit code back. This arm is the
/// half that keeps the SENTENCES equal to it, and it fails on a message that
/// goes back to naming a code the contract does not have.
#[test]
fn the_six_operator_sentences_name_the_rank_whose_records_are_lacking() {
    let root = repo_root();
    let sites: &[(&str, &[&str])] = &[
        (
            "crates/cerulion_bagd/src/lib.rs",
            &[
                "so every anchor of this run LACKS rank {gap}'s",
                "records. A resim of a capture from this run is refused as not",
                "replay-grade and exits 2 with no verdict, unless that capture's window",
            ],
        ),
        (
            "crates/cerulion_bagd/src/state_coverage.rs",
            &[
                "ranks, so every anchor of this run LACKS that rank's records (see",
                "refused as not replay-grade and exits 2 with no verdict, unless that",
            ],
        ),
        (
            "crates/cerulion_cli_engine/src/graph_cmd.rs",
            &[
                "state ring, so it captures NOTHING and every anchor of this run LACKS",
                "the records of the rank this event names. A resim of a capture from this",
                "run is refused as not replay-grade and exits 2 with no verdict, unless",
            ],
        ),
        (
            "crates/cerulion_cli_engine/src/state_arm_attach.rs",
            &[
                // The `PlaneRole::Worker` scope sentence.
                "run LACKS rank {rank}'s records. A resim of a capture from this run is refused",
                // The ring NAME cannot be derived.
                "derived, so this rank captures NOTHING, and because a graph-wide anchor is",
                // The ring could not be CREATED.
                "created, so this rank captures NOTHING, and because a graph-wide anchor is",
                // Both ring sites carry the consequence and the promise that the
                // graph is not taken down with the plane.
                "not replay-grade and exits 2 with no verdict, unless that capture's window",
                "reaches step 0 and needs no anchor. The graph continues running",
            ],
        ),
    ];
    for (path, needles) in sites {
        let text = std::fs::read_to_string(root.join(path))
            .unwrap_or_else(|e| panic!("{path} must be readable: {e}"));
        for needle in *needles {
            assert!(
                text.contains(needle),
                "{path} must carry the renamed sentence {needle:?}"
            );
        }
    }

    // THE CONTROL, and the half that fails if the rename was reverted rather
    // than reworded: the OLD sentences are gone from every one of these files.
    // Without it every assert above could hold beside a leftover copy. The last
    // two are the CONSEQUENCE half's control: a verdict word and an exit code
    // the resim contract does not have, either of which reaching an operator
    // sends them looking for a failure mode that cannot occur.
    for (path, _) in sites {
        let text = std::fs::read_to_string(root.join(path)).expect("readable");
        for gone in [
            "every anchor of this run is partial",
            "every anchor of this run will be reported partial",
            "every anchor of this run will be reported PARTIAL",
            "reports PARTIAL",
            "exits 8",
        ] {
            assert!(
                !text.contains(gone),
                "{path} still carries the old sentence {gone:?}"
            );
        }
    }
}

/// The ONE sentence deliberately left alone, pinned so a later sweep does not
/// take it.
///
/// "Non-empty makes the recording INCOMPLETE" is the RECORDING-level word and
/// stays the recording-level word. It shares a doc run with a sentence this
/// commit did rewrite, so the rename was sentence-level inside that run, and a
/// run-level rewrite would have taken this one with it.
#[test]
fn the_recording_level_word_incomplete_is_left_alone() {
    let text =
        std::fs::read_to_string(repo_root().join("crates/cerulion_bagd/src/state_coverage.rs"))
            .expect("readable");
    assert!(
        text.contains("/// Non-empty makes the recording INCOMPLETE."),
        "the recording-level sentence must be UNCHANGED"
    );
}
