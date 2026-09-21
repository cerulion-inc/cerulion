// SPDX-License-Identifier: AGPL-3.0-only
//! Every `cerulion_core` test binary carries a row in the internals test map.
//!
//! # Why this is a gate and not a convention
//!
//! `docs/internals/core-testing.md` opens by declaring its own scope — "Every
//! integration test binary under `cerulion_core/tests/`" — and
//! that tree IS the repo's claim about itself. It is
//! what a contributor or an agent reads before touching anything.
//!
//! Unenforced, the claim drifts false, and the omissions can include
//! the file whose own header calls it the KEYSTONE of state capture and restore
//! and the one that states the per-message FIFO delivery contract. A map that
//! silently omits a pin is worse than no map: the reader concludes the
//! behaviour is unpinned and then duplicates the work, or changes the thing the
//! omitted test guards.
//!
//! The drift is also **one-way**. The map has zero stale rows — nobody deletes
//! a row for a file they removed, because removing a test is rare and
//! deliberate. What happens constantly is a new test file landing with no row,
//! which is invisible to every other check. So a hand-list of holes closes
//! exactly once; this walk is what stops the next one.
//!
//! # What it enforces, stated at its real strength
//!
//! PRESENCE, not substance. A row reading "pins behaviour" satisfies this file
//! and buys nothing. That limit is inherent — no test can judge whether prose
//! describes code — and it is why the map's own "Adding a test here" section
//! carries the quality rules this gate cannot. What the gate does buy is that a
//! new binary cannot land UNMENTIONED, which is the failure mode that actually
//! occurs.
//!
//! # The enumeration, and why it needs no fixture waiver
//!
//! The subject set is the depth-1 `*.rs` files under `cerulion_core/tests/` —
//! exactly what cargo builds as test binaries and exactly what
//! `scripts/ci_test_shard.sh` enumerates, so the three agree by construction.
//! The 104 trybuild fixtures (`tests/ui/**`) and the helper `tests/common/mod.rs`
//! are NOT depth-1 files, so they are excluded by the walk's shape rather than
//! by a list that could go stale. [`EXEMPT`] exists for the case the shape
//! cannot cover — a depth-1 file that genuinely should not be documented — and
//! is asserted in BOTH directions so a waiver cannot outlive its reason.
//!
//! # Multi-file rows are real and must be parsed
//!
//! 18 existing rows name 2-4 files in one first cell (`a_test.rs` / `b_test.rs`
//! / `c_test.rs`), because those tests are one subject. A reader that took only
//! the first backticked name would report 27 false holes and teach the next
//! person to split rows that belong together. [`documented`] therefore takes
//! EVERY backticked `*.rs` in the first cell, and
//! [`the_row_reader_handles_every_row_shape_the_map_uses`] drives it against a
//! hand-written fixture — a gate whose own parser has quietly stopped matching
//! looks exactly like a clean tree, which is the failure mode this family of
//! walks exists to prevent.
//!
//! # The IDENTITY of a row is its TARGET PATH, never its base name
//!
//! A row may cross-link a sibling crate, and must then be QUALIFIED
//! (`cerulion_link/tests/loopback_test.rs`). Keying anything on the base name
//! breaks three different ways the moment two crates have a same-named test, and
//! all three are silent:
//!
//! * PRESENCE — a qualified sibling row would satisfy the core binary that shares
//!   its name, so a core file with no row of its own reads as documented.
//! * STALENESS — if the shared base name is a live core binary, the sibling token
//!   rides its coat-tails and a DELETED sibling is never checked.
//! * DUPLICATES — `crate_a/tests/foo.rs` and `crate_b/tests/foo.rs` are two
//!   different files and would be reported as one documented twice.
//!
//! So every token is resolved ONCE, by [`token_target`], into the repo-relative
//! path it means — a bare name means this crate's own test directory, which is
//! the map's declared scope; a path means itself — and all three questions are
//! asked about that target. Presence counts only targets under
//! [`TESTS_DIR`]; staleness asks whether each target exists, independently, with
//! no fast path; duplicates are counted per target.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test doc_inventory_discipline_test
//! ```
//!
//! Pure file parsing — no transport, no SHM, no fixtures — so it is
//! parallel-safe and needs no `#[serial]`.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

/// The map this file gates, relative to the repo root.
const MAP: &str = "docs/internals/core-testing.md";

/// The directory whose depth-1 `*.rs` files cargo builds as test binaries.
const TESTS_DIR: &str = "crates/cerulion_core/tests";

/// A floor on the number of row TOKENS, so a broken reader cannot make the whole
/// file vacuous. Deliberately far below the real number (260-odd at the time of
/// writing — tokens, not rows, because 18 rows name several files): this is a
/// "the parser still works at all" tripwire, not a census that needs updating
/// whenever a test lands.
const MIN_ROWS_EXPECTED: usize = 200;

/// Two-sided exemption inventory: `(file name, why it needs no row)`.
///
/// EMPTY on purpose. Every depth-1 file under `tests/` is a test binary, and
/// every test binary is in this map's declared scope, so there is currently
/// nothing to exempt — the non-binary files (the trybuild fixtures, the helper
/// module) are excluded by the walk's depth rather than by a waiver.
///
/// It exists anyway because the alternative to a declared waiver is a silent
/// one: the next person with a genuine reason would otherwise widen the
/// enumeration, which removes the check for everything else too. Both
/// directions are asserted by
/// [`the_exemption_inventory_is_not_stale`], so a waiver whose file has since
/// been documented, or has been deleted, fails here instead of quietly
/// pre-authorising the next hole.
const EXEMPT: &[(&str, &str)] = &[];

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cerulion_core has a parent directory")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf()
}

/// Depth-1 `*.rs` files under `cerulion_core/tests/` — the set cargo builds as
/// test binaries. Directories (`ui/`, `common/`) are skipped, which is what
/// keeps the trybuild fixtures and the helper module out without a waiver.
fn test_binaries(root: &Path) -> BTreeSet<String> {
    let dir = root.join(TESTS_DIR);
    let mut out = BTreeSet::new();
    for entry in fs::read_dir(&dir).unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display())) {
        let entry = entry.expect("read a directory entry");
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        if path.extension().and_then(|e| e.to_str()) != Some("rs") {
            continue;
        }
        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .expect("a UTF-8 file name")
            .to_string();
        out.insert(name);
    }
    assert!(
        !out.is_empty(),
        "no test binaries found under {}/{TESTS_DIR} — the walk is looking in the wrong place \
         and every assertion below would be vacuous",
        root.display()
    );
    out
}

/// Every `*.rs` token appearing in backticks in the FIRST cell of a table row,
/// VERBATIM and once per row that names it.
///
/// No grouping and no normalisation happen here: the caller resolves each token
/// through [`token_target`], because the base name is not an identity (see the
/// header's IDENTITY section).
///
/// Fenced code blocks are skipped: the map's run recipes contain `cargo test
/// --test <name>` lines that are not rows, and one stray table-shaped line in a
/// shell block would otherwise register as documentation.
fn documented_tokens(map_text: &str) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    let mut in_fence = false;
    for line in map_text.lines() {
        if line.trim_start().starts_with("```") {
            in_fence = !in_fence;
            continue;
        }
        if in_fence {
            continue;
        }
        let trimmed = line.trim_start();
        if !trimmed.starts_with('|') {
            continue;
        }
        // The first cell: everything between the leading `|` and the next one.
        let Some(first_cell) = trimmed[1..].split('|').next() else {
            continue;
        };
        out.extend(backticked_rs_tokens(first_cell));
    }
    out
}

/// The repo-relative path a token MEANS — the token's identity.
///
/// A BARE name means this crate's own test directory, because that is the map's
/// declared scope and a bare name cannot be resolved anywhere else. A token that
/// already carries a path means itself.
fn token_target(token: &str) -> String {
    if token.contains('/') {
        token.to_string()
    } else {
        format!("{TESTS_DIR}/{token}")
    }
}

/// Base names of the tokens that resolve INTO this crate's test directory — the
/// set that answers "does this core binary have a row?".
///
/// A qualified sibling row is deliberately absent from this set even when its
/// base name matches a core binary: it documents a different file.
fn documented_core_names(tokens: &[String]) -> BTreeSet<String> {
    let prefix = format!("{TESTS_DIR}/");
    tokens
        .iter()
        .filter_map(|tok| {
            token_target(tok)
                .strip_prefix(&prefix)
                .filter(|rest| !rest.contains('/'))
                .map(str::to_string)
        })
        .collect()
}

/// Every backticked `*.rs` token in one cell, VERBATIM — the caller derives the
/// base name, so a qualified path survives for [`stale_rows`] to resolve.
fn backticked_rs_tokens(cell: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut rest = cell;
    while let Some(open) = rest.find('`') {
        let after = &rest[open + 1..];
        let Some(close) = after.find('`') else { break };
        let token = &after[..close];
        rest = &after[close + 1..];
        if token.ends_with(".rs") && !token.contains(char::is_whitespace) {
            out.push(token.to_string());
        }
    }
    out
}

fn read_map(root: &Path) -> String {
    let path = root.join(MAP);
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

#[test]
fn every_core_test_binary_has_a_row_in_the_internals_map() {
    let root = repo_root();
    let binaries = test_binaries(&root);
    let documented = documented_core_names(&documented_tokens(&read_map(&root)));
    let exempt: BTreeSet<&str> = EXEMPT.iter().map(|(f, _)| *f).collect();

    let undocumented: Vec<&String> = binaries
        .iter()
        .filter(|b| !documented.contains(b.as_str()) && !exempt.contains(b.as_str()))
        .collect();

    assert!(
        undocumented.is_empty(),
        "{} cerulion_core test binaries have no row in {MAP}: {undocumented:#?}\n\n\
         That file declares its scope as \"Every integration test binary under \
         `cerulion_core/tests/`\", and it is what a contributor \
         or an agent reads first — an omitted pin reads as an ABSENT pin, and the next person \
         duplicates the work or changes the behaviour it guards.\n\n\
         FIX: add one row to the table of the section it belongs to:\n\
         \x20 | `<file>.rs` | <what it pins, one or two sentences from the file's own header> | \
         <parallel | `#[serial]` | tt1 | release | hardware-only> | <`cargo build -p …` fixtures, or —> |\n\n\
         Take the Serial column from the FILE, not from a guess: several headers state \
         \"parallel-safe, no `#[serial]`\" outright, and a `#[serial]` token can appear in a doc \
         comment or a string literal without the file carrying the attribute at all.\n\n\
         If a depth-1 file genuinely should not be documented, add it to EXEMPT in this file \
         WITH THE REASON — an undocumented binary nobody can justify is one somebody will \
         rediscover as a hole.",
        undocumented.len()
    );
}

/// The stale-ROW rule, pure so it can be driven against a hand fixture.
///
/// EVERY token is resolved and checked INDEPENDENTLY. There is deliberately no
/// "the base name is a live core binary, so skip the group" fast path: with one,
/// a deleted `cerulion_link/tests/foo.rs` rides the coat-tails of a live core
/// `foo.rs` and is never checked.
///
/// There is also no file-name-suffix heuristic. One would be exactly as leaky as
/// the names it guesses about — `adaptive_sizing_proptest.rs` is a real binary
/// whose name ends in nothing a guess would list, so a guessing rule stays green
/// when its row outlives it. The rule is [`token_target`] plus existence, which
/// is total.
///
/// `exists` is injected (repo-root-relative) so every arm is testable without
/// planting files in the tree.
fn stale_rows(tokens: &[String], exists: &dyn Fn(&str) -> bool) -> Vec<String> {
    let mut out: Vec<String> = tokens
        .iter()
        .filter(|tok| !exists(&token_target(tok)))
        .cloned()
        .collect();
    out.sort();
    out.dedup();
    out
}

/// Tokens that name the SAME target more than once, with the count.
///
/// Keyed by target, not by base name: two crates' same-named tests are two files
/// and must not read as one documented twice.
fn duplicate_targets(tokens: &[String]) -> Vec<(String, usize)> {
    let mut seen: BTreeMap<String, usize> = BTreeMap::new();
    for tok in tokens {
        *seen.entry(token_target(tok)).or_insert(0) += 1;
    }
    seen.into_iter().filter(|(_, n)| *n > 1).collect()
}

#[test]
fn no_row_names_a_file_that_does_not_exist() {
    let root = repo_root();
    let tokens = documented_tokens(&read_map(&root));
    let ghosts = stale_rows(&tokens, &|rel| root.join(rel).exists());

    assert!(
        ghosts.is_empty(),
        "{MAP} has rows for files that cannot be found: {ghosts:#?}\n\n\
         Either the file was deleted and its row was left behind — delete the row, carrying any \
         breadcrumb the deletion promised — or the row means another crate's file and did not \
         say so.\n\n\
         THE RULE: a row naming a BARE base name means `{TESTS_DIR}/<name>`, because that is \
         this map's declared scope and a bare name cannot be resolved anywhere else. To \
         cross-link a sibling crate's test, write its QUALIFIED path \
         (`cerulion_link/tests/loopback_test.rs`) and it is resolved there instead — including \
         when its base name happens to match one of this crate's own binaries, which is a \
         different file and is checked separately."
    );
}

#[test]
fn the_exemption_inventory_is_not_stale() {
    let root = repo_root();
    let binaries = test_binaries(&root);
    let documented = documented_core_names(&documented_tokens(&read_map(&root)));

    let declared: BTreeSet<&str> = EXEMPT.iter().map(|(f, _)| *f).collect();
    assert_eq!(
        declared.len(),
        EXEMPT.len(),
        "EXEMPT lists a file twice — a duplicate makes the set comparisons pass while the two \
         reasons disagree about why"
    );

    for (file, reason) in EXEMPT {
        assert!(
            !reason.trim().is_empty(),
            "EXEMPT entry `{file}` carries no reason. A waiver without one cannot be reviewed \
             and will outlive whatever justified it"
        );
        assert!(
            binaries.contains(*file),
            "EXEMPT names `{file}`, which is not a depth-1 file under {TESTS_DIR}. Either it was \
             deleted — remove the entry — or it never matched the walk's enumeration at all, in \
             which case the entry pre-authorises a hole it does not even cover"
        );
        assert!(
            !documented.contains(*file),
            "EXEMPT names `{file}`, but {MAP} now HAS a row for it. This is the dangerous \
             direction: the waiver says the file is deliberately undocumented while the map \
             documents it, so the next person to read one of the two is misled.\n\n\
             FIX: delete the EXEMPT entry — the row is the better outcome"
        );
    }
}

#[test]
fn the_row_reader_handles_every_row_shape_the_map_uses() {
    // A hand-written fixture covering each shape the real map contains, because
    // a reader that has quietly stopped matching is indistinguishable from a
    // documented tree.
    let fixture = "\
# Heading

Prose mentioning `not_a_row_test.rs` outside any table.

```bash
# A run recipe, not a row — must NOT count.
cargo test -p cerulion_core --test in_a_fence_test.rs
| `inside_a_fence_test.rs` | looks like a row but is fenced | parallel | — |
```

| Test file | Pins | Serial | Fixtures |
|---|---|---|---|
| `single_test.rs` | One subject. | parallel | — |
| `first_test.rs` / `second_test.rs` / `third_test.rs` | One subject, three files. | `#[serial]` | — |
| `escaped_test.rs` | A cell with an escaped pipe: `block \\| sample(N)`. | tt1 | — |
| `crates/cerulion_core/tests/with_path_test.rs` | Named by full path, not base name. | parallel | — |
| `crates/cerulion_link/tests/single_test.rs` | A SIBLING crate's file whose base name collides. | parallel | — |
| `referrer_test.rs` | Coverage lives in `referenced_elsewhere_test.rs` instead. | parallel | — |
";
    let tokens = documented_tokens(fixture);
    assert_eq!(
        tokens,
        vec![
            "single_test.rs",
            "first_test.rs",
            "second_test.rs",
            "third_test.rs",
            "escaped_test.rs",
            "crates/cerulion_core/tests/with_path_test.rs",
            "crates/cerulion_link/tests/single_test.rs",
            "referrer_test.rs",
        ],
        "the reader must yield exactly the first-cell tokens, VERBATIM and in order.\n\
         A MULTI-FILE row must contribute all of its names (18 real rows name 2-4 files, and \
         taking only the first would report 27 false holes).\n\
         A name in a LATER cell must not count (`referenced_elsewhere_test.rs` is a \
         cross-reference, not documentation of itself).\n\
         A row inside a FENCED block must not count (the map's run recipes are not rows).\n\
         And a qualified token must survive UNREDUCED — its base name is not its identity."
    );

    // The collision is the point: the sibling row names `single_test.rs` too, and
    // must neither document this crate's `single_test.rs` nor read as a duplicate
    // of it.
    let core = documented_core_names(&tokens);
    assert!(
        core.contains("single_test.rs") && core.contains("with_path_test.rs"),
        "a bare row and a `{TESTS_DIR}/`-qualified row both document a core binary; got {core:?}"
    );
    assert!(
        !core.contains("loopback_test.rs"),
        "sanity: a name nothing documents must not appear"
    );
    assert_eq!(
        duplicate_targets(&tokens),
        Vec::<(String, usize)>::new(),
        "`single_test.rs` and `cerulion_link/tests/single_test.rs` are TWO files — keying \
         duplicates on the base name would report them as one documented twice"
    );
}

#[test]
fn a_qualified_sibling_row_is_neither_coverage_nor_a_duplicate_of_a_same_named_core_binary() {
    // The collision class, driven both ways. `foo_test.rs` is a live core binary
    // AND a sibling crate documents its own `foo_test.rs`.
    let sibling_present = "crates/cerulion_link/tests/foo_test.rs".to_string();
    let sibling_missing = "cerulion_gone/tests/foo_test.rs".to_string();

    // (a) PRESENCE: the sibling row must NOT document the core binary.
    let only_sibling = vec![sibling_present.clone()];
    assert!(
        !documented_core_names(&only_sibling).contains("foo_test.rs"),
        "a sibling crate's `foo_test.rs` row documents a DIFFERENT file; letting it satisfy the \
         core binary of the same name is how a core file with no row of its own reads as \
         documented"
    );
    // ...and the bare row for the core binary still does.
    let both = vec!["foo_test.rs".to_string(), sibling_present.clone()];
    assert!(
        documented_core_names(&both).contains("foo_test.rs"),
        "the bare row must still count"
    );

    // (b) STALENESS, the direction a base-name fast path silently skipped: the
    // core file EXISTS, so a grouping rule would pass the whole group and never
    // check the sibling path at all.
    let with_missing = vec!["foo_test.rs".to_string(), sibling_missing.clone()];
    let exists = |rel: &str| rel == format!("{TESTS_DIR}/foo_test.rs") || rel == sibling_present;
    assert_eq!(
        stale_rows(&with_missing, &exists),
        vec![sibling_missing.clone()],
        "a DELETED sibling path must be reported even though a live core binary shares its base \
         name — riding the core file's coat-tails is exactly the hole this arm exists for"
    );
    // The present pair is clean, which is the control that keeps (b) from passing
    // on an always-report rule.
    assert_eq!(
        stale_rows(&both, &exists),
        Vec::<String>::new(),
        "a present core binary and a present sibling link are both fine"
    );

    // (c) DUPLICATES: same base name, different targets — not a duplicate. Two
    // rows for the SAME target are.
    assert_eq!(
        duplicate_targets(&both),
        Vec::<(String, usize)>::new(),
        "`foo_test.rs` and `cerulion_link/tests/foo_test.rs` are two files"
    );
    assert_eq!(
        duplicate_targets(&[
            "foo_test.rs".to_string(),
            format!("{TESTS_DIR}/foo_test.rs"),
        ]),
        vec![(format!("{TESTS_DIR}/foo_test.rs"), 2)],
        "a bare row and a `{TESTS_DIR}/`-qualified row for the SAME file ARE a duplicate — the \
         two spellings resolve to one target"
    );
}

#[test]
fn the_stale_row_rule_is_total_over_every_token() {
    // No suffix heuristic: a bare name that is not a core binary is stale
    // whatever it is called, and a qualified path is resolved at its own path.
    let tokens: Vec<String> = [
        "present_test.rs",
        "crates/cerulion_link/tests/loopback_test.rs",
        "crates/cerulion_link/tests/gone_test.rs",
        "adaptive_sizing_proptest.rs",
    ]
    .into_iter()
    .map(str::to_string)
    .collect();

    let ghosts = stale_rows(&tokens, &|rel| {
        rel == format!("{TESTS_DIR}/present_test.rs")
            || rel == "crates/cerulion_link/tests/loopback_test.rs"
    });

    assert_eq!(
        ghosts,
        vec![
            "adaptive_sizing_proptest.rs".to_string(),
            "crates/cerulion_link/tests/gone_test.rs".to_string(),
        ],
        "the rule must (a) NOT report a qualified sibling link whose file exists — reducing it \
         to its base name and looking only in this crate is what made a real file read as a \
         ghost; (b) report a qualified path whose file is gone; and (c) report a BARE name whose \
         `{TESTS_DIR}/` target is absent, WHATEVER its suffix — `adaptive_sizing_proptest.rs` is \
         a real binary today, so a rule that only looked at `_test.rs`/`_probe.rs`/`_harness.rs` \
         would stay green if its row outlived it"
    );
}

#[test]
fn the_map_reader_is_not_vacuous_on_the_real_file() {
    let root = repo_root();
    let tokens = documented_tokens(&read_map(&root));
    assert!(
        tokens.len() >= MIN_ROWS_EXPECTED,
        "the reader found only {} row tokens in {MAP} (expected at least {MIN_ROWS_EXPECTED}). \
         The file has been restructured, or the reader has broken — either way the \
         every-binary-has-a-row assertion above is no longer meaningful. Fix the reader before \
         lowering this floor.",
        tokens.len()
    );

    let duplicated = duplicate_targets(&tokens);
    assert!(
        duplicated.is_empty(),
        "{MAP} documents the same file in more than one row: {duplicated:#?}\n\n\
         Two rows for one file drift apart, and a reader who finds the stale one has no way to \
         know the other exists. Keep one row; if the file genuinely spans two sections, say so \
         in the row's prose.\n\n\
         NOTE: this is keyed on the TARGET PATH, so two crates' same-named tests are two files \
         and are not reported — but a bare row and a `{TESTS_DIR}/`-qualified row for the same \
         file are."
    );
}
