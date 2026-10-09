// SPDX-License-Identifier: AGPL-3.0-only
//! Every integration test that reads a path OUTSIDE its own crate is PINNED in
//! `.github/workflows/ci.yml`.
//!
//! WHY THIS FILE EXISTS. A CI run that decides which test steps to run from the
//! set of changed paths has exactly one way to be wrong that matters: skipping a
//! test the change could have broken. The population that a
//! `crates/<package>/**` rule cannot see is small and identifiable: the test
//! binaries that open the repository's SHARED trees (`docs`, `tools`,
//! `.github`, `benches`, `examples`) or a markdown file at the repository root.
//! Those binaries fail on an edit to a file their own crate does not contain.
//!
//! So the membership of that population is DERIVED here rather than listed. A
//! hand list of "the tests that read the documentation tree" is stale the moment
//! a test adds a path, and nothing tells anybody. This walk reads every
//! workspace member's `tests/*.rs` AND every `src/**/*.rs`, extracts the string
//! literals its CODE carries, keeps the ones that name a shared tree or a root
//! markdown file, and turns each hit into a `(package, test binary) -> roots`
//! pin. `src/` is in the walk because a `#[cfg(test)]` unit test opens a shared
//! tree exactly as an integration test does and was invisible here: two of them
//! in this crate alone. Those reads all compile into the ONE library test
//! binary, so they are pooled and attributed to a binary named after the
//! package (`<package>::<package>`), the name the step that runs them carries.
//! Every pin must appear in `.github/workflows/ci.yml` as a marker line beside
//! the step that runs that package:
//!
//! ```text
//!       # doc-pin: <package>::<test binary> reads <root>, <root>
//! ```
//!
//! The marker is a YAML comment, so it changes no behaviour and the coverage
//! walk in `ci_test_coverage_test` (which strips comments before it scans)
//! cannot be satisfied by one.
//!
//! TWO-SIDED SET EQUALITY, deliberately. A derived pin with no marker FAILS: a
//! path-to-step map will have no way to know the binary must run. A marker with
//! no derived pin ALSO fails, as stale: a renamed or deleted test binary leaves
//! exactly that shape behind, and a marker that describes nothing silently
//! pre-authorises the next hole.
//!
//! # What counts as a path
//!
//! A literal has to pass BOTH halves: it has to LOOK like a shared-root path,
//! and it has to be USED as one.
//!
//! THE SHAPE. A literal is rooted at a shared tree when, after any leading run
//! of `./` and `../` segments is stripped, it opens with one of the five
//! directory names followed by `/`; or when the WHOLE literal is one of the
//! root markdown file names. Three rules keep that from drowning in false hits,
//! each one measured against the tree:
//!
//!   * a literal carrying WHITESPACE is prose, not a path. Assertion messages in
//!     this workspace routinely open with a path and continue into a sentence
//!     (`"docs/…​.md cites a 500 ms deadman window; update the doc if …"`), and a
//!     message is not a read;
//!   * a directory root needs the separator. A bare `"docs"` in the tree is a
//!     JSON key, a step marker or a directory NAME, never a read. `"benches/"`
//!     is rooted and `"benches"` is not;
//!   * a root markdown name counts only as the whole literal. `"msg/README.md"`
//!     names a file inside a crate.
//!
//! THE USE, and why looking like a path was never enough. This workspace is full
//! of tests that NAME a shared path in order to assert about it: one checks the
//! wording of a refusal that mentions a documentation file, another checks that
//! a walk stayed OUT of a tree by testing its output for that tree's name.
//! Neither opens anything, and a pin derived from one records a read that does
//! not happen. So a rooted literal counts only where the surrounding CODE uses
//! it as a path:
//!
//!   * it is the argument of `Path::new(`, `PathBuf::from(`, `.join(`,
//!     `read_to_string(`, `read_dir(`, `File::open(`, `include_str!(`,
//!     `include_bytes!(`, or any `fs::` call, optionally inside a `format!(` or
//!     behind a `&`; or
//!   * it sits in a `const` or `static` item whose name is used elsewhere in the
//!     same source. Path tables are the ordinary shape here: a const holding
//!     the path, joined onto the repository root somewhere else, and the walk
//!     does not trace where the use leads: following a name into a crate-local
//!     helper is the data-flow analysis this file declines to do.
//!
//! Two uses that pass both halves are still not reads, and each one had a row
//! recording a read that does not happen:
//!
//!   * a path the code WRITES. `completions_test` writes a `README.md` fixture
//!     into a temporary directory to prove the completion skips a FILE under
//!     `nodes/`; the repository's own README is never opened. The destination
//!     of a write is the first argument of a write call, so the walk reads
//!     outward past the path openers the literal is spelled inside and asks
//!     what the literal is an argument OF;
//!   * a path joined onto the crate's OWN directory. `cerulion_wsd`'s
//!     `doc_deference_test` opens its crate's `AGENTS.md`, which is
//!     `crates/cerulion_wsd/AGENTS.md` and not the repository root's file of
//!     that name. A literal that does not climb out with `../` names something
//!     inside the crate, so it counts only where the receiver is not the
//!     crate's manifest directory, resolved one hop, through a binding in the
//!     same source, the same single hop the const rule takes.
//!
//! The code around a literal is read from a VIEW of the source with every
//! comment body and every literal body blanked to spaces, so a call spelled
//! inside a comment or inside another literal vouches for nothing.
//!
//! Comments are stripped and STRING LITERALS ARE MODELLED IN THE SAME PASS, not
//! by a comment-only pre-pass. A pre-pass that does not know about literals cuts
//! a line at the `//` of a URL inside one and takes every path literal after it
//! on that line with it: a silent hole, in the one direction this file exists
//! to close. Modelling literals is also what lets this file spell `/*` and `*/`
//! plainly: the walk reads THIS source too, and a literal here is data, never a
//! comment opener.
//!
//! A raw string is ONE literal, whitespace and all. That is why a manifest
//! fixture written as `r#"include = ["src/", "README.md"]"#` is not a read of
//! the root markdown file: the crate under test writes that text into a scratch
//! directory.
//!
//! # Limits
//!
//! A path assembled at run time is invisible here, and so is one reached through
//! a helper in the crate's library. Both are stated limits, not oversights: the
//! analysis that would see them is a data-flow analysis, and a wrong answer from
//! one is a demand no maintainer can discharge.
//!
//! The known miss in the tree today is `cerulion_core::serial_discipline_test`.
//! It opens the shell scripts a workflow line names, but the only spelling of
//! that tree in its source is the prefix it matches those lines against, and the
//! path it finally opens is built from the workflow text at run time. Its
//! `.github` read is pinned; the shell tree it reaches from there is not. The
//! same shape in a new test is a miss in the same direction, and the answer is
//! to name the directory it opens in a `const` the test joins, not to widen the
//! rule until an assertion message counts as a read.
//!
//! In the other direction the pin is deliberately over-inclusive: a literal that
//! merely LOOKS like a path and is handed to one of the calls above costs one
//! extra test step, while a missed one costs a silent skip.
//!
//! A `src/` read is attributed to the library test binary as a whole, so the
//! marker names the package twice and says nothing about WHICH unit test opens
//! the root. That is the finest name the workflow carries: the step that runs
//! them is `cargo test -p <package>`.
//!
//! A name bound to the crate's own directory is recognised by NAME across the
//! whole file, so a name bound to it in one function suppresses a same-named
//! binding in another.
//!
//! A package that carried an integration test binary named exactly like the
//! package would have its pin merged with the library pin under one key.
//!
//! # What this pin covers, and what it does not
//!
//! It covers the SHARED trees: a test binary that opens `docs`, `tools`,
//! `.github`, `benches`, `examples` or a root markdown file. That is one of the
//! ways a test observes something no `crates/<package>/**` rule attributes to
//! it. The other is a cargo dependency edge, and
//! `tools/scripts/ci_selected_packages.py` closes over those: normal, build
//! and dev.
//!
//! Neither covers a test that reaches into ANOTHER crate's tree through a
//! source path literal, a walk over the whole repository, or a `dlopen` of an
//! artifact built from another package. Those are an OPEN class, and they have
//! to be pinned before any CI step is gated on the selection.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

/// The shared directory trees a test may read from outside its own crate.
///
/// Spelled without a trailing separator because that is how a marker names them;
/// [`doc_root_of`] requires the separator on the LITERAL it is matching.
const DOC_ROOT_DIRS: &[&str] = &[".github", "benches", "docs", "examples", "tools"];

/// The root markdown files, by STEM.
///
/// Exactly the four `git ls-files` reports for `*.md` with no directory in the
/// path. A name that is NOT at the repository root has no business here: it
/// pre-authorises a pin no test can produce, and it produced one: a
/// documentation file named only in a refusal's wording was read as a read.
///
/// The extension is appended rather than spelled because the walk below reads
/// THIS file as well, and a bare `"README.md"` literal here would pin this
/// binary to a file it never opens. A stem carries no separator and no
/// extension, so it matches nothing.
const ROOT_MARKDOWN_STEMS: &[&str] = &["AGENTS", "CHANGELOG", "CLAUDE", "README"];

/// The extension every [`ROOT_MARKDOWN_STEMS`] entry carries on disk.
const MARKDOWN_EXT: &str = ".md";

/// The fixed opening of a marker line, comment character included.
const MARKER_PREFIX: &str = "# doc-pin:";

/// The word between the binary and its roots in a marker line.
const MARKER_VERB: &str = " reads ";

/// The hand scan of the tree this walk is checked against.
///
/// Each row is `(package, test binary, directory roots, root markdown stems)`,
/// and every row was produced by OPENING the named source and reading the call
/// that takes the literal, not by running the walk and copying what it said.
/// Five rows earlier revisions carried recorded reads that do not happen at
/// all: a refusal's wording, a tree a walk asserts it stayed OUT of, a set of
/// file names asserted to be ABSENT from a package, a root markdown file a test
/// WRITES into a temporary directory, and a crate's own copy of one. Those rows
/// are gone rather than tolerated, and the classifier refuses the last two
/// shapes outright.
///
/// So the rows say what a reader of the sources determines, which is the only
/// thing that makes this an outside opinion: the walk cannot move a row by
/// changing its mind. What the rows do NOT claim is completeness against every
/// path a test opens, a path the walk is documented not to see (module docs,
/// "Limits") has no row either, because a row the walk cannot reproduce fails
/// this arm without saying anything true about the tree.
///
/// The rows are compared to the derived pins as a whole MAP against a whole
/// map, after asserting the rows name each binary once: a binary that gains or
/// loses a root fails here, a derived pin with no row fails here, and a
/// duplicated row cannot hide either of them behind a length that still adds up.
#[allow(clippy::type_complexity)]
const HAND_SCANNED_DOC_PINS: &[(&str, &str, &[&str], &[&str])] = &[
    // The user-facing reference page, joined onto the repository root.
    (
        "cerulion_cli",
        "graph_run_validate_gate_e2e_test",
        &["docs"],
        &[],
    ),
    // The bag reference page, reached from the crate manifest directory.
    (
        "cerulion_cli_engine",
        "bag_record_run_attach_test",
        &["docs"],
        &[],
    ),
    // The crate's own unit tests, which cargo compiles into ONE library test
    // binary: `src/auth.rs` joins the login-seed script under the tools tree,
    // and `src/system_deps.rs` joins the example node manifest.
    (
        "cerulion_cli_engine",
        "cerulion_cli_engine",
        &["examples", "tools"],
        &[],
    ),
    // This walk's own read of the workflow it gates, and of the committed
    // observation-edge table it holds to the sources.
    (
        "cerulion_cli_engine",
        "ci_doc_pin_walk_test",
        &[".github", "tools"],
        &[],
    ),
    // The workflow directory, and the shard runner it names.
    (
        "cerulion_cli_engine",
        "ci_test_coverage_test",
        &[".github", "tools"],
        &[],
    ),
    // The robot example workspace (a checked-workspace entry that becomes a
    // metadata invocation), and the dependency-ban configuration.
    (
        "cerulion_cli_engine",
        "dependency_door_test",
        &["examples", "tools"],
        &[],
    ),
    // The mirrored-workspace table, expanded into member manifests.
    (
        "cerulion_cli_engine",
        "library_print_ban_test",
        &["examples"],
        &[],
    ),
    // The bridge node's sources, reached from the crate manifest directory.
    ("cerulion_cli_engine", "ros_attach_test", &["examples"], &[]),
    // The user-facing reference page, reached from the manifest directory.
    (
        "cerulion_cli_engine",
        "user_api_inspector_doc_test",
        &["docs"],
        &[],
    ),
    // The mirrored-workspace table, expanded into member manifests.
    (
        "cerulion_cli_engine",
        "workspace_lints_manifest_test",
        &["examples"],
        &[],
    ),
    // The test map it gates, held in a const and joined onto the root.
    (
        "cerulion_core",
        "doc_inventory_discipline_test",
        &["docs"],
        &[],
    ),
    // The workflow directory. It also opens shell scripts under the tools
    // tree, but builds those paths from the workflow text at run time, the
    // miss the module docs name.
    ("cerulion_core", "serial_discipline_test", &[".github"], &[]),
    // The license texts the crates copy.
    (
        "cerulion_hygiene",
        "crate_license_texts_test",
        &["docs"],
        &[],
    ),
    // The robot example workspace, joined onto the root twice.
    (
        "cerulion_hygiene",
        "dependency_rules_test",
        &["examples"],
        &[],
    ),
    // Two const tables of files, each entry joined onto the root and read,
    // plus the user-facing reference page.
    (
        "cerulion_hygiene",
        "shipped_surface_structure_test",
        &[".github", "benches", "docs", "tools"],
        &[],
    ),
    // The user-facing reference page.
    ("cerulion_hygiene", "user_api_doc_test", &["docs"], &[]),
    // The refresh script, reached from the crate manifest directory.
    (
        "native_ros2_messages",
        "upstream_drift_test",
        &["tools"],
        &[],
    ),
    // The per-distro gate table, joined onto the repository root: the test
    // parses its rows and holds the absent-export derivation to them.
    (
        "rmw_cerulion",
        "rmw_absent_export_table_test",
        &["tools"],
        &[],
    ),
];

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cerulion_cli_engine has a parent")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf()
}

/// `<stem>` plus the markdown extension: the name a marker carries.
fn root_markdown_name(stem: &str) -> String {
    format!("{stem}{MARKDOWN_EXT}")
}

// ---------------------------------------------------------------------------
// The literal extractor.
// ---------------------------------------------------------------------------

/// A string literal the code carries, and where its opening token starts.
struct CodeLiteral {
    /// The literal's contents, escapes undecoded.
    text: String,
    /// The byte offset of the literal's first byte, prefix (`b`, `c`, `r#`)
    /// included, so the code BEFORE it can be read.
    at: usize,
}

/// Blank `view[from..to]` to spaces, keeping newlines and byte offsets.
///
/// Every replaced byte becomes an ASCII space, so a multi-byte character turns
/// into as many spaces as it had bytes and the view stays valid UTF-8 and
/// exactly as long as the source.
fn blank(view: &mut [u8], from: usize, to: usize) {
    for byte in &mut view[from..to] {
        if *byte != b'\n' {
            *byte = b' ';
        }
    }
}

/// The two VIEWS of one source, and every string literal its code carries.
struct SourceViews {
    /// `src` with every comment body AND every literal body blanked to spaces.
    code: String,
    /// `src` with every comment body blanked and literal bodies KEPT.
    ///
    /// The crate-directory rule has to read `env!("CARGO_MANIFEST_DIR")`, and
    /// that name is a literal: in the blanked view it is spaces.
    code_with_literals: String,
    /// Every string literal the CODE carries, in file order.
    literals: Vec<CodeLiteral>,
}

/// The CODE view of `src`, and every STRING LITERAL its code carries.
///
/// The view is `src` with every comment and every literal BODY blanked to
/// spaces. Blanking rather than deleting is what makes a literal's offset
/// usable: `code[..literal.at]` is the code that precedes the literal, and no
/// path spelled inside a comment or inside another literal can be read as the
/// call around it.
///
/// One pass over the source, tracking four states, because they interleave:
/// line comments, block comments (which NEST in Rust), character literals and
/// string literals. A character literal has to be recognised so `'"'` does not
/// open a string, and a lifetime (`'a`) has to be distinguished from one.
///
/// Every literal form the workspace uses is read: `"…"`, the `b`/`c` prefixed
/// forms, and raw strings with any number of hashes (`r"…"`, `br#"…"#`). A raw
/// string yields its contents as ONE literal, which is the point: a manifest or
/// workflow fixture embedded as a raw string is a document the test WRITES, not
/// a set of paths it reads.
///
/// Escapes are not decoded. A path literal carries none, and decoding them would
/// invent bytes the source does not have.
///
/// Fail-closed at the edges: an unterminated literal or block comment consumes
/// the rest of the file, so the pins its tail would have produced go missing and
/// the workflow marker for them reads as stale: a red, never a quiet pass.
fn scan_code(src: &str) -> SourceViews {
    let bytes = src.as_bytes();
    let mut view = bytes.to_vec();
    let mut kept = bytes.to_vec();
    let mut out = Vec::new();
    let mut i = 0usize;
    let mut block_depth = 0usize;

    while i < bytes.len() {
        if block_depth > 0 {
            if bytes[i..].starts_with(b"/*") {
                block_depth += 1;
                blank(&mut view, i, i + 2);
                blank(&mut kept, i, i + 2);
                i += 2;
            } else if bytes[i..].starts_with(b"*/") {
                block_depth -= 1;
                blank(&mut view, i, i + 2);
                blank(&mut kept, i, i + 2);
                i += 2;
            } else {
                blank(&mut view, i, i + 1);
                blank(&mut kept, i, i + 1);
                i += 1;
            }
            continue;
        }
        if bytes[i..].starts_with(b"//") {
            let end = match src[i..].find('\n') {
                Some(nl) => i + nl,
                None => bytes.len(),
            };
            blank(&mut view, i, end);
            blank(&mut kept, i, end);
            i = end;
            continue;
        }
        if bytes[i..].starts_with(b"/*") {
            block_depth = 1;
            blank(&mut view, i, i + 2);
            blank(&mut kept, i, i + 2);
            i += 2;
            continue;
        }
        if bytes[i] == b'\'' {
            i = end_of_char_or_lifetime(bytes, i);
            continue;
        }
        if let Some(literal) = read_string_literal(src, i) {
            blank(&mut view, literal.body_start, literal.body_end);
            out.push(CodeLiteral {
                text: literal.text,
                at: i,
            });
            i = literal.next;
            continue;
        }
        i += 1;
    }
    let expect = "blanking writes ASCII spaces, so the view stays valid UTF-8";
    SourceViews {
        code: String::from_utf8(view).expect(expect),
        code_with_literals: String::from_utf8(kept).expect(expect),
        literals: out,
    }
}

/// One past a `'`-introduced token: a character literal, or a lifetime.
///
/// `'a'`, `'\n'` and `'\u{2a}'` are literals; `'a` and `'static` are lifetimes.
/// The distinguishing feature is the CLOSING quote, so an escaped literal is
/// scanned to it and a bare one is recognised by the quote two bytes on. Getting
/// this wrong in the lifetime direction would open a string at the next `"` and
/// swallow the file.
fn end_of_char_or_lifetime(bytes: &[u8], at: usize) -> usize {
    if bytes[at..].starts_with(b"'\\") {
        // Step over the backslash AND the character it escapes before looking
        // for the closing quote, or `'\''` ends on its own escaped quote and
        // the scan walks into the code after it one byte out of phase.
        let mut j = (at + 3).min(bytes.len());
        while j < bytes.len() && bytes[j] != b'\'' {
            j += 1;
        }
        return (j + 1).min(bytes.len());
    }
    // A one-character literal: the quote sits two bytes on. Anything else is a
    // lifetime, whose name ends where the identifier ends.
    if at + 2 < bytes.len() && bytes[at + 2] == b'\'' {
        return at + 3;
    }
    at + 1
}

/// One string literal, as [`read_string_literal`] reports it.
struct ReadLiteral {
    /// The literal's contents, escapes undecoded.
    text: String,
    /// One past the closing delimiter.
    next: usize,
    /// The byte range of the CONTENTS, delimiters excluded.
    body_start: usize,
    body_end: usize,
}

/// The string literal starting at `at`, and one past its closing delimiter.
///
/// Returns `None` when `at` does not open one. The `b` and `c` prefixes and any
/// number of raw-string hashes are accepted; the prefix is consumed and the
/// CONTENTS are returned.
fn read_string_literal(src: &str, at: usize) -> Option<ReadLiteral> {
    let bytes = src.as_bytes();
    // A prefix letter is a prefix only at the start of a token: the `b` of
    // `verb` is not a byte-string opener, and reading it as one would consume
    // the rest of the line.
    if at > 0 && bytes[at] != b'"' {
        let prev = bytes[at - 1];
        if prev.is_ascii_alphanumeric() || prev == b'_' {
            return None;
        }
    }
    let mut i = at;
    if matches!(bytes.get(i), Some(b'b' | b'c')) {
        i += 1;
    }
    if bytes.get(i) == Some(&b'r') {
        i += 1;
        let hash_start = i;
        while bytes.get(i) == Some(&b'#') {
            i += 1;
        }
        if bytes.get(i) != Some(&b'"') {
            return None;
        }
        let hashes = i - hash_start;
        let body_start = i + 1;
        let mut close = String::with_capacity(hashes + 1);
        close.push('"');
        for _ in 0..hashes {
            close.push('#');
        }
        let end = src[body_start..].find(&close).map(|o| body_start + o);
        return match end {
            Some(end) => Some(ReadLiteral {
                text: src[body_start..end].to_string(),
                next: end + close.len(),
                body_start,
                body_end: end,
            }),
            // Unterminated: consume the rest, which loses the tail's pins and
            // reds the gate rather than passing on text no compiler accepts.
            None => Some(ReadLiteral {
                text: src[body_start..].to_string(),
                next: src.len(),
                body_start,
                body_end: src.len(),
            }),
        };
    }
    if bytes.get(i) != Some(&b'"') {
        return None;
    }
    let body_start = i + 1;
    let mut j = body_start;
    let mut body = String::new();
    while j < bytes.len() {
        match bytes[j] {
            b'\\' => {
                // Keep the escape as written; only the delimiter search cares.
                body.push('\\');
                match src[j + 1..].chars().next() {
                    Some(ch) => {
                        body.push(ch);
                        j += 1 + ch.len_utf8();
                    }
                    None => j += 1,
                }
            }
            b'"' => {
                return Some(ReadLiteral {
                    text: body,
                    next: j + 1,
                    body_start,
                    body_end: j,
                })
            }
            _ => {
                let ch = src[j..].chars().next()?;
                body.push(ch);
                j += ch.len_utf8();
            }
        }
    }
    Some(ReadLiteral {
        text: body,
        next: bytes.len(),
        body_start,
        body_end: bytes.len(),
    })
}

// ---------------------------------------------------------------------------
// Is the literal USED as a path?
// ---------------------------------------------------------------------------

/// The call forms whose argument this walk reads as a path.
///
/// Spelled WITH the opening parenthesis and matched against the code that
/// PRECEDES a literal, so a name that merely appears nearby vouches for
/// nothing. Filesystem calls reached through the `fs` module are matched
/// separately, by module rather than by name: the module is the part that says
/// this touches the disk.
const PATH_OPENERS: &[&str] = &[
    "Path::new(",
    "PathBuf::from(",
    ".join(",
    "read_to_string(",
    "read_dir(",
    "File::open(",
    "include_str!(",
    "include_bytes!(",
];

/// Forms a path may sit inside without ceasing to be one.
///
/// `format!(` is the interpolated path and `&` is the borrow a `&Path`
/// argument takes. Both are stripped and the code before them is asked again,
/// so a `format!` that is NOT inside one of [`PATH_OPENERS`], an assertion
/// message, say, still answers no.
const PATH_WRAPPERS: &[&str] = &["format!(", "&"];

/// The module every filesystem call this walk credits is reached through.
const FS_MODULE: &str = "fs::";

/// The call NAMES [`PATH_OPENERS`] spells, without the parenthesis and without
/// a receiver's dot: what [`enclosing_calls`] reports.
const PATH_OPENER_NAMES: &[&str] = &[
    "Path::new",
    "PathBuf::from",
    "join",
    "read_to_string",
    "read_dir",
    "File::open",
    "include_str!",
    "include_bytes!",
];

/// Calls whose FIRST argument is a path they WRITE.
///
/// Matched on the last `::` segment, so `write`, `fs::write` and
/// `std::fs::write` are one entry. A write is not a read: `completions_test`
/// writes a `README.md` fixture into a temporary directory, and reading that as
/// a read pinned the binary to a repository file it never opens. Calls whose
/// SECOND argument is the destination (`copy`, `rename`) are deliberately
/// absent: crediting their source argument is the over-inclusive direction,
/// which costs one test step rather than a silent skip.
const WRITE_CALLS: &[&str] = &[
    "write",
    "create",
    "create_new",
    "create_dir",
    "create_dir_all",
    "remove_file",
    "remove_dir",
    "remove_dir_all",
    "set_permissions",
];

/// The environment variable that names the crate's own directory.
const CRATE_DIR_ENV: &str = "CARGO_MANIFEST_DIR";

/// Does `code_before` (the code immediately preceding a literal) use it as a
/// path?
///
/// The rule, and why it is not "looks like a path", are in the module docs.
/// This reads only the text to the LEFT of the literal, which is what keeps it
/// a one-pass syntactic rule rather than a data-flow analysis.
fn is_path_position(code_before: &str) -> bool {
    let mut head = code_before.trim_end();
    loop {
        if PATH_OPENERS.iter().any(|opener| head.ends_with(opener)) {
            return true;
        }
        if let Some(before) = head.strip_suffix('(') {
            // The call NAME is the identifier run before the parenthesis; what
            // sits before that run is the path the call was reached through.
            let name_at = before
                .char_indices()
                .rev()
                .find(|(_, ch)| !(ch.is_ascii_alphanumeric() || *ch == '_'))
                .map_or(0, |(i, ch)| i + ch.len_utf8());
            if before[..name_at].ends_with(FS_MODULE) {
                return true;
            }
        }
        let Some(wrapper) = PATH_WRAPPERS.iter().find(|w| head.ends_with(**w)) else {
            return false;
        };
        head = head[..head.len() - wrapper.len()].trim_end();
    }
}

/// The calls that enclose a literal, innermost first: the call's NAME and
/// whether the literal sits in that call's FIRST argument.
///
/// Read right-to-left over the blanked code view, so a parenthesis inside a
/// comment or another literal is a space. It stops at the statement boundary
/// and after a handful of calls: the question is what the literal is an
/// argument OF, not what the whole expression eventually does.
fn enclosing_calls(code_before: &str) -> Vec<(String, bool)> {
    let bytes = code_before.as_bytes();
    let mut out = Vec::new();
    let mut depth = 0i32;
    let mut saw_comma = false;
    let mut i = code_before.len();
    while i > 0 && out.len() < 8 {
        i -= 1;
        match bytes[i] {
            b')' | b']' | b'}' => depth += 1,
            b'(' if depth == 0 => {
                out.push((call_name_before(code_before, i), !saw_comma));
                saw_comma = false;
            }
            b'[' | b'{' if depth == 0 => saw_comma = false,
            b'(' | b'[' | b'{' => depth -= 1,
            b',' if depth == 0 => saw_comma = true,
            b';' if depth == 0 => break,
            _ => {}
        }
    }
    out
}

/// The name of the call whose opening parenthesis sits at `at`.
///
/// The identifier run before the parenthesis, a `!` for a macro, and any
/// `::`-qualified path in front of it. A method's receiver dot is NOT part of
/// the name: `nodes.join(` is `join`.
fn call_name_before(code: &str, at: usize) -> String {
    let head = &code[..at];
    let bang = head.ends_with('!');
    let head = if bang { &head[..head.len() - 1] } else { head };
    let ident_start = |text: &str| {
        text.char_indices()
            .rev()
            .find(|(_, ch)| !(ch.is_ascii_alphanumeric() || *ch == '_'))
            .map_or(0, |(i, ch)| i + ch.len_utf8())
    };
    let mut start = ident_start(head);
    while let Some(rest) = head[..start].strip_suffix("::") {
        let segment = ident_start(rest);
        if segment == rest.len() {
            break;
        }
        start = segment;
    }
    let mut name = head[start..].to_string();
    if bang {
        name.push('!');
    }
    name
}

/// Is the literal the DESTINATION of a write?
///
/// The enclosing calls are read outward past the path openers and wrappers the
/// literal may be spelled inside; the first call that is neither decides. A
/// read nested in a write's LATER argument
/// (`fs::write(&out, fs::read_to_string(root.join(P))?)`) is still a read: the
/// destination is the FIRST argument, and that is what this asks about.
fn is_write_target(code_before: &str) -> bool {
    for (name, first_argument) in enclosing_calls(code_before) {
        let last = name.rsplit("::").next().unwrap_or("");
        if WRITE_CALLS.contains(&last) {
            return first_argument;
        }
        let is_opener = PATH_OPENER_NAMES.contains(&name.as_str())
            || name.starts_with(FS_MODULE)
            || name.contains(&format!("::{FS_MODULE}"))
            || name == "format!"
            || name.is_empty();
        if !is_opener {
            return false;
        }
    }
    false
}

/// The two constructors that turn the manifest-directory string into a path.
const CRATE_DIR_CONSTRUCTORS: &[&str] = &["Path::new", "PathBuf::from"];

/// The one further call a crate-directory expression may carry: it COPIES the
/// path, it does not move it.
const TO_PATH_BUF: &str = ".to_path_buf()";

/// The constructor's argument, with its literal blanked and its whitespace
/// removed.
const CRATE_DIR_ARGUMENT: &str = "(env!(\"\"))";

/// Is `code[start..end]` EXACTLY the crate's own manifest directory?
///
/// `Path::new(env!("CARGO_MANIFEST_DIR"))` or
/// `PathBuf::from(env!("CARGO_MANIFEST_DIR"))`, optionally through one
/// [`TO_PATH_BUF`], and nothing else. A substring test for `parent(` was the
/// whole rule before, and a source climbs out of its crate four ways:
/// `parent()`, `pop()`, `join("..")`, `join("../..")`, so an expression
/// carrying ANY other call answers NO here. That is the over-approximating
/// side, deliberately: a receiver this reader cannot place is not the crate
/// directory, so a shared-root literal joined onto it is COUNTED, and an extra
/// pin costs a test step while a missing one is a silent skip.
///
/// Both views, for the reason everything here takes both: the SHAPE is read
/// from the blanked code, so a call spelled inside a message is not a call, and
/// the environment variable's name is then read from the same byte range of the
/// view that kept its literals.
fn is_crate_directory_expression(
    code: &str,
    code_with_literals: &str,
    start: usize,
    end: usize,
) -> bool {
    let text = &code[start..end];
    let start = start + (text.len() - text.trim_start().len());
    let end = start + code[start..end].trim_end().len();
    let end = match code[start..end].strip_suffix(TO_PATH_BUF) {
        Some(receiver) => start + receiver.trim_end().len(),
        None => end,
    };
    let expr = &code[start..end];
    let Some(open) = expr.find('(') else {
        return false;
    };
    let callee = &expr[..open];
    let is_constructor = CRATE_DIR_CONSTRUCTORS
        .iter()
        .any(|name| callee == *name || callee.ends_with(&format!("::{name}")));
    let argument: String = expr[open..]
        .chars()
        .filter(|ch| !ch.is_whitespace())
        .collect();
    is_constructor
        && argument == CRATE_DIR_ARGUMENT
        && code_with_literals[start..end].contains(CRATE_DIR_ENV)
}

/// Where the call that ENDS at `end` begins: the `(` matching its final `)`,
/// walked back over the callee path in front of it.
///
/// `None` when `code[..end]` does not end in a call at all. The parenthesis
/// walk runs over the blanked view, so a parenthesis inside a literal closes
/// nothing.
fn call_expression_start(code: &str, end: usize) -> Option<usize> {
    let head = &code[..end];
    if !head.ends_with(')') {
        return None;
    }
    let mut depth = 0i32;
    for (i, ch) in head.char_indices().rev() {
        match ch {
            ')' => depth += 1,
            '(' => {
                depth -= 1;
                if depth == 0 {
                    return Some(
                        head[..i]
                            .char_indices()
                            .rev()
                            .find(|(_, c)| !(c.is_ascii_alphanumeric() || *c == '_' || *c == ':'))
                            .map_or(0, |(j, c)| j + c.len_utf8()),
                    );
                }
            }
            _ => {}
        }
    }
    None
}

/// Does the source climb out of `name` by MUTATING it in place?
///
/// `PathBuf::pop` is the one path mutation that climbs, and it is a statement
/// of its own rather than part of the binding, so the initialiser rule cannot
/// see it. Read on the blanked view at a token boundary, so `root.pop(` inside
/// a message pops nothing and `newroot.pop(` is not `root`.
fn is_popped(code: &str, name: &str) -> bool {
    let needle = format!("{name}.pop(");
    let mut from = 0usize;
    while let Some(offset) = code[from..].find(&needle) {
        let at = from + offset;
        from = at + needle.len();
        let opens_a_token = code[..at]
            .chars()
            .next_back()
            .is_none_or(|ch| !(ch.is_ascii_alphanumeric() || ch == '_'));
        if opens_a_token {
            return true;
        }
    }
    false
}

/// Names this source binds to the crate's OWN directory.
///
/// A `let`, `const` or `static` whose initialiser is EXACTLY the crate's
/// manifest directory by [`is_crate_directory_expression`], and whose name the
/// source never pops. ONE hop, and no further: a name assigned from another
/// name is not followed, which is the same single hop the const-table rule
/// takes.
///
/// The STRUCTURE (where a binding starts and where its `;` is) is read from
/// the blanked view, so a keyword or a semicolon inside a literal starts and
/// ends nothing; the CONTENT is then read from the same byte range of the view
/// that kept its literals, because the name it is looking for is one.
fn crate_directory_bindings(code: &str, code_with_literals: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let name_of = |text: &str| -> String {
        text.chars()
            .take_while(|ch| ch.is_ascii_alphanumeric() || *ch == '_')
            .collect()
    };
    for keyword in ["let ", "const ", "static "] {
        let mut from = 0usize;
        while let Some(offset) = code[from..].find(keyword) {
            let at = from + offset;
            from = at + keyword.len();
            let opens_a_token = code[..at]
                .chars()
                .next_back()
                .is_none_or(|ch| !(ch.is_ascii_alphanumeric() || ch == '_'));
            if !opens_a_token {
                continue;
            }
            let mut start = from;
            let skip_blanks = |text: &str| text.len() - text.trim_start().len();
            start += skip_blanks(&code[start..]);
            if let Some(rest) = code[start..].strip_prefix("mut ") {
                start += "mut ".len() + skip_blanks(rest);
            }
            let name = name_of(&code[start..]);
            if name.is_empty() {
                continue;
            }
            let end = code[start..]
                .find(';')
                .map_or(code.len(), |semicolon| start + semicolon);
            // The env! is CODE and the variable name is a LITERAL, so the two
            // halves are asked of the two views. A name that appears only
            // inside a message binds nothing.
            let Some(equals) = code[start..end].find('=') else {
                continue;
            };
            if is_crate_directory_expression(code, code_with_literals, start + equals + 1, end) {
                out.insert(name);
            }
        }
    }
    // A name the source POPS is not the crate directory after that line, and
    // this reader has no line order: `let mut root = PathBuf::from(env!(…));
    // root.pop();` is how a source climbs to the workspace without ever naming
    // `parent`. A popped name is dropped, so a shared-root literal joined onto
    // it is COUNTED.
    out.retain(|name| !is_popped(code, name));
    out
}

/// Does `literal` climb OUT of the directory it is joined onto?
fn climbs_out_of_its_directory(literal: &str) -> bool {
    let mut rest = literal;
    while let Some(next) = rest.strip_prefix("./") {
        rest = next;
    }
    rest.starts_with("../")
}

/// Is this literal joined onto the crate's OWN manifest directory?
///
/// `<crate dir>/AGENTS.md` is `crates/<crate>/AGENTS.md`: the crate's own copy
/// of a root-markdown NAME, not the repository's, and a literal that does not
/// climb out names something inside the crate whatever it is called. The
/// receiver is resolved one hop, through [`crate_directory_bindings`] or from
/// the inline expression itself.
/// Both views again, and for the same reason [`crate_directory_bindings`]
/// takes both: the receiver's parentheses are matched on the BLANKED view, so a
/// parenthesis inside a message never closes a call, and the text is then read
/// from the same byte range of the view that kept its literals.
fn joined_onto_the_crate_directory(
    code: &str,
    code_with_literals: &str,
    at: usize,
    crate_dir_names: &BTreeSet<String>,
) -> bool {
    let trimmed = code[..at].trim_end().len();
    let Some(without_join) = code[..trimmed].strip_suffix(".join(") else {
        return false;
    };
    let head_end = without_join.trim_end().len();
    let head = &code[..head_end];
    if head.ends_with(')') {
        // An inline receiver: `Path::new(env!("CARGO_MANIFEST_DIR")).join(`.
        // The WHOLE receiver goes to the rule the bindings take, so a call this
        // reader cannot place, `workspace_root(env!("CARGO_MANIFEST_DIR"))`,
        // is not the crate directory and the literal on it is counted.
        let receiver_end = match head.strip_suffix(TO_PATH_BUF) {
            Some(receiver) => receiver.trim_end().len(),
            None => head_end,
        };
        let Some(start) = call_expression_start(code, receiver_end) else {
            return false;
        };
        return is_crate_directory_expression(code, code_with_literals, start, head_end);
    }
    let start = head
        .char_indices()
        .rev()
        .find(|(_, ch)| !(ch.is_ascii_alphanumeric() || *ch == '_'))
        .map_or(0, |(i, ch)| i + ch.len_utf8());
    let name = &head[start..];
    !name.is_empty() && crate_dir_names.contains(name)
}

/// A `const` or `static` item: its name, and the byte range of the whole item.
struct ConstItem {
    name: String,
    start: usize,
    end: usize,
}

/// Every `const` / `static` item the CODE declares.
///
/// An item ends at the `;` that closes it at bracket depth zero, so a table
/// spanning many lines is ONE item. The depth walk is safe because it runs over
/// the blanked code view: a bracket inside a literal or a comment is a space.
fn const_items(code: &str) -> Vec<ConstItem> {
    let bytes = code.as_bytes();
    let mut out = Vec::new();
    let mut line_start = 0usize;
    for line in code.split_inclusive('\n') {
        let trimmed = line.trim_start();
        let mut rest = trimmed;
        if let Some(after) = rest.strip_prefix("pub") {
            let after = match after.strip_prefix('(') {
                Some(paren) => match paren.find(')') {
                    Some(close) => &paren[close + 1..],
                    None => after,
                },
                None => after,
            };
            if after.starts_with(char::is_whitespace) {
                rest = after.trim_start();
            }
        }
        let Some(after_keyword) = rest
            .strip_prefix("const ")
            .or_else(|| rest.strip_prefix("static "))
        else {
            line_start += line.len();
            continue;
        };
        let after_keyword = after_keyword.trim_start();
        let after_keyword = after_keyword
            .strip_prefix("mut ")
            .unwrap_or(after_keyword)
            .trim_start();
        let name: String = after_keyword
            .chars()
            .take_while(|ch| ch.is_ascii_alphanumeric() || *ch == '_')
            .collect();
        if name.is_empty() || !after_keyword[name.len()..].trim_start().starts_with(':') {
            line_start += line.len();
            continue;
        }
        let start = line_start + (line.len() - trimmed.len());
        let mut depth = 0i32;
        let mut i = start;
        let mut end = code.len();
        while i < bytes.len() {
            match bytes[i] {
                b'(' | b'[' | b'{' => depth += 1,
                b')' | b']' | b'}' => depth -= 1,
                b';' if depth <= 0 => {
                    end = i + 1;
                    break;
                }
                _ => {}
            }
            i += 1;
        }
        out.push(ConstItem { name, start, end });
        line_start += line.len();
    }
    out
}

/// Is `item`'s name mentioned anywhere in `code` outside `item` itself?
///
/// Whole-token, so a name is not found inside a longer one. A const nobody
/// reads is dead text, and dead text's paths are not reads.
fn const_is_used_elsewhere(code: &str, item: &ConstItem) -> bool {
    let boundary =
        |ch: Option<char>| ch.is_none_or(|ch| !(ch.is_ascii_alphanumeric() || ch == '_'));
    let mut from = 0usize;
    while let Some(offset) = code[from..].find(&item.name) {
        let at = from + offset;
        let after = at + item.name.len();
        if boundary(code[..at].chars().next_back())
            && boundary(code[after..].chars().next())
            && !(item.start..item.end).contains(&at)
        {
            return true;
        }
        from = after;
    }
    false
}

// ---------------------------------------------------------------------------
// Literal -> doc root.
// ---------------------------------------------------------------------------

/// `literal` with any leading run of `./` and `../` segments removed.
///
/// A test names a shared tree from its own crate directory, so the spelling in
/// the source is a relative climb followed by the root. Classifying at position
/// zero without this strip reads that as a path with no shared root at all,
/// which is the hole it left: three binaries that open the documentation tree
/// and the example workspace through their manifest directory were unpinned.
fn strip_relative_prefix(literal: &str) -> &str {
    let mut rest = literal;
    loop {
        if let Some(next) = rest.strip_prefix("./") {
            rest = next;
            continue;
        }
        if let Some(next) = rest.strip_prefix("../") {
            rest = next;
            continue;
        }
        return rest;
    }
}

/// The shared root `literal` is rooted at, if it is rooted at one.
///
/// SHAPE ONLY. Whether the literal is USED as a path is [`is_path_position`]'s
/// question, and both halves have to answer yes before a pin exists. The rules
/// and the measurements behind them are in the module docs.
fn doc_root_of(literal: &str) -> Option<String> {
    if literal.chars().any(char::is_whitespace) {
        return None;
    }
    let literal = strip_relative_prefix(literal);
    for dir in DOC_ROOT_DIRS {
        if let Some(rest) = literal.strip_prefix(*dir) {
            if rest.starts_with('/') {
                return Some((*dir).to_string());
            }
        }
    }
    for stem in ROOT_MARKDOWN_STEMS {
        let name = root_markdown_name(stem);
        if literal == name {
            return Some(name);
        }
    }
    None
}

/// Every shared root the CODE of one source READS.
///
/// A literal counts when it is rooted at a shared tree AND the code uses it as
/// a path: directly, or through a `const` / `static` item something else in the
/// file reads. Two uses that LOOK like reads are not: a path the code WRITES,
/// and a path joined onto the crate's own directory, which never leaves the
/// crate however it is spelled.
///
/// Sources with no rooted literal at all (almost every file under `src/`)
/// leave here before the const and binding scans run.
fn doc_roots_read_by_source(src: &str) -> BTreeSet<String> {
    let views = scan_code(src);
    let candidates: Vec<(&CodeLiteral, String)> = views
        .literals
        .iter()
        .filter_map(|literal| doc_root_of(&literal.text).map(|root| (literal, root)))
        .collect();
    if candidates.is_empty() {
        return BTreeSet::new();
    }

    let read_tables: Vec<ConstItem> = const_items(&views.code)
        .into_iter()
        .filter(|item| const_is_used_elsewhere(&views.code, item))
        .collect();
    let crate_dir_names = crate_directory_bindings(&views.code, &views.code_with_literals);

    let mut roots = BTreeSet::new();
    for (literal, root) in candidates {
        let before = &views.code[..literal.at];
        let used_as_a_path = is_path_position(before)
            || read_tables
                .iter()
                .any(|item| (item.start..item.end).contains(&literal.at));
        if !used_as_a_path || is_write_target(before) {
            continue;
        }
        if !climbs_out_of_its_directory(&literal.text)
            && joined_onto_the_crate_directory(
                &views.code,
                &views.code_with_literals,
                literal.at,
                &crate_dir_names,
            )
        {
            continue;
        }
        roots.insert(root);
    }
    roots
}

// ---------------------------------------------------------------------------
// Workspace members, and their integration-test sources.
// ---------------------------------------------------------------------------

/// `path` with `.` and `..` components resolved textually.
///
/// Lexical, not `canonicalize`: a path dependency is spelled relative to the
/// manifest that declares it, and the answer has to be the same string on every
/// machine so the member set can be deduplicated by equality.
fn normalised(path: &Path) -> PathBuf {
    let mut out = PathBuf::new();
    for component in path.components() {
        match component {
            std::path::Component::ParentDir => {
                out.pop();
            }
            std::path::Component::CurDir => {}
            other => out.push(other.as_os_str()),
        }
    }
    out
}

/// The directories `dir`'s manifest names as `path = "..."` dependencies.
///
/// Any dependency table, any kind: a dev-dependency folds a crate into the
/// workspace just as firmly as a normal one does.
fn path_dependency_dirs(dir: &Path) -> Vec<PathBuf> {
    let text = std::fs::read_to_string(dir.join("Cargo.toml")).unwrap_or_default();
    let mut out = Vec::new();
    for line in text.lines() {
        if line.trim_start().starts_with('#') {
            continue;
        }
        let mut rest = line;
        while let Some(at) = rest.find("path") {
            rest = &rest[at + "path".len()..];
            let Some(after) = rest.trim_start().strip_prefix('=') else {
                continue;
            };
            let Some(after) = after.trim_start().strip_prefix('"') else {
                continue;
            };
            let Some(close) = after.find('"') else {
                continue;
            };
            out.push(normalised(&dir.join(&after[..close])));
        }
    }
    out
}

/// The number of members `cargo metadata --format-version 1 --no-deps` reports
/// for this workspace.
///
/// Measured by running that command and counting `workspace_members`. It is the
/// only reader that resolves a path dependency into a member, and it reported
/// 66 where the root manifest's `members` array lists 65: the extra one is the
/// example node crate a viz library reaches through a path dev-dependency.
///
/// The walk below follows those edges ITSELF rather than shelling out to cargo,
/// and this number is what holds it to the same answer. A member cargo resolves
/// and the walk does not reach leaves that crate's test sources unscanned and
/// every pin they owe missing.
const WORKSPACE_MEMBER_COUNT: usize = 66;

/// Every workspace member directory: the root manifest's `members = [ ... ]`
/// array with globs expanded, then closed over in-workspace path dependencies.
///
/// Hand-rolled rather than `cargo metadata` for the reason the coverage walk
/// gives: the property under test is what the MANIFEST declares, and shelling
/// out to cargo inherits its lock and its multi-second resolve. The closure over
/// path dependencies is what keeps that cheap reading from being a SMALLER
/// reading. See [`WORKSPACE_MEMBER_COUNT`].
fn workspace_member_dirs() -> Vec<PathBuf> {
    let root = normalised(&repo_root());
    let manifest = std::fs::read_to_string(root.join("Cargo.toml")).expect("root Cargo.toml");
    let start = manifest
        .find("members = [")
        .expect("root Cargo.toml declares `members = [`");
    let rest = &manifest[start..];
    let end = rest
        .find("\n]")
        .expect("the members array is closed by a `]`");
    let body = &rest[..end];

    let mut out = Vec::new();
    for line in body.lines() {
        let line = line.trim();
        if line.starts_with('#') {
            continue;
        }
        let Some(open) = line.find('"') else { continue };
        let Some(close_rel) = line[open + 1..].find('"') else {
            continue;
        };
        let entry = &line[open + 1..open + 1 + close_rel];
        if let Some(parent) = entry.strip_suffix("/*") {
            let dir = root.join(parent);
            let mut expanded: Vec<PathBuf> = std::fs::read_dir(&dir)
                .unwrap_or_else(|e| panic!("cannot expand glob {entry}: {e}"))
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.join("Cargo.toml").is_file())
                .collect();
            expanded.sort();
            out.extend(expanded.iter().map(|p| normalised(p)));
        } else {
            out.push(normalised(&root.join(entry)));
        }
    }
    let declared = out.len();
    assert!(
        declared > 20,
        "the members walk found only {declared} entries: it is not parsing the \
         root manifest"
    );

    // Close over path dependencies that live inside the repository. A crate
    // reached this way is a workspace member cargo resolves, so `--workspace`
    // builds its tests and they can read a shared tree like any other.
    let mut seen: BTreeSet<PathBuf> = out.iter().cloned().collect();
    let mut pending = out.clone();
    while let Some(dir) = pending.pop() {
        for target in path_dependency_dirs(&dir) {
            if !target.starts_with(&root) || !target.join("Cargo.toml").is_file() {
                continue;
            }
            if seen.insert(target.clone()) {
                out.push(target.clone());
                pending.push(target);
            }
        }
    }
    out.sort();
    assert_eq!(
        out.len(),
        WORKSPACE_MEMBER_COUNT,
        "the member walk reached {} director(ies): {declared} declared in the \
         root manifest and {} added through in-workspace path dependencies, \
         but cargo resolves {WORKSPACE_MEMBER_COUNT}. A member the walk cannot \
         see has its test sources unscanned and every pin they owe missing; a \
         member it invents scans a directory cargo never builds. Re-measure \
         with `cargo metadata --format-version 1 --no-deps` and move the \
         constant with the tree.",
        out.len(),
        out.len() - declared,
    );
    out
}

/// The `name = "..."` of a crate manifest.
///
/// Read from the manifest rather than inferred from the directory because they
/// DIVERGE: `cerulion_wire/` publishes as `cerulion-wire`.
fn package_name(dir: &Path) -> String {
    let text = std::fs::read_to_string(dir.join("Cargo.toml"))
        .unwrap_or_else(|e| panic!("cannot read {}/Cargo.toml: {e}", dir.display()));
    for line in text.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("name") {
            let rest = rest.trim_start();
            if let Some(rest) = rest.strip_prefix('=') {
                let rest = rest.trim();
                if let Some(inner) = rest.strip_prefix('"') {
                    if let Some(end) = inner.find('"') {
                        return inner[..end].to_string();
                    }
                }
            }
        }
        // Stop at the next table: a `name` under `[[bin]]` is not the package.
        if line.starts_with('[') && !line.starts_with("[package]") {
            break;
        }
    }
    panic!("{}/Cargo.toml declares no package name", dir.display());
}

/// Every `.rs` file under `dir`, at any depth, sorted.
fn rust_sources_under(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(next) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&next) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
            } else if path.extension().is_some_and(|e| e == "rs") {
                out.push(path);
            }
        }
    }
    out.sort();
    out
}

/// `(package, test binary) -> the shared roots that binary reads`.
///
/// TWO populations, because cargo builds two kinds of test binary out of a
/// crate:
///
///   * depth-1 `tests/*.rs`, one binary each, named by the file stem, and a
///     `tests/common/mod.rs` helper or a `tests/ui/**` trybuild source is not a
///     target of its own and cannot be named in a marker;
///   * every `src/**/*.rs`, whose `#[cfg(test)]` units all compile into the ONE
///     library test binary. Their reads are pooled and attributed to a binary
///     named after the PACKAGE (`<package>::<package>`), because the step that
///     runs them is the package's own `cargo test -p <package>`, and no finer
///     name survives into the workflow.
fn derived_doc_pins() -> BTreeMap<(String, String), BTreeSet<String>> {
    let mut out = BTreeMap::new();
    let mut integration_sources = 0usize;
    let mut library_sources = 0usize;
    for dir in workspace_member_dirs() {
        let package = package_name(&dir);

        if let Ok(entries) = std::fs::read_dir(dir.join("tests")) {
            let mut files: Vec<PathBuf> = entries
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.is_file() && p.extension().is_some_and(|e| e == "rs"))
                .collect();
            files.sort();
            for file in files {
                let binary = file
                    .file_stem()
                    .expect("a `.rs` file has a stem")
                    .to_string_lossy()
                    .into_owned();
                let src = std::fs::read_to_string(&file)
                    .unwrap_or_else(|e| panic!("cannot read {}: {e}", file.display()));
                integration_sources += 1;
                let roots = doc_roots_read_by_source(&src);
                if !roots.is_empty() {
                    out.insert((package.clone(), binary), roots);
                }
            }
        }

        let mut library_roots = BTreeSet::new();
        for file in rust_sources_under(&dir.join("src")) {
            let src = std::fs::read_to_string(&file)
                .unwrap_or_else(|e| panic!("cannot read {}: {e}", file.display()));
            library_sources += 1;
            library_roots.extend(doc_roots_read_by_source(&src));
        }
        if !library_roots.is_empty() {
            out.insert((package.clone(), package.clone()), library_roots);
        }
    }
    assert!(
        integration_sources >= 200,
        "the walk read only {integration_sources} integration-test source(s): \
         it is not reaching the tree, and every pin it reports would be vacuous"
    );
    assert!(
        library_sources >= 400,
        "the walk read only {library_sources} library source(s): the `src/` \
         half is not reaching the tree, and a unit test that opens a shared \
         root would be invisible again"
    );
    out
}

// ---------------------------------------------------------------------------
// The workflow side.
// ---------------------------------------------------------------------------

fn ci_workflow_path() -> PathBuf {
    repo_root().join(".github/workflows/ci.yml")
}

/// The workflow as written, comments INCLUDED, because the markers are
/// comments.
fn ci_workflow_text() -> String {
    let path = ci_workflow_path();
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()))
}

/// The marker line one pin demands, exactly as it must appear in the workflow.
fn marker_line(package: &str, binary: &str, roots: &BTreeSet<String>) -> String {
    let roots: Vec<&str> = roots.iter().map(String::as_str).collect();
    format!(
        "{MARKER_PREFIX} {package}::{binary}{MARKER_VERB}{}",
        roots.join(", ")
    )
}

/// Every marker line the workflow carries, trimmed of its indent.
fn marker_lines(text: &str) -> Vec<String> {
    text.lines()
        .map(|l| l.trim().to_string())
        .filter(|l| l.starts_with(MARKER_PREFIX))
        .collect()
}

/// The package a marker line names, or `None` when the line is malformed.
fn marker_package(line: &str) -> Option<&str> {
    let rest = line.strip_prefix(MARKER_PREFIX)?.trim_start();
    let (pair, _) = rest.split_once(MARKER_VERB)?;
    let (package, binary) = pair.split_once("::")?;
    if package.is_empty() || binary.is_empty() {
        return None;
    }
    Some(package)
}

/// Split the `jobs:` mapping into `(name, block)` pairs, in file order.
fn jobs_of(text: &str) -> Vec<(String, String)> {
    let lines: Vec<&str> = text.lines().collect();
    let start = lines
        .iter()
        .position(|l| l.starts_with("jobs:"))
        .map(|i| i + 1)
        .unwrap_or(0);
    let mut out: Vec<(String, Vec<&str>)> = Vec::new();
    for line in &lines[start..] {
        if let Some(name) = job_key(line) {
            out.push((name, Vec::new()));
        }
        if let Some(last) = out.last_mut() {
            last.1.push(line);
        }
    }
    out.into_iter()
        .map(|(n, lines)| (n, lines.join("\n")))
        .collect()
}

/// The job id `line` opens, if it opens one: two spaces, an identifier, a colon.
///
/// The identifier test is what keeps a COMMENT out. `ci.yml` carries rationale
/// blocks at two-space indent, and one of them ending in a colon would otherwise
/// start a job block that swallows the real job below it.
fn job_key(line: &str) -> Option<String> {
    let rest = line.trim_end().strip_prefix("  ")?;
    if rest.starts_with(' ') {
        return None;
    }
    let name = rest.strip_suffix(':')?;
    if name.is_empty()
        || !name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
    {
        return None;
    }
    Some(name.to_string())
}

/// Does ONE workflow line name `package` as a package it runs tests for?
///
/// The two forms `ci.yml` uses, matched by WHOLE TOKEN so `cerulion_viz` and
/// `cerulion_vizd` stay apart: `cargo test … -p <package>` in every spelling
/// cargo accepts, and `ci_test_shard.sh <package> …`, which takes the name as
/// its first argument precisely so the name stays literally in the workflow.
fn line_names_package(line: &str, package: &str) -> bool {
    if line.trim().starts_with(MARKER_PREFIX) {
        return false;
    }
    if let Some(pos) = line.find("ci_test_shard.sh ") {
        let tail = &line[pos + "ci_test_shard.sh ".len()..];
        if tail.split_whitespace().next() == Some(package) {
            return true;
        }
    }
    let Some(pos) = line.find("cargo test") else {
        return false;
    };
    let mut it = line[pos..].split_whitespace();
    while let Some(tok) = it.next() {
        let named = match tok {
            "-p" | "--package" => it.next() == Some(package),
            _ => tok
                .strip_prefix("--package=")
                .or_else(|| tok.strip_prefix("-p"))
                .is_some_and(|rest| rest == package),
        };
        if named {
            return true;
        }
    }
    false
}

// ---------------------------------------------------------------------------
// THE GATE
// ---------------------------------------------------------------------------

#[test]
fn every_doc_reading_test_binary_is_pinned_in_ci_yml() {
    let pins = derived_doc_pins();
    assert!(
        !pins.is_empty(),
        "the walk derived no doc-pin at all: the literal extractor or the \
         root classifier is broken, and this gate would be vacuous"
    );

    let expected: BTreeSet<String> = pins
        .iter()
        .map(|((package, binary), roots)| marker_line(package, binary, roots))
        .collect();
    let found: BTreeSet<String> = marker_lines(&ci_workflow_text()).into_iter().collect();

    let missing: Vec<&String> = expected.difference(&found).collect();
    let stale: Vec<&String> = found.difference(&expected).collect();

    assert_eq!(
        found,
        expected,
        "the doc-pin markers in {} do not match the tests that read a shared \
         root.\n\nMISSING: add each of these as a YAML comment beside the step \
         that runs its package:\n{}\n\nSTALE: these markers describe no test \
         that reads a shared root; delete them:\n{}\n\nA test binary is pinned \
         when a literal in its code names `{}` or a root markdown file. The \
         marker changes no behaviour: it is the line a path-to-step map will \
         read to decide that this binary has to run when that root changes.",
        ci_workflow_path().display(),
        missing
            .iter()
            .map(|m| format!("  {m}"))
            .collect::<Vec<_>>()
            .join("\n"),
        stale
            .iter()
            .map(|m| format!("  {m}"))
            .collect::<Vec<_>>()
            .join("\n"),
        DOC_ROOT_DIRS.join(", "),
    );
}

/// The walk, against the hand scan of the same tree.
///
/// The gate above compares the walk with the workflow, so a walk that lost its
/// sight and a workflow that lost its markers agree with each other perfectly.
/// This arm is the outside opinion: rows read out of the sources by hand.
///
/// SET AGAINST SET, after the rows are proven unique. A length comparison plus
/// a row-keyed lookup asks the walk only about binaries somebody already
/// thought of, and a row written twice makes the length add up while hiding a
/// derived pin nobody wrote down: the map keyed by `(package, binary)` collapses
/// the duplicate, and the length that matched counted it twice.
#[test]
fn the_walk_reproduces_the_hand_scan_of_the_tree() {
    let rows: Vec<(String, String)> = HAND_SCANNED_DOC_PINS
        .iter()
        .map(|(package, binary, _, _)| ((*package).to_string(), (*binary).to_string()))
        .collect();
    let unique: BTreeSet<&(String, String)> = rows.iter().collect();
    assert_eq!(
        unique.len(),
        rows.len(),
        "HAND_SCANNED_DOC_PINS names one (package, binary) twice. Two rows for \
         one binary collapse into one map entry, so a derived pin with no row \
         of its own would ride along unnoticed."
    );

    let hand: BTreeMap<(String, String), BTreeSet<String>> = HAND_SCANNED_DOC_PINS
        .iter()
        .map(|(package, binary, dirs, stems)| {
            let roots: BTreeSet<String> = dirs
                .iter()
                .map(|d| (*d).to_string())
                .chain(stems.iter().map(|s| root_markdown_name(s)))
                .collect();
            (((*package).to_string(), (*binary).to_string()), roots)
        })
        .collect();

    let pins = derived_doc_pins();
    let derived_only: Vec<String> = pins
        .keys()
        .filter(|key| !hand.contains_key(*key))
        .map(|(package, binary)| format!("  {package}::{binary}"))
        .collect();
    let hand_only: Vec<String> = hand
        .keys()
        .filter(|key| !pins.contains_key(*key))
        .map(|(package, binary)| format!("  {package}::{binary}"))
        .collect();

    assert_eq!(
        pins,
        hand,
        "the walk disagrees with the hand scan.\n\nDERIVED WITH NO ROW: open \
         each source, read the call that takes the literal, and add the \
         row:\n{}\n\nA ROW THE WALK DOES NOT DERIVE: either the test stopped \
         reading the path (delete its row and its marker) or the extractor \
         stopped seeing it:\n{}\n\nA row whose ROOTS differ means the binary \
         gained or lost a read, and the row moves with the code.",
        derived_only.join("\n"),
        hand_only.join("\n"),
    );
}

/// A marker sits in the job that runs its package, not loose in the file.
///
/// Presence alone is satisfiable by a block of markers parked in the header,
/// which reads as a pin and pins nothing to a step. Every marker is therefore
/// required to live inside a job block that also carries a line running its
/// package, and the count of markers found inside job blocks is required to
/// equal the count in the file, so a marker outside every job cannot hide.
#[test]
fn every_doc_pin_marker_sits_in_the_job_that_runs_its_package() {
    let text = ci_workflow_text();
    let jobs = jobs_of(&text);
    assert!(
        jobs.len() >= 5,
        "the job split found only {} job(s) in {}: it is not parsing the \
         workflow",
        jobs.len(),
        ci_workflow_path().display()
    );

    let mut complaints: Vec<String> = Vec::new();
    let mut seen_in_jobs = 0usize;
    for (job, block) in &jobs {
        for line in block.lines() {
            let line = line.trim();
            if !line.starts_with(MARKER_PREFIX) {
                continue;
            }
            seen_in_jobs += 1;
            let Some(package) = marker_package(line) else {
                complaints.push(format!(
                    "  {job}: malformed marker `{line}`, the shape is \
                     `{MARKER_PREFIX} <package>::<test binary>{MARKER_VERB}<roots>`"
                ));
                continue;
            };
            if !block.lines().any(|l| line_names_package(l, package)) {
                complaints.push(format!(
                    "  {job}: `{line}` but no step in this job runs \
                     `{package}`; move the marker beside the step that does"
                ));
            }
        }
    }

    let total = marker_lines(&text).len();
    assert_eq!(
        seen_in_jobs,
        total,
        "{} of the {total} doc-pin marker(s) in {} sit outside every job block \
         (a marker that is not beside a step pins nothing)",
        total - seen_in_jobs,
        ci_workflow_path().display()
    );
    assert!(
        complaints.is_empty(),
        "doc-pin markers in the wrong place:\n{}",
        complaints.join("\n")
    );
}

// ---------------------------------------------------------------------------
// The extractor and the classifier, on hand-built input.
// ---------------------------------------------------------------------------

/// Assemble a path from a root and a tail.
///
/// Fixtures are composed rather than spelled for the reason
/// [`ROOT_MARKDOWN_STEMS`] gives: the walk reads this file, and a fixture
/// spelled as a whole rooted path would pin this binary to a tree it never
/// opens. `"docs"` alone carries no separator and `"/guide.md"` no root, so
/// neither half matches.
fn doc_path(root: &str, tail: &str) -> String {
    format!("{root}{tail}")
}

/// A path literal inside a comment is not a read, in either comment syntax,
/// nested, and with the positive control beside it.
///
/// Without this, the guard is text matching: a paragraph explaining why a test
/// no longer opens the documentation tree would keep the pin alive, and a read
/// commented out would keep its marker.
#[test]
fn a_path_literal_inside_a_comment_is_not_a_doc_pin() {
    let path = doc_path("docs", "/internals/cli.md");
    let docs_only: BTreeSet<String> = [String::from("docs")].into_iter().collect();

    // The positive control FIRST: the same literal in code is a read.
    let code = format!("let p = root.join(\"{path}\");\n");
    assert_eq!(doc_roots_read_by_source(&code), docs_only);

    let line_comment = format!("// let p = root.join(\"{path}\");\nfn t() {{}}\n");
    assert_eq!(
        doc_roots_read_by_source(&line_comment),
        BTreeSet::<String>::new()
    );

    let block_comment = format!("/* let p = root.join(\"{path}\"); */\nfn t() {{}}\n");
    assert_eq!(
        doc_roots_read_by_source(&block_comment),
        BTreeSet::<String>::new()
    );

    // Rust block comments NEST: the inner closer must not end the outer one.
    let nested = format!("/* outer /* inner */ let p = root.join(\"{path}\"); */\nfn t() {{}}\n");
    assert_eq!(doc_roots_read_by_source(&nested), BTreeSet::<String>::new());

    // A block opener inside a LINE comment opens nothing, so the code after it
    // is still read.
    let opener_in_line = format!("// /* note\nlet p = root.join(\"{path}\");\n");
    assert_eq!(doc_roots_read_by_source(&opener_in_line), docs_only);
}

/// A path inside the crate's own directory is not a shared-root read, and a
/// shared root that is not at the ROOT of the literal is not one either.
///
/// The classifier anchors at position zero on purpose. A substring rule would
/// pin every test that keeps a fixture under a directory named after a shared
/// tree, and those fixtures move with their crate.
#[test]
fn a_path_inside_the_crates_own_directory_is_not_a_doc_pin() {
    for local in [
        doc_path("tests", "/fixtures/graph.yaml"),
        doc_path("src", "/lib.rs"),
        // Contains a shared-root name, rooted at neither.
        format!("tests/fixtures/{}/guide.md", "docs"),
        format!("src/{}/render.rs", "examples"),
        // The crate's own copy of a root markdown file, named through the tree.
        format!(
            "crates/cerulion_cli_engine/{}",
            root_markdown_name("README")
        ),
    ] {
        assert_eq!(
            doc_root_of(&local),
            None,
            "`{local}` names a path inside a crate and must not pin anything"
        );
    }

    hand_typed_classifier_rows();
}

/// Hand-typed classifier rows: the literal, and the root it must report.
///
/// Typed out rather than looped over [`DOC_ROOT_DIRS`] and
/// [`ROOT_MARKDOWN_STEMS`]: a loop that builds its fixture from the same
/// constant the code reads asserts only that the classifier agrees with itself,
/// and an edit that broke the rule while editing the constant would keep
/// passing. These rows say what the answer is.
///
/// Each row is spelled in HALVES, `(head, tail, expected directory root,
/// expected root markdown stem)`, for the reason [`ROOT_MARKDOWN_STEMS`]
/// gives: the walk reads this file, and a whole rooted path or a whole root
/// markdown NAME spelled here would pin this binary to a tree it never opens.
/// At most one expectation is `Some`; both `None` means the literal names no
/// shared root.
#[allow(clippy::type_complexity)]
const CLASSIFIER_ORACLE: &[(&str, &str, Option<&str>, Option<&str>)] = &[
    // One row per shared tree, rooted.
    (".github", "/workflows/ci.yml", Some(".github"), None),
    ("benches", "/latency/bench.py", Some("benches"), None),
    ("docs", "/internals/cli.md", Some("docs"), None),
    ("examples", "/go2/Cargo.toml", Some("examples"), None),
    ("tools", "/scripts/ci_test_shard.sh", Some("tools"), None),
    // One row per root markdown file, as the WHOLE literal.
    ("AGENTS", ".md", None, Some("AGENTS")),
    ("CHANGELOG", ".md", None, Some("CHANGELOG")),
    ("CLAUDE", ".md", None, Some("CLAUDE")),
    ("README", ".md", None, Some("README")),
    // Spelled from a crate directory, which is how a test reaches the tree.
    ("../../docs", "/user-api.md", Some("docs"), None),
    ("../../../examples", "/go2", Some("examples"), None),
    ("./tools", "/scripts/build_deb.sh", Some("tools"), None),
    ("../../AGENTS", ".md", None, Some("AGENTS")),
    // A bare root NAME is a name: a directory, a key, a step marker.
    ("docs", "", None, None),
    ("tools", "", None, None),
    ("../../docs", "", None, None),
    // A longer name that merely opens with a root's letters.
    ("documentation", "/guide.md", None, None),
    ("toolset", "/run.sh", None, None),
    // A shared-root name that is not at the root of the literal.
    ("crates/cerulion_core/docs", "/notes.md", None, None),
    ("tests/fixtures/README", ".md", None, None),
    // Prose that opens with a path.
    ("docs", "/bag.md cites a 500 ms window", None, None),
];

fn hand_typed_classifier_rows() {
    for (head, tail, dir_root, markdown_stem) in CLASSIFIER_ORACLE {
        let literal = doc_path(head, tail);
        let expected = match (dir_root, markdown_stem) {
            (Some(dir), None) => Some((*dir).to_string()),
            (None, Some(stem)) => Some(root_markdown_name(stem)),
            (None, None) => None,
            (Some(_), Some(_)) => panic!("the row for `{literal}` names two expectations"),
        };
        assert_eq!(
            doc_root_of(&literal),
            expected,
            "the classifier disagrees with the hand-typed row for `{literal}`"
        );
    }

    // The table has to keep up with the constants: a new shared tree or root
    // markdown file with no row here would be classified by code nothing in
    // this file has an opinion about.
    for dir in DOC_ROOT_DIRS {
        assert!(
            CLASSIFIER_ORACLE
                .iter()
                .any(|(_, _, root, _)| *root == Some(*dir)),
            "no hand-typed row expects `{dir}`; add one before adding the root"
        );
    }
    for stem in ROOT_MARKDOWN_STEMS {
        assert!(
            CLASSIFIER_ORACLE
                .iter()
                .any(|(_, _, _, expected)| *expected == Some(*stem)),
            "no hand-typed row expects `{stem}`; add one before adding the file"
        );
    }
}

/// A format string rooted at a shared tree is a read, and a sentence that opens
/// with a path is not.
#[test]
fn a_format_string_rooted_at_a_doc_root_is_a_doc_pin() {
    let docs_only: BTreeSet<String> = [String::from("docs")].into_iter().collect();

    let interpolated = format!(
        "let p = root.join(format!(\"{}/internals/{{name}}.md\"));\n",
        "docs"
    );
    assert_eq!(doc_roots_read_by_source(&interpolated), docs_only);

    // An assertion message that opens with a path is prose: it carries spaces,
    // and a message is not a read.
    let message = format!(
        "assert!(ok, \"{}/bag.md cites a 500 ms window; update the doc\");\n",
        "docs"
    );
    assert_eq!(
        doc_roots_read_by_source(&message),
        BTreeSet::<String>::new()
    );
}

/// A raw string is ONE literal, so a manifest fixture that lists a root markdown
/// file inside an `include` array is not a read of that file.
///
/// The crate this shape comes from writes the manifest into a scratch directory
/// and never opens the repository's own copy.
#[test]
fn a_raw_string_fixture_is_one_literal_and_not_a_path() {
    let readme = root_markdown_name("README");
    let fixture = format!(
        "let manifest = r#\"\n[package]\nname = \"scratch\"\ninclude = [\"src/\", \"{readme}\"]\n\"#;\n"
    );
    assert_eq!(
        doc_roots_read_by_source(&fixture),
        BTreeSet::<String>::new()
    );

    // The same name as a literal of its own IS a read.
    let direct = format!("let p = root.join(\"{readme}\");\n");
    assert_eq!(
        doc_roots_read_by_source(&direct),
        [readme].into_iter().collect::<BTreeSet<String>>()
    );
}

/// The one-pass design, pinned: a `//` inside a string literal must not cut the
/// line, a `"` inside a character literal must not open a string, and a lifetime
/// must not be mistaken for one.
///
/// Each of these, got wrong, silently drops every path literal after it on the
/// line or in the file: the exact direction this file exists to close.
#[test]
fn comment_and_literal_markers_inside_literals_do_not_cut_the_scan() {
    let tools_only: BTreeSet<String> = [String::from("tools")].into_iter().collect();
    let script = doc_path("tools", "/scripts/build.sh");

    let url_first =
        format!("let u = \"https://example.invalid\"; let p = root.join(\"{script}\");\n");
    assert_eq!(doc_roots_read_by_source(&url_first), tools_only);

    let quote_char = format!("let q = '\"'; let p = root.join(\"{script}\");\n");
    assert_eq!(doc_roots_read_by_source(&quote_char), tools_only);

    let lifetime = format!("fn f<'a>(s: &'a str) {{ let p = root.join(\"{script}\"); }}\n");
    assert_eq!(doc_roots_read_by_source(&lifetime), tools_only);

    let escaped_char = format!("let n = '\\n'; let p = root.join(\"{script}\");\n");
    assert_eq!(doc_roots_read_by_source(&escaped_char), tools_only);

    // A block-comment opener inside a literal is data.
    let opener_literal = format!("let o = \"/*\"; let p = root.join(\"{script}\");\n");
    assert_eq!(doc_roots_read_by_source(&opener_literal), tools_only);

    // An escaped quote does not end the literal, so the path after it is seen.
    let escaped_quote = format!("let s = \"a\\\"b\"; let p = root.join(\"{script}\");\n");
    assert_eq!(doc_roots_read_by_source(&escaped_quote), tools_only);

    // Multi-byte characters survive the byte scan.
    let wide = format!("let s = \"é ü ø\"; let p = root.join(\"{script}\");\n");
    assert_eq!(doc_roots_read_by_source(&wide), tools_only);

    // The code around a literal is read from the BLANKED view, so a call that
    // appears only INSIDE another literal vouches for nothing.
    let call_inside_a_literal =
        format!("let s = \"root.join(\"; let p = \"{script}\";\nfn t() {{}}\n");
    assert_eq!(
        doc_roots_read_by_source(&call_inside_a_literal),
        BTreeSet::<String>::new()
    );
}

/// A rooted literal is a read only where the CODE opens or joins it.
///
/// The population this rule exists for is real and large: tests that name a
/// shared path in order to assert about a message, a classification or an
/// absence. Each negative row below is one of those shapes, and the positive
/// rows are the call forms the module docs list.
#[test]
fn a_rooted_literal_counts_only_where_the_code_uses_it_as_a_path() {
    let tools_only: BTreeSet<String> = [String::from("tools")].into_iter().collect();
    let script = doc_path("tools", "/scripts/build.sh");
    let nothing = BTreeSet::<String>::new();

    for opened in [
        format!("let p = Path::new(\"{script}\");\n"),
        format!("let p = PathBuf::from(\"{script}\");\n"),
        format!("let p = root.join(\"{script}\");\n"),
        format!("let t = std::fs::read_to_string(\"{script}\").unwrap();\n"),
        format!("let d = fs::read_dir(\"{script}\").unwrap();\n"),
        format!("let f = File::open(\"{script}\").unwrap();\n"),
        format!("let t = include_str!(\"{script}\");\n"),
        format!("let m = fs::metadata(\"{script}\").unwrap();\n"),
        format!("let p = Path::new(&format!(\"{script}\"));\n"),
    ] {
        assert_eq!(
            doc_roots_read_by_source(&opened),
            tools_only,
            "this code opens the path and must pin it:\n{opened}"
        );
    }

    for named_only in [
        // A refusal's wording.
        format!("assert!(reason.contains(\"{script}\"));\n"),
        // A tree a walk asserts it stayed OUT of.
        format!("assert!(!files.iter().any(|f| f.contains(\"{script}\")));\n"),
        // A bare binding nothing opens.
        format!("let p = \"{script}\";\n"),
        // A message argument beside a real assertion.
        format!("assert!(ok, \"{script}\");\n"),
        // A member of an inline array a loop compares against.
        format!("for out in [\"{script}\"] {{ assert!(!f.contains(out)); }}\n"),
    ] {
        assert_eq!(
            doc_roots_read_by_source(&named_only),
            nothing,
            "this code only NAMES the path and must pin nothing:\n{named_only}"
        );
    }
}

/// A path table in a `const` counts when something reads the const, and not
/// otherwise.
///
/// This is the one hop the walk takes without tracing where the name goes, and
/// both sides of it are pinned here: an unread table is dead text, and a read
/// one is a path whether the join happens two lines later or inside a helper.
#[test]
fn a_const_path_table_counts_only_when_the_const_is_read() {
    let map = doc_path("docs", "/internals/core-testing.md");
    let docs_only: BTreeSet<String> = [String::from("docs")].into_iter().collect();

    let read_later =
        format!("const MAP: &str = \"{map}\";\nfn t() {{ let p = root.join(MAP); }}\n");
    assert_eq!(doc_roots_read_by_source(&read_later), docs_only);

    // Through a helper, which is the shape the walk cannot follow and credits
    // anyway.
    let read_through_a_helper =
        format!("const WS: &[&str] = &[\"{map}\"];\nfn t() {{ members_of(WS); }}\n");
    assert_eq!(doc_roots_read_by_source(&read_through_a_helper), docs_only);

    // A table nobody reads is dead text.
    let never_read = format!("const MAP: &str = \"{map}\";\nfn t() {{}}\n");
    assert_eq!(
        doc_roots_read_by_source(&never_read),
        BTreeSet::<String>::new()
    );

    // Whole-token matching: a longer name is not a read of the shorter one.
    let prefix_only = format!("const MAP: &str = \"{map}\";\nfn t() {{ let r = MAP_ROWS; }}\n");
    assert_eq!(
        doc_roots_read_by_source(&prefix_only),
        BTreeSet::<String>::new()
    );

    // A multi-line table is ONE item: its closing `;` is found at bracket depth
    // zero, so the rows inside belong to the const and not to what follows.
    let table = format!(
        "const ROWS: &[(&str, &str)] = &[\n    (\"{map}\", \"why\"),\n];\nfn t() {{ use_rows(ROWS); }}\n"
    );
    assert_eq!(doc_roots_read_by_source(&table), docs_only);
}

/// The marker text, both halves, against hand-written strings.
///
/// The gate compares whole marker LINES, so a formatter that drifted by one
/// space would report every pin missing and every marker stale at once.
#[test]
fn a_marker_line_is_the_package_the_binary_and_the_sorted_roots() {
    let roots: BTreeSet<String> = [".github", "tools"].iter().map(|s| s.to_string()).collect();
    let line = marker_line("cerulion_core", "serial_discipline_test", &roots);
    assert_eq!(
        line,
        "# doc-pin: cerulion_core::serial_discipline_test reads .github, tools"
    );
    assert_eq!(marker_package(&line), Some("cerulion_core"));

    let one: BTreeSet<String> = [root_markdown_name("AGENTS")].into_iter().collect();
    assert_eq!(
        marker_line("cerulion_wsd", "doc_deference_test", &one),
        "# doc-pin: cerulion_wsd::doc_deference_test reads AGENTS.md"
    );

    // Malformed markers are named rather than ignored.
    assert_eq!(marker_package("# doc-pin: no separator here"), None);
    assert_eq!(marker_package("# doc-pin: pkg::bin"), None);
    assert_eq!(marker_package("# doc-pin: ::bin reads docs"), None);
    assert_eq!(marker_package("# doc-pin: pkg:: reads docs"), None);
}

/// The workflow readers, on hand-written documents.
///
/// A job splitter that mistook a two-space comment for a job key would merge the
/// real job that follows it into the comment's block, and every marker in that
/// job would then be checked against the wrong steps.
#[test]
fn a_two_space_comment_does_not_open_a_job_and_a_marker_never_names_its_own_package() {
    let doc = "on:\n  pull_request:\njobs:\n  \
               # a rationale block that ends in a colon:\n  \
               real-job:\n    runs-on: ubuntu-latest\n    steps:\n      \
               # doc-pin: pkg::bin reads docs\n      - run: cargo test -p pkg\n  \
               other-job:\n    runs-on: ubuntu-latest\n    steps:\n      - run: true\n";
    let jobs = jobs_of(doc);
    assert_eq!(
        jobs.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(),
        vec!["real-job", "other-job"]
    );
    assert!(jobs[0].1.contains("cargo test -p pkg"));

    // The marker line itself must not vouch for the package it names.
    assert!(!line_names_package(
        "      # doc-pin: pkg::bin reads docs",
        "pkg"
    ));
    assert!(line_names_package("      - run: cargo test -p pkg", "pkg"));
    assert!(line_names_package(
        "      - run: ./tools/scripts/ci_test_shard.sh pkg 0 4",
        "pkg"
    ));
    // Whole-token matching, which is what keeps a name off its own prefix.
    assert!(!line_names_package(
        "      - run: cargo test -p pkgd",
        "pkg"
    ));
    assert!(!line_names_package(
        "      - run: cargo build -p pkg",
        "pkg"
    ));
}

/// A path the code WRITES is not a path it reads.
///
/// The row this kills is real: `completions_test` writes a root-markdown NAME
/// into a temporary directory to prove a FILE under `nodes/` is not a node
/// type, and the pin said it opened the repository's own copy. The read beside
/// it (a read nested in a write's LATER argument) still counts, because the
/// destination is the write's FIRST argument.
#[test]
fn a_path_the_code_writes_is_not_a_doc_pin() {
    let page = doc_path("docs", "/internals/cli.md");
    let readme = root_markdown_name("README");
    let docs_only: BTreeSet<String> = [String::from("docs")].into_iter().collect();
    let nothing = BTreeSet::<String>::new();

    for written in [
        format!("write(&nodes.join(\"{readme}\"), \"notes\\n\");\n"),
        format!("std::fs::write(root.join(\"{page}\"), body).unwrap();\n"),
        format!("fs::create_dir_all(root.join(\"{page}\")).unwrap();\n"),
        format!("fs::remove_file(Path::new(\"{page}\")).unwrap();\n"),
    ] {
        assert_eq!(
            doc_roots_read_by_source(&written),
            nothing,
            "this code writes the path and must pin nothing:\n{written}"
        );
    }

    // The positive controls: the same paths, opened.
    assert_eq!(
        doc_roots_read_by_source(&format!(
            "let t = std::fs::read_to_string(root.join(\"{page}\")).unwrap();\n"
        )),
        docs_only
    );
    assert_eq!(
        doc_roots_read_by_source(&format!("let p = root.join(\"{readme}\");\n")),
        [readme].into_iter().collect::<BTreeSet<String>>()
    );
    // A read inside a write's SECOND argument is still a read: the destination
    // is the first one.
    assert_eq!(
        doc_roots_read_by_source(&format!(
            "fs::write(&out, fs::read_to_string(root.join(\"{page}\")).unwrap()).unwrap();\n"
        )),
        docs_only
    );
}

/// A path joined onto the crate's OWN directory is inside the crate, whatever
/// it is called.
///
/// `cerulion_wsd`'s doc-deference pin recorded a read of the repository's root
/// agent-docs file; what it opens is its own crate's copy. The receiver is
/// resolved one hop, so the same literal joined onto the repository root is
/// still a read, and a literal that climbs OUT with `../` is one however it is
/// reached, which is the shape three real rows have.
#[test]
fn a_path_joined_onto_the_crates_own_directory_is_not_a_doc_pin() {
    let agents = root_markdown_name("AGENTS");
    let page = doc_path("docs", "/user-api.md");
    let climb = doc_path("../../docs", "/user-api.md");
    let agents_only: BTreeSet<String> = [agents.clone()].into_iter().collect();
    let docs_only: BTreeSet<String> = [String::from("docs")].into_iter().collect();
    let nothing = BTreeSet::<String>::new();
    let bind = format!("let crate_root = Path::new(env!(\"{CRATE_DIR_ENV}\"));\n");

    // The crate's own copy, through a binding and inline.
    assert_eq!(
        doc_roots_read_by_source(&format!("{bind}let f = crate_root.join(\"{agents}\");\n")),
        nothing
    );
    assert_eq!(
        doc_roots_read_by_source(&format!(
            "let f = Path::new(env!(\"{CRATE_DIR_ENV}\")).join(\"{agents}\");\n"
        )),
        nothing
    );
    // A directory inside the crate that happens to be named after a shared
    // tree is inside the crate too.
    assert_eq!(
        doc_roots_read_by_source(&format!("{bind}let f = crate_root.join(\"{page}\");\n")),
        nothing
    );

    // The same literal reached from the repository root IS a read.
    assert_eq!(
        doc_roots_read_by_source(&format!(
            "let root = repo_root();\nlet f = root.join(\"{agents}\");\n"
        )),
        agents_only
    );
    // And so is one that climbs out of the crate, which is how the real rows
    // reach the tree from their manifest directory.
    assert_eq!(
        doc_roots_read_by_source(&format!("{bind}let f = crate_root.join(\"{climb}\");\n")),
        docs_only
    );
    // A receiver that climbed out with `parent()` is not the crate directory.
    assert_eq!(
        doc_roots_read_by_source(&format!(
            "let root = Path::new(env!(\"{CRATE_DIR_ENV}\")).parent().unwrap();\n\
             let f = root.join(\"{agents}\");\n"
        )),
        agents_only
    );
    // And `parent(` is only ONE of the ways a real source climbs. The rule is
    // that the initialiser is EXACTLY the manifest directory, so a `pop()`, a
    // `join("..")`, a `join("../..")` and an `ancestors()` each take the
    // binding out of the set and leave the literal COUNTED.
    for climb in [
        format!("let mut root = PathBuf::from(env!(\"{CRATE_DIR_ENV}\"));\nroot.pop();\n"),
        format!("let root = PathBuf::from(env!(\"{CRATE_DIR_ENV}\")).join(\"..\");\n"),
        format!("let root = PathBuf::from(env!(\"{CRATE_DIR_ENV}\")).join(\"../..\");\n"),
        format!("let root = Path::new(env!(\"{CRATE_DIR_ENV}\")).ancestors().nth(1).unwrap();\n"),
    ] {
        assert_eq!(
            doc_roots_read_by_source(&format!("{climb}let f = root.join(\"{agents}\");\n")),
            agents_only,
            "a binding that climbed out of the crate is not the crate \
             directory:\n{climb}"
        );
    }

    // The other side of the same rule: the plain manifest-directory binding
    // still SUPPRESSES, in both constructors, with and without the copy, for
    // both root-markdown names.
    let readme = root_markdown_name("README");
    for binding in [
        format!("let crate_root = Path::new(env!(\"{CRATE_DIR_ENV}\"));\n"),
        format!("let crate_root = PathBuf::from(env!(\"{CRATE_DIR_ENV}\"));\n"),
        format!("let crate_root = Path::new(env!(\"{CRATE_DIR_ENV}\")).to_path_buf();\n"),
    ] {
        for name in [&agents, &readme] {
            assert_eq!(
                doc_roots_read_by_source(&format!(
                    "{binding}let f = crate_root.join(\"{name}\");\n"
                )),
                nothing,
                "the crate's own copy of `{name}` is not a doc pin:\n{binding}"
            );
        }
    }

    // An INLINE receiver this reader cannot place is not the crate directory
    // either: only the two constructors are, so a helper call around the
    // environment variable leaves the literal counted, and the copy does not.
    assert_eq!(
        doc_roots_read_by_source(&format!(
            "let f = workspace_root(env!(\"{CRATE_DIR_ENV}\")).join(\"{agents}\");\n"
        )),
        agents_only
    );
    assert_eq!(
        doc_roots_read_by_source(&format!(
            "let f = Path::new(env!(\"{CRATE_DIR_ENV}\")).to_path_buf().join(\"{agents}\");\n"
        )),
        nothing
    );

    // The binding reader itself, both ways, and a `let` spelled inside a
    // MESSAGE binds nothing, because the structure is read from the blanked
    // view.
    let views = scan_code(&format!(
        "{bind}let root = crate_root.parent().unwrap();\nlet other = repo_root();\n\
         let msg = \"let planted = env!(CARGO_MANIFEST_DIR)\";\n"
    ));
    let bindings = crate_directory_bindings(&views.code, &views.code_with_literals);
    assert_eq!(
        bindings,
        ["crate_root".to_string()]
            .into_iter()
            .collect::<BTreeSet<String>>()
    );
}

// ---------------------------------------------------------------------------
// OBSERVATION EDGES: what a test observes WITHOUT a cargo dependency edge.
// ---------------------------------------------------------------------------
//
// THE POPULATION. The cargo reverse-dependency closure
// (`tools/scripts/ci_selected_packages.py`) carries every package whose tests
// reach another package through a manifest edge. Three classes reach one
// WITHOUT an edge, and a selection that gates a step without them skips a test
// the change could have broken:
//
//   * a path literal into another member's directory, opened or joined
//     (`crate-path`);
//   * a walk over a tree that holds more than one member, or over the
//     repository root itself (`whole-tree`), which observes EVERY package;
//   * a `dlopen` of an artifact another member builds, named by its
//     `lib<name>.so` / `lib<name>.dylib` file name (`dlopen`).
//
// FAIL CLOSED. A literal this walk cannot attribute to one package, a path
// whose package segment is built at run time or an artifact name built at run
// time, records the observing package as observing `all`: the selector then
// adds it to every selection that names a package, and no step of it may be
// gated. That is the fourth kind, `unattributable`.
//
// The rules reuse the doc-pin classifier above, literal for literal: the same
// blanked views, the same `is_path_position`, the same write-target refusal and
// the same crate-directory suppression. What differs is only what a literal is
// compared AGAINST: the workspace member directories rather than the shared
// trees.

/// The kind column of one row of the committed table.
const EDGE_CRATE_PATH: &str = "crate-path";
const EDGE_DLOPEN: &str = "dlopen";
const EDGE_WHOLE_TREE: &str = "whole-tree";
const EDGE_UNATTRIBUTABLE: &str = "unattributable";

/// The observed-package spelling that means EVERY workspace member.
const OBSERVES_EVERYTHING: &str = "all";

/// The committed derived table, relative to the repository root.
///
/// Spelled in halves for the reason [`ROOT_MARKDOWN_STEMS`] gives: this walk
/// reads its own source, and a whole `tools/`-rooted literal here would pin
/// this binary to a read it does not perform.
const OBSERVATION_EDGES_DIR: &str = "tools/ci";
const OBSERVATION_EDGES_NAME: &str = "observation_edges.tsv";

/// The path of the committed table, from the repository root.
fn observation_edges_path() -> PathBuf {
    repo_root()
        .join(OBSERVATION_EDGES_DIR)
        .join(OBSERVATION_EDGES_NAME)
}

/// The 1-based line `offset` sits on.
fn line_at(src: &str, offset: usize) -> usize {
    src[..offset].matches('\n').count() + 1
}

/// Every workspace member as `(directory relative to the repository root,
/// package name)`, longest directory first so a nested member wins over its
/// parent.
fn member_directories() -> Vec<(String, String)> {
    let root = normalised(&repo_root());
    let mut out: Vec<(String, String)> = workspace_member_dirs()
        .into_iter()
        .map(|dir| {
            let rel = dir
                .strip_prefix(&root)
                .unwrap_or_else(|_| panic!("{} is outside the repository", dir.display()))
                .to_string_lossy()
                .replace('\\', "/");
            (rel, package_name(&dir))
        })
        .collect();
    out.sort_by(|a, b| b.0.len().cmp(&a.0.len()).then_with(|| a.0.cmp(&b.0)));
    out
}

/// `<artifact stem> -> package name` for every member that builds a `cdylib`.
///
/// The stem is the package name with `-` replaced by `_`, which is the name
/// cargo gives the shared object (`libcerulion_heaphook.so`). Read from the
/// manifests rather than from `cargo metadata` for the reason
/// [`workspace_member_dirs`] gives.
fn cdylib_artifact_stems() -> BTreeMap<String, String> {
    let root = normalised(&repo_root());
    let mut out = BTreeMap::new();
    for (rel, package) in member_directories() {
        let manifest = root.join(&rel).join("Cargo.toml");
        let text = std::fs::read_to_string(&manifest).unwrap_or_default();
        if !manifest_declares_cdylib(&text) {
            continue;
        }
        out.insert(package.replace('-', "_"), package);
    }
    out
}

/// Does this manifest declare a `cdylib` crate type?
///
/// The `crate-type` key of any target table, in either TOML list spelling. A
/// `#` comment line is skipped, so a commented-out crate type declares nothing.
fn manifest_declares_cdylib(text: &str) -> bool {
    for line in text.lines() {
        let line = line.trim();
        if line.starts_with('#') {
            continue;
        }
        let Some(rest) = line.strip_prefix("crate-type") else {
            continue;
        };
        let Some(rest) = rest.trim_start().strip_prefix('=') else {
            continue;
        };
        if rest.contains("cdylib") {
            return true;
        }
    }
    false
}

/// Is `path` inside, or equal to, the directory `dir`?
fn is_inside(path: &str, dir: &str) -> bool {
    if dir.is_empty() {
        return true;
    }
    path == dir || path.starts_with(&format!("{dir}/"))
}

/// The directories that hold MORE THAN ONE member, plus the repository root.
///
/// A literal naming one of these is a walk over a tree that holds several
/// packages, so the observing package observes all of them.
fn multi_member_directories(members: &[(String, String)]) -> BTreeSet<String> {
    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    for (rel, _) in members {
        let mut parts: Vec<&str> = rel.split('/').collect();
        while parts.pop().is_some() {
            *counts.entry(parts.join("/")).or_insert(0) += 1;
        }
    }
    counts
        .into_iter()
        .filter(|(dir, count)| *count > 1 && !dir.is_empty())
        .map(|(dir, _)| dir)
        .collect()
}

/// `path` with `.` and `..` resolved textually, as a `/`-joined string.
///
/// `None` when the path climbs above its base, which names something outside
/// the repository and is nothing this walk has an opinion about.
fn normalised_relative(path: &str) -> Option<String> {
    let mut out: Vec<&str> = Vec::new();
    for part in path.split('/') {
        match part {
            "" | "." => {}
            ".." => {
                out.pop()?;
            }
            other => out.push(other),
        }
    }
    Some(out.join("/"))
}

/// The artifact stem of a `lib<name>.so` / `lib<name>.dylib` file name.
fn dlopen_artifact_stem(name: &str) -> Option<&str> {
    let stem = name
        .strip_suffix(".so")
        .or_else(|| name.strip_suffix(".dylib"))?;
    let stem = stem.strip_prefix("lib")?;
    (!stem.is_empty()
        && stem
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-'))
    .then_some(stem)
}

/// One derived edge: what is observed, how, and on which line.
type SourceEdge = (String, &'static str, usize);

/// Does a literal carrying an interpolation name a member tree or an artifact?
///
/// A `{` in a literal means a segment is built at run time. That is only this
/// walk's business where the literal would otherwise have been attributable:
/// a segment naming a directory that holds members (`crates/{pkg}/...`), or a
/// file name shaped like a shared object (`lib{name}.so`). Anything else is an
/// ordinary run-time path, almost always inside a scratch directory, and is a
/// stated limit rather than a refusal.
fn interpolated_literal_is_unattributable(literal: &str, multi: &BTreeSet<String>) -> bool {
    if !literal.contains('{') {
        return false;
    }
    let last = literal.rsplit('/').next().unwrap_or(literal);
    if last.starts_with("lib") && (last.ends_with(".so") || last.ends_with(".dylib")) {
        return true;
    }
    literal
        .split('/')
        .any(|segment| !segment.is_empty() && multi.contains(segment))
}

/// The literal texts a chain of `.join("...")` calls carries in FRONT of the
/// literal that opens at `at`.
///
/// `root.join("crates").join("cerulion_core").join("tests")` reaches this walk
/// as three separate literals, and the LAST of them is the only one whose
/// classification can see the whole path. Without the chain the first literal
/// reads as a walk over the whole `crates` tree and the package is never
/// attributed.
///
/// Read from the view that KEPT literal bodies, because the previous link's
/// text is a literal; comments are blanked there, so a chain spelled inside one
/// joins nothing. A chain this reader cannot follow yields the links it did
/// follow, which classifies a SHORTER path: the over-inclusive direction.
fn join_chain_prefix(code_with_literals: &str, at: usize) -> (Vec<String>, usize) {
    let mut parts: Vec<String> = Vec::new();
    let mut end = code_with_literals[..at].trim_end().len();
    while parts.len() < 16 {
        let head = &code_with_literals[..end];
        let Some(head) = head.strip_suffix(".join(") else {
            break;
        };
        let head = head.trim_end();
        let Some(head) = head.strip_suffix(')') else {
            break;
        };
        let head = head.trim_end();
        let Some(head) = head.strip_suffix('"') else {
            break;
        };
        let Some(open) = head.rfind('"') else {
            break;
        };
        // The text in FRONT of the literal has to be the link's own `.join(`.
        // Without this the walk steps straight past a receiver that merely ENDS
        // in a call carrying a literal, `Path::new(env!("CARGO_MANIFEST_DIR"))`
        // above all, and splices the environment variable's NAME onto the front
        // of the path.
        if !head[..open].ends_with(".join(") {
            break;
        }
        parts.push(head[open + 1..].to_string());
        end = open;
    }
    parts.reverse();
    // `end` now sits just after the OUTERMOST `.join(` of the chain, which is
    // where the receiver that the whole path is resolved against ends.
    (parts, end)
}

/// The bare NAME a `.join(` at `end` is called on, if it is called on one.
///
/// `end` is the offset just after a `.join(` token. A receiver that is a call
/// rather than a name answers `None`: this reader resolves one hop through a
/// binding and no further.
fn join_receiver_name(code: &str, end: usize) -> Option<&str> {
    let head = code[..end].trim_end().strip_suffix(".join(")?;
    let head = head.trim_end();
    let start = head
        .char_indices()
        .rev()
        .find(|(_, ch)| !(ch.is_ascii_alphanumeric() || *ch == '_'))
        .map_or(0, |(i, ch)| i + ch.len_utf8());
    let name = &head[start..];
    (!name.is_empty()).then_some(name)
}

/// Names this source binds to the crate's own manifest directory and then POPS.
///
/// A popped manifest directory is the directory that HOLDS the members
/// (`crates/` for `crates/<x>`), so a literal joined onto such a name is a
/// SIBLING crate's path. [`crate_directory_bindings`] drops these names, which
/// is right for the suppression rule it serves and wrong here: read against the
/// repository root instead, `root.pop(); root.join("cerulion_bagd/src")` names
/// no member at all and a real cross-crate read derives nothing.
fn popped_crate_directory_bindings(code: &str, code_with_literals: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let name_of = |text: &str| -> String {
        text.chars()
            .take_while(|ch| ch.is_ascii_alphanumeric() || *ch == '_')
            .collect()
    };
    for keyword in ["let ", "const ", "static "] {
        let mut from = 0usize;
        while let Some(offset) = code[from..].find(keyword) {
            let at = from + offset;
            from = at + keyword.len();
            let opens_a_token = code[..at]
                .chars()
                .next_back()
                .is_none_or(|ch| !(ch.is_ascii_alphanumeric() || ch == '_'));
            if !opens_a_token {
                continue;
            }
            let mut start = from;
            let skip_blanks = |text: &str| text.len() - text.trim_start().len();
            start += skip_blanks(&code[start..]);
            if let Some(rest) = code[start..].strip_prefix("mut ") {
                start += "mut ".len() + skip_blanks(rest);
            }
            let name = name_of(&code[start..]);
            if name.is_empty() || !is_popped(code, &name) {
                continue;
            }
            let end = code[start..]
                .find(';')
                .map_or(code.len(), |semicolon| start + semicolon);
            let Some(equals) = code[start..end].find('=') else {
                continue;
            };
            if is_crate_directory_expression(code, code_with_literals, start + equals + 1, end) {
                out.insert(name);
            }
        }
    }
    out
}

/// The calls whose argument a repository-root path can be WITHOUT the source
/// walking the tree.
///
/// Each of these reads the root as a VALUE (a prefix to strip, a string to
/// print), never as a directory to enumerate. Anything else that takes the
/// repository root as an argument can read any file under it, so it counts.
const BENIGN_ROOT_ARGUMENT_CALLS: &[&str] = &[
    "assert!",
    "assert_eq!",
    "assert_ne!",
    "contains",
    "display",
    "ends_with",
    "eprintln!",
    "eq",
    "expect",
    "format!",
    "panic!",
    "println!",
    "push_str",
    "starts_with",
    "strip_prefix",
    "to_string",
    "unwrap_or",
    "write!",
    "writeln!",
];

/// The call names this walk reads as "the repository root", spelled without
/// their parentheses.
const REPO_ROOT_CALLS: &[&str] = &["repo_root", "workspace_root"];

/// Is this initialiser text the REPOSITORY ROOT?
///
/// Two shapes, and no others. A call to one of [`REPO_ROOT_CALLS`] with no
/// argument, which is this workspace's universal spelling; or the crate's own
/// manifest directory followed by a climb out of it (`parent`, `ancestors`, a
/// `..` join), which is what a source writes when it has no helper.
fn is_repo_root_expression(code: &str, code_with_literals: &str, start: usize, end: usize) -> bool {
    let expr = code[start..end].trim();
    for name in REPO_ROOT_CALLS {
        let call = format!("{name}()");
        if expr == call || expr.ends_with(&format!("::{call}")) {
            return true;
        }
    }
    if !code_with_literals[start..end].contains(CRATE_DIR_ENV) {
        return false;
    }
    if ![".parent(", ".ancestors(", ".join(\"..", ".join(\"../"]
        .iter()
        .any(|climb| expr.contains(climb))
    {
        return false;
    }
    // A climb that ENDS in a named join is not the root: it is one file, and
    // the literal that names it is what classifies the read. Only a climb whose
    // last step is a `..` (or nothing at all) leaves the binding pointing at the
    // tree, and a climb joined with a value this walk cannot see
    // (`root.join(crate_rel)`) is the root as far as it can tell.
    if !expr.ends_with("\")") {
        return true;
    }
    match last_literal_in(&code_with_literals[start..end]) {
        Some(text) => text.chars().all(|ch| ch == '.' || ch == '/'),
        None => true,
    }
}

/// The contents of the LAST double-quoted literal in `text`, if it has one.
fn last_literal_in(text: &str) -> Option<&str> {
    let close = text.rfind('"')?;
    let open = text[..close].rfind('"')?;
    Some(&text[open + 1..close])
}

/// Names this source binds to the REPOSITORY ROOT.
///
/// The same one-hop `let` / `const` / `static` scan
/// [`crate_directory_bindings`] runs, with the repository-root predicate, plus
/// the names bound to the crate directory that the source later POPS: a
/// popped manifest directory is the workspace, which is how a source climbs out
/// without ever naming `parent`.
fn repo_root_bindings(code: &str, code_with_literals: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let name_of = |text: &str| -> String {
        text.chars()
            .take_while(|ch| ch.is_ascii_alphanumeric() || *ch == '_')
            .collect()
    };
    for keyword in ["let ", "const ", "static "] {
        let mut from = 0usize;
        while let Some(offset) = code[from..].find(keyword) {
            let at = from + offset;
            from = at + keyword.len();
            let opens_a_token = code[..at]
                .chars()
                .next_back()
                .is_none_or(|ch| !(ch.is_ascii_alphanumeric() || ch == '_'));
            if !opens_a_token {
                continue;
            }
            let mut start = from;
            let skip_blanks = |text: &str| text.len() - text.trim_start().len();
            start += skip_blanks(&code[start..]);
            if let Some(rest) = code[start..].strip_prefix("mut ") {
                start += "mut ".len() + skip_blanks(rest);
            }
            let name = name_of(&code[start..]);
            if name.is_empty() {
                continue;
            }
            let end = code[start..]
                .find(';')
                .map_or(code.len(), |semicolon| start + semicolon);
            let Some(equals) = code[start..end].find('=') else {
                continue;
            };
            let value_at = start + equals + 1;
            if is_repo_root_expression(code, code_with_literals, value_at, end)
                || (is_crate_directory_expression(code, code_with_literals, value_at, end)
                    && is_popped(code, &name))
            {
                out.insert(name);
            }
        }
    }
    out
}

/// Every offset at which this source hands the REPOSITORY ROOT to a call that
/// can read the tree under it.
///
/// A root path used as a call ARGUMENT can be enumerated by that call, so it
/// counts; a root path used as the RECEIVER of `.join(` is classified by the
/// literal it joins, and every other receiver use (`.display()`) reads the
/// value rather than the tree. [`BENIGN_ROOT_ARGUMENT_CALLS`] carries the
/// argument positions that are values too.
fn whole_tree_walk_offsets(code: &str, root_names: &BTreeSet<String>) -> Vec<usize> {
    let mut out = Vec::new();
    let mut needles: Vec<String> = root_names.iter().cloned().collect();
    for name in REPO_ROOT_CALLS {
        needles.push(format!("{name}()"));
    }
    for needle in needles {
        let mut from = 0usize;
        while let Some(offset) = code[from..].find(&needle) {
            let at = from + offset;
            from = at + needle.len();
            let opens = code[..at]
                .chars()
                .next_back()
                .is_none_or(|ch| !(ch.is_ascii_alphanumeric() || ch == '_'));
            let after = code[at + needle.len()..].trim_start();
            let closes = !after.starts_with(|ch: char| ch.is_ascii_alphanumeric() || ch == '_');
            if !opens || !closes {
                continue;
            }
            // A `.join(` receiver is classified by its literal, never here.
            if after.starts_with(".join(") {
                continue;
            }
            // A DECLARATION is not a use. `fn publishable_members(root: &Path)`
            // puts the name in argument position, after a `(`, in every source
            // that also binds a name of its own to the root, and reading a
            // parameter list as a walk made two packages observe everything.
            if after.starts_with(':') {
                continue;
            }
            // An argument opens after `(`, `,` or a borrow of either.
            let before = code[..at].trim_end();
            let before = before.strip_suffix("mut").unwrap_or(before).trim_end();
            let before = before.strip_suffix('&').unwrap_or(before).trim_end();
            if !(before.ends_with('(') || before.ends_with(',')) {
                continue;
            }
            let innermost = enclosing_calls(&code[..at])
                .into_iter()
                .map(|(name, _)| name)
                .find(|name| !name.is_empty());
            if innermost
                .as_deref()
                .is_some_and(|name| BENIGN_ROOT_ARGUMENT_CALLS.contains(&name))
            {
                continue;
            }
            out.push(at);
        }
    }
    out.sort_unstable();
    out
}

/// Every observation edge one source carries.
///
/// `observing` is the package the source belongs to and `source_dir` is that
/// package's directory relative to the repository root, which is the base a
/// relative literal is resolved against. An edge to the observing package
/// itself is not an edge: a package always observes its own tree.
fn observation_edges_of_source(
    src: &str,
    observing: &str,
    source_dir: &str,
    members: &[(String, String)],
    multi: &BTreeSet<String>,
    cdylibs: &BTreeMap<String, String>,
) -> BTreeSet<SourceEdge> {
    let views = scan_code(src);
    let mut out: BTreeSet<SourceEdge> = BTreeSet::new();

    let root_names = repo_root_bindings(&views.code, &views.code_with_literals);
    for at in whole_tree_walk_offsets(&views.code, &root_names) {
        out.insert((
            OBSERVES_EVERYTHING.to_string(),
            EDGE_WHOLE_TREE,
            line_at(src, at),
        ));
    }

    if views.literals.is_empty() {
        return out;
    }
    let read_tables: Vec<ConstItem> = const_items(&views.code)
        .into_iter()
        .filter(|item| const_is_used_elsewhere(&views.code, item))
        .collect();
    let crate_dir_names = crate_directory_bindings(&views.code, &views.code_with_literals);
    let popped_dir_names = popped_crate_directory_bindings(&views.code, &views.code_with_literals);

    for literal in &views.literals {
        // PROSE IS NOT A PATH, the rule `doc_root_of` lives by and for the same
        // measured reason: this workspace spells refusals that OPEN with a
        // crate path and continue into a sentence, and one of them was read as
        // a read of that crate.
        if literal.text.chars().any(char::is_whitespace) {
            continue;
        }
        let before = &views.code[..literal.at];
        let in_a_read_table = read_tables
            .iter()
            .any(|item| (item.start..item.end).contains(&literal.at));
        // A CONST TABLE entry has to LOOK like a path, because the table rule is
        // the weak half of the two: it credits a literal nobody opens on this
        // line. A bare directory NAME in one is a classification key (`examples`
        // beside the sentence explaining what that directory holds), and reading
        // one as a path attributed a whole example workspace to the package that
        // merely names it. A literal in a real path position needs no separator:
        // `root.join("crates")` is a walk of the crates tree however it is
        // spelled.
        // The shape is read AFTER the climb: `../../../examples` is a bare
        // directory NAME reached from a crate directory, and a table row that
        // carries one is a classification key rather than a read. Reading the
        // climb's own separators as path structure attributed a whole example
        // workspace to a package that only names the directory.
        let table_shape = strip_relative_prefix(&literal.text).contains('/')
            || dlopen_artifact_stem(&literal.text).is_some();
        let used_as_a_path = is_path_position(before) || (in_a_read_table && table_shape);
        if !used_as_a_path || is_write_target(before) {
            continue;
        }
        if !climbs_out_of_its_directory(&literal.text)
            && joined_onto_the_crate_directory(
                &views.code,
                &views.code_with_literals,
                literal.at,
                &crate_dir_names,
            )
        {
            continue;
        }
        // A link that is NOT the last of its chain is classified with the rest
        // of the path, never on its own: `root.join("crates").join("beta")`
        // reads as a walk of the whole crates tree at its first link and as one
        // package at its second, and only the second is what the code opens.
        // A chain that continues with something this walk cannot read
        // (`.join(pkg)`) is a different case and stays classified here, which
        // is the fail-closed answer.
        if let Some(read) = read_string_literal(&views.code, literal.at) {
            if views.code[read.next..]
                .trim_start()
                .starts_with(").join(\"")
            {
                continue;
            }
        }
        let (mut chain, chain_start) = join_chain_prefix(&views.code_with_literals, literal.at);
        chain.push(literal.text.clone());
        let whole = chain.join("/");
        let line = line_at(src, literal.at);

        if interpolated_literal_is_unattributable(&whole, multi) {
            out.insert((OBSERVES_EVERYTHING.to_string(), EDGE_UNATTRIBUTABLE, line));
            continue;
        }
        if whole.contains('{') {
            continue;
        }
        // EVERY reading of the literal, because this tree spells one all three
        // ways and the union is the over-inclusive direction:
        //
        //   * joined onto the crate's own directory, which is what a relative
        //     climb is written against;
        //   * joined onto the repository root;
        //   * joined onto the crate directory's PARENT, and ONLY where the
        //     receiver is a name this source popped its manifest directory
        //     into. That reading was missing, and a source that pops its own
        //     crate out and then joins a sibling's name (`root.pop();
        //     root.join("cerulion_bagd/src")`) derived no edge at all: a real
        //     cross-crate read, invisible, in the miss direction.
        let mut candidates: Vec<String> = Vec::new();
        if whole.starts_with("./") || whole.starts_with("../") {
            if let Some(resolved) = normalised_relative(&format!("{source_dir}/{whole}")) {
                candidates.push(resolved);
            }
            if let Some(resolved) = normalised_relative(strip_relative_prefix(&whole)) {
                candidates.push(resolved);
            }
        } else if let Some(resolved) = normalised_relative(&whole) {
            candidates.push(resolved);
        }
        // ...and the third reading applies ONLY where the receiver is a name
        // this source popped its own manifest directory into. A blanket parent
        // reading turned every bare literal into a sibling crate's path and
        // manufactured edges nobody reads.
        let popped_receiver = join_receiver_name(&views.code, chain_start)
            .is_some_and(|name| popped_dir_names.contains(name));
        if popped_receiver {
            if let Some((parent, _)) = source_dir.rsplit_once('/') {
                if !parent.is_empty() {
                    if let Some(resolved) = normalised_relative(&format!("{parent}/{whole}")) {
                        candidates.push(resolved);
                    }
                }
            }
        }

        for candidate in candidates {
            // A literal that normalises to NOTHING is a separator or a `.`,
            // never a walk of the repository root: `parts.join("/")` and
            // `Path::new(".")` both land here, and reading them as the root
            // made five packages observe the whole workspace. A walk that has
            // no literal at all is what `whole_tree_walk_offsets` is for.
            if candidate.is_empty() {
                continue;
            }
            if let Some((_, package)) = members.iter().find(|(dir, _)| is_inside(&candidate, dir)) {
                if package != observing {
                    out.insert((package.clone(), EDGE_CRATE_PATH, line));
                }
                continue;
            }
            if multi.contains(&candidate) {
                out.insert((OBSERVES_EVERYTHING.to_string(), EDGE_WHOLE_TREE, line));
                continue;
            }
            let held: Vec<&(String, String)> = members
                .iter()
                .filter(|(dir, _)| is_inside(dir, &candidate))
                .collect();
            if held.len() == 1 && held[0].1 != observing {
                out.insert((held[0].1.clone(), EDGE_CRATE_PATH, line));
                continue;
            }
            let name = candidate.rsplit('/').next().unwrap_or(&candidate);
            if let Some(stem) = dlopen_artifact_stem(name) {
                if let Some(package) = cdylibs.get(stem) {
                    if package != observing {
                        out.insert((package.clone(), EDGE_DLOPEN, line));
                    }
                }
            }
        }
    }
    out
}

/// One row of the committed table.
///
/// `witness` is the source FILE, and `line` is the line inside it that the walk
/// derived; a row parsed from the table carries `None`. The witness column is
/// the file alone: a change that reflows a source above a witnessed literal
/// moves the line and alters no edge, and the parser refuses a witness ending
/// in `:digits`, naming the rule.
#[derive(Clone, Debug)]
struct ObservationEdge {
    observing: String,
    observed: String,
    kind: String,
    witness: String,
    line: Option<usize>,
}

impl ObservationEdge {
    /// The row as the table carries it: the witness file, never the line.
    fn row(&self) -> String {
        format!(
            "{}\t{}\t{}\t{}",
            self.observing, self.observed, self.kind, self.witness
        )
    }

    /// Where a reader opens the edge: the witness with the line when the edge
    /// came from the walk, the file alone when the row was parsed from the table.
    fn location(&self) -> String {
        match self.line {
            Some(line) => format!("{}:{line}", self.witness),
            None => self.witness.clone(),
        }
    }
}

/// The observation edges this tree carries, one row per
/// `(observing, observed, kind)` with the FIRST source that witnesses it.
///
/// One row per FACT, not one per occurrence: a package that walks the whole
/// tree from forty sources owes one row, and the row names the source a reader
/// opens to redetermine it. The table stays proportional to the number of
/// edges, which is what makes a hand scan of it possible at all.
fn derived_observation_edges() -> Vec<ObservationEdge> {
    let root = normalised(&repo_root());
    let members = member_directories();
    let multi = multi_member_directories(&members);
    let cdylibs = cdylib_artifact_stems();

    let mut best: BTreeMap<(String, String, String), (String, usize)> = BTreeMap::new();
    let mut sources_read = 0usize;
    for (rel, package) in &members {
        let dir = root.join(rel);
        let mut files = rust_sources_under(&dir.join("tests"));
        files.extend(rust_sources_under(&dir.join("src")));
        files.sort();
        for file in files {
            let src = std::fs::read_to_string(&file)
                .unwrap_or_else(|e| panic!("cannot read {}: {e}", file.display()));
            sources_read += 1;
            let source_rel = file
                .strip_prefix(&root)
                .unwrap_or(&file)
                .to_string_lossy()
                .replace('\\', "/");
            for (observed, kind, line) in
                observation_edges_of_source(&src, package, rel, &members, &multi, &cdylibs)
            {
                // The witness is the FIRST source, and `first` is decided on
                // the (path, LINE NUMBER) pair, so line 9 precedes line 10. The
                // ROW carries the path alone; the line rides along and reaches
                // a MISSING row in the failure message, where a reader opens it.
                let witness = (source_rel.clone(), line);
                let key = (package.clone(), observed, kind.to_string());
                best.entry(key)
                    .and_modify(|held| {
                        if witness < *held {
                            *held = witness.clone();
                        }
                    })
                    .or_insert(witness);
            }
        }
    }
    assert!(
        sources_read >= 600,
        "the observation walk read only {sources_read} source(s): it is not \
         reaching the tree, and every edge it reports would be vacuous"
    );
    best.into_iter()
        .map(
            |((observing, observed, kind), (source, line))| ObservationEdge {
                observing,
                observed,
                kind,
                witness: source,
                line: Some(line),
            },
        )
        .collect()
}

/// The committed table, parsed.
fn committed_observation_edges() -> Vec<ObservationEdge> {
    let path = observation_edges_path();
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "cannot read {}: {e}\n\nThe derived observation-edge table is \
             committed so the selector can read it without compiling this \
             test. Run this test with the file absent to see the rows it \
             demands.",
            path.display()
        )
    });
    parse_observation_edges(&text, &path)
}

/// The rows of one table text, `path` named in every refusal.
///
/// A row this parser cannot read is a REFUSAL, never a skipped row: a field
/// count other than four, a field with surrounding whitespace (the failure
/// message indents the rows it prints by two spaces) or a witness ending in
/// `:digits` (the line the table does not carry) stops the parse and names the
/// file, the line and the value refused. Blank lines and lines opening with `#`
/// are skipped. An empty field
/// passes through to the table comparison, where no derived row matches it;
/// `ci_selected_packages.py` refuses it outright.
fn parse_observation_edges(text: &str, path: &Path) -> Vec<ObservationEdge> {
    let mut out = Vec::new();
    for (index, line) in text.lines().enumerate() {
        if line.trim().is_empty() || line.starts_with('#') {
            continue;
        }
        let fields: Vec<&str> = line.split('\t').collect();
        assert_eq!(
            fields.len(),
            4,
            "{}:{} has {} tab-separated field(s), not 4: `{line}`",
            path.display(),
            index + 1,
            fields.len()
        );
        if let Some((column, field)) = fields.iter().enumerate().find(|(_, f)| **f != f.trim()) {
            panic!(
                "{}:{} column {} reads `{field}` with surrounding whitespace; every \
                 field is the bare value, and the failure message indents the rows \
                 it prints by two spaces",
                path.display(),
                index + 1,
                column + 1
            );
        }
        let witness = fields[3];
        assert!(
            !witness_ends_in_a_line(witness),
            "{}:{} names the witness `{witness}`; the witness column is the FILE \
             alone, with no `:line` suffix. The walk prints the line beside a \
             MISSING row in the failure message.",
            path.display(),
            index + 1
        );
        out.push(ObservationEdge {
            observing: fields[0].to_string(),
            observed: fields[1].to_string(),
            kind: fields[2].to_string(),
            witness: witness.to_string(),
            line: None,
        });
    }
    out
}

/// Whether `witness` ends in `:` and digits, the line a row must not carry.
fn witness_ends_in_a_line(witness: &str) -> bool {
    matches!(
        witness.rsplit_once(':'),
        Some((_, tail)) if !tail.is_empty() && tail.bytes().all(|b| b.is_ascii_digit())
    )
}

/// The rows the walk derives and the table lacks, each followed by where the
/// walk reads the edge, and the rows the table carries that no source supports.
fn missing_and_stale(
    derived: &[ObservationEdge],
    committed: &[ObservationEdge],
) -> (Vec<String>, Vec<String>) {
    let derived_rows: BTreeMap<String, String> =
        derived.iter().map(|e| (e.row(), e.location())).collect();
    let committed_rows: BTreeSet<String> = committed.iter().map(ObservationEdge::row).collect();
    let missing = derived_rows
        .iter()
        .filter(|(row, _)| !committed_rows.contains(*row))
        .map(|(row, location)| format!("  {row}   (the walk reads it at {location})"))
        .collect();
    let stale = committed_rows
        .iter()
        .filter(|row| !derived_rows.contains_key(*row))
        .map(|row| format!("  {row}"))
        .collect();
    (missing, stale)
}

/// The hand scan of the observation edges this tree carries.
///
/// Each row is `(observing package, observed package, kind)`, and every row was
/// produced by OPENING the source the committed table names and reading the
/// call that takes the literal, not by running the walk and copying what it
/// said. The comment above each row names that source.
///
/// The WITNESS column is deliberately not here. This file is one of the sources
/// the walk reads, and a whole `crates/`-rooted path literal in a const table
/// it reads is an observation edge of its own: the hand rows would manufacture
/// the very edges they are meant to check. The witness is pinned in the
/// committed table instead, which is data rather than source, and
/// `the_committed_observation_edge_table_matches_the_walk` compares that table
/// whole.
///
/// Compared as a SET against a set after the rows are proven unique, for the
/// reason [`HAND_SCANNED_DOC_PINS`] gives: a row written twice makes a length
/// add up while hiding a derived edge nobody wrote down.
const HAND_SCANNED_OBSERVATION_EDGES: &[(&str, &str, &str)] = &[
    // state_anchor_partial_scan_test's REGION table names sources of two other
    // crates by their paths under the crates tree: the cli engine's graph
    // command, replay state and state arm attach, and the core's state ring.
    ("cerulion_bagd", "cerulion_cli_engine", "crate-path"),
    ("cerulion_bagd", "cerulion_core", "crate-path"),
    // upstream_waivers_test declares the waived arms as literal paths from the
    // workspace root, and three of them name a `cerulion_cli` test file, so the
    // walk derives this edge from the literals alone.
    ("cerulion_core", "cerulion_cli", "crate-path"),
    // mp_supervisor_box_test loads three node libraries whose file names it
    // builds at run time: `debug_dir.join(format!("lib{id}.so"))`. The walk
    // cannot say which package builds them.
    ("cerulion_cli", "all", "unattributable"),
    // replay_cli_test copies a fixture source out of the crates tree by a
    // relative path chosen at run time: the manifest directory's parent joined
    // with `src_rel`, handed to `std::fs::copy`.
    ("cerulion_cli", "all", "whole-tree"),
    // ros2_run_e2e_test names the preload the launcher injects,
    // `libcerulion_heaphook.so`, which is the artifact cerulion_heaphook builds.
    ("cerulion_cli", "cerulion_heaphook", "dlopen"),
    // ros2_graph_e2e_test names `librmw_cerulion.so`, the artifact rmw_cerulion
    // builds, under the sandbox library directory.
    ("cerulion_cli", "rmw_cerulion", "dlopen"),
    // block_colocation_adoption_test's `read_src` reads the manifest directory's
    // parent joined with a crate-relative path its callers choose.
    ("cerulion_cli_engine", "all", "whole-tree"),
    // shm_state's own unit test scans `here.join("../cerulion_cli/src")` beside
    // its own source tree.
    ("cerulion_cli_engine", "cerulion_cli", "crate-path"),
    // replay_engine's unit test opens the manifest directory joined with
    // `..`, `cerulion_core`, `src`.
    ("cerulion_cli_engine", "cerulion_core", "crate-path"),
    // ros2_cmd holds the preload's file name in a const the launcher joins:
    // `HEAPHOOK_FILENAME`.
    ("cerulion_cli_engine", "cerulion_heaphook", "dlopen"),
    // dependency_door_test checks the members of the `examples/go2` workspace,
    // which holds one workspace member of its own: the demo's tf source node.
    ("cerulion_cli_engine", "go2_tf_source", "crate-path"),
    // workspace_lock_verb_adoption_test reads
    // `engine.join("../cerulion_wsd/src/protocol.rs")`.
    ("cerulion_cli_engine", "cerulion_wsd", "crate-path"),
    // workspace resolves `base.join("crates/native_ros2_messages")`.
    ("cerulion_cli_engine", "native_ros2_messages", "crate-path"),
    // ros2_cmd holds `RMW_LIB_FILENAME`, `librmw_cerulion.so`, the artifact
    // rmw_cerulion builds.
    ("cerulion_cli_engine", "rmw_cerulion", "dlopen"),
    // cdylib_iox2_log_level_test's EXCLUSIONS table names `crates/test_fixtures/`,
    // a tree that holds many members, and the same package's
    // serial_discipline_test walks the repository root itself
    // (`collect_rs(&root, &root, &mut all_rs)`).
    ("cerulion_core", "all", "whole-tree"),
    // state_record_framing_scan_test's table reads
    // `crates/cerulion_bag/src/writer.rs`.
    ("cerulion_core", "cerulion_bag", "crate-path"),
    // doc_attachment_discipline_test's table reads `../cerulion_bagd/src/lib.rs`.
    ("cerulion_core", "cerulion_bagd", "crate-path"),
    // cdylib_iox2_log_level_test reads
    // `crates/cerulion_cli_engine/src/templates.rs`.
    ("cerulion_core", "cerulion_cli_engine", "crate-path"),
    // the same test's EXCLUSIONS table names
    // `crates/cerulion_macros/src/codegen.rs`.
    ("cerulion_core", "cerulion_macros", "crate-path"),
    // dynamic_generated_parity_test reads the `examples/go2` workspace's schema
    // file, and that workspace holds one member of its own: the demo's tf
    // source node.
    ("cerulion_core", "go2_tf_source", "crate-path"),
    // crate_license_texts_test hands the workspace root to `publishable_members`,
    // which enumerates every member.
    ("cerulion_hygiene", "all", "whole-tree"),
    // shipped_surface_structure_test reads
    // `crates/cerulion_cli/src/main.rs`.
    ("cerulion_hygiene", "cerulion_cli", "crate-path"),
    // dependency_rules_test walks `crates/cerulion_viz/lib/go2_tf`.
    ("cerulion_hygiene", "go2_tf", "crate-path"),
    // the same test walks `examples/go2`, which holds that member.
    ("cerulion_hygiene", "go2_tf_source", "crate-path"),
    // crate_license_texts_test reads `crates/native_ros2_messages`.
    ("cerulion_hygiene", "native_ros2_messages", "crate-path"),
    // heaphook's own unit test reads `../cerulion_heaphook/src`.
    ("rmw_cerulion", "cerulion_heaphook", "crate-path"),
    // rmw_adopt_take_linux_test loads `libcerulion_heaphook.so` out of the
    // profile directory.
    ("rmw_cerulion", "cerulion_heaphook", "dlopen"),
];

/// The walk, against the hand scan of the same tree.
#[test]
fn the_observation_edge_walk_reproduces_the_hand_scan() {
    let rows: Vec<(String, String, String)> = HAND_SCANNED_OBSERVATION_EDGES
        .iter()
        .map(|(a, b, c)| ((*a).to_string(), (*b).to_string(), (*c).to_string()))
        .collect();
    let hand: BTreeSet<(String, String, String)> = rows.iter().cloned().collect();
    assert_eq!(
        hand.len(),
        rows.len(),
        "HAND_SCANNED_OBSERVATION_EDGES names one (observing, observed, kind) \
         twice. Two rows for one edge collapse into one set member, so a \
         derived edge with no row of its own would ride along unnoticed."
    );

    let derived: BTreeSet<(String, String, String)> = derived_observation_edges()
        .into_iter()
        .map(|edge| (edge.observing, edge.observed, edge.kind))
        .collect();
    let derived_only: Vec<String> = derived
        .difference(&hand)
        .map(|(a, b, c)| format!("  (\"{a}\", \"{b}\", \"{c}\"),"))
        .collect();
    let hand_only: Vec<String> = hand
        .difference(&derived)
        .map(|(a, b, c)| format!("  (\"{a}\", \"{b}\", \"{c}\"),"))
        .collect();
    assert_eq!(
        derived,
        hand,
        "the observation walk disagrees with the hand scan.\n\nDERIVED WITH NO \
         ROW: open the source the committed table names, read the call that \
         takes the literal, and add the row:\n{}\n\nA ROW THE WALK DOES NOT \
         DERIVE: either the source stopped observing that package (delete the \
         row) or the walk stopped seeing it:\n{}",
        derived_only.join("\n"),
        hand_only.join("\n"),
    );
    assert!(
        !derived.is_empty(),
        "the walk derived no observation edge at all: this workspace has \
         cross-crate reads and dlopen fixtures, so an empty answer means the \
         classifier is broken and every gate built on it would be vacuous"
    );
}

/// The committed table against the walk, both directions.
///
/// MISSING is an edge the walk derives and the file does not carry: the
/// selector would then gate a step on a closure that cannot see it. STALE is a
/// row for a read that no longer exists: it pre-authorises the next hole
/// exactly as a stale doc-pin marker does.
#[test]
fn the_committed_observation_edge_table_matches_the_walk() {
    let derived = derived_observation_edges();
    let committed = committed_observation_edges();

    let unique: BTreeSet<(String, String, String)> = committed
        .iter()
        .map(|e| (e.observing.clone(), e.observed.clone(), e.kind.clone()))
        .collect();
    assert_eq!(
        unique.len(),
        committed.len(),
        "{} names one (observing, observed, kind) twice; the table carries one \
         row per edge",
        observation_edges_path().display()
    );
    let committed_rows: Vec<String> = committed.iter().map(ObservationEdge::row).collect();
    let mut sorted = committed_rows.clone();
    sorted.sort();
    assert_eq!(
        committed_rows,
        sorted,
        "{} is not in byte order; the rows are sorted so a diff of the file is a \
         diff of the edges",
        observation_edges_path().display()
    );

    // THE ROW IS THE EQUALITY; a MISSING row prints beside it where the walk
    // reads the edge (file and line). The rows a reader adds carry the file alone.
    let derived_rows: BTreeSet<String> = derived.iter().map(ObservationEdge::row).collect();
    assert!(
        !derived.is_empty(),
        "the walk derived no observation edge; the table has nothing to match"
    );
    let committed_rows: BTreeSet<String> = committed_rows.into_iter().collect();
    let (missing, stale) = missing_and_stale(&derived, &committed);
    assert_eq!(
        committed_rows,
        derived_rows,
        "{} does not match the observation edges the sources carry.\n\nMISSING \
         (add each row to the file, WITHOUT the line):\n{}\n\nSTALE (delete \
         each row):\n{}\n\nThe file is read by \
         `tools/scripts/ci_selected_packages.py --observation-edges`, so a \
         MISSING row is a package the selection can skip while a test of it \
         observes the change.",
        observation_edges_path().display(),
        missing.join("\n"),
        stale.join("\n"),
    );
}

// ---------------------------------------------------------------------------
// The observation classifier, on hand-built input.
// ---------------------------------------------------------------------------

/// A hand-built workspace: four members, one of them nested, one of them under
/// the examples tree, and one that builds a `cdylib`.
///
/// Spelled in HALVES wherever a fixture would otherwise carry a whole rooted
/// path, for the reason [`ROOT_MARKDOWN_STEMS`] gives: this walk reads its own
/// source, and a literal here that looked like a path into another member's
/// tree would manufacture an observation edge of its own.
fn fixture_members() -> Vec<(String, String)> {
    let mut out: Vec<(String, String)> = [
        (format!("crates{}alpha", "/"), "alpha"),
        (format!("crates{}beta", "/"), "beta"),
        (format!("crates{}nest{}gamma", "/", "/"), "gamma"),
        (format!("examples{}demo{}delta", "/", "/"), "delta"),
    ]
    .into_iter()
    .map(|(dir, name)| (dir, name.to_string()))
    .collect();
    out.sort_by(|a, b| b.0.len().cmp(&a.0.len()).then_with(|| a.0.cmp(&b.0)));
    out
}

/// `observation_edges_of_source` over [`fixture_members`], observed from
/// `alpha`.
fn fixture_edges(src: &str) -> BTreeSet<SourceEdge> {
    let members = fixture_members();
    let multi = multi_member_directories(&members);
    let cdylibs: BTreeMap<String, String> = [("gamma".to_string(), "gamma".to_string())]
        .into_iter()
        .collect();
    let source_dir = format!("crates{}alpha", "/");
    observation_edges_of_source(src, "alpha", &source_dir, &members, &multi, &cdylibs)
}

fn edge(observed: &str, kind: &'static str, line: usize) -> BTreeSet<SourceEdge> {
    [(observed.to_string(), kind, line)].into_iter().collect()
}

/// A line added above a witnessed literal leaves the ROW and moves the line the
/// failure prints, and an edge onto another member is another row.
#[test]
fn a_line_above_a_witness_moves_the_printed_line_and_leaves_the_row() {
    let beta_src = format!("crates{}beta{}src{}lib.rs", "/", "/", "/");
    let gamma_src = format!("crates{}nest{}gamma{}src{}lib.rs", "/", "/", "/", "/");
    let plain = format!("let p = root.join(\"{beta_src}\");\n");
    let reflowed = format!("// a line this change adds above it\n{plain}");
    let witness = format!("crates{}alpha{}src{}lib.rs", "/", "/", "/");
    let edges = |src: &str| -> Vec<ObservationEdge> {
        fixture_edges(src)
            .into_iter()
            .map(|(observed, kind, line)| ObservationEdge {
                observing: "alpha".to_string(),
                observed,
                kind: kind.to_string(),
                witness: witness.clone(),
                line: Some(line),
            })
            .collect()
    };
    let rows =
        |src: &str| -> BTreeSet<String> { edges(src).iter().map(ObservationEdge::row).collect() };
    let locations = |src: &str| -> BTreeSet<String> {
        edges(src).iter().map(ObservationEdge::location).collect()
    };
    let one = |row: String| -> BTreeSet<String> { [row].into_iter().collect() };

    assert_eq!(
        rows(&plain),
        one(format!("alpha\tbeta\tcrate-path\t{witness}")),
        "the plain source is one crate-path row onto beta"
    );
    assert_eq!(
        rows(&reflowed),
        rows(&plain),
        "the same edge one line further down is the same row"
    );
    assert_eq!(
        locations(&plain),
        one(format!("{witness}:1")),
        "the walk reads the plain source's edge at line 1"
    );
    assert_eq!(
        locations(&reflowed),
        one(format!("{witness}:2")),
        "and at line 2, below the added line"
    );

    // BOTH SIDES: an edge onto another member is a different row, so the rule
    // is not satisfied by a projection that forgets what it observed.
    let changed = format!("let p = root.join(\"{gamma_src}\");\n");
    assert_eq!(
        rows(&changed),
        one(format!("alpha\tgamma\tcrate-path\t{witness}")),
        "an edge onto another member is another row"
    );
}

/// A MISSING row prints the row the reader adds and, beside it, the file and
/// line where the walk reads the edge; a STALE row prints the row alone.
#[test]
fn a_missing_row_prints_where_the_walk_reads_it_and_a_stale_row_prints_bare() {
    let witness = format!("crates{}alpha{}src{}lib.rs", "/", "/", "/");
    let edge = |observed: &str, line: Option<usize>| ObservationEdge {
        observing: "alpha".to_string(),
        observed: observed.to_string(),
        kind: EDGE_CRATE_PATH.to_string(),
        witness: witness.clone(),
        line,
    };
    let derived = [edge("beta", Some(7)), edge("delta", Some(3))];
    let committed = [edge("delta", None), edge("gamma", None)];
    let (missing, stale) = missing_and_stale(&derived, &committed);
    assert_eq!(
        missing,
        [format!(
            "  alpha\tbeta\tcrate-path\t{witness}   (the walk reads it at {witness}:7)"
        )],
        "the row the table lacks, then where the walk reads it"
    );
    assert_eq!(
        stale,
        [format!("  alpha\tgamma\tcrate-path\t{witness}")],
        "the row no source supports, bare"
    );
}

/// The parser takes a row's witness as the file, verbatim and without a line,
/// past comment and blank lines.
#[test]
fn the_table_parser_takes_the_witness_as_the_file_alone() {
    let alpha_src = format!("crates{}alpha{}src{}lib.rs", "/", "/", "/");
    let text = format!("# a comment\n\nalpha\tbeta\tcrate-path\t{alpha_src}\n");
    let rows = parse_observation_edges(&text, Path::new("observation_edges.tsv"));
    assert_eq!(
        rows.len(),
        1,
        "one data row past a comment and a blank line"
    );
    assert_eq!(
        rows[0].row(),
        format!("alpha\tbeta\tcrate-path\t{alpha_src}")
    );
    assert_eq!(rows[0].line, None, "a parsed row carries no line");
    assert_eq!(
        rows[0].location(),
        alpha_src,
        "and its location is the file alone"
    );
}

/// A row pasted with the two-space indent the failure message prints is refused
/// with the column named.
#[test]
#[should_panic(expected = "column 1 reads `  alpha` with surrounding whitespace")]
fn the_table_parser_refuses_a_field_with_surrounding_whitespace() {
    let alpha_src = format!("crates{}alpha{}src{}lib.rs", "/", "/", "/");
    parse_observation_edges(
        &format!("  alpha\tbeta\tcrate-path\t{alpha_src}\n"),
        Path::new("observation_edges.tsv"),
    );
}

/// A row whose witness still carries `:line` is refused with the rule named.
#[test]
#[should_panic(expected = "no `:line` suffix")]
fn the_table_parser_refuses_a_witness_that_carries_a_line() {
    let alpha_src = format!("crates{}alpha{}src{}lib.rs", "/", "/", "/");
    parse_observation_edges(
        &format!("alpha\tbeta\tcrate-path\t{alpha_src}:12\n"),
        Path::new("observation_edges.tsv"),
    );
}

/// The directory that holds more than one member is the one a walk of it
/// observes everything through, and no other.
#[test]
fn the_multi_member_directory_set_is_exactly_the_shared_parents() {
    let multi = multi_member_directories(&fixture_members());
    assert_eq!(
        multi,
        ["crates".to_string()]
            .into_iter()
            .collect::<BTreeSet<String>>(),
        "only `crates` holds more than one member of the fixture workspace; the \
         repository root is excluded because a literal that normalises to \
         nothing is a separator, not a walk"
    );
}

/// Each kind of edge, derived from the shape that produces it.
#[test]
fn each_observation_edge_kind_is_derived_from_its_own_shape() {
    let beta_src = format!("crates{}beta{}src{}lib.rs", "/", "/", "/");

    // (a) A path literal into another member's tree.
    assert_eq!(
        fixture_edges(&format!("let p = root.join(\"{beta_src}\");\n")),
        edge("beta", EDGE_CRATE_PATH, 1)
    );

    // (b) The same read spelled RELATIVE to the observing crate's directory,
    //     which is how this tree spells it.
    assert_eq!(
        fixture_edges("let p = here.join(\"../beta/src/lib.rs\");\n"),
        edge("beta", EDGE_CRATE_PATH, 1)
    );

    // (c) A CHAIN of joins is one path. Without the chain the first link reads
    //     as a walk of the whole crates tree and the package is never
    //     attributed, which is the difference between selecting one package and
    //     selecting all of them.
    assert_eq!(
        fixture_edges("let p = root.join(\"crates\").join(\"beta\").join(\"src\");\n"),
        edge("beta", EDGE_CRATE_PATH, 1)
    );

    // (d) A walk of a tree that holds more than one member.
    assert_eq!(
        fixture_edges("let p = root.join(\"crates\");\n"),
        edge(OBSERVES_EVERYTHING, EDGE_WHOLE_TREE, 1)
    );

    // (e) A `dlopen` of an artifact another member builds.
    assert_eq!(
        fixture_edges("let p = dir.join(\"libgamma.so\");\n"),
        edge("gamma", EDGE_DLOPEN, 1)
    );
    assert_eq!(
        fixture_edges("let p = dir.join(\"libgamma.dylib\");\n"),
        edge("gamma", EDGE_DLOPEN, 1)
    );

    // (f) An artifact name built at RUN TIME cannot be attributed.
    assert_eq!(
        fixture_edges("let p = dir.join(format!(\"lib{name}.so\"));\n"),
        edge(OBSERVES_EVERYTHING, EDGE_UNATTRIBUTABLE, 1)
    );

    // (g) A package segment built at run time cannot be attributed either.
    assert_eq!(
        fixture_edges("let p = root.join(format!(\"crates/{pkg}/Cargo.toml\"));\n"),
        edge(OBSERVES_EVERYTHING, EDGE_UNATTRIBUTABLE, 1)
    );

    // (h) The repository root handed to a call that can read the tree under it,
    //     with no literal anywhere: the shape a whole-tree walk actually has.
    assert_eq!(
        fixture_edges("let root = repo_root();\ncollect_rs(&root, &mut out);\n"),
        edge(OBSERVES_EVERYTHING, EDGE_WHOLE_TREE, 2)
    );

    // (i) A tree that holds EXACTLY ONE member is that member.
    assert_eq!(
        fixture_edges("let p = root.join(\"examples/demo\");\n"),
        edge("delta", EDGE_CRATE_PATH, 1)
    );

    // (j) A manifest directory that has been POPPED points at the directory
    //     that HOLDS the members, so a literal joined onto it names a sibling
    //     crate. Read against the repository root instead, `beta/src/lib.rs`
    //     matches no member and the edge is lost.
    let popped = format!(
        "let mut root = PathBuf::from(env!(\"{CRATE_DIR_ENV}\"));\nroot.pop();\n\
         let p = root.join(\"beta/src/lib.rs\");\n"
    );
    assert_eq!(
        fixture_edges(&popped),
        edge("beta", EDGE_CRATE_PATH, 3),
        "a literal joined onto a popped manifest directory is a sibling crate's \
         path"
    );
    // The other side: the SAME shape naming the observing package's own
    // directory is not an edge, so the parent reading does not manufacture one.
    let own = format!(
        "let mut root = PathBuf::from(env!(\"{CRATE_DIR_ENV}\"));\nroot.pop();\n\
         let p = root.join(\"alpha/src/lib.rs\");\n"
    );
    assert_eq!(
        fixture_edges(&own),
        BTreeSet::<SourceEdge>::new(),
        "a package always observes its own tree, however the path is reached"
    );
    // And the parent reading is NOT blanket: the SAME member name joined onto
    // an ORDINARY receiver derives nothing. The literal resolves under the
    // observing crate's own directory, which is no member, so the walk places
    // it nowhere. Read against the parent it would name `crates/beta` and
    // derive a `beta` edge, which is the reading this row rules out; a literal
    // no member could match rules out nothing.
    assert_eq!(
        fixture_edges("let p = tmp.join(\"beta/src/lib.rs\");\n"),
        BTreeSet::<SourceEdge>::new(),
        "a member's name joined onto an ordinary receiver is no edge"
    );
}

/// The other side of every rule above: the shapes that must derive NOTHING.
#[test]
fn a_shape_that_is_not_an_observation_derives_no_edge() {
    let beta_src = format!("crates{}beta{}src{}lib.rs", "/", "/", "/");
    let alpha_src = format!("crates{}alpha{}src{}lib.rs", "/", "/", "/");
    let nothing = BTreeSet::<SourceEdge>::new();

    for (why, source) in [
        // A path the code only NAMES.
        ("a refusal's wording", format!("assert!(msg.contains(\"{beta_src}\"));\n")),
        // The observing package's OWN tree is not an observation.
        ("the package's own tree", format!("let p = root.join(\"{alpha_src}\");\n")),
        // A path the code WRITES.
        ("a write target", format!("std::fs::write(root.join(\"{beta_src}\"), body).unwrap();\n")),
        // A path joined onto the crate's own directory never leaves the crate.
        (
            "the crate's own directory",
            format!(
                "let crate_root = Path::new(env!(\"{CRATE_DIR_ENV}\"));\n\
                 let p = crate_root.join(\"tests/fixtures/graph.yaml\");\n"
            ),
        ),
        // A separator handed to `[String]::join`, which is not a path join at
        // all: reading it as the repository root made five packages observe
        // everything.
        ("a separator", "let s = parts.join(\"/\");\n".to_string()),
        ("a dot", "let p = Path::new(\".\");\n".to_string()),
        // Prose that opens with a path.
        (
            "prose",
            format!("let why = \"{beta_src} is stale; regenerate it\";\nfn t() {{}}\n"),
        ),
        // The repository root as a `.join(` RECEIVER is classified by the
        // literal it joins, never as a walk.
        (
            "a root receiver",
            "let root = repo_root();\nlet p = root.join(\"docs/guide.md\");\n".to_string(),
        ),
        // A DECLARATION is not a use, even in a source that binds the root.
        (
            "a parameter list",
            "let root = repo_root();\nfn helper(root: &Path) -> usize { 0 }\n".to_string(),
        ),
        // The root printed rather than walked.
        (
            "a message",
            "let root = repo_root();\npanic!(\"missing under {}\", root.display());\n".to_string(),
        ),
        // An interpolated path that names no member tree and no artifact is an
        // ordinary run-time path, almost always inside a scratch directory.
        (
            "a scratch path",
            "let p = tmp.join(format!(\"{name}/graph.yaml\"));\n".to_string(),
        ),
        // A shared object no member of this workspace builds.
        ("a foreign artifact", "let p = dir.join(\"libsomething_else.so\");\n".to_string()),
        // A const table entry that is a bare NAME rather than a path: a
        // classification key, not a read.
        (
            "a bare directory name in a table",
            "const ROWS: &[(&str, &str)] = &[(\"examples\", \"why\")];\nfn t() { use_rows(ROWS); }\n"
                .to_string(),
        ),
    ] {
        assert_eq!(
            fixture_edges(&source),
            nothing,
            "{why} must derive no observation edge:\n{source}"
        );
    }

    // The positive control for the last row: the SAME table with a real path in
    // it does derive one, so the rule refuses bare names and nothing else.
    let table = format!(
        "const ROWS: &[&str] = &[\"crates{}beta{}src{}lib.rs\"];\nfn t() {{ use_rows(ROWS); }}\n",
        "/", "/", "/"
    );
    assert_eq!(fixture_edges(&table), edge("beta", EDGE_CRATE_PATH, 1));
}

// ---------------------------------------------------------------------------
// A gated step's doc-pin marker abuts the step it pins.
// ---------------------------------------------------------------------------

/// The opening of the per-package selection gate `ci.yml` spells.
///
/// Held here as a literal because that is the only form
/// `cerulion_cli_engine::ci_test_coverage_test` credits: a step gated on
/// anything else is not a gated step and owes no adjacency.
const SELECTION_GATE_OPEN: &str = "contains(fromJSON(needs.changes.outputs.pkgs), '";

/// Is this workflow line the per-package selection gate for `package`?
fn line_gates_package(line: &str, package: &str) -> bool {
    let line = line.trim();
    if line.starts_with('#') {
        return false;
    }
    line.contains(&format!("{SELECTION_GATE_OPEN}{package}'"))
}

/// The line index of every step a job block opens, at the `steps:` indent.
///
/// Spelled once and shared, because the step index is what decides whether this
/// rule has anything to judge: a block whose steps this reader cannot find has
/// no gated marker, so every marker passes and the live arm reports a clean
/// tree it never looked at. The count of gated markers below is read off the
/// SAME index for that reason.
fn step_starts_of(lines: &[&str]) -> Vec<usize> {
    (0..lines.len())
        .filter(|i| lines[*i].starts_with("      - ") && !lines[*i].starts_with("       "))
        .collect()
}

/// Every doc-pin marker of this block whose package is GATED on the selection
/// by a step of the same job, as `(line index, the marker line, the package)`.
fn gated_doc_pin_markers(lines: &[&str]) -> Vec<(usize, String, String)> {
    let step_at = step_starts_of(lines);
    let step_end = |k: usize| -> usize {
        step_at
            .iter()
            .copied()
            .find(|&j| j > step_at[k])
            .unwrap_or(lines.len())
    };
    let mut out = Vec::new();
    for (i, line) in lines.iter().enumerate() {
        let trimmed = line.trim();
        if !trimmed.starts_with(MARKER_PREFIX) {
            continue;
        }
        let Some(package) = marker_package(trimmed) else {
            continue;
        };
        let gated = (0..step_at.len()).any(|k| {
            let body = &lines[step_at[k]..step_end(k)];
            body.iter().any(|l| line_names_package(l, package))
                && body.iter().any(|l| line_gates_package(l, package))
        });
        if gated {
            out.push((i, trimmed.to_string(), package.to_string()));
        }
    }
    out
}

/// Every complaint about a doc-pin marker that does not abut the GATED step it
/// pins.
///
/// A marker beside another step of the same job is enough while nothing is
/// gated: the job runs the package either way. Once a step is gated on the
/// selection, the marker is what a path-to-step map reads to decide that THIS
/// step has to run, and a marker parked beside a different step of the same job
/// points at the wrong condition.
///
/// A marker for a package no step of the job gates is left alone, which is the
/// state every marker in `ci.yml` was in before the gates existed.
fn doc_pin_adjacency_complaints(job: &str, block: &str) -> Vec<String> {
    let lines: Vec<&str> = block.lines().collect();
    let step_at = step_starts_of(&lines);
    let step_end = |k: usize| -> usize {
        step_at
            .iter()
            .copied()
            .find(|&j| j > step_at[k])
            .unwrap_or(lines.len())
    };

    let mut out = Vec::new();
    for (i, trimmed, package) in gated_doc_pin_markers(&lines) {
        let abutting = step_at.iter().copied().find(|&j| j > i).filter(|&j| {
            lines[i + 1..j]
                .iter()
                .all(|l| l.trim().is_empty() || l.trim().starts_with('#'))
        });
        let names_it = abutting.is_some_and(|j| {
            let k = step_at
                .iter()
                .position(|&s| s == j)
                .expect("j is a step start");
            lines[j..step_end(k)]
                .iter()
                .any(|l| line_names_package(l, &package))
        });
        if !names_it {
            out.push(format!(
                "  {job}: `{trimmed}` is the pin of a step GATED on the \
                 selection, so it has to sit directly above THAT step. Nothing \
                 but comments may come between them."
            ));
        }
    }
    out
}

/// The floor of gated doc-pin markers the shipped workflow carries.
///
/// The rule is satisfied by finding nothing, so the count is asserted beside
/// it. The step reader keys on the `steps:` indent, so a reindent of the
/// workflow, or a marker prefix that stops matching, leaves the walk with no
/// gated marker to judge and every complaint list empty: a clean verdict over a
/// tree nothing read. A floor turns that into a red.
///
/// TWO, derived from the tree rather than guessed: `ci.yml` carries eighteen
/// doc-pin markers and two of them pin a step gated on the selection,
/// `rmw_cerulion` and `native_ros2_messages`, both in `test-linux`. Every other
/// marker pins a step gated on `matrix.shard` alone, or on nothing, and owes no
/// adjacency.
const GATED_DOC_PIN_MARKER_FLOOR: usize = 2;

/// The live workflow: every gated step's doc-pin marker abuts it.
#[test]
fn a_gated_steps_doc_pin_marker_abuts_the_step_it_pins() {
    let text = ci_workflow_text();
    let mut complaints = Vec::new();
    let mut judged = 0usize;
    for (job, block) in jobs_of(&text) {
        let lines: Vec<&str> = block.lines().collect();
        judged += gated_doc_pin_markers(&lines).len();
        complaints.extend(doc_pin_adjacency_complaints(&job, &block));
    }
    assert!(
        complaints.is_empty(),
        "doc-pin markers beside gated steps are in the wrong place:\n{}",
        complaints.join("\n")
    );
    assert!(
        judged >= GATED_DOC_PIN_MARKER_FLOOR,
        "the walk judged {judged} gated doc-pin marker(s) and the workflow \
         carries at least {GATED_DOC_PIN_MARKER_FLOOR}. An EMPTY set is not a \
         clean tree: this reader finds a step by its `steps:` indent and a \
         marker by its prefix, so a reindent or a renamed prefix leaves it \
         nothing to judge and every complaint list empty. Re-derive the floor \
         only when the workflow really loses gated steps."
    );
}

/// The adjacency rule, both sides, on synthetic jobs.
#[test]
fn a_marker_parked_away_from_its_gated_step_is_a_red_walker() {
    let gate = format!("        if: {SELECTION_GATE_OPEN}pkg')\n");
    let marker = format!("      {MARKER_PREFIX} pkg::bin{MARKER_VERB}docs\n");
    let gated_step = format!("{gate}        run: cargo test -p pkg\n");
    let other = "      - name: something else\n        run: true\n";

    // Adjacent, with a comment between: legal.
    let good =
        format!("  j:\n    steps:\n{marker}      # a note\n      - name: pkg tests\n{gated_step}");
    assert_eq!(
        doc_pin_adjacency_complaints("j", &good),
        Vec::<String>::new(),
        "a marker directly above its gated step, comments allowed, is legal"
    );

    // Parked beside ANOTHER step of the same job: refused.
    let bad = format!(
        "  j:\n    steps:\n{marker}      - name: something else\n        run: true\n      \
         - name: pkg tests\n{gated_step}"
    );
    assert_eq!(
        doc_pin_adjacency_complaints("j", &bad).len(),
        1,
        "a marker beside a step that does not run its package must be refused"
    );

    // The same layout with the step UNGATED: legal, because the job runs the
    // package on every event and no condition reads the marker.
    let ungated = format!(
        "  j:\n    steps:\n{marker}{other}      - name: pkg tests\n        run: cargo test -p pkg\n"
    );
    assert_eq!(
        doc_pin_adjacency_complaints("j", &ungated),
        Vec::<String>::new(),
        "a marker in a job whose steps are ungated owes no adjacency"
    );

    // A gate on a DIFFERENT package does not make this marker's step gated.
    let other_gate = format!(
        "  j:\n    steps:\n{marker}{other}      - name: pkg tests\n        if: \
         {SELECTION_GATE_OPEN}elsewhere')\n        run: cargo test -p pkg\n"
    );
    assert_eq!(
        doc_pin_adjacency_complaints("j", &other_gate),
        Vec::<String>::new(),
        "the gate has to name the marker's own package"
    );
}
