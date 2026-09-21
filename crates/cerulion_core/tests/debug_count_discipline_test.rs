// SPDX-License-Identifier: AGPL-3.0-only
//! The release-safe DEBUG-count discipline as a GATE instead of prose.
//!
//! # The class
//!
//! `cerulion_core` enables `tracing/release_max_level_info`, and Cargo unifies
//! that feature across every binary linking the crate, so a `--release` build
//! compiles every `debug!` out: `STATIC_MAX_LEVEL` is `INFO`, and a test that
//! counts DEBUG-level lines reads 0 no matter what the code under test did. A
//! test demanding a NONZERO DEBUG count therefore fails in release
//! deterministically — and it fails in exactly ONE CI job, `Latency Threshold
//! (Linux)`, the only job running `cargo test -p cerulion_core --lib --release`,
//! which runs on push to main (plus a manual `workflow_dispatch`) only. Every
//! PR shard is a debug profile: the PR is green, main goes red at the merge.
//!
//! That has happened TWICE by the same route — `codegen::unknown_hash` (fixed
//! in 62191c47e) and `flashback::channel`'s later arms
//! (`ce27b6eb8`: 26 consecutive red main runs, blocking the
//! 2e soak gate and hiding every other main regression behind it).
//! Both fixes were the same shape, and both were per-instance. Adding a release
//! build to PR CI was rejected on cost, so this walk is the cheap structural
//! substitute: it makes the release-safe shape a RULE over the four trees of the
//! two crates that hit it — `cerulion_core` and `rmw_cerulion`, src AND tests —
//! enforced on every PR in a debug profile.
//!
//! SCOPE: the same shape exists, UNGATED, outside those roots — in the
//! test code (unit tests inside `src/` as well as `tests/`) of
//! `cerulion_bagd`, `cerulion_netd`, `cerulion_viz` and `cerulion_cli_engine`,
//! plus a `cfg!(debug_assertions)`-keyed copy of the helper this walk replaced
//! (`cerulion_viz/lib/cerulion_viz/tests/unknown_hash_diagnostic_test.rs`). No
//! hard count is written here: a number in prose rots with every unrelated
//! commit and nothing gates it. Those crates have no release CI job today, so
//! the class is latent rather than red there; extending `ROOTS` to them needs
//! the `test-helpers` feature on each crate's `cerulion_core` dev-dependency
//! (the gate helpers live behind it) and is deliberately not done
//! in this file.
//!
//! # The rule
//!
//! Every DEBUG-count assertion must route its expectation through
//! [`cerulion_core::testing::debug_lines_expected`] (a count) or
//! [`cerulion_core::testing::debug_level_compiled_in`] (a presence check; its
//! TRACE twin is `trace_level_compiled_in`) — the ONE helper set keyed on
//! tracing's own static gate. The walk reads the real
//! source of `cerulion_core/{src,tests}` and `rmw_cerulion/{src,tests}` and
//! fails naming each offending `file:line` and its enclosing `fn`.
//!
//! # The grammar, stated precisely
//!
//! Over a COMMENT-STRIPPED view of each file (line and depth-tracked nested
//! block comments removed; string, raw-string, byte-string and char literals
//! KEPT, since the token of interest is a string literal; every unterminated
//! comment or literal is a PANIC, never a truncated view — see [`strip`]):
//!
//! * A **level literal** is one of the five whole string literals `"TRACE"`,
//!   `"DEBUG"`, `"INFO"`, `"WARN"`, `"ERROR"`.
//! * A **level list** is a `,`- or `|`-separated RUN of three or more level
//!   literals — the `matches!(t, "TRACE" | "DEBUG" | …)` arm of a `level_of`
//!   helper, or a `["ERROR", "WARN", "INFO", "DEBUG", "TRACE"]` table. A
//!   `"DEBUG"` inside such a run selects nothing and is EXEMPT; the exemption
//!   is per-occurrence, so three level literals elsewhere on the same line
//!   shield nothing.
//! * A **DEBUG site** is any other occurrence of the literal `"DEBUG"` that is
//!   immediately preceded (after optional whitespace, NEWLINES included — a
//!   rustfmt-wrapped argument first on its own line is still a site) by `(`,
//!   `,`, `=`, `[` or `|` — i.e. passed as an argument (`count_at(lines,
//!   "DEBUG", m)`, `l.contains("DEBUG")`, `count_at!("DEBUG", m)`,
//!   `("DEBUG", MARKER)`, `["DEBUG"]`) or compared (`level == "DEBUG"`). Every
//!   DEBUG-line filter in these trees is one of those shapes; a `"DEBUG"` in
//!   any other position (a match arm returning a level NAME, prose inside a
//!   larger literal) is not a site.
//! * An **absence site** is a site that is a genuine SILENCE CONTROL — "nothing
//!   at DEBUG was logged", release-safe by construction since a DEBUG count is
//!   0 in both profiles — so it needs no gate and is EXEMPT. Two things must
//!   hold. The zero predicate must be ATTACHED: walking outward from the call
//!   holding the token through closing parens and `.len()`/`.count()`/
//!   `.collect…()` hops, the STATEMENT continues with `.is_empty()`, `!= 0`,
//!   `== 0` or the `, 0` of an `assert_eq!`/`assert_ne!`; or the call chain is
//!   immediately negated (`!logs_contain(m)`, `!x.is_empty()`); or the site is
//!   a `let NAME = …;` statement whose binding the fn later compares to zero
//!   by THAT binding and to nothing else. And the predicate's POLARITY in its
//!   context must actually claim zero: `assert!(count == 0)`,
//!   `assert_eq!(count, 0)`, `if count != 0 { Err }`, `if count == 0 { Ok }`
//!   and `assert!(!logs_contain(m))` do; `assert!(count != 0)`,
//!   `assert_ne!(count, 0)`, `assert!(!x.is_empty())` and `if count == 0 {
//!   Err }` DEMAND a DEBUG line — the very class this walk exists for, in a
//!   silence control's clothing — and exempt nothing, nor does any context the
//!   walk does not model (a bare `let ok = n == 0;`, a `while`, a `match`).
//!   It is the statement that is inspected, not the line — rustfmt wrapping a
//!   chain changes nothing — and a zero check that is not on the attached path
//!   (`count == 5 && other == 0`) lends nothing: rewrite it as a predicate on
//!   the expression, or gate it. The walk never guesses beyond these.
//! * A literal in MESSAGE position — the argument of `assert!`/`assert_eq!`/
//!   `panic!`/`expect`/`format!`/… — is prose, never a matcher, and is not a
//!   marker site.
//! * EVERY site on a line is classified on its own — two counts sharing a
//!   statement, or a count beside a marker, are two sites, each with its own
//!   absence, markers (a rule-1 site's are the values in its OWN call, else its
//!   statement's), gate and twin. One site per line would miss them.
//! * A site's **enclosing fn** is the innermost `fn` whose body braces contain
//!   it, computed on a view with literal bodies blanked (format strings carry
//!   `{`/`}`) — closures and `logs_assert` bodies belong to the fn they sit in.
//!   A site outside any fn is a violation outright.
//! * A **TRACE/DEBUG-only marker site** is the same class in a different coat: a
//!   line carrying a string literal (or an identifier naming a same-file
//!   `const … : &str`) that is a substring of some `trace!` or `debug!` MESSAGE
//!   in the walked trees and of no louder level's message — a presence check on it
//!   (`lines.iter().find(|l| l.contains("decode failure suppressed"))…?`) can
//!   never pass where `debug!` is compiled out, and it names no level literal
//!   for the first rule to see. Messages are the LAST string literal inside
//!   each `trace!/debug!/info!/warn!/error!(…)` invocation, continuations
//!   joined; a marker shorter than [`MIN_MARKER_LEN`] is never cross-referenced;
//!   the emit itself and the `const` definition are not sites. This rule
//!   exists for a class only a RELEASE run exposes, e.g. running
//!   `rmw_cerulion --lib` in release: `decode_failure_latch`'s
//!   field-name pin asserts the suppressed repeat's line is present.
//! * A gated site must also be **twinned**: the same fn must hold a loud-level
//!   SWEEP naming THAT SITE'S MARKER — a call to a helper whose name ends in
//!   `never_loud` whose arguments carry the marker, or a `"WARN"` literal whose
//!   sweep (the `for level in [...] { … }` block or the `.filter(|l| …)`
//!   statement it opens) also carries `"INFO"`, `"ERROR"` and the marker. A
//!   rule-2 site's marker is its own; a rule-1 site's markers are the non-level
//!   literals and same-file `const`s in its statement; literal and `const`
//!   forms match by VALUE, and a `const` resolves to the declaration VISIBLE at
//!   the use under one lexical rule for the whole file: in scope only inside its
//!   innermost enclosing block (a top-level one everywhere), innermost wins —
//!   so a nested block's same-named `const` never pairs an outer site with an
//!   inner sweep. Three loud literals in an unrelated
//!   message, or a sweep for a different marker, name nothing for the site. A
//!   site with no resolvable marker (a `level == "DEBUG"` control-flow gate)
//!   falls back to ANY sweep in the fn. In release the gate skips the only assertion, so without the twin a
//!   suppressed arm promoted to `warn!` passes; the twin is what fails it.
//!   `decode_failure_latch`'s gated pin is
//!   exactly that shape.
//! * A site is **gated** iff its enclosing fn's body names `debug_lines_expected(`,
//!   `debug_level_compiled_in(` or `trace_level_compiled_in(` OUTSIDE any literal (matched on the blanked
//!   view, so a helper name quoted in a message is not a call). Otherwise it is
//!   a VIOLATION.
//!
//! # What the walk cannot see — each boundary pinned by a probe
//!
//! Pinned in `the_walks_blind_spots_are_pinned_on_both_sides`, so a change to
//! any of them is deliberate: a marker built at runtime (`logs_contain(&format!
//! ("total={N}"))`) and a `const` imported with `use` from another module are
//! INVISIBLE (gate them by hand); a helper fn that returns a DEBUG count for a
//! caller to compare is caught AT THE HELPER (its body holds the selector, so
//! the helper must carry the gate — strict, but never silent); a
//! `macro_rules!` body holding a selector is flagged as a site outside any fn;
//! a `#[cfg(debug_assertions)]` fn is EXEMPT (it does not exist in release, so
//! it can be neither release-unsafe nor a release pin). Covered, and pinned
//! alongside: a raw-string `r"DEBUG"`, a level held in a `const`/`static`/`let`,
//! the `Level::DEBUG` path, a list element (`for level in ["DEBUG"]`), a marker
//! held in a `let`, iterator adaptors, multi-line statements, nested closures,
//! and unit tests inside `src/`.
//!
//! # Scope
//!
//! The scope is the FN BODY, not the assertion: a fn holding one gated and one
//! ungated site passes. That is the trade every walk in this repo makes (the
//! `cerulion_node_init` walk requires the log-level call in the fn body, not on
//! a particular line), and it targets the failure mode that actually recurs
//! — a NEW test written from scratch with no gate anywhere. A helper fn that
//! counts DEBUG lines and returns the number for a caller to compare
//! unconditionally would also escape; none exists in the four trees today.
//! The second rule sees only LITERAL and same-file-`const` markers: a marker
//! built at runtime (`logs_contain(&format!("total={N}"))` on a `debug!` line)
//! is invisible to it and must be gated by hand — `backpressure_event_iox2_test`
//! carries two, found only by running that file in release.
//!
//! Parallel-safe — pure file parse, no transport.

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

/// The five level tokens `tracing-test` renders, as string literals appear in
/// test source (quotes included).
const LEVEL_LITERALS: [&str; 5] = [
    "\"TRACE\"",
    "\"DEBUG\"",
    "\"INFO\"",
    "\"WARN\"",
    "\"ERROR\"",
];

/// The ONE sanctioned helper set — see `cerulion_core::testing`.
const GATE_HELPERS: [&str; 3] = [
    "debug_lines_expected(",
    "debug_level_compiled_in(",
    "trace_level_compiled_in(",
];

/// What a zero predicate CLAIMS about the count it is attached to.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ZeroClaim {
    /// `== 0`, `.is_empty()`, `assert_eq!(…, 0)`, a negated boolean presence
    /// (`!logs_contain(m)`): "nothing was logged".
    Zero,
    /// `!= 0`, `!x.is_empty()`, `assert_ne!(…, 0)`: a NONZERO demand — the
    /// exact class the walk exists for, in a silence control's clothing.
    NonZero,
}

/// Which way the surrounding control flow turns a claim.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum PredicateContext {
    /// The argument of `assert!`/`debug_assert!`, the compared side of an
    /// `assert_eq!`/`assert_ne!` against `0`, or the condition of an `if` whose
    /// block opens by PASSING (`Ok(`/`return Ok`): the claim is ASSERTED.
    Asserted,
    /// The condition of an `if` whose block opens by FAILING (`Err(`,
    /// `return Err`, `panic!`, `unreachable!`): the claim is REFUTED.
    Refuted,
    /// Anything the walk does not model — exempts nothing (fail closed).
    Unknown,
}

/// A genuine silence control asserts "nothing at DEBUG" or refutes "something
/// at DEBUG" — release-safe by construction, since a DEBUG count is 0 in both
/// profiles. The other two combinations DEMAND a nonzero DEBUG count and read
/// 0 in release; a walk that exempts them by token shape alone is wrong.
fn is_silence_control(claim: ZeroClaim, ctx: PredicateContext) -> bool {
    matches!(
        (claim, ctx),
        (ZeroClaim::Zero, PredicateContext::Asserted)
            | (ZeroClaim::NonZero, PredicateContext::Refuted)
    )
}

/// Iterator adaptors a predicate may sit inside without changing which
/// assertion owns it (`assert!(lines.iter().all(|l| !l.contains(M)))`).
const TRANSPARENT_CALLEES: [&str; 6] = ["all", "any", "filter", "find", "position", "map"];

/// The callee of the innermost unmatched `(` before `pos` within the
/// statement, with the `(`'s offset: `("assert!", open)` for
/// `assert!(count == 0)`, `(".all", open)` for `.all(|l| …)`.
fn enclosing_callee(text: &str, pos: usize) -> Option<(&str, usize)> {
    let b = text.as_bytes();
    let mut depth = 0i32;
    let mut k = pos;
    while k > 0 {
        k -= 1;
        match b[k] {
            b')' => depth += 1,
            b'(' if depth == 0 => {
                let start = callee_chain_start(b, k);
                return Some((&text[start..k], k));
            }
            b'(' => depth -= 1,
            b';' | b'{' | b'}' => return None,
            _ => {}
        }
    }
    None
}

/// The context of the claim spanning `[from, to)` of `text`: walk outward
/// through the enclosing calls — `assert!`/`debug_assert!` asserts it, an
/// iterator adaptor is transparent, any other callee is unmodelled — and,
/// outside every call, read the statement head: an `if` (or `else if`) whose
/// block opens with `Ok(`/`return Ok` asserts the claim, one opening with
/// `Err(`/`return Err`/`panic!`/`unreachable!` refutes it.
fn predicate_context(text: &str, from: usize, to: usize) -> PredicateContext {
    let mut probe = from;
    while let Some((callee, open)) = enclosing_callee(text, probe) {
        let name = callee.rsplit(['.', ':']).next().unwrap_or(callee);
        match name {
            "assert!" | "debug_assert!" => return PredicateContext::Asserted,
            n if TRANSPARENT_CALLEES.contains(&n) => probe = open,
            _ => return PredicateContext::Unknown,
        }
    }
    let head = text[statement_start(text, from, 0)..from].trim_start();
    let head = head.strip_prefix("else").map_or(head, str::trim_start);
    if !(head.starts_with("if ") || head.starts_with("if(")) {
        return PredicateContext::Unknown;
    }
    // The block the condition opens: forward over balanced brackets (a closure
    // body inside the condition is balanced) to the first `{` at depth 0.
    let b = text.as_bytes();
    let mut depth = 0i32;
    let mut i = to;
    let brace = loop {
        if i >= b.len() {
            return PredicateContext::Unknown;
        }
        match b[i] {
            b'(' | b'[' => depth += 1,
            b')' | b']' => depth -= 1,
            b'{' if depth == 0 => break i,
            b'{' => depth += 1,
            b'}' => depth -= 1,
            b';' if depth == 0 => return PredicateContext::Unknown,
            _ => {}
        }
        i += 1;
    };
    let body = text[brace + 1..].trim_start();
    if [
        "Err(",
        "return Err",
        "panic!(",
        "unreachable!(",
        "assert!(false",
    ]
    .iter()
    .any(|h| body.starts_with(h))
    {
        PredicateContext::Refuted
    } else if body.starts_with("Ok(") || body.starts_with("return Ok") {
        PredicateContext::Asserted
    } else {
        PredicateContext::Unknown
    }
}

/// `, 0` followed by `)` or `,` — the zero argument of `assert_eq!(x, 0)` /
/// `assert_ne!(x, 0)` / `assert_eq!(x, 0, "msg")`, whitespace (rustfmt puts
/// each argument on its own line) tolerated. Returns what follows the `0`.
fn strip_zero_argument(rest: &str) -> Option<&str> {
    let r = rest.strip_prefix(',')?.trim_start().strip_prefix('0')?;
    r.trim_start().starts_with([')', ',']).then_some(r)
}

/// The trees under the gate, relative to the workspace root.
const ROOTS: [&str; 4] = [
    "cerulion_core/src",
    "cerulion_core/tests",
    "rmw_cerulion/src",
    "rmw_cerulion/tests",
];

/// This file holds `"DEBUG"` sites of its own — the classifier oracles below
/// feed it synthetic snippets — so it is excluded from the walk by name, and
/// the anti-tautology arm asserts the exclusion actually fired.
const SELF_FILE: &str = "cerulion_core/tests/debug_count_discipline_test.rs";

fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cerulion_core sits one level below the workspace root")
        .to_path_buf()
}

/// Two views of one source text with IDENTICAL byte offsets: `code` has
/// comments removed and literals kept; `structure` additionally has every
/// literal BODY replaced by spaces (newlines kept), so brace tracking cannot be
/// fooled by a `{want}` format placeholder.
struct Views {
    code: String,
    structure: String,
}

/// Strip comments; keep or blank literals. FAILS CLOSED on any unterminated
/// comment or literal, because every assertion in this file is over the whole
/// file and a silently truncated view would police only the prefix it read.
fn strip(src: &str) -> Views {
    let b = src.as_bytes();
    // Byte buffers, not `String`s: a multi-byte char inside a literal must blank to
    // the SAME number of bytes it occupied, or the two views lose offset parity.
    let mut code: Vec<u8> = Vec::with_capacity(src.len());
    let mut structure: Vec<u8> = Vec::with_capacity(src.len());
    let ident_byte = |c: u8| c.is_ascii_alphanumeric() || c == b'_';
    // Emit one byte to both views; `blank` decides whether `structure` sees it.
    let mut emit = |c: u8, blank: bool| {
        code.push(c);
        structure.push(if blank && c != b'\n' { b' ' } else { c });
    };
    let mut i = 0usize;
    while i < b.len() {
        // ---- comments ----
        if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'/' {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
            let mut depth = 1usize;
            i += 2;
            while i < b.len() && depth > 0 {
                if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
                    depth += 1;
                    i += 2;
                } else if b[i] == b'*' && i + 1 < b.len() && b[i + 1] == b'/' {
                    depth -= 1;
                    i += 2;
                } else {
                    if b[i] == b'\n' {
                        emit(b'\n', false);
                    }
                    i += 1;
                }
            }
            assert_eq!(
                depth, 0,
                "unterminated block comment — the view would be a PREFIX"
            );
            continue;
        }
        // ---- raw strings: r"…", r#"…"#, br##"…"## (no escapes) ----
        {
            let prev_is_ident = i > 0 && ident_byte(b[i - 1]);
            let mut j = i;
            if !prev_is_ident && j < b.len() && b[j] == b'b' {
                j += 1;
            }
            if !prev_is_ident && j < b.len() && b[j] == b'r' {
                let hash_start = j + 1;
                let mut hashes = hash_start;
                while hashes < b.len() && b[hashes] == b'#' {
                    hashes += 1;
                }
                if hashes < b.len() && b[hashes] == b'"' {
                    let n = hashes - hash_start;
                    for &c in &b[i..=hashes] {
                        emit(c, false);
                    }
                    let mut k = hashes + 1;
                    let mut closed = false;
                    while k < b.len() {
                        if b[k] == b'"' {
                            let mut m = 0usize;
                            while m < n && k + 1 + m < b.len() && b[k + 1 + m] == b'#' {
                                m += 1;
                            }
                            if m == n {
                                for &c in &b[k..k + 1 + n] {
                                    emit(c, false);
                                }
                                k += 1 + n;
                                closed = true;
                                break;
                            }
                        }
                        emit(b[k], true);
                        k += 1;
                    }
                    assert!(
                        closed,
                        "unterminated raw string — the view would be a PREFIX"
                    );
                    i = k;
                    continue;
                }
            }
        }
        // ---- plain / byte strings ----
        if b[i] == b'"' {
            emit(b'"', false);
            i += 1;
            let mut closed = false;
            while i < b.len() {
                if b[i] == b'\\' && i + 1 < b.len() {
                    emit(b[i], true);
                    emit(b[i + 1], true);
                    i += 2;
                    continue;
                }
                if b[i] == b'"' {
                    emit(b'"', false);
                    i += 1;
                    closed = true;
                    break;
                }
                emit(b[i], true);
                i += 1;
            }
            assert!(closed, "unterminated string — the view would be a PREFIX");
            continue;
        }
        // ---- char literals vs lifetimes ----
        if b[i] == b'\'' {
            let is_char = if i + 1 < b.len() && b[i + 1] == b'\\' {
                true
            } else {
                i + 2 < b.len() && b[i + 2] == b'\''
            };
            if is_char {
                emit(b'\'', false);
                i += 1;
                let mut closed = false;
                while i < b.len() {
                    if b[i] == b'\\' && i + 1 < b.len() {
                        emit(b[i], true);
                        emit(b[i + 1], true);
                        i += 2;
                        continue;
                    }
                    if b[i] == b'\'' {
                        emit(b'\'', false);
                        i += 1;
                        closed = true;
                        break;
                    }
                    emit(b[i], true);
                    i += 1;
                }
                assert!(
                    closed,
                    "unterminated char literal — the view would be a PREFIX"
                );
                continue;
            }
        }
        emit(b[i], false);
        i += 1;
    }
    assert_eq!(
        code.len(),
        structure.len(),
        "the two views must share byte offsets"
    );
    assert_eq!(
        src.bytes().filter(|&c| c == b'\n').count(),
        code.iter().filter(|&&c| c == b'\n').count(),
        "every newline of the source must survive stripping: reported line numbers depend on it"
    );
    Views {
        code: String::from_utf8(code).expect("comment removal drops whole chars only"),
        structure: String::from_utf8(structure).expect("blanking replaces bytes with ASCII"),
    }
}

/// One `fn` with a body, by byte offset into the structure view.
struct FnRange {
    /// Its `{` and its matching `}`.
    body: (usize, usize),
    name: String,
    /// A `#[cfg(debug_assertions)]` attribute precedes the `fn`: it does not
    /// exist in release, so nothing in it is a site.
    debug_only: bool,
}

impl FnRange {
    fn contains(&self, pos: usize) -> bool {
        self.body.0 < pos && pos < self.body.1
    }
}

/// Every `fn` with a body.
fn fn_ranges(structure: &str) -> Vec<FnRange> {
    let b = structure.as_bytes();
    let ident_byte = |c: u8| c.is_ascii_alphanumeric() || c == b'_';
    let mut depth = 0i64;
    // Declarations seen but whose body brace has not opened yet: (name, depth,
    // debug-only — a `#[cfg(debug_assertions)]` attribute precedes the `fn`).
    let mut pending: Vec<(String, i64, bool)> = Vec::new();
    // Bodies currently open: (start, decl depth, name, debug-only).
    let mut open: Vec<(usize, i64, String, bool)> = Vec::new();
    let mut out = Vec::new();
    let mut i = 0usize;
    while i < b.len() {
        let at_boundary = i == 0 || !ident_byte(b[i - 1]);
        if at_boundary
            && b[i..].starts_with(b"fn")
            && i + 2 < b.len()
            && b[i + 2].is_ascii_whitespace()
        {
            let mut j = i + 2;
            while j < b.len() && b[j].is_ascii_whitespace() {
                j += 1;
            }
            let name_start = j;
            while j < b.len() && ident_byte(b[j]) {
                j += 1;
            }
            let name = String::from_utf8_lossy(&b[name_start..j]).into_owned();
            // The attributes before this `fn`: back to the previous item's `}`
            // or `;`. A `#[cfg(debug_assertions)]` there means the fn does not
            // exist in release at all.
            let mut a = i;
            while a > 0 && !matches!(b[a - 1], b'}' | b';') {
                a -= 1;
            }
            let debug_only = structure[a..i].contains("cfg(debug_assertions)");
            pending.push((name, depth, debug_only));
            i = j;
            continue;
        }
        match b[i] {
            b'{' => {
                if let Some((_, d, _)) = pending.last() {
                    if *d == depth {
                        let (name, _, debug_only) = pending.pop().expect("just peeked");
                        open.push((i, depth, name, debug_only));
                    }
                }
                depth += 1;
            }
            b'}' => {
                depth -= 1;
                if let Some((_, d, _, _)) = open.last() {
                    if *d == depth {
                        let (start, _, name, debug_only) = open.pop().expect("just peeked");
                        out.push(FnRange {
                            body: (start, i),
                            name,
                            debug_only,
                        });
                    }
                }
            }
            b';' => {
                if let Some((_, d, _)) = pending.last() {
                    if *d == depth {
                        pending.pop();
                    }
                }
            }
            _ => {}
        }
        i += 1;
    }
    out
}

/// One site in a file, classified.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Site {
    /// 1-based line number.
    line: usize,
    /// The trimmed source line.
    text: String,
    /// Which rule fired: the `"DEBUG"` level literal, or a DEBUG-ONLY marker.
    kind: String,
    /// Expected-zero ATTACHED to the count — exempt.
    absence: bool,
    /// The innermost enclosing fn, if any.
    fn_name: Option<String>,
    /// Whether that fn's body names a gate helper (outside any literal).
    gated: bool,
    /// Whether that fn's body also holds the LEVEL-FREE TWIN — a loud-level
    /// sweep asserting THIS site's marker never appears at WARN/INFO/ERROR.
    twinned: bool,
    /// The site's marker VALUES (a rule-2 site's own marker; a rule-1 site's
    /// non-level literals and same-file `const`s within its statement).
    markers: Vec<String>,
}

/// A site's verdict — ONE of four, so the two reported classes can never
/// overlap and an absence is always exempt from both.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    /// An expected-zero: release-safe by construction.
    Exempt,
    /// No gate helper in the enclosing fn: release-unsafe.
    Ungated,
    /// Gated but alone: in release the gate skips the only assertion, so a
    /// suppressed arm promoted to `warn!` passes — the twin is what fails it.
    GatedTwinless,
    Ok,
}

impl Site {
    fn verdict(&self) -> Verdict {
        match (self.absence, self.gated, self.twinned) {
            (true, _, _) => Verdict::Exempt,
            (false, false, _) => Verdict::Ungated,
            (false, true, false) => Verdict::GatedTwinless,
            (false, true, true) => Verdict::Ok,
        }
    }
    fn violates(&self) -> bool {
        self.verdict() == Verdict::Ungated
    }
    fn missing_twin(&self) -> bool {
        self.verdict() == Verdict::GatedTwinless
    }
}

/// The five level names, as runtime VALUES — never a marker.
const LEVEL_VALUES: [&str; 5] = ["TRACE", "DEBUG", "INFO", "WARN", "ERROR"];

/// Every marker-ish VALUE inside `structure[a..b)` of `facts`: each string
/// literal's runtime value, and each identifier that names a `const … : &str`
/// visible at that identifier's position (see [`resolve_const`]).
fn values_in(facts: &FileFacts, a: usize, b: usize) -> Vec<String> {
    let mut out: Vec<String> = facts
        .spans
        .iter()
        .zip(&facts.span_values)
        .filter(|((s, e), _)| *s >= a && *e < b)
        .map(|(_, v)| v.clone())
        .collect();
    let st = &facts.views.structure[a..b];
    let sb = st.as_bytes();
    let mut pos = 0usize;
    while pos < sb.len() {
        if sb[pos].is_ascii_alphabetic() || sb[pos] == b'_' {
            let start = pos;
            while pos < sb.len() && (sb[pos].is_ascii_alphanumeric() || sb[pos] == b'_') {
                pos += 1;
            }
            if let Some(v) = resolve_const(facts, &st[start..pos], a + start) {
                out.push(v);
            }
        } else {
            pos += 1;
        }
    }
    out
}

/// The innermost `{ … }` block holding `pos`, as `(open, close)`; `None` at
/// file top level (a declaration there is visible everywhere in the file).
fn enclosing_block(structure: &str, pos: usize) -> Option<(usize, usize)> {
    let b = structure.as_bytes();
    let mut depth = 0i32;
    let mut k = pos;
    let open = loop {
        if k == 0 {
            return None;
        }
        k -= 1;
        match b[k] {
            b'}' => depth += 1,
            b'{' => {
                if depth == 0 {
                    break k;
                }
                depth -= 1;
            }
            _ => {}
        }
    };
    let mut d = 0i32;
    for (i, &c) in b.iter().enumerate().skip(open) {
        match c {
            b'{' => d += 1,
            b'}' => {
                d -= 1;
                if d == 0 {
                    return Some((open, i));
                }
            }
            _ => {}
        }
    }
    None
}

/// The value of the `const IDENT: &str = "…"` VISIBLE at `use_at` — ONE lexical
/// rule for the whole file: a declaration is in scope only inside its innermost
/// enclosing block (a fn body, a nested block, a `mod`), whatever its order
/// within that block (a `const` is an item), and a top-level declaration
/// everywhere; the innermost visible one wins. So a nested block's same-named
/// `const` is invisible to a site outside it, and an outer site is never
/// paired with an inner sweep's marker.
fn resolve_const(facts: &FileFacts, ident: &str, use_at: usize) -> Option<String> {
    let decls = facts.consts.get(ident)?;
    // (innermost block open — `None` at top level, which `Option`'s ordering
    // ranks lowest —, declaration offset, value): the innermost visible block
    // wins, and within it the LATEST preceding declaration (a `let` shadowing
    // an earlier `let`).
    let mut best: Option<(Option<usize>, usize, &str)> = None;
    for d in decls {
        if let Some((open, close)) = d.block {
            if !(open < use_at && use_at < close) {
                continue;
            }
        }
        if d.kind.ordered() && d.decl_at >= use_at {
            continue;
        }
        let key = (d.block.map(|(o, _)| o), d.decl_at);
        if best.is_none_or(|(bo, ba, _)| key > (bo, ba)) {
            best = Some((key.0, key.1, d.value.as_str()));
        }
    }
    best.map(|(_, _, v)| v.to_string())
}

/// Start of the statement holding `pos`: back to the previous `;`, `{` or `}`.
fn statement_start(structure: &str, pos: usize, floor: usize) -> usize {
    let b = structure.as_bytes();
    let mut k = pos;
    while k > floor && !matches!(b[k - 1], b';' | b'{' | b'}') {
        k -= 1;
    }
    k
}

/// End of the SWEEP that a `"WARN"` literal at `pos` belongs to: a block
/// (`for level in [...] { … }`) runs to its matching `}`; a single statement
/// (`.filter(|l| (l.contains("WARN") || …) && l.contains(M)).count();`) to its `;`.
fn sweep_end(structure: &str, pos: usize, ceiling: usize) -> usize {
    let b = structure.as_bytes();
    let mut depth = 0i32;
    let mut i = pos;
    while i < ceiling {
        match b[i] {
            b'(' => depth += 1,
            b')' => depth -= 1,
            b';' if depth <= 0 => return i + 1,
            b'}' if depth <= 0 => return i,
            b'{' if depth <= 0 => {
                let mut d = 0i32;
                let mut k = i;
                while k < ceiling {
                    match b[k] {
                        b'{' => d += 1,
                        b'}' => {
                            d -= 1;
                            if d == 0 {
                                return k + 1;
                            }
                        }
                        _ => {}
                    }
                    k += 1;
                }
                return ceiling;
            }
            _ => {}
        }
        i += 1;
    }
    ceiling
}

/// Does the fn `fn_range` hold a loud-level SWEEP that names `marker`?
///
/// Two shapes count, and only when they carry THIS marker's value (a literal,
/// or a `const` resolving to it — the two forms are interchangeable):
/// * a call to a helper whose name ends in `never_loud` whose arguments carry it;
/// * a `"WARN"` literal whose sweep (the block or statement it opens) also
///   carries `"INFO"`, `"ERROR"` and the marker.
///
/// A fn's three loud literals in an unrelated assertion message, or a sweep
/// for a DIFFERENT marker, name nothing for this site: with the gate skipping
/// the site's own assertion in release, only a check on its OWN marker fails a
/// `debug!`→`warn!` promotion of that marker.
fn has_loud_sweep_for(facts: &FileFacts, fn_range: (usize, usize), marker: &str) -> bool {
    let (s, e) = fn_range;
    let structure = &facts.views.structure;
    let body = &structure[s..=e];
    // (a) `*never_loud(` calls — in THIS fn's own body, never one buried in a
    // nested `fn` the enclosing one may not call.
    let mut from = 0usize;
    while let Some(rel) = body[from..].find("never_loud(") {
        let open = s + from + rel + "never_loud(".len() - 1;
        from = from + rel + "never_loud(".len();
        if in_nested_fn(facts, fn_range, open) {
            continue;
        }
        let mut depth = 0i32;
        let mut close = open;
        for (k, &c) in structure.as_bytes().iter().enumerate().skip(open) {
            match c {
                b'(' => depth += 1,
                b')' => {
                    depth -= 1;
                    if depth == 0 {
                        close = k;
                        break;
                    }
                }
                _ => {}
            }
        }
        if values_in(facts, open, close + 1)
            .iter()
            .any(|v| v == marker)
        {
            return true;
        }
    }
    // (b) `"WARN"` sweeps — same rule.
    for &ws in facts
        .warn_spans
        .iter()
        .filter(|ws| **ws >= s && **ws <= e && !in_nested_fn(facts, fn_range, **ws))
    {
        let a = statement_start(structure, ws, s);
        let b = sweep_end(structure, ws, e + 1);
        let vals = values_in(facts, a, b);
        let has = |x: &str| vals.iter().any(|v| v == x);
        if has("INFO") && has("ERROR") && has(marker) {
            return true;
        }
    }
    false
}

/// The fn-level fallback for a site whose marker cannot be resolved (a
/// `level == "DEBUG"` control-flow gate names none): ANY sweep in the fn.
fn has_any_loud_sweep(facts: &FileFacts, fn_range: (usize, usize)) -> bool {
    // Same nested-`fn` rule as [`has_loud_sweep_for`]: a sweep inside a nested
    // helper vouches for nothing in the enclosing fn.
    occurs_outside_nested_fns(facts, fn_range, &facts.views.structure, "never_loud(")
        || ["\"WARN\"", "\"INFO\"", "\"ERROR\""]
            .iter()
            .all(|l| occurs_outside_nested_fns(facts, fn_range, &facts.views.code, l))
}

/// The tracing level macros whose MESSAGE literals the cross-reference collects.
const LEVEL_MACROS: [&str; 5] = ["trace!(", "debug!(", "info!(", "warn!(", "error!("];

/// A marker shorter than this is never cross-referenced. MEASURED on the four
/// trees: every generic phrase that is ALSO a substring of some `debug!`
/// message — `.expect("wake listener")`, a `"max_publishers"` field name,
/// `"transient failure"` as a test input, `"zenoh session"` in an error
/// string — is 17 bytes or fewer, while every real log marker a test matches
/// on is 18 or more. The residual is a presence check keyed on a phrase
/// shorter than this, which the walk cannot tell from those without parsing.
const MIN_MARKER_LEN: usize = 18;

/// Every tracing MESSAGE in the walked trees, split into the QUIET levels
/// (`trace!` and `debug!` — both compiled out by `release_max_level_info`) and
/// the louder ones.
#[derive(Default)]
struct Messages {
    quiet: Vec<String>,
    other: Vec<String>,
}

impl Messages {
    /// QUIET-ONLY: some `trace!`/`debug!` message carries `marker` and no
    /// louder level's does — so the only line it can ever match exists solely
    /// where those levels are compiled in, and asserting its PRESENCE is the
    /// class in a different coat (the shape `rmw_cerulion`'s decode-failure
    /// field-name pin and `rewriter_var_field_assign_test`'s per-skip TRACE
    /// non-event both had).
    fn is_quiet_only(&self, marker: &str) -> bool {
        marker.len() >= MIN_MARKER_LEN
            && self.quiet.iter().any(|m| m.contains(marker))
            && !self.other.iter().any(|m| m.contains(marker))
    }
}

/// Byte spans (quotes included) of every string literal, read off the blanked
/// view, where a literal is exactly `"` + spaces + `"` (raw-string fixings were
/// kept and char-literal bodies blanked, so no stray quote survives).
fn literal_spans(structure: &str) -> Vec<(usize, usize)> {
    let b = structure.as_bytes();
    let mut out = Vec::new();
    let mut i = 0usize;
    while i < b.len() {
        if b[i] == b'"' {
            let mut j = i + 1;
            while j < b.len() && b[j] != b'"' {
                j += 1;
            }
            if j < b.len() {
                out.push((i, j));
            }
            i = j + 1;
        } else {
            i += 1;
        }
    }
    out
}

/// The runtime value of the literal at `span` in the code view: a raw string
/// verbatim; otherwise escapes resolved and `\`-newline continuations joined
/// (with the continuation's leading whitespace dropped, as rustc does).
fn literal_value(code: &str, (s, e): (usize, usize)) -> String {
    let body = &code[s + 1..e];
    if s > 0 && matches!(code.as_bytes()[s - 1], b'#' | b'r') {
        return body.to_string();
    }
    let mut out = String::new();
    let mut chars = body.chars().peekable();
    while let Some(c) = chars.next() {
        if c != '\\' {
            out.push(c);
            continue;
        }
        // EVERY escape rustc accepts in a non-raw string literal (otherwise
        // `\u{42}` decodes to `u`, so `"DE\u{42}UG"` — runtime `DEBUG` — is
        // invisible to the classifier). A malformed escape cannot occur in
        // a file that compiles, so the fallback arms only keep the walk
        // total over text rustc would refuse.
        match chars.next() {
            Some('\n') => {
                while matches!(chars.peek(), Some(' ' | '\t' | '\r' | '\n')) {
                    chars.next();
                }
            }
            Some('n') => out.push('\n'),
            Some('r') => out.push('\r'),
            Some('t') => out.push('\t'),
            Some('0') => out.push('\0'),
            Some('x') => {
                let hex: String = (0..2).filter_map(|_| chars.next()).collect();
                out.push(u8::from_str_radix(&hex, 16).map_or('?', char::from));
            }
            Some('u') => {
                let mut hex = String::new();
                if chars.next_if_eq(&'{').is_some() {
                    while let Some(h) = chars.next_if(|h| *h != '}') {
                        if h != '_' {
                            hex.push(h);
                        }
                    }
                    chars.next();
                }
                out.push(
                    u32::from_str_radix(&hex, 16)
                        .ok()
                        .and_then(char::from_u32)
                        .unwrap_or('?'),
                );
            }
            Some(other) => out.push(other),
            None => {}
        }
    }
    out
}

/// `(open, close)` paren spans of every level-macro invocation — an EMIT, which
/// is never an assertion.
fn emit_spans(structure: &str) -> Vec<(usize, usize)> {
    let b = structure.as_bytes();
    let ident_byte = |c: u8| c.is_ascii_alphanumeric() || c == b'_';
    let mut out = Vec::new();
    for name in LEVEL_MACROS {
        let mut from = 0usize;
        while let Some(rel) = structure[from..].find(name) {
            let at = from + rel;
            from = at + name.len();
            if at > 0 && ident_byte(b[at - 1]) {
                continue;
            }
            let open = at + name.len() - 1;
            let mut depth = 0i32;
            for (k, &c) in b.iter().enumerate().skip(open) {
                match c {
                    b'(' => depth += 1,
                    b')' => {
                        depth -= 1;
                        if depth == 0 {
                            out.push((open, k));
                            break;
                        }
                    }
                    _ => {}
                }
            }
        }
    }
    out.sort_unstable();
    out
}

/// Which declaration keyword bound a `&str`: a `const`/`static` is an item,
/// visible throughout its block; a `let` is visible only after it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DeclKind {
    Const,
    Static,
    Let,
}

impl DeclKind {
    fn ordered(self) -> bool {
        matches!(self, Self::Let)
    }
}

/// One `const IDENT: &str = "…"` declaration: its innermost enclosing block
/// (`None` at file top level) and its value.
struct ConstDecl {
    block: Option<(usize, usize)>,
    value: String,
    /// Where the declaration sits — a `let` is visible only AFTER it.
    decl_at: usize,
    kind: DeclKind,
    /// Where its initializer literal sits: that literal is the DEFINITION of a
    /// value, never a use of it.
    lit_at: usize,
    /// Where the declared name sits — the one occurrence that is not a use.
    name_at: usize,
}

/// What the walk knows about one file before classifying it — indexed ONCE,
/// because the sweep and marker checks below ask these questions per site,
/// per sweep region and per identifier.
struct FileFacts {
    views: Views,
    spans: Vec<(usize, usize)>,
    /// Each span's decoded value, parallel to `spans`.
    span_values: Vec<String>,
    /// Start offsets of every `"WARN"` literal.
    warn_spans: Vec<usize>,
    emits: Vec<(usize, usize)>,
    /// Every `const … : &str` declaration, by name.
    consts: BTreeMap<String, Vec<ConstDecl>>,
    /// Every `fn` body in the file — so the gate and twin scans can tell an
    /// occurrence in the enclosing fn's OWN body from one buried in a nested
    /// `fn` that the enclosing fn may never call.
    fns: Vec<FnRange>,
}

/// Is `pos` inside a `fn` nested STRICTLY within `outer`?
///
/// A gate helper or a loud sweep sitting in a nested `fn` proves nothing about
/// the outer fn: the nested fn may be a helper the outer one never calls, or one
/// it calls on a path the site is not on. Scanning the enclosing fn's whole
/// span — nested bodies included — let such a helper vouch for a DEBUG
/// assertion it never guards.
fn in_nested_fn(facts: &FileFacts, outer: (usize, usize), pos: usize) -> bool {
    facts
        .fns
        .iter()
        .any(|r| r.body != outer && r.body.0 >= outer.0 && r.body.1 <= outer.1 && r.contains(pos))
}

/// Does `needle` occur in `outer`'s OWN body — not only inside a nested `fn`?
///
/// `haystack` is one of the two whole-file views, both offset-identical to the
/// source: the STRUCTURE view (literal bodies blanked) for a call name, so a
/// helper named inside a message or a doc example is not a call; the CODE view
/// for a search whose needle IS a literal (`"WARN"`), which the structure view
/// has blanked away.
fn occurs_outside_nested_fns(
    facts: &FileFacts,
    outer: (usize, usize),
    haystack: &str,
    needle: &str,
) -> bool {
    let body = &haystack[outer.0..=outer.1];
    let mut from = 0usize;
    while let Some(rel) = body[from..].find(needle) {
        let abs = outer.0 + from + rel;
        // An EXACT identifier, not a suffix of one:
        // `fake_debug_lines_expected(5)` contains the helper's name and
        // must not count as the sanctioned gate.
        // Checking the previous CHARACTER with `is_alphanumeric` is not enough,
        // since a combining mark fails it
        // (`a\u{301}debug_lines_expected(`). Rust identifiers continue on
        // XID_Continue, which no std predicate spells; the DELIMITER set is
        // what std can spell exactly — the needle starts an identifier iff
        // it sits at the start of text or right after one of the ASCII
        // characters that can precede an identifier in Rust source.
        // Anything else (letters, digits, `_`, any non-ASCII scalar,
        // marks included) means the needle is INSIDE an identifier.
        let starts_an_identifier = haystack[..abs].chars().next_back().is_none_or(|prev| {
            prev.is_ascii_whitespace()
                || matches!(
                    prev,
                    '(' | ')'
                        | '{'
                        | '}'
                        | '['
                        | ']'
                        | ';'
                        | ','
                        | '='
                        | '.'
                        | '!'
                        | '&'
                        | '|'
                        | '<'
                        | '>'
                        | '+'
                        | '-'
                        | '*'
                        | '/'
                        | ':'
                        | '?'
                        | '#'
                        | '@'
                        | '"'
                        | '\''
                )
        });
        if starts_an_identifier && !in_nested_fn(facts, outer, abs) {
            return true;
        }
        from += rel + needle.len();
    }
    false
}

fn file_facts(src: &str) -> FileFacts {
    let views = strip(src);
    let spans = literal_spans(&views.structure);
    let span_values: Vec<String> = spans
        .iter()
        .map(|&span| literal_value(&views.code, span))
        .collect();
    let warn_spans = spans
        .iter()
        .zip(&span_values)
        .filter(|(_, v)| v.as_str() == "WARN")
        .map(|((s, _), _)| *s)
        .collect();
    let emits = emit_spans(&views.structure);
    let consts = const_decls(&views, &spans, &span_values);
    let fns = fn_ranges(&views.structure);
    FileFacts {
        views,
        spans,
        span_values,
        warn_spans,
        emits,
        consts,
        fns,
    }
}

/// Index every `const IDENT: &str = "…"` (or `&'static str`) in the file.
fn const_decls(
    views: &Views,
    spans: &[(usize, usize)],
    span_values: &[String],
) -> BTreeMap<String, Vec<ConstDecl>> {
    let st = &views.structure;
    let b = st.as_bytes();
    let ident_byte = |c: u8| c.is_ascii_alphanumeric() || c == b'_';
    let mut out: BTreeMap<String, Vec<ConstDecl>> = BTreeMap::new();
    for (keyword, kind) in [
        ("const ", DeclKind::Const),
        ("static ", DeclKind::Static),
        ("let ", DeclKind::Let),
    ] {
        let mut from = 0usize;
        while let Some(rel) = st[from..].find(keyword) {
            let at = from + rel;
            from = at + keyword.len();
            if at > 0 && ident_byte(b[at - 1]) {
                continue;
            }
            let rest = &st[at + keyword.len()..];
            let name_at = at + keyword.len() + if rest.starts_with("mut ") { 4 } else { 0 };
            let rest = rest.strip_prefix("mut ").unwrap_or(rest);
            let name_len = rest.bytes().take_while(|c| ident_byte(*c)).count();
            if name_len == 0 {
                continue;
            }
            let name = &rest[..name_len];
            let after_name = rest[name_len..].trim_start();
            // `: &str` / `: &'static str` (required for const/static, optional for let).
            let after_ty = match after_name.strip_prefix(':') {
                Some(after) => {
                    let ty = after.trim_start();
                    let ty = ty
                        .strip_prefix("&'static")
                        .or_else(|| ty.strip_prefix('&'))
                        .unwrap_or(ty)
                        .trim_start();
                    match ty.strip_prefix("str") {
                        Some(x) => x,
                        None => continue,
                    }
                }
                None if kind.ordered() => after_name,
                None => continue,
            };
            let Some(after_eq) = after_ty.trim_start().strip_prefix('=') else {
                continue;
            };
            let lit_at = st.len() - after_eq.trim_start().len();
            let Some(idx) = spans.iter().position(|(s, _)| *s == lit_at) else {
                continue;
            };
            out.entry(name.to_string()).or_default().push(ConstDecl {
                block: enclosing_block(st, at),
                value: span_values[idx].clone(),
                decl_at: at,
                kind,
                lit_at,
                name_at,
            });
        }
    }
    out
}

/// Collect `facts`'s messages: the LAST literal inside each invocation is
/// tracing's message position (fields come first).
fn collect_messages(facts: &FileFacts, into: &mut Messages) {
    for &(open, close) in &facts.emits {
        let Some(idx) = facts
            .spans
            .iter()
            .rposition(|(s, e)| *s > open && *e < close)
        else {
            continue;
        };
        let msg = facts.span_values[idx].clone();
        let head = &facts.views.structure[..open];
        if head.ends_with("debug!") || head.ends_with("trace!") {
            into.quiet.push(msg);
        } else {
            into.other.push(msg);
        }
    }
}

/// EVERY TRACE/DEBUG-only marker on the line `[line_start, line_start + len)`,
/// each as `(position just past the token, description, value)` — literals on
/// the line, and identifiers naming a `const … : &str` visible at that use;
/// never one that sits inside an emit or in message position.
fn debug_only_markers_on(
    facts: &FileFacts,
    msgs: &Messages,
    line_start: usize,
    len: usize,
) -> Vec<(usize, String, String)> {
    let line_end = line_start + len;
    let inside_emit = |s: usize, e: usize| facts.emits.iter().any(|(o, c)| *o < s && e < *c);
    let mut out = Vec::new();
    for (idx, &(s, e)) in facts
        .spans
        .iter()
        .enumerate()
        // A span is visited on the line it STARTS on, once — a literal whose
        // `\`-newline continuation crosses the boundary is still
        // one selector, not two skipped halves.
        .filter(|(_, (s, _))| *s >= line_start && *s < line_end)
    {
        if inside_emit(s, e) || facts.consts.values().flatten().any(|d| d.lit_at == s) {
            continue;
        }
        let value = facts.span_values[idx].clone();
        if msgs.is_quiet_only(&value) && !in_message_position(&facts.views.structure, e + 1) {
            out.push((
                e + 1 - line_start,
                format!("the TRACE/DEBUG-only marker {value:?}"),
                value,
            ));
        }
    }
    let structure_line = &facts.views.structure[line_start..line_end];
    let b = structure_line.as_bytes();
    let mut pos = 0usize;
    while pos < b.len() {
        if b[pos].is_ascii_alphabetic() || b[pos] == b'_' {
            let start = pos;
            while pos < b.len() && (b[pos].is_ascii_alphanumeric() || b[pos] == b'_') {
                pos += 1;
            }
            let ident = &structure_line[start..pos];
            // The declared NAME of a definition is not a use of it.
            if facts
                .consts
                .values()
                .flatten()
                .any(|d| d.name_at == line_start + start)
            {
                continue;
            }
            // Resolved as visible at the use: the innermost enclosing block's
            // declaration, else a top-level one.
            if let Some(value) = resolve_const(facts, ident, line_start + start) {
                if msgs.is_quiet_only(&value) && !inside_emit(line_start + start, line_start + pos)
                {
                    out.push((
                        pos,
                        format!("the TRACE/DEBUG-only marker {value:?} via `{ident}`"),
                        value,
                    ));
                }
            }
        } else {
            pos += 1;
        }
    }
    out
}

/// Length of the `,`/`|`-separated run of level literals containing the
/// literal that starts at `idx` — the shape of a `matches!` alternation or a
/// level table. Only a run of three or more is a level LIST; three level
/// literals scattered elsewhere on the line exempt nothing. `lit_end` is the
/// offset just past the literal's closing quote ON THIS LINE — the walk to
/// the right starts there, not at a fixed `"DEBUG"` width, because an escaped
/// spelling is longer.
fn level_run_len(line: &str, idx: usize, lit_end: usize) -> usize {
    fn level_ending_at(line: &str, end: usize) -> Option<usize> {
        LEVEL_LITERALS
            .iter()
            .find(|l| line[..end].ends_with(*l))
            .map(|l| l.len())
    }
    fn level_starting_at(line: &str, start: usize) -> Option<usize> {
        LEVEL_LITERALS
            .iter()
            .find(|l| line[start..].starts_with(*l))
            .map(|l| l.len())
    }
    let mut n = 1usize;
    let mut left = idx;
    loop {
        let t = line[..left].trim_end();
        let Some(before_sep) = t.strip_suffix(',').or_else(|| t.strip_suffix('|')) else {
            break;
        };
        let t2 = before_sep.trim_end();
        match level_ending_at(line, t2.len()) {
            Some(len) => {
                n += 1;
                left = t2.len() - len;
            }
            None => break,
        }
    }
    let mut right = lit_end;
    loop {
        let t = line[right..].trim_start();
        let Some(after_sep) = t.strip_prefix(',').or_else(|| t.strip_prefix('|')) else {
            break;
        };
        let start = line.len() - after_sep.trim_start().len();
        match level_starting_at(line, start) {
            Some(len) => {
                n += 1;
                right = start + len;
            }
            None => break,
        }
    }
    n
}

/// Is the level token starting at absolute byte `at` of `structure` passed as
/// an argument or compared? Looks back over whitespace INCLUDING NEWLINES, so
/// a rustfmt-wrapped argument list — the token first on its own line — is
/// still a site (a line-scoped form silently drops exactly that,
/// and the walked trees already wrap sibling level literals that way).
fn is_argument_position(structure: &str, at: usize) -> bool {
    structure[..at]
        .trim_end()
        .chars()
        .next_back()
        .is_some_and(|c| matches!(c, '(' | ',' | '=' | '[' | '|'))
}

/// The start of the callee chain ending at the `(` at `open`: walks left over
/// identifier, `.` and `::` bytes (`out.stderr.contains`), stopping at a macro
/// bang so `count_at!(` yields the position of `count_at`.
fn callee_chain_start(b: &[u8], open: usize) -> usize {
    let mut k = open;
    if k > 0 && b[k - 1] == b'!' {
        k -= 1;
    }
    while k > 0 && (b[k - 1].is_ascii_alphanumeric() || matches!(b[k - 1], b'_' | b'.' | b':')) {
        k -= 1;
    }
    k
}

/// Callees whose string arguments are PROSE — an assertion message, an
/// `expect` reason, a format — never a matcher. A marker in that position is
/// not a site: `assert!(taps.is_empty(), "egress tap detached")` names a
/// `debug!` message by coincidence and asserts nothing about the log.
const MESSAGE_CALLEES: [&str; 10] = [
    "assert!",
    "assert_eq!",
    "assert_ne!",
    "panic!",
    "expect",
    "format!",
    "println!",
    "eprintln!",
    "write!",
    "writeln!",
];

/// Is the token ending at global byte `after` the argument of a MESSAGE callee?
fn in_message_position(structure: &str, after: usize) -> bool {
    let b = structure.as_bytes();
    let mut d = 0i32;
    let mut k = after;
    let open = loop {
        if k == 0 {
            return false;
        }
        k -= 1;
        match b[k] {
            b')' => d += 1,
            b'(' => {
                if d == 0 {
                    break k;
                }
                d -= 1;
            }
            _ => {}
        }
    };
    let start = callee_chain_start(b, open);
    let callee = &structure[start..open];
    MESSAGE_CALLEES
        .iter()
        .any(|c| callee == *c || callee.ends_with(&format!(".{c}")))
}

/// Is the site's statement a `let NAME = …;` whose binding is LATER compared to
/// zero (`NAME == 0`, `NAME != 0`, `NAME.is_empty()`) — by THAT binding, not by
/// any same-named one, and compared to NOTHING ELSE? The search runs forward
/// from the binding's own statement, and a re-binding of the name (`let NAME`,
/// `let mut NAME`, `for NAME in`) ENDS the association, so a shadowed zero check
/// elsewhere in the fn exempts nothing. The exemption is EXCLUSIVE: a binding
/// that is also compared to a nonzero value (`if debugs == 0 { … }` beside
/// `assert_eq!(debugs, 5)`, or `debugs >= 1`) is a count assertion with an
/// early-out, not a silence control — its nonzero leg reads 0 in release, so
/// the zero branch exempts nothing. The multi-line silence control
/// (`let any = lines.iter().filter(|l| l.contains(A) || l.contains(B)).count();
/// if any == 0 { Ok(()) } else { Err(..) }`) asserts NOTHING was logged and is
/// release-safe; its predicate is one statement away from the marker.
fn absence_via_binding(structure: &str, after: usize, body: (usize, usize)) -> bool {
    let b = structure.as_bytes();
    // The statement holding the token: back to the previous `;` (braces are
    // NOT boundaries — the marker usually sits inside a `.filter(|l| { … })`
    // closure body), with the fn body as the floor.
    let mut k = after;
    while k > body.0 + 1 && b[k - 1] != b';' {
        k -= 1;
    }
    // Its first `let` is the statement's own binding (a closure's inner `let`
    // comes later in the text).
    let stmt = &structure[k..after];
    let Some(let_at) = stmt.find("let ").filter(|&p| {
        p == 0 || stmt.as_bytes()[p - 1].is_ascii_whitespace() || stmt.as_bytes()[p - 1] == b'{'
    }) else {
        return false;
    };
    let rest = stmt[let_at + 4..].trim_start();
    let rest = rest.strip_prefix("mut ").unwrap_or(rest).trim_start();
    let name_len = rest
        .bytes()
        .take_while(|c| c.is_ascii_alphanumeric() || *c == b'_')
        .count();
    if name_len == 0 {
        return false;
    }
    let name = &rest[..name_len];
    // Forward from the END of this binding's statement, to the fn's end.
    let stmt_end = structure[after..]
        .find(';')
        .map_or(body.1, |p| after + p + 1);
    let tail = &structure[stmt_end..=body.1];
    let tb = tail.as_bytes();
    let ident_byte = |c: u8| c.is_ascii_alphanumeric() || c == b'_';
    let mut from = 0usize;
    let mut zero_seen = false;
    while let Some(rel) = tail[from..].find(name) {
        let at = from + rel;
        from = at + name.len();
        let end = at + name.len();
        let bounded =
            (at == 0 || !ident_byte(tb[at - 1])) && (end >= tb.len() || !ident_byte(tb[end]));
        if !bounded {
            continue;
        }
        let head = tail[..at].trim_end();
        if head.ends_with("let") || head.ends_with("let mut") || head.ends_with("for") {
            // A re-binding of the name: the association ends here.
            break;
        }
        let negated = head.ends_with('!') && !head.ends_with("!=");
        let after_name = tail[end..].trim_start();
        // What THIS occurrence claims, and where the claim ends.
        let (claim, to) = if let Some(r) = after_name.strip_prefix(".is_empty()") {
            let claim = if negated {
                ZeroClaim::NonZero
            } else {
                ZeroClaim::Zero
            };
            (claim, tail.len() - r.len())
        } else if let Some(r) = after_name.strip_prefix("== 0") {
            (ZeroClaim::Zero, tail.len() - r.len())
        } else if let Some(r) = after_name.strip_prefix("!= 0") {
            (ZeroClaim::NonZero, tail.len() - r.len())
        } else if head.ends_with("assert_eq!(") || head.ends_with("assert_ne!(") {
            // The macro itself asserts: `assert_eq!(n, 0)` is a silence control,
            // `assert_ne!(n, 0)` a nonzero demand, and any other right-hand
            // side (`assert_eq!(n, 5)`) a count assertion.
            let zero = strip_zero_argument(after_name).is_some();
            if zero && head.ends_with("assert_eq!(") {
                zero_seen = true;
                continue;
            }
            return false;
        } else if ["==", "!=", "<", ">"]
            .iter()
            .any(|op| after_name.starts_with(op))
        {
            // The SAME binding compared to anything else — `debugs == 5`,
            // `debugs >= 1` — makes it a count assertion whose zero check is an
            // early-out, not a silence control. The exemption is exclusive,
            // whichever order the two comparisons come in.
            return false;
        } else {
            continue;
        };
        // The claim's polarity in its context decides: `if n != 0 { Err }` and
        // `assert!(n == 0)` are silence controls; `if n == 0 { Err }` and
        // `assert!(n != 0)` demand a nonzero count (0 in release) and exempt
        // nothing — nor does a context the walk does not model.
        let pred_from = if negated { at - 1 } else { at };
        if is_silence_control(claim, predicate_context(tail, pred_from, to)) {
            zero_seen = true;
        } else {
            return false;
        }
    }
    zero_seen
}

/// Is the site whose token ends at global byte `after` an ABSENCE — a genuine
/// silence control?
///
/// Attached means: after the innermost call's closing paren, walking OUTWARD
/// through further closing parens and `.len()` / `.count()` / `.collect…()`
/// hops, the statement continues with `.is_empty()`, `!= 0`, `== 0` or the
/// `, 0` of an `assert_eq!`/`assert_ne!`; or the call chain holding the token
/// is immediately preceded by `!` (`!logs_contain(m)`, `!x.is_empty()`). It is
/// the STATEMENT that is inspected, not the line, so rustfmt wrapping a chain
/// over several lines changes nothing — and an unrelated `== 0` elsewhere in
/// the statement (`count == 5 && other == 0`) is not on that path and lends
/// nothing.
///
/// The token's POLARITY in its context then decides ([`is_silence_control`]):
/// `assert!(count == 0)`, `if count != 0 { Err }`, `assert_eq!(count, 0)` and
/// `!logs_contain(m)` say "nothing was logged"; `assert!(count != 0)`,
/// `if count == 0 { Err }`, `assert!(!x.is_empty())` and `assert_ne!(count, 0)`
/// DEMAND a DEBUG line, read 0 in release, and are exactly the class this walk
/// exists for — they exempt nothing.
///
/// Scanned on the literal-blanked view, so a `)` inside a marker string cannot
/// close the call early.
fn absence_attached(structure: &str, after: usize) -> bool {
    let b = structure.as_bytes();
    // The innermost call's closing paren.
    let mut depth = 1i32;
    let mut i = after;
    while i < b.len() {
        match b[i] {
            b'(' => depth += 1,
            b')' => {
                depth -= 1;
                if depth == 0 {
                    break;
                }
            }
            b';' => return false,
            _ => {}
        }
        i += 1;
    }
    if i >= b.len() {
        return false;
    }
    let open = {
        // The innermost `(` before `after`, for the negation check.
        let mut d = 0i32;
        let mut k = after;
        loop {
            if k == 0 {
                break None;
            }
            k -= 1;
            match b[k] {
                b')' => d += 1,
                b'(' => {
                    if d == 0 {
                        break Some(k);
                    }
                    d -= 1;
                }
                _ => {}
            }
        }
    };
    let Some(open) = open else {
        return false;
    };
    // Walk left over the callee chain (`out.stderr.contains`); a `!` right
    // before that chain is a negation, while a `!` right before the `(`
    // (`count_at!(`) is a macro bang and is not.
    let chain_start = callee_chain_start(b, open);
    let negated = chain_start < open && chain_start > 0 && b[chain_start - 1] == b'!';
    let from = if negated {
        chain_start - 1
    } else {
        chain_start
    };
    let mut rest = &structure[i + 1..];
    loop {
        rest = rest.trim_start();
        if let Some(r) = rest.strip_prefix(')') {
            rest = r;
            continue;
        }
        if let Some(r) = rest
            .strip_prefix(".len()")
            .or_else(|| rest.strip_prefix(".count()"))
        {
            rest = r;
            continue;
        }
        if let Some(r) = rest.strip_prefix(".collect") {
            // `.collect()` or `.collect::<Vec<_>>()`.
            let r = r.trim_start();
            let r = match r.strip_prefix("::<") {
                Some(g) => &g[g.find('>').map_or(0, |p| p + 1)..],
                None => r,
            };
            let r = r.trim_start_matches('>');
            if let Some(r) = r.strip_prefix("()") {
                rest = r;
                continue;
            }
        }
        break;
    }
    // What the statement claims at that point, and where the claim ends.
    let (claim, remaining) = if let Some(r) = rest.strip_prefix(".is_empty()") {
        let claim = if negated {
            ZeroClaim::NonZero
        } else {
            ZeroClaim::Zero
        };
        (claim, r)
    } else if let Some(r) = rest.strip_prefix("== 0") {
        (ZeroClaim::Zero, r)
    } else if let Some(r) = rest.strip_prefix("!= 0") {
        (ZeroClaim::NonZero, r)
    } else if strip_zero_argument(rest).is_some() {
        // `assert_eq!(<count>, 0)` asserts zero; `assert_ne!(<count>, 0)` demands
        // a DEBUG line; any other macro is unmodelled.
        return matches!(enclosing_callee(structure, from), Some(("assert_eq!", _)));
    } else if negated {
        // `!logs_contain(m)`: a negated boolean presence claims zero.
        (ZeroClaim::Zero, rest)
    } else {
        return false;
    };
    let to = structure.len() - remaining.len();
    is_silence_control(claim, predicate_context(structure, from, to))
}

/// The innermost call holding the position `after` (just past a token inside
/// its argument list), as `(open, close)` paren offsets.
fn innermost_call(structure: &str, after: usize) -> Option<(usize, usize)> {
    let b = structure.as_bytes();
    let mut d = 0i32;
    let mut k = after;
    let open = loop {
        if k == 0 {
            return None;
        }
        k -= 1;
        match b[k] {
            b')' => d += 1,
            b'(' => {
                if d == 0 {
                    break k;
                }
                d -= 1;
            }
            _ => {}
        }
    };
    let mut depth = 0i32;
    for (i, &c) in b.iter().enumerate().skip(open) {
        match c {
            b'(' => depth += 1,
            b')' => {
                depth -= 1;
                if depth == 0 {
                    return Some((open, i));
                }
            }
            _ => {}
        }
    }
    None
}

/// A rule-1 site's markers: the other values in its OWN call (so two counts
/// sharing a statement keep their own markers), else its statement's.
fn rule1_markers(facts: &FileFacts, after: usize, call: Option<(usize, usize)>) -> Vec<String> {
    let structure = &facts.views.structure;
    let keep = |v: &String| v.len() >= 3 && !LEVEL_VALUES.contains(&v.as_str());
    if let Some((open, close)) = call {
        let in_call: Vec<String> = values_in(facts, open, close + 1)
            .into_iter()
            .filter(keep)
            .collect();
        if !in_call.is_empty() {
            return in_call;
        }
    }
    let a = statement_start(structure, after, 0);
    let b = structure[after..]
        .find(';')
        .map_or(structure.len(), |p| after + p);
    values_in(facts, a, b).into_iter().filter(keep).collect()
}

/// Classify every site in one file against the trees' message sets — EVERY
/// site on a line, each on its own: two counts on one line, or a count beside
/// a marker, are two sites with their own absence, markers, gate and twin.
fn classify_with(facts: &FileFacts, msgs: &Messages) -> Vec<Site> {
    let views = &facts.views;
    let ranges = fn_ranges(&views.structure);
    let mut sites = Vec::new();
    let mut offset = 0usize;
    for (idx0, line) in views.code.split('\n').enumerate() {
        let line_start = offset;
        offset += line.len() + 1;
        // Same byte offsets in both views, so the blanked line is this slice.
        let structure_line = &views.structure[line_start..line_start + line.len()];
        // Innermost enclosing fn: the containing range with the largest start.
        let enclosing = ranges
            .iter()
            .filter(|r| r.contains(line_start))
            .max_by_key(|r| r.body.0);
        // A fn that does not exist in release cannot be release-unsafe (and
        // pins nothing there): nothing in it is a site.
        if enclosing.is_some_and(|r| r.debug_only) {
            continue;
        }
        // (position just past the token, kind, markers)
        let mut hits: Vec<(usize, String, Vec<String>)> = Vec::new();
        let mut rule1_calls: Vec<(usize, usize)> = Vec::new();
        // Rule 1: EVERY DEBUG level selector on the line used as an argument,
        // comparand or list element — a `"DEBUG"` literal (plain or raw), an
        // identifier whose const/static/let resolves to "DEBUG", or the
        // `Level::DEBUG` path. Each is a site at the offset just past it.
        let mut rule1_ends: Vec<usize> = Vec::new();
        let line_end = line_start + line.len();
        for (idx, &(s, e)) in facts
            .spans
            .iter()
            .enumerate()
            // A span is visited on the line it STARTS on, once — a literal whose
            // `\`-newline continuation crosses the boundary is still
            // one selector, not two skipped halves.
            .filter(|(_, (s, _))| *s >= line_start && *s < line_end)
        {
            if facts.span_values[idx] != "DEBUG" {
                continue;
            }
            // A level LIST is an inline shape; a literal whose continuation
            // crosses the line boundary is never one of its members.
            if e < line_end && level_run_len(line, s - line_start, e + 1 - line_start) >= 3 {
                continue;
            }
            // `const LEVEL: &str = "DEBUG";` DEFINES a selector; the uses are the sites.
            if facts.consts.values().flatten().any(|d| d.lit_at == s) {
                continue;
            }
            // Skip a raw-string prefix (`r"…"`, `br#"…"#`) before the quote.
            let mut p = s;
            while p > line_start && matches!(views.code.as_bytes()[p - 1], b'r' | b'b' | b'#') {
                p -= 1;
            }
            if is_argument_position(&views.structure, p) {
                rule1_ends.push(e + 1);
            }
        }
        let sb = structure_line.as_bytes();
        let mut pos = 0usize;
        while pos < sb.len() {
            if sb[pos].is_ascii_alphabetic() || sb[pos] == b'_' {
                let start = pos;
                while pos < sb.len() && (sb[pos].is_ascii_alphanumeric() || sb[pos] == b'_') {
                    pos += 1;
                }
                let ident = &structure_line[start..pos];
                let is_path = ident == "DEBUG" && structure_line[..start].ends_with("Level::");
                let anchor = if is_path {
                    start - "Level::".len()
                } else {
                    start
                };
                let selects_debug = is_path
                    || resolve_const(facts, ident, line_start + start).as_deref() == Some("DEBUG");
                if selects_debug && is_argument_position(&views.structure, line_start + anchor) {
                    rule1_ends.push(line_start + pos);
                }
            } else {
                pos += 1;
            }
        }
        for after in rule1_ends {
            let call = innermost_call(&views.structure, after);
            if let Some(c) = call {
                rule1_calls.push(c);
            }
            hits.push((
                after - line_start,
                "a DEBUG level selector".to_string(),
                rule1_markers(facts, after, call),
            ));
        }
        // Rule 2: EVERY quiet-only marker on the line — outside a definition, and
        // not the marker OF a rule-1 site's own call (that is the same site).
        let head = structure_line.trim_start();
        let is_definition = ["const ", "pub const ", "pub(crate) const ", "static "]
            .iter()
            .any(|p| head.starts_with(p));
        if !is_definition {
            for (pos, kind, value) in debug_only_markers_on(facts, msgs, line_start, line.len()) {
                let abs = line_start + pos;
                if rule1_calls.iter().any(|(o, c)| *o < abs && abs <= *c) {
                    continue;
                }
                hits.push((pos, kind, vec![value]));
            }
        }
        for (at, kind, markers) in hits {
            let absence = absence_attached(&views.structure, line_start + at)
                || enclosing.is_some_and(|r| {
                    absence_via_binding(&views.structure, line_start + at, r.body)
                });
            let (fn_name, gated, twinned) = match enclosing {
                Some(r) => {
                    // The BLANKED view: a helper name that occurs only inside a
                    // string literal (a message, a doc example) is not a call.
                    // And it must sit in THIS fn's own body: a gate buried in a
                    // nested `fn` the enclosing one may never call vouches for
                    // nothing (same rule for the twin, inside its own helpers).
                    let twinned = if markers.is_empty() {
                        has_any_loud_sweep(facts, r.body)
                    } else {
                        markers.iter().any(|m| has_loud_sweep_for(facts, r.body, m))
                    };
                    (
                        Some(r.name.clone()),
                        GATE_HELPERS
                            .iter()
                            .any(|h| occurs_outside_nested_fns(facts, r.body, &views.structure, h)),
                        twinned,
                    )
                }
                None => (None, false, false),
            };
            sites.push(Site {
                line: idx0 + 1,
                text: line.trim().to_string(),
                kind,
                absence,
                fn_name,
                gated,
                twinned,
                markers,
            });
        }
    }
    sites
}

/// Classify one self-contained source text: its own emits are the message set
/// (the oracle entry point).
fn classify(src: &str) -> Vec<Site> {
    let facts = file_facts(src);
    let mut msgs = Messages::default();
    collect_messages(&facts, &mut msgs);
    classify_with(&facts, &msgs)
}

fn rs_files_under(dir: &Path, out: &mut Vec<PathBuf>) {
    let rd = fs::read_dir(dir).unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display()));
    for entry in rd {
        let path = entry.expect("dir entry").path();
        if path.is_dir() {
            rs_files_under(&path, out);
        } else if path.extension().is_some_and(|x| x == "rs") {
            out.push(path);
        }
    }
    out.sort();
}

struct Walk {
    /// Sites per file, keyed by the workspace-relative path.
    sites: BTreeMap<String, Vec<Site>>,
    /// Files visited per root (the walk must really reach all four).
    files_per_root: BTreeMap<&'static str, usize>,
    /// Whether [`SELF_FILE`] was seen and skipped.
    self_excluded: bool,
    /// The message sets the cross-reference ran against.
    messages: Messages,
}

fn walk() -> Walk {
    let root = workspace_root();
    let mut files_per_root = BTreeMap::new();
    let mut self_excluded = false;
    // Pass 1: read every file and collect the trees' messages.
    let mut facts: Vec<(String, FileFacts)> = Vec::new();
    let mut messages = Messages::default();
    for r in ROOTS {
        let dir = root.join(r);
        assert!(
            dir.is_dir(),
            "walk root {} is missing — a moved crate must not silently leave the gate",
            dir.display()
        );
        let mut files = Vec::new();
        rs_files_under(&dir, &mut files);
        files_per_root.insert(r, files.len());
        for path in files {
            let rel = path
                .strip_prefix(&root)
                .expect("under the workspace root")
                .to_string_lossy()
                .replace('\\', "/");
            if rel == SELF_FILE {
                self_excluded = true;
                continue;
            }
            let src = fs::read_to_string(&path)
                .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
            let f = file_facts(&src);
            collect_messages(&f, &mut messages);
            facts.push((rel, f));
        }
    }
    // Pass 2: classify against the whole trees' messages.
    let mut sites = BTreeMap::new();
    for (rel, f) in &facts {
        let found = classify_with(f, &messages);
        if !found.is_empty() {
            sites.insert(rel.clone(), found);
        }
    }
    Walk {
        sites,
        files_per_root,
        self_excluded,
        messages,
    }
}

const REMEDY: &str = "\
REMEDY: a DEBUG-level line count reads 0 in a release build (`tracing/release_max_level_info` \
compiles `debug!` out), so an unconditional expectation fails deterministically in the ONE job \
that runs `cargo test -p cerulion_core --lib --release` — and only after merge to main. Route \
the expectation through `cerulion_core::testing::debug_lines_expected(n)` (exactly `n` where \
`debug!` is compiled in, `0` where it is not), or gate a presence check on \
`cerulion_core::testing::debug_level_compiled_in()`; then ALSO pin the release contract \
level-free (the suppressed message never at WARN/INFO/ERROR, plus the site's unconditional \
counter). A genuine silence control (`assert!(count == 0)`, `assert_eq!(count, 0)`, \
`if count != 0 { Err }`, or a `let` binding the fn compares ONLY to zero that way) is \
release-safe and needs no gate — but the zero must sit on the count's own STATEMENT or \
binding, and it must actually claim zero: `assert!(count != 0)` / `if count == 0 { Err }` \
demand a DEBUG line and are the class itself. A site can also be a MARKER: a string \
(or a same-file `const`) carried only by a `debug!` message — asserting that line is PRESENT is \
the same class, and the fix is the same gate. EVERY gated site also needs its level-free TWIN in \
the same fn: a `never_loud(lines, MARKER)?`-style helper call, or a WARN/INFO/ERROR sweep \
asserting the marker never appears loudly — that is what fails a `debug!`→`warn!` promotion in \
release, where the gated count reads 0. See the module docs of \
cerulion_core/tests/debug_count_discipline_test.rs.";

/// THE gate: no DEBUG-count assertion in the four trees escapes the static-level helper.
#[test]
fn every_debug_count_assertion_routes_through_the_static_level_gate() {
    let w = walk();
    let mut report = String::new();
    let mut twinless = String::new();
    for (file, sites) in &w.sites {
        for s in sites.iter().filter(|s| s.violates()) {
            let scope = match &s.fn_name {
                Some(name) => format!("in fn `{name}`, whose body names no gate helper"),
                None => "outside any fn body".to_string(),
            };
            report.push_str(&format!(
                "  {file}:{}: {}\n    site: {}; {scope}\n",
                s.line, s.text, s.kind
            ));
        }
        for s in sites.iter().filter(|s| s.missing_twin()) {
            twinless.push_str(&format!(
                "  {file}:{}: {}\n    site: {}; in fn `{}`, gated but with no loud-level sweep naming \
                 its marker {:?}\n",
                s.line,
                s.text,
                s.kind,
                s.fn_name.as_deref().unwrap_or("?"),
                s.markers
            ));
        }
    }
    assert!(
        report.is_empty() && twinless.is_empty(),
        "{} DEBUG-count assertion(s) are not release-safe:\n{report}\n{} gated site(s) have no \
         level-free TWIN (in release the gate skips the only assertion, so a suppressed arm \
         promoted to warn!/info!/error! passes):\n{twinless}\n{REMEDY}",
        report.lines().count() / 2,
        twinless.lines().count() / 2
    );
}

/// ANTI-TAUTOLOGY: the walk reaches every root, sees real sites in each, finds
/// real GATED fns, and really skipped its own file. Without this, a wrong root
/// path or a broken stripper would make the gate above pass over nothing.
#[test]
fn the_walk_reaches_all_four_roots_and_finds_real_gated_sites() {
    let w = walk();
    for r in ROOTS {
        assert!(
            w.files_per_root.get(r).copied().unwrap_or(0) > 0,
            "root {r} yielded no .rs files"
        );
        let sites_in_root: usize = w
            .sites
            .iter()
            .filter(|(f, _)| f.starts_with(&format!("{r}/")))
            .map(|(_, s)| s.len())
            .sum();
        assert!(
            sites_in_root >= 1,
            "root {r} yielded no DEBUG site at all — every root is known to carry at least one; \
             either the tests moved or the classifier went blind"
        );
    }
    let gated_fns: usize = w
        .sites
        .values()
        .map(|sites| {
            let mut names: Vec<&str> = sites
                .iter()
                .filter(|s| s.gated && !s.absence)
                .filter_map(|s| s.fn_name.as_deref())
                .collect();
            names.sort_unstable();
            names.dedup();
            names.len()
        })
        .sum();
    assert!(
        gated_fns >= 3,
        "expected at least 3 distinct GATED fns across the trees, found {gated_fns} — the \
         detector cannot see the real sites"
    );
    assert!(
        w.self_excluded,
        "the walk never saw {SELF_FILE}: the exclusion is not reaching the file it is for"
    );
    // The cross-reference ran against REAL message sets — a broken extractor
    // would make every marker assertion vacuous.
    assert!(
        w.messages.quiet.len() >= 50 && w.messages.other.len() >= 100,
        "message extraction went blind: {} trace!/debug!, {} other",
        w.messages.quiet.len(),
        w.messages.other.len()
    );
    let marker_only_gated = w
        .sites
        .values()
        .flatten()
        .filter(|s| s.kind.contains("-only marker") && s.gated && !s.text.contains("\"DEBUG\""))
        .count();
    assert!(
        marker_only_gated >= 1,
        "no gated site is a pure DEBUG-only-marker site — the second rule sees nothing real"
    );
}

// ---------------------------------------------------------------------------
// Classifier oracles over SYNTHETIC snippets (this file is excluded from the
// walk precisely so these can spell the token out).
// ---------------------------------------------------------------------------

/// Build a snippet: a fn body holding `stmt`.
fn in_fn(name: &str, stmt: &str) -> String {
    format!("#[test]\nfn {name}() {{\n    let lines: Vec<&str> = vec![];\n    {stmt}\n}}\n")
}

#[test]
fn an_unconditional_debug_count_is_a_violation_named_by_fn() {
    let src = in_fn(
        "planted",
        "let n = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    assert_eq!(n, 5);",
    );
    let sites = classify(&src);
    assert_eq!(sites.len(), 1, "{sites:?}");
    assert!(sites[0].violates());
    assert_eq!(sites[0].fn_name.as_deref(), Some("planted"));
    assert_eq!(sites[0].line, 4);
}

#[test]
fn every_argument_shape_is_a_site_and_every_gate_helper_clears_it() {
    for stmt in [
        "let n = count_at(&lines, \"DEBUG\", \"m\");",
        "let n = lines_at(&lines, \"DEBUG\", \"m\");",
        "let n = count_at!(\"DEBUG\", \"m\");",
        "for (l, m) in [(\"DEBUG\", \"m\")] { let _ = (l, m); }",
        "let n = lines.iter().filter(|l| level_of(l) == Some(\"DEBUG\")).count();",
        "let ok = level == \"DEBUG\";",
    ] {
        let ungated = classify(&in_fn("t", stmt));
        assert_eq!(ungated.len(), 1, "not detected as a site: {stmt}");
        assert!(ungated[0].violates(), "{stmt}");
        for gate in [
            "let w = debug_lines_expected(3);",
            "if debug_level_compiled_in() {}",
            "if trace_level_compiled_in() {}",
        ] {
            let gated = classify(&in_fn("t", &format!("{gate}\n    {stmt}")));
            assert_eq!(gated.len(), 1);
            assert!(!gated[0].violates(), "gate `{gate}` did not clear: {stmt}");
        }
    }
}

#[test]
fn an_expected_zero_on_the_sites_own_line_is_exempt_but_on_the_next_line_is_not() {
    for stmt in [
        "assert!(lines_at(&lines, \"DEBUG\", \"m\").is_empty());",
        "if count_at(&lines, \"DEBUG\", \"m\") != 0 { panic!() }",
        "assert!(count_at(&lines, \"DEBUG\", \"m\") == 0);",
    ] {
        let sites = classify(&in_fn("t", stmt));
        assert_eq!(sites.len(), 1);
        assert!(sites[0].absence && !sites[0].violates(), "{stmt}");
    }
    // `assert_eq!(<count>, 0)` is a silence control — the macro name
    // decides the claim, and rustfmt putting the `0` on its own line is not a
    // guess (it is an argument of the SAME call) — while `assert_ne!` demands.
    let split = classify(&in_fn(
        "t",
        "assert_eq!(\n        count_at(&lines, \"DEBUG\", \"m\"),\n        0,\n    );",
    ));
    assert_eq!(split.len(), 1);
    assert!(split[0].absence && !split[0].violates(), "{split:?}");
    let split_ne = classify(&in_fn(
        "t",
        "assert_ne!(\n        count_at(&lines, \"DEBUG\", \"m\"),\n        0,\n    );",
    ));
    assert_eq!(split_ne.len(), 1);
    assert!(split_ne[0].violates(), "{split_ne:?}");
    // A zero that is NOT attached to the count — on another expression, on the
    // next line — must NOT be recognised: the walk never guesses.
    let elsewhere = classify(&in_fn(
        "t",
        "let n = count_at(&lines, \"DEBUG\", \"m\");\n    assert_eq!(other, 0);",
    ));
    assert_eq!(elsewhere.len(), 1);
    assert!(
        elsewhere[0].violates(),
        "a zero on another expression lends nothing: {elsewhere:?}"
    );
    // A wrapped chain is inspected as a STATEMENT: the zero is still attached.
    let wrapped = classify(&in_fn(
        "t",
        "if lines\n        .iter()\n        .filter(|l| l.contains(\"DEBUG\"))\n        .count()\n        != 0\n    {\n        panic!()\n    }",
    ));
    assert_eq!(wrapped.len(), 1, "{wrapped:?}");
    assert!(wrapped[0].absence, "{wrapped:?}");
    // A leading negation is an absence, a macro bang is not.
    for (stmt, absence) in [
        (
            "assert!(!lines.iter().any(|l| l.contains(\"DEBUG\") && l.len() > 5).clone());",
            false,
        ),
        (
            "assert!(\n        !count_at_debug(&lines, \"DEBUG\")\n    );",
            true,
        ),
        (
            "if !lines_at(&lines, \"DEBUG\", \"m\").is_empty() { panic!() }",
            true,
        ),
        ("assert_eq!(count_at(&lines, \"DEBUG\", \"m\"), 5);", false),
    ] {
        let sites = classify(&in_fn("t", stmt));
        assert_eq!(sites.len(), 1, "{stmt}");
        assert_eq!(sites[0].absence, absence, "{stmt}");
    }
    // A zero check elsewhere on the SAME line lends nothing to a nonzero count.
    for stmt in [
        "assert!(count_at(&lines, \"DEBUG\", \"m\") == 5 && other.is_empty());",
        "assert!(other == 0 && count_at(&lines, \"DEBUG\", \"m\") == 5);",
        "assert!(lines_at(&lines, \"DEBUG\", \"m)\").len() == 5 && other != 0);",
    ] {
        let sites = classify(&in_fn("t", stmt));
        assert_eq!(sites.len(), 1, "{stmt}");
        assert!(
            !sites[0].absence && sites[0].violates(),
            "an unattached zero must not exempt: {stmt}"
        );
    }
}

#[test]
fn three_unrelated_level_literals_on_the_line_do_not_shield_a_count() {
    for stmt in [
        "assert!(count_at(&lines, \"DEBUG\", \"m\") == 5 || [\"WARN\", \"INFO\", \"ERROR\"].is_empty());",
        "let (n, _) = (count_at(&lines, \"DEBUG\", \"m\"), (\"WARN\", \"INFO\", \"ERROR\"));",
    ] {
        let sites = classify(&in_fn("t", stmt));
        assert_eq!(sites.len(), 1, "not detected: {stmt}");
        assert!(sites[0].violates(), "{stmt}");
    }
    // …while a genuine run still exempts, whatever else shares the line.
    let run = classify(&in_fn(
        "t",
        "let ok = matches!(t, \"TRACE\" | \"DEBUG\" | \"INFO\" | \"WARN\" | \"ERROR\"); let n = 5;",
    ));
    assert!(run.is_empty(), "{run:?}");
}

#[test]
fn a_gate_helper_named_only_inside_a_string_literal_does_not_gate() {
    for stmt in [
        "let msg = \"route it through debug_lines_expected(n)\";\n    \
         let n = count_at(&lines, \"DEBUG\", \"m\");\n    assert_eq!(n, 5);",
        "let n = count_at(&lines, \"DEBUG\", r#\"debug_level_compiled_in() says\"#);\n    \
         assert_eq!(n, 5);",
    ] {
        let sites = classify(&in_fn("t", stmt));
        assert_eq!(sites.len(), 1, "{stmt}");
        assert!(
            sites[0].violates(),
            "a quoted helper name must not gate: {stmt}"
        );
    }
}

#[test]
fn an_escaped_or_line_spanning_debug_selector_is_still_a_selector() {
    // Two hazards for the walk: a `literal_value` that decodes only
    // `\n`, `\t` and continuations leaves a `\u{..}` / `\x..` spelling of
    // DEBUG invisible; and a literal whose continuation crosses a line
    // boundary can be filtered out of BOTH lines' span sets. Each spelling is
    // ONE site, and it violates when ungated.
    for stmt in [
        "let n = count_at(&lines, \"DE\\u{42}UG\", \"m\");\n    assert!(n >= 1);",
        "let n = count_at(&lines, \"DE\\x42UG\", \"m\");\n    assert!(n >= 1);",
        "let n = count_at(&lines, \"\\u{44}\\u{45}\\u{42}\\u{55}\\u{47}\", \"m\");\n    assert!(n >= 1);",
        // The continuation: `"DE\` newline `    BUG"` — a span across two lines.
        "let n = count_at(&lines, \"DE\\\n        BUG\", \"m\");\n    assert!(n >= 1);",
    ] {
        let sites = classify(&in_fn("t", stmt));
        assert_eq!(sites.len(), 1, "exactly one site (visited once): {stmt}");
        assert!(
            sites[0].violates(),
            "an ungated count keyed on an escaped/line-spanning DEBUG selector must \
             violate: {stmt}"
        );
    }
    // A RAW string keeps its backslashes: `r"DE\u{42}UG"` is not DEBUG at
    // runtime, so it is not a selector — the decoder tells the two apart.
    let stmt = "let n = count_at(&lines, r\"DE\\u{42}UG\", \"m\");\n    assert!(n >= 1);";
    let sites = classify(&in_fn("t", stmt));
    assert!(
        sites.is_empty(),
        "a raw literal is verbatim, never decoded: {sites:?}"
    );
}

#[test]
fn a_helper_name_that_is_the_tail_of_another_identifier_does_not_gate() {
    // Detecting the gate by substring would let
    // `fake_debug_lines_expected(5)` gate an unconditional DEBUG count.
    for stmt in [
        "let want = fake_debug_lines_expected(5);\n    \
         let n = count_at(&lines, \"DEBUG\", \"m\");\n    assert_eq!(n, want);",
        "let want = my_debug_level_compiled_in(true);\n    \
         let n = count_at(&lines, \"DEBUG\", \"m\");\n    assert_eq!(n, want);",
        // A NON-ASCII letter is an identifier character too.
        "let want = édebug_lines_expected(5);\n    \
         let n = count_at(&lines, \"DEBUG\", \"m\");\n    assert_eq!(n, want);",
        // ...and so is a COMBINING MARK, which `is_alphanumeric` does not
        // call a letter.
        "let want = a\u{301}debug_lines_expected(5);\n    \
         let n = count_at(&lines, \"DEBUG\", \"m\");\n    assert_eq!(n, want);",
    ] {
        let sites = classify(&in_fn("t", stmt));
        assert_eq!(sites.len(), 1, "{stmt}");
        assert!(
            sites[0].violates(),
            "a prefixed helper name must not gate: {stmt}"
        );
    }
    // The exact name still gates — the boundary refuses a prefix, not the helper.
    let stmt = "let want = debug_lines_expected(5);\n    \
         let n = count_at(&lines, \"DEBUG\", \"m\");\n    assert_eq!(n, want);";
    let sites = classify(&in_fn("t", stmt));
    assert_eq!(sites.len(), 1);
    assert!(
        !sites[0].violates(),
        "the exact helper name must still gate: {stmt}"
    );
}

#[test]
fn level_lists_prose_and_comments_are_not_sites() {
    let src = "\
fn level_of(line: &str) -> Option<&str> {
    line.split_whitespace()
        .find(|t| matches!(*t, \"TRACE\" | \"DEBUG\" | \"INFO\" | \"WARN\" | \"ERROR\"))
}
const LEVELS: [&str; 5] = [\"ERROR\", \"WARN\", \"INFO\", \"DEBUG\", \"TRACE\"];
fn name(l: Level) -> &'static str { match l { Level::Debug => \"DEBUG\", _ => \"OTHER\" } }
fn prose() -> &'static str { \"expected 8 DEBUG repeats, got {n}\" }
fn commented() {
    // let n = count_at(&lines, \"DEBUG\", \"m\");
    /* let n = count_at(&lines, \"DEBUG\", \"m\"); /* nested */ */
}
";
    assert!(classify(src).is_empty(), "{:?}", classify(src));
}

#[test]
fn a_site_inside_a_closure_belongs_to_the_enclosing_fn_and_braces_in_strings_do_not_confuse_it() {
    let src = "\
fn outer() {
    let want = debug_lines_expected(2);
    logs_assert(|lines: &[&str]| {
        let msg = \"unbalanced { brace in a string\";
        let n = count_at(lines, \"DEBUG\", msg);
        if n != want { return Err(format!(\"{want} vs {n}\")); }
        Ok(())
    });
}
fn later() {
    let n = count_at(&[], \"DEBUG\", \"m\");
    assert_eq!(n, 1);
}
";
    let sites = classify(src);
    assert_eq!(sites.len(), 2, "{sites:?}");
    assert_eq!(sites[0].fn_name.as_deref(), Some("outer"));
    assert!(!sites[0].violates());
    assert_eq!(sites[1].fn_name.as_deref(), Some("later"));
    assert!(sites[1].violates());
}

#[test]
fn a_presence_check_on_a_debug_only_marker_is_a_site_and_a_gate_clears_it() {
    let emit = "fn emit() {\n    tracing::debug!(n = 1, \"widget backlog suppressed (regime \\\n        open)\");\n    tracing::warn!(\"widget backlog opened loudly\");\n}\n";
    // A literal marker in a presence check.
    let src = format!(
        "{emit}{}",
        in_fn("t", "assert!(lines.iter().any(|l| l.contains(\"widget backlog suppressed (regime open)\")));")
    );
    let sites = classify(&src);
    assert_eq!(sites.len(), 1, "{sites:?}");
    assert!(
        sites[0].violates() && sites[0].kind.contains("-only marker"),
        "{sites:?}"
    );
    assert_eq!(sites[0].fn_name.as_deref(), Some("t"));
    // The same marker through a same-file const.
    let src = format!(
        "const M: &str = \"widget backlog suppressed\";\n{emit}{}",
        in_fn("t", "assert!(logs_contain(M));")
    );
    let sites = classify(&src);
    assert_eq!(sites.len(), 1, "{sites:?}");
    assert!(
        sites[0].violates() && sites[0].kind.contains("via `M`"),
        "{sites:?}"
    );
    // A gate clears it.
    let src = format!(
        "{emit}{}",
        in_fn("t", "if debug_level_compiled_in() { assert!(logs_contain(\"widget backlog suppressed\")); }")
    );
    let sites = classify(&src);
    assert_eq!(sites.len(), 1);
    assert!(!sites[0].violates());
    // A marker a LOUDER message also carries is not debug-only.
    let src = format!(
        "{emit}{}",
        in_fn("t", "assert!(logs_contain(\"widget backlog\"));")
    );
    assert!(classify(&src).is_empty(), "{:?}", classify(&src));
    // The emit itself and the const definition are never sites.
    let src = format!("const M: &str = \"widget backlog suppressed\";\n{emit}");
    assert!(classify(&src).is_empty(), "{:?}", classify(&src));
    // A `trace!`-only marker is the same class one level lower.
    let src = format!(
        "fn e() {{ tracing::trace!(\"widget skipped this tick (no loan)\"); }}\n{}",
        in_fn(
            "t",
            "assert!(logs_contain(\"widget skipped this tick (no loan)\"));"
        )
    );
    let sites = classify(&src);
    assert_eq!(sites.len(), 1, "{sites:?}");
    assert!(sites[0].violates(), "{sites:?}");
    let src = format!(
        "fn e() {{ tracing::trace!(\"widget skipped this tick (no loan)\"); }}\n{}",
        in_fn("t", "if trace_level_compiled_in() { assert!(logs_contain(\"widget skipped this tick (no loan)\")); }")
    );
    assert!(!classify(&src)[0].violates());
    // A marker in MESSAGE position is prose, not a matcher.
    for stmt in [
        "assert!(lines.is_empty(), \"widget backlog suppressed (regime open)\");",
        "let _ = lines.first().expect(\"widget backlog suppressed (regime open)\");",
    ] {
        let src = format!("{emit}{}", in_fn("t", stmt));
        assert!(classify(&src).is_empty(), "{stmt}: {:?}", classify(&src));
    }
    // A binding later compared to zero is a silence control; compared to N it is not.
    for (tail, absence) in [
        ("if any == 0 { Ok(()) } else { Err(any) }.unwrap();", true),
        ("assert!(any.is_empty());", true),
        ("assert_eq!(any, 5);", false),
    ] {
        // A BRACED closure body, as the silence controls in the trees write it.
        let stmt = format!(
            "let any = lines\n        .iter()\n        .filter(|l| {{\n            l.contains(\"other line\")\n                || l.contains(\"widget backlog suppressed\")\n        }})\n        .count();\n    {tail}"
        );
        let src = format!("{emit}{}", in_fn("t", &stmt));
        let sites = classify(&src);
        assert_eq!(sites.len(), 1, "{tail}: {sites:?}");
        assert_eq!(sites[0].absence, absence, "{tail}: {sites:?}");
    }
    // An attached zero exempts a marker site too.
    let src = format!(
        "{emit}{}",
        in_fn(
            "t",
            "assert!(count_at(&lines, \"WARN\", \"widget backlog suppressed\") == 0);"
        )
    );
    let sites = classify(&src);
    assert_eq!(sites.len(), 1);
    assert!(sites[0].absence && !sites[0].violates());
}

/// A gate or a twin that lives in a NESTED `fn` vouches for nothing.
///
/// `fn_ranges` returns every `fn` in the file and a site's enclosing fn is the
/// INNERMOST one containing it — so the gate and twin scans must not search that
/// fn's whole SPAN, nested bodies included. Otherwise an unrelated helper defined
/// inside a test — one the test may never call, or call on a path the site is
/// not on — could carry `debug_lines_expected(...)` and `never_loud(...)` and
/// silently make an UNCONDITIONAL DEBUG assertion beside it look release-safe.
/// That is this walk's own class, one level in.
///
/// The two halves are pinned SEPARATELY, because a `Site`'s verdict is a
/// ladder: an ungated site is `Ungated` whatever its twin, so a nested-gate
/// probe can say nothing about twinning. Arm (a) hides only the GATE; arm (b)
/// gates in the outer body and hides only the TWIN; arm (c) is the
/// anti-tautology — the SAME helpers in the fn's own body still clear it, so
/// the rule is about WHERE they sit, not about the presence of a nested `fn`.
#[test]
fn a_gate_or_twin_inside_a_nested_fn_does_not_clear_the_outer_site() {
    const SITE: &str = "let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");";
    let nested = |body: &str| {
        format!("fn unused_helper(lines: &[&str]) -> Result<(), String> {{\n        let _ = lines;\n        {body}\n        Ok(())\n    }}")
    };

    // (a) the GATE is nested; the twin is in the outer body.
    let gate_hidden = classify(&in_fn(
        "t",
        &format!(
            "{}\n    {SITE}\n    assert_eq!(n, 5);\n    never_loud(&lines, \"widget suppressed\")?;",
            nested("let _ = debug_lines_expected(5);")
        ),
    ));
    assert_eq!(gate_hidden.len(), 1, "{gate_hidden:?}");
    assert!(
        gate_hidden[0].violates(),
        "a gate buried in a nested fn must not make an unconditional DEBUG count \
         release-safe: {gate_hidden:?}"
    );

    // (b) the site IS gated in the outer body; only the TWIN is nested.
    let twin_hidden = classify(&in_fn(
        "t",
        &format!(
            "{}\n    let want = debug_lines_expected(5);\n    {SITE}\n    assert_eq!(n, want);",
            nested("never_loud(lines, \"widget suppressed\")?;")
        ),
    ));
    assert_eq!(twin_hidden.len(), 1, "{twin_hidden:?}");
    assert!(
        twin_hidden[0].missing_twin(),
        "a never_loud buried in a nested fn must not twin the outer site: {twin_hidden:?}"
    );

    // (c) ANTI-TAUTOLOGY: both helpers in the fn's OWN body, with the nested fn
    // still present, and the site is clean.
    let outer = classify(&in_fn(
        "t",
        &format!(
            "{}\n    let want = debug_lines_expected(5);\n    {SITE}\n    \
             assert_eq!(n, want);\n    never_loud(&lines, \"widget suppressed\")?;",
            nested("let _ = 1;")
        ),
    ));
    assert_eq!(outer.len(), 1, "{outer:?}");
    assert!(
        !outer[0].violates() && !outer[0].missing_twin(),
        "the same helpers in the fn's OWN body still clear the site: {outer:?}"
    );
}

#[test]
fn a_gated_site_without_a_loud_level_sweep_is_reported_as_twinless() {
    let bare = classify(&in_fn(
        "t",
        "let want = debug_lines_expected(5);\n    let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert_eq!(n, want);",
    ));
    assert_eq!(bare.len(), 1);
    assert!(!bare[0].violates() && bare[0].missing_twin(), "{bare:?}");
    // Either twin shape clears it.
    for twin in [
        "never_loud(&lines, \"widget suppressed\")?;",
        "suppressed_never_loud(&lines, \"widget suppressed\")?;",
        "for level in [\"WARN\", \"INFO\", \"ERROR\"] { assert_eq!(count_at(&lines, level, \"widget suppressed\"), 0); }",
        "let loud = lines.iter().filter(|l| (l.contains(\"WARN\") || l.contains(\"INFO\") || l.contains(\"ERROR\")) && l.contains(\"widget suppressed\")).count();",
    ] {
        let sites = classify(&in_fn(
            "t",
            &format!("let want = debug_lines_expected(5);\n    let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert_eq!(n, want);\n    {twin}"),
        ));
        assert!(!sites[0].missing_twin(), "twin not recognised: {twin}");
    }
    // (i) The three loud literals in an UNRELATED message name nothing.
    let unrelated = classify(&in_fn(
        "t",
        "let want = debug_lines_expected(5);\n    let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert_eq!(n, want, \"{}\", [\"WARN\", \"INFO\", \"ERROR\"].concat());",
    ));
    assert!(unrelated[0].missing_twin(), "{unrelated:?}");
    // (ii) A sweep for a DIFFERENT marker names nothing for this site.
    let other = classify(&in_fn(
        "t",
        "let want = debug_lines_expected(5);\n    let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert_eq!(n, want);\n    for level in [\"WARN\", \"INFO\", \"ERROR\"] { assert_eq!(count_at(&lines, level, \"other marker\"), 0); }",
    ));
    assert!(other[0].missing_twin(), "{other:?}");
    let other_helper = classify(&in_fn(
        "t",
        "let want = debug_lines_expected(5);\n    let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert_eq!(n, want);\n    never_loud(&lines, \"other marker\")?;",
    ));
    assert!(other_helper[0].missing_twin(), "{other_helper:?}");
    // (iii) The same marker through a `const`, in either direction, is a twin.
    for (site, twin) in [
        ("count_at(&lines, \"DEBUG\", M)", "never_loud(&lines, M)?;"),
        ("count_at(&lines, \"DEBUG\", \"widget suppressed\")", "never_loud(&lines, M)?;"),
        ("count_at(&lines, \"DEBUG\", M)", "for level in [\"WARN\", \"INFO\", \"ERROR\"] { assert_eq!(count_at(&lines, level, \"widget suppressed\"), 0); }"),
    ] {
        let src = format!(
            "const M: &str = \"widget suppressed\";\n{}",
            in_fn("t", &format!("let want = debug_lines_expected(5);\n    let n = {site};\n    assert_eq!(n, want);\n    {twin}"))
        );
        let sites = classify(&src);
        assert_eq!(sites.len(), 1, "{sites:?}");
        assert!(!sites[0].missing_twin(), "{site} / {twin}: {sites:?}");
    }
    // A `const` resolves to the declaration VISIBLE at the use: an inner block's
    // shadowing `const` is invisible to the outer site, so a sweep naming only
    // the inner marker is no twin — while one naming the outer marker is.
    let outer_site = "let want = debug_lines_expected(5);\n    let n = count_at(&lines, \"DEBUG\", M);\n    assert_eq!(n, want);";
    let shadowed = classify(&format!(
        "const M: &str = \"outer suppressed\";\n{}",
        in_fn("t", &format!("{outer_site}\n    {{\n        const M: &str = \"inner suppressed\";\n        never_loud(&lines, M)?;\n    }}"))
    ));
    assert_eq!(shadowed.len(), 1, "{shadowed:?}");
    assert_eq!(
        shadowed[0].markers,
        vec!["outer suppressed".to_string()],
        "{shadowed:?}"
    );
    assert!(
        shadowed[0].missing_twin(),
        "an inner shadowing const must not twin the outer site: {shadowed:?}"
    );
    let control = classify(&format!(
        "const M: &str = \"outer suppressed\";\n{}",
        in_fn("t", &format!("{outer_site}\n    {{\n        const M: &str = \"inner suppressed\";\n        never_loud(&lines, \"outer suppressed\")?;\n    }}"))
    ));
    assert!(!control[0].missing_twin(), "{control:?}");
    // …and a fn-level const stays visible to every site in the fn, whatever the order.
    let fn_level = classify(&in_fn(
        "t",
        &format!(
            "{outer_site}\n    never_loud(&lines, M)?;\n    const M: &str = \"late but visible\";"
        ),
    ));
    assert_eq!(
        fn_level[0].markers,
        vec!["late but visible".to_string()],
        "{fn_level:?}"
    );
    assert!(!fn_level[0].missing_twin(), "{fn_level:?}");
    // A rule-2 site is twinned by a sweep naming ITS marker, not another.
    let emit = "fn emit() { tracing::debug!(\"widget backlog suppressed (regime open)\"); tracing::debug!(\"gadget backlog suppressed (regime open)\"); }\n";
    let same = classify(&format!("{emit}{}", in_fn("t", "if debug_level_compiled_in() { assert!(logs_contain(\"widget backlog suppressed\")); }\n    for level in [\"WARN\", \"INFO\", \"ERROR\"] { assert!(!lines.iter().any(|l| l.contains(level) && l.contains(\"widget backlog suppressed\"))); }")));
    assert!(!same[0].missing_twin(), "{same:?}");
    let different = classify(&format!("{emit}{}", in_fn("t", "if debug_level_compiled_in() { assert!(logs_contain(\"widget backlog suppressed\")); }\n    for level in [\"WARN\", \"INFO\", \"ERROR\"] { assert!(!lines.iter().any(|l| l.contains(level) && l.contains(\"gadget backlog suppressed\"))); }")));
    assert!(different[0].missing_twin(), "{different:?}");
    // The helper name quoted in a message is not a sweep.
    let quoted = classify(&in_fn(
        "t",
        "let want = debug_lines_expected(5);\n    let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert_eq!(n, want, \"add never_loud(lines, widget suppressed)\");",
    ));
    assert!(quoted[0].missing_twin(), "{quoted:?}");
    // An absence site needs no twin.
    let absent = classify(&in_fn(
        "t",
        "assert!(count_at(&lines, \"DEBUG\", \"widget suppressed\") == 0);",
    ));
    assert!(!absent[0].missing_twin());
}

#[test]
fn a_zero_check_on_a_shadowing_or_earlier_binding_exempts_nothing() {
    // Shadowed AFTER: the count's own binding is compared to 5; the zero belongs
    // to a re-`let` of the same name.
    let shadowed = classify(&in_fn(
        "t",
        "let any = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    assert_eq!(any, 5);\n    let any = lines.len();\n    assert!(any == 0);",
    ));
    assert_eq!(shadowed.len(), 1, "{shadowed:?}");
    assert!(
        !shadowed[0].absence && shadowed[0].violates(),
        "{shadowed:?}"
    );
    // A zero check BEFORE the binding belongs to an earlier binding.
    let earlier = classify(&in_fn(
        "t",
        "let any = lines.len();\n    assert!(any == 0);\n    let any = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    assert_eq!(any, 5);",
    ));
    assert_eq!(earlier.len(), 1, "{earlier:?}");
    assert!(!earlier[0].absence && earlier[0].violates(), "{earlier:?}");
    // …while the genuine silence control, closure body and all, stays exempt.
    let genuine = classify(&in_fn(
        "t",
        "let any = lines\n        .iter()\n        .filter(|l| {\n            l.contains(\"other\") || l.contains(\"DEBUG\")\n        })\n        .count();\n    if any == 0 { Ok(()) } else { Err(any) }.unwrap();",
    ));
    assert_eq!(genuine.len(), 1);
    assert!(genuine[0].absence, "{genuine:?}");
}

#[test]
fn a_binding_also_compared_to_a_nonzero_value_is_not_an_absence() {
    // `assert!(debugs == 0)` beside `assert_eq!(debugs, 5)`: the zero check is
    // an early-out, the nonzero comparison is the count assertion — it reads 0
    // in release and must not be exempted by its sibling zero check, in EITHER
    // order, for each comparison shape the walk knows: `assert_eq!(n, N)`,
    // `assert_ne!(n, N)`, infix `==`/`!=`/`<`/`>`(`=`) against a nonzero.
    for stmt in [
        "let debugs = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    assert!(debugs == 0);\n    assert_eq!(debugs, 5);",
        "let debugs = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    assert_eq!(debugs, 5);\n    assert!(debugs == 0);",
        "let debugs = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    assert_ne!(debugs, 5);\n    assert!(debugs == 0);",
        "let debugs = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    assert!(debugs >= 1);\n    assert!(debugs == 0);",
        "let debugs = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    assert!(debugs <= 9);\n    assert!(debugs == 0);",
        "let debugs = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    if debugs == 5 { return Err(\"five\".into()); }\n    assert!(debugs == 0);",
        "let debugs = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    if debugs != 0 { return Err(\"leak\".into()); }\n    if debugs != 4 { return Err(format!(\"{debugs}\")); }",
    ] {
        let sites = classify(&in_fn("t", stmt));
        assert_eq!(sites.len(), 1, "{stmt}\n{sites:?}");
        assert!(
            !sites[0].absence && sites[0].violates(),
            "a binding compared to a nonzero value is not a silence control:\n{stmt}\n{sites:?}"
        );
    }
    // …while a binding whose ONLY comparison is the zero check stays exempt,
    // even when it is used again afterwards without a comparison.
    let only = classify(&in_fn(
        "t",
        "let debugs = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    if debugs == 0 { Ok(()) } else { Err(format!(\"{debugs} lines\")) }.unwrap();",
    ));
    assert_eq!(only.len(), 1, "{only:?}");
    assert!(only[0].absence, "{only:?}");
    // …and a re-binding of the name ENDS the association: a nonzero comparison
    // on the NEW binding is not a comparison of the count.
    let rebound = classify(&in_fn(
        "t",
        "let n = lines.iter().filter(|l| l.contains(\"DEBUG\")).count();\n    assert!(n == 0);\n    let n = lines.len();\n    assert_eq!(n, 5);",
    ));
    assert_eq!(rebound.len(), 1, "{rebound:?}");
    assert!(rebound[0].absence, "{rebound:?}");
}

#[test]
fn the_absence_exemption_reads_the_claims_polarity() {
    // Genuine silence controls: "nothing at DEBUG" is ASSERTED, or "something
    // at DEBUG" is REFUTED — release-safe by construction, EXEMPT.
    for stmt in [
        "assert!(count_at(&lines, \"DEBUG\", \"widget suppressed\") == 0);",
        "assert!(count_at(&lines, \"DEBUG\", \"widget suppressed\") == 0, \"leak\");",
        "assert!(lines_at(&lines, \"DEBUG\", \"widget suppressed\").is_empty());",
        "assert_eq!(count_at(&lines, \"DEBUG\", \"widget suppressed\"), 0);",
        "assert_eq!(count_at(&lines, \"DEBUG\", \"widget suppressed\"), 0, \"leak\");",
        "assert_eq!(\n        count_at(&lines, \"DEBUG\", \"widget suppressed\"),\n        0,\n        \"leak\"\n    );",
        "let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert_eq!(\n        n,\n        0,\n        \"leak\"\n    );",
        "if count_at(&lines, \"DEBUG\", \"widget suppressed\") != 0 { return Err(\"leak\".into()); }",
        "if lines_at(&lines, \"DEBUG\", \"widget suppressed\").is_empty() { Ok(()) } else { Err(\"leak\".to_string()) }.unwrap();",
        "if other == 5 || count_at(&lines, \"DEBUG\", \"widget suppressed\") != 0 { return Err(\"leak\".into()); }",
        "let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert_eq!(n, 0);",
        "let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    if n != 0 { return Err(format!(\"{n} leaked\")); }",
        "let quiet = lines_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert!(quiet.is_empty(), \"got {quiet:?}\");",
        "let quiet = lines_at(&lines, \"DEBUG\", \"widget suppressed\");\n    if !quiet.is_empty() { return Err(format!(\"{quiet:?}\")); }",
    ] {
        let sites = classify(&in_fn("t", stmt));
        assert_eq!(sites.len(), 1, "{stmt}\n{sites:?}");
        assert!(
            sites[0].absence && !sites[0].violates(),
            "a claim of ZERO, asserted (or of NONZERO, refuted) is a silence control:\n{stmt}\n{sites:?}"
        );
    }
    // The negated boolean presence is a zero claim too (a rule-2 marker site).
    let emit = "fn emit() { tracing::debug!(\"widget backlog suppressed (regime open)\"); }\n";
    let negated = classify(&format!(
        "{emit}{}",
        in_fn(
            "t",
            "assert!(!logs_contain(\"widget backlog suppressed\"));"
        )
    ));
    assert_eq!(negated.len(), 1, "{negated:?}");
    assert!(negated[0].absence, "{negated:?}");
    // Inverse polarity: a NONZERO demand in a silence control's clothing — it
    // reads 0 in release, and a walk keyed on token shape alone would exempt
    // every one of these. A VIOLATION, each.
    for stmt in [
        "assert!(count_at(&lines, \"DEBUG\", \"widget suppressed\") != 0);",
        "assert!(!lines_at(&lines, \"DEBUG\", \"widget suppressed\").is_empty());",
        "assert_ne!(count_at(&lines, \"DEBUG\", \"widget suppressed\"), 0);",
        "if count_at(&lines, \"DEBUG\", \"widget suppressed\") == 0 { return Err(\"none\".into()); }",
        "if lines_at(&lines, \"DEBUG\", \"widget suppressed\").is_empty() { return Err(\"none\".into()); }",
        "if count_at(&lines, \"DEBUG\", \"widget suppressed\") == 0 { Err(\"none\".to_string()) } else { Ok(()) }.unwrap();",
        "let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    if n == 0 { return Err(\"none\".into()); }",
        "let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert!(n != 0);",
        "let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert_ne!(n, 0);",
        "let quiet = lines_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert!(!quiet.is_empty());",
        "let quiet = lines_at(&lines, \"DEBUG\", \"widget suppressed\");\n    if quiet.is_empty() { return Err(\"none\".into()); }",
    ] {
        let sites = classify(&in_fn("t", stmt));
        assert_eq!(sites.len(), 1, "{stmt}\n{sites:?}");
        assert!(
            !sites[0].absence && sites[0].violates(),
            "a claim of NONZERO, asserted (or of ZERO, refuted) demands a DEBUG line:\n{stmt}\n{sites:?}"
        );
    }
    // A context the walk does not model exempts nothing (fail closed).
    for stmt in [
        "let ok = count_at(&lines, \"DEBUG\", \"widget suppressed\") == 0;",
        "while count_at(&lines, \"DEBUG\", \"widget suppressed\") != 0 { return Err(\"leak\".into()); }",
        "if count_at(&lines, \"DEBUG\", \"widget suppressed\") != 0 { let n = 1; return Err(format!(\"{n}\")); }",
    ] {
        let sites = classify(&in_fn("t", stmt));
        assert_eq!(sites.len(), 1, "{stmt}\n{sites:?}");
        assert!(
            !sites[0].absence && sites[0].violates(),
            "an unmodelled context is not a silence control:\n{stmt}\n{sites:?}"
        );
    }
}

#[test]
fn every_site_on_a_line_is_classified_on_its_own() {
    // An attached-zero first count must not shield an ungated second one.
    let two = classify(&in_fn(
        "t",
        "assert!(count_at(&lines, \"DEBUG\", \"first marker\") == 0 && count_at(&lines, \"DEBUG\", \"second marker\") == 5);",
    ));
    assert_eq!(two.len(), 2, "{two:?}");
    assert!(two[0].absence && !two[0].violates(), "{two:?}");
    assert!(!two[1].absence && two[1].violates(), "{two:?}");
    assert_eq!(two[1].markers, vec!["second marker".to_string()], "{two:?}");
    // A twin for the first count's marker must not twin the second's.
    let twin = classify(&in_fn(
        "t",
        "let want = debug_lines_expected(5);\n    let (a, b) = (count_at(&lines, \"DEBUG\", \"first marker\"), count_at(&lines, \"DEBUG\", \"second marker\"));\n    assert_eq!((a, b), (want, want));\n    never_loud(&lines, \"first marker\")?;",
    ));
    assert_eq!(twin.len(), 2, "{twin:?}");
    assert!(
        !twin[0].missing_twin() && twin[1].missing_twin(),
        "{twin:?}"
    );
    // Two quiet-only markers on one line are two sites.
    let emit = "fn emit() { tracing::debug!(\"widget backlog suppressed (regime open)\"); tracing::debug!(\"gadget backlog suppressed (regime open)\"); }\n";
    let pair = classify(&format!(
        "{emit}{}",
        in_fn("t", "assert!(logs_contain(\"widget backlog suppressed\") && logs_contain(\"gadget backlog suppressed\"));")
    ));
    assert_eq!(pair.len(), 2, "{pair:?}");
    assert!(pair.iter().all(|s| s.violates()), "{pair:?}");
    // …while a count's own marker is that count's site, not a second one.
    let one = classify(&format!(
        "{emit}const M: &str = \"widget backlog suppressed\";\n{}",
        in_fn(
            "t",
            "let n = count_at(&lines, \"DEBUG\", M);\n    assert_eq!(n, 5);"
        )
    ));
    assert_eq!(one.len(), 1, "{one:?}");
    assert_eq!(
        one[0].markers,
        vec!["widget backlog suppressed".to_string()],
        "{one:?}"
    );
}

#[test]
fn the_walks_blind_spots_are_pinned_on_both_sides() {
    // ---- COVERED: every shape below is a site ----
    let one_violation = |src: String, why: &str| {
        let sites = classify(&src);
        assert_eq!(sites.len(), 1, "{why}: {sites:?}");
        assert!(sites[0].violates(), "{why}: {sites:?}");
        sites
    };
    let count = "let n = count_at(&lines, LEVEL, \"widget suppressed\");\n    assert_eq!(n, 5);";
    let s = one_violation(
        format!("const LEVEL: &str = \"DEBUG\";\n{}", in_fn("t", count)),
        "const level",
    );
    assert_eq!(s[0].markers, vec!["widget suppressed".to_string()]);
    one_violation(
        format!("static LEVEL: &str = \"DEBUG\";\n{}", in_fn("t", count)),
        "static level",
    );
    one_violation(
        in_fn(
            "t",
            "let n = count_at(&lines, r\"DEBUG\", \"widget suppressed\");\n    assert_eq!(n, 5);",
        ),
        "raw-string level",
    );
    one_violation(
        in_fn("t", "for level in [\"DEBUG\"] { assert_eq!(count_at(&lines, level, \"widget suppressed\"), 5); }"),
        "list element",
    );
    // rustfmt wraps a long call one argument per line: the level token is then
    // FIRST on its line, and a line-scoped detector saw no `(`/`,` before it.
    one_violation(
        in_fn(
            "t",
            "let n = count_at(\n        &lines,\n        \"DEBUG\",\n        \"widget suppressed\",\n    );\n    assert_eq!(n, 5);",
        ),
        "rustfmt-wrapped level literal",
    );
    one_violation(
        format!(
            "const LEVEL: &str = \"DEBUG\";\n{}",
            in_fn(
                "t",
                "let n = count_at(\n        &lines,\n        LEVEL,\n        \"widget suppressed\",\n    );\n    assert_eq!(n, 5);",
            )
        ),
        "rustfmt-wrapped const level",
    );
    one_violation(
        in_fn("t", "let n = lines.iter().filter(|l| l.contains(Level::DEBUG.as_str())).count();\n    assert_eq!(n, 5);"),
        "Level::DEBUG path",
    );
    let emit = "fn emit() { tracing::debug!(\"widget backlog suppressed (regime open)\"); }\n";
    let s = one_violation(
        format!(
            "{emit}{}",
            in_fn(
                "t",
                "let marker = \"widget backlog suppressed\";\n    assert!(logs_contain(marker));"
            )
        ),
        "let-bound marker",
    );
    assert!(s[0].kind.contains("via `marker`"), "{s:?}");
    let s = one_violation(
        format!("{emit}{}", in_fn("t", "let marker = \"other text\";\n    let marker = \"widget backlog suppressed\";\n    assert!(logs_contain(marker));")),
        "shadowing let resolves to the latest",
    );
    assert_eq!(s[0].markers, vec!["widget backlog suppressed".to_string()]);
    // A `let` is visible only AFTER it.
    let later = classify(&format!(
        "{emit}{}",
        in_fn(
            "t",
            "assert!(logs_contain(marker));\n    let marker = \"widget backlog suppressed\";"
        )
    ));
    assert!(later.is_empty(), "{later:?}");
    // ---- EXEMPT: a fn that does not exist in release ----
    let cfg = classify("#[cfg(debug_assertions)]\n#[test]\nfn t() {\n    let lines: Vec<&str> = vec![];\n    assert_eq!(count_at(&lines, \"DEBUG\", \"widget suppressed\"), 5);\n}\n");
    assert!(cfg.is_empty(), "{cfg:?}");
    // ---- BOUNDARIES: invisible, documented, pinned so a change is deliberate ----
    let dynamic = classify(&format!(
        "{emit}{}",
        in_fn(
            "t",
            "assert!(logs_contain(&format!(\"widget backlog {}\", \"suppressed\")));"
        )
    ));
    assert!(
        dynamic.is_empty(),
        "a runtime-built marker is invisible: {dynamic:?}"
    );
    let imported = classify(&format!(
        "use other::SUPPRESSED;\n{emit}{}",
        in_fn("t", "assert!(logs_contain(SUPPRESSED));")
    ));
    assert!(
        imported.is_empty(),
        "a use-imported const is invisible: {imported:?}"
    );
    // The twin is accepted by the callee's NAME and argument — never by its
    // body, which lives in `cerulion_core::testing` and is not
    // in the walked fn's file. A stub `never_loud` therefore satisfies the
    // twin rule; this pins that boundary so a change to it is deliberate.
    let stub = classify(&format!(
        "fn never_loud(_: &[&str], _: &str) -> Result<(), String> {{ Ok(()) }}\n{}",
        in_fn(
            "t",
            "let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    assert_eq!(n, debug_lines_expected(5));\n    never_loud(&lines, \"widget suppressed\").unwrap();"
        )
    ));
    assert_eq!(stub.len(), 1, "{stub:?}");
    assert!(
        !stub[0].violates(),
        "the twin is accepted by name + argument, not by the callee's body: {stub:?}"
    );
    // ---- CAUGHT ELSEWHERE: at the helper, and outside any fn ----
    let helper = classify(&format!(
        "fn count_debug(lines: &[&str]) -> usize {{\n    lines_at(lines, \"DEBUG\", \"widget suppressed\").len()\n}}\n{}",
        in_fn("t", "assert_eq!(count_debug(&lines), 5);")
    ));
    assert_eq!(helper.len(), 1, "{helper:?}");
    assert_eq!(helper[0].fn_name.as_deref(), Some("count_debug"));
    assert!(helper[0].violates());
    let mac = classify("macro_rules! pin {\n    () => {\n        let n = count_at(&lines, \"DEBUG\", \"widget suppressed\");\n    };\n}\n");
    assert_eq!(mac.len(), 1, "{mac:?}");
    assert_eq!(mac[0].fn_name, None);
    assert!(mac[0].violates());
}

#[test]
fn a_site_outside_any_fn_is_a_violation() {
    let sites = classify("const N: usize = count_at(&[], \"DEBUG\", \"m\");\n");
    assert_eq!(sites.len(), 1);
    assert!(sites[0].violates());
    assert_eq!(sites[0].fn_name, None);
}

#[test]
#[should_panic(expected = "unterminated")]
fn the_stripper_fails_closed_on_an_unterminated_literal() {
    let _ = strip("fn t() { let s = \"never closed; }\n");
}

#[test]
fn the_stripper_keeps_literals_in_code_and_blanks_them_in_structure() {
    let v = strip("let s = \"a{b\"; let c = '\"'; let r = r#\"x\"y\"#; // gone\n");
    assert_eq!(v.code.len(), v.structure.len());
    assert!(v.code.contains("\"a{b\""));
    assert!(!v.structure.contains('{'), "{}", v.structure);
    assert!(!v.code.contains("gone"));
    assert!(v.code.contains("r#\"x\"y\"#"), "{}", v.code);
}
