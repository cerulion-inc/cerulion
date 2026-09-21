// SPDX-License-Identifier: AGPL-3.0-only
//! The `idle_wait_gate` test seam is NEVER installed by shipped code — a
//! STRUCTURAL guard, because a violation is invisible to every behavioural arm in
//! the crate.
//!
//! # The regression this exists to catch
//!
//! `GatewayEgressPlane::set_idle_wait_gate_for_test` installs a hook that
//! [`GatewayRuntime::idle_wait`] runs BETWEEN its flag scan and its blocking wait —
//! ON the drive thread, blocking it for as long as the hook takes. That is exactly
//! what the wake arms need (it turns an unwinnable race into a rendezvous)
//! and exactly what a shipped daemon must never do: a hook installed on a production
//! path stops the egress gateway's drive loop inside its own idle wait, and because
//! `RunningGateway::drop` JOINS that thread, it wedges the plane's teardown too.
//!
//! The field's own doc comment states the rule ("NEVER set in production, and
//! nothing in `src/` may INSTALL one"). Prose is not a gate. This file is.
//!
//! # Why a classifier rather than a bare `!contains`
//!
//! The obvious guard — "the needle must not appear in shipped code" — is WRONG here,
//! and that is the interesting difference from this file's sibling
//! (`mdns_suppression_confinement_test.rs`, whose needles genuinely have no shipped
//! occurrence). `GatewayEgressPlane::ensure_gateway_booted` legitimately READS the
//! field and forwards whatever is there onto the freshly built `GatewayRuntime`:
//!
//! ```text
//! if let Some(gate) = self.idle_wait_gate.lock()….clone() {
//!     gateway.set_idle_wait_gate_for_test(gate);
//! }
//! ```
//!
//! That is the PLUMBING, not an install: it is a no-op unless something already put
//! a hook in the field, and the only thing that can is the `_for_test` setter. So the
//! invariant this file enforces is the precise one:
//!
//! > **Every surviving mention of the gate in shipped code is a DECLARATION, an
//! > initialisation to `None`, a READ, or the guarded forward — never an install.**
//!
//! Stated as a TOTAL classifier (every occurrence must match one of four sanctioned
//! shapes) rather than an inventory of known-good lines, so a NEW mention anywhere in
//! `src/` fails until it is classified — the "invert, don't enumerate" rule. A call in
//! `main.rs`, in `GatewayEgressPlane::new`, or a direct `= Some(..)` write to the
//! field all fail, because none of them is one of the four.
//!
//! # What "shipped code" means here
//!
//! Cribbed from `mdns_suppression_confinement_test.rs` (which cribs
//! `cerulion_viz/bin/cerulion_vizd/tests/convergence_adoption_test.rs` and
//! `cerulion_cli_engine/tests/convergence_adoption_test.rs`): a DIRECTORY walk over
//! `src/**/*.rs` rather than a hand-written file list (a list reproduces its own
//! failure mode, where the sweep missed the then-shipping `rerun_sink` because nothing
//! enumerated the sites), over a view with comments AND string/char literals blanked
//! (`egress.rs` NAMES the seam in its docs in order to explain the rule, so a naive
//! search is satisfied by exactly the prose this file replaces), failing LOUDLY on an
//! unbalanced scan. Then two further strips define "shipped":
//!
//! * `#[cfg(test)] mod <name> { … }` — a call there ships in no daemon.
//! * the SIGNATURE AND BODY of every `fn …_for_test` — which removes the setter's own
//!   definition, i.e. the one place a `= Some(gate)` write is correct.
//!
//! The stripper is a near-copy of the sibling file's rather than a shared module, for
//! the same reason the repo already carries four adapted copies (`cerulion_cli_engine`,
//! `cerulion_vizd`, `cerulion_netd`, `cerulion_core`'s cdylib log guard): each is tuned
//! to its own needles. This one differs deliberately in ONE respect — it preserves
//! newlines inside block comments and multi-line literals, so a reported line number
//! is the REAL line in the file rather than one that drifts past the first block
//! comment.
//!
//! It lives in `tests/` rather than in the module so its own needle literals are not
//! inside the text it searches.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

/// The substring that finds every mention of the seam.
///
/// `set_idle_wait_gate_for_test` CONTAINS `idle_wait_gate`, so this one needle finds
/// the field, the setter, and any future spelling built out of either.
const NEEDLE: &str = "idle_wait_gate";

/// `cerulion_netd`'s `src/` directory.
fn src_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src")
}

/// Every `*.rs` file under `src/`, RECURSIVELY — never a hand-written list.
///
/// Fails LOUDLY on a walk that yields almost nothing: a silently-empty walk would make
/// every assertion below vacuous.
fn src_files() -> Vec<PathBuf> {
    let mut out = Vec::new();
    walk(&src_dir(), &mut out);
    out.sort();
    assert!(
        out.len() >= 8,
        "the src walk found only {} file(s) under {} — a walk that yields nothing makes \
         every assertion in this file vacuous",
        out.len(),
        src_dir().display()
    );
    out
}

fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
    let entries = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display()))
        .collect::<Result<Vec<_>, _>>()
        .unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display()));
    for entry in entries {
        let path = entry.path();
        if path.is_dir() {
            walk(&path, out);
        } else if path.extension().is_some_and(|e| e == "rs") {
            out.push(path);
        }
    }
}

fn read(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

/// The outcome of stripping comments + string literals.
struct Stripped {
    code: String,
    /// Depth of unclosed `/*` at EOF. Anything but 0 means the tail was silently
    /// dropped — see [`code_only`].
    unclosed_depth: usize,
}

/// A view of `src` with comments AND string/char literals blanked, PRESERVING every
/// newline so byte offsets still map to the file's real line numbers.
///
/// All three halves are load-bearing:
///
/// * COMMENTS, because `egress.rs` NAMES the seam in its docs in order to explain that
///   nothing may install one — a naive search is satisfied by exactly that prose.
/// * STRING LITERALS, because a lone `"` inside a char literal would otherwise open the
///   string arm and swallow everything to the next quote anywhere in the file, leaving
///   `unclosed_depth == 0` so the loud guard could not see it. Only the SHORT char
///   shapes are consumed, so a lifetime survives.
/// * NEWLINES, because this file REPORTS a line number and a drifting one sends a
///   reader to the wrong place (the sibling guard reports only file names, so it does
///   not need this).
fn code_only(src: &str) -> Stripped {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    // Blank `[from, to)` to spaces, keeping newlines.
    let blank = |out: &mut String, from: usize, to: usize| {
        for &b in &bytes[from..to.min(bytes.len())] {
            out.push(if b == b'\n' { '\n' } else { ' ' });
        }
    };
    while i < bytes.len() {
        if depth == 0 {
            // A CHAR literal — only the short shapes, so a lifetime is never eaten.
            if bytes[i] == b'\'' {
                let close = if bytes.get(i + 1) == Some(&b'\\') {
                    (bytes.get(i + 3) == Some(&b'\'')).then_some(i + 3)
                } else {
                    (bytes.get(i + 2) == Some(&b'\'')).then_some(i + 2)
                };
                if let Some(end) = close {
                    blank(&mut out, i, end + 1);
                    i = end + 1;
                    continue;
                }
                // No closer in range ⇒ a lifetime; fall through and keep it.
            }
            // A RAW string literal: `r`, zero or more `#`, then `"`.
            if bytes[i] == b'r' {
                let mut j = i + 1;
                while j < bytes.len() && bytes[j] == b'#' {
                    j += 1;
                }
                if j < bytes.len() && bytes[j] == b'"' {
                    let hashes = j - i - 1;
                    let mut k = j + 1;
                    'raw: while k < bytes.len() {
                        if bytes[k] == b'"' {
                            let mut h = 0usize;
                            while h < hashes && k + 1 + h < bytes.len() && bytes[k + 1 + h] == b'#'
                            {
                                h += 1;
                            }
                            if h == hashes {
                                k = k + 1 + hashes;
                                break 'raw;
                            }
                        }
                        k += 1;
                    }
                    let end = k.min(bytes.len());
                    blank(&mut out, i, end);
                    i = end;
                    continue;
                }
            }
            // A regular (or byte) string literal.
            if bytes[i] == b'"' {
                let mut k = i + 1;
                while k < bytes.len() {
                    match bytes[k] {
                        b'\\' => k += 2,
                        b'"' => {
                            k += 1;
                            break;
                        }
                        _ => k += 1,
                    }
                }
                let end = k.min(bytes.len());
                blank(&mut out, i, end);
                i = end;
                continue;
            }
            if bytes[i..].starts_with(b"//") {
                let start = i;
                while i < bytes.len() && bytes[i] != b'\n' {
                    i += 1;
                }
                blank(&mut out, start, i);
                continue;
            }
        }
        if bytes[i..].starts_with(b"/*") {
            depth += 1;
            out.push_str("  ");
            i += 2;
            continue;
        }
        if depth > 0 && bytes[i..].starts_with(b"*/") {
            depth -= 1;
            out.push_str("  ");
            i += 2;
            continue;
        }
        if depth == 0 {
            let ch = src[i..].chars().next().expect("valid utf-8 boundary");
            out.push(ch);
            i += ch.len_utf8();
        } else {
            // Inside a block comment: keep the newline, blank everything else.
            out.push(if bytes[i] == b'\n' { '\n' } else { ' ' });
            i += 1;
        }
    }
    Stripped {
        code: out,
        unclosed_depth: depth,
    }
}

/// The byte offset just past the brace matching the FIRST `{` at or after `from`, or
/// `None` if the source has no balanced closer (which a compiling file cannot).
fn end_of_block(src: &str, from: usize) -> Option<usize> {
    let open = from + src[from..].find('{')?;
    let mut depth = 0usize;
    for (off, ch) in src[open..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(open + off + 1);
                }
            }
            _ => {}
        }
    }
    None
}

/// Blank every `[start, end)` span, preserving newlines so a failure diagnostic still
/// reads like the file it came from AND line numbers stay exact.
fn blank_spans(src: &str, spans: &mut [(usize, usize)]) -> String {
    spans.sort_unstable();
    let mut out = String::with_capacity(src.len());
    let mut cursor = 0usize;
    for &(start, end) in spans.iter() {
        if start < cursor {
            continue; // Already inside a blanked span.
        }
        out.push_str(&src[cursor..start]);
        for ch in src[start..end].chars() {
            out.push(if ch == '\n' { '\n' } else { ' ' });
        }
        cursor = end;
    }
    out.push_str(&src[cursor..]);
    out
}

/// One file's shipped view, plus the seam names the strip REMOVED.
struct Shipped {
    /// Code a shipped `cerulion-netd` can execute.
    code: String,
    /// Every `fn …_for_test` this file DEFINES, whose signature and body the strip
    /// blanked. Retained because blanking a body HIDES what it does: a helper
    /// `fn install_gate_for_test() { *self.idle_wait_gate.lock()… = Some(hook) }`
    /// disappears, and a production `fn boot() { self.install_gate_for_test() }`
    /// installs a gate while naming the needle NOWHERE. A shipped CALL to a
    /// stripped seam is therefore itself the violation — see
    /// [`stray_seam_calls`], which classifies rather than blanket-fails, because
    /// the ONE sanctioned shape (`ensure_gateway_booted`'s forward) is also a
    /// shipped call to a `_for_test` name.
    for_test_names: Vec<String>,
}

/// The SHIPPED view of one file: comments + literals blanked, then every
/// `#[cfg(test)] mod … { }` block and every `fn …_for_test` (signature AND body)
/// blanked too — the removed seam names retained.
///
/// What survives is code a shipped `cerulion-netd` can execute — the only place a gate
/// INSTALL is a defect.
fn shipped_view_of(path: &Path) -> Shipped {
    let stripped = code_only(&read(path));
    assert_eq!(
        stripped.unclosed_depth,
        0,
        "{}: the stripper ended inside {} unclosed block comment(s), so the tail was \
         dropped and every assertion over it would run on a PREFIX",
        path.display(),
        stripped.unclosed_depth
    );
    let code = stripped.code;

    let mut spans: Vec<(usize, usize)> = Vec::new();

    // (1) `#[cfg(test)] mod <name> { … }`. Matched on the `mod` that FOLLOWS the
    //     attribute, so a `#[cfg(test)]` on some other item is left alone.
    let mut from = 0usize;
    while let Some(rel) = code[from..].find("#[cfg(test)]") {
        let at = from + rel;
        let rest = &code[at + "#[cfg(test)]".len()..];
        let lead = rest.len() - rest.trim_start().len();
        if rest.trim_start().starts_with("mod ") {
            let mod_at = at + "#[cfg(test)]".len() + lead;
            if let Some(end) = end_of_block(&code, mod_at) {
                spans.push((at, end));
                from = end;
                continue;
            }
        }
        from = at + "#[cfg(test)]".len();
    }

    // (2) Every `fn <ident>_for_test` — signature and body. This removes the setter's
    //     own definition, which is the ONE place a `= Some(gate)` write is correct.
    //     The NAME is retained: see `Shipped::for_test_names`.
    let mut for_test_names = Vec::new();
    let mut from = 0usize;
    while let Some(rel) = code[from..].find("_for_test") {
        let at = from + rel;
        // Walk back over the identifier to its start, then require `fn ` before it.
        let mut name_start = at;
        while name_start > 0 {
            let prev = code.as_bytes()[name_start - 1];
            if prev == b'_' || prev.is_ascii_alphanumeric() {
                name_start -= 1;
            } else {
                break;
            }
        }
        let is_definition = code[..name_start].trim_end().ends_with("fn");
        if is_definition {
            if let Some(end) = end_of_block(&code, at) {
                let start = code[..name_start].rfind("fn").unwrap_or(name_start);
                for_test_names.push(code[name_start..at + "_for_test".len()].to_string());
                spans.push((start, end));
                from = end;
                continue;
            }
        }
        from = at + "_for_test".len();
    }

    Shipped {
        code: blank_spans(&code, &mut spans),
        for_test_names,
    }
}

/// The shipped view alone — for the arms that assert over the text.
fn production_code_of(path: &Path) -> String {
    shipped_view_of(path).code
}

// ────────────────────────────────────────────────────────────────────────────
// The classifier.
// ────────────────────────────────────────────────────────────────────────────

/// The four shapes a mention of the gate may take in SHIPPED code.
///
/// Anything else is an install (or a new spelling nobody has adjudicated) and fails.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum Sanctioned {
    /// `idle_wait_gate: Mutex<…>` — the struct field declaration.
    FieldDeclaration,
    /// `idle_wait_gate: Mutex::new(None)` — initialised EMPTY at construction. The
    /// `None` is load-bearing: an init to `Some(..)` is an install.
    InitialisedToNone,
    /// `.idle_wait_gate.lock()` — a READ. Reading cannot install anything.
    GuardedRead,
    /// `gateway.set_idle_wait_gate_for_test(..)` — the guarded forward onto the
    /// freshly built `GatewayRuntime`, a no-op unless the field already holds a hook.
    RuntimeForward,
}

/// One mention of the needle in a shipped view, with enough context to report it.
struct Mention {
    line: usize,
    ident: String,
    context: String,
    verdict: Option<Sanctioned>,
}

fn is_ident_byte(b: u8) -> bool {
    b == b'_' || b.is_ascii_alphanumeric()
}

/// The full identifier containing `at`, as `(start, end)`.
fn ident_span(code: &str, at: usize) -> (usize, usize) {
    let bytes = code.as_bytes();
    let mut start = at;
    while start > 0 && is_ident_byte(bytes[start - 1]) {
        start -= 1;
    }
    let mut end = at;
    while end < bytes.len() && is_ident_byte(bytes[end]) {
        end += 1;
    }
    (start, end)
}

/// The next non-whitespace byte offset at or after `from`.
fn skip_ws(code: &str, from: usize) -> usize {
    let bytes = code.as_bytes();
    let mut i = from;
    while i < bytes.len() && bytes[i].is_ascii_whitespace() {
        i += 1;
    }
    i
}

/// The last non-whitespace byte offset strictly before `before`, if any.
fn prev_non_ws(code: &str, before: usize) -> Option<usize> {
    let bytes = code.as_bytes();
    let mut i = before;
    while i > 0 {
        i -= 1;
        if !bytes[i].is_ascii_whitespace() {
            return Some(i);
        }
    }
    None
}

/// The ONLY method names a sanctioned lock-READ chain may use after `.lock()`.
///
/// A DECLARED inventory rather than a blacklist of mutators: `.lock().unwrap()` and
/// `.lock().unwrap().replace(hook)` are one token apart, both parse as "a `.lock()`
/// follows the field", and only one of them is a read. Anything not listed fails
/// until somebody adjudicates it — the same invert-don't-enumerate rule the
/// classifier itself follows.
const READ_ONLY_LOCK_CHAIN: &[&str] = &[
    "unwrap",
    "unwrap_or_else",
    "expect",
    "clone",
    "as_ref",
    "is_some",
    "is_none",
];

/// The end of the statement containing `from`: the first `;`, `{` or `}` at
/// bracket-depth 0. `None` ⇒ the scan ran off the end (fail CLOSED).
fn statement_end(code: &str, from: usize) -> Option<usize> {
    let bytes = code.as_bytes();
    let mut depth = 0i32;
    let mut i = from;
    while i < bytes.len() {
        match bytes[i] {
            b'(' | b'[' => depth += 1,
            b')' | b']' => {
                if depth == 0 {
                    return Some(i); // A closing bracket we never opened ends it too.
                }
                depth -= 1;
            }
            b';' | b'{' | b'}' if depth == 0 => return Some(i),
            _ => {}
        }
        i += 1;
    }
    None
}

/// Does the statement containing this mention ASSIGN through it?
///
/// `*self.idle_wait_gate.lock().unwrap() = Some(gate)` is a WRITE that reads
/// character-for-character like the sanctioned lock-read up to the `=`. The
/// discriminator is a top-level `=` AFTER the mention (the `=` of an enclosing
/// `if let Some(gate) = …` binding is BEFORE it), excluding every comparison and
/// fat-arrow spelling. Fails CLOSED on an unterminated statement.
fn statement_assigns_through(code: &str, mention_end: usize) -> bool {
    let Some(stop) = statement_end(code, mention_end) else {
        return true; // Could not delimit it ⇒ refuse to call it a read.
    };
    let bytes = code.as_bytes();
    let mut depth = 0i32;
    let mut i = mention_end;
    while i < stop {
        match bytes[i] {
            b'(' | b'[' => depth += 1,
            b')' | b']' => depth -= 1,
            b'=' if depth == 0 => {
                let prev = i.checked_sub(1).map(|p| bytes[p]);
                let next = bytes.get(i + 1).copied();
                let comparison = matches!(prev, Some(b'=' | b'!' | b'<' | b'>'))
                    || matches!(next, Some(b'=' | b'>'));
                if !comparison {
                    return true;
                }
            }
            _ => {}
        }
        i += 1;
    }
    false
}

/// Is the method chain after `.lock()` made ONLY of declared read-only calls?
///
/// Deliberately returns `true` when it meets an `=` — that shape is
/// [`statement_assigns_through`]'s to refuse, so each rule stays independently
/// testable. Fails CLOSED on anything it cannot parse.
fn lock_chain_is_read_only(code: &str, mention_end: usize) -> bool {
    let bytes = code.as_bytes();
    let mut i = skip_ws(code, mention_end);
    if !code[i..].starts_with(".lock()") {
        return false;
    }
    i += ".lock()".len();
    loop {
        i = skip_ws(code, i);
        match bytes.get(i) {
            None => return false, // Ran off the end ⇒ fail closed.
            Some(b'.') => {
                let (s, e) = ident_span(code, i + 1);
                if s != i + 1 || e == s || !READ_ONLY_LOCK_CHAIN.contains(&&code[s..e]) {
                    return false;
                }
                let j = skip_ws(code, e);
                if bytes.get(j) != Some(&b'(') {
                    return false;
                }
                let mut depth = 0i32;
                let mut k = j;
                loop {
                    match bytes.get(k) {
                        None => return false,
                        Some(b'(') => depth += 1,
                        Some(b')') => {
                            depth -= 1;
                            if depth == 0 {
                                break;
                            }
                        }
                        _ => {}
                    }
                    k += 1;
                }
                i = k + 1;
            }
            // The statement (or expression) ends: a pure read chain.
            Some(b';' | b'{' | b'}' | b',' | b')' | b']') => return true,
            // An assignment: not this rule's to judge (see the doc above).
            Some(b'=') => return true,
            Some(_) => return false,
        }
    }
}

/// The single BARE IDENTIFIER argument of the call whose name ends at `name_end`,
/// or `None` for anything else (a constructor, a closure, a path, a literal, several
/// arguments). `gateway.set_idle_wait_gate_for_test(Arc::new(|| {}))` is a NEW
/// install wearing the sanctioned receiver, and this is what refuses it.
fn sole_bare_identifier_argument(code: &str, name_end: usize) -> Option<String> {
    let bytes = code.as_bytes();
    let open = skip_ws(code, name_end);
    if bytes.get(open) != Some(&b'(') {
        return None;
    }
    let mut depth = 0i32;
    let mut i = open;
    let close = loop {
        match bytes.get(i) {
            None => return None,
            Some(b'(') => depth += 1,
            Some(b')') => {
                depth -= 1;
                if depth == 0 {
                    break i;
                }
            }
            _ => {}
        }
        i += 1;
    };
    let arg = code[open + 1..close].trim();
    let bare =
        !arg.is_empty() && arg.bytes().all(is_ident_byte) && !arg.as_bytes()[0].is_ascii_digit();
    bare.then(|| arg.to_string())
}

/// Does the code before `before` end with the whole keyword `kw` (whitespace
/// skipped)? Returns the keyword's own offset.
///
/// Whole-token, never a suffix: `applet` does not end with `let`.
fn ends_with_keyword(code: &str, before: usize, kw: &str) -> Option<usize> {
    let head = code[..before].trim_end();
    let start = head.len().checked_sub(kw.len())?;
    if !head.ends_with(kw) {
        return None;
    }
    if start > 0 && is_ident_byte(head.as_bytes()[start - 1]) {
        return None;
    }
    Some(start)
}

/// Is the `Some(..)` pattern at `pattern_at` the binding of an `if let` /
/// `while let` — as opposed to a MATCH ARM, which reaches the same
/// `Some(x)` … `=` shape (`=>`) over a scrutinee this rule knows nothing about?
fn is_refutable_let_binding(code: &str, pattern_at: usize) -> bool {
    let Some(let_at) = ends_with_keyword(code, pattern_at, "let") else {
        return false;
    };
    ends_with_keyword(code, let_at, "if").is_some()
        || ends_with_keyword(code, let_at, "while").is_some()
}

/// The `{` a binding's block opens at, scanning from `from` at bracket-depth 0.
///
/// Fails CLOSED: a `;` first means there is no block (a `let … else`, a plain
/// statement), and a scan that runs off the end means the source is not something
/// this rule can adjudicate.
fn block_open_after(code: &str, from: usize) -> Option<usize> {
    let bytes = code.as_bytes();
    let mut depth = 0i32;
    let mut i = from;
    while i < bytes.len() {
        match bytes[i] {
            b'(' | b'[' => depth += 1,
            b')' | b']' => depth -= 1,
            b'{' if depth == 0 => return Some(i),
            b';' if depth == 0 => return None,
            _ => {}
        }
        i += 1;
    }
    None
}

/// Is the expression in `code[start..end)` a read of THE GATE FIELD under its own
/// lock, ending in `.clone()`?
///
/// This is what makes the forwarded value a no-op unless a test already installed a
/// hook. It reuses [`lock_chain_is_read_only`] rather than restating it, and
/// requires exactly ONE mention so a compound expression cannot smuggle a second.
fn rhs_is_field_lock_read_clone(code: &str, start: usize, end: usize) -> bool {
    let rhs = &code[start..end];
    let Some(rel) = rhs.find(NEEDLE) else {
        return false;
    };
    let (s, e) = ident_span(code, start + rel);
    if &code[s..e] != "idle_wait_gate" {
        return false;
    }
    if prev_non_ws(code, s).map(|p| code.as_bytes()[p]) != Some(b'.') {
        return false;
    }
    if !code[skip_ws(code, e)..].starts_with(".lock()") {
        return false;
    }
    if !lock_chain_is_read_only(code, e) {
        return false;
    }
    if rhs[rel + NEEDLE.len()..].contains(NEEDLE) {
        return false;
    }
    rhs.trim_end().ends_with(".clone()")
}

/// Is the identifier at `s` the binding of a MATCH ARM — `Some(x) =>`, `(a, x) =>`,
/// a bare `x =>`?
///
/// Reads FORWARD, which is what makes it independent of [`pattern_head_before`]: from
/// the end of the identifier only pattern-CLOSING punctuation may intervene before the
/// `=>`. A use inside an arm BODY (`A => f(x), B => …`) reaches the NEXT arm's pattern
/// instead and stops at its first identifier byte, so the two do not collide.
///
/// Residual, stated because it is real: an arm carrying a GUARD (`Some(x) if c => …`)
/// stops at the `if` and is left to [`pattern_head_before`], which walks back to the
/// `match` and catches it there.
fn is_match_arm_binding(code: &str, s: usize) -> bool {
    let bytes = code.as_bytes();
    let (_, mut i) = ident_span(code, s);
    while i < bytes.len() {
        match bytes[i] {
            b')' | b']' | b'}' | b',' | b'|' => i += 1,
            b if b.is_ascii_whitespace() => i += 1,
            b'=' => return bytes.get(i + 1) == Some(&b'>'),
            _ => return false,
        }
    }
    false
}

/// Is the `>` at `p` the CLOSE of a generic argument list, rather than the tail of an
/// ARROW (`=>`, `->`)?
///
/// This is the whole of the ambiguity that made `>` a stop byte for both backward walks
/// below, and the three shapes that reach them are separated in three different places:
///
/// * a GENERIC close — `|a: Arc<T>, x|`, `Some::<T>(x)` — must be SKIPPED, or every
///   binding after a generically-typed one is invisible;
/// * an ARROW's tail — a match arm's `=>`, a return type's `->` — is a real boundary,
///   and THIS predicate is what keeps it one. It is also why a generic list containing
///   an arrow (`Arc<dyn Fn() -> u8>`) closes ONCE rather than twice;
/// * a COMPARISON — `f(a > b, x)` — is neither, and is refused by the SCAN rather than
///   here: an expression that is not a type carries no matching `<` before the
///   statement boundary, so [`matching_angle_open`] gives up and the walk stops exactly
///   where it used to.
///
/// The discriminator is the byte BEFORE the `>`, because that is the one place the
/// three differ that a BACKWARD walk has already read: no type ends in `-` or `=`.
fn is_generic_close(code: &str, p: usize) -> bool {
    !matches!(
        p.checked_sub(1).map(|q| code.as_bytes()[q]),
        Some(b'-') | Some(b'=')
    )
}

/// The offset of the `<` opening the generic argument list that the `>` at `p` closes,
/// or `None` if this `>` closes no such list.
///
/// Depth-tracked, so `Arc<Mutex<Option<T>>>` resolves to its outermost `<`, and BOUNDED
/// by the bytes a generic argument list cannot contain (`;`, `{`, `}`, `|`) so a `>`
/// that is really a comparison runs into the statement boundary and refuses instead of
/// letting the caller's walk wander back across it.
///
/// Fails CLOSED: anything it cannot resolve — a missing opener, a scan that reaches the
/// start of the file, a caller that passed something other than a `>` — is `None`, and
/// every caller treats `None` exactly as it treated `>` before this rule existed.
fn matching_angle_open(code: &str, p: usize) -> Option<usize> {
    let bytes = code.as_bytes();
    if bytes.get(p) != Some(&b'>') || !is_generic_close(code, p) {
        return None;
    }
    let mut depth = 0usize;
    let mut i = p;
    loop {
        match bytes[i] {
            b'>' if is_generic_close(code, i) => depth += 1,
            b'<' => {
                depth = depth.checked_sub(1)?;
                if depth == 0 {
                    return Some(i);
                }
            }
            // A generic argument list cannot contain these, so meeting one means this
            // `>` was never a generic close.
            b';' | b'{' | b'}' | b'|' => return None,
            _ => {}
        }
        i = i.checked_sub(1)?;
    }
}

/// Is the identifier at `s` a CLOSURE PARAMETER — `|x|`, `|a, x|`, `move |x|`?
///
/// Walks back over what a parameter list can hold (names, `,`, `:`, `&`, and a generic
/// argument list) to the opening `|`, then separates that `|` from an or-pattern and
/// from a bitwise/logical or by what PRECEDES it: a closure's `|` follows a bracket, a
/// comma, an operator or `move` — never a value, and never another `|` (`||` is an
/// empty parameter list or a logical or, and in both the identifier after it is an
/// operand rather than a parameter).
///
/// The generic arm is the one that is not obvious. A parameter's TYPE can contain
/// `<…>`, so a walk that stopped at `>` saw nothing after the first generically-typed
/// parameter — `|a: Arc<T>, x|` reported `x` as unbound, which is the direction that
/// SILENTLY ADMITS an install (see [`is_binding_position`]'s "which way this detector
/// must be wrong"). [`matching_angle_open`] skips the whole list, and keeps every
/// non-generic `>` a stop byte exactly as before.
fn is_closure_parameter(code: &str, s: usize) -> bool {
    let bytes = code.as_bytes();
    let mut i = s;
    loop {
        let Some(p) = prev_non_ws(code, i) else {
            return false;
        };
        match bytes[p] {
            b'|' => {
                let Some(before) = prev_non_ws(code, p) else {
                    return true;
                };
                return match bytes[before] {
                    b'|' => false,
                    b')' | b']' => false,
                    b if is_ident_byte(b) => ends_with_keyword(code, p, "move").is_some(),
                    _ => true,
                };
            }
            b',' | b':' | b'&' => i = p,
            b'>' => match matching_angle_open(code, p) {
                Some(open) => i = open,
                None => return false,
            },
            b if is_ident_byte(b) => i = ident_span(code, p).0,
            _ => return false,
        }
    }
}

/// Does a `let`, a `for` or a `match` arm pattern ENCLOSE the identifier at `s`?
///
/// Walks back through the pattern's own punctuation — nested groups, path heads,
/// commas, struct-field colons — to the keyword that introduced it, so it recognises
/// every SHAPE a pattern can take (`let (a, x)`, `let Foo { x }`, `if let A::B(x)`,
/// `for (i, x) in …`, `match … { Some(x) => … }`) rather than the single `Some(x)` form
/// the first version of this rule knew.
///
/// What keeps an ordinary USE from resolving to some enclosing binding is that the walk
/// STOPS at every byte a pattern cannot contain: `=` ends a `let`'s pattern, `>` (of
/// `=>`) a match arm's, `;` a statement, `.` a method chain — and the keyword `in` ends
/// a `for`'s, which is what stops a use in a loop BODY from reaching that loop's `for`.
/// Item keywords stop it too, so the walk can never wander into a previous function.
///
/// The `>` of a `=>` is still a stop byte; the `>` closing a GENERIC argument list is
/// not, because a pattern's path can carry one (`if let Some::<Arc<T>>(x) = …`) and a
/// walk that stopped there would report `x` as unbound — the direction that silently
/// admits an install. [`matching_angle_open`] is what tells the two apart, and it
/// refuses everything that is neither.
///
/// `match` needs one thing more, because a SCRUTINEE reaches it just as an arm pattern
/// does (`match kind_of(&x) {` walks back through the call to the keyword): an arm
/// pattern lives inside the match's braces, so it must have crossed an enclosing `{`.
fn pattern_head_before(code: &str, s: usize) -> bool {
    let bytes = code.as_bytes();
    let mut i = s;
    let mut depth = 0i32;
    let mut crossed_brace = false;
    loop {
        let Some(p) = prev_non_ws(code, i) else {
            return false;
        };
        i = p;
        match bytes[p] {
            b')' | b']' | b'}' => depth += 1,
            b'(' | b'[' | b'{' => {
                // At depth 0 this is a group ENCLOSING the identifier, so the walk
                // continues out of it to look for its head.
                if depth == 0 && bytes[p] == b'{' {
                    crossed_brace = true;
                }
                depth = (depth - 1).max(0);
            }
            b',' | b'&' | b'@' | b'|' | b':' => {}
            // A generic argument list in the pattern's own path. An ARROW's `>` and a
            // comparison's both resolve to `None` here and stop the walk, so the
            // match-arm and scrutinee rules above are untouched.
            b'>' => match matching_angle_open(code, p) {
                Some(open) => i = open,
                None => return false,
            },
            b if is_ident_byte(b) => {
                let (start, end) = ident_span(code, p);
                match &code[start..end] {
                    "let" | "for" => return true,
                    "match" => return crossed_brace,
                    // Past `in` we are in a `for`'s ITERATOR expression, and past that
                    // in its body — neither is the loop's pattern.
                    "in" => return false,
                    // A pattern never spans an item boundary.
                    "fn" | "impl" | "mod" | "struct" | "enum" | "trait" | "use" | "else" => {
                        return false;
                    }
                    // A path segment (`Some`, `Foo`), or the callee of a call —
                    // whichever it is, the verdict is decided further back.
                    _ => {}
                }
                i = start;
            }
            _ => return false,
        }
    }
}

/// Is the identifier at `s` being BOUND rather than used?
///
/// # Which way this detector must be wrong
///
/// A verdict of BOUND makes [`rebound_between`] report a re-bind, which makes
/// [`argument_is_the_field_binding`] reject that candidate binding, which — with no
/// other binding vouching for the forward — makes [`classify`] return `None` and the
/// walk REFUSE. So an over-detection costs a LOUD false failure while an
/// under-detection SILENTLY admits an install, and the rule must therefore lean toward
/// "bound" wherever it cannot tell. (A spurious verdict can only ever remove a
/// candidate from consideration; it can never make some OTHER pattern vouch for a
/// forward it would not otherwise have vouched for, because each candidate is judged on
/// its own five conditions.)
///
/// Four families, because a name can be taken over by any of them and the first version
/// of this rule knew only the first:
///
/// 1. an immediate binding keyword — `let x`, `let mut x`, `for x`, `ref mut x`;
/// 2. a `let` / `for` / `match`-arm PATTERN of any shape, via [`pattern_head_before`];
/// 3. a match ARM, via [`is_match_arm_binding`];
/// 4. a CLOSURE PARAMETER, via [`is_closure_parameter`].
///
/// **Family 1 is SUBSUMED by family 2 — measured, not assumed.** Narrowing its keyword
/// list back to `let` alone kills NOTHING (the `for` fixture is refused by the pattern
/// walk, which finds the same keyword one step further back), so no fixture is
/// attributable to it. It is kept because it states the rule a reader looks for first
/// and decides the common case without walking anything — recorded the same way as the
/// match-arm triple guard above, rather than as a claim that every
/// conjunct is load-bearing.
///
/// Families 2, 3 and 4 ARE each independently testable: deleting the pattern
/// walk fails the `tuple` and `struct` fixtures, the forward reader the `match_arm` one,
/// and the closure rule the `closure_param` one.
///
/// Both BACKWARD walks (2 and 4) additionally read over a GENERIC argument list, which
/// they used to stop at — so a parameter or a path element after a generically-typed one
/// was invisible and its re-bind went unseen, silently. The two arms are attributed
/// SEPARATELY, because they are not interchangeable and MEASURING that is what stopped
/// one of them shipping unpinned:
///
/// * the CLOSURE walk's arm is what the `closure_param_after_generic` and
///   `closure_param_after_nested_generic` fixtures kill, and the only thing that
///   catches the real-tree plant recorded on the PR — the pattern walk cannot help
///   there, since it reads over `|` and reaches the `=` of the enclosing `let` first;
/// * the PATTERN walk's arm is killed by `let_pattern_with_a_turbofish_path` alone.
///
/// What keeps them from OVER-detecting — the direction that would refuse the production
/// forward — is that a `>` which is not a generic close still stops the walk, pinned by
/// the three angle lines of the `f4_uses_control` anti-tautology arm and by
/// [`the_angle_scan_separates_a_generic_close_from_an_arrow_and_a_comparison`].
///
/// **The REAL-TREE plant, because a fixture is a string this file wrote and the thing
/// under test is a walk over `src/`.** `ensure_gateway_booted`'s forward was replaced
/// with the "extract a helper" refactor in its most ordinary form — a closure taking
/// the plane's own `Arc<TransportManager>` first and the hook second, called with a
/// fresh `Arc::new(|| {})` — which COMPILES and installs a gate on every boot. Measured
/// against the committed baseline: the extended walk REFUSES it at `egress.rs:524`
/// naming the line, and with the closure arm reverted the same plant leaves
/// [`no_production_path_installs_an_idle_wait_gate`] GREEN. The plant is not
/// checked in — the fixtures are its standing form — but the pair is the evidence that
/// this guard covers shipped code rather than only the strings below.
fn is_binding_position(code: &str, s: usize) -> bool {
    for kw in ["let", "for", "ref"] {
        if ends_with_keyword(code, s, kw).is_some() {
            return true;
        }
    }
    if let Some(mut_at) = ends_with_keyword(code, s, "mut") {
        for kw in ["let", "for", "ref"] {
            if ends_with_keyword(code, mut_at, kw).is_some() {
                return true;
            }
        }
        // `(mut x, y)`, `|mut x|` — a `mut` introduced by its pattern GROUP. The `&mut`
        // spelling is deliberately absent: that is a borrow of an existing binding.
        if let Some(p) = prev_non_ws(code, mut_at) {
            if matches!(code.as_bytes()[p], b'(' | b'[' | b'{' | b',' | b'|') {
                return true;
            }
        }
    }
    is_match_arm_binding(code, s) || is_closure_parameter(code, s) || pattern_head_before(code, s)
}

/// Is `arg` RE-BOUND or ASSIGNED between `from` and `to`?
///
/// A binding proves nothing about a name that was shadowed before it was forwarded, and
/// Rust has more ways to take a name over than `let`: a tuple or struct pattern, a
/// `for`, a closure parameter, a match arm. [`is_binding_position`] knows all four —
/// each one it did NOT know was a shape in which a shipped forward could hand on a
/// freshly built hook while this walk still called it the field's own value.
///
/// Whole-identifier matching throughout — `gateway` is not `gate`.
fn rebound_between(code: &str, from: usize, to: usize, arg: &str) -> bool {
    let bytes = code.as_bytes();
    let mut i = from;
    while let Some(rel) = code[i..to].find(arg) {
        let at = i + rel;
        let (s, e) = ident_span(code, at);
        i = e.max(at + arg.len());
        if &code[s..e] != arg {
            continue;
        }
        if is_binding_position(code, s) {
            return true;
        }
        let nx = skip_ws(code, e);
        if bytes.get(nx) == Some(&b'=') && !matches!(bytes.get(nx + 1), Some(b'=') | Some(b'>')) {
            return true;
        }
    }
    false
}

/// Was `arg` bound, in a block ENCLOSING this call, by a read of the gate field
/// under its lock — and not re-bound since?
///
/// The sanctioned forward hands on the value `ensure_gateway_booted` just CLONED out
/// of the field, so it is a no-op unless a test already installed a hook. A forward
/// handed anything else is an install, whatever its receiver is called.
///
/// SYNTAX- AND SCOPE-AWARE, because the substring form this replaces was neither: it
/// searched back for `Some(<arg>)` followed by text containing `.lock()` and the
/// needle, which a MATCH ARM satisfies (`Some(gate) =>` starts with `=`, and any
/// field read inside the arm supplies the rest) although `gate` came from the
/// scrutinee, and which a later `let gate = …` cannot disturb because the search
/// took the LAST pattern rather than the LIVE binding. Five conditions now:
///
/// 1. the pattern is an `if let` / `while let` binding, never a match arm;
/// 2. it is followed by `=`, never `=>`;
/// 3. its right-hand side is the field's own lock-read-clone;
/// 4. the block it opens ENCLOSES the call (so a binding in another function, or in
///    a sibling block, cannot vouch for it);
/// 5. nothing re-binds or assigns `arg` between that block and the call.
///
/// **The MATCH-ARM shape is TRIPLY guarded, measured rather than assumed.**
/// Conditions 1, 2 and 3 each refuse it on their own, so no single dropped conjunct
/// is attributable to that fixture — it fails only when all three are dropped, and
/// what it pins is the OUTCOME (a real evasion shape is refused), not any one rule.
/// The reason is Rust's own syntax: a `Some(x)` pattern followed by a bare `=` is a
/// `let`-family binding or a destructuring assignment (which ends in `;`, so
/// condition 3's `block_open_after` refuses it), and a match arm's `=>` leaves
/// `> …` as the right-hand side, which carries no needle. Conditions 1 and 2 are
/// kept anyway because they state the rule a reader needs — this must be a BINDING,
/// of this field — rather than leaning on those coincidences.
///
/// Conditions 3, 4 and 5 ARE each independently testable, pinned by the
/// other-field, sibling-block and shadowed fixtures respectively.
///
/// Fails CLOSED throughout: anything it cannot parse is refused.
fn argument_is_the_field_binding(code: &str, call_start: usize, arg: &str) -> bool {
    let pattern = format!("Some({arg})");
    let mut from = 0usize;
    while let Some(rel) = code[from..call_start].find(&pattern) {
        let at = from + rel;
        from = at + pattern.len();

        if !is_refutable_let_binding(code, at) {
            continue;
        }
        let eq = skip_ws(code, at + pattern.len());
        if code.as_bytes().get(eq) != Some(&b'=') || code.as_bytes().get(eq + 1) == Some(&b'>') {
            continue;
        }
        let Some(brace) = block_open_after(code, eq + 1) else {
            continue;
        };
        if !rhs_is_field_lock_read_clone(code, eq + 1, brace) {
            continue;
        }
        let Some(block_end) = end_of_block(code, brace) else {
            continue;
        };
        if !(brace < call_start && call_start < block_end) {
            continue;
        }
        if rebound_between(code, brace, call_start, arg) {
            continue;
        }
        return true;
    }
    false
}

/// Classify one mention. `None` ⇒ not sanctioned ⇒ a failure.
fn classify(code: &str, start: usize, end: usize) -> Option<Sanctioned> {
    let ident = &code[start..end];
    match ident {
        "idle_wait_gate" => {
            let after = skip_ws(code, end);
            let dotted = prev_non_ws(code, start).is_some_and(|p| code.as_bytes()[p] == b'.');
            if dotted {
                // A `.lock()` follows the field on BOTH the sanctioned read and the
                // write `*self.idle_wait_gate.lock().unwrap() = Some(gate)`, so the
                // `.lock()` alone decides nothing. Two further rules do, each
                // refusing a shape the other admits.
                if !code[after..].trim_start().starts_with(".lock()") {
                    return None;
                }
                if statement_assigns_through(code, end) {
                    return None;
                }
                if !lock_chain_is_read_only(code, end) {
                    return None;
                }
                return Some(Sanctioned::GuardedRead);
            }
            if code.as_bytes().get(after) != Some(&b':') {
                return None;
            }
            let value = code[skip_ws(code, after + 1)..].trim_start();
            if value.starts_with("Mutex::new(None)") {
                return Some(Sanctioned::InitialisedToNone);
            }
            if value.starts_with("Mutex<") {
                return Some(Sanctioned::FieldDeclaration);
            }
            None
        }
        "set_idle_wait_gate_for_test" => {
            // The ONE sanctioned call site forwards onto the local `GatewayRuntime`
            // built two lines above — never onto a plane. The RECEIVER is only half
            // of it: a new install can spell its variable `gateway` too, so the
            // ARGUMENT must be the value already cloned out of the field.
            let dot = prev_non_ws(code, start)?;
            if code.as_bytes()[dot] != b'.' {
                return None;
            }
            let recv_end = prev_non_ws(code, dot)? + 1;
            let (recv_start, _) = ident_span(code, recv_end - 1);
            if &code[recv_start..recv_end] != "gateway" {
                return None;
            }
            let arg = sole_bare_identifier_argument(code, end)?;
            argument_is_the_field_binding(code, start, &arg).then_some(Sanctioned::RuntimeForward)
        }
        _ => None,
    }
}

/// One shipped call to a `_for_test` seam whose body the strip removed.
struct SeamCall {
    line: usize,
    name: String,
    context: String,
}

/// Every shipped call to a stripped `_for_test` seam that the needle classifier does
/// NOT already adjudicate.
///
/// Calls whose name carries [`NEEDLE`] are EXCLUDED here and handled by
/// [`classify`] instead — the sanctioned boot forward is one of them, so a blanket
/// "shipped code may not call a test seam" would refuse the one shape that is
/// correct. What remains is a seam whose NAME says nothing, whose body
/// the strip hid, and whose shipped caller therefore names the needle nowhere.
fn stray_seam_calls(code: &str, seam_names: &BTreeSet<String>) -> Vec<SeamCall> {
    let mut out = Vec::new();
    for name in seam_names {
        if name.contains(NEEDLE) {
            continue;
        }
        let mut from = 0usize;
        while let Some(rel) = code[from..].find(name.as_str()) {
            let at = from + rel;
            let (s, e) = ident_span(code, at);
            from = e.max(at + name.len());
            if &code[s..e] != name.as_str() {
                continue; // A longer identifier merely CONTAINING the seam name.
            }
            if code.as_bytes().get(skip_ws(code, e)) != Some(&b'(') {
                continue; // Named, not called (a path in a `use`, say).
            }
            let line_start = code[..s].rfind('\n').map(|n| n + 1).unwrap_or(0);
            let line_end = code[s..].find('\n').map(|n| s + n).unwrap_or(code.len());
            out.push(SeamCall {
                line: code[..s].matches('\n').count() + 1,
                name: name.clone(),
                context: code[line_start..line_end].trim().to_string(),
            });
        }
    }
    out.sort_by_key(|c| c.line);
    out
}

/// Every mention of the needle in one file's SHIPPED view, classified.
fn mentions_in(path: &Path) -> Vec<Mention> {
    mentions_in_code(&production_code_of(path))
}

/// [`mentions_in`] over an already-computed shipped view.
fn mentions_in_code(code: &str) -> Vec<Mention> {
    let mut out = Vec::new();
    let mut from = 0usize;
    while let Some(rel) = code[from..].find(NEEDLE) {
        let at = from + rel;
        let (start, end) = ident_span(code, at);
        let line = code[..start].matches('\n').count() + 1;
        let line_start = code[..start].rfind('\n').map(|n| n + 1).unwrap_or(0);
        let line_end = code[start..]
            .find('\n')
            .map(|n| start + n)
            .unwrap_or(code.len());
        out.push(Mention {
            line,
            ident: code[start..end].to_string(),
            context: code[line_start..line_end].trim().to_string(),
            verdict: classify(code, start, end),
        });
        from = end.max(at + NEEDLE.len());
    }
    out
}

/// The WHOLE adjudication, over already-computed shipped views: every violation
/// found, and every mention that WAS sanctioned.
///
/// Pure and separately callable so the fixtures below drive the SAME decision the
/// walk makes — against the real (passing) tree every failure branch here is
/// unreachable, which is exactly how a broken rule ships green (the
/// `native_ros2_messages::adjudicate` precedent).
fn adjudicate(views: &[(PathBuf, Shipped)]) -> (Vec<String>, Vec<(String, Sanctioned)>) {
    let seam_names: BTreeSet<String> = views
        .iter()
        .flat_map(|(_, s)| s.for_test_names.iter().cloned())
        .collect();
    let mut violations = Vec::new();
    let mut sanctioned = Vec::new();

    // A shipped CALL to a seam whose BODY the strip hid. The body could install
    // the gate while the caller names the needle nowhere, so the needle classifier is
    // structurally blind to it.
    for (f, view) in views {
        for call in stray_seam_calls(&view.code, &seam_names) {
            violations.push(format!(
                "{}:{} CALLS the test seam `{}` from code a SHIPPED cerulion-netd \
                 executes:\n\n    {}\n\n\
                 A `fn …_for_test` body is BLANKED by this guard's strip, so what it does \
                 is invisible here — a seam that installs an idle-wait gate would park the \
                 egress gateway's drive thread inside `GatewayRuntime::idle_wait` (and wedge \
                 the teardown that JOINS it) while no shipped line mentions the gate at all. \
                 A test seam may be called only from a `#[cfg(test)] mod` or another \
                 `fn …_for_test`.",
                f.display(),
                call.line,
                call.name,
                call.context
            ));
        }
    }

    for (f, view) in views {
        for m in mentions_in_code(&view.code) {
            match m.verdict {
                Some(kind) => sanctioned.push((f.display().to_string(), kind)),
                None => violations.push(format!(
                    "{}:{} mentions `{}` from code a SHIPPED cerulion-netd executes, in a shape \
                     that is not one of the four sanctioned ones:\n\n    {}\n\n\
                     Installing an idle-wait gate on a production path STOPS the egress \
                     gateway's drive thread inside `GatewayRuntime::idle_wait` — between its \
                     flag scan and its blocking wait — for as long as the hook takes. Because \
                     `RunningGateway::drop` JOINS that thread, it wedges the plane's teardown \
                     with it. The hook exists ONLY so the wake arms can rendezvous with \
                     that window instead of racing it.\n\n\
                     The sanctioned shapes are: the field declaration; \
                     `idle_wait_gate: Mutex::new(None)` at construction; a READ \
                     (`.idle_wait_gate.lock()`); and the guarded forward \
                     `gateway.set_idle_wait_gate_for_test(..)` in `ensure_gateway_booted`, \
                     which is a no-op unless a test already installed one. An install \
                     belongs in a `#[cfg(test)] mod` or another `fn …_for_test` seam.",
                    f.display(),
                    m.line,
                    m.ident,
                    m.context
                )),
            }
        }
    }
    (violations, sanctioned)
}

/// THE pin: no shipped code path in `cerulion_netd` INSTALLS a lost-wakeup-window hook.
///
/// The failure mode it closes is not subtle once it happens and impossible to see
/// before: a hook on a production path parks the egress gateway's drive thread inside
/// its own idle wait, and `RunningGateway::drop` then JOINS that thread.
#[test]
fn no_production_path_installs_an_idle_wait_gate() {
    let files = src_files();
    let scanned: Vec<String> = files
        .iter()
        .map(|f| f.file_name().unwrap().to_string_lossy().to_string())
        .collect();
    let saw_needle_before_stripping = files.iter().any(|f| read(f).contains(NEEDLE));

    // The shipped view of every file, computed ONCE — the seam-call check needs the
    // UNION of the names, because a seam defined in one module is callable from any
    // other.
    let views: Vec<(PathBuf, Shipped)> = files
        .iter()
        .map(|f| (f.clone(), shipped_view_of(f)))
        .collect();
    assert!(
        views.iter().any(|(_, s)| !s.for_test_names.is_empty()),
        "the strip removed no `fn …_for_test` at all, so the seam-call check is vacuous \
         — `src/` defines several, starting with `set_idle_wait_gate_for_test`"
    );

    let (violations, sanctioned) = adjudicate(&views);
    if let Some(first) = violations.first() {
        panic!("{first}");
    }

    // ANTI-TAUTOLOGY (1): the walk reached the modules that matter — otherwise a broken
    // walk passes in silence.
    for known in ["egress.rs", "main.rs", "daemon.rs", "lib.rs"] {
        assert!(
            scanned.iter().any(|f| f == known),
            "the src walk did not reach {known}; it saw {scanned:?}"
        );
    }

    // ANTI-TAUTOLOGY (2): the needle really is present in the tree BEFORE stripping.
    // Without this, deleting or renaming the seam would make the whole test vacuously
    // green while the rule it enforces quietly stopped existing.
    assert!(
        saw_needle_before_stripping,
        "no file under {} mentions `{NEEDLE}` at all — if the seam was renamed, rename it \
         here too; if it was DELETED, delete this guard with it",
        src_dir().display()
    );

    // ANTI-TAUTOLOGY (3): all four sanctioned shapes are ACTUALLY REACHED. A classifier
    // nothing exercises would accept anything the day someone widens it, and this is
    // also what notices the plumbing being deleted (which would silently make the
    // `_for_test` setter INERT — a seam that installs a hook no gateway ever runs).
    let kinds: Vec<Sanctioned> = sanctioned.iter().map(|(_, k)| *k).collect();
    for expected in [
        Sanctioned::FieldDeclaration,
        Sanctioned::InitialisedToNone,
        Sanctioned::GuardedRead,
        Sanctioned::RuntimeForward,
    ] {
        assert!(
            kinds.contains(&expected),
            "the shipped tree no longer contains a {expected:?} mention of the gate — the \
             classifier is running on a hole. Sanctioned mentions found: {sanctioned:?}"
        );
    }
}

/// ANTI-TAUTOLOGY (4), and the one the others cannot make: the strip removes the TEST
/// scaffolding and nothing else.
///
/// `production_code_of` blanks two whole classes of span. If either over-reached — a
/// `#[cfg(test)]` match that swallowed the rest of the file, a `_for_test` walk that ate
/// its enclosing `impl` — the classifier above would run over an empty string.
#[test]
fn the_shipped_view_keeps_production_code_and_drops_only_the_for_test_seam() {
    let egress = production_code_of(&src_dir().join("egress.rs"));

    for kept in [
        "pub fn new(manager: Arc<TransportManager>)",
        "fn ensure_gateway_booted",
        "fn register_egress",
        // The tail: the last item before `mod tests`, so its presence proves the strip
        // did not truncate the file on its way past the two `_for_test` seams that
        // precede it.
        "fn hash_for_topic",
    ] {
        assert!(
            egress.contains(kept),
            "the shipped view of egress.rs lost `{kept}` — the strip over-reached, and the \
             classifier would be running on a hole"
        );
    }

    // …and the scaffolding really did go: the setter's DEFINITION (a `_for_test` fn,
    // whose body is the one legitimate write) and the `#[cfg(test)] mod tests` block.
    assert!(
        !egress.contains("pub fn set_idle_wait_gate_for_test"),
        "the shipped view of egress.rs still carries the setter's own definition"
    );
    assert!(
        !egress.contains("fn noop_egress_plane_refuses_loudly_and_release_is_a_noop"),
        "the `#[cfg(test)] mod tests` block was not stripped"
    );
    // The doc comment NAMES the seam repeatedly; the literal blanking is what stops
    // that prose from satisfying (or tripping) the classifier.
    assert!(
        !egress.contains("NEVER set in production"),
        "comments were not blanked, so the guard is reading prose rather than code"
    );
}

/// The stripper is oracle-tested rather than trusted: every verdict above is computed
/// over ITS output, so a stripper that dropped the tail would make them all vacuous.
#[test]
fn code_only_strips_comments_and_literals_and_keeps_line_numbers_exact() {
    let s = code_only("keep1 // gone\nkeep2 /* gone */ keep3");
    assert!(s.code.contains("keep1") && s.code.contains("keep2") && s.code.contains("keep3"));
    assert!(!s.code.contains("gone"));
    assert_eq!(s.unclosed_depth, 0);

    // Block comments NEST.
    let s = code_only("a /* one /* two */ still */ b");
    assert!(s.code.contains('a') && s.code.contains('b'));
    assert!(!s.code.contains("still"));

    // A `/*` inside a LINE comment opens nothing.
    let s = code_only("// /* not an opener\nreal_code");
    assert!(s.code.contains("real_code"));
    assert_eq!(s.unclosed_depth, 0);

    // String literals are blanked — including the raw form. This is the arm that
    // matters most here: `egress.rs` names the seam in prose.
    let s = code_only("let x = \"idle_wait_gate\"; let y = r#\"also /* here\"#; z");
    assert!(!s.code.contains("idle_wait_gate"), "{}", s.code);
    assert!(!s.code.contains("also"), "{}", s.code);
    assert!(s.code.contains('z'));
    assert_eq!(
        s.unclosed_depth, 0,
        "a `/*` inside a raw literal must not open a comment"
    );

    // A char literal carrying a lone quote must not swallow the rest of the file.
    let s = code_only("let q = '\"'; marker_after");
    assert!(s.code.contains("marker_after"), "{}", s.code);

    // ... while a LIFETIME survives.
    let s = code_only("fn f<'a>(x: &'a str) -> &'a str { x }");
    assert!(s.code.contains("'a"), "{}", s.code);

    // An unterminated block is REPORTED, not silently swallowed.
    assert_eq!(code_only("a /* unterminated").unclosed_depth, 1);
    assert_eq!(code_only("a /* closed */ b").unclosed_depth, 0);

    // THIS file's own addition over its sibling: newlines survive every blanking arm,
    // so a reported line number is the file's real one. A drifting number sends a
    // reader to the wrong place, which is worse than no number.
    let src = "one\n/* two\nthree\nfour */\nlet s = \"five\nsix\";\nseven";
    let s = code_only(src);
    assert_eq!(
        s.code.matches('\n').count(),
        src.matches('\n').count(),
        "newline count changed:\n{}",
        s.code
    );
    let seven_at = s.code.find("seven").expect("tail survives");
    assert_eq!(
        s.code[..seven_at].matches('\n').count() + 1,
        7,
        "`seven` must still be on line 7"
    );
}

/// The ANGLE-BRACKET rule, oracle-tested directly, because the three shapes it
/// separates are ONE BYTE apart and a fixture can only ever reach them through a whole
/// walk — where a wrong answer is indistinguishable from a walk that stopped for some
/// other reason.
#[test]
fn the_angle_scan_separates_a_generic_close_from_an_arrow_and_a_comparison() {
    // A generic close resolves to its OWN opener, at every nesting depth…
    let s = "|a: Arc<T>, x|";
    assert_eq!(
        matching_angle_open(s, s.rfind('>').expect("close")),
        Some(s.find('<').expect("open"))
    );
    let s = "|a: Arc<Mutex<Option<T>>>, x|";
    assert_eq!(
        matching_angle_open(s, s.rfind('>').expect("close")),
        Some(s.find('<').expect("open"))
    );

    // …and an ARROW inside one is not a second close: `Arc<dyn Fn() -> u8>` opens and
    // closes exactly ONCE, so a counter that miscounted it would look for two openers,
    // find one, and refuse — the under-detection this whole rule exists to remove.
    let s = "|a: Arc<dyn Fn() -> u8>, x|";
    assert_eq!(
        matching_angle_open(s, s.rfind('>').expect("close")),
        Some(s.find('<').expect("open"))
    );

    // A match arm's `=>` and a return type's `->` are not closes at all — which is what
    // keeps this file's match-arm handling exactly as it was.
    for arrow in ["Some(g) => f(g)", "fn f() -> u8 { 0 }"] {
        let at = arrow.find('>').expect("arrow");
        assert!(!is_generic_close(arrow, at), "{arrow}");
        assert_eq!(matching_angle_open(arrow, at), None, "{arrow}");
    }

    // A COMPARISON is refused by the SCAN rather than by the predicate: nothing before
    // it is `-` or `=`, so it LOOKS like a close, and what stops it is running into a
    // statement boundary without finding an opener.
    let s = "{ let flag = budget(n) > limit; }";
    let cmp = s.find("> limit").expect("comparison");
    assert!(
        is_generic_close(s, cmp),
        "the predicate alone cannot tell a comparison from a close"
    );
    assert_eq!(matching_angle_open(s, cmp), None, "bounded by the brace");

    // …including when the expression genuinely carries matching `<`s: those balance,
    // so the scan reaches the statement boundary with the comparison still unmatched.
    let s = "{ let flag = probe::<Vec<u8>>() > limit; }";
    assert_eq!(
        matching_angle_open(s, s.find("> limit").expect("comparison")),
        None
    );

    // And this is what makes the BOUND load-bearing rather than decorative: an
    // UNRELATED `<` in an EARLIER statement is a perfectly good opener as far as a
    // depth counter is concerned. Without the boundary the scan pairs a comparison in
    // one statement with a comparison in another and hands the caller's walk a
    // position it has no business at — over-detection, i.e. a guard that starts
    // refusing the production forward.
    let s = "{ let ready = state < LIMIT; let flag = budget(n) > gate; }";
    assert_eq!(
        matching_angle_open(s, s.find("> gate").expect("comparison")),
        None,
        "the scan must stop at the statement boundary, not pair across it"
    );

    // Fails CLOSED: a caller that did not pass a `>`, and a `>` with no opener at all.
    assert_eq!(matching_angle_open("abc", 1), None);
    assert_eq!(matching_angle_open("T>", 1), None);
}

/// The two SPAN strips and the CLASSIFIER, oracle-tested on hand-written input for the
/// same reason: against the real (passing) tree their over- and under-reach are both
/// invisible.
#[test]
fn the_span_strips_and_the_classifier_accept_exactly_the_sanctioned_shapes() {
    let tmp = std::env::temp_dir().join(format!(
        "gate_oracle_{}_{}.rs",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("clock")
            .as_nanos()
    ));
    // Every sanctioned shape, plus the scaffolding that must vanish, plus a tail marker.
    let src = r#"
pub struct P {
    idle_wait_gate: Mutex<Option<Arc<dyn Fn() + Send + Sync>>>,
}
fn build() -> P { P { idle_wait_gate: Mutex::new(None) } }
fn boot(&self) {
    if let Some(gate) = self
        .idle_wait_gate
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clone()
    {
        gateway.set_idle_wait_gate_for_test(gate);
    }
}
pub fn set_idle_wait_gate_for_test(&self, gate: Arc<dyn Fn()>) {
    *self.idle_wait_gate.lock().unwrap() = Some(gate);
}
fn tail_marker() {}
#[cfg(test)]
mod tests {
    fn inner() { plane.set_idle_wait_gate_for_test(hook()); }
}
pub fn after_tests() {}
"#;
    std::fs::write(&tmp, src).expect("write oracle fixture");
    let out = production_code_of(&tmp);
    let verdicts: Vec<Option<Sanctioned>> =
        mentions_in(&tmp).into_iter().map(|m| m.verdict).collect();
    std::fs::remove_file(&tmp).ok();

    // A `_for_test` fn and a `#[cfg(test)] mod` are both gone, bodies included.
    assert!(!out.contains("pub fn set_idle_wait_gate_for_test"), "{out}");
    assert!(!out.contains("mod tests"), "{out}");
    assert!(!out.contains("fn inner"), "{out}");

    // Everything on either side survives — the UNDER-reach half. `after_tests` is the
    // one that catches a `#[cfg(test)]` strip that ran to EOF.
    assert!(out.contains("fn build"), "{out}");
    assert!(out.contains("fn tail_marker"), "{out}");
    assert!(out.contains("fn after_tests"), "{out}");

    // The four surviving mentions are exactly the four sanctioned shapes, in order.
    assert_eq!(
        verdicts,
        vec![
            Some(Sanctioned::FieldDeclaration),
            Some(Sanctioned::InitialisedToNone),
            Some(Sanctioned::GuardedRead),
            Some(Sanctioned::RuntimeForward),
        ],
        "the classifier disagreed with the hand oracle"
    );

    // And the shapes an INSTALL takes are refused. Each is written the way a real
    // regression would arrive.
    for install in [
        // A plane-level install from a production path (the `main.rs` shape).
        "fn boot() { plane.set_idle_wait_gate_for_test(hook()); }",
        // The same, on `self` inside the plane's own constructor.
        "fn new() { self.set_idle_wait_gate_for_test(hook()); }",
        // A bare call with no receiver at all.
        "fn boot() { set_idle_wait_gate_for_test(hook()); }",
        // A direct write to the field, initialised NON-empty at construction.
        "fn build() -> P { P { idle_wait_gate: Mutex::new(Some(hook())) } }",
        // A future spelling built out of the needle that nobody has adjudicated.
        "fn boot() { self.idle_wait_gate_override = Some(hook()); }",
    ] {
        let f = tmp.with_file_name(format!(
            "install_{}.rs",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("clock")
                .as_nanos()
        ));
        std::fs::write(&f, install).expect("write install fixture");
        let v: Vec<Option<Sanctioned>> = mentions_in(&f).into_iter().map(|m| m.verdict).collect();
        std::fs::remove_file(&f).ok();
        assert!(
            v.iter().any(|x| x.is_none()),
            "an INSTALL was classified as sanctioned — `{install}` produced {v:?}"
        );
    }
}

// ────────────────────────────────────────────────────────────────────────────
// The three shapes a production install could wear while satisfying the FIRST
// version of this guard, which classified by SURFACE SYNTAX alone.
//
// Each is written the way a real regression would arrive, and each is PAIRED with a
// positive control over the REAL tree — because a rule tightened until nothing
// passes is not a tighter rule, it is a broken one, and the sanctioned shapes are
// exactly what these three fixtures resemble.
// ────────────────────────────────────────────────────────────────────────────

/// Write `src` to a scratch `.rs`, run the walk over it, and delete it.
fn with_fixture<T>(tag: &str, src: &str, f: impl FnOnce(&Path) -> T) -> T {
    let path = std::env::temp_dir().join(format!(
        "{tag}_{}_{}.rs",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("clock")
            .as_nanos()
    ));
    std::fs::write(&path, src).expect("write fixture");
    let out = f(&path);
    std::fs::remove_file(&path).ok();
    out
}

/// The needle verdicts of one fixture's shipped view.
fn verdicts_of(tag: &str, src: &str) -> Vec<Option<Sanctioned>> {
    with_fixture(tag, src, |p| {
        mentions_in(p).into_iter().map(|m| m.verdict).collect()
    })
}

/// The violations the WALK's own adjudicator reports for one fixture — the same
/// decision `no_production_path_installs_an_idle_wait_gate` acts on.
fn violations_of(tag: &str, src: &str) -> Vec<String> {
    with_fixture(tag, src, |p| {
        adjudicate(&[(p.to_path_buf(), shipped_view_of(p))]).0
    })
}

/// **F1**: a seam whose NAME carries no needle and whose BODY the strip hid.
///
/// `install_gate_for_test` writes the gate; `boot` calls it. The strip blanks the
/// seam's signature AND body, so the shipped view mentions the gate only in its own
/// field declaration — a sanctioned shape. Classifying by the needle alone therefore
/// sees a clean file while a production path installs a hook on every boot.
#[test]
fn f1_a_shipped_caller_of_a_hidden_test_seam_is_refused() {
    let src = r#"
pub struct P {
    idle_wait_gate: Mutex<Option<Arc<dyn Fn() + Send + Sync>>>,
}
fn install_gate_for_test(&self, hook: Arc<dyn Fn() + Send + Sync>) {
    *self.idle_wait_gate.lock().unwrap() = Some(hook);
}
fn boot(&self) {
    self.install_gate_for_test(window_hook());
}
"#;
    // The needle classifier alone is CLEAN — which is the hole, stated as an
    // assertion so the fixture cannot be mistaken for one the old guard caught.
    assert_eq!(
        verdicts_of("f1_needles", src),
        vec![Some(Sanctioned::FieldDeclaration)],
        "the needle classifier is structurally blind to this shape"
    );
    // …and the WALK refuses it, naming the seam.
    let violations = violations_of("f1_walk", src);
    assert_eq!(
        violations.len(),
        1,
        "expected exactly one, got {violations:?}"
    );
    assert!(
        violations[0].contains("CALLS the test seam `install_gate_for_test`"),
        "the walk must name the seam it refused, got:\n{}",
        violations[0]
    );

    // POSITIVE CONTROL, over the REAL tree: the sanctioned boot forward IS a shipped
    // call to a `_for_test` name, and must NOT be swept up by this rule (the
    // classifier adjudicates it instead). A blanket refusal would fail here.
    let views: Vec<(PathBuf, Shipped)> = src_files()
        .iter()
        .map(|f| (f.clone(), shipped_view_of(f)))
        .collect();
    let names: BTreeSet<String> = views
        .iter()
        .flat_map(|(_, s)| s.for_test_names.iter().cloned())
        .collect();
    assert!(
        names.contains("set_idle_wait_gate_for_test"),
        "the real tree must define the seam this control is about; it has {names:?}"
    );
    let (real_violations, real_sanctioned) = adjudicate(&views);
    assert!(
        real_violations.is_empty(),
        "the real tree must be clean under this rule, got {real_violations:?}"
    );
    assert!(
        real_sanctioned
            .iter()
            .any(|(_, k)| *k == Sanctioned::RuntimeForward),
        "the sanctioned forward must still classify — otherwise this control passes \
         because the tree stopped forwarding at all"
    );
}

/// **F2**: a WRITE through the guard, which reads character-for-character like the
/// sanctioned lock-read until the `=`.
///
/// Two spellings, because they are refused by two different rules and each must be
/// independently killable: an ASSIGNMENT through the deref, and a MUTATING method on
/// the guard (which carries no `=` at all).
#[test]
fn f2_a_write_through_the_lock_guard_is_not_a_read() {
    let assign = r#"
fn boot(&self) {
    *self.idle_wait_gate.lock().unwrap_or_else(PoisonError::into_inner) = Some(gate);
}
"#;
    assert_eq!(
        verdicts_of("f2_assign", assign),
        vec![None],
        "an assignment THROUGH the lock guard is an install, not a read"
    );

    let mutate = r#"
fn boot(&self) {
    self.idle_wait_gate.lock().unwrap().replace(gate);
}
"#;
    assert_eq!(
        verdicts_of("f2_mutate", mutate),
        vec![None],
        "a mutating method on the lock guard is an install, not a read"
    );

    // POSITIVE CONTROL: the shape `ensure_gateway_booted` actually uses.
    let read = r#"
fn boot(&self) {
    if let Some(gate) = self
        .idle_wait_gate
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clone()
    {
        gateway.set_idle_wait_gate_for_test(gate);
    }
}
"#;
    assert_eq!(
        verdicts_of("f2_control", read),
        vec![
            Some(Sanctioned::GuardedRead),
            Some(Sanctioned::RuntimeForward)
        ],
        "the production lock-READ must still classify"
    );
}

/// **F3**: a NEW install wearing the sanctioned receiver.
///
/// `gateway` is an ordinary variable name, so the receiver alone decides nothing —
/// what makes the shipped forward a no-op is that it hands on the value just CLONED
/// out of the field. Two spellings: a constructor inline, and a bare identifier bound
/// from somewhere else entirely.
#[test]
fn f3_a_forward_that_hands_on_a_new_hook_is_refused() {
    let constructed = r#"
fn boot(&self) {
    let mut gateway = GatewayRuntime::new_embedded()?;
    gateway.set_idle_wait_gate_for_test(Arc::new(|| {}));
}
"#;
    assert_eq!(
        verdicts_of("f3_constructed", constructed),
        vec![None],
        "a forward handed a freshly constructed hook is an install"
    );

    let elsewhere = r#"
fn boot(&self) {
    let mut gateway = GatewayRuntime::new_embedded()?;
    let hook = window_hook();
    gateway.set_idle_wait_gate_for_test(hook);
}
"#;
    assert_eq!(
        verdicts_of("f3_elsewhere", elsewhere),
        vec![None],
        "a forward handed a hook bound anywhere but the field is an install"
    );

    // A binding of the RIGHT name in a DIFFERENT function must not vouch for it.
    let other_fn = r#"
fn read_it(&self) {
    if let Some(gate) = self.idle_wait_gate.lock().unwrap_or_else(f).clone() { drop(gate); }
}
fn boot(&self) {
    let gate = window_hook();
    gateway.set_idle_wait_gate_for_test(gate);
}
"#;
    assert_eq!(
        verdicts_of("f3_other_fn", other_fn),
        vec![Some(Sanctioned::GuardedRead), None],
        "a field binding in another function cannot vouch for this forward"
    );

    // A MATCH ARM over a FOREIGN scrutinee reaches the same `Some(gate)` … `=` shape
    // (`=>` starts with `=`) and its body supplies a field read, so a substring rule
    // reads `gate` as the field's binding although it came from `source`.
    let match_arm = r#"
fn boot(&self) {
    match source {
        Some(gate) => {
            let _ = self
                .idle_wait_gate
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .clone();
            gateway.set_idle_wait_gate_for_test(gate);
        }
        None => {}
    }
}
"#;
    assert_eq!(
        verdicts_of("f3_match_arm", match_arm),
        vec![Some(Sanctioned::GuardedRead), None],
        "a match-arm binding over a foreign scrutinee is not the field's binding"
    );

    // SHADOWING: the field really was read and bound here, and then the name was
    // taken over by something else before the forward.
    let shadowed = r#"
fn boot(&self) {
    if let Some(gate) = self
        .idle_wait_gate
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clone()
    {
        let gate = window_hook();
        gateway.set_idle_wait_gate_for_test(gate);
    }
}
"#;
    assert_eq!(
        verdicts_of("f3_shadowed", shadowed),
        vec![Some(Sanctioned::GuardedRead), None],
        "a re-bound `gate` is no longer the value cloned out of the field"
    );

    // A binding off ANOTHER field, in the sanctioned SHAPE. Only the right-hand side
    // separates this from the plumbing, so it is what isolates that conjunct.
    let other_field = r#"
fn boot(&self) {
    if let Some(gate) = self
        .some_other_slot
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clone()
    {
        gateway.set_idle_wait_gate_for_test(gate);
    }
}
"#;
    assert_eq!(
        verdicts_of("f3_other_field", other_field),
        vec![None],
        "a hook cloned out of a DIFFERENT field is not the gate's own binding"
    );

    // A SIBLING block: the field really was read, in the sanctioned shape, in this
    // very function — and the forward is outside the block that bound it.
    let sibling_block = r#"
fn boot(&self) {
    if let Some(gate) = self
        .idle_wait_gate
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clone()
    {
        drop(gate);
    }
    gateway.set_idle_wait_gate_for_test(gate);
}
"#;
    assert_eq!(
        verdicts_of("f3_sibling_block", sibling_block),
        vec![Some(Sanctioned::GuardedRead), None],
        "a binding whose block does not ENCLOSE the forward cannot vouch for it"
    );

    // POSITIVE CONTROL: the production shape, receiver AND argument.
    let sanctioned = r#"
fn ensure_gateway_booted(&self) {
    let mut gateway = GatewayRuntime::new_embedded()?;
    if let Some(gate) = self
        .idle_wait_gate
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clone()
    {
        gateway.set_idle_wait_gate_for_test(gate);
    }
}
"#;
    assert_eq!(
        verdicts_of("f3_control", sanctioned),
        vec![
            Some(Sanctioned::GuardedRead),
            Some(Sanctioned::RuntimeForward)
        ],
        "the production forward must still classify"
    );
}

/// The sanctioned shape with `body` spliced INSIDE the block that bound the field's
/// value — the one window in which a re-binding can break the chain the forward's
/// verdict rests on.
fn inside_the_bound_block(body: &str) -> String {
    format!(
        r#"
fn ensure_gateway_booted(&self) {{
    let mut gateway = GatewayRuntime::new_embedded()?;
    if let Some(gate) = self
        .idle_wait_gate
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clone()
    {{
{body}
    }}
}}
"#
    )
}

/// **F4**: the OTHER ways Rust takes a name over.
///
/// [`rebound_between`] is what stops a shadowed name from inheriting the field
/// binding's verdict, and it knew only `let x`, `let mut x` and a `let`-scoped
/// `Some(x)`. Every other binding form was therefore a shape in which shipped code
/// could forward a freshly built hook while this walk called it the field's own value —
/// silently, since a missed re-bind ACCEPTS rather than refuses.
///
/// Each fixture re-binds `gate` inside the block that bound it and forwards the new
/// value, which is how the refactor that introduces one actually looks.
#[test]
fn f4_a_forward_whose_argument_was_re_bound_by_any_pattern_form_is_refused() {
    for (tag, body) in [
        // A TUPLE `let`: the name is not adjacent to the keyword, so the
        // `ends_with_keyword(.., "let")` probe never sees it.
        (
            "tuple",
            "        let (gate, _at) = (window_hook(), Instant::now());\n\
             \x20       gateway.set_idle_wait_gate_for_test(gate);",
        ),
        // A STRUCT pattern — the same, through braces and a field name.
        (
            "struct",
            "        let Wrapper { gate, .. } = rebuild();\n\
             \x20       gateway.set_idle_wait_gate_for_test(gate);",
        ),
        // A FOR-loop binding: the forward runs once per hook in a fresh list.
        (
            "for_loop",
            "        for gate in candidate_hooks() {\n\
             \x20           gateway.set_idle_wait_gate_for_test(gate);\n\
             \x20       }",
        ),
        // A CLOSURE PARAMETER — the "extract a helper" refactor, which is what makes
        // this the most likely of the four to arrive by accident.
        (
            "closure_param",
            "        let mut install = |gate: IdleWaitGate| gateway.set_idle_wait_gate_for_test(gate);\n\
             \x20       install(window_hook());",
        ),
        // The SAME refactor, with an EARLIER parameter carrying a GENERIC type. The
        // backward walk that finds a closure parameter reads over what a parameter list
        // can hold — names, `,`, `:`, `&` — and a `>` was a stop byte, so every
        // parameter AFTER a generically-typed one was invisible to it and the re-bind
        // went unseen. `Arc<TransportManager>` is not a contrived type here: it is what
        // the plane holds, so an extracted helper that takes it is the shape this
        // refactor actually arrives in.
        (
            "closure_param_after_generic",
            "        let mut install = |plane: Arc<TransportManager>, gate: IdleWaitGate| {\n\
             \x20           let _ = plane;\n\
             \x20           gateway.set_idle_wait_gate_for_test(gate);\n\
             \x20       };\n\
             \x20       install(Arc::clone(&self.manager), window_hook());",
        ),
        // The same hole in the PATTERN walk rather than the closure one: a refutable
        // binding whose own path carries a turbofish. This is the arm that would
        // otherwise ship unattributed — the closure fixtures above are killed by
        // `is_closure_parameter`'s arm alone, because the pattern walk reads over `|`
        // and reaches the `=` of the `let` before it can rule on a closure parameter.
        (
            "let_pattern_with_a_turbofish_path",
            "        if let Some::<Arc<u8>>(gate) = rebuild() {\n\
             \x20           gateway.set_idle_wait_gate_for_test(gate);\n\
             \x20       }",
        ),
        // The same hole one nesting level deeper, and with an ARROW inside the generic
        // list. `Arc<dyn Fn() -> u8>` closes ONCE, so a depth counter that treats the
        // `>` of `->` as a close looks for two openers, finds one, and refuses — which
        // is the under-detection this fixture exists to forbid.
        (
            "closure_param_after_nested_generic",
            "        let mut install = |make: Arc<Mutex<Option<Arc<dyn Fn() -> u8>>>>, gate: IdleWaitGate| {\n\
             \x20           let _ = make;\n\
             \x20           gateway.set_idle_wait_gate_for_test(gate);\n\
             \x20       };\n\
             \x20       install(factory(), window_hook());",
        ),
        // A MATCH ARM over a foreign scrutinee, INSIDE the bound block (another
        // fixture's arm is the whole function, so the block condition refuses it there
        // and this rule is never reached).
        //
        // The scrutinee is a METHOD CALL deliberately: the `.` stops the backward
        // pattern walk before it reaches the `match`, so this arm is caught by the
        // FORWARD read to `=>` and by nothing else. Written the other way (a bare
        // `rebuild()`) the walk reaches the keyword and the two rules overlap, which
        // MEASURED left `is_match_arm_binding` deletable with the whole suite green.
        (
            "match_arm",
            "        match self.pending.take() {\n\
             \x20           Some(gate) => gateway.set_idle_wait_gate_for_test(gate),\n\
             \x20           None => {}\n\
             \x20       }",
        ),
    ] {
        assert_eq!(
            verdicts_of(&format!("f4_{tag}"), &inside_the_bound_block(body)),
            vec![Some(Sanctioned::GuardedRead), None],
            "a `{tag}` re-binding of the forwarded name must break the chain"
        );
    }

    // ANTI-TAUTOLOGY, and the half that keeps the widened rule a RULE rather than a
    // blanket refusal: the same five constructs, with `gate` merely USED inside them.
    // A detector that answered "bound" to any of these would refuse the production
    // forward the moment anybody logged the hook on the way past.
    //
    // The match SCRUTINEE is the one that bit: it reaches the `match` keyword through
    // the backward walk exactly as an arm pattern does, so without the crossed-brace
    // condition this control fails with the forward classified `None`.
    //
    // The last three lines are the ANGLE-BRACKET half, and they are the reason `>` was
    // a stop byte in the first place. Each puts a `>` exactly where a backward walk
    // LANDS on it while `gate` is merely used:
    //
    // * a COMPARISON against a TURBOFISH result — the adversarial one, because the
    //   expression really does carry matching `<`s, so only the scan's statement bound
    //   stops it walking back to the `let` and reporting a USE as a binding;
    // * a bare COMPARISON, where there is no `<` to find at all;
    // * a MATCH ARM whose body STARTS with the name, so the walk's first byte is the
    //   `>` of the `=>` — the shape this file's match-arm handling turns on.
    let uses = "        for attempt in 0..RETRIES {\n\
        \x20           note(attempt, &gate);\n\
        \x20       }\n\
        \x20       match kind_of(&gate) {\n\
        \x20           Kind::Slow => note_slow(&gate),\n\
        \x20           other => note_other(other),\n\
        \x20       }\n\
        \x20       let announce = move || note_installed(&gate);\n\
        \x20       announce();\n\
        \x20       let _turbofish = probe::<Vec<u8>>() > gate;\n\
        \x20       let _plain = budget(RETRIES) > gate;\n\
        \x20       match tier_of(&gate) {\n\
        \x20           Tier::Fast => gate.record(),\n\
        \x20           _ => {}\n\
        \x20       }\n\
        \x20       gateway.set_idle_wait_gate_for_test(gate);";
    assert_eq!(
        verdicts_of("f4_uses_control", &inside_the_bound_block(uses)),
        vec![
            Some(Sanctioned::GuardedRead),
            Some(Sanctioned::RuntimeForward)
        ],
        "a name merely READ in a loop body, a match scrutinee, an arm body or a closure \
         body is not re-bound, and the production forward must still classify"
    );
}
