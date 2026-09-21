// SPDX-License-Identifier: AGPL-3.0-only
//! STRUCTURAL adoption guards for `cerulion-vizd`'s control handlers — the shapes
//! whose regression is invisible to a hermetic e2e because the harness answers
//! instantly or because the seam needs a live LAN.
//!
//! `cerulion-vizd` **answers promptly** — no
//! discovery wait ever creeps back into a control handler — and each seam that
//! CAN say "not found YET" actually says it.
//!
//! The layout half: every ATTACHED-topic reporter reads the LIVE layout
//! signals, so none of them can report a placement the installed blueprint does
//! not have — see the section at the bottom of this file for why one of the four
//! is unreachable from any hermetic arm.
//!
//! # Why a source walk, when the behaviour is pinned e2e
//!
//! `vizd_e2e_test.rs` proves the handlers gather exactly once and emit the right hints.
//! What it cannot reach is the SHAPE that makes a handler-side wait a BLOCKER: a
//! wait inside a handler is invisible to a test that injects a scripted `DemandPlane`,
//! because such a plane answers instantly — the cost only appears against a real
//! `cerulion-netd`, on an operator's LAN, through `cerulion_cli_engine::viz_client`'s
//! 5 s `SO_RCVTIMEO`. That combination is unreachable from CI, which is the classic
//! inert-shipping shape.
//!
//! The specific regression: `NetdClient::query_{catalog,schema}_converged` run a
//! multi-second wait behind `&mut self`. Inside `NetdDemandPlane::with_client` they
//! would hold vizd's ONE netd client mutex for the whole ceiling AND push the reply
//! past the client's read deadline, so `cerulion viz` would abort on an errno instead
//! of being a few seconds slower. Neither verb may appear anywhere in vizd's `src/`.
//!
//! # The walk is a DIRECTORY walk, deliberately
//!
//! A guard that iterates a hand-written file list while its
//! docs claim it covers `src/` reproduces the failure mode the guard's own docs
//! already record — "a list reproduces its own failure mode, where the sweep missed
//! the then-shipping `rerun_sink` because nothing enumerated hand-written inits" — and
//! it has teeth here: `NetdDemandPlane` is ~200 lines inside a 9,000-line `daemon.rs`,
//! and a routine refactor moving it to `src/plane.rs` would silently take it out of
//! scope of every assertion below. So the forbidden-verb scan reads `src/**/*.rs`, and
//! the file-scoped guards LOCATE their target by content rather than by filename.
//!
//! The comment/literal stripper is a deliberate copy of the CLI guard's (a `tests/`
//! binary cannot import another crate's test helpers) and is oracle-tested here rather
//! than trusted.
//!
//! It lives in `tests/` rather than in the module so its own needle literals are not
//! inside the text it searches.

use std::path::{Path, PathBuf};

/// vizd's `src/` directory.
fn src_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src")
}

/// Every `*.rs` file under `src/`, RECURSIVELY — never a hand-written list.
///
/// Fails LOUDLY on a walk that yields almost nothing: a silently-empty walk would make
/// every negative assertion below vacuous.
fn src_files() -> Vec<PathBuf> {
    let mut out = Vec::new();
    walk(&src_dir(), &mut out);
    out.sort();
    assert!(
        out.len() >= 4,
        "the src walk found only {} file(s) under {} — a walk that yields nothing \
         makes every `!contains` assertion in this file vacuous",
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

/// A view of `src` with comments AND string/char literals blanked.
///
/// Both halves are load-bearing here:
///
/// * COMMENTS, because this module's docs NAME the forbidden verbs in order to explain
///   why they are not called — a naive search would be satisfied by prose. So do vizd's
///   own docs.
/// * STRING LITERALS, because a lone `"` inside a char literal would otherwise open the
///   string arm and swallow everything to the next quote anywhere in the file, leaving
///   `unclosed_depth == 0` so the loud guard could not see it. Only the SHORT char
///   shapes are consumed, so a lifetime survives.
fn code_only(src: &str) -> Stripped {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
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
                    out.push(' ');
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
                    out.push(' ');
                    i = k.min(bytes.len());
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
                out.push(' ');
                i = k.min(bytes.len());
                continue;
            }
            if bytes[i..].starts_with(b"//") {
                while i < bytes.len() && bytes[i] != b'\n' {
                    i += 1;
                }
                continue;
            }
        }
        if bytes[i..].starts_with(b"/*") {
            depth += 1;
            i += 2;
            continue;
        }
        if depth > 0 && bytes[i..].starts_with(b"*/") {
            depth -= 1;
            i += 2;
            continue;
        }
        if depth == 0 {
            let ch = src[i..].chars().next().expect("valid utf-8 boundary");
            out.push(ch);
            i += ch.len_utf8();
        } else {
            i += 1;
        }
    }
    Stripped {
        code: out,
        unclosed_depth: depth,
    }
}

/// The code-only view of one file, failing LOUDLY on an unbalanced scan.
fn code_of(path: &Path) -> String {
    let s = code_only(&read(path));
    assert_eq!(
        s.unclosed_depth,
        0,
        "{}: the stripper ended inside {} unclosed block comment(s), so the tail was \
         dropped and every assertion over it would run on a PREFIX",
        path.display(),
        s.unclosed_depth
    );
    s.code
}

/// The code-only view of the ONE `src/` file containing `needle`, found by walking.
///
/// Locating by CONTENT rather than by filename is the point: a guard that hard-codes
/// `daemon.rs` stops guarding the moment the code it names moves to a new module, which
/// is exactly how a negative guard rots into a vacuous one.
fn code_containing(needle: &str) -> String {
    let mut hits: Vec<(PathBuf, String)> = Vec::new();
    for f in src_files() {
        let code = code_of(&f);
        if code.contains(needle) {
            hits.push((f, code));
        }
    }
    assert_eq!(
        hits.len(),
        1,
        "expected exactly ONE src file defining `{needle}`, found {:?} — if it moved, \
         this guard follows it automatically; if it was DUPLICATED, that is the bug",
        hits.iter()
            .map(|(p, _)| p.display().to_string())
            .collect::<Vec<_>>()
    );
    hits.pop().expect("one hit").1
}

/// The body of `fn <name>(` in `src`, brace-matched from its opening `{`.
///
/// Assertions must be FUNCTION-SCOPED: "the file contains `loop`" is true of any
/// 9,000-line module and says nothing about the handler under guard.
fn fn_body<'a>(src: &'a str, signature: &str) -> &'a str {
    let at = src
        .find(signature)
        .unwrap_or_else(|| panic!("`{signature}` not found in the stripped source"));
    let open = src[at..]
        .find('{')
        .unwrap_or_else(|| panic!("no opening brace after `{signature}`"))
        + at;
    let mut depth = 0usize;
    for (off, ch) in src[open..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return &src[open..open + off + 1];
                }
            }
            _ => {}
        }
    }
    panic!("unbalanced braces after `{signature}`");
}

/// The brace-matched block starting at `open` (which must index a `{`), inclusive.
fn block_at(src: &str, open: usize) -> Option<&str> {
    let mut depth = 0usize;
    for (off, ch) in src[open..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(&src[open..open + off + 1]);
                }
            }
            _ => {}
        }
    }
    None
}

/// Is the byte at `at` the start of a TOKEN rather than the tail of an identifier?
/// Without this, `motif ` contributes a phantom `if `.
fn token_start(src: &str, at: usize) -> bool {
    src[..at]
        .chars()
        .next_back()
        .is_none_or(|c| !c.is_alphanumeric() && c != '_')
}

/// Every offset in `body` at which `ident` is the receiver of a `.store(` call.
///
/// A PRESENCE check on the identifier is not a check on the publication, and the
/// gap is exactly the failure that matters: a body that merely MENTIONS
/// `monitor_rows_ineligible` inside the lock — a comment stripped away, a
/// borrow, a debug read — while the `.store(` sits outside satisfies
/// `scope.contains(ident)` and leaves the race untouched.
///
/// Whitespace between the receiver and the `.` is skipped because rustfmt puts
/// the `.` of a method chain on the NEXT line; a needle spelled `ident.store(`
/// could not match the shape rustfmt actually emits (the lesson already recorded
/// on the `monitor_samples_taken` survivor below).
fn store_sites(body: &str, ident: &str) -> Vec<usize> {
    let mut out = Vec::new();
    let mut from = 0usize;
    while let Some(rel) = body[from..].find(ident) {
        let at = from + rel;
        from = at + ident.len();
        if !token_start(body, at) {
            continue;
        }
        let after = body[from..].trim_start();
        if after.starts_with(".store(") {
            out.push(at);
        }
    }
    out
}

/// The `let outcome = { … }` LOCK SCOPE of a monitor pass, brace-matched.
///
/// Returned as `(scope, start, end)` so a caller can ask both "is this inside the
/// scope?" and "is this after it?".
///
/// Brace-matched rather than found by the first textual `};`, which is the needle
/// a naive version would use. That is the same class as the `monitor_samples_taken.`
/// survivor recorded below: a closure, a struct literal or a nested block ending in
/// `};` anywhere before the folds silently moves the boundary, and every assertion
/// keyed on it then answers about the wrong region — passing whatever it was
/// written to catch.
fn lock_scope<'a>(body: &'a str, of: &str) -> (&'a str, usize, usize) {
    let head = "let outcome = {";
    let at = body
        .find(head)
        .unwrap_or_else(|| panic!("`{of}` must bind its pass under one lock scope:\n{body}"));
    let open = at + head.len() - 1;
    let block = block_at(body, open)
        .unwrap_or_else(|| panic!("`{of}`'s lock scope has unbalanced braces:\n{body}"));
    (block, open, open + block.len())
}

/// The PAREN-MATCHED invocation of `macro_name!` inside `scope`, `!` and argument
/// list included.
///
/// A per-item logging claim has to be scoped to ONE CALL, not to the loop body.
/// Asserting `body.contains("warn!")` and `body.contains("robot =")` separately is
/// satisfied by `warn!("something was unusable"); debug!(robot = %r, "…")` — two
/// calls, neither of which is the line the guard describes — and that is not a
/// contrived shape: it is what a refactor splitting a noisy line in two produces.
/// Reading the invocation's own span makes the two claims one claim.
///
/// Paren-matched rather than read to the next `;`, because a `tracing` call's
/// arguments routinely contain both.
fn macro_invocation<'a>(scope: &'a str, macro_name: &str) -> &'a str {
    let needle = format!("{macro_name}!");
    let mut from = 0usize;
    while let Some(rel) = scope[from..].find(&needle) {
        let at = from + rel;
        from = at + needle.len();
        if !token_start(scope, at) {
            continue;
        }
        let Some(open_rel) = scope[at..].find('(') else {
            break;
        };
        let open = at + open_rel;
        let mut depth = 0usize;
        for (off, ch) in scope[open..].char_indices() {
            match ch {
                '(' => depth += 1,
                ')' => {
                    depth -= 1;
                    if depth == 0 {
                        return &scope[at..open + off + 1];
                    }
                }
                _ => {}
            }
        }
        panic!("unbalanced parens after `{needle}`");
    }
    panic!("no `{needle}` invocation found in:\n{scope}")
}

/// The brace-matched BODY of the `for` loop that iterates `collection`.
///
/// Scoping to the loop is what makes a per-ITEM claim checkable. "The function
/// mentions `warn!` somewhere after the collection" is true of any handler that
/// logs anything at all later on — including the netd-unreachable `warn!` in this
/// very function — so it passes a handler that dropped the per-item line entirely,
/// which is exactly the regression.
///
/// Matched on `for … in <collection>` rather than on the collection alone, so a
/// mention in a comment (stripped anyway) or a struct-literal field of the same
/// name cannot be read as a loop.
fn loop_body_over<'a>(body: &'a str, collection: &str) -> &'a str {
    let mut from = 0usize;
    while let Some(rel) = body[from..].find("for ") {
        let at = from + rel;
        from = at + 4;
        if !token_start(body, at) {
            continue;
        }
        let Some(open_rel) = body[at..].find('{') else {
            break;
        };
        let open = at + open_rel;
        // The loop HEADER — everything between `for` and its opening brace — must
        // name the collection, or this is some other loop.
        if !body[at..open].contains(collection) {
            continue;
        }
        return block_at(body, open)
            .unwrap_or_else(|| panic!("the `for … in {collection}` body has unbalanced braces"));
    }
    panic!("no `for … in {collection}` loop found in:\n{body}")
}

/// The CONDITION of the innermost `if` whose block contains `needle`.
///
/// The point is structure rather than text order: "the constant appears earlier in
/// the function than the call" is satisfied by an UNCONDITIONAL call preceded by any
/// mention of the constant, which is exactly the regression an interval guard exists
/// to catch. A condition can only be returned here if the call really is inside the
/// branch it guards.
fn governing_condition<'a>(body: &'a str, needle: &str) -> &'a str {
    let target = body
        .find(needle)
        .unwrap_or_else(|| panic!("`{needle}` not found in:\n{body}"));
    let mut best: Option<(&str, usize)> = None;
    let mut search = 0usize;
    while let Some(rel) = body[search..].find("if ") {
        let at = search + rel;
        search = at + 3;
        if !token_start(body, at) {
            continue;
        }
        let Some(open_rel) = body[at..].find('{') else {
            break;
        };
        let open = at + open_rel;
        let Some(block) = block_at(body, open) else {
            continue;
        };
        if open < target && target < open + block.len() {
            // Innermost = the smallest containing block.
            if best.is_none_or(|(_, len)| block.len() < len) {
                best = Some((body[at + 2..open].trim(), block.len()));
            }
        }
    }
    match best {
        Some((cond, _)) => cond,
        None => panic!("`{needle}` is not inside any `if` block:\n{body}"),
    }
}

/// One level of `let <name> = …;` resolution, so a condition held in a binding is
/// checked against what the binding actually computes.
///
/// An optional `: Type` annotation is SKIPPED rather than unmatched. `let robot:
/// Option<String> = …` is an ordinary spelling of the same binding, and a reader that
/// understood only the bare form would answer "not found" on correct code — which,
/// for a caller that then treats the name as its own value, is a guard that fails
/// where it should pass and passes where it should fail.
///
/// The name must END at the match too, or `let robot` resolves `let robotics`.
fn resolve_binding<'a>(body: &'a str, expr: &'a str) -> &'a str {
    if !expr.chars().all(|c| c.is_alphanumeric() || c == '_') {
        return expr; // already an expression
    }
    let head = format!("let {expr}");
    let mut from = 0usize;
    let rhs = loop {
        let Some(rel) = body[from..].find(&head) else {
            return expr;
        };
        let at = from + rel;
        from = at + head.len();
        let after = &body[at + head.len()..];
        if !token_start(body, at)
            || after
                .chars()
                .next()
                .is_some_and(|c| c.is_alphanumeric() || c == '_')
        {
            continue;
        }
        if let Some(eq) = init_eq(after) {
            break &after[eq + 1..];
        }
    };
    let mut depth = 0usize;
    for (off, ch) in rhs.char_indices() {
        match ch {
            '{' | '(' | '[' => depth += 1,
            '}' | ')' | ']' => depth = depth.saturating_sub(1),
            ';' if depth == 0 => return rhs[..off].trim(),
            _ => {}
        }
    }
    rhs.trim()
}

/// The offset of the INITIALISING `=` in what follows a `let <name>` — directly, or
/// past a `: Type` annotation — or `None` for a declaration with no initialiser.
///
/// Depth-tracked over `<>`/`()`/`[]` so a generic argument cannot end the scan, and
/// `==` / `=>` / `>=` / `<=` / `!=` are not initialisers.
fn init_eq(after: &str) -> Option<usize> {
    let b = after.as_bytes();
    let mut depth = 0usize;
    for (i, &c) in b.iter().enumerate() {
        match c {
            b'<' | b'(' | b'[' => depth += 1,
            b'>' | b')' | b']' => depth = depth.saturating_sub(1),
            b';' if depth == 0 => return None, // `let x;`
            b'=' if depth == 0 => {
                let prev = i.checked_sub(1).map(|p| b[p]);
                if b.get(i + 1) == Some(&b'=')
                    || matches!(prev, Some(b'=' | b'!' | b'<' | b'>' | b'+' | b'-'))
                {
                    continue;
                }
                return Some(i);
            }
            _ => {}
        }
    }
    None
}

/// Is `expr` EXACTLY a call to `name` — `a.b.name(..)`, `Type::name(..)`, `name(..)`
/// — with nothing before it and nothing after it?
///
/// Both ends are load-bearing, and each closes a failure a laxer check admits.
///
/// * BEFORE the `(`: only a plain path. A field whose value is a BLOCK that calls
///   the helper and then answers with something else mentions `name(` just as
///   happily, so a containment check passes the discarded-result regression.
/// * AFTER the closing `)`: nothing but whitespace. This is the sharper one:
///   a variant playing the same game back,
///   `alerts_at(now_ns).into_iter().take(0).collect()` has a perfectly plain head
///   and serves an EMPTY list, so a head-only check passes a handler that stamps
///   the ring correctly and then throws it away. Any trailing transformation is
///   rejected — which also means a legitimate future one has to update this guard
///   deliberately, and that is the point.
fn is_call_to(expr: &str, name: &str) -> bool {
    let expr = expr.trim();
    let Some(open) = expr.find('(') else {
        return false;
    };
    let head = &expr[..open];
    if !head
        .chars()
        .all(|c| c.is_alphanumeric() || c == '_' || c == '.' || c == ':')
        || head.rsplit(['.', ':']).next() != Some(name)
    {
        return false;
    }
    // The call must be the WHOLE expression: paren-match its argument list and
    // require nothing to follow.
    let mut depth = 0usize;
    for (off, ch) in expr[open..].char_indices() {
        match ch {
            '(' => depth += 1,
            ')' => {
                depth -= 1;
                if depth == 0 {
                    return expr[open + off + 1..].trim().is_empty();
                }
            }
            _ => {}
        }
    }
    false
}

/// Does `gate` COMPARE an elapsed span against `constant`?
///
/// Structural, and the distinction is exactly what a substring check misses: the
/// constant has to sit on one side of a comparison whose OTHER side computes an
/// elapsed duration. A gate reading `is_none_or(|_| CONSTANT > 0)` over the mark
/// names BOTH and gates NOTHING — it is `true` on every pass — so a guard built
/// from two `contains` calls passes the every-poll regression it exists to catch.
/// (That regression is a vector below.)
///
/// The comparison examined is the one NEAREST the constant, and the constant's own
/// side must NOT also contain the elapsed call — so the two really are on opposite
/// sides rather than merely both present somewhere in the expression.
fn compares_elapsed_against(gate: &str, constant: &str) -> bool {
    const ELAPSED: [&str; 3] = ["saturating_duration_since", "duration_since", "elapsed"];
    let Some(at) = gate.find(constant) else {
        return false;
    };
    // The comparison operator nearest the constant. Two-character operators are
    // matched first so `>=` is never read as a bare `>`.
    let mut best: Option<(usize, usize)> = None;
    for op in [">=", "<=", ">", "<"] {
        let mut from = 0usize;
        while let Some(rel) = gate[from..].find(op) {
            let start = from + rel;
            from = start + op.len();
            // A bare `>`/`<` that is really half of `>=`/`<=`, or the tail of `->`
            // / `=>`, is not a comparison of its own.
            if op.len() == 1 {
                let two = &gate[start..];
                let prev = gate[..start].chars().next_back();
                if two.starts_with(">=")
                    || two.starts_with("<=")
                    || prev == Some('-')
                    || prev == Some('=')
                {
                    continue;
                }
            }
            if best.is_none_or(|(b, _)| at.abs_diff(start) < at.abs_diff(b)) {
                best = Some((start, op.len()));
            }
        }
    }
    let Some((start, len)) = best else {
        return false;
    };
    let (lhs, rhs) = (&gate[..start], &gate[start + len..]);
    let (with_constant, other) = if lhs.contains(constant) {
        (lhs, rhs)
    } else {
        (rhs, lhs)
    };
    !ELAPSED.iter().any(|e| with_constant.contains(e)) && ELAPSED.iter().any(|e| other.contains(e))
}

/// The value expression of struct-literal field `field` inside `literal`.
///
/// Reads to the field-separating `,` at nesting depth 0, so a value that is itself a
/// call (or a nested literal) is returned whole.
fn struct_field<'a>(literal: &'a str, field: &str) -> &'a str {
    let key = format!("{field}:");
    let at = literal
        .find(&key)
        .unwrap_or_else(|| panic!("no `{key}` field in:\n{literal}"));
    let rhs = &literal[at + key.len()..];
    let mut depth = 0usize;
    for (off, ch) in rhs.char_indices() {
        match ch {
            '{' | '(' | '[' => depth += 1,
            '}' | ')' | ']' => {
                if depth == 0 {
                    return rhs[..off].trim();
                }
                depth -= 1;
            }
            ',' if depth == 0 => return rhs[..off].trim(),
            _ => {}
        }
    }
    rhs.trim()
}

/// The first occurrence of `needle` that starts a TOKEN, so `robot,` is not found
/// inside `origin_robot,`.
fn find_token(src: &str, needle: &str) -> Option<usize> {
    let mut from = 0usize;
    while let Some(rel) = src[from..].find(needle) {
        let at = from + rel;
        from = at + 1;
        if token_start(src, at) {
            return Some(at);
        }
    }
    None
}

/// The value expression a struct-literal field CARRIES, in either of Rust's two
/// spellings, with a `let` binding resolved one level.
///
/// [`struct_field`] alone understands `field: expr` and PANICS on the SHORTHAND
/// `field,` — which is how the shipped `AttachResponse` writes `robot`, so a guard
/// built on it would not have failed loudly, it would not have compiled a claim at
/// all. Both spellings mean the same thing and a refactor may swap them freely, so
/// the reader has to accept both.
///
/// A shorthand whose binding cannot be found is a LOUD failure rather than a
/// silently-unresolved name: reading `robot` as its own value would make every
/// assertion over the result vacuous, which is the exact failure mode this whole
/// file is written against.
fn struct_field_value<'a>(body: &'a str, literal: &'a str, field: &'a str) -> &'a str {
    if find_token(literal, &format!("{field}:")).is_some() {
        return resolve_binding(body, struct_field(literal, field));
    }
    assert!(
        find_token(literal, &format!("{field},")).is_some(),
        "no `{field}` field, in either the `{field}: expr` or the `{field},` \
         shorthand spelling, in:\n{literal}"
    );
    let resolved = resolve_binding(body, field);
    assert_ne!(
        resolved, field,
        "`{field}` is written in the struct-literal SHORTHAND, so its value is \
         whatever the binding of that name computes — and no `let {field} =` was \
         found in the enclosing function. Reading the name as its own value would \
         make every assertion over it vacuous."
    );
    resolved
}

/// THE non-blocking pin: vizd never calls netd's LOOPING query verbs — anywhere in
/// `src/`, by DIRECTORY WALK.
///
/// Those verbs run a multi-second first-contact wait behind `&mut self`. Reached from
/// `NetdDemandPlane::with_client` they would (a) hold vizd's ONE netd client mutex for
/// the whole ceiling and (b) push the control reply past `viz_client`'s 5 s
/// `SO_RCVTIMEO`, turning a slow attach into a DEAD one. This is the exact INVERSE of
/// `cerulion_cli_engine`'s guard, which REQUIRES those verbs — same feature, two
/// consumers, two shapes, which is why each needs its own guard.
#[test]
fn vizd_never_calls_netds_looping_query_verbs() {
    let files = src_files();
    let mut scanned = Vec::new();
    for f in &files {
        let code = code_of(f);
        for looping in ["query_catalog_converged", "query_schema_converged"] {
            assert!(
                !code.contains(looping),
                "{} calls `{looping}`, which runs a multi-second wait behind \
                 `&mut self`. In a vizd control handler that holds the ONE netd client \
                 mutex for its whole ceiling AND pushes the reply past viz_client's 5s \
                 SO_RCVTIMEO — `cerulion viz` then dies on an errno instead of being \
                 slower. Answer in one round trip and emit a `RetryHint` instead; the \
                 wait belongs in `viz_client::attach_waiting_for_discovery`.",
                f.display()
            );
        }
        scanned.push(f.file_name().unwrap().to_string_lossy().to_string());
    }

    // ANTI-TAUTOLOGY, two ways. (1) The walk really reached the modules that matter —
    // otherwise a broken walk passes silently. (2) The NON-looping verb vizd DOES use
    // must still be there, or the negatives above would also pass a vizd that had
    // stopped querying netd at all.
    for known in ["daemon.rs", "protocol.rs", "events.rs", "net.rs"] {
        assert!(
            scanned.iter().any(|f| f == known),
            "the src walk did not reach {known}; it saw {scanned:?}"
        );
    }
    assert!(
        code_containing("fn query_catalog_all_with_discovery")
            .contains("query_catalog_with_discovery"),
        "vizd still queries netd through the NON-looping verb"
    );
}

/// The LOCK-SCOPE pin: the netd client mutex is taken in exactly ONE place.
///
/// `with_client` takes the guard, runs one round trip, and drops it. Every
/// `DemandPlane` method must reach the client through it — a method that locked the
/// client itself could hold it across anything.
#[test]
fn the_netd_client_mutex_is_taken_only_inside_with_client() {
    let code = code_containing("fn with_client<");
    let needle = "self.client.lock()";
    let total = code.matches(needle).count();
    assert!(
        total >= 1,
        "the production plane must lock its client somewhere"
    );
    let inside = fn_body(&code, "fn with_client<").matches(needle).count();
    assert_eq!(
        total, inside,
        "`{needle}` appears {total} time(s) but only {inside} inside `with_client`, \
         whose guard is scoped to ONE round trip"
    );
}

/// The PROMPT-ANSWER pin: the catalog gather every attach seam funnels through runs ONE
/// plane call and contains no loop, sleep or wait.
///
/// This is the shape assertion the e2e cannot make. A scripted `DemandPlane` answers
/// instantly, so a handler that looped would look identical in every behavioural arm
/// while being fatal against a real netd through a 5 s-deadlined socket.
#[test]
fn the_catalog_gather_seam_takes_exactly_one_round_trip() {
    let code = code_containing("fn gather_catalog(");
    let body = fn_body(&code, "fn gather_catalog(");
    for forbidden in ["loop", "while ", "sleep", "ConvergenceWait", "Instant::now"] {
        assert!(
            !body.contains(forbidden),
            "`gather_catalog` contains `{forbidden}` — this seam must answer in ONE \
             round trip. A wait here is unreachable through viz_client's 5s reply \
             deadline; emit a `RetryHint` and let the client keep asking.\n\nbody:\n{body}"
        );
    }
    // ANTI-TAUTOLOGY: it really does query the plane (a body that did nothing would
    // satisfy every negative above).
    assert!(
        body.contains("query_catalog_with_discovery")
            && body.contains("query_catalog_all_with_discovery"),
        "and it really does gather, on both the single-robot and LAN-wide verbs:\n{body}"
    );
}

/// The HINT pin: both attach seams mark a NOT-FOUND-YET, and the schema-fetch sibling
/// deliberately does not.
///
/// Function-scoped, because "the file mints hints somewhere" is true the moment ONE
/// seam does and cannot see a seam that stopped. The `resolve_remote_type` half is the
/// posture assertion whose failure is a REGRESSION rather than a revert when it
/// is wrong: by then the catalog has named the producing robot, so re-asking would burn
/// a client's budget on an answer that cannot change.
#[test]
fn the_two_attach_seams_mark_a_not_found_yet_and_the_schema_fetch_does_not() {
    let code = code_containing("fn catalog_resolve_type(");

    let single_robot = fn_body(&code, "fn catalog_resolve_type(");
    assert!(
        single_robot.contains("not_found_yet"),
        "the schema-less seam must mark a missing type re-askable:\n{single_robot}"
    );

    // The LAN-wide seam mints its hint from the resolution's own `retryable` verdict.
    let lan_wide = fn_body(&code, "fn attach_resolving_remote(");
    assert!(
        lan_wide.contains("retry_hint_for") && lan_wide.contains("retryable"),
        "the no-robot seam must forward the resolution's retryable verdict:\n{lan_wide}"
    );

    let schema_fetch = fn_body(&code, "fn resolve_remote_type(");
    assert!(
        !schema_fetch.contains("not_found_yet"),
        "the schema FETCH half must NOT be marked re-askable — the robot is already \
         named, so re-asking cannot change the answer:\n{schema_fetch}"
    );
}

/// The stripper is oracle-tested rather than trusted: every negative assertion above is
/// enforced over ITS output, so a stripper that dropped the tail would make them all
/// vacuous.
#[test]
fn code_only_strips_comments_and_literals_and_reports_an_unbalanced_scan() {
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

    // String literals are blanked — including the raw form.
    let s = code_only("let x = \"query_catalog_converged\"; let y = r#\"also /* here\"#; z");
    assert!(!s.code.contains("query_catalog_converged"), "{}", s.code);
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
}

/// The two structural PREDICATES are oracle-tested, not trusted.
///
/// They are what the monitors guards assert THROUGH, so a predicate that answers
/// `true` too easily makes its guard vacuous without failing anything — which is
/// exactly the hole both of them can leave open. The
/// vectors below include those two shapes verbatim, and every case is hand-written
/// against the answer it must give.
#[test]
fn the_structural_predicates_answer_their_hand_written_vectors() {
    // ── store_sites: a PUBLICATION, not a mention ──────────────────────────────
    // These are what the gauge guards assert THROUGH, so a predicate that answers
    // too easily makes its guard vacuous without failing anything — which is
    // precisely how the identifier-presence version shipped.
    assert_eq!(
        store_sites("self.g.store(v, Ordering::Relaxed);", "g").len(),
        1
    );
    // rustfmt puts the `.` of a chain on the NEXT line — the shape a needle
    // spelled `g.store(` could not match.
    assert_eq!(
        store_sites("self.g\n    .store(v, Ordering::Relaxed);", "g").len(),
        1
    );
    // THE HAZARD: a mention under the lock, the store elsewhere. The mention must
    // NOT be reported as a publication, or the guard is satisfied by the race.
    assert_eq!(store_sites("let _r = &self.g;", "g").len(), 0);
    assert_eq!(
        store_sites("let v = self.g.load(Ordering::Relaxed);", "g").len(),
        0
    );
    assert_eq!(
        store_sites("// self.g.store(x, y);", "g").len(),
        1,
        "the caller strips comments; this predicate is not asked to"
    );
    // Two publications are two sites, and a LONGER identifier sharing the prefix
    // is not this one (token-start + the receiver ending at the `.`).
    assert_eq!(
        store_sites("self.g.store(a, b); self.g\n.store(c, d);", "g").len(),
        2
    );
    assert_eq!(store_sites("self.gauge.store(v, o);", "g").len(), 0);
    assert_eq!(store_sites("self.my_g.store(v, o);", "g").len(), 0);

    // ── is_call_to: the value must BE the call ─────────────────────────────────
    assert!(is_call_to(
        "st.monitors.alerts_at(now_ns / 1_000)",
        "alerts_at"
    ));
    assert!(is_call_to("alerts_at()", "alerts_at"));
    assert!(is_call_to("Plane::alerts_at(a, b(c))", "alerts_at"));
    assert!(
        is_call_to("  st.monitors.rows(now_ns)  ", "rows"),
        "surrounding whitespace is not a transformation"
    );
    // THE EVASIVE SHAPE: a correctly-stamped ring thrown away by a trailing
    // transformation. The head is perfectly plain, so a head-only check admits it.
    assert!(
        !is_call_to(
            "st.monitors.alerts_at(now_ns).into_iter().take(0).collect()",
            "alerts_at"
        ),
        "a trailing transformation can serve an EMPTY list from a correct call"
    );
    // The discarded-result shapes.
    assert!(!is_call_to("Vec::new()", "alerts_at"));
    assert!(
        !is_call_to(
            "{ let _s = st.monitors.alerts_at(n); Vec::new() }",
            "alerts_at"
        ),
        "a BLOCK that calls the helper and answers with something else"
    );
    // Not a call at all, and a name that merely ends in the right letters.
    assert!(!is_call_to("st.monitors.alerts", "alerts_at"));
    assert!(!is_call_to("st.monitors.stale_alerts_at(n)", "alerts_at"));
    assert!(!is_call_to("alerts_at(unbalanced", "alerts_at"));

    // ── loop_body_over: the per-ITEM scope, not the rest of the function ───────
    // This is what the report guards assert THROUGH, and the shape it
    // exists to reject is the one its predecessor accepted: a line AFTER the
    // collection but OUTSIDE the loop.
    assert_eq!(
        loop_body_over("for x in fold.silent { debug!(x); }", "fold.silent"),
        "{ debug!(x); }"
    );
    // The loop is found among OTHERS, and by its own collection.
    assert_eq!(
        loop_body_over(
            "for a in fold.unusable { warn!(a); } for b in fold.silent { debug!(b); }",
            "fold.silent"
        ),
        "{ debug!(b); }"
    );
    // THE HAZARD: the report moved OUT of the loop. The body must not contain it,
    // or the guard is satisfied by the very regression it exists to catch.
    let moved_out = "for x in fold.silent { let _ = x; } debug!(robot = \"?\");";
    assert!(
        !loop_body_over(moved_out, "fold.silent").contains("debug!"),
        "a line after the loop is not a line inside it"
    );
    // A nested block does not end the body early.
    assert_eq!(
        loop_body_over("for x in fold.silent { if p { q(); } r(); }", "fold.silent"),
        "{ if p { q(); } r(); }"
    );
    // `for` as the tail of an identifier is not a loop keyword.
    assert_eq!(
        loop_body_over(
            "let metafor = 1; for x in fold.silent { debug!(x); }",
            "fold.silent"
        ),
        "{ debug!(x); }"
    );

    // ── macro_invocation: ONE call's span, not the enclosing block ─────────────
    // This is what the report guards assert THROUGH, and the shape it
    // exists to reject is the one its predecessor accepted: two claims satisfied by
    // two DIFFERENT calls.
    assert_eq!(
        macro_invocation("{ warn!(robot = %r, \"gone\"); }", "warn"),
        "warn!(robot = %r, \"gone\")"
    );
    // THE HAZARD: the field lives on a DIFFERENT call. The warn's own span must not
    // contain it, or the guard is satisfied by the split it forbids.
    assert!(
        !macro_invocation("warn!(\"unusable\"); debug!(robot = %r, \"x\");", "warn")
            .contains("robot ="),
        "a field on a neighbouring call is not on this one"
    );
    // Nested parens inside the argument list do not end the span early.
    assert_eq!(
        macro_invocation("warn!(robot = %f(g(x)), \"m\") ;", "warn"),
        "warn!(robot = %f(g(x)), \"m\")"
    );
    // A path-qualified name is found, and the BARE name finds the same call.
    assert_eq!(
        macro_invocation("tracing::warn!(a, b)", "tracing::warn"),
        "tracing::warn!(a, b)"
    );
    assert_eq!(macro_invocation("tracing::warn!(a)", "warn"), "warn!(a)");
    // A longer identifier ending in the needle is NOT this macro.
    assert_eq!(
        macro_invocation("mywarn!(x); warn!(y)", "warn"),
        "warn!(y)",
        "`mywarn!` is not `warn!`"
    );

    // ── compares_elapsed_against: the constant must be IN the comparison ───────
    const K: &str = "MONITOR_SAMPLE_INTERVAL_NS";
    assert!(
        compares_elapsed_against(
            "last_monitor_sample.is_none_or(|prev| { \
             iteration_start.saturating_duration_since(prev) \
             >= Duration::from_nanos(MONITOR_SAMPLE_INTERVAL_NS) })",
            K
        ),
        "the shipped gate"
    );
    assert!(
        compares_elapsed_against(
            "Duration::from_nanos(MONITOR_SAMPLE_INTERVAL_NS) < mark.elapsed()",
            K
        ),
        "either side, either operator"
    );
    // THE EVASIVE SHAPE: both names present, gates nothing, `true` every pass.
    assert!(
        !compares_elapsed_against(
            "last_monitor_sample.is_none_or(|_| MONITOR_SAMPLE_INTERVAL_NS > 0)",
            K
        ),
        "the constant is present but not compared against an elapsed span"
    );
    // No comparison at all, and a comparison the constant is not part of.
    assert!(!compares_elapsed_against(
        "mark.elapsed() >= Duration::from_nanos(400)",
        K
    ));
    assert!(
        !compares_elapsed_against(
            "let _ = MONITOR_SAMPLE_INTERVAL_NS; mark.elapsed() >= other",
            K
        ),
        "a mention beside an unrelated comparison is not the gate"
    );

    // ── struct_field_value: BOTH spellings, and a shorthand really resolves ────
    // The shipped `AttachResponse` writes `robot` in the SHORTHAND, so a reader that
    // handled only `field: expr` would not compile a weaker claim — it would panic.
    let body = "let robot = self.attribution_snapshot().get(&t).cloned(); \
                R(A { ok: true, robot, })";
    assert_eq!(
        struct_field_value(body, "{ ok: true, robot, }", "robot"),
        "self.attribution_snapshot().get(&t).cloned()",
        "a shorthand field resolves through its binding"
    );
    // THE HAZARD THIS ARM EXISTS FOR: the call survives, its answer is discarded, and
    // the reply says `None`. A body-wide `contains` passes it; this must not — in
    // BOTH spellings of the discarded binding, since either is a shape
    // a real regression can take.
    for discarded in [
        "let _ = self.attribution_snapshot(); let robot = None; R(A { ok: true, robot, })",
        "let _ = self.attribution_snapshot(); let robot: Option<String> = None; \
         R(A { ok: true, robot, })",
    ] {
        assert_eq!(
            struct_field_value(discarded, "{ ok: true, robot, }", "robot"),
            "None",
            "a discarded call beside a `None` robot must resolve to the NONE, not to \
             the body that still mentions the call"
        );
    }
    // …and the annotated form of the CORRECT code still resolves, so the guard does
    // not fail on a refactor that only adds a type.
    assert_eq!(
        struct_field_value(
            "let robot: Option<String> = self.attribution_snapshot().get(&t).cloned(); \
             R(A { robot, })",
            "{ robot, }",
            "robot"
        ),
        "self.attribution_snapshot().get(&t).cloned()"
    );
    // A generic argument's `>` must not end the annotation scan early, and a
    // longer name must not be resolved in place of the one asked for.
    assert_eq!(
        resolve_binding("let m: BTreeMap<String, Vec<u8>> = built();", "m"),
        "built()"
    );
    assert_eq!(
        resolve_binding("let robotics = wrong(); let robot = right();", "robot"),
        "right()",
        "`let robot` must not resolve `let robotics`"
    );
    assert_eq!(
        resolve_binding("let robot;", "robot"),
        "robot",
        "a declaration with no initialiser resolves to nothing"
    );
    // The explicit spelling, so a refactor swapping the two is not a false failure.
    assert_eq!(
        struct_field_value("", "{ ok: true, robot: gathered(x), }", "robot"),
        "gathered(x)"
    );
    // `robot,` must not be found inside `origin_robot,`.
    assert_eq!(
        struct_field_value(
            "let robot = mine();",
            "{ origin_robot: a, robot, }",
            "robot"
        ),
        "mine()",
        "the token scan must not match the tail of a longer field name"
    );
}

/// The shorthand reader FAILS LOUDLY rather than reading a name as its own value —
/// an unresolved binding would make every assertion over the result vacuous.
#[test]
#[should_panic(expected = "SHORTHAND")]
fn struct_field_value_panics_on_a_shorthand_whose_binding_is_missing() {
    struct_field_value("no binding here;", "{ ok: true, robot, }", "robot");
}

/// …and on a field that is not in the literal at all, in either spelling.
#[test]
#[should_panic(expected = "in either")]
fn struct_field_value_panics_on_an_absent_field() {
    struct_field_value("let robot = mine();", "{ ok: true, entity, }", "robot");
}

/// ANTI-TAUTOLOGY for the walk itself: the stripped view of the biggest module reaches
/// the END of the file, so no negative assertion above runs on a prefix.
#[test]
fn the_stripped_view_reaches_the_end_of_the_file() {
    let code = code_containing("fn gather_catalog(");
    assert!(
        code.contains("fn start_with_planes"),
        "the stripped view must reach the daemon's start ladder"
    );
    assert!(
        code.contains("mod tests"),
        "and its tail — otherwise every `!contains` above would be vacuous"
    );
}

// ===========================================================================
// Every ATTACHED-topic reporter reads the LIVE signals
// ===========================================================================

/// The four reporters `Ctx::layout_metadata_for_route`'s own doc names — "so the
/// four ATTACHED-topic reporters (both attach replies, `list`, `status`) cannot
/// drift from the applied blueprint by forgetting a signal" — must each go
/// through it, and none of them may reach for the blank-signal `layout_metadata`.
///
/// # Why this is not redundant with the e2e arms
///
/// Without this guard only one of the four is pinned by anything. Reverting
/// `list` plus both attach replies together leaves the
/// rest of the `cerulion_vizd` suite green, because
/// an e2e arm whose only report oracle is `status` has its other assertions
/// read the applied plan through an independent mirror read.
///
/// `vizd_e2e_test` now pins `list` and the LOCAL attach reply behaviourally. The
/// REMOTE attach reply (`attach_remote`) is the one that cannot be reached that
/// way: it demands a mirror from `cerulion-netd` and then taps it, so a hermetic
/// arm would have to stand up a netd, a remote robot and a re-injected mirror to
/// observe one field of one reply. That is the inert-shipping shape this file
/// exists for — so the seam is guarded structurally, and the other three are
/// guarded here TOO, because a structural guard that covers three of four seams
/// is one refactor away from covering none.
#[test]
fn every_attached_topic_reporter_reads_the_live_layout_signals() {
    let code = code_containing("fn layout_metadata_for_route");
    // Located by CONTENT, so the guard follows the code if it moves modules.
    for signature in [
        "fn attach_local(",
        "fn attach_remote(",
        "fn list(",
        "fn status(",
    ] {
        let body = fn_body(&code, signature);
        assert!(
            body.contains("layout_metadata_for_route("),
            "`{signature}` must build its reported placement from the LIVE signals \
             (`layout_metadata_for_route`), or it reports a placement the applied \
             blueprint does not have — the drift this guard exists to remove:\n{body}"
        );
        // `layout_metadata_for_route` CONTAINS `layout_metadata`, so the negative
        // assertion has to be on the blank-signal call SHAPE — its second argument
        // list ends in the two blanks a reverting refactor would restore.
        assert!(
            !body.contains("RenderProof::default()"),
            "`{signature}` must not hand the layout a BLANK render proof — that is \
             precisely the revert no behavioural test catches:\n{body}"
        );
    }
}

/// The complement, and the reason the arm above cannot simply forbid the
/// blank-signal helper outright: `discover` KEEPS it, for the rows where it is the
/// correct answer.
///
/// A `discover` row for a topic this desk has never attached has no live signal at
/// all — nothing has rendered it — so the default (companion KEPT) is the
/// truthful report rather than a lookup somebody forgot. The fix made
/// `discover` thread the live signals for rows that ARE attached (see
/// `Ctx::layout_metadata_for_discover_row`), which is where the "one run, two
/// answers" drift lived; the defaulted path stays, and this pins that it does.
#[test]
fn discover_keeps_the_defaulted_path_for_rows_nothing_has_rendered() {
    let code = code_containing("fn layout_metadata_for_discover_row");
    let body = fn_body(&code, "fn layout_metadata_for_discover_row");
    assert!(
        body.contains("RenderProof::default()"),
        "the defaulted arm must survive: a topic nobody has rendered has no live \
         signal, and inventing one would be the inverse defect:\n{body}"
    );
    assert!(
        body.contains("layout_metadata_for_route("),
        "…and an ATTACHED discover row must go through the live-signal seam, or \
         `discover` and `status` answer differently about the same topic on the \
         same daemon:\n{body}"
    );
}

/// A `discover` row reads its state once.
///
/// `list` and `status` capture a row's representation AND its route inside a single
/// `st` guard. The `discover` row took the lock THREE times — representation, route,
/// undecodable verdict — so a `set_representation` or a `detach` landing between them
/// could pair a representation from one snapshot with a route from another, and the
/// row's own `representation` FIELD (a third read) could disagree with the
/// `view_kinds` printed beside it, which is precisely what the representation verb carries that
/// field to prevent. That is the "one run, two answers" class this seam exists to
/// close, re-created inside the seam itself.
///
/// # Why this guard is STRUCTURAL
///
/// The window is between two lock acquisitions on one thread. Reproducing it needs an
/// interleaving no hermetic arm can force without a seam inside the helper — and a
/// seam whose only purpose is to widen the window it is testing proves the seam, not
/// the code. What IS checkable, deterministically, is the SHAPE: the placement helper
/// takes no lock at all, the snapshot takes exactly one, and `discover` reaches state
/// through nothing else. Each of the three multi-lock reads fails a different assertion
/// below, so a partial revert cannot hide.
#[test]
fn a_discover_row_reads_its_state_under_one_lock() {
    let code = code_containing("fn discover_row_state(");

    // (1) The SNAPSHOT is the single acquisition — not two under one function name.
    let snapshot = fn_body(&code, "fn discover_row_state(");
    assert_eq!(
        snapshot.matches("self.state.lock(").count(),
        1,
        "`discover_row_state` must take the state lock EXACTLY once — its whole \
         purpose is that a row's representation, route and undecodable verdict come \
         from ONE snapshot:\n{snapshot}"
    );

    // (2) The placement helper reads NO state. It receives the snapshot's values, so
    // a lookup here would be a second acquisition returning.
    let placement = fn_body(&code, "fn layout_metadata_for_discover_row(");
    for forbidden in ["self.state.lock(", "representation_for("] {
        assert!(
            !placement.contains(forbidden),
            "`layout_metadata_for_discover_row` contains `{forbidden}` — it must be \
             handed the snapshot, not look state up itself, or the representation it \
             places by can be newer or older than the route it places on:\n{placement}"
        );
    }

    // (3) The VERB reaches state through the snapshot and nothing else — this is the
    // half that catches the row's own `representation` / `undecodable` fields
    // drifting away from the placement printed beside them.
    let discover = fn_body(&code, "fn discover(&self, id: u64)");
    for forbidden in ["self.state.lock(", "representation_for("] {
        assert!(
            !discover.contains(forbidden),
            "`discover` contains `{forbidden}` — every state value a row reports must \
             come from its `discover_row_state` snapshot, or the row's own fields can \
             disagree with its `view_kinds`:\n{discover}"
        );
    }

    // (4) ANTI-TAUTOLOGY: the assertions above are all negative, so they would also
    // pass a `discover` that had stopped reporting placements entirely. Every row arm
    // that computes one must take exactly one snapshot to compute it from.
    let placements = discover
        .matches("layout_metadata_for_discover_row(")
        .count();
    let snapshots = discover.matches("discover_row_state(").count();
    assert!(
        placements >= 2,
        "`discover` must still build a placement for BOTH row arms (genuine-local and \
         streaming-mirror); found {placements}:\n{discover}"
    );
    assert_eq!(
        placements, snapshots,
        "{placements} placement(s) but {snapshots} snapshot(s) — each row arm takes \
         its own single snapshot and computes everything from it:\n{discover}"
    );
}

/// The fold that decides which row a client actually READS must
/// keep carrying the mirror's live layout metadata onto a catalog-won row.
///
/// `remote_discovered_entry` builds its row from BLANK signals — correct for a
/// topic this desk is not rendering, and the reason the defect was invisible: the
/// `discover` seam computed a live row, the fold's duplicate guard `continue`d,
/// and the blank row survived. A refactor restoring that `continue` reverts the
/// Studio sidebar to reporting a pane the installed blueprint does not have,
/// while every OTHER surface stays right.
///
/// Behaviourally pinned by
/// `vizd_e2e_test::a_rendering_mirror_reports_its_applied_placement_on_the_robots_section_e2e`
/// and two pure fold arms; guarded here too because the guard is what a reader
/// grepping `remote_discovered_entry`'s blanks finds.
#[test]
fn the_mirror_fold_carries_live_layout_metadata_onto_a_catalog_won_row() {
    let code = code_containing("fn fold_streaming_mirrors_into_robots");
    let body = fn_body(&code, "fn fold_streaming_mirrors_into_robots");
    for needle in [
        "existing.components",
        "existing.view_kinds",
        // The CAUSE travels with the consequence — carrying the layout pair
        // while leaving `representation` absent makes the row report a placement its
        // archetype alone does not imply, with nothing to explain why.
        "existing.representation",
    ] {
        assert!(
            body.contains(needle),
            "the duplicate-row arm must CARRY `{needle}` from the mirror, not discard \
             the mirror entry whole — otherwise the surviving row is the blank-signal \
             one and `discover` disagrees with `status`:\n{body}"
        );
    }
    // The gate is part of the claim: carrying across a DIFFERENT archetype would
    // place a row under views its own `archetype` field does not imply.
    assert!(
        body.contains("existing.archetype == entry.archetype"),
        "the carry must be gated on the two archetypes agreeing:\n{body}"
    );
}

// ===========================================================================
// Monitors — the standing watchdog's SAMPLER and its handler
// ===========================================================================

/// The sampler runs on the DRAIN LOOP, and nowhere else.
///
/// This is the shape assertion no behavioural arm can make, and it guards two
/// distinct failures at once.
///
/// A sampler driven from a control HANDLER would make the watchdog's cadence a
/// function of who is polling it — and the engine's confirmations are counted in
/// SAMPLES, so a topic would be judged faster the more often Studio's sidebar
/// refreshed and never at all while nobody looked. An e2e cannot see that: the
/// harness polls, so the samples arrive either way and every alert still appears.
///
/// A sampler on a THREAD of its own would need the netd client mutex or a second
/// lock over the daemon state, which is the class `the_netd_client_mutex_is_taken_only_inside_with_client`
/// above exists to prevent — and `poll_loop` is scoped by none of those assertions
/// precisely because it is the one place this work belongs.
#[test]
fn monitors_v1_the_sampler_is_driven_from_the_drain_loop_and_no_handler_samples() {
    let code = code_containing("fn poll_loop(");
    let body = fn_body(&code, "fn poll_loop(");
    assert!(
        body.contains("sample_monitors"),
        "the drain loop must drive the monitor sampler:\n{body}"
    );

    // …and NOTHING else may. The call site count over the whole of `src/` must be
    // exactly the definition plus the ONE call in `poll_loop`.
    let mut call_sites = 0usize;
    for f in src_files() {
        call_sites += code_of(&f).matches("sample_monitors").count();
    }
    assert_eq!(
        call_sites, 2,
        "expected exactly the `fn sample_monitors` definition and ONE call (in \
         `poll_loop`), found {call_sites} mentions across src/ — a second caller \
         makes the watchdog's cadence depend on who is asking"
    );
}

/// The sampler is GATED, and the gate is the whole cost claim.
///
/// The cost promise — "per 16 ms pass when the sample interval has not
/// elapsed: one `Instant` compare" — is a statement about the drain loop's body,
/// so it is checked there. Sampling every pass instead would still pass every
/// behavioural arm (the alerts appear SOONER, which no oracle forbids) while
/// re-reading one robot-side observation ~12 times per sweep, taking the daemon's
/// state lock 25× more often than intended, and confirming a fault in a fraction
/// of the span the lull-free-restart residual needs.
///
/// The constant is named rather than a literal, so it cannot drift from the
/// substrate constant it is derived from.
///
/// The assertion is STRUCTURAL, not textual, and that is the whole of it: a
/// textual check that the constant appears before the call is one which
/// an UNCONDITIONAL `ctx.sample_monitors(..)` preceded by any mention of the
/// constant satisfies — i.e. it passes the every-poll regression it is written to
/// catch. What is checked is that the call really sits inside a branch, and
/// that the condition of THAT branch (resolved through its binding) is computed
/// from the interval.
#[test]
fn monitors_v1_the_sampler_is_gated_on_the_derived_interval_not_every_pass() {
    let code = code_containing("fn poll_loop(");
    let body = fn_body(&code, "fn poll_loop(");
    // The call must be INSIDE a conditional — this panics if it is not.
    let condition = governing_condition(body, "ctx.sample_monitors");
    // …and that conditional must be the interval gate, not some unrelated branch.
    let gate = resolve_binding(body, condition);
    // The gate must READ the mark it advances, or it fires on every pass anyway.
    assert!(
        gate.contains("last_monitor_sample"),
        "the gate must consult when the sampler LAST ran:\n{gate}"
    );
    // And the interval must be IN THE COMPARISON, not merely somewhere in the
    // expression. Two `contains` calls accept
    // `last_monitor_sample.is_none_or(|_| MONITOR_SAMPLE_INTERVAL_NS > 0)` — both
    // names present, gates nothing, `true` on every pass.
    assert!(
        compares_elapsed_against(gate, "MONITOR_SAMPLE_INTERVAL_NS"),
        "the branch guarding `ctx.sample_monitors` must COMPARE the elapsed span \
         since the mark against the DERIVED interval. Sampling every 16 ms pass \
         re-reads one robot-side observation ~12 times per sweep and takes the \
         state lock 25x more often, while passing every behavioural arm (the \
         alerts appear SOONER, which no oracle forbids).\n\ncondition: \
         {condition}\ngate: {gate}"
    );
}

/// The `monitors` handler ANSWERS — it never samples, gathers, waits or sleeps.
///
/// Same contract every other vizd handler is held to, plus one rule of its
/// own: a verb that sampled on demand would be a second, client-driven cadence for
/// a watchdog whose confirmations are counted in samples. Function-scoped, because
/// "the file contains no `sleep`" is true of nothing.
#[test]
fn monitors_v1_the_handler_answers_promptly_and_samples_nothing() {
    let code = code_containing("fn monitors(&self");
    let body = fn_body(&code, "fn monitors(&self");
    for forbidden in [
        "loop",
        "while ",
        "sleep",
        "ConvergenceWait",
        "gather_catalog",
        "sample_monitors",
        "observe_pass",
        // And the DISCOVERY plane's feed, which additionally reaches a
        // netd gather's answer — `observe_pass` above already covers
        // `observe_catalog_pass` as a substring, but the seam a handler would
        // reach for is the `Ctx` method.
        "observe_catalog_monitors",
    ] {
        assert!(
            !body.contains(forbidden),
            "`monitors` contains `{forbidden}` — this handler must ANSWER in one \
             lock acquisition. Sampling belongs on the drain loop, where the \
             cadence does not depend on who is asking.\n\nbody:\n{body}"
        );
    }
    // ANTI-TAUTOLOGY: it really does serve the plane (a body that did nothing
    // would satisfy every negative above).
    assert!(
        body.contains("st.monitors.rows(") && body.contains("st.monitors.alerts_at("),
        "and it really does serve the rows AND the ring:\n{body}"
    );
    // ONE state-lock acquisition. The handler's own doc makes this claim and
    // nothing checked it: a handler that locked separately for the rows and for
    // the ring would pass every assertion above while serving a response whose two
    // halves are dated from different instants — a row that says `stalled` beside
    // an alert list that does not yet contain the stall, or the reverse. It is a
    // race, so no deterministic arm can catch it; the shape is what is checkable.
    let locks = body.matches("self.state.lock()").count();
    assert_eq!(
        locks, 1,
        "`monitors` must answer from ONE acquisition — its rows and its alert ages \
         are dated from a single instant, and a second lock lets a sampling pass \
         land between the two halves of one response:\n{body}"
    );
}

/// Every served alert is stamped with its SERVE-TIME age, through the ONE helper.
///
/// `raised_at_ms` / `cleared_at_ms` are this daemon's monotonic clock, which no
/// reader shares — so an unstamped ring is not merely missing a convenience, it
/// leaves a reader with two numbers it cannot subtract anything from. Serving them
/// raw is the easy regression (the field is `Option` and `None` serializes to
/// nothing at all, so the response stays valid and the omission is invisible), and
/// hand-computing the age at the handler is the other one — which is why the
/// assertion is that the ONE helper is called rather than that some subtraction
/// happens.
///
/// And it is checked where the value LANDS, not merely where the call appears: a
/// handler that called `alerts_at` into a discarded local and put an unstamped (or
/// empty) list on the response satisfies "the body mentions `alerts_at(`" and the
/// helper's own `with_age` check at the same time. So the assertion reads the
/// `alerts` FIELD of the response literal and requires the helper to be what
/// populates it.
#[test]
fn monitors_v1_the_served_ring_is_stamped_through_the_one_age_helper() {
    let code = code_containing("fn alerts_at(");
    let body = fn_body(&code, "fn alerts_at(");
    assert!(
        body.contains("with_age"),
        "the ring must be stamped through `Alert::with_age`, which owns the rule \
         (the age is measured from the TRANSITION this entry records):\n{body}"
    );
    // And the handler must go through THAT — as the VALUE of the response's
    // `alerts` field, not as a call whose result goes nowhere.
    let handler_code = code_containing("fn monitors(&self");
    let handler = fn_body(&handler_code, "fn monitors(&self");
    let open = handler
        .find("MonitorsResponse {")
        .map(|at| at + "MonitorsResponse ".len())
        .expect("the handler builds a MonitorsResponse");
    let literal = block_at(handler, open).expect("the response literal is balanced");
    let alerts = resolve_binding(handler, struct_field(literal, "alerts"));
    assert!(
        is_call_to(alerts, "alerts_at"),
        "the response's `alerts` field must BE the stamped ring — not merely a body \
         that MENTIONS the helper. A discarded result beside a raw or empty list is \
         the regression, and it reads as valid on the wire because `age_ms` is an \
         `Option` that serializes to nothing.\n\nalerts: {alerts}\nliteral: {literal}"
    );
    // The rows travel the same way, from the same acquisition.
    let rows = resolve_binding(handler, struct_field(literal, "monitors"));
    assert!(
        is_call_to(rows, "rows"),
        "…and the `monitors` field must BE the plane's rows: {rows}"
    );
}

/// The sampler's snapshot and its engine pass are ONE state-lock acquisition.
///
/// Two acquisitions leave a window in which a concurrent `detach` releases a row
/// between them — and the engine then RESURRECTS it from the already-captured
/// sample, as a row nothing can ever update again, reporting UNKNOWN with a
/// forever-growing age. It is a race, so no deterministic arm can catch it; what
/// is checkable is the shape.
#[test]
fn monitors_v1_the_sampler_takes_the_state_lock_exactly_once() {
    let code = code_containing("fn sample_monitors(&self");
    let body = fn_body(&code, "fn sample_monitors(&self");
    let locks = body.matches("self.state.lock()").count();
    assert_eq!(
        locks, 1,
        "the snapshot and the engine pass must share ONE acquisition — a second \
         one lets a concurrent `detach` land between them:\n{body}"
    );
    // The DELTAS are folded OUTSIDE it: a relaxed `fetch_add` under the lock the
    // drain needs every pass is work that does not belong there.
    let (_, scope_open, scope_end) = lock_scope(body, "sample_monitors");
    for counter in [
        "monitor_samples_taken",
        "monitor_alerts_raised",
        "monitor_alerts_cleared",
    ] {
        let at = body
            .find(counter)
            .unwrap_or_else(|| panic!("`{counter}` is folded in sample_monitors:\n{body}"));
        assert!(
            at > scope_end,
            "`{counter}` must be bumped OUTSIDE the lock scope:\n{body}"
        );
    }
    // …and the GAUGE is published INSIDE it — see the catalog feed's twin for the
    // commutative-vs-latest-wins argument. BOTH planes write this one atomic, so a
    // fix applied to one of them only leaves the race exactly as it was.
    let stores = store_sites(body, "monitor_rows_ineligible");
    assert_eq!(stores.len(), 1, "exactly ONE gauge publication:\n{body}");
    assert!(
        stores[0] > scope_open && stores[0] < scope_end,
        "the `monitor_rows_ineligible` GAUGE must be STORED inside the lock scope \
         that produced it. Checking only that the identifier OCCURS in the scope \
         passes a body that reads it under the lock and stores after — which is \
         the race verbatim:\n{body}"
    );
}

/// The ONE writer of a tap's robot is `set_origin_robot`, and a NAME-KEYED claim
/// only ever reaches it through the refresher.
///
/// This is the tombstone rule as a shape. The `/__cerulion/mirrors` map is
/// keyed by TOPIC NAME, so it describes a name rather than a tap's generation, and
/// recording one into a tap at attach time turns a 3-second staleness into a
/// permanent, affirmatively-wrong robot on a desk-owned topic (MEASURED: it never
/// healed, because the HELD half of `attribution_snapshot` then re-asserts it to
/// every surface forever). The rule that prevents it is not "be careful" — it is
/// that the only path from that map to a tap runs through a refresher that re-reads
/// it every pass and can therefore take the claim BACK.
///
/// So: the attach seams may not WRITE the map onto a tap, and the refresher must
/// consult the CACHE rather than gather (it runs on the poll thread every 400 ms).
///
/// # …and they may not pay a BLOCKING gather to compute one either
///
/// The writer rule alone leaves a hole: a seam that calls
/// `gather_mirror_provenance` and then does nothing with the answer writes no robot,
/// so every assertion above passes while the handler has taken on a
/// `MIRROR_GATHER_WINDOW`. That is vizd's promptness contract, not the tombstone
/// one, and it is invisible behaviourally — the gather is TTL-cached, so on a hit it
/// answers from the same map instantly and the only difference is wall time on a
/// handler whose budget is already spoken for.
///
/// An earlier guard (`monitors_v1_compose_attributes_from_the_cache_and_never_gathers`,
/// added in `e1e67e0c9`) made exactly that assertion for `compose_layout`. A later
/// change deleted it wholesale when the attribution moved to the sampler, taking the
/// promptness half with the attribution half, so the fetch rule is restored HERE —
/// beside the writer rule it belongs with.
///
/// The two seams differ on WHICH fetches are forbidden, and the difference is the
/// point rather than an inconsistency (see the per-seam notes below).
///
/// # Where the VALUE contract lives, and why it is not here
///
/// `attach_local`'s gather is ALLOWED, so the anti-tautology arm at the bottom has to
/// assert the seam still makes it. That arm pins the CALL and the DERIVATION — the
/// reply's `robot` is computed from the merged snapshot — and it deliberately stops
/// there: a source walk cannot tell a correct lookup from a wrong one.
///
/// The VALUE is pinned behaviourally, and value-sensitively, by
/// `vizd_e2e_test::a_locally_attached_mirror_is_monitored_under_its_robot_e2e`
/// (`vizd_e2e_test.rs:11201`). It registers a REAL mirror provenance record, converges
/// the cache so the attach cannot race it, attaches over the LOCAL path (no `robot` in
/// the request), and requires the reply's `robot` to read `Some("ubuntu")`.
/// A variant that keeps the call, discards its answer and replies `robot: None` fails
/// exactly that arm — so the production contract does not rest on the structural guard
/// below, which is the right division of labour rather than a gap in it.
#[test]
fn monitors_v1_a_name_keyed_claim_reaches_a_tap_only_through_the_refresher() {
    let code = code_containing("fn refresh_hinted_origin_robots(&mut self");
    // The refresher reads the cache FIELD and never fetches — it is on the poll
    // thread, where a MIRROR_GATHER_WINDOW would be a slow daemon, not a slow
    // monitor.
    let refresher = fn_body(&code, "fn refresh_hinted_origin_robots(&mut self");
    assert!(
        refresher.contains("mirror_provenance"),
        "the refresher must read the provenance CACHE:\n{refresher}"
    );
    for gathering in ["gather_mirror_provenance", "attribution_snapshot"] {
        assert!(
            !refresher.contains(gathering),
            "the refresher reaches `{gathering}`, which FETCHES (up to \
             MIRROR_GATHER_WINDOW) — on the poll thread every \
             MONITOR_SAMPLE_INTERVAL_NS:\n{refresher}"
        );
    }
    // It must also be able to take a claim BACK: an `Option` target, so a topic the
    // map no longer names is CLEARED rather than left behind.
    assert!(
        refresher.contains("set_origin_robot"),
        "and it must write through the one writer, which releases the row the old \
         identity opened:\n{refresher}"
    );

    // THE ATTACH SEAMS MAY NOT WRITE THE MAP ONTO A TAP, AND MAY NOT PAY A BLOCKING
    // GATHER. The forbidden-FETCH set is per seam, because the two seams genuinely
    // differ — a single list would either miss `compose_layout`'s rule or fail on
    // `attach_local`'s documented one.
    //
    // The enumeration is taken from the registry's whole read surface rather than
    // guessed. In vizd's `src/` a provenance FETCH is reachable by exactly two
    // names: `Ctx::gather_mirror_provenance` (which wraps
    // `TransportManager::gather_mirror_provenance{,_checked}` — the `_checked` twin
    // carries the bare name as a prefix, so one needle covers both), and
    // `Ctx::attribution_snapshot`, which calls it and then merges the authoritative
    // held half. Everything else in that API is pure (`merge_attribution`), a WRITE
    // (`{,un}register_mirror_provenance`, called nowhere in vizd) or a cache read
    // (the `mirror_provenance` FIELD, which is what the refresher above uses).
    //
    // Each seam carries its OWN `why`, because a shared sentence would be wrong for
    // one of them: compose's regression is a gather whose answer is thrown away,
    // attach_local's is a gather that BYPASSES the merge. Both are forbidden here,
    // for reasons that are not the same reason.
    for (seam, forbidden_fetches, why) in [
        (
            "fn attach_local(&self",
            // `attribution_snapshot` is LEGITIMATE here and deliberately not listed:
            // `attach_local` computes it for its REPLY's `robot` field
            // — a per-call answer
            // thrown away, never written onto the tap. It is also load-bearing to the
            // convergence bound `refresh_hinted_origin_robots`' own doc states, which
            // names `attach_local` among the verbs whose gather REPLACES the cached
            // snapshot a wrong hint survives on. What is forbidden is reaching the RAW
            // map underneath it: that half is name-keyed with no authoritative merge,
            // so a reply built from it loses the robot of a tap whose mirror netd tore
            // down under it, the attribution flicker `attribution_snapshot`'s held half
            // exists to prevent.
            &["gather_mirror_provenance"][..],
            "that is the RAW name-keyed map, reached UNDER the merge. \
             `attribution_snapshot` is the legitimate call here (it computes the \
             reply's `robot`, and it is one of the gathering verbs the wrong-hint \
             convergence bound is stated against) — what it adds is the AUTHORITATIVE \
             held half, without which the reply loses the robot of a tap whose mirror \
             netd tore down under it, which is the attribution flicker that half exists \
             to prevent",
        ),
        (
            "fn compose_layout(&self",
            // Compose consults the map at no depth. A later change found that the stamps
            // bought nothing (a monitor row is created BY the sampler, so there was
            // never a window in which a row existed without a sampling pass having
            // run), which leaves the cost argument standing alone: this handler
            // already spends up to COMPOSE_PEEK_BUDGET resolving silent topics, and a
            // MIRROR_GATHER_WINDOW on top pushes a large layout toward viz_client's 5 s
            // read deadline — a slow compose is one thing, a DEAD one is another.
            // `attribution_snapshot` is a substring of the deleted
            // `cached_attribution_snapshot`, so re-introducing even the read-only
            // accessor here has to update this guard deliberately. That is the point.
            &["gather_mirror_provenance", "attribution_snapshot"][..],
            "this handler consults the map at NO depth, so the answer is DISCARDED \
             and the writer assertions above cannot see the call at all — it stamps \
             no robot and passes every one of them. Compose already spends up to \
             COMPOSE_PEEK_BUDGET resolving silent topics; a MIRROR_GATHER_WINDOW on \
             top pushes a large layout toward viz_client's 5 s read deadline, and a \
             slow compose is one thing while a DEAD one is another",
        ),
    ] {
        let body = fn_body(&code, seam);
        for writer in ["set_origin_robot", "retarget_origin_robot"] {
            assert!(
                !body.contains(writer),
                "`{seam}` writes a tap's robot via `{writer}`. A name-keyed claim \
                 recorded at attach time cannot be taken back, which is how a \
                 retired-then-reused topic keeps a robot it never had. Let the \
                 refresher maintain it.\n\n{body}"
            );
        }
        for fetch in forbidden_fetches {
            assert!(
                !body.contains(fetch),
                "`{seam}` reaches `{fetch}`, which FETCHES on a TTL miss (up to \
                 MIRROR_GATHER_WINDOW, ~600 ms) — and {why}. Behaviourally this is \
                 invisible either way: on a cache HIT the call answers instantly from \
                 the same map, so the only difference is wall time on a handler whose \
                 budget is already spoken for. That is the promptness \
                 contract.\n\n{body}"
            );
        }
    }
    // ANTI-TAUTOLOGY, twice over — every rule above is a NEGATIVE, so each needs a
    // positive beside it or a daemon that had stopped attributing anything at all
    // would satisfy the lot.
    //
    // (1) The AUTHORITATIVE seam still records what it DEMANDED.
    let remote = fn_body(&code, "fn attach_remote(");
    assert!(
        remote.contains("retarget_origin_robot"),
        "the REMOTE attach path records the robot it demanded from — that record is \
         first-hand and the refresher must not revise it:\n{remote}"
    );
    // (2) …and `attach_local` still ANSWERS the attribution question for its reply.
    // Without this, the forbidden-fetch list above reads as "attach_local must not
    // gather", which is the opposite of the shipped contract: dropping the call would
    // both blank the reply's `robot` and remove one of the four verbs whose
    // gather is what lets a wrong hint converge at all.
    //
    // Checked where the value LANDS, not merely that the body mentions the call —
    // the same lesson `monitors_v1_the_served_ring_is_stamped_through_the_one_age_helper`
    // records three tests above, and THIS arm can repeat the mistake
    // it warns about: `let _ = self.attribution_snapshot(); let robot = None;` keeps
    // the token and answers nothing, and a `contains` over the body passes it.
    //
    // `contains` on the RESOLVED value rather than `is_call_to`, and the difference
    // from the `alerts` case is deliberate: there, ANY trailing transformation can
    // serve an empty list from a correct call, so the call must be the whole
    // expression. Here the trailing `.get(&topic).cloned()` IS the required lookup,
    // so demanding a bare call would forbid the shipped code. The claim this makes
    // is DERIVATION — the robot the reply carries is computed from the merged
    // snapshot — and its limit is that it cannot tell that lookup from a
    // wrong one. That half is behavioural, and it is pinned: see the doc above.
    let local = fn_body(&code, "fn attach_local(&self");
    let open = local
        .find("AttachResponse {")
        .map(|at| at + "AttachResponse ".len())
        .expect("`attach_local` builds an AttachResponse");
    let literal = block_at(local, open).expect("the reply literal is balanced");
    let robot = struct_field_value(local, literal, "robot");
    assert!(
        robot.contains("attribution_snapshot"),
        "the attach reply's `robot` must be COMPUTED from the merged snapshot — that \
         per-call answer is the attribution rule's, and `attribution_snapshot` is also one of the \
         gathering verbs the hint-refresh convergence bound is stated against. A call \
         whose result is discarded beside a `None` robot satisfies a body-wide \
         `contains` and answers nothing.\n\nrobot: {robot}\nliteral: {literal}"
    );
}

/// The monitor plane never reaches the network, on any verb.
///
/// The cost claim is "zero netd round trips", and the whole reason the attached
/// plane reads `TopicStat` is that everything it needs is already computed. The
/// tempting regression is `attribution_snapshot` — it is the map every OTHER vizd
/// surface uses for exactly this attribution question, it is one call away, and it
/// pays a 150-600 ms `/__cerulion/mirrors` gather to answer. On the poll thread
/// every 400 ms that is not a slow monitor, it is a slow DAEMON, and no assertion
/// about alerts would notice.
#[test]
fn monitors_v1_the_plane_pays_no_gather_and_opens_no_connection() {
    let code = code_containing("fn sample_monitors(&self");
    for body_of in [
        "fn sample_monitors(&self",
        "fn monitor_snapshot(",
        "fn monitors(&self",
    ] {
        let body = fn_body(&code, body_of);
        for forbidden in [
            "attribution_snapshot",
            "gather_mirror_provenance",
            "registration_snapshot",
            "demand_plane",
            "list_topics",
        ] {
            assert!(
                !body.contains(forbidden),
                "`{body_of}` reaches `{forbidden}` — the monitor plane is a pure \
                 consumer of values the drain loop already computed, and this one \
                 costs a gather or a round trip on the poll thread every \
                 MONITOR_SAMPLE_INTERVAL_NS.\n\nbody:\n{body}"
            );
        }
    }
    // ANTI-TAUTOLOGY: the snapshot really does read the per-tap origin, which is
    // the attribution it uses INSTEAD — otherwise every negative above would also
    // pass a builder that attributed nothing at all.
    let snapshot = fn_body(&code, "fn monitor_snapshot(");
    assert!(
        snapshot.contains("origin_robot"),
        "and it really does key rows on the TAP's own origin:\n{snapshot}"
    );
}

/// Monitors: the DISCOVERY plane's feed reaches the engine from a
/// CONTROLLER thread, and it must never hold the state lock across the network.
///
/// This is the feed's one real hazard, and it is why the plane has no leaf mutex
/// of its own (the `discover` handler must reach the same engine from a
/// controller thread anyway). The tempting shape is the obvious one: `discover`
/// has a `demand_plane` in scope and a gather is one call away, so a feed written
/// inside the lock — or worse, one that re-gathers to get "fresh" catalogs —
/// reads perfectly naturally. The poll thread takes that same lock on EVERY 16 ms
/// pass, so a lock held across a 150-600 ms netd round trip does not slow the
/// monitor down, it stalls the daemon's whole drain. No alert oracle would see it.
///
/// The seam is that the feed takes the catalogs BY REFERENCE and holds no query
/// plane at all; this arm is what keeps that true.
#[test]
fn monitors_v1_the_catalog_feed_pays_no_gather_and_opens_no_connection() {
    let code = code_containing("fn observe_catalog_monitors(");
    for body_of in [
        "fn observe_catalog_monitors(",
        "fn catalog_monitor_samples(",
        "fn attached_row_keys(",
    ] {
        let body = fn_body(&code, body_of);
        for forbidden in [
            "gather_catalog",
            "query_catalog",
            "query_schema",
            "demand_plane",
            "with_client",
            "attribution_snapshot",
            "gather_mirror_provenance",
            "list_topics",
            "ConvergenceWait",
            "loop",
            "while ",
            "sleep",
        ] {
            assert!(
                !body.contains(forbidden),
                "`{body_of}` reaches `{forbidden}` — the discovery plane is a pure \
                 consumer of a gather `discover` has ALREADY made, and this one \
                 would put the network inside a lock the drain loop takes every \
                 16 ms.\n\nbody:\n{body}"
            );
        }
    }
    // ANTI-TAUTOLOGY: it really does reach the engine. Every negative above is
    // also satisfied by a feed that does nothing at all, which is precisely the
    // shape this chunk ships dead in.
    let feed = fn_body(&code, "fn observe_catalog_monitors(");
    assert!(
        feed.contains("observe_catalog_pass"),
        "and it really does drive the engine:\n{feed}"
    );
    let builder = fn_body(&code, "fn catalog_monitor_samples(");
    assert!(
        builder.contains("entry.liveness"),
        "…from the ROBOT's own observation, which is the only evidence an \
         UNATTACHED row can have:\n{builder}"
    );
}

/// The catalog feed answers from ONE state-lock acquisition, and folds its
/// counters outside it.
///
/// Same rule as the sampler's, plus one of its own: the ATTACHED-key set is what
/// protects a tapped row from retirement, so reading it in a separate acquisition
/// would let an `attach` land between the read and the pass and have the very next
/// gather retire the row it just opened. `MonitorPlane::forget` drops a row
/// whoever owns it, so that loss is the ATTACHED plane's baseline — silently, on
/// the one row a user is looking at. It is a race, so no deterministic arm can
/// catch it; the shape is what is checkable.
#[test]
fn monitors_v1_the_catalog_feed_takes_the_state_lock_exactly_once() {
    let code = code_containing("fn observe_catalog_monitors(");
    let body = fn_body(&code, "fn observe_catalog_monitors(");
    let locks = body.matches("self.state.lock()").count();
    assert_eq!(
        locks, 1,
        "the attached-key read, the sample build and the engine pass must share \
         ONE acquisition:\n{body}"
    );
    let (_, scope_open, scope_end) = lock_scope(body, "observe_catalog_monitors");
    // The DELTAS are folded OUTSIDE the lock — a relaxed `fetch_add` has no
    // business running under the lock the drain needs every pass.
    for counter in [
        "monitor_catalog_samples_taken",
        "monitor_catalog_rows_retired",
        "monitor_alerts_raised",
        "monitor_alerts_cleared",
    ] {
        let at = body.find(counter).unwrap_or_else(|| {
            panic!("`{counter}` is folded in observe_catalog_monitors:\n{body}")
        });
        assert!(
            at > scope_end,
            "`{counter}` must be bumped OUTSIDE the lock scope:\n{body}"
        );
    }
    // …and the GAUGE is published INSIDE it. The split is not stylistic. A
    // `fetch_add` is COMMUTATIVE, so two passes may land in either order and the
    // total is the same; a `store` is LATEST-WINS, and "latest" has to be decided
    // by the same lock that decided the value. Computing under the lock and
    // storing after it lets an older pass's store land behind a newer pass's, so
    // the gauge describes a row set that is no longer current — MEASURED by
    // executing the interleaving and watching a 2 overwrite a 1.
    let stores = store_sites(body, "monitor_rows_ineligible");
    assert_eq!(stores.len(), 1, "exactly ONE gauge publication:\n{body}");
    assert!(
        stores[0] > scope_open && stores[0] < scope_end,
        "the `monitor_rows_ineligible` GAUGE must be STORED inside the lock scope \
         that produced it — a latest-wins store ordered outside its own lock can \
         publish an older pass's value over a newer one. An identifier-PRESENCE \
         check does not say this: a body that reads the counter under the lock \
         and stores after satisfies it:\n{body}"
    );
    // The two planes must NOT share a samples counter. Each one is its own plane's
    // no-inert-shipping observable, so a catalog pass bumping the sampler's would
    // let a daemon that deleted the drain loop's call site keep that number
    // climbing — and the guard that says the sampler is wired would be satisfied
    // by the OTHER plane running.
    //
    // The needle is the BARE identifier. Spelling it
    // `monitor_samples_taken.` to mean "a use, not a mention" fails — rustfmt puts
    // the `.` of a method chain on the NEXT line, so that needle cannot match
    // the very shape it is written to catch: a change that adds
    // `self.monitor_samples_taken.fetch_add(..)` here passes that assertion.
    assert!(
        !body.contains("monitor_samples_taken"),
        "the catalog feed must not touch the DRAIN LOOP's sample counter — that \
         number is the sampler's own no-inert-shipping signal, and a second \
         writer would let a daemon that deleted the sampler's call site keep it \
         climbing:\n{body}"
    );
    // A THROTTLED pass must fold NOTHING. `observe_catalog_pass` answers `None`
    // for one, and the type makes that hard to ignore — but `unwrap_or_default()`
    // is one call away and reads like a tidy-up. It is not: `monitor_rows_ineligible`
    // is a GAUGE, so a zeroed outcome STORES a positive "nothing is being withheld
    // from the agent right now", made by a pass that looked at nothing. The three
    // deltas beside it would add zero and hide the mistake.
    assert!(
        !body.contains("unwrap_or_default") && !body.contains("unwrap_or("),
        "a dropped catalog pass must be RETURNED FROM, never defaulted — the \
         `rows_ineligible` gauge is overwritten by whatever is folded, so a zeroed \
         outcome is a false claim rather than a no-op:\n{body}"
    );
    assert!(
        body.contains("let Some(outcome)"),
        "…and the guard is the destructuring that returns early:\n{body}"
    );
}

/// The catalog feed is driven from `discover`, from nowhere else, and it is handed
/// THAT gather's own catalogs and THAT gather's own discovery marker.
///
/// Both halves matter and they fail differently.
///
/// The single-call-site half is the piggyback rule: this plane piggybacks on the gather
/// Studio already makes, and a second caller is a second LAN harvest on a
/// stateless per-machine daemon — the standing self-refresh the plane explicitly
/// fences out, arriving by accident.
///
/// The arguments half is the ORDERING proof, and it is structural rather than
/// textual: `gather.catalogs` cannot be named before the match arm binds it, so a
/// call passing it provably runs after the round trip returned. Passing
/// `catalog_discovery` is the evidence half — the engine suppresses every condition
/// on an unconverged plane, and a hardcoded `true` here would
/// make every cold-start `discover` feed rows whose emptiness is an artefact of
/// the gather.
#[test]
fn monitors_v1_the_catalog_feed_is_driven_from_discover_with_that_gathers_own_answer() {
    let code = code_containing("fn observe_catalog_monitors(");
    let discover = fn_body(&code, "fn discover(&self");
    let at = discover
        .find("self.observe_catalog_monitors(")
        .unwrap_or_else(|| panic!("`discover` must feed the discovery plane:\n{discover}"));
    let args = &discover[at..];
    let end = args.find(");").expect("the call closes");
    let args = &args[..end];
    // EXACT arguments, not `contains`. A `contains` accepts an expression that
    // merely MENTIONS the marker while computing a constant —
    // `if true { Settled } else { catalog_discovery }` names it and gates nothing.
    // The behavioural arms below are the primary defense (they kill the plain
    // hardcode outright), but a guard whose whole job is the SHAPE should not be
    // satisfiable by a shape it was written to reject.
    let args: Vec<&str> = args
        .trim_start_matches("self.observe_catalog_monitors(")
        .split(',')
        .map(str::trim)
        .filter(|a| !a.is_empty())
        .collect();
    assert_eq!(
        args,
        vec!["&gather.catalogs", "catalog_discovery", "gather_gen"],
        "the feed must be handed, VERBATIM: THIS gather's own catalogs (naming \
         them is what proves it runs after the round trip — re-gathering would be \
         a second LAN harvest), THIS gather's own discovery marker (never a \
         hardcoded `Settled`: an unconverged catalog proves nothing about any \
         topic), and THIS gather's own GENERATION (taken before the round trip, so \
         a slow gather that hands over last cannot apply its older answer)"
    );

    let mut call_sites = 0usize;
    for f in src_files() {
        call_sites += code_of(&f).matches("observe_catalog_monitors").count();
    }
    assert_eq!(
        call_sites, 2,
        "expected exactly the `fn observe_catalog_monitors` definition and ONE call \
         (in `discover`), found {call_sites} mentions across src/ — a second caller \
         is a second netd harvest, which is the standing self-refresh the plane \
         fences out"
    );
}

/// Every engine WRITE dates through the LATCHED clock; only the SERVING handler
/// reads the unlatched one.
///
/// The type already forbids the inversion — `WriteStamp`'s field is private to
/// `monitors.rs` and `advance_to` is its only constructor, so a write dated from
/// `now_ns` does not compile — and that is the real enforcement. What this arm
/// adds is the other direction, which the type cannot express: that the READ path
/// stays unlatched. A `monitors` handler switched to `advance_to` would compile
/// perfectly and would let a client's polling cadence decide where the next
/// confirmation span is anchored — the "cadence depends on who is asking" defect
/// the sampler's interval gate exists to prevent, arriving through the read path.
#[test]
fn monitors_v1_the_writers_latch_the_clock_and_the_reader_does_not() {
    let code = code_containing("fn observe_catalog_monitors(");
    for writer in ["fn sample_monitors(&self", "fn observe_catalog_monitors("] {
        let body = fn_body(&code, writer);
        assert!(
            body.contains("advance_to("),
            "`{writer}` is a WRITER and must date through the latched clock:\n{body}"
        );
    }
    let handler = fn_body(&code, "fn monitors(&self");
    assert!(
        !handler.contains("advance_to("),
        "the `monitors` handler must NOT latch — serving a response would then \
         advance the writers' number line, and a client's polling cadence would \
         decide where the next confirmation span is anchored:\n{handler}"
    );
    // ANTI-TAUTOLOGY: the handler really does date its response, so the negative
    // above is about WHICH clock rather than about a handler that reads none.
    assert!(
        handler.contains("now_ns("),
        "…while still dating its rows and alert ages from the read clock:\n{handler}"
    );
}

// ===========================================================================
// The `runs` verb
// ===========================================================================

/// The `runs` handler takes NO state lock, and never waits.
///
/// # Why the lock half is structural and not merely tidy
///
/// Both of this handler's arms are GATHERS — a bounded listen on the local run
/// registry, and one netd round trip — and the poll thread needs the state lock
/// every `DEFAULT_POLL_INTERVAL` to drain every tap. A lock held across either
/// gather stalls the daemon's whole DATA path to answer a control question, which
/// is the hazard `observe_catalog_monitors` documents at length and the reason the
/// monitors sampler is pinned to one acquisition. This handler needs nothing out of
/// `DaemonState`, so the right bar is ZERO — and zero is a bar a behavioural arm
/// cannot check at all, because the cost is a stall nobody asserts on.
///
/// The wait half is the contract every vizd handler is held to: a scripted
/// `DemandPlane` answers instantly, so a handler that looped would look identical in
/// every e2e arm while being fatal against a real netd through `viz_client`'s 5 s
/// reply deadline.
#[test]
fn the_runs_handler_takes_no_state_lock_and_never_waits() {
    let code = code_containing("fn runs(&self, id: u64");
    let body = fn_body(&code, "fn runs(&self, id: u64");

    let locks = body.matches("self.state.lock()").count();
    assert_eq!(
        locks, 0,
        "`runs` must take the state lock ZERO times — it reads nothing from \
         `DaemonState`, and both of its arms gather, so any acquisition here is held \
         across a gather the poll thread is waiting behind:\n{body}"
    );
    for forbidden in ["loop", "while ", "ConvergenceWait"] {
        assert!(
            !body.contains(forbidden),
            "`runs` contains `{forbidden}` — this handler gathers ONCE per arm and \
             answers. A wait is unreachable through viz_client's 5 s reply deadline; \
             report the facts (`completeness`, `discovery`, `unusable`, `degraded`) \
             and let the client decide.\n\nbody:\n{body}"
        );
    }
    // `sleep` is handled apart, and NOT dropped: this handler carries exactly one,
    // the test-only local-arm delay, whose production value is `0`. So
    // the guard is the STRONGER claim rather than an exemption — every `sleep` in
    // the body must be GOVERNED by that injected span being non-zero, which a real
    // wait (unconditional, or conditioned on anything else) cannot satisfy.
    let sleeps = body.matches("sleep").count();
    assert_eq!(
        sleeps, 1,
        "`runs` must carry exactly ONE `sleep` — the test-only injected local-arm \
         delay. A second one is a wait, and a wait here is unreachable through \
         viz_client's 5 s reply deadline:\n{body}"
    );
    let gate = governing_condition(body, "sleep");
    assert!(
        gate.contains("injected"),
        "…and that `sleep` must be governed by the INJECTED delay (`0` in \
         production — nothing in the daemon writes it), not by any other \
         condition. Its gate reads `{gate}`:\n{body}"
    );
    // ANTI-TAUTOLOGY: it really does run both arms (a body that did nothing would
    // satisfy every negative above).
    assert!(
        body.contains("gather_live_runs(") && body.contains("query_runs("),
        "and it really does gather BOTH halves:\n{body}"
    );
}

/// The LOCAL arm asks THIS manager's namespace, never the process-global one.
///
/// `gather_current_runs()` reads the global iceoryx2 config; `gather_live_runs`
/// reads the manager's own. On a developer's desk they usually agree, which is
/// exactly what makes the wrong one easy to write and invisible afterwards — and on
/// any isolated SHM root (every test, and every multi-process deployment) the global
/// namespace is a DIFFERENT MACHINE.
///
/// Behaviourally pinned by
/// `vizd_e2e_test::a_live_local_run_is_gathered_from_this_managers_namespace_e2e`,
/// which publishes a real run on an isolated root; guarded here too because this is
/// what a reader grepping for the gather finds, and because the guard follows the
/// code if the handler moves modules.
#[test]
fn the_local_runs_arm_asks_this_managers_namespace() {
    let code = code_containing("fn runs(&self, id: u64");
    assert!(
        !code.contains("gather_current_runs"),
        "nothing in this module may reach for the process-GLOBAL run gather — it \
         answers about a different machine on every isolated SHM root"
    );
    let body = fn_body(&code, "fn runs(&self, id: u64");
    assert!(
        body.contains("self.manager.gather_live_runs("),
        "…and the local arm asks the MANAGER's own namespace:\n{body}"
    );
}

/// The remote arm goes through the `DemandPlane`, never through a `NetdClient` of
/// its own.
///
/// One connection per daemon is the whole point of the plane (netd sees vizd as ONE
/// consumer, and the client mutex is taken and dropped inside `with_client`). A
/// handler that opened its own client would bypass that, and — since it would still
/// work — nothing behavioural would notice.
#[test]
fn the_remote_runs_arm_goes_through_the_demand_plane() {
    let code = code_containing("fn runs(&self, id: u64");
    let body = fn_body(&code, "fn runs(&self, id: u64");
    assert!(
        body.contains("self.demand_plane.query_runs("),
        "the remote arm must ride the ONE shared plane:\n{body}"
    );
    assert!(
        !body.contains("NetdClient"),
        "…and must not mint a client of its own:\n{body}"
    );
}

/// The handler never hands the fold a robot name for a LOCAL run.
///
/// The fold's own type forbids the phantom row (`RunsOrigin` has no
/// `robot: Option<String>` to fill in wrongly, and `wire()` is pinned purely), so
/// what is left for a handler to get wrong is upstream of it: passing the desk's
/// hostname as the local arm's identity. The daemon has one to hand — `discover`
/// reads attribution maps full of them — so this asserts the local arm is built from
/// the run registry alone.
#[test]
fn the_local_runs_arm_names_no_robot() {
    let code = code_containing("fn runs(&self, id: u64");
    let body = fn_body(&code, "fn runs(&self, id: u64");
    for forbidden in ["hostname", "robot_identity", "attribution_snapshot"] {
        assert!(
            !body.contains(forbidden),
            "`runs` reaches for `{forbidden}` — a LOCAL run is attributed to NO robot, \
             and stamping this desk's own identity would mint the phantom \
             'this machine' row in the one list whose purpose is saying WHERE \
             something runs:\n{body}"
        );
    }
    // ANTI-TAUTOLOGY: the local arm is built, and from the registry fold.
    assert!(
        body.contains("local_reply(") && body.contains("collect_run_entries("),
        "…while still building the local reply from the registry's own fold:\n{body}"
    );
}

/// An UNUSABLE robot is reported LOUDLY, ONCE PER ROBOT, and its line names the
/// robot and the redeploy.
///
/// The counters this desk keeps are per-run, not per-robot, so nothing else on the
/// machine will ever notice a robot whose binary has drifted: the log is the whole
/// window. Pinned structurally because the arm that drives it e2e asserts the WIRE,
/// and a loud line is a separate obligation from a carried field.
///
/// # Why the assertion is scoped to the LOOP BODY
///
/// The first version found `fold.unusable` and asked whether ANY `warn!` appeared
/// after it — which is satisfied by a handler that dropped the per-robot line
/// entirely (the netd-unreachable `warn!` sits further down the same function and
/// would answer for it), and by one that emitted something unrelated. Both are the
/// regression: an operator greps for the ROBOT, and a line that names no robot
/// cannot tell them WHICH machine to redeploy. So the guard brace-matches the
/// iteration over `fold.unusable` and asserts INSIDE it — the same lesson the
/// lock-scope guard above records, applied to a loop rather than a block.
#[test]
fn an_unusable_robot_is_named_in_a_loud_line() {
    let code = code_containing("fn runs(&self, id: u64");
    let body = fn_body(&code, "fn runs(&self, id: u64");
    let loop_body = loop_body_over(body, "fold.unusable");

    // The report must be LOUD, and the robot must be named BY THAT SAME CALL. Two
    // independent `contains` checks over the loop body are satisfied by a `warn!`
    // that names nothing beside a `debug!` that does — exactly the split an
    // operator cannot grep.
    let warn = macro_invocation(loop_body, "warn");
    assert!(
        warn.contains("robot ="),
        "the per-robot report must be LOUD **and** NAME the robot in the SAME \
         invocation — nothing else on this desk observes a version-skewed robot, so \
         the log is the only window, and a line reporting only that SOMETHING was \
         unusable cannot say which machine to redeploy:\n{warn}"
    );

    // The REMEDY is in the message TEXT, which `code_only` blanks — so this one
    // assertion reads the RAW source. Scoping it to the INVOCATION rather than to
    // the loop is what keeps the claim tight: the span ends at
    // the call's closing paren, so a comment ELSEWHERE in the loop can no longer
    // satisfy it. (A comment INSIDE the argument list still could; that is a
    // narrower hole than "anywhere in the loop", and the same span is what carries
    // the message literal, so there is nowhere tighter to look.)
    let raw = std::fs::read_to_string(
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("daemon.rs"),
    )
    .expect("read daemon.rs");
    let raw_warn = macro_invocation(loop_body_over(&raw, "fold.unusable"), "tracing::warn");
    assert!(
        raw_warn.contains("REDEPLOY"),
        "…and it must name the REMEDY. Such a robot ANSWERED and will re-serve the \
         same undecodable bytes forever, so the one thing this line must not read \
         as is 'keep waiting':\n{raw_warn}"
    );
}

/// The SILENT robots — the coverage half — are reported per robot, and NOT at
/// `warn!`.
///
/// The level is the whole point of this arm. A silent robot is reachable on a
/// perfectly healthy LAN — a binary predating the `runs` verb, or a Strict
/// ingress-only gateway that declares no query surface — and it answers nothing
/// FOREVER, so a `warn!` here fires on every poll for the life of the desk: the
/// disk-fill flood class, arriving through a line that describes a supported
/// configuration. The response CARRIES the list, which is where a renderer reads
/// it; the log is a diagnostic, so it is `debug!`.
#[test]
fn a_silent_robot_is_reported_per_robot_and_not_at_warn() {
    let code = code_containing("fn runs(&self, id: u64");
    let body = fn_body(&code, "fn runs(&self, id: u64");
    let loop_body = loop_body_over(body, "fold.silent");

    // Same discipline as its `unusable` sibling: the name and the LEVEL are one
    // claim about ONE call. Checked separately over the loop body, a `debug!` that
    // names the robot beside a `warn!` that does not satisfies both halves while
    // being precisely the flood this arm exists to forbid.
    let debug = macro_invocation(loop_body, "debug");
    assert!(
        debug.contains("robot ="),
        "the silent robots must be named individually — a count cannot be acted \
         on:\n{debug}"
    );
    assert!(
        !loop_body.contains("warn!"),
        "…at `debug!`, never `warn!`: a robot predating the verb (or serving no \
         query surface) is a SUPPORTED configuration that answers nothing forever, \
         so a warn here floods for the life of the desk — the disk-fill \
         class:\n{loop_body}"
    );
}
