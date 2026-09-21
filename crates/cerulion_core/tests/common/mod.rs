// SPDX-License-Identifier: AGPL-3.0-only
//! Shared source-walk helpers for the structural gates in this directory.
//!
//! ONE copy, deliberately. `config_deny_unknown_fields_test.rs` and
//! `serial_discipline_test.rs` both strip comments before searching Rust
//! source, and both were carrying their own transcription of the same
//! state machine. That is how [`code_only`]'s string-literal blindness came
//! to exist in two places at once and be fixed in neither.
//!
//! `tests/common/mod.rs` is a MODULE, not a test target — cargo only builds
//! top-level `tests/*.rs` as binaries — so this file adds no test binary. Its
//! `#[test]` arms run inside every binary that declares `mod common;`.

/// A view of `src` with `//`-to-end-of-line and `/* … */` block comments
/// removed, newlines preserved so reported line numbers stay meaningful.
///
/// STRING AND CHAR LITERALS ARE MODELLED, and that is the whole point.
/// A comment-only stripper treats the `/*` inside `"nodes/*"` as opening a
/// block comment, and because Rust block comments NEST and this one never
/// closes, everything after it is swallowed — the walk then sees an empty
/// file and reports a CLEAN SHEET. It fails OPEN, on real code: a glob, a
/// path pattern or a doc example carrying `/*` silently disables every check
/// below it. (The guard hit the same hazard and dodged it by
/// excluding itself, which is not available to a walk over production
/// source.)
///
/// Literal CONTENT is preserved, not blanked. Callers depend on it — the
/// legal-key readers parse `&["a", "b"]` slices out of source, and the warn
/// reader matches a `section = "<top-level>"` argument. Only the comment
/// SCANNER is taught to skip literals; nothing is removed from them.
///
/// Modelled: ordinary `"…"` with `\`-escapes, raw `r"…"` / `r#"…"#` /
/// `br#"…"#` with matching hash counts, and `'x'` / `'\n'` char literals —
/// with lifetimes (`'static`, `'a>`) deliberately NOT treated as literals,
/// which is the case a naive quote-counter gets wrong.
///
/// An unterminated block comment still swallows the rest, which fails CLOSED
/// (the walk sees less code, never more).
pub fn code_only(src: &str) -> String {
    strip(src, false)
}

/// [`code_only`], but with string and char literal CONTENT blanked to spaces.
///
/// The two views answer different questions, and the difference is the whole
/// reason this one exists. `code_only` preserves literals because its callers
/// parse data out of them. A walk asking "does this file CALL X?" must not read
/// a file that merely NAMES `X` in a string, and three files in
/// `cerulion_core/tests` do exactly that: `serial_discipline_test.rs` holds the
/// singleton accessors as its watch list, and `error_message_test.rs` asserts
/// that an error message mentions `TransportManager::init()`. Under
/// `code_only` all three read as callers.
///
/// Blanking rather than DELETING keeps LINE NUMBERS intact, so a diagnostic
/// computed over this view still names the right line. The delimiters survive
/// for the same reason — only what is BETWEEN them is spaced out.
///
/// Byte offsets are NOT preserved and must not be relied on: a multi-byte
/// character inside a literal blanks to a one-byte space. What holds is the
/// CHARACTER count (which is what this module's oracle asserts) and the line
/// count. Offsets are in any case relative to this view, not to the real file,
/// since both views delete comments outright.
pub fn code_only_blank_literals(src: &str) -> String {
    strip(src, true)
}

fn strip(src: &str, blank_literals: bool) -> String {
    #[derive(Clone, Copy)]
    enum St {
        Code,
        Line,
        Block(usize),
        Str,
        Raw(usize),
        Ch,
    }
    let chars: Vec<char> = src.chars().collect();
    let mut out = String::with_capacity(src.len());
    let mut st = St::Code;
    let mut i = 0usize;
    while i < chars.len() {
        let c = chars[i];
        match st {
            St::Line => {
                if c == '\n' {
                    out.push('\n');
                    st = St::Code;
                }
                i += 1;
            }
            St::Block(d) => {
                if c == '/' && chars.get(i + 1) == Some(&'*') {
                    st = St::Block(d + 1);
                    i += 2;
                } else if c == '*' && chars.get(i + 1) == Some(&'/') {
                    st = if d == 1 { St::Code } else { St::Block(d - 1) };
                    i += 2;
                } else {
                    if c == '\n' {
                        out.push('\n');
                    }
                    i += 1;
                }
            }
            St::Str | St::Ch => {
                let closer = if matches!(st, St::Str) { '"' } else { '\'' };
                // The CLOSER is structure and always survives; everything else
                // is content, blanked under `blank_literals` so the position of
                // every later byte is preserved. A newline inside a literal is
                // kept as a newline for the same reason.
                let emit = |out: &mut String, ch: char, is_delim: bool| {
                    if !blank_literals || is_delim {
                        out.push(ch);
                    } else if ch == '\n' {
                        out.push('\n');
                    } else {
                        out.push(' ');
                    }
                };
                if c == '\\' {
                    emit(&mut out, c, false);
                    if let Some(&n) = chars.get(i + 1) {
                        emit(&mut out, n, false);
                    }
                    i += 2;
                } else {
                    let is_close = c == closer;
                    emit(&mut out, c, is_close);
                    if is_close {
                        st = St::Code;
                    }
                    i += 1;
                }
            }
            St::Raw(h) => {
                if c == '"' {
                    let mut k = 0usize;
                    while chars.get(i + 1 + k) == Some(&'#') {
                        k += 1;
                    }
                    if k >= h {
                        out.push('"');
                        for _ in 0..h {
                            out.push('#');
                        }
                        st = St::Code;
                        i += 1 + h;
                        continue;
                    }
                }
                if !blank_literals {
                    out.push(c);
                } else if c == '\n' {
                    out.push('\n');
                } else {
                    out.push(' ');
                }
                i += 1;
            }
            St::Code => {
                if c == '/' && chars.get(i + 1) == Some(&'/') {
                    st = St::Line;
                    i += 2;
                } else if c == '/' && chars.get(i + 1) == Some(&'*') {
                    st = St::Block(1);
                    i += 2;
                } else if c == '"' {
                    out.push(c);
                    st = St::Str;
                    i += 1;
                } else if c == '\'' {
                    // `'x'` and `'\n'` are literals; `'static` / `'a>` are
                    // LIFETIMES and must stay ordinary code, or the next
                    // apostrophe in the file would close a phantom literal.
                    let is_literal =
                        chars.get(i + 2) == Some(&'\'') || chars.get(i + 1) == Some(&'\\');
                    out.push(c);
                    if is_literal {
                        st = St::Ch;
                    }
                    i += 1;
                } else if (c == 'r' || c == 'b') && !prev_is_ident(&chars, i) {
                    // A raw-string prefix must START a token, or the `r` in
                    // `for` / `substr` would open one.
                    let mut j = i;
                    if c == 'b' && chars.get(j + 1) == Some(&'r') {
                        j += 1;
                    }
                    if chars.get(j) == Some(&'r') {
                        let mut k = 0usize;
                        while chars.get(j + 1 + k) == Some(&'#') {
                            k += 1;
                        }
                        if chars.get(j + 1 + k) == Some(&'"') {
                            out.extend(chars[i..=(j + 1 + k)].iter());
                            st = St::Raw(k);
                            i = j + k + 2;
                            continue;
                        }
                    }
                    out.push(c);
                    i += 1;
                } else {
                    out.push(c);
                    i += 1;
                }
            }
        }
    }
    out
}

fn prev_is_ident(chars: &[char], i: usize) -> bool {
    i > 0 && (chars[i - 1].is_alphanumeric() || chars[i - 1] == '_')
}

#[test]
fn the_stripper_removes_both_comment_syntaxes_and_nothing_else() {
    assert_eq!(
        code_only("#[serde(deny_unknown_fields)] // x\nstruct A;\n"),
        "#[serde(deny_unknown_fields)] \nstruct A;\n"
    );
    // A doc comment PRAISING the attribute must not survive as code.
    assert_eq!(
        code_only("/// deny_unknown_fields\nstruct A;\n"),
        "\nstruct A;\n"
    );
    // Rust block comments NEST.
    assert_eq!(code_only("a/* x /* y */ z */b"), "ab");
    // Unterminated block swallows the rest — fails CLOSED.
    assert_eq!(code_only("a/* x\nb\nc"), "a\n\n");
}

#[test]
fn a_comment_marker_inside_a_string_literal_does_not_swallow_the_file() {
    // THE regression this module exists for. A `/*` inside the glob would
    // open a block comment that never closes, so everything after it —
    // every singleton call, every attribute — becomes invisible and the walk
    // reports a clean sheet.
    let src = "let g = \"nodes/*\";\nfn later() { TransportManager::get_or_init(); }\n";
    let got = code_only(src);
    assert!(
        got.contains("TransportManager::get_or_init("),
        "code after a string containing `/*` must survive; got: {got:?}"
    );
    // The line-comment marker has the same hazard.
    let src2 = "let u = \"http://x\"; // note\nfn later() { CALL; }\n";
    assert!(code_only(src2).contains("CALL"));
    // And it must still be a STRING in the output — the legal-key readers
    // parse slice literals straight out of this view.
    assert!(code_only("const K: &[&str] = &[\"a\", \"b\"];").contains("\"a\", \"b\""));
}

#[test]
fn raw_and_char_literals_are_modelled() {
    // A raw string holding an unbalanced comment opener.
    assert!(code_only("let r = r#\"/* not a comment \"#;\nfn f(){}\n").contains("fn f()"));
    // Matching hash counts: the inner `"#` does not close a `r##\"…\"##`.
    assert!(code_only("let r = r##\"a \"# b\"##;\nfn f(){}\n").contains("fn f()"));
    // Byte-raw prefix.
    assert!(code_only("let b = br#\"/*\"#;\nfn f(){}\n").contains("fn f()"));
    // A char literal holding a quote must not open a string.
    assert!(code_only("let q = '\"'; fn f(){}").contains("fn f()"));
    // LIFETIMES are not char literals — `'static` followed later by another
    // apostrophe must not swallow the span between them.
    assert!(
        code_only("fn f<'a>(x: &'a str) -> &'static str { \"y\" }\nfn g(){}").contains("fn g()")
    );
    // `r` inside an identifier is not a raw-string prefix.
    assert!(code_only("for x in y { CALL; }").contains("CALL"));
}

#[test]
fn the_literal_blanking_view_hides_content_and_keeps_positions() {
    // THE case it exists for: a file that NAMES a call in a string is not a
    // caller. `code_only` cannot tell the two apart — it preserves literals on
    // purpose — so the two views must disagree here, and that disagreement is
    // asserted rather than assumed.
    let src = "const W: &str = \"TransportManager::get_or_init(\";\n";
    assert!(
        code_only(src).contains("TransportManager::get_or_init("),
        "the preserving view must still see it — other callers parse literals"
    );
    assert!(
        !code_only_blank_literals(src).contains("TransportManager::get_or_init("),
        "the blanking view must NOT read a string as a call"
    );

    // A real call beside a string that names one: the call survives.
    let both = "let s = \"CALL_ME\"; fn f() { CALL_ME(); }\n";
    let got = code_only_blank_literals(both);
    assert_eq!(
        got.matches("CALL_ME").count(),
        1,
        "exactly the CODE occurrence survives; got {got:?}"
    );

    // POSITIONS are preserved — same length, same line count — so a
    // diagnostic computed over this view still points at the real file.
    for s in [
        src,
        both,
        "let r = r#\"TransportManager::init(\"#;\nfn f(){}\n",
        "let c = 'x'; let n = '\\n'; fn f(){}\n",
    ] {
        let blanked = code_only_blank_literals(s);
        assert_eq!(
            blanked.chars().count(),
            code_only(s).chars().count(),
            "blanking must not change length for {s:?}"
        );
        assert_eq!(
            blanked.lines().count(),
            code_only(s).lines().count(),
            "blanking must not change line count for {s:?}"
        );
    }

    // Raw strings are blanked too — a watch list written as `r#"..."#` is
    // still data. The delimiters survive; the content does not.
    let raw = "let r = r#\"TransportManager::init(\"#;\nfn f(){ REAL(); }\n";
    let rb = code_only_blank_literals(raw);
    assert!(!rb.contains("TransportManager::init("));
    assert!(
        rb.contains("REAL()"),
        "code after a raw string must survive"
    );
    assert!(
        rb.contains("r#\""),
        "the delimiter is structure, not content"
    );

    // A lifetime is NOT a char literal, so the span after it must not be
    // blanked — the naive quote-counter's failure, in the blanking mode.
    let lt = "fn f<'a>(x: &'a str) -> &'static str { \"y\" }\nfn KEEP(){}\n";
    assert!(
        code_only_blank_literals(lt).contains("KEEP"),
        "a lifetime must not open a literal that swallows the rest"
    );

    // An ESCAPED closing quote does not end the literal early.
    let esc = "let s = \"a\\\"CALL_ME\\\"b\"; fn KEEP(){}\n";
    let eb = code_only_blank_literals(esc);
    assert!(
        !eb.contains("CALL_ME"),
        "escaped quotes must not end the span"
    );
    assert!(eb.contains("KEEP"));
}

/// Is `dir` the root of a DIFFERENT checkout than the one being walked?
///
/// A git worktree carries a `.git` FILE (a gitdir pointer); a clone carries a
/// `.git` DIRECTORY. Either way, its presence below the repo root means the
/// subtree belongs to another checkout.
///
/// This is what stops a filesystem walk from wandering into sibling
/// worktrees nested under the repo root. On the main checkout there are ~22 sibling
/// worktrees, each holding another branch's IN-PROGRESS edits — and, during
/// a mutation run, deliberately broken code. A gate that reads them reports
/// on work that is not in this branch, and its verdict depends on what else
/// happens to be open on the developer's machine. Skipping `target/` alone does not help:
/// the worktrees sit under a tool's dot-directory, which no build-output filter names.
pub fn is_other_checkout(dir: &std::path::Path) -> bool {
    dir.join(".git").exists()
}
