// SPDX-License-Identifier: AGPL-3.0-only
//! The writer reads NO clock.
//!
//! `cerulion_bag/AGENTS.md` states the invariant: "ZERO wall-clock reads in
//! this crate — the byte-determinism gate. Every timestamp comes from the
//! caller; any time-based policy injects its clock from the caller (the chunk
//! time floor lives in bagd, not here)." This walk enforces it. A single
//! `SystemTime::now()` reaching a record field, a chunk-close decision, or an
//! attachment header would make two runs of the same input sequence differ —
//! and `bag_determinism_test` would catch it only if the value landed in the
//! bytes, not if it merely steered a flush.
//!
//! This is a STRUCTURAL walk, not a behavioural one, for the reason structural
//! walks exist in this repo: the property is "no code path anywhere in the
//! crate does X", and no finite set of inputs can demonstrate that. It reads
//! every `.rs` file under `cerulion_bag/src/` through a comment stripper and
//! requires zero hits for the clock vocabulary.
//!
//! The stripper is load-bearing, and its own correctness is the walk's only
//! soft spot: a broken stripper that returned the empty string would find
//! nothing and pass forever. Two things close that — `code_only` is pinned
//! directly against hand oracles below, and the walk's positive control runs
//! the SAME function over a synthetic string that does contain a clock read
//! and requires it to be FOUND.
//!
//! The stripper is STRING-LITERAL AWARE, and that is a correctness fix rather
//! than a refinement: a comment-only stripper reads `//` straight off the raw
//! stream, so any string containing one — every URL in the crate — opens a
//! phantom line comment that swallows the REST OF THAT LINE, a real
//! `std::time::Instant::now()` after it included. Modelling strings also
//! removes a false-positive class in the other direction: a clock token inside
//! an error message is text, not a clock read.

use std::fs;
use std::path::{Path, PathBuf};

/// The clock vocabulary. A hit on any of these in non-comment source is a
/// wall-clock read (or the import that enables one) and breaks byte
/// determinism.
///
/// The Rust-level names cover the crate's own idiom; the OS-level ones
/// (`clock_gettime`, `gettimeofday`, `mach_absolute_time`, plus the
/// `libc::time` call matched separately by [`libc_time_call_lines`]) cover
/// the `libc` route around them, which reads the same clocks with none of the
/// `std::time` vocabulary. That route is not hypothetical here: `libc` is
/// already a dependency of this crate (`writev(2)`, `sysconf(3)`, `madvise(2)`),
/// so a clock read is one line away with nothing to add to `Cargo.toml` and no
/// Rust-level name for the walk to catch it by.
///
/// Bare `time` is DELIBERATELY NOT a token, and the exclusion is the reason the
/// OS names are spelled out in full: the walker is SUBSTRING-based, while the
/// writer's own domain language is timestamps — `log_time`, `publish_time`,
/// `create_time`, `message_start_time`, `message_end_time` are MCAP field names
/// carrying CALLER-supplied stamps, i.e. exactly what the invariant REQUIRES,
/// and `lifetime` is not a stamp at all. The substring occurs 136 times across
/// `src/` at the time of writing, and adding a bare `time` token was MEASURED to
/// make the walk report 69 offending lines in all 10 of the crate's files — not
/// a gate, just noise. The `libc::time` CALL is subject to the same pressure —
/// it must be caught without catching `libc::timespec`, which this crate really
/// does use — and it is the one token that cannot be expressed as a substring
/// at all, so it is matched by [`libc_time_call_lines`] instead of living here.
/// The exclusion is pinned by
/// [`the_writers_own_timestamp_vocabulary_is_not_a_clock_read`].
///
/// This list catches DIRECT API reads. Its complement is dependency review: a
/// clock reached through a NEW crate (`quanta`, `coarsetime`, a re-exported
/// `chrono`) arrives as a `Cargo.toml` line, which is a reviewed surface here —
/// new dependencies need maintainer approval and pass the `deny.toml` gate. The
/// two halves are what make "no wall-clock reads" enforceable without an
/// unbounded name list.
const CLOCK_TOKENS: &[&str] = &[
    "SystemTime",
    "Instant",
    "std::time",
    "chrono",
    "clock_gettime",
    "gettimeofday",
    "mach_absolute_time",
];

/// The `libc::time` call, as [`clock_hits`] REPORTS it. Spelled with its open
/// paren because that is what the token means — the call, not the path — and
/// kept stable so an offender line reads the same as it did when this was a
/// plain substring.
const LIBC_TIME_TOKEN: &str = "libc::time(";

/// The path half of [`LIBC_TIME_TOKEN`], which is what is actually SEARCHED for.
const LIBC_TIME_PATH: &str = "libc::time";

/// The longest body a char-literal escape can have before its closing quote —
/// `'\u{10FFFF}'` is the worst case at 10 chars. Anything longer is a lifetime
/// tick, not an unterminated literal. See [`char_literal_end`].
const MAX_CHAR_ESCAPE_BODY: usize = 12;

/// Strip Rust comments AND the CONTENTS of string literals, keeping newlines so
/// a reported line stays meaningful.
///
/// Block comments are DEPTH-TRACKED because Rust nests them — a non-nesting
/// stripper ends the comment at the first `*/` and exposes the tail as code.
/// An unterminated block swallows the rest of the file, i.e. fails CLOSED: the
/// walk then sees LESS code, never more, so a malformed file cannot smuggle a
/// token through as "not a comment" — it can only hide code the walk would
/// have inspected, which the `code_only` oracles below make visible. An
/// unterminated string literal fails closed the same way.
///
/// Comments and strings are recognised in ONE interleaved pass rather than two
/// sequential ones, because each is inert inside the other and only a single
/// pass gets both directions right: a string pre-pass would see the odd quote
/// in `// don't` or `// he said "hi` as an opener and blank code on the lines
/// after it, while a comment-first pass is exactly the bug being fixed — `//`
/// inside a string opening a phantom comment. Inside a comment a quote is just
/// a character; inside a string `//` is just two characters.
///
/// A single quote is a char literal only if it CLOSES like one within
/// [`MAX_CHAR_ESCAPE_BODY`]; otherwise it is a lifetime tick (`&'a str`,
/// `'static`) and is emitted verbatim. Char-literal BODIES are kept verbatim
/// too: one character can hold neither a clock token nor a comment opener, so
/// the pass only has to SKIP the literal — `'"'` must not open a string and
/// `'/'` must not pair with what follows it.
///
/// This file holds the clock tokens as literals (`CLOCK_TOKENS`, the positive
/// control) and would strip its own — but the walk covers `src/` and this
/// file lives in `tests/`, so it is not in the walked set either way.
fn code_only(src: &str) -> String {
    let chars: Vec<char> = src.chars().collect();
    let mut out = String::with_capacity(src.len());
    let mut i = 0;
    let mut block_depth = 0usize;
    while i < chars.len() {
        if block_depth > 0 {
            if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
                block_depth += 1;
                i += 2;
            } else if chars[i] == '*' && chars.get(i + 1) == Some(&'/') {
                block_depth -= 1;
                i += 2;
            } else {
                if chars[i] == '\n' {
                    out.push('\n');
                }
                i += 1;
            }
            continue;
        }
        if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
            block_depth = 1;
            i += 2;
        } else if chars[i] == '/' && chars.get(i + 1) == Some(&'/') {
            while i < chars.len() && chars[i] != '\n' {
                i += 1;
            }
        } else if chars[i] == 'r' && starts_raw_string(&chars, i) {
            i = blank_raw_string(&chars, i, &mut out);
        } else if chars[i] == '"' {
            i = blank_string(&chars, i, &mut out);
        } else if chars[i] == '\'' {
            match char_literal_end(&chars, i) {
                // A char literal: copied verbatim, but SKIPPED as a unit so
                // neither its quote nor its body is re-read as an opener.
                Some(end) => {
                    out.extend(&chars[i..end]);
                    i = end;
                }
                // A lifetime tick — ordinary code.
                None => {
                    out.push('\'');
                    i += 1;
                }
            }
        } else {
            out.push(chars[i]);
            i += 1;
        }
    }
    out
}

/// Does a raw string open at `i` (where `chars[i] == 'r'`)?
///
/// The `r` must be a PREFIX, not the tail of an identifier — `str` immediately
/// followed by a quote is not valid Rust, but the check costs nothing and keeps
/// the pass from depending on that. A leading `b` OR `c` is allowed so `br"…"`,
/// `br#"…"#`, `cr"…"` and `cr#"…"#` are all recognised (the prefix letter is
/// emitted as ordinary code first). `cr` is the C-string form stabilised in Rust
/// 1.77; without it such a literal parses as ordinary code plus an ORDINARY
/// string, so a `"` in its body closes early and a `//` after that opens a
/// phantom line comment — hiding every clock read to the end of that line, which
/// is precisely the bug modelling strings exists to fix.
///
/// The non-raw C string `c"…"` needs nothing here: its `c` is emitted as
/// ordinary code and the `"` takes the [`blank_string`] path, whose escape rules
/// are the ones `c"…"` follows.
///
/// Pinned by [`c_string_literals_take_the_same_paths_as_their_byte_string_twins`]
/// and [`a_raw_c_string_does_not_hide_the_clock_read_after_it`]; the token
/// boundary by [`an_identifier_ending_in_a_prefix_letter_does_not_open_a_raw_string`].
fn starts_raw_string(chars: &[char], i: usize) -> bool {
    let ident = |c: char| c.is_alphanumeric() || c == '_';
    let prefix_ok = match i.checked_sub(1).map(|p| chars[p]) {
        None => true,
        // A prefix letter only counts when IT is at a token boundary — `incr"…"`
        // is an identifier, not a raw string with an `inc` prefix.
        Some('b' | 'c') => i.checked_sub(2).map(|p| chars[p]).is_none_or(|c| !ident(c)),
        Some(c) => !ident(c),
    };
    if !prefix_ok {
        return false;
    }
    let mut j = i + 1;
    while chars.get(j) == Some(&'#') {
        j += 1;
    }
    chars.get(j) == Some(&'"')
}

/// Blank the body of the raw string opening at `i`, keeping the `r`, the hashes
/// and both quotes. Returns the index just past the closing delimiter (or the
/// end of input, which fails CLOSED).
fn blank_raw_string(chars: &[char], i: usize, out: &mut String) -> usize {
    let mut j = i + 1;
    let mut hashes = 0usize;
    while chars.get(j) == Some(&'#') {
        hashes += 1;
        j += 1;
    }
    out.push('r');
    for _ in 0..hashes {
        out.push('#');
    }
    out.push('"');
    j += 1;
    while j < chars.len() {
        // `(1..=0)` is empty and `all` holds, so a hash-less `r"…"` closes at
        // the first quote — which is exactly its rule.
        if chars[j] == '"' && (1..=hashes).all(|k| chars.get(j + k) == Some(&'#')) {
            out.push('"');
            for _ in 0..hashes {
                out.push('#');
            }
            return j + 1 + hashes;
        }
        out.push(if chars[j] == '\n' { '\n' } else { ' ' });
        j += 1;
    }
    j
}

/// Blank the body of the ordinary string opening at `i` (`chars[i] == '"'`),
/// keeping both quotes and every newline. Handles `\"` and `\\`. Returns the
/// index just past the closing quote (or the end of input, which fails CLOSED).
fn blank_string(chars: &[char], i: usize, out: &mut String) -> usize {
    out.push('"');
    let mut j = i + 1;
    while j < chars.len() {
        match chars[j] {
            '\\' => {
                out.push(' ');
                // A backslash-newline is a line continuation: keep the newline
                // so line numbers survive.
                if let Some(&next) = chars.get(j + 1) {
                    out.push(if next == '\n' { '\n' } else { ' ' });
                }
                j += 2;
            }
            '"' => {
                out.push('"');
                return j + 1;
            }
            '\n' => {
                out.push('\n');
                j += 1;
            }
            _ => {
                out.push(' ');
                j += 1;
            }
        }
    }
    j
}

/// If a char literal opens at `i` (`chars[i] == '\''`), the index just past its
/// closing quote; otherwise `None`, meaning the quote is a LIFETIME tick.
///
/// The discriminator is CONSERVATIVE by construction: a char literal closes
/// within a handful of characters, so `'a'` (closing quote at `i + 2`) is a
/// literal while `'a` in `&'a str`, `<'a>` or `'static` is not. Escapes are
/// bounded by [`MAX_CHAR_ESCAPE_BODY`] and cannot span a newline, so an
/// unbalanced quote never eats the rest of a file.
fn char_literal_end(chars: &[char], i: usize) -> Option<usize> {
    if chars.get(i + 1) == Some(&'\\') {
        // `i + 2` is the escaped character itself (`'\''` closes at `i + 3`,
        // not at that quote); the search for the closing quote starts past it.
        let limit = (i + 3 + MAX_CHAR_ESCAPE_BODY).min(chars.len());
        for (j, c) in chars.iter().enumerate().take(limit).skip(i + 3) {
            match c {
                '\'' => return Some(j + 1),
                '\n' => return None,
                _ => {}
            }
        }
        return None;
    }
    match (chars.get(i + 1), chars.get(i + 2)) {
        (Some('\n'), _) | (None, _) => None,
        (Some(_), Some('\'')) => Some(i + 3),
        _ => None,
    }
}

/// An ASCII identifier byte — what decides whether a matched path is the WHOLE
/// identifier or merely a prefix of a longer one.
fn is_ident_byte(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_'
}

/// The 1-based lines of `code` (already comment- and string-stripped) that CALL
/// `libc::time`.
///
/// # Why this token cannot be a substring
///
/// It is the only entry in the vocabulary that has to distinguish a CALL from a
/// longer identifier: `libc::timespec` is a type this crate really uses, so the
/// bare path over-matches, while the contiguous `libc::time(` needle
/// under-matches — `libc::time (ptr)` and a path with the paren on the NEXT
/// LINE are both ordinary Rust that `rustfmt` will not rewrite back (a long
/// argument list wraps exactly that way), and either one slipped the walk
/// silently. A gate that a formatter can turn off is not a gate.
///
/// So the path is matched at its BOUNDARIES instead:
///
/// * the byte before it must not be an identifier byte, so the match is a path
///   and not the tail of a longer one; and
/// * the byte after it must not be an identifier byte — this, and only this, is
///   what keeps `libc::timespec` out, and it is checked BEFORE the whitespace
///   skip rather than folded into it, or `libc::timespec (x)` would match; and
/// * the next NON-WHITESPACE byte must be `(`, which is the call.
///
/// The whitespace skip crosses newlines, which is why this reads the whole
/// stripped source rather than one line at a time — a per-line scan cannot see
/// a paren on the following line by construction. The line REPORTED is the
/// path's, which is where a reader looks for the call.
///
/// Comments and string contents are already gone, so the skip cannot walk over
/// a comment and there is nothing here that re-implements the stripper.
fn libc_time_call_lines(code: &str) -> Vec<usize> {
    let bytes = code.as_bytes();
    let mut lines = Vec::new();
    let mut from = 0usize;
    while let Some(rel) = code[from..].find(LIBC_TIME_PATH) {
        let at = from + rel;
        let end = at + LIBC_TIME_PATH.len();
        from = end;
        if at > 0 && is_ident_byte(bytes[at - 1]) {
            continue;
        }
        if bytes.get(end).is_some_and(|b| is_ident_byte(*b)) {
            continue;
        }
        if code[end..].trim_start().starts_with('(') {
            lines.push(code[..at].matches('\n').count() + 1);
        }
    }
    lines
}

/// Every clock token present in `src`'s NON-COMMENT text, with the 1-based
/// line it sits on. This is the walker; the file walk and the positive control
/// both drive exactly this function.
fn clock_hits(src: &str) -> Vec<(usize, &'static str)> {
    let code = code_only(src);
    let mut hits = Vec::new();
    for (n, line) in code.lines().enumerate() {
        for token in CLOCK_TOKENS {
            if line.contains(token) {
                hits.push((n + 1, *token));
            }
        }
    }
    // …and the one token that is matched at its boundaries over the whole
    // stripped source rather than per line — see `libc_time_call_lines`.
    hits.extend(
        libc_time_call_lines(&code)
            .into_iter()
            .map(|line| (line, LIBC_TIME_TOKEN)),
    );
    // Line order, so an offender list reads top to bottom whichever matcher
    // produced each entry.
    hits.sort_unstable();
    hits
}

/// Every `.rs` file under `dir`, recursively.
fn rust_sources(dir: &Path) -> Vec<PathBuf> {
    let mut found = Vec::new();
    let entries = fs::read_dir(dir).unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display()));
    for entry in entries {
        let path = entry.expect("dir entry").path();
        if path.is_dir() {
            found.extend(rust_sources(&path));
        } else if path.extension().is_some_and(|e| e == "rs") {
            found.push(path);
        }
    }
    found.sort();
    found
}

#[test]
fn the_comment_stripper_removes_both_syntaxes_and_nothing_else() {
    assert_eq!(
        code_only("let a = 1; // Instant\nlet b = 2;\n"),
        "let a = 1; \nlet b = 2;\n"
    );
    assert_eq!(code_only("a /* Instant */ b"), "a  b");
    // Nested block comments — Rust allows them, and a non-nesting stripper
    // would end the comment early and expose the tail as code.
    assert_eq!(code_only("a /* x /* y */ Instant */ b"), "a  b");
    // A block opener inside a line comment is not an opener.
    assert_eq!(code_only("// /* Instant\nreal code\n"), "\nreal code\n");
    // An unterminated block swallows the rest — fails CLOSED.
    assert_eq!(code_only("code /* Instant"), "code ");
}

/// The string half of the stripper, pinned against hand oracles the same way.
///
/// The headline is the first arm: before strings were modelled, the `//` in any
/// URL opened a phantom line comment and blanked the REST OF THAT LINE.
#[test]
fn the_stripper_neutralises_string_literals_without_eating_code() {
    // A `//` inside a string does NOT open a comment: the code after it
    // survives, delimiters and all.
    assert_eq!(
        code_only(r#"let url = "https://x"; let t = 1;"#),
        r#"let url = "         "; let t = 1;"#
    );
    // Escapes: `\"` does not close the literal, and `\\` is the escape — the
    // quote after it does close.
    assert_eq!(code_only(r#"a "x\"y" b"#), r#"a "    " b"#);
    assert_eq!(code_only(r#"a "x\\" b"#), r#"a "   " b"#);
    // Raw strings: no escapes, so `//` inside one is still just text.
    assert_eq!(code_only(r####"a r"x//y" b"####), r####"a r"    " b"####);
    // Hash-counted: a bare `"` inside an `r#"…"#` does not close it.
    assert_eq!(code_only(r####"a r#"x"y"# b"####), r####"a r#"   "# b"####);
    // Byte and raw-byte strings ride the same two paths.
    assert_eq!(code_only(r#"a b"x//y" c"#), r#"a b"    " c"#);
    assert_eq!(
        code_only(r####"a br#"x"y"# b"####),
        r####"a br#"   "# b"####
    );
    // A multi-line string keeps its newlines, so reported line numbers survive.
    assert_eq!(code_only("a \"x\ny\" b"), "a \" \n \" b");
    // An unterminated string fails CLOSED, like an unterminated block comment.
    assert_eq!(code_only("code \"Instant"), "code \"       ");
    // Lifetimes are NOT char literals and pass through untouched.
    assert_eq!(
        code_only("fn f<'a>(x: &'a str) {}"),
        "fn f<'a>(x: &'a str) {}"
    );
    assert_eq!(code_only("&'static str"), "&'static str");
    // Char literals are skipped as a unit, so their body cannot open anything.
    assert_eq!(code_only(r#"m(c, '"', "s//x")"#), r#"m(c, '"', "    ")"#);
    assert_eq!(code_only(r"m(c, '\'', '/')"), r"m(c, '\'', '/')");
    // A comment still wins where it really opens — quotes inside one are inert,
    // so an odd quote in prose cannot blank the lines below it.
    assert_eq!(
        code_only("// don't say \"Instant\"\nlet t = 2;\n"),
        "\nlet t = 2;\n"
    );
}

/// C-string literals (`c"…"`, `cr"…"`, `cr#"…"#`, stabilised in Rust 1.77) take
/// exactly the two paths their byte-string twins take, pinned against the same
/// hand oracles as those twins so the two prefixes cannot drift apart.
#[test]
fn c_string_literals_take_the_same_paths_as_their_byte_string_twins() {
    // Non-raw: the `c` is emitted as ordinary code and the `"` takes the
    // ordinary-string path — the same shape as the `b"…"` arm above.
    assert_eq!(code_only(r#"a c"x//y" b"#), r#"a c"    " b"#);
    // Raw, hash-less: no escapes, so a `//` inside is still just text.
    assert_eq!(code_only(r####"a cr"x//y" b"####), r####"a cr"    " b"####);
    // Raw, hash-counted: a bare `"` inside an `cr#"…"#` does not close it.
    assert_eq!(
        code_only(r####"a cr#"x"y"# b"####),
        r####"a cr#"   "# b"####
    );
}

/// The prefix letter must ITSELF sit at a token boundary. `b` and `c` are
/// ordinary identifier characters, so an identifier ending in one, immediately
/// followed by `r"`, must not be read as a prefixed raw string.
///
/// The discriminator is a backslash, because it is the one character the two
/// readings disagree about: a raw string has no escapes, so a wrongly-opened
/// `r"a\"` closes at the quote right after the backslash and re-exposes the rest
/// of the literal as code — where the trailing `"` then opens a run-to-EOF
/// string that blanks everything after it, the clock read included. Under the
/// correct reading the quoted run is one ordinary string and the code after it
/// survives.
#[test]
fn an_identifier_ending_in_a_prefix_letter_does_not_open_a_raw_string() {
    for src in [
        r####"let x = incr"a\"b"; let t = std::time::Instant::now();"####,
        r####"let x = verb"a\"b"; let t = std::time::Instant::now();"####,
    ] {
        let hits = clock_hits(src);
        assert!(
            !hits.is_empty(),
            "`{src}`: the quoted run is an ordinary string, so the clock read \
             after it must still be found, got {hits:?}"
        );
    }
}

/// The positive control (anti-tautology): the SAME walker, over a SYNTHETIC
/// in-test string, must FIND a clock read. Without it, a walker that returned
/// nothing for every input — a broken stripper, a typo'd token list, an
/// `is_empty()` that always holds — would make the walk below vacuous and it
/// would pass forever. The control reads no file, so it cannot be satisfied by
/// (or coupled to) any other crate's source.
#[test]
fn the_walker_finds_a_clock_read_in_synthetic_source() {
    let hit = clock_hits("fn f() {\n    let t = std::time::Instant::now();\n}\n");
    assert!(
        !hit.is_empty(),
        "the walker must find a clock read in source that plainly contains one"
    );
    assert!(
        hit.iter().all(|(line, _)| *line == 2),
        "the reported line must be the line the read is on: {hit:?}"
    );
    let tokens: Vec<&str> = hit.iter().map(|(_, t)| *t).collect();
    assert!(tokens.contains(&"Instant"), "found {tokens:?}");
    assert!(tokens.contains(&"std::time"), "found {tokens:?}");

    // The negative half of the same control: the identical text inside a
    // comment is NOT a hit. Without this, a walker that ignored the stripper
    // entirely would also pass the positive half.
    assert!(
        clock_hits("fn f() {\n    // let t = std::time::Instant::now();\n}\n").is_empty(),
        "a clock read inside a comment is not a clock read"
    );
    assert!(
        clock_hits("/* let t = std::time::Instant::now(); */\nfn f() {}\n").is_empty(),
        "a clock read inside a block comment is not a clock read"
    );
}

/// The walker's string behaviour, at the level that matters: does a clock read
/// still get FOUND, and does text still get IGNORED?
///
/// The first arm is the regression this pins. With a comment-only stripper the
/// `//` in `https://` opened a phantom line comment, so everything after it on
/// that line — including a real `Instant::now()` — was stripped before the
/// token scan ever saw it. The walk would have reported ZERO offenders over a
/// crate that read a clock.
#[test]
fn a_url_in_a_string_does_not_hide_the_clock_read_after_it() {
    let hits = clock_hits(r#"let url = "https://x"; let t = std::time::Instant::now();"#);
    let tokens: Vec<&str> = hits.iter().map(|(_, t)| *t).collect();
    assert!(
        tokens.contains(&"Instant") && tokens.contains(&"std::time"),
        "a clock read after a URL literal must still be found, got {hits:?}"
    );

    // The same shape one line down, so the phantom comment (which ran to the
    // end of ITS line) would have to be gone for this to be found at all.
    let hits = clock_hits("let url = \"https://x\";\nlet t = std::time::Instant::now();\n");
    assert!(
        hits.iter().any(|(line, _)| *line == 2),
        "expected the hit on line 2, got {hits:?}"
    );

    // Raw strings take the other code path and must behave the same.
    let hits = clock_hits(r####"let u = r#"https://x"#; let t = std::time::Instant::now();"####);
    assert!(
        !hits.is_empty(),
        "a clock read after a RAW string containing `//` must still be found"
    );

    // A lifetime tick is not a literal opener: the clock read after one is
    // still found, and the tick itself is not a hit.
    let hits = clock_hits("fn f<'a>(x: &'a str) { let t = std::time::Instant::now(); }\n");
    assert!(
        !hits.is_empty(),
        "a clock read after a lifetime tick must still be found"
    );
}

/// The other direction: a clock token that is TEXT is not a clock read. This
/// is a false-positive class the comment-only stripper carried — an error
/// message naming `Instant` would have failed the walk.
#[test]
fn a_clock_token_inside_a_string_is_not_a_clock_read() {
    assert!(
        clock_hits(r#"let msg = "Instant";"#).is_empty(),
        "a clock token inside a string literal is text, not a clock read"
    );
    assert!(
        clock_hits(r#"panic!("no SystemTime here: {e}");"#).is_empty(),
        "a clock token inside a string literal is text, not a clock read"
    );
    assert!(
        clock_hits(r####"let s = r#"std::time::Instant"#;"####).is_empty(),
        "a clock token inside a RAW string literal is text, not a clock read"
    );
}

/// The raw C-string form, at walker level — the arm the prefix fix exists for.
///
/// Without `c` in the prefix set the `r` is not a raw-string opener, so the `"`
/// after the hash takes the ORDINARY-string path and closes at the body's own
/// quote. The `//` that follows is then read as code, opens a phantom line
/// comment, and swallows the rest of the line — a real
/// `std::time::Instant::now()` included. That is the same class the string
/// modelling was added to fix, re-opened by one literal form.
#[test]
fn a_raw_c_string_does_not_hide_the_clock_read_after_it() {
    let hits = clock_hits(r####"let s = cr#"a"b//c"#; let t = std::time::Instant::now();"####);
    let tokens: Vec<&str> = hits.iter().map(|(_, t)| *t).collect();
    assert!(
        tokens.contains(&"Instant") && tokens.contains(&"std::time"),
        "a clock read after a raw C-string literal must still be found, got {hits:?}"
    );

    // The non-raw `c"…"` rides the ordinary-string path and must behave the
    // same: its `//` is text, the clock read after it is code.
    let hits = clock_hits(r#"let s = c"a//b"; let t = std::time::Instant::now();"#);
    assert!(
        !hits.is_empty(),
        "a clock read after a c\"…\" literal must still be found, got {hits:?}"
    );

    // And the other direction, for both forms: a clock token inside one is text.
    assert!(
        clock_hits(r####"let s = cr#"std::time::Instant"#;"####).is_empty(),
        "a clock token inside a raw C-string literal is text, not a clock read"
    );
    assert!(
        clock_hits(r#"let s = c"SystemTime";"#).is_empty(),
        "a clock token inside a c\"…\" literal is text, not a clock read"
    );
}

/// The OS-level half of the vocabulary. `libc` is ALREADY a dependency of this
/// crate (`writev(2)`, `sysconf(3)`, `madvise(2)`), so this route needs no
/// manifest change and carries none of the Rust-level names the walk started
/// with — a clock read through it would have been invisible.
#[test]
fn the_walker_finds_an_os_level_clock_read() {
    for (src, token) in [
        (
            "let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };\n\
             unsafe { libc::clock_gettime(libc::CLOCK_REALTIME, &mut ts) };\n",
            "clock_gettime",
        ),
        (
            "unsafe { libc::gettimeofday(&mut tv, std::ptr::null_mut()) };\n",
            "gettimeofday",
        ),
        (
            "let ticks = unsafe { mach_absolute_time() };\n",
            "mach_absolute_time",
        ),
        (
            "let now = unsafe { libc::time(std::ptr::null_mut()) };\n",
            "libc::time(",
        ),
    ] {
        let hits = clock_hits(src);
        let tokens: Vec<&str> = hits.iter().map(|(_, t)| *t).collect();
        assert!(
            tokens.contains(&token),
            "`{token}` must be part of the clock vocabulary — the libc route reads \
             the same clocks with none of the std::time names; got {hits:?} for \
             `{src}`"
        );
    }
}

/// The `libc::time` CALL is matched at its BOUNDARIES, not as a
/// contiguous substring.
///
/// A needle of the literal `libc::time(` would require the paren to
/// sit immediately after the path. Two spellings of the same call slip it,
/// and neither is exotic — `libc::time (ptr)` is legal Rust, and a wrapped
/// argument list puts the paren on the NEXT LINE, which a per-line substring
/// scan cannot see at all. `rustfmt` leaves both alone, so such a walk could be
/// turned off by formatting.
///
/// The whole difficulty is that the match must not reach `libc::timespec`, which
/// this crate really uses: the arms below drive the trailing-identifier case
/// WITH a following paren, which is the shape a naive "drop the paren from the
/// needle, skip whitespace" approach admits.
#[test]
fn a_libc_time_call_is_matched_at_its_boundaries_not_as_a_contiguous_substring() {
    let found = |src: &str| clock_hits(src).iter().any(|(_, t)| *t == LIBC_TIME_TOKEN);

    // FOUND: the contiguous spelling (the shape a literal needle catches), the
    // space, the wrapped call, and a wider gap.
    assert!(
        found("let now = unsafe { libc::time(std::ptr::null_mut()) };\n"),
        "the contiguous call must still be found"
    );
    assert!(
        found("let now = unsafe { libc::time (std::ptr::null_mut()) };\n"),
        "a space between the path and its paren is the same call"
    );
    assert!(
        found("let now = unsafe {\n    libc::time\n        (std::ptr::null_mut())\n};\n"),
        "a paren on the NEXT LINE is the same call — this is what a wrapped \
         argument list looks like"
    );
    assert!(
        found("libc::time\t (ptr)\n"),
        "any whitespace run between the path and its paren is the same call"
    );

    // …and the LINE reported is the path's, not the paren's.
    let hits = clock_hits("fn f() {\n    libc::time\n        (ptr);\n}\n");
    assert_eq!(
        hits.iter()
            .filter(|(_, t)| *t == LIBC_TIME_TOKEN)
            .map(|(line, _)| *line)
            .collect::<Vec<_>>(),
        vec![2],
        "the offender line must be where the call is written: {hits:?}"
    );

    // NOT FOUND: the identifier CONTINUES, so this is a different name. Both
    // spellings, because the second is the one a whitespace-skipping fix that
    // forgot the right boundary would report.
    assert!(
        !found("let ts: libc::timespec = zeroed();\n"),
        "`libc::timespec` is a type this crate uses, not a clock read"
    );
    assert!(
        !found("let ts = libc::timespec (0, 0);\n"),
        "a following paren must not rescue a match whose identifier continues"
    );
    // …and the LEFT boundary: a path that merely ENDS in the token is not it.
    assert!(
        !found("let now = mylibc::time(ptr);\n"),
        "`libc::time` must be a path token, not the tail of a longer one"
    );
    // …and the token names the CALL, so a path with no call is not a hit.
    assert!(
        !found("use libc::time;\n"),
        "an import is not a clock READ — the token names the call"
    );

    // The stripper still governs: the same call as comment or as text is not a
    // read. Without this the boundary matcher could have bypassed `code_only`.
    assert!(
        !found("// let now = libc::time (ptr);\n"),
        "a call inside a comment is not a clock read"
    );
    assert!(
        !found("panic!(\"do not call libc::time (ptr) here\");\n"),
        "a call named inside a string is text, not a clock read"
    );
}

/// The complement of the widened set, and the reason bare `time` is NOT a token:
/// the writer's own domain language is timestamps. Every name below is an MCAP
/// field carrying a CALLER-supplied stamp — exactly what the invariant REQUIRES
/// — and `libc::timespec` is a type this crate really uses. A bare `time` token
/// would report all of them and the walk would be useless noise.
#[test]
fn the_writers_own_timestamp_vocabulary_is_not_a_clock_read() {
    assert!(
        clock_hits(
            "let log_time = msg.publish_time;\n\
             header.message_start_time = first;\n\
             header.message_end_time = last;\n\
             let create_time = caller_stamp;\n\
             let ts: libc::timespec = zeroed();\n\
             fn f<'a>(x: &'a str) -> Lifetime { lifetime(x) }\n"
        )
        .is_empty(),
        "a caller-supplied timestamp field is not a clock read — bare `time` must \
         not be in the vocabulary"
    );
}

#[test]
fn the_writer_reads_no_clock() {
    let src_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let files = rust_sources(&src_dir);
    // A broken enumeration must fail LOUDLY rather than pass as "no clocks
    // found" over an empty set.
    assert!(
        !files.is_empty(),
        "found no .rs files under {} — the walk enumerated nothing, so it \
         proved nothing",
        src_dir.display()
    );

    let mut offenders: Vec<String> = Vec::new();
    for path in &files {
        let src =
            fs::read_to_string(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        for (line, token) in clock_hits(&src) {
            offenders.push(format!("{}:{line}: {token}", path.display()));
        }
    }

    assert!(
        offenders.is_empty(),
        "cerulion_bag reads a clock — that breaks byte determinism (the crate's \
         first invariant: every timestamp comes from the caller, and any \
         time-based policy injects its clock from the caller; the chunk time \
         floor lives in cerulion_bagd, not here). Offenders across {} file(s):\n{}",
        files.len(),
        offenders.join("\n")
    );
}
