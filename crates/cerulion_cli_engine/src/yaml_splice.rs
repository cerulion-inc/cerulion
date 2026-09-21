// SPDX-License-Identifier: AGPL-3.0-only
//! Surgical top-level-block splicing for hand-authored YAML.
//!
//! This was extracted out of [`crate::partition_emit`], where it
//! was written for the `process_groups:` rewrite. It is the machinery a
//! splice needs and a serde round-trip destroys: exact byte spans per physical
//! line (LF and CRLF), a column-0 key scan whose block bounds treat comments
//! and blank lines as NEUTRAL, and a scalar renderer that re-parses to the
//! string it was given.
//!
//! It lives in its own module rather than in `partition_emit` because
//! it has a SECOND caller with nothing to do with partitioning —
//! `graph_cmd`'s `node stage`, which appends into the `nodes:` block. Two
//! copies of a scanner whose characteristic bug class is "the block bounds were
//! re-derived somewhere else" is precisely how two copies drift; one copy,
//! imported twice, is the alternative.
//!
//! Every function here is PURE (`&str` in, `String`/spans out) and has no
//! in-module tests by design: the behaviour is pinned through its two callers
//! in `tests/partition_emit_test.rs` and `tests/node_stage_preservation_test.rs`,
//! where the oracles are whole documents rather than fragments.

/// Render a node id / group name as a YAML scalar valid in BOTH a block mapping
/// key and inside a flow sequence `[...]`. A strict identifier-like token is
/// emitted plain; anything else is DOUBLE-QUOTED (always unambiguous in flow
/// context), so a token containing a comma, colon, bracket, space — or a bare
/// YAML keyword/number — can never re-parse as something other than the
/// original string.
pub(crate) fn render_scalar(s: &str) -> String {
    if is_plain_safe(s) {
        s.to_string()
    } else {
        format!("\"{}\"", escape_double_quoted(s))
    }
}

/// Escape `s` for YAML's DOUBLE-QUOTED scalar style, which is the one style
/// that can represent any string — including the control characters that have
/// no other spelling.
///
/// **Escaping only `\` and `"` is not enough.** That
/// would assume node ids and group names are "single-line printable text",
/// and nothing enforces that: `cerulion node stage --id` takes an arbitrary
/// string, `validate_graph` has no charset rule for a node id (it checks
/// uniqueness, and the derived-topic rule only bites on a node that HAS an
/// output), and a hand-authored graph can carry one through a quoted scalar of
/// its own. Two failure modes follow, both of them the corruption class this
/// module exists to prevent:
///
/// * A literal NEWLINE inside `"..."` is legal YAML — a quoted scalar may span
///   lines — and the break FOLDS to a space, so `a\nb` comes back as `a b`. A
///   silently different id, in a file nothing would flag.
/// * A `NUL`/`SOH`/… byte is not in YAML's printable set at all, so the
///   emitted document cannot be re-read: `graph_read` fails and the graph
///   is bricked.
///
/// Everything else passes through unchanged, so an ordinary quoted token
/// (`a,b`, `c: d`, `1e3`) renders verbatim.
fn escape_double_quoted(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    for ch in s.chars() {
        match ch {
            '\\' => out.push_str("\\\\"),
            '"' => out.push_str("\\\""),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\0' => out.push_str("\\0"),
            '\u{7}' => out.push_str("\\a"),
            '\u{8}' => out.push_str("\\b"),
            '\u{b}' => out.push_str("\\v"),
            '\u{c}' => out.push_str("\\f"),
            '\u{1b}' => out.push_str("\\e"),
            // The rest of C0, DEL and C1 (`char::is_control` covers all
            // three), plus the two Unicode separators YAML 1.1 reads as line
            // BREAKS inside a quoted scalar — conservative, since escaping a
            // character that needed no escape costs nothing while emitting one
            // that did costs the document.
            c if c.is_control() || c == '\u{2028}' || c == '\u{2029}' => {
                let n = c as u32;
                if n <= 0xFF {
                    out.push_str(&format!("\\x{n:02x}"));
                } else if n <= 0xFFFF {
                    out.push_str(&format!("\\u{n:04x}"));
                } else {
                    out.push_str(&format!("\\U{n:08x}"));
                }
            }
            c => out.push(c),
        }
    }
    out
}

/// True when `s` may be emitted as a PLAIN (unquoted) YAML scalar with no
/// ambiguity in flow-sequence or block-key position. Conservative on purpose —
/// a `false` result only means "quote it", which is always safe.
///
/// TWO gates, in order. The CHARSET gate rejects anything whose punctuation
/// could change the document's SHAPE (a comma, colon, bracket, quote, space, a
/// leading `-`/`.`/`+`), and the RESOLUTION gate ([`resolves_as_non_string`])
/// rejects what survives it but would be read as a number, boolean, null or
/// timestamp.
pub(crate) fn is_plain_safe(s: &str) -> bool {
    let Some(first) = s.chars().next() else {
        return false; // empty ⇒ must be `""`
    };
    // Leading char: alnum, `_`, or `/` (absolute topic-style ids). A leading
    // `-`/`.`/`+` (a signed number, `.inf`, `.nan`) is rejected here, which is
    // why `resolves_as_non_string` never sees one through this path.
    if !(first.is_ascii_alphanumeric() || first == '_' || first == '/') {
        return false;
    }
    if !s
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '/' | '.' | '-'))
    {
        return false;
    }
    !resolves_as_non_string(s)
}

/// True when a YAML reader can resolve `s` as something OTHER than a string.
///
/// **The bar is ANY common reader, not the one this crate happens to link.**
/// MEASURED on `serde_yaml` 0.9.34: its typed `String`
/// deserialization is LENIENT — `1e3`, `0x10`, `true` and `null` all come back
/// as their original TEXT, as values AND as mapping keys, through the real
/// `parse_graph_raw`. So an unquoted numeric-looking scalar does NOT break the
/// graph parser, and it does not "fail string
/// deserialization" for that parser. What it DOES break is
/// everything else: the same document read as an untyped `serde_yaml::Value`
/// resolves `1e3` to `Number(1000.0)`, and so does PyYAML, `yq`, or any future
/// parser this crate migrates to (`serde_yaml` is deprecated). A graph file is
/// a documented HAND-EDIT surface that other tools read, so the emitter must
/// not write a token that means something else to them.
///
/// Deliberately WIDER than any single reader's grammar — YAML 1.1 resolves
/// `yes`/`on`/`010`/`1_000`/`1:2:3` as non-strings where `serde_yaml` does not,
/// and quoting one extra token costs nothing while emitting one costs a
/// silently retyped value. Realistic identifiers are untouched: `camera`,
/// `relay`, `n0`, `p0`, `geometry_msgs/Vector3`, `/tf`, `1.2.3` and `0.0.0.0`
/// are all still plain.
pub(crate) fn resolves_as_non_string(s: &str) -> bool {
    let lower = s.to_ascii_lowercase();
    // Bare boolean/null literals — the YAML 1.1 set, which is a superset of
    // 1.2's (`yes`/`no`/`on`/`off`/`y`/`n` are strings to `serde_yaml`).
    if matches!(
        lower.as_str(),
        "true" | "false" | "null" | "yes" | "no" | "on" | "off" | "y" | "n" | "~"
    ) {
        return true;
    }
    // A sign is unreachable through `is_plain_safe`'s charset gate; stripped
    // here so this predicate is correct as a standalone rule.
    let unsigned = lower.strip_prefix(['+', '-']).unwrap_or(&lower);
    if matches!(unsigned, ".inf" | ".nan" | "inf" | "nan") {
        return true;
    }
    is_yaml_number(unsigned) || is_yaml_date(s)
}

/// True when `body` (already lowercased and unsigned) is a YAML integer or
/// float in any of the resolvers' spellings: `0x`/`0o`/`0b` radix forms, a
/// decimal integer, or a decimal float with an optional exponent. `_` digit
/// separators are accepted because YAML 1.1 does.
fn is_yaml_number(body: &str) -> bool {
    type DigitTest = fn(char) -> bool;
    const RADIX: [(&str, DigitTest); 3] = [
        ("0x", |c| c.is_ascii_hexdigit()),
        ("0o", |c| c.is_digit(8)),
        ("0b", |c| c == '0' || c == '1'),
    ];
    for (prefix, is_digit) in RADIX {
        if let Some(rest) = body.strip_prefix(prefix) {
            // A name like `0bad` / `0xyz` strips the prefix but is not a
            // number, so the digit test decides — never the prefix alone.
            return rest.chars().any(is_digit) && rest.chars().all(|c| is_digit(c) || c == '_');
        }
    }

    let (mantissa, exponent) = match body.split_once('e') {
        Some((m, e)) => (m, Some(e)),
        None => (body, None),
    };
    if let Some(exp) = exponent {
        let exp = exp.strip_prefix(['+', '-']).unwrap_or(exp);
        if exp.is_empty() || !exp.chars().all(|c| c.is_ascii_digit()) {
            return false;
        }
    }
    let (int_part, frac_part) = match mantissa.split_once('.') {
        Some((i, f)) => (i, Some(f)),
        None => (mantissa, None),
    };
    let digits_only = |p: &str| p.chars().all(|c| c.is_ascii_digit() || c == '_');
    if !digits_only(int_part) {
        return false;
    }
    if let Some(frac) = frac_part {
        // A SECOND `.` lands here (`1.2.3` ⇒ frac `2.3`), which is what keeps
        // version-ish and dotted-quad ids plain.
        if !digits_only(frac) {
            return false;
        }
    }
    mantissa.chars().any(|c| c.is_ascii_digit())
}

/// True when `s` has the `YYYY-M-D` shape YAML 1.1's timestamp resolver reads
/// as a date. `serde_yaml` calls it a string; PyYAML returns a `datetime.date`.
fn is_yaml_date(s: &str) -> bool {
    let mut parts = s.split('-');
    let (Some(year), Some(month), Some(day), None) =
        (parts.next(), parts.next(), parts.next(), parts.next())
    else {
        return false;
    };
    let numeric = |p: &str| !p.is_empty() && p.chars().all(|c| c.is_ascii_digit());
    year.len() == 4
        && numeric(year)
        && (1..=2).contains(&month.len())
        && numeric(month)
        && (1..=2).contains(&day.len())
        && numeric(day)
}

/// The verdict of scanning a document for ONE top-level block.
pub(crate) enum BlockScan {
    /// The key does not appear at column 0.
    Absent,
    /// Exactly one block: byte span `[start, end)` — the key line through the
    /// end of its LAST indented line. Trailing blank lines AND trailing
    /// column-0 `#` comments (between the block's last indented line and the
    /// next column-0 key) stay OUTSIDE the span — they belong to the NEXT
    /// section, so splices preserve them byte-identically.
    One { start: usize, end: usize },
    /// More than one column-0 occurrence — ambiguous. Carries the 0-based
    /// key-line indices for the caller's diagnostic.
    Duplicate { key_lines: Vec<usize> },
}

/// Locate the sole top-level `key:` block by indentation scan: the block = the
/// column-0 key line plus every following line up to the next column-0 KEY
/// (non-blank, non-comment) line; the returned span ends after the LAST
/// indented line. Shared by the `process_groups` rewrite, the
/// `process_group_order` removal, and the preview diff (one scanning truth).
///
/// **Column-0 comments and blank lines never TERMINATE a block**:
/// a `# --- divider` between two entries of a block is part of
/// that block — ending the scan there would make a rewrite silently
/// DROP every group after the divider (worse: the orphaned indented
/// survivors would then re-parse as members of the NEW block — the exact
/// corruption class the surgical splice exists to prevent). An INTERIOR
/// divider (a more-indented line follows before the next column-0 key) is
/// consumed WITH the old block on replace — a deliberate choice: the comment
/// annotated entries that no longer exist, and keeping it above
/// freshly-derived groups would be misleading. Trailing comments/blanks
/// (nothing indented follows before the next key) stay OUTSIDE the span,
/// preserved byte-identically.
pub(crate) fn scan_top_level_block(lines: &[Line<'_>], key: &str) -> BlockScan {
    let key_lines: Vec<usize> = lines
        .iter()
        .enumerate()
        .filter(|(_, l)| top_level_key(l.content) == Some(key))
        .map(|(i, _)| i)
        .collect();

    match key_lines.as_slice() {
        [] => BlockScan::Absent,
        [only] => {
            let idx = *only;
            let mut last_content = idx;
            let mut j = idx + 1;
            while j < lines.len() {
                let content = lines[j].content;
                // Blank lines and column-0 comments are NEUTRAL: they join
                // the block only retroactively (when a later indented line
                // extends `last_content` past them); otherwise they stay
                // outside the span.
                if is_blank(content) || content.starts_with('#') {
                    j += 1;
                    continue;
                }
                // A COLUMN-0 block-sequence item belongs to the
                // block too. YAML permits a block sequence to sit at its key's
                // own column, and `serde_yaml` emits exactly that
                // (`nodes:\n- id: …`), so every graph a `serde_yaml` writer
                // produced is in that shape. A scan that `break`s
                // on the first such line reports a block that ends at its
                // own KEY line — and an append then lands between `nodes:` and
                // the first node, at the wrong indentation.
                //
                // Safe for the `process_groups:` / `process_group_order:` /
                // `level_assignments:` callers: those blocks are MAPPINGS, and a
                // top-level document cannot mix a mapping with a root-level
                // sequence, so no valid graph can put a column-0 `- ` line after
                // one of them.
                if starts_with_indent(content) || is_block_sequence_item(content) {
                    last_content = j;
                    j += 1;
                } else {
                    break; // the next column-0 KEY line ends the block
                }
            }
            BlockScan::One {
                start: lines[idx].start,
                end: lines[last_content].end,
            }
        }
        _ => BlockScan::Duplicate { key_lines },
    }
}

/// One physical line: its byte span `[start, end)` (`end` is AFTER the line's
/// `\n`, or EOF for a final newline-less line) plus the content WITHOUT the
/// trailing `\n` (and without a CRLF `\r`).
pub(crate) struct Line<'a> {
    pub(crate) start: usize,
    pub(crate) end: usize,
    pub(crate) content: &'a str,
}

/// Split `raw` into physical lines, preserving exact byte spans so the caller
/// can splice on `\n` boundaries. Handles LF and CRLF; the byte spans always
/// include the terminator so a concatenation of untouched spans is lossless.
pub(crate) fn split_lines(raw: &str) -> Vec<Line<'_>> {
    let mut lines = Vec::new();
    let bytes = raw.as_bytes();
    let mut start = 0usize;
    let mut i = 0usize;
    while i < bytes.len() {
        if bytes[i] == b'\n' {
            let end = i + 1;
            let mut content_end = i;
            if content_end > start && bytes[content_end - 1] == b'\r' {
                content_end -= 1; // drop the CRLF carriage return from `content`
            }
            lines.push(Line {
                start,
                end,
                content: &raw[start..content_end],
            });
            start = end;
        }
        i += 1;
    }
    if start < bytes.len() {
        lines.push(Line {
            start,
            end: bytes.len(),
            content: &raw[start..],
        });
    }
    lines
}

/// The key of a top-level (column-0, non-blank, non-comment) mapping line —
/// the text before the first `:`, trimmed and unquoted. `None` for blank,
/// comment, or indented lines (which never open a top-level element).
pub(crate) fn top_level_key(content: &str) -> Option<&str> {
    let first = content.as_bytes().first().copied()?;
    if first == b' ' || first == b'\t' || first == b'#' {
        return None;
    }
    let key = match content.find(':') {
        Some(colon) => &content[..colon],
        None => content,
    };
    Some(key.trim().trim_matches(|c| c == '"' || c == '\''))
}

/// True when the line is blank (empty or whitespace-only).
pub(crate) fn is_blank(content: &str) -> bool {
    content.trim().is_empty()
}

/// True when the (non-blank) line begins with indentation (space or tab).
pub(crate) fn starts_with_indent(content: &str) -> bool {
    content.starts_with(' ') || content.starts_with('\t')
}

/// True when `content` opens a YAML block-sequence ITEM — `- name: x`, or a
/// bare `-` continuation. Used by [`scan_top_level_block`] to keep a sequence
/// that sits at its key's own column inside that key's block; see the comment
/// at its call site for why that cannot over-extend a mapping block.
pub(crate) fn is_block_sequence_item(content: &str) -> bool {
    let trimmed = content.trim_start();
    trimmed == "-" || trimmed.starts_with("- ")
}

/// Render 0-based line indices as human 1-based line numbers for diagnostics.
pub(crate) fn line_numbers(indices: &[usize]) -> String {
    indices
        .iter()
        .map(|&i| (i + 1).to_string())
        .collect::<Vec<_>>()
        .join(", ")
}
