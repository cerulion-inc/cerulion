// SPDX-License-Identifier: AGPL-3.0-only
//! Find a YAML anchor (`&name`) or alias (`*name`) in raw text.
//!
//! The YAML parser expands aliases silently, so the codec looks for them
//! in the text before parsing. YAML forbids a plain scalar from STARTING
//! with `&` or `*`, so either character at the start of a node is an
//! anchor or an alias. The scanner tracks quoted scalars, comments, flow
//! brackets and block scalars (`|`, `>`) to know where a node starts.
//!
//! One shape reads as a false refusal: a multi-line plain scalar whose
//! continuation line starts with `&` or `*`. Quoting the string avoids it;
//! the encoder always quotes strings.

/// The 1-based `(line, column)` of the first anchor or alias in `text`.
pub(super) fn find_anchor_or_alias(text: &str) -> Option<(usize, usize)> {
    let mut flow_depth = 0usize;
    let mut quote: Option<char> = None;
    // Column of the node that owns an open block scalar (the key of
    // `key: |`, the key after the dash of `- key: |`, the dash of `- |`);
    // its content lines (more indented, or blank) are skipped. The line's
    // indent is NOT that column: in `- key: |` a sibling key sits at the
    // key's column, past the dash, and must still be scanned.
    let mut block_scalar_parent: Option<usize> = None;

    for (line_idx, line) in text.split('\n').enumerate() {
        let line = line.strip_suffix('\r').unwrap_or(line);
        let indent = line.len() - line.trim_start_matches(' ').len();
        if let Some(parent) = block_scalar_parent {
            if line.trim().is_empty() || indent > parent {
                continue;
            }
            block_scalar_parent = None;
        }

        let chars: Vec<(usize, char)> = line.char_indices().collect();
        // True where the next non-blank character starts a node.
        let mut node_start = quote.is_none();
        // Column of the last node or indicator that started on this line.
        let mut last_node_col: Option<usize> = None;
        // True on a line that opens with a document marker: a node after
        // `---` is at structural column 0, whatever its physical column.
        let mut after_marker = false;
        // True right after a quoted scalar or a flow collection closed
        // (blanks aside): a `:` there is a value indicator even with no
        // blank after it (`{"a":*x}`), as YAML allows for a JSON-like key.
        let mut after_close = false;
        // True inside a plain scalar (up to a blank): in a flow collection a
        // `:` there is text unless a blank or a flow indicator follows, and
        // anywhere else it is a value indicator.
        let mut in_plain = false;
        let mut i = 0;
        // Document markers: `---` / `...` followed by a blank or the end.
        if quote.is_none() && (line.starts_with("---") || line.starts_with("...")) {
            let rest = &line[3..];
            if rest.is_empty() || rest.starts_with([' ', '\t']) {
                i = chars
                    .iter()
                    .position(|&(b, _)| b >= 3)
                    .unwrap_or(chars.len());
                after_marker = true;
            }
        }
        while i < chars.len() {
            let (byte, c) = chars[i];
            let next = chars.get(i + 1).map(|&(_, n)| n);
            if let Some(q) = quote {
                match (q, c) {
                    ('"', '\\') => i += 1,
                    ('"', '"') => {
                        quote = None;
                        after_close = true;
                    }
                    ('\'', '\'') if next == Some('\'') => i += 1,
                    ('\'', '\'') => {
                        quote = None;
                        after_close = true;
                    }
                    _ => {}
                }
                i += 1;
                continue;
            }
            let blank_before = byte == 0 || matches!(line[..byte].chars().last(), Some(' ' | '\t'));
            if c == '#' && blank_before {
                break;
            }
            if c == ' ' || c == '\t' {
                in_plain = false;
                i += 1;
                continue;
            }
            let ends_indicator = matches!(next, None | Some(' ' | '\t'));
            let closed_before = std::mem::take(&mut after_close);
            if node_start {
                let col = if after_marker {
                    0
                } else {
                    line[..byte].chars().count()
                };
                match c {
                    '&' | '*' => return Some((line_idx + 1, line[..byte].chars().count() + 1)),
                    '"' | '\'' => {
                        quote = Some(c);
                        node_start = false;
                        last_node_col = Some(col);
                    }
                    '-' | '?' if ends_indicator => last_node_col = Some(col),
                    ':' if ends_indicator => {}
                    '[' | '{' => flow_depth += 1,
                    ']' | '}' => {
                        flow_depth = flow_depth.saturating_sub(1);
                        node_start = false;
                        after_close = true;
                    }
                    ',' if flow_depth > 0 => {}
                    '!' => {
                        // A tag: skip to the next blank; a node follows.
                        while i + 1 < chars.len() && !matches!(chars[i + 1].1, ' ' | '\t') {
                            i += 1;
                        }
                    }
                    '|' | '>' => {
                        block_scalar_parent = Some(last_node_col.unwrap_or(indent));
                        break;
                    }
                    _ => {
                        node_start = false;
                        in_plain = true;
                        last_node_col = Some(col);
                    }
                }
            } else {
                match c {
                    ':' if ends_indicator
                        || closed_before
                        || (flow_depth > 0
                            && (!in_plain
                                || matches!(next, Some(',' | ']' | '}' | '[' | '{')))) =>
                    {
                        node_start = true;
                        in_plain = false;
                    }
                    ',' if flow_depth > 0 => {
                        node_start = true;
                        in_plain = false;
                    }
                    '[' | '{' if flow_depth > 0 => {
                        flow_depth += 1;
                        node_start = true;
                        in_plain = false;
                    }
                    ']' | '}' if flow_depth > 0 => {
                        flow_depth -= 1;
                        in_plain = false;
                        after_close = true;
                    }
                    _ => in_plain = true,
                }
            }
            i += 1;
        }
    }
    None
}
