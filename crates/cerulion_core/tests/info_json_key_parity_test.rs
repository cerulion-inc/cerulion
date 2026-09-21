// SPDX-License-Identifier: AGPL-3.0-only
//! CI hardening: **the cdylib info-JSON emitter and its host
//! parser must agree on the key set.**
//!
//! `cerulion_macros::codegen::gen_cdylib` WRITES the JSON that
//! `DylibNodeEntry::parse_info_json_labeled` READS. The two live in different
//! crates, are edited months apart, and — this is the dangerous part — the
//! parser's structs are all `#[serde(default)]`, so a key the emitter starts
//! writing that the parser never learned about is **not** an error: it is
//! dropped, the field takes its default, and the node runs with the setting
//! silently absent. That is the dropped-policy-key shape (`unbounded_sync` reaching no
//! host arm, a fusion node degrading to `Data`) seen from the other end.
//!
//! The macro-side half of this contract — every ADVERTISED attribute reaches
//! the emitter at all — is
//! `cerulion_macros::codegen::advertised_attribute_parity_tests`, which can
//! run the real emitter and diff its expansion. This file is the other half:
//! whatever the emitter writes, the host must have somewhere to put it.
//!
//! Both sides are read as TEXT because neither is reachable behaviourally
//! from here: `gen_cdylib` lives in a proc-macro crate (not linkable as a
//! dependency of this one), and `parse_info_json` is a private associated fn
//! cfg-gated to `test`/`fuzz-helpers`.

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cerulion_core lives one level under the repo root")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf()
}

/// Emitted keys the host deliberately does NOT parse, each with the reason.
///
/// Empty today, and that is the point: every key the macro writes has a home.
/// An entry here is a claim that a key crosses the FFI and is meant to be
/// ignored — which is exactly the defect this gate exists to catch, so it
/// needs a real argument, not a shrug.
const UNPARSED_BY_DESIGN: &[(&str, &str)] = &[];

/// Comment-stripped view (house pattern; block comments NEST, unterminated
/// fails CLOSED). Load-bearing on BOTH sides: `codegen.rs` and `node.rs` both
/// carry long doc comments that quote JSON fragments (`{"period_ms": 100}` is
/// literally in `PolicyJson`'s docs), so a prose example must not be mistaken
/// for an emitted key or for a parser field.
fn code_only(src: &str) -> String {
    let chars: Vec<char> = src.chars().collect();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < chars.len() {
        if depth > 0 {
            if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
                depth += 1;
                i += 2;
            } else if chars[i] == '*' && chars.get(i + 1) == Some(&'/') {
                depth -= 1;
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
            depth = 1;
            i += 2;
        } else if chars[i] == '/' && chars.get(i + 1) == Some(&'/') {
            while i < chars.len() && chars[i] != '\n' {
                i += 1;
            }
        } else {
            out.push(chars[i]);
            i += 1;
        }
    }
    out
}

#[test]
fn the_comment_stripper_removes_both_syntaxes_and_nothing_else() {
    assert_eq!(code_only("x /* {\"period_ms\": 1} */ y"), "x  y");
    assert_eq!(code_only("/// {\"period_ms\": 1}\ncode\n"), "\ncode\n");
    assert_eq!(code_only("a /* x /* y */ z */ b"), "a  b");
    assert_eq!(code_only("// /* period_ms\nreal\n"), "\nreal\n");
    assert_eq!(code_only("code /* period_ms"), "code ");
}

/// The text between the first occurrence of `from` and the first following
/// occurrence of `to`.
fn slice_between(src: &str, from: &str, to: &str) -> String {
    let start = src
        .find(from)
        .unwrap_or_else(|| panic!("`{from}` not found — this walk is stale"));
    let rest = &src[start..];
    let end = rest
        .find(to)
        .unwrap_or_else(|| panic!("`{to}` not found after `{from}` — this walk is stale"));
    rest[..end].to_string()
}

/// Every `"key":` appearing in the emitter's JSON template literals.
fn emitted_keys(codegen_src: &str) -> BTreeSet<String> {
    // `gen_cdylib` is the whole cdylib emitter; its own test module (appended
    // at the end of the file) must not contribute — those literals are
    // probes, not production output.
    let region = slice_between(codegen_src, "fn gen_cdylib", "#[cfg(test)]");
    let mut out = BTreeSet::new();
    let bytes: Vec<char> = region.chars().collect();
    let mut i = 0usize;
    while i < bytes.len() {
        if bytes[i] != '"' {
            i += 1;
            continue;
        }
        let mut j = i + 1;
        let mut key = String::new();
        while j < bytes.len()
            && (bytes[j].is_ascii_lowercase() || bytes[j].is_ascii_digit() || bytes[j] == '_')
        {
            key.push(bytes[j]);
            j += 1;
        }
        // A JSON key is `"name":` — the quote must close and a colon follow.
        if !key.is_empty() && bytes.get(j) == Some(&'"') && bytes.get(j + 1) == Some(&':') {
            out.insert(key);
        }
        i = j.max(i + 1);
    }
    out
}

/// Every name the host parser can bind a JSON key to: the `serde` struct
/// fields it declares, plus the snake_cased variant names of the
/// `rename_all = "snake_case"` enums it declares.
fn parsed_names(node_src: &str) -> BTreeSet<String> {
    let mut regions = String::new();
    // The local types inside the parser (NodeInfoJson / InputJson /
    // BackpressureJson / OutputJson).
    regions.push_str(&slice_between(
        node_src,
        "fn parse_info_json_labeled",
        "let parsed: NodeInfoJson",
    ));
    // The two file-level types the parser reaches through.
    regions.push_str(&slice_between(
        node_src,
        "pub struct PolicyJson",
        "impl From<&MacroPolicy> for PolicyJson",
    ));

    let mut out = BTreeSet::new();
    for line in regions.lines() {
        let t = line.trim();
        // `name: String,` / `pub period_ms: Option<u64>,`
        let decl = t.strip_prefix("pub ").unwrap_or(t);
        if let Some((lhs, rhs)) = decl.split_once(':') {
            let lhs = lhs.trim();
            if !lhs.is_empty()
                && !rhs.trim().is_empty()
                && lhs
                    .chars()
                    .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
                && rhs
                    .trim()
                    .starts_with(|c: char| c.is_ascii_alphabetic() || c == '&' || c == '(')
            {
                out.insert(lhs.to_string());
            }
        }
        // `DropOldest,` / `Sample(u64),` — snake_case enum variants.
        let variant: String = t
            .chars()
            .take_while(|c| c.is_ascii_alphanumeric())
            .collect();
        if !variant.is_empty()
            && variant.starts_with(|c: char| c.is_ascii_uppercase())
            && t[variant.len()..].starts_with(['(', ',', ' ', '{'])
        {
            let mut snake = String::new();
            for (idx, ch) in variant.chars().enumerate() {
                if ch.is_ascii_uppercase() && idx > 0 {
                    snake.push('_');
                }
                snake.push(ch.to_ascii_lowercase());
            }
            out.insert(snake);
        }
    }
    out
}

/// Every file holding a parser for the cdylib info-JSON document, and how it
/// reports an unknown ENVELOPE key.
///
/// The document has more than one consumer, and for a long time only one of
/// them said anything: `graph::node`'s parser warned, while
/// `cerulion_cli_engine::node_metadata`'s dropped the key in silence — so
/// WHICH tool read your info block decided whether a typo was visible at all.
/// Both warn now, under the same `unknown_key=` field.
///
/// A THIRD parser must not arrive unclassified, which is what the walk below
/// enforces.
const INFO_JSON_PARSERS: &[(&str, &str)] = &[
    (
        "crates/cerulion_core/src/graph/node.rs",
        "the runtime loader (`DylibNodeEntry::parse_info_json_labeled`) — warns per unknown \
         envelope key and per unknown port-entry key",
    ),
    (
        "crates/cerulion_cli_engine/src/node_metadata.rs",
        "the CLI's raw-FFI reader (`try_parse_raw_ffi_node`, behind `cerulion node info` / \
         `node list`) — warns per unknown envelope key",
    ),
];

/// The structured fields that identify the ENVELOPE scan inside a parser file.
///
/// Scoping matters because a file may report at more than one LEVEL. `node.rs`
/// warns for a top-level key AND for a key inside a port entry, and a
/// file-wide "does an `unknown_key` warn exist" check is satisfied by either —
/// so deleting the ENVELOPE scan left this gate green while the very defect
/// the scan was added to fix came back. The envelope warn is the one that carries no
/// `port`, because a top-level key belongs to no port.
const ENVELOPE_WARN_REQUIRES: &[&str] = &["unknown_key"];
const ENVELOPE_WARN_FORBIDS: &[&str] = &["port"];

/// Files that deserialize JSON into a `Deserialize` type declaring BOTH an
/// `inputs` and an `outputs` field — the shape of an info-JSON envelope
/// parser.
///
/// The `serde_json` conjunct is load-bearing, not decoration: `graph/
/// config.rs`'s `NodeDef` carries `inputs` and `outputs` too, and it parses
/// graph YAML, which is a different document with its own (strict) rule.
///
/// SCOPE: this finds a parser that reaches the document through
/// `serde_json::from_str`. That is how both of today's parsers read it; a
/// future one arriving by another route would need adding here, which is why
/// the inventory is checked for staleness in both directions rather than
/// trusted.
fn info_json_parser_files(root: &Path) -> BTreeSet<String> {
    fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(entries) = fs::read_dir(dir) else {
            return;
        };
        for e in entries.flatten() {
            let path = e.path();
            let name = e.file_name();
            let name = name.to_string_lossy();
            if path.is_dir() {
                if name == "target" || name == ".git" || path.join(".git").exists() {
                    continue;
                }
                walk(&path, out);
            } else if name.ends_with(".rs") {
                out.push(path);
            }
        }
    }
    let mut files = Vec::new();
    walk(root, &mut files);
    let mut hits = BTreeSet::new();
    for path in files {
        let Ok(raw) = fs::read_to_string(&path) else {
            continue;
        };
        let code = code_only(&raw);
        if !code.contains("serde_json::from_str") {
            continue;
        }
        // A `Deserialize` item whose body declares both `inputs` and
        // `outputs`. Scoped to a brace-matched body so two unrelated structs
        // in one file cannot combine into a false hit.
        let mut from = 0usize;
        let mut found = false;
        while let Some(hit) = code[from..].find("Deserialize") {
            let at = from + hit;
            from = at + "Deserialize".len();
            let Some(open_rel) = code[at..].find('{') else {
                break;
            };
            let open = at + open_rel;
            let (mut depth, mut close) = (0i32, open);
            for (i, b) in code.bytes().enumerate().skip(open) {
                match b {
                    b'{' => depth += 1,
                    b'}' => {
                        depth -= 1;
                        if depth == 0 {
                            close = i;
                            break;
                        }
                    }
                    _ => {}
                }
            }
            let body = &code[open..close];
            if body.contains("inputs") && body.contains("outputs") {
                found = true;
                break;
            }
        }
        if found {
            if let Ok(rel) = path.strip_prefix(root) {
                hits.insert(rel.to_string_lossy().replace('\\', "/"));
            }
        }
    }
    hits
}

/// `field` appears in `span` as a WHOLE token immediately followed by `=`.
fn field_is_assigned_in(span: &str, field: &str) -> bool {
    let mut at = 0usize;
    while let Some(h) = span[at..].find(field) {
        let i = at + h;
        let before_ok = i == 0
            || !span[..i]
                .chars()
                .next_back()
                .is_some_and(|c| c.is_alphanumeric() || c == '_');
        let after = span[i + field.len()..].trim_start();
        if before_ok && after.starts_with('=') && !after.starts_with("==") {
            return true;
        }
        at = i + field.len();
    }
    false
}

/// Does `src` contain a `tracing::warn!` assigning EVERY field in `requires`
/// and NONE in `forbids`?
///
/// Paren-matched, so a distant mention cannot answer for it, and matching
/// `field =` keeps the warn's own MESSAGE from doing so. `forbids` is what
/// makes a per-LEVEL claim possible in a file that warns at more than one:
/// without it, `node.rs`'s port-entry warn answers for its envelope scan.
fn warn_span_matches(src: &str, requires: &[&str], forbids: &[&str]) -> bool {
    let mut from = 0usize;
    while let Some(hit) = src[from..].find("tracing::warn!(") {
        let open = from + hit + "tracing::warn!(".len();
        let (mut depth, mut end) = (1usize, open);
        for (i, ch) in src[open..].char_indices() {
            match ch {
                '(' => depth += 1,
                ')' => {
                    depth -= 1;
                    if depth == 0 {
                        end = open + i;
                        break;
                    }
                }
                _ => {}
            }
        }
        if depth == 0 {
            let span = &src[open..end];
            if requires.iter().all(|f| field_is_assigned_in(span, f))
                && !forbids.iter().any(|f| field_is_assigned_in(span, f))
            {
                return true;
            }
        }
        from = open;
    }
    false
}

/// A new parser of this document must be classified, not discovered later by
/// a user whose typo it swallowed.
#[test]
fn every_info_json_parser_in_the_repo_is_classified() {
    let root = repo_root();
    let found = info_json_parser_files(&root);
    let declared: BTreeSet<String> = INFO_JSON_PARSERS
        .iter()
        .map(|(f, _)| f.to_string())
        .collect();

    // Vacuity: the walk must still find the two known parsers, or it has gone
    // stale and every claim here is empty.
    for (f, _) in INFO_JSON_PARSERS {
        assert!(
            found.contains(*f),
            "the parser walk no longer finds the known parser {f} — it has gone stale \
             (found: {found:?})"
        );
    }

    // A classification is a CLAIM, not a label. Without this, deleting the
    // warn from a listed parser left the gate green — the entry said the file
    // reports unknown keys and nothing checked whether it still did, which is
    // precisely the defect the classification exists to rule out.
    let mut silent: Vec<String> = Vec::new();
    for (file, what) in INFO_JSON_PARSERS {
        let src = fs::read_to_string(root.join(file))
            .unwrap_or_else(|e| panic!("declared parser {file} is unreadable: {e}"));
        if !warn_span_matches(
            &code_only(&src),
            ENVELOPE_WARN_REQUIRES,
            ENVELOPE_WARN_FORBIDS,
        ) {
            silent.push(format!("  {file} — declared as: {what}"));
        }
    }
    assert!(
        silent.is_empty(),
        "a declared info-JSON parser no longer reports unknown keys:\n{}\n\nIts entry in \
         `INFO_JSON_PARSERS` claims it does. Restore the `tracing::warn!` carrying \
         `unknown_key = ...`, or reclassify the parser and say what reports its keys instead.",
        silent.join("\n")
    );

    let unclassified: Vec<&String> = found.difference(&declared).collect();
    assert!(
        unclassified.is_empty(),
        "file(s) parse the cdylib info-JSON envelope but are not classified: {unclassified:?}\n\n\
         Every consumer of this document must REPORT an unknown envelope key — a parser that \
         drops one in silence means which tool read your info block decides whether a typo is \
         visible.\n\nFIX: add the unknown-key warn (see `node_metadata::try_parse_raw_ffi_node`) \
         and list the file in `INFO_JSON_PARSERS` with what it does."
    );
}

/// The field names declared INSIDE one brace-delimited region of `src`,
/// starting at the first `{` after `head`.
///
/// Used to read one parser type at a time, because a flat name set across all
/// of them cannot answer an OWNERSHIP question (see the gate below).
fn fields_of(src: &str, head: &str) -> BTreeSet<String> {
    let start = src
        .find(head)
        .unwrap_or_else(|| panic!("`{head}` not found in node.rs — this walk is stale"));
    let rest = &src[start..];
    let open = rest
        .find('{')
        .unwrap_or_else(|| panic!("`{head}` has no body — this walk is stale"));
    let mut depth = 0i32;
    let mut end = rest.len();
    for (idx, ch) in rest[open..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    end = open + idx;
                    break;
                }
            }
            _ => {}
        }
    }
    let mut out = BTreeSet::new();
    for line in rest[open..end].lines() {
        let t = line.trim();
        let decl = t.strip_prefix("pub ").unwrap_or(t);
        let Some((lhs, rhs)) = decl.split_once(':') else {
            continue;
        };
        let lhs = lhs.trim();
        if !lhs.is_empty()
            && !rhs.trim().is_empty()
            && lhs
                .chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
        {
            out.insert(lhs.to_string());
        }
    }
    out
}

/// The `&["a", "b"]` string entries of the named `const … : &[&str]` in `src`.
fn legal_key_list(src: &str, konst: &str) -> BTreeSet<String> {
    let start = src
        .find(konst)
        .unwrap_or_else(|| panic!("`{konst}` not found in node.rs — this walk is stale"));
    let rest = &src[start..];
    let end = rest.find("];").unwrap_or_else(|| {
        panic!("`{konst}` is not a terminated slice literal — this walk is stale")
    });
    let mut out = BTreeSet::new();
    let mut chars = rest[..end].chars().peekable();
    while let Some(c) = chars.next() {
        if c != '"' {
            continue;
        }
        let mut key = String::new();
        for c in chars.by_ref() {
            if c == '"' {
                break;
            }
            key.push(c);
        }
        if !key.is_empty() {
            out.insert(key);
        }
    }
    out
}

/// A section's LEGAL key list and its parser type's fields must agree — in
/// BOTH directions.
///
/// The sibling gate below asks a set question — "does every emitted key have
/// a parser field SOMEWHERE" — and a flat set erases ownership. `schema_hash`
/// is declared by BOTH `InputJson::Full` and `OutputJson::Full`, so deleting
/// it from the input side leaves the flat check green while every input's
/// schema hash is silently ignored and defaulted. Same for `name`.
///
/// Scoping the question per SECTION is what fixes that, and the sections are
/// not invented here: `LEGAL_INPUT_KEYS` / `LEGAL_OUTPUT_KEYS` /
/// `LEGAL_ENVELOPE_KEYS` are the lists the runtime's own unknown-key warn is
/// driven by. So this also pins a second property the flat check could not
/// see — a key on a legal list that no field binds would be accepted WITHOUT
/// a warning and then dropped, which is the worst of both behaviours.
#[test]
fn every_legal_key_and_its_section_s_parser_fields_agree_both_ways() {
    let root = repo_root();
    let node = code_only(
        &fs::read_to_string(root.join("crates/cerulion_core/src/graph/node.rs"))
            .expect("read node.rs"),
    );

    let sections: [(&str, &str, &str); 3] = [
        ("envelope", "LEGAL_ENVELOPE_KEYS", "struct NodeInfoJson"),
        ("input entry", "LEGAL_INPUT_KEYS", "enum InputJson"),
        ("output entry", "LEGAL_OUTPUT_KEYS", "enum OutputJson"),
    ];

    let mut unbound: Vec<String> = Vec::new();
    for (label, konst, ty) in sections {
        let legal = legal_key_list(&node, konst);
        let fields = fields_of(&node, ty);
        assert!(
            legal.len() >= 4 && !fields.is_empty(),
            "the {label} walk came back implausibly small (legal={legal:?}, fields={fields:?}) \
             — it has gone stale and every check below would be vacuous"
        );
        for key in &legal {
            if !fields.contains(key) {
                unbound.push(format!(
                    "  `{key}` is on {konst} but no field of `{ty}` binds it ({label})"
                ));
            }
        }
        // The OTHER direction, and it is not symmetric decoration: a field
        // the parser binds but the legal list omits is a key the runtime
        // will WARN about on every single load — a legitimate new
        // `NodeInfoJson` field would make every macro-emitted cdylib log a
        // FALSE "unknown key" for a key it was right to emit. One-directional
        // drift checking cannot see that.
        for field in &fields {
            if !legal.contains(field) {
                unbound.push(format!(
                    "  `{field}` is a field of `{ty}` but is missing from {konst} ({label}), so \
                     every document carrying it is warned about by mistake"
                ));
            }
        }
    }

    assert!(
        unbound.is_empty(),
        "a key the runtime calls LEGAL is not bound by its section's parser type, so it is \
         accepted without a warning and then silently dropped:\n{}\n\nFIX: add the field to \
         that type, or remove the key from the legal list so a document carrying it is \
         reported.",
        unbound.join("\n")
    );
}

#[test]
fn every_info_json_key_the_macro_emits_has_a_host_parser_field() {
    let root = repo_root();
    let codegen = code_only(
        &fs::read_to_string(root.join("crates/cerulion_macros/src/codegen.rs"))
            .expect("read codegen.rs"),
    );
    let node = code_only(
        &fs::read_to_string(root.join("crates/cerulion_core/src/graph/node.rs"))
            .expect("read node.rs"),
    );

    let emitted = emitted_keys(&codegen);
    let parsed = parsed_names(&node);
    let exempt: BTreeSet<&str> = UNPARSED_BY_DESIGN.iter().map(|(k, _)| *k).collect();

    // Vacuity guards: both halves must be recognisably populated, or the
    // subset check below passes because one side came back empty.
    assert!(
        emitted.len() >= 15,
        "only {} info-JSON key(s) were read out of `gen_cdylib` — the emitter walk has \
         broken, which would make this gate vacuous (found: {emitted:?})",
        emitted.len()
    );
    assert!(
        parsed.len() >= 15,
        "only {} parser name(s) were read out of `parse_info_json_labeled` / `PolicyJson` — \
         the parser walk has broken (found: {parsed:?})",
        parsed.len()
    );
    let orphans: Vec<String> = emitted
        .iter()
        .filter(|k| !parsed.contains(*k) && !exempt.contains(k.as_str()))
        .map(|k| format!("  `\"{k}\":` is written by `gen_cdylib` but no host parser field or variant binds it"))
        .collect();
    assert!(
        orphans.is_empty(),
        "the cdylib info-JSON emitter and its host parser have drifted:\n{}\n\nEvery parser \
         struct is `#[serde(default)]`, so an unbound key is DROPPED silently and the node \
         runs with the setting absent (the dropped-policy-key shape). FIX: add the field to the \
         matching struct in `cerulion_core/src/graph/node.rs` \
         (`NodeInfoJson`/`InputJson::Full`/`OutputJson::Full`/`PolicyJson`), or — only if \
         the key genuinely must be ignored — record it in `UNPARSED_BY_DESIGN` with the \
         reason.",
        orphans.join("\n")
    );

    let stale: Vec<String> = UNPARSED_BY_DESIGN
        .iter()
        .filter(|(k, _)| !emitted.contains(*k) || parsed.contains(*k))
        .map(|(k, _)| format!("  `{k}`"))
        .collect();
    assert!(
        stale.is_empty(),
        "STALE `UNPARSED_BY_DESIGN` entr(ies) — no longer emitted, or now parsed after \
         all:\n{}\n\nFIX: delete the entry.",
        stale.join("\n")
    );

    // Anchored last, so a genuine emitter/parser divergence is diagnosed
    // above by name rather than being reported here as a broken walk. What
    // reaches this point is a walk that returned a plausible-sized set with
    // the wrong CONTENT — e.g. one that started collecting Rust identifiers
    // instead of JSON keys, which would make the subset check pass by
    // coincidence.
    for anchor in ["policy", "tick_within_ms", "backpressure", "schema_hash"] {
        assert!(
            emitted.contains(anchor),
            "`{anchor}` is missing from the emitted-key set — either `gen_cdylib` stopped \
             emitting it (a real regression) or the emitter walk is reading the wrong \
             region (found: {emitted:?})"
        );
        assert!(
            parsed.contains(anchor),
            "`{anchor}` is missing from the parser-name set — either the host parser \
             dropped the field (a real regression) or the parser walk is reading the wrong \
             region (found: {parsed:?})"
        );
    }
}
