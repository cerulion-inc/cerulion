// SPDX-License-Identifier: AGPL-3.0-only
//! The structural guard that no production code
//! path reads a `GraphConfig`'s deprecated `name:` key for IDENTITY.
//!
//! # Why a source walk and not a behavioural test
//!
//! The rule's behaviour is pinned end to end elsewhere: `graph::mod`'s unit
//! arms drive the divergence warn and the stemless seed, `graph_cmd`'s engine
//! arms drive the bag stem and the ring tags, and
//! `mp_record_e2e_test::mp_record_stems_the_bag_by_the_file_not_the_declared_name`
//! drives the real binary. What none of them can see is a SINGLE site that
//! quietly reads `config.name` again.
//!
//! That is not hypothetical, and the shape of the field is why: `name` is
//! `Option<String>` while `identity()` returns `&str`, so most re-introductions
//! are compile errors — but `%config.name` in a `tracing` field, a
//! `{:?}`-formatted diagnostic, or a `config.name.as_deref().unwrap_or(..)`
//! fallback all compile cleanly and put the ignored key back in charge of a
//! surface. No behavioural test fails when one such read is re-introduced.
//!
//! # What is walked, and how
//!
//! A COMMENT- and STRING-LITERAL-stripped view (`code_only`) of every
//! production module that handles the field. Stripping is load-bearing in
//! BOTH directions here: `config.rs`'s own doc comments name the field dozens
//! of times to explain the deprecation, and the divergence warn's message text
//! is a string literal mentioning `name:`. A naive `contains` scan would be
//! satisfied — or tripped — by prose.
//!
//! The walk allows EXACTLY the two sanctioned touch points, and it locates them
//! by FUNCTION BODY rather than by file, so "somewhere in `graph/mod.rs`" is
//! not good enough: the seed must be inside `parse_graph_raw` and the
//! divergence warn inside `adopt_file_stem_identity`.

use std::path::{Path, PathBuf};

/// Production modules that read `GraphConfig::name` before the file-stem identity rule.
///
/// A LIST rather than a directory walk, deliberately and narrowly: `.name` is
/// the commonest field name in the tree (`NodeDef`, `InputDef`, `OutputDef`,
/// every attachment record, every clap arg), so a whole-tree scan would be
/// almost entirely false positives. What makes the list safe is that it is
/// checked against the ORIGINAL reader set — every file that had a read is
/// here, and the anti-tautology arm proves each one is still reached.
const WALKED: &[&str] = &[
    "cerulion_core/src/graph/mod.rs",
    "cerulion_core/src/graph/runtime.rs",
    "cerulion_core/src/graph/partition.rs",
    "cerulion_core/src/graph/topology.rs",
    "cerulion_core/src/graph/validation.rs",
    "cerulion_cli_engine/src/graph_cmd.rs",
    "cerulion_cli_engine/src/multiprocess.rs",
    "cerulion_cli_engine/src/partition_emit.rs",
    "cerulion_cli_engine/src/replay_engine.rs",
    // `replay_rank` is a NEW `GraphConfig` holder (it restricts a rank's
    // subgraph and names the graph in two of its refusals), so it belongs on
    // the reader set for the same reason `replay_engine` does.
    "cerulion_cli_engine/src/replay_rank.rs",
];
// `run_dir.rs` is deliberately ABSENT: it never holds a `GraphConfig` at all
// (it takes the resolved `graph_name: &str`), so it had no read to lose and
// listing it would make the anti-tautology arm below assert something untrue
// of it.

/// The two functions allowed to touch the deprecated key, and nothing else.
const SANCTIONED: &[(&str, &str)] = &[
    // SEEDS the identity for a parse with no file to stem from.
    ("cerulion_core/src/graph/mod.rs", "fn parse_graph_raw("),
    // WARNS when the key disagrees with the stem, then overwrites.
    (
        "cerulion_core/src/graph/mod.rs",
        "fn adopt_file_stem_identity(",
    ),
];

fn repo_root() -> PathBuf {
    // `cerulion_cli_engine/tests/` → the workspace root.
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("workspace root")
        .to_path_buf()
}

fn read(rel: &str) -> String {
    let p = repo_root().join(rel);
    std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("read {}: {e}", p.display()))
}

/// The body of the first `fn` whose signature starts with `sig`, brace-matched.
///
/// Brace-matched rather than "to the next `};`", for a reason learned earlier: a
/// textual terminator is moved by any closure or nested block before the line
/// under test, silently shrinking the slice an assertion is enforced over.
fn fn_body(code: &str, sig: &str) -> Option<String> {
    let at = code.find(sig)?;
    let open = at + code[at..].find('{')?;
    let bytes = code.as_bytes();
    let mut depth = 0usize;
    for (i, b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(code[open..=i].to_string());
                }
            }
            _ => {}
        }
    }
    None
}

/// Every way a `GraphConfig`'s deprecated key can be READ in compiling code.
///
/// `config.name` / `self.config.name` / `cfg.name` cover the receivers the
/// original 118 reads used; the bare `.name` forms are deliberately NOT scanned
/// (they match `node.name`, `output.name`, `a.name` — the field is ubiquitous),
/// which is why the WALKED list is scoped to modules that genuinely held one.
const READ_FORMS: &[&str] = &[
    "config.name",
    "cfg.name",
    "subgraph.name",
    "effective_config.name",
];

/// The outcome of stripping comments: the code-only view, plus whether the scan
/// finished BALANCED.
struct Stripped {
    code: String,
    /// Depth of unclosed `/*` at EOF. Anything but 0 means the tail was silently
    /// dropped — see [`code_only`].
    unclosed_depth: usize,
}

/// A view of `src` with `//`-to-end-of-line and `/* … */` comments removed.
///
/// Three things it models, each because the alternative silently loses coverage:
///
/// * Block comments NEST in Rust, so the scan is depth-tracked.
/// * STRING LITERALS are skipped — regular, byte (`b"…"`) and RAW (`r#"…"#`).
///   The target file holds `"a /* unterminated b"` inside its
///   OWN comment-stripper oracle test, so a literal-blind scan opened a block comment
///   at line 4133 that never closed and dropped **42 % of the file**, leaving every
///   negative assertion enforced over the surviving prefix only.
/// * CHAR literals holding a QUOTE (`'"'`). Skipping these on the grounds that
///   "a `/*` cannot fit in a char literal" is true, and the wrong hazard. The risk is
///   the opposite direction: a lone `"` inside a char literal opens the STRING arm,
///   which then scans to the next `"` anywhere in the file and deletes everything
///   between — and because that leaves `depth == 0`, the loud unclosed-depth guard
///   below cannot see it. Only `'x'` / `'\x'` shapes are consumed (a closing quote
///   within four bytes), so a LIFETIME (`&'static str`) is never mistaken for one.
/// * The unclosed depth is REPORTED, so a caller can fail LOUDLY rather than let a
///   truncated view masquerade as a clean one.
fn code_only(src: &str) -> Stripped {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < bytes.len() {
        if depth == 0 {
            // A CHAR literal — but ONLY the short shapes, so a lifetime is never
            // eaten. `'x'` is 3 bytes, `'\n'` / `'\''` are 4; a lifetime has no
            // closing quote within that window.
            if bytes[i] == b'\'' {
                let close = if bytes.get(i + 1) == Some(&b'\\') {
                    // Escaped: `'\x'` — the closer is at i+3.
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
                    // Closing delimiter: `"` followed by the same number of `#`.
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
            // Push the whole UTF-8 character, never a partial byte.
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

/// THE guard: outside the two sanctioned load-path functions, no production
/// module reads the deprecated key.
///
/// Test modules are excluded by construction — `#[cfg(test)]` blocks legitimately
/// assert the key round-trips, which is the back-compat half of the decision — so
/// the slice walked is everything ABOVE the first `#[cfg(test)]`.
#[test]
fn no_production_path_reads_the_deprecated_graph_name_for_identity() {
    let mut offenders: Vec<String> = Vec::new();

    for rel in WALKED {
        let src = read(rel);
        let stripped = code_only(&src);
        assert_eq!(
            stripped.unclosed_depth, 0,
            "{rel}: the comment stripper ended UNBALANCED, so the view is a \
             truncated prefix and every assertion below it is vacuous"
        );
        let mut code = stripped.code;
        // Production half only.
        if let Some(at) = code.find("#[cfg(test)]") {
            code.truncate(at);
        }
        // Blank out the sanctioned bodies, so what survives is everything else.
        for (file, sig) in SANCTIONED {
            if file != rel {
                continue;
            }
            let body = fn_body(&code, sig).unwrap_or_else(|| {
                panic!("{rel}: the sanctioned fn `{sig}` must still exist to be excluded")
            });
            code = code.replacen(&body, " ", 1);
        }
        for (i, line) in code.lines().enumerate() {
            if READ_FORMS.iter().any(|f| line.contains(f)) {
                offenders.push(format!("{rel}: {}", line.trim()));
                let _ = i;
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "project rule: `name:` is optional-and-IGNORED, so no production path may \
         read it for identity — read `GraphConfig::identity()` instead. Offending lines:\n{}",
        offenders.join("\n")
    );
}

/// ANTI-TAUTOLOGY: the walk really reaches live code in every file it lists.
///
/// Without this, a mistyped path, a stripper that swallowed a file, or a
/// `#[cfg(test)]` that moved to line 1 would each make the guard above pass
/// while inspecting nothing, the known failure mode of this
/// exact class of test.
#[test]
fn the_walk_reaches_live_code_in_every_module_it_lists() {
    for rel in WALKED {
        let src = read(rel);
        let stripped = code_only(&src);
        assert_eq!(stripped.unclosed_depth, 0, "{rel}: unbalanced stripper");
        let mut code = stripped.code;
        if let Some(at) = code.find("#[cfg(test)]") {
            code.truncate(at);
        }
        assert!(
            code.contains("identity()")
                || code.contains("identity:")
                || code.contains("identity ="),
            "{rel}: the production slice must still USE the resolved identity — if it does \
             not, either the file no longer belongs on this list or the walk is inspecting \
             the wrong text"
        );
    }
}

/// The two sanctioned touch points are located by BODY, not by file — so a read
/// that moved out of them is caught even while staying in `graph/mod.rs`.
#[test]
fn the_sanctioned_reads_live_inside_the_two_load_path_functions() {
    let code = code_only(&read("cerulion_core/src/graph/mod.rs")).code;

    let seed = fn_body(&code, "fn parse_graph_raw(").expect("parse_graph_raw must exist");
    assert!(
        seed.contains("config.name"),
        "`parse_graph_raw` must SEED the identity from the legacy key — without it a bag's \
         embedded `graph.yaml` (which has no file to stem from) loses its name entirely"
    );

    let adopt = fn_body(&code, "fn adopt_file_stem_identity(")
        .expect("adopt_file_stem_identity must exist");
    assert!(
        adopt.contains("config.name"),
        "`adopt_file_stem_identity` must COMPARE the legacy key against the stem — without \
         it the deprecation is silent and a renamed graph never tells anyone"
    );
    assert!(
        adopt.contains("config.identity ="),
        "…and must then OVERWRITE the identity with the stem"
    );
}

/// The worker RESTORES the identity its plan carries — pinned structurally,
/// because the behaviour needs a real supervisor, a real plan file and a real
/// transport.
///
/// `WorkerPlan::graph_identity` crossing the wire is pinned purely
/// (`multiprocess::tests::a_workers_identity_survives_the_plan_json_hop`),
/// but a carried value nobody READS is inert: every multi-process worker would
/// log `graph=unnamed` for the whole run on the DEFAULT Unix path, with the
/// pure arm still green. The two halves together are what make the hop real.
#[test]
fn the_worker_restores_the_identity_its_plan_carried() {
    let code = code_only(&read("cerulion_cli_engine/src/graph_cmd.rs")).code;
    let body = fn_body(&code, "pub fn graph_run_worker(").expect("graph_run_worker must exist");
    assert!(
        body.contains("config.identity = plan.graph_identity"),
        "`graph run-worker` must restore the identity its plan carried — \
         `GraphConfig::identity` is `serde(skip)`, so the subgraph arrives without one"
    );
}

/// The stripper's own oracle. It is what every negative assertion above is
/// enforced through, so a stripper that answers too easily makes the guard
/// vacuous WITHOUT failing anything — which is how this class of hole ships.
#[test]
fn the_stripper_removes_comments_and_literals_and_nothing_else() {
    let strip = |s: &str| code_only(s).code;
    assert_eq!(strip("let a = 1; // config.name\n"), "let a = 1; \n");
    assert_eq!(strip("a /* config.name */ b"), "a  b");
    assert_eq!(strip("a /* x /* config.name */ z */ b"), "a  b");
    assert_eq!(strip("let s = \"config.name\";"), "let s =  ;");
    assert_eq!(strip("let s = r#\"config.name\"#;"), "let s =  ;");
    assert_eq!(strip("let c = '\"'; config.name"), "let c =  ; config.name");
    // A lifetime is NOT a char literal.
    assert!(strip("fn f<'a>(x: &'a str) {}").contains("&'a str"));
    // Unbalanced input fails CLOSED (reported, not silently truncated).
    assert!(code_only("a /* b").unclosed_depth > 0);
}
