// SPDX-License-Identifier: AGPL-3.0-only
//! The no-inert-shipping guard for the plan-time split-pair warn.
//!
//! The detector and the reporter are `pub` items of a LIBRARY crate, so
//! `dead_code = deny` cannot see them go unused: deleting the one call site in
//! `graph_run_supervisor` leaves every unit test in `multiprocess.rs` green
//! while the feature ships completely inert. This file is the arm that fails.
//!
//! It asserts four things about the SOURCE of `graph_cmd.rs`:
//!
//! 1. `graph_run_supervisor` calls `warn_split_pair_report`;
//! 2. the call sits BEFORE the worker-spawn loop (a warn emitted after the
//!    children are already running is not a plan-time warn);
//! 3. it is inside the multi-process supervisor, which
//!    `resolve_deployment` never routes an unpartitioned, `--single-process`
//!    or non-Unix run to — that placement is the whole reason the warn cannot
//!    fire on a monolith, and no runtime test can observe it; and
//! 4. **PROVENANCE** — the detector is handed the metadata the runtime
//!    LOADED, not the source-parsed metadata `validate_partition` reads. The
//!    `SourceNodeMetadata` / `LoadedNodeMetadata` newtypes make a SWAP a
//!    compile error, but they cannot see a call that wraps the SOURCE map in
//!    `LoadedNodeMetadata(..)` — same underlying type, compiles cleanly,
//!    silently restores the drift bug. Only a source walk can catch that, and
//!    only here: no runtime test can build a graph whose cdylibs disagree
//!    with their own source; and
//! 5. **ORDERING** — the stale-build warn precedes the `?`-propagated
//!    `validate_partition`, so a source-vs-loaded disagreement is named even
//!    on the path where source-based validation REFUSES. Same reason no
//!    runtime test can see it.
//!
//! # The stripper
//!
//! [`code_only`] removes line comments, NESTING block comments, and string /
//! raw-string / char literals. All three literal forms are load-bearing rather
//! than defensive: `graph_cmd.rs` really does contain `/*` inside string
//! literals (MEASURED — a comments-only stripper ends the file at block depth
//! 4 and swallows real code), a `'"'` char literal would otherwise open the
//! string arm and delete everything to the next quote, and `r#"…"#` bodies
//! contain bare quotes.
//!
//! It is a PORT of the same helper in `convergence_adoption_test.rs`,
//! duplicated because integration tests are separate binaries and this crate
//! has no shared test-support target. Both copies carry their own oracle test
//! (`code_only_strips_comments_and_literals_and_nothing_else` below), so
//! neither can rot silently into the other.

use std::path::PathBuf;

/// The split-pair advisory the supervisor must emit before spawning workers.
const REPORTER_CALL: &str = "crate::multiprocess::report_split_same_level_non_trigger_pairs(";
/// The split-pair classifier — takes ONLY the loaded metadata.
const CLASSIFY_CALL: &str = "crate::multiprocess::classify_split_same_level_non_trigger_pairs(";
/// The stale-build reporter, which must precede the refusing validator.
const DRIFT_REPORTER_CALL: &str = "crate::multiprocess::warn_trigger_classification_drift(";
/// The stale-build cross-check — the one seam that takes BOTH provenances.
const DRIFT_CLASSIFY_CALL: &str = "crate::multiprocess::classify_trigger_metadata_drift(";
/// The partition validator, which is `?`-propagated and can abort the run.
const VALIDATE_CALL: &str = "cerulion_core::graph::validate_partition(";
/// The supervisor function that owns the multi-process deployment.
const SUPERVISOR_FN: &str = "fn graph_run_supervisor(";
/// The worker-spawn loop — everything the warn must precede.
const SPAWN_LOOP: &str = "crate::multiprocess::spawn_order(";

fn graph_cmd_src() -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("graph_cmd.rs");
    std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("could not read {}: {e}", path.display()))
}

/// Strip comments (line + NESTING block) and string / raw-string / char
/// literals. Returns the stripped view plus the unclosed block depth, so a
/// caller can refuse a view it cannot trust. Every literal becomes a single
/// space, so token boundaries survive.
fn code_only(src: &str) -> (String, usize) {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < bytes.len() {
        if depth == 0 {
            // A CHAR literal — only the short shapes, so a LIFETIME (which has
            // no closing quote in that window) is never eaten.
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
            }
            // A RAW string: `r`, zero or more `#`, then `"`.
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
        if depth > 0 {
            if bytes[i..].starts_with(b"*/") {
                depth -= 1;
                i += 2;
            } else {
                i += 1;
            }
            continue;
        }
        let ch = src[i..].chars().next().expect("valid utf-8 boundary");
        out.push(ch);
        i += ch.len_utf8();
    }
    (out, depth)
}

#[test]
fn the_supervisor_reports_split_non_trigger_pairs_before_spawning_workers() {
    let (code, depth) = code_only(&graph_cmd_src());
    assert_eq!(
        depth, 0,
        "unbalanced block comment — the stripped view is untrustworthy"
    );

    let supervisor = code
        .find(SUPERVISOR_FN)
        .unwrap_or_else(|| panic!("`{SUPERVISOR_FN}` not found — did the fn get renamed?"));
    let spawn = code[supervisor..]
        .find(SPAWN_LOOP)
        .map(|o| supervisor + o)
        .unwrap_or_else(|| panic!("`{SPAWN_LOOP}` not found after the supervisor fn"));
    let call = code[supervisor..]
        .find(REPORTER_CALL)
        .map(|o| supervisor + o);

    let Some(call) = call else {
        panic!(
            "the split-partition warn is INERT: `graph_run_supervisor` never calls \
             `{REPORTER_CALL}`. The detector and reporter are `pub` library items, so \
             nothing else can notice — every unit test in `multiprocess.rs` stays green \
             while a split same-level non-trigger partition runs silently. Restore the \
             call beside the pre-spawn `validate_partition`."
        );
    };
    assert!(
        call < spawn,
        "the split-partition warn is emitted AFTER the worker-spawn loop — it must be a \
         PLAN-time warn (before any child exists), not a post-hoc note"
    );
}

/// Return the balanced-paren argument list of the call starting at `call`
/// (which must point at the opening of `…(`), split on DEPTH-0 commas with
/// whitespace collapsed.
fn call_args(code: &str, call: usize) -> Vec<String> {
    let open = call + code[call..].find('(').expect("call has an open paren");
    let bytes = code.as_bytes();
    let mut depth = 0usize;
    let mut end = None;
    for (i, b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'(' | b'[' | b'{' => depth += 1,
            b')' | b']' | b'}' => {
                depth -= 1;
                if depth == 0 {
                    end = Some(i);
                    break;
                }
            }
            _ => {}
        }
    }
    let end = end.expect("call's argument list is unbalanced");
    let inner = &code[open + 1..end];

    let mut args = Vec::new();
    let mut depth = 0usize;
    let mut start = 0usize;
    for (i, b) in inner.as_bytes().iter().enumerate() {
        match b {
            b'(' | b'[' | b'{' | b'<' => depth += 1,
            b')' | b']' | b'}' | b'>' => depth = depth.saturating_sub(1),
            b',' if depth == 0 => {
                args.push(
                    inner[start..i]
                        .split_whitespace()
                        .collect::<Vec<_>>()
                        .join(" "),
                );
                start = i + 1;
            }
            _ => {}
        }
    }
    let tail: String = inner[start..]
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ");
    if !tail.is_empty() {
        args.push(tail);
    }
    args
}

/// Pull the single identifier out of `Wrapper(&ident)`.
fn newtype_binding(arg: &str, wrapper: &str) -> String {
    let at = arg
        .find(wrapper)
        .unwrap_or_else(|| panic!("argument `{arg}` does not wrap `{wrapper}`"));
    let rest = &arg[at + wrapper.len()..];
    rest.trim_start_matches(['(', '&', ' '])
        .split(|c: char| !(c.is_alphanumeric() || c == '_'))
        .next()
        .unwrap_or_default()
        .to_string()
}

/// THE PROVENANCE ARM. `runtime.levels()` describes the LOADED cdylib
/// metadata, so the detector's trigger classification must come from the same
/// place. A call that wrapped the SOURCE map in `LoadedNodeMetadata(..)`
/// type-checks and reinstates the drift bug in silence, so the newtypes cannot
/// pin this on their own.
#[test]
fn the_detector_is_fed_the_metadata_the_runtime_loaded_not_the_source() {
    let (code, depth) = code_only(&graph_cmd_src());
    assert_eq!(depth, 0, "unbalanced block comment");
    let supervisor = code
        .find(SUPERVISOR_FN)
        .unwrap_or_else(|| panic!("`{SUPERVISOR_FN}` not found"));
    let find = |needle: &str| -> usize {
        code[supervisor..]
            .find(needle)
            .map(|o| supervisor + o)
            .unwrap_or_else(|| {
                panic!(
                    "`{needle}` not found in `graph_run_supervisor`. Both split-partition \
                     diagnostics must be reached through their OWN seam — a hand-rolled call \
                     site is how the source-vs-loaded drift bug got in."
                )
            })
    };

    // `let X` + the next few hundred characters of its initialiser.
    let window = |binding: &str| -> String {
        let at = code[supervisor..]
            .find(&format!("let {binding}"))
            .map(|o| supervisor + o)
            .unwrap_or_else(|| panic!("`let {binding}` not found in the supervisor"));
        code[at..(at + 600).min(code.len())].to_string()
    };
    let assert_is_loaded = |binding: &str| {
        let init = window(binding);
        assert!(
            init.contains("entry.info()"),
            "`{binding}` is passed as LoadedNodeMetadata but is not harvested from the \
             planning factories' own `entry.info()` — that harvest IS the definition of \
             \"loaded\", and it is what `GraphRuntime::build_*` levelized with"
        );
        assert!(
            !init.contains("source_entry_infos("),
            "`{binding}` is source-parsed metadata wearing the LoadedNodeMetadata newtype \
             — the exact silent-swap this walk exists to catch"
        );
    };

    // (1) The DRIFT cross-check is the one seam taking BOTH provenances.
    let drift_args = call_args(&code, find(DRIFT_CLASSIFY_CALL));
    assert_eq!(
        drift_args.len(),
        3,
        "unexpected argument list for the drift seam: {drift_args:?}"
    );
    let source_binding = newtype_binding(&drift_args[1], "SourceNodeMetadata");
    let loaded_binding = newtype_binding(&drift_args[2], "LoadedNodeMetadata");
    assert_ne!(
        source_binding, loaded_binding,
        "the SAME map is passed as BOTH provenances (`{source_binding}`) — the drift \
         cross-check is then vacuous and can never fire"
    );
    assert_is_loaded(&loaded_binding);
    assert!(
        window(&source_binding).contains("source_entry_infos("),
        "`{source_binding}` is passed as SourceNodeMetadata but is not produced by \
         `source_entry_infos` — the cross-check would compare the loaded metadata against \
         itself and never fire"
    );

    // (2) The SPLIT-PAIR classifier takes ONLY the loaded metadata. There is
    // no source map in its signature to swap, so the walk checks the one
    // argument that can still be wrong.
    let pair_args = call_args(&code, find(CLASSIFY_CALL));
    assert_eq!(
        pair_args.len(),
        3,
        "unexpected argument list for the split-pair seam: {pair_args:?}"
    );
    assert_is_loaded(&newtype_binding(&pair_args[1], "LoadedNodeMetadata"));
}

/// The ordering arm. `validate_partition` judges on the
/// SOURCE metadata while the run it judges was levelized from the
/// LOADED cdylibs — so a stale build can make it REFUSE a partition the loaded
/// runtime would have run. That call is `?`-propagated, so ANY diagnostic
/// emitted after it is skipped on the refusing path: the operator got a bare
/// "invalid partition" with no hint that a stale build was the cause, on
/// precisely the shape where the source-vs-loaded disagreement matters most.
///
/// The stale-build reporter (and the check feeding it) must therefore sit
/// BEFORE the validator. The split-pair advisory stays AFTER it on purpose — it
/// describes a run that is about to happen, and a refused partition never runs.
///
/// No runtime test can observe this: reaching the refusal needs a real
/// workspace whose built cdylibs disagree with their own source.
#[test]
fn the_stale_build_warn_precedes_the_partition_validator_that_can_refuse() {
    let (code, depth) = code_only(&graph_cmd_src());
    assert_eq!(depth, 0, "unbalanced block comment");
    let supervisor = code
        .find(SUPERVISOR_FN)
        .unwrap_or_else(|| panic!("`{SUPERVISOR_FN}` not found"));
    let find = |needle: &str| -> usize {
        code[supervisor..]
            .find(needle)
            .map(|o| supervisor + o)
            .unwrap_or_else(|| panic!("`{needle}` not found in `graph_run_supervisor`"))
    };

    let drift_check = find(DRIFT_CLASSIFY_CALL);
    let drift_warn = find(DRIFT_REPORTER_CALL);
    let validate = find(VALIDATE_CALL);
    let pair_warn = find(REPORTER_CALL);

    assert!(
        drift_warn < validate,
        "the stale-build warn is emitted AFTER `validate_partition`, which is \
         `?`-propagated — so on a stale build that makes SOURCE-based validation refuse a \
         partition the LOADED runtime would run, the operator gets a bare partition \
         rejection and the stale-build cause is never named. Hoist the warn above the \
         validator (do NOT change which metadata the validator reads — that is a separate change)."
    );
    assert!(
        drift_check < validate,
        "the drift CHECK runs after `validate_partition`, so its warn cannot possibly \
         precede the refusal"
    );
    assert!(
        validate < pair_warn,
        "the split-pair ADVISORY moved above `validate_partition` — it describes a run that \
         is about to happen, and a refused partition never runs, so warning about its \
         pairing would be noise"
    );
}

/// ANTI-TAUTOLOGY: prove the stripped view still contains real code, so the
/// walk above cannot pass (or fail) on an empty/corrupted string.
#[test]
fn the_walk_sees_real_code_not_an_empty_view() {
    let (code, _) = code_only(&graph_cmd_src());
    for anchor in [
        SUPERVISOR_FN,
        SPAWN_LOOP,
        CLASSIFY_CALL,
        DRIFT_CLASSIFY_CALL,
        DRIFT_REPORTER_CALL,
        VALIDATE_CALL,
        "crate::multiprocess::plan_deployment(",
    ] {
        assert!(
            code.contains(anchor),
            "anchor `{anchor}` vanished from the stripped view — the stripper ate real code"
        );
    }
    assert!(
        code.len() > 100_000,
        "the stripped view collapsed to {} bytes; graph_cmd.rs is far larger",
        code.len()
    );
}

/// The stripper's own oracle. A `//` or `/*` in a COMMENT must not satisfy the
/// walk (the dodge this file exists to close), and no literal form may
/// truncate the view.
#[test]
fn code_only_strips_comments_and_literals_and_nothing_else() {
    let strip = |s: &str| code_only(s).0;
    assert_eq!(
        strip("let a = 1; // note\nlet b = 2;\n"),
        "let a = 1; \nlet b = 2;\n"
    );
    assert_eq!(strip("a /* gone */ b"), "a  b");
    assert_eq!(strip("a /* one /* two */ still gone */ b"), "a  b");
    assert_eq!(strip("a /* x // y\n z */ b"), "a  b");
    assert_eq!(strip("héllo — wörld // ✂\n"), "héllo — wörld \n");
    assert_eq!(strip("plain code"), "plain code");

    // THE class that broke the comments-only first cut: a comment opener
    // inside a literal must not open a block.
    assert_eq!(
        strip("let s = \"a /* unterminated b\"; let t = 1;"),
        "let s =  ; let t = 1;"
    );
    assert_eq!(
        strip("let s = \"esc \\\" /* still a literal\"; x"),
        "let s =  ; x"
    );
    assert_eq!(strip("let s = r#\"raw /* nope \"# ; y"), "let s =   ; y");
    assert_eq!(strip("let s = r\"raw /* nope\"; z"), "let s =  ; z");
    assert_eq!(strip("let s = b\"bytes /* nope\"; w"), "let s = b ; w");
    // A quote inside a CHAR literal must not swallow the following code ...
    assert_eq!(
        strip("let q = '\"'; let s = \"kept\"; tail"),
        "let q =  ; let s =  ; tail"
    );
    assert_eq!(strip("let e = '\\''; y"), "let e =  ; y");
    // ... while LIFETIMES survive untouched.
    assert_eq!(
        strip("fn f<'a>(x: &'a str) -> &'static str { x }"),
        "fn f<'a>(x: &'a str) -> &'static str { x }"
    );

    // A commented-out call must NOT survive, in EITHER syntax.
    assert!(!strip(&format!("// {REPORTER_CALL}\n")).contains(REPORTER_CALL));
    assert!(!strip(&format!("/* {REPORTER_CALL} */")).contains(REPORTER_CALL));
    // Unterminated blocks are REPORTED, never silently swallowed.
    assert_eq!(code_only("a /* forever").1, 1);
    assert_eq!(code_only("a /* closed */ b").1, 0);
}
