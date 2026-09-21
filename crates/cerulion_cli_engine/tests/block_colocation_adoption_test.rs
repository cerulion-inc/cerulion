// SPDX-License-Identifier: AGPL-3.0-only
//! The no-inert-shipping + PLACEMENT guard for the `block`
//! co-location constraint.
//!
//! Three of this feature's four load-bearing properties are invisible to every
//! runtime and unit test in the repo, because they are properties of WHERE a
//! call sits rather than of what it computes:
//!
//! 1. **the refusal lives INSIDE `validate_partition`.** That function is the
//!    one seam shared by `graph run`'s pre-spawn gate and the read-only `graph
//!    levels` verdict. Hoisting the check into the supervisor instead would
//!    leave every oracle in `partition.rs` green while `graph levels` — the
//!    verb an operator reaches for to ask "is my partition OK?" — silently
//!    stopped answering the question. `dead_code = deny` cannot see it: the
//!    checker is a private fn with unit tests of its own.
//!
//! 2. **the seed runs on BOTH derivation entry points, BEFORE the greedy
//!    loop.** `baseline_process_per_node` is the DEFAULT path (`cerulion graph
//!    run` with no cost snapshot), and it is a different function from
//!    `auto_partition` — a fix wired into only the fused path ships inert for
//!    every unprofiled graph, which is nearly all of them. And the greedy
//!    loop's budget fold sums `node_cost` over the CURRENT merged set, so a
//!    seed applied after it folds over the wrong members.
//!
//! 3. **the stale-build cross-check precedes the `?`-propagated
//!    validator**, for exactly the reason the split-pair warn's does: the partition is
//!    derived from SOURCE while the run is gated by the LOADED cdylib, and a
//!    source that says `drop_oldest` against a `.so` that says `block` yields a
//!    partition that splits a block edge. Reported after the refusal, that line
//!    would go missing on the shape where it matters most.
//!
//! # The stripper
//!
//! [`code_only`] is a PORT of the helper in `warn_adoption_test.rs`
//! (itself a port of `convergence_adoption_test.rs`'s), duplicated because
//! integration tests are separate binaries and this crate has no shared
//! test-support target. It carries its own oracle test below, so no copy can
//! rot silently into another.

use std::path::PathBuf;

/// The plan-time `block` co-location refusal, in its `?`-PROPAGATING form.
///
/// The `?` is part of the pin, not decoration: a call whose result is dropped
/// (`let _ = validate_block_colocation(..)`) leaves the text present and every
/// structural assertion green while the refusal stops refusing. This const is
/// what makes the walk see that variant.
///
/// A later review changed that shape: the walk now returns its credited
/// edges BESIDE the verdict (`BlockColocationOutcome`), so the production call
/// propagates `.verdict?` rather than the whole `Result`. The needle moved with
/// it. That it had to be moved by hand is the point — a source-walking pin is a
/// claim about text, so a call-shape change in a file this test never edits
/// silently turns it from a pin into a failure, which is exactly what happened.
const BLOCK_CHECK_CALL: &str = "validate_block_colocation(groups, &topo).verdict?;";
/// The public validator it must live inside.
const VALIDATE_PARTITION_FN: &str = "pub fn validate_partition(";
/// The seed + repair the derivation applies.
const SEED_CALL: &str = "seed_and_repair_block_colocation(";
/// The two derivation entry points.
const AUTO_PARTITION_FN: &str = "pub fn auto_partition(";
const BASELINE_FN: &str = "pub fn baseline_process_per_node(";
/// The greedy fusion loop's header — everything the seed must precede.
const GREEDY_LOOP: &str = "for &(coupling, pi, ci) in &candidates";
/// The union-find the seed mutates.
const UNION_FIND_NEW: &str = "UnionFind::new(";
/// The stale-build reporter + its check, in the supervisor.
const BP_DRIFT_BLOCK_CHECK_CALL: &str =
    "crate::multiprocess::warn_backpressure_classification_drift(";
const BP_DRIFT_CLASSIFY_CALL: &str = "crate::multiprocess::classify_backpressure_metadata_drift(";
/// The `?`-propagated partition validator in the supervisor.
const SUPERVISOR_VALIDATE_CALL: &str = "cerulion_core::graph::validate_partition(";
/// The supervisor function that owns the multi-process deployment.
const SUPERVISOR_FN: &str = "fn graph_run_supervisor(";

fn read_src(crate_rel: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("workspace root")
        .join(crate_rel);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("could not read {}: {e}", path.display()))
}

fn partition_src() -> String {
    read_src("cerulion_core/src/graph/partition.rs")
}

fn graph_cmd_src() -> String {
    read_src("cerulion_cli_engine/src/graph_cmd.rs")
}

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

/// Byte offset of `needle` in `code`, or a panic naming it.
fn find_in(code: &str, needle: &str) -> usize {
    code.find(needle)
        .unwrap_or_else(|| panic!("`{needle}` not found"))
}

/// (1) The refusal must live INSIDE `validate_partition`, not beside it.
#[test]
fn the_block_colocation_refusal_lives_inside_the_shared_partition_validator() {
    let (code, depth) = code_only(&partition_src());
    assert_eq!(depth, 0, "unbalanced block comment in partition.rs");
    let fn_start = find_in(&code, VALIDATE_PARTITION_FN);
    // The next `\npub fn ` after it bounds the body (the checker itself is a
    // private `fn`, so it cannot be mistaken for the boundary).
    let body_end = code[fn_start + VALIDATE_PARTITION_FN.len()..]
        .find("\npub fn ")
        .map(|o| fn_start + VALIDATE_PARTITION_FN.len() + o)
        .unwrap_or(code.len());
    let body = &code[fn_start..body_end];
    assert!(
        body.contains(BLOCK_CHECK_CALL),
        "`validate_partition` does not call `{BLOCK_CHECK_CALL}`. That function is the ONE \
         seam shared by `graph run`'s pre-spawn gate and the read-only `graph levels` \
         verdict — a refusal hoisted out of it stops answering the question `graph levels` \
         exists to answer, with every unit oracle still green."
    );
}

/// (2a) The seed must be applied on BOTH derivation entry points. The baseline
/// is the DEFAULT path, and it is a different function.
#[test]
fn the_seed_is_applied_on_both_derivation_entry_points() {
    let (code, depth) = code_only(&partition_src());
    assert_eq!(depth, 0, "unbalanced block comment in partition.rs");
    for (fn_header, why) in [
        (
            AUTO_PARTITION_FN,
            "the cost-fused path (a profiled graph) would split its block edges",
        ),
        (
            BASELINE_FN,
            "the BASELINE is the literal default — `cerulion graph run` with no cost \
             snapshot — so a fix wired only into the fused path ships inert for nearly \
             every graph",
        ),
    ] {
        let start = find_in(&code, fn_header);
        let end = code[start + fn_header.len()..]
            .find("\npub fn ")
            .map(|o| start + fn_header.len() + o)
            .unwrap_or(code.len());
        assert!(
            code[start..end].contains(SEED_CALL),
            "`{fn_header}` does not call `{SEED_CALL}` — {why}"
        );
    }
}

/// (2b) In `auto_partition` the seed must sit between `UnionFind::new` and the
/// greedy loop. Seeded after it, the loop's budget fold — which sums
/// `node_cost` over the CURRENT merged set — folds over the wrong members, and
/// the already-same-group arm that keeps a co-located pair from being recorded
/// as an `Unprofitable` rejection never sees the pair.
#[test]
fn the_seed_precedes_the_greedy_fusion_loop() {
    let (code, depth) = code_only(&partition_src());
    assert_eq!(depth, 0, "unbalanced block comment in partition.rs");
    let start = find_in(&code, AUTO_PARTITION_FN);
    let region = &code[start..];
    let uf = find_in(region, UNION_FIND_NEW);
    let seed = find_in(region, SEED_CALL);
    let greedy = find_in(region, GREEDY_LOOP);
    assert!(
        uf < seed,
        "the seed runs before the union-find it mutates exists"
    );
    assert!(
        seed < greedy,
        "the block co-location seed is applied AFTER the greedy fusion loop. The loop's \
         budget gate folds `node_cost` over the CURRENT merged set, so a later seed means \
         every budget verdict in that loop was computed against a member set the partition \
         does not have — and the loop's already-same-group arm, which exists so a \
         co-located pair asks for nothing, never sees the pair."
    );
}

/// (3) The stale-build cross-check must precede the `?`-propagated
/// `validate_partition`, for the same reason the split-pair warn's does.
#[test]
fn the_backpressure_stale_build_warn_precedes_the_partition_validator() {
    let (code, depth) = code_only(&graph_cmd_src());
    assert_eq!(depth, 0, "unbalanced block comment in graph_cmd.rs");
    let supervisor = find_in(&code, SUPERVISOR_FN);
    // BOUND the search to the supervisor's own body (the sibling arm above
    // does the same). `cerulion_core::graph::validate_partition(` occurs TWICE
    // after this function — here, and in `graph_levels` ~1500 lines later — so
    // an unbounded walk silently re-anchors on the WRONG one if the
    // supervisor's own call is deleted, and both ordering asserts then pass
    // vacuously while the closure's panic message still claims a scope the
    // search does not have.
    let body_end = code[supervisor + SUPERVISOR_FN.len()..]
        .find("\npub fn ")
        .map(|o| supervisor + SUPERVISOR_FN.len() + o)
        .unwrap_or(code.len());
    let body = &code[supervisor..body_end];
    let find = |needle: &str| -> usize {
        body.find(needle)
            .unwrap_or_else(|| panic!("`{needle}` not found in `graph_run_supervisor`"))
    };
    let classify = find(BP_DRIFT_CLASSIFY_CALL);
    let warn = find(BP_DRIFT_BLOCK_CHECK_CALL);
    let validate = find(SUPERVISOR_VALIDATE_CALL);
    assert!(
        warn < validate,
        "the block-colocation stale-build warn is emitted AFTER `validate_partition`, which is \
         `?`-propagated. The partition was derived from SOURCE while the run is gated by \
         the LOADED cdylib, so a source saying `drop_oldest` against a `.so` saying \
         `block` produces a partition that splits a block edge — and the operator would \
         get the bare refusal with no hint that a stale build caused it."
    );
    assert!(
        classify < validate,
        "the drift CHECK runs after the validator, so its warn cannot possibly precede the \
         refusal"
    );
}

/// ANTI-TAUTOLOGY: the stripped views still contain real code, so the walks
/// above cannot pass (or fail) on an empty or corrupted string.
#[test]
fn the_walks_see_real_code_not_an_empty_view() {
    let (partition, d1) = code_only(&partition_src());
    assert_eq!(d1, 0);
    assert!(partition.contains(VALIDATE_PARTITION_FN));
    assert!(partition.contains(AUTO_PARTITION_FN));
    assert!(partition.contains(BASELINE_FN));
    assert!(partition.contains(GREEDY_LOOP));
    let (graph_cmd, d2) = code_only(&graph_cmd_src());
    assert_eq!(d2, 0);
    assert!(graph_cmd.contains(SUPERVISOR_FN));
    assert!(graph_cmd.contains(SUPERVISOR_VALIDATE_CALL));
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
    assert!(!strip(&format!("// {BLOCK_CHECK_CALL}\n")).contains(BLOCK_CHECK_CALL));
    assert!(!strip(&format!("/* {BLOCK_CHECK_CALL} */")).contains(BLOCK_CHECK_CALL));
    // Unterminated blocks are REPORTED, never silently swallowed.
    assert_eq!(code_only("a /* forever").1, 1);
    assert_eq!(code_only("a /* closed */ b").1, 0);
}
