// SPDX-License-Identifier: AGPL-3.0-only
//! The STRUCTURAL guard that the replay engine HARVESTS
//! the two scheduler-side counters it is the only reader of.
//!
//! # Why a source walk and not a behavioural test
//!
//! Both counters are maintained by `cerulion_core`'s scheduler and both are
//! already pinned BEHAVIOURALLY where they are produced —
//! `cerulion_core/tests/scheduler_test.rs` and
//! `read_outcome_capture_iox2_test::a_trace_driven_burst_whose_frame_is_missing_fires_and_counts_the_shortfall`
//! drive them against hand oracles. What no test could see is the ENGINE not
//! reading them, which is the state this guard exists to prevent:
//! `GraphRuntime::replay_plan_mismatches` and
//! `GraphRuntime::replay_refill_shortfalls` with ZERO callers outside
//! `cerulion_core`'s own tests, so that every value they carry is discarded and
//! the two conditions they name — a harness install/step mispairing, and a
//! trace-driven burst the harness under-fed — surface to an operator as a data
//! violation blaming the candidate.
//!
//! Reaching them behaviourally from the engine is not merely awkward, it is
//! unreachable from the fixtures that exist, and for two DIFFERENT reasons:
//!
//! * `replay_plan_mismatches` counts a decide seam running against a plan
//!   installed for another step. A correct engine installs one plan per step
//!   immediately before stepping, so on a correct engine the count is zero BY
//!   CONSTRUCTION and no bag can make it otherwise. The counter exists as a
//!   Principle-#3 observable of an internal invariant, and the engine's
//!   obligation is to READ it and refuse loudly.
//! * `replay_refill_shortfalls` needs a trace-driven burst on a node that HAS a
//!   trigger refill hook — i.e. a data-trigger consumer — inside a FREE-RUN
//!   (multi-rank) recording, because the fire plan is installed on the free-run
//!   path only. Every free-run fixture in `replay_engine_test.rs` is an
//!   all-source graph, so no such node exists to under-feed. No
//!   cross-rank data-trigger fixture exists, so the
//!   engine's obligation is structural and the counter's own semantics are
//!   pinned in `cerulion_core`.
//!
//! The walk is over a COMMENT- and STRING-LITERAL-stripped view, and both
//! halves matter here: the module explains at length WHY each counter is read
//! and what it means, so a naive `contains` would be satisfied by prose alone.

use std::path::{Path, PathBuf};

const ENGINE: &str = "cerulion_cli_engine/src/replay_engine.rs";

/// The function that must do the harvesting — located by BODY, so a read that
/// drifted out of the pass epilogue into (say) a helper nothing calls is
/// caught.
const PASS_FN: &str = "fn run_rank_pass(";

fn repo_root() -> PathBuf {
    // `CARGO_MANIFEST_DIR` is `<root>/cerulion_cli_engine`.
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the crate dir has a parent")
        .to_path_buf()
}

fn read(rel: &str) -> String {
    let p = repo_root().join(rel);
    std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("read {}: {e}", p.display()))
}

/// The outcome of stripping: the code-only view plus the unclosed `/*` depth at
/// EOF, so a truncated view fails LOUDLY instead of making every assertion over
/// it vacuous.
struct Stripped {
    code: String,
    unclosed_depth: usize,
}

/// A view of `src` with comments and string/char literals removed.
///
/// Same three hazards `graph_identity_adoption_test::code_only` models and for
/// the same reasons: Rust block comments NEST, a string literal can hold a
/// `/*` that would open a phantom comment, and a lone `"` inside a char literal
/// opens the string arm and silently swallows everything to the next quote —
/// leaving `depth == 0`, so the loud guard cannot see it.
fn code_only(src: &str) -> Stripped {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < bytes.len() {
        if depth == 0 {
            // A CHAR literal — only the short shapes, so a LIFETIME is never
            // mistaken for one.
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
            // A RAW string: `r`, `r#`, `r##`, … then `"`.
            if bytes[i] == b'r' {
                let mut h = i + 1;
                while bytes.get(h) == Some(&b'#') {
                    h += 1;
                }
                if bytes.get(h) == Some(&b'"') {
                    let hashes = h - (i + 1);
                    let mut j = h + 1;
                    let closer: String = std::iter::once('"')
                        .chain(std::iter::repeat_n('#', hashes))
                        .collect();
                    while j < bytes.len() {
                        if src[j..].starts_with(&closer) {
                            j += closer.len();
                            break;
                        }
                        j += 1;
                    }
                    out.push(' ');
                    i = j;
                    continue;
                }
            }
            // A regular or byte string.
            if bytes[i] == b'"' || (bytes[i] == b'b' && bytes.get(i + 1) == Some(&b'"')) {
                let mut j = if bytes[i] == b'b' { i + 2 } else { i + 1 };
                while j < bytes.len() {
                    match bytes[j] {
                        b'\\' => j += 2,
                        b'"' => {
                            j += 1;
                            break;
                        }
                        _ => j += 1,
                    }
                }
                out.push(' ');
                i = j;
                continue;
            }
            // A line comment.
            if src[i..].starts_with("//") {
                while i < bytes.len() && bytes[i] != b'\n' {
                    i += 1;
                }
                continue;
            }
        }
        if src[i..].starts_with("/*") {
            depth += 1;
            i += 2;
            continue;
        }
        if depth > 0 && src[i..].starts_with("*/") {
            depth -= 1;
            i += 2;
            out.push(' ');
            continue;
        }
        if depth == 0 {
            out.push(src[i..].chars().next().expect("char boundary"));
        }
        i += src[i..].chars().next().map_or(1, char::len_utf8);
    }
    Stripped {
        code: out,
        unclosed_depth: depth,
    }
}

/// A statement-shape needle, matched WITHOUT whitespace on either side.
///
/// `rustfmt` decides where a multi-token statement breaks (e.g.
/// `seam.unconsumed_pauses = seam` on its own line, `.unconsumed_pauses` and
/// `.saturating_add(...)` each on the next), and that decision moves with an
/// unrelated edit nearby — so a needle pinning the one-line spelling would
/// fail on a purely cosmetic reflow while a real un-wiring slipped past a
/// hurried re-blessing. Stripping ALL whitespace pins the STATEMENT, which is
/// what the feed-the-gate needles are about.
fn contains_stmt(body: &str, needle: &str) -> bool {
    let flatten = |s: &str| -> String { s.chars().filter(|c| !c.is_whitespace()).collect() };
    flatten(body).contains(&flatten(needle))
}

/// The brace-matched body of the function whose signature starts with `sig`.
fn fn_body(code: &str, sig: &str) -> Option<String> {
    let at = code.find(sig)?;
    let open = code[at..].find('{')? + at;
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

/// The production slice (everything above the first `#[cfg(test)]`), stripped.
fn engine_production_code() -> String {
    let stripped = code_only(&read(ENGINE));
    assert_eq!(
        stripped.unclosed_depth, 0,
        "{ENGINE}: the stripper ended UNBALANCED, so the view is a truncated prefix \
         and every assertion over it is vacuous"
    );
    let mut code = stripped.code;
    if let Some(at) = code.find("#[cfg(test)]") {
        code.truncate(at);
    }
    code
}

#[test]
fn the_pass_epilogue_harvests_the_plan_mismatch_counter_and_refuses_internally() {
    // A non-zero `replay_plan_mismatches` means decide seams ran against a plan
    // installed for a DIFFERENT step, so they fired nothing and the pass's fire
    // schedule is the harness's rather than the candidate's. It must be an
    // `Internal` refusal (exit 5) and NEVER a divergence: reporting it as exit
    // 6 would tell a CI job the code under test regressed.
    //
    // Deleting the read leaves the counter with no caller outside
    // `cerulion_core`'s own tests — and this arm
    // fails.
    let code = engine_production_code();
    let body = fn_body(&code, PASS_FN).expect("run_rank_pass must exist to be walked");
    assert!(
        body.contains("replay_plan_mismatches()"),
        "the pass epilogue must READ the plan-mismatch counter — it is the engine's own \
         install/step pairing, observable nowhere else"
    );
    // The refusal, and its CLASS. `ReplayError::Internal` is exit 5; the two
    // codes a reader must never see here are 1 (data violation) and 6 (trace
    // divergence), both of which name the candidate.
    let at = body
        .find("replay_plan_mismatches()")
        .expect("just asserted present");
    let after = &body[at..];
    assert!(
        after.contains("ReplayError::Internal"),
        "a non-zero count is a HARNESS bug, so it must refuse as `Internal` (exit 5) — \
         never as a divergence that blames the candidate"
    );
    assert!(
        !after[..after.find("ReplayError::Internal").unwrap()].contains("TraceDivergenceReport"),
        "and it must not be routed through the divergence report on the way there"
    );
}

#[test]
fn the_pass_epilogue_reads_every_intra_step_seam_counter_and_refuses_a_dirty_seam_internally() {
    // An unreached intra-step pause,
    // a stale pause install, a panicking injection hook, an unmatched or a
    // stranded slot each mean the HARNESS did not put the recording's foreign
    // frames where the recording put them, so no verdict over the candidate
    // is meaningful. The pass must READ all three scheduler-side counters,
    // fold the dispatch's two, and refuse as `Internal` (exit 5) — the
    // `replay_plan_mismatches` precedent — never as a divergence (exit 6) or
    // a data violation (exit 1), both of which name the candidate.
    //
    // Deleting any one read leaves that counter with no caller
    // outside `cerulion_core`'s own tests — a seam that would
    // only mint report-only notes — and this arm fails on the missing
    // token. Routing the refusal through `TraceDivergenceReport` fails the
    // last assertion.
    let code = engine_production_code();
    let body = fn_body(&code, PASS_FN).expect("run_rank_pass must exist to be walked");
    // A bare `body.contains(read)` only proves
    // each accessor appears SOMEWHERE in the function — a read whose result
    // is discarded (`let _ = runtime.replay_pause_mismatches();`) would still
    // pass, while a dirty count never reaches `seam` and `is_clean()` stays
    // vacuously true. Each needle below is the LITERAL statement that both
    // calls the accessor AND assigns/accumulates its result into `seam`,
    // proving the read actually FEEDS the gate rather than merely appearing
    // in the body.
    for feed in [
        // The per-step half: read then accumulated across the loop.
        "let unreached = runtime.unconsumed_replay_pauses();",
        "seam.unconsumed_pauses = seam.unconsumed_pauses.saturating_add(unreached.len() as u64);",
        // The epilogue pair, each a direct assignment.
        "seam.pause_mismatches = runtime.replay_pause_mismatches();",
        "seam.hook_panics = runtime.replay_hook_panics();",
        // The dispatch's own two, folded by MUTABLE REFERENCE into the SAME
        // `seam` the accessors above fed.
        "fold_slot_counters(&mut seam);",
    ] {
        assert!(
            contains_stmt(&body, feed),
            "the pass must feed `{feed}` into the seam gate — a counter read \
             but not assigned into `seam` is a seam nothing can audit \
             (Principle #3), and `is_clean()` cannot see it either"
        );
    }
    // The gate itself: the fold is followed by the NEGATED `is_clean` test
    // and an `Internal` refusal, with no divergence report between them.
    //
    // The bare `is_clean()` needle used
    // to admit `if seam.is_clean()` — the INVERTED polarity, which would
    // refuse every CLEAN pass and accept every DIRTY one — since it never
    // pinned the `!`. The literal below is the whole condition.
    let at = body
        .find("fold_slot_counters(")
        .expect("just asserted present");
    let after = &body[at..];
    let gate = after.find("if !seam.is_clean()").expect(
        "the folded seam is tested with the NEGATED condition — a dirty \
                 seam (not a clean one) is what refuses the pass",
    );
    let refusal = after[gate..]
        .find("ReplayError::Internal")
        .expect("a dirty seam refuses as `Internal` (exit 5)");
    assert!(
        !after[gate..gate + refusal].contains("TraceDivergenceReport"),
        "and it must not be routed through the divergence report on the way there"
    );
    // ORDER: the seam gate sits BEFORE the re-derivation verdict is
    // assembled, so a harness failure is never dressed as a schedule
    // divergence. The verdict is the first `verify(` call after the gate.
    let verdict = body
        .find("replay_rederive::verify(")
        .expect("the pass runs the re-derivation verifier");
    assert!(
        at < verdict,
        "the seam gate must precede the re-derivation verdict (gate at {at}, verdict at \
         {verdict})"
    );
}

#[test]
fn the_pass_epilogue_harvests_the_refill_shortfall_counter_per_node() {
    // A trace-driven burst that asked its refill hook for the next FIFO frame
    // and got nothing means the HARNESS under-fed the node: it fires anyway
    // (the plan is authoritative for the schedule), publishes from its held
    // head, and the frame diff sees an ordinary byte mismatch blaming the
    // candidate. The engine must read the counter so the real cause is
    // nameable.
    //
    // Deleting the loop leaves `ReplayInputShortfall` unconstructed —
    // which `dead_code = deny` refuses outright — so the only variant that still
    // compiles is a loop that never READS the counter, and that fails this arm.
    let code = engine_production_code();
    let body = fn_body(&code, PASS_FN).expect("run_rank_pass must exist to be walked");
    assert!(
        body.contains("replay_refill_shortfalls("),
        "the pass epilogue must READ each member's refill-shortfall counter"
    );
    assert!(
        body.contains("ReplayInputShortfall"),
        "and REPORT it — a `warn!` alone leaves the durable artifact silent about a \
         cause that otherwise reads as a candidate fault"
    );
    // PER NODE, not a bare total: the report entry names the node, so the read
    // has to be inside a walk over the plan's members.
    let at = body
        .find("replay_refill_shortfalls(")
        .expect("just asserted present");
    let before = &body[..at];
    // `RankPlan`'s fields are private, so the walk reads the
    // subgraph through its accessor.
    assert!(
        before
            .rfind("for node in &plan.subgraph().nodes")
            .is_some_and(|f| f > before.rfind("for credit in").unwrap_or(0)),
        "the read must sit inside a walk over this rank's own nodes, so the report can \
         name WHICH node was under-fed"
    );
}

#[test]
fn the_verifiers_trigger_set_comes_from_the_shared_intersection_rule() {
    // The trigger set the re-derivation verifier aligns on must be
    // trigger-marked **∩ WIRED**, from `cerulion_core`'s shared
    // `wired_trigger_input_names` — the same function the LIVE
    // `macro_policy_to_trigger` reads.
    //
    // Taking every trigger-MARKED port instead makes
    // `sync_aligned` wait for a stamp no producer feeds on a node whose
    // instance did not wire one, so it answers `false` on every step and the
    // verifier fabricates a FIRE-SCHEDULE divergence (exit 6) against a
    // candidate that did nothing wrong.
    //
    // STRUCTURAL, and the reason is worth stating: that shape is UNREACHABLE
    // through the replay engine, because `GraphRuntime::build*` runs
    // `validate_macro_sync_trigger_wiring` inside `build_with_scheduler` and
    // refuses such a graph BEFORE `prepare_pass_verification` is ever reached.
    // The hazard is therefore held by a build-time guard in ANOTHER crate
    // rather than by anything in the derivation — which is precisely why the
    // rule is one shared function, and why the caller's use of it is
    // pinned here instead of behaviourally. The rule's own oracle is
    // `cerulion_core::graph::runtime::tests::the_trigger_set_is_the_intersection_of_marked_and_wired`.
    //
    // Reverting `node_trigger_decl` to
    // `info.input_meta().iter().filter(|m| m.trigger)` fails this arm.
    let code = engine_production_code();
    let body = fn_body(&code, "fn node_trigger_decl(").expect("node_trigger_decl must exist");
    assert!(
        body.contains("wired_trigger_input_names("),
        "the verifier's trigger set must come from the SHARED intersection rule"
    );
    assert!(
        !body.contains("m.trigger"),
        "…and must not re-derive it from the declared metadata alone, which drops the \
         WIRED half of the intersection"
    );
}

#[test]
fn the_walk_reaches_live_code_and_the_stripper_is_balanced() {
    // ANTI-TAUTOLOGY. Without it, a renamed function, a `#[cfg(test)]` that
    // moved to line 1, or a stripper that swallowed the file would each make
    // both guards above pass while inspecting nothing.
    let code = engine_production_code();
    assert!(
        code.len() > 100_000,
        "the production slice is the bulk of a very large module; {} bytes means the \
         stripper or the `#[cfg(test)]` truncation ate it",
        code.len()
    );
    let body = fn_body(&code, PASS_FN).expect("run_rank_pass must exist");
    assert!(
        body.contains("runtime.step("),
        "the walked body must be the real pass loop"
    );
    assert!(
        body.contains("block_credits()"),
        "…and must still reach its epilogue"
    );
}

#[test]
fn the_stripper_removes_comments_and_literals_and_nothing_else() {
    // The predicate the three guards assert THROUGH, against hand vectors — a
    // stripper that answers too easily makes every guard vacuous without
    // failing anything.
    let s = code_only("let a = 1; // replay_plan_mismatches()\nlet b = 2;");
    assert_eq!(s.unclosed_depth, 0);
    assert!(!s.code.contains("replay_plan_mismatches"));
    assert!(s.code.contains("let b = 2;"));

    let s = code_only("/* outer /* inner */ still */ let c = 3;");
    assert_eq!(s.unclosed_depth, 0);
    assert!(s.code.contains("let c = 3;"));
    assert!(!s.code.contains("inner"));

    let s = code_only(r#"let m = "replay_refill_shortfalls("; let d = 4;"#);
    assert_eq!(s.unclosed_depth, 0);
    assert!(!s.code.contains("replay_refill_shortfalls"));
    assert!(s.code.contains("let d = 4;"));

    // A lifetime is NOT a char literal.
    let s = code_only("fn f<'a>(x: &'a str) -> &'a str { x }");
    assert_eq!(s.unclosed_depth, 0);
    assert!(s.code.contains("&'a str"));

    // An unterminated block fails CLOSED (reported, not silently swallowed).
    assert_ne!(code_only("/* never closed").unclosed_depth, 0);
}
