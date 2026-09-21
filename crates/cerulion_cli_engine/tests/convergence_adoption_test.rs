// SPDX-License-Identifier: AGPL-3.0-only
//! The STRUCTURAL guard that the desk observers really route their netd
//! catalog/schema queries through the first-contact convergence wait — and that each
//! seam passes the RIGHT wait posture.
//!
//! # Why a source walk and not a behavioural test
//!
//! The wait's behaviour is pinned end to end in
//! `cerulion_netd/tests/convergence_wait_e2e_test.rs`, against a scripted fake
//! daemon. What NO test in this repository can reach is the CLI's own call sites:
//! `netd_resolve_topic_schema` / `netd_fetch_remote_schema` connect to the real
//! per-computer `cerulion-netd` (spawning it if absent) and then talk to whatever
//! robots are on the operator's LAN. Exercising them needs a daemon and a robot, so
//! a CI test cannot.
//!
//! That is exactly the inert-shipping shape, and the stakes are twofold: the
//! feature could be reverted at the two lines that call it
//! (same types, compiles clean, every other test green), AND
//! a seam could pass the WRONG posture, which is not a revert but a regression: the
//! `topic echo` local-walker fallback inheriting the waiting posture stalls every
//! LOCAL topic for the full ceiling.
//!
//! # What is walked, and how
//!
//! A COMMENT-STRIPPED view of `src/topic_cmd.rs` (the module's docs name the
//! non-waiting verbs to explain why they are not called, so a naive search
//! would be satisfied by prose), and per-FUNCTION-BODY slices for the posture
//! assertions — because "the file mentions `no_wait` somewhere" says nothing about
//! WHICH seam uses it.
//!
//! It lives in `tests/` rather than in the module so its own needle literals are not
//! inside the text it searches.

use std::path::PathBuf;

/// The production source under guard.
fn topic_cmd_source() -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("topic_cmd.rs");
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

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

/// The comment-stripped code of `topic_cmd.rs`, with a LOUD failure if the scan did
/// not finish balanced (a truncated view makes every negative assertion vacuous).
fn code() -> String {
    let s = code_only(&topic_cmd_source());
    assert_eq!(
        s.unclosed_depth, 0,
        "the comment stripper ended inside {} unclosed block comment(s), so the tail \
         of topic_cmd.rs was DROPPED and every assertion below covers only a prefix. \
         This is what happens when a `/*` inside a string literal is not modelled.",
        s.unclosed_depth
    );
    s.code
}

/// The body of `fn <name>(` in `src`, brace-matched from its opening `{`.
///
/// Posture assertions must be FUNCTION-SCOPED: "the file contains `no_wait`" is true
/// the moment any seam uses it (a
/// whole-file pin cannot see a seam passing the wrong policy).
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

/// THE guard: both netd query seams call the WAITING verb, and neither of the
/// non-waiting siblings appears in code anywhere in the module.
///
/// The negative half is what makes this a revert detector rather than a presence
/// check: adding a converged call somewhere would satisfy a positive-only test even
/// if the real seams had been reverted.
#[test]
fn the_netd_query_seams_call_the_waiting_verbs_and_never_the_bare_ones() {
    let code = code();

    for waiting in ["query_catalog_converged", "query_schema_converged"] {
        assert!(
            code.contains(waiting),
            "topic_cmd must call `{waiting}` — the desk observers' whole convergence fix \
             is that call. If a seam was intentionally moved, move this guard with it."
        );
    }

    // The non-waiting siblings: correct for a caller with no absence claim to make,
    // and WRONG here, because both of these seams render an empty answer to a human.
    for bare in [
        "query_catalog_with_discovery",
        "query_schema_with_discovery",
    ] {
        assert!(
            !code.contains(bare),
            "topic_cmd calls `{bare}` in CODE. That verb answers on the FIRST round \
             trip, so the desk goes back to telling users to retry by hand on a cold \
             daemon. Use the `_converged` verb; discussing the bare one in \
             a doc comment is fine — this walk strips comments."
        );
    }
}

/// THE posture guard: each seam passes the wait policy
/// its own claim licenses.
///
/// A whole-file assertion cannot see this. `resolve_remote_walker_for_topic_with_opts`
/// is `topic echo`'s LOCAL-topic fallback — it renders no claim and degrades to hex —
/// and inheriting the default posture made every local topic pay the ceiling before
/// its first frame. `resolve_remote_ingress_target*` and `fetch_remote_schema` DO
/// render the UNKNOWN verdict, so they must wait.
#[test]
fn each_seam_passes_the_posture_its_own_claim_licenses() {
    let code = code();

    let walker = fn_body(&code, "fn resolve_remote_walker_for_topic_with_opts(");
    assert!(
        walker.contains("ResolveWait::no_wait"),
        "the local-walker fallback makes NO absence claim and must not wait — \
         inheriting the first-contact posture stalls every LOCAL `topic echo` for \
         the full ceiling before its first frame. Body:\n{walker}"
    );
    assert!(
        !walker.contains("first_contact"),
        "…and it must not ALSO carry the waiting posture. Body:\n{walker}"
    );

    // The two `resolve_remote_ingress_target*` seams need DIFFERENT assertions,
    // because one MINTS a posture and the other THREADS its caller's. A single
    // `contains("ResolveWait::first_contact") || contains("wait")` covered both and
    // was unfailable: `wait` is a substring of `ResolveWait`, of `no_wait`, of the
    // parameter name, and of the callee `..._with_opts_and_wait` — so the exact
    // regression this arm exists to catch (the minting seam flipped to `no_wait`)
    // satisfied it three times over.
    let minting = fn_body(&code, "fn resolve_remote_ingress_target_with_opts(");
    assert!(
        minting.contains("ResolveWait::first_contact"),
        "`resolve_remote_ingress_target_with_opts` MINTS a posture and renders an \
         absence claim, so it must mint the waiting one. Body:\n{minting}"
    );
    assert!(
        !minting.contains("ResolveWait::no_wait"),
        "…and must not mint the non-waiting one. Body:\n{minting}"
    );

    let threading = fn_body(&code, "fn resolve_remote_ingress_target(");
    assert!(
        threading.contains("_and_wait("),
        "`resolve_remote_ingress_target` THREADS its caller's posture through the \
         `_and_wait` entry — it must not resolve by any other route. Body:\n{threading}"
    );
    assert!(
        !threading.contains("ResolveWait::"),
        "…and must not OVERRIDE it with a posture of its own (in either direction). \
         Body:\n{threading}"
    );

    let ensure = fn_body(&code, "fn ensure_topic_available(");
    assert!(
        ensure.contains("ResolveWait::first_contact"),
        "`ensure_topic_available` is THE absence-claim seam (its empty answer becomes \
         the UNKNOWN verdict), so it waits. Body:\n{ensure}"
    );

    let fetch = fn_body(&code, "pub fn fetch_remote_schema(");
    assert!(
        fetch.contains("ResolveWait::first_contact"),
        "`schema info` renders an absence claim too. Body:\n{fetch}"
    );
}

/// BOTH netd seams must route a CANCELLED wait away from the
/// absence classifier.
///
/// If `WaitOutcome::Cancelled` is consumed only by the one-line epitaph and
/// then discarded, the cancelled (empty, NotConverged) answer flows on and a
/// Ctrl-C exits nonzero asserting the topic's existence is UNKNOWN. The guard is one
/// early return per seam — and like the posture, it is unreachable from CI (both
/// seams need a real netd and a real LAN), so a structural arm is what stands
/// between it and a silent revert. The RENDERING half is oracle-tested in
/// `topic_cmd.rs`; this is the WIRING half.
///
/// (`dead_code = "deny"` catches an edit that deletes ONE seam's arm, because the
/// enum variant becomes unconstructed — but not one that deletes BOTH, nor one that
/// keeps the check and drops the early return.)
#[test]
fn both_netd_seams_route_a_cancelled_wait_away_from_the_absence_classifier() {
    let code = code();
    for seam in [
        "fn netd_resolve_topic_schema(",
        "fn netd_fetch_remote_schema(",
    ] {
        let body = fn_body(&code, seam);
        assert!(
            body.contains("WaitOutcome::Cancelled"),
            "`{seam}` must check the terminal outcome — without it a cancelled \
             observation reaches the absence classifier and Ctrl-C renders a verdict \
             about a topic the user stopped looking for. Body:\n{body}"
        );
        assert!(
            body.contains("Cancelled)"),
            "…and must RETURN the Cancelled resolve arm, not merely inspect it. \
             Body:\n{body}"
        );
    }
}

/// The POLICY itself is pinned, not just the verb: a seam could quietly mint a
/// zero-ceiling `ConvergenceWait` and ship the feature inert with every other guard
/// green.
///
/// Only the two shared constructors may produce a policy here, so a hand-rolled
/// `ConvergenceWait::new(…)` anywhere in the CLI is a loud failure.
#[test]
fn only_the_shared_constructors_mint_a_wait_policy() {
    let code = code();
    assert!(
        !code.contains("ConvergenceWait::new"),
        "topic_cmd mints its own `ConvergenceWait` — a zero (or shrunken) ceiling at \
         one seam ships the convergence wait inert while every other guard stays green. Use \
         `ResolveWait::first_contact` / `ResolveWait::no_wait`."
    );
    assert!(
        code.contains("ConvergenceWait::off"),
        "the no-wait posture must come from the shared `off()` constructor"
    );
    assert!(
        code.contains("first_contact_wait()"),
        "the waiting posture must come from the shared `first_contact_wait()` helper, \
         so both waiting seams cannot drift to different ceilings"
    );

    // …and that helper must actually RETURN the shipped policy. None of the three
    // assertions above constrains its BODY: mutating it to `ConvergenceWait::off()`
    // keeps `ConvergenceWait::new` absent (because `off()` lives in cerulion_netd),
    // satisfies the `ConvergenceWait::off` presence check this very test makes, keeps
    // `first_contact_wait()` present — and ships the convergence wait fully inert with all seven
    // arms green.
    let helper = fn_body(&code, "fn first_contact_wait(");
    assert!(
        helper.contains("ConvergenceWait::default"),
        "`first_contact_wait` must return the SHIPPED policy. Body:\n{helper}"
    );
    for wrong in ["::off", "::new"] {
        assert!(
            !helper.contains(wrong),
            "`first_contact_wait` must not mint `{wrong}` — that ships the feature \
             inert while every other guard stays green. Body:\n{helper}"
        );
    }
}

/// The wait is WIRED at both seams, not merely called at one.
///
/// `emit_convergence_progress` and `note_convergence_wait_outcome` are the two
/// user-visible halves (the live line while waiting, the line that closes an
/// unconverged wait). Each is defined once and must be referenced at BOTH seams, so
/// three code occurrences apiece is the floor. Quietly passing a no-op
/// sink at one seam — leaving that command looking hung for ten seconds — trips this.
#[test]
fn both_seams_wire_the_progress_and_give_up_reporting() {
    let code = code();
    for (name, what) in [
        ("emit_convergence_progress", "the live progress line"),
        (
            "note_convergence_wait_outcome",
            "the line that closes an unconverged wait",
        ),
        (
            "note_convergence_wait_abandoned",
            "the line that closes a wait a transport error abandoned",
        ),
    ] {
        let occurrences = code.matches(name).count();
        assert!(
            occurrences >= 3,
            "`{name}` ({what}) appears {occurrences} time(s) in code; expected at \
             least 3 — one definition plus a reference at EACH of the two netd query \
             seams. A seam that drops it leaves that command silent while it waits."
        );
    }
}

/// The stdout/stderr split, which no in-process test can observe.
///
/// `topic echo`'s frames and `topic hz`'s rate lines are STDOUT, and a Studio-class
/// parser reading that stream must not have a progress spinner spliced into it — but
/// a unit test cannot see which of its OWN process streams a write landed on. What it
/// can check is that every convergence line goes through the single write seam and
/// that the seam is only ever handed stderr.
#[test]
fn every_convergence_line_is_written_to_stderr() {
    let code = code();
    let seam = fn_body(&code, "fn write_convergence_line(");
    assert!(
        !seam.contains("stdout"),
        "the convergence write seam must never touch stdout. Body:\n{seam}"
    );
    for emitter in [
        "fn emit_convergence_progress(",
        "fn note_convergence_wait_outcome(",
        "fn note_convergence_wait_abandoned(",
    ] {
        let body = fn_body(&code, emitter);
        assert!(
            body.contains("write_convergence_line"),
            "`{emitter}` must route through the ONE write seam, so the stream choice \
             is decided in a single place. Body:\n{body}"
        );
        assert!(
            body.contains("stderr") && !body.contains("stdout"),
            "`{emitter}` must hand the seam STDERR — a spinner on stdout corrupts \
             `topic echo`'s frame stream. Body:\n{body}"
        );
    }
    // Anti-tautology: `println!` would bypass the seam entirely, so pin that no
    // convergence emitter uses one.
    for emitter in [
        "fn emit_convergence_progress(",
        "fn note_convergence_wait_outcome(",
        "fn note_convergence_wait_abandoned(",
    ] {
        let body = fn_body(&code, emitter);
        assert!(
            !body.contains("println!"),
            "`{emitter}` must not print directly. Body:\n{body}"
        );
    }
}

/// ANTI-TAUTOLOGY for the walk itself.
///
/// Every assertion above is a substring test over `code_only`'s output, so a stripper
/// that returned an empty — or TRUNCATED — string would make each negative arm pass
/// vacuously. The truncation half is not hypothetical: a literal-blind stripper
/// drops 42 % of the file silently, and an "is it non-empty"
/// check could not see it. So this pins a marker from the very END of the file.
#[test]
fn the_stripped_view_reaches_the_end_of_the_file_and_drops_comments() {
    let src = topic_cmd_source();
    let stripped = code_only(&src);
    assert_eq!(stripped.unclosed_depth, 0, "the scan must end balanced");
    let code = stripped.code;

    assert!(
        code.contains("fn netd_resolve_topic_schema"),
        "the stripped view lost real code — every negative assertion above would be \
         vacuous"
    );
    assert!(
        code.contains("fn netd_fetch_remote_schema"),
        "the stripped view lost real code"
    );

    // THE TAIL MARKER. `topic_cmd.rs`'s own tests live at the bottom of the file and
    // hold comment openers inside string literals; a literal-blind stripper opens a
    // block comment there and drops everything after it. This is the assertion that
    // sees that, and a prefix-only view cannot satisfy it.
    let last_fn = src
        .rfind("    fn ")
        .expect("topic_cmd.rs must contain test functions near its end");
    let tail_marker: String = src[last_fn..]
        .lines()
        .next()
        .expect("a function signature line")
        .trim()
        .to_string();
    assert!(
        code.contains(&tail_marker),
        "the stripped view does NOT reach the last function in the file \
         (`{tail_marker}`) — the tail was silently dropped, so every negative \
         assertion above only covers a prefix"
    );

    // A phrase that exists ONLY in prose in that module. Present in the raw source,
    // absent from the stripped view.
    let doc_only = "the desk cannot tell those apart";
    assert!(
        src.contains(doc_only),
        "precondition: this phrase must still exist in topic_cmd's docs, else the \
         next assertion is vacuous"
    );
    assert!(
        !code.contains(doc_only),
        "the stripper is not removing comments — every negative assertion above \
         could then be satisfied by prose"
    );
}

/// `code_only` against hand-written inputs — the stripper is load-bearing for every
/// assertion in this file, so it is tested directly rather than trusted.
#[test]
fn code_only_strips_comments_and_string_literals_and_nothing_else() {
    let strip = |s: &str| code_only(s).code;
    assert_eq!(
        strip("let a = 1; // note\nlet b = 2;\n"),
        "let a = 1; \nlet b = 2;\n"
    );
    assert_eq!(strip("a /* gone */ b"), "a  b");
    assert_eq!(strip("a /* one /* two */ still gone */ b"), "a  b");
    // A `//` INSIDE a block comment must not end the block.
    assert_eq!(strip("a /* x // y\n z */ b"), "a  b");
    // Multi-byte characters survive intact (the module is full of them).
    assert_eq!(strip("héllo — wörld // ✂\n"), "héllo — wörld \n");
    // Nothing to strip is a no-op.
    assert_eq!(strip("plain code"), "plain code");

    // THE literal cases — the class that silently truncated the walk.
    assert_eq!(
        strip("let s = \"a /* unterminated b\"; let t = 1;"),
        "let s =  ; let t = 1;",
        "a comment opener inside a string literal must NOT open a block comment"
    );
    assert_eq!(
        strip("let s = \"esc \\\" /* still a literal\"; x"),
        "let s =  ; x",
        "an escaped quote does not end the literal"
    );
    assert_eq!(
        strip("let s = r#\"raw /* nope \"# ; y"),
        "let s =   ; y",
        "raw strings are skipped, hashes and all"
    );
    assert_eq!(strip("let s = r\"raw /* nope\"; z"), "let s =  ; z");
    assert_eq!(strip("let s = b\"bytes /* nope\"; w"), "let s = b ; w");

    // CHAR literals: a lone quote inside one must NOT open the string arm (which
    // would scan to the next `"` in the file and delete everything between, while
    // leaving `unclosed_depth == 0` so the loud guard could not see it).
    assert_eq!(
        strip("let q = '\"'; let s = \"kept\"; tail"),
        "let q =  ; let s =  ; tail",
        "a quote inside a char literal must not swallow the following code"
    );
    assert_eq!(strip("let c = 'x'; y"), "let c =  ; y");
    assert_eq!(strip("let e = '\\''; y"), "let e =  ; y");
    // …while a LIFETIME is untouched (no closing quote within the char window).
    assert_eq!(
        strip("fn f<'a>(x: &'a str) -> &'static str { x }"),
        "fn f<'a>(x: &'a str) -> &'static str { x }",
        "lifetimes must survive the char-literal arm"
    );

    // And the DEPTH report: an unterminated block outside a literal is REPORTED, not
    // silently swallowed.
    let unterminated = code_only("a /* forever");
    assert_eq!(unterminated.code, "a ");
    assert_eq!(
        unterminated.unclosed_depth, 1,
        "an unclosed block must be reported so a caller can fail loudly"
    );
    assert_eq!(code_only("a /* closed */ b").unclosed_depth, 0);
}
