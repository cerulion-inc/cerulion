// SPDX-License-Identifier: AGPL-3.0-only
//! The structural guard that the three `graph_cmd` seams
//! really wire the checkpoint arm.
//!
//! Unreachable behaviourally, which is the whole reason this file exists. The
//! monolith seam sits inside `graph_run`'s live arm (it needs a real graph, a
//! real live loop and a Ctrl-C), the worker seam inside `graph_run_worker`
//! (a real supervisor-spawned process), and the supervisor seam inside the
//! multi-process spawn path. So the CLAMP can be fully correct — every
//! `cerulion_core` arm green — while no shipping run is ever armed: the classic
//! inert-shipping shape, and exactly what this file exists to catch.
//!
//! Each assertion is FUNCTION-SCOPED. A whole-file `contains` is true the moment
//! ANY seam wires the arm and is structurally blind to the one that stopped.

use std::path::PathBuf;

fn graph_cmd_source() -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("graph_cmd.rs");
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

/// A comment-stripped view plus the unclosed-`/*` depth at EOF.
struct Stripped {
    code: String,
    unclosed_depth: usize,
}

/// Strip `//`-to-end-of-line and `/* … */` (DEPTH-TRACKED — Rust block comments
/// nest), reporting the unclosed depth rather than silently dropping the tail.
///
/// Comments must go because this file's own seams are DOCUMENTED with comments
/// naming the very functions asserted below, so a raw-text `contains` would be
/// satisfied by prose describing a call that had been deleted.
///
/// STRING LITERALS are skipped — regular, byte and RAW — and that is not
/// optional here: MEASURED, `graph_cmd.rs` holds **four** `/*` sequences inside
/// literals, so a literal-blind scan opens four comments it never closes and
/// truncates the view. (That was found by RUNNING the literal-blind version:
/// both guards in [`code`] fired, which is the behaviour they exist for.) CHAR
/// literals holding a quote are consumed too, in the short shapes only, so a
/// lifetime (`&'static str`) is never mistaken for one. The unclosed depth is
/// still REPORTED, so any future shape this misses fails loudly rather than
/// enforcing the assertions over a prefix.
fn code_only(src: &str) -> Stripped {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < bytes.len() {
        if depth == 0 {
            // A CHAR literal — ONLY the short shapes, so a lifetime is never eaten.
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
            // A RAW string literal: `r`, zero or more `#`, then `"`.
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
        }
        if depth == 0 && bytes[i..].starts_with(b"//") {
            while i < bytes.len() && bytes[i] != b'\n' {
                i += 1;
            }
            continue;
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

/// The stripped source, with BOTH truncation guards armed.
fn code() -> String {
    let s = code_only(&graph_cmd_source());
    assert_eq!(
        s.unclosed_depth, 0,
        "the comment stripper ended inside {} unclosed block comment(s) — the tail of \
         graph_cmd.rs was DROPPED and the assertions below cover only a prefix",
        s.unclosed_depth
    );
    // Tail marker: the LAST thing the stripper must still be able to see. A
    // literal-borne truncation that happens to leave depth balanced is caught
    // here instead.
    assert!(
        s.code.contains("fn stamp_state_arm_tag"),
        "the stripped view lost the tail of graph_cmd.rs — every assertion below \
         would be enforced over a prefix"
    );
    s.code
}

/// Collapse every whitespace run to ONE space.
///
/// The assertions below match CALL SHAPES (`f(&mut x`), not bare names, because a
/// name alone is satisfied by a MENTION: replacing the
/// supervisor's `stamp_state_arm_tag(&mut plan, …)` with `let _ =
/// stamp_state_arm_tag;` passed a name-only version of this file. A call shape
/// spans an argument, and rustfmt is free to wrap it, so the view is normalised
/// first.
/// A call-shape needle, matched WITHOUT whitespace on either side.
///
/// `rustfmt` decides where a multi-argument call breaks, and that decision moves
/// when an argument is added — so a guard that pinned the one-line spelling
/// would fail on a purely cosmetic reflow while a real un-wiring slipped past a
/// hurried re-blessing. Stripping whitespace pins the CALL, which is what the
/// guard is about.
fn contains_call(body: &str, needle: &str) -> bool {
    normalize_call(body).contains(&normalize_call(needle))
}

/// Whitespace removed, and rustfmt's TRAILING COMMA before a closing paren
/// normalized away — it appears only when the call happens to be broken across
/// lines, which is a formatting outcome and not part of the call.
fn normalize_call(s: &str) -> String {
    let flat: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    flat.replace(",)", ")")
}

fn squash_ws(src: &str) -> String {
    let mut out = String::with_capacity(src.len());
    let mut in_ws = false;
    for ch in src.chars() {
        if ch.is_whitespace() {
            if !in_ws {
                out.push(' ');
                in_ws = true;
            }
        } else {
            out.push(ch);
            in_ws = false;
        }
    }
    out
}

/// The body of `fn <name>(` in `src`, brace-matched from its opening `{`.
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

/// The body of `fn <name>(` that CONTAINS `landmark`.
///
/// `graph_cmd.rs` declares `run_graph_recording` TWICE — a `#[cfg(not(unix))]`
/// stub and the real `#[cfg(unix)]` body — and the stub is declared FIRST, so
/// [`fn_body`]'s `find` would scope every assertion to the arm that does no
/// recording at all. Selecting by a landmark the real body must contain (rather
/// than by "the second one") keeps the choice meaningful if the order changes,
/// and asserting EXACTLY ONE match is what stops a landmark that has gone stale
/// from silently selecting nothing.
fn fn_body_with<'a>(src: &'a str, signature: &str, landmark: &str) -> &'a str {
    let mut from = 0usize;
    let mut hits: Vec<&'a str> = Vec::new();
    while let Some(rel) = src[from..].find(signature) {
        let at = from + rel;
        let body = fn_body(&src[at..], signature);
        if squash_ws(body).contains(landmark) {
            hits.push(body);
        }
        from = at + signature.len();
    }
    assert_eq!(
        hits.len(),
        1,
        "expected EXACTLY ONE `{signature}` body containing `{landmark}`, found {}",
        hits.len()
    );
    hits[0]
}

/// The byte offset of `needle` in `body`, or a panic naming what is missing.
fn index_of(body: &str, needle: &str, what: &str) -> usize {
    // Whitespace-insensitive, for `contains_call`'s reason: rustfmt owns where a
    // multi-argument call breaks, and an ORDERING guard must not fail on a
    // reflow. Both sides are stripped, so the indices are comparable.
    let stripped = normalize_call(body);
    let want = normalize_call(needle);
    stripped
        .find(&want)
        .unwrap_or_else(|| panic!("{what}: `{needle}` not found in the function body"))
}

/// The brace-matched block that OPENS at the end of `needle`.
///
/// This helper was earned rather than anticipated: a function-scoped
/// `contains` is NOT scoped enough when the function builds more than one struct
/// literal with the same field name. MEASURED — `graph_run_supervisor` passes
/// `state_tag: armed_tag.as_deref()` to BOTH the recording bring-up and the
/// always-on window spawn, so a window spawn missing its `state_tag` left the
/// other occurrence satisfying the assertion and the guard passed with the
/// handoff fully dead. Scoping to the CALL's own argument literal is what closes
/// it.
///
/// `needle` must end at the opening `{`, and `body` must be a [`squash_ws`] view
/// so the needle matches verbatim. Panics naming what was missing, so a stale
/// needle fails loudly instead of silently selecting nothing.
///
/// It does its OWN `find` rather than borrowing [`index_of`], and that is not a
/// duplication: `index_of` answers in the coordinates of a WHITESPACE-STRIPPED
/// copy (deliberately — an ordering guard must survive a rustfmt reflow), so
/// using its answer to slice the original string returns a window from somewhere
/// else entirely: reusing that offset here selected a
/// neighbouring call's argument list and the guard failed on the WRONG
/// assertion — a passing-for-the-wrong-reason risk in the other direction.
fn braced_after<'a>(body: &'a str, needle: &str) -> &'a str {
    assert!(
        needle.ends_with('{'),
        "`braced_after` needs a needle ending at the block's opening brace: {needle:?}"
    );
    let start = body
        .find(needle)
        .unwrap_or_else(|| panic!("`{needle}` not found in the (squashed) function body"))
        + needle.len();
    let bytes = body.as_bytes();
    let mut depth = 1usize;
    let mut i = start;
    while i < bytes.len() {
        match bytes[i] {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return &body[start..i];
                }
            }
            _ => {}
        }
        i += 1;
    }
    panic!("unbalanced braces after {needle:?}");
}

/// Assert `earlier` appears BEFORE `later` in `body`.
///
/// A bare `contains` is satisfied by a call that sits anywhere in the function —
/// including BELOW the blocking `run_live`, where the runtime executes its
/// entire life unarmed and the arm is installed only as the loop returns. The
/// clamp is a per-step decision, so "somewhere in this function" is not the
/// contract; "before the live loop starts" is.
fn assert_precedes(body: &str, earlier: &str, later: &str, why: &str) {
    let e = index_of(body, earlier, why);
    let l = index_of(body, later, why);
    assert!(
        e < l,
        "{why}: `{earlier}` must appear BEFORE `{later}` (found at {e} and {l})"
    );
}

/// The one live-loop call every armed seam must precede.
const LIVE_LOOP: &str = "runtime.run_live(&running)";

/// The MONOLITH seam: `graph_run` CREATES and arms its capture plane.
///
/// This is the `cerulion graph run --single-process` path and the pre-partitioning
/// shape: a run nobody partitioned still has to wire the checkpoint arm.
#[test]
fn the_monolith_graph_run_creates_and_arms_its_capture_plane() {
    let code = code();
    let body = squash_ws(fn_body(&code, "pub fn graph_run("));
    // This seam was widened once, and the always-on plane then inverted it: the seam now
    // CREATES the plane (nothing else does — a graph that waited for a recorder
    // to create one would capture nothing on the overwhelming majority of runs)
    // and then attaches its rank-0 state ring, through TWO named calls. Naming
    // both is what keeps the guard sound: a seam that armed the clamp and
    // created no ring would satisfy the first assertion while capturing nothing,
    // and one that attached without creating would be inert again.
    assert!(
        contains_call(
            &body,
            "arm_capture_plane( crate::state_arm_attach::PlaneRole::Monolith, tag, runtime.tightest_timing_ns(), )"
        ),
        "`graph_run` must CREATE and ARM this run's capture plane; the graph is the \
         owner by design, so a seam that only opens one ships Flashback inert on every \
         run nobody is recording"
    );
    // The two are SPLIT: the arm binds `capture_plane` so the always-on
    // recorder spawned below can be handed the tag of a plane this run REALLY
    // armed (see `the_always_on_recorders_are_handed_the_plane_and_the_run_context`),
    // and the attach then borrows the owner out of it. The pair is the same pair.
    assert!(
        contains_call(
            &body,
            "attach_state_capture_owned(&mut runtime, arm.clone(), tag, 0, &node_ids)"
        ),
        "`graph_run` must attach its rank-0 state ring to the plane it just armed, or \
         the anchors have nowhere to go"
    );
    assert_precedes(
        &body,
        "arm_capture_plane( crate::state_arm_attach::PlaneRole::Monolith, tag, runtime.tightest_timing_ns(), )",
        LIVE_LOOP,
        "`graph_run` monolith seam",
    );
    // And it is `graph_run` that RESOLVES the tag — once, from the
    // run descriptor it alone holds — then threads it. A seam that re-resolved
    // would miss the derivation and disagree with its siblings.
    assert!(
        body.contains("state_arm_tag_for_process("),
        "`graph_run` must resolve this run's arm tag (explicit env, else derived from \
         its run_id) — nothing else in the process holds the run descriptor"
    );
}

/// THE ALWAYS-ON CONTRACT: the capture plane is gated on the run's IDENTITY, not
/// on its DESCRIPTION having been written.
///
/// Run-descriptor creation is deliberately never-fatal: a read-only
/// home, a full disk, or a graph that will not re-render each yield ONE warn and
/// `None`, and the run carries on. An arm tag derived from
/// `_run_descriptor.run_id` would leave exactly those hosts with no tag, and
/// therefore no plane and no ring: Flashback would switch itself off as a side effect
/// of a bookkeeping file failing to appear, silently, on the machines where an
/// operator can least afford to lose the black box. That inverts decisions 75-77.
///
/// Minting is infallible, so the identity now precedes the write and survives its
/// failure. Both halves are asserted because either alone is a silent failure: a
/// mint that happened INSIDE the success arm would be back to square one, and an
/// arm still reading the descriptor would ignore the mint.
///
/// Structural rather than behavioural, for this file's stated reason: reproducing
/// it end-to-end means a real `graph run` against a poisoned `CERULION_HOME`. The
/// seam-level behavioural half — a failed descriptor, then a plane that really
/// appears in SHM — is `flashback_plane_e2e_test`'s
/// `a_run_whose_descriptor_cannot_be_written_still_arms_its_plane`.
#[test]
fn the_capture_plane_survives_a_run_descriptor_that_cannot_be_written() {
    let code = code();
    let body = squash_ws(fn_body(&code, "pub fn graph_run("));
    assert!(
        contains_call(&body, "state_arm_tag_for_process(Some(run_id))"),
        "`graph_run` must derive its arm tag from the run IDENTITY it minted, not from \
         the descriptor it may have failed to write — that write degrades on \
         purpose, so gating the plane on it turns every read-only-home robot into one \
         with no black box and no notice that it lost one"
    );
    assert!(
        !contains_call(&body, "state_arm_tag_for_process( _run_descriptor"),
        "…and it must not go back to reading the descriptor for that identity"
    );
    assert_precedes(
        &body,
        "let run_id = crate::run_dir::mint_run_id()",
        "begin_run_descriptor(",
        "`graph_run` must MINT the run identity before it tries to persist anything — a \
         mint inside the success arm is the same bug with more steps",
    );
}

/// The WORKER seam: `graph_run_worker` arms from its PLAN, never from its own
/// environment.
///
/// Both halves matter. Reading the plan is what makes every rank open the SAME
/// word (two ranks resolving their own tags could clamp from two different onset
/// steps); the env-read is forbidden here for exactly that reason, so its
/// ABSENCE is asserted alongside the plan read.
#[test]
fn the_worker_arms_from_its_stamped_plan_and_not_from_its_own_environment() {
    let code = code();
    let body = squash_ws(fn_body(&code, "pub fn graph_run_worker("));
    assert!(
        contains_call(
            &body,
            "attach_state_capture_from(&mut runtime, tag, \
             crate::state_arm_attach::ArmTagSource::Explicit, rank, &recording_node_ids)"
        ),
        "`graph_run_worker` must arm its runtime AND create THIS RANK's state ring, or \
         every multi-process rank ships with the catch-up clamp inert"
    );
    // The rank must be the worker's own cross-process rank, checked
    // through the same guard the trace ring uses — a rank stamped with the
    // departure sentinel would make an anchor's provenance
    // indistinguishable from a worker departure.
    assert!(
        body.contains("worker_ring_rank(plan.rank)"),
        "the worker's state-ring rank must come from the CHECKED rank conversion"
    );
    assert!(
        body.contains("plan.state_arm_tag"),
        "the worker's tag must come off its PLAN — every rank must open the SAME arm word"
    );
    assert!(
        !body.contains("attach_state_arm_from_env") && !body.contains("STATE_ARM_TAG_ENV"),
        "a worker must NOT resolve its own tag from the environment: two ranks doing so \
         could open two different words and clamp from two different onset steps"
    );
    assert_precedes(
        &body,
        "attach_state_capture_from(&mut runtime, tag, crate::state_arm_attach::ArmTagSource::Explicit, rank, &recording_node_ids)",
        LIVE_LOOP,
        "`graph_run_worker` seam",
    );

    // This worker must run the admission gate on its
    // own resident memory before it attaches.
    //
    // The supervisor arms the plane, but the supervisor is not the process that
    // forks — this one is. Gating only at the supervisor measured a few megabytes
    // of spawn bookkeeping and left the operator's ceiling unenforced on every
    // process it was set for (measured: a 1 MiB supervisor armed a
    // plane that a 129 MiB worker opened, under a 64 MiB ceiling).
    assert!(
        contains_call(
            &body,
            "admit_capture_plane( crate::state_arm_attach::PlaneRole::Worker { rank }, tag, runtime.tightest_timing_ns(), )"
        ),
        "every worker must evaluate the Flashback admission gate on ITS OWN memory, as \
         `PlaneRole::Worker`, or a rank above the ceiling records whenever its \
         supervisor is below it"
    );
    assert_precedes(
        &body,
        "admit_capture_plane(",
        "attach_state_capture_from(",
        "`graph_run_worker`: the gate must decide BEFORE the rank attaches — a ring \
         created and then regretted is a ring the recorder has already found",
    );
}

/// The RECORDING seam: `graph run --record` arms its runtime too.
///
/// `graph_run` RETURNS into `run_graph_recording` at its recording dispatch,
/// which is ABOVE the monolith arm's attach — so the seam pinned by the first
/// test is unreachable on this path and a recorded run executed unclamped. That
/// matters more here than anywhere: `graph run --record --single-process` is
/// one of the three wall-gated paths that must arm the clamp, and the stall the clamp
/// bounds is a RECORDER's stall.
///
/// Scoped to the `#[cfg(unix)]` body via [`fn_body_with`] — the non-unix stub
/// is declared first and runs no loop at all.
#[test]
fn the_recording_run_arms_its_runtime_before_its_live_loop() {
    let code = code();
    let body = squash_ws(fn_body_with(&code, "fn run_graph_recording(", LIVE_LOOP));
    assert!(
        contains_call(
            &body,
            "arm_capture_plane( crate::state_arm_attach::PlaneRole::Monolith, tag, runtime.tightest_timing_ns(), )"
        ) && contains_call(
            &body,
            "attach_state_capture_owned( &mut runtime, arm.clone(), tag, 0, &node_ids, )"
        ),
        "`run_graph_recording` must CREATE its capture plane and attach its rank-0 state \
         ring. This is THE headline capture-plane path: without it a `graph run --record` bag \
         carries no anchors and is not resimmable, and the run is unclamped on the one \
         path whose own recorder is the stall the catch-up clamp bounds"
    );
    assert_precedes(
        &body,
        "arm_capture_plane( crate::state_arm_attach::PlaneRole::Monolith, tag, runtime.tightest_timing_ns(), )",
        LIVE_LOOP,
        "`run_graph_recording` seam",
    );
}

/// The recorder is handed the tag of a plane this run HAS
/// ARMED, never one it merely resolved — and the ordering that makes that possible.
///
/// # The defect this pins shut
///
/// An arm sitting BELOW the bagd spawn (as if a
/// RECORDER created the word), with the tag handed over unconditionally, means a
/// run whose plane the kill switch or the RAM gate REFUSED still sends bagd sweeping
/// for `(tag, rank)` rings. A DERIVED tag is safe by construction — a `run_id` is
/// unique, nothing can be there — but a REUSED EXPLICIT `CERULION_STATE_ARM_TAG`
/// can find a ring leaked by a crashed earlier run, and `state_ring_run_id` derives
/// from the TAG, so the foreign-run filter cannot reject it either. That is a
/// PREVIOUS run's node state in THIS bag: replay not identical to live.
///
/// # Why a SOURCE pin rather than a behavioural one
///
/// Reproducing it needs a crashed prior run under a reused explicit tag AND a
/// refusal, in one process tree. The invariant that removes the whole class is an
/// ORDERING and a DATA FLOW, and both are visible here: the arm precedes the spawn,
/// and the spawn's `state_tag` reads the ARMED binding rather than the resolved one.
#[test]
fn the_recorder_is_handed_only_a_plane_this_run_actually_armed() {
    let code = code();
    let body = squash_ws(fn_body_with(&code, "fn run_graph_recording(", LIVE_LOOP));

    // (a) the ARM happens BEFORE the recorder is spawned. Without this the tag
    // cannot be conditioned on the arm at all — there is nothing to condition on.
    assert_precedes(
        &body,
        "arm_capture_plane( crate::state_arm_attach::PlaneRole::Monolith, tag, runtime.tightest_timing_ns(), )",
        "spawn_bagd_recorder(BagdSpawnSpec {",
        "`run_graph_recording`: the plane must be armed before the recorder is spawned",
    );

    // (b) the tag handed over is the ARMED one. This is the assertion that bites:
    // `state_arm` is still in scope and still spells a tag, so reverting just this
    // line compiles and restores the defect in full.
    assert!(
        body.contains("state_tag: capture_plane.as_ref().map(|(tag, _)| tag.as_str())"),
        "the recorder's `--state-tag` must come from the ARMED plane (`capture_plane`), \
         never from the merely-RESOLVED `state_arm`: a refused plane must hand over NO \
         tag, or bagd sweeps for rings this run did not create and can adopt a leaked \
         earlier run's under a reused explicit tag"
    );
    assert!(
        !body.contains("state_tag: state_arm."),
        "…and it must not read the resolved tag directly, which is exactly the reverted \
         form: {body:?}"
    );
}

/// The SUPERVISOR seam: the multi-process spawn path stamps the resolved tag
/// into the worker plans it is about to write.
///
/// Without it the worker seam above is wired to a field nothing ever fills, so
/// each half is necessary and neither is sufficient.
#[test]
fn the_supervisor_stamps_the_resolved_tag_into_every_worker_plan() {
    let code = code();
    let body = squash_ws(fn_body(&code, "fn graph_run_supervisor("));
    assert!(
        body.contains("stamp_state_arm_tag(&mut plan,"),
        "the supervisor must stamp the arm tag into its worker plans, or the worker's \
         `plan.state_arm_tag` is always None and the catch-up clamp is inert on every \
         multi-process run"
    );
    // The RESOLUTION moved up to `graph_run` (it is the only scope
    // holding this run's `run_id`, which the tag DERIVES from), and
    // the always-on switch made the supervisor the creator of the deployment's one plane.
    // Both halves are asserted, because either alone is a silent failure: a
    // supervisor that stamped without arming would send every worker to open a
    // word nobody created, and one that armed without stamping would hold a plane
    // no rank ever finds.
    assert!(
        contains_call(
            &body,
            "arm_capture_plane( crate::state_arm_attach::PlaneRole::Supervisor, tag, quantum, )"
        ),
        "the supervisor must CREATE and ARM the deployment's ONE capture plane — it is \
         the only process that can, since `create_owned` unlinks first and N workers \
         creating under one tag would map N different objects — and it must do so as \
         `PlaneRole::Supervisor`, because gating its OWN memory would let a fat \
         supervisor with workers that all fit disable capture for the whole deployment"
    );
    assert!(
        body.contains("stamp_state_arm_tag(&mut plan, armed.as_deref());"),
        "…and it must stamp the tag it ACTUALLY ARMED, so a worker never opens a plane \
         the kill switch or the RAM gate declined"
    );
    // ORDERING, which the two `contains` above cannot see: a plan is stamped in
    // MEMORY and then SERIALIZED to a file per group, and a worker reads only the
    // file. Moving the stamp below that serialization leaves every rank tagless —
    // `plan.state_arm_tag` is `None` in every file on disk — while the supervisor
    // holds a perfectly good armed plane nobody can find, and BOTH assertions
    // above still pass because both calls are still present in the body.
    //
    // `serde_json::to_string(w)` is the write loop's landmark, and it is a CODE
    // landmark on purpose: `code()` strips string literals, so the `plan_{}.json`
    // filename this loop builds is not in the searched text at all.
    assert_precedes(
        &body,
        "stamp_state_arm_tag(&mut plan, armed.as_deref());",
        "serde_json::to_string(w)",
        "the supervisor must stamp the arm tag BEFORE it serializes the worker plans — a \
         stamp after the write is invisible to every rank",
    );
    assert!(
        !body.contains("state_arm_tag_for_process("),
        "the supervisor must stamp the tag `graph_run` RESOLVED, not resolve a second \
         one — two resolutions is how one deployment ends up with two words"
    );
    assert!(
        !body.contains("STATE_ARM_TAG_ENV"),
        "the supervisor must NOT re-resolve from the environment: it would miss the \
         run-derived tag and disagree with the monolith seam"
    );
}

/// **The always-on spawns (decisions 94 + 95) really hand
/// the recorder the capture plane and the run context.**
///
/// This is the file's own class, one layer out. `flashback_argv` can be perfect —
/// every unit pin in `graph_cmd::tests` green — while both CALL SITES pass
/// `state_tag: None` and an empty handoff, and nothing anywhere would notice: the
/// recorder starts, the window fills, captures are written, and every one of them
/// silently carries no graph, no env, no host identity and no anchors. That is
/// precisely the defect this file exists to catch.
///
/// Behaviourally unreachable for this file's stated reason: the monolith seam is
/// inside `graph_run`'s live arm (a real graph, a real live loop, a real Ctrl-C)
/// and the supervisor seam is inside the multi-process spawn path.
///
/// Both seams are asserted, scoped to their own function bodies, because they are
/// INDEPENDENT call sites — the multi-process one is the Unix DEFAULT, so a guard
/// that only saw the monolith would pass while almost every real run handed over
/// nothing.
#[test]
fn the_always_on_recorders_are_handed_the_plane_and_the_run_context() {
    let code = code();

    // ---- the MONOLITH seam (`graph run --single-process`, and the pre-partitioning
    // shape). Scoped to the SPAWN's own argument literal, not merely to the
    // function — see `braced_after` for the case that motivates the distinction.
    let monolith = squash_ws(fn_body(&code, "pub fn graph_run("));
    assert!(
        monolith.contains("spawn_flashback_recorder(FlashbackSpawnSpec {"),
        "ANTI-TAUTOLOGY: the always-on spawn must still be here at all, or every \
         assertion below is about a seam that no longer exists"
    );
    let spec = braced_after(&monolith, "spawn_flashback_recorder(FlashbackSpawnSpec {");
    assert!(
        spec.contains("state_tag: _state_ring .as_ref() .and(capture_plane.as_ref())"),
        "the window recorder's `--state-tag` must come from the ARMED plane — without it \
         the recorder sweeps for no state ring, and the anchors every serving run now \
         takes go into a ring no process was ever told about: {spec:?}"
    );
    // …and it must be gated on the RING as well as the plane. This seam can do
    // that (the ring is created before the spawn) and `run_graph_recording`
    // cannot, so it is asserted here and nowhere else: a plane that armed while
    // its rank-0 ring creation FAILED must hand over NO tag, or the recorder
    // sweeps `(tag, rank)` and — under a reused explicit
    // CERULION_STATE_ARM_TAG — adopts a crashed earlier run's leaked ring, whose
    // TAG-derived `state_ring_run_id` the foreign-run filter cannot reject.
    assert!(
        spec.contains("_state_ring"),
        "the tag must be conditioned on this run having really CREATED its rank-0 ring, \
         not merely on the plane having armed: {spec:?}"
    );
    assert!(
        !spec.contains("state_tag: state_arm."),
        "…and never from the merely-RESOLVED tag: a refused plane must hand over NO tag, \
         or a leaked earlier run's ring under a reused explicit \
         CERULION_STATE_ARM_TAG is adopted into this run's captures: {spec:?}"
    );
    assert!(
        spec.contains("handoff: &handoff,"),
        "the run context (graph.yaml / env.json / recorder.json, from this run's \
         run directory) must reach the recorder, or every capture can be VIEWED and none \
         re-executed: {spec:?}"
    );
    assert!(
        monolith.contains("resolve_flashback_handoff("),
        "…and the handoff must be RESOLVED from the run directory rather than defaulted"
    );
    // …and the plane is armed BEFORE the spawn, which is what lets the tag be
    // conditioned on the arm at all (the `run_graph_recording` rule, applied to
    // the seam that had no such ordering because it handed over nothing).
    assert_precedes(
        &monolith,
        "arm_capture_plane( crate::state_arm_attach::PlaneRole::Monolith, tag, runtime.tightest_timing_ns(), )",
        "spawn_flashback_recorder(FlashbackSpawnSpec {",
        "`graph_run`: the plane must be armed before the window recorder is spawned",
    );

    // ---- the SUPERVISOR seam (the Unix DEFAULT).
    let supervisor = squash_ws(fn_body(&code, "fn graph_run_supervisor("));
    assert!(
        supervisor.contains("spawn_flashback_recorder(FlashbackSpawnSpec {"),
        "ANTI-TAUTOLOGY, for the multi-process seam"
    );
    // SCOPED to the spawn's own literal, and here it is load-bearing rather than
    // tidy: `graph_run_supervisor` ALSO passes `state_tag: armed_tag.as_deref()`
    // to the recording bring-up, so a function-scoped `contains` passed with the
    // window spawn's tag mutated to `None` (a variant that slipped past a
    // function-scoped check).
    let spec = braced_after(&supervisor, "spawn_flashback_recorder(FlashbackSpawnSpec {");
    assert!(
        spec.contains("state_tag: armed_tag.as_deref(),"),
        "the supervisor must hand the window recorder the tag IT armed — the same word it \
         stamped into every worker plan, so the sweep finds exactly the per-rank rings \
         this deployment publishes: {spec:?}"
    );
    assert!(
        spec.contains("handoff: &handoff,"),
        "the supervisor must hand over this run's run-directory artifacts too: {spec:?}"
    );
    assert!(
        supervisor.contains("resolve_flashback_handoff(run_dir)"),
        "…resolved from the `run_dir` `graph_run` threaded in, not defaulted"
    );
}

/// ANTI-TAUTOLOGY: the helpers really see the file, and `fn_body` really scopes.
///
/// Without this, a stripper that returned an empty string (or a `fn_body` that
/// returned the whole file) would make every assertion above either fail
/// confusingly or pass for the wrong reason.
#[test]
fn the_source_walk_helpers_see_the_code_they_claim_to() {
    let code = code();
    let worker = fn_body(&code, "pub fn graph_run_worker(");
    let monolith = fn_body(&code, "pub fn graph_run(");

    // Each body contains its own CODE landmark and NOT the other's — the proof
    // that `fn_body` scopes rather than returning the file. Landmarks are
    // identifiers, not message text: the stripper removes string literals, so a
    // message-based landmark would assert nothing.
    assert!(worker.contains("leave_barrier_cohort"));
    assert!(!monolith.contains("leave_barrier_cohort"));
    assert!(monolith.contains("time_source"));
    assert!(!worker.contains("time_source"));
    // A body is a strict slice of the file.
    assert!(worker.len() < code.len() && monolith.len() < code.len());
}
