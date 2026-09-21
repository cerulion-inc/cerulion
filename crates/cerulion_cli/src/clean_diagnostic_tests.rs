// SPDX-License-Identifier: AGPL-3.0-only
//! Pins that `cerulion clean` actually RUNS the state-file
//! diagnostic, and runs it in the right ORDER.
//!
//! # Why a source walk
//!
//! `report_shm_state_population` reads the one directory
//! `iceoryx2-pal-posix` hardcodes — literally `/tmp/` — and, without
//! `--report-only`, DELETES from it. A behavioural test of that function
//! would therefore have to operate on the developer's (or the runner's) real
//! `/tmp`, which a test must never touch and which would race every other
//! process on the machine. The engine half is fully oracle-tested over a tempdir with an
//! injected system probe (`cerulion_cli_engine::shm_state`); what CANNOT be
//! reached that way is the one thing the CLI owns:
//!
//! * that the diagnostic is CALLED at all — the classic inert-shipping shape,
//!   where every engine arm stays green while the verb does nothing, and
//! * that the dead-node SWEEP runs FIRST. Ordering is a correctness property
//!   here, not tidiness: the sweep removes state files of its own through
//!   iceoryx2's `shm_unlink`, so a population reported before it over-counts,
//!   and a reclamation running first would unlink objects the sweep was about
//!   to inspect.
//!
//! Reading source is a weaker oracle than running code, so the walk is
//! literal about what it recognises and carries an anti-tautology arm proving
//! the stripped view still contains the code it claims to read.

use std::path::{Path, PathBuf};

fn main_rs() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("src/main.rs")
}

fn read_main() -> String {
    let p = main_rs();
    std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("read {}: {e}", p.display()))
}

/// Strip `//`-to-end-of-line comments, so the names this module's own docs and
/// the function's doc comment mention are never read as calls.
fn code_only(src: &str) -> String {
    src.lines()
        .map(|line| match line.find("//") {
            Some(i) => &line[..i],
            None => line,
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// The brace-matched body of `fn <name>`, so an assertion about ONE function
/// cannot be satisfied by a call somewhere else in the file.
///
/// SCOPE: the count includes braces inside string literals, so a body
/// holding an UNBALANCED one would end the scope in the wrong place. Both
/// bodies this file reads hold only balanced ones (`println!("{line}")`), and
/// an unbalanced literal would show up as a panic here rather than as a
/// silently weaker assertion — the fail-loud direction.
fn fn_body(src: &str, name: &str) -> String {
    let sig = format!("fn {name}");
    let start = src
        .find(&sig)
        .unwrap_or_else(|| panic!("`{sig}` must exist in main.rs"));
    let open = src[start..]
        .find('{')
        .unwrap_or_else(|| panic!("`{sig}` must have a body"))
        + start;
    let bytes = src.as_bytes();
    let mut depth = 0usize;
    for (i, b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return src[open..=i].to_string();
                }
            }
            _ => {}
        }
    }
    panic!("`{sig}` body is not brace-balanced");
}

#[test]
fn the_clean_verb_runs_the_state_file_diagnostic() {
    // Without this the whole feature ships INERT: every engine arm stays
    // green while `cerulion clean` prints exactly what it printed before.
    let src = code_only(&read_main());
    let body = fn_body(&src, "clean_iceoryx2_state");
    assert!(
        body.contains("report_shm_state_population("),
        "`cerulion clean` must call the shm-state population diagnostic; body was:\n{body}"
    );
}

#[test]
fn the_dead_node_sweep_runs_before_the_state_file_diagnostic() {
    // The sweep removes state files of its own (iceoryx2's `shm_unlink`
    // removes the file its object was named by), so a population reported
    // before it over-counts, and a reclamation running first would unlink
    // objects the sweep was about to inspect.
    let src = code_only(&read_main());
    let body = fn_body(&src, "clean_iceoryx2_state");
    let sweep = body
        .find("clean_dead_nodes(")
        .expect("the sweep must still be called");
    let diagnostic = body
        .find("report_shm_state_population(")
        .expect("the diagnostic must still be called");
    assert!(
        sweep < diagnostic,
        "the dead-node sweep must precede the state-file diagnostic; body was:\n{body}"
    );
}

#[test]
fn the_reclamation_stands_down_when_dead_nodes_are_still_registered() {
    // A `.shm_state` file is the ONLY mapping from an iceoryx2
    // resource name to the object behind it, so removing one a still-registered
    // dead node needs makes that node permanently unreclaimable — no later
    // sweep can read its details again. Measured, against a control, in
    // `crates/cerulion_cli_engine/tests/reclaim_ordering_test.rs`.
    //
    // Structural rather than behavioural for the same reason the arms above
    // are: the reclamation reads and deletes from the developer's real `/tmp`.
    let src = code_only(&read_main());
    let body = fn_body(&src, "clean_iceoryx2_state");
    // The CONJUNCTION, not merely a mention of `converged`. Reclaiming requires
    // BOTH "the operator asked for it" and "the registry converged", so an
    // `||` here would reclaim on a wedged desk whenever `--report-only` was
    // absent — which is the whole defect. A bare `contains("converged")` is
    // satisfied by that variant too, so it is pinned as a whole expression.
    let normalized = body.split_whitespace().collect::<Vec<_>>().join(" ");
    assert!(
        normalized.contains("let reclaim = !report_only && converged;"),
        "`cerulion clean` must reclaim only when the operator asked AND the sweep left no \
         dead nodes registered — an `||` would reclaim on a wedged desk; body was:\n{body}"
    );
    // The DESTRUCTIVE half is what the gate must govern: `report_only` alone
    // reaching the diagnostic would leave the reclamation ungated.
    let call = body
        .find("report_shm_state_population(")
        .expect("the diagnostic must still be called");
    let arg_end = body[call..]
        .find(')')
        .expect("the diagnostic call must be closed");
    let arg = &body[call..call + arg_end];
    assert!(
        arg.contains("reclaim") || arg.contains("converged"),
        "the diagnostic's report-only argument must carry the convergence gate, not just \
         the flag; argument was:\n{arg}"
    );
}

/// The two brace-matched arms of `if <cond> { .. } else { .. }`, located by
/// the CONDITION text.
///
/// Both arms are needed because an assertion over the whole body cannot see a
/// SWAP: `ScanMode::ReportOnly` and `ScanMode::Reclaim` both appear either
/// way, and so does `report_only`.
fn if_else_arms(body: &str, condition: &str) -> (String, String) {
    let needle = format!("if {condition}");
    let start = body
        .find(&needle)
        .unwrap_or_else(|| panic!("`{needle}` must exist in the body:\n{body}"));
    let then = block_after(body, start);
    let after_then = start + body[start..].find(&then).expect("then-block offset") + then.len();
    let else_at = body[after_then..]
        .find("else")
        .unwrap_or_else(|| panic!("`{needle}` must have an else arm:\n{body}"))
        + after_then;
    let otherwise = block_after(body, else_at);
    (then, otherwise)
}

/// The first brace-matched `{ .. }` at or after `from`.
fn block_after(src: &str, from: usize) -> String {
    let open = src[from..]
        .find('{')
        .unwrap_or_else(|| panic!("expected a block after byte {from}"))
        + from;
    let bytes = src.as_bytes();
    let mut depth = 0usize;
    for (i, b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return src[open..=i].to_string();
                }
            }
            _ => {}
        }
    }
    panic!("unbalanced block after byte {from}");
}

#[test]
fn the_diagnostic_reclaims_only_when_the_operator_did_not_ask_for_a_report() {
    // `--report-only` is the look-first escape hatch, and the load-bearing
    // property is the MAPPING, not the presence of both names: a diagnostic
    // with its arms SWAPPED still mentions both modes and the flag, so it
    // would delete on the one run whose whole point was not to.
    let src = code_only(&read_main());
    let body = fn_body(&src, "report_shm_state_population");
    let (report_arm, reclaim_arm) = if_else_arms(&body, "report_only");

    assert!(
        report_arm.contains("ScanMode::ReportOnly") && !report_arm.contains("ScanMode::Reclaim"),
        "the `--report-only` arm must choose ReportOnly and nothing else; arm was:\n{report_arm}"
    );
    assert!(
        reclaim_arm.contains("ScanMode::Reclaim") && !reclaim_arm.contains("ScanMode::ReportOnly"),
        "the default arm must choose Reclaim and nothing else; arm was:\n{reclaim_arm}"
    );
    // The budgets ride the same fork, so a swap there is the same class of
    // defect: a report that spends the remedy's budget, or a remedy bounded
    // like a report.
    assert!(
        report_arm.contains("SHM_STATE_REPORT_BUDGET")
            && !report_arm.contains("SHM_STATE_RECLAIM_BUDGET"),
        "the report arm must take the report budget; arm was:\n{report_arm}"
    );
    assert!(
        reclaim_arm.contains("SHM_STATE_RECLAIM_BUDGET")
            && !reclaim_arm.contains("SHM_STATE_REPORT_BUDGET"),
        "the reclaim arm must take the reclaim budget; arm was:\n{reclaim_arm}"
    );
}

#[test]
fn the_walk_reads_the_code_it_claims_to_read() {
    // ANTI-TAUTOLOGY: every assertion above is a `contains` over a stripped
    // view, so a stripper that emptied the file would make all of them
    // vacuous — except this one.
    let raw = read_main();
    let src = code_only(&raw);
    assert!(
        src.contains("fn clean_iceoryx2_state"),
        "the stripped view must still hold the function under test"
    );
    assert!(
        src.contains("fn report_shm_state_population"),
        "and the diagnostic it calls"
    );
    // And the stripper must really strip: `main.rs` is full of `//` comments,
    // so a no-op stripper leaves the view identical to the source.
    assert!(
        src.len() < raw.len(),
        "the stripped view must be shorter than the source it came from"
    );
}

// ---------------------------------------------------------------------------
// The per-node refusal listing.
//
// Behavioural where it can be: `render_refused_nodes` is PURE, so the whole
// listing is pinned against hand-written line vectors with no dead node in
// the global iceoryx2 namespace. A subprocess test in
// `tests/trace_inspect_and_clean_cli_test.rs` could only exercise this arm by
// PLANTING a dead node in the shared `iox2_` namespace, which is exactly the
// contamination that file's convergence test detects — so the CLI file is
// deliberately untouched and the wiring is pinned by the source walk below.
// ---------------------------------------------------------------------------

use cerulion_cli_engine::ipc_cleanup::{classify_cleanup_failures, FailedNodeCleanup};
use cerulion_cli_engine::orphan_port_tags::{OrphanTagReclaim, ReclaimVerdict};
use cerulion_core::iceoryx_logger::CapturedLog;
use iceoryx2::prelude::LogLevel;

/// The node token exactly as iceoryx2 0.9.1 renders it — kept verbatim so a
/// reader's grep of the listing against a raw trace matches.
const NODE_A: &str =
    "UniqueNodeId(UniqueSystemId { value: 1, pid: 4242, creation_time: Time { seconds: 7, nanoseconds: 0 } })";

fn refusal(node: &str, variant: &str, causes: &[&str]) -> FailedNodeCleanup {
    FailedNodeCleanup {
        node: node.to_string(),
        variant: variant.to_string(),
        causes: causes.iter().map(|c| c.to_string()).collect(),
    }
}

#[test]
fn a_refused_node_renders_its_variant_and_every_sub_cause_indented_beneath_it() {
    let failures = [refusal(
        NODE_A,
        "InternalError",
        &[
            "Unable to remove stale resources since the service tags could not be read due to an internal error.",
            "Unable to remove stale resources since the stale resources of the port 9 could not be removed due to an internal failure.",
        ],
    )];
    let expected = vec![
        "Refused nodes:".to_string(),
        format!("  node {NODE_A}: InternalError"),
        "    Unable to remove stale resources since the service tags could not be read due to an internal error.".to_string(),
        "    Unable to remove stale resources since the stale resources of the port 9 could not be removed due to an internal failure.".to_string(),
    ];
    assert_eq!(super::render_refused_nodes(&failures), expected);
}

#[test]
fn a_converged_sweep_renders_no_listing_at_all() {
    // Not even the header: `cerulion clean` prints the listing
    // unconditionally after the breakdown, so an empty input must be silent.
    assert_eq!(super::render_refused_nodes(&[]), Vec::<String>::new());
}

#[test]
fn absences_are_stated_never_fabricated() {
    // A refusal line with no parenthesised variant, and a node iceoryx2 gave
    // no captured explanation for: each renders a STATEMENT of the absence,
    // and the no-explanation arm is the one place the trace hint survives.
    let failures = [refusal(NODE_A, "", &[])];
    let lines = super::render_refused_nodes(&failures);
    assert_eq!(lines[1], format!("  node {NODE_A}: (variant not reported)"));
    assert_eq!(
        lines.len(),
        3,
        "header + node + the absence line:\n{lines:?}"
    );
    assert!(
        lines[2].starts_with("    (no sub-cause captured"),
        "the absence must be stated:\n{lines:?}"
    );
    assert!(
        lines[2].contains("RUST_LOG=iceoryx2=trace"),
        "the hint must name the env var that actually gates the raw lines for this verb \
         (the sweep already pins iceoryx2 at Trace, so `IOX2_LOG_LEVEL` is inert):\n{lines:?}"
    );
    // A node WITH causes carries no hint — the listing IS the explanation.
    let explained = [refusal(NODE_A, "InternalError", &["x since the y."])];
    assert!(
        !super::render_refused_nodes(&explained)
            .iter()
            .any(|l| l.contains("RUST_LOG")),
        "an explained node must not carry the trace hint"
    );
}

#[test]
fn the_listing_is_capped_and_says_how_many_it_folded() {
    let cap = super::REFUSED_NODES_SHOWN;
    let node = |i: usize| {
        format!("UniqueNodeId(UniqueSystemId {{ value: {i}, pid: 1, creation_time: 0 }})")
    };
    let make = |n: usize| -> Vec<FailedNodeCleanup> {
        (0..n)
            .map(|i| refusal(&node(i), "InternalError", &["c since the d."]))
            .collect()
    };

    // cap + 2 ⇒ exactly `cap` nodes in full (the FIRST `cap`, in order), then
    // one fold line naming the remainder; nothing from the folded nodes.
    let lines = super::render_refused_nodes(&make(cap + 2));
    let node_lines: Vec<&String> = lines.iter().filter(|l| l.starts_with("  node ")).collect();
    assert_eq!(node_lines.len(), cap, "{lines:?}");
    for (i, l) in node_lines.iter().enumerate() {
        assert_eq!(**l, format!("  node {}: InternalError", node(i)));
    }
    assert_eq!(lines.last().unwrap(), "  … and 2 more", "{lines:?}");
    assert!(
        !lines.iter().any(|l| l.contains(&node(cap))),
        "a folded node must not leak past the cap:\n{lines:?}"
    );

    // Exactly `cap` ⇒ every node shown and NO fold line (`… and 0 more`
    // would be a lie about a remainder that does not exist).
    let lines = super::render_refused_nodes(&make(cap));
    assert_eq!(
        lines.iter().filter(|l| l.starts_with("  node ")).count(),
        cap
    );
    assert!(!lines.iter().any(|l| l.contains("more")), "{lines:?}");

    // cap + 1 ⇒ the boundary on the other side: exactly one folded.
    let lines = super::render_refused_nodes(&make(cap + 1));
    assert_eq!(lines.last().unwrap(), "  … and 1 more", "{lines:?}");
}

#[test]
fn the_cap_is_ten_nodes() {
    // The item's contract, pinned so a drift is a deliberate change.
    assert_eq!(super::REFUSED_NODES_SHOWN, 10);
}

#[test]
fn the_clean_verb_prints_the_listing_between_the_breakdown_and_the_unclassified_arm() {
    // The inert-shipping guard: every arm above stays green with the renderer
    // unwired. Ordering matters for the reader — the breakdown's
    // internal-error remediation points DOWN at the listing, so the
    // listing must follow it, and the unclassified hint closes the report.
    let src = code_only(&read_main());
    // The printing lives in `report_sweep` because the orphan-tag reclaim makes
    // `clean_dead_nodes` run TWO sweeps through one reporter.
    let body = fn_body(&src, "report_sweep");
    let breakdown = body
        .find("Failure breakdown:")
        .expect("the per-cause breakdown must still be printed");
    let listing = body
        .find("render_refused_nodes(")
        .expect("`cerulion clean` must render the refused-nodes per-node listing");
    let unclassified = body
        .find("Unclassified failures")
        .expect("the unclassified arm must still be printed");
    assert!(
        breakdown < listing && listing < unclassified,
        "breakdown, then listing, then unclassified; body was:\n{body}"
    );
    // The internal-error remediation must not send the operator to a
    // trace flag the listing makes unnecessary — such a flag is also
    // inert for this verb (the sweep pins iceoryx2 at Trace itself).
    let internal = body
        .find("\"iceoryx2 internal error\" =>")
        .expect("the internal-error arm must still exist");
    let arm_end = body[internal..].find("}").expect("arm closes") + internal;
    let arm = &body[internal..arm_end];
    assert!(
        !arm.contains("IOX2_LOG_LEVEL") && !arm.contains("RUST_LOG"),
        "the internal-error remediation must point at the listing, not a trace flag; arm was:\n{arm}"
    );
    assert!(
        arm.contains("refused nodes"),
        "the internal-error remediation must point the reader at the listing; arm was:\n{arm}"
    );
    // The unclassified arm keeps its hint — through the one shared constant.
    let tail = &body[unclassified..];
    assert!(
        tail.contains("RAW_IOX2_LINES_HINT"),
        "the unclassified arm must keep the raw-lines hint; tail was:\n{tail}"
    );
}

/// The paren-balanced argument list of the FIRST `<callee>(` in `body`,
/// whitespace-normalised — so an assertion about WHAT a renderer is handed
/// cannot be satisfied by the call merely existing.
fn call_args(body: &str, callee: &str) -> String {
    let needle = format!("{callee}(");
    let at = body
        .find(&needle)
        .unwrap_or_else(|| panic!("`{needle}` must be called; body was:\n{body}"));
    let open = at + callee.len();
    let bytes = body.as_bytes();
    let mut depth = 0usize;
    for (i, b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'(' => depth += 1,
            b')' => {
                depth -= 1;
                if depth == 0 {
                    let args = body[open + 1..i]
                        .split_whitespace()
                        .collect::<Vec<_>>()
                        .join(" ");
                    // rustfmt leaves a trailing comma on a wrapped argument list.
                    return args.strip_suffix(',').unwrap_or(&args).to_string();
                }
            }
            _ => {}
        }
    }
    panic!("`{needle}` is not paren-balanced; body was:\n{body}");
}

#[test]
fn each_renderer_is_handed_the_reports_own_field_never_an_empty_slice() {
    // The walks above pin that each renderer is CALLED. That is not the
    // wiring: `render_refused_nodes(&[])` keeps every call in place, keeps
    // every hand oracle on the PURE renderer green, and prints an empty
    // listing for every refused node — the inert-shipping shape one level
    // down. The ARGUMENT is pinned by VALUE: the sweep report's own field,
    // and the report is the reporter's one parameter (a shadowing
    // `let report = …` inside the body would make `&report.failures` a
    // local's, so the body may bind no `report` of its own).
    let src = code_only(&read_main());
    let printing = fn_body(&src, "report_sweep");
    assert_eq!(
        call_args(&printing, "render_refused_nodes"),
        "&report.failures",
        "the per-node listing must render the sweep's refusals; body was:\n{printing}"
    );
    assert_eq!(
        call_args(&printing, "render_registry_errors"),
        "&report.registry_errors",
        "the registry block must render the sweep's registry errors; body was:\n{printing}"
    );
    assert!(
        !printing.contains("let report"),
        "`report` must be the parameter, never re-bound; body was:\n{printing}"
    );
    let sig_start = src
        .find("fn report_sweep")
        .expect("the reporter must exist");
    let sig = &src[sig_start..sig_start + src[sig_start..].find('{').expect("body")];
    let sig = sig.split_whitespace().collect::<Vec<_>>().join(" ");
    assert!(
        sig.contains("(report: &cerulion_cli_engine::ipc_cleanup::CleanupReport)"),
        "the reporter takes the sweep report as `report`; signature was:\n{sig}"
    );

    // Same class, one function over: the reclaim renderer is handed the
    // reclaim outcomes and the operator's flag, never a literal.
    let verb = fn_body(&src, "clean_dead_nodes");
    assert_eq!(
        call_args(&verb, "render_orphan_tag_reclaims"),
        "&reclaims, report_only",
        "the reclaim listing must render the reclaim outcomes under the operator's flag; body \
         was:\n{verb}"
    );
}

// ---------------------------------------------------------------------------
// The orphan port-tag reclaim: a dead node whose directory holds nothing but
// the tags of ports it had already deregistered (a publisher destroyed while
// a loaned sample was leaked) fails every sweep at the final `rmdir`, so
// `cerulion clean` reclaims the tags and sweeps once more. The renderer is
// PURE and pinned against hand oracles; the wiring — WHICH sweep feeds the
// selector, that the reclaim sits BETWEEN the two sweeps, that `--report-only`
// travels as the dry-run bit, and that the convergence the `.shm_state` gate
// sees is the SECOND sweep's — is a source walk for the same reason the arms
// above are: the real thing deletes from the developer's `/tmp/iceoryx2`.
// The behavioural half over a REAL minted shape is
// `crates/cerulion_cli_engine/tests/clean_orphan_port_tag_test.rs`.
// ---------------------------------------------------------------------------

fn reclaim(node_id: u128, pid: u32, removed: &[u128], refused: Option<&str>) -> OrphanTagReclaim {
    OrphanTagReclaim {
        node: format!(
            "UniqueNodeId(UniqueSystemId {{ value: {node_id}, pid: {pid}, creation_time: 0 }})"
        ),
        node_id,
        pid,
        dir: std::path::PathBuf::from(format!("/tmp/iceoryx2/nodes/{node_id}")),
        removed: removed.to_vec(),
        verdict: match refused {
            Some(reason) => ReclaimVerdict::Refused(reason.to_string()),
            None => ReclaimVerdict::Reclaimed,
        },
    }
}

#[test]
fn an_already_empty_directory_renders_as_converged_pending_sweep_never_as_a_refusal() {
    // The truthful line is kept — the directory IS empty and the next sweep IS
    // what removes it — but it is not a "not reclaimed" line: the node is
    // converged pending sweep, and the second sweep the verb runs right after
    // this listing is the one that removes it.
    let empty = OrphanTagReclaim {
        verdict: ReclaimVerdict::AlreadyEmpty,
        ..reclaim(4242, 77, &[], None)
    };
    for report_only in [false, true] {
        let lines = super::render_orphan_tag_reclaims(std::slice::from_ref(&empty), report_only);
        assert_eq!(
            lines[1],
            "node 4242 (pid 77, process gone): directory already empty — nothing to reclaim; the \
             next sweep removes it",
            "report_only={report_only}"
        );
        assert!(
            !lines[1].contains("not reclaimed") && !lines[1].contains("removed 0"),
            "neither a refusal nor a phantom removal: {lines:?}"
        );
    }
}

#[test]
fn a_reclaimed_node_renders_its_pid_the_gone_verdict_and_every_removed_port_id() {
    let lines = super::render_orphan_tag_reclaims(&[reclaim(4242, 77, &[1, 22], None)], false);
    assert_eq!(lines.len(), 2, "{lines:?}");
    assert!(
        lines[0].starts_with("Reclaiming orphan port tags"),
        "{lines:?}"
    );
    assert_eq!(
        lines[1],
        "node 4242 (pid 77, process gone): directory not empty — removed 2 orphan port tag(s) [1, 22]"
    );
}

#[test]
fn report_only_says_would_remove_and_closes_with_the_nothing_removed_line() {
    let lines = super::render_orphan_tag_reclaims(&[reclaim(4242, 77, &[1], None)], true);
    assert_eq!(lines.len(), 3, "{lines:?}");
    assert!(
        lines[0].contains("report only") && lines[0].contains("nothing removed"),
        "{lines:?}"
    );
    assert_eq!(
        lines[1],
        "node 4242 (pid 77, process gone): directory not empty — would remove 1 orphan port tag(s) [1]"
    );
    assert!(
        !lines.iter().any(|l| l.contains(" removed 1 ")),
        "a report must never claim a removal: {lines:?}"
    );
    assert!(lines[2].contains("without `--report-only`"), "{lines:?}");
}

#[test]
fn a_refusal_renders_the_reason_and_never_claims_the_gone_verdict() {
    let lines = super::render_orphan_tag_reclaims(
        &[reclaim(4242, 77, &[], Some("process 77 is still alive"))],
        false,
    );
    assert_eq!(
        lines[1],
        "node 4242 (pid 77): not reclaimed — process 77 is still alive"
    );
    assert!(!lines[1].contains("process gone"), "{lines:?}");

    // A mid-way failure states what WAS removed rather than hiding it.
    let lines = super::render_orphan_tag_reclaims(
        &[reclaim(4242, 77, &[5], Some("removing `x` failed: boom"))],
        false,
    );
    assert_eq!(
        lines[1],
        "node 4242 (pid 77): not fully reclaimed — removing `x` failed: boom; 1 tag(s) removed \
         before the failure [5]"
    );
}

#[test]
fn no_candidates_render_nothing_at_all() {
    assert_eq!(
        super::render_orphan_tag_reclaims(&[], false),
        Vec::<String>::new()
    );
    assert_eq!(
        super::render_orphan_tag_reclaims(&[], true),
        Vec::<String>::new()
    );
}

#[test]
fn the_clean_verb_reclaims_orphan_tags_between_exactly_two_sweeps() {
    let src = code_only(&read_main());
    let body = fn_body(&src, "clean_dead_nodes");
    let sweep = "cleanup_dead_iceoryx2_nodes_with_diagnostics()";
    let sweeps: Vec<usize> = body.match_indices(sweep).map(|(i, _)| i).collect();
    assert_eq!(
        sweeps.len(),
        2,
        "the verb runs the first sweep, reclaims, and runs ONE more; body was:\n{body}"
    );
    let select = body
        .find("orphan_port_tag_candidates(")
        .expect("the verb must select candidates from the first sweep's refusals");
    let reclaim = body
        .find("reclaim_orphan_port_tags(")
        .expect("the verb must reclaim the candidates");
    let render = body
        .find("render_orphan_tag_reclaims(")
        .expect("the verb must print the reclaim outcome");
    assert!(
        sweeps[0] < select && select < reclaim && reclaim < render && render < sweeps[1],
        "sweep, select, reclaim, print, sweep again — in that order; body was:\n{body}"
    );
    // Both sweeps print through the ONE reporter, and EACH report is tied to
    // ITS sweep: the first sweep's binding precedes its report, the second's
    // binding precedes the second report, and there is no third. A count of
    // two `report_sweep(` calls alone is satisfied by printing the FIRST
    // report twice and never the second.
    let first_bind = body
        .find("let report = ipc_cleanup::cleanup_dead_iceoryx2_nodes_with_diagnostics();")
        .expect("the first sweep is bound to `report`");
    let first_report = body
        .find("report_sweep(&report)")
        .expect("the first sweep's report is printed");
    let second_bind = body
        .find("let second = ipc_cleanup::cleanup_dead_iceoryx2_nodes_with_diagnostics();")
        .expect("the second sweep is bound to `second`");
    let second_report = body
        .find("report_sweep(&second)")
        .expect("the second sweep's report is printed");
    assert!(
        first_bind < first_report
            && first_report < select
            && render < second_bind
            && second_bind < second_report,
        "bind the first sweep, report it, select, reclaim, print, bind the second sweep, report \
         it; body was:\n{body}"
    );
    assert_eq!(
        body.matches("report_sweep(").count(),
        2,
        "exactly the two reports — one per sweep; body was:\n{body}"
    );
    assert_eq!(
        body.matches("let second").count(),
        1,
        "`second` is bound once, by its sweep; body was:\n{body}"
    );
}

#[test]
fn report_only_travels_as_the_dry_run_bit_and_gates_the_second_sweep() {
    let src = code_only(&read_main());
    let body = fn_body(&src, "clean_dead_nodes");
    // POSITION and VALUE, not a substring: `!report_only` in the dry-run slot
    // contains `report_only` and reclaims on the one run whose whole point
    // was not to; `report_only` in another slot and a literal in the dry-run
    // slot would pass a substring check just the same. The reclaim's
    // signature is `(candidates, config, dry_run, verdict)`, so the operator's
    // flag must be the THIRD argument verbatim and `shm_state`'s one liveness
    // verdict the FOURTH.
    let args = call_args(&body, "reclaim_orphan_port_tags");
    let args: Vec<&str> = args.split(", ").collect();
    assert_eq!(
        args,
        vec!["&candidates", "config", "report_only", "&creator_verdict"],
        "the reclaim's arguments, in order — the dry-run bit is the operator's flag itself, \
         never its negation or a literal, and the verdict is `shm_state`'s; body was:\n{body}"
    );
    let call = body
        .find("reclaim_orphan_port_tags(")
        .expect("reclaim call");
    let args_end = body[call..].find(')').expect("call closes") + call;
    let second = body
        .rfind("cleanup_dead_iceoryx2_nodes_with_diagnostics()")
        .expect("second sweep");
    let between = &body[args_end..second];
    // The GUARD GOVERNS the second sweep, and nothing else does: the one
    // `if` between the reclaim and the second sweep forks on the operator's
    // flag, its block is exactly the non-converged early return, and from
    // its end to the second sweep there is no other `if` and no other
    // `return` — so the sweep runs on the guard's fall-through whenever
    // candidates existed, and ONLY `--report-only` skips it. A guard that
    // also asked "did a tag come off?" would leave an already-empty
    // directory standing for one more run.
    let guard = between
        .find("if report_only {")
        .expect("the dry-run guard sits between the reclaim and the second sweep");
    let then = block_after(between, guard);
    assert_eq!(
        then.split_whitespace().collect::<Vec<_>>().join(" "),
        "{ return (Ok(()), false); }",
        "the guard's block is the non-converged early return and nothing else; block was:\n{then}"
    );
    let after_guard = &between[guard + then.len()..];
    assert!(
        !after_guard.contains("if ") && !after_guard.contains("return"),
        "no other fork or return between the guard and the second sweep; was:\n{after_guard}"
    );
    assert_eq!(
        between.matches("if ").count(),
        1,
        "the operator's flag is the ONLY condition on the second sweep; between was:\n{between}"
    );
    assert!(
        !between.contains("removed_any"),
        "the second sweep must not be gated on a tag having come off; between was:\n{between}"
    );
}

#[test]
fn the_convergence_handed_to_the_state_file_gate_is_the_second_sweeps() {
    let src = code_only(&read_main());
    let body = fn_body(&src, "clean_dead_nodes");
    let second = body
        .rfind("cleanup_dead_iceoryx2_nodes_with_diagnostics()")
        .expect("second sweep");
    let tail = body[second..]
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ");
    // BOUND to the second sweep's result: the verb's LAST convergence value
    // is the full conjunction over `second` — the binding the second sweep
    // was assigned to — and it is computed after that sweep's report. A
    // `contains("second.failed_cleanups == 0")` alone is satisfied by a tail
    // that computes the conjunction and returns `converged` anyway.
    let last = convergence_positions(&body)
        .pop()
        .expect("the verb hands back a convergence");
    assert_eq!(
        last, "second.failed_cleanups == 0 && second.registry_errors.is_empty()",
        "the tail convergence is the second sweep's full conjunction; body was:\n{body}"
    );
    let reported = body
        .rfind("report_sweep(&second)")
        .expect("the second sweep is reported");
    let last_tuple = body.rfind("Ok(()),").expect("a convergence tuple");
    assert!(
        reported < last_tuple,
        "the second sweep is reported, then ITS convergence is handed back; tail was:\n{tail}"
    );
    // And the pre-reclaim early returns hand back the FIRST sweep's truth: a
    // converged first sweep is `true`, an unreclaimable refusal is `false`.
    let first_return = body
        .find("return (Ok(()), converged);")
        .expect("the first sweep's early return hands back ITS convergence");
    let select = body.find("orphan_port_tag_candidates(").expect("select");
    assert!(
        first_return < select,
        "a converged first sweep returns before selecting"
    );
    assert!(
        body[select..second].contains("return (Ok(()), false);"),
        "a refusal the reclaim cannot heal must hand back `false`; body was:\n{body}"
    );
}

/// Every convergence VALUE `clean_dead_nodes` hands back — the second half of
/// each `(Ok(()), <expr>)` tuple, whether behind a `return` or as the tail —
/// in source order, whitespace-normalised.
fn convergence_positions(body: &str) -> Vec<String> {
    body.match_indices("Ok(()),")
        .map(|(at, _)| {
            // The tuple's own `(` is the last non-whitespace byte before
            // `Ok(()),` — rustfmt puts a newline between them when the tail
            // does not fit on one line.
            let open = body[..at]
                .rfind(|c: char| !c.is_whitespace())
                .expect("a tuple opens before `Ok(()),`");
            assert_eq!(
                &body[open..=open],
                "(",
                "every `Ok(()),` sits inside a tuple; body was:\n{body}"
            );
            let bytes = body.as_bytes();
            let mut depth = 0usize;
            for (i, b) in bytes.iter().enumerate().skip(open) {
                match b {
                    b'(' => depth += 1,
                    b')' => {
                        depth -= 1;
                        if depth == 0 {
                            let expr = body[open + 1..i]
                                .trim_start()
                                .strip_prefix("Ok(()),")
                                .expect("the tuple opens with `Ok(()),`")
                                .trim_end();
                            // rustfmt leaves a trailing comma on the last
                            // element of a multi-line tuple.
                            let expr = expr.strip_suffix(',').unwrap_or(expr);
                            return expr.split_whitespace().collect::<Vec<_>>().join(" ");
                        }
                    }
                    _ => {}
                }
            }
            panic!("a `(Ok(()),` tuple is not paren-balanced; body was:\n{body}");
        })
        .collect()
}

#[test]
fn every_convergence_the_verb_hands_back_is_computed_never_a_literal_true() {
    // The arms above `find` ONE `return (Ok(()), converged);` and ONE
    // `return (Ok(()), false);` somewhere in the body. A verb with FOUR
    // convergence positions has three others a `find` never looks at, and
    // the `.shm_state` gate reclaims on whichever of them fires: a `true` on
    // the dry-run/nothing-removed arm reclaims on a wedged desk the moment
    // every candidate is refused; a bare `report.failed_cleanups == 0` on the
    // early return reclaims on a 0/0 sweep whose registry could not be
    // scanned. So EVERY position is enumerated and pinned, in order.
    let src = code_only(&read_main());
    let body = fn_body(&src, "clean_dead_nodes");
    assert_eq!(
        convergence_positions(&body),
        vec![
            // The first sweep's, computed above — FALSE on a 0/0 sweep with
            // registry errors, which is exactly the run the early return
            // takes.
            "converged".to_string(),
            // No candidate: a refusal the reclaim cannot heal.
            "false".to_string(),
            // A dry run, or nothing removed: the registry is as the first
            // sweep left it, and that sweep did not converge.
            "false".to_string(),
            // The second sweep's, computed from ITS report.
            "second.failed_cleanups == 0 && second.registry_errors.is_empty()".to_string(),
        ],
        "every convergence position, in order; body was:\n{body}"
    );
    // The value the early return hands back is the FULL first-sweep
    // conjunction, and the guard on that return is the failure count alone —
    // so a scan failure with nothing refused takes the early return and
    // hands back `converged == false`, never the guard's own truth.
    let normalized = body.split_whitespace().collect::<Vec<_>>().join(" ");
    assert!(
        normalized.contains(
            "let converged = report.failed_cleanups == 0 && report.registry_errors.is_empty(); \
             if report.failed_cleanups == 0 { return (Ok(()), converged); }"
        ),
        "the early return must hand back the computed conjunction under the failure-count \
         guard; body was:\n{body}"
    );
}

/// The cleaner-lock family end to end — capture → classifier → listing. Two
/// sweeps racing over one dead node (every `graph run` startup sweeps, and
/// `try_cleanup_dead_nodes` blocks for ZERO time) refuse with
/// `AnotherInstanceIsCleaningUpTheNode`, and iceoryx2's explanation reads
/// `since another instance …` / `since an interrupt …` (`mod.rs:598/758/762/766`),
/// not `since the …`. A predicate that only knew `since the` dropped those
/// lines, and the listing then sent the operator to `RUST_LOG=iceoryx2=trace`
/// for a line the tool had already captured. Each row must render its
/// verbatim explanation beneath the node and NO trace hint.
#[test]
fn a_cleaner_lock_refusal_renders_its_captured_explanation_not_the_trace_hint() {
    let rows: [(&str, &str); 4] = [
        (
            "Unable to remove stale resources since another instance is already cleaning up the dead nodes resources.",
            "AnotherInstanceIsCleaningUpTheNode",
        ),
        (
            "Unable to acquire monitor cleaner since another instance is already cleaning up all resources.",
            "AnotherInstanceIsCleaningUpTheNode",
        ),
        (
            "Unable to acquire monitor cleaner since another instance has already cleaned up all resources.",
            "ResourcesAlreadyCleanedUp",
        ),
        (
            "Unable to acquire monitor cleaner since an interrupt signal was received.",
            "Interrupt",
        ),
    ];
    let sweep_origin = "Node::<iceoryx2::service::ipc_threadsafe::Service>::cleanup_dead_nodes()";
    for (message, variant) in rows {
        let captured = vec![
            CapturedLog {
                level: LogLevel::Debug,
                origin: sweep_origin.to_string(),
                message: format!("Dead node ({NODE_A}) detected"),
            },
            CapturedLog {
                level: LogLevel::Debug,
                origin: format!("DeadNodeView(AliveNodeView {{ id: {NODE_A}, details: None, _service: PhantomData<iceoryx2::service::ipc_threadsafe::Service> }})"),
                message: message.to_string(),
            },
            CapturedLog {
                level: LogLevel::Trace,
                origin: sweep_origin.to_string(),
                message: format!("Unable to remove dead node {NODE_A} ({variant})."),
            },
        ];

        let lines = super::render_refused_nodes(&classify_cleanup_failures(&captured).failures);

        assert_eq!(
            lines,
            vec![
                "Refused nodes:".to_string(),
                format!("  node {NODE_A}: {variant}"),
                format!("    {message}"),
            ],
            "{variant}: the captured explanation must be the listing, with no trace hint"
        );
        assert!(
            !lines.iter().any(|l| l.contains("RUST_LOG")),
            "{variant}: the hint must not be printed for a line the tool already holds"
        );
    }
}

// ---------------------------------------------------------------------------
// The registry-wide block.
//
// iceoryx2 also logs lines about the registry WALK itself — the full-scan
// failure (`iceoryx2-0.9.1/src/node/mod.rs:1242`), `Node::list`'s own lines
// (`:1138`, `:1145`) and `list_all_nodes` (`:1261`, `:1265`) — which name no
// node. The classifier files them under `registry_errors`; the verb must print
// them under their own heading BEFORE any per-node listing, so a global
// failure reads as global rather than as the cause of whichever node is
// listed next.
// ---------------------------------------------------------------------------

const FULL_SCAN_LINE: &str =
    "Unable to perform a full scan for dead nodes since the all existing nodes could not be listed (InsufficientPermissions).";
const LIST_LINE: &str =
    "Unable to iterate over Node list since the node list could not be acquired (InsufficientPermissions).";
const LIST_ALL_NODES_LINE: &str =
    "Unable to list all nodes due to insufficient permissions while listing all nodes.";

#[test]
fn registry_wide_errors_render_under_their_own_heading_verbatim_and_in_order() {
    // The realistic 0.9.1 chain for a registry that cannot be listed, in
    // emission order (`list_all_nodes` → `Node::list` → the sweep entry).
    let errors = [
        LIST_ALL_NODES_LINE.to_string(),
        LIST_LINE.to_string(),
        FULL_SCAN_LINE.to_string(),
    ];
    let expected = vec![
        "registry-wide sweep errors:".to_string(),
        format!("  {LIST_ALL_NODES_LINE}"),
        format!("  {LIST_LINE}"),
        format!("  {FULL_SCAN_LINE}"),
        "  (the registry itself could not be fully scanned — not any node's fault; a dead node \
         the sweep never reached may still be registered and is counted nowhere above)"
            .to_string(),
    ];
    assert_eq!(super::render_registry_errors(&errors), expected);
}

#[test]
fn a_fully_scanned_registry_renders_no_registry_block_at_all() {
    // Not even the heading: the block is printed unconditionally at the top
    // of the verb's report, so an empty input must be silent.
    assert_eq!(super::render_registry_errors(&[]), Vec::<String>::new());
}

/// The scenario end to end — capture → classifier → BOTH renderers. The
/// full-scan line sits immediately before a refusal, token-free and
/// sub-cause-shaped; the per-node listing must NOT carry it beneath the
/// node, and the registry block must carry it verbatim. Hand oracle on the
/// exact lines of each.
#[test]
fn a_registry_wide_line_before_a_refusal_renders_in_the_registry_block_not_under_the_node() {
    let sweep_origin = "Node::<iceoryx2::service::ipc_threadsafe::Service>::cleanup_dead_nodes()";
    let own = "Unable to remove stale resources since the port tags could not be read due to an internal error.";
    let captured = vec![
        CapturedLog {
            level: LogLevel::Debug,
            origin: sweep_origin.to_string(),
            message: FULL_SCAN_LINE.to_string(),
        },
        CapturedLog {
            level: LogLevel::Debug,
            origin: sweep_origin.to_string(),
            message: format!("Dead node ({NODE_A}) detected"),
        },
        CapturedLog {
            level: LogLevel::Debug,
            origin: format!("DeadNodeView(AliveNodeView {{ id: {NODE_A}, details: None, _service: PhantomData<iceoryx2::service::ipc_threadsafe::Service> }})"),
            message: own.to_string(),
        },
        CapturedLog {
            level: LogLevel::Trace,
            origin: sweep_origin.to_string(),
            message: format!("Unable to remove dead node {NODE_A} (InternalError)."),
        },
    ];

    let parts = classify_cleanup_failures(&captured);

    assert_eq!(
        super::render_refused_nodes(&parts.failures),
        vec![
            "Refused nodes:".to_string(),
            format!("  node {NODE_A}: InternalError"),
            format!("    {own}"),
        ],
        "the node's listing carries exactly its own explanation — never the scan failure"
    );
    assert_eq!(
        super::render_registry_errors(&parts.registry_errors),
        vec![
            "registry-wide sweep errors:".to_string(),
            format!("  {FULL_SCAN_LINE}"),
            "  (the registry itself could not be fully scanned — not any node's fault; a dead node \
             the sweep never reached may still be registered and is counted nowhere above)"
                .to_string(),
        ],
        "the scan failure renders as global"
    );
}

/// A capture exercising every attribution arm at once: two interleaved nodes
/// (the id arm), an ownerless service-layer line (the adjacency arm), a
/// registry-wide line (arm 0), a foreign-token line (withheld), a
/// cleaner-lock explanation (`since another …`), and a variant the table
/// does not classify (`Interrupt`, into `unclassified`).
const NODE_B: &str =
    "UniqueNodeId(UniqueSystemId { value: 2, pid: 2020, creation_time: Time { seconds: 8, nanoseconds: 0 } })";
const NODE_C: &str =
    "UniqueNodeId(UniqueSystemId { value: 3, pid: 3030, creation_time: Time { seconds: 9, nanoseconds: 0 } })";
/// A node the walk VISITED and never refused (an inaccessible neighbour):
/// its line names it, so it is nobody's in the listing.
const NODE_D: &str =
    "UniqueNodeId(UniqueSystemId { value: 4, pid: 4040, creation_time: Time { seconds: 10, nanoseconds: 0 } })";
const OWNERLESS_LINE: &str =
    "Unable to unlink the port resources since the process does not have sufficient permissions.";
const A_OWN_LINE: &str =
    "Unable to remove stale resources since the service tags could not be read due to an internal error.";
const B_LOCK_LINE: &str =
    "Unable to acquire monitor cleaner since another instance is already cleaning up all resources.";

fn every_arm_capture() -> Vec<CapturedLog> {
    let sweep_origin = "Node::<iceoryx2::service::ipc_threadsafe::Service>::cleanup_dead_nodes()";
    let (a, b, c, d) = (NODE_A, NODE_B, NODE_C, NODE_D);
    let view = |node: &str| {
        format!("DeadNodeView(AliveNodeView {{ id: {node}, details: None, _service: PhantomData<iceoryx2::service::ipc_threadsafe::Service> }})")
    };
    let l = |origin: String, message: String| CapturedLog {
        level: LogLevel::Debug,
        origin,
        message,
    };
    vec![
        l(sweep_origin.to_string(), FULL_SCAN_LINE.to_string()),
        l(
            sweep_origin.to_string(),
            format!("Dead node ({a}) detected"),
        ),
        l(
            "Service::remove_node".to_string(),
            OWNERLESS_LINE.to_string(),
        ),
        l(
            sweep_origin.to_string(),
            format!("Dead node ({b}) detected"),
        ),
        l(view(a), A_OWN_LINE.to_string()),
        l(
            format!("Node::get_node_state(Config {{ .. }}, {d})"),
            "Unable to acquire node monitor due to insufficient permissions.".to_string(),
        ),
        l(view(b), B_LOCK_LINE.to_string()),
        l(
            sweep_origin.to_string(),
            format!("Unable to remove dead node {a} (InternalError)."),
        ),
        l(
            sweep_origin.to_string(),
            format!("Unable to remove dead node {b} (AnotherInstanceIsCleaningUpTheNode)."),
        ),
        l(
            sweep_origin.to_string(),
            format!("Unable to remove dead node {c} (Interrupt)."),
        ),
    ]
}

#[test]
fn classification_is_bit_identical_across_runs_and_matches_its_oracle() {
    // Principle #7 for a diagnostic: the same capture must classify the same
    // way every time, or two `cerulion clean` runs over one wedged desk would
    // name different culprits. The two-run compare is anchored to a hand
    // oracle first (the crib, `local_harvest`'s determinism arm, is a bare
    // self-compare — a classifier that deterministically produced the WRONG
    // parts would pass it).
    let captured = every_arm_capture();
    let run1 = classify_cleanup_failures(&captured);
    assert_eq!(
        run1.failures
            .iter()
            .map(|f| (f.variant.as_str(), f.causes.len()))
            .collect::<Vec<_>>(),
        vec![
            ("InternalError", 2),
            ("AnotherInstanceIsCleaningUpTheNode", 1),
            ("Interrupt", 0),
        ],
        "the oracle: A takes the ownerless line and its own, B its cleaner-lock explanation, \
         C nothing (the only line near it names the visited node D): {:?}",
        run1.failures
    );
    assert_eq!(run1.registry_errors, vec![FULL_SCAN_LINE.to_string()]);
    assert_eq!(run1.unclassified.len(), 1, "{:?}", run1.unclassified);
    assert_eq!(
        run1.failures_by_cause,
        std::collections::BTreeMap::from([
            ("iceoryx2 internal error".to_string(), 1),
            ("lock contention".to_string(), 1),
        ])
    );

    let run2 = classify_cleanup_failures(&captured);
    assert_eq!(
        run1, run2,
        "two runs over one capture must be bit-identical"
    );
}

#[test]
fn the_renderers_are_bit_identical_across_runs_and_match_their_oracles() {
    // The three PURE renderers, each run twice over the same input and
    // anchored to the hand oracles the arms above establish.
    let parts = classify_cleanup_failures(&every_arm_capture());
    let refused1 = super::render_refused_nodes(&parts.failures);
    let refused2 = super::render_refused_nodes(&parts.failures);
    assert_eq!(
        refused1,
        vec![
            "Refused nodes:".to_string(),
            format!("  node {NODE_A}: InternalError"),
            format!("    {OWNERLESS_LINE}"),
            format!("    {A_OWN_LINE}"),
            format!("  node {NODE_B}: AnotherInstanceIsCleaningUpTheNode"),
            format!("    {B_LOCK_LINE}"),
            format!("  node {NODE_C}: Interrupt"),
            // C's only nearby line names the visited node D, so C carries the
            // stated absence — never D's line.
            format!(
                "    (no sub-cause captured — {})",
                super::RAW_IOX2_LINES_HINT
            ),
        ],
        "the listing, line for line"
    );
    assert_eq!(refused1, refused2);

    let registry1 = super::render_registry_errors(&parts.registry_errors);
    let registry2 = super::render_registry_errors(&parts.registry_errors);
    assert_eq!(registry1[1], format!("  {FULL_SCAN_LINE}"));
    assert_eq!(registry1, registry2);

    let reclaims = [
        reclaim(4242, 77, &[1, 22], None),
        reclaim(4243, 78, &[], Some("process 78 is still alive")),
    ];
    for report_only in [false, true] {
        let lines1 = super::render_orphan_tag_reclaims(&reclaims, report_only);
        let lines2 = super::render_orphan_tag_reclaims(&reclaims, report_only);
        assert_eq!(lines1.len(), if report_only { 4 } else { 3 }, "{lines1:?}");
        assert_eq!(lines1, lines2, "report_only={report_only}");
    }
}

#[test]
fn the_clean_verb_prints_the_registry_block_first_and_gates_convergence_on_it() {
    // The inert-shipping guard for the block, plus the two ORDERING facts
    // that make it mean anything: it must come BEFORE the per-node listing
    // (a global failure read after a node listing reads as an afterthought)
    // and BEFORE the early "nothing to clean" return — the 0/0 counters of a
    // scan that listed nothing are otherwise indistinguishable from a clean
    // registry, and that return would skip the block entirely.
    let src = code_only(&read_main());
    // The printing lives in `report_sweep` — the one reporter BOTH sweeps of
    // `clean_dead_nodes` go through — so the ordering facts are read there;
    // the convergence conjunction is the verb's own and is read below.
    let printing = fn_body(&src, "report_sweep");
    let body = fn_body(&src, "clean_dead_nodes");
    let registry = printing
        .find("render_registry_errors(")
        .expect("`cerulion clean` must render the registry-wide block");
    let nothing_to_clean = printing
        .find("No dead iceoryx2 nodes found")
        .expect("the converged early return must still exist");
    let listing = printing
        .find("render_refused_nodes(")
        .expect("the per-node listing must still be rendered");
    assert!(
        registry < nothing_to_clean && registry < listing,
        "the registry block must precede both the early return and the listing; body was:\n{printing}"
    );
    // Both sweeps print through it: the block is per sweep, never only the first.
    assert_eq!(
        body.matches("report_sweep(").count(),
        2,
        "both sweeps must print through `report_sweep`; body was:\n{body}"
    );
    // CONVERGENCE: a registry the walk could not cover leaves dead nodes
    // registered that appear in no counter, so the `.shm_state` reclamation
    // must stand down for it exactly as for a refused node. Pinned as the
    // whole conjunction — `failed_cleanups == 0` alone does not satisfy the pin.
    let normalized = body.split_whitespace().collect::<Vec<_>>().join(" ");
    assert!(
        normalized.contains(
            "let converged = report.failed_cleanups == 0 && report.registry_errors.is_empty();"
        ),
        "convergence must require an empty registry-error set; body was:\n{body}"
    );
    assert!(
        normalized.contains("second.failed_cleanups == 0 && second.registry_errors.is_empty()"),
        "the SECOND sweep's convergence must require it too; body was:\n{body}"
    );
    // And the "nothing to clean" claim is reserved for a registry that WAS
    // scanned: the early-return arm forks on the registry errors, and the
    // MAPPING is what matters — swapped arms would claim "nothing to clean"
    // on exactly the run that scanned nothing.
    let (scanned, unscanned) = if_else_arms(&printing, "report.registry_errors.is_empty()");
    assert!(
        scanned.contains("No dead iceoryx2 nodes found") && !scanned.contains("not evidence"),
        "the fully-scanned arm alone may claim nothing to clean; arm was:\n{scanned}"
    );
    assert!(
        unscanned.contains("not evidence") && !unscanned.contains("No dead iceoryx2 nodes found"),
        "the unscanned arm must refuse the absence claim; arm was:\n{unscanned}"
    );
}
