// SPDX-License-Identifier: AGPL-3.0-only
//! Every workspace package that HAS tests must be named in a CI test step.
//!
//! WHY THIS FILE EXISTS. `.github/workflows/ci.yml` runs no blanket
//! `cargo test --workspace` — it deadlocks on iceoryx2's shared-memory
//! singleton — so every test job enumerates its packages with explicit `-p`
//! flags. That makes coverage a HAND LIST, and a hand list goes stale in
//! ways like these, each found one package at a time:
//!
//!   * `cerulion_bag` / `cerulion_bagd` compiled under
//!     `clippy --all-targets` and never RAN ("a pin that cannot fail a PR is
//!     not a gate");
//!   * an extraction moved 13 live tests out of
//!     `cerulion_cli_engine` into `cerulion_discovery`, silently un-running
//!     them;
//!   * `go2_tf`, the viz tree's third package, was executed by no
//!     job at all;
//!   * EIGHT packages and 729 listed tests at once
//!     (`cerulion_pairing`, `cerulion_remoted`, `cerulion_accountd`, `cerud`,
//!     `cerulion_dds`, `cerulion_link`, `cerulion-wire`, `cerulion_mdns`).
//!
//! Every one of those was invisible for the same reason: nothing enumerated
//! the thing being enumerated. So this is a WALK, not a list. It reads the
//! workspace members out of the root `Cargo.toml`, decides from the FILES
//! which of them carry tests, and reads the workflows as text to see whether
//! each one is named. Adding a crate fails this test until it is either
//! covered or classified.
//!
//! TWO-SIDED SET EQUALITY, deliberately. The declared exemption inventory is
//! checked in BOTH directions:
//!   * a package with tests that is neither covered nor exempt FAILS
//!     (the coverage hole);
//!   * an exemption whose package IS covered, or has no tests, or is not a
//!     member at all, ALSO fails (a stale waiver silently pre-authorises the
//!     next hole).
//!
//! PR-BLOCKING ONLY, and this is the difference between a gate and a
//! decoration. Scheduled and dispatch-only lanes re-list the same packages,
//! and two ci.yml jobs sit behind a `push || workflow_dispatch` allowlist
//! (`test-latency`, `cli-e2e-latency`). None of those can fail a pull request,
//! so a naive text match over every workflow would report a package "covered"
//! by a job that gates nothing. MEASURED: deleting the `cerulion_discovery`
//! step from BOTH ci.yml test jobs leaves an all-workflow match GREEN, because
//! a scheduled lane still names it.
//!
//! So a workflow counts only if its `on:` carries an UNFILTERED
//! `pull_request` trigger (a `paths:`- or `branches:`-filtered one runs on
//! some pull requests and not others — `examples-replay.yml` is the `paths:`
//! shape), and within it:
//!
//!   * a JOB counts only if it carries no job-level `if:` AND no job-level
//!     `continue-on-error: true`. The second is not a nicety — `fuzz` and
//!     `miri` both run on every pull request and neither can FAIL one, so a
//!     package covered only there is covered by nothing;
//!   * a STEP counts only if its `if:` cannot stop it running on a pull
//!     request. Absent, `always()` and `success()` obviously cannot; nor can a
//!     `matrix.<key> == <literal>` LEG SELECTOR whose key the job's own matrix
//!     DECLARES and whose literal is one of that key's legs, because every leg
//!     of a PR-blocking job's matrix runs on every pull request, so the step
//!     runs on exactly one of them. A selector naming a key or a leg the matrix
//!     does not carry matches no leg at all: it runs on NO pull request while
//!     reading as an ordinary leg-selected step, so it credits nothing.
//!     Everything else, `github.event_name == 'push'` most of all,
//!     disqualifies the step.
//!
//! Both job rules and the step rule are deliberately blunt in the FAIL-CLOSED
//! direction: a future condition that genuinely still runs on pull requests
//! costs a maintainer one line here rather than costing everyone a silent hole.
//!
//! THE `if:` VALUE IS READ IN EVERY YAML SCALAR FORM the workflow tree uses: a
//! plain scalar on the key's own line, a single- or double-quoted one, a folded
//! or literal block scalar (`>`, `>-`, `|`, `|-`, chomping included), and a
//! plain scalar written on the following more-indented lines. They all mean the
//! same expression. Reading one of them as an EMPTY condition is the whole
//! failure mode: an empty condition cannot stop a step, so a step behind
//! `github.event_name == 'push'` written on the line after its `if:` would
//! credit coverage. A form the reader cannot classify FAILS this test rather
//! than reading as no condition at all.
//!
//! SELECTION CONDITIONS are the one widening of that rule, and there are
//! exactly three, spelled as constants below:
//! `needs.changes.outputs.code == 'true'`,
//! `needs.changes.outputs.docs == 'true'`, and the per-package
//! `contains(fromJSON(needs.changes.outputs.pkgs), '<pkg>')`. A step behind one
//! of them runs on every pull request whose changed paths select it, so it can
//! fail one and it credits the packages it names. Everything else keeps the old
//! answer: `needs.changes.outputs.code == 'false'` and
//! `needs.other.outputs.code == 'true'` are ordinary disqualified conditions,
//! any `||` disqualifies, and a `&&` chain is sanctioned only when every term
//! is.
//!
//! GROUNDED, not merely well spelled. A condition is an expression over a value
//! another job produced, so each of the three counts only where that value
//! exists: the job carrying the step must `needs:` the `changes` job, `changes`
//! must declare the output the condition reads, and that declaration must be
//! EXACTLY `${{ steps.<id>.outputs.<name> }}` naming a step of that job that
//! can SET an output: a `run:` step that appends to `$GITHUB_OUTPUT`, or a
//! `uses:` step, whose action's outputs are not in this file to read. A LITERAL
//! grounds nothing: `code: 'false'` is a value no classifier produced, and a
//! step gated on `== 'true'` behind it runs on no event at all. Neither does an
//! expression carrying another operand: `${{ github.event_name == 'push' &&
//! steps.c.outputs.code }}` names a step that exists and is false on every pull
//! request. Any one of these missing means the condition reads the empty string
//! on every event. The step runs NOWHERE while reading as a selected step, so
//! the walk fails closed and drops it. All of them are asserted on synthetic
//! workflows by
//! `a_selection_condition_credits_nothing_when_its_output_is_not_grounded`.
//!
//! THE GUARD, OWED BY EVERY DEPENDANT OF THE CLASSIFIER. A job that `needs:`
//! the `changes` job is SKIPPED when that job fails, and branch protection
//! counts a skipped required context as SATISFIED, so the pull request merges
//! with that job's contexts never run. Every dependant therefore carries a
//! job-level `if:` that OPENS with `!cancelled()`: `${{ !cancelled() }}` on the
//! four test jobs, and `!cancelled() && ...` where the job keeps a gate of its
//! own. The population is the dependency, never the spelling of one step: a job
//! reading the selection only through `env:`, and a job whose gate is a folded
//! block scalar, owe the guard exactly as much as a job with a one-line `if:`.
//! Whether a guarded job is PR-blocking is a separate question, decided by
//! `job_is_gated` and pinned by
//! `the_not_cancelled_guard_is_the_only_job_condition_that_keeps_a_job_pr_blocking`.
//!
//! The producing rule reads the script TEXT, so `ci.yml`'s own `changes` job
//! grounds today only because its non-pull-request branch spells
//! `packaging=false` literally, its pull-request branch piping
//! `ci_changed_paths.sh` into the output file without ever naming a class.
//!
//! TOTALITY UNDER SELECTION is the second thing the walk now decides, because
//! being NAMED in a sanctioned step stops being enough once a selector exists:
//! the step also has to RUN on the pull request that touches that package
//! alone. So the walk evaluates the workflows against a hypothetical selected
//! set: a per-package condition is true iff its package is in the set, `code`
//! iff the set is non-empty, `docs` iff the set holds the `docs` marker, and
//! every other surviving condition is independent of the set and holds, and
//! requires every package with tests to stay credited when the set is exactly
//! itself. A workflow that carries no selection condition survives every set,
//! so the arm passes on it and stands as the guard for the first condition
//! added. Its own non-vacuity is pinned on synthetic input by
//! `a_step_gated_on_another_packages_selection_loses_its_own`.
//!
//! SCOPE. This asserts that a package is NAMED in a PR-blocking step,
//! not that its tests pass. Naming is the failure mode that has actually
//! bitten.
//!
//! It also does NOT see DOCTESTS. A package whose only tests are doctests
//! (`/// ```` fenced examples with no `#[test]` and no `tests/*.rs`) is
//! classified as having no tests and is never demanded of. Nothing in the
//! workspace is that shape today; detecting it would mean parsing doc
//! comments, and a wrong answer there is a false demand rather than a missed
//! hole. This is a stated limitation.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

/// Packages that have tests and are deliberately NOT named in a CI test step.
///
/// Each entry is `(package name, reason)`. Empty is the healthy state and is
/// not a bug: it means every package that has tests is run. An entry here is
/// a promise that somebody decided, not that somebody forgot.
const EXEMPT_PACKAGES: &[(&str, &str)] = &[];

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cerulion_cli_engine has a parent")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf()
}

// ---------------------------------------------------------------------------
// Workflow text, with YAML comments removed.
// ---------------------------------------------------------------------------

/// Strip YAML comments from one line: a `#` at the start of the line (modulo
/// leading whitespace) or one preceded by whitespace opens a comment.
///
/// It is deliberately the CONSERVATIVE direction: a
/// line it over-strips simply stops matching, which fails CLOSED (a red a
/// maintainer resolves), never open.
///
/// It matters a great deal here. `ci.yml` is more comment than YAML — its
/// rationale blocks quote whole `cargo test -p <package>` invocations while
/// explaining why a package was ADDED or why one is NOT run. Without the
/// strip, a package could be "covered" by the paragraph mourning its absence.
fn strip_yaml_comment(line: &str) -> &str {
    let bytes = line.as_bytes();
    let mut i = 0;
    // Leading whitespace, then a `#`, is a whole-line comment.
    while i < bytes.len() && (bytes[i] == b' ' || bytes[i] == b'\t') {
        i += 1;
    }
    if i < bytes.len() && bytes[i] == b'#' {
        return "";
    }
    // Otherwise a `#` preceded by whitespace opens a trailing comment.
    let mut prev_ws = false;
    for (idx, ch) in line.char_indices() {
        if ch == '#' && prev_ws {
            // Trim the whitespace that separated code from comment, so the
            // stripped line is the code exactly (the same `s/[[:space:]]+$//`
            // the shell twin applies).
            return line[..idx].trim_end();
        }
        prev_ws = ch == ' ' || ch == '\t';
    }
    line
}

/// Does this (comment-stripped) workflow carry an UNFILTERED `pull_request`
/// trigger — i.e. does it run on EVERY pull request?
///
/// A `paths:`- or `branches:`-filtered trigger is deliberately NOT enough: it
/// runs on some pull requests and not others, so a package covered only there
/// is uncovered for most changes. `branches:` is the sharper of the two —
/// `pull_request: branches: [release/**]` runs on essentially nothing a
/// contributor opens, while reading as a perfectly ordinary trigger.
fn runs_on_every_pull_request(text: &str) -> bool {
    let mut in_on = false;
    let mut in_pr = false;
    for line in text.lines() {
        if line.starts_with("on:") {
            in_on = true;
            continue;
        }
        if !in_on {
            continue;
        }
        // A new top-level key ends the `on:` mapping.
        if !line.starts_with(' ') && !line.trim().is_empty() {
            break;
        }
        let trimmed = line.trim();
        if in_pr {
            let indent = line.len() - line.trim_start().len();
            if trimmed.is_empty() {
                continue;
            }
            if indent > 2 {
                // Still inside the `pull_request:` mapping.
                if trimmed.starts_with("paths:")
                    || trimmed.starts_with("paths-ignore:")
                    || trimmed.starts_with("branches:")
                    || trimmed.starts_with("branches-ignore:")
                {
                    return false;
                }
                continue;
            }
            // Dedented back to a sibling key: the trigger carried no filter.
            return true;
        }
        if trimmed.starts_with("pull_request:") {
            in_pr = true;
        }
    }
    in_pr
}

/// Split the `jobs:` mapping into `(name, block)` pairs, in file order.
///
/// Works on a whole workflow AND on the output of [`pr_blocking_jobs`], which
/// is a sequence of job blocks with no `jobs:` header of its own: when no
/// `jobs:` line is present the whole text IS the mapping. Without that, the
/// shard-matrix walk re-split the filtered text, found nothing, and its own
/// anti-vacuity floor was the only thing that noticed.
fn jobs_of(text: &str) -> Vec<(String, String)> {
    let lines: Vec<&str> = text.lines().collect();
    let start = lines
        .iter()
        .position(|l| l.starts_with("jobs:"))
        .map(|i| i + 1)
        .unwrap_or(0);
    let mut out: Vec<(String, Vec<&str>)> = Vec::new();
    for line in &lines[start..] {
        let is_job_key =
            line.starts_with("  ") && !line.starts_with("   ") && line.trim_end().ends_with(':');
        if is_job_key {
            let name = line.trim().trim_end_matches(':').to_string();
            out.push((name, Vec::new()));
        }
        if let Some(last) = out.last_mut() {
            last.1.push(line);
        }
    }
    out.into_iter()
        .map(|(n, lines)| (n, lines.join("\n")))
        .collect()
}

/// The ONE job-level condition this walk sanctions, spelled whole.
///
/// A job-level `if:` normally disqualifies a job outright: it can stop the job
/// on a pull request, and a package covered only there is covered by nothing.
/// `!cancelled()` is the exception and it is the exception for the OPPOSITE
/// reason: it exists so the job runs when a job it `needs:` FAILED. GitHub
/// skips a dependant of a failed job, a skipped required context counts as
/// satisfied, and a pull request would then merge with that job's contexts
/// never run. So a job carrying exactly this condition runs on every pull
/// request that is not cancelled, and a cancelled run is not a pass.
///
/// EXACT, not a substring: `!cancelled() && github.event_name != 'pull_request'`
/// is a different condition and stays disqualified, which is why `deb-smoke`
/// (whose `if:` opens with the same call) is still dropped from this view.
const JOB_IF_NOT_CANCELLED: &str = "${{ !cancelled() }}";

/// The indent a job's own keys sit at: two for the job id, two more for the key.
const JOB_KEY_INDENT: &str = "    ";

/// The job-level `if:` value of one job block, read in every scalar form.
///
/// `None` when the job carries none. `Err` for a form the scalar reader cannot
/// classify, which the caller turns into a failure rather than into "no
/// condition": an unread job condition reads as an ungated job, and a job
/// behind `github.event_name == 'push'` would then credit coverage.
fn job_if_of(block: &str) -> Option<Result<String, String>> {
    let lines: Vec<&str> = block.lines().collect();
    let at = lines.iter().position(|l| l.starts_with("    if:"))?;
    let rest = &lines[at]["    if:".len()..];
    Some(read_scalar_value(
        "if",
        rest,
        &lines,
        at,
        JOB_KEY_INDENT.len(),
    ))
}

/// Does this job carry a job-level condition that can stop it on a pull
/// request?
fn job_is_gated(block: &str) -> bool {
    match job_if_of(block) {
        None => false,
        Some(Ok(cond)) => cond.trim() != JOB_IF_NOT_CANCELLED,
        Some(Err(why)) => panic!(
            "{why}\n\nThe walk cannot say whether this JOB runs on a pull \
             request, so it refuses to guess. Spell the job's `if:` as a \
             NON-EMPTY plain, quoted or block scalar, or teach \
             `read_scalar_form` the form."
        ),
    }
}

/// The member of a selection set that means "the doc classes changed".
///
/// It shares the set with package names, so it only works while no workspace
/// member carries that name. That is ASSERTED where the set is built, in
/// `every_package_with_tests_is_credited_when_it_alone_is_selected`, rather
/// than claimed here.
const SELECTION_DOCS_MARKER: &str = "docs";

/// The job id of the changed-path classifier whose outputs this walk reads.
const SELECTION_JOB: &str = "changes";

/// The output names the three sanctioned conditions read.
const SELECTION_CODE_OUTPUT: &str = "code";
const SELECTION_DOCS_OUTPUT: &str = "docs";
const SELECTION_PKGS_OUTPUT: &str = "pkgs";

/// The step conditions a changed-path classifier sets, spelled exactly.
///
/// Exact, not parsed: the value of these constants is that a near miss reads as
/// an ordinary unknown condition and fails closed. `== 'false'` inverts the
/// gate, and a different job id (`needs.other.outputs.code`) is a different
/// classifier whose rules this walk has never seen.
///
/// Spelling them whole AND naming the job and the outputs separately is
/// deliberate: the grounding check needs the parts, and
/// `the_sanctioned_conditions_are_spelled_from_the_job_and_output_names` holds
/// the two spellings to each other.
const SELECTION_CODE_IF: &str = "needs.changes.outputs.code == 'true'";
const SELECTION_DOCS_IF: &str = "needs.changes.outputs.docs == 'true'";

/// The per-package selection condition, as its two literal halves around the
/// package name: `contains(fromJSON(needs.changes.outputs.pkgs), '<pkg>')`.
const SELECTION_PKG_IF_OPEN: &str = "contains(fromJSON(needs.changes.outputs.pkgs), '";
const SELECTION_PKG_IF_CLOSE: &str = "')";

/// The package a per-package selection condition names, or `None` if the term
/// is not exactly that form around a cargo package name (ASCII alphanumeric,
/// `_` and `-`).
///
/// The charset is what keeps the form from swallowing a nested expression: a
/// term such as `contains(fromJSON(needs.changes.outputs.pkgs), 'a') ||
/// github.event_name == 'push'` has already been refused by the `||` rule, and
/// anything else that reaches here with a quote, a space or a dot inside is not
/// a package name and is not sanctioned.
fn selection_condition_package(term: &str) -> Option<&str> {
    let inner = term
        .trim()
        .strip_prefix(SELECTION_PKG_IF_OPEN)?
        .strip_suffix(SELECTION_PKG_IF_CLOSE)?;
    let is_package_name = !inner.is_empty()
        && inner
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
    is_package_name.then_some(inner)
}

/// The selection outputs a step's condition may read: the ones the classifier
/// declares AND produces, available to a job that needs the classifier.
type GroundedOutputs = BTreeSet<String>;

/// The legs a job's `strategy: matrix:` declares, `key -> {leg, ...}`.
type MatrixLegs = BTreeMap<String, BTreeSet<String>>;

/// The identifier charset a workflow uses for job ids, step ids, matrix keys
/// and package names.
fn is_name_char(ch: char) -> bool {
    ch.is_ascii_alphanumeric() || ch == '_' || ch == '-'
}

/// The indent of a key in a job's `outputs:` mapping: two for the job, two for
/// the key, two for the mapping under it.
const OUTPUT_KEY_INDENT: &str = "      ";

/// Which of the three selection outputs the classifier job of `jobs` declares
/// AND produces.
///
/// Read from that job's `outputs:` mapping, the keys at six-space indent under
/// it, so an output nobody publishes can ground nothing. A BLANK line does not
/// end the mapping: comments are blanked to empty strings before this walk sees
/// the file, and reading one as the end truncated the map at the first comment
/// between two keys.
///
/// Declaring the key is half of it. The VALUE has to be a value a STEP of that
/// job produced, because an output wired to anything else is the empty string
/// on every event, exactly like an output nobody declared at all: a step id the
/// job does not carry, a step that writes no output, a step that writes some
/// OTHER output's name, a literal.
///
/// The value is READ with [`read_scalar_value`], the same reader the `if:`
/// values go through, so an output written as a BLOCK scalar, `code: >-` with
/// the expression on the line below or `code: |`, is grounded on the EXPRESSION
/// rather than on the `>-` header. On the header alone it grounded
/// as a literal, and an output wired to a step id the classifier does not carry
/// was credited. A value this reader cannot classify FAILS the walk.
fn declared_selection_outputs(jobs: &[(String, String)]) -> GroundedOutputs {
    let mut out = BTreeSet::new();
    let Some((_, block)) = jobs.iter().find(|(name, _)| name == SELECTION_JOB) else {
        return out;
    };
    let producing = output_producing_steps(block);
    let lines: Vec<&str> = block.lines().collect();
    let Some(at) = lines.iter().position(|l| l.trim_end() == "    outputs:") else {
        return out;
    };
    for (offset, line) in lines[at + 1..].iter().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let Some(rest) = line.strip_prefix(OUTPUT_KEY_INDENT) else {
            break;
        };
        if rest.starts_with(' ') {
            continue;
        }
        let Some((key, head)) = rest.split_once(':') else {
            break;
        };
        let key = key.trim();
        if ![
            SELECTION_CODE_OUTPUT,
            SELECTION_DOCS_OUTPUT,
            SELECTION_PKGS_OUTPUT,
        ]
        .contains(&key)
        {
            continue;
        }
        let value = read_scalar_value(key, head, &lines, at + 1 + offset, OUTPUT_KEY_INDENT.len())
            .unwrap_or_else(|why| {
                panic!(
                    "{why}\n\nThe walk cannot say where the `{key}` output's \
                     value comes from, so it refuses to guess. Spell the value \
                     as a NON-EMPTY plain, quoted or block scalar, or teach \
                     `read_scalar_form` the form. Never leave it unread: an \
                     unread value reads as a literal that grounds itself, and \
                     every step gated on \
                     `needs.{SELECTION_JOB}.outputs.{key}` would then credit \
                     coverage from an output nobody produces."
                )
            });
        if output_is_produced(&value, &producing) {
            out.insert(key.to_string());
        }
    }
    out
}

/// The environment file a `run:` step appends to in order to set an output.
///
/// There is no other way for a run step to set one, so a run step whose block
/// never names it produces NOTHING, whatever `id:` it carries.
const GITHUB_OUTPUT: &str = "GITHUB_OUTPUT";

/// Does a declared output's VALUE come from a step that PRODUCES it?
///
/// Four rules, each of them fail-closed, and a declaration has to satisfy all
/// four. The value must be a PURE step-output reference
/// ([`pure_step_output_reference`]); the step it names must be one this job
/// carries; that step must be able to SET an output; and it must set the output
/// of THAT NAME ([`output_producing_steps`]).
///
/// A LITERAL grounds nothing. It reads like a value, `code: 'true'`, and it is
/// not one a classifier produced: nothing about the pull request can change
/// it, and its twin `code: 'false'` makes every step gated on `== 'true'` run
/// on no event at all while reading as a selected step. Crediting either is the
/// silent skip this walk exists to refuse.
///
/// `value` is the FOLDED scalar [`read_scalar_value`] returned, never the raw
/// text after the `:`: a block scalar's header carries no expression at all, so
/// asking this about `>-` asked it about a literal.
fn output_is_produced(value: &str, producing: &BTreeMap<String, StepOutputs>) -> bool {
    let Some((id, name)) = pure_step_output_reference(value) else {
        return false;
    };
    producing.get(&id).is_some_and(|step| step.writes(&name))
}

/// The step id AND the output name a value carries when it is EXACTLY one
/// step-output reference.
///
/// `${{ steps.<id>.outputs.<name> }}` and nothing else. The whitespace inside
/// the braces is YAML's to ignore and the scalar reader has already folded it,
/// but another OPERAND is not whitespace: `${{ github.event_name == 'push' &&
/// steps.c.outputs.code }}` names a step this job carries and is the empty
/// string on every pull request, so an output declared that way grounds a step
/// that never runs. A prefix, a suffix, a `||` default and a second reference
/// are the same shape and answer the same way.
fn pure_step_output_reference(value: &str) -> Option<(String, String)> {
    let inner = value.trim().strip_prefix("${{")?.strip_suffix("}}")?.trim();
    let after_steps = inner.strip_prefix("steps.")?;
    let id: String = after_steps
        .chars()
        .take_while(|c| is_name_char(*c))
        .collect();
    if id.is_empty() {
        return None;
    }
    let name = after_steps[id.len()..].strip_prefix(".outputs.")?;
    let is_output_name = !name.is_empty() && name.chars().all(is_name_char);
    is_output_name.then(|| (id, name.to_string()))
}

/// The lines of each step of one job block, in file order.
///
/// A step starts at a list item at the FIRST item's indent, so a `- ` inside a
/// `run:` script starts nothing, and it runs to the line before the next one.
fn step_blocks(job: &str) -> Vec<Vec<&str>> {
    let lines: Vec<&str> = job.lines().collect();
    let mut out: Vec<Vec<&str>> = Vec::new();
    let Some(steps_at) = lines.iter().position(|l| l.trim() == "steps:") else {
        return out;
    };
    let body = &lines[steps_at + 1..];
    let Some(step_indent) = body
        .iter()
        .find(|l| l.trim_start().starts_with("- "))
        .map(|l| indent_of(l))
    else {
        return out;
    };
    for line in body {
        if indent_of(line) == step_indent && line.trim_start().starts_with("- ") {
            out.push(Vec::new());
        }
        if let Some(step) = out.last_mut() {
            step.push(line);
        }
    }
    out
}

/// Does this step block carry `key` at the step's OWN indent?
///
/// The item line's `- <key>`, or a key two deeper, so a line inside a `run:`
/// script declares nothing.
fn step_carries_key(block: &[&str], key: &str) -> bool {
    let Some(first) = block.first() else {
        return false;
    };
    let step_indent = indent_of(first);
    block.iter().any(|line| {
        let trimmed = line.trim_start();
        match trimmed.strip_prefix("- ") {
            Some(item) if indent_of(line) == step_indent => item.trim_start().starts_with(key),
            _ => indent_of(line) == step_indent + 2 && trimmed.starts_with(key),
        }
    })
}

/// The `id:` one step block declares, or `None`.
///
/// Read at the step's own indent, the item line's `- id:` or a key two deeper,
/// so a line inside a `run:` script cannot invent one.
fn step_id_of(block: &[&str]) -> Option<String> {
    let step_indent = indent_of(block.first()?);
    for line in block {
        let trimmed = line.trim_start();
        let value = if indent_of(line) == step_indent && trimmed.starts_with("- ") {
            trimmed
                .strip_prefix("- ")
                .and_then(|r| r.strip_prefix("id:"))
        } else if indent_of(line) == step_indent + 2 {
            trimmed.strip_prefix("id:")
        } else {
            None
        };
        if let Some(id) = value {
            let id = id.trim().trim_matches(['\'', '"']);
            if !id.is_empty() {
                return Some(id.to_string());
            }
        }
    }
    None
}

/// What one step of a job can set.
///
/// A `uses:` step is credited with ANY output: the action's outputs are not in
/// this file to read, and refusing them would red a workflow that works. A
/// `run:` step is credited with exactly the names its SCRIPT writes.
enum StepOutputs {
    /// A `uses:` step: any name the action declares.
    Any,
    /// A `run:` step's script text, which is asked for each name.
    Written(String),
}

/// The characters a written output name may sit immediately after.
///
/// The start of the script and whitespace are the ordinary ones; either quote
/// covers `"<name>=$value"`, `{` a brace group, `(` a subshell, `;` the end of
/// the command before it, and `&` the second half of an `&&`. Anything else
/// means the name is the TAIL of a longer word.
const OUTPUT_TOKEN_OPENERS: &[char] = &['\'', '"', '{', '(', ';', '&'];

impl StepOutputs {
    /// Does this step write the output called `name`?
    ///
    /// A run script sets one by writing `<name>=<value>` into the output file,
    /// so the literal `<name>=` is what proves it. Asking only whether the
    /// script names the FILE credited every declared output to a classifier
    /// that writes one: the fixture here wrote `code=` alone, declared `code:`
    /// and `pkgs:`, and the step gated on the package list was credited while
    /// `fromJSON('')` would have failed on every pull request.
    ///
    /// A WHOLE token, at an [`OUTPUT_TOKEN_OPENERS`] boundary. As a bare
    /// substring `code=` is inside `barcode=1`, which writes `barcode` and
    /// leaves `code` the empty string, so a step gated on the `code` class
    /// would have been credited by a script that never sets it.
    fn writes(&self, name: &str) -> bool {
        let StepOutputs::Written(script) = self else {
            return true;
        };
        let needle = format!("{name}=");
        let mut from = 0usize;
        while let Some(offset) = script[from..].find(&needle) {
            let at = from + offset;
            from = at + needle.len();
            let opens_a_token = script[..at]
                .chars()
                .next_back()
                .is_none_or(|ch| ch.is_whitespace() || OUTPUT_TOKEN_OPENERS.contains(&ch));
            if opens_a_token {
                return true;
            }
        }
        false
    }
}

/// The text of one step's `run:` script, or `None` when it carries no `run:`.
///
/// The key's own line after the `run:` token, plus every following line
/// indented deeper than that key. NOT the step block whole: a `name:`, an
/// `env:` or an `if:` that merely mentions the output file writes nothing into
/// it, and reading the block whole credited a step on a word in its name.
fn run_script_of(block: &[&str]) -> Option<String> {
    let step_indent = indent_of(block.first()?);
    let key_column = step_indent + 2;
    let mut found = None;
    for (i, line) in block.iter().enumerate() {
        let trimmed = line.trim_start();
        let rest = match trimmed.strip_prefix("- ") {
            Some(item) if indent_of(line) == step_indent => item.trim_start().strip_prefix("run:"),
            _ if indent_of(line) == key_column => trimmed.strip_prefix("run:"),
            _ => None,
        };
        if let Some(rest) = rest {
            found = Some((i, rest));
            break;
        }
    }
    let (at, head) = found?;
    let mut script = String::from(head);
    for line in &block[at + 1..] {
        if line.trim().is_empty() {
            continue;
        }
        if indent_of(line) <= key_column {
            break;
        }
        script.push('\n');
        script.push_str(line.trim());
    }
    Some(script)
}

/// Every step of one job that can SET an output, by `id:`, and what it sets.
///
/// Two shapes, and no others. A `run:` step sets an output by appending to
/// [`GITHUB_OUTPUT`], so a `run:` step whose SCRIPT never names it produces
/// nothing: `- id: c` with `run: true` under it declares an id and writes no
/// output, and an output wired to it is the empty string on every event. A
/// `uses:` step is credited on its declaration alone.
fn output_producing_steps(job: &str) -> BTreeMap<String, StepOutputs> {
    let mut out = BTreeMap::new();
    for block in step_blocks(job) {
        let Some(id) = step_id_of(&block) else {
            continue;
        };
        let produces = if step_carries_key(&block, "uses:") {
            Some(StepOutputs::Any)
        } else {
            run_script_of(&block)
                .filter(|script| script.contains(GITHUB_OUTPUT))
                .map(StepOutputs::Written)
        };
        if let Some(produces) = produces {
            out.insert(id, produces);
        }
    }
    out
}

/// The legs a job's matrix declares, `key -> {leg, ...}`, in both YAML list
/// forms.
///
/// A matrix carrying `include:` or `exclude:` adds or removes legs this reader
/// does not model, so it reports NO legs at all and every `matrix.<key>`
/// condition in that job fails closed.
fn job_matrix_legs(block: &str) -> MatrixLegs {
    let lines: Vec<&str> = block.lines().collect();
    let mut out = MatrixLegs::new();
    let Some(at) = lines.iter().position(|l| l.trim() == "matrix:") else {
        return out;
    };
    let matrix_indent = indent_of(lines[at]);
    let mut i = at + 1;
    while i < lines.len() {
        let line = lines[i];
        if line.trim().is_empty() {
            i += 1;
            continue;
        }
        if indent_of(line) <= matrix_indent {
            break;
        }
        let key_indent = indent_of(line);
        let Some((key, value)) = line.trim().split_once(':') else {
            i += 1;
            continue;
        };
        let key = key.trim().to_string();
        if key == "include" || key == "exclude" {
            return MatrixLegs::new();
        }
        let value = value.trim();
        if let Some(inner) = value.strip_prefix('[').and_then(|v| v.strip_suffix(']')) {
            out.insert(
                key,
                inner
                    .split(',')
                    .map(|t| t.trim().trim_matches(['\'', '"']).to_string())
                    .filter(|t| !t.is_empty())
                    .collect(),
            );
            i += 1;
            continue;
        }
        if !value.is_empty() {
            i += 1;
            continue;
        }
        let mut legs = BTreeSet::new();
        i += 1;
        while i < lines.len() {
            let l = lines[i];
            if l.trim().is_empty() {
                i += 1;
                continue;
            }
            if indent_of(l) <= key_indent {
                break;
            }
            let Some(item) = l.trim().strip_prefix("- ") else {
                break;
            };
            legs.insert(item.trim().trim_matches(['\'', '"']).to_string());
            i += 1;
        }
        out.insert(key, legs);
    }
    out
}

/// Can this step-level `if:` condition stop the step running on a pull
/// request?
///
/// FAIL-CLOSED: anything not recognised as harmless disqualifies the step.
/// Recognised as harmless are `always()`, `success()`, and a
/// `matrix.<key> == <literal>` / `!= <literal>` LEG SELECTOR the job's own
/// matrix can satisfy: every leg of a PR-blocking job's matrix runs on every
/// pull request, so a step selected onto a leg that EXISTS runs on at least one
/// of them and can therefore fail one.
/// `github.event_name == 'push'`, `runner.os == 'Linux'` and anything with a
/// `||` do not qualify.
///
/// `grounded` names the selection outputs this step's job can actually read. A
/// selection condition over anything else is an expression over the empty
/// string: false on every event, so the step runs nowhere and is not
/// sanctioned.
///
/// `matrix` names the legs the step's own job declares. A selector whose key
/// the matrix does not carry, or whose literal is not one of that key's legs,
/// matches no leg at all: the step runs on no pull request while reading as an
/// ordinary leg-selected step, so it credits nothing. `!=` asks the same
/// question the other way round: some leg has to differ from the literal, or
/// the step is excluded from every leg there is.
fn step_if_is_pr_blocking_grounded(
    cond: &str,
    grounded: &GroundedOutputs,
    matrix: &MatrixLegs,
) -> bool {
    let cond = cond.trim();
    if cond.is_empty() {
        return true;
    }
    if cond.contains("||") {
        return false;
    }
    cond.split("&&").all(|term| {
        let t = term.trim();
        if t == "always()" || t == "success()" {
            return true;
        }
        // A sanctioned selection condition runs the step on exactly the pull
        // requests whose changed paths select it, so it can fail one, but
        // only where the value it reads exists.
        if t == SELECTION_CODE_IF {
            return grounded.contains(SELECTION_CODE_OUTPUT);
        }
        if t == SELECTION_DOCS_IF {
            return grounded.contains(SELECTION_DOCS_OUTPUT);
        }
        if selection_condition_package(t).is_some() {
            return grounded.contains(SELECTION_PKGS_OUTPUT);
        }
        let Some(rest) = t.strip_prefix("matrix.") else {
            return false;
        };
        let (key, literal, negated) = if let Some((key, literal)) = rest.split_once("==") {
            (key.trim(), literal.trim(), false)
        } else if let Some((key, literal)) = rest.split_once("!=") {
            (key.trim(), literal.trim(), true)
        } else {
            return false;
        };
        let literal = literal.trim_matches(['\'', '"']);
        let Some(legs) = matrix.get(key) else {
            return false;
        };
        if negated {
            legs.iter().any(|leg| leg != literal)
        } else {
            legs.contains(literal)
        }
    })
}

/// Every selection output, for the arms that ask about a CONDITION rather
/// than about a step in a workflow.
fn every_selection_output() -> GroundedOutputs {
    [
        SELECTION_CODE_OUTPUT,
        SELECTION_DOCS_OUTPUT,
        SELECTION_PKGS_OUTPUT,
    ]
    .into_iter()
    .map(str::to_string)
    .collect()
}

/// Every leg `ci.yml`'s matrix jobs declare, for the arms that ask about a
/// CONDITION rather than about a step in a job. An arm about the RESOLUTION of
/// a selector against a matrix builds its own.
fn every_matrix_leg() -> MatrixLegs {
    [
        ("shard", &["0", "1", "2", "3"][..]),
        ("lane", &["viz", "vizd"][..]),
        ("os", &["ubuntu-latest", "macos-latest"][..]),
    ]
    .into_iter()
    .map(|(key, legs)| {
        (
            key.to_string(),
            legs.iter().map(|leg| (*leg).to_string()).collect(),
        )
    })
    .collect()
}

/// [`step_if_is_pr_blocking_grounded`] with every selection output available
/// and every leg `ci.yml` declares: the question "is this CONDITION one of the
/// sanctioned shapes?", with grounding and leg resolution set aside.
fn step_if_is_pr_blocking(cond: &str) -> bool {
    step_if_is_pr_blocking_grounded(cond, &every_selection_output(), &every_matrix_leg())
}

/// Does this step-level `if:` hold when the classifier selected exactly
/// `selected`?
///
/// Only the three sanctioned selection terms consult the set: the per-package
/// form is true iff its package is in it, `code` iff the set is non-empty,
/// `docs` iff the set holds [`SELECTION_DOCS_MARKER`]. Every other term a
/// PR-blocking step can still carry (absent, `always()`, `success()`, a
/// `matrix.<key>` leg selector) does not depend on the selection and holds.
/// A condition this walk does not sanction never reaches here: its step was
/// already dropped by [`pr_blocking_workflow_texts`].
///
/// `code` follows the set being NON-EMPTY, which is the permissive reading for
/// a set whose only member is the `docs` marker. That shape is pinned by
/// `the_selection_evaluator_reads_the_code_and_docs_markers` and is never the
/// shape the totality arm evaluates, which is always one package name.
fn step_if_holds_under_selection(cond: &str, selected: &BTreeSet<String>) -> bool {
    cond.split("&&").all(|term| {
        let t = term.trim();
        if t == SELECTION_CODE_IF {
            return !selected.is_empty();
        }
        if t == SELECTION_DOCS_IF {
            return selected.contains(SELECTION_DOCS_MARKER);
        }
        match selection_condition_package(t) {
            Some(pkg) => selected.contains(pkg),
            None => true,
        }
    })
}

/// Drop every STEP of one job whose `if:` could stop it on a pull request.
///
/// `grounded` is the set of selection outputs THIS job can read, empty for a
/// job that does not need the classifier, so a selection condition there is not
/// sanctioned and its step goes. `matrix` is the legs THIS job declares, so a
/// leg selector is resolved against the matrix that would have to run it.
fn drop_gated_steps(job: &str, grounded: &GroundedOutputs, matrix: &MatrixLegs) -> String {
    retain_steps(job, |cond| {
        step_if_is_pr_blocking_grounded(cond, grounded, matrix)
    })
}

/// The indentation of one line.
fn indent_of(line: &str) -> usize {
    line.len() - line.trim_start().len()
}

/// One `<key>:` value, in every YAML scalar form the workflow tree uses, with
/// runs of whitespace folded to single spaces, and NEVER the empty string.
///
/// `key` is the key the value belongs to, spelled into the message an
/// unreadable form carries. `rest` is the text after the `key:` token on
/// `lines[at]`, and `key_column` is the column that token starts at: a scalar
/// CONTINUES on the following lines indented deeper than it, and stops at the
/// first non-blank line that is not. A blank line does not stop it, because
/// comments are blanked before this walk sees the file.
///
/// The forms: a plain scalar on the key's own line, a single- or double-quoted
/// one, a folded or literal block scalar (`>`, `>-`, `>+`, `|`, `|-`, `|+`),
/// and a plain scalar whose value begins on the following line. All of them
/// mean the same expression, and GitHub ignores the whitespace between its
/// tokens, which is why the value comes back folded.
///
/// FAIL CLOSED, twice. An unreadable form is an `Err` its caller turns into a
/// test failure, never an empty condition: an empty condition cannot stop a
/// step, so reading one is exactly how a gated step gets credited. An EMPTY
/// value is that same hole spelled in valid YAML. GitHub evaluates `if: ''` as
/// false and skips the step on every event, and an empty `outputs:` value is
/// the empty string every job that needs it reads, so it is an `Err` too.
fn read_scalar_value(
    key: &str,
    rest: &str,
    lines: &[&str],
    at: usize,
    key_column: usize,
) -> Result<String, String> {
    let value = read_scalar_form(key, rest, lines, at, key_column)?;
    if value.is_empty() {
        return Err(format!(
            "an empty `{key}:` value: `{key}: ''` and `{key}: \"\"` are valid \
             YAML and mean NOTHING at all"
        ));
    }
    Ok(value)
}

/// The scalar FORM of one `<key>:` value, folded, before the empty-value rule
/// [`read_scalar_value`] holds it to.
fn read_scalar_form(
    key: &str,
    rest: &str,
    lines: &[&str],
    at: usize,
    key_column: usize,
) -> Result<String, String> {
    let head = rest.trim();
    let mut continuation: Vec<&str> = Vec::new();
    for line in &lines[at + 1..] {
        if line.trim().is_empty() {
            continue;
        }
        if indent_of(line) <= key_column {
            break;
        }
        continuation.push(line.trim());
    }
    let folded = |parts: &[&str]| -> String {
        parts
            .join(" ")
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ")
    };

    if let Some(indicator) = head.strip_prefix(['>', '|']) {
        if !matches!(indicator, "" | "-" | "+") {
            return Err(format!(
                "a block scalar header this reader cannot classify: `{key}: {head}`"
            ));
        }
        if continuation.is_empty() {
            return Err(format!("a block `{key}:` with no value under it"));
        }
        return Ok(folded(&continuation));
    }
    if head.is_empty() {
        if continuation.is_empty() {
            return Err(format!("an `{key}:` with no value at all"));
        }
        if continuation[0].starts_with("- ") {
            return Err(format!("an `{key}:` whose value is a sequence"));
        }
        return Ok(folded(&continuation));
    }
    if head.starts_with(['\'', '"']) {
        let quote = head.chars().next().expect("the head is not empty");
        if !continuation.is_empty() {
            return Err(format!(
                "a quoted `{key}:` continued on another line: {head}"
            ));
        }
        let inner = head
            .strip_prefix(quote)
            .and_then(|h| h.strip_suffix(quote))
            .ok_or_else(|| format!("an unterminated quoted `{key}:`: {head}"))?;
        if quote == '"' && inner.contains('\\') {
            return Err(format!("an escaped double-quoted `{key}:`: {head}"));
        }
        let inner = if quote == '\'' {
            inner.replace("''", "'")
        } else {
            inner.to_string()
        };
        return Ok(folded(&[inner.as_str()]));
    }
    if head.starts_with(['*', '&', '!']) {
        return Err(format!(
            "an anchor, alias or tag as an `{key}:` value: {head}"
        ));
    }
    let mut parts = vec![head];
    parts.extend(continuation);
    Ok(folded(&parts))
}

/// Keep the steps of one job whose every `if:` satisfies `keep`, and drop the
/// rest.
///
/// Steps are list items under `steps:`; the item indent is read from the first
/// one rather than hard-coded, and a step's own keys sit two spaces deeper. The
/// `if:` VALUE is read in every scalar form by [`read_scalar_value`], and a form
/// it cannot classify PANICS: this walk exists to decide whether a condition
/// can stop a step, so a condition it cannot read is not one it may skip past.
fn retain_steps<F: Fn(&str) -> bool>(job: &str, keep: F) -> String {
    let lines: Vec<&str> = job.lines().collect();
    let Some(steps_at) = lines.iter().position(|l| l.trim() == "steps:") else {
        return job.to_string();
    };
    let mut out: Vec<&str> = lines[..=steps_at].to_vec();
    let body: Vec<&str> = lines[steps_at + 1..].to_vec();

    let Some(step_indent) = body
        .iter()
        .find(|l| l.trim_start().starts_with("- "))
        .map(|l| indent_of(l))
    else {
        out.extend_from_slice(&body);
        return out.join("\n");
    };
    let is_step_start = |l: &str| indent_of(l) == step_indent && l.trim_start().starts_with("- ");

    let mut cur: Vec<&str> = Vec::new();
    let mut gated = false;
    for (i, &line) in body.iter().enumerate() {
        if is_step_start(line) {
            if !gated {
                out.append(&mut cur);
            }
            cur.clear();
            gated = false;
        }
        let trimmed = line.trim_start();
        // A step's `if:` is either its own key (two deeper than the item) or
        // the first key on the `- if: …` item line itself. Either way the value
        // may continue on the lines below it.
        let found = if indent_of(line) == step_indent + 2 {
            trimmed.strip_prefix("if:")
        } else if is_step_start(line) {
            trimmed
                .strip_prefix("- ")
                .and_then(|r| r.strip_prefix("if:"))
        } else {
            None
        };
        if let Some(rest) = found {
            let cond =
                read_scalar_value("if", rest, &body, i, step_indent + 2).unwrap_or_else(|why| {
                    panic!(
                        "{why}\n\nThe walk cannot say whether this step runs on \
                         a pull request, so it refuses to guess. Spell the `if:` \
                         as a NON-EMPTY plain, quoted or block scalar, or teach \
                         `read_scalar_form` the form. Never leave it unread: an \
                         unread condition reads as NO condition, and a step \
                         behind `github.event_name == 'push'` would then credit \
                         coverage."
                    )
                });
            if !keep(&cond) {
                gated = true;
            }
        }
        cur.push(line);
    }
    if !gated {
        out.append(&mut cur);
    }
    out.join("\n")
}

/// Drop every job that carries a job-level `if:` or `continue-on-error: true`
/// (four-space indent, directly under the two-space job key), then drop every
/// gated STEP of the jobs that survive.
///
/// `continue-on-error: true` is the one that reads harmless and is not: `fuzz`
/// and `miri` both run on every pull request and neither can turn one red, so
/// a package named only there is named in nothing that gates anything.
fn pr_blocking_jobs(text: &str) -> String {
    let jobs = jobs_of(text);
    let has_job_if = job_is_gated;
    let is_soft = |block: &str| {
        block.lines().any(|l| {
            l.starts_with("    continue-on-error:")
                && l.split(':').nth(1).is_some_and(|v| v.trim() == "true")
        })
    };

    // A job-level `if:` gates the job it sits on AND every job that `needs:`
    // it, transitively: a skipped dependency skips its dependants (GitHub's
    // implicit `success()`), so a job with no `if:` of its own that needs a
    // gated one runs on exactly the events the gate runs on. No job in ci.yml
    // is that shape today; the rule is pinned on synthetic input by
    // `only_unfiltered_pull_request_workflows_and_ungated_jobs_are_pr_blocking`,
    // so it is in place before the next gate job arrives.
    // `continue-on-error: true` does NOT propagate: a soft job "succeeds" even
    // when it fails, so its dependants still run.
    let mut gated: BTreeSet<String> = jobs
        .iter()
        .filter(|(_, block)| has_job_if(block))
        .map(|(name, _)| name.clone())
        .collect();
    loop {
        let before = gated.len();
        for (name, block) in &jobs {
            if !gated.contains(name) && job_needs(block).iter().any(|n| gated.contains(n)) {
                gated.insert(name.clone());
            }
        }
        if gated.len() == before {
            break;
        }
    }

    // The selection outputs the classifier publishes, and which jobs can read
    // them. A job that does not need the classifier reads an empty string from
    // its outputs, so a step gated on one runs on NO event: that is not a
    // selected step, it is an off step, and crediting it would put a package's
    // coverage behind a condition that is never true.
    let declared = declared_selection_outputs(&jobs);
    let no_outputs: GroundedOutputs = BTreeSet::new();

    jobs.iter()
        .filter(|(name, block)| !gated.contains(name) && !is_soft(block))
        .map(|(_, block)| {
            let grounded = if job_needs(block).iter().any(|n| n == SELECTION_JOB) {
                &declared
            } else {
                &no_outputs
            };
            drop_gated_steps(block, grounded, &job_matrix_legs(block))
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// The job ids a job-level `needs:` names — inline `[a, b]`, a bare `a`, or
/// the block form (`needs:` followed by `      - a` items).
fn job_needs(block: &str) -> Vec<String> {
    let lines: Vec<&str> = block.lines().collect();
    let Some(i) = lines.iter().position(|l| l.starts_with("    needs:")) else {
        return Vec::new();
    };
    let value = lines[i]["    needs:".len()..].trim();
    if let Some(inner) = value.strip_prefix('[').and_then(|v| v.strip_suffix(']')) {
        return inner
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();
    }
    if !value.is_empty() {
        return vec![value.to_string()];
    }
    lines[i + 1..]
        .iter()
        .take_while(|l| l.starts_with("      - "))
        .map(|l| l.trim_start_matches("      - ").trim().to_string())
        .collect()
}

/// The PR-BLOCKING text of every workflow, comments stripped, keyed by file.
///
/// "PR-blocking" = an unfiltered `pull_request` trigger, minus every job that
/// carries a job-level `if:` (module docs explain why both filters exist and
/// what was measured without them).
fn pr_blocking_workflow_texts() -> BTreeMap<String, String> {
    let dir = repo_root().join(".github/workflows");
    let mut out = BTreeMap::new();
    let entries =
        std::fs::read_dir(&dir).unwrap_or_else(|e| panic!("cannot read {}: {e}", dir.display()));
    let mut seen_any = 0usize;
    for entry in entries.flatten() {
        let path = entry.path();
        let name = entry.file_name().to_string_lossy().into_owned();
        if !(name.ends_with(".yml") || name.ends_with(".yaml")) {
            continue;
        }
        seen_any += 1;
        let raw = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
        let stripped: String = raw
            .lines()
            .map(strip_yaml_comment)
            .collect::<Vec<_>>()
            .join("\n");
        if !runs_on_every_pull_request(&stripped) {
            continue;
        }
        out.insert(name, pr_blocking_jobs(&stripped));
    }
    assert!(
        seen_any >= 5,
        "the workflow walk found only {seen_any} file(s) under {} — it is not \
         reaching the workflows",
        dir.display()
    );
    assert!(
        !out.is_empty(),
        "no workflow was classified as running on every pull request — the \
         trigger classifier is broken, and this gate would then demand the \
         impossible of every package"
    );
    out
}

/// Does ONE command line name `pkg` as a package it runs tests for?
///
/// Two forms, and EVERY package on the line counts in both:
///
///   * `cargo test … -p a -p b …` — the ordinary one. Reading
///     only the package after the FIRST `-p` would leave the `b` of
///     `cargo test -p a -p b` uncovered. Every
///     `-p` / `--package` occurrence after `cargo test` is read, in every
///     spelling cargo accepts (`-p x`, `-px`, `--package x`, `--package=x`).
///   * `ci_test_shard.sh <pkg> …` — the sharded form
///     (`tools/scripts/ci_test_shard.sh`, used by the `cerulion_core` legs of
///     `test-linux` / `test-macos`): the script takes the package name as its
///     first argument precisely so the name stays LITERALLY present in the
///     workflow and this walk keeps working across the split.
///
/// Matching is by WHOLE WHITESPACE TOKEN, which is what keeps `cerulion_viz`
/// and `cerulion_vizd` apart: a plain `contains` would report `cerulion_viz`
/// covered by the `cerulion_vizd` step and vice versa.
fn line_names_package(line: &str, pkg: &str) -> bool {
    if let Some(pos) = line.find("ci_test_shard.sh ") {
        let tail = &line[pos + "ci_test_shard.sh ".len()..];
        if tail.split_whitespace().next() == Some(pkg) {
            return true;
        }
    }
    let Some(pos) = line.find("cargo test") else {
        return false;
    };
    let mut it = line[pos..].split_whitespace();
    while let Some(tok) = it.next() {
        let named = match tok {
            "-p" | "--package" => it.next() == Some(pkg),
            _ => tok
                .strip_prefix("--package=")
                .or_else(|| tok.strip_prefix("-p"))
                .is_some_and(|rest| rest == pkg),
        };
        if named {
            return true;
        }
    }
    false
}

/// The first PR-blocking workflow whose text names `pkg` in a test command.
fn workflows_name(texts: &BTreeMap<String, String>, pkg: &str) -> Option<String> {
    texts
        .iter()
        .find(|(_, text)| text.lines().any(|l| line_names_package(l, pkg)))
        .map(|(file, _)| file.clone())
}

// ---------------------------------------------------------------------------
// Totality under a hypothetical selection.
// ---------------------------------------------------------------------------

/// One PR-blocking workflow's text, with every step dropped whose sanctioned
/// condition is false when the classifier selected exactly `selected`.
///
/// Pure: it reads text and a set and returns text, so a caller can ask what any
/// selection would run without a workflow run, a network call or a clock.
fn workflow_under_selection(text: &str, selected: &BTreeSet<String>) -> String {
    let keep = |cond: &str| step_if_holds_under_selection(cond, selected);
    let jobs = jobs_of(text);
    if jobs.is_empty() {
        return retain_steps(text, keep);
    }
    jobs.iter()
        .map(|(_, block)| retain_steps(block, keep))
        .collect::<Vec<_>>()
        .join("\n")
}

/// Which of `packages` a PR-blocking step still names once the classifier has
/// selected exactly `selected`.
fn packages_credited_under_selection(
    texts: &BTreeMap<String, String>,
    selected: &BTreeSet<String>,
    packages: &BTreeSet<String>,
) -> BTreeSet<String> {
    let under: BTreeMap<String, String> = texts
        .iter()
        .map(|(file, text)| (file.clone(), workflow_under_selection(text, selected)))
        .collect();
    packages
        .iter()
        .filter(|pkg| workflows_name(&under, pkg).is_some())
        .cloned()
        .collect()
}

/// The packages that stop being credited when the classifier selects exactly
/// that one package, sorted, so the failure names every one of them.
///
/// This is the shape a selection rule breaks silently: the step that runs a
/// package's tests gated on a selection the change to that package does not
/// produce, so the pull request that most needs the test is the one that skips
/// it, and every other pull request keeps it green.
fn packages_lost_to_their_own_selection(
    texts: &BTreeMap<String, String>,
    packages: &BTreeSet<String>,
) -> Vec<String> {
    packages
        .iter()
        .filter(|pkg| {
            let selected: BTreeSet<String> = [(*pkg).clone()].into_iter().collect();
            !packages_credited_under_selection(texts, &selected, &selected).contains(*pkg)
        })
        .cloned()
        .collect()
}

// ---------------------------------------------------------------------------
// Workspace members, and which of them carry tests.
// ---------------------------------------------------------------------------

/// The `members = [ ... ]` array of the root manifest, globs expanded.
///
/// Hand-rolled rather than `cargo metadata` on purpose: a test that shells out
/// to cargo inherits cargo's lock and its multi-second resolve, and the
/// property under test is what the MANIFEST declares, which is exactly what a
/// maintainer reads.
fn workspace_member_dirs() -> Vec<PathBuf> {
    let root = repo_root();
    let manifest = std::fs::read_to_string(root.join("Cargo.toml")).expect("root Cargo.toml");
    let start = manifest
        .find("members = [")
        .expect("root Cargo.toml declares `members = [`");
    let rest = &manifest[start..];
    let end = rest
        .find("\n]")
        .expect("the members array is closed by a `]`");
    let body = &rest[..end];

    let mut out = Vec::new();
    for line in body.lines() {
        let line = line.trim();
        if line.starts_with('#') {
            continue;
        }
        let Some(open) = line.find('"') else { continue };
        let Some(close_rel) = line[open + 1..].find('"') else {
            continue;
        };
        let entry = &line[open + 1..open + 1 + close_rel];
        if let Some(parent) = entry.strip_suffix("/*") {
            let dir = root.join(parent);
            let mut expanded: Vec<PathBuf> = std::fs::read_dir(&dir)
                .unwrap_or_else(|e| panic!("cannot expand glob {entry}: {e}"))
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.join("Cargo.toml").is_file())
                .collect();
            expanded.sort();
            out.extend(expanded);
        } else {
            out.push(root.join(entry));
        }
    }
    assert!(
        out.len() > 20,
        "the members walk found only {} entries — it is not parsing the root \
         manifest",
        out.len()
    );
    out
}

/// The `name = "..."` of a crate manifest.
///
/// Read from the manifest rather than inferred from the directory because
/// they DIVERGE: `cerulion_wire/` publishes as `cerulion-wire`, and a walk
/// that guessed from the path would demand a `-p cerulion_wire` step that
/// cargo would reject.
fn package_name(dir: &Path) -> String {
    let text = std::fs::read_to_string(dir.join("Cargo.toml"))
        .unwrap_or_else(|e| panic!("cannot read {}/Cargo.toml: {e}", dir.display()));
    for line in text.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("name") {
            let rest = rest.trim_start();
            if let Some(rest) = rest.strip_prefix('=') {
                let rest = rest.trim();
                if let Some(inner) = rest.strip_prefix('"') {
                    if let Some(end) = inner.find('"') {
                        return inner[..end].to_string();
                    }
                }
            }
        }
        // Stop at the next table: a `name` under `[[bin]]` is not the package.
        if line.starts_with('[') && !line.starts_with("[package]") {
            break;
        }
    }
    panic!("{}/Cargo.toml declares no package name", dir.display());
}

/// Does this package carry tests a CI step could run?
///
/// Two sources, matching what `cargo test -p <name>` would execute:
///   * an integration-test file — `tests/*.rs` at depth 1 (a `tests/ui/**`
///     trybuild source or a `tests/common/mod.rs` helper is not its own
///     target, which is why the check is depth-1);
///   * a test ATTRIBUTE anywhere under `src/` (the `--lib` unit suite, which
///     for several packages here is the ONLY suite).
///
/// Doctests are deliberately NOT detected — see the module docs for why that
/// is a stated limitation.
fn has_tests(dir: &Path) -> bool {
    let tests_dir = dir.join("tests");
    if let Ok(entries) = std::fs::read_dir(&tests_dir) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_file() && p.extension().is_some_and(|e| e == "rs") {
                return true;
            }
        }
    }
    src_has_test_attr(&dir.join("src"))
}

/// Attribute forms that make a function a test target.
///
/// PREFIXES, not exact strings: `#[tokio::test(flavor = "multi_thread")]` is
/// the shape the exact `#[tokio::test]` match missed entirely, so a package
/// whose whole suite is async read as having no tests and was never demanded
/// of. `#[test_case` / `#[rstest` are the same class one crate away.
const TEST_ATTR_PREFIXES: &[&str] = &["#[test]", "#[tokio::test", "#[test_case", "#[rstest"];

fn src_has_test_attr(dir: &Path) -> bool {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return false;
    };
    for entry in entries.flatten() {
        let p = entry.path();
        if p.is_dir() {
            if src_has_test_attr(&p) {
                return true;
            }
        } else if p.extension().is_some_and(|e| e == "rs") {
            if let Ok(text) = std::fs::read_to_string(&p) {
                if TEST_ATTR_PREFIXES.iter().any(|a| text.contains(a)) {
                    return true;
                }
            }
        }
    }
    false
}

/// `package name -> directory` for every member that carries tests.
fn packages_with_tests() -> BTreeMap<String, PathBuf> {
    let mut out = BTreeMap::new();
    for dir in workspace_member_dirs() {
        if !dir.join("Cargo.toml").is_file() {
            panic!(
                "{} is declared in the root `members` but has no Cargo.toml — \
                 a deleted crate must be removed from the manifest",
                dir.display()
            );
        }
        if has_tests(&dir) {
            out.insert(package_name(&dir), dir);
        }
    }
    assert!(
        out.len() >= 20,
        "only {} member(s) were classified as having tests — the classifier is \
         not reading the tree",
        out.len()
    );
    out
}

// ---------------------------------------------------------------------------
// THE GATE
// ---------------------------------------------------------------------------

#[test]
fn every_package_with_tests_is_named_in_a_ci_test_step() {
    let texts = pr_blocking_workflow_texts();
    let with_tests = packages_with_tests();
    let exempt: BTreeMap<&str, &str> = EXEMPT_PACKAGES.iter().copied().collect();

    let mut covered: BTreeMap<String, String> = BTreeMap::new();
    let mut uncovered: Vec<String> = Vec::new();

    for (pkg, dir) in &with_tests {
        match workflows_name(&texts, pkg) {
            Some(file) => {
                covered.insert(pkg.clone(), file);
            }
            None => {
                if !exempt.contains_key(pkg.as_str()) {
                    uncovered.push(format!(
                        "  {pkg}  ({})",
                        dir.strip_prefix(repo_root()).unwrap_or(dir).display()
                    ));
                }
            }
        }
    }

    assert!(
        uncovered.is_empty(),
        "these workspace packages have tests that run in NO CI job:\n{}\n\n\
         Every test step in .github/workflows/ci.yml names its packages \
         explicitly (there is no blanket `cargo test --workspace` — it \
         deadlocks on iceoryx2's shared-memory singleton), so a package is run \
         only if a step names it.\n\
         FIX: add `run: cargo test -p <package>` to a job in \
         .github/workflows/ (the `crate-tests` job is where the light packages \
         live), or — if it genuinely must not run — add it to EXEMPT_PACKAGES \
         in this file WITH A REASON.\n\
         A mention inside a YAML comment does not count: comments are stripped \
         before the scan, deliberately.",
        uncovered.join("\n")
    );

    // ---- the other direction: no stale or wrong exemption -----------------
    let mut bad_exemptions: Vec<String> = Vec::new();
    for (pkg, reason) in EXEMPT_PACKAGES {
        assert!(
            !reason.trim().is_empty(),
            "the EXEMPT_PACKAGES entry for `{pkg}` carries an empty reason — \
             an exemption without a reason is a hole nobody can review"
        );
        if let Some(file) = covered.get(*pkg) {
            bad_exemptions.push(format!(
                "  {pkg}: exempt, but {file} DOES name it — delete the exemption"
            ));
        } else if !with_tests.contains_key(*pkg) {
            bad_exemptions.push(format!(
                "  {pkg}: exempt, but it is not a workspace member with tests — \
                 delete the exemption (a waiver that describes nothing \
                 pre-authorises the next hole)"
            ));
        }
    }
    assert!(
        bad_exemptions.is_empty(),
        "EXEMPT_PACKAGES is stale:\n{}",
        bad_exemptions.join("\n")
    );

    // ---- anti-tautology ---------------------------------------------------
    // Without this, a stripper that emptied every workflow, or a matcher that
    // never matched, would make the assertions above vacuous in the SAFE
    // direction (nothing covered => nothing to contradict) only if the
    // exemption list also grew — but a matcher that ALWAYS matched would make
    // the coverage half vacuous silently. Require real, specific hits.
    assert!(
        covered.len() >= 20,
        "only {} package(s) were found named in a workflow — the matcher or the \
         comment stripper is broken, and this gate would be vacuous",
        covered.len()
    );
    for anchor in ["cerulion_core", "cerulion_cli_engine", "cerulion_vizd"] {
        assert!(
            covered.contains_key(anchor),
            "`{anchor}` is not detected as covered — it certainly is, so the \
             matcher is broken"
        );
    }
}

/// The name-boundary rule is what keeps `cerulion_viz` and `cerulion_vizd`
/// apart, and it is asserted directly rather than trusted: with a plain
/// `contains`, deleting the `cerulion_viz` step would leave the gate green
/// because the `cerulion_vizd` step's text still contains its name.
#[test]
fn a_longer_package_name_does_not_cover_its_own_prefix() {
    let texts: BTreeMap<String, String> = [(
        "fake.yml".to_string(),
        "      - run: cargo test -p cerulion_vizd -- --test-threads=1\n".to_string(),
    )]
    .into_iter()
    .collect();

    assert!(
        workflows_name(&texts, "cerulion_vizd").is_some(),
        "the exact name must match"
    );
    assert!(
        workflows_name(&texts, "cerulion_viz").is_none(),
        "`cerulion_viz` must NOT be reported covered by a `cerulion_vizd` step"
    );
    // The sharded form is recognised too, under the same boundary rule.
    let sharded: BTreeMap<String, String> = [(
        "fake.yml".to_string(),
        "      - run: ./tools/scripts/ci_test_shard.sh cerulion_core 0 4\n".to_string(),
    )]
    .into_iter()
    .collect();
    assert!(workflows_name(&sharded, "cerulion_core").is_some());
    assert!(workflows_name(&sharded, "cerulion_cor").is_none());
}

/// The comment stripper, pinned on both sides — over-stripping fails closed,
/// under-stripping makes a coverage claim satisfiable by prose.
#[test]
fn yaml_comments_are_stripped_and_nothing_else_is() {
    assert_eq!(strip_yaml_comment("  # cargo test -p ghost"), "");
    assert_eq!(strip_yaml_comment("# cargo test -p ghost"), "");
    assert_eq!(
        strip_yaml_comment("        run: cargo test -p real # why"),
        "        run: cargo test -p real"
    );
    // A `#` NOT preceded by whitespace is not a comment opener.
    assert_eq!(
        strip_yaml_comment("        run: echo a#b -p real"),
        "        run: echo a#b -p real"
    );
    assert_eq!(
        strip_yaml_comment("        run: cargo test -p real"),
        "        run: cargo test -p real"
    );

    // And end to end: a package named ONLY in a comment is not covered.
    let texts: BTreeMap<String, String> = [(
        "fake.yml".to_string(),
        "      # TODO: cargo test -p ghost_pkg\n"
            .lines()
            .map(strip_yaml_comment)
            .collect::<Vec<_>>()
            .join("\n"),
    )]
    .into_iter()
    .collect();
    assert!(
        workflows_name(&texts, "ghost_pkg").is_none(),
        "a package mentioned only in a YAML comment must not read as covered"
    );
}

/// `tools/scripts/ci_test_shard.sh --check` proves the shard partition is TOTAL and
/// DISJOINT — no test file runs twice, and none runs in no shard at all.
///
/// It is driven from here rather than left to CI alone for the same reason the
/// gate above exists: the shard split is the mechanism that decides what runs,
/// so a developer must be able to break it and see it break locally.
#[cfg(unix)]
#[test]
fn the_ci_test_shard_partition_is_total_and_disjoint() {
    let script = repo_root().join("tools/scripts/ci_test_shard.sh");
    assert!(script.is_file(), "{} is missing", script.display());

    let out = std::process::Command::new("bash")
        .arg(&script)
        .arg("--check")
        .current_dir(repo_root())
        .output()
        .expect("running tools/scripts/ci_test_shard.sh --check");

    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success(),
        "tools/scripts/ci_test_shard.sh --check FAILED — the shard split is not a \
         partition, so some test file would run twice or not at all.\n\
         stdout:\n{stdout}\nstderr:\n{stderr}"
    );
    assert!(
        stdout.contains("total and disjoint"),
        "the --check verdict line is missing; got:\n{stdout}"
    );

    // The ONE routing exception must be reported, by name and by shard.
    //
    // `--check` pins `macro_compile_fail_test` to a fixed shard because it is the
    // serial trybuild tail: one test that is essentially the whole of whichever
    // quarter holds it. This comment is
    // figure-free on purpose — the measurement and its provenance live in exactly
    // one place, `PINNED_TEST` in `tools/scripts/ci_test_shard.sh`. What matters here is
    // that `.github/workflows/ci.yml`'s package map counts the tail as a FIXED
    // term of that shard's wall and packs the package steps around it. The
    // assertions above cannot see any of that: a pin that drifted to another
    // shard, or one whose target file was renamed away so the tail rejoined the
    // round-robin, both leave the partition perfectly total and disjoint.
    //
    // So this asserts the LINE, which is what makes the pin observable. Without
    // it the pin's own guard can go inert — silently, and in exactly the
    // direction that puts the tail back on a runner chosen by an unrelated PR.
    let expected_pin = "pinned: macro_compile_fail_test -> shard 2";
    assert!(
        stdout.contains(expected_pin),
        "the --check pin line is missing.\n\
         Expected a line containing: {expected_pin}\n\
         Either the pin moved (re-balance the package map in \
         .github/workflows/ci.yml to match), or its target file was renamed \
         or deleted (see PINNED_TEST in tools/scripts/ci_test_shard.sh).\n\
         got:\n{stdout}"
    );
}

/// Split a command tail into arguments, treating a whole `${{ ... }}`
/// expression as ONE argument.
///
/// Naive whitespace splitting turns `${{ matrix.shard }}` into three tokens
/// and shifts every argument after it — which is not a cosmetic problem here,
/// because the argument that would then be read as the shard COUNT is the
/// literal `}}`.
fn shell_args(tail: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut rest = tail.trim();
    while !rest.is_empty() {
        if let Some(inner) = rest.strip_prefix("${{") {
            let Some(end) = inner.find("}}") else {
                break;
            };
            out.push(format!("${{{{{}}}}}", &inner[..end]));
            rest = inner[end + 2..].trim_start();
            continue;
        }
        let end = rest.find(char::is_whitespace).unwrap_or(rest.len());
        out.push(rest[..end].to_string());
        rest = rest[end..].trim_start();
    }
    out
}

/// The shard COUNT passed to `tools/scripts/ci_test_shard.sh` must equal the length
/// of the `shard:` matrix that supplies its index, and that matrix must be
/// exactly `0..count-1`.
///
/// Two numbers in two places decide what runs. If the matrix is
/// `[0, 1, 2, 3]` and the count argument says `3`, index 3 is out of range and
/// that leg dies loudly (the script rejects it) — survivable. The other
/// direction is the dangerous one: matrix `[0, 1, 2]` with a count of `4`
/// silently runs three quarters of the suite and reports green, which is the
/// exact "ran 178 of 237 files" shape the split exists not to have.
///
/// Read out of the workflow text rather than asserted against a constant here,
/// because the workflow is the thing that decides.
#[test]
fn the_ci_shard_matrix_agrees_with_the_shard_count_argument() {
    let texts = pr_blocking_workflow_texts();

    let mut invocations = 0usize;
    for (file, text) in &texts {
        // PER JOB. File-global collection let ONE job's correct matrix vouch
        // for a sibling job that declared none — and a shard invocation whose
        // job has no matrix runs with an empty `matrix.shard`, i.e. nothing.
        for (job, block) in jobs_of(text) {
            invocations += check_shard_invocations(file, &job, &block);
        }
    }

    // BOTH sharded jobs: `test-linux` (4) and `test-macos` (2) run on every
    // pull request and neither carries a job-level `if:`, so the walk above
    // reaches both and each one's matrix is checked against its OWN count. The
    // floor is 2 rather than 1, so losing either job's shard step (which would
    // leave the other vouching for the pair) fails here instead of passing
    // quietly.
    assert!(
        invocations >= 2,
        "expected the sharded core step in BOTH `test-linux` and `test-macos`, found \
         {invocations} shard invocation(s); this gate would be vacuous"
    );
}

/// Check every `ci_test_shard.sh <package> <index> <count>` invocation in ONE
/// job block against that job's OWN `shard:` matrix; returns how many it
/// checked so a caller can refuse a vacuous walk.
fn check_shard_invocations(file: &str, job: &str, block: &str) -> usize {
    let matrices = shard_matrices(block);
    let mut invocations = 0usize;
    for line in block.lines() {
        let Some(pos) = line.find("ci_test_shard.sh ") else {
            continue;
        };
        let args = shell_args(&line[pos + "ci_test_shard.sh ".len()..]);
        let args: Vec<&str> = args.iter().map(String::as_str).collect();
        if args.first().is_some_and(|a| a.starts_with("--")) {
            continue; // `--check` / `--list`, not a shard run
        }
        assert!(
            args.len() >= 3,
            "{file}:{job}: a shard invocation needs `<package> <index> <count>`; \
             got: {line}"
        );
        let index_expr = args[1];
        let count: usize = args[2].parse().unwrap_or_else(|e| {
            panic!(
                "{file}:{job}: shard count `{}` is not a number ({e}) in: {line}",
                args[2]
            )
        });
        assert_eq!(
            index_expr, "${{ matrix.shard }}",
            "{file}:{job}: the shard INDEX must come from the `shard:` matrix, so \
             the matrix length and the count argument stay checkable against each \
             other; got `{index_expr}` in: {line}"
        );
        assert!(
            !matrices.is_empty(),
            "{file}:{job}: a shard invocation reads `matrix.shard` but this JOB \
             declares no `shard:` matrix — a sibling job's matrix does not supply \
             one, and an unexpanded `matrix.shard` runs nothing"
        );
        for m in &matrices {
            let expected: Vec<String> = (0..count).map(|i| i.to_string()).collect();
            assert_eq!(
                *m,
                expected,
                "{file}:{job}: the `shard:` matrix must be exactly 0..{} to match \
                 the shard count `{count}` passed to ci_test_shard.sh. A matrix \
                 SHORTER than the count runs only part of the suite and still \
                 reports green.",
                count - 1
            );
        }
        invocations += 1;
    }
    invocations
}

/// The dangerous direction of the shard pin, on a synthetic job: a matrix
/// SHORTER than the count is refused (it would run part of the suite and
/// report green). The real-tree arms above are the positive controls.
#[test]
#[should_panic(expected = "must be exactly 0..3")]
fn a_shard_matrix_shorter_than_the_count_is_refused() {
    let block = "  j:\n    strategy:\n      matrix:\n        shard: [0, 1, 2]\n    steps:\n      \
                 - run: ./tools/scripts/ci_test_shard.sh pkg ${{ matrix.shard }} 4\n";
    check_shard_invocations("synthetic.yml", "j", block);
}

/// Every `shard:` matrix declared in one job block, in BOTH YAML list forms.
///
/// The inline `shard: [0, 1, 2, 3]` was the only one parsed, so a maintainer
/// reformatting the matrix into the equally-valid block form
/// (`shard:` / `  - 0` / `  - 1`) made it INVISIBLE — and an invisible matrix
/// is not a red, it is a `matrices.is_empty()` that no longer fires because
/// the assertion it guards was never reached with anything to compare.
fn shard_matrices(job: &str) -> Vec<Vec<String>> {
    let mut out = Vec::new();
    let lines: Vec<&str> = job.lines().collect();

    let mut i = 0usize;
    while i < lines.len() {
        let line = lines[i];
        let trimmed = line.trim();
        let Some(rest) = trimmed.strip_prefix("shard:") else {
            i += 1;
            continue;
        };
        let rest = rest.trim();
        if let Some(inner) = rest.strip_prefix('[').and_then(|r| r.strip_suffix(']')) {
            out.push(
                inner
                    .split(',')
                    .map(|t| t.trim().to_string())
                    .filter(|t| !t.is_empty())
                    .collect(),
            );
            i += 1;
            continue;
        }
        if !rest.is_empty() {
            i += 1;
            continue;
        }
        // Block form: `- <item>` lines indented deeper than the `shard:` key.
        let key_indent = indent_of(line);
        let mut items: Vec<String> = Vec::new();
        i += 1;
        while i < lines.len() {
            let l = lines[i];
            if l.trim().is_empty() {
                i += 1;
                continue;
            }
            if indent_of(l) <= key_indent {
                break;
            }
            let Some(item) = l.trim().strip_prefix("- ") else {
                break;
            };
            items.push(item.trim().trim_matches(['\'', '"']).to_string());
            i += 1;
        }
        out.push(items);
    }
    out
}

/// Fixture crates carry no tests, so they are correctly absent from the
/// coverage set — and that absence is asserted, not assumed. If a fixture ever
/// grows a `#[test]`, the gate above starts demanding a step for it, and this
/// test is where a reader learns that is intended.
#[test]
fn test_fixture_cdylibs_carry_no_tests_of_their_own() {
    let with_tests = packages_with_tests();
    let fixtures: BTreeSet<&String> = with_tests
        .iter()
        .filter(|(_, dir)| dir.to_string_lossy().contains("test_fixtures/"))
        .map(|(name, _)| name)
        .collect();
    assert!(
        fixtures.is_empty(),
        "these test_fixtures crates now carry their own tests and so need a CI \
         step (or an exemption): {fixtures:?}"
    );
}

/// The two PR-blocking classifiers, pinned on BOTH sides against hand-written
/// documents.
///
/// Without these, either classifier could silently widen — accepting a
/// `schedule`-only workflow, or a job behind the push allowlist — and the gate
/// above would keep reporting green while the packages it claims to cover ran
/// in nothing that can fail a pull request.
#[test]
fn only_unfiltered_pull_request_workflows_and_ungated_jobs_are_pr_blocking() {
    // ---- the trigger classifier ------------------------------------------
    assert!(
        runs_on_every_pull_request("on:\n  pull_request:\n  push:\n    branches: [main]\njobs:\n"),
        "a bare `pull_request:` trigger runs on every PR"
    );
    assert!(
        runs_on_every_pull_request(
            "on:\n  pull_request:\n    types: [opened, synchronize]\njobs:\n"
        ),
        "a `types:`-narrowed trigger still runs on every PR that reaches those types"
    );
    assert!(
        !runs_on_every_pull_request(
            "on:\n  pull_request:\n    paths: ['examples/**']\n  workflow_dispatch:\njobs:\n"
        ),
        "a `paths:`-filtered trigger runs on SOME pull requests — not coverage \
         (the examples-replay.yml shape)"
    );
    assert!(
        !runs_on_every_pull_request(
            "on:\n  schedule:\n    - cron: '0 3 * * *'\n  workflow_dispatch:\njobs:\n"
        ),
        "a nightly lane gates no pull request (the nightly-*.yml shape — the \
         measured false green this rule exists to kill)"
    );
    assert!(
        !runs_on_every_pull_request("on:\n  push:\n    branches: [main]\njobs:\n"),
        "push-to-main is not a pull-request gate"
    );

    // ---- the job classifier ----------------------------------------------
    let doc = "on:\n  pull_request:\njobs:\n  \
               open_job:\n    runs-on: ubuntu-latest\n    steps:\n      \
               - run: cargo test -p open_pkg\n  \
               gated_job:\n    runs-on: ubuntu-latest\n    \
               if: github.event_name == 'push'\n    steps:\n      \
               - run: cargo test -p allowlisted_pkg\n";
    let kept = pr_blocking_jobs(doc);
    assert!(
        kept.contains("open_pkg"),
        "an ungated job must survive; got:\n{kept}"
    );
    assert!(
        !kept.contains("allowlisted_pkg"),
        "a job behind a job-level `if:` gates no pull request and must be \
         dropped; got:\n{kept}"
    );

    // A STEP-level `if:` is deeper than four spaces and must NOT drop its job:
    // `test-linux`'s Linux-only reclaim step is exactly that shape.
    let step_if = "on:\n  pull_request:\njobs:\n  \
                   j:\n    runs-on: ubuntu-latest\n    steps:\n      \
                   - name: reclaim\n        if: runner.os == 'Linux'\n        \
                   run: df -h /\n      - run: cargo test -p step_if_pkg\n";
    assert!(
        pr_blocking_jobs(step_if).contains("step_if_pkg"),
        "a STEP-level `if:` must not disqualify its whole job"
    );

    // ---- a `needs:` on an `if:`-gated job gates TRANSITIVELY ---------------
    // A job with no `if:` of its own that `needs:` a gated one: a skipped
    // dependency skips its dependants, so the job runs on no pull request and
    // must not credit coverage, nor may anything downstream of it, in either
    // `needs:` form.
    let needs_doc = "on:\n  pull_request:\njobs:\n  \
                     gate:\n    runs-on: ubuntu-latest\n    \
                     if: github.event_name != 'pull_request'\n    steps:\n      \
                     - run: true\n  \
                     open_job:\n    runs-on: ubuntu-latest\n    steps:\n      \
                     - run: cargo test -p open_pkg\n  \
                     gated_job:\n    runs-on: ubuntu-latest\n    \
                     needs: [open_job, gate]\n    steps:\n      \
                     - run: cargo test -p needs_gated_pkg\n  \
                     downstream:\n    runs-on: ubuntu-latest\n    needs:\n      \
                     - gated_job\n    steps:\n      \
                     - run: cargo test -p transitively_gated_pkg\n  \
                     sibling:\n    runs-on: ubuntu-latest\n    needs: [open_job]\n    \
                     steps:\n      - run: cargo test -p needs_open_pkg\n";
    let kept = pr_blocking_jobs(needs_doc);
    assert!(
        kept.contains("open_pkg") && kept.contains("needs_open_pkg"),
        "jobs that need only ungated jobs must survive; got:\n{kept}"
    );
    assert!(
        !kept.contains("needs_gated_pkg"),
        "a job whose `needs:` names an `if:`-gated job is skipped whenever the gate \
         is and must be dropped; got:\n{kept}"
    );
    assert!(
        !kept.contains("transitively_gated_pkg"),
        "the gate propagates through `needs:` (block form too); got:\n{kept}"
    );

    // ---- and a `continue-on-error: true` JOB gates nothing ----------------
    // `fuzz` and `miri` are exactly this shape: they run on every pull request
    // and neither can turn one red, so a package named only there is named in
    // nothing that can fail.
    let soft = "on:\n  pull_request:\njobs:\n  \
                soft_job:\n    runs-on: ubuntu-latest\n    \
                continue-on-error: true\n    steps:\n      \
                - run: cargo test -p soft_pkg\n  \
                hard_job:\n    runs-on: ubuntu-latest\n    \
                continue-on-error: false\n    steps:\n      \
                - run: cargo test -p hard_pkg\n";
    let kept = pr_blocking_jobs(soft);
    assert!(
        !kept.contains("soft_pkg"),
        "a `continue-on-error: true` job cannot fail a pull request and must be \
         dropped; got:\n{kept}"
    );
    assert!(
        kept.contains("hard_pkg"),
        "`continue-on-error: false` is not a soft job — it must survive; got:\n{kept}"
    );
}

/// A STEP whose `if:` can stop it running on a pull request must not credit
/// coverage — and a leg selector, which cannot, must still count.
///
/// The blunt rule ("any `if:` disqualifies") was not available: twelve real
/// steps in `test-linux` / `test-macos` carry `if: matrix.shard == N`, and
/// every leg of a PR-blocking job's matrix runs on every pull request, so
/// those steps genuinely can fail one. The line is drawn at what the condition
/// depends on, not at whether one exists.
#[test]
fn a_step_gated_off_pull_requests_does_not_credit_coverage() {
    // ---- the pure predicate, both directions ------------------------------
    for ok in [
        "",
        "always()",
        "success()",
        "matrix.shard == 0",
        "matrix.shard != 0",
        "matrix.lane == 'viz'",
        "always() && matrix.shard == 3",
        // The sanctioned selection conditions, and a `&&` chain of them.
        "needs.changes.outputs.code == 'true'",
        "needs.changes.outputs.docs == 'true'",
        "contains(fromJSON(needs.changes.outputs.pkgs), 'cerulion_core')",
        "contains(fromJSON(needs.changes.outputs.pkgs), 'cerulion-wire')",
        "needs.changes.outputs.code == 'true' && matrix.shard == 0",
        "contains(fromJSON(needs.changes.outputs.pkgs), 'go2_tf') && success()",
    ] {
        assert!(
            step_if_is_pr_blocking(ok),
            "`{ok}` cannot stop a step running on a pull request"
        );
    }
    for bad in [
        "github.event_name == 'push'",
        "github.event_name != 'pull_request'",
        "runner.os == 'Linux'",
        "matrix.shard == 0 || github.event_name == 'push'",
        "failure()",
        "github.ref == 'refs/heads/main' && matrix.shard == 0",
        // Near misses of the sanctioned forms. The inverted comparison gates
        // the step on the classifier NOT selecting; a different job id is a
        // different classifier; a `||` reaches an event test whatever the
        // classifier said; a package name is not an expression.
        "needs.changes.outputs.code == 'false'",
        "needs.changes.outputs.docs == 'false'",
        "needs.other.outputs.code == 'true'",
        "needs.changes.outputs.pkgs == 'true'",
        "contains(fromJSON(needs.changes.outputs.pkgs), 'pkg') || github.event_name == 'push'",
        "contains(fromJSON(github.event.inputs.pkgs), 'cerulion_core')",
        "contains(fromJSON(needs.changes.outputs.pkgs), '')",
        "contains(fromJSON(needs.changes.outputs.pkgs), 'a b')",
        "contains(fromJSON(needs.changes.outputs.pkgs), 'a') && github.event_name == 'push'",
    ] {
        assert!(
            !step_if_is_pr_blocking(bad),
            "`{bad}` can stop a step on a pull request and must disqualify it"
        );
    }

    // ---- and end to end through the classifier ----------------------------
    let doc = "on:\n  pull_request:\njobs:\n  \
               j:\n    runs-on: ubuntu-latest\n    \
               strategy:\n      matrix:\n        shard: [0, 1]\n    steps:\n      \
               - name: push only\n        if: github.event_name == 'push'\n        \
               run: cargo test -p push_only_pkg\n      \
               - name: a leg\n        if: matrix.shard == 0\n        \
               run: cargo test -p leg_pkg\n      \
               - run: cargo test -p plain_pkg\n";
    let kept = pr_blocking_jobs(doc);
    assert!(
        !kept.contains("push_only_pkg"),
        "a step behind the push allowlist runs on NO pull request and must not \
         credit coverage; got:\n{kept}"
    );
    assert!(
        kept.contains("leg_pkg"),
        "a `matrix.<key> == <literal>` leg selector runs on every pull request \
         (on one leg) and must still count; got:\n{kept}"
    );
    assert!(
        kept.contains("plain_pkg"),
        "an unconditional step must survive; got:\n{kept}"
    );

    // ---- and the selection forms, end to end -----------------------------
    // Whole-set equality rather than substring probes: a `contains` arm passes
    // on a package name that survived inside a dropped step's text.
    let selection = "on:\n  pull_request:\njobs:\n  \
                     j:\n    runs-on: ubuntu-latest\n    needs: [changes]\n    steps:\n      \
                     - name: code\n        if: needs.changes.outputs.code == 'true'\n        \
                     run: cargo test -p code_pkg\n      \
                     - name: docs\n        if: needs.changes.outputs.docs == 'true'\n        \
                     run: cargo test -p docs_pkg\n      \
                     - name: one package\n        \
                     if: contains(fromJSON(needs.changes.outputs.pkgs), 'sel_pkg')\n        \
                     run: cargo test -p sel_pkg\n      \
                     - name: wrong classifier\n        \
                     if: needs.other.outputs.code == 'true'\n        \
                     run: cargo test -p other_classifier_pkg\n      \
                     - name: inverted\n        if: needs.changes.outputs.code == 'false'\n        \
                     run: cargo test -p inverted_pkg\n      \
                     - name: or an event test\n        \
                     if: contains(fromJSON(needs.changes.outputs.pkgs), 'x') || \
                     github.event_name == 'push'\n        run: cargo test -p or_push_pkg\n  \
                     changes:\n    runs-on: ubuntu-latest\n    outputs:\n      \
                     code: ${{ steps.classify.outputs.code }}\n      \
                     docs: ${{ steps.classify.outputs.docs }}\n      \
                     pkgs: ${{ steps.classify.outputs.pkgs }}\n    steps:\n      \
                     - id: classify\n        run: |\n          \
                     echo \"code=true\" >> \"$GITHUB_OUTPUT\"\n          \
                     echo \"docs=true\" >> \"$GITHUB_OUTPUT\"\n          \
                     echo \"pkgs=[]\" >> \"$GITHUB_OUTPUT\"\n";
    let candidates: BTreeSet<String> = [
        "code_pkg",
        "docs_pkg",
        "sel_pkg",
        "other_classifier_pkg",
        "inverted_pkg",
        "or_push_pkg",
    ]
    .into_iter()
    .map(str::to_string)
    .collect();
    let sanctioned: BTreeSet<String> = ["code_pkg", "docs_pkg", "sel_pkg"]
        .into_iter()
        .map(str::to_string)
        .collect();
    let texts: BTreeMap<String, String> = [("fake.yml".to_string(), pr_blocking_jobs(selection))]
        .into_iter()
        .collect();
    let everything: BTreeSet<String> = ["sel_pkg", SELECTION_DOCS_MARKER]
        .into_iter()
        .map(str::to_string)
        .collect();
    assert_eq!(
        packages_credited_under_selection(&texts, &everything, &candidates),
        sanctioned,
        "exactly the three sanctioned selection conditions credit their \
         package; every near miss stays disqualified"
    );
}

/// The two spellings of a sanctioned condition agree: the whole string, and the
/// job plus output names the grounding check reads.
///
/// They are separate constants because the check needs the parts, and two
/// spellings of one fact drift. This is the arm that stops them.
#[test]
fn the_sanctioned_conditions_are_spelled_from_the_job_and_output_names() {
    assert_eq!(
        SELECTION_CODE_IF,
        format!("needs.{SELECTION_JOB}.outputs.{SELECTION_CODE_OUTPUT} == 'true'")
    );
    assert_eq!(
        SELECTION_DOCS_IF,
        format!("needs.{SELECTION_JOB}.outputs.{SELECTION_DOCS_OUTPUT} == 'true'")
    );
    assert_eq!(
        SELECTION_PKG_IF_OPEN,
        format!("contains(fromJSON(needs.{SELECTION_JOB}.outputs.{SELECTION_PKGS_OUTPUT}), '")
    );
}

/// A selection condition credits its step only where the value it reads EXISTS:
/// the job needs the classifier, and the classifier declares that output.
///
/// Both are silent failures in the same direction. A step gated on a
/// classifier output in a job that does not need that classifier reads the
/// empty string, so the condition is false on every event and the step runs
/// nowhere, while the coverage walk reads it as a selected step and credits
/// the package it names. An output the classifier never declares does the
/// same. Each half is asserted against the workflow that has it and the
/// workflow that does not.
#[test]
fn a_selection_condition_credits_nothing_when_its_output_is_not_grounded() {
    let body_with_steps = |needs: &str, outputs: &str, steps: &str| {
        format!(
            "on:\n  pull_request:\njobs:\n  \
             j:\n    runs-on: ubuntu-latest\n{needs}    steps:\n      \
             - if: needs.changes.outputs.code == 'true'\n        \
             run: cargo test -p code_pkg\n      \
             - if: contains(fromJSON(needs.changes.outputs.pkgs), 'sel_pkg')\n        \
             run: cargo test -p sel_pkg\n      \
             - run: cargo test -p always_pkg\n  \
             changes:\n    runs-on: ubuntu-latest\n{outputs}    steps:\n{steps}"
        )
    };
    // The classifier step every arm below wires its outputs to: a `run:` step
    // that APPENDS to the output environment file, which is the only way a run
    // step sets an output at all.
    let producing_step = "      - id: c\n        run: |\n          \
                          echo \"code=true\" >> \"$GITHUB_OUTPUT\"\n          \
                          echo \"pkgs=[]\" >> \"$GITHUB_OUTPUT\"\n";
    let body = |needs: &str, outputs: &str| body_with_steps(needs, outputs, producing_step);
    let needs_changes = "    needs: [changes]\n";
    let all_outputs = "    outputs:\n      code: ${{ steps.c.outputs.code }}\n      \
                       pkgs: ${{ steps.c.outputs.pkgs }}\n";
    let other_output = "    outputs:\n      packaging: ${{ steps.c.outputs.packaging }}\n";
    // Declared, and wired to a step this job does not carry: the value is the
    // empty string on every event, exactly like an output nobody declared.
    let unproduced = "    outputs:\n      code: ${{ steps.gone.outputs.code }}\n      \
                      pkgs: ${{ steps.gone.outputs.pkgs }}\n";

    let credited = |text: &str| -> BTreeSet<String> {
        let texts: BTreeMap<String, String> = [("fake.yml".to_string(), pr_blocking_jobs(text))]
            .into_iter()
            .collect();
        ["code_pkg", "sel_pkg", "always_pkg"]
            .into_iter()
            .filter(|pkg| workflows_name(&texts, pkg).is_some())
            .map(str::to_string)
            .collect()
    };
    let set = |names: &[&str]| -> BTreeSet<String> {
        names.iter().copied().map(str::to_string).collect()
    };

    // GROUNDED: the job needs the classifier and the classifier declares both
    // outputs, so both selection steps credit their package.
    assert_eq!(
        credited(&body(needs_changes, all_outputs)),
        set(&["always_pkg", "code_pkg", "sel_pkg"]),
        "a grounded selection condition runs on the pull requests that select \
         it and must credit the package it names"
    );

    // NOT GROUNDED, half one: the job does not need the classifier, so both
    // conditions read an empty string and neither step ever runs.
    assert_eq!(
        credited(&body("", all_outputs)),
        set(&["always_pkg"]),
        "a job that does not need the classifier reads nothing from it, so a \
         step gated on one of its outputs runs on no event and credits nothing"
    );

    // NOT GROUNDED, half two: the classifier is needed but publishes neither
    // output.
    assert_eq!(
        credited(&body(needs_changes, other_output)),
        set(&["always_pkg"]),
        "an output the classifier does not declare grounds nothing"
    );

    // NOT GROUNDED, half three: the outputs are DECLARED, and no step of the
    // classifier produces them. A declaration is a promise about a value; the
    // value still has to come from somewhere.
    assert_eq!(
        credited(&body(needs_changes, unproduced)),
        set(&["always_pkg"]),
        "an output wired to a step id the classifier does not carry is the \
         empty string on every event and grounds nothing"
    );

    // BLOCK SCALARS, both ways. The value of `code: >-` is on the line BELOW
    // the key, so reading the header alone read no expression at all and the
    // output grounded as a literal, which credited a step wired to a step id
    // the classifier does not carry.
    let folded_gone = "    outputs:\n      code: >-\n        ${{ steps.gone.outputs.code }}\n";
    let folded_there = "    outputs:\n      code: >-\n        ${{ steps.c.outputs.code }}\n";
    let literal_there = "    outputs:\n      code: |\n        ${{ steps.c.outputs.code }}\n";
    assert_eq!(
        credited(&body(needs_changes, folded_gone)),
        set(&["always_pkg"]),
        "a block-scalar output wired to a step the classifier does not carry \
         grounds nothing"
    );
    assert_eq!(
        credited(&body(needs_changes, folded_there)),
        set(&["always_pkg", "code_pkg"]),
        "a block-scalar output wired to a step the classifier DOES carry \
         grounds, and the step that reads it credits the package it names"
    );

    // PRODUCED, not merely referenced. A `run:` step that never names the
    // output environment file writes no output, whatever `id:` it carries, so
    // an output wired to it is the empty string on every event. A `uses:` step
    // is the other side: its action's outputs are not in this file to read, so
    // the declaration is credited.
    let silent_step = "      - id: c\n        run: true\n";
    let uses_step = "      - id: c\n        uses: actions/github-script@v7\n";
    assert_eq!(
        credited(&body_with_steps(needs_changes, all_outputs, silent_step)),
        set(&["always_pkg"]),
        "a step that writes no output produces none, so an output wired to it \
         grounds nothing"
    );
    assert_eq!(
        credited(&body_with_steps(needs_changes, all_outputs, uses_step)),
        set(&["always_pkg", "code_pkg", "sel_pkg"]),
        "a `uses:` step may set an output, so an output wired to it grounds"
    );
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body_with_steps(
            needs_changes,
            all_outputs,
            silent_step
        ))),
        BTreeSet::<String>::new()
    );
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body_with_steps(
            needs_changes,
            all_outputs,
            uses_step
        ))),
        set(&["code", "pkgs"])
    );

    // The output NAME, not merely the output FILE. A classifier that writes
    // `code=` and nothing else produces `code` and NOT `pkgs`: at run time
    // `pkgs` is the empty string, `fromJSON('')` fails, and the step gated on
    // the package list runs nowhere while reading as a selected step.
    let writes_code = "      - id: c\n        run: echo \"code=true\" >> \"$GITHUB_OUTPUT\"\n";
    let writes_pkgs = "      - id: c\n        run: echo \"pkgs=[]\" >> \"$GITHUB_OUTPUT\"\n";
    assert_eq!(
        credited(&body_with_steps(needs_changes, all_outputs, writes_code)),
        set(&["always_pkg", "code_pkg"]),
        "a step that writes only `code=` grounds `code` and not `pkgs`"
    );
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body_with_steps(
            needs_changes,
            all_outputs,
            writes_code
        ))),
        set(&["code"])
    );
    assert_eq!(
        credited(&body_with_steps(needs_changes, all_outputs, writes_pkgs)),
        set(&["always_pkg", "sel_pkg"]),
        "a step that writes only `pkgs=` grounds `pkgs` and not `code`"
    );
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body_with_steps(
            needs_changes,
            all_outputs,
            writes_pkgs
        ))),
        set(&["pkgs"])
    );

    // A WHOLE token, not a substring. `barcode=1` writes `barcode`, and
    // grounding `code` on it would credit a step gated on a class the
    // classifier never sets; the same name at a token boundary still grounds,
    // bare and quoted.
    let writes_barcode = "      - id: c\n        run: echo \"barcode=1\" >> \"$GITHUB_OUTPUT\"\n";
    let writes_bare = "      - id: c\n        run: echo code=1 >> \"$GITHUB_OUTPUT\"\n";
    let writes_quoted = "      - id: c\n        run: echo \"code=$value\" >> \"$GITHUB_OUTPUT\"\n";
    assert_eq!(
        credited(&body_with_steps(needs_changes, all_outputs, writes_barcode)),
        set(&["always_pkg"]),
        "`barcode=1` writes `barcode`, so it grounds no `code` output"
    );
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body_with_steps(
            needs_changes,
            all_outputs,
            writes_barcode
        ))),
        BTreeSet::<String>::new()
    );
    for step in [writes_bare, writes_quoted] {
        assert_eq!(
            credited(&body_with_steps(needs_changes, all_outputs, step)),
            set(&["always_pkg", "code_pkg"]),
            "`code=` at a token boundary grounds `code`:\n{step}"
        );
        assert_eq!(
            declared_selection_outputs(&jobs_of(&body_with_steps(
                needs_changes,
                all_outputs,
                step
            ))),
            set(&["code"])
        );
    }

    // The `run:` SCRIPT, not the step block whole. A step whose NAME mentions
    // the output file and whose script never writes into it produces nothing.
    let names_the_file =
        "      - id: c\n        name: append to GITHUB_OUTPUT\n        run: true\n";
    assert_eq!(
        credited(&body_with_steps(needs_changes, all_outputs, names_the_file)),
        set(&["always_pkg"]),
        "naming the output file outside the script writes nothing into it"
    );
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body_with_steps(
            needs_changes,
            all_outputs,
            names_the_file
        ))),
        BTreeSet::<String>::new()
    );

    // A LITERAL grounds NOTHING, either way it is spelled. `code: 'true'` reads
    // like a permanently selected class and `code: 'false'` is its twin, under
    // which every step gated on `== 'true'` runs on no event at all. Neither is
    // a value the classifier produced.
    for literal in ["'true'", "'false'", "true", "false"] {
        let outputs = format!("    outputs:\n      code: {literal}\n");
        assert_eq!(
            credited(&body(needs_changes, &outputs)),
            set(&["always_pkg"]),
            "the literal `code: {literal}` is not a produced value and grounds \
             nothing"
        );
        assert_eq!(
            declared_selection_outputs(&jobs_of(&body(needs_changes, &outputs))),
            BTreeSet::<String>::new()
        );
    }

    // PURE, not merely containing a reference. An expression carrying another
    // operand names a step this job carries AND is the empty string on every
    // pull request the operand rules out.
    for impure in [
        "${{ github.event_name == 'push' && steps.c.outputs.code }}",
        "${{ steps.c.outputs.code || 'true' }}",
        "prefix-${{ steps.c.outputs.code }}",
        "${{ steps.c.outputs.code }}-suffix",
        "${{ steps.c.outputs.code }}${{ steps.c.outputs.pkgs }}",
        "${{ needs.other.outputs.code }}",
    ] {
        let outputs = format!("    outputs:\n      code: {impure}\n");
        assert_eq!(
            credited(&body(needs_changes, &outputs)),
            set(&["always_pkg"]),
            "`code: {impure}` is not one step output and grounds nothing"
        );
        assert_eq!(
            declared_selection_outputs(&jobs_of(&body(needs_changes, &outputs))),
            BTreeSet::<String>::new()
        );
    }
    // And the pure form, with and without the whitespace YAML folds away.
    for pure in ["${{ steps.c.outputs.code }}", "${{steps.c.outputs.code}}"] {
        let outputs = format!("    outputs:\n      code: {pure}\n");
        assert_eq!(
            credited(&body(needs_changes, &outputs)),
            set(&["always_pkg", "code_pkg"]),
            "`code: {pure}` is one produced step output and grounds"
        );
        assert_eq!(
            declared_selection_outputs(&jobs_of(&body(needs_changes, &outputs))),
            set(&["code"])
        );
    }

    // And the declaration reader itself, every way.
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body(needs_changes, all_outputs))),
        set(&["code", "pkgs"])
    );
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body(needs_changes, other_output))),
        BTreeSet::<String>::new()
    );
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body(needs_changes, unproduced))),
        BTreeSet::<String>::new()
    );
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body(needs_changes, folded_gone))),
        BTreeSet::<String>::new()
    );
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body(needs_changes, folded_there))),
        set(&["code"])
    );
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body(needs_changes, literal_there))),
        set(&["code"])
    );
    // A literal value grounds NOTHING: a classifier did not produce it, and
    // the walk cannot tell `'true'` from `'false'` by who wrote it.
    assert_eq!(
        declared_selection_outputs(&jobs_of(&body(
            needs_changes,
            "    outputs:\n      code: 'true'\n"
        ))),
        BTreeSet::<String>::new()
    );
    // A comment between two output keys does not end the mapping. Comments are
    // blanked to empty strings before this walk reads a workflow, so a blank
    // line is what it sees, and reading one as the end truncated the map.
    let commented = body(needs_changes, all_outputs).replace(
        "      pkgs: ",
        "      # the package list the per-package condition reads\n      pkgs: ",
    );
    let stripped: String = commented
        .lines()
        .map(strip_yaml_comment)
        .collect::<Vec<_>>()
        .join("\n");
    assert_eq!(
        declared_selection_outputs(&jobs_of(&stripped)),
        set(&["code", "pkgs"]),
        "a comment between two output keys must not truncate the map"
    );
}

/// Every `if:` [`retain_steps`] read out of one job block, in file order.
fn conditions_read(job: &str) -> Vec<String> {
    let seen = std::cell::RefCell::new(Vec::new());
    retain_steps(job, |cond| {
        seen.borrow_mut().push(cond.to_string());
        true
    });
    seen.into_inner()
}

/// One job with one conditional step, so an arm can vary the `if:` alone.
fn job_with_condition(if_block: &str) -> String {
    format!(
        "  j:\n    runs-on: ubuntu-latest\n    strategy:\n      matrix:\n        \
         shard: [0, 1]\n    steps:\n      \
         - name: conditional\n        {if_block}        run: cargo test -p gated_pkg\n      \
         - run: cargo test -p plain_pkg\n"
    )
}

/// The step condition is read in every YAML scalar form, and every form means
/// the same expression.
///
/// The oracle is the expression itself, typed once: each form below spells
/// `github.event_name == 'push'` in a different YAML shape, and the reader has
/// to return that one string from all of them. Reading a form as the EMPTY
/// condition is what this arm exists for: an empty condition cannot stop a
/// step, so the push-allowlisted step would credit coverage.
#[test]
fn a_step_condition_is_read_in_every_yaml_scalar_form() {
    let gate = "github.event_name == 'push'";
    let forms: [(&str, String); 9] = [
        ("plain, on the key's own line", format!("if: {gate}\n")),
        ("double quoted", format!("if: \"{gate}\"\n")),
        (
            "single quoted, inner quotes doubled",
            String::from("if: 'github.event_name == ''push'''\n"),
        ),
        ("folded, chomped", format!("if: >-\n          {gate}\n")),
        ("folded", format!("if: >\n          {gate}\n")),
        ("literal", format!("if: |\n          {gate}\n")),
        ("literal, chomped", format!("if: |-\n          {gate}\n")),
        (
            "plain, written on the following line",
            format!("if:\n          {gate}\n"),
        ),
        (
            "plain, continued on the following line",
            String::from("if: github.event_name\n          == 'push'\n"),
        ),
    ];

    for (name, if_block) in &forms {
        let job = job_with_condition(if_block);
        assert_eq!(
            conditions_read(&job),
            vec![gate.to_string()],
            "the `{name}` form must read as the expression it spells"
        );
        let kept = pr_blocking_jobs(&format!("on:\n  pull_request:\njobs:\n{job}"));
        assert!(
            !kept.contains("gated_pkg"),
            "the `{name}` form gates the step off pull requests and must not \
             credit coverage; got:\n{kept}"
        );
        assert!(
            kept.contains("plain_pkg"),
            "the step after a `{name}` condition must survive; got:\n{kept}"
        );
    }

    // The other side of each form: a condition that cannot stop the step keeps
    // it, so no form disqualifies a step by its shape alone.
    for (name, if_block) in [
        ("plain", String::from("if: always()\n")),
        ("double quoted", String::from("if: \"always()\"\n")),
        ("single quoted", String::from("if: 'always()'\n")),
        (
            "folded, chomped",
            String::from("if: >-\n          always()\n"),
        ),
        (
            "literal, chomped",
            String::from("if: |-\n          always()\n"),
        ),
        (
            "on the following line",
            String::from("if:\n          always()\n"),
        ),
    ] {
        let job = job_with_condition(&if_block);
        assert_eq!(conditions_read(&job), vec![String::from("always()")]);
        let kept = pr_blocking_jobs(&format!("on:\n  pull_request:\njobs:\n{job}"));
        assert!(
            kept.contains("gated_pkg"),
            "`always()` in the `{name}` form cannot stop the step; got:\n{kept}"
        );
    }

    // A step's own keys END the scalar: the `run:` below sits at the `if:`
    // column, so it is never swallowed as a continuation line.
    assert_eq!(
        conditions_read(&job_with_condition("if: matrix.shard == 0\n")),
        vec![String::from("matrix.shard == 0")]
    );
}

/// A condition form the reader cannot classify FAILS the walk rather than
/// reading as no condition at all.
#[test]
#[should_panic(expected = "an anchor, alias or tag as an `if:`")]
fn an_unreadable_step_condition_fails_the_walk() {
    retain_steps(&job_with_condition("if: *gate\n"), |_| true);
}

/// An output VALUE the reader cannot classify fails the walk too, rather than
/// grounding the output on a form nobody read.
///
/// The other side is every arm of
/// `a_selection_condition_credits_nothing_when_its_output_is_not_grounded`,
/// where a readable value grounds, or does not, on its own merits.
#[test]
#[should_panic(expected = "an anchor, alias or tag as an `code:`")]
fn an_unreadable_selection_output_fails_the_walk() {
    let text = "on:\n  pull_request:\njobs:\n  changes:\n    runs-on: ubuntu-latest\n    \
                outputs:\n      code: *anchor\n    steps:\n      - id: c\n        run: true\n";
    declared_selection_outputs(&jobs_of(text));
}

/// An EMPTY condition is unreadable, and a non-empty quoted one still reads.
///
/// GitHub evaluates an empty `if:` as FALSE and skips the step on every event,
/// so reading `if: ''` as the empty string reads a step that never runs as an
/// unconditional one and credits its package. Both empty forms, and both
/// non-empty ones.
#[test]
fn an_empty_condition_is_unreadable_and_a_non_empty_one_reads() {
    let read = |head: &str| read_scalar_value("if", head, &[head], 0, 8);
    let empty = String::from(
        "an empty `if:` value: `if: ''` and `if: \"\"` are valid YAML and mean \
         NOTHING at all",
    );
    assert_eq!(read("''"), Err(empty.clone()));
    assert_eq!(read("\"\""), Err(empty));
    assert_eq!(read("'always()'"), Ok(String::from("always()")));
    assert_eq!(read("\"always()\""), Ok(String::from("always()")));
}

/// An empty single-quoted condition fails the WALK, not only the reader.
#[test]
#[should_panic(expected = "an empty `if:` value")]
fn an_empty_single_quoted_condition_fails_the_walk() {
    retain_steps(&job_with_condition("if: ''\n"), |_| true);
}

/// And the double-quoted spelling of the same empty condition.
#[test]
#[should_panic(expected = "an empty `if:` value")]
fn an_empty_double_quoted_condition_fails_the_walk() {
    retain_steps(&job_with_condition("if: \"\"\n"), |_| true);
}

/// A leg selector counts only against a leg the step's OWN job declares.
///
/// Both halves: a selector naming a key the matrix does not carry, and one
/// naming a value that is not a leg of that key, select NO leg: the step runs
/// on no pull request at all while reading as an ordinary leg-selected step.
#[test]
fn a_leg_selector_counts_only_against_a_leg_the_job_declares() {
    let legs = |rows: &[(&str, &[&str])]| -> MatrixLegs {
        rows.iter()
            .map(|(key, legs)| {
                (
                    (*key).to_string(),
                    legs.iter().map(|leg| (*leg).to_string()).collect(),
                )
            })
            .collect()
    };
    let shards = legs(&[("shard", &["0", "1", "2", "3"])]);
    let one_shard = legs(&[("shard", &["0"])]);
    let lanes = legs(&[("lane", &["viz", "vizd"])]);
    let grounded = every_selection_output();

    for (cond, matrix) in [
        ("matrix.shard == 0", &shards),
        ("matrix.shard == 3", &shards),
        ("matrix.shard != 0", &shards),
        ("matrix.lane == 'viz'", &lanes),
        ("matrix.lane != 'viz'", &lanes),
    ] {
        assert!(
            step_if_is_pr_blocking_grounded(cond, &grounded, matrix),
            "`{cond}` selects a leg this job declares and must count"
        );
    }
    for (cond, matrix) in [
        // A leg the matrix does not carry: shard 4 of a four-way split.
        ("matrix.shard == 4", &shards),
        // A key the matrix does not declare at all, in both directions.
        ("matrix.lane == 'viz'", &shards),
        ("matrix.shard == 0", &lanes),
        // The only leg there is, excluded: the step runs on nothing.
        ("matrix.shard != 0", &one_shard),
    ] {
        assert!(
            !step_if_is_pr_blocking_grounded(cond, &grounded, matrix),
            "`{cond}` selects no leg of this job's matrix and must credit nothing"
        );
    }

    // The matrix reader itself: both list forms, no matrix at all, and the one
    // shape it refuses to model.
    assert_eq!(
        job_matrix_legs(
            "  j:\n    strategy:\n      matrix:\n        shard: [0, 1]\n        \
             lane: [viz, vizd]\n    steps:\n      - run: true\n"
        ),
        legs(&[("lane", &["viz", "vizd"]), ("shard", &["0", "1"])])
    );
    assert_eq!(
        job_matrix_legs(
            "  j:\n    strategy:\n      matrix:\n        shard:\n          - 0\n          \
             - 1\n    steps:\n      - run: true\n"
        ),
        legs(&[("shard", &["0", "1"])]),
        "a block-style leg list must be seen"
    );
    assert_eq!(
        job_matrix_legs(
            "  j:\n    strategy:\n      matrix:\n        shard: [0, 1]\n        \
             include:\n          - shard: 9\n    steps:\n      - run: true\n"
        ),
        MatrixLegs::new(),
        "`include:` adds legs this reader does not model, so it reports none and \
         every leg selector in that job fails closed"
    );
    assert_eq!(
        job_matrix_legs("  j:\n    steps:\n      - run: true\n"),
        MatrixLegs::new()
    );

    // End to end: the same step, once on a leg the job has and once on a leg it
    // does not.
    let doc = |leg: &str| {
        format!(
            "on:\n  pull_request:\njobs:\n  j:\n    runs-on: ubuntu-latest\n    \
             strategy:\n      matrix:\n        shard: [0, 1]\n    steps:\n      \
             - name: leg\n        if: matrix.shard == {leg}\n        \
             run: cargo test -p leg_pkg\n"
        )
    };
    assert!(
        pr_blocking_jobs(&doc("1")).contains("leg_pkg"),
        "a step selected onto a declared leg runs on every pull request, on that leg"
    );
    assert!(
        !pr_blocking_jobs(&doc("2")).contains("leg_pkg"),
        "a step selected onto a leg the matrix does not carry runs on no pull \
         request and must not credit coverage"
    );
}

/// `cargo test -p a -p b` covers BOTH — the module docs said so and the
/// matcher only ever read the first.
#[test]
fn every_package_named_in_one_step_is_credited() {
    let texts: BTreeMap<String, String> = [(
        "fake.yml".to_string(),
        "      - run: cargo test -p alpha -p beta --package gamma --package=delta\n".to_string(),
    )]
    .into_iter()
    .collect();

    for pkg in ["alpha", "beta", "gamma", "delta"] {
        assert!(
            workflows_name(&texts, pkg).is_some(),
            "`{pkg}` is named on the step and must be credited"
        );
    }
    assert!(
        workflows_name(&texts, "epsilon").is_none(),
        "a package the step does not name must not be credited"
    );
    // The whole-token rule still holds across the multi-`-p` form.
    assert!(workflows_name(&texts, "alph").is_none());
    assert!(workflows_name(&texts, "alphab").is_none());
    // `cargo build -p x` is not a test step.
    let build_only: BTreeMap<String, String> = [(
        "fake.yml".to_string(),
        "      - run: cargo build -p buildonly\n".to_string(),
    )]
    .into_iter()
    .collect();
    assert!(workflows_name(&build_only, "buildonly").is_none());
}

/// The `shard:` matrix is read in BOTH YAML list forms, and per JOB.
#[test]
fn the_shard_matrix_is_read_in_both_yaml_forms_and_per_job() {
    let inline = "  j:\n    strategy:\n      matrix:\n        shard: [0, 1, 2, 3]\n";
    assert_eq!(
        shard_matrices(inline),
        vec![vec![
            "0".to_string(),
            "1".to_string(),
            "2".to_string(),
            "3".to_string()
        ]]
    );

    let block = "  j:\n    strategy:\n      matrix:\n        shard:\n          - 0\n          \
                 - 1\n          - 2\n          - 3\n    steps:\n      - run: x\n";
    assert_eq!(
        shard_matrices(block),
        vec![vec![
            "0".to_string(),
            "1".to_string(),
            "2".to_string(),
            "3".to_string()
        ]],
        "a block-style `shard:` list must be seen — an invisible matrix silently \
         disables the count cross-check"
    );

    // Per-job: one job's matrix must not vouch for a sibling that has none.
    let two_jobs = "on:\n  pull_request:\njobs:\n  \
                    a:\n    strategy:\n      matrix:\n        shard: [0, 1]\n    \
                    steps:\n      - run: ./tools/scripts/ci_test_shard.sh pkg ${{ matrix.shard }} 2\n  \
                    b:\n    steps:\n      - run: echo hi\n";
    let jobs = jobs_of(two_jobs);
    assert_eq!(jobs.len(), 2, "both jobs must be split out; got {jobs:?}");
    assert_eq!(jobs[0].0, "a");
    assert_eq!(jobs[1].0, "b");
    assert!(!shard_matrices(&jobs[0].1).is_empty());
    assert!(
        shard_matrices(&jobs[1].1).is_empty(),
        "job `b` declares no matrix and must not inherit job `a`'s"
    );

    // ---- and end to end on the REAL tree ---------------------------------
    // The nightly lanes name these packages; ci.yml's PR-blocking jobs must be
    // where the coverage actually comes from. If this ever reports a nightly
    // file, the trigger classifier has widened.
    let texts = pr_blocking_workflow_texts();
    for f in texts.keys() {
        assert!(
            !f.starts_with("nightly-"),
            "a nightly lane was classified as PR-blocking: {f}"
        );
    }
    assert!(
        texts.contains_key("ci.yml"),
        "ci.yml must be classified as PR-blocking; got {:?}",
        texts.keys().collect::<Vec<_>>()
    );
    // `latency_threshold_test` runs ONLY in `test-latency`, which is behind the
    // push allowlist — so its name must NOT appear in the PR-blocking text.
    // This is the real-tree twin of the `allowlisted_pkg` arm above.
    assert!(
        !texts["ci.yml"].contains("latency_threshold_test"),
        "the push-allowlisted `test-latency` job was not dropped from the \
         PR-blocking view"
    );
}

/// Every package with tests must still be run by a PR-blocking step on the
/// pull request that touches THAT PACKAGE ALONE.
///
/// Naming is the property the gate above asserts; running is this one, and the
/// two come apart the moment a step carries a selection condition. A step that
/// runs `cerulion_bag` gated on `cerulion_core` being selected is named in
/// ci.yml, passes the gate above, and never runs on a bag-only change, which
/// is the only change whose tests it was there to protect.
///
/// On a ci.yml with no selection condition every step survives every selected
/// set, so this passes without asserting anything about gating; that is the
/// point. It is the guard standing before the first condition is added, and
/// its own non-vacuity is proven on synthetic input by
/// `a_step_gated_on_another_packages_selection_loses_its_own`.
#[test]
fn every_package_with_tests_is_credited_when_it_alone_is_selected() {
    let texts = pr_blocking_workflow_texts();
    let exempt: BTreeSet<&str> = EXEMPT_PACKAGES.iter().map(|(pkg, _)| *pkg).collect();
    let demanded: BTreeSet<String> = packages_with_tests()
        .into_keys()
        .filter(|pkg| !exempt.contains(pkg.as_str()))
        .collect();

    // The doc marker shares the set with package names, so a workspace member
    // of that name would make it mean two things at once and quietly credit
    // that package on every doc-only change.
    assert!(
        !demanded.contains(SELECTION_DOCS_MARKER),
        "a workspace package is named `{SELECTION_DOCS_MARKER}`, which is the \
         marker this evaluator puts in the same set as package names. Rename \
         the marker before the two meanings merge."
    );

    let lost = packages_lost_to_their_own_selection(&texts, &demanded);
    assert!(
        lost.is_empty(),
        "these packages are named in a PR-blocking step, but every step that \
         names them is gated on a selection that a pull request touching only \
         that package does NOT produce, so their tests skip on exactly the \
         change that needs them:\n  {}\n\
         FIX: gate the step on \
         `contains(fromJSON(needs.changes.outputs.pkgs), '<package>')` naming \
         the package it runs, on `needs.changes.outputs.code == 'true'`, or \
         leave it ungated.",
        lost.join("\n  ")
    );

    // ---- anti-tautology ---------------------------------------------------
    // An evaluator that dropped every step would report nothing lost only if
    // the matcher stopped matching too, but one that KEPT every step whatever
    // the set says is silently blind, and today's unconditional ci.yml cannot
    // tell the two apart. So require the evaluator to reproduce the ungated
    // coverage set when everything is selected, and require the set to be the
    // real workspace.
    let everything: BTreeSet<String> = demanded
        .iter()
        .cloned()
        .chain([SELECTION_DOCS_MARKER.to_string()])
        .collect();
    assert_eq!(
        packages_credited_under_selection(&texts, &everything, &demanded),
        demanded,
        "with every package selected the evaluator must credit exactly what the \
         ungated walk credits"
    );
    assert!(
        demanded.len() >= 20,
        "only {} package(s) were evaluated under selection: the member walk is \
         not reaching the tree and this arm would be vacuous",
        demanded.len()
    );

    // REAL GATES, not a hypothetical. A workflow that carries no selection
    // condition runs every step under every set, so this arm cannot fail on one
    // whatever the evaluator does. It holds a live population, and a workflow
    // that lost every gate has to say so here rather than go quietly green.
    let gated: usize = texts
        .values()
        .map(|text| {
            text.lines()
                .filter(|l| l.contains(SELECTION_PKG_IF_OPEN))
                .count()
        })
        .sum();
    assert!(
        gated >= 20,
        "only {gated} per-package selection condition(s) survive into the \
         PR-blocking text: either the gates were removed or the step filter is \
         dropping them, and this arm is then evaluating a workflow that runs \
         everything under every selection"
    );
}

/// The totality arm is not vacuous: a package whose only step is gated on a
/// DIFFERENT package's selection is reported lost, by name.
///
/// This is the mutant the arm exists to kill: the reviewer who moves a step
/// under the neighbouring package's condition because the two crates are
/// usually touched together.
#[test]
fn a_step_gated_on_another_packages_selection_loses_its_own() {
    let texts: BTreeMap<String, String> = [(
        "fake.yml".to_string(),
        "  j:\n    runs-on: ubuntu-latest\n    steps:\n      \
         - name: alpha\n        \
         if: contains(fromJSON(needs.changes.outputs.pkgs), 'alpha')\n        \
         run: cargo test -p alpha\n      \
         - name: beta\n        \
         if: contains(fromJSON(needs.changes.outputs.pkgs), 'alpha')\n        \
         run: cargo test -p beta\n"
            .to_string(),
    )]
    .into_iter()
    .collect();
    let packages: BTreeSet<String> = ["alpha", "beta"].into_iter().map(str::to_string).collect();

    assert_eq!(
        packages_lost_to_their_own_selection(&texts, &packages),
        vec!["beta".to_string()],
        "`beta` runs only when `alpha` is selected, so a pull request touching \
         `beta` alone never runs its tests, and the arm must name it"
    );

    // Both sides of the set: `beta` IS credited when `alpha` rides along, and
    // `alpha` is not credited when it is absent.
    assert_eq!(
        packages_credited_under_selection(&texts, &packages, &packages),
        packages,
        "selecting `alpha` runs both steps"
    );
    let only_beta: BTreeSet<String> = ["beta".to_string()].into_iter().collect();
    assert_eq!(
        packages_credited_under_selection(&texts, &only_beta, &packages),
        BTreeSet::<String>::new(),
        "selecting `beta` alone runs neither step, because both read `alpha`"
    );
}

/// The `code` and `docs` selection markers, both sides, on synthetic input.
///
/// `code` follows the set being NON-EMPTY and `docs` follows the set holding
/// the `docs` marker, so a set that holds nothing but the marker satisfies
/// both. That is the permissive corner of the model and it is asserted here
/// rather than left to a reader to infer.
#[test]
fn the_selection_evaluator_reads_the_code_and_docs_markers() {
    let texts: BTreeMap<String, String> = [(
        "fake.yml".to_string(),
        "  j:\n    runs-on: ubuntu-latest\n    steps:\n      \
         - if: needs.changes.outputs.code == 'true'\n        \
         run: cargo test -p code_pkg\n      \
         - if: needs.changes.outputs.docs == 'true'\n        \
         run: cargo test -p docs_pkg\n      \
         - run: cargo test -p always_pkg\n"
            .to_string(),
    )]
    .into_iter()
    .collect();
    let packages: BTreeSet<String> = ["code_pkg", "docs_pkg", "always_pkg"]
        .into_iter()
        .map(str::to_string)
        .collect();
    let set = |names: &[&str]| -> BTreeSet<String> {
        names.iter().copied().map(str::to_string).collect()
    };

    assert_eq!(
        packages_credited_under_selection(&texts, &set(&[]), &packages),
        set(&["always_pkg"]),
        "an empty selection runs neither the code step nor the doc step"
    );
    assert_eq!(
        packages_credited_under_selection(&texts, &set(&["cerulion_core"]), &packages),
        set(&["always_pkg", "code_pkg"]),
        "a selected package makes the set non-empty, which is `code` and is not \
         `docs`"
    );
    assert_eq!(
        packages_credited_under_selection(&texts, &set(&[SELECTION_DOCS_MARKER]), &packages),
        set(&["always_pkg", "code_pkg", "docs_pkg"]),
        "the `docs` marker is `docs`, and a set holding it is non-empty"
    );
}

/// The per-package condition is read as a whole form around a package name,
/// not by substring, the same boundary rule the package matcher lives by.
#[test]
fn only_the_exact_per_package_selection_form_names_a_package() {
    assert_eq!(
        selection_condition_package(
            "contains(fromJSON(needs.changes.outputs.pkgs), 'cerulion-wire')"
        ),
        Some("cerulion-wire")
    );
    assert_eq!(
        selection_condition_package("  contains(fromJSON(needs.changes.outputs.pkgs), 'go2_tf')  "),
        Some("go2_tf")
    );
    for not_sanctioned in [
        "contains(fromJSON(needs.other.outputs.pkgs), 'a')",
        "contains(fromJSON(needs.changes.outputs.pkgs), \"a\")",
        "contains(fromJSON(needs.changes.outputs.pkgs), '')",
        "contains(fromJSON(needs.changes.outputs.pkgs), 'a.b')",
        "!contains(fromJSON(needs.changes.outputs.pkgs), 'a')",
        "needs.changes.outputs.pkgs",
    ] {
        assert_eq!(
            selection_condition_package(not_sanctioned),
            None,
            "`{not_sanctioned}` is not the sanctioned per-package form"
        );
    }
}

// ---------------------------------------------------------------------------
// THE OFF SWITCH, AND THE SHAPE OF A STEP-LEVEL SELECTION GATE
// ---------------------------------------------------------------------------

/// The repository variable that stops the selection with no pull request.
const SELECTION_SWITCH: &str = "CI_SELECTION";

/// The one line `ci.yml` declares it on, default included.
///
/// Held whole rather than by prefix: the DEFAULT is the half that decides what
/// a repository with no variable does, and a declaration without it reads as an
/// empty switch, which the classifier treats as "not `on`" and answers by
/// selecting everything. That is the safe direction and it is also a silent
/// loss of the whole feature, so the line is pinned.
const SELECTION_SWITCH_DECLARATION: &str = "  CI_SELECTION: ${{ vars.CI_SELECTION || 'on' }}";

/// The job whose script has to READ the switch, and the step that reads it.
const SELECTION_SWITCH_READER_STEP: &str = "classify";

/// The prefix every selection skip line carries.
const SELECTION_MARKER_PREFIX: &str = "selection:";

/// The whole opening of the line a gated step's skip branch prints.
const SELECTION_SKIPPED_PREFIX: &str = "selection: skipped ";

/// Why this condition is not a legal step-level selection gate, if it is not.
///
/// THE RULE. The switch feeds the CLASSIFIER, and the classifier's three
/// outputs are the only thing a step may be gated on. A step that read the
/// variable itself would decide from a value the `changes` job never saw: the
/// classifier could be answering `pkgs` for a narrow selection while the step
/// reads `off` and runs, or the other way round, and the two would drift the
/// first time somebody changed one of them. A step gated on some OTHER output
/// of the classifier job is the same fault spelled differently: the walk has
/// never seen that output's rules, and `step_if_is_pr_blocking_grounded`
/// already refuses to credit it, so it would run on no event at all.
///
/// Returns `None` for a condition that is not a selection gate (a leg selector,
/// `always()`, an ordinary event test): those are the coverage walk's business,
/// not this rule's.
fn selection_gate_violation(cond: &str) -> Option<String> {
    if cond.contains(SELECTION_SWITCH) {
        return Some(format!(
            "reads the `{SELECTION_SWITCH}` switch directly. The switch feeds \
             the `{SELECTION_JOB}` job; a step reads the classifier's output, \
             never the variable"
        ));
    }
    let needle = format!("needs.{SELECTION_JOB}.outputs.");
    let mut from = 0usize;
    while let Some(offset) = cond[from..].find(&needle) {
        let at = from + offset + needle.len();
        from = at;
        let name: String = cond[at..]
            .chars()
            .take_while(|c| is_name_char(*c))
            .collect();
        if ![
            SELECTION_CODE_OUTPUT,
            SELECTION_DOCS_OUTPUT,
            SELECTION_PKGS_OUTPUT,
        ]
        .contains(&name.as_str())
        {
            return Some(format!(
                "reads `{needle}{name}`, which is not one of the three outputs \
                 the switch feeds (`{SELECTION_CODE_OUTPUT}`, \
                 `{SELECTION_DOCS_OUTPUT}`, `{SELECTION_PKGS_OUTPUT}`)"
            ));
        }
    }
    None
}

/// Every workflow's text with comments stripped, keyed by file name.
///
/// NOT [`pr_blocking_workflow_texts`]: that one DROPS the gated steps, which is
/// exactly the population the rules below are about.
fn workflow_texts() -> BTreeMap<String, String> {
    let dir = repo_root().join(".github/workflows");
    let mut out = BTreeMap::new();
    let entries =
        std::fs::read_dir(&dir).unwrap_or_else(|e| panic!("cannot read {}: {e}", dir.display()));
    for entry in entries.flatten() {
        let path = entry.path();
        let name = entry.file_name().to_string_lossy().into_owned();
        if !(name.ends_with(".yml") || name.ends_with(".yaml")) {
            continue;
        }
        let raw = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
        out.insert(
            name,
            raw.lines()
                .map(strip_yaml_comment)
                .collect::<Vec<_>>()
                .join("\n"),
        );
    }
    assert!(
        out.len() >= 5,
        "the workflow walk found only {} file(s) under {}: it is not reaching \
         the workflows",
        out.len(),
        dir.display()
    );
    out
}

/// The `if:` value of one step block, read in every scalar form.
///
/// `Err` for a form [`read_scalar_form`] cannot classify, which every caller
/// turns into a failure: a condition this walk cannot read is not one it may
/// skip past.
fn step_if_of(block: &[&str]) -> Result<Option<String>, String> {
    let Some(first) = block.first() else {
        return Ok(None);
    };
    let step_indent = indent_of(first);
    for (i, line) in block.iter().enumerate() {
        let trimmed = line.trim_start();
        let rest = if indent_of(line) == step_indent + 2 {
            trimmed.strip_prefix("if:")
        } else if indent_of(line) == step_indent && trimmed.starts_with("- ") {
            trimmed
                .strip_prefix("- ")
                .and_then(|r| r.strip_prefix("if:"))
        } else {
            None
        };
        if let Some(rest) = rest {
            return read_scalar_value("if", rest, block, i, step_indent + 2).map(Some);
        }
    }
    Ok(None)
}

/// The `name:` of one step block, or `None`.
fn step_name_of(block: &[&str]) -> Option<String> {
    let step_indent = indent_of(block.first()?);
    for line in block {
        let trimmed = line.trim_start();
        let rest = if indent_of(line) == step_indent && trimmed.starts_with("- ") {
            trimmed
                .strip_prefix("- ")
                .and_then(|r| r.strip_prefix("name:"))
        } else if indent_of(line) == step_indent + 2 {
            trimmed.strip_prefix("name:")
        } else {
            None
        };
        if let Some(rest) = rest {
            return Some(rest.trim().to_string());
        }
    }
    None
}

/// The exact condition a gated step's SKIP-BRANCH marker carries: the negation
/// of the per-package gate, wrapped so YAML reads the `!` as an expression
/// rather than as a tag.
fn selection_marker_condition(package: &str) -> String {
    format!("${{{{ !{SELECTION_PKG_IF_OPEN}{package}{SELECTION_PKG_IF_CLOSE} }}}}")
}

/// The one line that marker prints, up to the selection it names.
fn selection_marker_line(package: &str) -> String {
    format!("{SELECTION_SKIPPED_PREFIX}{package} (selected: ")
}

/// Every package a step of this job is gated on, and the packages its marker
/// steps cover.
fn selection_gates_and_markers(job: &str) -> (BTreeSet<String>, BTreeSet<String>, Vec<String>) {
    let mut gated: BTreeSet<String> = BTreeSet::new();
    let mut marked: BTreeSet<String> = BTreeSet::new();
    let mut complaints: Vec<String> = Vec::new();
    for block in step_blocks(job) {
        let cond = match step_if_of(&block) {
            Ok(Some(cond)) => cond,
            Ok(None) => continue,
            Err(why) => {
                complaints.push(format!("  {why}"));
                continue;
            }
        };
        for term in cond.split("&&") {
            if let Some(package) = selection_condition_package(term) {
                gated.insert(package.to_string());
            }
        }
        // A marker step is recognised by its CONDITION, which is the exact
        // negation of one gate, and then held to its print. Recognised by the
        // condition rather than by the name, so a step called "selection
        // marker" that gates on something else covers nothing.
        for package in marked_candidates(&cond) {
            if cond.trim() != selection_marker_condition(&package) {
                continue;
            }
            let script = run_script_of(&block).unwrap_or_default();
            let printed: Vec<&str> = script
                .lines()
                .filter(|l| l.contains(SELECTION_MARKER_PREFIX))
                .collect();
            let name = step_name_of(&block).unwrap_or_default();
            if printed.len() != 1 {
                complaints.push(format!(
                    "  `{name}` is the skip branch of the `{package}` gate and \
                     prints {} line(s) carrying `{SELECTION_MARKER_PREFIX}`; \
                     exactly one is the rule",
                    printed.len()
                ));
                continue;
            }
            if !printed[0].contains(&selection_marker_line(&package)) {
                complaints.push(format!(
                    "  `{name}` prints `{}` rather than a line opening \
                     `{}`",
                    printed[0].trim(),
                    selection_marker_line(&package)
                ));
                continue;
            }
            marked.insert(package);
        }
    }
    (gated, marked, complaints)
}

/// The packages a marker CONDITION could be about: the negated per-package form
/// names exactly one.
fn marked_candidates(cond: &str) -> Vec<String> {
    let cond = cond.trim();
    let Some(inner) = cond.strip_prefix("${{").and_then(|c| c.strip_suffix("}}")) else {
        return Vec::new();
    };
    let Some(term) = inner.trim().strip_prefix('!') else {
        return Vec::new();
    };
    selection_condition_package(term.trim())
        .map(|p| vec![p.to_string()])
        .unwrap_or_default()
}

/// The switch is declared once, at workflow level, WITH its default, and it
/// reaches the classifier.
#[test]
fn the_selection_switch_is_declared_once_and_read_by_the_classifier() {
    let texts = workflow_texts();
    let ci = texts
        .get("ci.yml")
        .unwrap_or_else(|| panic!("ci.yml is not among the workflows"));

    let declarations = ci
        .lines()
        .filter(|l| l.trim_end() == SELECTION_SWITCH_DECLARATION)
        .count();
    assert_eq!(
        declarations, 1,
        "ci.yml declares `{SELECTION_SWITCH}` on {declarations} line(s) reading \
         exactly `{SELECTION_SWITCH_DECLARATION}`; it is declared ONCE, at \
         workflow level, and the default in the expression is what a repository \
         with no variable set gets"
    );

    // The switch reaches the CLASSIFIER, and the classifier alone. A switch
    // nothing reads is a switch that stops nothing.
    let jobs = jobs_of(ci);
    let (_, block) = jobs
        .iter()
        .find(|(name, _)| name == SELECTION_JOB)
        .unwrap_or_else(|| panic!("ci.yml carries no `{SELECTION_JOB}` job"));
    let reader = step_blocks(block)
        .into_iter()
        .find(|b| step_id_of(b).as_deref() == Some(SELECTION_SWITCH_READER_STEP))
        .unwrap_or_else(|| {
            panic!(
                "the `{SELECTION_JOB}` job carries no step with id `{SELECTION_SWITCH_READER_STEP}`"
            )
        });
    let script = run_script_of(&reader).unwrap_or_default();
    assert!(
        script.contains(SELECTION_SWITCH),
        "the `{SELECTION_SWITCH_READER_STEP}` step of the `{SELECTION_JOB}` job \
         never names `{SELECTION_SWITCH}`: the switch would be declared and read \
         by nothing, and turning it off would change no run"
    );
    assert!(
        script.contains(SELECTION_MARKER_PREFIX),
        "the `{SELECTION_SWITCH_READER_STEP}` step never prints a \
         `{SELECTION_MARKER_PREFIX}` line, so a reader of the log cannot see \
         which way the switch was set"
    );
}

/// No step in any workflow is gated on the switch, or on an output the switch
/// does not feed.
#[test]
fn a_step_selection_gate_reads_only_an_output_the_switch_feeds() {
    let mut complaints: Vec<String> = Vec::new();
    let mut steps_read = 0usize;
    for (file, text) in workflow_texts() {
        for (job, block) in jobs_of(&text) {
            for step in step_blocks(&block) {
                steps_read += 1;
                let cond = match step_if_of(&step) {
                    Ok(Some(cond)) => cond,
                    Ok(None) => continue,
                    Err(why) => {
                        complaints.push(format!("  {file} / {job}: {why}"));
                        continue;
                    }
                };
                if let Some(why) = selection_gate_violation(&cond) {
                    let name = step_name_of(&step).unwrap_or_else(|| "<unnamed>".to_string());
                    complaints.push(format!("  {file} / {job} / `{name}`: {why}"));
                }
            }
        }
    }
    assert!(
        steps_read >= 100,
        "the step walk read only {steps_read} step(s): it is not reaching the \
         workflows and this rule would be vacuous"
    );
    assert!(
        complaints.is_empty(),
        "these step conditions are not legal selection gates:\n{}",
        complaints.join("\n")
    );
}

/// The rule itself, both sides, on hand-written conditions.
#[test]
fn the_selection_gate_rule_names_the_switch_and_the_wrong_output() {
    // Legal: the three sanctioned forms, and conditions that are no selection
    // gate at all.
    for allowed in [
        SELECTION_CODE_IF,
        SELECTION_DOCS_IF,
        "contains(fromJSON(needs.changes.outputs.pkgs), 'cerulion_bag')",
        "matrix.shard == 3 && contains(fromJSON(needs.changes.outputs.pkgs), 'go2_tf')",
        "matrix.shard == 0",
        "always()",
        "github.event_name == 'push'",
    ] {
        assert_eq!(
            selection_gate_violation(allowed),
            None,
            "`{allowed}` is a legal step condition"
        );
    }
    // The switch, in every spelling a step could reach it by.
    for refused in [
        "env.CI_SELECTION != 'off'",
        "vars.CI_SELECTION == 'on'",
        "matrix.shard == 3 && env.CI_SELECTION != 'off'",
    ] {
        let why = selection_gate_violation(refused)
            .unwrap_or_else(|| panic!("`{refused}` reads the switch and must be refused"));
        assert!(why.contains(SELECTION_SWITCH), "-> {why}");
    }
    // An output of the classifier job the switch does not feed.
    for refused in [
        "needs.changes.outputs.packaging == 'true'",
        "needs.changes.outputs.selected == 'true'",
        "matrix.shard == 3 && needs.changes.outputs.touched == 'true'",
    ] {
        let why = selection_gate_violation(refused).unwrap_or_else(|| {
            panic!("`{refused}` reads an ungoverned output and must be refused")
        });
        assert!(why.contains("is not one of the three outputs"), "-> {why}");
    }
}

/// Every gated step's job carries the skip-branch marker for its package.
#[test]
fn every_gated_step_has_a_selection_marker_in_its_job() {
    let mut complaints: Vec<String> = Vec::new();
    let mut gated_total = 0usize;
    for (file, text) in workflow_texts() {
        for (job, block) in jobs_of(&text) {
            let (gated, marked, mut trouble) = selection_gates_and_markers(&block);
            complaints.append(&mut trouble);
            gated_total += gated.len();
            for package in gated.difference(&marked) {
                complaints.push(format!(
                    "  {file} / {job}: a step is gated on `{package}` and the \
                     job carries no marker step for it. Add a step with \
                     `if: {}` whose script prints \
                     `{}<the selection>)`",
                    selection_marker_condition(package),
                    selection_marker_line(package)
                ));
            }
            for package in marked.difference(&gated) {
                complaints.push(format!(
                    "  {file} / {job}: a marker step names `{package}` and no \
                     step of this job is gated on it; a marker that describes \
                     nothing pre-authorises the next hole"
                ));
            }
        }
    }
    assert!(
        complaints.is_empty(),
        "selection markers do not match the gates:\n{}",
        complaints.join("\n")
    );
    // Non-vacuity. A workflow whose steps are all ungated satisfies this arm
    // and the totality arm without either one looking at anything, so the live
    // population is required to be real.
    assert!(
        gated_total >= 20,
        "only {gated_total} step gate(s) were found across the workflows: the \
         gate reader is broken, or the selection was removed, and this arm and \
         the totality arm below would both be vacuous"
    );
}

/// The marker rule, both sides, on synthetic jobs.
#[test]
fn a_gated_step_without_its_marker_is_a_red_walker() {
    let gate = format!(
        "        if: {}\n",
        SELECTION_PKG_IF_OPEN.to_string() + "alpha" + SELECTION_PKG_IF_CLOSE
    );
    let step = format!("      - name: alpha tests\n{gate}        run: cargo test -p alpha\n");
    let marker = format!(
        "      - name: selection marker, alpha\n        if: {}\n        run: |\n          echo \"{}$SELECTED)\"\n",
        selection_marker_condition("alpha"),
        selection_marker_line("alpha")
    );
    let job = |steps: &str| format!("  j:\n    runs-on: ubuntu-latest\n    steps:\n{steps}");

    let (gated, marked, trouble) = selection_gates_and_markers(&job(&format!("{step}{marker}")));
    assert!(trouble.is_empty(), "-> {trouble:?}");
    assert_eq!(gated, marked, "a gated step WITH its marker is covered");
    assert_eq!(gated, ["alpha".to_string()].into_iter().collect());

    let (gated, marked, trouble) = selection_gates_and_markers(&job(&step));
    assert!(trouble.is_empty(), "-> {trouble:?}");
    assert!(
        marked.is_empty() && gated.len() == 1,
        "a gated step with NO marker leaves its package uncovered: gated \
         {gated:?}, marked {marked:?}"
    );

    // An UNGATED step needs no marker, and produces none.
    let plain = "      - name: alpha tests\n        run: cargo test -p alpha\n";
    let (gated, marked, trouble) = selection_gates_and_markers(&job(plain));
    assert!(trouble.is_empty(), "-> {trouble:?}");
    assert!(gated.is_empty() && marked.is_empty());

    // A marker that prints nothing, and one that prints the wrong line, are
    // both complaints rather than silent passes.
    let silent = format!(
        "      - name: selection marker, alpha\n        if: {}\n        run: true\n",
        selection_marker_condition("alpha")
    );
    let (_, marked, trouble) = selection_gates_and_markers(&job(&format!("{step}{silent}")));
    assert!(marked.is_empty(), "a silent marker covers nothing");
    assert_eq!(trouble.len(), 1, "-> {trouble:?}");
    let wrong = format!(
        "      - name: selection marker, alpha\n        if: {}\n        run: |\n          echo \"selection: something else\"\n",
        selection_marker_condition("alpha")
    );
    let (_, marked, trouble) = selection_gates_and_markers(&job(&format!("{step}{wrong}")));
    assert!(
        marked.is_empty(),
        "a marker printing the wrong line covers nothing"
    );
    assert_eq!(trouble.len(), 1, "-> {trouble:?}");
}

/// The call every dependant of the classifier has to open its job condition
/// with, and the separator that follows it when the job keeps a gate of its
/// own.
const GUARD_CALL: &str = "!cancelled()";
const GUARD_CALL_AND: &str = "!cancelled() && ";

/// One condition as a single normalised expression: the `${{ }}` wrapper off,
/// runs of whitespace down to one space.
///
/// A job condition means the same thing spelled `${{ expr }}` on one line and
/// spelled `expr` in a folded block scalar, and `ci.yml` uses both.
fn condition_expression(cond: &str) -> String {
    let trimmed = cond.trim();
    let inner = trimmed
        .strip_prefix("${{")
        .and_then(|rest| rest.strip_suffix("}}"))
        .unwrap_or(trimmed);
    inner.split_whitespace().collect::<Vec<_>>().join(" ")
}

/// Does this job condition keep the job running when a job it `needs:` FAILED?
///
/// Exactly `!cancelled()`, or `!cancelled()` as the FIRST term of an `&&`
/// chain, which is `deb-smoke`'s shape. Nothing else: `success() &&
/// !cancelled()` carries the call and still skips on a failed dependency,
/// which is the whole fault.
fn condition_survives_a_failed_dependency(cond: &str) -> bool {
    let expr = condition_expression(cond);
    expr == GUARD_CALL || expr.starts_with(GUARD_CALL_AND)
}

/// Every dependant of the classifier that does NOT guard itself against the
/// classifier failing.
///
/// WHY THIS IS THE SEVEREST ARM IN THE FILE. GitHub SKIPS a dependant when its
/// dependency fails, and branch protection counts a SKIPPED required context as
/// SATISFIED: a classifier that failed for any reason, a checkout, a resolve, a
/// typo in the script, would skip four jobs carrying eleven of the twenty
/// required contexts and the pull request would merge with none of them run. A
/// condition opening with `!cancelled()` makes the job run anyway; `pkgs` is
/// then the empty string, `fromJSON('')` is an expression error, the step fails
/// and the job reds.
///
/// THE POPULATION IS EVERY DEPENDANT, not every job with a one-line gate. A job
/// that `needs:` the classifier is skipped by a classifier failure however it
/// consumes the outputs, and `ci.yml` already passes the selection as a plain
/// `env:` value on two steps. The step gates are read too, through
/// [`step_if_of`], so a gate spelled as a folded block scalar counts like any
/// other; a form that reader cannot classify counts as a gate as well, so an
/// unreadable condition makes the job owe the guard instead of escaping it.
///
/// The other direction is checked too: a job that neither needs the classifier
/// nor gates on the selection owes no guard, so the rule cannot be satisfied by
/// pasting `!cancelled()` everywhere.
fn jobs_missing_the_not_cancelled_guard(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    for (job, block) in jobs_of(text) {
        let depends = job_needs(&block).iter().any(|n| n == SELECTION_JOB);
        let gates = step_blocks(&block).iter().any(|step| match step_if_of(step) {
            Ok(Some(cond)) => cond.contains(SELECTION_PKG_IF_OPEN),
            Ok(None) => false,
            Err(_) => true,
        });
        if !(depends || gates) {
            continue;
        }
        let guarded = matches!(
            job_if_of(&block),
            Some(Ok(cond)) if condition_survives_a_failed_dependency(&cond)
        );
        if !guarded {
            out.push(job);
        }
    }
    out
}

#[test]
fn every_job_that_gates_a_step_on_the_selection_guards_itself() {
    let mut complaints: Vec<String> = Vec::new();
    let mut guarded = 0usize;
    for (file, text) in workflow_texts() {
        for job in jobs_missing_the_not_cancelled_guard(&text) {
            complaints.push(format!("  {file} / {job}"));
        }
        guarded += jobs_of(&text)
            .iter()
            .filter(|(_, block)| {
                matches!(job_if_of(block), Some(Ok(cond)) if cond.trim() == JOB_IF_NOT_CANCELLED)
            })
            .count();
    }
    assert!(
        complaints.is_empty(),
        "these jobs depend on the `{SELECTION_JOB}` job, or gate a step on the \
         selection, and their job-level `if:` does not open with \
         `{GUARD_CALL}`:\n{}\n\nGitHub skips a dependant of a FAILED job and a \
         skipped required context reads as satisfied, so without the guard a \
         classifier failure is a silent green around every context these jobs \
         report. `{JOB_IF_NOT_CANCELLED}` is the shape a job with no gate of its \
         own carries; a job keeping its own gate spells it \
         `{GUARD_CALL_AND}<the rest>`.",
        complaints.join("\n")
    );
    assert!(
        guarded >= 4,
        "only {guarded} job(s) carry the guard: the four jobs that gate steps on \
         the selection each need it, so either the reader is broken or the \
         guards were removed"
    );
    // The population is not empty, so an emptied reader is not a pass. Five
    // jobs of `ci.yml` need the classifier today.
    let dependants: usize = workflow_texts()
        .values()
        .map(|text| {
            jobs_of(text)
                .iter()
                .filter(|(_, block)| job_needs(block).iter().any(|n| n == SELECTION_JOB))
                .count()
        })
        .sum();
    assert!(
        dependants >= 5,
        "only {dependants} job(s) name `{SELECTION_JOB}` in `needs:`: the rule \
         above is judging a population the reader is no longer finding"
    );
}

/// The guard rule, both sides, on synthetic jobs.
#[test]
fn a_gating_job_without_the_guard_is_named_and_an_ungated_one_is_not() {
    let gate = format!("{SELECTION_PKG_IF_OPEN}alpha{SELECTION_PKG_IF_CLOSE}");
    let body = format!(
        "    needs: [changes]\n{{guard}}    steps:\n      - name: alpha tests\n        \
         if: {gate}\n        run: cargo test -p alpha\n"
    );
    let job = |guard: &str| format!("  j:\n{}", body.replace("{guard}", guard));

    assert_eq!(
        jobs_missing_the_not_cancelled_guard(&job("")),
        vec!["j".to_string()],
        "a job that gates a step and carries no job-level condition is named"
    );
    assert_eq!(
        jobs_missing_the_not_cancelled_guard(&job(&format!("    if: {JOB_IF_NOT_CANCELLED}\n"))),
        Vec::<String>::new(),
        "the guard satisfies the rule"
    );
    // A condition that OPENS with the call keeps the job running when the
    // classifier failed, so it satisfies THIS rule; whether such a job is
    // PR-blocking is the separate question `job_is_gated` decides, pinned by
    // `the_not_cancelled_guard_is_the_only_job_condition_that_keeps_a_job_pr_blocking`.
    assert_eq!(
        jobs_missing_the_not_cancelled_guard(&job(
            "    if: ${{ !cancelled() && github.event_name != 'pull_request' }}\n"
        )),
        Vec::<String>::new(),
        "a condition opening with the call still runs on a failed dependency"
    );
    // Carrying the call somewhere else is not opening with it: `success()` is
    // false the moment the classifier fails, so this job skips exactly when the
    // guard is meant to save it.
    assert_eq!(
        jobs_missing_the_not_cancelled_guard(&job(
            "    if: ${{ success() && !cancelled() }}\n"
        )),
        vec!["j".to_string()],
        "the call has to OPEN the condition"
    );
    // And a condition with no such call at all is named.
    assert_eq!(
        jobs_missing_the_not_cancelled_guard(&job(
            "    if: ${{ github.event_name != 'pull_request' }}\n"
        )),
        vec!["j".to_string()],
        "an ordinary event test is not the guard"
    );
    // The other side: a job that neither needs the classifier nor gates on the
    // selection owes nothing.
    let ungated = "  j:\n    steps:\n      - name: alpha tests\n        run: cargo test -p alpha\n";
    assert_eq!(
        jobs_missing_the_not_cancelled_guard(ungated),
        Vec::<String>::new(),
        "a job that is no dependant and carries no selection gate owes no guard"
    );
    // A gate spelled inside a COMMENT gates nothing, so it demands no guard.
    let commented = format!(
        "  j:\n    steps:\n      - name: alpha tests\n        # if: {gate}\n        \
         run: cargo test -p alpha\n"
    );
    assert_eq!(
        jobs_missing_the_not_cancelled_guard(&commented),
        Vec::<String>::new(),
        "a gate inside a comment gates nothing"
    );

    // A DEPENDANT THAT CARRIES NO GATE AT ALL. `ci.yml` already hands the
    // selection to two steps as a plain `env:` value, and the shard runner
    // intersects it script-side, so a job built that way has no `if:` naming
    // the selection and is skipped by a classifier failure just the same.
    let env_only = format!(
        "  j:\n    needs: [{SELECTION_JOB}]\n    env:\n      CI_SELECTED_PACKAGES: ${{{{ \
         needs.{SELECTION_JOB}.outputs.pkgs }}}}\n    steps:\n      - name: alpha tests\n        \
         run: ./tools/scripts/ci_test_shard.sh alpha\n"
    );
    assert_eq!(
        jobs_missing_the_not_cancelled_guard(&env_only),
        vec!["j".to_string()],
        "a dependant consuming the selection through `env:` still owes the guard"
    );

    // A GATE SPELLED AS A FOLDED BLOCK SCALAR. The workflows spell compound
    // conditions this way throughout, and the marker rule beside this one
    // already reads every scalar form; a reader that needs both halves on one
    // line drops the job out of the population.
    let folded = format!(
        "  j:\n    steps:\n      - name: alpha tests\n        if: >-\n          {gate}\n        \
         run: cargo test -p alpha\n"
    );
    assert_eq!(
        jobs_missing_the_not_cancelled_guard(&folded),
        vec!["j".to_string()],
        "a gate spelled as a folded block scalar is still a gate"
    );

    // THE PACKAGING JOB'S SHAPE, accepted: it needs the classifier, it keeps a
    // gate of its own, and it opens with the call, so a classifier failure
    // leaves it running with an empty `packaging` and it skips on its own
    // terms rather than on GitHub's.
    let packaging = format!(
        "  deb-smoke:\n    needs: [{SELECTION_JOB}]\n    if: >-\n      !cancelled()\n      \
         && ((github.event_name != 'pull_request' && github.event_name != 'merge_group')\n      \
         || (github.event_name == 'pull_request' && needs.{SELECTION_JOB}.outputs.packaging == \
         'true'))\n    steps:\n      - name: smoke\n        run: ./tools/scripts/build_deb.sh\n"
    );
    assert_eq!(
        jobs_missing_the_not_cancelled_guard(&packaging),
        Vec::<String>::new(),
        "a dependant whose own gate follows the call is guarded"
    );
}

/// A job carrying exactly `!cancelled()` still credits its packages, and one
/// carrying any other job-level condition still does not.
#[test]
fn the_not_cancelled_guard_is_the_only_job_condition_that_keeps_a_job_pr_blocking() {
    let body = "    steps:\n      - name: alpha tests\n        run: cargo test -p alpha\n";
    let with = format!("  j:\n    if: {JOB_IF_NOT_CANCELLED}\n{body}");
    let without = format!("  j:\n{body}");
    let other = format!("  j:\n    if: github.event_name == 'push'\n{body}");

    for (name, text) in [("guarded", &with), ("ungated", &without)] {
        assert!(
            pr_blocking_jobs(text).contains("cargo test -p alpha"),
            "the {name} job must stay in the PR-blocking view"
        );
    }
    assert!(
        !pr_blocking_jobs(&other).contains("cargo test -p alpha"),
        "a job behind an ordinary event test is still dropped"
    );
}

/// The name of the Lint step that must run the shard check, and the invocation
/// it must carry.
///
/// Spelled whole, because a renamed step is a step nobody can find in a log and
/// a step with a different argument is a different check: `--check` with no
/// argument is the SHIPPED configuration (the script's own defaults), and
/// `--check cerulion_core 3` proves a partition the workflow does not run.
const SHARD_CHECK_STEP: &str = "Shard partition and selection check";
const SHARD_CHECK_RUN: &str = "./tools/scripts/ci_test_shard.sh --check";

/// The workflow and the job the step belongs to.
///
/// A step of the right name running the right script proves nothing about WHEN
/// it runs. Six jobs of `ci.yml` sit behind `github.event_name != 'pull_request'
/// && github.event_name != 'merge_group'` under a cost policy, and the other
/// workflows run on their own events, so the same step moved into one of those
/// keeps its name and its script and stops running on a pull request, which is
/// the one property the step exists for.
const SHARD_CHECK_WORKFLOW: &str = "ci.yml";
const SHARD_CHECK_JOB: &str = "lint";

/// Is this run script the shipped invocation, WHOLE?
///
/// Equality, never `contains`. `./tools/scripts/ci_test_shard.sh --check
/// cerulion_core 2` and `./tools/scripts/ci_test_shard.sh --check || true` both
/// carry the bare form as a prefix: the first proves a two-shard partition the
/// workflow does not run, and the second reports success whatever the check
/// says. The reader's own both-sides test compares for equality already.
fn is_the_shard_check_run(script: &str) -> bool {
    script.trim() == SHARD_CHECK_RUN
}

/// Where a workflow runs the shard check, and what it runs there.
struct ShardCheckStep {
    /// The id of the job holding the step.
    job: String,
    /// The step's `run:` script, joined.
    script: String,
    /// The step's own `if:`, read in every scalar form. `Ok(None)` is a step
    /// with no condition of its own, which is the only shape the rule accepts:
    /// a step gated on an event runs nowhere else while its job still reports.
    condition: Result<Option<String>, String>,
}

/// Does any step of this workflow run the shard check under its own name?
fn shard_check_step_of(text: &str) -> Option<ShardCheckStep> {
    for (job, block) in jobs_of(text) {
        for step in step_blocks(&block) {
            if step_name_of(&step).as_deref() != Some(SHARD_CHECK_STEP) {
                continue;
            }
            return Some(ShardCheckStep {
                job,
                script: run_script_of(&step).unwrap_or_default(),
                condition: step_if_of(&step),
            });
        }
    }
    None
}

/// Everything wrong with the way one workflow runs the shard check, as the
/// lines a failure prints. EMPTY is the shipped shape.
///
/// The rule is a function so the synthetic rows below judge the same thing the
/// real workflow is judged on.
fn shard_check_complaints(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    let Some(found) = shard_check_step_of(text) else {
        out.push(format!("no job carries a step named `{SHARD_CHECK_STEP}`"));
        return out;
    };
    if found.job != SHARD_CHECK_JOB {
        out.push(format!(
            "the step sits in the `{}` job, and the rule names `{SHARD_CHECK_JOB}`",
            found.job
        ));
    }
    if !is_the_shard_check_run(&found.script) {
        out.push(format!(
            "the step runs `{}`, and the rule names `{SHARD_CHECK_RUN}`",
            found.script.trim()
        ));
    }
    match &found.condition {
        Ok(None) => {}
        Ok(Some(cond)) => out.push(format!(
            "the step carries its own condition `{cond}`, so it runs on fewer \
             events than the job that holds it"
        )),
        Err(why) => out.push(format!(
            "the step carries a condition this walk cannot read: {why}"
        )),
    }
    if !jobs_of(&pr_blocking_jobs(text))
        .iter()
        .any(|(job, _)| *job == found.job)
    {
        out.push(format!(
            "the `{}` job is not pull-request blocking, so the check does not \
             run on a pull request",
            found.job
        ));
    }
    out
}

/// The shard check runs on every pull request, in the `lint` job of `ci.yml`,
/// under a named step, with the invocation the script documents.
///
/// WHY A TEST HOLDS A WORKFLOW STEP. `ci_test_shard.sh --check` carries the
/// proof that the partition is total and disjoint AND the hand table its
/// selection reader is held to. Nothing in the workflow ran it: the only
/// caller was `the_ci_test_shard_partition_is_total_and_disjoint` in this
/// file, which runs under `cargo test -p cerulion_cli_engine`. That is a gate
/// whose CI invocation is one selection away from disappearing, and a gate
/// nothing invokes is inert.
///
/// WHY THE PLACE IS PART OF THE RULE. The name and the script say what runs,
/// never when. `ci.yml` alone is read, the job id is pinned, the job has to
/// survive the pull-request view, and the step may carry no condition of its
/// own: each of those is a way the step keeps its name and stops running where
/// it matters.
#[test]
fn the_shard_check_runs_in_a_named_lint_step() {
    let texts = workflow_texts();
    let ci = texts.get(SHARD_CHECK_WORKFLOW).unwrap_or_else(|| {
        panic!("the workflow walk found no `{SHARD_CHECK_WORKFLOW}`")
    });
    let complaints = shard_check_complaints(ci);
    assert!(
        complaints.is_empty(),
        "`{SHARD_CHECK_WORKFLOW}` does not run `{SHARD_CHECK_RUN}` the way the \
         rule names: {}. That invocation proves the shard partition is total \
         and disjoint and holds the selection reader to its hand table, and it \
         has to run on a pull request that never runs `cerulion_cli_engine`.",
        complaints.join("; ")
    );
}

/// The reader, both sides, on synthetic workflows.
#[test]
fn a_renamed_or_rewritten_shard_check_step_is_not_found() {
    let job = |name: &str, run: &str| {
        format!("  lint:\n    steps:\n      - name: {name}\n        run: {run}\n")
    };
    assert_eq!(
        shard_check_step_of(&job(SHARD_CHECK_STEP, SHARD_CHECK_RUN))
            .map(|found| found.script.trim().to_string())
            .as_deref(),
        Some(SHARD_CHECK_RUN),
        "the named step running the documented invocation is found"
    );
    assert!(
        shard_check_step_of(&job("Shard check", SHARD_CHECK_RUN)).is_none(),
        "a renamed step is not found"
    );
    let wrong = job(
        SHARD_CHECK_STEP,
        "./tools/scripts/ci_test_shard.sh --list cerulion_core 0 4",
    );
    assert!(
        !shard_check_step_of(&wrong)
            .expect("the step is named")
            .script
            .contains(SHARD_CHECK_RUN),
        "a step of the right name running something else does not satisfy the rule"
    );
    // THE SAME PREDICATE THE ARM ABOVE USES, on the two shapes its message
    // names. Both carry the bare invocation as a prefix, so a `contains`
    // reading accepts them: the first runs a two-shard partition the workflow
    // does not run, and the second passes the step whatever the check reports.
    for run in [
        "./tools/scripts/ci_test_shard.sh --check cerulion_core 2",
        "./tools/scripts/ci_test_shard.sh --check || true",
    ] {
        let script = shard_check_step_of(&job(SHARD_CHECK_STEP, run))
            .expect("the step is named")
            .script;
        assert!(
            script.contains(SHARD_CHECK_RUN),
            "`{run}` carries the bare invocation, so this row says something about \
             the predicate and not about the reader; it read back `{}`",
            script.trim()
        );
        assert!(
            !is_the_shard_check_run(&script),
            "`{run}` is not the shipped invocation, and the rule rejects it"
        );
    }
    let bare = shard_check_step_of(&job(SHARD_CHECK_STEP, SHARD_CHECK_RUN))
        .expect("the step is named")
        .script;
    assert!(
        is_the_shard_check_run(&bare),
        "the bare invocation IS the shipped one, so the rule is not satisfied by \
         rejecting everything"
    );
}

/// WHERE the shard check runs, both sides, on synthetic workflows.
#[test]
fn a_shard_check_step_outside_a_blocking_lint_job_is_named() {
    let shipped = format!(
        "jobs:\n  {SHARD_CHECK_JOB}:\n    steps:\n      - name: {SHARD_CHECK_STEP}\n        \
         run: {SHARD_CHECK_RUN}\n"
    );
    assert!(
        shard_check_complaints(&shipped).is_empty(),
        "the shipped shape draws no complaint: {:?}",
        shard_check_complaints(&shipped)
    );

    // The cost-policy move: the same step, same name, same invocation, in a
    // job that skips on a pull request. It is the wrong job AND the job is not
    // pull-request blocking, and both are said.
    let moved = format!(
        "jobs:\n  msrv:\n    if: github.event_name != 'pull_request'\n    steps:\n      \
         - name: {SHARD_CHECK_STEP}\n        run: {SHARD_CHECK_RUN}\n"
    );
    let complaints = shard_check_complaints(&moved);
    assert!(
        complaints.iter().any(|c| c.contains("`msrv` job")),
        "a step in another job is named by its job: {complaints:?}"
    );
    assert!(
        complaints.iter().any(|c| c.contains("pull-request blocking")),
        "a step in a job that skips on a pull request is named for that: \
         {complaints:?}"
    );

    // The job id alone is not the rule: `lint` behind an event test is still a
    // job the check does not run in on a pull request.
    let gated = format!(
        "jobs:\n  {SHARD_CHECK_JOB}:\n    if: github.event_name != 'pull_request'\n    \
         steps:\n      - name: {SHARD_CHECK_STEP}\n        run: {SHARD_CHECK_RUN}\n"
    );
    let complaints = shard_check_complaints(&gated);
    assert!(
        complaints.iter().any(|c| c.contains("pull-request blocking")),
        "the right job behind an event test is still named: {complaints:?}"
    );
    assert!(
        !complaints.iter().any(|c| c.contains("job, and the rule names")),
        "and it is not named for sitting in the wrong job: {complaints:?}"
    );

    // A step-level condition is the third way the step keeps its name and
    // stops running: the job reports, the step does not.
    let conditioned = format!(
        "jobs:\n  {SHARD_CHECK_JOB}:\n    steps:\n      - name: {SHARD_CHECK_STEP}\n        \
         if: github.event_name == 'push'\n        run: {SHARD_CHECK_RUN}\n"
    );
    let complaints = shard_check_complaints(&conditioned);
    assert!(
        complaints.iter().any(|c| c.contains("its own condition")),
        "a step with a condition of its own is named: {complaints:?}"
    );
}
