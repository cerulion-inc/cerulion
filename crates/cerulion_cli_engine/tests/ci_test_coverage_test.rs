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
//!     `matrix.<key> == <literal>` LEG SELECTOR, because every leg of a
//!     PR-blocking job's matrix runs on every pull request, so the step runs on
//!     exactly one of them. Everything else — `github.event_name == 'push'`
//!     most of all — disqualifies the step.
//!
//! Both job rules and the step rule are deliberately blunt in the FAIL-CLOSED
//! direction: a future condition that genuinely still runs on pull requests
//! costs a maintainer one line here rather than costing everyone a silent hole.
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

/// Can this step-level `if:` condition stop the step running on a pull
/// request?
///
/// FAIL-CLOSED: anything not recognised as harmless disqualifies the step.
/// Recognised as harmless are `always()`, `success()`, and a
/// `matrix.<key> == <literal>` / `!= <literal>` LEG SELECTOR — every leg of a
/// PR-blocking job's matrix runs on every pull request, so a leg-selected step
/// runs on at least one of them and can therefore fail one.
/// `github.event_name == 'push'`, `runner.os == 'Linux'` and anything with a
/// `||` do not qualify.
fn step_if_is_pr_blocking(cond: &str) -> bool {
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
        let Some(rest) = t.strip_prefix("matrix.") else {
            return false;
        };
        let Some((key, _)) = rest.split_once("==").or_else(|| rest.split_once("!=")) else {
            return false;
        };
        let key = key.trim().trim_end_matches('!');
        !key.is_empty()
            && key
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
    })
}

/// Drop every STEP of one job whose `if:` could stop it on a pull request.
///
/// Steps are list items under `steps:`; the item indent is read from the first
/// one rather than hard-coded, and a step's own keys sit two spaces deeper.
fn drop_gated_steps(job: &str) -> String {
    let lines: Vec<&str> = job.lines().collect();
    let Some(steps_at) = lines.iter().position(|l| l.trim() == "steps:") else {
        return job.to_string();
    };
    let mut out: Vec<&str> = lines[..=steps_at].to_vec();
    let body = &lines[steps_at + 1..];

    let indent_of = |l: &str| l.len() - l.trim_start().len();
    let Some(step_indent) = body
        .iter()
        .find(|l| l.trim_start().starts_with("- "))
        .map(|l| indent_of(l))
    else {
        out.extend_from_slice(body);
        return out.join("\n");
    };
    let is_step_start = |l: &str| indent_of(l) == step_indent && l.trim_start().starts_with("- ");

    let mut cur: Vec<&str> = Vec::new();
    let mut gated = false;
    for line in body {
        if is_step_start(line) {
            if !gated {
                out.append(&mut cur);
            }
            cur.clear();
            gated = false;
        }
        let trimmed = line.trim_start();
        // A step's `if:` is either its own key (two deeper than the item) or
        // the first key on the `- if: …` item line itself.
        let cond = if indent_of(line) == step_indent + 2 {
            trimmed.strip_prefix("if:")
        } else if is_step_start(line) {
            trimmed
                .strip_prefix("- ")
                .and_then(|r| r.strip_prefix("if:"))
        } else {
            None
        };
        if let Some(c) = cond {
            if !step_if_is_pr_blocking(c) {
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
    let has_job_if = |block: &str| block.lines().any(|l| l.starts_with("    if:"));
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

    jobs.iter()
        .filter(|(name, block)| !gated.contains(name) && !is_soft(block))
        .map(|(_, block)| drop_gated_steps(block))
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
    let indent_of = |l: &str| l.len() - l.trim_start().len();

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
