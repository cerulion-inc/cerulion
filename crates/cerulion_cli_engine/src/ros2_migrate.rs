// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion ros2 migrate` — rewrite a colcon workspace's C++ publish call
//! sites to the upstream loaned-message API where the AST PROVES it safe,
//! with a manual-candidates report for everything refused and a
//! report-only section for rclpy nodes (rclpy has no loan API upstream).
//!
//! # Division of labor
//!
//! The AST prover/rewriter is a standalone clang LibTooling tool
//! (`tools/ros2_migrate/cerulion_ros2_migrate_clang.cpp`, built in the ROS 2
//! toolchain container — the Rust workspace never links libclang). This
//! module ORCHESTRATES it: locate the workspace + `compile_commands.json`
//! (loud remediation naming the exact `colcon build --cmake-args
//! -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` command when absent), run the tool
//! per translation unit, merge + validate the proposed edits, render ONE
//! deterministic report (unified diff + candidates + rclpy), and — behind
//! the consent gate — apply.
//!
//! # The UX contract (the `ros2 attach` discipline)
//!
//! * Dry-run is the DEFAULT. It prints the full diff + candidates report and
//!   refreshes the machine-readable manifest at
//!   [`MANIFEST_REL_PATH`] — its only write.
//! * `--write` applies behind consent (TTY prompt, or `--yes`;
//!   no-TTY-no-`--yes` refuses loudly naming both). It REFUSES a dirty git
//!   tree, writes ONE commit plus [`PATCH_FILENAME`] (undo =
//!   `git revert <sha>` or `git apply -R`), then AUTOMATICALLY runs
//!   `colcon build --packages-select <affected>` — a build failure is loud
//!   and names the revert path, never silent.
//! * The dry-run diff and the written bytes come from the SAME
//!   [`MigrationPlan`] — the patch file is byte-identical to the printed
//!   diff by construction, and tests assert it.
//! * `--write` holds the WORKSPACE LOCK
//!   ([`crate::workspace_lock::WorkspaceLock::acquire_interruptibly`])
//!   for exactly its write batch — from after consent through the commit and
//!   the manifest refresh, released before the colcon build — so a
//!   concurrent Cerulion writer on the SAME root (a second `migrate`, or
//!   `ros2 attach`/`node`/`graph`/`schema`/`cerulion-wsd` in a layout where
//!   the colcon and Cerulion workspace roots coincide) waits instead of
//!   interleaving its writes with this one's. The lock is per-root and
//!   `--workspace` is the COLCON root, so two migrations of DIFFERENT
//!   workspaces inside one git repository are not serialized against each
//!   other even though they share an index.
//! * The dry-run takes NO lock. Not because it cannot —
//!   [`crate::workspace_lock::WorkspaceLock::acquire_read`] creates nothing —
//!   but because its one product is HEAD-keyed, so a manifest written by the
//!   loser of a race normally names a `workspace_key.head` its consumer
//!   discards. That is a COMMON-CASE argument, not a guarantee: HEAD is read
//!   AFTER the translation units are analysed, so a `--write` committing in
//!   that window leaves a manifest keyed to the NEW head while describing the
//!   OLD tree. The accepted cost is therefore both halves — a diff and counts
//!   that can describe a state that never existed, and a manifest that can
//!   survive invalidation. Re-run the dry-run if the report looks
//!   inconsistent.
//!
//! # The manifest
//!
//! `.cerulion/ros2-migrate-manifest.json`, schema
//! [`MANIFEST_SCHEMA`] v[`MANIFEST_VERSION`]: the workspace git key (HEAD +
//! a tracked-dirty flag — consumers invalidate on mismatch), the provable
//! call-site list (package/file/line/kind), the manual candidates with
//! reasons, the rclpy report, counts, and a `decision` slot this module
//! writes as `null` and PRESERVES verbatim on refresh (the launcher records
//! a declined offer there; a dry-run must not forget it). Writes are atomic
//! (temp + rename). `--write` clears the candidates it consumed and
//! re-keys to the post-commit HEAD.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde::{Deserialize, Serialize};

use crate::error::{CliError, CliResult};
use crate::workspace_lock::{AcquireError, WorkspaceLock};

/// Engine-binary basename (also what `--write`'s refusal names).
pub const TOOL_BIN: &str = "cerulion-ros2-migrate-clang";
/// Env override for the engine binary path. When set, it must exist — a
/// missing explicit path is a loud error, never a silent fallback.
pub const TOOL_ENV: &str = "CERULION_ROS2_MIGRATE_TOOL";
/// The tool-output format this module understands.
pub const TOOL_FORMAT_VERSION: u64 = 1;
/// The patch file `--write` emits into the workspace root.
pub const PATCH_FILENAME: &str = "cerulion-ros2-migration.patch";
/// The machine-readable manifest, workspace-relative.
pub const MANIFEST_REL_PATH: &str = ".cerulion/ros2-migrate-manifest.json";
/// Manifest schema identifier + version.
pub const MANIFEST_SCHEMA: &str = "cerulion-ros2-migrate-manifest";
pub const MANIFEST_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// Tool JSON contract (deserialized verbatim from the clang tool's stdout).
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolOutput {
    pub format: u64,
    #[serde(default)]
    pub tool_version: String,
    /// The translation unit analysed (absolute).
    pub file: String,
    #[serde(default)]
    pub rewrites: Vec<ToolRewrite>,
    #[serde(default)]
    pub candidates: Vec<ToolCandidate>,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct ToolRewrite {
    /// The file holding the edits (absolute; may be a header ≠ the TU).
    pub file: String,
    pub function: String,
    /// "unique_ptr" | "shared_ptr" | "stack".
    pub kind: String,
    pub message_type: String,
    pub publisher: String,
    /// Publish call line (1-based).
    pub line: u64,
    pub edits: Vec<ToolEdit>,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct ToolEdit {
    pub offset: usize,
    pub length: usize,
    /// The exact bytes the edit replaces — re-verified against the file on
    /// disk before anything is applied.
    pub original: String,
    pub replacement: String,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct ToolCandidate {
    pub file: String,
    pub function: String,
    pub line: u64,
    /// The per-call-site discriminator.
    ///
    /// Candidates dedupe on their location, and a macro whose body spells the
    /// publish gives EVERY invocation the same `(file, line)` — so without a
    /// column N distinct sites collapsed into one reported candidate and N-1
    /// vanished from the report and the manifest. The engine now anchors a
    /// candidate at its EXPANSION location and carries the column.
    ///
    /// `default` rather than required: this field is additive, and a
    /// `cerulion-ros2-migrate-clang` that predates the column (one already on the
    /// operator's PATH, or pinned by `CERULION_ROS2_MIGRATE_TOOL`) emits no
    /// column. Absent it reads 0 for every candidate, which degrades exactly
    /// to the pre-column dedup rather than refusing to parse.
    #[serde(default)]
    pub column: u64,
    pub reason: String,
    pub detail: String,
}

/// Parse + format-gate one tool invocation's stdout.
pub fn parse_tool_output(json: &str) -> CliResult<ToolOutput> {
    let out: ToolOutput = serde_json::from_str(json).map_err(|e| {
        CliError::Validation(format!("migration engine produced unparseable JSON: {e}"))
    })?;
    if out.format != TOOL_FORMAT_VERSION {
        return Err(CliError::Validation(format!(
            "migration engine speaks format {} but this cerulion understands \
             format {} — rebuild the engine and the CLI from the same \
             checkout (tools/ros2_migrate/README.md)",
            out.format, TOOL_FORMAT_VERSION
        )));
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Merge across translation units.
// ---------------------------------------------------------------------------

/// The merged, validated analysis over every TU: rewrites deduped (a header
/// site is seen once per including TU with byte-identical edits), candidates
/// deduped by (file, line, reason), and any file whose surviving edits
/// OVERLAP demoted wholesale to candidates (reason `conflicting-analyses`) —
/// two TUs proposed different bytes for one region, which only happens when
/// preprocessor context genuinely differs, and a guess is not a proof.
#[derive(Debug, Default)]
pub struct MergedAnalysis {
    pub rewrites: Vec<ToolRewrite>,
    pub candidates: Vec<ToolCandidate>,
}

pub fn merge_tool_outputs(outputs: &[ToolOutput]) -> MergedAnalysis {
    let mut rewrites: Vec<ToolRewrite> = Vec::new();
    for out in outputs {
        for rw in &out.rewrites {
            if !rewrites.contains(rw) {
                rewrites.push(rw.clone());
            }
        }
    }
    rewrites.sort_by(|a, b| {
        let ao = a.edits.first().map(|e| e.offset).unwrap_or(0);
        let bo = b.edits.first().map(|e| e.offset).unwrap_or(0);
        (a.file.as_str(), ao).cmp(&(b.file.as_str(), bo))
    });

    // Overlap detection per file — a conflict demotes the whole file.
    let mut by_file: BTreeMap<&str, Vec<&ToolRewrite>> = BTreeMap::new();
    for rw in &rewrites {
        by_file.entry(rw.file.as_str()).or_default().push(rw);
    }
    let mut conflicted: Vec<String> = Vec::new();
    for (file, rws) in &by_file {
        let mut spans: Vec<(usize, usize)> = rws
            .iter()
            .flat_map(|rw| rw.edits.iter().map(|e| (e.offset, e.offset + e.length)))
            .collect();
        spans.sort_unstable();
        if spans.windows(2).any(|w| w[1].0 < w[0].1) {
            conflicted.push((*file).to_string());
        }
    }

    fn push_candidate(c: ToolCandidate, list: &mut Vec<ToolCandidate>) {
        // The COLUMN is part of the identity. Two publishes on one
        // source line — or, before the engine's expansion-location fix, N
        // invocations of one publishing macro — are distinct sites and must
        // both reach the operator's report.
        let dup = list.iter().any(|x| {
            x.file == c.file && x.line == c.line && x.column == c.column && x.reason == c.reason
        });
        if !dup {
            list.push(c);
        }
    }
    let mut candidates: Vec<ToolCandidate> = Vec::new();
    for out in outputs {
        for c in &out.candidates {
            push_candidate(c.clone(), &mut candidates);
        }
    }
    let (kept, demoted): (Vec<ToolRewrite>, Vec<ToolRewrite>) = rewrites
        .into_iter()
        .partition(|rw| !conflicted.contains(&rw.file));
    for rw in demoted {
        push_candidate(
            ToolCandidate {
                file: rw.file,
                function: rw.function,
                line: rw.line,
                // A demoted REWRITE carries no column — a rewrite is
                // anchored at its spelling location, which is where its
                // edit must land, and it has no expansion column to
                // report. 0 is the "not available" value here, and the
                // demotion is per-FILE, so two demoted rewrites on one
                // line are already the same conflict.
                column: 0,
                reason: "conflicting-analyses".to_string(),
                detail: "two translation units proposed different bytes for \
                         this region (preprocessor context differs) — \
                         migrate by hand"
                    .to_string(),
            },
            &mut candidates,
        );
    }
    // The column joins the ORDER as well as the identity — two
    // sites the dedup now keeps apart must also come out in a stable,
    // deterministic order (the report and the manifest are byte-compared).
    candidates.sort_by(|a, b| {
        (a.file.as_str(), a.line, a.column, a.reason.as_str()).cmp(&(
            b.file.as_str(),
            b.line,
            b.column,
            b.reason.as_str(),
        ))
    });
    MergedAnalysis {
        rewrites: kept,
        candidates,
    }
}

// ---------------------------------------------------------------------------
// Edit application.
// ---------------------------------------------------------------------------

/// Apply non-overlapping byte edits (verified against `original` bytes) —
/// bottom-up, so earlier offsets stay valid.
pub fn apply_edits(source: &[u8], edits: &[ToolEdit]) -> CliResult<Vec<u8>> {
    let mut sorted: Vec<&ToolEdit> = edits.iter().collect();
    sorted.sort_by_key(|e| e.offset);
    for w in sorted.windows(2) {
        if w[0].offset + w[0].length > w[1].offset {
            return Err(CliError::Validation(
                "internal: overlapping edits survived merge validation".to_string(),
            ));
        }
    }
    for e in &sorted {
        let end = e
            .offset
            .checked_add(e.length)
            .filter(|&end| end <= source.len());
        let Some(end) = end else {
            return Err(CliError::Validation(format!(
                "edit at offset {} (+{}) is out of bounds for a {}-byte file \
                 — the file changed since analysis; re-run the dry-run",
                e.offset,
                e.length,
                source.len()
            )));
        };
        if &source[e.offset..end] != e.original.as_bytes() {
            return Err(CliError::Validation(format!(
                "the bytes at offset {} no longer match what the analysis \
                 saw — the file changed since analysis; nothing was written. \
                 Re-run the dry-run",
                e.offset
            )));
        }
    }
    let mut out = source.to_vec();
    for e in sorted.iter().rev() {
        out.splice(
            e.offset..e.offset + e.length,
            e.replacement.as_bytes().iter().copied(),
        );
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Unified diff (derived DIRECTLY from the edit spans — no heuristic diff
// algorithm; exact by construction, pinned against `git apply` in tests).
// ---------------------------------------------------------------------------

const DIFF_CONTEXT: usize = 3;

/// Byte offset of each line start (a line = up to and including its `\n`;
/// a trailing newline does NOT open an empty final line).
fn line_starts(bytes: &[u8]) -> Vec<usize> {
    let mut starts = vec![0usize];
    for (i, b) in bytes.iter().enumerate() {
        if *b == b'\n' && i + 1 < bytes.len() {
            starts.push(i + 1);
        }
    }
    starts
}

fn line_index_of(starts: &[usize], offset: usize) -> usize {
    match starts.binary_search(&offset) {
        Ok(i) => i,
        Err(i) => i.saturating_sub(1),
    }
}

fn split_lines(bytes: &[u8]) -> Vec<&[u8]> {
    if bytes.is_empty() {
        return Vec::new();
    }
    bytes.split_inclusive(|b| *b == b'\n').collect()
}

/// Render a unified diff for one file whose `new` bytes were produced from
/// `old` by `edits` (non-overlapping, ascending). `rel` is the a/-b/ path.
///
/// Change RUNS are per-edit (merged only when they touch the same/adjacent
/// lines), so the lines BETWEEN two edits — the fill lines of the chosen
/// 3-line rewrite shape — render as CONTEXT, never as a -/+ pair.
pub fn unified_diff(rel: &str, old: &[u8], new: &[u8], edits: &[ToolEdit]) -> String {
    if edits.is_empty() || old == new {
        return String::new();
    }
    let old_lines = split_lines(old);
    let new_lines = split_lines(new);
    let starts = line_starts(old);
    let new_starts = line_starts(new);

    let mut sorted: Vec<&ToolEdit> = edits.iter().collect();
    sorted.sort_by_key(|e| e.offset);

    // Cumulative byte delta BEFORE each edit (for locating new-file spans).
    let deltas: Vec<isize> = {
        let mut acc = 0isize;
        let mut v = Vec::with_capacity(sorted.len());
        for e in &sorted {
            v.push(acc);
            acc += e.replacement.len() as isize - e.length as isize;
        }
        v
    };

    // A run = a maximal set of edits on the same/adjacent old lines.
    struct Run {
        l0: usize,
        l1: usize,
        first: usize, // edit index
        last: usize,
    }
    let mut runs: Vec<Run> = Vec::new();
    for (i, e) in sorted.iter().enumerate() {
        let a = line_index_of(&starts, e.offset);
        let end_off = if e.length == 0 {
            e.offset
        } else {
            e.offset + e.length - 1
        };
        let b = line_index_of(&starts, end_off);
        if let Some(last) = runs.last_mut() {
            if a <= last.l1 + 1 {
                last.l1 = last.l1.max(b);
                last.last = i;
                continue;
            }
        }
        runs.push(Run {
            l0: a,
            l1: b,
            first: i,
            last: i,
        });
    }
    // A hunk = runs whose context windows touch.
    let mut hunks: Vec<Vec<Run>> = Vec::new();
    for run in runs {
        if let Some(last_hunk) = hunks.last_mut() {
            let prev_l1 = last_hunk.last().expect("non-empty hunk").l1;
            if run.l0 <= prev_l1 + 2 * DIFF_CONTEXT + 1 {
                last_hunk.push(run);
                continue;
            }
        }
        hunks.push(vec![run]);
    }

    let push_line = |out: &mut String, prefix: char, line: &[u8], is_last_of_file: bool| {
        out.push(prefix);
        let text = String::from_utf8_lossy(line);
        if let Some(stripped) = text.strip_suffix('\n') {
            out.push_str(stripped);
            out.push('\n');
        } else {
            out.push_str(&text);
            out.push('\n');
            if is_last_of_file {
                out.push_str("\\ No newline at end of file\n");
            }
        }
    };

    let mut out = String::new();
    out.push_str(&format!("--- a/{rel}\n"));
    out.push_str(&format!("+++ b/{rel}\n"));

    let mut line_delta: isize = 0; // new_line - old_line, accumulated
    for hunk in hunks {
        let l0 = hunk.first().expect("non-empty hunk").l0;
        let l1 = hunk.last().expect("non-empty hunk").l1;
        let ctx_start = l0.saturating_sub(DIFF_CONTEXT);
        let ctx_end = (l1 + DIFF_CONTEXT).min(old_lines.len().saturating_sub(1));

        let mut body = String::new();
        let mut removed_total = 0usize;
        let mut added_total = 0usize;
        let mut pos = ctx_start;
        for run in &hunk {
            for (i, line) in old_lines[pos..run.l0].iter().enumerate() {
                let idx = pos + i;
                push_line(&mut body, ' ', line, idx + 1 == old_lines.len());
            }
            for (i, line) in old_lines[run.l0..=run.l1].iter().enumerate() {
                let idx = run.l0 + i;
                push_line(&mut body, '-', line, idx + 1 == old_lines.len());
            }
            removed_total += run.l1 - run.l0 + 1;

            let old_block_start = starts[run.l0];
            let old_block_end = if run.l1 + 1 < starts.len() {
                starts[run.l1 + 1]
            } else {
                old.len()
            };
            let new_block_start = (old_block_start as isize + deltas[run.first]) as usize;
            let last = &sorted[run.last];
            let delta_after_last =
                deltas[run.last] + last.replacement.len() as isize - last.length as isize;
            let new_block_end = (old_block_end as isize + delta_after_last) as usize;
            let new_l0 = line_index_of(&new_starts, new_block_start);
            let new_block_lines = split_lines(&new[new_block_start..new_block_end]);
            for (i, line) in new_block_lines.iter().enumerate() {
                let idx = new_l0 + i;
                push_line(&mut body, '+', line, idx + 1 == new_lines.len());
            }
            added_total += new_block_lines.len();
            pos = run.l1 + 1;
        }
        for (i, line) in old_lines[pos..=ctx_end].iter().enumerate() {
            let idx = pos + i;
            push_line(&mut body, ' ', line, idx + 1 == old_lines.len());
        }

        let old_count = ctx_end - ctx_start + 1;
        let new_count = old_count - removed_total + added_total;
        let new_hunk_start = (ctx_start as isize + line_delta) as usize;
        out.push_str(&format!(
            "@@ -{},{} +{},{} @@\n",
            ctx_start + 1,
            old_count,
            new_hunk_start + 1,
            new_count
        ));
        out.push_str(&body);
        line_delta += added_total as isize - removed_total as isize;
    }
    out
}

// ---------------------------------------------------------------------------
// Affected-package derivation (PUBLIC — so a restore build
// can reuse it).
// ---------------------------------------------------------------------------

/// Extract `<name>…</name>` from a `package.xml` document. Pure.
///
/// XML comments are stripped FIRST — package.xml files routinely carry
/// boilerplate comment headers, and a `<name>` inside `<!-- … -->` must
/// never win over the real element (it would misdirect the affected-package
/// derivation and the automatic colcon build). Deliberately not a full XML
/// parser (no new dependency for one element); the residual is a `<name>`
/// inside a CDATA section, which no real package.xml carries.
pub fn package_name_from_xml(xml: &str) -> Option<String> {
    let mut stripped = String::with_capacity(xml.len());
    let mut rest = xml;
    while let Some(open) = rest.find("<!--") {
        stripped.push_str(&rest[..open]);
        match rest[open..].find("-->") {
            Some(close) => rest = &rest[open + close + "-->".len()..],
            None => {
                rest = ""; // unterminated comment swallows the tail
            }
        }
    }
    stripped.push_str(rest);
    // The name is the direct child of
    // `<package>`, not the first `<name>` anywhere — a nested
    // `<export><name>…</name></export>` (or any custom element) placed
    // before the package's own name would otherwise redirect the affected-package
    // derivation and the automatic colcon target. A focused scan over the
    // stripped text: find the root `<package …>` open tag, then walk its
    // children tracking depth, and take the `<name>` seen at depth 0.
    let pkg_open = stripped
        .match_indices("<package")
        .find(|(i, _)| {
            stripped[i + "<package".len()..]
                .chars()
                .next()
                .is_some_and(|c| c == '>' || c.is_whitespace())
        })
        .map(|(i, _)| i)?;
    let pkg_close = stripped[pkg_open..].find('>')? + pkg_open;
    if stripped[..=pkg_close].ends_with("/>") {
        return None; // `<package/>` has no children
    }
    let mut pos = pkg_close + 1;
    let mut depth = 0usize;
    loop {
        let lt = stripped[pos..].find('<')? + pos;
        let rest = &stripped[lt..];
        if rest.starts_with("</") {
            if depth == 0 {
                return None; // `</package>` before any direct <name>
            }
            depth -= 1;
            pos = rest.find('>')? + lt + 1;
        } else if rest.starts_with("<?") || rest.starts_with("<!") {
            pos = rest.find('>')? + lt + 1;
        } else {
            let gt = rest.find('>')?;
            let tag = &rest[1..gt];
            let self_closing = tag.trim_end().ends_with('/');
            let tag_name = tag
                .trim_end_matches('/')
                .split(|c: char| c.is_whitespace())
                .next()
                .unwrap_or("");
            pos = lt + gt + 1;
            if self_closing {
                continue;
            }
            if depth == 0 && tag_name == "name" {
                let close = stripped[pos..].find("</name>")?;
                let name = stripped[pos..pos + close].trim();
                return if name.is_empty() {
                    None
                } else {
                    Some(name.to_string())
                };
            }
            depth += 1;
        }
    }
}

/// Walk up from `file` to the nearest directory holding a `package.xml`,
/// stopping at `stop` (inclusive). Returns the package directory.
pub fn nearest_package_root(file: &Path, stop: &Path) -> Option<PathBuf> {
    let mut dir = file.parent()?;
    loop {
        if dir.join("package.xml").is_file() {
            return Some(dir.to_path_buf());
        }
        if dir == stop {
            return None;
        }
        dir = dir.parent()?;
    }
}

/// The ament package names owning `files` (sorted, deduped). Files outside
/// any package are reported under the error — an edit that cannot be attributed to
/// a package cannot be rebuilt, which the caller must surface loudly.
pub fn derive_affected_packages(ws_src: &Path, files: &[PathBuf]) -> CliResult<Vec<String>> {
    let mut names: Vec<String> = Vec::new();
    for f in files {
        let Some(pkg_dir) = nearest_package_root(f, ws_src) else {
            return Err(CliError::Validation(format!(
                "'{}' is not inside any ament package (no package.xml found \
                 walking up to the workspace src root) — cannot derive the \
                 affected package set",
                f.display()
            )));
        };
        let pkg_xml = pkg_dir.join("package.xml");
        let xml = read_regular_bounded_to_string(&pkg_xml).map_err(|e| {
            CliError::Validation(format!("cannot read '{}': {e}", pkg_xml.display()))
        })?;
        let Some(name) = package_name_from_xml(&xml) else {
            return Err(CliError::Validation(format!(
                "'{}' has no <name> element",
                pkg_dir.join("package.xml").display()
            )));
        };
        if !names.contains(&name) {
            names.push(name);
        }
    }
    names.sort();
    Ok(names)
}

/// Package name for one file, tolerating files outside any package (rclpy
/// scan uses this — a stray script is reported package-less, not an error).
pub fn package_of(ws_src: &Path, file: &Path) -> Option<String> {
    let pkg_dir = nearest_package_root(file, ws_src)?;
    let xml = read_regular_bounded_to_string(&pkg_dir.join("package.xml")).ok()?;
    package_name_from_xml(&xml)
}

// ---------------------------------------------------------------------------
// rclpy scan (report-only — rclpy has no loaned-message API upstream).
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct RclpyNode {
    pub package: Option<String>,
    /// Workspace-relative path.
    pub file: String,
    pub publishers: usize,
}

/// Pure classification of one Python source: is it an rclpy node, and how
/// many `create_publisher(` call sites does it carry?
pub fn classify_rclpy_source(text: &str) -> Option<usize> {
    let is_rclpy = text.lines().any(|l| {
        let t = l.trim_start();
        t.starts_with("import rclpy") || t.starts_with("from rclpy")
    });
    if !is_rclpy {
        return None;
    }
    Some(text.matches("create_publisher(").count())
}

fn walk_py_files(dir: &Path, depth: usize, out: &mut Vec<PathBuf>) {
    // Symlinked directories are SKIPPED (a link to an ancestor recurses
    // forever) and the depth is capped as belt-and-braces against exotic
    // non-symlink cycles (bind mounts).
    const MAX_WALK_DEPTH: usize = 64;
    if depth > MAX_WALK_DEPTH {
        return;
    }
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    let mut entries: Vec<_> = entries.flatten().collect();
    entries.sort_by_key(|e| e.file_name());
    for e in entries {
        let path = e.path();
        let name = e.file_name();
        let name = name.to_string_lossy();
        let is_symlink = e.file_type().map(|t| t.is_symlink()).unwrap_or(true);
        if path.is_dir() {
            if is_symlink || name == ".git" || name == "build" || name == "install" || name == "log"
            {
                continue;
            }
            walk_py_files(&path, depth + 1, out);
        } else if !is_symlink
            && e.file_type().map(|t| t.is_file()).unwrap_or(false)
            && name.ends_with(".py")
        {
            // Only regular, non-symlink files
            // are enqueued — `scan_rclpy` reads each with `read_to_string`,
            // and a writerless FIFO (or a symlink to one) named `x.py`
            // would block the migration indefinitely.
            out.push(path);
        }
    }
}

/// Scan `<ws>/src` for rclpy nodes. Deterministic (sorted walk).
pub fn scan_rclpy(ws: &Path, ws_src: &Path) -> Vec<RclpyNode> {
    let mut files = Vec::new();
    walk_py_files(ws_src, 0, &mut files);
    let mut nodes = Vec::new();
    for f in files {
        // The walker type-checked this pathname; the read must
        // still be the one that refuses a non-regular object, because the
        // two are separate syscalls on a path a workspace writer controls.
        let Ok(text) = read_regular_bounded_to_string(&f) else {
            continue;
        };
        if let Some(publishers) = classify_rclpy_source(&text) {
            nodes.push(RclpyNode {
                package: package_of(ws_src, &f),
                file: rel_to(ws, &f),
                publishers,
            });
        }
    }
    nodes
}

// ---------------------------------------------------------------------------
// Git.
// ---------------------------------------------------------------------------

/// One `git status --porcelain=v1 -z` entry. `path` is the CURRENT-tree
/// path; a rename/copy entry also carries the original path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PorcelainEntry {
    /// The two status characters (e.g. `??`, ` M`, `R `).
    pub code: String,
    pub path: String,
    /// Rename/copy source path (the second NUL field).
    pub orig_path: Option<String>,
}

impl PorcelainEntry {
    pub fn is_untracked(&self) -> bool {
        self.code == "??"
    }
}

/// Parse NUL-delimited `git status --porcelain=v1 -z` output. `-z` is
/// load-bearing, not cosmetic: the newline porcelain C-QUOTES an exotic
/// pathname (a quote, a newline, non-ASCII under `core.quotePath`), so a
/// display-form comparison MISSES that file — an untracked `talk"er.cpp`
/// the migration would edit sailed past the write gate, and reverting the
/// migration commit would then delete the user's original file. In `-z`
/// output paths are verbatim bytes.
pub fn parse_porcelain_z(bytes: &[u8]) -> Vec<PorcelainEntry> {
    let mut entries = Vec::new();
    let mut fields = bytes.split(|b| *b == 0);
    while let Some(field) = fields.next() {
        if field.len() < 4 || field[2] != b' ' {
            continue; // empty trailing field / malformed
        }
        let code = String::from_utf8_lossy(&field[..2]).into_owned();
        let path = String::from_utf8_lossy(&field[3..]).into_owned();
        // A rename/copy entry is followed by the ORIGINAL path as its own
        // NUL field.
        let orig_path = if code.contains('R') || code.contains('C') {
            fields
                .next()
                .filter(|f| !f.is_empty())
                .map(|f| String::from_utf8_lossy(f).into_owned())
        } else {
            None
        };
        entries.push(PorcelainEntry {
            code,
            path,
            orig_path,
        });
    }
    entries
}

/// Any TRACKED change in the parsed status (what the manifest's `dirty`
/// flag and the write gate key on).
pub fn any_tracked_change(entries: &[PorcelainEntry]) -> bool {
    entries.iter().any(|e| !e.is_untracked())
}

/// Everything the verb needs to know about the workspace's git state.
#[derive(Debug, Clone, Default)]
pub struct GitInfo {
    /// `git rev-parse --show-toplevel`, when the workspace is in a repo.
    pub toplevel: Option<PathBuf>,
    /// HEAD commit, when one exists.
    pub head: Option<String>,
    /// Parsed `git status --porcelain=v1 -z` entries (empty when not a
    /// repo, and empty when the status command FAILED — see
    /// `status_error`).
    pub status: Vec<PorcelainEntry>,
    /// Why `git status` failed, when it
    /// did. A failed status must not read as an EMPTY — clean — status, or
    /// `--write` could pass the clean-tree gate and commit over
    /// uncommitted tracked changes; the write gate refuses closed and
    /// the manifest records `dirty: true` while this is `Some`.
    pub status_error: Option<String>,
}

/// The environment policy
/// EVERY git child the verb spawns runs under. `-C <dir>` does NOT
/// neutralize an inherited `GIT_DIR` / `GIT_WORK_TREE` / `GIT_INDEX_FILE`:
/// with one of those set, status/add/commit target the redirected
/// repository while the verb's own source writes land in the requested
/// workspace — a "successful" migration whose commit lives somewhere else.
/// The policy is an explicit KEEP list: only what shapes WHICH config and
/// identity git uses passes through — the config file selectors
/// (`GIT_CONFIG_GLOBAL` / `SYSTEM` / `NOSYSTEM`), the runtime config
/// injection (`GIT_CONFIG_COUNT` / `KEY_n` / `VALUE_n`,
/// `GIT_CONFIG_PARAMETERS`), `GIT_ATTR_NOSYSTEM`, and the author/committer
/// identity (`GIT_AUTHOR_*` / `GIT_COMMITTER_*`). Every other `GIT_*` is
/// removed: the repository-redirection family (`GIT_DIR`, `GIT_WORK_TREE`,
/// `GIT_INDEX_FILE`, `GIT_OBJECT_DIRECTORY`,
/// `GIT_ALTERNATE_OBJECT_DIRECTORIES`, `GIT_COMMON_DIR`, `GIT_NAMESPACE`,
/// `GIT_CEILING_DIRECTORIES`, ...), the program-substitution family
/// (`GIT_EXEC_PATH`, `GIT_TEMPLATE_DIR`, `GIT_SSH_COMMAND`, `GIT_ASKPASS`,
/// `GIT_EDITOR`, `GIT_PAGER`, `GIT_EXTERNAL_DIFF`, ...), and any variable a
/// future git adds — a keep list fails closed. Hooks and the user's config
/// FILES are untouched: those are repository/config state, not
/// environment, and honoring them is product behavior the e2e suite pins.
pub fn git_env_var_kept(name: &str) -> bool {
    matches!(
        name,
        "GIT_CONFIG_GLOBAL"
            | "GIT_CONFIG_SYSTEM"
            | "GIT_CONFIG_NOSYSTEM"
            | "GIT_CONFIG_COUNT"
            | "GIT_CONFIG_PARAMETERS"
            | "GIT_ATTR_NOSYSTEM"
            | "GIT_AUTHOR_NAME"
            | "GIT_AUTHOR_EMAIL"
            | "GIT_AUTHOR_DATE"
            | "GIT_COMMITTER_NAME"
            | "GIT_COMMITTER_EMAIL"
            | "GIT_COMMITTER_DATE"
    ) || name.starts_with("GIT_CONFIG_KEY_")
        || name.starts_with("GIT_CONFIG_VALUE_")
}

/// A `git -C <dir>` child under the policy above. EVERY git the verb spawns
/// is built here — the write path's `git_run`, the identity probes, the
/// unstage probe, `gather_git_info`; a git `Command` constructed anywhere
/// else in this module is a regression (pinned by a source-count test).
fn git_command(dir: &Path) -> Command {
    let mut cmd = Command::new("git");
    cmd.arg("-C").arg(dir);
    for (key, _) in std::env::vars_os() {
        let name = key.to_string_lossy();
        if name.starts_with("GIT_") && !git_env_var_kept(&name) {
            cmd.env_remove(&key);
        }
    }
    cmd
}

pub fn gather_git_info(ws: &Path) -> GitInfo {
    let run = |args: &[&str]| -> Option<Vec<u8>> {
        let out = git_command(ws).args(args).output().ok()?;
        if !out.status.success() {
            return None;
        }
        Some(out.stdout)
    };
    let text = |v: Vec<u8>| String::from_utf8_lossy(&v).trim_end().to_string();
    let toplevel = run(&["rev-parse", "--show-toplevel"])
        .map(text)
        .map(PathBuf::from);
    let head = if toplevel.is_some() {
        run(&["rev-parse", "HEAD"]).map(text)
    } else {
        None
    };
    // `--untracked-files=all` is load-bearing — git's default collapses
    // an entirely untracked directory to ONE `?? dir/` entry, which
    // matches no planned file path, so a planned file inside an untracked
    // package would sail past the write gate: the source would be committed and a
    // `git revert` of the migration would then DELETE the user's original
    // file. Enumerated individually, the planned path matches and the
    // gate refuses (no committed baseline, no one-commit undo).
    let (status, status_error) = if toplevel.is_some() {
        match git_command(ws)
            .args(["status", "--porcelain=v1", "-z", "--untracked-files=all"])
            .output()
        {
            Ok(out) if out.status.success() => (parse_porcelain_z(&out.stdout), None),
            Ok(out) => (
                Vec::new(),
                Some(String::from_utf8_lossy(&out.stderr).trim().to_string()),
            ),
            Err(e) => (Vec::new(), Some(e.to_string())),
        }
    } else {
        (Vec::new(), None)
    };
    GitInfo {
        toplevel,
        head,
        status,
        status_error,
    }
}

/// The `--write` gate verdict over the parsed status.
///
/// Dirty = any TRACKED change — those are what a `git revert` could clobber
/// or tangle with. Untracked files are tolerated (a colcon workspace
/// routinely carries untracked `build/`/`install/` trees), EXCEPT an
/// untracked file the migration itself would edit: that file has no
/// committed baseline, so the one-commit undo contract cannot cover it.
#[derive(Debug, PartialEq, Eq)]
pub enum DirtyVerdict {
    Clean,
    Dirty {
        tracked: Vec<String>,
        untracked_planned: Vec<String>,
    },
}

pub fn classify_dirty(entries: &[PorcelainEntry], planned_rel_files: &[String]) -> DirtyVerdict {
    let mut tracked = Vec::new();
    let mut untracked_planned = Vec::new();
    for e in entries {
        if e.is_untracked() {
            if planned_rel_files.iter().any(|p| p == &e.path) {
                untracked_planned.push(e.path.clone());
            }
        } else {
            tracked.push(e.path.clone());
        }
    }
    if tracked.is_empty() && untracked_planned.is_empty() {
        DirtyVerdict::Clean
    } else {
        DirtyVerdict::Dirty {
            tracked,
            untracked_planned,
        }
    }
}

/// What the write path probes before the migration commit: the configured
/// identity (`git config user.name` / `user.email`, any scope) plus the
/// identity environment variables, each `None` when unset or blank.
#[derive(Debug, Clone, Copy, Default)]
pub struct IdentityProbe<'a> {
    pub config_user_name: Option<&'a str>,
    pub config_user_email: Option<&'a str>,
    pub env_email: Option<&'a str>,
    pub env_author_name: Option<&'a str>,
    pub env_author_email: Option<&'a str>,
    pub env_committer_name: Option<&'a str>,
    pub env_committer_email: Option<&'a str>,
}

/// Which SIDES of the migration commit's identity must carry the tool's
/// `-c` fallback (`user.name=cerulion ros2 migrate` /
/// `user.email=ros2-migrate@cerulion.invalid`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IdentityFallback {
    pub name: bool,
    pub email: bool,
}

/// The per-side fallback decision. PURE — the write path feeds it
/// [`IdentityProbe`].
///
/// The axis is CONFIGURED identity, not RESOLVABLE identity: with nothing
/// configured, git auto-detects `user@hostname` — which DIES under strict
/// ident on a domainless host (the container failure this fixes) and,
/// worse, SUCCEEDS on hosts with a plausible domain, baking a fabricated
/// address into the user's repository. Either way an unconfigured side
/// gets the tool identity, which names the generator.
///
/// The EMAIL side counts as configured when `user.email` is set in any
/// config scope, when `EMAIL` is set (git's documented fallback, which
/// `-c user.email` would wrongly outrank), or when BOTH per-side env
/// addresses are set. The NAME side counts as configured when `user.name`
/// is set in any config scope or BOTH per-side env names are set.
/// Treating
/// any configured email as a complete identity would be wrong: a repository with
/// `user.email` but no `user.name` would skip the fallback and git would REFUSE
/// the commit (`user.useConfigOnly`, or an empty gecos on a container),
/// forcing a rollback for nothing. Each side is decided ALONE and the
/// fallback is passed ONLY for the incomplete side, so a configured email
/// is never displaced by the tool's. One-sided env identity still takes
/// the fallback for that side — env outranks `-c` config for the side it
/// sets, and the fallback fills the other side that would otherwise
/// refuse the commit.
pub fn commit_identity_fallback(p: &IdentityProbe<'_>) -> IdentityFallback {
    fn set(v: Option<&str>) -> bool {
        v.is_some_and(|s| !s.trim().is_empty())
    }
    let email_configured = set(p.config_user_email)
        || set(p.env_email)
        || (set(p.env_author_email) && set(p.env_committer_email));
    let name_configured =
        set(p.config_user_name) || (set(p.env_author_name) && set(p.env_committer_name));
    IdentityFallback {
        name: !name_configured,
        email: !email_configured,
    }
}

// ---------------------------------------------------------------------------
// Manifest.
// ---------------------------------------------------------------------------

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ManifestWorkspaceKey {
    /// HEAD commit, `null` when the workspace is not a git repository (a
    /// consumer treats `null` as never-matching — no offer caching).
    pub head: Option<String>,
    /// Any TRACKED change present when the manifest was written.
    pub dirty: bool,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ManifestCounts {
    pub packages: usize,
    pub call_sites: usize,
    pub manual_candidates: usize,
    pub rclpy_nodes: usize,
}

#[derive(Debug, Serialize)]
pub struct ManifestCandidate {
    pub package: Option<String>,
    pub file: String,
    pub line: u64,
    pub kind: String,
}

#[derive(Debug, Serialize)]
pub struct ManifestManualCandidate {
    pub package: Option<String>,
    pub file: String,
    pub line: u64,
    pub function: String,
    pub reason: String,
    pub detail: String,
}

/// The machine-readable dry-run product. `decision` belongs to the CONSUMER
/// (the launcher offer): this module writes `null` on first write and
/// preserves whatever is there verbatim on every refresh.
#[derive(Debug, Serialize)]
pub struct MigrateManifest {
    pub schema: &'static str,
    pub version: u32,
    pub workspace_key: ManifestWorkspaceKey,
    /// "dry-run" | "write".
    pub generated_by: String,
    pub counts: ManifestCounts,
    pub candidates: Vec<ManifestCandidate>,
    pub manual_candidates: Vec<ManifestManualCandidate>,
    pub rclpy: Vec<RclpyNode>,
    pub decision: serde_json::Value,
}

/// The paths currently STAGED (index vs HEAD), optionally restricted to
/// `only`. The commit-scope check reads BOTH of its lists through
/// this one query — the full staged set and the subset restricted to this
/// run's own planned paths — so the two are in the same relativity and can
/// be compared directly, without the check having to know whether a planned
/// path is spelled relative to the workspace or to the repository toplevel.
/// `-z` because a pathname may contain anything but NUL, and the default
/// output would quote and escape such a name into something that matches
/// nothing.
fn staged_paths<F>(git_run: &F, only: &[&str]) -> CliResult<Vec<String>>
where
    F: Fn(&[&str]) -> CliResult<String>,
{
    let mut args: Vec<&str> = vec!["diff", "--cached", "--name-only", "-z"];
    if !only.is_empty() {
        args.push("--");
        args.extend(only.iter().copied());
    }
    Ok(git_run(&args)?
        .split('\0')
        .filter(|s| !s.is_empty())
        .map(str::to_string)
        .collect())
}

/// Read the `decision` slot of an existing manifest, if any.
///
/// Takes the WORKSPACE ROOT, not the manifest pathname — the manifest is
/// located under it at [`MANIFEST_REL_PATH`] by the same anchored walk that
/// writes it. (A caller passing the manifest path instead gets `Null`, because
/// the anchor open fails `ENOTDIR`.)
///
/// The read is
/// performed by the SAME anchored, per-component `O_NOFOLLOW` walk that
/// [`write_manifest_atomic`] writes through, not by a `read_to_string` of
/// the joined pathname. Two things are wrong with a pathname read, and
/// both land BEFORE the writer's advertised symlink refusal could speak:
/// a symlinked `.cerulion` (or manifest) is FOLLOWED, so the decision
/// slot can be lifted from a file outside the workspace and copied into
/// the manifest this run writes; and a symlink to a writerless FIFO
/// blocks every dry-run forever. The anchored open refuses a linked
/// component, opens `O_NONBLOCK`, and gates on an `fstat` of the opened
/// descriptor — so read and write agree about what the manifest is,
/// and neither follows a link. An absent, unreadable or refused manifest
/// is still `Null` (there is simply no prior decision to carry).
pub fn existing_decision(ws: &Path) -> serde_json::Value {
    let Ok(anchor) = anchored::Anchor::new(ws) else {
        return serde_json::Value::Null;
    };
    let Ok(mut f) = anchored::open_for_restore(&anchor, Path::new(MANIFEST_REL_PATH)) else {
        return serde_json::Value::Null;
    };
    let Ok((bytes, _)) = f.read_all() else {
        return serde_json::Value::Null;
    };
    let Ok(text) = String::from_utf8(bytes) else {
        return serde_json::Value::Null;
    };
    let Ok(value) = serde_json::from_str::<serde_json::Value>(&text) else {
        return serde_json::Value::Null;
    };
    value
        .get("decision")
        .cloned()
        .unwrap_or(serde_json::Value::Null)
}

/// Atomic (temp + rename) manifest write. Failures are returned — the
/// caller decides loudness (a dry-run's manifest failure must not mask the
/// report).
pub fn write_manifest_atomic(ws: &Path, manifest: &MigrateManifest) -> CliResult<PathBuf> {
    write_manifest_atomic_with(ws, manifest, &mut fresh_manifest_temp_name)
}

/// The per-attempt temporary basename for
/// the manifest write — `.<name>.tmp.<pid>.<seq>.<nonce>`. The old
/// `.<name>.tmp-<pid>` was PREDICTABLE and PID-keyed: a temp left by a
/// killed process whose pid a later run reused, or a file an actor
/// pre-created at that name, hit the `O_EXCL` create and every manifest
/// refresh failed until someone removed it. The nonce makes the candidate
/// unguessable and distinct per attempt; the write loop retries past an
/// occupied candidate (never touching the occupant — it is not ours).
fn fresh_manifest_temp_name(file_name: &str, _attempt: usize) -> String {
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let seq = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    format!(
        ".{file_name}.tmp.{pid}.{seq}.{nonce:016x}",
        pid = std::process::id(),
        nonce = fresh_nonce(seq)
    )
}

/// Bounded attempts at a free temporary name — a stale temp from a killed
/// run or a planted file occupies ONE candidate, never the refresh.
const MANIFEST_TEMP_ATTEMPTS: usize = 16;

/// The manifest write with its temp-name MINTER injected (production:
/// `fresh_manifest_temp_name`; the unit oracles inject occupied names).
fn write_manifest_atomic_with(
    ws: &Path,
    manifest: &MigrateManifest,
    mint: &mut dyn FnMut(&str, usize) -> String,
) -> CliResult<PathBuf> {
    let path = ws.join(MANIFEST_REL_PATH);
    let rel = Path::new(MANIFEST_REL_PATH);
    let rel_dir = rel
        .parent()
        .expect("manifest path has a parent by construction");
    let file_name = rel.file_name().expect("manifest path has a file name");
    let json = serde_json::to_string_pretty(manifest)
        .map_err(|e| CliError::Validation(format!("manifest render failed: {e}")))?;
    // A `create_dir_all` + `fs::write` would follow
    // a pre-existing (or swapped-in) `.cerulion` symlink, so a dry-run could
    // land its manifest — and overwrite a file — OUTSIDE the workspace. The
    // directory is ensured and the temp+rename sequence performed
    // RELATIVE TO DESCRIPTORS opened `O_NOFOLLOW` per component from the
    // workspace root (the same anchored walker `--write` uses): a symlinked
    // `.cerulion` is refused, never followed, and a swap after the open
    // cannot redirect the write or the rename.
    let anchor = anchored::Anchor::new(ws).map_err(|e| {
        CliError::Validation(format!(
            "cannot open the workspace '{}' for the manifest write: {e}",
            ws.display()
        ))
    })?;
    anchored::ensure_dir(&anchor, rel_dir).map_err(|e| manifest_dir_error(ws, rel_dir, &e))?;
    let file_name_str = file_name.to_string_lossy();
    let mut last_occupied: Option<String> = None;
    for attempt in 0..MANIFEST_TEMP_ATTEMPTS {
        let tmp = mint(&file_name_str, attempt);
        match anchored::write_replace(
            &anchor,
            rel_dir,
            std::ffi::OsStr::new(&tmp),
            file_name,
            json.as_bytes(),
        ) {
            Ok(()) => return Ok(path),
            // An occupied candidate (a stale temp of a killed run, a planted
            // file) is left exactly as found — `O_EXCL` never opened it —
            // and the next candidate is tried.
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
                last_occupied = Some(tmp);
            }
            Err(e) => return Err(manifest_dir_error(ws, rel_dir, &e)),
        }
    }
    Err(CliError::Validation(format!(
        "could not find a free temporary name for the manifest under '{}' \
         after {MANIFEST_TEMP_ATTEMPTS} attempts (last occupied: '{}') — stale \
         temporaries from killed runs or planted files occupy every \
         candidate; remove them and re-run",
        ws.join(rel_dir).display(),
        last_occupied.unwrap_or_default()
    )))
}

fn manifest_dir_error(ws: &Path, rel_dir: &Path, e: &std::io::Error) -> CliError {
    // The refusal itself came from the descriptor-level `O_NOFOLLOW` open
    // (ELOOP on Linux; macOS reports ENOTDIR for a symlink under
    // `O_DIRECTORY|O_NOFOLLOW`) — this `symlink_metadata` only chooses the
    // WORDING, and is never followed.
    let is_link = anchored::is_eloop(e)
        || std::fs::symlink_metadata(ws.join(rel_dir))
            .map(|m| m.file_type().is_symlink())
            .unwrap_or(false);
    if is_link {
        CliError::Validation(format!(
            "'{}' is a SYMLINK — the manifest is written only inside the \
             workspace and a link is never followed; move it aside and re-run",
            ws.join(rel_dir).display()
        ))
    } else {
        CliError::Validation(format!(
            "cannot write the manifest under '{}': {e} — it must be a real \
             directory inside the workspace",
            ws.join(rel_dir).display()
        ))
    }
}

// ---------------------------------------------------------------------------
// Engine seam.
// ---------------------------------------------------------------------------

/// The analysis engine — the clang tool in production, fixture JSON in
/// tests.
pub trait MigrateEngine {
    /// One line naming the engine for the report header. Must be stable for
    /// a given installation (the dry-run report is byte-deterministic).
    fn describe(&self) -> String;
    /// Analyse one translation unit; returns the tool's raw JSON stdout.
    fn analyze_tu(
        &mut self,
        compile_db_dir: &Path,
        src_root: &Path,
        tu: &Path,
    ) -> CliResult<String>;
}

/// Production engine: spawns the clang tool per TU.
pub struct ClangToolEngine {
    pub tool: PathBuf,
}

impl MigrateEngine for ClangToolEngine {
    fn describe(&self) -> String {
        self.tool.display().to_string()
    }

    fn analyze_tu(
        &mut self,
        compile_db_dir: &Path,
        src_root: &Path,
        tu: &Path,
    ) -> CliResult<String> {
        let out = Command::new(&self.tool)
            .arg("-p")
            .arg(compile_db_dir)
            .arg(format!("--src-root={}", src_root.display()))
            .arg(tu)
            .output()
            .map_err(|e| {
                CliError::Validation(format!(
                    "failed to run the migration engine '{}': {e}",
                    self.tool.display()
                ))
            })?;
        if !out.status.success() {
            let stderr = String::from_utf8_lossy(&out.stderr);
            let tail: String = stderr
                .lines()
                .rev()
                .take(8)
                .collect::<Vec<_>>()
                .into_iter()
                .rev()
                .collect::<Vec<_>>()
                .join("\n");
            return Err(CliError::Validation(format!(
                "migration engine failed on '{}' ({}). Engine stderr tail:\n{}",
                tu.display(),
                out.status,
                tail
            )));
        }
        Ok(String::from_utf8_lossy(&out.stdout).into_owned())
    }
}

/// Resolve the engine binary: `$CERULION_ROS2_MIGRATE_TOOL` (must exist when
/// set), then beside the current executable, then `PATH`. `Err` carries the
/// full remediation — the verb REFUSES rather than shipping an inert
/// analysis.
pub fn resolve_engine_binary() -> Result<PathBuf, String> {
    if let Ok(explicit) = std::env::var(TOOL_ENV) {
        let p = PathBuf::from(&explicit);
        if p.is_file() {
            return Ok(p);
        }
        return Err(format!(
            "{TOOL_ENV} points at '{explicit}' but no such file exists — \
             fix the path or unset the variable"
        ));
    }
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            let sibling = dir.join(TOOL_BIN);
            if sibling.is_file() {
                return Ok(sibling);
            }
        }
    }
    if let Some(paths) = std::env::var_os("PATH") {
        for dir in std::env::split_paths(&paths) {
            let cand = dir.join(TOOL_BIN);
            if cand.is_file() {
                return Ok(cand);
            }
        }
    }
    Err(format!(
        "migration engine not built: '{TOOL_BIN}' was not found beside the \
         cerulion binary, on PATH, or via {TOOL_ENV}. It is built from a checkout of \
         the Cerulion source tree, in the ROS 2 toolchain container (see \
         tools/ros2_migrate/README.md in that tree):\n\
         \x20 docker build -t ros2-bench tools/ros2_toolchain/\n\
         \x20 docker run --rm -v \"$PWD\":/work ros2-bench bash -lc \
         'cd /work/tools/ros2_migrate && ./build.sh'\n\
         then place '{TOOL_BIN}' beside the cerulion binary, on PATH, or \
         point {TOOL_ENV} at it."
    ))
}

// ---------------------------------------------------------------------------
// compile_commands.json discovery + TU enumeration.
// ---------------------------------------------------------------------------

/// A `compile_commands.json` entry. This is a FOREIGN schema — CMake/clang
/// own it, and real entries legitimately carry keys this reader ignores
/// (`output`, `arguments`, …) — so it deliberately does NOT
/// `deny_unknown_fields` (denying would refuse every real compile database)
/// and is classified `ExemptWithReason` in `config_deny_unknown_fields_test`.
#[derive(Debug, Deserialize)]
struct CompileCommandEntry {
    directory: String,
    file: String,
}

/// Per-package compile databases under `<ws>/build/*/compile_commands.json`
/// (sorted); falls back to a colcon-merged `<ws>/build/compile_commands.json`
/// only when no per-package database exists.
pub fn find_compile_dbs(ws: &Path) -> Vec<PathBuf> {
    let build = ws.join("build");
    let mut dbs = Vec::new();
    if let Ok(entries) = std::fs::read_dir(&build) {
        let mut dirs: Vec<_> = entries.flatten().map(|e| e.path()).collect();
        dirs.sort();
        for d in dirs {
            let cc = d.join("compile_commands.json");
            if d.is_dir() && cc.is_file() {
                dbs.push(cc);
            }
        }
    }
    if dbs.is_empty() {
        let merged = build.join("compile_commands.json");
        if merged.is_file() {
            dbs.push(merged);
        }
    }
    dbs
}

/// C++ translation units under `src_root` named by one compile database.
/// Pure over the JSON text.
/// Lexically normalize `.` and `..` components (no filesystem access — this
/// module's compile-database reader is pure over the JSON text). `..` at the
/// root stays at the root, matching how the OS resolves it.
pub fn lexically_normalize(p: &Path) -> PathBuf {
    use std::path::Component;
    let mut out = PathBuf::new();
    for c in p.components() {
        match c {
            Component::CurDir => {}
            Component::ParentDir => {
                if !out.pop() {
                    // `..` above the root of a relative path: keep it (the
                    // caller only containment-checks absolute paths, where
                    // pop() at the root is a no-op by the arm below).
                    out.push(Component::ParentDir);
                }
            }
            other => out.push(other),
        }
    }
    out
}

pub fn tus_for_db(db_json: &str, src_root: &Path) -> CliResult<Vec<PathBuf>> {
    let entries: Vec<CompileCommandEntry> = serde_json::from_str(db_json)
        .map_err(|e| CliError::Validation(format!("unparseable compile_commands.json: {e}")))?;
    let mut tus = Vec::new();
    for e in entries {
        let file = PathBuf::from(&e.file);
        // The CMake contract: a relative `file` resolves against the
        // entry's `directory` — and the JOINED path must be lexically
        // normalized before the containment check, because real databases
        // carry entries like `../../src/pkg/node.cpp` and `starts_with` is
        // LEXICAL: an un-normalized `..` path never matches src_root and
        // the TU is silently dropped. (Symlink escapes are caught later by
        // build_plan's canonical containment gate — this stage is pure.)
        let abs = lexically_normalize(&if file.is_absolute() {
            file
        } else {
            PathBuf::from(&e.directory).join(file)
        });
        // Best-effort filesystem resolution: with a symlinked src/ (the
        // colcon-overlay layout) the database may carry the SPELLED path
        // while src_root is canonical — resolve when the file exists;
        // a non-existent path (the pure unit vectors included) keeps its
        // lexical form.
        let abs = abs.canonicalize().unwrap_or(abs);
        let is_cpp = abs
            .extension()
            .and_then(|x| x.to_str())
            .map(|x| matches!(x, "cpp" | "cc" | "cxx"))
            .unwrap_or(false);
        if is_cpp && abs.starts_with(src_root) && !tus.contains(&abs) {
            tus.push(abs);
        }
    }
    tus.sort();
    Ok(tus)
}

// ---------------------------------------------------------------------------
// The migration plan (shared by dry-run and --write — ONE code path).
// ---------------------------------------------------------------------------

/// One file's application product.
#[derive(Debug)]
pub struct PlannedFile {
    /// Absolute path.
    pub path: PathBuf,
    /// Diff/patch path (relative to the git toplevel when in a repo, else to
    /// the workspace root).
    pub rel: String,
    /// Path relative to the CANONICAL SRC ROOT (all-Normal components by
    /// construction) — the write phase and rollback address the file
    /// through the anchored per-component `O_NOFOLLOW` walker with this,
    /// never by re-deriving from the absolute spelling.
    pub src_rel: PathBuf,
    /// The bytes the plan was built FROM — `--write` re-verifies these on
    /// disk immediately before writing (an edit made while reviewing the
    /// diff must refuse, not be overwritten), and a failed commit restores
    /// them.
    pub old_bytes: Vec<u8>,
    pub new_bytes: Vec<u8>,
    /// Source-swap TOCTOU: the [`OccupantIdentity`]
    /// of the exact file `old_bytes` were read from — captured off the SAME
    /// descriptor as the bytes. The `--write` TOCTOU gate requires identity
    /// match AND byte match: bytes alone cannot prove the file at the path
    /// is the file that was reviewed (a different regular file with
    /// identical contents atomically renamed over the path passes the byte
    /// check, so generated output would land in a file consent never covered —
    /// the same class the patch install guards against). Private:
    /// identity is an in-crate write-gate concern, not plan surface.
    old_identity: OccupantIdentity,
}

#[derive(Debug, Default)]
pub struct MigrationPlan {
    /// The full unified diff — printed by dry-run, written verbatim as the
    /// patch file by `--write`.
    pub diff_text: String,
    pub files: Vec<PlannedFile>,
    /// Sorted ament package names owning the edited files.
    pub affected_packages: Vec<String>,
}

fn rel_to(base: &Path, path: &Path) -> String {
    match path.strip_prefix(base) {
        Ok(p) => p.to_string_lossy().into_owned(),
        Err(_) => path.to_string_lossy().into_owned(),
    }
}

/// Build the plan from the merged analysis: read each edited file, verify +
/// apply the edits, render per-file diffs (sorted by relative path).
pub fn build_plan(
    ws_src: &Path,
    rel_base: &Path,
    analysis: &MergedAnalysis,
) -> CliResult<MigrationPlan> {
    let mut edits_by_file: BTreeMap<String, Vec<ToolEdit>> = BTreeMap::new();
    for rw in &analysis.rewrites {
        edits_by_file
            .entry(rw.file.clone())
            .or_default()
            .extend(rw.edits.iter().cloned());
    }
    let mut plan = MigrationPlan::default();
    let mut abs_files = Vec::new();
    for (file, edits) in &edits_by_file {
        let path = PathBuf::from(file);
        // Containment gate: the engine binary CONTROLS these paths, and the
        // verb writes to them under --write — an analysis naming a file
        // outside the workspace source tree (malformed engine, compromised
        // engine, symlinked source escaping src/) must refuse loudly, never
        // rewrite an arbitrary readable file. Canonicalization also
        // resolves symlinks, so a link inside src/ pointing outside is
        // refused too.
        let canonical = path.canonicalize().map_err(|e| {
            CliError::Validation(format!(
                "the analysis names '{}' but it cannot be resolved: {e}",
                path.display()
            ))
        })?;
        if !canonical.starts_with(ws_src) {
            return Err(CliError::Validation(format!(
                "the analysis names '{}' (resolves to '{}'), which is \
                 OUTSIDE the workspace source tree '{}' — refusing. The \
                 migration only ever edits files under src/.",
                path.display(),
                canonical.display(),
                ws_src.display()
            )));
        }
        // All I/O (and the diff/patch header) uses the CANONICAL path from
        // here on — a symlinked spelling inside src/ would otherwise be
        // FOLLOWED at write time through whatever it points at, while the
        // canonical path is the real file the containment gate just
        // approved.
        let path = canonical;
        // Workspace-root-relative address for the anchored write/rollback
        // walker. ws_src is canonical, so its parent is the canonical
        // workspace root and the strip is infallible given the containment
        // gate above — asserted rather than assumed.
        // Anchored-walker address: relative to the CANONICAL SRC ROOT (the
        // walker's pinned anchor). Infallible given the containment gate
        // above — asserted rather than assumed.
        let src_rel = path
            .strip_prefix(ws_src)
            .map_err(|_| {
                CliError::Validation(format!(
                    "'{}' is not under the workspace src root '{}'",
                    path.display(),
                    ws_src.display()
                ))
            })?
            .to_path_buf();
        // Bounded + type-gated — an analysis naming a FIFO (or a
        // source swapped for one) must refuse loudly, never wedge the read.
        // The identity rides the SAME fd as the bytes, so the
        // --write TOCTOU gate can later prove it is looking at the file
        // that was reviewed, not an identical-bytes replacement.
        let (old, old_identity) = match read_regular_bounded_with_identity(&path) {
            Ok(pair) => pair,
            Err(e) if e.kind() == std::io::ErrorKind::Unsupported => {
                return Err(CliError::Validation(format!(
                    "the analysis names '{}', which is not a regular file — \
                     refusing.",
                    path.display()
                )));
            }
            Err(e) => return Err(e.into()),
        };
        let new_bytes = apply_edits(&old, edits)?;
        let rel = rel_to(rel_base, &path);
        plan.diff_text
            .push_str(&unified_diff(&rel, &old, &new_bytes, edits));
        abs_files.push(path.clone());
        plan.files.push(PlannedFile {
            path,
            rel,
            src_rel,
            old_bytes: old,
            new_bytes,
            old_identity,
        });
    }
    plan.affected_packages = if abs_files.is_empty() {
        Vec::new()
    } else {
        derive_affected_packages(ws_src, &abs_files)?
    };
    Ok(plan)
}

/// Container-harness seam (`examples/ros2_migrate_apply.rs`, run_matrix.sh):
/// write a built plan's files in place through the SAME anchored
/// per-component `O_NOFOLLOW` walker the verb's `--write` path uses. The
/// caller reaches this only via `build_plan`, so every path has already
/// passed the canonicalization + containment gate — the harness cannot be
/// steered outside `ws_src` by a crafted analysis JSON (the hazard:
/// an example using raw `fs::read`/`fs::write` on engine-reported paths).
/// No rollback/consent/git — the matrix restores sources with git; a
/// failure is loud and names the file.
///
/// The write rides the same
/// identity+content VERIFIED seam as `--write` (`rewrite_verified`) — the
/// plan carries each file's plan-time identity and reviewed bytes, so a
/// source changed or swapped after planning REFUSES here exactly as the
/// production path refuses it. A truncate-at-open write here would be
/// the very TOCTOU class the main seam closes, alive at a second call
/// site; there is no unverified write seam to call (no `anchored::write`
/// or `write_phased`): ONE write discipline.
pub fn apply_plan_in_place(ws_src: &Path, plan: &MigrationPlan) -> CliResult<()> {
    let anchor = anchored::Anchor::new(ws_src)
        .map_err(|e| CliError::Validation(format!("open src root '{}': {e}", ws_src.display())))?;
    for f in &plan.files {
        anchored::rewrite_verified(
            &anchor,
            &f.src_rel,
            &f.old_identity,
            &f.old_bytes,
            &f.new_bytes,
        )
        .map_err(|(_, e)| CliError::Validation(format!("write {}: {e}", f.path.display())))?;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Report rendering.
// ---------------------------------------------------------------------------

/// Header facts for the report (bundled — the analysis/plan/rclpy carry the
/// body).
struct ReportMeta<'a> {
    tu_count: usize,
    db_count: usize,
    engine: &'a str,
    write_mode: bool,
}

fn render_report(
    ws: &Path,
    analysis: &MergedAnalysis,
    plan: &MigrationPlan,
    rclpy: &[RclpyNode],
    meta: &ReportMeta<'_>,
) -> String {
    let ReportMeta {
        tu_count,
        db_count,
        engine,
        write_mode,
    } = *meta;
    let mut out = String::new();
    out.push_str("cerulion ros2 migrate — loaned-message API migration\n");
    out.push_str(&format!("workspace: {}\n", ws.display()));
    out.push_str(&format!(
        "translation units analysed: {tu_count} (compile databases: {db_count})\n"
    ));
    out.push_str(&format!("engine: {engine}\n\n"));

    out.push_str(&format!(
        "PROPOSED REWRITES ({} call site(s) across {} package(s))\n",
        analysis.rewrites.len(),
        plan.affected_packages.len()
    ));
    out.push_str("=================================================\n");
    if plan.diff_text.is_empty() {
        out.push_str("none — no publish call site was proven safe to rewrite.\n");
    } else {
        out.push_str(&plan.diff_text);
    }
    out.push('\n');

    out.push_str(&format!(
        "MANUAL CANDIDATES ({} site(s) — NOT rewritten; each with its reason)\n",
        analysis.candidates.len()
    ));
    out.push_str("=================================================\n");
    if analysis.candidates.is_empty() {
        out.push_str("none.\n");
    } else {
        for c in &analysis.candidates {
            let rel = rel_to(ws, Path::new(&c.file));
            out.push_str(&format!(
                "{}:{}  {}\n    {}: {}\n",
                rel, c.line, c.function, c.reason, c.detail
            ));
        }
    }
    out.push('\n');

    out.push_str(&format!(
        "RCLPY NODES ({} — report-only; rclpy has no loaned-message API \
         upstream)\n",
        rclpy.len()
    ));
    out.push_str("=================================================\n");
    if rclpy.is_empty() {
        out.push_str("none.\n");
    } else {
        for n in rclpy {
            let pkg = n.package.as_deref().unwrap_or("(no package)");
            out.push_str(&format!(
                "{}  {}  ({} create_publisher call site(s))\n",
                pkg, n.file, n.publishers
            ));
        }
        out.push_str(
            "rclpy keeps one copy per side (two copies become one under the \
             launcher's heap hook). No Python file was or will be modified.\n",
        );
    }
    out.push('\n');

    if !write_mode {
        out.push_str(&format!(
            "dry-run: nothing was modified (manifest refreshed at {}).\n\
             Re-run with --write to apply: requires a clean git tree, writes \
             ONE commit plus {}, then runs `colcon build --packages-select \
             <affected>` automatically.\n",
            MANIFEST_REL_PATH, PATCH_FILENAME
        ));
    }
    out
}

// ---------------------------------------------------------------------------
// The migration driver.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct MigrateOptions {
    pub workspace: PathBuf,
    pub write: bool,
    pub assume_yes: bool,
}

/// What happened. `Applied.build_error` carries the loud colcon-failure text
/// (with the revert path) — the CLI prints it to stderr and exits nonzero.
#[derive(Debug)]
pub enum MigrateOutcome {
    /// Dry-run: report printed, manifest refreshed.
    Reported,
    /// The interactive confirm was answered "no" — nothing written.
    Declined,
    /// `--write` with nothing provable: report printed, nothing applied.
    NothingToApply,
    Applied {
        commit: String,
        affected_packages: Vec<String>,
        build_error: Option<String>,
    },
}

/// Injected effects: the engine, the consent seam, and the colcon runner.
pub struct MigrateDeps<'a> {
    pub engine: &'a mut dyn MigrateEngine,
    pub is_tty: bool,
    pub confirm: &'a mut dyn FnMut(&str) -> CliResult<bool>,
    /// Runs `colcon build --packages-select <packages>` in the workspace;
    /// returns whether the build succeeded. Injected so desk tests never
    /// need colcon.
    pub build_runner: &'a mut dyn FnMut(&Path, &[String]) -> CliResult<bool>,
    pub out: &'a mut dyn std::io::Write,
    /// Interrupt observation (INTERRUPT_SAFETY):
    /// the CLI installs a Ctrl-C handler that flips a ONE-SHOT flag instead
    /// of terminating, and the run polls this in two phases. BEFORE CONSENT —
    /// at the top of every per-TU analysis iteration (and when the engine
    /// FAILS with the flag set, since a terminal Ctrl-C kills the engine
    /// child too), as the first act of `--write` before any of its gates,
    /// and after a `yes` — a trip REFUSES having written nothing: the flag
    /// cannot be reset from here, and a pre-consent Ctrl-C would otherwise surface
    /// only as "interrupted" from the first write safepoint after the user's
    /// yes. Under `--yes` there is no prompt, so the pre-write-batch poll is
    /// the one consent-boundary read on that path. INSIDE THE WRITE WINDOW —
    /// while WAITING for the workspace lock (contended runs only — an
    /// uncontended acquire never calls it), then before the first write,
    /// between file writes, and before the commit — a trip during the lock
    /// wait refuses having taken no lock, and a trip at a safepoint rolls
    /// back everything written and refuses. After a successful commit an
    /// interrupt changes nothing (the commit IS the durable state).
    ///
    /// The per-TU poll is MODE-AGNOSTIC: a dry run polls it too. The CLI
    /// passes a latch that is never true for a dry run, so in production
    /// that poll is inert there; a caller that wires a live latch to a dry
    /// run gets the same analysis refusal from the per-TU poll, while a trip
    /// after the last TU is not seen by a dry run, which refreshes its
    /// manifest as usual.
    pub interrupted: &'a dyn Fn() -> bool,
}

/// The pre-write safepoint's refusal — the first consumer of the interrupt
/// latch INSIDE the write window. Exported so the tests that pin the consent
/// boundary can assert this exact string is ABSENT from
/// every pre-consent refusal, and one arm can assert it is PRODUCED at its
/// own site, without a hand-copied literal that a reword would silently
/// disarm.
pub const PRE_WRITE_INTERRUPT_REFUSAL: &str =
    "interrupted — nothing was written; nothing was migrated.";

/// Where a PRE-CONSENT interrupt was honoured. Every
/// window makes the same claim about the tree — before consent, `--write`
/// has written nothing — so that claim has ONE owner, and the set of
/// windows is enumerable here rather than spread over four literals.
enum PreConsentWindow {
    /// The per-TU poll at the top of an analysis iteration, or an engine
    /// failure observed with the latch tripped; `analysed` TUs completed.
    Analysis { analysed: usize },
    /// The first thing `--write` does, before any of its gates, on the
    /// PROMPTED path: consent has not been asked yet.
    BeforeConsent,
    /// The same poll under `--yes`, where the command line WAS the consent:
    /// the refusal must not describe a prompt the user never saw.
    BeforeWriteBatch,
    /// After a `yes` answered a prompt that was waiting when the interrupt
    /// arrived (or that was about to be asked — the few in-memory gates
    /// between the write head and the prompt are inside this window too).
    AfterYes,
}

fn interrupted_before_consent(window: PreConsentWindow) -> CliError {
    const TREE: &str = "no source file was rewritten and nothing was committed";
    CliError::Validation(match window {
        PreConsentWindow::Analysis { analysed } => format!(
            "interrupted — analysis stopped after {analysed} translation \
             unit(s); the migration was not applied: {TREE}. Re-run to \
             analyse again."
        ),
        PreConsentWindow::BeforeConsent => format!(
            "interrupted before consent — the migration was not applied: \
             {TREE}. Re-run to review and apply."
        ),
        PreConsentWindow::BeforeWriteBatch => format!(
            "interrupted before the write batch — the migration was not \
             applied: {TREE}. Re-run to apply."
        ),
        PreConsentWindow::AfterYes => format!(
            "interrupted while the consent prompt was waiting — the answer \
             was not applied because the interrupt was requested first: \
             {TREE}. Re-run and answer again to apply."
        ),
    })
}

/// How long an engine failure waits for the interrupt flag before it is
/// reported as the engine's own failure, polled in
/// [`ENGINE_FAILURE_LATCH_SLICE`]s. A terminal Ctrl-C is delivered to this
/// process DIRECTLY, in the same instant it reaches the engine child — but
/// the `ctrlc` closure that flips the flag runs on its OWN thread, while
/// `Command::output()` returns the child's death to THIS one, so the death
/// can be OBSERVED a scheduling quantum before the flip lands. The wait is
/// paid only on the engine-failure path, where a genuine crash is a terminal
/// error and a quarter second is nothing; a flip that has already landed
/// costs no wait at all.
const ENGINE_FAILURE_LATCH_GRACE: std::time::Duration = std::time::Duration::from_millis(250);
const ENGINE_FAILURE_LATCH_SLICE: std::time::Duration = std::time::Duration::from_millis(5);

/// The latch, read at once and then re-read every
/// [`ENGINE_FAILURE_LATCH_SLICE`] until it is true or
/// [`ENGINE_FAILURE_LATCH_GRACE`] has elapsed. Only for the engine-failure
/// path: everywhere else the poll is the first thing to run after the
/// signal, and no observation races the flip. The predicate is read
/// REPEATEDLY here. A flip that lands after the grace is reported as the
/// engine failure — nothing was written either way.
fn interrupted_with_grace(interrupted: &dyn Fn() -> bool) -> bool {
    let deadline = std::time::Instant::now() + ENGINE_FAILURE_LATCH_GRACE;
    loop {
        if interrupted() {
            return true;
        }
        if std::time::Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(ENGINE_FAILURE_LATCH_SLICE);
    }
}

/// The production colcon runner (stdio inherited — the user sees the build).
pub fn run_colcon_build(ws: &Path, packages: &[String]) -> CliResult<bool> {
    let status = Command::new("colcon")
        .arg("build")
        .arg("--packages-select")
        .args(packages)
        .current_dir(ws)
        .status()
        .map_err(|e| {
            CliError::Validation(format!(
                "failed to run `colcon build --packages-select {}`: {e} — is \
                 colcon on PATH?",
                packages.join(" ")
            ))
        })?;
    Ok(status.success())
}

/// ANCHORED source-file I/O: every operation walks from the PINNED anchor
/// (the canonical src root's directory fd, opened once at run entry) to the
/// target with per-component `openat(O_NOFOLLOW)` (Unix), so NO path
/// component — final file OR intermediate directory — can be a symlink at
/// operation time, and the anchor's own identity cannot change mid-run: a
/// swap of the src spelling changes what the PATH names, never what the
/// held fd IS. A final-only `O_NOFOLLOW` open protects the last component
/// but still FOLLOWS a swapped parent directory, which redirects the write
/// to an external tree — the same TOCTOU one directory level up.
///
/// Trust boundary: the anchor is the CANONICAL src root, resolved and
/// pinned at run start (a symlinked `src/` is the user's own overlay
/// layout, resolved once at entry); its parents are never walked.
/// Everything under it is component-checked. The in-place
/// `O_WRONLY|O_TRUNC` final open preserves the file's mode/inode and
/// honors its own write protection (a read-only source fails EACCES
/// exactly like `fs::write` would). Non-Unix has no `openat`; the
/// fallbacks are plain path-joined operations (the reported attack is
/// Unix-shaped).
#[cfg(unix)]
mod anchored {
    use std::ffi::{CString, OsStr};
    use std::io;
    use std::os::fd::{AsRawFd as _, FromRawFd as _, OwnedFd};
    use std::os::unix::ffi::OsStrExt as _;
    use std::path::{Component, Path};

    /// The pinned anchor: held for the whole run, CLOEXEC (engine
    /// subprocesses never inherit it). Every walk starts at this fd —
    /// never at a path re-open.
    pub struct Anchor {
        fd: OwnedFd,
    }

    impl Anchor {
        /// The root is
        /// bound WITHOUT following a replacement link, and the object the
        /// descriptor actually names is verified.
        ///
        /// Every caller hands a root that was `canonicalize`d at entry, so
        /// by construction its final component is not a symlink — which is
        /// exactly why `O_NOFOLLOW` cannot refuse a legitimate root (a
        /// user's symlinked `src/` overlay is already resolved by then) and
        /// why a link present HERE can only be one that arrived after the
        /// canonicalization. An open that follows it is unsafe: since every
        /// anchored write walks from this fd, one swapped component
        /// redirects the whole run's writes outside the workspace while
        /// the pin is still claimed.
        ///
        /// Two gates, because `O_NOFOLLOW` alone only rules out a link:
        /// `lstat` the root, open it `O_NOFOLLOW|O_DIRECTORY`, then `fstat`
        /// the descriptor and require the same `(dev, ino)` — a directory
        /// swapped for a DIFFERENT directory in that window is refused too.
        /// RESIDUAL: a swap that lands before the `lstat` binds a
        /// consistent-but-different object; no check inside this
        /// constructor can see that, because the caller's canonical
        /// pathname is the only thing it is given. What it does guarantee
        /// is that the fd every later write walks from is a real directory
        /// at the requested path, reached without traversing a link.
        pub fn new(root: &Path) -> io::Result<Self> {
            let c = cstr(root.as_os_str())?;
            let before = lstat_identity(&c)?;
            let fd = unsafe {
                libc::open(
                    c.as_ptr(),
                    libc::O_RDONLY | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC,
                )
            };
            if fd < 0 {
                return Err(io::Error::last_os_error());
            }
            let fd = unsafe { OwnedFd::from_raw_fd(fd) };
            let opened = identity_of(fd.as_raw_fd()).ok_or_else(|| {
                io::Error::other("fstat failed on the anchor root — cannot establish its identity")
            })?;
            if opened != before {
                return Err(io::Error::other(
                    "the anchor root was replaced while it was being opened \
                     (identity mismatch) — refusing to write through it",
                ));
            }
            Ok(Self { fd })
        }
    }

    /// `(st_dev, st_ino)` of the object a path names WITHOUT following a
    /// final symlink — the pre-open half of the anchor's identity gate.
    #[allow(clippy::unnecessary_cast)]
    fn lstat_identity(c: &CString) -> io::Result<FileIdentity> {
        let mut st = unsafe { std::mem::zeroed::<libc::stat>() };
        if unsafe { libc::lstat(c.as_ptr(), &mut st) } != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(FileIdentity {
            dev: st.st_dev as u64,
            ino: st.st_ino as u64,
        })
    }

    fn cstr(os: &OsStr) -> io::Result<CString> {
        CString::new(os.as_bytes())
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "NUL byte in path component"))
    }

    /// (keep-alive for the last intermediate fd, raw parent dirfd, final
    /// name) — every intermediate component opened `O_DIRECTORY|O_NOFOLLOW`
    /// RELATIVE to the previous fd, starting at the pinned anchor.
    fn walk_parent<'r>(
        anchor: &Anchor,
        rel: &'r Path,
    ) -> io::Result<(Option<OwnedFd>, libc::c_int, &'r OsStr)> {
        let comps: Vec<Component<'r>> = rel.components().collect();
        if comps.is_empty() || comps.iter().any(|c| !matches!(c, Component::Normal(_))) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "anchored path must be a non-empty, strictly-descending relative path",
            ));
        }
        let mut held: Option<OwnedFd> = None;
        for comp in &comps[..comps.len() - 1] {
            let Component::Normal(name) = comp else {
                unreachable!("validated above");
            };
            let base = held
                .as_ref()
                .map(|f| f.as_raw_fd())
                .unwrap_or(anchor.fd.as_raw_fd());
            let c = cstr(name)?;
            let fd = unsafe {
                libc::openat(
                    base,
                    c.as_ptr(),
                    libc::O_RDONLY | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC,
                )
            };
            if fd < 0 {
                return Err(io::Error::last_os_error());
            }
            held = Some(unsafe { OwnedFd::from_raw_fd(fd) });
        }
        let Component::Normal(last) = comps[comps.len() - 1] else {
            unreachable!("validated above");
        };
        let parent = held
            .as_ref()
            .map(|f| f.as_raw_fd())
            .unwrap_or(anchor.fd.as_raw_fd());
        Ok((held, parent, last))
    }

    /// Stable identity of an open file — `(st_dev, st_ino)` off `fstat`.
    /// The torn-restore hazard: a write
    /// that failed AFTER its `O_TRUNC` open truncated *some* file must name
    /// WHICH file it truncated, because an atomic replace between that open
    /// and the rollback's read hands the same PATH to another actor's file
    /// — restoring the snapshot there erases foreign work. The identity is
    /// taken from the fd itself (never a path re-stat), so it names the
    /// exact inode the migration modified.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct FileIdentity {
        dev: u64,
        ino: u64,
    }

    // st_dev/st_ino widths differ across libc targets (i32/u64 on macOS,
    // u64/u64 on Linux) — the cast is a no-op on some and required on
    // others, so the same-type-cast lint cannot be satisfied portably.
    #[allow(clippy::unnecessary_cast)]
    fn identity_of(fd: std::os::fd::RawFd) -> Option<FileIdentity> {
        let mut st: libc::stat = unsafe { std::mem::zeroed() };
        if unsafe { libc::fstat(fd, &mut st) } != 0 {
            return None;
        }
        Some(FileIdentity {
            dev: st.st_dev as u64,
            ino: st.st_ino as u64,
        })
    }

    /// The fd-bound REGULAR-FILE gate. Every open in this module
    /// carries `O_NONBLOCK` (a no-op for regular files; an immediate
    /// return for FIFOs — a writerless FIFO would wedge a blocking open
    /// forever) and then asks the FD what it opened:
    /// anything but `S_IFREG` is refused `ErrorKind::Unsupported` before a
    /// byte is read or written. fstat binds the answer to the object
    /// itself, so no path-level swap can slip a FIFO past a check.
    fn fd_is_regular(fd: std::os::fd::RawFd) -> bool {
        let mut st: libc::stat = unsafe { std::mem::zeroed() };
        if unsafe { libc::fstat(fd, &mut st) } != 0 {
            return false;
        }
        (st.st_mode & libc::S_IFMT) == libc::S_IFREG
    }

    fn not_regular_error() -> io::Error {
        io::Error::new(io::ErrorKind::Unsupported, "not a regular file")
    }

    /// `Unsupported` is minted ONLY by the regular-file gates in this
    /// module — the caller's preserve-and-report discriminator.
    pub fn is_not_regular(e: &io::Error) -> bool {
        e.kind() == io::ErrorKind::Unsupported
    }

    /// The payload inside the identity-mismatch refusal
    /// [`rewrite_verified`] mints — callers discriminate it via
    /// [`is_identity_mismatch`], the same way `Unsupported` is reserved
    /// for this module's regular-file gates.
    #[derive(Debug)]
    struct RewriteIdentityMismatch;

    impl std::fmt::Display for RewriteIdentityMismatch {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "the file at the path is not the reviewed file (identity mismatch)"
            )
        }
    }

    impl std::error::Error for RewriteIdentityMismatch {}

    /// True iff `e` is the identity-mismatch refusal minted by
    /// [`rewrite_verified`] — which refused BEFORE truncation, so the
    /// occupant was NOT modified.
    pub fn is_identity_mismatch(e: &io::Error) -> bool {
        e.get_ref()
            .is_some_and(|i| i.is::<RewriteIdentityMismatch>())
    }

    /// The companion to the pre-write identity gate:
    /// the identity-BOUND destructive write for the source-edit path.
    /// A truncate-at-open write
    /// modifies the occupant AT the open — so an identical-bytes
    /// replacement landing in the gate→write
    /// gap was truncated and rewritten before any check could run (the
    /// patch-install lesson, one seam later: verification must happen at the point
    /// of irreversible action). This opens the final component WITHOUT
    /// `O_TRUNC`, fstats the OPENED fd, compares its (dev, ino, mtime)
    /// against the identity the pre-write gate verified, and only on a
    /// match truncates and writes THROUGH THAT SAME fd — identity and
    /// write on one descriptor, no gap at all. A mismatch refuses with
    /// the occupant UNMODIFIED (`opened: None` phase; discriminate via
    /// [`is_identity_mismatch`]). Deliberately no `O_CREAT`: the reviewed
    /// file exists, so an absent occupant is a plain ENOENT refusal.
    /// The CONTENT binding rides the same descriptor: the
    /// stat identity is unforgeable for a different file, but an
    /// in-place swap on the SAME inode with a restored mtime passes it —
    /// the occupant's bytes are read back through this fd and must equal
    /// the reviewed bytes before anything is truncated. Both arms share
    /// this one shape (see the non-unix twin, where the byte binding is
    /// what collapses the forgeable mtime+len residual).
    // st_dev/st_ino/st_mtime widths differ across libc targets — see
    // `identity_of`.
    #[allow(clippy::unnecessary_cast)]
    pub fn rewrite_verified(
        anchor: &Anchor,
        rel: &Path,
        expected: &super::OccupantIdentity,
        expected_bytes: &[u8],
        bytes: &[u8],
    ) -> Result<(), (Option<FileIdentity>, io::Error)> {
        use std::io::{Read as _, Seek as _, Write as _};
        let (_held, parent, name) = match walk_parent(anchor, rel) {
            Ok(t) => t,
            Err(e) => return Err((None, e)),
        };
        let c = match cstr(name) {
            Ok(c) => c,
            Err(e) => return Err((None, e)),
        };
        // O_NONBLOCK: the FIFO discipline — an open of a
        // writerless FIFO could otherwise block forever. O_RDWR
        // rather than O_WRONLY: the content binding below
        // reads the occupant back through this same descriptor.
        let fd = unsafe {
            libc::openat(
                parent,
                c.as_ptr(),
                libc::O_RDWR | libc::O_NOFOLLOW | libc::O_CLOEXEC | libc::O_NONBLOCK,
            )
        };
        if fd < 0 {
            return Err((None, io::Error::last_os_error()));
        }
        let mut f = std::fs::File::from(unsafe { OwnedFd::from_raw_fd(fd) });
        if !fd_is_regular(f.as_raw_fd()) {
            return Err((None, not_regular_error()));
        }
        let mut st: libc::stat = unsafe { std::mem::zeroed() };
        if unsafe { libc::fstat(f.as_raw_fd(), &mut st) } != 0 {
            return Err((None, io::Error::last_os_error()));
        }
        let actual = super::OccupantIdentity {
            dev: st.st_dev as u64,
            ino: st.st_ino as u64,
            mtime: st.st_mtime as i64,
            mtime_nsec: st.st_mtime_nsec as i64,
        };
        if !super::write_seam_identity_holds(expected, &actual) {
            return Err((None, io::Error::other(RewriteIdentityMismatch)));
        }
        // Content binding through the same
        // descriptor, checked AFTER the identity gate (foreign files are
        // refused without reading them). The (dev, ino, mtime) identity
        // is unforgeable for a DIFFERENT file, but an in-place content
        // swap on the SAME inode with its mtime restored (utimensat)
        // passes it — only the bytes catch that. Migrate is not a hot
        // path; one extra read at write time is the whole cost.
        // The verification read is
        // BOUNDED by the reviewed length. The unix identity carries no
        // length, so a same-inode appender with a restored mtime could
        // otherwise feed a multi-GB occupant into an unbounded read
        // before the compare refuses. One byte past the reviewed size is
        // enough to prove divergence — a longer occupant is refused
        // without its tail ever being read or held in memory.
        let mut now_bytes = Vec::new();
        let cap = expected_bytes.len() as u64 + 1;
        if let Err(e) = (&mut f).take(cap).read_to_end(&mut now_bytes) {
            return Err((None, e));
        }
        if now_bytes != expected_bytes {
            return Err((None, io::Error::other(RewriteIdentityMismatch)));
        }
        // The read moved the cursor to EOF — rewind before the truncating
        // write, or the payload would land at the stale offset behind a
        // hole of zeros.
        if let Err(e) = f.seek(std::io::SeekFrom::Start(0)) {
            return Err((None, e));
        }
        // Only now is the occupant provably the reviewed file, content
        // included — truncate and write through the very fd the identity
        // and bytes came from.
        if unsafe { libc::ftruncate(f.as_raw_fd(), 0) } != 0 {
            // The open succeeded on the verified file; report the torn
            // phase (Some) — the torn contract — and let the
            // rollback's `torn_claim_holds` guard decide the restore.
            let opened = identity_of(f.as_raw_fd());
            return Err((opened, io::Error::last_os_error()));
        }
        let opened = identity_of(f.as_raw_fd());
        f.write_all(bytes).map_err(|e| (opened, e))
    }

    /// `O_NOFOLLOW` hitting a symlink reports ELOOP (Linux and macOS).
    pub fn is_eloop(e: &io::Error) -> bool {
        e.raw_os_error() == Some(libc::ELOOP)
    }

    /// The STRUCTURAL answer to the rollback-vs-concurrent-actor
    /// race family (each instance a fresh check-to-use gap): every destructive rollback act
    /// is bound to the very fd whose content it validated. The file is
    /// opened ONCE; content + identity are read off that fd; the pure
    /// disposition decides; a restore ftruncates and rewrites THROUGH THE
    /// SAME FD. An atomic rename landing after our open retargets the PATH,
    /// never this fd — a restore then lands on the orphaned inode we
    /// validated (harmless: it is no longer reachable by name) while the
    /// replacement at the path is untouched. There is no check-to-use gap
    /// because there is no re-open.
    pub struct RestoreFile {
        file: std::fs::File,
        /// False when `O_RDWR` was refused (EACCES on a read-only file) and
        /// the fallback `O_RDONLY` descriptor is held instead — the
        /// disposition can still run; a Restore verdict is reported, not
        /// performed.
        pub writable: bool,
    }

    pub fn open_for_restore(anchor: &Anchor, rel: &Path) -> io::Result<RestoreFile> {
        let (_held, parent, name) = walk_parent(anchor, rel)?;
        let c = cstr(name)?;
        // O_NONBLOCK: O_RDWR on a FIFO happens to return on
        // Linux/macOS, but the subsequent read would wait forever with no
        // EOF (this process holds the write end) — the flag makes the open
        // deterministic and the fd gate below refuses any non-regular
        // object BEFORE a read, so a planted FIFO can never wedge the
        // rollback. Regular files ignore O_NONBLOCK entirely.
        let open_with = |flags: libc::c_int| -> io::Result<std::fs::File> {
            let fd = unsafe { libc::openat(parent, c.as_ptr(), flags | libc::O_NONBLOCK) };
            if fd < 0 {
                return Err(io::Error::last_os_error());
            }
            let f = std::fs::File::from(unsafe { OwnedFd::from_raw_fd(fd) });
            if !fd_is_regular(f.as_raw_fd()) {
                return Err(not_regular_error());
            }
            Ok(f)
        };
        match open_with(libc::O_RDWR | libc::O_NOFOLLOW | libc::O_CLOEXEC) {
            Ok(file) => Ok(RestoreFile {
                file,
                writable: true,
            }),
            Err(e) if e.raw_os_error() == Some(libc::EACCES) => {
                let file = open_with(libc::O_RDONLY | libc::O_NOFOLLOW | libc::O_CLOEXEC)?;
                Ok(RestoreFile {
                    file,
                    writable: false,
                })
            }
            Err(e) => Err(e),
        }
    }

    impl RestoreFile {
        /// Bytes + identity off THIS descriptor — the same file the
        /// rewrite, if any, will land on.
        pub fn read_all(&mut self) -> io::Result<(Vec<u8>, FileIdentity)> {
            use std::io::Read as _;
            let identity = identity_of(self.file.as_raw_fd()).ok_or_else(|| {
                io::Error::other("fstat failed on an open fd — cannot establish file identity")
            })?;
            let mut out = Vec::new();
            self.file.read_to_end(&mut out)?;
            Ok((out, identity))
        }

        /// Truncate + rewrite through the validated descriptor — but ONLY
        /// after re-reading it and confirming the content is still exactly
        /// what the disposition validated (`expected`). The fd binding
        /// closes the rename-over race completely; it cannot stop another
        /// writer editing THE SAME INODE in place between validation and
        /// truncation, so the re-verify converts that shape into
        /// preserve-and-report (`Ok(false)` — the caller notes it).
        /// RESIDUAL: the re-verify narrows the in-place window to
        /// the adjacent-syscall gap between the confirming read and
        /// `set_len` — it does not close it; without mandatory locking
        /// (which no portable API offers, and which an uncooperating actor
        /// would not take anyway) content cannot be compare-and-swapped
        /// atomically. Stated rather than hidden.
        pub fn rewrite_validated(&mut self, expected: &[u8], bytes: &[u8]) -> io::Result<bool> {
            use std::io::{Read as _, Seek as _, Write as _};
            self.file.seek(std::io::SeekFrom::Start(0))?;
            let mut now = Vec::new();
            self.file.read_to_end(&mut now)?;
            if now != expected {
                return Ok(false);
            }
            self.file.seek(std::io::SeekFrom::Start(0))?;
            self.file.set_len(0)?;
            self.file.write_all(bytes)?;
            Ok(true)
        }
    }

    /// Create-only write for the rollback's vanished-file arm: `O_EXCL`
    /// cannot truncate or follow ANYTHING that appeared at the path
    /// meanwhile — recreation happens strictly into absence, or fails
    /// `AlreadyExists` for the caller to preserve-and-report.
    pub fn create_excl(anchor: &Anchor, rel: &Path, bytes: &[u8]) -> io::Result<()> {
        use std::io::Write as _;
        let (_held, parent, name) = walk_parent(anchor, rel)?;
        let c = cstr(name)?;
        let fd = unsafe {
            libc::openat(
                parent,
                c.as_ptr(),
                libc::O_WRONLY | libc::O_CREAT | libc::O_EXCL | libc::O_NOFOLLOW | libc::O_CLOEXEC,
                0o644 as libc::c_uint,
            )
        };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        let mut f = std::fs::File::from(unsafe { OwnedFd::from_raw_fd(fd) });
        f.write_all(bytes)
    }

    /// Ensure `rel` (strictly descending) is a REAL directory
    /// under the anchor — created if absent, refused if the name is
    /// occupied by anything else: the re-open is `O_DIRECTORY|O_NOFOLLOW`,
    /// so a symlink is never followed (ELOOP on Linux; ENOTDIR on macOS,
    /// which applies `O_DIRECTORY` first) and a file reports ENOTDIR.
    pub fn ensure_dir(anchor: &Anchor, rel: &Path) -> io::Result<()> {
        let (_held, parent, name) = walk_parent(anchor, rel)?;
        let c = cstr(name)?;
        if unsafe { libc::mkdirat(parent, c.as_ptr(), 0o755) } != 0 {
            let e = io::Error::last_os_error();
            if e.kind() != io::ErrorKind::AlreadyExists {
                return Err(e);
            }
        }
        let fd = unsafe {
            libc::openat(
                parent,
                c.as_ptr(),
                libc::O_RDONLY | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC,
            )
        };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        drop(unsafe { OwnedFd::from_raw_fd(fd) });
        Ok(())
    }

    /// Write `bytes` to `<dir_rel>/<final_name>` atomically — an
    /// `O_EXCL|O_NOFOLLOW` temp beside it, then `renameat` over the final
    /// name — with BOTH steps relative to ONE descriptor of the directory,
    /// opened `O_NOFOLLOW` per component from the anchor. A swap of the
    /// directory's pathname after the open cannot redirect the write or
    /// the rename.
    pub fn write_replace(
        anchor: &Anchor,
        dir_rel: &Path,
        tmp_name: &std::ffi::OsStr,
        final_name: &std::ffi::OsStr,
        bytes: &[u8],
    ) -> io::Result<()> {
        use std::io::Write as _;
        // The walker opens every component of `dir_rel` (the "-" tail is a
        // placeholder last component it never opens) and hands back the
        // directory's own descriptor as `parent`.
        let (_held, dirfd, _) = walk_parent(anchor, &dir_rel.join("-"))?;
        let tmp = cstr(tmp_name)?;
        let fin = cstr(final_name)?;
        let fd = unsafe {
            libc::openat(
                dirfd,
                tmp.as_ptr(),
                libc::O_WRONLY | libc::O_CREAT | libc::O_EXCL | libc::O_NOFOLLOW | libc::O_CLOEXEC,
                0o644 as libc::c_uint,
            )
        };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        let mut f = std::fs::File::from(unsafe { OwnedFd::from_raw_fd(fd) });
        let written = f.write_all(bytes).and_then(|()| f.sync_data());
        drop(f);
        let unlink_tmp = || unsafe { libc::unlinkat(dirfd, tmp.as_ptr(), 0) };
        if let Err(e) = written {
            let _ = unlink_tmp();
            return Err(e);
        }
        if unsafe { libc::renameat(dirfd, tmp.as_ptr(), dirfd, fin.as_ptr()) } != 0 {
            let e = io::Error::last_os_error();
            let _ = unlink_tmp();
            return Err(e);
        }
        Ok(())
    }
}

/// Non-Unix fallbacks: plain path-joined operations (no `openat`; the
/// reported attack is Unix-shaped).
#[cfg(not(unix))]
mod anchored {
    use std::io;
    use std::path::{Path, PathBuf};

    pub struct Anchor {
        root: PathBuf,
    }

    impl Anchor {
        pub fn new(root: &Path) -> io::Result<Self> {
            Ok(Self {
                root: root.to_path_buf(),
            })
        }
    }

    /// Non-Unix identity stub: no `fstat`-grade file identity is exposed
    /// here, so every identity compares EQUAL and the torn-restore check
    /// degrades to the weaker semantics (post-open failure = torn) — the
    /// same declared-weaker tier as `is_eloop` always-false on this arm.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct FileIdentity;

    fn not_regular_error() -> io::Error {
        io::Error::new(io::ErrorKind::Unsupported, "not a regular file")
    }

    pub fn is_not_regular(e: &io::Error) -> bool {
        e.kind() == io::ErrorKind::Unsupported
    }

    /// The payload inside this arm's identity-mismatch
    /// refusal — the twin of the Unix module's marker (each cfg module
    /// defines the full API).
    #[derive(Debug)]
    struct RewriteIdentityMismatch;

    impl std::fmt::Display for RewriteIdentityMismatch {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "the file at the path is not the reviewed file (identity mismatch)"
            )
        }
    }

    impl std::error::Error for RewriteIdentityMismatch {}

    /// Non-unix arm of the identity-bound rewrite. An arm that
    /// IGNORED the approved identity and
    /// delegated straight to a truncate-at-open write would leave the
    /// whole reviewed-identity chain — the pre-write gate AND the
    /// write-seam binding — with zero effect off Unix (an identical-bytes
    /// replacement truncated and
    /// rewritten with Applied reported). It mirrors the Unix shape
    /// at the strength the platform offers: open WITHOUT truncate (and
    /// without create — the reviewed file exists, absence is a real
    /// error), read the identity off the OPENED HANDLE
    /// (`File::metadata` is handle-bound on every platform; a path
    /// re-stat would re-open the very race this fn closes), compare via
    /// the shared [`super::write_seam_identity_holds`], and only on a
    /// match truncate (`set_len(0)`) and write THROUGH THAT SAME
    /// handle. The mtime+len identity alone is
    /// FORGEABLE — a replacement can keep the length and have its
    /// timestamp restored — so the occupant's BYTES are also read back
    /// through the same handle and must equal the reviewed bytes before
    /// anything is truncated. That collapses the residual: file IDENTITY
    /// remains unprovable off-Unix (the `(dev, ino)` fd identity and the
    /// per-component `O_NOFOLLOW` walk are Unix-only), but content
    /// DIVERGENCE is now impossible — a forge must be byte-identical to
    /// the reviewed source, and rewriting a byte-identical occupant
    /// produces exactly the consented output bytes, which is harmless in
    /// effect. Cost: one extra full read at write time — migrate is not
    /// a hot path. The path-level type gate keeps this arm's
    /// declared-weaker tier.
    pub fn rewrite_verified(
        anchor: &Anchor,
        rel: &Path,
        expected: &super::OccupantIdentity,
        expected_bytes: &[u8],
        bytes: &[u8],
    ) -> Result<(), (Option<FileIdentity>, io::Error)> {
        use std::io::{Read as _, Seek as _, Write as _};
        let path = anchor.root.join(rel);
        if let Ok(m) = std::fs::symlink_metadata(&path) {
            if !m.file_type().is_file() {
                return Err((None, not_regular_error()));
            }
        }
        let mut f = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(&path)
            .map_err(|e| (None, e))?;
        let meta = f.metadata().map_err(|e| (None, e))?;
        let actual = super::OccupantIdentity {
            // No mtime ⇒ no identity ⇒ REFUSE (the occupant is
            // unmodified — nothing was truncated), never a `None` the
            // equality gate would accept. See the capture-site twin in
            // `read_regular_bounded_with_identity`.
            mtime: meta.modified().map_err(|e| {
                (
                    None,
                    io::Error::other(format!(
                        "cannot verify the reviewed file's identity at the \
                         write: the filesystem reports no modification \
                         time ({e})"
                    )),
                )
            })?,
            len: meta.len(),
        };
        if !super::write_seam_identity_holds(expected, &actual) {
            return Err((None, io::Error::other(RewriteIdentityMismatch)));
        }
        // The content binding — checked AFTER the identity
        // gate (a wrong-length occupant is refused without being read).
        // Bounded like the unix arm. This tier's identity
        // carries the length, so a longer occupant was already refused
        // above without being read — the cap makes that ceiling
        // structural rather than an inference across two checks.
        let mut now_bytes = Vec::new();
        let cap = expected_bytes.len() as u64 + 1;
        (&mut f)
            .take(cap)
            .read_to_end(&mut now_bytes)
            .map_err(|e| (None, e))?;
        if now_bytes != expected_bytes {
            return Err((None, io::Error::other(RewriteIdentityMismatch)));
        }
        // The read moved the cursor to EOF — rewind before the truncating
        // write, or the payload would land at the stale offset behind a
        // hole of zeros.
        f.seek(std::io::SeekFrom::Start(0)).map_err(|e| (None, e))?;
        // Only now is the occupant — to this tier's strength, content
        // included — the reviewed file: truncate and write through the
        // very handle the identity and bytes came from. Post-truncate
        // failures take the torn phase (`Some`) — the torn
        // contract.
        f.set_len(0).map_err(|e| (Some(FileIdentity), e))?;
        f.write_all(bytes).map_err(|e| (Some(FileIdentity), e))
    }

    /// True iff `e` is the identity-mismatch refusal minted by this
    /// arm's [`rewrite_verified`] — which refused BEFORE truncation, so
    /// the occupant was NOT modified.
    pub fn is_identity_mismatch(e: &io::Error) -> bool {
        e.get_ref()
            .is_some_and(|i| i.is::<RewriteIdentityMismatch>())
    }

    pub fn is_eloop(_e: &io::Error) -> bool {
        false
    }

    /// Non-unix fallback of the single-fd restore handle — the same API,
    /// with the declared-weaker guarantees of this arm (no `openat` walk,
    /// unit identity).
    pub struct RestoreFile {
        file: std::fs::File,
        pub writable: bool,
    }

    pub fn open_for_restore(anchor: &Anchor, rel: &Path) -> io::Result<RestoreFile> {
        let path = anchor.root.join(rel);
        // Declared-weaker gate for this arm (path-level, no fd
        // binding on this fallback tier).
        if let Ok(m) = std::fs::symlink_metadata(&path) {
            if !m.file_type().is_file() {
                return Err(not_regular_error());
            }
        }
        match std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(&path)
        {
            Ok(file) => Ok(RestoreFile {
                file,
                writable: true,
            }),
            Err(e) if e.kind() == io::ErrorKind::PermissionDenied => {
                let file = std::fs::OpenOptions::new().read(true).open(&path)?;
                Ok(RestoreFile {
                    file,
                    writable: false,
                })
            }
            Err(e) => Err(e),
        }
    }

    impl RestoreFile {
        pub fn read_all(&mut self) -> io::Result<(Vec<u8>, FileIdentity)> {
            use std::io::Read as _;
            let mut out = Vec::new();
            self.file.read_to_end(&mut out)?;
            Ok((out, FileIdentity))
        }

        pub fn rewrite_validated(&mut self, expected: &[u8], bytes: &[u8]) -> io::Result<bool> {
            use std::io::{Read as _, Seek as _, Write as _};
            self.file.seek(std::io::SeekFrom::Start(0))?;
            let mut now = Vec::new();
            self.file.read_to_end(&mut now)?;
            if now != expected {
                return Ok(false);
            }
            self.file.seek(std::io::SeekFrom::Start(0))?;
            self.file.set_len(0)?;
            self.file.write_all(bytes)?;
            Ok(true)
        }
    }

    pub fn create_excl(anchor: &Anchor, rel: &Path, bytes: &[u8]) -> io::Result<()> {
        use std::io::Write as _;
        let mut f = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(anchor.root.join(rel))?;
        f.write_all(bytes)
    }

    /// Declared-weaker non-unix arm of the directory ensure: a
    /// `symlink_metadata` gate (never followed) then `create_dir`.
    pub fn ensure_dir(anchor: &Anchor, rel: &Path) -> io::Result<()> {
        let dir = anchor.root.join(rel);
        match std::fs::symlink_metadata(&dir) {
            Ok(m) if m.file_type().is_dir() => Ok(()),
            Ok(_) => Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "not a directory (a symlink or file occupies the name)",
            )),
            Err(e) if e.kind() == io::ErrorKind::NotFound => std::fs::create_dir(&dir),
            Err(e) => Err(e),
        }
    }

    /// Declared-weaker non-unix arm of the atomic replace: the
    /// directory is re-gated by `symlink_metadata` immediately before the
    /// create-new temp write and the rename (pathname operations).
    pub fn write_replace(
        anchor: &Anchor,
        dir_rel: &Path,
        tmp_name: &std::ffi::OsStr,
        final_name: &std::ffi::OsStr,
        bytes: &[u8],
    ) -> io::Result<()> {
        use std::io::Write as _;
        let dir = anchor.root.join(dir_rel);
        if !std::fs::symlink_metadata(&dir)?.file_type().is_dir() {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "not a directory",
            ));
        }
        let tmp = dir.join(tmp_name);
        let mut f = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&tmp)?;
        let written = f.write_all(bytes);
        drop(f);
        if let Err(e) = written {
            let _ = std::fs::remove_file(&tmp);
            return Err(e);
        }
        std::fs::rename(&tmp, dir.join(final_name)).inspect_err(|_| {
            let _ = std::fs::remove_file(&tmp);
        })
    }
}

/// The rollback's per-file decision, PURE (the ownership rule).
///
/// `torn` is true ONLY for the file whose own write failed AFTER the final
/// open succeeded — `O_TRUNC` truncates at open, so migration-owned partial
/// output provably exists on disk and restoring the snapshot is the heal —
/// AND whose current on-disk identity is still the very file that open
/// truncated (`torn_claim_holds`): an atomic replace between the truncating
/// open and the rollback's read hands the PATH to another actor's file, and
/// restoring the snapshot there erases foreign work (the shape: an
/// interposer failing the write post-open while a
/// rename swaps the file underneath). A write that failed BEFORE opening (a
/// planted symlink's ELOOP, a read-only EACCES, a swapped parent's ENOTDIR)
/// never modified the file, so foreign content there is another actor's
/// work: preserved and reported, never erased (granting `torn`
/// unconditionally to the failed-write file is real data loss).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RollbackDisposition {
    /// On-disk bytes are the pre-migration original — nothing to do.
    Skip,
    /// Provably the migration's own state (its bytes, or its torn write) —
    /// restore the snapshot.
    Restore,
    /// Another actor's content — preserve it and report.
    PreserveForeign,
}

pub fn rollback_disposition(
    current: &[u8],
    old: &[u8],
    new: &[u8],
    torn: bool,
) -> RollbackDisposition {
    if current == old {
        RollbackDisposition::Skip
    } else if current == new || torn {
        RollbackDisposition::Restore
    } else {
        RollbackDisposition::PreserveForeign
    }
}

/// The anchored walker's file identity, re-exported for the torn-claim
/// policy and its oracle tests.
pub use anchored::FileIdentity;

/// PURE: does the torn-write claim hold for the file at position `idx`,
/// whose rollback read observed `current` identity? It holds ONLY when the
/// failed write (a) was THIS file's (position match), (b) provably
/// truncated a file (the open succeeded — `reported` is `Some`), and
/// (c) that file is still the one at the path (identity match). A miss on
/// (c) is the post-open-replacement race: an atomic replace between the truncating open
/// and the rollback's read — the path now names another actor's file, and
/// the claim is WITHDRAWN so the disposition falls through to
/// preserve-and-report rather than erasing the replacement.
pub fn torn_claim_holds(
    reported: Option<&(usize, FileIdentity)>,
    idx: usize,
    current: &FileIdentity,
) -> bool {
    matches!(reported, Some((i, id)) if *i == idx && id == current)
}

/// Read a file that MUST be a regular file, without ever
/// blocking on the open or the read — the wedge class:
/// an actor swaps a path the migration will read for a WRITERLESS FIFO,
/// and a plain `fs::read`'s open blocks forever waiting for a writer, so
/// the run never returns and never reaches its rollback/cleanup. The open
/// carries `O_NONBLOCK` (a no-op for regular files; an immediate success
/// or `ENXIO` for FIFOs — never a wait) and the fd is `fstat`-gated to
/// `S_IFREG` BEFORE any read: anything else (FIFO, socket, device,
/// directory) fails `ErrorKind::Unsupported` without a byte read. Symlinks
/// are deliberately FOLLOWED (the callers' existing semantics — their
/// symlink defenses live at the write seam / their own metadata gates);
/// the type gate binds to the fd, so a swap after any caller-side check
/// still cannot wedge. Regular-file reads ignore `O_NONBLOCK` on every
/// unix, so the flag is not cleared afterwards.
fn read_regular_bounded(path: &Path) -> std::io::Result<Vec<u8>> {
    read_regular_bounded_with_identity(path).map(|(bytes, _)| bytes)
}

/// The one bounded read seam. It is
/// `read_to_string`'s exact contract — bytes as UTF-8, `InvalidData`
/// otherwise — over `read_regular_bounded` instead of a bare
/// `File::open`. EVERY read of a workspace-supplied file goes through
/// here. A plain `fs::read_to_string` opens the pathname and blocks: a
/// writerless FIFO (or a symlink to one) planted at any scanned path —
/// `package.xml`, a `.py` the walker already type-checked, the compile
/// database, the manifest — wedges the verb FOREVER with no timeout, and
/// the walker's own `file_type()` check cannot help because the object is
/// replaced between the check and the open. `read_regular_bounded` opens
/// `O_NONBLOCK` and gates on an `fstat` of the OPENED descriptor, so the
/// swap is a loud refusal instead of a hang, and it is refused on the
/// object actually opened rather than on a pathname re-check.
///
/// `pub` for ONE reason: the matrix's `ros2_migrate_apply` helper reads
/// prover output the same way and must not grow a second copy of this
/// policy.
pub fn read_regular_bounded_to_string(path: &Path) -> std::io::Result<String> {
    let bytes = read_regular_bounded(path)?;
    String::from_utf8(bytes).map_err(|_| {
        std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "stream did not contain valid UTF-8",
        )
    })
}

/// Ownership TOCTOU: the filesystem identity of
/// the file a snapshot's bytes were read from — `(st_dev, st_ino)` plus
/// mtime, taken by `fstat` off the SAME descriptor the bytes came
/// through, never a path re-stat. Byte equality alone cannot prove
/// ownership: another same-user process can create an identical-bytes
/// file at the patch path AFTER the preflight, and deleting it on a byte
/// match destroys that actor's file. Actual strength: inode numbers
/// RECYCLE, so `(dev, ino)` is strong evidence of "the same file", not
/// cryptographic proof — but a recycled inode would also have to carry
/// the identical mtime (seconds AND nanoseconds) and identical bytes,
/// and land inside the unpredictable-nonce quarantine capture, which
/// leaves no constructible shape of the attack.
#[cfg(unix)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct OccupantIdentity {
    dev: u64,
    ino: u64,
    mtime: i64,
    mtime_nsec: i64,
}

/// Declared-weaker non-unix arm (the same tier as this file's other
/// non-unix fallbacks): no `(dev, ino)` fd-bound identity is available
/// portably, so the identity carries what `std::fs::File::metadata` —
/// HANDLE-bound on every platform — can attest: the mtime and the byte
/// length. A UNIT here (every identity
/// comparing EQUAL) would make the pre-write gate and the
/// write-seam binding both VACUOUS off Unix — an identical-bytes
/// replacement truncated and rewritten while the command reported
/// Applied. Weaker tier means weaker evidence,
/// never zero enforcement: a replacement must also forge the
/// reviewed file's mtime AND length — and the write
/// seam additionally re-reads the occupant's BYTES through the same
/// handle, so a forge must be byte-identical to the reviewed source,
/// at which point the rewrite it receives is exactly the consented
/// output (harmless in effect; see `rewrite_verified`). The mtime
/// is deliberately not an `Option` — an
/// `Option` field made a filesystem with no mtime support capture
/// `None`, and the equality gate then read `None == None` as a MATCH,
/// so an equal-length replacement passed the identity check. Missing
/// identity must never be representable as valid identity: both capture
/// sites PROPAGATE `Metadata::modified`'s failure as a loud refusal
/// instead, and the compiler enforces that no identity exists without a
/// timestamp.
#[cfg(not(unix))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct OccupantIdentity {
    mtime: std::time::SystemTime,
    len: u64,
}

/// The preflight snapshot of a pre-existing patch: its bytes AND the
/// identity of the exact file they were read from (the
/// install verifies BOTH before it will unlink an occupant).
struct PriorPatch {
    bytes: Vec<u8>,
    identity: OccupantIdentity,
}

/// Snapshot a pre-existing patch for the install's ownership check:
/// bytes + identity off one descriptor (`read_regular_bounded_with_identity`).
fn snapshot_prior_patch(path: &Path) -> std::io::Result<PriorPatch> {
    let (bytes, identity) = read_regular_bounded_with_identity(path)?;
    Ok(PriorPatch { bytes, identity })
}

/// Rollback reporting: the note for a
/// rollback that could NOT capture the patch path for verification (the
/// quarantine name minting or the capture rename failed for a
/// non-NotFound reason). When a PRE-EXISTING patch was displaced by this
/// run's install, the note must ALSO say it was NOT restored — the
/// operator's own artifact is still overwritten on disk (the path holds
/// THIS run's patch), and a note naming only the capture failure hides
/// exactly the state they have to repair. Pure so the both-arms oracle
/// test can pin it.
fn capture_failure_note(patch_path: &Path, err: &std::io::Error, had_prior_patch: bool) -> String {
    let mut note = format!(
        "{} could not be captured for verification ({err}) — left as-is",
        patch_path.display()
    );
    if had_prior_patch {
        note.push_str(
            "; the PRE-EXISTING patch this run displaced was NOT restored \
             (the path still holds this run's patch)",
        );
    }
    note
}

/// `read_regular_bounded` plus the `OccupantIdentity` of the opened file
/// — ONE `fstat` on the opened fd serves both the regular-file
/// gate and the identity capture, so the identity names exactly
/// the object the bytes were read from.
fn read_regular_bounded_with_identity(path: &Path) -> std::io::Result<(Vec<u8>, OccupantIdentity)> {
    use std::io::Read as _;
    let mut opts = std::fs::OpenOptions::new();
    opts.read(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt as _;
        opts.custom_flags(libc::O_NONBLOCK);
    }
    let mut f = opts.open(path)?;
    let meta = f.metadata()?;
    if !meta.file_type().is_file() {
        return Err(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "not a regular file",
        ));
    }
    #[cfg(unix)]
    let identity = {
        use std::os::unix::fs::MetadataExt as _;
        OccupantIdentity {
            dev: meta.dev(),
            ino: meta.ino(),
            mtime: meta.mtime(),
            mtime_nsec: meta.mtime_nsec(),
        }
    };
    #[cfg(not(unix))]
    let identity = OccupantIdentity {
        // A filesystem that cannot report an mtime cannot
        // supply an identity — REFUSE loudly rather than capture an
        // absent field the equality gate would treat as matching. The
        // wrap deliberately replaces the error's kind: `Unsupported` is
        // reserved at this fn's call sites for the
        // not-a-regular-file gate, and this failure must not borrow that
        // message.
        mtime: meta.modified().map_err(|e| {
            std::io::Error::other(format!(
                "cannot capture the reviewed file's identity: the \
                 filesystem reports no modification time ({e})"
            ))
        })?,
        len: meta.len(),
    };
    let mut out = Vec::new();
    f.read_to_end(&mut out)?;
    Ok((out, identity))
}

/// The ONE write-seam identity verdict — does the file the
/// destructive write just opened carry the identity the pre-write gate
/// verified? Pure and cfg-FREE on purpose: the non-unix
/// `rewrite_verified` arm never compiles on a Unix host, so
/// its ENFORCEMENT is pinned through this shared fn instead — both
/// anchored arms route their compare here, and neutralizing it fails
/// the Unix-built post-gate swap e2e plus the verdict unit pin (the
/// check that proves the non-unix arm's verdict path is the same live
/// code, not a CI-invisible twin).
fn write_seam_identity_holds(expected: &OccupantIdentity, actual: &OccupantIdentity) -> bool {
    actual == expected
}

/// Quarantine collision: mint a fresh,
/// unpredictable quarantine name per capture. A fixed
/// pid-scoped name (`<file>.<tag>.<pid>`) can COLLIDE with a stale
/// quarantine left by a crashed prior run (pids recycle), and
/// `std::fs::rename` REPLACES an existing destination — so the capture
/// itself would silently destroy the stale entry, which is exactly the
/// unverified-delete class the quarantine exists to prevent. The name
/// folds in the pid, a process-global monotonic counter (uniqueness
/// within this process), and a 64-bit random component (`RandomState` is
/// seeded from OS randomness — no new dependency) hashed over the
/// current `SystemTime` nanos, and the destination is verified ABSENT
/// immediately before it is used. The exists-check→rename gap is not
/// closable with portable primitives, but with the nonce a collision in
/// that gap is no longer CONSTRUCTIBLE: an actor would have to plant a
/// file at a name containing 64 random bits it cannot predict from
/// outside this process — unlike the fixed per-pid name, which every
/// crashed past run left at a knowable path.
/// A per-call 64-bit nonce (a fresh `RandomState` seeded hash of the
/// wall clock and a process-wide sequence) — the collision-resistant half
/// of every temporary name this module mints.
fn fresh_nonce(seq: u64) -> u64 {
    use std::hash::{BuildHasher as _, Hasher as _};
    let mut h = std::collections::hash_map::RandomState::new().build_hasher();
    h.write_u128(
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0),
    );
    h.write_u64(seq);
    h.finish()
}

fn fresh_quarantine_path(path: &Path, tag: &str) -> std::io::Result<PathBuf> {
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    const NAME_ATTEMPTS: usize = 16;
    let file_name = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| PATCH_FILENAME.to_string());
    for _ in 0..NAME_ATTEMPTS {
        let seq = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let nonce = fresh_nonce(seq);
        let candidate = path.with_file_name(format!(
            "{file_name}.{tag}.{pid}.{seq}.{nonce:016x}",
            pid = std::process::id(),
        ));
        match candidate.symlink_metadata() {
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(candidate),
            // Occupied (a stale quarantine, or anything else): NEVER
            // rename onto it — pick a fresh name instead.
            Ok(_) => continue,
            Err(e) => return Err(e),
        }
    }
    Err(std::io::Error::other(
        "could not find a free quarantine name after repeated attempts",
    ))
}

/// Give a quarantined foreign object back to `dest` using
/// NO-REPLACE primitives ONLY — POSIX `rename` REPLACES a destination
/// created between any occupancy check and the rename (a new writer's
/// patch, created in that gap, would be silently deleted by the
/// returning quarantine), so the give-back must be structurally incapable
/// of clobbering, not merely unlikely to. A regular file returns via
/// `hard_link` (same inode, so byte-identical; creation fails
/// `AlreadyExists` if the path is occupied) followed by unlinking the
/// quarantine name; a symlink is re-created atomically via `symlink` from
/// its read target (symlink creation never replaces). Returns whether the
/// object made it back; `false` means the caller preserves it at the
/// REPORTED quarantine name. Never destroys anything at `dest`.
fn return_quarantined(quarantine: &Path, dest: &Path, is_link: bool) -> bool {
    if is_link {
        #[cfg(unix)]
        {
            let Ok(target) = std::fs::read_link(quarantine) else {
                return false;
            };
            if std::os::unix::fs::symlink(&target, dest).is_err() {
                return false;
            }
            let _ = std::fs::remove_file(quarantine);
            true
        }
        #[cfg(not(unix))]
        {
            // Declared-weaker arm (the same tier as this file's other
            // non-unix fallbacks): a quarantined link stays at the
            // reported quarantine name.
            false
        }
    } else {
        if std::fs::hard_link(quarantine, dest).is_err() {
            return false;
        }
        let _ = std::fs::remove_file(quarantine);
        true
    }
}

/// TOCTOU: install the patch create-new first —
/// the install never unlinks an occupant it has not verified as this run's
/// own prior-patch snapshot. A remove-first shape (remove whatever sits at the
/// path, then `create_new`) is sound against the occupant the preflight
/// examined, but a file another same-user process created AFTER the
/// preflight lands in that unconditional remove — deleted unverified (the
/// teardown class, replayed on the install side). Instead:
///
/// 1. `create_new` first — the common case (nothing at the path) touches
///    nothing else, and a link planted at the path is never followed
///    (`O_EXCL` fails `AlreadyExists` on any occupant, links included).
/// 2. On `AlreadyExists`, the occupant is captured by ATOMIC RENAME to a
///    private quarantine name (the teardown pattern:
///    verification and every destructive act operate on an object no
///    other actor addresses — no verify→unlink window is reopened). The
///    name is NONCE-UNIQUE per capture (`fresh_quarantine_path`)
///    and verified absent immediately beforehand — a fixed pid-scoped
///    name collided with a stale quarantine from a crashed prior run
///    (pids recycle) and the rename silently REPLACED it.
/// 3. The QUARANTINED object is verified: a regular file (never a link,
///    never followed) whose FILESYSTEM IDENTITY — `(st_dev, st_ino)` +
///    mtime, fstat'd off the same fd the bytes are read through — equals
///    the preflight snapshot's, AND whose bytes equal the snapshot
///    (byte equality alone cannot prove ownership — an
///    identical-bytes file another actor created after the preflight is
///    a DIFFERENT file, and deleting it destroys that actor's work).
///    Ours → unlinked (rollback restores exactly those bytes, so nothing
///    is lost) and the create retried into the gap. Anything else — ANY
///    occupant when the preflight saw none, a link, a non-regular object,
///    a different identity, different bytes — is another actor's state:
///    given BACK via the no-replace return path and REFUSED loudly; if
///    the path re-occupied meanwhile it is preserved at the REPORTED
///    quarantine name, never destroyed.
/// 4. Bounded attempts: a path that keeps re-occupying refuses rather
///    than spinning.
fn install_patch_no_clobber(
    path: &Path,
    bytes: &[u8],
    prior: Option<&PriorPatch>,
) -> std::io::Result<Option<String>> {
    install_patch_no_clobber_with(path, bytes, prior, &mut |f, b| {
        use std::io::Write as _;
        f.write_all(b)
    })
}

/// The `fstat` identity of an open descriptor — the file THIS process
/// created, named by what it is rather than by the path it was created at.
/// A failure is RETURNED, never collapsed into an absent identity (a
/// missing mtime must stay unrepresentable, so the non-unix arm
/// propagates `Metadata::modified`'s error).
fn occupant_identity_of(f: &std::fs::File) -> std::io::Result<OccupantIdentity> {
    let meta = f.metadata()?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt as _;
        Ok(OccupantIdentity {
            dev: meta.dev(),
            ino: meta.ino(),
            mtime: meta.mtime(),
            mtime_nsec: meta.mtime_nsec(),
        })
    }
    #[cfg(not(unix))]
    {
        Ok(OccupantIdentity {
            // A missing mtime is unrepresentable — the error is
            // propagated with context, never collapsed into an absent
            // identity. This
            // arm is compiled by NO target the workspace can build for, so
            // its field set is pinned structurally (see
            // `every_occupant_identity_initializer_sets_one_arms_full_field_set`).
            mtime: meta.modified().map_err(|e| {
                std::io::Error::other(format!(
                    "cannot capture the partial patch's identity: the \
                     filesystem reports no modification time ({e})"
                ))
            })?,
            len: meta.len(),
        })
    }
}

/// A replacement write
/// that failed part-way leaves THIS run's partial file at the patch path.
/// Retire it — verified by the identity of the descriptor the run created,
/// never by the path — so the displaced prior patch can go back into
/// confirmed absence. A different occupant (another actor re-occupied the
/// path meanwhile) is returned untouched. Returns the error to report.
fn retire_own_partial(
    path: &Path,
    own: std::io::Result<OccupantIdentity>,
    write_err: std::io::Error,
) -> std::io::Error {
    let base = format!("writing the patch failed ({write_err})");
    let own = match own {
        Ok(own) => own,
        Err(e) => {
            return std::io::Error::other(format!(
                "{base}; the partial file's identity could not be read ({e}), \
                 so it was left at '{}'",
                path.display()
            ));
        }
    };
    let quarantine = match fresh_quarantine_path(path, "torn") {
        Ok(q) => q,
        Err(e) => {
            return std::io::Error::other(format!(
                "{base}; the partial file could not be captured for removal \
                 ({e}) and was left at '{}'",
                path.display()
            ));
        }
    };
    match std::fs::rename(path, &quarantine) {
        Ok(()) => {}
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return std::io::Error::other(format!(
                "{base}; the partial file had already vanished from the path"
            ));
        }
        Err(e) => {
            return std::io::Error::other(format!(
                "{base}; the partial file could not be captured ({e}) and was \
                 left at '{}'",
                path.display()
            ));
        }
    }
    let is_link = quarantine
        .symlink_metadata()
        .map(|m| m.file_type().is_symlink())
        .unwrap_or(false);
    let ours = !is_link
        && read_regular_bounded_with_identity(&quarantine)
            .map(|(_, id)| id == own)
            .unwrap_or(false);
    if ours {
        match std::fs::remove_file(&quarantine) {
            Ok(()) => std::io::Error::other(format!("{base}; the partial patch was removed")),
            Err(e) => std::io::Error::other(format!(
                "{base}; the partial patch could not be removed ({e}) — \
                 preserved at '{}'",
                quarantine.display()
            )),
        }
    } else if return_quarantined(&quarantine, path, is_link) {
        std::io::Error::other(format!(
            "{base}; the path meanwhile holds another actor's file, left in \
             place byte-untouched"
        ))
    } else {
        std::io::Error::other(format!(
            "{base}; another actor's file at the path could not be returned \
             in place — preserved at '{}'",
            quarantine.display()
        ))
    }
}

/// The install with its WRITE injected (production: `write_all`; the unit
/// tests inject a torn write). Unlinking the displaced prior patch the
/// moment its quarantined copy verified as this run's — before the
/// replacement was created, let alone fully written — would mean a create or write
/// failure after that point leaves a partial NEW file at the path, which the
/// rollback rightly classifies as not-the-prior and preserves, and the
/// prior's only on-disk copy is gone. So the verified prior is HELD at
/// its quarantine name until the replacement is fully written: on success
/// the held copy is removed (its bytes are also in memory for the
/// rollback); on ANY failure this run's partial file is retired
/// (`retire_own_partial`) and the prior is returned into confirmed absence
/// by no-replace primitives — and when the path re-occupied meanwhile, the
/// prior is preserved at its quarantine name, which the error NAMES.
/// Returns an optional note for the caller to surface (a held copy that
/// could not be removed after a successful install).
fn install_patch_no_clobber_with(
    path: &Path,
    bytes: &[u8],
    prior: Option<&PriorPatch>,
    write: &mut dyn FnMut(&mut std::fs::File, &[u8]) -> std::io::Result<()>,
) -> std::io::Result<Option<String>> {
    const ATTEMPTS: usize = 3;
    let mut held_prior: Option<PathBuf> = None;
    let mut outcome: std::io::Result<()> = Err(std::io::Error::other(
        "the patch path keeps being re-occupied by another writer — \
         refusing after repeated attempts; stop the other writer and re-run",
    ));
    for _ in 0..ATTEMPTS {
        match std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
        {
            Ok(mut f) => {
                outcome = match write(&mut f, bytes) {
                    Ok(()) => Ok(()),
                    Err(e) => {
                        let own = occupant_identity_of(&f);
                        drop(f);
                        Err(retire_own_partial(path, own, e))
                    }
                };
                break;
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {}
            Err(e) => {
                outcome = Err(e);
                break;
            }
        }
        // Occupied: capture the occupant atomically at a FRESH
        // nonce-unique quarantine name (never a fixed name an
        // earlier crashed run could have left occupied), then verify the
        // CAPTURED object itself — never the path, which another actor
        // can retarget between a verify and a remove.
        let quarantine = match fresh_quarantine_path(path, "install") {
            Ok(q) => q,
            Err(e) => {
                outcome = Err(e);
                break;
            }
        };
        match std::fs::rename(path, &quarantine) {
            Ok(()) => {}
            // Vanished between the failed create and the capture — retry
            // the create into the gap.
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => continue,
            Err(e) => {
                outcome = Err(e);
                break;
            }
        }
        let is_link = quarantine
            .symlink_metadata()
            .map(|m| m.file_type().is_symlink())
            .unwrap_or(false);
        // `read_regular_bounded_with_identity` is fd-type-gated and
        // non-blocking: a FIFO or device classifies foreign
        // without a byte read and without a wedge. Ownership is IDENTITY
        // first: the quarantined occupant must be the SAME
        // FILE the preflight snapshotted — `(dev, ino)` + mtime off the
        // reading fd — with the byte compare kept as a secondary check.
        // An identical-bytes file another actor created after the
        // preflight carries a different identity and classifies foreign.
        let ours = !is_link
            && prior.is_some_and(|p| {
                read_regular_bounded_with_identity(&quarantine)
                    .map(|(b, id)| id == p.identity && b == p.bytes)
                    .unwrap_or(false)
            });
        if ours {
            // HELD, not unlinked — the prior survives on disk
            // until the replacement is fully written.
            if let Some(stale) = held_prior.replace(quarantine) {
                let _ = std::fs::remove_file(stale);
            }
            continue;
        }
        // Foreign: give it back by no-replace primitives only, then
        // refuse — the occupant survives byte-untouched either way.
        let returned = return_quarantined(&quarantine, path, is_link);
        outcome = Err(std::io::Error::other(format!(
            "a file appeared at the patch path after the pre-run check and \
             it is not the pre-existing patch this run snapshotted (another \
             actor's) — refusing to overwrite it{}. Move it aside and \
             re-run",
            if returned {
                "; it was left in place byte-untouched".to_string()
            } else {
                format!(
                    "; the path re-occupied while returning it, so it is \
                     preserved at '{}'",
                    quarantine.display()
                )
            }
        )));
        break;
    }
    match outcome {
        Ok(()) => {
            let mut note = None;
            if let Some(q) = held_prior {
                if let Err(e) = std::fs::remove_file(&q) {
                    note = Some(format!(
                        "the displaced pre-existing patch's on-disk copy could \
                         not be removed after the install ({e}) — it remains \
                         at '{}'",
                        q.display()
                    ));
                }
            }
            Ok(note)
        }
        Err(e) => {
            let Some(q) = held_prior else {
                return Err(e);
            };
            if return_quarantined(&q, path, false) {
                Err(std::io::Error::other(format!(
                    "{e}; the pre-existing patch was restored in place"
                )))
            } else {
                Err(std::io::Error::other(format!(
                    "{e}; the pre-existing patch is preserved at '{}' (the \
                     path re-occupied while restoring it)",
                    q.display()
                )))
            }
        }
    }
}

/// The refusal `--write` gives when the workspace lock could not be taken.
///
/// Pure — it reads nothing and writes nothing — so the three remedies can be
/// oracle-tested directly, which matters because the whole point of the three
/// arms is that they send the user to three DIFFERENT places and the compiler
/// cannot tell you when one of them points at the wrong one.
///
/// The acquire lands AFTER consent, so whatever this returns is the answer to
/// the user's "yes, apply". All three arms therefore say the same thing about
/// the workspace — no source file was rewritten, nothing was committed, the
/// attempt may have left the untracked lock directory behind — and differ only
/// in WHY and WHAT TO DO:
///
/// * [`AcquireError::Interrupted`] — you were QUEUED BEHIND another `cerulion`
///   and gave up. Nothing is wrong; re-run when it finishes. (Distinct wording
///   is also what lets a test tell the lock wait apart from a later write
///   safepoint, whose refusal is the bare "interrupted — …".)
/// * [`AcquireError::Failed`] — the WORKSPACE or its filesystem is the
///   problem, and the user can fix it: a `.cerulion` that is a symlink or not a
///   directory, a permission on it, a filesystem that cannot lock.
/// * [`AcquireError::Internal`] — a bug in `cerulion`. These arrive from the
///   lock re-entering its own acquire, or its reservation vanishing
///   mid-acquire; NOTHING about the user's workspace is wrong. Folding them
///   into the arm above would send
///   a user to audit a `.cerulion` that is perfectly fine, for a `cerulion` bug — the
///   cause is interpolated, so it is never silent, but the REMEDY beside it
///   would point the wrong way.
fn workspace_lock_refusal(error: &AcquireError, lock_path: &Path, lock_dir: &Path) -> CliError {
    let (lock_path, lock_dir) = (lock_path.display(), lock_dir.display());
    CliError::Validation(match error {
        AcquireError::Interrupted => format!(
            "interrupted while waiting for the workspace lock at '{lock_path}' — no \
             source file was rewritten and nothing was committed (the wait \
             itself may have created the untracked '{lock_dir}'). Something else was \
             holding that lock — another `cerulion` command, `cerulion-wsd`, \
             or another request in this process; re-run once it finishes."
        ),
        AcquireError::Failed(cause) => format!(
            "could not take the workspace lock at '{lock_path}' ({cause}) — no source \
             file was rewritten and nothing was committed, though this run \
             may have created '{lock_dir}' (untracked) before failing. That lock is \
             how this migration serializes against every other `cerulion` \
             writer on this workspace; fix that path — a `.cerulion` that \
             is a symlink or not a directory, a permission on it, or a \
             filesystem that cannot lock — and re-run."
        ),
        AcquireError::Internal(cause) => format!(
            "the workspace lock at '{lock_path}' refused because of a BUG IN \
             `cerulion` ({cause}) — no source file was rewritten and nothing \
             was committed, though this run may have created '{lock_dir}' \
             (untracked) before failing. Nothing is wrong with your workspace \
             and there is nothing to fix in it: please report this at \
             https://github.com/cerulion-inc/cerulion/issues with the \
             command you ran and the message above. Re-running may succeed if \
             this was a transient internal race."
        ),
    })
}

pub fn run_migrate(opts: &MigrateOptions, deps: &mut MigrateDeps<'_>) -> CliResult<MigrateOutcome> {
    let ws = opts.workspace.canonicalize().map_err(|e| {
        CliError::Validation(format!(
            "workspace '{}' is not accessible: {e}",
            opts.workspace.display()
        ))
    })?;
    let ws_src = ws.join("src");
    if !ws_src.is_dir() {
        return Err(CliError::Validation(format!(
            "'{}' does not look like a colcon workspace (no src/ directory)",
            ws.display()
        )));
    }
    // Canonicalize the SRC ROOT itself: `src/` being a symlink is a
    // legitimate colcon-overlay layout — the user's own spelling, not an
    // attack — and the clang engine canonicalizes `--src-root` and emits
    // CANONICAL edit paths, so every comparison against engine output
    // (compile-db containment, build_plan's gate, package-root joins) must
    // happen in canonical space or a legitimate workspace is refused
    // outright. The per-component walker is NOT weakened by moving its
    // boundary here: the anchor below pins the canonical root as a HELD
    // FD at entry, so a mid-run swap of the spelling changes what the
    // path names, never what the anchor is.
    let ws_src = ws_src.canonicalize().map_err(|e| {
        CliError::Validation(format!(
            "cannot resolve the workspace src root '{}': {e}",
            ws_src.display()
        ))
    })?;
    let src_anchor = anchored::Anchor::new(&ws_src).map_err(|e| {
        CliError::Validation(format!(
            "cannot open the workspace src root '{}': {e}",
            ws_src.display()
        ))
    })?;

    // -- compile databases -------------------------------------------------
    let dbs = find_compile_dbs(&ws);
    if dbs.is_empty() {
        return Err(CliError::Validation(format!(
            "no compile_commands.json under '{}/build' — the migration \
             analyses the workspace through its compile database. Build the \
             workspace with it exported:\n  cd {} && colcon build \
             --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON\nthen re-run \
             this verb.",
            ws.display(),
            ws.display()
        )));
    }

    // -- run the engine per TU --------------------------------------------
    let mut outputs = Vec::new();
    let mut analysed: Vec<PathBuf> = Vec::new();
    for db in &dbs {
        let db_dir = db.parent().expect("db path has a parent").to_path_buf();
        let json = read_regular_bounded_to_string(db).map_err(|e| {
            CliError::Validation(format!(
                "cannot read the compile database '{}': {e}",
                db.display()
            ))
        })?;
        for tu in tus_for_db(&json, &ws_src)? {
            if analysed.contains(&tu) {
                continue; // a TU can appear in the merged + per-package dbs
            }
            // The interrupt latch is polled at the top of
            // every TU, not first consumed inside the write window. The CLI
            // arms its Ctrl-C handler before this loop, and a real clang pass
            // over a workspace is minutes long — an interrupt honoured only at
            // the end of it is not honoured. Nothing has been written at this
            // point, so the refusal says so.
            if (deps.interrupted)() {
                return Err(interrupted_before_consent(PreConsentWindow::Analysis {
                    analysed: analysed.len(),
                }));
            }
            let raw = match deps.engine.analyze_tu(&db_dir, &ws_src, &tu) {
                Ok(raw) => raw,
                // A TERMINAL Ctrl-C reaches the engine too: it runs in this
                // process group and dies at its default disposition, so on
                // the shape a user actually produces the engine's failure IS
                // the interrupt (a `kill -INT <pid>` aimed at this process
                // alone leaves the engine alive, and the next poll — the
                // per-TU one if another TU follows, else the write head —
                // sees the flag). Attribute it instead of reporting a prover
                // crash to the user who pressed Ctrl-C; the engine's own text
                // stays observable at debug level.
                Err(e) if interrupted_with_grace(deps.interrupted) => {
                    tracing::debug!(
                        error = %e,
                        "the engine failed while an interrupt was pending — attributed to the interrupt"
                    );
                    return Err(interrupted_before_consent(PreConsentWindow::Analysis {
                        analysed: analysed.len(),
                    }));
                }
                Err(e) => return Err(e),
            };
            outputs.push(parse_tool_output(&raw)?);
            analysed.push(tu);
        }
    }
    let analysis = merge_tool_outputs(&outputs);

    // -- git + plan --------------------------------------------------------
    let git = gather_git_info(&ws);
    let rel_base = git.toplevel.clone().unwrap_or_else(|| ws.clone());
    let plan = build_plan(&ws_src, &rel_base, &analysis)?;
    let rclpy = scan_rclpy(&ws, &ws_src);

    // -- report ------------------------------------------------------------
    let engine_desc = deps.engine.describe();
    let report = render_report(
        &ws,
        &analysis,
        &plan,
        &rclpy,
        &ReportMeta {
            tu_count: analysed.len(),
            db_count: dbs.len(),
            engine: &engine_desc,
            write_mode: opts.write,
        },
    );
    deps.out.write_all(report.as_bytes())?;

    let manifest_candidates: Vec<ManifestCandidate> = analysis
        .rewrites
        .iter()
        .map(|rw| ManifestCandidate {
            package: package_of(&ws_src, Path::new(&rw.file)),
            file: rel_to(&ws, Path::new(&rw.file)),
            line: rw.line,
            kind: rw.kind.clone(),
        })
        .collect();
    let manifest_manual: Vec<ManifestManualCandidate> = analysis
        .candidates
        .iter()
        .map(|c| ManifestManualCandidate {
            package: package_of(&ws_src, Path::new(&c.file)),
            file: rel_to(&ws, Path::new(&c.file)),
            line: c.line,
            function: c.function.clone(),
            reason: c.reason.clone(),
            detail: c.detail.clone(),
        })
        .collect();
    // An UNVERIFIABLE tree (a failed `git status`) is recorded as
    // dirty — never as clean — and said out loud.
    let dirty_tracked = git.status_error.is_some() || any_tracked_change(&git.status);
    if let Some(e) = &git.status_error {
        writeln!(
            deps.out,
            "note: `git status` failed ({e}) — the tree could not be verified \
             clean; the manifest records dirty=true and --write will refuse \
             until it succeeds"
        )?;
    }
    let manifest_path = ws.join(MANIFEST_REL_PATH);

    if !opts.write {
        let manifest = MigrateManifest {
            schema: MANIFEST_SCHEMA,
            version: MANIFEST_VERSION,
            workspace_key: ManifestWorkspaceKey {
                head: git.head.clone(),
                dirty: dirty_tracked,
            },
            generated_by: "dry-run".to_string(),
            counts: ManifestCounts {
                packages: plan.affected_packages.len(),
                call_sites: analysis.rewrites.len(),
                manual_candidates: analysis.candidates.len(),
                rclpy_nodes: rclpy.len(),
            },
            candidates: manifest_candidates,
            manual_candidates: manifest_manual,
            rclpy: rclpy.clone(),
            decision: existing_decision(&ws),
        };
        if let Err(e) = write_manifest_atomic(&ws, &manifest) {
            // The report has already printed in full (never masked), but
            // the manifest IS the dry-run's one machine-readable product —
            // its consumer (the launch-time offer) reads it, so failing to
            // refresh it is a FAILED dry-run, exit nonzero.
            return Err(CliError::Validation(format!(
                "the report above is complete, but the manifest refresh at \
                 {} FAILED: {e}",
                manifest_path.display()
            )));
        }
        return Ok(MigrateOutcome::Reported);
    }

    // -- --write -----------------------------------------------------------
    // The FIRST thing --write does is honour a pending
    // interrupt. One requested during the analysis, the report, or the git
    // gathering — before consent was ever asked — is answered HERE, ahead of
    // every gate below (the nothing-to-apply exit, the git gates, the prompt),
    // rather than by the first write safepoint after the user's yes: a gate's
    // own message would otherwise MASK the interrupt (a `git status` the
    // Ctrl-C killed reads as "repair the repository"; an empty plan exits 0
    // as if nothing had been asked). The latch is a one-shot flag the CLI
    // arms before the analysis and this engine can only READ, so a
    // pre-consent trip cannot be cleared — and it must not be ignored either,
    // since the same flag is the write window's whole protection. So the run
    // refuses, before a question whose yes could not be honoured is asked.
    // Nothing has been written at this point in --write mode (the write-mode
    // manifest lands after the commit), so the refusal says so.
    if (deps.interrupted)() {
        return Err(interrupted_before_consent(if opts.assume_yes {
            PreConsentWindow::BeforeWriteBatch
        } else {
            PreConsentWindow::BeforeConsent
        }));
    }
    if plan.files.is_empty() {
        writeln!(
            deps.out,
            "--write: nothing to apply — no publish call site was proven \
             safe to rewrite."
        )?;
        return Ok(MigrateOutcome::NothingToApply);
    }
    let Some(toplevel) = git.toplevel.clone() else {
        return Err(CliError::Validation(format!(
            "'{}' is not inside a git repository — --write refuses without \
             one (the undo contract is `git revert <commit>` / `git apply \
             -R {}`). Run `git init && git add -A && git commit` first, or \
             use the dry-run report to migrate by hand.",
            ws.display(),
            PATCH_FILENAME
        )));
    };
    if git.head.is_none() {
        return Err(CliError::Validation(
            "the git repository has no commits — --write needs a committed \
             baseline to revert to. Commit your workspace first."
                .to_string(),
        ));
    }
    // A `git status` that failed must not read
    // as an empty — clean — status, or --write could commit over
    // uncommitted tracked changes. Fail CLOSED.
    if let Some(e) = &git.status_error {
        return Err(CliError::Validation(format!(
            "--write refuses: the git tree could not be verified clean — \
             `git status` failed in '{}': {e}. Repair the repository state \
             and re-run.",
            ws.display()
        )));
    }
    let planned_rel: Vec<String> = plan.files.iter().map(|f| f.rel.clone()).collect();
    match classify_dirty(&git.status, &planned_rel) {
        DirtyVerdict::Clean => {}
        DirtyVerdict::Dirty {
            tracked,
            untracked_planned,
        } => {
            let mut msg = String::from(
                "--write refuses a dirty git tree (the ONE-commit undo \
                 contract needs a clean baseline). ",
            );
            if !tracked.is_empty() {
                msg.push_str(&format!(
                    "Uncommitted tracked changes: {}. ",
                    tracked.join(", ")
                ));
            }
            if !untracked_planned.is_empty() {
                msg.push_str(&format!(
                    "Files the migration would edit but git does not track: \
                     {}. ",
                    untracked_planned.join(", ")
                ));
            }
            msg.push_str("Commit or stash first, then re-run.");
            return Err(CliError::Validation(msg));
        }
    }

    // Consent.
    if !deps.is_tty && !opts.assume_yes {
        return Err(CliError::Validation(format!(
            "--write would edit {} file(s), commit, and build {} package(s), \
             but stdin is not a TTY and --yes was not passed — nothing was \
             written. Re-run with --yes to apply non-interactively, or drop \
             --write for the dry-run report.",
            plan.files.len(),
            plan.affected_packages.len()
        )));
    }
    if !opts.assume_yes {
        let prompt = format!(
            "Apply the migration above? {} file(s), one commit, then `colcon \
             build --packages-select {}` [y/N] ",
            plan.files.len(),
            plan.affected_packages.join(" ")
        );
        if !(deps.confirm)(&prompt)? {
            writeln!(deps.out, "declined — nothing written.")?;
            return Ok(MigrateOutcome::Declined);
        }
        // The prompt's read is not interruptible — `ctrlc`
        // installs its handler with SA_RESTART, so a Ctrl-C landing while the
        // read waits leaves the prompt waiting and only flips the flag. The
        // earliest the engine can honour it is the answer's arrival: a yes
        // that followed an interrupt is refused HERE, in the prompt's own
        // vocabulary, instead of being accepted and then reported as
        // "interrupted" from a write safepoint after the lock. A no needs no
        // such check — it is already the safe answer.
        if (deps.interrupted)() {
            return Err(interrupted_before_consent(PreConsentWindow::AfterYes));
        }
    }

    // The WRITE BATCH is serialized against every other Cerulion writer on
    // this workspace (`ros2 attach`, `node`/`graph`/`schema`, `cerulion-wsd`,
    // a second `ros2 migrate`) — they all take the SAME
    // `<ws>/.cerulion/workspace.lock`. Scope matters in both directions: the
    // lock is taken HERE, after consent, because holding it across the
    // analysis and an interactive prompt would block every other writer for
    // as long as a human takes to read a diff; and it is released before the
    // colcon build (below), which is a compile, not a workspace mutation.
    //
    // It is the TRACKED-WRITE-FREE variant: the plain `acquire` appends
    // `.cerulion/` to an existing `.gitignore` the first time it creates the
    // lock directory, which is a TRACKED modification — and this verb
    // promises that a rolled-back `--write` leaves a pristine tree. (Only that
    // promise is load-bearing HERE: the dry-run returns long before this
    // statement, so it never takes a lock of either kind.) Taking `acquire`
    // here breaks it
    // (the rollback arms
    // `write_rolls_back_when_the_commit_fails` and
    // `a_failing_source_write_rolls_back_files_already_written` fail on
    // `M .gitignore`), and the entry is not this verb's to add to a ROS 2
    // workspace `cerulion` never scaffolded.
    //
    // It also WAITS differently. `acquire` blocks in the kernel, and `ctrlc`
    // installs its handler with `SA_RESTART`, so that `flock` is auto-restarted
    // and answers no ordinary signal: a user who consented and then hit a held
    // lock would have no way out but an uncaught one, inside the verb that installs a
    // Ctrl-C handler precisely so the write window stays interruptible.
    //
    // Not preceded by ANOTHER `(deps.interrupted)()` read. The consent
    // boundary above already polled the latch — before the
    // prompt, and again after a yes — so an interrupt requested before this
    // point was refused before reaching it; one requested between that poll
    // and here is seen by the wait loop's first check on a CONTENDED acquire,
    // and by the pre-write safepoint (after the TOCTOU gate, at the head of
    // the WRITE PHASE — nothing between here and there writes) on an
    // uncontended one. The seam is also a TIMING HOOK for the tests that swap
    // a source "after the TOCTOU gate, before the first write": they anchor on
    // the lock FILE existing (on an UNCONTENDED acquire — those fixtures, by
    // construction — the first poll that sees it is that safepoint; a
    // contended one polls from the wait loop, which sees the file too), not
    // on a call count, so a poll added before the lock cannot move them —
    // while one added between the lock and the gate would, and must not be.
    // An EXHAUSTIVE match, not a `?`: `AcquireError` has no `From` into
    // `CliError` precisely so the unlocked path cannot be reached by accident.
    // The three arms are reached under different conditions and have different
    // REMEDIES, which is the whole reason they are three — see
    // `workspace_lock_refusal`.
    let lock_path = ws
        .join(crate::workspace_lock::LOCK_DIR)
        .join(crate::workspace_lock::LOCK_FILE);
    let lock_dir = ws.join(crate::workspace_lock::LOCK_DIR);
    let write_lock = match WorkspaceLock::acquire_interruptibly(&ws, deps.interrupted) {
        Ok(lock) => lock,
        Err(error) => return Err(workspace_lock_refusal(&error, &lock_path, &lock_dir)),
    };

    // TOCTOU gate: the plan's bytes were read BEFORE the report and the
    // consent prompt — an edit made while the user reviewed the diff must
    // refuse, never be silently overwritten. Verify EVERY file before
    // writing ANY. It runs INSIDE the lock so a cooperating writer cannot
    // land an edit between the verification and the write.
    for f in &plan.files {
        // Bounded + type-gated — a FIFO swapped in here would otherwise
        // wedge the open forever (the read still FOLLOWS a symlink, as it
        // always did: the byte compare is this gate's job, and the symlink
        // defense is the anchored write seam).
        // Bytes alone are not the whole gate — a
        // different regular file with IDENTICAL contents atomically renamed
        // over the path passed the byte check, so the write landed
        // generated output in a file consent never covered. The identity
        // captured at plan time (same fd as the bytes) must match too.
        let (now, now_identity) = match read_regular_bounded_with_identity(&f.path) {
            Ok(pair) => pair,
            Err(e) if e.kind() == std::io::ErrorKind::Unsupported => {
                return Err(CliError::Validation(format!(
                    "'{}' is no longer a regular file (another actor \
                     replaced it while the migration was being reviewed) — \
                     nothing was written. Re-run the dry-run and try again.",
                    f.path.display()
                )));
            }
            Err(e) => return Err(e.into()),
        };
        if now != f.old_bytes {
            return Err(CliError::Validation(format!(
                "'{}' changed while the migration was being reviewed — \
                 nothing was written. Re-run the dry-run and try again.",
                f.path.display()
            )));
        }
        if now_identity != f.old_identity {
            // Same bytes, different file (or the same inode rewritten in
            // place — mtime is part of the identity): what was reviewed is
            // not what occupies the path now, so consent does not carry
            // over. Checked AFTER the byte compare so an ordinary
            // mid-review edit keeps its accurate "changed while" message.
            return Err(CliError::Validation(format!(
                "'{}' changed identity since the migration was reviewed \
                 (a different file — with the same contents — now occupies \
                 the path, or the file was rewritten in place) — nothing \
                 was written. Re-run the dry-run and try again.",
                f.path.display()
            )));
        }
    }

    // Identity environment variables that are SET but blank outrank `-c`
    // config with an EMPTY identity — remove them from every git child so
    // the fallback (and a configured identity) cannot be hollowed out.
    let blank_idents: Vec<&'static str> = [
        "GIT_AUTHOR_NAME",
        "GIT_AUTHOR_EMAIL",
        "GIT_COMMITTER_NAME",
        "GIT_COMMITTER_EMAIL",
        "EMAIL",
    ]
    .into_iter()
    .filter(|k| std::env::var(k).is_ok_and(|v| v.trim().is_empty()))
    .collect();
    let git_run = |args: &[&str]| -> CliResult<String> {
        let mut cmd = git_command(&toplevel);
        cmd.args(args);
        for k in &blank_idents {
            cmd.env_remove(k);
        }
        let out = cmd
            .output()
            .map_err(|e| CliError::Validation(format!("git failed to run: {e}")))?;
        if !out.status.success() {
            return Err(CliError::Validation(format!(
                "git {} failed: {}",
                args.join(" "),
                String::from_utf8_lossy(&out.stderr).trim()
            )));
        }
        Ok(String::from_utf8_lossy(&out.stdout).trim_end().to_string())
    };

    // A pre-existing patch file (an earlier run's artifact — untracked, so
    // the dirty gate rightly tolerates it) is SNAPSHOTTED before this run
    // overwrites it: every rollback below RESTORES it, never deletes a
    // user-owned file. One it cannot READ it refuses up front — a
    // restoration it cannot promise must not be silently skipped.
    let patch_path = ws.join(PATCH_FILENAME);
    // The pre-write probe is symlink_metadata, NEVER a following read: a
    // pre-existing patch path that is a SYMLINK is refused outright. A
    // VALID link would make the patch write overwrite whatever it points at
    // (a contributor can plant `cerulion-ros2-migration.patch -> anywhere`
    // in a shared workspace — the reported security shape), and a DANGLING
    // link read as "absent" would be DELETED by rollback's remove arm. Only
    // a regular file is snapshotted; anything else refuses before a byte is
    // written.
    let prior_patch: Option<PriorPatch> = match std::fs::symlink_metadata(&patch_path) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
        Err(e) => {
            return Err(CliError::Validation(format!(
                "cannot examine the pre-existing '{}' ({e}) — this run would \
                 overwrite it and could not restore it on failure. Move it \
                 aside and re-run.",
                patch_path.display()
            )));
        }
        Ok(meta) if meta.file_type().is_symlink() => {
            return Err(CliError::Validation(format!(
                "'{}' is a SYMLINK — this run writes the patch at that exact \
                 path and will not follow or replace a link (a link target \
                 outside the workspace would be silently overwritten). Move \
                 the link aside and re-run.",
                patch_path.display()
            )));
        }
        Ok(meta) if !meta.file_type().is_file() => {
            return Err(CliError::Validation(format!(
                "'{}' exists but is not a regular file — move it aside and \
                 re-run.",
                patch_path.display()
            )));
        }
        // The read is bounded + fd-type-gated — the `is_file`
        // metadata arm above refuses non-regular objects, but a FIFO
        // swapped in between that check and this read would WEDGE a blocking
        // open; `read_regular_bounded` turns that swap into the same loud
        // refusal instead. The snapshot carries the occupant's
        // FILESYSTEM IDENTITY (dev/ino + mtime, off the reading fd)
        // alongside its bytes — the install's ownership check requires
        // the identity, because bytes alone cannot distinguish the
        // snapshotted file from an identical-bytes file another actor
        // creates after this preflight.
        Ok(_) => match snapshot_prior_patch(&patch_path) {
            Ok(prior) => Some(prior),
            Err(e) => {
                return Err(CliError::Validation(format!(
                    "a pre-existing '{}' exists but cannot be read ({e}) — \
                     this run would overwrite it and could not restore it on \
                     failure. Move it aside and re-run.",
                    patch_path.display()
                )));
            }
        },
    };

    // The patch INSTALL is the module-level `install_patch_no_clobber`
    // (see there): create-new first, atomic-capture verification of any
    // occupant against the `prior_patch` snapshot above — the install
    // never unlinks a file it has not verified as this run's own.

    // Create-only patch write for the teardown's restore arms: the prior
    // patch is only ever restored into CONFIRMED ABSENCE, so a new occupant
    // appearing meanwhile refuses (`create_new`) and is reported — never
    // clobbered.
    fn write_patch_create_new(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
        use std::io::Write as _;
        let mut f = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)?;
        f.write_all(bytes)
    }

    // Restore the tree after a failure: put back the first `written_count`
    // planned files where the on-disk state is provably the MIGRATION'S
    // OWN, restore-or-remove the patch, and optionally unstage. Returns
    // what it could NOT (or deliberately did not) fix.
    let restore_tree = |written_count: usize,
                        unstage: bool,
                        torn: Option<(usize, FileIdentity)>,
                        restore_patch: bool|
     -> Vec<String> {
        let mut notes: Vec<String> = Vec::new();
        for (idx, f) in plan.files.iter().take(written_count).enumerate() {
            // Everything here goes through the ANCHORED walker — nothing is
            // ever read or written through a planted link, final OR parent.
            // OWNERSHIP rule (`rollback_disposition`, pure): restore only
            // the migration's own state — its migrated bytes, or the torn
            // output of the ONE write that provably MODIFIED the file
            // before failing (`torn` carries the failed write's position
            // AND the fstat identity of the file its open truncated; the
            // claim holds only while the file at the path still carries
            // that identity — `torn_claim_holds` — so an atomic replace
            // landing between the truncating open and this read is
            // preserved, not erased). Everything else — foreign
            // bytes, a planted symlink (a link is never migration-owned,
            // so there is deliberately NO unlink-and-recreate heal) —
            // is another actor's state: preserved and reported, never
            // erased.
            // STRUCTURE (`anchored::RestoreFile`): validation
            // and restoration share ONE fd — the disposition runs on
            // bytes+identity read off the descriptor the rewrite lands
            // on, so an atomic rename landing after our open retargets
            // the PATH, never the restore (a
            // read-then-reopen shape truncates the replacement).
            match anchored::open_for_restore(&src_anchor, &f.src_rel) {
                Ok(mut of) => {
                    let (cur, cur_id) = match of.read_all() {
                        Ok(t) => t,
                        Err(e) => {
                            notes.push(format!("restore {}: {e}", f.path.display()));
                            continue;
                        }
                    };
                    let torn_here = torn_claim_holds(torn.as_ref(), idx, &cur_id);
                    match rollback_disposition(&cur, &f.old_bytes, &f.new_bytes, torn_here) {
                        RollbackDisposition::Skip => {}
                        RollbackDisposition::PreserveForeign => {
                            notes.push(format!(
                                "{} holds neither the original nor the \
                                     migration's bytes (another actor wrote \
                                     it mid-run) — left as-is",
                                f.path.display()
                            ));
                        }
                        RollbackDisposition::Restore => {
                            if !of.writable {
                                notes.push(format!(
                                    "restore {}: the file is read-only — \
                                         restore the snapshot by hand",
                                    f.path.display()
                                ));
                            } else {
                                // Re-verified through the SAME fd right
                                // before truncation: an in-place edit
                                // landing since validation withdraws
                                // the restore (the fd stops
                                // rename-over, not a same-inode
                                // writer).
                                match of.rewrite_validated(&cur, &f.old_bytes) {
                                    Ok(true) => {}
                                    Ok(false) => {
                                        notes.push(format!(
                                            "{} changed while the \
                                                 rollback was deciding \
                                                 (another actor's in-place \
                                                 edit) — left as-is",
                                            f.path.display()
                                        ));
                                    }
                                    Err(e) => {
                                        notes.push(format!("restore {}: {e}", f.path.display()));
                                    }
                                }
                            }
                        }
                    }
                }
                Err(e) if anchored::is_eloop(&e) => {
                    notes.push(format!(
                        "{} is (or is behind) a symlink planted mid-run — \
                             left as-is",
                        f.path.display()
                    ));
                }
                Err(e) if anchored::is_not_regular(&e) => {
                    // A FIFO/socket/device planted mid-run is
                    // refused by the fd gate WITHOUT opening for a read
                    // that could block forever — another actor's
                    // object, preserved and reported.
                    notes.push(format!(
                        "{} is not a regular file (another actor's \
                             object planted mid-run) — left as-is",
                        f.path.display()
                    ));
                }
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                    // Vanished mid-run: recreate the original strictly
                    // INTO ABSENCE — `O_EXCL` cannot truncate anything
                    // that appeared at the path meanwhile.
                    match anchored::create_excl(&src_anchor, &f.src_rel, &f.old_bytes) {
                        Ok(()) => {}
                        Err(e2) if e2.kind() == std::io::ErrorKind::AlreadyExists => {
                            notes.push(format!(
                                "{} vanished mid-run and something else \
                                     now sits at the path (another actor's) \
                                     — left as-is",
                                f.path.display()
                            ));
                        }
                        Err(e2) => {
                            notes.push(format!("restore {}: {e2}", f.path.display()));
                        }
                    }
                }
                Err(e) => {
                    notes.push(format!("restore {}: {e}", f.path.display()));
                }
            }
        }
        // PATCH TEARDOWN — atomic capture, THEN verify, THEN destroy
        // (a replacement landing between a
        // pathname verify and a destructive remove would be deleted; the
        // lack of a portable funlink does not matter, because this order
        // does not need one). Whatever sits at the patch path is RENAMED to a
        // private quarantine name first — rename is atomic and captures
        // exactly the object at the path, file or link — so
        // verification and every destructive act operate on an object
        // no other actor addresses. Ours -> the quarantine is unlinked
        // (destroying only the verified object) and the prior patch,
        // if any, is restored into ABSENCE (create_new — a new
        // occupant refuses loudly, never clobbered). Foreign -> given
        // BACK by rename, but only into absence; if the path was
        // re-occupied meanwhile, the foreign object stays at the
        // REPORTED quarantine name rather than silently replacing
        // anyone. Nothing is ever deleted unverified. The
        // quarantine name is NONCE-UNIQUE (`fresh_quarantine_path`) —
        // a fixed pid-scoped name could collide with, and the
        // rename silently REPLACE, a stale quarantine left by a
        // crashed prior run (pids recycle). A minting failure lands
        // in the generic could-not-capture arm below (the helper
        // never fails `NotFound`, so that arm still uniquely means
        // "nothing at the patch path"). The ownership check here
        // deliberately stays a byte compare against `plan.diff_text`:
        // unlike the install's preflight snapshot (a user artifact
        // from an earlier run), the compared bytes are THIS run's own
        // just-written output, and what an identical-bytes occupant
        // would lose is a copy of exactly the diff this run prints —
        // re-derivable, not user state.
        if restore_patch {
            match fresh_quarantine_path(&patch_path, "rollback")
                .and_then(|q| std::fs::rename(&patch_path, &q).map(|()| q))
            {
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                    // Nothing at the path — restore a prior patch into the
                    // gap, else done.
                    if let Some(prior) = &prior_patch {
                        if let Err(e) = write_patch_create_new(&patch_path, &prior.bytes) {
                            notes.push(format!("restore prior patch: {e}"));
                        }
                    }
                }
                Err(e) => {
                    // The note must carry the prior-patch state
                    // too — see `capture_failure_note`.
                    notes.push(capture_failure_note(&patch_path, &e, prior_patch.is_some()));
                }
                Ok(quarantine) => {
                    let is_link = quarantine
                        .symlink_metadata()
                        .map(|m| m.file_type().is_symlink())
                        .unwrap_or(false);
                    // The verify read is type-gated and
                    // non-blocking — a WRITERLESS FIFO planted at the patch
                    // path would wedge `fs::read`'s open forever, so the
                    // rollback would never finish and never reach the index
                    // cleanup. `read_regular_bounded` fstat-gates the fd:
                    // any non-regular object (FIFO, socket, device, dir)
                    // classifies FOREIGN without a single byte read, and
                    // the give-back returns it by `hard_link` — which never
                    // opens it either.
                    let ours = !is_link
                        && read_regular_bounded(&quarantine)
                            .map(|b| b == plan.diff_text.as_bytes())
                            .unwrap_or(false);
                    if ours {
                        if let Err(e) = std::fs::remove_file(&quarantine) {
                            notes.push(format!("remove patch: {e}"));
                        }
                        if let Some(prior) = &prior_patch {
                            if let Err(e) = write_patch_create_new(&patch_path, &prior.bytes) {
                                notes.push(format!("restore prior patch: {e}"));
                            }
                        }
                    } else {
                        // Foreign: a replacement, a planted link, or
                        // unreadable — never destroyed. Returned in place
                        // via NO-REPLACE primitives ONLY (POSIX
                        // rename REPLACES a destination created between any
                        // occupancy check and the rename — the give-back
                        // must be incapable of clobbering a new writer's
                        // patch, not merely unlikely to); when the path is
                        // occupied, preserved at the quarantine name and
                        // reported.
                        let displaced = if prior_patch.is_some() {
                            "; the pre-existing patch it displaced was NOT \
                             restored"
                        } else {
                            ""
                        };
                        if return_quarantined(&quarantine, &patch_path, is_link) {
                            notes.push(format!(
                                "{} was replaced mid-run (not the migration's \
                                 bytes) — left as-is{displaced}",
                                patch_path.display()
                            ));
                        } else {
                            notes.push(format!(
                                "{} holds another actor's content that could \
                                 not be returned in place — preserved at \
                                 {}{displaced}",
                                patch_path.display(),
                                quarantine.display()
                            ));
                        }
                    }
                }
            }
        }
        if unstage {
            for f in plan.files.iter().take(written_count) {
                // Reset ONLY an index entry the migration itself staged —
                // the blob byte-identical to the migrated content. An entry
                // another actor staged on this path mid-run (a formatter
                // hook's re-stage) is not the migration's to erase; one at
                // the pre-migration baseline has nothing to reset. Raw
                // output deliberately: the text helper trims, and blob
                // comparison must be byte-exact.
                //
                // RESIDUAL: the `git show :path`
                // verify and the `git reset -- path` are two git
                // invocations, and git's CLI surface offers no
                // per-entry compare-and-swap — an actor staging in the
                // show-to-reset gap has that INDEX entry discarded.
                // Considered and rejected: holding `.git/index.lock`
                // ourselves blocks git's own commands (including the
                // reset), and hand-parsing the binary index formats
                // (v2/v3/v4, split-index, sparse) is a larger
                // correctness risk than the race it closes. The harm
                // class is the weakest on this path: an index entry is
                // a POINTER re-derivable from the worktree (`git add`
                // again), the staged blob survives in the object store,
                // and the WORKTREE content is untouched — unlike the
                // file-content races above, nothing is irrecoverable.
                let staged = git_command(&toplevel)
                    .args(["show", &format!(":{}", f.rel)])
                    .output();
                match staged {
                    Ok(o) if o.status.success() && o.stdout == f.new_bytes => {
                        if let Err(e) = git_run(&["reset", "-q", "--", f.rel.as_str()]) {
                            notes.push(format!("unstage {}: {e}", f.rel));
                        }
                    }
                    Ok(o) if o.status.success() && o.stdout == f.old_bytes => {
                        // Index at the pre-migration baseline — nothing of
                        // the migration's is staged.
                    }
                    Ok(o) if o.status.success() => {
                        notes.push(format!(
                            "the index entry for {} is not the migration's \
                             staging (another actor staged it mid-run) — \
                             left as-is",
                            f.rel
                        ));
                    }
                    _ => {
                        // No readable index entry — nothing staged to reset.
                    }
                }
            }
        }
        notes
    };
    let render_rollback = |notes: Vec<String>| -> String {
        if notes.is_empty() {
            "The working tree was rolled back: sources restored, the patch \
             restored/removed, nothing left staged."
                .to_string()
        } else {
            format!(
                "Rollback was INCOMPLETE — finish by hand: {}",
                notes.join("; ")
            )
        }
    };

    // WRITE PHASE — all-or-nothing: the patch first (same bytes as the
    // printed diff), then the edits; a failure at file N restores files
    // 0..=N and the patch before returning, so a partial migration is never
    // left behind with no commit to revert.
    //
    // INTERRUPT SAFEPOINTS: the CLI's Ctrl-C handler flips a
    // flag instead of terminating, and this window polls it — here (before
    // anything is written), between file writes, and before the commit. A
    // trip rolls back and refuses; a SIGINT can therefore never leave a
    // partial uncommitted migration behind (a SIGKILL still can — that is
    // what the patch file + clean-tree gate remediation is for).
    if (deps.interrupted)() {
        return Err(CliError::Validation(
            PRE_WRITE_INTERRUPT_REFUSAL.to_string(),
        ));
    }
    match install_patch_no_clobber(&patch_path, plan.diff_text.as_bytes(), prior_patch.as_ref()) {
        Ok(None) => {}
        Ok(Some(note)) => writeln!(deps.out, "note: {note}")?,
        Err(e) => {
            // The install already settled the patch path itself
            // (its partial retired, the displaced prior returned or
            // preserved and NAMED in `e`), so the rollback must not run
            // its patch arm — that arm would capture the RETURNED prior,
            // classify it foreign (it is not this run's diff) and report
            // it as an unrestored displacement.
            let notes = restore_tree(0, false, None, false);
            return Err(CliError::Validation(format!(
                "failed to write the patch file '{}': {e} — nothing was \
                 migrated. {}",
                patch_path.display(),
                render_rollback(notes)
            )));
        }
    }
    for (i, f) in plan.files.iter().enumerate() {
        // Interrupt safepoint: between file writes — everything written so
        // far (files 0..i and the patch) is rolled back.
        if (deps.interrupted)() {
            let notes = restore_tree(i, false, None, true);
            return Err(CliError::Validation(format!(
                "interrupted — every write was rolled back; nothing was \
                 migrated. {}",
                render_rollback(notes)
            )));
        }
        // NO-FOLLOW ON EVERY COMPONENT at the write seam: the write walks
        // from the workspace root with per-component openat(O_NOFOLLOW),
        // so a source swapped for a symlink after the review-time byte
        // check fails at its component — the final file OR an intermediate
        // DIRECTORY (a swapped parent redirected the final-only form) —
        // and never reaches the link's external target. The
        // write is additionally IDENTITY-BOUND — `rewrite_verified` fstats
        // the opened fd against the plan-time identity the pre-write gate
        // verified and refuses BEFORE truncating, then writes through that
        // same fd. This is not a path-level re-verification
        // (that would be the same TOCTOU): the
        // identity and the write share one descriptor, so an
        // identical-bytes replacement renamed over the path in the
        // gate→write gap is refused with the replacement unmodified.
        // Every failure lands in the all-or-nothing rollback.
        if let Err((opened, e)) = anchored::rewrite_verified(
            &src_anchor,
            &f.src_rel,
            &f.old_identity,
            &f.old_bytes,
            &f.new_bytes,
        ) {
            if anchored::is_identity_mismatch(&e) {
                // Refused before truncation: file i is UNTOUCHED — roll
                // back the files already written (0..i) and the patch,
                // with no torn evidence for i.
                let notes = restore_tree(i, false, None, true);
                return Err(CliError::Validation(format!(
                    "'{}' changed identity since the migration was \
                     reviewed (caught at the moment of write — what \
                     occupies the path is not the reviewed file) — \
                     nothing was migrated; the file now at the path was \
                     not modified. {}",
                    f.path.display(),
                    render_rollback(notes)
                )));
            }
            // Torn-restore applies ONLY when this failed write provably
            // MODIFIED the file (the final open succeeded — O_TRUNC
            // truncates at open), and the evidence is the IDENTITY of the
            // file that open truncated: the rollback restores the snapshot
            // only where the path still names that very file
            // (`torn_claim_holds`: an atomic replace landing
            // after the truncating open must be preserved, not erased). A
            // write that failed before opening never touched the file, and
            // any foreign content there is another actor's to keep.
            let notes = restore_tree(i + 1, false, opened.map(|id| (i, id)), true);
            return Err(CliError::Validation(format!(
                "failed to write '{}': {e} — nothing was migrated. {}",
                f.path.display(),
                render_rollback(notes)
            )));
        }
    }

    let commit_msg = format!(
        "ros2 migrate: rewrite {} publish call site(s) to the \
         loaned-message API\n\nGenerated by `cerulion ros2 migrate \
         --write`. Undo: `git revert <this commit>` or `git apply -R {}`.",
        analysis.rewrites.len(),
        PATCH_FILENAME
    );
    // An environment with NO configured git identity (a container, a bare CI
    // runner) must not fail the commit: pass a tool identity as `-c` config
    // ON the commit invocation itself, PER SIDE. The gate is
    // CONFIGURED-vs-not, probed per `commit_identity_fallback` — a
    // resolvability probe like
    // `git var GIT_COMMITTER_IDENT` is NOT reliable here (measured: on hosts
    // whose hostname yields a plausible domain, git auto-fabricates
    // `user@host.local` and the probe succeeds, silently baking a junk
    // identity into the user's repo; on a domainless container the same
    // auto-detection dies, as an early run on a test machine showed). Env identity
    // (`GIT_AUTHOR_EMAIL`/`GIT_COMMITTER_EMAIL`/`EMAIL`) outranks or
    // composes with `-c` config per git's own precedence, and a configured
    // repo/global identity wins outright because the fallback is then not
    // passed at all.
    let config_value = |key: &str| {
        git_command(&toplevel)
            .args(["config", key])
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
    };
    let config_name = config_value("user.name");
    let config_email = config_value("user.email");
    let env_ident = |k: &str| std::env::var(k).ok().filter(|v| !v.trim().is_empty());
    // COMMIT PHASE. The rollback envelope covers EXACTLY staging + `git
    // commit` — a failure there leaves HEAD unchanged, so the documented
    // `git revert` recovery does not exist yet and stranding rewritten,
    // staged sources would be worse than the starting state. Nothing
    // AFTER a successful commit may trigger this rollback: rolling back
    // past a commit that EXISTS would leave a migration commit at HEAD
    // with a reverse working tree while reporting failure.
    // Interrupt safepoint: before staging/commit — everything written is
    // rolled back (nothing is staged yet, so no unstage pass is needed).
    if (deps.interrupted)() {
        let notes = restore_tree(plan.files.len(), false, None, true);
        return Err(CliError::Validation(format!(
            "interrupted — every write was rolled back; nothing was \
             migrated. {}",
            render_rollback(notes)
        )));
    }
    // A post-commit failure must NEVER reach the
    // rollback envelope below — rolling back past a commit that EXISTS leaves
    // a migration commit at HEAD with a reverted working tree while reporting
    // "nothing was migrated", which is exactly the state this module forbids.
    // So the only post-commit condition allowed to return `Err` from the
    // closure is the one where the commit was successfully UNWOUND (there is
    // then no commit, and the rollback is correct). Everything else is
    // recorded here and reported after the envelope, in the same shape the
    // `rev-parse` read-back below already uses.
    let mut mixed_commit_note: Option<String> = None;
    let commit_result: CliResult<()> = (|| {
        for f in &plan.files {
            git_run(&["add", "--", f.rel.as_str()])?;
        }
        // The fallback is PER SIDE — only an incomplete side gets
        // the tool's `-c`, so a configured email is never displaced.
        let fallback = commit_identity_fallback(&IdentityProbe {
            config_user_name: config_name.as_deref(),
            config_user_email: config_email.as_deref(),
            env_email: env_ident("EMAIL").as_deref(),
            env_author_name: env_ident("GIT_AUTHOR_NAME").as_deref(),
            env_author_email: env_ident("GIT_AUTHOR_EMAIL").as_deref(),
            env_committer_name: env_ident("GIT_COMMITTER_NAME").as_deref(),
            env_committer_email: env_ident("GIT_COMMITTER_EMAIL").as_deref(),
        });
        let mut commit_args: Vec<&str> = Vec::new();
        if fallback.name {
            commit_args.extend(["-c", "user.name=cerulion ros2 migrate"]);
        }
        if fallback.email {
            commit_args.extend(["-c", "user.email=ros2-migrate@cerulion.invalid"]);
        }
        // The migration commits ONLY what it
        // staged, and REFUSES if the index carries anything else.
        //
        // The git clean-tree gate runs exactly once, early — before the
        // report is rendered and before the consent prompt, which is an
        // unbounded blocking read. The `git add --` loop above scopes what
        // this run STAGES, but a bare `git commit -m` commits the whole
        // INDEX as of commit time. So anything another actor staged while
        // the operator read the diff (a second terminal, an IDE, a
        // `pre-commit` hook re-staging with `git add -A`) was swept into the
        // migration commit — and the undo this verb documents, `git revert
        // <sha>`, would then revert that work too. On a scratch repository,
        // with an unrelated `theirs.txt` staged by a third
        // party, `git commit -m msg` produced a commit carrying `ours.cpp`
        // AND `theirs.txt`.
        //
        // The fix is a REFUSAL, not a pathspec, and the difference was
        // measured. Adding `-- <paths>` puts git in `--only` mode, which
        // commits from a TEMPORARY index; on the failure path that discards
        // the entry a `pre-commit` hook staged, which is exactly what the
        // rollback's ownership rule promises to preserve
        // (`a_hook_staged_edit_on_a_planned_path_survives_the_rollback`
        // fails under it: the index came back holding the ORIGINAL bytes
        // instead of the hook's). Refusing keeps git's commit semantics
        // untouched, so every tested rollback path still behaves as pinned,
        // and it matches what this verb already does with a dirty tree at
        // entry: refuse loudly and name the offender rather than guess.
        //
        // Both lists are read through git in the SAME relativity — the
        // second query is restricted to our own paths — so this never
        // depends on whether `f.rel` is workspace- or toplevel-relative.
        let staged_all = staged_paths(&git_run, &[])?;
        let planned_rel: Vec<&str> = plan.files.iter().map(|f| f.rel.as_str()).collect();
        let staged_ours = staged_paths(&git_run, &planned_rel)?;
        let foreign: Vec<&String> = staged_all
            .iter()
            .filter(|p| !staged_ours.contains(p))
            .collect();
        if !foreign.is_empty() {
            let names: Vec<&str> = foreign.iter().map(|s| s.as_str()).collect();
            return Err(CliError::Validation(format!(
                "the index carries {} change(s) this migration did not stage \
                 ({}) — they appeared after the clean-tree check, while the \
                 migration was being reviewed. Committing now would put them \
                 in the migration commit, and `git revert` would then take \
                 them with it. Unstage them (`git restore --staged <path>`) \
                 and re-run.",
                foreign.len(),
                names.join(", ")
            )));
        }
        commit_args.extend(["commit", "-m", commit_msg.as_str()]);
        git_run(&commit_args)?;
        // A SUCCESSFUL pre-commit HOOK stages INSIDE `git commit` — after
        // this verb's own index check and before the commit object is
        // written — so a hook that stages (`git add -A` is the common shape,
        // and auto-formatters do exactly that) puts foreign paths into the
        // migration commit.
        //
        // The rule: the hook is the user's problem; commit and warn.
        // So the commit STANDS and the operator is told. Neither
        // prevention nor refusal is offered: prevention has no window (the hook
        // runs inside git), the mechanism that would exclude its entries
        // (`--only`) breaks the rollback's ownership guarantee,
        // and this verb does not unwind a commit git has written.
        //
        // What the warning owes the operator is the SAFE UNDO, because the
        // one this verb advertises is not safe here: `git revert <sha>` would
        // take the hook's paths with it. `git apply -R <patch>` reverses only
        // the migration's own edits, which is why the patch file exists. The
        // same qualification is in docs/user-api.md and in `--help`, attached to
        // the one-commit promise rather than left for the operator to infer.
        // `-z`, for the same reason
        // `staged_paths` uses it a few lines above and `parse_porcelain_z`
        // documents. Without it, `core.quotePath` — git's DEFAULT — returns
        // a path containing a non-ASCII byte, `"`, `\` or a control
        // character in C-QUOTED display form (`"src/pkg/caf\303\251.cpp"`),
        // while `plan.files[].rel` is the raw path. The comparison below then
        // calls the operator's OWN migrated file "unplanned", fires the
        // hook warning with no hook installed, and tells them `git revert` is
        // unsafe when it is not. `-z` emits raw bytes and NUL separators, so
        // it also carries leading/trailing spaces, which git does not quote
        // and `trim` would have eaten.
        let read_back = match git_run(&["show", "--name-only", "--format=", "-z", "HEAD"]) {
            Ok(out) => out,
            Err(e) => {
                // The commit exists; failing to READ it cannot justify
                // reverting the tree.
                mixed_commit_note = Some(format!(
                    "the migration commit SUCCEEDED but its contents could \
                     not be read back ({e}), so this run cannot say whether a \
                     pre-commit hook added paths to it. Check `git show \
                     --name-only HEAD` before relying on `git revert`."
                ));
                return Ok(());
            }
        };
        let committed: Vec<String> = read_back
            .split('\0')
            .filter(|l| !l.is_empty())
            .map(str::to_string)
            .collect();
        let planned: Vec<&str> = plan.files.iter().map(|f| f.rel.as_str()).collect();
        let unplanned: Vec<&String> = committed
            .iter()
            .filter(|p| !planned.iter().any(|q| q == &p.as_str()))
            .collect();
        // The recommendation is validated
        // before it is made. A hook that also EDITS a file this migration
        // wrote — an auto-formatter touching the rewritten region is the
        // realistic shape — invalidates the patch, which was generated
        // before the hook ran. Recommending `git apply -R` there hands the
        // operator a command that fails with "patch does not apply", in
        // exactly the state the warning exists to help them out of; a
        // warning that names a broken recovery is worse than none.
        //
        // Checked with git itself rather than reasoned about, and the check
        // also covers the case with NO unplanned path: a hook that only
        // reformats a planned source breaks the advertised undo while
        // nothing else looks wrong, and before this that ran silent.
        let patch_arg = ws.join(PATCH_FILENAME);
        let patch_arg = patch_arg.to_string_lossy().to_string();
        let patch_reverses = git_run(&["apply", "-R", "--check", patch_arg.as_str()]).is_ok();
        if !unplanned.is_empty() || !patch_reverses {
            let names: String = unplanned
                .iter()
                .map(|s| s.as_str())
                .collect::<Vec<&str>>()
                .join(", ");
            tracing::warn!(
                paths = %names,
                patch_reverses,
                "a pre-commit hook changed what this run committed"
            );
            mixed_commit_note = Some(match (unplanned.is_empty(), patch_reverses) {
                // Unplanned paths only: the patch still reverses exactly the
                // migration's edits, so it IS the selective undo.
                (false, true) => format!(
                    "the migration commit also carries {} path(s) this run \
                     did not plan ({}). A pre-commit hook stages INSIDE `git \
                     commit`, after this verb's own index check, so its \
                     additions land in the commit. SAFE UNDO for this run: \
                     `git apply -R {}` — that reverses only the migration's \
                     own edits. `git revert <sha>` is NOT equivalent while \
                     this warning stands: it would undo the hook's paths as \
                     well.",
                    unplanned.len(),
                    names,
                    PATCH_FILENAME
                ),
                // Both: neither undo is safe, and saying so is the whole point.
                (false, false) => format!(
                    "the migration commit also carries {} path(s) this run \
                     did not plan ({}), AND `git apply -R {}` no longer \
                     reverses it (checked with `git apply -R --check`). There \
                     is no single-command undo for that state: the patch does \
                     not apply, and `git revert <sha>` would delete those \
                     paths. Inspect `git show HEAD` and split the commit by \
                     hand.",
                    unplanned.len(),
                    names,
                    PATCH_FILENAME
                ),
                // Hook edited a planned file only: revert still works, the
                // patch does not — say which one to reach for.
                // The CAUSE is deliberately not asserted. A hook editing a
                // planned file is the common one, but it is not the only
                // one — a source carrying non-UTF-8 bytes near an edit also
                // produces a patch that cannot reverse —
                // and this check observes the EFFECT, never the cause.
                // Naming a cause it has not established would be the same
                // mistake as recommending an undo it has not validated.
                (true, false) => format!(
                    "`git apply -R {}` does not reverse this commit (checked \
                     with `git apply -R --check`): something changed a file \
                     this migration wrote after the patch was generated — a \
                     `pre-commit` hook is the usual cause — or the patch \
                     could not represent that file's bytes exactly. Use `git \
                     revert HEAD` instead, which reverses the whole commit.",
                    PATCH_FILENAME
                ),
                (true, true) => unreachable!("guarded by the condition above"),
            });
        }
        Ok(())
    })();
    if let Err(commit_err) = commit_result {
        let notes = restore_tree(plan.files.len(), true, None, true);
        return Err(CliError::Validation(format!(
            "the migration commit FAILED: {commit_err} — nothing was \
             migrated. {}",
            render_rollback(notes)
        )));
    }
    // A commit that exists but could not be unwound (or could not be read
    // back) is reported here, OUTSIDE the rollback envelope: the tree and HEAD
    // are consistent and must stay that way. Best-effort output — a broken
    // pipe must not turn a real commit into a reported failure — with the
    // structured log as the durable record.
    if let Some(note) = mixed_commit_note {
        tracing::warn!(note = %note, "the migration commit needs manual review");
        let _ = writeln!(deps.out, "warning: {note}");
    }
    // POST-COMMIT: the commit exists from here on. A failure reading its id
    // back is reported loudly WITHOUT any rollback — the tree and HEAD are
    // consistent, only this process's knowledge of the sha is missing.
    let commit = git_run(&["rev-parse", "HEAD"]).map_err(|e| {
        CliError::Validation(format!(
            "the migration commit SUCCEEDED but reading its id back failed: \
             {e}. The commit is in place (see `git log -1` in '{}') and the \
             working tree was NOT rolled back; undo remains `git revert \
             <that commit>` or `git apply -R {}`.",
            toplevel.display(),
            PATCH_FILENAME
        ))
    })?;
    writeln!(
        deps.out,
        "applied: {} file(s) rewritten; commit {} ; patch {}",
        plan.files.len(),
        commit,
        patch_path.display()
    )?;

    // Manifest: the applied candidates are CONSUMED; re-key to the new HEAD.
    let post_git = gather_git_info(&ws);
    let manifest = MigrateManifest {
        schema: MANIFEST_SCHEMA,
        version: MANIFEST_VERSION,
        workspace_key: ManifestWorkspaceKey {
            head: post_git.head.clone(),
            dirty: post_git.status_error.is_some() || any_tracked_change(&post_git.status),
        },
        generated_by: "write".to_string(),
        counts: ManifestCounts {
            packages: 0,
            call_sites: 0,
            manual_candidates: analysis.candidates.len(),
            rclpy_nodes: rclpy.len(),
        },
        candidates: Vec::new(),
        manual_candidates: manifest_manual,
        rclpy: rclpy.clone(),
        decision: existing_decision(&ws),
    };
    if let Err(e) = write_manifest_atomic(&ws, &manifest) {
        // Deliberately warn-and-proceed HERE (unlike the dry-run, which
        // fails): the commit and the build are the user's product and both
        // are real at this point, while a stale manifest is SELF-INVALIDATING
        // — its workspace_key still names the pre-migration HEAD, so any
        // consumer discards it and the next dry-run rewrites it. Failing an
        // applied migration over a cache write would be strictly worse.
        tracing::warn!(error = %e, "manifest refresh failed");
        writeln!(deps.out, "warning: manifest refresh failed: {e}")?;
    }

    // END OF THE WRITE BATCH. The tree, the commit and the manifest are all
    // final; the build below runs a compiler over the workspace and must not
    // hold a mutation lock for its duration.
    drop(write_lock);

    // Automatic build of the affected packages.
    writeln!(
        deps.out,
        "building affected package(s): colcon build --packages-select {}",
        plan.affected_packages.join(" ")
    )?;
    let build_error = match (deps.build_runner)(&ws, &plan.affected_packages) {
        Ok(true) => None,
        Ok(false) => Some(format!(
            "colcon build --packages-select {} FAILED. The migration commit \
             {} is in place — revert it with `git revert {}` (or `git apply \
             -R {}`), fix, and re-run.",
            plan.affected_packages.join(" "),
            commit,
            commit,
            PATCH_FILENAME
        )),
        Err(e) => Some(format!(
            "could not run colcon to build the affected package(s): {e}. \
             The migration commit {} is in place — build manually with \
             `colcon build --packages-select {}`, or revert with `git \
             revert {}` (or `git apply -R {}`).",
            commit,
            plan.affected_packages.join(" "),
            commit,
            PATCH_FILENAME
        )),
    };
    Ok(MigrateOutcome::Applied {
        commit,
        affected_packages: plan.affected_packages.clone(),
        build_error,
    })
}

// ---------------------------------------------------------------------------
// Pure-function unit tests (oracle vectors; filesystem-heavy coverage lives
// in tests/ros2_migrate_test.rs).
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// The three lock refusals send the user to three
    /// DIFFERENT places, and the one thing no compiler can catch is an arm
    /// pointing at the wrong one — for example both
    /// internal-bug refusals arriving as `Failed` and wrapped in "fix that
    /// path — a `.cerulion` that is a symlink or not a directory, a permission
    /// on it, or a filesystem that cannot lock".
    ///
    /// Each arm is asserted to CARRY its own remedy and to LACK the other two,
    /// because "contains the right words" passes just as well on a build that
    /// concatenated all three.
    ///
    /// PURE is the right level for the `Internal` arm, and that is a finding
    /// rather than a convenience. An end-to-end arm driving
    /// `run_migrate` to an `Internal` through a `deps.interrupted` predicate
    /// that re-enters the lock does not reach: `acquire_with` hands the
    /// re-entrancy refusal to the call that finds its OWN unpromoted
    /// reservation — the INNER one, i.e. the predicate's — while migrate's
    /// acquire is the outer one and keeps polling. The attempt HUNG the suite
    /// rather than failing it, which is how the premise was caught. The other
    /// `Internal` cause (the reservation vanishing mid-acquire) is documented
    /// unreachable at its own site. So no production path drives migrate to
    /// this arm today; it is typed and worded correctly for the day one does,
    /// and the oracle below is what keeps it so.
    #[test]
    fn each_lock_refusal_carries_its_own_remedy_and_not_another_arms() {
        let lock_path = Path::new("/ws/.cerulion/workspace.lock");
        let lock_dir = Path::new("/ws/.cerulion");
        let cause = || CliError::Validation("THE UNDERLYING CAUSE".to_string());
        let text =
            |error: &AcquireError| workspace_lock_refusal(error, lock_path, lock_dir).to_string();

        // The remedy fragments, one per arm. None may appear in another's text.
        const QUEUED: &str = "re-run once it finishes";
        const YOUR_FILESYSTEM: &str = "fix that path";
        const OUR_BUG: &str = "Nothing is wrong with your workspace";

        let interrupted = text(&AcquireError::Interrupted);
        let failed = text(&AcquireError::Failed(cause()));
        let internal = text(&AcquireError::Internal(cause()));

        for (label, body, mine, theirs) in [
            (
                "Interrupted",
                &interrupted,
                QUEUED,
                [YOUR_FILESYSTEM, OUR_BUG],
            ),
            ("Failed", &failed, YOUR_FILESYSTEM, [QUEUED, OUR_BUG]),
            ("Internal", &internal, OUR_BUG, [QUEUED, YOUR_FILESYSTEM]),
        ] {
            assert!(
                body.contains(mine),
                "{label} must carry its own remedy ({mine}); got: {body}"
            );
            for other in theirs {
                assert!(
                    !body.contains(other),
                    "{label} carries ANOTHER arm's remedy ({other}) — that is the defect, \
                     not the fix; got: {body}"
                );
            }
            assert!(
                body.contains("nothing was committed"),
                "{label} must still say what is TRUE of the workspace; got: {body}"
            );
            assert!(
                body.contains("/ws/.cerulion"),
                "{label} must name the lock path it is talking about; got: {body}"
            );
        }

        // The two that carry a CAUSE must interpolate it — an internal-bug
        // refusal that hid the cause would leave the user with nothing to report.
        assert!(failed.contains("THE UNDERLYING CAUSE"), "got: {failed}");
        assert!(internal.contains("THE UNDERLYING CAUSE"), "got: {internal}");
        // And the internal one must say whose bug it is, in so many words: a
        // user who reads only the first clause must not go looking at their
        // own tree.
        assert!(
            internal.contains("BUG IN"),
            "the internal-bug arm must say so up front; got: {internal}"
        );
        assert!(
            internal.contains("https://github.com/cerulion-inc/cerulion/issues"),
            "and must say where to report it; got: {internal}"
        );
    }

    fn edit(offset: usize, original: &str, replacement: &str) -> ToolEdit {
        ToolEdit {
            offset,
            length: original.len(),
            original: original.to_string(),
            replacement: replacement.to_string(),
        }
    }

    #[test]
    fn apply_edits_replaces_bottom_up_against_hand_oracle() {
        let src = b"alpha\nbravo\ncharlie\n";
        let edits = vec![edit(0, "alpha", "A"), edit(12, "charlie", "CHARLIE!")];
        let out = apply_edits(src, &edits).expect("applies");
        assert_eq!(out, b"A\nbravo\nCHARLIE!\n");
    }

    #[test]
    fn apply_edits_refuses_a_changed_file() {
        let src = b"alpha\n";
        let edits = vec![edit(0, "ALPHA", "x")];
        let err = apply_edits(src, &edits).unwrap_err().to_string();
        assert!(
            err.contains("no longer match"),
            "want the changed-since-analysis refusal, got: {err}"
        );
    }

    #[test]
    fn apply_edits_refuses_out_of_bounds() {
        let src = b"ab\n";
        let edits = vec![edit(1, "b\nX", "y")];
        let err = apply_edits(src, &edits).unwrap_err().to_string();
        assert!(err.contains("out of bounds"), "got: {err}");
    }

    #[test]
    fn merge_dedupes_identical_header_rewrites_and_demotes_conflicts() {
        let rw = |offset: usize, repl: &str| ToolRewrite {
            file: "/ws/src/pkg/include/node.hpp".to_string(),
            function: "N::tick".to_string(),
            kind: "unique_ptr".to_string(),
            message_type: "std_msgs::msg::String_<std::allocator<void>>".to_string(),
            publisher: "pub_".to_string(),
            line: 10,
            edits: vec![edit(offset, "old", repl)],
        };
        // Identical rewrites from two TUs collapse to one.
        let a = ToolOutput {
            format: 1,
            tool_version: String::new(),
            file: "/ws/src/pkg/src/a.cpp".to_string(),
            rewrites: vec![rw(5, "new")],
            candidates: vec![],
        };
        let b = ToolOutput {
            format: 1,
            tool_version: String::new(),
            file: "/ws/src/pkg/src/b.cpp".to_string(),
            rewrites: vec![rw(5, "new")],
            candidates: vec![],
        };
        let merged = merge_tool_outputs(&[a.clone(), b]);
        assert_eq!(merged.rewrites.len(), 1);
        assert!(merged.candidates.is_empty());

        // DIFFERENT bytes for an overlapping region demote the whole file.
        let c = ToolOutput {
            format: 1,
            tool_version: String::new(),
            file: "/ws/src/pkg/src/c.cpp".to_string(),
            rewrites: vec![rw(6, "other")],
            candidates: vec![],
        };
        let merged = merge_tool_outputs(&[a, c]);
        assert!(merged.rewrites.is_empty(), "conflicting file must demote");
        // Both demoted rewrites share (file, line), so the candidate list
        // carries ONE conflicting-analyses entry for the site.
        assert_eq!(merged.candidates.len(), 1);
        assert_eq!(merged.candidates[0].reason, "conflicting-analyses");
        assert_eq!(merged.candidates[0].line, 10);
    }

    #[test]
    fn unified_diff_renders_the_hand_written_hunk() {
        let old = b"one\ntwo\nthree\nfour\nfive\nsix\nseven\neight\n";
        // Replace "four" with two lines.
        let edits = vec![edit(14, "four", "FOUR-A\nFOUR-B")];
        let new = apply_edits(old, &edits).expect("applies");
        let diff = unified_diff("src/x.cpp", old, &new, &edits);
        let want = "--- a/src/x.cpp\n\
                    +++ b/src/x.cpp\n\
                    @@ -1,7 +1,8 @@\n\
                    \x20one\n\
                    \x20two\n\
                    \x20three\n\
                    -four\n\
                    +FOUR-A\n\
                    +FOUR-B\n\
                    \x20five\n\
                    \x20six\n\
                    \x20seven\n";
        assert_eq!(diff, want);
    }

    #[test]
    fn unified_diff_keeps_lines_between_edits_as_context() {
        // The chosen rewrite shape: decl edit + publish edit with the fill
        // line(s) between them UNTOUCHED — they must render as context,
        // never as a -/+ pair.
        let old = b"head\ndecl\nfill\ncall\ntail\n";
        let edits = vec![edit(5, "decl", "DECL-A\nDECL-B"), edit(15, "call", "CALL")];
        let new = apply_edits(old, &edits).expect("applies");
        let diff = unified_diff("f", old, &new, &edits);
        let want = "--- a/f\n\
                    +++ b/f\n\
                    @@ -1,5 +1,6 @@\n\
                    \x20head\n\
                    -decl\n\
                    +DECL-A\n\
                    +DECL-B\n\
                    \x20fill\n\
                    -call\n\
                    +CALL\n\
                    \x20tail\n";
        assert_eq!(diff, want);
    }

    #[test]
    fn unified_diff_two_far_edits_render_two_hunks_with_correct_offsets() {
        let mut old = String::new();
        for i in 1..=30 {
            old.push_str(&format!("line{i:02}\n"));
        }
        let old = old.into_bytes();
        // "line03" starts at offset 14; "line25" at 24*7=168.
        let edits = vec![edit(14, "line03", "L3A\nL3B"), edit(168, "line25", "L25")];
        let new = apply_edits(&old, &edits).expect("applies");
        let diff = unified_diff("f", &old, &new, &edits);
        assert!(
            diff.contains("@@ -1,6 +1,7 @@"),
            "hunk 1 header in:\n{diff}"
        );
        // Second hunk: old line 25 with 3 lines context each side = start 22,
        // count 7; new side shifted by +1 line from hunk 1.
        assert!(
            diff.contains("@@ -22,7 +23,7 @@"),
            "hunk 2 header in:\n{diff}"
        );
        assert!(diff.contains("-line25\n+L25\n"), "hunk 2 body in:\n{diff}");
    }

    #[test]
    fn unified_diff_no_trailing_newline_carries_the_marker() {
        let old = b"a\nb\nlast";
        let edits = vec![edit(4, "last", "LAST")];
        let new = apply_edits(old, &edits).expect("applies");
        let diff = unified_diff("f", old, &new, &edits);
        assert!(
            diff.contains("-last\n\\ No newline at end of file\n"),
            "old-side marker in:\n{diff}"
        );
        assert!(
            diff.contains("+LAST\n\\ No newline at end of file\n"),
            "new-side marker in:\n{diff}"
        );
    }

    #[test]
    fn package_name_from_xml_extracts_and_rejects() {
        assert_eq!(
            package_name_from_xml("<package><name>demo_pkg</name></package>"),
            Some("demo_pkg".to_string())
        );
        assert_eq!(package_name_from_xml("<package></package>"), None);
        assert_eq!(package_name_from_xml("<name></name>"), None);
        // A <name> inside an XML comment must never win over the real
        // element (it would misdirect the affected-package derivation and
        // the automatic colcon build).
        assert_eq!(
            package_name_from_xml(
                "<!-- template: <name>fill_me_in</name> -->\n\
                 <package><name>real_pkg</name></package>"
            ),
            Some("real_pkg".to_string())
        );
        // A document whose ONLY <name> is commented out has no name.
        assert_eq!(
            package_name_from_xml("<!-- <name>ghost</name> --><package/>"),
            None
        );
        // An unterminated comment swallows the tail rather than resurrecting
        // commented content.
        assert_eq!(package_name_from_xml("<!-- <name>ghost</name>"), None);
        // The name is the DIRECT child of <package> — a nested
        // <name> that appears FIRST (an export, a custom element) must not
        // win, whatever its depth.
        assert_eq!(
            package_name_from_xml(
                "<?xml version=\"1.0\"?>\n<package format=\"3\">\n  <export>\n    \
                 <name>plugin_name</name>\n    <deep><name>deeper</name></deep>\n  \
                 </export>\n  <name>real_pkg</name>\n</package>"
            ),
            Some("real_pkg".to_string())
        );
        // A <name> BEFORE the root element is not the package's.
        assert_eq!(
            package_name_from_xml("<name>stray</name><package><name>real_pkg</name></package>"),
            Some("real_pkg".to_string())
        );
        // Self-closing siblings before the name do not change depth.
        assert_eq!(
            package_name_from_xml("<package><license/><name>real_pkg</name></package>"),
            Some("real_pkg".to_string())
        );
        // Only a NESTED name -> the package has no direct name.
        assert_eq!(
            package_name_from_xml("<package><export><name>x</name></export></package>"),
            None
        );
        // A `<packageX>` element is not the root; `<package/>` has none.
        assert_eq!(
            package_name_from_xml("<packageX><name>no</name></packageX><package/>"),
            None
        );
        // A blank direct name is no name.
        assert_eq!(
            package_name_from_xml("<package><name>  </name></package>"),
            None
        );
    }

    /// The `.py` walker enqueues only
    /// non-symlink REGULAR files — a writerless FIFO named `x.py` (or a
    /// symlink to one) would otherwise wedge `scan_rclpy`'s `read_to_string`
    /// forever. Bounded by a channel timeout so a regression FAILS rather
    /// than hangs the suite.
    #[cfg(unix)]
    #[test]
    fn the_py_walker_enqueues_only_regular_non_symlink_files() {
        use std::os::unix::ffi::OsStrExt as _;
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        std::fs::write(root.join("node.py"), "import rclpy\n").unwrap();
        let fifo = root.join("fifo.py");
        let c = std::ffi::CString::new(fifo.as_os_str().as_bytes()).unwrap();
        assert_eq!(unsafe { libc::mkfifo(c.as_ptr(), 0o644) }, 0, "mkfifo");
        std::os::unix::fs::symlink(root.join("node.py"), root.join("link.py")).unwrap();
        std::os::unix::fs::symlink(&fifo, root.join("fifo_link.py")).unwrap();
        let (tx, rx) = std::sync::mpsc::channel();
        let r2 = root.clone();
        std::thread::spawn(move || {
            let mut files = Vec::new();
            walk_py_files(&r2, 0, &mut files);
            let scanned = scan_rclpy(&r2, &r2).len();
            let _ = tx.send((files, scanned));
        });
        let (files, _scanned) = rx
            .recv_timeout(std::time::Duration::from_secs(20))
            .expect("the walk + scan must not block on the FIFO");
        assert_eq!(
            files,
            vec![root.join("node.py")],
            "only the regular, non-symlink .py file may be enqueued"
        );
    }

    /// The manifest is written only inside
    /// the workspace object — a `.cerulion` that is a symlink (to anywhere)
    /// or a file is refused and nothing lands outside; the healthy control
    /// writes and replaces the manifest with no temp residue.
    #[cfg(unix)]
    #[test]
    fn the_manifest_is_never_written_through_a_symlinked_cerulion_dir() {
        let ws_tmp = tempfile::tempdir().expect("tempdir");
        let ws = ws_tmp.path().canonicalize().unwrap();
        let outside_tmp = tempfile::tempdir().expect("tempdir");
        let outside = outside_tmp.path().canonicalize().unwrap();
        let manifest = MigrateManifest {
            schema: MANIFEST_SCHEMA,
            version: MANIFEST_VERSION,
            workspace_key: ManifestWorkspaceKey {
                head: None,
                dirty: false,
            },
            generated_by: "test".to_string(),
            counts: ManifestCounts {
                packages: 0,
                call_sites: 0,
                manual_candidates: 0,
                rclpy_nodes: 0,
            },
            candidates: vec![],
            manual_candidates: vec![],
            rclpy: vec![],
            decision: serde_json::Value::Null,
        };
        std::os::unix::fs::symlink(&outside, ws.join(".cerulion")).unwrap();
        let err = write_manifest_atomic(&ws, &manifest)
            .expect_err("a symlinked .cerulion must refuse")
            .to_string();
        assert!(err.contains("SYMLINK"), "{err}");
        assert!(
            std::fs::read_dir(&outside).unwrap().next().is_none(),
            "nothing may be written outside the workspace"
        );
        std::fs::remove_file(ws.join(".cerulion")).unwrap();
        std::fs::write(ws.join(".cerulion"), b"not a directory").unwrap();
        let err = write_manifest_atomic(&ws, &manifest)
            .expect_err("a file at .cerulion must refuse")
            .to_string();
        assert!(err.contains("real directory"), "{err}");
        std::fs::remove_file(ws.join(".cerulion")).unwrap();
        let p = write_manifest_atomic(&ws, &manifest).expect("healthy write");
        assert_eq!(p, ws.join(MANIFEST_REL_PATH));
        assert!(p.is_file());
        write_manifest_atomic(&ws, &manifest).expect("atomic replace");
        let entries: Vec<String> = std::fs::read_dir(ws.join(".cerulion"))
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .collect();
        assert_eq!(
            entries,
            vec![Path::new(MANIFEST_REL_PATH)
                .file_name()
                .unwrap()
                .to_string_lossy()
                .into_owned()],
            "no temp residue"
        );
    }

    fn empty_manifest() -> MigrateManifest {
        MigrateManifest {
            schema: MANIFEST_SCHEMA,
            version: MANIFEST_VERSION,
            workspace_key: ManifestWorkspaceKey {
                head: None,
                dirty: false,
            },
            generated_by: "test".to_string(),
            counts: ManifestCounts {
                packages: 0,
                call_sites: 0,
                manual_candidates: 0,
                rclpy_nodes: 0,
            },
            candidates: vec![],
            manual_candidates: vec![],
            rclpy: vec![],
            decision: serde_json::Value::Null,
        }
    }

    /// A temp left by a killed run under the
    /// OLD predictable `.<name>.tmp-<pid>` scheme — for THIS very pid — no
    /// longer blocks the refresh, and is never touched; the minted names
    /// carry a per-attempt nonce.
    #[test]
    fn a_stale_pid_named_temp_never_blocks_the_manifest_refresh() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let ws = tmp.path().canonicalize().unwrap();
        let dir = ws.join(".cerulion");
        std::fs::create_dir(&dir).unwrap();
        let name = Path::new(MANIFEST_REL_PATH)
            .file_name()
            .unwrap()
            .to_string_lossy()
            .into_owned();
        let stale = dir.join(format!(".{name}.tmp-{}", std::process::id()));
        std::fs::write(&stale, b"left behind by a killed run").unwrap();
        let p = write_manifest_atomic(&ws, &empty_manifest()).expect("the refresh must succeed");
        assert!(p.is_file());
        assert_eq!(
            std::fs::read(&stale).unwrap(),
            b"left behind by a killed run".to_vec(),
            "a stale temp is never touched"
        );
        let minted = fresh_manifest_temp_name(&name, 0);
        assert!(minted.starts_with(&format!(".{name}.tmp.")), "{minted}");
        assert_ne!(
            fresh_manifest_temp_name(&name, 0),
            fresh_manifest_temp_name(&name, 0),
            "consecutive mints must differ"
        );
    }

    /// Occupied candidates are skipped — retried past, the
    /// occupants byte-untouched — and an endless occupancy gives up loudly
    /// after the bounded attempts with the manifest still unwritten.
    #[test]
    fn manifest_temp_minting_retries_past_occupied_candidates_and_gives_up_loudly() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let ws = tmp.path().canonicalize().unwrap();
        let dir = ws.join(".cerulion");
        std::fs::create_dir(&dir).unwrap();
        for i in 0..3 {
            std::fs::write(dir.join(format!("occupied-{i}")), format!("occupant {i}")).unwrap();
        }
        let mut seen = Vec::new();
        let mut mint = |_: &str, attempt: usize| {
            seen.push(attempt);
            if attempt < 3 {
                format!("occupied-{attempt}")
            } else {
                "free-candidate".to_string()
            }
        };
        let p = write_manifest_atomic_with(&ws, &empty_manifest(), &mut mint)
            .expect("the fourth candidate is free");
        assert!(p.is_file());
        assert_eq!(
            seen,
            vec![0, 1, 2, 3],
            "every occupied candidate is retried past"
        );
        for i in 0..3 {
            assert_eq!(
                std::fs::read_to_string(dir.join(format!("occupied-{i}"))).unwrap(),
                format!("occupant {i}"),
                "an occupant is never touched"
            );
        }
        assert!(
            !dir.join("free-candidate").exists(),
            "the temp was renamed away"
        );
        // Endless occupancy: bounded, loud, nothing written.
        std::fs::remove_file(&p).unwrap();
        let mut always = |_: &str, _: usize| "occupied-0".to_string();
        let err = write_manifest_atomic_with(&ws, &empty_manifest(), &mut always)
            .expect_err("every candidate occupied must refuse")
            .to_string();
        assert!(err.contains("free temporary name"), "{err}");
        assert!(err.contains("occupied-0"), "{err}");
        assert!(
            !p.exists(),
            "nothing may be written when no candidate is free"
        );
        assert_eq!(
            std::fs::read_to_string(dir.join("occupied-0")).unwrap(),
            "occupant 0"
        );
    }

    #[test]
    fn classify_rclpy_source_oracle() {
        assert_eq!(classify_rclpy_source("import os\n"), None);
        assert_eq!(classify_rclpy_source("import rclpy\n"), Some(0));
        assert_eq!(
            classify_rclpy_source(
                "from rclpy.node import Node\n\
                 p = self.create_publisher(String, 't', 10)\n\
                 q = node.create_publisher(String, 'u', 10)\n"
            ),
            Some(2)
        );
    }

    #[test]
    fn classify_dirty_oracle() {
        let z = parse_porcelain_z;
        assert_eq!(classify_dirty(&z(b""), &[]), DirtyVerdict::Clean);
        // Untracked files are tolerated…
        assert_eq!(
            classify_dirty(&z(b"?? build/\0?? notes.txt\0"), &["src/a.cpp".into()]),
            DirtyVerdict::Clean
        );
        // …unless the migration would edit one.
        assert_eq!(
            classify_dirty(&z(b"?? src/a.cpp\0"), &["src/a.cpp".into()]),
            DirtyVerdict::Dirty {
                tracked: vec![],
                untracked_planned: vec!["src/a.cpp".into()]
            }
        );
        // Any tracked change refuses.
        assert_eq!(
            classify_dirty(&z(b" M src/b.cpp\0?? build/\0"), &[]),
            DirtyVerdict::Dirty {
                tracked: vec!["src/b.cpp".into()],
                untracked_planned: vec![]
            }
        );
    }

    #[test]
    fn porcelain_z_parses_exotic_paths_and_renames_verbatim() {
        // The write-gate bypass class: newline porcelain C-QUOTES a path
        // holding a quote character, so a display-form comparison misses
        // it. `-z` output carries the bytes VERBATIM — an untracked
        // `src/talk"er.cpp` the migration would edit must refuse.
        let planned = vec!["src/talk\"er.cpp".to_string()];
        let entries = parse_porcelain_z(b"?? src/talk\"er.cpp\0");
        assert_eq!(
            entries,
            vec![PorcelainEntry {
                code: "??".into(),
                path: "src/talk\"er.cpp".into(),
                orig_path: None
            }]
        );
        assert_eq!(
            classify_dirty(&entries, &planned),
            DirtyVerdict::Dirty {
                tracked: vec![],
                untracked_planned: vec!["src/talk\"er.cpp".into()]
            }
        );
        // A rename entry carries its ORIGINAL path as a second NUL field —
        // it must consume that field, not misread it as another entry.
        let entries = parse_porcelain_z(b"R  src/new.cpp\0src/old.cpp\0?? x\0");
        assert_eq!(entries.len(), 2);
        assert_eq!(entries[0].code, "R ");
        assert_eq!(entries[0].path, "src/new.cpp");
        assert_eq!(entries[0].orig_path.as_deref(), Some("src/old.cpp"));
        assert_eq!(entries[1].code, "??");
        assert!(any_tracked_change(&entries));
    }

    #[test]
    fn commit_identity_fallback_decision_oracle() {
        let both = IdentityFallback {
            name: true,
            email: true,
        };
        let none = IdentityFallback {
            name: false,
            email: false,
        };
        let name_only = IdentityFallback {
            name: true,
            email: false,
        };
        let email_only = IdentityFallback {
            name: false,
            email: true,
        };
        let f = |p: IdentityProbe<'_>| commit_identity_fallback(&p);
        // The container shape: nothing configured anywhere -> both sides.
        assert_eq!(f(IdentityProbe::default()), both);
        // Whitespace/empty values are NOT identity.
        assert_eq!(
            f(IdentityProbe {
                config_user_name: Some(" "),
                config_user_email: Some(""),
                env_email: Some("  "),
                ..Default::default()
            }),
            both
        );
        // A configured EMAIL alone is HALF an identity — the name
        // side still takes the fallback, the email side does not (so the
        // configured email is never displaced by the tool's).
        assert_eq!(
            f(IdentityProbe {
                config_user_email: Some("owner@example.invalid"),
                ..Default::default()
            }),
            name_only
        );
        // ... and a configured NAME alone is the mirror image.
        assert_eq!(
            f(IdentityProbe {
                config_user_name: Some("Owner"),
                ..Default::default()
            }),
            email_only
        );
        // Both configured -> nothing.
        assert_eq!(
            f(IdentityProbe {
                config_user_name: Some("Owner"),
                config_user_email: Some("owner@example.invalid"),
                ..Default::default()
            }),
            none
        );
        // EMAIL is git's own fallback and `-c` would wrongly outrank it —
        // it settles the email side only.
        assert_eq!(
            f(IdentityProbe {
                env_email: Some("someone@example.invalid"),
                ..Default::default()
            }),
            name_only
        );
        // BOTH per-side env addresses + BOTH per-side env names ->
        // resolvable on both sides -> nothing.
        assert_eq!(
            f(IdentityProbe {
                env_author_name: Some("A"),
                env_author_email: Some("a@example.invalid"),
                env_committer_name: Some("C"),
                env_committer_email: Some("c@example.invalid"),
                ..Default::default()
            }),
            none
        );
        // ONE-SIDED env identity still needs the fallback for that side
        // (env outranks -c for its own side; the other side would refuse
        // the commit) — per side.
        assert_eq!(
            f(IdentityProbe {
                env_author_email: Some("a@example.invalid"),
                ..Default::default()
            }),
            both
        );
        assert_eq!(
            f(IdentityProbe {
                config_user_email: Some("owner@example.invalid"),
                env_author_name: Some("A"),
                ..Default::default()
            }),
            name_only
        );
        assert_eq!(
            f(IdentityProbe {
                config_user_name: Some("Owner"),
                env_committer_email: Some("c@example.invalid"),
                ..Default::default()
            }),
            email_only
        );
    }

    /// GIT_REPO_REDIRECT: the keep-list is exact — config and
    /// identity pass, every redirection / program-substitution / unknown
    /// `GIT_*` is stripped.
    #[test]
    fn git_env_policy_keeps_config_and_identity_and_strips_everything_else() {
        for kept in [
            "GIT_CONFIG_GLOBAL",
            "GIT_CONFIG_SYSTEM",
            "GIT_CONFIG_NOSYSTEM",
            "GIT_CONFIG_COUNT",
            "GIT_CONFIG_KEY_0",
            "GIT_CONFIG_VALUE_0",
            "GIT_CONFIG_KEY_12",
            "GIT_CONFIG_VALUE_12",
            "GIT_CONFIG_PARAMETERS",
            "GIT_ATTR_NOSYSTEM",
            "GIT_AUTHOR_NAME",
            "GIT_AUTHOR_EMAIL",
            "GIT_AUTHOR_DATE",
            "GIT_COMMITTER_NAME",
            "GIT_COMMITTER_EMAIL",
            "GIT_COMMITTER_DATE",
        ] {
            assert!(git_env_var_kept(kept), "{kept} must pass through");
        }
        for stripped in [
            "GIT_DIR",
            "GIT_WORK_TREE",
            "GIT_INDEX_FILE",
            "GIT_OBJECT_DIRECTORY",
            "GIT_ALTERNATE_OBJECT_DIRECTORIES",
            "GIT_COMMON_DIR",
            "GIT_NAMESPACE",
            "GIT_CEILING_DIRECTORIES",
            "GIT_DISCOVERY_ACROSS_FILESYSTEM",
            "GIT_EXEC_PATH",
            "GIT_TEMPLATE_DIR",
            "GIT_SSH",
            "GIT_SSH_COMMAND",
            "GIT_ASKPASS",
            "GIT_EDITOR",
            "GIT_SEQUENCE_EDITOR",
            "GIT_PAGER",
            "GIT_EXTERNAL_DIFF",
            "GIT_PROXY_COMMAND",
            "GIT_TRACE",
            "GIT_FUTURE_UNKNOWN",
            "GIT_CONFIG",
            "GIT_CONFIG_KEY",
        ] {
            assert!(!git_env_var_kept(stripped), "{stripped} must be stripped");
        }
    }

    /// Every read of a workspace-supplied file goes
    /// through the ONE bounded seam. This is the structural half of the
    /// guarantee, and it is the half that has to be structural: each of the five
    /// sites CHECKS a pathname (`walk_py_files`'s `file_type()`,
    /// `nearest_package_root`'s `is_file()`, `find_compile_dbs`'s
    /// `is_file()`) and then OPENS it in a separate syscall, so the swap
    /// the findings describe lands in a window no test can enter without a
    /// seam in production code. A `std::fs::read_to_string` reappearing in
    /// the production half is therefore the only observable form of the
    /// regression, exactly as
    /// `every_git_child_is_built_through_the_policy_builder` treats a
    /// second git construction site. The behavioural half is
    /// `a_fifo_is_refused_at_every_open_seam_without_blocking`. Doc
    /// comments deliberately spell the call without its parenthesis so the
    /// count is the call sites alone.
    #[test]
    fn every_workspace_file_read_goes_through_the_bounded_seam() {
        let src = include_str!("ros2_migrate.rs");
        let marker = concat!("#[cfg(", "test)]\nmod tests {");
        let (production, _) = src
            .split_once(marker)
            .expect("the test module marker delimits the production half");
        let needle = concat!("fs::read_to_", "string(");
        assert_eq!(
            production.matches(needle).count(),
            0,
            "a workspace-file read outside read_regular_bounded_to_string \
             blocks forever on a planted FIFO"
        );
        // Anti-vacuity: the split really did keep the production half (the
        // seam it must route through lives there).
        assert!(
            production.contains("fn read_regular_bounded_to_string"),
            "the production half was not isolated — the marker moved"
        );
    }

    /// Every git child in this module is built through the ONE
    /// policy builder — a second construction site would spawn git outside
    /// the environment policy. The doc comments deliberately avoid the
    /// literal so this count is the builder alone.
    #[test]
    fn every_git_child_is_built_through_the_policy_builder() {
        let src = include_str!("ros2_migrate.rs");
        let needle = concat!("Command::new(\"", "git\")");
        assert_eq!(
            src.matches(needle).count(),
            1,
            "exactly one git Command construction site (git_command) is allowed"
        );
    }

    /// PATCH_DATA_LOSS: a replacement write that fails part-way
    /// after the prior patch was displaced must leave the PRIOR back at the
    /// path, byte-identical, with no partial file and no quarantine residue.
    #[test]
    fn a_torn_replacement_write_restores_the_displaced_prior_patch() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        let path = root.join(PATCH_FILENAME);
        let dir_entries = || -> Vec<String> {
            let mut v: Vec<String> = std::fs::read_dir(&root)
                .unwrap()
                .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
                .collect();
            v.sort();
            v
        };
        std::fs::write(&path, b"the user's earlier patch").unwrap();
        let prior = snapshot_prior_patch(&path).expect("snapshot the genuine prior");
        let mut torn = |f: &mut std::fs::File, b: &[u8]| -> std::io::Result<()> {
            use std::io::Write as _;
            f.write_all(&b[..3])?;
            Err(std::io::Error::other("disk full (injected)"))
        };
        let err =
            install_patch_no_clobber_with(&path, b"replacement patch", Some(&prior), &mut torn)
                .expect_err("a torn replacement must fail");
        let msg = err.to_string();
        assert!(msg.contains("disk full (injected)"), "{msg}");
        assert!(msg.contains("the partial patch was removed"), "{msg}");
        assert!(
            msg.contains("the pre-existing patch was restored in place"),
            "{msg}"
        );
        assert_eq!(
            std::fs::read(&path).unwrap(),
            b"the user's earlier patch".to_vec(),
            "the prior patch must be back at the path, byte-identical"
        );
        assert_eq!(
            dir_entries(),
            vec![PATCH_FILENAME.to_string()],
            "no partial file and no quarantine residue may remain"
        );
        // The happy path with a prior: replaced, and the held copy is gone.
        let prior = snapshot_prior_patch(&path).expect("snapshot again");
        let note = install_patch_no_clobber(&path, b"replacement patch", Some(&prior))
            .expect("the snapshotted prior is this run's to replace");
        assert_eq!(note, None);
        assert_eq!(std::fs::read(&path).unwrap(), b"replacement patch".to_vec());
        assert_eq!(dir_entries(), vec![PATCH_FILENAME.to_string()]);
    }

    /// PATCH_DATA_LOSS: when the path is RE-OCCUPIED by another
    /// actor while the torn replacement is being retired, the foreign
    /// occupant is preserved in place AND the prior's bytes are preserved at
    /// a quarantine name the error NAMES — nothing is lost, nothing foreign
    /// is destroyed.
    #[test]
    fn a_torn_replacement_with_a_foreign_reoccupant_preserves_both() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        let path = root.join(PATCH_FILENAME);
        std::fs::write(&path, b"the user's earlier patch").unwrap();
        let prior = snapshot_prior_patch(&path).expect("snapshot the genuine prior");
        let aside = root.join("our-partial-moved-aside");
        let path2 = path.clone();
        let mut torn = move |f: &mut std::fs::File, b: &[u8]| -> std::io::Result<()> {
            use std::io::Write as _;
            f.write_all(&b[..3])?;
            // Another actor moves our partial aside and plants its own
            // file at the path before the write "fails".
            std::fs::rename(&path2, &aside).unwrap();
            std::fs::write(&path2, b"another actor's file").unwrap();
            Err(std::io::Error::other("disk full (injected)"))
        };
        let err =
            install_patch_no_clobber_with(&path, b"replacement patch", Some(&prior), &mut torn)
                .expect_err("a torn replacement must fail");
        let msg = err.to_string();
        assert!(msg.contains("another actor's file"), "{msg}");
        assert_eq!(
            std::fs::read(&path).unwrap(),
            b"another actor's file".to_vec(),
            "the foreign occupant must survive byte-identical at the path"
        );
        let preserved = msg
            .split("the pre-existing patch is preserved at '")
            .nth(1)
            .and_then(|rest| rest.split('\'').next())
            .map(PathBuf::from)
            .expect("the error must NAME where the prior is preserved");
        assert_eq!(
            std::fs::read(&preserved).unwrap(),
            b"the user's earlier patch".to_vec(),
            "the prior's bytes must be preserved at the named quarantine"
        );
    }

    #[test]
    fn rollback_disposition_oracle() {
        use RollbackDisposition as D;
        let old: &[u8] = b"old";
        let new: &[u8] = b"new";
        // Untouched original — nothing to do, torn or not.
        assert_eq!(rollback_disposition(b"old", old, new, false), D::Skip);
        assert_eq!(rollback_disposition(b"old", old, new, true), D::Skip);
        // The migration's own bytes — restore.
        assert_eq!(rollback_disposition(b"new", old, new, false), D::Restore);
        // Foreign bytes when the write never modified the file — another
        // actor's work, preserved (the erasure class: torn
        // granted unconditionally erases a substituted source).
        assert_eq!(
            rollback_disposition(b"substituted by another actor", old, new, false),
            D::PreserveForeign
        );
        // Arbitrary bytes on the file whose own write TRUNCATED it before
        // failing — migration-owned partial output, restore (the heal).
        assert_eq!(rollback_disposition(b"ne", old, new, true), D::Restore);
        assert_eq!(rollback_disposition(b"", old, new, true), D::Restore);
    }

    /// Migrated from the deleted `write_phased` — the verified
    /// seam keeps the phased contract's pre-open half (`opened: None` =
    /// the file was never modified). Pre-open failures never reach the
    /// identity compare, so those arms pass a placeholder identity; the
    /// happy arm captures the real one.
    #[cfg(unix)]
    #[test]
    fn rewrite_verified_reports_pre_open_failures_as_unmodified() {
        use std::os::unix::fs::PermissionsExt as _;
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        let anchor = anchored::Anchor::new(&root).expect("anchor");
        let dummy = OccupantIdentity {
            dev: 0,
            ino: 0,
            mtime: 0,
            mtime_nsec: 0,
        };
        // A read-only file fails at open (EACCES) — BEFORE modification,
        // bytes intact.
        std::fs::write(root.join("ro.cpp"), b"keep").unwrap();
        std::fs::set_permissions(root.join("ro.cpp"), std::fs::Permissions::from_mode(0o444))
            .unwrap();
        let (opened, _e) =
            anchored::rewrite_verified(&anchor, Path::new("ro.cpp"), &dummy, b"keep", b"x")
                .unwrap_err();
        assert!(opened.is_none(), "EACCES is pre-modification");
        assert_eq!(
            std::fs::read(root.join("ro.cpp")).unwrap(),
            b"keep".to_vec()
        );
        // A symlink fails at open (ELOOP) — before modification, target
        // untouched.
        std::fs::write(root.join("target.cpp"), b"precious").unwrap();
        std::os::unix::fs::symlink(root.join("target.cpp"), root.join("link.cpp")).unwrap();
        let (opened, e) =
            anchored::rewrite_verified(&anchor, Path::new("link.cpp"), &dummy, b"precious", b"x")
                .unwrap_err();
        assert!(opened.is_none(), "ELOOP is pre-modification");
        assert!(anchored::is_eloop(&e));
        assert_eq!(
            std::fs::read(root.join("target.cpp")).unwrap(),
            b"precious".to_vec()
        );
        // The happy path writes — through the full identity + content
        // verification, with the REAL plan-time capture.
        std::fs::write(root.join("ok.cpp"), b"old").unwrap();
        let (bytes, identity) =
            read_regular_bounded_with_identity(&root.join("ok.cpp")).expect("capture");
        anchored::rewrite_verified(&anchor, Path::new("ok.cpp"), &identity, &bytes, b"written")
            .expect("writes");
        assert_eq!(
            std::fs::read(root.join("ok.cpp")).unwrap(),
            b"written".to_vec()
        );
    }

    /// The post-open-replacement race: the torn claim is
    /// IDENTITY-scoped. Real syscalls pin the primitive — the identity read
    /// off one fd is stable across re-reads of the same file, an atomic
    /// rename-over changes it — and the pure claim policy is pinned on all
    /// four arms: it holds only for the right position AND the very file
    /// the truncating open touched; a replaced file (the race) withdraws
    /// it, so the disposition preserves the replacement instead of erasing
    /// it with the snapshot.
    #[cfg(unix)]
    #[test]
    fn a_torn_claim_requires_position_and_the_opened_files_identity() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        std::fs::write(root.join("a.cpp"), b"aaa").unwrap();
        std::fs::write(root.join("b.cpp"), b"bbb").unwrap();
        let anchor = anchored::Anchor::new(&root).expect("anchor");
        // Identity read via the production restore handle (one fd).
        let read_id =
            |anchor: &anchored::Anchor, rel: &Path| -> std::io::Result<(Vec<u8>, FileIdentity)> {
                anchored::open_for_restore(anchor, rel)?.read_all()
            };
        let (_, id_a) = read_id(&anchor, Path::new("a.cpp")).unwrap();
        let (_, id_b) = read_id(&anchor, Path::new("b.cpp")).unwrap();
        // The primitive: stable across re-reads, distinct across files.
        let (_, id_a2) = read_id(&anchor, Path::new("a.cpp")).unwrap();
        assert_eq!(id_a, id_a2, "same file, same identity");
        assert_ne!(id_a, id_b, "different files, different identities");
        // An atomic replace (rename-over) hands the path a NEW identity —
        // exactly what the race plants between the truncating open and the
        // rollback's read.
        std::fs::rename(root.join("b.cpp"), root.join("a.cpp")).unwrap();
        let (_, id_replaced) = read_id(&anchor, Path::new("a.cpp")).unwrap();
        assert_eq!(
            id_replaced, id_b,
            "the replacement carries its own identity"
        );
        assert_ne!(
            id_replaced, id_a,
            "the replaced path no longer names the opened file"
        );
        // The pure claim policy, all four arms.
        assert!(torn_claim_holds(Some(&(2, id_a)), 2, &id_a));
        assert!(
            !torn_claim_holds(Some(&(2, id_a)), 1, &id_a),
            "another file's read never inherits the claim"
        );
        assert!(
            !torn_claim_holds(Some(&(2, id_a)), 2, &id_replaced),
            "a replaced file withdraws the claim — the race arm"
        );
        assert!(
            !torn_claim_holds(None, 2, &id_a),
            "no failed write, no claim"
        );
    }

    /// THE structural race pin, deterministic on real syscalls
    /// (no interposer needed): validation and restoration share one fd, so
    /// the exact adversarial interleaving — an atomic rename landing AFTER the
    /// rollback validated the file and BEFORE it restores — is driven here
    /// with a plain `rename` between `read_all` and `rewrite`. The
    /// replacement at the path must be untouched: the rewrite lands on the
    /// orphaned inode that was validated, because `RestoreFile` has no
    /// pathname to reopen (re-opening the path would
    /// truncate the replacement).
    #[cfg(unix)]
    #[test]
    fn a_replacement_renamed_over_the_path_after_validation_is_never_touched() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        std::fs::write(root.join("a.cpp"), b"migration-bytes").unwrap();
        std::fs::write(root.join("decoy.cpp"), b"another actor's replacement").unwrap();
        let anchor = anchored::Anchor::new(&root).expect("anchor");

        let mut of = anchored::open_for_restore(&anchor, Path::new("a.cpp")).expect("open");
        assert!(of.writable);
        let (cur, _id) = of.read_all().expect("read");
        assert_eq!(cur, b"migration-bytes".to_vec());

        // The race: an atomic replace lands after validation.
        std::fs::rename(root.join("decoy.cpp"), root.join("a.cpp")).unwrap();

        // The restore still succeeds — INTO THE VALIDATED (now orphaned)
        // inode — and the replacement at the path is byte-untouched.
        // The re-verify passes: the ORPHANED inode's content is exactly
        // what was validated (the rename moved the path, not the inode).
        assert!(of
            .rewrite_validated(&cur, b"original-snapshot")
            .expect("rewrite lands on the validated fd"));
        assert_eq!(
            std::fs::read(root.join("a.cpp")).unwrap(),
            b"another actor's replacement".to_vec(),
            "the replacement at the path must never be touched"
        );
    }

    /// ROLLBACK_RACE: the fd binding closes rename-over; it
    /// cannot stop another writer editing THE SAME INODE in place between
    /// validation and truncation. `rewrite_validated` re-reads through the
    /// same fd immediately before truncating and WITHDRAWS the restore
    /// when the content is no longer what the disposition validated — the
    /// actor's in-place edit survives and is reported. Driven
    /// deterministically at the seam where the race occurs (an in-place
    /// write via a separate handle between `read_all` and the rewrite),
    /// with the unchanged-content control proving the re-verify does not
    /// block legitimate restores.
    #[cfg(unix)]
    #[test]
    fn an_in_place_edit_after_validation_withdraws_the_restore() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        std::fs::write(root.join("a.cpp"), b"migration-bytes").unwrap();
        let anchor = anchored::Anchor::new(&root).expect("anchor");

        let mut of = anchored::open_for_restore(&anchor, Path::new("a.cpp")).expect("open");
        let (cur, _id) = of.read_all().expect("read");
        assert_eq!(cur, b"migration-bytes".to_vec());

        // The race: an in-place edit on the SAME inode lands after
        // validation (std::fs::write truncates through the path — same
        // file, new content).
        std::fs::write(root.join("a.cpp"), b"concurrent in-place edit").unwrap();

        // The restore is WITHDRAWN — the actor's edit survives.
        assert!(
            !of.rewrite_validated(&cur, b"original-snapshot")
                .expect("re-verify runs"),
            "a changed inode must withdraw the restore"
        );
        assert_eq!(
            std::fs::read(root.join("a.cpp")).unwrap(),
            b"concurrent in-place edit".to_vec(),
            "the in-place edit must never be truncated away"
        );

        // Control: an unchanged file restores normally.
        std::fs::write(root.join("b.cpp"), b"migration-bytes").unwrap();
        let mut ofb = anchored::open_for_restore(&anchor, Path::new("b.cpp")).expect("open");
        let (curb, _idb) = ofb.read_all().expect("read");
        assert!(ofb
            .rewrite_validated(&curb, b"original-snapshot")
            .expect("rewrites"));
        assert_eq!(
            std::fs::read(root.join("b.cpp")).unwrap(),
            b"original-snapshot".to_vec()
        );
    }

    /// (A writerless FIFO planted where the migration
    /// reads/writes would BLOCK a plain open forever — the run would never
    /// return). Every open seam is O_NONBLOCK + fd-type-gated:
    /// non-regular objects are refused BEFORE any read/write, and the
    /// refusal is immediate. Bounded harness deliberately: a regression
    /// here WEDGES rather than fails, so the seams run on a thread
    /// collected with a timeout.
    #[cfg(unix)]
    #[test]
    fn a_fifo_is_refused_at_every_open_seam_without_blocking() {
        use std::os::unix::ffi::OsStrExt as _;
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        let fifo = root.join("wedge.cpp");
        let mkfifo_at = |p: &Path| {
            let c = std::ffi::CString::new(p.as_os_str().as_bytes()).unwrap();
            assert_eq!(unsafe { libc::mkfifo(c.as_ptr(), 0o644) }, 0, "mkfifo");
        };
        mkfifo_at(&fifo);
        // The same wedge at the two pathname reads the verb
        // performs on workspace-supplied files.
        mkfifo_at(&root.join("wedge.py"));
        std::fs::create_dir_all(root.join(".cerulion")).unwrap();
        mkfifo_at(&root.join(MANIFEST_REL_PATH));

        let (tx, rx) = std::sync::mpsc::channel();
        let fifo2 = fifo.clone();
        let root2 = root.clone();
        std::thread::spawn(move || {
            let anchor = anchored::Anchor::new(&root2).expect("anchor");
            // (a) the rollback's restore open: refused as not-regular.
            let r1 = matches!(
                anchored::open_for_restore(&anchor, Path::new("wedge.cpp")),
                Err(e) if anchored::is_not_regular(&e)
            );
            // (b) the write seam: refused PRE-MODIFICATION (now
            // the seam is `rewrite_verified` — its O_RDWR|O_NONBLOCK open
            // of a FIFO succeeds regardless of readers, and the fd gate
            // then refuses it as not-regular; `opened == None`, nothing
            // written, and the identity compare is never reached).
            let wedge_dummy = super::OccupantIdentity {
                dev: 0,
                ino: 0,
                mtime: 0,
                mtime_nsec: 0,
            };
            let r2 = matches!(
                anchored::rewrite_verified(
                    &anchor,
                    Path::new("wedge.cpp"),
                    &wedge_dummy,
                    b"",
                    b"x"
                ),
                Err((None, _))
            );
            // (c) the bounded read helper (the quarantine verify, the
            // pre-write TOCTOU gate and build_plan all ride it).
            let r3 = matches!(
                read_regular_bounded(&fifo2),
                Err(e) if e.kind() == std::io::ErrorKind::Unsupported
            );
            // (d) the text seam every workspace-file read now
            // goes through — `package.xml`, the scanned `.py` files and
            // the compile database were each a bare `read_to_string`,
            // which opens the pathname and waits forever.
            let r4 = matches!(
                read_regular_bounded_to_string(&fifo2),
                Err(e) if e.kind() == std::io::ErrorKind::Unsupported
            );
            // (e) composed: the whole scan returns over a
            // workspace carrying a FIFO named like a `.py`. SCOPE —
            // the walker's own type check is the first gate here, so this
            // arm alone does not prove the READ is bound; the swap that
            // lands between the walker's `file_type()` and the read cannot
            // be constructed without a seam in production code. What pins
            // the read half is (d) plus the structural
            // `every_workspace_file_read_goes_through_the_bounded_seam`.
            let scan = super::scan_rclpy(&root2, &root2);
            // (f) the manifest's decision slot is read before
            // the writer's symlink refusal can speak. A FIFO there must
            // yield "no prior decision", not a wedged dry-run.
            let decision = super::existing_decision(&root2);
            tx.send((r1, r2, r3, r4, scan.len(), decision)).ok();
        });
        let (r1, r2, r3, r4, scanned, decision) = rx
            .recv_timeout(std::time::Duration::from_secs(20))
            .expect("a planted FIFO must be refused, never block an open or read");
        assert!(r1, "open_for_restore must refuse a FIFO as not-regular");
        assert!(r2, "write_phased must refuse a FIFO pre-modification");
        assert!(r3, "read_regular_bounded must refuse a FIFO");
        assert!(r4, "read_regular_bounded_to_string must refuse a FIFO");
        assert_eq!(
            scanned, 0,
            "a FIFO named like a .py file must be skipped, not scanned"
        );
        assert_eq!(
            decision,
            serde_json::Value::Null,
            "a FIFO at the manifest path must read as no prior decision"
        );
        // The FIFO itself is untouched.
        use std::os::unix::fs::FileTypeExt as _;
        assert!(fifo.symlink_metadata().unwrap().file_type().is_fifo());
    }

    /// The manifest read (the symlink
    /// half): the decision read walks the SAME anchored,
    /// per-component `O_NOFOLLOW` path the manifest WRITE walks, so a
    /// symlinked `.cerulion` is refused instead of followed. A
    /// `read_to_string(ws.join(MANIFEST_REL_PATH))` follows the link and
    /// lifts a `decision` out of a file OUTSIDE the workspace — into the
    /// manifest this run then writes — and does so BEFORE the writer's
    /// advertised symlink refusal could speak. Reverting to
    /// the pathname read makes the first assertion serve the foreign
    /// decision.
    #[cfg(unix)]
    #[test]
    fn a_symlinked_manifest_directory_is_refused_not_followed() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        let outside = tempfile::tempdir().expect("tempdir");
        let outside = outside.path().canonicalize().unwrap();

        // A decision the workspace must never be able to reach.
        std::fs::write(
            outside.join("ros2-migrate-manifest.json"),
            br#"{"decision":{"accepted":"FOREIGN"}}"#,
        )
        .unwrap();
        std::os::unix::fs::symlink(&outside, root.join(".cerulion")).unwrap();

        assert_eq!(
            existing_decision(&root),
            serde_json::Value::Null,
            "a symlinked .cerulion must be refused, never followed outside the workspace"
        );

        // Control (anti-vacuity): a REAL .cerulion directory with the same
        // bytes reads normally — the refusal is about the link, not about
        // the reader being inert.
        std::fs::remove_file(root.join(".cerulion")).unwrap();
        std::fs::create_dir(root.join(".cerulion")).unwrap();
        std::fs::write(
            root.join(MANIFEST_REL_PATH),
            br#"{"decision":{"accepted":"LOCAL"}}"#,
        )
        .unwrap();
        assert_eq!(
            existing_decision(&root),
            serde_json::json!({"accepted": "LOCAL"}),
            "a real in-workspace manifest must still carry its decision forward"
        );
    }

    /// Every anchored
    /// write walks from the anchor's descriptor, so if the root open
    /// follows a link the "pinned root" pins nothing — the whole run's
    /// writes land wherever the link points. Callers hand a `canonicalize`d
    /// root, whose final component is a symlink only if one arrived AFTER
    /// the canonicalization, so refusing costs no legitimate layout (the
    /// user's symlinked `src/` overlay is already resolved).
    /// Dropping `O_NOFOLLOW` makes the first assertion bind the link
    /// target instead of refusing. SCOPE: this arm pins the
    /// no-follow half only — the `lstat`/`fstat` identity compare guards a
    /// directory-for-DIRECTORY swap inside the open window, which cannot be
    /// constructed deterministically without a seam in the constructor, so
    /// it is not pinned here and a variant that drops it survives this test.
    #[cfg(unix)]
    #[test]
    fn the_anchor_root_is_bound_without_following_a_replacement_link() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        std::fs::create_dir(root.join("real")).unwrap();
        std::fs::create_dir(root.join("elsewhere")).unwrap();
        std::os::unix::fs::symlink(root.join("elsewhere"), root.join("swapped")).unwrap();

        // The swap: the canonical root's name now resolves to a directory
        // outside it. The anchor must refuse rather than bind the target.
        let Err(err) = anchored::Anchor::new(&root.join("swapped")) else {
            panic!("a symlinked anchor root must be refused");
        };
        assert!(
            anchored::is_eloop(&err) || err.kind() == std::io::ErrorKind::NotADirectory,
            "the refusal must come from the no-follow open, got {err:?} ({:?})",
            err.kind()
        );
        // Nothing was created through the link.
        assert!(
            std::fs::read_dir(root.join("elsewhere"))
                .unwrap()
                .next()
                .is_none(),
            "the link target must be untouched"
        );

        // Control (anti-vacuity): a real directory binds, and a write
        // through it lands inside that directory.
        let anchor = anchored::Anchor::new(&root.join("real")).expect("a real root must bind");
        anchored::create_excl(&anchor, Path::new("f.txt"), b"in-root").expect("write");
        assert_eq!(
            std::fs::read(root.join("real/f.txt")).unwrap(),
            b"in-root".to_vec()
        );
    }

    /// The quarantine give-back is structurally incapable of
    /// replacing a destination — POSIX `rename` REPLACES a dest created
    /// between any occupancy check and the rename (the race
    /// shape), so the return rides no-replace primitives (`hard_link` for
    /// files, `symlink` re-creation for links) and an occupied path keeps
    /// the object at the quarantine name. Both arms driven both ways
    /// against hand oracles; the occupied arms ARE the race post-state
    /// (a new writer's patch at the path when the give-back runs).
    #[cfg(unix)]
    #[test]
    fn a_quarantined_object_returns_only_by_no_replace() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        let dest = root.join("patch");
        let q = root.join("patch.rollback.q");

        // File, dest free -> returned in place, quarantine gone.
        std::fs::write(&q, b"foreign patch").unwrap();
        assert!(return_quarantined(&q, &dest, false));
        assert_eq!(std::fs::read(&dest).unwrap(), b"foreign patch".to_vec());
        assert!(!q.exists());

        // File, dest OCCUPIED (the race post-state: a new writer's patch
        // appeared) -> kept at the quarantine name; the new patch is
        // byte-untouched.
        std::fs::write(&dest, b"a NEW writer's patch").unwrap();
        std::fs::write(&q, b"foreign patch").unwrap();
        assert!(!return_quarantined(&q, &dest, false));
        assert_eq!(
            std::fs::read(&dest).unwrap(),
            b"a NEW writer's patch".to_vec(),
            "the new writer's patch must never be replaced"
        );
        assert_eq!(std::fs::read(&q).unwrap(), b"foreign patch".to_vec());
        std::fs::remove_file(&dest).unwrap();
        std::fs::remove_file(&q).unwrap();

        // Link, dest free -> re-created as a LINK with the same target.
        std::fs::write(root.join("target.txt"), b"t").unwrap();
        std::os::unix::fs::symlink("target.txt", &q).unwrap();
        assert!(return_quarantined(&q, &dest, true));
        let m = dest.symlink_metadata().unwrap();
        assert!(m.file_type().is_symlink());
        assert_eq!(
            std::fs::read_link(&dest).unwrap(),
            std::path::PathBuf::from("target.txt")
        );
        assert!(q.symlink_metadata().is_err(), "quarantined link removed");
        std::fs::remove_file(&dest).unwrap();

        // Link, dest OCCUPIED -> kept, occupant untouched.
        std::fs::write(&dest, b"occupant").unwrap();
        std::os::unix::fs::symlink("target.txt", &q).unwrap();
        assert!(!return_quarantined(&q, &dest, true));
        assert_eq!(std::fs::read(&dest).unwrap(), b"occupant".to_vec());
        assert!(q.symlink_metadata().unwrap().file_type().is_symlink());
    }

    /// The TOCTOU install: the patch install is CREATE-NEW
    /// FIRST and verifies any occupant against the preflight snapshot
    /// before removing it — a file another actor created at the patch
    /// path AFTER the preflight is REFUSED and survives byte-identical
    /// (a `remove_file` + `create_new` shape deletes it
    /// unverified). Hand oracles; a remove-first variant
    /// fails the foreign arms with the occupant's bytes replaced.
    #[test]
    fn patch_install_never_unlinks_an_unverified_occupant() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        let path = root.join(PATCH_FILENAME);
        let dir_entries = || -> Vec<String> {
            let mut v: Vec<String> = std::fs::read_dir(&root)
                .unwrap()
                .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
                .collect();
            v.sort();
            v
        };

        // Absent path: plain create-new, bytes exact.
        install_patch_no_clobber(&path, b"new patch", None).expect("create into absence");
        assert_eq!(std::fs::read(&path).unwrap(), b"new patch".to_vec());
        std::fs::remove_file(&path).unwrap();

        // FOREIGN occupant, preflight saw NOTHING (the reported TOCTOU
        // shape: the file appeared in the preflight→install gap):
        // refused loudly, survives byte-identical, no quarantine residue.
        std::fs::write(&path, b"another actor's file").unwrap();
        let err = install_patch_no_clobber(&path, b"new patch", None)
            .expect_err("a post-preflight occupant must refuse");
        assert!(
            err.to_string().contains("another actor's"),
            "refusal must name the foreign occupant: {err}"
        );
        assert_eq!(
            std::fs::read(&path).unwrap(),
            b"another actor's file".to_vec(),
            "the foreign occupant must survive byte-identical"
        );
        assert_eq!(
            dir_entries(),
            vec![PATCH_FILENAME.to_string()],
            "the give-back must leave no quarantine residue"
        );

        // FOREIGN occupant, the snapshotted prior was REPLACED mid-gap
        // with DIFFERENT bytes: refused, untouched. The snapshot is
        // taken from the genuine prior via the production helper
        // (bytes + identity), then the actor swaps in other
        // content.
        std::fs::remove_file(&path).unwrap();
        std::fs::write(&path, b"old snapshot").unwrap();
        let prior = snapshot_prior_patch(&path).expect("snapshot the genuine prior");
        std::fs::remove_file(&path).unwrap();
        std::fs::write(&path, b"another actor's file").unwrap();
        let err = install_patch_no_clobber(&path, b"new patch", Some(&prior))
            .expect_err("a bytes-mismatched occupant must refuse");
        assert!(err.to_string().contains("another actor's"), "{err}");
        assert_eq!(
            std::fs::read(&path).unwrap(),
            b"another actor's file".to_vec()
        );
        assert_eq!(dir_entries(), vec![PATCH_FILENAME.to_string()]);

        // OUR occupant (the SAME file the preflight snapshotted — bytes
        // AND identity, the normal re-run case): removed and replaced by
        // the new patch.
        std::fs::remove_file(&path).unwrap();
        std::fs::write(&path, b"old snapshot").unwrap();
        let prior = snapshot_prior_patch(&path).expect("snapshot the genuine prior");
        install_patch_no_clobber(&path, b"new patch", Some(&prior))
            .expect("the snapshotted prior patch is this run's to replace");
        assert_eq!(std::fs::read(&path).unwrap(), b"new patch".to_vec());
        assert_eq!(dir_entries(), vec![PATCH_FILENAME.to_string()]);
    }

    /// Quarantine collision: the capture never
    /// renames onto an existing path. A fixed pid-scoped
    /// quarantine name would mean a stale `.install.<pid>` left by a crashed
    /// prior run (pids recycle) is silently REPLACED by the capture
    /// rename — destroyed unverified, the exact class the quarantine
    /// exists to prevent. The name is nonce-unique and verified
    /// absent immediately before use, so the stale entry survives
    /// byte-identical through BOTH the ours-replace arm and the
    /// foreign-refusal arm, and neither leaves any new quarantine
    /// residue (the directory oracle proves the chosen capture name was
    /// fresh and was consumed). Reverting to a fixed
    /// `.install.<pid>` name fails both halves — the stale file's bytes
    /// are replaced by the captured occupant, then unlinked (ours arm)
    /// or returned to the patch path (foreign arm).
    #[test]
    fn patch_install_capture_never_targets_an_existing_quarantine_path() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        let path = root.join(PATCH_FILENAME);
        // A fixed pid-scoped quarantine name for THIS pid — exactly what a
        // crashed prior run under a recycled pid leaves behind.
        let stale = root.join(format!("{}.install.{}", PATCH_FILENAME, std::process::id()));
        let stale_name = stale.file_name().unwrap().to_string_lossy().into_owned();
        std::fs::write(&stale, b"crashed run's quarantined file").unwrap();
        let dir_entries = || -> Vec<String> {
            let mut v: Vec<String> = std::fs::read_dir(&root)
                .unwrap()
                .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
                .collect();
            v.sort();
            v
        };

        // Ours-replace arm: the capture quarantines the snapshotted
        // prior — at a FRESH name, never the stale one.
        std::fs::write(&path, b"old snapshot").unwrap();
        let prior = snapshot_prior_patch(&path).expect("snapshot");
        install_patch_no_clobber(&path, b"new patch", Some(&prior))
            .expect("the snapshotted prior patch is this run's to replace");
        assert_eq!(std::fs::read(&path).unwrap(), b"new patch".to_vec());
        assert_eq!(
            std::fs::read(&stale).unwrap(),
            b"crashed run's quarantined file".to_vec(),
            "the stale quarantine from a crashed run must survive the capture"
        );
        assert_eq!(
            dir_entries(),
            vec![PATCH_FILENAME.to_string(), stale_name.clone()],
            "no new quarantine residue — the fresh capture name was consumed"
        );

        // Foreign-refusal arm: an occupant the preflight never saw is
        // captured (fresh name again), classified foreign, given back.
        std::fs::remove_file(&path).unwrap();
        std::fs::write(&path, b"another actor's file").unwrap();
        let err = install_patch_no_clobber(&path, b"new patch", None)
            .expect_err("a post-preflight occupant must refuse");
        assert!(err.to_string().contains("another actor's"), "{err}");
        assert_eq!(
            std::fs::read(&path).unwrap(),
            b"another actor's file".to_vec()
        );
        assert_eq!(
            std::fs::read(&stale).unwrap(),
            b"crashed run's quarantined file".to_vec(),
            "the stale quarantine must also survive a foreign capture"
        );
        assert_eq!(
            dir_entries(),
            vec![PATCH_FILENAME.to_string(), stale_name],
            "the give-back must leave no new quarantine residue"
        );
    }

    /// Ownership TOCTOU: byte equality is not
    /// ownership. The preflight snapshot names the exact FILE it read —
    /// `(dev, ino)` + mtime off the reading fd — and the install refuses
    /// any occupant on a different identity even when its bytes equal
    /// the snapshot: an identical-bytes file another actor created at
    /// the patch path AFTER the preflight is a DIFFERENT file, and the
    /// a byte-only check would delete it. The foreign file is minted
    /// while the original still exists (then swapped in by rename), so
    /// it provably occupies a different inode — no inode-reuse flake.
    /// The happy identity-match control is the "OUR occupant" arm of
    /// `patch_install_never_unlinks_an_unverified_occupant` (the same
    /// snapshot helper, the same install — proving the refusal here is
    /// the identity, not the apparatus). Reverting the `ours`
    /// check to the bare byte compare fails this test — the foreign
    /// file is deleted and the install succeeds.
    #[cfg(unix)]
    #[test]
    fn patch_install_refuses_an_identical_bytes_foreign_occupant() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        let path = root.join(PATCH_FILENAME);

        // Preflight: snapshot the genuine prior patch (bytes + identity).
        std::fs::write(&path, b"old snapshot").unwrap();
        let prior = snapshot_prior_patch(&path).expect("snapshot");

        // Another actor replaces it with an IDENTICAL-BYTES file, staged
        // beside the original so both exist at once (a different inode
        // by construction) and swapped in atomically.
        let staging = root.join("foreign-staging");
        std::fs::write(&staging, b"old snapshot").unwrap();
        std::fs::rename(&staging, &path).unwrap();

        let err = install_patch_no_clobber(&path, b"new patch", Some(&prior))
            .expect_err("an identical-bytes file on a different inode must refuse");
        assert!(err.to_string().contains("another actor's"), "{err}");
        assert_eq!(
            std::fs::read(&path).unwrap(),
            b"old snapshot".to_vec(),
            "the foreign identical-bytes file must survive byte-identical"
        );
        let mut entries: Vec<String> = std::fs::read_dir(&root)
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .collect();
        entries.sort();
        assert_eq!(
            entries,
            vec![PATCH_FILENAME.to_string()],
            "the give-back must leave no quarantine residue"
        );
    }

    /// A symlink planted at the patch path after the preflight
    /// is never followed and never deleted — classified foreign WITHOUT
    /// reading through it, returned in place as a LINK with the same
    /// target. The oracle is sharp: the snapshot is taken from
    /// the link's TARGET itself, so a regression that dropped the
    /// `is_link` gate would FOLLOW the link and find matching bytes AND
    /// matching identity — and delete the link.
    #[cfg(unix)]
    #[test]
    fn patch_install_refuses_a_planted_symlink_and_preserves_it() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        let path = root.join(PATCH_FILENAME);
        std::fs::write(root.join("target.txt"), b"old snapshot").unwrap();
        let prior = snapshot_prior_patch(&root.join("target.txt")).expect("snapshot");
        std::os::unix::fs::symlink("target.txt", &path).unwrap();

        let err = install_patch_no_clobber(&path, b"new patch", Some(&prior))
            .expect_err("a planted link must refuse even with matching target bytes");
        assert!(err.to_string().contains("another actor's"), "{err}");
        let m = path.symlink_metadata().unwrap();
        assert!(
            m.file_type().is_symlink(),
            "the link must survive AS a link"
        );
        assert_eq!(
            std::fs::read_link(&path).unwrap(),
            std::path::PathBuf::from("target.txt")
        );
        assert_eq!(
            std::fs::read(root.join("target.txt")).unwrap(),
            b"old snapshot".to_vec(),
            "the link target must be untouched"
        );
    }

    /// The read-only fallback (EACCES on `O_RDWR` falls back to a
    /// readable descriptor with `writable == false`, so the disposition can
    /// still run and a Restore verdict is reported rather than performed)
    /// and the vanished-file arm's `create_excl` (recreation strictly INTO
    /// ABSENCE — anything at the path, a regular file or a planted link,
    /// fails `AlreadyExists`-shaped and survives byte-untouched).
    #[cfg(unix)]
    #[test]
    fn readonly_fallback_reports_not_writable_and_create_excl_never_replaces() {
        use std::os::unix::fs::PermissionsExt as _;
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        let anchor = anchored::Anchor::new(&root).expect("anchor");

        std::fs::write(root.join("ro.cpp"), b"frozen").unwrap();
        std::fs::set_permissions(root.join("ro.cpp"), std::fs::Permissions::from_mode(0o444))
            .unwrap();
        let mut of = anchored::open_for_restore(&anchor, Path::new("ro.cpp")).expect("fallback");
        assert!(!of.writable, "EACCES falls back to a read-only descriptor");
        let (cur, _id) = of.read_all().expect("still readable");
        assert_eq!(cur, b"frozen".to_vec());

        // create_excl: absent -> created; present -> refused untouched;
        // a planted (even dangling) link -> refused, link survives.
        anchored::create_excl(&anchor, Path::new("fresh.cpp"), b"recreated").expect("into absence");
        assert_eq!(
            std::fs::read(root.join("fresh.cpp")).unwrap(),
            b"recreated".to_vec()
        );
        let e = anchored::create_excl(&anchor, Path::new("fresh.cpp"), b"clobber").unwrap_err();
        assert_eq!(e.kind(), std::io::ErrorKind::AlreadyExists);
        assert_eq!(
            std::fs::read(root.join("fresh.cpp")).unwrap(),
            b"recreated".to_vec()
        );
        std::os::unix::fs::symlink(root.join("nowhere"), root.join("planted.cpp")).unwrap();
        anchored::create_excl(&anchor, Path::new("planted.cpp"), b"x").unwrap_err();
        assert!(root
            .join("planted.cpp")
            .symlink_metadata()
            .unwrap()
            .file_type()
            .is_symlink());
    }

    /// The shared write-seam identity verdict — the ONE fn
    /// BOTH anchored arms route their compare through. The non-unix arm
    /// never compiles on a Unix host, so this pin (plus the
    /// post-gate swap e2e, which rides the same fn through the Unix arm)
    /// is that arm's enforcement coverage: neutralizing the fn to `true`
    /// fails both. Hand oracle over every field axis — a verdict that
    /// ignores any one of them is killed by its arm.
    #[cfg(unix)]
    #[test]
    fn write_seam_identity_verdict_accepts_only_an_exact_match() {
        let base = OccupantIdentity {
            dev: 7,
            ino: 42,
            mtime: 1_000,
            mtime_nsec: 500,
        };
        assert!(write_seam_identity_holds(&base, &base));
        for wrong in [
            OccupantIdentity { dev: 8, ..base },
            OccupantIdentity { ino: 43, ..base },
            OccupantIdentity {
                mtime: 1_001,
                ..base
            },
            OccupantIdentity {
                mtime_nsec: 501,
                ..base
            },
        ] {
            assert!(
                !write_seam_identity_holds(&base, &wrong),
                "must refuse {wrong:?}"
            );
        }
    }

    /// Line + NESTED block comments removed; string literals deliberately
    /// NOT modelled — which is exactly why the structural needles in the
    /// walk below are concatenated at runtime (a literal needle in this
    /// tests mod would otherwise satisfy or trip its own scan). An
    /// unterminated block fails CLOSED by stripping to EOF.
    fn strip_comments(src: &str) -> String {
        let b: Vec<char> = src.chars().collect();
        let mut out = String::with_capacity(src.len());
        let mut i = 0;
        while i < b.len() {
            if b[i] == '/' && i + 1 < b.len() && b[i + 1] == '/' {
                while i < b.len() && b[i] != '\n' {
                    i += 1;
                }
            } else if b[i] == '/' && i + 1 < b.len() && b[i + 1] == '*' {
                let mut depth = 1usize;
                i += 2;
                while i < b.len() && depth > 0 {
                    if b[i] == '/' && i + 1 < b.len() && b[i + 1] == '*' {
                        depth += 1;
                        i += 2;
                    } else if b[i] == '*' && i + 1 < b.len() && b[i + 1] == '/' {
                        depth -= 1;
                        i += 2;
                    } else {
                        i += 1;
                    }
                }
            } else {
                out.push(b[i]);
                i += 1;
            }
        }
        out
    }

    /// A missing mtime must be
    /// UNREPRESENTABLE in an `OccupantIdentity`, and no capture site may
    /// collapse `Metadata::modified`'s failure into an absent value — on
    /// the non-unix tier an `Option` field captures `None` on a no-mtime
    /// filesystem and the equality gate reads a `None`-vs-`None` compare
    /// as a MATCH, so an equal-length replacement passes the identity
    /// check and is truncated. The compiler enforces presence
    /// (non-`Option` fields); this pin is the RUNNABLE half for the arm
    /// that never compiles on a Unix host — STRUCTURAL over
    /// the comment-stripped source, the repo's established cross-cfg
    /// pattern. Reverting the field to an
    /// `Option` + the captures to the swallow fails this test.
    #[test]
    fn a_missing_mtime_is_unrepresentable_and_never_swallowed() {
        let code = strip_comments(include_str!("ros2_migrate.rs"));
        // Both identity declarations (unix + non-unix) exist, and neither
        // body carries an optional field.
        let decl = format!("struct {}", "OccupantIdentity");
        let decls: Vec<usize> = code.match_indices(&decl).map(|(i, _)| i).collect();
        assert_eq!(
            decls.len(),
            2,
            "expected exactly the unix + non-unix OccupantIdentity declarations"
        );
        for at in decls {
            let open = code[at..].find('{').expect("declaration body") + at;
            let close = code[open..].find('}').expect("declaration end") + open;
            let body = &code[open..close];
            assert!(
                !body.contains("Option"),
                "an OccupantIdentity field must not be optional — missing \
                 identity must be unrepresentable, never comparable:{body}"
            );
        }
        // No capture site collapses the modified() failure into absence.
        for collapse in [
            format!("{}{}", ".modified()", ".ok()"),
            format!("{}{}", ".modified()", ".unwrap_or"),
        ] {
            assert!(
                !code.contains(&collapse),
                "a capture site swallows the mtime failure via `{collapse}`"
            );
        }
        // Anti-vacuity: the walk still sees the real capture calls (the
        // read seam and the non-unix write seam).
        let probe = format!("{}{}", ".modif", "ied(");
        assert!(
            code.matches(&probe).count() >= 2,
            "the walk no longer sees the modified() capture sites"
        );
    }

    /// The `#[cfg(not(unix))]`
    /// arm of this file is compiled by NO target this workspace can build
    /// for — the only non-unix target is Windows, where the dependency tree
    /// fails in `iceoryx2-pal-posix`'s build script long before this crate
    /// (measured: `cargo check -p cerulion_cli_engine --target
    /// x86_64-pc-windows-msvc`) — so a non-unix `OccupantIdentity`
    /// initializer that omits a field (an `occupant_identity_of` that
    /// leaves out `len`) compiles nowhere and fails nowhere. STRUCTURAL pin
    /// over the comment-stripped source, the repo's established cross-cfg
    /// pattern: every `OccupantIdentity { .. }` initializer must set EXACTLY
    /// one arm's full field set (struct-update `..base` forms are exempt —
    /// they inherit the rest). Dropping `len:`
    /// from the non-unix constructor fails this test.
    #[test]
    fn every_occupant_identity_initializer_sets_one_arms_full_field_set() {
        use std::collections::BTreeSet;
        fn matching_brace(code: &str, open: usize) -> usize {
            let bytes = code.as_bytes();
            let mut depth = 0usize;
            for (i, b) in bytes.iter().enumerate().skip(open) {
                match b {
                    b'{' => depth += 1,
                    b'}' => {
                        depth -= 1;
                        if depth == 0 {
                            return i;
                        }
                    }
                    _ => {}
                }
            }
            panic!("unbalanced braces after offset {open}");
        }
        fn field_names(body: &str) -> BTreeSet<String> {
            // Top-level comma split (nesting tracked), then the leading
            // identifier of each segment: `name: expr` or shorthand `name`.
            let mut out = BTreeSet::new();
            let mut depth = 0i32;
            let mut seg = String::new();
            let push = |seg: &str, out: &mut BTreeSet<String>| {
                let name = seg.split(':').next().unwrap_or("").trim();
                if !name.is_empty() {
                    out.insert(name.to_string());
                }
            };
            for ch in body.chars() {
                match ch {
                    '(' | '[' | '{' => depth += 1,
                    ')' | ']' | '}' => depth -= 1,
                    ',' if depth == 0 => {
                        push(&seg, &mut out);
                        seg.clear();
                        continue;
                    }
                    _ => {}
                }
                seg.push(ch);
            }
            push(&seg, &mut out);
            out
        }
        let code = strip_comments(include_str!("ros2_migrate.rs"));
        let decl = format!("struct {}", "OccupantIdentity");
        let mut arms: Vec<BTreeSet<String>> = Vec::new();
        for (at, _) in code.match_indices(&decl) {
            let open = code[at..].find('{').expect("declaration body") + at;
            let close = matching_brace(&code, open);
            arms.push(field_names(&code[open + 1..close]));
        }
        assert_eq!(arms.len(), 2, "expected the unix + non-unix declarations");
        assert_ne!(
            arms[0], arms[1],
            "the two arms must declare different field sets"
        );
        let init = format!("{} {{", "OccupantIdentity");
        let mut checked = 0usize;
        for (at, _) in code.match_indices(&init) {
            if code[..at].trim_end().ends_with("struct") {
                continue; // the declaration itself
            }
            let open = at + init.len() - 1;
            let close = matching_brace(&code, open);
            let body = &code[open + 1..close];
            if body.contains("..") {
                continue; // struct update: the omitted fields come from the base
            }
            let fields = field_names(body);
            assert!(
                arms.contains(&fields),
                "an OccupantIdentity initializer sets {fields:?}, which is neither \
                 the unix arm {:?} nor the non-unix arm {:?}:{body}",
                arms[0],
                arms[1]
            );
            checked += 1;
        }
        assert!(
            checked >= 4,
            "the walk saw only {checked} initializer(s) — apparatus broken"
        );
    }

    /// The write-seam content-verification
    /// read is BOUNDED — the unix identity carries no length, so an
    /// unbounded read let a same-inode appender with a restored mtime feed
    /// a multi-GB occupant into memory before the refusal. Refusal alone
    /// cannot discriminate this failure (an unbounded read also refuses in
    /// the end), so the pin is STRUCTURAL like the missing-mtime arm: within EACH
    /// `rewrite_verified` body, every content read is take-capped (counts
    /// equal, both nonzero). Needles runtime-concatenated so this test's
    /// own literals can neither satisfy nor trip the walk. The refusal
    /// SEMANTICS of a longer occupant are pinned e2e by the integration
    /// appended-tail arm.
    #[test]
    fn the_write_seam_verification_read_is_bounded() {
        let code = strip_comments(include_str!("ros2_migrate.rs"));
        let decl = format!("pub fn {}", "rewrite_verified");
        let starts: Vec<usize> = code.match_indices(&decl).map(|(i, _)| i).collect();
        assert_eq!(starts.len(), 2, "expected the unix + non-unix write seams");
        let fn_boundary = format!("pub fn{}", " ");
        let read_needle = format!("{}{}", "read_", "to_end");
        let take_needle = format!("{}{}", ".ta", "ke(");
        for at in starts {
            let rest = &code[at + decl.len()..];
            let end = rest.find(&fn_boundary).unwrap_or(rest.len());
            let body = &rest[..end];
            let reads = body.matches(&read_needle).count();
            let takes = body.matches(&take_needle).count();
            assert!(reads >= 1, "the seam must still read the occupant back");
            assert_eq!(
                reads, takes,
                "every verification read must be take-capped; reads={reads} takes={takes}"
            );
        }
    }

    /// Rollback reporting: the
    /// capture-failure note must carry the prior-patch state — an operator
    /// whose pre-existing patch was displaced by this run's install has to
    /// be told it was NOT restored (the path still holds this run's
    /// patch), never only that the capture failed.
    #[test]
    fn a_capture_failure_note_reports_the_undisplaced_prior_patch() {
        let err = std::io::Error::other("disk full");
        let path = Path::new("/ws/cerulion-ros2-migration.patch");
        let with_prior = capture_failure_note(path, &err, true);
        assert!(
            with_prior.contains("could not be captured for verification"),
            "got: {with_prior}"
        );
        assert!(with_prior.contains("disk full"), "the cause: {with_prior}");
        assert!(
            with_prior.contains("PRE-EXISTING patch") && with_prior.contains("NOT restored"),
            "must name the undisplaced prior patch: {with_prior}"
        );
        // The note is a line an operator READS, so
        // it is asserted as RENDERED, not merely by its keywords. The
        // literal wrapped without a trailing `\` bakes its
        // fourteen columns of source indentation into the string — the
        // operator sees a long gap before "(the path". Every keyword
        // assertion above passes straight over it, which is why the whole
        // rendering is pinned here instead of one more substring.
        assert!(
            !with_prior.contains("  "),
            "the note must not carry a run of literal spaces: {with_prior}"
        );
        assert!(
            with_prior.ends_with(
                "; the PRE-EXISTING patch this run displaced was NOT restored \
                 (the path still holds this run's patch)"
                    .replace('\n', "")
                    .split_whitespace()
                    .collect::<Vec<_>>()
                    .join(" ")
                    .as_str()
            ),
            "the note must render single-spaced: {with_prior}"
        );
        let no_prior = capture_failure_note(path, &err, false);
        assert!(no_prior.contains("could not be captured for verification"));
        assert!(
            !no_prior.contains("NOT restored"),
            "no prior patch existed — the note must not claim one: {no_prior}"
        );
    }

    /// The container-harness apply seam: `apply_plan_in_place`
    /// writes through the SAME anchored `O_NOFOLLOW` walker as `--write` —
    /// a planted link is refused with the target untouched, never followed.
    /// (The path-ADMISSION half — a crafted analysis naming a file outside
    /// the src root — is `build_plan`'s containment gate, pinned by the
    /// integration test's OUTSIDE-the-workspace arm; the example reaches
    /// this seam only through `build_plan`.)
    #[cfg(unix)]
    #[test]
    fn apply_plan_in_place_writes_through_the_anchor_and_refuses_links() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path().canonicalize().unwrap();
        std::fs::create_dir(root.join("pkg")).unwrap();
        std::fs::write(root.join("pkg/a.cpp"), b"old-a").unwrap();
        // This seam now rides the VERIFIED write, so the happy
        // arm carries the REAL plan-time capture; `mk` (placeholder
        // identity) serves only the REFUSAL arms, which fail at the open —
        // before the identity compare is reached.
        let mk = |name: &str, bytes: &[u8]| PlannedFile {
            path: root.join(name),
            rel: name.to_string(),
            src_rel: PathBuf::from(name),
            old_bytes: b"old".to_vec(),
            new_bytes: bytes.to_vec(),
            old_identity: OccupantIdentity {
                dev: 0,
                ino: 0,
                mtime: 0,
                mtime_nsec: 0,
            },
        };
        let planned = |name: &str, new_bytes: &[u8]| {
            let (old_bytes, old_identity) =
                read_regular_bounded_with_identity(&root.join(name)).expect("capture");
            PlannedFile {
                path: root.join(name),
                rel: name.to_string(),
                src_rel: PathBuf::from(name),
                old_bytes,
                new_bytes: new_bytes.to_vec(),
                old_identity,
            }
        };
        let mut plan = MigrationPlan::default();
        plan.files.push(planned("pkg/a.cpp", b"new-a"));
        apply_plan_in_place(&root, &plan).expect("write through the anchor");
        assert_eq!(std::fs::read(root.join("pkg/a.cpp")).unwrap(), b"new-a");

        // The matrix seam refuses the same
        // post-planning swap the production --write path refuses — an
        // identical-bytes replacement renamed over the source AFTER the
        // plan captured its identity must not be truncated (a truncate-at-open
        // write here would overwrite it).
        std::fs::write(root.join("pkg/d.cpp"), b"old-d").unwrap();
        let mut plan_swap = MigrationPlan::default();
        plan_swap.files.push(planned("pkg/d.cpp", b"new-d"));
        std::fs::write(root.join("pkg/d.swap"), b"old-d").unwrap();
        std::fs::rename(root.join("pkg/d.swap"), root.join("pkg/d.cpp")).unwrap();
        let err = apply_plan_in_place(&root, &plan_swap).expect_err("swap refused");
        assert!(
            err.to_string().contains("d.cpp") && err.to_string().contains("identity mismatch"),
            "must refuse the post-planning swap by name: {err}"
        );
        assert_eq!(
            std::fs::read(root.join("pkg/d.cpp")).unwrap(),
            b"old-d",
            "the replacement must survive unmodified"
        );

        // A file swapped for a symlink is refused (O_NOFOLLOW), the error
        // names the file, and the link + its target are untouched.
        std::fs::write(root.join("target.txt"), b"external").unwrap();
        std::os::unix::fs::symlink(root.join("target.txt"), root.join("pkg/b.cpp")).unwrap();
        let mut plan2 = MigrationPlan::default();
        plan2.files.push(mk("pkg/b.cpp", b"attack"));
        let err = apply_plan_in_place(&root, &plan2).expect_err("link refused");
        assert!(err.to_string().contains("b.cpp"), "names the file: {err}");
        assert_eq!(std::fs::read(root.join("target.txt")).unwrap(), b"external");
        assert!(root
            .join("pkg/b.cpp")
            .symlink_metadata()
            .unwrap()
            .file_type()
            .is_symlink());

        // An intermediate DIRECTORY that is a symlink is refused
        // at that component (the per-component O_NOFOLLOW walk), the decoy
        // tree untouched. This arm moved here from the integration test
        // `a_parent_directory_swapped_for_a_symlink_never_redirects_the_write`
        // when the identity gate started refusing that shape BEFORE the
        // write — the walker still owns a swap landing in the gate→write
        // gap, and this seam has no gate in front of it.
        let decoy = tempfile::tempdir().expect("decoy dir");
        let decoy_root = decoy.path().canonicalize().unwrap();
        std::fs::write(decoy_root.join("c.cpp"), b"decoy-c").unwrap();
        std::os::unix::fs::symlink(&decoy_root, root.join("pkg_link")).unwrap();
        let mut plan3 = MigrationPlan::default();
        plan3.files.push(mk("pkg_link/c.cpp", b"attack"));
        let err = apply_plan_in_place(&root, &plan3).expect_err("swapped parent refused");
        assert!(err.to_string().contains("c.cpp"), "names the file: {err}");
        assert_eq!(
            std::fs::read(decoy_root.join("c.cpp")).unwrap(),
            b"decoy-c",
            "the walk must refuse AT the link component, never write through it"
        );
    }

    #[test]
    fn parse_tool_output_gates_the_format_version() {
        let ok = r#"{"format":1,"file":"/x.cpp","rewrites":[],"candidates":[]}"#;
        assert!(parse_tool_output(ok).is_ok());
        let wrong = r#"{"format":2,"file":"/x.cpp","rewrites":[],"candidates":[]}"#;
        let err = parse_tool_output(wrong).unwrap_err().to_string();
        assert!(err.contains("format 2"), "got: {err}");
    }

    /// Two publish sites that share a `(file,
    /// line, reason)` but differ by COLUMN are DISTINCT sites and must both
    /// reach the operator's report.
    ///
    /// The shape this exists for: a macro whose body spells the publish gives
    /// every invocation the same location, so unless the engine anchors
    /// candidates at their EXPANSION location and carries a column, N sites
    /// collapse into one and N-1 vanish from the report and the manifest —
    /// silently, and pointing at a `#define` the operator may not own.
    ///
    /// Dropping `x.column == c.column` from `push_candidate`
    /// merges the two and fails the first assertion at 1.
    #[test]
    fn candidates_on_one_line_are_distinct_sites_when_their_columns_differ() {
        let cand = |line: u64, column: u64, reason: &str| ToolCandidate {
            file: "/ws/src/pkg/a.cpp".to_string(),
            function: "f".to_string(),
            line,
            column,
            reason: reason.to_string(),
            detail: "d".to_string(),
        };
        let out = |cs: Vec<ToolCandidate>| ToolOutput {
            format: TOOL_FORMAT_VERSION,
            tool_version: String::new(),
            file: "/ws/src/pkg/a.cpp".to_string(),
            rewrites: Vec::new(),
            candidates: cs,
        };
        // Two macro invocations on ONE line: same file/line/reason, different
        // columns. Both survive.
        let merged = merge_tool_outputs(&[out(vec![
            cand(7, 3, "pointer-escapes"),
            cand(7, 41, "pointer-escapes"),
        ])]);
        assert_eq!(
            merged.candidates.len(),
            2,
            "two columns are two sites: {:?}",
            merged.candidates
        );
        // ...and in a deterministic order (the report is byte-compared).
        assert_eq!(
            merged
                .candidates
                .iter()
                .map(|c| c.column)
                .collect::<Vec<_>>(),
            vec![3, 41]
        );

        // ANTI-TAUTOLOGY: the dedup still WORKS. The genuine duplicate — the
        // same site reported by two translation units, which is why
        // `push_candidate` exists — collapses to one.
        let merged = merge_tool_outputs(&[
            out(vec![cand(7, 3, "pointer-escapes")]),
            out(vec![cand(7, 3, "pointer-escapes")]),
        ]);
        assert_eq!(
            merged.candidates.len(),
            1,
            "the same site from two TUs is still one candidate: {:?}",
            merged.candidates
        );
    }

    /// `column` is ADDITIVE on the engine's wire. A
    /// `cerulion-ros2-migrate-clang` that predates the column — one already on the
    /// operator's PATH, or pinned by `CERULION_ROS2_MIGRATE_TOOL` — emits no
    /// column, and that must degrade to the pre-column dedup rather than refuse
    /// to parse. (`deny_unknown_fields` guards the other direction; this pins
    /// the missing-field direction, which `#[serde(default)]` owns.)
    #[test]
    fn a_pre_round_43_engine_output_still_parses_with_no_column() {
        let old = r#"{"format":1,"file":"/x.cpp","rewrites":[],"candidates":[
            {"file":"/x.cpp","function":"f","line":9,
             "reason":"pointer-escapes","detail":"d"}]}"#;
        let parsed = parse_tool_output(old).expect("a pre-43 engine must parse");
        assert_eq!(parsed.candidates.len(), 1);
        assert_eq!(
            parsed.candidates[0].column, 0,
            "an absent column reads 0, which is the pre-43 dedup exactly"
        );
    }

    #[test]
    fn parse_tool_output_rejects_an_unknown_key_never_applies_it_blind() {
        // `deny_unknown_fields` on the engine's OWN output: an unknown key
        // means a stale/garbled/version-skewed edit, and applying it blind
        // is exactly the disaster the gate exists to prevent. An
        // unknown key inside an EDIT is the sharpest case — that is the byte
        // range that gets written.
        let bad_edit = r#"{"format":1,"file":"/x.cpp","rewrites":[{
            "file":"/x.cpp","function":"f","kind":"stack","message_type":"T",
            "publisher":"pub_","line":1,"edits":[{"offset":0,"length":1,
            "original":"a","replacement":"b","surprise":"payload"}]}],
            "candidates":[]}"#;
        assert!(
            parse_tool_output(bad_edit).is_err(),
            "an unknown edit key must be refused, never applied"
        );
        let bad_top = r#"{"format":1,"file":"/x.cpp","rewrites":[],
            "candidates":[],"extra":true}"#;
        assert!(parse_tool_output(bad_top).is_err());
        // …but a well-formed document with only the documented keys parses
        // (the anti-tautology half — deny must not reject valid output).
        let good = r#"{"format":1,"tool_version":"0.1.0","file":"/x.cpp",
            "rewrites":[{"file":"/x.cpp","function":"f","kind":"stack",
            "message_type":"T","publisher":"pub_","line":1,
            "edits":[{"offset":0,"length":1,"original":"a","replacement":"b"}]}],
            "candidates":[{"file":"/x.cpp","function":"g","line":2,
            "reason":"pointer-escapes","detail":"d"}]}"#;
        assert!(parse_tool_output(good).is_ok());
    }

    #[test]
    fn tus_for_db_filters_to_cpp_under_src_root() {
        // Real compile databases carry RELATIVE entries with `..` (the
        // CMake contract resolves them against `directory`); an
        // un-normalized join never lexically matches src_root and the TU
        // is silently dropped — while a `..` ESCAPE must stay excluded.
        let db = r#"[
          {"directory":"/ws/build/p","command":"c","file":"/ws/src/p/src/a.cpp"},
          {"directory":"/ws/build/p","command":"c","file":"/ws/src/p/src/a.cpp"},
          {"directory":"/ws/src/p","command":"c","file":"src/b.cc"},
          {"directory":"/ws/build/p/deep","command":"c","file":"../../../src/p/src/rel.cpp"},
          {"directory":"/ws/build/p","command":"c","file":"/ws/src/../etc/evil.cpp"},
          {"directory":"/ws/build/p","command":"c","file":"/opt/ros/x.cpp"},
          {"directory":"/ws/build/p","command":"c","file":"/ws/src/p/inc/h.hpp"}
        ]"#;
        let tus = tus_for_db(db, Path::new("/ws/src")).expect("parses");
        assert_eq!(
            tus,
            vec![
                PathBuf::from("/ws/src/p/src/a.cpp"),
                PathBuf::from("/ws/src/p/src/b.cc"),
                PathBuf::from("/ws/src/p/src/rel.cpp")
            ]
        );
        // The pure normalizer itself, both directions.
        assert_eq!(
            lexically_normalize(Path::new("/ws/build/p/../../src/a.cpp")),
            PathBuf::from("/ws/src/a.cpp")
        );
        assert_eq!(
            lexically_normalize(Path::new("/ws/./src/../etc/x.cpp")),
            PathBuf::from("/ws/etc/x.cpp")
        );
    }
}
