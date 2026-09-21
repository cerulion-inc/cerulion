// SPDX-License-Identifier: AGPL-3.0-only
//! Auto-partition EMIT + surgical `process_groups:` rewrite.
//!
//! Two engine-core pieces sit between the pure partitioner
//! ([`cerulion_core::graph::auto_partition`] /
//! [`cerulion_core::graph::baseline_process_per_node`]) and the `graph
//! partition` verb:
//!
//! * [`derive_emit_groups`] — the emit glue. Given a parsed graph and an
//!   OPTIONAL cost artifact it returns the derived `process_groups` mapping:
//!   with costs → the cost-aware fused optimum; without → the process-per-node
//!   baseline (maximal fault isolation, no fabricated costs — Principle #13) —
//!   except that `block` co-location constraints are seeded on BOTH
//!   arms, so a graph carrying a `block` input has multi-member groups in the
//!   "baseline" too (a `block` defer mirror is process-local; splitting the
//!   flow refuses to build).
//! * [`rewrite_process_groups_block`] — a SURGICAL text splice. The graph file
//!   is a user-authored, comment-carrying document, so the emit path must NOT
//!   round-trip it through serde (which drops comments, blank lines, key order,
//!   and quoting). This function locates (or inserts) exactly the top-level
//!   `process_groups:` block by an indentation scan and replaces only those
//!   bytes, preserving every other byte verbatim. A file whose structure
//!   defeats the scan is refused with a loud, actionable error — never a silent
//!   fall back to the lossy serde round-trip.

use std::path::{Path, PathBuf};

use indexmap::IndexMap;

use cerulion_core::graph::{
    auto_partition, baseline_process_per_node, block_colocation_seeds, build_trigger_edges,
    default_prefix, parse_graph_raw, validate_graph, GraphConfig, GraphTopology, LevelGrowth,
    Levels, NodeInfo, TriggerEdges,
};

use crate::error::{CliError, CliResult};
// The block scanner + scalar renderer live in
// `crate::yaml_splice`, which `node stage` also calls. Imported
// rather than re-declared — the bug class to avoid is a block bound
// re-derived in a second place.
use crate::graph_cmd::{
    default_artifact_path, parse_profile_artifact, render_levels_report, source_entry_infos,
    write_yaml_atomically, ExpectedPrior, ProfileArtifact, TimeSource,
};
use crate::yaml_splice::{
    line_numbers, render_scalar, scan_top_level_block, split_lines, BlockScan,
};

/// Derive the `process_groups` mapping the emitter will write.
///
/// * `Some(artifact)` — thaw the cost snapshot (via
///   [`ProfileArtifact::to_costs`], which carries its own loud hand-edit
///   validation) and run the cost-aware [`auto_partition`]: process-per-node
///   baseline + validated greedy fusion under `budget_ns`.
/// * `None` — no measured costs, so fall back to
///   [`baseline_process_per_node`]: every node in its own process (maximal
///   fault isolation). Deriving the baseline directly (rather than feeding
///   [`auto_partition`] a synthesized zero-cost snapshot) keeps the fallback
///   measurement-free — no fabricated per-node durations (Principle #13).
///
/// Both arms name groups + order them identically (`grp_<lead>`, pipeline
/// order) because they share the partitioner's internal assembly. Pure: the
/// only effect is a `tracing::warn!` from [`auto_partition`] on a stray
/// cost-snapshot key.
///
/// The returned map drops straight into [`GraphConfig::process_groups`] and
/// feeds [`rewrite_process_groups_block`].
/// Write a graph YAML while HOLDING the workspace mutation lock.
///
/// `graph partition` and `graph run`'s auto-partition persist rewrite
/// `graphs/<g>.yaml`; `cerulion-wsd` and the CLI's `node stage`/`graph create`
/// serialize their own writes on `<root>/.cerulion/workspace.lock`, and a
/// writer outside that lock would let the daemon's compare-and-swap commit
/// against bytes its client never saw. The lock covers the re-check AND the
/// rename inside [`write_yaml_atomically`]; the derive/preview/prompt span
/// stays lock-free, which is what [`ExpectedPrior::Contents`] exists for.
fn write_graph_locked(
    workspace_root: &Path,
    dest: &Path,
    contents: &str,
    file_kind: &str,
    expected: ExpectedPrior<'_>,
) -> CliResult<Option<PathBuf>> {
    let _lock = crate::workspace_lock::WorkspaceLock::acquire_and_track_gitignore(workspace_root)?;
    write_yaml_atomically(dest, contents, file_kind, expected)
}

pub fn derive_emit_groups(
    config: &GraphConfig,
    entry_infos: &IndexMap<String, NodeInfo>,
    trigger_edges: &TriggerEdges,
    artifact: Option<&ProfileArtifact>,
    budget_ns: u64,
) -> CliResult<IndexMap<String, Vec<String>>> {
    match artifact {
        Some(artifact) => {
            let (costs, isolated) = artifact.to_costs()?;
            let partition = auto_partition(
                config,
                entry_infos,
                trigger_edges,
                &costs,
                &isolated,
                budget_ns,
            )?;
            Ok(partition.groups)
        }
        None => Ok(baseline_process_per_node(
            config,
            entry_infos,
            trigger_edges,
        )?),
    }
}

/// Replace (or insert) the top-level `process_groups:` block in
/// a graph YAML document WITHOUT disturbing any other byte.
///
/// `raw_yaml` is the graph file's exact bytes; `groups` is the derived mapping
/// ([`derive_emit_groups`]); `source_label` names the file in diagnostics (a
/// path in production, any label in tests). Returns the rewritten document.
///
/// # Behavior
///
/// * **Replace** — when exactly one top-level (`column 0`) `process_groups:` key
///   exists, its block is replaced with the freshly rendered one. Block bounds
///   come from `scan_top_level_block` — the SINGLE authority on block
///   extent: the block ends at the next column-0 KEY line, and column-0 `#`
///   comments / blank lines are NEUTRAL — they never terminate the block. A
///   neutral run joins the block (and is consumed with it on replace) only
///   when a more-indented line follows before the next column-0 key (an
///   interior divider); trailing comments/blanks after the block's last
///   indented line stay OUTSIDE the replaced span, preserved byte-identically
///   (they belong to the next section). Do not re-derive bounds here — an
///   "up to the next column-0 non-blank line" reading silently
///   DROPS every group after an interior divider.
/// * **Insert** — when no `process_groups:` block exists, the rendered block is
///   inserted immediately before the top-level `nodes:` key, followed by a blank
///   line separator.
/// * **Preserve** — every byte outside the replaced/inserted span is copied
///   verbatim: comments, blank lines, key order, quoting styles, and CRLF/LF
///   line endings are all untouched.
///
/// # Errors (refuse-and-diagnose — never a silent serde fallback)
///
/// * more than one top-level `process_groups:` key (the block bounds are
///   ambiguous);
/// * an insert is needed but the file has no — or more than one — top-level
///   `nodes:` key (no unambiguous insertion point);
/// * the derived `groups` mapping is empty (an internal error: a partition must
///   have ≥1 group).
///
/// Each error names `source_label`, what was searched for, and the fix.
pub fn rewrite_process_groups_block(
    raw_yaml: &str,
    groups: &IndexMap<String, Vec<String>>,
    source_label: &str,
) -> CliResult<String> {
    if groups.is_empty() {
        return Err(CliError::Validation(format!(
            "cannot rewrite the process_groups block in '{source_label}': the derived grouping is \
             empty (a partition must contain at least one group) — this is an internal error, not \
             a problem with the file"
        )));
    }

    let lines = split_lines(raw_yaml);
    let rendered = render_process_groups_block(groups);

    match scan_top_level_block(&lines, "process_groups") {
        BlockScan::Duplicate { key_lines } => Err(CliError::Validation(format!(
            "cannot rewrite the process_groups block in '{source_label}': found {} top-level \
             `process_groups:` keys at column 0 (lines {}), so the block bounds are ambiguous — a \
             valid graph declares AT MOST ONE. Remove the duplicate(s) and retry.",
            key_lines.len(),
            line_numbers(&key_lines)
        ))),
        BlockScan::One { start, end } => {
            // REPLACE exactly the block span (trailing blank separators after
            // the last indented line stay outside the splice).
            let mut out = String::with_capacity(raw_yaml.len() + rendered.len());
            out.push_str(&raw_yaml[..start]);
            out.push_str(&rendered);
            out.push_str(&raw_yaml[end..]);
            Ok(out)
        }
        BlockScan::Absent => {
            // INSERT before the sole top-level `nodes:` key.
            let nodes_idx = match scan_top_level_block(&lines, "nodes") {
                BlockScan::One { start, .. } => start,
                BlockScan::Absent => {
                    return Err(CliError::Validation(format!(
                        "cannot insert a process_groups block into '{source_label}': no top-level \
                         `nodes:` key was found at column 0 to insert before. A graph declares \
                         its nodes under a top-level `nodes:` mapping — add one (or confirm this \
                         is the intended graph file) and retry."
                    )));
                }
                BlockScan::Duplicate { key_lines } => {
                    return Err(CliError::Validation(format!(
                        "cannot insert a process_groups block into '{source_label}': found {} \
                         top-level `nodes:` keys at column 0 (lines {}), so the insertion point \
                         is ambiguous — a valid graph declares EXACTLY ONE. Fix the duplicate and \
                         retry.",
                        key_lines.len(),
                        line_numbers(&key_lines)
                    )));
                }
            };

            let mut out = String::with_capacity(raw_yaml.len() + rendered.len() + 1);
            out.push_str(&raw_yaml[..nodes_idx]);
            out.push_str(&rendered);
            out.push('\n'); // blank line between the inserted block and `nodes:`
            out.push_str(&raw_yaml[nodes_idx..]);
            Ok(out)
        }
    }
}

/// Replace (or insert) the top-level `level_assignments:`
/// block in a graph YAML document WITHOUT disturbing any other byte — the
/// [`rewrite_process_groups_block`] discipline applied to the baked
/// level-refinement output.
///
/// `assignments` MUST cover every node (the full-coverage contract —
/// the emitter builds it from the refined [`cerulion_core::graph::Levels`]
/// over `config.nodes`, so coverage holds by construction), rendered in the
/// map's (graph) order. Same replace/insert/preserve semantics and the same
/// refuse-and-diagnose arms as the process-groups rewrite: duplicate
/// top-level keys are ambiguous; an insert needs exactly one top-level
/// `nodes:` key (the block lands immediately before it — so an emit that
/// splices `process_groups:` first and this block second puts
/// `level_assignments:` between the two, reading top-down as groups →
/// levels → nodes).
pub fn rewrite_level_assignments_block(
    raw_yaml: &str,
    assignments: &IndexMap<String, usize>,
    source_label: &str,
) -> CliResult<String> {
    if assignments.is_empty() {
        return Err(CliError::Validation(format!(
            "cannot rewrite the level_assignments block in '{source_label}': the derived \
             assignment is empty (a refined levelization must cover every node) — this is an \
             internal error, not a problem with the file"
        )));
    }

    let lines = split_lines(raw_yaml);
    let rendered = render_level_assignments_block(assignments);

    match scan_top_level_block(&lines, "level_assignments") {
        BlockScan::Duplicate { key_lines } => Err(CliError::Validation(format!(
            "cannot rewrite the level_assignments block in '{source_label}': found {} top-level \
             `level_assignments:` keys at column 0 (lines {}), so the block bounds are ambiguous \
             — a valid graph declares AT MOST ONE. Remove the duplicate(s) and retry.",
            key_lines.len(),
            line_numbers(&key_lines)
        ))),
        BlockScan::One { start, end } => {
            let mut out = String::with_capacity(raw_yaml.len() + rendered.len());
            out.push_str(&raw_yaml[..start]);
            out.push_str(&rendered);
            out.push_str(&raw_yaml[end..]);
            Ok(out)
        }
        BlockScan::Absent => {
            let nodes_idx = match scan_top_level_block(&lines, "nodes") {
                BlockScan::One { start, .. } => start,
                BlockScan::Absent => {
                    return Err(CliError::Validation(format!(
                        "cannot insert a level_assignments block into '{source_label}': no \
                         top-level `nodes:` key was found at column 0 to insert before. A graph \
                         declares its nodes under a top-level `nodes:` mapping — add one (or \
                         confirm this is the intended graph file) and retry."
                    )));
                }
                BlockScan::Duplicate { key_lines } => {
                    return Err(CliError::Validation(format!(
                        "cannot insert a level_assignments block into '{source_label}': found {} \
                         top-level `nodes:` keys at column 0 (lines {}), so the insertion point \
                         is ambiguous — a valid graph declares EXACTLY ONE. Fix the duplicate \
                         and retry.",
                        key_lines.len(),
                        line_numbers(&key_lines)
                    )));
                }
            };

            let mut out = String::with_capacity(raw_yaml.len() + rendered.len() + 1);
            out.push_str(&raw_yaml[..nodes_idx]);
            out.push_str(&rendered);
            out.push('\n'); // blank line between the inserted block and `nodes:`
            out.push_str(&raw_yaml[nodes_idx..]);
            Ok(out)
        }
    }
}

/// Surgically REMOVE the top-level `process_group_order:`
/// block from a graph YAML document, if present.
///
/// The auto-partitioner's emitted `process_groups` LISTING order IS the
/// cross-process rank order, so an explicit `process_group_order:` list is
/// redundant after an emit — and STALE the moment the emitted group names
/// differ from the old ones (a stale list fails `validate_process_groups`
/// loudly at the next graph load). Removal is therefore mandatory on the emit
/// path.
///
/// Pure, no tracing: the loud removal warn fires at the
/// WRITE-COMMITTING sites (the internal `warn_order_block_removed`, invoked
/// when the rewritten YAML actually lands on disk). A `--dry-run`/preview
/// path computes the removal for its diff, and a warn claiming a removal
/// happened when nothing was written would be misleading.
///
/// Returns `(rewritten_yaml, removed_block_text)`:
/// * absent key → the input is returned UNCHANGED (byte-identical) with `None`;
/// * present once → the block (bounds from `scan_top_level_block`, the
///   single authority — interior column-0 comments belong to the block,
///   trailing ones stay outside) is removed; when the removal would leave a doubled blank line at the
///   seam (a blank line both before and after the old block), ONE of them is
///   consumed so the file keeps a single separator. The removed block's text is
///   returned for the caller's preview/diff.
///
/// # Errors
///
/// More than one top-level `process_group_order:` key — the block bounds are
/// ambiguous (refuse-and-diagnose, same philosophy as
/// [`rewrite_process_groups_block`]).
pub fn remove_process_group_order_block(
    raw_yaml: &str,
    source_label: &str,
) -> CliResult<(String, Option<String>)> {
    remove_top_level_block(raw_yaml, "process_group_order", source_label)
}

/// The KEY-GENERIC surgical top-level-block removal —
/// shared with [`remove_process_group_order_block`] (which
/// delegates here) so the `level_assignments:` no-op-removal path shares the one
/// scanning/splicing truth (`scan_top_level_block` + the doubled-blank-line
/// seam tidy-up) instead of re-deriving block bounds.
///
/// Same contract as the order removal: absent ⇒ input returned byte-identical
/// with `None`; present once ⇒ block removed (returned for the preview);
/// duplicate ⇒ refuse-and-diagnose naming the key lines.
fn remove_top_level_block(
    raw_yaml: &str,
    key: &str,
    source_label: &str,
) -> CliResult<(String, Option<String>)> {
    let lines = split_lines(raw_yaml);
    match scan_top_level_block(&lines, key) {
        BlockScan::Absent => Ok((raw_yaml.to_string(), None)),
        BlockScan::Duplicate { key_lines } => Err(CliError::Validation(format!(
            "cannot remove the {key} block from '{source_label}': found {} \
             top-level `{key}:` keys at column 0 (lines {}), so the block bounds \
             are ambiguous — a valid graph declares AT MOST ONE. Remove the duplicate(s) and \
             retry.",
            key_lines.len(),
            line_numbers(&key_lines)
        ))),
        BlockScan::One { start, end } => {
            let removed = raw_yaml[start..end].to_string();
            let prefix = &raw_yaml[..start];
            let mut suffix = &raw_yaml[end..];
            // Seam tidy-up: if a blank line preceded the block AND another
            // follows it, removing the block would leave a DOUBLED blank line —
            // consume one so exactly one separator survives. Deterministic and
            // byte-oracle-pinned; every other byte is untouched.
            if prefix.ends_with("\n\n") || prefix.ends_with("\n\r\n") {
                if let Some(rest) = suffix.strip_prefix("\r\n") {
                    suffix = rest;
                } else if let Some(rest) = suffix.strip_prefix('\n') {
                    suffix = rest;
                }
            }
            Ok((format!("{prefix}{suffix}"), Some(removed)))
        }
    }
}

// ===========================================================================
// The `cerulion graph partition` verb.
// ===========================================================================

/// Options for [`graph_partition`] (the `cerulion graph partition` verb).
#[derive(Debug, Clone)]
pub struct PartitionOptions {
    /// Explicit cost-artifact path (`--costs`). `None` ⇒ the default
    /// [`default_artifact_path`] (`graphs/<name>.costs.yaml`), which may
    /// legitimately be absent (⇒ baseline mode). An EXPLICITLY named path must
    /// exist — a named-but-missing file is a loud error, never a silent
    /// baseline fallback.
    pub costs_path: Option<PathBuf>,
    /// Per-group compute budget (ns) for the fusion gate (`--budget-ns`).
    /// `None` (the default) = use the cost artifact's FROZEN
    /// core-count budget (`derived_budget_ns`, computed at profile time);
    /// an older (v1) artifact or an absent frozen value falls back to
    /// UNBOUNDED fusion with a loud info suggesting a re-profile. An explicit
    /// `Some(n)` always overrides the frozen value. `Some(0)` is rejected
    /// loudly (clap already enforces `1..`; the engine re-validates —
    /// defense in depth). Resolution happens at ONE point
    /// (`derive_partition`), shared with the `graph run` preflight.
    pub budget_ns: Option<u64>,
    /// `--dry-run`: preview only, never write. WINS over `assume_yes` when
    /// both are set.
    pub dry_run: bool,
    /// `--yes`: skip the interactive confirm and write (scripts / non-TTY).
    pub assume_yes: bool,
}

/// Which cost mode [`graph_partition`] ran in (surfaced in the preview and
/// the CLI stanza).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PartitionMode {
    /// A cost artifact was read — cost-aware fused partition.
    Fused {
        /// The artifact that was read (explicit `--costs` or the default).
        artifact_path: PathBuf,
    },
    /// No artifact at the default path — process-per-node baseline.
    Baseline {
        /// The default artifact path that was checked and found absent (the
        /// "why" of the fallback, named in the preview).
        missing_artifact_path: PathBuf,
    },
    /// The ZERO-FLAG `graph run` default found the DEFAULT
    /// artifact PRESENT but unusable (unreadable / malformed / wrong version /
    /// hand-edit validation failure) and DEGRADED to the process-per-node
    /// baseline with a loud warn — the run proceeds. Only reachable under the
    /// internal lenient-default costs contract (the zero-flag run path); the
    /// verb and `--auto-partition` hard-error instead.
    BaselineDegraded {
        /// The present-but-unusable artifact (named in the warn + preview).
        artifact_path: PathBuf,
    },
}

/// What [`graph_partition`] did with the graph file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PartitionOutcome {
    /// `--dry-run`: previewed, nothing written.
    DryRun,
    /// The file already carries EXACTLY this partition (the rewrite is
    /// byte-identical to the current file) — nothing to write, no backup
    /// churn.
    Unchanged,
    /// Written through [`write_yaml_atomically`]; `backup` is the `.bak` of
    /// the prior file (always `Some` in practice — the graph file exists by
    /// definition on this path).
    Written {
        /// Where the pre-write file was backed up.
        backup: Option<PathBuf>,
    },
    /// The interactive confirm was answered "no" — nothing written.
    Declined,
}

/// The result of [`graph_partition`] — everything the binary needs to print.
#[derive(Debug, Clone)]
pub struct PartitionReport {
    /// The full preview: mode + summary, the proposed grouping's levelization
    /// bands (via the `graph levels` renderer), and the block-scoped YAML
    /// diff. The interactive confirm displays it; for every other outcome the
    /// binary prints it (see [`Self::preview_shown`]).
    pub preview: String,
    /// The derived grouping (drops into [`GraphConfig::process_groups`]).
    pub groups: IndexMap<String, Vec<String>>,
    /// Which cost mode ran.
    pub mode: PartitionMode,
    /// What happened to the file.
    pub outcome: PartitionOutcome,
    /// The graph file this verb targeted.
    pub graph_path: PathBuf,
    /// True when a stale top-level `process_group_order:` block was removed
    /// as part of the rewrite (loudly warned; shown in the diff).
    pub removed_process_group_order: bool,
    /// The rewrite carries a (fresh or replaced)
    /// `level_assignments:` block — cost-aware refinement moved ≥1 node off
    /// its derived Kahn level (shown in the diff).
    pub level_assignments_written: bool,
    /// A pre-existing `level_assignments:` block was REMOVED
    /// — the refined levels equal Kahn, and the block only exists when it
    /// changes something (loudly warned on write; shown in the diff).
    pub removed_level_assignments: bool,
    /// True when the confirm provider was invoked (it receives — and is
    /// responsible for displaying — the preview). False on the
    /// dry-run/unchanged/`--yes` paths, where the caller should print
    /// [`Self::preview`] itself.
    pub preview_shown: bool,
}

/// `cerulion graph partition <graph>` — derive the
/// process-group partition (cost-aware when a cost artifact exists,
/// process-per-node baseline otherwise) and SURGICALLY write it into the graph
/// YAML file.
///
/// # Modes
///
/// * `--costs PATH` names an artifact explicitly — it MUST exist and parse
///   (loud `Err` otherwise).
/// * No `--costs`: the default `graphs/<name>.costs.yaml` is used when
///   present; when absent, the PROCESS-PER-NODE baseline is emitted with a
///   loud `tracing::info!` naming the missing artifact and the
///   `graph profile` remedy. A PRESENT-but-malformed artifact is a hard `Err`
///   on both paths — never a silent fall back to baseline.
///
/// # Write protocol (the consent floor: never mutate without consent)
///
/// 1. `dry_run` → preview only ([`PartitionOutcome::DryRun`]); wins over
///    `assume_yes`.
/// 2. Rewrite byte-identical to the current file →
///    [`PartitionOutcome::Unchanged`], no write, no backup churn.
/// 3. `assume_yes` → write through [`write_yaml_atomically`] (`.bak` + loud
///    warn).
/// 4. No TTY and no `assume_yes` → loud `Err` naming `--yes` (scripts) and
///    `--dry-run` (inspection); the file is NEVER touched.
/// 5. Interactive: `confirm(&preview)` decides — `true` writes, `false` is
///    [`PartitionOutcome::Declined`].
///
/// A top-level `process_group_order:` block is REMOVED as part of the rewrite
/// (see [`remove_process_group_order_block`] — the emitted `process_groups`
/// listing order IS the rank order; a stale list would fail validation at the
/// next load). The removal appears in the preview diff.
///
/// `is_tty` is threaded (rather than read here) so the refusal arm is testable
/// without a real terminal; the binary passes
/// `std::io::stdin().is_terminal()`. `confirm` is only invoked on the
/// interactive arm and receives the full preview.
pub fn graph_partition(
    workspace_root: &Path,
    graph_name: &str,
    opts: &PartitionOptions,
    is_tty: bool,
    confirm: &mut dyn FnMut(&str) -> CliResult<bool>,
) -> CliResult<PartitionReport> {
    // Defense in depth with the clap `range(1..)` gate (CLI/engine contract
    // alignment): a 0 budget would reject EVERY fusion as over-budget, which
    // is never what a user meant — refuse loudly.
    if opts.budget_ns == Some(0) {
        return Err(CliError::Validation(
            "graph partition: --budget-ns must be > 0 (a zero budget rejects every fusion; \
             omit the flag to use the artifact's frozen core-count budget)"
                .to_string(),
        ));
    }

    let graph_path = workspace_root
        .join("graphs")
        .join(format!("{graph_name}.yaml"));
    if !graph_path.exists() {
        return Err(CliError::GraphNotFound {
            name: graph_name.to_string(),
        });
    }
    let raw = std::fs::read_to_string(&graph_path)?;
    // The raw parser (faithful round-trips); resolve the default prefix
    // exactly like graph_run/graph_levels before validation.
    let mut config = parse_graph_raw(&raw)?;
    // This verb loads the file itself rather than through
    // `graph_read`, so it adopts the stem itself — otherwise the identity
    // would fall back to whatever legacy `name:` the file happens to carry.
    cerulion_core::graph::adopt_file_stem_identity(&mut config, graph_name);
    if config.prefix.is_empty() {
        config.prefix = default_prefix(config.identity());
    }
    // Replace-scoped validation: this verb is the recovery tool
    // for a stale/broken partition block — refusing to fix the exact blocks it
    // exists to rewrite would be user-hostile. Every non-partition validation
    // arm stays mandatory.
    validate_graph_replace_scoped(&config)?;

    let derived = derive_partition(
        workspace_root,
        graph_name,
        &config,
        &raw,
        &graph_path,
        // The verb is the EXPLICIT cost-aware surface — a broken artifact is
        // a hard Err (never a silent baseline).
        &CostsRequest {
            explicit_path: opts.costs_path.as_deref(),
            budget_ns: opts.budget_ns,
            lenient_default: false,
        },
        // The verb IS the refinement surface — it emits (or
        // removes) the `level_assignments:` block and bands the partition
        // over the refined levels, in ONE consented rewrite.
        true,
    )?;

    // The write decision ladder (the consent floor).
    let mut preview_shown = false;
    let outcome = if opts.dry_run {
        PartitionOutcome::DryRun
    } else if derived.unchanged {
        PartitionOutcome::Unchanged
    } else if opts.assume_yes {
        let backup = write_graph_locked(
            workspace_root,
            &graph_path,
            &derived.new_yaml,
            "graph file",
            ExpectedPrior::Contents(&raw),
        )?;
        if derived.removed_order {
            warn_order_block_removed(&graph_path);
        }
        warn_creditable_split_overwritten(&graph_path, &derived.overwritten_creditable);
        if derived.removed_level_assignments {
            warn_level_block_removed(&graph_path);
        }
        PartitionOutcome::Written { backup }
    } else if !is_tty {
        return Err(CliError::Validation(format!(
            "graph partition would modify '{}' but stdin is not a TTY and --yes was not \
             passed — nothing was written. Re-run with --yes to apply non-interactively \
             (scripts/CI), or --dry-run to inspect the proposed change without writing.",
            graph_path.display()
        )));
    } else {
        preview_shown = true;
        if confirm(&derived.preview)? {
            let backup = write_graph_locked(
                workspace_root,
                &graph_path,
                &derived.new_yaml,
                "graph file",
                ExpectedPrior::Contents(&raw),
            )?;
            if derived.removed_order {
                warn_order_block_removed(&graph_path);
            }
            warn_creditable_split_overwritten(&graph_path, &derived.overwritten_creditable);
            if derived.removed_level_assignments {
                warn_level_block_removed(&graph_path);
            }
            PartitionOutcome::Written { backup }
        } else {
            PartitionOutcome::Declined
        }
    };

    Ok(PartitionReport {
        preview: derived.preview,
        groups: derived.groups,
        mode: derived.mode,
        outcome,
        graph_path,
        removed_process_group_order: derived.removed_order,
        level_assignments_written: derived.level_assignments_written,
        removed_level_assignments: derived.removed_level_assignments,
        preview_shown,
    })
}

/// How a partition flow sources its cost snapshot (see [`resolve_costs`]).
struct CostsRequest<'a> {
    /// Explicit `--costs` path (`graph partition` only); `None` ⇒ the default
    /// `graphs/<name>.costs.yaml`.
    explicit_path: Option<&'a Path>,
    /// Per-group fusion budget OVERRIDE (ns). `None` ⇒ resolve to
    /// the artifact's frozen `derived_budget_ns` (unbounded fallback, loud,
    /// when absent) at the SINGLE resolution point in [`derive_partition`] —
    /// the verb and the `graph run` preflight cannot diverge.
    budget_ns: Option<u64>,
    /// On the zero-flag `graph run` default path a
    /// present-but-unusable DEFAULT artifact degrades to the baseline with a
    /// loud warn (a hand-edit typo must not abort a plain run that never
    /// asked for cost-aware behavior). `false` (the verb + `--auto-partition`
    /// — the user asked for cost-aware) keeps the hard `Err`.
    lenient_default: bool,
}

/// The shared derive-and-render core of the partition flows — everything
/// between "a parsed, prefix-resolved graph" and "a consent decision":
/// cost-mode resolution, group derivation, the bands+diff preview, and the
/// spliced replacement YAML. Used by [`graph_partition`] (the verb) and
/// [`run_auto_partition_preflight`] (the `graph run` mp-default pre-flight) so
/// the two can NEVER diverge on what is derived or written.
struct DerivedPartition {
    /// The derived grouping.
    groups: IndexMap<String, Vec<String>>,
    /// Which cost mode ran.
    mode: PartitionMode,
    /// The full preview (mode + summary, proposed bands, block-scoped diff) —
    /// WITHOUT any consent-hint line (each caller appends its own).
    preview: String,
    /// The complete replacement file contents (order block stripped, groups
    /// block spliced; on the refining verb path also the `level_assignments:`
    /// block spliced-or-removed).
    new_yaml: String,
    /// `new_yaml` is byte-identical to the current file.
    unchanged: bool,
    /// A top-level `process_group_order:` block was stripped (warned at the
    /// write-committing sites only).
    removed_order: bool,
    /// Split `block` edges the EXISTING hand-written partition
    /// made creditable, which writing the derived block would replace with a
    /// co-located flow. Warned at the write-committing sites, like
    /// `removed_order` — the preview alone is not enough, because
    /// `graph run --auto-partition --yes` never renders it.
    overwritten_creditable: Vec<cerulion_core::graph::CreditableSplitEdge>,
    /// `new_yaml` carries a (fresh or replaced)
    /// `level_assignments:` block — cost-aware refinement moved ≥1 node off
    /// its Kahn level. Always `false` on the non-refining (`graph run`
    /// preflight) path.
    level_assignments_written: bool,
    /// A pre-existing `level_assignments:` block was REMOVED
    /// because the refined levels equal Kahn (the block only exists when it
    /// changes something). Warned at the write-committing sites only, like
    /// `removed_order`. Always `false` on the non-refining path.
    removed_level_assignments: bool,
}

fn derive_partition(
    workspace_root: &Path,
    graph_name: &str,
    config: &GraphConfig,
    raw: &str,
    graph_path: &Path,
    costs: &CostsRequest<'_>,
    refine: bool,
) -> CliResult<DerivedPartition> {
    use std::fmt::Write as _;

    // `refine` (cost-aware level refinement + the
    // `level_assignments:` block emit) is the `graph partition` VERB's arm
    // only. The `graph run` preflight passes `false` — REFINEMENT REQUIRES
    // THE WRITTEN BLOCK: an in-memory refined-levels override would band
    // groups over levels the yaml does not carry, so the runtime
    // (`resolve_levels` over the file's config) and a bag replay of that file
    // would run DIFFERENT levels than the bands assumed (Replay=Live,
    // Principle #7). The preflight instead bands over `resolve_levels` of the
    // file's own state — Kahn when no block, the persisted assignment when a
    // prior `graph partition` wrote one — which is exactly what the runtime
    // runs either way. Refinement is opt-in via `graph partition` consent.
    //
    // The lenient-costs degrade arms below assume the artifact can be dropped
    // AFTER derivation decisions; refinement reads the artifact EARLY, so the
    // two flags must never combine (the verb is strict; the preflight is the
    // only lenient caller and never refines).
    debug_assert!(
        !(refine && costs.lenient_default),
        "refine (the verb) and lenient_default (the run preflight) are mutually exclusive"
    );

    let nodes_dir = workspace_root.join("nodes");
    let entry_infos = source_entry_infos(
        &nodes_dir,
        config,
        "graph partition derives the grouping from each node's declared trigger policy",
    )?;
    let trigger_edges = build_trigger_edges(config, &entry_infos);

    let (mut artifact, mut mode) = resolve_costs(
        workspace_root,
        graph_name,
        costs.explicit_path,
        costs.lenient_default,
    )?;

    // The frozen-ZERO guard.
    // `from_profile` floors the frozen value at 1, but the artifact is a
    // DOCUMENTED hand-edit surface — a hand-edited `derived_budget_ns: 0`
    // would otherwise silently force process-per-node with only an info
    // breadcrumb. Only consulted when NO --budget-ns override is present
    // (the override never touches the frozen value). Two arms:
    // * STRICT paths (the `graph partition` verb, `--auto-partition`) — a
    //   hard Err mirroring the --budget-ns override guard;
    // * the ZERO-FLAG `graph run` lenient default (`lenient_default`) — a
    //   hand-edit typo must not abort a plain run that never
    //   asked for cost-aware behavior: DEGRADE to the process-per-node
    //   baseline with the SAME loud warn class as the other
    //   present-but-unusable-artifact arms, and accurate `BaselineDegraded`
    //   preview semantics.
    if costs.budget_ns.is_none()
        && artifact
            .as_ref()
            .is_some_and(|a| a.derived_budget_ns == Some(0))
    {
        if costs.lenient_default {
            let artifact_path = match &mode {
                PartitionMode::Fused { artifact_path } => artifact_path.clone(),
                // Unreachable: a present artifact only pairs with Fused.
                _ => default_artifact_path(workspace_root, graph_name),
            };
            let reason = "the artifact's derived_budget_ns is 0 — a zero budget rejects \
                          every fusion, which is never what a profile freezes \
                          (hand-edited?); re-profile to freeze a valid core-count budget, \
                          or pass --budget-ns via `cerulion graph partition`";
            warn_default_artifact_degraded(graph_name, &artifact_path, &reason);
            mode = PartitionMode::BaselineDegraded { artifact_path };
            artifact = None;
        } else {
            return Err(CliError::Validation(
                "the artifact's derived_budget_ns is 0 — a zero budget rejects every \
                 fusion, which is never what a profile freezes (hand-edited?); \
                 re-profile to freeze a valid core-count budget, or override it via \
                 `cerulion graph partition --budget-ns <n>`"
                    .to_string(),
            ));
        }
    }

    // THE single budget resolution point — both the `graph
    // partition` verb and the `graph run` preflight route through here, so
    // the two surfaces cannot diverge. Precedence:
    //   explicit --budget-ns override  >  the artifact's FROZEN core-count
    //   budget (`derived_budget_ns`, computed at profile time — never
    //   re-derived from THIS machine's cores)  >  unbounded (u64::MAX, loud).
    // On the baseline path (no artifact) the budget is moot — baseline
    // derivation consumes no costs — so the quiet u64::MAX fallback is
    // deliberately info-free there (no "re-profile" nag for a graph that was
    // never profiled; resolve_costs already announced baseline mode).
    let effective_budget_ns = match (costs.budget_ns, &artifact) {
        (Some(explicit), _) => explicit,
        (None, Some(artifact)) => match artifact.derived_budget_ns {
            // `Some(0)` is unreachable here — the frozen-ZERO guard above
            // either erred (strict) or dropped the artifact (lenient).
            Some(frozen) => {
                tracing::info!(
                    graph = %graph_name,
                    budget_ns = frozen,
                    profile_cores = ?artifact.profile_cores,
                    "graph partition: using the artifact's frozen core-count budget \
                     (override with --budget-ns)"
                );
                frozen
            }
            None => {
                tracing::info!(
                    graph = %graph_name,
                    "graph partition: no frozen budget in the artifact (an older \
                     profile?) — running UNBOUNDED fusion; re-profile to freeze a \
                     core-count budget"
                );
                u64::MAX
            }
        },
        (None, None) => u64::MAX,
    };

    // The refinement stage — Kahn base → cost-fed
    // `refine_levels` → the `level_assignments` emit decision. The Kahn base
    // deliberately IGNORES any existing `level_assignments:` block (this is a
    // re-derivation; a stale block is replaced, the recovery contract), which
    // also makes the verb idempotent: same costs ⇒ same refined levels ⇒ same
    // block. The rule: the block is emitted only when
    // refinement moves ≥1 node — `refined == kahn` (including the no-costs
    // baseline arm, where empty `RefineInputs` returns Kahn byte-identical)
    // OMITS it and REMOVES a stale existing one, so the block's presence
    // always means "these levels differ from Kahn". A hand-written block that
    // happens to equal Kahn is removed too (it changed nothing) — the preview
    // names the removal.
    //
    // Level-count GROWTH is SHAPE-GATED. Growth (the phase-2
    // chain-cascade appending levels) helps a monolith (measured −10% p50,
    // tail collapse) but costs a multi-process split ~19% chain cadence at
    // neutral p50 (+1 level = +1 barrier generation per step), so the verb
    // pre-bands the KAHN view — the real emitted-shape signal, correct even
    // on the legacy unbounded-budget arm where a Σ-vs-budget heuristic
    // would misread — and DENIES growth for a multi-group-destined graph
    // (single-group keeps Allow growth). Refinement and banding are
    // PAIRED in one closure so the groups always band over EXACTLY the
    // levels the refinement produced (the point of the pairing). FIXED
    // POINT: if the Kahn pre-band predicted single-group but the GROWN
    // banding comes out multi-group (growth falsified its own signal), the
    // refinement re-runs ONCE with growth denied and THAT result is banded —
    // Deny is the fixed point (a Deny output re-banding multi-group already
    // paid no growth, so no further pass can help; the reverse flip —
    // Deny-refined levels banding SINGLE-group — is accepted conservatively
    // rather than re-opened with Allow, which could oscillate). The gate
    // fires on the emitted level COUNT (`refined.len() > kahn.len()`): the
    // count is what drives barrier generations per step, so a refinement
    // whose final count stays ≤ Kahn's costs the split nothing regardless of
    // its internal cascade history.
    let mut effective_config = config.clone();
    let mut refined_moved: usize = 0;
    // The growth policy the FINAL refinement ran under (Deny appends
    // the denial clause to the written-block notice) + whether the fixed-point
    // re-refine fired (surfaced in the preview + a loud `info!`).
    let mut growth_denied = false;
    let mut growth_fixed_point = false;
    // On the refining (verb) path the banding is computed HERE, paired with
    // the refinement it banded (`Some(groups)`); the non-refining preflight
    // keeps the standalone derivation below (`None`).
    let refine_banded: Option<IndexMap<String, Vec<String>>> = if refine {
        let mut kahn_view = config.clone();
        kahn_view.level_assignments = None;
        let base_topology = GraphTopology::build(&kahn_view, &entry_infos)?;
        let kahn = cerulion_core::graph::resolve_levels(&kahn_view, &base_topology, &trigger_edges)
            .map_err(CliError::from)?;
        let refine_inputs = match &artifact {
            Some(a) => {
                // `to_costs` re-runs its (pure) hand-edit validation — cheap,
                // and every `derive_emit_groups` call below thaws the same
                // artifact identically, so the stages can never see
                // different costs.
                let (costs, _isolated) = a.to_costs()?;
                refine_inputs_from_costs(&costs)
            }
            None => cerulion_core::graph::RefineInputs::default(),
        };

        // The PRE-BAND shape signal — band the KAHN view (no
        // refinement) to learn the graph's REAL destination shape. `> 1`
        // groups ⇒ multi-process-destined ⇒ growth denied. `<= 1` ⇒ growth
        // allowed; the only `0` arm is a node-less
        // graph, where refinement is vacuous (nothing to grow) and the emit
        // fails loudly at the empty-groups rewrite guard regardless.
        let kahn_groups = derive_emit_groups(
            &kahn_view,
            &entry_infos,
            &trigger_edges,
            artifact.as_ref(),
            effective_budget_ns,
        )?;
        let mut growth = if kahn_groups.len() > 1 {
            LevelGrowth::Deny
        } else {
            LevelGrowth::Allow
        };

        // The PAIRED refine+band step: one growth policy in, the refined
        // levels + the candidate config carrying their emit decision + the
        // groups banded over EXACTLY those levels out. Called once, or twice
        // when the fixed point fires — never with mismatched halves.
        let refine_and_band = |growth: LevelGrowth| -> CliResult<(
            Levels,
            GraphConfig,
            IndexMap<String, Vec<String>>,
        )> {
            let mut inputs = refine_inputs.clone();
            inputs.level_growth = growth;
            let refined = base_topology.refine_levels(&kahn, &trigger_edges, &inputs);
            let mut candidate = config.clone();
            candidate.level_assignments = if refined == kahn {
                None
            } else {
                // Full coverage in graph order (the rewrite's contract;
                // refinement partitions every node, so the lookup is total).
                Some(
                    kahn_view
                        .nodes
                        .iter()
                        .map(|n| {
                            let level = refined
                                .level_of(&n.id)
                                .expect("refine_levels partitions every graph node");
                            (n.id.clone(), level)
                        })
                        .collect(),
                )
            };
            let groups = derive_emit_groups(
                &candidate,
                &entry_infos,
                &trigger_edges,
                artifact.as_ref(),
                effective_budget_ns,
            )?;
            Ok((refined, candidate, groups))
        };

        let (mut refined, mut banded_config, mut groups) = refine_and_band(growth)?;

        // The fixed point: the single-group signal let growth run, but
        // the GROWN levels band multi-group — deny the growth and band the
        // denied result (exactly one extra pass; deterministic).
        if growth == LevelGrowth::Allow && refined.len() > kahn.len() && groups.len() > 1 {
            tracing::info!(
                graph = %graph_name,
                kahn_levels = kahn.len(),
                grown_levels = refined.len(),
                groups = groups.len(),
                "graph partition: level-growth fixed point — the Kahn shape pre-banded \
                 single-group (growth allowed), but the grown levels band MULTI-group; \
                 re-refining with growth denied so the multi-process split pays no extra \
                 barrier generation per step"
            );
            growth = LevelGrowth::Deny;
            (refined, banded_config, groups) = refine_and_band(growth)?;
            growth_fixed_point = true;
        }
        growth_denied = growth == LevelGrowth::Deny;
        refined_moved = kahn_view
            .nodes
            .iter()
            .filter(|n| refined.level_of(&n.id) != kahn.level_of(&n.id))
            .count();
        effective_config = banded_config;
        Some(groups)
    } else {
        None
    };

    let groups = match refine_banded {
        // The verb path already banded inside the paired
        // refine+band flow above — re-deriving here would risk banding over
        // levels other than the refinement's.
        Some(groups) => groups,
        None => match derive_emit_groups(
            &effective_config,
            &entry_infos,
            &trigger_edges,
            artifact.as_ref(),
            effective_budget_ns,
        ) {
            Ok(groups) => groups,
            // On the zero-flag lenient path,
            // ANY cost-derivation failure arising from the artifact degrades to
            // the baseline — e.g. a stale-but-well-formed snapshot missing a
            // newly-added node's cost (auto_partition's missing-p50 error).
            // Strict paths keep every hard Err.
            //
            // The DISCRIMINATOR: retry the BASELINE (which consumes no costs)
            // FIRST, silently. If the retry SUCCEEDS, the artifact-driven path
            // was the differentiator ⇒ the "artifact UNUSABLE" warn is accurate
            // — emit it and proceed BaselineDegraded. If the retry ALSO fails,
            // the failure is STRUCTURAL (a trigger cycle, a topology error) and
            // the artifact was never the problem: propagate the structural error
            // with NO artifact-blaming warn (a warn fired before the
            // retry would blame a valid artifact for a broken graph).
            Err(e) if costs.lenient_default && artifact.is_some() => {
                // (`effective_config == config` here — the lenient path never
                // refines, per the debug_assert at entry.)
                //
                // A structural retry failure propagates as-is (`?` — rustc
                // 1.97's clippy::question_mark insists on the sugar here).
                let groups = derive_emit_groups(
                    &effective_config,
                    &entry_infos,
                    &trigger_edges,
                    None,
                    effective_budget_ns,
                )?;
                let artifact_path = match &mode {
                    PartitionMode::Fused { artifact_path } => artifact_path.clone(),
                    // Unreachable: `artifact.is_some()` only pairs with Fused.
                    _ => default_artifact_path(workspace_root, graph_name),
                };
                warn_default_artifact_degraded(graph_name, &artifact_path, &e);
                mode = PartitionMode::BaselineDegraded { artifact_path };
                groups
            }
            Err(e) => return Err(e),
        },
    };

    // The PROPOSED grouping's levelization preview, via the `graph levels`
    // renderer over a config clone carrying the derived groups (grouping does
    // not affect levelization). The clone bases off
    // `effective_config` and levelizes through `resolve_levels`, so the
    // preview bands + verdict are computed over EXACTLY the levels the
    // post-write runtime will run (the refined assignment on the verb path;
    // the file's own state on the preflight path).
    let mut proposed = effective_config.clone();
    proposed.process_groups = groups.clone();
    proposed.process_group_order = Vec::new();
    let topology = GraphTopology::build(&proposed, &entry_infos)?;
    let levels = cerulion_core::graph::resolve_levels(&proposed, &topology, &trigger_edges)
        .map_err(CliError::from)?;
    let levels_report =
        render_levels_report(&proposed, &entry_infos, &trigger_edges, &topology, &levels)?;
    if let Some(diag) = levels_report.partition_error {
        // auto_partition/baseline emit spawner-consumable groups by
        // construction — reaching this is an internal bug, not a user error.
        return Err(CliError::Validation(format!(
            "internal error: the derived partition failed its own spawner-consumability \
             check — {diag}"
        )));
    }

    // New file contents: strip any stale process_group_order, splice the
    // derived groups block, then (verb path only) splice-or-remove the
    // `level_assignments:` block per the emit decision. Every step is
    // surgical (comment-preserving) and loud on ambiguous structure.
    let label = graph_path.display().to_string();
    let (without_order, removed_order) = remove_process_group_order_block(raw, &label)?;
    let with_groups = rewrite_process_groups_block(&without_order, &groups, &label)?;
    // The pre-existing level_assignments block's text, from the ORIGINAL
    // bytes (for the diff). Duplicate folds into Absent harmlessly — the
    // rewrite/remove below refuses the duplicate first on the paths that
    // touch the block.
    let old_level_block = match scan_top_level_block(&split_lines(raw), "level_assignments") {
        BlockScan::One { start, end } => Some(raw[start..end].to_string()),
        _ => None,
    };
    let (new_yaml, level_assignments_written, removed_level_assignments) = if refine {
        match &effective_config.level_assignments {
            Some(map) => (
                rewrite_level_assignments_block(&with_groups, map, &label)?,
                true,
                false,
            ),
            None => {
                let (removed_yaml, removed) =
                    remove_top_level_block(&with_groups, "level_assignments", &label)?;
                (removed_yaml, false, removed.is_some())
            }
        }
    } else {
        // The preflight never touches the block: an existing (persisted)
        // `level_assignments:` survives byte-identically through the
        // process_groups splice, and the banding above consumed it.
        (with_groups, false, false)
    };
    let unchanged = new_yaml == raw;

    // The current block's text (for the block-scoped diff), from the ORIGINAL
    // bytes. A Duplicate scan is unreachable here — the rewrite above already
    // refused it — so it folds into the Absent arm harmlessly.
    let old_block = match scan_top_level_block(&split_lines(raw), "process_groups") {
        BlockScan::One { start, end } => Some(raw[start..end].to_string()),
        _ => None,
    };

    // The co-location constraints this partition had to satisfy.
    // Derived from the SAME `block_colocation_seeds` the partitioner seeds on
    // (one function, called twice over the same topology — not a second copy of
    // the rule), so the preview cannot describe a different partition than the
    // one below it.
    let colocation_seeds = block_colocation_seeds(&topology);

    // A hand-written partition may DELIBERATELY
    // split a `block` edge — that is legal, and the supervisor mints it a
    // credit word. The derivation always co-locates (a boundary costs a real
    // hop), so writing the derived block over it DESTROYS that choice. The
    // `.bak` and the diff mitigate it, but neither says what was lost, and
    // `--yes` skips the diff entirely. Name it.
    let overwritten_creditable = if config.process_groups.is_empty() {
        Vec::new()
    } else {
        cerulion_core::graph::creditable_split_block_edges(
            &config.process_groups,
            config,
            &entry_infos,
            &trigger_edges,
        )
    };

    // Assemble the preview: mode + summary, the proposed bands, the diff.
    let mut preview = String::new();
    let _ = writeln!(preview, "graph partition: {graph_name}");
    if !overwritten_creditable.is_empty() {
        let _ = writeln!(
            preview,
            "WARNING: your existing `process_groups:` deliberately SPLITS {} `block` edge(s) \
             that the supervisor credits with a cross-process word ({}). The derived \
             partition CO-LOCATES every `block` flow, so writing it REPLACES that choice \
             with a co-located one — legal, and one process hop cheaper, but no longer the \
             deployment you wrote. If you go ahead, the previous file is kept as `.bak`.",
            overwritten_creditable.len(),
            overwritten_creditable
                .iter()
                .map(|e| e.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    // A `block` edge makes the baseline NOT process-per-node — the
    // seeded groups are multi-member by construction. The label says which
    // shape the bands below actually are; claiming "maximal fault isolation"
    // over a merged group is a mode line its own output contradicts.
    let baseline_label = if colocation_seeds.is_empty() {
        "process-per-node baseline"
    } else {
        "process-per-node baseline + `block` co-location"
    };
    match &mode {
        PartitionMode::Fused { artifact_path } => {
            let _ = writeln!(
                preview,
                "mode: cost-aware fusion (artifact: {})",
                artifact_path.display()
            );
        }
        PartitionMode::Baseline {
            missing_artifact_path,
        } => {
            let _ = writeln!(
                preview,
                "mode: {baseline_label} (no cost artifact at {} — run `cerulion graph \
                 profile {graph_name}` for a cost-aware partition)",
                missing_artifact_path.display()
            );
        }
        PartitionMode::BaselineDegraded { artifact_path } => {
            let _ = writeln!(
                preview,
                "mode: {baseline_label} (cost artifact {} is PRESENT but UNUSABLE — \
                 see the warning; fix or delete it, or re-run `cerulion graph profile \
                 {graph_name}` to regenerate)",
                artifact_path.display()
            );
        }
    }
    let _ = writeln!(
        preview,
        "nodes: {}  groups: {}",
        config.nodes.len(),
        groups.len()
    );
    // Surface the fixed-point re-refine — the operator should see
    // that the single-group Kahn signal flipped multi-group under growth and
    // the emitted levels therefore paid none. Shown on every consent path
    // (dry-run included): it is derivation reality, not a write effect.
    if growth_fixed_point {
        let _ = writeln!(
            preview,
            "level growth: the single-group Kahn shape re-banded MULTI-group after level \
             growth — re-refined with growth denied"
        );
    }
    // The co-location constraints, on the surface the operator
    // actually reads before consenting. The announcements the partitioner logs
    // are `tracing` events, and `graph partition` is classified OneShot (its
    // default filter is `cerulion=warn`), so on a default invocation they do
    // not print at all — which would leave a merged group in the bands below
    // with no rationale anywhere. Shown on EVERY consent path (dry-run
    // included), following the level-growth precedent: it is derivation
    // reality, not a write effect.
    if !colocation_seeds.is_empty() {
        let owner = |node: &str| -> String {
            groups
                .iter()
                .find(|(_, members)| members.iter().any(|m| m == node))
                .map(|(name, _)| name.clone())
                .unwrap_or_else(|| "<unplaced>".to_string())
        };
        let _ = writeln!(
            preview,
            "`block` co-location: {} topic(s) keep their WHOLE flow in one process group\
             ",
            colocation_seeds.len()
        );
        for seed in &colocation_seeds {
            let group = seed
                .producers
                .first()
                .map(|p| owner(p))
                .unwrap_or_else(|| "<unplaced>".to_string());
            let _ = writeln!(
                preview,
                "  - {} -> group '{group}': producer(s) {}; `block` input(s) {}{}",
                seed.topic,
                seed.producers.join(", "),
                seed.block_consumers
                    .iter()
                    .map(|(c, i)| format!("{c}.{i}"))
                    .collect::<Vec<_>>()
                    .join(", "),
                if seed.other_consumers.is_empty() {
                    String::new()
                } else {
                    format!(
                        "; non-`block` sibling(s) {}",
                        seed.other_consumers
                            .iter()
                            .map(|(c, i)| format!("{c}.{i}"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    )
                }
            );
        }
        // A split flow does not "refuse to
        // build": a creditable split runs on a
        // supervisor-minted credit word. Co-location is a latency PREFERENCE
        // the derivation always takes (a boundary costs a real hop whether or
        // not the edge is correct), not a refusal it is enforcing.
        let _ = writeln!(
            preview,
            "  (co-located because crossing a process boundary costs a real hop; a split \
             `block` edge is legal only where the supervisor can mint it a cross-process \
             credit word — one in-graph producer, no non-`block` consumers. The non-`block` \
             siblings join too, or the worker would read its flow as all-`block` and INSTALL \
             a defer that `--single-process` degrades)"
        );
    }
    let _ = writeln!(preview);
    preview.push_str(&levels_report.rendered);
    let _ = writeln!(preview);
    if unchanged {
        let _ = writeln!(
            preview,
            "no change: {label} already carries exactly this partition"
        );
    } else {
        let _ = writeln!(preview, "proposed change to {label}:");
        let _ = writeln!(preview, "--- current");
        let _ = writeln!(preview, "+++ proposed");
        match &old_block {
            Some(old) => {
                for line in old.lines() {
                    let _ = writeln!(preview, "-{line}");
                }
            }
            None => {
                let _ = writeln!(
                    preview,
                    "(no existing process_groups block — inserting before `nodes:`)"
                );
            }
        }
        for line in render_process_groups_block(&groups).lines() {
            let _ = writeln!(preview, "+{line}");
        }
        if let Some(order) = &removed_order {
            for line in order.lines() {
                let _ = writeln!(preview, "-{line}");
            }
            let _ = writeln!(
                preview,
                "(process_group_order removed: the emitted process_groups listing order IS the \
                 rank order)"
            );
        }
        // The level_assignments delta. Written/updated shows
        // the -old/+new block lines + WHY (how many nodes refinement moved);
        // removed shows the -old lines + the "block only exists when it
        // changes something" rule (which also covers the hand-written-equals-
        // Kahn case — such a block changes nothing and is removed, named
        // here). The non-refining preflight never reaches these arms.
        if level_assignments_written {
            if let Some(old) = &old_level_block {
                for line in old.lines() {
                    let _ = writeln!(preview, "-{line}");
                }
            } else {
                let _ = writeln!(
                    preview,
                    "(no existing level_assignments block — inserting before `nodes:`)"
                );
            }
            if let Some(map) = &effective_config.level_assignments {
                for line in render_level_assignments_block(map).lines() {
                    let _ = writeln!(preview, "+{line}");
                }
            }
            // When the refinement ran under Deny the notice states
            // the growth policy — the emitted assignment carries no level
            // beyond the Kahn count.
            let growth_clause = if growth_denied {
                "; level-count growth denied (multi-process shape)"
            } else {
                ""
            };
            let _ = writeln!(
                preview,
                "(level_assignments written: cost-aware refinement moved {refined_moved} \
                 node(s) off their derived Kahn level{growth_clause}; the runtime consumes \
                 this block)"
            );
        } else if removed_level_assignments {
            if let Some(old) = &old_level_block {
                for line in old.lines() {
                    let _ = writeln!(preview, "-{line}");
                }
            }
            let _ = writeln!(
                preview,
                "(level_assignments removed: the refined levels equal the derived Kahn \
                 levelization, so the block changed nothing — it is only emitted when \
                 refinement moves a node. A hand-written block that happens to equal Kahn \
                 is removed for the same reason)"
            );
        }
    }

    Ok(DerivedPartition {
        groups,
        mode,
        preview,
        new_yaml,
        unchanged,
        removed_order: removed_order.is_some(),
        overwritten_creditable,
        level_assignments_written,
        removed_level_assignments,
    })
}

/// The loud half of a `level_assignments` block REMOVAL —
/// same write-committing discipline as [`warn_order_block_removed`]:
/// fires only when the removal actually lands on disk;
/// preview/dry-run paths show the removal in the diff instead.
fn warn_level_block_removed(graph_path: &Path) {
    tracing::warn!(
        graph_file = %graph_path.display(),
        "graph partition: removed the level_assignments block — the refined levels equal the \
         derived Kahn levelization, so the block changed nothing (it is only emitted when \
         cost-aware refinement moves a node off its Kahn level). The runtime now derives \
         levels directly"
    );
}

/// The loud half of the `process_group_order` removal: fires
/// ONLY when the removal is actually COMMITTED to disk — every write site
/// calls this right after a successful [`write_yaml_atomically`] when the
/// spliced contents had an order block stripped. Preview/dry-run paths show
/// the removal in the diff instead of warning about a write that never
/// happened.
/// The ONE warn surface for overwriting a
/// hand-written CREDITABLE split.
///
/// The consent PREVIEW already names this, but the preview is not a channel
/// every path reads: `graph run --auto-partition --yes` drops
/// `pre.preview` entirely and prints only "partition written (--yes)", so the
/// scriptable path — the one most likely to run unattended — replaced a
/// deliberate, legal, credited deployment in silence. Warning at the WRITE
/// site means no caller can lose it, which is exactly why
/// `warn_order_block_removed` sits here too.
fn warn_creditable_split_overwritten(
    graph_path: &Path,
    overwritten: &[cerulion_core::graph::CreditableSplitEdge],
) {
    if overwritten.is_empty() {
        return;
    }
    tracing::warn!(
        graph_file = %graph_path.display(),
        edges = %overwritten
            .iter()
            .map(|e| e.to_string())
            .collect::<Vec<_>>()
            .join(", "),
        count = overwritten.len(),
        "graph partition: REPLACED a hand-written partition that deliberately SPLIT \
         `block` edge(s) the supervisor can credit with a cross-process word. The derived \
         partition co-locates every `block` flow — legal, and one process hop cheaper, but \
         no longer the deployment that was written. The previous file is kept as `.bak`"
    );
}

fn warn_order_block_removed(graph_path: &Path) {
    tracing::warn!(
        graph_file = %graph_path.display(),
        "graph partition: removed the process_group_order block — the emitted process_groups \
         listing order IS the cross-process rank order, and a stale explicit order list (naming \
         the old group names) would fail validation at the next graph load"
    );
}

/// The ONE warn surface for the lenient-costs degrade (cause-
/// parameterized): fired when the ZERO-FLAG `graph run` default finds the
/// DEFAULT cost artifact unusable — unreadable / malformed / wrong version
/// (from `resolve_costs`) OR failing cost DERIVATION against this graph, e.g.
/// a stale snapshot missing a newly-added node's cost (from the
/// `derive_partition` retry). `error` carries the cause (auto_partition's
/// message names the failing node); the text names both remedies.
fn warn_default_artifact_degraded(
    graph_name: &str,
    artifact_path: &Path,
    error: &dyn std::fmt::Display,
) {
    tracing::warn!(
        graph = %graph_name,
        artifact = %artifact_path.display(),
        error = %error,
        "graph run: the default cost artifact is PRESENT but UNUSABLE for this graph — \
         falling back to the process-per-node baseline partition for this run. Re-run \
         `cerulion graph profile` to refresh the snapshot (or fix/delete the artifact); an \
         explicit `cerulion graph partition` / `--auto-partition` would refuse instead"
    );
}

/// Replace-scoped graph validation for the partition flows. It runs
/// every validation arm EXCEPT the checks on the blocks the caller is about
/// to REPLACE: `process_groups`/`process_group_order`, and
/// `level_assignments` — the verb re-derives and splices-or-removes that
/// block too, so it is equally the recovery tool for a stale/invalid one (a
/// hand-mangled assignment must be REPLACEABLE, not fatal). On the
/// `graph run` preflight path (which never rewrites `level_assignments`) an
/// invalid block still fails LOUDLY downstream at `resolve_levels` with its
/// own diagnostic — nothing goes silent. Implemented by validating a
/// clone with the fields cleared, so this can never drift from
/// [`validate_graph`]'s other arms. INTERNAL to the partition flows —
/// deliberately not a public `--no-validate`-style bypass.
pub(crate) fn validate_graph_replace_scoped(
    config: &GraphConfig,
) -> cerulion_core::TransportResult<()> {
    validate_graph(&replace_scoped_config(config))
}

/// The config [`validate_graph_replace_scoped`] validates: the caller's, with
/// every partition field CLEARED.
///
/// Exposed separately so `graph run`'s fail-closed validation REPORT can be
/// produced under the SAME scoping the recovery path validates under. Without
/// that, the report refuses a stale `process_groups:` block before
/// `resolve_partition_intent` can pick the recovery that repairs it, and
/// `--auto-partition` — the documented recovery tool for exactly that block —
/// cannot run. One definition of "replace-scoped", called
/// twice; two copies would drift.
pub(crate) fn replace_scoped_config(config: &GraphConfig) -> GraphConfig {
    let mut scoped = config.clone();
    scoped.process_groups = IndexMap::new();
    scoped.process_group_order = Vec::new();
    scoped.level_assignments = None;
    scoped
}

// ===========================================================================
// The LITERAL multi-process default at `graph run`.
// ===========================================================================

/// The `graph run` consent seam for the auto-partition pre-flight — the
/// binary's `--auto-partition`/`--yes` flags plus the REAL TTY probe and the
/// y/N prompt provider. Threaded (rather than read inside the engine) so every
/// consent arm is testable without a terminal. Internal callers that must
/// never auto-partition (e.g. `node run`'s temp graph) pass `None` to
/// [`graph_run`](crate::graph_cmd::graph_run) instead of a seam.
pub struct PartitionConsent<'a> {
    /// `--auto-partition`: re-derive even when the graph already declares
    /// `process_groups:` (rejected alongside `--single-process` — see
    /// [`resolve_partition_intent`]).
    pub auto_partition: bool,
    /// `--yes`: persist the derived partition without the interactive confirm.
    pub assume_yes: bool,
    /// The binary passes `std::io::stdin().is_terminal()`.
    pub is_tty: bool,
    /// Displays the preview and asks y/N; only invoked on the interactive arm.
    pub confirm: &'a mut dyn FnMut(&str) -> CliResult<bool>,
}

/// What the `graph run` pre-flight decided to do about partition derivation —
/// the PURE decision (see [`resolve_partition_intent`]), oracle-testable arm
/// by arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionIntent {
    /// Run the file as declared — no derivation. Covers: hand-written
    /// `process_groups:` without `--auto-partition`, `--single-process`,
    /// non-Unix hosts, and the non-Real time sources on unpartitioned graphs
    /// (the mp default applies to the REAL-clock live path only).
    RespectFile {
        /// `Some(reason)` when an explicit `--auto-partition` is being
        /// IGNORED (non-Unix host) — the dispatch site warns loudly.
        auto_partition_ignored: Option<&'static str>,
    },
    /// Derive groups + run the consent ladder
    /// ([`run_auto_partition_preflight`]).
    Derive {
        /// The graph already declares `process_groups:` (`--auto-partition`
        /// re-derive): a TTY decline KEEPS the existing groups instead of
        /// running the derivation in-memory.
        re_derive: bool,
    },
}

/// The `--single-process` × `--auto-partition` rejection text (contradictory
/// intent; also enforced at clap parse via `conflicts_with` — defense in
/// depth).
pub const SINGLE_PROCESS_AUTO_PARTITION_REJECTION: &str =
    "--single-process and --auto-partition contradict each other: one forces the monolith, the \
     other derives a multi-process partition. Drop one of the two.";

/// The non-Unix `--auto-partition` ignore notice (deriving groups that cannot
/// run multi-process on this host would be noise).
pub const NON_UNIX_AUTO_PARTITION_IGNORED: &str =
    "--auto-partition is ignored: this host is not a Unix, so a multi-process deployment cannot \
     run here (the cross-process barrier is POSIX SHM) — running the single-process monolith";

/// The loud-inference rule: `--yes` passed on a run where NOTHING is
/// being derived is INERT — say so instead of silently ignoring it. Emitted by
/// `graph_run` when `assume_yes` is set but the partition intent resolved to
/// `RespectFile` (a hand-written `process_groups:` without `--auto-partition`,
/// `--single-process`, a non-Real `--time-source`, or a non-Unix host).
pub const YES_INERT_NOTICE: &str =
    "--yes is inert on this run: no partition is being derived (the graph either already \
     declares process_groups:, or --single-process / a non-Real --time-source / a non-Unix host \
     routed this run to its declared shape). Pass --auto-partition to re-derive the partition, \
     or drop --yes.";

/// The PURE partition-intent decision for `graph run` — does
/// this run derive a partition (and with which decline semantics), or respect
/// the file as-is? Pure (no I/O, no `cfg!`) so every arm is oracle-testable on
/// any machine; the caller supplies `is_unix = cfg!(unix)`.
///
/// The matrix (first match wins):
///
/// 1. `--single-process` + `--auto-partition` → `Err` (contradictory intent).
/// 2. `--single-process` → respect (the monolith opt-out ALSO skips the
///    auto-derive entirely).
/// 3. non-Unix → respect (mp cannot run; an explicit `--auto-partition` is
///    ignored LOUDLY via the carried reason).
/// 4. `--auto-partition` → derive (`re_derive` = the graph already has
///    groups). Applies under any time source — parity with a hand-written
///    block, whose supervisor path warns on a non-Real clock and rejects
///    External itself.
/// 5. hand-written `process_groups:` → respect (the block runs as written).
/// 6. unpartitioned + `TimeSource::Real` → derive — THE LITERAL MULTI-PROCESS
///    DEFAULT.
/// 7. unpartitioned + virtual/external → respect (the monolith poll/live
///    paths).
pub fn resolve_partition_intent(
    has_groups: bool,
    single_process: bool,
    auto_partition: bool,
    is_unix: bool,
    time_source: TimeSource,
) -> Result<PartitionIntent, &'static str> {
    if single_process && auto_partition {
        return Err(SINGLE_PROCESS_AUTO_PARTITION_REJECTION);
    }
    if single_process {
        return Ok(PartitionIntent::RespectFile {
            auto_partition_ignored: None,
        });
    }
    if !is_unix {
        return Ok(PartitionIntent::RespectFile {
            auto_partition_ignored: auto_partition.then_some(NON_UNIX_AUTO_PARTITION_IGNORED),
        });
    }
    if auto_partition {
        return Ok(PartitionIntent::Derive {
            re_derive: has_groups,
        });
    }
    if has_groups {
        return Ok(PartitionIntent::RespectFile {
            auto_partition_ignored: None,
        });
    }
    if time_source == TimeSource::Real {
        Ok(PartitionIntent::Derive { re_derive: false })
    } else {
        Ok(PartitionIntent::RespectFile {
            auto_partition_ignored: None,
        })
    }
}

/// What [`run_auto_partition_preflight`] did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunPartitionOutcome {
    /// The derived partition was WRITTEN to the graph file (surgical splice,
    /// atomic write, `.bak`); the run uses it.
    Persisted {
        /// The `.bak` of the pre-write file.
        backup: Option<PathBuf>,
    },
    /// The run uses the derived groups IN-MEMORY; the graph file is untouched
    /// (the no-TTY floor, or a TTY decline on an unpartitioned graph).
    InMemory,
    /// `--auto-partition` TTY decline: the run keeps the file's EXISTING
    /// `process_groups:`; the derivation is discarded.
    KeptExisting,
    /// The file already carries exactly the derived partition — nothing to
    /// write (only reachable on `--auto-partition` re-derive).
    AlreadyCurrent,
}

/// The result of the `graph run` auto-partition pre-flight.
#[derive(Debug, Clone)]
pub struct RunPartitionPreflight {
    /// The config the run proceeds with (derived groups installed for
    /// `Persisted`/`InMemory`; the original for
    /// `KeptExisting`/`AlreadyCurrent`).
    pub config: GraphConfig,
    /// What happened.
    pub outcome: RunPartitionOutcome,
    /// The derivation preview (mode + bands + diff), without consent hints.
    pub preview: String,
}

/// The `graph run` pre-flight knobs (bundled — four adjacent `bool`s would be
/// a silent-swap hazard as positional args).
#[derive(Debug, Clone, Copy)]
pub struct PreflightOptions {
    /// The graph already declares `process_groups:` (`--auto-partition`
    /// re-derive): a TTY decline KEEPS the existing groups.
    pub re_derive: bool,
    /// `true` on the zero-flag literal-default path only — a
    /// present-but-unusable DEFAULT cost artifact degrades to the baseline
    /// with a loud warn instead of aborting the run. `false` under
    /// `--auto-partition` (the user asked for cost-aware behavior — hard
    /// `Err`). `graph_run` passes `!consent.auto_partition`.
    pub lenient_costs: bool,
    /// `--yes`: persist without the interactive confirm.
    pub assume_yes: bool,
    /// The binary passes `std::io::stdin().is_terminal()`.
    pub is_tty: bool,
}

/// The `graph run` auto-partition PRE-FLIGHT — derive the
/// partition (cost artifact at the default path → fused; absent → the
/// process-per-node baseline) and run the consent ladder for persisting
/// it. Returns the [`GraphConfig`] the run proceeds with; the deployment
/// dispatch downstream consumes THAT config, so an in-memory derivation
/// produces EXACTLY the deployment a written file would (same
/// `plan_deployment` inputs — pinned by the equality test).
///
/// The ladder (`re_derive` = the graph already declared groups,
/// `--auto-partition`):
///
/// 1. Derivation byte-identical to the file → [`RunPartitionOutcome::AlreadyCurrent`]
///    (no write, loud info).
/// 2. `assume_yes` → write (+`.bak`) and run the derived groups.
/// 3. No TTY → NEVER mutate: run the derived groups IN-MEMORY with the loud
///    floor notice naming `--yes` (persist) and `--single-process` (monolith).
/// 4. TTY: `confirm(preview + hint)` — `y` writes and runs; `N` runs the
///    derived groups IN-MEMORY (file untouched, logged at `info`: the person
///    was asked and chose it, unlike rung 3) on an unpartitioned graph, or
///    KEEPS the existing groups on a `re_derive` (decline means "don't change
///    what I wrote").
///
/// `config` must be prefix-resolved and (replace-scope) validated by the
/// caller. Pure-testable: no transport, no TTY (the seam is threaded).
pub fn run_auto_partition_preflight(
    workspace_root: &Path,
    graph_name: &str,
    config: GraphConfig,
    raw: &str,
    opts: &PreflightOptions,
    confirm: &mut dyn FnMut(&str) -> CliResult<bool>,
) -> CliResult<RunPartitionPreflight> {
    let &PreflightOptions {
        re_derive,
        lenient_costs,
        assume_yes,
        is_tty,
    } = opts;
    let graph_path = workspace_root
        .join("graphs")
        .join(format!("{graph_name}.yaml"));
    if !graph_path.exists() {
        return Err(CliError::GraphNotFound {
            name: graph_name.to_string(),
        });
    }
    // `raw` is the CALLER's bytes — the same read `config` was parsed from.
    // Re-reading here would make this the two-read shape
    // `graph_read_raw`'s own doc forbids: the groups derived from
    // config@read-1 while the write holds a precondition captured at read-2, so
    // a save landing in the gap between them — cdylib staleness walks plus a
    // syn-parsing validation report, tens to hundreds of ms — satisfies the
    // precondition and is silently replaced by groups derived from the
    // pre-save revision. `graph partition` (the verb) parses its
    // config from its own raw; this is the same shape.

    // The run path always uses the DEFAULT artifact location and NO budget
    // override — the tuning knobs (`--costs`, `--budget-ns`) live on the
    // `graph partition` verb; the run default stays zero-configuration.
    // `budget_ns: None` routes through derive_partition's SINGLE
    // resolution point, so the run default picks up the artifact's FROZEN
    // core-count budget exactly like the verb does (a hardcoded u64::MAX
    // here would silently diverge the two surfaces back to unbounded).
    // `lenient_default`: only the zero-flag default path
    // degrades a broken sidecar to the baseline (loudly).
    let derived = derive_partition(
        workspace_root,
        graph_name,
        &config,
        raw,
        &graph_path,
        &CostsRequest {
            explicit_path: None,
            budget_ns: None,
            lenient_default: lenient_costs,
        },
        // The run preflight never refines: an
        // in-memory refined-levels override would band groups over levels
        // the yaml does not carry (the runtime + a bag replay of that file
        // would run Kahn while the bands assumed refined levels — a
        // band/level mismatch breaking Replay=Live derivability). The
        // preflight bands over the file's OWN levels (Kahn, or a previously
        // PERSISTED `level_assignments:` block, which it preserves
        // byte-identically); refinement is opt-in via `graph partition`.
        false,
    )?;

    // The config the run proceeds with when the derivation is adopted
    // (persisted OR in-memory): derived groups installed, any explicit order
    // list cleared (the listing order IS the rank order).
    let adopt = |groups: &IndexMap<String, Vec<String>>| -> GraphConfig {
        let mut adopted = config.clone();
        adopted.process_groups = groups.clone();
        adopted.process_group_order = Vec::new();
        adopted
    };

    if derived.unchanged {
        // Only reachable on re-derive: the file already carries exactly this
        // derivation, so there is nothing to persist and nothing to ask.
        tracing::info!(
            graph = %config.identity(),
            graph_file = %graph_path.display(),
            "auto-partition: the graph file already carries exactly the derived partition — \
             nothing to rewrite"
        );
        return Ok(RunPartitionPreflight {
            config,
            outcome: RunPartitionOutcome::AlreadyCurrent,
            preview: derived.preview,
        });
    }

    if assume_yes {
        let backup = write_graph_locked(
            workspace_root,
            &graph_path,
            &derived.new_yaml,
            "graph file",
            ExpectedPrior::Contents(raw),
        )?;
        if derived.removed_order {
            warn_order_block_removed(&graph_path);
        }
        warn_creditable_split_overwritten(&graph_path, &derived.overwritten_creditable);
        tracing::info!(
            graph = %config.identity(),
            graph_file = %graph_path.display(),
            "auto-partition: partition written to the graph file (--yes); running multi-process"
        );
        let adopted = adopt(&derived.groups);
        return Ok(RunPartitionPreflight {
            config: adopted,
            outcome: RunPartitionOutcome::Persisted { backup },
            preview: derived.preview,
        });
    }

    if !is_tty {
        // The decided floor: never mutate the file without a TTY confirm or an
        // explicit --yes. The run still goes multi-process with the derived
        // groups held in-memory.
        tracing::warn!(
            graph = %config.identity(),
            graph_file = %graph_path.display(),
            "auto-partition: running MULTI-PROCESS with the derived process groups IN-MEMORY — \
             the graph file is untouched (no TTY to confirm). Persist the partition with --yes \
             (or `cerulion graph partition`), or force the single-process monolith with \
             --single-process"
        );
        let adopted = adopt(&derived.groups);
        return Ok(RunPartitionPreflight {
            config: adopted,
            outcome: RunPartitionOutcome::InMemory,
            preview: derived.preview,
        });
    }

    // Interactive: the confirm provider displays the preview + the
    // decline-semantics hint and asks y/N.
    let hint = if re_derive {
        "(y = apply the re-derived partition to the graph file; N = keep the existing \
         process_groups — file untouched)\n"
    } else {
        "(y = write this partition to the graph file and run; N = run with these groups \
         IN-MEMORY — file untouched)\n"
    };
    let interactive_preview = format!("{}{hint}", derived.preview);
    if confirm(&interactive_preview)? {
        let backup = write_graph_locked(
            workspace_root,
            &graph_path,
            &derived.new_yaml,
            "graph file",
            ExpectedPrior::Contents(raw),
        )?;
        if derived.removed_order {
            warn_order_block_removed(&graph_path);
        }
        warn_creditable_split_overwritten(&graph_path, &derived.overwritten_creditable);
        tracing::info!(
            graph = %config.identity(),
            graph_file = %graph_path.display(),
            "auto-partition: partition written to the graph file (confirmed); running \
             multi-process"
        );
        let adopted = adopt(&derived.groups);
        Ok(RunPartitionPreflight {
            config: adopted,
            outcome: RunPartitionOutcome::Persisted { backup },
            preview: derived.preview,
        })
    } else if re_derive {
        tracing::info!(
            graph = %config.identity(),
            "auto-partition: keeping the existing process_groups from the graph file \
             (re-derived partition discarded)"
        );
        Ok(RunPartitionPreflight {
            config,
            outcome: RunPartitionOutcome::KeptExisting,
            preview: derived.preview,
        })
    } else {
        // INFO, unlike the no-TTY floor above: here a person was shown the
        // preview and answered no, which is the documented default, so the
        // run is doing what they chose. The floor adopts a layout nobody was
        // asked about, and stays loud.
        tracing::info!(
            graph = %config.identity(),
            graph_file = %graph_path.display(),
            "auto-partition declined: running MULTI-PROCESS with the derived process groups \
             IN-MEMORY; the graph file is untouched. Use --single-process for the monolith, or \
             --yes / `cerulion graph partition` to persist"
        );
        let adopted = adopt(&derived.groups);
        Ok(RunPartitionPreflight {
            config: adopted,
            outcome: RunPartitionOutcome::InMemory,
            preview: derived.preview,
        })
    }
}

/// Resolve which cost mode the verb runs in (see [`graph_partition`] "Modes").
/// Loud on every non-happy path: explicit-but-missing is an `Err`, malformed
/// is an `Err` (via [`read_costs_artifact`]), a FOREIGN artifact is an `Err`
/// (via [`check_costs_provenance`]), and the baseline fallback names the
/// missing default + the remedy at `info` level.
fn resolve_costs(
    workspace_root: &Path,
    graph_name: &str,
    explicit: Option<&Path>,
    lenient_default: bool,
) -> CliResult<(Option<ProfileArtifact>, PartitionMode)> {
    match explicit {
        Some(path) => {
            if !path.exists() {
                return Err(CliError::Validation(format!(
                    "cost artifact '{}' (from --costs) does not exist; name an existing \
                     artifact, or omit --costs to use the default \
                     graphs/{graph_name}.costs.yaml (an absent DEFAULT falls back to the \
                     process-per-node baseline — an explicitly named file never does)",
                    path.display()
                )));
            }
            // The provenance gate. An explicitly named file NEVER
            // falls back (the same doctrine as the missing-file arm above), so
            // a wrong-graph artifact is a hard `Err` — the user pointed at a
            // specific file and fusing from it would derive process groups from
            // ANOTHER graph's measured costs. Checked BEFORE the info! below so
            // a refused run never announces that it is fusing.
            let artifact = check_costs_provenance(path, graph_name, read_costs_artifact(path)?)?;
            tracing::info!(
                graph = %graph_name,
                artifact = %path.display(),
                "graph partition: cost-aware mode — fusing from the --costs artifact"
            );
            Ok((
                Some(artifact),
                PartitionMode::Fused {
                    artifact_path: path.to_path_buf(),
                },
            ))
        }
        None => {
            let path = default_artifact_path(workspace_root, graph_name);
            if path.exists() {
                // Validate END-TO-END here (parse + the provenance
                // gate + the to_costs hand-edit checks incl. the version gate)
                // so the lenient arm catches EVERY unusable-artifact class, not
                // just parse failures. (derive_emit_groups re-runs to_costs —
                // pure and cheap.)
                //
                // Provenance is checked BEFORE `to_costs`, because a
                // foreign artifact's `to_costs` failure would be about THIS
                // graph's node ids missing from it — sending the operator to
                // fix a file that should not be read at all. Routing it through
                // the SAME `loaded` chain is what gives it the posture every
                // other unusable-artifact class already has here: a hard `Err`
                // for the verb and `--auto-partition`, a loud degrade to the
                // baseline on the ZERO-FLAG `graph run` default.
                let loaded = read_costs_artifact(&path)
                    .and_then(|artifact| check_costs_provenance(&path, graph_name, artifact))
                    .and_then(|artifact| artifact.to_costs().map(|_| artifact));
                match loaded {
                    Ok(artifact) => {
                        tracing::info!(
                            graph = %graph_name,
                            artifact = %path.display(),
                            "graph partition: cost-aware mode — fusing from the default cost \
                             artifact"
                        );
                        Ok((
                            Some(artifact),
                            PartitionMode::Fused {
                                artifact_path: path,
                            },
                        ))
                    }
                    Err(e) if lenient_default => {
                        // The zero-flag default path must not
                        // abort a plain run over a sidecar the user never
                        // asked to consume — degrade LOUDLY to the baseline.
                        warn_default_artifact_degraded(graph_name, &path, &e);
                        Ok((
                            None,
                            PartitionMode::BaselineDegraded {
                                artifact_path: path,
                            },
                        ))
                    }
                    Err(e) => Err(e),
                }
            } else {
                tracing::info!(
                    graph = %graph_name,
                    missing_artifact = %path.display(),
                    "graph partition: no cost artifact found — emitting the process-per-node \
                     baseline (maximal fault isolation, EXCEPT that `block` \
                     co-location constraints are still honoured, so a `block` topic's flow \
                     shares one group); run `cerulion graph profile` first for a cost-aware \
                     fused partition"
                );
                Ok((
                    None,
                    PartitionMode::Baseline {
                        missing_artifact_path: path,
                    },
                ))
            }
        }
    }
}

/// Refuse a cost artifact whose `graph:` provenance names ANOTHER
/// graph.
///
/// `cerulion graph profile` stamps the graph it harvested from into
/// [`ProfileArtifact::graph`], and this check is what reads that field. A stale or
/// copied `graphs/<name>.costs.yaml` — the same node ids, another graph's
/// measured p50s and edge rates — fuses a partition from costs that describe
/// something else, and the result is CREDIBLE: real group names, plausible
/// bands, a preview that looks exactly like a good one. That is worse than no
/// snapshot, because the baseline the operator would otherwise get is at least
/// explicit about knowing nothing.
///
/// Keyed on the CLI-resolved `graph_name` — the file stem — which is what
/// [`default_artifact_path`] reads by and what `graph_profile` writes under, so
/// the comparison is against the same key the artifact was named for. (It is
/// NOT the graph's internal `name:` field; the two differ whenever a graph's
/// file name is not its `name:`, and keying on the internal one would compare
/// against a path nothing writes.)
///
/// Returns the artifact unchanged when the provenance agrees, so it composes
/// into [`resolve_costs`]'s existing `Result` chain — which is what gives each
/// arm its own posture (hard `Err` for an explicit `--costs` and for the verb,
/// a loud baseline degrade on the lenient `graph run` default) without this
/// function knowing which caller it is serving.
fn check_costs_provenance(
    path: &Path,
    graph_name: &str,
    artifact: ProfileArtifact,
) -> CliResult<ProfileArtifact> {
    if artifact.graph == graph_name {
        return Ok(artifact);
    }
    Err(CliError::Validation(format!(
        "cost artifact '{}' was harvested from graph '{}', not '{graph_name}' — two graphs \
         can share node ids, so fusing from it would derive process groups from ANOTHER \
         graph's measured costs; run `cerulion graph profile {graph_name}` to harvest this \
         graph's own snapshot (or name an artifact whose `graph:` is '{graph_name}'). A \
         foreign artifact is refused loudly, never fused from silently",
        path.display(),
        artifact.graph,
    )))
}

/// Read + parse a cost artifact that EXISTS. A parse failure is a hard `Err`
/// naming the file and the remedy — a present-but-broken artifact must never
/// silently degrade to the baseline partition (the user thinks they are
/// getting a cost-aware split).
///
/// Parses via [`parse_profile_artifact`] (the VERSION-FIRST
/// probe) so a file written by a NEWER build fails on the actionable
/// version message instead of a raw serde unknown-field error (the
/// `deny_unknown_fields` interaction).
fn read_costs_artifact(path: &Path) -> CliResult<ProfileArtifact> {
    let text = std::fs::read_to_string(path)?;
    parse_profile_artifact(&text).map_err(|e| {
        CliError::Validation(format!(
            "cost artifact '{}' is present but MALFORMED for the profile-artifact schema ({e}); \
             fix the hand edit or re-run `cerulion graph profile` — a present-but-broken \
             artifact is refused loudly, never silently ignored in favor of the baseline",
            path.display()
        ))
    })
}

/// Render the `process_groups:` block in the shape
/// [`GraphConfig::process_groups`] parses — `  <group>: [<node>, …]` per line,
/// flow-style for readability (matching the hand-authored split fixtures).
/// Deterministic: group order = the map's iteration (partitioner) order, node
/// order = each group's member order. Every scalar is rendered so it re-parses
/// to its exact original string (see [`render_scalar`]).
fn render_process_groups_block(groups: &IndexMap<String, Vec<String>>) -> String {
    let mut out = String::from("process_groups:\n");
    for (name, members) in groups {
        out.push_str("  ");
        out.push_str(&render_scalar(name));
        out.push_str(": [");
        for (i, member) in members.iter().enumerate() {
            if i > 0 {
                out.push_str(", ");
            }
            out.push_str(&render_scalar(member));
        }
        out.push_str("]\n");
    }
    out
}

/// Render the `level_assignments:` block — one
/// `  <node>: <level>` line per node, in the map's (graph) order. Keys go
/// through [`render_scalar`] (same quoting discipline as the process-groups
/// renderer); levels are plain integers.
fn render_level_assignments_block(assignments: &IndexMap<String, usize>) -> String {
    let mut out = String::from("level_assignments:\n");
    for (node, level) in assignments {
        out.push_str("  ");
        out.push_str(&render_scalar(node));
        out.push_str(": ");
        out.push_str(&level.to_string());
        out.push('\n');
    }
    out
}

/// The [`PartitionCosts`] → [`RefineInputs`] projection —
/// the adapter (`node_p50_ns` → `node_cost_ns`,
/// `edge_rate_mhz` → `edge_rate_mhz`, dropping the partition-only `hop`
/// field). Pure two-field copy; BTreeMap iteration is sorted so the built
/// IndexMaps are deterministic (they are only ever point-queried anyway).
///
/// Isolated nodes carry NO cost entry (Principle #13 — never fabricated), so
/// `refine_levels` PINS them at their Kahn level: the refinement is exactly
/// as conservative about under-sampled nodes as the partitioner.
///
/// [`PartitionCosts`]: cerulion_core::graph::PartitionCosts
/// [`RefineInputs`]: cerulion_core::graph::RefineInputs
pub fn refine_inputs_from_costs(
    costs: &cerulion_core::graph::PartitionCosts,
) -> cerulion_core::graph::RefineInputs {
    cerulion_core::graph::RefineInputs {
        node_cost_ns: costs
            .node_p50_ns
            .iter()
            .map(|(k, v)| (k.clone(), *v))
            .collect(),
        edge_rate_mhz: costs
            .edge_rate_mhz
            .iter()
            .map(|(k, v)| (k.clone(), *v))
            .collect(),
        // The growth policy is NOT a cost — this pure projection
        // leaves it at the default `LevelGrowth::Allow`; the shape gate in
        // `derive_partition` overrides it per the Kahn pre-band (Deny for a
        // multi-group-destined graph) before refining.
        ..Default::default()
    }
}
