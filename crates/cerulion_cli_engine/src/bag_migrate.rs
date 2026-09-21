// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion bag migrate` — rewrite a bag whose embedded `graph.yaml` carries
//! keys the graph format no longer defines.
//!
//! # The problem this exists for
//!
//! Graph YAML checks every key. A legacy bag
//! can embed the ON-DISK `graphs/<name>.yaml` rather than the EFFECTIVE config the
//! run executed, so a graph file of that era could carry a since-removed
//! setting (the old `policy:` block is the one that really happens),
//! and that key travels into the bag. `cerulion bag play --resim` refuses such
//! a bag at the attachment parse (exit 2), and an MCAP attachment is sealed:
//! there is no edit path. Re-recording works and is usually the better answer,
//! but it is not available for a recording of something that already happened.
//!
//! This verb is the other path: it writes a NEW bag whose `graph.yaml` is the
//! same graph with the undefined keys REMOVED, and copies everything else
//! through unchanged.
//!
//! # What it does and does not touch
//!
//! * The input bag is **never** modified and never written to. A migration
//!   always produces a second file (`<stem>.migrated.mcap` by default), and the
//!   verb refuses outright if the resolved output would be the input.
//! * Every recorded frame — user topics and the reserved `__cerulion/*`
//!   streams alike — is copied with its topic, sequence, log time, publish time
//!   and payload BYTES unchanged, in file order.
//! * Every attachment other than `graph.yaml` is copied byte-for-byte,
//!   including its media type and both timestamps.
//! * `graph.yaml` is replaced by the re-rendered config, and one new
//!   attachment, [`MIGRATION_ATTACHMENT`], records what was removed.
//!
//! What is NOT preserved, and why: MCAP channel ids are assigned by the writer
//! from the sorted topic set, so they are a function of the topic list rather
//! than of the input file, and the header's `library` string names the writer —
//! which for this file really is the migrate verb, not the recorder. Chunk
//! boundaries are the writer's own. None of those are readable data; the
//! per-message and per-attachment contents above are.
//!
//! # The re-render, and what it costs
//!
//! The new document is produced by `graph_cmd::render_effective_graph_yaml` —
//! the same renderer `graph run --record` uses, so a
//! migrated bag has exactly the shape the current recorder writes. The cost is
//! the one that renderer already documents: the document is serde-normalized, so
//! comments are dropped and key order follows the struct. The `graph.yaml`
//! attachment is a machine-read artifact, never the user's file, and the user's
//! file is not touched by anything here.
//!
//! The config is parsed with [`parse_graph_raw`] rather than `parse_graph`
//! deliberately: `parse_graph` fills an absent `prefix:` from the HOST it runs
//! on, and baking the migrating desk's hostname into a robot's recording would
//! silently change which topic names the bag resolves to. A prefix-less
//! embedded graph stays prefix-less, exactly as the original recorder left
//! it.
//!
//! # How the stripped keys are found
//!
//! The oracle is the REAL parser, never a second copy of the graph schema. The
//! loop parses the document with `serde_yaml::from_str::<GraphConfig>`; a
//! `deny_unknown_fields` failure names the offending key, carries serde's own
//! structural PATH (`nodes[0].outputs[0]`), and locates it at a line and
//! column. That key's block is then BLANKED — replaced by empty lines rather
//! than deleted — so every later error's line number still refers to the
//! ORIGINAL document, and the loop runs again. Nothing here knows what the
//! graph format's keys are, so nothing here can drift out of step with it.

use std::path::{Path, PathBuf};

use cerulion_bag::{
    BagAttachment, BagChannel, BagReader, BagWriter, BagWriterConfig, TopicSchema,
    DESCRIPTOR_VERSION, RESERVED_PREFIX, SCHEMA_ENCODING,
};
// The unknown-key classifier is IMPORTED, never
// re-implemented. Three call sites branch on it — this strip loop,
// `replay_cmd`'s refusal and `cerulion_bagd`'s capture judge — and a private
// copy here is exactly how they would come to disagree about whether one bag
// is repairable (the two-copies class).
use cerulion_core::graph::{parse_graph_raw, unknown_field_key, GraphConfig};
use serde::{Deserialize, Serialize};
use sha2::Digest as _;

use crate::error::{CliError, CliResult};

/// The bag's embedded graph document. Spelled here rather than imported
/// because `replay_cmd` and `cerulion_bagd` each keep their own private
/// spelling of the same literal for the same reason — the reader/writer pair
/// has to agree on it, and a shared constant would hide which side changed.
pub const GRAPH_ATTACHMENT: &str = "graph.yaml";

/// The migration provenance attachment this verb adds.
pub const MIGRATION_ATTACHMENT: &str = "__cerulion/migration.json";
/// Its media type.
pub const MIGRATION_MEDIA_TYPE: &str = "application/json";

/// The MCAP Header `library` string a migrated bag carries.
///
/// It names the writer, and the writer of this file really is the migrate verb
/// — not `cerulion_bagd`, whose recording it descends from. Keeping the
/// recorder's string would make a rewritten file claim to be a recording.
pub const MIGRATE_LIBRARY: &str = "cerulion_bag_migrate";

/// The [`MigrationRecord`] format version.
pub const MIGRATION_RECORD_VERSION: u32 = 1;

/// The suffix appended to the input stem when `-o` is not given.
const DEFAULT_OUTPUT_SUFFIX: &str = "migrated.mcap";

/// A guard against a pathological document driving the strip loop forever.
/// Every iteration blanks at least one line, so the document's own line count
/// plus slack is a hard ceiling.
const MAX_STRIP_ITERATIONS_SLACK: usize = 8;

// ===========================================================================
// The timestamp seam
// ===========================================================================

/// The wall time stamped into [`MIGRATION_ATTACHMENT`], supplied by the CALLER.
///
/// A migration is otherwise a pure function of its input: `cerulion_bag`'s
/// writer reads no clock (that is its byte-determinism contract), and every
/// message timestamp is copied from the recording. The migration record's own
/// date is the single value that is not, so it enters through this type rather
/// than through a `SystemTime::now()` buried in the rewrite — the binary calls
/// [`MigrationStamp::now`] at the CLI boundary, and a test that needs two
/// migrations to be byte-identical passes [`MigrationStamp::at_epoch_ns`].
///
/// Nanoseconds since the Unix epoch, matching `__cerulion/recorder.json`'s
/// `recorded_at_ns` rather than inventing a second date spelling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MigrationStamp {
    epoch_ns: u64,
}

impl MigrationStamp {
    /// Read the wall clock. The ONE clock read in the migration path, and it
    /// belongs to the caller — see the type docs.
    pub fn now() -> Self {
        let epoch_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos().min(u64::MAX as u128) as u64)
            // A clock before the epoch is not a reason to refuse a migration;
            // it is a reason to record "unknown" rather than a fabricated date.
            .unwrap_or(0);
        Self { epoch_ns }
    }

    /// A fixed stamp — the injection point for a determinism test.
    pub fn at_epoch_ns(epoch_ns: u64) -> Self {
        Self { epoch_ns }
    }

    /// Nanoseconds since the Unix epoch.
    pub fn epoch_ns(&self) -> u64 {
        self.epoch_ns
    }
}

// ===========================================================================
// Public surface
// ===========================================================================

/// One key removed from the embedded graph document.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StrippedKey {
    /// The key itself, as the graph document spelled it.
    pub key: String,
    /// Its structural path — serde's own (`nodes[0].outputs[0].dpeth`), so
    /// this report and the parse error it came from agree.
    pub path: String,
    /// Its 1-based line in the ORIGINAL embedded `graph.yaml`.
    pub line: usize,
}

/// How `cerulion bag migrate` was invoked.
#[derive(Debug, Clone, Default)]
pub struct MigrateOptions {
    /// `-o` — where to write. Defaults to `<stem>.migrated.mcap` beside the
    /// input.
    pub out: Option<PathBuf>,
    /// Preview only, write nothing. Wins over [`assume_yes`](Self::assume_yes).
    pub dry_run: bool,
    /// `--yes` — the non-interactive consent.
    pub assume_yes: bool,
}

/// What the consent ladder decided.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MigrateOutcome {
    /// `--dry-run`: the preview was produced and nothing was written.
    DryRun,
    /// The migrated bag was written.
    Written,
    /// The interactive prompt was declined; nothing was written.
    Declined,
}

/// The result of a migration attempt.
#[derive(Debug, Clone)]
pub struct MigrateReport {
    /// The bag that was read (never written).
    pub input: PathBuf,
    /// Where the migrated bag goes (or would have gone).
    pub output: PathBuf,
    /// Every key removed from the embedded graph, in the order the parser
    /// found them.
    pub stripped: Vec<StrippedKey>,
    /// The human-readable preview — what the interactive prompt shows, and
    /// what the binary prints on the paths where no prompt ran.
    pub preview: String,
    /// The consent decision.
    pub outcome: MigrateOutcome,
    /// Whether the confirm closure was invoked (and so is responsible for
    /// having displayed [`Self::preview`]). Mirrors `graph partition`.
    pub preview_shown: bool,
}

/// The `__cerulion/migration.json` attachment.
///
/// Deliberately NOT `deny_unknown_fields`: this is a persisted bag attachment
/// in the `record_coverage.json` class, where fields are added additively and
/// an older reader must keep working. The graph-YAML rule is the opposite one
/// because that document is USER-authored, and a key a user typed that nothing
/// reads is a silently-dropped setting.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MigrationRecord {
    /// [`MIGRATION_RECORD_VERSION`].
    pub version: u32,
    /// The verb that produced this file.
    pub tool: String,
    /// Nanoseconds since the Unix epoch — see [`MigrationStamp`].
    pub migrated_at_ns: u64,
    /// The source bag's file NAME (not its path — a path is a fact about the
    /// machine that ran the migration, not about the recording).
    pub source_bag_file_name: String,
    /// The source bag's size in bytes.
    pub source_bag_size_bytes: u64,
    /// SHA-256 of the whole source bag, lowercase hex. This is what makes the
    /// provenance checkable: given the original file, anyone can confirm this
    /// really descends from it.
    pub source_bag_sha256: String,
    /// SHA-256 of the `graph.yaml` attachment that was REPLACED.
    pub source_graph_yaml_sha256: String,
    /// SHA-256 of the `graph.yaml` attachment this bag carries.
    pub migrated_graph_yaml_sha256: String,
    /// The keys that were removed.
    pub stripped: Vec<StrippedKey>,
}

// ===========================================================================
// The verb
// ===========================================================================

/// Rewrite `input`'s embedded `graph.yaml` without the keys the graph format
/// no longer defines, into a NEW bag.
///
/// # Refusals
///
/// Each of these exits non-zero naming what happened; none of them writes
/// anything:
///
/// * the bag's graph ALREADY parses — there is nothing to migrate, and minting
///   a copy anyway would hand the user a second file that is identical in
///   every way that matters;
/// * the bag carries no `graph.yaml` attachment at all;
/// * the bag is not finalized (a torn tail has no summary, so its attachment
///   index cannot even be read);
/// * the graph fails to parse for a reason that is NOT an unknown key — real
///   YAML damage, which this verb cannot repair and must not pretend to;
/// * the resolved output is the input, or already exists;
/// * a recorded channel cannot be reproduced faithfully (see
///   [`user_topic_schemas`]).
///
/// # Consent (the decided floor: never write without it)
///
/// Cribbed from `graph partition`, including the argument for each rung:
/// `dry_run` wins over `assume_yes`; `assume_yes` writes; no TTY and no
/// `assume_yes` is a loud refusal naming both escape hatches; otherwise
/// `confirm(&preview)` decides. `is_tty` is threaded rather than read here so
/// the refusal arm is testable without a terminal.
pub fn bag_migrate(
    input: &Path,
    opts: &MigrateOptions,
    stamp: MigrationStamp,
    is_tty: bool,
    confirm: &mut dyn FnMut(&str) -> CliResult<bool>,
) -> CliResult<MigrateReport> {
    if !input.exists() {
        return Err(CliError::Validation(format!(
            "bag migrate: bag '{}' does not exist",
            input.display()
        )));
    }
    let reader = BagReader::open(input)
        .map_err(|e| CliError::Validation(format!("bag migrate: cannot open bag: {e}")))?;
    #[cfg(test)]
    run_after_open_hook_with(input);

    // A torn bag has no summary, and the attachment index lives in the summary
    // — so `attachments()` would answer "none" rather than "unreadable". Ask
    // the cheap question the frame walk already answers, then pay for the
    // precise classification ONLY to describe the refusal.
    if reader.user_frames().is_err() {
        let how = match reader.completeness() {
            Ok(c) => format!("{c:?}"),
            Err(e) => format!("unreadable ({e})"),
        };
        return Err(CliError::Validation(format!(
            "bag migrate: bag '{}' is not finalized ({how}), so its attachment index cannot be \
             read and there is no `graph.yaml` to rewrite. `cerulion bag info` reports what IS \
             readable in it. There is no repair path for a torn bag — re-record, stopping the \
             recorder with Ctrl-C / SIGTERM rather than killing it.",
            input.display()
        )));
    }

    let attachments = reader.attachments().map_err(|e| {
        CliError::Validation(format!(
            "bag migrate: cannot read the attachments of '{}': {e}",
            input.display()
        ))
    })?;
    let Some(graph_att) = attachments.iter().find(|a| a.name == GRAPH_ATTACHMENT) else {
        return Err(CliError::Validation(format!(
            "bag migrate: bag '{}' carries no `{GRAPH_ATTACHMENT}` attachment, so there is no \
             embedded graph to migrate. This verb rewrites the graph a `cerulion graph run \
             --record` bag embeds; a `cerulion bag record` bag has no graph in it and needs \
             none.",
            input.display()
        )));
    };
    let yaml = std::str::from_utf8(&graph_att.data).map_err(|e| {
        CliError::Validation(format!(
            "bag migrate: the bag's `{GRAPH_ATTACHMENT}` attachment is not valid UTF-8 ({e}), so \
             it cannot be parsed as YAML at all. This is damage the migration cannot repair."
        ))
    })?;

    // THE nothing-to-do gate, decided by literally running the strict parser.
    if parse_graph_raw(yaml).is_ok() {
        return Err(CliError::Validation(format!(
            "bag migrate: the `{GRAPH_ATTACHMENT}` embedded in '{}' already parses — there is \
             nothing to migrate, and this verb will not mint a copy that differs from the \
             original in no way a reader can see. If `bag play --resim` still refuses this bag, \
             the reason is something else (its own message names it); migration only removes \
             keys the graph format no longer defines.",
            input.display()
        )));
    }

    // A bag that ALREADY carries a migration record is refused, loudly.
    //
    // MCAP attachment names are not unique and `BagReader::attachment` answers
    // with the FIRST match, so appending a second record does not replace the
    // first — it HIDES behind it. Every reader of `MIGRATION_ATTACHMENT` would
    // then get the OLDER migration's provenance (its source hashes, its stripped
    // key list, its timestamp) for a bag those numbers do not describe, which is
    // exactly the claim this attachment exists to make true.
    //
    // Placed AFTER the already-parses gate on purpose: re-running the verb on a
    // clean migrated bag is the ordinary mistake and gets the specific
    // "nothing to migrate" answer above. This arm is the other reachable state —
    // a migrated bag whose graph STILL does not parse (migrated by an older
    // build, or hand-assembled) — where proceeding is what would corrupt the
    // provenance.
    //
    // There is deliberately no "replace it" mode. The record names the SOURCE
    // bag by name, size and SHA-256; a second migration's source is this
    // intermediate file, so a replaced record would describe a chain by its last
    // link only, and a chain is what a reader needs. Migrating the ORIGINAL
    // again produces a bag with one accurate record.
    if let Some(existing) = attachments.iter().find(|a| a.name == MIGRATION_ATTACHMENT) {
        return Err(CliError::Validation(format!(
            "bag migrate: bag '{}' already carries a `{MIGRATION_ATTACHMENT}` attachment{} — it \
             is itself the product of a migration, and this verb will not append a second \
             provenance record beside the first (a reader asking for the record gets whichever \
             comes first, so the bag would describe itself with the wrong one). Its graph still \
             does not parse, which means the earlier migration did not finish the job: migrate \
             the ORIGINAL recording again with this build, and you get one bag with one accurate \
             record. Nothing was written.",
            input.display(),
            describe_existing_migration(&existing.data),
        )));
    }

    let strip = strip_unknown_graph_keys(yaml)?;
    let migrated_yaml = crate::graph_cmd::render_effective_graph_yaml(&strip.config)?;

    let output = resolve_output_path(input, opts.out.as_deref())?;
    let preview = render_preview(input, &output, &strip.stripped);

    let mut preview_shown = false;
    let outcome = if opts.dry_run {
        MigrateOutcome::DryRun
    } else if opts.assume_yes {
        write_migrated_bag(
            &reader,
            input,
            &output,
            &attachments,
            &migrated_yaml,
            &strip,
            stamp,
        )?;
        MigrateOutcome::Written
    } else if !is_tty {
        return Err(CliError::Validation(format!(
            "bag migrate would write '{}' but stdin is not a TTY and --yes was not passed — \
             nothing was written. Re-run with --yes to migrate non-interactively (scripts/CI), \
             or --dry-run to see exactly which keys it would remove.",
            output.display()
        )));
    } else {
        preview_shown = true;
        if confirm(&preview)? {
            write_migrated_bag(
                &reader,
                input,
                &output,
                &attachments,
                &migrated_yaml,
                &strip,
                stamp,
            )?;
            MigrateOutcome::Written
        } else {
            MigrateOutcome::Declined
        }
    };

    Ok(MigrateReport {
        input: input.to_path_buf(),
        output,
        stripped: strip.stripped,
        preview,
        outcome,
        preview_shown,
    })
}

/// `<stem>.migrated.mcap` beside the input, unless `-o` names somewhere else.
///
/// Refuses an output that IS the input (the never-in-place rule, checked on
/// canonicalized paths so `./x.mcap` and `x.mcap` cannot slip past it) and one
/// that already exists (clobbering a file the user did not name is worse than
/// making them pick another name).
fn resolve_output_path(input: &Path, out: Option<&Path>) -> CliResult<PathBuf> {
    let output = match out {
        Some(p) => p.to_path_buf(),
        None => {
            let stem = input
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_else(|| "bag".to_string());
            input.with_file_name(format!("{stem}.{DEFAULT_OUTPUT_SUFFIX}"))
        }
    };
    if same_file(input, &output) {
        return Err(CliError::Validation(format!(
            "bag migrate: the output would be the input ('{}'). A migration NEVER rewrites a bag \
             in place — the original recording is the evidence, and a half-written rewrite of it \
             is unrecoverable. Pass -o with a different path.",
            output.display()
        )));
    }
    if output.exists() {
        return Err(CliError::Validation(format!(
            "bag migrate: '{}' already exists — refusing to overwrite it. Pass -o with a path \
             that does not exist, or move the existing file aside.",
            output.display()
        )));
    }
    Ok(output)
}

/// Whether two paths name the same file. Canonicalization is the reliable test
/// but only works on a path that EXISTS, so a not-yet-created output falls back
/// to comparing the canonical parent directory plus the file name.
fn same_file(a: &Path, b: &Path) -> bool {
    if let (Ok(ca), Ok(cb)) = (a.canonicalize(), b.canonicalize()) {
        return ca == cb;
    }
    let dir_of = |p: &Path| {
        p.parent()
            .filter(|d| !d.as_os_str().is_empty())
            .unwrap_or(Path::new("."))
            .canonicalize()
            .ok()
    };
    match (dir_of(a), dir_of(b)) {
        (Some(da), Some(db)) => da == db && a.file_name() == b.file_name(),
        _ => a == b,
    }
}

/// What the user is asked to consent to.
fn render_preview(input: &Path, output: &Path, stripped: &[StrippedKey]) -> String {
    let mut out = String::new();
    out.push_str("cerulion bag migrate\n");
    out.push_str(&format!("  bag: {}\n", input.display()));
    out.push_str(&format!("  out: {}\n", output.display()));
    out.push_str(&format!(
        "\nThe embedded `{GRAPH_ATTACHMENT}` carries {} key(s) the graph format no longer \
         defines.\nThey are REMOVED from the migrated copy:\n\n",
        stripped.len()
    ));
    for s in stripped {
        out.push_str(&format!("  line {:<5} {}\n", s.line, s.path));
    }
    out.push_str(
        "\nEverything else is copied through unchanged: every recorded frame, the scheduler \
         trace,\nand every other attachment, each with its own timestamps and bytes. The input \
         bag is\nnever modified.\n\nThe migrated graph is re-rendered from the parsed config, so \
         YAML comments and key order\ndo not survive it — the same normalization `cerulion graph \
         run --record` applies to its embedded effective configuration. A `",
    );
    out.push_str(MIGRATION_ATTACHMENT);
    out.push_str("` attachment records what was removed.\n");
    out
}

// ===========================================================================
// The strip engine (pure — no bag, no filesystem)
// ===========================================================================

/// The outcome of stripping a graph document.
#[derive(Debug, Clone)]
pub struct GraphStrip {
    /// The document with each offending key's block blanked out. Line numbers
    /// are unchanged from the input, which is what lets every iteration's
    /// reported line refer to the ORIGINAL document.
    pub cleaned_yaml: String,
    /// The config the cleaned document parses to.
    pub config: GraphConfig,
    /// What was removed, in the order the parser found it.
    pub stripped: Vec<StrippedKey>,
}

/// Remove every key the graph format does not define from `yaml`.
///
/// Fails — never guesses — when the document's parse failure is not an unknown
/// key, when the parser gives no location for one, or when blanking a located
/// key changes nothing (which would loop forever).
pub fn strip_unknown_graph_keys(yaml: &str) -> CliResult<GraphStrip> {
    // The parser and this rewrite must agree on what a LINE is, or a reported
    // line number names different text on each side. libyaml breaks a line on
    // `\r`, NEL (U+0085), LS (U+2028) and PS (U+2029) as well as `\n`;
    // `str::lines` splits on `\n` alone. Measured, the mismatch DELETED a kept
    // field and shipped: in `legacy_junk: 1<NEL>name: keepme` the parser sees
    // two lines and reports the junk key on the first, while this rewrite sees
    // ONE line and blanks all of it — taking `name: keepme` with it, re-parsing
    // clean, and reporting only the junk key.
    //
    // Normalising is a real fix rather than a refusal because the blanked text
    // is never the OUTPUT: it is parser input only, and the migrated document
    // is re-rendered from the parsed config. Every character replaced here is
    // one libyaml already treats as a break, in scalars too (it normalises them
    // to `\n` in scalar values itself), so the parse is unchanged and the two
    // line models now agree by construction.
    let normalized = normalize_yaml_line_breaks(yaml);
    let yaml: &str = &normalized;
    let mut lines: Vec<String> = yaml.lines().map(|l| l.to_string()).collect();
    // `str::lines` drops the information about a trailing newline; the parser
    // does not care, and the rendered output replaces this document anyway.
    let mut stripped: Vec<StrippedKey> = Vec::new();
    let ceiling = lines.len() + MAX_STRIP_ITERATIONS_SLACK;

    for _ in 0..ceiling {
        let work = lines.join("\n");
        let err = match serde_yaml::from_str::<GraphConfig>(&work) {
            Ok(_) => {
                // Re-parse through the PRODUCTION entry point so the config
                // this returns is the one the graph loader would build (it
                // seeds `identity` from a legacy `name:`), not a second
                // deserialization path's idea of it.
                let config = parse_graph_raw(&work)?;
                return Ok(GraphStrip {
                    cleaned_yaml: work,
                    config,
                    stripped,
                });
            }
            Err(e) => e,
        };
        let text = err.to_string();
        let Some(key) = unknown_field_key(&text) else {
            return Err(CliError::Validation(format!(
                "bag migrate: the bag's `{GRAPH_ATTACHMENT}` does not parse, and the reason is \
                 NOT a key the format no longer defines: {text}. Migration only removes undefined \
                 keys — this is real YAML damage (or a graph the format never accepted), and \
                 rewriting it would be guessing at what the recording meant."
            )));
        };
        let path = unknown_field_path(&text, &key);
        let Some(loc) = err.location() else {
            return Err(CliError::Validation(format!(
                "bag migrate: the bag's `{GRAPH_ATTACHMENT}` carries the undefined key `{key}`, \
                 but the parser reported no line for it, so the migration cannot tell which \
                 occurrence to remove. Refusing rather than removing the wrong one."
            )));
        };
        let (line, column) = (loc.line(), loc.column());
        if line == 0 || line > lines.len() || column == 0 {
            return Err(CliError::Validation(format!(
                "bag migrate: the parser located the undefined key `{key}` at line {line} column \
                 {column}, which is not a position in the bag's `{GRAPH_ATTACHMENT}` \
                 ({} lines). Refusing rather than editing blindly.",
                lines.len()
            )));
        }
        // The parser's column is a 1-based CHARACTER index (serde_yaml adds one
        // to libyaml's mark, and libyaml advances that mark by one per
        // CHARACTER while advancing its buffer by the codepoint's WIDTH); every
        // slice below is BYTE-indexed. Block keys were safe by accident — only
        // ASCII indentation precedes one — but on a FLOW line any earlier
        // multi-byte character shifted the landing point left, and the verb
        // refused a repairable graph blaming a "multi-line quoted scalar, or a
        // block scalar" for a document whose only property was a non-ASCII
        // string. Converted once, here, so nothing downstream has to know.
        let col = char_col_to_byte(&lines[line - 1], column - 1);
        // BLOCK or FLOW is decided by the document's own bracket depth at the
        // key, never by how the line LOOKS: a key inside a multi-line flow
        // mapping sits behind nothing but spaces, so a prefix test reads it as
        // a block key and blanking its line would tear the collection open.
        let removed = if flow_depth_at(&lines, line - 1, col) == 0 {
            blank_key_block(&mut lines, line - 1, col)
        } else {
            blank_flow_entry(&mut lines, line - 1, col)
        };
        if !removed {
            return Err(CliError::Validation(format!(
                "bag migrate: the undefined key `{key}` is reported at line {line} column \
                 {column} of the bag's `{GRAPH_ATTACHMENT}`, but the migration could not \
                 isolate it there — the document may use a YAML shape this rewrite does not \
                 model (a multi-line quoted scalar, or a block scalar). Refusing rather than \
                 editing blindly."
            )));
        }
        stripped.push(StrippedKey { key, path, line });
    }

    Err(CliError::Validation(format!(
        "bag migrate: the bag's `{GRAPH_ATTACHMENT}` still does not parse after {ceiling} \
         removals. Refusing rather than looping — please report this bag upstream."
    )))
}

/// serde's own structural path for the offending key.
///
/// `serde_yaml` prefixes the message with the path of the CONTAINER
/// (`nodes[0].outputs[0]: unknown field …`) and omits it entirely at the top
/// level, so the full path is that prefix plus the key. Reporting serde's
/// spelling rather than inventing one keeps this listing and the parse error it
/// came from talking about the same place.
fn unknown_field_path(err: &str, key: &str) -> String {
    let Some(idx) = err.find("unknown field `") else {
        return key.to_string();
    };
    let container = err[..idx].trim_end().trim_end_matches(':').trim();
    if container.is_empty() {
        key.to_string()
    } else {
        format!("{container}.{key}")
    }
}

/// Blank the key at `lines[idx]`, column `col` (both 0-based), together with
/// the block it owns.
///
/// BLANKING rather than deleting is the whole trick: line numbers stay put, so
/// the next parse error still locates itself in the ORIGINAL document and every
/// reported line means what the user's editor will show.
///
/// A line that STARTS a sequence entry (`  - policy:`) keeps its `-`: the entry
/// itself is not what is being removed, only its key, and the entry's remaining
/// keys sit at a deeper indentation where YAML still reads them as the entry's
/// mapping.
///
/// Returns `false` when `lines[idx]` does not carry a key at `col` — the caller
/// treats that as "refuse", never as "nothing to do".
fn blank_key_block(lines: &mut [String], idx: usize, col: usize) -> bool {
    let line = &lines[idx];
    if col >= line.len() {
        return false;
    }
    // The reported column must be the start of a key token, i.e. the byte at
    // `col` is not whitespace and there is a `:` after it on this line.
    if !line.is_char_boundary(col) {
        return false;
    }
    let rest = &line[col..];
    if rest.starts_with(char::is_whitespace) || !rest.contains(':') {
        return false;
    }
    let prefix = &line[..col];
    if !prefix.chars().all(|c| c == ' ' || c == '-' || c == '\t') {
        return false;
    }
    // Keep a sequence-entry dash; drop everything from the key onward.
    let kept = if prefix.trim_end().ends_with('-') {
        prefix.trim_end().to_string()
    } else {
        String::new()
    };
    lines[idx] = kept;

    // The key's value: every following line indented DEEPER than the key
    // belongs to it. The first non-blank line at or above the key's own
    // indentation is the next sibling and ends it — with two exceptions that a
    // plain indentation test gets wrong.
    let mut i = idx + 1;
    while i < lines.len() {
        let l = &lines[i];
        let trimmed = l.trim_start();
        if trimmed.is_empty() {
            i += 1;
            continue;
        }
        // EXCEPTION 1 — a COMMENT never ends a block. YAML allows a comment at
        // ANY indentation inside one, so a hand-commented-out line at column 0
        // (ordinary editing, and the old `policy:` block is exactly
        // where it happens) was read as the next sibling: the sweep stopped and
        // ORPHANED the rest of the removed block. Measured, both outcomes are
        // bad — the orphan usually makes the re-parse fail and the verb refuses
        // blaming "real YAML damage" on a document that was legal, and in the
        // shape where the orphan is a valid key at the right indentation it
        // REATTACHES to the preceding KEPT block and ships: a `connect: ["tcp/…"]`
        // authored inside the removed block gets adopted into a real `network:` block,
        // with the report naming only the removed key.
        // A migration inventing network wiring is the worst thing in this file.
        //
        // Blanked rather than skipped: comments do not survive the re-render
        // either way, so consuming one costs nothing and keeps the sweep's
        // output uniform.
        if trimmed.starts_with('#') {
            lines[i] = String::new();
            i += 1;
            continue;
        }
        if indent_of(l) <= col {
            // EXCEPTION 2 — YAML's sequence-indentation exception. `topics:`
            // followed by `- /a` at the SAME indent is mainstream hand-written
            // style, and those items are the key's own value, not its sibling.
            // Blanking the key line alone orphaned them and the verb refused a
            // repairable bag blaming the document.
            //
            // Safe because a sibling KEY at this indent never starts with a
            // dash, and the PARENT sequence's own entries sit at the dash
            // column, which is strictly SHALLOWER than a key that follows the
            // dash on its line — so `== col` cannot capture them.
            if indent_of(l) == col && (trimmed == "-" || trimmed.starts_with("- ")) {
                lines[i] = String::new();
                i += 1;
                continue;
            }
            break;
        }
        lines[i] = String::new();
        i += 1;
    }
    true
}

/// Every line break libyaml recognises, rewritten to `\n`.
///
/// `\r\n` collapses to one break, as it does in the parser. Returns the input
/// untouched (no allocation) when there is nothing to normalise, which is every
/// document a Cerulion recorder ever wrote.
fn normalize_yaml_line_breaks(yaml: &str) -> std::borrow::Cow<'_, str> {
    const NEL: char = '\u{85}';
    const LS: char = '\u{2028}';
    const PS: char = '\u{2029}';
    if !yaml.contains(['\r', NEL, LS, PS]) {
        return std::borrow::Cow::Borrowed(yaml);
    }
    let mut out = String::with_capacity(yaml.len());
    let mut chars = yaml.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '\r' => {
                // CRLF is ONE break, not two.
                if chars.peek() == Some(&'\n') {
                    chars.next();
                }
                out.push('\n');
            }
            NEL | LS | PS => out.push('\n'),
            other => out.push(other),
        }
    }
    std::borrow::Cow::Owned(out)
}

/// The BYTE offset of the `char_col`-th character of `line`.
///
/// Past the end of the line it answers `line.len()`, which the blankers' own
/// bounds checks then refuse — the same outcome an out-of-range column had
/// before, reached without slicing at a byte that is not a boundary.
fn char_col_to_byte(line: &str, char_col: usize) -> usize {
    line.char_indices()
        .nth(char_col)
        .map_or(line.len(), |(byte, _)| byte)
}

/// Leading-space count. Tabs are not legal YAML indentation, so a tab is
/// counted as one column like any other character — it can only appear inside a
/// document that is already a parse error.
fn indent_of(line: &str) -> usize {
    line.len() - line.trim_start_matches(' ').len()
}

// ── flow collections ──────────────────────────────────────────────────────

/// How many FLOW collections (`[…]` / `{…}`) enclose the position
/// `lines[idx][col]`.
///
/// Zero means the key is an ordinary BLOCK mapping key and owns the rest of its
/// line; anything else means it is one entry of a flow collection and the rest
/// of that line belongs to its siblings. Nothing about how the LINE looks can
/// answer this — `outputs: [\n  {name: o,\n   dpeth: 4}\n]` puts `dpeth` behind
/// nothing but spaces, exactly like a block key — so the whole document is
/// walked to the position rather than the line inspected.
///
/// Quoted scalars, `#` comments and BLOCK SCALARS (`|` / `>`) are skipped so a
/// bracket inside any of them does not move the depth.
///
/// The block-scalar span is the one that is not merely defensive. A `|` or `>`
/// scalar's body is ORDINARY TEXT — a description, a shell snippet, a note —
/// and an unbalanced `[` or `{` anywhere in it left the counter raised for the
/// REST OF THE DOCUMENT, so every later block key read as a flow entry and the
/// migration refused a graph it should fix. Its content is found the way YAML
/// itself finds it: every following line indented deeper than the owning key
/// (blank lines included) belongs to the scalar. Block scalars are illegal
/// INSIDE a flow collection, so a header is only ever looked for at depth 0.
///
/// NOT modelled, and refused rather than mis-read (the caller's own "could not
/// isolate it" arm): a QUOTED scalar spanning LINES — quote state is reset per
/// line, so an unclosed quote cannot swallow the rest of the document.
fn flow_depth_at(lines: &[String], idx: usize, col: usize) -> usize {
    let mut depth = 0usize;
    // `Some(indent)`: every following line indented deeper than `indent` is the
    // body of a block scalar owned by a key at that indentation.
    let mut scalar_body_of: Option<usize> = None;
    for (i, line) in lines.iter().enumerate().take(idx + 1) {
        if let Some(owner_indent) = scalar_body_of {
            if line.trim().is_empty() || indent_of(line) > owner_indent {
                // Still inside the scalar: text, not structure.
                continue;
            }
            scalar_body_of = None;
        }
        let limit = if i == idx { col } else { line.len() };
        depth = scan_line_flow_depth(line, limit, depth);
        // A header only opens a scalar for the lines AFTER it, and only outside
        // a flow collection. `i == idx` is the line the key itself is on, so
        // there is nothing after it left to skip.
        if depth == 0 && i != idx {
            scalar_body_of = block_scalar_header_indent(line);
        }
    }
    depth
}

/// The indentation of the key that owns a BLOCK SCALAR opened on this line, if
/// the line opens one.
///
/// A header is a value position — after a `:` or a sequence `-` — holding
/// nothing but `|` or `>` and its indicators (an explicit indentation digit and
/// at most one chomping `+`/`-`), optionally followed by a comment. Requiring
/// the value POSITION is what keeps `prefix: a|b` (a plain scalar that happens
/// to contain a pipe) and `prefix: "|"` (a quoted one) from reading as headers.
fn block_scalar_header_indent(line: &str) -> Option<usize> {
    let code = strip_trailing_comment(line);
    let trimmed = code.trim_end();
    let token_start = trimmed.rfind(char::is_whitespace).map_or(0, |i| i + 1);
    let token = &trimmed[token_start..];
    if !is_block_scalar_header_token(token) {
        return None;
    }
    // What precedes it must be a value position: `key:`, or nothing but
    // sequence indicators.
    //
    // A sequence test of `before.ends_with('-')` would read the
    // final dash of ANY plain scalar as a sequence marker.
    // `|` and `>` are indicators only at the START of a scalar, so
    // `id: a - |` is a perfectly ordinary plain scalar — read as a
    // header, it opens a span over every following deeper line. Concretely:
    // with an unknown key in a flow collection on such a line, the span hides
    // the flow brackets, the key is classified as a BLOCK key, and the verb
    // REFUSES a graph it exists to fix while blaming the user's document ("the
    // migration could not isolate it there ... a YAML shape this rewrite does
    // not model").
    //
    // The narrowing is to sequence indicators and nothing else — but NOT to a
    // single bare `-`: `- - |` (a block scalar as the first item of a nested
    // sequence) is a genuine value position that a `trim() == "-"` test would
    // start refusing, which would be a fresh false NEGATIVE traded for the
    // false positive. Every whitespace-separated token must be exactly `-`.
    let before = trimmed[..token_start].trim_end();
    if !(before.ends_with(':') || is_sequence_indicator_prefix(before)) {
        return None;
    }
    Some(indent_of(line))
}

/// Is `before` nothing but YAML sequence indicators (`-`, `- -`, …)?
///
/// EMPTY is not: a bare `|` in no value position is the document's own content,
/// not a header opening one.
fn is_sequence_indicator_prefix(before: &str) -> bool {
    let trimmed = before.trim();
    !trimmed.is_empty() && trimmed.split_whitespace().all(|token| token == "-")
}

/// `|`, `>`, `|-`, `>+`, `|2`, `|2-` … and nothing else.
fn is_block_scalar_header_token(token: &str) -> bool {
    let mut chars = token.chars();
    if !matches!(chars.next(), Some('|') | Some('>')) {
        return false;
    }
    let rest: Vec<char> = chars.collect();
    // Indicators only: at most one chomping sign, at most one indentation
    // digit. A longer tail is not a header — it is some other token.
    rest.len() <= 2
        && rest
            .iter()
            .all(|c| c.is_ascii_digit() || *c == '+' || *c == '-')
        && rest.iter().filter(|c| **c == '+' || **c == '-').count() <= 1
}

/// `line` with any `#` comment removed. A `#` opens a comment only at the start
/// of a line or after whitespace, and never inside a quoted scalar.
fn strip_trailing_comment(line: &str) -> &str {
    let bytes = line.as_bytes();
    let mut quote: Option<u8> = None;
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i];
        match quote {
            Some(b'\'') => {
                if c == b'\'' {
                    if bytes.get(i + 1) == Some(&b'\'') {
                        i += 1;
                    } else {
                        quote = None;
                    }
                }
            }
            Some(_) => {
                if c == b'\\' {
                    i += 1;
                } else if c == b'"' {
                    quote = None;
                }
            }
            None => match c {
                b'\'' | b'"' => quote = Some(c),
                b'#' if i == 0 || bytes[i - 1].is_ascii_whitespace() => return &line[..i],
                _ => {}
            },
        }
        i += 1;
    }
    line
}

/// Advance `depth` across `line[..limit]`.
///
/// # Why a bracket is not always a bracket
///
/// In BLOCK context YAML permits `[`, `]`, `{`, `}` and `,` INSIDE a plain
/// scalar — `name: see [ref-123` is the four-word string, unbalanced bracket and
/// all. (libyaml's plain-scalar scanner guards its `,[]{}` break condition on
/// `flow_level != 0`, so those characters only terminate a scalar in FLOW
/// context.) Counting every bracket therefore left the depth stuck at 1 for the
/// REST OF THE DOCUMENT, and the next unknown key — a genuine BLOCK key — was
/// blanked as a flow entry. Measured: that erased a node's
/// whole `outputs:` block, and because `outputs` is `#[serde(default)]` the
/// document re-parsed CLEANLY and the bag SHIPPED with the loss reported as
/// nothing at all. The commoner outcome of the same root cause is a false
/// refusal blaming the user's document.
///
/// So at depth 0 a bracket opens a collection ONLY in a VALUE POSITION: at the
/// start of the line's content, or after a `- ` sequence indicator, or after a
/// `key:` — `at_value` tracks exactly that, surviving spaces and cleared by the
/// first other byte. Once inside a collection (`depth > 0`) every bracket
/// counts again, because there a plain scalar genuinely cannot contain one.
fn scan_line_flow_depth(line: &str, limit: usize, mut depth: usize) -> usize {
    let bytes = line.as_bytes();
    let limit = limit.min(bytes.len());
    let mut quote: Option<u8> = None;
    // True where a VALUE may begin. A bracket here — and only here — opens a
    // flow collection while `depth == 0`.
    let mut at_value = true;
    let mut i = 0;
    while i < limit {
        let c = bytes[i];
        match quote {
            Some(b'\'') => {
                if c == b'\'' {
                    // `''` is an escaped quote inside a single-quoted scalar.
                    if bytes.get(i + 1) == Some(&b'\'') {
                        i += 1;
                    } else {
                        quote = None;
                        at_value = false;
                    }
                }
            }
            Some(_) => {
                if c == b'\\' {
                    i += 1;
                } else if c == b'"' {
                    quote = None;
                    at_value = false;
                }
            }
            None => match c {
                b'\'' | b'"' => quote = Some(c),
                // A `#` opens a comment only at the start of a line or after
                // whitespace; inside a token it is an ordinary character.
                b'#' if i == 0 || bytes[i - 1].is_ascii_whitespace() => break,
                b'[' | b'{' => {
                    // In a plain scalar at depth 0 this is ordinary text.
                    if depth > 0 || at_value {
                        depth += 1;
                    }
                    at_value = false;
                }
                b']' | b'}' => {
                    depth = depth.saturating_sub(1);
                    at_value = false;
                }
                // `key:` opens a value position — but only where a `:` really
                // is a mapping separator: at depth 0 that needs a following
                // space or end of line, since a plain scalar may contain a
                // colon inside a token (`topic: /a:b`).
                b':' if depth > 0 || bytes.get(i + 1).is_none_or(|n| n.is_ascii_whitespace()) => {
                    at_value = true;
                }
                // A block sequence indicator keeps the value position open.
                b'-' if depth == 0 && at_value && bytes.get(i + 1).is_none_or(|n| *n == b' ') => {
                    at_value = true;
                }
                // A flow entry separator opens the next value position.
                b',' if depth > 0 => at_value = true,
                // Spaces do not close a value position; anything else does,
                // because a plain scalar has begun.
                b' ' | b'\t' => {}
                _ => at_value = false,
            },
        }
        i += 1;
    }
    depth
}

/// A position in the line-split document.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Pos {
    line: usize,
    col: usize,
}

/// Remove ONE entry of a flow collection: the key at `lines[idx][col]`, its
/// value, and exactly one of the commas around it.
///
/// The entry's siblings share its line, so this cannot blank lines the way the
/// block path does. It overwrites the entry's own bytes with SPACES instead,
/// which keeps both the line count AND every byte offset on the line — so the
/// parser's next error still locates itself in the ORIGINAL document even when
/// two undefined keys sit inside one flow mapping.
///
/// The comma is what keeps the result parseable: an entry followed by one takes
/// it with it (`{a: 1, b: 2}` minus `a` leaves `{      b: 2}`), and the LAST
/// entry takes the comma BEFORE it instead (`{a: 1, b: 2}` minus `b` leaves
/// `{a: 1       }`), because YAML flow collections do not accept a trailing
/// comma. Removing the only entry leaves an empty collection, which is valid —
/// and if that makes the document fail for a different reason, the caller's
/// not-an-unknown-key arm refuses it rather than guessing further.
///
/// Returns `false` when the position does not look like a key or the entry has
/// no visible end, which the caller treats as "refuse", never "nothing to do".
fn blank_flow_entry(lines: &mut [String], idx: usize, col: usize) -> bool {
    let line = &lines[idx];
    if col >= line.len() || !line.is_char_boundary(col) {
        return false;
    }
    let rest = &line[col..];
    if rest.starts_with(char::is_whitespace) || !rest.contains(':') {
        return false;
    }
    let start = Pos { line: idx, col };
    let Some((end, had_trailing_comma)) = flow_entry_end(lines, start) else {
        return false;
    };
    blank_span(lines, start, end);
    if !had_trailing_comma {
        // The last entry of its collection: the comma that separated it from
        // its predecessor would otherwise be left trailing.
        if let Some(comma) = preceding_comma(lines, start) {
            blank_span(
                lines,
                comma,
                Pos {
                    line: comma.line,
                    col: comma.col + 1,
                },
            );
        }
    }
    true
}

/// Where the flow entry starting at `start` ends, and whether the terminator
/// was its OWN comma (`true`) or the collection's closing bracket (`false`).
///
/// The returned position is EXCLUSIVE and already includes the comma when there
/// is one.
fn flow_entry_end(lines: &[String], start: Pos) -> Option<(Pos, bool)> {
    // Depth RELATIVE to the entry: a `[` or `{` the value opens must not let
    // its own separators end the entry.
    let mut depth = 0i32;
    let mut quote: Option<u8> = None;
    for (li, line) in lines.iter().enumerate().skip(start.line) {
        let bytes = line.as_bytes();
        let mut i = if li == start.line { start.col } else { 0 };
        while i < bytes.len() {
            let c = bytes[i];
            match quote {
                Some(b'\'') => {
                    if c == b'\'' {
                        if bytes.get(i + 1) == Some(&b'\'') {
                            i += 1;
                        } else {
                            quote = None;
                        }
                    }
                }
                Some(_) => {
                    if c == b'\\' {
                        i += 1;
                    } else if c == b'"' {
                        quote = None;
                    }
                }
                None => match c {
                    b'\'' | b'"' => quote = Some(c),
                    b'#' if i == 0 || bytes[i - 1].is_ascii_whitespace() => break,
                    b'[' | b'{' => depth += 1,
                    b']' | b'}' => {
                        if depth == 0 {
                            return Some((Pos { line: li, col: i }, false));
                        }
                        depth -= 1;
                    }
                    b',' if depth == 0 => {
                        return Some((
                            Pos {
                                line: li,
                                col: i + 1,
                            },
                            true,
                        ));
                    }
                    _ => {}
                },
            }
            i += 1;
        }
        quote = None;
    }
    None
}

/// The comma immediately before `start`, skipping whitespace and line breaks.
fn preceding_comma(lines: &[String], start: Pos) -> Option<Pos> {
    let mut line = start.line;
    let mut col = start.col;
    loop {
        while col > 0 {
            col -= 1;
            let c = lines[line].as_bytes()[col];
            if c == b',' {
                return Some(Pos { line, col });
            }
            if !c.is_ascii_whitespace() {
                return None;
            }
        }
        if line == 0 {
            return None;
        }
        line -= 1;
        col = lines[line].len();
    }
}

/// Overwrite `[start, end)` with spaces, one line at a time.
///
/// Byte-for-byte, so every offset after the span on the same line is unchanged
/// — which is what lets a second undefined key in the same flow mapping still
/// be located by the parser's own column.
fn blank_span(lines: &mut [String], start: Pos, end: Pos) {
    for li in start.line..=end.line.min(lines.len().saturating_sub(1)) {
        let len = lines[li].len();
        let from = if li == start.line { start.col } else { 0 };
        let to = if li == end.line {
            end.col.min(len)
        } else {
            len
        };
        if from >= to || !lines[li].is_char_boundary(from) || !lines[li].is_char_boundary(to) {
            continue;
        }
        let blanked = " ".repeat(to - from);
        lines[li].replace_range(from..to, &blanked);
    }
}

// ===========================================================================
// The rewrite
// ===========================================================================

/// The per-topic schema registration for the output bag, taken from the input's
/// channel table.
///
/// REFUSES rather than approximating, on three counts the writer cannot
/// round-trip:
///
/// * a channel with no Cerulion descriptor (a foreign MCAP writer's channel) —
///   there is no schema hash to re-register;
/// * a channel whose descriptor version or hash RECIPE is not this build's —
///   `SchemaDescriptor::new` stamps the current values, so copying such a
///   channel would silently RE-LABEL its hash as current-recipe, and replay's
///   schema-drift preflight, which deliberately skips legacy-recipe channels,
///   would then compare hashes that do not mean the same thing;
/// * a channel whose schema or message ENCODING is not `cerulion` — the writer
///   hardcodes both.
///
/// Every one of these is unreachable for a bag this verb exists to fix (a
/// `graph run --record` bag written by any release that embeds `graph.yaml`),
/// which is exactly why they refuse instead of degrading: if one ever fires, the
/// correct answer is that this migration would have lost something.
pub fn user_topic_schemas(channels: &[BagChannel]) -> CliResult<Vec<TopicSchema>> {
    let mut out = Vec::new();
    for ch in channels {
        if ch.topic.starts_with(RESERVED_PREFIX) {
            // The writer registers every reserved channel itself.
            continue;
        }
        let Some(d) = ch.descriptor else {
            return Err(CliError::Validation(format!(
                "bag migrate: channel '{}' carries no Cerulion schema descriptor (schema \
                 encoding '{}'), so the migrated bag could not re-register it faithfully. \
                 Refusing rather than writing a copy that has lost it.",
                ch.topic, ch.schema_encoding
            )));
        };
        if d.descriptor_version != DESCRIPTOR_VERSION
            || d.hash_recipe != cerulion_core::trace::bag::HASH_RECIPE
        {
            return Err(CliError::Validation(format!(
                "bag migrate: channel '{}' was recorded with schema descriptor version {} / hash \
                 recipe {}, and this build writes version {DESCRIPTOR_VERSION} / recipe {}. A \
                 migrated copy would re-stamp them, silently claiming its hashes were computed \
                 by a recipe that did not compute them. Refusing.",
                ch.topic,
                d.descriptor_version,
                d.hash_recipe,
                cerulion_core::trace::bag::HASH_RECIPE
            )));
        }
        if ch.schema_encoding != SCHEMA_ENCODING || ch.message_encoding != SCHEMA_ENCODING {
            return Err(CliError::Validation(format!(
                "bag migrate: channel '{}' uses schema encoding '{}' / message encoding '{}', and \
                 the writer emits '{SCHEMA_ENCODING}' for both. A migrated copy would change \
                 them. Refusing.",
                ch.topic, ch.schema_encoding, ch.message_encoding
            )));
        }
        out.push(TopicSchema {
            topic: ch.topic.clone(),
            schema_name: ch.schema_name.clone(),
            schema_hash: d.schema_hash,
            wire_fixed_size: d.wire_fixed_size,
        });
    }
    Ok(out)
}

#[allow(clippy::too_many_arguments)]
fn write_migrated_bag(
    reader: &BagReader,
    input: &Path,
    output: &Path,
    attachments: &[BagAttachment],
    migrated_yaml: &str,
    strip: &GraphStrip,
    stamp: MigrationStamp,
) -> CliResult<()> {
    let channels = reader.channels().map_err(|e| {
        CliError::Validation(format!(
            "bag migrate: cannot read the channel table of '{}': {e}",
            input.display()
        ))
    })?;
    let topics = user_topic_schemas(&channels)?;
    let provisioning = channels
        .iter()
        .filter(|c| !c.provisioning.is_empty())
        .map(|c| (c.topic.clone(), c.provisioning))
        .collect();

    let record = MigrationRecord {
        version: MIGRATION_RECORD_VERSION,
        tool: "cerulion bag migrate".to_string(),
        migrated_at_ns: stamp.epoch_ns(),
        source_bag_file_name: input
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default(),
        // From the READER's own mapped bytes, never a fresh read of the input
        // NAME. `BagReader::open` mapped that inode once at verb start and every
        // frame and attachment below is copied from it; re-resolving the path
        // here — seconds later on a large bag — let a concurrent replacement
        // stamp the SUBSTITUTE's hash and size into a record describing bytes
        // the migration never read, inverting this field's whole purpose
        // (verification against the true original would FAIL and against an
        // attacker-chosen file SUCCEED). It is the fd-coherence rule this file
        // already applies to the OUTPUT, applied to the input.
        //
        // It also saves a full extra I/O pass: a `sha256_file` would re-read
        // the whole bag from disk to hash bytes that are already mapped.
        source_bag_size_bytes: reader.bytes().len() as u64,
        source_bag_sha256: sha256_hex(reader.bytes()),
        source_graph_yaml_sha256: sha256_hex(
            &attachments
                .iter()
                .find(|a| a.name == GRAPH_ATTACHMENT)
                .map(|a| a.data.clone())
                .unwrap_or_default(),
        ),
        migrated_graph_yaml_sha256: sha256_hex(migrated_yaml.as_bytes()),
        stripped: strip.stripped.clone(),
    };
    let record_bytes = serde_json::to_vec_pretty(&record).map_err(|e| {
        CliError::Validation(format!(
            "bag migrate: cannot render the {MIGRATION_ATTACHMENT} attachment: {e}"
        ))
    })?;

    // Write to a sibling scratch and hard-link it into place, so an interrupted
    // migration never leaves a plausible-looking half-bag at the name the user
    // will reach for. The scratch is CLAIMED atomically and written through the
    // handle that claimed it — see [`ScratchFile`].
    let scratch = ScratchFile::claim(output)?;
    #[cfg(test)]
    run_after_claim_hook_with(scratch.path());
    let result = write_bag_contents(
        reader,
        input,
        &scratch,
        &topics,
        provisioning,
        attachments,
        migrated_yaml,
        &record_bytes,
    );
    match result {
        Ok(()) => {
            scratch.adopt_permissions_of(reader);
            install_no_clobber(&scratch, output)
        }
        Err(e) => {
            scratch.remove_if_ours();
            Err(e)
        }
    }
}

/// Move the finished scratch bag to `output` WITHOUT ever replacing a file that
/// appeared there in the meantime.
///
/// [`resolve_output_path`]'s existence check runs BEFORE the migration, and a
/// migration reads and rewrites a whole bag — seconds on a large one — so the
/// check and the install are far apart in time. `std::fs::rename` REPLACES its
/// destination silently, so a concurrent migration, a second operator, or a
/// script that wrote there in that window lost its file: the no-overwrite
/// contract this verb states, and Principle #6, both broken by the last step.
///
/// `hard_link` is the fix because it is ATOMIC and it FAILS on an existing
/// destination — the check and the claim become one operation, so there is no
/// window left to lose. The scratch file is a SIBLING of the destination (same
/// directory, therefore the same filesystem), which is what makes a link legal
/// here at all.
///
/// Crash safety is BETTER than the rename it replaces, not merely equal: at
/// every instant the destination is either absent or the complete bag, and an
/// interruption leaves only the scratch file behind.
///
/// **What it costs**, stated because it is real: a filesystem with no hard
/// links (FAT/exFAT — a USB stick is a plausible destination on a robot)
/// refuses the install. That is reported LOUDLY with the scratch path NAMED and
/// the scratch file KEPT, so the migrated bag is not lost — the alternative
/// would be to fall back to a racing `rename`, which is a silent degrade of the
/// exact guarantee this function exists to make.
///
/// # What the link RESOLVES, and why that needed a second check
///
/// `hard_link` takes a PATH, so it links whatever the scratch NAME resolves to
/// at that instant — not necessarily the file this migration wrote. A process
/// that can write to the output directory can unlink the scratch and put its
/// own file (or a symlink) there in the window between the last write and this
/// call, and the migration would then publish the substitute and report
/// success. That race is real against this verb, so the
/// install VERIFIES what it published: [`ScratchFile`] carries the identity of
/// the inode it claimed, and a destination whose identity does not match it is
/// unlinked again and the migration refuses. The migrated bag is discarded
/// rather than published under a name it does not describe — the same choice
/// the `AlreadyExists` arm makes, for the same reason.
fn install_no_clobber(scratch: &ScratchFile, output: &Path) -> CliResult<()> {
    run_before_install_hook();
    let temp = scratch.path();
    match std::fs::hard_link(temp, output) {
        Ok(()) => {
            // The link is made — but to WHAT? If the name was substituted in the
            // window above, `output` is now a link to somebody else's file, and
            // publishing it would be worse than refusing: the caller is told a
            // migration succeeded and hands that path to `bag play`.
            //
            // TWO sub-cases reach the mismatch below, and the second one is
            // easy to overlook:
            //
            // 1. The SCRATCH was replaced before the link. `output` is then a
            //    second link to the substitute, created by us — removing it is
            //    unambiguously right.
            // 2. The OUTPUT was replaced AFTER a successful link: a racer
            //    unlinked the bag we just published and put their own file
            //    there before the `lstat` below. `remove_file(output)` then
            //    deletes the RACER's file — the very outcome `remove_if_ours`
            //    is engineered to avoid on the scratch side.
            //
            // Sub-case 2 is the same microsecond lstat-then-unlink residual
            // documented on `ScratchFile`, and it is ACCEPTED for the same
            // reasons: there is no portable "unlink only if it is this inode",
            // it has no benign path into it (the racer must first delete a file
            // we created a microsecond earlier), and the alternative — leaving a
            // name the migration cannot vouch for — publishes an unverified bag. What is
            // NOT acceptable is claiming it cannot happen, so the refusal below
            // names both possibilities instead of diagnosing one.
            if !scratch.published_at(output) {
                let _ = std::fs::remove_file(output);
                scratch.remove_if_ours();
                return Err(CliError::Validation(format!(
                    "bag migrate: '{}' is not the file this migration wrote — either the scratch \
                     file '{}' was replaced before it was put in place, or '{}' was replaced \
                     immediately after. Refusing to publish a bag this migration cannot vouch \
                     for. The recording is untouched, and nothing of this migration's remains at \
                     '{}'. Something else is writing into that directory: re-run with -o naming \
                     a path in a directory only you can write to.",
                    output.display(),
                    temp.display(),
                    output.display(),
                    output.display()
                )));
            }
            // The destination is complete. Dropping our scratch NAME is
            // tidiness, not correctness, so a failure here must not fail a
            // migration that has already succeeded.
            if let Err(e) = scratch.remove_if_ours_reporting() {
                tracing::warn!(
                    scratch = %temp.display(),
                    error = %e,
                    "bag migrate: the migrated bag is in place, but its scratch copy could not \
                     be removed — delete it by hand"
                );
            }
            Ok(())
        }
        Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
            scratch.remove_if_ours();
            // `link(2)` reports EEXIST for anything already at the name — which
            // is USUALLY a file that appeared during the run, but two other
            // shapes reach here and a generic message would misname both:
            //
            //  * a DANGLING SYMLINK already at the output. `resolve_output_path`
            //    asks `exists()`, which FOLLOWS symlinks and so answers false
            //    for one pointing nowhere, and the entry is only caught here —
            //    it predates the run rather than appearing during it. `lstat`
            //    distinguishes it, so it gets its own sentence.
            //  * on NFS, `link(2)`'s classic lost-reply retransmit can report
            //    EEXIST for a link that actually SUCCEEDED. Nothing is lost
            //    either way (the output link holds the inode, and dropping the
            //    scratch name is safe), but "the migration was discarded" would
            //    be false there, so the wording does not claim it.
            let dangling = std::fs::symlink_metadata(output)
                .map(|md| md.file_type().is_symlink() && !output.exists())
                .unwrap_or(false);
            let what = if dangling {
                format!(
                    "'{}' is a symlink that points at nothing, and it already occupied that name \
                     before this migration started",
                    output.display()
                )
            } else {
                format!(
                    "'{}' already exists — it appeared while this migration was running, or it \
                     is this migration's own output already in place",
                    output.display()
                )
            };
            Err(CliError::Validation(format!(
                "bag migrate: {what}. Refusing to overwrite it. The recording is untouched and \
                 the file at that name is whoever wrote it — check it before assuming this \
                 migration failed. Re-run with -o naming a path that does not exist."
            )))
        }
        Err(e) => Err(CliError::Validation(format!(
            "bag migrate: cannot put the migrated bag in place at '{}': {e}. The migrated bag is \
             COMPLETE and has been left at '{}' — move it yourself, or delete it and re-run with \
             -o naming a destination on a filesystem that supports hard links.",
            output.display(),
            temp.display()
        ))),
    }
}

// A test seam, armed for exactly ONE install.
//
// The TOCTOU window it exists to pin is the interval between
// `resolve_output_path`'s existence check and `install_no_clobber`, and nothing
// OUTSIDE the migration can write into it — by the time a caller regains
// control the install has already happened. So the hook runs at the one instant
// a racing writer would land, letting the full production path be driven
// through the state a race produces.
//
// A plain comment, not a doc comment: rustdoc generates nothing for a macro
// invocation, so `///` here is an `unused_doc_comments` error.
#[cfg(test)]
thread_local! {
    static BEFORE_INSTALL_HOOK: std::cell::RefCell<Option<Box<dyn Fn()>>> =
        const { std::cell::RefCell::new(None) };
}

#[cfg(test)]
fn arm_before_install_hook(f: Box<dyn Fn()>) {
    BEFORE_INSTALL_HOOK.with(|h| *h.borrow_mut() = Some(f));
}

/// TAKES the hook, so an armed race fires once and a second migration in the
/// same test is an ordinary one.
#[cfg(test)]
fn run_before_install_hook() {
    let hook = BEFORE_INSTALL_HOOK.with(|h| h.borrow_mut().take());
    if let Some(f) = hook {
        f();
    }
}

#[cfg(not(test))]
fn run_before_install_hook() {}

/// The after-OPEN hook's type — see [`AfterClaimHook`] on why it is named.
#[cfg(test)]
type AfterOpenHook = Box<dyn Fn(&Path)>;

// A third test seam, armed for exactly ONE migration, fired the instant the
// input bag has been mapped and before anything is read back about it.
//
// It exists because "the provenance describes the bytes we copied" has no other
// observable: on a quiet filesystem the mapped inode and the path agree, so a
// by-name read passes every arm. Replacing the input NAME in this window
// separates them — the record must still carry the ORIGINAL's hash and size and
// the output must still carry its mode, because that is the file the migrated
// bag actually descends from.
#[cfg(test)]
thread_local! {
    static AFTER_OPEN_HOOK: std::cell::RefCell<Option<AfterOpenHook>> =
        const { std::cell::RefCell::new(None) };
}

#[cfg(test)]
fn arm_after_open_hook(f: AfterOpenHook) {
    AFTER_OPEN_HOOK.with(|h| *h.borrow_mut() = Some(f));
}

#[cfg(test)]
fn run_after_open_hook_with(path: &Path) {
    let hook = AFTER_OPEN_HOOK.with(|h| h.borrow_mut().take());
    if let Some(f) = hook {
        f(path);
    }
}

// A second test seam, armed for exactly ONE migration, fired between the
// scratch CLAIM and the first byte written into it.
//
// It exists because "the bag is written through the claimed HANDLE, never by
// re-resolving the NAME" has no other observable: on an unmolested filesystem
// both spellings produce the same file, so a writer that re-opens the path
// passes every other arm in this module. Substituting the name in THIS window
// separates them — the handle writes into the inode it claimed and leaves the
// substitute alone, while a path-resolving writer TRUNCATES the substitute and
// writes the migrated bag into somebody else's file.
/// The after-claim hook's own type. Named rather than written inline because
/// `clippy::type_complexity` (CI's stable, 1.98) refuses the nested form — the
/// sibling `BEFORE_INSTALL_HOOK` takes no argument and stays under the
/// threshold, so the alias is what the `&Path` costs.
#[cfg(test)]
type AfterClaimHook = Box<dyn Fn(&Path)>;

#[cfg(test)]
thread_local! {
    static AFTER_CLAIM_HOOK: std::cell::RefCell<Option<AfterClaimHook>> =
        const { std::cell::RefCell::new(None) };
}

#[cfg(test)]
fn arm_after_claim_hook(f: AfterClaimHook) {
    AFTER_CLAIM_HOOK.with(|h| *h.borrow_mut() = Some(f));
}

#[cfg(test)]
fn run_after_claim_hook_with(path: &Path) {
    let hook = AFTER_CLAIM_HOOK.with(|h| h.borrow_mut().take());
    if let Some(f) = hook {
        f(path);
    }
}

/// The parenthetical the already-migrated refusal appends, naming the EXISTING
/// record so an operator can tell which migration they are looking at.
///
/// Best-effort by construction: the bytes come from a file this build did not
/// write, so an unreadable record must not turn a clear refusal into a parse
/// error. An unparseable one is REPORTED as unparseable rather than skipped —
/// "there is a record here and it is damaged" is a fact worth carrying.
fn describe_existing_migration(data: &[u8]) -> String {
    match serde_json::from_slice::<MigrationRecord>(data) {
        Ok(r) => format!(
            " (written by `{}` at {} ns, from '{}', removing {} key(s))",
            r.tool,
            r.migrated_at_ns,
            r.source_bag_file_name,
            r.stripped.len()
        ),
        Err(_) => " (whose contents this build cannot parse)".to_string(),
    }
}

/// The scratch prefix every migration's temporary file starts with.
///
/// A stable PREFIX (with an unpredictable tail) rather than a stable NAME, so a
/// sweep for leftovers is still possible while the exact path is not guessable.
const SCRATCH_PREFIX: &str = ".migrating.";

/// The scratch bag a migration writes into: a sibling path this process CLAIMED
/// atomically, plus the OPEN HANDLE that claimed it.
///
/// # Why the handle, and not just the path
///
/// A deterministic sibling scratch name (`.<output>.migrating`) would be one
/// that three separate operations resolve independently — an `exists()`
/// pre-check, the writer's own `File::create`, and `hard_link` at the install.
/// Each resolution is a fresh look at a name, so anyone who can write to the
/// output directory can act between any two of them. Both halves of that are
/// real against this verb, and the second publishes
/// attacker-chosen bytes at the requested output while reporting success.
///
/// Three things close it, and all three are needed:
///
/// 1. **The name is claimed atomically.** `create_new(true)` is one syscall that
///    creates-or-fails, so the `exists()`-then-create window is gone and a
///    pre-placed file is a loud refusal rather than a silent adoption.
/// 2. **The bytes go through the claiming handle.** The writer is handed this
///    `File` ([`cerulion_bag::FileSink::from_file`]), never the path, so the
///    migration cannot be redirected into a substitute mid-write.
/// 3. **The identity is CARRIED and re-checked.** `hard_link` still resolves a
///    name, so the install compares what it published against the inode this
///    file claimed ([`Self::published_at`]) and refuses if they differ, and
///    cleanup unlinks the name only while it is still ours
///    ([`Self::remove_if_ours`]) — deleting a substituted path would destroy a
///    file belonging to whoever put it there.
///
/// The unpredictable tail is defence in depth, not the fix: it means an attacker
/// cannot pre-place or target the scratch without first scanning for it, while
/// (3) is what holds even against one who knows the name exactly.
///
/// # Residual, stated rather than implied
///
/// [`Self::remove_if_ours`] re-checks identity and then unlinks, and there is no
/// portable "unlink only if it is this inode" — so a substitution landing
/// between those two steps deletes the substitute. It is a microsecond window
/// against a filesystem call, the published bag is already safe by then (the
/// hard link is made before any cleanup runs), and the alternative — never
/// cleaning up — leaves a whole bag on disk after every migration.
struct ScratchFile {
    path: PathBuf,
    /// The handle `create_new` returned. Its `(dev, ino)` — not its name — is
    /// the identity every later check compares against.
    ///
    /// HELD for the migration's lifetime, and that is load-bearing rather than
    /// convenience: `(dev, ino)` is a pair of numbers, and an inode number is
    /// reusable once its last link and last open descriptor are gone. An open
    /// handle keeps the inode alive, so a substitution can never be handed OUR
    /// numbers by the filesystem recycling them — which would make every
    /// identity check below answer `true` for a file we did not write.
    file: std::fs::File,
    dev: u64,
    ino: u64,
}

impl ScratchFile {
    /// Claim an unpredictable sibling of `output`, atomically.
    fn claim(output: &Path) -> CliResult<Self> {
        use std::os::unix::fs::{MetadataExt as _, OpenOptionsExt as _};

        let name = output
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "bag.mcap".to_string());

        // The tail only has to be unguessable-in-practice and collision-free:
        // (3) above is what makes a KNOWN name safe, so this needs no CSPRNG.
        // pid separates concurrent processes; the nanosecond clock and the
        // per-process counter separate attempts within one.
        let pid = std::process::id();
        let mut last_err: Option<std::io::Error> = None;
        for attempt in 0..SCRATCH_CLAIM_ATTEMPTS {
            let nonce = scratch_nonce();
            let path = output.with_file_name(format!(
                "{SCRATCH_PREFIX}{name}.{pid}.{nonce:016x}.{attempt}"
            ));
            match std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                // Owner-only: the scratch holds the whole bag, and a bag can
                // carry an `env.json` the recorder redacts. It is also what
                // stops a bystander truncating it in place.
                .mode(0o600)
                .open(&path)
            {
                Ok(file) => {
                    // The identity is read from the HANDLE (`fstat`), never from
                    // the path — reading it back by name would be one more
                    // name resolution, i.e. the very thing this type exists to
                    // stop doing.
                    let md = file.metadata().map_err(|e| {
                        CliError::Validation(format!(
                            "bag migrate: cannot stat the scratch file '{}': {e}",
                            path.display()
                        ))
                    })?;
                    return Ok(Self {
                        dev: md.dev(),
                        ino: md.ino(),
                        path,
                        file,
                    });
                }
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
                    last_err = Some(e);
                    continue;
                }
                Err(e) => {
                    return Err(CliError::Validation(format!(
                        "bag migrate: cannot create a scratch file beside '{}': {e}. The \
                         migration writes its output to a sibling of the destination first, so \
                         the destination's directory has to be writable.",
                        output.display()
                    )));
                }
            }
        }
        Err(CliError::Validation(format!(
            "bag migrate: could not claim a scratch file beside '{}' after \
             {SCRATCH_CLAIM_ATTEMPTS} attempts ({}). Something is creating files at this \
             migration's scratch names as fast as it can pick them; nothing was written.",
            output.display(),
            last_err
                .map(|e| e.to_string())
                .unwrap_or_else(|| "no error recorded".to_string()),
        )))
    }

    fn path(&self) -> &Path {
        &self.path
    }

    /// Give the finished scratch the SOURCE recording's permissions, so the
    /// published copy is readable by exactly whoever could read the original.
    ///
    /// The scratch is created `0600` because it is a transient whole copy of the
    /// bag — attachments included — and there is no reason for it to be
    /// world-readable while it is being written. But the install is a HARD LINK,
    /// so the scratch's mode becomes the OUTPUT's mode: left alone, `0600` would
    /// quietly make every migrated bag stricter than the recording it descends
    /// from and stricter than `cerulion_bagd` writes one, which is a user-facing
    /// change nobody asked for. Mirroring the input is the right answer for a
    /// COPY: same content, same audience.
    ///
    /// Applied through the HANDLE (`File::set_permissions`), never by path — a
    /// path here would be one more name resolution, which is the thing this type
    /// exists to stop doing.
    ///
    /// Best-effort: a failure leaves the strict mode, which is the safe
    /// direction, and says so once.
    fn adopt_permissions_of(&self, reader: &BagReader) {
        // Through the READER's handle, not the input path: a fresh
        // `metadata(input)` here describes whatever the name resolves to now,
        // so a concurrent replacement would publish the migrated bag under a
        // SUBSTITUTE's mode. Same reason the provenance hash comes from the
        // mapped bytes.
        let outcome = reader
            .file()
            .ok_or_else(|| {
                std::io::Error::other("this reader has no file handle (built from bytes)")
            })
            .and_then(|f| f.metadata())
            .and_then(|md| self.file.set_permissions(md.permissions()));
        if let Err(e) = outcome {
            tracing::warn!(
                error = %e,
                "bag migrate: could not read the recording's permissions, so the migrated bag \
                 keeps the scratch file's owner-only mode — widen it by hand if others need to \
                 read it"
            );
        }
    }

    /// The `File` the bag writer must write THROUGH.
    ///
    /// Takes `&self` and hands back a DUP rather than moving the handle out, so
    /// the identity checks stay available after the writer has consumed its own.
    fn writer_handle(&self) -> CliResult<std::fs::File> {
        self.file.try_clone().map_err(|e| {
            CliError::Validation(format!(
                "bag migrate: cannot duplicate the scratch handle for '{}': {e}",
                self.path.display()
            ))
        })
    }

    /// Does `path` name the very inode this scratch claimed?
    ///
    /// `symlink_metadata` (lstat), never `metadata`: a substitute may be a
    /// SYMLINK to somewhere else entirely, and following it would compare the
    /// wrong file — and could report a match for a link pointing back at us.
    fn is_ours(&self, path: &Path) -> bool {
        use std::os::unix::fs::MetadataExt as _;
        std::fs::symlink_metadata(path)
            .map(|md| md.dev() == self.dev && md.ino() == self.ino)
            .unwrap_or(false)
    }

    /// Did the install publish OUR file at `output`?
    fn published_at(&self, output: &Path) -> bool {
        self.is_ours(output)
    }

    /// Unlink the scratch NAME, but only while it still names our file.
    fn remove_if_ours(&self) {
        let _ = self.remove_if_ours_reporting();
    }

    /// [`Self::remove_if_ours`], reporting why it did not remove anything.
    ///
    /// A path that is no longer ours is reported as a SUCCESS with nothing
    /// removed: there is nothing of ours left to clean up, and the caller's
    /// "could not remove the scratch" warning would name the wrong problem.
    fn remove_if_ours_reporting(&self) -> std::io::Result<()> {
        if !self.is_ours(&self.path) {
            return Ok(());
        }
        std::fs::remove_file(&self.path)
    }
}

/// How many scratch names [`ScratchFile::claim`] will try before giving up.
///
/// A collision needs another process to hold the same pid, nanosecond and
/// counter, or to be creating files at our names deliberately. Bounded so the
/// second case is a loud refusal rather than a spin.
const SCRATCH_CLAIM_ATTEMPTS: u32 = 16;

/// A per-process-unique 64-bit tail for a scratch name.
fn scratch_nonce() -> u64 {
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    nanos
        ^ (COUNTER
            .fetch_add(1, Ordering::Relaxed)
            .wrapping_mul(0x9E37_79B9_7F4A_7C15))
}

#[allow(clippy::too_many_arguments)]
fn write_bag_contents(
    reader: &BagReader,
    input: &Path,
    scratch: &ScratchFile,
    topics: &[TopicSchema],
    provisioning: std::collections::BTreeMap<String, cerulion_bag::ChannelProvisioning>,
    attachments: &[BagAttachment],
    migrated_yaml: &str,
    record_bytes: &[u8],
) -> CliResult<()> {
    let temp = scratch.path();
    let config = BagWriterConfig {
        library: MIGRATE_LIBRARY.to_string(),
        provisioning,
        ..Default::default()
    };
    // Through the HANDLE, never the path: `BagWriter::create` would resolve the
    // scratch NAME again, which is the redirection [`ScratchFile`] exists to
    // stop. `with_sink` runs the same registration validation `create` does.
    let sink = cerulion_bag::FileSink::from_file(scratch.writer_handle()?).map_err(|e| {
        CliError::Validation(format!(
            "bag migrate: cannot open a writer on the scratch file '{}': {e}",
            temp.display()
        ))
    })?;
    let mut writer = BagWriter::with_sink(sink, config, topics).map_err(|e| {
        CliError::Validation(format!(
            "bag migrate: cannot create '{}': {e}",
            temp.display()
        ))
    })?;

    // Every topic the input carries must resolve in the OUTPUT's channel table
    // before a single frame is written. This is asked of the writer itself
    // rather than of a hand-copied list of reserved topics, so a reserved
    // channel added to `cerulion_bag` after this file was written cannot make
    // the migration drop frames — it fails here, loudly, instead.
    for ch in reader.channels().map_err(|e| {
        CliError::Validation(format!("bag migrate: cannot read the channel table: {e}"))
    })? {
        if writer.channel_id(&ch.topic).is_none() {
            return Err(CliError::Validation(format!(
                "bag migrate: '{}' records the channel '{}', which this build's bag writer does \
                 not know how to register. The bag was written by a NEWER Cerulion than the one \
                 running this migration — upgrade, and migrate with that. Refusing rather than \
                 writing a copy with the channel dropped.",
                input.display(),
                ch.topic
            )));
        }
    }

    // Frames, in file order, each with its own timestamps and bytes.
    let messages = reader.messages().map_err(|e| {
        CliError::Validation(format!(
            "bag migrate: cannot walk the frames of '{}': {e}",
            input.display()
        ))
    })?;
    for m in messages {
        let m = m.map_err(|e| {
            CliError::Validation(format!(
                "bag migrate: cannot read a frame of '{}': {e}",
                input.display()
            ))
        })?;
        writer
            .write_message(&m.topic, m.sequence, m.log_time, m.publish_time, &[&m.data])
            .map_err(|e| {
                CliError::Validation(format!(
                    "bag migrate: cannot write a frame of '{}': {e}",
                    m.topic
                ))
            })?;
    }

    // Attachments, in the input's own order, with `graph.yaml` replaced.
    for a in attachments {
        let data: &[u8] = if a.name == GRAPH_ATTACHMENT {
            migrated_yaml.as_bytes()
        } else {
            &a.data
        };
        writer
            .write_attachment(&a.name, &a.media_type, a.log_time, a.create_time, data)
            .map_err(|e| {
                CliError::Validation(format!(
                    "bag migrate: cannot write the {} attachment: {e}",
                    a.name
                ))
            })?;
    }
    writer
        .write_attachment(
            MIGRATION_ATTACHMENT,
            MIGRATION_MEDIA_TYPE,
            0,
            0,
            record_bytes,
        )
        .map_err(|e| {
            CliError::Validation(format!(
                "bag migrate: cannot write the {MIGRATION_ATTACHMENT} attachment: {e}"
            ))
        })?;
    writer.finalize().map_err(|e| {
        CliError::Validation(format!(
            "bag migrate: cannot finalize '{}': {e}",
            temp.display()
        ))
    })
}

fn sha256_hex(bytes: &[u8]) -> String {
    let d = sha2::Sha256::digest(bytes);
    d.iter().map(|b| format!("{b:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── the strip engine: path + line, against crafted documents ───────────

    #[test]
    fn a_legacy_policy_block_is_located_by_path_and_line() {
        let yaml = "prefix: p\nnodes:\n  - id: n0\n    type: t0\n    policy:\n      period_ms: \
                    10\n    outputs:\n      - name: o\n        schema: a/B\n";
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip.stripped,
            vec![StrippedKey {
                key: "policy".to_string(),
                path: "nodes[0].policy".to_string(),
                line: 5,
            }]
        );
        // The surviving graph is intact — the block went, nothing else did.
        assert_eq!(strip.config.prefix, "p");
        assert_eq!(strip.config.nodes.len(), 1);
        assert_eq!(strip.config.nodes[0].id, "n0");
        assert_eq!(strip.config.nodes[0].outputs.len(), 1);
        assert_eq!(strip.config.nodes[0].outputs[0].name, "o");
    }

    #[test]
    fn keys_at_three_different_depths_are_each_reported_with_their_own_path_and_line() {
        // Line numbers are hand-counted against the literal below, and each
        // key sits at a DIFFERENT nesting depth so a path built from the wrong
        // level cannot pass.
        let yaml = concat!(
            "legacy_top: 1\n",       // 1  top level
            "prefix: p\n",           // 2
            "nodes:\n",              // 3
            "  - id: n0\n",          // 4
            "    type: t0\n",        // 5
            "    policy:\n",         // 6  nodes[0]
            "      period_ms: 10\n", // 7
            "    outputs:\n",        // 8
            "      - name: o\n",     // 9
            "        schema: a/B\n", // 10
            "        dpeth: 3\n",    // 11 nodes[0].outputs[0]
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        let mut got: Vec<(String, usize)> = strip
            .stripped
            .iter()
            .map(|s| (s.path.clone(), s.line))
            .collect();
        got.sort();
        assert_eq!(
            got,
            vec![
                ("legacy_top".to_string(), 1),
                ("nodes[0].outputs[0].dpeth".to_string(), 11),
                ("nodes[0].policy".to_string(), 6),
            ]
        );
        assert_eq!(strip.config.nodes[0].outputs[0].schema, "a/B");
    }

    #[test]
    fn the_same_key_on_two_nodes_is_reported_twice_with_distinct_lines() {
        // The ambiguous case: one key name, two offending occurrences. The
        // report must distinguish them, which is the whole reason a path and a
        // line are carried rather than just a key name.
        let yaml = concat!(
            "nodes:\n",        // 1
            "  - id: a\n",     // 2
            "    type: t\n",   // 3
            "    policy: 1\n", // 4
            "  - id: b\n",     // 5
            "    type: t\n",   // 6
            "    policy: 2\n", // 7
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].policy", 4), ("nodes[1].policy", 7)]
        );
        assert_eq!(strip.config.nodes.len(), 2);
        assert_eq!(strip.config.nodes[1].id, "b");
    }

    #[test]
    fn a_key_that_starts_a_sequence_entry_keeps_the_entry() {
        // `- policy:` — blanking the whole line would take the dash with it and
        // silently merge the entry into its neighbour.
        let yaml = concat!(
            "nodes:\n",      // 1
            "  - policy:\n", // 2
            "      x: 1\n",  // 3
            "    id: a\n",   // 4
            "    type: t\n", // 5
            "  - id: b\n",   // 6
            "    type: t\n", // 7
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].policy", 2)]
        );
        assert_eq!(strip.config.nodes.len(), 2, "the entry must survive");
        assert_eq!(strip.config.nodes[0].id, "a");
        assert_eq!(strip.config.nodes[1].id, "b");
    }

    #[test]
    fn a_top_level_block_is_removed_whole() {
        let yaml = concat!(
            "nodes: []\n", // 1
            "policy:\n",   // 2
            "  a: 1\n",    // 3
            "  b:\n",      // 4
            "    c: 2\n",  // 5
            "prefix: p\n", // 6
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("policy", 2)]
        );
        assert_eq!(strip.config.prefix, "p", "the sibling key must survive");
    }

    #[test]
    fn a_network_block_key_is_reported_under_the_block() {
        let yaml = "nodes: []\nnetwork:\n  mode: peer\n  bogus: 1\n";
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("network.bogus", 4)]
        );
        let net = strip.config.network.expect("network survives");
        assert_eq!(net.mode, cerulion_core::graph::config::NetworkMode::Peer);
    }

    // ── flow style ─────────────────────────────────────────────────────────

    #[test]
    fn a_flow_mapping_key_is_removed_with_its_comma_and_its_siblings_survive() {
        // `outputs: [{…}]` is ordinary legacy YAML, and the offending key sits
        // BEHIND its siblings on the line — so blanking the line would take the
        // whole node with it.
        let yaml = concat!(
            "prefix: p\n",                                       // 1
            "nodes:\n",                                          // 2
            "  - id: n0\n",                                      // 3
            "    type: t0\n",                                    // 4
            "    outputs: [{name: o, dpeth: 4, schema: a/B}]\n", // 5
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].outputs[0].dpeth", 5)]
        );
        // Both siblings survive — a whole-line blank loses them AND the node.
        assert_eq!(strip.config.nodes.len(), 1);
        assert_eq!(strip.config.nodes[0].id, "n0");
        assert_eq!(strip.config.nodes[0].outputs.len(), 1);
        assert_eq!(strip.config.nodes[0].outputs[0].name, "o");
        assert_eq!(strip.config.nodes[0].outputs[0].schema, "a/B");
    }

    #[test]
    fn a_flow_key_at_the_top_level_and_one_nested_are_each_removed() {
        // Top level `{…}` (depth 1) and a mapping inside a flow SEQUENCE
        // inside it (depth 2), so the depth walk has to be right at two levels.
        let yaml = "{prefix: p, legacy_top: 1, nodes: [{id: n0, type: t0, policy: 9}]}\n";
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        let mut got: Vec<&str> = strip.stripped.iter().map(|s| s.path.as_str()).collect();
        got.sort();
        assert_eq!(got, vec!["legacy_top", "nodes[0].policy"]);
        assert!(strip.stripped.iter().all(|s| s.line == 1));
        assert_eq!(strip.config.prefix, "p");
        assert_eq!(strip.config.nodes.len(), 1);
        assert_eq!(strip.config.nodes[0].id, "n0");
        assert_eq!(strip.config.nodes[0].node_type, "t0");
    }

    #[test]
    fn the_last_entry_of_a_flow_mapping_takes_the_comma_before_it() {
        // A trailing comma is not legal YAML, so removing the LAST entry has to
        // take the comma that preceded it instead of its own.
        let yaml = "prefix: p\nnodes: [{id: n0, type: t0, policy: 9}]\n";
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].policy", 2)]
        );
        assert_eq!(strip.config.nodes[0].id, "n0");
        assert!(
            !strip.cleaned_yaml.contains(",  ]") && !strip.cleaned_yaml.contains(", ]"),
            "no trailing comma may survive: {}",
            strip.cleaned_yaml
        );
    }

    #[test]
    fn a_flow_mapping_spanning_lines_is_still_read_as_flow_not_as_a_block_key() {
        // THE case a prefix test gets wrong: `dpeth` sits behind nothing but
        // spaces, exactly like a block key, while blanking its line unbalances
        // the collection.
        let yaml = concat!(
            "prefix: p\n",           // 1
            "nodes:\n",              // 2
            "  - id: n0\n",          // 3
            "    type: t0\n",        // 4
            "    outputs: [\n",      // 5
            "      {name: o,\n",     // 6
            "       schema: a/B,\n", // 7
            "       dpeth: 4}\n",    // 8
            "    ]\n",               // 9
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].outputs[0].dpeth", 8)]
        );
        assert_eq!(strip.config.nodes[0].outputs[0].name, "o");
        assert_eq!(strip.config.nodes[0].outputs[0].schema, "a/B");
    }

    #[test]
    fn two_flow_keys_on_one_line_are_each_located_by_their_own_column() {
        // Blanking with SPACES rather than deleting is what makes this work:
        // the second key's column is still valid after the first is removed.
        let yaml = "prefix: p\nnodes: [{id: n0, type: t0, policy: 1, dpeth: 2}]\n";
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        let mut got: Vec<&str> = strip.stripped.iter().map(|s| s.path.as_str()).collect();
        got.sort();
        assert_eq!(got, vec!["nodes[0].dpeth", "nodes[0].policy"]);
        assert!(strip.stripped.iter().all(|s| s.line == 2));
        assert_eq!(strip.config.nodes[0].id, "n0");
        assert_eq!(strip.config.nodes[0].node_type, "t0");
    }

    #[test]
    fn a_flow_value_carrying_its_own_brackets_and_commas_ends_at_the_right_place() {
        // The entry's own value is a flow sequence, so its commas and its
        // closing bracket must not be read as the entry's terminator.
        let yaml = "prefix: p\nnodes: [{id: n0, policy: [1, 2, {a: 3}], type: t0}]\n";
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| s.path.as_str())
                .collect::<Vec<_>>(),
            vec!["nodes[0].policy"]
        );
        assert_eq!(strip.config.nodes[0].id, "n0");
        assert_eq!(strip.config.nodes[0].node_type, "t0");
    }

    #[test]
    fn a_bracket_inside_a_quoted_scalar_does_not_move_the_flow_depth() {
        // ANTI-TAUTOLOGY for the depth walk: a `{` in a STRING must not make a
        // later block key look like a flow entry (which would blank its
        // neighbours instead of its own block).
        let yaml = concat!(
            "prefix: \"p{[\"\n", // 1
            "nodes:\n",          // 2
            "  - id: n0\n",      // 3
            "    type: t0\n",    // 4
            "    policy: 1\n",   // 5
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].policy", 5)]
        );
        assert_eq!(strip.config.prefix, "p{[");
        assert_eq!(strip.config.nodes[0].id, "n0");
    }

    #[test]
    fn a_bracket_inside_a_comment_does_not_move_the_flow_depth() {
        let yaml = concat!(
            "prefix: p   # a note with { and [ in it\n", // 1
            "nodes:\n",                                  // 2
            "  - id: n0\n",                              // 3
            "    type: t0\n",                            // 4
            "    policy: 1\n",                           // 5
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].policy", 5)]
        );
        assert_eq!(strip.config.nodes[0].id, "n0");
    }

    // ── block scalars ──────────────────────────────────────────────────────

    #[test]
    fn a_block_scalar_containing_brackets_does_not_make_a_later_block_key_flow() {
        // THE block-scalar defect. A `|` scalar's body is ordinary text, so an
        // unbalanced `[` in it left the flow counter raised for the rest of the
        // document — and every later BLOCK key was then read as a flow entry,
        // which `flow_entry_end` cannot terminate, so the migration REFUSED a
        // graph it exists to fix.
        let yaml = concat!(
            "prefix: |\n",        // 1
            "  see [section 3\n", // 2  <- unbalanced, and just text
            "nodes:\n",           // 3
            "  - id: n0\n",       // 4
            "    type: t0\n",     // 5
            "    policy: 1\n",    // 6
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].policy", 6)]
        );
        assert_eq!(strip.config.prefix, "see [section 3\n");
        assert_eq!(strip.config.nodes[0].id, "n0");
    }

    #[test]
    fn a_folded_scalars_body_is_text_too_and_the_span_ends_at_the_dedent() {
        // The `>` twin, plus the SPAN's own boundary: a sibling key at the
        // owner's indentation ENDS the scalar, so structure after it is read
        // as structure again — a span that never closed would swallow the rest
        // of the document and hide every later key from the parser's view.
        // Both key styles ride ONE document, deliberately: a `>` arm that
        // carried only the FLOW key would be output-equivalent under a variant
        // that opens no span at all (that key really is in a flow collection,
        // so reading it as one is right by accident). The BLOCK key is what
        // makes the span load-bearing here; the flow key is what proves the
        // span CLOSED rather than swallowing the rest of the document.
        let yaml = concat!(
            "prefix: >\n",                                       // 1
            "  a note with { in it\n",                           // 2
            "  and [ in it too\n",                               // 3
            "nodes:\n",                                          // 4 structure again
            "  - id: n0\n",                                      // 5
            "    type: t0\n",                                    // 6
            "    policy: 1\n",                                   // 7 BLOCK key
            "  - id: n1\n",                                      // 8
            "    type: t1\n",                                    // 9
            "    outputs: [{name: o, dpeth: 4, schema: a/B}]\n", // 10 FLOW key
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        let mut got: Vec<(&str, usize)> = strip
            .stripped
            .iter()
            .map(|s| (s.path.as_str(), s.line))
            .collect();
        got.sort();
        assert_eq!(
            got,
            vec![("nodes[0].policy", 7), ("nodes[1].outputs[0].dpeth", 10)]
        );
        // The flow entry on line 10 was removed SURGICALLY (its siblings
        // survive), which is only possible if the span really did close.
        assert_eq!(strip.config.nodes[0].id, "n0");
        assert_eq!(strip.config.nodes[0].node_type, "t0");
        assert_eq!(strip.config.nodes[1].outputs[0].name, "o");
        assert_eq!(strip.config.nodes[1].outputs[0].schema, "a/B");
    }

    #[test]
    fn a_value_ending_in_a_dash_before_a_pipe_is_a_scalar_not_a_header() {
        // At the document level: `|` and `>` are indicators
        // only at the START of a scalar, so `id: a - |` is an ordinary plain
        // scalar — but a header reader that trusts `before.ends_with('-')` reads the
        // scalar's own final dash as a sequence marker and opens a span over
        // every following deeper line.
        //
        // Exactly here, that span hides line 5's flow
        // brackets, `dpeth` is classified as a BLOCK key, and the verb refuses
        // with "the migration could not isolate it there — the document may use
        // a YAML shape this rewrite does not model" — blaming the user's
        // document for a perfectly ordinary one.
        //
        // Both indicator spellings ride one document: `>` on a nested key
        // (line 4) proves the narrowing is not special-cased to `|` or to the
        // sequence-entry line, and its own deeper sibling would be swallowed by
        // the same false span.
        let yaml = concat!(
            "prefix: p\n",                                       // 1
            "nodes:\n",                                          // 2
            "  - id: a - |\n",                                   // 3  plain scalar
            "    type: t0 - >\n",                                // 4  plain scalar
            "    outputs: [{name: o, schema: a/B, dpeth: 4}]\n", // 5  FLOW key
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips rather than refusing");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].outputs[0].dpeth", 5)]
        );
        // The scalars really did carry their dashes through, and the flow entry
        // was edited SURGICALLY — both only possible if no span was opened.
        assert_eq!(strip.config.nodes[0].id, "a - |");
        assert_eq!(strip.config.nodes[0].node_type, "t0 - >");
        assert_eq!(strip.config.nodes[0].outputs[0].name, "o");
        assert_eq!(strip.config.nodes[0].outputs[0].schema, "a/B");
    }

    #[test]
    fn a_real_sequence_entry_block_scalar_still_opens_its_span() {
        // The ANTI-TAUTOLOGY half: the narrowing must not be satisfiable by a
        // reader that opens no span at all. A genuine `- |` sequence entry still
        // owns its body, so an unbalanced bracket in that body must NOT leave the
        // flow counter raised for the rest of the document — the block-scalar
        // defect this span exists to prevent, driven through the SEQUENCE arm
        // the header narrowing touched (the two block-scalar arms above both use
        // `key: |`).
        let yaml = concat!(
            "prefix: p\n",               // 1
            "multi_publisher_topics:\n", // 2
            "  - |\n",                   // 3  REAL header
            "    /tf [unbalanced\n",     // 4  its body: text
            "nodes:\n",                  // 5  structure again
            "  - id: n0\n",              // 6
            "    type: t0\n",            // 7
            "    policy: 1\n",           // 8  BLOCK key
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].policy", 8)]
        );
        assert_eq!(
            strip.config.multi_publisher_topics,
            vec!["/tf [unbalanced\n".to_string()],
            "the scalar's body is text, brackets and all"
        );
        assert_eq!(strip.config.nodes[0].id, "n0");
    }

    // ── the blanker's five defect shapes ───────────────────────────────────

    #[test]
    fn a_bracket_in_a_plain_scalar_does_not_turn_a_later_block_key_into_a_flow_entry() {
        // SILENT LOSS. YAML permits `[ ] { } ,` inside a plain
        // scalar in BLOCK context, so `name: see [ref-123` is that four-word
        // string — but a scanner that counts every bracket leaves the flow depth stuck
        // at 1 for the REST of the document. The next unknown key, a genuine
        // BLOCK key, then takes the flow path, and `flow_entry_end` scans
        // forward to the first comma — which a later plain scalar (`topic: /x,`)
        // supplies.
        //
        // THE FAILURE THIS PINS: the node's whole `outputs:` block gets
        // erased, the document re-parses CLEANLY (`outputs` is
        // `#[serde(default)]`), and the migration SHIPS a graph with no
        // producer for a topic the bag's channels still carry — reporting the
        // loss as nothing at all.
        let yaml = concat!(
            "identity: g\n",         // 1
            "name: see [ref-123\n",  // 2  plain scalar, unbalanced
            "nodes:\n",              // 3
            "  - id: n1\n",          // 4
            "    type: t\n",         // 5
            "    policy: gone\n",    // 6  the unknown BLOCK key
            "    outputs:\n",        // 7
            "      - name: o1\n",    // 8
            "        schema: a/B\n", // 9
            "        topic: /x,\n",  // 10 a legal comma in a plain scalar
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| s.key.as_str())
                .collect::<Vec<_>>(),
            vec!["identity", "policy"]
        );
        // THE PIN: the kept block survives (an over-eager erase leaves 0).
        assert_eq!(
            strip.config.nodes[0].outputs.len(),
            1,
            "the node's outputs must survive — erasing them shipped silently"
        );
        assert_eq!(strip.config.nodes[0].outputs[0].name, "o1");
        assert_eq!(strip.config.nodes[0].outputs[0].schema, "a/B");
        // …and the scalar that started it is intact, brackets and all.
        assert_eq!(strip.config.name.as_deref(), Some("see [ref-123"));
    }

    #[test]
    fn a_comment_inside_a_removed_block_does_not_end_the_sweep() {
        // FALSE REFUSAL. YAML allows a comment at ANY indentation
        // inside a block, and a hand-commented-out line at column 0 is ordinary
        // editing, in the old `policy:` block, which this module names
        // as "the one that really happens". A naive sweep treats it as the next
        // sibling, stops, and ORPHANS `period_ms: 100`; the re-parse then
        // fails and the verb refuses claiming "real YAML damage" about a
        // document that is legal, naming neither the real cause nor a remedy.
        let yaml = concat!(
            "name: g\n",               // 1
            "nodes:\n",                // 2
            "  - id: a\n",             // 3
            "    type: t\n",           // 4
            "    policy:\n",           // 5  unknown key
            "      trigger: period\n", // 6  its body
            "# note\n",                // 7  a comment at column 0
            "      period_ms: 100\n",  // 8  still its body
            "    outputs: []\n",       // 9  the REAL next sibling
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips rather than refusing");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| s.key.as_str())
                .collect::<Vec<_>>(),
            vec!["policy"]
        );
        // The sweep stopped in the right place: the sibling after the comment
        // survives, so the comment did not swallow the rest of the document.
        assert_eq!(strip.config.nodes[0].id, "a");
        assert_eq!(strip.config.nodes[0].node_type, "t");
    }

    #[test]
    fn an_orphan_from_a_removed_block_is_never_adopted_into_a_kept_one() {
        // THE WORST ONE. Same root cause as the arm above, in the
        // shape where the orphan is a valid key at the right indentation: it
        // REATTACHES to the preceding KEPT block and the document re-parses
        // CLEANLY, so the bag SHIPS.
        //
        // THE FAILURE THIS PINS: `connect: ["tcp/1.2.3.4:7447"]`, authored
        // inside the REMOVED `junk:` block, adopted into the real
        // `network:` block — a migration inventing network wiring, with the
        // report naming only `junk`. A robot would dial a locator nobody in the
        // recording ever configured.
        let yaml = concat!(
            "network:\n",                          // 1  KEPT
            "  mode: peer\n",                      // 2
            "junk:\n",                             // 3  unknown key
            "  sub: 1\n",                          // 4  its body
            "# c\n",                               // 5  comment at column 0
            "  connect: [\"tcp/1.2.3.4:7447\"]\n", // 6  STILL junk's body
            "nodes: []\n",                         // 7
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| s.key.as_str())
                .collect::<Vec<_>>(),
            vec!["junk"]
        );
        let network = strip
            .config
            .network
            .as_ref()
            .expect("the kept block survives");
        assert_eq!(
            network.connect,
            Vec::<String>::new(),
            "wiring authored inside a REMOVED block must never appear in a kept one"
        );
        // ANTI-TAUTOLOGY: the kept block is really still there, so the
        // assertion above is not passing on an absent `network:`.
        assert_eq!(network.mode, cerulion_core::graph::NetworkMode::Peer);
    }

    #[test]
    fn a_non_ascii_value_before_an_unknown_flow_key_still_migrates() {
        // FALSE REFUSAL with the wrong blame. serde_yaml's column
        // is a 1-based CHARACTER index; every slice here is byte-indexed. One
        // two-byte character earlier on a FLOW line shifts the landing point
        // left, `blank_flow_entry` refuses, and the verb blames "a multi-line
        // quoted scalar, or a block scalar" for a document whose only property
        // is an accented string.
        let yaml = concat!(
            "prefix: p\n",                                       // 1
            "nodes:\n",                                          // 2
            "  - id: n0\n",                                      // 3
            "    type: t0\n",                                    // 4
            "    inputs:\n",                                     // 5
            "      - {name: caf\u{e9}, source: /x, dpeth: 4}\n", // 6
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips rather than refusing");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| s.key.as_str())
                .collect::<Vec<_>>(),
            vec!["dpeth"]
        );
        // Both siblings survive, and the accented one is byte-intact — a
        // mis-landed slice would have taken part of it.
        assert_eq!(strip.config.nodes[0].inputs.len(), 1);
        assert_eq!(strip.config.nodes[0].inputs[0].name, "caf\u{e9}");
        assert_eq!(strip.config.nodes[0].inputs[0].source, "/x");
    }

    #[test]
    fn a_non_lf_line_break_inside_a_blanked_line_never_takes_a_kept_key_with_it() {
        // SILENT LOSS. libyaml breaks a line on `\r`, NEL, LS and
        // PS as well as `\n`; `str::lines` splits on `\n` alone. So the parser
        // sees TWO lines and reports the junk key on the first, while a
        // line-naive blanker sees ONE and blanks all of it — deleting `name: keepme`, then
        // re-parsing clean and reporting only the junk key.
        //
        // Both spellings, because they arrive from different places: a NEL from
        // CP1252 mojibake, a bare CR from a classic-Mac or truncated-CRLF file.
        for (label, sep) in [("NEL", "\u{85}"), ("bare CR", "\r")] {
            let yaml = format!("identity: g\nlegacy_junk: 1{sep}name: keepme\nnodes: []");
            let strip = strip_unknown_graph_keys(&yaml).expect("strips");
            assert_eq!(
                strip.config.name.as_deref(),
                Some("keepme"),
                "{label}: a kept key must survive the removal of its line-mate"
            );
            assert_eq!(
                strip
                    .stripped
                    .iter()
                    .map(|s| s.key.as_str())
                    .collect::<Vec<_>>(),
                vec!["identity", "legacy_junk"],
                "{label}: and the report must name exactly what went"
            );
        }
        // ANTI-TAUTOLOGY: ordinary CRLF was never broken and must stay
        // unbroken — normalising must not have changed what a normal document
        // parses to.
        let crlf = "identity: g\r\nlegacy_junk: 1\r\nname: keepme\r\nnodes: []\r\n";
        let strip = strip_unknown_graph_keys(crlf).expect("strips");
        assert_eq!(strip.config.name.as_deref(), Some("keepme"));
    }

    #[test]
    fn a_same_indent_sequence_value_goes_with_the_key_that_owns_it() {
        // FALSE REFUSAL with the wrong blame. YAML's
        // sequence-indentation exception makes `topics:` followed by `- /a` at
        // the SAME indent mainstream hand-written style. Blanking the key line
        // alone orphans the items, the re-parse fails at scan level, and the
        // verb refuses calling a legal document "real YAML damage" — on a bag
        // whose only problem is the exact class this verb ships to fix, with no
        // next step, since re-recording something that already happened is not
        // available.
        //
        // Driven at BOTH nestings: top level, and inside a node entry where the
        // key sits after a `- ` and its items align with the key rather than
        // with the parent sequence's dash.
        let top = "name: x\nnodes: []\ntopics:\n- /a\n- /b\n";
        let strip = strip_unknown_graph_keys(top).expect("strips rather than refusing");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| s.key.as_str())
                .collect::<Vec<_>>(),
            vec!["topics"]
        );
        assert_eq!(strip.config.name.as_deref(), Some("x"));

        let nested = concat!(
            "prefix: p\n",   // 1
            "nodes:\n",      // 2
            "  - id: a\n",   // 3
            "    subs:\n",   // 4  unknown key
            "    - /a\n",    // 5  its value, at the SAME indent
            "    - /b\n",    // 6
            "    type: t\n", // 7  the REAL next sibling
        );
        let strip = strip_unknown_graph_keys(nested).expect("strips rather than refusing");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| s.key.as_str())
                .collect::<Vec<_>>(),
            vec!["subs"]
        );
        // The sibling AFTER the sequence survives, which is what proves the
        // sweep consumed the items and then stopped in the right place rather
        // than running to the end of the entry.
        assert_eq!(strip.config.nodes[0].id, "a");
        assert_eq!(strip.config.nodes[0].node_type, "t");
    }

    #[test]
    fn an_unknown_key_inside_a_block_scalar_owner_is_removed_with_its_body() {
        // The scalar belongs to the key being stripped. Blanking the key line
        // alone would leave its body behind as an orphaned, over-indented
        // block — a YAML error the loop would then report as "damage".
        let yaml = concat!(
            "prefix: p\n",           // 1
            "nodes:\n",              // 2
            "  - id: n0\n",          // 3
            "    type: t0\n",        // 4
            "    policy: |\n",       // 5
            "      period_ms: 10\n", // 6
            "      note: [a, b\n",   // 7
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].policy", 5)]
        );
        assert_eq!(strip.config.nodes[0].id, "n0");
        assert_eq!(strip.config.nodes[0].node_type, "t0");
    }

    #[test]
    fn a_pipe_inside_a_scalar_value_does_not_open_a_block_scalar() {
        // ANTI-TAUTOLOGY for the header rule, both plain and quoted: the
        // coordinator's case (c). A header is a VALUE POSITION holding nothing
        // but `|`/`>` and its indicators; a value that merely CONTAINS a pipe
        // opens nothing.
        //
        // The SHAPE is what makes this observable, and it was measured rather
        // than assumed. A false header only ever DEPRESSES the depth (its span
        // skips the bracket counting), so it can only turn a FLOW key into a
        // block one — which means a false header at the top level is
        // output-equivalent (the next top-level key dedents, closing the span
        // before it can hide anything). The pipe therefore sits on a SEQUENCE
        // ENTRY, whose span would swallow that entry's own deeper lines and
        // hide the flow mapping on them.
        let yaml = concat!(
            "prefix: p\n",                                       // 1
            "nodes:\n",                                          // 2
            "  - id: a|b\n",                                     // 3 plain pipe
            "    type: \"x|y\"\n",                               // 4 quoted pipe
            "    outputs: [{name: o, dpeth: 4, schema: a/B}]\n", // 5 FLOW key
        );
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| (s.path.as_str(), s.line))
                .collect::<Vec<_>>(),
            vec![("nodes[0].outputs[0].dpeth", 5)]
        );
        // Read as a block key, line 5 is blanked WHOLE and the outputs entry
        // disappears with it — so the siblings surviving is the discriminator.
        assert_eq!(strip.config.nodes[0].id, "a|b");
        assert_eq!(strip.config.nodes[0].node_type, "x|y");
        assert_eq!(strip.config.nodes[0].outputs.len(), 1);
        assert_eq!(strip.config.nodes[0].outputs[0].name, "o");
        assert_eq!(strip.config.nodes[0].outputs[0].schema, "a/B");
    }

    #[test]
    fn the_block_scalar_header_reader_answers_its_hand_written_vectors() {
        // The predicate the whole span turns on, driven directly — a predicate
        // that answers too easily makes its guard vacuous without failing
        // anything.
        for (line, want) in [
            ("prefix: |", Some(0)),
            ("prefix: >", Some(0)),
            ("    policy: |-", Some(4)),
            ("    policy: >+", Some(4)),
            ("prefix: |2", Some(0)),
            ("prefix: |2-", Some(0)),
            ("  - |", Some(2)),
            ("prefix: | # a note", Some(0)),
            // A nested sequence entry is STILL a value position — the header
            // narrowing must not trade a false positive for a false negative.
            ("  - - |", Some(2)),
            ("  -   -   >-", Some(2)),
            // NOT headers: a value that merely contains one, a quoted one, a
            // longer token, a bare pipe in no value position, ordinary lines.
            ("prefix: a|b", None),
            ("prefix: \"|\"", None),
            ("prefix: '|'", None),
            ("prefix: |abc", None),
            ("prefix: |12345", None),
            // `|`/`>` are indicators only at the START of a
            // scalar, so each of these is an ordinary PLAIN scalar that merely
            // ends in a dash. A trailing-dash rule would read every one as a header.
            ("  - id: a - |", None),
            ("  - id: a - >", None),
            ("    note: step 1 - |", None),
            ("    topic: /a/b- |", None),
            ("|", None),
            ("nodes:", None),
            ("  - id: n0", None),
            ("", None),
        ] {
            assert_eq!(
                block_scalar_header_indent(line),
                want,
                "header reading of {line:?}"
            );
        }
    }

    #[test]
    fn the_comment_stripper_keeps_a_hash_that_is_not_a_comment() {
        assert_eq!(strip_trailing_comment("prefix: p # note"), "prefix: p ");
        assert_eq!(strip_trailing_comment("# whole line"), "");
        // A `#` inside a token is not a comment...
        assert_eq!(strip_trailing_comment("prefix: a#b"), "prefix: a#b");
        // ...nor is one inside a quoted scalar.
        assert_eq!(
            strip_trailing_comment("prefix: \"a # b\""),
            "prefix: \"a # b\""
        );
        assert_eq!(strip_trailing_comment("prefix: p"), "prefix: p");
    }

    #[test]
    fn the_flow_depth_walk_answers_its_hand_written_vectors() {
        // The predicate the block/flow dispatch turns on, driven directly: a
        // predicate that answers too easily makes its guard vacuous without
        // failing anything.
        let doc = |s: &str| -> Vec<String> { s.lines().map(|l| l.to_string()).collect() };

        let block = doc("a:\n  b: 1\n");
        assert_eq!(flow_depth_at(&block, 1, 2), 0, "an ordinary block key");

        // `a: [{b: 1}]` — the depth is read BEFORE the byte at `col`, so the
        // key `b` (col 5) is inside both, `{` (col 4) is inside the sequence
        // only, and `[` (col 3) is still outside.
        let flow = doc("a: [{b: 1}]\n");
        assert_eq!(flow_depth_at(&flow, 0, 5), 2, "inside a map inside a seq");
        assert_eq!(flow_depth_at(&flow, 0, 4), 1, "inside the seq alone");
        assert_eq!(flow_depth_at(&flow, 0, 3), 0, "the bracket itself");
        assert_eq!(flow_depth_at(&flow, 0, 0), 0, "before anything opens");

        let closed = doc("a: [1]\nb: 2\n");
        assert_eq!(flow_depth_at(&closed, 1, 0), 0, "the collection closed");

        let quoted = doc("a: \"{[\"\nb: 2\n");
        assert_eq!(flow_depth_at(&quoted, 1, 0), 0, "brackets in a string");

        let commented = doc("a: 1 # {[\nb: 2\n");
        assert_eq!(flow_depth_at(&commented, 1, 0), 0, "brackets in a comment");

        // A `#` inside a token is not a comment — so the scan must NOT stop
        // there and miss what follows. Driven INSIDE a flow collection, which
        // is the only place the answer discriminates: a vector of
        // `a: x#{` asserting depth 1 would encode the bracket defect (a
        // brace in a BLOCK-context plain scalar is ordinary text, and the
        // correct answer there is 0), so it would pass for the wrong reason
        // whatever the `#` rule did.
        let hashy = doc("a: [x#{\nb: 2\n");
        assert_eq!(
            flow_depth_at(&hashy, 1, 0),
            2,
            "a mid-token # does not stop the scan: the brace after it still counts"
        );
        // …and its ANTI-TAUTOLOGY twin: a REAL comment (a `#` after
        // whitespace) does stop it, so the same brace is invisible.
        let real_comment = doc("a: [x #{\nb: 2\n");
        assert_eq!(
            flow_depth_at(&real_comment, 1, 0),
            1,
            "a genuine comment hides the brace"
        );

        // `[`, `]`, `{`, `}` are legal INSIDE a plain scalar in BLOCK
        // context, so they are text — not structure. Unbalanced on purpose:
        // counting it left the depth stuck for the rest of the document.
        let plain = doc("name: see [ref-123\nb: 2\n");
        assert_eq!(
            flow_depth_at(&plain, 1, 0),
            0,
            "a bracket in a block-context plain scalar is text"
        );
        // The VALUE POSITION is what separates the two: the same bracket, as
        // the first thing after `key:`, really does open a collection.
        let opener = doc("name: [see, ref-123\nb: 2\n");
        assert_eq!(
            flow_depth_at(&opener, 1, 0),
            1,
            "a bracket in a value position opens a collection"
        );
        // …and after a sequence indicator too.
        let seq = doc("- {a: 1\nb: 2\n");
        assert_eq!(
            flow_depth_at(&seq, 1, 0),
            1,
            "a brace after a sequence indicator opens a collection"
        );
        // A nested mapping on a sequence line is NOT a scalar: its own `key:`
        // still opens a value position, so a brace there counts.
        let nested = doc("- name: {a: 1\nb: 2\n");
        assert_eq!(
            flow_depth_at(&nested, 1, 0),
            1,
            "a brace after a nested mapping key opens a collection"
        );

        let multi = doc("a: [\n  {b: 1}\n]\n");
        assert_eq!(flow_depth_at(&multi, 1, 3), 2, "across a line break");
    }

    #[test]
    fn a_legacy_name_key_survives_because_the_format_still_defines_it() {
        // ANTI-TAUTOLOGY for the strip engine: `name:` is deprecated but
        // ACCEPTED by the format, so a stripper that removed everything
        // deprecated would take it too.
        let yaml = "name: legacy\nnodes: []\npolicy: 1\n";
        let strip = strip_unknown_graph_keys(yaml).expect("strips");
        assert_eq!(
            strip
                .stripped
                .iter()
                .map(|s| s.path.as_str())
                .collect::<Vec<_>>(),
            vec!["policy"]
        );
        assert_eq!(strip.config.name.as_deref(), Some("legacy"));
    }

    #[test]
    fn real_yaml_damage_is_refused_rather_than_rewritten() {
        let err = strip_unknown_graph_keys("nodes: [")
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("NOT a key the format no longer defines"),
            "must say why migration cannot help: {err}"
        );
        assert!(
            !err.contains("removed"),
            "must not imply anything was rewritten: {err}"
        );
    }

    #[test]
    fn a_document_that_already_parses_strips_nothing() {
        let strip = strip_unknown_graph_keys("prefix: p\nnodes: []\n").expect("parses");
        assert!(strip.stripped.is_empty());
        assert_eq!(strip.cleaned_yaml, "prefix: p\nnodes: []");
    }

    // ── the error readers ──────────────────────────────────────────────────

    #[test]
    fn the_key_reader_reads_the_key_and_nothing_else() {
        assert_eq!(
            unknown_field_key("nodes[0]: unknown field `policy`, expected one of `id`").as_deref(),
            Some("policy")
        );
        assert_eq!(unknown_field_key("invalid type: string"), None);
        assert_eq!(unknown_field_key("unknown field ``, expected"), None);
    }

    #[test]
    fn the_path_reader_uses_serdes_own_spelling_and_degrades_to_the_bare_key() {
        assert_eq!(
            unknown_field_path(
                "nodes[0].outputs[0]: unknown field `dpeth`, expected",
                "dpeth"
            ),
            "nodes[0].outputs[0].dpeth"
        );
        // Top level: serde prints no container prefix at all.
        assert_eq!(
            unknown_field_path("unknown field `policy`, expected one of", "policy"),
            "policy"
        );
        // A message shape this reader does not recognise still names the key.
        assert_eq!(unknown_field_path("something else entirely", "k"), "k");
    }

    // ── the blanker ────────────────────────────────────────────────────────

    #[test]
    fn blanking_preserves_line_count_and_stops_at_the_next_sibling() {
        let mut lines: Vec<String> = vec![
            "a:".into(),
            "  b:".into(),
            "    c: 1".into(),
            "".into(),
            "    d: 2".into(),
            "  e: 3".into(),
        ];
        assert!(blank_key_block(&mut lines, 1, 2));
        assert_eq!(
            lines,
            vec![
                "a:".to_string(),
                String::new(),
                String::new(),
                String::new(),
                String::new(),
                "  e: 3".to_string(),
            ],
            "the block goes, the sibling stays, and the line count is unchanged"
        );
    }

    #[test]
    fn blanking_refuses_a_position_that_is_not_a_key() {
        let mut lines: Vec<String> = vec!["  - scalar".into()];
        // Column 4 is `s`, and there is no `:` after it.
        assert!(!blank_key_block(&mut lines, 0, 4));
        assert_eq!(lines[0], "  - scalar", "a refusal must change nothing");

        // Column pointing at whitespace.
        let mut lines2: Vec<String> = vec!["  key: 1".into()];
        assert!(!blank_key_block(&mut lines2, 0, 1));
        assert_eq!(lines2[0], "  key: 1");

        // Column past the end of the line.
        let mut lines3: Vec<String> = vec!["k: 1".into()];
        assert!(!blank_key_block(&mut lines3, 0, 40));
        assert_eq!(lines3[0], "k: 1");
    }

    // ── output-path resolution ─────────────────────────────────────────────

    #[test]
    fn the_default_output_is_the_stem_plus_migrated_beside_the_input() {
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("run_2026.mcap");
        std::fs::write(&input, b"x").unwrap();
        let out = resolve_output_path(&input, None).expect("resolves");
        assert_eq!(out, dir.path().join("run_2026.migrated.mcap"));
    }

    #[test]
    fn an_output_that_is_the_input_is_refused_by_name() {
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("run.mcap");
        std::fs::write(&input, b"x").unwrap();
        // Same file reached by a different spelling.
        let alias = dir.path().join(".").join("run.mcap");
        let err = resolve_output_path(&input, Some(&alias))
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("NEVER rewrites a bag in place"),
            "must state the rule: {err}"
        );
    }

    #[test]
    fn an_existing_output_is_refused_rather_than_overwritten() {
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("run.mcap");
        std::fs::write(&input, b"x").unwrap();
        let out = dir.path().join("already-there.mcap");
        std::fs::write(&out, b"precious").unwrap();
        let err = resolve_output_path(&input, Some(&out))
            .unwrap_err()
            .to_string();
        assert!(err.contains("already exists"), "got: {err}");
        assert_eq!(
            std::fs::read(&out).unwrap(),
            b"precious",
            "the refusal must not have touched it"
        );
    }

    // ── the no-clobber install ─────────────────────────────────────────────

    /// Claim a real scratch beside `output` and write `bytes` THROUGH its
    /// handle — the production shape, so an install test cannot pass against a
    /// scratch whose identity was never established.
    fn write_scratch(output: &Path, bytes: &[u8]) -> ScratchFile {
        use std::io::Write as _;
        let scratch = ScratchFile::claim(output).expect("claims a scratch");
        (&scratch.file)
            .write_all(bytes)
            .expect("writes through the claimed handle");
        scratch
    }

    /// Every scratch file left in `dir`.
    ///
    /// A NAME assertion is no longer available — the scratch tail is
    /// unpredictable on purpose (see [`ScratchFile`]) — so leftovers are swept
    /// by PREFIX. That is strictly stronger than the name check it replaces: it
    /// catches a leftover under ANY scratch name, including one from a retry.
    fn surviving_scratch_files(dir: &Path) -> Vec<String> {
        let mut found: Vec<String> = std::fs::read_dir(dir)
            .expect("reads the dir")
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.starts_with(SCRATCH_PREFIX))
            .collect();
        found.sort();
        found
    }

    #[test]
    fn the_install_refuses_a_destination_that_appeared_after_the_pre_flight() {
        // The state a race PRODUCES, driven at the install itself: the
        // pre-flight said the path was free, and by the time we get here it is
        // not. `rename` would replace it silently.
        let dir = tempfile::tempdir().unwrap();
        let out = dir.path().join("out.mcap");
        let scratch = write_scratch(&out, b"the migrated bag");
        let temp = scratch.path().to_path_buf();
        std::fs::write(&out, b"somebody else's file").unwrap();

        let err = install_no_clobber(&scratch, &out).unwrap_err().to_string();
        assert!(
            err.contains("appeared while this migration was running"),
            "must name the race: {err}"
        );
        assert_eq!(
            std::fs::read(&out).unwrap(),
            b"somebody else's file",
            "the file that was there must be byte-untouched"
        );
        assert!(!temp.exists(), "the discarded scratch must be cleaned up");
    }

    #[test]
    fn the_install_places_the_bag_and_removes_its_scratch() {
        // ANTI-TAUTOLOGY: without this, "the install refuses" is satisfied by
        // an install that refuses everything.
        let dir = tempfile::tempdir().unwrap();
        let out = dir.path().join("out.mcap");
        let scratch = write_scratch(&out, b"the migrated bag");
        let temp = scratch.path().to_path_buf();

        install_no_clobber(&scratch, &out).expect("a free destination installs");
        assert_eq!(std::fs::read(&out).unwrap(), b"the migrated bag");
        assert!(!temp.exists(), "the scratch name must be gone");
    }

    #[test]
    fn a_destination_created_mid_migration_is_refused_by_the_whole_verb() {
        // The FULL production path through the race window: the pre-flight
        // passes (the path really is free when it runs), a writer lands while
        // the bag is being rewritten, and the migration must discard its work
        // rather than replace that writer's file.
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("legacy.mcap");
        write_legacy_bag(&input);
        let out = dir.path().join("legacy.migrated.mcap");
        let before_input = std::fs::read(&input).unwrap();

        let racer = out.clone();
        arm_before_install_hook(Box::new(move || {
            std::fs::write(&racer, b"a file that appeared mid-migration").unwrap();
        }));

        let mut confirm = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
        let err = bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(out.clone()),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(1),
            false,
            &mut confirm,
        )
        .unwrap_err()
        .to_string();

        assert!(
            err.contains("appeared while this migration was running"),
            "must name the race: {err}"
        );
        assert_eq!(
            std::fs::read(&out).unwrap(),
            b"a file that appeared mid-migration",
            "the racer's file must survive byte-identical"
        );
        assert_eq!(
            std::fs::read(&input).unwrap(),
            before_input,
            "the recording must be untouched"
        );
        assert_eq!(
            surviving_scratch_files(dir.path()),
            Vec::<String>::new(),
            "no scratch may be left behind"
        );

        // ...and with no racer, the SAME migration succeeds — so the refusal
        // above is the race and not a broken fixture.
        let clean = dir.path().join("clean-run.mcap");
        let mut confirm2 = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
        bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(clean.clone()),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(1),
            false,
            &mut confirm2,
        )
        .expect("an unraced migration writes");
        assert!(clean.is_file());
    }

    /// A minimal legacy bag: one topic, one frame, a `graph.yaml` carrying
    /// a key the format no longer defines.
    fn write_legacy_bag(path: &Path) {
        let payload = [0xABu8; 8];
        let topics = [cerulion_bag::TopicSchema {
            topic: "/data".into(),
            schema_name: "std_msgs/UInt8".into(),
            schema_hash: 0x0102_0304_0506_0708,
            wire_fixed_size: 8,
        }];
        let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics).unwrap();
        w.write_chunk(|c| c.write_message("/data", 0, 1000, 1000, &[&payload[..]]))
            .unwrap();
        w.write_attachment(
            GRAPH_ATTACHMENT,
            "application/yaml",
            0,
            0,
            b"prefix: p\nnodes:\n  - id: n0\n    type: t0\n    policy: 1\n",
        )
        .unwrap();
        w.finalize().unwrap();
    }

    /// `write_legacy_bag`, plus an attachment of the caller's choosing — the
    /// shape a bag that has ALREADY been through a migration has.
    fn write_legacy_bag_with_attachment(path: &Path, name: &str, data: &[u8]) {
        let payload = [0xABu8; 8];
        let topics = [cerulion_bag::TopicSchema {
            topic: "/data".into(),
            schema_name: "std_msgs/UInt8".into(),
            schema_hash: 0x0102_0304_0506_0708,
            wire_fixed_size: 8,
        }];
        let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics).unwrap();
        w.write_chunk(|c| c.write_message("/data", 0, 1000, 1000, &[&payload[..]]))
            .unwrap();
        w.write_attachment(
            GRAPH_ATTACHMENT,
            "application/yaml",
            0,
            0,
            b"prefix: p\nnodes:\n  - id: n0\n    type: t0\n    policy: 1\n",
        )
        .unwrap();
        w.write_attachment(name, MIGRATION_MEDIA_TYPE, 0, 0, data)
            .unwrap();
        w.finalize().unwrap();
    }

    // ── the scratch file's identity ────────────────────────────────────────

    #[test]
    fn a_substituted_scratch_name_is_neither_ours_nor_deleted_by_our_cleanup() {
        // The three guarantees `ScratchFile` makes, driven directly. The
        // attack unlinks the scratch NAME and puts its own file
        // there; from that instant the name resolves to somebody else's inode,
        // and BOTH of our uses of it — publish and clean up — must notice.
        let dir = tempfile::tempdir().unwrap();
        let out = dir.path().join("out.mcap");
        let scratch = write_scratch(&out, b"the migrated bag");
        let path = scratch.path().to_path_buf();

        // Our own file, before anything happens to it.
        assert!(scratch.is_ours(&path), "the claimed name is ours");

        // The substitution, exactly as the harness performed it.
        std::fs::remove_file(&path).unwrap();
        std::fs::write(&path, b"ATTACKER-SELECTED-BYTES").unwrap();

        assert!(
            !scratch.is_ours(&path),
            "a replaced name must not read as our file — that is the whole check"
        );
        assert!(
            !scratch.published_at(&path),
            "…and it must not be publishable as our migrated bag"
        );

        // Cleanup must not unlink a path that is no longer ours: the file there
        // belongs to whoever wrote it, exactly like the destination on the
        // `AlreadyExists` arm.
        scratch.remove_if_ours();
        assert_eq!(
            std::fs::read(&path).unwrap(),
            b"ATTACKER-SELECTED-BYTES",
            "the substitute must survive our cleanup byte-identical"
        );

        // ANTI-TAUTOLOGY: cleanup of a scratch that IS still ours does remove
        // it — without this, "cleanup does not delete" passes a cleanup that
        // never deletes anything.
        let out2 = dir.path().join("out2.mcap");
        let mine = write_scratch(&out2, b"mine");
        let mine_path = mine.path().to_path_buf();
        mine.remove_if_ours();
        assert!(!mine_path.exists(), "our own scratch is cleaned up");
    }

    #[test]
    fn a_scratch_replaced_before_the_install_is_refused_not_published() {
        // THE attack, through the FULL production verb. An install that
        // resolves the scratch NAME again at `hard_link` lets a
        // process that can write to the output directory replace the file in
        // that window, and the migration publishes ATTACKER-SELECTED-BYTES at the
        // requested output while returning `Written`.
        //
        // The racer SCANS for the scratch rather than being told its name: the
        // tail is unpredictable now, and an attacker who cannot find it cannot
        // demonstrate anything. Finding it is the realistic capability.
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("legacy.mcap");
        write_legacy_bag(&input);
        let out = dir.path().join("legacy.migrated.mcap");
        let before_input = std::fs::read(&input).unwrap();

        let scan_dir = dir.path().to_path_buf();
        arm_before_install_hook(Box::new(move || {
            let names = surviving_scratch_files(&scan_dir);
            assert_eq!(
                names.len(),
                1,
                "exactly one scratch to substitute: {names:?}"
            );
            let victim = scan_dir.join(&names[0]);
            // …and it really is the finished bag at this instant, so the
            // substitution is replacing something worth publishing.
            assert!(
                std::fs::read(&victim).unwrap().starts_with(b"\x89MCAP"),
                "the scratch must hold the written bag before it is replaced"
            );
            std::fs::remove_file(&victim).unwrap();
            std::fs::write(&victim, b"ATTACKER-SELECTED-BYTES").unwrap();
        }));

        let mut confirm = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
        let err = bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(out.clone()),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(1),
            false,
            &mut confirm,
        )
        .unwrap_err()
        .to_string();

        // The refusal must name BOTH reachable causes, not diagnose one: the
        // scratch replaced BEFORE the link, or the output replaced AFTER it.
        // A message confidently naming the first would be wrong for a
        // condition either can produce.
        assert!(
            err.contains("is not the file this migration wrote"),
            "must say what is actually wrong: {err}"
        );
        assert!(
            err.contains("was replaced before it was put in place")
                && err.contains("was replaced immediately after"),
            "must name BOTH possibilities rather than diagnosing one: {err}"
        );
        assert!(
            !out.exists(),
            "nothing may be published at the requested output — an install \
             that trusted the name would publish ATTACKER-SELECTED BYTES"
        );
        assert_eq!(
            std::fs::read(&input).unwrap(),
            before_input,
            "the recording must be untouched"
        );

        // ANTI-TAUTOLOGY: with no racer, the SAME migration succeeds and
        // publishes a real bag — so the refusal above is the substitution and
        // not a broken fixture.
        let clean = dir.path().join("clean-run.mcap");
        let mut confirm2 = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
        bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(clean.clone()),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(1),
            false,
            &mut confirm2,
        )
        .expect("an unraced migration writes");
        assert!(std::fs::read(&clean).unwrap().starts_with(b"\x89MCAP"));

        // Exactly ONE scratch-named file survives the pair of runs, and it is
        // the racer's — our cleanup refused to unlink a name that was no longer
        // ours, exactly as it refuses to overwrite a destination that is not.
        // The clean run left nothing, so this also pins that the second
        // migration cleaned up after itself.
        let left = surviving_scratch_files(dir.path());
        assert_eq!(left.len(), 1, "only the substitute may survive: {left:?}");
        assert_eq!(
            std::fs::read(dir.path().join(&left[0])).unwrap(),
            b"ATTACKER-SELECTED-BYTES",
            "and it must be byte-identical — deleting somebody else's file is \
             not this verb's to do"
        );
    }

    #[test]
    fn the_bag_is_written_through_the_claimed_handle_never_the_name() {
        // The OTHER half, and the half no other arm can see: on an
        // unmolested filesystem, writing through the handle and re-opening the
        // path produce the same file, so a path-resolving writer passes every
        // behavioural arm in this module.
        //
        // Substituting the name between the CLAIM and the first byte separates
        // them. Correct: the writer holds the inode it claimed, so the migrated
        // bag lands there and the substitute is never touched. A writer that
        // re-resolves the path: `File::create` TRUNCATES the substitute and
        // writes the whole bag into somebody else's file — a data-destroying
        // difference that a "did the migration succeed?" oracle cannot see,
        // since BOTH spellings then fail the install for the same reason (the
        // name no longer resolves to our inode).
        //
        // So the oracle is the SUBSTITUTE's bytes, not the verb's verdict.
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("legacy.mcap");
        write_legacy_bag(&input);
        let out = dir.path().join("legacy.migrated.mcap");

        const SQUATTER: &[u8] = b"A FILE THAT IS NOT OURS";
        arm_after_claim_hook(Box::new(|scratch: &Path| {
            // The window a claim-then-reopen writer leaves open.
            std::fs::remove_file(scratch).unwrap();
            std::fs::write(scratch, SQUATTER).unwrap();
        }));

        let mut confirm = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
        let err = bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(out.clone()),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(1),
            false,
            &mut confirm,
        )
        .unwrap_err()
        .to_string();
        assert!(
            err.contains("is not the file this migration wrote"),
            "the install still refuses, whichever way the bytes were written: {err}"
        );
        assert!(!out.exists(), "and nothing is published");

        let left = surviving_scratch_files(dir.path());
        assert_eq!(
            left.len(),
            1,
            "the substitute is the only leftover: {left:?}"
        );
        assert_eq!(
            std::fs::read(dir.path().join(&left[0])).unwrap(),
            SQUATTER,
            "THE PIN: the migration's bytes went to the inode it claimed, so the \
             file now at that name is untouched. A writer that re-resolved the \
             path would have truncated it and written the bag here."
        );
    }

    #[test]
    fn a_dangling_symlink_at_the_output_is_named_for_what_it_is() {
        // `resolve_output_path` asks `exists()`, which FOLLOWS
        // symlinks — so a symlink pointing at nothing answers false and sails
        // through the pre-flight. `link(2)` then reports EEXIST for the symlink
        // ENTRY, and a message saying the file "appeared while this migration
        // was running" would be about something that predated the run by however long.
        // Not a corruption; a refusal that sends an operator looking for a
        // concurrent writer that does not exist.
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("legacy.mcap");
        write_legacy_bag(&input);
        let out = dir.path().join("legacy.migrated.mcap");
        std::os::unix::fs::symlink(dir.path().join("nowhere.mcap"), &out).unwrap();
        assert!(!out.exists(), "the fixture must be a DANGLING symlink");

        let mut confirm = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
        let err = bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(out.clone()),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(1),
            false,
            &mut confirm,
        )
        .unwrap_err()
        .to_string();

        assert!(
            err.contains("symlink that points at nothing")
                && err.contains("before this migration started"),
            "must name the symlink and its age: {err}"
        );
        assert!(
            !err.contains("appeared while this migration was running"),
            "and must NOT blame a concurrent writer that does not exist: {err}"
        );
        // ANTI-TAUTOLOGY: a genuine mid-run arrival still gets the other
        // sentence, so the symlink arm is a real discrimination rather than a
        // blanket reword.
        let out2 = dir.path().join("racy.mcap");
        let racer = out2.clone();
        arm_before_install_hook(Box::new(move || {
            std::fs::write(&racer, b"a file that appeared mid-migration").unwrap();
        }));
        let mut confirm2 = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
        let err2 = bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(out2),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(1),
            false,
            &mut confirm2,
        )
        .unwrap_err()
        .to_string();
        assert!(
            err2.contains("appeared while this migration was running"),
            "a real mid-run arrival keeps its own diagnosis: {err2}"
        );
        assert!(!err2.contains("symlink"), "{err2}");
    }

    #[test]
    fn the_provenance_describes_the_bytes_the_migration_actually_read() {
        // The record's whole point is checkability — "given the
        // original file, anyone can confirm this really descends from it" — and
        // its hash, size and the published mode were all read by RE-RESOLVING
        // the input NAME, seconds after `BagReader::open` mapped the inode the
        // frames are copied from. The default output sits beside the input, so
        // the attacker capability this file already reproduces ("a process that
        // can write to the output directory") covers the input's directory too.
        //
        // Replacing the name in that window would stamp the DECOY's hash and
        // size into a bag whose every byte came from the original, and publish
        // it under the decoy's mode: verification against the true original
        // would FAIL and against an attacker-chosen file SUCCEED — the field
        // inverted rather than merely wrong.
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("legacy.mcap");
        write_legacy_bag(&input);
        std::fs::set_permissions(&input, std::fs::Permissions::from_mode(0o640)).unwrap();
        let original_bytes = std::fs::read(&input).unwrap();
        let original_sha = sha256_hex(&original_bytes);

        // A decoy that is a REAL migratable bag, so nothing downstream can
        // reject it for being malformed — the difference has to be visible only
        // in the provenance.
        let decoy_src = dir.path().join("decoy-src.mcap");
        write_legacy_bag(&decoy_src);
        std::fs::write(
            &decoy_src,
            [std::fs::read(&decoy_src).unwrap(), vec![0u8; 64]].concat(),
        )
        .unwrap();
        let decoy_bytes = std::fs::read(&decoy_src).unwrap();
        assert_ne!(decoy_bytes, original_bytes, "the decoy must differ");
        let decoy_sha = sha256_hex(&decoy_bytes);

        let stash = dir.path().join("original-moved-aside.mcap");
        arm_after_open_hook(Box::new(move |path: &Path| {
            // The original is renamed aside and the decoy takes its NAME —
            // exactly what a concurrent writer in that directory can do.
            std::fs::rename(path, &stash).unwrap();
            std::fs::write(path, &decoy_bytes).unwrap();
            std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o666)).unwrap();
        }));

        let out = dir.path().join("legacy.migrated.mcap");
        let mut confirm = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
        bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(out.clone()),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(1),
            false,
            &mut confirm,
        )
        .expect("migrates");

        let reader = BagReader::open(&out).unwrap();
        let att = reader
            .attachment(MIGRATION_ATTACHMENT)
            .unwrap()
            .expect("the record is there");
        let record: MigrationRecord = serde_json::from_slice(&att.data).unwrap();
        assert_eq!(
            record.source_bag_sha256, original_sha,
            "the record must describe the bytes the migration READ, not whatever \
             the input name resolves to now"
        );
        assert_ne!(
            record.source_bag_sha256, decoy_sha,
            "…and specifically not the decoy's — without this the assertion \
             above could pass on two identical files"
        );
        assert_eq!(record.source_bag_size_bytes, original_bytes.len() as u64);
        assert_eq!(
            std::fs::metadata(&out).unwrap().permissions().mode() & 0o777,
            0o640,
            "the published mode is the RECORDING's, not the decoy's 0666"
        );
    }

    #[test]
    fn the_migrated_bag_carries_the_recordings_own_permissions() {
        // The scratch is created 0600 — a transient whole copy of a bag, whose
        // attachments include the recorder's `env.json`, has no business being
        // world-readable while it is written. But the install is a HARD LINK,
        // so the scratch's mode BECOMES the output's: left alone, every
        // migrated bag would come out stricter than the recording it descends
        // from and stricter than `cerulion_bagd` writes one, which is a
        // user-facing change nobody asked for.
        //
        // Driven at TWO different input modes, because a single one is
        // satisfied by any implementation that happens to hardcode it.
        use std::os::unix::fs::PermissionsExt as _;
        for mode in [0o644, 0o640] {
            let dir = tempfile::tempdir().unwrap();
            let input = dir.path().join("legacy.mcap");
            write_legacy_bag(&input);
            std::fs::set_permissions(&input, std::fs::Permissions::from_mode(mode)).unwrap();
            let out = dir.path().join("legacy.migrated.mcap");

            let mut confirm = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
            bag_migrate(
                &input,
                &MigrateOptions {
                    out: Some(out.clone()),
                    dry_run: false,
                    assume_yes: true,
                },
                MigrationStamp::at_epoch_ns(1),
                false,
                &mut confirm,
            )
            .expect("migrates");

            assert_eq!(
                std::fs::metadata(&out).unwrap().permissions().mode() & 0o777,
                mode,
                "the migrated bag must be readable by exactly whoever could read \
                 the recording (input mode {mode:o})"
            );
        }
    }

    // ── one provenance record per bag ──────────────────────────────────────

    #[test]
    fn a_bag_that_already_carries_a_migration_record_is_refused_by_name() {
        // MCAP attachment names are NOT unique and `BagReader::attachment`
        // answers with the FIRST match, so appending a second record hides
        // behind the first: every reader of the provenance would get the OLDER
        // migration's source hashes and stripped-key list for a bag they do not
        // describe. Refused, naming the record that is already there.
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("already.mcap");
        let record = serde_json::to_vec(&MigrationRecord {
            version: MIGRATION_RECORD_VERSION,
            tool: "cerulion bag migrate".to_string(),
            migrated_at_ns: 1_700_000_000_000_000_000,
            source_bag_file_name: "original.mcap".to_string(),
            source_bag_size_bytes: 4096,
            source_bag_sha256: "aa".repeat(32),
            source_graph_yaml_sha256: "bb".repeat(32),
            migrated_graph_yaml_sha256: "cc".repeat(32),
            stripped: vec![StrippedKey {
                key: "policy".into(),
                path: "nodes[0].policy".into(),
                line: 5,
            }],
        })
        .unwrap();
        write_legacy_bag_with_attachment(&input, MIGRATION_ATTACHMENT, &record);
        let out = dir.path().join("again.mcap");

        let mut confirm = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
        let err = bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(out.clone()),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(2),
            false,
            &mut confirm,
        )
        .unwrap_err()
        .to_string();

        assert!(
            err.contains("already carries a `__cerulion/migration.json` attachment"),
            "must name the attachment: {err}"
        );
        assert!(
            err.contains("original.mcap") && err.contains("1700000000000000000"),
            "must name the EXISTING record so an operator can tell which one: {err}"
        );
        assert!(!out.exists(), "nothing may be written");
        assert_eq!(
            surviving_scratch_files(dir.path()),
            Vec::<String>::new(),
            "and no scratch either — the refusal is a pre-flight"
        );
    }

    #[test]
    fn the_already_migrated_refusal_reports_an_unreadable_record_as_unreadable() {
        // The record's bytes come from a file this build did not write, so an
        // unparseable one must not turn a clear refusal into a parse error —
        // and must not be silently skipped either, since "there is a record
        // here and it is damaged" is the fact the operator needs.
        assert_eq!(
            describe_existing_migration(b"{ not json"),
            " (whose contents this build cannot parse)"
        );

        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("damaged-record.mcap");
        write_legacy_bag_with_attachment(&input, MIGRATION_ATTACHMENT, b"{ not json");
        let mut confirm = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
        let err = bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(dir.path().join("again.mcap")),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(2),
            false,
            &mut confirm,
        )
        .unwrap_err()
        .to_string();
        assert!(
            err.contains("already carries") && err.contains("cannot parse"),
            "got: {err}"
        );
    }

    #[test]
    fn an_unrelated_attachment_does_not_trip_the_already_migrated_refusal() {
        // ANTI-TAUTOLOGY: without this, "a bag with an attachment is refused"
        // is satisfied by a check keyed on the wrong thing — and every
        // production bag carries `env.json`, `recorder.json` and friends.
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("ordinary.mcap");
        write_legacy_bag_with_attachment(&input, "env.json", b"{}");
        let out = dir.path().join("ordinary.migrated.mcap");
        let mut confirm = |_: &str| -> CliResult<bool> { unreachable!("--yes was passed") };
        let report = bag_migrate(
            &input,
            &MigrateOptions {
                out: Some(out.clone()),
                dry_run: false,
                assume_yes: true,
            },
            MigrationStamp::at_epoch_ns(2),
            false,
            &mut confirm,
        )
        .expect("an ordinary legacy bag migrates");
        assert_eq!(report.outcome, MigrateOutcome::Written);
        assert!(out.exists());

        // …and the product carries EXACTLY ONE provenance record, which is the
        // invariant the refusal above protects.
        let reader = BagReader::open(&out).unwrap();
        let atts = reader.attachments().unwrap();
        assert_eq!(
            atts.iter()
                .filter(|a| a.name == MIGRATION_ATTACHMENT)
                .count(),
            1
        );
        assert_eq!(atts.iter().filter(|a| a.name == "env.json").count(), 1);
    }

    // ── the timestamp seam ─────────────────────────────────────────────────

    #[test]
    fn a_fixed_stamp_round_trips_and_now_reads_the_clock() {
        assert_eq!(MigrationStamp::at_epoch_ns(42).epoch_ns(), 42);
        // Loose: the only claim is that `now()` is a real clock read, i.e. it
        // is somewhere after 2020. A tighter bound would be timing, not a
        // contract.
        assert!(MigrationStamp::now().epoch_ns() > 1_577_836_800_000_000_000);
    }

    // ── the preview ────────────────────────────────────────────────────────

    #[test]
    fn the_preview_names_both_paths_every_stripped_key_and_the_never_modified_rule() {
        let preview = render_preview(
            Path::new("/bags/in.mcap"),
            Path::new("/bags/in.migrated.mcap"),
            &[
                StrippedKey {
                    key: "policy".into(),
                    path: "nodes[0].policy".into(),
                    line: 5,
                },
                StrippedKey {
                    key: "dpeth".into(),
                    path: "nodes[0].outputs[0].dpeth".into(),
                    line: 11,
                },
            ],
        );
        assert!(preview.contains("/bags/in.mcap"));
        assert!(preview.contains("/bags/in.migrated.mcap"));
        assert!(preview.contains("nodes[0].policy"));
        assert!(preview.contains("nodes[0].outputs[0].dpeth"));
        assert!(preview.contains("line 5"));
        assert!(preview.contains("line 11"));
        assert!(preview.contains("2 key(s)"));
        assert!(preview.contains("input bag is\nnever modified"));
        assert!(preview.contains(MIGRATION_ATTACHMENT));
    }

    // ── the channel-fidelity gate ──────────────────────────────────────────

    fn chan(topic: &str, descriptor: Option<cerulion_bag::SchemaDescriptor>) -> BagChannel {
        BagChannel {
            id: 0,
            topic: topic.to_string(),
            schema_name: "pkg/Msg".to_string(),
            schema_encoding: SCHEMA_ENCODING.to_string(),
            message_encoding: SCHEMA_ENCODING.to_string(),
            descriptor,
            provisioning: cerulion_bag::ChannelProvisioning::default(),
        }
    }

    #[test]
    fn reserved_channels_are_left_to_the_writer_and_user_channels_are_carried() {
        let chans = vec![
            chan(
                "/camera",
                Some(cerulion_bag::SchemaDescriptor::new(0xAB, 12)),
            ),
            chan(
                cerulion_bag::SCHEDULER_TRACE_TOPIC,
                Some(cerulion_bag::SchemaDescriptor::new(0, 40)),
            ),
        ];
        let got = user_topic_schemas(&chans).expect("ok");
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].topic, "/camera");
        assert_eq!(got[0].schema_hash, 0xAB);
        assert_eq!(got[0].wire_fixed_size, 12);
        assert_eq!(got[0].schema_name, "pkg/Msg");
    }

    #[test]
    fn a_descriptor_less_channel_is_refused() {
        let mut c = chan("/foreign", None);
        c.schema_encoding = "ros2msg".to_string();
        let err = user_topic_schemas(&[c]).unwrap_err().to_string();
        assert!(err.contains("no Cerulion schema descriptor"), "got: {err}");
    }

    #[test]
    fn a_legacy_hash_recipe_channel_is_refused_rather_than_re_stamped() {
        let mut d = cerulion_bag::SchemaDescriptor::new(0xAB, 12);
        d.hash_recipe = cerulion_core::trace::bag::HASH_RECIPE - 1;
        let err = user_topic_schemas(&[chan("/old", Some(d))])
            .unwrap_err()
            .to_string();
        assert!(err.contains("hash recipe"), "got: {err}");
        assert!(
            err.contains("did not compute them"),
            "must say what re-stamping would claim: {err}"
        );
    }

    #[test]
    fn a_foreign_encoding_channel_is_refused() {
        let mut c = chan("/foreign", Some(cerulion_bag::SchemaDescriptor::new(1, 1)));
        c.message_encoding = "cdr".to_string();
        let err = user_topic_schemas(&[c]).unwrap_err().to_string();
        assert!(err.contains("message encoding 'cdr'"), "got: {err}");
    }
}
