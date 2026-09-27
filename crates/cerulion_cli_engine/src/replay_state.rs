//! Reading a recording's node-state ANCHORS.
//!
//! The recorder puts anchors in the bag (`__cerulion/state`, plus the
//! `__cerulion/state_coverage.json` manifest that relates a record's
//! `node_idx` to a node id). The pure decision layer is
//! [`cerulion_core::state_restore`]. This module is the READER between them:
//! it turns a mapped bag into the [`AnchorFact`]s `plan_restore` judges and the
//! framed blobs `GraphRuntime::restore_node_states` applies.
//!
//! # Absent means FROM START, and that is the whole back-compat story
//!
//! Every pre-anchor bag carries no state channel, no state records and no
//! coverage manifest. [`read_bag_anchors`] returns [`BagAnchors::default`] for
//! all three, `plan_restore` sees a recording that begins at step 0, and the
//! answer is [`RestorePlan::FromStart`](cerulion_core::state_restore::RestorePlan::FromStart)
//! — byte-for-byte the replay a bag without anchors has always had.
//!
//! # What is REFUSED rather than guessed
//!
//! Two shapes cannot be read and are named instead of worked around:
//!
//! - **`rings_declared > 1`.** From state record format version 1 a state
//!   record carries its producer's RANK, and the coverage manifest carries the
//!   ring to rank join, so a recording of that format does say which ring a
//!   record came from. What is still rankless is this READER: its
//!   index table is keyed by `node_idx` alone, and [`StateAssembler`] groups
//!   parts by `(run_id, step, node_idx)`, which carries no rank either. Every
//!   ring numbers its own nodes from 0, so with two declared rings two
//!   producers' `node_idx = 0` records would collide inside the assembler
//!   before the manifest's join could be consulted at all. Refusing is an OVER
//!   refusal and never a wrong answer; guessing would apply one node's recorded
//!   state to a DIFFERENT node and then report a confident divergence about it.
//!   Teaching the reader to carry the rank through is a read side change of its
//!   own, so the remedy today is a recording of a single rank.
//! - **Several runs' anchors at the resume step.** A machine-wide recording
//!   legitimately carries more than one run, and picking one is picking which
//!   execution the verdict is about. `plan_restore` takes the run as an INPUT
//!   for exactly this reason, so the ambiguity is resolved here — by refusing.
//!
//! # The assembler's mode is READ, never assumed
//!
//! [`StateAssembler::armed`] discards records until the first `part == 0`,
//! which is correct for a recorder that attached MID-RUN (its ring cursor
//! landed inside somebody's blob) and WRONG for one that read from the start
//! (it would silently eat a whole anchor's head and report the rest as torn).
//! The manifest states which happened (`attached_mid_run`), so the mode is
//! chosen from that bit and never inferred from the record stream.

use std::collections::{BTreeMap, BTreeSet};

use cerulion_bag::{BagReader, STATE_TOPIC};
use cerulion_bagd::{StateCoverage, STATE_COVERAGE_ATTACHMENT};
use cerulion_core::state_restore::{AnchorFact, AnchorOutcome};
use cerulion_core::state_ring::{StateAnchorEvent, StateAssembler};

/// Everything a bag says about the node-state anchors it carries.
///
/// `Default` is the pre-anchor reading (no manifest, no records, no facts)
/// and is what every existing bag produces.
#[derive(Debug, Default)]
pub struct BagAnchors {
    /// One fact per (run, step, node) the bag's state records describe.
    pub facts: Vec<AnchorFact>,
    /// The reassembled anchor payloads, keyed `(run_id, step)` then node id.
    ///
    /// Only COMPLETE anchors appear: a torn or skipped one has a
    /// [`AnchorFact`] (so `plan_restore` can name it) and no bytes (so nothing
    /// can apply it by accident).
    pub blobs: BTreeMap<(u64, u64), BTreeMap<String, Vec<u8>>>,
    /// Whether the bag carried a `__cerulion/state_coverage.json` at all.
    ///
    /// Distinguishes "this recorder was not configured for checkpoints" from
    /// "it was, and found nothing" — the two need different remedies and only
    /// the manifest's presence separates them.
    pub coverage_present: bool,
    /// Leading records the reader discarded as a mid-run attach's partial head.
    pub head_records_discarded: u64,
    /// Records the reader could not key at all.
    pub malformed_records: u64,
    /// Records whose `node_idx` no manifest entry names.
    ///
    /// Reported rather than dropped: an anchor that cannot be attributed is
    /// exactly how a node's state goes missing without anyone being told, and
    /// `plan_restore` will refuse the node as `Missing` with no idea why.
    pub unattributable_records: u64,
}

impl BagAnchors {
    /// The framed blobs for `(run_id, step)`, or an empty map.
    pub fn blobs_at(&self, run_id: u64, step: u64) -> BTreeMap<String, Vec<u8>> {
        self.blobs.get(&(run_id, step)).cloned().unwrap_or_default()
    }

    /// The distinct run ids whose anchors describe `step`, ascending.
    ///
    /// The resume step is fixed by the trace (`first_recorded_step - 1`), so
    /// the only remaining freedom is WHICH run's anchor at that step to apply
    /// — and more than one is an ambiguity the caller must refuse.
    pub fn runs_at_step(&self, step: u64) -> Vec<u64> {
        self.facts
            .iter()
            .filter(|f| f.step == step)
            .map(|f| f.run_id)
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect()
    }
}

/// Why a bag's anchors cannot be read.
///
/// Deliberately a small closed set: everything else about an anchor is a
/// DECISION and belongs to [`cerulion_core::state_restore`], which already owns
/// the vocabulary for it.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum AnchorReadRefusal {
    /// The bag declares more than one state ring, so `node_idx` is ambiguous.
    ///
    /// The rank clause is a property of the FORMAT, and the version in it is a
    /// literal 1 rather than `STATE_RECORD_FORMAT_VERSION`: rank arrived on the
    /// record at version 1 and stays arrived whatever this build reads, so
    /// templating the number would make the sentence claim the property began
    /// wherever the constant happens to sit.
    ///
    /// The claim about the BAG AT HAND is the second clause, and it branches on
    /// what the manifest proves. A manifest naming no state record format is a
    /// recording from before the key existed, and telling its operator that its
    /// records carry a rank is the same false sentence one bag population over.
    ///
    /// COMPATIBILITY: this variant GAINED the `state_record_format` field.
    /// [`AnchorReadRefusal`] is public and is not `non_exhaustive`, so a struct
    /// pattern on this variant written outside this crate must account for the
    /// new field: a brace pattern ending in `..` is unaffected, and a pattern
    /// naming only `rings` stops compiling. A struct variant has no path only
    /// pattern form, so there is no third shape to exempt here, and an
    /// identifier pattern that binds the refusal without destructuring it never
    /// reads a field at all.
    #[error(
        "this recording drained {rings} state rings. From state record format version 1 a state \
         record carries its producer's rank and the coverage manifest carries the ring to rank \
         join, so a recording of that format does say which ring each record came from; {}. \
         What is rankless in either case is this READER: its index table is keyed by node index \
         alone, and its assembler groups parts by run, step and node index, and neither of those \
         keys carries a rank. Every ring numbers its own nodes from 0, so two producers' first \
         nodes would collide before any join could be consulted, and applying one node's recorded \
         state to another would produce a confident divergence report about an execution that \
         never happened. Fix: replay a recording of a single rank (`cerulion graph run --record \
         --single-process`)",
        match state_record_format {
            Some(v) => format!("this recording's manifest names state record format version {v}"),
            None => "this recording's manifest names no state record format at all, so it makes \
                     no such claim about its own records"
                .to_string(),
        }
    )]
    MultiRingAmbiguous {
        /// How many rings the manifest declares.
        rings: usize,
        /// The state record format the manifest names, or `None` when it names
        /// none, which is every recording from before the key existed.
        state_record_format: Option<u32>,
    },

    /// The anchors at the resume step belong to more than one run.
    #[error(
        "this recording carries anchors from {} runs at step {step} ({}), so which execution the \
         replay resumes is ambiguous — and choosing one silently decides which run the verdict is \
         about. Fix: replay a recording of a single run, or record with `cerulion bag record \
         --run=<RUN>` so the bag is bound to one",
        runs.len(),
        runs.iter().map(|r| format!("{r:#018x}")).collect::<Vec<_>>().join(", ")
    )]
    AmbiguousRun {
        /// The resume step (`first_recorded_step - 1`).
        step: u64,
        /// Every run id whose anchor describes that step.
        runs: Vec<u64>,
    },

    /// The bag's records predate the state record format this build reads.
    #[error(
        "this recording's state records were written under {} and this build reads state record \
         format version {known}. The layouts differ: the header is a different width, so every \
         payload would be taken from the wrong offset, and a state record carries no checksum and \
         no version of its own for a reader to catch that on. Applying those bytes would restore a \
         node from data that is not its state and report a confident divergence about an execution \
         that never happened. Fix: recorded before state record format {known}; re-record",
        match carried {
            Some(v) => format!("state record format version {v}"),
            None => "a state record format this bag does not name".to_string(),
        }
    )]
    StateRecordFormatTooOld {
        /// The version the bag's manifest carries, or `None` when it names none.
        carried: Option<u32>,
        /// The version this build reads.
        known: u32,
    },

    /// The bag's records were written under a LATER state record format than
    /// this build reads.
    ///
    /// Its own arm rather than a reuse of
    /// [`StateRecordFormatTooOld`](Self::StateRecordFormatTooOld), because the
    /// REMEDY is the opposite one: a bag from an earlier format has to be
    /// re-recorded, and a bag from a later format has to be read by a later
    /// build. An operator handed the wrong one of those two sentences throws
    /// away a recording that was never damaged.
    #[error(
        "this recording's state records were written under state record format version {carried} \
         and this build reads state record format version {known}. A later format is free to lay \
         the record out differently, and a state record carries no checksum for a reader to catch \
         a decode from the wrong offset on, so assembling these bytes could restore a node from \
         data that is not its state and report a confident divergence about an execution that \
         never happened. The recording is not damaged and does not need re-recording. Fix: read \
         it with a build that reads state record format version {carried}"
    )]
    StateRecordFormatTooNew {
        /// The version the bag's manifest carries, above this build's.
        carried: u32,
        /// The version this build reads.
        known: u32,
    },

    /// The restore points in this bag were selected by more than one capture.
    #[error(
        "this capture's restore points name {} different capture events ({}), so the ranks were \
         cut at two unrelated instants and a resume from them would start the graph from two \
         different moments at once: every cross-rank edge between them would be replayed against \
         state that never coexisted, and the divergence report would be about an execution that \
         never happened. Fix: resume from a bag whose ranks were all selected by one capture, or \
         re-record",
        captures.len(),
        captures
            .iter()
            .map(|(seq, ranks)| format!(
                "capture {seq} carries rank(s) {}",
                ranks.iter().map(u32::to_string).collect::<Vec<_>>().join(", ")
            ))
            .collect::<Vec<_>>()
            .join("; ")
    )]
    MixedCaptureIdentity {
        /// Every capture number the restore points name, each with the ranks
        /// that carry it, in capture order.
        ///
        /// BOTH halves are quoted in the message. The numbers alone say the set
        /// is mixed; the ranks are what tells an operator which half of their
        /// graph came from where, which is the difference between a message they
        /// can act on and one they can only believe.
        captures: Vec<(u64, Vec<u32>)>,
    },
}

/// Read the OPTIONAL `__cerulion/state_coverage.json` manifest.
///
/// The three arms mirror [`crate::replay_engine`]'s `record_coverage` reader,
/// and for the same reason — but the CONSEQUENCE of the failure arms is
/// different and worth naming, because here it is not benign. An unreadable
/// state manifest means the `node_idx → node id` table is gone, so every state
/// record becomes unattributable; the reader reports that count and the
/// restore then refuses per node with `Missing`. That is loud in the right
/// place (the operator learns their run's nodes have no anchor) and the `warn!`
/// is what connects it to the manifest rather than to the recorder.
///
/// # A manifest from a NEWER bagd is READ, not refused
///
/// `version` is compared and NAMED, never used as a gate. That is the same
/// rule `replay_engine`'s `record_coverage` reader follows
/// (pinned by
/// `replay_engine_test::a_coverage_manifest_from_a_newer_bagd_is_read_and_the_skew_is_named`):
/// reading it is the right call, because refusing a readable bag on a version
/// bump turns every additive field into a compatibility break — and additive is
/// exactly what the manifest fields `mirrors_established`, `prefix_lost` and
/// `run_binding` are (each carries NO
/// version bump, so old readers keep working).
///
/// The reason that is SAFE here — and it is a property of this reader, not an
/// assumption — is that every field a newer manifest could change the meaning
/// of lands on a REFUSAL rather than a confident restore:
///
/// - `rings_declared` gates [`AnchorReadRefusal::MultiRingAmbiguous`]. A v2 that
///   RELAXES multi-ring (by keying this reader on the rank its records already
///   carry, which is the residual that refusal itself names) makes this build
///   refuse a bag a newer one could read. Over-refusal, never a wrong answer.
/// - `attached_mid_run` selects the assembler mode, and BOTH readings of it
///   refuse: armed-on-a-from-start eats the head and reports the node uncovered,
///   passthrough-on-a-mid-run reports the headless tail `Torn`. The discriminator
///   is WHICH refusal, which is exactly what the two arms
///   `a_mid_run_attach_discards_the_partial_head_it_landed_inside_and_serves_the_next_anchor`
///   and `a_from_start_recording_missing_an_anchors_head_is_reported_torn_not_discarded`
///   pin.
/// - `nodes[*].node_idx` is written by the SAME recorder that wrote the records
///   it keys, so an ADDITIVE field cannot desynchronise the two halves.
///
/// **The one change that would oblige a gate** is therefore a v2 that redefines
/// what `node_idx` MEANS while keeping its name and shape — because a
/// mis-attribution is the only outcome here that is a confident WRONG claim
/// rather than a refusal, and this module's own `MultiRingAmbiguous` text says
/// what that costs: applying one node's recorded state to another "would
/// produce a confident divergence report about an execution that never
/// happened". A field like that must ship the full treatment (bump the
/// version and gate the READER on it) rather than relying on this warn.
fn read_state_coverage(reader: &BagReader) -> Option<StateCoverage> {
    read_state_coverage_reporting(reader, true)
}

/// [`read_state_coverage`] with its DIAGNOSTICS switchable.
///
/// # Why a second consumer must read QUIETLY
///
/// This attachment has a SECOND reader: [`read_state_arm`],
/// which the replay engine calls to reconstruct the catch-up clamp. Both read
/// the SAME bytes off the SAME `BagReader`, so with both announcing, a mid-run
/// resume printed every skew/malformed line TWICE: one condition, one bag, two
/// identical lines an operator cannot act on differently.
///
/// The announcement belongs to the ANCHORS path and stays there. Every line
/// here is about a restore ("this recording's anchors are unusable", "if the
/// restore reports missing anchors"), and the arm read performs no restore — it
/// takes one `Option` field and installs a clamp. So the quiet read is not a
/// suppressed warning, it is a reader that has nothing of its own to say.
///
/// It also RESTORES the earlier reading on the from-start path exactly: a
/// from-start replay returns at `first.step == 0` before `read_bag_anchors` is
/// ever called, so that path announced nothing before this arm existed and
/// announces nothing now.
fn read_state_coverage_reporting(reader: &BagReader, report: bool) -> Option<StateCoverage> {
    let att = match reader.attachment(STATE_COVERAGE_ATTACHMENT) {
        Ok(Some(att)) => att,
        Ok(None) => return None,
        Err(e) => {
            if report {
                tracing::warn!(
                    attachment = STATE_COVERAGE_ATTACHMENT,
                    error = %e,
                    "replay: could not read the checkpoint coverage manifest — the node index \
                     every state record carries cannot be resolved to a node id, so this \
                     recording's anchors are unusable and a mid-run replay will refuse"
                );
            }
            return None;
        }
    };
    match serde_json::from_slice::<StateCoverage>(&att.data) {
        Ok(coverage) => {
            // FORWARD only, and WARN only, which is the right treatment for the
            // direction it owns: a newer manifest's extra keys are ignorable by
            // construction (the type carries no `deny_unknown_fields`), so the
            // bag is readable and refusing it would turn every additive field
            // into a compatibility break. The BACKWARD direction is a different
            // gate and cannot be this one: it lives in `read_bag_anchors`, it
            // REFUSES rather than warns, and it reads
            // `state_record_format_version` rather than this constant, because
            // an older bag's RECORDS are laid out differently and reading them
            // is the silent wrong restore this warn would wave through.
            if report && coverage.version > cerulion_bagd::STATE_COVERAGE_VERSION {
                tracing::warn!(
                    attachment = STATE_COVERAGE_ATTACHMENT,
                    bag_version = coverage.version,
                    supported_version = cerulion_bagd::STATE_COVERAGE_VERSION,
                    "replay: this bag's checkpoint manifest was recorded by a NEWER bagd \
                     (manifest version {} vs the {} this build understands) — it is being read as \
                     the version this build knows. If the restore reports missing anchors, \
                     suspect this skew before suspecting the recording",
                    coverage.version,
                    cerulion_bagd::STATE_COVERAGE_VERSION
                );
            }
            Some(coverage)
        }
        Err(e) => {
            if report {
                tracing::warn!(
                    attachment = STATE_COVERAGE_ATTACHMENT,
                    error = %e,
                    "replay: malformed state_coverage.json attachment — the node index every \
                     state record carries cannot be resolved to a node id, so this recording's \
                     anchors are unusable and a mid-run replay will refuse"
                );
            }
            None
        }
    }
}

/// The CAPTURE PLANE this recording's anchors were taken
/// under, or `None` when the bag names none.
///
/// # Why this is read APART from the resume plan
///
/// The catch-up clamp applies to the whole replayed run, and a replay
/// has TWO entry shapes: a mid-run RESUME, and a FROM-START replay of a
/// `--record` bag (`resolve_resume` answers `Ok(None)` at step 0 — the
/// constructor's state IS the state). Only the first builds a `ResumePlan`, so
/// a clamp carried on the plan is installed on exactly the shape that is NOT
/// the ordinary `graph run --record` bag — while the plane is always-on, so
/// that bag's live steps WERE clamped and its replay would run the full
/// catch-up burst.
///
/// So the arm is read from the BAG, once, on both paths.
///
/// It reads the coverage attachment ONLY — no state records are walked — which
/// is what makes it affordable on the from-start path, where nothing else needs
/// them.
///
/// QUIET by construction: on a mid-run resume `read_bag_anchors` has already
/// read and announced this same attachment, and a second identical line is
/// noise.
///
/// (`read_state_coverage_reporting`, which carries the full adjudication, is
/// deliberately PROSE in backticks rather than an intra-doc link: it is private
/// and this item is `pub`, so a link fails CI's Documentation job under
/// `RUSTDOCFLAGS=-D warnings` — the same class `graph/runtime.rs` states for
/// `attach_state_arm`.)
pub fn read_state_arm(reader: &BagReader) -> Option<cerulion_bagd::StateArmCoverage> {
    let coverage = read_state_coverage_reporting(reader, false);
    if coverage.is_none()
        && reader
            .attachment(STATE_COVERAGE_ATTACHMENT)
            .ok()
            .flatten()
            .is_some()
    {
        // The attachment IS there and could not be read. That is the one arm
        // this reader must not swallow: it silently costs the replay its clamp,
        // and on a from-start bag NOTHING else reads this attachment, so without
        // this line the loss is invisible at every level.
        //
        // Its OWN sentence, not the anchors path's. That path reports a
        // RESTORE that will refuse; this one reports a `Period` catch-up burst
        // the recording did not run, which shows up as a divergence the
        // candidate did not cause — different consequence, different remedy,
        // so a reader on the mid-run path gets two lines that say different
        // things rather than the duplicate this reader was made quiet for.
        tracing::warn!(
            attachment = STATE_COVERAGE_ATTACHMENT,
            "replay: this recording's checkpoint manifest is present but unreadable, so the \
             catch-up clamp its live steps ran under cannot be reconstructed — this replay runs \
             UNCLAMPED, and a `Period` node may fire a catch-up burst the recording never did. \
             Re-record to fix; the divergence that follows is not the candidate's"
        );
    }
    coverage.and_then(|c| c.armed)
}

/// The `node_idx → node id` table the manifest states, and how many rings it
/// was drawn from.
///
/// A node whose `node_idx` the manifest does not carry (a writer that predates
/// the field) simply has no entry, which makes its records unattributable — the
/// correct reading, since the index really is absent rather than zero.
fn index_table(coverage: &StateCoverage) -> BTreeMap<u32, String> {
    let mut table = BTreeMap::new();
    for (node_id, node) in &coverage.nodes {
        if let Some(idx) = node.node_idx {
            table.insert(idx, node_id.clone());
        }
    }
    table
}

/// The `__cerulion/flashback.json` attachment a Flashback CAPTURE carries.
///
/// A `--record` bag has none, and that absence is the gate on the identity
/// refusal below: nothing about an ordinary recording's resume changes.
const FLASHBACK_MANIFEST_ATTACHMENT: &str = "__cerulion/flashback.json";

/// The capture numbers this bag's restore points name, when there is more than
/// one of them. `None` when the set is whole.
///
/// # Read tolerantly, and every degradation means "not mixed"
///
/// The same rule `replay_engine`'s `capture_anchor_target_ns` is read under, and
/// for the same reason: this attachment describes what a capture was ABOUT, and
/// a resume that refused because a prose field would not parse would be refusing
/// over metadata it does not otherwise read. So a bag with no attachment, an
/// attachment that is not JSON, a manifest with no `anchor.per_rank` block, an
/// entry with no `capture_seq`, and a set whose entries agree all reach the same
/// answer: `None`, not mixed, carry on. That covers every bag written before the
/// per-rank block existed and every ordinary capture written after it.
///
/// The refusal therefore fires on exactly one shape: a manifest that DOES carry
/// per-rank restore points and whose points name two or more DIFFERENT captures.
/// A bag cannot reach that shape by being old or by being damaged; it reaches it
/// by having been assembled from two captures.
///
/// # Why the manifest and not the records
///
/// The identity is stamped at SELECTION, and a state record is written at
/// harvest, long before any capture exists. There is nowhere on the record for
/// it to be, which is the same reason the recorder puts it on the manifest.
fn mixed_capture_identities(reader: &BagReader) -> Option<Vec<(u64, Vec<u32>)>> {
    let attachment = reader.attachment(FLASHBACK_MANIFEST_ATTACHMENT).ok()??;
    let parsed: serde_json::Value = serde_json::from_slice(&attachment.data).ok()?;
    let per_rank = parsed.get("anchor")?.get("per_rank")?.as_object()?;
    // Keyed by capture, valued by the ranks that name it, so the message can
    // state which half of the graph came from where. `BTreeMap` for both, so two
    // reads of one bag render the same sentence.
    let mut by_capture: BTreeMap<u64, Vec<u32>> = BTreeMap::new();
    for (rank, entry) in per_rank {
        let Some(seq) = entry.get("capture_seq").and_then(serde_json::Value::as_u64) else {
            continue;
        };
        // A rank key that is not a number is a manifest this reader does not
        // understand, and the tolerant rule says carry on rather than refuse.
        let Ok(rank) = rank.parse::<u32>() else {
            continue;
        };
        by_capture.entry(seq).or_default().push(rank);
    }
    if by_capture.len() < 2 {
        return None;
    }
    for ranks in by_capture.values_mut() {
        ranks.sort_unstable();
    }
    Some(by_capture.into_iter().collect())
}

/// Read every anchor a bag carries.
///
/// The record stream is the bag's `__cerulion/state` channel in file order —
/// which is the ring order the recorder wrote, so it is exactly what
/// [`StateAssembler`] expects. Records from several nodes interleave (the bag
/// merges every worker rank's), and the assembler keys by
/// `(run_id, step, node_idx)`, so they reassemble independently.
pub fn read_bag_anchors(reader: &BagReader) -> Result<BagAnchors, AnchorReadRefusal> {
    let Some(coverage) = read_state_coverage(reader) else {
        // No manifest: either a pre-anchor bag (the overwhelming majority) or
        // one whose manifest could not be read (already warned above). Either
        // way there is no index table, so no record could be attributed — and
        // reading them only to report every one unattributable would turn every
        // ordinary bag into a noisy no-op.
        return Ok(BagAnchors::default());
    };
    // The ambiguity refusal comes FIRST: it is a property of the manifest, so
    // it must not depend on whether this particular bag happened to hold any
    // records at the resume step (a recording that declares two rings is
    // unreadable whether or not the walk finds anything).
    if coverage.rings_declared > 1 {
        return Err(AnchorReadRefusal::MultiRingAmbiguous {
            rings: coverage.rings_declared,
            state_record_format: coverage.state_record_format_version,
        });
    }
    // The version gate, BOTH WAYS, and it lives HERE rather than in the parse.
    //
    // A state record carries no version of its own before format version 1 and
    // no checksum at any version, so nothing at the RECORD level can catch a bag
    // written under a different layout: `StateRecordHeader::from_bytes` takes a
    // fixed-width array and would simply read a different field out of each
    // offset. The only discriminator is the BAG's, which is why the manifest
    // carries the writer's record format and why an absent key is refused rather
    // than read as "version 0": a manifest that predates the key and a manifest
    // whose writer forgot it are the same bytes, and the safe reading of both is
    // a refusal.
    //
    // EXACT EQUALITY, and the too-new half is the half worth arguing for. This
    // gate read `v >= known`, which admitted a manifest naming format 2 or later
    // into a version 1 assembler. Every record then failed `validate` and landed
    // as `Malformed`, which is counted into `malformed_records` and logged at
    // `debug`, so the reader answered "this bag records no anchor" and sent the
    // operator to re-record a bag that is perfectly good and simply newer than
    // this build. That is the same misdiagnosis the too-old arm exists to
    // prevent, arriving from the other side, so it gets the same treatment: a
    // refusal that NAMES both versions and says which build to reach for. It
    // fires before the index table is built and before the assembler exists, so
    // no record of a future format is ever assembled or counted.
    //
    // It is in `read_bag_anchors` and NOT in `read_state_coverage`, which
    // `read_state_arm` shares: a refusal at the parse would take the catch-up
    // clamp away from every bag whose records this build cannot read, on the
    // from-start path, which this gate has no business touching.
    let known = cerulion_core::state_ring::STATE_RECORD_FORMAT_VERSION;
    match coverage.state_record_format_version {
        Some(v) if v == known => {}
        Some(v) if v > known => {
            return Err(AnchorReadRefusal::StateRecordFormatTooNew { carried: v, known })
        }
        carried => return Err(AnchorReadRefusal::StateRecordFormatTooOld { carried, known }),
    }
    // THE MIXED IDENTITY REFUSAL, third and last, so the two refusals above keep
    // their precedence exactly. A bag that is multi-ring ambiguous or written
    // under an older record format is still refused for THAT, which is the
    // stronger fact: those two say the records cannot be read at all, while this
    // one says they can be read and must not be combined.
    if let Some(captures) = mixed_capture_identities(reader) {
        return Err(AnchorReadRefusal::MixedCaptureIdentity { captures });
    }
    let table = index_table(&coverage);

    let mut out = BagAnchors {
        coverage_present: true,
        ..BagAnchors::default()
    };
    // The mode is READ from the manifest, never inferred (see the module docs).
    let mut assembler = if coverage.attached_mid_run {
        StateAssembler::armed()
    } else {
        StateAssembler::passthrough()
    };

    // STREAMED, and filtered to the state channel INSIDE the reader — never
    // `recover_messages`, which collects every message in the bag into a
    // `Vec<BagMessage>` whose payloads are owned copies. A recording is
    // routinely gigabytes of camera frames and the anchors are a few hundred
    // kilobytes on ONE reserved channel, so collecting first would make every
    // mid-run replay's peak memory the size of the bag before a single frame of
    // it was needed. `recover_messages_on_topic` tests the topic against the
    // borrowed message and copies only what matches.
    //
    // A read failure here is NOT a refusal: the bag's mandatory artifacts were
    // already read to get this far, and an unreadable OPTIONAL channel must not
    // out-rank them. It leaves `facts` empty, which reads downstream as "no
    // anchor for this node" — the same answer as a bag that has none.
    let messages = match reader.recover_messages_on_topic(STATE_TOPIC) {
        Ok(messages) => messages,
        Err(e) => {
            tracing::warn!(
                error = %e,
                "replay: could not walk this bag's messages to collect its checkpoint anchors — \
                 a mid-run replay will refuse for want of state it cannot read"
            );
            return Ok(out);
        }
    };

    let resolve = |node_idx: u32, out: &mut BagAnchors| -> Option<String> {
        match table.get(&node_idx) {
            Some(id) => Some(id.clone()),
            None => {
                out.unattributable_records += 1;
                None
            }
        }
    };

    for message in messages {
        // A mid-stream read error ENDS the walk with whatever was assembled,
        // which is exactly what `recover_messages` did: it returned the
        // messages before the tear paired with `TornTail`, and this reader
        // ignored the verdict. A torn state channel is not silent — it leaves
        // the affected anchors incomplete, so `plan_restore` refuses the nodes
        // they belong to by name.
        let message = match message {
            Ok(m) => m,
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "replay: this bag's message stream is torn — the checkpoint anchors after the \
                     tear are unreadable, so a mid-run replay will refuse for the nodes they \
                     describe"
                );
                break;
            }
        };
        let Some(event) = assembler.feed(&message.data) else {
            continue;
        };
        match event {
            StateAnchorEvent::Complete {
                run_id,
                step,
                node_idx,
                bytes,
                ..
            } => {
                let Some(node) = resolve(node_idx, &mut out) else {
                    continue;
                };
                out.facts.push(AnchorFact {
                    run_id,
                    step,
                    node: node.clone(),
                    outcome: AnchorOutcome::Complete,
                });
                out.blobs
                    .entry((run_id, step))
                    .or_default()
                    .insert(node, bytes);
            }
            StateAnchorEvent::Torn {
                run_id,
                step,
                node_idx,
                ..
            } => {
                let Some(node) = resolve(node_idx, &mut out) else {
                    continue;
                };
                out.facts.push(AnchorFact {
                    run_id,
                    step,
                    node,
                    outcome: AnchorOutcome::Torn,
                });
            }
            // Precedence: a SKIP for an anchor this stream already COMPLETED is
            // a DIAGNOSTIC, not an outcome. Recording it as one would put a Complete
            // and a Skipped fact on the same `(run, step, node)` and let a restore
            // apply state the very next fact contradicts — which is the whole reason
            // the rule exists. The anchor already produced its fact and its blob; this
            // says a capture child was killed between publishing that record and
            // bumping its accounting word, which is worth an operator's attention and
            // nothing else.
            StateAnchorEvent::SkipAfterComplete {
                run_id,
                step,
                node_idx,
                cause,
                ..
            } => {
                let node = resolve(node_idx, &mut out).unwrap_or_default();
                tracing::warn!(
                    run_id,
                    step,
                    node = %node,
                    cause = ?cause,
                    "replay: this bag carries BOTH a complete anchor and a skip for one node at \
                     one step — a capture child was killed between publishing its final record \
                     and recording that it had. The COMPLETE anchor is authoritative and is \
                     being used; the skip is the parent's post-mortem guess about accounting it \
                     could not see"
                );
            }
            StateAnchorEvent::Skipped {
                run_id,
                step,
                node_idx,
                cause,
                ..
            } => {
                let Some(node) = resolve(node_idx, &mut out) else {
                    continue;
                };
                out.facts.push(AnchorFact {
                    run_id,
                    step,
                    node,
                    outcome: AnchorOutcome::Skipped(cause),
                });
            }
            StateAnchorEvent::Malformed { reason } => {
                out.malformed_records += 1;
                tracing::debug!(
                    reason = %reason,
                    "replay: a state record could not be keyed; it contributes to no anchor"
                );
            }
        }
    }
    // Anything still open when the stream ends is TORN, not short — the same
    // rule the recorder's own ledger applies at finish.
    for event in assembler.finish() {
        if let StateAnchorEvent::Torn {
            run_id,
            step,
            node_idx,
            ..
        } = event
        {
            let Some(node) = resolve(node_idx, &mut out) else {
                continue;
            };
            out.facts.push(AnchorFact {
                run_id,
                step,
                node,
                outcome: AnchorOutcome::Torn,
            });
        }
    }
    out.head_records_discarded = assembler.discarded();

    if out.unattributable_records > 0 {
        tracing::warn!(
            records = out.unattributable_records,
            "replay: this recording carries state records whose node index no manifest entry \
             names, so they belong to no node here — the nodes they describe will read as having \
             no anchor. Suspect a recorder older than the manifest's node_idx field, or a bag \
             whose coverage manifest was written by a different run"
        );
    }
    Ok(out)
}

/// Pick the one run whose anchor at `step` a resume may use.
///
/// Zero runs is NOT an error here: it means the bag has no anchor at that step
/// at all, and `plan_restore` says so per node (naming which are missing) —
/// which is a far better message than one this function could write.
pub fn resolve_run_at(anchors: &BagAnchors, step: u64) -> Result<Option<u64>, AnchorReadRefusal> {
    let runs = anchors.runs_at_step(step);
    match runs.len() {
        0 => Ok(None),
        1 => Ok(Some(runs[0])),
        _ => Err(AnchorReadRefusal::AmbiguousRun { step, runs }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_bagd::{StateCoverage, StateNodeCoverage};

    fn node_cov(ring: &str, idx: Option<u32>) -> StateNodeCoverage {
        StateNodeCoverage {
            ring: ring.to_string(),
            node_idx: idx,
            anchors_complete: 1,
            ..StateNodeCoverage::default()
        }
    }

    fn coverage(nodes: &[(&str, Option<u32>)], rings: usize) -> StateCoverage {
        StateCoverage {
            version: cerulion_bagd::STATE_COVERAGE_VERSION,
            attached_mid_run: false,
            armed: None,
            rings_declared: rings,
            ring_ranks: Default::default(),
            state_record_format_version: Some(
                cerulion_core::state_ring::STATE_RECORD_FORMAT_VERSION,
            ),
            ranks_discovered: Vec::new(),
            ranks_missing: Vec::new(),
            rings_unavailable: Default::default(),
            records: 0,
            head_records_discarded: 0,
            malformed_records: 0,
            foreign_run_records: 0,
            nodes: nodes
                .iter()
                .map(|(id, idx)| (id.to_string(), node_cov("r0", *idx)))
                .collect(),
            unattributed_indices: Default::default(),
        }
    }

    #[test]
    fn the_index_table_carries_only_nodes_whose_index_the_manifest_states() {
        // A writer that predates the field left `node_idx` absent. That node is NOT
        // given index 0 by default — a fabricated index would attribute some
        // other node's anchor to it, which is the one outcome worse than
        // reporting no anchor at all.
        let cov = coverage(&[("alpha", Some(0)), ("beta", None), ("gamma", Some(2))], 1);
        let table = index_table(&cov);
        assert_eq!(table.get(&0).map(String::as_str), Some("alpha"));
        assert_eq!(table.get(&2).map(String::as_str), Some("gamma"));
        assert_eq!(
            table.len(),
            2,
            "beta has no index, so it has no entry: {table:?}"
        );
    }

    #[test]
    fn a_two_ring_manifest_is_refused_by_name_rather_than_read() {
        let cov = coverage(&[("alpha", Some(0))], 2);
        // The refusal is a property of the manifest alone, so it is asserted on
        // the classifier the reader consults rather than through a crafted bag.
        assert!(cov.rings_declared > 1);
        let refusal = AnchorReadRefusal::MultiRingAmbiguous {
            rings: cov.rings_declared,
            state_record_format: cov.state_record_format_version,
        };
        let text = refusal.to_string();
        assert!(text.contains("2 state rings"), "{text}");
        assert!(text.contains("--single-process"), "names the fix: {text}");

        // And the sentence says what is ACTUALLY true, which is the half that
        // went stale under this stack. The rank clause is a property of the
        // FORMAT and is stated as one, so it holds for every bag that reads
        // this sentence: from state record format version 1 the records carry
        // their producer's rank and the manifest carries the ring to rank
        // join. The remedy the sentence used to name, re-record once the
        // records carry their rank, is a LOOP: the operator re-records, the
        // records carry their rank exactly as they already did, and the same
        // refusal fires.
        assert!(
            text.contains(
                "From state record format version 1 a state record carries its producer's rank"
            ),
            "the clause is about the FORMAT, not about this bag: {text}"
        );
        assert!(
            text.contains("the coverage manifest carries the ring to rank join"),
            "and it names the other half of the format's own claim: {text}"
        );
        assert!(
            !text.contains("no rank"),
            "and never claims the opposite of it: {text}"
        );
        assert!(
            !text.contains("re-record"),
            "and never sends an operator round a loop that ends at this same \
             refusal: {text}"
        );
        assert!(
            text.contains("replay a recording of a single rank"),
            "the remedy is the one that actually clears the refusal: {text}"
        );

        // THE BRANCH, which is the half a format-level clause alone does not
        // buy. The ring gate is asked BEFORE the format gate, so a two-ring bag
        // whose manifest names no state record format reads this same
        // sentence, and that bag is a recording from before rank existed. It
        // gets the property of the format, which is true whoever reads it, and
        // NO claim that its own records carry a rank.
        let pre_rank = AnchorReadRefusal::MultiRingAmbiguous {
            rings: 2,
            state_record_format: None,
        }
        .to_string();
        assert!(
            pre_rank.contains(
                "From state record format version 1 a state record carries its producer's rank"
            ),
            "the format's property holds for every bag: {pre_rank}"
        );
        assert!(
            pre_rank.contains("names no state record format at all"),
            "and the bag clause says what THIS bag proves: {pre_rank}"
        );
        assert!(
            !pre_rank.contains("manifest names state record format version"),
            "a pre-rank recording is never told its manifest names a format it \
             does not name: {pre_rank}"
        );

        // And the bag that DOES name one is told which, so the branch is a
        // branch rather than one arm wearing two.
        let named = AnchorReadRefusal::MultiRingAmbiguous {
            rings: 2,
            state_record_format: Some(1),
        }
        .to_string();
        assert!(
            named.contains("manifest names state record format version 1"),
            "the bag clause names the version the manifest carries: {named}"
        );
        assert!(
            !named.contains("names no state record format at all"),
            "and not the other arm: {named}"
        );
    }

    #[test]
    fn one_run_at_the_step_resolves_and_two_refuse_naming_both() {
        let mut anchors = BagAnchors::default();
        anchors.facts.push(AnchorFact {
            run_id: 7,
            step: 41,
            node: "alpha".into(),
            outcome: AnchorOutcome::Complete,
        });
        assert_eq!(resolve_run_at(&anchors, 41), Ok(Some(7)));
        // A step the bag says nothing about is NOT an error here — the per-node
        // refusal `plan_restore` writes is the better message.
        assert_eq!(resolve_run_at(&anchors, 40), Ok(None));

        anchors.facts.push(AnchorFact {
            run_id: 9,
            step: 41,
            node: "alpha".into(),
            outcome: AnchorOutcome::Complete,
        });
        let err = resolve_run_at(&anchors, 41).expect_err("two runs at one step is ambiguous");
        let text = err.to_string();
        assert!(text.contains("anchors from 2 runs"), "{text}");
        // BOTH ids, so an operator can tell which recording is which.
        assert!(text.contains("0x0000000000000007"), "{text}");
        assert!(text.contains("0x0000000000000009"), "{text}");
    }

    #[test]
    fn runs_at_step_is_sorted_and_deduplicated() {
        // Sorted so a refusal names the same runs in the same order every run,
        // and deduplicated so one run with forty nodes is not "40 runs".
        let mut anchors = BagAnchors::default();
        for (run, node) in [(9u64, "a"), (7, "b"), (9, "c"), (7, "d")] {
            anchors.facts.push(AnchorFact {
                run_id: run,
                step: 3,
                node: node.into(),
                outcome: AnchorOutcome::Complete,
            });
        }
        assert_eq!(anchors.runs_at_step(3), vec![7, 9]);
    }

    #[test]
    fn blobs_at_serves_only_the_requested_run_and_step() {
        let mut anchors = BagAnchors::default();
        anchors
            .blobs
            .entry((7, 41))
            .or_default()
            .insert("alpha".into(), vec![1, 2, 3]);
        anchors
            .blobs
            .entry((7, 42))
            .or_default()
            .insert("alpha".into(), vec![9]);
        assert_eq!(
            anchors.blobs_at(7, 41).get("alpha").map(Vec::as_slice),
            Some(&[1u8, 2, 3][..])
        );
        assert!(anchors.blobs_at(8, 41).is_empty(), "a different run's step");
        assert!(anchors.blobs_at(7, 40).is_empty(), "a step with no anchor");
    }

    // =======================================================================
    // The BACKWARD gate: a bag recorded before this record format
    // =======================================================================

    /// Write a bag carrying `manifest` as its state-coverage attachment and
    /// `records` on the reserved state channel, and read it back.
    ///
    /// The records are written VERBATIM, which is what lets an arm hand this a
    /// stream in the PREVIOUS record layout: the writer's only rule for this
    /// channel is the 512-byte record size, which did not move.
    fn craft_and_read(
        manifest: &str,
        records: &[Vec<u8>],
    ) -> Result<BagAnchors, AnchorReadRefusal> {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("crafted.mcap");
        {
            let mut w = cerulion_bag::BagWriter::create(
                &path,
                cerulion_bag::BagWriterConfig::default(),
                &[],
            )
            .expect("create bag");
            w.write_attachment(
                STATE_COVERAGE_ATTACHMENT,
                "application/json",
                0,
                0,
                manifest.as_bytes(),
            )
            .expect("write the manifest");
            let state_id = w.state_channel_id();
            w.write_chunk(|c| {
                for (i, r) in records.iter().enumerate() {
                    c.write_message(state_id, i as u32, 1_000 + i as u64, 1_000 + i as u64, &[r])?;
                }
                Ok(())
            })
            .expect("write the records");
            w.finalize().expect("finalize");
        }
        let reader = BagReader::open(&path).expect("open the crafted bag");
        read_bag_anchors(&reader)
    }

    /// One record in the layout this build MINTS, for the control arm.
    fn this_format_record(node_idx: u32) -> Vec<u8> {
        cerulion_core::state_ring::encode_record(
            &cerulion_core::state_ring::StateRecordHeader {
                run_id: 7,
                step: 41,
                node_idx,
                part: 0,
                kind: cerulion_core::state_ring::RECORD_KIND_FINAL_V2,
                len: 4,
                rank: 0,
                format_version: cerulion_core::state_ring::STATE_RECORD_FORMAT_VERSION,
            },
            &[1, 2, 3, 4],
        )
        .to_vec()
    }

    /// One record in the layout that PRECEDED this one: a narrower header whose
    /// kind word sits at bytes 24 to 28 and whose payload begins at byte 32.
    fn previous_format_record(node_idx: u32) -> Vec<u8> {
        let mut r = vec![0u8; cerulion_core::state_ring::STATE_RECORD_SIZE as usize];
        r[0..8].copy_from_slice(&7u64.to_le_bytes());
        r[8..16].copy_from_slice(&41u64.to_le_bytes());
        r[16..20].copy_from_slice(&node_idx.to_le_bytes());
        r[24..28].copy_from_slice(&2u32.to_le_bytes()); // the previous FINAL kind
        r[28..32].copy_from_slice(&4u32.to_le_bytes());
        r[32..36].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);
        r
    }

    /// One record this build CANNOT read, whatever version admits it.
    ///
    /// The kind word carries a value no format has minted, so
    /// [`StateRecordHeader::validate`] refuses it on its own terms rather than
    /// on a version disagreement: it is the record shape that lets a test tell
    /// "the gate fired first" apart from "the reader could not read these
    /// anyway", because at this build's version the same bytes are WALKED and
    /// counted as malformed instead of refused.
    fn unreadable_record() -> Vec<u8> {
        let mut r = vec![0u8; cerulion_core::state_ring::STATE_RECORD_SIZE as usize];
        r[0..8].copy_from_slice(&7u64.to_le_bytes());
        r[8..16].copy_from_slice(&41u64.to_le_bytes());
        r[24..28].copy_from_slice(&99u32.to_le_bytes()); // a kind no format mints
        r[36..40]
            .copy_from_slice(&cerulion_core::state_ring::STATE_RECORD_FORMAT_VERSION.to_le_bytes());
        r
    }

    // =======================================================================
    // ORACLE 14: the MIXED CAPTURE IDENTITY refusal
    // =======================================================================

    /// A flashback capture manifest whose per-rank block names `entries` as
    /// `(rank, capture_seq)` pairs.
    ///
    /// Only the keys the identity reader looks at are written out: the reader
    /// must work on a manifest it does not otherwise understand, and a fixture
    /// carrying the whole block would hide a reader that had quietly started
    /// depending on a neighbour field.
    fn flashback_json(entries: &[(u32, u64)]) -> String {
        let per_rank: Vec<String> = entries
            .iter()
            .map(|(rank, seq)| format!(r#""{rank}": {{ "capture_seq": {seq}, "step": 41 }}"#))
            .collect();
        format!(
            r#"{{ "version": 1, "seq": 7, "anchor": {{ "embedded": true,
                 "per_rank": {{ {} }} }} }}"#,
            per_rank.join(", ")
        )
    }

    /// [`craft_and_read`] with a flashback capture manifest attached too.
    fn craft_with_flashback(
        manifest: &str,
        flashback: Option<&str>,
        records: &[Vec<u8>],
    ) -> Result<BagAnchors, AnchorReadRefusal> {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("crafted.mcap");
        {
            let mut w = cerulion_bag::BagWriter::create(
                &path,
                cerulion_bag::BagWriterConfig::default(),
                &[],
            )
            .expect("create bag");
            w.write_attachment(
                STATE_COVERAGE_ATTACHMENT,
                "application/json",
                0,
                0,
                manifest.as_bytes(),
            )
            .expect("write the manifest");
            if let Some(fb) = flashback {
                w.write_attachment(
                    FLASHBACK_MANIFEST_ATTACHMENT,
                    "application/json",
                    0,
                    0,
                    fb.as_bytes(),
                )
                .expect("write the flashback manifest");
            }
            let state_id = w.state_channel_id();
            w.write_chunk(|c| {
                for (i, r) in records.iter().enumerate() {
                    c.write_message(state_id, i as u32, 1_000 + i as u64, 1_000 + i as u64, &[r])?;
                }
                Ok(())
            })
            .expect("write the records");
            w.finalize().expect("finalize");
        }
        let reader = BagReader::open(&path).expect("open the crafted bag");
        read_bag_anchors(&reader)
    }

    /// ORACLE 14, arm (a): a bag whose restore points name two captures refuses
    /// the RESUME, and the sentence names both numbers, both ranks and a remedy.
    ///
    /// Asserted as literal substrings rather than against a second call of the
    /// formatter: a message tested against its own `format!` passes for any
    /// wording, including one that never states the second capture.
    #[test]
    fn restore_points_naming_two_captures_refuse_the_resume_by_name() {
        let err = craft_with_flashback(
            &manifest_json(Some("1"), 1),
            Some(&flashback_json(&[(0, 7), (1, 9)])),
            &[this_format_record(0)],
        )
        .expect_err("a mixed set must refuse");

        assert_eq!(
            err,
            AnchorReadRefusal::MixedCaptureIdentity {
                captures: vec![(7, vec![0]), (9, vec![1])],
            }
        );
        let text = err.to_string();
        for needle in [
            "name 2 different capture events",
            "capture 7 carries rank(s) 0",
            "capture 9 carries rank(s) 1",
            "Fix: resume from a bag whose ranks were all selected by one capture",
        ] {
            assert!(text.contains(needle), "missing {needle:?} in: {text}");
        }
    }

    /// ORACLE 14, arm (b): the refusal is RESUME SCOPED. A bag whose restore
    /// points name two captures still RENDERS under `bag info` and still PLAYS
    /// its frames; only the resume refuses.
    ///
    /// The commit's own claim, and the one the arm above cannot make: a reader
    /// that refused such a bag outright would take an operator's evidence away
    /// at exactly the moment they need it, so the refusal belongs to
    /// [`read_bag_anchors`] and to nothing else. This drives the three paths
    /// apart on ONE bag.
    ///
    /// The frame is written with a hand-built wire header and read back BYTE FOR
    /// BYTE, because "plays" is a claim about the bytes rather than about a count.
    #[test]
    fn a_bag_whose_restore_points_name_two_captures_still_renders_and_still_plays() {
        use cerulion_bag::{BagWriterConfig, TopicSchema};

        const TOPIC: &str = "/imu";
        const HASH: u64 = 0x1417_1417_1417_1417;
        let payload = [7u8; 24];
        let mut frame = vec![0u8; cerulion_core::WireHeader::SIZE + payload.len()];
        cerulion_core::WireHeader {
            schema_hash: HASH,
            total_size: (cerulion_core::WireHeader::SIZE + payload.len()) as u32,
            offset_table_offset: 0,
            offset_table_count: 0,
            sequence: 0,
            timestamp_ns: 1_000_000_000,
        }
        .write_to_buf(&mut frame[..cerulion_core::WireHeader::SIZE]);
        frame[cerulion_core::WireHeader::SIZE..].copy_from_slice(&payload);

        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("mixed.mcap");
        {
            let mut w = cerulion_bag::BagWriter::create(
                &path,
                BagWriterConfig::default(),
                &[TopicSchema {
                    topic: TOPIC.to_string(),
                    schema_name: "geometry_msgs/Vector3".to_string(),
                    schema_hash: HASH,
                    wire_fixed_size: 24,
                }],
            )
            .expect("create bag");
            w.write_attachment(
                STATE_COVERAGE_ATTACHMENT,
                "application/json",
                0,
                0,
                manifest_json(Some("1"), 1).as_bytes(),
            )
            .expect("write the manifest");
            // The MIXED set: rank 0 selected by capture 7, rank 1 by capture 9.
            w.write_attachment(
                FLASHBACK_MANIFEST_ATTACHMENT,
                "application/json",
                0,
                0,
                flashback_json(&[(0, 7), (1, 9)]).as_bytes(),
            )
            .expect("write the flashback manifest");
            let state_id = w.state_channel_id();
            let record = this_format_record(0);
            w.write_chunk(|c| {
                c.write_message(state_id, 0, 1_000, 1_000, &[&record[..]])?;
                c.write_message(TOPIC, 0, 1_000_000_000, 1_000_000_000, &[&frame[..]])
            })
            .expect("write the records and the frame");
            w.finalize().expect("finalize");
        }

        // (1) THE RESUME refuses, by name. The precondition for the other two
        // halves meaning anything: a bag that was not refused would render and
        // play for uninteresting reasons.
        let reader = BagReader::open(&path).expect("open");
        assert_eq!(
            read_bag_anchors(&reader).expect_err("a mixed set must refuse the resume"),
            AnchorReadRefusal::MixedCaptureIdentity {
                captures: vec![(7, vec![0]), (9, vec![1])],
            }
        );

        // (2) `bag info` RENDERS it, and names the topic it holds.
        let info = crate::bag_cmd::bag_info(&path, None).expect("bag info must still render");
        assert!(
            info.contains(TOPIC),
            "bag info must still name the bag's topic: {info}"
        );

        // (3) THE FRAMES still play, byte for byte.
        let index = reader.user_message_index().expect("the user message index");
        let spans = index.get(TOPIC).expect("the topic's frames");
        assert_eq!(spans.len(), 1, "one frame was written and one is readable");
        assert_eq!(
            reader.frame(&spans[0]),
            &frame[..],
            "the carried frame must come back byte for byte"
        );
    }

    /// ORACLE 14, arm (c), ANTI-VACUITY: the SAME bag with both entries at one
    /// capture passes the gate.
    ///
    /// One character apart from the arm above, so what the refusal reads is the
    /// disagreement and not the presence of the block.
    #[test]
    fn restore_points_naming_one_capture_pass_the_gate() {
        let read = craft_with_flashback(
            &manifest_json(Some("1"), 1),
            Some(&flashback_json(&[(0, 7), (1, 7)])),
            &[this_format_record(0)],
        )
        .expect("one capture, one answer");
        assert!(read.coverage_present);
    }

    /// Every bag in existence today: no flashback manifest at all, so nothing
    /// about its resume changes.
    ///
    /// The degradation arms are folded in beside it, because they must all reach
    /// the SAME answer and asserting them apart would let one drift into a
    /// refusal unnoticed: an attachment that is not JSON, a manifest with no
    /// `anchor` block, one whose `per_rank` entries carry no `capture_seq`, and
    /// one with a single entry.
    #[test]
    fn a_bag_without_per_rank_capture_identities_is_never_refused_for_them() {
        for (label, flashback) in [
            ("no attachment at all", None),
            ("not JSON", Some("this is not a manifest".to_string())),
            (
                "no anchor block",
                Some(r#"{ "version": 1, "seq": 7 }"#.to_string()),
            ),
            (
                "per_rank entries carrying no capture number",
                Some(
                    r#"{ "anchor": { "per_rank": { "0": { "step": 41 },
                         "1": { "step": 44 } } } }"#
                        .to_string(),
                ),
            ),
            ("one rank", Some(flashback_json(&[(0, 7)]))),
        ] {
            let read = craft_with_flashback(
                &manifest_json(Some("1"), 1),
                flashback.as_deref(),
                &[this_format_record(0)],
            )
            .unwrap_or_else(|e| panic!("{label} must not be refused: {e}"));
            assert!(read.coverage_present, "{label}");
        }
    }

    /// The PRECEDENCE: a bag that is refused for a stronger reason keeps that
    /// reason even when its restore points are also mixed.
    ///
    /// The two refusals above say the records cannot be read AT ALL; this one
    /// says they can be read and must not be combined. Reporting the weaker one
    /// would send an operator after the wrong problem, and the ordering in
    /// `read_bag_anchors` is the only thing that decides it.
    #[test]
    fn a_stronger_refusal_outranks_the_mixed_identity_one() {
        let multi_ring = craft_with_flashback(
            &manifest_json(Some("1"), 2),
            Some(&flashback_json(&[(0, 7), (1, 9)])),
            &[this_format_record(0)],
        )
        .expect_err("two rings is still two rings");
        // The refusal carries the manifest's own state record format key, which
        // this fixture names as version 1, so the expected value names it too.
        assert_eq!(
            multi_ring,
            AnchorReadRefusal::MultiRingAmbiguous {
                rings: 2,
                state_record_format: Some(1)
            }
        );

        let too_old = craft_with_flashback(
            &manifest_json(None, 1),
            Some(&flashback_json(&[(0, 7), (1, 9)])),
            &[previous_format_record(0)],
        )
        .expect_err("an unreadable record layout is still unreadable");
        assert!(matches!(
            too_old,
            AnchorReadRefusal::StateRecordFormatTooOld { carried: None, .. }
        ));
    }

    /// A manifest with the two keys this build writes, or without them.
    fn manifest_json(format_version: Option<&str>, rings: usize) -> String {
        let key = match format_version {
            Some(v) => format!(r#""state_record_format_version": {v},"#),
            None => String::new(),
        };
        format!(
            r#"{{ "version": 1, {key} "rings_declared": {rings}, "records": 1,
                 "nodes": {{ "alpha": {{ "ring": "r0", "node_idx": 0,
                                         "anchors_complete": 1 }} }} }}"#
        )
    }

    /// A bag recorded before this state record format is REFUSED BY NAME, and the
    /// refusal carries the remedy an operator can act on.
    ///
    /// The refusal cannot come from the record: a state record written under the
    /// previous layout carries no version and there is no checksum anywhere on
    /// one, so a reader handed those bytes would take each payload from the wrong
    /// offset and could not know. The bag is the only thing that can say, which is
    /// what the manifest key is for.
    #[test]
    fn a_bag_recorded_before_this_state_record_format_is_refused_by_name() {
        let records = vec![previous_format_record(0)];
        let refusal = craft_and_read(&manifest_json(None, 1), &records)
            .expect_err("a manifest naming no record format must be refused");
        assert!(
            matches!(
                refusal,
                AnchorReadRefusal::StateRecordFormatTooOld { carried: None, .. }
            ),
            "refused by name: {refusal:?}"
        );
        let text = refusal.to_string();
        assert!(
            text.contains("recorded before state record format 1; re-record"),
            "the remedy, literally: {text}"
        );
        assert!(
            text.contains("a state record format this bag does not name"),
            "and what the bag said: {text}"
        );

        // The DISCRIMINATOR is the VALUE, not the key's presence: the same bag
        // carrying the key at a version BELOW this build's is refused too, and
        // the sentence names the version it carried.
        let below = craft_and_read(&manifest_json(Some("0"), 1), &records)
            .expect_err("a version below this build's must be refused");
        assert!(matches!(
            below,
            AnchorReadRefusal::StateRecordFormatTooOld {
                carried: Some(0),
                known: 1
            }
        ));
        assert!(
            below.to_string().contains("state record format version 0"),
            "{below}"
        );
    }

    /// The gate is LIVE on exactly the path an accepted bag walks, and the
    /// accepted bag really does reach the decode the refused one never got to.
    ///
    /// This is the anti-vacuity half. The refusal above proves nothing on its own:
    /// a reader that refused every bag would pass it. Here the SAME shape with the
    /// key at this build's value is accepted AND its records are walked, which is
    /// the work the early return skipped.
    #[test]
    fn a_bag_at_this_record_format_is_accepted_and_its_records_are_walked() {
        let anchors = craft_and_read(&manifest_json(Some("1"), 1), &[this_format_record(0)])
            .expect("a bag at this build's record format is not refused");
        assert!(anchors.coverage_present, "the manifest was read");
        assert_eq!(
            anchors.facts.len(),
            1,
            "and the record was DECODED, which the refused bag's never was: {:?}",
            anchors.facts
        );
        assert_eq!(anchors.facts[0].node, "alpha");
        assert_eq!(anchors.facts[0].step, 41);

        // The unattributable counter is the other proof the walk happened: a
        // record whose index the manifest does not name is COUNTED, and a walk
        // that never ran could not count it.
        let anchors = craft_and_read(&manifest_json(Some("1"), 1), &[this_format_record(9)])
            .expect("not refused");
        assert_eq!(anchors.unattributable_records, 1);
    }

    /// A bag recorded under a LATER state record format is refused BY NAME, with
    /// both versions in the sentence, and nothing in it is ever assembled.
    ///
    /// The gate this pins read `v >= known`, which let a manifest naming format 2
    /// into a version 1 assembler. The records then failed `validate` one by one
    /// and were counted as malformed behind a `debug` line, so the reader's answer
    /// was "this bag records no anchor" and the operator was sent to re-record a
    /// recording that was not damaged at all. Naming BOTH versions is what turns
    /// that into the one diagnosis an operator can act on, and the remedy has to
    /// be the opposite of the too-old arm's: reach for a newer build, do not
    /// re-record.
    ///
    /// Every expected value below is TYPED OUT rather than computed from the
    /// constant. A test that writes `STATE_RECORD_FORMAT_VERSION + 1` into the
    /// manifest and then asserts the message mentions `STATE_RECORD_FORMAT_VERSION
    /// + 1` passes whatever the gate does with the two numbers.
    #[test]
    fn a_bag_recorded_after_this_state_record_format_is_refused_by_name() {
        // The records are VALID at this build's format, so the only thing wrong
        // with this bag is the version its manifest names. That is what makes the
        // absence assertion below mean something: these records would assemble.
        let records = vec![this_format_record(0)];
        let refusal = craft_and_read(&manifest_json(Some("2"), 1), &records)
            .expect_err("a manifest naming a later record format must be refused");
        assert_eq!(
            refusal,
            AnchorReadRefusal::StateRecordFormatTooNew {
                carried: 2,
                known: 1
            },
            "refused by name, not swallowed: {refusal:?}"
        );
        let text = refusal.to_string();
        assert!(
            text.contains("state record format version 2"),
            "the BAG's version, literally: {text}"
        );
        assert!(
            text.contains("state record format version 1"),
            "and THIS BUILD's version, literally: {text}"
        );
        assert!(
            text.contains("read it with a build that reads state record format version 2"),
            "the remedy names the build to reach for: {text}"
        );
        // And it names that version EXACTLY. The remedy used to read "version 2
        // or later", which the gate below it does not honour: it admits only
        // `v == known`, so a build at version 3 hands this same bag the TOO OLD
        // sentence and tells the operator to re-record a recording the sentence
        // two clauses above just called undamaged. That is the misdiagnosis this
        // arm exists to prevent, arriving from the third side, so the clause is
        // pinned literally and the widening words are pinned ABSENT.
        assert!(
            !text.contains("or later"),
            "the remedy must not widen past the one version the gate admits: {text}"
        );
        assert!(
            text.ends_with("read it with a build that reads state record format version 2"),
            "and the remedy is the LAST clause, ending at that version: {text}"
        );
        assert!(
            text.contains("does not need re-recording"),
            "and says the recording is not the thing at fault: {text}"
        );

        // THE DERIVATION, and it is asserted at a seam that can SEE it. This
        // read the same call again and asked `is_err()`, three lines under an
        // `assert_eq!` on the whole error value: it could not fail while the
        // assertion above it passed, and it would pass unchanged if the early
        // return were deleted and the assembler let loose on these records.
        //
        // What THIS assertion pins, and the whole of it: the version decision
        // is taken from the MANIFEST and not from the records. Hand the same
        // version 2 manifest a stream of records this build rejects one by one
        // and the answer is still the version refusal, so a reader that read
        // the decision off the record stream is caught. The control under it is
        // what makes that mean anything: at this build's version those very
        // same records are walked and counted.
        //
        // What it does NOT pin is the PLACEMENT. A gate that keeps the manifest
        // decision and simply asks it AFTER the walk returns this identical
        // error value, because no count rides the `Err` path for a caller to
        // see. That is pinned by
        // `the_record_format_gate_fires_before_any_record_is_opened_for_keying`
        // below, which reads what the walk leaves behind rather than what it
        // returns.
        let malformed = vec![unreadable_record(), unreadable_record()];
        assert_eq!(
            craft_and_read(&manifest_json(Some("2"), 1), &malformed)
                .expect_err("the version decision is taken before the records are"),
            AnchorReadRefusal::StateRecordFormatTooNew {
                carried: 2,
                known: 1
            },
            "the version is decided before a single record is validated, so a bag \
             of unreadable records under a later format still refuses by version"
        );
        let read_them = craft_and_read(&manifest_json(Some("1"), 1), &malformed)
            .expect("the same records at this build's version are READ, not refused");
        assert_eq!(
            read_them.malformed_records, 2,
            "the control: when the gate admits the bag the reader really does walk \
             these records and count them, so the refusal above is the gate firing \
             first and not a reader that cannot read them either way"
        );
        assert!(
            read_them.facts.is_empty(),
            "and none of them assembled into an anchor: {:?}",
            read_them.facts
        );

        // THE CONTROL, and it is the point of pairing it with these exact
        // records: the SAME records under a manifest at this build's version
        // assemble into one anchor with nothing malformed. So the refusal above
        // withheld a readable anchor on the strength of the version alone, which
        // is the behaviour, and it is not a reader that refuses everything.
        let ok = craft_and_read(&manifest_json(Some("1"), 1), &records)
            .expect("the same records at this build's version are read");
        assert_eq!(ok.facts.len(), 1, "one anchor: {:?}", ok.facts);
        assert_eq!(ok.facts[0].node, "alpha");
        assert_eq!(ok.malformed_records, 0, "and nothing malformed about them");
    }

    /// The record format gate fires BEFORE any record is opened for keying,
    /// and the witness is the line the walk writes over the first record it
    /// does open. The name says `for keying` because that is the seam the
    /// witness sits on: the line is written where a record is resolved
    /// against the index table or refused by the assembler, and a placement
    /// earlier than that seam is the one case below says it cannot rule out.
    ///
    /// The assertion above it pins the DERIVATION: the version decision is
    /// taken from the manifest, so a reader that derived it from the record
    /// stream fails there. It cannot pin the PLACEMENT, and that is a gap
    /// rather than a quibble. A gate moved past the walk, asking the manifest
    /// only once every record has been opened, returns the IDENTICAL error
    /// value: no count and no marker rides the `Err` path, so the two
    /// placements cannot be told apart by what the reader RETURNS.
    ///
    /// They are told apart by what the walk WRITES. The FIRST record this arm
    /// hands the reader is one no format has minted, so the assembler
    /// classifies it as malformed and the reader logs it BY RECORD, inside the
    /// loop, on the first pass through it. A reader that refused first cannot
    /// have written that line, and a reader that opened one record cannot have
    /// withheld it, so no deferral past the first record survives it.
    ///
    /// The unattributable report is asserted beside it and is the WEAKER of the
    /// two, which is said here rather than left to be found: that report is
    /// written ONCE at the end of the walk out of a counter, so a gate deferred
    /// to just above it withholds it exactly as the committed placement does
    /// and would pass on that assertion alone.
    ///
    /// WHAT THIS DOES NOT DISTINGUISH, and the one deferral it does not: any
    /// placement between the ambiguity refusal and the first record being
    /// opened. A gate sitting under the index table, or immediately above the
    /// walk, opens no record either and passes every assertion here. The
    /// reader's own comment at the gate claims more than that, since it says
    /// the gate fires before the index table is built and before the assembler
    /// exists, and this test pins neither of those two halves.
    ///
    /// A bag with NO records, or one whose message stream is unreadable, does
    /// not separate the placements at all: the reader's unreadable-stream arm
    /// is the only early exit under the gate, and it is not reachable from a
    /// bag a test can craft, because building the message stream cannot fail
    /// and a bad stream reports itself on its FIRST ITEM, which ends the walk
    /// at the same final answer. What the walk writes is the seam that does
    /// separate them.
    ///
    /// The control runs SECOND on purpose: an absence proves nothing until the
    /// same records under an admitted manifest do write the lines looked for.
    #[test]
    #[tracing_test::traced_test]
    fn the_record_format_gate_fires_before_any_record_is_opened_for_keying() {
        // FIRST a record this build cannot key at any version, so a walk that
        // opens it must say so on the spot; THEN two records this build reads
        // perfectly well, keyed to indices the manifest names nowhere, which
        // under an admitted manifest are resolved, missed and counted.
        let mixed = vec![
            unreadable_record(),
            this_format_record(9),
            this_format_record(11),
        ];
        let refusal = craft_and_read(&manifest_json(Some("2"), 1), &mixed)
            .expect_err("a later format is refused whatever its records hold");
        assert_eq!(
            refusal,
            AnchorReadRefusal::StateRecordFormatTooNew {
                carried: 2,
                known: 1
            },
            "refused by name: {refusal:?}"
        );
        let text = refusal.to_string();
        assert!(
            text.contains("state record format version 2")
                && text.contains("state record format version 1"),
            "and the sentence names BOTH versions: {text}"
        );
        // THE PLACEMENT ASSERTION, and it is the PER RECORD line: a reader that
        // opened even its first record wrote this one, whatever it returned
        // afterwards, so every deferral past that point is caught here.
        assert!(
            !logs_contain("could not be keyed"),
            "the gate returned before the walk opened a single record, so the \
             per record line for the unkeyable one cannot have been written"
        );
        // The weaker companion, and it is kept for the walk it describes rather
        // than for the placement: a gate deferred to just above this report
        // would withhold it too, so it does not stand on its own.
        assert!(
            !logs_contain("no manifest entry names"),
            "and no end of walk report either"
        );

        // THE CONTROL: the same records under a manifest this build admits.
        let read_them = craft_and_read(&manifest_json(Some("1"), 1), &mixed)
            .expect("the same records at this build's version are read");
        assert_eq!(
            read_them.malformed_records, 1,
            "the walk really does open the unkeyable record: {read_them:?}"
        );
        assert_eq!(
            read_them.unattributable_records, 2,
            "and really does resolve the other two and miss: {read_them:?}"
        );
        assert!(
            logs_contain("could not be keyed"),
            "so the per record absence above is this reader not walking, rather \
             than this test not looking"
        );
        assert!(
            logs_contain("no manifest entry names"),
            "and the same for the end of walk report"
        );
    }

    /// The AMBIGUITY refusal still comes FIRST, so a k>1 bag reads the sentence
    /// its own capture manifest predicted rather than a version complaint.
    ///
    /// Order matters here and is not cosmetic: a two-ring bag written by THIS
    /// build carries the format key, so only the order decides which of the two
    /// refusals an operator sees, and the capture judge's `resimmable_reason`
    /// names the ambiguity.
    #[test]
    fn the_ambiguity_refusal_still_precedes_the_record_format_gate() {
        let two_rings = craft_and_read(&manifest_json(Some("1"), 2), &[])
            .expect_err("two rings is still refused");
        assert_eq!(
            two_rings,
            AnchorReadRefusal::MultiRingAmbiguous {
                rings: 2,
                state_record_format: Some(1)
            },
            "{two_rings:?}"
        );
        // And a two-ring bag that ALSO predates the format reads the ambiguity
        // sentence, because that gate is asked first.
        let both = craft_and_read(&manifest_json(None, 2), &[]).expect_err("still refused");
        assert_eq!(
            both,
            AnchorReadRefusal::MultiRingAmbiguous {
                rings: 2,
                state_record_format: None
            },
            "the manifest property is asked before the format: {both:?}"
        );
        // And the arm this fix ADDED, which is the one whose placement was newly
        // decided and the one nothing pinned. A k>1 bag written by a LATER build
        // is two unreadable things at once, and the ambiguity is the one its own
        // capture judge wrote down, so that is the sentence an operator must
        // meet. Hoisting the too-new arm above the ring check answers a version
        // complaint instead, and a build the operator then goes and fetches
        // refuses the bag all over again for the reason nobody mentioned.
        let newer_and_ambiguous =
            craft_and_read(&manifest_json(Some("2"), 2), &[]).expect_err("still refused");
        assert_eq!(
            newer_and_ambiguous,
            AnchorReadRefusal::MultiRingAmbiguous {
                rings: 2,
                state_record_format: Some(2)
            },
            "the ring count is asked before the record format, so the capture's \
             own reason is the one served: {newer_and_ambiguous:?}"
        );
    }

    /// A bag with NO manifest at all keeps its old answer: no anchors, no
    /// refusal.
    ///
    /// It declares nothing, so there is nothing to mis-attribute, and turning
    /// every ordinary pre-checkpoint bag into a refusal would be the
    /// over-refusal this gate must not become.
    #[test]
    fn a_bag_with_no_state_manifest_is_not_refused_by_the_record_format_gate() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("plain.mcap");
        {
            let w = cerulion_bag::BagWriter::create(
                &path,
                cerulion_bag::BagWriterConfig::default(),
                &[],
            )
            .expect("create bag");
            w.finalize().expect("finalize");
        }
        let reader = BagReader::open(&path).expect("open");
        let anchors = read_bag_anchors(&reader).expect("no manifest is no refusal");
        assert!(!anchors.coverage_present);
        assert!(anchors.facts.is_empty());
    }

    /// The catch-up clamp still answers on a bag recorded before the two new
    /// keys, and says nothing about it.
    ///
    /// This is the arm that catches a version of the gate that would break a path
    /// it does not claim to touch. `read_state_arm` shares the parse with
    /// `read_bag_anchors`; declared without a serde default the new keys would be
    /// REQUIRED, the parse would fail on every older bag, and a from-start replay
    /// would start warning that it runs UNCLAMPED, on a path with no other reader
    /// of this attachment at all.
    #[test]
    fn the_catch_up_clamp_still_reads_a_bag_recorded_before_the_two_new_keys() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("armed.mcap");
        let previous = r#"{ "version": 1, "rings_declared": 1, "records": 0,
            "armed": { "tag": "run-1", "cadence_steps": 30000, "first_anchor_step": 7 },
            "nodes": {} }"#;
        {
            let mut w = cerulion_bag::BagWriter::create(
                &path,
                cerulion_bag::BagWriterConfig::default(),
                &[],
            )
            .expect("create bag");
            w.write_attachment(
                STATE_COVERAGE_ATTACHMENT,
                "application/json",
                0,
                0,
                previous.as_bytes(),
            )
            .expect("write the manifest");
            w.finalize().expect("finalize");
        }
        let reader = BagReader::open(&path).expect("open");
        let arm = read_state_arm(&reader).expect("the clamp must still be readable");
        assert_eq!(arm.tag, "run-1");
        assert_eq!(arm.cadence_steps, 30_000);
        assert_eq!(arm.first_anchor_step, 7);
    }
}
