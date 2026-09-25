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
//! - **`rings_declared > 1`.** A state record carries `node_idx` and no rank,
//!   and every ring numbers its own nodes from 0, so with two declared rings a
//!   record with `node_idx = 0` could belong to either ring's first node and
//!   nothing in the bag disambiguates it. `StateNodeCoverage::node_idx`'s own
//!   documentation names this residual and says the restore engine refuses;
//!   this is that refusal. Guessing would apply one node's recorded state to a
//!   DIFFERENT node and then report a confident divergence about it.
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
    #[error(
        "this recording drained {rings} state rings, and a state record carries a node index but \
         no rank — every ring numbers its own nodes from 0, so a record's index names a different \
         node in each of them and nothing in the bag says which ring it came from. Applying one \
         node's recorded state to another would produce a confident divergence report about an \
         execution that never happened. Fix: replay a single-process recording (`cerulion graph \
         run --record --single-process`), or re-record once state records carry their rank"
    )]
    MultiRingAmbiguous {
        /// How many rings the manifest declares.
        rings: usize,
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
///   RELAXES multi-ring (the remedy that refusal itself names — "re-record once
///   state records carry their rank") makes this build refuse a bag a newer one
///   could read. Over-refusal, never a wrong answer.
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
        });
    }
    // The BACKWARD version gate, and it lives HERE rather than in the parse.
    //
    // A state record carries no version of its own before format version 1 and
    // no checksum at any version, so nothing at the RECORD level can catch a bag
    // written under the earlier layout: `StateRecordHeader::from_bytes` takes a
    // fixed-width array and would simply read a different field out of each
    // offset. The only discriminator is the BAG's, which is why the manifest
    // carries the writer's record format and why an absent key is refused rather
    // than read as "version 0": a manifest that predates the key and a manifest
    // whose writer forgot it are the same bytes, and the safe reading of both is
    // a refusal.
    //
    // It is in `read_bag_anchors` and NOT in `read_state_coverage`, which
    // `read_state_arm` shares: a refusal at the parse would take the catch-up
    // clamp away from every bag recorded before this format, on the from-start
    // path, which this gate has no business touching.
    let known = cerulion_core::state_ring::STATE_RECORD_FORMAT_VERSION;
    match coverage.state_record_format_version {
        Some(v) if v >= known => {}
        carried => return Err(AnchorReadRefusal::StateRecordFormatTooOld { carried, known }),
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
        };
        let text = refusal.to_string();
        assert!(text.contains("2 state rings"), "{text}");
        assert!(text.contains("--single-process"), "names the fix: {text}");
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
        assert!(
            matches!(
                two_rings,
                AnchorReadRefusal::MultiRingAmbiguous { rings: 2 }
            ),
            "{two_rings:?}"
        );
        // And a two-ring bag that ALSO predates the format reads the ambiguity
        // sentence, because that gate is asked first.
        let both = craft_and_read(&manifest_json(None, 2), &[]).expect_err("still refused");
        assert!(
            matches!(both, AnchorReadRefusal::MultiRingAmbiguous { rings: 2 }),
            "the manifest property is asked before the format: {both:?}"
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
