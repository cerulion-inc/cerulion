// SPDX-License-Identifier: AGPL-3.0-only
//! What a recording says about the node-state anchors it carries: the
//! recorder-side ledger and the `__cerulion/state_coverage.json` attachment it
//! becomes.
//!
//! # Why a LEDGER and not [`cerulion_core::state_ring::StateAssembler`]
//!
//! The assembler is the RESTORE-side reader: it reassembles each anchor into an
//! owned `Vec<u8>` so a caller can hand the bytes to a decoder. The recorder
//! must never do that. The RAM budget is already the design's
//! weakest point (~2 x state in the ring plus the parent's CoW spike), and it
//! contains no term for a SECOND full copy of a 500 MB anchor sitting in the
//! recorder — the whole reason the child writes straight into `MAP_SHARED` is to
//! delete exactly that copy.
//!
//! So this module reads HEADERS and counts. It applies the same rules the
//! assembler applies — same part ordering, same short-chunk refusal, same
//! "one broken anchor yields one verdict, not one per surviving record", same
//! mid-run-attach arming — and it holds no payload. The BYTES go through to the
//! bag verbatim either way; what differs is only what the recorder is able to
//! SAY about them.
//!
//! # The arming rule, and why the bag still carries the records it discards
//!
//! [`StateRingConsumer::open_at_live`](cerulion_core::state_ring::StateRingConsumer::open_at_live)
//! lands the read cursor MID-BLOB, so the first records a mid-run recorder sees
//! can be the tail of an anchor whose head was committed before the attach.
//! Calling that TORN would cry wolf on every mid-run attach, so an ARMED ledger
//! discards records until the first `part == 0` — byte-for-byte the rule
//! `StateAssembler::armed` states, and monotone for the same reason.
//!
//! Those discarded records are still WRITTEN to the bag. That is deliberate: the
//! recorder does not decode, a headless anchor is detectable from the records
//! themselves at read time, and the one place the discard rule lives is the
//! assembler a reader already runs. What the coverage manifest owes in exchange
//! is the COUNT, which is
//! [`head_records_discarded`](StateCoverage::head_records_discarded) — so the
//! extra records in the bag are explained rather than merely present.

use std::collections::BTreeMap;

use cerulion_core::state_ring::{
    SkipCause, StateRecordHeader, RECORD_KIND_FINAL, RECORD_KIND_SKIP, STATE_RECORD_HEADER_SIZE,
    STATE_RECORD_PAYLOAD, STATE_RECORD_SIZE,
};
use serde::{Deserialize, Serialize};

/// The `__cerulion/state_coverage.json` attachment name.
pub const STATE_COVERAGE_ATTACHMENT: &str = "__cerulion/state_coverage.json";

/// The [`StateCoverage`] wire version.
pub const STATE_COVERAGE_VERSION: u32 = 1;

// ===========================================================================
// The ledger — pure, header-only
// ===========================================================================

/// Per-anchor reassembly state, without the bytes.
#[derive(Debug)]
enum AnchorState {
    /// Still whole: the part index this stream owes next.
    Open { next_part: u32, bytes: u64 },
    /// Already counted TORN — swallow the rest of this anchor's records so one
    /// broken anchor yields ONE verdict, not one per surviving record.
    Voided,
}

/// One node's anchor tally within one ring.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct NodeTally {
    /// Anchors that reached their FINAL record with every part in order.
    pub complete: u64,
    /// Anchors a lost/duplicated/reordered/short record broke, plus anchors the
    /// record stream simply stopped in the middle of.
    pub torn: u64,
    /// Anchors the WRITER voided on purpose, with a named cause.
    pub skipped: u64,
    /// The step of the newest COMPLETE anchor — what a restore can actually
    /// resume from.
    pub last_complete_step: Option<u64>,
    /// Payload bytes across every COMPLETE anchor (framing excluded).
    pub bytes: u64,
    /// Skip causes, by wire discriminant, with their counts.
    pub skip_causes: BTreeMap<u32, u64>,
    /// Precedence: SKIPs that arrived for an anchor this ring had ALREADY
    /// completed — counted here rather than as `skipped`.
    ///
    /// The capture child publishes a node's final record and THEN bumps its
    /// accounting word, so a kill between the two leaves real data in the ring
    /// and a parent post-mortem that skips the same key. The anchor is
    /// authoritative; folding its skip into `skipped` would make a bag report a
    /// voided anchor it can serve perfectly well, and would count one node's one
    /// cadence twice.
    pub skipped_after_complete: u64,
}

/// The header-only anchor accounting for ONE state ring.
///
/// One ledger per ring, held beside the consumer it judges for the same reason
/// `SendRing` holds its `HeadStepGate` there: a parallel vector would have to
/// stay index-aligned across every future edit, and a misalignment would
/// silently attribute one ring's anchors to another's node table.
#[derive(Debug)]
pub struct StateAnchorLedger {
    armed: bool,
    discarded: u64,
    records: u64,
    malformed: u64,
    truncated: u64,
    open: BTreeMap<(u64, u64, u32), AnchorState>,
    nodes: BTreeMap<u32, NodeTally>,
    /// The run every record in this ring belongs to, learned from the first one.
    ///
    /// `nodes` is keyed by `node_idx` ALONE, so every per-node number it holds —
    /// `complete`, `torn`, `skipped`, `bytes`, `last_complete_step`, and the
    /// precedence check that reads that step — is an accounting over ONE run. That is
    /// true by construction (one ledger per ring, one ring per `StateRingOwner`, one
    /// `run_id` per owner, and every production write path stamps it), so this field
    /// does not fix a live bug: it makes the assumption CHECKABLE instead of implicit,
    /// which is the difference between an invariant and a hope.
    run_id: Option<u64>,
    /// Records whose `run_id` is not this ring's — refused rather than tallied.
    ///
    /// Merging them would blend two runs' anchors into one node's numbers, which is
    /// worse than reporting nothing: a reader cannot un-blend them, and the blend
    /// looks exactly like a healthy tally.
    foreign_run_records: u64,
}

impl Default for StateAnchorLedger {
    fn default() -> Self {
        Self::passthrough()
    }
}

impl StateAnchorLedger {
    /// A ledger for a ring read from its START: every record is judged, and a
    /// headless anchor is TORN.
    pub fn passthrough() -> Self {
        Self {
            armed: false,
            discarded: 0,
            records: 0,
            malformed: 0,
            truncated: 0,
            open: BTreeMap::new(),
            nodes: BTreeMap::new(),
            run_id: None,
            foreign_run_records: 0,
        }
    }

    /// A ledger for a MID-RUN attach: discards records until the first
    /// `part == 0`, so the partial head anchor the live cursor landed inside is
    /// dropped rather than misreported as corruption.
    pub fn armed() -> Self {
        Self {
            armed: true,
            ..Self::passthrough()
        }
    }

    /// Choose the ledger a ring's OPEN policy implies.
    ///
    /// Paired with the consumer constructor rather than left to the call site,
    /// because an `open_at_live` ring judged by a passthrough ledger reports a
    /// torn anchor on every mid-run attach and an `open` ring judged by an armed
    /// one silently drops a real leading anchor.
    pub fn for_open(attached_mid_run: bool) -> Self {
        if attached_mid_run {
            Self::armed()
        } else {
            Self::passthrough()
        }
    }

    /// Offer one whole record's BYTES, in ring order.
    ///
    /// Reads the 32-byte header (and, for a SKIP, its 4-byte cause word) and
    /// nothing else — the payload is never retained.
    pub fn feed(&mut self, record: &[u8]) {
        if record.len() < STATE_RECORD_SIZE as usize {
            self.malformed += 1;
            return;
        }
        let mut hb = [0u8; STATE_RECORD_HEADER_SIZE];
        hb.copy_from_slice(&record[..STATE_RECORD_HEADER_SIZE]);
        let header = StateRecordHeader::from_bytes(&hb);
        if header.validate().is_err() {
            self.malformed += 1;
            return;
        }
        // ONE ring, ONE run — see `run_id`. A record from another run cannot be
        // attributed by a node table this one keys by index alone, so it is refused
        // and counted rather than folded into someone else's anchors.
        match self.run_id {
            None => self.run_id = Some(header.run_id),
            Some(known) if known != header.run_id => {
                self.foreign_run_records += 1;
                return;
            }
            Some(_) => {}
        }
        // The partial head anchor of a mid-run attach. Monotone — the first
        // `part == 0` opens the ledger for good.
        if self.armed {
            if header.part != 0 {
                self.discarded += 1;
                return;
            }
            self.armed = false;
        }
        self.records += 1;
        let key = (header.run_id, header.step, header.node_idx);

        if header.kind == RECORD_KIND_SKIP {
            // A skip is self-contained and AUTHORITATIVE: whatever was in flight
            // for this anchor is void, and the writer just said why.
            self.open.remove(&key);
            let cause = u32::from_le_bytes(
                record[STATE_RECORD_HEADER_SIZE..STATE_RECORD_HEADER_SIZE + 4]
                    .try_into()
                    .expect("4-byte slice"),
            );
            let tally = self.nodes.entry(header.node_idx).or_default();
            // Precedence: a skip for an anchor this node already COMPLETED at
            // this step is the parent's post-mortem disagreeing with data the child
            // really published. Counted as the contradiction it is, never as a
            // voided anchor — the anchor is intact and a restore will use it.
            if tally.last_complete_step == Some(header.step) {
                tally.skipped_after_complete += 1;
                return;
            }
            tally.skipped += 1;
            *tally.skip_causes.entry(cause).or_default() += 1;
            return;
        }

        let is_final = header.kind == RECORD_KIND_FINAL;
        match self.open.get_mut(&key) {
            None => {
                if header.part != 0 {
                    if !is_final {
                        self.open.insert(key, AnchorState::Voided);
                    }
                    self.tear(header.node_idx);
                    return;
                }
                if !is_final && header.len as usize != STATE_RECORD_PAYLOAD {
                    self.open.insert(key, AnchorState::Voided);
                    self.tear(header.node_idx);
                    return;
                }
                if is_final {
                    self.complete(header.node_idx, header.step, header.len as u64);
                } else {
                    self.open.insert(
                        key,
                        AnchorState::Open {
                            next_part: 1,
                            bytes: header.len as u64,
                        },
                    );
                }
            }
            Some(AnchorState::Voided) => {
                // Already counted. The FINAL record CLOSES the tombstone, so a
                // later anchor under the same key starts clean —
                // `StateAssembler`'s rule, verbatim. Leaving it standing made
                // the void terminal for that `(run, step, node)` forever: a
                // re-sent anchor would be swallowed with no verdict at all, and
                // on a long run with a flaky node the tombstones accumulated
                // unboundedly because nothing else ever removes one.
                if is_final {
                    self.open.remove(&key);
                }
            }
            Some(AnchorState::Open { next_part, bytes }) => {
                if header.part != *next_part {
                    // A FINAL closes the anchor whatever its verdict — it is the
                    // last record this key will carry, so a tombstone left
                    // behind could only ever swallow a LATER one. Again the
                    // assembler's rule.
                    if is_final {
                        self.open.remove(&key);
                    } else {
                        self.open.insert(key, AnchorState::Voided);
                    }
                    self.tear(header.node_idx);
                    return;
                }
                if !is_final && header.len as usize != STATE_RECORD_PAYLOAD {
                    self.open.insert(key, AnchorState::Voided);
                    self.tear(header.node_idx);
                    return;
                }
                let total = *bytes + header.len as u64;
                if is_final {
                    self.open.remove(&key);
                    self.complete(header.node_idx, header.step, total);
                } else {
                    *next_part += 1;
                    *bytes = total;
                }
            }
        }
    }

    /// Close the ledger: every anchor still open when the stream ended is TORN
    /// (`Truncated` — the writer died mid-encode, or the recorder stopped
    /// draining before the anchor finished).
    ///
    /// Idempotent, so a caller that finishes twice does not double-count.
    pub fn finish(&mut self) {
        let open: Vec<(u64, u64, u32)> = self
            .open
            .iter()
            .filter(|(_, v)| matches!(v, AnchorState::Open { .. }))
            .map(|(k, _)| *k)
            .collect();
        for key in open {
            self.open.remove(&key);
            self.truncated += 1;
            self.tear(key.2);
        }
        self.open.clear();
    }

    fn tear(&mut self, node_idx: u32) {
        self.nodes.entry(node_idx).or_default().torn += 1;
    }

    fn complete(&mut self, node_idx: u32, step: u64, bytes: u64) {
        let tally = self.nodes.entry(node_idx).or_default();
        tally.complete += 1;
        tally.bytes += bytes;
        tally.last_complete_step = Some(match tally.last_complete_step {
            Some(prev) => prev.max(step),
            None => step,
        });
    }

    /// State records this ledger judged (the discarded head is NOT counted).
    pub fn records(&self) -> u64 {
        self.records
    }

    /// Leading records dropped as a mid-run attach's partial head anchor.
    pub fn discarded(&self) -> u64 {
        self.discarded
    }

    /// Records refused because they name a run other than this ring's (see
    /// [`StateAnchorLedger`]'s `run_id`).
    ///
    /// Zero in every shipping shape. Non-zero means one ledger was fed two runs and
    /// REFUSED the foreign records — the refusal happens before any tally, so the
    /// per-node numbers this ledger holds stay scoped to its own ring's run and remain
    /// valid. What is lost is the refused records themselves: they were in the ring and
    /// are accounted for nowhere.
    pub fn foreign_run_records(&self) -> u64 {
        self.foreign_run_records
    }

    /// Records this build could not even key (short, or an unknown/zeroed kind).
    pub fn malformed(&self) -> u64 {
        self.malformed
    }

    /// Anchors that were still open when the stream ended.
    pub fn truncated(&self) -> u64 {
        self.truncated
    }

    /// Whether the ledger is still discarding a partial head anchor.
    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// The per-node tallies, by manifest index.
    pub fn nodes(&self) -> &BTreeMap<u32, NodeTally> {
        &self.nodes
    }

    /// Anchors this ledger is still carrying state for — open ones and
    /// not-yet-closed tombstones.
    ///
    /// Test-only, and it exists because a VERDICT comparison cannot see a
    /// tombstone that is never removed: the counts agree either way, and what
    /// differs is whether the map grows for the life of the run.
    #[cfg(test)]
    pub(crate) fn open_keys_for_test(&self) -> usize {
        self.open.len()
    }
}

// ===========================================================================
// The attachment
// ===========================================================================

/// The capture plane this recording DRAINED, as its arm word described it.
///
/// It does NOT mean "what the recorder armed":
/// no recorder arms a plane — the plane belongs to the graph, and a
/// recorder reads its word rather than creating one. So this states a fact
/// about the plane whose anchors are in this bag, which is strictly more useful:
/// it is the cadence the anchors were REALLY taken at, not the cadence some
/// process asked for.
///
/// `None` on the coverage means the recorder never saw an arm word — a run whose
/// Flashback plane was refused or switched off, a bag predating that change, or a
/// word that had not appeared by the time the bag was created. It is therefore
/// read as "this bag makes no claim about a cadence", and
/// [`StateCoverage::is_incomplete`] does not escalate a node with no anchor on
/// it, which is the safe direction.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StateArmCoverage {
    /// The mapped-SHM tag the arm word lives under.
    pub tag: String,
    /// Boundaries between anchors.
    pub cadence_steps: u64,
    /// The first boundary at which an anchor is due.
    pub first_anchor_step: u64,
}

/// One node's anchor coverage, as the attachment records it.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct StateNodeCoverage {
    /// The ring this node's anchors came from.
    pub ring: String,
    /// This node's index IN ITS RING'S MANIFEST — the `node_idx` every state
    /// record carries.
    ///
    /// It is here because NOTHING ELSE carries the index into the bag: a state
    /// record has a `node_idx` and no node id, this map is keyed by node ID, and
    /// the ring manifest that relates the two never leaves the live ring. A
    /// restore engine reading a finished bag therefore had no way at all to say
    /// WHICH node a record belongs to — the conversion that built this map was
    /// the last place the index existed, and it dropped it.
    ///
    /// ADDITIVE and OPTIONAL: `None` means a writer predating this field
    /// produced the manifest, never "this node has no index". No
    /// [`STATE_COVERAGE_VERSION`] bump — an absent key decodes to `None` on an
    /// old bag, which is the house pattern for an additive field.
    ///
    /// RESIDUAL, relevant to the restore engine and deliberately NOT closed
    /// here: with `rings_declared > 1` the `node_idx → node` mapping is still
    /// ambiguous BAG-SIDE, because a record carries no rank. Two workers' rings
    /// both number their nodes from 0, so a record with `node_idx = 0` could be
    /// either ring's first node and nothing in the bag disambiguates it — the
    /// [`ring`](Self::ring) field answers "which ring is this NODE in", not
    /// "which ring did this RECORD come from". The restore engine REFUSES a
    /// multi-ring bag for now; this field makes the single-ring case — every
    /// shipping shape today — exact. Closing it needs a rank on the record or on
    /// the channel, which is a wire change and belongs with the restore work.
    ///
    /// A node id declared by TWO rings at DIFFERENT indices serves `None`: the
    /// index is then not a fact this bag can state, and the alternative is
    /// letting whichever ring was walked last silently win.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub node_idx: Option<u32>,
    /// Anchors that reached their FINAL record with every part present.
    pub anchors_complete: u64,
    /// Anchors broken by a lost/duplicated/reordered/short record, or left open
    /// when the stream ended.
    #[serde(default, skip_serializing_if = "is_zero_u64")]
    pub anchors_torn: u64,
    /// Anchors the writer VOIDED on purpose.
    #[serde(default, skip_serializing_if = "is_zero_u64")]
    pub anchors_skipped: u64,
    /// Precedence: SKIPs that named an anchor this bag CARRIES complete.
    ///
    /// Not an anchor this recording lost — the anchor is intact and a restore will
    /// use it. It is evidence that a capture child was killed between publishing a
    /// node's final record and recording that it had, which is worth reading beside
    /// a run that ended badly. Absent from an ordinary bag (additive, defaulted).
    #[serde(default, skip_serializing_if = "is_zero_u64")]
    pub anchors_skipped_after_complete: u64,
    /// The newest step a COMPLETE anchor covers — what a restore can resume
    /// from. `None` means this node has no anchor in this bag at all.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_complete_step: Option<u64>,
    /// Payload bytes across every complete anchor (framing excluded).
    #[serde(default, skip_serializing_if = "is_zero_u64")]
    pub bytes: u64,
    /// Skip causes by NAME, with their counts — an unrecognised code keeps its
    /// number so a future cause survives to the operator rather than being
    /// rounded off to "unknown".
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub skip_causes: BTreeMap<String, u64>,
}

fn is_zero_u64(v: &u64) -> bool {
    *v == 0
}

fn is_false(v: &bool) -> bool {
    !*v
}

/// What this recording says about the node-state anchors it holds.
///
/// Written ONLY when the recording was configured for checkpoints (a state ring
/// was declared, or this recorder armed a checkpoint word). An ordinary bag
/// carries no such attachment and is byte-identical to one written before
/// checkpoints existed. This is the `mirrors_established: None` posture: an
/// absent manifest is NO CLAIM, never a clean-coverage claim, and `bag info`
/// says exactly that.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StateCoverage {
    /// [`STATE_COVERAGE_VERSION`].
    pub version: u32,
    /// Whether this recorder attached to a run already in flight, so its rings
    /// were opened at the LIVE cursor and anchors before the attach are outside
    /// the window (never "lost").
    #[serde(default, skip_serializing_if = "is_false")]
    pub attached_mid_run: bool,
    /// What this recorder armed, if it armed anything.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub armed: Option<StateArmCoverage>,
    /// State rings this recorder was ASKED to drain.
    pub rings_declared: usize,
    /// The producer RANKS whose state rings this recording actually
    /// drained, ascending.
    ///
    /// Under a `process_groups:` (or auto-derived) deployment the rank COUNT
    /// is decided at run time — after the recorder's argv was fixed — so per-rank
    /// rings cannot be declared and are DISCOVERED by name. This is what was
    /// found, and it is the only place a reader can learn how many processes the
    /// recording's anchors came from.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub ranks_discovered: Vec<u32>,
    /// Ranks that provably EXIST and published no ring.
    ///
    /// Ranks are dense, so a hole below the highest discovered rank is evidence,
    /// not absence of it — and a graph-wide anchor is all-or-nothing across ranks
    /// so a single missing rank makes every anchor of the run partial.
    /// Non-empty makes the recording INCOMPLETE.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub ranks_missing: Vec<u32>,
    /// Declared rings that could not be opened, with the reason — the same
    /// report the trace rings get, for the same reason: a bag that was asked for
    /// anchors and could not look must say so.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub rings_unavailable: BTreeMap<String, String>,
    /// State records written to the `__cerulion/state` channel.
    ///
    /// WRITTEN, which is a SUPERSET of what the ledger judged: every record the
    /// recorder wrote is offered to the ledger, and one that is
    /// [`malformed`](Self::malformed_records) or discarded as a mid-run
    /// attach's [partial head](Self::head_records_discarded) is in the bag while
    /// contributing to no node's tally. So
    /// `records == judged + head_records_discarded + malformed_records`, and
    /// this number equals `BagdSummary::state_records` for the same recording —
    /// a bag that claimed more records than it holds would be exactly the
    /// silent-loss report the topic-level record coverage closed one layer up.
    pub records: u64,
    /// Leading records the mid-run attach discarded as a partial head anchor.
    /// They ARE in the bag (see the module docs); this is what explains them.
    #[serde(default, skip_serializing_if = "is_zero_u64")]
    pub head_records_discarded: u64,
    /// Records this recorder could not key at all.
    #[serde(default, skip_serializing_if = "is_zero_u64")]
    pub malformed_records: u64,
    /// Records a ring's ledger refused because they named a RUN other than that
    /// ring's.
    ///
    /// Zero in every shipping shape — one ledger judges one ring, and a ring carries
    /// one run — so this is absent from an ordinary bag. Non-zero means a ledger was
    /// fed two runs and REFUSED the foreign records, before any tally: the per-node
    /// numbers below stay scoped to the ring's own run and remain valid. What this
    /// recording loses is COMPLETENESS — those records were in the ring and are
    /// accounted for nowhere in the bag — which is why it escalates.
    #[serde(default, skip_serializing_if = "is_zero_u64")]
    pub foreign_run_records: u64,
    /// Every node the drained rings' MANIFESTS declare, whether or not it
    /// anchored — a node with no anchor is exactly what `is_incomplete` exists
    /// to surface, and a map built only from records that arrived could never
    /// contain it.
    ///
    /// Keyed by node ID, and it holds ONLY manifest-declared nodes. Records
    /// carrying an index no manifest declares live in
    /// [`unattributed_indices`](Self::unattributed_indices) — see there for why
    /// they are not in here.
    pub nodes: BTreeMap<String, StateNodeCoverage>,
    /// Anchor tallies for a `node_idx` NO ring manifest declares.
    ///
    /// Such a record is still evidence and is reported rather than dropped — a
    /// reader that silently discards what it cannot name is how an anchor goes
    /// missing without anyone being told. What it must NOT do is share a
    /// keyspace with real node ids.
    ///
    /// Filing the entry in [`nodes`](Self::nodes) under a
    /// SYNTHETIC string, `node_idx {i} (not in the ring manifest)`, would. Node ids
    /// come from graph YAML and are **completely unvalidated** — MEASURED:
    /// `validate_graph` accepts a node whose id is that exact string, and checks
    /// node ids for DUPLICATES and nothing else (no charset rule, not even the
    /// zenoh-reserved set the graph PREFIX is checked against). So a real node
    /// so named would collide with the synthetic entry, and the unknown-index walk
    /// would overwrite that node's [`node_idx`](StateNodeCoverage::node_idx) and
    /// merge a stranger's anchor tallies into its row — on the very field a
    /// restore engine uses to decide which node a record belongs to.
    ///
    /// An index-keyed map rather than an escaped or prefixed string key: no
    /// character is reserved in a node id, so no prefix is safe, while a
    /// `String` can never be a key in an index-keyed map. The disjointness is
    /// the TYPE, which is the one form of it a future edit cannot erode.
    ///
    /// NESTED BY RING, because `node_idx` is scoped to ONE ring's manifest and
    /// nothing else. Every ring numbers its own nodes from 0, so two declared
    /// rings each emitting an unknown index 1 are two unrelated facts: a single
    /// `u32` key merged their tallies into one row and let whichever ring was
    /// walked last overwrite [`ring`](StateNodeCoverage::ring), reporting one
    /// ring's tears against the other's name. The outer key is the DECLARED ring
    /// name and is authoritative; the inner `ring` field repeats it so the value
    /// type is the same one [`nodes`](Self::nodes) uses.
    ///
    /// JSON shape: a map of maps — `{"<ring>": {"<index>": { … }}}`. The index
    /// keys are JSON strings because JSON object keys always are; serde restores
    /// them to `u32`. A composite string key (`"<ring>#<index>"`) was rejected
    /// for the same reason the synthetic node key was: a ring NAME is not a
    /// validated charset either, so a separator can appear inside one.
    ///
    /// Additive: absent decodes to empty, so an old bag reads exactly as before
    /// and no [`STATE_COVERAGE_VERSION`] bump is owed.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub unattributed_indices: BTreeMap<String, BTreeMap<u32, StateNodeCoverage>>,
}

impl StateCoverage {
    /// The manifest for a FLASHBACK CAPTURE — a bag holding
    /// exactly ONE checkpoint rather than a run's whole anchor stream.
    ///
    /// It lives here, beside the type, rather than being hand-built at the
    /// capture writer: this attachment is what makes a bag's state records
    /// READABLE at all (`replay_state::read_bag_anchors` returns nothing without
    /// it, and takes the `node_idx → node id` table from `nodes`), so a second
    /// copy of its shape would be two copies of one rule, free to drift apart,
    /// on the one artifact a resume depends on.
    ///
    /// # Three fields carry the whole contract, and each is a claim
    ///
    /// * `attached_mid_run: false` — a capture writes WHOLE anchors from part 0,
    ///   so a reader must judge every record (`StateAssembler::passthrough`).
    ///   Claiming a mid-run attach would make it DISCARD the head of the very
    ///   anchor the capture exists to carry.
    /// * `rings_declared` — the number of DISTINCT rings the embedded anchors
    ///   came from. A restore engine refuses `> 1` because `node_idx` carries no
    ///   rank, so this is an explicit refusal rather than an ambiguous read.
    /// * `armed` — the graph's cadence, carried through so a capture says what
    ///   plane produced it.
    ///
    /// `records` counts what the capture WRITES, so the manifest's own
    /// `records == judged + discarded + malformed` invariant holds on a bag
    /// whose stream is one checkpoint long.
    pub fn for_capture(
        armed: Option<StateArmCoverage>,
        rings_declared: usize,
        records: u64,
        nodes: BTreeMap<String, StateNodeCoverage>,
        unattributed_indices: BTreeMap<String, BTreeMap<u32, StateNodeCoverage>>,
    ) -> Self {
        Self {
            version: STATE_COVERAGE_VERSION,
            // See the doc above: a capture's records begin at part 0.
            attached_mid_run: false,
            armed,
            rings_declared,
            // A capture makes no claim about a run's RANK SPACE: it walked no
            // rank space, so it can witness neither density nor a hole. Empty is
            // the correct answer, not zero-ranks-found.
            ranks_discovered: Vec::new(),
            ranks_missing: Vec::new(),
            rings_unavailable: BTreeMap::new(),
            records,
            head_records_discarded: 0,
            malformed_records: 0,
            foreign_run_records: 0,
            nodes,
            unattributed_indices,
        }
    }

    /// Nodes that ended the run with NO complete anchor.
    ///
    /// The escalation term: a bag in which some node never anchored cannot be
    /// used to resume that node, whatever else it contains.
    /// Deliberately over `nodes` ONLY: this is the "a node that was DUE to
    /// anchor never did" escalation, and an unattributed index is by definition
    /// not a declared node — it exists only because records arrived for it, so
    /// counting one here would report a starved node that no manifest ever
    /// promised.
    pub fn nodes_without_anchor(&self) -> usize {
        self.nodes
            .values()
            .filter(|n| n.anchors_complete == 0)
            .count()
    }

    /// Anchors broken across every node — INCLUDING records no manifest could
    /// name, which are broken anchors whoever they belonged to.
    ///
    /// Splitting the keyspace must not lose an escalation: before the split
    /// these entries were in `nodes` and counted here, and a tear that stopped
    /// being counted would quietly turn an INCOMPLETE recording into a clean-
    /// looking one.
    pub fn torn_anchors(&self) -> u64 {
        self.nodes
            .values()
            .chain(
                self.unattributed_indices
                    .values()
                    .flat_map(BTreeMap::values),
            )
            .map(|n| n.anchors_torn)
            .sum()
    }

    /// Anchors the writer voided on purpose, across every node and every
    /// unattributed index (same reasoning as [`torn_anchors`](Self::torn_anchors)).
    pub fn skipped_anchors(&self) -> u64 {
        self.nodes
            .values()
            .chain(
                self.unattributed_indices
                    .values()
                    .flat_map(BTreeMap::values),
            )
            .map(|n| n.anchors_skipped)
            .sum()
    }

    /// Whether this recording's CHECKPOINT coverage is incomplete.
    ///
    /// Gated on [`armed`](Self::armed), on the `schema_demand_requested`
    /// precedent: a recorder that merely drained whatever rings it was handed
    /// asked for no cadence and failed at nothing, so it must not escalate a
    /// terminal line on a node that was never due to anchor. A recorder that
    /// ARMED the word did ask, and a node that never answered is the exact
    /// condition an operator must read on the day it starts being true.
    ///
    /// A declared ring that could not be OPENED escalates whether or not this
    /// recorder armed: it was asked for those anchors and could not even look.
    /// A SKIPPED anchor deliberately does NOT escalate on its own — a skip is
    /// the mechanism working (the writer named a cause and the next cadence
    /// retries), and escalating every contended probe would train the operator
    /// to skim the line that matters.
    pub fn is_incomplete(&self) -> bool {
        !self.incomplete_reasons().is_empty()
    }

    /// EVERY reason this recording's checkpoint coverage is incomplete, in the order
    /// an operator should read them.
    ///
    /// # Why the reasons are a VALUE and not a boolean plus a log
    ///
    /// If [`is_incomplete`](Self::is_incomplete) and the operator's terminal line were
    /// two independent lists of conditions, they would drift: a terminal
    /// `else if` chain with no arm for `foreign_run_records` or for `ranks_missing` means
    /// a bag whose `state_coverage.json` says INCOMPLETE prints the clean "every node
    /// ... has an anchor" line to the person reading the run. The manifest and the
    /// terminal are both surfaces of the same judgement, and the terminal is the one an
    /// operator actually reads.
    ///
    /// So the judgement is made ONCE, here, and rendered by
    /// [`log_state_coverage_terminal`]. Its `match` is exhaustive over
    /// [`IncompleteReason`], which is what makes the next condition unable to miss the
    /// terminal: adding a variant without a line does not compile.
    ///
    /// ALL of them, not the first: several can hold at once, and an operator told only
    /// about the ring that would not open will go looking for that and never learn a
    /// rank published nothing.
    pub fn incomplete_reasons(&self) -> Vec<IncompleteReason> {
        let mut out = Vec::new();
        if !self.rings_unavailable.is_empty() {
            out.push(IncompleteReason::RingsUnavailable);
        }
        // Records the ledger REFUSED as another run's, before any tally. The per-node
        // numbers stay valid; what is missing is the refused records, which were in
        // the ring and are accounted for nowhere — so this is not complete coverage.
        if self.foreign_run_records > 0 {
            out.push(IncompleteReason::ForeignRunRecords);
        }
        if self.torn_anchors() > 0 {
            out.push(IncompleteReason::TornAnchors);
        }
        if self.malformed_records > 0 {
            out.push(IncompleteReason::MalformedRecords);
        }
        if self.armed.is_some() && self.nodes_without_anchor() > 0 {
            out.push(IncompleteReason::NodesWithoutAnchor);
        }
        // A rank that exists and published no ring voids every
        // graph-wide anchor of the run (all-or-nothing), so this cannot
        // be a note beside a bag that otherwise reads complete.
        if !self.ranks_missing.is_empty() {
            out.push(IncompleteReason::RanksMissing);
        }
        out
    }
}

/// One reason a recording's checkpoint coverage is INCOMPLETE.
///
/// The single list both surfaces judge from — see
/// [`StateCoverage::incomplete_reasons`]. Every variant must be renderable by
/// [`log_state_coverage_terminal`], and the compiler enforces that.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IncompleteReason {
    /// A ring this recording DECLARED could not be opened.
    RingsUnavailable,
    /// A ledger was fed records naming a run other than its ring's.
    ForeignRunRecords,
    /// An anchor is missing records, so it can never be served.
    TornAnchors,
    /// A record this recording could not decode at all.
    MalformedRecords,
    /// This recorder ARMED capture and some node ended the run with no anchor.
    NodesWithoutAnchor,
    /// A rank below the highest discovered one published no ring.
    RanksMissing,
}

/// Report a recording's checkpoint coverage on the operator's terminal.
///
/// ONE line per reason, at `warn!`, or a single `info!` when there is none. The
/// `match` is EXHAUSTIVE over [`IncompleteReason`] on purpose: it is the structural
/// half of the guarantee that the manifest and the terminal cannot disagree again.
///
/// Lives here, beside the judgement it renders, rather than on `Recorder` — it reads
/// nothing but the coverage, and keeping the two apart is how they drifted.
pub fn log_state_coverage_terminal(sc: &StateCoverage) {
    let reasons = sc.incomplete_reasons();
    if reasons.is_empty() {
        // The clean verdict: `debug!`. Every INCOMPLETE reason below is a warn
        // naming what the bag cannot be used for.
        tracing::debug!(
            nodes = sc.nodes.len(),
            records = sc.records,
            skipped = sc.skipped_anchors(),
            head_records_discarded = sc.head_records_discarded,
            armed = sc.armed.is_some(),
            "bagd checkpoint coverage: every node the drained rings declare has an anchor in \
             this bag"
        );
        return;
    }
    for reason in reasons {
        match reason {
            IncompleteReason::RingsUnavailable => tracing::warn!(
                rings_declared = sc.rings_declared,
                rings_unavailable = sc.rings_unavailable.len(),
                records = sc.records,
                "bagd checkpoint coverage INCOMPLETE: a node-state ring this recording \
                 DECLARED could not be opened, so anchors it carried are not in this bag \
                 (see state_coverage.json's rings_unavailable)"
            ),
            IncompleteReason::ForeignRunRecords => tracing::warn!(
                foreign_run_records = sc.foreign_run_records,
                records = sc.records,
                "bagd checkpoint coverage INCOMPLETE: a ring's ledger was fed records naming \
                 a DIFFERENT run and REFUSED them — one ledger judges one ring, so they were \
                 never tallied. The per-node numbers in this manifest are UNAFFECTED and stay \
                 scoped to this ring's own run; what this bag cannot claim is COMPLETE \
                 coverage, because those records were in the ring and are accounted for \
                 nowhere in it (see state_coverage.json's foreign_run_records)"
            ),
            IncompleteReason::TornAnchors => tracing::warn!(
                records = sc.records,
                torn = sc.torn_anchors(),
                "bagd checkpoint coverage INCOMPLETE: a node-state anchor is missing records, \
                 so it can never be served — a torn anchor is reported, NEVER restored short \
                 (see state_coverage.json)"
            ),
            IncompleteReason::MalformedRecords => tracing::warn!(
                records = sc.records,
                malformed = sc.malformed_records,
                "bagd checkpoint coverage INCOMPLETE: a node-state record could not be decoded \
                 at all, so whatever anchor it belonged to is unreadable (see \
                 state_coverage.json)"
            ),
            IncompleteReason::NodesWithoutAnchor => tracing::warn!(
                nodes = sc.nodes.len(),
                nodes_without_anchor = sc.nodes_without_anchor(),
                records = sc.records,
                skipped = sc.skipped_anchors(),
                "bagd checkpoint coverage INCOMPLETE: this recorder ARMED node-state capture \
                 and some node ended the run with NO anchor, so this bag cannot be used to \
                 resume it (see state_coverage.json)"
            ),
            IncompleteReason::RanksMissing => tracing::warn!(
                ranks_missing = ?sc.ranks_missing,
                ranks_discovered = ?sc.ranks_discovered,
                records = sc.records,
                "bagd checkpoint coverage INCOMPLETE: a rank BELOW the highest one discovered \
                 published no state ring, and a graph-wide anchor is all-or-nothing across \
                 ranks — so every anchor of this run is partial (see state_coverage.json's \
                 ranks_missing)"
            ),
        }
    }
}

/// Render a skip cause's wire discriminant as the name the manifest carries.
///
/// An unrecognised code keeps its number rather than collapsing to "unknown":
/// a future cause an old reader meets is still evidence, and rounding it off is
/// how a reader stops being able to tell two failures apart.
pub fn skip_cause_name(raw: u32) -> String {
    match SkipCause::from_wire(raw) {
        SkipCause::Contended => "contended".to_string(),
        SkipCause::LowMemory => "low_memory".to_string(),
        SkipCause::StillEncoding => "still_encoding".to_string(),
        SkipCause::ForkFailed => "fork_failed".to_string(),
        SkipCause::ChildTimeout => "child_timeout".to_string(),
        SkipCause::CaptureFailed => "capture_failed".to_string(),
        SkipCause::ChildPanicked => "child_panicked".to_string(),
        SkipCause::ChildCrashed => "child_crashed".to_string(),
        SkipCause::RecorderBehind => "recorder_behind".to_string(),
        SkipCause::Unrecognized(code) => format!("unrecognized_{code}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::state_ring::{encode_record, encode_skip_record, RECORD_KIND_CHUNK};

    const RUN: u64 = 0xABCD_1234;

    fn rec(step: u64, node: u32, part: u32, kind: u32, len: usize) -> Vec<u8> {
        let payload = vec![0x5Au8; len];
        encode_record(
            &StateRecordHeader {
                run_id: RUN,
                step,
                node_idx: node,
                part,
                kind,
                len: len as u32,
            },
            &payload,
        )
        .to_vec()
    }

    /// A two-part anchor completes, and its tally is the hand-computed one.
    #[test]
    fn a_two_part_anchor_completes_and_carries_its_step_and_bytes() {
        let mut l = StateAnchorLedger::passthrough();
        l.feed(&rec(10, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
        l.feed(&rec(10, 0, 1, RECORD_KIND_FINAL, 7));
        l.finish();
        let t = &l.nodes()[&0];
        assert_eq!(t.complete, 1);
        assert_eq!(t.torn, 0);
        assert_eq!(t.last_complete_step, Some(10));
        assert_eq!(t.bytes, STATE_RECORD_PAYLOAD as u64 + 7);
        assert_eq!(l.records(), 2);
    }

    /// A ZERO-length anchor is ONE record and a real, complete anchor — the
    /// `parts_for_len` rule read from the other side.
    #[test]
    fn a_zero_length_anchor_is_one_complete_record() {
        let mut l = StateAnchorLedger::passthrough();
        l.feed(&rec(3, 1, 0, RECORD_KIND_FINAL, 0));
        l.finish();
        assert_eq!(l.nodes()[&1].complete, 1);
        assert_eq!(l.nodes()[&1].bytes, 0);
    }

    /// Two nodes interleaved on ONE ring reassemble independently — the bag
    /// merges every worker's records, so a reader must not depend on ordering.
    #[test]
    fn two_interleaved_node_streams_are_tallied_independently() {
        let mut l = StateAnchorLedger::passthrough();
        l.feed(&rec(5, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
        l.feed(&rec(5, 1, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
        l.feed(&rec(5, 1, 1, RECORD_KIND_FINAL, 1));
        l.feed(&rec(5, 0, 1, RECORD_KIND_FINAL, 2));
        l.finish();
        assert_eq!(l.nodes()[&0].complete, 1);
        assert_eq!(l.nodes()[&1].complete, 1);
        assert_eq!(l.nodes()[&0].torn, 0);
        assert_eq!(l.nodes()[&1].torn, 0);
    }

    /// A missing part TEARS the anchor ONCE, not once per surviving record.
    #[test]
    fn a_lost_part_tears_the_anchor_exactly_once() {
        let mut l = StateAnchorLedger::passthrough();
        l.feed(&rec(8, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
        // part 1 is lost
        l.feed(&rec(8, 0, 2, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
        l.feed(&rec(8, 0, 3, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
        l.feed(&rec(8, 0, 4, RECORD_KIND_FINAL, 1));
        l.finish();
        let t = &l.nodes()[&0];
        assert_eq!(t.torn, 1, "one broken anchor is ONE verdict");
        assert_eq!(t.complete, 0);
        assert_eq!(t.last_complete_step, None);
    }

    /// A SHORT non-final chunk is a shape the chunker cannot produce, so the
    /// stream is corrupt — and it is torn, never silently served short.
    #[test]
    fn a_short_non_final_chunk_tears_the_anchor() {
        let mut l = StateAnchorLedger::passthrough();
        l.feed(&rec(9, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD - 1));
        l.finish();
        assert_eq!(l.nodes()[&0].torn, 1);
    }

    /// An anchor still open when the stream ends is TORN and counted as
    /// truncated — a recorder that stopped mid-anchor has an unusable one.
    #[test]
    fn an_anchor_left_open_at_finish_is_torn_and_counted_truncated() {
        let mut l = StateAnchorLedger::passthrough();
        l.feed(&rec(11, 2, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
        assert_eq!(l.truncated(), 0, "not yet — the stream has not ended");
        l.finish();
        assert_eq!(l.truncated(), 1);
        assert_eq!(l.nodes()[&2].torn, 1);
        // Idempotent: finishing twice must not double-count.
        l.finish();
        assert_eq!(l.truncated(), 1);
        assert_eq!(l.nodes()[&2].torn, 1);
    }

    /// A SKIP is authoritative: it voids whatever was in flight and names why.
    #[test]
    fn a_skip_voids_the_anchor_in_flight_and_names_its_cause() {
        let mut l = StateAnchorLedger::passthrough();
        l.feed(&rec(12, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
        l.feed(&encode_skip_record(RUN, 12, 0, SkipCause::Contended, "n"));
        l.finish();
        let t = &l.nodes()[&0];
        assert_eq!(t.skipped, 1);
        assert_eq!(t.skip_causes.get(&SkipCause::Contended.as_wire()), Some(&1));
        assert_eq!(
            t.torn, 0,
            "a deliberately voided anchor is NOT a broken one — the writer said why"
        );
        assert_eq!(l.truncated(), 0, "the skip closed it, so nothing was open");
    }

    /// An unrecognised skip code survives to the operator with its number.
    #[test]
    fn an_unrecognised_skip_cause_keeps_its_number() {
        assert_eq!(
            skip_cause_name(SkipCause::ChildTimeout.as_wire()),
            "child_timeout"
        );
        assert_eq!(skip_cause_name(9_999), "unrecognized_9999");
    }

    /// The MID-RUN attach: leading records with no head are DISCARDED, counted,
    /// and the first `part == 0` opens the ledger for good.
    #[test]
    fn an_armed_ledger_discards_the_partial_head_then_judges_everything() {
        let mut l = StateAnchorLedger::armed();
        // The tail of an anchor whose head was committed before the attach.
        l.feed(&rec(20, 0, 3, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
        l.feed(&rec(20, 0, 4, RECORD_KIND_FINAL, 5));
        assert!(l.is_armed(), "still discarding — no part 0 has arrived");
        assert_eq!(l.discarded(), 2);
        assert!(l.nodes().is_empty(), "a discarded head tallies NOTHING");
        // The next whole anchor is judged normally.
        l.feed(&rec(21, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
        l.feed(&rec(21, 0, 1, RECORD_KIND_FINAL, 4));
        l.finish();
        assert!(!l.is_armed());
        assert_eq!(
            l.discarded(),
            2,
            "monotone — it opens once and never re-arms"
        );
        assert_eq!(
            l.records(),
            2,
            "the discarded head is not counted as judged"
        );
        assert_eq!(l.nodes()[&0].complete, 1);
        assert_eq!(l.nodes()[&0].last_complete_step, Some(21));
    }

    /// The ANTI-TAUTOLOGY control for the arm: a PASSTHROUGH ledger on the same
    /// stream tears it, so "armed discards" is about the arming and not about
    /// the ledger tolerating headless anchors generally.
    #[test]
    fn a_passthrough_ledger_tears_the_same_headless_anchor_the_armed_one_discards() {
        let mut l = StateAnchorLedger::passthrough();
        l.feed(&rec(20, 0, 3, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
        l.feed(&rec(20, 0, 4, RECORD_KIND_FINAL, 5));
        l.finish();
        assert_eq!(l.discarded(), 0);
        assert_eq!(l.nodes()[&0].torn, 1);
    }

    /// `for_open` pairs the ledger with the consumer constructor.
    #[test]
    fn for_open_pairs_the_ledger_with_the_rings_open_policy() {
        assert!(StateAnchorLedger::for_open(true).is_armed());
        assert!(!StateAnchorLedger::for_open(false).is_armed());
    }

    /// A record this build cannot key is REPORTED, never skipped — a stream that
    /// quietly drops what it does not understand is how a reader serves a short
    /// answer without knowing it.
    #[test]
    fn a_malformed_record_is_counted_rather_than_ignored() {
        let mut l = StateAnchorLedger::passthrough();
        l.feed(&[0u8; 10]); // short of a whole record
        let mut zeroed = vec![0u8; STATE_RECORD_SIZE as usize]; // kind 0
        zeroed[0] = 1;
        l.feed(&zeroed);
        l.finish();
        assert_eq!(l.malformed(), 2);
        assert_eq!(l.records(), 0);
        assert!(l.nodes().is_empty());
    }

    /// THE AGREEMENT ORACLE: this ledger and `StateAssembler` reach the SAME
    /// verdict on the same record stream.
    ///
    /// The module docs claim the ledger "applies the same rules the assembler
    /// applies", and nothing checked it — so the two had DIVERGED in the only
    /// two places a `Voided` tombstone can be closed. The assembler removes it
    /// on a FINAL record ("so a LATER anchor for the same key starts clean");
    /// the ledger left it standing forever, which swallowed any re-sent anchor
    /// under that `(run, step, node)` with no verdict at all and grew the map
    /// unboundedly on a long run with a flaky node. A recorder whose count of a
    /// bag's anchors disagrees with what a RESTORE will make of the same bytes
    /// is worse than a recorder that counted nothing.
    ///
    /// Counting by CLASS rather than by event: the assembler owns payloads and
    /// causes the ledger deliberately never holds (the RAM budget), so what
    /// must agree is the verdict per node — complete / torn / skipped — plus the
    /// malformed count.
    #[derive(Debug, Default, PartialEq, Eq)]
    struct Verdict {
        complete: u64,
        torn: u64,
        skipped: u64,
        /// Precedence: skips naming an anchor the stream already completed.
        skipped_after_complete: u64,
        malformed: u64,
    }

    fn assembler_verdict(records: &[Vec<u8>], armed: bool) -> Verdict {
        use cerulion_core::state_ring::{StateAnchorEvent, StateAssembler};
        let mut a = if armed {
            StateAssembler::armed()
        } else {
            StateAssembler::passthrough()
        };
        let mut v = Verdict::default();
        let mut tally = |e: &StateAnchorEvent| match e {
            StateAnchorEvent::Complete { .. } => v.complete += 1,
            StateAnchorEvent::Torn { .. } => v.torn += 1,
            StateAnchorEvent::Skipped { .. } => v.skipped += 1,
            // Precedence: a note, not a refusal — counted separately, exactly as
            // the ledger counts it, so this cross-check compares like with like.
            StateAnchorEvent::SkipAfterComplete { .. } => v.skipped_after_complete += 1,
            StateAnchorEvent::Malformed { .. } => v.malformed += 1,
        };
        for r in records {
            if let Some(e) = a.feed(r) {
                tally(&e);
            }
        }
        for e in a.finish() {
            tally(&e);
        }
        v
    }

    fn ledger_verdict(records: &[Vec<u8>], armed: bool) -> Verdict {
        let mut l = if armed {
            StateAnchorLedger::armed()
        } else {
            StateAnchorLedger::passthrough()
        };
        for r in records {
            l.feed(r);
        }
        l.finish();
        Verdict {
            complete: l.nodes().values().map(|t| t.complete).sum(),
            torn: l.nodes().values().map(|t| t.torn).sum(),
            skipped: l.nodes().values().map(|t| t.skipped).sum(),
            skipped_after_complete: l.nodes().values().map(|t| t.skipped_after_complete).sum(),
            malformed: l.malformed(),
        }
    }

    #[test]
    fn the_ledger_and_the_assembler_reach_the_same_verdict_on_the_same_stream() {
        let mut malformed = vec![0u8; STATE_RECORD_SIZE as usize];
        malformed[0] = 1; // a non-zero byte with kind 0 — no valid record

        // Each case is a whole stream, named for the rule it exercises. The last
        // three are the ones that diverged.
        let cases: Vec<(&str, Vec<Vec<u8>>, bool)> = vec![
            (
                "two whole anchors",
                [
                    rec(10, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD),
                    rec(10, 0, 1, RECORD_KIND_FINAL, 7),
                    rec(11, 1, 0, RECORD_KIND_FINAL, 0),
                ]
                .into(),
                false,
            ),
            (
                "a lost part tears exactly once",
                [
                    rec(8, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD),
                    rec(8, 0, 2, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD),
                    rec(8, 0, 3, RECORD_KIND_FINAL, 1),
                ]
                .into(),
                false,
            ),
            (
                "a skip voids what was in flight",
                [
                    rec(12, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD),
                    encode_skip_record(RUN, 12, 0, SkipCause::Contended, "n").to_vec(),
                ]
                .into(),
                false,
            ),
            (
                "an anchor the stream stopped inside",
                [rec(11, 2, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD)].into(),
                false,
            ),
            (
                "a malformed record is counted, not swallowed",
                [malformed.clone(), rec(3, 0, 0, RECORD_KIND_FINAL, 2)].into(),
                false,
            ),
            (
                "the mid-run attach discards its partial head",
                [
                    rec(20, 0, 3, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD),
                    rec(20, 0, 4, RECORD_KIND_FINAL, 5),
                    rec(21, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD),
                    rec(21, 0, 1, RECORD_KIND_FINAL, 4),
                ]
                .into(),
                true,
            ),
            // THE DIVERGENCE, shape 1: a torn anchor's FINAL closes its
            // tombstone, so the NEXT anchor under the same key is judged
            // normally. A ledger that swallowed it would report 1 complete
            // where 2 belong.
            (
                "a re-sent anchor after a torn one is judged, not swallowed",
                [
                    rec(30, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD),
                    rec(30, 0, 2, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD),
                    rec(30, 0, 3, RECORD_KIND_FINAL, 1),
                    rec(30, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD),
                    rec(30, 0, 1, RECORD_KIND_FINAL, 6),
                ]
                .into(),
                false,
            ),
            // THE DIVERGENCE, shape 2: the out-of-order record that breaks an
            // OPEN anchor is itself the FINAL, so nothing may be left behind.
            (
                "an out-of-order FINAL leaves no tombstone",
                [
                    rec(40, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD),
                    rec(40, 0, 9, RECORD_KIND_FINAL, 3),
                    rec(40, 0, 0, RECORD_KIND_FINAL, 4),
                ]
                .into(),
                false,
            ),
            (
                "a short non-final chunk still tears, and its FINAL still closes",
                [
                    rec(50, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD - 1),
                    rec(50, 0, 1, RECORD_KIND_FINAL, 2),
                    rec(50, 0, 0, RECORD_KIND_FINAL, 5),
                ]
                .into(),
                false,
            ),
        ];

        for (name, records, armed) in cases {
            let a = assembler_verdict(&records, armed);
            let l = ledger_verdict(&records, armed);
            assert_eq!(
                l, a,
                "`{name}`: the recorder's header-only ledger must reach the SAME \
                 verdict as the assembler a restore runs — ledger {l:?} vs \
                 assembler {a:?}"
            );
        }
    }

    /// The tombstone is CLOSED, not merely overwritten — the map does not grow
    /// for the life of a run with a flaky node.
    ///
    /// A verdict comparison cannot see this: both builds agree on the counts for
    /// a single torn-then-final anchor, and what differs is whether the entry
    /// survives. On a long recording that entry is per `(run, step, node)`, so
    /// nothing ever removed it and the map grew without bound.
    #[test]
    fn a_finished_anchors_tombstone_does_not_outlive_it() {
        let mut l = StateAnchorLedger::passthrough();
        for step in 0..64u64 {
            l.feed(&rec(step, 0, 0, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
            // The part that breaks it, and then the FINAL that closes it.
            l.feed(&rec(step, 0, 7, RECORD_KIND_CHUNK, STATE_RECORD_PAYLOAD));
            l.feed(&rec(step, 0, 8, RECORD_KIND_FINAL, 1));
            assert!(
                l.open_keys_for_test() <= 1,
                "step {step}: a closed anchor must not be carried forever"
            );
        }
        assert_eq!(l.nodes()[&0].torn, 64);
        assert_eq!(l.nodes()[&0].complete, 0);
    }

    fn coverage_of(nodes: &[(&str, u64, u64)], armed: bool) -> StateCoverage {
        StateCoverage {
            version: STATE_COVERAGE_VERSION,
            attached_mid_run: false,
            armed: armed.then(|| StateArmCoverage {
                tag: "run-1".into(),
                cadence_steps: 30_000,
                first_anchor_step: 1,
            }),
            rings_declared: 1,
            ranks_discovered: Vec::new(),
            ranks_missing: Vec::new(),
            rings_unavailable: BTreeMap::new(),
            records: 4,
            head_records_discarded: 0,
            malformed_records: 0,
            foreign_run_records: 0,
            nodes: nodes
                .iter()
                .map(|(id, complete, torn)| {
                    (
                        (*id).to_string(),
                        StateNodeCoverage {
                            ring: "r0".into(),
                            anchors_complete: *complete,
                            anchors_torn: *torn,
                            ..Default::default()
                        },
                    )
                })
                .collect(),
            unattributed_indices: BTreeMap::new(),
        }
    }

    /// The unknown-index keyspace is DISJOINT from real node ids, and its
    /// tallies still escalate.
    ///
    /// A node id is unvalidated (`validate_graph` checks duplicates and nothing
    /// else — MEASURED), so a real node can be named exactly what the old
    /// synthetic key was. Both halves matter: the entries may not collide, AND
    /// splitting them must not drop a tear that used to be counted.
    #[test]
    fn an_unknown_index_cannot_shadow_a_node_named_like_its_label() {
        const COLLIDING_ID: &str = "node_idx 1 (not in the ring manifest)";
        let mut c = coverage_of(&[(COLLIDING_ID, 2, 0)], true);
        c.nodes.get_mut(COLLIDING_ID).unwrap().node_idx = Some(0);
        c.unattributed_indices.insert(
            "r0".into(),
            [(
                1,
                StateNodeCoverage {
                    ring: "r0".into(),
                    node_idx: Some(1),
                    anchors_torn: 1,
                    anchors_skipped: 2,
                    ..Default::default()
                },
            )]
            .into_iter()
            .collect(),
        );

        // The real node is untouched by an index whose label equals its id.
        assert_eq!(c.nodes[COLLIDING_ID].node_idx, Some(0));
        assert_eq!(c.nodes[COLLIDING_ID].anchors_complete, 2);
        assert_eq!(c.nodes.len(), 1);

        // The escalations the split must not lose.
        assert_eq!(c.torn_anchors(), 1, "a tear counts wherever it lives");
        assert_eq!(c.skipped_anchors(), 2);
        assert!(c.is_incomplete());
        // ...and the one it must not INVENT: an index no manifest declares was
        // never due to anchor, so it is not a starved node.
        assert_eq!(c.nodes_without_anchor(), 0);

        // ADDITIVE on the wire, and the two keyspaces stay apart across a
        // round-trip (a `String` cannot key a `BTreeMap<u32, _>`).
        let json = serde_json::to_string(&c).unwrap();
        let back: StateCoverage = serde_json::from_str(&json).unwrap();
        assert_eq!(back, c);
        assert_eq!(back.unattributed_indices["r0"][&1].node_idx, Some(1));

        // The RING scoping, purely: the SAME unknown index reported by a second
        // ring is a second fact, not a merge. `node_idx` is scoped to one ring's
        // manifest, so a bare index key would fold these into one row and let
        // the last writer name the ring.
        c.unattributed_indices.insert(
            "r1".into(),
            [(
                1,
                StateNodeCoverage {
                    ring: "r1".into(),
                    node_idx: Some(1),
                    anchors_torn: 3,
                    ..Default::default()
                },
            )]
            .into_iter()
            .collect(),
        );
        assert_eq!(c.unattributed_indices["r0"][&1].anchors_torn, 1);
        assert_eq!(c.unattributed_indices["r1"][&1].anchors_torn, 3);
        assert_eq!(c.torn_anchors(), 4, "every ring's tears are counted");
        let two: StateCoverage = serde_json::from_str(&serde_json::to_string(&c).unwrap()).unwrap();
        assert_eq!(two, c, "the nested map round-trips");

        // An old bag carries no such key and decodes to EMPTY, never to a
        // fabricated entry.
        let mut empty = coverage_of(&[("a", 1, 0)], false);
        let old_json = serde_json::to_string(&empty).unwrap();
        assert!(!old_json.contains("unattributed_indices"), "{old_json}");
        empty.unattributed_indices.clear();
        assert_eq!(
            serde_json::from_str::<StateCoverage>(&old_json).unwrap(),
            empty
        );
    }

    /// The escalation, one clause at a time.
    #[test]
    fn is_incomplete_escalates_on_every_declared_condition_and_nothing_else() {
        // Clean, armed, every node anchored.
        let clean = coverage_of(&[("a", 2, 0), ("b", 1, 0)], true);
        assert!(!clean.is_incomplete());

        // A node with no anchor — escalates ONLY because this recorder armed.
        let starved = coverage_of(&[("a", 2, 0), ("b", 0, 0)], true);
        assert!(starved.is_incomplete());
        assert_eq!(starved.nodes_without_anchor(), 1);
        let unarmed = coverage_of(&[("a", 2, 0), ("b", 0, 0)], false);
        assert!(
            !unarmed.is_incomplete(),
            "a recorder that asked for no cadence failed at nothing"
        );

        // A torn anchor escalates whether or not we armed.
        let torn = coverage_of(&[("a", 2, 1)], false);
        assert!(torn.is_incomplete());
        assert_eq!(torn.torn_anchors(), 1);

        // A malformed record escalates.
        let mut malformed = coverage_of(&[("a", 2, 0)], false);
        malformed.malformed_records = 1;
        assert!(malformed.is_incomplete());

        // A record the ledger REFUSED as another run's escalates: it was in the ring
        // and is accounted for nowhere, even though the tallies stay valid.
        let mut foreign = coverage_of(&[("a", 2, 0)], false);
        foreign.foreign_run_records = 1;
        assert!(foreign.is_incomplete());

        // An unopenable ring escalates.
        let mut ring_gone = coverage_of(&[("a", 2, 0)], false);
        ring_gone
            .rings_unavailable
            .insert("/cer_st_x".into(), "No such file".into());
        assert!(ring_gone.is_incomplete());

        // A SKIP alone does NOT — the mechanism worked and named a cause.
        let mut skipped = coverage_of(&[("a", 2, 0)], true);
        skipped.nodes.get_mut("a").unwrap().anchors_skipped = 3;
        assert!(!skipped.is_incomplete());
        assert_eq!(skipped.skipped_anchors(), 3);
    }

    /// The attachment is ADDITIVE at its defaults: a clean coverage round-trips
    /// without emitting a key for anything that did not happen.
    #[test]
    fn a_clean_coverage_round_trips_and_omits_its_zero_valued_keys() {
        let c = coverage_of(&[("a", 1, 0)], false);
        let json = serde_json::to_string(&c).unwrap();
        assert!(!json.contains("anchors_torn"), "{json}");
        assert!(!json.contains("head_records_discarded"), "{json}");
        assert!(!json.contains("attached_mid_run"), "{json}");
        assert!(!json.contains("\"armed\""), "{json}");
        assert!(
            !json.contains("node_idx"),
            "an absent index emits no key at all — that is what lets an old \
             reader decode a new bag and a new reader decode an old one: {json}"
        );
        let back: StateCoverage = serde_json::from_str(&json).unwrap();
        assert_eq!(back, c);
    }

    /// The node INDEX survives to the bag, and its absence is a no-claim rather
    /// than a zero. The restore engine depends on it.
    ///
    /// A state record carries `node_idx` and no node id; this map is keyed by
    /// node ID; the ring manifest that relates the two never leaves the live
    /// ring. So without this field a restore engine reading a finished bag
    /// cannot say which node a record belongs to AT ALL — the ledger-to-coverage
    /// conversion was the last place the index existed.
    #[test]
    fn the_node_index_round_trips_and_an_absent_one_decodes_to_no_claim() {
        let mut c = coverage_of(&[("alpha", 1, 0), ("beta", 1, 0)], false);
        c.nodes.get_mut("alpha").unwrap().node_idx = Some(0);
        c.nodes.get_mut("beta").unwrap().node_idx = Some(1);
        let json = serde_json::to_string(&c).unwrap();
        assert!(json.contains("\"node_idx\":0"), "{json}");
        assert!(json.contains("\"node_idx\":1"), "{json}");
        let back: StateCoverage = serde_json::from_str(&json).unwrap();
        assert_eq!(back, c);
        assert_eq!(back.nodes["alpha"].node_idx, Some(0));
        assert_eq!(back.nodes["beta"].node_idx, Some(1));

        // BACK-COMPAT: a manifest written before this field decodes to `None`,
        // never to index 0 — which would be an affirmatively WRONG mapping, and
        // exactly the one a restore engine would act on first.
        let old_bag = json
            .replace(",\"node_idx\":0", "")
            .replace(",\"node_idx\":1", "");
        assert!(!old_bag.contains("node_idx"), "{old_bag}");
        let decoded: StateCoverage = serde_json::from_str(&old_bag).unwrap();
        assert_eq!(decoded.nodes["alpha"].node_idx, None);
        assert_eq!(decoded.nodes["beta"].node_idx, None);
    }

    /// THE REFUTATION, executable: one ledger judges one ring, and one ring carries
    /// one run — so the precedence check reading `last_complete_step` without a
    /// run key cannot be masked by another run's completion.
    ///
    /// Reading the precedence check against the ASSEMBLER's `(run, node)` memory
    /// suggests the coverage side dropped the run key. It never had one to
    /// drop: `nodes` is keyed by `node_idx` alone and always has been — for
    /// `complete`, `torn`, `skipped`, `bytes` and `last_complete_step` alike. What
    /// makes that sound is upstream: every production write path stamps the ONE
    /// `run_id` its producer was built with, and there is exactly one producer per
    /// ring.
    ///
    /// This pins the writer half, which is the half a test can hold: every record a
    /// production producer emits — chunked anchor parts AND skips — carries the run id
    /// derived from the ring's own tag, so a ledger fed from one ring sees one run.
    #[test]
    fn every_record_one_production_producer_writes_carries_that_rings_one_run_id() {
        use cerulion_core::state::StateSink;
        use cerulion_core::state_ring::{
            state_ring_run_id, StateRingConsumer, StateRingOwner, STATE_RECORD_HEADER_SIZE,
            STATE_RECORD_PAYLOAD, STATE_RECORD_SIZE,
        };

        let tag = format!("runinv_{}", std::process::id());
        let expected = state_ring_run_id(&tag);
        let mut owner = StateRingOwner::create(&tag, 64, 0, expected, &["n0"]).expect("ring");
        let name = owner.name().to_string();
        let mut producer = owner.producer().expect("producer");

        // A MULTI-record anchor (so the chunker's own per-record header path runs, not
        // just a single final record), then a skip — the two production write paths.
        {
            let mut sink = producer.sink(7, 0);
            sink.write(&vec![0xAB; STATE_RECORD_PAYLOAD * 2 + 5])
                .expect("room");
            sink.finish();
        }
        producer.push_skip(8, 0, SkipCause::Contended, "a refusal");

        let mut consumer = StateRingConsumer::open(&name).expect("open");
        let (a, b) = consumer.drain_slices().expect("drain the whole ring");
        // The house `as_chunks` shape (clippy 1.98 `chunks_exact_to_as_chunks`);
        // the array refs coerce to `&[u8]` at the collect's element type.
        const RS: usize = STATE_RECORD_SIZE as usize;
        let records: Vec<&[u8]> = a
            .as_chunks::<RS>()
            .0
            .iter()
            .chain(b.as_chunks::<RS>().0)
            .map(|r| r.as_slice())
            .collect();
        assert!(
            records.len() >= 4,
            "3 anchor parts + 1 skip at least, got {}",
            records.len()
        );
        for (i, r) in records.iter().enumerate() {
            let mut hb = [0u8; STATE_RECORD_HEADER_SIZE];
            hb.copy_from_slice(&r[..STATE_RECORD_HEADER_SIZE]);
            let h = StateRecordHeader::from_bytes(&hb);
            assert_eq!(
                h.run_id, expected,
                "record {i} must carry the ring's ONE run id — if this ever fails, the \
                 node table's per-index accounting is judging two runs at once"
            );
        }
        drop(owner);
    }

    /// ...and the invariant is CHECKED, not assumed: a record naming another run is
    /// refused and counted, never folded into a node's numbers.
    ///
    /// The belt-and-braces half of the refutation, and the shape that matters:
    /// run A completes `(step 5, node 0)`, run B skips the same key. Merging it would
    /// blend two runs' anchors into one node's tally — which a reader cannot un-blend
    /// and which looks exactly like a healthy tally — so it is refused and reported,
    /// and the recording stops claiming complete coverage.
    #[test]
    fn a_record_from_another_run_is_refused_and_counted_never_merged() {
        const OTHER: u64 = RUN ^ 0xFFFF;

        fn rec_for(run: u64, step: u64, node: u32, kind: u32, len: usize) -> Vec<u8> {
            encode_record(
                &StateRecordHeader {
                    run_id: run,
                    step,
                    node_idx: node,
                    part: 0,
                    kind,
                    len: len as u32,
                },
                &vec![0x5A; len],
            )
            .to_vec()
        }

        let mut l = StateAnchorLedger::passthrough();
        l.feed(&rec_for(RUN, 5, 0, RECORD_KIND_FINAL, 8));
        l.feed(&encode_skip_record(
            OTHER,
            5,
            0,
            SkipCause::Contended,
            "other run",
        ));
        l.finish();

        let t = &l.nodes()[&0];
        assert_eq!(t.complete, 1, "this run's anchor is intact");
        assert_eq!(
            t.skipped_after_complete, 0,
            "a foreign run's skip must NOT be read as this run's precedence note"
        );
        assert_eq!(
            t.skipped, 0,
            "nor merged into this node's refusals — two runs' numbers cannot share one \
             per-index tally"
        );
        assert_eq!(l.foreign_run_records(), 1, "it is COUNTED, not absorbed");
        assert_eq!(
            l.records(),
            1,
            "and it is not counted as one of this ring's records"
        );

        // The SAME-run skip at the same key is still the precedence note, so the guard
        // has not simply disabled the rule.
        let mut same = StateAnchorLedger::passthrough();
        same.feed(&rec_for(RUN, 5, 0, RECORD_KIND_FINAL, 8));
        same.feed(&encode_skip_record(
            RUN,
            5,
            0,
            SkipCause::Contended,
            "same run",
        ));
        same.finish();
        assert_eq!(same.nodes()[&0].skipped_after_complete, 1);
        assert_eq!(same.foreign_run_records(), 0);
    }

    /// The terminal an operator READS must never contradict the manifest.
    ///
    /// `is_incomplete()` and the terminal line were two independent lists of
    /// conditions, and they had drifted twice over: the `else if` chain had no arm for
    /// `foreign_run_records` (added when the ledger began refusing cross-run records)
    /// and none for `ranks_missing` (there since C5). Either one produced a
    /// `state_coverage.json` reading INCOMPLETE beside a terminal saying "every node
    /// ... has an anchor" — and the terminal is the surface anyone actually sees.
    ///
    /// One list now, rendered by an EXHAUSTIVE match, so a new condition cannot miss
    /// the terminal without failing to compile. This drives every reason through the
    /// real logger and requires the clean line to be absent each time.
    #[test]
    #[tracing_test::traced_test]
    fn every_incomplete_reason_prints_a_warn_and_never_the_clean_line() {
        const CLEAN: &str = "every node the drained rings declare has an anchor";

        // Each row: a coverage carrying exactly ONE condition, and a marker only that
        // condition's line contains. Hand-written — the point is that the logger says
        // something SPECIFIC, not merely that it warns.
        let cases: Vec<(IncompleteReason, StateCoverage, &str)> = vec![
            (
                IncompleteReason::RingsUnavailable,
                {
                    let mut c = coverage_of(&[("a", 1, 0)], false);
                    c.rings_unavailable
                        .insert("/cer_st_x".into(), "boom".into());
                    c
                },
                "could not be opened",
            ),
            (
                IncompleteReason::ForeignRunRecords,
                {
                    let mut c = coverage_of(&[("a", 1, 0)], false);
                    c.foreign_run_records = 3;
                    c
                },
                "they were never tallied",
            ),
            (
                IncompleteReason::TornAnchors,
                coverage_of(&[("a", 1, 1)], false),
                "missing records",
            ),
            (
                IncompleteReason::MalformedRecords,
                {
                    let mut c = coverage_of(&[("a", 1, 0)], false);
                    c.malformed_records = 2;
                    c
                },
                "could not be decoded",
            ),
            (
                IncompleteReason::NodesWithoutAnchor,
                coverage_of(&[("a", 1, 0), ("b", 0, 0)], true),
                "ended the run with NO anchor",
            ),
            (
                IncompleteReason::RanksMissing,
                {
                    let mut c = coverage_of(&[("a", 1, 0)], false);
                    c.ranks_discovered = vec![0, 2];
                    c.ranks_missing = vec![1];
                    c
                },
                "published no state ring",
            ),
        ];

        for (reason, cov, marker) in &cases {
            assert_eq!(
                cov.incomplete_reasons(),
                vec![*reason],
                "the fixture for {reason:?} must carry exactly that one condition"
            );
            assert!(cov.is_incomplete(), "{reason:?} must escalate");
            log_state_coverage_terminal(cov);
            assert!(
                logs_contain(marker),
                "{reason:?} must print its own terminal line (marker: {marker})"
            );
        }
        // THE ARM: across every one of those runs, the clean line never printed. A
        // fall-through arm is exactly how the manifest and the terminal disagree.
        assert!(
            !logs_contain(CLEAN),
            "the clean line must never print for a coverage that reads INCOMPLETE — the \
             terminal is the surface an operator reads, and it was saying the opposite of \
             the artifact"
        );

        // The foreign-run line must say what is actually TRUE. The refusal happens
        // BEFORE any tally, so the accepted per-node numbers stay scoped to the ring's
        // own run — an operator told to distrust them would go auditing valid data
        // instead of the mixed-run recording that is the real problem.
        assert!(
            logs_contain("are UNAFFECTED and stay scoped"),
            "the line must say the surviving tallies are still trustworthy"
        );
        assert!(
            !logs_contain("nobody intended"),
            "and must NOT claim the per-node numbers are an accounting over an \
             unintended set — the ledger refuses foreign records before tallying, so \
             that was never true of the numbers it kept"
        );

        // ANTI-TAUTOLOGY: the clean line DOES print for clean coverage, so the
        // assertion above is a fact about the arms and not about a marker that never
        // appears.
        let clean = coverage_of(&[("a", 2, 0), ("b", 1, 0)], true);
        assert!(clean.incomplete_reasons().is_empty());
        log_state_coverage_terminal(&clean);
        assert!(logs_contain(CLEAN), "clean coverage still reports clean");
    }

    /// Several conditions at once are ALL reported, not just the first.
    ///
    /// The old chain was `else if`, so a bag with an unopenable ring AND a missing
    /// rank told the operator about the ring only — and they would go looking for that
    /// and never learn a rank had published nothing.
    #[test]
    #[tracing_test::traced_test]
    fn a_coverage_failing_several_conditions_reports_every_one() {
        let mut c = coverage_of(&[("a", 1, 1), ("b", 0, 0)], true);
        c.rings_unavailable
            .insert("/cer_st_x".into(), "boom".into());
        c.foreign_run_records = 1;
        c.malformed_records = 1;
        c.ranks_discovered = vec![0, 2];
        c.ranks_missing = vec![1];

        assert_eq!(
            c.incomplete_reasons(),
            vec![
                IncompleteReason::RingsUnavailable,
                IncompleteReason::ForeignRunRecords,
                IncompleteReason::TornAnchors,
                IncompleteReason::MalformedRecords,
                IncompleteReason::NodesWithoutAnchor,
                IncompleteReason::RanksMissing,
            ],
            "every condition that holds is reported, in reading order"
        );
        log_state_coverage_terminal(&c);
        for marker in [
            "could not be opened",
            "they were never tallied",
            "missing records",
            "could not be decoded",
            "ended the run with NO anchor",
            "published no state ring",
        ] {
            assert!(logs_contain(marker), "missing terminal line: {marker}");
        }
        assert!(!logs_contain(
            "every node the drained rings declare has an anchor"
        ));
    }
}
