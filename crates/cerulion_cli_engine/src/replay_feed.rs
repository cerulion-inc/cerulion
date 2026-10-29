// SPDX-License-Identifier: AGPL-3.0-only
//! Identity-keyed PRIVATE REPLAY FEEDS, the engine side.
//!
//! A bag recorded at `trace_format` 8 or later names, for every read on every
//! edge, the writer of the frame it served (the producer annotation before the
//! read, a 64-bit token over the writer's port id) and that frame's wire
//! sequence. Under `bag play --resim all --verify` the engine replays each rank
//! with every read stage subscribed to its own private service
//! (`cerulion_core::replay_feed`) and, at the read gate's admission, publishes
//! onto it exactly the frames that recorded read consumed. Live evictions,
//! fan-out and several writers on one topic then change nothing about what a
//! replayed read sees.
//!
//! This module holds the three halves of that:
//!
//! * [`FrameIndex`]: the bag's recorded frames of the consumed topics, keyed by
//!   `(writer token, wire sequence)`. A multi-writer topic's frames carry the
//!   recorder's per-frame producer labels; a topic with no label at all was
//!   written by one writer.
//! * [`plan_feeds`]: for every gated admission that pops, the list of frames it
//!   consumed, in pop order. A read whose frames cannot be named is REFUSED by
//!   node, input, writer and sequence ([`FeedRefusal`]); there is no fallback
//!   to timestamps.
//! * [`BagReadFeed`]: the `ReadFeed` the runtime's gates call, which publishes a
//!   planned read's frames verbatim from the mapped bag.
//!
//! # Which frames a read consumed
//!
//! A read record names ONE frame, the one it served. A read that popped `n`
//! frames on an edge with ONE writer consumed that writer's `n` frames ending
//! at the served one: the queue drops its oldest frame on overflow, so what one
//! pop takes is a contiguous run of one writer's sequence. On a topic with
//! several writers the frames below the served one are not named by anything
//! the recording holds today, so such a read is refused
//! ([`FeedRefusalReason::BatchUnattributed`]); the per-frame records that close
//! that hole arrive as a list of identities per read, which is the shape
//! [`PlannedRead::frames`] already takes. A `sample(N)` read that dropped the
//! frames it popped names none of them, and is refused the same way
//! ([`FeedRefusalReason::DecimatedUnattributed`]).

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fmt;
use std::sync::{Arc, Mutex};

use cerulion_bag::{FrameSpan, ProducerAttribution};
use cerulion_core::read_outcome::{DueShape, StageKey};
use cerulion_core::replay_feed::{ReadFeed, ReadFeedFault};
use cerulion_core::transport::publisher::CerulionPublisher;

use crate::replay_inject::{ReadKind, StageAdmission};

/// The writer token a recorded port id hashes to: the function the recorder
/// mints producer annotations with, so a frame label and a read annotation
/// naming one port carry one token.
#[must_use]
pub fn writer_token(publisher_id: u128) -> u64 {
    cerulion_core::wire::fnv1a_hash(&publisher_id.to_le_bytes())
}

/// One recorded frame of a topic, in the topic's file order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecordedFrame {
    /// Where the frame's bytes are in the mapped bag.
    pub span: FrameSpan,
    /// Its wire sequence (`None` = no parseable header).
    pub sequence: Option<u32>,
    /// Its wire timestamp (`None` = no parseable header).
    pub timestamp_ns: Option<u64>,
}

/// What the index holds at one `(writer, sequence)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Slot {
    /// Exactly one recorded frame.
    One(FrameSpan),
    /// Several recorded frames share the writer and the sequence, so which one
    /// a read served cannot be told apart.
    Many(usize),
}

/// One topic's recorded frames, keyed by identity.
#[derive(Debug, Default)]
struct TopicIndex {
    /// `true` when the recorder labeled this topic's frames with their writers.
    labeled: bool,
    /// Labeled: keyed by `(writer token, sequence)`. Unlabeled: keyed by
    /// `(0, sequence)`, the writer being the topic's only one.
    by_id: HashMap<(u64, u32), Slot>,
    /// Labeled frames whose writer the labels do not state (a gap or a
    /// contradiction in the label stream).
    unattributed: usize,
    /// The distinct writers the labels name.
    writers: BTreeSet<u64>,
    /// Every headered frame in file order, for the timestamp-feed mutant.
    #[cfg(feature = "oracle-mutants")]
    ordered: Vec<(u64, FrameSpan)>,
}

/// The recorded frames of the topics a rank consumes, keyed by identity.
#[derive(Debug, Default)]
pub struct FrameIndex {
    topics: HashMap<String, TopicIndex>,
}

/// One topic's producer labels, in file order: what each label attributes, and
/// to which port.
pub type TopicLabels = Vec<(ProducerAttribution, u128)>;

impl FrameIndex {
    /// Index `frames` (per topic, in file order) under `labels` (per topic).
    ///
    /// A topic with at least one label is LABELED: a `FrameLabel` names one
    /// frame's writer by its ordinal, a `CatchUpPrefix` the writer of a leading
    /// run, and a frame two labels give two writers is unattributed. A topic
    /// with no label is indexed by sequence alone.
    #[must_use]
    pub fn build(
        frames: &BTreeMap<String, Vec<RecordedFrame>>,
        labels: &BTreeMap<String, TopicLabels>,
    ) -> Self {
        let mut topics = HashMap::new();
        for (topic, list) in frames {
            let topic_labels = labels.get(topic).map(Vec::as_slice).unwrap_or(&[]);
            let mut idx = TopicIndex {
                labeled: !topic_labels.is_empty(),
                ..TopicIndex::default()
            };
            // Writer per ordinal: `None` = unstated, `Some(None)` = contradicted.
            let mut writer_of: Vec<Option<Option<u128>>> = vec![None; list.len()];
            // `contested`: a record that may contradict an attribution but
            // never originates one (a second catch-up prefix naming another
            // writer, which the recorder never writes).
            let mut attribute = |ordinal: usize, id: u128, contested: bool| {
                if let Some(slot) = writer_of.get_mut(ordinal) {
                    *slot = match *slot {
                        None if contested => None,
                        None => Some(Some(id)),
                        Some(Some(prev)) if prev == id => Some(Some(id)),
                        Some(_) => Some(None),
                    };
                }
            };
            let mut first_prefix: Option<u128> = None;
            for (attribution, id) in topic_labels {
                match *attribution {
                    ProducerAttribution::FrameLabel { frame_index } => {
                        if let Ok(ordinal) = usize::try_from(frame_index) {
                            attribute(ordinal, *id, false);
                        }
                    }
                    ProducerAttribution::CatchUpPrefix { prefix_len } => {
                        let contested = first_prefix.is_some_and(|p| p != *id);
                        first_prefix.get_or_insert(*id);
                        let end = usize::try_from(prefix_len)
                            .unwrap_or(usize::MAX)
                            .min(list.len());
                        for ordinal in 0..end {
                            attribute(ordinal, *id, contested);
                        }
                    }
                }
            }
            for (ordinal, frame) in list.iter().enumerate() {
                let Some(seq) = frame.sequence else {
                    continue;
                };
                let writer = if idx.labeled {
                    match writer_of[ordinal] {
                        Some(Some(id)) => {
                            let token = writer_token(id);
                            idx.writers.insert(token);
                            token
                        }
                        _ => {
                            idx.unattributed += 1;
                            continue;
                        }
                    }
                } else {
                    0
                };
                idx.by_id
                    .entry((writer, seq))
                    .and_modify(|slot| {
                        *slot = match *slot {
                            Slot::One(_) => Slot::Many(2),
                            Slot::Many(n) => Slot::Many(n + 1),
                        }
                    })
                    .or_insert(Slot::One(frame.span));
                #[cfg(feature = "oracle-mutants")]
                if let Some(ts) = frame.timestamp_ns {
                    idx.ordered.push((ts, frame.span));
                }
            }
            topics.insert(topic.clone(), idx);
        }
        Self { topics }
    }
}

/// Why a recorded read's frames cannot be fed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FeedRefusalReason {
    /// The read popped frames and no producer annotation named their writer.
    IdentityAbsent,
    /// The bag holds no frame of this writer at this sequence on the topic.
    FrameMissing,
    /// Several recorded frames carry this writer and this sequence, so which
    /// one the read served cannot be told apart. A gateway that republishes
    /// several remote writers' frames under its own one port is the shape that
    /// produces it.
    WritersIndistinguishable {
        /// How many frames share the identity.
        frames: usize,
    },
    /// A read popped `popped` frames on a topic with several writers, and the
    /// recording names only the newest.
    BatchUnattributed {
        /// Frames the read popped.
        popped: u32,
    },
    /// A `sample(N)` read dropped the frames it popped and names none of them.
    DecimatedUnattributed {
        /// Frames the read popped.
        popped: u32,
    },
    /// The topic's frames carry no writer labels and its reads name more than
    /// one writer, so no frame can be attributed.
    FramesUnattributed {
        /// The distinct writers the reads name.
        writers: usize,
    },
    /// The bag holds no frame with a readable wire header on the topic this
    /// input reads.
    TopicUnrecorded,
    /// An earlier read on the same stage already consumed the frame of this
    /// writer at this sequence. A queue hands a frame to one pop, so the read
    /// log attributes one of the two reads to the wrong writer: an annotation
    /// that opened a writer run is missing from it.
    FrameClaimedTwice {
        /// The recorded step of the read that consumed the frame first.
        first_step: u64,
    },
}

impl FeedRefusalReason {
    /// The stable token a report keys on.
    #[must_use]
    pub fn code(&self) -> &'static str {
        match self {
            Self::IdentityAbsent => "read_identity_absent",
            Self::FrameMissing => "read_identity_frame_missing",
            Self::WritersIndistinguishable { .. } => "read_identity_writers_indistinguishable",
            Self::BatchUnattributed { .. } => "read_identity_batch_unattributed",
            Self::DecimatedUnattributed { .. } => "read_identity_decimated_unattributed",
            Self::FramesUnattributed { .. } => "read_identity_frames_unattributed",
            Self::TopicUnrecorded => "read_identity_topic_unrecorded",
            Self::FrameClaimedTwice { .. } => "read_identity_frame_claimed_twice",
        }
    }
}

/// A recorded read whose frames cannot be fed, named.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FeedRefusal {
    /// The consuming node.
    pub node: String,
    /// The input, by name.
    pub input: String,
    /// The topic the input reads.
    pub topic: String,
    /// The recorded step of the read.
    pub step: u64,
    /// The writer token the read named, when it named one.
    pub writer: Option<u64>,
    /// The sequence that could not be fed (the served one, or the frame of a
    /// batch the lookup failed on).
    pub sequence: Option<u32>,
    /// Why.
    pub reason: FeedRefusalReason,
}

impl fmt::Display for FeedRefusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let writer = self
            .writer
            .map_or_else(|| "none".to_string(), |w| format!("{w:016x}"));
        let sequence = self
            .sequence
            .map_or_else(|| "none".to_string(), |s| s.to_string());
        write!(
            f,
            "node '{}', input '{}' (topic '{}'), step {}, writer {writer}, sequence {sequence}: ",
            self.node, self.input, self.topic, self.step
        )?;
        match &self.reason {
            FeedRefusalReason::IdentityAbsent => write!(
                f,
                "the read popped frames and the recording names no writer for them"
            ),
            FeedRefusalReason::FrameMissing => write!(
                f,
                "the bag holds no frame of that writer at that sequence (the recorder lost it)"
            ),
            FeedRefusalReason::WritersIndistinguishable { frames } => write!(
                f,
                "{frames} recorded frames carry that writer and that sequence, so the frame the \
                 read served cannot be told apart (a gateway republishes several remote writers \
                 under one port)"
            ),
            FeedRefusalReason::BatchUnattributed { popped } => write!(
                f,
                "the read popped {popped} frames on a topic with several writers and the \
                 recording names only the newest"
            ),
            FeedRefusalReason::DecimatedUnattributed { popped } => write!(
                f,
                "a sample(N) read dropped the {popped} frame(s) it popped and the recording \
                 names none of them"
            ),
            FeedRefusalReason::FramesUnattributed { writers } => write!(
                f,
                "the topic's frames carry no writer labels while its reads name {writers} \
                 writers, so no frame can be attributed"
            ),
            FeedRefusalReason::TopicUnrecorded => write!(
                f,
                "the bag holds no frame with a readable wire header on the topic this input reads"
            ),
            FeedRefusalReason::FrameClaimedTwice { first_step } => write!(
                f,
                "a read at step {first_step} on this input already consumed that frame, so the \
                 read log names the wrong writer for one of the two (an annotation opening a \
                 writer run is missing)"
            ),
        }
    }
}

/// One fed read: the frames it consumed, in pop order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlannedRead {
    /// The frames, oldest first.
    pub frames: Vec<FrameSpan>,
    /// The `(writer, sequence)` of each frame, parallel to [`Self::frames`].
    pub ids: Vec<(u64, u32)>,
}

/// Every fed read of one pass, keyed by stage and by `(step, ordinal)`, the
/// position the read gate asks for.
#[derive(Debug, Default)]
pub struct FeedPlan {
    reads: HashMap<StageKey, HashMap<(u64, u32), PlannedRead>>,
    /// Per stage, the most frames one read consumes: the private service's
    /// buffer has to hold them all at once.
    buffers: HashMap<StageKey, usize>,
    /// Per stage, the topic whose recorded frames feed it.
    topics: HashMap<StageKey, String>,
    /// Fed reads over the pass.
    reads_planned: usize,
    /// Fed frames over the pass.
    frames_planned: usize,
}

impl FeedPlan {
    /// The per-stage buffer floors, for the fed runtime build.
    #[must_use]
    pub fn buffers(&self) -> &HashMap<StageKey, usize> {
        &self.buffers
    }

    /// The planned read at `key`, `step`, `ordinal`.
    #[must_use]
    pub fn read(&self, key: &StageKey, step: u64, ordinal: u32) -> Option<&PlannedRead> {
        self.reads.get(key)?.get(&(step, ordinal))
    }

    /// Fed reads over the pass.
    #[must_use]
    pub fn reads_planned(&self) -> usize {
        self.reads_planned
    }

    /// Fed frames over the pass.
    #[must_use]
    pub fn frames_planned(&self) -> usize {
        self.frames_planned
    }

    /// Every stage the plan names a buffer for.
    pub fn stages(&self) -> impl Iterator<Item = &StageKey> {
        self.buffers.keys()
    }

    /// The topic whose recorded frames feed `key`.
    #[must_use]
    pub fn topic(&self, key: &StageKey) -> Option<&str> {
        self.topics.get(key).map(String::as_str)
    }
}

/// One stage the feed plan covers: its admissions, its input's name and the
/// topic the input reads live.
#[derive(Debug, Clone, Copy)]
pub struct FedStage<'a> {
    /// The stage's admissions, as the read gate installs them.
    pub admission: &'a StageAdmission,
    /// The input's name (for the refusal sentence).
    pub input: &'a str,
    /// The topic whose recorded frames feed it.
    pub topic: &'a str,
    /// The input is a multi-writer edge, whose recorder stages an annotation
    /// before every read: a popping read without one of its own lost it.
    pub every_read: bool,
}

/// The feed MUTANTS the oracle tests must catch, compiled only with the
/// `oracle-mutants` feature.
#[cfg(feature = "oracle-mutants")]
pub mod mutants {
    use std::sync::atomic::{AtomicU8, Ordering};

    /// No mutant.
    const NONE: u8 = 0;
    /// Feed each read the newest frames of its topic stamped before the step's
    /// recorded boundary instead of the frames it named.
    const BY_TIMESTAMP: u8 = 1;

    static ARMED: AtomicU8 = AtomicU8::new(NONE);

    /// A scope that arms the timestamp feed until it drops.
    #[derive(Debug)]
    pub struct MutantGuard(());

    impl Drop for MutantGuard {
        fn drop(&mut self) {
            ARMED.store(NONE, Ordering::SeqCst);
        }
    }

    /// Arm the TIMESTAMP feed: every fed read gets the `popped` newest frames
    /// of its topic (all writers, file order) stamped strictly before its
    /// step's recorded boundary, the recorded-clock window replay used before
    /// read identity.
    #[must_use]
    pub fn feed_by_timestamp() -> MutantGuard {
        ARMED.store(BY_TIMESTAMP, Ordering::SeqCst);
        MutantGuard(())
    }

    pub(super) fn by_timestamp() -> bool {
        ARMED.load(Ordering::SeqCst) == BY_TIMESTAMP
    }
}

/// Plan every fed read of a pass.
///
/// For each admission that pops, the frames its recorded identity names, looked
/// up in `index`. `step_targets` maps a step to its recorded boundary target
/// and is read only by the timestamp mutant.
///
/// # Errors
///
/// The first read whose frames cannot be named, as a [`FeedRefusal`].
pub fn plan_feeds(
    stages: &[FedStage<'_>],
    index: &FrameIndex,
    step_targets: &BTreeMap<u64, u64>,
) -> Result<FeedPlan, FeedRefusal> {
    let _ = step_targets;
    // Pass 1: an unlabeled topic must be read under ONE writer.
    let mut writers_by_topic: BTreeMap<&str, BTreeSet<u64>> = BTreeMap::new();
    for stage in stages {
        for step in &stage.admission.per_step {
            for id in &step.identity {
                if let Some(w) = id.writer {
                    writers_by_topic.entry(stage.topic).or_default().insert(w);
                }
            }
        }
    }
    let mut plan = FeedPlan::default();
    for stage in stages {
        let key = &stage.admission.key;
        let refuse = |step: u64, writer: Option<u64>, sequence: Option<u32>, reason| FeedRefusal {
            node: key.node.clone(),
            input: stage.input.to_string(),
            topic: stage.topic.to_string(),
            step,
            writer,
            sequence,
            reason,
        };
        plan.buffers.entry(key.clone()).or_insert(0);
        plan.topics.insert(key.clone(), stage.topic.to_string());
        // Per lookup identity, the step of the read on this stage that
        // consumed it.
        let mut claimed: HashMap<(u64, u32), u64> = HashMap::new();
        for step in &stage.admission.per_step {
            for (ordinal, (due, id)) in step.due.iter().zip(&step.identity).enumerate() {
                let DueShape::Pops { pops, .. } = due.shape() else {
                    continue;
                };
                let ordinal = u32::try_from(ordinal).unwrap_or(u32::MAX);
                let Some(topic) = index
                    .topics
                    .get(stage.topic)
                    .filter(|t| !t.by_id.is_empty() || t.unattributed > 0)
                else {
                    return Err(refuse(
                        step.step,
                        id.writer,
                        id.served_seq,
                        FeedRefusalReason::TopicUnrecorded,
                    ));
                };
                if id.kind == Some(ReadKind::Decimated) {
                    return Err(refuse(
                        step.step,
                        id.writer,
                        id.served_seq,
                        FeedRefusalReason::DecimatedUnattributed { popped: pops },
                    ));
                }
                if stage.every_read && !id.annotated {
                    return Err(refuse(
                        step.step,
                        None,
                        id.served_seq,
                        FeedRefusalReason::IdentityAbsent,
                    ));
                }
                let Some(writer) = id.writer else {
                    return Err(refuse(
                        step.step,
                        None,
                        id.served_seq,
                        FeedRefusalReason::IdentityAbsent,
                    ));
                };
                let Some(served) = id.served_seq else {
                    return Err(refuse(
                        step.step,
                        Some(writer),
                        None,
                        FeedRefusalReason::IdentityAbsent,
                    ));
                };
                if !topic.labeled {
                    let named = writers_by_topic.get(stage.topic).map_or(0, BTreeSet::len);
                    if named > 1 {
                        return Err(refuse(
                            step.step,
                            Some(writer),
                            Some(served),
                            FeedRefusalReason::FramesUnattributed { writers: named },
                        ));
                    }
                } else if pops > 1 && topic.writers.len() > 1 {
                    return Err(refuse(
                        step.step,
                        Some(writer),
                        Some(served),
                        FeedRefusalReason::BatchUnattributed { popped: pops },
                    ));
                }
                let mut read = PlannedRead {
                    frames: Vec::with_capacity(pops as usize),
                    ids: Vec::with_capacity(pops as usize),
                };
                #[cfg(feature = "oracle-mutants")]
                if mutants::by_timestamp() {
                    let target = step_targets.get(&step.step).copied().unwrap_or(u64::MAX);
                    let before: Vec<FrameSpan> = topic
                        .ordered
                        .iter()
                        .filter(|(ts, _)| *ts < target)
                        .map(|(_, span)| *span)
                        .collect();
                    let take = (pops as usize).min(before.len());
                    for span in &before[before.len() - take..] {
                        read.frames.push(*span);
                        read.ids.push((writer, served));
                    }
                    plan.insert(key, step.step, ordinal, read);
                    continue;
                }
                // Oldest first: the writer's `pops` frames ending at the served
                // one.
                for back in (0..pops).rev() {
                    let seq = served.wrapping_sub(back);
                    let lookup_writer = if topic.labeled { writer } else { 0 };
                    if let Some(first_step) = claimed.insert((lookup_writer, seq), step.step) {
                        return Err(refuse(
                            step.step,
                            Some(writer),
                            Some(seq),
                            FeedRefusalReason::FrameClaimedTwice { first_step },
                        ));
                    }
                    match topic.by_id.get(&(lookup_writer, seq)) {
                        Some(Slot::One(span)) => {
                            read.frames.push(*span);
                            read.ids.push((writer, seq));
                        }
                        Some(Slot::Many(frames)) => {
                            return Err(refuse(
                                step.step,
                                Some(writer),
                                Some(seq),
                                FeedRefusalReason::WritersIndistinguishable { frames: *frames },
                            ));
                        }
                        None => {
                            return Err(refuse(
                                step.step,
                                Some(writer),
                                Some(seq),
                                FeedRefusalReason::FrameMissing,
                            ));
                        }
                    }
                }
                plan.insert(key, step.step, ordinal, read);
            }
        }
    }
    Ok(plan)
}

impl FeedPlan {
    fn insert(&mut self, key: &StageKey, step: u64, ordinal: u32, read: PlannedRead) {
        let n = read.frames.len();
        self.reads_planned += 1;
        self.frames_planned += n;
        let floor = self.buffers.entry(key.clone()).or_insert(0);
        *floor = (*floor).max(n);
        self.reads
            .entry(key.clone())
            .or_default()
            .insert((step, ordinal), read);
    }
}

/// One fed read as the feed actually published it: the oracle's view of what
/// the replay served, beside the node's own.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FedRead {
    /// The stage.
    pub key: StageKey,
    /// The step.
    pub step: u64,
    /// The admission's position in the step's plan.
    pub ordinal: u32,
    /// The `(writer, sequence)` of each frame published, oldest first.
    pub ids: Vec<(u64, u32)>,
}

/// The `ReadFeed` a fed replay installs: publishes a planned read's frames,
/// verbatim, from the mapped bag onto the stage's private service.
pub struct BagReadFeed {
    reader: Arc<cerulion_bag::BagReader>,
    plan: Arc<FeedPlan>,
    ports: HashMap<StageKey, Mutex<CerulionPublisher>>,
    log: Mutex<FeedLog>,
}

/// What the feed did over the pass.
#[derive(Debug, Default)]
struct FeedLog {
    /// Reads published.
    reads: usize,
    /// Frames published.
    frames: usize,
    /// Faults, in order (bounded).
    faults: Vec<String>,
    /// Faults the bound dropped.
    faults_dropped: usize,
}

/// The faults a feed keeps for its report.
const FEED_FAULTS_RETAINED: usize = 32;

impl fmt::Debug for BagReadFeed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BagReadFeed")
            .field("stages", &self.ports.len())
            .field("reads_planned", &self.plan.reads_planned)
            .finish_non_exhaustive()
    }
}

impl BagReadFeed {
    /// A feed over `plan`, publishing through `ports` (one per fed stage).
    #[must_use]
    pub fn new(
        reader: Arc<cerulion_bag::BagReader>,
        plan: Arc<FeedPlan>,
        ports: HashMap<StageKey, CerulionPublisher>,
    ) -> Self {
        Self {
            reader,
            plan,
            ports: ports.into_iter().map(|(k, p)| (k, Mutex::new(p))).collect(),
            log: Mutex::new(FeedLog::default()),
        }
    }

    /// `(reads, frames)` published so far.
    #[must_use]
    pub fn published(&self) -> (usize, usize) {
        let log = self
            .log
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        (log.reads, log.frames)
    }

    /// The faults the feed met, oldest first, and how many past the bound.
    #[must_use]
    pub fn faults(&self) -> (Vec<String>, usize) {
        let log = self
            .log
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        (log.faults.clone(), log.faults_dropped)
    }

    fn fault(&self, detail: String) -> ReadFeedFault {
        let mut log = self
            .log
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if log.faults.len() < FEED_FAULTS_RETAINED {
            log.faults.push(detail.clone());
        } else {
            log.faults_dropped += 1;
        }
        ReadFeedFault { detail }
    }
}

impl ReadFeed for BagReadFeed {
    fn fill(&self, key: &StageKey, step: u64, ordinal: u32) -> Result<(), ReadFeedFault> {
        let Some(read) = self.plan.read(key, step, ordinal) else {
            return Err(self.fault(format!(
                "stage {} step {step}: the gate admitted read {ordinal}, which the feed plan \
                 does not hold",
                key.label()
            )));
        };
        let Some(port) = self.ports.get(key) else {
            return Err(self.fault(format!(
                "stage {}: no private feed port is open",
                key.label()
            )));
        };
        let mut port = port
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        for span in &read.frames {
            if let Err(e) = port.publish_raw(self.reader.frame(span)) {
                return Err(self.fault(format!(
                    "stage {} step {step} read {ordinal}: publishing a recorded frame failed: {e}",
                    key.label()
                )));
            }
        }
        drop(port);
        let mut log = self
            .log
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        log.reads += 1;
        log.frames += read.frames.len();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::replay_inject::{ReadIdentity, StepAdmission};
    use cerulion_core::read_outcome::{DueRead, ReadStageRole};

    fn span(offset: usize) -> FrameSpan {
        FrameSpan { offset, len: 1 }
    }

    fn frame(offset: usize, seq: u32) -> RecordedFrame {
        RecordedFrame {
            span: span(offset),
            sequence: Some(seq),
            timestamp_ns: Some(offset as u64),
        }
    }

    fn key() -> StageKey {
        StageKey {
            node: "sink".into(),
            input_idx: 0,
            role: ReadStageRole::Body,
        }
    }

    fn admission(reads: &[(u64, u32, Option<u64>, u32, ReadKind)]) -> StageAdmission {
        let mut per_step: BTreeMap<u64, StepAdmission> = BTreeMap::new();
        for &(step, seq, writer, popped, kind) in reads {
            let entry = per_step.entry(step).or_insert_with(|| StepAdmission {
                step,
                due: Vec::new(),
                identity: Vec::new(),
            });
            entry.due.push(if kind == ReadKind::Decimated {
                DueRead::decimated(popped, Some(seq))
            } else {
                DueRead::pops(popped, Some(seq))
            });
            entry.identity.push(ReadIdentity {
                writer,
                annotated: writer.is_some(),
                served_seq: Some(seq),
                popped,
                kind: Some(kind),
            });
        }
        StageAdmission {
            key: key(),
            mode: crate::replay_inject::ConsumeMode::Latest,
            per_step: per_step.into_values().collect(),
        }
    }

    fn index(frames: Vec<RecordedFrame>, labels: TopicLabels) -> FrameIndex {
        let mut f = BTreeMap::new();
        f.insert("/t".to_string(), frames);
        let mut l = BTreeMap::new();
        if !labels.is_empty() {
            l.insert("/t".to_string(), labels);
        }
        FrameIndex::build(&f, &l)
    }

    fn plan(adm: &StageAdmission, idx: &FrameIndex) -> Result<FeedPlan, FeedRefusal> {
        plan_with(adm, idx, false)
    }

    fn plan_with(
        adm: &StageAdmission,
        idx: &FrameIndex,
        every_read: bool,
    ) -> Result<FeedPlan, FeedRefusal> {
        plan_feeds(
            &[FedStage {
                admission: adm,
                input: "inp",
                topic: "/t",
                every_read,
            }],
            idx,
            &BTreeMap::new(),
        )
    }

    /// A single-writer Latest read that popped 2 after the live queue evicted
    /// seq 1 is fed exactly 2 and 3, never 1.
    #[test]
    fn a_batch_on_one_writer_is_fed_the_run_ending_at_the_served_frame() {
        let idx = index(vec![frame(10, 1), frame(20, 2), frame(30, 3)], vec![]);
        let adm = admission(&[(4, 3, Some(7), 2, ReadKind::Served)]);
        let p = plan(&adm, &idx).expect("plans");
        let read = p.read(&key(), 4, 0).expect("read planned");
        assert_eq!(read.frames, vec![span(20), span(30)]);
        assert_eq!(read.ids, vec![(7, 2), (7, 3)]);
        assert_eq!((p.reads_planned(), p.frames_planned()), (1, 2));
        assert_eq!(p.buffers().get(&key()), Some(&2));
    }

    /// Three writers, each at sequence 5: the label decides which frame.
    #[test]
    fn a_labeled_topic_is_fed_by_writer_and_sequence() {
        let ids = [11u128, 22, 33];
        let frames = vec![frame(1, 5), frame(2, 5), frame(3, 5)];
        let labels = ids
            .iter()
            .enumerate()
            .map(|(i, id)| {
                (
                    ProducerAttribution::FrameLabel {
                        frame_index: i as u64,
                    },
                    *id,
                )
            })
            .collect();
        let idx = index(frames, labels);
        let adm = admission(&[(1, 5, Some(writer_token(22)), 1, ReadKind::Served)]);
        let p = plan(&adm, &idx).expect("plans");
        assert_eq!(p.read(&key(), 1, 0).unwrap().frames, vec![span(2)]);
    }

    #[test]
    fn a_read_with_no_writer_is_refused_by_name() {
        let idx = index(vec![frame(1, 5)], vec![]);
        let adm = admission(&[(3, 5, None, 1, ReadKind::Served)]);
        let r = plan(&adm, &idx).unwrap_err();
        assert_eq!(r.reason, FeedRefusalReason::IdentityAbsent);
        let text = r.to_string();
        for part in ["'sink'", "'inp'", "step 3", "writer none", "sequence 5"] {
            assert!(text.contains(part), "{text}");
        }
    }

    #[test]
    fn a_missing_frame_is_refused_naming_its_sequence() {
        let idx = index(vec![frame(1, 5), frame(2, 7)], vec![]);
        let adm = admission(&[(3, 7, Some(9), 2, ReadKind::Served)]);
        let r = plan(&adm, &idx).unwrap_err();
        assert_eq!(r.reason, FeedRefusalReason::FrameMissing);
        assert_eq!(r.sequence, Some(6));
    }

    #[test]
    fn two_frames_with_one_writer_and_one_sequence_are_indistinguishable() {
        let labels = vec![(ProducerAttribution::CatchUpPrefix { prefix_len: 2 }, 4u128)];
        let idx = index(vec![frame(1, 5), frame(2, 5)], labels);
        let adm = admission(&[(1, 5, Some(writer_token(4)), 1, ReadKind::Served)]);
        assert_eq!(
            plan(&adm, &idx).unwrap_err().reason,
            FeedRefusalReason::WritersIndistinguishable { frames: 2 }
        );
    }

    #[test]
    fn a_batch_on_several_writers_and_a_decimated_read_are_refused() {
        let labels = vec![
            (ProducerAttribution::FrameLabel { frame_index: 0 }, 1u128),
            (ProducerAttribution::FrameLabel { frame_index: 1 }, 2u128),
        ];
        let idx = index(vec![frame(1, 5), frame(2, 6)], labels);
        let adm = admission(&[(1, 6, Some(writer_token(2)), 2, ReadKind::DrainedBatch)]);
        assert_eq!(
            plan(&adm, &idx).unwrap_err().reason,
            FeedRefusalReason::BatchUnattributed { popped: 2 }
        );
        let idx = index(vec![frame(1, 5)], vec![]);
        let adm = admission(&[(1, 5, Some(1), 1, ReadKind::Decimated)]);
        assert_eq!(
            plan(&adm, &idx).unwrap_err().reason,
            FeedRefusalReason::DecimatedUnattributed { popped: 1 }
        );
    }

    /// Two writers at sequence 5, and a second read attributed to the first
    /// writer (the annotation that switched the run to the other writer is
    /// missing): a queue hands a frame to one pop, so the second claim is
    /// refused by name rather than fed the first writer's frame again.
    #[test]
    fn a_frame_claimed_by_two_reads_of_one_stage_is_refused() {
        let labels = vec![
            (ProducerAttribution::FrameLabel { frame_index: 0 }, 1u128),
            (ProducerAttribution::FrameLabel { frame_index: 1 }, 2u128),
        ];
        let idx = index(vec![frame(1, 5), frame(2, 5)], labels);
        let a = writer_token(1);
        let adm = admission(&[
            (1, 5, Some(a), 1, ReadKind::Served),
            (2, 5, Some(a), 1, ReadKind::Served),
        ]);
        let r = plan(&adm, &idx).unwrap_err();
        assert_eq!(
            r.reason,
            FeedRefusalReason::FrameClaimedTwice { first_step: 1 }
        );
        assert_eq!((r.step, r.sequence), (2, Some(5)));
        assert_eq!(r.reason.code(), "read_identity_frame_claimed_twice");
        // The same two reads attributed to their own writers are fed.
        let adm = admission(&[
            (1, 5, Some(a), 1, ReadKind::Served),
            (2, 5, Some(writer_token(2)), 1, ReadKind::Served),
        ]);
        let p = plan(&adm, &idx).expect("plans");
        assert_eq!(p.read(&key(), 2, 0).unwrap().frames, vec![span(2)]);
    }

    /// On a multi-writer edge every read carries its own annotation, so a
    /// popping read that only CONTINUES a run lost its annotation: it is refused
    /// as having no recorded writer rather than fed the previous read's writer.
    #[test]
    fn a_multi_writer_read_without_its_own_annotation_is_refused() {
        let labels = vec![
            (ProducerAttribution::FrameLabel { frame_index: 0 }, 1u128),
            (ProducerAttribution::FrameLabel { frame_index: 1 }, 2u128),
        ];
        let idx = index(vec![frame(1, 5), frame(2, 6)], labels);
        let mut adm = admission(&[
            (1, 5, Some(writer_token(1)), 1, ReadKind::Served),
            (2, 6, Some(writer_token(1)), 1, ReadKind::Served),
        ]);
        adm.per_step[1].identity[0].annotated = false;
        let r = plan_with(&adm, &idx, true).unwrap_err();
        assert_eq!(r.reason, FeedRefusalReason::IdentityAbsent);
        assert_eq!((r.step, r.writer, r.sequence), (2, None, Some(6)));
        // A single-writer edge's run carries the writer forward.
        let idx = index(vec![frame(1, 5), frame(2, 6)], vec![]);
        plan_with(&adm, &idx, false).expect("a run continues the writer");
    }

    #[test]
    fn an_unlabeled_topic_read_under_two_writers_is_refused() {
        let idx = index(vec![frame(1, 5), frame(2, 6)], vec![]);
        let adm = admission(&[
            (1, 5, Some(1), 1, ReadKind::Served),
            (2, 6, Some(2), 1, ReadKind::Served),
        ]);
        assert_eq!(
            plan(&adm, &idx).unwrap_err().reason,
            FeedRefusalReason::FramesUnattributed { writers: 2 }
        );
    }

    /// A second catch-up prefix naming another writer contradicts what the first
    /// attributed and attributes nothing on its own.
    #[test]
    fn a_second_differing_prefix_attributes_nothing() {
        let labels = vec![
            (ProducerAttribution::CatchUpPrefix { prefix_len: 1 }, 4u128),
            (ProducerAttribution::CatchUpPrefix { prefix_len: 2 }, 5u128),
        ];
        let idx = index(vec![frame(1, 5), frame(2, 6)], labels);
        let named_by_first = admission(&[(1, 5, Some(writer_token(4)), 1, ReadKind::Served)]);
        assert_eq!(
            plan(&named_by_first, &idx).unwrap_err().reason,
            FeedRefusalReason::FrameMissing,
            "frame 0 is contradicted"
        );
        let named_by_second = admission(&[(1, 6, Some(writer_token(5)), 1, ReadKind::Served)]);
        assert_eq!(
            plan(&named_by_second, &idx).unwrap_err().reason,
            FeedRefusalReason::FrameMissing,
            "frame 1 is not attributed by the contested prefix"
        );
    }

    #[test]
    fn a_topic_the_bag_holds_no_frame_of_is_named_unrecorded() {
        let idx = index(vec![], vec![]);
        let adm = admission(&[(1, 5, Some(1), 1, ReadKind::Served)]);
        assert_eq!(
            plan(&adm, &idx).unwrap_err().reason,
            FeedRefusalReason::TopicUnrecorded
        );
    }

    /// A pop-less admission is never planned, so the gate never asks for it.
    #[test]
    fn only_popping_admissions_are_planned() {
        let idx = index(vec![frame(1, 5)], vec![]);
        let mut adm = admission(&[(1, 5, Some(1), 1, ReadKind::Served)]);
        adm.per_step[0].due.insert(0, DueRead::nothing());
        adm.per_step[0].identity.insert(
            0,
            ReadIdentity {
                writer: None,
                annotated: false,
                served_seq: None,
                popped: 0,
                kind: Some(ReadKind::NoFrame),
            },
        );
        let p = plan(&adm, &idx).expect("plans");
        assert!(p.read(&key(), 1, 0).is_none());
        assert_eq!(p.read(&key(), 1, 1).unwrap().frames, vec![span(1)]);
    }
}
