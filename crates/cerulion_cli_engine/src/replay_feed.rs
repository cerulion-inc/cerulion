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
//! This module holds the four parts of that:
//!
//! * [`FeedNeed`]: the frame identities the selected reads look up, charged
//!   against the feed index budget before the bag is walked.
//! * [`FrameIndex`]: the bag's recorded frames of the consumed topics, keyed by
//!   `(writer token, wire sequence)`, built by [`FrameIndexBuilder`] and, under
//!   a need, holding only the identities it names. A multi-writer topic's
//!   frames carry the recorder's per-frame producer labels; a topic with no
//!   label at all was written by one writer.
//! * [`plan_feeds`]: for every gated admission that pops, the list of frames it
//!   consumed, in pop order. A read whose frames cannot be named is REFUSED by
//!   node, input, writer and sequence ([`FeedRefusal`]); there is no fallback
//!   to timestamps.
//! * [`BagReadFeed`]: the `ReadFeed` the runtime's gates call, which publishes a
//!   planned read's frames verbatim from the mapped bag.
//!
//! # Which frames a read consumed
//!
//! A read record names ONE frame, the one it served. From `trace_format` 9 a
//! read that popped more than one frame is preceded by one `BatchFrame` record
//! per other frame it consumed, in pop order, each naming that frame's
//! sequence and, on an annotated edge, its writer. Nothing is derived from the
//! served sequence: on a `drop_oldest` queue a writer can refill and overflow
//! the queue while the node's callback runs between two receives of one batch,
//! so a depth-two queue can hand one batch 1, then 19 and 20. One function
//! yields a read's consumed list (its batch frames, then the served frame);
//! [`plan_feeds`] looks up exactly that list and [`FeedNeed`] names exactly
//! that list, under one key rule, so an index built under a need holds every
//! frame the plan looks up. A multi-frame read recorded without its batch
//! frames (a bag below format 9) is refused
//! ([`FeedRefusalReason::BatchUnattributed`]). A `sample(N)` read that dropped
//! the frames it popped names none of them, and is refused the same way
//! ([`FeedRefusalReason::DecimatedUnattributed`]).

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fmt;
use std::sync::{Arc, Mutex};

use cerulion_bag::{FrameSpan, ProducerAttribution};
use cerulion_core::read_outcome::{DueShape, StageKey};
use cerulion_core::replay_feed::{ReadFeed, ReadFeedFault};
use cerulion_core::transport::publisher::CerulionPublisher;

use crate::replay_inject::{ReadIdentity, ReadKind, StageAdmission};

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
    /// The identities held, sorted and distinct. Labeled: `(writer token,
    /// sequence)`. Unlabeled: `(0, sequence)`, the writer being the topic's
    /// only one. Under a [`FeedNeed`] these are the need's own keys for the
    /// topic, moved in at build.
    keys: Vec<(u64, u32)>,
    /// Parallel to `keys`: what the bag holds at each identity, `None` when it
    /// holds no frame there.
    slots: Vec<Option<Slot>>,
    /// Built with no need: every headered frame's identity and span, in file
    /// order, sorted into `keys` and `slots` by [`FrameIndexBuilder::finish`].
    unsorted: Vec<((u64, u32), FrameSpan)>,
    /// Headered frames on the topic, kept or not.
    headered: usize,
    /// Labeled frames whose writer the labels do not state (a gap or a
    /// contradiction in the label stream).
    unattributed: usize,
    /// The distinct writers the labels name.
    writers: BTreeSet<u64>,
    /// Every headered frame in file order, for the timestamp-feed mutant.
    #[cfg(feature = "oracle-mutants")]
    ordered: Vec<(u64, FrameSpan)>,
}

impl TopicIndex {
    /// What the bag holds at `key`, `None` when it holds no frame there.
    fn get(&self, key: (u64, u32)) -> Option<&Slot> {
        let pos = self.keys.binary_search(&key).ok()?;
        self.slots[pos].as_ref()
    }

    /// One more recorded frame at `slot`: the first is `One`, each later one
    /// makes it `Many`.
    fn take_frame(slot: &mut Option<Slot>, span: FrameSpan) -> bool {
        let (next, first) = match *slot {
            None => (Slot::One(span), true),
            Some(Slot::One(_)) => (Slot::Many(2), false),
            Some(Slot::Many(n)) => (Slot::Many(n + 1), false),
        };
        *slot = Some(next);
        first
    }
}

/// The recorded frames of the topics a rank consumes, keyed by identity.
#[derive(Debug, Default)]
pub struct FrameIndex {
    topics: HashMap<String, TopicIndex>,
    /// Identity entries held, over every topic.
    entries: u64,
}

/// One topic's producer labels, in file order: what each label attributes, and
/// to which port.
pub type TopicLabels = Vec<(ProducerAttribution, u128)>;

/// `topic`'s labels in `labels`, empty when the recorder wrote none for it. A
/// topic with at least one is LABELED.
fn labels_of<'l>(
    labels: &'l BTreeMap<String, TopicLabels>,
    topic: &str,
) -> &'l [(ProducerAttribution, u128)] {
    labels.get(topic).map_or(&[], Vec::as_slice)
}

/// The key a topic's index holds the frame of `writer` at `seq` under:
/// `(writer, seq)` on a labeled topic, `(0, seq)` on an unlabeled one.
fn index_key(labeled: bool, writer: u64, seq: u32) -> (u64, u32) {
    (if labeled { writer } else { 0 }, seq)
}

/// The `(writer token, sequence)` of every frame the read `id` consumed, in
/// pop order: its recorded batch frames, each under its own writer (`writer`
/// where the record names none), then the frame it served, `(writer, served)`.
///
/// [`plan_feeds`] looks up exactly these frames and [`FeedNeed::add`] names
/// exactly these, each through [`index_key`].
fn consumed_frames(
    id: &ReadIdentity,
    writer: u64,
    served: u32,
) -> impl Iterator<Item = (u64, u32)> + '_ {
    id.batch
        .iter()
        .map(move |f| (f.writer.unwrap_or(writer), f.sequence))
        .chain(std::iter::once((writer, served)))
}

/// Bytes one [`FrameIndex`] entry is charged against the feed index budget:
/// its `(writer, sequence)` key and its slot, the two arrays an index built
/// under a [`FeedNeed`] holds one element of per identity the need names.
pub const FEED_INDEX_ENTRY_BYTES: u64 =
    (std::mem::size_of::<(u64, u32)>() + std::mem::size_of::<Option<Slot>>()) as u64;

/// Bytes one producer label of a consumed topic is charged against the feed
/// index budget: the label as the sidecar read holds it, and its position in
/// the order the index build resolves writers through.
pub const FEED_LABEL_BYTES: u64 =
    (std::mem::size_of::<(ProducerAttribution, u128)>() + std::mem::size_of::<usize>()) as u64;

/// The feed index budget a fed replay runs under when none is given: 1 GiB.
pub const DEFAULT_FEED_INDEX_BUDGET: u64 = 1 << 30;

/// The identities a set of fed reads looks up, per topic, in the index's own
/// key space: `(writer token, sequence)` on a labeled topic, `(0, sequence)` on
/// an unlabeled one. Each topic's identities are held sorted and distinct, one
/// array element each, and an index built under the need takes that array as
/// its own keys.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct FeedNeed {
    by_topic: HashMap<String, Vec<(u64, u32)>>,
    entries: u64,
}

impl FeedNeed {
    /// Add the identities `stages` look up at steps below `step_end` (`None`
    /// = every step): for each read that pops and names its writer and served
    /// frame, every frame of its consumed list, under the key an index built
    /// with the same `labels` holds that frame under. A topic is labeled when
    /// `labels` holds a label for it.
    pub fn add(
        &mut self,
        stages: &[FedStage<'_>],
        labels: &BTreeMap<String, TopicLabels>,
        step_end: Option<u64>,
    ) {
        for stage in stages {
            let labeled = !labels_of(labels, stage.topic).is_empty();
            for step in &stage.admission.per_step {
                if step_end.is_some_and(|end| step.step >= end) {
                    continue;
                }
                for (due, id) in step.due.iter().zip(&step.identity) {
                    if !matches!(due.shape(), DueShape::Pops { .. }) {
                        continue;
                    }
                    let (Some(writer), Some(served)) = (id.writer, id.served_seq) else {
                        continue;
                    };
                    let keys = self.by_topic.entry(stage.topic.to_string()).or_default();
                    #[cfg(feature = "oracle-mutants")]
                    if mutants::need_counted_back() {
                        for back in 0..due.pop_count() {
                            keys.push(index_key(labeled, writer, served.wrapping_sub(back)));
                        }
                        continue;
                    }
                    for (frame_writer, seq) in consumed_frames(id, writer, served) {
                        keys.push(index_key(labeled, frame_writer, seq));
                    }
                }
            }
        }
        let mut entries = 0u64;
        for keys in self.by_topic.values_mut() {
            keys.sort_unstable();
            keys.dedup();
            keys.shrink_to_fit();
            entries += keys.len() as u64;
        }
        self.entries = entries;
    }

    /// Distinct identities named.
    #[must_use]
    pub fn entries(&self) -> u64 {
        self.entries
    }

    /// Bytes an index holding every named identity is charged.
    #[must_use]
    pub fn index_bytes(&self) -> u64 {
        self.entries.saturating_mul(FEED_INDEX_ENTRY_BYTES)
    }

    /// `Err` with the bytes needed when the index and the `labels` of the
    /// consumed topics the build resolves writers through exceed `budget`.
    ///
    /// # Errors
    ///
    /// [`FeedIndexOverBudget`] when [`Self::index_bytes`] plus
    /// [`FEED_LABEL_BYTES`] per label is above `budget`.
    pub fn check_budget(
        &self,
        budget: u64,
        labels: &BTreeMap<String, TopicLabels>,
    ) -> Result<(), FeedIndexOverBudget> {
        let label_count: u64 = labels.values().map(|l| l.len() as u64).sum();
        let needed = self
            .index_bytes()
            .saturating_add(label_count.saturating_mul(FEED_LABEL_BYTES));
        if needed > budget {
            return Err(FeedIndexOverBudget {
                budget,
                needed,
                entries: self.entries,
                labels: label_count,
            });
        }
        Ok(())
    }
}

/// A feed index the selected reads need is larger than the budget allows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FeedIndexOverBudget {
    /// The budget, in bytes.
    pub budget: u64,
    /// The bytes the index needs.
    pub needed: u64,
    /// The identity entries the index needs.
    pub entries: u64,
    /// The producer labels of the consumed topics the build reads.
    pub labels: u64,
}

/// The ordinal the label at `pos` of `labels` names: `Some` for a
/// `FrameLabel` whose ordinal fits a `usize`, `None` otherwise.
fn label_ordinal(labels: &[(ProducerAttribution, u128)], pos: usize) -> Option<usize> {
    match labels[pos].0 {
        ProducerAttribution::FrameLabel { frame_index } => usize::try_from(frame_index).ok(),
        ProducerAttribution::CatchUpPrefix { .. } => None,
    }
}

/// Which writer the labels give one ordinal of a topic.
#[derive(Debug, Default)]
struct LabelResolver<'l> {
    /// The topic's labels, in file order.
    labels: &'l [(ProducerAttribution, u128)],
    /// The label positions of every `FrameLabel`, sorted by the ordinal it
    /// names and then by position.
    frame_labels: Vec<usize>,
    /// Every `CatchUpPrefix` as `(label position, prefix length, port,
    /// contested)`, in label order.
    prefixes: Vec<(usize, usize, u128, bool)>,
}

impl<'l> LabelResolver<'l> {
    fn new(labels: &'l [(ProducerAttribution, u128)]) -> Self {
        let mut out = Self {
            labels,
            ..Self::default()
        };
        let mut first_prefix: Option<u128> = None;
        for (pos, (attribution, id)) in labels.iter().enumerate() {
            match *attribution {
                ProducerAttribution::FrameLabel { .. } => out.frame_labels.push(pos),
                ProducerAttribution::CatchUpPrefix { prefix_len } => {
                    let contested = first_prefix.is_some_and(|p| p != *id);
                    first_prefix.get_or_insert(*id);
                    let end = usize::try_from(prefix_len).unwrap_or(usize::MAX);
                    out.prefixes.push((pos, end, *id, contested));
                }
            }
        }
        // Stable, so the labels naming one ordinal stay in label order.
        out.frame_labels
            .sort_by_key(|&pos| label_ordinal(labels, pos));
        out
    }

    /// The writer of `ordinal`: `None` = unstated, `Some(None)` = contradicted.
    /// Folds the labels covering it in label order.
    fn writer_of(&self, ordinal: usize) -> Option<Option<u128>> {
        let start = self
            .frame_labels
            .partition_point(|&pos| label_ordinal(self.labels, pos) < Some(ordinal));
        let mut frames = self.frame_labels[start..]
            .iter()
            .take_while(|&&pos| label_ordinal(self.labels, pos) == Some(ordinal))
            .map(|&pos| (pos, self.labels[pos].1, false))
            .peekable();
        let mut prefixes = self
            .prefixes
            .iter()
            .filter(|(_, end, _, _)| ordinal < *end)
            .map(|&(pos, _, id, contested)| (pos, id, contested))
            .peekable();
        let mut slot: Option<Option<u128>> = None;
        loop {
            // The two lists are each in label order; take the earlier head.
            let frame_first = match (frames.peek(), prefixes.peek()) {
                (Some(f), Some(p)) => f.0 < p.0,
                (Some(_), None) => true,
                (None, Some(_)) => false,
                (None, None) => break,
            };
            let next = if frame_first {
                frames.next()
            } else {
                prefixes.next()
            };
            let Some((_, id, contested)) = next else {
                break;
            };
            slot = match slot {
                None if contested => None,
                None => Some(Some(id)),
                Some(Some(prev)) if prev == id => Some(Some(id)),
                Some(_) => Some(None),
            };
        }
        slot
    }
}

/// Builds a [`FrameIndex`] from one walk of the bag, frame by frame.
#[derive(Debug)]
pub struct FrameIndexBuilder<'l> {
    topics: HashMap<String, (TopicIndex, LabelResolver<'l>, usize)>,
    /// Built under a need: only the identities it named have slots.
    needed: bool,
    entries: u64,
}

impl<'l> FrameIndexBuilder<'l> {
    /// A builder over `topics` under `labels` (per topic). With `need`, only
    /// the identities it names are kept: each topic's index takes the need's
    /// keys for it, with one empty slot per key.
    #[must_use]
    pub fn new<I: IntoIterator<Item = String>>(
        topics: I,
        labels: &'l BTreeMap<String, TopicLabels>,
        need: Option<FeedNeed>,
    ) -> Self {
        let needed = need.is_some();
        let mut need_keys = need.map(|n| n.by_topic).unwrap_or_default();
        let topics = topics
            .into_iter()
            .map(|topic| {
                let topic_labels = labels_of(labels, &topic);
                let keys = need_keys.remove(&topic).unwrap_or_default();
                let idx = TopicIndex {
                    labeled: !topic_labels.is_empty(),
                    slots: vec![None; keys.len()],
                    keys,
                    ..TopicIndex::default()
                };
                (topic, (idx, LabelResolver::new(topic_labels), 0))
            })
            .collect();
        Self {
            topics,
            needed,
            entries: 0,
        }
    }

    /// Take the next frame of `topic`, in the topic's file order. A topic the
    /// builder was not given is ignored.
    pub fn push(&mut self, topic: &str, frame: RecordedFrame) {
        let Some((idx, resolver, next_ordinal)) = self.topics.get_mut(topic) else {
            return;
        };
        let ordinal = *next_ordinal;
        *next_ordinal += 1;
        let Some(seq) = frame.sequence else {
            return;
        };
        idx.headered += 1;
        let writer = if idx.labeled {
            match resolver.writer_of(ordinal) {
                Some(Some(id)) => {
                    let token = writer_token(id);
                    idx.writers.insert(token);
                    token
                }
                _ => {
                    idx.unattributed += 1;
                    return;
                }
            }
        } else {
            0
        };
        #[cfg(feature = "oracle-mutants")]
        if let Some(ts) = frame.timestamp_ns {
            idx.ordered.push((ts, frame.span));
        }
        if !self.needed {
            idx.unsorted.push(((writer, seq), frame.span));
            return;
        }
        if let Ok(pos) = idx.keys.binary_search(&(writer, seq)) {
            if TopicIndex::take_frame(&mut idx.slots[pos], frame.span) {
                self.entries += 1;
            }
        }
    }

    /// The index.
    #[must_use]
    pub fn finish(self) -> FrameIndex {
        let mut entries = self.entries;
        let topics = self
            .topics
            .into_iter()
            .map(|(topic, (mut idx, _, _))| {
                let mut unsorted = std::mem::take(&mut idx.unsorted);
                // Stable, so the first frame at an identity stays the one
                // `Slot::One` holds.
                unsorted.sort_by_key(|&(key, _)| key);
                for (key, span) in unsorted {
                    if idx.keys.last() != Some(&key) {
                        idx.keys.push(key);
                        idx.slots.push(None);
                    }
                    if let Some(slot) = idx.slots.last_mut() {
                        if TopicIndex::take_frame(slot, span) {
                            entries += 1;
                        }
                    }
                }
                (topic, idx)
            })
            .collect();
        FrameIndex { topics, entries }
    }
}

impl FrameIndex {
    /// Index every frame of `frames` (per topic, in file order) under `labels`
    /// (per topic).
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
        let mut builder = FrameIndexBuilder::new(frames.keys().cloned(), labels, None);
        for (topic, list) in frames {
            for frame in list {
                builder.push(topic, *frame);
            }
        }
        builder.finish()
    }

    /// Identity entries held, over every topic.
    #[must_use]
    pub fn entries(&self) -> u64 {
        self.entries
    }

    /// Bytes the index's key and slot arrays hold, over every topic.
    #[must_use]
    pub fn held_bytes(&self) -> u64 {
        self.topics
            .values()
            .map(|t| {
                t.keys.capacity() * std::mem::size_of::<(u64, u32)>()
                    + t.slots.capacity() * std::mem::size_of::<Option<Slot>>()
            })
            .sum::<usize>() as u64
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
    /// A read popped `popped` frames and the recording does not name each one
    /// (a bag below `trace_format` 9 names only the newest).
    BatchUnattributed {
        /// Frames the read popped.
        popped: u32,
    },
    /// A read popped `popped` frames and the recording names `named` frames
    /// it consumed, more than it popped: a batch frame record before it
    /// belongs to no frame of this read.
    BatchMiscounted {
        /// Frames the read popped.
        popped: u32,
        /// Frames the recording names for it: its batch frames and the frame
        /// it served.
        named: usize,
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
            Self::BatchMiscounted { .. } => "read_identity_batch_miscounted",
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
    /// The writer token of the frame that could not be fed, when the
    /// recording names one: the read's own writer, or the writer of the
    /// frame of a batch the lookup failed on.
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
                "the read popped {popped} frames and the recording does not name each of \
                 them (recorded below trace format 9, or a frame record was lost)"
            ),
            FeedRefusalReason::BatchMiscounted { popped, named } => write!(
                f,
                "the read popped {popped} frame(s) and the recording names {named} frames it \
                 consumed, so a batch frame record before it names no frame of this read"
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

    thread_local! {
        static NEED_COUNTED_BACK: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    }

    /// A scope that arms the counted-back need on its own thread until it
    /// drops. Thread-local, so a test arming it changes no other test's need.
    #[derive(Debug)]
    pub struct NeedGuard(());

    impl Drop for NeedGuard {
        fn drop(&mut self) {
            NEED_COUNTED_BACK.with(|c| c.set(false));
        }
    }

    /// Arm the COUNTED-BACK need: [`super::FeedNeed::add`] names, for a read
    /// that popped `n` frames, the `n` sequences of its writer ending at the
    /// served one instead of the frames its recording names, while
    /// [`super::plan_feeds`] still looks up the recorded frames.
    #[must_use]
    pub fn need_counted_back_from_served() -> NeedGuard {
        NEED_COUNTED_BACK.with(|c| c.set(true));
        NeedGuard(())
    }

    pub(super) fn need_counted_back() -> bool {
        NEED_COUNTED_BACK.with(std::cell::Cell::get)
    }

    thread_local! {
        static BATCH_FRAME_KEEPS_ANNOTATION: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    }

    /// A scope that arms the kept annotation on its own thread until it drops.
    #[derive(Debug)]
    pub struct AnnotationGuard(());

    impl Drop for AnnotationGuard {
        fn drop(&mut self) {
            BATCH_FRAME_KEEPS_ANNOTATION.with(|c| c.set(false));
        }
    }

    /// Arm the KEPT ANNOTATION: the admission planner leaves the annotation
    /// before a `BatchFrame` record pending, so the read after it counts as
    /// annotated with no annotation of its own.
    #[must_use]
    pub fn batch_frame_keeps_annotation() -> AnnotationGuard {
        BATCH_FRAME_KEEPS_ANNOTATION.with(|c| c.set(true));
        AnnotationGuard(())
    }

    pub(crate) fn batch_frame_keeps_annotation_armed() -> bool {
        BATCH_FRAME_KEEPS_ANNOTATION.with(std::cell::Cell::get)
    }
}

/// Plan every fed read of a pass.
///
/// For each admission that pops, the frames its recorded identity names, looked
/// up in `index`. Steps at or past `step_end` (`None` = no bound) are not
/// planned. `step_targets` maps a step to its recorded boundary target and is
/// read only by the timestamp mutant.
///
/// # Errors
///
/// The first read whose frames cannot be named, as a [`FeedRefusal`].
pub fn plan_feeds(
    stages: &[FedStage<'_>],
    index: &FrameIndex,
    step_targets: &BTreeMap<u64, u64>,
    step_end: Option<u64>,
) -> Result<FeedPlan, FeedRefusal> {
    let _ = step_targets;
    // Pass 1: an unlabeled topic must be read under ONE writer, counting the
    // writers its reads' batch frames name.
    let mut writers_by_topic: BTreeMap<&str, BTreeSet<u64>> = BTreeMap::new();
    for stage in stages {
        for step in &stage.admission.per_step {
            for id in &step.identity {
                let named = id.batch.iter().filter_map(|f| f.writer).chain(id.writer);
                writers_by_topic
                    .entry(stage.topic)
                    .or_default()
                    .extend(named);
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
            if step_end.is_some_and(|end| step.step >= end) {
                continue;
            }
            for (ordinal, (due, id)) in step.due.iter().zip(&step.identity).enumerate() {
                // The recorder writes batch frames before a read that popped
                // several, one per frame it consumed besides the one it
                // served, so a read naming more frames than it popped carries a
                // record of no frame it consumed.
                let named = id.batch.len() + usize::from(id.served_seq.is_some());
                if !id.batch.is_empty() && named > due.pop_count() as usize {
                    return Err(refuse(
                        step.step,
                        id.writer,
                        id.served_seq,
                        FeedRefusalReason::BatchMiscounted {
                            popped: due.pop_count(),
                            named,
                        },
                    ));
                }
                let DueShape::Pops { pops, .. } = due.shape() else {
                    continue;
                };
                let ordinal = u32::try_from(ordinal).unwrap_or(u32::MAX);
                let Some(topic) = index.topics.get(stage.topic).filter(|t| t.headered > 0) else {
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
                // The same rule per batch frame: on a multi-writer edge each
                // carries an annotation of its own.
                if let Some(lost) = id.batch.iter().find(|f| stage.every_read && !f.annotated) {
                    return Err(refuse(
                        step.step,
                        None,
                        Some(lost.sequence),
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
                }
                // The read's consumed list (`consumed_frames`) is its recorded
                // batch frames, then the frame it names. A multi-frame read
                // whose batch frames do not account for every pop is refused
                // rather than derived from `served`.
                if pops > 1 && id.batch.len() + 1 != pops as usize {
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
                // Each refusal names the frame whose lookup failed: its own
                // writer and sequence.
                for (frame_writer, seq) in consumed_frames(id, writer, served) {
                    let lookup = index_key(topic.labeled, frame_writer, seq);
                    if let Some(first_step) = claimed.insert(lookup, step.step) {
                        return Err(refuse(
                            step.step,
                            Some(frame_writer),
                            Some(seq),
                            FeedRefusalReason::FrameClaimedTwice { first_step },
                        ));
                    }
                    match topic.get(lookup) {
                        Some(Slot::One(span)) => {
                            read.frames.push(*span);
                            read.ids.push((frame_writer, seq));
                        }
                        Some(Slot::Many(frames)) => {
                            return Err(refuse(
                                step.step,
                                Some(frame_writer),
                                Some(seq),
                                FeedRefusalReason::WritersIndistinguishable { frames: *frames },
                            ));
                        }
                        None => {
                            return Err(refuse(
                                step.step,
                                Some(frame_writer),
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
    use crate::replay_inject::{ConsumedFrame, ReadIdentity, StepAdmission};
    use cerulion_core::read_outcome::{DueRead, ReadStageRole};

    /// No producer label on any topic.
    static NO_LABELS: BTreeMap<String, TopicLabels> = BTreeMap::new();

    /// Batch frames as a multi-writer recorder stages them: each `(writer,
    /// sequence)` after an annotation of its own.
    fn frames_of(list: &[(Option<u64>, u32)]) -> Vec<ConsumedFrame> {
        list.iter()
            .map(|&(writer, sequence)| ConsumedFrame {
                writer,
                sequence,
                annotated: true,
            })
            .collect()
    }

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
                batch: Vec::new(),
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
            None,
        )
    }

    fn need_of(adm: &StageAdmission, step_end: Option<u64>) -> FeedNeed {
        let mut need = FeedNeed::default();
        need.add(
            &[FedStage {
                admission: adm,
                input: "inp",
                topic: "/t",
                every_read: false,
            }],
            &BTreeMap::new(),
            step_end,
        );
        need
    }

    /// An index built under a need keeps only the identities the reads name,
    /// and the plan over it is the plan over the whole index.
    #[test]
    fn an_index_under_a_need_keeps_only_the_named_frames() {
        let frames: Vec<RecordedFrame> = (1..=10).map(|s| frame(s as usize * 10, s)).collect();
        let mut adm = admission(&[
            (1, 3, Some(7), 2, ReadKind::Served),
            (2, 4, Some(7), 1, ReadKind::Served),
        ]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(7), 2)]);
        let need = need_of(&adm, None);
        assert_eq!(need.entries(), 3);
        let mut b = FrameIndexBuilder::new(["/t".to_string()], &NO_LABELS, Some(need.clone()));
        for f in &frames {
            b.push("/t", *f);
        }
        let narrow = b.finish();
        assert_eq!(narrow.entries(), 3);
        let whole = index(frames, vec![]);
        assert_eq!(whole.entries(), 10);
        let a = plan(&adm, &narrow).expect("plans over the narrow index");
        let b = plan(&adm, &whole).expect("plans over the whole index");
        assert_eq!(a.read(&key(), 1, 0), b.read(&key(), 1, 0));
        assert_eq!(a.read(&key(), 2, 0), b.read(&key(), 2, 0));
    }

    /// A step end leaves the reads at and past it out of the need and the plan.
    #[test]
    fn a_step_end_bounds_the_need_and_the_plan() {
        let adm = admission(&[
            (1, 1, Some(7), 1, ReadKind::Served),
            (2, 2, Some(7), 1, ReadKind::Served),
            (3, 3, Some(7), 1, ReadKind::Served),
        ]);
        let need = need_of(&adm, Some(2));
        assert_eq!(need.entries(), 1);
        let mut b = FrameIndexBuilder::new(["/t".to_string()], &NO_LABELS, Some(need.clone()));
        for s in 1..=3 {
            b.push("/t", frame(s as usize, s));
        }
        let idx = b.finish();
        let p = plan_feeds(
            &[FedStage {
                admission: &adm,
                input: "inp",
                topic: "/t",
                every_read: false,
            }],
            &idx,
            &BTreeMap::new(),
            Some(2),
        )
        .expect("plans the bounded steps");
        assert_eq!(p.reads_planned(), 1);
    }

    /// The budget admits a need at or under it and refuses one byte over.
    #[test]
    fn the_feed_index_budget_admits_at_and_under_and_refuses_over() {
        let mut adm = admission(&[(1, 3, Some(7), 3, ReadKind::Served)]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(7), 1), (Some(7), 2)]);
        let need = need_of(&adm, None);
        let bytes = 3 * FEED_INDEX_ENTRY_BYTES;
        assert_eq!(need.index_bytes(), bytes);
        let none = BTreeMap::new();
        assert_eq!(need.check_budget(bytes + 1, &none), Ok(()));
        assert_eq!(need.check_budget(bytes, &none), Ok(()));
        assert_eq!(
            need.check_budget(bytes - 1, &none),
            Err(FeedIndexOverBudget {
                budget: bytes - 1,
                needed: bytes,
                entries: 3,
                labels: 0,
            })
        );
    }

    /// The depth-two overflow batch (1, then 19 and 20 after 2 to 18 were
    /// evicted mid-drain) of an unlabeled topic holding frames 1 to 20: its
    /// need, and its plan over the index built under that need.
    fn overflow_batch_through_a_budgeted_index() -> (FeedNeed, Result<FeedPlan, FeedRefusal>) {
        let mut adm = admission(&[(4, 20, Some(7), 3, ReadKind::DrainedBatch)]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(7), 1), (Some(7), 19)]);
        let need = need_of(&adm, None);
        let mut b = FrameIndexBuilder::new(["/t".to_string()], &NO_LABELS, Some(need.clone()));
        for s in 1..=20u32 {
            b.push("/t", frame(s as usize * 10, s));
        }
        let idx = b.finish();
        (need, plan(&adm, &idx))
    }

    /// Through the budgeted index (need, index under the need, plan), the
    /// overflowed batch is fed exactly the frames it recorded, and its need
    /// holds exactly those three.
    #[test]
    fn a_budgeted_index_feeds_an_overflowed_batch_exactly_its_recorded_frames() {
        let (need, planned) = overflow_batch_through_a_budgeted_index();
        assert_eq!(need.entries(), 3);
        let p = planned.expect("plans over the index built under the need");
        let read = p.read(&key(), 4, 0).expect("read planned");
        assert_eq!(read.frames, vec![span(10), span(190), span(200)]);
        assert_eq!(read.ids, vec![(7, 1), (7, 19), (7, 20)]);
    }

    /// MUTANT (`oracle-mutants`): the need counts back from the served frame
    /// (18, 19, 20) while the plan looks up the recorded frames, so the index
    /// built under it lacks frame 1 and the budgeted overflow test fails.
    #[cfg(feature = "oracle-mutants")]
    #[test]
    fn the_budgeted_overflow_test_catches_a_need_counted_back_from_the_served_frame() {
        let _armed = mutants::need_counted_back_from_served();
        let (need, planned) = overflow_batch_through_a_budgeted_index();
        assert_eq!(need.entries(), 3, "the mutant names 18, 19 and 20");
        let refusal = planned.expect_err("frame 1 is outside the mutant's index");
        assert_eq!(refusal.reason, FeedRefusalReason::FrameMissing);
        assert_eq!(refusal.sequence, Some(1));
        let caught = std::panic::catch_unwind(
            a_budgeted_index_feeds_an_overflowed_batch_exactly_its_recorded_frames,
        );
        assert!(
            caught.is_err(),
            "the budgeted overflow test passed under the mutant"
        );
    }

    /// On a labeled topic each batch frame is needed and fed under its own
    /// writer: three writers at sequence 5, one batch consuming all three.
    #[test]
    fn a_labeled_batch_is_needed_and_fed_under_each_frames_writer() {
        let ids = [11u128, 22, 33];
        let labels: TopicLabels = ids
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
        let (a, b, c) = (writer_token(11), writer_token(22), writer_token(33));
        let mut adm = admission(&[(1, 5, Some(c), 3, ReadKind::DrainedBatch)]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(a), 5), (Some(b), 5)]);
        let mut by_topic = BTreeMap::new();
        by_topic.insert("/t".to_string(), labels);
        let mut need = FeedNeed::default();
        need.add(
            &[FedStage {
                admission: &adm,
                input: "inp",
                topic: "/t",
                every_read: false,
            }],
            &by_topic,
            None,
        );
        assert_eq!(need.entries(), 3);
        let mut builder = FrameIndexBuilder::new(["/t".to_string()], &by_topic, Some(need.clone()));
        for f in [frame(1, 5), frame(2, 5), frame(3, 5)] {
            builder.push("/t", f);
        }
        let p = plan(&adm, &builder.finish()).expect("plans");
        let read = p.read(&key(), 1, 0).expect("read planned");
        assert_eq!(read.frames, vec![span(1), span(2), span(3)]);
        assert_eq!(read.ids, vec![(a, 5), (b, 5), (c, 5)]);
    }

    /// An unlabeled topic whose reads name one writer while a batch frame
    /// names another is refused: the index keys its frames by sequence alone.
    #[test]
    fn an_unlabeled_topic_counts_the_writers_its_batch_frames_name() {
        let idx = index(vec![frame(10, 1), frame(20, 2)], vec![]);
        let mut adm = admission(&[(4, 2, Some(7), 2, ReadKind::DrainedBatch)]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(8), 1)]);
        assert_eq!(
            plan(&adm, &idx).unwrap_err().reason,
            FeedRefusalReason::FramesUnattributed { writers: 2 }
        );
    }

    /// A catch-up prefix and a frame label that disagree leave the frame
    /// unattributed, read frame by frame as the whole-list build did.
    #[test]
    fn labels_resolve_per_ordinal_in_label_order() {
        let labels: TopicLabels = vec![
            (ProducerAttribution::CatchUpPrefix { prefix_len: 2 }, 11),
            (ProducerAttribution::FrameLabel { frame_index: 1 }, 22),
            (ProducerAttribution::FrameLabel { frame_index: 2 }, 22),
        ];
        let r = LabelResolver::new(&labels);
        assert_eq!(r.writer_of(0), Some(Some(11)));
        assert_eq!(r.writer_of(1), Some(None));
        assert_eq!(r.writer_of(2), Some(Some(22)));
        assert_eq!(r.writer_of(3), None);
    }

    /// A single-writer Latest read that popped 2 after the live queue evicted
    /// seq 1 is fed exactly 2 and 3, never 1.
    #[test]
    fn a_batch_on_one_writer_is_fed_the_run_ending_at_the_served_frame() {
        let idx = index(vec![frame(10, 1), frame(20, 2), frame(30, 3)], vec![]);
        let mut adm = admission(&[(4, 3, Some(7), 2, ReadKind::Served)]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(7), 2)]);
        let p = plan(&adm, &idx).expect("plans");
        let read = p.read(&key(), 4, 0).expect("read planned");
        assert_eq!(read.frames, vec![span(20), span(30)]);
        assert_eq!(read.ids, vec![(7, 2), (7, 3)]);
        assert_eq!((p.reads_planned(), p.frames_planned()), (1, 2));
        assert_eq!(p.buffers().get(&key()), Some(&2));
    }

    /// A depth-two `drop_oldest` queue overflowed mid-batch: the batch
    /// consumed 1, then 19 and 20 after 2 to 18 were evicted. The recording
    /// names 1 and 19 as batch frames, and the feed takes exactly 1, 19, 20,
    /// never the 18, 19, 20 a count back from the served frame would name.
    #[test]
    fn a_batch_overflowed_mid_drain_is_fed_the_frames_it_recorded() {
        let frames = (1..=20u32).map(|s| frame(s as usize * 10, s)).collect();
        let idx = index(frames, vec![]);
        let mut adm = admission(&[(4, 20, Some(7), 3, ReadKind::DrainedBatch)]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(7), 1), (Some(7), 19)]);
        let p = plan(&adm, &idx).expect("plans");
        let read = p.read(&key(), 4, 0).expect("read planned");
        assert_eq!(read.frames, vec![span(10), span(190), span(200)]);
        assert_eq!(read.ids, vec![(7, 1), (7, 19), (7, 20)]);
    }

    /// A multi-frame read whose recording names only its newest frame (a bag
    /// below trace format 9) is refused by name, never fed a derived run.
    #[test]
    fn a_batch_recorded_without_its_frames_is_refused() {
        let idx = index(vec![frame(10, 1), frame(20, 2), frame(30, 3)], vec![]);
        let adm = admission(&[(4, 3, Some(7), 2, ReadKind::DrainedBatch)]);
        assert_eq!(
            plan(&adm, &idx).unwrap_err().reason,
            FeedRefusalReason::BatchUnattributed { popped: 2 }
        );
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
        let mut adm = admission(&[(3, 7, Some(9), 2, ReadKind::Served)]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(9), 6)]);
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
                batch: Vec::new(),
            },
        );
        let p = plan(&adm, &idx).expect("plans");
        assert!(p.read(&key(), 1, 0).is_none());
        assert_eq!(p.read(&key(), 1, 1).unwrap().frames, vec![span(1)]);
    }

    /// Two writers on a labeled topic: A (port 11) at sequences 30 and 31, B
    /// (port 22) at 12 and 13, labeled one frame each in that order.
    fn two_writer_index(b_frames: &[(usize, u32)]) -> (FrameIndex, u64, u64) {
        let mut frames = vec![frame(1, 30), frame(2, 31)];
        let mut labels: TopicLabels = vec![
            (ProducerAttribution::FrameLabel { frame_index: 0 }, 11),
            (ProducerAttribution::FrameLabel { frame_index: 1 }, 11),
        ];
        for (i, &(offset, seq)) in b_frames.iter().enumerate() {
            frames.push(frame(offset, seq));
            labels.push((
                ProducerAttribution::FrameLabel {
                    frame_index: 2 + i as u64,
                },
                22,
            ));
        }
        (index(frames, labels), writer_token(11), writer_token(22))
    }

    /// A batch frame of writer B that the bag lost, on a read that served A's
    /// frame, is refused naming B and the lost sequence: the writer and the
    /// sequence of one frame, which the sentence says the bag lacks.
    #[test]
    fn a_missing_batch_frame_is_refused_under_its_own_writer() {
        let (idx, a, b) = two_writer_index(&[(3, 13)]);
        let mut adm = admission(&[(7, 30, Some(a), 2, ReadKind::DrainedBatch)]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(b), 12)]);
        let r = plan(&adm, &idx).unwrap_err();
        assert_eq!(r.reason, FeedRefusalReason::FrameMissing);
        assert_eq!((r.step, r.writer, r.sequence), (7, Some(b), Some(12)));
        assert!(
            r.to_string()
                .contains(&format!("writer {b:016x}, sequence 12:")),
            "{r}"
        );
    }

    /// A batch frame of writer B that an earlier read of the stage already
    /// consumed is refused naming B, the writer the claim is about.
    #[test]
    fn a_batch_frame_claimed_twice_is_refused_under_its_own_writer() {
        let (idx, a, b) = two_writer_index(&[(3, 12)]);
        let mut adm = admission(&[
            (1, 12, Some(b), 1, ReadKind::Served),
            (2, 30, Some(a), 2, ReadKind::DrainedBatch),
        ]);
        adm.per_step[1].identity[0].batch = frames_of(&[(Some(b), 12)]);
        let r = plan(&adm, &idx).unwrap_err();
        assert_eq!(
            r.reason,
            FeedRefusalReason::FrameClaimedTwice { first_step: 1 }
        );
        assert_eq!((r.step, r.writer, r.sequence), (2, Some(b), Some(12)));
    }

    /// A batch frame whose writer and sequence two recorded frames share is
    /// refused naming that writer, not the served frame's.
    #[test]
    fn an_indistinguishable_batch_frame_is_refused_under_its_own_writer() {
        let (idx, a, b) = two_writer_index(&[(3, 12), (4, 12)]);
        let mut adm = admission(&[(5, 30, Some(a), 2, ReadKind::DrainedBatch)]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(b), 12)]);
        let r = plan(&adm, &idx).unwrap_err();
        assert_eq!(
            r.reason,
            FeedRefusalReason::WritersIndistinguishable { frames: 2 }
        );
        assert_eq!((r.step, r.writer, r.sequence), (5, Some(b), Some(12)));
    }

    /// A read that popped one frame and carries a batch frame names more
    /// frames than it popped, and is refused by name rather than planned with
    /// two frames for one pop.
    #[test]
    fn a_single_pop_read_carrying_a_batch_frame_is_refused() {
        let idx = index(vec![frame(1, 4), frame(2, 5)], vec![]);
        let mut adm = admission(&[(3, 5, Some(7), 1, ReadKind::Served)]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(7), 4)]);
        let r = plan(&adm, &idx).unwrap_err();
        assert_eq!(r.reason.code(), "read_identity_batch_miscounted");
        assert_eq!((r.step, r.sequence), (3, Some(5)));
        assert!(
            r.to_string()
                .contains("the read popped 1 frame(s) and the recording names 2"),
            "{r}"
        );
    }

    /// A read that popped two frames and carries two batch frames names three,
    /// and is refused by name.
    #[test]
    fn a_batch_naming_more_frames_than_it_popped_is_refused() {
        let idx = index(vec![frame(1, 3), frame(2, 4), frame(3, 5)], vec![]);
        let mut adm = admission(&[(3, 5, Some(7), 2, ReadKind::DrainedBatch)]);
        adm.per_step[0].identity[0].batch = frames_of(&[(Some(7), 3), (Some(7), 4)]);
        let r = plan(&adm, &idx).unwrap_err();
        assert_eq!(r.reason.code(), "read_identity_batch_miscounted");
        assert!(
            r.to_string()
                .contains("the read popped 2 frame(s) and the recording names 3"),
            "{r}"
        );
    }

    /// A read that popped nothing and carries a batch frame is refused by name:
    /// batch frames are recorded before a read that popped several.
    #[test]
    fn a_read_that_popped_nothing_carrying_a_batch_frame_is_refused() {
        let idx = index(vec![frame(1, 4), frame(2, 5)], vec![]);
        let mut adm = admission(&[(1, 5, Some(7), 1, ReadKind::Served)]);
        adm.per_step[0].due.insert(0, DueRead::nothing());
        adm.per_step[0].identity.insert(
            0,
            ReadIdentity {
                writer: Some(7),
                annotated: false,
                served_seq: None,
                popped: 0,
                kind: Some(ReadKind::NoFrame),
                batch: frames_of(&[(Some(7), 4)]),
            },
        );
        let r = plan(&adm, &idx).unwrap_err();
        assert_eq!(r.reason.code(), "read_identity_batch_miscounted");
        assert!(
            r.to_string()
                .contains("the read popped 0 frame(s) and the recording names 1"),
            "{r}"
        );
    }

    /// One index entry is charged the bytes it takes: its `(writer, sequence)`
    /// key and its slot, 16 and 24 bytes on a 64-bit target.
    #[test]
    fn an_index_entry_is_charged_the_bytes_of_its_key_and_its_slot() {
        assert_eq!(
            FEED_INDEX_ENTRY_BYTES as usize,
            std::mem::size_of::<(u64, u32)>() + std::mem::size_of::<Option<Slot>>()
        );
        #[cfg(target_pointer_width = "64")]
        assert_eq!(FEED_INDEX_ENTRY_BYTES, 40);
    }

    /// An index built under a need holds, in its key and slot arrays, no more
    /// bytes than the need is charged: one key and one slot per identity. 897
    /// identities is one past the load a 1024 bucket hash table holds, the
    /// count where a table costs the most per entry.
    #[test]
    fn an_index_under_a_need_holds_no_more_bytes_than_it_is_charged() {
        let reads: Vec<(u64, u32, Option<u64>, u32, ReadKind)> = (1..=897u32)
            .map(|s| (u64::from(s), s, Some(7), 1, ReadKind::Served))
            .collect();
        let adm = admission(&reads);
        let need = need_of(&adm, None);
        assert_eq!(need.entries(), 897);
        let mut b = FrameIndexBuilder::new(["/t".to_string()], &NO_LABELS, Some(need.clone()));
        for s in 1..=1000u32 {
            b.push("/t", frame(s as usize, s));
        }
        let idx = b.finish();
        assert_eq!(idx.entries(), 897);
        assert!(
            idx.held_bytes() <= need.index_bytes(),
            "the index holds {} bytes, charged {}",
            idx.held_bytes(),
            need.index_bytes()
        );
        #[cfg(target_pointer_width = "64")]
        assert_eq!(need.index_bytes(), 897 * 40);
    }

    /// The budget charges the producer labels of the consumed topics beside
    /// the index entries: one identity on a topic with three labels is charged
    /// one entry and three labels, and refused one byte below that.
    #[test]
    fn the_budget_charges_the_labels_the_index_build_reads() {
        let labels: TopicLabels = (0..3u64)
            .map(|i| (ProducerAttribution::FrameLabel { frame_index: i }, 11u128))
            .collect();
        let by_topic = BTreeMap::from([("/t".to_string(), labels)]);
        let adm = admission(&[(1, 5, Some(writer_token(11)), 1, ReadKind::Served)]);
        let mut need = FeedNeed::default();
        need.add(
            &[FedStage {
                admission: &adm,
                input: "inp",
                topic: "/t",
                every_read: false,
            }],
            &by_topic,
            None,
        );
        let bytes = FEED_INDEX_ENTRY_BYTES + 3 * FEED_LABEL_BYTES;
        assert_eq!(need.check_budget(bytes, &by_topic), Ok(()));
        assert_eq!(
            need.check_budget(bytes - 1, &by_topic),
            Err(FeedIndexOverBudget {
                budget: bytes - 1,
                needed: bytes,
                entries: 1,
                labels: 3,
            })
        );
        #[cfg(target_pointer_width = "64")]
        assert_eq!(FEED_LABEL_BYTES, 40);
    }

    /// Frame labels out of file order resolve by the ordinal each names.
    #[test]
    fn labels_resolve_by_ordinal_out_of_file_order() {
        let labels: TopicLabels = vec![
            (ProducerAttribution::FrameLabel { frame_index: 2 }, 33),
            (ProducerAttribution::FrameLabel { frame_index: 0 }, 11),
            (ProducerAttribution::FrameLabel { frame_index: 1 }, 22),
            (ProducerAttribution::FrameLabel { frame_index: 0 }, 11),
        ];
        let r = LabelResolver::new(&labels);
        assert_eq!(r.writer_of(0), Some(Some(11)));
        assert_eq!(r.writer_of(1), Some(Some(22)));
        assert_eq!(r.writer_of(2), Some(Some(33)));
        assert_eq!(r.writer_of(3), None);
    }
}
