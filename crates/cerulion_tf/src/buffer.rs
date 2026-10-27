// SPDX-License-Identifier: AGPL-3.0-only
//! The tf buffer: a frame tree with a time-bounded ring per edge, and the
//! `lookup` that interpolates along the chain between two frames.
//!
//! See [`TfBuffer`] for the rules. Nothing here allocates after
//! construction: the frame table, the name table and every ring are sized
//! by [`TfBuffer::with_config`], so inserts and lookups run inside a node
//! tick without touching the heap.

use std::collections::VecDeque;
use std::num::{NonZeroU32, NonZeroUsize};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use crate::edge_policy::extrapolate;
use crate::error::Describe;
use crate::frame_id::normalize;
use crate::names::NameTable;
use crate::{EdgePolicy, FrameId, FrameLabel, TfError, Transform};

/// The frame capacity of [`TfBufferConfig::default`]: how many distinct
/// frame ids the buffer holds.
pub const DEFAULT_FRAME_CAPACITY: usize = 64;

/// The retention of [`TfBufferConfig::default`]: how far back from its
/// newest sample each dynamic edge keeps history, the tf2 default.
pub const DEFAULT_RETENTION: Duration = Duration::from_secs(10);

/// The publish rate bound of [`TfBufferConfig::default`], in Hz: the
/// fastest rate at which an edge keeps its whole retention window.
pub const DEFAULT_MAX_RATE_HZ: u32 = 100;

/// The jump limit of [`TfBufferConfig::new`]: how far past an edge's newest
/// stamp a dynamic insert may land before it needs a second stamp to
/// confirm it.
pub const DEFAULT_JUMP_LIMIT: Duration = Duration::from_secs(60);

/// The name bytes [`TfBufferConfig::new`] gives each frame, on average.
pub const NAME_BYTES_PER_FRAME: usize = 64;

/// The name byte budget of [`TfBufferConfig::default`]: the bytes the
/// buffer holds for all frame ids together.
pub const DEFAULT_NAME_BYTES: usize = NAME_BYTES_PER_FRAME * DEFAULT_FRAME_CAPACITY;

/// How a [`TfBuffer`] is sized. Everything the buffer will ever hold is
/// allocated from these numbers at construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TfBufferConfig {
    /// How many distinct frame ids the buffer holds.
    pub frames: NonZeroUsize,
    /// How far back from its newest sample a dynamic edge keeps history.
    pub retention: Duration,
    /// How many samples each dynamic edge's ring holds. An edge that
    /// publishes more samples than this inside one retention window fills
    /// its ring before its window, and every sample that pushes out counts
    /// in [`EdgeInfo::overflows`]. [`Self::new`] sizes it from a rate bound.
    pub ring_capacity: NonZeroUsize,
    /// The bytes the buffer holds for all frame ids together. A frame id
    /// has no length limit of its own; a new frame that does not fit in
    /// what is left is refused ([`TfError::NameBudgetFull`]).
    pub name_bytes: usize,
    /// How far past an edge's newest stamp a dynamic insert may land without
    /// a second stamp to confirm it; the buffer uses the larger of this and
    /// [`Self::retention`]. It is apart from the retention so that an edge
    /// that publishes less often than its window keeps updating while each
    /// gap is within it; a stamp past it is refused with
    /// [`TfError::StampTooFarAhead`] until a later insert within the limit
    /// of the refused stamp confirms the jump. Set it to at least the longest
    /// gap an edge publishes with, or `Duration::MAX` to turn the check off.
    pub jump_limit: Duration,
}

impl Default for TfBufferConfig {
    /// 64 frames, 10 s of history at up to 100 Hz per edge, 4096 bytes of
    /// frame ids, and a 60 s jump limit.
    fn default() -> Self {
        Self::new(
            NonZeroUsize::new(DEFAULT_FRAME_CAPACITY).expect("a nonzero default"),
            DEFAULT_RETENTION,
            NonZeroU32::new(DEFAULT_MAX_RATE_HZ).expect("a nonzero default"),
        )
    }
}

impl TfBufferConfig {
    /// `frames` frames, `retention` of history per edge, rings that hold
    /// the whole window for an edge publishing at up to `max_rate_hz`
    /// (`retention * max_rate_hz + 1` samples, rounded down),
    /// [`NAME_BYTES_PER_FRAME`] name bytes per frame, and the
    /// [`DEFAULT_JUMP_LIMIT`].
    ///
    /// # Panics
    ///
    /// When the ring capacity or the name budget does not fit a `usize`.
    pub fn new(frames: NonZeroUsize, retention: Duration, max_rate_hz: NonZeroU32) -> Self {
        let per_window = retention.as_nanos() * u128::from(max_rate_hz.get()) / 1_000_000_000;
        let ring_capacity = usize::try_from(per_window)
            .ok()
            .and_then(|n| n.checked_add(1))
            .and_then(NonZeroUsize::new)
            .expect("the ring capacity fits a usize");
        Self {
            frames,
            retention,
            ring_capacity,
            name_bytes: frames
                .get()
                .checked_mul(NAME_BYTES_PER_FRAME)
                .expect("the name budget fits a usize"),
            jump_limit: DEFAULT_JUMP_LIMIT,
        }
    }
}

/// One stamped transform `T_parent_child` on an edge's ring.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Sample {
    stamp_ns: u64,
    transform: Transform,
}

/// A frame and, when it has a parent, the edge from that parent to it.
#[derive(Debug)]
struct Frame {
    /// Index of the parent frame; `None` for a root.
    parent: Option<usize>,
    /// The edge came in through `insert_static`: `static_transform` holds
    /// at every time and `samples` is empty.
    is_static: bool,
    static_transform: Transform,
    policy: EdgePolicy,
    /// Dynamic samples on a ring, sorted by stamp, at most the ring
    /// capacity long.
    samples: VecDeque<Sample>,
    overflows: u64,
    static_refusals: u64,
    window_refusals: u64,
    jump_refusals: u64,
    /// The stamp of the last insert refused as [`TfError::StampTooFarAhead`]
    /// when no insert was accepted after it: the next insert at a later
    /// stamp within the jump limit of it confirms the jump.
    pending_jump: Option<u64>,
}

impl Frame {
    /// How far past its newest sample this edge answers, in nanoseconds.
    /// One sample alone gives no motion, so an extrapolating edge with one
    /// sample reaches only that sample.
    fn reach_ns(&self) -> u64 {
        match self.policy {
            EdgePolicy::Extrapolate { .. } if self.samples.len() < 2 => 0,
            policy => policy.reach_ns(),
        }
    }
}

/// What the buffer holds for one edge, read through [`TfBuffer::edge`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeInfo<'a> {
    /// The parent frame of the edge.
    pub parent: &'a str,
    /// The edge came in through [`TfBuffer::insert_static`].
    pub is_static: bool,
    /// The edge's policy past its newest sample.
    pub policy: EdgePolicy,
    /// Samples on the ring (always 0 for a static edge).
    pub samples: usize,
    /// The oldest buffered stamp, in nanoseconds; `None` when the ring is
    /// empty.
    pub oldest_ns: Option<u64>,
    /// The newest buffered stamp, in nanoseconds; `None` when the ring is
    /// empty.
    pub newest_ns: Option<u64>,
    /// Samples lost because the ring was full before its retention window:
    /// the edge publishes faster than its ring was sized for
    /// ([`TfBufferConfig::ring_capacity`]), so its history is shorter than
    /// the retention.
    pub overflows: u64,
    /// Every dynamic insert refused on this static edge with
    /// [`TfError::EdgeIsStatic`].
    pub static_refusals: u64,
    /// Every insert refused on this edge with
    /// [`TfError::StampBeforeWindow`]. A count that keeps rising is an edge
    /// whose publisher's clock went back without a new epoch.
    pub window_refusals: u64,
    /// Every insert refused on this edge with
    /// [`TfError::StampTooFarAhead`].
    pub jump_refusals: u64,
}

/// One static edge, read through [`TfBuffer::static_edges`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StaticEdge<'a> {
    /// The parent frame of the edge (`header.frame_id`).
    pub parent: &'a str,
    /// The child frame of the edge (`child_frame_id`).
    pub child: &'a str,
    /// `T_parent_child`, the last transform inserted for the edge.
    pub transform: Transform,
}

/// A buffer of stamped transforms, queried by `lookup(parent, child, time)`.
///
/// # The tree
///
/// Every inserted transform is an edge `T_parent_child`, the shape of
/// `geometry_msgs/TransformStamped`. The edges form a forest: a frame has at
/// most one parent, so a second parent for a child, a self edge or an edge
/// that would close a cycle is refused at insert and leaves the buffer as
/// it was. Frame ids are interned: each registered id is stored once in a
/// name table sized at construction and is named everywhere else by its
/// [`FrameId`]. Leading `/`s are removed first: `/odom` and `odom` name one
/// frame.
///
/// The buffer does not know which node published a transform: every
/// sample of an edge is one feed, whoever sent it. Two nodes that publish
/// the same edge interleave on its ring, so publish each edge from one
/// node.
///
/// # The rings
///
/// Each dynamic edge keeps the samples of the last
/// [`TfBufferConfig::retention`] before its newest stamp, sorted by stamp.
/// An insert expires every sample older than `newest - retention` (counted
/// in [`Self::expired`]), and refuses a stamp older than that with
/// [`TfError::StampBeforeWindow`]. A stamp more than the jump limit past
/// the newest sample (the larger of [`TfBufferConfig::jump_limit`] and the
/// retention) would let one bad stamp empty an edge and leave every later
/// good sample before its window; it is refused with
/// [`TfError::StampTooFarAhead`]. The next insert confirms the jump when its
/// stamp is later than the refused one and within the jump limit of it: two
/// inserts that agree on the new time are a clock that moved forward (a
/// publisher that paused), and the edge moves to it. The jump limit is apart
/// from the retention, so an edge that publishes less often than its window
/// keeps updating, each sample replacing the last, as long as each gap is
/// within the jump limit. An edge whose every gap is longer than the jump
/// limit never updates, since no later sample confirms a refused one; raise
/// [`TfBufferConfig::jump_limit`] to at least its longest gap. Both
/// refusals return their error every time and count on the edge
/// ([`EdgeInfo::window_refusals`], [`EdgeInfo::jump_refusals`]).
///
/// A sample at a stamp the ring already holds is a no-op when the
/// transform is the same and replaces it when it differs (counted in
/// [`Self::replacements`]). The ring itself has a fixed
/// capacity ([`TfBufferConfig::ring_capacity`]); when an edge publishes
/// faster than the ring was sized for, the ring fills before its window,
/// the oldest sample makes room, and every such loss counts in
/// [`EdgeInfo::overflows`].
///
/// A static edge ([`Self::insert_static`]) holds one transform that never
/// expires and wins at every time. A dynamic insert on it is refused with
/// [`TfError::EdgeIsStatic`] and counted in [`EdgeInfo::static_refusals`].
///
/// # Refusals
///
/// A refused insert returns its error every time: a sample the buffer did
/// not keep is never an `Ok(())`, so a caller never loses data silently.
/// The edge counts each kind of refusal as well ([`EdgeInfo::static_refusals`],
/// [`EdgeInfo::window_refusals`], [`EdgeInfo::jump_refusals`]), so a node
/// that would log every error on a 100 Hz edge can log the first and read
/// the running total from [`Self::edge`] instead.
///
/// # Epochs
///
/// A clock reset or a bag loop moves time backwards. [`Self::begin_epoch`]
/// clears every dynamic ring, keeps every frame, edge, policy and static
/// transform, and counts the epoch in [`Self::epoch`]; the new
/// timeline then fills the rings from empty. Without it, the first stamps
/// of the new timeline are older than the window and are refused.
///
/// # Lookup
///
/// [`Self::lookup`] walks from both frames up to their nearest common
/// ancestor and composes the edges it crosses, so the result is
/// `T_parent_child` whether `child` descends from `parent`, `parent`
/// descends from `child`, or the two share an ancestor. On each dynamic
/// edge the sample at the requested time is an exact hit or an
/// interpolation ([`Transform::interpolate`]) between the two samples that
/// bracket it. A time older than an edge's oldest sample is
/// [`TfError::OutOfRange`]. A time newer than its newest is the edge's
/// [`EdgePolicy`]: [`TfError::NotYetAvailable`] by default, or the newest
/// sample held, or the last motion continued, within the policy's bound.
/// Lookups are non-blocking: an error is a fact about the buffer now, and
/// the node retries on its next fire.
///
/// The same inputs give bit-identical results: the walk order is fixed by
/// the tree, the name index is an unseeded hash, and every operation is the
/// deterministic math of this crate, so lookups replay byte-exact.
#[derive(Debug)]
pub struct TfBuffer {
    /// The identity every [`FrameId`] this buffer issues carries.
    tag: u64,
    /// `frames.len()` is the capacity; the first `names` slots are in use.
    frames: Vec<Frame>,
    names: NameTable,
    ring_capacity: usize,
    retention_ns: u64,
    /// The larger of the jump limit and the retention, in nanoseconds.
    jump_limit_ns: u64,
    epoch: u64,
    expired: u64,
    replacements: u64,
}

impl Default for TfBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl TfBuffer {
    /// A buffer sized by [`TfBufferConfig::default`]: 64 frames, 10 s of
    /// history at up to 100 Hz per edge, and 4096 bytes of frame ids.
    pub fn new() -> Self {
        Self::with_config(TfBufferConfig::default())
    }

    /// A buffer sized by `config`.
    ///
    /// This is the one place the buffer allocates: the frame table, the
    /// name table and every ring are sized here, so `insert` and `lookup`
    /// never touch the heap.
    ///
    /// # Panics
    ///
    /// When `config.frames` exceeds `u32::MAX`, or when the process has
    /// built `u64::MAX` buffers and has no identity left to give.
    // hot-path-alloc-ok-fn: construction, once per node; it sizes the frame
    // table, the name table and every ring so that insert and lookup never
    // allocate.
    pub fn with_config(config: TfBufferConfig) -> Self {
        let frames = config.frames.get();
        assert!(
            u32::try_from(frames).is_ok(),
            "a tf buffer holds at most u32::MAX frames"
        );
        let ring_capacity = config.ring_capacity.get();
        let table = (0..frames)
            .map(|_| Frame {
                parent: None,
                is_static: false,
                static_transform: Transform::IDENTITY,
                policy: EdgePolicy::Interpolate,
                samples: VecDeque::with_capacity(ring_capacity),
                overflows: 0,
                static_refusals: 0,
                window_refusals: 0,
                jump_refusals: 0,
                pending_jump: None,
            })
            .collect();
        Self {
            tag: next_tag(),
            frames: table,
            names: NameTable::with_capacity(frames, config.name_bytes),
            ring_capacity,
            retention_ns: nanos(config.retention),
            jump_limit_ns: nanos(config.jump_limit.max(config.retention)),
            epoch: 0,
            expired: 0,
            replacements: 0,
        }
    }

    /// How many distinct frame ids the buffer can hold.
    pub fn frame_capacity(&self) -> usize {
        self.frames.len()
    }

    /// How many samples each dynamic edge's ring holds.
    pub fn ring_capacity(&self) -> usize {
        self.ring_capacity
    }

    /// How far back from its newest sample a dynamic edge keeps history.
    pub fn retention(&self) -> Duration {
        Duration::from_nanos(self.retention_ns)
    }

    /// How far past an edge's newest stamp a dynamic insert may land
    /// before it needs a second stamp to confirm it: the larger of
    /// [`TfBufferConfig::jump_limit`] and the retention.
    pub fn jump_limit(&self) -> Duration {
        Duration::from_nanos(self.jump_limit_ns)
    }

    /// The bytes the buffer holds for all frame ids together.
    pub fn name_budget(&self) -> usize {
        self.names.budget()
    }

    /// How many distinct frame ids the buffer has registered.
    pub fn frame_count(&self) -> usize {
        self.active()
    }

    /// How many times [`Self::begin_epoch`] has run.
    pub fn epoch(&self) -> u64 {
        self.epoch
    }

    /// Every sample retired in normal operation: aged out of its edge's
    /// retention window, cleared from an edge by a static insert, or
    /// cleared by [`Self::begin_epoch`].
    pub fn expired(&self) -> u64 {
        self.expired
    }

    /// Every stored transform overwritten by a different one at the same
    /// stamp (dynamic) or by a different static value (static). An insert
    /// of the value the edge already holds is a no-op and is not counted.
    pub fn replacements(&self) -> u64 {
        self.replacements
    }

    /// The handle of the registered frame `frame`; `None` when no inserted
    /// transform has named it.
    pub fn frame_id(&self, frame: &str) -> Option<FrameId> {
        self.names.find(normalize(frame)).map(|slot| self.id(slot))
    }

    /// The id of the frame behind `frame`; `None` for a handle this buffer
    /// never issued.
    pub fn frame_name(&self, frame: FrameId) -> Option<&str> {
        self.owns(frame).then(|| self.names.name(frame.index()))
    }

    /// Whether an inserted transform has named this frame.
    pub fn contains_frame(&self, frame: &str) -> bool {
        self.frame_id(frame).is_some()
    }

    /// `error` printed with this buffer's frame names in place of the
    /// `#n` handles of its plain text.
    pub fn describe<'a>(&'a self, error: &'a TfError) -> Describe<'a> {
        Describe {
            error,
            buffer: self,
        }
    }

    /// The edge from `child`'s parent to `child`; `None` when `child` is
    /// unknown or a root.
    pub fn edge(&self, child: &str) -> Option<EdgeInfo<'_>> {
        let c = self.names.find(normalize(child))?;
        let frame = &self.frames[c];
        let parent = frame.parent?;
        Some(EdgeInfo {
            parent: self.names.name(parent),
            is_static: frame.is_static,
            policy: frame.policy,
            samples: frame.samples.len(),
            oldest_ns: frame.samples.front().map(|s| s.stamp_ns),
            newest_ns: frame.samples.back().map(|s| s.stamp_ns),
            overflows: frame.overflows,
            static_refusals: frame.static_refusals,
            window_refusals: frame.window_refusals,
            jump_refusals: frame.jump_refusals,
        })
    }

    /// Every static edge, one per child frame, in the order the child
    /// frames were registered.
    ///
    /// Each edge holds the last transform [`Self::insert_static`] gave it:
    /// inserting a static transform for a child the buffer already holds
    /// replaces it. A node that publishes `/tf_static` keeps its static set
    /// in a buffer and writes every edge of this iterator into each
    /// `TFMessage` it sends, so its one latched message holds its whole set,
    /// merged by child frame. Reads only; never allocates.
    pub fn static_edges(&self) -> impl Iterator<Item = StaticEdge<'_>> + '_ {
        self.frames[..self.active()]
            .iter()
            .enumerate()
            .filter(|(_, frame)| frame.is_static)
            .filter_map(|(c, frame)| {
                let p = frame.parent?;
                Some(StaticEdge {
                    parent: self.names.name(p),
                    child: self.names.name(c),
                    transform: frame.static_transform,
                })
            })
    }

    /// Records `T_parent_child` at `stamp_ns` on the edge's ring.
    ///
    /// Frames the buffer has not seen are registered. The sample lands in
    /// stamp order; see the type docs for retention, the ring capacity and
    /// repeated stamps.
    ///
    /// # Errors
    ///
    /// [`TfError::EmptyFrameId`], [`TfError::SelfEdge`],
    /// [`TfError::SecondParent`], [`TfError::Cycle`],
    /// [`TfError::FrameTableFull`], [`TfError::NameBudgetFull`],
    /// [`TfError::EdgeIsStatic`], [`TfError::StampBeforeWindow`] and
    /// [`TfError::StampTooFarAhead`], each returned every time. A refusal
    /// changes nothing but the edge's refusal counters, and a
    /// [`TfError::StampTooFarAhead`] refusal also records its stamp for the
    /// next insert to confirm.
    pub fn insert(
        &mut self,
        parent: &str,
        child: &str,
        stamp_ns: u64,
        transform: Transform,
    ) -> Result<(), TfError> {
        let (p, c) = self.admit(parent, child, Admit::Dynamic)?;
        let (parent, child) = (self.id(p), self.id(c));
        let retention_ns = self.retention_ns;
        let jump_limit_ns = self.jump_limit_ns;
        let cap = self.ring_capacity;
        let frame = &mut self.frames[c];
        if let Some(newest_ns) = frame.samples.back().map(|s| s.stamp_ns) {
            if stamp_ns < newest_ns.saturating_sub(retention_ns) {
                frame.window_refusals += 1;
                return Err(TfError::StampBeforeWindow {
                    parent,
                    child,
                    stamp_ns,
                    newest_ns,
                });
            }
            if stamp_ns.saturating_sub(newest_ns) > jump_limit_ns {
                let confirmed = frame
                    .pending_jump
                    .is_some_and(|prev| prev < stamp_ns && stamp_ns - prev <= jump_limit_ns);
                if !confirmed {
                    frame.pending_jump = Some(stamp_ns);
                    frame.jump_refusals += 1;
                    return Err(TfError::StampTooFarAhead {
                        parent,
                        child,
                        stamp_ns,
                        newest_ns,
                        jump_limit_ns,
                    });
                }
            }
        }
        // Every check passed: the writes start here.
        frame.pending_jump = None;
        let ring = &mut frame.samples;
        let mut i = match ring.binary_search_by_key(&stamp_ns, |s| s.stamp_ns) {
            Ok(i) => {
                if ring[i].transform != transform {
                    ring[i].transform = transform;
                    self.replacements += 1;
                }
                return Ok(());
            }
            Err(i) => i,
        };
        // The window ends at the newer of the ring's newest stamp and this
        // one, and this stamp is never before its start, so a sample that
        // expires sits before `i` or is the whole ring when the stamp moves
        // the window past every sample it holds.
        let newest = ring.back().map_or(stamp_ns, |s| s.stamp_ns.max(stamp_ns));
        let cutoff = newest.saturating_sub(retention_ns);
        while ring.front().is_some_and(|s| s.stamp_ns < cutoff) {
            ring.pop_front();
            self.expired += 1;
            i -= 1;
        }
        let sample = Sample {
            stamp_ns,
            transform,
        };
        if ring.len() < cap {
            ring.insert(i, sample);
        } else if i == 0 {
            // Older than everything on a full ring: it would be the first
            // to make room, so it is the sample lost.
            frame.overflows += 1;
        } else {
            // The oldest sample makes room.
            ring.pop_front();
            ring.insert(i - 1, sample);
            frame.overflows += 1;
        }
        Ok(())
    }

    /// Records `T_parent_child` as a static transform: it holds at every
    /// time and never expires. Re-inserting a static edge
    /// sets its value; a static insert on an edge with dynamic history
    /// clears that history, since the static transform wins at every time
    /// anyway, and counts every cleared sample in [`Self::expired`].
    ///
    /// # Errors
    ///
    /// [`TfError::EmptyFrameId`], [`TfError::SelfEdge`],
    /// [`TfError::SecondParent`], [`TfError::Cycle`],
    /// [`TfError::FrameTableFull`] and [`TfError::NameBudgetFull`]. A
    /// refusal changes nothing.
    pub fn insert_static(
        &mut self,
        parent: &str,
        child: &str,
        transform: Transform,
    ) -> Result<(), TfError> {
        let (_, c) = self.admit(parent, child, Admit::Static)?;
        let frame = &mut self.frames[c];
        if frame.is_static && frame.static_transform != transform {
            self.replacements += 1;
        }
        self.expired += frame.samples.len() as u64;
        frame.samples.clear();
        frame.pending_jump = None;
        frame.is_static = true;
        frame.static_transform = transform;
        Ok(())
    }

    /// Sets what the dynamic edge `parent` to `child` answers past its
    /// newest sample. The edge need not have a sample yet: an unknown edge
    /// is declared, registering its frames and its parent link with an
    /// empty ring, so a node can set its policies before the first
    /// transform arrives.
    ///
    /// # Errors
    ///
    /// [`TfError::EmptyFrameId`], [`TfError::SelfEdge`],
    /// [`TfError::SecondParent`], [`TfError::Cycle`],
    /// [`TfError::FrameTableFull`], [`TfError::NameBudgetFull`], and
    /// [`TfError::EdgeIsStatic`] on a static edge. A refusal changes
    /// nothing.
    pub fn set_edge_policy(
        &mut self,
        parent: &str,
        child: &str,
        policy: EdgePolicy,
    ) -> Result<(), TfError> {
        let (_, c) = self.admit(parent, child, Admit::Policy)?;
        self.frames[c].policy = policy;
        Ok(())
    }

    /// Starts a new timeline after a clock reset or a bag loop: clears
    /// every dynamic ring and every unconfirmed forward jump (counting the
    /// cleared samples in [`Self::expired`]) and increments
    /// [`Self::epoch`]. Frames, edges, policies and static transforms are
    /// kept. The runtime that resets the clock calls it at the reset, so a
    /// replay calls it at the same point.
    pub fn begin_epoch(&mut self) {
        let active = self.active();
        for frame in &mut self.frames[..active] {
            self.expired += frame.samples.len() as u64;
            frame.samples.clear();
            frame.pending_jump = None;
        }
        self.epoch += 1;
    }

    /// `T_parent_child` at `stamp_ns`, interpolated along the chain between
    /// the two frames. See the type docs for the rules.
    ///
    /// # Errors
    ///
    /// [`TfError::EmptyFrameId`], [`TfError::UnknownFrame`],
    /// [`TfError::NoPath`], [`TfError::EdgeEmpty`],
    /// [`TfError::NotYetAvailable`], [`TfError::OutOfRange`], and
    /// [`TfError::ChainMath`] when composing the chain overflows.
    pub fn lookup(&self, parent: &str, child: &str, stamp_ns: u64) -> Result<Transform, TfError> {
        let (p, c) = (self.slot_of(parent)?, self.slot_of(child)?);
        self.lookup_slots(p, c, Some(stamp_ns))
    }

    /// [`Self::lookup`] by handle, with no string work: resolve the frame
    /// ids once with [`Self::frame_id`] and look up by handle on every
    /// tick.
    ///
    /// # Errors
    ///
    /// The same as [`Self::lookup`], with [`TfError::UnknownFrameId`] for a
    /// handle this buffer never issued.
    pub fn lookup_ids(
        &self,
        parent: FrameId,
        child: FrameId,
        stamp_ns: u64,
    ) -> Result<Transform, TfError> {
        let (p, c) = (self.slot_of_id(parent)?, self.slot_of_id(child)?);
        self.lookup_slots(p, c, Some(stamp_ns))
    }

    /// `T_parent_child` at the latest time the chain answers, through the
    /// same code path as [`Self::lookup`]. Each dynamic edge reaches its
    /// newest stamp, plus the bound of its [`EdgePolicy`] past it (an
    /// extrapolating edge with one sample reaches only that sample); the
    /// chain resolves at the earliest of those reaches, and never later
    /// than the newest stamp on the chain. With the default policy on every
    /// edge that is the oldest of the newest stamps, so every edge has a
    /// sample there; a held slow edge does not hold a fast chain back. A
    /// chain of static edges only has no time and resolves directly.
    ///
    /// # Errors
    ///
    /// The same as [`Self::lookup`]; [`TfError::OutOfRange`] when the
    /// chain's edges hold no common time yet.
    pub fn lookup_latest(&self, parent: &str, child: &str) -> Result<Transform, TfError> {
        let (p, c) = (self.slot_of(parent)?, self.slot_of(child)?);
        self.lookup_slots(p, c, None)
    }

    /// [`Self::lookup_latest`] by handle.
    ///
    /// # Errors
    ///
    /// The same as [`Self::lookup_latest`], with
    /// [`TfError::UnknownFrameId`] for a handle this buffer never issued.
    pub fn lookup_latest_ids(&self, parent: FrameId, child: FrameId) -> Result<Transform, TfError> {
        let (p, c) = (self.slot_of_id(parent)?, self.slot_of_id(child)?);
        self.lookup_slots(p, c, None)
    }

    // ----------------------------------------------------------- internals

    fn active(&self) -> usize {
        self.names.len()
    }

    /// The slot of a lookup frame, or why it has none.
    fn slot_of(&self, frame: &str) -> Result<usize, TfError> {
        let name = normalize(frame);
        if name.is_empty() {
            return Err(TfError::EmptyFrameId);
        }
        self.names.find(name).ok_or(TfError::UnknownFrame {
            frame: FrameLabel::of(name),
        })
    }

    fn slot_of_id(&self, frame: FrameId) -> Result<usize, TfError> {
        if self.owns(frame) {
            Ok(frame.index())
        } else {
            Err(TfError::UnknownFrameId { frame })
        }
    }

    /// Whether this buffer issued `frame`: it carries this buffer's
    /// identity and names a registered slot.
    fn owns(&self, frame: FrameId) -> bool {
        frame.buffer() == self.tag && frame.index() < self.active()
    }

    fn id(&self, slot: usize) -> FrameId {
        FrameId::new(self.tag, slot)
    }

    /// Whether walking up from `from` reaches `target`.
    fn descends_from(&self, from: usize, target: usize) -> bool {
        let mut cur = from;
        // A frame has one parent and the tree holds no cycle, so the walk
        // ends within `active` steps; the bound only guards the loop.
        for _ in 0..=self.active() {
            if cur == target {
                return true;
            }
            match self.frames[cur].parent {
                Some(next) => cur = next,
                None => return false,
            }
        }
        false
    }

    /// Validates an edge write in full, then registers any new frame and
    /// the parent link. Returns the `(parent, child)` slots. Every refusal
    /// happens before the first write to the tree.
    fn admit(&mut self, parent: &str, child: &str, kind: Admit) -> Result<(usize, usize), TfError> {
        let parent_name = normalize(parent);
        let child_name = normalize(child);
        if parent_name.is_empty() || child_name.is_empty() {
            return Err(TfError::EmptyFrameId);
        }
        if parent_name == child_name {
            return Err(TfError::SelfEdge {
                frame: FrameLabel::of(child_name),
            });
        }
        let p = self.names.find(parent_name);
        let c = self.names.find(child_name);
        if let Some(c) = c {
            let frame = &self.frames[c];
            match frame.parent {
                Some(existing) if Some(existing) != p => {
                    return Err(TfError::SecondParent {
                        child: self.id(c),
                        parent: FrameLabel::of(parent_name),
                        existing_parent: self.id(existing),
                    });
                }
                Some(_) => {}
                None => {
                    if let Some(p) = p {
                        if self.descends_from(p, c) {
                            return Err(TfError::Cycle {
                                parent: self.id(p),
                                child: self.id(c),
                            });
                        }
                    }
                }
            }
            if let Some(p) = p {
                if let Some(refusal) = self.refusal(p, c, kind) {
                    return Err(refusal);
                }
            }
        }
        // The parent registers first, so the refusal names the first frame
        // that finds no slot: the parent when none is left, the child when
        // the parent takes the last one.
        let after_parent = self.active() + usize::from(p.is_none());
        let after_child = after_parent + usize::from(c.is_none());
        if after_child > self.frames.len() {
            let frame = if after_parent > self.frames.len() {
                parent_name
            } else {
                child_name
            };
            return Err(TfError::FrameTableFull {
                frame: FrameLabel::of(frame),
                capacity: self.frames.len(),
            });
        }
        // The parent's id is interned first, so the child's must fit in
        // what the parent leaves.
        let parent_bytes = if p.is_none() { parent_name.len() } else { 0 };
        let child_bytes = if c.is_none() { child_name.len() } else { 0 };
        let spare = self.names.spare();
        let unfit = if parent_bytes > spare {
            Some(parent_name)
        } else if parent_bytes + child_bytes > spare {
            Some(child_name)
        } else {
            None
        };
        if let Some(frame) = unfit {
            // The bytes left for this frame: for a new child, what the
            // new parent leaves.
            let left = if frame == parent_name {
                spare
            } else {
                spare.saturating_sub(parent_bytes)
            };
            return Err(TfError::NameBudgetFull {
                frame: FrameLabel::of(frame),
                len: frame.len(),
                left,
                budget: self.names.budget(),
            });
        }
        // Every check passed: the writes start here.
        let p = p.unwrap_or_else(|| self.register(parent_name));
        let c = c.unwrap_or_else(|| self.register(child_name));
        self.frames[c].parent = Some(p);
        Ok((p, c))
    }

    /// The static check of an existing edge `p` to `c`: the refusal, or
    /// `None` when the write may go ahead.
    fn refusal(&mut self, p: usize, c: usize, kind: Admit) -> Option<TfError> {
        let (parent, child) = (self.id(p), self.id(c));
        let frame = &mut self.frames[c];
        match kind {
            // A policy has no place on a static edge; setting one is a
            // configuration call, not a dropped sample, so it is not
            // counted.
            Admit::Policy if frame.is_static => Some(TfError::EdgeIsStatic { parent, child }),
            Admit::Dynamic if frame.is_static => {
                frame.static_refusals += 1;
                Some(TfError::EdgeIsStatic { parent, child })
            }
            Admit::Policy | Admit::Dynamic | Admit::Static => None,
        }
    }

    /// Takes the next free slot for a new frame.
    fn register(&mut self, name: &str) -> usize {
        let i = self.names.register(name);
        let frame = &mut self.frames[i];
        frame.parent = None;
        frame.is_static = false;
        frame.static_transform = Transform::IDENTITY;
        frame.policy = EdgePolicy::Interpolate;
        frame.samples.clear();
        frame.pending_jump = None;
        i
    }

    /// The lookup behind every public form: at `stamp_ns`, or at the
    /// chain's latest time when it is `None`.
    fn lookup_slots(
        &self,
        p: usize,
        c: usize,
        stamp_ns: Option<u64>,
    ) -> Result<Transform, TfError> {
        let anc = self.common_ancestor(p, c).ok_or(TfError::NoPath {
            parent: self.id(p),
            child: self.id(c),
        })?;
        let stamp_ns = match stamp_ns {
            Some(stamp_ns) => stamp_ns,
            None => {
                let mut latest = Latest::default();
                self.chain_latest(c, anc, &mut latest);
                self.chain_latest(p, anc, &mut latest);
                latest.stamp_ns()
            }
        };
        self.resolve(p, c, anc, stamp_ns)
    }

    fn depth(&self, mut i: usize) -> usize {
        let mut d = 0;
        while let Some(next) = self.frames[i].parent {
            i = next;
            d += 1;
        }
        d
    }

    fn up(&self, mut i: usize, steps: usize) -> usize {
        for _ in 0..steps {
            if let Some(next) = self.frames[i].parent {
                i = next;
            }
        }
        i
    }

    /// The nearest frame both `a` and `b` descend from (either one itself
    /// when it is an ancestor of the other); `None` in separate trees.
    fn common_ancestor(&self, a: usize, b: usize) -> Option<usize> {
        let (da, db) = (self.depth(a), self.depth(b));
        let mut a = self.up(a, da.saturating_sub(db));
        let mut b = self.up(b, db.saturating_sub(da));
        loop {
            if a == b {
                return Some(a);
            }
            match (self.frames[a].parent, self.frames[b].parent) {
                (Some(pa), Some(pb)) => {
                    a = pa;
                    b = pb;
                }
                _ => return None,
            }
        }
    }

    /// `T_parent_child` from the frames `p` and `c` through their common
    /// ancestor `anc`, at `stamp_ns`. A translation overflow along the chain
    /// comes back as [`TfError::ChainMath`], naming the two frames.
    fn resolve(&self, p: usize, c: usize, anc: usize, stamp_ns: u64) -> Result<Transform, TfError> {
        self.compose_chain(p, c, anc, stamp_ns)
            .map_err(|err| match err {
                TfError::NonFiniteTranslation => TfError::ChainMath {
                    parent: self.id(p),
                    child: self.id(c),
                },
                other => other,
            })
    }

    /// The composition behind [`Self::resolve`].
    fn compose_chain(
        &self,
        p: usize,
        c: usize,
        anc: usize,
        stamp_ns: u64,
    ) -> Result<Transform, TfError> {
        // A single-edge chain returns the sample itself (or its exact
        // inverse), never a product with the identity: a product
        // renormalizes and could move the bits.
        match (p == anc, c == anc) {
            (true, true) => Ok(Transform::IDENTITY),
            (true, false) => self.chain(c, anc, stamp_ns),
            (false, true) => Ok(self.chain(p, anc, stamp_ns)?.inverse()?),
            (false, false) => {
                let anc_parent = self.chain(p, anc, stamp_ns)?;
                let anc_child = self.chain(c, anc, stamp_ns)?;
                Ok(anc_parent.inverse()?.compose(&anc_child)?)
            }
        }
    }

    /// `T_anc_from`: the edges from `from` up to (not including) `anc`,
    /// composed in a fixed order. `from` must descend from `anc` and differ
    /// from it.
    fn chain(&self, from: usize, anc: usize, stamp_ns: u64) -> Result<Transform, TfError> {
        let mut acc = self.edge_at(from, stamp_ns)?;
        let mut cur = self.frames[from].parent;
        while let Some(i) = cur {
            if i == anc {
                break;
            }
            acc = self.edge_at(i, stamp_ns)?.compose(&acc)?;
            cur = self.frames[i].parent;
        }
        Ok(acc)
    }

    /// Folds the newest stamps of the dynamic edges on the chain from
    /// `from` up to `anc` into `latest`.
    fn chain_latest(&self, from: usize, anc: usize, latest: &mut Latest) {
        let mut cur = from;
        while cur != anc {
            let frame = &self.frames[cur];
            if let (false, Some(last)) = (frame.is_static, frame.samples.back()) {
                latest.fold(last.stamp_ns, frame.reach_ns());
            }
            match frame.parent {
                Some(next) => cur = next,
                None => break,
            }
        }
    }

    /// `T_parent_child` for the edge into `child` at `stamp_ns`: the static
    /// value, an exact hit, the interpolation between the bracketing
    /// samples, or the edge's policy past its newest sample.
    fn edge_at(&self, child: usize, stamp_ns: u64) -> Result<Transform, TfError> {
        let frame = &self.frames[child];
        if frame.is_static {
            return Ok(frame.static_transform);
        }
        // Every frame on a chain below the common ancestor has a parent.
        let parent = self.id(frame.parent.unwrap_or(child));
        let child_id = self.id(child);
        let ring = &frame.samples;
        let (Some(first), Some(last)) = (ring.front(), ring.back()) else {
            return Err(TfError::EdgeEmpty {
                parent,
                child: child_id,
            });
        };
        if stamp_ns < first.stamp_ns {
            return Err(TfError::OutOfRange {
                parent,
                child: child_id,
                requested_ns: stamp_ns,
                oldest_ns: first.stamp_ns,
                newest_ns: last.stamp_ns,
            });
        }
        if stamp_ns > last.stamp_ns {
            let ahead_ns = stamp_ns - last.stamp_ns;
            let not_yet = TfError::NotYetAvailable {
                parent,
                child: child_id,
                requested_ns: stamp_ns,
                newest_ns: last.stamp_ns,
            };
            if ahead_ns > frame.reach_ns() {
                return Err(not_yet);
            }
            return match frame.policy {
                EdgePolicy::Interpolate => Err(not_yet),
                EdgePolicy::HoldLast { .. } => Ok(last.transform),
                EdgePolicy::Extrapolate { .. } => {
                    // A nonzero reach means the ring holds two samples.
                    let before = &ring[ring.len() - 2];
                    extrapolate(
                        &before.transform,
                        &last.transform,
                        last.stamp_ns - before.stamp_ns,
                        ahead_ns,
                    )
                }
            };
        }
        let i = ring.partition_point(|s| s.stamp_ns < stamp_ns);
        let after = &ring[i];
        if after.stamp_ns == stamp_ns {
            return Ok(after.transform);
        }
        let before = &ring[i - 1];
        // The differences are exact in u64 and convert to f64 exactly below
        // 2^53 ns (about 104 days); a wider gap rounds, the same way on
        // every run. The stamps themselves (nanoseconds since an epoch) are
        // too large to convert exactly, so the fraction is never formed
        // from them directly.
        let span = (after.stamp_ns - before.stamp_ns) as f64;
        let into = (stamp_ns - before.stamp_ns) as f64;
        before.transform.interpolate(&after.transform, into / span)
    }
}

/// The identity the next buffer takes.
static NEXT_TAG: AtomicU64 = AtomicU64::new(0);

/// A buffer identity no other buffer in this process has taken.
///
/// # Panics
///
/// When `u64::MAX` identities are taken: the counter never wraps, so a
/// stale [`FrameId`] can never match a later buffer.
fn next_tag() -> u64 {
    let mut tag = NEXT_TAG.load(Ordering::Relaxed);
    loop {
        let next = tag
            .checked_add(1)
            .expect("a process builds fewer than u64::MAX tf buffers");
        match NEXT_TAG.compare_exchange_weak(tag, next, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => return tag,
            Err(now) => tag = now,
        }
    }
}

/// `duration` in nanoseconds, `u64::MAX` when it is longer.
fn nanos(duration: Duration) -> u64 {
    u64::try_from(duration.as_nanos()).unwrap_or(u64::MAX)
}

/// What an edge write is, for the checks in [`TfBuffer::admit`].
#[derive(Debug, Clone, Copy)]
enum Admit {
    Dynamic,
    Static,
    Policy,
}

/// The latest time a chain answers, folded edge by edge: the earliest of
/// what each dynamic edge reaches (its newest stamp, plus how far past it
/// the edge answers), never later than the newest stamp on the chain.
#[derive(Debug, Default)]
struct Latest {
    /// The earliest reach of the dynamic edges.
    reach: Option<u64>,
    /// The newest stamp of the dynamic edges.
    newest: Option<u64>,
}

impl Latest {
    fn fold(&mut self, newest_ns: u64, reach_ns: u64) {
        let reach = newest_ns.saturating_add(reach_ns);
        self.reach = Some(self.reach.map_or(reach, |r| r.min(reach)));
        self.newest = Some(self.newest.map_or(newest_ns, |n| n.max(newest_ns)));
    }

    fn stamp_ns(&self) -> u64 {
        match (self.reach, self.newest) {
            (Some(reach), Some(newest)) => reach.min(newest),
            // Static edges only: every time is as good as any other.
            _ => 0,
        }
    }
}
