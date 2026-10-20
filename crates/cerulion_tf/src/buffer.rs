// SPDX-License-Identifier: AGPL-3.0-only
//! The tf buffer: a frame tree with a bounded time ring per edge, and the
//! `lookup` that interpolates along the chain between two frames.
//!
//! See [`TfBuffer`] for the rules. Nothing here allocates after
//! construction: the frame table and every ring are sized by
//! [`TfBuffer::with_capacity`], so inserts and lookups run inside a node
//! tick without touching the heap.

use std::collections::VecDeque;
use std::num::NonZeroUsize;

use crate::{FrameName, TfError, Transform};

/// The frame capacity of [`TfBuffer::new`]: how many distinct frame ids the
/// buffer holds.
pub const DEFAULT_FRAME_CAPACITY: usize = 64;

/// The ring capacity of [`TfBuffer::new`]: how many stamped samples each
/// dynamic edge keeps before the oldest is evicted.
pub const DEFAULT_SAMPLES_PER_EDGE: usize = 64;

const DEFAULT_FRAMES: NonZeroUsize = NonZeroUsize::new(DEFAULT_FRAME_CAPACITY).unwrap();
const DEFAULT_SAMPLES: NonZeroUsize = NonZeroUsize::new(DEFAULT_SAMPLES_PER_EDGE).unwrap();

/// One stamped transform `T_parent_child` on an edge's ring.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Sample {
    stamp_ns: u64,
    transform: Transform,
}

/// A frame and, when it has a parent, the edge from that parent to it.
#[derive(Debug)]
struct Frame {
    name: FrameName,
    /// Index of the parent frame; `None` for a root.
    parent: Option<usize>,
    /// The edge came in through `insert_static`: `static_transform` holds
    /// at every time and `samples` is empty.
    is_static: bool,
    static_transform: Transform,
    /// Dynamic samples on a ring, sorted by stamp, at most
    /// `samples_per_edge` long.
    samples: VecDeque<Sample>,
}

/// What the buffer holds for one edge, read through [`TfBuffer::edge`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeInfo<'a> {
    /// The parent frame of the edge.
    pub parent: &'a str,
    /// The edge came in through [`TfBuffer::insert_static`].
    pub is_static: bool,
    /// Samples on the ring (always 0 for a static edge).
    pub samples: usize,
    /// The oldest buffered stamp, in nanoseconds; `None` when the ring is
    /// empty.
    pub oldest_ns: Option<u64>,
    /// The newest buffered stamp, in nanoseconds; `None` when the ring is
    /// empty.
    pub newest_ns: Option<u64>,
}

/// A buffer of stamped transforms, queried by `lookup(parent, child, time)`.
///
/// # The tree
///
/// Every inserted transform is an edge `T_parent_child`, the shape of
/// `geometry_msgs/TransformStamped`. The edges form a forest: a frame has at
/// most one parent, so a second parent for a child, a self edge or an edge
/// that would close a cycle is refused at insert and leaves the buffer as
/// it was. Frame ids are at most [`crate::MAX_FRAME_ID_LEN`] bytes after
/// their leading `/`s are removed: `/odom` and `odom` name one frame.
///
/// # The rings
///
/// Each dynamic edge keeps its samples on a ring of fixed capacity, sorted
/// by stamp; a sample that arrives in stamp order costs one slot, and one
/// that arrives out of order shifts the shorter side of the ring. A sample
/// at a stamp the ring already holds replaces it (tf2 ignores the repeat;
/// here a corrected re-send wins). On a full ring the oldest sample is
/// evicted to make room, and a sample older than everything on a full ring
/// is dropped on arrival; both count in [`Self::evictions`],
/// unconditionally, and so does every dynamic sample a static insert
/// clears. A static edge ([`Self::insert_static`]) holds one transform that
/// never expires and wins at every time; a dynamic transform for a static
/// edge is refused.
///
/// # Lookup
///
/// [`Self::lookup`] walks from both frames up to their nearest common
/// ancestor and composes the edges it crosses, so the result is
/// `T_parent_child` whether `child` descends from `parent`, `parent`
/// descends from `child`, or the two share an ancestor. On each dynamic
/// edge the sample at the requested time is an exact hit or an
/// interpolation ([`Transform::interpolate`]) between the two samples that
/// bracket it. Lookups never extrapolate: a time newer than an edge's
/// newest sample is [`TfError::NotYetAvailable`], a time older than its
/// oldest is [`TfError::OutOfRange`], and both name the edge. An unknown
/// frame is [`TfError::UnknownFrame`]; two frames in separate trees are
/// [`TfError::NoPath`]. Lookups are non-blocking: an error is a fact about
/// the buffer now, and the node retries on its next fire.
///
/// The same inputs give bit-identical results: the walk order is fixed by
/// the tree and every operation is the deterministic math of this crate, so
/// lookups replay byte-exact.
#[derive(Debug)]
pub struct TfBuffer {
    /// `frames.len()` is the capacity; the first `active` slots are in use.
    frames: Vec<Frame>,
    active: usize,
    samples_per_edge: usize,
    evictions: u64,
}

impl Default for TfBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl TfBuffer {
    /// A buffer for [`DEFAULT_FRAME_CAPACITY`] frames with
    /// [`DEFAULT_SAMPLES_PER_EDGE`] samples per edge.
    pub fn new() -> Self {
        Self::with_capacity(DEFAULT_FRAMES, DEFAULT_SAMPLES)
    }

    /// A buffer for `frames` distinct frame ids with `samples_per_edge`
    /// samples on each dynamic edge's ring.
    ///
    /// This is the one place the buffer allocates: the frame table and
    /// every ring are sized here, so `insert` and `lookup` never touch the
    /// heap.
    // hot-path-alloc-ok-fn: construction, once per node; it sizes the frame
    // table and every ring so that insert and lookup never allocate.
    pub fn with_capacity(frames: NonZeroUsize, samples_per_edge: NonZeroUsize) -> Self {
        let frames = (0..frames.get())
            .map(|_| Frame {
                name: FrameName::EMPTY,
                parent: None,
                is_static: false,
                static_transform: Transform::IDENTITY,
                samples: VecDeque::with_capacity(samples_per_edge.get()),
            })
            .collect();
        Self {
            frames,
            active: 0,
            samples_per_edge: samples_per_edge.get(),
            evictions: 0,
        }
    }

    /// How many distinct frame ids the buffer can hold.
    pub fn frame_capacity(&self) -> usize {
        self.frames.len()
    }

    /// How many samples each dynamic edge's ring holds.
    pub fn samples_per_edge(&self) -> usize {
        self.samples_per_edge
    }

    /// How many distinct frame ids the buffer has seen.
    pub fn frame_count(&self) -> usize {
        self.active
    }

    /// Every sample dropped so far: the oldest sample of a full ring making
    /// room for a newer one, an arriving sample older than everything on a
    /// full ring, and each dynamic sample a static insert clears from its
    /// edge. Increments on every drop.
    pub fn evictions(&self) -> u64 {
        self.evictions
    }

    /// Whether an inserted transform has named this frame. An id the buffer
    /// could never hold (empty, or over the length limit) is absent.
    pub fn contains_frame(&self, frame: &str) -> bool {
        frame
            .parse::<FrameName>()
            .ok()
            .and_then(|name| self.index_of(&name))
            .is_some()
    }

    /// The edge from `child`'s parent to `child`; `None` when `child` is
    /// unknown or a root.
    pub fn edge(&self, child: &str) -> Option<EdgeInfo<'_>> {
        let child: FrameName = child.parse().ok()?;
        let frame = &self.frames[self.index_of(&child)?];
        let parent = frame.parent?;
        Some(EdgeInfo {
            parent: self.frames[parent].name.as_str(),
            is_static: frame.is_static,
            samples: frame.samples.len(),
            oldest_ns: frame.samples.front().map(|s| s.stamp_ns),
            newest_ns: frame.samples.back().map(|s| s.stamp_ns),
        })
    }

    /// Records `T_parent_child` at `stamp_ns` on the edge's ring.
    ///
    /// Frames the buffer has not seen are registered. The sample lands in
    /// stamp order; see the type docs for what a full ring does. A sample
    /// older than everything on a full ring is dropped on arrival and the
    /// call still returns `Ok(())`: the drop shows in [`Self::evictions`],
    /// which a caller that needs to know compares before and after.
    ///
    /// # Errors
    ///
    /// [`TfError::EmptyFrameId`], [`TfError::FrameIdTooLong`],
    /// [`TfError::SelfEdge`], [`TfError::SecondParent`], [`TfError::Cycle`],
    /// [`TfError::EdgeIsStatic`] and [`TfError::FrameTableFull`]. A refusal
    /// changes nothing.
    pub fn insert(
        &mut self,
        parent: &str,
        child: &str,
        stamp_ns: u64,
        transform: Transform,
    ) -> Result<(), TfError> {
        let (_, child) = self.admit(parent, child, false)?;
        let cap = self.samples_per_edge;
        let ring = &mut self.frames[child].samples;
        match ring.binary_search_by_key(&stamp_ns, |s| s.stamp_ns) {
            Ok(i) => ring[i].transform = transform,
            Err(i) => {
                let sample = Sample {
                    stamp_ns,
                    transform,
                };
                if ring.len() < cap {
                    ring.insert(i, sample);
                } else if i == 0 {
                    // Older than everything on a full ring: it would be the
                    // first evicted, so it is dropped on arrival.
                    self.evictions += 1;
                } else {
                    // The oldest sample makes room. In stamp order `i` is
                    // the end of the ring, so this is a pop and a push.
                    ring.pop_front();
                    ring.insert(i - 1, sample);
                    self.evictions += 1;
                }
            }
        }
        Ok(())
    }

    /// Records `T_parent_child` as a static transform: it holds at every
    /// time and never expires. Re-inserting a static edge replaces its
    /// value; a static insert on an edge with dynamic history clears that
    /// history, since the static transform wins at every time anyway, and
    /// counts every cleared sample in [`Self::evictions`].
    ///
    /// # Errors
    ///
    /// [`TfError::EmptyFrameId`], [`TfError::FrameIdTooLong`],
    /// [`TfError::SelfEdge`], [`TfError::SecondParent`], [`TfError::Cycle`]
    /// and [`TfError::FrameTableFull`]. A refusal changes nothing.
    pub fn insert_static(
        &mut self,
        parent: &str,
        child: &str,
        transform: Transform,
    ) -> Result<(), TfError> {
        let (_, child) = self.admit(parent, child, true)?;
        let frame = &mut self.frames[child];
        self.evictions += frame.samples.len() as u64;
        frame.samples.clear();
        frame.is_static = true;
        frame.static_transform = transform;
        Ok(())
    }

    /// `T_parent_child` at `stamp_ns`, interpolated along the chain between
    /// the two frames. See the type docs for the rules.
    ///
    /// # Errors
    ///
    /// [`TfError::EmptyFrameId`] and [`TfError::FrameIdTooLong`] for an id
    /// the buffer could never hold; [`TfError::UnknownFrame`],
    /// [`TfError::NoPath`], [`TfError::NotYetAvailable`],
    /// [`TfError::OutOfRange`], and [`TfError::ChainMath`] when composing
    /// the chain overflows.
    pub fn lookup(&self, parent: &str, child: &str, stamp_ns: u64) -> Result<Transform, TfError> {
        let (p, c, anc) = self.chain_ends(parent, child)?;
        self.resolve(p, c, anc, stamp_ns)
    }

    /// `T_parent_child` at the newest time every dynamic edge on the chain
    /// has a sample for, through the same code path as [`Self::lookup`].
    /// A chain of static edges only has no time and resolves directly.
    ///
    /// # Errors
    ///
    /// The same as [`Self::lookup`]; [`TfError::OutOfRange`] when the
    /// chain's edges hold no common time yet.
    pub fn lookup_latest(&self, parent: &str, child: &str) -> Result<Transform, TfError> {
        let (p, c, anc) = self.chain_ends(parent, child)?;
        let newest = match (self.chain_newest(c, anc), self.chain_newest(p, anc)) {
            (Some(a), Some(b)) => a.min(b),
            (Some(a), None) | (None, Some(a)) => a,
            // Static edges only: every time is as good as any other.
            (None, None) => 0,
        };
        self.resolve(p, c, anc, newest)
    }

    // ----------------------------------------------------------- internals

    fn index_of(&self, name: &FrameName) -> Option<usize> {
        self.frames[..self.active]
            .iter()
            .position(|f| f.name == *name)
    }

    /// The slot of a lookup frame, or why it has none: the id's own refusal
    /// for one the buffer could never hold, `UnknownFrame` otherwise.
    fn lookup_slot(&self, id: &str) -> Result<usize, TfError> {
        let name: FrameName = id.parse()?;
        self.index_of(&name)
            .ok_or(TfError::UnknownFrame { frame: name })
    }

    /// Whether walking up from `from` reaches `target`.
    fn descends_from(&self, from: usize, target: usize) -> bool {
        let mut cur = from;
        // A frame has one parent and the tree holds no cycle, so the walk
        // ends within `active` steps; the bound only guards the loop.
        for _ in 0..=self.active {
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

    /// Validates an edge insert in full, then registers any new frame and
    /// the parent link. Returns `(parent, child)` indices. Every refusal
    /// happens before the first write.
    fn admit(
        &mut self,
        parent: &str,
        child: &str,
        is_static: bool,
    ) -> Result<(usize, usize), TfError> {
        let parent_name: FrameName = parent.parse()?;
        let child_name: FrameName = child.parse()?;
        if parent_name == child_name {
            return Err(TfError::SelfEdge { frame: child_name });
        }
        let p = self.index_of(&parent_name);
        let c = self.index_of(&child_name);
        if let Some(c) = c {
            let frame = &self.frames[c];
            match frame.parent {
                Some(existing) if Some(existing) != p => {
                    return Err(TfError::SecondParent {
                        child: child_name,
                        parent: parent_name,
                        existing_parent: self.frames[existing].name,
                    });
                }
                Some(_) => {}
                None => {
                    if let Some(p) = p {
                        if self.descends_from(p, c) {
                            return Err(TfError::Cycle {
                                parent: parent_name,
                                child: child_name,
                            });
                        }
                    }
                }
            }
            if frame.is_static && !is_static {
                return Err(TfError::EdgeIsStatic {
                    parent: parent_name,
                    child: child_name,
                });
            }
        }
        let needed = usize::from(p.is_none()) + usize::from(c.is_none());
        if self.active + needed > self.frames.len() {
            return Err(TfError::FrameTableFull {
                frame: if p.is_none() { parent_name } else { child_name },
                capacity: self.frames.len(),
            });
        }
        // Every check passed: the writes start here.
        let p = match p {
            Some(p) => p,
            None => self.register(parent_name),
        };
        let c = match c {
            Some(c) => c,
            None => self.register(child_name),
        };
        self.frames[c].parent = Some(p);
        Ok((p, c))
    }

    /// Takes the next free slot for a new frame.
    fn register(&mut self, name: FrameName) -> usize {
        let i = self.active;
        let frame = &mut self.frames[i];
        frame.name = name;
        frame.parent = None;
        frame.is_static = false;
        frame.static_transform = Transform::IDENTITY;
        frame.samples.clear();
        self.active += 1;
        i
    }

    /// Resolves both frames and their nearest common ancestor.
    fn chain_ends(&self, parent: &str, child: &str) -> Result<(usize, usize, usize), TfError> {
        let p = self.lookup_slot(parent)?;
        let c = self.lookup_slot(child)?;
        let anc = self.common_ancestor(p, c).ok_or(TfError::NoPath {
            parent: self.frames[p].name,
            child: self.frames[c].name,
        })?;
        Ok((p, c, anc))
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
                    parent: self.frames[p].name,
                    child: self.frames[c].name,
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

    /// The oldest of the newest stamps of the dynamic edges on the chain
    /// from `from` up to `anc`: the latest time every one of them has a
    /// sample for. `None` when every edge on it is static.
    fn chain_newest(&self, from: usize, anc: usize) -> Option<u64> {
        let mut newest = None;
        let mut cur = from;
        while cur != anc {
            let frame = &self.frames[cur];
            if let Some(last) = frame.samples.back() {
                newest = Some(newest.map_or(last.stamp_ns, |n: u64| n.min(last.stamp_ns)));
            }
            match frame.parent {
                Some(next) => cur = next,
                None => break,
            }
        }
        newest
    }

    /// `T_parent_child` for the edge into `child` at `stamp_ns`: the static
    /// value, an exact hit, or the interpolation between the bracketing
    /// samples. Never extrapolates.
    fn edge_at(&self, child: usize, stamp_ns: u64) -> Result<Transform, TfError> {
        let frame = &self.frames[child];
        if frame.is_static {
            return Ok(frame.static_transform);
        }
        let ring = &frame.samples;
        let named =
            |f: fn(FrameName, FrameName, u64, u64, u64) -> TfError, oldest: u64, newest: u64| {
                let parent = frame
                    .parent
                    .map_or(FrameName::EMPTY, |p| self.frames[p].name);
                f(parent, frame.name, stamp_ns, oldest, newest)
            };
        let (Some(first), Some(last)) = (ring.front(), ring.back()) else {
            // A dynamic edge always holds a sample: `insert` records one as
            // it registers the parent link. This arm only keeps the match
            // total.
            return Err(named(out_of_range, 0, 0));
        };
        if stamp_ns < first.stamp_ns {
            return Err(named(out_of_range, first.stamp_ns, last.stamp_ns));
        }
        if stamp_ns > last.stamp_ns {
            return Err(named(not_yet_available, first.stamp_ns, last.stamp_ns));
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

fn out_of_range(
    parent: FrameName,
    child: FrameName,
    requested_ns: u64,
    oldest_ns: u64,
    newest_ns: u64,
) -> TfError {
    TfError::OutOfRange {
        parent,
        child,
        requested_ns,
        oldest_ns,
        newest_ns,
    }
}

fn not_yet_available(
    parent: FrameName,
    child: FrameName,
    requested_ns: u64,
    _oldest_ns: u64,
    newest_ns: u64,
) -> TfError {
    TfError::NotYetAvailable {
        parent,
        child,
        requested_ns,
        newest_ns,
    }
}
