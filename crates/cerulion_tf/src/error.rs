// SPDX-License-Identifier: AGPL-3.0-only
//! The one error type of the crate: why an SE(3) operation refused its
//! input, why the tf buffer refused an insert, or why a lookup returned no
//! transform.

use std::fmt;

use crate::{FrameId, FrameLabel, TfBuffer};

/// The former name of [`TfError`], from before the crate had a buffer.
/// Every variant it had is a variant of [`TfError`] with the same fields,
/// so code that names it or builds its variants still compiles. A `match`
/// over it now needs an arm for the buffer refusals too.
#[deprecated(note = "renamed to `TfError`, which adds the buffer refusals")]
pub type TfMathError = TfError;

/// Why an SE(3) operation or the tf buffer refused its input.
///
/// **The math.** Every operation that builds a [`crate::Transform`] or a
/// [`crate::UnitQuaternion`] checks its result, so a value of either type is
/// always finite and its rotation always unit length. A bad input is an
/// error here, never a NaN in the output. The math refusals are
/// `NonFiniteQuaternion`, `DegenerateQuaternion`, `NonFiniteTranslation`,
/// `FractionNotANumber` and `FractionOutOfRange`. In a node they come from
/// the feed, where [`crate::Transform::new`] or [`crate::UnitQuaternion::new`]
/// refuses a value before it reaches the buffer.
///
/// **The buffer.** Every refusal that involves a frame names it, and every
/// lookup refusal is a plain fact about the buffer, never a guessed
/// transform. A frame the buffer has registered is named by its
/// [`FrameId`]; a frame id it has not registered is named by a
/// [`FrameLabel`] held inline. Either way building an error allocates
/// nothing and the error stays `Copy`. The plain text of an error prints a
/// [`FrameId`] as `#n`; [`TfBuffer::describe`] prints it with the frame
/// names of the buffer that returned it.
///
/// The insert refusals are `EmptyFrameId`, `SelfEdge`, `SecondParent`,
/// `Cycle`, `EdgeIsStatic`, `StampBeforeWindow`, `StampTooFarAhead`,
/// `FrameTableFull` and `NameBudgetFull`. The lookup refusals are
/// `EmptyFrameId`, `UnknownFrame`, `UnknownFrameId`, `NoPath`, `EdgeEmpty`,
/// `NotYetAvailable`, `OutOfRange` and `ChainMath`.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum TfError {
    /// A quaternion component is NaN or infinite.
    NonFiniteQuaternion,
    /// The quaternion is too short to name a rotation: its norm is below
    /// [`crate::UnitQuaternion::MIN_NORM`]. The zero quaternion reports a
    /// norm of `0.0`.
    DegenerateQuaternion {
        /// The norm of the refused quaternion.
        norm: f64,
    },
    /// A translation component is NaN or infinite, in the input or in the
    /// result of a compose or inverse that overflowed. Interpolation never
    /// returns it: its linear blend stays within rounding of the segment
    /// between two finite endpoints.
    NonFiniteTranslation,
    /// The interpolation fraction is NaN.
    FractionNotANumber,
    /// The interpolation fraction is outside `[0, 1]`. Interpolation never
    /// extrapolates.
    FractionOutOfRange {
        /// The refused fraction.
        t: f64,
    },
    /// A frame id is the empty string (or only `/`s).
    EmptyFrameId,
    /// The parent and the child are the same frame.
    SelfEdge {
        /// The frame named on both ends.
        frame: FrameLabel,
    },
    /// The child already has a different parent. A frame has one parent;
    /// re-parenting is refused rather than applied.
    SecondParent {
        /// The child that already has a parent.
        child: FrameId,
        /// The parent the refused transform named.
        parent: FrameLabel,
        /// The parent the buffer holds.
        existing_parent: FrameId,
    },
    /// The edge would make a frame its own ancestor.
    Cycle {
        /// The parent the refused transform named.
        parent: FrameId,
        /// The child the refused transform named.
        child: FrameId,
    },
    /// The edge is static (it came in through [`TfBuffer::insert_static`])
    /// and a dynamic transform was inserted for it, or an edge policy was
    /// set on it. A static transform wins at every time, so the dynamic one
    /// is refused rather than ignored. Returned for the first such insert
    /// on an edge; the edge counts every one.
    EdgeIsStatic {
        /// The parent of the static edge.
        parent: FrameId,
        /// The child of the static edge.
        child: FrameId,
    },
    /// The stamp is older than the edge's retention window: more than the
    /// buffer's retention before the newest sample on the edge. A clock
    /// reset or a bag loop that the buffer was not told about
    /// ([`TfBuffer::begin_epoch`]) looks like this.
    StampBeforeWindow {
        /// The parent of the edge.
        parent: FrameId,
        /// The child of the edge.
        child: FrameId,
        /// The refused stamp, in nanoseconds.
        stamp_ns: u64,
        /// The newest stamp the edge holds, in nanoseconds.
        newest_ns: u64,
    },
    /// The stamp is more than the buffer's jump limit
    /// ([`TfBuffer::jump_limit`]) past the newest sample on the edge:
    /// accepted, one bad stamp would expire the whole ring at once. A single
    /// far-future stamp is refused this way and leaves the ring as it was;
    /// the next insert at a later stamp within the jump limit of the
    /// refused one confirms the jump, since two inserts that agree on the
    /// new time are a clock that moved forward. Returned for every such
    /// insert; the edge counts every one.
    StampTooFarAhead {
        /// The parent of the edge.
        parent: FrameId,
        /// The child of the edge.
        child: FrameId,
        /// The refused stamp, in nanoseconds.
        stamp_ns: u64,
        /// The newest stamp the edge holds, in nanoseconds.
        newest_ns: u64,
    },
    /// The buffer holds as many frames as it was built for and the transform
    /// names a frame it has never seen.
    FrameTableFull {
        /// The new frame that does not fit.
        frame: FrameLabel,
        /// The buffer's frame capacity.
        capacity: usize,
    },
    /// The new frame's id does not fit in what is left of the buffer's name
    /// byte budget.
    NameBudgetFull {
        /// The new frame that does not fit.
        frame: FrameLabel,
        /// The length of its id, in bytes.
        len: usize,
        /// The name bytes left for it, in bytes: for a new child named
        /// beside a new parent, what the parent leaves.
        left: usize,
        /// The buffer's name byte budget.
        budget: usize,
    },
    /// A lookup named a frame no inserted transform has mentioned.
    UnknownFrame {
        /// The frame the lookup named.
        frame: FrameLabel,
    },
    /// A lookup by handle named a [`FrameId`] this buffer never issued.
    UnknownFrameId {
        /// The handle the lookup named.
        frame: FrameId,
    },
    /// Both frames are known, but no chain of edges joins them: they are in
    /// separate trees.
    NoPath {
        /// The parent the lookup named.
        parent: FrameId,
        /// The child the lookup named.
        child: FrameId,
    },
    /// A dynamic edge on the chain holds no sample: none has arrived since
    /// the edge was declared ([`TfBuffer::set_edge_policy`]) or since the
    /// last [`TfBuffer::begin_epoch`]. The same lookup on the node's next
    /// fire may succeed.
    EdgeEmpty {
        /// The parent of the empty edge.
        parent: FrameId,
        /// The child of the empty edge.
        child: FrameId,
    },
    /// The requested time is newer than the newest sample of an edge on the
    /// chain, and further than the edge's policy reaches. The data for that
    /// time has not arrived; the same lookup on the node's next fire may
    /// succeed.
    NotYetAvailable {
        /// The parent of the edge that has no sample at the requested time.
        parent: FrameId,
        /// The child of that edge.
        child: FrameId,
        /// The requested time, in nanoseconds.
        requested_ns: u64,
        /// The newest stamp that edge holds, in nanoseconds.
        newest_ns: u64,
    },
    /// The requested time is older than the oldest sample of an edge on the
    /// chain: it left the retention window, or was never buffered. Waiting
    /// does not help.
    OutOfRange {
        /// The parent of the edge that has no sample at the requested time.
        parent: FrameId,
        /// The child of that edge.
        child: FrameId,
        /// The requested time, in nanoseconds.
        requested_ns: u64,
        /// The oldest stamp that edge holds, in nanoseconds.
        oldest_ns: u64,
        /// The newest stamp that edge holds, in nanoseconds.
        newest_ns: u64,
    },
    /// Composing the chain between the two frames overflowed: the
    /// translations along it, or an extrapolated edge, add past `f64::MAX`.
    /// The frames are the ones the lookup named. It is the buffer's form of
    /// [`Self::NonFiniteTranslation`], the only math refusal a chain can
    /// meet.
    ChainMath {
        /// The parent the lookup named.
        parent: FrameId,
        /// The child the lookup named.
        child: FrameId,
    },
}

/// A [`TfError`] printed with the frame names of the buffer that returned
/// it; built by [`TfBuffer::describe`].
#[derive(Debug, Clone, Copy)]
pub struct Describe<'a> {
    pub(crate) error: &'a TfError,
    pub(crate) buffer: &'a TfBuffer,
}

impl fmt::Display for Describe<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.error.render(f, Some(self.buffer))
    }
}

impl fmt::Display for TfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.render(f, None)
    }
}

/// A frame handle printed as its name when a buffer that knows it is at
/// hand, and as `#n` otherwise.
struct Named<'a>(FrameId, Option<&'a TfBuffer>);

impl fmt::Display for Named<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.1.and_then(|b| b.frame_name(self.0)) {
            Some(name) => f.write_str(name),
            None => fmt::Display::fmt(&self.0, f),
        }
    }
}

impl TfError {
    fn render(&self, f: &mut fmt::Formatter<'_>, names: Option<&TfBuffer>) -> fmt::Result {
        let n = |id: &FrameId| Named(*id, names);
        match self {
            Self::NonFiniteQuaternion => {
                f.write_str("quaternion has a non-finite component (NaN or infinity)")
            }
            Self::DegenerateQuaternion { norm } => write!(
                f,
                "quaternion norm {norm:e} is below the minimum of 1e-12, so it names no rotation"
            ),
            Self::NonFiniteTranslation => {
                f.write_str("translation has a non-finite component (NaN or infinity)")
            }
            Self::FractionNotANumber => f.write_str("interpolation fraction is NaN"),
            Self::FractionOutOfRange { t } => write!(
                f,
                "interpolation fraction {t} is outside [0, 1]; interpolation never extrapolates"
            ),
            Self::EmptyFrameId => f.write_str("frame id is empty"),
            Self::SelfEdge { frame } => write!(
                f,
                "transform `{frame}` to `{frame}` is a self edge: a frame cannot be its own parent"
            ),
            Self::SecondParent {
                child,
                parent,
                existing_parent,
            } => write!(
                f,
                "frame `{}` already has the parent `{}`; a transform from `{parent}` would give \
                 it a second parent",
                n(child),
                n(existing_parent)
            ),
            Self::Cycle { parent, child } => write!(
                f,
                "transform `{p}` to `{c}` would close a cycle: `{p}` descends from `{c}`",
                p = n(parent),
                c = n(child)
            ),
            Self::EdgeIsStatic { parent, child } => write!(
                f,
                "transform `{}` to `{}` is static; a dynamic transform or an edge policy on a \
                 static edge is refused",
                n(parent),
                n(child)
            ),
            Self::StampBeforeWindow {
                parent,
                child,
                stamp_ns,
                newest_ns,
            } => write!(
                f,
                "transform `{}` to `{}` at {stamp_ns} ns is older than the retention window \
                 before the newest stamp {newest_ns} ns; after a clock reset or a bag loop, \
                 begin a new epoch",
                n(parent),
                n(child)
            ),
            Self::StampTooFarAhead {
                parent,
                child,
                stamp_ns,
                newest_ns,
            } => write!(
                f,
                "transform `{}` to `{}` at {stamp_ns} ns is more than the jump limit past the \
                 newest stamp {newest_ns} ns and is refused; a later transform within the jump \
                 limit of it confirms the jump",
                n(parent),
                n(child)
            ),
            Self::FrameTableFull { frame, capacity } => write!(
                f,
                "frame `{frame}` does not fit: the buffer holds its capacity of {capacity} frames"
            ),
            Self::NameBudgetFull {
                frame,
                len,
                left,
                budget,
            } => write!(
                f,
                "frame `{frame}` does not fit: its {len}-byte id is longer than the {left} bytes \
                 left of the name budget of {budget} bytes"
            ),
            Self::UnknownFrame { frame } => write!(
                f,
                "frame `{frame}` is unknown: no transform naming it has been received"
            ),
            Self::UnknownFrameId { frame } => {
                write!(f, "frame handle {frame} was not issued by this buffer")
            }
            Self::NoPath { parent, child } => write!(
                f,
                "no path between frames `{}` and `{}`: they are in separate trees",
                n(parent),
                n(child)
            ),
            Self::EdgeEmpty { parent, child } => write!(
                f,
                "transform `{}` to `{}` has no sample yet in this epoch",
                n(parent),
                n(child)
            ),
            Self::NotYetAvailable {
                parent,
                child,
                requested_ns,
                newest_ns,
            } => write!(
                f,
                "transform `{}` to `{}` at {requested_ns} ns is not yet available: the newest \
                 buffered stamp is {newest_ns} ns, and the edge's policy does not reach that far",
                n(parent),
                n(child)
            ),
            Self::OutOfRange {
                parent,
                child,
                requested_ns,
                oldest_ns,
                newest_ns,
            } => write!(
                f,
                "transform `{}` to `{}` at {requested_ns} ns is out of range: the buffered \
                 stamps run from {oldest_ns} ns to {newest_ns} ns",
                n(parent),
                n(child)
            ),
            Self::ChainMath { parent, child } => write!(
                f,
                "transform `{}` to `{}` could not be composed along its chain: translation has \
                 a non-finite component (NaN or infinity)",
                n(parent),
                n(child)
            ),
        }
    }
}
