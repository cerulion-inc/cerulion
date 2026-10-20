// SPDX-License-Identifier: AGPL-3.0-only
//! The one error type of the crate: why an SE(3) operation refused its
//! input, why the tf buffer refused an insert, or why a lookup returned no
//! transform.

use crate::{FrameName, MAX_FRAME_ID_LEN};

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
/// **The buffer.** Every refusal that involves a frame names it in its text,
/// and every lookup refusal is a plain fact about the buffer, never a
/// guessed transform: lookups never extrapolate. The variants carry their
/// frame ids inline ([`FrameName`]), so building one allocates nothing.
/// `EmptyFrameId` has no frame to name: the id is the empty string. The
/// insert refusals are `EmptyFrameId`, `FrameIdTooLong`, `SelfEdge`,
/// `SecondParent`, `Cycle`, `EdgeIsStatic` and `FrameTableFull`. The lookup
/// refusals are `EmptyFrameId`, `FrameIdTooLong`, `UnknownFrame`, `NoPath`,
/// `NotYetAvailable`, `OutOfRange` and `ChainMath`.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum TfError {
    /// A quaternion component is NaN or infinite.
    #[error("quaternion has a non-finite component (NaN or infinity)")]
    NonFiniteQuaternion,
    /// The quaternion is too short to name a rotation: its norm is below
    /// [`crate::UnitQuaternion::MIN_NORM`]. The zero quaternion reports a
    /// norm of `0.0`.
    #[error("quaternion norm {norm:e} is below the minimum of 1e-12, so it names no rotation")]
    DegenerateQuaternion {
        /// The norm of the refused quaternion.
        norm: f64,
    },
    /// A translation component is NaN or infinite, in the input or in the
    /// result of a compose or inverse that overflowed. Interpolation never
    /// returns it: its linear blend stays within rounding of the segment
    /// between two finite endpoints.
    #[error("translation has a non-finite component (NaN or infinity)")]
    NonFiniteTranslation,
    /// The interpolation fraction is NaN.
    #[error("interpolation fraction is NaN")]
    FractionNotANumber,
    /// The interpolation fraction is outside `[0, 1]`. Interpolation never
    /// extrapolates.
    #[error("interpolation fraction {t} is outside [0, 1]; interpolation never extrapolates")]
    FractionOutOfRange {
        /// The refused fraction.
        t: f64,
    },
    /// A frame id is the empty string (or only `/`s).
    #[error("frame id is empty")]
    EmptyFrameId,
    /// A frame id is longer than [`MAX_FRAME_ID_LEN`] bytes.
    #[error(
        "frame id `{prefix}...` is {len} bytes long; the limit is {limit} bytes",
        limit = MAX_FRAME_ID_LEN
    )]
    FrameIdTooLong {
        /// The first [`MAX_FRAME_ID_LEN`] bytes of the refused id, cut at a
        /// character boundary.
        prefix: FrameName,
        /// The length of the refused id, in bytes.
        len: usize,
    },
    /// The parent and the child are the same frame.
    #[error("transform `{frame}` to `{frame}` is a self edge: a frame cannot be its own parent")]
    SelfEdge {
        /// The frame named on both ends.
        frame: FrameName,
    },
    /// The child already has a different parent. A frame has one parent;
    /// re-parenting is refused rather than applied.
    #[error(
        "frame `{child}` already has the parent `{existing_parent}`; a transform from \
         `{parent}` would give it a second parent"
    )]
    SecondParent {
        /// The child that already has a parent.
        child: FrameName,
        /// The parent the refused transform named.
        parent: FrameName,
        /// The parent the buffer holds.
        existing_parent: FrameName,
    },
    /// The edge would make a frame its own ancestor.
    #[error(
        "transform `{parent}` to `{child}` would close a cycle: `{parent}` descends from `{child}`"
    )]
    Cycle {
        /// The parent the refused transform named.
        parent: FrameName,
        /// The child the refused transform named.
        child: FrameName,
    },
    /// The edge is static (it came in through [`crate::TfBuffer::insert_static`])
    /// and a dynamic transform was inserted for it. A static transform wins
    /// at every time, so the dynamic one is refused rather than ignored.
    #[error("transform `{parent}` to `{child}` is static; a dynamic transform on a static edge is refused")]
    EdgeIsStatic {
        /// The parent of the static edge.
        parent: FrameName,
        /// The child of the static edge.
        child: FrameName,
    },
    /// The buffer holds as many frames as it was built for and the transform
    /// names a frame it has never seen.
    #[error("frame `{frame}` does not fit: the buffer holds its capacity of {capacity} frames")]
    FrameTableFull {
        /// The new frame that does not fit.
        frame: FrameName,
        /// The buffer's frame capacity.
        capacity: usize,
    },
    /// A lookup named a frame no inserted transform has mentioned.
    #[error("frame `{frame}` is unknown: no transform naming it has been received")]
    UnknownFrame {
        /// The frame the lookup named.
        frame: FrameName,
    },
    /// Both frames are known, but no chain of edges joins them: they are in
    /// separate trees.
    #[error("no path between frames `{parent}` and `{child}`: they are in separate trees")]
    NoPath {
        /// The parent the lookup named.
        parent: FrameName,
        /// The child the lookup named.
        child: FrameName,
    },
    /// The requested time is newer than the newest sample of an edge on the
    /// chain. The data for that time has not arrived; the same lookup on the
    /// node's next fire may succeed.
    #[error(
        "transform `{parent}` to `{child}` at {requested_ns} ns is not yet available: the \
         newest buffered stamp is {newest_ns} ns, and lookups never extrapolate"
    )]
    NotYetAvailable {
        /// The parent of the edge that has no sample at the requested time.
        parent: FrameName,
        /// The child of that edge.
        child: FrameName,
        /// The requested time, in nanoseconds.
        requested_ns: u64,
        /// The newest stamp that edge holds, in nanoseconds.
        newest_ns: u64,
    },
    /// The requested time is older than the oldest sample of an edge on the
    /// chain: it was evicted, or never buffered. Waiting does not help.
    #[error(
        "transform `{parent}` to `{child}` at {requested_ns} ns is out of range: the buffered \
         stamps run from {oldest_ns} ns to {newest_ns} ns, and lookups never extrapolate"
    )]
    OutOfRange {
        /// The parent of the edge that has no sample at the requested time.
        parent: FrameName,
        /// The child of that edge.
        child: FrameName,
        /// The requested time, in nanoseconds.
        requested_ns: u64,
        /// The oldest stamp that edge holds, in nanoseconds.
        oldest_ns: u64,
        /// The newest stamp that edge holds, in nanoseconds.
        newest_ns: u64,
    },
    /// Composing the chain between the two frames overflowed: the
    /// translations along it add past `f64::MAX`. The frames are the ones
    /// the lookup named. It is the buffer's form of
    /// [`Self::NonFiniteTranslation`], the only math refusal a chain can
    /// meet.
    #[error(
        "transform `{parent}` to `{child}` could not be composed along its chain: \
         translation has a non-finite component (NaN or infinity)"
    )]
    ChainMath {
        /// The parent the lookup named.
        parent: FrameName,
        /// The child the lookup named.
        child: FrameName,
    },
}
