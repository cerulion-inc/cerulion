// SPDX-License-Identifier: AGPL-3.0-only
//! The error taxonomy of the tf buffer: why an insert was refused, or why a
//! lookup returned no transform.

use crate::{FrameName, TfMathError, MAX_FRAME_ID_LEN};

/// Why the tf buffer refused an insert or a lookup.
///
/// Every variant names the frames involved in its text, and every lookup
/// refusal is a plain fact about the buffer, never a guessed transform:
/// lookups never extrapolate. The variants carry their frame ids inline
/// ([`FrameName`]), so building one allocates nothing.
///
/// The insert refusals are `EmptyFrameId`, `FrameIdTooLong`, `SelfEdge`,
/// `SecondParent`, `Cycle`, `EdgeIsStatic`, `FrameTableFull` and `Math`
/// (a transform the SE(3) math refused). The lookup refusals are
/// `UnknownFrame`, `NoPath`, `NotYetAvailable` and `OutOfRange`.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum TfError {
    /// The SE(3) math refused a value: a non-finite or degenerate
    /// quaternion, a non-finite translation, or an overflow while composing
    /// a chain.
    #[error(transparent)]
    Math(#[from] TfMathError),
    /// A frame id is the empty string.
    #[error("frame id is empty")]
    EmptyFrameId,
    /// A frame id is longer than [`MAX_FRAME_ID_LEN`] bytes.
    #[error("frame id is {len} bytes long; the limit is {limit} bytes", limit = MAX_FRAME_ID_LEN)]
    FrameIdTooLong {
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
}
