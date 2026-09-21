// SPDX-License-Identifier: AGPL-3.0-only
//! Why a checkpoint anchor was VOIDED: the skip-cause vocabulary.
//!
//! # Why this vocabulary lives on the PORTABLE side
//!
//! Its natural home is `crate::state_ring` — named in prose, NOT as an
//! intra-doc link, because THIS module is portable while that one is
//! `#[cfg(unix)]`, so the link is unresolvable on a non-unix target (harmless
//! only for as long as `mod skip_cause` stays private, since rustdoc lints
//! links in rendered items; one `pub` away from failing the docs gate —
//! a hazard pinned by `cfg_audit_test`). That module is `#[cfg(unix)]`
//! because it is POSIX SHM. The CAUSES are not: they are a wire vocabulary a
//! reader decodes out of a bag, and the machine that reads a bag need not be
//! the machine that recorded it. A restore's very first decision — "is this
//! anchor usable, and if not, why?" ([`crate::state_restore`]) — has to name a
//! skipped capture's cause, so gating the vocabulary on `unix` would gate the
//! whole restore SIDE on the OS that can run the CAPTURE side, which is the
//! opposite of the desk/robot split every other reader in this repo assumes.
//!
//! `state_ring` re-exports it, so `cerulion_core::state_ring::SkipCause` — the
//! path `cerulion_bagd` and the `cerulion_bag` channel tests already use —
//! still resolves unchanged.

/// Why an anchor (or one node's part of it) was VOIDED, carried by a
/// `RECORD_KIND_SKIP` record (`crate::state_ring`, whose constants are
/// unix-only; this type deliberately is not — see the module doc).
///
/// The causes are the capture path's failure table. WHICH of them a carrier can mint, and whether
/// an anchor-wide skip is spelled as one record per node or one record on a
/// designated node, is the carrier's decision — this layer owns the vocabulary so the carrier
/// does not have to change the record format to say something this vocabulary already
/// names.
///
/// [`Unrecognized`](Self::Unrecognized) exists so a cause minted by a NEWER writer is
/// REPORTED verbatim rather than silently dropped: an unreadable reason for a voided
/// anchor is worse than an unfamiliar one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkipCause {
    /// A node mutex or a DECLARED state lock was held at the pre-fork probe.
    Contended,
    /// The pre-fork memory reservation refused.
    LowMemory,
    /// A previous child was still draining, so no worker starts an anchor that would
    /// be rejected as `PartialAnchor`.
    StillEncoding,
    /// `fork(2)` itself failed.
    ForkFailed,
    /// The progress watchdog fired — the child stalled.
    ChildTimeout,
    /// The node's encoder returned an error, or a cdylib's `capture_state` returned
    /// non-zero.
    CaptureFailed,
    /// The capture child PANICKED inside a node's encoder.
    ///
    /// Distinct from [`CaptureFailed`](Self::CaptureFailed) because the remedies
    /// differ: a returned error is a refusal the encoder chose, while a panic is a
    /// bug in code that was never supposed to unwind — and the child's panic hook
    /// `_exit`s from inside the hook precisely so that unwind never reaches the
    /// parent's frames. Without its own cause the operator would read "capture
    /// failed" and go looking for a refusal that never happened.
    ChildPanicked,
    /// The capture child died on a SIGNAL (a segfault in an
    /// encoder, an OOM kill, the macOS post-fork CoreFoundation abort).
    ///
    /// Deliberately NOT folded into [`ChildTimeout`](Self::ChildTimeout): a signal
    /// death is PROMPT and the parent has an exit status for it, while a timeout
    /// means the watchdog stopped a child that never finished. Labelling the first
    /// as the second sends an operator hunting a deadlock that does not exist.
    ChildCrashed,
    /// The RECORDER had not drained enough
    /// of the state ring for this anchor to be written without blocking.
    ///
    /// The discrimination that makes this worth its own code: the symptom of a
    /// dead or stalled `bagd` is a checkpoint that stops appearing, and every
    /// other cause in this enum points at the ENCODER. An operator reading
    /// "capture failed" or "child stalled" would debug node code while the
    /// recorder daemon is the thing that is wrong.
    RecorderBehind,
    /// A cause code this build does not know, preserved verbatim.
    Unrecognized(u32),
}

impl SkipCause {
    /// The wire discriminant. Frozen: these values are in the bag.
    pub fn as_wire(self) -> u32 {
        match self {
            Self::Contended => 1,
            Self::LowMemory => 2,
            Self::StillEncoding => 3,
            Self::ForkFailed => 4,
            Self::ChildTimeout => 5,
            Self::CaptureFailed => 6,
            Self::ChildPanicked => 7,
            Self::ChildCrashed => 8,
            Self::RecorderBehind => 9,
            Self::Unrecognized(raw) => raw,
        }
    }

    /// Decode a wire discriminant. Never fails — an unknown code becomes
    /// [`Unrecognized`](Self::Unrecognized) so it survives to the operator.
    pub fn from_wire(raw: u32) -> Self {
        match raw {
            1 => Self::Contended,
            2 => Self::LowMemory,
            3 => Self::StillEncoding,
            4 => Self::ForkFailed,
            5 => Self::ChildTimeout,
            6 => Self::CaptureFailed,
            7 => Self::ChildPanicked,
            8 => Self::ChildCrashed,
            9 => Self::RecorderBehind,
            other => Self::Unrecognized(other),
        }
    }
}
