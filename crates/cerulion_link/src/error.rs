// SPDX-License-Identifier: MIT OR Apache-2.0
//! The unified error type for `cerulion_link`.

/// A boxed source error, used to carry iroh's many concrete error types
/// (`BindError`, `ConnectError`, stream read/write errors, …) without coupling
/// this thin wrapper's public enum to their exact type names. The source chain
/// (`Display` + [`std::error::Error::source`]) is preserved.
pub type BoxError = Box<dyn std::error::Error + Send + Sync + 'static>;

/// Errors returned by `cerulion_link`.
#[derive(Debug, thiserror::Error)]
pub enum LinkError {
    /// Binding the iroh endpoint to a local socket failed.
    #[error("failed to bind iroh endpoint")]
    Bind(#[source] BoxError),

    /// Dialing a peer failed. `peer` is the target endpoint id, hex-encoded.
    #[error("failed to dial peer {peer}")]
    Connect {
        /// The target endpoint id (hex).
        peer: String,
        /// The underlying iroh connect error.
        #[source]
        source: BoxError,
    },

    /// Accepting / completing an incoming connection's handshake failed.
    #[error("failed to accept incoming connection")]
    Accept(#[source] BoxError),

    /// A stream operation failed: opening or accepting a stream — bidirectional
    /// ([`open_frame_stream`](crate::open_frame_stream) /
    /// [`accept_frame_stream`](crate::accept_frame_stream)) OR unidirectional
    /// ([`open_uni_frame_stream`](crate::open_uni_frame_stream) /
    /// [`accept_uni_frame_stream`](crate::accept_uni_frame_stream)) — or a
    /// stream read/write. The direction and the concrete iroh error ride the
    /// source chain (`source`), not this variant's `Display`.
    #[error("stream error")]
    Stream(#[source] BoxError),

    /// WRITE side: a frame whose length does not fit the wire format's `u32`
    /// length prefix — a HARD wire-format limit of `u32::MAX` bytes (~4 GiB),
    /// not a configurable cap. Such a frame can NEVER be sent; the only remedy
    /// is a smaller frame. Distinct from [`FrameExceedsMaxLen`], which is a
    /// caller-tunable ceiling.
    ///
    /// [`FrameExceedsMaxLen`]: LinkError::FrameExceedsMaxLen
    #[error(
        "frame length {len} exceeds the u32 wire-format length-prefix limit ({} bytes)",
        u32::MAX
    )]
    FrameExceedsWireLimit {
        /// The frame length that overflowed the `u32` length prefix.
        len: usize,
    },

    /// READ side: a peer's declared frame length exceeds the caller-supplied
    /// anti-DoS cap (`max_len` passed to [`read_frame`](crate::read_frame)).
    /// Unlike [`FrameExceedsWireLimit`], this is the CALLER's configured
    /// ceiling.
    ///
    /// **The stream is unusable after this error — drop it.**
    /// [`read_frame`](crate::read_frame) has already consumed the 4-byte length
    /// prefix but NOT the payload, so
    /// the stream is left mid-frame and permanently desynced (exactly like a
    /// cancelled `read_frame`). This is NOT retryable on the same stream:
    /// raising `max_len` (or having the peer send a smaller frame) only helps a
    /// FUTURE stream / session, never the desynced one.
    ///
    /// [`FrameExceedsWireLimit`]: LinkError::FrameExceedsWireLimit
    #[error("declared frame length {len} exceeds the configured maximum {max}")]
    FrameExceedsMaxLen {
        /// The peer's declared frame length.
        len: usize,
        /// The caller-supplied `max_len` cap that was exceeded.
        max: usize,
    },

    /// The connection's mutually-authenticated peer identity did not match the
    /// expected device key. `expected` / `actual` are hex-encoded 32-byte keys.
    #[error("peer identity mismatch: expected {expected}, got {actual}")]
    IdentityMismatch {
        /// The expected device key (hex).
        expected: String,
        /// The peer's actual TLS-authenticated key (hex).
        actual: String,
    },

    /// A peer's advertised `eid=` (hex [`EndpointId`](crate::EndpointId), e.g.
    /// from an mDNS TXT record or a pinned key) was not a valid 32-byte ed25519
    /// public key — bad hex, the wrong length, or a non-canonical key. This is a
    /// MALFORMED advertisement caught BEFORE any dial, distinct from a dial-time
    /// [`IdentityMismatch`](LinkError::IdentityMismatch) (which compares a real
    /// authenticated peer against an expected key).
    #[error("invalid EndpointId '{eid}': {reason}")]
    BadEndpointId {
        /// The offending hex string.
        eid: String,
        /// Why it could not be parsed (bad hex / wrong length / non-canonical key).
        reason: String,
    },

    /// A LAN-discovered peer advertised no `iroh_port` TXT key, so a direct-dial
    /// [`EndpointAddr`](crate::EndpointAddr) cannot be built — iroh's UDP/QUIC
    /// port is DISTINCT from the zenoh SRV/TCP port carried in the mDNS SRV
    /// record, so there is no address to dial. The caller should fall back to
    /// relay / discovery dialing (a bare [`EndpointId`](crate::EndpointId)).
    #[error(
        "the discovered peer advertised no iroh_port; cannot build a LAN direct-dial address \
         (iroh's UDP port is distinct from the zenoh SRV/TCP port)"
    )]
    MissingIrohPort,
}
