// SPDX-License-Identifier: MIT OR Apache-2.0
//! Errors returned by the `cerulion/ops/1` framing, codec, handshake and
//! client.
//!
//! Each variant carries the same fields and prints the same message as the
//! `cerud::CerudError` variant of the same name; `cerud` converts one into the
//! other variant for variant.

/// Errors from the ops protocol layer.
///
/// Not `#[non_exhaustive]`: `cerud` maps every variant onto its own error
/// enum with an exhaustive `match`, so a new variant here fails that build
/// until it is mapped.
#[derive(Debug, thiserror::Error)]
pub enum OpsProtoError {
    /// Underlying I/O failure on the stream.
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),

    /// JSON serialization failure for an envelope.
    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),

    /// A frame's declared or actual length exceeds
    /// [`crate::protocol::MAX_FRAME_BYTES`].
    #[error("protocol frame too large: {size} bytes exceeds the {max}-byte cap")]
    FrameTooLarge { size: usize, max: usize },

    /// A framed payload could not be decoded into the expected message, or a
    /// message is self-inconsistent (an inverted version range, a response id
    /// that does not match its request).
    #[error("protocol decode error: {0}")]
    Decode(String),

    /// The two version ranges do not overlap. Produced by
    /// [`crate::protocol::negotiate_version`] on the server side; the client
    /// reports a refusal as [`OpsProtoError::HandshakeRejected`].
    #[error(
        "protocol version mismatch: server supports {server_min}..={server_max}, \
         client offered {client_min}..={client_max}"
    )]
    VersionMismatch {
        client_min: u16,
        client_max: u16,
        server_min: u16,
        server_max: u16,
    },

    /// The server answered the [`crate::protocol::Hello`] with a
    /// [`crate::protocol::HandshakeReply::Reject`]. `reason` is the frame's
    /// `message`, verbatim, and `server_min`/`server_max` its version range.
    #[error("handshake rejected by server (supports {server_min}..={server_max}): {reason}")]
    HandshakeRejected {
        reason: String,
        server_min: u16,
        server_max: u16,
    },

    /// The peer closed the stream at a frame boundary (EOF before the length
    /// prefix).
    #[error("connection closed by peer")]
    ConnectionClosed,

    /// The server answered a [`crate::protocol::Request`] with a
    /// [`crate::protocol::Response::Err`] carrying this `kind` and `message`.
    #[error("remote error [{kind}]: {message}")]
    Remote { kind: String, message: String },
}

/// Result alias for the ops protocol layer.
pub type OpsProtoResult<T> = Result<T, OpsProtoError>;

/// The machine-readable error class written into a
/// [`crate::protocol::Response::Err`] frame's `kind` field.
///
/// [`crate::protocol::Response::from_error`] takes any error implementing this
/// and `Display`; `cerud` implements it for its own error enum.
pub trait ErrorKind {
    /// The `kind` string for this error.
    fn kind(&self) -> &'static str;
}
