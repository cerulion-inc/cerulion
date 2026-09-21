// SPDX-License-Identifier: AGPL-3.0-only
//! Unified error type for `cerud`.

/// Errors produced anywhere in `cerud`.
///
/// `#[non_exhaustive]` so future variants stay additive across crate
/// boundaries (mirrors `cerulion_core::TransportError`).
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum CerudError {
    /// Underlying I/O failure (socket, receipt-log file, verb file reads).
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),

    /// JSON (de)serialization failure for an envelope / manifest / verb value.
    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),

    /// Malformed configuration (e.g. an unparseable `CERULION_OPS_PORT`).
    #[error("config error: {0}")]
    Config(String),

    /// A protocol frame's declared length exceeds the hard cap.
    #[error("protocol frame too large: {size} bytes exceeds the {max}-byte cap")]
    FrameTooLarge { size: usize, max: usize },

    /// A framed payload could not be decoded into the expected message.
    #[error("protocol decode error: {0}")]
    Decode(String),

    /// The peer offered no protocol version the server supports (refused at
    /// the connect handshake, before any verb runs). This is the SERVER-side
    /// classification produced by [`crate::protocol::negotiate_version`]; the
    /// client surfaces a rejection as [`CerudError::HandshakeRejected`].
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

    /// The server refused the connect handshake. `reason` is the server's own
    /// explanation, surfaced VERBATIM from the wire `Reject` frame, and
    /// `server_min`/`server_max` are the version range it supports.
    ///
    /// This is the CLIENT-side surfacing of ANY handshake rejection. The wire
    /// `Reject.message` is free-form, so the reason may be a version
    /// non-overlap, a malformed (inverted) client range, or — from the
    /// remote-plane pairing bootstrap — a not-yet-paired refusal. The client
    /// does not reinterpret it as a version mismatch; it reports what the
    /// server actually said, so a non-version rejection is never mislabeled.
    #[error("handshake rejected by server (supports {server_min}..={server_max}): {reason}")]
    HandshakeRejected {
        reason: String,
        server_min: u16,
        server_max: u16,
    },

    /// The peer closed the connection cleanly (EOF at a frame boundary).
    #[error("connection closed by peer")]
    ConnectionClosed,

    /// The per-verb authorization seam denied the call. Recorded in the
    /// receipt log with outcome `Denied`.
    #[error("authorization denied for caller '{caller}' verb '{verb}': {reason}")]
    Denied {
        caller: String,
        verb: String,
        reason: String,
    },

    /// The request named a verb the server does not implement.
    #[error("unknown verb: '{0}'")]
    UnknownVerb(String),

    /// A verb ran but failed (bad args, target error, etc.).
    #[error("verb error: {0}")]
    Verb(String),

    /// A verb is not supported on this host (e.g. `restart` off Linux). This
    /// is a structured refusal, never a panic.
    #[error("unsupported on this host: {0}")]
    UnsupportedHost(String),

    /// A deploy-bundle manifest is malformed or self-inconsistent.
    #[error("deploy manifest error: {0}")]
    Manifest(String),

    /// A bundle file's on-disk SHA-256 did not match its manifest digest.
    #[error("digest mismatch for '{path}': manifest {expected}, on disk {actual}")]
    DigestMismatch {
        path: String,
        expected: String,
        actual: String,
    },

    /// A `log-tail` request named a path that escapes the allow-listed root.
    #[error("path traversal refused: '{requested}' escapes allow-listed root '{root}'")]
    PathTraversal { requested: String, root: String },

    /// A control-lease / deadman / e-stop state-machine violation.
    #[error("lease error: {0}")]
    Lease(String),

    /// A receipt-log read/append/parse failure.
    #[error("receipt log error: {0}")]
    Receipt(String),

    /// A structured error surfaced by the REMOTE peer (client side): the
    /// server returned a `Response::Err` carrying this `kind` + `message`.
    #[error("remote error [{kind}]: {message}")]
    Remote { kind: String, message: String },
}

/// Convenience alias, mirroring `cerulion_core::TransportResult`.
pub type CerudResult<T> = Result<T, CerudError>;

impl CerudError {
    /// Machine-readable error class stamped into a `Response::Err` frame so
    /// clients can branch on the failure without string-matching the message.
    pub fn kind(&self) -> &'static str {
        match self {
            CerudError::Denied { .. } => "denied",
            CerudError::UnknownVerb(_) => "unknown_verb",
            CerudError::Verb(_) => "verb_error",
            CerudError::UnsupportedHost(_) => "unsupported_host",
            CerudError::PathTraversal { .. } => "path_traversal",
            CerudError::Manifest(_) => "manifest_error",
            CerudError::DigestMismatch { .. } => "digest_mismatch",
            CerudError::Lease(_) => "lease_error",
            CerudError::Decode(_) => "decode_error",
            CerudError::FrameTooLarge { .. } => "frame_too_large",
            CerudError::VersionMismatch { .. } => "version_mismatch",
            CerudError::HandshakeRejected { .. } => "handshake_rejected",
            CerudError::Config(_) => "config_error",
            CerudError::Receipt(_) => "receipt_error",
            CerudError::ConnectionClosed => "connection_closed",
            CerudError::Remote { .. } => "remote_error",
            CerudError::Io(_) | CerudError::Json(_) => "internal",
        }
    }
}
