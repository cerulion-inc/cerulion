// SPDX-License-Identifier: MIT OR Apache-2.0
//! The `cerulion/ops/1` wire protocol: length-prefixed frames, a connect-time
//! version negotiation, and the request/response envelopes.
//!
//! The protocol is layered ON TOP of any "framed bidirectional bytes" stream,
//! so every transport (the `cerud` Unix-domain-socket dev transport, the iroh
//! `cerulion/ops/1` stream) shares one framing and one handshake.
//!
//! # Framing
//!
//! Each frame is a big-endian `u32` length prefix followed by that many
//! payload bytes. Payloads are JSON. A hard [`MAX_FRAME_BYTES`] cap bounds a
//! hostile/garbled length prefix.
//!
//! # Handshake
//!
//! On connect the client sends a [`Hello`] carrying the inclusive version
//! range it supports and the [`SessionKind`] it requests. Ordinary ops support
//! versions 1 and 2; shell and exec require version 2. The server replies with
//! [`HandshakeReply::Ack`] naming the highest common version for that session,
//! or [`HandshakeReply::Reject`] before any verb runs if there is no overlap.
//! A common version does not grant execution: the service must also support and
//! authorize the requested session. An ordinary ops service rejects shell and exec.

use std::fmt::Display;
use std::io::{Read, Write};

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::error::{ErrorKind, OpsProtoError, OpsProtoResult};
pub use crate::session::{
    ClientFrame, ExecRequest, ExitReason, OutputStream, ServerFrame, SessionExit, SessionKind,
    ShellRequest, WindowSize, MAX_DATA_BYTES, MAX_SESSION_FRAME_BYTES,
};

/// The newest protocol version this build speaks.
pub const PROTOCOL_VERSION: u16 = 2;

/// The oldest protocol version accepted for ordinary request/response ops.
pub const MIN_PROTOCOL_VERSION: u16 = 1;

/// The oldest protocol version accepted for shell and exec sessions.
pub const MIN_SESSION_PROTOCOL_VERSION: u16 = 2;

/// Hard cap on a single frame's payload (16 MiB). A declared length above this
/// is rejected without allocating the buffer.
pub const MAX_FRAME_BYTES: usize = 16 * 1024 * 1024;

/// Write one length-prefixed frame and flush.
pub fn write_frame<W: Write>(w: &mut W, payload: &[u8]) -> OpsProtoResult<()> {
    if payload.len() > MAX_FRAME_BYTES {
        return Err(OpsProtoError::FrameTooLarge {
            size: payload.len(),
            max: MAX_FRAME_BYTES,
        });
    }
    let len = payload.len() as u32;
    w.write_all(&len.to_be_bytes())?;
    w.write_all(payload)?;
    w.flush()?;
    Ok(())
}

/// Read one length-prefixed frame.
///
/// A clean EOF at the length prefix (nothing more from the peer) is reported
/// as [`OpsProtoError::ConnectionClosed`]; a truncated frame mid-payload is a
/// genuine I/O error.
pub fn read_frame<R: Read>(r: &mut R) -> OpsProtoResult<Vec<u8>> {
    let mut len_buf = [0u8; 4];
    match r.read_exact(&mut len_buf) {
        Ok(()) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
            return Err(OpsProtoError::ConnectionClosed);
        }
        Err(e) => return Err(e.into()),
    }
    let len = u32::from_be_bytes(len_buf) as usize;
    if len > MAX_FRAME_BYTES {
        return Err(OpsProtoError::FrameTooLarge {
            size: len,
            max: MAX_FRAME_BYTES,
        });
    }
    let mut buf = vec![0u8; len];
    r.read_exact(&mut buf)?; // a mid-payload EOF is a truncated-frame I/O error
    Ok(buf)
}

/// Serialize a message to a JSON frame payload.
pub fn encode<T: Serialize>(value: &T) -> OpsProtoResult<Vec<u8>> {
    Ok(serde_json::to_vec(value)?)
}

/// Deserialize a JSON frame payload into a message.
pub fn decode<T: DeserializeOwned>(bytes: &[u8]) -> OpsProtoResult<T> {
    serde_json::from_slice(bytes).map_err(|e| OpsProtoError::Decode(e.to_string()))
}

/// The client's opening message: its supported version range and session mode.
/// When constructing this value directly, use [`SessionKind::Ops`] for ordinary
/// verbs. [`Hello::current`] supplies that mode and this build's version range.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Hello {
    pub min_version: u16,
    pub max_version: u16,
    /// The mode of this connection. Ordinary ops omit this field on the wire.
    #[serde(default, skip_serializing_if = "is_ordinary_ops")]
    pub session: SessionKind,
}

fn is_ordinary_ops(session: &SessionKind) -> bool {
    matches!(session, SessionKind::Ops)
}

impl Hello {
    /// A Hello advertising exactly this build's supported range.
    pub fn current() -> Self {
        Hello {
            min_version: MIN_PROTOCOL_VERSION,
            max_version: PROTOCOL_VERSION,
            session: SessionKind::Ops,
        }
    }
}

/// The server's reply to a [`Hello`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum HandshakeReply {
    /// Negotiation succeeded on `chosen_version`.
    Ack {
        chosen_version: u16,
        server_name: String,
    },
    /// No common version; the connection is refused.
    Reject {
        server_min: u16,
        server_max: u16,
        message: String,
    },
}

/// A verb invocation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Request {
    /// Correlates the response with this request.
    pub id: u64,
    /// The verb name (used for both authz scope and dispatch).
    pub verb: String,
    /// Verb arguments (each verb deserializes its own typed args from this).
    #[serde(default)]
    pub args: serde_json::Value,
}

/// A verb result.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum Response {
    /// The verb ran and produced `result`.
    Ok { id: u64, result: serde_json::Value },
    /// The verb was refused or failed; `kind` is a machine-readable class.
    Err {
        id: u64,
        kind: String,
        message: String,
    },
}

impl Response {
    /// Build an error response from a request id and an error: `kind` is
    /// [`ErrorKind::kind`], `message` is the error's `Display` text.
    pub fn from_error<E: ErrorKind + Display>(id: u64, err: &E) -> Self {
        Response::Err {
            id,
            kind: err.kind().to_string(),
            message: err.to_string(),
        }
    }
}

/// Negotiate the shared protocol version: the highest version common to both
/// the client's `[client_min, client_max]` and the server's
/// `[server_min, server_max]` ranges.
///
/// Pure and oracle-testable. Returns [`OpsProtoError::VersionMismatch`] when
/// the ranges do not overlap, or [`OpsProtoError::Decode`] when the client
/// range is itself inverted (`min > max`).
pub fn negotiate_version(
    client_min: u16,
    client_max: u16,
    server_min: u16,
    server_max: u16,
) -> OpsProtoResult<u16> {
    if client_min > client_max {
        return Err(OpsProtoError::Decode(format!(
            "client offered an inverted version range {client_min}..={client_max}"
        )));
    }
    let low = client_min.max(server_min);
    let high = client_max.min(server_max);
    if low > high {
        return Err(OpsProtoError::VersionMismatch {
            client_min,
            client_max,
            server_min,
            server_max,
        });
    }
    Ok(high)
}
