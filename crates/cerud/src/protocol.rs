// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerud` wire protocol: length-prefixed frames, a connect-time version
//! negotiation, and the request/response envelopes.
//!
//! The protocol is layered ON TOP of any "framed bidirectional bytes" stream
//! (see [`crate::transport`]), so every transport — the Unix-domain-socket dev
//! transport today, iroh later — shares one framing + one handshake.
//!
//! # Framing
//!
//! Each frame is a big-endian `u32` length prefix followed by that many
//! payload bytes. Payloads are JSON. A hard [`MAX_FRAME_BYTES`] cap bounds a
//! hostile/garbled length prefix.
//!
//! # Handshake (version negotiation, day one)
//!
//! On connect the client sends a [`Hello`] carrying the inclusive version
//! range it supports. The server replies with [`HandshakeReply::Ack`] naming
//! the chosen version (the highest common) or [`HandshakeReply::Reject`] if
//! there is no overlap — refused before any verb runs.

use std::io::{Read, Write};

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::error::{CerudError, CerudResult};

/// The newest protocol version this build speaks.
pub const PROTOCOL_VERSION: u16 = 1;

/// The oldest protocol version this build still accepts.
pub const MIN_PROTOCOL_VERSION: u16 = 1;

/// Hard cap on a single frame's payload (16 MiB). A declared length above this
/// is rejected without allocating the buffer.
pub const MAX_FRAME_BYTES: usize = 16 * 1024 * 1024;

/// Write one length-prefixed frame and flush.
pub fn write_frame<W: Write>(w: &mut W, payload: &[u8]) -> CerudResult<()> {
    if payload.len() > MAX_FRAME_BYTES {
        return Err(CerudError::FrameTooLarge {
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
/// as [`CerudError::ConnectionClosed`]; a truncated frame mid-payload is a
/// genuine I/O error.
pub fn read_frame<R: Read>(r: &mut R) -> CerudResult<Vec<u8>> {
    let mut len_buf = [0u8; 4];
    match r.read_exact(&mut len_buf) {
        Ok(()) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
            return Err(CerudError::ConnectionClosed);
        }
        Err(e) => return Err(e.into()),
    }
    let len = u32::from_be_bytes(len_buf) as usize;
    if len > MAX_FRAME_BYTES {
        return Err(CerudError::FrameTooLarge {
            size: len,
            max: MAX_FRAME_BYTES,
        });
    }
    let mut buf = vec![0u8; len];
    r.read_exact(&mut buf)?; // a mid-payload EOF is a truncated-frame I/O error
    Ok(buf)
}

/// Serialize a message to a JSON frame payload.
pub fn encode<T: Serialize>(value: &T) -> CerudResult<Vec<u8>> {
    Ok(serde_json::to_vec(value)?)
}

/// Deserialize a JSON frame payload into a message.
pub fn decode<T: DeserializeOwned>(bytes: &[u8]) -> CerudResult<T> {
    serde_json::from_slice(bytes).map_err(|e| CerudError::Decode(e.to_string()))
}

/// The client's opening message: the inclusive protocol-version range it
/// supports.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Hello {
    pub min_version: u16,
    pub max_version: u16,
}

impl Hello {
    /// A Hello advertising exactly this build's supported range.
    pub fn current() -> Self {
        Hello {
            min_version: MIN_PROTOCOL_VERSION,
            max_version: PROTOCOL_VERSION,
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
    /// Build an error response from a request id + a [`CerudError`].
    pub fn from_error(id: u64, err: &CerudError) -> Self {
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
/// Pure and oracle-testable. Returns [`CerudError::VersionMismatch`] when the
/// ranges do not overlap, or [`CerudError::Decode`] when the client range is
/// itself inverted (`min > max`).
pub fn negotiate_version(
    client_min: u16,
    client_max: u16,
    server_min: u16,
    server_max: u16,
) -> CerudResult<u16> {
    if client_min > client_max {
        return Err(CerudError::Decode(format!(
            "client offered an inverted version range {client_min}..={client_max}"
        )));
    }
    let low = client_min.max(server_min);
    let high = client_max.min(server_max);
    if low > high {
        return Err(CerudError::VersionMismatch {
            client_min,
            client_max,
            server_min,
            server_max,
        });
    }
    Ok(high)
}
