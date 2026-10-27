// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerud` wire protocol: length-prefixed frames, a connect-time version
//! negotiation, and the request/response envelopes.
//!
//! The wire types and constants live in the permissive `cerulion_ops_proto`
//! crate and are re-exported here at their `cerud::protocol` paths. The five
//! functions below call the `cerulion_ops_proto` function of the same name
//! and convert its [`cerulion_ops_proto::OpsProtoError`] into the
//! [`crate::CerudError`] variant of the same name, so a `cerud` caller keeps
//! matching on [`crate::CerudError`].
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
use serde::Serialize;

use crate::error::CerudResult;

pub use cerulion_ops_proto::protocol::{
    HandshakeReply, Hello, Request, Response, MAX_FRAME_BYTES, MIN_PROTOCOL_VERSION,
    PROTOCOL_VERSION,
};

/// Write one length-prefixed frame and flush.
pub fn write_frame<W: Write>(w: &mut W, payload: &[u8]) -> CerudResult<()> {
    Ok(cerulion_ops_proto::protocol::write_frame(w, payload)?)
}

/// Read one length-prefixed frame.
///
/// A clean EOF at the length prefix (nothing more from the peer) is reported
/// as [`crate::CerudError::ConnectionClosed`]; a truncated frame mid-payload is a
/// genuine I/O error.
pub fn read_frame<R: Read>(r: &mut R) -> CerudResult<Vec<u8>> {
    Ok(cerulion_ops_proto::protocol::read_frame(r)?)
}

/// Serialize a message to a JSON frame payload.
pub fn encode<T: Serialize>(value: &T) -> CerudResult<Vec<u8>> {
    Ok(cerulion_ops_proto::protocol::encode(value)?)
}

/// Deserialize a JSON frame payload into a message.
pub fn decode<T: DeserializeOwned>(bytes: &[u8]) -> CerudResult<T> {
    Ok(cerulion_ops_proto::protocol::decode(bytes)?)
}

/// Negotiate the shared protocol version: the highest version common to both
/// the client's `[client_min, client_max]` and the server's
/// `[server_min, server_max]` ranges.
///
/// Pure and oracle-testable. Returns [`crate::CerudError::VersionMismatch`] when the
/// ranges do not overlap, or [`crate::CerudError::Decode`] when the client range is
/// itself inverted (`min > max`).
pub fn negotiate_version(
    client_min: u16,
    client_max: u16,
    server_min: u16,
    server_max: u16,
) -> CerudResult<u16> {
    Ok(cerulion_ops_proto::protocol::negotiate_version(
        client_min, client_max, server_min, server_max,
    )?)
}
