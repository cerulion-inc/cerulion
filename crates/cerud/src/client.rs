// SPDX-License-Identifier: AGPL-3.0-only
//! A minimal ops client: performs the version handshake and issues verb calls
//! over any framed byte stream.
//!
//! This is the client half of [`crate::protocol`] — used by the end-to-end
//! tests and any dev tooling. Production callers (the app/cloud side) speak
//! the same protocol over the iroh transport.
//!
//! [`OpsClient`] wraps [`cerulion_ops_proto::OpsClient`] and converts each
//! [`cerulion_ops_proto::OpsProtoError`] into the [`crate::CerudError`] variant of
//! the same name.

use std::io::{Read, Write};

use crate::error::CerudResult;

/// A connected ops client over a framed byte stream `S`.
pub struct OpsClient<S: Read + Write> {
    inner: cerulion_ops_proto::OpsClient<S>,
}

impl<S: Read + Write> OpsClient<S> {
    /// Handshake over `stream`, advertising the given inclusive version range.
    /// Returns the connected client, or [`crate::CerudError::HandshakeRejected`] if
    /// the server refuses the connection. The wire `Reject` reason is surfaced
    /// verbatim — version non-overlap is the cause today, but a non-version
    /// refusal (e.g. pairing bootstrap, or a malformed client range) is
    /// reported faithfully rather than mislabeled a version mismatch.
    pub fn connect(stream: S, min_version: u16, max_version: u16) -> CerudResult<Self> {
        let inner = cerulion_ops_proto::OpsClient::connect(stream, min_version, max_version)?;
        Ok(OpsClient { inner })
    }

    /// Handshake advertising this build's supported range.
    pub fn connect_current(stream: S) -> CerudResult<Self> {
        let inner = cerulion_ops_proto::OpsClient::connect_current(stream)?;
        Ok(OpsClient { inner })
    }

    /// The protocol version both sides agreed on.
    pub fn negotiated_version(&self) -> u16 {
        self.inner.negotiated_version()
    }

    /// The server's self-reported name.
    pub fn server_name(&self) -> &str {
        self.inner.server_name()
    }

    /// Call `verb` with `args`, returning its JSON result. A server-side
    /// refusal/failure surfaces as [`crate::CerudError::Remote`] carrying the
    /// server's error `kind` + message.
    pub fn call(&mut self, verb: &str, args: serde_json::Value) -> CerudResult<serde_json::Value> {
        Ok(self.inner.call(verb, args)?)
    }
}
