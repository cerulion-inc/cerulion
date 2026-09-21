// SPDX-License-Identifier: AGPL-3.0-only
//! A minimal ops client: performs the version handshake and issues verb calls
//! over any framed byte stream.
//!
//! This is the client half of [`crate::protocol`] — used by the end-to-end
//! tests and any dev tooling. Production callers (the app/cloud side) speak
//! the same protocol over the iroh transport.

use std::io::{Read, Write};

use crate::error::{CerudError, CerudResult};
use crate::protocol::{self, HandshakeReply, Hello, Request, Response};

/// A connected ops client over a framed byte stream `S`.
pub struct OpsClient<S: Read + Write> {
    stream: S,
    negotiated_version: u16,
    server_name: String,
    next_id: u64,
}

impl<S: Read + Write> OpsClient<S> {
    /// Handshake over `stream`, advertising the given inclusive version range.
    /// Returns the connected client, or [`CerudError::HandshakeRejected`] if
    /// the server refuses the connection. The wire `Reject` reason is surfaced
    /// verbatim — version non-overlap is the cause today, but a non-version
    /// refusal (e.g. pairing bootstrap, or a malformed client range) is
    /// reported faithfully rather than mislabeled a version mismatch.
    pub fn connect(mut stream: S, min_version: u16, max_version: u16) -> CerudResult<Self> {
        let hello = Hello {
            min_version,
            max_version,
        };
        let payload = protocol::encode(&hello)?;
        protocol::write_frame(&mut stream, &payload)?;

        let frame = protocol::read_frame(&mut stream)?;
        let reply: HandshakeReply = protocol::decode(&frame)?;
        match reply {
            HandshakeReply::Ack {
                chosen_version,
                server_name,
            } => Ok(OpsClient {
                stream,
                negotiated_version: chosen_version,
                server_name,
                next_id: 0,
            }),
            HandshakeReply::Reject {
                server_min,
                server_max,
                message,
            } => Err(CerudError::HandshakeRejected {
                reason: message,
                server_min,
                server_max,
            }),
        }
    }

    /// Handshake advertising this build's supported range.
    pub fn connect_current(stream: S) -> CerudResult<Self> {
        let hello = Hello::current();
        Self::connect(stream, hello.min_version, hello.max_version)
    }

    /// The protocol version both sides agreed on.
    pub fn negotiated_version(&self) -> u16 {
        self.negotiated_version
    }

    /// The server's self-reported name.
    pub fn server_name(&self) -> &str {
        &self.server_name
    }

    /// Call `verb` with `args`, returning its JSON result. A server-side
    /// refusal/failure surfaces as [`CerudError::Remote`] carrying the
    /// server's error `kind` + message.
    pub fn call(&mut self, verb: &str, args: serde_json::Value) -> CerudResult<serde_json::Value> {
        let id = self.next_id;
        self.next_id += 1;
        let request = Request {
            id,
            verb: verb.to_string(),
            args,
        };
        let payload = protocol::encode(&request)?;
        protocol::write_frame(&mut self.stream, &payload)?;

        let frame = protocol::read_frame(&mut self.stream)?;
        let response: Response = protocol::decode(&frame)?;
        match response {
            Response::Ok {
                id: resp_id,
                result,
            } => {
                if resp_id != id {
                    return Err(CerudError::Decode(format!(
                        "response id {resp_id} does not match request id {id}"
                    )));
                }
                Ok(result)
            }
            Response::Err { kind, message, .. } => Err(CerudError::Remote { kind, message }),
        }
    }
}
