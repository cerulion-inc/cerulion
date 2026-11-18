// SPDX-License-Identifier: MIT OR Apache-2.0
//! `cerulion_ops_proto`: the `cerulion/ops/1` wire protocol, permissive-licensed.
//!
//! These are the types and functions both ends of an ops connection share:
//!
//! - [`protocol`]: length-prefixed frames (a big-endian `u32` length, then a
//!   JSON payload), the connect-time version handshake ([`Hello`],
//!   [`HandshakeReply`], [`negotiate_version`]) and the verb envelopes
//!   ([`Request`], [`Response`]).
//! - [`client`]: [`OpsClient`], a blocking client that performs the handshake
//!   and issues verb calls over any `Read + Write` byte stream.
//! - [`inventory`]: [`Inventory`], the result the `inventory` verb returns.
//! - [`session`]: bounded shell and exec frame codecs, with separate limits on
//!   JSON envelopes and binary chunks. These codecs require session negotiation
//!   by their caller; [`OpsClient`] issues ordinary verb calls only.
//! - [`recording`]: hash-chained session events, [`RecordingWitness`] commitments
//!   and complete or prefix verification against an independently retained witness.
//! - [`error`]: [`OpsProtoError`], the errors the functions above return.
//!
//! The robot-side service `cerud` (AGPL-3.0-only) depends on this crate. It
//! re-exports the types and constants at their former `cerud::protocol` and
//! `cerud::verbs::inventory` paths, and its `cerud::protocol` functions and
//! `cerud::client::OpsClient` call the ones here and convert
//! [`OpsProtoError`] into `cerud::CerudError`, so the bytes `cerud` writes are
//! the bytes this crate writes. `tests/wire_golden_test.rs` pins them to golden frames.
//!
//! No transport, no I/O beyond the stream the caller hands in, and no
//! dependency on any AGPL crate.

// Library code never prints. Scoped `not(test)` so unit tests keep printing
// diagnostics, mirroring `cerud`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod client;
pub mod error;
pub mod inventory;
pub mod protocol;
pub mod recording;
pub mod session;

pub use client::OpsClient;
pub use error::{ErrorKind, OpsProtoError, OpsProtoResult};
pub use inventory::Inventory;
pub use protocol::{
    decode, encode, negotiate_version, read_frame, write_frame, HandshakeReply, Hello, Request,
    Response, MAX_FRAME_BYTES, MIN_PROTOCOL_VERSION, PROTOCOL_VERSION,
};
pub use recording::{
    verify_recording, verify_recording_prefix, RecordingChain, RecordingEvent, RecordingPayload,
    RecordingWitness,
};
pub use session::{
    decode_data, encode_data, ClientFrame, ExecRequest, ExitReason, OutputStream, ServerFrame,
    SessionExit, SessionKind, ShellRequest, WindowSize, MAX_DATA_BYTES, MAX_SESSION_FRAME_BYTES,
};
