// SPDX-License-Identifier: MIT OR Apache-2.0
//! Protocol version 2 command sessions. Data is standard padded base64, with
//! separate caps on the JSON envelope and the decoded byte chunk.

use std::io::{Read, Write};

use base64::engine::general_purpose::STANDARD;
use base64::Engine;
use serde::{Deserialize, Serialize};

use crate::{OpsProtoError, OpsProtoResult, RecordingWitness};

/// Maximum JSON payload in a shell or exec frame (1 MiB).
pub const MAX_SESSION_FRAME_BYTES: usize = 1024 * 1024;
/// Maximum decoded bytes in one stdin or output event (64 KiB).
pub const MAX_DATA_BYTES: usize = 64 * 1024;

/// The connection's purpose, selected before any process can be spawned.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum SessionKind {
    /// Ordinary request/response ops.
    #[default]
    Ops,
    /// A login shell attached to a pseudoterminal.
    Shell(ShellRequest),
    /// A program invoked directly, without shell interpolation.
    Exec(ExecRequest),
}

impl<'de> Deserialize<'de> for SessionKind {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        // Serde ignores extra fields on internally tagged unit variants, even
        // with deny_unknown_fields. Empty struct variants enforce the contract.
        #[derive(Deserialize)]
        #[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
        enum Wire {
            Ops {},
            Shell(ShellRequest),
            Exec(ExecRequest),
        }
        Ok(match Wire::deserialize(deserializer)? {
            Wire::Ops {} => Self::Ops,
            Wire::Shell(request) => Self::Shell(request),
            Wire::Exec(request) => Self::Exec(request),
        })
    }
}

impl SessionKind {
    /// The authorization verb, or `None` for ordinary ops.
    pub fn verb(&self) -> Option<&'static str> {
        match self {
            Self::Ops => None,
            Self::Shell(_) => Some("shell"),
            Self::Exec(_) => Some("exec"),
        }
    }

    /// Reject malformed process or terminal parameters.
    pub fn validate(&self) -> OpsProtoResult<()> {
        match self {
            Self::Ops => Ok(()),
            Self::Shell(request) => {
                request.size.validate()?;
                if request.term.is_empty()
                    || request.term.len() > 64
                    || !request
                        .term
                        .bytes()
                        .all(|b| b.is_ascii_alphanumeric() || b"._+-".contains(&b))
                {
                    return Err(invalid("invalid terminal type"));
                }
                Ok(())
            }
            Self::Exec(request) => {
                if request.argv.is_empty()
                    || request.argv[0].is_empty()
                    || request.argv.iter().any(|arg| arg.contains('\0'))
                {
                    return Err(invalid(
                        "exec requires a nonempty program and NUL-free arguments",
                    ));
                }
                // Bound metadata before opening a recording or spawning a child.
                encode_session(self).map(|_| ())
            }
        }
    }
}

/// PTY startup parameters.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ShellRequest {
    pub term: String,
    pub size: WindowSize,
}

/// Direct argv execution; no environment or working-directory override.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExecRequest {
    pub argv: Vec<String>,
}

/// Terminal geometry. Pixel dimensions may be zero when unknown.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WindowSize {
    pub rows: u16,
    pub cols: u16,
    pub pixel_width: u16,
    pub pixel_height: u16,
}

impl WindowSize {
    /// Terminal row and column counts must both be nonzero.
    pub fn validate(&self) -> OpsProtoResult<()> {
        if self.rows == 0 || self.cols == 0 {
            return Err(invalid("terminal rows and columns must be nonzero"));
        }
        Ok(())
    }
}

/// Frames from the desk after the session handshake.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum ClientFrame {
    Stdin { data: String },
    StdinEof,
    Resize { size: WindowSize },
    Close,
}

impl<'de> Deserialize<'de> for ClientFrame {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        #[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
        enum Wire {
            Stdin { data: String },
            StdinEof {},
            Resize { size: WindowSize },
            Close {},
        }
        Ok(match Wire::deserialize(deserializer)? {
            Wire::Stdin { data } => Self::Stdin { data },
            Wire::StdinEof {} => Self::StdinEof,
            Wire::Resize { size } => Self::Resize { size },
            Wire::Close {} => Self::Close,
        })
    }
}

impl ClientFrame {
    /// Validate a decoded message before applying it to a process.
    pub fn validate(&self) -> OpsProtoResult<()> {
        match self {
            Self::Stdin { data } => decode_data(data).map(|_| ()),
            Self::Resize { size } => size.validate(),
            Self::StdinEof | Self::Close => Ok(()),
        }
    }
}

/// Output stream identity; PTY output has one combined byte stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputStream {
    Stdout,
    Stderr,
    Pty,
}

/// Why the child session ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExitReason {
    Exited,
    ClientClosed,
    Disconnected,
    Revoked,
    ProtocolError,
    RecordingFailed,
    CleanupFailed,
    SpawnFailed,
    TimedOut,
    ResourceLimit,
}

/// Final process status and commitment to the complete on-robot recording.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SessionExit {
    pub code: Option<i32>,
    pub signal: Option<i32>,
    pub reason: ExitReason,
    pub recording_head: String,
    pub events: u64,
    pub stdout_truncated: bool,
    pub stderr_truncated: bool,
}

/// Frames sent by the robot after the session handshake.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum ServerFrame {
    Started {
        session_id: String,
    },
    Data {
        stream: OutputStream,
        data: String,
    },
    /// Committed recording prefix, retained independently by the client.
    Witness {
        witness: RecordingWitness,
    },
    Exit(SessionExit),
    Error {
        message: String,
    },
}

impl ServerFrame {
    /// Validate a decoded frame before displaying bytes or trusting a witness.
    pub fn validate(&self) -> OpsProtoResult<()> {
        match self {
            Self::Started { session_id } if session_id.is_empty() => {
                Err(invalid("empty session id"))
            }
            Self::Data { data, .. } => decode_data(data).map(|_| ()),
            Self::Witness { witness } => {
                if witness.session_id.is_empty()
                    || !valid_hash(&witness.head)
                    || witness.events == 0
                {
                    return Err(invalid("invalid recording witness"));
                }
                Ok(())
            }
            Self::Exit(exit) => {
                if !valid_hash(&exit.recording_head) || exit.events < 2 {
                    return Err(invalid("invalid recording witness"));
                }
                if exit.code.is_some() && exit.signal.is_some() {
                    return Err(invalid("exit cannot have both a code and a signal"));
                }
                Ok(())
            }
            Self::Started { .. } | Self::Error { .. } => Ok(()),
        }
    }
}

/// Encode one bounded binary chunk as standard padded base64.
pub fn encode_data(bytes: &[u8]) -> OpsProtoResult<String> {
    check_size(bytes.len(), MAX_DATA_BYTES)?;
    Ok(STANDARD.encode(bytes))
}

/// Decode a bounded canonical base64 chunk, rejecting malformed data.
pub fn decode_data(data: &str) -> OpsProtoResult<Vec<u8>> {
    check_size(data.len(), MAX_DATA_BYTES.div_ceil(3) * 4)?;
    let bytes = STANDARD
        .decode(data)
        .map_err(|_| invalid("invalid session base64 data"))?;
    check_size(bytes.len(), MAX_DATA_BYTES)?;
    Ok(bytes)
}

/// Serialize an envelope with the session frame size cap.
pub fn encode_session<T: Serialize>(value: &T) -> OpsProtoResult<Vec<u8>> {
    let bytes = serde_json::to_vec(value)?;
    check_size(bytes.len(), MAX_SESSION_FRAME_BYTES)?;
    Ok(bytes)
}

/// Decode and validate a client session frame.
pub fn decode_client_frame(bytes: &[u8]) -> OpsProtoResult<ClientFrame> {
    check_size(bytes.len(), MAX_SESSION_FRAME_BYTES)?;
    let frame: ClientFrame = crate::decode(bytes)?;
    frame.validate()?;
    Ok(frame)
}

/// Decode and validate a server session frame.
pub fn decode_server_frame(bytes: &[u8]) -> OpsProtoResult<ServerFrame> {
    check_size(bytes.len(), MAX_SESSION_FRAME_BYTES)?;
    let frame: ServerFrame = crate::decode(bytes)?;
    frame.validate()?;
    Ok(frame)
}

/// Read a session frame, refusing oversized lengths before allocation.
pub fn read_session_frame<R: Read>(reader: &mut R) -> OpsProtoResult<Vec<u8>> {
    let mut prefix = [0; 4];
    match reader.read_exact(&mut prefix[..1]) {
        Ok(()) => reader.read_exact(&mut prefix[1..])?,
        Err(error) if error.kind() == std::io::ErrorKind::UnexpectedEof => {
            return Err(OpsProtoError::ConnectionClosed);
        }
        Err(error) => return Err(error.into()),
    }
    let length = u32::from_be_bytes(prefix) as usize;
    check_size(length, MAX_SESSION_FRAME_BYTES)?;
    let mut bytes = vec![0; length];
    reader.read_exact(&mut bytes)?;
    Ok(bytes)
}

/// Write one bounded session payload using the shared big-endian framing.
pub fn write_session_frame<W: Write>(writer: &mut W, bytes: &[u8]) -> OpsProtoResult<()> {
    check_size(bytes.len(), MAX_SESSION_FRAME_BYTES)?;
    crate::write_frame(writer, bytes)
}

pub(crate) fn valid_hash(hash: &str) -> bool {
    hash.len() == 64
        && hash
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

fn check_size(size: usize, max: usize) -> OpsProtoResult<()> {
    if size > max {
        return Err(OpsProtoError::FrameTooLarge { size, max });
    }
    Ok(())
}

fn invalid(message: &str) -> OpsProtoError {
    OpsProtoError::Decode(message.to_owned())
}
