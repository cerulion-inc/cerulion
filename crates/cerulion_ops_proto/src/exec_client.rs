// SPDX-License-Identifier: MIT OR Apache-2.0
//! Transport-independent, bounded collection of an argv-only exec session.
//!
//! [`ExecClient`] produces the existing exec [`Hello`] and child-stdin EOF,
//! then consumes decoded handshake and session frames. The caller owns
//! authentication, big-endian framing, concurrent reads/writes, timeouts,
//! cancellation and persistence of recording witnesses. This module performs
//! no I/O, reconnects or retries. A lost handshake reply can hide an accepted
//! command; it must not trigger an automatic replay.
//!
//! Peer error text is returned verbatim, as in [`crate::OpsClient`]. The caller
//! must sanitize and bound it before displaying or logging it. The caller must
//! also bound incoming frame lengths before allocation; use the session codecs
//! rather than the larger ordinary-ops frame limit.

use crate::session::encode_session;
use crate::{
    decode_data, ClientFrame, ExecRequest, HandshakeReply, Hello, OpsProtoError, OpsProtoResult,
    OutputStream, RecordingWitness, ServerFrame, SessionExit, SessionKind,
    MIN_SESSION_PROTOCOL_VERSION, PROTOCOL_VERSION,
};

/// Maximum decoded output bytes retained independently for each stream.
///
/// Zero is allowed and discards nonempty data while continuing to consume
/// frames. These are client capture limits, not the robot's delivery policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExecCaptureLimits {
    pub stdout_bytes: usize,
    pub stderr_bytes: usize,
}

/// A completed exec result, available only after a valid final Exit frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExecResult {
    pub stdout: Vec<u8>,
    pub stderr: Vec<u8>,
    /// True if either this client or the robot truncated stdout delivery.
    pub stdout_truncated: bool,
    /// True if either this client or the robot truncated stderr delivery.
    pub stderr_truncated: bool,
    /// Original robot status, reason and server-only truncation flags.
    pub exit: SessionExit,
    /// Final commitment bound to the Started frame's session identifier.
    ///
    /// The caller must retain this outside the robot. Receiving it does not
    /// itself verify the recording's contents or provide a download location.
    pub recording: RecordingWitness,
}

/// Snapshot of an incomplete session; it contains no final status.
///
/// Output and metadata are cloned, so changes to this value do not change the
/// client. After completion, output is empty because [`ExecEvent::Complete`]
/// transferred the buffers into [`ExecResult`]; retain that result instead.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExecPartial {
    pub session_id: Option<String>,
    pub stdout: Vec<u8>,
    pub stderr: Vec<u8>,
    /// Whether this client's capture limit discarded stdout bytes.
    pub stdout_truncated: bool,
    /// Whether this client's capture limit discarded stderr bytes.
    pub stderr_truncated: bool,
    /// Latest accepted live witness, not a claim of durable local persistence.
    pub checkpoint: Option<RecordingWitness>,
}

/// Observable progress from one validated server session frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExecEvent {
    /// Started, output, or an exact repeat of the latest checkpoint.
    Progress,
    /// A new live checkpoint which the caller may independently persist.
    Checkpoint(RecordingWitness),
    /// The sole completed result, transferring ownership of captured output.
    Complete(ExecResult),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Handshake,
    Started,
    Output,
    Complete,
    Failed,
}

/// Pure state machine for one argv-only exec invocation.
///
/// Call [`Self::accept_handshake`] once, send its [`ClientFrame::StdinEof`]
/// without finishing the protocol stream, then feed [`Self::accept_frame`].
/// Continue reading after capture limits are reached or explicitly cancel the
/// transport. Any error from either acceptance method makes this instance
/// terminal; subsequent acceptance calls fail without changing captured data.
///
/// Transport EOF, timeout and cancellation are caller-owned: stop feeding this
/// instance and close its stream. [`Self::partial`] can report previously
/// validated bytes and a checkpoint, but never fabricates a final result.
pub struct ExecClient {
    hello: Hello,
    limits: ExecCaptureLimits,
    phase: Phase,
    session_id: Option<String>,
    stdout: Vec<u8>,
    stderr: Vec<u8>,
    stdout_truncated: bool,
    stderr_truncated: bool,
    checkpoint: Option<RecordingWitness>,
}

impl ExecClient {
    /// Validate argv and the complete Hello against the session envelope cap.
    ///
    /// Arguments are preserved exactly; no shell parsing or interpolation
    /// occurs. Capture buffers grow only as accepted output arrives.
    pub fn new(argv: Vec<String>, limits: ExecCaptureLimits) -> OpsProtoResult<Self> {
        let hello = Hello {
            min_version: MIN_SESSION_PROTOCOL_VERSION,
            max_version: PROTOCOL_VERSION,
            session: SessionKind::Exec(ExecRequest { argv }),
        };
        hello.session.validate()?;
        encode_session(&hello)?;
        Ok(Self {
            hello,
            limits,
            phase: Phase::Handshake,
            session_id: None,
            stdout: Vec::new(),
            stderr: Vec::new(),
            stdout_truncated: false,
            stderr_truncated: false,
            checkpoint: None,
        })
    }

    /// The exec Hello to encode once on the authenticated OPS stream.
    pub fn hello(&self) -> &Hello {
        &self.hello
    }

    /// Accept exactly one supported Ack and produce child stdin EOF.
    ///
    /// Reject preserves the peer's version range and untrusted message in
    /// [`OpsProtoError::HandshakeRejected`]. No refusal triggers a retry.
    pub fn accept_handshake(&mut self, reply: HandshakeReply) -> OpsProtoResult<ClientFrame> {
        let phase = std::mem::replace(&mut self.phase, Phase::Failed);
        if phase != Phase::Handshake {
            return Err(invalid("exec handshake is not pending"));
        }
        match reply {
            HandshakeReply::Ack { chosen_version, .. }
                if chosen_version == PROTOCOL_VERSION
                    && (self.hello.min_version..=self.hello.max_version)
                        .contains(&chosen_version) =>
            {
                self.phase = Phase::Started;
                Ok(ClientFrame::StdinEof)
            }
            HandshakeReply::Ack { .. } => Err(invalid("unsupported exec protocol version")),
            HandshakeReply::Reject {
                server_min,
                server_max,
                message,
            } => Err(OpsProtoError::HandshakeRejected {
                reason: message,
                server_min,
                server_max,
            }),
        }
    }

    /// Validate and consume a frame without performing transport or file I/O.
    ///
    /// Captured bytes retain per-stream order. Exact repeated witnesses yield
    /// [`ExecEvent::Progress`]; new witnesses yield [`ExecEvent::Checkpoint`].
    /// A final witness must not move behind or contradict the last checkpoint.
    /// Witness consistency is not a verification of recording contents.
    ///
    /// [`ServerFrame::Error`] maps to [`OpsProtoError::Remote`] with the locally
    /// assigned kind `exec_session`. That kind is not a wire refusal code; the
    /// peer message remains untrusted and is returned verbatim.
    pub fn accept_frame(&mut self, frame: ServerFrame) -> OpsProtoResult<ExecEvent> {
        let phase = std::mem::replace(&mut self.phase, Phase::Failed);
        if !matches!(phase, Phase::Started | Phase::Output) {
            return Err(invalid("exec session is not accepting frames"));
        }
        frame.validate()?;
        match frame {
            ServerFrame::Error { message } => Err(OpsProtoError::Remote {
                kind: "exec_session".into(),
                message,
            }),
            ServerFrame::Started { session_id } => {
                if phase != Phase::Started {
                    return Err(invalid("repeated exec session start"));
                }
                self.session_id = Some(session_id);
                self.phase = Phase::Output;
                Ok(ExecEvent::Progress)
            }
            _ if phase != Phase::Output => Err(invalid("exec frame arrived before session start")),
            ServerFrame::Data { stream, data } => {
                let bytes = decode_data(&data)?;
                match stream {
                    OutputStream::Stdout => capture(
                        &mut self.stdout,
                        &mut self.stdout_truncated,
                        self.limits.stdout_bytes,
                        &bytes,
                    ),
                    OutputStream::Stderr => capture(
                        &mut self.stderr,
                        &mut self.stderr_truncated,
                        self.limits.stderr_bytes,
                        &bytes,
                    ),
                    OutputStream::Pty => return Err(invalid("PTY output is invalid for exec")),
                }
                self.phase = Phase::Output;
                Ok(ExecEvent::Progress)
            }
            ServerFrame::Witness { witness } => {
                self.check_witness(&witness)?;
                self.phase = Phase::Output;
                if self.checkpoint.as_ref() == Some(&witness) {
                    return Ok(ExecEvent::Progress);
                }
                self.checkpoint = Some(witness.clone());
                Ok(ExecEvent::Checkpoint(witness))
            }
            ServerFrame::Exit(exit) => {
                let recording = RecordingWitness {
                    session_id: self
                        .session_id
                        .as_ref()
                        .ok_or_else(|| invalid("exec exit has no session identity"))?
                        .clone(),
                    head: exit.recording_head.clone(),
                    events: exit.events,
                };
                self.check_witness(&recording)?;
                let result = ExecResult {
                    stdout: std::mem::take(&mut self.stdout),
                    stderr: std::mem::take(&mut self.stderr),
                    stdout_truncated: self.stdout_truncated || exit.stdout_truncated,
                    stderr_truncated: self.stderr_truncated || exit.stderr_truncated,
                    exit,
                    recording,
                };
                self.phase = Phase::Complete;
                Ok(ExecEvent::Complete(result))
            }
        }
    }

    /// Clone the last valid incomplete state, including after a protocol error.
    ///
    /// This does not persist or authenticate a recording. After Complete, the
    /// output buffers are empty; local truncation flags, the session id and the
    /// latest live checkpoint remain available, but no final status is supplied.
    pub fn partial(&self) -> ExecPartial {
        ExecPartial {
            session_id: self.session_id.clone(),
            stdout: self.stdout.clone(),
            stderr: self.stderr.clone(),
            stdout_truncated: self.stdout_truncated,
            stderr_truncated: self.stderr_truncated,
            checkpoint: self.checkpoint.clone(),
        }
    }

    fn check_witness(&self, witness: &RecordingWitness) -> OpsProtoResult<()> {
        if self.session_id.as_deref() != Some(witness.session_id.as_str()) {
            return Err(invalid("exec witness belongs to another session"));
        }
        if let Some(previous) = &self.checkpoint {
            if witness.events < previous.events
                || (witness.events == previous.events && witness.head != previous.head)
            {
                return Err(invalid("exec witness moved backwards or changed its head"));
            }
        }
        Ok(())
    }
}

fn capture(buffer: &mut Vec<u8>, truncated: &mut bool, limit: usize, bytes: &[u8]) {
    let retained = limit.saturating_sub(buffer.len()).min(bytes.len());
    buffer.extend_from_slice(&bytes[..retained]);
    *truncated |= retained < bytes.len();
}

fn invalid(message: &str) -> OpsProtoError {
    OpsProtoError::Decode(message.into())
}
