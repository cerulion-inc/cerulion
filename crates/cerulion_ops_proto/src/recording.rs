// SPDX-License-Identifier: MIT OR Apache-2.0
//! Hash commitments for shell and exec audit recordings. Verification requires
//! a witness retained outside the robot: an unkeyed chain alone cannot detect
//! a replacement recomputed by the uid that owns the recording.
//!
//! The hash preimage is `b"cerulion-session-recording-v1\0"` followed by compact
//! UTF-8 JSON with fields ordered `sequence`, `timestamp_micros`, `payload`,
//! `previous_hash`. Payload field order is the declaration order below; strings
//! use serde_json's escaping, integers are decimal, and hashes are lowercase hex.
//! The first previous hash is 64 zeroes. The Exit event does not contain its own
//! hash, avoiding a circular commitment. Recordings retain input/output up to
//! the recording limit, including output beyond transport delivery limits.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::session::{decode_data, valid_hash};
use crate::{ExitReason, OpsProtoError, OpsProtoResult, OutputStream, SessionKind, WindowSize};

const GENESIS: &str = "0000000000000000000000000000000000000000000000000000000000000000";
const DOMAIN: &[u8] = b"cerulion-session-recording-v1\0";

/// Audit event contents; binary data uses the same encoding as session frames.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum RecordingPayload {
    Started {
        session_id: String,
        principal_id: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        account_id: Option<String>,
        session: SessionKind,
    },
    Stdin {
        data: String,
    },
    StdinEof,
    Output {
        stream: OutputStream,
        data: String,
    },
    Resize {
        size: WindowSize,
    },
    Exit {
        code: Option<i32>,
        signal: Option<i32>,
        reason: ExitReason,
    },
}

impl<'de> Deserialize<'de> for RecordingPayload {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        // Keep empty payloads strict too: serde's unit variant visitor otherwise
        // discards extra fields that would be absent from the hash preimage.
        #[derive(Deserialize)]
        #[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
        enum Wire {
            Started {
                session_id: String,
                principal_id: String,
                #[serde(default)]
                account_id: Option<String>,
                session: SessionKind,
            },
            Stdin {
                data: String,
            },
            StdinEof {},
            Output {
                stream: OutputStream,
                data: String,
            },
            Resize {
                size: WindowSize,
            },
            Exit {
                code: Option<i32>,
                signal: Option<i32>,
                reason: ExitReason,
            },
        }
        Ok(match Wire::deserialize(deserializer)? {
            Wire::Started {
                session_id,
                principal_id,
                account_id,
                session,
            } => Self::Started {
                session_id,
                principal_id,
                account_id,
                session,
            },
            Wire::Stdin { data } => Self::Stdin { data },
            Wire::StdinEof {} => Self::StdinEof,
            Wire::Output { stream, data } => Self::Output { stream, data },
            Wire::Resize { size } => Self::Resize { size },
            Wire::Exit {
                code,
                signal,
                reason,
            } => Self::Exit {
                code,
                signal,
                reason,
            },
        })
    }
}

/// One append-only audit entry. Timestamps are robot microseconds: the robot
/// clock at session start plus monotonic elapsed time, so wall-clock adjustments
/// cannot move an active recording backwards.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordingEvent {
    pub sequence: u64,
    pub timestamp_micros: u64,
    pub payload: RecordingPayload,
    pub previous_hash: String,
    pub hash: String,
}

/// The session commitment stored by the client outside the robot's control.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordingWitness {
    pub session_id: String,
    pub head: String,
    pub events: u64,
}

/// Incremental hash-chain builder. Write each event before forwarding its data;
/// publish its witness only after a durable commit. Recording failures terminate
/// the live session.
#[derive(Debug, Clone)]
pub struct RecordingChain {
    head: String,
    events: u64,
    last_timestamp: u64,
    ended: bool,
}

impl Default for RecordingChain {
    fn default() -> Self {
        Self::new()
    }
}

impl RecordingChain {
    /// Begin an empty chain; its first event must be Started.
    pub fn new() -> Self {
        Self {
            head: GENESIS.to_owned(),
            events: 0,
            last_timestamp: 0,
            ended: false,
        }
    }

    /// Commit to a validated event, preserving timestamp and event order.
    pub fn append(
        &mut self,
        timestamp_micros: u64,
        payload: RecordingPayload,
    ) -> OpsProtoResult<RecordingEvent> {
        if self.ended {
            return Err(invalid("recording already ended"));
        }
        if timestamp_micros < self.last_timestamp {
            return Err(invalid("recording timestamp moved backwards"));
        }
        if matches!(payload, RecordingPayload::Started { .. }) != (self.events == 0) {
            return Err(invalid(
                "recording must begin with exactly one started event",
            ));
        }
        validate_payload(&payload)?;
        let next_events = self
            .events
            .checked_add(1)
            .ok_or_else(|| invalid("recording event count overflow"))?;
        let mut event = RecordingEvent {
            sequence: self.events,
            timestamp_micros,
            payload,
            previous_hash: self.head.clone(),
            hash: String::new(),
        };
        event.hash = event_hash(&event)?;
        self.ended = matches!(event.payload, RecordingPayload::Exit { .. });
        self.head.clone_from(&event.hash);
        self.events = next_events;
        self.last_timestamp = timestamp_micros;
        Ok(event)
    }

    /// Current lowercase SHA-256 head, including the most recently appended event.
    pub fn head(&self) -> &str {
        &self.head
    }

    /// Number of appended events, including Started and Exit.
    pub fn events(&self) -> u64 {
        self.events
    }
}

/// Verify a complete recording against its required off-robot witness.
///
/// A locally self-consistent chain is insufficient: the final hash, count and
/// session identity must all equal the desk's independently retained values.
pub fn verify_recording(
    events: &[RecordingEvent],
    expected: &RecordingWitness,
) -> OpsProtoResult<()> {
    let chain = verify_chain(events, expected)?;
    if !chain.ended {
        return Err(invalid("recording has no final exit event"));
    }
    Ok(())
}

/// Verify exactly the prefix covered by a live off-robot witness.
///
/// The prefix must begin with Started and match the witness's session, count and
/// hash. It need not end with Exit. Callers must supply only the witnessed prefix;
/// any later events or incomplete bytes remain unverified. Use [`verify_recording`]
/// when a complete recording is required.
pub fn verify_recording_prefix(
    events: &[RecordingEvent],
    expected: &RecordingWitness,
) -> OpsProtoResult<()> {
    verify_chain(events, expected).map(|_| ())
}

fn verify_chain(
    events: &[RecordingEvent],
    expected: &RecordingWitness,
) -> OpsProtoResult<RecordingChain> {
    if expected.session_id.is_empty() || !valid_hash(&expected.head) || expected.events == 0 {
        return Err(invalid("invalid off-robot recording witness"));
    }
    let mut chain = RecordingChain::new();
    for event in events {
        if event.sequence != chain.events || event.previous_hash != chain.head {
            return Err(invalid(
                "recording event sequence or previous hash mismatch",
            ));
        }
        if let RecordingPayload::Started { session_id, .. } = &event.payload {
            if session_id != &expected.session_id {
                return Err(invalid(
                    "recording session does not match off-robot witness",
                ));
            }
        }
        let recomputed = chain.append(event.timestamp_micros, event.payload.clone())?;
        if recomputed.hash != event.hash {
            return Err(invalid("recording event hash mismatch"));
        }
    }
    if chain.head != expected.head || chain.events != expected.events {
        return Err(invalid("recording does not match off-robot witness"));
    }
    Ok(chain)
}

fn validate_payload(payload: &RecordingPayload) -> OpsProtoResult<()> {
    match payload {
        RecordingPayload::Started {
            session_id,
            principal_id,
            session,
            ..
        } => {
            if session_id.is_empty() || principal_id.is_empty() || session.verb().is_none() {
                return Err(invalid(
                    "recording requires session and principal identity and a command session",
                ));
            }
            session.validate()
        }
        RecordingPayload::Stdin { data } | RecordingPayload::Output { data, .. } => {
            decode_data(data).map(|_| ())
        }
        RecordingPayload::Resize { size } => size.validate(),
        RecordingPayload::Exit {
            code: Some(_),
            signal: Some(_),
            ..
        } => Err(invalid("exit cannot have both a code and a signal")),
        RecordingPayload::StdinEof | RecordingPayload::Exit { .. } => Ok(()),
    }
}

fn event_hash(event: &RecordingEvent) -> OpsProtoResult<String> {
    #[derive(Serialize)]
    struct Preimage<'a> {
        sequence: u64,
        timestamp_micros: u64,
        payload: &'a RecordingPayload,
        previous_hash: &'a str,
    }
    let bytes = serde_json::to_vec(&Preimage {
        sequence: event.sequence,
        timestamp_micros: event.timestamp_micros,
        payload: &event.payload,
        previous_hash: &event.previous_hash,
    })?;
    let mut digest = Sha256::new();
    digest.update(DOMAIN);
    digest.update(bytes);
    Ok(format!("{:x}", digest.finalize()))
}

fn invalid(message: &str) -> OpsProtoError {
    OpsProtoError::Decode(message.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_count_overflow_leaves_the_chain_unchanged() {
        let mut chain = RecordingChain {
            head: GENESIS.into(),
            events: u64::MAX,
            last_timestamp: 10,
            ended: false,
        };
        assert!(matches!(
            chain.append(11, RecordingPayload::StdinEof),
            Err(OpsProtoError::Decode(message)) if message == "recording event count overflow"
        ));
        assert_eq!(chain.head(), GENESIS);
        assert_eq!(chain.events(), u64::MAX);
        assert_eq!(chain.last_timestamp, 10);
        assert!(!chain.ended);
    }
}
