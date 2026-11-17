// SPDX-License-Identifier: MIT OR Apache-2.0
//! Commitments to a session recording, retained by the client independently of
//! the recording's storage. A commitment contains the session identity, chain
//! head and event count; accepting its shape alone does not verify a recording.

use serde::{Deserialize, Serialize};

/// The session commitment stored by the client outside the robot's control.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordingWitness {
    pub session_id: String,
    pub head: String,
    pub events: u64,
}
