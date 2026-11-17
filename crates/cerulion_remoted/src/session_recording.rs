// SPDX-License-Identifier: AGPL-3.0-only
//! Private append-only session audit files. These JSONL files are not MCAP bags.

use std::ffi::OsStr;
use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::os::unix::fs::{DirBuilderExt, OpenOptionsExt, PermissionsExt};
use std::path::Path;
use std::sync::Arc;

use cerulion_ops_proto::recording::{RecordingChain, RecordingPayload, RecordingWitness};

use crate::remote_session::MAX_RECORDING_EVENT_BYTES;

// Maximum serialized Exit line, including its newline: u64 sequence/timestamp,
// an i32 status, the longest reason, and two 64-byte hex hashes. Independent
// wire-vector tests pin every reason and status shape against this bound.
const EXIT_RESERVE_BYTES: u64 = 318;

/// Whether an event was appended or refused to preserve the terminal-event budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AppendOutcome {
    /// The bytes were written and the in-memory commitment advanced.
    Appended,
    /// No bytes or commitment changed; room remains reserved for an Exit event.
    LimitReached,
}

fn exceeds_recording_limit(total: u64, cap: u64, exit: bool) -> bool {
    let limit = if exit {
        cap
    } else {
        cap.saturating_sub(EXIT_RESERVE_BYTES)
    };
    total > limit
}

/// A private, exclusively created audit file and its current recording commitment.
///
/// Filesystem calls are blocking. Callers keep them off async executor threads and
/// forward an event only after its append succeeds. Started and Exit events are
/// synced before success; an ordinary append alone is not a durable checkpoint.
/// Any write or sync failure permanently refuses further appends on this writer.
pub struct RecordingFile {
    pub(crate) file: Arc<File>,
    chain: RecordingChain,
    session_id: String,
    pub(crate) bytes: u64,
    cap: u64,
    failed: bool,
}

impl RecordingFile {
    /// Create a new 0600 file inside a private, non-symlink directory.
    ///
    /// The directory is created as 0700 if absent. Its parent must already exist.
    /// `session_id` must be one filename component. Existing files are never
    /// overwritten, and directory entries are synced before returning success.
    pub fn create(directory: &Path, session_id: String, cap: u64) -> io::Result<Self> {
        if session_id.is_empty()
            || session_id.contains('\0')
            || Path::new(&session_id).file_name() != Some(OsStr::new(&session_id))
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "session ID must be one filename component",
            ));
        }
        match std::fs::DirBuilder::new().mode(0o700).create(directory) {
            Ok(()) => (),
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => (),
            Err(error) => return Err(error),
        }
        let metadata = std::fs::symlink_metadata(directory)?;
        if !metadata.is_dir() || metadata.permissions().mode() & 0o077 != 0 {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "session directory must be private and not a symbolic link",
            ));
        }
        let file = OpenOptions::new()
            .create_new(true)
            .write(true)
            .mode(0o600)
            .open(directory.join(format!("{session_id}.jsonl")))?;
        // An existing directory does not prove that a concurrent creator synced
        // its entry. Persist the recording entry and every directory ancestor.
        let directory = std::fs::canonicalize(directory)?;
        for ancestor in directory.ancestors() {
            File::open(ancestor)?.sync_all()?;
        }
        Ok(Self {
            file: Arc::new(file),
            chain: RecordingChain::new(),
            session_id,
            bytes: 0,
            cap,
            failed: false,
        })
    }

    /// Append an event before forwarding it, reserving space for the final Exit.
    ///
    /// Invalid, oversized or over-budget events leave the file and commitment
    /// unchanged. Each serialized line, including its newline, must fit the
    /// recording readers' event limit.
    /// I/O errors may leave uncommitted bytes in the file; discard the writer and
    /// retain an earlier durable witness instead of retrying that event.
    pub fn append(
        &mut self,
        timestamp_micros: u64,
        payload: RecordingPayload,
    ) -> io::Result<AppendOutcome> {
        if self.failed {
            return Err(io::Error::other("session recording write failed"));
        }
        if matches!(&payload, RecordingPayload::Started { session_id, .. } if session_id != &self.session_id)
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "recording start does not match its session ID",
            ));
        }
        let exit = matches!(payload, RecordingPayload::Exit { .. });
        let boundary = matches!(
            payload,
            RecordingPayload::Started { .. } | RecordingPayload::Exit { .. }
        );
        // A refused preview cannot advance sequence, timestamp or head.
        let mut next_chain = self.chain.clone();
        let event = next_chain
            .append(timestamp_micros, payload)
            .map_err(io::Error::other)?;
        let mut bytes = serde_json::to_vec(&event)?;
        bytes.push(b'\n');
        if bytes.len() > MAX_RECORDING_EVENT_BYTES {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "recording event is oversized",
            ));
        }
        let total = self
            .bytes
            .checked_add(bytes.len() as u64)
            .ok_or_else(|| io::Error::other("session recording size overflow"))?;
        if exceeds_recording_limit(total, self.cap, exit) {
            if boundary {
                return Err(io::Error::other("session recording boundary exceeds limit"));
            }
            return Ok(AppendOutcome::LimitReached);
        }
        let result = self.file.as_ref().write_all(&bytes).and_then(|()| {
            if boundary {
                self.file.sync_data()
            } else {
                Ok(())
            }
        });
        if let Err(error) = result {
            self.failed = true;
            return Err(error);
        }
        self.chain = next_chain;
        self.bytes = total;
        Ok(AppendOutcome::Appended)
    }

    /// The written commitment. Ordinary events require a flush before advertising
    /// this value as durable; Started and Exit appends already include that flush.
    pub fn witness(&self) -> RecordingWitness {
        RecordingWitness {
            session_id: self.session_id.clone(),
            head: self.chain.head().to_owned(),
            events: self.chain.events(),
        }
    }
}

#[cfg(test)]
#[path = "session_recording_test.rs"]
mod tests;
