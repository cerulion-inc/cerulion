// SPDX-License-Identifier: AGPL-3.0-only
//! Read-only inspection of session audit recordings against external witnesses.

use std::io;
use std::path::{Path, PathBuf};

use cerulion_ops_proto::recording::{RecordingEvent, RecordingPayload, RecordingWitness};

use super::MAX_RECORDING_EVENT_BYTES;

/// List witnesses from complete recordings, skipping valid unfinished prefixes.
/// The caller still verifies each witness against its receipt or the desk's
/// independent saved witness.
pub fn list_recordings(directory: &Path) -> io::Result<Vec<RecordingWitness>> {
    Ok(list_recordings_with_incomplete(directory)?.witnesses)
}

/// Complete local chain heads and paths omitted because their recordings are unfinished.
pub struct RecordingListing {
    pub witnesses: Vec<RecordingWitness>,
    pub incomplete: Vec<PathBuf>,
}

/// Inspect recordings while retaining unfinished paths for a caller's diagnostics.
/// No locally computed head authenticates a recording without an external witness.
pub fn list_recordings_with_incomplete(directory: &Path) -> io::Result<RecordingListing> {
    let mut witnesses = Vec::new();
    let mut incomplete = Vec::new();
    for entry in std::fs::read_dir(directory)? {
        let entry = entry?;
        if entry
            .path()
            .extension()
            .is_none_or(|extension| extension != "jsonl")
        {
            continue;
        }
        let (events, partial_tail) = read_recording_for_listing(&entry.path())?;
        match classify_recording(&events, partial_tail)? {
            Some(witness) => witnesses.push(witness),
            None => incomplete.push(entry.path()),
        }
    }
    witnesses.sort_by(|left, right| {
        (&left.session_id, &left.head, left.events).cmp(&(
            &right.session_id,
            &right.head,
            right.events,
        ))
    });
    incomplete.sort();
    Ok(RecordingListing {
        witnesses,
        incomplete,
    })
}

// Validate parsed values only; directory traversal and notices stay with callers.
fn classify_recording(
    events: &[RecordingEvent],
    partial_tail: bool,
) -> io::Result<Option<RecordingWitness>> {
    if events.is_empty() {
        return Ok(None);
    }
    let Some(RecordingEvent {
        payload: RecordingPayload::Started { session_id, .. },
        ..
    }) = events.first()
    else {
        return Err(io::Error::other("recording has no opening event"));
    };
    let Some(tail) = events.last() else {
        return Ok(None);
    };
    let witness = RecordingWitness {
        session_id: session_id.clone(),
        head: tail.hash.clone(),
        events: events.len() as u64,
    };
    if matches!(tail.payload, RecordingPayload::Exit { .. }) {
        if partial_tail {
            return Err(io::Error::other(
                "recording has data after final exit event",
            ));
        }
        cerulion_ops_proto::recording::verify_recording(events, &witness)
            .map_err(io::Error::other)?;
        Ok(Some(witness))
    } else {
        // This locally derived witness checks structure, not authenticity.
        cerulion_ops_proto::recording::verify_recording_prefix(events, &witness)
            .map_err(io::Error::other)?;
        Ok(None)
    }
}

/// Verify a recording against a separately retained, mandatory witness.
pub fn verify_recording_file(path: &Path, witness: &RecordingWitness) -> io::Result<()> {
    cerulion_ops_proto::recording::verify_recording(&read_recording(path)?, witness)
        .map_err(io::Error::other)
}

/// Verify exactly the committed prefix retained by the desk. Bytes after that
/// prefix may include an interrupted append and are outside this commitment.
pub fn verify_recording_prefix_file(
    path: &Path,
    witness: &RecordingWitness,
) -> io::Result<RecordingPrefixSummary> {
    use std::io::{BufRead, Read};
    let mut reader = open_recording(path)?;
    let mut events = Vec::new();
    let mut line = String::new();
    for _ in 0..witness.events {
        line.clear();
        (&mut reader)
            .take(MAX_RECORDING_EVENT_BYTES as u64 + 1)
            .read_line(&mut line)?;
        if line.len() > MAX_RECORDING_EVENT_BYTES {
            return Err(io::Error::other("recording prefix event is oversized"));
        }
        if !line.ends_with('\n') {
            return Err(io::Error::other(
                "recording is shorter than its committed prefix",
            ));
        }
        events.push(serde_json::from_str(&line)?);
    }
    cerulion_ops_proto::recording::verify_recording_prefix(&events, witness)
        .map_err(io::Error::other)?;
    let mut summary = RecordingPrefixSummary::default();
    let mut buffer = [0; 8192];
    loop {
        let count = Read::read(&mut reader, &mut buffer)?;
        if count == 0 {
            return Ok(summary);
        }
        summary.unverified_bytes = summary
            .unverified_bytes
            .checked_add(count as u64)
            .ok_or_else(|| io::Error::other("recording tail size overflow"))?;
        summary.unverified_event_lines = summary
            .unverified_event_lines
            .checked_add(
                buffer[..count]
                    .iter()
                    .filter(|byte| **byte == b'\n')
                    .count() as u64,
            )
            .ok_or_else(|| io::Error::other("recording tail line count overflow"))?;
        summary.unverified_partial_line = buffer[count - 1] != b'\n';
    }
}

/// Unverified bytes following a successfully checked durable prefix.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RecordingPrefixSummary {
    /// Newline-terminated lines after the prefix, without parsing or validating
    /// their contents. A line is counted even when it is not a valid event.
    pub unverified_event_lines: u64,
    /// All bytes after the prefix, including newline delimiters and any tail.
    pub unverified_bytes: u64,
    /// The unverified tail ends with bytes not terminated by a newline.
    pub unverified_partial_line: bool,
}

// A writer can be between bytes of its next append. Only newline-terminated
// events are parsed; callers must reject an unfinished tail after a final Exit.
fn read_recording_for_listing(path: &Path) -> io::Result<(Vec<RecordingEvent>, bool)> {
    use std::io::{BufRead, Read};
    let mut reader = open_recording(path)?;
    let mut events = Vec::new();
    let mut line = Vec::new();
    loop {
        line.clear();
        (&mut reader)
            .take(MAX_RECORDING_EVENT_BYTES as u64 + 1)
            .read_until(b'\n', &mut line)?;
        if line.len() > MAX_RECORDING_EVENT_BYTES {
            return Err(io::Error::other("recording event is oversized"));
        }
        if line.is_empty() {
            return Ok((events, false));
        }
        if !line.ends_with(b"\n") {
            return Ok((events, true));
        }
        events.push(serde_json::from_slice(&line)?);
    }
}

fn read_recording(path: &Path) -> io::Result<Vec<RecordingEvent>> {
    let (events, partial_tail) = read_recording_for_listing(path)?;
    if partial_tail {
        return Err(io::Error::other("recording ends with an incomplete event"));
    }
    Ok(events)
}

fn open_recording(path: &Path) -> io::Result<std::io::BufReader<std::fs::File>> {
    let mut options = std::fs::OpenOptions::new();
    options.read(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        // A replaced symlink or FIFO must not redirect or block inspection.
        options.custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK);
    }
    open_recording_with(path, |path| options.open(path))
}

fn open_recording_with(
    path: &Path,
    open: impl FnOnce(&Path) -> io::Result<std::fs::File>,
) -> io::Result<std::io::BufReader<std::fs::File>> {
    let inspected = std::fs::symlink_metadata(path)?;
    if !inspected.is_file() {
        return Err(io::Error::other("recording must be a regular file"));
    }
    let file = open(path)?;
    let opened = file.metadata()?;
    if !opened.is_file() {
        return Err(io::Error::other("recording must be a regular file"));
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        if (inspected.dev(), inspected.ino()) != (opened.dev(), opened.ino()) {
            return Err(io::Error::other("recording changed while opening"));
        }
    }
    Ok(std::io::BufReader::new(file))
}

#[cfg(test)]
mod tests;
