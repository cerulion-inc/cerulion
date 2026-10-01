// SPDX-License-Identifier: AGPL-3.0-only
//! The `--record-out` sink of `cerulion bag play --resim`.
//!
//! A resim re-executes the bag's graph and captures every graph-produced frame
//! to diff it. With `--record-out PATH` the same captured frames are also
//! written, unchanged, to a fresh MCAP bag. This module owns that file and
//! nothing else: the diff, the verdict and the exit code do not read it.
//!
//! # What the output holds
//!
//! - One channel per graph-produced topic, copied from the input bag's channel
//!   table: the same topic, schema name, schema hash and provisioning.
//! - Every frame the re-executed graph published on those topics, as the full
//!   wire frame, stamped with the sequence and timestamp in its own header (the
//!   rule the recorder uses).
//!
//! The recorded external inputs a resim injects are not copied: they are
//! unchanged and already in the input bag. The output carries no scheduler
//! trace and no graph attachment, so it is a recording of frames to read or
//! index, not a bag `--resim` can re-execute.
//!
//! # Failure
//!
//! The file is created with `create_new`, so an existing file is never
//! overwritten. A run that does not finish (an aborted pass, a write error) removes the
//! partial file instead of leaving an unfinalized bag behind.

use std::collections::{BTreeMap, BTreeSet};
use std::fs::OpenOptions;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

use cerulion_bag::{
    BagChannel, BagWriter, BagWriterConfig, ChannelProvisioning, FileSink, TopicSchema,
    DESCRIPTOR_VERSION, SCHEMA_ENCODING,
};
use cerulion_core::wire::WireHeader;

use crate::replay_cmd::ReplayError;

/// The MCAP header `library` string of a `--record-out` bag.
const RECORD_OUT_LIBRARY: &str = "cerulion_resim_record_out";

/// What `run_replay` hands the engine: where to write, and the input bag's
/// channel table to copy each produced topic's channel from.
pub struct RecordOutPlan {
    /// The output path. Must not exist.
    pub path: PathBuf,
    /// The input bag's user channels (reserved `__cerulion/` channels excluded).
    pub channels: Vec<BagChannel>,
}

/// An open `--record-out` bag. Shared by every rank's pass behind one lock; the
/// passes run one after another, so the lock is never contended.
pub(crate) struct RecordOut {
    path: PathBuf,
    writer: Mutex<Option<BagWriter>>,
    /// The topics with a channel in the output.
    registered: BTreeSet<String>,
    /// Set by [`Self::finalize`]; an unfinalized sink removes its file on drop.
    finalized: AtomicBool,
}

fn internal(reason: String) -> ReplayError {
    ReplayError::Internal { reason }
}

/// The schema registration for one input channel, or the reason it cannot be
/// copied faithfully. The writer stamps the current descriptor version, hash
/// recipe and encoding on every channel, so a channel recorded under different
/// ones would be silently re-labelled; it is refused instead.
fn topic_schema(ch: &BagChannel) -> Result<TopicSchema, String> {
    let Some(d) = ch.descriptor else {
        return Err(format!(
            "channel '{}' carries no Cerulion schema descriptor",
            ch.topic
        ));
    };
    if d.descriptor_version != DESCRIPTOR_VERSION
        || d.hash_recipe != cerulion_core::trace::bag::HASH_RECIPE
    {
        return Err(format!(
            "channel '{}' was recorded with a different schema descriptor version or hash \
             recipe than this build writes",
            ch.topic
        ));
    }
    if ch.schema_encoding != SCHEMA_ENCODING || ch.message_encoding != SCHEMA_ENCODING {
        return Err(format!(
            "channel '{}' uses an encoding other than '{SCHEMA_ENCODING}'",
            ch.topic
        ));
    }
    Ok(TopicSchema {
        topic: ch.topic.clone(),
        schema_name: ch.schema_name.clone(),
        schema_hash: d.schema_hash,
        wire_fixed_size: d.wire_fixed_size,
    })
}

impl RecordOut {
    /// Create the output bag with one channel per `produced` topic that the
    /// input bag also carries. A produced topic the input has no channel for
    /// (a node added since the recording) cannot copy one, so it is skipped
    /// with a warning rather than invented or failed.
    pub(crate) fn open(plan: RecordOutPlan, produced: &[String]) -> Result<Self, ReplayError> {
        let RecordOutPlan { path, channels } = plan;
        let mut schemas = Vec::new();
        let mut provisioning: BTreeMap<String, ChannelProvisioning> = BTreeMap::new();
        for topic in produced {
            let Some(ch) = channels.iter().find(|c| &c.topic == topic) else {
                tracing::warn!(
                    topic = %topic,
                    "resim: --record-out skips a produced topic the input bag has no channel \
                     for, so its frames are not in the output"
                );
                continue;
            };
            schemas.push(topic_schema(ch).map_err(|why| {
                internal(format!(
                    "--record-out cannot copy {why}, so the output would mislabel its schema"
                ))
            })?);
            if !ch.provisioning.is_empty() {
                provisioning.insert(topic.clone(), ch.provisioning);
            }
        }
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .map_err(|e| {
                internal(format!(
                    "--record-out cannot create '{}': {e}",
                    path.display()
                ))
            })?;
        // From here the file exists: every failure path below must remove it.
        let mut out = Self {
            path,
            writer: Mutex::new(None),
            registered: schemas.iter().map(|s| s.topic.clone()).collect(),
            finalized: AtomicBool::new(false),
        };
        let config = BagWriterConfig {
            library: RECORD_OUT_LIBRARY.to_string(),
            provisioning,
            ..Default::default()
        };
        let sink = FileSink::from_file(file).map_err(|e| {
            internal(format!(
                "--record-out cannot write '{}': {e}",
                out.path.display()
            ))
        })?;
        let writer = BagWriter::with_sink(sink, config, &schemas).map_err(|e| {
            internal(format!(
                "--record-out cannot start the bag '{}': {e}",
                out.path.display()
            ))
        })?;
        *out.writer.get_mut().unwrap_or_else(|p| p.into_inner()) = Some(writer);
        Ok(out)
    }

    /// Write one captured wire frame to its topic's channel. A topic with no
    /// channel in the output is a no-op (it was skipped at [`Self::open`]).
    pub(crate) fn write_frame(&self, topic: &str, frame: &[u8]) -> Result<(), ReplayError> {
        if !self.registered.contains(topic) {
            return Ok(());
        }
        let (seq, ts) = WireHeader::read_from_buf(frame)
            .map(|h| (h.sequence, h.timestamp_ns))
            .unwrap_or((0, 0));
        let mut guard = self.writer.lock().unwrap_or_else(|p| p.into_inner());
        let Some(writer) = guard.as_mut() else {
            return Err(internal(format!(
                "--record-out writer for '{}' is closed",
                self.path.display()
            )));
        };
        writer
            .write_message(topic, seq, ts, ts, &[frame])
            .map_err(|e| {
                internal(format!(
                    "--record-out cannot write a frame of '{topic}' to '{}': {e}",
                    self.path.display()
                ))
            })
    }

    /// Close the bag and return its path. Until this succeeds the file is
    /// removed on drop.
    pub(crate) fn finalize(&self) -> Result<String, ReplayError> {
        let path = self.path.display().to_string();
        let writer = self.writer.lock().unwrap_or_else(|p| p.into_inner()).take();
        let Some(writer) = writer else {
            return Err(internal(format!("--record-out '{path}' is already closed")));
        };
        writer
            .finalize()
            .map_err(|e| internal(format!("--record-out cannot finish '{path}': {e}")))?;
        self.finalized.store(true, Ordering::Release);
        Ok(path)
    }
}

impl Drop for RecordOut {
    fn drop(&mut self) {
        if !self.finalized.load(Ordering::Acquire) {
            // Best effort: an unfinalized bag is worse than none, and the run
            // that was writing it already reports its own failure.
            let _ = std::fs::remove_file(&self.path);
        }
    }
}
