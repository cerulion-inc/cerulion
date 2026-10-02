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
    BagChannel, BagSchemaCatalog, BagWriter, BagWriterConfig, ChannelProvisioning, FileSink,
    TopicSchema, DESCRIPTOR_VERSION, SCHEMA_ENCODING,
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
    /// The input bag's schema catalog, if it carries one. The output gets the
    /// part of it its own channels use, so custom types stay readable.
    pub catalog: Option<BagSchemaCatalog>,
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
        let RecordOutPlan {
            path,
            channels,
            catalog,
        } = plan;
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
                if e.kind() == std::io::ErrorKind::AlreadyExists {
                    return ReplayError::RecordOutExists { path: path.clone() };
                }
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
        let mut writer = writer;
        if let Some(catalog) = catalog {
            // Only what the exported channels use, the way the recorder prunes.
            let used = catalog.closure_for_hashes(schemas.iter().map(|s| s.schema_hash));
            writer.write_schema_catalog(&used).map_err(|e| {
                internal(format!(
                    "--record-out cannot write the schema catalog of '{}': {e}",
                    out.path.display()
                ))
            })?;
        }
        *out.writer.get_mut().unwrap_or_else(|p| p.into_inner()) = Some(writer);
        Ok(out)
    }

    /// Write one captured wire frame to its topic's channel. A topic with no
    /// channel in the output is a no-op (it was skipped at [`Self::open`]).
    pub(crate) fn write_frame(&self, topic: &str, frame: &[u8]) -> Result<(), ReplayError> {
        if !self.registered.contains(topic) {
            return Ok(());
        }
        // A frame with no wire header has no sequence or timestamp to record, and
        // a zero would claim values the frame never carried.
        let Some(header) = WireHeader::read_from_buf(frame) else {
            return Err(internal(format!(
                "--record-out cannot write a frame of '{topic}': it is {} bytes, shorter than \
                 the wire header, so it has no sequence or timestamp to record",
                frame.len()
            )));
        };
        let (seq, ts) = (header.sequence, header.timestamp_ns);
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

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_bag::{BagReader, SchemaDescriptor};
    use cerulion_core::{SchemaDoc, SchemaEncoding, SchemaHashName};

    const HASH: u64 = 0x11;

    fn channel(topic: &str) -> BagChannel {
        BagChannel {
            id: 1,
            topic: topic.to_string(),
            schema_name: "go/State".to_string(),
            schema_encoding: SCHEMA_ENCODING.to_string(),
            message_encoding: SCHEMA_ENCODING.to_string(),
            descriptor: Some(SchemaDescriptor {
                descriptor_version: DESCRIPTOR_VERSION,
                hash_recipe: cerulion_core::trace::bag::HASH_RECIPE,
                schema_hash: HASH,
                wire_fixed_size: 8,
            }),
            provisioning: ChannelProvisioning::default(),
        }
    }

    fn catalog() -> BagSchemaCatalog {
        let doc = |q: &str| SchemaDoc {
            qualified: q.to_string(),
            encoding: SchemaEncoding::Msg,
            text: "float32 q\n".to_string(),
            deps: Vec::new(),
        };
        BagSchemaCatalog::new(
            vec![doc("go/State"), doc("go/Unrelated")],
            vec![
                SchemaHashName {
                    schema_hash: HASH,
                    qualified: "go/State".to_string(),
                },
                SchemaHashName {
                    schema_hash: 0x22,
                    qualified: "go/Unrelated".to_string(),
                },
            ],
        )
    }

    fn open(dir: &std::path::Path, with_catalog: bool) -> (RecordOut, PathBuf) {
        let path = dir.join("out.mcap");
        let plan = RecordOutPlan {
            path: path.clone(),
            channels: vec![channel("/state")],
            catalog: with_catalog.then(catalog),
        };
        (
            RecordOut::open(plan, &["/state".to_string()]).expect("opens"),
            path,
        )
    }

    fn frame(seq: u32, ts: u64) -> Vec<u8> {
        let mut buf = vec![0u8; WireHeader::SIZE + 8];
        WireHeader::new(HASH, seq, ts).write_to_buf(&mut buf);
        buf
    }

    /// A frame shorter than the wire header has no sequence or timestamp, so
    /// writing it is refused rather than stamped with zeros.
    #[test]
    fn a_headerless_frame_is_refused_not_stamped_with_zeros() {
        let dir = tempfile::tempdir().unwrap();
        let (out, _) = open(dir.path(), false);
        let err = out.write_frame("/state", &[1, 2, 3]).unwrap_err();
        assert!(err.to_string().contains("wire header"), "{err}");
        // The message reads as one sentence: no run of spaces from a lost line
        // continuation.
        assert!(!err.to_string().contains("  "), "{err}");
        // ANTI-TAUTOLOGY: a full frame on the same sink is accepted.
        out.write_frame("/state", &frame(7, 99))
            .expect("full frame");
    }

    /// A path taken after the surface's precheck is a usage error (exit 2),
    /// never an overwrite and never an internal failure.
    #[test]
    fn a_path_taken_before_create_is_a_usage_error_and_left_untouched() {
        let dir = tempfile::tempdir().unwrap();
        let taken = dir.path().join("taken.mcap");
        std::fs::write(&taken, b"precious").unwrap();
        let plan = RecordOutPlan {
            path: taken.clone(),
            channels: vec![channel("/state")],
            catalog: None,
        };
        let err = RecordOut::open(plan, &["/state".to_string()])
            .err()
            .expect("refused");
        assert!(
            matches!(err, ReplayError::RecordOutExists { .. }),
            "{err:?}"
        );
        assert_eq!(err.exit_code(), 2);
        assert_eq!(std::fs::read(&taken).unwrap(), b"precious");
        // ANTI-TAUTOLOGY: a free path on the same plan opens.
        let (_out, free) = open(dir.path(), false);
        assert!(free.exists());
    }

    /// The output keeps the input's schema definitions for the types it still
    /// carries, and only those.
    #[test]
    fn the_output_carries_the_pruned_schema_catalog() {
        let dir = tempfile::tempdir().unwrap();
        let (out, path) = open(dir.path(), true);
        out.write_frame("/state", &frame(1, 5)).unwrap();
        out.finalize().unwrap();
        let got = BagReader::open(&path)
            .unwrap()
            .schema_catalog()
            .expect("catalog");
        let names: Vec<&str> = got.docs.iter().map(|d| d.qualified.as_str()).collect();
        assert_eq!(names, ["go/State"], "pruned to the exported channel's type");

        // ANTI-TAUTOLOGY: an input with no catalog yields an output with none.
        let dir = tempfile::tempdir().unwrap();
        let (out, path) = open(dir.path(), false);
        out.finalize().unwrap();
        assert!(BagReader::open(&path).unwrap().schema_catalog().is_none());
    }
}
