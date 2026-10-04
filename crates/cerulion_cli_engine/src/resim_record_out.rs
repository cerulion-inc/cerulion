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
//! - One channel per graph-produced topic. Its schema HASH is the one the
//!   re-executed graph publishes: the current workspace's hash for the topic
//!   when it resolves, else the input channel's when that was recorded under
//!   this build's hash recipe, else the hash in the topic's first written frame.
//!   The input bag's channel is never trusted for the hash on its own, because
//!   a channel with no recorded frames escapes the schema-drift preflight and
//!   a legacy recipe hashes differently. Provisioning is copied from the input
//!   bag's channel where it has one. Its schema name and fixed wire size are
//!   copied only when its hash, under this build's recipe, equals the label;
//!   otherwise the channel takes the current workspace's schema name (else the
//!   input's, else `unknown`) and a fixed wire size of 0, the recorder's
//!   convention for a size it cannot vouch for.
//! - Every frame the re-executed graph published on those topics, as the full
//!   wire frame, stamped with the sequence and timestamp in its own header. A
//!   frame shorter than the wire header is written whole with sequence and
//!   timestamp 0, the rule the recorder uses, and warned once per topic.
//!
//! A frame whose header names a schema hash other than its channel's is
//! refused and the run fails, so the output never labels a frame with a schema
//! it does not carry. A channel whose first frame had no header is labelled 0,
//! and a later frame naming any other hash is refused the same way.
//!
//! The input bag's custom-type schema catalog is carried over at finalize,
//! pruned to the hashes the output's channels carry. A hash the catalog does
//! not bind (a legacy recipe's) brings no definition: the catalog cannot vouch
//! that its text hashes to it.
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
    TopicSchema,
};
use cerulion_core::wire::WireHeader;

use crate::replay_cmd::ReplayError;

/// The MCAP header `library` string of a `--record-out` bag.
const RECORD_OUT_LIBRARY: &str = "cerulion_resim_record_out";

/// The schema name of a channel whose type nothing names, the recorder's
/// placeholder for the same case.
const UNKNOWN_SCHEMA_NAME: &str = "unknown";

/// What the current workspace says a produced topic's schema is, as resolved
/// by the replay's field registry. Either half may be unknown.
#[derive(Debug, Clone, Default)]
pub struct CurrentSchema {
    /// The qualified name of the topic's current root schema.
    pub name: Option<String>,
    /// The current recipe hash of that schema.
    pub hash: Option<u64>,
}

/// What `run_replay` hands the engine: where to write, the input bag's
/// channel table to copy each produced topic's name and provisioning from, and
/// the current workspace's schema for each topic.
pub struct RecordOutPlan {
    /// The output path. Must not exist.
    pub path: PathBuf,
    /// The input bag's user channels (reserved `__cerulion/` channels excluded).
    pub channels: Vec<BagChannel>,
    /// The input bag's schema catalog, if it carries one. The output gets the
    /// part of it its own channels use, so custom types stay readable.
    pub catalog: Option<BagSchemaCatalog>,
    /// The current workspace's schema per topic (topics it does not resolve
    /// may be absent).
    pub current: BTreeMap<String, CurrentSchema>,
}

/// What a produced topic's channel is built from, whenever it is registered.
struct Template {
    /// The input channel's schema name, if the input bag has the channel.
    input_name: Option<String>,
    /// The input channel's fixed wire size (0 without a descriptor).
    input_size: u32,
    /// The input channel's hash when it was recorded under this build's recipe
    /// and is not the unknown hash 0: the only input hash comparable to a label.
    input_hash: Option<u64>,
    /// The current workspace's schema name for the topic.
    current_name: Option<String>,
}

impl Template {
    fn new(input: Option<&BagChannel>, current: Option<&CurrentSchema>) -> Self {
        Self {
            input_name: input.map(|ch| ch.schema_name.clone()),
            input_size: input
                .and_then(|ch| ch.descriptor)
                .map_or(0, |d| d.wire_fixed_size),
            input_hash: input
                .and_then(|ch| ch.descriptor)
                .filter(|d| {
                    d.hash_recipe == cerulion_core::trace::bag::HASH_RECIPE && d.schema_hash != 0
                })
                .map(|d| d.schema_hash),
            current_name: current.and_then(|c| c.name.clone()),
        }
    }

    /// The channel for `topic` labelled `schema_hash`. The input's name and
    /// fixed size describe the input's hash, so they are kept only when that is
    /// the label.
    fn schema(&self, topic: &str, schema_hash: u64) -> TopicSchema {
        let (schema_name, wire_fixed_size) = if self.input_hash == Some(schema_hash) {
            let name = self.input_name.clone().unwrap_or_default();
            (name, self.input_size)
        } else {
            let name = self
                .current_name
                .clone()
                .or_else(|| self.input_name.clone())
                .unwrap_or_else(|| UNKNOWN_SCHEMA_NAME.to_string());
            (name, 0)
        };
        TopicSchema {
            topic: topic.to_string(),
            schema_name,
            schema_hash,
            wire_fixed_size,
        }
    }
}

/// The writer and the per-topic state every frame consults, behind one lock.
struct Inner {
    writer: Option<BagWriter>,
    /// Each registered topic's channel hash.
    hashes: BTreeMap<String, u64>,
    /// Produced topics not registered yet (their hash is learned from a frame).
    pending: BTreeMap<String, Template>,
    /// The input bag's schema catalog, written pruned at finalize.
    catalog: Option<BagSchemaCatalog>,
    /// Topics a headerless frame has already been warned for.
    warned_headerless: BTreeSet<String>,
}

/// An open `--record-out` bag. Shared by every rank's pass behind one lock; the
/// passes run one after another, so the lock is never contended.
pub(crate) struct RecordOut {
    path: PathBuf,
    inner: Mutex<Inner>,
    /// Set by [`Self::finalize`]; an unfinalized sink removes its file on drop.
    finalized: AtomicBool,
}

fn internal(reason: String) -> ReplayError {
    ReplayError::Internal { reason }
}

impl RecordOut {
    /// Create the output bag. Every produced topic whose hash is known up front
    /// gets its channel now; the rest get theirs from their first frame, or
    /// with hash 0 at [`Self::finalize`] if none arrives.
    pub(crate) fn open(plan: RecordOutPlan, produced: &[String]) -> Result<Self, ReplayError> {
        let RecordOutPlan {
            path,
            channels,
            catalog,
            current,
        } = plan;
        let mut schemas = Vec::new();
        let mut pending = BTreeMap::new();
        let mut provisioning: BTreeMap<String, ChannelProvisioning> = BTreeMap::new();
        for topic in produced {
            let input = channels.iter().find(|c| &c.topic == topic);
            let now = current.get(topic);
            if let Some(ch) = input.filter(|ch| !ch.provisioning.is_empty()) {
                provisioning.insert(topic.clone(), ch.provisioning);
            }
            let template = Template::new(input, now);
            // The current workspace's hash, else the input's comparable one;
            // with neither, only a frame can tell.
            match now.and_then(|c| c.hash).or(template.input_hash) {
                Some(hash) => schemas.push(template.schema(topic, hash)),
                None => {
                    pending.insert(topic.clone(), template);
                }
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
            inner: Mutex::new(Inner {
                writer: None,
                hashes: schemas
                    .iter()
                    .map(|s| (s.topic.clone(), s.schema_hash))
                    .collect(),
                pending,
                catalog,
                warned_headerless: BTreeSet::new(),
            }),
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
        out.inner
            .get_mut()
            .unwrap_or_else(|p| p.into_inner())
            .writer = Some(writer);
        Ok(out)
    }

    /// Write one captured wire frame to its topic's channel. A topic that is
    /// not graph-produced is a no-op.
    pub(crate) fn write_frame(&self, topic: &str, frame: &[u8]) -> Result<(), ReplayError> {
        let mut guard = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        let inner = &mut *guard;
        let header = WireHeader::read_from_buf(frame);
        if !inner.hashes.contains_key(topic) {
            let Some(template) = inner.pending.remove(topic) else {
                return Ok(());
            };
            // The first frame names the hash; a headerless one cannot, so the
            // channel takes 0, the recorder's "unknown" hash.
            let schema = template.schema(topic, header.map_or(0, |h| h.schema_hash));
            let Some(writer) = inner.writer.as_mut() else {
                return Err(self.closed());
            };
            writer.register_topic(&schema).map_err(|e| {
                internal(format!(
                    "--record-out cannot add the channel of '{topic}' to '{}': {e}",
                    self.path.display()
                ))
            })?;
            inner.hashes.insert(topic.to_string(), schema.schema_hash);
        }
        let label = inner.hashes[topic];
        let (seq, ts) = match header {
            Some(h) => {
                if h.schema_hash != label {
                    return Err(internal(format!(
                        "--record-out cannot write a frame of '{topic}': it carries schema hash \
                         {:#018x} but the channel is labelled {label:#018x}, so the output \
                         would mislabel it",
                        h.schema_hash
                    )));
                }
                (h.sequence, h.timestamp_ns)
            }
            None => {
                // Recorded whole with sequence and timestamp 0, as the recorder
                // does: nothing trustworthy is on the wire.
                if inner.warned_headerless.insert(topic.to_string()) {
                    tracing::warn!(
                        topic = %topic,
                        len = frame.len(),
                        "resim: --record-out writes a frame shorter than the wire header \
                         whole, with sequence and timestamp 0"
                    );
                }
                (0, 0)
            }
        };
        let Some(writer) = inner.writer.as_mut() else {
            return Err(self.closed());
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

    fn closed(&self) -> ReplayError {
        internal(format!(
            "--record-out writer for '{}' is closed",
            self.path.display()
        ))
    }

    /// Close the bag and return its path. Until this succeeds the file is
    /// removed on drop.
    pub(crate) fn finalize(&self) -> Result<String, ReplayError> {
        let path = self.path.display().to_string();
        let mut guard = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        let inner = &mut *guard;
        let Some(mut writer) = inner.writer.take() else {
            return Err(internal(format!("--record-out '{path}' is already closed")));
        };
        // A produced topic that published nothing still gets its channel, with
        // the "unknown" hash 0 since no frame named one.
        for (topic, template) in std::mem::take(&mut inner.pending) {
            writer
                .register_topic(&template.schema(&topic, 0))
                .map_err(|e| {
                    internal(format!(
                        "--record-out cannot add the channel of '{topic}' to '{path}': {e}"
                    ))
                })?;
            inner.hashes.insert(topic, 0);
        }
        if let Some(catalog) = inner.catalog.take() {
            // Only what the output's channels carry, the way the recorder
            // prunes, including the channels a frame registered late.
            let used = catalog.closure_for_hashes(inner.hashes.values().copied());
            writer.write_schema_catalog(&used).map_err(|e| {
                internal(format!(
                    "--record-out cannot write the schema catalog of '{path}': {e}"
                ))
            })?;
        }
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
    use cerulion_bag::{BagReader, SchemaDescriptor, DESCRIPTOR_VERSION, SCHEMA_ENCODING};
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
            current: BTreeMap::new(),
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

    fn frame_with(hash: u64, seq: u32, ts: u64) -> Vec<u8> {
        let mut buf = vec![0u8; WireHeader::SIZE + 8];
        WireHeader::new(hash, seq, ts).write_to_buf(&mut buf);
        buf
    }

    /// Finalize, then read back `(topic, schema_name, schema_hash,
    /// wire_fixed_size)` per channel and `(topic, sequence, log_time, len)` per
    /// message.
    #[allow(clippy::type_complexity)]
    fn read_back(
        out: RecordOut,
        path: &std::path::Path,
    ) -> (
        Vec<(String, String, u64, u32)>,
        Vec<(String, u32, u64, usize)>,
    ) {
        out.finalize().expect("finalizes");
        let reader = BagReader::open(path).unwrap();
        let mut channels: Vec<_> = reader
            .channels()
            .unwrap()
            .into_iter()
            .filter(|c| !c.topic.starts_with(cerulion_bag::RESERVED_PREFIX))
            .map(|c| {
                let d = c.descriptor.expect("descriptor");
                (c.topic, c.schema_name, d.schema_hash, d.wire_fixed_size)
            })
            .collect();
        channels.sort();
        let messages = reader
            .messages()
            .unwrap()
            .map(|m| m.unwrap())
            .filter(|m| !m.topic.starts_with(cerulion_bag::RESERVED_PREFIX))
            .map(|m| (m.topic, m.sequence, m.log_time, m.data.len()))
            .collect();
        (channels, messages)
    }

    fn plan(dir: &std::path::Path, channels: Vec<BagChannel>) -> (RecordOutPlan, PathBuf) {
        let path = dir.join("out.mcap");
        (
            RecordOutPlan {
                path: path.clone(),
                channels,
                catalog: None,
                current: BTreeMap::new(),
            },
            path,
        )
    }

    /// A frame shorter than the wire header is written whole with sequence and
    /// timestamp 0, the recorder's rule, and the run goes on.
    #[test]
    fn a_headerless_frame_is_written_whole_with_zero_stamps() {
        let dir = tempfile::tempdir().unwrap();
        let (out, path) = open(dir.path(), false);
        out.write_frame("/state", &[1, 2, 3]).expect("headerless");
        out.write_frame("/state", &frame(7, 99))
            .expect("full frame");
        let (_, messages) = read_back(out, &path);
        assert_eq!(
            messages,
            [
                ("/state".to_string(), 0, 0, 3),
                ("/state".to_string(), 7, 99, WireHeader::SIZE + 8),
            ]
        );
    }

    /// A channel with no recorded frames escapes the schema-drift preflight,
    /// so its descriptor can be stale. The output is labelled with the CURRENT
    /// hash, the one the re-executed frames carry, and with the current name;
    /// the input's fixed size describes the old layout, so it is not kept.
    #[test]
    fn a_stale_input_descriptor_is_relabelled_with_the_current_hash() {
        const CURRENT: u64 = 0x99;
        let dir = tempfile::tempdir().unwrap();
        let (mut plan, path) = plan(dir.path(), vec![channel("/state")]);
        plan.current.insert(
            "/state".to_string(),
            CurrentSchema {
                name: Some("go/NewState".to_string()),
                hash: Some(CURRENT),
            },
        );
        let out = RecordOut::open(plan, &["/state".to_string()]).expect("opens");
        out.write_frame("/state", &frame_with(CURRENT, 1, 5))
            .expect("current frame");
        // A frame naming any other hash is refused rather than mislabelled.
        let err = out
            .write_frame("/state", &frame_with(HASH, 2, 6))
            .unwrap_err();
        assert!(err.to_string().contains("mislabel"), "{err}");
        assert!(!err.to_string().contains("  "), "{err}");
        let (channels, messages) = read_back(out, &path);
        assert_eq!(
            channels,
            [("/state".to_string(), "go/NewState".to_string(), CURRENT, 0)]
        );
        assert_eq!(messages.len(), 1);
    }

    /// A legacy bag (another hash recipe) that plain replay accepts is
    /// accepted here too: with no current hash the channel learns it from the
    /// first frame. It keeps the input's name; the input's fixed size belongs
    /// to a hash it cannot compare, so it is not kept.
    #[test]
    fn a_legacy_recipe_channel_takes_its_hash_from_the_first_frame() {
        let dir = tempfile::tempdir().unwrap();
        let mut legacy = channel("/state");
        legacy.descriptor = Some(SchemaDescriptor {
            descriptor_version: DESCRIPTOR_VERSION,
            hash_recipe: cerulion_core::trace::bag::HASH_RECIPE.wrapping_sub(1),
            schema_hash: HASH,
            wire_fixed_size: 8,
        });
        legacy.schema_encoding = SCHEMA_ENCODING.to_string();
        let (plan, path) = plan(dir.path(), vec![legacy]);
        let out = RecordOut::open(plan, &["/state".to_string()]).expect("opens");
        out.write_frame("/state", &frame_with(0x77, 1, 5)).unwrap();
        out.write_frame("/state", &frame_with(0x77, 2, 6)).unwrap();
        // The learned label is enforced like an up-front one.
        assert!(out.write_frame("/state", &frame_with(0x78, 3, 7)).is_err());
        let (channels, messages) = read_back(out, &path);
        assert_eq!(
            channels,
            [("/state".to_string(), "go/State".to_string(), 0x77, 0)]
        );
        assert_eq!(messages.len(), 2);
    }

    /// Every graph-produced topic gets a channel: one the input bag has no
    /// channel for takes the current workspace's name (or `unknown`), and one
    /// that published nothing is still registered at finalize.
    #[test]
    fn every_produced_topic_gets_a_channel_even_without_an_input_channel() {
        let dir = tempfile::tempdir().unwrap();
        let (mut plan, path) = plan(dir.path(), Vec::new());
        plan.current.insert(
            "/added".to_string(),
            CurrentSchema {
                name: Some("go/Added".to_string()),
                hash: Some(0x55),
            },
        );
        let produced = ["/added", "/nameless", "/silent"].map(str::to_string);
        let out = RecordOut::open(plan, &produced).expect("opens");
        out.write_frame("/added", &frame_with(0x55, 1, 5)).unwrap();
        out.write_frame("/nameless", &frame_with(0x66, 1, 5))
            .unwrap();
        // ANTI-TAUTOLOGY: a topic the graph does not produce stays out.
        out.write_frame("/injected", &frame_with(0x66, 1, 5))
            .unwrap();
        let (channels, messages) = read_back(out, &path);
        assert_eq!(
            channels,
            [
                ("/added".to_string(), "go/Added".to_string(), 0x55, 0),
                ("/nameless".to_string(), "unknown".to_string(), 0x66, 0),
                ("/silent".to_string(), "unknown".to_string(), 0, 0),
            ]
        );
        assert_eq!(messages.len(), 2);
    }

    /// A channel whose first frame had no header is labelled 0, and a later
    /// frame naming a real hash is refused rather than filed under it.
    #[test]
    fn a_channel_labelled_by_a_headerless_frame_refuses_any_other_hash() {
        let dir = tempfile::tempdir().unwrap();
        let (plan, path) = plan(dir.path(), Vec::new());
        let out = RecordOut::open(plan, &["/state".to_string()]).expect("opens");
        out.write_frame("/state", &[1, 2, 3]).expect("headerless");
        let err = out
            .write_frame("/state", &frame_with(0x77, 1, 5))
            .unwrap_err();
        assert!(err.to_string().contains("mislabel"), "{err}");
        // ANTI-TAUTOLOGY: a frame that does carry hash 0 still fits.
        out.write_frame("/state", &frame_with(0, 2, 6))
            .expect("hash 0 frame");
        let (channels, messages) = read_back(out, &path);
        assert_eq!(channels[0].2, 0);
        assert_eq!(messages.len(), 2);
    }

    /// A channel registered by its first frame still gets its definitions
    /// from the input catalog: pruning runs at finalize, over every channel.
    #[test]
    fn a_late_channel_keeps_its_catalog_definitions() {
        let dir = tempfile::tempdir().unwrap();
        let mut placeholder = channel("/state");
        // Hash 0 is the unknown placeholder: nothing comparable, so the
        // channel waits for its first frame.
        placeholder.descriptor = Some(SchemaDescriptor {
            descriptor_version: DESCRIPTOR_VERSION,
            hash_recipe: cerulion_core::trace::bag::HASH_RECIPE,
            schema_hash: 0,
            wire_fixed_size: 8,
        });
        let (mut plan, path) = plan(dir.path(), vec![placeholder]);
        plan.catalog = Some(catalog());
        let out = RecordOut::open(plan, &["/state".to_string()]).expect("opens");
        out.write_frame("/state", &frame(1, 5)).unwrap();
        let (channels, _) = read_back(out, &path);
        assert_eq!(channels[0].2, HASH);
        let got = BagReader::open(&path)
            .unwrap()
            .schema_catalog()
            .expect("catalog");
        let names: Vec<&str> = got.docs.iter().map(|d| d.qualified.as_str()).collect();
        assert_eq!(names, ["go/State"]);
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
            current: BTreeMap::new(),
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
        // A matching input channel is copied whole, fixed size included.
        let ch = BagReader::open(&path)
            .unwrap()
            .channels()
            .unwrap()
            .into_iter()
            .find(|c| c.topic == "/state")
            .expect("channel");
        assert_eq!(ch.schema_name, "go/State");
        assert_eq!(ch.descriptor.expect("descriptor").wire_fixed_size, 8);
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
