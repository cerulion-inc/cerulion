// SPDX-License-Identifier: AGPL-3.0-only
//! [`BagWriter`] — a hand-rolled, chunk-buffering MCAP writer.
//!
//! # Why hand-rolled
//!
//! The `mcap` crate's `Writer` gives no control over chunk boundaries, and
//! Cerulion's bag format is a byte-determinism contract (below) plus a
//! torn-tail-readable crash model, so this crate assembles the MCAP byte
//! stream itself: message framing AND message payloads are appended, in write
//! order, to one contiguous chunk arena that is handed to `writev(2)` when the
//! chunk closes.
//!
//! # Determinism (format contract)
//!
//! The writer contains ZERO wall-clock reads. Channel ids are assigned by sorted
//! topic name for the CONSTRUCTION set and in registration order for topics
//! added later with [`register_topic`](BagWriter::register_topic); schema ids by
//! sorted schema identity, then in registration order for a new identity; and
//! every timestamp comes from the caller. Every id and every record POSITION is
//! therefore a function of the call sequence, so two identical input sequences
//! produce byte-identical files (the `bag_determinism_test` gate). Chunk
//! boundaries are the caller's: [`flush_chunk`](BagWriter::flush_chunk) closes
//! one, and the writer closes one on its own ONLY at the size threshold
//! (`chunk_max_bytes`) — never on a clock it cannot have.
//!
//! What this does NOT claim: that two RECORDER runs match. They never
//! did — message arrival and chunk boundaries are properties of the live system,
//! not of this writer — and a recorder that registers a topic when its first
//! frame arrives makes the late ids a function of arrival order too. The
//! contract is over the WRITER's call sequence.
//!
//! # Payloads are COPIED into the chunk arena
//!
//! [`BagWriter::write_message`] copies its payload parts into `chunk_frames`
//! immediately, so **the caller's buffer is free the instant the call returns**
//! and carries no lifetime obligation at all.
//!
//! It did not always work that way. The writer used to stash raw pointers into
//! the caller's buffers and dereference them at `writev` time, behind a
//! compiler-enforced `'buf` contract (a scoped-chunk API whose closure
//! parameter pinned every payload for the whole call) plus a drop guard that
//! discarded the pending chunk on panic so no stashed pointer could outlive its
//! buffer. That bought ONE user-space copy — and cost the recorder a whole
//! second write mode, because a payload pointer that must stay valid until the
//! disk write completes is a shared-memory sample that must stay BORROWED until
//! then, and iceoryx2 bakes a subscriber's borrow budget into a service at
//! creation, by its producer. A topic created by someone else (an `ros2 attach`
//! bridge route, any foreign publisher) is provisioned at the stock budget of
//! 2, which cannot afford the recorder's concurrent in-flight batches — so the
//! recorder fell back to writing on its own drain thread and lost ~25% of a
//! live firehose.
//!
//! The copy that replaces it is measured to be free at recording rates: every
//! recorded byte is ALREADY streamed through the CPU twice here (`chunk_crc`
//! and `data_crc`) and copied once by the kernel at `writev`, against a
//! measured ~2.8 MB/s bag rate. What it buys is that a recorded frame's owner
//! is released at record time, which is what lets ONE write path serve every
//! producer whatever budget it was created with.
//!
//! Chunk-scoped writing survives as a CONVENIENCE ([`BagWriter::write_chunk`] /
//! [`ChunkScope`]): "these messages form one chunk". It carries no lifetime
//! contract any more, and its drop guard is now purely error containment — a
//! failed or panicking closure discards the pending chunk rather than leaving a
//! half-recorded one for the next flush to emit.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::io::{self, IoSlice, Write};
use std::path::Path;

use cerulion_core::state_ring::STATE_RECORD_SIZE;
use cerulion_core::trace_ring::{TraceRingRecord, TRACE_RECORD_SIZE};

use crate::error::{BagError, BagResult};
use crate::producers::PRODUCER_RECORD_SIZE;
use crate::provisioning::ChannelProvisioning;
use crate::record::{
    self, encode_attachment, encode_attachment_index, encode_channel, encode_chunk_header,
    encode_chunk_index, encode_data_end, encode_footer_prefix, encode_header, encode_message_frame,
    encode_message_index, encode_schema, encode_statistics, encode_summary_offset, op,
    MessageIndexOffset,
};
use crate::schema::{
    SchemaDescriptor, FRAME_PRODUCERS_SCHEMA, FRAME_PRODUCERS_TOPIC, NONDETERMINISM_SCHEMA,
    NONDETERMINISM_TOPIC, RESERVED_PREFIX, SCHEDULER_TRACE_SCHEMA, SCHEDULER_TRACE_TOPIC,
    SCHEMA_ENCODING, STATE_SCHEMA, STATE_TOPIC,
};

/// The fixed MCAP `profile` string for a Cerulion bag.
pub const PROFILE: &str = "cerulion";
/// Default chunk flush threshold: 4 MiB of uncompressed body.
pub const DEFAULT_CHUNK_MAX_BYTES: usize = 4 * 1024 * 1024;
/// Fallback `IOV_MAX` when `sysconf(_SC_IOV_MAX)` is unavailable — the POSIX
/// guaranteed minimum.
const IOV_MAX_FALLBACK: usize = 16;

// ===========================================================================
// The syscall seam
// ===========================================================================

/// Outcome of one `writev(2)`-equivalent syscall.
#[derive(Debug)]
pub enum WritevOutcome {
    /// Accepted this many bytes (may be short, may be 0).
    Wrote(usize),
    /// Interrupted before writing (`EINTR`) — retry with the same iovecs.
    Interrupted,
    /// Failed with this `errno`.
    Failed(i32),
}

/// The write seam behind [`BagWriter`]. Abstracted so the partial-write / EINTR
/// / `IOV_MAX`-batching loop (`flush_iovecs`) is deterministically testable
/// against a scripted in-memory sink.
///
/// `#[doc(hidden)]`-style visibility: `pub` so integration tests can inject a
/// sink under the `test-helpers` feature, but not part of the stable surface.
pub trait WritevSink {
    /// Perform ONE `writev`-equivalent call over `iovs` (already capped to
    /// [`iov_max`](Self::iov_max) entries by the caller).
    fn writev_once(&mut self, iovs: &[IoSlice<'_>]) -> WritevOutcome;
    /// The maximum iovec count a single [`writev_once`](Self::writev_once) may
    /// be handed.
    fn iov_max(&self) -> usize;
    /// Append cold-path framed bytes (header, schemas, channels, message
    /// indexes, DataEnd, summary) — no zero-copy payload involved.
    fn write_bytes(&mut self, buf: &[u8]) -> io::Result<()>;
    /// Flush + fsync the stream (called by [`BagWriter::finalize`]).
    fn sync_all(&mut self) -> io::Result<()>;
}

/// The shared write loop: batch `iovs` into `iov_max`-sized `writev` calls,
/// resume across partial writes (advancing base/len through the array), retry
/// on `EINTR`, and treat a no-progress `0` as a hard error rather than spinning
/// or silently truncating. A release-mode `assert!` (not `debug_assert!`) pins
/// that the total written equals the total requested.
pub(crate) fn flush_iovecs<S: WritevSink + ?Sized>(
    sink: &mut S,
    iovs: &mut [IoSlice<'_>],
    start_offset: u64,
) -> BagResult<u64> {
    let expected: u64 = iovs.iter().map(|s| s.len() as u64).sum();
    let iov_max = sink.iov_max().max(1);
    let mut written: u64 = 0;
    let mut cur: &mut [IoSlice<'_>] = iovs;
    while !cur.is_empty() {
        let batch_len = cur.len().min(iov_max);
        match sink.writev_once(&cur[..batch_len]) {
            WritevOutcome::Interrupted => continue, // EINTR: retry same window
            WritevOutcome::Failed(errno) => {
                return Err(BagError::Writev {
                    errno,
                    offset: start_offset + written,
                    iov_count: batch_len,
                    msg: io::Error::from_raw_os_error(errno).to_string(),
                });
            }
            WritevOutcome::Wrote(0) => {
                return Err(BagError::WritevNoProgress {
                    offset: start_offset + written,
                    remaining: expected - written,
                });
            }
            WritevOutcome::Wrote(n) => {
                written += n as u64;
                IoSlice::advance_slices(&mut cur, n);
            }
        }
    }
    // Release-mode backstop (NOT debug_assert): a short flush must never pass
    // silently — it would corrupt the bag.
    assert!(
        written == expected,
        "bag flush wrote {written} bytes but expected {expected} at offset {start_offset}"
    );
    Ok(written)
}

/// The real `File`-backed sink: `writev(2)` for the zero-copy chunk body,
/// `write(2)` for framed sections, `fsync(2)` on close.
pub struct FileSink {
    file: std::fs::File,
    iov_max: usize,
}

impl FileSink {
    /// Create/truncate `path` for writing.
    pub fn create(path: impl AsRef<Path>) -> io::Result<Self> {
        Self::from_file(std::fs::File::create(path)?)
    }

    /// Adopt an ALREADY-OPEN, caller-owned file and write through THAT handle.
    ///
    /// [`Self::create`] resolves a NAME, which is the right thing for a
    /// recorder writing a file it owns. It is the wrong thing for a caller that
    /// has to keep writing to the same INODE it claimed: between the claim and
    /// the write, the name can be unlinked and re-created by anyone who can
    /// write to the directory, and a path-resolving open would then write into
    /// the replacement. `cerulion bag migrate` claims its scratch file with
    /// `create_new(true)` and hands the resulting handle here, so its bytes
    /// cannot be redirected by a name it no longer controls.
    ///
    /// The file is used AS GIVEN — not truncated, not repositioned — so the
    /// caller must hand over a freshly-created (or explicitly truncated and
    /// rewound) handle. A `create_new` handle satisfies that by construction.
    pub fn from_file(file: std::fs::File) -> io::Result<Self> {
        // SAFETY: sysconf is a pure query; a non-positive result means the
        // limit is indeterminate, so fall back to the POSIX minimum.
        let raw = unsafe { libc::sysconf(libc::_SC_IOV_MAX) };
        let iov_max = if raw < 1 {
            IOV_MAX_FALLBACK
        } else {
            raw as usize
        };
        Ok(Self { file, iov_max })
    }
}

impl WritevSink for FileSink {
    fn writev_once(&mut self, iovs: &[IoSlice<'_>]) -> WritevOutcome {
        use std::os::unix::io::AsRawFd;
        // SAFETY: `IoSlice` is guaranteed ABI-compatible with `libc::iovec` on
        // Unix, and `iovs.len()` is capped to `iov_max <= IOV_MAX` by the
        // caller. Every buffer behind the iovecs is a live Rust slice borrowed
        // for this call — the writer now owns its chunk bytes
        // outright (payloads are copied into the arena at record time), so
        // there is no caller-lifetime obligation to reason about here.
        let ret = unsafe {
            libc::writev(
                self.file.as_raw_fd(),
                iovs.as_ptr() as *const libc::iovec,
                iovs.len() as libc::c_int,
            )
        };
        if ret < 0 {
            let e = io::Error::last_os_error();
            if e.raw_os_error() == Some(libc::EINTR) {
                WritevOutcome::Interrupted
            } else {
                WritevOutcome::Failed(e.raw_os_error().unwrap_or(libc::EIO))
            }
        } else {
            WritevOutcome::Wrote(ret as usize)
        }
    }

    fn iov_max(&self) -> usize {
        self.iov_max
    }

    fn write_bytes(&mut self, buf: &[u8]) -> io::Result<()> {
        self.file.write_all(buf)
    }

    fn sync_all(&mut self) -> io::Result<()> {
        self.file.flush()?;
        self.file.sync_all()
    }
}

// ===========================================================================
// Configuration + registration
// ===========================================================================

/// Writer configuration. `profile` is fixed to [`PROFILE`]; `library` is
/// caller-supplied (NOT auto-derived from build info — a determinism
/// requirement).
#[derive(Debug, Clone)]
pub struct BagWriterConfig {
    /// Flush the current chunk once its uncompressed body reaches this many
    /// bytes. Defaults to [`DEFAULT_CHUNK_MAX_BYTES`].
    pub chunk_max_bytes: usize,
    /// The MCAP `profile` string. Fixed to [`PROFILE`].
    pub profile: String,
    /// The MCAP Header `library` string. A fixed caller-supplied literal, never
    /// build info.
    pub library: String,
    /// Per-topic PROVISIONING, keyed by topic name.
    ///
    /// Carried HERE rather than on [`TopicSchema`] deliberately: `TopicSchema`
    /// is constructed as a literal in dozens of places across four crates, so a
    /// new required field would be a wide mechanical churn for a value almost
    /// every caller has nothing to say about — whereas `BagWriterConfig` is
    /// built through `Default` at all but two call sites. A topic absent from
    /// this map (the default, empty) emits an EMPTY metadata map, i.e. bytes
    /// identical to a pre-provisioning bag.
    ///
    /// A key in the reserved `__cerulion/` namespace is REFUSED at construction
    /// ([`BagError::ReservedProvisioningKey`]): the lookup runs over the
    /// auto-registered reserved channels too, so such a key would stamp
    /// metadata onto one of them.
    ///
    /// **No shipping producer populates this yet** — `cerulion_bagd` leaves it at
    /// `Default`, and cannot do better, since a recorder observes a topic through
    /// a tap that exposes none of these four values. Filling it has to come
    /// from the side that created the service.
    /// See the [`provisioning`](crate::provisioning) module docs.
    pub provisioning: BTreeMap<String, ChannelProvisioning>,
}

impl Default for BagWriterConfig {
    fn default() -> Self {
        Self {
            chunk_max_bytes: DEFAULT_CHUNK_MAX_BYTES,
            profile: PROFILE.to_string(),
            library: "cerulion_bag".to_string(),
            provisioning: BTreeMap::new(),
        }
    }
}

/// A topic + its schema descriptor — handed to the constructor for the topics
/// known up front, or to [`BagWriter::register_topic`] for one that appears
/// later (the topic set is no longer frozen at construction).
#[derive(Debug, Clone)]
pub struct TopicSchema {
    /// The topic name (e.g. `/camera/image`). Must NOT start with
    /// [`RESERVED_PREFIX`].
    pub topic: String,
    /// The qualified schema name (e.g. `sensor_msgs/Image`).
    pub schema_name: String,
    /// The layout-sensitive schema hash.
    pub schema_hash: u64,
    /// The schema's fixed wire size in bytes.
    pub wire_fixed_size: u32,
}

/// A write target: either a topic name or a resolved channel id.
#[derive(Debug, Clone, Copy)]
pub enum WriteTarget<'a> {
    /// Resolve by topic name.
    Topic(&'a str),
    /// A pre-resolved channel id.
    Channel(u16),
}
impl<'a> From<&'a str> for WriteTarget<'a> {
    fn from(t: &'a str) -> Self {
        WriteTarget::Topic(t)
    }
}
impl<'a> From<&'a String> for WriteTarget<'a> {
    fn from(t: &'a String) -> Self {
        WriteTarget::Topic(t.as_str())
    }
}
impl From<u16> for WriteTarget<'_> {
    fn from(id: u16) -> Self {
        WriteTarget::Channel(id)
    }
}

// ===========================================================================
// Internal state
// ===========================================================================

struct ChannelInfo {
    topic: String,
    schema_id: u16,
    /// The channel's provisioning metadata pairs, already
    /// rendered + sorted. EMPTY for every reserved channel and for any topic
    /// the caller declared nothing about, which is the pre-provisioning encoding.
    ///
    /// The reserved half is ENFORCED, not merely expected: the lookup below
    /// runs over every registration INCLUDING the auto-registered reserved
    /// channels, so `validate_registration` refuses a reserved-prefix
    /// provisioning key up front.
    metadata: Vec<(String, String)>,
}

struct SchemaRecordInfo {
    id: u16,
    name: String,
    data: [u8; crate::schema::DESCRIPTOR_LEN],
}

struct ChunkIndexData {
    message_start_time: u64,
    message_end_time: u64,
    chunk_start_offset: u64,
    chunk_length: u64,
    message_index_offsets: Vec<MessageIndexOffset>,
    message_index_length: u64,
    size: u64, // compressed == uncompressed (never compressed)
}

struct AttachmentIndexData {
    offset: u64,
    length: u64,
    log_time: u64,
    create_time: u64,
    data_size: u64,
    name: String,
    media_type: String,
}

// ===========================================================================
// BagWriter
// ===========================================================================

/// A hand-rolled, zero-copy MCAP writer (see the module docs).
pub struct BagWriter<S: WritevSink = FileSink> {
    sink: S,
    profile: String,
    library: String,
    chunk_max_bytes: u64,

    channels: Vec<ChannelInfo>, // index == channel id
    topic_to_id: HashMap<String, u16>,
    schemas: Vec<SchemaRecordInfo>, // ascending by id
    /// The schema-identity dedup map, PROMOTED from a `with_sink`
    /// local. [`register_topic`](Self::register_topic) applies the same dedup
    /// the constructor does — a late topic whose `(schema name, descriptor)`
    /// pair is already in the table reuses that schema id and mints no second
    /// Schema record.
    schema_id_of: HashMap<(String, [u8; crate::schema::DESCRIPTOR_LEN]), u16>,
    /// The per-topic provisioning map, PROMOTED from `config`.
    /// A late channel is looked up here exactly as a construction-set one is,
    /// so a topic the caller declared nothing about carries an EMPTY metadata
    /// map whenever it is registered.
    provisioning: BTreeMap<String, ChannelProvisioning>,
    /// How many of [`channels`](Self::channels) are the writer's own
    /// auto-registered reserved channels — read off `reserved_channels()` once,
    /// so [`register_topic`](Self::register_topic)'s `TooManyChannels` can
    /// report the same two addends `validate_registration` does without
    /// re-deriving (or hard-coding) the count.
    reserved_count: usize,
    /// TERMINAL once set — the operation whose write failed.
    ///
    /// A failed write may be PARTIAL, leaving the sink's file offset ahead of
    /// `pos`, so every later record would be framed at an offset that does not
    /// exist and the summary's seek offsets would point into the torn record.
    /// Set by the two sites that can leave that state ([`emit_framed`](Self::emit_framed)
    /// and the chunk `writev` in [`flush_chunk`](Self::flush_chunk)) and checked
    /// by every entry point that writes. There is no retry and no reset.
    poisoned: Option<&'static str>,
    scheduler_trace_id: u16,
    /// The `__cerulion/state` channel id, resolved once at
    /// construction for the same reason `scheduler_trace_id` is — the length
    /// gate below has to recognise the channel on EVERY `write_message`, and a
    /// per-call name lookup on a hot path would be paid by every user frame.
    state_id: u16,
    /// The `__cerulion/frame_producers` channel id, resolved once at
    /// construction for the same reason `state_id` is.
    frame_producers_id: u16,

    pos: u64,
    data_crc: crc32fast::Hasher,

    // Current (open) chunk. `chunk_frames` IS the chunk body,
    // contiguously and in write order: message framing and message payloads
    // alternate in it exactly as they appear on disk, so there is no iovec
    // PLAN to keep beside it and no CRC to accumulate incrementally (the flush
    // hashes the buffer, over the identical bytes in the identical order).
    chunk_frames: Vec<u8>,
    chunk_msg_start: Option<u64>,
    chunk_msg_end: Option<u64>,
    chunk_index_entries: BTreeMap<u16, Vec<(u64, u64)>>,
    scratch: Vec<u8>,

    // Summary accumulators.
    chunk_index_records: Vec<ChunkIndexData>,
    attachment_index_records: Vec<AttachmentIndexData>,
    total_message_count: u64,
    per_channel_count: BTreeMap<u16, u64>,
    file_msg_start: Option<u64>,
    file_msg_end: Option<u64>,

    finalized: bool,

    /// TEST SEAM (`test-helpers` feature): make the next `n` CONTENT-bearing
    /// [`flush_chunk`](Self::flush_chunk) calls fail exactly as a `writev(2)`
    /// failure would, then heal.
    ///
    /// It exists because a `flush_chunk` failure is otherwise reachable only by
    /// injecting a [`WritevSink`], and `cerulion_bagd` builds its writer over
    /// the real [`FileSink`] — so the recorder's own "the chunk could not be
    /// closed after a write error" branch, which is the one branch that can
    /// destroy an EARLIER batch's frames, had no test caller anywhere. This is
    /// the same shape as bagd's own `fault_inject_flush_error_after_messages`,
    /// one layer down.
    ///
    /// Fires from the position a real `writev` failure would (after the header
    /// is encoded, before anything is committed), so it also exercises the
    /// error path's state-restoration obligations rather than short-circuiting
    /// past them. Compiled out of production builds; `0` (the default) is inert,
    /// so every existing caller is byte-for-byte unaffected.
    ///
    /// It deliberately does NOT set the `poisoned` latch, and that is
    /// a real difference from a live `writev` failure rather than an oversight.
    /// The latch exists for PARTIAL progress — bytes on disk the writer cannot
    /// account for — and this seam fires before a single byte reaches the sink,
    /// which is why its own contract can promise the caller "discard or RETRY".
    /// The genuinely-partial shape is driven instead through
    /// [`ScriptedSink`](crate::test_sink::ScriptedSink)'s `writev_once` and
    /// `write_bytes` scripts, which fail at the sink itself.
    #[cfg(any(test, feature = "test-helpers"))]
    fault_inject_flush_failures: u32,

    /// TEST SEAM: content-bearing [`flush_chunk`](Self::flush_chunk)
    /// calls to let SUCCEED before `fault_inject_flush_failures` starts firing.
    /// See [`fault_inject_flush_failures_for_test`](Self::fault_inject_flush_failures_for_test).
    #[cfg(any(test, feature = "test-helpers"))]
    fault_inject_flush_skip: u32,
}

impl BagWriter<FileSink> {
    /// Create a bag at `path`. `topics` are the channels registered in the
    /// PRELUDE; any topic may be added later with
    /// [`register_topic`](BagWriter::register_topic). Rejects a topic
    /// in the reserved `__cerulion/` namespace, duplicate topics, a
    /// `config.provisioning` KEY in that same reserved namespace, and a
    /// registration whose total channel count (user topics plus the reserved
    /// channels) exceeds the `u16` channel-id space. The reserved channels are
    /// registered automatically.
    /// Writes the leading magic, Header, Schema and Channel records
    /// immediately.
    pub fn create(
        path: impl AsRef<Path>,
        config: BagWriterConfig,
        topics: &[TopicSchema],
    ) -> BagResult<Self> {
        // Validate the registration BEFORE touching the filesystem so an
        // invalid config never leaves a stray empty file behind.
        validate_registration(topics, &config.provisioning)?;
        let sink = FileSink::create(path)?;
        Self::with_sink(sink, config, topics)
    }
}

/// The channel-id space: a channel id is a 16-bit MCAP field, so a bag can
/// address exactly `u16::MAX + 1` channels.
const CHANNEL_ID_SPACE: usize = u16::MAX as usize + 1;

/// The USABLE schema-id space: a schema id is also a 16-bit MCAP field, but id
/// 0 means "no schema", so ids are assigned from 1 and only `u16::MAX` of them
/// can be addressed — one FEWER than [`CHANNEL_ID_SPACE`], where id 0 is an
/// ordinary channel.
const USABLE_SCHEMA_IDS: usize = u16::MAX as usize;

/// The reserved channels the writer auto-registers, in DECLARATION order
/// ([`BagWriter::with_sink`] sorts them by name before assigning ids).
///
/// ONE definition, because two things have to agree about how many there are:
/// `with_sink` REGISTERS them, and [`validate_registration`] PRICES them
/// against [`CHANNEL_ID_SPACE`]. A literal in the second would go stale the
/// next time a reserved channel is added — which is exactly the move that
/// shifted the boundary when the fourth was added.
fn reserved_channels() -> [(&'static str, &'static str, SchemaDescriptor); 4] {
    [
        (
            SCHEDULER_TRACE_TOPIC,
            SCHEDULER_TRACE_SCHEMA,
            SchemaDescriptor::new(0, TRACE_RECORD_SIZE),
        ),
        (
            NONDETERMINISM_TOPIC,
            NONDETERMINISM_SCHEMA,
            SchemaDescriptor::new(0, 0),
        ),
        // The node-state checkpoint stream.
        (
            STATE_TOPIC,
            STATE_SCHEMA,
            // `wire_fixed_size` is the record's fixed size, which for this
            // channel is a real number rather than the trace channel's
            // stand-in: every message on it is exactly one 512-byte
            // `StateRecordHeader` + payload. `schema_hash` is 0 for the
            // same reason its siblings' is — the payload is a framework
            // record, not a user schema, so there is no recipe-3 hash to
            // carry and fabricating one would make a reader think it could
            // look the layout up.
            SchemaDescriptor::new(0, STATE_RECORD_SIZE),
        ),
        // The record-time producer labels for
        // `multi_publisher_topics` topics. Sorts FIRST of the reserved set
        // (`f` < `n` < `s`), so it takes the lowest reserved id and shifts
        // its three siblings up by one; the ids are derived rather than
        // frozen, so that shift is the format doing what it says.
        (
            FRAME_PRODUCERS_TOPIC,
            FRAME_PRODUCERS_SCHEMA,
            SchemaDescriptor::new(0, PRODUCER_RECORD_SIZE as u32),
        ),
    ]
}

/// Reject reserved-prefix user topics, duplicates, reserved-prefix
/// PROVISIONING keys, and a registration too large for the channel-id space.
/// Called before the file is opened (in [`BagWriter::create`]) and again in
/// [`BagWriter::with_sink`] (the sink-injection entry point).
///
/// The provisioning half takes BOTH inputs through this ONE function rather
/// than a second `validate_*` call the constructors must each remember: there
/// are two entry points, and a guard a caller can forget is a guard that
/// eventually is.
fn validate_registration(
    topics: &[TopicSchema],
    provisioning: &BTreeMap<String, ChannelProvisioning>,
) -> BagResult<()> {
    let mut seen = std::collections::HashSet::new();
    for t in topics {
        if t.topic.starts_with(RESERVED_PREFIX) {
            return Err(BagError::ReservedTopicPrefix {
                topic: t.topic.clone(),
            });
        }
        if !seen.insert(t.topic.as_str()) {
            return Err(BagError::DuplicateTopic {
                topic: t.topic.clone(),
            });
        }
    }
    // The provisioning map is keyed by TOPIC and is looked
    // up over `regs`, which INCLUDES the auto-registered reserved channels
    // — so a reserved key would stamp metadata onto a reserved Channel record,
    // contradicting `ChannelInfo::metadata`'s "EMPTY for every reserved
    // channel" and breaking those channels' byte-identity with older bags.
    // The topic guard above cannot see it: it walks the CALLER's `topics`
    // slice, and a provisioning key need not appear there at all.
    //
    // FAIL-CLOSED rather than trusted: no shipping caller populates this map
    // (`cerulion_bagd` leaves it at `Default`), so today the guard is
    // unreachable in production. The play-fidelity writer is the named future
    // caller, and this is the boundary it will meet.
    //
    // `BTreeMap` iterates in KEY order, so with several offending keys the one
    // reported is deterministic run to run.
    for key in provisioning.keys() {
        if key.starts_with(RESERVED_PREFIX) {
            return Err(BagError::ReservedProvisioningKey { topic: key.clone() });
        }
    }

    // The channel-id space is 16 bits, and BOTH id-assignment
    // sites derive an id from an index with `as u16` — so a registration whose
    // total channel count exceeds the space WRAPS silently, handing id 0 to a
    // second channel and making every message on it read back as the
    // canonically-first user topic's. That total is `topics + reserved`, so the
    // fourth reserved channel really did move the boundary when it was added;
    // the count is read off the ARRAY rather than written here, because a
    // literal is what would go stale at the fifth.
    //
    // LAST of the four guards deliberately. The three above each name a
    // specific thing to fix (a topic to rename, a duplicate to drop, a
    // provisioning entry to delete), and a registration can trip both — so
    // reporting the actionable name first, with the scale condition as the
    // structural backstop, is strictly more useful than the reverse. Nothing
    // above assigns an id, so "before any id is assigned" holds either way.
    //
    // The check prices the list AS GIVEN, duplicates included: every entry
    // becomes its own channel registration, so `topics.len()` IS the count that
    // would be assigned (and a duplicate is refused above regardless).
    let reserved_table = reserved_channels();
    let reserved = reserved_table.len();
    let total = topics.len().saturating_add(reserved);
    if total > CHANNEL_ID_SPACE {
        return Err(BagError::TooManyChannels {
            topics: topics.len(),
            reserved,
            total,
            cap: CHANNEL_ID_SPACE,
        });
    }

    // The SCHEMA-id space is the same `as u16` hazard one
    // table over — `with_sink` assigns schema ids as `(i + 1) as u16` over the
    // DEDUPED identities — and the channel cap above does not bound it, because
    // the two are counted differently: channels per registration ENTRY, schemas
    // per distinct `(schema name, descriptor)`. The cap is one SMALLER (MCAP's
    // id 0 means "no schema"), so the one shape that overruns it is a
    // registration that exactly FILLS the channel space with all-distinct
    // schemas: `CHANNEL_ID_SPACE` identities against `USABLE_SCHEMA_IDS` ids.
    //
    // Counted HERE, ahead of `with_sink`'s dedup, for two reasons — and both
    // are about the REFUSAL rather than about tidiness. It keeps this the ONE
    // guard function `create` runs before opening the file, so an over-cap
    // registration leaves no stray bag (every sibling refusal above has that
    // property, and a validation-class error that lands after file creation
    // would be the odd one out). And it makes the refusal REACHABLE in bounded
    // time: `with_sink`'s dedup is an O(n^2) `Vec::contains` scan, so a
    // registration large enough to trip this cap would spend minutes inside it
    // before any post-dedup check could fire.
    //
    // This is a cheap PRE-COUNT, not a second dedup: it computes a COUNT with a
    // hash set in O(n), while the dedup below must also produce the sorted list
    // and the id map. The identity is the same pair, and the AUTHORITATIVE
    // check still sits at the dedup seam where `unique` really exists — so if
    // the two ever disagree, the registration is refused there rather than
    // wrapping.
    let mut identities: HashSet<(&str, [u8; crate::schema::DESCRIPTOR_LEN])> =
        HashSet::with_capacity(total);
    for t in topics {
        identities.insert((
            t.schema_name.as_str(),
            SchemaDescriptor::new(t.schema_hash, t.wire_fixed_size).encode(),
        ));
    }
    for (_, schema_name, descriptor) in reserved_table {
        identities.insert((schema_name, descriptor.encode()));
    }
    if identities.len() > USABLE_SCHEMA_IDS {
        return Err(BagError::TooManySchemas {
            schemas: identities.len(),
            cap: USABLE_SCHEMA_IDS,
        });
    }
    Ok(())
}

impl<S: WritevSink> BagWriter<S> {
    /// Construct over an arbitrary [`WritevSink`] (the injection seam used by
    /// the syscall tests). Prefer [`BagWriter::create`] in production.
    pub fn with_sink(sink: S, config: BagWriterConfig, topics: &[TopicSchema]) -> BagResult<Self> {
        validate_registration(topics, &config.provisioning)?;

        // --- assign channel ids: user topics by sorted name (0..N), then the
        //     reserved channels by sorted name (N..) ---
        let mut sorted_user: Vec<&TopicSchema> = topics.iter().collect();
        sorted_user.sort_by(|a, b| a.topic.cmp(&b.topic));

        // Reserved channels, appended after user channels, ordered by name.
        // EVERY one is registered UNCONDITIONALLY — a channel table that varied
        // with whether a run happened to checkpoint (or to carry a
        // multi-publisher topic) would make the channel-id assignment a
        // function of run-time configuration, and every id here is derived from
        // the SORTED name list precisely so it is a function of the
        // registration alone.
        //
        // The table itself lives in `reserved_channels()` so that
        // `validate_registration`'s channel-id-space check counts the SAME
        // array this loop registers.
        let mut reserved = reserved_channels();
        reserved.sort_by(|a, b| a.0.cmp(b.0));
        // Read the count off the ARRAY, here, for the same reason
        // `validate_registration` does — a literal is what goes stale at the
        // fifth reserved channel. `register_topic`'s `TooManyChannels` reports
        // the two addends from it.
        let reserved_count = reserved.len();

        // (topic, schema_name, descriptor) tuples in final channel-id order.
        struct Reg {
            topic: String,
            schema_name: String,
            descriptor: SchemaDescriptor,
        }
        let mut regs: Vec<Reg> = Vec::with_capacity(topics.len() + reserved.len());
        for t in &sorted_user {
            regs.push(Reg {
                topic: t.topic.clone(),
                schema_name: t.schema_name.clone(),
                descriptor: SchemaDescriptor::new(t.schema_hash, t.wire_fixed_size),
            });
        }
        for (topic, schema_name, descriptor) in reserved {
            regs.push(Reg {
                topic: topic.to_string(),
                schema_name: schema_name.to_string(),
                descriptor,
            });
        }

        // --- dedup schemas by (name, descriptor bytes), assign ids by sorted
        //     identity starting at 1 (id 0 means "no schema" in MCAP) ---
        let mut unique: Vec<(String, [u8; crate::schema::DESCRIPTOR_LEN])> = Vec::new();
        for r in &regs {
            let key = (r.schema_name.clone(), r.descriptor.encode());
            if !unique.contains(&key) {
                unique.push(key);
            }
        }
        unique.sort();
        // The AUTHORITATIVE schema-id-space check, at the one
        // place the deduped set really exists — the loop below assigns
        // `(i + 1) as u16`, so one identity past `USABLE_SCHEMA_IDS` wraps to
        // id 0, which MCAP reads as "no schema".
        //
        // `validate_registration`'s pre-count is what refuses this BEFORE the
        // file is opened, and on every shipping path it fires first, so this
        // arm is unreachable in production today (the same
        // fail-closed-rather-than-trusted footing as the reserved-provisioning
        // guard above). It is kept because the two read the identity through
        // DIFFERENT expressions: if the dedup key ever changes and the
        // pre-count is not changed with it, the pre-count UNDER-counts and
        // fails OPEN — and this check, which reads `unique` itself, cannot.
        if unique.len() > USABLE_SCHEMA_IDS {
            return Err(BagError::TooManySchemas {
                schemas: unique.len(),
                cap: USABLE_SCHEMA_IDS,
            });
        }
        let mut schema_id_of: HashMap<(String, [u8; crate::schema::DESCRIPTOR_LEN]), u16> =
            HashMap::new();
        let mut schemas: Vec<SchemaRecordInfo> = Vec::with_capacity(unique.len());
        for (i, (name, data)) in unique.into_iter().enumerate() {
            let id = (i + 1) as u16;
            schema_id_of.insert((name.clone(), data), id);
            schemas.push(SchemaRecordInfo { id, name, data });
        }

        // --- build channel table (id == index) ---
        let mut channels = Vec::with_capacity(regs.len());
        let mut topic_to_id = HashMap::with_capacity(regs.len());
        for (id, r) in regs.iter().enumerate() {
            let schema_id = schema_id_of[&(r.schema_name.clone(), r.descriptor.encode())];
            // A topic the caller said nothing about carries an EMPTY
            // map, byte-identical to the pre-provisioning encoding. `regs`
            // INCLUDES the two reserved channels, so their emptiness rests on
            // `validate_registration` having refused a reserved-prefix key
            // rather than on this lookup missing.
            let metadata = config
                .provisioning
                .get(&r.topic)
                .map(ChannelProvisioning::to_metadata)
                .unwrap_or_default();
            channels.push(ChannelInfo {
                topic: r.topic.clone(),
                schema_id,
                metadata,
            });
            topic_to_id.insert(r.topic.clone(), id as u16);
        }
        let scheduler_trace_id = topic_to_id[SCHEDULER_TRACE_TOPIC];
        let state_id = topic_to_id[STATE_TOPIC];
        let frame_producers_id = topic_to_id[FRAME_PRODUCERS_TOPIC];

        let mut w = Self {
            sink,
            profile: config.profile,
            library: config.library,
            chunk_max_bytes: config.chunk_max_bytes as u64,
            channels,
            topic_to_id,
            schemas,
            schema_id_of,
            provisioning: config.provisioning,
            reserved_count,
            poisoned: None,
            scheduler_trace_id,
            state_id,
            frame_producers_id,
            pos: 0,
            data_crc: crc32fast::Hasher::new(),
            chunk_frames: Vec::new(),
            chunk_msg_start: None,
            chunk_msg_end: None,
            chunk_index_entries: BTreeMap::new(),
            scratch: Vec::new(),
            chunk_index_records: Vec::new(),
            attachment_index_records: Vec::new(),
            total_message_count: 0,
            per_channel_count: BTreeMap::new(),
            file_msg_start: None,
            file_msg_end: None,
            finalized: false,
            #[cfg(any(test, feature = "test-helpers"))]
            fault_inject_flush_skip: 0,
            #[cfg(any(test, feature = "test-helpers"))]
            fault_inject_flush_failures: 0,
        };
        w.write_prelude()?;
        Ok(w)
    }

    /// TEST SEAM — see the `fault_inject_flush_failures` field. The next `n`
    /// content-bearing [`flush_chunk`](Self::flush_chunk) calls fail exactly as
    /// a `writev(2)` failure would; the rest succeed.
    ///
    /// `skip` lets the first `skip` content-bearing flushes SUCCEED
    /// before the failures begin. `write_batch`'s error path calls `flush_chunk`
    /// TWICE in a row — once to close the durable prefix, once to close the
    /// salvaged remainder — and its three frame-destroying branches are told
    /// apart by WHICH of those two fails. A count alone can only ever fail the
    /// first, which is why the second and third branches had no test caller.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fault_inject_flush_failures_for_test(&mut self, skip: u32, n: u32) {
        self.fault_inject_flush_skip = skip;
        self.fault_inject_flush_failures = n;
    }

    /// Refuse every write once the writer is poisoned.
    ///
    /// Called at the top of each entry point that can reach the sink, BEFORE it
    /// touches any state, so a poisoned writer is inert rather than accumulating
    /// arena bytes and table rows it can never emit.
    ///
    /// # The classified surface
    ///
    /// The rule is UNIFORM — every `pub fn` that can reach the sink calls this
    /// as its FIRST statement — because the alternative is a case-by-case
    /// argument, and a case-by-case argument is what let
    /// [`write_scheduler_trace`](Self::write_scheduler_trace) ship unguarded: it
    /// is the one entry point that appends to the arena without going through
    /// [`write_message`](Self::write_message), and being the sixth of a set
    /// described as five is precisely how it was missed.
    ///
    /// **Self-guards (8):** [`register_topic`](Self::register_topic),
    /// [`write_message`](Self::write_message),
    /// [`write_scheduler_trace`](Self::write_scheduler_trace),
    /// [`write_chunk`](Self::write_chunk), [`flush_chunk`](Self::flush_chunk),
    /// [`write_attachment`](Self::write_attachment),
    /// [`write_schema_catalog`](Self::write_schema_catalog),
    /// [`finalize`](Self::finalize).
    ///
    /// **Cannot reach the sink, deliberately unguarded (4):**
    /// [`discard_pending_chunk`](Self::discard_pending_chunk) only CLEARS
    /// pending state — it never appends and never writes, `ChunkScope`'s `Drop`
    /// calls it where no error can be returned, and clearing a dead writer's
    /// arena is the right thing anyway; `ChunkScope`'s two writers delegate to
    /// the guarded `BagWriter` twins in their single statement;
    /// `fault_inject_flush_failures_for_test` sets two counters behind
    /// `#[cfg(any(test, feature = "test-helpers"))]`.
    ///
    /// **Constructors (4):** [`create`](BagWriter::create) and
    /// [`with_sink`](Self::with_sink) — there is no writer yet to poison — plus
    /// `FileSink`'s two, which build a sink rather than a writer.
    ///
    /// **Read-only (11):** the accessors, plus the `sink()` test seam.
    ///
    /// **Crate-internal (1):** `flush_iovecs`, the `writev` loop, reached only
    /// from the guarded `flush_chunk`.
    ///
    /// `bag_validation_test::every_public_writer_entry_point_is_classified_for_the_poison_latch`
    /// walks this file and fails until a NEW `pub fn` is added to that
    /// inventory, and `a_poisoned_writer_refuses_every_later_call` drives each
    /// of the eight guarded entry points separately — a check missing from any
    /// one of them lets that path write at an offset that does not exist.
    ///
    /// # Why four of the eight checks are redundant today
    ///
    /// [`write_attachment`](Self::write_attachment) and
    /// [`finalize`](Self::finalize) call [`flush_chunk`](Self::flush_chunk)
    /// first, [`write_chunk`](Self::write_chunk) ends in it, and
    /// [`write_schema_catalog`](Self::write_schema_catalog) delegates to
    /// `write_attachment` — and `flush_chunk`'s own check is its FIRST statement
    /// (ahead of the empty-arena early return), so each would refuse a poisoned
    /// writer with no check of its own. MEASURED, not assumed — deleting either
    /// of the original two ALONE kills no test, while deleting it together with
    /// `flush_chunk`'s fails the very assertions that passed before, which is
    /// what shows the oracle has no hole and the guard is simply duplicated.
    ///
    /// They stay because the alternative makes a safety property depend on an
    /// implementation detail: `flush_chunk` already returns early on an empty
    /// arena, so an entirely reasonable future edit — skip the flush when
    /// nothing is pending — would silently delete the guard from all four paths.
    /// An entry point that self-guards cannot lose it that way.
    fn check_poisoned(&self) -> BagResult<()> {
        match self.poisoned {
            Some(after) => Err(BagError::Poisoned { after }),
            None => Ok(()),
        }
    }

    /// Emit framed bytes: write to the sink, THEN fold into the data-section
    /// CRC and advance `pos`. INVARIANT: `data_crc` covers exactly the bytes
    /// durably handed to the sink — fold/advance only after the write
    /// succeeds, so an I/O error never leaves ghost bytes in the running CRC
    /// (see `writev_chunk_records` for the chunk-path half).
    ///
    /// A FAILED write POISONS the writer. `write_all` may be partial,
    /// so on error the sink's file offset can be anywhere between `pos` and
    /// `pos + buf.len()` while `data_crc` and `pos` still describe the prefix —
    /// the two have diverged and nothing can measure by how much. Every later
    /// record would be framed at a file position that does not exist, and the
    /// summary's ChunkIndex/AttachmentIndex offsets (which readers SEEK to) would
    /// address the middle of the torn record. The latch is set HERE, at the one
    /// place that knows the write failed, rather than at each caller.
    fn emit_framed(&mut self, buf: &[u8]) -> BagResult<()> {
        if let Err(e) = self.sink.write_bytes(buf) {
            self.poisoned = Some("a failed framed-record write");
            return Err(e.into());
        }
        self.data_crc.update(buf);
        self.pos += buf.len() as u64;
        Ok(())
    }

    /// Write the leading magic, Header, and the CONSTRUCTION set's Schema and
    /// Channel records into the data section. A topic registered later
    /// ([`register_topic`](Self::register_topic)) writes its own pair at the
    /// position it is registered at, not here.
    fn write_prelude(&mut self) -> BagResult<()> {
        self.scratch.clear();
        self.scratch.extend_from_slice(&record::MAGIC);
        encode_header(&mut self.scratch, &self.profile, &self.library);
        for s in &self.schemas {
            encode_schema(&mut self.scratch, s.id, &s.name, SCHEMA_ENCODING, &s.data);
        }
        for (id, c) in self.channels.iter().enumerate() {
            encode_channel(
                &mut self.scratch,
                id as u16,
                c.schema_id,
                &c.topic,
                SCHEMA_ENCODING,
                &c.metadata,
            );
        }
        // `scratch` is borrowed above; move it out to satisfy the borrow
        // checker, emit, then restore the (now-empty) arena for reuse.
        let prelude = std::mem::take(&mut self.scratch);
        self.emit_framed(&prelude)?;
        self.scratch = prelude;
        self.scratch.clear();
        Ok(())
    }

    /// The channel id for a topic, if registered.
    pub fn channel_id(&self, topic: &str) -> Option<u16> {
        self.topic_to_id.get(topic).copied()
    }

    /// Register a topic AFTER construction, so a producer that
    /// appears mid-run is recorded from that moment.
    ///
    /// Emits the topic's Schema record (only if its `(schema_name, descriptor)`
    /// identity is new — the same dedup [`with_sink`](Self::with_sink) applies)
    /// and its Channel record as TOP-LEVEL data-section records at the current
    /// write position, via `emit_framed`, WITHOUT closing the open chunk: the
    /// open arena is written LATER at a higher offset ([`flush_chunk`](Self::flush_chunk)
    /// captures `chunk_start` at flush time, and MessageIndex offsets are
    /// arena-relative), so in file order Schema precedes Channel precedes the
    /// chunk holding the first Message on it — the only order any linear reader
    /// enforces.
    ///
    /// # Why top-level rather than inside the open chunk
    ///
    /// Three concrete hazards, each of which the in-arena form really has:
    ///
    /// - `flush_chunk` early-returns when `chunk_index_entries` is empty, so a
    ///   registration written into an otherwise-empty arena would be silently
    ///   DISCARDED while the channel table (and therefore the summary) named it.
    /// - [`discard_pending_chunk`](Self::discard_pending_chunk) — which runs on
    ///   every `write_chunk` error and on closure panic — would DESTROY an
    ///   in-arena registration while `channels`/`topic_to_id` kept it, so the
    ///   caller's salvage retry would write frames onto a channel no reader can
    ///   resolve.
    /// - A message-less chunk folds `unwrap_or(0)` into the file time bounds.
    ///
    /// Top-level emission has none of them: the record is durable before this
    /// method returns. (Upstream's `mcap::Writer` puts Schema/Channel INSIDE its
    /// open chunk, which is what proves a reader tolerates either placement; it
    /// does not make in-arena right for THIS writer, whose arena can be
    /// discarded.) Nothing in the arena holds an absolute file offset, so no
    /// flush is needed and a burst of registrations adds no chunk boundary.
    ///
    /// # Ids
    ///
    /// The construction set keeps sorted-name assignment; a topic registered
    /// here takes the NEXT id, `channels.len()` — REGISTRATION order, which is
    /// the only defined answer, since there is no ordering over topics that have not
    /// appeared yet. A new schema identity takes `schemas.len() + 1`.
    ///
    /// # Failure
    ///
    /// Tables are mutated ONLY after the bytes are durably written, so the
    /// summary can never name a channel the data section never received. A
    /// registration whose write FAILS is TERMINAL for this writer, exactly like
    /// a failed attachment or MessageIndex write: the sink's file offset may be
    /// ahead of `pos`, so no later write is trustworthy — the writer latches
    /// [`BagError::Poisoned`] and refuses every further `write_*` /
    /// `register_topic` / `flush_chunk` / `finalize`. There is NO retry.
    ///
    /// Refuses a reserved-prefix topic, an already-registered topic, and a
    /// registration that would exhaust the `u16` channel- or schema-id space,
    /// with the SAME variants [`create`](BagWriter::create) uses. Reads no clock.
    pub fn register_topic(&mut self, topic: &TopicSchema) -> BagResult<u16> {
        // --- 1. Guards. Every one runs BEFORE any encoding or state change, so
        //        a refused registration leaves the writer byte-for-byte and
        //        table-for-table as it was.
        self.check_poisoned()?;
        if topic.topic.starts_with(RESERVED_PREFIX) {
            return Err(BagError::ReservedTopicPrefix {
                topic: topic.topic.clone(),
            });
        }
        // A second channel for one NAME is refused outright rather than
        // tolerated as an alias: every consumer keys by topic name, so two
        // channels sharing a name make "the messages on /x" ambiguous with no
        // way for a reader to pick.
        if self.topic_to_id.contains_key(&topic.topic) {
            return Err(BagError::DuplicateTopic {
                topic: topic.topic.clone(),
            });
        }
        // The `as u16` id derivation below has the same wrap hazard the
        // constructor guards: one channel past the space silently gives two
        // channels id 0. `reserved_count` is carried rather than recomputed so
        // the two addends match what `validate_registration` reports.
        if self.channels.len() >= CHANNEL_ID_SPACE {
            return Err(BagError::TooManyChannels {
                topics: self.channels.len() - self.reserved_count,
                reserved: self.reserved_count,
                total: self.channels.len() + 1,
                cap: CHANNEL_ID_SPACE,
            });
        }
        let descriptor = SchemaDescriptor::new(topic.schema_hash, topic.wire_fixed_size).encode();
        let identity = (topic.schema_name.clone(), descriptor);
        let existing_schema_id = self.schema_id_of.get(&identity).copied();
        // Only a NEW identity consumes a schema id, so a late topic sharing a
        // type with an already-registered one can always be added.
        if existing_schema_id.is_none() && self.schemas.len() >= USABLE_SCHEMA_IDS {
            return Err(BagError::TooManySchemas {
                schemas: self.schemas.len() + 1,
                cap: USABLE_SCHEMA_IDS,
            });
        }

        // --- 2. Encode both records into ONE scratch buffer. Schema FIRST: a
        //        linear reader resolves a Channel's `schema_id` against the
        //        schemas it has already seen.
        let channel_id = self.channels.len() as u16;
        let schema_id = existing_schema_id.unwrap_or((self.schemas.len() + 1) as u16);
        let metadata = self
            .provisioning
            .get(&topic.topic)
            .map(ChannelProvisioning::to_metadata)
            .unwrap_or_default();

        self.scratch.clear();
        if existing_schema_id.is_none() {
            encode_schema(
                &mut self.scratch,
                schema_id,
                &topic.schema_name,
                SCHEMA_ENCODING,
                &descriptor,
            );
        }
        encode_channel(
            &mut self.scratch,
            channel_id,
            schema_id,
            &topic.topic,
            SCHEMA_ENCODING,
            &metadata,
        );

        // --- 3. ONE write, at the current position, without closing the chunk.
        //        `scratch` is moved out and back so the borrow checker sees
        //        disjoint borrows (the same dance `write_prelude` does).
        let buf = std::mem::take(&mut self.scratch);
        let emitted = self.emit_framed(&buf);
        self.scratch = buf;
        self.scratch.clear();
        // On Err the writer is already poisoned (`emit_framed` latched it) and
        // NOTHING below has run: the tables still describe exactly the bytes on
        // disk, which is what keeps a torn bag's data section and its channel
        // table telling the same story.
        emitted?;

        // --- 4. Commit. Only now, with the bytes durable.
        if existing_schema_id.is_none() {
            self.schema_id_of.insert(identity, schema_id);
            self.schemas.push(SchemaRecordInfo {
                id: schema_id,
                name: topic.schema_name.clone(),
                data: descriptor,
            });
        }
        self.channels.push(ChannelInfo {
            topic: topic.topic.clone(),
            schema_id,
            metadata,
        });
        self.topic_to_id.insert(topic.topic.clone(), channel_id);
        Ok(channel_id)
    }

    /// The channel table as it stands — `(topic, schema_name,
    /// schema_hash, wire_fixed_size)` in channel-id order, USER channels only.
    ///
    /// This is what a rotation sibling is constructed from: rotating mid-run
    /// must carry every channel the closing file had learned, not just the set
    /// its own constructor was handed. The reserved channels are excluded
    /// because [`with_sink`](Self::with_sink) auto-registers them
    /// unconditionally — feeding them back in would be refused as a
    /// reserved-prefix topic.
    pub fn registered_topics(&self) -> Vec<TopicSchema> {
        self.channels
            .iter()
            .filter(|c| !c.topic.starts_with(RESERVED_PREFIX))
            .map(|c| {
                // Schema ids are assigned from 1 and `schemas` is ascending by
                // id with no gaps, so `id - 1` indexes it. The descriptor
                // round-trips through its own decoder rather than a hand-made
                // reverse of `encode`, so a descriptor format change cannot
                // silently desync the two.
                let s = &self.schemas[c.schema_id as usize - 1];
                let d = SchemaDescriptor::decode(&s.data).unwrap_or_else(|_| {
                    // Unreachable: `s.data` was produced by this writer's own
                    // `SchemaDescriptor::encode`. Degrading rather than
                    // panicking keeps a diagnostic accessor from being the thing
                    // that kills a recorder.
                    SchemaDescriptor::new(0, 0)
                });
                TopicSchema {
                    topic: c.topic.clone(),
                    schema_name: s.name.clone(),
                    schema_hash: d.schema_hash,
                    wire_fixed_size: d.wire_fixed_size,
                }
            })
            .collect()
    }

    /// The auto-registered `__cerulion/scheduler_trace` channel id.
    pub fn scheduler_trace_channel_id(&self) -> u16 {
        self.scheduler_trace_id
    }

    /// The auto-registered `__cerulion/state` channel id.
    pub fn state_channel_id(&self) -> u16 {
        self.state_id
    }

    /// The auto-registered `__cerulion/frame_producers` channel id —
    /// the stream carrying record-time producer labels for
    /// `multi_publisher_topics` topics.
    pub fn frame_producers_channel_id(&self) -> u16 {
        self.frame_producers_id
    }

    /// Total bytes DURABLY handed to the sink so far (the leading magic,
    /// prelude, every flushed chunk + its message indexes, and attachments).
    /// A pending (unflushed) chunk is NOT counted — its bytes are not yet
    /// written. The recorder (`bagd`) reads this AFTER a flush to decide
    /// size-cap rotation.
    pub fn bytes_written(&self) -> u64 {
        self.pos
    }

    /// Total messages committed in DURABLY-flushed chunks so far (messages in
    /// the still-pending chunk are NOT counted -- a pending chunk is discarded
    /// on error, never persisted). On a mid-`write_chunk`
    /// error after one or more size-triggered auto-flushes, the recorder
    /// diffs this counter across the call to learn the DURABLE PREFIX of its
    /// write sequence and skip exactly those messages on the salvage retry
    /// (the exactly-once property).
    pub fn messages_persisted(&self) -> u64 {
        self.total_message_count
    }

    /// Number of DURABLY-flushed chunks so far (the pending chunk excluded).
    /// Paired with [`messages_persisted`](Self::messages_persisted)
    /// so the recorder's chunk counter stays truthful across mid-`write_chunk`
    /// errors and multi-chunk (auto-flushing) calls.
    pub fn chunks_flushed(&self) -> u64 {
        self.chunk_index_records.len() as u64
    }

    fn resolve(&self, target: WriteTarget<'_>) -> BagResult<u16> {
        match target {
            WriteTarget::Topic(t) => {
                self.topic_to_id
                    .get(t)
                    .copied()
                    .ok_or_else(|| BagError::UnknownTarget {
                        target: t.to_string(),
                        registered: self.channels.len(),
                    })
            }
            WriteTarget::Channel(id) => {
                if (id as usize) < self.channels.len() {
                    Ok(id)
                } else {
                    Err(BagError::UnknownTarget {
                        target: format!("channel_id {id}"),
                        registered: self.channels.len(),
                    })
                }
            }
        }
    }

    /// Open a chunk scope — a CONVENIENCE for "these messages form one chunk":
    /// at closure end (Ok path) the pending chunk is ALWAYS flushed via
    /// `writev` before returning, so scope end is a chunk boundary. The
    /// size-threshold auto-flush still applies inside the scope.
    ///
    /// This carries **no lifetime contract**: payloads are copied
    /// into the chunk arena by [`write_message`](BagWriter::write_message), so a
    /// buffer created inside the closure is perfectly fine and callers may write
    /// messages directly on the writer instead (the recorder does — it lets one
    /// chunk span many drain cycles, which is what the MCAP chunk is for).
    ///
    /// On the closure or flush **Err** path the pending chunk is DISCARDED
    /// without `writev` and the error returned: the pending chunk's messages
    /// (including any trace records queued since the last flush) are LOST —
    /// fail-loud, never a silent partial chunk. Previously auto-flushed chunks
    /// within the same scope are already durable. After an I/O `Err` the file
    /// may have a torn tail; the bag stays crash-recoverable up to the last
    /// complete record.
    ///
    /// ```
    /// use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};
    /// let path = std::env::temp_dir().join(format!("cerulion_doc_{}.mcap", std::process::id()));
    /// let topics = vec![TopicSchema {
    ///     topic: "/imu".into(),
    ///     schema_name: "sensor_msgs/Imu".into(),
    ///     schema_hash: 0xABCD,
    ///     wire_fixed_size: 32,
    /// }];
    /// let mut w = BagWriter::create(&path, BagWriterConfig::default(), &topics)?;
    /// let samples: Vec<Vec<u8>> = vec![vec![1, 2, 3], vec![4, 5, 6]];
    /// w.write_chunk(|chunk| {
    ///     for (i, s) in samples.iter().enumerate() {
    ///         chunk.write_message("/imu", i as u32, 1000 + i as u64, 1000 + i as u64, &[&s[..]])?;
    ///     }
    ///     Ok(())
    /// })?;
    /// w.finalize()?;
    /// # std::fs::remove_file(&path).ok();
    /// # Ok::<(), cerulion_bag::BagError>(())
    /// ```
    pub fn write_chunk<F>(&mut self, f: F) -> BagResult<()>
    where
        F: FnOnce(&mut ChunkScope<'_, S>) -> BagResult<()>,
    {
        self.check_poisoned()?;
        // `scope` doubles as the error/panic guard: if `f` unwinds or returns
        // Err, `ChunkScope`'s `Drop` discards the pending chunk rather than
        // leaving a half-recorded one for a later flush to emit. On the Ok path
        // the flush below empties the pending chunk first, making the
        // drop-time discard a no-op.
        let mut scope = ChunkScope { writer: self };
        f(&mut scope)?;
        scope.writer.flush_chunk()
    }

    /// Record one message into the open chunk.
    ///
    /// **The payload parts are COPIED** into the chunk arena, in order, so the
    /// caller's buffers are free the instant this returns (see the
    /// module docs for why that copy exists and what it bought). Auto-flushes
    /// when the chunk body crosses `chunk_max_bytes`; the caller decides every
    /// other chunk boundary via [`flush_chunk`](Self::flush_chunk).
    ///
    /// `log_time`/`publish_time` come from the caller — the writer reads no
    /// clock.
    pub fn write_message<'t>(
        &mut self,
        target: impl Into<WriteTarget<'t>>,
        sequence: u32,
        log_time: u64,
        publish_time: u64,
        payload_parts: &[&[u8]],
    ) -> BagResult<()> {
        self.check_poisoned()?;
        let channel_id = self.resolve(target.into())?;
        let payload_total: usize = payload_parts.iter().map(|p| p.len()).sum();

        if channel_id == self.scheduler_trace_id && payload_total != TRACE_RECORD_SIZE as usize {
            return Err(BagError::BadTraceRecordLen {
                channel: SCHEDULER_TRACE_TOPIC.to_string(),
                actual: payload_total,
                expected: TRACE_RECORD_SIZE as usize,
            });
        }
        // The same gate for the state channel, and it is on
        // `write_message` rather than only on `write_state_record` because the
        // recorder writes state records THROUGH this entry point (it splices
        // ring SHM spans, exactly as it does for trace records) — a gate that
        // lived only on the convenience wrapper would be reachable by no
        // production caller.
        if channel_id == self.state_id && payload_total != STATE_RECORD_SIZE as usize {
            return Err(BagError::BadStateRecordLen {
                channel: STATE_TOPIC.to_string(),
                actual: payload_total,
                expected: STATE_RECORD_SIZE as usize,
            });
        }
        // And the same gate for the producer-label channel, on
        // `write_message` for the same reason — the recorder writes labels
        // through this entry point, so a gate that lived only on a convenience
        // wrapper would be reachable by no production caller.
        if channel_id == self.frame_producers_id && payload_total != PRODUCER_RECORD_SIZE {
            return Err(BagError::BadProducerRecordLen {
                channel: FRAME_PRODUCERS_TOPIC.to_string(),
                actual: payload_total,
                expected: PRODUCER_RECORD_SIZE,
            });
        }

        self.begin_message(channel_id, sequence, log_time, publish_time, payload_total);
        for part in payload_parts {
            self.chunk_frames.extend_from_slice(part);
        }
        self.maybe_flush_chunk()
    }

    /// Record one scheduler-trace record on the reserved
    /// `__cerulion/scheduler_trace` channel. The 40-byte `TraceRingRecord` wire
    /// form is appended to the chunk arena like any other payload.
    /// `log_time`/`publish_time` come from the caller.
    pub fn write_scheduler_trace(
        &mut self,
        sequence: u32,
        log_time: u64,
        publish_time: u64,
        record: &TraceRingRecord,
    ) -> BagResult<()> {
        self.check_poisoned()?;
        let channel_id = self.scheduler_trace_id;
        let bytes = record.as_bytes();
        self.begin_message(channel_id, sequence, log_time, publish_time, bytes.len());
        self.chunk_frames.extend_from_slice(&bytes);
        self.maybe_flush_chunk()
    }

    /// Append the message frame and record its index entry + time bounds. The
    /// caller then appends the payload bytes, so the arena holds
    /// `[frame][payload][frame][payload]…` — exactly the on-disk chunk body.
    fn begin_message(
        &mut self,
        channel_id: u16,
        sequence: u32,
        log_time: u64,
        publish_time: u64,
        payload_len: usize,
    ) {
        // MessageIndex offset = position of this record within the chunk body,
        // captured BEFORE appending the frame. The arena IS the body, so its
        // current length is that offset.
        self.chunk_index_entries
            .entry(channel_id)
            .or_default()
            .push((log_time, self.chunk_frames.len() as u64));

        encode_message_frame(
            &mut self.chunk_frames,
            channel_id,
            sequence,
            log_time,
            publish_time,
            payload_len,
        );

        // Chunk time bounds only. File-level Statistics (counts + file time
        // bounds) are folded in `flush_chunk` AFTER a successful writev, so a
        // discarded (error/panic) chunk can never inflate them — Statistics
        // derive ONLY from data actually written.
        self.chunk_msg_start = Some(self.chunk_msg_start.map_or(log_time, |s| s.min(log_time)));
        self.chunk_msg_end = Some(self.chunk_msg_end.map_or(log_time, |e| e.max(log_time)));
    }

    /// The open chunk's body size — the arena IS the body.
    ///
    /// The recorder reads this to decide when to close a chunk on its own
    /// (time floor) without duplicating the writer's byte accounting.
    pub fn open_chunk_bytes(&self) -> u64 {
        self.chunk_frames.len() as u64
    }

    /// Messages recorded into the still-OPEN chunk.
    ///
    /// [`messages_persisted`](Self::messages_persisted) counts only DURABLE
    /// chunks, and a chunk spans many of the recorder's write
    /// calls — so `messages_persisted() + open_chunk_messages()` is the count
    /// of messages the writer has ACCEPTED, which is what a caller diffs to
    /// learn how far into its own write sequence a failed call got.
    pub fn open_chunk_messages(&self) -> u64 {
        self.chunk_index_entries
            .values()
            .map(Vec::len)
            .sum::<usize>() as u64
    }

    fn maybe_flush_chunk(&mut self) -> BagResult<()> {
        if self.chunk_frames.len() as u64 >= self.chunk_max_bytes {
            self.flush_chunk()?;
        }
        Ok(())
    }

    /// Flush the current chunk (if it has any messages): write the Chunk record
    /// (header + body) via `writev`, then its per-channel MessageIndex records,
    /// and record a ChunkIndex for the summary. A no-op if the chunk is empty.
    pub fn flush_chunk(&mut self) -> BagResult<()> {
        self.check_poisoned()?;
        if self.chunk_index_entries.is_empty() {
            return Ok(());
        }
        let chunk_start = self.pos;
        // The arena is the body, contiguous and in write order, so the
        // chunk CRC is one hash over it — the identical bytes in the identical
        // order an incremental hasher would fold.
        let uncompressed_crc = crc32fast::hash(&self.chunk_frames);
        // READ, never `take`. These bounds belong to the chunk, and the chunk
        // survives a failed flush: `writev_chunk_records` below can fail, and
        // this call's caller may either DISCARD the chunk (which clears them
        // itself) or RETRY the flush. Taking them here made the retry stamp
        // 0..0 into both the Chunk header and the ChunkIndex while the messages
        // inside kept their real log times — a bag whose index disagrees with
        // its own content, on the exact path a mid-run write error takes. They
        // are cleared below, once the bytes are durable.
        let msg_start = self.chunk_msg_start.unwrap_or(0);
        let msg_end = self.chunk_msg_end.unwrap_or(0);
        let body_len = self.chunk_frames.len() as u64;

        self.scratch.clear();
        encode_chunk_header(
            &mut self.scratch,
            msg_start,
            msg_end,
            uncompressed_crc,
            body_len,
        );
        let header_len = self.scratch.len() as u64;

        // TEST SEAM: stand exactly where a `writev` failure stands — the header
        // is encoded, nothing is committed, the arena and the chunk bounds are
        // intact and the caller owns the decision to discard or retry.
        #[cfg(any(test, feature = "test-helpers"))]
        if self.fault_inject_flush_skip > 0 && self.fault_inject_flush_failures > 0 {
            self.fault_inject_flush_skip -= 1;
        } else if self.fault_inject_flush_failures > 0 {
            self.fault_inject_flush_failures -= 1;
            return Err(BagError::Writev {
                errno: libc::EIO,
                offset: chunk_start,
                iov_count: 2,
                msg: "fault injection: chunk writev failure (test seam)".to_string(),
            });
        }

        // The chunk-path half of the poison latch — and it poisons on
        // PARTIAL progress ONLY, which is a real asymmetry with `emit_framed`
        // rather than a weaker rule.
        //
        // `flush_iovecs` resumes across short writes and reports how far it got
        // when it gives up: both variants it can return carry
        // `offset == chunk_start + written`. So the two shapes are
        // DISTINGUISHABLE here, and they are genuinely different states:
        //
        // - ZERO progress (`offset == chunk_start`): not one byte reached the
        //   sink, and `writev_chunk_records` folds `data_crc` only AFTER a
        //   success, so `pos`, the CRC, the arena and the chunk bounds are all
        //   exactly as they were. The chunk is legitimately RETRYABLE, which is
        //   what `bag_writer_syscall_test::a_retried_chunk_flush_keeps_its_real_message_time_bounds`
        //   and `..._does_not_pollute_data_section_crc` pin, and what the
        //   recorder's salvage retry does in production. Poisoning here would
        //   destroy a working recovery path to guard a state that did not occur.
        // - PARTIAL progress (`offset > chunk_start`): some of the chunk really
        //   is on disk while `pos` still names its start, so a retry would
        //   re-emit the prefix and leave a stray torn Chunk record before a
        //   complete one. Nothing could reconcile that, so the writer latches.
        //
        // `emit_framed` cannot make this distinction and therefore does not try:
        // `WritevSink::write_bytes` returns `io::Result<()>`, an all-or-nothing
        // signature over a `write_all` that may have written an unknown prefix.
        // Unmeasurable progress is treated as partial — the safe direction.
        let written = match writev_chunk_records(
            &mut self.sink,
            &mut self.data_crc,
            &self.scratch,
            &self.chunk_frames,
            chunk_start,
        ) {
            Ok(n) => n,
            Err(e) => {
                let bytes_on_disk = match &e {
                    BagError::Writev { offset, .. } | BagError::WritevNoProgress { offset, .. } => {
                        *offset > chunk_start
                    }
                    // Unreachable today (the loop returns only those two), and
                    // fail-CLOSED rather than trusted: a future error variant
                    // that cannot report its progress is treated as partial.
                    _ => true,
                };
                if bytes_on_disk {
                    self.poisoned = Some("a partially-written chunk flush");
                }
                return Err(e);
            }
        };
        // Release-mode backstop mirroring the one inside flush_iovecs.
        assert!(
            written == header_len + body_len,
            "chunk flush wrote {written} != expected {}",
            header_len + body_len
        );
        self.pos += written;
        let chunk_length = written;

        // The bytes are durable — only NOW may the chunk's bounds be released
        // (see the read-not-take above).
        self.chunk_msg_start = None;
        self.chunk_msg_end = None;

        // The chunk is durable: fold its file-level Statistics now (never at
        // record time — see `begin_message`).
        self.file_msg_start = Some(self.file_msg_start.map_or(msg_start, |s| s.min(msg_start)));
        self.file_msg_end = Some(self.file_msg_end.map_or(msg_end, |e| e.max(msg_end)));

        // MessageIndex records after the chunk (framed), tracking file offsets.
        let entries = std::mem::take(&mut self.chunk_index_entries);
        let mut message_index_offsets: Vec<MessageIndexOffset> = Vec::with_capacity(entries.len());
        let mut message_index_length: u64 = 0;
        for (channel_id, records) in &entries {
            self.total_message_count += records.len() as u64;
            *self.per_channel_count.entry(*channel_id).or_insert(0) += records.len() as u64;
            let mi_offset = self.pos;
            self.scratch.clear();
            encode_message_index(&mut self.scratch, *channel_id, records);
            let buf = std::mem::take(&mut self.scratch);
            self.emit_framed(&buf)?;
            message_index_length += buf.len() as u64;
            self.scratch = buf;
            message_index_offsets.push((*channel_id, mi_offset));
        }

        self.chunk_index_records.push(ChunkIndexData {
            message_start_time: msg_start,
            message_end_time: msg_end,
            chunk_start_offset: chunk_start,
            chunk_length,
            message_index_offsets,
            message_index_length,
            size: body_len,
        });

        // Reset chunk state (keep the arena's capacity — it is reused for the
        // life of the writer and is what makes steady-state recording
        // allocation-free).
        self.chunk_frames.clear();
        Ok(())
    }

    /// Drop the pending (unflushed) chunk without `writev`. Runs on the
    /// [`write_chunk`](Self::write_chunk) Err path and on closure panic (via
    /// [`ChunkScope`]'s drop guard): a half-recorded chunk must not be left for
    /// a later flush to emit. A no-op when nothing is pending. Loud:
    /// discarding recorded messages is never silent.
    pub fn discard_pending_chunk(&mut self) {
        if !self.chunk_index_entries.is_empty() {
            let discarded = self.open_chunk_messages();
            tracing::warn!(
                discarded_messages = discarded,
                "discarding the pending chunk after an error or panic — these messages are NOT in the bag"
            );
        }
        self.chunk_frames.clear();
        self.chunk_msg_start = None;
        self.chunk_msg_end = None;
        self.chunk_index_entries.clear();
    }

    /// Write an attachment (e.g. `graph.yaml`, `env.json`). Flushes any open
    /// chunk first (attachments are data-section records, never inside a
    /// chunk). `log_time`/`create_time` come from the caller. The attachment
    /// `data` is copied into scratch (cold path).
    pub fn write_attachment(
        &mut self,
        name: &str,
        media_type: &str,
        log_time: u64,
        create_time: u64,
        data: &[u8],
    ) -> BagResult<()> {
        self.check_poisoned()?;
        self.flush_chunk()?;
        let offset = self.pos;
        self.scratch.clear();
        let len = encode_attachment(
            &mut self.scratch,
            log_time,
            create_time,
            name,
            media_type,
            data,
        );
        let buf = std::mem::take(&mut self.scratch);
        self.emit_framed(&buf)?;
        self.scratch = buf;
        self.attachment_index_records.push(AttachmentIndexData {
            offset,
            length: len as u64,
            log_time,
            create_time,
            data_size: data.len() as u64,
            name: name.to_string(),
            media_type: media_type.to_string(),
        });
        Ok(())
    }

    /// Write the bag's schema-provenance attachment
    /// ([`SCHEMA_DOCS_ATTACHMENT`](crate::SCHEMA_DOCS_ATTACHMENT)) — the verbatim
    /// text of the CUSTOM types its frames are stamped with, plus the
    /// `schema_hash` → qualified-name bindings the recorder resolved.
    ///
    /// An EMPTY catalog writes NOTHING and returns `Ok(())`: a recording whose
    /// every topic is a built-in ROS 2 type has no provenance to add (the reader
    /// compiled those in), so its bytes stay identical to a pre-attachment bag.
    /// `log_time`/`create_time` are stamped 0 like bagd's other metadata
    /// attachments — the bag's determinism contract forbids a wall clock here.
    pub fn write_schema_catalog(&mut self, catalog: &crate::BagSchemaCatalog) -> BagResult<()> {
        // Ahead of the empty-catalog early return, so a poisoned writer answers
        // the same way whatever the catalog holds. The alternative — Ok(()) for
        // an empty catalog on a dead writer — is a success answer from a writer
        // that will never emit another byte.
        self.check_poisoned()?;
        if catalog.is_empty() {
            return Ok(());
        }
        let bytes = catalog.encode()?;
        self.write_attachment(
            crate::SCHEMA_DOCS_ATTACHMENT,
            crate::SCHEMA_DOCS_MEDIA_TYPE,
            0,
            0,
            &bytes,
        )
    }

    /// Flush the final chunk, write DataEnd, the Summary section, the Footer,
    /// closing magic, and `fsync`. Consumes the writer (post-finalize writes
    /// are a compile error).
    pub fn finalize(mut self) -> BagResult<()> {
        self.check_poisoned()?;
        self.flush_chunk()?;

        // DataEnd — its CRC covers everything from the leading magic up to (but
        // not including) the DataEnd record itself. Snapshot the running CRC,
        // then write DataEnd OUTSIDE the covered region.
        let data_section_crc = self.data_crc.clone().finalize();
        self.scratch.clear();
        encode_data_end(&mut self.scratch, data_section_crc);
        let de = std::mem::take(&mut self.scratch);
        self.sink.write_bytes(&de)?;
        self.pos += de.len() as u64;
        self.scratch = de;

        // Summary section: build entirely in one buffer, tracking group spans.
        let summary_start = self.pos;
        let mut buf: Vec<u8> = Vec::new();
        let mut offsets: Vec<(u8, u64, u64)> = Vec::new(); // (group_opcode, group_start, group_length)

        // Schemas (repeat).
        let g = buf.len();
        for s in &self.schemas {
            encode_schema(&mut buf, s.id, &s.name, SCHEMA_ENCODING, &s.data);
        }
        offsets.push((op::SCHEMA, summary_start + g as u64, (buf.len() - g) as u64));

        // Channels (repeat).
        let g = buf.len();
        for (id, c) in self.channels.iter().enumerate() {
            encode_channel(
                &mut buf,
                id as u16,
                c.schema_id,
                &c.topic,
                SCHEMA_ENCODING,
                &c.metadata,
            );
        }
        offsets.push((
            op::CHANNEL,
            summary_start + g as u64,
            (buf.len() - g) as u64,
        ));

        // Statistics.
        let g = buf.len();
        let counts: Vec<(u16, u64)> = self
            .per_channel_count
            .iter()
            .map(|(k, v)| (*k, *v))
            .collect();
        encode_statistics(
            &mut buf,
            self.total_message_count,
            self.schemas.len() as u16,
            self.channels.len() as u32,
            self.attachment_index_records.len() as u32,
            0, // metadata_count
            self.chunk_index_records.len() as u32,
            self.file_msg_start.unwrap_or(0),
            self.file_msg_end.unwrap_or(0),
            &counts,
        );
        offsets.push((
            op::STATISTICS,
            summary_start + g as u64,
            (buf.len() - g) as u64,
        ));

        // ChunkIndexes.
        if !self.chunk_index_records.is_empty() {
            let g = buf.len();
            for ci in &self.chunk_index_records {
                encode_chunk_index(
                    &mut buf,
                    ci.message_start_time,
                    ci.message_end_time,
                    ci.chunk_start_offset,
                    ci.chunk_length,
                    &ci.message_index_offsets,
                    ci.message_index_length,
                    ci.size,
                    ci.size,
                );
            }
            offsets.push((
                op::CHUNK_INDEX,
                summary_start + g as u64,
                (buf.len() - g) as u64,
            ));
        }

        // AttachmentIndexes.
        if !self.attachment_index_records.is_empty() {
            let g = buf.len();
            for ai in &self.attachment_index_records {
                encode_attachment_index(
                    &mut buf,
                    ai.offset,
                    ai.length,
                    ai.log_time,
                    ai.create_time,
                    ai.data_size,
                    &ai.name,
                    &ai.media_type,
                );
            }
            offsets.push((
                op::ATTACHMENT_INDEX,
                summary_start + g as u64,
                (buf.len() - g) as u64,
            ));
        }

        // Summary offsets.
        let summary_offset_start = summary_start + buf.len() as u64;
        for (group_opcode, group_start, group_length) in &offsets {
            encode_summary_offset(&mut buf, *group_opcode, *group_start, *group_length);
        }

        // Footer prefix, then the self-referencing summary CRC (covers the whole
        // summary buffer through the footer prefix), then closing magic.
        encode_footer_prefix(&mut buf, summary_start, summary_offset_start);
        let summary_crc = crc32fast::hash(&buf);
        buf.extend_from_slice(&summary_crc.to_le_bytes());
        buf.extend_from_slice(&record::MAGIC);

        self.sink.write_bytes(&buf)?;
        self.pos += buf.len() as u64;
        self.sink.sync_all()?;
        self.finalized = true;
        Ok(())
    }
}

// ===========================================================================
// ChunkScope — the scoped write handle
// ===========================================================================

/// A scoped handle for recording messages that form ONE chunk.
///
/// Created by [`BagWriter::write_chunk`]. It is a convenience,
/// not a contract: payloads are copied into the arena by
/// [`BagWriter::write_message`], so the scope adds exactly two things — a
/// guaranteed chunk boundary at scope end, and error/panic containment (its
/// `Drop` discards a half-recorded pending chunk rather than leaving it for a
/// later flush).
pub struct ChunkScope<'w, S: WritevSink = FileSink> {
    writer: &'w mut BagWriter<S>,
}

impl<S: WritevSink> ChunkScope<'_, S> {
    /// Record a message into this chunk. Delegates to
    /// [`BagWriter::write_message`] — payload parts are COPIED into the chunk
    /// arena, so they carry no lifetime obligation.
    pub fn write_message<'t>(
        &mut self,
        target: impl Into<WriteTarget<'t>>,
        sequence: u32,
        log_time: u64,
        publish_time: u64,
        payload_parts: &[&[u8]],
    ) -> BagResult<()> {
        self.writer
            .write_message(target, sequence, log_time, publish_time, payload_parts)
    }

    /// Record a scheduler-trace record inside the scope (interleaves with data
    /// messages in the same chunk). Delegates to
    /// [`BagWriter::write_scheduler_trace`].
    pub fn write_scheduler_trace(
        &mut self,
        sequence: u32,
        log_time: u64,
        publish_time: u64,
        record: &TraceRingRecord,
    ) -> BagResult<()> {
        self.writer
            .write_scheduler_trace(sequence, log_time, publish_time, record)
    }
}

impl<S: WritevSink> Drop for ChunkScope<'_, S> {
    /// Error containment: if the closure unwinds (or `write_chunk` returns
    /// early on Err), discard the pending chunk so a half-recorded chunk is
    /// never emitted by a later flush. After a successful scope-end flush the
    /// pending chunk is empty and this is a no-op.
    fn drop(&mut self) {
        self.writer.discard_pending_chunk();
    }
}

#[cfg(any(test, feature = "test-helpers"))]
impl<S: WritevSink> BagWriter<S> {
    /// Borrow the underlying sink (test seam — inspect the scripted sink's
    /// recorded byte stream after a [`flush_chunk`](Self::flush_chunk)).
    pub fn sink(&self) -> &S {
        &self.sink
    }
}

impl<S: WritevSink> Drop for BagWriter<S> {
    fn drop(&mut self) {
        if !self.finalized {
            tracing::warn!(
                "BagWriter dropped without finalize(): the bag has no summary/footer and is only \
                 crash-recoverable up to the last flushed chunk"
            );
        }
    }
}

/// `writev` the chunk as `[header][body]` and — ONLY after the writev
/// succeeded — fold the flushed bytes into `data_crc`. INVARIANT: `data_crc`
/// covers exactly the bytes durably handed to the sink, so a
/// failed-and-discarded chunk can never poison the DataEnd `data_section_crc`
/// of a bag that keeps writing.
///
/// Exactly TWO iovecs, because the arena IS the body. Before, this
/// built one iovec per message frame AND one per payload part, resolving the
/// payload arms through a raw pointer; a 7-frame batch cost 15 iovecs and a
/// 4 MiB chunk could exceed `IOV_MAX` and need several `writev` calls.
///
/// Split out as a free function so the disjoint borrows of `BagWriter`'s
/// fields (sink vs. frames vs. crc) type-check.
fn writev_chunk_records<S: WritevSink>(
    sink: &mut S,
    data_crc: &mut crc32fast::Hasher,
    header_buf: &[u8],
    chunk_body: &[u8],
    start_offset: u64,
) -> BagResult<u64> {
    let mut iovs = [IoSlice::new(header_buf), IoSlice::new(chunk_body)];
    let written = flush_iovecs(sink, &mut iovs, start_offset)?;
    data_crc.update(header_buf);
    data_crc.update(chunk_body);
    Ok(written)
}
