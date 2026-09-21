// SPDX-License-Identifier: AGPL-3.0-only
//! [`BagError`] — the unified error type for the hand-rolled MCAP writer and
//! reader.
//!
//! Every variant is actionable: the write-path syscall variants carry the raw
//! `errno` plus the byte counts (expected vs. actually written) so a failed
//! recorder flush is diagnosable from the log line alone, and the format
//! variants name the offending topic / channel / record.

use std::io;

/// Result alias for bag operations.
pub type BagResult<T> = Result<T, BagError>;

/// Errors from the [`crate`] MCAP writer / reader.
///
/// `#[non_exhaustive]` — new variants may be added without a breaking change;
/// match with a `_` arm.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum BagError {
    /// A `write_message` referenced a topic (or channel id) that is not in the
    /// writer's channel table. An unregistered topic is a hard error rather
    /// than a silent drop.
    ///
    /// "registered at construction" was made false by
    /// [`crate::BagWriter::register_topic`] — a topic may now be added at any
    /// point in the file — so the remedy names the method rather than telling
    /// the caller to go back and change a constructor call they may not own.
    #[error("unknown write target {target:?}: not among the {registered} channels registered so far — register it with `register_topic` before writing to it")]
    UnknownTarget {
        /// The offending topic name or `channel_id` (stringified).
        target: String,
        /// How many channels are registered so far.
        registered: usize,
    },

    /// The writer is TERMINALLY poisoned — an earlier write to the
    /// sink failed, so every later call is refused.
    ///
    /// A failed `write_all` / `writev` may be PARTIAL: the sink's file offset
    /// can be ahead of the writer's `pos` while the running data-section CRC
    /// covers only the bytes the writer believes it wrote. Every subsequent
    /// record would then be framed at an offset that does not exist, and the
    /// summary's ChunkIndex/AttachmentIndex offsets — which a reader seeks to
    /// directly — would point into the middle of the torn record. There is
    /// deliberately NO retry and no reset: the correct recovery is to stop and
    /// let the bag be read as the crash-recoverable prefix it is.
    ///
    /// `after` names the operation that poisoned the writer, so the log line
    /// alone says which of the two partial-progress shapes (a framed cold-path
    /// record, or a chunk `writev`) tore the file.
    ///
    /// NOT every failed write poisons, and the line between them is measured
    /// progress rather than severity. A chunk `writev` that made ZERO progress
    /// left the file untouched and is deliberately RETRYABLE — the recorder's
    /// salvage retry depends on it. The cold path has no such exemption because
    /// `write_all` cannot report how far it got, so its failure is treated as
    /// partial. See `BagWriter::flush_chunk`.
    #[error("the bag writer is poisoned after {after}: a partial write may have left the file offset ahead of the writer's, so no later write is trustworthy — this writer is terminal (no retry); the bag is crash-recoverable up to its last complete record")]
    Poisoned {
        /// The operation whose failure poisoned the writer.
        after: &'static str,
    },

    /// A user topic name began with the reserved `__cerulion/` prefix. That
    /// namespace is owned by the writer's auto-registered reserved channels.
    #[error("topic {topic:?} uses the reserved `__cerulion/` prefix; that namespace belongs to the writer's auto-registered reserved channels — rename the user topic")]
    ReservedTopicPrefix {
        /// The rejected user topic name.
        topic: String,
    },

    /// A `BagWriterConfig::provisioning` KEY began with the reserved
    /// `__cerulion/` prefix.
    ///
    /// Its own variant rather than [`Self::ReservedTopicPrefix`] because the
    /// remedy differs: nothing here is a user topic to rename — the caller put
    /// a reserved name in a metadata map, and the fix is to drop the entry. A
    /// reserved channel is auto-registered by the writer and carries no
    /// provisioning by construction.
    #[error("provisioning key {topic:?} uses the reserved `__cerulion/` prefix; the reserved channels are auto-registered and carry no provisioning — drop the entry")]
    ReservedProvisioningKey {
        /// The rejected provisioning key.
        topic: String,
    },

    /// Two registered topics resolved to the same name.
    #[error("duplicate topic {topic:?} in the registration list — each topic must be unique")]
    DuplicateTopic {
        /// The duplicated topic name.
        topic: String,
    },

    /// The registration's TOTAL channel count (user topics plus the writer's
    /// auto-registered reserved channels) exceeded the `u16` channel-id
    /// space.
    ///
    /// A channel id is a 16-bit MCAP field and both id-assignment sites derive
    /// it from an index with `as u16`, so one channel past the space WRAPS to
    /// id 0 and two channels silently share an id: every message written to one
    /// reads back as the other's, and the FIRST wrapped id collides with the
    /// canonically-first user topic. Refused BEFORE any id is assigned, so a
    /// rejected registration leaves no file and no half-built channel table.
    ///
    /// The reserved count is a real term, not a rounding detail: it is what
    /// moved the boundary when the fourth reserved channel was added, and
    /// it is why the error reports the two addends as well as their sum — a
    /// caller sizing its own topic set needs to know how much of the space it
    /// does not own.
    #[error("registration declares {topics} user topic(s) plus {reserved} auto-registered reserved channel(s) = {total} channels, which exceeds the {cap} addressable channel ids (a bag addresses a channel with a 16-bit id, so registering more would wrap and give two channels the same id) — record fewer topics per bag")]
    TooManyChannels {
        /// How many user topics the caller registered.
        topics: usize,
        /// How many reserved channels the writer auto-registers.
        reserved: usize,
        /// The total (`topics + reserved`) that exceeded the cap.
        total: usize,
        /// The channel-id space (`u16::MAX as usize + 1`).
        cap: usize,
    },

    /// The registration's DISTINCT schema identities
    /// exceeded the USABLE schema-id space.
    ///
    /// The same `as u16` class as [`Self::TooManyChannels`] and a SEPARATE
    /// variant because it is a different quantity against a different cap with
    /// a different remedy: channels are counted per REGISTRATION ENTRY, schemas
    /// per distinct `(schema name, descriptor)` — so two topics sharing one
    /// message type cost two channels and ONE schema, and a caller told to
    /// "record fewer topics" would be reading the wrong instruction.
    ///
    /// The cap is one SMALLER than the channel cap, which the message states
    /// rather than leaving to be derived: MCAP reserves schema id 0 for "no
    /// schema", so ids are assigned from 1 and only `u16::MAX` of them can be
    /// addressed, where channel id 0 is an ordinary channel.
    #[error("registration carries {schemas} distinct schema identities (schema name + descriptor), which exceeds the {cap} usable schema ids — a bag addresses a schema with a 16-bit id AND reserves id 0 for \"no schema\", so ids run 1..={cap} and the space is one SMALLER than the channel-id space; record fewer distinct schemas per bag")]
    TooManySchemas {
        /// How many distinct schema identities the registration carries
        /// (user topics plus the auto-registered reserved channels).
        schemas: usize,
        /// The usable schema-id space (`u16::MAX as usize`, id 0 reserved).
        cap: usize,
    },

    /// A `write_message` for the `__cerulion/scheduler_trace` channel carried a
    /// payload whose length was not exactly [`crate::TRACE_RECORD_SIZE`].
    #[error("scheduler-trace payload on channel {channel:?} is {actual} bytes, expected exactly {expected} (a 40-byte TraceRingRecord)")]
    BadTraceRecordLen {
        /// The channel name.
        channel: String,
        /// The actual payload length in bytes.
        actual: usize,
        /// The required length ([`crate::TRACE_RECORD_SIZE`]).
        expected: usize,
    },

    /// A `write_message` for the `__cerulion/state` channel carried a
    /// payload whose length was not exactly [`crate::STATE_RECORD_SIZE`].
    ///
    /// State records are FIXED-SIZE by construction (chunking is what
    /// deletes the size refusal, so a bigger anchor is more records, never a
    /// bigger record). A short one on this channel means the caller sliced the
    /// ring wrongly, and writing it would make the reader's part accounting
    /// silently wrong rather than loudly absent.
    #[error("state-checkpoint payload on channel {channel:?} is {actual} bytes, expected exactly {expected} (one fixed-size state record)")]
    BadStateRecordLen {
        /// The channel name.
        channel: String,
        /// The actual payload length in bytes.
        actual: usize,
        /// The required length ([`crate::STATE_RECORD_SIZE`]).
        expected: usize,
    },

    /// A `write_message` for the `__cerulion/frame_producers` channel
    /// carried a payload whose length was not exactly
    /// [`crate::PRODUCER_RECORD_SIZE`].
    ///
    /// Producer labels are FIXED-SIZE by construction (both kinds share one
    /// layout, so more attribution is more records, never a bigger record). A
    /// wrong-length payload on this channel means the caller framed the stream
    /// wrongly, and writing it would leave a reader silently mis-attributing
    /// frames rather than loudly unable to read the labels.
    #[error("producer-label payload on channel {channel:?} is {actual} bytes, expected exactly {expected} (one fixed-size producer record)")]
    BadProducerRecordLen {
        /// The channel name.
        channel: String,
        /// The actual payload length in bytes.
        actual: usize,
        /// The required length ([`crate::PRODUCER_RECORD_SIZE`]).
        expected: usize,
    },

    /// A `writev(2)` call failed. Carries the raw `errno`, the batch that was
    /// in flight (iovec count), and the running byte position so a truncated
    /// flush is diagnosable.
    #[error("writev failed at file offset {offset} (errno {errno}, {iov_count} iovecs in flight): {msg}")]
    Writev {
        /// The `errno` from the failed `writev`.
        errno: i32,
        /// The running file offset at the point of failure.
        offset: u64,
        /// The number of iovecs in the batch that failed.
        iov_count: usize,
        /// The `strerror`-style message.
        msg: String,
    },

    /// A `writev(2)` returned `0` with bytes still pending (no forward
    /// progress). Treated as a hard error — writing on rather than looping
    /// would either spin forever or silently truncate the bag.
    #[error("writev made no progress (returned 0) at file offset {offset} with {remaining} bytes still pending — refusing to spin or silently truncate")]
    WritevNoProgress {
        /// The running file offset where progress stalled.
        offset: u64,
        /// Bytes still pending in the flush when progress stalled.
        remaining: u64,
    },

    // NOTE: there is deliberately NO "short flush" error variant. A flush that
    // writes fewer bytes than requested can only mean the resumption loop
    // itself is broken; that invariant is guarded by release-mode `assert!`s
    // in the writer (fail-loud panic, never a recoverable typed error).
    /// An underlying I/O error (open / write / fsync / read).
    #[error("bag I/O error: {0}")]
    Io(#[from] io::Error),

    /// The `mcap` crate rejected the file on the read path (truncated tail,
    /// bad chunk CRC, malformed record, ...).
    #[error("mcap read error: {0}")]
    Mcap(String),

    /// A `cerulion` schema descriptor blob could not be decoded (wrong length /
    /// unknown version).
    #[error("cerulion schema descriptor decode failed: {reason}")]
    SchemaDescriptor {
        /// What was malformed.
        reason: String,
    },

    /// The `__cerulion/schemas.json` schema-provenance attachment could
    /// not be encoded or decoded (malformed JSON / a version this build does not
    /// read). Distinct from [`BagError::SchemaDescriptor`], which is the
    /// per-channel 18-byte blob: this one is the bag-level closure attachment,
    /// and a reader must be able to tell "this bag's channels are unreadable"
    /// from "this bag's schema TEXT is unreadable" (the second still plays).
    #[error("cerulion schema catalog: {reason}")]
    SchemaCatalog {
        /// What was malformed.
        reason: String,
    },

    /// A `__cerulion/frame_producers` record could not be decoded.
    ///
    /// Its own variant rather than [`Self::Malformed`], mirroring
    /// [`Self::SchemaDescriptor`], so that a caller can tell a producer-label
    /// decode failure apart from every other malformed-bag condition and decide
    /// what to do about the LABELS specifically — a bag whose frames are intact
    /// and whose attribution is unreadable is still a usable recording.
    ///
    /// It does NOT discriminate the CAUSE. The three conditions
    /// [`ProducerRecord::decode`](crate::ProducerRecord::decode) refuses on —
    /// wrong length, unknown version, unknown kind — collapse into this one
    /// variant and are told apart only by `reason`, which is PROSE for an
    /// operator, never a token to branch on (this repo's structural-detection
    /// rule). So a reader that wants to treat a version SKEW ("labels from a
    /// build I do not read", remedy: upgrade) differently from corruption
    /// cannot get that from the type as it stands; adding it means a cause
    /// enum, not a message match.
    #[error("cerulion producer record decode failed: {reason}")]
    ProducerRecord {
        /// What was malformed.
        reason: String,
    },

    /// A read-path record carried an unexpected value (e.g. a scheduler-trace
    /// channel message whose payload was not 40 bytes).
    #[error("malformed bag record: {reason}")]
    Malformed {
        /// What was malformed.
        reason: String,
    },
}

impl From<mcap::McapError> for BagError {
    fn from(e: mcap::McapError) -> Self {
        BagError::Mcap(e.to_string())
    }
}
