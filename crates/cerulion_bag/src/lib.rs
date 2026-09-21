// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion_bag`: the MCAP writer and reader behind Cerulion recordings.
//!
//! A Cerulion bag is a standard MCAP file (readable by the `mcap` crate, standard
//! MCAP viewers and `mcap doctor`). This crate writes the bytes itself rather than
//! through the `mcap` crate's writer, because the recording format makes three
//! promises that writer cannot: the output is byte-deterministic (the writer reads
//! no clock, so the same call sequence produces the same file), chunk boundaries
//! belong to the caller, and a bag cut short by a crash stays readable up to its
//! last complete record. Message payloads are copied into the chunk arena when
//! they are written, so the caller's buffer is free as soon as the call returns;
//! the arena goes to disk in `writev(2)` batches. The `mcap` crate is used only in
//! the [`reader`] and as an independent test oracle.
//!
//! # Who uses it
//!
//! `cerulion graph run --record` and `cerulion bag record` write bags through the
//! recorder daemon (`cerulion_bagd`); `cerulion bag play`, `cerulion bag info` and
//! `cerulion bag play --resim all --verify` read them. Depend on this crate directly
//! to read Cerulion bags from your own tools ([`BagReader`]).
//!
//! # Layout
//!
//! | Module | Role |
//! |---|---|
//! | [`record`] | MCAP framing primitives (opcodes, magic, record encoders) |
//! | [`schema`] | The `cerulion` schema descriptor encoding |
//! | [`catalog`] | The bag's schema-provenance attachment (custom-type text, hash-to-name map) |
//! | [`producers`] | The record-time producer label carried on the `__cerulion/frame_producers` channel |
//! | [`writer`] | [`BagWriter`], the chunk-buffering writer, and the [`writer::WritevSink`] seam |
//! | [`reader`] | [`BagReader`], which reads a bag back through the `mcap` crate |
//! | [`error`]  | [`BagError`] and [`BagResult`] |
//!
//! Unix only (POSIX `writev` and the `cerulion_core::trace_ring` record type, which
//! is itself `#[cfg(unix)]`). Design notes for contributors live in
//! `docs/internals/recording.md` in the repository.

// The writer's `writev` path and the reused `TraceRingRecord` (a
// `cerulion_core` `#[cfg(unix)]` item) make this a Unix-only crate. On other
// targets the crate compiles to nothing rather than failing the build.
#![cfg(unix)]
// Principle #12 (structured logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod catalog;
pub mod error;
pub mod producers;
pub mod provisioning;
pub mod reader;
pub mod record;
pub mod schema;
pub mod writer;

#[cfg(any(test, feature = "test-helpers"))]
pub mod test_sink;

pub use catalog::{
    BagSchemaCatalog, SCHEMA_CATALOG_VERSION, SCHEMA_DOCS_ATTACHMENT, SCHEMA_DOCS_MEDIA_TYPE,
};
pub use error::{BagError, BagResult};
pub use producers::{
    ProducerAttribution, ProducerRecord, ProducerRecordKind, PRODUCER_RECORD_SIZE,
    PRODUCER_RECORD_VERSION,
};
pub use provisioning::{
    ChannelProvisioning, KEY_BUFFER_DEPTH, KEY_HISTORY_DEPTH, KEY_LATCHED, KEY_MAX_SLICE_LEN,
    PROVISIONING_KEY_PREFIX,
};
pub use reader::{
    AdviseCall, AdviseCursor, BagAttachment, BagChannel, BagCompleteness, BagMessage, BagReader,
    FrameSpan, TraceRecordIter, UserFrameWalk, ADVISE_BEHIND_BATCH_BYTES, ADVISE_MAX_LAG_BYTES,
};
pub use schema::{
    SchemaDescriptor, DESCRIPTOR_LEN, DESCRIPTOR_VERSION, FRAME_PRODUCERS_SCHEMA,
    FRAME_PRODUCERS_TOPIC, NONDETERMINISM_SCHEMA, NONDETERMINISM_TOPIC, RESERVED_PREFIX,
    SCHEDULER_TRACE_SCHEMA, SCHEDULER_TRACE_TOPIC, SCHEMA_ENCODING, STATE_SCHEMA, STATE_TOPIC,
};
pub use writer::{
    BagWriter, BagWriterConfig, ChunkScope, FileSink, TopicSchema, WriteTarget, WritevOutcome,
    WritevSink, DEFAULT_CHUNK_MAX_BYTES, PROFILE,
};

// Re-export the shared trace record type + its fixed wire size so callers get
// them from one place (the scheduler-trace stream carries this verbatim).
pub use cerulion_core::trace_ring::{TraceRingRecord, TRACE_RECORD_SIZE};

// The same rule for the node-state checkpoint stream — the
// `__cerulion/state` channel carries `cerulion_core`'s state records VERBATIM,
// so the size that gates them comes from the one module that defines the format.
pub use cerulion_core::state_ring::STATE_RECORD_SIZE;
