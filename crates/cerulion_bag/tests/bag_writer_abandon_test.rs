// SPDX-License-Identifier: AGPL-3.0-only
//! `BagWriter::abandon`: a writer closed without finishing its bag, for a
//! caller that removes the file itself.
//!
//! The claims, each against a hand oracle:
//!
//! - an abandoned writer logs nothing, where a writer dropped without
//!   `finalize` logs exactly one WARN and a finalized one logs nothing;
//! - abandoning writes no byte: the file keeps the bytes it held, one flushed
//!   chunk and no DataEnd or footer, and it is still there, because removing it
//!   is the caller's job.
//!
//! The log capture is THREAD-SCOPED (`tracing::subscriber::with_default`), so
//! it records what the test's own thread emits and nothing a parallel test
//! does. Every test in this binary installs it before it builds a writer:
//! `tracing` caches a callsite's interest on the callsite's first hit, and a
//! first hit on a thread with no subscriber would turn the warning off for the
//! rest of the binary.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};

/// The warning a writer dropped without `finalize` logs.
const UNFINALIZED_WARNING: &str = "BagWriter dropped without finalize()";

/// MCAP top-level opcodes this file asserts on (MCAP specification).
const OP_FOOTER: u8 = 0x02;
const OP_CHUNK: u8 = 0x06;
const OP_DATA_END: u8 = 0x0F;

/// A unique scratch path: the process id and a counter, no clock.
fn tmp(tag: &str) -> PathBuf {
    static C: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = C.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let p = std::env::temp_dir().join(format!(
        "cerulion_bag_abandon_{}_{}_{tag}.mcap",
        std::process::id(),
        n
    ));
    // A reused pid can meet a leftover file from an interrupted earlier run.
    let _ = std::fs::remove_file(&p);
    p
}

/// A `tracing` subscriber that records every event of the thread it is
/// installed on, as `"<LEVEL> <message>"`.
struct ThreadLogs(Arc<Mutex<Vec<String>>>);

impl tracing::Subscriber for ThreadLogs {
    fn enabled(&self, _: &tracing::Metadata<'_>) -> bool {
        true
    }
    fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::Id {
        tracing::Id::from_u64(1)
    }
    fn record(&self, _: &tracing::Id, _: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _: &tracing::Id, _: &tracing::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        struct Msg(String);
        impl tracing::field::Visit for Msg {
            fn record_debug(&mut self, f: &tracing::field::Field, v: &dyn std::fmt::Debug) {
                if f.name() == "message" {
                    self.0.push_str(&format!("{v:?}"));
                }
            }
        }
        let mut msg = Msg(String::new());
        event.record(&mut msg);
        self.0
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(format!("{} {}", event.metadata().level(), msg.0));
    }
    fn enter(&self, _: &tracing::Id) {}
    fn exit(&self, _: &tracing::Id) {}
}

/// Run `f` under a [`ThreadLogs`] capture and return what it logged.
fn captured(f: impl FnOnce()) -> Vec<String> {
    let lines = Arc::new(Mutex::new(Vec::new()));
    tracing::subscriber::with_default(ThreadLogs(lines.clone()), f);
    let lines = lines.lock().unwrap_or_else(|e| e.into_inner()).clone();
    lines
}

/// A writer at `path` with one flushed chunk holding message 1 and message 2
/// still in the open chunk.
fn writer_with_an_open_chunk(path: &Path) -> BagWriter {
    let topic = TopicSchema {
        topic: "/abandon".to_string(),
        schema_name: "test/Abandon".to_string(),
        schema_hash: 0xA0A0,
        wire_fixed_size: 4,
    };
    let mut w = BagWriter::create(path, BagWriterConfig::default(), &[topic]).unwrap();
    w.write_message("/abandon", 1, 10, 10, &[&[1, 1, 1, 1]])
        .unwrap();
    w.flush_chunk().unwrap();
    w.write_message("/abandon", 2, 20, 20, &[&[2, 2, 2, 2]])
        .unwrap();
    assert_eq!(
        w.open_chunk_messages(),
        1,
        "PREMISE: message 2 is not flushed"
    );
    w
}

/// The top-level record opcodes of `bytes`, in file order, read with the
/// upstream `mcap` reader and stopped at the first record it cannot read (an
/// unfinished bag ends without its closing magic).
fn top_level_opcodes(bytes: &[u8]) -> Vec<u8> {
    let reader = mcap::read::LinearReader::new_with_options(
        bytes,
        mcap::read::Options::IgnoreEndMagic.into(),
    )
    .expect("linear reader over the bag bytes");
    let mut out = Vec::new();
    for rec in reader {
        let Ok(rec) = rec else { break };
        out.push(rec.opcode());
    }
    out
}

/// What a writer that ends unfinished leaves on disk: the bytes held before it
/// ended, unchanged, with one chunk (message 1's) and no DataEnd or footer.
fn assert_left_unfinished(path: &Path, before: &[u8]) {
    let after = std::fs::read(path).expect("the file is still there");
    assert_eq!(after, before, "no byte is written after the writer ends");
    let ops = top_level_opcodes(&after);
    assert_eq!(
        ops.iter().filter(|&&op| op == OP_CHUNK).count(),
        1,
        "only message 1's flushed chunk is on disk: {ops:02x?}"
    );
    assert!(
        !ops.contains(&OP_DATA_END) && !ops.contains(&OP_FOOTER),
        "no DataEnd and no footer: {ops:02x?}"
    );
}

/// An abandoned writer logs nothing and writes nothing more; the file stays
/// for its caller to remove.
#[test]
fn an_abandoned_writer_drops_without_the_unfinalized_warning() {
    let path = tmp("abandoned");
    let mut before = Vec::new();
    let lines = captured(|| {
        let w = writer_with_an_open_chunk(&path);
        before = std::fs::read(&path).unwrap();
        w.abandon();
    });
    assert_eq!(
        lines,
        Vec::<String>::new(),
        "an abandoned writer logs nothing"
    );
    assert_left_unfinished(&path, &before);
    std::fs::remove_file(&path).unwrap();
}

/// CONTROL for the arm above, over the same writer: dropped without
/// `finalize` it logs the warning exactly once, as a WARN, and leaves the same
/// bytes; finalized, it logs nothing.
#[test]
fn a_dropped_unfinalized_writer_still_warns() {
    let path = tmp("dropped");
    let mut before = Vec::new();
    let lines = captured(|| {
        let w = writer_with_an_open_chunk(&path);
        before = std::fs::read(&path).unwrap();
        drop(w);
    });
    assert_eq!(lines.len(), 1, "one line: {lines:?}");
    assert!(
        lines[0].starts_with("WARN ") && lines[0].contains(UNFINALIZED_WARNING),
        "the unfinalized warning, as a WARN: {lines:?}"
    );
    assert_left_unfinished(&path, &before);
    std::fs::remove_file(&path).unwrap();

    let path = tmp("finalized");
    let lines = captured(|| writer_with_an_open_chunk(&path).finalize().unwrap());
    assert_eq!(
        lines,
        Vec::<String>::new(),
        "a finalized writer logs nothing"
    );
    let ops = top_level_opcodes(&std::fs::read(&path).unwrap());
    assert!(
        ops.contains(&OP_DATA_END) && ops.contains(&OP_FOOTER),
        "the finalized bag is finished: {ops:02x?}"
    );
    std::fs::remove_file(&path).unwrap();
}
