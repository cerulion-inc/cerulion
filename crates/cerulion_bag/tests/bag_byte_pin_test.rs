// SPDX-License-Identifier: AGPL-3.0-only
//! C0 — the byte-determinism gate: today's writer bytes,
//! captured as committed `(crc32, len)` literals.
//!
//! `bag_determinism_test` proves the writer is SELF-consistent (the same input
//! sequence twice is byte-identical). That is a relative claim: a change that
//! moves every byte the same way keeps it green. These pins are the ABSOLUTE
//! complement — three fixed input sequences, each reproduced here from an
//! existing in-tree fixture, pinned against literals captured on `main` before
//! any mid-file-channel work began. Every later chunk is judged
//! against them: on the no-late path (no channel registered after creation) the
//! bytes must not move at all.
//!
//! **These pins are DELIBERATELY brittle.** Any byte drift on the no-late path
//! fails them, which is the whole point — a writer change that silently
//! reshuffles the file is exactly what they exist to catch. A DELIBERATE format
//! change re-blesses the literals *in the same commit that makes it*, with the
//! reason stated in the commit message. Never re-bless to make a red test
//! green.
//!
//! ## Two pins per fixture, and why neither replaces the other
//!
//! Each fixture is pinned TWICE, in this order:
//!
//! 1. a **byte compare** against a committed fixture file under
//!    `tests/fixtures/byte_pin_*.mcap`, which on mismatch reports the FIRST
//!    DIVERGING OFFSET with hex context either side. CRC-32 is not injective,
//!    and — the reason that actually matters here — a CRC mismatch cannot say
//!    *which* byte moved. These pins exist to police the C1/C2 mid-file-channel
//!    work, where "the bytes moved" is the start of the question, not the
//!    answer.
//! 2. the **`(crc32, len)` literal**, unchanged from the first pass. It is the
//!    design doc's stated contract, and it is also what guards the fixture
//!    FILES: a fixture regenerated to match drifted writer output still fails
//!    the literal, so the two pins cannot be quietly walked forward together.
//!    A fixture edited by hand fails it too.
//!
//! The literals were captured on `main` at
//! `9124aca656e17511a6af46776d800778fa83901c`, by running these tests with
//! placeholder values and reading the actual pair out of the assertion failure
//! — never hand-computed. The fixture FILES were generated from the very writer
//! sequences below (see [`assert_matches_fixture`]), never hand-edited.
//!
//! ## Re-blessing (both halves, in one commit)
//!
//! ```text
//! BLESS_BYTE_PINS=1 cargo test -p cerulion_bag --test bag_byte_pin_test
//! cargo test -p cerulion_bag --test bag_byte_pin_test   # read the new literals
//! ```
//!
//! The first run rewrites the fixture files and the second fails on the
//! `(crc32, len)` literals, printing the actual pairs to copy in. Blessing
//! therefore cannot make a test vacuously green: the literal still has to be
//! moved by hand, in the same commit, with the reason stated there.
//!
//! Bytes are recorded through the in-memory [`ScriptedSink`] rather than a
//! temp file: the sink is a pure byte recorder (it decides only how the stream
//! is split into `writev` calls, and `bag_writer_syscall_test` pins that the
//! split does not change the bytes), so the fixtures carry nothing
//! host-dependent — no path, no clock, no randomness. Every timestamp in them
//! is a caller-supplied literal, and `BagWriterConfig::default()`'s `library`
//! is the fixed string `"cerulion_bag"`, never build info.

use std::cell::RefCell;
use std::fs;
use std::io::{self, IoSlice};
use std::path::{Path, PathBuf};
use std::rc::Rc;

use cerulion_bag::test_sink::ScriptedSink;
use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema, WritevOutcome, WritevSink};
use cerulion_core::trace_ring::{TraceRingRecord, RECORD_TYPE_DEPARTURE, RECORD_TYPE_FIRE};

/// A delegating sink so the test keeps its handle on the recorded byte stream
/// after `finalize()` consumes the writer (same shape as
/// `bag_writer_syscall_test`'s `SharedSink`).
struct SharedSink(Rc<RefCell<ScriptedSink>>);

impl WritevSink for SharedSink {
    fn writev_once(&mut self, iovs: &[IoSlice<'_>]) -> WritevOutcome {
        self.0.borrow_mut().writev_once(iovs)
    }
    fn iov_max(&self) -> usize {
        self.0.borrow().iov_max()
    }
    fn write_bytes(&mut self, buf: &[u8]) -> io::Result<()> {
        self.0.borrow_mut().write_bytes(buf)
    }
    fn sync_all(&mut self) -> io::Result<()> {
        self.0.borrow_mut().sync_all()
    }
}

/// Run `write` against a fresh in-memory writer over `topics` and return the
/// finalized byte stream. `write` receives the writer and must leave it ready
/// to finalize (it does NOT call `finalize` itself).
fn recorded(topics: &[TopicSchema], write: impl FnOnce(&mut BagWriter<SharedSink>)) -> Vec<u8> {
    let inner = Rc::new(RefCell::new(ScriptedSink::single_call()));
    let mut w = BagWriter::with_sink(
        SharedSink(Rc::clone(&inner)),
        BagWriterConfig::default(),
        topics,
    )
    .expect("writer construction");
    write(&mut w);
    w.finalize().expect("finalize");
    // Bound to a local (rather than returned straight from `inner.borrow()`)
    // so the `Ref` is dropped before `inner` at end of scope.
    let sink = inner.borrow();
    sink.bytes().to_vec()
}

/// The pinned identity of a byte stream: its CRC-32 and its exact length. The
/// length is carried beside the CRC so a failure says *how* the bytes moved
/// (a size change reads differently from a same-size reshuffle) instead of
/// only that they did.
fn identity(bytes: &[u8]) -> (u32, usize) {
    (crc32fast::hash(bytes), bytes.len())
}

/// Setting this regenerates every fixture file from the writer sequences in
/// this file. It does NOT touch the `(crc32, len)` literals — see the module
/// docs' re-blessing recipe.
const BLESS_ENV: &str = "BLESS_BYTE_PINS";

/// Bytes of context rendered either side of the first divergence.
const CONTEXT_BYTES: usize = 16;

fn fixture_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join(format!("byte_pin_{name}.mcap"))
}

/// Render `bytes` as space-separated hex, bracketing the byte at `mark` (a
/// `mark` past the end simply brackets nothing — the shorter of two streams
/// legitimately has no byte at the divergence offset).
fn hex(bytes: &[u8], mark: usize) -> String {
    if bytes.is_empty() {
        return "<empty>".to_string();
    }
    let mut parts: Vec<String> = bytes.iter().map(|b| format!("{b:02x}")).collect();
    if let Some(p) = parts.get_mut(mark) {
        *p = format!("[{p}]");
    }
    parts.join(" ")
}

fn byte_at(bytes: &[u8], at: usize) -> String {
    bytes
        .get(at)
        .map_or_else(|| "<end of stream>".to_string(), |b| format!("0x{b:02x}"))
}

/// Compare `actual` against the committed fixture for `name`, byte for byte.
///
/// On mismatch this reports the first diverging OFFSET, both lengths, the byte
/// each stream carries there, and [`CONTEXT_BYTES`] of hex either side — the
/// diagnostic a `(crc32, len)` pair structurally cannot give.
///
/// Under [`BLESS_ENV`] it instead REGENERATES the fixture from `actual`. That
/// is the only way a fixture is ever written: they are generated from the same
/// writer sequences the assertions run, never hand-edited.
fn assert_matches_fixture(name: &str, actual: &[u8]) {
    let path = fixture_path(name);

    if std::env::var_os(BLESS_ENV).is_some() {
        let dir = path.parent().expect("fixture path has a parent");
        fs::create_dir_all(dir).unwrap_or_else(|e| panic!("create {}: {e}", dir.display()));
        fs::write(&path, actual).unwrap_or_else(|e| panic!("write {}: {e}", path.display()));
        println!("blessed {} ({} bytes)", path.display(), actual.len());
        return;
    }

    let expected = fs::read(&path).unwrap_or_else(|e| {
        panic!(
            "cannot read the committed byte fixture {}: {e}\n\
             it is generated from the writer sequence in this test, never by hand:\n  \
             {BLESS_ENV}=1 cargo test -p cerulion_bag --test bag_byte_pin_test",
            path.display()
        )
    });

    if expected == actual {
        return;
    }

    let at = expected
        .iter()
        .zip(actual.iter())
        .position(|(e, a)| e != a)
        .unwrap_or_else(|| expected.len().min(actual.len()));
    let lo = at.saturating_sub(CONTEXT_BYTES);
    let hi = at + CONTEXT_BYTES + 1;

    panic!(
        "the writer's bytes diverge from the committed fixture {}\n\
         first differing offset: {at} (0x{at:x})\n\
         expected byte: {}   actual byte: {}\n\
         expected len:  {}   actual len:  {}\n\
         expected [{lo}..]: {}\n\
         actual   [{lo}..]: {}\n\
         (hex, {CONTEXT_BYTES} bytes of context either side; [..] brackets the differing byte)\n\
         if the drift was DELIBERATE, regenerate the fixture with {BLESS_ENV}=1 AND re-bless \
         the (crc32, len) literal below in the same commit, stating the reason there",
        path.display(),
        byte_at(&expected, at),
        byte_at(actual, at),
        expected.len(),
        actual.len(),
        hex(&expected[lo..hi.min(expected.len())], at - lo),
        hex(&actual[lo..hi.min(actual.len())], at - lo),
    );
}

/// The two-topic set from `bag_determinism_test::topics`.
fn determinism_topics() -> Vec<TopicSchema> {
    vec![
        TopicSchema {
            topic: "/camera".into(),
            schema_name: "sensor_msgs/Image".into(),
            schema_hash: 0xAAAA,
            wire_fixed_size: 64,
        },
        TopicSchema {
            topic: "/imu".into(),
            schema_name: "sensor_msgs/Imu".into(),
            schema_hash: 0xBBBB,
            wire_fixed_size: 320,
        },
    ]
}

/// The three-topic set from `bag_roundtrip_test::topics`.
fn roundtrip_topics() -> Vec<TopicSchema> {
    vec![
        TopicSchema {
            topic: "/camera/image".into(),
            schema_name: "sensor_msgs/Image".into(),
            schema_hash: 0x1111_2222_3333_4444,
            wire_fixed_size: 64,
        },
        TopicSchema {
            topic: "/imu".into(),
            schema_name: "sensor_msgs/Imu".into(),
            schema_hash: 0x5555_6666_7777_8888,
            wire_fixed_size: 320,
        },
        TopicSchema {
            topic: "/pose".into(),
            schema_name: "geometry_msgs/Pose".into(),
            schema_hash: 0x9999_AAAA_BBBB_CCCC,
            wire_fixed_size: 56,
        },
    ]
}

/// The `bag_determinism_test::write_one` sequence: two user topics, one
/// message each, one scheduler-trace record, one attachment — the smallest
/// fixture that exercises a user channel, a reserved channel and an
/// attachment in one bag.
///
/// Literal captured on `main` at `9124aca656e17511a6af46776d800778fa83901c`.
/// DELIBERATELY brittle: any byte drift on the no-late path fails this. A
/// deliberate format change re-blesses the literal in the same commit, with
/// the reason stated there.
#[test]
fn the_two_topic_determinism_fixture_matches_its_pinned_crc_and_length() {
    let img: Vec<u8> = vec![0x11; 20];
    let imu: Vec<u8> = vec![0x22; 12];
    let rec = TraceRingRecord {
        step: 7,
        fire_time_ns: 42,
        duration_ns: 9,
        node_idx: 1,
        global_level: 0,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    };

    let bytes = recorded(&determinism_topics(), |w| {
        w.write_chunk(|c| {
            c.write_message("/camera", 0, 100, 100, &[&img[..]])?;
            c.write_message("/imu", 0, 101, 101, &[&imu[..]])?;
            c.write_scheduler_trace(1, 102, 102, &rec)
        })
        .unwrap();
        w.write_attachment("graph.yaml", "application/yaml", 200, 199, b"g: 1\n")
            .unwrap();
    });

    assert_matches_fixture("two_topic_determinism", &bytes);

    // Also the fixture file's own guard — see the module docs, "Two pins".
    assert_eq!(
        identity(&bytes),
        (0xE637_D33C, 2301),
        "the two-topic determinism fixture's bytes moved; if that was DELIBERATE, \
         re-bless this literal in the same commit and state the reason there"
    );
}

/// The `bag_roundtrip_test::roundtrip_via_mcap_oracle_and_bagreader` sequence:
/// three user topics, four user messages (one of them assembled from two
/// payload parts), two scheduler-trace records and two attachments — the
/// widest fixture in the crate.
///
/// Literal captured on `main` at `9124aca656e17511a6af46776d800778fa83901c`.
/// DELIBERATELY brittle: any byte drift on the no-late path fails this. A
/// deliberate format change re-blesses the literal in the same commit, with
/// the reason stated there.
#[test]
fn the_six_message_roundtrip_fixture_matches_its_pinned_crc_and_length() {
    let img0: Vec<u8> = vec![0xAA; 16];
    let imu_a: Vec<u8> = vec![0x01, 0x02, 0x03];
    let imu_b: Vec<u8> = vec![0x04, 0x05];
    let pose0: Vec<u8> = vec![0xBB; 8];
    let img1: Vec<u8> = vec![0xCC; 16];
    let graph_yaml = b"nodes:\n  - camera\n  - detector\n".to_vec();
    let env_json = br#"{"RUST_LOG":"info"}"#.to_vec();

    let fire = TraceRingRecord {
        step: 1,
        fire_time_ns: 1004,
        duration_ns: 500,
        node_idx: 2,
        global_level: 1,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    };
    let departure = TraceRingRecord {
        step: 2,
        fire_time_ns: 1005,
        duration_ns: 0,
        node_idx: 3,
        global_level: 2,
        record_type: RECORD_TYPE_DEPARTURE,
        reserved: 0,
    };

    let bytes = recorded(&roundtrip_topics(), |w| {
        w.write_chunk(|c| {
            c.write_message("/camera/image", 0, 1000, 1000, &[&img0[..]])?;
            c.write_message("/imu", 0, 1001, 1001, &[&imu_a[..], &imu_b[..]])?;
            c.write_message("/pose", 0, 1002, 1002, &[&pose0[..]])?;
            c.write_message("/camera/image", 1, 1003, 1003, &[&img1[..]])?;
            c.write_scheduler_trace(1, 1004, 1004, &fire)?;
            c.write_scheduler_trace(2, 1005, 1005, &departure)?;
            Ok(())
        })
        .unwrap();
        w.write_attachment("graph.yaml", "application/yaml", 2000, 1999, &graph_yaml)
            .unwrap();
        w.write_attachment("env.json", "application/json", 2001, 2000, &env_json)
            .unwrap();
    });

    assert_matches_fixture("six_message_roundtrip", &bytes);

    // Also the fixture file's own guard — see the module docs, "Two pins".
    assert_eq!(
        identity(&bytes),
        (0x6631_0572, 2947),
        "the six-message roundtrip fixture's bytes moved; if that was DELIBERATE, \
         re-bless this literal in the same commit and state the reason there"
    );
}

/// The `bag_validation_test::empty_topic_list_still_registers_reserved_channels`
/// bag: no user topics, no messages, no attachments — nothing but the header,
/// the auto-registered reserved channels and the summary. The floor of the
/// format, and the pin most sensitive to a change in the reserved channel set
/// or in how an empty data section is framed.
///
/// Literal captured on `main` at `9124aca656e17511a6af46776d800778fa83901c`.
/// DELIBERATELY brittle: any byte drift on the no-late path fails this. A
/// deliberate format change re-blesses the literal in the same commit, with
/// the reason stated there.
#[test]
fn the_empty_topic_list_bag_matches_its_pinned_crc_and_length() {
    let bytes = recorded(&[], |_w| {});

    assert_matches_fixture("empty_topic_list", &bytes);

    // Also the fixture file's own guard — see the module docs, "Two pins".
    assert_eq!(
        identity(&bytes),
        (0x68EE_6CC6, 1236),
        "the empty-topic-list bag's bytes moved; if that was DELIBERATE, \
         re-bless this literal in the same commit and state the reason there"
    );
}
