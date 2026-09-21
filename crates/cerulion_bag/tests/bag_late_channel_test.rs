// SPDX-License-Identifier: AGPL-3.0-only
//! `BagWriter::register_topic` — a topic registered AFTER
//! construction, so a producer that appears mid-run is recorded from that
//! moment.
//!
//! The claims under test, and how each is made to bite:
//!
//! - **Ids** are a function of the call sequence: an existing schema identity is
//!   REUSED, a late channel takes `channels.len()`, and the construction set is
//!   untouched.
//! - **Placement** is TOP-LEVEL, before the chunk carrying the first message on
//!   the new channel. Pinned by an EXACT top-level opcode sequence, walked with
//!   the upstream `mcap` reader and STOPPED at `DataEnd` — the summary repeats
//!   every Schema and Channel, so an unbounded walk would count each twice and
//!   an in-arena regression would still "find" its pair.
//! - **Bytes** are deterministic and hand-assemblable: the Channel record is
//!   compared against a byte oracle built here, never against a second run of
//!   the same writer.
//! - **Failure** is terminal: a registration whose write fails leaves the tables
//!   as they were and poisons the writer.
//! - **Cost** is one extra `write_bytes` and NO extra chunk boundary, read off
//!   the sink's call log (`writev_calls()` alone cannot see it, and
//!   `iov_captures()` never sees a cold-path write at all).
//!
//! Every oracle is hand-written. Where two runs are compared for determinism,
//! the SAME body also checks a hand-assembled record, so agreement between two
//! wrong runs cannot pass.

use std::path::PathBuf;

use cerulion_bag::test_sink::{BytesAction, ScriptedSink, SinkAction, SinkCall};
use cerulion_bag::{
    BagError, BagReader, BagWriter, BagWriterConfig, ChannelProvisioning, TopicSchema,
};

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// A unique scratch path. Process id + a monotonic counter already make the
/// name unique within and across concurrent runs; a wall-clock component would
/// add nothing, and this crate's whole invariant is that a clock read has no
/// business here (`BagWriter::create` truncates, so even a reused pid meeting a
/// leftover file is harmless).
fn tmp(tag: &str) -> PathBuf {
    static C: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = C.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let p = std::env::temp_dir().join(format!(
        "cerulion_bag_late_{}_{}_{tag}.mcap",
        std::process::id(),
        n
    ));
    // A REUSED pid meeting a leftover artifact from an interrupted earlier run
    // must not leak into this run's assertions (a rejection-path test never
    // truncates the path it refuses to create), so the path is cleared at
    // issuance — deterministic, and still clock-free.
    let _ = std::fs::remove_file(&p);
    p
}

fn ts(topic: &str, schema_name: &str, hash: u64, size: u32) -> TopicSchema {
    TopicSchema {
        topic: topic.into(),
        schema_name: schema_name.into(),
        schema_hash: hash,
        wire_fixed_size: size,
    }
}

/// The construction set every test starts from: ONE user topic, so the
/// prelude's record counts are small enough to write out by hand.
fn early() -> TopicSchema {
    ts("/early", "sensor_msgs/Imu", 0xAAAA, 320)
}

/// A late topic carrying a DIFFERENT schema identity from [`early`].
fn late() -> TopicSchema {
    ts("/late", "sensor_msgs/Image", 0xBBBB, 64)
}

/// The reserved channels `BagWriter` auto-registers. Pinned independently by
/// `bag_validation_test::empty_topic_list_still_registers_reserved_channels`.
const RESERVED: usize = 4;

// ---------------------------------------------------------------------------
// Oracles
// ---------------------------------------------------------------------------

/// The TOP-LEVEL record opcodes of a bag, in file order, STOPPED at `DataEnd`.
///
/// Stopping there is load-bearing, not tidiness: the summary section repeats
/// every Schema and Channel record, so a walk that runs to the footer reports
/// each twice — and an in-arena registration, which writes NO top-level pair,
/// would still show one from the summary alone. Everything this file asserts
/// about PLACEMENT is a claim about the data section.
fn top_level_opcodes(bytes: &[u8]) -> Vec<u8> {
    use mcap::records::Record;
    let reader = mcap::read::LinearReader::new_with_options(
        bytes,
        mcap::read::Options::IgnoreEndMagic.into(),
    )
    .expect("linear reader over the bag bytes");
    let mut out = Vec::new();
    for rec in reader {
        let rec = match rec {
            Ok(r) => r,
            // A torn tail is expected on the truncation arms; report what is
            // readable rather than panicking.
            Err(_) => break,
        };
        let op = rec.opcode();
        let done = matches!(rec, Record::DataEnd(_));
        out.push(op);
        if done {
            break;
        }
    }
    out
}

/// Hand-assemble the Channel record this writer must emit, from the MCAP spec
/// rather than from the encoder: opcode `0x04`, a u64 content length, then
/// `id: u16`, `schema_id: u16`, the topic string, the message-encoding string
/// and the metadata map (a u32 BYTE length then u32-length-prefixed pairs).
///
/// Cribs `bag_channel_provisioning_test`'s oracle so the two agree on the
/// layout by construction.
fn channel_record_oracle(
    id: u16,
    schema_id: u16,
    topic: &str,
    metadata: &[(&str, &str)],
) -> Vec<u8> {
    const ENC: &str = "cerulion";
    let map_bytes: usize = metadata
        .iter()
        .map(|(k, v)| 4 + k.len() + 4 + v.len())
        .sum();
    let content_len = (2 + 2 + 4 + topic.len() + 4 + ENC.len() + 4 + map_bytes) as u64;
    let mut o: Vec<u8> = Vec::new();
    o.push(0x04);
    o.extend_from_slice(&content_len.to_le_bytes());
    o.extend_from_slice(&id.to_le_bytes());
    o.extend_from_slice(&schema_id.to_le_bytes());
    o.extend_from_slice(&(topic.len() as u32).to_le_bytes());
    o.extend_from_slice(topic.as_bytes());
    o.extend_from_slice(&(ENC.len() as u32).to_le_bytes());
    o.extend_from_slice(ENC.as_bytes());
    o.extend_from_slice(&(map_bytes as u32).to_le_bytes());
    for (k, v) in metadata {
        o.extend_from_slice(&(k.len() as u32).to_le_bytes());
        o.extend_from_slice(k.as_bytes());
        o.extend_from_slice(&(v.len() as u32).to_le_bytes());
        o.extend_from_slice(v.as_bytes());
    }
    o
}

/// The byte offset of `needle` in `hay`, asserting it occurs EXACTLY once.
///
/// "Exactly once" matters for the placement arms: the summary repeats the
/// Channel record verbatim, so a search that merely found *a* match could be
/// satisfied by the summary copy on a writer that emitted no data-section one.
fn sole_offset(hay: &[u8], needle: &[u8], what: &str) -> usize {
    let hits: Vec<usize> = hay
        .windows(needle.len())
        .enumerate()
        .filter(|(_, w)| *w == needle)
        .map(|(i, _)| i)
        .collect();
    assert_eq!(
        hits.len(),
        1,
        "{what}: expected exactly one occurrence, found {}",
        hits.len()
    );
    hits[0]
}

// ---------------------------------------------------------------------------
// Row 1 — schema-id reuse
// ---------------------------------------------------------------------------

/// A late topic whose `(schema name, descriptor)` identity is
/// already in the table REUSES that schema id and mints NO second Schema record;
/// one carrying a new identity takes `schemas.len() + 1`.
///
/// The oracle is the FILE — `Summary::read`'s schema count and the id the
/// Channel record names — not an accessor, so an always-mint regression is
/// caught in the bytes a reader actually resolves against.
#[test]
fn a_late_topic_sharing_an_existing_schema_identity_reuses_its_schema_id() {
    let p = tmp("reuse");
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[early()]).unwrap();

    // Construction: 4 reserved identities + `/early`'s = 5 schemas.
    // `sensor_msgs/Imu` sorts after all four `cerulion.*` names, so it is id 5.
    let shared_schema_id = 5u16;

    // (a) SAME identity as `/early` — same name, same hash, same fixed size.
    let twin = ts("/twin", "sensor_msgs/Imu", 0xAAAA, 320);
    let twin_id = w.register_topic(&twin).unwrap();

    // (b) A DIFFERENT identity: same schema NAME, different HASH. The dedup key
    // is the pair, so this must mint — a name-only key would reuse and produce a
    // bag whose two channels claim one descriptor.
    let renamed = ts("/renamed", "sensor_msgs/Imu", 0xCCCC, 320);
    let renamed_id = w.register_topic(&renamed).unwrap();

    w.finalize().unwrap();
    let bytes = std::fs::read(&p).unwrap();
    let summary = mcap::Summary::read(&bytes).unwrap().expect("valid summary");

    assert_eq!(
        summary.schemas.len(),
        RESERVED + 2,
        "5 construction schemas + exactly ONE minted for the new identity; the shared-identity \
         registration must mint nothing"
    );
    assert_eq!(
        summary.channels[&twin_id].schema.as_ref().unwrap().id,
        shared_schema_id,
        "a late topic sharing an identity reuses its schema id"
    );
    assert_eq!(
        summary.channels[&renamed_id].schema.as_ref().unwrap().id,
        (RESERVED + 2) as u16,
        "a new identity takes schemas.len() + 1"
    );
    // The reused id really is `/early`'s, not merely a small number.
    let early_id = summary
        .channels
        .values()
        .find(|c| c.topic == "/early")
        .unwrap()
        .schema
        .as_ref()
        .unwrap()
        .id;
    assert_eq!(early_id, shared_schema_id);

    // Exactly ONE top-level Schema record was added by the two registrations.
    let ops = top_level_opcodes(&bytes);
    let schema_records = ops.iter().filter(|o| **o == 0x03).count();
    assert_eq!(
        schema_records,
        RESERVED + 2,
        "the data section carries the 5 prelude Schema records plus ONE late one"
    );
    std::fs::remove_file(&p).ok();
}

// ---------------------------------------------------------------------------
// Row 2 — id assignment
// ---------------------------------------------------------------------------

/// Late channel ids are ARRIVAL order — `channels.len()` at the
/// call — and the construction set's sorted-name ids are unchanged by a
/// registration.
///
/// Deliberately registers in REVERSE-alphabetical order, so a writer that
/// re-sorted the table on every registration would produce the opposite
/// assignment. The construction half is what a re-sort would also break: a
/// sorted rebuild over `{/early, /zeta, /alpha}` moves `/early` off id 0.
#[test]
fn late_ids_are_arrival_order_and_construction_ids_are_unchanged() {
    let p = tmp("ids");
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[early()]).unwrap();

    // Construction: `/early` is the only user topic -> id 0; the reserved
    // channels take 1..=4.
    assert_eq!(w.channel_id("/early"), Some(0));
    let reserved_before: Vec<Option<u16>> = [
        cerulion_bag::FRAME_PRODUCERS_TOPIC,
        cerulion_bag::NONDETERMINISM_TOPIC,
        cerulion_bag::SCHEDULER_TRACE_TOPIC,
        cerulion_bag::STATE_TOPIC,
    ]
    .iter()
    .map(|t| w.channel_id(t))
    .collect();
    assert_eq!(
        reserved_before,
        vec![Some(1), Some(2), Some(3), Some(4)],
        "structural precondition: the reserved channels sit at 1..=4"
    );

    let zeta = w.register_topic(&ts("/zeta", "pkg/Z", 0x1, 4)).unwrap();
    let alpha = w.register_topic(&ts("/alpha", "pkg/A", 0x2, 4)).unwrap();

    assert_eq!(
        (zeta, alpha),
        (5, 6),
        "late ids are ARRIVAL order: /zeta registered first takes 5 even though /alpha sorts \
         before it"
    );
    assert_eq!(w.channel_id("/zeta"), Some(5));
    assert_eq!(w.channel_id("/alpha"), Some(6));

    // The construction set did NOT move.
    assert_eq!(w.channel_id("/early"), Some(0));
    let reserved_after: Vec<Option<u16>> = [
        cerulion_bag::FRAME_PRODUCERS_TOPIC,
        cerulion_bag::NONDETERMINISM_TOPIC,
        cerulion_bag::SCHEDULER_TRACE_TOPIC,
        cerulion_bag::STATE_TOPIC,
    ]
    .iter()
    .map(|t| w.channel_id(t))
    .collect();
    assert_eq!(
        reserved_after, reserved_before,
        "registering a topic must not renumber the construction set"
    );

    // `registered_topics()` reports the table in id order, USER channels only —
    // what a rotation sibling is built from.
    let carried = w.registered_topics();
    let carried_names: Vec<&str> = carried.iter().map(|t| t.topic.as_str()).collect();
    assert_eq!(
        carried_names,
        vec!["/early", "/zeta", "/alpha"],
        "registered_topics is id order (NOT sorted), and excludes the reserved channels"
    );
    // It round-trips the full identity, not just the name.
    assert_eq!(carried[1].schema_name, "pkg/Z");
    assert_eq!(carried[1].schema_hash, 0x1);
    assert_eq!(carried[1].wire_fixed_size, 4);
    assert_eq!(carried[0].schema_name, "sensor_msgs/Imu");
    assert_eq!(carried[0].schema_hash, 0xAAAA);
    assert_eq!(carried[0].wire_fixed_size, 320);

    w.finalize().unwrap();
    std::fs::remove_file(&p).ok();
}

// ---------------------------------------------------------------------------
// Row 3 — record bytes
// ---------------------------------------------------------------------------

/// A late channel is looked up in the provisioning map exactly as a
/// construction-set one is: a DECLARED topic carries its sorted metadata pairs,
/// an UNDECLARED one the empty map (byte-identical to a pre-provisioning bag).
///
/// Both arms are hand-assembled Channel records found in the FILE, so this
/// cannot be satisfied by an accessor that agrees with the encoder.
#[test]
fn a_late_channel_carries_its_provisioning_metadata_and_an_undeclared_one_the_empty_map() {
    let mut config = BagWriterConfig::default();
    config.provisioning.insert(
        "/declared".to_string(),
        ChannelProvisioning {
            buffer_depth: Some(16),
            max_slice_len: Some(2_097_152),
            ..Default::default()
        },
    );

    let p = tmp("prov");
    let mut w = BagWriter::create(&p, config, &[early()]).unwrap();
    // The map is keyed by TOPIC and outlives construction, so it must serve a
    // topic that did not exist when it was handed over.
    let declared = w.register_topic(&ts("/declared", "pkg/D", 0x7, 8)).unwrap();
    let undeclared = w
        .register_topic(&ts("/undeclared", "pkg/D", 0x7, 8))
        .unwrap();
    w.finalize().unwrap();

    let bytes = std::fs::read(&p).unwrap();
    // Both share ONE identity (`pkg/D`, same descriptor), so exactly one schema
    // was minted for the pair: id 6.
    let declared_oracle = channel_record_oracle(
        declared,
        6,
        "/declared",
        // `to_metadata` renders from a `BTreeMap`, so pairs are KEY-sorted.
        &[
            ("cerulion.buffer_depth", "16"),
            ("cerulion.max_slice_len", "2097152"),
        ],
    );
    let undeclared_oracle = channel_record_oracle(undeclared, 6, "/undeclared", &[]);

    // Each appears TWICE in a finalized bag — once in the data section, once in
    // the summary's repeat — and the two copies must be byte-identical, which is
    // what upstream's `ChannelAccumulator` requires of a repeated id.
    let count = |needle: &[u8]| bytes.windows(needle.len()).filter(|w| *w == needle).count();
    assert_eq!(
        count(&declared_oracle),
        2,
        "the declared late channel's hand-assembled record must appear in the data section AND \
         verbatim again in the summary"
    );
    assert_eq!(
        count(&undeclared_oracle),
        2,
        "an undeclared late channel must carry the EMPTY metadata map — the pre-provisioning encoding"
    );

    // And a reader reads them back as declared / not-recorded respectively.
    let reader = BagReader::open(&p).unwrap();
    let channels = reader.channels().unwrap();
    let d = channels.iter().find(|c| c.topic == "/declared").unwrap();
    assert_eq!(d.provisioning.buffer_depth, Some(16));
    assert_eq!(d.provisioning.max_slice_len, Some(2_097_152));
    let u = channels.iter().find(|c| c.topic == "/undeclared").unwrap();
    assert_eq!(u.provisioning, ChannelProvisioning::default());
    std::fs::remove_file(&p).ok();
}

// ---------------------------------------------------------------------------
// Row 4 — record POSITION
// ---------------------------------------------------------------------------

/// The registration's records are TOP-LEVEL, at the position of the
/// call, and precede the chunk carrying the first message on the new channel.
///
/// The oracle is the EXACT top-level opcode sequence, which pins placement and
/// the absence of an extra chunk boundary in one statement. An in-arena variant
/// loses the `SCHEMA, CHANNEL` pair from this list entirely (it moves into the
/// second chunk's body); a `flush_chunk()` inside `register_topic` inserts a
/// third `CHUNK, MESSAGE_INDEX` pair; and registering AFTER the first frame is
/// refused upstream as `UnknownChannel` rather than reordering anything.
#[test]
fn a_late_channel_precedes_its_first_message_in_file_order() {
    const HEADER: u8 = 0x01;
    const SCHEMA: u8 = 0x03;
    const CHANNEL: u8 = 0x04;
    const CHUNK: u8 = 0x06;
    const MESSAGE_INDEX: u8 = 0x07;
    const DATA_END: u8 = 0x0F;

    let p = tmp("order");
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[early()]).unwrap();
    let payload = [0x11u8; 8];
    w.write_message("/early", 0, 100, 100, &[&payload[..]])
        .unwrap();
    w.flush_chunk().unwrap();

    w.register_topic(&late()).unwrap();

    w.write_message("/late", 0, 200, 200, &[&payload[..]])
        .unwrap();
    w.flush_chunk().unwrap();
    w.finalize().unwrap();

    let bytes = std::fs::read(&p).unwrap();
    let mut expected = vec![HEADER];
    // The prelude: 4 reserved schema identities + `/early`'s, then 5 channels.
    expected.extend(std::iter::repeat_n(SCHEMA, RESERVED + 1));
    expected.extend(std::iter::repeat_n(CHANNEL, RESERVED + 1));
    // Chunk 1 (one message on /early) + its single MessageIndex.
    expected.extend([CHUNK, MESSAGE_INDEX]);
    // THE REGISTRATION — top-level, between the two chunks. Schema FIRST: a
    // linear reader resolves a Channel's `schema_id` against schemas already
    // seen.
    expected.extend([SCHEMA, CHANNEL]);
    // Chunk 2 (one message on /late) + its MessageIndex, then DataEnd.
    expected.extend([CHUNK, MESSAGE_INDEX, DATA_END]);

    assert_eq!(
        top_level_opcodes(&bytes),
        expected,
        "the registration must be two TOP-LEVEL records between the two chunks, adding no chunk \
         boundary of its own"
    );

    // The byte offsets agree with the opcode order: the Channel record sits
    // strictly between the two Chunk records.
    let chan = channel_record_oracle(w_late_id(&bytes), 6, "/late", &[]);
    let chan_at = sole_offset(
        &bytes[..data_end_offset(&bytes)],
        &chan,
        "late Channel record",
    );
    let (first_chunk, second_chunk) = chunk_offsets(&bytes);
    assert!(
        first_chunk < chan_at && chan_at < second_chunk,
        "late Channel at {chan_at} must sit between chunk 1 ({first_chunk}) and chunk 2 \
         ({second_chunk})"
    );
    std::fs::remove_file(&p).ok();
}

/// The late channel's id, read back from the finalized bag's summary (the
/// placement test asserts on bytes, so it must not take the id from the writer
/// it is testing).
fn w_late_id(bytes: &[u8]) -> u16 {
    let summary = mcap::Summary::read(bytes).unwrap().expect("valid summary");
    summary
        .channels
        .values()
        .find(|c| c.topic == "/late")
        .expect("the late channel is in the summary")
        .id
}

/// The offset of the `DataEnd` record — the end of the data section.
fn data_end_offset(bytes: &[u8]) -> usize {
    let summary_start = mcap::read::footer(bytes).unwrap().summary_start as usize;
    // DataEnd is 13 bytes (opcode + u64 length + u32 CRC) and sits immediately
    // before the summary.
    let at = summary_start - 13;
    assert_eq!(bytes[at], 0x0F, "structural precondition: DataEnd here");
    at
}

/// The file offsets of the first two Chunk records, from the summary's index.
fn chunk_offsets(bytes: &[u8]) -> (usize, usize) {
    let summary = mcap::Summary::read(bytes).unwrap().expect("valid summary");
    let mut starts: Vec<u64> = summary
        .chunk_indexes
        .iter()
        .map(|c| c.chunk_start_offset)
        .collect();
    starts.sort_unstable();
    assert_eq!(starts.len(), 2, "this fixture writes exactly two chunks");
    (starts[0] as usize, starts[1] as usize)
}

// ---------------------------------------------------------------------------
// Row 7 — determinism of a sequence WITH registrations
// ---------------------------------------------------------------------------

/// A call sequence containing registrations is byte-deterministic.
///
/// Two runs are compared, AND the same body checks a hand-assembled Channel
/// record at a top-level offset strictly between two Chunk records — without
/// that half, two runs of an identically-wrong writer would agree and pass.
#[test]
fn a_topic_registered_mid_file_is_byte_deterministic() {
    fn write_one(path: &std::path::Path) {
        let payload = [0x5Au8; 12];
        let mut w = BagWriter::create(path, BagWriterConfig::default(), &[early()]).unwrap();
        w.write_message("/early", 0, 100, 100, &[&payload[..]])
            .unwrap();
        w.flush_chunk().unwrap();
        w.register_topic(&late()).unwrap();
        w.write_message("/late", 0, 200, 200, &[&payload[..]])
            .unwrap();
        w.write_message("/early", 1, 300, 300, &[&payload[..]])
            .unwrap();
        w.flush_chunk().unwrap();
        w.finalize().unwrap();
    }

    let a = tmp("det_a");
    let b = tmp("det_b");
    write_one(&a);
    write_one(&b);
    let ba = std::fs::read(&a).unwrap();
    let bb = std::fs::read(&b).unwrap();
    assert_eq!(
        ba, bb,
        "two runs of the same call sequence, registrations included, must be byte-identical"
    );
    assert!(!ba.is_empty());

    // The independent half: a hand-assembled record at a position the two runs
    // could not have agreed on by accident.
    let oracle = channel_record_oracle(w_late_id(&ba), 6, "/late", &[]);
    let at = sole_offset(&ba[..data_end_offset(&ba)], &oracle, "late Channel record");
    let (first_chunk, second_chunk) = chunk_offsets(&ba);
    assert!(
        first_chunk < at && at < second_chunk,
        "the late Channel record must be a TOP-LEVEL record between the two chunks"
    );
    std::fs::remove_file(&a).ok();
    std::fs::remove_file(&b).ok();
}

// ---------------------------------------------------------------------------
// Row 8 — a write failure mid-registration
// ---------------------------------------------------------------------------

/// A registration whose write FAILS leaves the topic UNREGISTERED
/// and the writer POISONED.
///
/// The failure is the PARTIAL shape (`ShortThenFail`): bytes really land in the
/// sink and then the call fails, which is the state the latch exists for — the
/// sink's stream is now longer than the writer believes it wrote.
///
/// Three independent assertions, because each kills a different variant:
/// `channel_id` is `None` (tables pushed before `emit_framed` -> a summary-only
/// channel a reader cannot resolve); a fresh reader over the captured bytes sees
/// no such channel; and EVERY later call returns `Poisoned` (latch dropped -> a
/// second registration "succeeds" at an offset that does not exist).
#[test]
fn a_registration_whose_write_fails_leaves_the_writer_unregistered_and_poisoned() {
    // The prelude is the first `write_bytes`; let it through, then tear the
    // registration's write in half.
    let sink = ScriptedSink::single_call().with_byte_script([
        BytesAction::Full,
        BytesAction::ShortThenFail {
            n: 7,
            errno: libc::EIO,
        },
    ]);
    let mut w = BagWriter::with_sink(sink, BagWriterConfig::default(), &[early()]).unwrap();

    let before = w.sink().bytes().len();
    let err = w
        .register_topic(&late())
        .expect_err("the scripted cold-path failure must surface");
    assert!(
        matches!(err, BagError::Io(_)),
        "a failed write_bytes surfaces as an I/O error; got {err:?}"
    );

    // (1) The tables never learned about it.
    assert_eq!(
        w.channel_id("/late"),
        None,
        "a registration whose bytes did not land must not appear in the channel table"
    );
    assert!(
        !w.registered_topics().iter().any(|t| t.topic == "/late"),
        "nor in registered_topics — a rotation sibling must not inherit a channel that was never \
         written"
    );

    // (2) The partial bytes really did land — so this is the shape the latch is
    // for, not a no-op failure that would make the arm vacuous.
    assert_eq!(
        w.sink().bytes().len(),
        before + 7,
        "the fixture must produce PARTIAL progress; without it the poison arm proves nothing"
    );

    // (3) Every later call is refused, and each is checked separately because
    // they are five separate call sites.
    let payload = [0u8; 8];
    assert!(matches!(
        w.write_message("/early", 0, 1, 1, &[&payload[..]]),
        Err(BagError::Poisoned { .. })
    ));
    assert!(matches!(
        w.write_attachment("graph.yaml", "application/yaml", 0, 0, b"g: 1\n"),
        Err(BagError::Poisoned { .. })
    ));
    assert!(matches!(
        w.register_topic(&ts("/other", "pkg/O", 0x9, 4)),
        Err(BagError::Poisoned { .. })
    ));
    assert!(matches!(w.flush_chunk(), Err(BagError::Poisoned { .. })));

    // The refusals wrote NOTHING: the sink's stream is still the durable prefix
    // plus the torn fragment.
    assert_eq!(w.sink().bytes().len(), before + 7);
    let captured = w.sink().bytes().to_vec();
    assert!(matches!(w.finalize(), Err(BagError::Poisoned { .. })));

    // (4) A fresh reader over the captured bytes sees no such channel. There is
    // no summary, so this reads the DATA section — exactly where a
    // committed-before-durable regression would have left one.
    let ops = top_level_opcodes(&captured);
    assert_eq!(
        ops.iter().filter(|o| **o == 0x04).count(),
        RESERVED + 1,
        "only the prelude's channels are in the data section"
    );
    let r = BagReader::from_bytes(captured);
    assert!(
        r.channels().is_err(),
        "an un-finalized bag has no summary to read channels from"
    );
    let (msgs, completeness) = r.recover_messages().unwrap();
    assert!(msgs.is_empty());
    assert!(
        !completeness.is_finalized(),
        "a poisoned writer's bag is a crash-recoverable prefix, never Finalized"
    );
}

/// The poison latch's ONE exemption, pinned on both sides in one body.
///
/// A chunk `writev` that made ZERO progress left the file untouched, so the
/// chunk is legitimately retryable — the recorder's salvage retry depends on it,
/// and `bag_writer_syscall_test` pins the resulting bag's correctness. A PARTIAL
/// one left bytes on disk that `pos` does not account for, so a retry would
/// re-emit the prefix and leave a stray torn Chunk record before a complete one.
///
/// Both arms use the SAME writer shape and differ only in the script, so the
/// discriminator is the progress and nothing else. Deleting the `bytes_on_disk`
/// test (poisoning unconditionally) fails the zero-progress arm here AND the two
/// committed syscall tests; hard-coding it to `false` fails the partial arm.
#[test]
fn a_partially_written_chunk_flush_poisons_but_a_zero_progress_one_stays_retryable() {
    let payload = [0x33u8; 16];

    // --- ZERO progress: not one byte accepted, so the retry is sound ---
    let mut w = BagWriter::with_sink(
        ScriptedSink::new(usize::MAX, [SinkAction::Fail(libc::EIO)]),
        BagWriterConfig::default(),
        &[early()],
    )
    .unwrap();
    w.write_message("/early", 0, 100, 100, &[&payload[..]])
        .unwrap();
    let err = w.flush_chunk().expect_err("the scripted failure surfaces");
    assert!(matches!(err, BagError::Writev { .. }), "got {err:?}");
    // Not poisoned: the retry succeeds and the bag finalizes.
    w.flush_chunk()
        .expect("a zero-progress chunk failure must leave the chunk RETRYABLE");
    w.register_topic(&late())
        .expect("and must leave the writer usable");
    w.finalize().expect("and finalizable");

    // --- PARTIAL progress: some of the chunk is on disk, `pos` names its start ---
    let mut w = BagWriter::with_sink(
        // Accept 5 bytes, then refuse to make further progress. `flush_iovecs`
        // reports `offset = chunk_start + 5`, which is what makes the two shapes
        // distinguishable at all.
        ScriptedSink::new(usize::MAX, [SinkAction::Short(5), SinkAction::Zero]),
        BagWriterConfig::default(),
        &[early()],
    )
    .unwrap();
    w.write_message("/early", 0, 100, 100, &[&payload[..]])
        .unwrap();
    let err = w.flush_chunk().expect_err("the scripted failure surfaces");
    assert!(
        matches!(err, BagError::WritevNoProgress { .. }),
        "got {err:?}"
    );
    assert!(
        matches!(w.flush_chunk(), Err(BagError::Poisoned { .. })),
        "a PARTIALLY-written chunk flush must poison: retrying it would re-emit the prefix already \
         on disk"
    );
    assert!(matches!(
        w.register_topic(&late()),
        Err(BagError::Poisoned { .. })
    ));
    assert!(matches!(w.finalize(), Err(BagError::Poisoned { .. })));
}

// ---------------------------------------------------------------------------
// Row 9 — the salvage / discard path
// ---------------------------------------------------------------------------

/// A registration is durable the moment it returns, so a later
/// chunk DISCARD cannot touch it.
///
/// Drives the writer's own fault seam (which stands exactly where a `writev`
/// failure stands, before any byte reaches the sink), then discards the pending
/// chunk — the shape `write_chunk`'s error path and `ChunkScope`'s drop guard
/// take. An in-arena variant loses the registration with the discarded chunk
/// while the tables keep it, which is the state that would let a salvage retry
/// write frames onto an unresolvable channel.
#[test]
fn registration_survives_a_discarded_pending_chunk() {
    let p = tmp("discard");
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[early()]).unwrap();
    w.register_topic(&late()).unwrap();

    // A message on the late channel, then a flush that fails, then a discard —
    // the message is lost by design; the REGISTRATION must not be.
    let payload = [0x44u8; 8];
    w.fault_inject_flush_failures_for_test(0, 1);
    w.write_message("/late", 0, 100, 100, &[&payload[..]])
        .unwrap();
    let err = w.flush_chunk().expect_err("the injected failure surfaces");
    assert!(matches!(err, BagError::Writev { .. }), "got {err:?}");
    w.discard_pending_chunk();

    // The writer keeps going (the seam models a pre-write failure) and the late
    // channel is still writable.
    w.write_message("/late", 1, 200, 200, &[&payload[..]])
        .unwrap();
    w.flush_chunk().unwrap();
    w.finalize().unwrap();

    let bytes = std::fs::read(&p).unwrap();
    // The registration is in the DATA section, not merely in the summary.
    let oracle = channel_record_oracle(w_late_id(&bytes), 6, "/late", &[]);
    sole_offset(
        &bytes[..data_end_offset(&bytes)],
        &oracle,
        "late Channel record in the data section",
    );
    // And the surviving message reads back on it.
    let msgs: Vec<mcap::Message> = mcap::MessageStream::new(&bytes)
        .unwrap()
        .collect::<Result<_, _>>()
        .unwrap();
    assert_eq!(msgs.len(), 1, "only the post-discard message survives");
    assert_eq!(msgs[0].channel.topic, "/late");
    assert_eq!(msgs[0].sequence, 1);
    std::fs::remove_file(&p).ok();
}

/// A channel registered BEFORE a discarded chunk survives in the data
/// section — the `discard_pending_chunk()` half, driven directly rather than
/// through a flush failure.
#[test]
fn a_channel_registered_before_a_discarded_chunk_survives_in_the_data_section() {
    let p = tmp("discard2");
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[early()]).unwrap();
    w.register_topic(&late()).unwrap();

    let payload = [0x55u8; 8];
    w.write_message("/late", 0, 100, 100, &[&payload[..]])
        .unwrap();
    w.discard_pending_chunk();
    w.write_message("/late", 1, 200, 200, &[&payload[..]])
        .unwrap();
    w.finalize().unwrap();

    let bytes = std::fs::read(&p).unwrap();
    let msgs: Vec<mcap::Message> = mcap::MessageStream::new(&bytes)
        .unwrap()
        .collect::<Result<_, _>>()
        .unwrap();
    assert_eq!(msgs.len(), 1);
    assert_eq!(msgs[0].channel.topic, "/late");
    assert_eq!(
        msgs[0].sequence, 1,
        "the discarded message is gone; the one written after it is on the SAME late channel"
    );
    let oracle = channel_record_oracle(w_late_id(&bytes), 6, "/late", &[]);
    sole_offset(
        &bytes[..data_end_offset(&bytes)],
        &oracle,
        "late Channel record in the data section",
    );
    std::fs::remove_file(&p).ok();
}

// ---------------------------------------------------------------------------
// Row 10 — syscall shape
// ---------------------------------------------------------------------------

/// A registration costs exactly ONE extra `write_bytes` and NO
/// extra chunk boundary.
///
/// The oracle is the sink's CALL LOG, which is the only probe that can see it:
/// `writev_calls()` counts chunk flushes only, and `iov_captures()` never
/// records a cold-path write at all — so without the log, "one write, no extra
/// writev" is unassertable.
///
/// The registration's write is identified by its LENGTH — the exact byte length
/// of the Schema + Channel records it must emit, hand-assembled here — so this
/// cannot be satisfied by any other cold-path write that happens to occur.
///
/// **The chunk is deliberately OPEN across the registration**, which is what
/// makes the claim testable at all AND is the shape the recorder uses (it
/// registers a topic in the same `WriteBatch` that carries its first frame). A
/// test that flushes first has an empty arena, so a spurious
/// `flush_chunk()` inside `register_topic` returns early without a `writev` —
/// and that variant passes a test written specifically to catch it. Registering
/// mid-chunk is the only arrangement in which "does not close the open chunk"
/// has any content.
#[test]
fn a_late_registration_is_one_write_and_no_extra_chunk_boundary() {
    let payload = [0x66u8; 8];

    // Baseline: the same message sequence WITHOUT the registration — ONE chunk
    // spanning both messages, so exactly one writev.
    let mut base = BagWriter::with_sink(
        ScriptedSink::single_call(),
        BagWriterConfig::default(),
        &[early()],
    )
    .unwrap();
    base.write_message("/early", 0, 100, 100, &[&payload[..]])
        .unwrap();
    base.write_message("/early", 1, 200, 200, &[&payload[..]])
        .unwrap();
    base.flush_chunk().unwrap();
    let base_calls: Vec<SinkCall> = base.sink().calls().to_vec();
    let base_writevs = base.sink().writev_calls();
    assert_eq!(
        base_writevs, 1,
        "structural precondition: the baseline is ONE chunk, so an extra boundary is visible"
    );

    // The same sequence WITH a registration in the MIDDLE of the open chunk.
    let mut w = BagWriter::with_sink(
        ScriptedSink::single_call(),
        BagWriterConfig::default(),
        &[early()],
    )
    .unwrap();
    w.write_message("/early", 0, 100, 100, &[&payload[..]])
        .unwrap();
    // The arena is NON-EMPTY here: a `flush_chunk()` inside `register_topic`
    // would close it and emit a Chunk + MessageIndex.
    assert!(w.open_chunk_bytes() > 0, "the chunk must be open");
    let pos_before = w.bytes_written();
    w.register_topic(&late()).unwrap();
    let registration_bytes = (w.bytes_written() - pos_before) as usize;
    w.write_message("/early", 1, 200, 200, &[&payload[..]])
        .unwrap();
    w.flush_chunk().unwrap();
    let calls: Vec<SinkCall> = w.sink().calls().to_vec();

    // NO extra chunk boundary: the writev count is unchanged, and the two
    // messages still share ONE chunk.
    assert_eq!(
        w.sink().writev_calls(),
        base_writevs,
        "a registration must not close the open chunk — a flush_chunk() inside register_topic \
         would add a writev"
    );
    assert_eq!(
        w.chunks_flushed(),
        1,
        "and both messages must still be in ONE chunk"
    );

    // The call log differs by EXACTLY one inserted WriteBytes, of exactly the
    // registration's byte length, in exactly the right place.
    let mut expected = base_calls.clone();
    // The baseline's calls are: WriteBytes(prelude), Writev(2), WriteBytes(mi).
    // The registration lands after the prelude and before the chunk's writev.
    expected.insert(1, SinkCall::WriteBytes(registration_bytes));
    assert_eq!(
        calls, expected,
        "the registration must add exactly ONE write_bytes, between the prelude and the chunk's \
         writev — no flush, no second MessageIndex"
    );

    // The length really is the two records', not an incidental match: assemble
    // them by hand.
    let chan = channel_record_oracle(5, 6, "/late", &[]);
    // Schema record: opcode 0x03, u64 content len, id u16, name, encoding,
    // u32-prefixed 18-byte descriptor.
    const ENC: &str = "cerulion";
    const DESCRIPTOR_LEN: usize = 18;
    let name = "sensor_msgs/Image";
    let schema_len = 1 + 8 + (2 + 4 + name.len() + 4 + ENC.len() + 4 + DESCRIPTOR_LEN);
    assert_eq!(
        registration_bytes,
        schema_len + chan.len(),
        "one write carrying exactly [Schema][Channel]"
    );
}

// ---------------------------------------------------------------------------
// Row 11 — the DataEnd CRC
// ---------------------------------------------------------------------------

/// The data-section CRC still covers exactly the bytes durably
/// written after a late registration.
///
/// `emit_framed` folds AFTER the write; a fold-before-write regression would
/// stamp a CRC over bytes that never landed. Nothing else checks this
/// invariant for the writer as a whole: `MessageStream` validates CHUNK CRCs
/// only, and `LinearReaderOptions::default()` leaves `validate_data_section_crc`
/// off — so this arm is the data-section-CRC gate, and it covers the
/// WHOLE writer, not only registration.
///
/// Both oracles are independent of our reader: the upstream sans-io reader with
/// validation ON must reach `DataEnd` without a CRC error, and the CRC is
/// recomputed here by hand and compared with the stamped value.
#[test]
fn the_data_section_crc_survives_a_late_registration() {
    let p = tmp("crc");
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[early()]).unwrap();
    let payload = [0x77u8; 16];
    w.write_message("/early", 0, 100, 100, &[&payload[..]])
        .unwrap();
    w.flush_chunk().unwrap();
    w.register_topic(&late()).unwrap();
    w.write_message("/late", 0, 200, 200, &[&payload[..]])
        .unwrap();
    w.write_attachment("graph.yaml", "application/yaml", 0, 0, b"g: 1\n")
        .unwrap();
    w.register_topic(&ts("/later", "pkg/L", 0xDDDD, 4)).unwrap();
    w.write_message("/later", 0, 300, 300, &[&payload[..4]])
        .unwrap();
    w.finalize().unwrap();
    let bytes = std::fs::read(&p).unwrap();

    // (a) The upstream reader, with data-section CRC validation ON, must reach
    // DataEnd. `LinearReaderOptions::default()` leaves it off, which is why a
    // plain `MessageStream` read passes even on a corrupt-CRC bag.
    use mcap::sans_io::{LinearReadEvent, LinearReader, LinearReaderOptions};
    let mut reader = LinearReader::new_with_options(
        LinearReaderOptions::default().with_validate_data_section_crc(true),
    );
    let mut cursor = std::io::Cursor::new(&bytes);
    let mut saw_data_end = false;
    let mut guard = 0;
    while let Some(event) = reader.next_event() {
        match event.expect("the data-section CRC must validate after a late registration") {
            LinearReadEvent::ReadRequest(n) => {
                use std::io::Read;
                let written = cursor.read(reader.insert(n)).unwrap();
                reader.notify_read(written);
            }
            LinearReadEvent::Record { opcode, .. } => {
                if opcode == 0x0F {
                    saw_data_end = true;
                }
            }
        }
        guard += 1;
        assert!(guard < 100_000, "guard against an infinite reader loop");
    }
    assert!(saw_data_end, "the reader must reach DataEnd");

    // (b) And recompute it by hand, so this does not rest on the upstream
    // reader honouring the option.
    let at = data_end_offset(&bytes);
    let stamped = u32::from_le_bytes(bytes[at + 9..at + 13].try_into().unwrap());
    assert_eq!(
        stamped,
        crc32fast::hash(&bytes[..at]),
        "data_section_crc must cover exactly the bytes before DataEnd"
    );
    std::fs::remove_file(&p).ok();
}

// ---------------------------------------------------------------------------
// Read-back through both oracles
// ---------------------------------------------------------------------------

/// A late channel's messages read back through the upstream `mcap`
/// oracle AND through `BagReader`.
///
/// `MessageStream` is STRICT and reads the summary's repeated Schema/Channel
/// records too, so this also exercises upstream's `ChannelAccumulator`: a
/// summary copy that differed from the data-section copy would be
/// `ConflictingChannels`, a Channel emitted before its Schema `UnknownSchema`,
/// and a summary-only registration `UnknownChannel`.
#[test]
fn a_late_channels_messages_read_back_through_the_mcap_oracle_and_the_bag_reader() {
    let p = tmp("readback");
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[early()]).unwrap();
    let early_payload: Vec<u8> = (0..8u8).collect();
    let late_payload: Vec<u8> = (100..108u8).collect();

    w.write_message("/early", 0, 100, 100, &[&early_payload[..]])
        .unwrap();
    w.flush_chunk().unwrap();
    w.register_topic(&late()).unwrap();
    w.write_message("/late", 0, 200, 200, &[&late_payload[..]])
        .unwrap();
    w.write_message("/early", 1, 300, 300, &[&early_payload[..]])
        .unwrap();
    w.finalize().unwrap();
    let bytes = std::fs::read(&p).unwrap();

    // (a) The upstream oracle: every message, with topic AND schema name.
    let msgs: Vec<mcap::Message> = mcap::MessageStream::new(&bytes)
        .unwrap()
        .collect::<Result<_, _>>()
        .unwrap();
    let got: Vec<(&str, &str, u32, Vec<u8>)> = msgs
        .iter()
        .map(|m| {
            (
                m.channel.topic.as_str(),
                m.channel.schema.as_ref().unwrap().name.as_str(),
                m.sequence,
                m.data.to_vec(),
            )
        })
        .collect();
    assert_eq!(
        got,
        vec![
            ("/early", "sensor_msgs/Imu", 0, early_payload.clone()),
            ("/late", "sensor_msgs/Image", 0, late_payload.clone()),
            ("/early", "sensor_msgs/Imu", 1, early_payload.clone()),
        ],
        "every message reads back on its own channel, with the schema NAME its registration \
         declared"
    );

    // (b) Our reader buckets both topics.
    let r = BagReader::open(&p).unwrap();
    let index = r.user_message_index().unwrap();
    assert_eq!(index.get("/late").map(Vec::len), Some(1));
    assert_eq!(index.get("/early").map(Vec::len), Some(2));

    // (c) And `messages()` yields the late one with its payload intact.
    let late_msgs: Vec<_> = r
        .messages()
        .unwrap()
        .map(|m| m.unwrap())
        .filter(|m| m.topic == "/late")
        .collect();
    assert_eq!(late_msgs.len(), 1);
    assert_eq!(late_msgs[0].data, late_payload);
    std::fs::remove_file(&p).ok();
}

/// A late channel is INDEXED — it appears in the summary's channel and
/// schema tables and in the Statistics' per-channel message counts.
///
/// The variant this kills is a summary rendered from a creation snapshot rather
/// than the live tables: the messages would still read back through the data
/// section while `bag info` and every summary-driven tool showed nothing.
#[test]
fn a_late_channel_is_indexed_in_the_summary_and_statistics() {
    let p = tmp("summary");
    let mut w = BagWriter::create(&p, BagWriterConfig::default(), &[early()]).unwrap();
    let payload = [0x88u8; 8];
    w.write_message("/early", 0, 100, 100, &[&payload[..]])
        .unwrap();
    w.flush_chunk().unwrap();
    let late_id = w.register_topic(&late()).unwrap();
    for i in 0..3u32 {
        w.write_message("/late", i, 200 + i as u64, 200 + i as u64, &[&payload[..]])
            .unwrap();
    }
    w.finalize().unwrap();
    let bytes = std::fs::read(&p).unwrap();

    let summary = mcap::Summary::read(&bytes).unwrap().expect("valid summary");
    assert_eq!(
        summary.channels.len(),
        RESERVED + 2,
        "the summary must carry the construction set AND the late channel"
    );
    assert_eq!(
        summary.schemas.len(),
        RESERVED + 2,
        "and the schema minted for it"
    );
    let ch = &summary.channels[&late_id];
    assert_eq!(ch.topic, "/late");
    assert_eq!(ch.schema.as_ref().unwrap().name, "sensor_msgs/Image");

    let stats = summary.stats.as_ref().expect("Statistics record");
    assert_eq!(stats.channel_count, (RESERVED + 2) as u32);
    assert_eq!(stats.schema_count, (RESERVED + 2) as u16);
    assert_eq!(
        stats.channel_message_counts.get(&late_id).copied(),
        Some(3),
        "the late channel's three messages must be counted"
    );
    assert_eq!(stats.message_count, 4);

    // And our reader lists it.
    let r = BagReader::open(&p).unwrap();
    let channels = r.channels().unwrap();
    let late_ch = channels
        .iter()
        .find(|c| c.topic == "/late")
        .expect("BagReader::channels lists the late channel");
    assert_eq!(late_ch.id, late_id);
    assert_eq!(late_ch.schema_name, "sensor_msgs/Image");
    let d = late_ch.descriptor.expect("a cerulion descriptor");
    assert_eq!(d.schema_hash, 0xBBBB);
    assert_eq!(d.wire_fixed_size, 64);
    std::fs::remove_file(&p).ok();
}
