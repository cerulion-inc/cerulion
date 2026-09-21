// SPDX-License-Identifier: AGPL-3.0-only
//! The bag's schema-provenance attachment, through a REAL bag.
//!
//! `catalog.rs`'s in-module tests pin the pure encode/prune/normalize algebra
//! against hand oracles. This file pins the FILE-level contract the writer and
//! reader make to each other, which those cannot see:
//!
//! - a written catalog reads back EQUAL to the hand-built one, and its docs
//!   really are the verbatim text (a byte compare, not a name compare);
//! - a bag with NO catalog — every earlier recording, and every all-built-in
//!   one — reads `None` SILENTLY and is otherwise byte-identical, so the feature
//!   cannot have changed what an existing bag looks like;
//! - a bag whose attachment is CORRUPT still opens, still reports its channels,
//!   and answers `None` rather than failing the open (the warn-never-refuse
//!   contract that keeps a playable bag playable);
//! - the attachment is byte-DETERMINISTIC — two runs of the same input produce
//!   identical files, extending the writer's existing determinism gate over the
//!   new bytes.
//!
//! Parallel-safe: each test writes its own uniquely-named file, no transport.

use std::path::PathBuf;

use cerulion_bag::{
    BagReader, BagSchemaCatalog, BagWriter, BagWriterConfig, TopicSchema, SCHEMA_DOCS_ATTACHMENT,
};
use cerulion_core::{SchemaDoc, SchemaEncoding, SchemaHashName};

fn tempdir() -> PathBuf {
    static C: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = C.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let p = std::env::temp_dir().join(format!(
        "cerulion_bag_cat_{}_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos(),
        n
    ));
    std::fs::create_dir_all(&p).unwrap();
    p
}

const LOWSTATE_TEXT: &str = "# a vendor type nobody's desk compiled\nuint8 head\ngo/Bms bms\n";
const BMS_TEXT: &str = "uint8 soc\nuint16 volt\n";

/// A SECOND hash binding, deliberately ordered so it is not already first.
///
/// R1: `oracle()` carried exactly ONE `SchemaHashName`, which made
/// `two_runs_carrying_the_same_catalog_are_byte_identical`'s
/// `shuffled.hashes.reverse()` a NO-OP — so `encode()`'s normalization of the
/// HASH list was pinned by nothing, and a variant normalizing only `docs`
/// produced byte-identical files and passed. This one sorts BELOW
/// `LOWSTATE_HASH`, so the normalized order and the constructed order differ and
/// a reversal is a real permutation.
const BMS_HASH: u64 = 0x0123_4567_89AB_CDEF;
const LOWSTATE_HASH: u64 = 0xABCD_1234_5678_9F00;

/// The hand oracle: one vendor root + its one nested custom, plus the hash→name
/// bindings for the attach-mode channels recorded as `"unknown"`.
fn oracle() -> BagSchemaCatalog {
    BagSchemaCatalog::new(
        vec![
            SchemaDoc {
                qualified: "go/LowState".into(),
                encoding: SchemaEncoding::Msg,
                text: LOWSTATE_TEXT.into(),
                deps: vec!["go/Bms".into()],
            },
            SchemaDoc {
                qualified: "go/Bms".into(),
                encoding: SchemaEncoding::Msg,
                text: BMS_TEXT.into(),
                deps: vec![],
            },
        ],
        vec![
            SchemaHashName {
                schema_hash: LOWSTATE_HASH,
                qualified: "go/LowState".into(),
            },
            SchemaHashName {
                schema_hash: BMS_HASH,
                qualified: "go/Bms".into(),
            },
        ],
    )
}

fn topics() -> Vec<TopicSchema> {
    vec![TopicSchema {
        topic: "/lowstate".into(),
        schema_name: "go/LowState".into(),
        schema_hash: LOWSTATE_HASH,
        wire_fixed_size: 0,
    }]
}

/// Write a bag carrying `catalog` (when `Some`) plus one frame.
fn write_bag(path: &std::path::Path, catalog: Option<&BagSchemaCatalog>) {
    let payload: Vec<u8> = vec![0x7Eu8; 16];
    let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics()).unwrap();
    if let Some(c) = catalog {
        w.write_schema_catalog(c).unwrap();
    }
    w.write_chunk(|c| c.write_message("/lowstate", 0, 100, 100, &[&payload[..]]))
        .unwrap();
    w.finalize().unwrap();
}

#[test]
fn a_written_catalog_reads_back_equal_and_carries_verbatim_text() {
    let dir = tempdir();
    let path = dir.join("with.mcap");
    let expected = oracle();
    write_bag(&path, Some(&expected));

    let reader = BagReader::open(&path).unwrap();
    let got = reader
        .schema_catalog()
        .expect("the bag must carry its schema provenance");
    assert_eq!(got, expected, "the catalog round-trips through a real bag");

    // The TEXT is what a desk decodes from — assert the bytes, not just the name.
    let low = got
        .docs
        .iter()
        .find(|d| d.qualified == "go/LowState")
        .expect("root doc");
    assert_eq!(low.text, LOWSTATE_TEXT);
    assert_eq!(low.deps, vec!["go/Bms".to_string()]);
    let bms = got
        .docs
        .iter()
        .find(|d| d.qualified == "go/Bms")
        .expect("closure member");
    assert_eq!(bms.text, BMS_TEXT);

    // The hash binding is what NAMES an attach-mode channel recorded "unknown".
    assert_eq!(
        got.name_for_hash(0xABCD_1234_5678_9F00),
        Some("go/LowState")
    );

    // The frames are untouched by any of this.
    let msgs: Vec<_> = reader
        .messages()
        .unwrap()
        .collect::<Result<Vec<_>, _>>()
        .unwrap();
    assert_eq!(msgs.len(), 1);
    assert_eq!(msgs[0].data, vec![0x7Eu8; 16]);
}

#[test]
fn a_bag_without_a_catalog_reads_none_and_is_byte_identical_to_an_empty_one() {
    let dir = tempdir();
    let absent = dir.join("absent.mcap");
    let empty = dir.join("empty.mcap");
    write_bag(&absent, None);
    // An EMPTY catalog must write nothing at all — the all-built-in recording.
    write_bag(&empty, Some(&BagSchemaCatalog::empty()));

    assert!(
        BagReader::open(&absent).unwrap().schema_catalog().is_none(),
        "a pre-catalog bag carries no provenance and says so quietly"
    );
    assert!(BagReader::open(&empty).unwrap().schema_catalog().is_none());
    assert_eq!(
        std::fs::read(&absent).unwrap(),
        std::fs::read(&empty).unwrap(),
        "an empty catalog must leave the bag BYTE-identical — a recording of \
         built-in types only cannot grow an attachment"
    );
    // ...and the attachment genuinely is not there (not merely undecodable).
    assert!(BagReader::open(&absent)
        .unwrap()
        .attachment(SCHEMA_DOCS_ATTACHMENT)
        .unwrap()
        .is_none());
}

/// A catalog payload this build cannot read leaves the bag fully playable.
///
/// The corruption is written as a WELL-FORMED MCAP attachment carrying a bad
/// payload — NOT by overwriting bytes in the finished file. Byte-patching a
/// finalized bag also breaks the attachment CRC and the DataEnd CRC, so the
/// `mcap` crate rejects the whole file and the test would prove nothing about
/// this crate's decode path. A well-formed attachment with an unreadable body is
/// also the shape the real cases take: a bag written by a NEWER cerulion (a
/// catalog version this build does not read) and a hand-edited one.
#[test]
fn an_unreadable_catalog_payload_leaves_the_bag_playable_and_answers_none() {
    let dir = tempdir();
    let payload: Vec<u8> = vec![0x7Eu8; 16];

    for (tag, body) in [
        ("garbage", b"not json at all".to_vec()),
        ("future", {
            // A catalog from a FUTURE cerulion: valid JSON, version bumped.
            let mut v: serde_json::Value =
                serde_json::from_slice(&oracle().encode().unwrap()).unwrap();
            v["version"] = serde_json::json!(cerulion_bag::SCHEMA_CATALOG_VERSION + 1);
            serde_json::to_vec(&v).unwrap()
        }),
    ] {
        let path = dir.join(format!("{tag}.mcap"));
        let mut w = BagWriter::create(&path, BagWriterConfig::default(), &topics()).unwrap();
        w.write_attachment(SCHEMA_DOCS_ATTACHMENT, "application/json", 0, 0, &body)
            .unwrap();
        w.write_chunk(|c| c.write_message("/lowstate", 0, 100, 100, &[&payload[..]]))
            .unwrap();
        w.finalize().unwrap();

        let reader = BagReader::open(&path).unwrap();
        assert!(
            reader.schema_catalog().is_none(),
            "[{tag}] an unreadable catalog is None — never a panic, never a half-read set"
        );
        // The attachment IS there — so `None` really came from the decode, not
        // from the reader failing to find it (which would make this vacuous).
        assert!(reader.attachment(SCHEMA_DOCS_ATTACHMENT).unwrap().is_some());
        // The bag is still a bag: channels and frames unaffected.
        let chans = reader.channels().unwrap();
        assert!(chans.iter().any(|c| c.topic == "/lowstate"), "[{tag}]");
        let msgs: Vec<_> = reader
            .messages()
            .unwrap()
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        assert_eq!(
            msgs.len(),
            1,
            "[{tag}] an unreadable catalog costs no frame"
        );
        assert_eq!(msgs[0].data, payload, "[{tag}]");
    }
}

#[test]
fn two_runs_carrying_the_same_catalog_are_byte_identical() {
    let dir = tempdir();
    let a = dir.join("a.mcap");
    let b = dir.join("b.mcap");
    write_bag(&a, Some(&oracle()));

    // Build the SAME content in a different order, mutating the PUBLIC fields
    // AFTER construction so the normalising constructor is bypassed — the writer
    // must not leak the caller's accumulation order into the file. (This is a
    // real regression pin: encoding `self` verbatim fails
    // here.)
    //
    // BOTH lists must be permuted, and BOTH must really move — R1 found the
    // `hashes` half unpinned because `oracle()` carried one binding, making its
    // reversal a no-op and leaving `encode()`'s hash normalization free to be
    // deleted. The precondition below is what keeps that from silently
    // recurring if a future edit trims the fixture.
    let mut shuffled = oracle();
    assert!(
        shuffled.docs.len() >= 2 && shuffled.hashes.len() >= 2,
        "PRECONDITION: both lists need >= 2 entries or `reverse()` is a no-op and \
         this test pins nothing: docs={} hashes={}",
        shuffled.docs.len(),
        shuffled.hashes.len()
    );
    shuffled.docs.reverse();
    shuffled.hashes.reverse();
    assert_ne!(
        shuffled.hashes,
        oracle().hashes,
        "PRECONDITION: the reversal must actually PERMUTE the hash list"
    );
    assert_ne!(
        shuffled.docs,
        oracle().docs,
        "PRECONDITION: the reversal must actually PERMUTE the doc list"
    );
    write_bag(&b, Some(&shuffled));

    assert_eq!(
        std::fs::read(&a).unwrap(),
        std::fs::read(&b).unwrap(),
        "the schema attachment must be byte-deterministic like the rest of the bag"
    );
}
