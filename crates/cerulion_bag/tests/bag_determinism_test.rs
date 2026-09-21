// SPDX-License-Identifier: AGPL-3.0-only
//! The format-contract determinism gate: the SAME input sequence written twice
//! produces byte-identical files, and channel ids depend only on sorted topic
//! name (not registration order).

use std::path::PathBuf;

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};
use cerulion_core::trace_ring::{TraceRingRecord, RECORD_TYPE_FIRE};

/// A unique scratch path — process id + a monotonic counter, no clock (see
/// `bag_late_channel_test::tmp`).
fn tmp(tag: &str) -> PathBuf {
    static C: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = C.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let p = std::env::temp_dir().join(format!(
        "cerulion_bag_det_{}_{}_{tag}",
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

fn topics() -> Vec<TopicSchema> {
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

fn write_one(path: &std::path::Path, topics: &[TopicSchema]) {
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
    let mut w = BagWriter::create(path, BagWriterConfig::default(), topics).unwrap();
    w.write_chunk(|c| {
        c.write_message("/camera", 0, 100, 100, &[&img[..]])?;
        c.write_message("/imu", 0, 101, 101, &[&imu[..]])?;
        c.write_scheduler_trace(1, 102, 102, &rec)
    })
    .unwrap();
    w.write_attachment("graph.yaml", "application/yaml", 200, 199, b"g: 1\n")
        .unwrap();
    w.finalize().unwrap();
}

#[test]
fn identical_input_sequence_is_byte_identical() {
    let a = tmp("a");
    let b = tmp("b");
    write_one(&a, &topics());
    write_one(&b, &topics());
    let ba = std::fs::read(&a).unwrap();
    let bb = std::fs::read(&b).unwrap();
    assert_eq!(
        ba, bb,
        "two runs of the same input sequence must be byte-identical"
    );
    assert!(!ba.is_empty());
    std::fs::remove_file(&a).ok();
    std::fs::remove_file(&b).ok();
}

/// The CONSTRUCTION set's channel ids depend only on sorted topic name, never
/// on the order the caller listed them in.
///
/// This is a property of the construction set, not of the writer. The broader
/// claim held when the topic set was frozen at construction and read as a
/// property of the writer; `register_topic` makes it false in general, because a
/// LATE channel takes the next free id in REGISTRATION order (there is no
/// ordering over topics that have not appeared yet, so arrival is the only
/// defined one). What survives, and is what this test exercises,
/// is the property of the construction set. The late half is pinned by
/// `bag_late_channel_test::late_ids_are_arrival_order_and_construction_ids_are_unchanged`,
/// which also asserts that a registration does not renumber what is below.
#[test]
fn construction_set_channel_ids_are_registration_order_independent() {
    let forward = topics();
    let mut reversed = topics();
    reversed.reverse();

    let pf = tmp("fwd");
    let pr = tmp("rev");
    let wf = BagWriter::create(&pf, BagWriterConfig::default(), &forward).unwrap();
    let wr = BagWriter::create(&pr, BagWriterConfig::default(), &reversed).unwrap();

    // Sorted-name assignment: /camera -> 0, /imu -> 1, regardless of input order.
    for t in ["/camera", "/imu"] {
        assert_eq!(
            wf.channel_id(t),
            wr.channel_id(t),
            "channel id for {t} is order-independent"
        );
    }
    assert_eq!(wf.channel_id("/camera"), Some(0));
    assert_eq!(wf.channel_id("/imu"), Some(1));
    assert_eq!(
        wf.scheduler_trace_channel_id(),
        wr.scheduler_trace_channel_id()
    );

    // The two files (no messages written) are also byte-identical. finalize()
    // must SUCCEED — dropping the Result would let a broken finalize leave both
    // files identically incomplete and pass the byte-identity assert vacuously.
    wf.finalize().unwrap();
    wr.finalize().unwrap();
    assert_eq!(std::fs::read(&pf).unwrap(), std::fs::read(&pr).unwrap());
    std::fs::remove_file(&pf).ok();
    std::fs::remove_file(&pr).ok();
}
