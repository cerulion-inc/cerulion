// SPDX-License-Identifier: AGPL-3.0-only
//! bagd BYTE-FIDELITY for the GAP FRAME, the
//! recorder half of the wire-legality oracle.
//!
//! Borrow-window frames place their big variable field page-aligned in the payload
//! tail with dead gap bytes inside `total_size` and fields out of declaration
//! order (the one stated layout in [`cerulion_core::testing::gap_frame`]).
//! bagd copies each drained frame verbatim into the MCAP chunk arena, so a
//! recorded gap frame must come back BYTE-IDENTICAL — gap bytes (`0xEE`
//! fill), trailing slack and all, carried inside `[0, total_size)`. A
//! recorder that re-framed, compacted or re-zeroed the dead bytes would fail
//! here, which is exactly what makes replaying a recording of borrow-window
//! frames possible.
//!
//! Oracle: the frames are HAND-BUILT inputs (never a self-compare); the bag
//! is re-read with [`BagReader`] AND independently with the upstream `mcap`
//! crate (the e2e_test.rs convention).

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_bag::BagReader;
use cerulion_bagd::{run_bagd, BagdConfig, TapSpec};
use cerulion_core::testing::gap_frame::{image_gap_frame, TOTAL_SIZE};
use cerulion_core::ShmMessage;
use native_ros2_messages::sensor_msgs::Image;

use common::*;

/// Two gap frames (distinct sequence/timestamp, same stated layout) recorded
/// by bagd come back byte-identical, in order, with the wire stamps carried
/// verbatim into the MCAP records.
#[test]
fn gap_frames_record_and_read_back_byte_identical() {
    let mgr = make_manager(32);
    let topic = unique_topic("gap");
    let out = unique_out("gap");
    let ready = unique_ready_file("gap");

    // The publisher creates the service (64 KiB slice ceiling — the gap
    // frame is ~8.2 KiB) BEFORE the tap attaches.
    let mut publisher = publisher(&mgr, &topic, 64 * 1024);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = BagdConfig::new(out.clone(), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready.clone());
    cfg.status_period = None;
    cfg.schema_wait = Duration::from_millis(500);

    let mgr_clone = mgr.clone();
    let handle = std::thread::spawn({
        let shutdown = shutdown.clone();
        move || run_bagd(mgr_clone, cfg, shutdown)
    });
    assert!(
        wait_for_file(&ready, RECORDER_READY_DEADLINE),
        "ready-file never appeared"
    );
    settle();

    // Hand-built gap frames: the ONE stated layout, distinct wire stamps.
    let expected: Vec<Vec<u8>> = (0..2u32)
        .map(|i| image_gap_frame(Image::SCHEMA_HASH, i, 1_000 + i as u64))
        .collect();
    for frame in &expected {
        assert_eq!(frame.len(), TOTAL_SIZE, "the fixture's stated total size");
        publisher.publish_raw(frame).expect("publish gap frame");
    }

    settle();
    settle();
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");
    assert_eq!(summary.messages, 2, "both gap frames recorded");

    // BagReader oracle: byte-identical frames in arrival order, wire stamps
    // carried verbatim (log_time == publish_time == the hand timestamp).
    let reader = BagReader::open(&out).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover_messages");
    assert!(completeness.is_finalized(), "bag must be Finalized");
    let recorded: Vec<&cerulion_bag::BagMessage> =
        msgs.iter().filter(|m| m.topic == topic).collect();
    assert_eq!(recorded.len(), 2);
    for (i, m) in recorded.iter().enumerate() {
        assert_eq!(
            m.data, expected[i],
            "gap frame {i} must be byte-identical — dead gap bytes included"
        );
        assert_eq!(m.sequence, i as u32, "wire sequence carried");
        assert_eq!(m.log_time, 1_000 + i as u64, "wire timestamp carried");
        assert_eq!(m.publish_time, m.log_time);
    }

    // Independent oracle: the upstream `mcap` crate reads the same bytes.
    let bytes = std::fs::read(&out).expect("read bag");
    let mut mcap_frames: Vec<Vec<u8>> = Vec::new();
    for item in mcap::MessageStream::new(&bytes).expect("mcap stream") {
        let m = item.expect("mcap message");
        if m.channel.topic == topic {
            mcap_frames.push(m.data.into_owned());
        }
    }
    assert_eq!(
        mcap_frames, expected,
        "the upstream mcap reader recovers the identical gap-frame bytes"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}
