// SPDX-License-Identifier: AGPL-3.0-only
//! An rmw_cerulion ADDRESSED frame (`WireHeader::is_rmw_addressed`: CDR for
//! particular buffer-aware rmw subscriptions, possibly naming device memory)
//! that reaches a recorded topic is refused and counted, never recorded. It is
//! published FIRST, the frame an attach-mode tap learns its channel schema
//! from, so the test also pins that the channel keeps the topic's own hash.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_bag::BagReader;
use cerulion_bagd::{run_bagd, BagdConfig, TapSpec};
use cerulion_core::testing::gap_frame::image_gap_frame;
use cerulion_core::wire::{WireHeader, RMW_ADDRESSED_FRAME_MARK};
use cerulion_core::ShmMessage;
use native_ros2_messages::sensor_msgs::Image;

use common::*;

/// An addressed frame as rmw_cerulion builds one: its own schema hash, no
/// offset table, the mark, then an opaque payload.
fn addressed_frame() -> Vec<u8> {
    let payload = [0xabu8; 96];
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    let header = WireHeader {
        schema_hash: 0x0123_4567_89ab_cdef,
        total_size: frame.len() as u32,
        offset_table_offset: RMW_ADDRESSED_FRAME_MARK,
        offset_table_count: 0,
        sequence: 0,
        timestamp_ns: 500,
    };
    assert!(header.is_rmw_addressed());
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

#[test]
fn an_addressed_frame_is_refused_counted_and_never_recorded() {
    let mgr = make_manager(32);
    let topic = unique_topic("addr");
    let out = unique_out("addr");
    let ready = unique_ready_file("addr");
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

    publisher
        .publish_raw(&addressed_frame())
        .expect("publish addressed frame");
    let natives: Vec<Vec<u8>> = (0..2u32)
        .map(|i| image_gap_frame(Image::SCHEMA_HASH, i, 1_000 + i as u64))
        .collect();
    for frame in &natives {
        publisher.publish_raw(frame).expect("publish native frame");
    }
    settle();
    settle();
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");
    assert_eq!(
        summary.addressed_refused, 1,
        "the addressed frame is counted"
    );
    assert_eq!(summary.messages, 2, "only the native frames are recorded");

    let reader = BagReader::open(&out).expect("open bag");
    let (msgs, _) = reader.recover_messages().expect("recover_messages");
    let recorded: Vec<&cerulion_bag::BagMessage> =
        msgs.iter().filter(|m| m.topic == topic).collect();
    assert_eq!(recorded.len(), 2);
    for (m, expected) in recorded.iter().zip(&natives) {
        assert_eq!(&m.data, expected, "native frames recorded byte-identical");
    }
    // The attach-mode channel learned the topic's own hash, not the addressed
    // frame's, although the addressed frame arrived first.
    let chan = reader
        .channels()
        .expect("channels")
        .into_iter()
        .find(|c| c.topic == topic)
        .expect("a channel for the topic");
    let d = chan
        .descriptor
        .expect("a cerulion channel carries a descriptor");
    assert_eq!(
        d.schema_hash,
        Image::SCHEMA_HASH,
        "the channel keeps the topic's hash"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}
