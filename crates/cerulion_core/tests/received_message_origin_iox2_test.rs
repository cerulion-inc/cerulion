// SPDX-License-Identifier: AGPL-3.0-only
//! `ReceivedMessage::origin` over real iceoryx2 (per-test SHM root via
//! `TransportManager::init_for_test`, so parallel-safe).
//!
//! Two publishers write one topic. Every frame a reader receives must name
//! the publisher that sent it, through both callback read paths
//! (`wait_for_message`, which drains, and `try_receive_one`). The oracle is
//! each publisher's own `publisher_id()`, read from the sending side, never
//! from the receive path under test.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::CerulionPublisher;
use cerulion_core::ReceivedMessage;

const TOPIC: &str = "/origin_test/out";
const BODY: usize = 8;

fn manager(name: &str) -> Arc<TransportManager> {
    let config = TransportConfig {
        node_name: name.into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 16,
        network: None,
    };
    TransportManager::init_for_test(config, cerulion_core::testing::iceoryx_test_config())
        .expect("transport")
}

/// One well-formed frame stamped `ts`, its body all `tag`.
fn publish(publisher: &mut CerulionPublisher, seq: u32, ts: u64, tag: u8) {
    let mut header = WireHeader::new(0xBEEF, seq, ts);
    header.total_size = (WireHeader::SIZE + BODY) as u32;
    let mut frame = vec![tag; WireHeader::SIZE + BODY];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    publisher.publish_raw(&frame).expect("raw publish");
}

#[test]
fn every_frame_names_the_publisher_that_sent_it() {
    let mgr = manager("origin_two_writers");
    let mut a = mgr
        .create_publisher(TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("publisher a");
    let mut b = mgr
        .create_publisher(TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("publisher b (the default service takes two)");
    let subscriber = mgr.create_subscriber(TOPIC).expect("subscriber");
    let (id_a, id_b) = (a.publisher_id(), b.publisher_id());
    assert_ne!(id_a, id_b, "precondition: two ports, two identities");

    // The drain path: tag each body with its sender, and read the pairs back.
    for seq in 0..3 {
        publish(&mut a, seq, 10 + u64::from(seq), 0xA);
        publish(&mut b, seq, 900 + u64::from(seq), 0xB);
    }
    let mut seen: Vec<(u8, u128)> = Vec::new();
    while seen.len() < 6 {
        let got = subscriber
            .wait_for_message(Duration::from_secs(5), |msg| {
                seen.push((msg.payload()[0], msg.origin()));
            })
            .expect("wait");
        assert!(got > 0, "six frames were sent; got {seen:?}");
    }
    assert_eq!(seen.iter().filter(|(t, _)| *t == 0xA).count(), 3);
    for (tag, origin) in &seen {
        let want = if *tag == 0xA { id_a } else { id_b };
        assert_eq!(*origin, want, "frame tagged {tag:#x} names its sender");
    }

    // The one-frame path.
    publish(&mut b, 3, 903, 0xB);
    let mut one: Option<u128> = None;
    let delivered = subscriber
        .try_receive_one(|msg| one = Some(msg.origin()))
        .expect("receive one");
    assert!(delivered, "the frame was sent before the read");
    assert_eq!(one, Some(id_b));
}

#[test]
fn a_message_built_by_hand_has_no_publisher_identity() {
    let header = WireHeader::new(0xBEEF, 0, 1);
    assert_eq!(ReceivedMessage::new(header, &[1, 2, 3]).origin(), 0);
}
