// SPDX-License-Identifier: AGPL-3.0-only
//! Variable-schema spill round-trip across the
//! cdylib FFI boundary.
//!
//! Loads `test_node_macro_overflow_cdylib` via `DylibNodeEntry`, wires
//! it into an iceoryx2 `TestTransport` publisher + subscriber, and verifies:
//!
//! 1. Warm-up ticks (1 KiB payload each, ×16) populate the adaptive
//!    sliding window so the next loan is small (~1.5 KiB).
//! 2. The 17th tick spikes to 12 KiB. The cdylib's `tick()` calls
//!    `proxy.set_data(&12_KiB)` which:
//!    - triggers `ensure_capacity_for` (1.5 KiB loan, 12 KiB requested)
//!    - spills into the heap `Box<[u64]>` buffer
//!    - `OutputProxy::Drop` re-loans a fresh sample sized to fit and
//!      memcpies the spill in.
//! 3. Subscriber receives the 12 KiB payload byte-for-byte.
//!
//! Why this test exists separately from the in-process e2e suite:
//! the spill mechanism uses the codegen-emitted `<Name>Shm` struct
//! (`Arc<str> topic`, `Option<Box<[u64]>> overflow`, `MaxPayloadCapacity
//! max_capacity` — the fields the overflow redirect needs).
//! That struct exists in BOTH the host (`cerulion_core` linked into the
//! test binary) AND the cdylib (`test_node_macro_overflow_cdylib`).
//! A struct-layout drift between the two compile units would
//! manifest as SIGSEGV when `OutputProxy::Drop` walks the writer
//! struct's fields (the cfg-gated-field SIGSEGV pattern this test
//! explicitly defends against). This test exercises that exact
//! Drop-time walk through a fresh cdylib load.
//!
//! Must run with `--test-threads=1` (uses the iceoryx2 transport via
//! `TestTransport`; each instance is rooted at an isolated SHM prefix).

use cerulion_core::graph::node::{
    AnyPublisher, AnySubscriber, DylibNodeEntry, NodeContext, NodeEntry,
};
use cerulion_core::testing::TestTransport;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::sensor_msgs::Image;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::SystemTime;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/cdylib_overflow/{base}/{nanos}/{id}")
}

/// Find the overflow test cdylib in the target directory.
fn find_overflow_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_overflow_cdylib")
}

/// Build a NodeContext with a single `image_out` publisher (variable
/// schema, 16 KiB ceiling) and a subscriber channel the test can read
/// from. The cdylib's tick writes through the publisher; the test reads
/// the resulting frame off the subscriber.
fn make_overflow_ctx() -> (TestTransport, NodeContext, CerulionSubscriber) {
    let topic = unique_topic("image_out");
    let tt = TestTransport::with_buffer_size(64);

    let image_pub = tt.publisher(
        &topic,
        // 16 KiB ceiling — fits the 12 KiB spike.
        MaxSliceLen::const_new(16 * 1024),
        // No history — test only reads the latest frame.
        0,
    );
    let image_sub = tt.subscriber(&topic);

    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("image_out".to_string(), AnyPublisher::Ipc(image_pub));
    let subscribers: IndexMap<String, AnySubscriber> = IndexMap::new();

    let ctx = NodeContext::for_tests(publishers, subscribers);
    (tt, ctx, image_sub)
}

#[test]
fn cdylib_variable_schema_spill_round_trip() {
    let path = find_overflow_cdylib();
    let mut node = DylibNodeEntry::load(&path).expect("load overflow cdylib");

    // `tt` (TestTransport) must outlive the publisher/subscriber/NodeContext
    // it created — bind it for the whole test fn so the underlying iceoryx2
    // TransportManager stays alive (the lifetime rule).
    let (_tt, ctx, mut image_sub) = make_overflow_ctx();
    node.init(ctx).expect("init");

    // Warm-up ticks: the cdylib's `tick()` writes 1 KiB on ticks 0..16.
    // This populates the sliding window so the next loan converges
    // downward toward ~1.5 KiB.
    for _ in 0..16 {
        node.tick().expect("warm-up tick");
        // Drain the subscriber so we don't fill the channel.
        let _ = image_sub.try_view::<Image, _>(|_| ()).expect("drain");
    }

    // Spike tick: tick 16+ writes 12 KiB — exceeds the warm loan
    // (~1.5 KiB) but fits in the 16 KiB ceiling. Spill fires inside
    // the cdylib's `tick()`; `OutputProxy::Drop` re-loans + memcpies.
    node.tick().expect("spike tick");

    // Subscriber sees the full 12 KiB payload, bit-for-bit.
    let observed = image_sub
        .try_view::<Image, _>(|view| (view.height, view.data().to_vec()))
        .expect("try_view")
        .expect("spike frame present");

    assert_eq!(
        observed.0, 16,
        "height carries the tick index (16 = first spike tick)"
    );
    assert_eq!(
        observed.1.len(),
        12 * 1024,
        "subscriber must see the full 12 KiB payload \
         (not truncated to the warm loan size) — cdylib spill round-trip"
    );
    // The cdylib fills the payload with `(tick & 0xff) as u8`.
    let expected_byte = (16u64 & 0xff) as u8;
    assert!(
        observed.1.iter().all(|&b| b == expected_byte),
        "every byte must match the cdylib's spike-fill pattern (0x{expected_byte:02x})"
    );

    node.shutdown().expect("shutdown");
}
