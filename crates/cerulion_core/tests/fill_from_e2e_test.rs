// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end fill_from through the iceoryx2 publisher
//! and subscriber path.
//!
//! Unlike the codegen test (`fill_from_codegen_test.rs`) which
//! exercises the `<Name>Shm::fill_from_<f>` method directly on a raw
//! byte buffer, this file goes through the real publisher → iceoryx2
//! → subscriber wire so we know the trait + codegen + transport
//! together actually deliver bytes the way users will see.
//!
//! # Why a separate test file
//!
//! These tests require iceoryx2 transport (real SHM segments), so they
//! must run serial (`--test-threads=1`). The codegen tests are
//! pure direct-API exercises and run parallel. Keeping them in
//! different files lets `cargo test --workspace` parallelize freely.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test fill_from_e2e_test -- --test-threads=1
//! ```

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::fill_from::SliceSource;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::TransportError;
use native_ros2_messages::sensor_msgs::Image;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

/// Per-process unique topic suffix — every test run gets a fresh topic
/// so independent tests don't share iceoryx2 state.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(prefix: &str) -> String {
    let n = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/{prefix}/{}/{n}", std::process::id())
}

/// Slot budget large enough for a tiny Image (3 fixed fields + 3
/// variable fields with a few-KiB data payload).
fn image_max_slice() -> MaxSliceLen {
    MaxSliceLen::const_new(
        (cerulion_core::wire::WireHeader::SIZE + Image::WIRE_FIXED_SIZE + 8 * 3 + 4096) as u32,
    )
}

// ============================================================
// Happy path: fill_from writes round-trip through pub/sub
// ============================================================

#[test]
fn fill_from_publish_subscribe_round_trip() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("happy");

    let mut publisher = mgr
        .create_publisher_simple(&topic, image_max_slice())
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Publish: fixed fields via direct write; data via fill_from.
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.height = 480;
        proxy.width = 640;
        proxy.is_bigendian = 0;
        proxy.step = 1920;
        proxy.set_header_bytes(&[]).expect("");
        proxy.set_encoding("rgb8").expect("");
        proxy
            .fill_from_data(|buf: &mut [u8]| {
                buf[..5].copy_from_slice(b"hello");
                Ok(5)
            })
            .expect("fill_from_data");
    }

    std::thread::sleep(Duration::from_millis(50));

    let observed = subscriber
        .try_view::<Image, _>(|view| {
            (
                view.height,
                view.width,
                view.is_bigendian,
                view.step,
                view.data().to_vec(),
                view.encoding().map(str::to_string).unwrap_or_default(),
            )
        })
        .expect("try_view")
        .expect("subscriber should see the published Image");

    assert_eq!(observed.0, 480, "height round-trip");
    assert_eq!(observed.1, 640, "width round-trip");
    assert_eq!(observed.2, 0, "is_bigendian round-trip");
    assert_eq!(observed.3, 1920, "step round-trip");
    assert_eq!(observed.4, b"hello", "data round-trip");
    assert_eq!(observed.5, "rgb8", "encoding round-trip");
}

// ============================================================
// SliceSource path through real transport
// ============================================================

#[test]
fn fill_from_slice_source_round_trip() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("slice");

    let mut publisher = mgr
        .create_publisher_simple(&topic, image_max_slice())
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    let payload = b"world";

    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.height = 1;
        proxy.width = 1;
        proxy.is_bigendian = 0;
        proxy.step = 1;
        proxy.set_header_bytes(&[]).expect("");
        proxy.set_encoding("g").expect("");
        proxy
            .fill_from_data(SliceSource::new(payload))
            .expect("fill_from with SliceSource");
    }

    std::thread::sleep(Duration::from_millis(50));

    let observed = subscriber
        .try_view::<Image, _>(|view| view.data().to_vec())
        .expect("try_view")
        .expect("subscriber should see frame");

    assert_eq!(observed, payload, "SliceSource bytes round-trip");
}

// ============================================================
// Producer Err: NO frame is published (publish gate refuses)
// ============================================================

#[test]
fn fill_from_producer_err_does_not_publish() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("err_no_publish");

    let mut publisher = mgr
        .create_publisher_simple(&topic, image_max_slice())
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.height = 7;
        proxy.width = 7;
        proxy.is_bigendian = 0;
        proxy.step = 7;
        proxy.set_header_bytes(&[]).expect("");
        proxy.set_encoding("g").expect("");

        let err = proxy
            .fill_from_data(|_buf: &mut [u8]| -> Result<usize, TransportError> {
                Err(TransportError::NodeError {
                    node_id: "camera".into(),
                    reason: "device disconnected".into(),
                })
            })
            .expect_err("producer Err must propagate");
        assert!(matches!(err, TransportError::NodeError { .. }));

        // The proxy is now dropped without `data` marked written.
        // OutputProxy::Drop must NOT publish a partial frame.
    }

    std::thread::sleep(Duration::from_millis(50));

    // Subscriber sees NOTHING — the publish gate refused.
    let result = subscriber
        .try_view::<Image, _>(|view| view.height)
        .expect("try_view");
    assert!(
        result.is_none(),
        "no frame must be published when fill_from producer returns Err"
    );
}

// ============================================================
// Critical regression: fill_from(Err) after a
// successful set_<f> must not corrupt the published frame
// ============================================================
//
// The hazard: after `set_data(b"good")`
// set field_starts[data] + offset_entry[data] + mark_written[data]=true,
// a subsequent `fill_from_data(Err)` would overwrite field_starts +
// offset_entry during the loan-reservation phase but leave
// mark_written=true. OutputProxy::Drop would publish a frame whose
// offset_entry pointed at the failed loan's reservation region
// instead of the original "good" bytes — silent data corruption
// reaching the subscriber.
//
// Contract: fill_from defers ALL state mutations to the Ok arm.
// Err leaves writer state bit-for-bit unchanged. The published frame
// reflects the prior successful write.

#[test]
fn fill_from_err_after_set_data_publishes_original_not_corrupt() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("err_after_good");

    let mut publisher = mgr
        .create_publisher_simple(&topic, image_max_slice())
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
        proxy.height = 1;
        proxy.width = 9;
        proxy.is_bigendian = 0;
        proxy.step = 9;
        proxy.set_header_bytes(&[]).expect("");
        proxy.set_encoding("g").expect("");

        // First write: successful — locks in offset_entry[data].
        proxy.set_data(b"good_data").expect("first write");

        // Second call on the SAME field — failing. An eager mutation
        // would corrupt offset_entry[data]; it MUST
        // leave the prior good entry intact.
        let _ = proxy.fill_from_data(|_buf: &mut [u8]| -> Result<usize, TransportError> {
            Err(TransportError::NodeError {
                node_id: "camera".into(),
                reason: "second-call failure".into(),
            })
        });
        // Proxy drops — should publish "good_data", NOT corrupt bytes.
    }

    std::thread::sleep(Duration::from_millis(50));

    let observed = subscriber
        .try_view::<Image, _>(|view| view.data().to_vec())
        .expect("try_view")
        .expect("frame MUST publish (prior write was successful)");

    assert_eq!(
        observed, b"good_data",
        "publish MUST preserve the prior set_data bytes despite the later fill_from Err"
    );
}

// ============================================================
// Determinism (Principle #7): same producer sequence across 100
// publishes yields byte-identical SHM frames
// ============================================================

#[test]
fn fill_from_determinism_100_runs_byte_identical() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("determinism");

    let mut publisher = mgr
        .create_publisher_simple(&topic, image_max_slice())
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    let payload = b"determinism";
    let mut observed_payloads: Vec<Vec<u8>> = Vec::with_capacity(100);

    for _ in 0..100 {
        {
            let mut proxy = publisher.loan_proxy::<Image>().expect("loan_proxy");
            proxy.height = 1;
            proxy.width = 11;
            proxy.is_bigendian = 0;
            proxy.step = 11;
            proxy.set_header_bytes(&[]).expect("");
            proxy.set_encoding("rgb8").expect("");
            proxy
                .fill_from_data(SliceSource::new(payload))
                .expect("fill_from");
        }
        // Drain the subscriber on each tick so subsequent publishes
        // don't sit in the queue and back-pressure the publisher.
        std::thread::sleep(Duration::from_millis(2));
        if let Some(buf) = subscriber
            .try_view::<Image, _>(|view| view.data().to_vec())
            .expect("try_view")
        {
            observed_payloads.push(buf);
        }
    }

    assert!(
        !observed_payloads.is_empty(),
        "subscriber must observe at least one frame across 100 publishes"
    );
    // All observed frames must be byte-identical to the input. This
    // is the Principle #7 contract (Replay = Live): the SHM payload
    // for the same producer input is deterministic across runs.
    for (i, p) in observed_payloads.iter().enumerate() {
        assert_eq!(
            p, payload,
            "observed payload #{i} must be byte-identical to the deterministic input"
        );
    }
}
