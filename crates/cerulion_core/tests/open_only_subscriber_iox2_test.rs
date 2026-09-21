// SPDX-License-Identifier: AGPL-3.0-only
//! `TransportManager::create_subscriber_open_only` (the TOCTOU close)
//! — the introspection subscriber path that STRUCTURALLY cannot create
//! services (data service opened via the builder's `.open()`, never
//! `open_or_create`). `cerulion topic echo`/`hz`/`info` route through it;
//! the CLI's list-then-check existence guard is demoted to error-text UX.
//!
//! Parallel-safe: each test generates its own isolated iceoryx2 config.

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;

fn manager_with(name: &str, buffer: usize, ix: iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: buffer,
            network: None,
        },
        ix,
    )
    .expect("init isolated transport manager")
}

/// Count the iceoryx2 services visible under an isolated config.
fn service_count(ix: &iceoryx2::config::Config) -> usize {
    let mut n = 0;
    <iceoryx2::service::ipc::Service as iceoryx2::service::Service>::list(ix, |_| {
        n += 1;
        iceoryx2::prelude::CallbackProgression::Continue
    })
    .expect("service enumeration");
    n
}

#[test]
fn open_only_on_missing_topic_errors_and_creates_nothing() {
    // THE TOCTOU pin: a typo'd topic must error AND leave zero services
    // behind. (Mutation oracle: reverting `.open()` to `.open_or_create()`
    // makes the call SUCCEED and mints both halves of a phantom topic —
    // killed by both asserts.)
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = manager_with("open_only_missing", 8, ix.clone());
    let msg = match mgr.create_subscriber_open_only("/ghost/cam/image") {
        Ok(_) => panic!("open-only on a missing topic must NOT succeed"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("does not exist") && msg.contains("open-only"),
        "the error must name the missing topic + the open-only contract: {msg}"
    );
    assert_eq!(
        service_count(&ix),
        0,
        "a failed open-only attach must leave ZERO services behind"
    );
}

#[test]
fn open_only_attaches_and_receives_on_live_topic() {
    // Happy path: data published on a live topic reaches the open-only
    // subscriber end-to-end.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = manager_with("open_only_live", 8, ix);
    let mut publisher = mgr
        .create_publisher("/live/cam/image", MaxSliceLen::const_new(256), 0)
        .expect("publisher creates the topic");
    let subscriber = mgr
        .create_subscriber_open_only("/live/cam/image")
        .expect("open-only subscriber attaches to the live topic");

    let mut proxy = publisher
        .loan_proxy::<native_ros2_messages::geometry_msgs::Vector3>()
        .expect("loan");
    proxy.x = 42.0;
    drop(proxy); // publish

    let mut seen = 0u32;
    let received = subscriber
        .try_receive(|_msg| {
            seen += 1;
        })
        .expect("receive");
    assert!(
        received >= 1 && seen >= 1,
        "the open-only subscriber must receive the published frame \
         (received={received}, seen={seen})"
    );
}

#[test]
fn open_only_attaches_where_default_opener_is_rejected() {
    // The no-requirements pin: a foreign service with a buffer ceiling (4)
    // BELOW this manager's default (16) rejects the default opener
    // (`create_subscriber` arms ceiling ≥ 16 as an open requirement) but
    // must admit the open-only subscriber, which arms nothing and takes
    // the service as it is. (Mutation oracle: arming the ceiling — or any
    // requirement — in the open-only path fails this attach.)
    let ix = cerulion_core::testing::iceoryx_test_config();
    let shallow_creator = manager_with("shallow_creator", 4, ix.clone());
    let _pubr = shallow_creator
        .create_publisher("/foreign/shallow", MaxSliceLen::const_new(256), 0)
        .expect("foreign service created at ceiling 4");

    let deep_tool = manager_with("deep_tool", 16, ix);
    if deep_tool.create_subscriber("/foreign/shallow").is_ok() {
        panic!("the default opener requires ceiling >= 16 and must be rejected");
    }
    deep_tool
        .create_subscriber_open_only("/foreign/shallow")
        .expect("open-only arms no requirements and must attach to the ceiling-4 service");
}

#[test]
fn open_only_on_foreign_typed_service_surfaces_type_skew_diagnostic() {
    // A foreign iceoryx2 service with a non-`[u8]`
    // payload squatting on the data name is exactly what an introspection
    // tool trips over. The open-only `.open()` must carry the same
    // type-skew remedy the graph openers do (shared
    // `publish_subscribe_open_env_hint`) — not the bare variant name.
    // (Mutation oracle: dropping the env-hint fallback from the open-only
    // map_err leaves only the bare `IncompatibleTypes` Display and this
    // assert fails.)
    let ix = cerulion_core::testing::iceoryx_test_config();
    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let data_name: iceoryx2::service::service_name::ServiceName =
        "/foreign/typed/data".try_into().expect("service name");
    let _foreign_typed = raw_node
        .service_builder(&data_name)
        .publish_subscribe::<u64>()
        .create()
        .expect("foreign u64-typed data service");

    let mgr = manager_with("typed_squatter", 8, ix);
    let msg = match mgr.create_subscriber_open_only("/foreign/typed") {
        Ok(_) => panic!("a foreign-typed service must NOT open as a Cerulion topic"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("different payload type"),
        "the rejection must carry the type-skew remedy, not the bare variant: {msg}"
    );
}

#[test]
fn open_only_rejects_bad_names_without_minting_services() {
    // Extend the TOCTOU guarantee to the bad-name
    // paths, not just the missing-topic path — an empty topic dies at
    // `validate_topic_name`, a malformed shape dies at `ServiceName`
    // conversion or the open itself, and in EVERY case zero services may
    // be left behind.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = manager_with("bad_names", 8, ix.clone());
    let msg = match mgr.create_subscriber_open_only("") {
        Ok(_) => panic!("an empty topic name must be rejected"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("cannot be empty"),
        "the empty-name rejection must come from validate_topic_name: {msg}"
    );
    // iceoryx2's ServiceName accepts the malformed shape verbatim
    // (no normalization), so the rejection arrives as
    // the open-only DoesNotExist arm — pin the reason, not just any Err
    // (an is_ok-only check would survive the error changing for
    // an unrelated cause).
    let msg = match mgr.create_subscriber_open_only("//bad//shape") {
        Ok(_) => panic!("a malformed topic name must not open anything"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("does not exist") && msg.contains("open-only"),
        "the malformed name must die at the open-only DoesNotExist arm: {msg}"
    );
    assert_eq!(
        service_count(&ix),
        0,
        "bad-name attempts must leave ZERO services behind"
    );
}

#[test]
fn open_only_heals_missing_event_half_without_minting_topics() {
    // The event half stays `open_or_create` BY DESIGN: a foreign
    // data-only service (raw iceoryx2 publisher, no Cerulion event
    // service) is a real topic per `topic list` (the data service is the
    // existence marker), so echo must attach — creating the missing event
    // half cannot mint a phantom topic because the data `.open()` gates
    // first. (Mutation oracle: flipping the event half to strict `.open()`
    // fails this attach.)
    let ix = cerulion_core::testing::iceoryx_test_config();
    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let data_name: iceoryx2::service::service_name::ServiceName =
        "/foreign/raw/data".try_into().expect("service name");
    let _foreign_data = raw_node
        .service_builder(&data_name)
        .publish_subscribe::<[u8]>()
        .create()
        .expect("foreign data-only service");

    let mgr = manager_with("event_heal", 8, ix.clone());
    // Bind the subscriber: dropping it would let iceoryx2 reclaim the
    // just-created event service (last-handle cleanup) before the count.
    let _sub = mgr
        .create_subscriber_open_only("/foreign/raw")
        .expect("open-only attaches to a data-only foreign topic (event half created)");
    // Exactly the data + event pair exists — no extra services minted.
    assert_eq!(
        service_count(&ix),
        2,
        "only the foreign data service + the healed event half may exist"
    );
}
