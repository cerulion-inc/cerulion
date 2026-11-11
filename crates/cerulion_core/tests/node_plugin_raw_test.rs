// SPDX-License-Identifier: AGPL-3.0-only
//! Actual downstream delivery through both local raw SDK backends.
//! Run this binary alone with --test-threads=1 under the build desk lock.
use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::{DylibNodeEntry, NodeContext, ShutdownSignal};
use cerulion_core::graph::node_host_abi::{
    register_context, remove_context, with_context, HostContext, HOST_API,
};
use cerulion_core::graph::node_plugin_abi::{HostClient, NodeContext as PluginContext, RawIngress};
use cerulion_core::message::ShmMessage;
use cerulion_core::node_abi::ContextHandle;
use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::{FullQueueSend, TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use std::collections::HashMap;
use std::sync::Arc;

#[test]
fn native_and_plugin_raw_sdk_preserve_oracle_bytes_and_recover_after_bad_frames() {
    let clock = Arc::new(VirtualClock::new());
    clock.set(777);
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "node_plugin_raw_oracle".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        iceoryx_test_config(),
    )
    .unwrap();
    let env = Arc::new(HashMap::new());
    let mut native = NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        clock,
        ShutdownSignal::new(),
        Arc::clone(&env),
    );
    native.set_transport(Arc::clone(&manager));
    let host = HostContext::new(native, Vec::new(), env, 4)
        .unwrap()
        .with_raw_publisher_budget(2)
        .unwrap();
    let context = ContextHandle {
        value: register_context(host).unwrap(),
    };
    // SAFETY (U05): actual immutable process HOST_API and registered host
    // context remain live until every SDK publisher/client/read is dropped.
    let client = unsafe { HostClient::connect(&HOST_API, context, 0) }.unwrap();
    let info = DylibNodeEntry::parse_info_json_labeled(
        r#"{"inputs":[],"outputs":[],"policy":{"period_ms":1}}"#,
        "raw SDK",
    )
    .unwrap();
    let plugin = PluginContext::new(HostClient::share(&client), &info).unwrap();
    let native_factory = RawIngress::native(Arc::clone(&manager));
    let plugin_factory = plugin.raw_ingress().unwrap();
    for (factory, topic) in [
        (&native_factory, "abi/raw/sdk/native"),
        (&plugin_factory, "abi/raw/sdk/plugin"),
    ] {
        let mut publisher = factory
            .create_publisher(
                topic,
                Vector3::SCHEMA_HASH,
                MaxSliceLen::try_new(64).unwrap(),
                FullQueueSend::Discard,
            )
            .unwrap();
        let mut receiver = manager.create_subscriber(topic).unwrap();
        let mut frame = [0u8; 64];
        let mut header = WireHeader::new(Vector3::SCHEMA_HASH, 41, 123);
        header.total_size = 64;
        header.write_to_buf(&mut frame);
        let payload = [7.0f64, -2.25, 0.5];
        for (index, value) in payload.iter().enumerate() {
            frame[40 + index * 8..48 + index * 8].copy_from_slice(&value.to_le_bytes());
        }
        assert!(publisher.publish_raw(&frame[..39]).is_err());
        assert!(receiver
            .view_raw_expecting(Vector3::SCHEMA_HASH)
            .unwrap()
            .is_none());
        for _ in 0..2 {
            assert_eq!(publisher.publish_raw(&frame).unwrap(), 1);
            let received = receiver
                .view_raw_expecting(Vector3::SCHEMA_HASH)
                .unwrap()
                .expect("SHM downstream frame");
            assert_eq!(&received[..], &frame);
            assert_eq!(received.header().sequence, 41);
            assert_eq!(received.header().timestamp_ns, 123);
            assert_eq!(&received[40..48], &7.0f64.to_le_bytes());
            assert_eq!(&received[48..56], &(-2.25f64).to_le_bytes());
            assert_eq!(&received[56..64], &0.5f64.to_le_bytes());
        }
    }
    assert!(!client.has_failure());
    assert_eq!(
        with_context(context.value, |host| Ok(host.outstanding_leases())).unwrap(),
        0
    );
    drop(plugin_factory);
    drop(native_factory);
    drop(plugin);
    drop(client);
    remove_context(context.value).unwrap();
}
