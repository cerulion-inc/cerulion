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
        clock.clone(),
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
    with_context(context.value, |host| host.begin_node_call()).unwrap();
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
        header.offset_table_offset = 64;
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
        header.offset_table_offset = 0;
        header.write_to_buf(&mut frame);
        let malformed = publisher.publish_raw(&frame).unwrap_err();
        assert_eq!(
            cerulion_core::graph::node_plugin_abi::diagnostic_reason(&malformed),
            "raw frame has an invalid offset table"
        );
        assert!(receiver
            .view_raw_expecting(Vector3::SCHEMA_HASH)
            .unwrap()
            .is_none());
        header.offset_table_offset = 64;
        let scripted = factory.clock_arc();
        for timestamp in [123_u64, 456] {
            clock.set(timestamp);
            assert_eq!(scripted.now_ns(), timestamp);
            header.timestamp_ns = scripted.now_ns();
            header.write_to_buf(&mut frame);
            assert_eq!(publisher.publish_raw(&frame).unwrap(), 1);
            let received = receiver
                .view_raw_expecting(Vector3::SCHEMA_HASH)
                .unwrap()
                .expect("SHM downstream frame");
            assert_eq!(&received[..], &frame);
            assert_eq!(received.header().sequence, 41);
            assert_eq!(received.header().timestamp_ns, timestamp);
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
    with_context(context.value, |host| host.end_node_call()).unwrap();
    remove_context(context.value).unwrap();
}

#[test]
fn helper_clock_and_raw_publish_deliver_while_node_tick_gate_is_held() {
    use cerulion_core::graph::node::{AnyPublisher, NodeContext};
    use cerulion_core::graph::node_host_abi::HostPortSpec;
    use cerulion_core::graph::node_plugin_abi::NodeRegistry;
    use cerulion_core::transport::plugin_lease::{LeaseDirection, PluginWireLayout};
    use std::sync::Barrier;

    let clock = Arc::new(VirtualClock::new());
    clock.set(777);
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "node_plugin_clock_concurrent".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        iceoryx_test_config(),
    )
    .unwrap();
    let mut publishers = IndexMap::new();
    publishers.insert(
        "out".into(),
        AnyPublisher::Ipc(
            manager
                .create_publisher("abi/clock/tick", MaxSliceLen::try_new(64).unwrap(), 1)
                .unwrap(),
        ),
    );
    let mut tick_receiver = manager.create_subscriber("abi/clock/tick").unwrap();
    let mut raw_receiver = manager.create_subscriber("abi/clock/helper").unwrap();
    let env = Arc::new(HashMap::new());
    let mut native = NodeContext::with_runtime_env(
        publishers,
        IndexMap::new(),
        clock,
        ShutdownSignal::new(),
        env.clone(),
    );
    native.set_transport(manager.clone());
    let specs = vec![HostPortSpec {
        name: "out".into(),
        direction: LeaseDirection::Write,
        layout: PluginWireLayout {
            schema_hash: Vector3::SCHEMA_HASH,
            fixed_size: 24,
            variable_fields: 0,
            alignment: 8,
        },
        read_mode: 0,
        accumulate_cap: None,
    }];
    let context = ContextHandle {
        value: register_context(
            HostContext::new(native, specs, env, 4)
                .unwrap()
                .with_raw_publisher_budget(1)
                .unwrap(),
        )
        .unwrap(),
    };
    // SAFETY (U05): real immutable host table and registered context outlive both threads and all loans.
    let client = unsafe { HostClient::connect(&HOST_API, context, 0) }.unwrap();
    let info = DylibNodeEntry::parse_info_json_labeled(&format!(
        r#"{{"inputs":[],"outputs":[{{"name":"out","schema_hash":{},"max_slice_len_default":64}}],"policy":{{"period_ms":1}}}}"#,
        Vector3::SCHEMA_HASH), "clock concurrent").unwrap();
    with_context(context.value, |host| host.begin_node_call()).unwrap();
    let plugin = PluginContext::new(HostClient::share(&client), &info).unwrap();
    let factory = plugin.raw_ingress().unwrap();
    let mut raw_publisher = factory
        .create_publisher(
            "abi/clock/helper",
            Vector3::SCHEMA_HASH,
            MaxSliceLen::try_new(64).unwrap(),
            FullQueueSend::Discard,
        )
        .unwrap();
    let scripted = factory.clock_arc();
    let barrier = Arc::new(Barrier::new(2));
    let helper_barrier = barrier.clone();
    let helper = std::thread::spawn(move || {
        helper_barrier.wait();
        for sequence in 0..100 {
            let timestamp = scripted.now_ns();
            assert_eq!(timestamp, 777);
            let mut frame = [0u8; 64];
            let mut header = WireHeader::new(Vector3::SCHEMA_HASH, sequence, timestamp);
            header.total_size = 64;
            header.offset_table_offset = 64;
            header.write_to_buf(&mut frame);
            frame[40..48].copy_from_slice(&7.0f64.to_le_bytes());
            frame[48..56].copy_from_slice(&(-2.25f64).to_le_bytes());
            frame[56..64].copy_from_slice(&0.5f64.to_le_bytes());
            assert_eq!(raw_publisher.publish_raw(&frame).unwrap(), 1);
            let view = raw_receiver
                .view_raw_expecting(Vector3::SCHEMA_HASH)
                .unwrap()
                .unwrap();
            assert_eq!(&view[..], &frame);
            assert_eq!(view.header().timestamp_ns, 777);
        }
    });
    let registry = NodeRegistry::default();
    let handle = registry.insert(plugin).unwrap();
    with_context(context.value, |host| host.begin_fire()).unwrap();
    registry
        .call(handle, |node| {
            barrier.wait();
            for sequence in 0..100 {
                let mut loan = node
                    .publisher_mut("out")
                    .unwrap()
                    .loan_raw_frame(HostClient::request::<Vector3>(), 64)
                    .unwrap();
                let mut header = WireHeader::new(Vector3::SCHEMA_HASH, 0, 0);
                header.total_size = 64;
                header.offset_table_offset = 64;
                header.write_to_buf(loan.bytes_mut());
                loan.bytes_mut()[40..64].copy_from_slice(&[
                    0, 0, 0, 0, 0, 0, 28, 64, 0, 0, 0, 0, 0, 0, 2, 192, 0, 0, 0, 0, 0, 0, 224, 63,
                ]);
                loan.commit().unwrap();
                let view = tick_receiver
                    .view_raw_expecting(Vector3::SCHEMA_HASH)
                    .unwrap()
                    .unwrap();
                assert_eq!(view.header().sequence, sequence);
                assert_eq!(view.header().timestamp_ns, 777);
                assert_eq!(&view[40..48], &7.0f64.to_le_bytes());
                assert_eq!(&view[48..56], &(-2.25f64).to_le_bytes());
                assert_eq!(&view[56..64], &0.5f64.to_le_bytes());
            }
            // Keep the node's execution gate held for every helper callback.
            helper.join().unwrap();
            Ok(())
        })
        .unwrap();
    assert!(!client.has_failure());
    assert_eq!(
        with_context(context.value, |host| Ok(host.outstanding_leases())).unwrap(),
        0
    );
    registry.destroy(handle).unwrap();
    drop(factory);
    drop(client);
    with_context(context.value, |host| host.end_node_call()).unwrap();
    remove_context(context.value).unwrap();
}

#[test]
fn deferred_releases_free_retained_rows_once_and_match_native_delivery() {
    use cerulion_core::graph::node::{AnyPublisher, AnySubscriber};
    use cerulion_core::graph::node_host_abi::HostPortSpec;
    use cerulion_core::transport::plugin_lease::{LeaseDirection, PluginWireLayout};

    for _ in 0..2 {
        let clock = Arc::new(VirtualClock::new());
        clock.set(777);
        let manager = TransportManager::init_for_test(
            TransportConfig {
                node_name: "node_plugin_deferred_release".into(),
                clock: clock.clone(),
                subscriber_buffer_size: 16,
                network: None,
            },
            iceoryx_test_config(),
        )
        .unwrap();
        let maximum = MaxSliceLen::try_new(64).unwrap();
        let mut native_publisher = manager
            .create_publisher("abi/deferred/native", maximum, 1)
            .unwrap();
        let mut input_publisher = manager
            .create_publisher("abi/deferred/input", maximum, 1)
            .unwrap();
        let mut native_receiver = manager.create_subscriber("abi/deferred/native").unwrap();
        let mut publishers = IndexMap::new();
        publishers.insert(
            "out".into(),
            AnyPublisher::Ipc(
                manager
                    .create_publisher("abi/deferred/plugin", maximum, 1)
                    .unwrap(),
            ),
        );
        let mut receiver = manager.create_subscriber("abi/deferred/plugin").unwrap();
        let mut subscribers = IndexMap::new();
        subscribers.insert(
            "input".into(),
            AnySubscriber::Ipc(manager.create_subscriber("abi/deferred/input").unwrap()),
        );
        let env = Arc::new(HashMap::new());
        let native = NodeContext::with_runtime_env(
            publishers,
            subscribers,
            clock,
            ShutdownSignal::new(),
            env.clone(),
        );
        let specs = [
            ("input", LeaseDirection::Read),
            ("out", LeaseDirection::Write),
        ]
        .map(|(name, direction)| HostPortSpec {
            name: name.into(),
            direction,
            layout: PluginWireLayout {
                schema_hash: Vector3::SCHEMA_HASH,
                fixed_size: 24,
                variable_fields: 0,
                alignment: 8,
            },
            read_mode: 0,
            accumulate_cap: None,
        });
        let context = ContextHandle {
            value: register_context(HostContext::new(native, specs.into(), env, 9).unwrap())
                .unwrap(),
        };
        // SAFETY (U05): actual immutable HOST_API and this registered context
        // outlive the SDK, retained loans, deferred queue and all frame reads.
        let client = unsafe { HostClient::connect(&HOST_API, context, 0) }.unwrap();
        let info = DylibNodeEntry::parse_info_json_labeled(&format!(
        r#"{{"inputs":[{{"name":"input","schema_hash":{0}}}],"outputs":[{{"name":"out","schema_hash":{0},"max_slice_len_default":64}}],"policy":{{"period_ms":1}}}}"#,
        Vector3::SCHEMA_HASH), "deferred release").unwrap();
        with_context(context.value, |host| host.begin_node_call()).unwrap();
        let mut plugin = PluginContext::new(HostClient::share(&client), &info).unwrap();
        with_context(context.value, |host| host.begin_fire()).unwrap();
        let mut frame = [0u8; 64];
        let mut header = WireHeader::new(Vector3::SCHEMA_HASH, 0, 777);
        header.total_size = 64;
        header.offset_table_offset = 64;
        header.write_to_buf(&mut frame);
        frame[40..64].copy_from_slice(&[
            0, 0, 0, 0, 0, 0, 28, 64, 0, 0, 0, 0, 0, 0, 2, 192, 0, 0, 0, 0, 0, 0, 224, 63,
        ]);
        let native_retained = native_publisher.loan_raw_uninit(64).unwrap();
        let retained = plugin
            .publisher_mut("out")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Vector3>(), 64)
            .unwrap();
        assert!(plugin
            .publisher_mut("out")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Vector3>(), 64)
            .is_err());
        assert_eq!(client.deferred_release_count(), 0);
        with_context(context.value, |host| host.end_node_call()).unwrap();
        assert_eq!(
            with_context(context.value, |host| Ok(host.outstanding_leases())).unwrap(),
            1
        );
        assert!(receiver
            .view_raw_expecting(Vector3::SCHEMA_HASH)
            .unwrap()
            .is_none());
        drop(native_retained);
        drop(retained);
        assert_eq!(client.deferred_release_count(), 1);
        assert!(!client.has_failure());
        // An attempted flush outside admission keeps exactly the same row pinned.
        assert!(client.flush_deferred().is_err());
        assert_eq!(client.deferred_release_count(), 1);
        assert_eq!(
            with_context(context.value, |host| Ok(host.outstanding_leases())).unwrap(),
            1
        );
        with_context(context.value, |host| host.begin_node_call()).unwrap();
        with_context(context.value, |host| host.begin_fire()).unwrap();
        client.flush_deferred().unwrap();
        client.flush_deferred().unwrap();
        assert_eq!(client.deferred_release_count(), 0);
        assert_eq!(
            with_context(context.value, |host| Ok(host.outstanding_leases())).unwrap(),
            0
        );
        assert_eq!(
            with_context(context.value, |host| Ok(host.counters().discarded)).unwrap(),
            1
        );
        assert_eq!(native_publisher.publish_raw(&frame).unwrap(), 1);
        let mut loan = plugin
            .publisher_mut("out")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Vector3>(), 64)
            .unwrap();
        loan.bytes_mut().copy_from_slice(&frame);
        loan.commit().unwrap();
        let native_view = native_receiver
            .view_raw_expecting(Vector3::SCHEMA_HASH)
            .unwrap()
            .unwrap();
        let view = receiver
            .view_raw_expecting(Vector3::SCHEMA_HASH)
            .unwrap()
            .unwrap();
        assert_eq!(&native_view[..], &frame);
        assert_eq!(&view[..], &frame);
        assert_eq!(
            with_context(context.value, |host| Ok(host.counters().committed)).unwrap(),
            1
        );
        drop(view);
        drop(native_view);
        // A retained input export has the same admission and exact-once rule.
        assert_eq!(input_publisher.publish_raw(&frame).unwrap(), 1);
        let read = plugin
            .subscriber_mut("input")
            .unwrap()
            .read_raw_frame(Vector3::SCHEMA_HASH)
            .unwrap()
            .unwrap();
        assert_eq!(read.bytes(), &frame);
        with_context(context.value, |host| host.end_node_call()).unwrap();
        assert_eq!(read.bytes(), &frame);
        drop(read);
        assert_eq!(client.deferred_release_count(), 1);
        with_context(context.value, |host| host.begin_node_call()).unwrap();
        with_context(context.value, |host| host.begin_fire()).unwrap();
        client.flush_deferred().unwrap();
        client.flush_deferred().unwrap();
        assert_eq!(
            with_context(context.value, |host| Ok(host.counters().released)).unwrap(),
            1
        );
        assert_eq!(
            with_context(context.value, |host| Ok(host.outstanding_leases())).unwrap(),
            0
        );
        assert_eq!(client.deferred_release_count(), 0);
        assert!(!client.has_failure());
        drop(plugin);
        drop(client);
        with_context(context.value, |host| host.end_node_call()).unwrap();
        remove_context(context.value).unwrap();
    }
}

#[test]
fn native_and_stable_output_discard_logs_match_exactly_and_recover_after_delivery() {
    use cerulion_core::graph::node::AnyPublisher;
    use cerulion_core::graph::node_host_abi::HostPortSpec;
    use cerulion_core::graph::node_plugin_abi::LazyOutput;
    use cerulion_core::transport::plugin_lease::{LeaseDirection, PluginWireLayout};
    use native_ros2_messages::std_msgs::String as RosString;
    use std::io::Write;
    use std::sync::Mutex;

    #[derive(Clone)]
    struct LogWriter(Arc<Mutex<Vec<u8>>>);
    impl Write for LogWriter {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(bytes);
            Ok(bytes.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
    let clock = Arc::new(VirtualClock::new());
    clock.set(777);
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "node_plugin_discard_log_parity".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        iceoryx_test_config(),
    )
    .unwrap();
    // Both phases use the same actual publisher, topic and native latch.
    let mut publisher = manager
        .create_publisher("abi/discard/parity", MaxSliceLen::try_new(128).unwrap(), 1)
        .unwrap();
    let mut receiver = manager.create_subscriber("abi/discard/parity").unwrap();
    let native_log = Arc::new(Mutex::new(Vec::new()));
    let writer = LogWriter(native_log.clone());
    let subscriber = tracing_subscriber::fmt()
        .without_time()
        .with_ansi(false)
        .with_target(false)
        .with_max_level(tracing::Level::DEBUG)
        .with_writer(move || writer.clone())
        .finish();
    tracing::subscriber::with_default(subscriber, || {
        for complete in [false, false, true] {
            let mut output = publisher.loan_proxy::<RosString>().unwrap();
            if complete {
                output.set_data("ok").unwrap();
            }
            drop(output);
            if !complete {
                assert!(receiver
                    .view_raw_expecting(RosString::SCHEMA_HASH)
                    .unwrap()
                    .is_none());
            }
        }
    });
    let view = receiver
        .view_raw_expecting(RosString::SCHEMA_HASH)
        .unwrap()
        .unwrap();
    assert_eq!(view.header().sequence, 0);
    assert_eq!(view.header().timestamp_ns, 777);
    assert_eq!(&view[48..50], b"ok");
    drop(view);
    let mut publishers = IndexMap::new();
    publishers.insert("out".into(), AnyPublisher::Ipc(publisher));
    let env = Arc::new(HashMap::new());
    let native = NodeContext::with_runtime_env(
        publishers,
        IndexMap::new(),
        clock,
        ShutdownSignal::new(),
        env.clone(),
    );
    let request = HostClient::request::<RosString>();
    let specs = vec![HostPortSpec {
        name: "out".into(),
        direction: LeaseDirection::Write,
        layout: PluginWireLayout {
            schema_hash: RosString::SCHEMA_HASH,
            fixed_size: request.wire_fixed_size,
            variable_fields: request.variable_count,
            alignment: request.alignment,
        },
        read_mode: 0,
        accumulate_cap: None,
    }];
    let context = ContextHandle {
        value: register_context(HostContext::new(native, specs, env, 5).unwrap()).unwrap(),
    };
    // SAFETY (U05): immutable HOST_API and real context retain their native
    // publisher and every lease until this local adapter and client are dropped.
    let client = unsafe { HostClient::connect(&HOST_API, context, 0) }.unwrap();
    let info = DylibNodeEntry::parse_info_json_labeled(&format!(
        r#"{{"inputs":[],"outputs":[{{"name":"out","schema_hash":{},"max_slice_len_default":128}}],"policy":{{"period_ms":1}}}}"#,
        RosString::SCHEMA_HASH), "discard parity").unwrap();
    with_context(context.value, |host| host.begin_node_call()).unwrap();
    let mut plugin = PluginContext::new(HostClient::share(&client), &info).unwrap();
    let stable_log = Arc::new(Mutex::new(Vec::new()));
    let writer = LogWriter(stable_log.clone());
    let subscriber = tracing_subscriber::fmt()
        .without_time()
        .with_ansi(false)
        .with_target(false)
        .with_max_level(tracing::Level::DEBUG)
        .with_writer(move || writer.clone())
        .finish();
    tracing::subscriber::with_default(subscriber, || {
        for complete in [false, false, true] {
            with_context(context.value, |host| host.begin_fire()).unwrap();
            let mut output = LazyOutput::<RosString>::new(plugin.publisher_mut("out").unwrap());
            let proxy = output.__cer_loan().unwrap();
            if complete {
                proxy.set_data("ok").unwrap();
            }
            output.__cer_arm_if_loaned();
            drop(output);
            if !complete {
                assert!(receiver
                    .view_raw_expecting(RosString::SCHEMA_HASH)
                    .unwrap()
                    .is_none());
            }
        }
    });
    let view = receiver
        .view_raw_expecting(RosString::SCHEMA_HASH)
        .unwrap()
        .unwrap();
    assert_eq!(view.header().sequence, 1);
    assert_eq!(view.header().timestamp_ns, 777);
    assert_eq!(&view[48..50], b"ok");
    drop(view);
    let native_text = std::string::String::from_utf8(native_log.lock().unwrap().clone()).unwrap();
    let stable_text = std::string::String::from_utf8(stable_log.lock().unwrap().clone()).unwrap();
    for text in [&native_text, &stable_text] {
        assert_eq!(text.matches("OutputProxy dropped without writing all declared variable fields; skipping publish").count(), 1);
        assert_eq!(
            text.matches("OutputProxy discard suppressed: incomplete output")
                .count(),
            1
        );
        assert_eq!(text.matches("OutputProxy: output recovered").count(), 1);
        assert!(text.contains("suppressed_count=1"));
    }
    let native_lines: Vec<_> = native_text
        .lines()
        .filter(|line| line.contains("OutputProxy"))
        .collect();
    let stable_lines: Vec<_> = stable_text
        .lines()
        .filter(|line| line.contains("OutputProxy"))
        .collect();
    assert_eq!(native_lines, stable_lines);
    assert_eq!(
        with_context(context.value, |host| Ok(host.counters().discarded)).unwrap(),
        2
    );
    assert_eq!(
        with_context(context.value, |host| Ok(host.counters().committed)).unwrap(),
        1
    );
    assert!(!client.has_failure());
    drop(plugin);
    drop(client);
    with_context(context.value, |host| host.end_node_call()).unwrap();
    remove_context(context.value).unwrap();
}
